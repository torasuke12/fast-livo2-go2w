"""Check initialization-only A/B captures and parity with a pre-change short replay."""
import argparse
import csv
import hashlib
import json
from collections import Counter
from pathlib import Path

import numpy as np
import yaml


def rows(path):
    with path.open() as stream:
        result = list(csv.DictReader(stream))
    assert result and all(None not in r and all(v is not None for v in r.values()) for r in result), path
    return result


def columns(data, names):
    return np.array([[float(r[k]) for k in names] for r in data])


def flatten(data, prefix=''):
    result = {}
    for key, value in data.items():
        name = prefix + key
        if isinstance(value, dict):
            result.update(flatten(value, name + '.'))
        else:
            result[name] = value
    return result


def params(path):
    data = yaml.safe_load(path.read_text())
    node = [v for k, v in data.items() if k.lstrip('/') == 'laserMapping']
    assert len(node) == 1
    return flatten(node[0]['ros__parameters'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('single', type=Path)
    parser.add_argument('legacy', type=Path)
    parser.add_argument('--previous-logs', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--allow-shutdown-sigsegv', action='store_true',
                        help='Analyze completed rows after duration-reached shutdown SIGSEGV; preserve incomplete status')
    args = parser.parse_args()
    result = {'captures': {}}
    data = {}
    for mode, path in [('single_scan', args.single), ('legacy_voxel', args.legacy)]:
        summary = json.loads((path/'summary.json').read_text())
        if summary['capture_integrity'] != 'ok':
            node_exit = summary['exit_statuses']['node']
            assert args.allow_shutdown_sigsegv and summary['stop_reason'] == 'estimator_duration_reached'
            assert node_exit['returncode'] == -11 and len(summary['problems']) == 1
            assert summary['problems'][0].startswith('node abnormal exit:'), summary
        records = rows(path/'logs/temporal_window.csv')
        data[mode] = records
        init = rows(path/'logs/temporal_initialization.csv')
        assert init[-1]['status'] == 'ready' and init[-1]['mode'] == mode
        assert float(init[-1]['time']) == float(records[0]['time'])
        assert records[0]['status'] == 'initialized' and int(records[0]['matches']) == 0
        covariance = columns(records, [f'cov_{i}_{j}' for i in range(15) for j in range(15)]).reshape(-1, 15, 15)
        eigenvalues = np.linalg.eigvalsh(covariance)
        assert np.isfinite(eigenvalues).all() and eigenvalues.min() > 0
        time = columns(records, ['time'])[:, 0]
        assert np.all(np.diff(time) > 0) and time[-1] >= 60
        correction = columns(records, [f'correction_{i}' for i in range(15)])
        metrics = []
        for begin, end in [(3.499, 10), (10, 20), (20, 60)]:
            mask = (time >= begin) & (time < end)
            entry = {'begin': begin, 'end': end, 'scans': int(mask.sum())}
            for name, values in [('rotation_update_deg', np.rad2deg(np.linalg.norm(correction[:, :3], axis=1))),
                                 ('position_update_m', np.linalg.norm(correction[:, 3:6], axis=1))]:
                entry[name] = dict(zip(['median', 'p95', 'max'], np.quantile(values[mask], [.5, .95, 1]).tolist()))
            metrics.append(entry)
        metadata = json.loads((path/'capture.json').read_text())
        result['captures'][mode] = {
            'path': str(path), 'binary_sha256': metadata['binary_sha256'],
            'integrity': summary['capture_integrity'], 'problems': summary['problems'], 'scans': len(records),
            'initialization': init, 'first_update_time': float(records[1]['time']),
            'last_time': float(time[-1]), 'status_counts': dict(Counter(r['status'] for r in records)),
            'minimum_covariance_eigenvalue': float(eigenvalues.min()), 'windows': metrics,
            'sha256': {name: hashlib.sha256((path/name).read_bytes()).hexdigest() for name in
                       ['logs/temporal_window.csv', 'logs/temporal_states.csv', 'logs/temporal_initialization.csv', 'logs/imu.txt']},
        }
    assert result['captures']['single_scan']['binary_sha256'] == result['captures']['legacy_voxel']['binary_sha256']
    profiles = [params(p/'effective_params.yaml') for p in (args.single, args.legacy)]
    differences = {k: [profiles[0].get(k), profiles[1].get(k)] for k in set(profiles[0]) | set(profiles[1])
                   if profiles[0].get(k) != profiles[1].get(k) and k not in ('debug.log_dir', 'lio.diagnostics.output_dir')}
    assert differences == {'lio.temporal.initialization_mode': ['single_scan', 'legacy_voxel']}, differences
    result['effective_parameter_differences_excluding_output_paths'] = differences
    a, b = [np.loadtxt(p/'logs/imu.txt') for p in (args.single, args.legacy)]
    count = min(len(a), len(b))
    assert np.array_equal(a[:count], b[:count]), 'processed IMU prefix differs'
    result['identical_processed_imu_prefix_rows'] = count
    result['default_parity'] = {}
    for name in ['temporal_window.csv', 'temporal_states.csv']:
        before, after = rows(args.previous_logs/name), rows(args.single/'logs'/name)
        assert len(after) >= len(before)
        names = [k for k in before[0] if k not in ('solve_ms', 'status')]
        delta = np.abs(columns(before, names) - columns(after[:len(before)], names))
        statuses_equal = all(a.get('status') == b.get('status') for a, b in zip(before, after))
        result['default_parity'][name] = {'rows': len(before), 'numeric_columns': len(names),
            'max_absolute_difference': float(delta.max()), 'statuses_equal': statuses_equal,
            'previous_sha256': hashlib.sha256((args.previous_logs/name).read_bytes()).hexdigest()}
        assert statuses_equal and delta.max() < 1e-6, 'default mode changed'
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
