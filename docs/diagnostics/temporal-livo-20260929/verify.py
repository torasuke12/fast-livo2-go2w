"""Check temporal LIVO capture integrity and unchanged LIO numerical outputs."""
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
        data = list(csv.DictReader(stream))
    assert data and all(None not in r and None not in r.values() for r in data), path
    return data


def newest(root):
    return sorted(root.glob('lio-reconstruction-*'))[-1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('work', type=Path)
    parser.add_argument('previous_lio', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--livo-root', default='livo-10hz')
    parser.add_argument('--lio-root', default='lio-interval')
    parser.add_argument('--max-speed', type=float, default=5.0,
                        help='Gross-regression check for this walking-robot 0-60 s bag; not a ground-truth score')
    parser.add_argument('--max-accel-bias', type=float, default=1.0)
    args = parser.parse_args()
    result = {}
    for mode, folder in [('livo-final', args.livo_root), ('lio-final', args.lio_root)]:
        cap = newest(args.work/folder)
        summary = json.loads((cap/'summary.json').read_text())
        assert summary['capture_integrity'] == 'ok', summary
        params = yaml.safe_load((cap/'effective_params.yaml').read_text())
        window = rows(cap/'logs/temporal_window.csv')
        states = rows(cap/'logs/temporal_states.csv')
        for row in window:
            assert all(np.isfinite(float(v)) for k, v in row.items() if k != 'status')
        for row in states:
            assert all(np.isfinite(float(v)) for v in row.values())
        cov = np.array([[float(r[f'cov_{i}_{j}']) for i in range(15) for j in range(15)] for r in window]).reshape(-1,15,15)
        assert np.linalg.eigvalsh(cov).min() > 0
        data = {'capture':str(cap), 'summary':summary, 'window_rows':len(window), 'state_rows':len(states),
                'minimum_lidar_covariance_eigenvalue':float(np.linalg.eigvalsh(cov).min()),
                'logs_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest()
                               for p in (cap/'logs').glob('temporal_*.csv')},
                'effective_parameters':params}
        if mode == 'livo-final':
            visual = rows(cap/'logs/temporal_visual.csv')
            assert any(r['status']=='updated' for r in visual)
            for row in visual:
                assert all(np.isfinite(float(v)) for k,v in row.items() if k!='status')
            vcov = np.array([[float(r[f'cov_{i}_{j}']) for i in range(15) for j in range(15)] for r in visual]).reshape(-1,15,15)
            assert np.linalg.eigvalsh(vcov).min() > 0
            correction = np.array([[float(r[f'correction_{i}']) for i in range(15)] for r in visual])
            velocity = np.array([[float(r[k]) for k in ['vx','vy','vz']] for r in visual])
            bias = np.array([[float(r[k]) for k in ['bax','bay','baz']] for r in visual])
            data.update(visual_rows=len(visual), visual_status_counts=dict(Counter(r['status'] for r in visual)),
                        maximum_speed=float(np.linalg.norm(velocity,axis=1).max()),
                        maximum_accel_bias_norm=float(np.linalg.norm(bias,axis=1).max()),
                        minimum_visual_covariance_eigenvalue=float(np.linalg.eigvalsh(vcov).min()),
                        maximum_position_update=float(np.linalg.norm(correction[:,3:6],axis=1).max()),
                        maximum_velocity_update=float(np.linalg.norm(correction[:,6:9],axis=1).max()),
                        maximum_gyro_bias_update=float(np.linalg.norm(correction[:,9:12],axis=1).max()),
                        maximum_accel_bias_update=float(np.linalg.norm(correction[:,12:15],axis=1).max()))
            assert data['maximum_position_update'] > 0
            assert data['maximum_speed'] < args.max_speed, data['maximum_speed']
            assert data['maximum_accel_bias_norm'] < args.max_accel_bias, data['maximum_accel_bias_norm']
        else:
            previous = newest(args.previous_lio)
            comparison = {}
            for name in ['temporal_window.csv','temporal_states.csv','temporal_initialization.csv']:
                old, new = rows(previous/'logs'/name), rows(cap/'logs'/name)
                count = min(len(old),len(new))
                keys = [k for k in old[0] if k!='solve_ms']
                assert all(old[i][k]==new[i][k] for i in range(count) for k in keys), name
                comparison[name] = {'identical_rows':count,'excluded_columns':['solve_ms'] if name=='temporal_window.csv' else []}
            old, new = np.loadtxt(previous/'logs/imu.txt'), np.loadtxt(cap/'logs/imu.txt')
            count = min(len(old),len(new))
            assert np.array_equal(old[:count],new[:count])
            data.update(previous_capture=str(previous), numerical_parity=comparison, identical_imu_prefix_rows=count)
        result[mode] = data
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:{f:v[f] for f in ['capture','window_rows','state_rows']} for k,v in result.items()},indent=2))


if __name__ == '__main__':
    main()
