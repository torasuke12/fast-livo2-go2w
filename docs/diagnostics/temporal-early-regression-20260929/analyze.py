"""Compare recorded estimates before 320 s, without treating either as truth."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml


def read_csv(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def columns(rows, names):
    return np.array([[float(row[name]) for name in names] for row in rows])


def rotation_angle(a, b):
    return np.rad2deg(np.arccos(np.clip((np.einsum('nij,nij->n', a, b)-1)/2, -1, 1)))


def flatten(data, prefix=''):
    result = {}
    for key, value in data.items():
        name = f'{prefix}.{key}' if prefix else key
        if isinstance(value, dict):
            result.update(flatten(value, name))
        else:
            result[name] = value
    return result


def parameters(path):
    data = yaml.safe_load(path.read_text())
    nodes = [value for key, value in data.items() if key.lstrip('/') == 'laserMapping']
    assert len(nodes) == 1, path
    return flatten(nodes[0]['ros__parameters'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('baseline', type=Path, help='preserved voxel-plane snapshot data directory')
    parser.add_argument('temporal', type=Path, help='temporal capture directory')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    manifest = json.loads((args.baseline/'snapshot.json').read_text())
    checked = {}
    for name in ['diagnostics/geometry_states.csv', 'logs/imu.txt', 'effective_params.yaml']:
        digest = hashlib.sha256((args.baseline/name).read_bytes()).hexdigest()
        assert digest == manifest['files'][name]['sha256'], name
        checked[name] = digest
    old = read_csv(args.baseline/'diagnostics/geometry_states.csv')
    new = read_csv(args.temporal/'logs/temporal_window.csv')
    old = [row for row in old if float(row['time']) < 320]
    time = columns(old, ['time'])[:, 0]
    nt = columns(new, ['time'])[:, 0]
    ix = np.clip(np.searchsorted(nt, time), 1, len(nt)-1)
    ix = np.where(abs(nt[ix]-time) < abs(nt[ix-1]-time), ix, ix-1)
    stamp_difference = float(abs(nt[ix]-time).max())
    assert stamp_difference < 1e-6 and len(np.unique(ix)) == len(time)
    new = [new[i] for i in ix]
    position = columns(old, [f'after_position_{i}_0' for i in range(3)])
    rotation = columns(old, [f'after_R_{i}_{j}' for i in range(3) for j in range(3)]).reshape(-1, 3, 3)
    prior_rotation = columns(old, [f'prior_R_{i}_{j}' for i in range(3) for j in range(3)]).reshape(-1, 3, 3)
    new_position = columns(new, ['px', 'py', 'pz'])
    w, x, y, z = columns(new, ['qw', 'qx', 'qy', 'qz']).T
    new_rotation = np.stack([
        1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y),
        2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x),
        2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y),
    ], axis=1).reshape(-1, 3, 3)
    old_rotation_update = rotation_angle(rotation, prior_rotation)
    new_rotation_update = np.rad2deg(np.linalg.norm(columns(new, [f'correction_{i}' for i in range(3)]), axis=1))
    old_position_update = np.linalg.norm(position-columns(old, [f'prior_position_{i}_0' for i in range(3)]), axis=1)
    new_position_update = np.linalg.norm(columns(new, [f'correction_{i}' for i in range(3, 6)]), axis=1)

    # One horizontal rigid transform estimated on the first travelling segment.
    # No scale change, time warp, per-segment alignment, or whole-run fitting.
    fit = (time >= 20) & (time < 60)
    a, b = new_position[fit, :2], position[fit, :2]
    u, _, vh = np.linalg.svd((a-a.mean(0)).T @ (b-b.mean(0)))
    alignment = vh.T @ np.diag([1, np.linalg.det(vh.T @ u.T)]) @ u.T
    translation = b.mean(0)-alignment @ a.mean(0)
    aligned = new_position[:, :2] @ alignment.T+translation
    xy_difference = np.linalg.norm(aligned-position[:, :2], axis=1)
    angle = rotation_angle(rotation, new_rotation)

    last_keyframes = {}
    for row in read_csv(args.temporal/'logs/temporal_states.csv'):
        if int(row['keyframe']):
            last_keyframes[int(row['node_id'])] = row
    keys = [r for _, r in sorted(last_keyframes.items()) if time[0] <= float(r['node_time']) < 320]
    key_time = columns(keys, ['node_time'])[:, 0]
    key_position = columns(keys, ['px', 'py', 'pz'])
    aligned_keys = key_position[:, :2] @ alignment.T+translation
    old_at_keys = np.column_stack([np.interp(key_time, time, position[:, i]) for i in range(2)])
    key_difference = np.linalg.norm(aligned_keys-old_at_keys, axis=1)

    windows = []
    for begin, end in [(float(time[0]), 10), (10, 20), (20, 60), (60, 140), (140, 180), (180, 220), (220, 300), (300, 320)]:
        mask = (time >= begin) & (time < end)
        row = dict(start=begin, end=end, scans=int(mask.sum()))
        for name, values in [('old_rotation_update_deg', old_rotation_update), ('new_rotation_update_deg', new_rotation_update),
                             ('old_position_update_m', old_position_update), ('new_position_update_m', new_position_update)]:
            row[name] = dict(zip(['median', 'p95', 'max'], np.quantile(values[mask], [.5, .95, 1]).tolist()))
        windows.append(row)
    checkpoints = []
    for target in [10, 20, 30, 60, 100, 140, 180, 220, 260, 300, 319.9]:
        i = int(np.argmin(abs(time-target)))
        k = int(np.argmin(abs(key_time-time[i])))
        checkpoints.append(dict(time=float(time[i]), position_difference_unaligned_m=(new_position[i]-position[i]).tolist(),
                                rotation_difference_deg=float(angle[i]), aligned_xy_difference_m=float(xy_difference[i]),
                                nearest_keyframe_time=float(key_time[k]), aligned_keyframe_xy_difference_m=float(key_difference[k])))
    old_imu = np.loadtxt(args.baseline/'logs/imu.txt')
    new_imu = np.loadtxt(args.temporal/'logs/imu.txt')
    imu_equal = bool(np.array_equal(old_imu, new_imu[:len(old_imu)]))
    assert imu_equal
    old_params = parameters(args.baseline/'effective_params.yaml')
    new_params = parameters(args.temporal/'effective_params.yaml')
    prefixes = ('common.', 'extrin_calib.', 'imu.', 'preprocess.', 'uav.')
    parameter_differences = {key: [old_params.get(key), new_params.get(key)]
                             for key in sorted(set(old_params) | set(new_params))
                             if key.startswith(prefixes) and old_params.get(key) != new_params.get(key)}
    peak = int(np.argmax(new_position_update))
    result = dict(baseline=str(args.baseline), temporal=str(args.temporal), baseline_snapshot_hashes=checked,
                  matched_scans=len(time), timestamp_difference_max_sec=stamp_difference,
                  imu_equal_prefix_rows=len(old_imu), imu_equal_all_columns=imu_equal,
                  input_and_initialization_parameter_differences=parameter_differences,
                  alignment=dict(fit_start=20, fit_end=60, yaw_deg=float(np.rad2deg(np.arctan2(alignment[1, 0], alignment[0, 0]))),
                                 translation_xy_m=translation.tolist(), fit_rmse_m=float(np.sqrt(np.mean(xy_difference[fit]**2)))),
                  largest_new_position_update=dict(time=float(time[peak]), norm_m=float(new_position_update[peak])),
                  windows=windows, checkpoints=checkpoints,
                  caveats=['Differences between methods are not ground-truth errors.',
                           'Updates include each estimator own prediction; they do not isolate LiDAR-only causality.',
                           'Latest online states and last optimized keyframes are different output histories.'])
    (args.output/'comparison.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    np.savez_compressed(args.output/'comparison.npz', time=time, old_position=position, new_position=new_position,
                        aligned_xy=aligned, old_rotation_update=old_rotation_update, new_rotation_update=new_rotation_update,
                        old_position_update=old_position_update, new_position_update=new_position_update,
                        angle=angle, xy_difference=xy_difference, key_time=key_time, aligned_keys=aligned_keys,
                        key_difference=key_difference)
    print(json.dumps({k: v for k, v in result.items() if k not in ('windows', 'checkpoints')}, indent=2))


if __name__ == '__main__':
    main()
