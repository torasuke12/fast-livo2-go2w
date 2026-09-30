#!/usr/bin/env python3
"""Compare saved LIO ON/OFF startup states, without treating either as truth."""
import argparse
import csv
import hashlib
import io
import json
from pathlib import Path

import numpy as np
import yaml


def columns(rows, names):
    return np.array([[float(row[name]) for name in names] for row in rows])


def rotations(rows):
    w, x, y, z = columns(rows, ['qw', 'qx', 'qy', 'qz']).T
    return np.stack([
        1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y),
        2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x),
        2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y),
    ], axis=1).reshape(-1, 3, 3)


def angle(a, b):
    return np.rad2deg(np.arccos(np.clip((np.einsum('...ij,...ij->...', a, b)-1)/2, -1, 1)))


def rpy(r):
    return np.rad2deg(np.column_stack([
        np.arctan2(r[:, 2, 1], r[:, 2, 2]),
        np.arcsin(np.clip(-r[:, 2, 0], -1, 1)),
        np.arctan2(r[:, 1, 0], r[:, 0, 0]),
    ]))


def flatten(data, prefix=''):
    out = {}
    for k, v in data.items():
        if isinstance(v, dict):
            out.update(flatten(v, prefix+k+'.'))
        else:
            out[prefix+k] = v
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('on', type=Path)
    parser.add_argument('off', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    hashes = {}

    def read(path):
        data = path.read_bytes()
        hashes[str(path)] = hashlib.sha256(data).hexdigest()
        return data.decode()

    def csv_rows(path):
        return list(csv.DictReader(io.StringIO(read(path))))

    def params(path):
        data = yaml.safe_load(read(path))
        nodes = [v for k, v in data.items() if k.lstrip('/') == 'laserMapping']
        assert len(nodes) == 1
        return flatten(nodes[0]['ros__parameters'])

    on_info, off_info = [json.loads(read(p/'capture.json')) for p in (args.on, args.off)]
    on_params, off_params = [params(p/'effective_params.yaml') for p in (args.on, args.off)]
    assert on_params['common.img_en'] == off_params['common.img_en'] == 0
    assert on_params['lio.temporal.enabled'] and not off_params['lio.temporal.enabled']
    geometry = sorted((args.off/'diagnostics').glob('*/geometry_states.csv'))
    assert len(geometry) == 1
    off = csv_rows(geometry[0])
    on_all = csv_rows(args.on/'logs/temporal_window.csv')
    states = csv_rows(args.on/'logs/temporal_states.csv')
    time = columns(off, ['time'])[:, 0]
    nt = columns(on_all, ['time'])[:, 0]
    ix = np.abs(nt[:, None]-time[None, :]).argmin(axis=0)
    assert np.max(np.abs(nt[ix]-time)) < 1e-6
    assert len(np.unique(ix)) == len(time)
    on = [on_all[i] for i in ix]
    on_r = rotations(on)
    off_r = columns(off, [f'after_R_{i}_{j}' for i in range(3) for j in range(3)]).reshape(-1, 3, 3)
    off_prior_r = columns(off, [f'prior_R_{i}_{j}' for i in range(3) for j in range(3)]).reshape(-1, 3, 3)
    on_p = columns(on, ['px', 'py', 'pz'])
    off_p = columns(off, [f'after_position_{i}_0' for i in range(3)])
    off_prior_p = columns(off, [f'prior_position_{i}_0' for i in range(3)])
    on_correction = np.rad2deg(np.linalg.norm(columns(on, [f'correction_{k}' for k in range(3)]), axis=1))
    off_correction = angle(off_r, off_prior_r)
    on_position_update = np.linalg.norm(columns(on, [f'correction_{k}' for k in range(3, 6)]), axis=1)
    off_position_update = np.linalg.norm(off_p-off_prior_p, axis=1)
    imu_on, imu_off = [np.loadtxt(io.StringIO(read(p/'logs/imu.txt'))) for p in (args.on, args.off)]
    assert len(imu_on) >= len(imu_off)
    assert np.array_equal(imu_on[:len(imu_off)], imu_off)
    initial_states = [row for row in states if int(row['node_id']) == 0]
    initial_r = rotations(initial_states)
    initial_p = columns(initial_states, ['px', 'py', 'pz'])
    initial_time = columns(initial_states, ['time'])[:, 0]
    revision_angle = angle(initial_r, initial_r[0])
    revision_position = np.linalg.norm(initial_p-initial_p[0], axis=1)
    first_updates = []
    for i in range(min(8, len(on))):
        k = int(np.abs(initial_time-time[i]).argmin())
        assert abs(initial_time[k]-time[i]) < 1e-6
        first_updates.append(dict(
            time=float(time[i]), on_rpy_deg=rpy(on_r)[i].tolist(), off_rpy_deg=rpy(off_r)[i].tolist(),
            on_position_m=on_p[i].tolist(), off_position_m=off_p[i].tolist(),
            on_prediction_correction_deg=float(on_correction[i]), off_prediction_correction_deg=float(off_correction[i]),
            first_node_revised_rotation_deg=float(revision_angle[k]),
            first_node_revised_position_m=float(revision_position[k]),
            simultaneous_first_to_latest_rotation_deg=float(angle(initial_r[k], on_r[i])),
            on_imu_rms=float(on[i]['imu_rms']), on_lidar_rms_m=float(on[i]['lidar_rms']),
        ))
    result = dict(
        on_capture=args.on.name, off_capture=args.off.name,
        on_arguments=on_info['arguments'], off_arguments=off_info['arguments'],
        same_binary=on_info['binary_sha256'] == off_info['binary_sha256'],
        binary_sha256=on_info['binary_sha256'],
        matched_scans=len(time), interval_sec=[float(time[0]), float(time[-1])],
        imu_identical_prefix_rows=len(imu_off),
        parameter_differences={k: [on_params.get(k), off_params.get(k)] for k in sorted(set(on_params)|set(off_params))
                               if on_params.get(k) != off_params.get(k)},
        first_updates=first_updates,
        max_prediction_rotation_update_deg=dict(on=float(on_correction.max()), off=float(off_correction.max())),
        max_prediction_position_update_m=dict(on=float(on_position_update.max()), off=float(off_position_update.max())),
        first_node_revision_max=dict(rotation_deg=float(revision_angle.max()), position_m=float(revision_position.max())),
        source_sha256=hashes,
        limitations=['OFF capture has no final summary; only fully saved rows are compared.',
                     'No ground truth; corrections and differences are not absolute errors.',
                     'Common IMU samples, scan times and binary are verified; this does not isolate every backend change.',
                     'The first-node revision is an optimized history change, not measured physical motion.'])
    # Do not silently analyze a file changing while it is being read.
    for path, expected in hashes.items():
        assert hashlib.sha256(Path(path).read_bytes()).hexdigest() == expected, path
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'comparison.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    for r, label in ((on_r, 'Temporal ON'), (off_r, 'Temporal OFF')):
        axes[0].plot(time, rpy(r)[:, 0], label=label)
    axes[0].set_ylabel('Published roll (deg)')
    for p, label in ((on_p, 'Temporal ON'), (off_p, 'Temporal OFF')):
        axes[1].plot(time, p[:, 2], label=label)
    axes[1].set_ylabel('Published z (m)')
    axes[2].plot(initial_time, revision_angle, label='Revision of initial pose, ON')
    axes[2].set_ylabel('Rotation from original (deg)')
    axes[2].set_xlabel('Seconds since first LiDAR')
    for ax in axes:
        ax.grid(alpha=.3)
        ax.legend()
        ax.set_xlim(float(time[0])-.1, float(time[-1]))
    fig.suptitle('Startup regression: same bag, LIO-only, rate 0.25\nSaved estimates; neither method is ground truth')
    fig.tight_layout()
    fig.savefig(args.output/'startup_comparison.png', dpi=160)
    plt.close(fig)
    print(json.dumps({k: v for k, v in result.items() if k not in ('source_sha256', 'parameter_differences')}, indent=2))


if __name__ == '__main__':
    main()
