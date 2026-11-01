#!/usr/bin/env python3
"""Export an interrupted or bounded offline capture, explicitly as partial evidence."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def rotate(points, quaternion):
    q = np.asarray(quaternion, dtype=np.float64)
    q /= np.linalg.norm(q)
    xyz = q[:3]
    return points + 2 * (q[3] * np.cross(xyz, points) + np.cross(xyz, np.cross(xyz, points)))


def voxelize(points, size):
    keys = np.floor(points / size).astype(np.int32)
    _, indices = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(indices)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path, help='supervisor output directory')
    parser.add_argument('--voxel', type=float, default=0.1)
    args = parser.parse_args()
    if args.voxel <= 0:
        parser.error('voxel must be positive')
    capture = args.capture
    result = json.loads((capture / 'supervisor_result.json').read_text())
    with (capture / 'frontend/poses.csv').open() as source:
        poses = list(csv.DictReader(source))
    if not poses:
        raise ValueError('No saved poses')
    out = capture / 'export'
    out.mkdir(exist_ok=False)
    trajectory = []
    clouds = []
    for row in poses:
        p = np.array([float(row[k]) for k in ('px', 'py', 'pz')])
        q = np.array([float(row[k]) for k in ('qx', 'qy', 'qz', 'qw')])
        stamp = float(row['stamp'])
        if not np.isfinite(np.r_[p, q, stamp]).all():
            raise ValueError('Nonfinite saved pose')
        trajectory.append([stamp, *p, *q])
        scan = capture / 'frontend/scans' / f"{int(row['id'])}.bin"
        if not scan.is_file():
            raise ValueError(f'Missing scan {scan}')
        raw = np.fromfile(scan, dtype=np.float32).reshape(-1, 3)
        raw = raw[np.isfinite(raw).all(axis=1)]
        if len(raw):
            clouds.append(voxelize(rotate(raw.astype(np.float64), q) + p, args.voxel))
    np.savetxt(out / 'trajectory.tum', np.asarray(trajectory), fmt='%.12f')
    points = voxelize(np.concatenate(clouds), args.voxel).astype('<f4')
    header = ('ply\nformat binary_little_endian 1.0\n'
              f'element vertex {len(points)}\n'
              'property float x\nproperty float y\nproperty float z\nend_header\n')
    with (out / 'map.ply').open('wb') as target:
        target.write(header.encode('ascii'))
        target.write(points.tobytes())
    (out / 'README.md').write_text(
        '# 部分区間の未補正export\n\n'
        f'監督停止理由: `{result["stop_reason"]}`。bag全区間の地図・軌道ではない。\n'
        f'姿勢 {len(poses)} 件、保存区間の終端 {float(poses[-1]["time"]):.3f} 秒、'
        f'地図 {len(points)} 点。軌道の位置合わせ・平滑化・ループ補正は行っていない。\n'
    )
    print(json.dumps({'poses': len(poses), 'points': len(points),
                      'last_time_s': float(poses[-1]['time']),
                      'stop_reason': result['stop_reason']}))


if __name__ == '__main__':
    main()
