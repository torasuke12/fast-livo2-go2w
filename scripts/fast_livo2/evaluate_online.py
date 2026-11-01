#!/usr/bin/env python3
"""Export and evaluate chronological FAST-LIVO2 poses without adjusting them.

No registration, trajectory smoothing, loop detection, or endpoint constraint
is performed. Room windows select evaluation points only.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def load_trajectory(capture):
    rows = np.atleast_1d(np.genfromtxt(capture/'poses.csv', delimiter=',', names=True))
    if len(rows) < 2 or not np.all(np.diff(rows['time']) > 0):
        raise ValueError('Trajectory must contain increasing acquisition times')
    p = np.column_stack([rows[n] for n in ('px', 'py', 'pz')])
    q = np.column_stack([rows[n] for n in ('qx', 'qy', 'qz', 'qw')])
    if not np.isfinite(p).all() or not np.isfinite(q).all():
        raise ValueError('Nonfinite online trajectory')
    return rows, p, q, Rotation.from_quat(q).as_matrix()


def make_cloud(points, voxel, normals=False):
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    cloud = cloud.voxel_down_sample(voxel)
    if normals:
        cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.5, max_nn=30))
    return cloud


def compare(source, target):
    s, t = np.asarray(source.points), np.asarray(target.points)
    d, index = cKDTree(t).query(s)
    residual = np.einsum('ij,ij->i', s-t[index], np.asarray(target.normals)[index])
    close = d < .15
    return dict(points=len(s), overlap_15cm=float(close.mean()), nearest_median_m=float(np.median(d)),
                nearest_p90_m=float(np.quantile(d,.9)),
                inlier_plane_rmse_m=float(np.sqrt(np.mean(residual[close]**2))) if close.any() else None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--start', nargs=2, type=float, default=[4.,12.])
    parser.add_argument('--return-window', nargs=2, type=float, default=[632.,648.])
    parser.add_argument('--map', action='store_true', help='Export all saved scans at unmodified online poses')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Use a new output directory')
    summary = json.loads((args.capture/'summary.json').read_text())
    if not summary.get('complete') or not summary.get('bag_exhausted'):
        raise ValueError('Evaluation requires a completed full replay')
    rows, p, q, r = load_trajectory(args.capture)
    args.output.mkdir(parents=True)
    rooms = []
    for bounds in (args.start, args.return_window):
        points = []
        for i in np.flatnonzero((rows['time'] >= bounds[0]) & (rows['time'] <= bounds[1])):
            raw = np.fromfile(args.capture/'scans'/f'{int(rows[i]["id"])}.bin',np.float32).reshape(-1,3)
            # Same sample selection and range as the previous room evaluator.
            raw = raw[1::2]
            raw = raw[np.isfinite(raw).all(1) & (np.linalg.norm(raw,axis=1)<15)]
            points.append(raw @ r[i].T + p[i])
        if not points:
            raise ValueError('No scans in evaluation window')
        rooms.append(make_cloud(np.concatenate(points), .07, normals=True))
    metrics = dict(start_to_return=compare(rooms[0],rooms[1]), return_to_start=compare(rooms[1],rooms[0]))
    for c, color in zip(rooms, [[.05,.3,1.],[1.,.4,.02]]):
        c.paint_uniform_color(color)
    o3d.io.write_point_cloud(str(args.output/'rooms.ply'), rooms[0]+rooms[1])
    metrics['windows_seconds'] = [args.start, args.return_window]
    metrics['definition'] = ('Unmodified online poses; no evaluation-time alignment. Visibility and occlusion affect nearest distances. '
                             'Plane RMSE describes only pairs within 15 cm, not absolute trajectory accuracy.')
    velocity = np.column_stack([rows[n] for n in ('vx','vy','vz')])
    bias = np.column_stack([rows[n] for n in ('bax','bay','baz')])
    metrics['trajectory'] = dict(last_time=float(rows['time'][-1]), max_speed=float(np.linalg.norm(velocity,axis=1).max()),
                                max_accel_bias_norm=float(np.linalg.norm(bias,axis=1).max()),
                                end_position=p[-1].tolist(), z_range=[float(p[:,2].min()),float(p[:,2].max())])
    metrics['trajectory_sha256'] = hashlib.sha256((args.capture/'poses.csv').read_bytes()).hexdigest()
    metrics['loop_constraints'] = 0
    (args.output/'room_metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
    np.savetxt(args.output/'trajectory.tum',np.column_stack((rows['stamp'],p,q)),fmt='%.12f')
    np.savez_compressed(args.output/'trajectory.npz',time=rows['time'],position=p,quaternion=q)
    if args.map:
        merged = o3d.geometry.PointCloud()
        for i in range(len(rows)):
            raw = np.fromfile(args.capture/'scans'/f'{int(rows[i]["id"])}.bin',np.float32).reshape(-1,3)
            raw = raw[np.isfinite(raw).all(1)]
            merged += make_cloud(raw @ r[i].T + p[i], .1)
            if i % 50 == 0:
                merged = merged.voxel_down_sample(.1)
        merged = merged.voxel_down_sample(.1)
        o3d.io.write_point_cloud(str(args.output/'map.ply'),merged)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2,2,figsize=(13,10))
    for ax, a, b, label in [(axes[0,0],0,1,'XY'),(axes[0,1],0,2,'XZ')]:
        for c, color, name in zip(rooms,['royalblue','darkorange'],['Departure','Return']):
            xyz = np.asarray(c.points)
            ax.scatter(xyz[::2,a],xyz[::2,b],s=.3,color=color,label=name)
        ax.set_aspect('equal');ax.set_title('Room '+label+' (m)');ax.legend(markerscale=8);ax.grid()
    axes[1,0].plot(p[:,0],p[:,1]);axes[1,0].set_aspect('equal');axes[1,0].set_title('Online XY trajectory (m)')
    axes[1,1].plot(rows['time'],p[:,2]);axes[1,1].set_title('Online height (m)');axes[1,1].set_xlabel('Time (s)')
    for ax in axes[1]: ax.grid()
    fig.suptitle('No loops, no trajectory correction, no evaluation alignment')
    fig.tight_layout();fig.savefig(args.output/'comparison.png',dpi=150);plt.close(fig)
    print(json.dumps(metrics,indent=2))


if __name__ == '__main__':
    main()
