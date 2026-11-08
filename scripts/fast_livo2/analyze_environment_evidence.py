#!/usr/bin/env python3
"""Join existing pose, temporal, visual, and keyframe-scan evidence by bag time.

This script reads saved captures only. Reference trajectories are comparison
trajectories, not independent ground truth. No replay or registration is done.
"""

import csv
import json
import math
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'diagnostic_captures/current_default_temporal_20261101'
REFERENCE = {
    'sep07': ROOT / 'diagnostic_captures/pre_oct1_f281e70_20260907/lio/frontend/poses.csv',
    'oct20': ROOT / 'diagnostic_captures/pre_oct1_f281e70_20261020/lio/frontend_retry/poses.csv',
}
INTERVALS = [(15, 20), (20, 25), (25, 30), (30, 35), (35, 40), (40, 44)]
SAMPLES = [20, 25, 30, 35, 40, 44]


def rows(path):
    with path.open() as source:
        return list(csv.DictReader(source))


def median(items):
    return float(np.median(items)) if len(items) else None


def segment(data, start, end):
    return [r for r in data if start <= float(r['time']) < end]


def pose_arrays(data):
    t = np.array([float(r['time']) for r in data])
    p = np.array([[float(r[k]) for k in ('px', 'py', 'pz')] for r in data])
    q = np.array([[float(r[k]) for k in ('qx', 'qy', 'qz', 'qw')] for r in data])
    return t, p, q


def position_at(pose, time):
    t, p, _ = pose
    if not t[0] <= time <= t[-1]:
        return None
    return np.array([np.interp(time, t, p[:, i]) for i in range(3)])


def yaw_at(pose, time):
    t, _, q = pose
    if not t[0] <= time <= t[-1]:
        return None
    x, y, z, w = [np.interp(time, t, q[:, i]) for i in range(4)]
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def separation(a, b, time):
    pa, pb = position_at(a, time), position_at(b, time)
    return float(np.linalg.norm(pa - pb)) if pa is not None and pb is not None else None


def angle_difference(a, b):
    return (a - b + 180) % 360 - 180


def initial_pose_aligned_separation(a, b, time, anchor=20):
    """Use only the 20 s position and yaw to remove the start-frame gauge."""
    pa, pb = position_at(a, time), position_at(b, time)
    aa, bb = position_at(a, anchor), position_at(b, anchor)
    if any(value is None for value in (pa, pb, aa, bb)):
        return None
    angle = math.radians(yaw_at(b, anchor) - yaw_at(a, anchor))
    c, s = math.cos(angle), math.sin(angle)
    rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return float(np.linalg.norm(rotation @ (pa - aa) + bb - pb))


def first_exceedance(a, b, limit):
    for time in np.arange(max(a[0][0], b[0][0]), min(a[0][-1], b[0][-1]), 0.1):
        if separation(a, b, time) > limit:
            return round(float(time), 1)
    return None


def scan_geometry(capture, pose_data, start, end):
    values = []
    for row in segment(pose_data, start, end):
        raw = np.fromfile(capture / 'frontend/scans' / f"{row['id']}.bin", dtype=np.float32).reshape(-1, 3)
        raw = raw[np.isfinite(raw).all(axis=1)]
        raw = raw[np.linalg.norm(raw, axis=1) < 20]
        if len(raw) < 100:
            continue
        eig = np.linalg.eigvalsh(np.cov(raw[:, :2].T))
        values.append((eig[1] / eig[0], np.median(np.linalg.norm(raw, axis=1)), len(raw)))
    return {'scans': len(values),
            'horizontal_covariance_eigenvalue_ratio_median': median([v[0] for v in values]),
            'point_range_median_m': median([v[1] for v in values]),
            'points_within_20m_median': median([v[2] for v in values])}


def diagnostics(capture, start, end):
    pose = segment(rows(capture / 'frontend/poses.csv'), start, end)
    lidar = [r for r in segment(rows(capture / 'frontend/logs/temporal_window.csv'), start, end)
             if r['status'] == 'updated']
    visual_path = capture / 'frontend/logs/temporal_visual.csv'
    visual = ([r for r in segment(rows(visual_path), start, end) if r['status'] == 'updated']
              if visual_path.exists() else [])
    bias = [np.linalg.norm([float(r[k]) for k in ('bax', 'bay', 'baz')]) for r in pose]
    speed = [np.linalg.norm([float(r[k]) for k in ('vx', 'vy', 'vz')]) for r in pose]
    correction = [np.linalg.norm([float(r[f'correction_{i}']) for i in (3, 4, 5)]) for r in visual]
    return {
        'pose_count': len(pose),
        'estimated_speed_median_m_s': median(speed),
        'estimated_accel_bias_norm_median_m_s2': median(bias),
        'lidar_updates': len(lidar),
        'imu_saturated_intervals_sum': sum(int(r['saturated_imu_intervals']) for r in lidar),
        'lidar_effective_matches_median': median([float(r['matches']) for r in lidar]),
        'lidar_rms_median_m': median([float(r['lidar_rms']) for r in lidar]),
        'position_covariance_diagonal_median_m2': [median([float(r[f'cov_{i}_{i}']) for r in lidar])
                                                    for i in (3, 4, 5)],
        'visual_updates': len(visual),
        'visual_pixels_median': median([float(r['pixels']) for r in visual]),
        'visual_candidate_points_median': median([float(r['candidate_points']) for r in visual]),
        'visual_position_correction_median_m': median(correction),
        'visual_position_correction_p90_m': float(np.quantile(correction, 0.9)) if correction else None,
    }


def main():
    result = {
        'method': ('No replay, trajectory fitting, or truth trajectory. Raw pose differences use linear time '
                   'interpolation in each bag start frame. Initial-pose-aligned differences use only the '
                   'position and yaw at 20 s to remove a fixed starting-frame offset. Diagnostics use '
                   'half-open bag-time intervals. Scan geometry '
                   'uses raw saved LIO keyframes, finite points within 20 m, and horizontal XY covariance '
                   'eigenvalue ratio; this is shape anisotropy, not LiDAR observability rank.'),
        'reference': {day: str(path.relative_to(ROOT)) for day, path in REFERENCE.items()},
        'bags': {},
    }
    for day in ('sep07', 'oct20'):
        captures = {mode: BASE / f'{day}_{mode}_45' for mode in ('lio', 'livo')}
        poses = {mode: rows(captures[mode] / 'frontend/poses.csv') for mode in captures}
        tracks = {mode: pose_arrays(poses[mode]) for mode in captures}
        reference = pose_arrays(rows(REFERENCE[day]))
        bag = {'first_lio_livo_separation_above_1m_s': first_exceedance(tracks['lio'], tracks['livo'], 1),
               'first_lio_livo_separation_above_5m_s': first_exceedance(tracks['lio'], tracks['livo'], 5),
               'samples': {}, 'intervals': {}}
        for time in SAMPLES:
            if any(position_at(tracks[mode], time) is None for mode in tracks):
                continue
            bag['samples'][str(time)] = {
                'lio_vs_livo_position_m': separation(tracks['lio'], tracks['livo'], time),
                'lio_vs_reference_position_m': separation(tracks['lio'], reference, time),
                'livo_vs_reference_position_m': separation(tracks['livo'], reference, time),
                'lio_vs_reference_initial_pose_aligned_position_m': initial_pose_aligned_separation(tracks['lio'], reference, time),
                'livo_vs_reference_initial_pose_aligned_position_m': initial_pose_aligned_separation(tracks['livo'], reference, time),
                'lio_vs_livo_initial_pose_aligned_position_m': initial_pose_aligned_separation(tracks['lio'], tracks['livo'], time),
                'lio_vs_reference_yaw_deg': angle_difference(yaw_at(tracks['lio'], time), yaw_at(reference, time)),
                'livo_vs_reference_yaw_deg': angle_difference(yaw_at(tracks['livo'], time), yaw_at(reference, time)),
                'positions_m': {mode: position_at(tracks[mode], time).tolist() for mode in tracks},
            }
        for start, end in INTERVALS:
            bag['intervals'][f'{start}-{end}'] = {
                'scene_geometry': scan_geometry(captures['lio'], poses['lio'], start, end),
                'lio': diagnostics(captures['lio'], start, end),
                'livo': diagnostics(captures['livo'], start, end),
            }
        result['bags'][day] = bag
    output = BASE / 'environment_evidence.json'
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(output)


if __name__ == '__main__':
    main()
