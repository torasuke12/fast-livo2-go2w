#!/usr/bin/env python3
"""Summarize the four bounded latest-default temporal captures, without alignment."""

import csv
import json
from pathlib import Path
from statistics import median

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'diagnostic_captures/current_default_temporal_20261101'
REFERENCE = {
    'sep07': ROOT / 'diagnostic_captures/pre_oct1_f281e70_20260907/lio/frontend/poses.csv',
    'oct20': ROOT / 'diagnostic_captures/pre_oct1_f281e70_20261020/lio/frontend_retry/poses.csv',
}
TIMES = [10, 20, 30, 35, 40, 44]


def read_csv(path):
    with path.open() as source:
        return list(csv.DictReader(source))


def positions(rows):
    return (np.array([float(r['time']) for r in rows]),
            np.array([[float(r[k]) for k in ('px', 'py', 'pz')] for r in rows]))


def interpolate(trajectory, query):
    t, xyz = trajectory
    if query < t[0] or query > t[-1]:
        return None
    return np.array([np.interp(query, t, xyz[:, axis]) for axis in range(3)])


def separation(first, second, query):
    a, b = interpolate(first, query), interpolate(second, query)
    return float(np.linalg.norm(a - b)) if a is not None and b is not None else None


def make_svg(trajectories):
    panels = []
    for index, day in enumerate(('sep07', 'oct20')):
        lines = [
            ('f281 LIO', positions(read_csv(REFERENCE[day])), '#555555'),
            ('temporal LIO', trajectories[f'{day}_lio_45'], '#1479b8'),
            ('temporal LIVO', trajectories[f'{day}_livo_45'], '#d65328'),
        ]
        lines[0] = (lines[0][0],
                    (lines[0][1][0][lines[0][1][0] <= 45],
                     lines[0][1][1][lines[0][1][0] <= 45]), lines[0][2])
        all_xy = np.concatenate([t[1][:, :2] for _, t, _ in lines])
        lo, hi = all_xy.min(axis=0), all_xy.max(axis=0)
        center = (lo + hi) / 2
        span = max(float(max(hi - lo)) * 1.15, 1)
        x0, y0 = 72 + 620 * index, 62
        side = 490
        panels.append(f'<text x="{x0}" y="35" font-size="20">{day}: first 45 s, unaligned XY</text>')
        panels.append(f'<rect x="{x0}" y="{y0}" width="{side}" height="{side}" fill="white" stroke="#999"/>')
        for label, (_, xy), color in lines:
            coords = [((p[0] - center[0]) / span * side + x0 + side / 2,
                       y0 + side / 2 - (p[1] - center[1]) / span * side) for p in xy]
            path = ' '.join(f'{x:.1f},{y:.1f}' for x, y in coords)
            panels.append(f'<polyline points="{path}" fill="none" stroke="{color}" stroke-width="2"/>')
            panels.append(f'<circle cx="{coords[-1][0]:.1f}" cy="{coords[-1][1]:.1f}" r="4" fill="{color}"/>')
        for j, (label, _, color) in enumerate(lines):
            lx, ly = x0 + j * 145, 580
            panels.append(f'<line x1="{lx}" y1="{ly}" x2="{lx+22}" y2="{ly}" stroke="{color}" stroke-width="3"/>')
            panels.append(f'<text x="{lx+27}" y="{ly+5}" font-size="13">{label}</text>')
    svg = ('<svg xmlns="http://www.w3.org/2000/svg" width="1260" height="620" '
           'viewBox="0 0 1260 620"><rect width="100%" height="100%" fill="#fafafa"/>'
           + ''.join(panels) + '</svg>\n')
    (BASE / 'comparison.svg').write_text(svg)


def capture(name):
    folder = BASE / name
    poses = read_csv(folder / 'frontend/poses.csv')
    window = read_csv(folder / 'frontend/logs/temporal_window.csv')
    visual_path = folder / 'frontend/logs/temporal_visual.csv'
    visual = read_csv(visual_path) if visual_path.exists() else []
    updates = [r for r in window if r['status'] == 'updated']
    visual_updates = [r for r in visual if r['status'] == 'updated']
    trajectory = positions(poses)
    speeds = np.linalg.norm(np.diff(trajectory[1], axis=0), axis=1) / np.diff(trajectory[0])
    result = json.loads((folder / 'supervisor_result.json').read_text())
    summary_path = folder / 'frontend/summary.json'
    summary = json.loads(summary_path.read_text()) if summary_path.exists() else None
    metrics = {
        'directory': str(folder.relative_to(ROOT)),
        'bag_exhausted': summary.get('bag_exhausted') if summary else False,
        'stop_reason': result['stop_reason'],
        'last_pose_time_s': float(poses[-1]['time']),
        'last_position_m': trajectory[1][-1].tolist(),
        'poses': len(poses), 'lidar_updates': len(updates),
        'max_saved_pose_interval_speed_m_s': float(max(speeds)),
        'matches_min': min(float(r['matches']) for r in updates),
        'matches_median': median(float(r['matches']) for r in updates),
        'lidar_rms_median_m': median(float(r['lidar_rms']) for r in updates),
        'lidar_rms_max_m': max(float(r['lidar_rms']) for r in updates),
        'last_lidar_position_covariance_diagonal_m2': [float(updates[-1][f'cov_{i}_{i}']) for i in (3, 4, 5)],
        'visual_updates': len(visual_updates),
        'visual_pixels_median': median(float(r['pixels']) for r in visual_updates) if visual_updates else None,
        'peak_rss_mib': result['peak_rss_bytes'] / 1024**2,
        'remaining_pids': result['remaining_pids'],
    }
    return metrics, trajectory


def main():
    output = {
        'definition': 'Bounded prefixes only. Positions are compared by linear time interpolation, in the shared unaligned start frame; no ground truth, registration, or full-bag evaluation.',
        'reference': {day: str(path.relative_to(ROOT)) for day, path in REFERENCE.items()},
        'captures': {}, 'separation_m': {},
    }
    trajectories = {}
    for day in ('sep07', 'oct20'):
        for mode in ('lio', 'livo'):
            name = f'{day}_{mode}_45'
            output['captures'][name], trajectories[name] = capture(name)
        reference = positions(read_csv(REFERENCE[day]))
        output['separation_m'][day] = {
            mode: {str(t): separation(trajectories[f'{day}_{mode}_45'], reference, t)
                   for t in TIMES if t <= trajectories[f'{day}_{mode}_45'][0][-1]}
            for mode in ('lio', 'livo')
        }
        output['separation_m'][day]['lio_vs_livo'] = {
            str(t): separation(trajectories[f'{day}_lio_45'], trajectories[f'{day}_livo_45'], t)
            for t in TIMES if t <= min(trajectories[f'{day}_lio_45'][0][-1],
                                      trajectories[f'{day}_livo_45'][0][-1])
        }
    (BASE / 'comparison.json').write_text(json.dumps(output, indent=2, allow_nan=False) + '\n')
    make_svg(trajectories)
    print(BASE / 'comparison.json')


if __name__ == '__main__':
    main()
