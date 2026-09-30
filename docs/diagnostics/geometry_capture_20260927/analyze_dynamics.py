"""Exact update decomposition for the geometry capture (not an alternative replay)."""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path('humble_ws/src/fast-livo2/scripts').resolve()))
from analyze_lio_diagnostics import Table

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('capture', type=Path)
a = p.parse_args()
run, = (a.capture / 'diagnostics').glob('run-*')
out = a.capture / 'analysis/root_cause'
out.mkdir(parents=True, exist_ok=True)
it = Table(run / 'iterations.csv')
sc = Table(run / 'scans.csv', ('status',))
x = Table(a.capture / 'capture/xicp_diagnostic.csv')
frames = Table(run / 'geometry_states.csv')
imu = np.loadtxt(a.capture / 'capture/imu.txt')
raw = np.loadtxt(a.capture / 'capture/mat_out.txt')
t, first = np.unique(x['time'], return_index=True)
last = np.r_[first[1:] - 1, len(x) - 1]
keys = {(int(s), int(i)): j for j, (s, i) in enumerate(zip(it['scan_id'], it['iteration']))}
sel = np.array([keys[(int(s), int(i))] for s, i in zip(sc['scan_id'], sc['committed_iteration'])])
assert np.max(abs(t - sc['time'])) < 1e-8 and len(t) == len(raw)
t = sc['time']


def mat(name, r, c):
    return it.matrix(name, r, c)[sel]


def xyz(prefix, idx):
    return np.column_stack([x[prefix + axis][idx] for axis in 'xyz'])


P, A = mat('P_iter', 19, 19), mat('HWH', 6, 6)
correction = mat('Kinnovation', 19, 1)[..., 0]
M = np.linalg.solve(P[:, :6, :6], P[:, :6, 7:16]).transpose(0, 2, 1)
parts = M * correction[:, None, :6]
conditional_error = float(np.max(abs(parts.sum(axis=2) - correction[:, 7:16])))
assert conditional_error < 1e-10
v, vp = xyz('after_v', last), xyz('prior_v', first)
ba, bg = xyz('after_ba_', last), xyz('after_bg_', last)
pitch, pitchp = x['after_pitch'][last], x['prior_pitch'][first]
forward = x['after_forward_speed'][last]
pred = np.vstack([np.zeros(3), vp[1:] - v[:-1]])
update = v - vp
assert np.max(abs(update - correction[:, 7:10])) < 1e-9
pitchpred = np.r_[0, pitchp[1:] - pitch[:-1]]
pitchupd = pitch - pitchp
std = np.sqrt(np.diagonal(sc.matrix('P_after', 19, 19), axis1=1, axis2=2))
score = mat('HTWinnovation', 6, 1)[..., 0]
k0 = (P[:, :, :6] @ np.linalg.solve(np.eye(6)[None] + A @ P[:, :6, :6], score[..., None]))[..., 0]
np.savez_compressed(out / 'dynamics.npz', time=t, scan_id=sc['scan_id'], iteration=sc['committed_iteration'],
                    P=P, HWH=A, M=M, correction=correction, K0innovation=k0,
                    velocity=v, prior_velocity=vp, ba=ba, bg=bg, pitch=pitch, forward=forward,
                    prediction_dv=pred, update_dv=update, std=std,
                    residual_rms=it['residual_rms'][sel], matches=it['matches'][sel],
                    constraints=it['constraints'][sel])
windows = []
for lo, hi in [(140, 160), (160, 180), (180, 200), (200, 210), (210, 220), (220, 230),
               (230, 260), (260, 280), (280, 300), (300, 310), (310, 315), (315, 320), (320, 325), (325, 330), (180, 310)]:
    mask = (t >= lo) & (t < hi)
    imask = (imu[:, 0] >= lo) & (imu[:, 0] < hi)
    amean = imu[imask, 4:7].mean(axis=0) * 9.81 / .9997
    psum = parts[mask].sum(axis=0)
    row = dict(start=lo, end=hi, scans=int(mask.sum()), pitch_mean_deg=float(np.rad2deg(pitch[mask].mean())),
               ba_mean=ba[mask].mean(axis=0).tolist(), bg_mean=bg[mask].mean(axis=0).tolist(),
               pitch_prediction_sum_deg=float(np.rad2deg(pitchpred[mask].sum())),
               pitch_update_sum_deg=float(np.rad2deg(pitchupd[mask].sum())),
               ba_change=correction[mask, 13:16].sum(axis=0).tolist(),
               ba_from_position=psum[6:9, 3:].sum(axis=1).tolist(),
               bg_from_rotation=psum[3:6, :3].sum(axis=1).tolist(),
               bg_from_position=psum[3:6, 3:].sum(axis=1).tolist(),
               prediction_dv=pred[mask].sum(axis=0).tolist(), lidar_dv=update[mask].sum(axis=0).tolist(),
               K0_dv=k0[mask, 7:10].sum(axis=0).tolist(), forward_mean=float(forward[mask].mean()),
               gyro_mean=imu[imask, 1:4].mean(axis=0).tolist(), scaled_accel_mean=amean.tolist(),
               accel_pitch_proxy_deg=float(np.rad2deg(np.arctan2(-amean[0], np.linalg.norm(amean[1:])))),
               residual_rms_median=float(np.median(it['residual_rms'][sel][mask])),
               std_median=np.median(std[mask], axis=0).tolist())
    windows.append(row)
summary = dict(run=str(run), conditional_identity_max_error=conditional_error, windows=windows)
(out / 'dynamics.json').write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
for w in windows:
    print(w['start'], w['end'], 'pitch', round(w['pitch_mean_deg'], 3), 'accel_pitch', round(w['accel_pitch_proxy_deg'], 3),
          'pitch IMU/LIO', np.round([w['pitch_prediction_sum_deg'], w['pitch_update_sum_deg']], 4),
          'dbaX', round(w['ba_change'][0], 5), 'dvX IMU/LIO', np.round([w['prediction_dv'][0], w['lidar_dv'][0]], 4))
