"""Summarize one capture's full-frame geometric and dynamical investigation."""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('capture', type=Path)
a = p.parse_args()
out = a.capture / 'analysis/root_cause'
d = np.load(out / 'dynamics.npz')
t = d['time']
geometry = json.loads((out / 'geometry_all.json').read_text())['rows']
assert len(geometry) == 1850
assert sum(r['matches'] for r in geometry) == 3014303
raw = np.loadtxt(a.capture / 'capture/mat_out.txt')
imu = np.loadtxt(a.capture / 'capture/imu.txt')
R = Rotation.from_euler('xyz', raw[:, 1:4] / 57.3).as_matrix()
dt = np.r_[np.nan, np.diff(t)]
backward = next(i for i in range(len(t) - 19) if (d['forward'][i:i+20] < 0).all())
bg = np.load(out / f'all_frames/geometry-{int(d["scan_id"][backward]):05}.npz')
state = dict(zip(bg['frame_columns'], bg['frame']))
Rp = np.array([[state[f'prior_R_{i}_{j}'] for j in range(3)] for i in range(3)])
summary = dict(run='run-1790505839456699151-0', scans=len(t), geometry_scans=len(geometry),
               onset=dict(time=float(t[backward]), previous_forward=float(d['forward'][backward-1]),
                          prior_forward=float(Rp[:, 0] @ d['prior_velocity'][backward]),
                          after_forward=float(d['forward'][backward]),
                          constraints=int(d['constraints'][backward]), matches=int(d['matches'][backward])),
               final_forward=float(d['forward'][-1]),
               contribution_max_error=max(r['contribution_reconstruction_max_error'] for r in geometry), windows=[])
windows = [(140,160),(160,180),(180,200),(200,220),(220,260),(260,280),(280,300),(300,310),(310,315),(315,320),(320,325),(180,310)]
for lo, hi in windows:
    selected = [r for r in geometry if lo <= r['time'] < hi]
    mask = (t >= lo) & (t < hi)
    counts = sum(r['matches'] for r in selected)
    assert len(selected) == mask.sum()
    np.testing.assert_allclose(np.sum([r['categories']['all']['correction'] for r in selected],axis=0),
                               d['correction'][mask][:,np.r_[0:6,7:16]].sum(axis=0), atol=1e-10, rtol=1e-10)
    row = dict(start=lo, end=hi, scans=int(mask.sum()), geometry_scans=len(selected), matches=counts)
    fields = ['floor_sensor_pitch_deg','floor_world_tilt_deg','map_floor_world_tilt_deg',
              'map_floor_angle_deg','floor_rms','floor_count','floor_accepted_fraction',
              'source_quality_fraction','floor_match_angle_quantiles','floor_map_pca_max_quantiles']
    for k in fields:
        v = [r[k] for r in selected if k in r and r[k] is not None]
        if v:
            row[k + '_median'] = np.median(v, axis=0).tolist()
    floor_status = np.sum([r['floor_status_counts'] for r in selected if 'floor_status_counts' in r], axis=0)
    row['floor_status_counts'] = floor_status.tolist()
    row['floor_planefit_reject_fraction'] = float(floor_status[3] / floor_status.sum())
    row['categories'] = {}
    info = sum(r['categories']['all']['forward_information'] for r in selected)
    for k in selected[0]['categories']:
        cats = [r['categories'][k] for r in selected]
        row['categories'][k] = dict(
            count=sum(c['count'] for c in cats), count_fraction=sum(c['count'] for c in cats) / counts,
            correction=np.sum([c['correction'] for c in cats], axis=0).tolist(),
            absolute=np.sum([c['absolute'] for c in cats], axis=0).tolist(),
            forward_information_fraction=sum(c['forward_information'] for c in cats) / info,
            map_to_source_forward_information_ratio=sum(c['forward_information'] for c in cats) / max(1e-20, sum(c['source_forward_information'] for c in cats)))
    row['map_to_source_schur_median'] = float(np.median([r['coupled_information']['map_forward_schur'] / r['coupled_information']['source_forward_schur'] for r in selected]))
    row['unconstrained_sensitivity'] = {k: np.sum([r['unconstrained_sensitivity'][k] for r in selected], axis=0).tolist() for k in selected[0]['unconstrained_sensitivity']}
    row.update(pitch_mean_deg=float(np.rad2deg(d['pitch'][mask].mean())),
               ba_mean=d['ba'][mask].mean(axis=0).tolist(), bg_mean=d['bg'][mask].mean(axis=0).tolist(),
               posterior_std_median=np.median(d['std'][mask], axis=0).tolist(),
               prediction_dv=d['prediction_dv'][mask].sum(axis=0).tolist(),
               lidar_dv=d['update_dv'][mask].sum(axis=0).tolist(),
               unconstrained_dv=d['K0innovation'][mask,7:10].sum(axis=0).tolist(),
               residual_rms_median=float(np.median(d['residual_rms'][mask])),
               constraint_free_fraction=float((d['constraints'][mask] == 0).mean()))
    diagonal = np.diagonal(d['HWH'][mask], axis1=1, axis2=2)
    normalized = d['HWH'][mask] / np.sqrt(diagonal[:, :, None] * diagonal[:, None, :])
    row['normalized_hessian_min_eigen_median'] = float(np.median(np.linalg.eigvalsh(normalized)[:, 0]))
    ii = np.flatnonzero((imu[:, 0] >= lo) & (imu[:, 0] < hi))
    si = np.clip(np.searchsorted(t, imu[ii, 0]), 0, len(t)-1)
    accel = np.einsum('nij,nj->ni', R[si], imu[ii, 4:7] * 9.81 / .9997) + [0,0,-9.81]
    bias = -np.einsum('nij,nj->ni', R[si], d['ba'][si])
    row['inertial_approximation'] = dict(no_bias=accel.mean(axis=0).tolist(), bias=bias.mean(axis=0).tolist(),
            sum=(accel+bias).mean(axis=0).tolist(), actual=(d['prediction_dv'][mask]/dt[mask,None]).mean(axis=0).tolist())
    summary['windows'].append(row)
    b = row['categories']['source_disagree30']
    print(lo,hi,'floor_reject',round(row['floor_planefit_reject_fraction'],3),'map_tilt',round(row['map_floor_world_tilt_deg_median'],3),
          'bad count/info',round(b['count_fraction'],3),round(b['forward_information_fraction'],3),
          'Schur ratio',round(row['map_to_source_schur_median'],2),'bad/total baX',round(b['correction'][12],5),
          round(row['categories']['all']['correction'][12],5), flush=True)
(out / 'root_cause_summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def binned(field):
    x, y = [], []
    for lo in range(140,325):
        values = [r[field] for r in geometry if lo <= r['time'] < lo+1 and field in r
                  and r.get('floor_count', 0) >= 150 and r.get('floor_rms', 1) < .015]
        if values:
            x.append(lo+.5)
            y.append(np.median(values, axis=0))
    return np.array(x), np.array(y)

fig, axes = plt.subplots(4,1,figsize=(12,11),sharex=True)
m = (t>=140) & (t<325)
axes[0].plot(t[m], np.rad2deg(d['pitch'][m]),label='estimated IMU pitch',lw=.8)
axes[0].plot(*binned('floor_sensor_pitch_deg'),label='scan-only floor pitch proxy',lw=1.3)
axes[0].set_ylabel('Pitch [deg]')
axes[1].plot(*binned('floor_world_tilt_deg'),label='scan floor in estimated world')
axes[1].plot(*binned('map_floor_world_tilt_deg'),label='queried map floor')
axes[1].set_ylabel('Tilt from world vertical [deg]')
axes[2].plot(*binned('floor_accepted_fraction'),label='accepted fraction of scan floor')
axes[2].set_ylabel('Accepted / scan floor')
gx=[]; gy=[]
for lo in range(140,325):
    rr=[r for r in geometry if lo<=r['time']<lo+1]
    gx.append(lo+.5)
    gy.append(sum(r['categories']['source_disagree30']['forward_information'] for r in rr) / sum(r['categories']['all']['forward_information'] for r in rr))
axes[3].plot(gx,gy,label='information from >30 deg source/map disagreement')
axes[3].set_ylabel('Fraction of forward information');axes[3].set_xlabel('Node-relative time [s]')
for ax in axes:
    ax.grid(alpha=.25); ax.legend(fontsize=9); ax.axvspan(310,315,color='gray',alpha=.12)
fig.tight_layout();fig.savefig(out/'geometry_root_cause.png',dpi=150);plt.close(fig)

fig, axes = plt.subplots(4,1,figsize=(12,11))
axes[0].plot(t,d['ba'][:,0],label='estimated accel bias X');axes[0].plot(t,d['ba'][:,1],label='estimated accel bias Y')
axes[0].set_xlim(140,330);axes[0].set_ylabel('Bias [m/s²]')
axes[1].plot(t,d['forward'],label='posterior body-X velocity');axes[1].axhline(0,color='k',lw=.5);axes[1].set_xlim(300,330);axes[1].set_ylabel('Forward speed [m/s]')
show=t>=315
axes[2].plot(t[show],np.cumsum(d['prediction_dv'][show,0]),label='IMU prediction cumulative dvX')
axes[2].plot(t[show],np.cumsum(d['update_dv'][show,0]),label='LiDAR cumulative dvX')
axes[2].set_ylabel('World-X delta-v [m/s]');axes[2].set_xlabel('Node-relative time [s]')
for i in range(3):axes[3].plot(t[t>=140],d['std'][t>=140,13+i],label='accel bias '+ 'XYZ'[i] +' std')
axes[3].set_xlim(140,330);axes[3].set_ylabel('Posterior std [m/s²]');axes[3].set_xlabel('Node-relative time [s]')
for ax in axes:ax.grid(alpha=.25);ax.legend(fontsize=9);ax.axvline(t[backward],color='r',ls=':',lw=.8)
fig.tight_layout();fig.savefig(out/'dynamics_root_cause.png',dpi=150);plt.close(fig)
print('onset',summary['onset'])
