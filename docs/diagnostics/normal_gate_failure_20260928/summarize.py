"""Summarize recorded normal-gate experiment; does not replay the estimator."""
from pathlib import Path
import argparse, json, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ap = argparse.ArgumentParser()
ap.add_argument('work', type=Path)
ap.add_argument('output', type=Path)
args = ap.parse_args()
p, out = args.work, args.output
out.mkdir(parents=True, exist_ok=True)
read = lambda path: json.loads(path.read_text())
groups = read(p/'out/all_point_groups.json')
floor = read(p/'sample_geometry.json')['rows']
dyn = read(p/'dynamics/summary.json')
d = np.load(p/'dynamics/dynamics.npz')
baseline = np.load('diagnostic_captures/lio-geometry-ehdz2pcz/analysis/root_cause/dynamics.npz')

windows = []
for lo, hi in [(180,200),(200,220),(220,260),(260,300),(300,310),(315,320),(320,325),(330,350),(180,310)]:
    rows = [r for r in groups['rows'] if lo <= r['time'] < hi]
    total_count = sum(r['groups']['all']['count'] for r in rows)
    total_information = sum(r['groups']['all']['forward_information'] for r in rows)
    g = {}
    for k in ['all','unavailable','angle0_5','angle5_15','angle15_30']:
        counts = sum(r['groups'][k]['count'] for r in rows)
        information = sum(r['groups'][k]['forward_information'] for r in rows)
        g[k] = dict(count=counts, count_fraction=counts/total_count,
                    forward_information_fraction=information/total_information,
                    bax_correction=sum(r['groups'][k]['correction'][6] for r in rows))
    # Floor statistics are from the cached 1 Hz + 295--325 s full-rate sample.
    fs = [r for r in floor if lo <= r['time'] < hi and r.get('floor_count',0)>=150 and r.get('floor_rms',1)<.015]
    f = {'scans':len(fs)}
    for k in ['floor_rms','floor_sensor_pitch_deg','floor_world_tilt_deg','map_floor_world_tilt_deg','map_floor_angle_deg','unknown_info_fraction','floor_info_fraction']:
        values = [r[k] for r in fs if k in r]
        f[k+'_median'] = float(np.median(values)) if values else None
    status = np.array([r['floor_status_counts'] for r in fs]).sum(axis=0)
    f['status_counts'] = status.tolist()
    f['status_fractions'] = (status/status.sum()).tolist()
    ratios = [r['coupled_information']['map']/r['coupled_information']['source'] for r in fs if r['coupled_information']['source']>0]
    f['same_usable_points_map_vs_source_schur_forward_ratio_median'] = float(np.median(ratios))
    windows.append(dict(start=lo,end=hi,scans=len(rows),groups=g,floor=f,
                        accepted_quality_counts=np.array([r['accepted_quality_counts'] for r in rows]).sum(axis=0).tolist(),
                        unconstrained_same_state_bax={k:sum(r['unconstrained_sensitivity'][k][13] for r in rows) for k in ['all','source_good','angle0_5']}))

# Inspect the actual terminal X-ICP categories, rather than infer them from rank.
sys.path.insert(0, str(Path('humble_ws/src/fast-livo2/scripts').resolve()))
from analyze_lio_diagnostics import Table
x = Table(p/'data/logs/xicp_diagnostic.csv')
t, first = np.unique(x['time'], return_index=True)
last = np.r_[first[1:]-1,len(x)-1]
axes = np.array([np.column_stack([x[f't{j}_axis_w{k}'][last] for k in 'xyz']) for j in range(3)]).transpose(1,0,2)
alignment = abs(np.einsum('nij,nj->ni',axes,d['Rp'][:,:,0]))
choice = alignment.argmax(axis=1)
categories = np.column_stack([x[f't{j}_category'][last] for j in range(3)])[np.arange(len(t)),choice]
for row in windows:
    m = (t>=row['start']) & (t<row['end'])
    row['xicp_forward_axis_category_counts'] = np.bincount(categories[m].astype(int),minlength=3).tolist()
i = np.argmin(abs(t-dyn['onset']['time']))
onset_xicp = {'rotation_categories':[int(x[f'r{j}_category'][last[i]]) for j in range(3)],
              'translation_categories':[int(x[f't{j}_category'][last[i]]) for j in range(3)],
              'translation_axis_alignment':alignment[i].tolist()}
sc = Table(p/'data/diagnostics/scans.csv',('status',))
f = Table(p/'data/diagnostics/geometry_states.csv')
assert np.max(abs(f.matrix('after_gravity',3,1)[...,0]-[0,0,-9.81])) < 1e-10
summary = dict(run='run-1790579862998301727-0',dynamics=dyn,
               geometry_windows=windows,xicp_onset=onset_xicp,
               max_point_gain_sum_error=groups['max_gain_error'],
               imu_comparison=read(p/'dynamics/imu_comparison.json'),
               capture=read(p/'data/capture.json'),capture_integrity=read(p/'data/summary.json'),
               source_identity=read(p/'data/source-identity.json'),
               scan_count=len(t),iteration_count=len(x),scan_gap_min_max=[float(np.diff(t).min()),float(np.diff(t).max())],
               source_normal_ms_quantiles=np.quantile(sc['source_normal_ms'],[.5,.95,1]).tolist(),
               floor_sample_scans=len(floor),
               notes=['Information is the weighted translation block projected onto horizontal estimated forward, not marginalized observability.',
                      'Group K*innovation sums are descriptive at the saved gain; excluding points changes that gain.',
                      'Counterfactuals keep state/prior/residuals fixed and omit X-ICP; they are not replay outcomes.',
                      'Floor fits use deskewed scans and actual map neighbors; neither is independent attitude ground truth.'])
for name, file in [('geometry_validation','geometry_validation/geometry_summary.json'),('filter_validation','filter_validation/summary.json')]:
    result=read(p/'out'/file)
    summary[name]={k:v for k,v in result.items() if k not in ['frames_detail','windows']}
assert summary['geometry_validation']['passed'] and summary['filter_validation']['passed']
(out/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')

plt.rcParams.update({'font.size':10,'axes.grid':True,'grid.alpha':.2})
fig, ax = plt.subplots(4,1,figsize=(11,10),sharex=True)
for dataset,label,color in [(baseline,'Previous capture: gate OFF','#777777'),(d,'New capture: gate ON','#176ab0')]:
    for a,k,scale in [(ax[0],'pitch',180/np.pi),(ax[1],'ba',1),(ax[2],'forward',1)]:
        y=dataset[k][:,0] if k=='ba' else dataset[k]
        a.plot(dataset['time'],y*scale,label=label,color=color,lw=1)
ax[0].set_ylabel('Estimated pitch [deg]');ax[0].set_ylim(8,28);ax[0].legend(loc='lower left')
ax[1].set_ylabel('Accel bias X [m/s²]')
ax[2].set_ylabel('Body forward speed [m/s]');ax[2].axhline(0,color='k',lw=.7)
bins=np.arange(140,366,1.)
means=[np.nanmean(d['aforward'][(t>=a)&(t<a+1)]) for a in bins[:-1]]
ax[3].plot(bins[:-1]+.5,means,color='#ba392e',label='IMU prediction: 1 s mean')
ax[3].set_ylabel('Forward acceleration [m/s²]');ax[3].axhline(0,color='k',lw=.7);ax[3].legend()
for a in ax:
    a.axvline(dyn['onset']['time'],color='#ba392e',ls='--',lw=1)
    a.axvspan(310,320,color='#efb64e',alpha=.15)
ax[-1].set_xlim(140,365.4);ax[-1].set_xlabel('Seconds since first LiDAR stamp')
fig.suptitle('Normal gate changed the drift, but backward divergence remained\nDashed: sustained reversal at 322.80 s; shaded: turn interval')
fig.tight_layout();fig.savefig(out/'state-comparison.png',dpi=160);plt.close(fig)

fs=sorted([r for r in floor if r.get('floor_count',0)>=150 and r.get('floor_rms',1)<.015 and 'map_floor_angle_deg' in r],key=lambda r:r['time'])
fig, ax = plt.subplots(3,1,figsize=(11,9),sharex=True)
for key,label,color in [('floor_world_tilt_deg','Scan floor vs world vertical','#176ab0'),('map_floor_world_tilt_deg','Map floor vs world vertical','#ba392e'),('map_floor_angle_deg','Scan floor vs map floor','#238b45')]:
    ax[0].plot([r['time'] for r in fs],[r[key] for r in fs],label=label,color=color,lw=1)
ax[0].set_ylim(0,10);ax[0].set_ylabel('Angle [deg]');ax[0].legend(loc='upper left')
ax[1].plot([r['time'] for r in fs],[r['floor_status_counts'][3]/sum(r['floor_status_counts']) for r in fs],color='#a05a2c',label='Floor points rejected at map-plane fit')
ax[1].set_ylabel('Rejected fraction');ax[1].set_ylim(0,1);ax[1].legend(loc='upper left')
rows=groups['rows'];gt=np.array([r['time'] for r in rows]);inf=np.array([r['groups']['all']['forward_information'] for r in rows]);bad=np.array([r['groups']['unavailable']['forward_information'] for r in rows]);fractions=[];mid=[]
for a in np.arange(180,330,1):
    m=(gt>=a)&(gt<a+1)
    fractions.append(bad[m].sum()/inf[m].sum());mid.append(a+.5)
ax[2].plot(mid,fractions,color='#793ca0',label='No usable source normal: share of forward information')
ax[2].set_ylim(0,1);ax[2].set_ylabel('Information fraction');ax[2].legend(loc='upper left')
for a in ax:a.axvspan(310,320,color='#efb64e',alpha=.15)
ax[-1].set_xlim(180,330);ax[-1].set_xlabel('Seconds since first LiDAR stamp')
fig.suptitle('Residual weaknesses after the normal gate\nFloor fit is a geometric consistency indicator, not attitude ground truth')
fig.tight_layout();fig.savefig(out/'remaining-geometry.png',dpi=160);plt.close(fig)
print('Wrote summary and figures:',out)
