"""Analyze the preserved completed-scan snapshot; no estimator changes or replay."""
from pathlib import Path
import argparse, json, re, sys
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'humble_ws/src/fast-livo2/scripts'))
from analyze_lio_diagnostics import Table, analyze

ap = argparse.ArgumentParser(); ap.add_argument('work', type=Path); args = ap.parse_args()
work = args.work; data = work/'data'; run = data/'diagnostics'
out = work/'dynamics'; out.mkdir(exist_ok=True)
sc = Table(run/'scans.csv', ('status',)); it = Table(run/'iterations.csv')
f = Table(run/'geometry_states.csv'); x = Table(data/'logs/xicp_diagnostic.csv')
assert np.all(sc['status']=='updated')
assert np.array_equal(sc['scan_id'], f['scan_id']) and np.array_equal(sc['time'], f['time'])
t = sc['time']; n = len(t)
lookup = {(int(s),int(i)): j for j,(s,i) in enumerate(zip(it['scan_id'],it['iteration']))}
sel = np.array([lookup[(int(s),int(i))] for s,i in zip(sc['scan_id'],sc['committed_iteration'])])
xi = np.r_[np.flatnonzero(np.diff(x['time'])),len(x)-1]
assert len(xi)==len(t) and np.max(abs(x['time'][xi]-t))<1e-8
assert np.array_equal(x['iteration'][xi],sc['committed_iteration'])
mat = lambda name,r,c: it.matrix(name,r,c)[sel]
R, Rp = f.matrix('after_R',3,3), f.matrix('prior_R',3,3)
v, vp = f.matrix('after_velocity',3,1)[...,0], f.matrix('prior_velocity',3,1)[...,0]
ba, bg = f.matrix('after_ba',3,1)[...,0], f.matrix('after_bg',3,1)[...,0]
pitch = np.arctan2(-R[:,2,0],np.hypot(R[:,0,0],R[:,1,0]))
ppitch = np.arctan2(-Rp[:,2,0],np.hypot(Rp[:,0,0],Rp[:,1,0]))
forward = np.einsum('ni,ni->n',v,R[:,:,0]); pf = np.einsum('ni,ni->n',vp,Rp[:,:,0])
pred = vp-np.vstack([vp[0],v[:-1]]); update = v-vp; dt=np.r_[np.nan,np.diff(t)]
af = np.einsum('ni,ni->n',pred,Rp[:,:,0])/dt
uf = np.einsum('ni,ni->n',update,Rp[:,:,0])
P, A = mat('P_iter',19,19),mat('HWH',6,6)
score, corr = mat('HTWinnovation',6,1)[...,0],mat('Kinnovation',19,1)[...,0]
k0=(P[:,:,:6]@np.linalg.solve(np.eye(6)[None]+A@P[:,:6,:6],score[...,None]))[...,0]
k0f=np.einsum('ni,ni->n',k0[:,7:10],Rp[:,:,0])
M=np.linalg.solve(P[:,:6,:6],P[:,:6,7:16]).transpose(0,2,1)
parts=M*corr[:,None,:6]
identity=float(np.max(abs(parts.sum(axis=2)-corr[:,7:16])))
assert identity<1e-8 and np.max(abs(update-corr[:,7:10]))<1e-8
C=mat('C',6,19); ca=np.linalg.norm(C[:,:,3:6]@Rp[:,:,0,None],axis=(1,2))
std=np.sqrt(np.diagonal(sc.matrix('P_after',19,19),axis1=1,axis2=2))
dpred=np.r_[0,ppitch[1:]-pitch[:-1]]; dup=pitch-ppitch
imu=np.loadtxt(data/'logs/imu.txt')
imu_ref=np.loadtxt(ROOT/'diagnostic_captures/normal-gate-analysis-20260928/data/logs/imu.txt')
imax=min(imu[-1,0],imu_ref[-1,0]); ia=imu[imu[:,0]<=imax]; ib=imu_ref[(imu_ref[:,0]>=ia[0,0])&(imu_ref[:,0]<=imax)]
imu_check=dict(rows=len(ia),reference_rows=len(ib),exact_all_columns=bool(ia.shape==ib.shape and np.array_equal(ia,ib)),
               start=float(ia[0,0]),end=float(ia[-1,0]),max_gap_sec=float(np.diff(ia[:,0]).max()))
categories=np.column_stack([x[f'{g}{j}_category'][xi] for g in ['r','t'] for j in range(3)])
world_axes=np.stack([np.column_stack([x[f't{j}_axis_w{a}'][xi] for a in 'xyz']) for j in range(3)],axis=1)
axes_dot=abs(np.einsum('nji,ni->nj',world_axes,Rp[:,:,0]))
nearaxis=axes_dot.argmax(axis=1); nearcat=categories[np.arange(n),nearaxis+3]
onset=next(i for i in range(n-19) if t[i]>20 and (forward[i:i+20]<0).all())
windows=[]
for lo,hi in [(3.5,20),(20,60),(60,100),(100,140),(140,180),(180,200),(200,220),(220,260),(260,300),(300,310),(310,315),(315,320),(320,float(t[onset])),(float(t[onset]),325),(325,330),(330,336),(180,310)]:
 z=(t>=lo)&(t<hi); ix=np.flatnonzero(z)
 if not len(ix):continue
 im=(imu[:,0]>=lo)&(imu[:,0]<hi); si=np.clip(np.searchsorted(t,imu[im,0]),0,n-1)
 a=imu[im,4:7]*9.81/.9997
 g=f.matrix('after_gravity',3,1)[...,0]
 no_bias=np.einsum('nij,nj->ni',R[si],a)+g[si];bias=-np.einsum('nij,nj->ni',R[si],ba[si])
 r=dict(start=lo,end=hi,scans=len(ix),pitch_mean_deg=float(np.rad2deg(pitch[z]).mean()),
        pitch_imu_deg=float(np.rad2deg(dpred[z].sum())),pitch_lidar_deg=float(np.rad2deg(dup[z].sum())),
        ba_mean=ba[z].mean(axis=0).tolist(),ba_delta=corr[z,13:16].sum(axis=0).tolist(),
        ba_via_rotation=parts[z,6:9,:3].sum(axis=(0,2)).tolist(),ba_via_position=parts[z,6:9,3:].sum(axis=(0,2)).tolist(),
        bg_mean=bg[z].mean(axis=0).tolist(),forward_end=float(forward[ix[-1]]),
        prior_accel_forward_mean=float(np.nanmean(af[z])),pred_dv_world=pred[z].sum(axis=0).tolist(),
        lidar_dv_world=update[z].sum(axis=0).tolist(),forward_prediction_sum=float(np.sum(af[z]*dt[z])),
        forward_lidar_sum=float(uf[z].sum()),forward_unconstrained_sum=float(k0f[z].sum()),
        accel_no_bias_forward_mean=float(np.einsum('ni,ni->n',no_bias,R[si,:,0]).mean()),
        accel_bias_forward_mean=float(np.einsum('ni,ni->n',bias,R[si,:,0]).mean()),
        constraint_free_fraction=float((it['constraints'][sel][z]==0).mean()),
        forward_category_fractions=[float((nearcat[z]==k).mean()) for k in range(3)],
        residual_rms_median=float(np.median(it['residual_rms'][sel][z])),matches_median=float(np.median(it['matches'][sel][z])),
        geometry_logging_ms_median=float(np.median(sc['geometry_logging_ms'][z])),posterior_std_median=np.median(std[z],axis=0).tolist())
 windows.append(r)
 print(lo,hi,'dPitch IMU/LIO',np.round([r['pitch_imu_deg'],r['pitch_lidar_deg']],4),'dBaX',round(r['ba_delta'][0],4),
       'aF',round(r['prior_accel_forward_mean'],4),'LIO/K0',np.round([r['forward_lidar_sum'],r['forward_unconstrained_sum']],4),
       'NONE/PART/FULL',np.round(r['forward_category_fractions'],3),flush=True)
on=dict(time=float(t[onset]),previous_forward=float(forward[onset-1]),prior_forward=float(pf[onset]),forward=float(forward[onset]),
        prior_accel_forward=float(af[onset]),lidar_forward=float(uf[onset]),unconstrained_forward=float(k0f[onset]),
        pitch_deg=float(np.rad2deg(pitch[onset])),ba=ba[onset].tolist(),categories=categories[onset].tolist(),
        forward_axis_alignment=float(axes_dot[onset,nearaxis[onset]]),constraint_alignment=float(ca[onset]))
summary=dict(onset=on,last_time=float(t[-1]),scans=n,iterations=len(it),imu_comparison=imu_check,
             max_identity_error=identity,min_forward=float(forward.min()),windows=windows,
             caveats=['Same-state unconstrained gain is not a replay.','Acceleration split uses scan-end pose and rounded IMU log.',
                      'Forward speed uses estimated IMU X axis; no ground-truth speed or bias is available.'])
(out/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
np.savez_compressed(out/'dynamics.npz',time=t,scan_id=sc['scan_id'],R=R,Rp=Rp,velocity=v,prior_velocity=vp,ba=ba,bg=bg,
                    pitch=pitch,forward=forward,prior_forward=pf,prediction_dv=pred,update_dv=update,correction=corr,
                    K0innovation=k0,P=P,HWH=A,M=M,std=std,aforward=af,uforward=uf,k0forward=k0f,
                    constraints=it['constraints'][sel],constraint_alignment=ca,forward_category=nearcat,
                    matches=it['matches'][sel],residual_rms=it['residual_rms'][sel])
# Validate every aggregate row. Detailed point files are validated separately on the documented sample.
check=work/'aggregate-input';check.mkdir(exist_ok=True)
for name in ['iterations.csv','scans.csv']:
 target=check/name
 if not target.exists():target.symlink_to((run/name).resolve())
meta=json.loads((run/'metadata.json').read_text());meta['detail_enabled']=False
meta['analysis_note']='Aggregate-only validation copy; original metadata remains unchanged.'
(check/'metadata.json').write_text(json.dumps(meta,indent=2)+'\n')
assert analyze(check,work/'aggregate-validation')
print('ONSET',on);print('IMU',imu_check)
