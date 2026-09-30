from pathlib import Path
import sys,json,re,numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path('humble_ws/src/fast-livo2/scripts').resolve()))
from analyze_lio_diagnostics import Table
import argparse
ap=argparse.ArgumentParser();ap.add_argument('work',type=Path);args=ap.parse_args()
o=args.work;p=o/'data';out=o/'dynamics';out.mkdir(exist_ok=True)
x=Table(p/'logs/xicp_diagnostic.csv');sc=Table(p/'diagnostics/scans.csv',('status',));it=Table(p/'diagnostics/iterations.csv');f=Table(p/'diagnostics/geometry_states.csv')
t,first=np.unique(x['time'],return_index=True);last=np.r_[first[1:]-1,len(x)-1]
assert len(t)==len(sc) and np.max(abs(t-sc['time']))<1e-8
idx={(int(s),int(i)):j for j,(s,i) in enumerate(zip(it['scan_id'],it['iteration']))};sel=np.array([idx[(int(s),int(i))] for s,i in zip(sc['scan_id'],sc['committed_iteration'])])
xyz=lambda name,ix:np.column_stack([x[name+a][ix] for a in 'xyz'])
v=xyz('after_v',last);vp=xyz('prior_v',first);ba=xyz('after_ba_',last);bg=xyz('after_bg_',last)
pitch=x['after_pitch'][last];ppitch=x['prior_pitch'][first];forward=x['after_forward_speed'][last];pf=x['prior_forward_speed'][first]
mat=lambda name,n,m:it.matrix(name,n,m)[sel]
P,A=mat('P_iter',19,19),mat('HWH',6,6);corr=mat('Kinnovation',19,1)[...,0]
score=mat('HTWinnovation',6,1)[...,0];k0=(P[:,:,:6]@np.linalg.solve(np.eye(6)[None]+A@P[:,:6,:6],score[...,None]))[...,0]
M=np.linalg.solve(P[:,:6,:6],P[:,:6,7:16]).transpose(0,2,1);parts=M*corr[:,None,:6]
assert np.max(abs(parts.sum(axis=2)-corr[:,7:16]))<1e-9
assert np.max(abs(v-vp-corr[:,7:10]))<1e-9
pred=vp-np.vstack([vp[0],v[:-1]]);update=v-vp;dt=np.r_[np.nan,np.diff(t)]
raw=np.loadtxt(p/'logs/mat_out.txt');imu=np.loadtxt(p/'logs/imu.txt');pre=np.loadtxt(p/'logs/mat_pre.txt')
def align_rows(data):
 idx=np.clip(np.searchsorted(data[:,0],t),0,len(data)-1);prev=np.maximum(idx-1,0)
 use=abs(data[prev,0]-t)<abs(data[idx,0]-t);idx[use]=prev[use]
 assert np.max(abs(data[idx,0]-t))<.0006
 return data[idx]
raw=align_rows(raw);pre=align_rows(pre)
R=Rotation.from_euler('xyz',raw[:,1:4]/57.3).as_matrix();Rp=Rotation.from_euler('xyz',pre[:,1:4]/57.3).as_matrix()
scanlookup={int(v):i for i,v in enumerate(sc['scan_id'])};fi=np.array([scanlookup[int(v)] for v in f['scan_id']]);assert np.max(abs(t[fi]-f['time']))<1e-8
R[fi]=f.matrix('after_R',3,3);Rp[fi]=f.matrix('prior_R',3,3)
aforward=np.einsum('ni,ni->n',pred,Rp[:,:,0])/dt
uforward=np.einsum('ni,ni->n',update,Rp[:,:,0]);k0forward=np.einsum('ni,ni->n',k0[:,7:10],Rp[:,:,0])
parts_forward=np.einsum('nij,ni->nj',parts[:,:3,:],Rp[:,:,0])
pitchpred=np.r_[0,ppitch[1:]-pitch[:-1]];pitchup=pitch-ppitch
std=np.sqrt(np.diagonal(sc.matrix('P_after',19,19),axis1=1,axis2=2))
C=mat('C',6,19);ranks=it['constraints'][sel]
# For C_pose, horizontal translation along IMU forward (unit tangent input).
constraint_alignment=np.linalg.norm(C[:,:,3:6]@Rp[:,:,0,None],axis=(1,2))
np.savez_compressed(out/'dynamics.npz',time=t,scan_id=sc['scan_id'],R=R,Rp=Rp,velocity=v,prior_velocity=vp,ba=ba,bg=bg,pitch=pitch,forward=forward,prior_forward=pf,prediction_dv=pred,update_dv=update,correction=corr,K0innovation=k0,P=P,HWH=A,M=M,std=std,aforward=aforward,uforward=uforward,k0forward=k0forward,constraints=ranks,constraint_alignment=constraint_alignment)
windows=[]
for lo,hi in [(3,20),(20,60),(60,100),(100,140),(140,180),(180,200),(200,220),(220,260),(260,300),(300,310),(310,315),(315,320),(320,322.8),(322.8,325),(325,330),(330,350),(350,360),(180,310)]:
 mask=(t>=lo)&(t<hi);i=np.flatnonzero(mask)
 im=(imu[:,0]>=lo)&(imu[:,0]<hi);si=np.clip(np.searchsorted(t,imu[im,0]),0,len(t)-1)
 a=imu[im,4:7]*9.81/.9997
 no_bias=np.einsum('nij,nj->ni',R[si],a)+[0,0,-9.81];bias=-np.einsum('nij,nj->ni',R[si],ba[si])
 row={'start':lo,'end':hi,'scans':len(i),'pitch_mean_deg':float(np.rad2deg(pitch[mask]).mean()),'pitch_imu_deg':float(np.rad2deg(pitchpred[mask].sum())),'pitch_update_deg':float(np.rad2deg(pitchup[mask].sum())),
 'bax_change':float(corr[mask,13].sum()),'ba_mean':ba[mask].mean(axis=0).tolist(),'ba_from_rotation':parts[mask,6:9,:3].sum(axis=(0,2)).tolist(),'ba_from_position':parts[mask,6:9,3:].sum(axis=(0,2)).tolist(),
 'bg_mean':bg[mask].mean(axis=0).tolist(),'forward_end':float(forward[i[-1]]),'prior_accel_forward_mean':float(np.nanmean(aforward[mask])),
 'prediction_dv':pred[mask].sum(axis=0).tolist(),'lidar_dv':update[mask].sum(axis=0).tolist(),
 'forward_lidar_sum':float(uforward[mask].sum()),'forward_unconstrained_sum':float(k0forward[mask].sum()),
 'constraint_alignment_mean':float(constraint_alignment[mask].mean()),'constraint_free_fraction':float((ranks[mask]==0).mean()),
 'residual_rms_median':float(np.median(it['residual_rms'][sel][mask])),'matches_median':float(np.median(it['matches'][sel][mask])),
 'source_valid_fraction':float(sc['source_normal_valid'][mask].sum()/(sc['source_normal_valid'][mask]+sc['source_normal_invalid'][mask]).sum()),
 'gate_rejected_fraction':float(sc['normal_rejected'][mask].sum()/sc['normal_comparisons'][mask].sum()),
 'posterior_std_median':np.median(std[mask],axis=0).tolist(),
 'accel_no_bias_forward_mean':float(np.einsum('ni,ni->n',no_bias,R[si,:,0]).mean()),'accel_bias_forward_mean':float(np.einsum('ni,ni->n',bias,R[si,:,0]).mean())}
 windows.append(row)
 print(lo,hi,'pitch',round(row['pitch_mean_deg'],3),'dPitchImu/lio',np.round([row['pitch_imu_deg'],row['pitch_update_deg']],4),'dBaX',round(row['bax_change'],4),'Afwd',round(row['prior_accel_forward_mean'],3),'LIO/K0',np.round([row['forward_lidar_sum'],row['forward_unconstrained_sum']],3),'Calign',round(row['constraint_alignment_mean'],3),flush=True)
onset=next(i for i in range(len(t)-19) if t[i]>20 and (forward[i:i+20]<0).all())
onsetrow={k:float(val[onset]) for k,val in dict(time=t,forward=forward,prior_forward=pf,prior_accel_forward=aforward,update_forward=uforward,unconstrained_forward=k0forward,constraints=ranks,constraint_alignment=constraint_alignment).items()}
onsetrow['previous_forward']=float(forward[onset-1]);onsetrow['ba']=ba[onset].tolist();onsetrow['pitch_deg']=float(np.rad2deg(pitch[onset]))
summary={'onset':onsetrow,'min_forward':float(forward.min()),'min_forward_time':float(t[forward.argmin()]),'final_forward':float(forward[-1]),'last_time':float(t[-1]),'windows':windows,
'identity_error':float(np.max(abs(parts.sum(axis=2)-corr[:,7:16]))),'covariance_gain_checked':True}
(out/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n');print('onset',onsetrow)
# Compare logged IMU inputs in overlapping baseline section; match by timestamp.
old=np.loadtxt('diagnostic_captures/lio-geometry-ehdz2pcz/capture/imu.txt')
a=imu[(imu[:,0]>=140)&(imu[:,0]<325)];b=old[(old[:,0]>=140)&(old[:,0]<325)]
jj=np.clip(np.searchsorted(b[:,0],a[:,0]),0,len(b)-1);jm=np.maximum(jj-1,0);choose=abs(b[jm,0]-a[:,0])<abs(b[jj,0]-a[:,0]);jj[choose]=jm[choose]
comp={'new_rows':len(a),'old_rows':len(b),'timestamp_absdiff_quantiles':np.quantile(abs(a[:,0]-b[jj,0]),[0,.5,.99,1]).tolist(),'exact_vector_fraction':float(np.all(a[:,1:]==b[jj,1:],axis=1).mean()),'near_vector_fraction':float(np.all(abs(a[:,1:]-b[jj,1:])<1e-5,axis=1).mean())}
(out/'imu_comparison.json').write_text(json.dumps(comp,indent=2)+'\n');print('imu match',comp)
