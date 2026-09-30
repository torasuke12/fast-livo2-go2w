"""Summarize the stopped second-U-turn capture without replaying or modifying logs."""
from pathlib import Path
import argparse, importlib.util, json
import numpy as np

p=argparse.ArgumentParser()
p.add_argument('snapshot',type=Path)
p.add_argument('--output',type=Path,required=True)
p.add_argument('--reference',type=Path,help='Optional prior replay directory containing imu.txt')
a=p.parse_args(); a.output.mkdir(parents=True,exist_ok=True)
spec=importlib.util.spec_from_file_location('diagnostics','humble_ws/src/fast-livo2/scripts/analyze_lio_diagnostics.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
it=m.Table(a.snapshot/'diagnostics/iterations.csv'); scans=m.Table(a.snapshot/'diagnostics/scans.csv',('status',))
x=m.Table(a.snapshot/'logs/xicp_diagnostic.csv')
state=np.loadtxt(a.snapshot/'logs/mat_out.txt'); rawpre=np.loadtxt(a.snapshot/'logs/mat_pre.txt')
imu=np.loadtxt(a.snapshot/'logs/imu.txt')
time,first=np.unique(x['time'],return_index=True); last=np.r_[first[1:]-1,len(x)-1]
t=scans['time']; n=len(t)
assert len(time)==n==len(state)
assert np.max(abs(t-time))<1e-8 and np.max(abs(t-state[:,0]))<6e-4
keys={(int(s),int(i)):j for j,(s,i) in enumerate(zip(it['scan_id'],it['iteration']))}
selected=np.array([keys[(int(s),int(i))] for s,i in zip(scans['scan_id'],scans['committed_iteration'])])
assert (scans['status']=='updated').all() and (it['final'][selected]==1).all()
assert np.max(abs(it['time'][selected]-t))==0

def stat(v):
 return {'min':float(np.min(v)),'median':float(np.median(v)),'p95':float(np.percentile(v,95)),'max':float(np.max(v))}
def matrix(name,rows,cols):return it.matrix(name,rows,cols)[selected]
def xyz(prefix,indices):return np.column_stack([x[prefix+axis][indices] for axis in 'xyz'])
postv=xyz('after_v',last); prev=xyz('prior_v',first)
postba=xyz('after_ba_',last); preba=xyz('prior_ba_',first)
postbg=xyz('after_bg_',last); prebg=xyz('prior_bg_',first)
pitch=x['after_pitch'][last]; prepitch=x['prior_pitch'][first]
forward=x['after_forward_speed'][last]; preforward=x['prior_forward_speed'][first]
dt=np.r_[np.nan,np.diff(t)]
predict=np.vstack([np.zeros(3),prev[1:]-postv[:-1]])
correct=postv-prev
pitchpredict=np.r_[0,prepitch[1:]-pitch[:-1]]; pitchcorrect=pitch-prepitch
forwardpredict=np.r_[0,preforward[1:]-forward[:-1]]; forwardcorrect=forward-preforward
P=matrix('P_iter',19,19); A=matrix('HWH',6,6); KH=matrix('KH',19,6)
score=matrix('HTWinnovation',6,1)[...,0]
Kinnovation=matrix('Kinnovation',19,1)[...,0]
unconstrained=(P[:,:,:6]@np.linalg.solve(np.eye(6)[None]+A@P[:,:6,:6],score[...,None]))[...,0]
diagonal=np.diagonal(A,axis1=1,axis2=2)
scale=np.divide(1,np.sqrt(np.maximum(diagonal,0)),out=np.zeros_like(diagonal),where=diagonal>0)
normalized=A*scale[:,:,None]*scale[:,None,:]
eigenvalues=np.linalg.eigvalsh(normalized)
prior=scans.matrix('P_prior',19,19); after=scans.matrix('P_after',19,19)
std=np.sqrt(np.maximum(np.diagonal(after,axis1=1,axis2=2),0))
priorstd=np.sqrt(np.maximum(np.diagonal(prior,axis1=1,axis2=2),0))
nis=it['innovation_wrss'][selected]-np.sum(score*unconstrained[:,:6],axis=1)
bgpred=np.vstack([np.zeros(3),prebg[1:]-postbg[:-1]])
bapred=np.vstack([np.zeros(3),preba[1:]-postba[:-1]])
# Each logged local IMU interval is a pre-bias two-sample average, not a raw packet.
imu_reference=a.reference/'imu.txt' if a.reference else None
old=np.loadtxt(imu_reference) if imu_reference else None
summary={'snapshot':str(a.snapshot),'state_count':n,'first_time':t[0],'last_time':t[-1],
         'nonfinite_state_values':int(np.sum(~np.isfinite(state))),
         'processed_imu_rows':len(imu),'imu_interval_max_sec_rounded':float(np.diff(imu[:,0]).max()),
         'imu_intervals_over_10ms':int(np.sum(np.diff(imu[:,0])>.01)),
         'imu_prefix_equal_to_prior_replay':bool(np.array_equal(imu,old[:len(imu)])) if old is not None else None,
         'imu_reference':str(imu_reference) if imu_reference else None,
         'max_prediction_ba_change':float(np.max(abs(bapred))),
         'max_prediction_bg_change':float(np.max(abs(bgpred))),
         'velocity_contribution_vs_state_change_max_error':float(np.max(abs(correct-Kinnovation[:,7:10]))),
         'bias_contribution_vs_state_change_max_error':float(np.max(abs(postba-preba-Kinnovation[:,13:16])))}
backward=[i for i in range(n-19) if t[i]>300 and np.all(forward[i:i+20]<0)]
summary['sustained_reverse_start_sec']=float(t[backward[0]]) if backward else None
if backward:
 i=backward[0];summary['reversal_scan']={'time':t[i], 'previous_forward':forward[i-1], 'predicted_forward':preforward[i], 'updated_forward':forward[i], 'predicted_v':prev[i].tolist(),'updated_v':postv[i].tolist()}
summary['checkpoints']=[]
for target in [180,200,220,280,300,310,315,318,320,325,327,337,t[-1]]:
 i=np.argmin(abs(t-target))
 summary['checkpoints'].append({'time':t[i],'forward_mps':forward[i],'velocity_world':postv[i].tolist(),
     'pitch_deg':float(np.rad2deg(pitch[i])),'bias_accel':postba[i].tolist(),'bias_accel_norm':float(np.linalg.norm(postba[i])),
     'world_position':state[i,4:7].tolist(),'residual_rms':float(it['residual_rms'][selected[i]]),
     'matches':int(it['matches'][selected[i]]),'hessian_min_normalized_eigenvalue':eigenvalues[i,0],
     'posterior_velocity_std':std[i,7:10].tolist(),'posterior_ba_std':std[i,13:16].tolist()})
windows=[]
for lo,hi in [(180,200),(200,220),(220,280),(280,300),(300,310),(310,315),(315,320),(320,325),(325,330),(330,337),(337,349),(180,310)]:
 mask=(t>=lo)&(t<hi)
 if not mask.any(): continue
 ids=np.flatnonzero(mask); q=selected[mask]; xo=last[mask]
 rot_cats=np.column_stack([x[f'r{k}_category'][xo] for k in range(3)])
 trans_cats=np.column_stack([x[f't{k}_category'][xo] for k in range(3)])
 trans_axes=np.stack([np.column_stack([x[f't{k}_axis_w{axis}'][xo] for axis in 'xyz']) for k in range(3)],axis=1)
 # World X is near the forward heading after this turn; before turn its sign is reversed.
 closest=np.argmax(abs(trans_axes[:,:,0]),axis=1)
 cats=trans_cats[np.arange(len(ids)),closest]
 row={'start':lo,'end':hi,'recorded_first':t[ids[0]],'recorded_last':t[ids[-1]],'scans':len(ids),
      'forward_first':forward[ids[0]],'forward_last':forward[ids[-1]],
      'prediction_dv_world_sum':predict[mask].sum(axis=0).tolist(),'lidar_dv_world_sum':correct[mask].sum(axis=0).tolist(),
      'prediction_dforward_sum_including_rotation':float(forwardpredict[mask].sum()),'lidar_dforward_sum_including_rotation':float(forwardcorrect[mask].sum()),
      'prediction_pitch_deg_sum':float(np.rad2deg(pitchpredict[mask].sum())),'lidar_pitch_deg_sum':float(np.rad2deg(pitchcorrect[mask].sum())),
      'lidar_ba_sum':(postba-preba)[mask].sum(axis=0).tolist(),
      'Kinnovation_sum':Kinnovation[mask].sum(axis=0).tolist(),'K0innovation_sum':unconstrained[mask].sum(axis=0).tolist(),
      'residual_rms':stat(it['residual_rms'][q]),'innovation_rms':stat(it['innovation_rms'][q]),
      'matches':stat(it['matches'][q]),'normalized_hessian_min_eigenvalue':stat(eigenvalues[mask,0]),
      'normalized_hessian_condition':stat(eigenvalues[mask,-1]/eigenvalues[mask,0]),
      'nis_per_match':stat(nis[mask]/it['matches'][q]),
      'posterior_std_median':np.median(std[mask],axis=0).tolist(),'prior_std_median':np.median(priorstd[mask],axis=0).tolist(),
      'gain_row_norm_median':np.median(matrix('gain_norm',19,1)[mask,:,0],axis=0).tolist(),
      'KH_diagonal_pose_median':np.median(np.diagonal(KH[mask,:6,:],axis1=1,axis2=2),axis=0).tolist(),
      'all_rotation_full_scans':int(np.sum((rot_cats==2).all(axis=1))),
      'world_x_closest_translation_categories':dict(zip(*[v.astype(int).tolist() for v in np.unique(cats,return_counts=True)])),
      'constraint_rows':stat(it['constraints'][q])}
 windows.append(row)
summary['windows']=windows
summary['last_state_posterior_std']=std[-1].tolist()
# Approximate inertial decomposition, using interval-average accelerometer data,
# posterior orientation at the next scan, and the same scale as the prior replay.
# This uses estimated orientation, so it is not an attitude ground truth.
def rotations(rpy):
 r,p,y=rpy.T
 cr,sr,cp,sp,cy,sy=np.cos(r),np.sin(r),np.cos(p),np.sin(p),np.cos(y),np.sin(y)
 R=np.empty((len(r),3,3))
 R[:,0,0]=cy*cp; R[:,0,1]=cy*sp*sr-sy*cr; R[:,0,2]=cy*sp*cr+sy*sr
 R[:,1,0]=sy*cp; R[:,1,1]=sy*sp*sr+cy*cr; R[:,1,2]=sy*sp*cr-cy*sr
 R[:,2,0]=-sp; R[:,2,1]=cp*sr; R[:,2,2]=cp*cr
 return R
R=rotations(state[:,1:4]/57.3)
summary['inertial_approximation']=[]
for lo,hi in [(280,300),(300,310),(315,320),(320,325),(325,330),(330,337),(337,349)]:
 mask=(t>=lo)&(t<hi); ids=np.flatnonzero((imu[:,0]>=lo)&(imu[:,0]<hi))
 si=np.clip(np.searchsorted(t,imu[ids,0]),0,n-1)
 acceleration=np.einsum('nij,nj->ni',R[si],imu[ids,4:7]*9.81/0.9997)+[0,0,-9.81]
 bias_term=-np.einsum('nij,nj->ni',R[si],postba[si])
 predicted=np.mean(predict[mask]/dt[mask,None],axis=0)
 total=(acceleration+bias_term).mean(axis=0)
 summary['inertial_approximation'].append({'start':lo,'end':hi,
    'accel_without_bias_plus_gravity_world_mean':acceleration.mean(axis=0).tolist(),
    'bias_world_term_mean':bias_term.mean(axis=0).tolist(),
    'reconstructed_accel_mean':total.tolist(),'actual_prediction_accel_mean':predicted.tolist(),
    'approximation_error':(total-predicted).tolist()})
summary['inertial_approximation_assumptions']=['a_world = R*(a_interval_mean*9.81/0.9997 - ba) + [0,0,-9.81]',
 'Mean-acceleration norm 0.9997 from the prior replay initialization; same IMU sequence and initial state.',
 'Next-scan posterior orientation/bias used, not every intermediate propagated orientation.']
(a.output/'uturn_summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axes=plt.subplots(4,1,figsize=(12,10),sharex=True)
show=(t>=280)
axes[0].plot(t[show],forward[show],label='posterior forward velocity')
axes[0].plot(t[show],preforward[show],label='predicted forward velocity',alpha=.6)
axes[0].axhline(0,color='black',lw=.7);axes[0].set_ylabel('Body-X velocity [m/s]')
axes[1].plot(t[show],np.cumsum(predict[show,0]),label='IMU prediction cumulative dvX')
axes[1].plot(t[show],np.cumsum(correct[show,0]),label='LiDAR cumulative dvX')
axes[1].plot(t[show],np.cumsum(predict[show,0]+correct[show,0]),label='net cumulative dvX')
axes[1].set_ylabel('World-X delta-v [m/s]')
axes[2].plot(t[show],it['residual_rms'][selected][show],label='residual RMS')
axes[2].plot(t[show],it['innovation_rms'][selected][show],label='innovation RMS');axes[2].set_ylabel('RMS [m]')
axes[3].plot(t[show],eigenvalues[show,0],label='smallest normalized eigenvalue')
axes[3].set_ylabel('Normalized HWH eigenvalue');axes[3].set_xlabel('Node-relative time [s]')
for ax in axes:
 ax.axvspan(310,317,color='gray',alpha=.12,label='estimated heading turn')
 if backward:ax.axvline(t[backward[0]],color='red',ls='--',lw=1,label='sustained reverse onset')
 ax.grid(alpha=.25); ax.legend(fontsize=8)
fig.tight_layout();fig.savefig(a.output/'uturn_dynamics.png',dpi=160);plt.close(fig)
fig,axes=plt.subplots(3,1,figsize=(12,9),sharex=True)
for j,axis in enumerate('xyz'):
 axes[0].plot(t[show],postba[show,j],label=f'estimated accel bias {axis}')
 axes[1].plot(t[show],std[show,13+j],label=f'posterior bias std {axis}')
 axes[2].plot(t[show],Kinnovation[show,13+j],label=f'actual ba{axis} correction')
 axes[2].plot(t[show],unconstrained[show,13+j],ls=':',label=f'same-state unconstrained ba{axis}')
axes[0].set_ylabel('Accel bias [m/s²]');axes[1].set_ylabel('Accel bias std [m/s²]');axes[2].set_ylabel('Correction [m/s²]')
axes[2].set_xlabel('Node-relative time [s]')
for ax in axes:ax.grid(alpha=.25);ax.legend(fontsize=8)
fig.tight_layout();fig.savefig(a.output/'uturn_bias.png',dpi=160);plt.close(fig)
print('SUMMARY',a.output/'uturn_summary.json')
print('reverse',summary['sustained_reverse_start_sec'],'last forward',forward[-1])
for w in windows:
 if w['start']<300:continue
 print(w['start'],w['end'],'dvX IMU/LIO',w['prediction_dv_world_sum'][0],w['lidar_dv_world_sum'][0],
       'RMS',w['residual_rms']['median'],'hess min',w['normalized_hessian_min_eigenvalue']['median'],
       'ba std',w['posterior_std_median'][13:16],'cats',w['world_x_closest_translation_categories'])
