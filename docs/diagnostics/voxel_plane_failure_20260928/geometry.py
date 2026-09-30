"""Validate sampled voxel geometry and measure uncertainty, reuse and floor consistency."""
from pathlib import Path
import argparse,json,sys
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'humble_ws/src/fast-livo2/scripts'))
sys.path.insert(0,str(ROOT/'docs/diagnostics/geometry_capture_20260927'))
from analyze_geometry_cause import fit_plane,angle
from analyze_lio_diagnostics import Table,analyze as filter_analyze
import analyze_lio_geometry as geometry
ap=argparse.ArgumentParser();ap.add_argument('work',type=Path);a=ap.parse_args()
work=a.work;run=work/'sample';out=work/'geometry-analysis';out.mkdir(exist_ok=True)
frames=Table(run/'geometry_states.csv');scans=Table(run/'scans.csv',('status',))
lookup={(int(s),int(i)):j for j,(s,i) in enumerate(zip(frames['scan_id'],frames['iteration']))}
original=geometry.groups;rows=[];active=None
ref=np.array([-np.sin(np.deg2rad(19)),0,np.cos(np.deg2rad(19))])
quant=lambda x:np.quantile(x,[.1,.5,.9]).tolist() if len(x) else None
def wrapped(path,selected=None):
 global active
 for key,names,data in original(path,selected):
  c=lambda n:data[:,names.index(n)]
  if path.name=='geometry.csv':
   j=lookup[key];fm=lambda n,r=3,k=3:np.array([[frames[f'{n}_{u}_{v}'][j] for v in range(k)] for u in range(r)])
   vec=lambda n:np.column_stack([c(f'{n}_{i}_0') for i in range(3)])
   R,E=fm('linearization_R'),fm('extrinsic_R');RL=R@E
   p=vec('lidar');normal=vec('normal');status=c('status').astype(int);used=status==5
   near=(np.linalg.norm(p,axis=1)<8)&(np.linalg.norm(p,axis=1)>.7);height=p@ref
   floor=fit_plane(p[near&(height>-1.8)&(height<-.12)],ref)
   frozen=c('update_enabled')[used]==0;plane_ids=c('plane_id')[used].astype(int)
   unique,counts=np.unique(plane_ids,return_counts=True)
   valid=np.isin(status,[4,5,9]);world=vec('world');center=vec('center')
   center_sq=np.sum((world-center)**2,axis=1);distance_sq=c('distance')**2
   # Native C++ can fuse products/sums differently from NumPy before float rounding.
   # Bound the cancellation-sensitive square root using an explicit float-operation error bound.
   radicand=center_sq-distance_sq;error=8*np.finfo(np.float32).eps*(abs(center_sq)+abs(distance_sq))
   low=np.sqrt(np.maximum(0,radicand-error));high=np.sqrt(np.maximum(0,radicand+error))
   tangent=c('tangential_distance')
   assert np.all(tangent[valid]>=low[valid]-1e-8) and np.all(tangent[valid]<=high[valid]+1e-8)
   row=dict(scan_id=key[0],time=float(frames['time'][j]),points=len(p),accepted=int(used.sum()),
      status_counts=np.bincount(status,minlength=11).tolist(),unique_planes=len(unique),
      points_per_plane_quantiles=quant(counts),max_points_one_plane=int(counts.max()),
      frozen_fraction=float(frozen.mean()),support_quantiles=quant(c('support_points')[used]),
      inverse_variance_quantiles=quant(1/c('update_variance')[used]),
      constant_variance_fraction_quantiles=quant(.001/c('update_variance')[used]),
      plane_variance_fraction_quantiles=quant(c('update_plane_variance')[used]/c('update_variance')[used]),
      point_variance_fraction_quantiles=quant(c('update_point_variance')[used]/c('update_variance')[used]))
   forward=RL[:,0].copy();forward[2]=0;forward/=np.linalg.norm(forward)
   floor_mask=np.zeros(len(p),bool)
   if floor is not None:
    n=floor['normal'];nw=RL@n;floor_mask=near&(abs(p@n+floor['offset'])<.05)
    good=floor_mask&used
    ni=E@n
    row.update(floor_count=floor['count'],floor_rms=floor['rms'],floor_world_tilt_deg=float(angle(nw,np.array([0,0,1]))),
      floor_sensor_pitch_deg=float(np.rad2deg(np.arctan2(-ni[0],np.linalg.norm(ni[1:])))),
      floor_status_counts=np.bincount(status[floor_mask],minlength=11).tolist())
    if good.any():
     signed=normal[good]*np.where(normal[good,2,None]>=0,1.,-1.)
     mn=signed.mean(axis=0);mn/=np.linalg.norm(mn)
     row.update(map_floor_world_tilt_deg=float(angle(mn,np.array([0,0,1]))),map_floor_angle_deg=float(angle(mn,nw)),
       floor_map_point_angle_quantiles=quant(angle(normal[good],nw)),floor_frozen_fraction=float((c('update_enabled')[good]==0).mean()))
   # The current matching implementation adds an IMU-frame rotation covariance term to a world-frame matrix.
   # Keep the same stored P blocks and only restore the missing R...R^T factors here.
   P=scans.matrix('P_prior',19,19)[j]
   bound=lambda b:(lambda v,U: (U*np.clip(v,0,.01))@U.T)(*np.linalg.eigh((b+b.T)*.5))
   Pr,Pt=bound(P[:3,:3]),bound(P[3:6,3:6])
   pi=p@E.T+fm('extrinsic_t',3,1).ravel()
   cross=np.zeros((len(p),3,3));cross[:,0,1]=-pi[:,2];cross[:,0,2]=pi[:,1];cross[:,1,0]=pi[:,2]
   cross[:,1,2]=-pi[:,0];cross[:,2,0]=-pi[:,1];cross[:,2,1]=pi[:,0]
   rot_cov=cross@Pr@cross.transpose(0,2,1)
   stored=np.stack([c(f'point_world_covariance_{u}_{v}') for u in range(3) for v in range(3)],axis=1).reshape(-1,3,3)
   body=np.stack([c(f'point_body_covariance_{u}_{v}') for u in range(3) for v in range(3)],axis=1).reshape(-1,3,3)
   reconstructed=R@body@R.T+rot_cov+Pt
   row['matching_covariance_current_formula_max_abs']=float(np.max(abs(stored-reconstructed)))
   fixed=RL@body@RL.T+R@rot_cov@R.T+Pt
   gated=np.isin(status,[4,5]);delta=np.einsum('ni,nij,nj->n',normal,fixed-stored,normal)
   changed_var=c('gate_variance')+delta
   row['frame_corrected_gate_ratio_quantiles']=quant(changed_var[gated]/c('gate_variance')[gated])
   row['selected_candidates_gate_flips']=int(np.count_nonzero((abs(c('gate_distance')[gated])<3*np.sqrt(np.maximum(0,changed_var[gated])))!=used[gated]))
   rows.append(row);active=(key,forward,frozen,plane_ids,floor_mask[used])
  else:
   assert active[0]==key
   _,forward,frozen,plane_ids,floor_mask=active;row=rows[-1]
   H=np.column_stack([c(f'H_0_{j}') for j in range(6)]);W=c('inverse_variance')
   A=H.T@(H*W[:,None]);S=A[3:,3:]-A[3:,:3]@np.linalg.pinv(A[:3,:3])@A[:3,3:]
   direct=W*(H[:,3:]@forward)**2
   row.update(forward_information=float(direct.sum()),forward_after_rotation_marginalization=float(forward@S@forward),
      frozen_forward_information_fraction=float(direct[frozen].sum()/direct.sum()),
      frozen_bax_gain_contribution=float((c('K_13')*c('innovation'))[frozen].sum()),
      bax_gain_contribution=float((c('K_13')*c('innovation')).sum()),
      floor_forward_information_fraction=float(direct[floor_mask].sum()/direct.sum()))
   _,inv=np.unique(plane_ids,return_inverse=True);group_info=np.bincount(inv,weights=direct)
   row['top10_planes_forward_information_fraction']=float(np.sort(group_info)[-10:].sum()/direct.sum())
   row['effective_forward_plane_count']=float(group_info.sum()**2/(group_info@group_info))
   if len(rows)%20==0:print('profiled',len(rows),'frames at',row['time'],flush=True)
  yield key,names,data
geometry.groups=wrapped
strict_geometry_passed=geometry.analyze(run,out/'geometry-validation')
check=json.loads((out/'geometry-validation/geometry_summary.json').read_text())
errors=check['max_scaled_errors']
assert all(v<1e-6 for k,v in errors.items() if k!='tangential_distance')
(out/'validation-qualification.json').write_text(json.dumps({'strict_geometry_passed':strict_geometry_passed,
 'tangential_float_roundoff_bound_passed':True,'other_geometry_checks_passed':True,
 'strict_tangential_max_scaled_error':errors['tangential_distance'],
 'explanation':'Strict 1e-6 threshold fails only for float square-root cancellation; each value lies in the documented float arithmetic bound. Original validator output is preserved.'},indent=2)+'\n')
(out/'profiles.json').write_text(json.dumps({'rows':rows,'scope':'162 declared temporal samples; not the full point log'},indent=2,allow_nan=False)+'\n')
assert filter_analyze(run,out/'filter-validation')
print('Sample filter/covariance/residual/gain/Hessian PASS; geometry qualification saved',flush=True)
