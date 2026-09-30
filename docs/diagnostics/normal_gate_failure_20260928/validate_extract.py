from pathlib import Path
import sys,json,numpy as np,os
sys.path.insert(0,str(Path('humble_ws/src/fast-livo2/scripts').resolve()))
import analyze_lio_geometry as geom
from analyze_lio_diagnostics import Table,analyze as filter_analyze
import argparse
ap=argparse.ArgumentParser();ap.add_argument('capture',type=Path);ap.add_argument('work',type=Path);args=ap.parse_args()
root=args.capture;run,=list((root/'diagnostics').glob('run-*'))
out=args.work/'out';out.mkdir(parents=True,exist_ok=True);samples=out/'samples';samples.mkdir(exist_ok=True);samples.chmod(0o755)
it=Table(run/'iterations.csv');frames=Table(run/'geometry_states.csv');scans=Table(run/'scans.csv',('status',))
flookup={(int(s),int(i)):j for j,(s,i) in enumerate(zip(frames['scan_id'],frames['iteration']))}
ilookup={(int(s),int(i)):j for j,(s,i) in enumerate(zip(it['scan_id'],it['iteration']))}
selected=set();last=-1
for s,i,t in zip(frames['scan_id'],frames['iteration'],frames['time']):
 if int(t)>last or 295<=t<=325:
  selected.add((int(s),int(i)));last=int(t)
original_groups=geom.groups;active=None;rows=[];max_gain_error=0.

def wrapped(path,selected_arg=None):
 global active,max_gain_error
 for key,names,data in original_groups(path,selected_arg):
  if key in selected:
   prefix='geometry' if path.name=='geometry.csv' else 'detail'
   fi=flookup[key]
   np.savez_compressed(samples/f'{prefix}-{key[0]:05}.npz',data=data,columns=names,frame=frames.values[fi],frame_columns=list(frames.columns))
  col=lambda name:data[:,names.index(name)]
  if path.name=='geometry.csv':
   status=col('status').astype(int);usable=col('source_usable')==1;angle=col('normal_angle_deg');accepted=status==5
   fi=flookup[key];R=np.array([[frames[f'linearization_R_{i}_{j}'][fi] for j in range(3)] for i in range(3)])
   forward=R[:,0].copy();forward[2]=0;forward/=np.linalg.norm(forward)
   masks={'all':accepted,'unavailable':accepted&~usable,'angle0_5':accepted&usable&(angle<=5),
          'angle5_15':accepted&usable&(angle>5)&(angle<=15),'angle15_30':accepted&usable&(angle>15),
          'source_good':accepted&usable}
   active=(key,{k:v[accepted] for k,v in masks.items()},forward)
   row={'scan_id':key[0],'time':float(col('time')[0]),'points':len(data),
        'status_counts':np.bincount(status,minlength=7).tolist(),
        'quality_counts':np.bincount(col('source_quality').astype(int),minlength=9).tolist(),
        'accepted_quality_counts':np.bincount(col('source_quality')[accepted].astype(int),minlength=9).tolist()}
   rows.append(row)
  else:
   assert active[0]==key
   masks,forward=active[1:];row=rows[-1]
   H=np.column_stack([col(f'H_0_{i}') for i in range(6)]);W=col('inverse_variance');innov=col('innovation')
   contrib=np.column_stack([col(f'K_{i}') for i in range(7,16)])*innov[:,None]
   ij=ilookup[key];expected=np.array([it[f'Kinnovation_{i}_0'][ij] for i in range(7,16)])
   error=float(np.max(abs(contrib.sum(axis=0)-expected)));assert error<1e-9
   max_gain_error=max(max_gain_error,error)
   information=W*(H[:,3:]@forward)**2
   row['groups']={k:{'count':int(m.sum()),'correction':contrib[m].sum(axis=0).tolist(),
     'absolute_correction':abs(contrib[m]).sum(axis=0).tolist(),'forward_information':float(information[m].sum())} for k,m in masks.items()}
   P=np.array([[it[f'P_iter_{i}_{j}'][ij] for j in range(19)] for i in range(19)])
   row['unconstrained_sensitivity']={}
   for name,m in [('all',masks['all']),('source_good',masks['source_good']),('angle0_5',masks['angle0_5'])]:
    h,w,inn=H[m],W[m],innov[m];A=h.T@(w[:,None]*h);score=h.T@(w*inn)
    corr=P[:,:6]@np.linalg.solve(np.eye(6)+A@P[:6,:6],score)
    row['unconstrained_sensitivity'][name]=corr.tolist()
   if len(rows)%200==0: print('validated/extracted scans',len(rows),'time',row['time'],flush=True)
  yield key,names,data
geom.groups=wrapped
assert geom.analyze(run,out/'geometry_validation')
(out/'all_point_groups.json').write_text(json.dumps({'rows':rows,'max_gain_error':max_gain_error},allow_nan=False)+'\n')
print('geometry + groups + samples complete',flush=True)
assert filter_analyze(run,out/'filter_validation')
print('all validation PASS',flush=True)
for folder in out.rglob('*'):
 if folder.is_dir():folder.chmod(0o755)
