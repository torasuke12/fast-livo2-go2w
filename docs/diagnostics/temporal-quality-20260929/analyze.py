import argparse,csv,json
from pathlib import Path
import numpy as np
parser=argparse.ArgumentParser(description='Separate published path revisions from online estimator motion')
parser.add_argument('capture',type=Path)
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args(); cap=args.capture; out=args.output; out.mkdir(parents=True,exist_ok=True)
with (cap/'logs/temporal_window.csv').open() as f: rows=list(csv.DictReader(f))
keys=[k for k in rows[0] if k!='status']; a=np.array([[float(r[k]) for k in keys] for r in rows]); d={k:a[:,i] for i,k in enumerate(keys)}
t=d['time'];p=np.column_stack([d['p'+k] for k in 'xyz']);v=np.column_stack([d['v'+k] for k in 'xyz']);q=np.column_stack([d[k] for k in ['qw','qx','qy','qz']]); w,x,y,z=q.T
pitch=np.rad2deg(np.arcsin(np.clip(2*(w*y-z*x),-1,1)));roll=np.rad2deg(np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y)));yaw=np.rad2deg(np.unwrap(np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))))
corr=np.column_stack([d['correction_'+str(k)] for k in range(15)])
statekeys=['qw','qx','qy','qz','px','py','pz','vx','vy','vz','bgx','bgy','bgz','bax','bay','baz']
first={};last={};is_key={};times={}; revisions=[]; previous={}
with (cap/'logs/temporal_states.csv').open() as f:
 for r in csv.DictReader(f):
  i=int(r['node_id']);now=float(r['time']);s=np.array([float(r[k]) for k in statekeys]);times[i]=float(r['node_time']);is_key[i]=int(r['keyframe']);first.setdefault(i,s.copy());last[i]=s
  if i in previous:
   prev=previous[i];revisions.append([now,i,*list(s[4:7]-prev[4:7]),np.rad2deg(2*np.arccos(np.clip(abs(s[:4]@prev[:4]),0,1)))])
  previous[i]=s
ids=sorted(last); finalpath=np.array([last[i][4:7] for i in ids]);firstpath=np.array([first[i][4:7] for i in ids]);keymask=np.array([bool(is_key[i]) for i in ids]);kt=t[keymask];kp=finalpath[keymask];kv=np.array([last[i][7:10] for i in ids if is_key[i]])
revisions=np.array(revisions)
stepv=np.diff(p,axis=0)/np.diff(t)[:,None];mixedv=np.diff(finalpath,axis=0)/np.diff(t)[:,None];keyv=np.diff(kp,axis=0)/np.diff(kt)[:,None]
metrics=[]
for begin,end in [(3.2,10),(10,60),(60,140),(140,280),(280,360),(360,450),(450,470),(470,490),(490,510),(510,590),(590,610),(610,630),(630,648.2)]:
 m=(t>=begin)&(t<end);sv=(t[1:]>=begin)&(t[1:]<end);km=(kt[1:]>=begin)&(kt[1:]<end)
 point={"start":begin,"end":end,"scans":int(m.sum()),'position_start':p[m][0].tolist(),'position_end':p[m][-1].tolist(), 'z_minmax':[float(p[m,2].min()),float(p[m,2].max())], 'roll_range':[float(roll[m].min()),float(roll[m].max())], 'pitch_range':[float(pitch[m].min()),float(pitch[m].max())], 'yaw_range':[float(yaw[m].min()),float(yaw[m].max())], 'vertical_velocity_abs_max':float(abs(v[m,2]).max()), 'online_position_step_max':float(np.linalg.norm(np.diff(p,axis=0)[sv],axis=1).max()),'mixed_path_step_max':float(np.linalg.norm(np.diff(finalpath,axis=0)[sv],axis=1).max()),'online_step_vz_abs_max':float(abs(stepv[sv,2]).max()),'mixed_path_step_vz_abs_max':float(abs(mixedv[sv,2]).max()),'keyframe_step_vz_abs_max':float(abs(keyv[km,2]).max()),'step_velocity_disagreement_rms':float(np.sqrt(np.mean((stepv[sv]-v[1:][sv])**2))), 'correction_position_max':float(np.linalg.norm(corr[m,3:6],axis=1).max()),'correction_z_abs_max':float(abs(corr[m,5]).max()),'correction_rotation_deg_max':float(np.rad2deg(np.linalg.norm(corr[m,:3],axis=1)).max()),'imu_rms_median':float(np.median(d['imu_rms'][m])),'imu_rms_max':float(d['imu_rms'][m].max()),'lidar_rms_median':float(np.median(d['lidar_rms'][m])),'matches_min':int(d['matches'][m].min()),'cost_increases':int(np.sum(d['cost_after'][m]>d['cost_before'][m]+1e-6))}
 metrics.append(point)
# The existing imu.txt contains adjacent-sample means; acceleration is still in g here.
imu=np.loadtxt(cap/'logs/imu.txt'); accnorm=np.linalg.norm(imu[:,4:7],axis=1);gyrnorm=np.linalg.norm(imu[:,1:4],axis=1)
for entry in metrics:
 m=(imu[:,0]>=entry['start'])&(imu[:,0]<entry['end']);entry['imu_accel_norm_g_max']=float(accnorm[m].max());entry['imu_gyro_norm_rad_s_max']=float(gyrnorm[m].max())
revdelta=finalpath-firstpath
peakids=np.argsort(abs(corr[:,5]))[-25:][::-1]
report={'capture':str(cap), 'scans':len(t),'keyframes':int(keymask.sum()),'final_position':p[-1].tolist(),'position_extent':(p.max(axis=0)-p.min(axis=0)).tolist(), 'online_length':float(np.linalg.norm(np.diff(p,axis=0),axis=1).sum()),'mixed_path_length':float(np.linalg.norm(np.diff(finalpath,axis=0),axis=1).sum()),'keyframe_length':float(np.linalg.norm(np.diff(kp,axis=0),axis=1).sum()),'keyframe_first_to_last_revision_position_max':float(np.linalg.norm(revdelta[keymask],axis=1).max()), 'keyframe_first_to_last_revision_z_max':float(abs(revdelta[keymask,2]).max()), 'nonkeyframe_revision_position_max':float(np.linalg.norm(revdelta[~keymask],axis=1).max()), 'windows':metrics, 'largest_z_corrections':[{'time':float(t[i]),'dp':corr[i,3:6].tolist(),'z':float(p[i,2]),'vz':float(v[i,2]),'keyframe':bool(keymask[i]),'matches':int(d['matches'][i]),'imu_rms':float(d['imu_rms'][i])} for i in peakids]}
(out/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
np.savez_compressed(out/'quality.npz',**d,p=p,v=v,q=q,roll=roll,pitch=pitch,yaw=yaw,finalpath=finalpath,firstpath=firstpath,keymask=keymask,keyframe_time=kt,keyframe_position=kp,keyframe_velocity=kv,revision_events=revisions,imu=imu)
print(json.dumps({k:v for k,v in report.items() if k not in ('windows','largest_z_corrections')},indent=2))
