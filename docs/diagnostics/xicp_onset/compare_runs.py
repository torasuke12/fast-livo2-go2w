from pathlib import Path
import argparse, json, hashlib
import numpy as np
p=argparse.ArgumentParser(); p.add_argument('runs',nargs='+',type=Path); p.add_argument('--out',required=True,type=Path); a=p.parse_args()
rows=[]; streams=[]
for run in a.runs:
    s=np.loadtxt(run/'mat_out.txt'); pre=np.loadtxt(run/'mat_pre.txt'); imu=np.loadtxt(run/'imu.txt')
    pre=pre[np.searchsorted(pre[:,0],s[:,0])]; assert np.array_equal(s[:,0],pre[:,0])
    pitch,yaw=s[:,2]/57.3,s[:,3]/57.3
    forward=(s[:,7]*np.cos(yaw)+s[:,8]*np.sin(yaw))*np.cos(pitch)-s[:,9]*np.sin(pitch)
    backward=[i for i in range(len(s)-19) if s[i,0]>315 and np.all(forward[i:i+20]<0)]
    dvi=np.vstack([np.zeros(3),pre[1:,7:10]-s[:-1,7:10]]); dvl=s[:,7:10]-pre[:,7:10]
    row={'run':run.name,'states':len(s),'last_time':s[-1,0],'nonfinite':int((~np.isfinite(s)).sum()),'imu_processed_rows':len(imu),
         'imu_processed_max_gap':float(np.diff(imu[:,0]).max()),'reversal_start':float(s[backward[0],0]) if backward else None,
         'checkpoints':{str(t):{'node_time':s[i,0],'forward_speed':forward[i],'bias_norm':float(np.linalg.norm(s[i,13:16])),'position':s[i,4:7].tolist()} for t in [180,200,250,300,310,320,327,337,350,359.9] if t<=s[-1,0]+.1 for i in [np.argmin(abs(s[:,0]-t))]},
         'windows':{str((lo,hi)):{'imu_dv':dvi[m].sum(axis=0).tolist(),'lio_dv':dvl[m].sum(axis=0).tolist()} for lo,hi in [(180,310),(310,320),(320,325),(337,360)] for m in [(s[:,0]>=lo)&(s[:,0]<hi)] if m.any()}}
    if (run/'summary.json').exists(): row['replay_summary']=json.loads((run/'summary.json').read_text())
    if (run/'received_imu_stamps.txt').exists():
        stamps=np.loadtxt(run/'received_imu_stamps.txt',dtype=np.int64); gaps=np.diff(stamps)/1e9
        row['monitor']={'count':len(stamps),'duplicates':int((gaps==0).sum()),'backward':int((gaps<0).sum()),'max_gap':float(gaps.max()),'gaps_over_10ms':int((gaps>.01).sum()),'start_ns':int(stamps[0]),'end_ns':int(stamps[-1])}
    rows.append(row); streams.append((run.name,imu,s))
    print(run.name,'states',row['states'],'end',row['last_time'],'backward',row['reversal_start'],'imu',len(imu),'maxgap',row['imu_processed_max_gap'])
    print(' monitor',row.get('monitor'))
    print(' checkpoints',row['checkpoints'])
for i in range(len(streams)):
    for j in range(i):
        n1,im1,s1=streams[i]; n2,im2,s2=streams[j]; n=min(len(im1),len(im2)); ns=min(len(s1),len(s2))
        print('PREFIX',n1,n2,'imu exact',np.array_equal(im1[:n],im2[:n]),'length',n,'scan times',np.array_equal(s1[:ns,0],s2[:ns,0]),'raw point counts',np.array_equal(s1[:ns,19],s2[:ns,19]))
a.out.write_text(json.dumps(rows,indent=2))
