from pathlib import Path
import json
import numpy as np
import argparse

p=argparse.ArgumentParser()
p.add_argument('runs', nargs='+', type=Path)
p.add_argument('--out', required=True, type=Path)
a=p.parse_args(); a.out.mkdir(parents=True,exist_ok=True)

def rotations(rpy):
    r,p,y=np.moveaxis(rpy,-1,0)
    cr,sr,cp,sp,cy,sy=np.cos(r),np.sin(r),np.cos(p),np.sin(p),np.cos(y),np.sin(y)
    R=np.empty((len(r),3,3))
    R[:,0,0]=cy*cp; R[:,0,1]=cy*sp*sr-sy*cr; R[:,0,2]=cy*sp*cr+sy*sr
    R[:,1,0]=sy*cp; R[:,1,1]=sy*sp*sr+cy*cr; R[:,1,2]=sy*sp*cr-cy*sr
    R[:,2,0]=-sp; R[:,2,1]=cp*sr; R[:,2,2]=cp*cr
    return R

def load(run):
    s=np.loadtxt(run/'mat_out.txt'); pre=np.loadtxt(run/'mat_pre.txt')
    assert np.isfinite(s).all() and np.isfinite(pre).all(), 'Non-finite state'
    idx=np.searchsorted(pre[:,0],s[:,0]); pre=pre[idx]
    assert np.array_equal(pre[:,0],s[:,0]), 'Prior/posterior time mismatch'
    t=s[:,0]; R=rotations(s[:,1:4]/57.3)
    forward=np.einsum('ni,ni->n',R[:,:,0],s[:,7:10])
    pred=np.vstack([np.zeros(18),pre[1:,1:]-s[:-1,1:19]])
    corr=s[:,1:19]-pre[:,1:]
    dt=np.r_[np.nan,np.diff(t)]
    d=None
    if (run/'xicp_diagnostic.csv').exists():
        d=np.genfromtxt(run/'xicp_diagnostic.csv',delimiter=',',names=True)
        times,first=np.unique(d['time'],return_index=True)
        last=np.r_[first[1:]-1,len(d)-1]
        assert len(times)==len(t) and np.max(abs(times-t))<0.0006
        for j,ax in enumerate('xyz'):
            s[:,10+j]=d['after_bg_'+ax][last]; pre[:,10+j]=d['prior_bg_'+ax][first]
            s[:,13+j]=d['after_ba_'+ax][last]; pre[:,13+j]=d['prior_ba_'+ax][first]
            s[:,7+j]=d['after_v'+ax][last]; pre[:,7+j]=d['prior_v'+ax][first]
        # Use precise CSV for velocity, bias, pitch. mat_* Euler angles use *57.3.
        t=times; dt=np.r_[np.nan,np.diff(t)]
        s[:,2]=d['after_pitch'][last]*57.3; pre[:,2]=d['prior_pitch'][first]*57.3
        pred=np.vstack([np.zeros(18),pre[1:,1:]-s[:-1,1:19]])
        corr=s[:,1:19]-pre[:,1:]
        forward=d['after_forward_speed'][last]
    return s,pre,t,R,forward,pred,corr,dt,d

all_rows=[]; runs=[]
for run in a.runs:
    s,pre,t,R,forward,pred,corr,dt,d=load(run)
    runs.append((run,s,t,forward))
    imu=np.loadtxt(run/'imu.txt')
    name='user327' if run.name=='fast-livo2-xicp-327s.ELNtlC' else run.name
    print('\nRUN',name,'scans',len(t),'end',t[-1],'imu',len(imu),'maxgap',np.diff(imu[:,0]).max())
    print('max prior bias change',np.max(abs(pred[:,12:15])),'max bg change',np.max(abs(pred[:,9:12])))
    print('window | pitch start/end | delta pitch IMU/LIO deg | ba start/end norm | LIO dba xyz | IMU aW | LIO dvW | height start/end')
    for start,end in [(140,150),(150,160),(160,170),(170,180),(180,190),(190,200),(200,210),(210,220),(180,200),(200,220),(220,240),(240,260),(260,280),(280,300),(300,310),(310,315),(315,320),(320,325),(325,337),(337,360)]:
        m=(t>=start)&(t<end)
        if not m.any(): continue
        ix=np.flatnonzero(m); lo,hi=ix[0],ix[-1]
        row={'run':name,'start':start,'end':end,'scans':int(m.sum()),'pitch_start_deg':float(s[lo,2]/57.3*180/np.pi),'pitch_end_deg':float(s[hi,2]/57.3*180/np.pi),
        'pitch_imu_deg':float(pred[m,1].sum()/57.3*180/np.pi),'pitch_lio_deg':float(corr[m,1].sum()/57.3*180/np.pi),
        'ba_start':s[lo,13:16].tolist(),'ba_end':s[hi,13:16].tolist(),'ba_norm_start':float(np.linalg.norm(s[lo,13:16])),'ba_norm_end':float(np.linalg.norm(s[hi,13:16])),
        'lio_dba':corr[m,12:15].sum(axis=0).tolist(),'imu_accel_world_mean':np.mean(pred[m,6:9]/dt[m,None],axis=0).tolist(),
        'lio_dv_world':corr[m,6:9].sum(axis=0).tolist(),'z_start':s[lo,6],'z_end':s[hi,6]}
        im=(imu[:,0]>=start)&(imu[:,0]<end)
        raw=imu[im,4:7].mean(axis=0)*9.81/0.9997
        row['raw_accel_pitch_proxy_deg']=float(np.rad2deg(np.arctan2(-raw[0],np.hypot(raw[1],raw[2]))))
        row['gyro_raw_mean']=imu[im,1:4].mean(axis=0).tolist()
        row['bg_mean']=s[m,10:13].mean(axis=0).tolist()
        row['forward_mean']=float(np.mean(forward[m])); row['ba_mean']=s[m,13:16].mean(axis=0).tolist()
        row['pitch_mean_deg']=float(s[m,2].mean()/57.3*180/np.pi)
        row['ba_slope']=np.linalg.lstsq(np.column_stack([np.ones(m.sum()),t[m]-start]),s[m,13:16],rcond=None)[0][1].tolist()
        if d is not None:
            dm=(d['time']>=start)&(d['time']<end)
            axes=np.column_stack([abs(d[f't{k}_axis_wx']) for k in range(3)])
            closest=axes.argmax(axis=1)
            cat=np.column_stack([d[f't{k}_category'] for k in range(3)])[np.arange(len(d)),closest]
            row['world_x_categories']=dict(zip(*[v.astype(int).tolist() for v in np.unique(cat[dm],return_counts=True)]))
            row['matches_median']=float(np.median(d['matches'][dm]))
            r_axes=np.column_stack([abs(d[f'r{k}_axis_ly']) for k in range(3)])
            r_closest=r_axes.argmax(axis=1)
            r_cat=np.column_stack([d[f'r{k}_category'] for k in range(3)])[np.arange(len(d)),r_closest]
            row['body_y_rotation_categories']=dict(zip(*[v.astype(int).tolist() for v in np.unique(r_cat[dm],return_counts=True)]))
            row['body_y_alignment_median']=float(np.median(r_axes[dm].max(axis=1)))
        all_rows.append(row)
        print(start,end, np.round([row['pitch_start_deg'],row['pitch_end_deg']],3),np.round([row['pitch_imu_deg'],row['pitch_lio_deg']],3),np.round([row['ba_norm_start'],row['ba_norm_end']],3),np.round(row['lio_dba'],3),np.round(row['imu_accel_world_mean'],3),np.round(row['lio_dv_world'],3),np.round([row['z_start'],row['z_end']],3))
    print('CHECKPOINT time, pitch, roll, yaw deg, ba, bg, pos, forward')
    for target in [150,160,170,180,190,200,220,240,260,280,300,310,320,327,337,350,359.9]:
        if target>t[-1]+.1: continue
        i=np.argmin(abs(t-target))
        print(round(t[i],4),np.round(s[i,[2,1,3]]/57.3*180/np.pi,4),np.round(s[i,13:16],6),np.round(s[i,10:13],6),np.round(s[i,4:7],3),round(forward[i],4))
    for start,end in [(180,310),(160,310)]:
        m=(t>=start)&(t<end)
        print('TOTAL',start,end,'pitch IMU/LIO',np.round([pred[m,1].sum(),corr[m,1].sum()]/np.array([57.3,57.3])*180/np.pi,4),'dba',corr[m,12:15].sum(axis=0),'dbg',corr[m,9:12].sum(axis=0),'dv IMU/LIO',pred[m,6:9].sum(axis=0),corr[m,6:9].sum(axis=0))
(a.out/'windows.json').write_text(json.dumps(all_rows,indent=2))
try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(4,1,figsize=(11,10),sharex=True)
    for run,s,t,forward in runs:
        name='user327' if run.name=='fast-livo2-xicp-327s.ELNtlC' else run.name
        visible=(t>=140)&(t<=360)
        ax[0].plot(t[visible],s[visible,2]/57.3*180/np.pi,label=name)
        ax[1].plot(t[visible],np.linalg.norm(s[visible,13:16],axis=1))
        ax[2].plot(t[visible],s[visible,6]); ax[3].plot(t[visible],forward[visible])
    for a0,label in zip(ax,['Estimated IMU pitch [deg]','Accel bias norm [m/s2]','Estimated world z [m]','Body x velocity [m/s]']):
        a0.set_ylabel(label); a0.grid(alpha=.25); a0.axvspan(308,315,color='gray',alpha=.15)
    handles,labels=ax[0].get_legend_handles_labels()
    fig.legend(handles,labels,loc='upper center',ncol=3,fontsize=9)
    ax[3].set_xlim(140,360); ax[3].axhline(0,color='black',linewidth=.8); ax[3].set_xlabel('Node-relative time [s]')
    fig.tight_layout(rect=(0,0,1,.94)); fig.savefig(a.out/'onset.png',dpi=150); fig.savefig(a.out/'onset.pdf'); plt.close(fig)
except ImportError as e:
    print('Plot unavailable:',e)
