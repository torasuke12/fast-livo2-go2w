import argparse
import numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path
parser=argparse.ArgumentParser();parser.add_argument('output',type=Path);root=parser.parse_args().output
a=np.load(root/'quality.npz');t=a['time'];p=a['p'];mixed=a['finalpath'];kt=a['keyframe_time'];kp=a['keyframe_position']
fig,axs=plt.subplots(4,1,figsize=(12,13))
axs[0].plot(p[:,0],p[:,1],label='Online newest poses');axs[0].plot(kp[:,0],kp[:,1],lw=.8,label='Last optimized keyframes');axs[0].axis('equal');axs[0].set_ylabel('y (m)');axs[0].set_xlabel('x (m)')
for ax,(lo,hi) in zip(axs[1:],[(3,648),(470,495),(528,560)]):
 m=(t>=lo)&(t<=hi); k=(kt>=lo)&(kt<=hi)
 ax.plot(t[m],mixed[m,2],lw=.7,label='Original /path: mixed revisions');ax.plot(t[m],p[m,2],lw=1.2,label='Online newest poses');ax.plot(kt[k],kp[k,2],label='Last optimized keyframes')
 ax.set_ylabel('z (m)');ax.set_xlabel('Sensor time (s)');ax.grid();ax.legend()
axs[0].legend();fig.tight_layout();fig.savefig(root/'path_revisions.png',dpi=150);plt.close(fig)
fig,axs=plt.subplots(5,2,figsize=(14,13),sharex='col')
for col,(lo,hi) in enumerate([(470,495),(525,565)]):
 m=(t>=lo)&(t<hi)
 for row,(vals,label) in enumerate([(p[:,2],'z (m)'),(a['correction_5'],'Position correction z (m)'),(a['pitch'],'IMU pitch (deg)'),(a['imu_rms'],'Whitened IMU RMS'),(a['lidar_rms'],'Point-plane RMS (m)')]):
  axs[row,col].plot(t[m],vals[m]);axs[row,col].set_ylabel(label);axs[row,col].grid()
 axs[-1,col].set_xlabel('Sensor time (s)')
fig.tight_layout();fig.savefig(root/'stairs_diagnostics.png',dpi=150)
