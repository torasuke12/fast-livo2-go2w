"""Standalone figures; estimates and corrections are not ground-truth errors."""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('analysis',type=Path)
args = parser.parse_args()
old = np.load(args.analysis/'old/data.npz')
new = np.load(args.analysis/'new/data.npz')
aligned = np.load(args.analysis/'alignment.npz')
fig, axes = plt.subplots(4,2,figsize=(13,11),sharex='col',layout='constrained')
for column,(start,end,title) in enumerate([(470,500,'U-turn vicinity'),(520,575,'Stair descent vicinity')]):
    for d,label,color in [(old,'Previous single-scan run','#999999'),(new,'Latest legacy-init run','#2166ac')]:
        t = d['t']
        m = (t>=start)&(t<=end)
        axes[0,column].plot(t[m],d['p'][m,2],label=label,color=color,lw=1.5)
        axes[1,column].plot(t[m],100*d['corr'][m,5],color=color,lw=1)
        axes[2,column].plot(t[m],d['ba'][m,0],color=color,lw=1.3)
        axes[3,column].plot(t[m],d['imu_rms'][m],color=color,lw=1.2)
    k = (new['kt']>=start)&(new['kt']<=end)
    axes[0,column].plot(new['kt'][k],new['kp'][k,2],color='#d95f02',ls='--',lw=1.4,label='Latest: last reported keyframes')
    axes[0,column].set_title(title)
    for row,label in enumerate(['Estimated z [m]','IMU prediction to update: dz [cm]', 'IMU-frame accel bias x [m/s^2]','Whitened IMU residual RMS (window)']):
        axes[row,column].set_ylabel(label)
        axes[row,column].grid(alpha=.25)
    axes[-1,column].set_xlabel('Sensor time from first LiDAR [s]')
    axes[0,column].legend(fontsize=8)
fig.suptitle('Initialization change: substantial vertical corrections remain\nHeight includes actual motion; no height ground truth is available',fontsize=13)
fig.savefig(args.analysis/'vertical_comparison.png',dpi=160)
plt.close(fig)

fig,axes = plt.subplots(3,1,figsize=(11,8),sharex=True,layout='constrained')
axes[0].plot(aligned['t'],aligned['yaw_difference'],color='#2166ac')
axes[0].set_ylabel('Latest - previous yaw [deg]')
axes[0].set_ylim(-17,-10)
axes[1].plot(aligned['t'],aligned['xy_difference'],color='#2166ac')
axes[1].set_ylabel('Aligned XY difference [m]')
axes[2].plot(aligned['t'],aligned['z_difference'],color='#2166ac')
axes[2].set_ylabel('Latest - previous z [m]')
axes[2].set_xlabel('Sensor time from first LiDAR [s]')
for ax in axes:
    ax.set_xlim(20,648.2)
    ax.grid(alpha=.25)
    ax.axvspan(470,495,color='#d95f02',alpha=.12)
    ax.axvspan(530,555,color='#d95f02',alpha=.12)
fig.suptitle('Separate initial heading from later shape changes\nXY uses one rigid transform fitted at 20-60 s; no scaling or whole-run fit',fontsize=13)
fig.savefig(args.analysis/'aligned_comparison.png',dpi=160)
