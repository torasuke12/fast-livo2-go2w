#!/usr/bin/env python3
"""Compare start/return geometry without aligning the evaluation clouds again."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from optimize_reconstruction import cloud,load_frames,transform


def distances(source,target):
    d,ids=cKDTree(np.asarray(target.points)).query(np.asarray(source.points))
    residual=np.abs(np.einsum('ij,ij->i',np.asarray(source.points)-np.asarray(target.points)[ids],np.asarray(target.normals)[ids]))
    keep=d<.15
    return dict(points=len(d),overlap_15cm=float(keep.mean()),nearest_median_m=float(np.median(d)),
                nearest_p90_m=float(np.quantile(d,.9)),
                inlier_plane_rmse_m=float(np.sqrt(np.mean(residual[keep]**2))) if keep.any() else None)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('capture',type=Path);p.add_argument('reconstruction',type=Path)
    p.add_argument('--start',nargs=2,type=float,default=[4.,12.])
    p.add_argument('--return-window',nargs=2,type=float,default=[632.,648.])
    args=p.parse_args()
    rows,_=load_frames(args.capture)
    trajectory=np.load(args.reconstruction/'trajectory.npz')
    metrics={}
    fig,axes=plt.subplots(2,3,figsize=(17,10))
    for row_index,(name,key) in enumerate([('before','original'),('after','corrected')]):
        poses=trajectory[key]
        rooms=[]
        for bounds in (args.start,args.return_window):
            points=[]
            for i in np.flatnonzero((rows['time']>=bounds[0])&(rows['time']<=bounds[1])):
                # Use the odd-index returns held out of ICP fitting. No additional
                # rigid alignment is applied for these reported measurements.
                raw=np.fromfile(args.capture/'scans'/f'{int(rows[i]["id"])}.bin',np.float32).reshape(-1,3)[1::2]
                raw=raw[np.isfinite(raw).all(1)&(np.linalg.norm(raw,axis=1)<15)]
                points.append(transform(raw,poses[i]))
            if not points:raise ValueError('No points in evaluation window')
            rooms.append(cloud(np.concatenate(points),.07))
        metrics[name]=dict(start_to_return=distances(rooms[0],rooms[1]),return_to_start=distances(rooms[1],rooms[0]))
        for room,color in zip(rooms,[[.05,.3,1.],[1.,.4,.02]]):room.paint_uniform_color(color)
        o3d.io.write_point_cloud(str(args.reconstruction/f'rooms_{name}.ply'),rooms[0]+rooms[1])
        for j,(a,b,label) in enumerate([(0,1,'XY'),(0,2,'XZ')]):
            ax=axes[row_index,j]
            for room,color in zip(rooms,['royalblue','darkorange']):
                points=np.asarray(room.points)
                if j==0:points=points[(points[:,2]>.1)&(points[:,2]<2.)] if name=='after' else points
                ax.scatter(points[::2,a],points[::2,b],s=.3,c=color,alpha=.55)
            ax.set_aspect('equal');ax.grid();ax.set_title(f'{name}: room {label} (m)')
        ax=axes[row_index,2]
        ax.plot(rows['time'],poses[:,2,3],lw=1.2)
        ax.axhline(0,color='gray',lw=.7);ax.grid()
        ax.set_xlabel('Time (s)');ax.set_ylabel('Height (m)')
        ax.set_title(f'{name}: climb, corridor, descent, return')
    fig.tight_layout();fig.savefig(args.reconstruction/'comparison.png',dpi=170)
    metrics['definition']='Held-out odd returns; no evaluation-time alignment. Nearest distances include visibility/occlusion differences. Inlier plane RMSE only uses correspondences within 15 cm, not absolute trajectory accuracy.'
    metrics['windows_seconds']=[args.start,args.return_window]
    (args.reconstruction/'room_metrics.json').write_text(json.dumps(metrics,indent=2))
    print(json.dumps(metrics,indent=2))


if __name__=='__main__':main()
