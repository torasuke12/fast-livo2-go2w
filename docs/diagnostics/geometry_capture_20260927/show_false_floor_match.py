"""Reproduce and plot a scan-floor point accepted against a near-vertical map plane."""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from analyze_geometry_cause import angle, matrix

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('capture', type=Path)
a = p.parse_args()
out = a.capture / 'analysis/root_cause'
rows = json.loads((out / 'geometry_cause.json').read_text())['rows']
r = next(r for r in rows if r['scan_id'] == 2018)
g = np.load(out / 'samples/geometry-02018.npz')
v, columns = g['data'], list(g['columns'])
state = dict(zip(g['frame_columns'], g['frame']))
i, = np.flatnonzero(v[:,columns.index('source_index')] == 1113)
record = {k: float(x) for k,x in zip(columns,v[i])}
points = v[:,9:12]
dist, ids = cKDTree(points).query(points[i],k=20)
centered = points[ids] - points[ids].mean(axis=0)
eig,vec = np.linalg.eigh(centered.T @ centered / 20)
neighbors = v[i,28:].reshape(5,3)
center = neighbors.mean(axis=0)
neig, nvec = np.linalg.eigh((neighbors-center).T @ (neighbors-center) / 5)
R = matrix(state,'linearization_R')
RL = R @ matrix(state,'extrinsic_R')
normal = v[i,18:21]
nf = np.array(r['floor_world_normal'])
ns = RL @ vec[:,0]
record.update(source_floor_normal=nf.tolist(), normal_difference_deg=float(angle(normal,nf)),
              source_local_normal_difference_deg=float(angle(normal,ns)),
              source_floor_distance=float(points[i]@r['floor_sensor_normal']+r['floor_offset']),
              source_local_eigenvalues=eig.tolist(), source_local_normal_lidar=vec[:,0].tolist(),
              source_max_neighbor_distance=float(dist[-1]), map_centered_eigenvalues=neig.tolist(),
              map_pca_to_fitted_angle_deg=float(angle(nvec[:,0],normal)),
              map_pca_to_floor_angle_deg=float(angle(nvec[:,0],nf)))
(out/'false_floor_match_example.json').write_text(json.dumps(record,indent=2)+'\n')

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

fig = plt.figure(figsize=(10,8))
ax = fig.add_subplot(111,projection='3d')
origin = v[i,12:15]
near = np.linalg.norm(points-points[i],axis=1)<.6
world = v[near,12:15]-origin
source_patch = v[ids,12:15]-origin
mp = neighbors-origin
ax.scatter(*world.T,s=6,c='gray',alpha=.2,label='surrounding scan points')
ax.scatter(*source_patch.T,s=22,c='tab:green',label='20 local scan neighbors')
ax.scatter(*mp.T,s=60,c='tab:blue',marker='s',label='five queried map neighbors')
ax.scatter(0,0,0,s=80,c='red',label='accepted query (source 1113)')
for n,offset,color in [(normal,normal@origin+record['plane_d'],'tab:blue'),
                       (nf,record['source_floor_distance'],'tab:green')]:
    tangent = np.cross(n,[0,1,0]);tangent/=np.linalg.norm(tangent)
    tangent2 = np.cross(n,tangent)
    u,w = np.meshgrid(np.linspace(-.5,.5,2),np.linspace(-.5,.5,2))
    plane = -offset*n+u[...,None]*tangent+w[...,None]*tangent2
    ax.plot_surface(plane[...,0],plane[...,1],plane[...,2],color=color,alpha=.18)
    ax.quiver(0,0,0,*n,length=.4,color=color,linewidth=2)
ax.set(xlabel='Relative world X [m]',ylabel='Relative world Y [m]',zlabel='Relative world Z [m]',
       xlim=(-.6,.6),ylim=(-.6,.6),zlim=(-.6,.6),
       title=f'205.10 s: accepted floor/map normal difference {record["normal_difference_deg"]:.1f} deg\n'
             f'query residual {abs(record["distance"])*1000:.1f} mm, map fit max {record["plane_fit_max"]*1000:.1f} mm')
ax.view_init(elev=20,azim=-45);ax.set_box_aspect((1,1,1));ax.legend(loc='upper left',fontsize=8)
fig.tight_layout();fig.savefig(out/'false_floor_match.png',dpi=150)
print(json.dumps(record,indent=2))
