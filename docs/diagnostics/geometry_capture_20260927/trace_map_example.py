"""Find earlier scan coordinates that could have generated the example's map points.

Coordinate agreement is not a persistent map ID or insertion event. Only the
recorded 140 s onward scan candidates are searched.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from analyze_geometry_cause import matrix, fit_plane

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('capture',type=Path)
a=p.parse_args()
out=a.capture/'analysis/root_cause'
example=json.loads((out/'false_floor_match_example.json').read_text())
targets=np.array([[example[f'neighbors_{i}_{j}'] for j in range(3)] for i in range(5)])
best=[dict(distance=float('inf')) for _ in range(5)]
reference=np.array([-np.sin(np.deg2rad(19)),0,np.cos(np.deg2rad(19))])
for path in sorted((out/'all_frames').glob('geometry-*.npz')):
    z=np.load(path)
    state=dict(zip(z['frame_columns'],z['frame']))
    if state['time'] >= example['time']:
        break
    v=z['data']; points=v[:,9:12]
    R=matrix(state,'after_R'); extR=matrix(state,'extrinsic_R')
    ti=matrix(state,'extrinsic_t',cols=1)[:,0]
    position=matrix(state,'after_position',cols=1)[:,0]
    world=((points@extR.T+ti)@R.T+position).astype(np.float32).astype(float)
    dist=np.sum((world[:,None]-targets[None])**2,axis=2)
    ids=dist.argmin(axis=0)
    changed=[k for k in range(5) if dist[ids[k],k] < best[k]['distance']**2]
    if not changed:
        continue
    near=(np.linalg.norm(points,axis=1)<8)&(np.linalg.norm(points,axis=1)>.7)
    projected=points@reference
    floor=fit_plane(points[near&(projected> -1.8)&(projected<-.12)],reference)
    for k in changed:
        i=ids[k]
        best[k]=dict(neighbor=k,distance=float(np.sqrt(dist[i,k])),time=state['time'],scan_id=int(state['scan_id']),
                     source_index=int(v[i,3]),lidar=points[i].tolist(),reconstructed_world=world[i].tolist(),
                     floor_distance=None if floor is None else float(points[i]@floor['normal']+floor['offset']),
                     source_floor_rms=None if floor is None else floor['rms'])
    if min(b['distance'] for b in best)<1e-5:
        print(state['time'],[(k,round(b['distance'],8)) for k,b in enumerate(best)],flush=True)
(out/'example_map_candidates.json').write_text(json.dumps(dict(example_time=example['time'],candidates=best),indent=2)+'\n')
z=np.load(out/f'all_frames/geometry-{int(example["scan_id"]):05}.npz')
state=dict(zip(z['frame_columns'],z['frame']))
v=z['data']; q=v[v[:,3]==example['source_index']][0]
R=matrix(state,'after_R')
world=((q[9:12]@matrix(state,'extrinsic_R').T+matrix(state,'extrinsic_t',cols=1)[:,0])@R.T+
       matrix(state,'after_position',cols=1)[:,0]).astype(np.float32)
center=(np.floor(world/.5)*.5+.25).astype(np.float32)
neighbors=q[28:].reshape(5,3).astype(np.float32)
cd=np.sum((world-center)**2); nd=np.sum((neighbors-center)**2,axis=1)
replacement=dict(candidate_world=world.tolist(),voxel_center=center.tolist(),candidate_distance_squared=float(cd),
                 neighbor_distance_squared=nd.tolist(),bypass_downsample=bool(np.all(abs(neighbors[0]-center)>.25)),
                 closer_neighbors=np.where(nd<cd)[0].tolist(),
                 decision='not added by UpdateFastLioMap prefilter',
                 note='Reconstruction of source-code branch, not an insertion event log; map voxel size is 0.5 m in saved parameters.')
assert not replacement['bypass_downsample'] and replacement['closer_neighbors']
(out/'map_replacement_example.json').write_text(json.dumps(replacement,indent=2)+'\n')
print(json.dumps(best,indent=2))
