from pathlib import Path
import sys,json,numpy as np,time,zipfile
sys.path.insert(0,str(Path('docs/diagnostics/geometry_capture_20260927').resolve()))
from analyze_geometry_cause import fit_plane,angle
import argparse
ap=argparse.ArgumentParser();ap.add_argument('work',type=Path);args=ap.parse_args()
p=args.work;done=p/'sample_results';done.mkdir(exist_ok=True)
d=np.load(p/'dynamics/dynamics.npz');idx={int(s):i for i,s in enumerate(d['scan_id'])};ref=np.array([-np.sin(np.deg2rad(19)),0,np.cos(np.deg2rad(19))])
def process(file):
 g=np.load(file);data,names=g['data'],list(g['columns']);state=dict(zip(g['frame_columns'],g['frame']));scan=int(state['scan_id']);j=idx[scan]
 col=lambda name:data[:,names.index(name)];vec=lambda name:np.column_stack([col(f'{name}_{k}_0') for k in range(3)])
 mat=lambda name:np.array([[state[f'{name}_{a}_{b}'] for b in range(3)] for a in range(3)])
 R,E=mat('linearization_R'),mat('extrinsic_R');RL=R@E;points=vec('lidar');status=col('status').astype(int);acc=status==5;good=col('source_usable')==1
 near=(np.linalg.norm(points,axis=1)<8)&(np.linalg.norm(points,axis=1)>.7);height=points@ref
 floor=fit_plane(points[near&(height>-1.8)&(height<-.12)],ref)
 normals=vec('normal');src=vec('source_normal')@RL.T
 neighbors=np.column_stack([col(f'neighbors_{i}_{j}') for i in range(5) for j in range(3)]).reshape(-1,5,3)
 row={'scan_id':scan,'time':state['time'],'matches':int(acc.sum()),'pitch_deg':float(np.rad2deg(d['pitch'][j])),'forward':float(d['forward'][j])}
 fm=np.zeros(len(points),bool)
 if floor is not None:
  n=floor['normal'];nw=RL@n;fm=near&(abs(points@n+floor['offset'])<.05);ni=E@n
  mapped=np.unique(neighbors[fm&(col('neighbor_count')==5)].reshape(-1,3),axis=0);mp=fit_plane(mapped,nw,.05)
  row.update(floor_count=floor['count'],floor_rms=floor['rms'],floor_sensor_pitch_deg=float(np.rad2deg(np.arctan2(-ni[0],np.linalg.norm(ni[1:])))),floor_world_tilt_deg=float(angle(nw,np.array([0,0,1]))),floor_status_counts=np.bincount(status[fm],minlength=7).tolist(),floor_sensor_normal=n.tolist(),floor_world_normal=nw.tolist())
  if mp is not None:row.update(map_floor_world_tilt_deg=float(angle(mp['normal'],np.array([0,0,1]))),map_floor_angle_deg=float(angle(mp['normal'],nw)),map_floor_rms=mp['rms'])
  row['floor_accepted_unknown_fraction']=float((fm&acc&~good).sum()/max(1,(fm&acc).sum()))
  row['floor_map_normal_angle_median']=float(np.median(angle(normals[fm&acc],nw))) if (fm&acc).any() else None
 detailfile=file.with_name(file.name.replace('geometry-','detail-'));gd=np.load(detailfile);dd,dn=gd['data'],list(gd['columns']);dc=lambda n:dd[:,dn.index(n)]
 assert np.array_equal(col('match_index')[acc],dc('point_index'))
 H=np.column_stack([dc(f'H_0_{i}') for i in range(6)]);w=dc('inverse_variance');contr=np.column_stack([dc(f'K_{i}') for i in range(7,16)])*dc('innovation')[:,None]
 # Independent scan-floor orientation used only to choose a tangent direction for diagnostics.
 forward=RL[:,0].copy()
 if floor is not None:forward-=nw*(forward@nw)
 forward/=np.linalg.norm(forward)
 info=w*(H[:,3:]@forward)**2
 fmask=fm[acc];quality=good[acc];ang=col('normal_angle_deg')[acc]
 if floor is not None:
  row['floor_info_fraction']=float(info[fmask].sum()/max(1e-20,info.sum()))
  row['floor_bax_correction']=float(contr[fmask,6].sum())
  row['floor_gt30_accepted_count']=int((fm&acc&(angle(normals,nw)>30)).sum())
 row['source_good_fraction']=float(quality.mean());row['unknown_info_fraction']=float(info[~quality].sum()/max(1e-20,info.sum()))
 e,v=np.linalg.eigh(np.einsum('nki,nkj->nij',neighbors[acc]-neighbors[acc].mean(axis=1)[:,None],neighbors[acc]-neighbors[acc].mean(axis=1)[:,None])/5)
 row['map_thin_ratio_median']=float(np.median(e[:,0]/np.maximum(e[:,1],1e-20)))
 row['map_line_fraction']=float((e[:,1]/np.maximum(e[:,2],1e-20)<.1).mean())
 row['map_fit_max_median']=float(np.median(col('plane_fit_max')[acc]))
 row['coupled_information']={}
 # Compare the same usable points with map vs scan normals. This is diagnostic geometry, not a replacement update.
 pi=points[acc]@E.T+np.array([state[f'extrinsic_t_{i}_0'] for i in range(3)])
 sourceH=np.column_stack([np.cross(pi,src[acc]@R),src[acc]])
 for name,h in [('map',H),('source',sourceH)]:
  h=h[quality];ww=w[quality];A=h.T@(h*ww[:,None]);S=A[3:,3:]-A[3:,:3]@np.linalg.pinv(A[:3,:3])@A[:3,3:]
  row['coupled_information'][name]=float(forward@S@forward)
 return row
seen=set()
while True:
 for file in sorted((p/'out/samples').glob('geometry-*.npz')):
  if file.name in seen:continue
  target=done/(file.stem+'.json')
  if target.exists():seen.add(file.name);continue
  if not file.with_name(file.name.replace('geometry-','detail-')).exists():continue
  try:row=process(file)
  except (zipfile.BadZipFile,EOFError):continue
  target.write_text(json.dumps(row,allow_nan=False)+'\n');seen.add(file.name)
  if len(seen)%30==0:print('sample geometries',len(seen),'t',row['time'],flush=True)
 if (p/'out/all_point_groups.json').exists():break
 time.sleep(1)
rows=[json.loads(x.read_text()) for x in sorted(done.glob('geometry-*.json'))]
(p/'sample_geometry.json').write_text(json.dumps({'rows':rows},allow_nan=False)+'\n');print('complete',len(rows),flush=True)
