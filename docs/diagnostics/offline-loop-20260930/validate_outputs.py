"""Recheck saved full/short frontend parity and the completed offline artifacts."""
import hashlib
import json
from pathlib import Path
import numpy as np

repo=Path(__file__).resolve().parents[3]
work=repo/'diagnostic_captures/offline-20260930'
output=repo/'reconstruction_results/raw_20260907_114335'
full=np.genfromtxt(work/'window-lio/logs/temporal_window.csv',delimiter=',',names=True,dtype=None,encoding='utf8')
short=np.genfromtxt(work/'final-short-v2/logs/temporal_window.csv',delimiter=',',names=True,dtype=None,encoding='utf8')
n=len(short)
assert np.array_equal(full['time'][:n],short['time'])
assert np.array_equal(full['status'][:n],short['status'])
numeric=[name for name in full.dtype.names if name not in ('status','solve_ms')]
differences={name:float(np.max(np.abs(full[name][:n]-short[name]))) for name in numeric}
assert max(differences.values())<1e-5
trajectory=np.load(output/'trajectory.npz')
poses=trajectory['corrected']
assert np.isfinite(poses).all()
rot=poses[:,:3,:3]
rotation_error=float(np.max(np.abs(rot@rot.transpose(0,2,1)-np.eye(3))))
assert rotation_error<1e-8
assert np.min(np.linalg.det(rot))>.99999999
summary=json.loads((work/'window-lio/summary.json').read_text())
assert summary['bag_exhausted'] and summary['complete']
assert summary['imu_messages']==129619 and summary['lidar_messages']==6481
assert summary['updates']==6448 and summary['last_time']>648
backend=json.loads((output/'summary.json').read_text())
assert backend['complete'] and backend['confirmed_return_constraints']==15
assert all(v['status']>0 for v in backend['optimization'].values())
metrics=json.loads((output/'room_metrics.json').read_text())
assert metrics['after']['return_to_start']['inlier_plane_rmse_m']<.04
assert metrics['after']['return_to_start']['nearest_median_m']<.3
viewer=json.loads((work/'viewer-check.json').read_text())
assert viewer==dict(map_points=909037,trajectory_poses=1215,returncode=0)
result=dict(frontend=summary,backend=backend,final_short_common_rows=n,
            max_short_full_numeric_difference=max(differences.values()),rotation_orthogonality_error=rotation_error,
            room_metrics=metrics,viewer=viewer)
files=[output/'map.ply',output/'trajectory.tum',output/'comparison.png',output/'frontend_params.yaml',
       repo/'humble_ws/build/fast_livo2/fastlivo2_offline',repo/'humble_ws/build/fast_livo2/fastlivo2_mapping']
files+=list((repo/'scripts/fast_livo2').glob('*reconstruction*.py'))
files += [repo/'scripts/fast_livo2/reconstruct_offline.sh',repo/'humble_ws/src/fast-livo2/src/offline.cpp']
result['sha256']={str(p.relative_to(repo)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
(Path(__file__).parent/'validation.json').write_text(json.dumps(result,indent=2))
(output/'validation.json').write_text(json.dumps(result,indent=2))
print(json.dumps({key:result[key] for key in ('final_short_common_rows','max_short_full_numeric_difference','rotation_orthogonality_error','viewer')},indent=2))
