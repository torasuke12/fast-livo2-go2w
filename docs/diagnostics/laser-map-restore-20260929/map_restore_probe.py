import csv,json,os,signal,subprocess,sys,time
from pathlib import Path
import yaml,numpy as np
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Path as PathMsg

root=Path('/external');work=Path('/work'); label=sys.argv[1]; dest=work/label; dest.mkdir(exist_ok=True)
sys.path.insert(0,str(root/'scripts/fast_livo2'))
from reconstruction_capture import parameters
params=parameters(root/'humble_ws/src/fast-livo2/config/go2w_fast_livo2.yaml',capture=dest,img_en=0,temporal_window='on')
(dest/'logs').mkdir(exist_ok=True); (dest/'diagnostics').mkdir(exist_ok=True)
(dest/'params.yaml').write_text(yaml.safe_dump(params))
rclpy.init(); node=rclpy.create_node('temporal_output_probe'); received=[]; paths=[]; late=[]; late_transient=[]; late_paths=[]; voxel_history=[]
qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE)
def callback(msg):
 fields={f.name:f for f in msg.fields}; dtype=np.dtype({'names':['x','y','z','intensity'],'formats':['<f4']*4,'offsets':[fields[k].offset for k in ['x','y','z','intensity']],'itemsize':msg.point_step})
 a=np.frombuffer(msg.data,dtype=dtype,count=msg.width*msg.height)
 xyz=np.column_stack([a[k] for k in 'xyz']); finite=np.isfinite(xyz).all(axis=1)
 voxel_history.append(set(map(tuple,np.floor(xyz[finite].astype(np.float64)/.5).astype(np.int64))))
 received.append({'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,'points':len(a),'finite':int(finite.sum()),'frame':msg.header.frame_id,'intensity_minmax':[float(a['intensity'].min()),float(a['intensity'].max())],'extent':(xyz.max(axis=0)-xyz.min(axis=0)).tolist()})
sub=node.create_subscription(PointCloud2,'/Laser_map',callback,qos)
pathsub=node.create_subscription(PathMsg,'/path',lambda msg:paths.append({'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,'poses':len(msg.poses),'last_stamp':msg.poses[-1].header.stamp.sec+msg.poses[-1].header.stamp.nanosec*1e-9 if msg.poses else None}),qos)
procs=[]; handles=[]
def start(name,args):
 h=(dest/(name+'.log')).open('w');handles.append(h);p=subprocess.Popen(args,stdout=h,stderr=subprocess.STDOUT,start_new_session=True);procs.append(p);return p
def stop(p):
 if p.poll() is None:
  os.killpg(p.pid,signal.SIGINT)
  try:p.wait(timeout=10)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
try:
 binary=root/'humble_ws/install/fast_livo2/lib/fast_livo2/fastlivo2_mapping'
 estimator=start('node',[str(binary),'--ros-args','--params-file',str(dest/'params.yaml')]);time.sleep(2)
 player=start('player',['ros2','bag','play',str(root/'bags/raw_20260907_114335'),'--rate','.25','--clock','--delay','2','--disable-keyboard-controls','--topics','/livox/lidar','/livox/imu'])
 deadline=time.monotonic()+100; first_stamp=None
 while time.monotonic()<deadline and estimator.poll() is None:
  rclpy.spin_once(node,timeout_sec=.1)
  if paths:
   if first_stamp is None:first_stamp=paths[0]['last_stamp']
   if paths[-1]['last_stamp']-first_stamp>=9:break
 else:raise RuntimeError('Probe timed out or estimator exited')
 stop(player)
 for i in range(30):rclpy.spin_once(node,timeout_sec=.1)
 old_count=len(received)
 late_sub=node.create_subscription(PointCloud2,'/Laser_map',lambda msg:late.append(msg.width*msg.height),qos)
 transient=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
 late_transient_sub=node.create_subscription(PointCloud2,'/Laser_map',lambda msg:late_transient.append(msg.width*msg.height),transient)
 late_path_sub=node.create_subscription(PathMsg,'/path',lambda msg:late_paths.append(len(msg.poses)),transient)
 deadline=time.monotonic()+4
 while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
 result={'label':label,'maps':received,'path_first':paths[0],'path_last':paths[-1],'late_volatile_messages':late,'late_transient_messages':late_transient,'maps_while_paused':len(received)-old_count,'rviz_render_tested':False,'late_transient_path_poses':late_paths}
 lost=[len(x-y) for x,y in zip(voxel_history,voxel_history[1:])]
 result['voxel_keys_lost_between_updates']=lost
 result['first_map_voxels']=len(voxel_history[0])
 result['first_map_voxels_retained_at_end']=len(voxel_history[0]&voxel_history[-1])
 result['map_span_seconds']=received[-1]['stamp']-received[0]['stamp']
 assert result['map_span_seconds']>6, 'probe did not outlast the estimator window'
 assert not any(lost), 'cumulative map dropped previously published voxels'
 assert result['first_map_voxels_retained_at_end']==result['first_map_voxels']
 assert len(late_transient)==0 and len(late)==0, 'map durability was not restored to volatile'
 assert len(late_paths)==1, 'trajectory publisher changed unexpectedly'
 assert len(received)==old_count, 'new maps were published during paused check'
 with (dest/'logs/temporal_states.csv').open() as f: state_rows=list(csv.DictReader(f))
 keys={int(r['node_id']) for r in state_rows if int(r['keyframe'])}
 expected=len(keys)+(0 if int(state_rows[-1]['keyframe']) else 1)
 assert paths[-1]['poses']==expected
 result['expected_final_path_poses']=expected
 (dest/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2),flush=True)
 assert received and all(m['points']>0 and m['points']==m['finite'] for m in received)
finally:
 for p in reversed(procs):stop(p)
 for h in handles:h.close()
 node.destroy_node();rclpy.shutdown()
