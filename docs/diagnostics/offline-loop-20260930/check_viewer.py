"""Run inside the isolated ROS container; verify the actual saved-map publisher."""
import json, pathlib, signal, subprocess, sys, time
import numpy as np
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Path
out=pathlib.Path('/result');log=(out/'viewer.log').open('w')
process=subprocess.Popen([sys.executable,'/external/scripts/fast_livo2/view_reconstruction.py','/external/reconstruction_results/raw_20260907_114335'],stdout=log,stderr=subprocess.STDOUT)
received={}
rclpy.init();node=rclpy.create_node('reconstruction_view_check')
qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
def cloud(m):
 assert m.header.frame_id=='camera_init' and m.point_step==12 and len(m.data)==m.width*12
 assert np.isfinite(np.frombuffer(m.data,dtype=np.float32)).all()
 received['map_points']=m.width
def path(m):
 assert m.header.frame_id=='camera_init'
 received['trajectory_poses']=len(m.poses)
node.create_subscription(PointCloud2,'/Laser_map',cloud,qos)
node.create_subscription(Path,'/path',path,qos)
try:
 deadline=time.monotonic()+30
 while len(received)<2 and time.monotonic()<deadline:
  if process.poll() is not None:raise RuntimeError('Viewer exited early')
  rclpy.spin_once(node,timeout_sec=.2)
 assert received=={'map_points':909037,'trajectory_poses':1215},received
finally:
 process.send_signal(signal.SIGINT);process.wait(timeout=10)
 node.destroy_node();rclpy.shutdown();log.close()
received['returncode']=process.returncode
(out/'viewer-check.json').write_text(json.dumps(received,indent=2))
assert process.returncode==0
print(received)
