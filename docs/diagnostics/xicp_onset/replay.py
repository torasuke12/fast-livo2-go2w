import argparse
import collections
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy
import hashlib, struct

p = argparse.ArgumentParser()
p.add_argument('name')
p.add_argument('--bag', default='raw_20260907_114335')
p.add_argument('--duration', type=float, default=70)
p.add_argument('--rate', type=float, default=1)
p.add_argument('--livo', action='store_true')
p.add_argument('--diagnostic', action='store_true')
p.add_argument('--variant', choices=['seedguard', 'cutfix'])
p.add_argument('--param', action='append', default=[])
p.add_argument('--binary')
p.add_argument('--params-file')
a = p.parse_args()
out = Path('/validation') / a.name
out.mkdir(exist_ok=False)
rclpy.init()
monitor = rclpy.create_node('comparison_monitor')
clocks = []
odom = []
imu_received = []
imu_hash = hashlib.sha256()
def on_imu(msg):
    stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
    imu_received.append(stamp)
    imu_hash.update(struct.pack("<q6d", stamp, msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z, msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z))
monitor.create_subscription(Imu, "/livox/imu", on_imu, QoSProfile(depth=2048, reliability=ReliabilityPolicy.RELIABLE))
def on_clock(msg):
    t = msg.clock.sec + msg.clock.nanosec * 1e-9
    if t > 0:
        if not clocks: clocks.append(t)
        if len(clocks) == 1: clocks.append(t)
        else: clocks[-1] = t
def on_odom(msg):
    q = msg.pose.pose
    odom.append([msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                 q.position.x, q.position.y, q.position.z,
                 q.orientation.x, q.orientation.y, q.orientation.z, q.orientation.w])
monitor.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)
monitor.create_subscription(Odometry, '/aft_mapped_to_init', on_odom, qos_profile_sensor_data)
cmd = ['/external/humble_ws/install/fast_livo2/lib/fast_livo2/fastlivo2_mapping',
       '--ros-args', '-r', '__node:=laserMapping', '--params-file',
       '/external/humble_ws/src/fast-livo2/config/go2w_fast_livo2.yaml',
       '-p', 'use_sim_time:=true', '-p', 'common.reliable_sensor_qos:=true',
       '-p', 'common.img_en:=' + ('1' if a.livo else '0')]
for param in a.param: cmd += ['-p', param]
if a.binary: cmd[0] = a.binary
if a.params_file: cmd[5] = a.params_file
if a.diagnostic: cmd[0] = '/validation/fastlivo2_diagnostic'
if a.variant:
    cmd[0] = '/validation/fastlivo2_mapper_diagnostic'
    os.environ['DIAG_SEED_GUARD' if a.variant == 'seedguard' else 'DIAG_CUT_FIX'] = '1'
topics = ['/livox/lidar', '/livox/imu']
if a.livo: topics += ['/camera/camera/color/image_raw', '/camera/camera/color/camera_info']
player_cmd = ['ros2', 'bag', 'play', '/external/bags/' + a.bag, '--clock',
              '--delay', '2', '--rate', str(a.rate), '--disable-keyboard-controls', '--topics'] + topics
processes = []
handles = []
reason = 'timeout'
start = time.monotonic()
try:
    for command, name in [(cmd, 'node'), (player_cmd, 'player')]:
        f = open(out / (name + '.log'), 'w')
        handles.append(f)
        processes.append(subprocess.Popen(command, stdout=f, stderr=subprocess.STDOUT, start_new_session=True))
        if name == 'node':
            until = time.monotonic() + 2
            while time.monotonic() < until: rclpy.spin_once(monitor, timeout_sec=0.1)
    while time.monotonic() - start < a.duration / a.rate + 35:
        rclpy.spin_once(monitor, timeout_sec=0.1)
        if processes[0].poll() is not None:
            reason = 'node_exit'; break
        if len(clocks) == 2 and clocks[-1] - clocks[0] >= a.duration:
            reason = 'duration'; break
finally:
    for proc in reversed(processes):
        if proc.poll() is None: os.killpg(proc.pid, signal.SIGINT)
        try: proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL); proc.wait()
    for f in handles: f.close()
    monitor.destroy_node()
    rclpy.shutdown()
for name in ['mat_out.txt', 'mat_pre.txt', 'imu.txt', 'xicp_diagnostic.csv']:
    src = Path('/external/humble_ws/src/fast-livo2/Log') / name
    if src.exists(): shutil.copy2(src, out / name)
rows = []
for line in (out / 'mat_out.txt').read_text().splitlines():
    try: rows.append([float(x) for x in line.split()])
    except ValueError: pass
rows = [row for row in rows if len(row) == 20]
def norm(v): return math.sqrt(sum(x*x for x in v))
finite = [row for row in rows if all(math.isfinite(x) for x in row)]
features = re.findall(r'downsampled feature num:(\d+) effective feature num: (\d+) average residual: ([\deE.+-]+)', (out / 'node.log').read_text())
summary = dict(name=a.name, args=vars(a), reason=reason, exit_codes=[x.returncode for x in processes],
               clock_elapsed=clocks[-1]-clocks[0] if clocks else None,
               states=len(rows), nonfinite=len(rows)-len(finite), odom_count=len(odom),
               first=rows[0] if rows else None, last=rows[-1] if rows else None,
               checkpoints={str(t): min(finite, key=lambda r: abs(r[0]-t)) for t in [3.2,5,10,20,40,60,65,70] if finite},
               max_speed=max((norm(row[7:10]) for row in finite), default=None),
               first10_max_position=max((norm(row[4:7]) for row in finite if row[0]<10), default=None),
               first_features=features[:12])
summary['received_imu_count'] = len(imu_received)
summary['received_imu_sha256'] = imu_hash.hexdigest()
(out / 'received_imu_stamps.txt').write_text(''.join(str(t) + '\n' for t in imu_received))
(out / 'summary.json').write_text(json.dumps(summary, indent=2))
print(json.dumps(summary, indent=2), flush=True)
