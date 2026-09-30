import csv,json,sqlite3,sys
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu,PointCloud2
capture=Path(sys.argv[1]);bag=Path(sys.argv[2]);output=Path(sys.argv[3])
connection=sqlite3.connect('file:'+str(next(bag.glob('*.db3')))+'?mode=ro',uri=True)
topics=dict(connection.execute('SELECT name,id FROM topics'))
lidar_data=connection.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp LIMIT 1',(topics['/livox/lidar'],)).fetchone()[0]
lidar=deserialize_message(lidar_data,PointCloud2)
first_lidar_ns=lidar.header.stamp.sec*10**9+lidar.header.stamp.nanosec
stamps=[];values=[]
for (data,) in connection.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp',(topics['/livox/imu'],)):
 m=deserialize_message(data,Imu)
 stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
 stamps.append(stamp)
 values.append([m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z,m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z])
stamps=np.array(stamps,dtype=np.int64);values=np.array(values)
received=np.array([int(r['stamp_ns']) for r in csv.DictReader((capture/'imu_stamps.csv').open())],dtype=np.int64)
raw_received=stamps[(stamps>=received[0])&(stamps<=received[-1])]
processed=np.loadtxt(capture/'logs/imu.txt')
# rclcpp::Time::seconds() divides integer nanoseconds by 1e9 (verified in the installed library).
# imu_cbk applies stampFromSec(stampToSec(stamp) - imu_time_offset), even for offset 0.
node_stamps=np.rint(stamps.astype(float)/1e9*1e9).astype(np.int64)
relative=node_stamps.astype(float)/1e9-float(first_lidar_ns)/1e9
rounded=np.array([float(format(t,'.6g')) for t in relative])
start=int(np.argmin(np.abs(rounded-processed[0,0])))
expected=np.column_stack([rounded[start:start+len(processed)],.5*(values[start:start+len(processed)]+values[start+1:start+len(processed)+1])])
for k in range(1,7):expected[:,k]=[float(format(v,'.6g')) for v in expected[:,k]]
reference=np.loadtxt('/external/diagnostic_captures/voxel-plane-analysis-20260928/data/logs/imu.txt')
n=min(len(reference),len(processed))
result=dict(received_count=len(received),raw_count_between_received_endpoints=len(raw_received),
 received_stamps_equal_raw=bool(np.array_equal(received,raw_received)),
 processed_count=len(processed),processed_start=float(processed[0,0]),processed_end=float(processed[-1,0]),
 processed_timestamps_equal_raw=bool(np.array_equal(processed[:,0],expected[:,0])),
 processed_all_columns_equal_raw_midpoints=bool(np.array_equal(processed,expected)),
 processed_max_abs_vs_raw=float(np.max(np.abs(processed-expected))),
 processed_sensor_midpoints_exact=bool(np.array_equal(processed[:,1:],expected[:,1:])),
 processed_timestamp_mismatch_count=int(np.count_nonzero(processed[:,0]!=expected[:,0])),
 reference_prefix_rows=n,reference_prefix_all_columns_exact=bool(np.array_equal(processed[:n],reference[:n])),
 max_processed_gap_sec=float(np.max(np.diff(processed[:,0]))),max_raw_gap_sec=float(np.max(np.diff(raw_received)))*1e-9)
output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
