"""Read-only camera/input compatibility audit inside the pinned ROS image."""
import json
from pathlib import Path
import sys
import yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

package=Path('/package')
manifest=json.loads((package/'provenance.json').read_text())
params=yaml.safe_load((package/'params/current_C.yaml').read_text())['laserMapping']['ros__parameters']
reports={}
for name in manifest['bags']:
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri='/inputs/'+name,storage_id=''),rosbag2_py.ConverterOptions('',''))
    types={x.name:x.type for x in reader.get_all_topics_and_types()}
    for key,typ in [('lid_topic','sensor_msgs/msg/PointCloud2'),('imu_topic','sensor_msgs/msg/Imu'),('img_topic','sensor_msgs/msg/Image')]:
        if types.get(params['common'][key])!=typ: raise RuntimeError(f'{name}: missing/wrong {key}')
    info_topics=[k for k,v in types.items() if v=='sensor_msgs/msg/CameraInfo']
    if len(info_topics)!=1: raise RuntimeError(f'{name}: expected exactly one camera_info topic')
    topic=info_topics[0]
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    if not reader.has_next(): raise RuntimeError(f'{name}: empty camera_info')
    _,data,stamp=reader.read_next(); info=deserialize_message(data,get_message(types[topic]))
    expected=params['camera']
    if (info.width,info.height)!=(expected['cam_width'],expected['cam_height']): raise RuntimeError(f'{name}: camera resolution mismatch')
    for key,value in [('cam_fx',info.k[0]),('cam_fy',info.k[4]),('cam_cx',info.k[2]),('cam_cy',info.k[5])]:
        if abs(expected[key]-value)>1e-3: raise RuntimeError(f'{name}: camera intrinsics mismatch: {key}')
    for i,value in enumerate(info.d):
        if abs(expected.get('cam_d'+str(i),0.)-value)>1e-6: raise RuntimeError(f'{name}: camera distortion mismatch')
    reports[name]=dict(topic=topic,first_camera_info_ns=stamp,width=info.width,height=info.height,k=list(info.k),d=list(info.d),topics=types)
(package/'input_compatibility.json').write_text(json.dumps(reports,indent=2)+'\n')
