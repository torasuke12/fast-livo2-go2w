#!/usr/bin/env python3
"""Publish an exported map and trajectory for the existing RViz config.

Run in the ROS environment. Files are displayed unchanged, including online
exports. No estimator or optimization runs. Only numpy and ROS are required.
"""
import argparse
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from sensor_msgs.msg import PointCloud2,PointField


def read_xyz(path):
    with path.open('rb') as f:
        if f.readline().strip()!=b'ply':
            raise ValueError('Not a PLY file')
        properties=[];count=None;fmt=None;vertex=False
        for line in f:
            tokens=line.decode('ascii').strip().split()
            if not tokens:
                continue
            if tokens[0]=='format':fmt=tokens[1]
            if tokens[0]=='element':
                vertex=tokens[1]=='vertex'
                if vertex:count=int(tokens[2])
            if tokens[0]=='property' and vertex:properties.append((tokens[2],tokens[1]))
            if tokens[0]=='end_header':break
        if fmt!='binary_little_endian' or count is None:
            raise ValueError('Expected binary little-endian Open3D PLY')
        kinds={'float':'<f4','double':'<f8','uchar':'u1','uint8':'u1'}
        points=np.fromfile(f,dtype=[(name,kinds[kind]) for name,kind in properties],count=count)
        if len(points)!=count:raise ValueError('Truncated PLY')
        return np.column_stack([points[n] for n in ('x','y','z')]).astype('<f4')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reconstruction',type=Path)
    args=parser.parse_args()
    xyz=read_xyz(args.reconstruction/'map.ply')
    poses=np.loadtxt(args.reconstruction/'trajectory.tum',ndmin=2)
    if not np.isfinite(xyz).all() or not np.isfinite(poses).all():
        raise ValueError('Nonfinite reconstruction')
    rclpy.init()
    node=rclpy.create_node('offline_reconstruction_view')
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    pub=node.create_publisher(PointCloud2,'/Laser_map',qos)
    path_pub=node.create_publisher(RosPath,'/path',qos)
    msg=PointCloud2()
    msg.header.frame_id='camera_init';msg.height=1;msg.width=len(xyz)
    msg.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z'))]
    msg.is_bigendian=False;msg.is_dense=True;msg.point_step=12;msg.row_step=12*len(xyz);msg.data=xyz.tobytes()
    path=RosPath();path.header.frame_id='camera_init'
    for row in poses:
        p=PoseStamped();p.header.frame_id='camera_init'
        p.pose.position.x,p.pose.position.y,p.pose.position.z=map(float,row[1:4])
        p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w=map(float,row[4:8])
        path.poses.append(p)
    def publish():
        msg.header.stamp=node.get_clock().now().to_msg();path.header.stamp=msg.header.stamp
        pub.publish(msg);path_pub.publish(path)
    timer=node.create_timer(2.,publish)
    publish()
    node.get_logger().info(f'Publishing {len(xyz)} map points and {len(poses)} poses; open the repository RViz config.')
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_timer(timer);node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
