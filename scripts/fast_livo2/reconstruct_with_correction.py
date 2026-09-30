#!/usr/bin/env python3
"""Run fresh FAST-LIVO2 and offline correction while RViz shows both stages.

Invoked by reconstruct_offline.sh after it prepares params/provenance. This is
batch loop correction: corrections become available after the whole bag ends.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import textwrap
import time

import numpy as np
import open3d as o3d
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path as RosPath
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from correction_preview import KeyframePreview, validate_correction
from view_reconstruction import read_xyz

REPO = Path(__file__).resolve().parents[2]
PREFIX = '/offline_correction/'


def cloud_message(xyz):
    xyz = np.asarray(xyz, dtype='<f4')
    msg = PointCloud2()
    msg.header.frame_id = 'camera_init'
    msg.height, msg.width = 1, len(xyz)
    msg.fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(('x', 'y', 'z'))]
    msg.is_dense, msg.is_bigendian = True, False
    msg.point_step, msg.row_step, msg.data = 12, 12*len(xyz), xyz.tobytes()
    return msg


def path_message(rows):
    msg = RosPath()
    msg.header.frame_id = 'camera_init'
    for row in rows:
        p = PoseStamped()
        p.header.frame_id = 'camera_init'
        stamp = float(row[0])
        p.header.stamp.sec = int(stamp)
        p.header.stamp.nanosec = int((stamp-int(stamp))*1e9)
        p.pose.position.x, p.pose.position.y, p.pose.position.z = map(float, row[1:4])
        p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = map(float, row[4:8])
        msg.poses.append(p)
    return msg


class Reconstruction(Node):
    def __init__(self, args):
        super().__init__('offline_correction_reconstruction')
        self.args = args
        self.output = args.output.resolve()
        self.preview = KeyframePreview(self.output / 'frontend')
        self.stage, self.detail = 'starting', 'Starting fresh estimation'
        self.done, self.exit_code = False, 0
        self.children, self.logs = [], []
        self.worker, self.rviz = None, None
        self.xyz = np.empty((0, 3))
        self.current_map = self.before_map = cloud_message(self.xyz)
        self.current_path = self.before_path = path_message([])
        self.vectors = Marker()
        self.vectors.header.frame_id = 'camera_init'
        self.vectors.ns, self.vectors.id = 'correction', 0
        self.vectors.type, self.vectors.action = Marker.LINE_LIST, Marker.ADD
        self.vectors.pose.orientation.w = 1.
        self.vectors.scale.x = .025
        self.vectors.color.r, self.vectors.color.b, self.vectors.color.a = 1., 1., .7
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        types = dict(map=PointCloud2, before_map=PointCloud2, path=RosPath,
                     before_path=RosPath, status=Marker, state=String, displacements=Marker)
        self.pubs = {name: self.create_publisher(kind, PREFIX+name, qos) for name, kind in types.items()}

    def launch(self, command, log):
        handle = (self.output / log).open('w')
        self.logs.append(handle)
        child = subprocess.Popen(list(map(str, command)), stdout=handle, stderr=subprocess.STDOUT,
                                 start_new_session=True)
        self.children.append(child)
        return child

    def start(self):
        if (self.output / 'frontend').exists() or (self.output / 'optimized').exists():
            raise ValueError('Fresh output is required; existing frontend/optimized directories are refused')
        if not self.args.no_rviz:
            self.rviz = self.launch(['rviz2', '-d', REPO/'config/fast_livo2/offline_correction.rviz',
                                     '--ros-args', '-p', 'use_sim_time:=false'], 'rviz.log')
        self.worker = self.launch([
            REPO/'humble_ws/install/fast_livo2/lib/fast_livo2/fastlivo2_offline',
            self.args.bag.resolve(), self.output/'frontend', '--ros-args',
            '--params-file', self.output/'params.yaml', '--log-level', 'error'], 'frontend.log')
        self.set_stage('estimating', 'LIO/LIVO: building uncorrected map')
        self.timer = self.create_timer(1., self.tick)

    def set_stage(self, stage, detail):
        self.stage, self.detail = stage, detail
        event = dict(stage=stage, detail=detail, frontend_seconds=self.preview.time,
                     keyframes=len(self.preview.rows), wall_time=time.time())
        with (self.output/'rviz_events.jsonl').open('a') as f:
            f.write(json.dumps(event)+'\n')
        (self.output/'rviz_state.json').write_text(json.dumps(event, indent=2))
        self.get_logger().info(detail)

    def update_preview(self):
        new = self.preview.update()
        if not len(new):
            return
        points = np.concatenate((self.xyz, new))
        cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        # Preview density only; full saved scans are used by the optimizer.
        self.xyz = np.asarray(cloud.voxel_down_sample(.15).points).copy()
        self.before_map = cloud_message(self.xyz)
        self.before_path = path_message(self.preview.trajectory)
        self.current_map, self.current_path = self.before_map, self.before_path

    def apply_correction(self):
        root = self.output/'optimized'
        summary, trajectory = validate_correction(self.preview, root)
        xyz = read_xyz(root/'map.ply')
        if not np.isfinite(xyz).all() or len(xyz) != summary['map_points']:
            raise ValueError('Invalid corrected map')
        # Replace the entire snapshot: no uncorrected points remain in this topic.
        self.current_map, self.current_path = cloud_message(xyz), path_message(trajectory)
        for before, after in zip(self.preview.trajectory[::10, 1:4], trajectory[::10, 1:4]):
            for xyz in (before, after):
                self.vectors.points.append(Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2])))
        self.set_stage('corrected', f'CORRECTED: {summary["accepted_loops"]} loop constraints, '
                       f'{summary["confirmed_return_constraints"]} verified return matches. '
                       'Cyan path = corrected; orange = before.')

    def tick(self):
        try:
            if self.rviz is not None and self.rviz.poll() is not None:
                if self.rviz.returncode != 0:
                    raise RuntimeError('RViz failed; see rviz.log')
                if self.stage not in ('corrected', 'failed'):
                    self.exit_code = 130
                    self.set_stage('cancelled', 'RViz closed before correction completed')
                self.done = True
                return
            if self.stage == 'estimating':
                self.update_preview()
                code = self.worker.poll()
                if code is not None:
                    if code:
                        raise RuntimeError(f'Frontend exited {code}; see frontend.log')
                    self.update_preview()
                    summary = json.loads((self.output/'frontend/summary.json').read_text())
                    if not summary.get('complete') or not summary.get('bag_exhausted'):
                        raise RuntimeError('Frontend did not finish the whole bag')
                    if summary['keyframes'] != len(self.preview.rows):
                        raise RuntimeError('Preview did not consume all frontend keyframes')
                    command = [sys.executable, '-u', REPO/'scripts/fast_livo2/optimize_reconstruction.py',
                               self.output/'frontend', self.output/'optimized']
                    if self.args.return_to_start:
                        command.append('--return-to-start')
                    self.worker = self.launch(command, 'optimization.log')
                    self.set_stage('optimizing', 'OFFLINE OPTIMIZATION: matching revisits; showing uncorrected map')
            elif self.stage == 'optimizing':
                code = self.worker.poll()
                if code is not None:
                    if code:
                        raise RuntimeError(f'Optimizer exited {code}; correction NOT applied. See optimization.log')
                    self.apply_correction()
            self.publish()
            if self.args.exit_when_complete and self.stage in ('corrected', 'failed'):
                self.done = True
        except Exception as exc:
            self.exit_code = 1
            self.set_stage('failed', str(exc))
            self.publish()
            if self.args.no_rviz or (self.rviz is not None and self.rviz.poll() is not None):
                self.done = True

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        for name, msg in dict(map=self.current_map, before_map=self.before_map, path=self.current_path,
                              before_path=self.before_path, displacements=self.vectors).items():
            msg.header.stamp = stamp
            self.pubs[name].publish(msg)
        marker = Marker()
        marker.header.frame_id, marker.header.stamp = 'camera_init', stamp
        marker.ns, marker.id, marker.type, marker.action = 'stage', 0, Marker.TEXT_VIEW_FACING, Marker.ADD
        marker.pose.position.z, marker.pose.orientation.w, marker.scale.z = 4., 1., .45
        marker.color.r = marker.color.a = 1.
        marker.color.g = marker.color.b = 0. if self.stage == 'failed' else 1.
        marker.text = textwrap.fill(self.detail, width=65) + f'\nSensor time: {self.preview.time:.1f}s / {len(self.preview.rows)} keyframes'
        self.pubs['status'].publish(marker)
        self.pubs['state'].publish(String(data=json.dumps(dict(stage=self.stage, detail=self.detail,
                                                              keyframes=len(self.preview.rows)))))

    def stop(self):
        active = [child for child in self.children if child.poll() is None]
        for child in active:
            try:
                os.killpg(child.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        for child in active:
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
        for handle in self.logs:
            handle.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--return-to-start', action='store_true')
    parser.add_argument('--no-rviz', action='store_true', help='For headless integration tests')
    parser.add_argument('--exit-when-complete', action='store_true', help='For headless integration tests')
    args = parser.parse_args()
    rclpy.init()
    node = Reconstruction(args)
    try:
        # Discover conflicting runs before starting any estimator subprocess.
        deadline = time.monotonic()+2
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        names = node.get_node_names()
        if 'laserMapping' in names or names.count(node.get_name()) > 1:
            raise RuntimeError('Reconstruction already exists in this ROS domain; stop it or select another domain')
        node.start()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        if node.stage not in ('corrected', 'failed'):
            node.exit_code = 130
            node.set_stage('cancelled', 'Interrupted; incomplete correction was not applied')
    except Exception as exc:
        node.exit_code = 1
        node.set_stage('failed', str(exc))
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return node.exit_code


if __name__ == '__main__':
    sys.exit(main())
