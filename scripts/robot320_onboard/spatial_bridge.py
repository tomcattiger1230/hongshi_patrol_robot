#!/usr/bin/env python3
"""Read-only ROS sensor/map to String DDS bridge; no command publishers."""
import base64
import json
import math
import time
import zlib
from pathlib import Path
import subprocess

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


class SpatialBridge(Node):
    def __init__(self):
        super().__init__('robot320_readonly_spatial_bridge')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.last_scan = 0.0
        self.scan_pub = self.create_publisher(String, '/robot320/spatial_scan', 2)
        self.pose_pub = self.create_publisher(String, '/robot320/spatial_pose', 2)
        self.map_pub = self.create_publisher(String, '/robot320/spatial_map', 2)
        self.create_subscription(PointCloud2, '/filtered_points', self.scan, qos_profile_sensor_data)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.map, map_qos)
        self.latest_map = None
        self.mapping_status_pub = self.create_publisher(String, '/robot320/mapping_status', 2)
        self.create_subscription(String, '/robot320/mapping_control', self.mapping_control, 2)
        self.create_timer(2.0, self.mapping_status)
        self.create_timer(2.0, self.send_map)
        self.create_timer(0.2, self.pose)

    def send(self, publisher, data):
        message = String()
        message.data = json.dumps(data, allow_nan=False, separators=(',', ':'))
        publisher.publish(message)

    def transform(self, target, source, stamp=None):
        return self.tf.lookup_transform(target, source, stamp or rclpy.time.Time()).transform

    def scan(self, msg):
        if time.monotonic() - self.last_scan < 0.2:
            return
        self.last_scan = time.monotonic()
        points = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        points = points[np.isfinite(points).all(axis=1)]
        points = points[::max(1, math.ceil(len(points) / 1200))]
        frame = msg.header.frame_id
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        # Transform at sensor acquisition time. Never mix unmatched frames.
        for target in ('map', 'odom'):
            try:
                t = self.transform(target, frame, stamp)
                q = t.rotation
                x, y, z, w = q.x, q.y, q.z, q.w
                rotation = np.array([
                    [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                    [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                    [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
                ])
                points = points @ rotation.T + [t.translation.x, t.translation.y, t.translation.z]
                frame = target
                break
            except Exception:
                continue
        self.send(self.scan_pub, {'frame': frame, 'points': np.round(points[:, :2].astype(float), 3).tolist()})

    def pose(self):
        for target in ('map', 'odom'):
            try:
                t = self.transform(target, 'base_link')
                q = t.rotation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                self.send(self.pose_pub, {'frame': target, 'pose': [t.translation.x, t.translation.y, yaw]})
                break
            except Exception:
                continue

    def map(self, msg):
        if msg.info.width * msg.info.height > 1_000_000:
            self.get_logger().warning('Map exceeds preview cell budget; not sent')
            return
        q = msg.info.origin.orientation
        self.latest_map = {
            'frame': msg.header.frame_id, 'width': msg.info.width, 'height': msg.info.height,
            'resolution': msg.info.resolution,
            'origin': [msg.info.origin.position.x, msg.info.origin.position.y,
                       math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))],
            'cells': base64.b64encode(zlib.compress(bytes((int(v)+1 for v in msg.data)))).decode(),
        }
        self.send_map()

    def send_map(self):
        if self.latest_map:
            self.send(self.map_pub, self.latest_map)

    def mapping_status(self, message=None):
        available = (Path.home() / '.config/systemd/user/robot320-spatial-mapping.service').is_file()
        state = subprocess.run(['systemctl', '--user', 'is-active', 'robot320-spatial-mapping.service'],
                               capture_output=True, text=True, timeout=3).stdout.strip()
        self.send(self.mapping_status_pub, {'available': available, 'state': state,
                  'message': message or f'车上建图服务：{state or "未安装"}（独立雷达建图，不控制车辆）'})

    def mapping_control(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get('action') not in ('start', 'stop') or abs(time.time()-float(data['stamp'])) > 5:
                raise ValueError('invalid or expired mapping request')
            # Fixed service allowlist: no shell, arbitrary command, or vehicle control.
            result = subprocess.run(['systemctl', '--user', data['action'], 'robot320-spatial-mapping.service'],
                                    capture_output=True, text=True, timeout=6)
            self.mapping_status('建图请求已处理，等待地图和服务状态' if result.returncode == 0
                                else '建图服务请求失败，请检查车上 journal')
        except (ValueError, TypeError, KeyError, AttributeError, subprocess.TimeoutExpired):
            self.mapping_status('建图请求无效、已过期或执行超时')


def main():
    rclpy.init()
    node = SpatialBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
