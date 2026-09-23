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
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from std_srvs.srv import Empty
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
        self.last_map_update = 0.0
        self.last_map_shape = None
        self.mapping_status_pub = self.create_publisher(String, '/robot320/mapping_status', 2)
        self.mapping_status_sequence = 0
        self.global_localization_client = self.create_client(
            Empty, '/reinitialize_global_localization'
        )
        self.nomotion_update_client = self.create_client(Empty, '/request_nomotion_update')
        self.nomotion_updates_remaining = 0
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.localization_uncertainty = None
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose, 10)
        self.create_subscription(String, '/robot320/mapping_control', self.mapping_control, 2)
        self.create_timer(2.0, self.mapping_status)
        self.create_timer(2.0, self.send_map)
        self.create_timer(0.2, self.pose)
        self.create_timer(1.0, self.nomotion_update)

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
        now = time.monotonic()
        if now - self.last_map_update < 2.0:
            return
        self.last_map_update = now
        width, height = msg.info.width, msg.info.height
        cells = np.asarray(msg.data, dtype=np.int16).reshape(height, width)
        factor = max(1, math.ceil(math.sqrt(width * height / 1_000_000)))
        if factor > 1:
            out_width, out_height = math.ceil(width / factor), math.ceil(height / factor)
            padded = np.full((out_height * factor, out_width * factor), -1, dtype=np.int16)
            padded[:height, :width] = cells
            # Conservative preview: retain the most occupied cell in each block.
            cells = padded.reshape(out_height, factor, out_width, factor).max(axis=(1, 3))
            shape = (width, height, out_width, out_height, factor)
            if shape != self.last_map_shape:
                self.get_logger().info(
                    'Downsampled map preview %dx%d -> %dx%d (factor %d)', *shape
                )
                self.last_map_shape = shape
            width, height = out_width, out_height
        encoded_cells = (cells + 1).astype(np.uint8, copy=False).tobytes()
        q = msg.info.origin.orientation
        self.latest_map = {
            'frame': msg.header.frame_id, 'width': width, 'height': height,
            'resolution': msg.info.resolution * factor,
            'origin': [msg.info.origin.position.x, msg.info.origin.position.y,
                       math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))],
            'cells': base64.b64encode(zlib.compress(encoded_cells)).decode(),
        }
        self.send_map()

    def send_map(self):
        if self.latest_map:
            self.send(self.map_pub, self.latest_map)

    def amcl_pose(self, msg):
        covariance = msg.pose.covariance
        position_sigma = math.sqrt(max(0.0, covariance[0], covariance[7]))
        yaw_sigma = math.sqrt(max(0.0, covariance[35]))
        if position_sigma < 0.5 and yaw_sigma < 0.35:
            quality = 'good'
        elif position_sigma < 2.0 and yaw_sigma < 1.0:
            quality = 'converging'
        else:
            quality = 'uncertain'
        self.localization_uncertainty = (position_sigma, yaw_sigma, quality)

    def mapping_status(self, message=None, request_id=None):
        available = (Path.home() / '.config/systemd/user/robot320-spatial-mapping.service').is_file()
        state = subprocess.run(['systemctl', '--user', 'is-active', 'robot320-spatial-mapping.service'],
                               capture_output=True, text=True, timeout=3).stdout.strip()
        localization_available = (
            Path.home() / '.config/systemd/user/robot320-spatial-localization.service'
        ).is_file()
        localization_state = subprocess.run(
            ['systemctl', '--user', 'is-active', 'robot320-spatial-localization.service'],
            capture_output=True, text=True, timeout=3,
        ).stdout.strip()
        self.mapping_status_sequence += 1
        if message is None:
            if localization_state == 'active':
                labels = {'good': '良好', 'converging': '正在收敛', 'uncertain': '不确定'}
                quality = self.localization_uncertainty[2] if self.localization_uncertainty else 'uncertain'
                message = f'车上定位服务：active · 定位置信度：{labels[quality]}'
            else:
                message = f'车上建图服务：{state or "未安装"}（独立雷达建图，不控制车辆）'
        payload = {
            'available': available,
            'state': state,
            'localization_available': localization_available,
            'localization_state': localization_state,
            'message': message,
            'stamp': time.time(),
            'sequence': self.mapping_status_sequence,
        }
        if self.localization_uncertainty:
            payload['position_sigma'], payload['yaw_sigma'], payload['localization_quality'] = (
                self.localization_uncertainty
            )
        if request_id:
            payload['request_id'] = request_id
        self.send(self.mapping_status_pub, payload)

    def mapping_control(self, msg):
        try:
            data = json.loads(msg.data)
            action = data.get('action')
            if action not in ('start', 'stop', 'localize', 'relocalize', 'stop_localization',
                              'initial_pose') \
                    or abs(time.time()-float(data['stamp'])) > 5:
                raise ValueError('invalid or expired mapping request')
            request_id = data.get('request_id')
            if not isinstance(request_id, str) or not request_id or len(request_id) > 80:
                raise ValueError('missing or invalid request id')
            self.get_logger().info(
                f'mapping control request action={action} request_id={request_id}'
            )
            if action == 'relocalize':
                if not self.global_localization_client.service_is_ready():
                    self.mapping_status('AMCL 全局重定位服务尚未就绪', request_id)
                    return
                future = self.global_localization_client.call_async(Empty.Request())
                future.add_done_callback(lambda _future: self._global_localization_started(request_id))
                self.mapping_status('已请求 AMCL 全地图匹配，正在等待扫描收敛', request_id)
                return
            if action == 'initial_pose':
                pose = data.get('pose')
                if (not isinstance(pose, list) or len(pose) != 3
                        or not all(isinstance(value, (int, float)) and math.isfinite(value)
                                   for value in pose)
                        or abs(pose[0]) > 10000 or abs(pose[1]) > 10000 or abs(pose[2]) > math.pi):
                    raise ValueError('invalid initial pose')
                message = PoseWithCovarianceStamped()
                message.header.stamp = self.get_clock().now().to_msg()
                message.header.frame_id = 'map'
                message.pose.pose.position.x, message.pose.pose.position.y = pose[0], pose[1]
                message.pose.pose.orientation.z = math.sin(pose[2] / 2)
                message.pose.pose.orientation.w = math.cos(pose[2] / 2)
                message.pose.covariance[0] = message.pose.covariance[7] = 0.25
                message.pose.covariance[35] = 0.12
                self.initial_pose_pub.publish(message)
                self.nomotion_updates_remaining = 5
                self.mapping_status('已发送手动初始位置，正在用雷达扫描快速校正', request_id)
                return
            # Fixed service allowlist: no shell, arbitrary command, or vehicle control.
            service_actions = {
                'start': ('start', 'robot320-spatial-mapping.service'),
                'stop': ('stop', 'robot320-spatial-mapping.service'),
                # Always restart so a newly uploaded active.yaml is re-read.
                'localize': ('restart', 'robot320-spatial-localization.service'),
                'stop_localization': ('stop', 'robot320-spatial-localization.service'),
            }
            verb, service = service_actions[action]
            result = subprocess.run(
                ['systemctl', '--user', verb, service], capture_output=True, text=True, timeout=15
            )
            self.get_logger().info(
                f'mapping control result action={action} request_id={request_id} '
                f'returncode={result.returncode}'
            )
            labels = {
                'start': '建图启动请求已处理', 'stop': '建图停止请求已处理',
                'localize': '静态地图定位启动请求已处理',
                'stop_localization': '定位停止请求已处理',
            }
            self.mapping_status(
                labels[action] if result.returncode == 0 else '车端服务请求失败，请检查 journal',
                request_id,
            )
        except (ValueError, TypeError, KeyError, AttributeError, subprocess.TimeoutExpired):
            self.mapping_status('建图请求无效、已过期或执行超时')

    def _global_localization_started(self, request_id):
        self.nomotion_updates_remaining = 5
        self.get_logger().info(f'global localization accepted request_id={request_id}')

    def nomotion_update(self):
        if self.nomotion_updates_remaining <= 0 or not self.nomotion_update_client.service_is_ready():
            return
        self.nomotion_update_client.call_async(Empty.Request())
        self.nomotion_updates_remaining -= 1


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
