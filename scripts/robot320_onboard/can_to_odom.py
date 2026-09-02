#!/usr/bin/env python3
"""
CAN 命令→轮式里程计节点 (使用实际 RPM 反馈)
订阅 /can/actual_speed 和 /can/actual_steering，发布原始轮式里程计。
不发布 TF；odom→base_link 仅由 EKF 发布。
"""

import math

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32


def is_valid_speed(speed, max_valid_speed):
    """Return True only for finite feedback inside the configured range."""
    return math.isfinite(speed) and abs(speed) <= max_valid_speed


def effective_speed(speed, last_speed_time_ns, current_time_ns, speed_timeout):
    """Return zero when no valid speed feedback arrived within the timeout."""
    if last_speed_time_ns is None:
        return 0.0
    age_seconds = (current_time_ns - last_speed_time_ns) / 1e9
    return speed if age_seconds <= speed_timeout else 0.0


def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class CanToOdom(Node):
    def __init__(self):
        super().__init__('can_to_odom')

        self.declare_parameter('wheelbase', 0.89)
        self.declare_parameter('max_steering_angle', 0.524)
        self.declare_parameter('wheel_diameter', 0.43)
        self.declare_parameter('max_valid_speed', 2.0)
        self.declare_parameter('speed_timeout', 0.5)

        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.max_steering = float(self.get_parameter('max_steering_angle').value)
        self.wheel_diameter = float(self.get_parameter('wheel_diameter').value)
        self.max_valid_speed = float(self.get_parameter('max_valid_speed').value)
        self.speed_timeout = float(self.get_parameter('speed_timeout').value)
        if self.max_valid_speed <= 0.0:
            raise ValueError('max_valid_speed must be greater than zero')
        if self.speed_timeout <= 0.0:
            raise ValueError('speed_timeout must be greater than zero')

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = None

        self.speed = 0.0
        self.last_speed_time_ns = None
        self.speed_timed_out = False
        self.last_invalid_warning_time_ns = None
        self.steering_angle = 0.0

        self.sub_actual_speed = self.create_subscription(
            Float32, '/can/actual_speed', self.actual_speed_callback, 10
        )
        self.sub_actual_steering = self.create_subscription(
            Float32, '/can/actual_steering', self.actual_steering_callback, 10
        )

        self.odom_pub = self.create_publisher(Odometry, '/wheel/odom_raw', 10)
        self.timer = self.create_timer(0.02, self.publish_odom)

        self.pose_covariance = [
            0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.001,
        ]
        self.twist_covariance = self.pose_covariance.copy()

        self.get_logger().info('=' * 50)
        self.get_logger().info('🚗 CAN to Odom (使用实际 RPM 反馈)')
        self.get_logger().info(f'轴距: {self.wheelbase:.2f}m')
        self.get_logger().info(f'轮径: {self.wheel_diameter:.3f}m')
        self.get_logger().info(f'有效速度范围: ±{self.max_valid_speed:.3f} m/s')
        self.get_logger().info(f'速度反馈超时: {self.speed_timeout:.3f} s')
        self.get_logger().info('订阅: /can/actual_speed, /can/actual_steering')
        self.get_logger().info('=' * 50)
        self.get_logger().info('⏳ 等待实际速度反馈...')

        self.publish_count = 0

    def actual_speed_callback(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        speed = float(msg.data)
        if not is_valid_speed(speed, self.max_valid_speed):
            if (
                self.last_invalid_warning_time_ns is None
                or now_ns - self.last_invalid_warning_time_ns >= 2_000_000_000
            ):
                self.get_logger().warning(
                    f'忽略无效速度反馈: {speed!r} m/s；有效范围为 '
                    f'±{self.max_valid_speed:.3f} m/s'
                )
                self.last_invalid_warning_time_ns = now_ns
            return

        self.speed = speed
        self.last_speed_time_ns = now_ns
        self.speed_timed_out = False
        self.get_logger().info(f'📥 收到速度反馈: {self.speed:.3f} m/s')

    def actual_steering_callback(self, msg):
        self.steering_angle = math.radians(msg.data)

    def publish_odom(self):
        now = self.get_clock().now()
        current_time = now.to_msg()

        if self.last_time is None:
            self.last_time = current_time
            return

        dt = (
            (current_time.sec - self.last_time.sec)
            + (current_time.nanosec - self.last_time.nanosec) / 1e9
        )
        self.last_time = current_time

        if dt > 0.1 or dt <= 0:
            return

        v = effective_speed(
            self.speed,
            self.last_speed_time_ns,
            now.nanoseconds,
            self.speed_timeout,
        )
        if v == 0.0 and self.speed != 0.0 and not self.speed_timed_out:
            self.get_logger().warning(
                f'速度反馈超过 {self.speed_timeout:.3f} s，wheel odom 速度归零'
            )
            self.speed_timed_out = True

        delta = self.steering_angle
        if abs(delta) > self.max_steering:
            delta = self.max_steering * (1 if delta > 0 else -1)

        if abs(delta) < 0.001:
            dx = v * dt * math.cos(self.yaw)
            dy = v * dt * math.sin(self.yaw)
            dyaw = 0.0
        else:
            radius = self.wheelbase / math.tan(delta)
            dyaw = (v * dt) / radius
            dx = radius * (math.sin(self.yaw + dyaw) - math.sin(self.yaw))
            dy = radius * (-math.cos(self.yaw + dyaw) + math.cos(self.yaw))

        self.x += dx
        self.y += dy
        self.yaw += dyaw
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        q = quaternion_from_euler(0, 0, self.yaw)
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = q.x
        odom_msg.pose.pose.orientation.y = q.y
        odom_msg.pose.pose.orientation.z = q.z
        odom_msg.pose.pose.orientation.w = q.w
        odom_msg.pose.covariance = self.pose_covariance
        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = dyaw / dt if dt > 0 else 0.0
        odom_msg.twist.covariance = self.twist_covariance
        try:
            self.odom_pub.publish(odom_msg)
        except Exception:
            # ROS 2 may invalidate the context while a timer callback is
            # publishing during SIGINT/SIGTERM shutdown. Ignore only that
            # shutdown race; preserve all exceptions while the context is live.
            if rclpy.ok():
                raise
            return

        self.publish_count += 1
        if self.publish_count % 10 == 0:
            self.get_logger().info(
                f'📍 x={self.x:.3f}, y={self.y:.3f}, '
                f'yaw={math.degrees(self.yaw):.1f}°, v={v:.3f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = CanToOdom()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
