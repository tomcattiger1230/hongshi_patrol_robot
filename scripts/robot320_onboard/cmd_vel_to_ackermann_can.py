#!/usr/bin/env python3
"""
阿克曼底盘速度→CAN 转换节点
将 /cmd_vel (Twist) 转换为 Robot320 CAN 指令
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math

class AckermannToCAN(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_can')

        # 参数
        self.declare_parameter('wheelbase', 0.89)
        self.declare_parameter('min_turning_radius', 2.35)
        self.declare_parameter('max_wheel_angle_deg', 20.75)
        self.declare_parameter('max_steering_command_deg', 350.0)
        self.declare_parameter('max_speed', 0.3)
        self.declare_parameter('min_speed', 0.02)
        self.declare_parameter('rpm_per_mps', 500.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_safe')

        self.wheelbase = self.get_parameter('wheelbase').value
        self.min_turning_radius = float(self.get_parameter('min_turning_radius').value)
        self.max_wheel_angle_deg = float(self.get_parameter('max_wheel_angle_deg').value)
        self.max_steering_command_deg = float(
            self.get_parameter('max_steering_command_deg').value
        )
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.rpm_per_mps = float(self.get_parameter('rpm_per_mps').value)
        if self.rpm_per_mps <= 0.0:
            raise ValueError('rpm_per_mps must be greater than zero')
        if self.min_turning_radius <= 0.0:
            raise ValueError('min_turning_radius must be greater than zero')
        if not 0.0 < self.max_wheel_angle_deg < 90.0:
            raise ValueError('max_wheel_angle_deg must be within (0, 90)')
        if self.max_steering_command_deg <= 0.0:
            raise ValueError('max_steering_command_deg must be greater than zero')
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        # Only consume the collision monitor output. Subscribing directly to
        # Nav2's raw /cmd_vel would bypass smoothing and obstacle stopping.
        self.sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10
        )

        # 发布 CAN 指令
        self.speed_pub = self.create_publisher(Float32, '/can/speed_cmd', 10)
        self.steering_pub = self.create_publisher(Float32, '/can/steering_cmd', 10)

        # 看门狗
        self.last_cmd_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.watchdog_check)

        self.get_logger().info('='*50)
        self.get_logger().info('🚗 Robot320 阿克曼 CAN 控制节点')
        self.get_logger().info(f'轴距: {self.wheelbase:.2f}m')
        self.get_logger().info(
            f'转向标定: 车轮 {self.max_wheel_angle_deg:.2f}° '
            f'→ 执行器 {self.max_steering_command_deg:.0f}°'
        )
        self.get_logger().info(f'最小转弯半径: {self.min_turning_radius:.2f}m')
        self.get_logger().info(f'最大速度: {self.max_speed:.2f}m/s')
        self.get_logger().info(f'速度比例: {self.rpm_per_mps:.1f} RPM/(m/s)')
        self.get_logger().info(f'安全速度输入: {self.cmd_vel_topic}')
        self.get_logger().info('='*50)

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()

        v = msg.linear.x
        omega = msg.angular.z

        # 限幅
        v = max(-self.max_speed, min(self.max_speed, v))

        # Convert requested curvature to the equivalent road-wheel angle,
        # constrain it by both geometry and the measured turning radius, then
        # scale it to the Robot320 steering actuator's +/-350 degree command.
        if abs(v) > self.min_speed:
            requested_curvature = omega / v
            radius_curvature_limit = 1.0 / self.min_turning_radius
            angle_curvature_limit = (
                math.tan(math.radians(self.max_wheel_angle_deg)) / self.wheelbase
            )
            curvature_limit = min(radius_curvature_limit, angle_curvature_limit)
            curvature = max(
                -curvature_limit, min(curvature_limit, requested_curvature)
            )
            wheel_angle = math.atan(self.wheelbase * curvature)
            steering_command = math.copysign(
                math.degrees(abs(wheel_angle))
                / self.max_wheel_angle_deg
                * self.max_steering_command_deg,
                wheel_angle,
            ) if wheel_angle != 0.0 else 0.0
        else:
            wheel_angle = 0.0
            steering_command = 0.0

        # Robot320 drive command is motor RPM, not wheel RPM. Keep the
        # calibrated platform conversion explicit and configurable.
        rpm = v * self.rpm_per_mps
        rpm = max(-500, min(500, rpm))

        # 发布
        self.speed_pub.publish(Float32(data=rpm))
        self.steering_pub.publish(Float32(data=steering_command))

        # 调试输出
        if int(self.get_clock().now().nanoseconds / 1e9) % 5 == 0:
            self.get_logger().info(
                f'v={v:.2f}m/s, ω={omega:.2f}rad/s, '
                f'车轮角={math.degrees(wheel_angle):.1f}°, '
                f'执行器={steering_command:.1f}°, RPM={rpm:.0f}'
            )

    def watchdog_check(self):
        """超时保护"""
        if (self.get_clock().now() - self.last_cmd_time).nanoseconds > 0.5e9:
            self.speed_pub.publish(Float32(data=0.0))
            self.steering_pub.publish(Float32(data=0.0))

def main(args=None):
    rclpy.init(args=args)
    node = AckermannToCAN()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
