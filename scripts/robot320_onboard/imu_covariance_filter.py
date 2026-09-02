#!/usr/bin/env python3
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu

G = 9.80665
BIAS = (-0.003418060105, -0.009969370443, -0.006636971922)
GV = (2.898143e-6, 3.499062e-6, 5.571002e-7)
AV = (1.9113477551844227e-6, 2.3803518037151163e-7, 2.5256199093441396e-6)


class ImuCovarianceFilter(Node):
    def __init__(self):
        super().__init__('imu_covariance_filter')
        self.declare_parameter('input_topic', '/livox/imu')
        self.declare_parameter('output_topic', '/livox/imu_corrected')
        self.declare_parameter('accel_input_is_g', True)
        self.declare_parameter('remove_gyro_bias', True)
        self.g_as_g = self.get_parameter('accel_input_is_g').value
        self.remove = self.get_parameter('remove_gyro_bias').value
        output_topic = self.get_parameter('output_topic').value
        input_topic = self.get_parameter('input_topic').value
        self.pub = self.create_publisher(Imu, output_topic, 20)
        self.sub = self.create_subscription(Imu, input_topic, self.cb, 100)
        self.get_logger().info(
            f'{input_topic} -> {output_topic}, '
            f'accel_input_is_g={self.g_as_g}, remove_gyro_bias={self.remove}'
        )

    def cb(self, message):
        output = Imu()
        output.header = message.header
        output.orientation = message.orientation
        output.orientation_covariance = [-1.0, 0, 0, 0, 0, 0, 0, 0, 0]
        gyro = [message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z]
        if self.remove:
            gyro = [gyro[i] - BIAS[i] for i in range(3)]
        output.angular_velocity.x, output.angular_velocity.y, output.angular_velocity.z = gyro
        output.angular_velocity_covariance = [GV[0], 0, 0, 0, GV[1], 0, 0, 0, GV[2]]
        accel = [message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z]
        scale = G if self.g_as_g else 1.0
        accel = [value * scale for value in accel]
        output.linear_acceleration.x, output.linear_acceleration.y, output.linear_acceleration.z = accel
        scale2 = scale * scale
        output.linear_acceleration_covariance = [AV[0] * scale2, 0, 0, 0, AV[1] * scale2, 0, 0, 0, AV[2] * scale2]
        try:
            self.pub.publish(output)
        except Exception:
            if rclpy.ok():
                raise


def main():
    rclpy.init()
    node = ImuCovarianceFilter()
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
