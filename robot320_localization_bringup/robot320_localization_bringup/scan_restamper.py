"""Restamp simulated laser scans after expensive point-cloud projection."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanRestamper(Node):
    """Publish projected scans with the current simulation timestamp."""

    def __init__(self) -> None:
        super().__init__("scan_restamper")
        self.declare_parameter("input_topic", "/scan_raw")
        self.declare_parameter("output_topic", "/scan")
        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self._publisher = self.create_publisher(LaserScan, output_topic, 10)
        self.create_subscription(
            LaserScan,
            input_topic,
            self._on_scan,
            qos_profile_sensor_data,
        )

    def _on_scan(self, scan: LaserScan) -> None:
        scan.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(scan)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanRestamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

