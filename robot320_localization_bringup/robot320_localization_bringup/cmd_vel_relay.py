"""Relay Nav2's smoothed command around the Isaac-only safety monitor."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelRelay(Node):
    """Forward velocity commands between configurable Twist topics."""

    def __init__(self) -> None:
        super().__init__("cmd_vel_relay")
        input_topic = str(
            self.declare_parameter("input_topic", "/cmd_vel_smoothed").value
        )
        output_topic = str(
            self.declare_parameter("output_topic", "/cmd_vel").value
        )
        self._publisher = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, input_topic, self._publisher.publish, 10)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
