"""Relay Nav2's smoothed command around the Isaac-only safety monitor."""

import time

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
        priority_input_topic = str(
            self.declare_parameter("priority_input_topic", "").value
        )
        self._priority_timeout_ns = int(
            float(self.declare_parameter("priority_timeout", 0.5).value) * 1e9
        )
        self._last_priority_ns = 0
        self._publisher = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, input_topic, self._on_input, 10)
        if priority_input_topic:
            self.create_subscription(
                Twist,
                priority_input_topic,
                self._on_priority_input,
                10,
            )

    def _on_input(self, message: Twist) -> None:
        """Forward Nav2 only while no recent manual command has priority."""
        if (
            self._last_priority_ns == 0
            or time.monotonic_ns() - self._last_priority_ns
            >= self._priority_timeout_ns
        ):
            self._publisher.publish(message)

    def _on_priority_input(self, message: Twist) -> None:
        """Forward a manual command and suppress Nav2 for the timeout window."""
        self._last_priority_ns = time.monotonic_ns()
        self._publisher.publish(message)


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
