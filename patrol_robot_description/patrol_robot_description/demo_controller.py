"""Publish a repeating forward-and-turn command sequence for the demo robot."""

from __future__ import annotations

from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


@dataclass(frozen=True)
class Motion:
    duration: float
    linear_x: float
    angular_z: float


class PatrolDemoController(Node):
    """Drive a rounded square while exposing all motion as regular cmd_vel."""

    def __init__(self) -> None:
        super().__init__("patrol_demo_controller")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("linear_speed", 0.35)
        self.declare_parameter("angular_speed", 0.65)
        self.declare_parameter("forward_duration", 4.0)
        self.declare_parameter("turn_duration", 2.4)

        topic = str(self.get_parameter("cmd_vel_topic").value)
        linear_speed = float(self.get_parameter("linear_speed").value)
        angular_speed = float(self.get_parameter("angular_speed").value)
        forward_duration = float(self.get_parameter("forward_duration").value)
        turn_duration = float(self.get_parameter("turn_duration").value)

        self._motions = (
            Motion(forward_duration, linear_speed, 0.0),
            Motion(turn_duration, 0.0, angular_speed),
        )
        self._motion_index = 0
        self._motion_started_ns = self.get_clock().now().nanoseconds
        self._publisher = self.create_publisher(Twist, topic, 10)
        self._timer = self.create_timer(0.05, self._tick)
        self.get_logger().info(f"Publishing patrol demo commands on {topic}")

    def _tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        motion = self._motions[self._motion_index]
        elapsed = (now_ns - self._motion_started_ns) / 1e9
        if elapsed >= motion.duration:
            self._motion_index = (self._motion_index + 1) % len(self._motions)
            self._motion_started_ns = now_ns
            motion = self._motions[self._motion_index]

        command = Twist()
        command.linear.x = motion.linear_x
        command.angular.z = motion.angular_z
        self._publisher.publish(command)

    def stop(self) -> None:
        self._publisher.publish(Twist())


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PatrolDemoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok(context=node.context):
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
