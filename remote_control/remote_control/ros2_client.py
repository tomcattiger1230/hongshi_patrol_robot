#!/usr/bin/env python3
"""ROS 2 remote control client.

Use this on an Ubuntu upper computer that has ROS 2 sourced. It publishes the
same standard topics consumed by ``mobile_platform.ros2_node``.
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Optional

from mobile_platform.messages import RobotTelemetry, telemetry_from_json

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from std_msgs.msg import Bool, String
except ImportError as exc:  # pragma: no cover - exercised on ROS machines.
    rclpy = None
    Node = object
    Twist = Bool = String = None
    _ROS_IMPORT_ERROR = exc
else:
    _ROS_IMPORT_ERROR = None


class RobotRemoteRosNode(Node):
    def __init__(self, topic_prefix: str = "/robot320"):
        super().__init__("robot320_remote_control")
        self.topic_prefix = topic_prefix.rstrip("/")
        self.latest_telemetry: Optional[RobotTelemetry] = None

        self.cmd_vel_pub = self.create_publisher(Twist, f"{self.topic_prefix}/cmd_vel", 10)
        self.brake_pub = self.create_publisher(Bool, f"{self.topic_prefix}/brake", 10)
        self.estop_pub = self.create_publisher(Bool, f"{self.topic_prefix}/emergency_stop", 10)
        self.mode_pub = self.create_publisher(String, f"{self.topic_prefix}/mode", 10)
        self.create_subscription(String, f"{self.topic_prefix}/telemetry", self.on_telemetry, 10)

    def send_manual_command(self, linear_speed_mps: float = 0.0, angular_speed_radps: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x = float(linear_speed_mps)
        msg.angular.z = float(angular_speed_radps)
        self.cmd_vel_pub.publish(msg)

    def stop(self) -> None:
        self.send_manual_command(0.0, 0.0)

    def brake(self, enabled: bool = True) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self.brake_pub.publish(msg)

    def emergency_stop(self, enabled: bool = True) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self.estop_pub.publish(msg)

    def reset_idle(self) -> None:
        msg = String()
        msg.data = "idle"
        self.mode_pub.publish(msg)

    def on_telemetry(self, msg: String) -> None:
        self.latest_telemetry = telemetry_from_json(msg.data)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Robot320 ROS 2 remote control")
    parser.add_argument("--topic-prefix", default="/robot320")
    sub = parser.add_subparsers(dest="command", required=True)

    move = sub.add_parser("move")
    move.add_argument("--linear", type=float, default=0.0)
    move.add_argument("--angular", type=float, default=0.0)
    move.add_argument("--duration", type=float, default=0.0)

    sub.add_parser("stop")
    sub.add_parser("brake")
    sub.add_parser("estop")
    sub.add_parser("reset")

    watch = sub.add_parser("watch")
    watch.add_argument("--seconds", type=float, default=30.0)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if rclpy is None:
        raise RuntimeError("ROS 2 Python packages are not available. Install/source ROS 2 first.") from _ROS_IMPORT_ERROR

    rclpy.init(args=None)
    node = RobotRemoteRosNode(args.topic_prefix)

    try:
        if args.command == "move":
            node.send_manual_command(args.linear, args.angular)
            rclpy.spin_once(node, timeout_sec=0.1)
            if args.duration > 0:
                deadline = time.monotonic() + args.duration
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.1)
                node.stop()
        elif args.command == "stop":
            node.stop()
        elif args.command == "brake":
            node.brake(True)
        elif args.command == "estop":
            node.emergency_stop(True)
        elif args.command == "reset":
            node.reset_idle()
        elif args.command == "watch":
            deadline = time.monotonic() + args.seconds
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.5)
                if node.latest_telemetry:
                    chassis = node.latest_telemetry.chassis
                    pose = node.latest_telemetry.pose
                    pose_text = (
                        f" pose=({pose.x_m:.2f},{pose.y_m:.2f},{pose.yaw_rad:.2f})"
                        if pose else " pose=unavailable"
                    )
                    print(
                        f"connected={chassis.connected} enabled={chassis.enabled} "
                        f"speed={chassis.speed_kmh} rpm={chassis.commanded_rpm} "
                        f"steering={chassis.steering_direction}:{chassis.steering_angle_deg}"
                        f"{pose_text}"
                    )
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
