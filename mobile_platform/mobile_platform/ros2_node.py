#!/usr/bin/env python3
"""ROS 2 wrapper for Robot320 CAN chassis control.

This node runs on the robot computer, usually Ubuntu with ROS 2 installed. It
subscribes to standard ROS 2 command topics, converts them to Robot320 CAN
commands through :class:`OnboardNode`, and publishes chassis telemetry as ROS
messages.
"""

from __future__ import annotations

import argparse
import logging
import math
import sys
import time

from .controlcan import CANAdapterConfig
from .messages import ChassisCommand, Pose2D, RobotTelemetry, to_json
from .onboard_node import OnboardNode
from .robot320 import Robot320Platform
from .safety import SafetyConfig, SafetyController

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist
    from rclpy.node import Node
    from std_msgs.msg import Bool, Float32, String
except ImportError as exc:  # pragma: no cover - exercised on ROS machines.
    rclpy = None
    Node = object
    PoseStamped = Twist = Bool = Float32 = String = None
    _ROS_IMPORT_ERROR = exc
else:
    _ROS_IMPORT_ERROR = None


LOGGER = logging.getLogger(__name__)


class Robot320RosNode(Node):
    def __init__(
        self,
        robot: Robot320Platform,
        safety: SafetyController,
        rpm_per_mps: float = 500.0,
        steering_gain_deg_per_radps: float = 180.0,
        telemetry_period_s: float = 0.2,
        topic_prefix: str = "/robot320",
        localization_pose_topic: str = "/tracked_pose",
        localization_stale_timeout_s: float = 1.0,
    ):
        super().__init__("robot320_can_bridge")
        self.controller = OnboardNode(
            robot=robot,
            command_subscriber=NoneCommandSubscriber(),
            telemetry_publisher=NoneTelemetryPublisher(),
            safety=safety,
            rpm_per_mps=rpm_per_mps,
            steering_gain_deg_per_radps=steering_gain_deg_per_radps,
        )
        self.topic_prefix = topic_prefix.rstrip("/")
        self._last_stop_due_timeout = False
        self._latest_pose: Pose2D | None = None
        self._last_pose_received = 0.0
        self.localization_stale_timeout_s = localization_stale_timeout_s

        self.telemetry_pub = self.create_publisher(String, f"{self.topic_prefix}/telemetry", 10)
        self.chassis_status_pub = self.create_publisher(String, f"{self.topic_prefix}/chassis_status", 10)
        self.speed_pub = self.create_publisher(Float32, f"{self.topic_prefix}/speed_kmh", 10)

        self.create_subscription(Twist, f"{self.topic_prefix}/cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(Bool, f"{self.topic_prefix}/brake", self.on_brake, 10)
        self.create_subscription(Bool, f"{self.topic_prefix}/emergency_stop", self.on_emergency_stop, 10)
        self.create_subscription(String, f"{self.topic_prefix}/mode", self.on_mode, 10)
        self.create_subscription(PoseStamped, localization_pose_topic, self.on_localization_pose, 10)

        self.controller.robot.connect(start_receiver=True)
        self.timer = self.create_timer(telemetry_period_s, self.on_timer)
        self.get_logger().info(f"Robot320 ROS 2 bridge started under {self.topic_prefix}")

    def on_cmd_vel(self, msg: Twist) -> None:
        command = ChassisCommand(
            linear_speed_mps=float(msg.linear.x),
            angular_speed_radps=float(msg.angular.z),
            mode="manual",
        )
        self._apply(command)

    def on_brake(self, msg: Bool) -> None:
        self._apply(ChassisCommand(brake=bool(msg.data), mode="manual"))

    def on_emergency_stop(self, msg: Bool) -> None:
        self._apply(ChassisCommand(emergency_stop=bool(msg.data), mode="manual"))

    def on_mode(self, msg: String) -> None:
        mode = msg.data.strip() or "manual"
        if mode not in {"idle", "manual", "navigation"}:
            self.get_logger().warning(f"Unsupported mode ignored: {mode}")
            return
        self._apply(ChassisCommand(mode=mode))

    def on_localization_pose(self, msg: PoseStamped) -> None:
        """Cache the latest Cartographer pose for remote telemetry."""
        q = msg.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1_000_000_000.0
        self._latest_pose = Pose2D(
            x_m=float(msg.pose.position.x),
            y_m=float(msg.pose.position.y),
            yaw_rad=yaw,
            frame_id=msg.header.frame_id or "map",
            stamp=stamp or time.time(),
        )
        self._last_pose_received = time.monotonic()

    def on_timer(self) -> None:
        if self.controller.safety.timed_out() and not self._last_stop_due_timeout:
            self.get_logger().warning("Command timeout, stopping chassis")
            self.controller.robot.stop_motor()
            self._last_stop_due_timeout = True

        telemetry = self.controller.build_telemetry()
        if (
            self._latest_pose is not None
            and time.monotonic() - self._last_pose_received <= self.localization_stale_timeout_s
        ):
            telemetry.pose = self._latest_pose
        self._publish_telemetry(telemetry)

    def destroy_node(self) -> bool:
        self.controller.robot.disconnect()
        return super().destroy_node()

    def _apply(self, command: ChassisCommand) -> None:
        self._last_stop_due_timeout = False
        self.controller.apply_command(self.controller.safety.accept(command))

    def _publish_telemetry(self, telemetry: RobotTelemetry) -> None:
        telemetry_msg = String()
        telemetry_msg.data = to_json(telemetry)
        self.telemetry_pub.publish(telemetry_msg)

        status_msg = String()
        status_msg.data = to_json(telemetry.chassis)
        self.chassis_status_pub.publish(status_msg)

        if telemetry.chassis.speed_kmh is not None:
            speed_msg = Float32()
            speed_msg.data = float(telemetry.chassis.speed_kmh)
            self.speed_pub.publish(speed_msg)


class NoneCommandSubscriber:
    def receive_command(self, timeout_s: float = 0.1):
        return None


class NoneTelemetryPublisher:
    def publish_telemetry(self, telemetry: RobotTelemetry) -> None:
        return None


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Robot320 ROS 2 CAN bridge")
    parser.add_argument("--lib", default=None)
    parser.add_argument("--device-index", type=int, default=0)
    parser.add_argument("--can-index", type=int, default=0)
    parser.add_argument("--topic-prefix", default="/robot320")
    parser.add_argument("--localization-pose-topic", default="/tracked_pose")
    parser.add_argument("--localization-stale-timeout", type=float, default=1.0)
    parser.add_argument("--telemetry-period", type=float, default=0.2)
    parser.add_argument("--command-timeout", type=float, default=0.6)
    parser.add_argument("--max-linear-speed", type=float, default=0.8)
    parser.add_argument("--max-angular-speed", type=float, default=1.2)
    parser.add_argument("--rpm-per-mps", type=float, default=500.0)
    parser.add_argument("--steering-gain", type=float, default=180.0)
    parser.add_argument("--verbose", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if rclpy is None:
        raise RuntimeError("ROS 2 Python packages are not available. Install/source ROS 2 first.") from _ROS_IMPORT_ERROR

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO, format="%(message)s")

    rclpy.init(args=None)
    node = Robot320RosNode(
        robot=Robot320Platform(
            config=CANAdapterConfig(
                device_index=args.device_index,
                can_index=args.can_index,
                library_path=args.lib,
            )
        ),
        safety=SafetyController(
            SafetyConfig(
                command_timeout_s=args.command_timeout,
                max_linear_speed_mps=args.max_linear_speed,
                max_angular_speed_radps=args.max_angular_speed,
            )
        ),
        rpm_per_mps=args.rpm_per_mps,
        steering_gain_deg_per_radps=args.steering_gain,
        telemetry_period_s=args.telemetry_period,
        topic_prefix=args.topic_prefix,
        localization_pose_topic=args.localization_pose_topic,
        localization_stale_timeout_s=args.localization_stale_timeout,
    )

    try:
        rclpy.spin(node)
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
