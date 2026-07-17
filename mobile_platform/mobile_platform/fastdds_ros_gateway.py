#!/usr/bin/env python3
"""NUC gateway using ROS 2 String topics for external Robot320 communication."""

from __future__ import annotations

import argparse
import json
import logging
import math
import os
import queue
import sys
import time
from dataclasses import fields

from robot320_interfaces.messages import (
    CommandReply,
    Heartbeat,
    LiftStatus,
    NavigationStatus,
    RobotCommand,
    RobotTelemetry,
    heartbeat_from_json,
    robot_command_from_json,
    telemetry_from_json,
    to_json,
)

try:
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import Twist
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from std_msgs.msg import Bool, String
except ImportError as exc:  # pragma: no cover - evaluated on the NUC.
    rclpy = None
    Node = object
    GoalStatus = Twist = NavigateToPose = ActionClient = Bool = String = None
    _ROS_IMPORT_ERROR = exc
else:
    _ROS_IMPORT_ERROR = None


LOGGER = logging.getLogger(__name__)


class Ros2RobotTransport:
    """String/JSON transport whose ROS topics are exposed through the active RMW."""

    backend = "ros2"

    def __init__(self, node: Node, topic_prefix: str, robot_id: str):
        self.robot_id = robot_id
        prefix = topic_prefix.rstrip("/")
        self._commands: queue.Queue[RobotCommand] = queue.Queue()
        self._heartbeats: queue.Queue[Heartbeat] = queue.Queue()
        self._state_pub = node.create_publisher(String, f"{prefix}/state", 10)
        self._reply_pub = node.create_publisher(String, f"{prefix}/reply", 10)
        self._heartbeat_pub = node.create_publisher(String, f"{prefix}/heartbeat", 10)
        node.create_subscription(String, f"{prefix}/command", self._on_command, 10)
        node.create_subscription(
            String, f"{prefix}/heartbeat", self._on_heartbeat, 10
        )

    def receive_command(self, timeout_s: float = 0.1):
        return _queue_get(self._commands, timeout_s)

    def receive_heartbeat(self, timeout_s: float = 0.1):
        return _queue_get(self._heartbeats, timeout_s)

    def publish_state(self, telemetry: RobotTelemetry, sequence: int) -> None:
        del sequence
        telemetry.robot_id = self.robot_id
        self._state_pub.publish(_string_message(to_json(telemetry)))

    def publish_reply(self, reply: CommandReply) -> None:
        self._reply_pub.publish(_string_message(to_json(reply)))

    def publish_heartbeat(self, sequence: int) -> None:
        heartbeat = Heartbeat(
            node_id=self.robot_id,
            role="robot",
            sequence=sequence,
            timestamp_ms=int(time.time() * 1000.0),
        )
        self._heartbeat_pub.publish(_string_message(to_json(heartbeat)))

    def close(self) -> None:
        pass

    def _on_command(self, message: String) -> None:
        self._commands.put(robot_command_from_json(message.data))

    def _on_heartbeat(self, message: String) -> None:
        heartbeat = heartbeat_from_json(message.data)
        if heartbeat.role == "remote":
            self._heartbeats.put(heartbeat)


def _string_message(payload: str):
    message = String()
    message.data = payload
    return message


def _queue_get(items: queue.Queue, timeout_s: float):
    try:
        return items.get(timeout=max(0.0, timeout_s))
    except queue.Empty:
        return None


class Robot320FastDDSRosGateway(Node):
    def __init__(
        self,
        domain_id: int = 20,
        robot_id: str = "robot320",
        topic_prefix: str = "/robot320",
        nav_action: str = "/navigate_to_pose",
        nav_cmd_vel_topic: str = "/cmd_vel",
        telemetry_period_s: float = 0.2,
        heartbeat_period_s: float = 1.0,
        max_command_age_s: float = 2.0,
        transport=None,
    ):
        super().__init__("robot320_communication_gateway")
        self.robot_id = robot_id
        self.topic_prefix = topic_prefix.rstrip("/")
        self.transport = transport or Ros2RobotTransport(self, self.topic_prefix, robot_id)
        self.max_command_age_s = max_command_age_s
        self._last_sequences: dict[str, int] = {}
        self._state_sequence = 0
        self._heartbeat_sequence = 0
        self._latest_telemetry: RobotTelemetry | None = None
        self._last_telemetry_received = 0.0
        self._lift_status = LiftStatus()
        self._navigation = NavigationStatus()
        self._active_goal_handle = None
        self._active_nav_command: RobotCommand | None = None
        self._pending_nav_command_id: str | None = None
        self._nav_velocity_enabled = False
        self._initial_goal_distance: float | None = None

        self.cmd_vel_pub = self.create_publisher(Twist, f"{self.topic_prefix}/cmd_vel", 10)
        self.brake_pub = self.create_publisher(Bool, f"{self.topic_prefix}/brake", 10)
        self.estop_pub = self.create_publisher(
            Bool, f"{self.topic_prefix}/emergency_stop", 10
        )
        self.mode_pub = self.create_publisher(String, f"{self.topic_prefix}/mode", 10)
        self.lift_command_pub = self.create_publisher(
            String, f"{self.topic_prefix}/lift/command", 10
        )
        self.create_subscription(
            String, f"{self.topic_prefix}/telemetry", self._on_telemetry, 10
        )
        self.create_subscription(
            String, f"{self.topic_prefix}/lift/status", self._on_lift_status, 10
        )
        self.create_subscription(Twist, nav_cmd_vel_topic, self._on_nav_cmd_vel, 10)
        self.nav_client = ActionClient(self, NavigateToPose, nav_action)

        self.create_timer(0.05, self._poll_commands)
        self.create_timer(telemetry_period_s, self._publish_state)
        self.create_timer(heartbeat_period_s, self._publish_heartbeat)
        self.get_logger().info(
            f"ROS 2 communication gateway started: domain={domain_id}, robot={robot_id}"
        )

    def destroy_node(self) -> bool:
        self.transport.close()
        return super().destroy_node()

    def _poll_commands(self) -> None:
        for _ in range(20):
            command = self.transport.receive_command(timeout_s=0.0)
            if command is None:
                break
            reason = self._validate_command(command)
            if reason:
                self._reply(command, "rejected", reason)
                continue
            self._last_sequences[command.client_id] = command.sequence
            try:
                self._dispatch(command)
            except Exception as exc:
                self.get_logger().error(
                    f"failed to dispatch Fast DDS command {command.command_id}: {exc}"
                )
                self._reply(command, "failed", str(exc))

    def _dispatch(self, command: RobotCommand) -> None:
        if command.kind == "manual_motion":
            self._request_nav_cancel()
            self._publish_twist(command.linear_speed_mps, command.angular_speed_radps)
            self._publish_mode("manual")
            self._reply(command, "accepted", "manual motion forwarded")
        elif command.kind == "stop":
            self._request_nav_cancel()
            self._publish_twist(0.0, 0.0)
            self._reply(command, "accepted", "stop forwarded")
        elif command.kind == "brake":
            self._request_nav_cancel()
            self._publish_bool(self.brake_pub, True)
            self._reply(command, "accepted", "brake forwarded")
        elif command.kind == "emergency_stop":
            self._request_nav_cancel()
            self._publish_bool(self.estop_pub, True)
            self._reply(command, "accepted", "emergency stop forwarded")
        elif command.kind == "reset_emergency_stop":
            self._publish_mode("idle")
            self._reply(command, "accepted", "emergency stop reset forwarded")
        elif command.kind == "set_mode":
            if command.mode != "navigation":
                self._request_nav_cancel()
            self._publish_mode(command.mode or "idle")
            self._reply(command, "accepted", "mode forwarded")
        elif command.kind == "navigation_goal":
            self._send_navigation_goal(command)
        elif command.kind == "cancel_navigation":
            self._cancel_navigation(command)
        elif command.kind == "lift":
            self._send_lift_command(command)
        else:
            self._reply(command, "rejected", f"unsupported command: {command.kind}")

    def _send_navigation_goal(self, command: RobotCommand) -> None:
        if command.goal is None:
            self._reply(command, "rejected", "navigation goal is missing")
            return
        if not self.nav_client.server_is_ready():
            self._reply(command, "rejected", "Nav2 navigate_to_pose server is unavailable")
            return
        self._request_nav_cancel()
        self._active_goal_handle = None
        self._active_nav_command = None
        self._nav_velocity_enabled = False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = command.goal.frame_id or "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(command.goal.x_m)
        goal.pose.pose.position.y = float(command.goal.y_m)
        goal.pose.pose.orientation.z = math.sin(command.goal.yaw_rad / 2.0)
        goal.pose.pose.orientation.w = math.cos(command.goal.yaw_rad / 2.0)
        self._publish_mode("navigation")
        self._navigation = NavigationStatus(
            state="sending",
            goal_id=command.command_id,
            target=command.goal,
            message="waiting for Nav2 goal acceptance",
        )
        self._pending_nav_command_id = command.command_id
        future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback, command_id=command.command_id: self._on_nav_feedback(
                command_id, feedback
            ),
        )
        future.add_done_callback(
            lambda result, original=command: self._on_nav_goal_response(original, result)
        )

    def _on_nav_goal_response(self, command: RobotCommand, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            if self._pending_nav_command_id != command.command_id:
                self._reply(command, "completed", "navigation goal was superseded")
                return
            self._pending_nav_command_id = None
            self._navigation.state = "failed"
            self._navigation.message = f"Nav2 goal request failed: {exc}"
            self._navigation.stamp = time.time()
            self._reply(command, "failed", self._navigation.message)
            return
        if self._pending_nav_command_id != command.command_id:
            if goal_handle is not None and goal_handle.accepted:
                goal_handle.cancel_goal_async()
            if self._navigation.goal_id == command.command_id:
                self._navigation.state = "canceled"
                self._navigation.message = "navigation goal was superseded"
                self._navigation.stamp = time.time()
            self._reply(command, "completed", "navigation goal was superseded")
            return
        self._pending_nav_command_id = None
        if goal_handle is None or not goal_handle.accepted:
            self._nav_velocity_enabled = False
            self._navigation.state = "rejected"
            self._navigation.message = "Nav2 rejected goal"
            self._navigation.stamp = time.time()
            self._reply(command, "rejected", "Nav2 rejected goal")
            return
        self._active_goal_handle = goal_handle
        self._active_nav_command = command
        self._nav_velocity_enabled = True
        self._initial_goal_distance = None
        self._navigation.state = "executing"
        self._navigation.message = "Nav2 accepted goal"
        self._navigation.stamp = time.time()
        self._reply(command, "accepted", "Nav2 accepted goal")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, command_id=command.command_id: self._on_nav_result(
                command_id, result
            )
        )

    def _on_nav_feedback(self, command_id: str, feedback_message) -> None:
        if self._navigation.goal_id != command_id:
            return
        distance = float(feedback_message.feedback.distance_remaining)
        if self._initial_goal_distance is None or distance > self._initial_goal_distance:
            self._initial_goal_distance = max(distance, 1e-6)
        self._navigation.progress = max(
            0.0, min(1.0, 1.0 - distance / self._initial_goal_distance)
        )
        self._navigation.message = f"distance remaining: {distance:.2f} m"
        self._navigation.stamp = time.time()

    def _on_nav_result(self, command_id: str, future) -> None:
        if self._navigation.goal_id != command_id:
            return
        try:
            status = future.result().status
        except Exception as exc:
            self._navigation.state = "failed"
            self._navigation.message = f"Nav2 result failed: {exc}"
            status = None
        state_by_status = {
            GoalStatus.STATUS_SUCCEEDED: "succeeded",
            GoalStatus.STATUS_CANCELED: "canceled",
            GoalStatus.STATUS_ABORTED: "aborted",
        }
        if status is not None:
            self._navigation.state = state_by_status.get(status, "failed")
        self._navigation.progress = 1.0 if status == GoalStatus.STATUS_SUCCEEDED else 0.0
        if status is not None:
            self._navigation.message = f"Nav2 finished with status {status}"
        self._navigation.stamp = time.time()
        self._nav_velocity_enabled = False
        self._active_goal_handle = None
        if self._active_nav_command is not None:
            reply_status = (
                "completed"
                if status in {GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED}
                else "failed"
            )
            self._reply(
                self._active_nav_command,
                reply_status,
                self._navigation.message,
            )
        self._active_nav_command = None

    def _cancel_navigation(self, command: RobotCommand) -> None:
        if self._active_goal_handle is None and self._pending_nav_command_id is None:
            self._reply(command, "rejected", "no active navigation goal")
            return
        self._request_nav_cancel()
        self._publish_twist(0.0, 0.0)
        self._reply(command, "accepted", "navigation cancel requested")

    def _request_nav_cancel(self) -> bool:
        self._nav_velocity_enabled = False
        had_pending_goal = self._pending_nav_command_id is not None
        self._pending_nav_command_id = None
        if self._active_goal_handle is None:
            if had_pending_goal:
                self._navigation.state = "canceling"
                self._navigation.message = "cancel requested before goal acceptance"
                self._navigation.stamp = time.time()
            return had_pending_goal
        self._active_goal_handle.cancel_goal_async()
        self._navigation.state = "canceling"
        self._navigation.message = "cancel requested"
        self._navigation.stamp = time.time()
        return True

    def _on_nav_cmd_vel(self, msg: Twist) -> None:
        """Relay Nav2 velocity output to the Robot320 chassis command topic."""
        if not self._nav_velocity_enabled:
            return
        self.cmd_vel_pub.publish(msg)

    def _send_lift_command(self, command: RobotCommand) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "command_id": command.command_id,
                "action": command.lift_action,
                "target_height_m": command.lift_target_height_m,
                "stamp": command.stamp,
            },
            separators=(",", ":"),
        )
        self.lift_command_pub.publish(msg)
        self._reply(command, "accepted", "lift command forwarded")

    def _on_telemetry(self, msg: String) -> None:
        try:
            self._latest_telemetry = telemetry_from_json(msg.data)
            self._last_telemetry_received = time.monotonic()
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"invalid chassis telemetry ignored: {exc}")

    def _on_lift_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            allowed = {item.name for item in fields(LiftStatus)}
            self._lift_status = LiftStatus(**{k: v for k, v in data.items() if k in allowed})
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"invalid lift status ignored: {exc}")

    def _publish_state(self) -> None:
        telemetry = self._latest_telemetry or RobotTelemetry(robot_id=self.robot_id)
        telemetry.robot_id = self.robot_id
        telemetry.online = (
            self._last_telemetry_received > 0
            and time.monotonic() - self._last_telemetry_received < 2.0
        )
        telemetry.lift = self._lift_status
        telemetry.navigation = self._navigation
        telemetry.stamp = time.time()
        self._state_sequence += 1
        self.transport.publish_state(telemetry, self._state_sequence)

    def _publish_heartbeat(self) -> None:
        self._heartbeat_sequence += 1
        self.transport.publish_heartbeat(self._heartbeat_sequence)

    def _publish_twist(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)

    def _publish_mode(self, mode: str) -> None:
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)

    @staticmethod
    def _publish_bool(publisher, value: bool) -> None:
        msg = Bool()
        msg.data = value
        publisher.publish(msg)

    def _validate_command(self, command: RobotCommand) -> str | None:
        age = time.time() - command.stamp
        if age > self.max_command_age_s:
            return f"stale command ({age:.2f}s old)"
        if age < -self.max_command_age_s:
            return "command timestamp is too far in the future"
        previous = self._last_sequences.get(command.client_id, -1)
        if command.sequence <= previous:
            return f"duplicate or out-of-order sequence {command.sequence}"
        return None

    def _reply(self, command: RobotCommand, status: str, message: str) -> None:
        self.transport.publish_reply(
            CommandReply(
                command_id=command.command_id,
                status=status,
                robot_id=self.robot_id,
                sequence=command.sequence,
                message=message,
            )
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Robot320 Fast DDS ROS 2 gateway")
    parser.add_argument("--domain-id", type=int, default=20)
    parser.add_argument("--robot-id", default="robot320")
    parser.add_argument("--topic-prefix", default="/robot320")
    parser.add_argument("--nav-action", default="/navigate_to_pose")
    parser.add_argument("--nav-cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--telemetry-period", type=float, default=0.2)
    parser.add_argument("--heartbeat-period", type=float, default=1.0)
    parser.add_argument("--max-command-age", type=float, default=2.0)
    return parser


def main(argv: list[str] | None = None) -> int:
    if rclpy is None:
        raise RuntimeError("ROS 2 Python packages are unavailable") from _ROS_IMPORT_ERROR
    args, ros_args = build_parser().parse_known_args(argv)
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    rclpy.init(args=ros_args)
    node = Robot320FastDDSRosGateway(
        domain_id=args.domain_id,
        robot_id=args.robot_id,
        topic_prefix=args.topic_prefix,
        nav_action=args.nav_action,
        nav_cmd_vel_topic=args.nav_cmd_vel_topic,
        telemetry_period_s=args.telemetry_period,
        heartbeat_period_s=args.heartbeat_period,
        max_command_age_s=args.max_command_age,
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
