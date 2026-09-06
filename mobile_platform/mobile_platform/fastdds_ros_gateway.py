#!/usr/bin/env python3
"""NUC gateway using ROS 2 String topics for external Robot320 communication."""

from __future__ import annotations

import argparse
import json
import logging
import math
import os
from pathlib import Path
import queue
import re
import sys
import time
from dataclasses import fields

from robot320_interfaces.messages import (
    ChassisStatus,
    CommandReply,
    Heartbeat,
    LiftStatus,
    NavigationStatus,
    Pose2D,
    RemoteMap,
    RobotCommand,
    RobotTelemetry,
    heartbeat_from_json,
    robot_command_from_json,
    remote_map,
    telemetry_from_json,
    to_json,
)

try:
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import Twist, TwistStamped
    from nav_msgs.msg import OccupancyGrid, Odometry
    from nav2_msgs.action import NavigateToPose
    from nav2_msgs.srv import LoadMap
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.parameter_client import AsyncParameterClient
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, String
    from std_srvs.srv import SetBool, Trigger
except ImportError as exc:  # pragma: no cover - evaluated on the NUC.
    rclpy = None
    Node = object
    GoalStatus = Twist = TwistStamped = OccupancyGrid = Odometry = None
    NavigateToPose = ActionClient = QoSProfile = None
    Parameter = AsyncParameterClient = None
    DurabilityPolicy = ReliabilityPolicy = SetBool = Trigger = LoadMap = None
    Bool = String = None
    _ROS_IMPORT_ERROR = exc
else:
    _ROS_IMPORT_ERROR = None

try:
    from cartographer_ros_msgs.srv import WriteState
except ImportError:  # pragma: no cover - optional Cartographer deployment.
    WriteState = None

try:
    from slam_toolbox.srv import DeserializePoseGraph, SaveMap, SerializePoseGraph
except ImportError:  # pragma: no cover - optional outside SLAM deployments.
    DeserializePoseGraph = SaveMap = SerializePoseGraph = None


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
        self._map_pub = node.create_publisher(String, f"{prefix}/map", 10)
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

    def publish_map(self, snapshot: RemoteMap) -> None:
        self._map_pub.publish(_string_message(to_json(snapshot)))

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
        map_topic: str = "/map",
        map_save_service: str = "/robot320/save_persistent_map",
        map_storage_directory: str = "~/robot320_maps",
        exploration_service: str = "/robot320/set_exploration_enabled",
        exploration_status_topic: str = "/robot320/exploration_enabled",
        cartographer_state_file: str = "~/robot320_maps/patrol_current.pbstream",
        map_publish_period_s: float = 1.0,
        odometry_topic: str = "",
        odometry_pose_frame: str = "map",
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
        self._latest_map: RemoteMap | None = None
        self._simulation_telemetry: RobotTelemetry | None = None
        self._last_odometry_received = 0.0
        self.odometry_pose_frame = odometry_pose_frame or "map"
        self._exploration_enabled = False
        self._map_operation_command: RobotCommand | None = None
        self.map_storage_directory = Path(
            os.path.expanduser(map_storage_directory)
        ).resolve()

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
        self.create_subscription(
            Twist, nav_cmd_vel_topic, self._on_nav_cmd_vel, 10
        )
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, map_topic, self._on_map, map_qos)
        if odometry_topic:
            self.create_subscription(Odometry, odometry_topic, self._on_odometry, 10)
        self.create_subscription(
            Bool,
            exploration_status_topic,
            self._on_exploration_status,
            10,
        )
        self.nav_client = ActionClient(self, NavigateToPose, nav_action)
        self.map_save_client = self.create_client(Trigger, map_save_service)
        self.slam_serialize_client = (
            self.create_client(SerializePoseGraph, "/slam_toolbox/serialize_map")
            if SerializePoseGraph is not None
            else None
        )
        self.slam_save_map_client = (
            self.create_client(SaveMap, "/slam_toolbox/save_map")
            if SaveMap is not None
            else None
        )
        self.slam_deserialize_client = (
            self.create_client(DeserializePoseGraph, "/slam_toolbox/deserialize_map")
            if DeserializePoseGraph is not None
            else None
        )
        self.map_load_client = (
            self.create_client(LoadMap, "/map_server/load_map")
            if LoadMap is not None
            else None
        )
        self.map_manager_parameter_client = AsyncParameterClient(
            self, "persistent_map_manager"
        )
        self.exploration_client = self.create_client(SetBool, exploration_service)
        self.cartographer_state_file = os.path.abspath(
            os.path.expanduser(cartographer_state_file)
        )
        self.cartographer_save_client = (
            self.create_client(WriteState, "/write_state")
            if WriteState is not None
            else None
        )

        self.create_timer(0.05, self._poll_commands)
        self.create_timer(telemetry_period_s, self._publish_state)
        self.create_timer(heartbeat_period_s, self._publish_heartbeat)
        self.create_timer(map_publish_period_s, self._publish_map)
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
            self._last_sequences[self._sequence_key(command)] = command.sequence
            try:
                self._dispatch(command)
            except Exception as exc:
                self.get_logger().error(
                    f"failed to dispatch Fast DDS command {command.command_id}: {exc}"
                )
                self._reply(command, "failed", str(exc))

    def _dispatch(self, command: RobotCommand) -> None:
        if command.kind == "manual_motion":
            self._disable_exploration_for_operator()
            self._request_nav_cancel()
            self._publish_twist(command.linear_speed_mps, command.angular_speed_radps)
            self._publish_mode("manual")
            self._reply(command, "accepted", "manual motion forwarded")
        elif command.kind == "stop":
            self._disable_exploration_for_operator()
            self._request_nav_cancel()
            self._publish_twist(0.0, 0.0)
            self._reply(command, "accepted", "stop forwarded")
        elif command.kind == "brake":
            self._disable_exploration_for_operator()
            self._request_nav_cancel()
            self._publish_bool(self.brake_pub, True)
            self._reply(command, "accepted", "brake forwarded")
        elif command.kind == "emergency_stop":
            self._disable_exploration_for_operator()
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
            self._disable_exploration_for_operator()
            self._send_navigation_goal(command)
        elif command.kind == "cancel_navigation":
            self._cancel_navigation(command)
        elif command.kind == "lift":
            self._send_lift_command(command)
        elif command.kind == "save_map":
            self._save_map(command)
        elif command.kind == "load_map":
            self._load_map(command)
        elif command.kind == "set_exploration":
            self._set_exploration(command)
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

    def _on_nav_cmd_vel(self, msg) -> None:
        """Relay Nav2 velocity output to the Robot320 chassis command topic."""
        if not self._nav_velocity_enabled:
            return
        self.cmd_vel_pub.publish(getattr(msg, "twist", msg))

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

    def _on_map(self, msg: OccupancyGrid) -> None:
        try:
            self._latest_map = _remote_map_from_occupancy_grid(msg)
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f"invalid occupancy grid ignored: {exc}")

    def _on_odometry(self, msg: Odometry) -> None:
        """Provide simulation telemetry when the physical chassis bridge is absent."""
        pose = msg.pose.pose
        orientation = pose.orientation
        yaw = math.atan2(
            2.0
            * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0
            - 2.0
            * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        speed_mps = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        stamp = msg.header.stamp
        stamp_seconds = float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0
        self._simulation_telemetry = RobotTelemetry(
            robot_id=self.robot_id,
            online=True,
            chassis=ChassisStatus(
                connected=True,
                enabled=True,
                speed_kmh=speed_mps * 3.6,
            ),
            pose=Pose2D(
                x_m=float(pose.position.x),
                y_m=float(pose.position.y),
                yaw_rad=yaw,
                frame_id=self.odometry_pose_frame,
                stamp=stamp_seconds or time.time(),
            ),
        )
        self._last_odometry_received = time.monotonic()

    def _on_exploration_status(self, msg: Bool) -> None:
        self._exploration_enabled = bool(msg.data)

    def _set_exploration(self, command: RobotCommand) -> None:
        if command.exploration_enabled is None:
            self._reply(command, "rejected", "exploration state is missing")
            return
        if not self.exploration_client.service_is_ready():
            self._reply(command, "rejected", "frontier exploration service is unavailable")
            return
        enabled = bool(command.exploration_enabled)
        if enabled:
            self._request_nav_cancel()
            self._publish_mode("navigation")
        else:
            self._publish_twist(0.0, 0.0)
        request = SetBool.Request()
        request.data = enabled
        future = self.exploration_client.call_async(request)
        future.add_done_callback(
            lambda result, original=command: self._on_exploration_result(
                original, result
            )
        )

    def _on_exploration_result(self, command: RobotCommand, future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self._reply(command, "failed", f"exploration request failed: {exc}")
            return
        if result.success:
            self._exploration_enabled = bool(command.exploration_enabled)
        status = "completed" if result.success else "failed"
        self._reply(command, status, result.message or "exploration state updated")

    def _disable_exploration_for_operator(self) -> None:
        if not self._exploration_enabled or not self.exploration_client.service_is_ready():
            return
        request = SetBool.Request()
        request.data = False
        self._exploration_enabled = False
        self.exploration_client.call_async(request)

    def _publish_map(self) -> None:
        if self._latest_map is not None:
            self.transport.publish_map(self._latest_map)

    def _save_map(self, command: RobotCommand) -> None:
        if command.map_prefix:
            try:
                prefix = _validated_map_prefix(
                    command.map_prefix, self.map_storage_directory
                )
            except ValueError as exc:
                self._reply(command, "rejected", str(exc))
                return
            if self._map_operation_command is not None:
                self._reply(command, "rejected", "another map operation is running")
                return
            if (
                self.slam_serialize_client is None
                or self.slam_save_map_client is None
                or not self.slam_serialize_client.service_is_ready()
                or not self.slam_save_map_client.service_is_ready()
            ):
                self._reply(command, "rejected", "SLAM Toolbox save services are unavailable")
                return
            prefix.parent.mkdir(parents=True, exist_ok=True)
            self._map_operation_command = command
            request = SerializePoseGraph.Request()
            request.filename = str(prefix)
            future = self.slam_serialize_client.call_async(request)
            future.add_done_callback(
                lambda result, original=command, target=prefix: self._on_posegraph_saved(
                    original, target, result
                )
            )
            return
        if self.map_save_client.service_is_ready():
            future = self.map_save_client.call_async(Trigger.Request())
            future.add_done_callback(
                lambda result, original=command: self._on_save_map_result(
                    original, result
                )
            )
            return
        if (
            self.cartographer_save_client is not None
            and self.cartographer_save_client.service_is_ready()
        ):
            os.makedirs(os.path.dirname(self.cartographer_state_file), exist_ok=True)
            request = WriteState.Request()
            request.filename = self.cartographer_state_file
            request.include_unfinished_submaps = True
            future = self.cartographer_save_client.call_async(request)
            future.add_done_callback(
                lambda result, original=command: self._on_cartographer_save_result(
                    original, result
                )
            )
            return
        self._reply(
            command,
            "rejected",
            "no SLAM Toolbox or Cartographer map save service is available",
        )

    def _on_posegraph_saved(self, command: RobotCommand, prefix: Path, future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self._finish_map_operation(command, "failed", f"pose graph save failed: {exc}")
            return
        if result.result != SerializePoseGraph.Response.RESULT_SUCCESS:
            self._finish_map_operation(
                command, "failed", f"pose graph save returned result {result.result}"
            )
            return
        request = SaveMap.Request()
        request.name.data = str(prefix)
        future = self.slam_save_map_client.call_async(request)
        future.add_done_callback(
            lambda result, original=command, target=prefix: self._on_session_map_saved(
                original, target, result
            )
        )

    def _on_session_map_saved(self, command: RobotCommand, prefix: Path, future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self._finish_map_operation(command, "failed", f"occupancy map save failed: {exc}")
            return
        if result.result != SaveMap.Response.RESULT_SUCCESS:
            self._finish_map_operation(
                command, "failed", f"occupancy map save returned result {result.result}"
            )
            return
        self._finish_map_operation(command, "completed", f"map session saved: {prefix}")

    def _load_map(self, command: RobotCommand) -> None:
        if not command.map_prefix:
            self._reply(command, "rejected", "map prefix is missing")
            return
        try:
            prefix = _validated_map_prefix(command.map_prefix, self.map_storage_directory)
        except ValueError as exc:
            self._reply(command, "rejected", str(exc))
            return
        if self._map_operation_command is not None:
            self._reply(command, "rejected", "another map operation is running")
            return
        self._disable_exploration_for_operator()
        self._request_nav_cancel()
        self._publish_twist(0.0, 0.0)
        mode = command.map_mode or "continuing"
        self._map_operation_command = command
        if mode == "continuing":
            if (
                self.slam_deserialize_client is None
                or not self.slam_deserialize_client.service_is_ready()
            ):
                self._finish_map_operation(
                    command, "rejected", "SLAM Toolbox deserialize service is unavailable"
                )
                return
            if not Path(f"{prefix}.posegraph").is_file() or not Path(
                f"{prefix}.data"
            ).is_file():
                self._finish_map_operation(
                    command, "rejected", "map session is missing .posegraph or .data"
                )
                return
            if self.map_manager_parameter_client.services_are_ready():
                future = self.map_manager_parameter_client.set_parameters(
                    [
                        Parameter(
                            "map_prefix",
                            Parameter.Type.STRING,
                            str(prefix),
                        )
                    ]
                )
                future.add_done_callback(
                    lambda result, original=command, target=prefix: self._on_map_prefix_changed(
                        original, target, result
                    )
                )
            else:
                self.get_logger().warning(
                    "persistent_map_manager is unavailable; loaded map will not auto-save"
                )
                self._deserialize_map(command, prefix)
            return
        if mode == "localization":
            if self.map_load_client is None or not self.map_load_client.service_is_ready():
                self._finish_map_operation(
                    command, "rejected", "map_server load service is unavailable"
                )
                return
            yaml_path = Path(f"{prefix}.yaml")
            if not yaml_path.is_file():
                self._finish_map_operation(command, "rejected", "map YAML is missing")
                return
            request = LoadMap.Request()
            request.map_url = str(yaml_path)
            future = self.map_load_client.call_async(request)
            future.add_done_callback(
                lambda result, original=command, target=prefix: self._on_map_loaded(
                    original, target, mode, result
                )
            )
            return
        self._finish_map_operation(command, "rejected", f"unsupported map mode: {mode}")

    def _on_map_prefix_changed(
        self, command: RobotCommand, prefix: Path, future
    ) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self._finish_map_operation(
                command, "failed", f"automatic save path update failed: {exc}"
            )
            return
        results = getattr(response, "results", response)
        reason = next(
            (result.reason for result in results if not result.successful),
            "",
        )
        if reason:
            self._finish_map_operation(
                command, "failed", f"automatic save path update failed: {reason}"
            )
            return
        self._deserialize_map(command, prefix)

    def _deserialize_map(self, command: RobotCommand, prefix: Path) -> None:
        request = DeserializePoseGraph.Request()
        request.filename = str(prefix)
        request.match_type = DeserializePoseGraph.Request.START_AT_FIRST_NODE
        future = self.slam_deserialize_client.call_async(request)
        future.add_done_callback(
            lambda result, original=command, target=prefix: self._on_map_loaded(
                original, target, "continuing", result
            )
        )

    def _on_map_loaded(
        self, command: RobotCommand, prefix: Path, mode: str, future
    ) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self._finish_map_operation(command, "failed", f"map load failed: {exc}")
            return
        success_code = (
            DeserializePoseGraph.Response.RESULT_SUCCESS
            if mode == "continuing"
            else LoadMap.Response.RESULT_SUCCESS
        )
        if result.result != success_code:
            self._finish_map_operation(
                command, "failed", f"map load returned result {result.result}"
            )
            return
        self._finish_map_operation(command, "completed", f"map loaded: {prefix} ({mode})")

    def _finish_map_operation(
        self, command: RobotCommand, status: str, message: str
    ) -> None:
        if self._map_operation_command is command:
            self._map_operation_command = None
        self._reply(command, status, message)

    def _on_save_map_result(self, command: RobotCommand, future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self._reply(command, "failed", f"map save request failed: {exc}")
            return
        status = "completed" if result.success else "failed"
        self._reply(command, status, result.message or "map save finished")

    def _on_cartographer_save_result(self, command: RobotCommand, future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self._reply(command, "failed", f"Cartographer save failed: {exc}")
            return
        status_object = getattr(result, "status", None)
        code = int(getattr(status_object, "code", 0))
        message = str(getattr(status_object, "message", ""))
        if code == 0:
            self._reply(
                command,
                "completed",
                message or f"Cartographer state saved: {self.cartographer_state_file}",
            )
        else:
            self._reply(command, "failed", message or f"Cartographer status {code}")

    def _publish_state(self) -> None:
        using_chassis = self._latest_telemetry is not None
        telemetry = (
            self._latest_telemetry
            or self._simulation_telemetry
            or RobotTelemetry(robot_id=self.robot_id)
        )
        telemetry.robot_id = self.robot_id
        source_received = (
            self._last_telemetry_received if using_chassis else self._last_odometry_received
        )
        telemetry.online = source_received > 0 and time.monotonic() - source_received < 2.0
        telemetry.lift = self._lift_status
        telemetry.navigation = self._navigation
        telemetry.map_revision = (
            self._latest_map.revision if self._latest_map is not None else None
        )
        telemetry.exploration_enabled = self._exploration_enabled
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
        previous = self._last_sequences.get(self._sequence_key(command), -1)
        if command.sequence <= previous:
            return f"duplicate or out-of-order sequence {command.sequence}"
        return None

    @staticmethod
    def _sequence_key(command: RobotCommand) -> str:
        session = command.session_id or "legacy"
        return f"{command.client_id}:{session}"

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


def _validated_map_prefix(value: str, storage_directory: Path) -> Path:
    """Resolve a client-provided prefix while confining it to the map directory."""
    if not value.strip():
        raise ValueError("map prefix is missing")
    expanded = Path(os.path.expanduser(value)).resolve()
    root = storage_directory.resolve()
    if expanded.parent != root:
        raise ValueError(f"map prefix must be directly inside {root}")
    if not re.fullmatch(r"[A-Za-z0-9._-]{1,80}", expanded.name):
        raise ValueError("map name contains unsupported characters")
    return expanded


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Robot320 Fast DDS ROS 2 gateway")
    parser.add_argument("--domain-id", type=int, default=20)
    parser.add_argument("--robot-id", default="robot320")
    parser.add_argument("--topic-prefix", default="/robot320")
    parser.add_argument("--nav-action", default="/navigate_to_pose")
    parser.add_argument("--nav-cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--map-storage-directory", default="~/robot320_maps")
    parser.add_argument(
        "--map-save-service", default="/robot320/save_persistent_map"
    )
    parser.add_argument(
        "--exploration-service", default="/robot320/set_exploration_enabled"
    )
    parser.add_argument(
        "--exploration-status-topic", default="/robot320/exploration_enabled"
    )
    parser.add_argument(
        "--cartographer-state-file",
        default="~/robot320_maps/patrol_current.pbstream",
    )
    parser.add_argument("--map-publish-period", type=float, default=1.0)
    parser.add_argument(
        "--odometry-topic",
        default="",
        help="simulation-only odometry fallback, for example /odom",
    )
    parser.add_argument("--odometry-pose-frame", default="map")
    parser.add_argument("--telemetry-period", type=float, default=0.2)
    parser.add_argument("--heartbeat-period", type=float, default=1.0)
    parser.add_argument("--max-command-age", type=float, default=2.0)
    return parser


def main(argv: list[str] | None = None) -> int:
    if rclpy is None:
        raise RuntimeError("ROS 2 Python packages are unavailable") from _ROS_IMPORT_ERROR
    args, ros_args = build_parser().parse_known_args(argv)
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    # The Lyrical simulation and navigation stack is validated with Cyclone DDS.
    # Keep an explicit operator override, but never silently fall back to Fast DDS
    # RMW, which can retain unbounded large-map samples for standalone readers.
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    rclpy.init(args=ros_args)
    node = Robot320FastDDSRosGateway(
        domain_id=args.domain_id,
        robot_id=args.robot_id,
        topic_prefix=args.topic_prefix,
        nav_action=args.nav_action,
        nav_cmd_vel_topic=args.nav_cmd_vel_topic,
        map_topic=args.map_topic,
        map_save_service=args.map_save_service,
        map_storage_directory=args.map_storage_directory,
        exploration_service=args.exploration_service,
        exploration_status_topic=args.exploration_status_topic,
        cartographer_state_file=args.cartographer_state_file,
        map_publish_period_s=args.map_publish_period,
        odometry_topic=args.odometry_topic,
        odometry_pose_frame=args.odometry_pose_frame,
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


def _remote_map_from_occupancy_grid(message) -> RemoteMap:
    orientation = message.info.origin.orientation
    sin_yaw = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y
    )
    cos_yaw = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    stamp = message.header.stamp
    stamp_seconds = float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0
    return remote_map(
        width=int(message.info.width),
        height=int(message.info.height),
        resolution=float(message.info.resolution),
        origin_x=float(message.info.origin.position.x),
        origin_y=float(message.info.origin.position.y),
        origin_yaw=math.atan2(sin_yaw, cos_yaw),
        data=message.data,
        frame_id=message.header.frame_id or "map",
        stamp=stamp_seconds or time.time(),
    )


if __name__ == "__main__":
    sys.exit(main())
