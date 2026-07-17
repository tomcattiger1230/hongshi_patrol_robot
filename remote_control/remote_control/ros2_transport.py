"""ROS 2 transport used automatically on Ubuntu systems with rclpy."""

from __future__ import annotations

import os
import queue
import threading
import time
from typing import Optional

from robot320_interfaces.messages import (
    CommandReply,
    Heartbeat,
    RobotCommand,
    RobotTelemetry,
    heartbeat_from_json,
    reply_from_json,
    telemetry_from_json,
    to_json,
)

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String
except ImportError as exc:  # pragma: no cover - depends on the host OS.
    rclpy = None
    SingleThreadedExecutor = String = None
    _ROS_IMPORT_ERROR = exc
else:
    _ROS_IMPORT_ERROR = None


class Ros2Unavailable(RuntimeError):
    pass


def ros2_available() -> bool:
    return rclpy is not None


def ros2_unavailability_reason() -> str:
    return str(_ROS_IMPORT_ERROR) if _ROS_IMPORT_ERROR is not None else "unknown error"


class Ros2RemoteTransport:
    backend = "ros2"

    def __init__(
        self,
        domain_id: int = 20,
        client_id: str = "remote_control",
        topic_prefix: str = "/robot320",
    ):
        if rclpy is None:
            raise Ros2Unavailable(
                "ROS 2 Python packages are unavailable. Recreate the desktop environment "
                "with './scripts/uv_setup.sh desktop' after installing ROS 2, then retry. "
                f"Import error: {ros2_unavailability_reason()}"
            ) from _ROS_IMPORT_ERROR
        self.client_id = client_id
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            os.environ["ROS_DOMAIN_ID"] = str(domain_id)
            rclpy.init(args=None)

        prefix = topic_prefix.rstrip("/")
        self._states: queue.Queue[RobotTelemetry] = queue.Queue()
        self._replies: queue.Queue[CommandReply] = queue.Queue()
        self._heartbeats: queue.Queue[Heartbeat] = queue.Queue()
        self._node = rclpy.create_node(f"robot320_remote_{_safe_node_name(client_id)}")
        self._command_pub = self._node.create_publisher(String, f"{prefix}/command", 10)
        self._heartbeat_pub = self._node.create_publisher(
            String, f"{prefix}/heartbeat", 10
        )
        self._node.create_subscription(String, f"{prefix}/state", self._on_state, 10)
        self._node.create_subscription(String, f"{prefix}/reply", self._on_reply, 10)
        self._node.create_subscription(
            String, f"{prefix}/heartbeat", self._on_heartbeat, 10
        )
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin,
            name="robot320-ros2-remote",
            daemon=True,
        )
        self._closed = False
        self._spin_thread.start()

    def publish_command(self, command: RobotCommand) -> None:
        self._command_pub.publish(_string_message(to_json(command)))

    def publish_heartbeat(self, sequence: int) -> None:
        heartbeat = Heartbeat(
            node_id=self.client_id,
            role="remote",
            sequence=sequence,
            timestamp_ms=int(time.time() * 1000.0),
        )
        self._heartbeat_pub.publish(_string_message(to_json(heartbeat)))

    def receive_state(self, timeout_s: float = 0.1) -> Optional[RobotTelemetry]:
        return _queue_get(self._states, timeout_s)

    def receive_reply(self, timeout_s: float = 0.1) -> Optional[CommandReply]:
        return _queue_get(self._replies, timeout_s)

    def receive_heartbeat(self, timeout_s: float = 0.1) -> Optional[Heartbeat]:
        return _queue_get(self._heartbeats, timeout_s)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._executor.shutdown(timeout_sec=1.0)
        self._spin_thread.join(timeout=1.5)
        self._executor.remove_node(self._node)
        self._node.destroy_node()
        if self._owns_context and rclpy.ok():
            rclpy.shutdown()

    def _on_state(self, message: String) -> None:
        self._states.put(telemetry_from_json(message.data))

    def _on_reply(self, message: String) -> None:
        self._replies.put(reply_from_json(message.data))

    def _on_heartbeat(self, message: String) -> None:
        heartbeat = heartbeat_from_json(message.data)
        if heartbeat.role == "robot":
            self._heartbeats.put(heartbeat)


def _string_message(payload: str):
    message = String()
    message.data = payload
    return message


def _safe_node_name(value: str) -> str:
    normalized = "".join(char if char.isalnum() else "_" for char in value)
    return normalized.strip("_") or "client"


def _queue_get(items: queue.Queue, timeout_s: float):
    try:
        return items.get(timeout=max(0.0, timeout_s))
    except queue.Empty:
        return None
