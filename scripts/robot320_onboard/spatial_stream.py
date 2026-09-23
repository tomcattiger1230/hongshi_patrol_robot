#!/usr/bin/env python3
"""Relay the bounded Robot320 spatial preview over one authenticated SSH stream."""

from __future__ import annotations

import json
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


KINDS = {
    "/robot320/spatial_scan": "scan",
    "/robot320/spatial_pose": "pose",
    "/robot320/spatial_map": "map",
    "/robot320/mapping_status": "mapping_status",
}


class SpatialStream(Node):
    def __init__(self) -> None:
        super().__init__("robot320_spatial_ssh_stream")
        self.output_lock = threading.Lock()
        self.mapping_writer = self.create_publisher(String, "/robot320/mapping_control", 2)
        for topic, kind in KINDS.items():
            self.create_subscription(String, topic, lambda message, value=kind: self.forward(value, message), 2)

    def emit(self, value: dict) -> None:
        line = json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
        with self.output_lock:
            print(line, flush=True)

    def forward(self, kind: str, message: String) -> None:
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict):
                raise ValueError("payload is not an object")
            self.emit({"kind": kind, "data": data})
        except (json.JSONDecodeError, ValueError, TypeError):
            self.get_logger().warning("ignored invalid %s payload", kind)

    def input_loop(self) -> None:
        for line in sys.stdin:
            request = None
            try:
                request = json.loads(line)
                action = request.get("action")
                if action not in {
                    "start", "stop", "localize", "relocalize", "stop_localization", "initial_pose"
                }:
                    raise ValueError("unsupported action")
                request_id = request.get("request_id")
                if not isinstance(request_id, str) or not request_id or len(request_id) > 80:
                    raise ValueError("missing or invalid request id")
                if self.mapping_writer.get_subscription_count() < 1:
                    raise RuntimeError("mapping bridge is not connected")
                message = String()
                payload = {"action": action, "stamp": time.time(), "request_id": request_id}
                if action == "initial_pose":
                    pose = request.get("pose")
                    if (not isinstance(pose, list) or len(pose) != 3
                            or not all(isinstance(value, (int, float)) for value in pose)):
                        raise ValueError("invalid initial pose")
                    payload["pose"] = pose
                message.data = json.dumps(payload, separators=(",", ":"))
                self.mapping_writer.publish(message)
                self.emit({
                    "kind": "control_ack",
                    "data": {"ok": True, "action": action, "request_id": request_id},
                })
            except (json.JSONDecodeError, ValueError, TypeError, AttributeError, RuntimeError):
                self.emit({
                    "kind": "control_ack",
                    "data": {"ok": False, "request_id": request.get("request_id") if isinstance(request, dict) else None},
                })


def main() -> int:
    rclpy.init()
    node = SpatialStream()
    deadline = time.monotonic() + 8.0
    while rclpy.ok() and node.mapping_writer.get_subscription_count() < 1 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.emit({
        "ready": True,
        "protocol": 1,
        "control_ready": node.mapping_writer.get_subscription_count() > 0,
    })
    threading.Thread(target=node.input_loop, daemon=True).start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, BrokenPipeError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
