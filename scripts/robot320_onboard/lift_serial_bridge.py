#!/usr/bin/env python3
"""Persistent ROS 2 to RS-485 bridge for the Robot320 lift platform."""

from __future__ import annotations

import argparse
import json
import os
import queue
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import Callable

try:
    import serial
except ImportError:  # pragma: no cover - pyserial is installed on the NUC.
    serial = None

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:  # pragma: no cover - exercised on the NUC.
    rclpy = None
    Node = object
    String = None
    ExternalShutdownException = RuntimeError


COMMANDS = {
    "raise": bytes.fromhex("AA 01 03 00"),
    "lower": bytes.fromhex("AA 01 0C 00"),
    "stop": bytes.fromhex("AA 01 0A 00"),
}


@dataclass
class LocalCommand:
    action: str
    done: threading.Event = field(default_factory=threading.Event)
    response: str = "error command was not processed"


class LiftSerialController:
    """Keep the FTDI port open and write exactly one frame per command."""

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        serial_factory: Callable[..., object] | None = None,
    ) -> None:
        self.port = port
        if serial_factory is None:
            if serial is None:
                raise RuntimeError("pyserial is required")
            serial_factory = serial.Serial
        self._serial_factory = serial_factory
        self._serial = None

    @property
    def connected(self) -> bool:
        return bool(self._serial is not None and self._serial.is_open)

    def connect(self) -> None:
        if self.connected:
            return
        self._serial = self._serial_factory(
            port=self.port,
            baudrate=9600,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=1,
            write_timeout=1,
        )

    def send(self, action: str) -> None:
        try:
            payload = COMMANDS[action]
        except KeyError as exc:
            raise ValueError(f"unsupported lift action: {action}") from exc
        self.connect()
        written = self._serial.write(payload)
        self._serial.flush()
        if written != len(payload):
            raise IOError(f"short serial write: {written}/{len(payload)} bytes")

    def disconnect(self, safe_stop: bool = False) -> None:
        if not self.connected:
            self._serial = None
            return
        try:
            if safe_stop:
                self._serial.write(COMMANDS["stop"])
                self._serial.flush()
        finally:
            self._serial.close()
            self._serial = None


class LiftSerialBridge(Node):
    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        topic_prefix: str = "/robot320",
        reconnect_period_s: float = 2.0,
        socket_path: str | None = None,
    ) -> None:
        super().__init__("robot320_lift_serial_bridge")
        prefix = topic_prefix.rstrip("/")
        self.controller = LiftSerialController(port)
        self.status_pub = self.create_publisher(String, f"{prefix}/lift/status", 10)
        self.create_subscription(String, f"{prefix}/lift/command", self._on_command, 10)
        self.create_timer(reconnect_period_s, self._ensure_connected)
        self.create_timer(0.02, self._drain_local_commands)
        self._moving = False
        self._fault: str | None = None
        self._local_commands: queue.Queue[LocalCommand] = queue.Queue()
        self._socket_path = socket_path or f"/run/user/{os.getuid()}/robot320-lift.sock"
        self._socket_stop = threading.Event()
        self._socket_server: socket.socket | None = None
        self._socket_thread = threading.Thread(
            target=self._serve_local_commands,
            name="lift-local-command-server",
            daemon=True,
        )
        self._ensure_connected()
        self._socket_thread.start()

    def _ensure_connected(self) -> None:
        if self.controller.connected:
            return
        try:
            self.controller.connect()
            self._fault = None
            self.get_logger().info(f"lift serial connected: {self.controller.port}")
        except Exception as exc:
            self._fault = str(exc)
            self.get_logger().warning(f"lift serial unavailable: {exc}")
        self._publish_status()

    def _on_command(self, message: String) -> None:
        try:
            data = json.loads(message.data)
            action = data.get("action")
            self._apply_action(action, data.get("command_id", ""))
        except Exception as exc:
            self._handle_command_error(exc)
        self._publish_status()

    def _apply_action(self, action: str, command_id: str = "") -> None:
        if action not in COMMANDS:
            raise ValueError(f"unsupported lift action: {action}")
        self.controller.send(action)
        self._moving = action != "stop"
        self._fault = None
        self.get_logger().info(
            f"lift command sent once: action={action} command_id={command_id}"
        )

    def _handle_command_error(self, exc: Exception) -> None:
        self._fault = str(exc)
        self.controller.disconnect()
        self.get_logger().error(f"lift command failed: {exc}")

    def _drain_local_commands(self) -> None:
        for _ in range(20):
            try:
                request = self._local_commands.get_nowait()
            except queue.Empty:
                return
            try:
                self._apply_action(request.action, "wan-ssh")
                request.response = f"ok {request.action}"
            except Exception as exc:
                self._handle_command_error(exc)
                request.response = f"error {exc}"
            finally:
                self._publish_status()
                request.done.set()

    def _serve_local_commands(self) -> None:
        try:
            if os.path.exists(self._socket_path):
                os.unlink(self._socket_path)
            server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self._socket_server = server
            server.bind(self._socket_path)
            os.chmod(self._socket_path, 0o600)
            server.listen(8)
            server.settimeout(0.5)
            while not self._socket_stop.is_set():
                try:
                    connection, _ = server.accept()
                except socket.timeout:
                    continue
                with connection:
                    connection.settimeout(2.0)
                    data = connection.recv(64)
                    action = data.decode("ascii", errors="strict").strip()
                    request = LocalCommand(action=action)
                    self._local_commands.put(request)
                    if request.done.wait(5.0):
                        response = request.response
                    else:
                        response = "error command timeout"
                    connection.sendall((response + "\n").encode("utf-8"))
        except Exception as exc:
            if not self._socket_stop.is_set():
                self.get_logger().error(f"lift local command socket failed: {exc}")
        finally:
            if self._socket_server is not None:
                self._socket_server.close()
                self._socket_server = None
            if os.path.exists(self._socket_path):
                os.unlink(self._socket_path)

    def _publish_status(self) -> None:
        message = String()
        message.data = json.dumps(
            {
                "available": self.controller.connected,
                "height_m": None,
                "target_height_m": None,
                "moving": self._moving,
                "upper_limit": False,
                "lower_limit": False,
                "fault_code": self._fault,
                "stamp": time.time(),
            },
            separators=(",", ":"),
        )
        self.status_pub.publish(message)

    def destroy_node(self) -> bool:
        self._socket_stop.set()
        if self._socket_server is not None:
            self._socket_server.close()
        self._socket_thread.join(timeout=1.0)
        self.controller.disconnect(safe_stop=True)
        return super().destroy_node()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Robot320 lift RS-485 bridge")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--topic-prefix", default="/robot320")
    parser.add_argument("--socket-path", default=None)
    return parser


def main(argv: list[str] | None = None) -> int:
    if rclpy is None:
        raise RuntimeError("ROS 2 rclpy/std_msgs are required")
    args = build_parser().parse_args(argv)
    rclpy.init(args=None)
    node = LiftSerialBridge(
        port=args.port,
        topic_prefix=args.topic_prefix,
        socket_path=args.socket_path,
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
