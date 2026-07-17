#!/usr/bin/env python3
"""ROS-independent Robot320 remote client using eProsima Fast DDS."""

from __future__ import annotations

import argparse
import sys
import threading
import time
from typing import Optional

from robot320_interfaces.fastdds_transport import (
    FastDDSUnavailable,
    FastDdsRemoteTransport,
)
from robot320_interfaces.messages import (
    CommandReply,
    Pose2D,
    RobotCommand,
    RobotTelemetry,
)


class RobotRemoteFastDDSClient:
    def __init__(
        self,
        domain_id: int = 20,
        client_id: str = "remote_control",
        heartbeat_period_s: float = 1.0,
        transport: FastDdsRemoteTransport | None = None,
    ):
        self.client_id = client_id
        self.latest_telemetry: Optional[RobotTelemetry] = None
        self._transport = transport or FastDdsRemoteTransport(domain_id, client_id)
        self._sequence = 0
        self._heartbeat_period_s = heartbeat_period_s
        self._running = True
        self._lock = threading.Lock()
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop,
            name="robot320-fastdds-heartbeat",
            daemon=True,
        )
        self._heartbeat_thread.start()

    def send_manual_command(
        self,
        linear_speed_mps: float = 0.0,
        angular_speed_radps: float = 0.0,
    ) -> str:
        return self._send(
            RobotCommand(
                kind="manual_motion",
                client_id=self.client_id,
                linear_speed_mps=linear_speed_mps,
                angular_speed_radps=angular_speed_radps,
            )
        )

    def send_navigation_goal(self, x_m: float, y_m: float, yaw_rad: float = 0.0) -> str:
        return self._send(
            RobotCommand(
                kind="navigation_goal",
                client_id=self.client_id,
                goal=Pose2D(x_m=x_m, y_m=y_m, yaw_rad=yaw_rad),
            )
        )

    def stop(self) -> str:
        return self._send(RobotCommand(kind="stop", client_id=self.client_id))

    def cancel_navigation(self) -> str:
        return self._send(
            RobotCommand(kind="cancel_navigation", client_id=self.client_id)
        )

    def brake(self) -> str:
        return self._send(RobotCommand(kind="brake", client_id=self.client_id))

    def emergency_stop(self) -> str:
        return self._send(RobotCommand(kind="emergency_stop", client_id=self.client_id))

    def reset_idle(self) -> str:
        return self._send(
            RobotCommand(kind="reset_emergency_stop", client_id=self.client_id)
        )

    def set_mode(self, mode: str) -> str:
        if mode not in {"idle", "manual", "navigation"}:
            raise ValueError(f"unsupported mode: {mode}")
        return self._send(
            RobotCommand(kind="set_mode", client_id=self.client_id, mode=mode)
        )

    def control_lift(self, action: str, target_height_m: float | None = None) -> str:
        if action not in {"stop", "raise", "lower", "move_to"}:
            raise ValueError(f"unsupported lift action: {action}")
        if action == "move_to" and target_height_m is None:
            raise ValueError("move_to requires target_height_m")
        return self._send(
            RobotCommand(
                kind="lift",
                client_id=self.client_id,
                lift_action=action,
                lift_target_height_m=target_height_m,
            )
        )

    def receive_telemetry(self, timeout_s: float = 0.1) -> Optional[RobotTelemetry]:
        telemetry = self._transport.receive_state(timeout_s)
        if telemetry is not None:
            self.latest_telemetry = telemetry
        return telemetry

    def receive_reply(self, timeout_s: float = 0.1) -> Optional[CommandReply]:
        return self._transport.receive_reply(timeout_s)

    def close(self) -> None:
        if not self._running:
            return
        self._running = False
        self._heartbeat_thread.join(timeout=self._heartbeat_period_s + 0.5)
        self._transport.close()

    def _send(self, command: RobotCommand) -> str:
        command.sequence = self._next_sequence()
        command.stamp = time.time()
        self._transport.publish_command(command)
        return command.command_id

    def _next_sequence(self) -> int:
        with self._lock:
            self._sequence += 1
            return self._sequence

    def _heartbeat_loop(self) -> None:
        while self._running:
            self._transport.publish_heartbeat(self._next_sequence())
            deadline = time.monotonic() + self._heartbeat_period_s
            while self._running and time.monotonic() < deadline:
                time.sleep(min(0.1, self._heartbeat_period_s))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Robot320 Fast DDS remote control")
    parser.add_argument("--domain-id", type=int, default=20)
    parser.add_argument("--client-id", default="remote_control")
    sub = parser.add_subparsers(dest="command", required=True)

    move = sub.add_parser("move")
    move.add_argument("--linear", type=float, default=0.0)
    move.add_argument("--angular", type=float, default=0.0)
    move.add_argument("--duration", type=float, default=0.0)

    goal = sub.add_parser("goal")
    goal.add_argument("--x", type=float, required=True)
    goal.add_argument("--y", type=float, required=True)
    goal.add_argument("--yaw", type=float, default=0.0)

    sub.add_parser("stop")
    sub.add_parser("brake")
    sub.add_parser("estop")
    sub.add_parser("reset")
    sub.add_parser("cancel")

    lift = sub.add_parser("lift")
    lift.add_argument("action", choices=["stop", "raise", "lower", "move_to"])
    lift.add_argument("--height", type=float)

    watch = sub.add_parser("watch")
    watch.add_argument("--seconds", type=float, default=30.0)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        client = RobotRemoteFastDDSClient(args.domain_id, args.client_id)
    except FastDDSUnavailable as exc:
        print(f"Fast DDS unavailable: {exc}", file=sys.stderr)
        return 2
    try:
        if args.command == "move":
            client.send_manual_command(args.linear, args.angular)
            if args.duration > 0:
                deadline = time.monotonic() + args.duration
                while time.monotonic() < deadline:
                    client.send_manual_command(args.linear, args.angular)
                    time.sleep(0.2)
                client.stop()
        elif args.command == "goal":
            client.send_navigation_goal(args.x, args.y, args.yaw)
        elif args.command == "stop":
            client.stop()
        elif args.command == "brake":
            client.brake()
        elif args.command == "estop":
            client.emergency_stop()
        elif args.command == "reset":
            client.reset_idle()
        elif args.command == "cancel":
            client.cancel_navigation()
        elif args.command == "lift":
            client.control_lift(args.action, args.height)
        elif args.command == "watch":
            deadline = time.monotonic() + args.seconds
            while time.monotonic() < deadline:
                telemetry = client.receive_telemetry(0.5)
                if telemetry:
                    print(_telemetry_line(telemetry))
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        client.close()


def _telemetry_line(telemetry: RobotTelemetry) -> str:
    pose = telemetry.pose
    pose_text = (
        f"({pose.x_m:.2f},{pose.y_m:.2f},{pose.yaw_rad:.2f})"
        if pose else "unavailable"
    )
    lift = telemetry.lift
    return (
        f"robot={telemetry.robot_id} online={telemetry.online} "
        f"speed={telemetry.chassis.speed_kmh} pose={pose_text} "
        f"lift_available={lift.available} lift_height={lift.height_m} "
        f"navigation={telemetry.navigation.state} faults={telemetry.faults}"
    )


if __name__ == "__main__":
    sys.exit(main())
