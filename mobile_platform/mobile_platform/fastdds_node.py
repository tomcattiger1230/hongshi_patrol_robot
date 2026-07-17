#!/usr/bin/env python3
"""ROS-independent Fast DDS to Robot320 CAN bridge for the NUC."""

from __future__ import annotations

import argparse
import logging
import sys
import time

from robot320_interfaces.fastdds_transport import (
    FastDDSUnavailable,
    FastDdsRobotTransport,
)
from robot320_interfaces.messages import CommandReply, RobotCommand, RobotTelemetry

from .controlcan import CANAdapterConfig
from .onboard_node import OnboardNode
from .robot320 import Robot320Platform
from .safety import SafetyConfig, SafetyController


LOGGER = logging.getLogger(__name__)


class Robot320FastDDSBridge:
    def __init__(
        self,
        robot: Robot320Platform,
        safety: SafetyController,
        domain_id: int = 20,
        robot_id: str = "robot320",
        rpm_per_mps: float = 500.0,
        steering_gain_deg_per_radps: float = 180.0,
        telemetry_period_s: float = 0.2,
        heartbeat_period_s: float = 1.0,
        max_command_age_s: float = 2.0,
        transport: FastDdsRobotTransport | None = None,
    ):
        self.robot_id = robot_id
        self.safety = safety
        self.controller = OnboardNode(
            robot=robot,
            command_subscriber=_NoCommandSubscriber(),
            telemetry_publisher=_NoTelemetryPublisher(),
            safety=safety,
            rpm_per_mps=rpm_per_mps,
            steering_gain_deg_per_radps=steering_gain_deg_per_radps,
        )
        self.transport = transport or FastDdsRobotTransport(domain_id, robot_id)
        self.telemetry_period_s = telemetry_period_s
        self.heartbeat_period_s = heartbeat_period_s
        self.max_command_age_s = max_command_age_s
        self._last_sequences: dict[str, int] = {}
        self._state_sequence = 0
        self._heartbeat_sequence = 0
        self._last_stop_due_timeout = False

    def run(self) -> None:
        self.controller.robot.connect(start_receiver=True)
        next_telemetry = time.monotonic()
        next_heartbeat = time.monotonic()
        try:
            while True:
                command = self.transport.receive_command(timeout_s=0.05)
                if command is not None:
                    self._handle_command(command)

                if self.safety.timed_out() and not self._last_stop_due_timeout:
                    LOGGER.warning("command timeout, stopping chassis")
                    self.controller.robot.stop_motor()
                    self._last_stop_due_timeout = True

                now = time.monotonic()
                if now >= next_telemetry:
                    self._state_sequence += 1
                    self.transport.publish_state(self.build_telemetry(), self._state_sequence)
                    next_telemetry = now + self.telemetry_period_s
                if now >= next_heartbeat:
                    self._heartbeat_sequence += 1
                    self.transport.publish_heartbeat(self._heartbeat_sequence)
                    next_heartbeat = now + self.heartbeat_period_s
        finally:
            self.controller.robot.disconnect()
            self.transport.close()

    def build_telemetry(self) -> RobotTelemetry:
        telemetry = self.controller.build_telemetry()
        telemetry.robot_id = self.robot_id
        telemetry.online = True
        telemetry.stamp = time.time()
        return telemetry

    def _handle_command(self, command: RobotCommand) -> None:
        reason = self._validate_command(command)
        if reason:
            self._reply(command, "rejected", reason)
            return

        self._last_sequences[command.client_id] = command.sequence
        try:
            if command.kind in {
                "manual_motion",
                "stop",
                "brake",
                "emergency_stop",
                "reset_emergency_stop",
                "set_mode",
            }:
                chassis_command = self.safety.accept(command.to_chassis_command())
                self.controller.apply_command(chassis_command)
                self._last_stop_due_timeout = False
                self._reply(command, "accepted", "chassis command applied")
            elif command.kind == "navigation_goal":
                self._reply(
                    command,
                    "rejected",
                    "navigation_goal requires the ROS 2 Fast DDS gateway",
                )
            elif command.kind == "lift":
                self._reply(
                    command,
                    "rejected",
                    "lift hardware adapter is not configured",
                )
        except Exception as exc:
            LOGGER.exception("failed to apply command %s", command.command_id)
            self._reply(command, "failed", str(exc))

    def _validate_command(self, command: RobotCommand) -> str | None:
        age = time.time() - command.stamp
        if age > self.max_command_age_s:
            return f"stale command ({age:.2f}s old)"
        if age < -self.max_command_age_s:
            return "command timestamp is too far in the future"
        previous = self._last_sequences.get(command.client_id, -1)
        if command.sequence <= previous:
            return f"duplicate or out-of-order sequence {command.sequence}"
        if command.kind == "navigation_goal" and command.goal is None:
            return "navigation_goal requires goal"
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


class _NoCommandSubscriber:
    def receive_command(self, timeout_s: float = 0.1):
        return None


class _NoTelemetryPublisher:
    def publish_telemetry(self, telemetry: RobotTelemetry) -> None:
        return None


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Robot320 Fast DDS CAN bridge")
    parser.add_argument("--lib", default=None)
    parser.add_argument("--device-index", type=int, default=0)
    parser.add_argument("--can-index", type=int, default=0)
    parser.add_argument("--domain-id", type=int, default=20)
    parser.add_argument("--robot-id", default="robot320")
    parser.add_argument("--command-timeout", type=float, default=0.6)
    parser.add_argument("--max-command-age", type=float, default=2.0)
    parser.add_argument("--max-linear-speed", type=float, default=0.8)
    parser.add_argument("--max-angular-speed", type=float, default=1.2)
    parser.add_argument("--rpm-per-mps", type=float, default=500.0)
    parser.add_argument("--steering-gain", type=float, default=180.0)
    parser.add_argument("--telemetry-period", type=float, default=0.2)
    parser.add_argument("--heartbeat-period", type=float, default=1.0)
    parser.add_argument("--verbose", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)
    try:
        bridge = Robot320FastDDSBridge(
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
            domain_id=args.domain_id,
            robot_id=args.robot_id,
            rpm_per_mps=args.rpm_per_mps,
            steering_gain_deg_per_radps=args.steering_gain,
            telemetry_period_s=args.telemetry_period,
            heartbeat_period_s=args.heartbeat_period,
            max_command_age_s=args.max_command_age,
        )
    except FastDDSUnavailable as exc:
        print(f"Fast DDS unavailable: {exc}", file=sys.stderr)
        return 2
    try:
        bridge.run()
        return 0
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
