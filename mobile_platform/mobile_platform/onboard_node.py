#!/usr/bin/env python3
"""Onboard node: receive semantic commands and control Robot320 chassis."""

from __future__ import annotations

import argparse
import logging
import sys
import time

from .ackermann import AckermannConfig, twist_to_steering
from .controlcan import CANAdapterConfig
from .messages import ChassisCommand, ChassisStatus, RobotTelemetry
from .protocol import Direction
from .robot320 import Robot320Platform
from .safety import SafetyConfig, SafetyController
from .transport import (
    CommandSubscriber,
    TelemetryPublisher,
    UdpEndpoint,
    UdpJsonCommandSubscriber,
    UdpJsonTelemetryPublisher,
)

LOGGER = logging.getLogger(__name__)


class OnboardNode:
    def __init__(
        self,
        robot: Robot320Platform,
        command_subscriber: CommandSubscriber,
        telemetry_publisher: TelemetryPublisher,
        safety: SafetyController | None = None,
        rpm_per_mps: float = 500.0,
        ackermann_config: AckermannConfig | None = None,
    ):
        self.robot = robot
        self.command_subscriber = command_subscriber
        self.telemetry_publisher = telemetry_publisher
        self.safety = safety or SafetyController()
        self.rpm_per_mps = rpm_per_mps
        self.ackermann_config = ackermann_config or AckermannConfig()
        self._last_telemetry = 0.0
        self._last_stop_due_timeout = False

    def run(self, telemetry_period_s: float = 0.2) -> None:
        self.robot.connect(start_receiver=True)
        try:
            while True:
                command = self.command_subscriber.receive_command(timeout_s=0.05)
                if command:
                    self._last_stop_due_timeout = False
                    self.apply_command(self.safety.accept(command))

                if self.safety.timed_out() and not self._last_stop_due_timeout:
                    LOGGER.warning("command timeout, stopping chassis")
                    self.robot.stop_motor()
                    self._last_stop_due_timeout = True

                now = time.monotonic()
                if now - self._last_telemetry >= telemetry_period_s:
                    self.telemetry_publisher.publish_telemetry(self.build_telemetry())
                    self._last_telemetry = now
        finally:
            self.robot.disconnect()

    def apply_command(self, command: ChassisCommand) -> None:
        if command.emergency_stop:
            self.robot.stop_motor()
            self.robot.brake()
            return

        if command.brake:
            self.robot.stop_motor()
            self.robot.brake()
            return

        self.robot.release_brake()

        steering = twist_to_steering(
            command.linear_speed_mps,
            command.angular_speed_radps,
            self.ackermann_config,
        )
        if steering.actuator_command == 0:
            self.robot.center_steering()
        else:
            turn_direction = Direction.LEFT if steering.wheel_angle_rad > 0.0 else Direction.RIGHT
            self.robot.turn(steering.actuator_command, turn_direction)

        rpm = int(abs(command.linear_speed_mps) * self.rpm_per_mps)
        if rpm <= 0:
            self.robot.stop_motor()
            return

        drive_direction = Direction.FORWARD if command.linear_speed_mps >= 0 else Direction.BACKWARD
        self.robot.set_motor_speed(drive_direction, rpm)

    def build_telemetry(self) -> RobotTelemetry:
        state = self.robot.snapshot()
        return RobotTelemetry(
            chassis=ChassisStatus(
                connected=state.connected,
                enabled=state.enabled,
                brake_engaged=state.brake_engaged,
                commanded_rpm=state.commanded_rpm,
                commanded_direction=state.commanded_direction.value,
                steering_angle_deg=state.steering_angle_deg,
                steering_direction=state.steering_direction.value,
                speed_kmh=state.speed_kmh,
                emergency_stopped=self.safety.emergency_stopped,
            )
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Robot320 onboard control node")
    parser.add_argument("--lib", default=None)
    parser.add_argument("--device-index", type=int, default=0)
    parser.add_argument("--can-index", type=int, default=0)
    parser.add_argument("--command-bind", default="0.0.0.0:15000")
    parser.add_argument("--telemetry-remote", default="127.0.0.1:15001")
    parser.add_argument("--command-timeout", type=float, default=0.6)
    parser.add_argument("--max-linear-speed", type=float, default=0.8)
    parser.add_argument("--max-angular-speed", type=float, default=1.2)
    parser.add_argument("--rpm-per-mps", type=float, default=500.0)
    parser.add_argument("--wheelbase", type=float, default=0.700)
    parser.add_argument("--min-turning-radius", type=float, default=2.350)
    parser.add_argument("--max-wheel-angle", type=float, default=16.59)
    parser.add_argument("--max-steering-command", type=int, default=350)
    parser.add_argument("--min-steering-speed", type=float, default=0.05)
    parser.add_argument("--verbose", action="store_true")
    return parser


def parse_endpoint(value: str) -> UdpEndpoint:
    host, port = value.rsplit(":", 1)
    return UdpEndpoint(host, int(port))


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO, format="%(message)s")

    robot = Robot320Platform(
        config=CANAdapterConfig(
            device_index=args.device_index,
            can_index=args.can_index,
            library_path=args.lib,
        )
    )
    node = OnboardNode(
        robot=robot,
        command_subscriber=UdpJsonCommandSubscriber(parse_endpoint(args.command_bind)),
        telemetry_publisher=UdpJsonTelemetryPublisher(parse_endpoint(args.telemetry_remote)),
        safety=SafetyController(
            SafetyConfig(
                command_timeout_s=args.command_timeout,
                max_linear_speed_mps=args.max_linear_speed,
                max_angular_speed_radps=args.max_angular_speed,
            )
        ),
        rpm_per_mps=args.rpm_per_mps,
        ackermann_config=AckermannConfig(
            wheelbase_m=args.wheelbase,
            min_turning_radius_m=args.min_turning_radius,
            max_wheel_angle_deg=args.max_wheel_angle,
            max_steering_command=args.max_steering_command,
            min_linear_speed_mps=args.min_steering_speed,
        ),
    )

    try:
        node.run()
        return 0
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
