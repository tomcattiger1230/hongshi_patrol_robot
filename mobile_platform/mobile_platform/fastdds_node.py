#!/usr/bin/env python3
"""FastDDS onboard bridge skeleton for Robot320 CAN control.

Use this entry point on the robot computer when commands arrive through native
FastDDS instead of ROS 2. The concrete FastDDS Python bindings should be
generated from ``docs/robot320_fastdds.idl`` and wired into this module.
"""

from __future__ import annotations

import argparse
import sys

from .controlcan import CANAdapterConfig
from .robot320 import Robot320Platform
from .safety import SafetyConfig, SafetyController


class FastDDSUnavailable(RuntimeError):
    pass


class Robot320FastDDSBridge:
    def __init__(
        self,
        robot: Robot320Platform,
        safety: SafetyController,
        domain_id: int = 0,
        rpm_per_mps: float = 500.0,
        steering_gain_deg_per_radps: float = 180.0,
    ):
        self.robot = robot
        self.safety = safety
        self.domain_id = domain_id
        self.rpm_per_mps = rpm_per_mps
        self.steering_gain_deg_per_radps = steering_gain_deg_per_radps
        raise FastDDSUnavailable(
            "FastDDS onboard bridge is not wired yet. Generate Python bindings from "
            "docs/robot320_fastdds.idl, then connect command subscription to "
            "mobile_platform.onboard_node.OnboardNode.apply_command and telemetry publication "
            "to OnboardNode.build_telemetry."
        )

    def run(self) -> None:
        raise FastDDSUnavailable("FastDDS onboard bridge is not configured yet")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Robot320 FastDDS CAN bridge")
    parser.add_argument("--lib", default=None)
    parser.add_argument("--device-index", type=int, default=0)
    parser.add_argument("--can-index", type=int, default=0)
    parser.add_argument("--domain-id", type=int, default=0)
    parser.add_argument("--command-timeout", type=float, default=0.6)
    parser.add_argument("--max-linear-speed", type=float, default=0.8)
    parser.add_argument("--max-angular-speed", type=float, default=1.2)
    parser.add_argument("--rpm-per-mps", type=float, default=500.0)
    parser.add_argument("--steering-gain", type=float, default=180.0)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
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
        rpm_per_mps=args.rpm_per_mps,
        steering_gain_deg_per_radps=args.steering_gain,
    )
    bridge.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
