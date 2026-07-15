"""FastDDS remote client adapter skeleton.

FastDDS Python bindings and generated IDL code vary by installation. This file
keeps the upper-computer API stable while the concrete FastDDS participant,
publisher, and subscriber are wired in later.
"""

from __future__ import annotations

import argparse
import sys
from typing import Optional

from mobile_platform.messages import ChassisCommand, RobotTelemetry


class FastDDSUnavailable(RuntimeError):
    pass


class RobotRemoteFastDDSClient:
    """Upper-computer client for macOS/Windows FastDDS deployments.

    The expected topic contract mirrors the ROS 2 semantic messages:

    - ``Robot320ChassisCommand`` for command publication
    - ``Robot320Telemetry`` for telemetry subscription

    Until generated FastDDS Python bindings are added, methods raise a clear
    error instead of silently falling back to an incompatible transport.
    """

    def __init__(self, domain_id: int = 0):
        self.domain_id = domain_id
        self.latest_telemetry: Optional[RobotTelemetry] = None
        raise FastDDSUnavailable(
            "FastDDS client bindings are not configured yet. Generate Python bindings from "
            "docs/robot320_fastdds.idl and wire them into remote_control.fastdds_client."
        )

    def publish_command(self, command: ChassisCommand) -> None:
        raise FastDDSUnavailable("FastDDS client bindings are not configured yet")

    def receive_telemetry(self, timeout_s: float = 0.1) -> Optional[RobotTelemetry]:
        raise FastDDSUnavailable("FastDDS client bindings are not configured yet")

    def close(self) -> None:
        return None


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Robot320 FastDDS remote client")
    parser.add_argument("--domain-id", type=int, default=0)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    RobotRemoteFastDDSClient(domain_id=args.domain_id)
    return 0


if __name__ == "__main__":
    sys.exit(main())
