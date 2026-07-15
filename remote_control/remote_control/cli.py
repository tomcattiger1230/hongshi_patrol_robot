#!/usr/bin/env python3
"""Remote CLI for sending semantic chassis commands."""

from __future__ import annotations

import argparse
import sys
import time

from mobile_platform.transport import UdpEndpoint

from .dds_client import RobotRemoteClient


def parse_endpoint(value: str) -> UdpEndpoint:
    host, port = value.rsplit(":", 1)
    return UdpEndpoint(host, int(port))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Remote Robot320 control client")
    parser.add_argument("--robot", default="127.0.0.1:15000", help="robot command endpoint")
    parser.add_argument("--telemetry-bind", default="0.0.0.0:15001", help="local telemetry bind endpoint")

    sub = parser.add_subparsers(dest="command", required=True)
    move = sub.add_parser("move")
    move.add_argument("--linear", type=float, default=0.0)
    move.add_argument("--angular", type=float, default=0.0)
    move.add_argument("--duration", type=float, default=0.0)

    sub.add_parser("stop")
    sub.add_parser("brake")
    sub.add_parser("estop")
    sub.add_parser("reset")

    watch = sub.add_parser("watch")
    watch.add_argument("--seconds", type=float, default=30.0)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    client = RobotRemoteClient(parse_endpoint(args.robot), parse_endpoint(args.telemetry_bind))
    try:
        if args.command == "move":
            client.send_manual_command(args.linear, args.angular)
            if args.duration > 0:
                time.sleep(args.duration)
                client.stop()
        elif args.command == "stop":
            client.stop()
        elif args.command == "brake":
            client.brake()
        elif args.command == "estop":
            client.emergency_stop()
        elif args.command == "reset":
            client.reset_idle()
        elif args.command == "watch":
            deadline = time.monotonic() + args.seconds
            while time.monotonic() < deadline:
                telemetry = client.receive_telemetry(timeout_s=0.5)
                if telemetry:
                    chassis = telemetry.chassis
                    print(
                        f"connected={chassis.connected} enabled={chassis.enabled} "
                        f"speed={chassis.speed_kmh} rpm={chassis.commanded_rpm} "
                        f"steering={chassis.steering_direction}:{chassis.steering_angle_deg}"
                    )
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        client.close()


if __name__ == "__main__":
    sys.exit(main())
