#!/usr/bin/env python3
"""Command-line tool for Robot320 chassis CAN control."""

from __future__ import annotations

import argparse
import logging
import sys
import time

from .controlcan import CANAdapterConfig
from .protocol import Direction
from .robot320 import Robot320Platform


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Robot320 mobile platform CAN control")
    parser.add_argument("--lib", default=None, help="path to libcontrolcan.so")
    parser.add_argument("--device-index", type=int, default=0)
    parser.add_argument("--can-index", type=int, default=0)
    parser.add_argument("--verbose", action="store_true")

    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("enable")
    sub.add_parser("disable")
    sub.add_parser("stop")
    sub.add_parser("release-brake")
    sub.add_parser("center")

    brake = sub.add_parser("brake")
    brake.add_argument("--pressure", type=float, default=5.0, help="brake pressure in MPa")

    speed = sub.add_parser("speed")
    speed.add_argument("direction", choices=[Direction.FORWARD.value, Direction.BACKWARD.value])
    speed.add_argument("rpm", type=int)
    speed.add_argument("--duration", type=float, default=0.0, help="seconds to keep command before stopping")

    turn = sub.add_parser("turn")
    turn.add_argument("direction", choices=[Direction.LEFT.value, Direction.RIGHT.value])
    turn.add_argument("angle", type=int)

    watch = sub.add_parser("watch")
    watch.add_argument("--seconds", type=float, default=30.0)

    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO, format="%(message)s")

    config = CANAdapterConfig(
        device_index=args.device_index,
        can_index=args.can_index,
        library_path=args.lib,
    )
    platform = Robot320Platform(config=config)

    try:
        board = platform.connect(start_receiver=args.command == "watch")
        if board:
            print(f"connected: {board.hardware_type} {board.serial_number} {board.firmware_version}")
        else:
            print("connected")

        if args.command == "enable":
            platform.enable_motor()
        elif args.command == "disable":
            platform.disable_motor()
        elif args.command == "stop":
            platform.stop_motor()
        elif args.command == "brake":
            platform.brake(args.pressure)
        elif args.command == "release-brake":
            platform.release_brake()
        elif args.command == "speed":
            platform.set_motor_speed(Direction(args.direction), args.rpm)
            if args.duration > 0:
                time.sleep(args.duration)
                platform.stop_motor()
        elif args.command == "turn":
            platform.turn(args.angle, Direction(args.direction))
        elif args.command == "center":
            platform.center_steering()
        elif args.command == "watch":
            platform.on_speed = lambda feedback: print(f"speed: {feedback.speed_kmh:.2f} km/h")
            deadline = time.monotonic() + args.seconds
            while time.monotonic() < deadline:
                time.sleep(0.2)

        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        platform.disconnect(safe_stop=args.command in {"speed", "watch"})


if __name__ == "__main__":
    sys.exit(main())
