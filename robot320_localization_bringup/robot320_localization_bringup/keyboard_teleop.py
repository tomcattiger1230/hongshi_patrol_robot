"""Keyboard teleoperation for manual Ackermann SLAM mapping."""

from __future__ import annotations

import select
import sys
import termios
import time
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

from robot320_localization_bringup.keyboard_teleop_state import (
    AckermannTeleopState,
)


HELP = """
Robot320 手动建图键盘遥控

  W / ↑     前进             S / ↓     倒车
  A / ←     左转             D / →     右转
  R         回正方向         Space      立即停车
  Q / Ctrl-C  停车并退出

按住运动键持续行驶；松开键盘超过安全超时时间后自动停车。
自行车底盘不能原地转向，请先前进或倒车，再配合 A/D 转向。
"""


class KeyboardTeleop(Node):
    """Publish deadman-protected keyboard commands on a Twist topic."""

    def __init__(self) -> None:
        super().__init__("robot320_keyboard_teleop")
        topic = str(self.declare_parameter("cmd_vel_topic", "/cmd_vel").value)
        forward_speed = float(
            self.declare_parameter("forward_speed", 0.35).value
        )
        reverse_speed = float(
            self.declare_parameter("reverse_speed", 0.20).value
        )
        min_turning_radius = float(
            self.declare_parameter("min_turning_radius", 2.35).value
        )
        self.publish_rate = float(
            self.declare_parameter("publish_rate", 20.0).value
        )
        self.command_timeout = float(
            self.declare_parameter("command_timeout", 0.8).value
        )
        if self.publish_rate <= 0.0:
            raise ValueError("publish_rate must be greater than zero")
        if min_turning_radius <= 0.0:
            raise ValueError("min_turning_radius must be greater than zero")
        if self.command_timeout <= 0.0:
            raise ValueError("command_timeout must be greater than zero")

        self.state = AckermannTeleopState(
            forward_speed=abs(forward_speed),
            reverse_speed=abs(reverse_speed),
            min_turning_radius=min_turning_radius,
        )
        self.publisher = self.create_publisher(Twist, topic, 10)
        self.get_logger().info(
            f"Keyboard teleop publishing to {topic}; "
            f"forward={self.state.forward_speed:.2f} m/s, "
            f"reverse={self.state.reverse_speed:.2f} m/s, "
            f"turn radius={self.state.min_turning_radius:.2f} m"
        )

    def publish_command(self) -> tuple[float, float]:
        linear, angular = self.state.command()
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        self.publisher.publish(message)
        return linear, angular

    def publish_stop(self) -> None:
        self.state.stop()
        for _ in range(3):
            self.publish_command()
            rclpy.spin_once(self, timeout_sec=0.01)


def _read_key() -> str:
    key = sys.stdin.read(1)
    if key != "\x1b":
        return key

    # Arrow keys arrive as a short escape sequence. Read the remaining bytes
    # without blocking normal publication or the deadman timeout.
    for _ in range(2):
        ready, _, _ = select.select([sys.stdin], [], [], 0.02)
        if not ready:
            break
        key += sys.stdin.read(1)
    return key


def _status(linear: float, angular: float, timed_out: bool = False) -> None:
    suffix = "  [松键超时，已停车]" if timed_out else ""
    print(
        f"\r速度 {linear:+.2f} m/s  角速度 {angular:+.3f} rad/s"
        f"{suffix}          ",
        end="",
        flush=True,
    )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = KeyboardTeleop()
    original_terminal = None
    try:
        if not sys.stdin.isatty():
            raise RuntimeError("keyboard_teleop must run in an interactive terminal")

        original_terminal = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        print(HELP, flush=True)

        last_motion_key = time.monotonic()
        timed_out = False
        period = 1.0 / node.publish_rate
        while rclpy.ok():
            ready, _, _ = select.select([sys.stdin], [], [], period)
            if ready:
                key = _read_key()
                if key in ("q", "Q", "\x03"):
                    break
                if node.state.apply_key(key):
                    last_motion_key = time.monotonic()
                    timed_out = False

            now = time.monotonic()
            moving = node.state.command() != (0.0, 0.0)
            if moving and now - last_motion_key > node.command_timeout:
                node.state.stop()
                timed_out = True

            linear, angular = node.publish_command()
            _status(linear, angular, timed_out)
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        if original_terminal is not None:
            termios.tcsetattr(
                sys.stdin, termios.TCSADRAIN, original_terminal
            )
        if rclpy.ok(context=node.context):
            node.publish_stop()
        print("\n机器人已停车。")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
