"""In-process Robot320 simulator for developing the GUI without a robot."""

from __future__ import annotations

import copy
import math
import queue
import threading
import time

from robot320_interfaces.messages import (
    BatteryStatus,
    ChassisStatus,
    CommandReply,
    LiftStatus,
    NavigationStatus,
    Pose2D,
    RobotCommand,
    RobotTelemetry,
)


class DemoRemoteTransport:
    """Small deterministic transport implementing the real client's interface.

    Nothing is published to DDS or ROS 2. Commands only update an in-memory
    robot model, making this backend safe for UI development and demonstrations.
    """

    backend = "demo"

    def __init__(self, robot_id: str = "robot320-demo"):
        self.robot_id = robot_id
        self._lock = threading.Lock()
        self._replies: queue.Queue[CommandReply] = queue.Queue()
        self._closed = False
        self._last_update = time.monotonic()
        self._linear_speed = 0.0
        self._angular_speed = 0.0
        self._pose = Pose2D()
        self._chassis = ChassisStatus(connected=True, enabled=False, speed_kmh=0.0)
        self._navigation = NavigationStatus(message="本地演示后端")
        self._navigation_started_at: float | None = None
        self._navigation_duration = 0.0
        self._navigation_start_pose: Pose2D | None = None
        self._lift = LiftStatus(available=True, height_m=0.0)
        self._battery = BatteryStatus(percentage=96.0, voltage_v=51.2)

    def publish_command(self, command: RobotCommand) -> None:
        with self._lock:
            self._ensure_open()
            self._advance()
            status, message = self._dispatch(command)
            self._replies.put(
                CommandReply(
                    command_id=command.command_id,
                    status=status,
                    robot_id=self.robot_id,
                    sequence=command.sequence,
                    message=message,
                )
            )

    def publish_heartbeat(self, sequence: int) -> None:
        del sequence
        with self._lock:
            self._ensure_open()

    def receive_state(self, timeout_s: float = 0.0) -> RobotTelemetry | None:
        del timeout_s
        with self._lock:
            if self._closed:
                return None
            self._advance()
            return RobotTelemetry(
                robot_id=self.robot_id,
                online=True,
                chassis=copy.deepcopy(self._chassis),
                lift=copy.deepcopy(self._lift),
                battery=copy.deepcopy(self._battery),
                pose=copy.deepcopy(self._pose),
                navigation=copy.deepcopy(self._navigation),
                map_revision="demo-map-v1",
            )

    def receive_reply(self, timeout_s: float = 0.0) -> CommandReply | None:
        try:
            return self._replies.get(timeout=max(0.0, timeout_s))
        except queue.Empty:
            return None

    def close(self) -> None:
        with self._lock:
            self._closed = True
            self._linear_speed = 0.0
            self._angular_speed = 0.0

    def _dispatch(self, command: RobotCommand) -> tuple[str, str]:
        if command.kind == "manual_motion":
            if self._chassis.emergency_stopped:
                return "rejected", "演示急停仍处于保持状态"
            self._cancel_navigation("已被手动控制接管")
            self._linear_speed = float(command.linear_speed_mps)
            self._angular_speed = float(command.angular_speed_radps)
            self._chassis.enabled = True
            self._chassis.brake_engaged = False
            return "accepted", "本地演示运动指令已接收"
        if command.kind == "stop":
            self._stop_motion()
            self._cancel_navigation("已停止")
            return "accepted", "本地演示已停止"
        if command.kind == "brake":
            self._stop_motion()
            self._cancel_navigation("已刹车")
            self._chassis.brake_engaged = True
            return "accepted", "本地演示刹车已应用"
        if command.kind == "emergency_stop":
            self._stop_motion()
            self._cancel_navigation("急停取消导航")
            self._chassis.emergency_stopped = True
            self._chassis.brake_engaged = True
            self._chassis.enabled = False
            return "accepted", "本地演示急停已保持"
        if command.kind == "reset_emergency_stop":
            self._chassis.emergency_stopped = False
            self._chassis.brake_engaged = False
            self._chassis.enabled = False
            return "accepted", "本地演示急停已解除"
        if command.kind == "set_mode":
            return "accepted", f"本地演示模式已设为 {command.mode or 'idle'}"
        if command.kind == "navigation_goal":
            return self._start_navigation(command)
        if command.kind == "cancel_navigation":
            if self._navigation_started_at is None:
                return "rejected", "本地演示中没有活动导航任务"
            self._cancel_navigation("用户取消")
            self._stop_motion()
            return "accepted", "本地演示导航已取消"
        if command.kind == "lift":
            return self._control_lift(command)
        return "rejected", f"本地演示不支持指令 {command.kind}"

    def _start_navigation(self, command: RobotCommand) -> tuple[str, str]:
        if self._chassis.emergency_stopped:
            return "rejected", "演示急停仍处于保持状态"
        if command.goal is None:
            return "rejected", "导航目标为空"
        distance = math.hypot(
            command.goal.x_m - self._pose.x_m,
            command.goal.y_m - self._pose.y_m,
        )
        self._navigation_start_pose = copy.deepcopy(self._pose)
        self._navigation_started_at = time.monotonic()
        self._navigation_duration = max(2.0, distance / 0.6)
        self._navigation = NavigationStatus(
            state="executing",
            goal_id=command.command_id,
            target=copy.deepcopy(command.goal),
            progress=0.0,
            message=f"本地演示，剩余 {distance:.2f} m",
        )
        self._chassis.enabled = True
        self._chassis.brake_engaged = False
        return "accepted", "本地演示导航目标已接受"

    def _control_lift(self, command: RobotCommand) -> tuple[str, str]:
        action = command.lift_action
        if action == "raise":
            self._lift.height_m = min(2.0, (self._lift.height_m or 0.0) + 0.1)
        elif action == "lower":
            self._lift.height_m = max(0.0, (self._lift.height_m or 0.0) - 0.1)
        elif action == "move_to" and command.lift_target_height_m is not None:
            self._lift.height_m = max(0.0, min(2.0, command.lift_target_height_m))
            self._lift.target_height_m = self._lift.height_m
        elif action != "stop":
            return "rejected", "升降杆演示指令无效"
        self._lift.moving = False
        return "accepted", "本地演示升降杆状态已更新"

    def _advance(self) -> None:
        now = time.monotonic()
        dt = max(0.0, min(0.25, now - self._last_update))
        self._last_update = now

        if self._navigation_started_at is not None:
            self._advance_navigation(now)
        else:
            self._pose.x_m += self._linear_speed * math.cos(self._pose.yaw_rad) * dt
            self._pose.y_m += self._linear_speed * math.sin(self._pose.yaw_rad) * dt
            if abs(self._linear_speed) > 1e-6:
                self._pose.yaw_rad = _normalize_angle(
                    self._pose.yaw_rad + self._angular_speed * dt
                )
        self._chassis.speed_kmh = abs(self._linear_speed) * 3.6
        self._chassis.commanded_rpm = round(self._linear_speed * 500.0)

    def _advance_navigation(self, now: float) -> None:
        assert self._navigation_started_at is not None
        assert self._navigation_start_pose is not None
        assert self._navigation.target is not None
        elapsed = now - self._navigation_started_at
        progress = min(1.0, elapsed / self._navigation_duration)
        start = self._navigation_start_pose
        target = self._navigation.target
        self._pose.x_m = start.x_m + (target.x_m - start.x_m) * progress
        self._pose.y_m = start.y_m + (target.y_m - start.y_m) * progress
        yaw_delta = _normalize_angle(target.yaw_rad - start.yaw_rad)
        self._pose.yaw_rad = _normalize_angle(start.yaw_rad + yaw_delta * progress)
        remaining = math.hypot(target.x_m - self._pose.x_m, target.y_m - self._pose.y_m)
        self._navigation.progress = progress
        self._navigation.message = f"本地演示，剩余 {remaining:.2f} m"
        self._linear_speed = 0.0 if progress >= 1.0 else min(0.6, remaining)
        self._angular_speed = 0.0
        if progress >= 1.0:
            self._navigation.state = "succeeded"
            self._navigation.message = "本地演示导航完成"
            self._navigation_started_at = None
            self._navigation_start_pose = None
            self._stop_motion()

    def _cancel_navigation(self, message: str) -> None:
        if self._navigation_started_at is None:
            return
        self._navigation.state = "canceled"
        self._navigation.progress = 0.0
        self._navigation.message = message
        self._navigation_started_at = None
        self._navigation_start_pose = None

    def _stop_motion(self) -> None:
        self._linear_speed = 0.0
        self._angular_speed = 0.0
        self._chassis.commanded_rpm = 0
        self._chassis.speed_kmh = 0.0

    def _ensure_open(self) -> None:
        if self._closed:
            raise RuntimeError("demo transport is closed")


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))

