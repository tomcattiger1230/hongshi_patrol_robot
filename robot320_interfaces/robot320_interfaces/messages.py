"""ROS-independent application messages shared over Fast DDS or debug UDP."""

from __future__ import annotations

import json
import time
import uuid
from dataclasses import asdict, dataclass, field
from typing import Any, Literal, Optional


ControlMode = Literal["idle", "manual", "navigation"]
CommandKind = Literal[
    "manual_motion",
    "navigation_goal",
    "stop",
    "brake",
    "emergency_stop",
    "reset_emergency_stop",
    "set_mode",
    "lift",
]
LiftAction = Literal["stop", "raise", "lower", "move_to"]
ReplyStatus = Literal["accepted", "completed", "rejected", "failed"]


@dataclass
class ChassisCommand:
    """Low-level semantic chassis command consumed by the NUC safety gate."""

    linear_speed_mps: float = 0.0
    angular_speed_radps: float = 0.0
    brake: bool = False
    emergency_stop: bool = False
    mode: ControlMode = "manual"
    stamp: float = field(default_factory=time.time)


@dataclass
class Pose2D:
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0
    frame_id: str = "map"
    stamp: float = field(default_factory=time.time)


@dataclass
class NavigationStatus:
    state: str = "idle"
    goal_id: Optional[str] = None
    target: Optional[Pose2D] = None
    progress: float = 0.0
    message: str = ""
    stamp: float = field(default_factory=time.time)


@dataclass
class ChassisStatus:
    connected: bool = False
    enabled: bool = False
    brake_engaged: bool = False
    commanded_rpm: int = 0
    commanded_direction: str = "center"
    steering_angle_deg: int = 0
    steering_direction: str = "center"
    speed_kmh: Optional[float] = None
    emergency_stopped: bool = False
    fault_code: Optional[str] = None
    stamp: float = field(default_factory=time.time)


@dataclass
class LiftStatus:
    available: bool = False
    height_m: Optional[float] = None
    target_height_m: Optional[float] = None
    moving: bool = False
    upper_limit: bool = False
    lower_limit: bool = False
    fault_code: Optional[str] = None
    stamp: float = field(default_factory=time.time)


@dataclass
class BatteryStatus:
    percentage: Optional[float] = None
    voltage_v: Optional[float] = None
    charging: bool = False
    stamp: float = field(default_factory=time.time)


@dataclass
class RobotTelemetry:
    robot_id: str = "robot320"
    online: bool = True
    chassis: ChassisStatus = field(default_factory=ChassisStatus)
    lift: LiftStatus = field(default_factory=LiftStatus)
    battery: BatteryStatus = field(default_factory=BatteryStatus)
    pose: Optional[Pose2D] = None
    navigation: NavigationStatus = field(default_factory=NavigationStatus)
    faults: list[str] = field(default_factory=list)
    map_revision: Optional[str] = None
    stamp: float = field(default_factory=time.time)


@dataclass
class RobotCommand:
    """High-level command sent by a ROS-independent remote controller."""

    kind: CommandKind
    command_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    client_id: str = "remote_control"
    sequence: int = 0
    stamp: float = field(default_factory=time.time)
    linear_speed_mps: float = 0.0
    angular_speed_radps: float = 0.0
    goal: Optional[Pose2D] = None
    mode: Optional[ControlMode] = None
    lift_action: Optional[LiftAction] = None
    lift_target_height_m: Optional[float] = None

    def to_chassis_command(self) -> ChassisCommand:
        if self.kind == "manual_motion":
            return ChassisCommand(
                linear_speed_mps=self.linear_speed_mps,
                angular_speed_radps=self.angular_speed_radps,
                mode="manual",
                stamp=self.stamp,
            )
        if self.kind == "brake":
            return ChassisCommand(brake=True, mode="manual", stamp=self.stamp)
        if self.kind == "emergency_stop":
            return ChassisCommand(emergency_stop=True, mode="manual", stamp=self.stamp)
        if self.kind == "reset_emergency_stop":
            return ChassisCommand(mode="idle", stamp=self.stamp)
        if self.kind == "set_mode":
            return ChassisCommand(mode=self.mode or "idle", stamp=self.stamp)
        if self.kind == "stop":
            return ChassisCommand(mode="manual", stamp=self.stamp)
        raise ValueError(f"{self.kind} is not a chassis command")


@dataclass
class CommandReply:
    command_id: str
    status: ReplyStatus
    robot_id: str = "robot320"
    sequence: int = 0
    message: str = ""
    stamp: float = field(default_factory=time.time)


def to_json(message: Any) -> str:
    return json.dumps(asdict(message), ensure_ascii=False, separators=(",", ":"))


def command_from_json(payload: str | bytes) -> ChassisCommand:
    return ChassisCommand(**_json_object(payload))


def robot_command_from_json(payload: str | bytes) -> RobotCommand:
    data = _json_object(payload)
    goal_data = data.get("goal")
    data["goal"] = Pose2D(**goal_data) if goal_data else None
    return RobotCommand(**data)


def reply_from_json(payload: str | bytes) -> CommandReply:
    return CommandReply(**_json_object(payload))


def telemetry_from_json(payload: str | bytes) -> RobotTelemetry:
    data = _json_object(payload)
    chassis = ChassisStatus(**data.get("chassis", {}))
    lift = LiftStatus(**data.get("lift", {}))
    battery = BatteryStatus(**data.get("battery", {}))
    pose_data = data.get("pose")
    pose = Pose2D(**pose_data) if pose_data else None
    nav_data = dict(data.get("navigation", {}))
    target_data = nav_data.get("target")
    if target_data:
        nav_data["target"] = Pose2D(**target_data)
    navigation = NavigationStatus(**nav_data)
    return RobotTelemetry(
        robot_id=data.get("robot_id", "robot320"),
        online=data.get("online", True),
        chassis=chassis,
        lift=lift,
        battery=battery,
        pose=pose,
        navigation=navigation,
        faults=list(data.get("faults", [])),
        map_revision=data.get("map_revision"),
        stamp=data.get("stamp", time.time()),
    )


def _json_object(payload: str | bytes) -> dict[str, Any]:
    if isinstance(payload, bytes):
        payload = payload.decode("utf-8")
    data = json.loads(payload)
    if not isinstance(data, dict):
        raise ValueError("message payload must be a JSON object")
    return data
