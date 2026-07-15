"""Shared semantic messages between remote UI and onboard controller."""

from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass, field
from typing import Any, Literal, Optional


ControlMode = Literal["idle", "manual", "navigation"]


@dataclass
class ChassisCommand:
    """Semantic chassis command.

    Remote clients should send this message instead of CAN frames. The onboard
    node is responsible for converting it into Robot320 CAN commands.
    """

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
    stamp: float = field(default_factory=time.time)


@dataclass
class RobotTelemetry:
    chassis: ChassisStatus = field(default_factory=ChassisStatus)
    pose: Optional[Pose2D] = None
    navigation: NavigationStatus = field(default_factory=NavigationStatus)
    map_revision: Optional[str] = None
    stamp: float = field(default_factory=time.time)


def to_json(message: Any) -> str:
    return json.dumps(asdict(message), ensure_ascii=False, separators=(",", ":"))


def command_from_json(payload: str | bytes) -> ChassisCommand:
    if isinstance(payload, bytes):
        payload = payload.decode("utf-8")
    data = json.loads(payload)
    return ChassisCommand(**data)


def telemetry_from_json(payload: str | bytes) -> RobotTelemetry:
    if isinstance(payload, bytes):
        payload = payload.decode("utf-8")
    data = json.loads(payload)
    chassis = ChassisStatus(**data.get("chassis", {}))
    pose_data = data.get("pose")
    pose = Pose2D(**pose_data) if pose_data else None
    nav_data = data.get("navigation", {})
    target_data = nav_data.get("target")
    if target_data:
        nav_data["target"] = Pose2D(**target_data)
    navigation = NavigationStatus(**nav_data)
    return RobotTelemetry(
        chassis=chassis,
        pose=pose,
        navigation=navigation,
        map_revision=data.get("map_revision"),
        stamp=data.get("stamp", time.time()),
    )
