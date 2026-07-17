"""Qt-independent presentation helpers for the Fast DDS control panel."""

from __future__ import annotations

import math
from dataclasses import dataclass

from robot320_interfaces.messages import RobotTelemetry


@dataclass(frozen=True)
class TelemetryView:
    robot_id: str
    online: bool
    chassis: str
    speed: str
    pose: str
    navigation: str
    navigation_progress: int
    lift: str
    battery: str
    faults: str


def telemetry_view(telemetry: RobotTelemetry) -> TelemetryView:
    chassis = telemetry.chassis
    pose = telemetry.pose
    navigation = telemetry.navigation
    lift = telemetry.lift
    battery = telemetry.battery

    chassis_parts = ["CAN 已连接" if chassis.connected else "CAN 未连接"]
    chassis_parts.append("已使能" if chassis.enabled else "未使能")
    if chassis.brake_engaged:
        chassis_parts.append("刹车中")
    if chassis.emergency_stopped:
        chassis_parts.append("急停中")
    if chassis.fault_code:
        chassis_parts.append(f"故障 {chassis.fault_code}")

    if pose is None:
        pose_text = "不可用"
    else:
        pose_text = (
            f"x={pose.x_m:.2f} m  y={pose.y_m:.2f} m  "
            f"yaw={math.degrees(pose.yaw_rad):.1f}°  [{pose.frame_id}]"
        )

    nav_text = navigation.state
    if navigation.message:
        nav_text += f" · {navigation.message}"

    if not lift.available:
        lift_text = "不可用"
    else:
        height = "--" if lift.height_m is None else f"{lift.height_m:.2f} m"
        lift_text = f"高度 {height}"
        if lift.moving:
            lift_text += " · 运动中"
        if lift.fault_code:
            lift_text += f" · 故障 {lift.fault_code}"

    if battery.percentage is None and battery.voltage_v is None:
        battery_text = "不可用"
    else:
        parts = []
        if battery.percentage is not None:
            parts.append(f"{battery.percentage:.0f}%")
        if battery.voltage_v is not None:
            parts.append(f"{battery.voltage_v:.1f} V")
        if battery.charging:
            parts.append("充电中")
        battery_text = " · ".join(parts)

    speed = "--" if chassis.speed_kmh is None else f"{chassis.speed_kmh:.2f} km/h"
    return TelemetryView(
        robot_id=telemetry.robot_id,
        online=telemetry.online,
        chassis=" · ".join(chassis_parts),
        speed=speed,
        pose=pose_text,
        navigation=nav_text,
        navigation_progress=round(max(0.0, min(1.0, navigation.progress)) * 100),
        lift=lift_text,
        battery=battery_text,
        faults="无" if not telemetry.faults else "；".join(telemetry.faults),
    )
