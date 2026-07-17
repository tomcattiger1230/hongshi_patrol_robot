from robot320_interfaces.messages import (
    BatteryStatus,
    ChassisStatus,
    LiftStatus,
    NavigationStatus,
    Pose2D,
    RobotTelemetry,
)
from remote_control.gui_model import telemetry_view


def test_telemetry_view_formats_complete_robot_state():
    telemetry = RobotTelemetry(
        robot_id="robot-test",
        online=True,
        chassis=ChassisStatus(
            connected=True,
            enabled=True,
            speed_kmh=1.25,
            brake_engaged=True,
        ),
        pose=Pose2D(x_m=1.2, y_m=-0.5, yaw_rad=1.5707963),
        navigation=NavigationStatus(
            state="executing", progress=0.426, message="distance remaining: 2 m"
        ),
        lift=LiftStatus(available=True, height_m=1.4, moving=True),
        battery=BatteryStatus(percentage=72.0, voltage_v=48.2),
        faults=["obstacle"],
    )

    view = telemetry_view(telemetry)

    assert view.robot_id == "robot-test"
    assert view.online is True
    assert "CAN 已连接" in view.chassis
    assert view.speed == "1.25 km/h"
    assert "yaw=90.0°" in view.pose
    assert view.navigation_progress == 43
    assert "1.40 m" in view.lift
    assert view.battery == "72% · 48.2 V"
    assert view.faults == "obstacle"


def test_telemetry_view_handles_missing_optional_state():
    view = telemetry_view(RobotTelemetry())

    assert view.speed == "--"
    assert view.pose == "不可用"
    assert view.lift == "不可用"
    assert view.battery == "不可用"
    assert view.faults == "无"
