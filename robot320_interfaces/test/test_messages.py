import math

from robot320_interfaces.messages import (
    BatteryStatus,
    LiftStatus,
    Pose2D,
    RobotCommand,
    RobotTelemetry,
    robot_command_from_json,
    telemetry_from_json,
    to_json,
)


def test_robot_command_round_trip_and_chassis_conversion():
    original = RobotCommand(
        kind="manual_motion",
        sequence=42,
        linear_speed_mps=0.3,
        angular_speed_radps=-0.4,
    )
    restored = robot_command_from_json(to_json(original))
    chassis = restored.to_chassis_command()

    assert restored.command_id == original.command_id
    assert restored.sequence == 42
    assert chassis.linear_speed_mps == 0.3
    assert chassis.angular_speed_radps == -0.4
    assert chassis.mode == "manual"


def test_navigation_goal_round_trip():
    command = RobotCommand(
        kind="navigation_goal",
        goal=Pose2D(x_m=1.2, y_m=-3.4, yaw_rad=math.pi / 2),
    )
    restored = robot_command_from_json(to_json(command))
    assert restored.goal is not None
    assert restored.goal.x_m == 1.2
    assert restored.goal.yaw_rad == math.pi / 2


def test_extended_telemetry_round_trip():
    telemetry = RobotTelemetry(
        lift=LiftStatus(available=True, height_m=1.1, moving=True),
        battery=BatteryStatus(percentage=75.0, voltage_v=48.2),
        pose=Pose2D(x_m=2.0, y_m=4.0),
        faults=["example_fault"],
    )
    restored = telemetry_from_json(to_json(telemetry))
    assert restored.lift.available is True
    assert restored.lift.height_m == 1.1
    assert restored.battery.percentage == 75.0
    assert restored.pose is not None and restored.pose.x_m == 2.0
    assert restored.faults == ["example_fault"]
