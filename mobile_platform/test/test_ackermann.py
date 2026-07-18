import math

import pytest

from mobile_platform.ackermann import AckermannConfig, twist_to_steering


CONFIG = AckermannConfig()


def test_straight_motion_centers_steering():
    command = twist_to_steering(0.5, 0.0, CONFIG)

    assert command.wheel_angle_rad == 0.0
    assert command.actuator_command == 0


def test_minimum_radius_maps_to_maximum_actuator_command():
    command = twist_to_steering(0.5, 0.5 / 2.35, CONFIG)

    assert math.degrees(command.wheel_angle_rad) == pytest.approx(16.59, abs=0.02)
    assert command.actuator_command == 350


def test_tighter_requested_turn_is_clamped():
    command = twist_to_steering(0.5, 1.2, CONFIG)

    assert command.actuator_command == 350


def test_reverse_motion_inverts_steering_for_same_positive_yaw_rate():
    command = twist_to_steering(-0.5, 0.5 / 2.35, CONFIG)

    assert command.wheel_angle_rad < 0.0
    assert command.actuator_command == 350


def test_stationary_rotation_request_is_rejected():
    command = twist_to_steering(0.0, 1.0, CONFIG)

    assert command.wheel_angle_rad == 0.0
    assert command.actuator_command == 0


def test_invalid_geometry_is_rejected():
    with pytest.raises(ValueError):
        AckermannConfig(wheelbase_m=0.0)
