"""Ackermann steering conversion for Robot320 velocity commands."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class AckermannConfig:
    """Geometry and actuator calibration for the single steering command."""

    wheelbase_m: float = 0.700
    min_turning_radius_m: float = 2.350
    max_wheel_angle_deg: float = 16.59
    max_steering_command: int = 350
    min_linear_speed_mps: float = 0.05

    def __post_init__(self) -> None:
        if self.wheelbase_m <= 0.0:
            raise ValueError("wheelbase_m must be positive")
        if self.min_turning_radius_m <= 0.0:
            raise ValueError("min_turning_radius_m must be positive")
        if not 0.0 < self.max_wheel_angle_deg < 90.0:
            raise ValueError("max_wheel_angle_deg must be between 0 and 90")
        if not 0 < self.max_steering_command <= 350:
            raise ValueError("max_steering_command must be between 1 and 350")
        if self.min_linear_speed_mps < 0.0:
            raise ValueError("min_linear_speed_mps must not be negative")


@dataclass(frozen=True)
class SteeringCommand:
    """Signed equivalent wheel angle and unsigned CAN actuator command."""

    wheel_angle_rad: float
    actuator_command: int


def twist_to_steering(
    linear_speed_mps: float,
    angular_speed_radps: float,
    config: AckermannConfig,
) -> SteeringCommand:
    """Convert a ROS ``Twist`` pair to a single Ackermann steering command.

    Positive wheel angle turns left. For reverse motion, the steering sign is
    naturally inverted by ``curvature = angular / linear`` so the requested
    yaw-rate semantics remain correct.
    """
    if abs(linear_speed_mps) < config.min_linear_speed_mps:
        return SteeringCommand(wheel_angle_rad=0.0, actuator_command=0)

    requested_curvature = angular_speed_radps / linear_speed_mps
    radius_curvature_limit = 1.0 / config.min_turning_radius_m
    angle_curvature_limit = math.tan(math.radians(config.max_wheel_angle_deg)) / config.wheelbase_m
    curvature_limit = min(radius_curvature_limit, angle_curvature_limit)
    curvature = max(-curvature_limit, min(curvature_limit, requested_curvature))

    wheel_angle_rad = math.atan(config.wheelbase_m * curvature)
    command = round(
        abs(math.degrees(wheel_angle_rad))
        / config.max_wheel_angle_deg
        * config.max_steering_command
    )
    return SteeringCommand(
        wheel_angle_rad=wheel_angle_rad,
        actuator_command=min(config.max_steering_command, command),
    )
