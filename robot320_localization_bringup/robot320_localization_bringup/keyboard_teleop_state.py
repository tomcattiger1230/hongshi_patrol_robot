"""ROS-independent keyboard state for Robot320 teleoperation."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class AckermannTeleopState:
    """Convert discrete keyboard state into a bicycle-compatible command."""

    forward_speed: float = 0.35
    reverse_speed: float = 0.20
    min_turning_radius: float = 2.35
    direction: int = 0
    steering: int = 0

    def apply_key(self, key: str) -> bool:
        normalized = {
            "\x1b[A": "w",
            "\x1b[B": "s",
            "\x1b[D": "a",
            "\x1b[C": "d",
        }.get(key, key.lower())

        if normalized == "w":
            self.direction = 1
        elif normalized == "s":
            self.direction = -1
        elif normalized == "a":
            self.steering = 1
        elif normalized == "d":
            self.steering = -1
        elif normalized == "r":
            self.steering = 0
        elif normalized == " ":
            self.stop()
        else:
            return False
        return True

    def stop(self) -> None:
        self.direction = 0
        self.steering = 0

    def command(self) -> tuple[float, float]:
        if self.direction > 0:
            linear = self.forward_speed
        elif self.direction < 0:
            linear = -self.reverse_speed
        else:
            return 0.0, 0.0

        angular = (
            self.steering * abs(linear) / self.min_turning_radius
            if self.min_turning_radius > 0.0
            else 0.0
        )
        return linear, angular
