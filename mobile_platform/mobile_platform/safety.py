"""Safety gate for remote chassis commands."""

from __future__ import annotations

import time
from dataclasses import dataclass

from .messages import ChassisCommand


@dataclass(frozen=True)
class SafetyConfig:
    command_timeout_s: float = 0.6
    max_linear_speed_mps: float = 0.8
    max_angular_speed_radps: float = 1.2


class SafetyController:
    def __init__(self, config: SafetyConfig | None = None):
        self.config = config or SafetyConfig()
        self._last_command_time = 0.0
        self._emergency_stopped = False

    @property
    def emergency_stopped(self) -> bool:
        return self._emergency_stopped

    def accept(self, command: ChassisCommand) -> ChassisCommand:
        self._last_command_time = time.monotonic()
        if command.emergency_stop:
            self._emergency_stopped = True
        if command.mode == "idle":
            self._emergency_stopped = False

        return ChassisCommand(
            linear_speed_mps=self._clamp(command.linear_speed_mps, self.config.max_linear_speed_mps),
            angular_speed_radps=self._clamp(command.angular_speed_radps, self.config.max_angular_speed_radps),
            brake=command.brake,
            emergency_stop=self._emergency_stopped,
            mode=command.mode,
            stamp=command.stamp,
        )

    def timed_out(self) -> bool:
        if self._last_command_time <= 0:
            return False
        return time.monotonic() - self._last_command_time > self.config.command_timeout_s

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-limit, min(value, limit))
