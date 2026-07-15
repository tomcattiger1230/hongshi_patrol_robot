"""Remote-side communication client.

The current implementation uses UDP JSON for development. Keep the public
methods stable when replacing the internals with DDS topics.
"""

from __future__ import annotations

from typing import Optional

from mobile_platform.messages import ChassisCommand, RobotTelemetry
from mobile_platform.transport import (
    UdpEndpoint,
    UdpJsonCommandPublisher,
    UdpJsonTelemetrySubscriber,
)


class RobotRemoteClient:
    def __init__(self, robot_command_endpoint: UdpEndpoint, telemetry_bind: UdpEndpoint):
        self._commands = UdpJsonCommandPublisher(robot_command_endpoint)
        self._telemetry = UdpJsonTelemetrySubscriber(telemetry_bind)
        self.latest_telemetry: Optional[RobotTelemetry] = None

    def send_manual_command(
        self,
        linear_speed_mps: float = 0.0,
        angular_speed_radps: float = 0.0,
        brake: bool = False,
        emergency_stop: bool = False,
    ) -> None:
        self._commands.publish_command(
            ChassisCommand(
                linear_speed_mps=linear_speed_mps,
                angular_speed_radps=angular_speed_radps,
                brake=brake,
                emergency_stop=emergency_stop,
                mode="manual",
            )
        )

    def stop(self) -> None:
        self.send_manual_command(0.0, 0.0)

    def brake(self) -> None:
        self.send_manual_command(0.0, 0.0, brake=True)

    def emergency_stop(self) -> None:
        self.send_manual_command(0.0, 0.0, emergency_stop=True)

    def reset_idle(self) -> None:
        self._commands.publish_command(ChassisCommand(mode="idle"))

    def receive_telemetry(self, timeout_s: float = 0.1) -> Optional[RobotTelemetry]:
        telemetry = self._telemetry.receive_telemetry(timeout_s)
        if telemetry:
            self.latest_telemetry = telemetry
        return telemetry

    def close(self) -> None:
        self._commands.close()
        self._telemetry.close()
