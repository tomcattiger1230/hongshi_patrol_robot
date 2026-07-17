"""Transport protocols and UDP JSON debug adapters."""

from __future__ import annotations

import socket
from dataclasses import dataclass
from typing import Optional, Protocol, Tuple

from .messages import ChassisCommand, RobotTelemetry, command_from_json, telemetry_from_json, to_json


class CommandSubscriber(Protocol):
    def receive_command(self, timeout_s: float = 0.1) -> Optional[ChassisCommand]: ...


class CommandPublisher(Protocol):
    def publish_command(self, command: ChassisCommand) -> None: ...


class TelemetryPublisher(Protocol):
    def publish_telemetry(self, telemetry: RobotTelemetry) -> None: ...


class TelemetrySubscriber(Protocol):
    def receive_telemetry(self, timeout_s: float = 0.1) -> Optional[RobotTelemetry]: ...


@dataclass(frozen=True)
class UdpEndpoint:
    host: str
    port: int


class UdpJsonCommandSubscriber:
    def __init__(self, bind: UdpEndpoint):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((bind.host, bind.port))

    def receive_command(self, timeout_s: float = 0.1) -> Optional[ChassisCommand]:
        self._sock.settimeout(timeout_s)
        try:
            payload, _ = self._sock.recvfrom(4096)
        except socket.timeout:
            return None
        return command_from_json(payload)

    def close(self) -> None:
        self._sock.close()


class UdpJsonCommandPublisher:
    def __init__(self, remote: UdpEndpoint):
        self._remote: Tuple[str, int] = (remote.host, remote.port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def publish_command(self, command: ChassisCommand) -> None:
        self._sock.sendto(to_json(command).encode("utf-8"), self._remote)

    def close(self) -> None:
        self._sock.close()


class UdpJsonTelemetryPublisher:
    def __init__(self, remote: UdpEndpoint):
        self._remote: Tuple[str, int] = (remote.host, remote.port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def publish_telemetry(self, telemetry: RobotTelemetry) -> None:
        self._sock.sendto(to_json(telemetry).encode("utf-8"), self._remote)

    def close(self) -> None:
        self._sock.close()


class UdpJsonTelemetrySubscriber:
    def __init__(self, bind: UdpEndpoint):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((bind.host, bind.port))

    def receive_telemetry(self, timeout_s: float = 0.1) -> Optional[RobotTelemetry]:
        self._sock.settimeout(timeout_s)
        try:
            payload, _ = self._sock.recvfrom(65535)
        except socket.timeout:
            return None
        return telemetry_from_json(payload)

    def close(self) -> None:
        self._sock.close()
