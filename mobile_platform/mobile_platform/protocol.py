"""Robot320 chassis CAN protocol helpers.

The values in this module come from the original PyQt5 debug program:

- CAN bitrate: 500 kbps
- Drive motor commands use extended CAN frames.
- Brake and steering commands use standard CAN frames.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Iterable, Optional


MOTOR_ENABLE_ID = 0x03011008
MOTOR_SPEED_ID = 0x030110BA
BRAKE_ID = 0x000007B9
STEERING_ID = 0x00000169
SPEED_REQUEST_ID = 0x020101B9

SPEED_FEEDBACK_IDS = {0x000110B9, SPEED_REQUEST_ID}


class Direction(str, Enum):
    FORWARD = "forward"
    BACKWARD = "backward"
    CENTER = "center"
    LEFT = "left"
    RIGHT = "right"


@dataclass(frozen=True)
class CANFrame:
    can_id: int
    data: bytes
    extended: bool = False


@dataclass(frozen=True)
class SpeedFeedback:
    speed_kmh: float
    raw_value: int


_DIRECTION_ALIASES = {
    "forward": Direction.FORWARD,
    "backward": Direction.BACKWARD,
    "center": Direction.CENTER,
    "left": Direction.LEFT,
    "right": Direction.RIGHT,
    "前进": Direction.FORWARD,
    "后退": Direction.BACKWARD,
    "中": Direction.CENTER,
    "居中": Direction.CENTER,
    "左": Direction.LEFT,
    "右": Direction.RIGHT,
}


def normalize_direction(direction: Direction | str) -> Direction:
    if isinstance(direction, Direction):
        return direction
    try:
        return _DIRECTION_ALIASES[direction]
    except KeyError as exc:
        raise ValueError(f"unsupported direction: {direction}") from exc


def _bytes(values: Iterable[int]) -> bytes:
    payload = bytes(values)
    if len(payload) > 8:
        raise ValueError("CAN payload cannot exceed 8 bytes")
    return payload


def motor_enable_frame(enabled: bool) -> CANFrame:
    return CANFrame(MOTOR_ENABLE_ID, _bytes([0x0A if enabled else 0x01, 0x00]), True)


def motor_speed_frame(direction: Direction | int, rpm: int) -> CANFrame:
    if rpm < 0:
        raise ValueError("rpm must be >= 0")
    if rpm > 32767:
        raise ValueError("rpm is too large for signed 16-bit command")

    if isinstance(direction, str):
        direction = normalize_direction(direction)

    if isinstance(direction, Direction):
        sign = 1 if direction == Direction.FORWARD else -1
    else:
        sign = 1 if direction >= 0 else -1

    speed_value = sign * rpm
    return CANFrame(MOTOR_SPEED_ID, speed_value.to_bytes(2, "little", signed=True), True)


def brake_frame(pressure_mpa: float = 5.0) -> CANFrame:
    if pressure_mpa < 0:
        raise ValueError("pressure_mpa must be >= 0")
    # Original GUI uses 0x64 for 5 MPa, so the command scale is 20 counts/MPa.
    pressure_counts = max(0, min(int(round(pressure_mpa * 20)), 0xFFFF))
    return CANFrame(
        BRAKE_ID,
        _bytes(
            [
                0x06,
                0x00,
                (pressure_counts >> 8) & 0xFF,
                pressure_counts & 0xFF,
                0x00,
                0x00,
                0x00,
                0x00,
            ]
        ),
        False,
    )


def release_brake_frame() -> CANFrame:
    return CANFrame(BRAKE_ID, _bytes([0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]), False)


def steering_frame(angle_deg: int, direction: Direction | str) -> CANFrame:
    if not 0 <= angle_deg <= 350:
        raise ValueError("angle_deg must be between 0 and 350")

    direction = normalize_direction(direction)

    if angle_deg == 0 or direction == Direction.CENTER:
        data = [0x02, 0x75, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00]
    else:
        if direction == Direction.RIGHT:
            value = int((3000 + angle_deg) / 0.1)
        elif direction == Direction.LEFT:
            value = int((3000 - angle_deg) / 0.1)
        else:
            raise ValueError("steering direction must be left, right, or center")
        data = [0x02, (value >> 8) & 0xFF, value & 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00]

    return CANFrame(STEERING_ID, _bytes(data), False)


def speed_request_frame() -> CANFrame:
    return CANFrame(SPEED_REQUEST_ID, _bytes([0x00, 0x00]), True)


def parse_speed_feedback(can_id: int, data: bytes, extended: bool) -> Optional[SpeedFeedback]:
    if not extended or can_id not in SPEED_FEEDBACK_IDS or len(data) < 4:
        return None
    raw = (data[2] << 8) | data[3]
    if raw <= 0:
        return None
    return SpeedFeedback(speed_kmh=raw / 100.0, raw_value=raw)
