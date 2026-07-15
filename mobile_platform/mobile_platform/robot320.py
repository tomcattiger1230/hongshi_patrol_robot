"""High-level Robot320 chassis controller."""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

from .controlcan import BoardInfo, CANAdapterConfig, ControlCANAdapter, ReceivedFrame
from .protocol import (
    Direction,
    SpeedFeedback,
    brake_frame,
    motor_enable_frame,
    motor_speed_frame,
    parse_speed_feedback,
    normalize_direction,
    release_brake_frame,
    speed_request_frame,
    steering_frame,
)

LOGGER = logging.getLogger(__name__)


@dataclass
class PlatformState:
    connected: bool = False
    enabled: bool = False
    brake_engaged: bool = False
    commanded_rpm: int = 0
    commanded_direction: Direction = Direction.CENTER
    steering_angle_deg: int = 0
    steering_direction: Direction = Direction.CENTER
    speed_kmh: Optional[float] = None
    last_frame: Optional[ReceivedFrame] = None


FrameCallback = Callable[[ReceivedFrame], None]
SpeedCallback = Callable[[SpeedFeedback], None]


class Robot320Platform:
    def __init__(
        self,
        adapter: Optional[ControlCANAdapter] = None,
        config: Optional[CANAdapterConfig] = None,
        speed_poll_interval_s: float = 2.5,
    ):
        self.adapter = adapter or ControlCANAdapter(config)
        self.state = PlatformState()
        self.speed_poll_interval_s = speed_poll_interval_s
        self.on_frame: Optional[FrameCallback] = None
        self.on_speed: Optional[SpeedCallback] = None
        self._running = False
        self._rx_thread: Optional[threading.Thread] = None
        self._lock = threading.RLock()

    def connect(self, start_receiver: bool = True) -> Optional[BoardInfo]:
        board_info = self.adapter.open()
        with self._lock:
            self.state.connected = True

        if start_receiver:
            self.start_receiver()
        return board_info

    def disconnect(self, safe_stop: bool = True) -> None:
        if safe_stop and self.state.connected:
            try:
                self.stop_motor()
            except Exception:
                LOGGER.exception("failed to stop motor during disconnect")

        self.stop_receiver()
        self.adapter.close()
        with self._lock:
            self.state.connected = False
            self.state.enabled = False
            self.state.brake_engaged = False
            self.state.commanded_rpm = 0
            self.state.commanded_direction = Direction.CENTER

    def enable_motor(self) -> bool:
        ok = self.adapter.send(motor_enable_frame(True))
        if ok:
            with self._lock:
                self.state.enabled = True
        return ok

    def disable_motor(self) -> bool:
        ok = self.adapter.send(motor_enable_frame(False))
        if ok:
            with self._lock:
                self.state.enabled = False
                self.state.commanded_rpm = 0
                self.state.commanded_direction = Direction.CENTER
        return ok

    def set_motor_speed(self, direction: Direction | int | str, rpm: int) -> bool:
        if isinstance(direction, str):
            direction = normalize_direction(direction)

        if rpm <= 0:
            return self.stop_motor()

        self.enable_motor()
        time.sleep(0.03)
        self.release_brake()
        time.sleep(0.03)
        ok = self.adapter.send(motor_speed_frame(direction, rpm))
        if ok:
            with self._lock:
                self.state.commanded_rpm = rpm
                if isinstance(direction, Direction):
                    self.state.commanded_direction = direction
                else:
                    self.state.commanded_direction = Direction.FORWARD if direction >= 0 else Direction.BACKWARD
        return ok

    def stop_motor(self) -> bool:
        ok = self.adapter.send(motor_enable_frame(False))
        if ok:
            with self._lock:
                self.state.enabled = False
                self.state.commanded_rpm = 0
                self.state.commanded_direction = Direction.CENTER
        return ok

    def brake(self, pressure_mpa: float = 5.0) -> bool:
        ok = self.adapter.send(brake_frame(pressure_mpa))
        if ok:
            with self._lock:
                self.state.brake_engaged = True
        return ok

    def release_brake(self) -> bool:
        ok = self.adapter.send(release_brake_frame())
        if ok:
            with self._lock:
                self.state.brake_engaged = False
        return ok

    def turn(self, angle_deg: int, direction: Direction | str) -> bool:
        if isinstance(direction, str):
            direction = normalize_direction(direction)

        ok = self.adapter.send(steering_frame(angle_deg, direction))
        if ok:
            with self._lock:
                self.state.steering_angle_deg = 0 if direction == Direction.CENTER else angle_deg
                self.state.steering_direction = direction
        return ok

    def center_steering(self) -> bool:
        return self.turn(0, Direction.CENTER)

    def request_speed(self) -> bool:
        return self.adapter.send(speed_request_frame())

    def receive_once(self, timeout_ms: int = 50) -> list[ReceivedFrame]:
        frames = self.adapter.receive(timeout_ms=timeout_ms)
        for frame in frames:
            self._handle_frame(frame)
        return frames

    def start_receiver(self) -> None:
        if self._running:
            return
        self._running = True
        self._rx_thread = threading.Thread(target=self._receive_loop, name="robot320-can-rx", daemon=True)
        self._rx_thread.start()

    def stop_receiver(self) -> None:
        self._running = False
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.5)
        self._rx_thread = None

    def snapshot(self) -> PlatformState:
        with self._lock:
            return PlatformState(**self.state.__dict__)

    def _receive_loop(self) -> None:
        next_speed_poll = time.monotonic() + 0.5
        while self._running:
            try:
                self.receive_once(timeout_ms=50)
                now = time.monotonic()
                if now >= next_speed_poll:
                    self.request_speed()
                    next_speed_poll = now + self.speed_poll_interval_s
            except Exception:
                LOGGER.exception("CAN receive loop error")
                time.sleep(0.2)
            time.sleep(0.01)

    def _handle_frame(self, frame: ReceivedFrame) -> None:
        feedback = parse_speed_feedback(frame.can_id, frame.data, frame.extended)
        with self._lock:
            self.state.last_frame = frame
            if feedback:
                self.state.speed_kmh = feedback.speed_kmh

        if self.on_frame:
            self.on_frame(frame)
        if feedback and self.on_speed:
            self.on_speed(feedback)
