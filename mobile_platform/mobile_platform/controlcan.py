"""Thin wrapper around libcontrolcan.so."""

from __future__ import annotations

import os
import platform
from ctypes import byref, cdll
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from .can_types import VCI_BOARD_INFO, VCI_CAN_OBJ, VCI_INIT_CONFIG
from .protocol import CANFrame


VCI_USBCAN2 = 4
STATUS_OK = 1


@dataclass(frozen=True)
class CANAdapterConfig:
    device_type: int = VCI_USBCAN2
    device_index: int = 0
    can_index: int = 0
    bitrate: str = "500k"
    library_path: Optional[str] = None


@dataclass(frozen=True)
class BoardInfo:
    serial_number: str
    hardware_type: str
    firmware_version: str


@dataclass(frozen=True)
class ReceivedFrame:
    can_id: int
    data: bytes
    extended: bool
    timestamp: int = 0


class ControlCANError(RuntimeError):
    pass


class ControlCANAdapter:
    def __init__(self, config: Optional[CANAdapterConfig] = None):
        self.config = config or CANAdapterConfig()
        self._dll = None
        self._opened = False
        self._started = False

    @property
    def is_open(self) -> bool:
        return self._opened

    def open(self) -> Optional[BoardInfo]:
        self._load_library()
        cfg = self.config

        ret = self._dll.VCI_OpenDevice(cfg.device_type, cfg.device_index, 0)
        if ret != STATUS_OK:
            raise ControlCANError("failed to open CAN device")
        self._opened = True

        board_info = self._read_board_info()
        self._init_can()
        self._start_can()
        return board_info

    def close(self) -> None:
        cfg = self.config
        if self._dll is None:
            return

        if self._opened:
            if self._started:
                self._dll.VCI_ResetCAN(cfg.device_type, cfg.device_index, cfg.can_index)
                self._started = False
            self._dll.VCI_CloseDevice(cfg.device_type, cfg.device_index)
            self._opened = False

    def send(self, frame: CANFrame) -> bool:
        if not self._opened or self._dll is None:
            raise ControlCANError("CAN device is not open")
        if len(frame.data) > 8:
            raise ValueError("CAN payload cannot exceed 8 bytes")

        msg = VCI_CAN_OBJ()
        msg.ID = frame.can_id
        msg.SendType = 1
        msg.RemoteFlag = 0
        msg.ExternFlag = 1 if frame.extended else 0
        msg.DataLen = len(frame.data)

        for index, value in enumerate(frame.data):
            msg.Data[index] = value

        cfg = self.config
        ret = self._dll.VCI_Transmit(cfg.device_type, cfg.device_index, cfg.can_index, byref(msg), 1)
        return ret == STATUS_OK

    def receive(self, max_frames: int = 100, timeout_ms: int = 50) -> list[ReceivedFrame]:
        if not self._opened or self._dll is None:
            raise ControlCANError("CAN device is not open")

        rec = (VCI_CAN_OBJ * max_frames)()
        cfg = self.config
        count = self._dll.VCI_Receive(
            cfg.device_type,
            cfg.device_index,
            cfg.can_index,
            rec,
            max_frames,
            timeout_ms,
        )

        frames: list[ReceivedFrame] = []
        if count <= 0:
            return frames

        for index in range(count):
            item = rec[index]
            frames.append(
                ReceivedFrame(
                    can_id=item.ID,
                    data=bytes(item.Data[: item.DataLen]),
                    extended=item.ExternFlag == 1,
                    timestamp=item.TimeStamp,
                )
            )
        return frames

    def _load_library(self) -> None:
        if self._dll is not None:
            return
        library_path = resolve_controlcan_library(self.config.library_path)
        self._dll = cdll.LoadLibrary(str(library_path))

    def _read_board_info(self) -> Optional[BoardInfo]:
        cfg = self.config
        info = VCI_BOARD_INFO()
        ret = self._dll.VCI_ReadBoardInfo(cfg.device_type, cfg.device_index, byref(info))
        if ret != STATUS_OK:
            return None

        serial = info.str_Serial_Num.decode("utf-8", errors="ignore").strip("\x00").strip()
        hardware = info.str_hw_Type.decode("utf-8", errors="ignore").strip("\x00").strip()
        fw = info.fw_Version
        firmware = f"V{(fw & 0xF00) >> 8}.{(fw & 0xF0) >> 4}{fw & 0xF}"
        return BoardInfo(serial_number=serial, hardware_type=hardware, firmware_version=firmware)

    def _init_can(self) -> None:
        cfg = self.config
        init_config = VCI_INIT_CONFIG()
        init_config.AccCode = 0
        init_config.AccMask = 0xFFFFFFFF
        init_config.Filter = 0
        init_config.Timing0 = 0x00
        init_config.Timing1 = 0x1C
        init_config.Mode = 0

        ret = self._dll.VCI_InitCAN(cfg.device_type, cfg.device_index, cfg.can_index, byref(init_config))
        if ret != STATUS_OK:
            self.close()
            raise ControlCANError("failed to initialize CAN channel at 500 kbps")

    def _start_can(self) -> None:
        cfg = self.config
        ret = self._dll.VCI_StartCAN(cfg.device_type, cfg.device_index, cfg.can_index)
        if ret != STATUS_OK:
            self.close()
            raise ControlCANError("failed to start CAN channel")
        self._started = True


def resolve_controlcan_library(configured_path: Optional[str] = None) -> Path:
    """Resolve the ControlCAN shared library path.

    Search order:
    1. Explicit config value, for CLI ``--lib`` and application config.
    2. ``CONTROL_CAN_LIB`` environment variable.
    3. ``./libcontrolcan.so`` in the current working directory.
    4. Bundled Linux vendor libraries selected by CPU architecture.
    """

    candidates: list[Path] = []
    if configured_path:
        candidates.append(Path(configured_path))

    env_path = os.environ.get("CONTROL_CAN_LIB")
    if env_path:
        candidates.append(Path(env_path))

    candidates.append(Path.cwd() / "libcontrolcan.so")

    bundled = _bundled_library_path()
    if bundled:
        candidates.append(bundled)

    for candidate in candidates:
        if candidate.exists():
            return candidate

    searched = ", ".join(str(candidate) for candidate in candidates)
    raise ControlCANError(f"libcontrolcan.so not found; searched: {searched}")


def _bundled_library_path() -> Optional[Path]:
    if platform.system().lower() != "linux":
        return None

    machine = platform.machine().lower()
    arch_map = {
        "x86_64": "linux-x86_64",
        "amd64": "linux-x86_64",
        "i386": "linux-x86",
        "i686": "linux-x86",
        "aarch64": "linux-aarch64",
        "arm64": "linux-aarch64",
        "armv7l": "linux-armv7",
        "armv6l": "linux-armv7",
        "arm": "linux-armv7",
    }
    arch_dir = arch_map.get(machine)
    if not arch_dir:
        return None

    return Path(__file__).resolve().parent / "vendor" / "controlcan" / arch_dir / "libcontrolcan.so"
