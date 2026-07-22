#!/usr/bin/env python3
"""Run an authenticated go2rtc proxy for low-latency LAN viewing."""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

try:
    from .push_stream import add_credentials, load_env_file, required
except ImportError:
    from push_stream import add_credentials, load_env_file, required


def build_config() -> dict[str, object]:
    camera_url = add_credentials(
        required("CAMERA_RTSP_URL"),
        os.getenv("CAMERA_USER", ""),
        os.getenv("CAMERA_PASSWORD", ""),
    )
    port = int(os.getenv("LOCAL_RTSP_PORT", "8554"))
    return {
        "app": {"modules": ["api", "rtsp"]},
        "streams": {os.getenv("STREAM_PATH", "robot").strip("/"): camera_url},
        "api": {"listen": "127.0.0.1:1984"},
        "rtsp": {
            "listen": f":{port}",
            "username": required("LOCAL_READ_USER"),
            "password": required("LOCAL_READ_PASSWORD"),
            "default_query": "video",
        },
    }


def main() -> int:
    load_env_file(Path(__file__).with_name(".env"))
    binary = Path(
        os.getenv("GO2RTC_BIN", str(Path.home() / "go2rtc/go2rtc_linux_amd64"))
    )
    try:
        config = build_config()
    except (ValueError, OverflowError) as exc:
        print(f"配置错误：{exc}", file=sys.stderr)
        return 2
    if not binary.is_file():
        print(f"找不到 go2rtc：{binary}", file=sys.stderr)
        return 2
    if not hasattr(os, "memfd_create"):
        print("当前系统不支持安全的内存配置文件", file=sys.stderr)
        return 2
    config_fd = os.memfd_create("go2rtc-config")
    os.write(config_fd, json.dumps(config).encode())
    os.lseek(config_fd, 0, os.SEEK_SET)
    os.set_inheritable(config_fd, True)
    os.execv(str(binary), [str(binary), "-config", f"/proc/self/fd/{config_fd}"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
