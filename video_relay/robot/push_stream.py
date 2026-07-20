#!/usr/bin/env python3
"""Continuously republish a Hikvision RTSP stream to a public MediaMTX server."""

from __future__ import annotations

import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from urllib.parse import quote, urlsplit, urlunsplit


STOP_REQUESTED = False
CREDENTIALS_PATTERN = re.compile(r"(?P<scheme>rtsps?://)[^/@\s]+@", re.IGNORECASE)


def load_env_file(path: Path) -> None:
    """Load a small KEY=VALUE env file without adding a third-party dependency."""
    if not path.exists():
        return
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        os.environ.setdefault(key.strip(), value.strip().strip("'\""))


def add_credentials(url: str, username: str, password: str) -> str:
    """Add percent-encoded credentials to an RTSP URL."""
    parts = urlsplit(url)
    if parts.scheme not in {"rtsp", "rtsps"} or not parts.hostname:
        raise ValueError("CAMERA_RTSP_URL 必须是有效的 rtsp:// 或 rtsps:// 地址")
    host = f"[{parts.hostname}]" if ":" in parts.hostname else parts.hostname
    if parts.port is not None:
        host = f"{host}:{parts.port}"
    auth = f"{quote(username, safe='')}:{quote(password, safe='')}@" if username else ""
    return urlunsplit((parts.scheme, auth + host, parts.path, parts.query, parts.fragment))


def required(name: str) -> str:
    value = os.getenv(name, "").strip()
    if not value:
        raise ValueError(f"缺少配置项 {name}")
    return value


def build_command() -> list[str]:
    camera_url = add_credentials(
        required("CAMERA_RTSP_URL"),
        os.getenv("CAMERA_USER", ""),
        os.getenv("CAMERA_PASSWORD", ""),
    )
    server_host = required("SERVER_HOST")
    server_port = os.getenv("SERVER_PORT", "8554")
    stream_path = os.getenv("STREAM_PATH", "robot").strip("/")
    publish_url = add_credentials(
        f"rtsp://{server_host}:{server_port}/{stream_path}",
        required("PUBLISH_USER"),
        required("PUBLISH_PASSWORD"),
    )
    mode = os.getenv("VIDEO_MODE", "copy").lower()
    if mode not in {"copy", "transcode"}:
        raise ValueError("VIDEO_MODE 只能是 copy 或 transcode")
    encoder = os.getenv("VIDEO_ENCODER", "libx264").lower()
    if encoder not in {"libx264", "h264_qsv"}:
        raise ValueError("VIDEO_ENCODER 只能是 libx264 或 h264_qsv")

    command = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        os.getenv("FFMPEG_LOG_LEVEL", "warning"),
        "-fflags",
        "+genpts",
        "-use_wallclock_as_timestamps",
        "1",
        "-rtsp_transport",
        "tcp",
        "-timeout",
        "15000000",
        "-i",
        camera_url,
        "-map",
        "0:v:0",
        "-an",
    ]
    if mode == "copy":
        command += ["-c:v", "copy"]
    else:
        fps = os.getenv("VIDEO_FPS", "25")
        bitrate = os.getenv("VIDEO_BITRATE", "2500k")
        if encoder == "libx264":
            command += [
                "-c:v", "libx264", "-preset", "veryfast",
                "-tune", "zerolatency", "-pix_fmt", "yuv420p",
            ]
        else:
            command += ["-vf", "format=nv12", "-c:v", "h264_qsv", "-preset", "veryfast"]
        command += [
            "-r",
            fps,
            "-g",
            str(int(fps) * 2),
            "-b:v",
            bitrate,
            "-maxrate",
            bitrate,
            "-bufsize",
            bitrate,
        ]
    command += ["-f", "rtsp", "-rtsp_transport", "tcp", publish_url]
    return command


def redact_command(command: list[str]) -> str:
    """Return a loggable command without camera or server credentials."""
    redacted: list[str] = []
    for item in command:
        if item.startswith(("rtsp://", "rtsps://")):
            parts = urlsplit(item)
            host = parts.hostname or "unknown"
            if parts.port:
                host = f"{host}:{parts.port}"
            item = urlunsplit((parts.scheme, host, parts.path, parts.query, ""))
        redacted.append(item)
    return " ".join(redacted)


def redact_text(value: str) -> str:
    """Remove RTSP URL credentials from a child-process log line."""
    return CREDENTIALS_PATTERN.sub(r"\g<scheme><credentials-redacted>@", value)


def forward_ffmpeg_logs(pipe: object) -> None:
    """Forward FFmpeg stderr while preventing credential disclosure."""
    if not hasattr(pipe, "readline"):
        return
    try:
        for line in iter(pipe.readline, ""):
            print(redact_text(line.rstrip("\n")), file=sys.stderr, flush=True)
    finally:
        pipe.close()


def request_stop(_signum: int, _frame: object) -> None:
    global STOP_REQUESTED
    STOP_REQUESTED = True


def main() -> int:
    load_env_file(Path(__file__).with_name(".env"))
    if shutil.which("ffmpeg") is None:
        print("错误：未找到 ffmpeg，请先安装 FFmpeg。", file=sys.stderr)
        return 2
    try:
        command = build_command()
        min_delay = max(1.0, float(os.getenv("RECONNECT_MIN_SECONDS", "2")))
        max_delay = max(min_delay, float(os.getenv("RECONNECT_MAX_SECONDS", "30")))
    except (ValueError, OverflowError) as exc:
        print(f"配置错误：{exc}", file=sys.stderr)
        return 2

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    delay = min_delay
    while not STOP_REQUESTED:
        print(f"启动视频转发：{redact_command(command)}", flush=True)
        started_at = time.monotonic()
        process = subprocess.Popen(
            command,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        log_thread = threading.Thread(
            target=forward_ffmpeg_logs,
            args=(process.stderr,),
            name="ffmpeg-log-forwarder",
            daemon=True,
        )
        log_thread.start()
        while process.poll() is None and not STOP_REQUESTED:
            time.sleep(0.5)
        if STOP_REQUESTED and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                process.terminate()
        log_thread.join(timeout=2)
        if STOP_REQUESTED:
            break
        runtime = time.monotonic() - started_at
        if runtime >= 60:
            delay = min_delay
        print(f"FFmpeg 已退出（状态 {process.returncode}），{delay:.0f} 秒后重连。", flush=True)
        time.sleep(delay)
        delay = min(delay * 2, max_delay)
    print("视频转发已停止。", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
