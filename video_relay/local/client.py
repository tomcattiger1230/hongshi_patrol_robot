#!/usr/bin/env python3
"""Play, probe, or record the relayed stream with FFmpeg tools."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from urllib.parse import quote


def load_env_file(path: Path) -> None:
    if not path.exists():
        return
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        os.environ.setdefault(key.strip(), value.strip().strip("'\""))


def stream_url() -> str:
    names = ("SERVER_HOST", "READ_USER", "READ_PASSWORD")
    missing = [name for name in names if not os.getenv(name, "").strip()]
    if missing:
        raise ValueError("缺少配置项：" + ", ".join(missing))
    user = quote(os.environ["READ_USER"], safe="")
    password = quote(os.environ["READ_PASSWORD"], safe="")
    host = os.environ["SERVER_HOST"].strip()
    port = os.getenv("SERVER_PORT", "8554")
    path = os.getenv("STREAM_PATH", "robot").strip("/")
    return f"rtsp://{user}:{password}@{host}:{port}/{path}"


def main() -> int:
    parser = argparse.ArgumentParser(description="查看或录制机器人公网视频流")
    subparsers = parser.add_subparsers(dest="action")
    subparsers.add_parser("play", help="低延迟播放（默认）")
    subparsers.add_parser("probe", help="检查码流信息")
    record = subparsers.add_parser("record", help="无损录制为 MKV")
    record.add_argument("output", nargs="?", help="输出文件路径")
    args = parser.parse_args()
    action = args.action or "play"
    load_env_file(Path(__file__).with_name(".env"))
    try:
        url = stream_url()
    except ValueError as exc:
        print(f"配置错误：{exc}", file=sys.stderr)
        return 2

    executable = "ffplay" if action == "play" else ("ffprobe" if action == "probe" else "ffmpeg")
    if shutil.which(executable) is None:
        print(f"错误：未找到 {executable}，请先安装 FFmpeg。", file=sys.stderr)
        return 2
    common = ["-rtsp_transport", "tcp"]
    if action == "play":
        command = [
            "ffplay", "-hide_banner", *common, "-fflags", "nobuffer",
            "-flags", "low_delay", "-framedrop", "-analyzeduration", "1000000",
            "-probesize", "1000000", url,
        ]
    elif action == "probe":
        command = ["ffprobe", "-hide_banner", *common, url]
    else:
        output = args.output or f"robot-{datetime.now():%Y%m%d-%H%M%S}.mkv"
        command = ["ffmpeg", "-hide_banner", *common, "-i", url, "-map", "0", "-c", "copy", output]
    return subprocess.call(command)


if __name__ == "__main__":
    raise SystemExit(main())
