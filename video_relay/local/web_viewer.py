#!/usr/bin/env python3
"""Expose the relayed RTSP stream as a small local MJPEG web interface."""

from __future__ import annotations

import atexit
import os
import sys
import threading
import time
from pathlib import Path
from typing import Any, Iterator

try:
    import cv2
    from flask import Flask, Response, jsonify
except ImportError as exc:
    print("缺少依赖，请执行：python3 -m pip install -r requirements.txt", file=sys.stderr)
    raise SystemExit(2) from exc

try:
    from .client import load_env_file, stream_url
except ImportError:
    from client import load_env_file, stream_url


INDEX_HTML = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>机器人实时视频</title>
  <style>
    :root { color-scheme: dark; font-family: system-ui, sans-serif; }
    body { margin: 0; background: #101317; color: #e9eef5; }
    main { max-width: 1200px; margin: auto; padding: 20px; }
    header { display: flex; align-items: center; gap: 12px; flex-wrap: wrap; }
    h1 { font-size: 20px; margin-right: auto; }
    #dot { width: 10px; height: 10px; border-radius: 50%; background: #e5a72e; }
    #dot.ok { background: #3bc878; }
    #dot.bad { background: #ed5c5c; }
    #video { display: block; width: 100%; min-height: 240px; margin-top: 14px;
      object-fit: contain; background: #050607; border-radius: 8px; }
    #detail { color: #9aa7b5; font-size: 14px; }
    a { color: #74b7ff; }
  </style>
</head>
<body><main>
  <header><h1>机器人实时视频</h1><span id="dot"></span><span id="state">正在连接</span></header>
  <div id="detail">等待视频数据……</div>
  <img id="video" src="/video" alt="机器人视频流">
  <p><a href="/snapshot.jpg" target="_blank">获取当前 JPEG 图片</a></p>
</main>
<script>
async function updateStatus() {
  try {
    const r = await fetch('/api/status', {cache: 'no-store'});
    const s = await r.json();
    const ok = s.state === 'connected';
    document.querySelector('#dot').className = ok ? 'ok' : (s.state === 'error' ? 'bad' : '');
    document.querySelector('#state').textContent = s.message;
    document.querySelector('#detail').textContent = ok
      ? `${s.width} × ${s.height} · ${s.fps.toFixed(1)} FPS · 最新帧 ${s.frame_age_seconds.toFixed(1)} 秒前`
      : '程序会自动重连，无需刷新页面';
  } catch (_) {
    document.querySelector('#dot').className = 'bad';
    document.querySelector('#state').textContent = 'Web 服务连接失败';
  }
}
updateStatus(); setInterval(updateStatus, 1000);
</script></body></html>"""


class RtspReader:
    def __init__(self, url: str, jpeg_quality: int) -> None:
        self.url = url
        self.jpeg_quality = jpeg_quality
        self.condition = threading.Condition()
        self.stop_event = threading.Event()
        self.frame: bytes | None = None
        self.sequence = 0
        self.state = "connecting"
        self.message = "正在连接 RTSP 视频流"
        self.width = 0
        self.height = 0
        self.fps = 0.0
        self.last_frame_at = 0.0
        self._fps_sample_started = 0.0
        self._fps_sample_frames = 0
        self._thread = threading.Thread(target=self._run, name="rtsp-reader", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        with self.condition:
            self.condition.notify_all()

    def _set_state(self, state: str, message: str) -> None:
        with self.condition:
            self.state = state
            self.message = message
            self.condition.notify_all()

    def _open_capture(self) -> Any:
        params = [
            cv2.CAP_PROP_OPEN_TIMEOUT_MSEC,
            15000,
            cv2.CAP_PROP_READ_TIMEOUT_MSEC,
            20000,
        ]
        capture = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG, params)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return capture

    def _run(self) -> None:
        while not self.stop_event.is_set():
            self._set_state("connecting", "正在连接 RTSP 视频流")
            capture = self._open_capture()
            if not capture.isOpened():
                capture.release()
                self._set_state("error", "无法连接视频流，2 秒后重试")
                self.stop_event.wait(2)
                continue

            self._set_state("connected", "视频已连接")
            while not self.stop_event.is_set():
                success, raw_frame = capture.read()
                if not success:
                    self._set_state("error", "视频中断，正在重新连接")
                    break
                encoded, jpeg = cv2.imencode(
                    ".jpg",
                    raw_frame,
                    [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
                )
                if not encoded:
                    continue
                now = time.monotonic()
                if not self._fps_sample_started:
                    self._fps_sample_started = now
                self._fps_sample_frames += 1
                sample_elapsed = now - self._fps_sample_started
                with self.condition:
                    self.frame = jpeg.tobytes()
                    self.sequence += 1
                    self.width = int(raw_frame.shape[1])
                    self.height = int(raw_frame.shape[0])
                    if sample_elapsed >= 1:
                        self.fps = self._fps_sample_frames / sample_elapsed
                        self._fps_sample_started = now
                        self._fps_sample_frames = 0
                    self.last_frame_at = now
                    self.state = "connected"
                    self.message = "视频已连接"
                    self.condition.notify_all()
            capture.release()
            self.stop_event.wait(1)

    def frames(self) -> Iterator[bytes]:
        last_sequence = -1
        while not self.stop_event.is_set():
            with self.condition:
                self.condition.wait_for(
                    lambda: self.sequence != last_sequence or self.stop_event.is_set(),
                    timeout=5,
                )
                if self.frame is None or self.sequence == last_sequence:
                    continue
                frame = self.frame
                last_sequence = self.sequence
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"

    def status(self) -> dict[str, Any]:
        with self.condition:
            age = time.monotonic() - self.last_frame_at if self.last_frame_at else 0.0
            return {
                "state": self.state,
                "message": self.message,
                "width": self.width,
                "height": self.height,
                "fps": round(self.fps, 2),
                "frame_age_seconds": round(age, 2),
                "frames_received": self.sequence,
            }

    def snapshot(self) -> bytes | None:
        with self.condition:
            return self.frame


def create_app(reader: RtspReader) -> Flask:
    app = Flask(__name__)

    @app.get("/")
    def index() -> Response:
        return Response(INDEX_HTML, content_type="text/html; charset=utf-8")

    @app.get("/video")
    def video() -> Response:
        return Response(reader.frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.get("/snapshot.jpg")
    def snapshot() -> Response:
        frame = reader.snapshot()
        if frame is None:
            return Response("尚未收到视频帧", status=503, content_type="text/plain; charset=utf-8")
        return Response(frame, mimetype="image/jpeg", headers={"Cache-Control": "no-store"})

    @app.get("/api/status")
    def status() -> Response:
        return jsonify(reader.status())

    return app


def main() -> int:
    load_env_file(Path(__file__).with_name(".env"))
    try:
        url = stream_url()
        quality = min(100, max(20, int(os.getenv("JPEG_QUALITY", "80"))))
        port = int(os.getenv("WEB_PORT", "8081"))
    except ValueError as exc:
        print(f"配置错误：{exc}", file=sys.stderr)
        return 2

    host = os.getenv("WEB_HOST", "127.0.0.1")
    reader = RtspReader(url, quality)
    reader.start()
    atexit.register(reader.stop)
    print(f"本地视频页面：http://{host}:{port}", flush=True)
    if host not in {"127.0.0.1", "localhost", "::1"}:
        print("警告：Web 页面没有登录认证，请勿直接暴露到公网。", file=sys.stderr)
    create_app(reader).run(host=host, port=port, threaded=True, use_reloader=False)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
