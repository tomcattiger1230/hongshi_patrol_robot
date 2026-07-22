#!/usr/bin/env python3
"""Expose the relayed RTSP stream as a small local MJPEG web interface."""

from __future__ import annotations

import atexit
import json
import os
import sys
import threading
import time
from pathlib import Path
from typing import Any, Iterator
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen

try:
    import cv2
    from flask import Flask, Response, jsonify, request
except ImportError as exc:
    print("缺少依赖，请执行：python3 -m pip install -r requirements.txt", file=sys.stderr)
    raise SystemExit(2) from exc

try:
    from .client import load_env_file, stream_urls
except ImportError:
    from client import load_env_file, stream_urls


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
    .panel { display: grid; grid-template-columns: 1fr auto; gap: 18px; margin-top: 14px; }
    .ptz { min-width: 260px; padding: 16px; border-radius: 8px; background: #1a2028; }
    .ptz h2 { margin: 0 0 10px; font-size: 16px; }
    .ptz-grid { display: grid; grid-template-columns: repeat(3, 64px); gap: 8px; justify-content: center; }
    .ptz button { min-height: 52px; border: 0; border-radius: 8px; color: #eef4fb;
      background: #2b3745; font-size: 20px; cursor: pointer; touch-action: none; user-select: none; }
    .ptz button:active, .ptz button.active { background: #3275b8; transform: scale(.96); }
    .ptz .stop { background: #8d3434; }
    .ptz-row { display: flex; align-items: center; gap: 10px; margin-top: 12px; }
    .ptz-row input { flex: 1; }
    #ptz-status { min-height: 20px; margin-top: 10px; color: #9aa7b5; font-size: 13px; }
    #ptz-status.ok { color: #3bc878; } #ptz-status.bad { color: #ed6b6b; }
    @media (max-width: 760px) { .panel { grid-template-columns: 1fr; } .ptz { min-width: 0; } }
    a { color: #74b7ff; }
  </style>
</head>
<body><main>
  <header><h1>机器人实时视频</h1><span id="dot"></span><span id="state">正在连接</span></header>
  <div id="detail">等待视频数据……</div>
  <div class="panel">
    <img id="video" src="/video" alt="机器人视频流">
    <section class="ptz">
      <h2>摄像头遥操作</h2>
      <div class="ptz-grid">
        <button data-direction="zoom_in" title="放大">＋</button>
        <button data-direction="up" title="向上">▲</button>
        <button data-direction="zoom_out" title="缩小">－</button>
        <button data-direction="left" title="向左">◀</button>
        <button class="stop" id="ptz-stop" title="停止">■</button>
        <button data-direction="right" title="向右">▶</button>
        <span></span><button data-direction="down" title="向下">▼</button><span></span>
      </div>
      <div class="ptz-row"><label for="ptz-speed">速度</label>
        <input id="ptz-speed" type="range" min="0.05" max="1" step="0.05" value="0.3">
        <span id="ptz-speed-value">0.30</span>
      </div>
      <div id="ptz-status">正在检查 PTZ Agent……</div>
    </section>
  </div>
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
      ? `${s.width} × ${s.height} · ${s.fps.toFixed(1)} FPS · ${s.source} · 最新帧 ${s.frame_age_seconds.toFixed(1)} 秒前`
      : '程序会自动重连，无需刷新页面';
  } catch (_) {
    document.querySelector('#dot').className = 'bad';
    document.querySelector('#state').textContent = 'Web 服务连接失败';
  }
}
updateStatus(); setInterval(updateStatus, 1000);

const ptzStatus = document.querySelector('#ptz-status');
const speedInput = document.querySelector('#ptz-speed');
let ptzMoving = false;
let ptzHeartbeat = null;
function setPtzStatus(message, state='') { ptzStatus.textContent = message; ptzStatus.className = state; }
async function ptzRequest(path, payload) {
  const options = {method: 'POST', headers: {'Content-Type': 'application/json'}, cache: 'no-store'};
  if (payload) options.body = JSON.stringify(payload);
  const response = await fetch(path, options);
  const data = await response.json();
  if (!response.ok) throw new Error(data.message || `HTTP ${response.status}`);
  return data;
}
async function startMove(button) {
  if (ptzMoving) await stopMove();
  ptzMoving = true; button.classList.add('active');
  try {
    const move = () => ptzRequest('/api/ptz/move', {
      direction: button.dataset.direction, speed: Number(speedInput.value)
    });
    await move();
    if (!ptzMoving) { await ptzRequest('/api/ptz/stop'); return; }
    ptzHeartbeat = setInterval(() => {
      if (ptzMoving) move()
        .then(() => { if (!ptzMoving) return ptzRequest('/api/ptz/stop'); })
        .catch(error => { setPtzStatus(error.message, 'bad'); stopMove(); });
    }, 600);
    setPtzStatus('移动中，松开按钮停止', 'ok');
  } catch (error) {
    ptzMoving = false; button.classList.remove('active');
    ptzRequest('/api/ptz/stop').catch(() => {});
    setPtzStatus(error.message, 'bad');
  }
}
async function stopMove() {
  document.querySelectorAll('.ptz button.active').forEach(b => b.classList.remove('active'));
  if (ptzHeartbeat) { clearInterval(ptzHeartbeat); ptzHeartbeat = null; }
  if (!ptzMoving) return;
  ptzMoving = false;
  try { await ptzRequest('/api/ptz/stop'); setPtzStatus('已停止', 'ok'); }
  catch (error) { setPtzStatus(error.message, 'bad'); }
}
document.querySelectorAll('[data-direction]').forEach(button => {
  button.addEventListener('pointerdown', event => { event.preventDefault(); button.setPointerCapture(event.pointerId); startMove(button); });
  button.addEventListener('pointerup', stopMove);
  button.addEventListener('pointercancel', stopMove);
  button.addEventListener('lostpointercapture', stopMove);
});
document.querySelector('#ptz-stop').addEventListener('click', () => { ptzMoving = true; stopMove(); });
speedInput.addEventListener('input', () => document.querySelector('#ptz-speed-value').textContent = Number(speedInput.value).toFixed(2));
window.addEventListener('blur', stopMove);
document.addEventListener('visibilitychange', () => { if (document.hidden) stopMove(); });
async function checkPtz() {
  try {
    const response = await fetch('/api/ptz/status', {cache: 'no-store'});
    const data = await response.json();
    if (!response.ok) throw new Error(data.message || 'PTZ Agent 不可用');
    if (!ptzMoving) setPtzStatus(data.message, 'ok');
  } catch (error) { if (!ptzMoving) setPtzStatus(error.message, 'bad'); }
}
checkPtz(); setInterval(checkPtz, 5000);
</script></body></html>"""


class PtzProxy:
    def __init__(self, base_url: str, token: str, timeout: float = 4.0) -> None:
        parts = urlsplit(base_url)
        if parts.scheme not in {"http", "https"} or not parts.hostname:
            raise ValueError("PTZ_AGENT_URL 必须是有效的 http:// 或 https:// 地址")
        self.base_url = base_url.rstrip("/")
        self.token = token
        self.timeout = timeout

    def request(self, path: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        data = json.dumps(payload).encode() if payload is not None else None
        request_object = Request(
            self.base_url + path,
            data=data,
            headers={
                "Authorization": f"Bearer {self.token}",
                "Content-Type": "application/json",
            },
            method="POST" if payload is not None else "GET",
        )
        try:
            with urlopen(request_object, timeout=self.timeout) as response:
                result = json.loads(response.read())
                if not isinstance(result, dict):
                    raise RuntimeError("PTZ Agent 返回了无效数据")
                return result
        except HTTPError as exc:
            try:
                message = json.loads(exc.read()).get("message", f"HTTP {exc.code}")
            except (json.JSONDecodeError, AttributeError):
                message = f"HTTP {exc.code}"
            raise RuntimeError(f"PTZ Agent：{message}") from exc
        except (TimeoutError, URLError) as exc:
            raise RuntimeError("无法连接 NUC PTZ Agent") from exc


class RtspReader:
    def __init__(self, sources: list[tuple[str, str]], jpeg_quality: int) -> None:
        if not sources:
            raise ValueError("至少需要一个 RTSP 视频源")
        self.sources = sources
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
        self.source = "尚未连接"
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

    def _open_capture(self, url: str) -> Any:
        params = [
            cv2.CAP_PROP_OPEN_TIMEOUT_MSEC,
            int(os.getenv("RTSP_OPEN_TIMEOUT_MS", "5000")),
            cv2.CAP_PROP_READ_TIMEOUT_MSEC,
            20000,
        ]
        capture = cv2.VideoCapture(url, cv2.CAP_FFMPEG, params)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return capture

    def _run(self) -> None:
        source_index = 0
        while not self.stop_event.is_set():
            source_name, source_url = self.sources[source_index]
            self._set_state("connecting", f"正在连接{source_name}视频流")
            capture = self._open_capture(source_url)
            if not capture.isOpened():
                capture.release()
                source_index = (source_index + 1) % len(self.sources)
                if source_index == 0:
                    self._set_state("error", "所有视频源均不可用，2 秒后重试")
                    self.stop_event.wait(2)
                continue

            with self.condition:
                self.source = source_name
            self._set_state("connected", f"视频已连接（{source_name}）")
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
                    self.message = f"视频已连接（{source_name}）"
                    self.condition.notify_all()
            capture.release()
            source_index = (source_index + 1) % len(self.sources)
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
                "source": self.source,
                "frame_age_seconds": round(age, 2),
                "frames_received": self.sequence,
            }

    def snapshot(self) -> bytes | None:
        with self.condition:
            return self.frame


def create_app(reader: RtspReader, ptz: PtzProxy | None = None) -> Flask:
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

    @app.get("/api/ptz/status")
    def ptz_status() -> tuple[Response, int] | Response:
        if ptz is None:
            return jsonify({"status": "error", "message": "本地尚未配置 PTZ Agent"}), 503
        try:
            return jsonify(ptz.request("/health"))
        except RuntimeError as exc:
            return jsonify({"status": "error", "message": str(exc)}), 502

    @app.post("/api/ptz/move")
    def ptz_move() -> tuple[Response, int] | Response:
        if ptz is None:
            return jsonify({"status": "error", "message": "本地尚未配置 PTZ Agent"}), 503
        payload = request.get_json(silent=True)
        if not isinstance(payload, dict):
            return jsonify({"status": "error", "message": "无效的 JSON 请求"}), 400
        direction = str(payload.get("direction", ""))
        try:
            speed = float(payload.get("speed", 0.3))
        except (TypeError, ValueError):
            return jsonify({"status": "error", "message": "无效的 PTZ 速度"}), 400
        if direction not in {"up", "down", "left", "right", "zoom_in", "zoom_out"}:
            return jsonify({"status": "error", "message": "未知 PTZ 方向"}), 400
        if not 0.05 <= speed <= 1.0:
            return jsonify({"status": "error", "message": "PTZ 速度超出范围"}), 400
        try:
            return jsonify(ptz.request("/move", {"direction": direction, "speed": speed}))
        except RuntimeError as exc:
            return jsonify({"status": "error", "message": str(exc)}), 502

    @app.post("/api/ptz/stop")
    def ptz_stop() -> tuple[Response, int] | Response:
        if ptz is None:
            return jsonify({"status": "error", "message": "本地尚未配置 PTZ Agent"}), 503
        try:
            return jsonify(ptz.request("/stop", {}))
        except RuntimeError as exc:
            return jsonify({"status": "error", "message": str(exc)}), 502

    return app


def main() -> int:
    load_env_file(Path(__file__).with_name(".env"))
    try:
        sources = stream_urls()
        quality = min(100, max(20, int(os.getenv("JPEG_QUALITY", "80"))))
        port = int(os.getenv("WEB_PORT", "8081"))
        ptz_url = os.getenv("PTZ_AGENT_URL", "").strip()
        ptz_token = os.getenv("PTZ_TOKEN", "").strip()
        ptz = PtzProxy(ptz_url, ptz_token) if ptz_url and ptz_token else None
    except ValueError as exc:
        print(f"配置错误：{exc}", file=sys.stderr)
        return 2

    host = os.getenv("WEB_HOST", "127.0.0.1")
    reader = RtspReader(sources, quality)
    reader.start()
    atexit.register(reader.stop)
    print(f"本地视频页面：http://{host}:{port}", flush=True)
    if host not in {"127.0.0.1", "localhost", "::1"}:
        print("警告：Web 页面没有登录认证，请勿直接暴露到公网。", file=sys.stderr)
    create_app(reader, ptz).run(host=host, port=port, threaded=True, use_reloader=False)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
