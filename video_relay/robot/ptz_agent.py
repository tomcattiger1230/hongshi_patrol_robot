#!/usr/bin/env python3
"""Token-protected ONVIF PTZ agent intended to run on the robot NUC."""

from __future__ import annotations

import base64
import hashlib
import hmac
import json
import os
import secrets
import signal
import sys
import threading
from dataclasses import dataclass
from datetime import datetime, timezone
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen
from xml.sax.saxutils import escape

try:
    from .push_stream import load_env_file
except ImportError:
    from push_stream import load_env_file


DIRECTIONS = {
    "up": (0.0, 1.0, 0.0),
    "down": (0.0, -1.0, 0.0),
    "left": (-1.0, 0.0, 0.0),
    "right": (1.0, 0.0, 0.0),
    "zoom_in": (0.0, 0.0, 1.0),
    "zoom_out": (0.0, 0.0, -1.0),
}


@dataclass(frozen=True)
class PtzConfig:
    service_url: str
    username: str
    password: str
    profile_token: str
    api_token: str
    request_timeout: float = 3.0
    deadman_timeout: float = 1.5


def required(name: str) -> str:
    value = os.getenv(name, "").strip()
    if not value:
        raise ValueError(f"缺少配置项 {name}")
    return value


def load_config() -> PtzConfig:
    camera_url = urlsplit(required("CAMERA_RTSP_URL"))
    if not camera_url.hostname:
        raise ValueError("CAMERA_RTSP_URL 中缺少摄像头地址")
    onvif_host = os.getenv("CAMERA_ONVIF_HOST", camera_url.hostname).strip()
    onvif_port = int(os.getenv("CAMERA_ONVIF_PORT", "80"))
    request_timeout = float(os.getenv("PTZ_REQUEST_TIMEOUT", "3"))
    deadman_timeout = float(os.getenv("PTZ_DEADMAN_SECONDS", "1.5"))
    if request_timeout <= 0:
        raise ValueError("PTZ_REQUEST_TIMEOUT 必须大于 0")
    if not 0.5 <= deadman_timeout <= 10:
        raise ValueError("PTZ_DEADMAN_SECONDS 必须在 0.5 到 10 之间")
    return PtzConfig(
        service_url=f"http://{onvif_host}:{onvif_port}/onvif/PTZ",
        username=required("CAMERA_USER"),
        password=required("CAMERA_PASSWORD"),
        profile_token=os.getenv("CAMERA_PROFILE_TOKEN", "Profile_101").strip(),
        api_token=required("PTZ_TOKEN"),
        request_timeout=request_timeout,
        deadman_timeout=deadman_timeout,
    )


def ws_security_header(config: PtzConfig) -> str:
    created = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"
    nonce = secrets.token_bytes(24)
    nonce_b64 = base64.b64encode(nonce).decode("ascii")
    digest = hashlib.sha1(nonce + created.encode() + config.password.encode()).digest()
    password_digest = base64.b64encode(digest).decode("ascii")
    return f"""<s:Header>
      <wsse:Security s:mustUnderstand="1"
        xmlns:wsse="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-secext-1.0.xsd"
        xmlns:wsu="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd">
        <wsse:UsernameToken>
          <wsse:Username>{escape(config.username)}</wsse:Username>
          <wsse:Password Type="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-username-token-profile-1.0#PasswordDigest">{password_digest}</wsse:Password>
          <wsse:Nonce EncodingType="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-soap-message-security-1.0#Base64Binary">{nonce_b64}</wsse:Nonce>
          <wsu:Created>{created}</wsu:Created>
        </wsse:UsernameToken>
      </wsse:Security>
    </s:Header>"""


def soap_envelope(config: PtzConfig, body: str) -> bytes:
    return f"""<?xml version="1.0" encoding="utf-8"?>
<s:Envelope xmlns:s="http://www.w3.org/2003/05/soap-envelope"
  xmlns:tptz="http://www.onvif.org/ver20/ptz/wsdl"
  xmlns:tt="http://www.onvif.org/ver10/schema">
  {ws_security_header(config)}
  <s:Body>{body}</s:Body>
</s:Envelope>""".encode()


def direction_velocity(direction: str, speed: float) -> tuple[float, float, float]:
    if direction not in DIRECTIONS:
        raise ValueError("未知 PTZ 方向")
    if not 0.05 <= speed <= 1.0:
        raise ValueError("PTZ 速度必须在 0.05 到 1.0 之间")
    pan, tilt, zoom = DIRECTIONS[direction]
    return pan * speed, tilt * speed, zoom * speed


class PtzController:
    def __init__(self, config: PtzConfig) -> None:
        self.config = config
        self._timer: threading.Timer | None = None
        self._generation = 0
        self._lock = threading.Lock()

    def _request(self, body: str) -> None:
        request = Request(
            self.config.service_url,
            data=soap_envelope(self.config, body),
            headers={"Content-Type": "application/soap+xml; charset=utf-8"},
            method="POST",
        )
        try:
            with urlopen(request, timeout=self.config.request_timeout) as response:
                if response.status not in {200, 202, 204}:
                    raise RuntimeError(f"ONVIF 返回 HTTP {response.status}")
        except HTTPError as exc:
            raise RuntimeError(f"ONVIF 返回 HTTP {exc.code}") from exc
        except (TimeoutError, URLError) as exc:
            raise RuntimeError("无法连接摄像头 ONVIF 服务") from exc

    def move(self, direction: str, speed: float) -> None:
        pan, tilt, zoom = direction_velocity(direction, speed)
        profile = escape(self.config.profile_token)
        self._request(f"""<tptz:ContinuousMove>
          <tptz:ProfileToken>{profile}</tptz:ProfileToken>
          <tptz:Velocity>
            <tt:PanTilt x="{pan:.3f}" y="{tilt:.3f}"/>
            <tt:Zoom x="{zoom:.3f}"/>
          </tptz:Velocity>
        </tptz:ContinuousMove>""")
        with self._lock:
            self._generation += 1
            generation = self._generation
            if self._timer is not None:
                self._timer.cancel()
            self._timer = threading.Timer(
                self.config.deadman_timeout,
                self._deadman_stop,
                args=(generation,),
            )
            self._timer.daemon = True
            self._timer.start()

    def stop(self) -> None:
        with self._lock:
            self._generation += 1
            if self._timer is not None:
                self._timer.cancel()
                self._timer = None
        self._send_stop()

    def _send_stop(self) -> None:
        profile = escape(self.config.profile_token)
        self._request(f"""<tptz:Stop>
          <tptz:ProfileToken>{profile}</tptz:ProfileToken>
          <tptz:PanTilt>true</tptz:PanTilt>
          <tptz:Zoom>true</tptz:Zoom>
        </tptz:Stop>""")

    def _deadman_stop(self, generation: int) -> None:
        with self._lock:
            if generation != self._generation:
                return
            self._generation += 1
            self._timer = None
        try:
            self._send_stop()
        except RuntimeError as exc:
            print(f"PTZ 自动停止失败：{exc}", file=sys.stderr, flush=True)


def make_handler(controller: PtzController) -> type[BaseHTTPRequestHandler]:
    class PtzRequestHandler(BaseHTTPRequestHandler):
        server_version = "RobotPtzAgent/1.0"

        def log_message(self, message: str, *args: object) -> None:
            print(f"PTZ {self.client_address[0]} - {message % args}", flush=True)

        def _authorized(self) -> bool:
            expected = f"Bearer {controller.config.api_token}"
            return hmac.compare_digest(self.headers.get("Authorization", ""), expected)

        def _json(self, status: int, payload: dict[str, Any]) -> None:
            data = json.dumps(payload, ensure_ascii=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def _read_json(self) -> dict[str, Any]:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > 4096:
                raise ValueError("无效的请求长度")
            payload = json.loads(self.rfile.read(length))
            if not isinstance(payload, dict):
                raise ValueError("JSON 请求必须是对象")
            return payload

        def do_GET(self) -> None:  # noqa: N802
            if self.path != "/health":
                self._json(HTTPStatus.NOT_FOUND, {"status": "error", "message": "not found"})
            elif not self._authorized():
                self._json(HTTPStatus.UNAUTHORIZED, {"status": "error", "message": "unauthorized"})
            else:
                self._json(HTTPStatus.OK, {"status": "ok", "message": "PTZ Agent 已连接"})

        def do_POST(self) -> None:  # noqa: N802
            if not self._authorized():
                self._json(HTTPStatus.UNAUTHORIZED, {"status": "error", "message": "unauthorized"})
                return
            try:
                if self.path == "/move":
                    payload = self._read_json()
                    controller.move(str(payload.get("direction", "")), float(payload.get("speed", 0.3)))
                    self._json(HTTPStatus.OK, {"status": "ok", "message": "PTZ 移动中"})
                elif self.path == "/stop":
                    controller.stop()
                    self._json(HTTPStatus.OK, {"status": "ok", "message": "PTZ 已停止"})
                else:
                    self._json(HTTPStatus.NOT_FOUND, {"status": "error", "message": "not found"})
            except (ValueError, json.JSONDecodeError) as exc:
                self._json(HTTPStatus.BAD_REQUEST, {"status": "error", "message": str(exc)})
            except RuntimeError as exc:
                self._json(HTTPStatus.BAD_GATEWAY, {"status": "error", "message": str(exc)})

    return PtzRequestHandler


def main() -> int:
    load_env_file(Path(__file__).with_name(".env"))
    try:
        config = load_config()
        host = os.getenv("PTZ_BIND_HOST", "0.0.0.0")
        port = int(os.getenv("PTZ_AGENT_PORT", "8090"))
    except (ValueError, OverflowError) as exc:
        print(f"配置错误：{exc}", file=sys.stderr)
        return 2

    try:
        server = ThreadingHTTPServer((host, port), make_handler(PtzController(config)))
    except OSError as exc:
        print(f"无法启动 PTZ Agent：{exc}", file=sys.stderr)
        return 2

    def request_shutdown(*_args: object) -> None:
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, request_shutdown)
    print(f"PTZ Agent 已启动：http://{host}:{port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
