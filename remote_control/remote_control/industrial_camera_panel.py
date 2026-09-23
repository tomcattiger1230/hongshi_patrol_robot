"""On-demand still capture from two onboard Hikrobot USB cameras over SSH."""

from __future__ import annotations

import os
import ipaddress
import json
from pathlib import Path
import re
import socket
import time

from PySide6.QtCore import QProcess, QTimer, Qt
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QComboBox,
    QFileDialog,
    QFrame,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)


LINKS = (("自动（局域网优先）", "auto"), ("局域网", "lan"), ("公网", "cloud"))
CAMERA_NAMES = {1: "右相机", 2: "左相机"}
SESSION_RE = re.compile(r"^\d{8}_\d{6}(?:_\d{2})?$")


def available_loopback_port() -> int:
    with socket.socket() as candidate:
        candidate.bind(("127.0.0.1", 0))
        return candidate.getsockname()[1]


def same_lan_ipv4(host: str) -> tuple[bool, str, str]:
    """Conservatively detect a directly routed private /24 IPv4 peer."""
    try:
        addresses = socket.getaddrinfo(host, 22, socket.AF_INET, socket.SOCK_STREAM)
        remote = next(ipaddress.ip_address(item[4][0]) for item in addresses)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.connect((str(remote), 22))
            local = ipaddress.ip_address(probe.getsockname()[0])
        same = local.is_private and remote.is_private and remote in ipaddress.ip_network(f"{local}/24", strict=False)
        return same, str(local), str(remote)
    except (OSError, StopIteration, ValueError):
        return False, "", ""


class IndustrialCameraPanel(QWidget):
    def __init__(self, parent=None, auto_tunnel=True, transport="remote"):
        super().__init__(parent)
        if transport not in {"remote", "local"}:
            raise ValueError("industrial camera transport must be remote or local")
        self.transport = transport
        self.local_mode = transport == "local"
        self.cloud_host = os.getenv("ROBOT_CLOUD_SSH_HOST", "hsjc_ecs")
        configured_port = os.getenv("ROBOT_INDUSTRIAL_WAN_SSH_PORT", "").strip()
        self.local_port = int(configured_port) if configured_port else available_loopback_port()
        self.cloud_reverse_port = int(os.getenv("ROBOT_CLOUD_REVERSE_PORT", "12220"))
        # The router assigns a changing DHCP address. macOS resolves the NUC's
        # Avahi/mDNS hostname to its current Wi-Fi address automatically.
        self.lan_host = os.getenv("ROBOT_LAN_SSH_HOST", "hs-nuc14lnk.local")
        self.host_key_alias = os.getenv("ROBOT_SSH_HOST_KEY_ALIAS", "192.168.88.108")
        self.tunnel = QProcess(self)
        self.capture_processes = {}
        self.capture_routes = {}
        self.record_process = None
        self.record_routes = []
        self.record_action = ""
        self.recording = False
        self.session_items = []
        self.download_process = None
        self.images = {}
        self.statuses = {}
        self.buttons = {}
        self._shutting_down = False

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        content = QWidget()
        root = QVBoxLayout(content)
        scroll.setWidget(content)
        outer.addWidget(scroll)
        self.scroll_area = scroll
        heading_row = QHBoxLayout()
        heading = QLabel("海康 USB 工业相机 · NUC 本地录像与按需静态抓图")
        heading.setStyleSheet("font-size: 17px; font-weight: 600;")
        heading_row.addWidget(heading)
        heading_row.addStretch()
        heading_row.addWidget(QLabel("传输链路"))
        self.link_selector = QComboBox()
        for label, value in LINKS:
            self.link_selector.addItem(label, value)
        if self.local_mode:
            self.link_selector.clear()
            self.link_selector.addItem("NUC 本机", "local")
            self.link_selector.setEnabled(False)
        self.link_selector.currentIndexChanged.connect(self._link_changed)
        heading_row.addWidget(self.link_selector)
        root.addLayout(heading_row)

        self.connection = QLabel(
            "NUC 本机模式：直接调用 MVS 相机与本地录像程序，不建立 SSH 或云端连接"
            if self.local_mode else
            "自动模式：同网时优先直连 NUC，失败后经云端 Wi-Fi/SIM 安全通道"
        )
        self.connection.setWordWrap(True)
        root.addWidget(self.connection)
        feeds = QHBoxLayout()
        self.camera_order = (2, 1)
        for camera in self.camera_order:
            card = QFrame()
            card.setStyleSheet("QFrame { background:white; border:1px solid #dce4ee; border-radius:10px; }")
            column = QVBoxLayout(card)
            title = QLabel(CAMERA_NAMES[camera])
            title.setStyleSheet("font-size:15px;font-weight:600;")
            column.addWidget(title)
            image = QLabel("尚未请求静态图像")
            image.setAlignment(Qt.AlignmentFlag.AlignCenter)
            image.setMinimumSize(320, 260)
            image.setStyleSheet("background:#111c2c;color:#a8bad0;border-radius:7px;")
            column.addWidget(image, 1)
            button = QPushButton("获取当前图像")
            button.clicked.connect(lambda _checked=False, c=camera: self.capture(c))
            column.addWidget(button)
            status = QLabel("相机只会在点击后打开、等待自动曝光稳定并采集一帧")
            status.setWordWrap(True)
            column.addWidget(status)
            self.images[camera], self.buttons[camera], self.statuses[camera] = image, button, status
            feeds.addWidget(card, 1)
        root.addLayout(feeds, 1)

        recorder = QFrame()
        recorder.setStyleSheet("QFrame { background:white; border:1px solid #dce4ee; border-radius:10px; }")
        recorder_layout = QVBoxLayout(recorder)
        recorder_title = QLabel("NUC 本地录像")
        recorder_title.setStyleSheet("font-size:15px;font-weight:600;")
        recorder_layout.addWidget(recorder_title)
        settings = QHBoxLayout()
        settings.addWidget(QLabel("相机"))
        self.record_camera = QComboBox()
        self.record_camera.addItem("左右相机同时录制", "both")
        self.record_camera.addItem("仅右相机", "right")
        self.record_camera.addItem("仅左相机", "left")
        settings.addWidget(self.record_camera)
        settings.addWidget(QLabel("帧率"))
        self.record_fps = QComboBox()
        for fps in (1, 2, 5, 10):
            self.record_fps.addItem(f"{fps} fps", fps)
        self.record_fps.setCurrentIndex(2)
        settings.addWidget(self.record_fps)
        settings.addWidget(QLabel("分辨率"))
        self.record_resolution = QComboBox()
        for width, height in ((4096, 2460), (2048, 1230), (1280, 768), (1024, 615)):
            self.record_resolution.addItem(f"{width}×{height}", (width, height))
        self.record_resolution.setCurrentIndex(1)
        settings.addWidget(self.record_resolution)
        settings.addWidget(QLabel("时长"))
        self.record_duration = QSpinBox()
        self.record_duration.setRange(5, 28800)
        self.record_duration.setValue(60)
        self.record_duration.setSuffix(" 秒")
        settings.addWidget(self.record_duration)
        self.record_start = QPushButton("启动录制")
        self.record_stop = QPushButton("停止录制")
        self.record_stop.setEnabled(False)
        self.record_start.clicked.connect(lambda: self.run_record_action("start"))
        self.record_stop.clicked.connect(lambda: self.run_record_action("stop"))
        settings.addWidget(self.record_start)
        settings.addWidget(self.record_stop)
        settings.addStretch()
        recorder_layout.addLayout(settings)
        self.record_status = QLabel("录像文件仅保存在 NUC：~/haikang_records/日期_时间/；未上传到 GUI")
        self.record_status.setWordWrap(True)
        recorder_layout.addWidget(self.record_status)
        root.addWidget(recorder)

        records = QFrame()
        records.setStyleSheet("QFrame { background:white; border:1px solid #dce4ee; border-radius:10px; }")
        records_layout = QVBoxLayout(records)
        records_heading = QHBoxLayout()
        records_heading.addWidget(QLabel("局域网录像回传"))
        self.records_refresh = QPushButton("检测同网并刷新记录")
        self.records_refresh.clicked.connect(self.refresh_records)
        records_heading.addWidget(self.records_refresh)
        self.records_download = QPushButton(
            "打开所选文件夹" if self.local_mode else "下载所选文件夹到 Mac…"
        )
        self.records_download.setEnabled(False)
        self.records_download.clicked.connect(self.download_record)
        records_heading.addWidget(self.records_download)
        records_heading.addStretch()
        records_layout.addLayout(records_heading)
        self.records_list = QListWidget()
        self.records_list.setMaximumHeight(110)
        self.records_list.itemSelectionChanged.connect(
            lambda: self.records_download.setEnabled(bool(self.records_list.selectedItems()) and self.download_process is None)
        )
        records_layout.addWidget(self.records_list)
        self.records_status = QLabel(
            "录像位于本机 ~/haikang_records；选择记录后可打开对应文件夹。"
            if self.local_mode else
            "仅当 GUI 与 NUC 位于同一局域网并可直接 SSH 时允许下载；不会经云端回传录像。"
        )
        self.records_status.setWordWrap(True)
        records_layout.addWidget(self.records_status)
        root.addWidget(records)
        note = QLabel(
            "自动模式优先尝试局域网，失败后使用 NUC→云端→Mac；公网不开放机器人 SSH。"
            "每次只返回一张 JPEG，完成后立即关闭相机。链路选择不会改变相机曝光或画面亮度。"
        )
        note.setWordWrap(True)
        root.addWidget(note)

        self.record_timer = QTimer(self)
        self.record_timer.setInterval(1000)
        self.record_timer.timeout.connect(lambda: self.run_record_action("status"))

        self.tunnel.started.connect(lambda: QTimer.singleShot(500, self._tunnel_ready))
        self.tunnel.errorOccurred.connect(lambda _error: self._tunnel_failed())
        self.tunnel.finished.connect(self._tunnel_finished)
        if auto_tunnel and not self.local_mode:
            self.start_tunnel()
        elif self.local_mode:
            self.connection.setText("NUC 本机相机控制已就绪 · 无 SSH/云端链路")
        else:
            self.connection.setText("测试模式：未建立公网连接；局域网抓图仍可按需尝试")
        self._update_buttons()

    @property
    def link_mode(self):
        return self.link_selector.currentData()

    def _link_changed(self, _index):
        if self.local_mode:
            return
        messages = {
            "auto": "自动模式：同网时优先直连 NUC，失败后经云端 Wi-Fi/SIM 安全通道",
            "lan": f"局域网模式：直接连接 NUC {self.lan_host}，不会回退公网",
            "cloud": "公网模式：经云服务器选择 Wi-Fi 或 SIM，不要求 Mac 与 NUC 同网",
        }
        self.connection.setText(messages[self.link_mode])
        if self.link_mode in ("auto", "cloud"):
            self.start_tunnel()
        self._update_buttons()

    def start_tunnel(self):
        if self.local_mode:
            return
        if self._shutting_down or self.tunnel.state() != QProcess.ProcessState.NotRunning:
            return
        self.tunnel.setProgram("ssh")
        self.tunnel.setArguments([
            "-N", "-T", "-o", "BatchMode=yes", "-o", "ExitOnForwardFailure=yes",
            "-o", "ServerAliveInterval=20", "-o", "ServerAliveCountMax=3",
            "-L", f"127.0.0.1:{self.local_port}:127.0.0.1:{self.cloud_reverse_port}",
            self.cloud_host,
        ])
        self.tunnel.start()

    def _tunnel_ready(self):
        if self.tunnel.state() == QProcess.ProcessState.Running:
            if self.link_mode == "cloud":
                self.connection.setText("公网安全通道已建立 · 等待按需抓图")
            self._update_buttons()

    def _tunnel_finished(self, *_args):
        self._tunnel_failed()
        if not self._shutting_down:
            QTimer.singleShot(2000, self.start_tunnel)

    def _tunnel_failed(self):
        if self.link_mode == "cloud":
            self.connection.setText("公网安全通道不可用，正在自动重连……")
        elif self.link_mode == "auto":
            self.connection.setText("公网通道暂不可用；自动模式仍可尝试局域网，公网正在重连")
        self._update_buttons()

    def _update_buttons(self):
        cloud_ready = self.tunnel.state() == QProcess.ProcessState.Running
        for camera, button in self.buttons.items():
            idle = camera not in self.capture_processes and camera not in self.capture_routes and not self.recording
            button.setEnabled(idle and (self.local_mode or self.link_mode != "cloud" or cloud_ready))
        action_idle = self.record_process is None
        route_ready = self.local_mode or self.link_mode != "cloud" or cloud_ready
        self.record_start.setEnabled(action_idle and route_ready and not self.recording)
        self.record_stop.setEnabled(action_idle and route_ready and self.recording)

    def capture_arguments(self, camera, link):
        if camera not in (1, 2) or link not in ("local", "lan", "cloud"):
            raise ValueError("unsupported camera or link")
        command = [
            "/home/hs/robot320_remote_spatial/hik_usb_snapshot.py",
            "--camera", str(camera), "--quality", "75",
        ]
        if link == "local":
            return command
        port = "22" if link == "lan" else str(self.local_port)
        target = "hs@" + self.lan_host if link == "lan" else "hs@127.0.0.1"
        return [
            "-p", port,
            "-o", f"HostKeyAlias={self.host_key_alias}",
            "-o", "StrictHostKeyChecking=yes", "-o", "BatchMode=yes",
            "-o", "ConnectTimeout=8",
            target,
        ] + command

    def ssh_prefix(self, link):
        if link == "local":
            return []
        if link not in ("lan", "cloud"):
            raise ValueError("unsupported link")
        port = "22" if link == "lan" else str(self.local_port)
        target = "hs@" + self.lan_host if link == "lan" else "hs@127.0.0.1"
        return [
            "-p", port, "-o", f"HostKeyAlias={self.host_key_alias}",
            "-o", "StrictHostKeyChecking=yes", "-o", "BatchMode=yes",
            "-o", "ConnectTimeout=8", target,
        ]

    def record_arguments(self, action, link):
        if action not in ("start", "stop", "status", "list"):
            raise ValueError("unsupported recorder action")
        arguments = self.ssh_prefix(link) + [
            "/home/hs/robot320_remote_spatial/hik_usb_record.py", action,
        ]
        if action == "start":
            width, height = self.record_resolution.currentData()
            arguments += [
                "--camera", self.record_camera.currentData(),
                "--fps", str(self.record_fps.currentData()),
                "--width", str(width), "--height", str(height),
                "--duration", str(self.record_duration.value()),
            ]
        elif action == "status":
            arguments.append("--include-preview")
        return arguments

    def run_record_action(self, action, force_lan=False):
        if self.record_process is not None:
            return
        if self.local_mode:
            routes = ["local"]
        elif action == "list":
            routes = ["lan"]
        elif force_lan:
            routes = ["lan"]
        else:
            routes = {"auto": ["lan", "cloud"], "lan": ["lan"], "cloud": ["cloud"]}[self.link_mode]
        self.record_routes = routes.copy()
        self.record_action = action
        self._try_record_action()

    def _try_record_action(self):
        if self._shutting_down:
            return
        if not self.record_routes:
            if self.record_action == "list":
                self.records_status.setText("未检测到可用的同一局域网直连，录像不会通过云端下载。")
            else:
                self.record_status.setText("录像操作失败：所选连接链路不可用")
            self.record_action = ""
            self._update_buttons()
            return
        link = self.record_routes.pop(0)
        if link == "cloud" and self.tunnel.state() != QProcess.ProcessState.Running:
            self.start_tunnel()
            self.record_routes.insert(0, link)
            QTimer.singleShot(1000, self._try_record_action)
            return
        process = QProcess(self)
        process.setProgram(self.record_arguments(self.record_action, link)[0] if link == "local" else "ssh")
        arguments = self.record_arguments(self.record_action, link)
        process.setArguments(arguments[1:] if link == "local" else arguments)
        process.finished.connect(
            lambda code, _status, p=process, route=link: self._record_finished(p, code, route)
        )
        self.record_process = process
        if self.record_action == "start":
            self.record_status.setText("正在请求 NUC 创建录像会话……")
        elif self.record_action == "stop":
            self.record_status.setText("正在停止录像并封装 MP4 文件……")
        process.start()
        QTimer.singleShot(20000, lambda p=process: IndustrialCameraPanel._kill_if_running(p))
        self._update_buttons()

    def _record_finished(self, process, code, link):
        action = self.record_action
        output = bytes(process.readAllStandardOutput())
        error = bytes(process.readAllStandardError()).decode(errors="replace").strip()
        self.record_process = None
        process.deleteLater()
        try:
            data = json.loads(output)
            valid = code == 0 and isinstance(data, dict)
        except (UnicodeDecodeError, ValueError):
            data, valid = {}, False
        if not valid and self.record_routes:
            QTimer.singleShot(0, self._try_record_action)
            return
        self.record_routes = []
        self.record_action = ""
        if not valid:
            target = self.records_status if action == "list" else self.record_status
            target.setText("操作失败：" + (error[-220:] or "NUC 未返回有效状态"))
            self._update_buttons()
            return
        if action == "list":
            self._show_sessions(data, link)
            self._update_buttons()
            return
        self._show_record_status(data, link)
        self._update_buttons()

    def _show_record_status(self, data, link):
        self.recording = data.get("recording") is True
        session = data.get("session") or "—"
        workers = data.get("workers") if isinstance(data.get("workers"), list) else []
        summaries = []
        for worker in workers:
            camera = worker.get("camera")
            label = {"right": "右", "left": "左"}.get(camera, camera)
            summaries.append(
                f'{label}:{worker.get("state", "?")} {worker.get("frames", 0)}帧/{worker.get("elapsed", 0):.1f}s'
            )
            preview_data = worker.get("preview_jpeg")
            if camera in ("right", "left") and isinstance(preview_data, str):
                pixmap = QPixmap()
                try:
                    decoded = __import__("base64").b64decode(preview_data, validate=True)
                except (ValueError, TypeError):
                    decoded = b""
                if len(decoded) <= 1024 * 1024 and pixmap.loadFromData(decoded, "JPEG"):
                    index = 1 if camera == "right" else 2
                    self.images[index].setPixmap(pixmap.scaled(
                        self.images[index].size(), Qt.AspectRatioMode.KeepAspectRatio,
                        Qt.TransformationMode.SmoothTransformation,
                    ))
                    self.statuses[index].setText("录像确认预览（最高 1 Hz，不代表录像原始分辨率）")
        route = {"local": "NUC本机", "lan": "局域网", "cloud": "公网"}.get(link, link)
        state = "录制中" if self.recording else "未在录制"
        self.record_status.setText(
            f"{state} · 会话 {session} · {route}状态确认" + (" · " + "；".join(summaries) if summaries else "")
        )
        if self.recording:
            if not self.record_timer.isActive():
                self.record_timer.start()
        else:
            self.record_timer.stop()

    def refresh_records(self):
        if self.local_mode:
            self.records_status.setText("正在读取 NUC 本机录像目录……")
            self.run_record_action("list")
            return
        same, local, remote = same_lan_ipv4(self.lan_host)
        self.records_download.setEnabled(False)
        if not same:
            self.records_list.clear()
            self.records_status.setText("未检测到 NUC 与 GUI 位于同一局域网；录像列表和下载已禁用。")
            return
        self.records_status.setText(f"检测到同网直连 {local} → {remote}，正在读取 NUC 录像目录……")
        self.run_record_action("list", force_lan=True)

    def _show_sessions(self, data, link):
        self.records_list.clear()
        sessions = data.get("sessions") if isinstance(data.get("sessions"), list) else []
        for session in sessions:
            name = session.get("name", "")
            if not SESSION_RE.fullmatch(name):
                continue
            size = int(session.get("bytes", 0))
            files = ", ".join(item.get("name", "") for item in session.get("files", [])) or "暂无完整 MP4"
            item = QListWidgetItem(f"{name} · {size / 1024**2:.1f} MiB · {files}")
            item.setData(Qt.ItemDataRole.UserRole, name)
            self.records_list.addItem(item)
        if self.local_mode:
            self.records_status.setText(f"NUC 本机共有 {len(sessions)} 个录像会话；选择后可打开文件夹。")
        else:
            self.records_status.setText(f"局域网直连已确认 · NUC 上共 {len(sessions)} 个录像会话；选择后可下载整个文件夹。")

    def download_record(self):
        selected = self.records_list.selectedItems()
        if not selected or self.download_process is not None:
            return
        session = selected[0].data(Qt.ItemDataRole.UserRole)
        if not isinstance(session, str) or not SESSION_RE.fullmatch(session):
            return
        if self.local_mode:
            QProcess.startDetached("xdg-open", [f"/home/hs/haikang_records/{session}"])
            self.records_status.setText(f"已请求打开 NUC 本机录像文件夹 {session}。")
            return
        same, _local, _remote = same_lan_ipv4(self.lan_host)
        if not same:
            self.records_status.setText("局域网已变化，下载已取消；不会回退到云端。")
            self.records_download.setEnabled(False)
            return
        destination = QFileDialog.getExistingDirectory(self, "选择 Mac 上的录像保存目录", str(Path.home() / "Downloads"))
        if not destination:
            return
        process = QProcess(self)
        process.setProgram("scp")
        process.setArguments([
            "-r", "-q", "-P", "22", "-o", f"HostKeyAlias={self.host_key_alias}",
            "-o", "StrictHostKeyChecking=yes", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8",
            f"hs@{self.lan_host}:/home/hs/haikang_records/{session}", destination,
        ])
        process.finished.connect(lambda code, _status, p=process, name=session: self._download_finished(p, code, name))
        self.download_process = process
        self.records_download.setEnabled(False)
        self.records_status.setText(f"正在通过局域网下载 {session} 到 {destination}……")
        process.start()

    def _download_finished(self, process, code, session):
        error = bytes(process.readAllStandardError()).decode(errors="replace").strip()
        self.download_process = None
        process.deleteLater()
        if code == 0:
            self.records_status.setText(f"录像文件夹 {session} 已通过局域网下载完成。")
        else:
            self.records_status.setText("局域网下载失败：" + (error[-220:] or f"scp exit {code}"))
        self.records_download.setEnabled(bool(self.records_list.selectedItems()))

    def capture(self, camera):
        if camera not in (1, 2) or camera in self.capture_processes or camera in self.capture_routes:
            return
        routes = (
            {"local": ["local"]} if self.local_mode else
            {"auto": ["lan", "cloud"], "lan": ["lan"], "cloud": ["cloud"]}
        )
        self.capture_routes[camera] = routes[self.link_mode].copy()
        self.buttons[camera].setEnabled(False)
        self._try_capture(camera)

    def _try_capture(self, camera):
        if self._shutting_down:
            return
        routes = self.capture_routes.get(camera, [])
        if not routes:
            self.capture_routes.pop(camera, None)
            self.statuses[camera].setText("抓图失败：所选链路均不可用")
            self._update_buttons()
            return
        link = routes.pop(0)
        if link == "cloud" and self.tunnel.state() != QProcess.ProcessState.Running:
            self.start_tunnel()
            routes.insert(0, link)
            self.statuses[camera].setText("正在等待公网安全通道……")
            QTimer.singleShot(1000, lambda c=camera: self._try_capture(c))
            return
        process = QProcess(self)
        arguments = self.capture_arguments(camera, link)
        process.setProgram(arguments[0] if link == "local" else "ssh")
        process.setArguments(arguments[1:] if link == "local" else arguments)
        process.setProcessChannelMode(QProcess.ProcessChannelMode.SeparateChannels)
        process.finished.connect(
            lambda code, _status, c=camera, p=process, route=link: self._capture_finished(c, p, code, route)
        )
        self.capture_processes[camera] = process
        names = {"local": "NUC本机", "lan": "局域网", "cloud": "公网"}
        self.statuses[camera].setText(f"正在通过{names[link]}打开相机并等待自动曝光稳定……")
        process.start()
        QTimer.singleShot(
            30000,
            lambda p=process: IndustrialCameraPanel._kill_if_running(p),
        )

    @staticmethod
    def _kill_if_running(process):
        try:
            if process.state() != QProcess.ProcessState.NotRunning:
                process.kill()
        except RuntimeError:
            # The owning panel may have already destroyed the C++ QProcess.
            pass

    def _capture_finished(self, camera, process, code, link):
        data = bytes(process.readAllStandardOutput())
        error = bytes(process.readAllStandardError()).decode(errors="replace").strip()
        self.capture_processes.pop(camera, None)
        pixmap = QPixmap()
        valid = code == 0 and 1024 <= len(data) <= 20 * 1024 * 1024 and pixmap.loadFromData(data, "JPEG")
        if not valid and self.capture_routes.get(camera):
            self.statuses[camera].setText("当前链路抓图失败，正在自动尝试公网……")
            process.deleteLater()
            QTimer.singleShot(0, lambda c=camera: self._try_capture(c))
            return
        self.capture_routes.pop(camera, None)
        if valid:
            self.images[camera].setPixmap(pixmap.scaled(
                self.images[camera].size(), Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            ))
            preview = pixmap.toImage().scaled(32, 20).convertToFormat(QImage.Format.Format_Grayscale8)
            luminance = sum(
                preview.pixelColor(x, y).red()
                for y in range(preview.height()) for x in range(preview.width())
            ) / (preview.width() * preview.height())
            route_name = {"local": "NUC本机", "lan": "局域网", "cloud": "公网"}[link]
            detail = (
                f"获取时间 {time.strftime('%H:%M:%S')} · {pixmap.width()}×{pixmap.height()} · "
                f"{len(data)/1024:.0f} KiB · {route_name}"
            )
            if luminance < 5:
                detail += " · ⚠ 原始画面接近全黑，请检查镜头盖、光圈和照明"
            elif luminance < 25:
                detail += " · ⚠ 原始画面明显偏暗"
            self.statuses[camera].setText(detail)
        else:
            self.statuses[camera].setText("抓图失败：" + (error[-180:] or "无有效 JPEG 返回"))
        process.deleteLater()
        self._update_buttons()

    def shutdown(self):
        self._shutting_down = True
        for process in list(self.capture_processes.values()):
            process.kill()
        self.capture_processes.clear()
        self.capture_routes.clear()
        self.record_timer.stop()
        if self.record_process is not None:
            self.record_process.kill()
            self.record_process = None
        if self.download_process is not None:
            self.download_process.kill()
            self.download_process = None
        if self.tunnel.state() != QProcess.ProcessState.NotRunning:
            self.tunnel.terminate()
            if not self.tunnel.waitForFinished(1000):
                self.tunnel.kill()
