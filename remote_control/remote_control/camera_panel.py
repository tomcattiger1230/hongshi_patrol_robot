"""Native camera display using the existing loopback video/PTZ web bridge."""

from __future__ import annotations

import json
from urllib.parse import urlsplit

from PySide6.QtCore import QByteArray, QEvent, QTimer, QUrl, Qt
from PySide6.QtGui import QPixmap
from PySide6.QtNetwork import QNetworkAccessManager, QNetworkReply, QNetworkRequest
from PySide6.QtWidgets import (
    QApplication, QComboBox, QDoubleSpinBox, QGridLayout, QHBoxLayout, QLabel,
    QFrame, QPushButton, QSizePolicy, QVBoxLayout, QWidget,
)


def camera_bridge_url(value: str) -> str:
    """Keep credentials and remote HTTP endpoints out of the desktop widget."""
    parts = urlsplit(value)
    if (
        parts.scheme != "http"
        or parts.hostname not in {"127.0.0.1", "localhost", "::1"}
        or parts.username is not None
        or parts.password is not None
        or parts.path not in {"", "/"}
        or parts.query or parts.fragment
    ):
        raise ValueError("camera URL must be a loopback HTTP origin, e.g. http://127.0.0.1:8081")
    # Validate the port without printing any credentials from the input.
    _ = parts.port
    return value.rstrip("/")


class CameraPanel(QWidget):
    def __init__(self, bridge_url: str = "", parent=None):
        super().__init__(parent)
        self.base_url = camera_bridge_url(bridge_url) if bridge_url else ""
        self.network = QNetworkAccessManager(self)
        self._closed = False
        self._frame_pending = False
        self._second_frame_pending = False
        self._move_pending = False
        self._direction = ""
        self._generation = 0
        self._pixmap = QPixmap()
        self._second_pixmap = QPixmap()
        self._link_status_pending = False
        self._ptz_status_pending = False
        self._link_switch_pending = set()
        self.link_selectors = {}
        self.link_labels = {}

        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(12)
        self.setObjectName("cameraPanel")
        self.setStyleSheet("""
            QWidget#cameraPanel { background: #f3f6fa; color: #223047; }
            QFrame#cameraCard, QFrame#ptzCard { background: white; border: 1px solid #dce4ee; border-radius: 12px; }
            QLabel { color: #334155; background: transparent; border: none; }
            QLabel#cameraTitle { font-size: 16px; font-weight: 600; }
            QLabel#videoSurface { background: #101923; color: #94a3b8; border-radius: 8px; }
            QComboBox, QDoubleSpinBox { background: white; color: #223047; border: 1px solid #cbd5e1; border-radius: 6px; padding: 6px; min-height: 22px; }
            QPushButton { background: #edf3fc; color: #244978; border: 1px solid #cedcee; border-radius: 6px; padding: 8px 12px; min-height: 20px; }
            QPushButton:hover { background: #dbeafe; }
            QPushButton:pressed { background: #bfdbfe; }
            QPushButton:disabled { color: #94a3b8; background: #f1f5f9; }
        """)
        self.status = QLabel("摄像头未启用。启动本地视频桥接后，使用 --camera-url 指定地址。")
        self.status.setWordWrap(True)
        self.status.setFixedHeight(24)
        layout.addWidget(self.status)
        feeds = QHBoxLayout()
        feeds.setSpacing(16)
        self.feed_status = {}
        images = []
        for channel in (1, 2):
            card = QFrame()
            card.setObjectName("cameraCard")
            card.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Expanding)
            column = QVBoxLayout(card)
            column.setContentsMargins(14, 14, 14, 14)
            column.setSpacing(10)
            title = QLabel(f"通道 {channel}")
            title.setObjectName("cameraTitle")
            title.setFixedHeight(24)
            column.addWidget(title)
            self._add_link_selector(column, channel)
            image = QLabel("等待实时画面")
            image.setObjectName("videoSurface")
            image.setAlignment(Qt.AlignmentFlag.AlignCenter)
            image.setMinimumSize(160, 180)
            # A decoded pixmap must never become the layout's size hint.
            image.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
            column.addWidget(image, 1)
            info = QLabel("等待视频 · 分辨率待确认")
            info.setFixedHeight(24)
            column.addWidget(info)
            self.feed_status[channel] = info
            images.append(image)
            feeds.addWidget(card, 1)
        self.image, self.second_image = images
        self.second_status = self.feed_status[2]
        layout.addLayout(feeds, 1)

        control_card = QFrame()
        control_card.setObjectName("ptzCard")
        controls = QHBoxLayout(control_card)
        controls.setContentsMargins(14, 14, 14, 14)
        controls.setSpacing(20)
        pad = QGridLayout()
        pad.setSpacing(8)
        self.buttons = []
        for text, direction, row, col in [
            ("放大 +", "zoom_in", 0, 0), ("上 ↑", "up", 0, 1),
            ("缩小 −", "zoom_out", 0, 2), ("左 ←", "left", 1, 0),
            ("右 →", "right", 1, 2), ("下 ↓", "down", 2, 1),
        ]:
            button = QPushButton(text)
            button.pressed.connect(lambda d=direction: self.start_move(d))
            button.released.connect(self.stop_move)
            pad.addWidget(button, row, col)
            self.buttons.append(button)
        stop = QPushButton("云台停止")
        stop.clicked.connect(self.stop_move)
        pad.addWidget(stop, 1, 1)
        self.buttons.append(stop)
        controls.addLayout(pad)
        controls.addWidget(QLabel("云台速度"))
        self.speed = QDoubleSpinBox()
        self.speed.setRange(0.05, 1.0)
        self.speed.setSingleStep(0.05)
        self.speed.setValue(0.3)
        controls.addWidget(self.speed)
        controls.addStretch(1)
        layout.addWidget(control_card)
        self.ptz_status = QLabel("按住云台方向键移动，松开或切出应用停止；仅控制摄像头，不控制车辆。")
        self.ptz_status.setWordWrap(True)
        layout.addWidget(self.ptz_status)
        for button in self.buttons:
            button.setEnabled(bool(self.base_url))

        self.frame_timer = QTimer(self)
        self.frame_timer.setInterval(100)
        self.frame_timer.timeout.connect(self.fetch_frame)
        self.frame_timer.timeout.connect(self.fetch_second_frame)
        self.move_timer = QTimer(self)
        self.move_timer.setInterval(600)
        self.move_timer.timeout.connect(self._send_move)
        self.link_timer = QTimer(self)
        self.link_timer.setInterval(1000)
        self.link_timer.timeout.connect(self.fetch_link_status)
        self.link_timer.timeout.connect(self.fetch_ptz_status)
        if self.base_url:
            self.status.setText("正在连接本地视频桥接……")
            self.frame_timer.start()
            self.link_timer.start()
        QApplication.instance().installEventFilter(self)

    def _add_link_selector(self, column, channel):
        selector = QComboBox()
        for label, mode in [("自动选择", "auto"), ("局域网", "lan"), ("云端", "cloud"), ("关闭视频", "off")]:
            selector.addItem(label, mode)
            selector.model().item(selector.count() - 1).setEnabled(mode in {"auto", "off"})
        selector.setEnabled(False)
        selector.setFixedHeight(36)
        selector.currentIndexChanged.connect(lambda _index, c=channel: self.select_link(c))
        column.addWidget(selector)
        label = QLabel("等待链路状态")
        label.setWordWrap(True)
        label.setFixedHeight(40)
        label.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Fixed)
        column.addWidget(label)
        self.link_selectors[channel] = selector
        self.link_labels[channel] = label

    def fetch_link_status(self):
        if self._closed or self._link_status_pending or not self.base_url:
            return
        self._link_status_pending = True
        remaining = {1, 2}
        for channel in (1, 2):
            reply = self._request("/api/status" + ("2" if channel == 2 else ""))

            def finished(reply=reply, channel=channel):
                if not self._closed and channel not in self._link_switch_pending:
                    if reply.error() == QNetworkReply.NetworkError.NoError:
                        try:
                            data = json.loads(bytes(reply.readAll()))
                            modes = data.get("available_modes", [])
                            selector = self.link_selectors[channel]
                            selector.setEnabled(bool(modes))
                            selector.blockSignals(True)
                            for index in range(selector.count()):
                                selector.model().item(index).setEnabled(selector.itemData(index) in modes)
                            index = selector.findData(data.get("link_mode", "auto"))
                            if index >= 0:
                                selector.setCurrentIndex(index)
                            selector.blockSignals(False)
                            if data.get("link_mode") == "off":
                                image = self.image if channel == 1 else self.second_image
                                image.setText("视频读取已关闭")
                                if channel == 1:
                                    self._pixmap = QPixmap()
                                else:
                                    self._second_pixmap = QPixmap()
                            self.link_labels[channel].setText(
                                " · ".join([data.get("message", ""), "实际链路：" + data.get("source", "尚未连接")])
                            )
                        except (ValueError, TypeError, AttributeError):
                            self.link_labels[channel].setText("链路状态格式无效")
                    else:
                        self.link_labels[channel].setText("本地视频桥接不可达")
                remaining.discard(channel)
                if not remaining:
                    self._link_status_pending = False
                reply.deleteLater()

            reply.finished.connect(finished)

    def fetch_ptz_status(self):
        if self._closed or self._ptz_status_pending or not self.base_url or self._direction:
            return
        self._ptz_status_pending = True
        reply = self._request("/api/ptz/status")

        def finished():
            self._ptz_status_pending = False
            available = reply.error() == QNetworkReply.NetworkError.NoError
            for button in self.buttons:
                button.setEnabled(available)
            if not available:
                self.ptz_status.setText("云台控制链路不可达；已禁用操作。视频链路可能仍可通过云端查看。")
            elif not self._closed:
                self.ptz_status.setText("云台链路已连接；按住方向键移动，松开停止。仅控制摄像头。")
            reply.deleteLater()

        reply.finished.connect(finished)

    def select_link(self, channel):
        if self._closed or not self.base_url:
            return
        selector = self.link_selectors[channel]
        mode = selector.currentData()
        self._link_switch_pending.add(channel)
        selector.setEnabled(False)
        self.link_labels[channel].setText("正在切换视频链路……")
        reply = self._request("/api/link", {"channel": channel, "mode": mode})

        def finished():
            self._link_switch_pending.discard(channel)
            if not self._closed:
                if reply.error() != QNetworkReply.NetworkError.NoError:
                    self.link_labels[channel].setText("链路切换失败，请检查视频桥接配置")
                selector.setEnabled(True)
                self.fetch_link_status()
            reply.deleteLater()

        reply.finished.connect(finished)

    def _request(self, path: str, payload=None):
        request = QNetworkRequest(QUrl(self.base_url + path))
        request.setTransferTimeout(3000)
        if payload is None:
            return self.network.get(request)
        request.setHeader(QNetworkRequest.KnownHeaders.ContentTypeHeader, "application/json")
        return self.network.post(request, QByteArray(json.dumps(payload).encode()))

    def fetch_frame(self):
        if self._closed or self._frame_pending or not self.base_url or self.link_selectors[1].currentData() == "off":
            return
        self._frame_pending = True
        reply = self._request("/snapshot.jpg")

        def finished():
            self._frame_pending = False
            if not self._closed and self.link_selectors[1].currentData() != "off":
                pixmap = QPixmap()
                if reply.error() == QNetworkReply.NetworkError.NoError and pixmap.loadFromData(reply.readAll()):
                    self._pixmap = pixmap
                    self._scale_image()
                    self.feed_status[1].setText(f"已连接 · {pixmap.width()} × {pixmap.height()}")
                    self.status.setText("实时监控 · 两路链路独立选择 · 云台控制仅作用于摄像头")
                else:
                    self._pixmap = QPixmap()
                    self.image.setText("视频暂不可用")
                    self.feed_status[1].setText("等待视频连接")
                    self.status.setText("未收到视频画面；请检查本地 web_viewer 和 NUC 视频代理。")
            reply.deleteLater()

        reply.finished.connect(finished)

    def _scale_image(self):
        if not self._pixmap.isNull():
            self.image.setPixmap(self._pixmap.scaled(
                self.image.size(), Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            ))
        if not self._second_pixmap.isNull():
            self.second_image.setPixmap(self._second_pixmap.scaled(
                self.second_image.size(), Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            ))

    def fetch_second_frame(self):
        if self._closed or self._second_frame_pending or not self.base_url or self.link_selectors[2].currentData() == "off":
            return
        self._second_frame_pending = True
        reply = self._request("/snapshot2.jpg")

        def finished():
            self._second_frame_pending = False
            if not self._closed and self.link_selectors[2].currentData() != "off":
                pixmap = QPixmap()
                if reply.error() == QNetworkReply.NetworkError.NoError and pixmap.loadFromData(reply.readAll()):
                    self._second_pixmap = pixmap
                    self._scale_image()
                    self.second_status.setText(f"已连接 · {pixmap.width()} × {pixmap.height()}")
                else:
                    self._second_pixmap = QPixmap()
                    self.second_image.setText("第二路暂不可用")
                    self.second_status.setText("等待第二路视频桥接")
            reply.deleteLater()

        reply.finished.connect(finished)

    def start_move(self, direction: str):
        if self._closed or not self.base_url:
            return
        self._generation += 1
        self._direction = direction
        self._send_move()
        self.move_timer.start()

    def _send_move(self):
        if not self._direction or self._move_pending or self._closed:
            return
        self._move_pending = True
        generation = self._generation
        reply = self._request("/api/ptz/move", {"direction": self._direction, "speed": self.speed.value()})

        def finished():
            self._move_pending = False
            ok = reply.error() == QNetworkReply.NetworkError.NoError
            reply.deleteLater()
            if generation != self._generation or self._closed:
                # A delayed move response must not undo a later stop.
                self._send_stop()
                return
            if ok:
                self.ptz_status.setText("云台移动中，松开按钮停止")
            else:
                self.ptz_status.setText("云台请求失败，已请求停止；NUC 另有超时自动停止保护。")
                self.stop_move()

        reply.finished.connect(finished)

    def _send_stop(self):
        if self.base_url:
            reply = self._request("/api/ptz/stop", {})

            def finished():
                if not self._closed and not self._direction:
                    self.ptz_status.setText(
                        "云台已停止" if reply.error() == QNetworkReply.NetworkError.NoError
                        else "停止请求未确认；NUC 超时保护会尝试停止云台。"
                    )
                reply.deleteLater()

            reply.finished.connect(finished)

    def stop_move(self):
        self._generation += 1
        self._direction = ""
        self.move_timer.stop()
        self.ptz_status.setText("已请求云台停止，等待服务应答")
        self._send_stop()

    def shutdown(self):
        self.frame_timer.stop()
        self.link_timer.stop()
        if self._direction or self._move_pending:
            self.stop_move()
        self._closed = True
        QApplication.instance().removeEventFilter(self)

    def eventFilter(self, watched, event):
        if event.type() == QEvent.Type.ApplicationDeactivate and self._direction:
            self.stop_move()
        return super().eventFilter(watched, event)

    def hideEvent(self, event):
        if self._direction:
            self.stop_move()
        super().hideEvent(event)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._scale_image()
