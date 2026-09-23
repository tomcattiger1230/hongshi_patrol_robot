"""On-demand still capture from two onboard Hikrobot USB cameras over SSH."""
import os
import time

from PySide6.QtCore import QProcess, QTimer, Qt
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QFrame, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget


class IndustrialCameraPanel(QWidget):
    def __init__(self, parent=None, auto_tunnel=True):
        super().__init__(parent)
        self.cloud_host = os.getenv("ROBOT_CLOUD_SSH_HOST", "hsjc_ecs")
        self.local_port = int(os.getenv("ROBOT_WAN_SSH_PORT", "12223"))
        self.cloud_reverse_port = int(os.getenv("ROBOT_CLOUD_REVERSE_PORT", "12220"))
        self.host_key_alias = os.getenv("ROBOT_SSH_HOST_KEY_ALIAS", "192.168.88.108")
        self.tunnel = QProcess(self)
        self.capture_processes = {}
        self.images = {}
        self.statuses = {}
        self.buttons = {}

        root = QVBoxLayout(self)
        heading = QLabel("海康 USB 工业相机 · 按需静态抓图")
        heading.setStyleSheet("font-size: 17px; font-weight: 600;")
        root.addWidget(heading)
        self.connection = QLabel("正在建立公网安全通道……不会持续采集或上传图像")
        self.connection.setWordWrap(True)
        root.addWidget(self.connection)
        feeds = QHBoxLayout()
        for camera in (1, 2):
            card = QFrame()
            card.setStyleSheet("QFrame { background:white; border:1px solid #dce4ee; border-radius:10px; }")
            column = QVBoxLayout(card)
            column.addWidget(QLabel(f"工业相机 {camera}"))
            image = QLabel("尚未请求静态图像")
            image.setAlignment(Qt.AlignmentFlag.AlignCenter)
            image.setMinimumSize(320, 260)
            image.setStyleSheet("background:#111c2c;color:#a8bad0;border-radius:7px;")
            column.addWidget(image, 1)
            button = QPushButton("获取当前图像")
            button.setEnabled(False)
            button.clicked.connect(lambda _checked=False, c=camera: self.capture(c))
            column.addWidget(button)
            status = QLabel("相机只会在点击后打开并采集一帧")
            status.setWordWrap(True)
            column.addWidget(status)
            self.images[camera], self.buttons[camera], self.statuses[camera] = image, button, status
            feeds.addWidget(card, 1)
        root.addLayout(feeds, 1)
        note = QLabel("传输路径：NUC 主动反向 SSH → 公网中继 → Mac 本地 SSH；公网不开放机器人 SSH 端口。每次仅返回一张 JPEG，完成后相机立即关闭。")
        note.setWordWrap(True)
        root.addWidget(note)
        if auto_tunnel:
            self.start_tunnel()
        else:
            self.connection.setText("测试模式：未建立网络连接")

    def start_tunnel(self):
        args = ["-N", "-T", "-o", "BatchMode=yes", "-o", "ExitOnForwardFailure=yes",
                "-o", "ServerAliveInterval=20", "-o", "ServerAliveCountMax=3",
                "-L", f"127.0.0.1:{self.local_port}:127.0.0.1:{self.cloud_reverse_port}", self.cloud_host]
        self.tunnel.setProgram("ssh")
        self.tunnel.setArguments(args)
        self.tunnel.started.connect(lambda: QTimer.singleShot(500, self._tunnel_ready))
        self.tunnel.errorOccurred.connect(lambda _error: self._tunnel_failed())
        self.tunnel.finished.connect(lambda *_: self._tunnel_failed())
        self.tunnel.start()

    def _tunnel_ready(self):
        if self.tunnel.state() == QProcess.ProcessState.Running:
            self.connection.setText("公网安全通道已建立 · 等待按需抓图")
            for button in self.buttons.values():
                button.setEnabled(True)

    def _tunnel_failed(self):
        self.connection.setText("公网安全通道不可用；请检查云服务器、NUC 反向隧道和 Mac SSH 配置")
        for button in self.buttons.values():
            button.setEnabled(False)

    def capture(self, camera):
        if camera in self.capture_processes:
            return
        process = QProcess(self)
        args = ["-p", str(self.local_port), "-o", f"HostKeyAlias={self.host_key_alias}",
                "-o", "StrictHostKeyChecking=yes", "-o", "BatchMode=yes",
                "-o", "ConnectTimeout=8", "hs@127.0.0.1",
                "/home/hs/robot320_remote_spatial/hik_usb_snapshot.py",
                "--camera", str(camera), "--quality", "75"]
        process.setProgram("ssh")
        process.setArguments(args)
        process.setProcessChannelMode(QProcess.ProcessChannelMode.SeparateChannels)
        process.finished.connect(lambda code, _status, c=camera, p=process: self._capture_finished(c, p, code))
        self.capture_processes[camera] = process
        self.buttons[camera].setEnabled(False)
        self.statuses[camera].setText("正在请求相机并传输一张静态图像……")
        process.start()
        QTimer.singleShot(20000, lambda p=process: p.kill() if p.state() != QProcess.ProcessState.NotRunning else None)

    def _capture_finished(self, camera, process, code):
        data = bytes(process.readAllStandardOutput())
        error = bytes(process.readAllStandardError()).decode(errors="replace").strip()
        self.capture_processes.pop(camera, None)
        self.buttons[camera].setEnabled(self.tunnel.state() == QProcess.ProcessState.Running)
        pixmap = QPixmap()
        if code == 0 and 1024 <= len(data) <= 20 * 1024 * 1024 and pixmap.loadFromData(data, "JPEG"):
            self.images[camera].setPixmap(pixmap.scaled(
                self.images[camera].size(), Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation))
            preview = pixmap.toImage().scaled(32, 20).convertToFormat(QImage.Format.Format_Grayscale8)
            luminance = sum(preview.pixelColor(x, y).red() for y in range(preview.height()) for x in range(preview.width())) / (preview.width() * preview.height())
            detail = f"获取时间 {time.strftime('%H:%M:%S')} · {pixmap.width()}×{pixmap.height()} · {len(data)/1024:.0f} KiB"
            if luminance < 5:
                detail += " · ⚠ 相机原始画面接近全黑，请检查镜头盖、光圈和照明"
            self.statuses[camera].setText(detail)
        else:
            self.statuses[camera].setText("抓图失败：" + (error[-180:] or "无有效 JPEG 返回"))
        process.deleteLater()

    def shutdown(self):
        for process in list(self.capture_processes.values()):
            process.kill()
        self.capture_processes.clear()
        if self.tunnel.state() != QProcess.ProcessState.NotRunning:
            self.tunnel.terminate()
            if not self.tunnel.waitForFinished(1000):
                self.tunnel.kill()
