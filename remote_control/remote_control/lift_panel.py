"""WAN-safe lift controls over the existing Robot320 reverse SSH path."""

from __future__ import annotations

import json
import os
import time

from PySide6.QtCore import QProcess, QTimer, Signal
from PySide6.QtWidgets import QGridLayout, QLabel, QPushButton, QVBoxLayout, QWidget


class LiftPanel(QWidget):
    availability_changed = Signal(bool)

    def __init__(self, parent=None, auto_tunnel: bool = True, transport: str = "wan"):
        super().__init__(parent)
        if transport not in {"wan", "local"}:
            raise ValueError("lift transport must be wan or local")
        self.transport = transport
        self.cloud_host = os.getenv("ROBOT_CLOUD_SSH_HOST", "hsjc_ecs")
        self.local_port = int(os.getenv("ROBOT_LIFT_WAN_SSH_PORT", "12224"))
        self.cloud_reverse_port = int(os.getenv("ROBOT_CLOUD_REVERSE_PORT", "12220"))
        self.host_key_alias = os.getenv("ROBOT_SSH_HOST_KEY_ALIAS", "192.168.88.108")
        self.tunnel = QProcess(self)
        self.control = QProcess(self)
        self._stdout_buffer = ""
        self._shutting_down = False
        self.buttons = []

        layout = QVBoxLayout(self)
        notice = QLabel(
            "每次点击只发送一次指令。平台会持续运动，直到点击“停止”或触发机械限位；"
            "链路不稳定时可重复点击，重复点击会重复发送。"
        )
        notice.setObjectName("liftNotice")
        notice.setWordWrap(True)
        layout.addWidget(notice)
        self.status = QLabel(
            "正在连接 NUC 本机升降控制……" if transport == "local"
            else "正在建立升降平台公网安全通道……"
        )
        self.status.setWordWrap(True)
        layout.addWidget(self.status)

        controls = QGridLayout()
        controls.setHorizontalSpacing(12)
        controls.setVerticalSpacing(12)
        for text, action, name, row, column, span in (
            ("↑  上升", "raise", "liftUp", 0, 0, 1),
            ("↓  下降", "lower", "liftDown", 0, 1, 1),
            ("■  停止", "stop", "liftStop", 1, 0, 2),
        ):
            button = QPushButton(text)
            button.setObjectName(name)
            button.setMinimumHeight(72)
            button.setEnabled(False)
            button.clicked.connect(lambda _checked=False, value=action: self.send(value))
            controls.addWidget(button, row, column, 1, span)
            self.buttons.append(button)
        layout.addLayout(controls)
        layout.addStretch()

        self.tunnel.started.connect(lambda: QTimer.singleShot(500, self._start_control))
        self.tunnel.errorOccurred.connect(lambda _error: self._failed("公网隧道启动失败，正在重连……"))
        self.tunnel.finished.connect(self._tunnel_finished)
        self.control.readyReadStandardOutput.connect(self._read_control_output)
        self.control.readyReadStandardError.connect(self._read_control_error)
        self.control.errorOccurred.connect(lambda _error: self._failed("升降控制连接失败，正在重连……"))
        self.control.finished.connect(self._control_finished)
        if auto_tunnel:
            if self.transport == "local":
                QTimer.singleShot(0, self._start_control)
            else:
                self.start_tunnel()
        else:
            self.status.setText("测试模式：未建立网络连接")

    def start_tunnel(self) -> None:
        if self.transport == "local":
            self._start_control()
            return
        if self._shutting_down or self.tunnel.state() != QProcess.ProcessState.NotRunning:
            return
        self.tunnel.setProgram("ssh")
        self.tunnel.setArguments(
            [
                "-N", "-T", "-o", "BatchMode=yes", "-o", "ExitOnForwardFailure=yes",
                "-o", "ServerAliveInterval=20", "-o", "ServerAliveCountMax=3",
                "-L", f"127.0.0.1:{self.local_port}:127.0.0.1:{self.cloud_reverse_port}", self.cloud_host,
            ]
        )
        self.tunnel.start()

    def _start_control(self) -> None:
        if self._shutting_down or self.control.state() != QProcess.ProcessState.NotRunning:
            return
        if self.transport == "local":
            self.control.setProgram("/usr/bin/python3")
            self.control.setArguments([
                os.getenv("ROBOT_LIFT_LOCAL_COMMAND", "/home/hs/robot320_lift/lift_control.py"),
                "--stream",
            ])
        else:
            if self.tunnel.state() != QProcess.ProcessState.Running:
                return
            self.control.setProgram("ssh")
            self.control.setArguments([
                "-p", str(self.local_port),
                "-o", f"HostKeyAlias={self.host_key_alias}",
                "-o", "StrictHostKeyChecking=yes",
                "-o", "BatchMode=yes",
                "-o", "ConnectTimeout=8",
                "hs@127.0.0.1",
                "/home/hs/robot320_lift/lift_control.py", "--stream",
            ])
        self.control.start()

    def _tunnel_finished(self, *_args) -> None:
        self._failed("公网隧道已断开，正在重连……")
        if not self._shutting_down:
            QTimer.singleShot(2000, self.start_tunnel)

    def _control_finished(self, *_args) -> None:
        source = "本机升降控制" if self.transport == "local" else "升降控制连接"
        self._failed(f"{source}已断开，正在重连……")
        if not self._shutting_down:
            QTimer.singleShot(1000, self._start_control)

    def send(self, action: str) -> None:
        if action not in {"raise", "lower", "stop"}:
            raise ValueError(f"unsupported lift action: {action}")
        if self.control.state() != QProcess.ProcessState.Running:
            self._failed("升降控制连接不可用，指令未发送")
            return
        payload = (action + "\n").encode("ascii")
        if self.control.write(payload) != len(payload):
            self._failed("升降控制连接写入失败，指令未发送")
            return
        labels = {"raise": "上升", "lower": "下降", "stop": "停止"}
        self.status.setText(f"{time.strftime('%H:%M:%S')} 已发送“{labels[action]}”，等待车端确认……")

    def _read_control_output(self) -> None:
        self._stdout_buffer += bytes(self.control.readAllStandardOutput()).decode(errors="replace")
        while "\n" in self._stdout_buffer:
            line, self._stdout_buffer = self._stdout_buffer.split("\n", 1)
            try:
                result = json.loads(line)
            except json.JSONDecodeError:
                continue
            if result.get("ready"):
                self._set_available(True, "升降平台控制通道已连接")
            elif result.get("ok"):
                labels = {"raise": "上升", "lower": "下降", "stop": "停止"}
                action = labels.get(result.get("action"), result.get("action", ""))
                self.status.setText(f"{time.strftime('%H:%M:%S')} 车端已向串口写入一次“{action}”指令")
            else:
                self.status.setText("车端拒绝指令：" + result.get("error", "未知错误"))

    def _read_control_error(self) -> None:
        error = bytes(self.control.readAllStandardError()).decode(errors="replace").strip()
        if error:
            self.status.setText("升降控制错误：" + error[-180:])

    def _set_available(self, available: bool, message: str) -> None:
        for button in self.buttons:
            button.setEnabled(available)
        self.status.setText(message)
        self.availability_changed.emit(available)

    def _failed(self, message: str) -> None:
        self._set_available(False, message)

    def shutdown(self) -> None:
        self._shutting_down = True
        for process in (self.control, self.tunnel):
            if process.state() != QProcess.ProcessState.NotRunning:
                process.terminate()
                if not process.waitForFinished(1000):
                    process.kill()
