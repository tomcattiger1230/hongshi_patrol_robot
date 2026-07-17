#!/usr/bin/env python3
"""PySide6 control panel using ROS 2 when available, otherwise Fast DDS."""

from __future__ import annotations

import argparse
import sys
import time
from typing import Callable

from .fastdds_client import RobotRemoteFastDDSClient
from .gui_model import telemetry_view

try:
    from PySide6.QtCore import QMetaObject, QObject, Qt, QThread, QTimer, Signal, Slot
    from PySide6.QtGui import QCloseEvent, QFont
    from PySide6.QtWidgets import (
        QApplication,
        QComboBox,
        QDoubleSpinBox,
        QFormLayout,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QPlainTextEdit,
        QProgressBar,
        QPushButton,
        QSplitter,
        QTabWidget,
        QVBoxLayout,
        QWidget,
    )
except ImportError:  # pragma: no cover - depends on desktop environment.
    QApplication = None


if QApplication is not None:

    class CommunicationWorker(QObject):
        telemetry_received = Signal(object)
        reply_received = Signal(object)
        command_sent = Signal(str, str)
        connection_changed = Signal(bool, str)
        error = Signal(str)

        def __init__(
            self,
            domain_id: int,
            client_id: str,
            backend: str = "auto",
            client_factory: Callable[..., RobotRemoteFastDDSClient] = RobotRemoteFastDDSClient,
        ):
            super().__init__()
            self.domain_id = domain_id
            self.client_id = client_id
            self.backend = backend
            self.client_factory = client_factory
            self.client: RobotRemoteFastDDSClient | None = None
            self.poll_timer: QTimer | None = None

        @Slot()
        def start(self) -> None:
            try:
                self.client = self.client_factory(
                    domain_id=self.domain_id,
                    client_id=self.client_id,
                    backend=self.backend,
                )
            except Exception as exc:
                self.connection_changed.emit(False, str(exc))
                self.error.emit(f"通信后端启动失败：{exc}")
                return
            self.poll_timer = QTimer(self)
            self.poll_timer.setInterval(50)
            self.poll_timer.timeout.connect(self.poll)
            self.poll_timer.start()
            backend_name = {"ros2": "ROS 2", "fastdds": "Fast DDS"}.get(
                self.client.backend, self.client.backend
            )
            self.connection_changed.emit(True, f"{backend_name} 已启动，等待机器人遥测")

        @Slot()
        def poll(self) -> None:
            if self.client is None:
                return
            try:
                telemetry = self.client.receive_telemetry(timeout_s=0.0)
                if telemetry is not None:
                    self.telemetry_received.emit(telemetry)
                for _ in range(20):
                    reply = self.client.receive_reply(timeout_s=0.0)
                    if reply is None:
                        break
                    self.reply_received.emit(reply)
            except Exception as exc:
                self.error.emit(f"接收机器人数据失败：{exc}")

        def _send(self, description: str, function: Callable, *args) -> None:
            if self.client is None:
                self.error.emit("通信后端尚未启动")
                return
            try:
                command_id = function(*args)
                self.command_sent.emit(command_id, description)
            except Exception as exc:
                self.error.emit(f"发送“{description}”失败：{exc}")

        @Slot(float, float)
        def manual_motion(self, linear: float, angular: float) -> None:
            if self.client:
                self._send(
                    f"手动运动 v={linear:.2f} m/s, w={angular:.2f} rad/s",
                    self.client.send_manual_command,
                    linear,
                    angular,
                )
            else:
                self.error.emit("通信后端尚未启动")

        @Slot()
        def stop_robot(self) -> None:
            if self.client:
                self._send("停止", self.client.stop)

        @Slot()
        def brake(self) -> None:
            if self.client:
                self._send("刹车", self.client.brake)

        @Slot()
        def emergency_stop(self) -> None:
            if self.client:
                self._send("急停", self.client.emergency_stop)

        @Slot()
        def reset_idle(self) -> None:
            if self.client:
                self._send("解除急停并进入空闲", self.client.reset_idle)

        @Slot(float, float, float)
        def navigation_goal(self, x_m: float, y_m: float, yaw_rad: float) -> None:
            if self.client:
                self._send(
                    f"导航目标 ({x_m:.2f}, {y_m:.2f}, {yaw_rad:.2f})",
                    self.client.send_navigation_goal,
                    x_m,
                    y_m,
                    yaw_rad,
                )

        @Slot()
        def cancel_navigation(self) -> None:
            if self.client:
                self._send("取消导航", self.client.cancel_navigation)

        @Slot(str, object)
        def lift(self, action: str, target_height_m: object) -> None:
            if self.client:
                self._send(
                    f"升降杆 {action}",
                    self.client.control_lift,
                    action,
                    target_height_m,
                )

        @Slot()
        def shutdown(self) -> None:
            if self.poll_timer is not None:
                self.poll_timer.stop()
            if self.client is not None:
                try:
                    self.client.stop()
                finally:
                    self.client.close()
                    self.client = None


    class RemoteControlWindow(QMainWindow):
        manual_requested = Signal(float, float)
        stop_requested = Signal()
        brake_requested = Signal()
        estop_requested = Signal()
        reset_requested = Signal()
        navigation_requested = Signal(float, float, float)
        cancel_navigation_requested = Signal()
        lift_requested = Signal(str, object)

        def __init__(self, domain_id: int, client_id: str, backend: str = "auto"):
            super().__init__()
            self.domain_id = domain_id
            self.client_id = client_id
            self.backend = backend
            self._last_telemetry_at = 0.0
            self._motion: tuple[float, float] | None = None
            self._closing = False

            self.setWindowTitle("Robot320 远程控制台")
            self.resize(1120, 760)
            self._build_ui()
            self._apply_style()

            self.worker_thread = QThread(self)
            self.worker = CommunicationWorker(domain_id, client_id, backend)
            self.worker.moveToThread(self.worker_thread)
            self.worker_thread.started.connect(self.worker.start)
            self.worker_thread.finished.connect(self.worker.deleteLater)
            self._connect_worker()
            self.worker_thread.start()

            self.motion_timer = QTimer(self)
            self.motion_timer.setInterval(200)
            self.motion_timer.timeout.connect(self._repeat_motion)
            self.health_timer = QTimer(self)
            self.health_timer.setInterval(500)
            self.health_timer.timeout.connect(self._update_health)
            self.health_timer.start()

        def _connect_worker(self) -> None:
            self.manual_requested.connect(self.worker.manual_motion)
            self.stop_requested.connect(self.worker.stop_robot)
            self.brake_requested.connect(self.worker.brake)
            self.estop_requested.connect(self.worker.emergency_stop)
            self.reset_requested.connect(self.worker.reset_idle)
            self.navigation_requested.connect(self.worker.navigation_goal)
            self.cancel_navigation_requested.connect(self.worker.cancel_navigation)
            self.lift_requested.connect(self.worker.lift)
            self.worker.telemetry_received.connect(self._on_telemetry)
            self.worker.reply_received.connect(self._on_reply)
            self.worker.command_sent.connect(self._on_command_sent)
            self.worker.connection_changed.connect(self._on_connection_changed)
            self.worker.error.connect(self._on_error)

        def _build_ui(self) -> None:
            central = QWidget()
            root = QVBoxLayout(central)
            root.setContentsMargins(18, 18, 18, 18)
            root.setSpacing(12)

            header = QHBoxLayout()
            title = QLabel("Robot320 远程控制")
            title.setObjectName("title")
            header.addWidget(title)
            header.addStretch()
            self.connection_label = QLabel(
                f"● 正在启动 DDS  ·  Domain {self.domain_id}  ·  {self.client_id}"
            )
            self.connection_label.setObjectName("connectionPending")
            header.addWidget(self.connection_label)
            root.addLayout(header)

            splitter = QSplitter(Qt.Orientation.Horizontal)
            splitter.addWidget(self._build_status_panel())
            splitter.addWidget(self._build_control_tabs())
            splitter.setSizes([480, 620])
            root.addWidget(splitter, 1)

            log_group = QGroupBox("指令与应答")
            log_layout = QVBoxLayout(log_group)
            self.log = QPlainTextEdit()
            self.log.setReadOnly(True)
            self.log.setMaximumBlockCount(500)
            log_layout.addWidget(self.log)
            root.addWidget(log_group, 0)
            self.setCentralWidget(central)

        def _build_status_panel(self) -> QWidget:
            panel = QGroupBox("机器人状态")
            layout = QFormLayout(panel)
            layout.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
            self.robot_value = QLabel("--")
            self.chassis_value = QLabel("等待遥测")
            self.speed_value = QLabel("--")
            self.pose_value = QLabel("不可用")
            self.navigation_value = QLabel("idle")
            self.navigation_value.setWordWrap(True)
            self.nav_progress = QProgressBar()
            self.nav_progress.setRange(0, 100)
            self.lift_value = QLabel("不可用")
            self.battery_value = QLabel("不可用")
            self.faults_value = QLabel("无")
            self.faults_value.setWordWrap(True)
            layout.addRow("机器人", self.robot_value)
            layout.addRow("底盘", self.chassis_value)
            layout.addRow("速度", self.speed_value)
            layout.addRow("SLAM 位姿", self.pose_value)
            layout.addRow("导航", self.navigation_value)
            layout.addRow("导航进度", self.nav_progress)
            layout.addRow("升降杆", self.lift_value)
            layout.addRow("电池", self.battery_value)
            layout.addRow("故障", self.faults_value)
            return panel

        def _build_control_tabs(self) -> QWidget:
            tabs = QTabWidget()
            tabs.addTab(self._build_manual_tab(), "手动与安全")
            tabs.addTab(self._build_navigation_tab(), "导航")
            tabs.addTab(self._build_lift_tab(), "升降杆")
            return tabs

        def _build_manual_tab(self) -> QWidget:
            page = QWidget()
            layout = QVBoxLayout(page)
            values = QFormLayout()
            self.linear_speed = self._spin(0.0, 0.8, 0.05, 0.25, " m/s")
            self.angular_speed = self._spin(0.0, 1.2, 0.05, 0.5, " rad/s")
            values.addRow("线速度", self.linear_speed)
            values.addRow("角速度", self.angular_speed)
            layout.addLayout(values)

            pad = QGridLayout()
            directions = [
                ("↖ 左前", 1.0, 1.0, 0, 0),
                ("↑ 前进", 1.0, 0.0, 0, 1),
                ("右前 ↗", 1.0, -1.0, 0, 2),
                ("← 左转", 0.0, 1.0, 1, 0),
                ("■ 停止", 0.0, 0.0, 1, 1),
                ("右转 →", 0.0, -1.0, 1, 2),
                ("↙ 左后", -1.0, -1.0, 2, 0),
                ("↓ 后退", -1.0, 0.0, 2, 1),
                ("右后 ↘", -1.0, 1.0, 2, 2),
            ]
            for text, linear, angular, row, column in directions:
                button = QPushButton(text)
                button.setMinimumHeight(48)
                if linear == 0.0 and angular == 0.0:
                    button.clicked.connect(self._stop_motion)
                else:
                    button.pressed.connect(
                        lambda linear_factor=linear, angular_factor=angular: self._start_motion(
                            linear_factor, angular_factor
                        )
                    )
                    button.released.connect(self._stop_motion)
                pad.addWidget(button, row, column)
            layout.addLayout(pad)

            safety = QHBoxLayout()
            stop = QPushButton("停止")
            stop.clicked.connect(self._stop_motion)
            brake = QPushButton("刹车")
            brake.clicked.connect(lambda _checked=False: self.brake_requested.emit())
            estop = QPushButton("紧急停止")
            estop.setObjectName("emergency")
            estop.clicked.connect(self._emergency_stop)
            reset = QPushButton("解除急停 / 空闲")
            reset.clicked.connect(lambda _checked=False: self.reset_requested.emit())
            for button in (stop, brake, estop, reset):
                button.setMinimumHeight(48)
                safety.addWidget(button)
            layout.addLayout(safety)
            layout.addStretch()
            return page

        def _build_navigation_tab(self) -> QWidget:
            page = QWidget()
            layout = QVBoxLayout(page)
            form = QFormLayout()
            self.goal_x = self._spin(-1000.0, 1000.0, 0.1, 0.0, " m")
            self.goal_y = self._spin(-1000.0, 1000.0, 0.1, 0.0, " m")
            self.goal_yaw = self._spin(-3.1416, 3.1416, 0.05, 0.0, " rad")
            form.addRow("目标 X", self.goal_x)
            form.addRow("目标 Y", self.goal_y)
            form.addRow("目标朝向", self.goal_yaw)
            layout.addLayout(form)
            send = QPushButton("发送导航目标")
            send.setMinimumHeight(52)
            send.clicked.connect(
                lambda _checked=False: self.navigation_requested.emit(
                    self.goal_x.value(), self.goal_y.value(), self.goal_yaw.value()
                )
            )
            cancel = QPushButton("取消当前导航")
            cancel.clicked.connect(
                lambda _checked=False: self.cancel_navigation_requested.emit()
            )
            layout.addWidget(send)
            layout.addWidget(cancel)
            layout.addStretch()
            return page

        def _build_lift_tab(self) -> QWidget:
            page = QWidget()
            layout = QVBoxLayout(page)
            form = QFormLayout()
            self.lift_action = QComboBox()
            self.lift_action.addItem("升起", "raise")
            self.lift_action.addItem("下降", "lower")
            self.lift_action.addItem("移动到高度", "move_to")
            self.lift_action.addItem("停止", "stop")
            self.lift_height = self._spin(0.0, 10.0, 0.05, 0.0, " m")
            form.addRow("动作", self.lift_action)
            form.addRow("目标高度", self.lift_height)
            layout.addLayout(form)
            send = QPushButton("发送升降杆指令")
            send.setMinimumHeight(52)
            send.clicked.connect(self._send_lift)
            layout.addWidget(send)
            layout.addStretch()
            return page

        @staticmethod
        def _spin(
            minimum: float,
            maximum: float,
            step: float,
            value: float,
            suffix: str,
        ) -> QDoubleSpinBox:
            widget = QDoubleSpinBox()
            widget.setRange(minimum, maximum)
            widget.setSingleStep(step)
            widget.setDecimals(3)
            widget.setValue(value)
            widget.setSuffix(suffix)
            return widget

        def _start_motion(self, linear_factor: float, angular_factor: float) -> None:
            self._motion = (
                linear_factor * self.linear_speed.value(),
                angular_factor * self.angular_speed.value(),
            )
            self._repeat_motion()
            self.motion_timer.start()

        def _repeat_motion(self) -> None:
            if self._motion is not None:
                self.manual_requested.emit(*self._motion)

        def _stop_motion(self) -> None:
            self.motion_timer.stop()
            self._motion = None
            self.stop_requested.emit()

        def _emergency_stop(self) -> None:
            self.motion_timer.stop()
            self._motion = None
            self.estop_requested.emit()

        def _send_lift(self) -> None:
            action = self.lift_action.currentData()
            target = self.lift_height.value() if action == "move_to" else None
            self.lift_requested.emit(action, target)

        @Slot(object)
        def _on_telemetry(self, telemetry) -> None:
            self._last_telemetry_at = time.monotonic()
            view = telemetry_view(telemetry)
            self.robot_value.setText(view.robot_id)
            self.chassis_value.setText(view.chassis)
            self.speed_value.setText(view.speed)
            self.pose_value.setText(view.pose)
            self.navigation_value.setText(view.navigation)
            self.nav_progress.setValue(view.navigation_progress)
            self.lift_value.setText(view.lift)
            self.battery_value.setText(view.battery)
            self.faults_value.setText(view.faults)
            state = "在线" if view.online else "底盘离线"
            self._set_connection(view.online, f"{state} · Domain {self.domain_id}")

        @Slot(object)
        def _on_reply(self, reply) -> None:
            self._append_log(
                f"应答 {reply.status.upper()}  {reply.command_id[:8]}  {reply.message}"
            )

        @Slot(str, str)
        def _on_command_sent(self, command_id: str, description: str) -> None:
            self._append_log(f"发送 {command_id[:8]}  {description}")

        @Slot(bool, str)
        def _on_connection_changed(self, connected: bool, message: str) -> None:
            self._set_connection(connected, message)
            self._append_log(message)

        @Slot(str)
        def _on_error(self, message: str) -> None:
            self._append_log(f"错误 {message}")
            self.statusBar().showMessage(message, 8000)

        def _update_health(self) -> None:
            if self._last_telemetry_at <= 0:
                return
            age = time.monotonic() - self._last_telemetry_at
            if age > 2.0:
                self._set_connection(False, f"机器人遥测超时 {age:.1f}s")

        def _set_connection(self, connected: bool, message: str) -> None:
            symbol = "●"
            self.connection_label.setText(f"{symbol} {message}")
            self.connection_label.setObjectName(
                "connectionOnline" if connected else "connectionOffline"
            )
            self.connection_label.style().unpolish(self.connection_label)
            self.connection_label.style().polish(self.connection_label)

        def _append_log(self, message: str) -> None:
            timestamp = time.strftime("%H:%M:%S")
            self.log.appendPlainText(f"[{timestamp}] {message}")

        def _apply_style(self) -> None:
            QApplication.instance().setFont(QFont("Sans Serif", 10))
            self.setStyleSheet(
                """
                QMainWindow, QWidget { background: #f4f6f8; color: #1f2933; }
                QGroupBox { background: white; border: 1px solid #d9e0e7;
                            border-radius: 8px; margin-top: 10px; padding-top: 12px; }
                QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 5px; }
                QPushButton { background: #e8eef5; border: 1px solid #c8d2dc;
                              border-radius: 6px; padding: 8px 12px; }
                QPushButton:hover { background: #dbe8f5; }
                QPushButton:pressed { background: #bdd6ee; }
                QPushButton#emergency { background: #c62828; color: white; font-weight: bold; }
                QLabel#title { font-size: 22px; font-weight: bold; }
                QLabel#connectionOnline { color: #16803c; font-weight: bold; }
                QLabel#connectionOffline { color: #c62828; font-weight: bold; }
                QLabel#connectionPending { color: #9a6700; font-weight: bold; }
                QPlainTextEdit, QDoubleSpinBox, QComboBox { background: white; }
                """
            )

        def closeEvent(self, event: QCloseEvent) -> None:
            if self._closing:
                event.accept()
                return
            self._closing = True
            self.motion_timer.stop()
            if self.worker_thread.isRunning():
                QMetaObject.invokeMethod(
                    self.worker,
                    "shutdown",
                    Qt.ConnectionType.BlockingQueuedConnection,
                )
                self.worker_thread.quit()
                self.worker_thread.wait(3000)
            event.accept()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Robot320 Qt control panel")
    parser.add_argument("--domain-id", type=int, default=20)
    parser.add_argument("--client-id", default="remote_control_gui")
    parser.add_argument("--backend", choices=["auto", "ros2", "fastdds"], default="auto")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if QApplication is None:
        print(
            "PySide6 is required for the GUI. Install it with: python3 -m pip install PySide6",
            file=sys.stderr,
        )
        return 2
    app = QApplication(sys.argv[:1])
    app.setApplicationName("Robot320 Remote Control")
    window = RemoteControlWindow(args.domain_id, args.client_id, args.backend)
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
