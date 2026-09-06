#!/usr/bin/env python3
"""PySide6 control panel using ROS 2, Fast DDS, or an offline demo backend."""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import sys
import threading
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
        QFileDialog,
        QFormLayout,
        QFrame,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QMessageBox,
        QPlainTextEdit,
        QProgressBar,
        QPushButton,
        QScrollArea,
        QSplitter,
        QTabWidget,
        QVBoxLayout,
        QWidget,
    )
except ImportError:  # pragma: no cover - depends on desktop environment.
    QApplication = None


if QApplication is not None:
    from .map_model import (
        load_map_yaml,
        map_snapshot,
        navigation_target_error,
        save_map_yaml,
    )
    from .map_transfer import SshMapSessionTransfer
    from .navigation_gui import MapView

    class CommunicationWorker(QObject):
        telemetry_received = Signal(object)
        map_received = Signal(object)
        reply_received = Signal(object)
        command_sent = Signal(str, str)
        map_save_sent = Signal(str)
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
                self.client.backend,
                "本地演示" if self.client.backend == "demo" else self.client.backend,
            )
            if self.client.backend == "demo":
                self.connection_changed.emit(True, "本地演示已启动（未连接机器人）")
            else:
                self.connection_changed.emit(True, f"{backend_name} 已启动，等待机器人遥测")

        @Slot()
        def poll(self) -> None:
            if self.client is None:
                return
            try:
                telemetry = self.client.receive_telemetry(timeout_s=0.0)
                if telemetry is not None:
                    self.telemetry_received.emit(telemetry)
                snapshot = self.client.receive_map(timeout_s=0.0)
                if snapshot is not None:
                    self.map_received.emit(snapshot)
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
                    f"导航目标 ({x_m:.2f}, {y_m:.2f}, "
                    f"{math.degrees(yaw_rad):.1f}°)",
                    self.client.send_navigation_goal,
                    x_m,
                    y_m,
                    yaw_rad,
                )

        @Slot()
        def cancel_navigation(self) -> None:
            if self.client:
                self._send("取消导航", self.client.cancel_navigation)

        @Slot(str)
        def set_mode(self, mode: str) -> None:
            if self.client:
                self._send(f"切换到 {mode} 模式", self.client.set_mode, mode)

        @Slot(object)
        def save_map(self, map_prefix: object = None) -> None:
            if self.client:
                prefix = str(map_prefix) if map_prefix else None
                try:
                    command_id = self.client.save_map(prefix)
                except Exception as exc:
                    self.error.emit(f"发送“保存当前地图”失败：{exc}")
                    return
                self.command_sent.emit(command_id, "保存当前地图")
                self.map_save_sent.emit(command_id)

        @Slot(str, str)
        def load_map(self, map_prefix: str, mode: str) -> None:
            if self.client:
                self._send(
                    f"载入地图 {map_prefix} ({mode})",
                    self.client.load_map,
                    map_prefix,
                    mode,
                )

        @Slot(bool)
        def set_exploration(self, enabled: bool) -> None:
            if self.client:
                action = "启动" if enabled else "停止"
                self._send(
                    f"{action}自由探索",
                    self.client.set_exploration,
                    enabled,
                )

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
        mode_requested = Signal(str)
        save_map_requested = Signal(object)
        load_map_requested = Signal(str, str)
        exploration_requested = Signal(bool)
        lift_requested = Signal(str, object)
        transfer_completed = Signal(str, object)
        transfer_failed = Signal(str, str)

        def __init__(
            self,
            domain_id: int,
            client_id: str,
            backend: str = "auto",
            ssh_target: str = "",
            remote_map_directory: str = "~/robot320_maps",
        ):
            super().__init__()
            self.domain_id = domain_id
            self.client_id = client_id
            self.backend = backend
            self._last_telemetry_at = 0.0
            self._last_map_revision: str | None = None
            self._map_snapshot = None
            self._motion: tuple[float, float] | None = None
            self._closing = False
            self._pending_download = None
            self._download_commands: dict[str, tuple[str, str]] = {}
            self._local_command_ids: set[str] = set()
            self._active_navigation_command_id: str | None = None
            self._pending_navigation_reply_id: str | None = None
            self.map_transfer = (
                SshMapSessionTransfer(ssh_target, remote_map_directory)
                if ssh_target and backend != "demo"
                else None
            )

            title_suffix = " [离线演示]" if backend == "demo" else ""
            self.setWindowTitle(f"Robot320 远程控制台{title_suffix}")
            self.resize(1440, 900)
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
            self.mode_requested.connect(self.worker.set_mode)
            self.save_map_requested.connect(self.worker.save_map)
            self.load_map_requested.connect(self.worker.load_map)
            self.exploration_requested.connect(self.worker.set_exploration)
            self.lift_requested.connect(self.worker.lift)
            self.worker.telemetry_received.connect(self._on_telemetry)
            self.worker.map_received.connect(self._on_map)
            self.worker.reply_received.connect(self._on_reply)
            self.worker.command_sent.connect(self._on_command_sent)
            self.worker.map_save_sent.connect(self._on_map_save_sent)
            self.worker.connection_changed.connect(self._on_connection_changed)
            self.worker.error.connect(self._on_error)
            self.transfer_completed.connect(self._on_transfer_completed)
            self.transfer_failed.connect(self._on_transfer_failed)

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
            pending_text = (
                "● 正在启动本地演示（未连接机器人）"
                if self.backend == "demo"
                else f"● 正在启动 DDS  ·  Domain {self.domain_id}  ·  {self.client_id}"
            )
            self.connection_label = QLabel(pending_text)
            self.connection_label.setObjectName("connectionPending")
            header.addWidget(self.connection_label)
            root.addLayout(header)

            if self.backend == "demo":
                demo_banner = QLabel(
                    "离线演示模式：所有状态和指令仅在本机内存中模拟，不会发送 DDS/ROS 2 消息。"
                )
                demo_banner.setObjectName("demoBanner")
                demo_banner.setWordWrap(True)
                root.addWidget(demo_banner)

            splitter = QSplitter(Qt.Orientation.Horizontal)
            splitter.addWidget(self._build_status_panel())
            splitter.addWidget(self._build_control_tabs())
            splitter.setStretchFactor(0, 0)
            splitter.setStretchFactor(1, 1)
            splitter.setSizes([440, 820])
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
            content = QWidget()
            content.setObjectName("statusPanel")
            layout = QVBoxLayout(content)
            layout.setContentsMargins(8, 4, 12, 8)
            layout.setSpacing(12)

            heading = QLabel("机器人状态")
            heading.setObjectName("sectionTitle")
            layout.addWidget(heading)

            self.robot_value = QLabel("--")
            self.chassis_value = QLabel("等待遥测")
            self.speed_value = QLabel("--")
            self.pose_value = QLabel("不可用")
            self.navigation_value = QLabel("idle")
            self.exploration_value = QLabel("已停止")
            self.nav_progress = QProgressBar()
            self.nav_progress.setRange(0, 100)
            self.nav_progress.setTextVisible(True)
            self.lift_value = QLabel("不可用")
            self.battery_value = QLabel("不可用")
            self.faults_value = QLabel("无")
            for value in (
                self.robot_value,
                self.chassis_value,
                self.speed_value,
                self.pose_value,
                self.navigation_value,
                self.exploration_value,
                self.lift_value,
                self.battery_value,
                self.faults_value,
            ):
                value.setObjectName("statusValue")
                value.setWordWrap(True)
                value.setTextInteractionFlags(
                    Qt.TextInteractionFlag.TextSelectableByMouse
                )

            overview = QGridLayout()
            overview.setHorizontalSpacing(10)
            overview.setVerticalSpacing(10)
            overview.addWidget(self._status_card("机器人", self.robot_value), 0, 0)
            overview.addWidget(self._status_card("当前速度", self.speed_value), 0, 1)
            overview.addWidget(
                self._status_card("SLAM 位姿", self.pose_value), 1, 0, 1, 2
            )
            layout.addLayout(overview)

            navigation = QGroupBox("导航任务")
            navigation_layout = QVBoxLayout(navigation)
            navigation_layout.setSpacing(10)
            navigation_layout.addWidget(self.navigation_value)
            progress_caption = QLabel("执行进度")
            progress_caption.setObjectName("statusCaption")
            navigation_layout.addWidget(progress_caption)
            navigation_layout.addWidget(self.nav_progress)
            exploration_row = QHBoxLayout()
            exploration_caption = QLabel("自由探索")
            exploration_caption.setObjectName("statusCaption")
            exploration_row.addWidget(exploration_caption)
            exploration_row.addStretch()
            exploration_row.addWidget(self.exploration_value)
            navigation_layout.addLayout(exploration_row)
            layout.addWidget(navigation)

            devices = QGroupBox("设备与能源")
            devices_layout = QGridLayout(devices)
            devices_layout.setHorizontalSpacing(10)
            devices_layout.setVerticalSpacing(10)
            devices_layout.addWidget(
                self._status_card("底盘", self.chassis_value), 0, 0, 1, 2
            )
            devices_layout.addWidget(self._status_card("升降杆", self.lift_value), 1, 0)
            devices_layout.addWidget(self._status_card("电池", self.battery_value), 1, 1)
            layout.addWidget(devices)

            faults = QGroupBox("故障与告警")
            faults.setObjectName("faultGroup")
            faults_layout = QVBoxLayout(faults)
            faults_layout.addWidget(self.faults_value)
            layout.addWidget(faults)
            layout.addStretch()

            scroll = QScrollArea()
            scroll.setObjectName("statusScroll")
            scroll.setFrameShape(QFrame.Shape.NoFrame)
            scroll.setWidgetResizable(True)
            scroll.setMinimumWidth(400)
            scroll.setWidget(content)
            return scroll

        @staticmethod
        def _status_card(title: str, value: QWidget) -> QFrame:
            card = QFrame()
            card.setObjectName("statusCard")
            card_layout = QVBoxLayout(card)
            card_layout.setContentsMargins(12, 9, 12, 10)
            card_layout.setSpacing(4)
            caption = QLabel(title)
            caption.setObjectName("statusCaption")
            card_layout.addWidget(caption)
            card_layout.addWidget(value)
            return card

        def _build_control_tabs(self) -> QWidget:
            tabs = QTabWidget()
            tabs.addTab(self._build_manual_tab(), "手动与安全")
            tabs.addTab(self._build_navigation_tab(), "导航与地图")
            tabs.addTab(self._build_lift_tab(), "升降杆")
            tabs.setCurrentIndex(1)
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

        def _build_navigation_tab(self) -> QWidget:
            page = QWidget()
            layout = QVBoxLayout(page)
            hint = QLabel(
                "左键点击并拖动选择目标与朝向；中键拖动画布；滚轮缩放。"
                "扫图时先进入人工模式，再到“手动与安全”驾驶。"
            )
            hint.setWordWrap(True)
            layout.addWidget(hint)

            self.map_view = MapView()
            self.map_view.setMinimumSize(520, 340)
            self.map_view.goal_changed.connect(self._on_map_goal)
            self.map_view.cursor_changed.connect(self._on_map_cursor)
            layout.addWidget(self.map_view, 1)

            info = QHBoxLayout()
            self.map_status = QLabel("等待 AGV 地图…")
            self.map_cursor = QLabel("光标: --")
            info.addWidget(self.map_status)
            info.addStretch()
            info.addWidget(self.map_cursor)
            layout.addLayout(info)

            self.selected_goal_status = QLabel("期望目标：尚未选择")
            self.selected_goal_status.setWordWrap(True)
            layout.addWidget(self.selected_goal_status)

            target = QGroupBox("期望目标（map 坐标系）")
            target_layout = QGridLayout(target)
            self.goal_x = self._spin(-1000.0, 1000.0, 0.1, 0.0, " m")
            self.goal_y = self._spin(-1000.0, 1000.0, 0.1, 0.0, " m")
            self.goal_yaw = self._spin(-180.0, 180.0, 1.0, 0.0, " °")
            target_layout.addWidget(QLabel("X"), 0, 0)
            target_layout.addWidget(self.goal_x, 0, 1)
            target_layout.addWidget(QLabel("Y"), 0, 2)
            target_layout.addWidget(self.goal_y, 0, 3)
            target_layout.addWidget(QLabel("朝向"), 0, 4)
            target_layout.addWidget(self.goal_yaw, 0, 5)
            self.map_navigate = QPushButton("发送地图所选目标")
            self.map_navigate.setEnabled(False)
            self.map_navigate.clicked.connect(self._send_selected_map_goal)
            send_coordinates = QPushButton("发送输入坐标")
            send_coordinates.clicked.connect(self._send_coordinate_goal)
            cancel_navigation = QPushButton("取消当前导航")
            cancel_navigation.clicked.connect(
                lambda _checked=False: self.cancel_navigation_requested.emit()
            )
            target_layout.addWidget(self.map_navigate, 1, 0, 1, 2)
            target_layout.addWidget(send_coordinates, 1, 2, 1, 2)
            target_layout.addWidget(cancel_navigation, 1, 4, 1, 2)
            layout.addWidget(target)

            view_actions = QHBoxLayout()
            fit = QPushButton("适应窗口")
            fit.clicked.connect(self.map_view.fit_map)
            zoom_in = QPushButton("放大")
            zoom_in.clicked.connect(self.map_view.zoom_in)
            zoom_out = QPushButton("缩小")
            zoom_out.clicked.connect(self.map_view.zoom_out)
            for button in (fit, zoom_in, zoom_out):
                view_actions.addWidget(button)
            layout.addLayout(view_actions)

            actions = QHBoxLayout()
            mapping = QPushButton("进入人工扫图模式")
            mapping.clicked.connect(
                lambda _checked=False: self.mode_requested.emit("manual")
            )
            save = QPushButton("保存到机器人")
            save.clicked.connect(
                lambda _checked=False: self.save_map_requested.emit(None)
            )
            export = QPushButton("导出栅格地图…")
            export.clicked.connect(self._export_grid_map)
            actions.addWidget(mapping)
            actions.addWidget(save)
            actions.addWidget(export)
            layout.addLayout(actions)

            session_actions = QHBoxLayout()
            self.download_session = QPushButton("保存完整会话到本机…")
            self.download_session.setEnabled(self.map_transfer is not None)
            self.download_session.clicked.connect(self._save_full_session_to_mac)
            self.upload_session = QPushButton("从本机载入地图…")
            self.upload_session.setEnabled(self.map_transfer is not None)
            self.upload_session.clicked.connect(self._load_map_from_mac)
            session_actions.addWidget(self.download_session)
            session_actions.addWidget(self.upload_session)
            layout.addLayout(session_actions)

            exploration_actions = QHBoxLayout()
            self.start_exploration = QPushButton("▶ 启动自由探索")
            self.start_exploration.clicked.connect(
                lambda _checked=False: self.exploration_requested.emit(True)
            )
            self.stop_exploration = QPushButton("■ 停止自由探索")
            self.stop_exploration.setEnabled(False)
            self.stop_exploration.clicked.connect(
                lambda _checked=False: self.exploration_requested.emit(False)
            )
            exploration_actions.addWidget(self.start_exploration)
            exploration_actions.addWidget(self.stop_exploration)
            layout.addLayout(exploration_actions)
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
        def _on_map(self, remote_map) -> None:
            if remote_map.revision == self._last_map_revision:
                return
            try:
                snapshot = map_snapshot(
                    width=remote_map.width,
                    height=remote_map.height,
                    resolution=remote_map.resolution,
                    origin_x=remote_map.origin_x,
                    origin_y=remote_map.origin_y,
                    origin_yaw=remote_map.origin_yaw,
                    data=remote_map.occupancy_data(),
                    frame_id=remote_map.frame_id,
                )
            except (TypeError, ValueError) as exc:
                self._on_error(f"地图数据无效：{exc}")
                return
            self.map_view.set_map(snapshot)
            self._map_snapshot = snapshot
            self._last_map_revision = remote_map.revision
            self.map_status.setText(
                f"地图 {remote_map.width}×{remote_map.height} · "
                f"{remote_map.resolution:.3f} m/cell · {remote_map.revision[:8]}"
            )

        @Slot(float, float, float)
        def _on_map_goal(self, x_m: float, y_m: float, yaw_rad: float) -> None:
            error = navigation_target_error(self._map_snapshot, x_m, y_m)
            if error:
                self.map_view.clear_goal()
                self.map_navigate.setEnabled(False)
                self.selected_goal_status.setText(f"期望目标不可用：{error}")
                self.statusBar().showMessage(f"期望目标未选择：{error}", 6000)
                return
            self.goal_x.setValue(x_m)
            self.goal_y.setValue(y_m)
            yaw_deg = math.degrees(yaw_rad)
            self.goal_yaw.setValue(yaw_deg)
            self.map_navigate.setEnabled(True)
            self.selected_goal_status.setText(
                f"期望目标：x={x_m:.2f} m，y={y_m:.2f} m，朝向={yaw_deg:.1f}°"
            )
            self.statusBar().showMessage(
                f"已选择目标 ({x_m:.2f}, {y_m:.2f}), 朝向 {yaw_deg:.1f}°",
                5000,
            )

        @Slot(float, float)
        def _on_map_cursor(self, x_m: float, y_m: float) -> None:
            self.map_cursor.setText(f"光标: {x_m:.2f}, {y_m:.2f} m")

        def _send_selected_map_goal(self) -> None:
            self._send_expected_goal(
                self.goal_x.value(),
                self.goal_y.value(),
                math.radians(self.goal_yaw.value()),
            )

        def _send_coordinate_goal(self) -> None:
            x_m = self.goal_x.value()
            y_m = self.goal_y.value()
            yaw_rad = math.radians(self.goal_yaw.value())
            if self._map_snapshot is not None:
                self.map_view.set_goal(x_m, y_m, yaw_rad)
            self._send_expected_goal(x_m, y_m, yaw_rad)

        def _send_expected_goal(self, x_m: float, y_m: float, yaw_rad: float) -> None:
            error = navigation_target_error(self._map_snapshot, x_m, y_m)
            if error:
                self.map_navigate.setEnabled(False)
                self.selected_goal_status.setText(f"期望目标未发送：{error}")
                self._on_error(f"期望目标未发送：{error}")
                return
            self.map_navigate.setEnabled(True)
            self.selected_goal_status.setText(
                f"期望目标发送中：x={x_m:.2f} m，y={y_m:.2f} m，"
                f"朝向={math.degrees(yaw_rad):.1f}°"
            )
            self.navigation_requested.emit(x_m, y_m, yaw_rad)

        def _export_grid_map(self) -> None:
            if self._map_snapshot is None:
                self._on_error("尚未收到地图，无法导出")
                return
            default = str(Path.home() / "robot320_maps" / "patrol_current.yaml")
            selected, _filter = QFileDialog.getSaveFileName(
                self, "导出 Robot320 栅格地图到本机", default, "ROS 地图 (*.yaml)"
            )
            if not selected:
                return
            try:
                yaml_path, pgm_path = save_map_yaml(self._map_snapshot, selected)
            except Exception as exc:
                self._on_error(f"导出地图失败：{exc}")
                return
            self._append_log(f"地图已导出到本机：{yaml_path}、{pgm_path}")

        def _save_full_session_to_mac(self) -> None:
            if self.map_transfer is None:
                self._on_error("未配置机器人 SSH 目标")
                return
            if self._pending_download is not None:
                self._on_error("已有地图保存任务正在等待机器人完成")
                return
            default = str(Path.home() / "robot320_maps" / "patrol_current.yaml")
            selected, _filter = QFileDialog.getSaveFileName(
                self, "保存完整 SLAM 会话到本机", default, "ROS 地图 (*.yaml)"
            )
            if not selected:
                return
            try:
                remote_prefix = self.map_transfer.remote_prefix(Path(selected).stem)
            except ValueError as exc:
                self._on_error(str(exc))
                return
            self._pending_download = (remote_prefix, selected)
            self.download_session.setEnabled(False)
            self._append_log(f"请求机器人生成完整地图会话：{remote_prefix}")
            self.save_map_requested.emit(remote_prefix)

        @Slot(str)
        def _on_map_save_sent(self, command_id: str) -> None:
            if self._pending_download is None:
                return
            self._download_commands[command_id] = self._pending_download
            self._pending_download = None

        def _load_map_from_mac(self) -> None:
            if self.map_transfer is None:
                self._on_error("未配置机器人 SSH 目标")
                return
            selected, _filter = QFileDialog.getOpenFileName(
                self,
                "从本机载入 Robot320 地图",
                str(Path.home() / "robot320_maps"),
                "ROS 地图 (*.yaml *.yml)",
            )
            if not selected:
                return
            try:
                snapshot = load_map_yaml(selected)
            except Exception as exc:
                self._on_error(f"读取地图失败：{exc}")
                return
            answer = QMessageBox.question(
                self,
                "确认载入地图",
                "载入会停止自由探索和当前导航，并替换机器人使用的地图。是否继续？",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No,
            )
            if answer != QMessageBox.StandardButton.Yes:
                return
            self.map_view.set_map(snapshot)
            self.map_status.setText(f"本机预览 · {Path(selected).name}")
            self.upload_session.setEnabled(False)
            self._append_log(f"正在通过 SSH 上传本机地图会话：{selected}")
            self._run_transfer("upload", self.map_transfer.upload, selected)

        def _run_transfer(self, operation: str, function: Callable, *args) -> None:
            def run() -> None:
                try:
                    result = function(*args)
                except Exception as exc:
                    message = getattr(exc, "stderr", None) or str(exc)
                    self.transfer_failed.emit(operation, str(message).strip())
                    return
                self.transfer_completed.emit(operation, result)

            threading.Thread(target=run, name=f"robot320-map-{operation}", daemon=True).start()

        @Slot(str, object)
        def _on_transfer_completed(self, operation: str, result: object) -> None:
            if operation == "upload":
                self.upload_session.setEnabled(True)
                self._append_log(
                    f"地图已上传，正在通知机器人载入：{result.remote_prefix}"
                )
                self.load_map_requested.emit(result.remote_prefix, result.mode)
                return
            self.download_session.setEnabled(True)
            paths = ", ".join(str(path) for path in result)
            self._append_log(f"完整地图会话已保存到本机：{paths}")
            self.statusBar().showMessage("完整地图会话已保存到本机", 8000)

        @Slot(str, str)
        def _on_transfer_failed(self, operation: str, message: str) -> None:
            if operation == "upload":
                self.upload_session.setEnabled(True)
            else:
                self.download_session.setEnabled(True)
            self._on_error(f"地图文件传输失败：{message}")

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
            exploring = bool(telemetry.exploration_enabled)
            self.exploration_value.setText("运行中" if exploring else "已停止")
            self.start_exploration.setEnabled(not exploring)
            self.stop_exploration.setEnabled(exploring)
            if telemetry.pose is not None:
                self.map_view.set_robot_pose(
                    telemetry.pose.x_m,
                    telemetry.pose.y_m,
                    telemetry.pose.yaw_rad,
                )
            if self.backend == "demo":
                self._set_connection(True, "本地演示（未连接机器人）")
            else:
                state = "在线" if view.online else "底盘离线"
                self._set_connection(view.online, f"{state} · Domain {self.domain_id}")

        @Slot(object)
        def _on_reply(self, reply) -> None:
            if reply.command_id not in self._local_command_ids:
                return
            self._append_log(
                f"应答 {reply.status.upper()}  {reply.command_id[:8]}  {reply.message}"
            )
            if reply.command_id == self._active_navigation_command_id:
                self._pending_navigation_reply_id = None
                if reply.status == "accepted":
                    self.selected_goal_status.setText("期望目标已被 Nav2 接受，正在执行")
                elif reply.status in {"completed", "failed", "rejected"}:
                    status_text = {
                        "completed": "期望目标执行完成",
                        "failed": "期望目标执行失败",
                        "rejected": "期望目标被拒绝",
                    }[reply.status]
                    self.selected_goal_status.setText(f"{status_text}：{reply.message}")
                    self._active_navigation_command_id = None
            download = self._download_commands.get(reply.command_id)
            if download is None or reply.status not in {"completed", "failed", "rejected"}:
                return
            self._download_commands.pop(reply.command_id, None)
            if reply.status != "completed":
                self.download_session.setEnabled(True)
                self._on_error(f"机器人保存完整地图失败：{reply.message}")
                return
            remote_prefix, local_path = download
            self._append_log("机器人保存完成，正在下载地图文件…")
            self._run_transfer(
                "download", self.map_transfer.download, remote_prefix, local_path
            )

        @Slot(str, str)
        def _on_command_sent(self, command_id: str, description: str) -> None:
            self._local_command_ids.add(command_id)
            if description.startswith("导航目标"):
                self._active_navigation_command_id = command_id
                self._pending_navigation_reply_id = command_id
                self.selected_goal_status.setText("期望目标已发送，等待 Nav2 接受")
                QTimer.singleShot(
                    10_000,
                    lambda expected_id=command_id: self._on_navigation_reply_timeout(
                        expected_id
                    ),
                )
            self._append_log(f"发送 {command_id[:8]}  {description}")

        def _on_navigation_reply_timeout(self, command_id: str) -> None:
            if self._pending_navigation_reply_id != command_id:
                return
            self._pending_navigation_reply_id = None
            self.selected_goal_status.setText(
                "Nav2 未在 10 秒内确认期望目标，请取消后重试"
            )
            self._append_log(
                f"警告 {command_id[:8]}  Nav2 目标确认超时，未收到应答"
            )

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
                QLabel#sectionTitle { font-size: 18px; font-weight: bold; color: #263746;
                                      padding: 4px 2px 2px 2px; }
                QLabel#statusCaption { color: #66788a; font-size: 11px;
                                       background: transparent; }
                QLabel#statusValue { font-size: 14px; font-weight: 600; color: #1f2933;
                                     background: transparent; }
                QFrame#statusCard { background: #f8fafc; border: 1px solid #dce4eb;
                                    border-radius: 8px; }
                QGroupBox#faultGroup { border-color: #e6c9c9; }
                QScrollArea#statusScroll, QWidget#statusPanel { background: transparent;
                                                                border: none; }
                QLabel#connectionOnline { color: #16803c; font-weight: bold; }
                QLabel#connectionOffline { color: #c62828; font-weight: bold; }
                QLabel#connectionPending { color: #9a6700; font-weight: bold; }
                QLabel#demoBanner { background: #fff4ce; color: #7a4d00;
                                    border: 1px solid #e5c365; border-radius: 6px;
                                    padding: 8px; font-weight: bold; }
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
    parser.add_argument(
        "--backend", choices=["auto", "ros2", "fastdds", "demo"], default="auto"
    )
    parser.add_argument(
        "--robot-ssh",
        default=os.environ.get("ROBOT320_SSH_TARGET", "arnold@192.168.0.218"),
        help="SSH target used for map-session file transfer",
    )
    parser.add_argument(
        "--remote-map-directory",
        default=os.environ.get("ROBOT320_REMOTE_MAP_DIR", "~/robot320_maps"),
        help="robot-side directory allowed for map sessions",
    )
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
    window = RemoteControlWindow(
        args.domain_id,
        args.client_id,
        args.backend,
        args.robot_ssh,
        args.remote_map_directory,
    )
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
