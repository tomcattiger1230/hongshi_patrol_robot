#!/usr/bin/env python3
"""ROS 2 map-click navigation GUI for Robot320."""

from __future__ import annotations

import argparse
import math
import os
import sys
from typing import Optional

from .map_model import (
    MapSnapshot,
    goal_yaw,
    map_snapshot,
    pose_uncertainty,
    yaw_from_quaternion,
)

try:
    from PySide6.QtCore import (
        QMetaObject,
        QObject,
        QPoint,
        Qt,
        QThread,
        QTimer,
        Signal,
        Slot,
    )
    from PySide6.QtGui import (
        QBrush,
        QCloseEvent,
        QColor,
        QFont,
        QImage,
        QPainter,
        QPen,
        QPixmap,
    )
    from PySide6.QtWidgets import (
        QApplication,
        QDoubleSpinBox,
        QFormLayout,
        QFrame,
        QGraphicsPixmapItem,
        QGraphicsScene,
        QGraphicsView,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QPushButton,
        QScrollArea,
        QSizePolicy,
        QSplitter,
        QVBoxLayout,
        QWidget,
    )
except ImportError:  # pragma: no cover - depends on the desktop environment.
    QApplication = None

try:
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import OccupancyGrid
    from rclpy.action import ActionClient
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from std_srvs.srv import Empty
    from tf2_ros import Buffer, TransformListener
except ImportError as exc:  # pragma: no cover - depends on ROS 2 installation.
    rclpy = None
    _ROS_IMPORT_ERROR = exc
else:
    _ROS_IMPORT_ERROR = None


if QApplication is not None:

    class MapView(QGraphicsView):
        """Occupancy-grid view with click-drag goal selection."""

        goal_changed = Signal(float, float, float)
        cursor_changed = Signal(float, float)

        def __init__(self) -> None:
            super().__init__()
            self.setRenderHint(QPainter.RenderHint.Antialiasing, True)
            self.setBackgroundBrush(QColor("#20262e"))
            self.setTransformationAnchor(
                QGraphicsView.ViewportAnchor.AnchorUnderMouse
            )
            self.setResizeAnchor(QGraphicsView.ViewportAnchor.AnchorViewCenter)
            self.setMinimumSize(680, 560)

            self._scene = QGraphicsScene(self)
            self.setScene(self._scene)
            self._map_item: Optional[QGraphicsPixmapItem] = None
            self._robot_items: list = []
            self._goal_items: list = []
            self._snapshot: Optional[MapSnapshot] = None
            self._goal_start: Optional[tuple[float, float]] = None
            self._pan_start: Optional[QPoint] = None
            self._robot_yaw = 0.0
            self._first_map = True

        def set_map(self, snapshot: MapSnapshot) -> None:
            geometry = snapshot.geometry
            pixels = bytearray(geometry.width * geometry.height)
            for scene_y in range(geometry.height):
                source_row = geometry.height - 1 - scene_y
                source_offset = source_row * geometry.width
                target_offset = scene_y * geometry.width
                for column in range(geometry.width):
                    occupancy = snapshot.data[source_offset + column]
                    if occupancy < 0:
                        shade = 112
                    else:
                        shade = max(25, 245 - round(2.2 * min(100, occupancy)))
                    pixels[target_offset + column] = shade

            image = QImage(
                pixels,
                geometry.width,
                geometry.height,
                geometry.width,
                QImage.Format.Format_Grayscale8,
            ).copy()
            pixmap = QPixmap.fromImage(image)
            if self._map_item is None:
                self._map_item = self._scene.addPixmap(pixmap)
                self._map_item.setZValue(0)
            else:
                self._map_item.setPixmap(pixmap)
            self._snapshot = snapshot
            self._scene.setSceneRect(0, 0, geometry.width, geometry.height)
            if self._first_map:
                self.fit_map()
                self._first_map = False

        def set_robot_pose(self, x_m: float, y_m: float, yaw_rad: float) -> None:
            self._robot_yaw = yaw_rad
            self._remove_items(self._robot_items)
            if self._snapshot is None:
                return
            geometry = self._snapshot.geometry
            if not geometry.contains_world(x_m, y_m):
                return
            scene_x, scene_y = geometry.world_to_scene(x_m, y_m)
            tip_x, tip_y = geometry.world_to_scene(
                x_m + 0.75 * math.cos(yaw_rad),
                y_m + 0.75 * math.sin(yaw_rad),
            )
            radius = max(3.5, 0.22 / geometry.resolution)
            body = self._scene.addEllipse(
                scene_x - radius,
                scene_y - radius,
                radius * 2.0,
                radius * 2.0,
                self._cosmetic_pen("#ffffff", 2.0),
                QBrush(QColor("#1976d2")),
            )
            heading = self._scene.addLine(
                scene_x,
                scene_y,
                tip_x,
                tip_y,
                self._cosmetic_pen("#0d47a1", 3.0),
            )
            body.setZValue(4)
            heading.setZValue(4)
            self._robot_items.extend([body, heading])

        def set_goal(self, x_m: float, y_m: float, yaw_rad: float) -> None:
            self._remove_items(self._goal_items)
            if self._snapshot is None:
                return
            geometry = self._snapshot.geometry
            scene_x, scene_y = geometry.world_to_scene(x_m, y_m)
            tip_x, tip_y = geometry.world_to_scene(
                x_m + 0.9 * math.cos(yaw_rad),
                y_m + 0.9 * math.sin(yaw_rad),
            )
            radius = max(3.0, 0.18 / geometry.resolution)
            marker = self._scene.addEllipse(
                scene_x - radius,
                scene_y - radius,
                radius * 2.0,
                radius * 2.0,
                self._cosmetic_pen("#fff3e0", 2.0),
                QBrush(QColor("#f57c00")),
            )
            heading = self._scene.addLine(
                scene_x,
                scene_y,
                tip_x,
                tip_y,
                self._cosmetic_pen("#ef6c00", 3.0),
            )
            marker.setZValue(5)
            heading.setZValue(5)
            self._goal_items.extend([marker, heading])

        def clear_goal(self) -> None:
            self._goal_start = None
            self._remove_items(self._goal_items)

        def fit_map(self) -> None:
            if self._map_item is not None:
                self.fitInView(
                    self._scene.sceneRect(),
                    Qt.AspectRatioMode.KeepAspectRatio,
                )

        def zoom_in(self) -> None:
            self.scale(1.25, 1.25)

        def zoom_out(self) -> None:
            self.scale(0.8, 0.8)

        def wheelEvent(self, event) -> None:
            factor = 1.18 if event.angleDelta().y() > 0 else 1.0 / 1.18
            self.scale(factor, factor)

        def mousePressEvent(self, event) -> None:
            if event.button() == Qt.MouseButton.MiddleButton:
                self._pan_start = event.position().toPoint()
                self.setCursor(Qt.CursorShape.ClosedHandCursor)
                event.accept()
                return
            if (
                event.button() == Qt.MouseButton.LeftButton
                and self._snapshot is not None
            ):
                scene_position = self.mapToScene(event.position().toPoint())
                if self._scene.sceneRect().contains(scene_position):
                    self._goal_start = self._snapshot.geometry.scene_to_world(
                        scene_position.x(), scene_position.y()
                    )
                    self.set_goal(*self._goal_start, self._robot_yaw)
                    event.accept()
                    return
            super().mousePressEvent(event)

        def mouseMoveEvent(self, event) -> None:
            if self._pan_start is not None:
                current = event.position().toPoint()
                delta = current - self._pan_start
                self._pan_start = current
                self.horizontalScrollBar().setValue(
                    self.horizontalScrollBar().value() - delta.x()
                )
                self.verticalScrollBar().setValue(
                    self.verticalScrollBar().value() - delta.y()
                )
                event.accept()
                return
            if self._snapshot is not None:
                scene_position = self.mapToScene(event.position().toPoint())
                x_m, y_m = self._snapshot.geometry.scene_to_world(
                    scene_position.x(), scene_position.y()
                )
                self.cursor_changed.emit(x_m, y_m)
                if self._goal_start is not None:
                    start_x, start_y = self._goal_start
                    yaw_rad = goal_yaw(
                        start_x,
                        start_y,
                        x_m,
                        y_m,
                        fallback=self._robot_yaw,
                    )
                    self.set_goal(start_x, start_y, yaw_rad)
                    event.accept()
                    return
            super().mouseMoveEvent(event)

        def mouseReleaseEvent(self, event) -> None:
            if (
                event.button() == Qt.MouseButton.MiddleButton
                and self._pan_start is not None
            ):
                self._pan_start = None
                self.unsetCursor()
                event.accept()
                return
            if (
                event.button() == Qt.MouseButton.LeftButton
                and self._snapshot is not None
                and self._goal_start is not None
            ):
                scene_position = self.mapToScene(event.position().toPoint())
                end_x, end_y = self._snapshot.geometry.scene_to_world(
                    scene_position.x(), scene_position.y()
                )
                start_x, start_y = self._goal_start
                yaw_rad = goal_yaw(
                    start_x,
                    start_y,
                    end_x,
                    end_y,
                    fallback=self._robot_yaw,
                )
                self.set_goal(start_x, start_y, yaw_rad)
                self.goal_changed.emit(start_x, start_y, yaw_rad)
                self._goal_start = None
                event.accept()
                return
            super().mouseReleaseEvent(event)

        @staticmethod
        def _cosmetic_pen(color: str, width: float) -> QPen:
            pen = QPen(QColor(color), width)
            pen.setCosmetic(True)
            return pen

        def _remove_items(self, items: list) -> None:
            for item in items:
                self._scene.removeItem(item)
            items.clear()


    class NavigationWorker(QObject):
        """ROS node and Nav2 action client hosted in a Qt worker thread."""

        map_received = Signal(object)
        pose_received = Signal(float, float, float)
        connection_changed = Signal(bool, str)
        navigation_changed = Signal(str)
        feedback_changed = Signal(float, float)
        localization_quality_changed = Signal(float, float)
        relocalization_changed = Signal(str)
        error = Signal(str)

        def __init__(
            self,
            map_topic: str,
            map_frame: str,
            base_frame: str,
            action_name: str,
            use_sim_time: bool,
        ) -> None:
            super().__init__()
            self.map_topic = map_topic
            self.map_frame = map_frame
            self.base_frame = base_frame
            self.action_name = action_name
            self.use_sim_time = use_sim_time
            self.node = None
            self.executor = None
            self.spin_timer: Optional[QTimer] = None
            self.tf_buffer = None
            self.tf_listener = None
            self.nav_client = None
            self.initial_pose_pub = None
            self.global_localization_client = None
            self.nomotion_update_client = None
            self.goal_handle = None
            self._owns_rclpy = False
            self._last_tf_emit_ns = 0
            self._server_ready = False

        @Slot()
        def start(self) -> None:
            if rclpy is None:
                self.error.emit(f"ROS 2 Python 模块不可用：{_ROS_IMPORT_ERROR}")
                return
            try:
                self._owns_rclpy = not rclpy.ok()
                if self._owns_rclpy:
                    rclpy.init(args=[])
                self.node = rclpy.create_node("robot320_navigation_gui")
                self.node.set_parameters(
                    [
                        Parameter(
                            "use_sim_time",
                            Parameter.Type.BOOL,
                            self.use_sim_time,
                        )
                    ]
                )
                map_qos = QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                )
                self.node.create_subscription(
                    OccupancyGrid,
                    self.map_topic,
                    self._on_map,
                    map_qos,
                )
                self.node.create_subscription(
                    PoseWithCovarianceStamped,
                    "/amcl_pose",
                    self._on_amcl_pose,
                    10,
                )
                self.initial_pose_pub = self.node.create_publisher(
                    PoseWithCovarianceStamped,
                    "/initialpose",
                    10,
                )
                self.global_localization_client = self.node.create_client(
                    Empty,
                    "/reinitialize_global_localization",
                )
                self.nomotion_update_client = self.node.create_client(
                    Empty,
                    "/request_nomotion_update",
                )
                self.tf_buffer = Buffer()
                self.tf_listener = TransformListener(
                    self.tf_buffer,
                    self.node,
                    spin_thread=False,
                )
                self.nav_client = ActionClient(
                    self.node,
                    NavigateToPose,
                    self.action_name,
                )
                self.executor = SingleThreadedExecutor()
                self.executor.add_node(self.node)
                self.spin_timer = QTimer(self)
                self.spin_timer.setInterval(20)
                self.spin_timer.timeout.connect(self.poll)
                self.spin_timer.start()
                self.connection_changed.emit(
                    False,
                    f"等待地图 {self.map_topic} 和 Nav2 {self.action_name}",
                )
            except Exception as exc:
                self.error.emit(f"ROS 2 导航界面启动失败：{exc}")

        @Slot()
        def poll(self) -> None:
            if self.executor is None or self.node is None:
                return
            try:
                self.executor.spin_once(timeout_sec=0.0)
                ready = bool(self.nav_client and self.nav_client.server_is_ready())
                if ready != self._server_ready:
                    self._server_ready = ready
                    self.connection_changed.emit(
                        ready,
                        "Nav2 已连接" if ready else "等待 Nav2 action server",
                    )
                now_ns = self.node.get_clock().now().nanoseconds
                if now_ns - self._last_tf_emit_ns >= 100_000_000:
                    self._publish_latest_pose()
                    self._last_tf_emit_ns = now_ns
            except Exception as exc:
                self.error.emit(f"ROS 2 数据处理失败：{exc}")

        def _publish_latest_pose(self) -> None:
            if self.tf_buffer is None:
                return
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.base_frame,
                    Time(),
                )
            except Exception:
                return
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            yaw_rad = math.atan2(
                2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
            )
            self.pose_received.emit(translation.x, translation.y, yaw_rad)

        def _on_map(self, message: OccupancyGrid) -> None:
            origin = message.info.origin
            snapshot = map_snapshot(
                width=message.info.width,
                height=message.info.height,
                resolution=message.info.resolution,
                origin_x=origin.position.x,
                origin_y=origin.position.y,
                origin_yaw=yaw_from_quaternion(
                    origin.orientation.z,
                    origin.orientation.w,
                ),
                data=message.data,
                frame_id=message.header.frame_id or self.map_frame,
            )
            self.map_frame = snapshot.frame_id
            self.map_received.emit(snapshot)

        def _on_amcl_pose(self, message: PoseWithCovarianceStamped) -> None:
            position_sigma, yaw_sigma = pose_uncertainty(message.pose.covariance)
            self.localization_quality_changed.emit(position_sigma, yaw_sigma)

        @Slot(float, float, float, str)
        def set_initial_pose(
            self,
            x_m: float,
            y_m: float,
            yaw_rad: float,
            frame_id: str,
        ) -> None:
            if self.initial_pose_pub is None or self.node is None:
                self.error.emit("ROS 2 初始位姿发布器尚未就绪")
                return
            self._cancel_active_goal()
            message = PoseWithCovarianceStamped()
            message.header.frame_id = frame_id or self.map_frame
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.pose.pose.position.x = x_m
            message.pose.pose.position.y = y_m
            message.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
            message.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
            message.pose.covariance[0] = 0.25
            message.pose.covariance[7] = 0.25
            message.pose.covariance[35] = math.radians(15.0) ** 2
            self.initial_pose_pub.publish(message)
            self.relocalization_changed.emit(
                "已发布粗略初始位姿；请观察蓝色机器人标记是否稳定"
            )

        @Slot()
        def global_relocalize(self) -> None:
            if (
                self.global_localization_client is None
                or not self.global_localization_client.service_is_ready()
            ):
                self.error.emit("AMCL 全局重定位服务尚未就绪")
                return
            self._cancel_active_goal()
            self.relocalization_changed.emit("正在请求 AMCL 全地图粒子搜索……")
            future = self.global_localization_client.call_async(Empty.Request())
            future.add_done_callback(self._on_global_relocalize_done)

        def _on_global_relocalize_done(self, future) -> None:
            try:
                future.result()
            except Exception as exc:
                self.relocalization_changed.emit(f"全局重定位请求失败：{exc}")
                return
            self.relocalization_changed.emit(
                "全局搜索已启动；请手动低速走大弧线，待定位置信度收敛后再导航"
            )

        @Slot()
        def request_nomotion_update(self) -> None:
            if (
                self.nomotion_update_client is None
                or not self.nomotion_update_client.service_is_ready()
            ):
                self.error.emit("AMCL 强制更新服务尚未就绪")
                return
            future = self.nomotion_update_client.call_async(Empty.Request())
            future.add_done_callback(self._on_nomotion_update_done)

        def _on_nomotion_update_done(self, future) -> None:
            try:
                future.result()
            except Exception as exc:
                self.relocalization_changed.emit(f"强制定位更新失败：{exc}")
                return
            self.relocalization_changed.emit("已使用当前 MID-360 扫描强制更新定位")

        @Slot(float, float, float, str)
        def send_goal(
            self,
            x_m: float,
            y_m: float,
            yaw_rad: float,
            frame_id: str,
        ) -> None:
            if self.nav_client is None or not self.nav_client.server_is_ready():
                self.error.emit("Nav2 /navigate_to_pose 尚未就绪")
                return
            self._cancel_active_goal()

            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = frame_id or self.map_frame
            # Use the latest available TF. In simulation, stamping with "now"
            # can be a few milliseconds newer than slam_toolbox's map->odom.
            goal.pose.pose.position.x = x_m
            goal.pose.pose.position.y = y_m
            goal.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
            self.navigation_changed.emit("正在发送目标……")
            future = self.nav_client.send_goal_async(
                goal,
                feedback_callback=self._on_feedback,
            )
            future.add_done_callback(self._on_goal_response)

        def _on_goal_response(self, future) -> None:
            try:
                goal_handle = future.result()
            except Exception as exc:
                self.navigation_changed.emit(f"目标发送失败：{exc}")
                return
            if goal_handle is None or not goal_handle.accepted:
                self.navigation_changed.emit("Nav2 拒绝了目标")
                return
            self.goal_handle = goal_handle
            self.navigation_changed.emit("目标已接受，AGV 正在行驶")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._on_result)

        def _on_feedback(self, feedback_message) -> None:
            feedback = feedback_message.feedback
            duration = feedback.estimated_time_remaining
            eta_seconds = float(duration.sec) + float(duration.nanosec) / 1e9
            self.feedback_changed.emit(
                float(feedback.distance_remaining),
                eta_seconds,
            )

        def _on_result(self, future) -> None:
            try:
                status = future.result().status
            except Exception as exc:
                self.navigation_changed.emit(f"读取导航结果失败：{exc}")
                return
            messages = {
                GoalStatus.STATUS_SUCCEEDED: "已到达目标点",
                GoalStatus.STATUS_CANCELED: "导航已取消",
                GoalStatus.STATUS_ABORTED: "导航失败或已中止",
            }
            self.navigation_changed.emit(messages.get(status, f"导航结束：{status}"))
            self.goal_handle = None

        @Slot()
        def cancel_goal(self) -> None:
            if self.goal_handle is None:
                self.navigation_changed.emit("当前没有活动导航目标")
                return
            self.goal_handle.cancel_goal_async()
            self.navigation_changed.emit("正在取消导航……")

        def _cancel_active_goal(self) -> None:
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
                self.goal_handle = None

        @Slot()
        def shutdown(self) -> None:
            if self.spin_timer is not None:
                self.spin_timer.stop()
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            if self.executor is not None and self.node is not None:
                self.executor.remove_node(self.node)
            if self.node is not None:
                self.node.destroy_node()
            if self.executor is not None:
                self.executor.shutdown(timeout_sec=0.5)
            if self._owns_rclpy and rclpy is not None and rclpy.ok():
                rclpy.shutdown()


    class NavigationWindow(QMainWindow):
        goal_requested = Signal(float, float, float, str)
        cancel_requested = Signal()
        initial_pose_requested = Signal(float, float, float, str)
        global_relocalize_requested = Signal()
        nomotion_update_requested = Signal()

        def __init__(
            self,
            map_topic: str,
            map_frame: str,
            base_frame: str,
            action_name: str,
            use_sim_time: bool,
        ) -> None:
            super().__init__()
            self.map_frame = map_frame
            self.snapshot: Optional[MapSnapshot] = None
            self.goal: Optional[tuple[float, float, float]] = None
            self._closing = False
            self.setWindowTitle("Robot320 地图导航")
            self.resize(1280, 900)
            self._build_ui()
            self._apply_style()

            self.worker_thread = QThread(self)
            self.worker = NavigationWorker(
                map_topic,
                map_frame,
                base_frame,
                action_name,
                use_sim_time,
            )
            self.worker.moveToThread(self.worker_thread)
            self.worker_thread.started.connect(self.worker.start)
            self.worker_thread.finished.connect(self.worker.deleteLater)
            self.goal_requested.connect(self.worker.send_goal)
            self.cancel_requested.connect(self.worker.cancel_goal)
            self.initial_pose_requested.connect(self.worker.set_initial_pose)
            self.global_relocalize_requested.connect(self.worker.global_relocalize)
            self.nomotion_update_requested.connect(self.worker.request_nomotion_update)
            self.worker.map_received.connect(self._on_map)
            self.worker.pose_received.connect(self._on_pose)
            self.worker.connection_changed.connect(self._on_connection)
            self.worker.navigation_changed.connect(self.navigation_value.setText)
            self.worker.feedback_changed.connect(self._on_feedback)
            self.worker.localization_quality_changed.connect(
                self._on_localization_quality
            )
            self.worker.relocalization_changed.connect(
                self.relocalization_value.setText
            )
            self.worker.error.connect(self._on_error)
            self.worker_thread.start()

        def _build_ui(self) -> None:
            central = QWidget()
            root = QVBoxLayout(central)
            root.setContentsMargins(16, 16, 16, 16)
            header = QHBoxLayout()
            title = QLabel("Robot320 地图导航")
            title.setObjectName("title")
            header.addWidget(title)
            header.addStretch()
            self.connection_value = QLabel("● 正在启动 ROS 2")
            self.connection_value.setObjectName("connectionPending")
            header.addWidget(self.connection_value)
            root.addLayout(header)

            splitter = QSplitter(Qt.Orientation.Horizontal)
            self.map_view = MapView()
            self.map_view.goal_changed.connect(self._on_goal_selected)
            self.map_view.cursor_changed.connect(self._on_cursor)
            splitter.addWidget(self.map_view)
            control_panel = self._build_control_panel()
            control_panel.setMinimumWidth(340)
            control_scroll = QScrollArea()
            control_scroll.setObjectName("controlScroll")
            control_scroll.setWidgetResizable(True)
            control_scroll.setFrameShape(QFrame.Shape.NoFrame)
            control_scroll.setHorizontalScrollBarPolicy(
                Qt.ScrollBarPolicy.ScrollBarAlwaysOff
            )
            control_scroll.setMinimumWidth(360)
            control_scroll.setWidget(control_panel)
            splitter.addWidget(control_scroll)
            splitter.setSizes([880, 380])
            splitter.setStretchFactor(0, 1)
            splitter.setStretchFactor(1, 0)
            splitter.setCollapsible(1, False)
            root.addWidget(splitter, 1)
            self.setCentralWidget(central)

        def _build_control_panel(self) -> QWidget:
            panel = QWidget()
            layout = QVBoxLayout(panel)

            map_group = QGroupBox("地图与机器人")
            map_layout = QFormLayout(map_group)
            self._configure_form_layout(map_layout)
            self.map_value = QLabel("等待 /map")
            self.pose_value = QLabel("等待 map → base_footprint")
            self.localization_quality_value = QLabel("等待 /amcl_pose")
            self.cursor_value = QLabel("--")
            for value_label in (
                self.map_value,
                self.pose_value,
                self.localization_quality_value,
                self.cursor_value,
            ):
                self._configure_value_label(value_label)
            map_layout.addRow("地图", self.map_value)
            map_layout.addRow("机器人", self.pose_value)
            map_layout.addRow("定位置信度", self.localization_quality_value)
            map_layout.addRow("鼠标", self.cursor_value)
            layout.addWidget(map_group)

            goal_group = QGroupBox("地图选中位姿")
            goal_layout = QFormLayout(goal_group)
            self._configure_form_layout(goal_layout)
            self.goal_x = self._spin(-1000.0, 1000.0, " m")
            self.goal_y = self._spin(-1000.0, 1000.0, " m")
            self.goal_yaw = self._spin(-180.0, 180.0, "°")
            goal_layout.addRow("X", self.goal_x)
            goal_layout.addRow("Y", self.goal_y)
            goal_layout.addRow("朝向", self.goal_yaw)
            layout.addWidget(goal_group)

            relocalization_group = QGroupBox("重定位")
            relocalization_layout = QVBoxLayout(relocalization_group)
            set_initial_pose_button = QPushButton("将选中位姿设为初始位置")
            set_initial_pose_button.clicked.connect(self._set_initial_pose)
            global_relocalize_button = QPushButton("不知道位置：全局重定位")
            global_relocalize_button.clicked.connect(
                lambda _checked=False: self.global_relocalize_requested.emit()
            )
            nomotion_update_button = QPushButton("使用当前扫描强制更新")
            nomotion_update_button.clicked.connect(
                lambda _checked=False: self.nomotion_update_requested.emit()
            )
            for button in (
                set_initial_pose_button,
                global_relocalize_button,
                nomotion_update_button,
            ):
                button.setMinimumHeight(38)
            self.relocalization_value = QLabel("可设置粗略位置或启动全地图搜索")
            self._configure_value_label(self.relocalization_value)
            relocalization_layout.addWidget(set_initial_pose_button)
            relocalization_layout.addWidget(global_relocalize_button)
            relocalization_layout.addWidget(nomotion_update_button)
            relocalization_layout.addWidget(self.relocalization_value)
            layout.addWidget(relocalization_group)

            self.send_button = QPushButton("发送目标，开始自动导航")
            self.send_button.setObjectName("primary")
            self.send_button.setMinimumHeight(48)
            self.send_button.setEnabled(False)
            self.send_button.clicked.connect(self._send_goal)
            cancel_button = QPushButton("取消当前导航")
            cancel_button.clicked.connect(
                lambda _checked=False: self.cancel_requested.emit()
            )
            clear_button = QPushButton("清除目标标记")
            clear_button.clicked.connect(self._clear_goal)
            layout.addWidget(self.send_button)
            layout.addWidget(cancel_button)
            layout.addWidget(clear_button)

            view_buttons = QHBoxLayout()
            fit_button = QPushButton("适配地图")
            fit_button.clicked.connect(self.map_view.fit_map)
            zoom_in = QPushButton("放大")
            zoom_in.clicked.connect(self.map_view.zoom_in)
            zoom_out = QPushButton("缩小")
            zoom_out.clicked.connect(self.map_view.zoom_out)
            view_buttons.addWidget(fit_button)
            view_buttons.addWidget(zoom_in)
            view_buttons.addWidget(zoom_out)
            layout.addLayout(view_buttons)

            status_group = QGroupBox("导航状态")
            status_layout = QVBoxLayout(status_group)
            self.navigation_value = QLabel("等待目标")
            self.feedback_value = QLabel("剩余距离：--　预计时间：--")
            self._configure_value_label(self.navigation_value)
            self._configure_value_label(self.feedback_value)
            status_layout.addWidget(self.navigation_value)
            status_layout.addWidget(self.feedback_value)
            layout.addWidget(status_group)

            help_text = QLabel(
                "操作：在可通行区域按下鼠标左键确定目标点，拖动决定车头朝向，"
                "松开后可将它用作初始位姿或导航目标。滚轮缩放，中键拖动平移地图。"
            )
            help_text.setWordWrap(True)
            help_text.setObjectName("help")
            layout.addWidget(help_text)
            layout.addStretch()
            return panel

        @staticmethod
        def _configure_form_layout(layout: QFormLayout) -> None:
            layout.setFieldGrowthPolicy(
                QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow
            )
            layout.setRowWrapPolicy(QFormLayout.RowWrapPolicy.WrapLongRows)

        @staticmethod
        def _configure_value_label(label: QLabel) -> None:
            label.setWordWrap(True)
            label.setMinimumWidth(0)
            label.setSizePolicy(
                QSizePolicy.Policy.Ignored,
                QSizePolicy.Policy.Preferred,
            )

        @staticmethod
        def _spin(minimum: float, maximum: float, suffix: str) -> QDoubleSpinBox:
            spin = QDoubleSpinBox()
            spin.setRange(minimum, maximum)
            spin.setDecimals(3)
            spin.setSingleStep(0.1)
            spin.setSuffix(suffix)
            return spin

        @Slot(object)
        def _on_map(self, snapshot: MapSnapshot) -> None:
            self.snapshot = snapshot
            self.map_frame = snapshot.frame_id
            self.map_view.set_map(snapshot)
            geometry = snapshot.geometry
            self.map_value.setText(
                f"{geometry.width} × {geometry.height}，"
                f"{geometry.resolution:.3f} m/格，frame={snapshot.frame_id}"
            )

        @Slot(float, float, float)
        def _on_pose(self, x_m: float, y_m: float, yaw_rad: float) -> None:
            self.map_view.set_robot_pose(x_m, y_m, yaw_rad)
            self.pose_value.setText(
                f"x={x_m:.2f} m，y={y_m:.2f} m，"
                f"yaw={math.degrees(yaw_rad):.1f}°"
            )

        @Slot(float, float, float)
        def _on_goal_selected(self, x_m: float, y_m: float, yaw_rad: float) -> None:
            self.goal = (x_m, y_m, yaw_rad)
            self.goal_x.setValue(x_m)
            self.goal_y.setValue(y_m)
            self.goal_yaw.setValue(math.degrees(yaw_rad))
            self.send_button.setEnabled(True)
            self.navigation_value.setText("目标已选择，等待确认发送")

        @Slot(float, float)
        def _on_cursor(self, x_m: float, y_m: float) -> None:
            self.cursor_value.setText(f"x={x_m:.2f} m，y={y_m:.2f} m")

        def _send_goal(self) -> None:
            selected_pose = self._validated_selected_pose()
            if selected_pose is None:
                return
            x_m, y_m, yaw_rad = selected_pose
            self.goal = (x_m, y_m, yaw_rad)
            self.map_view.set_goal(*self.goal)
            self.goal_requested.emit(x_m, y_m, yaw_rad, self.map_frame)

        def _set_initial_pose(self) -> None:
            selected_pose = self._validated_selected_pose()
            if selected_pose is None:
                return
            self.initial_pose_requested.emit(*selected_pose, self.map_frame)

        def _validated_selected_pose(self) -> Optional[tuple[float, float, float]]:
            if self.snapshot is None:
                self._on_error("尚未收到地图")
                return None
            if self.goal is None:
                self._on_error("请先在地图中点击并拖动选择位姿")
                return None
            x_m = self.goal_x.value()
            y_m = self.goal_y.value()
            yaw_rad = math.radians(self.goal_yaw.value())
            occupancy = self.snapshot.occupancy_at_world(x_m, y_m)
            if occupancy is None:
                self._on_error("选中位置不在当前地图范围内")
                return None
            if not self.snapshot.is_traversable(x_m, y_m):
                description = "未知区域" if occupancy < 0 else f"占用值 {occupancy}"
                self._on_error(f"选中位置位于障碍物或{description}，请重新选择")
                return None
            return x_m, y_m, yaw_rad

        def _clear_goal(self) -> None:
            self.goal = None
            self.map_view.clear_goal()
            self.send_button.setEnabled(False)

        @Slot(float, float)
        def _on_feedback(self, distance_m: float, eta_seconds: float) -> None:
            eta_text = f"{eta_seconds:.0f} s" if math.isfinite(eta_seconds) else "--"
            self.feedback_value.setText(
                f"剩余距离：{distance_m:.2f} m　预计时间：{eta_text}"
            )

        @Slot(float, float)
        def _on_localization_quality(
            self,
            position_sigma_m: float,
            yaw_sigma_rad: float,
        ) -> None:
            yaw_sigma_deg = math.degrees(yaw_sigma_rad)
            if position_sigma_m <= 0.25 and yaw_sigma_deg <= 10.0:
                level = "良好"
            elif position_sigma_m <= 0.75 and yaw_sigma_deg <= 25.0:
                level = "正在收敛"
            else:
                level = "不确定"
            self.localization_quality_value.setText(
                f"{level} · σ位置={position_sigma_m:.2f} m，"
                f"σ朝向={yaw_sigma_deg:.1f}°"
            )

        @Slot(bool, str)
        def _on_connection(self, connected: bool, message: str) -> None:
            self.connection_value.setText(f"● {message}")
            self.connection_value.setObjectName(
                "connectionOnline" if connected else "connectionPending"
            )
            self.connection_value.style().unpolish(self.connection_value)
            self.connection_value.style().polish(self.connection_value)

        @Slot(str)
        def _on_error(self, message: str) -> None:
            self.navigation_value.setText(f"错误：{message}")
            self.statusBar().showMessage(message, 8000)

        def _apply_style(self) -> None:
            QApplication.instance().setFont(QFont("Sans Serif", 10))
            self.setStyleSheet(
                """
                QMainWindow, QWidget { background: #f3f5f7; color: #20262e; }
                QGroupBox { background: white; border: 1px solid #d8dee5;
                            border-radius: 8px; margin-top: 10px; padding-top: 12px; }
                QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 5px; }
                QPushButton { background: #e7edf3; border: 1px solid #c4ced8;
                              border-radius: 6px; padding: 8px 10px; }
                QPushButton:hover { background: #d9e6f2; }
                QPushButton#primary { background: #1565c0; color: white;
                                      font-weight: bold; border: none; }
                QPushButton#primary:disabled { background: #9eabb7; }
                QLabel#title { font-size: 22px; font-weight: bold; }
                QLabel#connectionOnline { color: #16803c; font-weight: bold; }
                QLabel#connectionPending { color: #9a6700; font-weight: bold; }
                QLabel#help { color: #52606d; padding: 8px; }
                QDoubleSpinBox { background: white; padding: 5px; }
                """
            )

        def closeEvent(self, event: QCloseEvent) -> None:
            if self._closing:
                event.accept()
                return
            self._closing = True
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
    parser = argparse.ArgumentParser(description="Robot320 ROS 2 map navigation GUI")
    parser.add_argument("--domain-id", type=int, default=20)
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_footprint")
    parser.add_argument("--action-name", default="/navigate_to_pose")
    parser.add_argument("--use-sim-time", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if QApplication is None:
        print(
            "PySide6 is required. Run './scripts/uv_setup.sh desktop' first.",
            file=sys.stderr,
        )
        return 2
    if rclpy is None:
        print(
            f"ROS 2 Python packages are required: {_ROS_IMPORT_ERROR}",
            file=sys.stderr,
        )
        return 2
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    app = QApplication(sys.argv[:1])
    app.setApplicationName("Robot320 Map Navigation")
    window = NavigationWindow(
        args.map_topic,
        args.map_frame,
        args.base_frame,
        args.action_name,
        args.use_sim_time,
    )
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
