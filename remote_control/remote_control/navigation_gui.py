#!/usr/bin/env python3
"""ROS 2 map-click navigation GUI for Robot320."""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import sys
from typing import Optional

from .map_model import (
    MapSnapshot,
    goal_yaw,
    load_map_yaml,
    map_snapshot,
    polyline_length,
    pose_uncertainty,
    project_laser_scan,
    scan_alignment_score,
    save_map_yaml,
    yaw_from_quaternion,
)


def normalized_frame_id(frame_id: str, fallback: str = "map") -> str:
    """Return a non-empty ROS frame for navigation goals."""
    return frame_id.strip() or fallback.strip() or "map"


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
        QPainterPath,
        QPen,
        QPixmap,
    )
    from PySide6.QtWidgets import (
        QApplication,
        QDoubleSpinBox,
        QFileDialog,
        QFormLayout,
        QFrame,
        QGraphicsItem,
        QGraphicsPixmapItem,
        QGraphicsScene,
        QGraphicsView,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QListWidget,
        QMainWindow,
        QMessageBox,
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
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from nav2_msgs.action import FollowWaypoints, NavigateToPose
    from nav2_msgs.srv import LoadMap
    from nav_msgs.msg import OccupancyGrid, Path as NavPath
    from rclpy.action import ActionClient
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.parameter_client import AsyncParameterClient
    from rclpy.qos import (
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from std_srvs.srv import Empty
    from tf2_ros import Buffer, TransformListener
except ImportError as exc:  # pragma: no cover - depends on ROS 2 installation.
    rclpy = None
    DeserializePoseGraph = None
    SaveMap = None
    SerializePoseGraph = None
    _ROS_IMPORT_ERROR = exc
else:
    try:
        from slam_toolbox.srv import (
            DeserializePoseGraph,
            SaveMap,
            SerializePoseGraph,
        )
    except ImportError:  # pragma: no cover - optional outside SLAM hosts.
        DeserializePoseGraph = None
        SaveMap = None
        SerializePoseGraph = None
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
            self._scan_item = None
            self._scan_points: tuple[tuple[float, float], ...] = ()
            self._global_plan_item = None
            self._local_trajectory_item = None
            self._global_plan_points: tuple[tuple[float, float], ...] = ()
            self._local_trajectory_points: tuple[tuple[float, float], ...] = ()
            self._waypoint_items: list = []
            self._waypoints: tuple[tuple[float, float, float], ...] = ()
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
            self._draw_scan_points()
            self._draw_navigation_paths()
            self._draw_waypoints()
            if self._first_map:
                self.fit_map()
                self._first_map = False

        def set_scan_points(
            self, points: tuple[tuple[float, float], ...]
        ) -> None:
            self._scan_points = points
            self._draw_scan_points()

        def _draw_scan_points(self) -> None:
            if self._scan_item is not None:
                self._scene.removeItem(self._scan_item)
                self._scan_item = None
            if self._snapshot is None or not self._scan_points:
                return
            geometry = self._snapshot.geometry
            path = QPainterPath()
            radius = max(1.0, 0.035 / geometry.resolution)
            for world_x, world_y in self._scan_points:
                if not geometry.contains_world(world_x, world_y):
                    continue
                scene_x, scene_y = geometry.world_to_scene(world_x, world_y)
                path.addEllipse(
                    scene_x - radius,
                    scene_y - radius,
                    radius * 2.0,
                    radius * 2.0,
                )
            self._scan_item = self._scene.addPath(
                path,
                self._cosmetic_pen("#ff3d00", 1.2),
                QBrush(QColor("#ff6d00")),
            )
            self._scan_item.setZValue(3)

        def set_global_plan(
            self, points: tuple[tuple[float, float], ...]
        ) -> None:
            self._global_plan_points = points
            self._draw_navigation_paths()

        def set_local_trajectory(
            self, points: tuple[tuple[float, float], ...]
        ) -> None:
            self._local_trajectory_points = points
            self._draw_navigation_paths()

        def _draw_navigation_paths(self) -> None:
            for item_name in ("_global_plan_item", "_local_trajectory_item"):
                item = getattr(self, item_name)
                if item is not None:
                    self._scene.removeItem(item)
                    setattr(self, item_name, None)
            if self._snapshot is None:
                return
            self._global_plan_item = self._add_polyline(
                self._global_plan_points,
                "#14dcff",
                3.0,
                5,
            )
            self._local_trajectory_item = self._add_polyline(
                self._local_trajectory_points,
                "#ffc107",
                4.0,
                6,
            )

        def _add_polyline(
            self,
            points: tuple[tuple[float, float], ...],
            color: str,
            width: float,
            z_value: float,
        ):
            if self._snapshot is None or len(points) < 2:
                return None
            geometry = self._snapshot.geometry
            path = QPainterPath()
            first_x, first_y = geometry.world_to_scene(*points[0])
            path.moveTo(first_x, first_y)
            for world_x, world_y in points[1:]:
                scene_x, scene_y = geometry.world_to_scene(world_x, world_y)
                path.lineTo(scene_x, scene_y)
            item = self._scene.addPath(
                path,
                self._cosmetic_pen(color, width),
            )
            item.setZValue(z_value)
            return item

        def set_waypoints(
            self,
            waypoints: tuple[tuple[float, float, float], ...],
        ) -> None:
            self._waypoints = waypoints
            self._draw_waypoints()

        def _draw_waypoints(self) -> None:
            self._remove_items(self._waypoint_items)
            if self._snapshot is None or not self._waypoints:
                return
            geometry = self._snapshot.geometry
            sequence_path = QPainterPath()
            for index, (x_m, y_m, yaw_rad) in enumerate(self._waypoints):
                scene_x, scene_y = geometry.world_to_scene(x_m, y_m)
                if index == 0:
                    sequence_path.moveTo(scene_x, scene_y)
                else:
                    sequence_path.lineTo(scene_x, scene_y)
                tip_x, tip_y = geometry.world_to_scene(
                    x_m + 0.55 * math.cos(yaw_rad),
                    y_m + 0.55 * math.sin(yaw_rad),
                )
                radius = max(3.0, 0.16 / geometry.resolution)
                marker = self._scene.addEllipse(
                    scene_x - radius,
                    scene_y - radius,
                    radius * 2.0,
                    radius * 2.0,
                    self._cosmetic_pen("#ffffff", 2.0),
                    QBrush(QColor("#7b1fa2")),
                )
                heading = self._scene.addLine(
                    scene_x,
                    scene_y,
                    tip_x,
                    tip_y,
                    self._cosmetic_pen("#4a148c", 3.0),
                )
                number = self._scene.addText(str(index + 1), QFont("Sans Serif", 9))
                number.setDefaultTextColor(QColor("#7b1fa2"))
                number.setFlag(
                    QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations,
                    True,
                )
                number.setPos(scene_x + radius, scene_y - radius)
                for item in (marker, heading, number):
                    item.setZValue(7)
                self._waypoint_items.extend([marker, heading, number])
            if len(self._waypoints) >= 2:
                sequence_pen = self._cosmetic_pen("#ab47bc", 2.0)
                sequence_pen.setStyle(Qt.PenStyle.DashLine)
                sequence = self._scene.addPath(sequence_path, sequence_pen)
                sequence.setZValue(6.5)
                self._waypoint_items.append(sequence)

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
        waypoint_progress_changed = Signal(int, int)
        feedback_changed = Signal(float, float)
        localization_quality_changed = Signal(float, float, str)
        scan_received = Signal(object)
        scan_status_changed = Signal(str)
        global_plan_received = Signal(object)
        local_trajectory_received = Signal(object)
        localization_backend_changed = Signal(str)
        relocalization_changed = Signal(str)
        map_operation_changed = Signal(str)
        error = Signal(str)

        def __init__(
            self,
            map_topic: str,
            map_frame: str,
            base_frame: str,
            action_name: str,
            pose_graph: str,
            use_sim_time: bool,
        ) -> None:
            super().__init__()
            self.map_topic = map_topic
            self.map_frame = normalized_frame_id(map_frame)
            self.base_frame = base_frame
            self.action_name = action_name
            self.pose_graph = str(Path(pose_graph).expanduser().resolve())
            self.use_sim_time = use_sim_time
            self.node = None
            self.executor = None
            self.spin_timer: Optional[QTimer] = None
            self.tf_buffer = None
            self.tf_listener = None
            self.nav_client = None
            self.waypoint_client = None
            self.initial_pose_pub = None
            self.global_localization_client = None
            self.nomotion_update_client = None
            self.slam_deserialize_client = None
            self.slam_serialize_client = None
            self.slam_save_map_client = None
            self.map_server_load_client = None
            self.map_manager_parameter_client = None
            self.goal_handle = None
            self._owns_rclpy = False
            self._last_tf_emit_ns = 0
            self._server_ready = False
            self._localization_backend = ""
            self._waypoint_count = 0

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
                    lambda message: self._on_localization_pose(message, "AMCL"),
                    10,
                )
                self.node.create_subscription(
                    PoseWithCovarianceStamped,
                    "/pose",
                    lambda message: self._on_localization_pose(
                        message, "SLAM Toolbox"
                    ),
                    10,
                )
                self.node.create_subscription(
                    LaserScan,
                    "/scan",
                    self._on_scan,
                    qos_profile_sensor_data,
                )
                self.node.create_subscription(
                    NavPath,
                    "/plan",
                    lambda message: self._on_navigation_path(
                        message,
                        self.global_plan_received,
                    ),
                    10,
                )
                self.node.create_subscription(
                    NavPath,
                    "/lookahead_collision_arc",
                    lambda message: self._on_navigation_path(
                        message,
                        self.local_trajectory_received,
                    ),
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
                if DeserializePoseGraph is not None:
                    self.slam_deserialize_client = self.node.create_client(
                        DeserializePoseGraph,
                        "/slam_toolbox/deserialize_map",
                    )
                    self.slam_serialize_client = self.node.create_client(
                        SerializePoseGraph,
                        "/slam_toolbox/serialize_map",
                    )
                    self.slam_save_map_client = self.node.create_client(
                        SaveMap,
                        "/slam_toolbox/save_map",
                    )
                self.map_server_load_client = self.node.create_client(
                    LoadMap,
                    "/map_server/load_map",
                )
                self.map_manager_parameter_client = AsyncParameterClient(
                    self.node,
                    "persistent_map_manager",
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
                self.waypoint_client = ActionClient(
                    self.node,
                    FollowWaypoints,
                    "/follow_waypoints",
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
                self._update_localization_backend()
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

        def _update_localization_backend(self) -> None:
            if (
                self.slam_deserialize_client is not None
                and self.slam_deserialize_client.service_is_ready()
            ):
                backend = "slam"
            elif (
                self.global_localization_client is not None
                and self.global_localization_client.service_is_ready()
            ):
                backend = "amcl"
            else:
                backend = "waiting"
            if backend != self._localization_backend:
                self._localization_backend = backend
                self.localization_backend_changed.emit(backend)

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
            self.map_frame = normalized_frame_id(snapshot.frame_id, self.map_frame)
            self.map_received.emit(snapshot)

        def _on_localization_pose(
            self,
            message: PoseWithCovarianceStamped,
            source: str,
        ) -> None:
            position_sigma, yaw_sigma = pose_uncertainty(message.pose.covariance)
            self.localization_quality_changed.emit(
                position_sigma,
                yaw_sigma,
                source,
            )

        def _on_scan(self, message: LaserScan) -> None:
            if self.tf_buffer is None or not message.header.frame_id:
                return
            try:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.map_frame,
                        message.header.frame_id,
                        Time.from_msg(message.header.stamp),
                    )
                except Exception:
                    transform = self.tf_buffer.lookup_transform(
                        self.map_frame,
                        message.header.frame_id,
                        Time(),
                    )
            except Exception:
                self.scan_status_changed.emit(
                    f"等待 TF：{message.header.frame_id} → {self.map_frame}"
                )
                return
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            yaw_rad = math.atan2(
                2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
            )
            points = project_laser_scan(
                message.ranges,
                angle_min=message.angle_min,
                angle_increment=message.angle_increment,
                sensor_x=translation.x,
                sensor_y=translation.y,
                sensor_yaw=yaw_rad,
                range_min=max(0.0, message.range_min),
                range_max=message.range_max,
            )
            self.scan_received.emit(points)

        def _on_navigation_path(self, message: NavPath, output_signal) -> None:
            points = tuple(
                (pose.pose.position.x, pose.pose.position.y)
                for pose in message.poses
            )
            frame_id = message.header.frame_id or self.map_frame
            if not points or frame_id == self.map_frame:
                output_signal.emit(points)
                return
            if self.tf_buffer is None:
                return
            try:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.map_frame,
                        frame_id,
                        Time.from_msg(message.header.stamp),
                    )
                except Exception:
                    transform = self.tf_buffer.lookup_transform(
                        self.map_frame,
                        frame_id,
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
            cosine = math.cos(yaw_rad)
            sine = math.sin(yaw_rad)
            output_signal.emit(
                tuple(
                    (
                        translation.x + cosine * x_m - sine * y_m,
                        translation.y + sine * x_m + cosine * y_m,
                    )
                    for x_m, y_m in points
                )
            )

        @Slot(str)
        def save_map_session(self, prefix: str) -> None:
            prefix = str(Path(prefix).expanduser().resolve())
            if (
                self.slam_serialize_client is None
                or self.slam_save_map_client is None
                or not self.slam_serialize_client.service_is_ready()
                or not self.slam_save_map_client.service_is_ready()
            ):
                self.map_operation_changed.emit(
                    f"栅格地图已保存：{prefix}.yaml（当前无 SLAM pose graph 服务）"
                )
                return
            request = SerializePoseGraph.Request()
            request.filename = prefix
            self.map_operation_changed.emit(
                f"正在保存 pose graph：{prefix}.posegraph/.data"
            )
            future = self.slam_serialize_client.call_async(request)
            future.add_done_callback(
                lambda result: self._on_pose_graph_exported(result, prefix)
            )

        def _on_pose_graph_exported(self, future, prefix: str) -> None:
            try:
                response = future.result()
            except Exception as exc:
                self.map_operation_changed.emit(f"保存 pose graph 失败：{exc}")
                return
            if response.result != SerializePoseGraph.Response.RESULT_SUCCESS:
                self.map_operation_changed.emit(
                    f"保存 pose graph 失败，结果码：{response.result}"
                )
                return
            request = SaveMap.Request()
            request.name.data = prefix
            future = self.slam_save_map_client.call_async(request)
            future.add_done_callback(
                lambda result: self._on_map_exported(result, prefix)
            )

        def _on_map_exported(self, future, prefix: str) -> None:
            try:
                response = future.result()
            except Exception as exc:
                self.map_operation_changed.emit(f"保存地图图像失败：{exc}")
                return
            if response.result != SaveMap.Response.RESULT_SUCCESS:
                self.map_operation_changed.emit(
                    f"保存地图图像失败，结果码：{response.result}"
                )
                return
            self.map_operation_changed.emit(
                f"地图会话已保存：{prefix}.yaml/.pgm/.posegraph/.data"
            )

        @Slot(str)
        def load_map_session(self, yaml_path: str) -> None:
            yaml_path = str(Path(yaml_path).expanduser().resolve())
            prefix = str(Path(yaml_path).with_suffix(""))
            if (
                self.slam_deserialize_client is not None
                and self.slam_deserialize_client.service_is_ready()
            ):
                if not Path(f"{prefix}.posegraph").is_file() or not Path(
                    f"{prefix}.data"
                ).is_file():
                    self.map_operation_changed.emit(
                        "持续建图必须同时存在同名 .posegraph 和 .data 文件"
                    )
                    return
                self._cancel_active_goal()
                self.pose_graph = prefix
                parameter_client = self.map_manager_parameter_client
                if (
                    parameter_client is not None
                    and parameter_client.services_are_ready()
                ):
                    future = parameter_client.set_parameters(
                        [
                            Parameter(
                                "map_prefix",
                                Parameter.Type.STRING,
                                prefix,
                            )
                        ]
                    )
                    future.add_done_callback(
                        lambda result: self._on_map_prefix_changed(result, prefix)
                    )
                    return
                self.map_operation_changed.emit(
                    "未发现持久化管理器；载入后不会自动保存"
                )
                self._deserialize_loaded_map(prefix)
                return

            if (
                self.map_server_load_client is not None
                and self.map_server_load_client.service_is_ready()
            ):
                self._cancel_active_goal()
                request = LoadMap.Request()
                request.map_url = yaml_path
                self.map_operation_changed.emit(f"正在载入静态地图：{yaml_path}")
                future = self.map_server_load_client.call_async(request)
                future.add_done_callback(
                    lambda result: self._on_static_map_loaded(result, yaml_path)
                )
                return
            self.map_operation_changed.emit(
                "没有可用的 SLAM Toolbox 或 Map Server 载入服务"
            )

        def _on_map_prefix_changed(self, future, prefix: str) -> None:
            try:
                results = future.result()
            except Exception as exc:
                self.map_operation_changed.emit(f"切换自动保存路径失败：{exc}")
                return
            failed = next(
                (result.reason for result in results if not result.successful),
                "",
            )
            if failed:
                self.map_operation_changed.emit(f"切换自动保存路径失败：{failed}")
                return
            self._deserialize_loaded_map(prefix)

        def _deserialize_loaded_map(self, prefix: str) -> None:
            request = DeserializePoseGraph.Request()
            request.filename = prefix
            request.match_type = DeserializePoseGraph.Request.START_AT_FIRST_NODE
            self.map_operation_changed.emit(
                f"正在载入持续地图：{prefix}（按建图起点匹配）"
            )
            future = self.slam_deserialize_client.call_async(request)
            future.add_done_callback(
                lambda result: self._on_pose_graph_loaded(result, prefix)
            )

        def _on_pose_graph_loaded(self, future, prefix: str) -> None:
            try:
                future.result()
            except Exception as exc:
                self.map_operation_changed.emit(f"载入持续地图失败：{exc}")
                return
            self.map_operation_changed.emit(
                f"已载入 {prefix}；若车辆不在建图起点，请重新设置初始位姿"
            )

        def _on_static_map_loaded(self, future, yaml_path: str) -> None:
            try:
                response = future.result()
            except Exception as exc:
                self.map_operation_changed.emit(f"载入静态地图失败：{exc}")
                return
            if response.result != LoadMap.Response.RESULT_SUCCESS:
                self.map_operation_changed.emit(
                    f"载入静态地图失败，结果码：{response.result}"
                )
                return
            self.map_operation_changed.emit(
                f"静态地图已载入：{yaml_path}；请重新设置初始位姿"
            )

        @Slot(float, float, float, str)
        def set_initial_pose(
            self,
            x_m: float,
            y_m: float,
            yaw_rad: float,
            frame_id: str,
        ) -> None:
            if (
                self.slam_deserialize_client is not None
                and self.slam_deserialize_client.service_is_ready()
            ):
                self._relocalize_slam(
                    x_m,
                    y_m,
                    yaw_rad,
                    DeserializePoseGraph.Request.START_AT_GIVEN_POSE,
                    "正在重新载入持续地图，并在选中位姿附近匹配 MID-360……",
                )
                return
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
                "已发布粗略初始位姿；正在用连续 MID-360 扫描更新粒子"
            )
            self._schedule_amcl_scan_updates()

        @Slot()
        def global_relocalize(self) -> None:
            if (
                self.global_localization_client is not None
                and self.global_localization_client.service_is_ready()
            ):
                self._cancel_active_goal()
                self.relocalization_changed.emit("正在请求 AMCL 全地图粒子搜索……")
                future = self.global_localization_client.call_async(Empty.Request())
                future.add_done_callback(self._on_global_relocalize_done)
                return
            if (
                self.slam_deserialize_client is not None
                and self.slam_deserialize_client.service_is_ready()
            ):
                self._relocalize_slam(
                    0.0,
                    0.0,
                    0.0,
                    DeserializePoseGraph.Request.START_AT_FIRST_NODE,
                    "正在重新载入持续地图；请确保车辆位于原始建图起点附近……",
                )
                return
            self.error.emit("尚未发现 AMCL 或 SLAM Toolbox 重定位服务")

        def _relocalize_slam(
            self,
            x_m: float,
            y_m: float,
            yaw_rad: float,
            match_type: int,
            status: str,
        ) -> None:
            if self.slam_deserialize_client is None:
                self.error.emit("SLAM Toolbox 重定位服务尚未就绪")
                return
            if not Path(f"{self.pose_graph}.posegraph").is_file() or not Path(
                f"{self.pose_graph}.data"
            ).is_file():
                self.error.emit(
                    f"找不到持续地图 pose graph：{self.pose_graph}.posegraph/.data"
                )
                return
            self._cancel_active_goal()
            request = DeserializePoseGraph.Request()
            request.filename = self.pose_graph
            request.match_type = match_type
            request.initial_pose.x = x_m
            request.initial_pose.y = y_m
            request.initial_pose.theta = yaw_rad
            self.relocalization_changed.emit(status)
            future = self.slam_deserialize_client.call_async(request)
            future.add_done_callback(self._on_slam_relocalize_done)

        def _on_slam_relocalize_done(self, future) -> None:
            try:
                future.result()
            except Exception as exc:
                self.relocalization_changed.emit(
                    f"SLAM Toolbox 重定位请求失败：{exc}"
                )
                return
            self.relocalization_changed.emit(
                "地图已重新载入；低速前行或走大弧线，等待红色雷达点贴合墙面"
            )

        def _on_global_relocalize_done(self, future) -> None:
            try:
                future.result()
            except Exception as exc:
                self.relocalization_changed.emit(f"全局重定位请求失败：{exc}")
                return
            self.relocalization_changed.emit(
                "全局搜索已启动并自动触发扫描更新；请继续低速走大弧线"
            )
            self._schedule_amcl_scan_updates()

        def _schedule_amcl_scan_updates(self) -> None:
            for delay_ms in (300, 900, 1800):
                QTimer.singleShot(delay_ms, self._request_amcl_scan_update)

        def _request_amcl_scan_update(self) -> None:
            if (
                self.nomotion_update_client is None
                or not self.nomotion_update_client.service_is_ready()
            ):
                return
            future = self.nomotion_update_client.call_async(Empty.Request())
            future.add_done_callback(self._on_nomotion_update_done)

        @Slot()
        def request_nomotion_update(self) -> None:
            if self._localization_backend == "slam":
                self.relocalization_changed.emit(
                    "持续建图按运动触发匹配：请低速移动至少 0.10 m 或转向 0.05 rad"
                )
                return
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

        @Slot(object, str)
        def send_waypoints(
            self,
            waypoints: tuple[tuple[float, float, float], ...],
            frame_id: str,
        ) -> None:
            if (
                self.waypoint_client is None
                or not self.waypoint_client.server_is_ready()
            ):
                self.error.emit("Nav2 /follow_waypoints 尚未就绪")
                return
            if not waypoints:
                self.error.emit("路径点列表为空")
                return
            if self.goal_handle is not None:
                self.error.emit("已有导航任务，请先取消并等待任务结束")
                return
            if not all(
                math.isfinite(value)
                for waypoint in waypoints
                for value in waypoint
            ):
                self.error.emit("路径点包含无效坐标")
                return
            goal_frame = normalized_frame_id(frame_id, self.map_frame)
            goal = FollowWaypoints.Goal()
            goal.number_of_loops = 0
            goal.goal_index = 0
            for x_m, y_m, yaw_rad in waypoints:
                waypoint = PoseStamped()
                waypoint.header.frame_id = goal_frame
                waypoint.pose.position.x = x_m
                waypoint.pose.position.y = y_m
                waypoint.pose.orientation.z = math.sin(yaw_rad / 2.0)
                waypoint.pose.orientation.w = math.cos(yaw_rad / 2.0)
                goal.poses.append(waypoint)
            self._waypoint_count = len(waypoints)
            self.navigation_changed.emit(
                f"正在发送 {self._waypoint_count} 个路径点……"
            )
            future = self.waypoint_client.send_goal_async(
                goal,
                feedback_callback=self._on_waypoint_feedback,
            )
            future.add_done_callback(self._on_waypoint_goal_response)

        def _on_waypoint_goal_response(self, future) -> None:
            try:
                goal_handle = future.result()
            except Exception as exc:
                self.navigation_changed.emit(f"路径点任务发送失败：{exc}")
                return
            if goal_handle is None or not goal_handle.accepted:
                self.navigation_changed.emit("Nav2 拒绝了路径点任务")
                return
            self.goal_handle = goal_handle
            self.waypoint_progress_changed.emit(1, self._waypoint_count)
            self.navigation_changed.emit(
                f"路径点任务已接受，共 {self._waypoint_count} 点"
            )
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._on_waypoint_result)

        def _on_waypoint_feedback(self, feedback_message) -> None:
            current = int(feedback_message.feedback.current_waypoint) + 1
            self.waypoint_progress_changed.emit(current, self._waypoint_count)
            self.navigation_changed.emit(
                f"正在执行路径点 {current}/{self._waypoint_count}"
            )

        def _on_waypoint_result(self, future) -> None:
            try:
                wrapped_result = future.result()
                status = wrapped_result.status
                result = wrapped_result.result
            except Exception as exc:
                self.navigation_changed.emit(f"读取路径点任务结果失败：{exc}")
                return
            missed = [int(item.index) + 1 for item in result.missed_waypoints]
            if status == GoalStatus.STATUS_SUCCEEDED and not missed:
                message = f"全部 {self._waypoint_count} 个路径点执行完成"
            elif status == GoalStatus.STATUS_CANCELED:
                message = "路径点任务已取消"
            elif missed:
                message = "路径点任务结束，未到达：" + ", ".join(map(str, missed))
            else:
                detail = result.error_msg or f"状态码 {status}"
                message = f"路径点任务失败：{detail}"
            self.navigation_changed.emit(message)
            self.goal_handle = None

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
            if self.goal_handle is not None:
                self.error.emit("已有导航任务，请先取消并等待任务结束")
                return
            if not all(math.isfinite(value) for value in (x_m, y_m, yaw_rad)):
                self.error.emit("目标位姿包含无效坐标")
                return

            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = normalized_frame_id(
                frame_id,
                self.map_frame,
            )
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
        map_save_requested = Signal(str)
        map_load_requested = Signal(str)
        waypoints_requested = Signal(object, str)

        def __init__(
            self,
            map_topic: str,
            map_frame: str,
            base_frame: str,
            action_name: str,
            map_file: str,
            pose_graph: str,
            use_sim_time: bool,
        ) -> None:
            super().__init__()
            self.map_frame = normalized_frame_id(map_frame)
            self.snapshot: Optional[MapSnapshot] = None
            self.goal: Optional[tuple[float, float, float]] = None
            self.waypoints: list[tuple[float, float, float]] = []
            self.localization_backend = "waiting"
            self.current_map_file = str(Path(map_file).expanduser())
            self._closing = False
            self.setWindowTitle("Robot320 地图导航")
            self.resize(1280, 900)
            self._build_ui()
            self._apply_style()
            self._preload_map(map_file)

            self.worker_thread = QThread(self)
            self.worker = NavigationWorker(
                map_topic,
                map_frame,
                base_frame,
                action_name,
                pose_graph,
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
            self.map_save_requested.connect(self.worker.save_map_session)
            self.map_load_requested.connect(self.worker.load_map_session)
            self.waypoints_requested.connect(self.worker.send_waypoints)
            self.worker.map_received.connect(self._on_map)
            self.worker.pose_received.connect(self._on_pose)
            self.worker.scan_received.connect(self._on_scan_points)
            self.worker.scan_status_changed.connect(self.scan_value.setText)
            self.worker.global_plan_received.connect(self._on_global_plan)
            self.worker.local_trajectory_received.connect(
                self._on_local_trajectory
            )
            self.worker.connection_changed.connect(self._on_connection)
            self.worker.navigation_changed.connect(self.navigation_value.setText)
            self.worker.waypoint_progress_changed.connect(
                self._on_waypoint_progress
            )
            self.worker.feedback_changed.connect(self._on_feedback)
            self.worker.localization_quality_changed.connect(
                self._on_localization_quality
            )
            self.worker.localization_backend_changed.connect(
                self._on_localization_backend
            )
            self.worker.relocalization_changed.connect(
                self.relocalization_value.setText
            )
            self.worker.map_operation_changed.connect(self._on_map_operation)
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
            self.localization_quality_value = QLabel("等待定位协方差")
            self.localization_backend_value = QLabel("等待定位后端")
            self.scan_value = QLabel("等待 /scan 与地图 TF")
            self.cursor_value = QLabel("--")
            for value_label in (
                self.map_value,
                self.pose_value,
                self.localization_quality_value,
                self.localization_backend_value,
                self.scan_value,
                self.cursor_value,
            ):
                self._configure_value_label(value_label)
            map_layout.addRow("地图", self.map_value)
            map_layout.addRow("机器人", self.pose_value)
            map_layout.addRow("定位后端", self.localization_backend_value)
            map_layout.addRow("定位置信度", self.localization_quality_value)
            map_layout.addRow("雷达匹配", self.scan_value)
            map_layout.addRow("鼠标", self.cursor_value)
            layout.addWidget(map_group)

            map_file_group = QGroupBox("地图文件")
            map_file_layout = QVBoxLayout(map_file_group)
            self.map_file_value = QLabel(self.current_map_file)
            self._configure_value_label(self.map_file_value)
            save_map_button = QPushButton("保存当前地图…")
            save_map_button.setMinimumHeight(38)
            save_map_button.clicked.connect(self._save_current_map)
            load_map_button = QPushButton("载入已有地图…")
            load_map_button.setMinimumHeight(38)
            load_map_button.clicked.connect(self._load_map_file)
            self.map_operation_value = QLabel("可保存或切换 ROS 地图会话")
            self._configure_value_label(self.map_operation_value)
            map_file_layout.addWidget(self.map_file_value)
            map_file_layout.addWidget(save_map_button)
            map_file_layout.addWidget(load_map_button)
            map_file_layout.addWidget(self.map_operation_value)
            layout.addWidget(map_file_group)

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

            waypoint_group = QGroupBox("路径点任务")
            waypoint_layout = QVBoxLayout(waypoint_group)
            self.waypoint_list = QListWidget()
            self.waypoint_list.setMinimumHeight(110)
            self.waypoint_list.currentRowChanged.connect(
                self._on_waypoint_selected
            )
            add_waypoint_button = QPushButton("添加当前选中位姿")
            add_waypoint_button.clicked.connect(self._add_waypoint)
            order_buttons = QHBoxLayout()
            move_up_button = QPushButton("上移")
            move_up_button.clicked.connect(lambda: self._move_waypoint(-1))
            move_down_button = QPushButton("下移")
            move_down_button.clicked.connect(lambda: self._move_waypoint(1))
            order_buttons.addWidget(move_up_button)
            order_buttons.addWidget(move_down_button)
            edit_buttons = QHBoxLayout()
            remove_waypoint_button = QPushButton("删除")
            remove_waypoint_button.clicked.connect(self._remove_waypoint)
            clear_waypoints_button = QPushButton("清空")
            clear_waypoints_button.clicked.connect(self._clear_waypoints)
            edit_buttons.addWidget(remove_waypoint_button)
            edit_buttons.addWidget(clear_waypoints_button)
            self.execute_waypoints_button = QPushButton("依次执行全部路径点")
            self.execute_waypoints_button.setObjectName("primary")
            self.execute_waypoints_button.setMinimumHeight(44)
            self.execute_waypoints_button.setEnabled(False)
            self.execute_waypoints_button.clicked.connect(
                self._execute_waypoints
            )
            self.waypoint_status_value = QLabel("尚未添加路径点")
            self._configure_value_label(self.waypoint_status_value)
            waypoint_layout.addWidget(self.waypoint_list)
            waypoint_layout.addWidget(add_waypoint_button)
            waypoint_layout.addLayout(order_buttons)
            waypoint_layout.addLayout(edit_buttons)
            waypoint_layout.addWidget(self.execute_waypoints_button)
            waypoint_layout.addWidget(self.waypoint_status_value)
            layout.addWidget(waypoint_group)

            relocalization_group = QGroupBox("重定位")
            relocalization_layout = QVBoxLayout(relocalization_group)
            set_initial_pose_button = QPushButton("将选中位姿设为初始位置")
            set_initial_pose_button.clicked.connect(self._set_initial_pose)
            self.global_relocalize_button = QPushButton("不知道位置：全局重定位")
            self.global_relocalize_button.clicked.connect(
                lambda _checked=False: self.global_relocalize_requested.emit()
            )
            self.nomotion_update_button = QPushButton("使用当前扫描强制更新")
            self.nomotion_update_button.clicked.connect(
                lambda _checked=False: self.nomotion_update_requested.emit()
            )
            for button in (
                set_initial_pose_button,
                self.global_relocalize_button,
                self.nomotion_update_button,
            ):
                button.setMinimumHeight(38)
            self.relocalization_value = QLabel("可设置粗略位置或启动全地图搜索")
            self._configure_value_label(self.relocalization_value)
            relocalization_layout.addWidget(set_initial_pose_button)
            relocalization_layout.addWidget(self.global_relocalize_button)
            relocalization_layout.addWidget(self.nomotion_update_button)
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
            self.global_plan_value = QLabel("全局路径：等待 /plan")
            self.local_trajectory_value = QLabel(
                "局部轨迹：等待 /lookahead_collision_arc"
            )
            self._configure_value_label(self.navigation_value)
            self._configure_value_label(self.feedback_value)
            self._configure_value_label(self.global_plan_value)
            self._configure_value_label(self.local_trajectory_value)
            status_layout.addWidget(self.navigation_value)
            status_layout.addWidget(self.feedback_value)
            status_layout.addWidget(self.global_plan_value)
            status_layout.addWidget(self.local_trajectory_value)
            layout.addWidget(status_group)

            help_text = QLabel(
                "操作：在可通行区域按下鼠标左键确定目标点，拖动决定车头朝向，"
                "松开后可将它用作初始位姿或导航目标。红点是 MID-360 扫描经当前"
                "定位变换后的地图匹配结果。持续建图时可选择灰色边界作为探索目标；"
                "青线是 Nav2 全局路径，黄线是控制器局部前视轨迹。静态定位时灰色"
                "区域不会更新。紫色编号标记是路径点，拖动选好方向后逐个添加并排序。"
                "滚轮缩放，中键拖动平移地图。"
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

        def _save_current_map(self) -> None:
            if self.snapshot is None:
                self._on_error("尚未收到或预载地图，无法保存")
                return
            default_path = self.current_map_file or str(
                Path.home() / "robot320_maps" / "patrol_current.yaml"
            )
            selected_path, _selected_filter = QFileDialog.getSaveFileName(
                self,
                "保存当前 Robot320 地图",
                default_path,
                "ROS 地图 (*.yaml);;所有文件 (*)",
            )
            if not selected_path:
                return
            try:
                yaml_path, _pgm_path = save_map_yaml(
                    self.snapshot,
                    selected_path,
                )
            except Exception as exc:
                QMessageBox.critical(self, "地图保存失败", str(exc))
                return
            self.current_map_file = str(yaml_path)
            self.map_file_value.setText(self.current_map_file)
            prefix = str(yaml_path.with_suffix(""))
            self.map_operation_value.setText("栅格地图已保存，正在保存完整会话……")
            self.map_save_requested.emit(prefix)

        def _load_map_file(self) -> None:
            start_path = self.current_map_file or str(
                Path.home() / "robot320_maps"
            )
            selected_path, _selected_filter = QFileDialog.getOpenFileName(
                self,
                "载入 Robot320 地图",
                start_path,
                "ROS 地图 (*.yaml *.yml);;所有文件 (*)",
            )
            if not selected_path:
                return
            confirmation = QMessageBox.question(
                self,
                "确认载入地图",
                "载入会取消当前导航并替换后端地图。\n"
                "持续建图模式要求同目录存在同名 .posegraph 和 .data。\n"
                "是否继续？",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No,
            )
            if confirmation != QMessageBox.StandardButton.Yes:
                return
            try:
                snapshot = load_map_yaml(selected_path, self.map_frame)
            except Exception as exc:
                QMessageBox.critical(self, "地图载入失败", str(exc))
                return
            self.current_map_file = str(Path(selected_path).expanduser().resolve())
            self.map_file_value.setText(self.current_map_file)
            self._clear_waypoints()
            self._display_map(snapshot, f"文件预览 · {Path(selected_path).name}")
            self.map_operation_value.setText("文件已读取，正在切换 ROS 后端地图……")
            self.map_load_requested.emit(self.current_map_file)

        @Slot(str)
        def _on_map_operation(self, message: str) -> None:
            self.map_operation_value.setText(message)
            self.statusBar().showMessage(message, 10000)

        def _preload_map(self, map_file: str) -> None:
            if not map_file:
                return
            path = Path(map_file).expanduser()
            if not path.is_file():
                self.map_value.setText(f"等待 /map；未找到预载地图 {path}")
                return
            try:
                snapshot = load_map_yaml(path, self.map_frame)
            except Exception as exc:
                self.map_value.setText(f"预载地图失败：{exc}")
                return
            self._display_map(snapshot, f"本地预载 · {path.name}")

        @Slot(object)
        def _on_map(self, snapshot: MapSnapshot) -> None:
            self._display_map(snapshot, "ROS 实时地图")

        def _display_map(self, snapshot: MapSnapshot, source: str) -> None:
            self.snapshot = snapshot
            self.map_frame = normalized_frame_id(snapshot.frame_id, self.map_frame)
            self.map_view.set_map(snapshot)
            geometry = snapshot.geometry
            self.map_value.setText(
                f"{source} · {geometry.width} × {geometry.height}，"
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

        @Slot(object)
        def _on_scan_points(
            self,
            points: tuple[tuple[float, float], ...],
        ) -> None:
            self.map_view.set_scan_points(points)
            if self.snapshot is None:
                self.scan_value.setText(f"{len(points)} 个匹配点 · 红色")
                return
            matched, evaluated = scan_alignment_score(self.snapshot, points)
            ratio = 100.0 * matched / evaluated if evaluated else 0.0
            self.scan_value.setText(
                f"{len(points)} 个红色点 · 贴墙率 {ratio:.0f}%"
            )

        @Slot(object)
        def _on_global_plan(
            self,
            points: tuple[tuple[float, float], ...],
        ) -> None:
            self.map_view.set_global_plan(points)
            self.global_plan_value.setText(
                f"全局路径：{len(points)} 点 · "
                f"{polyline_length(points):.2f} m · 青色"
            )

        @Slot(object)
        def _on_local_trajectory(
            self,
            points: tuple[tuple[float, float], ...],
        ) -> None:
            self.map_view.set_local_trajectory(points)
            self.local_trajectory_value.setText(
                f"局部轨迹：{len(points)} 点 · "
                f"{polyline_length(points):.2f} m · 黄色"
            )

        def _add_waypoint(self) -> None:
            selected_pose = self._validated_selected_pose()
            if selected_pose is None:
                return
            self.waypoints.append(selected_pose)
            self._refresh_waypoints(len(self.waypoints) - 1)
            self.waypoint_status_value.setText(
                f"已添加路径点 {len(self.waypoints)}"
            )

        def _remove_waypoint(self) -> None:
            row = self.waypoint_list.currentRow()
            if not 0 <= row < len(self.waypoints):
                self._on_error("请先选择要删除的路径点")
                return
            self.waypoints.pop(row)
            self._refresh_waypoints(min(row, len(self.waypoints) - 1))

        def _clear_waypoints(self) -> None:
            self.waypoints.clear()
            self._refresh_waypoints()
            self.waypoint_status_value.setText("路径点列表已清空")

        def _move_waypoint(self, offset: int) -> None:
            row = self.waypoint_list.currentRow()
            target = row + offset
            if not 0 <= row < len(self.waypoints) or not 0 <= target < len(
                self.waypoints
            ):
                return
            self.waypoints[row], self.waypoints[target] = (
                self.waypoints[target],
                self.waypoints[row],
            )
            self._refresh_waypoints(target)

        def _refresh_waypoints(self, selected_row: int = -1) -> None:
            self.waypoint_list.blockSignals(True)
            self.waypoint_list.clear()
            for index, (x_m, y_m, yaw_rad) in enumerate(self.waypoints, start=1):
                self.waypoint_list.addItem(
                    f"{index:02d}  x={x_m:.2f}  y={y_m:.2f}  "
                    f"yaw={math.degrees(yaw_rad):.1f}°"
                )
            self.waypoint_list.blockSignals(False)
            self.map_view.set_waypoints(tuple(self.waypoints))
            self.execute_waypoints_button.setEnabled(bool(self.waypoints))
            if 0 <= selected_row < len(self.waypoints):
                self.waypoint_list.setCurrentRow(selected_row)
            if self.waypoints:
                self.waypoint_status_value.setText(
                    f"共 {len(self.waypoints)} 个路径点，可调整顺序后执行"
                )
            else:
                self.waypoint_status_value.setText("尚未添加路径点")

        @Slot(int)
        def _on_waypoint_selected(self, row: int) -> None:
            if not 0 <= row < len(self.waypoints):
                return
            x_m, y_m, yaw_rad = self.waypoints[row]
            self.goal = (x_m, y_m, yaw_rad)
            self.goal_x.setValue(x_m)
            self.goal_y.setValue(y_m)
            self.goal_yaw.setValue(math.degrees(yaw_rad))
            self.map_view.set_goal(x_m, y_m, yaw_rad)
            self.send_button.setEnabled(True)

        def _execute_waypoints(self) -> None:
            if self.snapshot is None or not self.waypoints:
                self._on_error("请先添加路径点")
                return
            allow_unknown = self.localization_backend == "slam"
            for index, (x_m, y_m, _yaw_rad) in enumerate(
                self.waypoints,
                start=1,
            ):
                if not self.snapshot.is_traversable(
                    x_m,
                    y_m,
                    allow_unknown=allow_unknown,
                ):
                    self._on_error(f"路径点 {index} 当前不可通行，请调整")
                    return
            self.waypoint_status_value.setText(
                f"正在发送 {len(self.waypoints)} 个路径点……"
            )
            self.waypoints_requested.emit(
                tuple(self.waypoints),
                self.map_frame,
            )

        @Slot(int, int)
        def _on_waypoint_progress(self, current: int, total: int) -> None:
            self.waypoint_status_value.setText(
                f"正在执行路径点 {current}/{total}"
            )
            if 1 <= current <= len(self.waypoints):
                self.waypoint_list.setCurrentRow(current - 1)

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
            allow_unknown = self.localization_backend == "slam"
            if not self.snapshot.is_traversable(
                x_m,
                y_m,
                allow_unknown=allow_unknown,
            ):
                if occupancy < 0:
                    self._on_error(
                        "当前是静态定位模式，灰色区域不会更新；"
                        "请切换 mode:=continuing 后发送探索目标"
                    )
                    return None
                description = "未知区域" if occupancy < 0 else f"占用值 {occupancy}"
                self._on_error(f"选中位置位于障碍物或{description}，请重新选择")
                return None
            if occupancy < 0:
                self.navigation_value.setText(
                    "灰色探索目标已接受；建议选择未知边界附近，不要直接跨越大片未知区"
                )
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
            source: str,
        ) -> None:
            yaw_sigma_deg = math.degrees(yaw_sigma_rad)
            if position_sigma_m <= 0.25 and yaw_sigma_deg <= 10.0:
                level = "良好"
            elif position_sigma_m <= 0.75 and yaw_sigma_deg <= 25.0:
                level = "正在收敛"
            else:
                level = "不确定"
            self.localization_quality_value.setText(
                f"{source} · {level} · σ位置={position_sigma_m:.2f} m，"
                f"σ朝向={yaw_sigma_deg:.1f}°"
            )

        @Slot(str)
        def _on_localization_backend(self, backend: str) -> None:
            self.localization_backend = backend
            if backend == "slam":
                self.localization_backend_value.setText(
                    "SLAM Toolbox · 持续建图/雷达匹配"
                )
                self.global_relocalize_button.setText(
                    "回到建图起点：雷达重匹配"
                )
                self.nomotion_update_button.setText("如何触发下一次雷达匹配")
            elif backend == "amcl":
                self.localization_backend_value.setText("AMCL · 静态地图定位")
                self.global_relocalize_button.setText("不知道位置：全局重定位")
                self.nomotion_update_button.setText("使用当前扫描强制更新")
            else:
                self.localization_backend_value.setText("等待定位服务")

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
    parser.add_argument(
        "--map-file",
        default="~/robot320_maps/patrol_current.yaml",
        help="Map-server YAML to display before ROS publishes /map; empty disables.",
    )
    parser.add_argument(
        "--pose-graph",
        default="~/robot320_maps/patrol_current",
        help="SLAM Toolbox serialized pose-graph prefix used for relocalization.",
    )
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
        args.map_file,
        args.pose_graph,
        args.use_sim_time,
    )
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
