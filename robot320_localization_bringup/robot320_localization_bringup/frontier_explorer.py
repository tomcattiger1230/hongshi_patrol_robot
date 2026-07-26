"""Autonomously send Nav2 goals to occupancy-grid frontiers."""

from __future__ import annotations

import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener

from .frontier import choose_frontier, find_frontiers


class FrontierExplorer(Node):
    """Select safe frontiers and let Nav2 perform collision-aware driving."""

    def __init__(self) -> None:
        super().__init__("frontier_explorer")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("navigation_action", "/navigate_to_pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("min_frontier_size", 8)
        self.declare_parameter("clearance_radius", 1.25)
        self.declare_parameter("goal_timeout", 90.0)
        self.declare_parameter("retry_radius", 1.0)

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._map: OccupancyGrid | None = None
        self._goal_active = False
        self._current_goal_handle = None
        self._goal_started = self.get_clock().now()
        self._failed_goals: list[tuple[float, float]] = []
        self._empty_cycles = 0
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._navigator = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter("navigation_action").value),
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self._map_callback,
            map_qos,
        )
        self.create_timer(2.0, self._tick)
        self.get_logger().info(
            "Autonomous frontier mapping enabled; waiting for /map and Nav2"
        )

    def _map_callback(self, message: OccupancyGrid) -> None:
        self._map = message

    def _tick(self) -> None:
        if self._goal_active:
            timeout = float(self.get_parameter("goal_timeout").value)
            if self.get_clock().now() - self._goal_started > Duration(seconds=timeout):
                self.get_logger().warning("Frontier goal timed out; cancelling")
                if self._current_goal_handle is not None:
                    self._current_goal_handle.cancel_goal_async()
                self._goal_active = False
            return
        if self._map is None or not self._navigator.server_is_ready():
            return

        map_frame = str(self.get_parameter("map_frame").value)
        robot_frame = str(self.get_parameter("robot_frame").value)
        try:
            transform = self._tf_buffer.lookup_transform(
                map_frame,
                robot_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as error:
            self.get_logger().warning(
                f"Waiting for {map_frame}->{robot_frame}: {error}",
                throttle_duration_sec=5.0,
            )
            return

        resolution = self._map.info.resolution
        clearance_cells = max(
            1,
            math.ceil(
                float(self.get_parameter("clearance_radius").value) / resolution
            ),
        )
        frontiers = find_frontiers(
            self._map.data,
            self._map.info.width,
            self._map.info.height,
            min_size=int(self.get_parameter("min_frontier_size").value),
            clearance_cells=clearance_cells,
        )
        origin = self._map.info.origin.position
        robot_column = (transform.transform.translation.x - origin.x) / resolution
        robot_row = (transform.transform.translation.y - origin.y) / resolution
        retry_radius = float(self.get_parameter("retry_radius").value)
        frontiers = [
            frontier
            for frontier in frontiers
            if all(
                math.hypot(
                    origin.x + (frontier.column + 0.5) * resolution - failed_x,
                    origin.y + (frontier.row + 0.5) * resolution - failed_y,
                )
                > retry_radius
                for failed_x, failed_y in self._failed_goals
            )
        ]
        frontier = choose_frontier(frontiers, robot_row, robot_column)
        if frontier is None:
            self._empty_cycles += 1
            if self._empty_cycles == 3:
                self.get_logger().info(
                    "No reachable frontiers remain; autonomous mapping is complete"
                )
            return
        self._empty_cycles = 0

        goal_x = origin.x + (frontier.column + 0.5) * resolution
        goal_y = origin.y + (frontier.row + 0.5) * resolution
        yaw = math.atan2(
            goal_y - transform.transform.translation.y,
            goal_x - transform.transform.translation.x,
        )
        target = PoseStamped()
        target.header.frame_id = map_frame
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose.position.x = goal_x
        target.pose.position.y = goal_y
        target.pose.orientation.z = math.sin(yaw / 2.0)
        target.pose.orientation.w = math.cos(yaw / 2.0)

        request = NavigateToPose.Goal()
        request.pose = target
        self._goal_active = True
        self._goal_started = self.get_clock().now()
        future = self._navigator.send_goal_async(request)
        future.add_done_callback(
            lambda result, x=goal_x, y=goal_y: self._goal_response(result, x, y)
        )
        self.get_logger().info(
            f"Exploring frontier ({goal_x:.2f}, {goal_y:.2f}), "
            f"gain={frontier.size} cells"
        )

    def _goal_response(self, future, goal_x: float, goal_y: float) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._failed_goals.append((goal_x, goal_y))
            self._goal_active = False
            return
        self._current_goal_handle = goal_handle
        result = goal_handle.get_result_async()
        result.add_done_callback(
            lambda completed, x=goal_x, y=goal_y: self._goal_result(
                completed, x, y
            )
        )

    def _goal_result(self, future, goal_x: float, goal_y: float) -> None:
        status = future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._failed_goals.append((goal_x, goal_y))
            self.get_logger().warning(
                f"Frontier goal failed with status {status}; selecting another"
            )
        self._current_goal_handle = None
        self._goal_active = False


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
