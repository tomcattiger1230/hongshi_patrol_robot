"""Restamp simulated laser scans after expensive point-cloud projection."""

import copy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanRestamper(Node):
    """Publish projected scans with the current simulation timestamp."""

    def __init__(self) -> None:
        super().__init__("scan_restamper")
        self.declare_parameter("input_topic", "/scan_raw")
        self.declare_parameter("output_topic", "/scan")
        self.declare_parameter("visualization_topic", "")
        self.declare_parameter("visualization_no_return_range", 8.0)
        self.declare_parameter("frame_id", "")
        self.declare_parameter("sensor_x", 0.0)
        self.declare_parameter("sensor_y", 0.0)
        self.declare_parameter("self_filter_x_min", 0.0)
        self.declare_parameter("self_filter_x_max", 0.0)
        self.declare_parameter("self_filter_y_abs", 0.0)
        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        visualization_topic = str(self.get_parameter("visualization_topic").value)
        self._visualization_no_return_range = float(
            self.get_parameter("visualization_no_return_range").value
        )
        self._frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        self._sensor_x = float(self.get_parameter("sensor_x").value)
        self._sensor_y = float(self.get_parameter("sensor_y").value)
        self._self_filter_x_min = float(
            self.get_parameter("self_filter_x_min").value
        )
        self._self_filter_x_max = float(
            self.get_parameter("self_filter_x_max").value
        )
        self._self_filter_y_abs = float(
            self.get_parameter("self_filter_y_abs").value
        )
        self._publisher = self.create_publisher(LaserScan, output_topic, 10)
        self._visualization_publisher = None
        if visualization_topic:
            self._visualization_publisher = self.create_publisher(
                LaserScan, visualization_topic, 10
            )
        self.create_subscription(
            LaserScan,
            input_topic,
            self._on_scan,
            qos_profile_sensor_data,
        )

    def _on_scan(self, scan: LaserScan) -> None:
        scan.header.stamp = self.get_clock().now().to_msg()
        if self._frame_id:
            # Gazebo Harmonic scopes GPU lidar frame names by model and sensor.
            # The scan coordinates still use the URDF sensor axes, so expose the
            # stable ROS frame consumed by SLAM, Nav2, and Isaac Sim.
            scan.header.frame_id = self._frame_id
        if (
            self._self_filter_x_max > self._self_filter_x_min
            and self._self_filter_y_abs > 0.0
        ):
            for index, distance in enumerate(scan.ranges):
                if not math.isfinite(distance):
                    continue
                angle = scan.angle_min + index * scan.angle_increment
                base_x = self._sensor_x + distance * math.cos(angle)
                base_y = self._sensor_y + distance * math.sin(angle)
                if (
                    self._self_filter_x_min <= base_x <= self._self_filter_x_max
                    and abs(base_y) <= self._self_filter_y_abs
                ):
                    scan.ranges[index] = math.inf
        self._publisher.publish(scan)
        if self._visualization_publisher is not None:
            # Keep no-return rays as infinity on the navigation topic: SLAM and
            # Nav2 must not mistake empty space for an obstacle.  RViz does not
            # draw infinite ranges, however, so publish a separate display-only
            # scan with those rays placed on a finite reference ring.
            visual_scan = copy.deepcopy(scan)
            no_return_range = min(
                max(self._visualization_no_return_range, scan.range_min),
                scan.range_max,
            )
            visual_scan.ranges = [
                distance if math.isfinite(distance) else no_return_range
                for distance in scan.ranges
            ]
            self._visualization_publisher.publish(visual_scan)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanRestamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
