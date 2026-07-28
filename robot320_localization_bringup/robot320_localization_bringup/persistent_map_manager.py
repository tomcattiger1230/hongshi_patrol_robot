"""Periodically persist both the SLAM pose graph and occupancy map."""

from __future__ import annotations

from pathlib import Path
import time

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from std_srvs.srv import Trigger


class PersistentMapManager(Node):
    """Save a continuing SLAM session without blocking scan processing."""

    def __init__(self) -> None:
        super().__init__("persistent_map_manager")
        prefix = str(self.declare_parameter("map_prefix", "").value)
        self._interval = float(
            self.declare_parameter("save_interval", 30.0).value
        )
        self._startup_delay = float(
            self.declare_parameter("startup_delay", 15.0).value
        )
        if not prefix:
            raise ValueError("map_prefix must not be empty")
        if self._interval <= 0.0:
            raise ValueError("save_interval must be greater than zero")

        self._prefix = str(Path(prefix).expanduser().resolve())
        Path(self._prefix).parent.mkdir(parents=True, exist_ok=True)
        self._serialize_client = self.create_client(
            SerializePoseGraph, "/slam_toolbox/serialize_map"
        )
        self._map_client = self.create_client(
            SaveMap, "/slam_toolbox/save_map"
        )
        self._busy = False
        self._started_at = time.monotonic()
        self._last_save = self._started_at
        self.add_on_set_parameters_callback(self._on_parameters)
        self.create_timer(1.0, self._tick)
        self.create_service(
            Trigger, "/robot320/save_persistent_map", self._save_service
        )
        self.get_logger().info(
            f"Persistent SLAM output: {self._prefix} "
            f"(every {self._interval:.0f} s)"
        )

    def _on_parameters(self, parameters) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name != "map_prefix":
                continue
            if self._busy:
                return SetParametersResult(
                    successful=False,
                    reason="A persistent map save is currently running",
                )
            prefix = str(parameter.value).strip()
            if not prefix:
                return SetParametersResult(
                    successful=False,
                    reason="map_prefix must not be empty",
                )
            self._prefix = str(Path(prefix).expanduser().resolve())
            Path(self._prefix).parent.mkdir(parents=True, exist_ok=True)
            self._last_save = time.monotonic()
            self.get_logger().info(
                f"Persistent SLAM output changed to: {self._prefix}"
            )
        return SetParametersResult(successful=True)

    def _services_ready(self) -> bool:
        return (
            self._serialize_client.service_is_ready()
            and self._map_client.service_is_ready()
        )

    def _tick(self) -> None:
        now = time.monotonic()
        if now - self._started_at < self._startup_delay:
            return
        if now - self._last_save >= self._interval:
            self._request_save()

    def _save_service(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        if self._busy:
            response.success = False
            response.message = "A persistent map save is already running"
            return response
        if not self._services_ready():
            response.success = False
            response.message = "SLAM Toolbox save services are not ready"
            return response
        self._request_save()
        response.success = True
        response.message = f"Saving pose graph and map to {self._prefix}"
        return response

    def _request_save(self) -> None:
        if self._busy or not self._services_ready():
            return
        self._busy = True
        request = SerializePoseGraph.Request()
        request.filename = self._prefix
        future = self._serialize_client.call_async(request)
        future.add_done_callback(self._posegraph_saved)

    def _posegraph_saved(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:  # pragma: no cover - ROS transport failure
            self.get_logger().error(f"Pose graph save failed: {error}")
            self._busy = False
            return
        if response.result != SerializePoseGraph.Response.RESULT_SUCCESS:
            self.get_logger().error(
                f"Pose graph save returned result {response.result}"
            )
            self._busy = False
            return

        request = SaveMap.Request()
        request.name.data = self._prefix
        future = self._map_client.call_async(request)
        future.add_done_callback(self._map_saved)

    def _map_saved(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:  # pragma: no cover - ROS transport failure
            self.get_logger().error(f"Occupancy map save failed: {error}")
            self._busy = False
            return
        self._busy = False
        if response.result != SaveMap.Response.RESULT_SUCCESS:
            self.get_logger().error(
                f"Occupancy map save returned result {response.result}"
            )
            return
        self._last_save = time.monotonic()
        self.get_logger().info(
            f"Saved persistent pose graph and occupancy map: {self._prefix}"
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PersistentMapManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
