"""Qt-independent map geometry helpers for the navigation GUI."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class MapGeometry:
    """Geometry of a ROS ``nav_msgs/OccupancyGrid``."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("map width and height must be positive")
        if self.resolution <= 0.0:
            raise ValueError("map resolution must be positive")

    def grid_to_world(self, grid_x: float, grid_y: float) -> tuple[float, float]:
        """Convert continuous grid coordinates to map-frame coordinates."""
        local_x = grid_x * self.resolution
        local_y = grid_y * self.resolution
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        return (
            self.origin_x + cosine * local_x - sine * local_y,
            self.origin_y + sine * local_x + cosine * local_y,
        )

    def world_to_grid(self, world_x: float, world_y: float) -> tuple[float, float]:
        """Convert map-frame coordinates to continuous grid coordinates."""
        delta_x = world_x - self.origin_x
        delta_y = world_y - self.origin_y
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        return (
            (cosine * delta_x + sine * delta_y) / self.resolution,
            (-sine * delta_x + cosine * delta_y) / self.resolution,
        )

    def scene_to_world(self, scene_x: float, scene_y: float) -> tuple[float, float]:
        """Convert Qt scene pixels (top-left origin) to map coordinates."""
        return self.grid_to_world(scene_x, self.height - scene_y)

    def world_to_scene(self, world_x: float, world_y: float) -> tuple[float, float]:
        """Convert map coordinates to Qt scene pixels (top-left origin)."""
        grid_x, grid_y = self.world_to_grid(world_x, world_y)
        return grid_x, self.height - grid_y

    def contains_world(self, world_x: float, world_y: float) -> bool:
        grid_x, grid_y = self.world_to_grid(world_x, world_y)
        return 0.0 <= grid_x < self.width and 0.0 <= grid_y < self.height


@dataclass(frozen=True)
class MapSnapshot:
    """Immutable map payload safe to pass from a ROS thread to Qt."""

    geometry: MapGeometry
    data: tuple[int, ...]
    frame_id: str = "map"

    def __post_init__(self) -> None:
        expected = self.geometry.width * self.geometry.height
        if len(self.data) != expected:
            raise ValueError(f"map has {len(self.data)} cells, expected {expected}")

    def occupancy_at_world(self, world_x: float, world_y: float) -> int | None:
        """Return the occupancy value at a map point, or ``None`` outside."""
        grid_x, grid_y = self.geometry.world_to_grid(world_x, world_y)
        column = math.floor(grid_x)
        row = math.floor(grid_y)
        if not (0 <= column < self.geometry.width and 0 <= row < self.geometry.height):
            return None
        return self.data[row * self.geometry.width + column]

    def is_traversable(
        self,
        world_x: float,
        world_y: float,
        occupied_threshold: int = 65,
    ) -> bool:
        """Apply a simple GUI-side occupied/unknown cell check."""
        occupancy = self.occupancy_at_world(world_x, world_y)
        return occupancy is not None and 0 <= occupancy < occupied_threshold


def map_snapshot(
    *,
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
    data: Sequence[int],
    frame_id: str = "map",
) -> MapSnapshot:
    """Build and validate a snapshot from a ROS OccupancyGrid-like payload."""
    return MapSnapshot(
        geometry=MapGeometry(
            width=width,
            height=height,
            resolution=resolution,
            origin_x=origin_x,
            origin_y=origin_y,
            origin_yaw=origin_yaw,
        ),
        data=tuple(int(value) for value in data),
        frame_id=frame_id or "map",
    )


def yaw_from_quaternion(z: float, w: float) -> float:
    """Return planar yaw for an OccupancyGrid origin quaternion."""
    return math.atan2(2.0 * z * w, 1.0 - 2.0 * z * z)


def goal_yaw(
    start_x: float,
    start_y: float,
    end_x: float,
    end_y: float,
    fallback: float = 0.0,
    minimum_drag_m: float = 0.05,
) -> float:
    """Compute goal heading from a click-drag gesture."""
    delta_x = end_x - start_x
    delta_y = end_y - start_y
    if math.hypot(delta_x, delta_y) < minimum_drag_m:
        return fallback
    return math.atan2(delta_y, delta_x)
