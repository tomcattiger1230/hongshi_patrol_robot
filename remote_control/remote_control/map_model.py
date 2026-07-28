"""Qt-independent map geometry helpers for the navigation GUI."""

from __future__ import annotations

import ast
from dataclasses import dataclass
import math
from pathlib import Path
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


def pose_uncertainty(covariance: Sequence[float]) -> tuple[float, float]:
    """Return conservative planar position and yaw standard deviations."""
    if len(covariance) != 36:
        raise ValueError("pose covariance must contain 36 values")
    position_variance = max(0.0, float(covariance[0]), float(covariance[7]))
    yaw_variance = max(0.0, float(covariance[35]))
    return math.sqrt(position_variance), math.sqrt(yaw_variance)


def load_map_yaml(path: str | Path, frame_id: str = "map") -> MapSnapshot:
    """Load a ROS map-server YAML and PGM without requiring PyYAML."""
    yaml_path = Path(path).expanduser().resolve()
    values: dict[str, str] = {}
    for raw_line in yaml_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        values[key.strip()] = value.strip()

    required = {"image", "resolution", "origin"}
    missing = sorted(required.difference(values))
    if missing:
        raise ValueError(f"map YAML is missing: {', '.join(missing)}")

    image_value = values["image"]
    if image_value[:1] in {"'", '"'}:
        image_value = str(ast.literal_eval(image_value))
    image_path = Path(image_value).expanduser()
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path

    width, height, pixels = _load_pgm(image_path)
    origin = ast.literal_eval(values["origin"])
    if not isinstance(origin, (list, tuple)) or len(origin) != 3:
        raise ValueError("map origin must be [x, y, yaw]")
    negate = bool(int(values.get("negate", "0")))
    occupied_threshold = float(values.get("occupied_thresh", "0.65"))
    free_threshold = float(values.get("free_thresh", "0.196"))

    data: list[int] = []
    for grid_y in range(height):
        image_row = height - 1 - grid_y
        row_offset = image_row * width
        for column in range(width):
            shade = pixels[row_offset + column]
            occupancy = shade / 255.0 if negate else (255 - shade) / 255.0
            if occupancy > occupied_threshold:
                data.append(100)
            elif occupancy < free_threshold:
                data.append(0)
            else:
                data.append(-1)

    return map_snapshot(
        width=width,
        height=height,
        resolution=float(values["resolution"]),
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
        origin_yaw=float(origin[2]),
        data=data,
        frame_id=frame_id,
    )


def project_laser_scan(
    ranges: Sequence[float],
    *,
    angle_min: float,
    angle_increment: float,
    sensor_x: float,
    sensor_y: float,
    sensor_yaw: float,
    range_min: float,
    range_max: float,
    max_points: int = 720,
) -> tuple[tuple[float, float], ...]:
    """Project a planar scan into map-frame points."""
    if max_points <= 0:
        raise ValueError("max_points must be positive")
    step = max(1, math.ceil(len(ranges) / max_points))
    cosine = math.cos(sensor_yaw)
    sine = math.sin(sensor_yaw)
    points: list[tuple[float, float]] = []
    for index in range(0, len(ranges), step):
        distance = float(ranges[index])
        if not math.isfinite(distance) or not range_min <= distance <= range_max:
            continue
        angle = angle_min + index * angle_increment
        local_x = distance * math.cos(angle)
        local_y = distance * math.sin(angle)
        points.append(
            (
                sensor_x + cosine * local_x - sine * local_y,
                sensor_y + sine * local_x + cosine * local_y,
            )
        )
    return tuple(points)


def scan_alignment_score(
    snapshot: MapSnapshot,
    points: Sequence[tuple[float, float]],
    *,
    search_radius_m: float = 0.15,
    occupied_threshold: int = 65,
) -> tuple[int, int]:
    """Count scan endpoints that fall near occupied map cells."""
    if search_radius_m < 0.0:
        raise ValueError("search_radius_m must not be negative")
    geometry = snapshot.geometry
    radius_cells = math.ceil(search_radius_m / geometry.resolution)
    matched = 0
    evaluated = 0
    for world_x, world_y in points:
        grid_x, grid_y = geometry.world_to_grid(world_x, world_y)
        column = math.floor(grid_x)
        row = math.floor(grid_y)
        if not (0 <= column < geometry.width and 0 <= row < geometry.height):
            continue
        evaluated += 1
        found_occupied = False
        for neighbor_y in range(
            max(0, row - radius_cells),
            min(geometry.height, row + radius_cells + 1),
        ):
            offset = neighbor_y * geometry.width
            for neighbor_x in range(
                max(0, column - radius_cells),
                min(geometry.width, column + radius_cells + 1),
            ):
                if snapshot.data[offset + neighbor_x] >= occupied_threshold:
                    found_occupied = True
                    break
            if found_occupied:
                break
        matched += int(found_occupied)
    return matched, evaluated


def _load_pgm(path: Path) -> tuple[int, int, bytes]:
    payload = path.read_bytes()
    position = 0

    def token() -> bytes:
        nonlocal position
        while position < len(payload):
            if payload[position : position + 1] == b"#":
                newline = payload.find(b"\n", position)
                position = len(payload) if newline < 0 else newline + 1
            elif payload[position : position + 1].isspace():
                position += 1
            else:
                break
        start = position
        while position < len(payload):
            current = payload[position : position + 1]
            if current.isspace() or current == b"#":
                break
            position += 1
        if start == position:
            raise ValueError(f"invalid PGM header: {path}")
        return payload[start:position]

    magic = token()
    width = int(token())
    height = int(token())
    max_value = int(token())
    if width <= 0 or height <= 0 or not 0 < max_value <= 255:
        raise ValueError(f"unsupported PGM geometry: {path}")

    if magic == b"P5":
        if position >= len(payload) or not payload[position : position + 1].isspace():
            raise ValueError(f"invalid binary PGM separator: {path}")
        if payload[position : position + 2] == b"\r\n":
            position += 2
        else:
            position += 1
        pixels = payload[position : position + width * height]
    elif magic == b"P2":
        pixels = bytes(round(int(token()) * 255 / max_value) for _ in range(width * height))
        max_value = 255
    else:
        raise ValueError(f"unsupported PGM type {magic!r}: {path}")

    if len(pixels) != width * height:
        raise ValueError(f"PGM has {len(pixels)} pixels, expected {width * height}")
    if max_value != 255:
        pixels = bytes(round(value * 255 / max_value) for value in pixels)
    return width, height, pixels
