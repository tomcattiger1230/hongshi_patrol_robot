"""Pure occupancy-grid frontier extraction used by autonomous mapping."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class Frontier:
    """A reachable free-cell frontier and its information gain."""

    row: int
    column: int
    size: int


def _index(row: int, column: int, width: int) -> int:
    return row * width + column


def _neighbors(row: int, column: int, width: int, height: int):
    for row_offset in (-1, 0, 1):
        for column_offset in (-1, 0, 1):
            if row_offset == 0 and column_offset == 0:
                continue
            neighbor_row = row + row_offset
            neighbor_column = column + column_offset
            if (
                0 <= neighbor_row < height
                and 0 <= neighbor_column < width
            ):
                yield neighbor_row, neighbor_column


def _has_clearance(
    data: Sequence[int],
    row: int,
    column: int,
    width: int,
    height: int,
    clearance_cells: int,
) -> bool:
    radius_squared = clearance_cells * clearance_cells
    for row_offset in range(-clearance_cells, clearance_cells + 1):
        for column_offset in range(-clearance_cells, clearance_cells + 1):
            if row_offset * row_offset + column_offset * column_offset > radius_squared:
                continue
            check_row = row + row_offset
            check_column = column + column_offset
            if not (0 <= check_row < height and 0 <= check_column < width):
                return False
            if data[_index(check_row, check_column, width)] >= 50:
                return False
    return True


def find_frontiers(
    data: Sequence[int],
    width: int,
    height: int,
    *,
    min_size: int = 8,
    clearance_cells: int = 1,
) -> list[Frontier]:
    """Return clustered free cells adjacent to unknown map space."""
    if width <= 0 or height <= 0 or len(data) != width * height:
        raise ValueError("occupancy grid dimensions do not match its data")

    candidates: set[tuple[int, int]] = set()
    for row in range(height):
        for column in range(width):
            if data[_index(row, column, width)] != 0:
                continue
            if any(
                data[_index(neighbor_row, neighbor_column, width)] < 0
                for neighbor_row, neighbor_column in _neighbors(
                    row, column, width, height
                )
            ):
                candidates.add((row, column))

    frontiers: list[Frontier] = []
    while candidates:
        seed = candidates.pop()
        cluster = [seed]
        queue = deque([seed])
        while queue:
            row, column = queue.popleft()
            for neighbor in _neighbors(row, column, width, height):
                if neighbor in candidates:
                    candidates.remove(neighbor)
                    cluster.append(neighbor)
                    queue.append(neighbor)
        if len(cluster) < min_size:
            continue

        mean_row = sum(cell[0] for cell in cluster) / len(cluster)
        mean_column = sum(cell[1] for cell in cluster) / len(cluster)
        ordered = sorted(
            cluster,
            key=lambda cell: (
                (cell[0] - mean_row) ** 2 + (cell[1] - mean_column) ** 2
            ),
        )
        goal = next(
            (
                cell
                for cell in ordered
                if _has_clearance(
                    data,
                    cell[0],
                    cell[1],
                    width,
                    height,
                    clearance_cells,
                )
            ),
            None,
        )
        if goal is not None:
            frontiers.append(Frontier(goal[0], goal[1], len(cluster)))

    return frontiers


def choose_frontier(
    frontiers: Sequence[Frontier],
    robot_row: float,
    robot_column: float,
    *,
    information_gain_weight: float = 0.35,
) -> Frontier | None:
    """Prefer nearby frontiers while rewarding larger unexplored boundaries."""
    if not frontiers:
        return None
    return min(
        frontiers,
        key=lambda frontier: (
            math.hypot(
                frontier.row - robot_row,
                frontier.column - robot_column,
            )
            - information_gain_weight * math.sqrt(frontier.size)
        ),
    )
