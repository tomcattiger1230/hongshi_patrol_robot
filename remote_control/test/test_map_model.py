import math

import pytest

from remote_control.map_model import (
    MapGeometry,
    goal_yaw,
    map_snapshot,
    pose_uncertainty,
)


def test_map_geometry_round_trip_without_rotation():
    geometry = MapGeometry(
        width=200,
        height=100,
        resolution=0.05,
        origin_x=-2.0,
        origin_y=-1.0,
    )

    scene = geometry.world_to_scene(1.25, 0.5)
    world = geometry.scene_to_world(*scene)

    assert world == pytest.approx((1.25, 0.5))
    assert geometry.contains_world(1.25, 0.5)
    assert not geometry.contains_world(20.0, 20.0)


def test_map_geometry_round_trip_with_rotated_origin():
    geometry = MapGeometry(
        width=20,
        height=20,
        resolution=0.5,
        origin_x=3.0,
        origin_y=-4.0,
        origin_yaw=math.pi / 2.0,
    )

    world = geometry.grid_to_world(2.0, 3.0)
    grid = geometry.world_to_grid(*world)

    assert world == pytest.approx((1.5, -3.0))
    assert grid == pytest.approx((2.0, 3.0))


def test_goal_yaw_uses_drag_direction_and_click_fallback():
    assert goal_yaw(1.0, 2.0, 1.0, 3.0) == pytest.approx(math.pi / 2.0)
    assert goal_yaw(1.0, 2.0, 1.01, 2.01, fallback=-0.4) == pytest.approx(-0.4)


def test_map_snapshot_rejects_wrong_cell_count():
    with pytest.raises(ValueError, match="expected 4"):
        map_snapshot(
            width=2,
            height=2,
            resolution=0.1,
            origin_x=0.0,
            origin_y=0.0,
            origin_yaw=0.0,
            data=[0, 100],
        )


def test_map_snapshot_reports_occupied_unknown_and_outside_cells():
    snapshot = map_snapshot(
        width=2,
        height=2,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        data=[0, 100, -1, 25],
    )

    assert snapshot.is_traversable(0.5, 0.5)
    assert not snapshot.is_traversable(1.5, 0.5)
    assert not snapshot.is_traversable(0.5, 1.5)
    assert snapshot.is_traversable(1.5, 1.5)
    assert snapshot.occupancy_at_world(3.0, 3.0) is None


def test_pose_uncertainty_uses_largest_planar_variance():
    covariance = [0.0] * 36
    covariance[0] = 0.04
    covariance[7] = 0.09
    covariance[35] = math.radians(10.0) ** 2

    position_sigma, yaw_sigma = pose_uncertainty(covariance)

    assert position_sigma == pytest.approx(0.3)
    assert math.degrees(yaw_sigma) == pytest.approx(10.0)
