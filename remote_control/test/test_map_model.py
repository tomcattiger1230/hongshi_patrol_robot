import math

import pytest

from remote_control.map_model import (
    MapGeometry,
    goal_yaw,
    load_map_yaml,
    map_snapshot,
    polyline_length,
    pose_uncertainty,
    project_laser_scan,
    scan_alignment_score,
    save_map_yaml,
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
    assert snapshot.is_traversable(0.5, 1.5, allow_unknown=True)
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


def test_load_map_yaml_reads_pgm_and_flips_image_rows(tmp_path):
    (tmp_path / "map.pgm").write_bytes(
        b"P5\n# top row then bottom row\n2 2\n255\n"
        + bytes([0, 205, 254, 100])
    )
    (tmp_path / "map.yaml").write_text(
        "\n".join(
            [
                "image: map.pgm",
                "resolution: 0.05",
                "origin: [-1.0, -2.0, 0.25]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
            ]
        ),
        encoding="utf-8",
    )

    snapshot = load_map_yaml(tmp_path / "map.yaml")

    assert snapshot.geometry.width == 2
    assert snapshot.geometry.height == 2
    assert snapshot.geometry.resolution == pytest.approx(0.05)
    assert snapshot.geometry.origin_x == pytest.approx(-1.0)
    assert snapshot.geometry.origin_yaw == pytest.approx(0.25)
    assert snapshot.data == (0, -1, 100, -1)


def test_save_map_yaml_round_trips_trinary_snapshot(tmp_path):
    original = map_snapshot(
        width=3,
        height=2,
        resolution=0.05,
        origin_x=-1.5,
        origin_y=2.25,
        origin_yaw=0.1,
        data=[0, 100, -1, -1, 0, 100],
    )

    yaml_path, pgm_path = save_map_yaml(original, tmp_path / "saved_map")
    restored = load_map_yaml(yaml_path)

    assert yaml_path.name == "saved_map.yaml"
    assert pgm_path.is_file()
    assert restored.geometry == original.geometry
    assert restored.data == original.data


def test_project_laser_scan_uses_sensor_map_transform_and_filters_ranges():
    points = project_laser_scan(
        [1.0, math.inf, 3.0],
        angle_min=0.0,
        angle_increment=math.pi / 2.0,
        sensor_x=2.0,
        sensor_y=3.0,
        sensor_yaw=math.pi / 2.0,
        range_min=0.2,
        range_max=2.0,
    )

    assert len(points) == 1
    assert points[0] == pytest.approx((2.0, 4.0))


def test_scan_alignment_score_matches_nearby_occupied_cells():
    snapshot = map_snapshot(
        width=4,
        height=3,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        data=[
            0,
            0,
            0,
            0,
            0,
            100,
            0,
            0,
            0,
            0,
            0,
            0,
        ],
    )

    matched, evaluated = scan_alignment_score(
        snapshot,
        [(1.5, 1.5), (3.5, 2.5), (10.0, 10.0)],
        search_radius_m=0.0,
    )

    assert matched == 1
    assert evaluated == 2


def test_polyline_length_accumulates_segments():
    assert polyline_length([(0.0, 0.0), (3.0, 4.0), (3.0, 6.0)]) == pytest.approx(
        7.0
    )
    assert polyline_length([]) == 0.0
