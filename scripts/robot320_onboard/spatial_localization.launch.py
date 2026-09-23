"""Static-map AMCL localization using query-only wheel speed and MID-360."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    directory = Path(__file__).parent
    map_file = os.environ.get(
        "ROBOT320_LOCALIZATION_MAP", str(directory / "maps" / "active.yaml")
    )
    description = (
        Path(get_package_share_directory("robot320_description"))
        / "launch/robot_state_publisher.launch.py"
    )
    amcl_params = str(directory / "config/localization_amcl.yaml")
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(description))),
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="robot320_pointcloud_to_laserscan",
            parameters=[{
                "target_frame": "livox_frame", "transform_tolerance": 0.1,
                "min_height": -2.0, "max_height": 2.0,
                "angle_min": -3.141592654, "angle_max": 3.141592654,
                "angle_increment": 0.008726646, "scan_time": 0.1,
                "range_min": 0.30, "range_max": 20.0, "use_inf": True,
            }],
            remappings=[("cloud_in", "/filtered_points"), ("scan", "/scan")],
        ),
        ExecuteProcess(cmd=["/usr/bin/python3", str(directory / "b9_speed_feedback.py")]),
        ExecuteProcess(cmd=[
            "/usr/bin/python3", str(directory / "can_to_odom.py"), "--ros-args",
            "-r", "/can/actual_speed:=/can/actual_speed_b9",
        ]),
        ExecuteProcess(cmd=["/usr/bin/python3", str(directory / "imu_covariance_filter.py")]),
        Node(
            package="robot_localization", executable="ekf_node", name="ekf_filter_node",
            parameters=[str(directory / "config/localization_ekf.yaml")],
        ),
        Node(
            package="nav2_map_server", executable="map_server", name="map_server",
            parameters=[amcl_params, {"yaml_filename": map_file}],
        ),
        Node(
            package="nav2_amcl", executable="amcl", name="amcl", parameters=[amcl_params],
        ),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="localization_lifecycle_manager", parameters=[amcl_params],
        ),
    ])
