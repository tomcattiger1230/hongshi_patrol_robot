"""Bring up Robot320 chassis control and MID-360s Cartographer localization."""

import json
import os
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _value(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context)


def _enabled(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _write_livox_config(host_ip: str, lidar_ip: str) -> str:
    """Create the SDK JSON config from launch arguments.

    Livox SDK reads addresses from JSON rather than ROS parameters, so the
    generated file keeps deployment-specific IPs out of the source tree.
    """
    config = {
        "lidar_summary_info": {"lidar_type": 8},
        "Mid360s": {
            "lidar_net_info": {
                "cmd_data_port": 56101,
                "push_msg_port": 56201,
                "point_data_port": 56301,
                "imu_data_port": 56401,
                "log_data_port": 56501,
            },
            "host_net_info": [
                {
                    "host_ip": host_ip,
                    "cmd_data_port": 56101,
                    "push_msg_port": 56201,
                    "point_data_port": 56301,
                    "imu_data_port": 56401,
                    "log_data_port": 56501,
                }
            ],
        },
        "lidar_configs": [
            {
                "ip": lidar_ip,
                "pcl_data_type": 2,
                "pattern_mode": 0,
                "extrinsic_parameter": {
                    "roll": 0.0,
                    "pitch": 0.0,
                    "yaw": 0.0,
                    "x": 0,
                    "y": 0,
                    "z": 0,
                },
            }
        ],
    }
    path = Path(tempfile.gettempdir()) / f"robot320_mid360s_{os.getpid()}.json"
    path.write_text(json.dumps(config, indent=2), encoding="utf-8")
    return str(path)


def _launch_setup(context):
    mode = _value(context, "mode").strip().lower()
    if mode not in {"mapping", "localization"}:
        raise RuntimeError("mode must be 'mapping' or 'localization'")

    package_share = Path(get_package_share_directory("robot320_localization_bringup"))
    mobile_platform_share = Path(get_package_share_directory("mobile_platform"))
    config_dir = package_share / "config"
    lidar_config = _write_livox_config(
        _value(context, "host_ip"),
        _value(context, "lidar_ip"),
    )

    map_state_file = os.path.abspath(os.path.expanduser(_value(context, "map_state_file")))
    if mode == "localization" and not Path(map_state_file).is_file():
        raise RuntimeError(
            "localization mode requires an existing Cartographer .pbstream: "
            f"{map_state_file}"
        )

    actions = []
    if _enabled(_value(context, "enable_chassis")):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(mobile_platform_share / "launch" / "robot320_ros2.launch.py")
                ),
                launch_arguments={
                    "lib": _value(context, "lib"),
                    "device_index": _value(context, "device_index"),
                    "can_index": _value(context, "can_index"),
                    "topic_prefix": _value(context, "topic_prefix"),
                    "localization_pose_topic": _value(context, "tracked_pose_topic"),
                    "command_timeout": _value(context, "command_timeout"),
                }.items(),
            )
        )

    if _enabled(_value(context, "enable_fastdds_gateway")):
        actions.append(
            Node(
                package="mobile_platform",
                executable="robot320_ros_gateway",
                name="robot320_communication_gateway",
                output="screen",
                additional_env={
                    "ROS_DOMAIN_ID": _value(context, "fastdds_domain_id"),
                },
                arguments=[
                    "--domain-id",
                    _value(context, "fastdds_domain_id"),
                    "--robot-id",
                    _value(context, "robot_id"),
                    "--topic-prefix",
                    _value(context, "topic_prefix"),
                    "--nav-action",
                    _value(context, "nav_action"),
                    "--nav-cmd-vel-topic",
                    _value(context, "nav_cmd_vel_topic"),
                ],
            )
        )

    actions.extend(
        [
            Node(
                package="livox_ros_driver2",
                executable="livox_ros_driver2_node",
                name="livox_lidar_publisher",
                output="screen",
                parameters=[
                    {
                        "xfer_format": 0,
                        "multi_topic": 0,
                        "data_src": 0,
                        "publish_freq": float(_value(context, "publish_frequency")),
                        "output_data_type": 0,
                        "frame_id": _value(context, "lidar_frame"),
                        "user_config_path": lidar_config,
                        "cmdline_input_bd_code": "livox0000000001",
                    }
                ],
            ),
            Node(
                package="mid360_preprocess",
                executable="mid360_preprocess_node",
                name="mid360_preprocess",
                output="screen",
                parameters=[
                    {
                        "input_topic": _value(context, "pointcloud_topic"),
                        "output_topic": _value(context, "filtered_points_topic"),
                        "min_z": float(_value(context, "min_z")),
                        "max_z": float(_value(context, "max_z")),
                        "voxel_size": float(_value(context, "voxel_size")),
                    }
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_livox_tf",
                output="screen",
                arguments=[
                    _value(context, "lidar_x"),
                    _value(context, "lidar_y"),
                    _value(context, "lidar_z"),
                    _value(context, "lidar_yaw"),
                    _value(context, "lidar_pitch"),
                    _value(context, "lidar_roll"),
                    "base_link",
                    _value(context, "lidar_frame"),
                ],
            ),
        ]
    )

    cartographer_arguments = [
        "-configuration_directory",
        str(config_dir),
        "-configuration_basename",
        "mid360_localization.lua" if mode == "localization" else "mid360_2d.lua",
    ]
    if mode == "localization":
        cartographer_arguments.extend(["-load_state_filename", map_state_file])

    actions.extend(
        [
            Node(
                package="cartographer_ros",
                executable="cartographer_node",
                name="cartographer_node",
                output="screen",
                arguments=cartographer_arguments,
                remappings=[
                    ("points2", _value(context, "filtered_points_topic")),
                    ("tracked_pose", _value(context, "tracked_pose_topic")),
                ],
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_occupancy_grid_node",
                name="cartographer_occupancy_grid_node",
                output="screen",
                arguments=["-resolution", _value(context, "map_resolution")],
            ),
        ]
    )
    return actions


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument("mode", default_value="localization"),
        DeclareLaunchArgument("map_state_file", default_value=""),
        DeclareLaunchArgument("enable_chassis", default_value="true"),
        DeclareLaunchArgument("enable_fastdds_gateway", default_value="true"),
        DeclareLaunchArgument("fastdds_domain_id", default_value="20"),
        DeclareLaunchArgument("robot_id", default_value="robot320"),
        DeclareLaunchArgument("nav_action", default_value="/navigate_to_pose"),
        DeclareLaunchArgument("nav_cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("lib", default_value=""),
        DeclareLaunchArgument("device_index", default_value="0"),
        DeclareLaunchArgument("can_index", default_value="0"),
        DeclareLaunchArgument("topic_prefix", default_value="/robot320"),
        DeclareLaunchArgument("command_timeout", default_value="0.6"),
        DeclareLaunchArgument("host_ip", default_value="192.168.1.50"),
        DeclareLaunchArgument("lidar_ip", default_value="192.168.1.107"),
        DeclareLaunchArgument("lidar_frame", default_value="livox_frame"),
        DeclareLaunchArgument("lidar_x", default_value="0.0"),
        DeclareLaunchArgument("lidar_y", default_value="0.0"),
        DeclareLaunchArgument("lidar_z", default_value="0.0"),
        DeclareLaunchArgument("lidar_roll", default_value="0.0"),
        DeclareLaunchArgument("lidar_pitch", default_value="0.0"),
        DeclareLaunchArgument("lidar_yaw", default_value="0.0"),
        DeclareLaunchArgument("pointcloud_topic", default_value="/livox/lidar"),
        DeclareLaunchArgument("filtered_points_topic", default_value="/filtered_points"),
        DeclareLaunchArgument("tracked_pose_topic", default_value="/tracked_pose"),
        DeclareLaunchArgument("publish_frequency", default_value="10.0"),
        DeclareLaunchArgument("min_z", default_value="-0.2"),
        DeclareLaunchArgument("max_z", default_value="2.5"),
        DeclareLaunchArgument("voxel_size", default_value="0.05"),
        DeclareLaunchArgument("map_resolution", default_value="0.05"),
    ]
    return LaunchDescription([*arguments, OpaqueFunction(function=_launch_setup)])
