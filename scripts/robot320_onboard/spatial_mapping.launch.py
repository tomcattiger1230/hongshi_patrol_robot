"""Independent LiDAR-only Cartographer mapping; no CAN, EKF or Nav2."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    description = Path(get_package_share_directory('robot320_description')) / 'launch/robot_state_publisher.launch.py'
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(description))),
        Node(package='cartographer_ros', executable='cartographer_node',
             arguments=['-configuration_directory', str(Path(__file__).parent / 'mapping_config'),
                        '-configuration_basename', 'mid360_2d.lua'],
             remappings=[('points2', '/filtered_points')]),
        Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
             arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']),
    ])
