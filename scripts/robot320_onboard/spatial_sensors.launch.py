"""Only LiDAR acquisition, preprocessing and read-only spatial telemetry."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    driver = Path(get_package_share_directory('livox_ros_driver2')) / 'launch_ROS2/msg_MID360s_launch.py'
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(driver))),
        Node(package='mid360_preprocess', executable='mid360_preprocess_node'),
        ExecuteProcess(cmd=['/usr/bin/python3', str(Path(__file__).with_name('spatial_bridge.py'))]),
    ])
