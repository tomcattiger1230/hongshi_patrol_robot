from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
            parameters=[
                {"lidar_ip": "192.168.1.111"},
                {"host_ip": "192.168.1.50"},
                {"pcl_data_type": 1},
                {"cmd_data_type": 2},
                {"multi_topic": 1},
                {"enable_high_data_rate": 0},
                {"enable_high_performance": 0},
                {"config_path": os.path.expanduser("~/roboracer_ws/src/livox_ros_driver2/config/MID360s_config.json")}
            ]
        )
    ])
