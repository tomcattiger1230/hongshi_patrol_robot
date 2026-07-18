"""Launch the Robot320 onboard ROS 2 CAN bridge.

Usage from inside the built workspace:

    ros2 launch mobile_platform robot320_ros2.launch.py

The launch process resolves the absolute path of the installed
``robot320_ros2_bridge`` console script via ``ament_index_python``; if it is
not yet installed (source checkout without colcon build), it falls back to
``python3 -m mobile_platform.ros2_node`` running from the source root.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def resolve_bridge_cmd() -> str:
    """Return the command list prefix that runs the bridge entry point."""
    try:
        share_dir = Path(get_package_share_directory("mobile_platform"))
        candidate = share_dir.parent.parent / "lib" / "mobile_platform" / "robot320_ros2_bridge"
        if candidate.exists():
            return str(candidate)
    except Exception:
        pass
    # Fallback for in-tree runs: ``python3 -m mobile_platform.ros2_node``
    # requires PYTHONPATH=src/hongshi_agent (or installed package).
    return "python3 -m mobile_platform.ros2_node"


def generate_launch_description() -> LaunchDescription:
    bridge_cmd = resolve_bridge_cmd()

    args = [
        DeclareLaunchArgument("lib", default_value=""),
        DeclareLaunchArgument("device_index", default_value="0"),
        DeclareLaunchArgument("can_index", default_value="0"),
        DeclareLaunchArgument("topic_prefix", default_value="/robot320"),
        DeclareLaunchArgument("localization_pose_topic", default_value="/tracked_pose"),
        DeclareLaunchArgument("localization_stale_timeout", default_value="1.0"),
        DeclareLaunchArgument("telemetry_period", default_value="0.2"),
        DeclareLaunchArgument("command_timeout", default_value="0.6"),
        DeclareLaunchArgument("max_linear_speed", default_value="0.8"),
        DeclareLaunchArgument("max_angular_speed", default_value="1.2"),
        DeclareLaunchArgument("rpm_per_mps", default_value="500"),
        DeclareLaunchArgument("wheelbase", default_value="0.700"),
        DeclareLaunchArgument("min_turning_radius", default_value="2.350"),
        DeclareLaunchArgument("max_wheel_angle", default_value="16.59"),
        DeclareLaunchArgument("max_steering_command", default_value="350"),
        DeclareLaunchArgument("min_steering_speed", default_value="0.05"),
    ]

    if bridge_cmd.startswith("python3"):
        bridge = ExecuteProcess(
            cmd=[
                "python3",
                "-m",
                "mobile_platform.ros2_node",
                "--lib", LaunchConfiguration("lib"),
                "--device-index", LaunchConfiguration("device_index"),
                "--can-index", LaunchConfiguration("can_index"),
                "--topic-prefix", LaunchConfiguration("topic_prefix"),
                "--localization-pose-topic", LaunchConfiguration("localization_pose_topic"),
                "--localization-stale-timeout", LaunchConfiguration("localization_stale_timeout"),
                "--telemetry-period", LaunchConfiguration("telemetry_period"),
                "--command-timeout", LaunchConfiguration("command_timeout"),
                "--max-linear-speed", LaunchConfiguration("max_linear_speed"),
                "--max-angular-speed", LaunchConfiguration("max_angular_speed"),
                "--rpm-per-mps", LaunchConfiguration("rpm_per_mps"),
                "--wheelbase", LaunchConfiguration("wheelbase"),
                "--min-turning-radius", LaunchConfiguration("min_turning_radius"),
                "--max-wheel-angle", LaunchConfiguration("max_wheel_angle"),
                "--max-steering-command", LaunchConfiguration("max_steering_command"),
                "--min-steering-speed", LaunchConfiguration("min_steering_speed"),
            ],
            output="screen",
        )
    else:
        bridge = ExecuteProcess(
            cmd=[
                bridge_cmd,
                "--lib", LaunchConfiguration("lib"),
                "--device-index", LaunchConfiguration("device_index"),
                "--can-index", LaunchConfiguration("can_index"),
                "--topic-prefix", LaunchConfiguration("topic_prefix"),
                "--localization-pose-topic", LaunchConfiguration("localization_pose_topic"),
                "--localization-stale-timeout", LaunchConfiguration("localization_stale_timeout"),
                "--telemetry-period", LaunchConfiguration("telemetry_period"),
                "--command-timeout", LaunchConfiguration("command_timeout"),
                "--max-linear-speed", LaunchConfiguration("max_linear_speed"),
                "--max-angular-speed", LaunchConfiguration("max_angular_speed"),
                "--rpm-per-mps", LaunchConfiguration("rpm_per_mps"),
                "--wheelbase", LaunchConfiguration("wheelbase"),
                "--min-turning-radius", LaunchConfiguration("min_turning_radius"),
                "--max-wheel-angle", LaunchConfiguration("max_wheel_angle"),
                "--max-steering-command", LaunchConfiguration("max_steering_command"),
                "--min-steering-speed", LaunchConfiguration("min_steering_speed"),
            ],
            output="screen",
        )

    return LaunchDescription([*args, bridge])
