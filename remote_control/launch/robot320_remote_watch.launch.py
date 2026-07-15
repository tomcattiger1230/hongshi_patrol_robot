"""Launch Robot320 remote ROS 2 telemetry watcher."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def resolve_watcher_command() -> str:
    """Return the installed remote ROS 2 console script when available."""
    try:
        share_dir = Path(get_package_share_directory("remote_control"))
        candidate = share_dir.parent.parent / "lib" / "remote_control" / "robot320_remote_ros2"
        if candidate.exists():
            return str(candidate)
    except Exception:
        pass
    return "python3 -m remote_control.ros2_client"


def generate_launch_description() -> LaunchDescription:
    watcher_command = resolve_watcher_command()
    if watcher_command.startswith("python3"):
        command = [
            "python3",
            "-m",
            "remote_control.ros2_client",
        ]
    else:
        command = [watcher_command]

    command.extend(
        [
            "--topic-prefix",
            LaunchConfiguration("topic_prefix"),
            "watch",
            "--seconds",
            LaunchConfiguration("seconds"),
        ]
    )

    args = [
        DeclareLaunchArgument("topic_prefix", default_value="/robot320"),
        DeclareLaunchArgument("seconds", default_value="3600"),
    ]
    watcher = ExecuteProcess(cmd=command, output="screen")
    return LaunchDescription([*args, watcher])
