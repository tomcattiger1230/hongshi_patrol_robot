"""Launch the Robot320 Ackermann Nav2 stack without a localization node."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("robot320_localization_bringup"))
    nav2_share = Path(get_package_share_directory("nav2_bringup"))
    default_params = package_share / "config" / "nav2_ackermann.yaml"

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav2_share / "launch" / "navigation_launch.py")),
        launch_arguments={
            "params_file": params_file,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=str(default_params)),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            navigation,
        ]
    )
