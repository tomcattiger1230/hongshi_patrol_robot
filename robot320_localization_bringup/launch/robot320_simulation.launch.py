"""Launch the obstacle scene with MID-360 projection, SLAM, and Nav2."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EqualsSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    localization_share = Path(
        get_package_share_directory("robot320_localization_bringup")
    )
    description_share = Path(
        get_package_share_directory("patrol_robot_description")
    )

    mode = LaunchConfiguration("mode")
    map_file = LaunchConfiguration("map")
    navigation = LaunchConfiguration("navigation")
    exploration = LaunchConfiguration("exploration")
    rviz = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    nav2_params = str(localization_share / "config" / "nav2_ackermann.yaml")

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(description_share / "launch" / "patrol_robot_sim.launch.py")
        ),
        launch_arguments={
            "gui": LaunchConfiguration("gui"),
            "demo": LaunchConfiguration("demo"),
        }.items(),
    )

    cloud_filter = Node(
        package="mid360_preprocess",
        executable="mid360_preprocess_node",
        name="mid360_navigation_filter",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_topic": "/livox/lidar",
                "output_topic": "/filtered_points",
                "min_z": -1.35,
                "max_z": -0.10,
                "voxel_size": 0.05,
            }
        ],
    )

    cloud_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[
            str(localization_share / "config" / "mid360_to_scan.yaml"),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("cloud_in", "/livox/lidar"),
            ("scan", "/scan"),
        ],
    )

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("slam_toolbox"),
                    "launch",
                    "online_async_launch.py",
                ]
            )
        ),
        launch_arguments={
            "slam_params_file": str(
                localization_share / "config" / "slam_toolbox_mapping.yaml"
            ),
            "use_sim_time": use_sim_time,
            "autostart": "true",
        }.items(),
        condition=IfCondition(EqualsSubstitution(mode, "mapping")),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "localization_launch.py",
                ]
            )
        ),
        launch_arguments={
            "map": map_file,
            "params_file": nav2_params,
            "use_sim_time": use_sim_time,
            "autostart": "true",
        }.items(),
        condition=IfCondition(EqualsSubstitution(mode, "localization")),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(localization_share / "launch" / "robot320_nav2.launch.py")
        ),
        launch_arguments={
            "params_file": nav2_params,
            "use_sim_time": use_sim_time,
            "autostart": "true",
        }.items(),
        condition=IfCondition(navigation),
    )
    frontier_explorer = Node(
        package="robot320_localization_bringup",
        executable="frontier_explorer",
        name="frontier_explorer",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "min_frontier_size": 8,
                "clearance_radius": 0.95,
                "goal_timeout": 90.0,
            }
        ],
        condition=IfCondition(exploration),
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="robot320_navigation_rviz",
        output="screen",
        arguments=[
            "-d",
            str(localization_share / "rviz" / "robot320_navigation.rviz"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        [
            # Lyrical's Fast DDS launch adapter can leak its participant sockets
            # into lifecycle-managed child processes. Cyclone DDS avoids the
            # resulting startup stall and remains DDS-compatible with peers.
            SetEnvironmentVariable(
                "RMW_IMPLEMENTATION",
                "rmw_cyclonedds_cpp",
            ),
            DeclareLaunchArgument(
                "mode",
                default_value="mapping",
                choices=["mapping", "localization"],
                description="Use SLAM Toolbox mapping or map-server plus AMCL.",
            ),
            DeclareLaunchArgument(
                "map",
                default_value="",
                description="Map YAML required when mode is localization.",
            ),
            DeclareLaunchArgument(
                "navigation",
                default_value="false",
                description="Start Smac Hybrid and MPPI Ackermann Nav2.",
            ),
            DeclareLaunchArgument(
                "exploration",
                default_value="false",
                description="Send autonomous frontier goals while mapping.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start the map/navigation RViz configuration.",
            ),
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("demo", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            simulator,
            cloud_filter,
            cloud_to_scan,
            mapping,
            localization,
            nav2,
            frontier_explorer,
            rviz_node,
        ]
    )
