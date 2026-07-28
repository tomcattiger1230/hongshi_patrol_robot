"""Launch the obstacle scene with MID-360 projection, SLAM, and Nav2."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    EqualsSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def _mapping_actions(context, *, localization_share, use_sim_time):
    mode = LaunchConfiguration("mode").perform(context)
    if mode not in ("mapping", "continuing"):
        return []

    prefix = os.path.abspath(
        os.path.expanduser(LaunchConfiguration("persistent_map").perform(context))
    )
    has_posegraph = Path(f"{prefix}.posegraph").is_file() and Path(
        f"{prefix}.data"
    ).is_file()
    continuing = mode == "continuing"
    load_existing = continuing and has_posegraph
    slam_params = RewrittenYaml(
        source_file=str(
            localization_share / "config" / "slam_toolbox_mapping.yaml"
        ),
        root_key="",
        param_rewrites={
            "map_file_name": prefix if load_existing else "",
            "map_start_at_dock": "true" if load_existing else "false",
        },
        convert_types=True,
    )

    if load_existing:
        status = f"Continuing persistent SLAM map: {prefix}"
    elif continuing:
        status = (
            "No serialized pose graph found; starting a new persistent map at "
            f"{prefix}. It will be loaded automatically on the next run."
        )
    else:
        status = "Starting a new non-persistent mapping session"

    actions = [
        LogInfo(msg=status),
        IncludeLaunchDescription(
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
                "slam_params_file": slam_params,
                "use_sim_time": use_sim_time,
                "autostart": "true",
            }.items(),
        ),
    ]
    if continuing:
        actions.append(
            Node(
                package="robot320_localization_bringup",
                executable="persistent_map_manager",
                name="persistent_map_manager",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "map_prefix": prefix,
                        "save_interval": LaunchConfiguration(
                            "persistent_save_interval"
                        ),
                    }
                ],
            )
        )
    return actions


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
    gazebo = LaunchConfiguration("gazebo")
    nav2_params = RewrittenYaml(
        source_file=str(
            localization_share / "config" / "nav2_ackermann.yaml"
        ),
        root_key="",
        param_rewrites={
            "use_sim_time": use_sim_time,
            # Both simulators spawn at the map origin. Publishing this initial
            # estimate lets AMCL establish map->odom before Nav2 activates.
            "set_initial_pose": "true",
            # Isaac RTX lidar frames can be more than one simulated second
            # apart on Spark. Keep the real-robot safety timeout unchanged.
            "source_timeout": "30.0",
            "cmd_vel_out_topic": PythonExpression(
                [
                    "'/cmd_vel' if '",
                    gazebo,
                    "' == 'true' else '/cmd_vel_collision'",
                ]
            ),
        },
        convert_types=True,
    )

    simulator = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        description_share
                        / "launch"
                        / "patrol_robot_sim.launch.py"
                    )
                ),
                launch_arguments={
                    "gui": LaunchConfiguration("gui"),
                    "demo": LaunchConfiguration("demo"),
                    # This launch starts the navigation RViz below. Do not
                    # start patrol_sim.rviz as a second RViz process.
                    "rviz": "false",
                }.items(),
            )
        ],
        condition=IfCondition(gazebo),
    )
    external_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot320_external_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(
                        [
                            "xacro",
                            " ",
                            str(
                                description_share
                                / "urdf"
                                / "patrol_robot.urdf.xacro"
                            ),
                        ]
                    ),
                    value_type=str,
                ),
                "use_sim_time": use_sim_time,
            }
        ],
        condition=UnlessCondition(gazebo),
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
                # Gazebo can expose a scoped sensor path as the PointCloud2
                # frame. Normalize it to the URDF/TF lidar link for RViz.
                "output_frame": "lidar_link",
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
            ("scan", "/scan_raw"),
        ],
    )
    scan_restamper = Node(
        package="robot320_localization_bringup",
        executable="scan_restamper",
        name="scan_restamper",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_topic": "/scan_raw",
                "output_topic": "/scan",
                "frame_id": "lidar_link",
                "sensor_x": 0.40,
                "sensor_y": 0.0,
                # RTX rays include the imported wheels and body outside the
                # nominal navigation footprint. Use the full swept envelope.
                "self_filter_x_min": -1.50,
                "self_filter_x_max": 1.50,
                "self_filter_y_abs": 1.00,
            }
        ],
    )
    isaac_cmd_vel_relay = Node(
        package="robot320_localization_bringup",
        executable="cmd_vel_relay",
        name="isaac_cmd_vel_relay",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_topic": "/cmd_vel_smoothed",
                # Keep manual /cmd_vel and Nav2's 20 Hz output off Isaac's
                # final command topic. Otherwise their alternating non-zero
                # and zero messages make the vehicle shake violently.
                "priority_input_topic": "/cmd_vel",
                "priority_timeout": 0.5,
                "output_topic": "/cmd_vel_isaac",
            }
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    navigation,
                    "' == 'true' and '",
                    gazebo,
                    "' == 'false'",
                ]
            )
        ),
    )

    mapping = OpaqueFunction(
        function=_mapping_actions,
        kwargs={
            "localization_share": localization_share,
            "use_sim_time": use_sim_time,
        },
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
                "clearance_radius": 1.25,
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
            DeclareLaunchArgument(
                "mode",
                default_value="continuing",
                choices=["continuing", "mapping", "localization"],
                description=(
                    "Continue the persistent SLAM pose graph by default, "
                    "start a fresh map, or use static map-server plus AMCL."
                ),
            ),
            DeclareLaunchArgument(
                "map",
                default_value="",
                description="Map YAML required when mode is localization.",
            ),
            DeclareLaunchArgument(
                "persistent_map",
                default_value=PathJoinSubstitution(
                    [
                        EnvironmentVariable("HOME"),
                        "robot320_maps",
                        "patrol_current",
                    ]
                ),
                description=(
                    "File prefix for persistent .posegraph/.data/.yaml/.pgm."
                ),
            ),
            DeclareLaunchArgument(
                "persistent_save_interval",
                default_value="30.0",
                description="Seconds between persistent pose-graph saves.",
            ),
            DeclareLaunchArgument(
                "navigation",
                default_value="false",
                description="Start Smac Hybrid and RPP Ackermann Nav2.",
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
            DeclareLaunchArgument(
                "gazebo",
                default_value="true",
                description=(
                    "Start Gazebo; set false when Isaac Sim publishes "
                    "clock, odometry, TF, joint states, and lidar."
                ),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            simulator,
            external_state_publisher,
            cloud_filter,
            cloud_to_scan,
            scan_restamper,
            isaac_cmd_vel_relay,
            mapping,
            localization,
            # Give Gazebo, the robot TF tree, and SLAM/AMCL time to initialize
            # before Nav2's lifecycle manager starts configuring its costmaps.
            TimerAction(period=5.0, actions=[nav2]),
            frontier_explorer,
            rviz_node,
        ]
    )
