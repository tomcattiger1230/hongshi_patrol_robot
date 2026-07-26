"""Launch the primitive patrol robot in Gazebo Harmonic."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("patrol_robot_description"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))
    world = str(package_share / "worlds" / "patrol_test_world.sdf")
    model = str(package_share / "urdf" / "patrol_robot.urdf.xacro")

    gui = LaunchConfiguration("gui")
    demo = LaunchConfiguration("demo")
    rviz = LaunchConfiguration("rviz")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    robot_description = ParameterValue(
        Command(["xacro", " ", model]),
        value_type=str,
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ros_gz_share / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-r -s -v 3 {world}",
            "on_exit_shutdown": "true",
        }.items(),
        condition=UnlessCondition(gui),
    )
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ros_gz_share / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-r -v 3 {world}",
            "on_exit_shutdown": "true",
        }.items(),
        condition=IfCondition(gui),
    )

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
    )
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            "patrol_robot",
            "-topic",
            "robot_description",
            "-x",
            spawn_x,
            "-y",
            spawn_y,
            "-z",
            "0.02",
            "-Y",
            spawn_yaw,
        ],
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="patrol_sim_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/livox/lidar_scan/points@sensor_msgs/msg/PointCloud2"
            "[gz.msgs.PointCloudPacked",
        ],
        remappings=[("/livox/lidar_scan/points", "/livox/lidar")],
    )
    demo_controller = Node(
        package="patrol_robot_description",
        executable="patrol_demo_controller",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(demo),
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="patrol_sim_rviz",
        output="screen",
        arguments=[
            "-d",
            str(package_share / "rviz" / "patrol_sim.rviz"),
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui",
                default_value="false",
                description="Start the Gazebo graphical client.",
            ),
            DeclareLaunchArgument(
                "demo",
                default_value="false",
                description="Run the repeating autonomous motion demo.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start RViz with the simulation-time configuration.",
            ),
            DeclareLaunchArgument(
                "spawn_x",
                default_value="-4.8",
                description="Robot initial X position in the world.",
            ),
            DeclareLaunchArgument(
                "spawn_y",
                default_value="-4.5",
                description="Robot initial Y position in the world.",
            ),
            DeclareLaunchArgument(
                "spawn_yaw",
                default_value="0.0",
                description="Robot initial yaw angle in radians.",
            ),
            gazebo_server,
            gazebo_gui,
            state_publisher,
            spawn_robot,
            bridge,
            demo_controller,
            rviz_node,
        ]
    )
