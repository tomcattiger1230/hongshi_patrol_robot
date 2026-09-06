import time

import mobile_platform.fastdds_ros_gateway as gateway_module
from mobile_platform.fastdds_ros_gateway import (
    Robot320FastDDSRosGateway,
    Ros2RobotTransport,
)
from robot320_interfaces.messages import (
    NavigationStatus,
    RobotCommand,
    RobotTelemetry,
    remote_map,
    to_json,
)


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _String:
    def __init__(self):
        self.data = ""


class _StampedTwist:
    def __init__(self, twist):
        self.twist = twist


class _Node:
    def __init__(self):
        self.publishers = {}
        self.subscriptions = {}

    def create_publisher(self, _message_type, topic, _qos):
        publisher = _Publisher()
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(self, _message_type, topic, callback, _qos):
        self.subscriptions[topic] = callback
        return object()


def _gateway_for_validation():
    gateway = Robot320FastDDSRosGateway.__new__(Robot320FastDDSRosGateway)
    gateway.max_command_age_s = 2.0
    gateway._last_sequences = {}
    return gateway


def test_command_validation_accepts_fresh_increasing_sequence():
    gateway = _gateway_for_validation()
    command = RobotCommand(
        kind="stop", client_id="remote-a", sequence=3, stamp=time.time()
    )

    assert gateway._validate_command(command) is None


def test_command_validation_rejects_stale_and_duplicate_commands():
    gateway = _gateway_for_validation()
    stale = RobotCommand(
        kind="stop", client_id="remote-a", sequence=3, stamp=time.time() - 5.0
    )
    duplicate = RobotCommand(
        kind="stop", client_id="remote-a", sequence=3, stamp=time.time()
    )
    gateway._last_sequences["remote-a"] = 3

    assert "stale" in gateway._validate_command(stale)
    assert "duplicate" in gateway._validate_command(duplicate)


def test_nav_velocity_relay_is_closed_immediately_on_cancel():
    gateway = Robot320FastDDSRosGateway.__new__(Robot320FastDDSRosGateway)
    gateway.cmd_vel_pub = _Publisher()
    gateway._nav_velocity_enabled = True
    gateway._active_goal_handle = None
    gateway._pending_nav_command_id = "pending-goal"
    gateway._navigation = NavigationStatus(state="sending", goal_id="pending-goal")

    marker = object()
    stamped = _StampedTwist(marker)
    gateway._on_nav_cmd_vel(stamped)
    assert gateway.cmd_vel_pub.messages == [marker]

    assert gateway._request_nav_cancel() is True
    gateway._on_nav_cmd_vel(stamped)
    assert gateway.cmd_vel_pub.messages == [marker]
    assert gateway._navigation.state == "canceling"


def test_ros2_transport_uses_string_json_topics(monkeypatch):
    monkeypatch.setattr(gateway_module, "String", _String)
    node = _Node()
    transport = Ros2RobotTransport(node, "/robot320", "robot-test")

    command = RobotCommand(kind="stop", client_id="remote", sequence=4)
    message = _String()
    message.data = to_json(command)
    node.subscriptions["/robot320/command"](message)

    received = transport.receive_command(0.0)
    assert received is not None and received.sequence == 4

    transport.publish_state(RobotTelemetry(), 1)
    state_message = node.publishers["/robot320/state"].messages[-1]
    assert '"robot_id":"robot-test"' in state_message.data

    snapshot = remote_map(
        width=2,
        height=1,
        resolution=0.05,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        data=(-1, 100),
    )
    transport.publish_map(snapshot)
    assert snapshot.revision in node.publishers["/robot320/map"].messages[-1].data


def test_occupancy_grid_is_compressed_for_remote_transport():
    class Value:
        pass

    message = Value()
    message.header = Value()
    message.header.frame_id = "map"
    message.header.stamp = Value()
    message.header.stamp.sec = 12
    message.header.stamp.nanosec = 500_000_000
    message.info = Value()
    message.info.width = 3
    message.info.height = 2
    message.info.resolution = 0.1
    message.info.origin = Value()
    message.info.origin.position = Value()
    message.info.origin.position.x = -1.0
    message.info.origin.position.y = 2.0
    message.info.origin.orientation = Value()
    message.info.origin.orientation.x = 0.0
    message.info.origin.orientation.y = 0.0
    message.info.origin.orientation.z = 0.0
    message.info.origin.orientation.w = 1.0
    message.data = [-1, 0, 10, 50, 99, 100]

    snapshot = gateway_module._remote_map_from_occupancy_grid(message)

    assert snapshot.frame_id == "map"
    assert snapshot.stamp == 12.5
    assert snapshot.occupancy_data() == tuple(message.data)


def test_simulation_odometry_provides_online_pose_and_speed():
    class Value:
        pass

    gateway = Robot320FastDDSRosGateway.__new__(Robot320FastDDSRosGateway)
    gateway.robot_id = "robot-test"
    gateway.odometry_pose_frame = "map"
    message = Value()
    message.header = Value()
    message.header.stamp = Value()
    message.header.stamp.sec = 3
    message.header.stamp.nanosec = 0
    message.pose = Value()
    message.pose.pose = Value()
    message.pose.pose.position = Value()
    message.pose.pose.position.x = 1.25
    message.pose.pose.position.y = -0.5
    message.pose.pose.orientation = Value()
    message.pose.pose.orientation.x = 0.0
    message.pose.pose.orientation.y = 0.0
    message.pose.pose.orientation.z = 0.0
    message.pose.pose.orientation.w = 1.0
    message.twist = Value()
    message.twist.twist = Value()
    message.twist.twist.linear = Value()
    message.twist.twist.linear.x = 0.5
    message.twist.twist.linear.y = 0.0

    gateway._on_odometry(message)

    telemetry = gateway._simulation_telemetry
    assert telemetry is not None and telemetry.pose is not None
    assert telemetry.pose.x_m == 1.25
    assert telemetry.pose.frame_id == "map"
    assert telemetry.chassis.speed_kmh == 1.8
    assert gateway._last_odometry_received > 0.0
