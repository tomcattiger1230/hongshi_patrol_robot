import time

from mobile_platform.fastdds_ros_gateway import Robot320FastDDSRosGateway
from robot320_interfaces.messages import NavigationStatus, RobotCommand


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


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
    gateway._on_nav_cmd_vel(marker)
    assert gateway.cmd_vel_pub.messages == [marker]

    assert gateway._request_nav_cancel() is True
    gateway._on_nav_cmd_vel(marker)
    assert gateway.cmd_vel_pub.messages == [marker]
    assert gateway._navigation.state == "canceling"
