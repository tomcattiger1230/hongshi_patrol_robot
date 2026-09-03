import time

from robot320_interfaces.messages import Pose2D, RobotCommand
from remote_control.demo_transport import DemoRemoteTransport


def command(kind, sequence, **kwargs):
    return RobotCommand(kind=kind, client_id="test-gui", sequence=sequence, **kwargs)


def test_demo_transport_simulates_manual_motion_without_network():
    transport = DemoRemoteTransport()
    transport.publish_command(command("manual_motion", 1, linear_speed_mps=0.5))
    time.sleep(0.02)

    state = transport.receive_state()
    reply = transport.receive_reply()

    assert state is not None
    assert state.online is True
    assert state.robot_id == "robot320-demo"
    assert state.chassis.connected is True
    assert state.chassis.speed_kmh == 1.8
    assert state.pose is not None and state.pose.x_m > 0.0
    assert reply is not None and reply.status == "accepted"


def test_demo_transport_runs_and_cancels_navigation():
    transport = DemoRemoteTransport()
    transport.publish_command(
        command("navigation_goal", 1, goal=Pose2D(x_m=1.0, y_m=0.5, yaw_rad=0.2))
    )
    state = transport.receive_state()
    assert state is not None
    assert state.navigation.state == "executing"
    assert state.navigation.target is not None

    transport.publish_command(command("cancel_navigation", 2))
    state = transport.receive_state()
    replies = [transport.receive_reply(), transport.receive_reply()]

    assert state is not None and state.navigation.state == "canceled"
    assert [reply.status for reply in replies if reply is not None] == [
        "accepted",
        "accepted",
    ]


def test_demo_transport_keeps_emergency_stop_until_reset():
    transport = DemoRemoteTransport()
    transport.publish_command(command("emergency_stop", 1))
    transport.publish_command(command("manual_motion", 2, linear_speed_mps=0.3))

    state = transport.receive_state()
    replies = [transport.receive_reply(), transport.receive_reply()]
    assert state is not None and state.chassis.emergency_stopped is True
    assert replies[1] is not None and replies[1].status == "rejected"

    transport.publish_command(command("reset_emergency_stop", 3))
    state = transport.receive_state()
    assert state is not None and state.chassis.emergency_stopped is False

