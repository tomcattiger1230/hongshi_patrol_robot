import time

from mobile_platform.fastdds_node import Robot320FastDDSBridge
from mobile_platform.robot320 import PlatformState
from mobile_platform.safety import SafetyConfig, SafetyController
from robot320_interfaces.messages import RobotCommand


class FakeRobot:
    def __init__(self):
        self.state = PlatformState(connected=True)
        self.calls = []

    def release_brake(self):
        self.calls.append(("release_brake",))

    def center_steering(self):
        self.calls.append(("center_steering",))

    def turn(self, angle, direction):
        self.calls.append(("turn", angle, direction))

    def set_motor_speed(self, direction, rpm):
        self.calls.append(("set_motor_speed", direction, rpm))

    def stop_motor(self):
        self.calls.append(("stop_motor",))

    def brake(self):
        self.calls.append(("brake",))

    def snapshot(self):
        return self.state


class FakeRobotTransport:
    def __init__(self):
        self.replies = []

    def publish_reply(self, reply):
        self.replies.append(reply)


def _bridge():
    robot = FakeRobot()
    transport = FakeRobotTransport()
    bridge = Robot320FastDDSBridge(
        robot=robot,
        safety=SafetyController(SafetyConfig(command_timeout_s=0.6)),
        transport=transport,
    )
    return bridge, robot, transport


def test_bridge_applies_manual_command_and_replies():
    bridge, robot, transport = _bridge()
    bridge._handle_command(
        RobotCommand(
            kind="manual_motion",
            client_id="test",
            sequence=1,
            stamp=time.time(),
            linear_speed_mps=0.2,
        )
    )
    assert any(call[0] == "set_motor_speed" for call in robot.calls)
    assert transport.replies[-1].status == "accepted"


def test_bridge_rejects_duplicate_and_stale_commands():
    bridge, _, transport = _bridge()
    command = RobotCommand(kind="stop", client_id="test", sequence=2, stamp=time.time())
    bridge._handle_command(command)
    bridge._handle_command(command)
    bridge._handle_command(
        RobotCommand(kind="stop", client_id="other", sequence=1, stamp=time.time() - 5.0)
    )
    assert [reply.status for reply in transport.replies] == [
        "accepted",
        "rejected",
        "rejected",
    ]
