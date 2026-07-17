import time

import remote_control.fastdds_client as client_module
from remote_control.fastdds_client import RobotRemoteFastDDSClient, _create_transport


class FakeRemoteTransport:
    def __init__(self):
        self.commands = []
        self.heartbeats = []
        self.closed = False

    def publish_command(self, command):
        self.commands.append(command)

    def publish_heartbeat(self, sequence):
        self.heartbeats.append(sequence)

    def receive_state(self, timeout_s):
        return None

    def receive_reply(self, timeout_s):
        return None

    def close(self):
        self.closed = True


def test_remote_client_builds_high_level_commands():
    transport = FakeRemoteTransport()
    client = RobotRemoteFastDDSClient(
        client_id="test-console",
        heartbeat_period_s=0.01,
        transport=transport,
    )
    try:
        client.send_manual_command(0.3, -0.2)
        client.send_navigation_goal(1.0, 2.0, 0.5)
        client.cancel_navigation()
        client.control_lift("move_to", 1.4)
        time.sleep(0.02)
    finally:
        client.close()

    assert [item.kind for item in transport.commands] == [
        "manual_motion",
        "navigation_goal",
        "cancel_navigation",
        "lift",
    ]
    assert transport.commands[0].linear_speed_mps == 0.3
    assert transport.commands[1].goal.x_m == 1.0
    assert transport.commands[3].lift_target_height_m == 1.4
    assert all(item.sequence > 0 for item in transport.commands)
    assert transport.heartbeats
    assert transport.closed is True


def test_auto_backend_prefers_ros2_when_available(monkeypatch):
    marker = object()
    monkeypatch.setattr(client_module, "ros2_available", lambda: True)
    monkeypatch.setattr(client_module, "Ros2RemoteTransport", lambda *_: marker)

    assert _create_transport("auto", 20, "test") is marker
