from lift_serial_bridge import COMMANDS, LiftSerialController
from lift_control import send_action

import pytest


class FakeSerial:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.is_open = True
        self.writes = []
        self.flush_count = 0

    def write(self, payload):
        self.writes.append(payload)
        return len(payload)

    def flush(self):
        self.flush_count += 1

    def close(self):
        self.is_open = False


def test_each_click_writes_exactly_one_frame_and_reuses_connection():
    instances = []

    def factory(**kwargs):
        instance = FakeSerial(**kwargs)
        instances.append(instance)
        return instance

    controller = LiftSerialController("/dev/test", serial_factory=factory)
    controller.send("raise")
    controller.send("raise")
    controller.send("lower")
    controller.send("stop")

    assert len(instances) == 1
    assert instances[0].writes == [
        COMMANDS["raise"],
        COMMANDS["raise"],
        COMMANDS["lower"],
        COMMANDS["stop"],
    ]
    assert instances[0].flush_count == 4


def test_disconnect_can_send_one_safety_stop():
    instance = FakeSerial()
    controller = LiftSerialController(serial_factory=lambda **_: instance)
    controller.connect()
    controller.disconnect(safe_stop=True)

    assert instance.writes == [COMMANDS["stop"]]
    assert instance.is_open is False


def test_control_client_rejects_any_action_outside_allowlist_before_connecting():
    with pytest.raises(ValueError, match="unsupported"):
        send_action("shell command", "/tmp/socket-must-not-be-opened")
