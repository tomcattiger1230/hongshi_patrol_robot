import sys
import types

from remote_control.ros2_string_transport import Ros2StringParticipant


class FakeContext:
    def __init__(self):
        self.domain_id = None

    def init(self, args=None, domain_id=None):
        self.domain_id = domain_id

    def shutdown(self):
        pass


class FakeExecutor:
    def __init__(self, context=None):
        self.context = context

    def add_node(self, node):
        self.node = node

    def spin(self):
        pass

    def shutdown(self, timeout_sec=None):
        pass

    def remove_node(self, node):
        pass


class FakeString:
    data = ""


class FakeNode:
    def create_subscription(self, _type, topic, callback, qos):
        return (topic, callback, qos)

    def create_publisher(self, _type, topic, qos):
        class Publisher:
            def __init__(self):
                self.topic = topic
                self.qos = qos
                self.published = []

            def publish(self, message):
                self.published.append(message)

        return Publisher()

    def destroy_node(self):
        pass


def test_local_ros2_string_participant_uses_isolated_domain_context(monkeypatch):
    rclpy = types.ModuleType("rclpy")
    rclpy.create_node = lambda _name, context=None: FakeNode()
    context_module = types.ModuleType("rclpy.context")
    context_module.Context = FakeContext
    executor_module = types.ModuleType("rclpy.executors")
    executor_module.SingleThreadedExecutor = FakeExecutor
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = FakeString
    for name, module in {
        "rclpy": rclpy, "rclpy.context": context_module,
        "rclpy.executors": executor_module, "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)
    participant = Ros2StringParticipant(20, "test_nuc_gui")
    received = []
    reader = participant.create_reader("/scan", received.append)
    writer = participant.create_writer("/control")
    participant.write_string(writer, "start")
    assert participant.context.domain_id == 20
    assert reader[0] == "/scan"
    assert writer.published[0].data == "start"
    participant.close()
