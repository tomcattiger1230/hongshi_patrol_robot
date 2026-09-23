"""Small local ROS 2 String participant used by the NUC GUI panels."""

from __future__ import annotations

import threading


class Ros2StringUnavailable(RuntimeError):
    pass


class Ros2StringParticipant:
    def __init__(self, domain_id=20, participant_name="robot320_nuc_gui"):
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from std_msgs.msg import String
        except ImportError as exc:
            raise Ros2StringUnavailable(
                "NUC 本机 ROS 2 Python 环境不可用；请通过 scripts/start_nuc_gui.sh 启动。"
            ) from exc
        self.rclpy = rclpy
        self.string_type = String
        self.context = Context()
        self.context.init(args=None, domain_id=domain_id)
        self.node = rclpy.create_node(participant_name, context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.publishers = []
        self.subscriptions = []
        self.closed = False
        self.thread = threading.Thread(
            target=self.executor.spin, name="robot320-nuc-spatial-ros2", daemon=True,
        )
        self.thread.start()

    def create_reader(self, topic_name, callback):
        subscription = self.node.create_subscription(
            self.string_type, topic_name, lambda message: callback(message.data), 10,
        )
        self.subscriptions.append(subscription)
        return subscription

    def create_writer(self, topic_name):
        publisher = self.node.create_publisher(self.string_type, topic_name, 10)
        self.publishers.append(publisher)
        return publisher

    def write_string(self, writer, payload):
        message = self.string_type()
        message.data = payload
        writer.publish(message)

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.executor.shutdown(timeout_sec=1.0)
        self.thread.join(timeout=1.5)
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        self.context.shutdown()
