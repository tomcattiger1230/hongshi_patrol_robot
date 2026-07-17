"""Standalone Fast DDS transport interoperable with ROS 2 String topics."""

from __future__ import annotations

import importlib
import queue
import time
from types import ModuleType
from typing import Any, Callable, Optional

from .messages import (
    CommandReply,
    Heartbeat,
    RobotCommand,
    RobotTelemetry,
    heartbeat_from_json,
    reply_from_json,
    robot_command_from_json,
    telemetry_from_json,
    to_json,
)


COMMAND_TOPIC = "/robot320/command"
STATE_TOPIC = "/robot320/state"
REPLY_TOPIC = "/robot320/reply"
HEARTBEAT_TOPIC = "/robot320/heartbeat"


class FastDDSUnavailable(RuntimeError):
    pass


def ros_topic_to_dds(topic_name: str) -> str:
    """Map a ROS 2 topic to its standard DDS topic name."""
    if topic_name.startswith(("rt/", "rq/", "rr/")):
        return topic_name
    return "rt/" + topic_name.lstrip("/")


class FastDdsParticipant:
    """Fast DDS participant using ROS 2's ``std_msgs/msg/String`` wire type."""

    def __init__(self, domain_id: int = 20, participant_name: str = "robot320"):
        self.fastdds, self.types = _load_fastdds_modules()
        self.factory = self.fastdds.DomainParticipantFactory.get_instance()
        participant_qos = self.fastdds.DomainParticipantQos()
        self.factory.get_default_participant_qos(participant_qos)
        _set_participant_name(participant_qos, participant_name)
        self.participant = self.factory.create_participant(domain_id, participant_qos)
        if self.participant is None:
            raise FastDDSUnavailable(
                f"failed to create Fast DDS participant in domain {domain_id}"
            )

        publisher_qos = self.fastdds.PublisherQos()
        self.participant.get_default_publisher_qos(publisher_qos)
        self.publisher = self.participant.create_publisher(publisher_qos)
        subscriber_qos = self.fastdds.SubscriberQos()
        self.participant.get_default_subscriber_qos(subscriber_qos)
        self.subscriber = self.participant.create_subscriber(subscriber_qos)

        self._topic_type = self.types.String_PubSubType()
        self._type_support = self.fastdds.TypeSupport(self._topic_type)
        self.participant.register_type(self._type_support)
        self._topics: dict[str, Any] = {}
        self._listeners: list[Any] = []
        self._closed = False

    def create_writer(self, topic_name: str):
        topic = self._topic(topic_name)
        qos = self.fastdds.DataWriterQos()
        self.publisher.get_default_datawriter_qos(qos)
        qos.reliability().kind = self.fastdds.RELIABLE_RELIABILITY_QOS
        writer = self.publisher.create_datawriter(topic, qos)
        if writer is None:
            raise FastDDSUnavailable(f"failed to create writer for {topic_name}")
        return writer

    def create_reader(self, topic_name: str, callback: Callable[[str], None]):
        topic = self._topic(topic_name)
        string_class = self.types.String_
        fastdds = self.fastdds

        class StringListener(fastdds.DataReaderListener):
            def __init__(self):
                super().__init__()

            def on_data_available(self, reader):
                while True:
                    sample = string_class()
                    info = fastdds.SampleInfo()
                    if reader.take_next_sample(sample, info) != fastdds.RETCODE_OK:
                        break
                    valid = getattr(info, "valid_data", True)
                    if callable(valid):
                        valid = valid()
                    if valid:
                        callback(sample.data())

        listener = StringListener()
        qos = self.fastdds.DataReaderQos()
        self.subscriber.get_default_datareader_qos(qos)
        qos.reliability().kind = self.fastdds.RELIABLE_RELIABILITY_QOS
        reader = self.subscriber.create_datareader(topic, qos, listener)
        if reader is None:
            raise FastDDSUnavailable(f"failed to create reader for {topic_name}")
        self._listeners.append(listener)
        return reader

    def write_string(self, writer, payload: str) -> None:
        sample = self.types.String_()
        sample.data(payload)
        writer.write(sample)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.participant.delete_contained_entities()
        self.factory.delete_participant(self.participant)

    def _topic(self, topic_name: str):
        dds_name = ros_topic_to_dds(topic_name)
        existing = self._topics.get(dds_name)
        if existing is not None:
            return existing
        topic_qos = self.fastdds.TopicQos()
        self.participant.get_default_topic_qos(topic_qos)
        topic = self.participant.create_topic(
            dds_name, _get_type_name(self._topic_type), topic_qos
        )
        if topic is None:
            raise FastDDSUnavailable(f"failed to create topic {dds_name}")
        self._topics[dds_name] = topic
        return topic


class FastDdsRemoteTransport:
    backend = "fastdds"

    def __init__(self, domain_id: int = 20, client_id: str = "remote_control"):
        self.client_id = client_id
        self.runtime = FastDdsParticipant(domain_id, f"robot320_remote_{client_id}")
        self._states: queue.Queue[RobotTelemetry] = queue.Queue()
        self._replies: queue.Queue[CommandReply] = queue.Queue()
        self._heartbeats: queue.Queue[Heartbeat] = queue.Queue()
        self._command_writer = self.runtime.create_writer(COMMAND_TOPIC)
        self._heartbeat_writer = self.runtime.create_writer(HEARTBEAT_TOPIC)
        self.runtime.create_reader(STATE_TOPIC, self._on_state)
        self.runtime.create_reader(REPLY_TOPIC, self._on_reply)
        self.runtime.create_reader(HEARTBEAT_TOPIC, self._on_heartbeat)

    def publish_command(self, command: RobotCommand) -> None:
        self.runtime.write_string(self._command_writer, to_json(command))

    def publish_heartbeat(self, sequence: int) -> None:
        heartbeat = Heartbeat(
            node_id=self.client_id,
            role="remote",
            sequence=sequence,
            timestamp_ms=int(time.time() * 1000.0),
        )
        self.runtime.write_string(self._heartbeat_writer, to_json(heartbeat))

    def receive_state(self, timeout_s: float = 0.1) -> Optional[RobotTelemetry]:
        return _queue_get(self._states, timeout_s)

    def receive_reply(self, timeout_s: float = 0.1) -> Optional[CommandReply]:
        return _queue_get(self._replies, timeout_s)

    def receive_heartbeat(self, timeout_s: float = 0.1) -> Optional[Heartbeat]:
        return _queue_get(self._heartbeats, timeout_s)

    def close(self) -> None:
        self.runtime.close()

    def _on_state(self, payload: str) -> None:
        self._states.put(telemetry_from_json(payload))

    def _on_reply(self, payload: str) -> None:
        self._replies.put(reply_from_json(payload))

    def _on_heartbeat(self, payload: str) -> None:
        heartbeat = heartbeat_from_json(payload)
        if heartbeat.role == "robot":
            self._heartbeats.put(heartbeat)


class FastDdsRobotTransport:
    """ROS-free compatibility transport; the production NUC uses rclpy."""

    backend = "fastdds"

    def __init__(self, domain_id: int = 20, robot_id: str = "robot320"):
        self.robot_id = robot_id
        self.runtime = FastDdsParticipant(domain_id, f"robot320_nuc_{robot_id}")
        self._commands: queue.Queue[RobotCommand] = queue.Queue()
        self._heartbeats: queue.Queue[Heartbeat] = queue.Queue()
        self._state_writer = self.runtime.create_writer(STATE_TOPIC)
        self._reply_writer = self.runtime.create_writer(REPLY_TOPIC)
        self._heartbeat_writer = self.runtime.create_writer(HEARTBEAT_TOPIC)
        self.runtime.create_reader(COMMAND_TOPIC, self._on_command)
        self.runtime.create_reader(HEARTBEAT_TOPIC, self._on_heartbeat)

    def receive_command(self, timeout_s: float = 0.1) -> Optional[RobotCommand]:
        return _queue_get(self._commands, timeout_s)

    def receive_heartbeat(self, timeout_s: float = 0.1) -> Optional[Heartbeat]:
        return _queue_get(self._heartbeats, timeout_s)

    def publish_state(self, telemetry: RobotTelemetry, sequence: int) -> None:
        del sequence  # Sequence remains inside the JSON application payload where needed.
        telemetry.robot_id = self.robot_id
        self.runtime.write_string(self._state_writer, to_json(telemetry))

    def publish_reply(self, reply: CommandReply) -> None:
        self.runtime.write_string(self._reply_writer, to_json(reply))

    def publish_heartbeat(self, sequence: int) -> None:
        heartbeat = Heartbeat(
            node_id=self.robot_id,
            role="robot",
            sequence=sequence,
            timestamp_ms=int(time.time() * 1000.0),
        )
        self.runtime.write_string(self._heartbeat_writer, to_json(heartbeat))

    def close(self) -> None:
        self.runtime.close()

    def _on_command(self, payload: str) -> None:
        self._commands.put(robot_command_from_json(payload))

    def _on_heartbeat(self, payload: str) -> None:
        heartbeat = heartbeat_from_json(payload)
        if heartbeat.role == "remote":
            self._heartbeats.put(heartbeat)


def _load_fastdds_modules() -> tuple[ModuleType, ModuleType]:
    try:
        fastdds = importlib.import_module("fastdds")
    except ImportError as exc:
        raise FastDDSUnavailable(
            "Fast DDS Python bindings are unavailable. Run "
            "'./scripts/setup_fastdds.sh' or source an existing Fast-DDS-python "
            "installation."
        ) from exc
    try:
        generated = importlib.import_module("Robot320String")
    except ImportError as exc:
        raise FastDDSUnavailable(
            "ROS 2 String type support is unavailable. Run "
            "'./scripts/setup_fastdds.sh' and retry."
        ) from exc
    return fastdds, generated


def _set_participant_name(qos, name: str) -> None:
    try:
        qos.name(name)
    except (AttributeError, TypeError):
        pass


def _get_type_name(topic_type) -> str:
    getter = getattr(topic_type, "get_name", None) or getattr(topic_type, "getName", None)
    if getter is None:
        raise FastDDSUnavailable("generated PubSubType has no type-name getter")
    return getter()


def _queue_get(items: queue.Queue, timeout_s: float):
    try:
        return items.get(timeout=max(0.0, timeout_s))
    except queue.Empty:
        return None
