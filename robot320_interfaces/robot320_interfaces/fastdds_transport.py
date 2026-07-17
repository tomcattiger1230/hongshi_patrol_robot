"""Fast DDS runtime shared by the NUC bridge and ROS-independent remote app."""

from __future__ import annotations

import importlib
import queue
import time
from dataclasses import dataclass
from types import ModuleType
from typing import Any, Callable, Optional

from .messages import (
    CommandReply,
    RobotCommand,
    RobotTelemetry,
    reply_from_json,
    robot_command_from_json,
    telemetry_from_json,
    to_json,
)


COMMAND_TOPIC = "robot320/command"
STATE_TOPIC = "robot320/state"
REPLY_TOPIC = "robot320/reply"
HEARTBEAT_TOPIC = "robot320/heartbeat"


class FastDDSUnavailable(RuntimeError):
    pass


@dataclass(frozen=True)
class Heartbeat:
    node_id: str
    role: str
    sequence: int
    timestamp_ms: int


class FastDdsParticipant:
    """Small owner for Fast DDS entities using generated Robot320Dds types."""

    def __init__(self, domain_id: int = 20, participant_name: str = "robot320"):
        self.fastdds, self.types = _load_fastdds_modules()
        self.factory = self.fastdds.DomainParticipantFactory.get_instance()
        participant_qos = self.fastdds.DomainParticipantQos()
        self.factory.get_default_participant_qos(participant_qos)
        _set_participant_name(participant_qos, participant_name)
        self.participant = self.factory.create_participant(domain_id, participant_qos)
        if self.participant is None:
            raise FastDDSUnavailable(f"failed to create Fast DDS participant in domain {domain_id}")

        publisher_qos = self.fastdds.PublisherQos()
        self.participant.get_default_publisher_qos(publisher_qos)
        self.publisher = self.participant.create_publisher(publisher_qos)
        subscriber_qos = self.fastdds.SubscriberQos()
        self.participant.get_default_subscriber_qos(subscriber_qos)
        self.subscriber = self.participant.create_subscriber(subscriber_qos)
        self._registered_types: dict[str, tuple[Any, Any]] = {}
        self._topics: dict[str, tuple[str, Any]] = {}
        self._listeners: list[Any] = []
        self._closed = False

    def create_writer(self, topic_name: str, type_name: str):
        topic = self._topic(topic_name, type_name)
        qos = self.fastdds.DataWriterQos()
        self.publisher.get_default_datawriter_qos(qos)
        qos.reliability().kind = self.fastdds.RELIABLE_RELIABILITY_QOS
        writer = self.publisher.create_datawriter(topic, qos)
        if writer is None:
            raise FastDDSUnavailable(f"failed to create writer for {topic_name}")
        return writer

    def create_reader(self, topic_name: str, type_name: str, callback: Callable[[Any], None]):
        topic = self._topic(topic_name, type_name)
        generated_class = getattr(self.types, type_name)
        fastdds = self.fastdds

        class QueueingListener(fastdds.DataReaderListener):
            def __init__(self):
                super().__init__()

            def on_data_available(self, reader):
                while True:
                    sample = generated_class()
                    info = fastdds.SampleInfo()
                    if reader.take_next_sample(sample, info) != fastdds.RETCODE_OK:
                        break
                    valid = getattr(info, "valid_data", True)
                    if callable(valid):
                        valid = valid()
                    if valid:
                        callback(sample)

        listener = QueueingListener()
        qos = self.fastdds.DataReaderQos()
        self.subscriber.get_default_datareader_qos(qos)
        qos.reliability().kind = self.fastdds.RELIABLE_RELIABILITY_QOS
        reader = self.subscriber.create_datareader(topic, qos, listener)
        if reader is None:
            raise FastDDSUnavailable(f"failed to create reader for {topic_name}")
        self._listeners.append(listener)
        return reader

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.participant.delete_contained_entities()
        self.factory.delete_participant(self.participant)

    def _topic(self, topic_name: str, type_name: str):
        existing = self._topics.get(topic_name)
        if existing is not None:
            existing_type, topic = existing
            if existing_type != type_name:
                raise FastDDSUnavailable(
                    f"topic {topic_name} already uses {existing_type}, not {type_name}"
                )
            return topic
        type_support, topic_type = self._register_type(type_name)
        del type_support  # Kept alive in _registered_types.
        topic_qos = self.fastdds.TopicQos()
        self.participant.get_default_topic_qos(topic_qos)
        topic = self.participant.create_topic(topic_name, _get_type_name(topic_type), topic_qos)
        if topic is None:
            raise FastDDSUnavailable(f"failed to create topic {topic_name}")
        self._topics[topic_name] = (type_name, topic)
        return topic

    def _register_type(self, type_name: str):
        registered = self._registered_types.get(type_name)
        if registered:
            return registered
        topic_type_class = getattr(self.types, f"{type_name}PubSubType")
        topic_type = topic_type_class()
        _set_type_name(topic_type, type_name)
        support = self.fastdds.TypeSupport(topic_type)
        self.participant.register_type(support)
        registered = (support, topic_type)
        self._registered_types[type_name] = registered
        return registered


class FastDdsRemoteTransport:
    def __init__(self, domain_id: int = 20, client_id: str = "remote_control"):
        self.client_id = client_id
        self.runtime = FastDdsParticipant(domain_id, f"robot320_remote_{client_id}")
        self._states: queue.Queue[RobotTelemetry] = queue.Queue()
        self._replies: queue.Queue[CommandReply] = queue.Queue()
        self._heartbeats: queue.Queue[Heartbeat] = queue.Queue()
        self._command_writer = self.runtime.create_writer(
            COMMAND_TOPIC, "Robot320CommandEnvelope"
        )
        self._heartbeat_writer = self.runtime.create_writer(
            HEARTBEAT_TOPIC, "Robot320HeartbeatEnvelope"
        )
        self.runtime.create_reader(
            STATE_TOPIC, "Robot320StateEnvelope", self._on_state
        )
        self.runtime.create_reader(
            REPLY_TOPIC, "Robot320ReplyEnvelope", self._on_reply
        )
        self.runtime.create_reader(
            HEARTBEAT_TOPIC, "Robot320HeartbeatEnvelope", self._on_heartbeat
        )

    def publish_command(self, command: RobotCommand) -> None:
        data = self.runtime.types.Robot320CommandEnvelope()
        data.command_id(command.command_id)
        data.client_id(command.client_id)
        data.sequence_number(command.sequence)
        data.timestamp_ms(_milliseconds(command.stamp))
        data.command_json(to_json(command))
        self._command_writer.write(data)

    def publish_heartbeat(self, sequence: int) -> None:
        data = self.runtime.types.Robot320HeartbeatEnvelope()
        data.node_id(self.client_id)
        data.role("remote")
        data.sequence_number(sequence)
        data.timestamp_ms(_milliseconds(time.time()))
        self._heartbeat_writer.write(data)

    def receive_state(self, timeout_s: float = 0.1) -> Optional[RobotTelemetry]:
        return _queue_get(self._states, timeout_s)

    def receive_reply(self, timeout_s: float = 0.1) -> Optional[CommandReply]:
        return _queue_get(self._replies, timeout_s)

    def receive_heartbeat(self, timeout_s: float = 0.1) -> Optional[Heartbeat]:
        return _queue_get(self._heartbeats, timeout_s)

    def close(self) -> None:
        self.runtime.close()

    def _on_state(self, data) -> None:
        self._states.put(telemetry_from_json(data.state_json()))

    def _on_reply(self, data) -> None:
        self._replies.put(reply_from_json(data.reply_json()))

    def _on_heartbeat(self, data) -> None:
        heartbeat = _heartbeat_from_dds(data)
        if heartbeat.role == "robot":
            self._heartbeats.put(heartbeat)


class FastDdsRobotTransport:
    def __init__(self, domain_id: int = 20, robot_id: str = "robot320"):
        self.robot_id = robot_id
        self.runtime = FastDdsParticipant(domain_id, f"robot320_nuc_{robot_id}")
        self._commands: queue.Queue[RobotCommand] = queue.Queue()
        self._heartbeats: queue.Queue[Heartbeat] = queue.Queue()
        self._state_writer = self.runtime.create_writer(STATE_TOPIC, "Robot320StateEnvelope")
        self._reply_writer = self.runtime.create_writer(REPLY_TOPIC, "Robot320ReplyEnvelope")
        self._heartbeat_writer = self.runtime.create_writer(
            HEARTBEAT_TOPIC, "Robot320HeartbeatEnvelope"
        )
        self.runtime.create_reader(
            COMMAND_TOPIC, "Robot320CommandEnvelope", self._on_command
        )
        self.runtime.create_reader(
            HEARTBEAT_TOPIC, "Robot320HeartbeatEnvelope", self._on_heartbeat
        )

    def receive_command(self, timeout_s: float = 0.1) -> Optional[RobotCommand]:
        return _queue_get(self._commands, timeout_s)

    def receive_heartbeat(self, timeout_s: float = 0.1) -> Optional[Heartbeat]:
        return _queue_get(self._heartbeats, timeout_s)

    def publish_state(self, telemetry: RobotTelemetry, sequence: int) -> None:
        telemetry.robot_id = self.robot_id
        data = self.runtime.types.Robot320StateEnvelope()
        data.robot_id(self.robot_id)
        data.sequence_number(sequence)
        data.timestamp_ms(_milliseconds(telemetry.stamp))
        data.state_json(to_json(telemetry))
        self._state_writer.write(data)

    def publish_reply(self, reply: CommandReply) -> None:
        data = self.runtime.types.Robot320ReplyEnvelope()
        data.command_id(reply.command_id)
        data.robot_id(self.robot_id)
        data.sequence_number(reply.sequence)
        data.timestamp_ms(_milliseconds(reply.stamp))
        data.reply_json(to_json(reply))
        self._reply_writer.write(data)

    def publish_heartbeat(self, sequence: int) -> None:
        data = self.runtime.types.Robot320HeartbeatEnvelope()
        data.node_id(self.robot_id)
        data.role("robot")
        data.sequence_number(sequence)
        data.timestamp_ms(_milliseconds(time.time()))
        self._heartbeat_writer.write(data)

    def close(self) -> None:
        self.runtime.close()

    def _on_command(self, data) -> None:
        command = robot_command_from_json(data.command_json())
        if command.command_id == data.command_id() and command.client_id == data.client_id():
            self._commands.put(command)

    def _on_heartbeat(self, data) -> None:
        heartbeat = _heartbeat_from_dds(data)
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
        generated = importlib.import_module("Robot320Dds")
    except ImportError as exc:
        raise FastDDSUnavailable(
            "Generated Robot320Dds module is unavailable. Run "
            "robot320_interfaces/scripts/generate_fastdds_types.sh and add its build "
            "directory to PYTHONPATH."
        ) from exc
    return fastdds, generated


def _set_participant_name(qos, name: str) -> None:
    try:
        qos.name(name)
    except (AttributeError, TypeError):
        pass


def _set_type_name(topic_type, name: str) -> None:
    setter = getattr(topic_type, "set_name", None) or getattr(topic_type, "setName", None)
    if setter is None:
        raise FastDDSUnavailable("generated PubSubType has no type-name setter")
    setter(name)


def _get_type_name(topic_type) -> str:
    getter = getattr(topic_type, "get_name", None) or getattr(topic_type, "getName", None)
    if getter is None:
        raise FastDDSUnavailable("generated PubSubType has no type-name getter")
    return getter()


def _heartbeat_from_dds(data) -> Heartbeat:
    return Heartbeat(
        node_id=data.node_id(),
        role=data.role(),
        sequence=data.sequence_number(),
        timestamp_ms=data.timestamp_ms(),
    )


def _milliseconds(stamp: float) -> int:
    return int(stamp * 1000.0)


def _queue_get(items: queue.Queue, timeout_s: float):
    try:
        return items.get(timeout=max(0.0, timeout_s))
    except queue.Empty:
        return None
