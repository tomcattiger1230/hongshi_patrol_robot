from robot320_interfaces.fastdds_transport import (
    FastDdsRemoteTransport,
    ros_topic_to_dds,
)


def test_ros_topics_use_standard_dds_prefix():
    assert ros_topic_to_dds("/robot320/command") == "rt/robot320/command"
    assert ros_topic_to_dds("rt/robot320/state") == "rt/robot320/state"


class _MatchedStatus:
    def __init__(self):
        self.current_count = 0


class _Endpoint:
    def __init__(self, count):
        self.count = count

    def get_publication_matched_status(self, status):
        status.current_count = self.count

    def get_subscription_matched_status(self, status):
        status.current_count = self.count


class _FastDdsApi:
    PublicationMatchedStatus = _MatchedStatus
    SubscriptionMatchedStatus = _MatchedStatus


def _transport_with_matches(command_readers: int, reply_writers: int):
    transport = FastDdsRemoteTransport.__new__(FastDdsRemoteTransport)
    transport.runtime = type("Runtime", (), {"fastdds": _FastDdsApi})()
    transport._command_writer = _Endpoint(command_readers)
    transport._reply_reader = _Endpoint(reply_writers)
    return transport


def test_command_path_requires_command_reader_and_reply_writer():
    assert _transport_with_matches(1, 1).wait_for_command_match(0.0) is True
    assert _transport_with_matches(1, 0).wait_for_command_match(0.0) is False
    assert _transport_with_matches(0, 1).wait_for_command_match(0.0) is False
