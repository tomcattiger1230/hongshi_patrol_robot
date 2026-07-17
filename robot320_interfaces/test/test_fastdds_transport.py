from robot320_interfaces.fastdds_transport import ros_topic_to_dds


def test_ros_topics_use_standard_dds_prefix():
    assert ros_topic_to_dds("/robot320/command") == "rt/robot320/command"
    assert ros_topic_to_dds("rt/robot320/state") == "rt/robot320/state"
