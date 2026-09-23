import os
from unittest.mock import patch

from video_relay.robot.local_rtsp_proxy import build_config


def test_second_camera_is_optional_and_uses_distinct_stream_path():
    config = {
        'CAMERA_RTSP_URL': 'rtsp://192.168.1.64:554/Streaming/Channels/102',
        'CAMERA_USER': 'test', 'CAMERA_PASSWORD': 'test-only',
        'LOCAL_READ_USER': 'viewer', 'LOCAL_READ_PASSWORD': 'test-only',
    }
    with patch.dict(os.environ, config, clear=True):
        assert set(build_config()['streams']) == {'robot'}
    config['CAMERA_SECOND_RTSP_URL'] = 'rtsp://192.168.1.64:554/Streaming/Channels/202'
    with patch.dict(os.environ, config, clear=True):
        streams = build_config()['streams']
        assert streams['robot'].endswith('/102')
        assert streams['robot2'].endswith('/202')
