import os
import unittest
from unittest.mock import patch

from video_relay.robot.push_stream import add_credentials, build_command, redact_command


class PushStreamTests(unittest.TestCase):
    def test_add_credentials_encodes_reserved_characters(self) -> None:
        actual = add_credentials("rtsp://192.168.1.64:554/live", "admin", "p@ss:word")
        self.assertEqual(actual, "rtsp://admin:p%40ss%3Aword@192.168.1.64:554/live")

    @patch.dict(
        os.environ,
        {
            "CAMERA_RTSP_URL": "rtsp://192.168.1.64:554/Streaming/Channels/101",
            "CAMERA_USER": "admin",
            "CAMERA_PASSWORD": "camera-secret",
            "SERVER_HOST": "relay.example.com",
            "PUBLISH_USER": "publisher",
            "PUBLISH_PASSWORD": "server-secret",
            "VIDEO_MODE": "copy",
        },
        clear=True,
    )
    def test_copy_command_uses_tcp_and_does_not_transcode(self) -> None:
        command = build_command()
        self.assertIn("copy", command)
        self.assertNotIn("libx264", command)
        self.assertEqual(command.count("-rtsp_transport"), 2)
        safe_log = redact_command(command)
        self.assertNotIn("camera-secret", safe_log)
        self.assertNotIn("server-secret", safe_log)

    @patch.dict(
        os.environ,
        {
            "CAMERA_RTSP_URL": "rtsp://192.168.1.64/live",
            "SERVER_HOST": "relay.example.com",
            "PUBLISH_USER": "publisher",
            "PUBLISH_PASSWORD": "secret",
            "VIDEO_MODE": "transcode",
            "VIDEO_ENCODER": "h264_qsv",
        },
        clear=True,
    )
    def test_intel_qsv_transcode_command(self) -> None:
        command = build_command()
        self.assertIn("h264_qsv", command)
        self.assertIn("format=nv12", command)
        self.assertNotIn("libx264", command)


if __name__ == "__main__":
    unittest.main()
