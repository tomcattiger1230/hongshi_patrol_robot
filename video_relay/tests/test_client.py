import os
import unittest
from unittest.mock import patch

from video_relay.local.client import stream_urls


class ClientStreamTests(unittest.TestCase):
    @patch.dict(
        os.environ,
        {
            "SERVER_HOST": "relay.example.com",
            "SERVER_PORT": "8554",
            "STREAM_PATH": "robot",
            "READ_USER": "viewer",
            "READ_PASSWORD": "secret",
            "LOCAL_SERVER_HOST": "10.0.0.2",
            "LOCAL_SERVER_PORT": "8555",
        },
        clear=True,
    )
    def test_local_stream_is_preferred_with_public_fallback(self) -> None:
        sources = stream_urls()
        self.assertEqual([name for name, _url in sources], ["局域网", "公网"])
        self.assertEqual(sources[0][1], "rtsp://viewer:secret@10.0.0.2:8555/robot")
        self.assertEqual(sources[1][1], "rtsp://viewer:secret@relay.example.com:8554/robot")


if __name__ == "__main__":
    unittest.main()
