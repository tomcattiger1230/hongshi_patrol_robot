import json
import threading
import time
import unittest
from dataclasses import dataclass
from http.server import ThreadingHTTPServer
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from video_relay.robot.ptz_agent import (
    PtzConfig,
    PtzController,
    direction_velocity,
    make_handler,
    soap_envelope,
)


class RecordingController(PtzController):
    def __init__(self, config: PtzConfig) -> None:
        super().__init__(config)
        self.requests: list[str] = []

    def _request(self, body: str) -> None:
        self.requests.append(body)


@dataclass
class FakeController:
    config: PtzConfig
    last_move: tuple[str, float] | None = None
    stopped: bool = False

    def move(self, direction: str, speed: float) -> None:
        direction_velocity(direction, speed)
        self.last_move = (direction, speed)

    def stop(self) -> None:
        self.stopped = True


class PtzAgentTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = PtzConfig(
            service_url="http://192.0.2.1/onvif/PTZ",
            username="admin<&",
            password="camera-secret",
            profile_token="Profile_101",
            api_token="agent-secret",
        )

    def test_direction_velocity(self) -> None:
        self.assertEqual(direction_velocity("left", 0.3), (-0.3, 0.0, 0.0))
        self.assertEqual(direction_velocity("zoom_in", 0.5), (0.0, 0.0, 0.5))
        with self.assertRaises(ValueError):
            direction_velocity("diagonal", 0.3)
        with self.assertRaises(ValueError):
            direction_velocity("up", 2.0)

    def test_soap_does_not_contain_plaintext_password(self) -> None:
        envelope = soap_envelope(self.config, "<tptz:Stop/>").decode()
        self.assertNotIn("camera-secret", envelope)
        self.assertIn("admin&lt;&amp;", envelope)
        self.assertIn("PasswordDigest", envelope)

    def test_http_api_requires_token_and_forwards_commands(self) -> None:
        controller = FakeController(self.config)
        server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(controller))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base_url = f"http://127.0.0.1:{server.server_port}"
        try:
            try:
                urlopen(base_url + "/health")
            except HTTPError as unauthorized:
                self.assertEqual(unauthorized.code, 401)
                unauthorized.close()
            else:
                self.fail("health endpoint accepted a request without a token")

            payload = json.dumps({"direction": "right", "speed": 0.4}).encode()
            request = Request(
                base_url + "/move",
                data=payload,
                headers={
                    "Authorization": "Bearer agent-secret",
                    "Content-Type": "application/json",
                },
                method="POST",
            )
            with urlopen(request) as response:
                self.assertEqual(response.status, 200)
            self.assertEqual(controller.last_move, ("right", 0.4))
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)

    def test_deadman_stops_camera_if_commands_are_not_renewed(self) -> None:
        config = PtzConfig(
            service_url=self.config.service_url,
            username=self.config.username,
            password=self.config.password,
            profile_token=self.config.profile_token,
            api_token=self.config.api_token,
            deadman_timeout=0.05,
        )
        controller = RecordingController(config)
        controller.move("up", 0.3)
        time.sleep(0.12)
        self.assertEqual(len(controller.requests), 2)
        self.assertIn("ContinuousMove", controller.requests[0])
        self.assertIn("Stop", controller.requests[1])


if __name__ == "__main__":
    unittest.main()
