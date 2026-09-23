import unittest
from typing import Any
from urllib.error import URLError

from video_relay.local.web_viewer import PtzProxy, create_app, ptz_tunnel_commands


class FakeReader:
    def frames(self) -> Any:
        return iter(())

    def snapshot(self) -> None:
        return None

    def status(self) -> dict[str, Any]:
        return {"state": "connecting", "message": "offline"}


class FakePtz:
    def __init__(self) -> None:
        self.calls: list[tuple[str, dict[str, Any] | None]] = []

    def request(self, path: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        self.calls.append((path, payload))
        return {"status": "ok", "message": "ok"}


class WebViewerPtzTests(unittest.TestCase):
    def test_ptz_cloud_tunnel_uses_fixed_reverse_selector(self) -> None:
        cloud, onboard = ptz_tunnel_commands(18022, 18090)
        self.assertIn("127.0.0.1:18022:127.0.0.1:12220", cloud)
        self.assertIn("hsjc_ecs", cloud)
        self.assertIn("127.0.0.1:18090:127.0.0.1:8090", onboard)
        self.assertIn("hs@127.0.0.1", onboard)

    def test_ptz_proxy_falls_back_to_cloud_and_keeps_using_it_for_moves(self) -> None:
        class Response:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            @staticmethod
            def read() -> bytes:
                return b'{"status":"ok","message":"connected"}'

        class Opener:
            def __init__(self) -> None:
                self.urls = []

            def open(self, request, timeout):
                self.urls.append((request.full_url, timeout))
                if "robot.local" in request.full_url:
                    raise URLError("offline")
                return Response()

        ptz = PtzProxy("http://robot.local:8090", "token", cloud_url="http://127.0.0.1:18090")
        opener = Opener()
        ptz.opener = opener
        self.assertIn("公网", ptz.request("/health")["message"])
        ptz.request("/move", {"direction": "left", "speed": 0.3})
        self.assertEqual([url for url, _timeout in opener.urls], [
            "http://robot.local:8090/health",
            "http://127.0.0.1:18090/health",
            "http://127.0.0.1:18090/move",
        ])

    def test_second_camera_routes_are_independent(self) -> None:
        class ImageReader(FakeReader):
            def __init__(self, frame: bytes) -> None:
                self.frame = frame

            def snapshot(self) -> bytes:
                return self.frame

        client = create_app(ImageReader(b'first'), second_reader=ImageReader(b'second')).test_client()
        self.assertEqual(client.get('/snapshot.jpg').data, b'first')
        self.assertEqual(client.get('/snapshot2.jpg').data, b'second')
        self.assertEqual(client.get('/api/status2').status_code, 200)
        self.assertIn('/video2', client.get('/').get_data(as_text=True))

    def test_unconfigured_second_camera_is_not_replaced_with_first(self) -> None:
        client = create_app(FakeReader()).test_client()
        self.assertEqual(client.get('/snapshot2.jpg').status_code, 503)
        self.assertEqual(client.get('/api/status2').get_json()['state'], 'disabled')

    def test_offline_ptz_is_reported_without_affecting_page(self) -> None:
        client = create_app(FakeReader()).test_client()  # type: ignore[arg-type]
        self.assertEqual(client.get("/").status_code, 200)
        response = client.get("/api/ptz/status")
        self.assertEqual(response.status_code, 503)
        self.assertIn("尚未配置", response.get_json()["message"])

    def test_move_is_validated_and_forwarded(self) -> None:
        ptz = FakePtz()
        client = create_app(FakeReader(), ptz).test_client()  # type: ignore[arg-type]
        invalid = client.post("/api/ptz/move", json={"direction": "diagonal"})
        self.assertEqual(invalid.status_code, 400)

        response = client.post("/api/ptz/move", json={"direction": "left", "speed": 0.4})
        self.assertEqual(response.status_code, 200)
        self.assertEqual(ptz.calls, [("/move", {"direction": "left", "speed": 0.4})])


if __name__ == "__main__":
    unittest.main()
