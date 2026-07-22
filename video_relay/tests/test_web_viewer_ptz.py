import unittest
from typing import Any

from video_relay.local.web_viewer import create_app


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
