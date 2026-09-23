import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")

from PySide6.QtCore import QObject, Signal  # noqa: E402
from PySide6.QtNetwork import QNetworkReply  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402
from remote_control.camera_panel import CameraPanel, camera_bridge_url  # noqa: E402


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


@pytest.mark.parametrize("url", [
    "http://192.168.88.75:8090", "https://localhost", "http://user:secret@localhost",
    "http://localhost/path", "http://localhost?token=x", "http://localhost:bad",
])
def test_camera_bridge_rejects_remote_credentials_and_non_origins(url):
    with pytest.raises(ValueError):
        camera_bridge_url(url)


def test_camera_bridge_accepts_loopback_origin():
    assert camera_bridge_url("http://127.0.0.1:8081/") == "http://127.0.0.1:8081"


def test_camera_is_disabled_without_explicit_url(app):
    panel = CameraPanel()
    assert not panel.frame_timer.isActive()
    assert all(not button.isEnabled() for button in panel.buttons)
    panel.start_move("left")
    assert not panel._direction
    panel.shutdown()


def test_video_geometry_is_equal_despite_different_frame_sizes(app):
    from PySide6.QtGui import QPixmap

    panel = CameraPanel()
    panel.resize(1100, 700)
    panel.show()
    panel._pixmap = QPixmap(2560, 1440)
    panel._second_pixmap = QPixmap(640, 360)
    app.processEvents()
    panel._scale_image()
    app.processEvents()
    assert panel.image.size() == panel.second_image.size()
    assert panel.link_selectors[1].geometry().height() == panel.link_selectors[2].geometry().height()
    panel.shutdown()
    panel.close()


def test_delayed_move_response_is_followed_by_another_stop(app, monkeypatch):
    class FakeReply(QObject):
        finished = Signal()

        def error(self):
            return QNetworkReply.NetworkError.NoError

    panel = CameraPanel("http://127.0.0.1:8081")
    panel.frame_timer.stop()
    requests = []

    def request(path, payload=None):
        reply = FakeReply(panel)
        requests.append((path, reply))
        return reply

    monkeypatch.setattr(panel, "_request", request)
    panel.start_move("left")
    panel.stop_move()
    requests[0][1].finished.emit()
    assert [path for path, _ in requests] == [
        "/api/ptz/move", "/api/ptz/stop", "/api/ptz/stop",
    ]
    panel.shutdown()
