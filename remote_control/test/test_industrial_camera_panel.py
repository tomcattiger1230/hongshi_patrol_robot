import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")
from PySide6.QtWidgets import QApplication  # noqa: E402
from remote_control.industrial_camera_panel import IndustrialCameraPanel  # noqa: E402


def test_industrial_cameras_never_capture_until_clicked():
    app = QApplication.instance() or QApplication([])
    panel = IndustrialCameraPanel(auto_tunnel=False)
    panel.show()
    app.processEvents()
    assert panel.capture_processes == {}
    assert all("尚未请求" in image.text() for image in panel.images.values())
    assert all(not button.isEnabled() for button in panel.buttons.values())
    panel.shutdown()
    panel.close()


def test_camera_command_is_fixed_to_two_allowed_devices():
    _app = QApplication.instance() or QApplication([])
    panel = IndustrialCameraPanel(auto_tunnel=False)
    panel.tunnel.start("/usr/bin/true", [])
    panel.tunnel.waitForFinished()
    panel.buttons[1].setEnabled(True)
    panel.capture(1)
    process = panel.capture_processes[1]
    args = process.arguments()
    assert args[-4:] == ["--camera", "1", "--quality", "75"]
    assert "/home/hs/robot320_remote_spatial/hik_usb_snapshot.py" in args
    process.kill()
    process.waitForFinished()
    panel.shutdown()
    panel.close()
