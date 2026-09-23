import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")
from PySide6.QtWidgets import QApplication  # noqa: E402
from remote_control.industrial_camera_panel import CAMERA_NAMES, IndustrialCameraPanel  # noqa: E402


def test_industrial_cameras_never_capture_until_clicked():
    app = QApplication.instance() or QApplication([])
    panel = IndustrialCameraPanel(auto_tunnel=False)
    panel.show()
    app.processEvents()
    assert panel.capture_processes == {}
    assert all("尚未请求" in image.text() for image in panel.images.values())
    assert all(button.isEnabled() for button in panel.buttons.values())
    assert panel.link_mode == "auto"
    assert CAMERA_NAMES == {1: "右相机", 2: "左相机"}
    assert panel.camera_order == (2, 1)
    assert panel.scroll_area.widgetResizable()
    assert panel.record_process is None
    assert panel.record_timer.interval() >= 1000
    panel.shutdown()
    panel.close()


def test_record_command_uses_allowlisted_parameters_and_nuc_storage_only():
    _app = QApplication.instance() or QApplication([])
    panel = IndustrialCameraPanel(auto_tunnel=False)
    args = panel.record_arguments("start", "lan")
    assert "/home/hs/robot320_remote_spatial/hik_usb_record.py" in args
    assert args[-10:] == [
        "--camera", "both", "--fps", "5", "--width", "2048",
        "--height", "1230", "--duration", "60",
    ]
    status = panel.record_arguments("status", "cloud")
    assert status[-2:] == ["status", "--include-preview"]
    with pytest.raises(ValueError):
        panel.record_arguments("delete", "lan")
    panel.shutdown()
    panel.close()


def test_nuc_mode_uses_local_camera_programs_without_ssh():
    _app = QApplication.instance() or QApplication([])
    panel = IndustrialCameraPanel(auto_tunnel=False, transport="local")
    capture = panel.capture_arguments(2, "local")
    record = panel.record_arguments("start", "local")
    assert capture[0] == "/home/hs/robot320_remote_spatial/hik_usb_snapshot.py"
    assert record[0] == "/home/hs/robot320_remote_spatial/hik_usb_record.py"
    assert "ssh" not in capture + record
    assert panel.link_mode == "local"
    assert not panel.link_selector.isEnabled()
    panel.shutdown()
    panel.close()


def test_camera_command_is_fixed_to_two_allowed_devices():
    _app = QApplication.instance() or QApplication([])
    panel = IndustrialCameraPanel(auto_tunnel=False)
    args = panel.capture_arguments(1, "lan")
    assert args[-4:] == ["--camera", "1", "--quality", "75"]
    assert "/home/hs/robot320_remote_spatial/hik_usb_snapshot.py" in args
    assert "hs@hs-nuc14lnk.local" in args
    cloud_args = panel.capture_arguments(2, "cloud")
    assert "hs@127.0.0.1" in cloud_args
    assert str(panel.local_port) in cloud_args
    with pytest.raises(ValueError):
        panel.capture_arguments(3, "lan")
    panel.shutdown()
    panel.close()
