import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")
from PySide6.QtWidgets import QApplication  # noqa: E402
from remote_control.lift_panel import LiftPanel  # noqa: E402


def test_lift_buttons_are_disabled_until_authenticated_stream_is_ready():
    _app = QApplication.instance() or QApplication([])
    panel = LiftPanel(auto_tunnel=False)
    assert all(not button.isEnabled() for button in panel.buttons)
    assert panel.control.state() == panel.control.ProcessState.NotRunning
    panel.shutdown()
    assert panel._shutting_down is True
    panel.close()


def test_lift_control_command_is_fixed_and_allowlisted():
    _app = QApplication.instance() or QApplication([])
    panel = LiftPanel(auto_tunnel=False)
    panel._start_control()
    assert panel.control.state() == panel.control.ProcessState.NotRunning
    with pytest.raises(ValueError, match="unsupported"):
        panel.send("arbitrary-shell-text")
    panel.shutdown()
    panel.close()
