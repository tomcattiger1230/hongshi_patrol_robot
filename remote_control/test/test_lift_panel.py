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


def test_nuc_lift_mode_uses_local_fixed_program_without_tunnel(monkeypatch):
    _app = QApplication.instance() or QApplication([])
    monkeypatch.setenv("ROBOT_LIFT_LOCAL_COMMAND", "/tmp/fixed-lift-control.py")
    panel = LiftPanel(auto_tunnel=False, transport="local")
    panel._start_control()
    assert panel.control.program() == "/usr/bin/python3"
    assert panel.control.arguments() == ["/tmp/fixed-lift-control.py", "--stream"]
    assert panel.tunnel.state() == panel.tunnel.ProcessState.NotRunning
    panel.control.kill()
    panel.control.waitForFinished(1000)
    panel.shutdown()
    panel.close()
