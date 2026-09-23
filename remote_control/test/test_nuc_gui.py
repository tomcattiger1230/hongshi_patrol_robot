from remote_control.nuc_gui import inhibitor_command


def test_nuc_launcher_wraps_gui_with_both_linux_inhibitors():
    programs = {
        "gnome-session-inhibit": "/usr/bin/gnome-session-inhibit",
        "systemd-inhibit": "/usr/bin/systemd-inhibit",
    }
    command, active = inhibitor_command(
        ["--domain-id", "20"], executable_lookup=programs.get,
    )
    assert command[0] == "/usr/bin/systemd-inhibit"
    assert "/usr/bin/gnome-session-inhibit" in command
    assert "remote_control.gui" in command
    assert ["GNOME idle/suspend", "systemd idle/sleep"] == active


def test_nuc_launcher_still_runs_gui_when_inhibitors_are_unavailable():
    command, active = inhibitor_command([], executable_lookup=lambda _name: None)
    assert command[1:4] == ["-m", "remote_control.gui", "--deployment"]
    assert active == []
