"""NUC-local GUI launcher with desktop-idle and system-sleep inhibition."""

from __future__ import annotations

import os
import shutil
import sys


def inhibitor_command(arguments, executable_lookup=shutil.which):
    command = [sys.executable, "-m", "remote_control.gui", "--deployment", "nuc", *arguments]
    active = []
    gnome = executable_lookup("gnome-session-inhibit")
    if gnome:
        command = [
            gnome, "--inhibit", "idle:suspend", "--reason",
            "Robot320 NUC control GUI is active", *command,
        ]
        active.append("GNOME idle/suspend")
    systemd = executable_lookup("systemd-inhibit")
    if systemd:
        command = [
            systemd, "--what=idle:sleep", "--who=Robot320",
            "--why=Robot320 NUC control GUI is active", "--mode=block", *command,
        ]
        active.append("systemd idle/sleep")
    return command, active


def main(argv=None):
    arguments = list(sys.argv[1:] if argv is None else argv)
    no_inhibit = "--no-inhibit" in arguments
    arguments = [value for value in arguments if value != "--no-inhibit"]
    if sys.platform.startswith("linux") and not no_inhibit:
        command, active = inhibitor_command(arguments)
        if active:
            environment = os.environ.copy()
            environment["ROBOT320_INHIBITOR_ACTIVE"] = " + ".join(active)
            os.execvpe(command[0], command, environment)
    os.environ.setdefault("ROBOT320_INHIBITOR_ACTIVE", "none")
    from .gui import main as gui_main
    return gui_main(["--deployment", "nuc", *arguments])


if __name__ == "__main__":
    raise SystemExit(main())
