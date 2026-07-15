"""Helpers re-exported for backward compatibility with earlier launch files.

This module is kept as a thin shim so older ``from mobile_platform.launch
import ...`` imports keep working when launched outside a built workspace.
The canonical implementation now lives inline in each launch file (see
``robot320_ros2.launch.py``) so ament_python does not have to import any
extra Python module just to launch ROS 2.
"""

from __future__ import annotations

from pathlib import Path


def resolve_executable(package: str, script_name: str, fallback: str) -> str:
    """Return ``script_name`` unchanged when ament_index is unavailable.

    The rich implementation lives inside each launch file; we keep this
    fallback here so older import sites still see a callable symbol.
    """
    _ = (Path, package)  # silence unused-arg linters on this re-export only
    return fallback
