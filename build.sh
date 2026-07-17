#!/usr/bin/env bash

set -euo pipefail

readonly REPOSITORY_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly ROS_SYSTEM_PYTHON_PATH="/usr/lib/python3/dist-packages"
readonly DEFAULT_PACKAGES=(
  livox_ros_driver2
  mid360_preprocess
  mobile_platform
  robot320_localization_bringup
  remote_control
)

if ! command -v colcon >/dev/null 2>&1; then
  echo "error: colcon is not available; install/source ROS 2 first" >&2
  exit 127
fi

if [[ -z "${AMENT_PREFIX_PATH:-}" ]]; then
  echo "warning: AMENT_PREFIX_PATH is empty; did you source /opt/ros/jazzy/setup.bash?" >&2
fi

# ROS 2 Jazzy's IDL generator imports the system-installed lark module. Make
# it visible when the active Python environment does not include dist-packages.
if [[ -d "${ROS_SYSTEM_PYTHON_PATH}" ]]; then
  export PYTHONPATH="${ROS_SYSTEM_PYTHON_PATH}${PYTHONPATH:+:${PYTHONPATH}}"
fi

cd "${REPOSITORY_ROOT}"

if (($# == 0)); then
  echo "Building Robot320 packages: ${DEFAULT_PACKAGES[*]}"
  exec colcon build --symlink-install --packages-select "${DEFAULT_PACKAGES[@]}"
fi

echo "Running colcon build with custom arguments: $*"
exec colcon build --symlink-install "$@"
