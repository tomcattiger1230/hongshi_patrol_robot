#!/usr/bin/env bash

set -euo pipefail

readonly REPOSITORY_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "$(basename "$(dirname "${REPOSITORY_ROOT}")")" == "src" ]]; then
  readonly BUILD_ROOT="$(cd "${REPOSITORY_ROOT}/../.." && pwd)"
else
  readonly BUILD_ROOT="${REPOSITORY_ROOT}"
fi
readonly ROS_SYSTEM_PYTHON_PATH="/usr/lib/python3/dist-packages"
if [[ "$(uname -s)" == "Linux" && -x /usr/bin/python3 ]]; then
  readonly ROS_BUILD_PYTHON="${ROS_BUILD_PYTHON:-/usr/bin/python3}"
else
  readonly ROS_BUILD_PYTHON="${ROS_BUILD_PYTHON:-$(command -v python3)}"
fi
readonly DEFAULT_PACKAGES=(
  robot320_interfaces
  livox_ros_driver2
  mid360_preprocess
  mobile_platform
  robot320_localization_bringup
  remote_control
  patrol_robot_description
)

if ! command -v colcon >/dev/null 2>&1; then
  echo "error: colcon is not available; install/source ROS 2 first" >&2
  exit 127
fi

if [[ -z "${AMENT_PREFIX_PATH:-}" ]]; then
  echo "warning: AMENT_PREFIX_PATH is empty; did you source /opt/ros/<distro>/setup.bash?" >&2
fi

# ROS 2's IDL generator imports the system-installed lark module. Make
# it visible when the active Python environment does not include dist-packages.
if [[ -d "${ROS_SYSTEM_PYTHON_PATH}" ]]; then
  export PYTHONPATH="${ROS_SYSTEM_PYTHON_PATH}${PYTHONPATH:+:${PYTHONPATH}}"
fi

cd "${BUILD_ROOT}"

if (($# == 0)); then
  build_options=(--symlink-install)
  if [[ "${ROBOT320_CMAKE_CLEAN_CACHE:-0}" == "1" ]]; then
    build_options+=(--cmake-clean-cache)
  fi
  echo "Building Robot320 packages: ${DEFAULT_PACKAGES[*]}"
  exec colcon build \
    "${build_options[@]}" \
    --packages-select "${DEFAULT_PACKAGES[@]}" \
    --cmake-args \
      "-DPython3_EXECUTABLE=${ROS_BUILD_PYTHON}" \
      "-DPYTHON_EXECUTABLE=${ROS_BUILD_PYTHON}"
fi

echo "Running colcon build with custom arguments: $*"
exec colcon build --symlink-install "$@"
