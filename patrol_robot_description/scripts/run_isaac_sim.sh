#!/usr/bin/env bash

set -euo pipefail

readonly PACKAGE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
readonly ISAAC_ROOT="${ISAAC_SIM_ROOT:-${HOME}/isaacsim}"
readonly ISAAC_ROS_DISTRO="${ISAAC_ROS_DISTRO:-jazzy}"
readonly INTERNAL_ROS_ROOT="${ISAAC_ROOT}/exts/isaacsim.ros2.core/${ISAAC_ROS_DISTRO}"
readonly ROS_SETUP_DEFAULT="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
ROS_SETUP="${ROS_SETUP:-${ROS_SETUP_DEFAULT}}"
# setup.zsh uses zsh-only syntax and cannot be sourced by bash. Normalize it
# to the bash equivalent from the same ROS prefix.
if [[ "${ROS_SETUP}" == *.zsh ]]; then
  if [[ -f "${ROS_SETUP%.zsh}.bash" ]]; then
    ROS_SETUP="${ROS_SETUP%.zsh}.bash"
  else
    echo "error: ROS_SETUP points to a zsh setup file without a bash equivalent: ${ROS_SETUP}" >&2
    exit 2
  fi
fi
readonly ROS_SETUP="${ROS_SETUP}"

if [[ ! -x "${ISAAC_ROOT}/python.sh" ]]; then
  echo "error: Isaac Sim python launcher not found: ${ISAAC_ROOT}/python.sh" >&2
  exit 2
fi
if [[ ! -d "${INTERNAL_ROS_ROOT}/rclpy" || ! -d "${INTERNAL_ROS_ROOT}/lib" ]]; then
  echo "error: Isaac Sim internal ROS libraries not found: ${INTERNAL_ROS_ROOT}" >&2
  exit 2
fi
if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "error: ROS setup file not found: ${ROS_SETUP}" >&2
  exit 2
fi

# Generate the URDF in the host ROS shell before configuring Isaac's bundled
# Python process.
URDF_DIR="$(mktemp -d "${TMPDIR:-/tmp}/patrol_robot_isaac_urdf.XXXXXX")"
readonly URDF_PATH="${URDF_DIR}/patrol_robot.urdf"
cleanup() {
  rm -f -- "${URDF_PATH}"
  rmdir -- "${URDF_DIR}"
}
trap cleanup EXIT
(
  set +u
  unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
  source "${ROS_SETUP}"
  xacro "${PACKAGE_ROOT}/urdf/patrol_robot.urdf.xacro" >"${URDF_PATH}"
)

# Keep host ROS Python packages out of Isaac's process and use the bundled
# Jazzy bridge libraries. Basic ROS interfaces communicate with the external
# SLAM and Nav2 processes over DDS.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
export ROS_DISTRO="${ISAAC_ROS_DISTRO}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONPATH="${INTERNAL_ROS_ROOT}/rclpy"
export LD_LIBRARY_PATH="${INTERNAL_ROS_ROOT}/lib"

"${ISAAC_ROOT}/python.sh" \
  "${PACKAGE_ROOT}/isaac_sim/patrol_robot_isaac_sim.py" \
  --urdf-path "${URDF_PATH}" "$@"
