#!/usr/bin/env bash

set -euo pipefail

readonly PACKAGE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
readonly ISAAC_ROOT="${ISAAC_SIM_ROOT:-${HOME}/isaacsim}"
readonly ISAAC_ROS_DISTRO="${ISAAC_ROS_DISTRO:-jazzy}"
readonly INTERNAL_ROS_ROOT="${ISAAC_ROOT}/exts/isaacsim.ros2.core/${ISAAC_ROS_DISTRO}"

if [[ ! -x "${ISAAC_ROOT}/python.sh" ]]; then
  echo "error: Isaac Sim python launcher not found: ${ISAAC_ROOT}/python.sh" >&2
  exit 2
fi
if [[ ! -d "${INTERNAL_ROS_ROOT}/rclpy" || ! -d "${INTERNAL_ROS_ROOT}/lib" ]]; then
  echo "error: Isaac Sim internal ROS libraries not found: ${INTERNAL_ROS_ROOT}" >&2
  exit 2
fi

# Isaac Sim 6 uses Python 3.12, while ROS 2 Lyrical on Ubuntu 26.04 uses
# Python 3.14. Keep Lyrical out of this process and use Isaac's Python 3.12
# Jazzy bridge libraries. Basic ROS interfaces communicate with the external
# Lyrical SLAM/Nav2 processes over Cyclone DDS.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
export ROS_DISTRO="${ISAAC_ROS_DISTRO}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONPATH="${INTERNAL_ROS_ROOT}/rclpy"
export LD_LIBRARY_PATH="${INTERNAL_ROS_ROOT}/lib"

exec "${ISAAC_ROOT}/python.sh" \
  "${PACKAGE_ROOT}/isaac_sim/patrol_robot_isaac_sim.py" "$@"
