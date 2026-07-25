#!/usr/bin/env bash

set -euo pipefail

readonly PACKAGE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
readonly ISAAC_ROOT="${ISAAC_SIM_ROOT:-${HOME}/isaacsim}"
readonly ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

if [[ ! -x "${ISAAC_ROOT}/python.sh" ]]; then
  echo "error: Isaac Sim python launcher not found: ${ISAAC_ROOT}/python.sh" >&2
  exit 2
fi
if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "error: ROS 2 setup not found: ${ROS_SETUP}" >&2
  exit 2
fi

set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${ISAAC_ROOT}/setup_ros_env.sh"
set -u

exec "${ISAAC_ROOT}/python.sh" \
  "${PACKAGE_ROOT}/isaac_sim/patrol_robot_isaac_sim.py" "$@"
