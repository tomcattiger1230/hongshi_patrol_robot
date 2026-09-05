#!/usr/bin/env sh
# Source the system ROS, distro-specific dependency overlay, and Robot320 workspace.

robot320_patrol_ws="${ROBOT320_PATROL_WS:-${HOME}/Develop/ROS_ws/patrol_robot}"
robot320_ros_distro="${1:-${ROS_DISTRO:-}}"

if [ -z "${robot320_ros_distro}" ]; then
  robot320_distro_count=0
  for robot320_candidate in /opt/ros/*; do
    [ -d "${robot320_candidate}" ] || continue
    robot320_ros_distro="$(basename "${robot320_candidate}")"
    robot320_distro_count=$((robot320_distro_count + 1))
  done
  if [ "${robot320_distro_count}" -ne 1 ]; then
    echo "error: pass the ROS distro explicitly: source scripts/source_robot320.sh <distro>" >&2
    return 2
  fi
fi

robot320_system_setup="/opt/ros/${robot320_ros_distro}/setup.sh"
if [ ! -f "${robot320_system_setup}" ]; then
  echo "error: ROS setup file not found: ${robot320_system_setup}" >&2
  return 2
fi
# shellcheck disable=SC1090
. "${robot320_system_setup}"

robot320_native_install="${robot320_patrol_ws}/.ros-deps/${robot320_ros_distro}/native_ws/install"
if [ -d "${robot320_native_install}" ]; then
  export CMAKE_PREFIX_PATH="${robot320_native_install}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
  export LD_LIBRARY_PATH="${robot320_native_install}/lib:${robot320_native_install}/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

robot320_dependency_setup="${robot320_patrol_ws}/.ros-deps/${robot320_ros_distro}/navigation_ws/install/setup.sh"
if [ -f "${robot320_dependency_setup}" ]; then
  # shellcheck disable=SC1090
  . "${robot320_dependency_setup}"
fi

robot320_project_setup="${robot320_patrol_ws}/install/setup.sh"
if [ ! -f "${robot320_project_setup}" ]; then
  echo "error: project setup file not found: ${robot320_project_setup}" >&2
  return 2
fi
# shellcheck disable=SC1090
. "${robot320_project_setup}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}"
export RMW_IMPLEMENTATION="${ROBOT320_RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

echo "Robot320 ready: ROS=${robot320_ros_distro} domain=${ROS_DOMAIN_ID} RMW=${RMW_IMPLEMENTATION}"

unset robot320_candidate robot320_dependency_setup robot320_distro_count robot320_native_install
unset robot320_patrol_ws robot320_project_setup robot320_ros_distro robot320_system_setup
