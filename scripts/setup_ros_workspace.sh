#!/usr/bin/env bash
# Configure and build the Robot320 colcon workspace for the installed ROS distro.

set -euo pipefail

readonly REPOSITORY_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "$(basename "$(dirname "${REPOSITORY_ROOT}")")" == "src" ]]; then
  WORKSPACE_ROOT="$(cd "${REPOSITORY_ROOT}/../.." && pwd)"
else
  WORKSPACE_ROOT="$(cd "${REPOSITORY_ROOT}/.." && pwd)"
fi
readonly WORKSPACE_ROOT

ros_distro="${1:-${ROS_DISTRO:-}}"
if [[ -z "${ros_distro}" ]]; then
  mapfile -t installed_distros < <(find /opt/ros -mindepth 1 -maxdepth 1 -type d -printf '%f\n' 2>/dev/null | sort)
  if ((${#installed_distros[@]} != 1)); then
    echo "error: pass the ROS distro explicitly (for example: $0 lyrical)" >&2
    exit 2
  fi
  ros_distro="${installed_distros[0]}"
fi

readonly ROS_SETUP="/opt/ros/${ros_distro}/setup.bash"
if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "error: ROS setup file not found: ${ROS_SETUP}" >&2
  exit 2
fi
# ROS-generated setup files may read unset tracing variables. Temporarily
# disable nounset while sourcing the underlay, then restore strict mode.
set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
set -u

case "${ros_distro}" in
  lyrical)
    native_workspace="${ROBOT320_NATIVE_WS:-${WORKSPACE_ROOT}/.ros-deps/lyrical/native_ws}"
    "${REPOSITORY_ROOT}/scripts/setup_lyrical_native.sh"
    export CMAKE_PREFIX_PATH="${native_workspace}/install${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
    export LD_LIBRARY_PATH="${native_workspace}/install/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
    navigation_workspace="${ROBOT320_NAV_WS:-${WORKSPACE_ROOT}/.ros-deps/lyrical/navigation_ws}"
    "${REPOSITORY_ROOT}/scripts/setup_lyrical_navigation.sh" "${navigation_workspace}"
    # shellcheck disable=SC1090
    source "${navigation_workspace}/install/setup.bash"
    ;;
  jazzy)
    # Jazzy provides the navigation stack as binary packages. rosdep resolves
    # the exact package set from package.xml instead of sharing Lyrical builds.
    sudo apt-get update
    rosdep install --from-paths "${REPOSITORY_ROOT}" --ignore-src --rosdistro jazzy -r -y
    ;;
  *)
    echo "error: unsupported ROS distro '${ros_distro}'; add dependencies/${ros_distro}_*.repos first" >&2
    exit 2
    ;;
esac

cd "${WORKSPACE_ROOT}"
ROBOT320_CMAKE_CLEAN_CACHE=1 "${REPOSITORY_ROOT}/build.sh"

echo
echo "Robot320 workspace ready for ROS 2 ${ros_distro}."
echo "Source it with:"
echo "  source ${REPOSITORY_ROOT}/scripts/source_robot320.sh ${ros_distro}"
