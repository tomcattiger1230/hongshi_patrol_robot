#!/usr/bin/env sh
# Source the complete ROS 2 Lyrical simulation overlay in bash or zsh.

case "$0" in
  *source_lyrical_sim.sh)
    echo "error: run this with 'source scripts/source_lyrical_sim.sh'" >&2
    exit 2
    ;;
esac

robot320_nav_ws="${ROBOT320_NAV_WS:-${HOME}/Develop/ROS2_ws/navigation_lyrical_ws}"
robot320_patrol_ws="${ROBOT320_PATROL_WS:-${HOME}/Develop/ROS2_ws/patrol_ws}"

for robot320_setup_file in \
  /opt/ros/lyrical/setup.sh \
  "${robot320_nav_ws}/install/setup.sh" \
  "${robot320_patrol_ws}/install/setup.sh"
do
  if [ ! -f "${robot320_setup_file}" ]; then
    echo "error: ROS setup file not found: ${robot320_setup_file}" >&2
    return 2
  fi
  # shellcheck disable=SC1090
  . "${robot320_setup_file}"
done

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if ! ros2 pkg prefix slam_toolbox >/dev/null 2>&1; then
  echo "error: slam_toolbox is still absent from the ROS package index" >&2
  return 2
fi

echo "ROS 2 Lyrical simulation environment ready (slam_toolbox found)."
