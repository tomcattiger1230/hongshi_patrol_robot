#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /home/hs/roboracer_ws/install/setup.bash
set -u
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:${PYTHONPATH:-}"
export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
map_file="$(dirname "$0")/maps/active.yaml"
test -r "$map_file" || { echo "Localization map is missing: $map_file" >&2; exit 2; }
export ROBOT320_LOCALIZATION_MAP="$map_file"
nodes=$(ros2 node list --no-daemon --spin-time 2)
if [[ "$nodes" == *cartographer* || "$nodes" == *amcl* || "$nodes" == *ekf_filter_node* || "$nodes" == *robot_state_publisher* ]]; then
    echo 'Existing SLAM/localization/TF nodes detected; refusing duplicate localization startup.' >&2
    exit 2
fi
exec ros2 launch "$(dirname "$0")/spatial_localization.launch.py"
