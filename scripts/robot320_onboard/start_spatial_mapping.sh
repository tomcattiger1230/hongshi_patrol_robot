#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /home/hs/roboracer_ws/install/setup.bash
set -u
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:${PYTHONPATH:-}"
export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Refuse to create competing TF authorities or another SLAM session.
mapping_nodes=$(ros2 node list --no-daemon --spin-time 2)
if [[ "$mapping_nodes" == *cartographer* || "$mapping_nodes" == *ekf* || "$mapping_nodes" == *robot_state_publisher* ]]; then
    echo 'Existing SLAM/EKF/URDF nodes detected; refusing independent mapping startup.' >&2
    exit 2
fi
exec ros2 launch "$(dirname "$0")/spatial_mapping.launch.py"
