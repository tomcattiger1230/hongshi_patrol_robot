#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /home/hs/roboracer_ws/install/setup.bash
set -u
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:${PYTHONPATH:-}"
export PYTHONUNBUFFERED=1
export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec /usr/bin/python3 "$(dirname "$0")/spatial_stream.py"
