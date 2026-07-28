#!/usr/bin/env bash
set -euo pipefail

if [[ "${ROS_DISTRO:-}" != "lyrical" ]]; then
  source /opt/ros/lyrical/setup.bash
fi

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
navigation_workspace="${1:-${HOME}/Develop/ROS2_ws/navigation_lyrical_ws}"

sudo apt-get update
sudo apt-get install -y \
  build-essential \
  cmake \
  git \
  libceres-dev \
  libdraco-dev \
  libgeographiclib-dev \
  libgraphicsmagick1-dev \
  libnanoflann-dev \
  libompl-dev \
  libqt6scxml6-dev \
  libzstd-dev \
  lm-sensors \
  python3-ntplib \
  python3-pyproj \
  python3-rosdep \
  python3-typeshed \
  python3-vcstool \
  python3-zmq \
  ros-lyrical-rmw-cyclonedds-cpp \
  zlib1g-dev

mkdir -p "${navigation_workspace}/src"
vcs import "${navigation_workspace}/src" \
  < "${repo_root}/dependencies/lyrical_navigation.repos"

slam_toolbox_patch="${repo_root}/dependencies/patches/slam_toolbox-cmake-3.27.patch"
if git -C "${navigation_workspace}/src/slam_toolbox" apply \
  --check "${slam_toolbox_patch}" 2>/dev/null; then
  git -C "${navigation_workspace}/src/slam_toolbox" apply \
    "${slam_toolbox_patch}"
fi

rosdep install \
  --from-paths "${navigation_workspace}/src" \
  --ignore-src \
  --rosdistro lyrical \
  --skip-keys graphicsmagick-libmagick-dev-compat \
  -r -y

cd "${navigation_workspace}"
colcon build \
  --symlink-install \
  --packages-up-to \
    slam_toolbox \
    nav2_bringup \
    nav2_regulated_pure_pursuit_controller \
    nav2_smac_planner \
  --cmake-args -DBUILD_TESTING=OFF

echo "Source ${navigation_workspace}/install/setup.bash before building this project."
