#!/usr/bin/env bash
set -euo pipefail

if [[ "${ROS_DISTRO:-}" != "lyrical" ]]; then
  set +u
  source /opt/ros/lyrical/setup.bash
  set -u
fi

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "$(basename "$(dirname "${repo_root}")")" == "src" ]]; then
  patrol_workspace="$(cd "${repo_root}/../.." && pwd)"
else
  patrol_workspace="$(cd "${repo_root}/.." && pwd)"
fi
navigation_workspace="${1:-${ROBOT320_NAV_WS:-${patrol_workspace}/.ros-deps/lyrical/navigation_ws}}"

apt_proxy="${ROBOT320_APT_PROXY:-}"
if [[ -z "${apt_proxy}" ]] && curl -fsSI --max-time 2 \
  -x http://127.0.0.1:7897 https://github.com >/dev/null 2>&1; then
  apt_proxy="http://127.0.0.1:7897"
fi
apt_options=()
if [[ -n "${apt_proxy}" ]]; then
  echo "Using APT proxy: ${apt_proxy}"
  apt_options+=(
    -o "Acquire::http::Proxy=${apt_proxy}"
    -o "Acquire::https::Proxy=${apt_proxy}"
  )
  export http_proxy="${apt_proxy}" https_proxy="${apt_proxy}"
fi

sudo apt-get "${apt_options[@]}" update
sudo apt-get "${apt_options[@]}" install -y \
  build-essential \
  cmake \
  git \
  libceres-dev \
  libdraco-dev \
  libgeographiclib-dev \
  libgraphicsmagick++1-dev \
  libgraphicsmagick1-dev \
  libnanoflann-dev \
  libomp-dev \
  libompl-dev \
  lcov \
  qt6-scxml-dev \
  qtbase5-dev \
  libzstd-dev \
  lm-sensors \
  python3-colcon-common-extensions \
  python3-importlib-metadata \
  python3-numpy \
  python3-ntplib \
  python3-pyproj \
  python3-rosdep \
  python3-typeshed \
  python3-vcstool \
  python3-zmq \
  ros-lyrical-ament-cmake-google-benchmark \
  ros-lyrical-diff-drive-controller \
  ros-lyrical-joint-state-broadcaster \
  ros-lyrical-launch-pytest \
  ros-lyrical-nav2-minimal-tb3-sim \
  ros-lyrical-nav2-minimal-tb4-sim \
  ros-lyrical-ompl \
  ros-lyrical-rmw-cyclonedds-cpp \
  ros-lyrical-rmw-fastrtps-cpp \
  ros-lyrical-gz-sim-vendor \
  ros-lyrical-pointcloud-to-laserscan \
  ros-lyrical-ros-gz-bridge \
  ros-lyrical-ros-gz-sim \
  ros-lyrical-test-msgs \
  ros-lyrical-xacro \
  zlib1g-dev

mkdir -p "${navigation_workspace}/src"
touch "${patrol_workspace}/.ros-deps/COLCON_IGNORE"
vcs import "${navigation_workspace}/src" \
  < "${repo_root}/dependencies/lyrical_navigation.repos"

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
for attempt in 1 2 3; do
  if rosdep update --rosdistro lyrical; then
    break
  fi
  if ((attempt == 3)); then
    echo "error: rosdep update failed after ${attempt} attempts" >&2
    exit 1
  fi
  echo "rosdep update failed (attempt ${attempt}/3); retrying..." >&2
  sleep 2
done

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
  --cmake-clean-cache \
  --packages-up-to \
    slam_toolbox \
    nav2_bringup \
    nav2_regulated_pure_pursuit_controller \
    nav2_smac_planner \
  --cmake-args \
    -DBUILD_TESTING=OFF \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3

echo "Source ${navigation_workspace}/install/setup.bash before building this project."
