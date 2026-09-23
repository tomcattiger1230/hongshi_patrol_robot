#!/usr/bin/env bash

set -euo pipefail

readonly REPOSITORY_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
readonly ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "error: ROS 2 setup not found: ${ROS_SETUP}" >&2
  exit 2
fi
if ! command -v uv >/dev/null 2>&1; then
  echo "error: uv is not installed" >&2
  exit 127
fi

set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
if [[ -f "${REPOSITORY_ROOT}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${REPOSITORY_ROOT}/install/setup.bash"
fi
set -u

cd "${REPOSITORY_ROOT}"
exec uv run --locked --no-default-groups --extra nuc -- robot320_nuc_gui "$@"
