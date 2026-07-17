#!/usr/bin/env bash

set -euo pipefail

readonly REPOSITORY_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
readonly PROFILE="${1:-}"

source_overlay() {
  # ROS 2/ament setup files are not nounset-safe and may probe optional vars.
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

usage() {
  echo "usage: $0 <desktop|nuc> [--dev] [--] <command> [args...]" >&2
}

if [[ "${PROFILE}" != "desktop" && "${PROFILE}" != "nuc" ]]; then
  usage
  exit 2
fi
shift
with_dev=false
if [[ "${1:-}" == "--dev" ]]; then
  with_dev=true
  shift
fi
if [[ "${1:-}" == "--" ]]; then
  shift
fi
if (($# == 0)); then
  usage
  exit 2
fi
if ! command -v uv >/dev/null 2>&1; then
  echo "error: uv is not installed" >&2
  exit 127
fi
venv_path="${UV_PROJECT_ENVIRONMENT:-${REPOSITORY_ROOT}/.venv}"
if [[ "${venv_path}" != /* ]]; then
  venv_path="${REPOSITORY_ROOT}/${venv_path}"
fi
if [[ ! -x "${venv_path}/bin/python" ]]; then
  echo "error: uv environment is missing at ${venv_path}; run uv_setup.sh first" >&2
  exit 2
fi

if [[ "${PROFILE}" == "nuc" || "$(uname -s)" == "Linux" ]]; then
  ros_setup="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
  if [[ "${PROFILE}" == "nuc" && ! -f "${ros_setup}" ]]; then
    echo "error: ROS 2 setup not found: ${ros_setup}" >&2
    exit 2
  fi
  if [[ -f "${ros_setup}" ]]; then
    source_overlay "${ros_setup}"
  fi
fi

if [[ -n "${FASTDDS_SETUP:-}" ]]; then
  if [[ ! -f "${FASTDDS_SETUP}" ]]; then
    echo "error: FASTDDS_SETUP does not exist: ${FASTDDS_SETUP}" >&2
    exit 2
  fi
  source_overlay "${FASTDDS_SETUP}"
fi

if [[ "${PROFILE}" == "nuc" ]]; then
  robot_setup="${ROBOT320_SETUP:-${REPOSITORY_ROOT}/install/setup.bash}"
  if [[ -f "${robot_setup}" ]]; then
    source_overlay "${robot_setup}"
  fi
fi

generated_types="${ROBOT320_DDS_TYPES:-${REPOSITORY_ROOT}/robot320_interfaces/generated/Robot320String/build}"
if [[ -d "${generated_types}" ]]; then
  export PYTHONPATH="${generated_types}${PYTHONPATH:+:${PYTHONPATH}}"
fi

cd "${REPOSITORY_ROOT}"
run_args=(--locked --no-default-groups --extra "${PROFILE}")
if [[ "${with_dev}" == true ]]; then
  run_args+=(--group dev)
fi
exec uv run "${run_args[@]}" -- "$@"
