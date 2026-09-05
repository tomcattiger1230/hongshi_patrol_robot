#!/usr/bin/env bash
# Build non-ROS native dependencies into a workspace-local prefix.

set -euo pipefail

readonly REPOSITORY_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "$(basename "$(dirname "${REPOSITORY_ROOT}")")" == "src" ]]; then
  readonly WORKSPACE_ROOT="$(cd "${REPOSITORY_ROOT}/../.." && pwd)"
else
  readonly WORKSPACE_ROOT="$(cd "${REPOSITORY_ROOT}/.." && pwd)"
fi
readonly NATIVE_WORKSPACE="${ROBOT320_NATIVE_WS:-${WORKSPACE_ROOT}/.ros-deps/lyrical/native_ws}"
readonly NATIVE_INSTALL="${NATIVE_WORKSPACE}/install"

native_proxy="${ROBOT320_APT_PROXY:-}"
if [[ -z "${native_proxy}" ]] && curl -fsSI --max-time 2 \
  -x http://127.0.0.1:7897 https://github.com >/dev/null 2>&1; then
  native_proxy="http://127.0.0.1:7897"
fi
if [[ -n "${native_proxy}" ]]; then
  export http_proxy="${native_proxy}" https_proxy="${native_proxy}"
fi

mkdir -p "${NATIVE_WORKSPACE}/src" "${NATIVE_WORKSPACE}/build" "${NATIVE_INSTALL}"
touch "${WORKSPACE_ROOT}/.ros-deps/COLCON_IGNORE"
vcs import "${NATIVE_WORKSPACE}/src" \
  < "${REPOSITORY_ROOT}/dependencies/lyrical_native.repos"

cmake \
  -S "${NATIVE_WORKSPACE}/src/Livox-SDK2" \
  -B "${NATIVE_WORKSPACE}/build/Livox-SDK2" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${NATIVE_INSTALL}" \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DCMAKE_CXX_FLAGS="-include cstdint"
cmake --build "${NATIVE_WORKSPACE}/build/Livox-SDK2" --parallel
cmake --install "${NATIVE_WORKSPACE}/build/Livox-SDK2"

echo "Livox SDK2 installed in ${NATIVE_INSTALL}"
