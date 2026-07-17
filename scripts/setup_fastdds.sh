#!/usr/bin/env bash

set -euo pipefail

readonly REPOSITORY_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
venv_path="${UV_PROJECT_ENVIRONMENT:-${REPOSITORY_ROOT}/.venv}"
if [[ "${venv_path}" != /* ]]; then
  venv_path="${REPOSITORY_ROOT}/${venv_path}"
fi
readonly VENV_PATH="${venv_path}"
readonly FASTDDS_PREFIX="${FASTDDS_PREFIX:-${REPOSITORY_ROOT}/../Fast-DDS/install}"
readonly FASTDDS_PYTHON_SOURCE="${FASTDDS_PYTHON_SOURCE:-${REPOSITORY_ROOT}/../Fast-DDS-python/fastdds_python}"
readonly BUILD_DIR="${FASTDDS_PYTHON_BUILD_DIR:-${REPOSITORY_ROOT}/build/fastdds_python}"

if [[ ! -x "${VENV_PATH}/bin/python" ]]; then
  echo "error: uv environment is missing at ${VENV_PATH}; run ./scripts/uv_setup.sh desktop first" >&2
  exit 2
fi
for command_name in cmake swig; do
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "error: ${command_name} is required to build Fast-DDS-python" >&2
    exit 127
  fi
done
prefix_paths=("${FASTDDS_PREFIX}")
if [[ -d "${FASTDDS_PREFIX}/fastdds" ]]; then
  prefix_paths=(
    "${FASTDDS_PREFIX}/fastdds"
    "${FASTDDS_PREFIX}/fastcdr"
    "${FASTDDS_PREFIX}/foonathan_memory_vendor"
  )
fi
cmake_prefix_path="$(IFS=';'; echo "${prefix_paths[*]}")"

if [[ "${FASTDDS_FORCE_BUILD:-0}" == "1" ]] || \
   ! "${VENV_PATH}/bin/python" -c 'import fastdds' >/dev/null 2>&1; then
  if [[ ! -f "${FASTDDS_PYTHON_SOURCE}/CMakeLists.txt" ]]; then
    echo "error: Fast-DDS-python source not found: ${FASTDDS_PYTHON_SOURCE}" >&2
    echo "set FASTDDS_PYTHON_SOURCE to the fastdds_python source directory" >&2
    exit 2
  fi
  echo "Building Fast-DDS-python into ${VENV_PATH}"
  cmake -S "${FASTDDS_PYTHON_SOURCE}" -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="${cmake_prefix_path}" \
    -DCMAKE_INSTALL_PREFIX="${VENV_PATH}" \
    -DPython3_EXECUTABLE="${VENV_PATH}/bin/python" \
    -DBUILD_TESTING=OFF
  cmake --build "${BUILD_DIR}" --parallel
  cmake --install "${BUILD_DIR}"

  if [[ "$(uname -s)" == "Darwin" ]]; then
    module_path="$(find "${VENV_PATH}" -path '*/site-packages/fastdds/_fastdds_python.so' -print -quit)"
    if [[ -n "${module_path}" ]]; then
      for library_path in "${prefix_paths[@]}"; do
        install_name_tool -add_rpath "${library_path}/lib" "${module_path}" 2>/dev/null || true
      done
    fi
  fi
else
  echo "Fast-DDS-python is already available in ${VENV_PATH}"
fi

env FASTDDS_PREFIX="${FASTDDS_PREFIX}" \
  PYTHON_BIN="${VENV_PATH}/bin/python" \
  "${REPOSITORY_ROOT}/robot320_interfaces/scripts/generate_fastdds_types.sh"

PYTHONPATH="${REPOSITORY_ROOT}/robot320_interfaces/generated/Robot320String/build${PYTHONPATH:+:${PYTHONPATH}}" \
  "${VENV_PATH}/bin/python" -c \
  'import Robot320String, fastdds; print("Fast DDS ready:", fastdds.__file__)'
