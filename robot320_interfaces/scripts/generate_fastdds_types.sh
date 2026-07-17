#!/usr/bin/env bash

set -euo pipefail

readonly PACKAGE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
readonly IDL_FILE="${PACKAGE_ROOT}/robot320_interfaces/dds/Robot320Dds.idl"
readonly OUTPUT_DIR="${1:-${PACKAGE_ROOT}/generated/Robot320Dds}"
readonly REPOSITORY_ROOT="$(cd -- "${PACKAGE_ROOT}/.." && pwd)"
readonly FASTDDS_PREFIX="${FASTDDS_PREFIX:-${REPOSITORY_ROOT}/../Fast-DDS/install}"
readonly PYTHON_BIN="${PYTHON_BIN:-${REPOSITORY_ROOT}/.venv/bin/python}"

if ! command -v cmake >/dev/null 2>&1; then
  echo "error: cmake is not available in PATH" >&2
  exit 127
fi

run_fastddsgen() {
  if [[ -n "${FASTDDSGEN:-}" ]]; then
    "${FASTDDSGEN}" "$@"
  elif command -v fastddsgen >/dev/null 2>&1; then
    fastddsgen "$@"
  else
    local source_root="${FASTDDSGEN_SOURCE:-${REPOSITORY_ROOT}/../Fast-DDS/src/fastddsgen}"
    local generator_jar="${source_root}/build/libs/fastddsgen.jar"
    if [[ ! -f "${generator_jar}" ]]; then
      if [[ ! -x "${source_root}/gradlew" ]]; then
        echo "error: fastddsgen is unavailable; set FASTDDSGEN or FASTDDSGEN_SOURCE" >&2
        return 127
      fi
      (cd "${source_root}" && ./gradlew assemble)
    fi
    java -jar "${generator_jar}" "$@"
  fi
}

prefix_paths=("${FASTDDS_PREFIX}")
if [[ -d "${FASTDDS_PREFIX}/fastdds" ]]; then
  prefix_paths=(
    "${FASTDDS_PREFIX}/fastdds"
    "${FASTDDS_PREFIX}/fastcdr"
    "${FASTDDS_PREFIX}/foonathan_memory_vendor"
  )
fi
cmake_prefix_path="$(IFS=';'; echo "${prefix_paths[*]}")"

mkdir -p "${OUTPUT_DIR}"
cd "${OUTPUT_DIR}"
run_fastddsgen -python -replace "${IDL_FILE}"
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="${cmake_prefix_path}" \
  -DPython3_EXECUTABLE="${PYTHON_BIN}"
cmake --build build --parallel

if [[ "$(uname -s)" == "Darwin" ]]; then
  module_path="$(find build -maxdepth 2 -name '_Robot320DdsWrapper.so' -print -quit)"
  if [[ -n "${module_path}" ]]; then
    for library_path in "${prefix_paths[@]}"; do
      install_name_tool -add_rpath "${library_path}/lib" "${module_path}" 2>/dev/null || true
    done
  fi
fi

echo "Generated Robot320Dds Python bindings in ${OUTPUT_DIR}/build"
echo "Add that directory to PYTHONPATH on the target machine."
