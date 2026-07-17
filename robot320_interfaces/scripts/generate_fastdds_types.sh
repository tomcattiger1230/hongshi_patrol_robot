#!/usr/bin/env bash

set -euo pipefail

readonly PACKAGE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
readonly IDL_FILE="${PACKAGE_ROOT}/robot320_interfaces/dds/Robot320Dds.idl"
readonly OUTPUT_DIR="${1:-${PACKAGE_ROOT}/generated/Robot320Dds}"

if ! command -v fastddsgen >/dev/null 2>&1; then
  echo "error: fastddsgen is not available in PATH" >&2
  exit 127
fi
if ! command -v cmake >/dev/null 2>&1; then
  echo "error: cmake is not available in PATH" >&2
  exit 127
fi

mkdir -p "${OUTPUT_DIR}"
cd "${OUTPUT_DIR}"
fastddsgen -python -replace "${IDL_FILE}"
cmake -S . -B build
cmake --build build --parallel

echo "Generated Robot320Dds Python bindings in ${OUTPUT_DIR}/build"
echo "Add that directory to PYTHONPATH on the target machine."
