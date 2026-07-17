#!/usr/bin/env bash

set -euo pipefail

readonly REPOSITORY_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
readonly PROFILE="${1:-}"

usage() {
  echo "usage: $0 <desktop|nuc> [--python PATH] [--dev]" >&2
}

if [[ "${PROFILE}" != "desktop" && "${PROFILE}" != "nuc" ]]; then
  usage
  exit 2
fi
if ! command -v uv >/dev/null 2>&1; then
  echo "error: uv is not installed; see https://docs.astral.sh/uv/getting-started/installation/" >&2
  exit 127
fi
shift

python="${UV_PYTHON:-}"
venv_path="${UV_PROJECT_ENVIRONMENT:-.venv}"
with_dev=false
while (($#)); do
  case "$1" in
    --python)
      if (($# < 2)); then
        usage
        exit 2
      fi
      python="$2"
      shift 2
      ;;
    --dev)
      with_dev=true
      shift
      ;;
    *)
      usage
      exit 2
      ;;
  esac
done

venv_args=(--clear)
if [[ "${PROFILE}" == "nuc" ]]; then
  python="${python:-/usr/bin/python3}"
  venv_args+=(--system-site-packages)
else
  python="${python:-python3}"
fi

cd "${REPOSITORY_ROOT}"
echo "Creating ${venv_path} for ${PROFILE} with ${python}"
uv venv "${venv_args[@]}" --python "${python}" "${venv_path}"
sync_args=(--locked --inexact --no-default-groups --extra "${PROFILE}")
if [[ "${with_dev}" == true ]]; then
  sync_args+=(--group dev)
fi
uv sync "${sync_args[@]}"

echo
echo "Environment ready. Run commands with:"
echo "  ./scripts/uv_run.sh ${PROFILE} <command> [args...]"
