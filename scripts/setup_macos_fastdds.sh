#!/usr/bin/env bash
# Build the standalone Fast DDS Python runtime on Apple Silicon.

set -euo pipefail

if [[ "$(uname -s)" != "Darwin" ]]; then
  echo "error: this setup entry point is for macOS" >&2
  exit 2
fi

readonly REPOSITORY_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
readonly FASTDDS_WORKSPACE="${FASTDDS_WORKSPACE:-${HOME}/Develop/fastdds-python}"
readonly SWIG_VERSION="4.1.1"
readonly SWIG_PREFIX="${FASTDDS_WORKSPACE}/toolchain/swig-${SWIG_VERSION}-install"

brew install cmake asio tinyxml2 openssl wget openjdk@21 autoconf automake pcre2

readonly JAVA_HOME="$(brew --prefix openjdk@21)/libexec/openjdk.jdk/Contents/Home"
export JAVA_HOME
export PATH="${SWIG_PREFIX}/bin:$(brew --prefix openjdk@21)/bin:${PATH}"
export CMAKE_PREFIX_PATH="$(brew --prefix)"
export OPENSSL_ROOT_DIR="$(brew --prefix openssl@3)"

"${REPOSITORY_ROOT}/scripts/uv_setup.sh" desktop --python 3.12

mkdir -p "${FASTDDS_WORKSPACE}/src" "${FASTDDS_WORKSPACE}/toolchain"
uvx --with 'setuptools<81' --from vcstool vcs import "${FASTDDS_WORKSPACE}/src" \
  < "${REPOSITORY_ROOT}/dependencies/macos_fastdds.repos"

if [[ ! -x "${SWIG_PREFIX}/bin/swig" ]]; then
  swig_archive="${FASTDDS_WORKSPACE}/toolchain/swig-${SWIG_VERSION}.tar.gz"
  curl -fL "https://github.com/swig/swig/archive/refs/tags/v${SWIG_VERSION}.tar.gz" \
    -o "${swig_archive}"
  tar -xzf "${swig_archive}" -C "${FASTDDS_WORKSPACE}/toolchain"
  (
    cd "${FASTDDS_WORKSPACE}/toolchain/swig-${SWIG_VERSION}"
    ./autogen.sh
    ./configure --prefix="${SWIG_PREFIX}" --with-pcre-prefix="$(brew --prefix)"
    make --jobs "$(sysctl -n hw.logicalcpu)"
    make install
  )
fi

fastdds_python_source="${FASTDDS_WORKSPACE}/src/fastdds_python"
fastdds_python_patch="${REPOSITORY_ROOT}/dependencies/patches/fastdds-python-macos-arm64.patch"
if git -C "${fastdds_python_source}" apply --unidiff-zero --check \
  "${fastdds_python_patch}" 2>/dev/null; then
  git -C "${fastdds_python_source}" apply --unidiff-zero "${fastdds_python_patch}"
fi

(
  cd "${FASTDDS_WORKSPACE}"
  uvx --with 'setuptools<81' --from colcon-common-extensions colcon build \
    --packages-up-to fastdds_python \
    --cmake-args \
      -DPython3_EXECUTABLE="${REPOSITORY_ROOT}/.venv/bin/python" \
      -DBUILD_TESTING=OFF \
      -DCMAKE_BUILD_TYPE=Release
)

FASTDDS_PREFIX="${FASTDDS_WORKSPACE}/install" \
FASTDDS_PYTHON_SOURCE="${fastdds_python_source}/fastdds_python" \
FASTDDSGEN_SOURCE="${FASTDDS_WORKSPACE}/src/fastddsgen" \
FASTDDS_PYTHON_BUILD_DIR="${FASTDDS_WORKSPACE}/build/venv_binding" \
  "${REPOSITORY_ROOT}/scripts/setup_fastdds.sh"

echo
echo "Fast DDS for macOS is ready in ${FASTDDS_WORKSPACE}."
echo "Before LAN use: source ${REPOSITORY_ROOT}/scripts/source_dds_lan.sh <ubuntu-ip>"
