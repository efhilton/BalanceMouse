#!/usr/bin/env bash
# Build and run the host (x86) unit tests with ASan + UBSan.
#
# Default: runs inside the same espressif/idf Docker image used for firmware builds, so the
# only host prerequisite is Docker (and Unity comes from that image's ESP-IDF).
#   host_test/run_host_tests.sh
# Use a local toolchain instead (needs cmake + a C++20 compiler; fetches Unity if IDF_PATH unset):
#   host_test/run_host_tests.sh --local
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IDF_IMAGE="${IDF_IMAGE:-espressif/idf:v5.5.4}"
BUILD_DIR="build_host_test"

if [[ "${1:-}" == "--local" ]]; then
    cd "${REPO_ROOT}"
    cmake -S host_test -B "${BUILD_DIR}" -G Ninja
    cmake --build "${BUILD_DIR}"
    ctest --test-dir "${BUILD_DIR}" --output-on-failure
    exit 0
fi

exec docker run --rm \
    --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -v "${REPO_ROOT}:/project" -w /project \
    "${IDF_IMAGE}" \
    bash -c "cmake -S host_test -B ${BUILD_DIR} -G Ninja \
             && cmake --build ${BUILD_DIR} \
             && ctest --test-dir ${BUILD_DIR} --output-on-failure -V"
