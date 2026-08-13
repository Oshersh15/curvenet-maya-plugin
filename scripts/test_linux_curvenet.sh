#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
maya_location="${MAYA_LOCATION:-/usr/autodesk/maya2025}"
build_dir="${CURVENET_LINUX_BUILD_DIR:-${project_dir}/plugin/build-linux-test}"
log_file="${CURVENET_LINUX_LOG:-/tmp/curvenet-linux-pipeline.log}"

if [[ ! -x "${maya_location}/bin/mayapy" ]]; then
    echo "Maya mayapy was not found at ${maya_location}/bin/mayapy" >&2
    echo "Set MAYA_LOCATION to the Maya 2025 installation directory." >&2
    exit 2
fi

cmake \
    -S "${project_dir}/plugin" \
    -B "${build_dir}" \
    -DMAYA_LOCATION="${maya_location}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_COMPILER=/usr/bin/gcc \
    -DCMAKE_CXX_COMPILER=/usr/bin/g++
cmake --build "${build_dir}" --target CurvenetPlugin HalfEdgeTests -j 4
ctest --test-dir "${build_dir}" --output-on-failure

export CURVENET_PLUGIN_PATH="${build_dir}/CurvenetPlugin.so"
export MAYA_DISABLE_CIP=1
export MAYA_DISABLE_CER=1

echo "Running the complete Curvenet pipeline in Maya standalone..."
set +e
timeout 120 "${maya_location}/bin/mayapy" \
    "${project_dir}/maya/test_linux_curvenet_pipeline.py" \
    2>&1 | tee "${log_file}"
status=${PIPESTATUS[0]}
set -e

if [[ ${status} -eq 124 ]]; then
    echo "CURVENET_LINUX_PIPELINE: TIMEOUT" | tee -a "${log_file}"
elif [[ ${status} -ne 0 ]]; then
    echo "CURVENET_LINUX_PIPELINE: PROCESS_EXIT_${status}" | tee -a "${log_file}"
fi

echo "Linux test log: ${log_file}"
exit "${status}"
