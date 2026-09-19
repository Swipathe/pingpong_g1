#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOTBRIDGE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SDK_DIR="${ROBOTBRIDGE_DIR}/vicon_datastream_sdk/linux64/Linux64"

if [[ ! -d "${SDK_DIR}" ]]; then
  echo "Vicon DataStream SDK Linux64 directory not found: ${SDK_DIR}" >&2
  exit 1
fi

mkdir -p "${SCRIPT_DIR}/bin"

g++ -std=c++17 -O2 \
  -I"${SDK_DIR}" \
  "${SCRIPT_DIR}/nexus_probe_cpp.cpp" \
  -L"${SDK_DIR}" \
  -Wl,-rpath,"${SDK_DIR}" \
  -lViconDataStreamSDK_CPP \
  -o "${SCRIPT_DIR}/bin/nexus_probe_cpp"

echo "Built ${SCRIPT_DIR}/bin/nexus_probe_cpp"

if [[ -f "${SCRIPT_DIR}/vicon_rigid_unlabeled_probe.cpp" ]]; then
  g++ -std=c++17 -O2 \
    -I"${SDK_DIR}" \
    "${SCRIPT_DIR}/vicon_rigid_unlabeled_probe.cpp" \
    -L"${SDK_DIR}" \
    -Wl,-rpath,"${SDK_DIR}" \
    -lViconDataStreamSDK_CPP \
    -o "${SCRIPT_DIR}/bin/vicon_rigid_unlabeled_probe"

  echo "Built ${SCRIPT_DIR}/bin/vicon_rigid_unlabeled_probe"
fi

if [[ -f "${SCRIPT_DIR}/vicon_table_lcm_bridge.cpp" ]]; then
  g++ -std=c++17 -O2 \
    -I"${SDK_DIR}" \
    -I"${ROBOTBRIDGE_DIR}" \
    "${SCRIPT_DIR}/vicon_table_lcm_bridge.cpp" \
    -L"${SDK_DIR}" \
    -Wl,-rpath,"${SDK_DIR}" \
    -lViconDataStreamSDK_CPP \
    -llcm \
    -o "${SCRIPT_DIR}/bin/vicon_table_lcm_bridge"

  echo "Built ${SCRIPT_DIR}/bin/vicon_table_lcm_bridge"
fi
