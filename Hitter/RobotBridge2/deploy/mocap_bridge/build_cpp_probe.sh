#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOTBRIDGE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SDK_DIR="${ROBOTBRIDGE_DIR}/vicon_datastream_sdk/linux64/Linux64"
UNITREE_INCLUDE_DIR="${ROBOTBRIDGE_DIR}/unitree_sdk2/include"
UNITREE_SDK_LIB="${ROBOTBRIDGE_DIR}/unitree_sdk2/lib/x86_64/libunitree_sdk2.a"
LCM_CFLAGS=()
LCM_LIBS=("-llcm")

if [[ ! -d "${SDK_DIR}" ]]; then
  echo "Vicon DataStream SDK Linux64 directory not found: ${SDK_DIR}" >&2
  exit 1
fi
if [[ ! -d "${UNITREE_INCLUDE_DIR}" || ! -f "${UNITREE_SDK_LIB}" ]]; then
  echo "Unitree SDK JSON support not found under ${ROBOTBRIDGE_DIR}/unitree_sdk2" >&2
  exit 1
fi

mkdir -p "${SCRIPT_DIR}/bin"

if command -v pkg-config >/dev/null 2>&1 && pkg-config --exists lcm; then
  # shellcheck disable=SC2207
  LCM_CFLAGS=($(pkg-config --cflags lcm))
  # shellcheck disable=SC2207
  LCM_LIBS=($(pkg-config --libs lcm))
else
  if [[ -z "${LCM_INCLUDE_DIR:-}" || -z "${LCM_LIB_DIR:-}" ]]; then
    for candidate in \
      "${ROBOTBRIDGE_DIR}/../lcm" \
      /home/*/.venvs/*/lib/python*/site-packages \
      /root/.venvs/*/lib/python*/site-packages \
      /opt/conda/envs/*/lib/python*/site-packages; do
      if [[ -f "${candidate}/include/lcm/lcm-cpp.hpp" && -f "${candidate}/lib/liblcm.so" ]]; then
        LCM_INCLUDE_DIR="${candidate}/include"
        LCM_LIB_DIR="${candidate}/lib"
        break
      fi
    done
  fi
  if [[ -n "${LCM_INCLUDE_DIR:-}" && -n "${LCM_LIB_DIR:-}" ]]; then
    LCM_CFLAGS=("-I${LCM_INCLUDE_DIR}")
    LCM_LIBS=("-L${LCM_LIB_DIR}" "-Wl,-rpath,${LCM_LIB_DIR}" "-llcm")
  fi
fi

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
    -I"${UNITREE_INCLUDE_DIR}" \
    "${LCM_CFLAGS[@]}" \
    "${SCRIPT_DIR}/vicon_table_lcm_bridge.cpp" \
    -L"${SDK_DIR}" \
    -Wl,-rpath,"${SDK_DIR}" \
    -lViconDataStreamSDK_CPP \
    "${UNITREE_SDK_LIB}" \
    "${LCM_LIBS[@]}" \
    -o "${SCRIPT_DIR}/bin/vicon_table_lcm_bridge"

  echo "Built ${SCRIPT_DIR}/bin/vicon_table_lcm_bridge"
fi

g++ -std=c++17 -O2 \
  -I"${SDK_DIR}" \
  -I"${ROBOTBRIDGE_DIR}" \
  -I"${UNITREE_INCLUDE_DIR}" \
  "${LCM_CFLAGS[@]}" \
  "${SCRIPT_DIR}/tests/test_vicon_table_lcm_bridge.cpp" \
  -L"${SDK_DIR}" \
  -Wl,-rpath,"${SDK_DIR}" \
  -lViconDataStreamSDK_CPP \
  "${UNITREE_SDK_LIB}" \
  "${LCM_LIBS[@]}" \
  -o "${SCRIPT_DIR}/bin/test_vicon_table_lcm_bridge"
