#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOTBRIDGE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SDK_DIR="${ROBOTBRIDGE_DIR}/vicon_datastream_sdk/linux64/Linux64"
UNITREE_INCLUDE_DIR="${ROBOTBRIDGE_DIR}/unitree_sdk2/include"
UNITREE_SDK_LIB="${ROBOTBRIDGE_DIR}/unitree_sdk2/lib/x86_64/libunitree_sdk2.a"
BUILD_DIR="${SCRIPT_DIR}/.build-v2"
mkdir -p "${BUILD_DIR}"

mapfile -t LCM_CFLAGS < <(pkg-config --cflags-only-I lcm | tr ' ' '\n' | sed '/^$/d')
mapfile -t LCM_LIBS < <(pkg-config --libs lcm | tr ' ' '\n' | sed '/^$/d')

g++ -std=c++17 -O2 -I"${ROBOTBRIDGE_DIR}" \
  "${LCM_CFLAGS[@]}" \
  "${SCRIPT_DIR}/tests/test_transformation_t_v2.cpp" \
  "${LCM_LIBS[@]}" \
  -o "${BUILD_DIR}/test_transformation_t_v2"

g++ -std=c++17 -O2 -I"${SDK_DIR}" -I"${ROBOTBRIDGE_DIR}" \
  -I"${UNITREE_INCLUDE_DIR}" "${LCM_CFLAGS[@]}" \
  "${SCRIPT_DIR}/vicon_table_lcm_bridge.cpp" \
  -L"${SDK_DIR}" -Wl,-rpath,"${SDK_DIR}" -lViconDataStreamSDK_CPP \
  "${UNITREE_SDK_LIB}" "${LCM_LIBS[@]}" \
  -o "${BUILD_DIR}/vicon_table_lcm_bridge_v2"

g++ -std=c++17 -O2 -I"${SDK_DIR}" -I"${ROBOTBRIDGE_DIR}" \
  -I"${UNITREE_INCLUDE_DIR}" "${LCM_CFLAGS[@]}" \
  "${SCRIPT_DIR}/tests/test_vicon_ball_track_v2.cpp" \
  -L"${SDK_DIR}" -Wl,-rpath,"${SDK_DIR}" -lViconDataStreamSDK_CPP \
  "${UNITREE_SDK_LIB}" "${LCM_LIBS[@]}" \
  -o "${BUILD_DIR}/test_vicon_ball_track_v2"
