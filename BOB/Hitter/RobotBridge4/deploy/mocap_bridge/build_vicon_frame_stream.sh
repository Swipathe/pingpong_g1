#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SDK_DIR="${ROOT_DIR}/vicon_datastream_sdk/linux64/Linux64"
OUT_DIR="${SCRIPT_DIR}/bin"
OUT="${OUT_DIR}/vicon_frame_stream"

mkdir -p "${OUT_DIR}"

g++ -std=c++17 -O2 -Wall -Wextra \
  -I"${SDK_DIR}" \
  "${SCRIPT_DIR}/vicon_frame_stream.cpp" \
  -L"${SDK_DIR}" \
  -Wl,-rpath,"${SDK_DIR}" \
  -lViconDataStreamSDK_CPP \
  -o "${OUT}"

echo "Built ${OUT}"
