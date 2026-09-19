#!/usr/bin/env bash
# Source this file before using the Vicon DataStream SDK from RobotBridge.

if [[ -n "${BASH_SOURCE[0]:-}" ]]; then
  SCRIPT_PATH="${BASH_SOURCE[0]}"
elif [[ -n "${(%):-%N}" ]]; then
  SCRIPT_PATH="${(%):-%N}"
else
  SCRIPT_PATH="$0"
fi

SCRIPT_DIR="$(cd "$(dirname "${SCRIPT_PATH}")" && pwd)"
export VICON_DATASTREAM_SDK_ROOT="${SCRIPT_DIR}"
export VICON_DATASTREAM_SDK_LINUX64="${VICON_DATASTREAM_SDK_ROOT}/linux64/Linux64"

if [[ -d "${VICON_DATASTREAM_SDK_LINUX64}" ]]; then
  export LD_LIBRARY_PATH="${VICON_DATASTREAM_SDK_LINUX64}:${LD_LIBRARY_PATH:-}"
fi

echo "VICON_DATASTREAM_SDK_ROOT=${VICON_DATASTREAM_SDK_ROOT}"
echo "VICON_DATASTREAM_SDK_LINUX64=${VICON_DATASTREAM_SDK_LINUX64}"
