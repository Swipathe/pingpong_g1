#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOTBRIDGE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
BRIDGE_BIN="${SCRIPT_DIR}/.build-v2/vicon_table_lcm_bridge_v2"
SDK_DIR="${ROBOTBRIDGE_DIR}/vicon_datastream_sdk/linux64/Linux64"
TABLE_CALIB="${SCRIPT_DIR}/calibrations/vicon_table_frame_20260822_validated.json"
PELVIS_CALIB="${SCRIPT_DIR}/calibrations/vicon_g1_pelvis_orientation_20260822_validated.json"
PLANNER_CONFIG="${ROBOTBRIDGE_DIR}/deploy/config/mimic/hitter.yaml"

TABLE_SHA256="3a7b08abd615e0fa06715c8af424b01cf346a39c0734f4e32bdf6d5d0663b649"
PELVIS_SHA256="a9f841d119ba8db531a7c463d5c7c2bcef27e747e5f62e88418531a15d9d56ee"

usage() {
  printf 'Usage: %s [--check-only|--check-live]\n' "$0" >&2
}

verify_sha256() {
  local label="$1"
  local file="$2"
  local expected="$3"
  local digest_output
  local actual

  if [[ ! -f "$file" ]]; then
    printf 'ERROR: %s calibration is missing: %s\n' "$label" "$file" >&2
    exit 1
  fi
  digest_output="$(sha256sum -- "$file")"
  actual="${digest_output%% *}"
  if [[ ! "$actual" =~ ^[0-9a-f]{64}$ ]]; then
    printf 'ERROR: could not resolve %s calibration SHA256: %s\n' "$label" "$file" >&2
    exit 1
  fi
  if [[ "$actual" != "$expected" ]]; then
    printf 'ERROR: %s calibration checksum mismatch.\n' "$label" >&2
    printf '  file:     %s\n' "$file" >&2
    printf '  expected: %s\n' "$expected" >&2
    printf '  actual:   %s\n' "$actual" >&2
    exit 1
  fi
  printf '%s calibration: %s (sha256=%s)\n' "$label" "$file" "$actual"
}

verify_policy_calibration_contract() {
  python3 - "$TABLE_CALIB" "$PELVIS_CALIB" "$PLANNER_CONFIG" <<'PY'
import ast
import json
import math
import sys
from pathlib import Path

table_path, pelvis_path, planner_path = map(Path, sys.argv[1:])
table = json.loads(table_path.read_text(encoding="utf-8"))
pelvis = json.loads(pelvis_path.read_text(encoding="utf-8"))

wanted_keys = {
    "channel",
    "base_subject",
    "table_center_xy_w",
    "table_height",
    "table_length",
    "table_width",
}
raw_values = {}
for line in planner_path.read_text(encoding="utf-8").splitlines():
    if not line.startswith("    ") or line.startswith("      "):
        continue
    key, separator, raw_value = line.strip().partition(":")
    if separator and key in wanted_keys:
        if key in raw_values:
            raise SystemExit(f"duplicate planner calibration key: {key}")
        raw_values[key] = raw_value.split("#", 1)[0].strip()

missing = sorted(wanted_keys - raw_values.keys())
if missing:
    raise SystemExit(f"missing planner calibration keys: {missing}")

actual = {
    "channel": raw_values["channel"],
    "base_subject": raw_values["base_subject"],
    "table_center_xy_w": ast.literal_eval(raw_values["table_center_xy_w"]),
    "table_height": float(raw_values["table_height"]),
    "table_length": float(raw_values["table_length"]),
    "table_width": float(raw_values["table_width"]),
}
expected = {
    "channel": "vicon_state_data_v2",
    "base_subject": pelvis["base_subject"],
    "table_center_xy_w": [float(table["table_length_m"]) / 2.0, 0.0],
    "table_height": float(table["table_height_m"]),
    "table_length": float(table["table_length_m"]),
    "table_width": float(table["table_width_m"]),
}

for key, expected_value in expected.items():
    actual_value = actual[key]
    if isinstance(expected_value, list):
        matches = (
            len(actual_value) == len(expected_value)
            and all(
                math.isclose(float(a), float(b), abs_tol=1e-9)
                for a, b in zip(actual_value, expected_value)
            )
        )
    elif isinstance(expected_value, float):
        matches = math.isclose(float(actual_value), expected_value, abs_tol=1e-9)
    else:
        matches = actual_value == expected_value
    if not matches:
        raise SystemExit(
            f"planner calibration mismatch for {key}: "
            f"actual={actual_value!r} expected={expected_value!r}"
        )

print(
    "policy calibration contract: "
    "vicon_state_data_v2/G2Pelvis, "
    f"center={actual['table_center_xy_w']}, "
    f"H/L/W={actual['table_height']}/{actual['table_length']}/{actual['table_width']}"
)
PY
}

MODE="publish"
case "${1:-}" in
  "") ;;
  --check-only) MODE="check-only" ;;
  --check-live) MODE="check-live" ;;
  *)
    usage
    exit 2
    ;;
esac
if (( $# > 1 )); then
  usage
  exit 2
fi

if [[ ! -x "$BRIDGE_BIN" ]]; then
  printf 'ERROR: Vicon bridge binary is missing or not executable: %s\n' \
    "$BRIDGE_BIN" >&2
  printf 'Build it with: bash %s/build_v2_mocap.sh\n' "$SCRIPT_DIR" >&2
  exit 1
fi

verify_sha256 "table" "$TABLE_CALIB" "$TABLE_SHA256"
verify_sha256 "pelvis" "$PELVIS_CALIB" "$PELVIS_SHA256"
verify_policy_calibration_contract
printf 'RobotBridge4 real deployment calibration bundle: 20260822 validated\n'

if [[ "$MODE" == "check-only" ]]; then
  printf 'Calibration preflight: PASS\n'
  exit 0
fi

cd "$ROBOTBRIDGE_DIR"
BRIDGE_ARGS=(
  --host 192.168.10.1:801
  --tracker-name G1Pelvis
  --base-subject G2Pelvis
  --table-calib "$TABLE_CALIB"
  --pelvis-orientation-calib "$PELVIS_CALIB"
  --lcm-url 'udpm://239.255.76.67:7667?ttl=255'
  --channel vicon_state_data_v2
  --vicon-frame-rate-hz 300
  --corner-exclusion-radius-mm 50
  --ball-track-association-radius-m 0.35
  --ball-track-end-timeout-s 0.25
)

if [[ "$MODE" == "check-live" ]]; then
  exec env \
    LD_LIBRARY_PATH="${SDK_DIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
    "$BRIDGE_BIN" \
      "${BRIDGE_ARGS[@]}" \
      --print-hz 2 \
      --duration 3 \
      --no-publish
fi

exec env \
  LD_LIBRARY_PATH="${SDK_DIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
  "$BRIDGE_BIN" \
    "${BRIDGE_ARGS[@]}" \
    --print-hz 1 \
    --duration 0 \
    --publish
