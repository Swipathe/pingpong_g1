#!/usr/bin/env bash
set -euo pipefail

GHOST_ROOT=/home/loco1/BOB/Hitter/RobotBridge2-chingmu-ghost-mujoco-20260731
GHOST_DEPLOY="$GHOST_ROOT/deploy"
GHOST_PY=/home/loco1/miniconda3/envs/rb-hitter-ghost-20260731/bin/python
GHOST_MODEL="$GHOST_DEPLOY/data/model/hitter/hitter_model14100_8x4090_poswin001_velstd18_velwin003_104_20260808.onnx"
GHOST_MODEL_SHA256=5439bf65b32303a965075c30f127e331b1342b3db5c4103e31551af1e626f25e
GHOST_CHECKPOINT="$GHOST_DEPLOY/data/model/hitter/model_14100_8x4090_poswin001_velstd18_velwin003_20260808.pt"
GHOST_CHECKPOINT_SHA256=cdb6e6e4cc3bf6a563a9ddecdab0a7d2e3e137e4e46f369f2689486e135c4569
GHOST_SIM_CONFIG="$GHOST_DEPLOY/config/sim/mujoco_chingmu_ghost.yaml"
GHOST_DEFAULT_CONFIG="$GHOST_DEPLOY/config/mimic/hitter.yaml"
GHOST_DEFAULT_CONFIG_SHA256=fdb50ce5b6116a3303692e9a077b26f5b8fb0896570d7a00b7c6bca8b5329085
GHOST_RUN_ID=hitter_ghost_model14100_8x4090_capturecurrent_vx0to6_vyneg05to05_vz0to6_20260808_133954
GHOST_LOG="$GHOST_DEPLOY/ghost_launch_logs/${GHOST_RUN_ID}.log"
GHOST_STATUS="$GHOST_DEPLOY/ghost_launch_logs/${GHOST_RUN_ID}.status"

record_exit() {
  local status=$?
  trap - EXIT
  printf 'exit_code=%s\nfinished_at=%s\n' "$status" "$(date -Is)" > "$GHOST_STATUS"
  exit "$status"
}
trap record_exit EXIT

mkdir -p "$GHOST_DEPLOY/ghost_launch_logs"

test -x "$GHOST_PY"
test -s "$GHOST_MODEL"
test -s "$GHOST_CHECKPOINT"
test -r /run/user/1001/gdm/Xauthority
test "$(sha256sum "$GHOST_MODEL" | cut -d' ' -f1)" = "$GHOST_MODEL_SHA256"
test "$(sha256sum "$GHOST_CHECKPOINT" | cut -d' ' -f1)" = "$GHOST_CHECKPOINT_SHA256"
test "$(sha256sum "$GHOST_DEFAULT_CONFIG" | cut -d' ' -f1)" = "$GHOST_DEFAULT_CONFIG_SHA256"
grep -q 'mode: chingmu_ghost' "$GHOST_SIM_CONFIG"
grep -q 'robot_backend: mujoco' "$GHOST_SIM_CONFIG"
grep -q 'read_only: true' "$GHOST_SIM_CONFIG"
grep -q 'ghost_collision: false' "$GHOST_SIM_CONFIG"
grep -q 'waiting_base_target_mode: capture_current' "$GHOST_DEFAULT_CONFIG"
test -z "$(pgrep -af '[p]ython.*run.py.*mujoco_chingmu_ghost' || true)"
test -z "$(pgrep -af '[p]ython.*run.py.*sim=real_world' || true)"

export DISPLAY=:1
export XAUTHORITY=/run/user/1001/gdm/Xauthority

cd "$GHOST_DEPLOY"

set +e
"$GHOST_PY" -u run.py \
  --config-name=hitter \
  sim=mujoco_chingmu_ghost \
  device=cpu \
  mimic.policy.checkpoint="$GHOST_MODEL" \
  mimic.policy.save_video=false \
  robot.control.viewer=true \
  robot.control.real_time=true \
  'mimic.motion.ball_planner.racket_velocity_component_ranges_mps.x=[0.0,6.0]' \
  'mimic.motion.ball_planner.racket_velocity_component_ranges_mps.y=[-0.5,0.5]' \
  'mimic.motion.ball_planner.racket_velocity_component_ranges_mps.z=[0.0,6.0]' \
  2>&1 | tee -a "$GHOST_LOG"
status=${PIPESTATUS[0]}
set -e
exit "$status"
