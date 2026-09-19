#!/usr/bin/env bash
set -euo pipefail

GHOST_ROOT=/home/loco1/BOB/Hitter/RobotBridge2-chingmu-ghost-mujoco-20260731
GHOST_DEPLOY="$GHOST_ROOT/deploy"
GHOST_PY=/home/loco1/miniconda3/envs/rb-hitter-ghost-20260731/bin/python
GHOST_MODEL="$GHOST_DEPLOY/data/model/hitter/hitter_user_667f62b8_104x29_20260807.onnx"
GHOST_MODEL_SHA256=667f62b88c51951dad1880ad2693820ee44c36194ac9655c44b2e8d58029248d
GHOST_SIM_CONFIG="$GHOST_DEPLOY/config/sim/mujoco_chingmu_ghost.yaml"
GHOST_DEFAULT_CONFIG="$GHOST_DEPLOY/config/mimic/hitter.yaml"
GHOST_DEFAULT_CONFIG_SHA256=0e258f3a1fa5940c50c900ae2a08cb6d3b66092c6bdcd30e6fd4f7117c21bea6
GHOST_RUN_ID=hitter_ghost_user_667f62b8_vx0to6_vyneg05to05_vz0to6_20260807_1145
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
test -r /run/user/1001/gdm/Xauthority
test "$(sha256sum "$GHOST_MODEL" | awk '{print $1}')" = "$GHOST_MODEL_SHA256"
test "$(sha256sum "$GHOST_DEFAULT_CONFIG" | awk '{print $1}')" = "$GHOST_DEFAULT_CONFIG_SHA256"
grep -q 'mode: chingmu_ghost' "$GHOST_SIM_CONFIG"
grep -q 'robot_backend: mujoco' "$GHOST_SIM_CONFIG"
grep -q 'read_only: true' "$GHOST_SIM_CONFIG"
grep -q 'ghost_collision: false' "$GHOST_SIM_CONFIG"
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
