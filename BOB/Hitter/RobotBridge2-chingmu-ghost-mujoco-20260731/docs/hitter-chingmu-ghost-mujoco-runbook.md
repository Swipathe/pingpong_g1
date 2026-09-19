# HITTER ChingMu Ghost-Ball MuJoCo Runbook

This runbook is for the isolated HIL project only.

## Isolation paths

- Project: `/home/loco1/BOB/Hitter/RobotBridge2-chingmu-ghost-mujoco-20260731`
- Conda env: `/home/loco1/miniconda3/envs/rb-hitter-ghost-20260731`
- Source project preserved: `/home/loco1/BOB/Hitter/RobotBridge2`
- Source env preserved: `/home/loco1/miniconda3/envs/rb`

Deleting the isolated project or isolated environment requires separate explicit approval.

## Prerequisite checks

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2-chingmu-ghost-mujoco-20260731/deploy
/home/loco1/miniconda3/envs/rb-hitter-ghost-20260731/bin/python -m py_compile \
  simulator/mujoco.py envs/hitter.py utils/hitter_chingmu_ghost.py
pgrep -af 'chingmu_table_lcm_bridge.py|trans.cpp|pd_plustau_targets' || true
```

Expected: existing ChingMu bridge may be present; no `trans.cpp` is required for this MuJoCo HIL mode.

## Receive-only control monitor

Run this in a separate terminal before a live smoke:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2-chingmu-ghost-mujoco-20260731/deploy
/home/loco1/miniconda3/envs/rb-hitter-ghost-20260731/bin/python \
  diagnostics/hitter_hil_control_audit.py \
  --lcm-url 'udpm://239.255.76.67:7667?ttl=255' \
  --channel pd_plustau_targets \
  --duration-s 20 \
  --output outputs/chingmu-ghost-control-audit.json
```

The expected `message_count` is `0`.

## MuJoCo HIL launch

Use an explicit ONNX override; do not change default YAML:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2-chingmu-ghost-mujoco-20260731/deploy
export MODEL_ONNX=/absolute/path/to/policy.onnx
MUJOCO_GL=egl \
/home/loco1/miniconda3/envs/rb-hitter-ghost-20260731/bin/python run.py \
  --config-name hitter \
  sim=mujoco_chingmu_ghost \
  mimic.policy.checkpoint="$MODEL_ONNX" \
  robot.control.viewer=true \
  robot.control.real_time=true
```

Expected safety banner/evidence:

- `simulator.mujoco.Mujoco`
- `hitter_ball_input.mode=chingmu_ghost`
- `planner_feed=snapshot_stream`
- `loop_pacing=simulator`
- `read_only=true`
- `use_mujoco_base_pose=true`
- `ghost_collision=false`
- no `RealWorld`
- no `trans.cpp`
- no `pd_plustau_targets` publish

## Three-ball smoke checklist

1. Start the receive-only control monitor.
2. Start the MuJoCo HIL command above.
3. Confirm the ghost ball appears only when the ChingMu stream is valid and table-gated.
4. Confirm the robot base/proprioception moves only through MuJoCo.
5. Confirm planner lifecycle reaches `ARMED` for valid incoming balls.
6. Confirm invalid/stale ball data hides the ghost and returns to `WAITING`.
7. Confirm the control monitor summary reports zero messages.

## Twenty-serve acceptance checklist

1. Capture valid within-track ball inter-arrival times.
2. Compute p99.
3. Set `stale_timeout_s = min(0.25, max(0.10, 5 * p99_s))` for the acceptance run.
4. Run 20 serves without control publications.
5. Record planner result age, policy TTS, ghost/racket distance, and ChingMu rejection counts.
6. Keep the control audit JSON with the run output.

If no valid track is observed, keep `stale_timeout_s=0.10` as preliminary and do not claim twenty-serve acceptance.

## Shutdown and audit

Stop MuJoCo with `Ctrl+C`, then check:

```bash
pgrep -af 'run.py|trans.cpp|pd_plustau_targets' || true
cat outputs/chingmu-ghost-control-audit.json
git status --short
```

Only ignored runtime outputs should change after a manual smoke.
