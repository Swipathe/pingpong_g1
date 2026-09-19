# Real-World Backhand Racket Y Decrement Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Feed the HITTER policy a copy of the planner racket velocity whose world/table Y component is reduced by `0.3 m/s` only for real-world backhand commands.

**Architecture:** Extend the existing policy-input velocity adapter in `HitterEnv`; planner and lifecycle objects remain immutable. Configuration, finite/`float32` validation, observation adaptation, and raw-versus-policy logging stay at the same boundary as the existing real-world forehand X offset.

**Tech Stack:** Python 3, NumPy, OmegaConf/Hydra, pytest, Loguru.

## Global Constraints

- Modify only `/home/loco1/BOB/Hitter/RobotBridge4`; do not modify RobotBridge3.
- Only `simulator.is_real` with final `strike_type == "backhand"` applies `v_policy_w[1] = v_planner_w[1] - 0.3`.
- Preserve the existing real-world forehand rule `v_policy_w[0] = v_planner_w[0] + 0.25` and do not apply the backhand Y decrement to forehand commands.
- MuJoCo must receive the unadjusted planner velocity for both strike types.
- Never mutate planner/lifecycle velocity storage and never accumulate an offset across observation frames.
- Log raw and policy vectors plus actual X and Y offsets; a real-world backhand logs `policy_vy_offset_mps=-0.3000`.
- The deployment config key is `policy.real_world_backhand_racket_velocity_y_decrement_mps: 0.3`, defaulting to `0.0` when absent.
- Reject bools, strings, negative values, non-finite values, values outside `float32`, and subtraction results outside `float32`.
- Preserve the dirty worktree. Do not reset, revert, clean, broad-stage, or commit implementation/test/config files that contain prior user changes.

---

### Task 1: Add and Verify the Real-World Backhand Policy Velocity Adjustment

**Files:**
- Modify: `deploy/config/mimic/hitter.yaml`
- Modify: `deploy/envs/hitter.py`
- Modify: `deploy/tests/test_hitter_forehand_policy_vx_offset.py`
- Modify: `deploy/tests/test_hitter_runtime_single_shot_integration.py`
- Modify: `deploy/tests/test_hitter_strike_target_logging.py`

**Interfaces:**
- Consumes: planner/lifecycle `v_racket_target_w`, final `strike_type`, and `simulator.is_real`.
- Produces: `_backhand_policy_vy_decrement_from_policy_cfg(policy_cfg) -> float` and `_policy_racket_target_velocity_w(...) -> tuple[np.ndarray, float, float]`, where the floats are the actually applied X and Y offsets.

- [ ] **Step 1: Write behavior-first failing tests**

Extend the existing tests with literal expectations that catch a missing or wrongly gated decrement:

```python
# real-world backhand
raw = [1.0, 0.1, 0.2]
expected_policy = [1.0, -0.2, 0.2]

# MuJoCo backhand
expected_policy = [1.0, 0.1, 0.2]
```

Add tests named:

- `test_backhand_policy_vy_decrement_reads_deployment_key_and_defaults_to_zero`
- `test_backhand_policy_vy_decrement_accepts_dictconfig_policy_node`
- `test_backhand_policy_vy_decrement_rejects_invalid_or_unrepresentable_values`
- `test_real_backhand_policy_velocity_rejects_float32_subtraction_overflow`
- `test_policy_racket_velocity_offsets_are_real_world_and_strike_side_specific`
- `test_real_backhand_policy_racket_velocity_y_decrement_does_not_accumulate`
- `test_real_backhand_logs_raw_and_policy_racket_velocity`

Assert that the frozen planner command remains `[1.0, 0.1, 0.2]`. For logging, use raw `[1.4, 0.1, 0.7]` and assert policy `[1.4, -0.2, 0.7]`, X offset `0.0000`, and Y offset `-0.3000`.

- [ ] **Step 2: Run the focused tests and verify RED**

Run:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -p no:cacheprovider -q \
  deploy/tests/test_hitter_forehand_policy_vx_offset.py \
  deploy/tests/test_hitter_runtime_single_shot_integration.py::test_policy_racket_velocity_offsets_are_real_world_and_strike_side_specific \
  deploy/tests/test_hitter_runtime_single_shot_integration.py::test_real_backhand_policy_racket_velocity_y_decrement_does_not_accumulate \
  deploy/tests/test_hitter_strike_target_logging.py::StrikeTargetPayloadTests::test_real_backhand_logs_raw_and_policy_racket_velocity
```

Expected: failures caused by the missing backhand configuration resolver/decrement/log field, including actual real-world backhand Y `0.1` versus expected `-0.2`.

- [ ] **Step 3: Implement the minimal policy-boundary adjustment**

Add the deployment configuration:

```yaml
real_world_backhand_racket_velocity_y_decrement_mps: 0.3
```

At environment initialization, resolve and validate it with default `0.0`. Extend `_policy_racket_target_velocity_w()` so it always begins from a validated independent `float32 (3,)` copy, applies exactly one branch for real-world `forehand` or `backhand`, performs arithmetic in `float64`, verifies the result is finite and representable as `float32`, and returns the vector with actual X/Y offsets. Update observation and strike-target logging callers for the new return contract. Append `policy_vy_offset_mps={:.4f}` to the existing target log.

- [ ] **Step 4: Run the focused tests and verify GREEN**

Run the exact Step 2 command. Expected: all selected tests pass; only the two already-known invalid-escape deprecation warnings may remain.

- [ ] **Step 5: Run the scoped regression suite**

Run:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -p no:cacheprovider -q \
  deploy/tests/test_hitter_forehand_policy_vx_offset.py \
  deploy/tests/test_hitter_runtime_single_shot_integration.py \
  deploy/tests/test_hitter_strike_target_logging.py
```

Expected: no new failures. The three pre-existing RECOVERY estimator-prewarm assertions may still fail and must be reported rather than changed in this task.

- [ ] **Step 6: Verify the scoped diff without staging implementation files**

Run `git diff --check` on the five task files, inspect `git diff` and `git status --short`, and confirm the existing model checkpoint, `virtual_hit_plane_x: 0.0`, forehand X offset `0.25`, and new backhand Y decrement `0.3` in the resolved Hydra configuration. Do not stage or commit these mixed-dirty implementation files.
