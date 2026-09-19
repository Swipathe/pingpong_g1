# Task 1 Report: Real-World Backhand Policy Y-Velocity Decrement

## Scope and files changed

- `deploy/config/mimic/hitter.yaml`
  - Added `policy.real_world_backhand_racket_velocity_y_decrement_mps: 0.2`.
- `deploy/envs/hitter.py`
  - Resolves and validates the new policy value at environment initialization.
  - Adds the backhand resolver and applies the real-world, backhand-only Y decrement at the policy-input boundary.
  - Returns applied X/Y offsets, and includes the Y offset in strike-target logging.
- `deploy/tests/test_hitter_forehand_policy_vx_offset.py`
  - Added resolver/default/DictConfig/invalid-value/float32-subtraction-overflow behavior tests.  This file was already untracked in the dirty worktree before this task.
- `deploy/tests/test_hitter_runtime_single_shot_integration.py`
  - Added real-world and strike-side-specific adapter coverage and non-accumulation coverage.
- `deploy/tests/test_hitter_strike_target_logging.py`
  - Added raw-versus-policy backhand logging coverage.
- `.superpowers/sdd/2026-08-15-real-world-backhand-racket-y-decrement/task-1-report.md`
  - This task report.

No files were staged or committed.

## RED: behavior tests before production/config implementation

Command run verbatim:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -p no:cacheprovider -q \
  deploy/tests/test_hitter_forehand_policy_vx_offset.py \
  deploy/tests/test_hitter_runtime_single_shot_integration.py::test_policy_racket_velocity_offsets_are_real_world_and_strike_side_specific \
  deploy/tests/test_hitter_runtime_single_shot_integration.py::test_real_backhand_policy_racket_velocity_y_decrement_does_not_accumulate \
  deploy/tests/test_hitter_strike_target_logging.py::StrikeTargetPayloadTests::test_real_backhand_logs_raw_and_policy_racket_velocity
```

Output summary:

```text
..........FFFFFFFFFF.F..FF                                               [100%]
13 failed, 13 passed, 2 warnings in 1.05s
```

The failures were correct and feature-specific:

- The new resolver tests raised `AttributeError` because `HitterEnv` had no `_backhand_policy_vy_decrement_from_policy_cfg`.
- The overflow test did not raise because no backhand subtraction existed.
- The real-world backhand observation was actual `[1.0, 0.1, 0.2]` but expected `[1.0, -0.1, 0.2]`.
- The log retained `v_racket_policy_w_mps=[1.4000,0.1000,0.7000]` and had no `policy_vy_offset_mps` field, rather than the required backhand policy velocity and offset.

## Minimal implementation

- The deployment key defaults to `0.0` when absent and accepts numeric finite, non-negative float32-representable values only.
- `_policy_racket_target_velocity_w()` begins from an independent validated float32 `(3,)` copy, promotes arithmetic to float64, and then verifies the whole result is finite and float32-representable before returning it.
- Only one real-world strike-side branch applies: forehand adds the existing X offset; backhand subtracts the new Y decrement. MuJoCo leaves the vector unchanged.
- The function returns `(policy_velocity, applied_x_offset, applied_y_offset)`. The observation path consumes only the returned policy vector; logging records raw and policy vectors plus both applied offsets.
- The lifecycle/planner command remains untouched; repeated observation construction re-adapts from the frozen raw command and therefore cannot accumulate either offset.

## GREEN: focused suite

Command run verbatim:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -p no:cacheprovider -q \
  deploy/tests/test_hitter_forehand_policy_vx_offset.py \
  deploy/tests/test_hitter_runtime_single_shot_integration.py::test_policy_racket_velocity_offsets_are_real_world_and_strike_side_specific \
  deploy/tests/test_hitter_runtime_single_shot_integration.py::test_real_backhand_policy_racket_velocity_y_decrement_does_not_accumulate \
  deploy/tests/test_hitter_strike_target_logging.py::StrikeTargetPayloadTests::test_real_backhand_logs_raw_and_policy_racket_velocity
```

Output:

```text
..........................                                               [100%]
26 passed, 2 warnings in 1.01s
```

The only warnings were the two already-known `DeprecationWarning: invalid escape sequence \\*` warnings.

## Scoped regression suite

Command run verbatim:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -p no:cacheprovider -q \
  deploy/tests/test_hitter_forehand_policy_vx_offset.py \
  deploy/tests/test_hitter_runtime_single_shot_integration.py \
  deploy/tests/test_hitter_strike_target_logging.py
```

Output:

```text
............................................FF...F...................... [100%]
3 failed, 69 passed, 2 warnings in 1.14s
```

The three failures are the pre-existing RECOVERY estimator-prewarm assertions, not failures of this adapter:

- `test_listener_consumes_every_snapshot_during_recovery[False]`
- `test_listener_consumes_every_snapshot_during_recovery[True]`
- `test_listener_rechecks_phase_after_waiting_outside_lifecycle_lock`

They assert that no snapshot is submitted while `RECOVERY`; the already-present dirty change in `_submit_hitter_planner_snapshot()` intentionally permits submission in `RECOVERY`. This task did not modify that behavior.

## Diff/status/config evidence

Command:

```bash
git diff --check -- deploy/config/mimic/hitter.yaml deploy/envs/hitter.py deploy/tests/test_hitter_forehand_policy_vx_offset.py deploy/tests/test_hitter_runtime_single_shot_integration.py deploy/tests/test_hitter_strike_target_logging.py
```

Result: no whitespace errors. `git diff --stat` reported four tracked target files because `deploy/tests/test_hitter_forehand_policy_vx_offset.py` was already untracked; it remains untracked and unstaged.

Scoped status after implementation:

```text
 M deploy/config/mimic/hitter.yaml
 M deploy/envs/hitter.py
 M deploy/tests/test_hitter_runtime_single_shot_integration.py
 M deploy/tests/test_hitter_strike_target_logging.py
?? deploy/tests/test_hitter_forehand_policy_vx_offset.py
```

Resolved Hydra policy/config composition printed:

```text
checkpoint=./data/model/hitter/hitter_model7100_A100_20260815_resume6700_posstd012_104.onnx
virtual_hit_plane_x=0.0
real_world_forehand_racket_velocity_x_offset_mps=0.25
real_world_backhand_racket_velocity_y_decrement_mps=0.2
```

## Self-review and concerns

- Backhand tests use literal raw `[1.0, 0.1, 0.2]` and expected policy `[1.0, -0.1, 0.2]`; both non-accumulation tests also assert the frozen planner values remain raw.
- The backhand log test uses raw `[1.4, 0.1, 0.7]` and checks policy `[1.4, -0.1, 0.7]`, `policy_vx_offset_mps=0.0000`, and `policy_vy_offset_mps=-0.2000`.
- The focused behavior suite is green. The only remaining scoped-regression failures are the three known RECOVERY estimator-prewarm assertions described above; they were left unchanged as required.
