# HITTER Stable Incoming Track Confirmation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stop raw Vicon X jitter from resetting ball epochs and require three consecutive 100 Hz snapshots with 31-frame fitted `Vx <= -0.20 m/s` before the real-world planner accepts a track.

**Architecture:** `RealWorld` will no longer infer track boundaries from adjacent raw X samples; explicit lifecycle reset and invalid/occluded messages remain the only real-world epoch boundaries. A small pure `IncomingTrackConfirmation` state object in `hitter_realtime.py` will confirm and latch stable incoming motion per epoch, and `HitterEnv` will apply it only on the real-world planner path before producing a command.

**Tech Stack:** Python 3, NumPy, unittest, Hydra/OmegaConf, existing latest-only planner worker.

## Global Constraints

- Preserve all existing dirty-worktree changes; do not reset, revert, or overwrite unrelated edits.
- Do not introduce action gating.
- Do not change waiting observation semantics or the values `waiting_time_to_strike_s: 0.92` and `waiting_base_target_xy_w: [-0.4, 0.0]`.
- Keep the estimator window at 31 samples and planner submission rate at 100 Hz.
- Apply stable confirmation only to the real-world planner path; MuJoCo behavior remains unchanged.
- Do not commit implementation files because target files already contain user-owned dirty changes; verify the final diff by path instead.

---

### Task 1: Remove raw direction-driven epoch resets

**Files:**
- Modify: `deploy/tests/test_real_world_hitter_snapshots.py:397-488`
- Modify: `deploy/simulator/real_world.py:113-117,246-299,403-423`

**Interfaces:**
- Consumes: existing `RealWorld._update_ball_state_from_vicon(msg, pos)` and `BallStateEstimator`.
- Produces: valid raw ball samples never advance `ball_track_epoch`; explicit `_clear_ball_tracking(..., advance_epoch=True)` remains unchanged.

- [ ] **Step 1: Replace raw reversal tests with a failing preservation test**

```python
def test_raw_x_direction_changes_do_not_advance_epoch_or_reset_estimator(self):
    sim = make_real_world_without_io()
    feed_positions(sim, [[1.0, 0.0, 0.9], [1.01, 0.0, 0.9]])
    old_epoch = sim.ball_track_epoch

    feed_positions(
        sim,
        [[1.0099999, 0.0, 0.9], [1.02, 0.0, 0.9], [1.01, 0.0, 0.9]],
        start_frame=2,
        start_time=0.02,
    )

    self.assertEqual(sim.ball_track_epoch, old_epoch)
    self.assertEqual(sim.ball_state_estimator_sample_count_tmp, 5)
```

- [ ] **Step 2: Run the test and verify RED**

Run:

```bash
PYTHONPATH=deploy conda run --no-capture-output -n rb \
python -m unittest -v \
tests.test_real_world_hitter_snapshots.RealWorldHitterSnapshotTests.test_raw_x_direction_changes_do_not_advance_epoch_or_reset_estimator
```

Expected: FAIL because the existing outgoing-to-incoming raw sign change advances epoch and resets the estimator sample count.

- [ ] **Step 3: Remove raw direction state and update calls**

Delete `_ball_last_raw_position_w`, `_ball_last_source_time_s`, and
`_ball_previous_direction_incoming` initialization/reset assignments. Delete
`_update_ball_direction()` and `_update_ball_direction_locked()`. In the valid branch of
`_update_ball_state_from_vicon()`, remove:

```python
source_time_s = float(getattr(msg, "vicon_time_s", 0.0) or 0.0)
self._update_ball_direction_locked(pos, source_time_s)
```

Keep estimator timestamp selection and `add_sample()` unchanged.

- [ ] **Step 4: Run the real-world snapshot suite and verify GREEN**

Run:

```bash
PYTHONPATH=deploy conda run --no-capture-output -n rb \
python -m unittest -v tests.test_real_world_hitter_snapshots
```

Expected: PASS, including invalid/occluded and explicit reset epoch tests.

### Task 2: Add pure stable incoming confirmation state

**Files:**
- Modify: `deploy/tests/test_hitter_realtime.py`
- Modify: `deploy/utils/hitter_realtime.py`

**Interfaces:**
- Produces: `IncomingTrackConfirmation(minimum_speed_x_mps: float, required_consecutive_snapshots: int)`.
- Produces: `observe(*, track_epoch: int, velocity_x_mps: float) -> bool` and `reset() -> None`.
- Semantics: confirmation latches within one epoch; a new epoch or explicit reset clears it.

- [ ] **Step 1: Add failing unit tests for threshold, consecutive count, reset, and latch**

```python
class IncomingTrackConfirmationTests(unittest.TestCase):
    def make_confirmation(self):
        return IncomingTrackConfirmation(
            minimum_speed_x_mps=0.20,
            required_consecutive_snapshots=3,
        )

    def test_requires_three_consecutive_stable_incoming_snapshots(self):
        confirmation = self.make_confirmation()
        self.assertFalse(confirmation.observe(track_epoch=4, velocity_x_mps=-0.21))
        self.assertFalse(confirmation.observe(track_epoch=4, velocity_x_mps=-0.30))
        self.assertTrue(confirmation.observe(track_epoch=4, velocity_x_mps=-0.40))

    def test_nonqualifying_snapshot_resets_unconfirmed_count(self):
        confirmation = self.make_confirmation()
        self.assertFalse(confirmation.observe(track_epoch=4, velocity_x_mps=-0.30))
        self.assertFalse(confirmation.observe(track_epoch=4, velocity_x_mps=-0.05))
        self.assertFalse(confirmation.observe(track_epoch=4, velocity_x_mps=-0.30))
        self.assertFalse(confirmation.observe(track_epoch=4, velocity_x_mps=-0.30))
        self.assertTrue(confirmation.observe(track_epoch=4, velocity_x_mps=-0.30))

    def test_new_epoch_clears_confirmation_and_confirmation_latches(self):
        confirmation = self.make_confirmation()
        for _ in range(3):
            confirmed = confirmation.observe(track_epoch=4, velocity_x_mps=-0.30)
        self.assertTrue(confirmed)
        self.assertTrue(confirmation.observe(track_epoch=4, velocity_x_mps=0.10))
        self.assertFalse(confirmation.observe(track_epoch=5, velocity_x_mps=-0.30))
```

- [ ] **Step 2: Run tests and verify RED**

Run:

```bash
PYTHONPATH=deploy conda run --no-capture-output -n rb \
python -m unittest -v tests.test_hitter_realtime.IncomingTrackConfirmationTests
```

Expected: ERROR importing `IncomingTrackConfirmation` because it does not exist.

- [ ] **Step 3: Implement the minimal pure state object**

Add to `deploy/utils/hitter_realtime.py`:

```python
class IncomingTrackConfirmation:
    def __init__(self, *, minimum_speed_x_mps: float, required_consecutive_snapshots: int):
        self.minimum_speed_x_mps = float(minimum_speed_x_mps)
        self.required_consecutive_snapshots = int(required_consecutive_snapshots)
        if not np.isfinite(self.minimum_speed_x_mps) or self.minimum_speed_x_mps <= 0.0:
            raise ValueError("minimum_speed_x_mps must be finite and positive")
        if self.required_consecutive_snapshots < 1:
            raise ValueError("required_consecutive_snapshots must be positive")
        self.reset()

    def reset(self) -> None:
        self._track_epoch = None
        self._consecutive_count = 0
        self._confirmed = False

    def observe(self, *, track_epoch: int, velocity_x_mps: float) -> bool:
        epoch = int(track_epoch)
        if self._track_epoch != epoch:
            self._track_epoch = epoch
            self._consecutive_count = 0
            self._confirmed = False
        if self._confirmed:
            return True
        velocity_x = float(velocity_x_mps)
        if np.isfinite(velocity_x) and velocity_x <= -self.minimum_speed_x_mps:
            self._consecutive_count += 1
        else:
            self._consecutive_count = 0
        if self._consecutive_count >= self.required_consecutive_snapshots:
            self._confirmed = True
        return self._confirmed
```

- [ ] **Step 4: Run the confirmation tests and verify GREEN**

Run the Task 2 Step 2 command. Expected: PASS.

### Task 3: Integrate confirmation into only the real-world planner

**Files:**
- Modify: `deploy/config/mimic/hitter.yaml:24-31`
- Modify: `deploy/tests/test_hitter_env_lifecycle.py`
- Modify: `deploy/envs/hitter.py:15-24,250-321,614-653`

**Interfaces:**
- Consumes: `IncomingTrackConfirmation.observe()` from Task 2.
- Produces: first two qualifying real-world snapshots return planner errors; third and later qualifying snapshots use the existing `HitterSystemPlanner`.
- Configuration defaults: `minimum_stable_incoming_speed_x_mps=0.20`, `stable_incoming_confirmation_snapshots=3`.

- [ ] **Step 1: Add failing environment tests**

Create a real-world `HitterEnv` test fixture with a stub `hitter_ball_planner.plan_command()` and three ready snapshots in one epoch. Assert calls one and two raise `ValueError` containing `not stably incoming`, call three returns the stub command. Add a second test that changes epoch after confirmation and asserts the first snapshot of the new epoch is rejected. Add a MuJoCo test with `simulator.is_real=False` and assert its first qualifying snapshot still plans immediately.

- [ ] **Step 2: Run the focused tests and verify RED**

Run:

```bash
PYTHONPATH=deploy conda run --no-capture-output -n rb \
python -m unittest -v \
tests.test_hitter_env_lifecycle.HitterEnvironmentLifecycleTests
```

Expected: FAIL because real-world snapshots currently plan on the first fitted negative velocity.

- [ ] **Step 3: Add configuration and environment integration**

Add to `deploy/config/mimic/hitter.yaml` under `ball_planner`:

```yaml
minimum_stable_incoming_speed_x_mps: 0.20
stable_incoming_confirmation_snapshots: 3
```

Import `IncomingTrackConfirmation` in `deploy/envs/hitter.py`. During HITTER motion-state initialization construct:

```python
self.hitter_incoming_track_confirmation = IncomingTrackConfirmation(
    minimum_speed_x_mps=float(
        planner_cfg.get("minimum_stable_incoming_speed_x_mps", 0.20)
    ),
    required_consecutive_snapshots=int(
        planner_cfg.get("stable_incoming_confirmation_snapshots", 3)
    ),
)
```

In `_plan_hitter_snapshot()`, after visible/ready/base validation and velocity vector validation, apply only for real hardware:

```python
if getattr(self.simulator, "is_real", False):
    confirmed = self.hitter_incoming_track_confirmation.observe(
        track_epoch=snapshot.track_epoch,
        velocity_x_mps=float(ball_vel_w[0]),
    )
    if not confirmed:
        raise ValueError(
            "HITTER ball track is not stably incoming: "
            f"vx={float(ball_vel_w[0]):.3f} m/s."
        )
```

Before raising `TRACK_ENDED_ERROR` for a non-visible real-world snapshot, call
`self.hitter_incoming_track_confirmation.reset()` so an invalid/ended track cannot retain confirmation.

- [ ] **Step 4: Run lifecycle/config tests and verify GREEN**

Run:

```bash
PYTHONPATH=deploy conda run --no-capture-output -n rb \
python -m unittest -v \
tests.test_hitter_env_lifecycle \
tests.test_hitter_realtime \
tests.test_real_world_hitter_snapshots
```

Expected: PASS.

### Task 4: Replay and regression verification

**Files:**
- Test only: `recordings/hitter_ball_20260715-143511_xyz_vx.csv`
- Verify: all modified paths from Tasks 1-3

**Interfaces:**
- Consumes: final real-world estimator, confirmation, planner, and lifecycle.
- Produces: evidence that the recorded real ball confirms before the arm boundary and all existing targeted tests remain green.

- [ ] **Step 1: Replay the current CSV at 100 Hz planner cadence**

Run a read-only replay using the existing `BallStateEstimator`, `IncomingTrackConfirmation`, and
`StrikePlanner`. Assert the third confirmation occurs with `time_to_strike > 0.90` and that a later
plan enters `[0.80, 0.90]`.

Expected evidence from the current recording: third confirmation near elapsed `0.291 s` with TTS
about `1.218 s`, followed by an arm-band plan near elapsed `0.495 s` with TTS about `0.896 s`.

- [ ] **Step 2: Run the full requested targeted regression**

Run:

```bash
PYTHONPATH=deploy conda run --no-capture-output -n rb python -m unittest -v \
  tests.test_hitter_env_lifecycle \
  tests.test_hitter_realtime \
  tests.test_real_world_hitter_snapshots \
  tests.test_hitter_planner_boundaries
```

Expected: all tests pass, with the previous 91 tests plus the new stable-confirmation tests.

- [ ] **Step 3: Check formatting and exact change scope**

Run:

```bash
git diff --check
git status --short
git diff -- \
  deploy/simulator/real_world.py \
  deploy/utils/hitter_realtime.py \
  deploy/envs/hitter.py \
  deploy/config/mimic/hitter.yaml \
  deploy/tests/test_real_world_hitter_snapshots.py \
  deploy/tests/test_hitter_realtime.py \
  deploy/tests/test_hitter_env_lifecycle.py
```

Expected: `git diff --check` has no output; the scoped diff contains no action gating, waiting observation changes, or unrelated edits introduced by this implementation.
