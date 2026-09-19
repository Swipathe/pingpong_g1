# HITTER Next Incoming-Ball Epoch Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Advance the real-world ball track at the predicted strike deadline so the next incoming ball can be planned without observing an outgoing trajectory.

**Architecture:** Keep the pure lifecycle unchanged. `HitterEnv` observes the existing `ARMED -> RECOVERY` transition and calls the existing real-world estimator reset exactly once; `RealWorld.reset_ball_state_estimator()` already clears samples and increments `ball_track_epoch`. MuJoCo and full-swing recovery behavior remain unchanged.

**Tech Stack:** Python 3, `unittest`, NumPy, existing HITTER lifecycle and RealWorld estimator APIs.

## Global Constraints

- Do not require an outgoing-ball observation.
- Do not add a ball-data timeout or action gating.
- Preserve the configured full swing and recovery duration.
- Keep the existing outgoing-to-incoming detector as secondary estimator cleanup.

---

### Task 1: Advance the real-world track at the strike deadline

**Files:**
- Modify: `deploy/envs/hitter.py:852-898`
- Test: `deploy/tests/test_hitter_env_lifecycle.py`

**Interfaces:**
- Consumes: `RealWorld.reset_ball_state_estimator() -> None`, which clears estimator state and increments `ball_track_epoch`.
- Produces: one reset call when `_update_hitter_command()` observes `CommandPhase.ARMED` transition to a later phase.

- [ ] **Step 1: Write the failing reset-on-strike tests**

Add tests that arm a real-world environment, set `simulator.ball_track_epoch` to the active epoch, cross the strike deadline, and assert:

```python
env._update_hitter_command(now=10.90)
self.assertEqual(env.hitter_command_lifecycle.phase, CommandPhase.RECOVERY)
self.assertEqual(env.simulator.reset_calls, 1)
self.assertEqual(env.simulator.ball_track_epoch, 3)

env._update_hitter_command(now=11.0)
self.assertEqual(env.simulator.reset_calls, 1)
```

Also inject an epoch-3 incoming command during recovery and assert it is cached and armed after the existing full-swing deadline.

- [ ] **Step 2: Run the focused tests and verify RED**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2
PYTHONPATH=deploy conda run --no-capture-output -n rb \
  python -m unittest -v tests.test_hitter_env_lifecycle.HitterEnvLifecycleTests.test_strike_deadline_advances_real_track_once
```

Expected: FAIL because `reset_calls` remains `0`.

- [ ] **Step 3: Implement the minimal transition hook**

Immediately after `HitterCommandLifecycle.advance(now)`, detect a previous
`ARMED` phase that is no longer `ARMED`. For real-world backends, call the
existing estimator reset once. If it is missing, log a warning without
interrupting recovery:

```python
crossed_strike = (
    previous_phase == CommandPhase.ARMED
    and phase_after_advance != CommandPhase.ARMED
)
if crossed_strike and getattr(self.simulator, "is_real", False):
    reset_estimator = getattr(self.simulator, "reset_ball_state_estimator", None)
    if callable(reset_estimator):
        reset_estimator()
    else:
        logger.warning(
            "Real-world HITTER cannot start the next ball track: "
            "reset_ball_state_estimator() is unavailable."
        )
```

- [ ] **Step 4: Run focused and related tests and verify GREEN**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2
PYTHONPATH=deploy conda run --no-capture-output -n rb python -m unittest -v \
  tests.test_hitter_env_lifecycle \
  tests.test_hitter_realtime \
  tests.test_real_world_hitter_snapshots \
  tests.test_hitter_planner_boundaries
```

Expected: all tests pass with zero failures and the real-world reset occurs exactly once per strike.

- [ ] **Step 5: Review the scoped diff**

Run:

```bash
git diff --check
git diff -- deploy/envs/hitter.py deploy/tests/test_hitter_env_lifecycle.py
```

Expected: only the strike-transition hook and its tests are present, with no whitespace errors.
