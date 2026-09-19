### Task 1: Isolated waiting-arm return and MuJoCo command

**Files:**
- Create: `deploy/tests/test_hitter_waiting_arm_return.py`
- Modify: `deploy/envs/hitter.py`
- Modify: `deploy/config/mimic/hitter.yaml`
- Create: `docs/superpowers/plans/2026-07-27-hitter-ready-pose-mujoco-isolated.md`

**Interfaces:**
- Consumes: `HitterEnv.step(action)`, `CommandPhase`, `simulator.dof_names`, `simulator.high_dt`, `self.dof_pos`.
- Produces: `HitterEnv._configure_waiting_arm_return()`, `HitterEnv._reset_waiting_arm_return_state(clear_last_target: bool)`, and `HitterEnv._apply_waiting_arm_return(sim_action: np.ndarray) -> np.ndarray`.

- [ ] **Step 1: Write focused failing tests**

Create a real `HitterEnv.__new__(HitterEnv)` fixture with a minimal simulator whose `dof_names` contain the 15 lower-body/waist names followed by the exact 14 arm names, `high_dt=0.1`, and a 29-D finite `dof_pos`. Call the production configuration and application methods directly.

The tests must cover these observable contracts with hand-derived values:

```python
def test_waiting_smoothstep_changes_only_sim_arm_indices(self):
    # At elapsed 0.0, output arms equal the captured start.
    # With duration 0.9 and high_dt 0.1, after the state has accumulated
    # 0.45 s, smoothstep(0.5) == 0.5.
    # Lower-body indices 0..14 always equal the current policy PD target.
    # At 0.9 s and afterward, indices 15..28 equal the literal target.

def test_tracking_continues_but_armed_and_recovery_cancel(self):
    # WAITING -> TRACKING keeps the same captured start and elapsed state.
    # ARMED and RECOVERY return all 29 current policy values unchanged and
    # clear transition state.
    # Re-entering WAITING captures the most recently dispatched target.

def test_startup_and_reset_capture_current_dof_position(self):
    # With no previous dispatched target, first WAITING output uses dof_pos.
    # _prepare_hitter_reset_state clears the prior target and transition.

def test_waiting_return_configuration_fails_closed(self):
    # Reject a non-mapping block, unknown/missing block keys, non-bool enabled,
    # non-numeric/bool/non-finite/non-positive duration, missing/extra joint
    # names, bool/string/non-finite target values, missing simulator joints,
    # non-15..28 mapping, and non-finite/non-positive high_dt.

def test_missing_or_disabled_block_preserves_policy_target(self):
    # No block is backward-compatible and disabled mode is bitwise passthrough.
```

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/.worktrees/hitter-ready-pose-mujoco-test-20260727/deploy
env -u DISPLAY PYTHONDONTWRITEBYTECODE=1 MUJOCO_GL=egl \
  /home/loco1/miniconda3/envs/rb/bin/python \
  -m unittest tests.test_hitter_waiting_arm_return -v
```

Expected: fail because the three production methods do not exist.

- [ ] **Step 2: Implement strict configuration and deterministic state**

Add a module-level tuple with the exact 14 arm names. In `__init__`, after `motion_cfg` is available, call `_configure_waiting_arm_return()`.

`_configure_waiting_arm_return()` must:

```python
raw = self.motion_cfg.get("waiting_arm_return")
# raw is absent -> disabled backward-compatible state.
# A present block has exactly enabled, duration_s, target_joint_pos.
# Validate concrete bool/numeric/mapping types and finite values.
# Resolve each name against simulator.dof_names.
# Require resolved indices == np.arange(15, 29).
# Store a float32 target in the same 14-name order.
# Validate finite positive simulator.high_dt when enabled.
```

`_reset_waiting_arm_return_state(clear_last_target)` clears the captured start and elapsed time. It clears `_last_dispatched_sim_pd_target` only when requested.

- [ ] **Step 3: Implement the WAITING/TRACKING smoothstep**

Immediately after converting the policy PD target to simulation order in `step()`:

```python
sim_action = self._apply_waiting_arm_return(sim_action)
self._last_dispatched_sim_pd_target = sim_action.copy()
```

`_apply_waiting_arm_return()` must copy its input. Disabled mode returns it unchanged. ARMED/RECOVERY clear transition state and return all 29 values unchanged. WAITING/TRACKING capture a start only once, then apply:

```python
u = np.clip(elapsed_s / duration_s, 0.0, 1.0)
blend = u * u * (3.0 - 2.0 * u)
result[arm_indices] = (
    (1.0 - blend) * captured_start_arm
    + blend * target_arm
)
elapsed_s = min(elapsed_s + high_dt, duration_s)
```

Require shape-compatible, finite input and start vectors. The first hold-phase output uses `u=0`. In `_prepare_hitter_reset_state()`, reset the transition with `clear_last_target=True`.

- [ ] **Step 4: Add the exact isolated Hydra configuration**

Under `motion` in `deploy/config/mimic/hitter.yaml`, add:

```yaml
  waiting_arm_return:
    enabled: true
    duration_s: 0.9
    target_joint_pos:
      left_shoulder_pitch_joint: 0.2
      left_shoulder_roll_joint: 0.2
      left_shoulder_yaw_joint: 0.0
      left_elbow_joint: 0.6
      left_wrist_roll_joint: 0.0
      left_wrist_pitch_joint: 0.0
      left_wrist_yaw_joint: 0.0
      right_shoulder_pitch_joint: -0.5946002267035698
      right_shoulder_roll_joint: -0.2
      right_shoulder_yaw_joint: 0.0
      right_elbow_joint: 0.6
      right_wrist_roll_joint: 0.0
      right_wrist_pitch_joint: 0.0
      right_wrist_yaw_joint: 0.0
```

- [ ] **Step 5: Verify RED becomes GREEN and run regressions**

Run the focused test command from Step 1, then:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/.worktrees/hitter-ready-pose-mujoco-test-20260727/deploy
env -u DISPLAY PYTHONDONTWRITEBYTECODE=1 MUJOCO_GL=egl \
  /home/loco1/miniconda3/envs/rb/bin/python \
  -m unittest discover -s tests -p 'test_*.py'
```

Expected: focused tests pass; the current isolated baseline of 238 tests plus the new tests has zero failures, with only the existing environment-dependent skips.

- [ ] **Step 6: Verify composition, syntax, isolation, and command inputs**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/.worktrees/hitter-ready-pose-mujoco-test-20260727/deploy
env PYTHONDONTWRITEBYTECODE=1 \
  /home/loco1/miniconda3/envs/rb/bin/python -m py_compile envs/hitter.py \
  tests/test_hitter_waiting_arm_return.py
/home/loco1/miniconda3/envs/rb/bin/python -u run.py \
  --config-name=hitter sim=mujoco device=cpu --cfg job
test -f /home/loco1/BOB/Hitter/RobotBridge2/deploy/data/model/hitter/hitter_model8000_oldmotion_104_20260725.onnx
git diff --check
```

Confirm the composed config contains the exact 14 values, duration `0.9`, `enabled: true`, `simulator.mujoco.Mujoco`, viewer enabled, and video disabled. Confirm the original main checkout has the same pre-task status and no new pose-test edits.

- [ ] **Step 7: Commit the isolated implementation**

```bash
git add \
  deploy/envs/hitter.py \
  deploy/config/mimic/hitter.yaml \
  deploy/tests/test_hitter_waiting_arm_return.py \
  docs/superpowers/plans/2026-07-27-hitter-ready-pose-mujoco-isolated.md
git commit -m "feat: add isolated HITTER waiting ready pose"
```

The ignored saved pose artifacts remain uncommitted and byte-identical to their source.
