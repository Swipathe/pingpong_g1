# HITTER Real-World Policy First Frame Transition Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 RobotBridge3 真机第二次 R2 放行后，用 2.0 秒从当前实测关节姿态平滑进入 policy 第一帧目标，之后恢复原有 policy 循环。

**Architecture:** 过渡逻辑放在 `HitterEnv.step()` 内，只在 `simulator.is_real` 且 reset 后第一次 policy step 时触发。第一帧 action 仍按现有 `action_beta/default/action_scale` 计算成绝对关节目标，然后冻结该目标并用 smoothstep 在真机 policy 周期内阻塞式下发。

**Tech Stack:** Python, NumPy, OmegaConf, pytest, RobotBridge3 existing deploy runtime.

## Global Constraints

- 所有修改必须限制在 `/home/loco1/BOB/Hitter/RobotBridge3` 内。
- 不修改 RobotBridge2、MOSAIC-main 或其他仓库。
- 不修改第一次 R2 默认姿态校准。
- 不修改第二次 R2 前的 `G2Pelvis` 有效性检查。
- 不修改 `RealWorld.apply_action()`、`unitree_sdk2/trans.cpp`、全局 PD 增益、模型 action scale 或正常 policy 行为。
- 仅 `simulator.is_real == true` 且 `policy.real_world_first_frame_transition_s > 0.0` 时启用。
- `deploy/config/mimic/hitter.yaml` 中 `policy.real_world_first_frame_transition_s` 必须为 `2.0`。
- 缺省配置值为 `0.0`，其他 HITTER 配置未显式启用时保持旧行为。
- 过渡目标只使用第二次 R2 后第一次 ONNX action 计算出的 policy 第一帧绝对关节目标。
- 过渡期间不请求或采用第二帧 ONNX 输出。
- 每次 reset 后重新 armed，一轮 policy entry 只执行一次过渡。

---

## File Structure

- Modify: `deploy/envs/hitter.py`
  - Parse and validate `policy.real_world_first_frame_transition_s`.
  - Track one-shot pending state across reset.
  - Provide small helper methods for real-world-only enablement, start-state validation, smoothstep interpolation, and blocking first-frame transition.
  - Route only the first real-world policy step through transition; all other `step()` calls keep the existing path.
- Modify: `deploy/config/mimic/hitter.yaml`
  - Add `real_world_first_frame_transition_s: 2.0` under `policy`.
- Create: `deploy/tests/test_hitter_policy_first_frame_transition.py`
  - Unit-test transition behavior using a fake simulator and `HitterEnv.__new__` to avoid hardware, MuJoCo, or full config initialization.

---

### Task 1: Add Focused Transition Tests

**Files:**
- Create: `deploy/tests/test_hitter_policy_first_frame_transition.py`

**Interfaces:**
- Consumes: current `HitterEnv.step(action)` behavior and private helpers to be implemented in Task 2:
  - `HitterEnv._reset_real_world_first_frame_transition_pending() -> None`
  - `HitterEnv._real_world_first_frame_transition_enabled() -> bool`
  - `HitterEnv._run_real_world_first_frame_transition(sim_action: np.ndarray) -> None`
- Produces: pytest coverage that fails before implementation and verifies exact scope.

- [ ] **Step 1: Write the failing test file**

Create `deploy/tests/test_hitter_policy_first_frame_transition.py` with:

```python
import time
from types import SimpleNamespace

import numpy as np
import pytest

from envs.hitter import HitterEnv


class FakeRealWorldSimulator:
    is_real = True
    high_dt = 0.02

    def __init__(self, dof_pos):
        self.dof_pos = np.asarray(dof_pos, dtype=np.float32)
        self.applied_actions = []
        self.get_state_calls = 0

    def get_state(self):
        self.get_state_calls += 1

    def apply_action(self, action):
        self.applied_actions.append(np.asarray(action, dtype=np.float32).copy())


class FakeMujocoSimulator(FakeRealWorldSimulator):
    is_real = False


def make_env(simulator, transition_s=2.0):
    env = HitterEnv.__new__(HitterEnv)
    env.simulator = simulator
    env.policy_model_meta = object()
    env._policy_dim = 3
    env.action_beta = 1.0
    env.prev_policy_action = np.zeros(3, dtype=np.float32)
    env.policy_action_scales = np.array([0.1, 0.2, 0.3], dtype=np.float32)
    env.policy_default_joint_pos = np.array([1.0, 2.0, 3.0], dtype=np.float32)
    env._policy_to_sim = np.array([0, 1, 2], dtype=np.int32)
    env.save_video_enabled = False
    env.real_world_first_frame_transition_s = transition_s
    env._real_world_first_frame_transition_pending = False
    env._reset_real_world_first_frame_transition_pending()
    env.action_clip_value = 10000.0
    env.episode_length_buf = np.zeros(1, dtype=np.int32)
    env.obs_buf_dict = {"actor_obs": np.zeros((1, 1), dtype=np.float32)}
    env.compute_observation = lambda: None
    return env


def test_real_world_first_step_interpolates_to_frozen_first_policy_target(monkeypatch):
    sim = FakeRealWorldSimulator(dof_pos=[0.0, 0.0, 0.0])
    env = make_env(sim, transition_s=2.0)
    sleeps = []
    now = [100.0]

    monkeypatch.setattr(time, "monotonic", lambda: now[0])

    def fake_sleep(duration):
        sleeps.append(duration)
        now[0] += duration

    monkeypatch.setattr(time, "sleep", fake_sleep)

    obs = env.step(np.array([1.0, -1.0, 0.5], dtype=np.float32))

    target = np.array([1.1, 1.8, 3.15], dtype=np.float32)
    assert obs is env.obs_buf_dict
    assert sim.get_state_calls == 1
    assert len(sim.applied_actions) == 100
    assert all(action.shape == (1, 3) for action in sim.applied_actions)
    assert np.all(np.isfinite(np.stack(sim.applied_actions)))
    assert np.allclose(sim.applied_actions[-1][0], target)

    first_alpha = 3 * (1 / 100) ** 2 - 2 * (1 / 100) ** 3
    assert np.allclose(sim.applied_actions[0][0], first_alpha * target)
    assert np.all(np.diff([action[0, 0] for action in sim.applied_actions]) > 0.0)
    assert env._real_world_first_frame_transition_pending is False
    assert env.episode_length_buf[0] == 1
    assert np.allclose(env.prev_policy_action, [1.0, -1.0, 0.5])
    assert len(sleeps) == 100
    assert all(duration >= 0.0 for duration in sleeps)


def test_real_world_transition_runs_once_then_normal_step(monkeypatch):
    sim = FakeRealWorldSimulator(dof_pos=[0.0, 0.0, 0.0])
    env = make_env(sim, transition_s=2.0)
    monkeypatch.setattr(time, "monotonic", lambda: 0.0)
    monkeypatch.setattr(time, "sleep", lambda duration: None)

    env.step(np.array([1.0, 0.0, 0.0], dtype=np.float32))
    first_count = len(sim.applied_actions)

    env.step(np.array([2.0, 0.0, 0.0], dtype=np.float32))

    assert first_count == 100
    assert len(sim.applied_actions) == 101
    assert np.allclose(sim.applied_actions[-1][0], [1.2, 2.0, 3.0])
    assert sim.get_state_calls == 1


def test_reset_rearms_real_world_transition(monkeypatch):
    sim = FakeRealWorldSimulator(dof_pos=[0.0, 0.0, 0.0])
    env = make_env(sim, transition_s=2.0)
    monkeypatch.setattr(time, "monotonic", lambda: 0.0)
    monkeypatch.setattr(time, "sleep", lambda duration: None)

    env.step(np.array([1.0, 0.0, 0.0], dtype=np.float32))
    env._reset_real_world_first_frame_transition_pending()
    sim.dof_pos = np.array([1.0, 2.0, 3.0], dtype=np.float32)
    env.step(np.array([0.0, 1.0, 0.0], dtype=np.float32))

    assert len(sim.applied_actions) == 200
    assert np.allclose(sim.applied_actions[-1][0], [1.0, 2.2, 3.0])


def test_mujoco_never_runs_first_frame_transition_even_if_configured():
    sim = FakeMujocoSimulator(dof_pos=[0.0, 0.0, 0.0])
    env = make_env(sim, transition_s=2.0)

    env.step(np.array([1.0, 0.0, 0.0], dtype=np.float32))

    assert len(sim.applied_actions) == 1
    assert sim.get_state_calls == 0


@pytest.mark.parametrize("transition_s", [0.0, -0.0])
def test_zero_duration_keeps_old_single_apply_behavior(transition_s):
    sim = FakeRealWorldSimulator(dof_pos=[0.0, 0.0, 0.0])
    env = make_env(sim, transition_s=transition_s)

    env.step(np.array([1.0, 0.0, 0.0], dtype=np.float32))

    assert len(sim.applied_actions) == 1
    assert sim.get_state_calls == 0


@pytest.mark.parametrize("transition_s", [-0.1, float("nan"), float("inf")])
def test_invalid_transition_duration_is_rejected_before_motion(transition_s):
    sim = FakeRealWorldSimulator(dof_pos=[0.0, 0.0, 0.0])
    env = HitterEnv.__new__(HitterEnv)
    env.simulator = sim
    env.real_world_first_frame_transition_s = transition_s

    with pytest.raises(ValueError):
        env._reset_real_world_first_frame_transition_pending()

    assert sim.applied_actions == []


def test_invalid_measured_start_fails_before_motion(monkeypatch):
    sim = FakeRealWorldSimulator(dof_pos=[0.0, float("nan"), 0.0])
    env = make_env(sim, transition_s=2.0)
    monkeypatch.setattr(time, "sleep", lambda duration: None)

    with pytest.raises(ValueError):
        env.step(np.array([1.0, 0.0, 0.0], dtype=np.float32))

    assert sim.applied_actions == []


def test_invalid_first_policy_target_fails_before_motion(monkeypatch):
    sim = FakeRealWorldSimulator(dof_pos=[0.0, 0.0, 0.0])
    env = make_env(sim, transition_s=2.0)
    monkeypatch.setattr(time, "sleep", lambda duration: None)

    with pytest.raises(ValueError):
        env.step(np.array([float("nan"), 0.0, 0.0], dtype=np.float32))

    assert sim.applied_actions == []
```

- [ ] **Step 2: Run the focused tests to verify they fail**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge3
pytest deploy/tests/test_hitter_policy_first_frame_transition.py -q
```

Expected before Task 2: FAIL because `_reset_real_world_first_frame_transition_pending` and transition helpers do not exist.

- [ ] **Step 3: Commit only the new failing tests**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge3
git add deploy/tests/test_hitter_policy_first_frame_transition.py
git commit -m "test: cover HITTER real-world policy first-frame transition"
```

---

### Task 2: Implement Real-World First-Frame Transition

**Files:**
- Modify: `deploy/envs/hitter.py`
- Modify: `deploy/config/mimic/hitter.yaml`
- Test: `deploy/tests/test_hitter_policy_first_frame_transition.py`

**Interfaces:**
- Consumes: Task 1 tests.
- Produces:
  - `HitterEnv.real_world_first_frame_transition_s: float`
  - `HitterEnv._real_world_first_frame_transition_pending: bool`
  - `HitterEnv._reset_real_world_first_frame_transition_pending() -> None`
  - `HitterEnv._real_world_first_frame_transition_enabled() -> bool`
  - `HitterEnv._run_real_world_first_frame_transition(sim_action: np.ndarray) -> None`

- [ ] **Step 1: Add config parsing and reset state**

In `deploy/envs/hitter.py`, after `self.action_beta = float(...)`, add:

```python
self.real_world_first_frame_transition_s = float(
    self.policy_cfg.get("real_world_first_frame_transition_s", 0.0)
)
self._real_world_first_frame_transition_pending = False
```

In `_prepare_hitter_reset_state()`, after resetting `prev_policy_action`, add:

```python
self._reset_real_world_first_frame_transition_pending()
```

Add this method near `_prepare_hitter_reset_state()`:

```python
def _reset_real_world_first_frame_transition_pending(self) -> None:
    duration_s = float(self.real_world_first_frame_transition_s)
    if not np.isfinite(duration_s) or duration_s < 0.0:
        raise ValueError(
            "policy.real_world_first_frame_transition_s must be a finite non-negative duration in seconds."
        )
    self.real_world_first_frame_transition_s = duration_s
    self._real_world_first_frame_transition_pending = (
        duration_s > 0.0 and bool(getattr(self.simulator, "is_real", False))
    )
```

- [ ] **Step 2: Add real-world transition helper methods**

In `deploy/envs/hitter.py`, near the reset helpers, add:

```python
def _real_world_first_frame_transition_enabled(self) -> bool:
    return (
        bool(getattr(self.simulator, "is_real", False))
        and bool(self._real_world_first_frame_transition_pending)
        and float(self.real_world_first_frame_transition_s) > 0.0
    )

def _run_real_world_first_frame_transition(self, sim_action: np.ndarray) -> None:
    target = np.asarray(sim_action, dtype=np.float32).reshape(-1)
    if not np.all(np.isfinite(target)):
        raise ValueError("real-world first-frame policy target contains non-finite values.")

    high_dt = float(self.simulator.high_dt)
    if not np.isfinite(high_dt) or high_dt <= 0.0:
        raise ValueError("simulator.high_dt must be a finite positive duration.")

    self.simulator.get_state()
    start = np.asarray(self.simulator.dof_pos, dtype=np.float32).reshape(-1)
    if start.shape != target.shape:
        raise ValueError(
            f"real-world first-frame transition start shape {start.shape} does not match target shape {target.shape}."
        )
    if not np.all(np.isfinite(start)):
        raise ValueError("real-world first-frame transition start contains non-finite values.")

    steps = max(1, int(np.ceil(float(self.real_world_first_frame_transition_s) / high_dt)))
    for step_idx in range(1, steps + 1):
        step_start_s = time.monotonic()
        u = step_idx / steps
        alpha = 3.0 * u * u - 2.0 * u * u * u
        q_cmd = ((1.0 - alpha) * start + alpha * target).astype(np.float32)
        self.simulator.apply_action(q_cmd[None, ...])
        elapsed_s = time.monotonic() - step_start_s
        time.sleep(max(0.0, high_dt - elapsed_s))

    self._real_world_first_frame_transition_pending = False
```

- [ ] **Step 3: Route the first real-world policy step through transition**

Replace the final line in `HitterEnv.step()`:

```python
return super().step(sim_action[None, ...])
```

with:

```python
sim_action_batch = sim_action[None, ...]
if self._real_world_first_frame_transition_enabled():
    self._pre_physics_step(sim_action_batch)
    self._run_real_world_first_frame_transition(self.action.reshape(-1))
    self._post_physics_step()
    return self.obs_buf_dict

return super().step(sim_action_batch)
```

This keeps one `episode_length_buf` increment and one observation update for the first policy frame, while avoiding an extra duplicate `apply_action()` after the 100 interpolation commands.

- [ ] **Step 4: Enable the 2.0 second transition in the HITTER mimic config**

In `deploy/config/mimic/hitter.yaml`, update the `policy` block to:

```yaml
policy:
  checkpoint: ./data/model/hitter/hitter_model7400_vx0to4_vyneg14to14_vz0to4_tts030to092_posstd012_velstd2_velwin004_104_20260806.onnx
  action_beta: 1.0
  real_world_first_frame_transition_s: 2.0
  hitter_seed: 0
  save_video: false
```

- [ ] **Step 5: Run focused tests**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge3
pytest deploy/tests/test_hitter_policy_first_frame_transition.py -q
```

Expected: all tests in this file PASS.

- [ ] **Step 6: Run existing adjacent HITTER env/runtime tests that still exist**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge3
pytest deploy/tests/test_hitter_runtime_factory.py deploy/tests/test_hitter_task_input_adapter.py -q
```

Expected: PASS, or report exact unrelated pre-existing failures without hiding them.

- [ ] **Step 7: Check diff scope**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge3
git diff -- deploy/envs/hitter.py deploy/config/mimic/hitter.yaml deploy/tests/test_hitter_policy_first_frame_transition.py
git diff --check -- deploy/envs/hitter.py deploy/config/mimic/hitter.yaml deploy/tests/test_hitter_policy_first_frame_transition.py
```

Expected: diff only contains the requested RobotBridge3 first-frame transition change, and `git diff --check` has no output.

- [ ] **Step 8: Commit implementation**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge3
git add deploy/envs/hitter.py deploy/config/mimic/hitter.yaml
git commit -m "fix: smooth HITTER real-world policy first frame"
```

Do not stage unrelated dirty files.

---

## Self-Review

- Spec coverage: Task 1 and Task 2 cover real-world-only enablement, 2.0 second duration, frozen first target, reset re-arm, old MuJoCo behavior, old zero-duration behavior, invalid inputs before motion, and no RobotBridge2 changes.
- Placeholder scan: no forbidden placeholder patterns or unspecified test instructions remain.
- Type consistency: helper names in Task 1 match Task 2 exactly; `sim_action` is a 1-D NumPy vector inside `_run_real_world_first_frame_transition`, and simulator receives `(1, dof)` actions.
