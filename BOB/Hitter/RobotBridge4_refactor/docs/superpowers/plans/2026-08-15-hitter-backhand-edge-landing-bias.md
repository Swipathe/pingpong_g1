# RobotBridge4 Backhand Edge Landing Bias Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 对左侧边缘反手平滑施加最大 `-0.10 m` 的 planner 落点 Y 偏置，并关闭生产配置中的旧 policy `vy - 0.3` 双重补偿。

**Architecture:** `StrikePlanner` 根据击球点 Y 和最终击球类型计算一次有效落点，再沿现有 shooting 与碰撞模型完整生成 `v_ball_out` 和 `v_racket_target`。三个显式配置字段经 runtime factory 注入；未配置或最大减量为零时保持旧行为。

**Tech Stack:** Python 3、NumPy、OmegaConf/Hydra、pytest。

## Global Constraints

- 只修改 `/home/loco1/BOB/Hitter/RobotBridge4`。
- 保留当前 dirty worktree，不还原或覆盖无关改动，不使用宽泛 `git add`。
- 基础落点保持 `[2.05, 0.0, 0.78]`，虚拟击球面保持 `x=0.0`。
- 反手边缘规则固定为 start `0.30 m`、full `0.50 m`、最大 Y decrement `0.10 m`。
- 使用 C1 smoothstep `3u^2-2u^3`；正手和阈值以下反手不变。
- 生产配置把 `real_world_backhand_racket_velocity_y_decrement_mps` 设为 `0.0`，不得与新偏置叠加。

---

### Task 1: 用测试定义 planner 的有效落点与完整重算

**Files:**
- Create: `deploy/tests/test_hitter_backhand_edge_landing_bias.py`
- Modify: `deploy/utils/hitter_planner.py:596-954`
- Modify: `deploy/utils/hitter_planner.py:1042-1086`

**Interfaces:**
- Produces: `StrikePlanner.desired_landing_point_for_strike(strike_position, *, strike_type=None) -> np.ndarray`
- Produces: `StrikePlanner.plan(ball_position, ball_velocity, *, strike_type=None) -> StrikePlan`
- Consumes: existing `strike_type_from_table_y()` and `validate_explicit_strike_type()` semantics.

- [ ] **Step 1: Write failing behavior tests**

Create a deterministic planner fixture whose `hit_plane_intersection()` returns a selected strike Y. Assert literal effective target Y values:

```python
@pytest.mark.parametrize(
    ("strike_y", "strike_type", "expected_y"),
    [
        (-0.50, "forehand", 0.0),
        (0.30, "backhand", 0.0),
        (0.40, "backhand", -0.05),
        (0.50, "backhand", -0.10),
        (0.70, "backhand", -0.10),
    ],
)
def test_effective_landing_y_is_smooth_and_bounded(...):
    ...
```

Add a full-plan test that integrates `plan.v_ball_out` for `0.48 s` with `_free_flight_endpoint()` and asserts the endpoint equals `[2.05, -0.10, 0.78]` within `2e-5 m`. Add a disabled/default test comparing output with an unmodified planner. Add validation tests for non-finite/negative values, `full <= start`, `full > table_width/2`, and an out-of-table final landing Y.

- [ ] **Step 2: Run tests and verify RED**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4/deploy
conda run --no-capture-output -n rb pytest -q tests/test_hitter_backhand_edge_landing_bias.py
```

Expected: collection or constructor failures because the three constructor parameters and `desired_landing_point_for_strike()` do not exist.

- [ ] **Step 3: Implement the minimum planner behavior**

Extend `StrikePlanner.__init__()` with:

```python
backhand_edge_landing_start_y_w_m: float = 0.30,
backhand_edge_landing_full_y_w_m: float = 0.50,
backhand_edge_landing_y_decrement_m: float = 0.0,
```

When enabled, validate the fields against `predictor.table_width` and the base landing point. Implement:

```python
u = np.clip((strike_y - start) / (full - start), 0.0, 1.0)
weight = u * u * (3.0 - 2.0 * u)
target[1] -= decrement * weight
```

Resolve the strike type before calculating `v_ball_out`; pass explicit forced type through `HitterSystemPlanner.plan_command()` when present. Use the effective target for both the shooting initial guess and every endpoint error correction.

- [ ] **Step 4: Run the new tests and verify GREEN**

Run the Step 2 command. Expected: all tests pass.

---

### Task 2: Wire production configuration and disable the old policy offset

**Files:**
- Modify: `deploy/config/mimic/hitter.yaml:1-35`
- Modify: `deploy/utils/hitter_runtime_factory.py:118-155`
- Modify: `deploy/tests/test_hitter_runtime_factory.py:81-192,224-278`
- Modify: `deploy/tests/test_hitter_forehand_policy_vx_offset.py:80-106`

**Interfaces:**
- Consumes: the three new `StrikePlanner` constructor keyword arguments from Task 1.
- Produces: resolved production planner values `0.30`, `0.50`, `0.10`; production policy Y decrement `0.0`.

- [ ] **Step 1: Write failing factory/config tests**

Update the deployment-config expectation from `0.3` to `0.0`. Extend the runtime factory reference builder and equality assertion with the three new fields, then assert the production YAML resolves to the literal tuple `(0.30, 0.50, 0.10)`. Add a real-world policy adapter assertion proving a backhand input `[1.0, -0.2, 0.3]` remains unchanged when the production decrement is zero.

- [ ] **Step 2: Run tests and verify RED**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4/deploy
conda run --no-capture-output -n rb pytest -q \
  tests/test_hitter_runtime_factory.py \
  tests/test_hitter_forehand_policy_vx_offset.py
```

Expected: failures because YAML still resolves the old `0.3` decrement and the factory does not pass the new fields.

- [ ] **Step 3: Apply production wiring**

Set the policy decrement to `0.0`, add the three ball-planner values directly after `desired_landing_point_w`, and pass them from `planner_config` to `StrikePlanner`. Preserve constructor defaults when fields are absent.

- [ ] **Step 4: Run focused regression tests**

Run Task 1 and Task 2 test commands together. Expected: all pass with zero failures.

- [ ] **Step 5: Verify the resolved real-world configuration**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4/deploy
conda run --no-capture-output -n rb python - <<'PY'
from omegaconf import OmegaConf
cfg = OmegaConf.load('config/mimic/hitter.yaml')
print(cfg.policy.real_world_backhand_racket_velocity_y_decrement_mps)
print(cfg.motion.ball_planner.backhand_edge_landing_start_y_w_m)
print(cfg.motion.ball_planner.backhand_edge_landing_full_y_w_m)
print(cfg.motion.ball_planner.backhand_edge_landing_y_decrement_m)
PY
```

Expected output is four lines: `0.0`, `0.3`, `0.5`, `0.1`.

- [ ] **Step 6: Review only scoped diffs**

Run:

```bash
git diff --check -- \
  deploy/config/mimic/hitter.yaml \
  deploy/utils/hitter_planner.py \
  deploy/utils/hitter_runtime_factory.py \
  deploy/tests/test_hitter_backhand_edge_landing_bias.py \
  deploy/tests/test_hitter_runtime_factory.py \
  deploy/tests/test_hitter_forehand_policy_vx_offset.py
git diff --stat -- <same explicit paths>
```

Expected: no whitespace errors; only the named behavior/config/test files are part of this feature review. Do not stage unrelated dirty files.
