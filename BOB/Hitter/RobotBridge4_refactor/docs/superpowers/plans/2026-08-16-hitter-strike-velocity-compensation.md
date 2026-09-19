# HITTER Strike Velocity Compensation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 RobotBridge4 当前生产配置调整为真机正手世界系目标球拍 `vx +0.20 m/s`、反手 `vy -0.30 m/s`，并在真机和 MuJoCo 共用配置中禁用反手边缘落点覆盖。

**Architecture:** 保留现有 `HitterEnv` 真机分侧速度补偿和 `StrikePlanner` 可选落点覆盖能力，只修改生产 YAML 的配置值与配置合同测试。planner 原始命令继续不可变；速度补偿只在真机 ONNX observation 组装时生效，落点覆盖因生产配置项缺失而回落到 factory 的 `None` 默认值。

**Tech Stack:** Python 3、OmegaConf/Hydra、NumPy、pytest、RobotBridge4 HITTER runtime。

## Global Constraints

- 只修改 `/home/loco1/BOB/Hitter/RobotBridge4`；RobotBridge2 和 RobotBridge3 不动。
- 保留 model10595、Vicon 标定、正反手自动分侧、base-target nominal y、速度分量范围及现有脏树改动。
- 不停止、重启或控制当前真机进程；配置不支持热加载。
- 生产配置最终值必须为正手 `0.20 m/s`、反手 y decrement `0.30 m/s`。
- 从共享 `ball_planner` 配置中删除 `backhand_edge_landing_threshold_y_w_m` 和 `backhand_edge_landing_target_y_w_m`；不删除 planner 的通用实现。
- 由于目标文件含用户已有未提交改动，本任务不自动提交实现文件，只用精确 diff 交付变更。

---

### Task 1: 锁定生产配置的新速度与落点合同

**Files:**
- Modify: `deploy/tests/test_hitter_forehand_policy_vx_offset.py:16-134`
- Modify: `deploy/tests/test_hitter_runtime_factory.py:394-406`
- Test: `deploy/tests/test_hitter_forehand_policy_vx_offset.py`
- Test: `deploy/tests/test_hitter_runtime_factory.py`

**Interfaces:**
- Consumes: `HitterEnv._forehand_policy_vx_offset_from_policy_cfg(mapping) -> float`、`HitterEnv._backhand_policy_vy_decrement_from_policy_cfg(mapping) -> float`、`HitterEnv._policy_racket_target_velocity_w(vector, strike_type=...) -> tuple[np.ndarray, float, float]`、`build_hitter_system_planner(mapping) -> HitterSystemPlanner`。
- Produces: 对生产 YAML 的四项可观察合同：正手 offset `0.20`、反手 decrement `0.30`、两项 direct landing override key 缺失、边缘反手仍使用通用落点 `y=0.0`。

- [ ] **Step 1: 修改正手生产配置测试，使其先期望 `0.20`**

将两个生产配置读取断言从 `0.25` 改为 `0.20`，并加入真实行为测试：

```python
def test_real_forehand_policy_velocity_adds_production_offset():
    mimic_config = OmegaConf.to_container(
        OmegaConf.load(DEPLOY_DIR / "config/mimic/hitter.yaml"),
        resolve=True,
    )
    env = HitterEnv.__new__(HitterEnv)
    env.simulator = SimpleNamespace(is_real=True)
    env.forehand_policy_vx_offset_mps = (
        HitterEnv._forehand_policy_vx_offset_from_policy_cfg(
            mimic_config["policy"]
        )
    )

    policy_velocity, policy_vx_offset, policy_vy_offset = (
        env._policy_racket_target_velocity_w(
            [1.0, -0.2, 0.3],
            strike_type="forehand",
        )
    )

    np.testing.assert_allclose(policy_velocity, [1.2, -0.2, 0.3])
    assert policy_vx_offset == 0.2
    assert policy_vy_offset == 0.0
```

- [ ] **Step 2: 修改反手生产配置测试，使其先期望 `0.30` 和实际 `vy-0.30`**

将生产配置读取断言从 `0.0` 改为 `0.30`，把原“生产零补偿保持不变”测试重命名并改成：

```python
def test_real_backhand_policy_velocity_subtracts_production_decrement():
    mimic_config = OmegaConf.to_container(
        OmegaConf.load(DEPLOY_DIR / "config/mimic/hitter.yaml"),
        resolve=True,
    )
    env = HitterEnv.__new__(HitterEnv)
    env.simulator = SimpleNamespace(is_real=True)
    env.backhand_policy_vy_decrement_mps = (
        HitterEnv._backhand_policy_vy_decrement_from_policy_cfg(
            mimic_config["policy"]
        )
    )

    policy_velocity, policy_vx_offset, policy_vy_offset = (
        env._policy_racket_target_velocity_w(
            [1.0, -0.2, 0.3],
            strike_type="backhand",
        )
    )

    np.testing.assert_allclose(policy_velocity, [1.0, -0.5, 0.3])
    assert policy_vx_offset == 0.0
    assert policy_vy_offset == -0.3
```

- [ ] **Step 3: 修改生产 planner 合同测试，使其要求落点覆盖 key 缺失且行为禁用**

把原 `test_yaml_planner_resolves_direct_backhand_landing_override_values` 拆成以下两个测试：

```python
def test_yaml_planner_disables_direct_backhand_landing_override():
    self.assertNotIn(
        "backhand_edge_landing_threshold_y_w_m",
        self.planner_config,
    )
    self.assertNotIn(
        "backhand_edge_landing_target_y_w_m",
        self.planner_config,
    )
    planner = build_hitter_system_planner(self.planner_config).strike_planner
    self.assertEqual(planner.backhand_edge_landing_target_y_w_m, None)

def test_yaml_planner_keeps_default_landing_for_edge_backhand():
    planner = build_hitter_system_planner(self.planner_config).strike_planner
    landing = planner.desired_landing_point_for_strike(
        [0.0, 0.50, 1.0],
        strike_type="backhand",
    )
    np.testing.assert_allclose(landing, [2.05, 0.0, 0.78])
```

- [ ] **Step 4: 运行 RED 测试并确认只因旧生产 YAML 失败**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4/deploy
PYTHONDONTWRITEBYTECODE=1 conda run --no-capture-output -n rb \
  python -m pytest -q \
  tests/test_hitter_forehand_policy_vx_offset.py \
  tests/test_hitter_runtime_factory.py
```

Expected: FAIL；失败值必须对应旧配置 `0.25`、`0.0`、仍存在的两项 landing override key，或旧边缘落点 `-0.30`，不能是 import/语法错误。

### Task 2: 最小修改生产 YAML 并恢复 GREEN

**Files:**
- Modify: `deploy/config/mimic/hitter.yaml:4-5`
- Modify: `deploy/config/mimic/hitter.yaml:32-33`
- Test: `deploy/tests/test_hitter_forehand_policy_vx_offset.py`
- Test: `deploy/tests/test_hitter_runtime_factory.py`
- Test: `deploy/tests/test_hitter_backhand_edge_landing_bias.py`

**Interfaces:**
- Consumes: Task 1 的生产配置合同测试。
- Produces: Hydra 可解析的 production mapping，其中 `real_world_forehand_racket_velocity_x_offset_mps=0.20`、`real_world_backhand_racket_velocity_y_decrement_mps=0.30`，且 direct landing override 两项 key 不存在。

- [ ] **Step 1: 只修改四处 YAML 行**

目标片段：

```yaml
policy:
  real_world_forehand_racket_velocity_x_offset_mps: 0.20
  real_world_backhand_racket_velocity_y_decrement_mps: 0.30
```

并从 `motion.ball_planner` 删除：

```yaml
backhand_edge_landing_threshold_y_w_m: 0.20
backhand_edge_landing_target_y_w_m: -0.30
```

- [ ] **Step 2: 运行 GREEN 合同测试**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4/deploy
PYTHONDONTWRITEBYTECODE=1 conda run --no-capture-output -n rb \
  python -m pytest -q \
  tests/test_hitter_forehand_policy_vx_offset.py \
  tests/test_hitter_runtime_factory.py
```

Expected: PASS，0 failures。

- [ ] **Step 3: 运行相关 planner、集成与日志回归**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4/deploy
PYTHONDONTWRITEBYTECODE=1 conda run --no-capture-output -n rb \
  python -m pytest -q \
  tests/test_hitter_backhand_edge_landing_bias.py \
  tests/test_hitter_runtime_single_shot_integration.py \
  tests/test_hitter_strike_target_logging.py
```

Expected: PASS；通用 landing override feature 的显式参数测试仍通过，真机/仿真分侧补偿和不累计合同保持通过。

### Task 3: 验证 Hydra 真机解析与交付边界

**Files:**
- Verify: `deploy/config/mimic/hitter.yaml`
- Verify: `deploy/tests/test_hitter_forehand_policy_vx_offset.py`
- Verify: `deploy/tests/test_hitter_runtime_factory.py`
- Verify: `docs/superpowers/specs/2026-08-16-hitter-strike-velocity-compensation-design.md`

**Interfaces:**
- Consumes: Task 2 的生产配置。
- Produces: 可复制的 RobotBridge4 三终端真机命令，以及当前运行进程仍使用旧启动时配置的明确说明。

- [ ] **Step 1: 解析最终 Hydra 真机配置**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4/deploy
set -o pipefail
conda run --no-capture-output -n rb \
  python run.py --config-name=hitter sim=real_world device=cpu \
  --cfg job --resolve | \
rg 'checkpoint:|real_world_forehand_racket_velocity_x_offset_mps|real_world_backhand_racket_velocity_y_decrement_mps|backhand_edge_landing_(threshold|target)_y_w_m|force_strike_type'
```

Expected: model10595、正手 `0.2`、反手 `0.3`、`force_strike_type: null`；不得出现两项 direct landing override key。

- [ ] **Step 2: 检查精确 diff 和格式**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
git diff --check -- \
  deploy/config/mimic/hitter.yaml \
  deploy/tests/test_hitter_forehand_policy_vx_offset.py \
  deploy/tests/test_hitter_runtime_factory.py
git diff -- \
  deploy/config/mimic/hitter.yaml \
  deploy/tests/test_hitter_forehand_policy_vx_offset.py \
  deploy/tests/test_hitter_runtime_factory.py
```

Expected: 无 whitespace error；本需求新增差异仅为两项速度数值、删除两项 landing override 配置，以及对应测试期望。目标文件相对 HEAD 的其他既有脏改动必须保留。

- [ ] **Step 3: 检查进程状态但不操作进程**

Run:

```bash
pgrep -af 'vicon_table_lcm_bridge_v2|\.build-robotbridge4-v2/bin/trans|python.*run.py.*config-name=hitter'
```

Expected: 只读报告当前进程。若旧 policy 仍在运行，明确说明它不会热加载新 YAML，用户需按真机安全顺序自行重启后才生效。

- [ ] **Step 4: 交付最新三终端命令**

复用当前已验证的 RobotBridge4 v2 Vicon bridge、`.build-robotbridge4-v2/bin/trans` 和 `run.py --config-name=hitter sim=real_world device=cpu` 命令；不在本任务中实际执行它们。
