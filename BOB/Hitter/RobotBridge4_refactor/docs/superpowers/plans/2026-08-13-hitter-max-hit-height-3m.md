# RobotBridge4 3 米击球高度上限实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 RobotBridge4 的世界坐标预测击球高度上限从 `1.45 m` 调整为 `3.00 m`。

**Architecture:** 保留现有 `table_height + maximum_hit_height_above_table_m` 计算路径，只把 RobotBridge4 当前 YAML 中的桌面以上高度配置改为 `2.24 m`。通过读取真实 YAML 并调用生产 factory 的回归测试锁定最终绝对上限，planner、Track ID 生命周期和其他门限均不改动。

**Tech Stack:** Python 3、OmegaConf、pytest、RobotBridge4 HITTER runtime factory。

## Global Constraints

- 只修改 `/home/loco1/BOB/Hitter/RobotBridge4`，RobotBridge3 不变。
- `table_height` 保持 `0.760 m`。
- `maximum_hit_height_above_table_m` 设置为 `2.24 m`，解析后的绝对上限必须精确为 `3.0 m`。
- 允许的预测击球高度区间变为 `(0.760, 3.000] m`。
- 不修改 Track ID、状态机、正反手、速度限制、击球时间或任何其他运行参数。
- 目标文件已有用户改动；禁止整文件暂存或提交，不得把既有改动混入本任务。

---

### Task 1: 锁定并修改 RobotBridge4 的 3 米绝对高度上限

**Files:**
- Modify: `deploy/tests/test_hitter_runtime_factory.py:301`
- Modify: `deploy/config/mimic/hitter.yaml:57`

**Interfaces:**
- Consumes: `load_yaml_mapping(relative_path: str) -> dict` 和 `build_hitter_system_planner(planner_config: dict) -> HitterSystemPlanner`。
- Produces: `planner.strike_planner.maximum_hit_height == 3.0` 的 RobotBridge4 生产配置契约。

- [ ] **Step 1: 写入会在旧配置下失败的生产 YAML 回归测试**

在 `RuntimeFactoryCharacterizationTests` 中加入：

```python
def test_yaml_resolves_absolute_three_meter_hit_height(self):
    planner = build_hitter_system_planner(self.planner_config)

    self.assertEqual(
        self.planner_config["maximum_hit_height_above_table_m"],
        2.24,
    )
    self.assertEqual(
        planner.strike_planner.minimum_hit_height,
        0.76,
    )
    self.assertEqual(
        planner.strike_planner.maximum_hit_height,
        3.0,
    )
```

- [ ] **Step 2: 运行单测并确认 RED**

从 `deploy` 目录运行：

```bash
conda run --no-capture-output -n rb \
  python -m pytest \
  tests/test_hitter_runtime_factory.py::RuntimeFactoryCharacterizationTests::test_yaml_resolves_absolute_three_meter_hit_height \
  -q
```

预期：测试因当前 `maximum_hit_height_above_table_m` 仍为 `0.69` 而失败；不能因导入、路径或语法错误失败。

- [ ] **Step 3: 做最小生产配置修改**

在 `deploy/config/mimic/hitter.yaml` 中只修改：

```yaml
maximum_hit_height_above_table_m: 2.24
```

- [ ] **Step 4: 运行单测并确认 GREEN**

重复 Step 2 的命令。预期：`1 passed`。

- [ ] **Step 5: 运行相关回归测试**

从 `deploy` 目录运行：

```bash
conda run --no-capture-output -n rb \
  python -m pytest \
  tests/test_hitter_runtime_factory.py \
  tests/test_hitter_planner_failure_reasons.py \
  -q
```

预期：两个测试文件全部通过；不得出现新的 warning、error 或失败。

- [ ] **Step 6: 核对最终差异但不提交重叠文件**

```bash
git diff --check -- \
  deploy/config/mimic/hitter.yaml \
  deploy/tests/test_hitter_runtime_factory.py
git diff -- \
  deploy/config/mimic/hitter.yaml \
  deploy/tests/test_hitter_runtime_factory.py
```

确认本任务新增的内容只有一条测试和 `0.69 -> 2.24` 配置变化。由于两个目标文件已经包含用户的既有未提交修改，本任务不暂存、不提交它们。
