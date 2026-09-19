# MuJoCo 两次发球实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: 使用 `superpowers:subagent-driven-development`（推荐）或 `superpowers:executing-plans`，按任务逐项实施并在每个 RED/GREEN 节点保存证据。

**目标：** 在一次 MuJoCo 仿真中只发两次初始状态完全相同的球，分别位于相对 MuJoCo 仿真时间 `0.0 s` 和 `20.0 s`。

**架构：** `HitterEnv` 负责读取发球计划、推进计划索引并同步 planner 的 track epoch；`Mujoco` 只提供保存和恢复完整球 `qpos/qvel` 的接口。计划模式下，击球指令生命周期不得直接重置物理球；未配置计划时继续使用原有按指令重置行为。

**技术栈：** Python 3.8、MuJoCo Python API、NumPy、Hydra/OmegaConf、`unittest`。

## 全局约束

- 所有新增文档和注释使用中文。
- 时间源只能是 `mujoco_data.time`，不得使用墙钟时间、线程或定时器。
- 发球后不得覆写飞行中球的位置或速度；反弹和击球继续完全由 MuJoCo 接触动力学计算。
- 第二球必须复用第一球的位置、姿态、线速度和角速度。
- 只允许两个发球事件，不允许指令结束或规划失败触发第三球。
- 只影响 MuJoCo 后端，真机路径保持不变。
- 保留工作区全部无关修改；生产文件已有用户改动，禁止整文件覆盖或回退。

---

### Task 1：提供球初始状态快照与恢复接口

**文件：**
- 修改：`deploy/simulator/mujoco.py:98-246`
- 测试：`deploy/tests/test_mujoco_physical_table_tennis.py`

**接口：**
- 产生：`Mujoco.capture_hitter_ball_launch_state() -> bool`
- 产生：`Mujoco.restore_hitter_ball_launch_state() -> bool`
- 保证：快照包含自由关节的 7 个 `qpos` 和 6 个 `qvel`，恢复后执行 `mujoco.mj_forward()` 并刷新公开球状态。

- [ ] **步骤 1：先写失败测试**

在 `deploy/tests/test_mujoco_physical_table_tennis.py` 增加：

```python
class BallLaunchStateTests(unittest.TestCase):
    def test_capture_and_restore_reproduces_complete_launch_state(self):
        simulator = _minimal_table_tennis_simulator({"enabled": True})
        simulator.reset_hitter_ball(
            pos=[1.8, 0.1, 1.05],
            lin_vel=[-2.2, 0.2, 0.3],
        )
        qposadr = simulator.hitter_ball_qposadr
        qveladr = simulator.hitter_ball_qveladr
        simulator.mujoco_data.qvel[qveladr + 3:qveladr + 6] = [1.0, 2.0, 3.0]
        mujoco.mj_forward(simulator.mujoco_model, simulator.mujoco_data)

        expected_qpos = simulator.mujoco_data.qpos[qposadr:qposadr + 7].copy()
        expected_qvel = simulator.mujoco_data.qvel[qveladr:qveladr + 6].copy()
        self.assertTrue(simulator.capture_hitter_ball_launch_state())

        simulator.mujoco_data.qpos[qposadr:qposadr + 3] = [0.2, -0.4, 0.3]
        simulator.mujoco_data.qvel[qveladr:qveladr + 6] = 0.0
        mujoco.mj_forward(simulator.mujoco_model, simulator.mujoco_data)

        self.assertTrue(simulator.restore_hitter_ball_launch_state())
        np.testing.assert_array_equal(
            simulator.mujoco_data.qpos[qposadr:qposadr + 7], expected_qpos
        )
        np.testing.assert_array_equal(
            simulator.mujoco_data.qvel[qveladr:qveladr + 6], expected_qvel
        )
```

- [ ] **步骤 2：运行测试并确认 RED**

运行：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
conda run --no-capture-output -n rb \
  python -m unittest \
  tests.test_mujoco_physical_table_tennis.BallLaunchStateTests -v
```

预期：因 `capture_hitter_ball_launch_state` 尚不存在而失败。

- [ ] **步骤 3：实现最小快照接口**

在 `_init_table_tennis_state()` 中初始化：

```python
self._hitter_ball_launch_qpos = None
self._hitter_ball_launch_qvel = None
```

在 `reset_hitter_ball()` 后增加两个方法：

```python
def capture_hitter_ball_launch_state(self) -> bool:
    if (
        not self.table_tennis_enabled
        or self.hitter_ball_qposadr is None
        or self.hitter_ball_qveladr is None
    ):
        return False
    qposadr = self.hitter_ball_qposadr
    qveladr = self.hitter_ball_qveladr
    with self._viewer_lock():
        self._hitter_ball_launch_qpos = self.mujoco_data.qpos[
            qposadr:qposadr + 7
        ].copy()
        self._hitter_ball_launch_qvel = self.mujoco_data.qvel[
            qveladr:qveladr + 6
        ].copy()
    return True

def restore_hitter_ball_launch_state(self) -> bool:
    if (
        not self.table_tennis_enabled
        or self.hitter_ball_qposadr is None
        or self.hitter_ball_qveladr is None
        or self._hitter_ball_launch_qpos is None
        or self._hitter_ball_launch_qvel is None
    ):
        return False
    qposadr = self.hitter_ball_qposadr
    qveladr = self.hitter_ball_qveladr
    with self._viewer_lock():
        self.mujoco_data.qpos[qposadr:qposadr + 7] = self._hitter_ball_launch_qpos
        self.mujoco_data.qvel[qveladr:qveladr + 6] = self._hitter_ball_launch_qvel
        mujoco.mj_forward(self.mujoco_model, self.mujoco_data)
    self._update_hitter_ball_state()
    return True
```

- [ ] **步骤 4：运行测试并确认 GREEN**

运行任务 1 的测试命令。预期：`BallLaunchStateTests` 全部通过。

- [ ] **步骤 5：检查任务 1 差异**

```bash
git diff --check -- \
  deploy/simulator/mujoco.py \
  deploy/tests/test_mujoco_physical_table_tennis.py
git diff -- \
  deploy/simulator/mujoco.py \
  deploy/tests/test_mujoco_physical_table_tennis.py
```

预期：仅出现快照接口和对应测试；不得提交或覆盖文件中的既有用户改动。

---

### Task 2：实现基于 MuJoCo 时间的两次发球计划

**文件：**
- 修改：`deploy/config/mimic/hitter.yaml:5-12`
- 修改：`deploy/envs/hitter.py:238-356`
- 修改：`deploy/envs/hitter.py:512-517`
- 修改：`deploy/envs/hitter.py:660-665`
- 测试：`deploy/tests/test_mujoco_physical_table_tennis.py`

**接口：**
- 消费：`Mujoco.capture_hitter_ball_launch_state() -> bool`
- 消费：`Mujoco.restore_hitter_ball_launch_state() -> bool`
- 产生：`HitterEnv._init_mujoco_serve_schedule_state() -> None`
- 产生：`HitterEnv._mujoco_serve_schedule_enabled() -> bool`
- 产生：`HitterEnv._update_mujoco_serve_schedule() -> bool`

- [ ] **步骤 1：先写配置与时序失败测试**

在测试文件增加辅助函数和测试：

```python
def _scheduled_serve_env(simulator: Mujoco) -> HitterEnv:
    env = HitterEnv.__new__(HitterEnv)
    env.simulator = simulator
    env.motion_cfg = {"mujoco_serve_times_s": [0.0, 20.0]}
    env.hitter_ball_sequence_needs_reset = True
    env._hitter_sync_track_epoch = 0
    env._hitter_sync_generation = 7
    env._init_mujoco_serve_schedule_state()
    return env


class TwoServeScheduleTests(unittest.TestCase):
    def test_hitter_config_defines_two_mujoco_serves(self):
        config = yaml.safe_load(
            (DEPLOY_DIR / "config" / "mimic" / "hitter.yaml").read_text()
        )
        self.assertEqual(config["motion"]["mujoco_serve_times_s"], [0.0, 20.0])

    def test_schedule_serves_at_zero_and_twenty_seconds_only(self):
        simulator = _minimal_table_tennis_simulator({"enabled": True})
        simulator.is_real = False
        env = _scheduled_serve_env(simulator)

        simulator.mujoco_data.time = 3.0
        self.assertTrue(env._update_mujoco_serve_schedule())
        qposadr = simulator.hitter_ball_qposadr
        qveladr = simulator.hitter_ball_qveladr
        first_qpos = simulator.mujoco_data.qpos[qposadr:qposadr + 7].copy()
        first_qvel = simulator.mujoco_data.qvel[qveladr:qveladr + 6].copy()

        simulator.mujoco_data.qpos[qposadr:qposadr + 3] = [0.1, 0.2, 0.3]
        simulator.mujoco_data.qvel[qveladr:qveladr + 6] = 0.0
        simulator.mujoco_data.time = 22.999
        self.assertFalse(env._update_mujoco_serve_schedule())
        self.assertEqual(env._mujoco_next_serve_index, 1)

        simulator.mujoco_data.time = 23.0
        self.assertTrue(env._update_mujoco_serve_schedule())
        np.testing.assert_array_equal(
            simulator.mujoco_data.qpos[qposadr:qposadr + 7], first_qpos
        )
        np.testing.assert_array_equal(
            simulator.mujoco_data.qvel[qveladr:qveladr + 6], first_qvel
        )
        self.assertEqual(env._hitter_sync_track_epoch, 2)
        self.assertEqual(env._hitter_sync_generation, 0)

        simulator.mujoco_data.time = 43.0
        self.assertFalse(env._update_mujoco_serve_schedule())
        self.assertEqual(env._mujoco_next_serve_index, 2)

    def test_command_reset_flag_cannot_create_an_unscheduled_serve(self):
        simulator = _minimal_table_tennis_simulator({"enabled": True})
        simulator.is_real = False
        env = _scheduled_serve_env(simulator)
        simulator.mujoco_data.time = 0.0
        self.assertTrue(env._update_mujoco_serve_schedule())

        env.hitter_ball_sequence_needs_reset = True
        simulator.mujoco_data.time = 5.0
        self.assertFalse(env._update_mujoco_serve_schedule())
        self.assertFalse(env.hitter_ball_sequence_needs_reset)
        self.assertEqual(env._mujoco_next_serve_index, 1)

    def test_real_backend_does_not_enable_mujoco_schedule(self):
        simulator = SimpleNamespace(is_real=True)
        env = _scheduled_serve_env(simulator)
        self.assertFalse(env._mujoco_serve_schedule_enabled())
```

- [ ] **步骤 2：运行测试并确认 RED**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
conda run --no-capture-output -n rb \
  python -m unittest \
  tests.test_mujoco_physical_table_tennis.TwoServeScheduleTests -v
```

预期：配置字段和三个时序方法尚不存在，测试失败。

- [ ] **步骤 3：增加配置**

在 `deploy/config/mimic/hitter.yaml` 的 `motion` 下增加：

```yaml
mujoco_serve_times_s: [0.0, 20.0]
```

- [ ] **步骤 4：实现计划状态与校验**

在 `_init_hitter_command_state()` 末尾调用：

```python
self._init_mujoco_serve_schedule_state()
```

并增加：

```python
def _init_mujoco_serve_schedule_state(self) -> None:
    raw_times = self.motion_cfg.get("mujoco_serve_times_s", None)
    if raw_times is None:
        serve_times = ()
    else:
        serve_times = tuple(float(value) for value in raw_times)
        if any(not np.isfinite(value) or value < 0.0 for value in serve_times):
            raise ValueError("mujoco_serve_times_s must contain finite non-negative values.")
        if any(
            current <= previous
            for previous, current in zip(serve_times, serve_times[1:])
        ):
            raise ValueError("mujoco_serve_times_s must be strictly increasing.")
        if serve_times and abs(serve_times[0]) > 1.0e-12:
            raise ValueError("mujoco_serve_times_s must start at 0.0.")
    self._mujoco_serve_times_s = serve_times
    self._mujoco_serve_origin_time_s = None
    self._mujoco_next_serve_index = 0

def _mujoco_serve_schedule_enabled(self) -> bool:
    return bool(self._mujoco_serve_times_s) and not bool(
        getattr(self.simulator, "is_real", False)
    )
```

- [ ] **步骤 5：实现两个发球事件**

增加：

```python
def _update_mujoco_serve_schedule(self) -> bool:
    if not self._mujoco_serve_schedule_enabled():
        return False
    data = getattr(self.simulator, "mujoco_data", None)
    if data is None:
        raise RuntimeError("MuJoCo serve schedule requires mujoco_data.")
    sim_time_s = float(data.time)
    if not np.isfinite(sim_time_s):
        raise RuntimeError("MuJoCo simulation time is not finite.")
    if self._mujoco_serve_origin_time_s is None:
        self._mujoco_serve_origin_time_s = sim_time_s
    if self._mujoco_next_serve_index >= len(self._mujoco_serve_times_s):
        self.hitter_ball_sequence_needs_reset = False
        return False

    elapsed_s = sim_time_s - self._mujoco_serve_origin_time_s
    due_s = self._mujoco_serve_times_s[self._mujoco_next_serve_index]
    if elapsed_s + 1.0e-12 < due_s:
        self.hitter_ball_sequence_needs_reset = False
        return False

    if self._mujoco_next_serve_index == 0:
        self.simulator.reset_hitter_ball()
        if not self.simulator.capture_hitter_ball_launch_state():
            raise RuntimeError("Failed to capture the first MuJoCo ball launch state.")
    elif not self.simulator.restore_hitter_ball_launch_state():
        raise RuntimeError("Failed to restore the MuJoCo ball launch state.")

    self._mujoco_next_serve_index += 1
    self._hitter_sync_track_epoch += 1
    self._hitter_sync_generation = 0
    self.hitter_ball_sequence_needs_reset = False
    logger.info(
        "MuJoCo HITTER serve {}/{} at simulation time {:.3f} s.",
        self._mujoco_next_serve_index,
        len(self._mujoco_serve_times_s),
        sim_time_s,
    )
    return True
```

- [ ] **步骤 6：接入 planner 同步路径并屏蔽计划外重发**

将 `_mujoco_planner_result()` 开头改为：

```python
if self._mujoco_serve_schedule_enabled():
    self._update_mujoco_serve_schedule()
elif self.hitter_ball_sequence_needs_reset:
    self._reset_hitter_ball_sequence_if_needed()
    self._hitter_sync_track_epoch += 1
    self._hitter_sync_generation = 0
```

计划模式不调用 `_reset_hitter_ball_sequence_if_needed()`，因此指令结束或 planner 跳过结果只会留下一个随后被清除的请求标志，不会产生物理重发。

- [ ] **步骤 7：运行任务 2 测试并确认 GREEN**

运行任务 2 的测试命令。预期：`TwoServeScheduleTests` 全部通过。

- [ ] **步骤 8：检查任务 2 差异**

```bash
git diff --check -- \
  deploy/config/mimic/hitter.yaml \
  deploy/envs/hitter.py \
  deploy/tests/test_mujoco_physical_table_tennis.py
```

预期：无空白错误；既有 planner 阈值、真机路径和纯物理接触逻辑不发生无关变化。

---

### Task 3：回归验证与可视化冒烟检查

**文件：**
- 验证：`deploy/config/mimic/hitter.yaml`
- 验证：`deploy/simulator/mujoco.py`
- 验证：`deploy/envs/hitter.py`
- 验证：`deploy/tests/test_mujoco_physical_table_tennis.py`

**接口：**
- 消费：任务 1 和任务 2 的全部接口。
- 产生：自动化测试、Hydra 配置组合和可视化日志证据。

- [ ] **步骤 1：运行完整聚焦测试**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
conda run --no-capture-output -n rb \
  python -m unittest tests.test_mujoco_physical_table_tennis -v
```

预期：该文件全部测试通过，包括已有纯物理接触、随机轨迹以及新增两次发球测试。

- [ ] **步骤 2：验证 Hydra 组合配置**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
conda run --no-capture-output -n rb \
  python -u run.py --config-name=hitter sim=mujoco device=cpu --cfg job \
  | rg -n "mujoco_serve_times_s|_target_: simulator.mujoco.Mujoco|enabled: true"
```

预期：输出包含 `mujoco_serve_times_s: [0.0, 20.0]`、MuJoCo target 和启用的 table tennis 配置。

- [ ] **步骤 3：运行语法与差异检查**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2
conda run --no-capture-output -n rb python -m py_compile \
  deploy/simulator/mujoco.py \
  deploy/envs/hitter.py \
  deploy/tests/test_mujoco_physical_table_tennis.py
git diff --check
git status --short
```

预期：编译和 diff check 成功；`git status` 中无关脏修改与实施前保持一致。

- [ ] **步骤 4：可视化冒烟检查**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
DISPLAY=:1 LOGURU_LEVEL=INFO \
conda run --no-capture-output -n rb \
python -u run.py \
  --config-name=hitter \
  sim=mujoco \
  device=cpu \
  robot.control.viewer=true \
  robot.control.real_time=true
```

预期日志恰好出现：

```text
MuJoCo HITTER serve 1/2 at simulation time ...
MuJoCo HITTER serve 2/2 at simulation time ...
```

两行仿真时间之差为约 `20.0 s`；之后继续观察至少 2 秒，不出现 `serve 3/2` 或第三次球状态重置。按 `Ctrl+C` 结束冒烟检查。

- [ ] **步骤 5：形成交付摘要**

报告修改文件、RED 失败原因、GREEN 测试数量、Hydra 组合结果和可视化两次发球时间。由于这些生产文件包含实施前已有修改，除非能够按 hunk 精确隔离，否则不创建包含既有用户改动的代码提交。
