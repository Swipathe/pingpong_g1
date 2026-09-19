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

