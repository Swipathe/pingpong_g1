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

