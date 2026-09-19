# Task 1 报告：球 launch 状态快照与恢复接口

## 结果

已按 TDD 完成 MuJoCo 乒乓球完整 launch 状态的捕获与恢复 API。实现只保存并恢复自由关节的完整状态：

- `qpos[qposadr:qposadr + 7]`
- `qvel[qveladr:qveladr + 6]`

未加入解析式反弹、击球或轨迹覆盖逻辑，也未修改 real-world 后端。未执行 `git add`、`git commit` 或 `git push`。

## RED

先新增 `BallLaunchStateTests.test_capture_and_restore_reproduces_complete_launch_state`，再运行：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
conda run --no-capture-output -n rb \
  python -m unittest \
  tests.test_mujoco_physical_table_tennis.BallLaunchStateTests -v
```

原始结果摘要：

```text
exit code: 1
test_capture_and_restore_reproduces_complete_launch_state ... ERROR
AttributeError: 'Mujoco' object has no attribute 'capture_hitter_ball_launch_state'
Ran 1 test in 0.132s
FAILED (errors=1)
```

失败原因与预期一致：生产代码尚无要求的 capture API。

## GREEN

加入最小实现后，重复相同的定向测试命令：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
conda run --no-capture-output -n rb \
  python -m unittest \
  tests.test_mujoco_physical_table_tennis.BallLaunchStateTests -v
```

原始结果摘要：

```text
exit code: 0
test_capture_and_restore_reproduces_complete_launch_state ... ok
Ran 1 test in 0.132s
OK
```

## 聚焦回归

运行完整聚焦测试文件：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
conda run --no-capture-output -n rb \
  python -m unittest tests.test_mujoco_physical_table_tennis -v
```

原始结果摘要：

```text
exit code: 0
Ran 11 tests in 1.181s
OK
```

全部 11 个测试通过，包括 launch 状态、纯物理接触、随机发球重置和解析覆盖移除测试。

## 静态验证

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
conda run --no-capture-output -n rb \
  python -m py_compile simulator/mujoco.py tests/test_mujoco_physical_table_tennis.py
```

结果：退出码 0，无输出。

```bash
git diff --check -- \
  deploy/simulator/mujoco.py \
  deploy/tests/test_mujoco_physical_table_tennis.py
```

结果：退出码 0，无空白错误。

## 修改范围

- `deploy/simulator/mujoco.py`
  - 初始化两个私有 launch 快照字段。
  - 新增 `capture_hitter_ball_launch_state()`。
  - 新增 `restore_hitter_ball_launch_state()`。
- `deploy/tests/test_mujoco_physical_table_tennis.py`
  - 新增 `BallLaunchStateTests`，验证完整 7 维 qpos 与 6 维 qvel 可精确恢复。
- `.superpowers/sdd/task-1-report.md`
  - 本报告。

工作树原本已有未提交改动，且测试文件原本为未跟踪文件；这些状态均被保留。本任务在生产文件中的新增内容仅限上述字段与两个 API。

## 自检

- 捕获使用 `.copy()`，避免快照与 MuJoCo 数据缓冲区共享引用。
- 捕获和恢复均在 viewer lock 内读写状态。
- 恢复后调用 `mujoco.mj_forward()`，并刷新公开的球状态缓存。
- table tennis 未启用、球地址缺失或尚未捕获快照时返回 `False`。
- 成功捕获或恢复时返回 `True`。
- 没有恢复或新增任何解析式球运动覆盖。
- 没有修改 real-world 后端或其他生产文件。
- 没有暂存、提交或推送。

## Fix Review Evidence

审查指出原测试只扰动位置和速度，不能证明四元数、
`mujoco.mj_forward()` 的派生状态以及公开 `ball_*` 缓存被完整恢复。
测试现已增强为：

- 初始状态和扰动状态分别使用两个不同且已归一化的 `wxyz` 四元数。
- 扰动完整 7D qpos 和完整 6D qvel。
- 恢复后精确比较 qpos 与 qvel。
- 恢复后比较球 body 的 `mujoco_data.xpos/xquat`。
- 恢复后比较 `ball_pos_world`、xyzw 顺序的 `ball_quat_world`、
  `ball_vel_world` 和 `ball_ang_vel_world`。

### Mutation evidence 1：遗漏四元数恢复

临时将生产实现改成只写回前三维位置后运行：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
conda run --no-capture-output -n rb \
  python -m unittest \
  tests.test_mujoco_physical_table_tennis.BallLaunchStateTests -v
```

失败摘要：

```text
exit code: 1
test_capture_and_restore_reproduces_complete_launch_state ... FAIL
Mismatched elements: 4 / 7 (57.1%)
x: [1.8, 0.1, 1.05, 0.707107, 0, 0, 0.707107]
y: [1.8, 0.1, 1.05, 0.5, 0.5, 0.5, 0.5]
Ran 1 test in 0.102s
FAILED (failures=1)
```

该失败证明增强测试能捕获“只恢复位置、不恢复四元数”的错误实现。

### Mutation evidence 2：遗漏公开缓存刷新

恢复完整 7D qpos 写回后，临时移除
`self._update_hitter_ball_state()`，再次运行相同目标测试。

失败摘要：

```text
exit code: 1
test_capture_and_restore_reproduces_complete_launch_state ... FAIL
Mismatched elements: 3 / 3 (100%)
x: [0.2, -0.4, 0.3]
y: [1.8, 0.1, 1.05]
Ran 1 test in 0.119s
FAILED (failures=1)
```

失败发生在 `ball_pos_world` 断言，证明增强测试能捕获恢复后未刷新公开缓存的错误实现。

### 恢复正确生产实现后的 GREEN

两处故障注入均已撤销，生产实现恢复为完整 7D qpos、6D qvel 写回，
随后调用 `mujoco.mj_forward()` 和 `_update_hitter_ball_state()`。

目标测试命令：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
conda run --no-capture-output -n rb \
  python -m unittest \
  tests.test_mujoco_physical_table_tennis.BallLaunchStateTests -v
```

结果：退出码 0，`Ran 1 test in 0.135s`，`OK`。

完整聚焦回归命令：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
conda run --no-capture-output -n rb \
  python -m unittest tests.test_mujoco_physical_table_tennis -v
```

结果：退出码 0，`Ran 11 tests in 1.078s`，`OK`。

本次审查修复的持久代码改动仅位于
`deploy/tests/test_mujoco_physical_table_tennis.py`；
`deploy/simulator/mujoco.py` 的两次修改仅用于故障注入，均已恢复。
未暂存、提交或推送。
