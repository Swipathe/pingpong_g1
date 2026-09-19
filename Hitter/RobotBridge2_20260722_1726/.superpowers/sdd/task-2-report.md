# 任务 2 实施报告：MuJoCo 两次发球计划

## 状态

- 实施状态：完成，未提交、未暂存。
- 开发方式：严格先增加 `TwoServeScheduleTests` 并观察预期失败，再最小修改配置与 `HitterEnv` 得到通过结果。
- 修改范围：`deploy/config/mimic/hitter.yaml`、`deploy/envs/hitter.py`、`deploy/tests/test_mujoco_physical_table_tennis.py` 以及本报告。
- 既有脏改动：同文件中任务 1 接口、解析式物理删除和 planner 高度阈值等改动均原样保留。

## RED 证据

命令（工作目录 `deploy/`）：

```bash
conda run --no-capture-output -n rb \
  python -m unittest \
  tests.test_mujoco_physical_table_tennis.TwoServeScheduleTests -v
```

原始失败摘要：

```text
test_command_reset_flag_cannot_create_an_unscheduled_serve ... ERROR
test_hitter_config_defines_two_mujoco_serves ... ERROR
test_mujoco_planner_result_runs_the_serve_schedule ... ERROR
test_real_backend_does_not_enable_mujoco_schedule ... ERROR
test_schedule_serves_at_zero_and_twenty_seconds_only ... ERROR

KeyError: 'mujoco_serve_times_s'
AttributeError: 'HitterEnv' object has no attribute '_init_mujoco_serve_schedule_state'

Ran 5 tests in 0.343s
FAILED (errors=5)
exit_code=1
```

RED 判定：失败原因与缺失功能一致；配置键和三个调度方法尚不存在，不是测试拼写、导入或依赖错误。

## GREEN 证据

同一任务命令首次通过摘要：

```text
test_command_reset_flag_cannot_create_an_unscheduled_serve ... ok
test_hitter_config_defines_two_mujoco_serves ... ok
test_mujoco_planner_result_runs_the_serve_schedule ... ok
test_real_backend_does_not_enable_mujoco_schedule ... ok
test_schedule_serves_at_zero_and_twenty_seconds_only ... ok

Ran 5 tests in 0.326s
OK
exit_code=0
```

强化“计划外时刻和第三次调用不覆写球状态”的断言后再次执行同一命令：`Ran 5 tests in 0.355s`，`OK`，`exit_code=0`。

## planner 接入断言

brief 示例中的时序测试直接调用 `_update_mujoco_serve_schedule()`，不能单独证明生产 planner 路径已接入。因此新增 `test_mujoco_planner_result_runs_the_serve_schedule`：

- 使用真实 MuJoCo model/data 和 `HitterEnv._mujoco_planner_result()`；
- 将 `now` 设为 `1000/2000/3000`，同时将 `mujoco_data.time` 设为 `2.0/21.999/22.0`，证明调度只依据 MuJoCo 仿真时间；
- 在第二球未到期前设置 lifecycle reset 标志并扰动球位置，确认 planner 路径只清除标志，不调用计划外 reset、不增加 track epoch、不覆写球状态；
- 到期时确认第二球事件进入 track epoch 2。

该断言只扩大测试证据，不扩大生产行为范围。

## 完整回归

命令（工作目录 `deploy/`）：

```bash
conda run --no-capture-output -n rb \
  python -m unittest discover -s tests -p 'test_*.py' -v
```

结果：当前工作树可发现的 `16` 个测试全部通过，`Ran 16 tests in 1.466s`，`OK`，`exit_code=0`。覆盖任务 1 发球状态捕获/恢复、解析式覆写删除、纯物理接触、随机 reset 和本任务两次发球调度。

## 语法与差异检查

语法检查：

```bash
conda run -n rb python -m py_compile \
  deploy/envs/hitter.py \
  deploy/simulator/mujoco.py \
  deploy/tests/test_mujoco_physical_table_tennis.py
```

结果：无输出，`exit_code=0`。

差异检查：

```bash
git diff --check -- \
  deploy/config/mimic/hitter.yaml \
  deploy/envs/hitter.py \
  deploy/tests/test_mujoco_physical_table_tennis.py
```

结果：无输出，`exit_code=0`。

## 自审

- 配置明确锁定两次发球时间 `[0.0, 20.0]`。
- 调度起点、到期判断和日志事件时间只读取 `simulator.mujoco_data.time`，没有使用 wall clock 或传入 planner 的 `now` 做调度。
- 第一次事件执行 reset 后调用任务 1 的 capture；第二次事件只调用任务 1 的 restore，完整复用第一球 7D qpos 与 6D qvel。
- 两次事件之间以及 `_mujoco_next_serve_index == 2` 后只清除 lifecycle reset 请求，不修改球状态；测试显式锁定这两种情况。
- 每个计划事件将 `_hitter_sync_track_epoch` 加一，并将 `_hitter_sync_generation` 清零；planner 随后正常生成当前 epoch 的下一代 snapshot。
- 计划模式通过 `_mujoco_planner_result()` 的互斥分支屏蔽 `_reset_hitter_ball_sequence_if_needed()`；未配置 schedule 时仍保留旧路径。
- `is_real=True` 时 schedule 明确禁用；真机 planner worker 路径和 estimator reset 路径未修改。
- 没有修改任务 1 的 `capture_hitter_ball_launch_state()` / `restore_hitter_ball_launch_state()`，仅消费它们。
- 没有执行 `git add`、`git commit`、reset、checkout 或清理；所有无关脏改动均保留。

## 关注项

无已知实现问题。当前完整回归范围受工作树中既有测试删除状态限制，只运行了当前可发现的 16 个测试；未恢复或改动这些无关删除。

## 控制器审查修复

控制器审查发现 hard reset 会同时重置发球配额并通过 `Mujoco.calibrate(refresh=True)` 覆写球状态。此次追加测试与最小修复覆盖该调用链，并将配置从“任意递增序列”收紧为精确两次事件。

### 追加 RED 证据

在只增加四类测试、尚未修改生产代码时，执行 Task 2 命令。原始摘要：

```text
test_calibrate_preserves_current_ball_during_scheduled_run ... FAIL
test_missing_schedule_uses_legacy_lifecycle_reset ... ok
test_reinitializing_schedule_preserves_completed_serve_quota ... FAIL
test_schedule_rejects_a_third_event ... FAIL

AssertionError: Arrays are not equal
x: array([1.8 , 0.  , 1.05, 1.  , 0.  , 0.  , 0.  ])
y: array([ 0.7, -0.1, 1.4, 0.5, 0.5, 0.5, 0.5])
AssertionError: True is not false
AssertionError: ValueError not raised

Ran 9 tests in 0.719s
FAILED (failures=3)
exit_code=1
```

RED 判定：三个失败分别证明校准覆写当前飞行球、重复 schedule 初始化产生第三次发球、三事件配置未被拒绝；未配置 schedule 的 legacy reset/epoch characterization 测试保持通过。

### 追加 GREEN 证据

最小生产修复后首次执行同一命令：

```text
Ran 9 tests in 0.612s
OK
exit_code=0
```

完成真机 flag 与机器人关节仍被校准的补充断言后，fresh Task 2 验证为：

```text
Ran 9 tests in 0.774s
OK
exit_code=0
```

### 修复说明

- `_init_mujoco_serve_schedule_state()` 每次仍校验配置并刷新 backend 保球 flag，但只在状态字段尚不存在时初始化 origin/index，因此同一进程内的 `env.reset()` 与 hard reset 不会恢复发球配额。
- 配置存在时必须精确等于 `(0.0, 20.0)`；`[0.0, 20.0, 40.0]` 以及任何其他序列均抛出 `ValueError`。配置缺失仍产生空 schedule 并保留旧 lifecycle reset 路径。
- 计划模式给 simulator 设置 `preserve_hitter_ball_state_on_calibrate=True`；真机 backend 明确为 false。
- `Mujoco.calibrate(refresh=True)` 仅在该 flag 生效且球 freejoint 有效时，于同一 viewer lock 内保存当前球 7D qpos/6D qvel，完成机器人全量校准后原样回填并执行 `mj_forward`。测试确认机器人关节仍恢复默认值，球状态不变。
- 该回填只发生在 env calibration 离散事件中，没有在发球后的物理 step 中新增任何状态覆写，也没有调用 launch restore 形成额外发球。

### 最终 fresh 验证

- Task 2：`Ran 9 tests in 0.774s`，`OK`，`exit_code=0`。
- 完整 discover：`Ran 20 tests in 1.855s`，`OK`，`exit_code=0`。
- `py_compile`：`deploy/envs/hitter.py`、`deploy/simulator/mujoco.py`、`deploy/tests/test_mujoco_physical_table_tennis.py` 无输出，`exit_code=0`。
- `git diff --check`：配置、环境、MuJoCo simulator 与测试四个相关文件无输出，`exit_code=0`。
- 未执行暂存、提交、reset、checkout 或清理；既有无关脏改动保持原状。

### 最终关注项

无已知实现问题。当前完整 discover 范围仍受工作树中既有测试删除状态限制，可发现并通过 20 个测试；未恢复或修改这些无关删除。

## 启动时序审查修复

真实 `HitterAgent` 的调用顺序为 `env.reset()`、首次推理、`env.step()`。原实现只在 step 后的 planner 路径更新 schedule，导致首球在仿真时间 `0.020 s` 才 reset/capture。

### 启动时序 RED

只新增 public reset 回归测试后执行：

```bash
conda run --no-capture-output -n rb \
  python -m unittest \
  tests.test_mujoco_physical_table_tennis.TwoServeScheduleTests.test_public_reset_serves_before_the_first_physics_step -v
```

原始结果：

```text
MuJoCo HITTER serve 1/2 at simulation time 0.020 s.
AssertionError:
  reset 后 next_index=0，launch 未捕获；
  0.020 s 调用返回 True，并覆写扰动后的 qpos/qvel。
Ran 1 test in 0.118s
FAILED (failures=1)
exit_code=1
```

### 启动时序 GREEN

最小修复仅在 `HitterEnv.reset()` 的 `super().reset()` 完成后、返回初始 observation 前，对启用的 MuJoCo schedule 调用一次 `_update_mujoco_serve_schedule()`。聚焦测试结果：

```text
MuJoCo HITTER serve 1/2 at simulation time 0.000 s.
Ran 1 test in 0.110s
OK
exit_code=0
```

测试使用真实 `HitterEnv.reset → BaseEnv.reset → Mujoco.calibrate/get_state` 与真实 MuJoCo model/data，确认：

- reset 返回时 `mujoco_data.time == 0.0`、`next_index == 1`；
- 第一球完整 7D qpos/6D qvel launch 已捕获；
- 随后 `0.020 s` 的 scheduler 调用返回 false，不 reset、不覆写扰动球状态；
- schedule guard 对真机与未配置 legacy 路径无影响；既有测试继续锁定 hard reset 不恢复配额及第二球相对 20 秒恰好一次。

### 启动时序最终验证

- `TwoServeScheduleTests`：`Ran 10 tests in 0.971s`，`OK`，`exit_code=0`。
- 聚焦完整文件：`Ran 21 tests in 2.215s`，`OK`，`exit_code=0`。
- `py_compile deploy/envs/hitter.py deploy/tests/test_mujoco_physical_table_tennis.py`：无输出，`exit_code=0`。
- `git diff --check -- deploy/envs/hitter.py deploy/tests/test_mujoco_physical_table_tennis.py`：无输出，`exit_code=0`。
- 未提交、未暂存、未回退或修改无关工作树内容。

### 启动时序关注项

无已知实现问题。
