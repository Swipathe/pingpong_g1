# Task 3 交付报告：独立于 pelvis 的球诊断状态

## 状态

已完成实现、TDD RED/GREEN、完整 HITTER task 测试套件、自审和显式路径提交。

## 实现

- 新增不可变 `BallDiagnosticState`，结构化保存：
  - estimator 当前样本数与窗口大小；
  - 有效速度向量与标量速度；
  - diagnostics-only incoming 当前连续计数、required count、状态与 blocker。
- 新增 `BallDiagnosticTracker`：
  - 只读取 `BallEstimateSnapshot` 及对应 estimator 样本数；
  - 不读取 `base_valid` 或其他 pelvis 字段；
  - 使用现有 `HitterRuntimeSettings` 的 incoming 速度阈值和 required count；
  - 以 `(track_epoch, generation)` 去重；
  - 非 incoming 样本将真实连续计数重置为 `0`；
  - track epoch 变化、球不可见和 strike deadline reset 分别清理 diagnostics-only 状态；
  - `NOT_SEEN`、`ESTIMATING`、`TRACK_ENDED` 等未评估状态使用 `None`，不伪造 `0`。
- `ShadowTaskPipeline` 在既有 planner update throttle 通过后才观察 diagnostics snapshot。
- 既有 `plan_snapshot()`、生产 `IncomingTrackConfirmation`、planner、lifecycle、task observation 和 action 路径均未重排或改写。

## 文件

- `deploy/diagnostics/hitter_task_models.py`
- `deploy/diagnostics/hitter_task_pipeline.py`
- `deploy/tests/test_hitter_task_pipeline.py`

未修改 `deploy/envs/hitter.py`；该文件已有的 dirty-worktree 改动保持原样。

## TDD：RED

命令：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
/home/loco1/miniconda3/envs/rb/bin/python -m unittest \
  tests.test_hitter_task_pipeline
```

退出码：`1`

关键输出：

```text
FAIL: test_ball_diagnostics_reaches_estimator_window_without_pelvis
FAIL: test_ball_diagnostics_deduplicates_ready_snapshots_without_touching_production
FAIL: test_ball_diagnostics_nonincoming_sample_resets_consecutive_count
FAIL: test_ball_diagnostics_track_end_is_explicit_and_next_epoch_restarts_count
FAIL: test_tick_arms_with_same_source_fields_and_latest_pelvis_then_resets_at_strike

Ran 35 tests in 0.124s
FAILED (failures=5)
```

失败原因符合预期：pipeline 尚未暴露独立的 `ball_diagnostics` tracker。

## TDD：GREEN

使用与 RED 相同的聚焦命令。

```text
...................................
----------------------------------------------------------------------
Ran 35 tests in 0.130s

OK
```

退出码为 `0`。

## 完整测试套件

命令：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
/home/loco1/miniconda3/envs/rb/bin/python -m unittest discover \
  -s tests -p 'test_hitter_task_*.py'
```

结果：

```text
----------------------------------------------------------------------
Ran 236 tests in 23.967s

OK (skipped=2)
```

退出码为 `0`；两个 skip 是套件既有条件跳过。

## 自审

- 重复 generation：同一 `SnapshotKey(track_epoch, generation)` 第二次观察直接返回原状态；测试锁定计数序列 `1, 1, 2, 3`。
- reset 边界：
  - epoch 变化先清零再观察新 snapshot；
  - 不可见球产生 `TRACK_ENDED / BALL_NOT_VISIBLE`；
  - strike deadline reset 产生 `TRACK_ENDED / STRIKE_DEADLINE_RESET`；
  - 新 epoch 的首个 incoming snapshot 从 `1` 重新开始。
- unknown 与 zero：
  - 从未见球、estimator 尚未 ready、track 已结束时不可用计数为 `None`；
  - estimator 已执行并明确判定 non-incoming 时连续计数为真实 `0`。
- production 隔离：
  - diagnostics 在 pelvis 无效时仍推进；
  - 测试确认生产 `incoming` 仍为 `track_epoch=None, count=0, confirmed=False`；
  - `plan_snapshot()` 没有改动，diagnostics tracker 不调用 planner、不写 lifecycle、不生成 action。
- mutation 检查：
  - 删除 key 去重会破坏 `1,1,2,3` 测试；
  - 删除 epoch/reset 清零会使新 track 从 `2` 开始并失败；
  - 把未知值恢复为 `0` 会使 track-end/initial 断言失败；
  - 让 diagnostics 写入生产 incoming 会使隔离断言失败。
- `git diff --check` 和 `git diff --cached --check` 均无输出。
- 暂存和提交只使用简报指定的三个路径，未包含其他 dirty-worktree 内容。

## 关注点

- 本任务只建立 pipeline 内的 canonical diagnostics-only 状态；向 EventHub/API/前端发布该状态属于后续任务。
- 本任务范围内无已知功能性阻塞。

## 提交

- SHA：`963fc47`
- Subject：`feat: add pelvis-independent ball diagnostics`
- 提交只包含上述三个任务文件。

---

## Fix Round 1

### 修复内容

- diagnostics terminal/reset 边界不再被 planner throttle 丢弃：
  - 最终不可见 snapshot 即使在上一 planner submit 后 `5 ms` 到达，也立即发布
    `TRACK_ENDED / BALL_NOT_VISIBLE`；
  - bounce snapshot 即使处于 throttle window，也先独立清除 diagnostics incoming，
    再发布真实 `ESTIMATING` 状态；
  - worker submit 仍保持原 planner throttle，不因 diagnostics 边界处理而提前提交。
- 将只比较相邻 `_last_key` 改为常量内存的 per-epoch generation watermark：
  - 同 epoch 中 generation 小于或等于已观察 watermark 时不重复计数；
  - epoch 变化时清零 incoming 并接受新 epoch 的 generation；
  - 显式 reset 会清除 watermark，不产生跨 session 的无界集合。
- bounce reset 不再保留 bounce 前的连续 incoming count；首个 post-bounce READY
  incoming snapshot 从 `1/required` 重新开始。
- `BallDiagnosticState.__post_init__()` 将可用速度复制为有限的 3 元
  `tuple[float, float, float]`，并拒绝长度错误或包含 `NaN/Inf` 的向量。

### 覆盖测试

- `test_ball_diagnostics_observes_final_invisible_sample_inside_throttle`
- `test_ball_diagnostics_deduplicates_nonadjacent_generation_per_epoch`
- `test_ball_diagnostics_bounce_inside_throttle_restarts_confirmation`
- `test_ball_diagnostic_state_detaches_immutable_velocity_tuple`
- `test_ball_diagnostic_state_rejects_invalid_velocity_tuple`

### TDD RED

各问题在生产修复前分别运行其最小测试：

```text
test_ball_diagnostics_observes_final_invisible_sample_inside_throttle
AssertionError: 'READY' != 'TRACK_ENDED'
Ran 1 test ... FAILED (failures=1)

test_ball_diagnostics_deduplicates_nonadjacent_generation_per_epoch
AssertionError: [1, 2, 3, 1] != [1, 2, 2, 1]
Ran 1 test ... FAILED (failures=1)

test_ball_diagnostics_bounce_inside_throttle_restarts_confirmation
AssertionError: 'READY' != 'ESTIMATING'
Ran 1 test ... FAILED (failures=1)

test_ball_diagnostic_state_detaches_immutable_velocity_tuple
test_ball_diagnostic_state_rejects_invalid_velocity_tuple
Ran 2 tests ... FAILED (failures=3)
```

失败原因分别精确对应：terminal frame 位于 throttle return 之后、dedup 仅记最后
一个 key、bounce 未形成 diagnostics reset boundary，以及 frozen dataclass 仍引用
调用方传入的 mutable list 且不验证向量。

### GREEN

逐项最小测试修复后均通过。最终 focused 命令：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
/home/loco1/miniconda3/envs/rb/bin/python -m unittest \
  tests.test_hitter_task_pipeline
```

输出：

```text
........................................
----------------------------------------------------------------------
Ran 40 tests in 0.127s

OK
```

### 完整测试套件

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
/home/loco1/miniconda3/envs/rb/bin/python -m unittest discover \
  -s tests -p 'test_hitter_task_*.py'
```

输出：

```text
----------------------------------------------------------------------
Ran 241 tests in 23.112s

OK (skipped=2)
```

### Fix Round 1 自审

- final invisible：diagnostics 在 throttle return 前观察 terminal snapshot；测试同时确认
  worker submissions 仍只有 generation `1`，没有扩大生产 planner 调用节奏。
- dedup：`1 → 2 → 1` 得到 `1 → 2 → 2`；新 epoch 的 generation `1`
  得到 `1`，同时仅保留一个 watermark，不随 session 长度增长。
- bounce：bounce 前计数为 `2`，throttled bounce 后状态为 `ESTIMATING` 且 count
  为 `None`，首个 READY 为 `1/3`；bounce generation 未提交给 worker。
- immutable model：输入 list 在构造后被修改不会改变 state；错误 shape 和非有限值均
  在模型边界被拒绝。
- production 隔离：`plan_snapshot()`、生产 `self.incoming`、planner、
  lifecycle、task observation 与 action 路径没有修改。
- `git diff --check` 与 `git diff --cached --check` 均无输出；提交只包含三个
  Task 3 文件，既有 `deploy/envs/hitter.py` dirty 改动未触碰。

### Fix Round 1 关注点

- 无已知功能性阻塞。
- EventHub/API/前端发布仍属于后续任务，不在本轮扩大范围。

### Fix Round 1 提交

- SHA：`1c46b9818323b664b5e6cb692ca6838c19878a10`
- Subject：`fix: harden HITTER ball diagnostics`
- 提交只包含：
  - `deploy/diagnostics/hitter_task_models.py`
  - `deploy/diagnostics/hitter_task_pipeline.py`
  - `deploy/tests/test_hitter_task_pipeline.py`
