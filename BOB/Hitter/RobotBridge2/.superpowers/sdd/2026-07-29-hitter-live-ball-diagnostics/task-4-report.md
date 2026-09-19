# Task 4 交付报告：自洽的 v2 实时诊断快照与生产 gate

## 状态

已完成实现、严格 TDD RED/GREEN、focused 测试、完整
`test_hitter_task_*.py` 套件和提交前自审。

## 实现

- 新增不可变的结构化模型：
  - `SubjectHealth`
  - 扩展后的 `BallDiagnosticState`
  - `ProductionGateState`
  - `LiveDiagnosticSnapshot`
- `DiagnosticState` 新增可空的 `live_snapshot`：
  - legacy v1 模型中的字段为 `None`；
  - v1 JSON 保持原字段集合，避免破坏既有 reader；
  - 收到 `LIVE_SNAPSHOT` 后原子升级为 schema v2，并通过
    `state_bytes()` / `/api/state` 直接输出结构化快照。
- monitor 每个 tick 只读取一次：
  - canonical `pipeline.ball_diagnostics.snapshot()`；
  - production incoming snapshot；
  - pelvis snapshot；
  - subject arrival snapshot；
  - recorder status；
  - EventHub 当前 health。
- 同一 tick 生成一个递增 revision 的 `LIVE_SNAPSHOT`，其中 ball、
  production gate 和 current attempt 均来自同一组已捕获值。
- `SubjectHealth` 明确区分：
  - `NEVER_SEEN`
  - `LIVE`
  - `STALE`
  - `INVALID`
- pelvis 缺失时保持真实生产门控：
  - diagnostics-only ball 可以 `READY / 3-of-3`；
  - production incoming 为 `NOT_EVALUATED`，count 为 `None`；
  - planner 为 `BLOCKED / PELVIS_UNAVAILABLE`；
  - planner TTS 为 `None`；
  - arm 为 `NOT_EVALUATED`；
  - task observation 为 `NOT_AVAILABLE`，有效维数与 clip count 为
    `None`。
- 配置中的 `arm_tts_s` 仅发布为 `arm_trigger_tts_s` 阈值；不会再写入
  attempt 的实时 `arm_tts_s`。只有真实可用的 live countdown 才会进入
  attempt。
- current attempt 的 estimator 与 diagnostics-only incoming 四个进度值
  直接使用同一份 ball snapshot，不再从生产 incoming 或 monitor 共享计数
  重新拼接。
- 保留原有 v1 `HEALTH_SNAPSHOT`、`LIFECYCLE_SNAPSHOT` 和
  `ATTEMPT_CURRENT` 事件，未修改 production planner、incoming、
  lifecycle、task observation 或 action 的执行语义。

## 文件

- `deploy/diagnostics/hitter_task_models.py`
- `deploy/diagnostics/hitter_task_monitor.py`
- `deploy/diagnostics/hitter_task_events.py`
- `deploy/tests/test_hitter_task_monitor.py`
- `deploy/tests/test_hitter_task_events.py`

未修改前端文件、`deploy/envs/hitter.py` 或任何 production 控制文件。
`deploy/envs/hitter.py` 在任务开始前已经是 dirty 状态，本任务保持原样且不暂存。

## TDD：RED

### RED 1：跨层 v2 snapshot、reducer 与 v1 unknown

命令：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
/home/loco1/miniconda3/envs/rb/bin/python -m unittest \
  tests.test_hitter_task_monitor.HitterTaskMonitorTest.test_tick_publishes_one_coherent_v2_live_snapshot_without_pelvis \
  tests.test_hitter_task_events.EventHubTests.test_v1_state_exposes_absent_live_snapshot_as_unknown \
  tests.test_hitter_task_events.EventHubTests.test_v2_live_snapshot_reducer_updates_all_fields_atomically
```

结果：

```text
FFF
Ran 3 tests in 0.002s
FAILED (failures=3)
```

失败原因符合预期：

- API state 尚无 `live_snapshot`；
- monitor 仍只发布三个 legacy projection；
- reducer 忽略 v2 live event，state 仍为 schema v1；
- legacy attempt 仍把配置阈值写入实时 `arm_tts_s`。

### RED 2：pelvis blocker 下不得泄漏 stale command TTS

```text
FAIL: test_blocked_pelvis_does_not_leak_stale_command_tts
AssertionError: 12.0 is not None
Ran 2 tests in 0.005s
FAILED (failures=1)
```

失败证明旧 projection 会在 pelvis 已阻塞时继续暴露残留 command 的绝对
deadline 和计算 TTS。

### RED 3：attempt 与 ball snapshot 必须使用同一 window/required

```text
FAIL: test_attempt_projection_uses_same_ball_window_and_required_counts
AssertionError: 'INCOMING_CONFIRMING 2/3' != 'INCOMING_CONFIRMING 2/4'
Ran 1 test in 0.002s
FAILED (failures=1)
```

失败证明 stage 和 attempt 仍读取配置 `3`，没有使用当前 ball snapshot 的
真实 required `4`。

### RED 4：残留 ARMED phase 不得越过 pelvis blocker

```text
FAIL: test_blocked_pelvis_does_not_leak_stale_command_tts
AssertionError: 'ARMED' != 'NOT_EVALUATED'
Ran 1 test in 0.002s
FAILED (failures=1)
```

失败证明只读取 lifecycle phase 会把已经被 pelvis gate 阻塞的 arm 错标成
`ARMED`。

## TDD：GREEN

focused 命令：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
/home/loco1/miniconda3/envs/rb/bin/python -m unittest \
  tests.test_hitter_task_monitor \
  tests.test_hitter_task_events
```

最终结果：

```text
...................................................
----------------------------------------------------------------------
Ran 51 tests in 1.795s

OK
```

新增行为测试覆盖：

- monitor → EventHub → `DiagnosticState.to_json_dict()` 的真实跨层路径；
- 同一个 revision 内 ball `31/31`、diagnostics incoming `3/3` 和
  attempt 完全一致；
- 每 tick 只调用一次 canonical ball diagnostics snapshot；
- `NEVER_SEEN / LIVE / STALE / INVALID` 四种 subject 状态；
- pelvis blocker、production incoming unknown、planner/arm/task obs gate；
- configured arm threshold 与 live TTS 分离；
- stale command / stale task observation 不得越过 pelvis blocker；
- 非默认 estimator window 和 incoming required 值保持一致；
- v1 state 兼容与 v2 reducer 原子更新。

## 完整测试套件

命令：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
/home/loco1/miniconda3/envs/rb/bin/python -m unittest discover \
  -s tests -p 'test_hitter_task_*.py'
```

最终结果：

```text
----------------------------------------------------------------------
Ran 247 tests in 23.368s

OK (skipped=2)
```

两个 skip 为套件既有条件跳过，没有失败或错误。

## 自审

### Schema 兼容

- `DiagnosticState` 接受 schema v1 和 v2。
- v1 实例的 `live_snapshot` 为 `None`，且 v1 JSON 不增加新 key，保持既有
  exact-schema reader 兼容。
- `LIVE_SNAPSHOT` payload 必须是 schema v2；reducer 一次性更新
  `schema_version`、`current_attempt` 和 `live_snapshot`。
- v1 attempt 缺失进度字段时继续保持 `None`，没有通过 stage 字符串回推。

### Snapshot 一致性

- ball diagnostics、production incoming、pelvis、arrival、recorder 均只读取
  一次，再构造 immutable `LiveDiagnosticSnapshot`。
- canonical ball 对象与 monitor 捕获的 frame/position metadata 只有 identity
  一致时才合并，避免数值相同但 generation 不同的错误拼接。
- estimator count/window、ball-only count/required、speed 和 attempt projection
  均来自同一 ball state。
- revision 每 tick 只增加一次；一个 tick 只发布一个 `LIVE_SNAPSHOT`。

### 无伪造默认值

- `NEVER_SEEN` 的 rate、age、frame、valid、occluded 保持 `None`。
- canonical ball diagnostics 不可用时 live ball count 保持 `None`，不合成
  `0/31` 或 `0/3`。
- production incoming 未执行时 count 为 `None`，而不是零。
- planner / predicted / arm TTS 不可用时均为 `None`。
- 非有限向量不会进入 v2 API；不可用字段保持 `None`。

### Blocker 与 task observation

- pelvis 不可用优先于残留 lifecycle、command 和 task observation。
- planner 固定输出 `BLOCKED / PELVIS_UNAVAILABLE`，不会泄漏 stale command
  deadline/TTS。
- arm 固定为 `NOT_EVALUATED`。
- task observation 固定为 `NOT_AVAILABLE`，有效维数与 clip count 为 `None`；
  不会把残留 11 维 observation 错标为可用。
- active attempt 保留已有 terminal blocker；仅在 blocker 为空时投影真实
  production gate blocker。

### Production 隔离与工作树

- `git diff` 中没有新增 `plan_snapshot`、`IncomingTrackConfirmation`、
  action、LCM control publish 或 planner 调用。
- 没有修改 `deploy/envs/hitter.py`、production incoming、planner、
  lifecycle、task observation 或 action 文件。
- `git diff --check` 对五个 Task 4 路径无输出。
- 暂存与提交使用明确路径，不包含大量既有 dirty-worktree 内容。

## 关注点

- 前端仍使用 legacy 字段；绑定 `live_snapshot` 属于 Task 5，本任务按要求
  未修改前端。
- 为保持 v1 exact-schema reader 兼容，v1 JSON 不输出 `live_snapshot` key；
  Python 模型字段仍明确为 `None`。v2 API state 会输出完整结构。
- v2 live event 使用既有 EventHub 外层事件信封，schema v2 位于
  `LIVE_SNAPSHOT` payload 与 DashboardState 中；既有 v1 event reader 仍可
  跳过未知 kind。

## Fix Round 1（2026-07-29）

### 修复结果

本轮按 review finding 修复了五类一致性问题：

1. canonical `BallDiagnosticState` 现在由
   `BallDiagnosticTracker.observe()` 直接携带该次 observation 的
   `source_frame / track_epoch / generation`、raw/estimated position、
   observed time、velocity、estimator count 和 ball-only incoming count。
   monitor 不再把节流后尚未被 tracker 接受的新 input metadata 拼到旧
   canonical 数值上。
2. monitor 新增统一的 capture lock：
   - input 在同一边界内完成 adapter ingest、pipeline ingest、arrival 和
     monitor 状态投影；
   - tick 在同一边界内完成 pipeline tick、incoming/ball/arrival/pelvis、
     gate、attempt 和 live snapshot 捕获；
   - monitor 自己的 transition、live、lifecycle、attempt 和 health
     `_offer()` 在 capture lock 释放后执行；pipeline 在其 ingest/tick
     内部产生的 diagnostics event 仍可能位于 capture boundary 内。
3. production gate 使用显式上游优先级：
   `pelvis -> estimator evidence -> production incoming -> planner -> arm
   -> task observation`。残留 ARMED phase、command 或 task result 不再越过
   已阻塞的上游 gate。
4. `UNKNOWN` ball 不再读取 monitor 的旧共享计数或速度；attempt 直接复制
   canonical ball 中的 `null`，且无 estimator evidence 时使用
   `BALL_STATE_UNKNOWN`，不会伪造成 `ESTIMATOR_WARMING`。
5. v2 中 `live_snapshot.current_attempt` 是权威值，顶层
   `current_attempt` 只是兼容镜像：
   - `DiagnosticState` 拒绝两份 current attempt 不一致的实例；
   - legacy `ATTEMPT_CURRENT` 在已有 live snapshot 后不能覆盖 frozen
     revision；
   - legacy `ATTEMPT_CLOSED` 继续更新 recent history，但不能局部清除或
     复活 live current。

### Reviewer-driven scope expansion

原 Task 4 实现主要限定在 models、monitor 和 events。本轮 critical finding
要求“canonical ball state 本身携带精确 observation identity/metadata”，
正确 ownership 位于 `hitter_task_pipeline.py` 的
`BallDiagnosticTracker`，因此经 reviewer/父任务允许，范围扩展为：

- `deploy/diagnostics/hitter_task_pipeline.py`
- `deploy/tests/test_hitter_task_pipeline.py`

该扩展只影响 diagnostics-only tracker 的不可变状态，不修改 production
planner、production incoming、action、LCM control 或
`deploy/envs/hitter.py`。

### TDD：RED

新增测试先在 `1814e1e` 行为上失败：

- `test_throttled_ingest_keeps_one_canonical_ball_observation`
  - identity/position 均为 `None`，无法证明与 velocity/count 属于同一代。
- `test_throttled_input_does_not_relabel_canonical_ball_generation`
  - canonical generation 11 被 monitor 错标为 throttled generation 12，
    frame/position 同时从 101/第一代变成 102/第二代。
- `test_tick_capture_waits_for_inflight_input_projection_boundary`
  - input 尚停在 adapter ingest 时，tick 已进入 `pipeline.tick`。
- `test_upstream_gate_precedence_dominates_stale_downstream_state`
  - 五个参数化场景中 stale `TASK_OBS_PASS / AVAILABLE` 越过 pelvis、
    estimator、incoming、planner 或 arm gate。
- `test_unknown_ball_projection_preserves_null_progress`
  - `UNKNOWN` 被错误解释成 `ESTIMATOR_WARMING`，并可能读取旧的 count/speed。
- 五个 v2 reducer ordering/invariant 测试：
  - legacy current 可覆盖顶层；
  - close 可只清顶层；
  - stale current 可在 live-null 后复活 attempt；
  - 两份 current mirror 不一致时模型未拒绝。

代表性 RED 结果：

```text
test_throttled_ingest_keeps_one_canonical_ball_observation ... FAIL
test_throttled_input_does_not_relabel_canonical_ball_generation ... FAIL
test_tick_capture_waits_for_inflight_input_projection_boundary ... FAIL
test_upstream_gate_precedence_dominates_stale_downstream_state ... FAIL
test_unknown_ball_projection_preserves_null_progress ... FAIL

Reducer ordering:
Ran 5 tests in 0.003s
FAILED (failures=3, errors=1)
```

### GREEN：focused

命令：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
/home/loco1/miniconda3/envs/rb/bin/python -m unittest \
  tests.test_hitter_task_monitor \
  tests.test_hitter_task_events \
  tests.test_hitter_task_pipeline
```

最终 fresh 结果：

```text
.....................................................................................................
----------------------------------------------------------------------
Ran 101 tests in 1.926s

OK
```

### GREEN：完整套件

命令：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
/home/loco1/miniconda3/envs/rb/bin/python -m unittest discover \
  -s tests -p 'test_hitter_task_*.py'
```

最终 fresh 结果：

```text
----------------------------------------------------------------------
Ran 257 tests in 23.737s

OK (skipped=2)
```

两个 skip 均为套件既有条件跳过。

### Fix Round 1 自审

- **同代数据**：真实 pipeline ingest/throttle 测试使用两代不同 frame、
  position、velocity 和 estimator count，确认发布状态完整保留被 tracker
  接受的第一代，而不是混入第二代 metadata。
- **锁顺序**：固定为
  `snapshot_capture -> tick/pipeline internal -> monitor state`；monitor
  直接调用的 `_offer()` 均在 capture lock 外。pipeline 自身在
  ingest/tick 内调用 event sink 的既有行为不在此断言范围内。
- **门控真值**：pelvis、estimator、incoming、planner rejection 和未 ARM
  五类场景均验证 task dimensions/clip 为 `None`，current stage/blocker
  来自最早阻塞的上游。
- **无默认值伪造**：删除 monitor 的 legacy count/speed fallback 及其死
  状态；UNKNOWN attempt 的 speed/count/required 保持与 live ball 完全相同。
- **v2 authority**：legacy event 仍会被完整解析验证，但不能在不增加
  revision、也不重采 ball/gate 的情况下局部改写 live snapshot。
- **兼容性**：无 live snapshot 的 v1 reducer 行为保持不变；old close 不会
  清除新的 live current，close 仍正常 upsert recent history。
- **工作树隔离**：仅显式暂存六个 diagnostics/test 路径；未修改或暂存
  frontend、`deploy/envs/hitter.py` 及其他既有 dirty 文件。

## Fix Round 2（2026-07-29）

### Finding 与根因

Fix Round 1 的 input/tick capture boundary 已防止读取撕裂，但
`_tick_once_under_projection()` 仍在申请 `_snapshot_capture_lock` 之前采样：

```text
lifecycle_now_s
obs_now_s
wall_time_us
```

如果 tick 先取得 `10.0` 后等待锁，input 随后在锁内提交
`observed_monotonic_s=11.0` 的 ball，tick 最终会把该 ball 放入
`captured_monotonic_s=10.0` 的 live snapshot。旧的
`max(0, obs_now_s - observed_monotonic_s)` 只会把负 age 掩盖成 `0.0`，
不能恢复真实时序。

### TDD：RED

新增确定性并发测试：

```text
test_tick_clocks_are_sampled_after_waiting_input_commits
```

测试使用包裹真实 `threading.RLock` 的观测锁固定顺序：

1. input 侧先持有 capture lock；
2. tick 线程到达并等待该锁；
3. input 在同一线程重入锁并提交 observed time `11.0`；
4. input 标记 commit 完成并释放锁；
5. tick 获锁后完成 capture。

命令：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
/home/loco1/miniconda3/envs/rb/bin/python -m unittest \
  tests.test_hitter_task_monitor.HitterTaskMonitorTest.test_tick_clocks_are_sampled_after_waiting_input_commits \
  -v
```

RED 结果：

```text
test_tick_clocks_are_sampled_after_waiting_input_commits ... FAIL

AssertionError: 10.0 not greater than or equal to 11.0

Ran 1 test in 0.002s
FAILED (failures=1)
```

失败直接证明 snapshot capture time 早于其中包含的 ball observation。

### 最小修复

只把 lifecycle、observation 和 wall 三次 tick 时钟采样移动到：

```python
with self._snapshot_capture_lock:
```

之后、`pipeline.tick()` 之前。未修改 age clamp、pipeline、gate、reducer、
event schema 或 publication 顺序。

### GREEN：单项

同一命令最终结果：

```text
test_tick_clocks_are_sampled_after_waiting_input_commits ... ok

Ran 1 test in 0.001s
OK
```

回归测试同时确认：

- `captured_monotonic_s=12.0 >= observed_monotonic_s=11.0`；
- `age_s` 精确等于实际差值 `1.0`，不依赖 clamp；
- lifecycle clock 与 observation clock 都是锁等待后的 `12.0`；
- wall clock 是锁等待后的 `2_000_000 us`；
- tick 线程在一秒内结束且无异常；
- monitor 直接控制的 event publication 发生时 capture lock 未被当前线程持有。

### GREEN：focused

命令：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
/home/loco1/miniconda3/envs/rb/bin/python -m unittest \
  tests.test_hitter_task_monitor \
  tests.test_hitter_task_events \
  tests.test_hitter_task_pipeline
```

结果：

```text
......................................................................................................
----------------------------------------------------------------------
Ran 102 tests in 2.026s

OK
```

### GREEN：完整套件

命令：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
/home/loco1/miniconda3/envs/rb/bin/python -m unittest discover \
  -s tests -p 'test_hitter_task_*.py'
```

结果：

```text
----------------------------------------------------------------------
Ran 258 tests in 22.750s

OK (skipped=2)
```

两个 skip 均为套件既有条件跳过。

### Fix Round 2 自审

- **时间边界**：所有定义本 tick lifecycle、observation、TTS 和 event wall
  time 的采样均在 capture lock 获取后进行，因此锁等待时间不会污染已发布
  snapshot。
- **因果顺序**：任何先于 tick 获锁并被本 tick 包含的 input，其 observed
  time 都不晚于 captured time。
- **真实 age**：测试显式比较 `captured - observed` 与 `age_s`，避免
  `max(0, ...)` 让错误时序仍然通过。
- **死锁**：测试固定真实争锁顺序并检查 tick thread 正常退出；实现没有新增
  lock，也没有改变既有 lock order。
- **publication 范围**：monitor 在 `_tick_once_under_projection()` 中直接
  发出的 `_offer()` 仍全部位于 capture lock 外；pipeline ingest/tick 的
  内部 event sink 可能在锁内，报告不再作过度承诺。
- **范围**：代码只修改 monitor 的三行时钟采样位置，并新增一个 monitor
  regression test；未修改 pipeline、events、frontend 或
  `deploy/envs/hitter.py`。
