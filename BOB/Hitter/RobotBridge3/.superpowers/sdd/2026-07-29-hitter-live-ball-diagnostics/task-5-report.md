# Task 5 交付报告：前端只渲染结构化真实 v2 值

## 状态

已完成严格 TDD RED/GREEN、真实 headless Chrome 行为测试、完整
`test_hitter_task_*.py` 回归、自审和显式路径提交。

## 实现

- 实时页面只把 schema v2 `live_snapshot` 当作权威：
  - BALL / PELVIS / TABLE 健康来自 `live_snapshot.subjects`；
  - 球检测、新鲜度、estimator、位置、速度、identity 和 diagnostics-only
    incoming 来自 `live_snapshot.ball`；
  - pelvis、production incoming、planner、arm 和 task observation 来自
    `live_snapshot.production_gate`；
  - 当前 attempt 只读取 `live_snapshot.current_attempt`，不使用顶层镜像补值。
- 球实时区和完整任务 gate 分开显示：
  - 球区固定显示 ball subject、estimator、BALL-ONLY INCOMING、raw position、
    estimated position、estimated velocity、speed / vx、track epoch、
    generation、source frame 和 age；
  - 完整任务 gate 固定显示 PELVIS、PRODUCTION INCOMING、PLANNER、ARMED 和
    TASK OBS；
  - pelvis 缺失时球区仍可显示 `READY 31/31` 和 ball-only `n/required`，
    production 区独立显示 `NOT_EVALUATED`、`BLOCKED /
    PELVIS_UNAVAILABLE` 和 `NOT_AVAILABLE`。
- 删除实时进度的 `stage` 字符串解析和 stage-index 着色：
  - estimator、两个 incoming、planner、arm、task observation 的数值与颜色
    都直接使用各自结构化字段；
  - attempt 的 `stage` 只保留给四个精确匹配的 recovery/tail notice，不参与
    实时进度、数值或颜色推断。
- 统一 unknown/null 格式：
  - nullable 数值、向量和 count pair 显示 `—`；
  - enum 缺失显示 `UNKNOWN`；
  - 不再合成 `0/31`、`0/3`、`0/11` 或 `0.000 s`；
  - v1 或缺少 `live_snapshot` 时显示“旧版后端、请重启 diagnostics 服务”，
    并保持所有实时值为 `UNKNOWN / —`；
  - 更高 schema 显示 `UNSUPPORTED_SCHEMA` 并停止猜测。
- TTS 语义分离：
  - planner TTS 只读取真实 `production_gate.planner_tts_s`；
  - 配置项 `production_gate.arm_trigger_tts_s` 明确标为 `ARM threshold`；
  - 不把配置阈值标成实时 ARM TTS。
- 历史：
  - estimator 和 ball-only incoming 新增独立列，直接显示每条持久化
    `AttemptSummary` 的 recorded count/window/required；
  - 任一 count pair 成员缺失时整项显示 `—`，不使用 stage 或默认常量补值；
  - `/api/state` 自动刷新采用 recent-row merge，不再清空已经加载的更早页；
  - `nextBefore` 只由 `/api/attempts` 分页响应更新，state refresh 不修改游标。
- 保留原有只读安全边界、SSE invalidation、请求限速、stale watermark 拒绝、
  text-only DOM 写入和 attempt detail / A/B 展示。

## 文件

- `deploy/diagnostics/static/hitter_task_monitor.html`
- `deploy/tests/test_hitter_task_frontend.py`

未修改 backend、planner、production control 或 `deploy/envs/hitter.py`。
`deploy/envs/hitter.py` 及其他大量 dirty-worktree 内容为任务开始前已有，本提交
没有暂存或改动这些内容。

## TDD：RED

生产前端尚未修改时运行：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
/home/loco1/miniconda3/envs/rb/bin/python -m unittest \
  tests.test_hitter_task_frontend
```

退出码：`1`

关键结果：

```text
F....s...FFFFFF...

FAIL: test_ball_and_production_gate_regions_and_shadow_disclaimer_exist
  新的 ball / production gate 节点不存在

FAIL: test_headless_history_uses_recorded_counts_without_defaults
  '19/31' not found

FAIL: test_headless_null_live_values_render_dashes_without_fake_zeroes
  '0/31' unexpectedly found

FAIL: test_headless_state_refresh_preserves_loaded_history_and_cursor
  '33|cursor=31' != '33,32,31|cursor=31'

FAIL: test_headless_subject_health_uses_structured_subject_values
  'BALL' not found

FAIL: test_headless_v1_does_not_infer_live_progress_from_stage_or_defaults
  '旧版后端' not found

FAIL: test_headless_v2_uses_canonical_ball_and_production_gate_fields
  ball-status-value was empty

Ran 18 tests in 13.116s
FAILED (failures=7, skipped=1)
```

这些失败分别证明旧页面仍缺 structured v2 卡片、使用假零、读取 legacy
stage/attempt、混淆 subject health，并在 state refresh 时删除已分页内容。

另为 higher-schema 拒绝语义单独执行最小 RED：

```text
FAIL: test_headless_newer_schema_stops_instead_of_guessing_values
AssertionError: 'UNSUPPORTED_SCHEMA' not found in
'旧版后端状态：缺少 live_snapshot；请重启 diagnostics 服务。'

Ran 1 test in 0.276s
FAILED (failures=1)
```

## TDD：GREEN

最终 focused 命令：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
/home/loco1/miniconda3/envs/rb/bin/python -m unittest -v \
  tests.test_hitter_task_frontend
```

结果：

```text
----------------------------------------------------------------------
Ran 19 tests in 13.595s

OK (skipped=1)
```

19 个测试中，18 个实际通过；唯一 skip 是既有 localhost HTTP + Chrome
navigation XSS fixture 在当前 Chrome 10 秒内不能结束。所有本任务新增的
file-URL headless Chrome 行为测试均实际执行并通过，包括：

- v2 canonical ball / production gate 绑定；
- conflicting human-readable stage 不改变 structured 数值或颜色；
- null / unknown；
- v1 和 unsupported schema；
- structured subject health；
- recorded history counts；
- history content 与 cursor 跨 state refresh 保留。

## 完整测试套件

提交前按要求运行：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
/home/loco1/miniconda3/envs/rb/bin/python -m unittest discover \
  -s tests -p 'test_hitter_task_*.py'
```

结果：

```text
2026-07-29 14:18:31.436 | INFO | ... - Using Humanoid Batch
......................s.....................................s...
----------------------------------------------------------------------
Ran 264 tests in 25.696s

OK (skipped=2)
```

退出码为 `0`；没有 failure 或 error。两个 skip 均为环境条件 skip。

## 自审

### Backend 字段逐项核对

对照当前 `hitter_task_models.py` 的 serializer：

- `SubjectHealth`：
  - `status`
  - `rate_hz`
  - `age_s`
  - `source_frame`
- `BallDiagnosticState.to_live_json_dict()`：
  - `status`
  - `source_frame`
  - `track_epoch`
  - `generation`
  - `raw_position_w`
  - `estimated_position_w`
  - `estimated_velocity_w`
  - `speed_mps`
  - `velocity_x_mps`
  - `estimator_sample_count`
  - `estimator_window_size`
  - `estimator_ready`
  - `last_estimator_reset_reason`
  - `ball_only_incoming_count`
  - `ball_only_incoming_required`
  - `ball_only_incoming_confirmed`
  - `incoming_status`
  - `blocker`
  - `age_s`
- `ProductionGateState`：
  - `pelvis_status`
  - `production_incoming_status`
  - `production_incoming_count`
  - `production_incoming_required`
  - `planner_status`
  - `planner_reason_code`
  - `planner_tts_s`
  - `arm_status`
  - `arm_trigger_tts_s`
  - `task_observation_status`
  - `task_observation_valid_dimensions`
  - `task_observation_total_dimensions`
  - `task_observation_clip_count`

前端不存在把 `ball_only_incoming_*` 与 `production_incoming_*` 互相回退的
逻辑。

### Truthfulness 与兼容

- 搜索确认已删除 `stageIndex`、`stageValue` 和 stage-substring live
  inference。
- 静态 HTML 初始值也是 `UNKNOWN / —`，页面 JavaScript 执行前不会闪现
  `0/31`、`0/3` 或 `0/11`。
- v1 fixture 故意提供 `TASK_OBS_PASS 11/11`、`31/31`、`3/3` 和 `0.0`
  TTS；页面仍只显示 old-backend notice 与 `UNKNOWN / —`。
- v2 fixture 故意让 human-readable attempt stage/summary 与 canonical live
  fields 冲突；页面仍显示 live ball `31/31`、ball-only `2/3`、
  production `NOT_EVALUATED / —`、planner `BLOCKED /
  PELVIS_UNAVAILABLE`、task `NOT_AVAILABLE / —`。

### History / mutation 检查

- 把 state refresh 恢复为 `renderHistoryRows(rows, false)` 会使
  `33,32,31|cursor=31` 测试失败。
- 把 history count 恢复为 stage/default 会使 `19/31`、`2/3` 和两个 `—`
  的独立单元格断言失败。
- 把 estimator 改读 attempt summary、把 ball-only 改读 production gate，
  或把 planner/task 改读 stage 时，canonical v2 headless 测试会失败。
- 把任一 nullable count/TTS 恢复为零默认会使 null 和 v1 测试失败。

### 安全与提交范围

- 动态数据继续只通过 `textContent` 写入；没有 `innerHTML`、外部脚本或写
  API。
- inline JavaScript 通过 Node parse check。
- `git diff --check` 与 `git diff --cached --check` 均无输出。
- 显式暂存仅包含两个 Task 5 文件；提交没有包含其他 dirty-worktree 内容。

## 关注点

- 当前机器的 localhost Chrome navigation 测试因 10 秒超时按既有逻辑
  skip；相同页面的所有新增 file-URL headless 渲染测试均通过，因此不阻塞
  本任务。没有已知功能性缺陷。
- 历史表中的 `ARM TTS 记录` 仍显示 attempt 持久化的真实计算值；
  live gate 中的配置 `arm_trigger_tts_s` 单独显示为 `ARM threshold`，两者没有
  混用。

## 提交

- SHA：`391f4036d9cdbb96493f06f7a8162454b33733da`
- Subject：`fix: render truthful HITTER live diagnostics`
- 提交只包含：
  - `deploy/diagnostics/static/hitter_task_monitor.html`
  - `deploy/tests/test_hitter_task_frontend.py`

