# HITTER 球链路实时诊断与真实数据展示设计

## 目标

修复 HITTER 旁路诊断网页中 `ESTIMATING 0/31` 等失真显示，并让页面当前展示的所有参数都来自明确、结构化、可追溯的实时状态。

本设计同时支持 pelvis 缺失时的球链路独立诊断：

- 球检测、球 estimator、球位置、球速度和诊断专用 incoming 候选状态继续运行；
- 生产 incoming、planner、armed 和 11 维 task observation 保持原有 pelvis 门控；
- 页面明确区分“球链路正常”和“完整任务链路因 pelvis 阻塞”。

本设计只修改 `deploy/diagnostics` 及其测试和文档，不修改 `deploy/envs/hitter.py` 的真机控制顺序，不向机器人发布控制消息。

## 已确认的问题

### 事件字段丢失

`HitterTaskMonitor` 发布的 `ATTEMPT_CURRENT` 事件已经包含：

- `estimator_sample_count`
- `estimator_window_size`
- `incoming_count`
- `incoming_required_count`

但 EventHub 的 `hitter_task_events.py::_attempt()` 没有反序列化这些字段，导致 `/api/state` 重新使用 `AttemptSummary` 的默认 `0/31` 和 `0/3`。

磁盘详情恢复路径 `hitter_task_recording.py::_summary_from_json()` 也遗漏相同字段，造成历史记录在 cache 淘汰或进程重启后重新归零。

### 前端通过字符串和默认值猜测状态

当前网页仍会：

- 从 `stage` 字符串推断 estimator、incoming、planner、armed 和 task observation；
- 把缺失值显示为 `0/31`、`0/3`、`0/11`；
- 把未知 recorder 状态显示成 `HEALTHY` 或 `COMPLETE`；
- 把未知 lifecycle phase 显示成 `waiting`；
- 把固定的 `arm_tts_s` 阈值显示成实时倒计时；
- 把绝对 monotonic deadline 当作相对秒数显示。

这些默认值无法区分真实零、尚未计算、从未收到和已经过期。

### Pelvis 门控掩盖球链路

生产链路和现有 diagnostics planner 都采用：

```text
visible
→ estimator ready
→ base_valid
→ production incoming
→ planner
```

因此 pelvis 缺失时：

- 球 estimator 已经可以得到位置、速度和 `31/31`；
- production incoming 尚未执行，而不是执行后得到 `0/3`；
- planner 应显示 `PELVIS_UNAVAILABLE`；
- 完整 task observation 不可用。

为了专门排查球，本设计新增独立且只读的 ball-only incoming 候选状态，但不改变上述生产门控。

## 设计原则

### 结构化状态优先

前端不得从人类可读的 `stage` 字符串推断数值或阶段。每个页面参数必须有独立 API 字段和明确状态枚举。

### 未知不是零

使用以下语义：

- `null`：尚无数值；
- `NEVER_SEEN`：本进程启动后从未收到；
- `NOT_EVALUATED`：先决条件不足，算法尚未执行；
- `STALE`：曾经收到，但超过新鲜度阈值；
- `INVALID`：收到显式无效数据；
- `BLOCKED`：后续阶段被明确门控；
- `REJECTED`：算法已经执行并返回拒绝原因。

前端分别显示 `—`、状态标签和 reason code，不使用伪造的零值或成功状态。

### 诊断状态不得影响生产状态

ball-only incoming 使用独立状态机：

- 输入只来自当前球轨迹的 ready estimator snapshot；
- 每个唯一 `(track_epoch, generation)` 最多观察一次；
- 观察节奏服从 diagnostics planner update throttle，不受 50 Hz 页面
  tick 或 HTTP 刷新次数影响；
- 判断阈值与配置中的 `minimum_stable_incoming_speed_x_mps` 和 `stable_incoming_confirmation_snapshots` 一致；
- 不要求 pelvis；
- 不写入生产 `IncomingTrackConfirmation`；
- 不驱动 planner、lifecycle、task observation 或 action；
- 在 track epoch 变化、球轨迹结束或显式 reset 时独立清零。

## 数据模型

### SubjectHealth

为 `ball`、`g1pelvis`、`table` 分别提供：

```text
status
rate_hz
age_s
source_frame
valid
occluded
```

全局 transport 在线状态与各 subject 是否可用分开表达。`table` 在线不得让页面误认为 ball 或 pelvis 也可用。

### BallDiagnosticState

当前球状态包含：

```text
status
source_frame
track_epoch
generation
raw_position_w[3]
estimated_position_w[3]
estimated_velocity_w[3]
speed_mps
velocity_x_mps
estimator_sample_count
estimator_window_size
estimator_ready
last_estimator_reset_reason
ball_only_incoming_count
ball_only_incoming_required
ball_only_incoming_confirmed
observed_monotonic_s
age_s
```

所有向量必须为有限数；不可用时字段为 `null`，同时由 `status` 解释原因。

### ProductionGateState

完整任务链路单独提供：

```text
pelvis_status
production_incoming_status
production_incoming_count
production_incoming_required
planner_status
planner_reason_code
planner_tts_s
arm_status
arm_trigger_tts_s
task_observation_status
task_observation_valid_dimensions
task_observation_total_dimensions
task_observation_clip_count
```

pelvis 缺失时的标准状态是：

```text
production_incoming_status = NOT_EVALUATED
planner_status = BLOCKED
planner_reason_code = PELVIS_UNAVAILABLE
arm_status = NOT_EVALUATED
task_observation_status = NOT_AVAILABLE
```

### AttemptSummary 持久化

当前与历史 attempt 均保存：

- estimator 当前值和已达到的最大样本数；
- production incoming 当前值和最大连续计数；
- ball-only incoming 当前值和最大连续计数；
- 终止时的球位置、速度和 blocker；
- structured stage code 和 reason code。

EventHub reducer、JSON 详情、CSV schema 和磁盘 repository reload 必须完整保留这些字段。

## 数据流

```text
ChingMu LCM
→ MocapFrameAdapter
→ BallStateEstimator
→ immutable BallDiagnosticSnapshot
→ ball-only incoming observer
→ monitor tick projection
→ ATTEMPT_CURRENT / HEALTH / LIFECYCLE events
→ EventHub reducer
→ DiagnosticState JSON
→ /api/state
→ structured frontend rendering
```

生产 planner 继续使用原有独立链路：

```text
BallEstimateSnapshot
→ base_valid gate
→ production incoming
→ planner
→ lifecycle
→ task observation
```

两个 incoming observer 不共享可变状态。

## Schema 与兼容性

新的 state、attempt event、attempt detail 和持久化记录使用
`schema_version = 2`。

- v2 writer 必须写出所有结构化状态字段；
- v2 reader 不得用默认零或默认成功掩盖缺失字段；
- repository 继续支持读取 v1 session；
- v1 中不存在的字段恢复为 `null` 和 `UNKNOWN`，不得恢复为真实
  `0/31`、`0/3`、`HEALTHY` 或 `waiting`；
- v1 历史数据不得通过 `stage` 字符串反向伪造精确实时数值；
- API 与页面若收到不支持的更高 schema，显示
  `UNSUPPORTED_SCHEMA`，而不是继续猜测。

## 一致性与并发

每个 monitor tick 只读取一次球诊断快照、production gate 快照和 recorder 状态，再用这组不可变值生成同一个 API 状态。

以下字段必须来自同一 snapshot：

- estimator count、ready、位置和速度；
- track epoch、generation 和 source frame；
- ball-only incoming count 与其输入速度；
- planner status、reason 和 TTS；
- task observation status、维数和 clip count。

不得在 stage 生成前后重复读取共享计数。

## 页面设计

### 运行健康

分别显示 BALL、PELVIS、TABLE：

```text
STATUS / RATE / AGE / SOURCE FRAME
```

显示 transport、recorder 和 active warnings，但未知状态不得默认为成功。

### 球实时状态

页面固定显示：

- raw position；
- estimated position；
- estimated velocity；
- speed 与 `vx`；
- estimator `n/window` 与 ready；
- ball-only incoming `n/required`；
- track epoch、generation、source frame 和 age。

pelvis 缺失不隐藏本区域。

### 完整任务门控

显示：

```text
PELVIS
→ PRODUCTION INCOMING
→ PLANNER
→ ARMED
→ TASK OBS
```

每一项显示结构化状态和 blocker。不得根据前一阶段的索引推断后续状态。

### 历史与详情

- 活动 attempt 始终显示在“当前来球”；
- 关闭后进入“逐球历史”；
- 页面自动刷新不得清除已加载的历史分页；
- 点击历史行显示持久化的球里程碑、planner blocker、task observation 和 A/B 详情；
- cache 命中与磁盘 reload 必须显示一致数据。

## 错误处理

- 非有限数值不得进入 API，字段置 `null` 并附带明确状态；
- EventHub 遇到缺失的新字段时只为旧 schema 提供兼容处理，不得把新事件中的缺失解释成真实零；
- subject stale 不删除最后值，但必须标记 `STALE` 并显示 age；
- planner rejection 保留最近一次 reason code，不得回退为 `PLANNER_WAITING`；
- recorder 状态未知时显示 `UNKNOWN`，不得显示 `HEALTHY`；
- ball-only observer 异常只能形成 diagnostics warning，不得影响生产 worker。

## 测试设计

### 事件与 API

- 构造非默认 estimator/incoming 计数，经 EventDraft、EventHub reducer、`state_bytes()` 和 JSON 后保持完全一致；
- 测试 current attempt 和 closed attempt；
- 测试 v1 payload 按旧 schema 兼容，新字段恢复为
  `null / UNKNOWN`，且不伪造成功；
- 测试一个 tick 内所有球字段来自同一 generation。

### 持久化

- 保存 attempt 后清空内存 cache，再从磁盘加载；
- estimator、production incoming、ball-only incoming 和里程碑保持一致；
- CSV 与 JSON schema 都覆盖新增字段。

### Pelvis 缺失

- pelvis `NEVER_SEEN` 时 estimator 可以从 `1/31` 到 `31/31`；
- ball-only incoming 可以从 `0/3` 到 `3/3`；
- 同一个 estimator generation 被多个 50 Hz tick 和 HTTP refresh
  重复观察时，ball-only incoming 只能累计一次；
- production incoming 保持 `NOT_EVALUATED`；
- planner 明确为 `BLOCKED / PELVIS_UNAVAILABLE`；
- task observation 为 `NOT_AVAILABLE`；
- 不产生 command 或控制消息。

### 前端

Headless 浏览器验证：

- `31/31` 不得变成 `0/31`；
- production incoming 未评估时不得显示 `0/3`；
- unknown、never-seen、stale、blocked、rejected 分别正确显示；
- position、velocity、speed、frame 和 age 与 API 一致；
- arm 阈值不再标成实时 TTS；
- task observation 不可用时不得显示假 `0/11`；
- 自动刷新不清除历史分页；
- 历史行点击可以加载 cache 和磁盘详情。

### 回归

- 所有 `test_hitter_task_*.py` 通过；
- 真机 `envs/hitter.py` 无修改；
- ball-only counter 不能改变生产 incoming、planner、lifecycle 或 action；
- diagnostics 仍只订阅状态消息，不发布机器人控制消息。

## 验收标准

现场 pelvis 缺失、球静止时，网页应显示类似：

```text
BALL VALID
ESTIMATOR READY 31/31
vx ≈ 0
BALL-ONLY INCOMING 0/3
PELVIS NEVER_SEEN
PRODUCTION INCOMING NOT_EVALUATED
PLANNER BLOCKED PELVIS_UNAVAILABLE
TASK OBS NOT_AVAILABLE
```

现场 pelvis 缺失、球满足 `vx <= -0.20 m/s` 连续三次时：

```text
BALL-ONLY INCOMING 3/3
PRODUCTION INCOMING NOT_EVALUATED
PLANNER BLOCKED PELVIS_UNAVAILABLE
```

pelvis 恢复有效后，production incoming、planner、armed 和 task observation 按原生产门控继续推进。
