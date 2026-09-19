# HITTER 真机任务 Observation 逐球诊断设计

日期：2026-07-27
状态：设计已由用户确认，等待书面规格 review

## 1. 背景

当前需要定位：为什么较高速来球经过 ChingMu 链路后，有时没有形成可供 HITTER policy 使用的有效击球任务 observation。

真机现有链路为：

```text
ChingMu SDK
→ chingmu_table_lcm_bridge.py
→ LCM transformation_t（ball / G2Pelvis）
→ BallStateEstimator（31 帧）
→ planner 提交节流（当前 100 Hz）
→ LatestOnlyPlannerWorker
   → plan_fn 内 IncomingTrackConfirmation（当前连续 3 次）
   → HitterSystemPlanner
→ HitterCommandLifecycle（50 Hz 消费）
→ HITTER task observation
→ 完整 policy observation / ONNX / action / 真机控制
```

本次只诊断到 `HITTER task observation`，不进入 ONNX、action 或控制链路。

该独立进程是生产算法与调度语义的shadow replica。其LCM收包时刻、OS线程调度、CPU竞争和50 Hz相位不可能等同于真实RobotBridge进程，因此它证明的是“在shadow诊断链路下可以组装任务observation”，不是“生产policy实际收到了该observation”。若以后需要证明生产事实，必须另行在真实 `HitterAgent` 中增加只读telemetry tap，不属于本规格。

## 2. 目标与非目标

### 2.1 目标

1. 只依赖 ChingMu bridge 发布的球和 `G2Pelvis` LCM 数据。
2. 对用户每次扔球建立独立 `attempt_id`，逐阶段显示进度。
3. 复用生产 estimator、planner worker 和 lifecycle 语义。
4. 判断当前球在shadow replica中是否形成有效的 11 维 task observation。
5. 精确记录每代rejection、最终primary blocker、输入值和时序。
6. 保存完整标准化重放输入，使同一颗球可以离线重放。
7. 用同一份输入比较：
   - 当前基线：incoming 连续 3 次、planner 100 Hz。
   - 最小对照：incoming 1 次、planner 100 Hz。
8. 在浏览器中实时显示当前球和历史球，并持久化诊断结果。

### 2.2 非目标

1. 不加载或执行 ONNX。
2. 不构造29维 action。
3. 不启动 `trans`，不读取关节、IMU或手柄。
4. 不调用真机 simulator step。
5. 不发布 `pd_plustau_targets` 或任何控制消息。
6. 不在诊断过程中自动修改 YAML、降低阈值或重启进程。
7. 第一阶段不把生产 planner 改为360 Hz。
8. 不恢复工作树中已经删除的旧监控脚本和旧测试。

## 3. 成功判据

当前104维 policy observation 中，本次只检查索引 `6:17` 对应的11维任务字段：

| Task索引 | 完整obs索引 | 字段 | 坐标系 |
|---:|---:|---|---|
| `0:2` | `6:8` | `base_forward_xy` | world |
| `2:4` | `8:10` | `base_target_xy` | base yaw frame |
| `4:7` | `10:13` | `racket_target_pos` | base yaw frame |
| `7:10` | `13:16` | `racket_target_vel` | world |
| `10` | `16` | `time_to_strike` | seconds |

某颗球的最终绿色状态必须同时满足：

```text
lifecycle phase == ARMED
active planner result 属于当前 attempt
active track_epoch / generation 与任务字段来源一致
task_obs.shape == (11,)
task_obs.dtype == float32
全部11个值 finite
clip_count == 0
0 < time_to_strike <= maximum_policy_time_to_strike_s
base/racket/TTS字段与同一控制 tick 的 active command 一致
```

同一50 Hz tick保留生产中的两次时钟语义：先记录 `lifecycle_now_s` 并推进/消费lifecycle，随后在task组装前记录 `obs_now_s` 并计算 `policy_tts()`。共享helper接收已经算好的TTS，不在内部读取时钟。tick组装时还要原子复制当时最新的pelvis位置/四元数。planner command使用旧snapshot中的pelvis规划，但task observation必须像生产代码一样使用obs时最新pelvis计算base forward及world到base-yaw变换。

`TRACKING` 不算成功，因为这时真实 HITTER observation 仍走 waiting 分支。`RECOVERY` 也不算首次成功，因为击球 deadline 已到达或越过。

页面最终文案固定为：

```text
SHADOW 3/100 TASK OBS 11/11 PASS
```

不使用“已经送入 policy/ONNX”“生产链路已经消费”或“完整104维 observation 已通过”的表述。

## 4. 总体架构

新增独立只读诊断进程：

```text
LCM subscriber
    ↓
MocapFrameAdapter + BallStateEstimator（LCM线程，匹配生产）
    ↓
AttemptTracker
    ↓
100 Hz LatestOnlyPlannerWorker
    └─ plan_fn：incoming confirmation → planner
    ↓
50 Hz HitterCommandLifecycle
    ↓
TaskObservationAssembler
    ↓
DiagnosticStateStore + EventHub（只读fan-out）
    ├─ Raw/SessionRecorder
    └─ localhost HTTP + SSE frontend
```

### 4.1 代码边界

计划使用以下独立组件：

1. `deploy/utils/hitter_task_observation.py`
   - 提供无副作用的11维 task observation 组装函数。
   - `HitterEnv` 与诊断进程共同调用，避免维护两套坐标变换。
   - 输入是observation时原子复制的“最新pelvis”、active command和调用方已经计算好的policy TTS，不是planner snapshot中的旧pelvis。
   - helper内部不读取时钟；`HitterEnv` 保留现有 lifecycle update 与 observation TTS 的两次 `time.monotonic()` 读取顺序。
   - 同时返回pre-clip与按当前 `obs_clip_value` 处理后的post-clip结果；PASS要求 `clip_count == 0`。
   - 这是对生产 `HitterEnv` 的机械提取：先做characterization test，再提取；提取前后完整 `obs[6:17]` 必须逐项一致，行为不变。

2. `deploy/diagnostics/hitter_task_pipeline.py`
   - 接收标准化的 ball / pelvis 帧。
   - 在LCM owner线程内按生产规则驱动estimator，然后把不可变snapshot提交给planner worker。
   - planner worker的 `plan_fn` 内依次执行incoming confirmation和完整planner。
   - 在一个50 Hz tick内按生产顺序分别记录 `lifecycle_now_s` 和 `obs_now_s`，并在obs阶段复制latest pelvis。
   - 用诊断wrapper把 `attempt_id/track_segment_id` 与生产 `BallEstimateSnapshot/PlannerResultSnapshot` 绑定；不改变生产dataclass API。
   - 生成不可变结构化诊断事件。

3. `deploy/diagnostics/hitter_task_attempts.py`
   - 管理 `attempt_id`、逐阶段状态、首次终态失败和汇总结果。
   - 不依赖 LCM、HTTP或文件系统。

4. `deploy/diagnostics/hitter_task_replay.py`
   - 对一颗球的完整标准化输入做离线、离散事件重放。
   - 使用虚拟monotonic时钟以及显式的100 Hz提交、capacity-one worker和50 Hz消费调度。
   - 第一阶段只比较 `3/100` 与 `1/100`。

5. `deploy/diagnostics/hitter_task_web.py`
   - 使用 Python 标准库 `ThreadingHTTPServer` 和 SSE。
   - 仅提供只读接口，绑定 `127.0.0.1`。

6. `deploy/diagnostics/hitter_task_monitor.py`
   - CLI入口；加载配置、创建会话目录、启动订阅/诊断/记录/网页服务并处理退出。

7. `deploy/diagnostics/static/hitter_task_monitor.html`
   - 原生 HTML/CSS/JavaScript 单页前端。
   - 不引入 Flask、FastAPI、Node或构建工具。

旧的、当前已删除的 `deploy/mocap_bridge/monitor_hitter_prediction.py` 不作为实现基础，也不恢复。

## 5. 数据源与时序

### 5.1 输入

诊断进程只订阅现有 `transformation_t` LCM频道：

- `name == "ball"`：位置、valid/occluded、source frame/time、publish time及原始payload长度。
- `name == "g1pelvis"`：位置、四元数、valid/occluded、source frame/time、publish time及原始payload长度。
- `name == "table"`：仅记录健康状态，不参与逐球规划。

`G2Pelvis` 四元数必须使用 bridge 已完成桌面世界系和 pelvis 朝向标定后的值，不做首帧 yaw 归零。

为匹配当前生产语义，每个ball snapshot使用收到该球消息时“最近一份有效pelvis”。同时记录两者的source frame差和source time差；不因为诊断层要求严格同帧而改变基线planner输入。若frame不一致或pelvis过旧，额外报告 `PELVIS_FRAME_MISMATCH`，但是否拒绝规划仍以当前生产 `base_valid` 语义为准。

### 5.2 三类时间

每条标准化输入按LCM真实到达顺序分配全局递增 `input_seq`，并保存：

- `source_time_s`：ChingMu源时间；有效且大于0时，作为estimator拟合时间。
- `publish_time_us`：source time无效时，作为estimator拟合时间。
- `received_monotonic_s`：本机单调时钟，用于耗时、TTS和deadline。
- `wall_time_us`：显示和跨文件定位。

estimator时间源严格复制生产优先级：

```text
有效 vicon_time_s
→ 有效 publish_time_us
→ 按配置sample rate递增的nominal fallback
```

禁止直接用 `wall_time - source_time` 判定网络延迟。两种时钟存在固定偏移时，只报告 `CLOCK_OFFSET_SUSPECTED`，不报告真实链路延迟。

### 5.3 频率

- LCM线程按真实消息顺序完成解码、pelvis状态更新和ball estimator更新，匹配当前生产handler；所有有效球帧进入estimator，名义输入约360 Hz。
- 当前基线最多100 Hz提交 planner worker。
- lifecycle和task observation按50 Hz推进。
- 浏览器刷新限制为5–10 Hz，阶段变化和终态事件立即推送。
- 完整标准化重放输入由独立raw记录通道交给后台线程写入，不在LCM回调中做JSON编码或磁盘刷新。

这里的“完整标准化输入”不是原始LCM bytes，而是无损保存诊断和重放所需的全部解码字段、float64数值、顺序和三类时间。如果raw记录通道丢失任何输入，立即增加 `raw_samples_dropped`，把当前attempt和session标记为 `RECORDING_INCOMPLETE`，并禁止对受影响attempt给出A/B结论。

默认健康阈值为：channel heartbeat（由持续table消息提供）超过0.10秒未更新、latest pelvis超过0.05秒、同一subject source frame delta大于1。它们均可通过CLI覆盖并写入session配置，只产生warning，不修改shadow生产状态。

## 6. 一颗球的身份与状态机

不能直接把 `track_epoch` 当作“一颗物理球”：当前 invalid ball 消息会先增加 epoch，再发布不可见 snapshot；bridge单帧找不到candidate也会立刻结束track；deadline reset还可能增加epoch。

诊断层同时维护两个身份：

- `track_segment_id`：严格跟随生产invalid/reset；每个segment有自己的estimator epoch、incoming状态和planner结果。
- `attempt_id`：仅用于把一次人工投球的短时断轨、重新捕获和post-deadline尾迹归在同一行展示。

规则：

1. 当前没有开放attempt，球出现可见上升沿：创建新 `attempt_id` 和 `track_segment_id`。
2. 收到invalid：立即按生产语义结束当前segment并reset estimator；invalid snapshot仍经过现有100 Hz throttle、worker和50 Hz消费，只有实际到达相应生产检查点时才reset incoming/通知lifecycle，不能由展示层提前执行。attempt进入 `REACQUIRE_GRACE`，暂不关闭。
3. 默认grace为 `0.20 s`。grace内重新可见：创建新segment并归入原attempt；超过grace仍不可见才关闭attempt。
4. 已经PASS后grace内的重新可见段标记为 `POST_DEADLINE_TAIL`，不创建新attempt，也不暗示发生了真实击球。
5. 页面健康timeout和LCM stale只产生warning，不驱动shadow estimator/lifecycle reset；只有生产语义中的明确invalid/reset才改变segment。
6. 每个snapshot/result通过 `(track_epoch, generation)` 映射到segment和attempt。异步结果不得只按当前可见attempt猜测归属。

主状态机：

```text
WAITING
→ DETECTED
→ ESTIMATING n/31
→ PLANNER-READY INCOMING n/3
→ PLANNED
→ ARMED
→ SHADOW TASK_OBS_PASS
```

incoming计数明确表示“实际进入worker plan_fn且调用 `observe()` 的planner-ready snapshot”，不是连续ChingMu原始帧。baseline严格复制当前跨not-ready/base-invalid/bounce空档保留计数以及confirmed latch的语义，不在诊断工具里悄悄修正。

attempt展示状态与全局lifecycle分开。上一颗仍在 `RECOVERY` 时，下一颗可以显示：

- `WAITING_FOR_PREVIOUS_RECOVERY`
- `CACHED_DURING_RECOVERY`

shadow pipeline必须复制active/cached result、recovery结束后重新ingest cached result，以及strike deadline后reset estimator/推进epoch的生产行为。正常的estimator积累、incoming确认、100 Hz节流、recovery等待和单个snapshot被拒绝都显示为进度或warning，不立即成为attempt终态失败。

## 7. Incoming 最小A/B检查

### 7.1 为什么先查 `3 → 1`

当前100 Hz提交下，从第一份合格snapshot到第三份合格snapshot，理论至少增加20 ms，360 Hz源上的典型跨度约22.2 ms。对高速球，这可能把首次结果从可ARM推迟到 `LATE_SKIP`。

`3 → 1` 仍保留：

- `vx <= -minimum_stable_incoming_speed_x_mps`；
- 每epoch reset；
- confirmed latch；
- 下游planner的 `vx < 0` 和未来有向交点约束。

它只移除额外两次防抖。完全删除incoming还会移除 `-0.20 m/s` 门槛，不作为第一阶段改动。

### 7.2 比较方式

实时页面以shadow生产语义 `3/100` 为主链路。attempt关闭且当前没有新球时，独立低优先级replay进程用同一份完整记录依次执行：

1. 重放 `3/100` 并与线上shadow baseline逐阶段对齐。
2. 若阶段、结果身份、ARM/LATE或task observation不一致，标记 `REPLAY_DIVERGENCE`，本attempt不给出反事实结论。
3. baseline一致后，再重放 `1/100`。

replay使用离散事件scheduler和fake monotonic clock，显式模拟：

- 原始输入到达顺序；
- 100 Hz提交相位；
- 每次submit、plan start、plan complete及worker capacity-one pending/in-flight/latest-result；
- 每个50 Hz tick的 `lifecycle_now_s` 与 `obs_now_s`；
- deadline reset和recovery/cached语义。

planner命令在replay进程中真实计算。线上baseline已经记录的planner duration直接复用；`1/100` 新增的早期完整规划调用使用本次离线实测duration，并把它写入分析记录。距离ARM阈值不足5 ms的结论标记 `BOUNDARY_SENSITIVE`，不当作稳定证据。单元/集成测试使用脚本化planner duration，不使用真实sleep。

每个variant独立保存：

- `TASK_OBS_PASS`
- `LATE_SKIP`
- `TRACK_ENDED_BEFORE_READY`
- `TRACK_ENDED_BEFORE_CONFIRM`
- `NO_VALID_PLAN`
- `PELVIS_UNAVAILABLE`
- `MALFORMED_COMMAND`
- `RECORDING_INCOMPLETE`
- `REPLAY_DIVERGENCE`

并比较：

- estimator首次ready时间；
- incoming首次confirmed时间；
- planner首次成功时间；
- lifecycle首次ARMED时间；
- ARM时剩余TTS；
- task observation首次PASS时间；
- planner提交、完成、失败、pending覆盖数；
- base/racket目标变化量。

摘要标签从两个独立outcome和数值delta派生，不替代原始结果：

- `SAME_PASS`：两者都PASS且目标差异在记录的数值容差内。
- `SAVED_BY_ONE_FRAME`：`1/100`可达到task PASS而`3/100` late/track-ended。
- `BASELINE_ONLY_PASS`：`3/100` PASS而 `1/100` 未PASS。
- `BOTH_FAIL_SAME` / `BOTH_FAIL_DIFFERENT`。
- `BOTH_PASS_DIFFERENT_COMMAND`：两者都PASS但command/目标不同。
- `INCONCLUSIVE`：记录不完整、baseline重放分歧或边界敏感。

另附正交warning `ONE_FRAME_UNSTABLE`：首份vx合格，但接下来两份100 Hz候选中至少一份vx不合格、nonfinite或完整planner失败。所有variant还记录 `delta_confirm_ms`、`delta_arm_ms`、`delta_arm_tts`、base/racket target delta，避免只看摘要标签。

### 7.3 360 Hz planner的升级条件

第一阶段始终保存360 Hz标准化输入，但不实时运行完整360 Hz planner。只有当一批记录完整的attempt显示：

1. `1/100`仍不能解决；
2. 首次可规划snapshot已足够早；
3. 100 Hz提交相位是主要剩余延迟；

才使用现有记录做第二阶段360 Hz离线重放，再讨论生产改动。

当前本机只读基准显示：正常planner约2.4–3.1 ms，困难的5秒无交点轨迹约10.6 ms；360 Hz周期仅2.78 ms。因此不能在没有CPU、覆盖和50 Hz消费证据时直接提升生产频率。

## 8. 并发与非阻塞保证

职责分离：

- LCM线程：按生产顺序解码、更新latest pelvis、运行ball estimator并形成不可变snapshot；只把标准化输入和小事件交给非阻塞通道，不做磁盘或HTTP工作。
- planner线程：沿用capacity-one、latest-only语义。
- 50 Hz主循环：依次捕获lifecycle time、消费latest result并推进lifecycle，再捕获observation time和latest pelvis组装task observation。
- recorder线程：写CSV/JSONL。
- HTTP线程：生成状态快照和SSE推送。
- replay独立进程：只有不存在active/reacquire attempt时才开始；新attempt到来就暂停当前分析并留待下一个空闲窗口，避免与实时Python线程争GIL。待分析attempt保存在磁盘索引中，不依赖无界内存队列。

raw记录通道与诊断event通道完全分离：

1. `RawRecordLane` 使用独立高容量有界队列。满时不阻塞LCM，但增加 `raw_samples_dropped`，使相关attempt的replay结论失效。
2. 所有生产线程通过一个极短、线程安全的 `publish(event)` 临界区更新诊断状态：在同一把锁内分配全局递增 `event_id`、用纯reducer生成新的immutable `DiagnosticStateStore` 快照、再append到ring。replay进程只能经父进程IPC请求publish。
3. `EventHub` 保存有界不可变ring；recorder和每个SSE客户端按独立cursor读取，不能通过竞争 `queue.get()` 抢走彼此事件。HTTP只读取原子替换后的immutable state snapshot。
4. progress在进入 `publish()` 之前按 `(attempt_id, stage)` 的producer mailbox/rate gate合并；事件一旦获得event id就永不修改，只能从ring头整体淘汰。
5. subscriber落后到ring已覆盖时，标记自身drop。recorder drop使session记录不完整；SSE drop则发送reset事件，客户端重新GET当前state。
6. 任何sink长期停滞都不阻塞LCM、estimator或planner。

planner输入和结果、attempt快照、task observation进入跨线程队列前必须复制为只读数据。网页线程不得直接读取 estimator、worker、lifecycle 的私有可变成员。

默认资源上限：`RawRecordLane=65536` 条、event ring `8192` 条、内存attempt detail cache最近 `100` 条、单事件序列化后最大 `64 KiB`、异常文本最大 `2048` 字符。均可由CLI降低或提高并写入session；达到上限时按上述drop/reset规则处理，不能让内存随运行时间无界增长。

recorder每轮最多交替drain 512条raw和128条event，避免任一路长期饿死另一路。raw drop通知直接publish到EventHub，不写回已满raw lane，并保存丢失的 `input_seq` 区间及当时attempt id。

## 9. 失败分类

snapshot rejection、warning和attempt终态严格分开。pelvis短暂无效、bounce、incoming不合格、单代planner无交点/高度越界、stale result等都可能恢复，只进入时间线，不立即成为 `primary_failure`。

attempt关闭时按到达的最远阶段回溯选择主结果：

1. 任一segment已经task PASS：attempt成功，无primary failure。
2. estimator从未ready：`TRACK_ENDED_BEFORE_READY`。
3. ready但从未confirmed：`TRACK_ENDED_BEFORE_CONFIRMATION`。
4. confirmed但从无有效plan：选择最后一个可解释planner rejection，否则 `NO_VALID_PLAN`。
5. 有plan但未ARM：优先 `LATE_SKIP`，否则 `TRACK_ENDED_BEFORE_ARM`。
6. 已ARM但task未PASS：选择明确的obs validation failure。

所有较早或后续事件仍完整保存在时间线中。

### 9.1 输入/追踪

- `LCM_HEARTBEAT_STALE`
- `BALL_INVALID_OR_OCCLUDED`
- `PELVIS_INVALID`
- `PELVIS_FRAME_MISMATCH`
- `SOURCE_FRAME_GAP`

`BALL_INVALID_OR_OCCLUDED`、`PELVIS_INVALID`、frame gap和frame mismatch首先是健康/时间线事件；短暂异常恢复后允许继续。heartbeat/age阈值只影响页面健康状态，不主动改变shadow生产状态机。

### 9.2 Estimator

- `ESTIMATOR_WARMING`：非终态进度
- `BOUNCE_RESET`：正常重回ESTIMATING
- `ESTIMATE_NONFINITE`
- `TRACK_ENDED_BEFORE_READY`

### 9.3 Incoming

- `INCOMING_CONFIRMING`：非终态进度
- `INCOMING_SPEED_REJECTED`
- `TRACK_ENDED_BEFORE_CONFIRMATION`

### 9.4 Planner

- `BALL_X_NOT_AHEAD`
- `BALL_NOT_INCOMING`
- `NO_DIRECTED_CROSSING`
- `HIT_HEIGHT_OUT_OF_RANGE`
- `NONFINITE_TRAJECTORY`
- `MALFORMED_COMMAND`
- `PLANNER_EXCEPTION`
- `NO_VALID_PLAN`

异常必须记录原始异常类型、文本、snapshot标识和相关数值，不能只增加累计 `failed`。

### 9.5 Lifecycle/task observation

- `TRACKING_TOO_EARLY`：非终态进度
- `LATE_SKIP`
- `STALE_AFTER_RESET`
- `TRACK_ENDED_BEFORE_ARM`
- `OBS_WRONG_SHAPE`
- `OBS_NONFINITE`
- `OBS_CLIPPED`
- `OBS_TTS_OUT_OF_RANGE`
- `OBS_COMMAND_MISMATCH`

bridge断流或pelvis无效不会让整个诊断进程崩溃；页面进入断开状态，恢复后继续接收下一颗球。

## 10. 页面与只读接口

### 10.1 页面

顶部健康栏：

- LCM接收Hz和消息年龄；
- ball/pelvis最近source frame；
- pelvis有效性和年龄；
- planner submitted/completed/failed/dropped；
- 当前phase；
- diagnostics自身丢事件数；
- 当前配置名和会话目录basename。

当前球：

```text
球检测
→ estimator 31/31
→ planner-ready incoming 3/3
→ planner
→ lifecycle ARMED
→ SHADOW 3/100 TASK OBS 11/11 PASS
```

如果上一segment仍在recovery，当前attempt显示 `WAITING_FOR_PREVIOUS_RECOVERY` 或 `CACHED_DURING_RECOVERY`，不把等待误报成planner失败。

颜色语义：

- 灰：未到达；
- 蓝：正常进行；
- 绿：通过；
- 红：终态失败；
- 黄：输入/系统暂不可用。

历史列表每颗球一行，显示球速、primary blocker、predicted strike、planner TTS、ARM TTS、task obs结果和A/B结论。展开后显示完整时间线、11维task observation、planner输入输出及异常。

### 10.2 HTTP接口

只提供：

- `GET /`
- `GET /api/state`
- `GET /api/attempts?limit=50&before=<attempt_id>`
- `GET /api/attempts/{attempt_id}`
- `GET /events`（SSE）

attempt列表默认按 `attempt_id desc` 排序；`before` 是exclusive游标。`limit` 默认50、最大200；attempt id只接受正整数。错误统一返回JSON格式的400/404/405，包含正确 `Content-Type`、`Content-Length` 和 `Cache-Control: no-store`。静态文件只映射固定 `/` 及明确列出的资源，禁止任意路径文件读取。

SSE只作为invalidator，不传增量状态patch：

- fresh client收到 `ready` 和当前watermark，然后GET `/api/state`；
- 可续传client从 `Last-Event-ID + 1` 补发 `invalidate`；
- 若ID已被ring覆盖，服务发送 `reset` 和当前watermark，客户端重新GET；
- `invalidate` 只包含受影响scope和attempt id，页面收到后debounce GET完整快照；
- 每15秒发送heartbeat。

由于旧事件不会作为状态patch应用，补发不会令UI状态倒退。最多允许4个SSE客户端，断开时必须释放线程和cursor。

服务不启用wildcard CORS。前端把错误和LCM文本写入DOM时只用 `textContent`，不使用 `innerHTML`。不提供POST、reset、配置修改、进程控制或真机控制接口。默认绑定 `127.0.0.1:8765`，端口可由CLI覆盖，启动日志打印实际地址。

HTTP总handler并发上限8，其中SSE上限4；超限返回503。普通请求设置5秒读写timeout，校验 `Host` 只允许 `127.0.0.1`、`localhost` 和实际监听端口。HTML/API响应使用 `Cache-Control: no-store`，SSE使用 `no-cache`，全部响应添加 `X-Content-Type-Options: nosniff`。

每个SSE客户端只消费只读事件序号和invalidator；慢客户端最多丢失中间progress invalidation，随后通过reset或下一次GET恢复当前状态，不能反向阻塞event bus或实时pipeline。

### 10.3 JSON契约

`GET /api/state` 返回完整替换快照：

```json
{
  "schema_version": 1,
  "watermark_event_id": 123,
  "health": {},
  "lifecycle": {},
  "current_attempt": null,
  "recent_attempts": []
}
```

`GET /api/attempts` 返回：

```json
{
  "schema_version": 1,
  "items": [],
  "has_more": false,
  "next_before": null
}
```

`GET /api/attempts/{attempt_id}` 返回immutable `AttemptDetail`，至少包含 `attempt_id`、segments、stage timeline、primary blocker、planner inputs/results、pre/post-clip task obs、variant outcomes和A/B deltas。

SSE只发送以下事件类型：

```text
ready      {"watermark_event_id": 123}
invalidate {"scope": "state|attempt", "attempt_id": 7}
reset      {"watermark_event_id": 456}
heartbeat  {"watermark_event_id": 456}
```

所有JSON字段使用固定schema version；未知字段可忽略，缺失必需字段是协议错误。

## 11. 持久化

每次运行创建独立目录：

```text
recordings/hitter_task_diagnostics/YYYYMMDD_HHMMSS_ffffff-p<PID>-<short_uuid>/
├── session.json
├── ball_samples.csv
├── events.jsonl
├── replay_analysis.jsonl
├── attempts.csv
└── attempt_details/
    └── <attempt_id>.json
```

CLI可用 `--output-dir` 覆盖根目录。运行记录不作为源码提交。

实现时在仓库 `.gitignore` 中只加入 `recordings/hitter_task_diagnostics/`，不忽略或修改用户现有的其他 `recordings/` 内容。

- `session.json`：resolved配置内容/hash、标定JSON路径/hash、LCM URL、git commit/dirty、Python与关键包版本、启动参数、阈值和schema version。
- `ball_samples.csv`：每个ball/pelvis标准化输入的 `input_seq`、全部消息字段、publish time及三类时间。
- `events.jsonl`：结构化阶段变化、错误、每次planner submit/start/complete、worker覆盖、每个50 Hz lifecycle/obs tick及task obs事件。
- `replay_analysis.jsonl`：variant outcome、虚拟调度事件、duration模型、数值delta与摘要标签。
- `attempts.csv`：每颗球一行最终汇总；正常情况下在A/B完成后写入。退出时仍pending的attempt写 `INCONCLUSIVE`，不留无结果的半行。
- `attempt_details/<id>.json`：attempt关闭/分析完成后原子替换的完整immutable详情；HTTP详情接口先查最近100条内存cache，再按严格正整数文件名读取，绝不扫描整个events日志或接受任意路径。

会话目录使用exclusive create避免并发碰撞，目录权限0700、文件0600。文件采用追加写入，每1秒或每1000行flush一次，attempt终态和session终态立即flush。磁盘满或写异常只降级recorder、页面亮黄灯并标记 `RECORDING_INCOMPLETE`，不停止实时pipeline。页面显示已写字节数和磁盘剩余空间；不在工具内自动删除历史记录。进程异常退出后，已有完整行仍可读取。HTTP API不提供任何会话文件下载接口，也不暴露绝对路径字段。

## 12. 测试设计

后续实现严格按TDD推进，先建立失败测试。

### 12.1 单元测试

1. task observation：
   - 11维顺序、shape、dtype；
   - yaw-only坐标变换；
   - world-frame racket velocity保持不变；
   - 50 Hz tick时latest pelvis而不是planner snapshot pelvis；
   - 保留 `lifecycle_now_s` 与 `obs_now_s` 两次时钟读取顺序；
   - pre/post clip及 `OBS_CLIPPED`；
   - nonfinite和TTS边界；
   - 与 `HitterEnv` 当前 `obs[6:17]` 数值逐项一致。

2. attempt tracker：
   - visible上升沿创建；
   - invalid消息epoch提前增加时segment身份不误关联；
   - 0.20秒grace内断轨重捕获仍属于同一attempt但创建新segment；
   - grace外重新可见创建新attempt；
   - post-deadline尾迹不创建新attempt；
   - active/cached attempt在recovery期间正确映射；
   - 健康timeout不驱动生产状态reset。

3. incoming A/B：
   - 3次连续合格；
   - 1次立即确认但保留速度门槛；
   - 不合格vx重置；
   - baseline严格保留当前bounce/not-ready/base-invalid空档及latch语义；
   - 每个variant outcome、完整摘要标签和数值delta；
   - `ONE_FRAME_UNSTABLE`与 `BOUNDARY_SENSITIVE` warning。

4. event bus/recorder：
   - raw lane与event hub隔离；
   - 原子publish、全局event id、immutable ring和每subscriber独立cursor；
   - progress只能在publish前合并；
   - recorder/SSE不竞争消费；
   - ring覆盖、raw drop和 `RECORDING_INCOMPLETE`；
   - raw/event公平drain及丢失input_seq区间；
   - CSV/JSONL、attempt detail schema、权限和flush。

5. web：
   - GET分页、参数上限和固定静态路由；
   - SSE ready/invalidate、heartbeat、Last-Event-ID补发与reset；
   - invalidator触发完整GET且不应用旧状态patch；
   - 慢客户端、HTTP/SSE最大连接数、timeout和断开清理；
   - 400/404/405/503 JSON、cache/header、Host校验、无wildcard CORS；
   - attempt detail从cache/固定磁盘索引读取；
   - 页面断开不影响pipeline。

### 12.2 集成测试

1. 合成360 Hz来球：
   - 完整通过31帧、planner-ready incoming、planner、ARMED和shadow 11维PASS。
2. 高速late球：
   - 当前 `3/100` late；
   - 同一输入 `1/100` ARM并形成shadow task PASS；
   - 汇总为 `SAVED_BY_ONE_FRAME`。
3. 非来球、无交点、高度越界、pelvis无效和track提前结束。
4. 单帧invalid重捕获、多segment同attempt以及grace外新attempt。
5. 多颗连续来球、上一颗recovery与下一颗cached，验证attempt隔离和日志映射。
6. 固定容量latest-only worker覆盖、result覆盖、deadline reset和50 Hz消费相位。
7. 虚拟时钟重放 `3/100` 必须先复现线上baseline；不一致时输出 `REPLAY_DIVERGENCE`。
8. 录制文件和脚本化planner duration应可重复得到相同阶段、reason code和task observation。
9. 距离ARM边界不足5 ms的重放结果必须标记 `BOUNDARY_SENSITIVE`。

### 12.3 性能与安全测试

1. 以360 Hz合成输入持续60秒，正常负载下 `raw_samples_dropped == 0`，且包含estimator的LCM handler p99低于2.78 ms源周期。
2. 浏览器刷新限制在5–10 Hz。
3. replay只在无active/reacquire attempt时运行；新attempt使其暂停，实时raw/event积压不得增加。
4. 测试进程不得发布LCM控制频道。
5. 不启动 `trans`、ONNX session或真机step也能完成完整诊断。
6. 验证退出顺序：停止接收新LCM → 关闭/join planner与replay → drain recorder → 写session终态并flush → shutdown HTTP → join所有线程；每步有有限timeout。

## 13. 验收标准

实现完成后必须满足：

1. 只启动 ChingMu bridge和诊断进程即可打开页面。
2. 页面明确显示ball和`G2Pelvis`健康状态。
3. 短时断轨的一次人工投球显示为一个attempt，同时保留每个生产segment；正常负载下保存完整标准化重放输入。
4. 每颗球逐阶段展示31帧、planner-ready incoming、planner、lifecycle和shadow task observation。
5. 失败球显示按最远阶段回溯出的primary blocker、精确reason code和关键数值。
6. 成功球至少出现一次 `ARMED + SHADOW 3/100 TASK OBS 11/11 PASS`，页面明确声明这不是生产policy消费证明。
7. replay完成后展示 `3/100` 与 `1/100` 的独立outcome、数值delta和摘要；记录不完整或重放分歧时显示 `INCONCLUSIVE`。
8. 页面、记录器或replay失效时，不影响LCM实时接收和planner主链路。
9. 全部新增测试和当前工作树中仍存在的相关测试通过；不恢复已由用户删除的旧测试。
10. 没有加载policy、发布action或控制机器人。

## 14. 后续决策

诊断工具交付后，用户连续投掷不同速度的来球。先依据逐球证据判断：

1. 若多次出现 `SAVED_BY_ONE_FRAME`，再单独设计和测试生产配置 `3 → 1`。
2. 若 `1/100`与当前相同，优先检查31帧estimator、首次可见距离、minimum arm TTS及坐标/轨迹约束。
3. 只有100 Hz提交相位被证明确为主要剩余瓶颈时，才对已记录数据做360 Hz planner离线重放。
4. 任何生产配置或控制链路改动都作为独立变更处理，不由本诊断工具自动执行。
