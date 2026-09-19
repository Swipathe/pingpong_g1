# Task 7 交付报告：RealWorld authoritative v2 consumer

## 结论

已在 RobotBridge4 实现并提交 authoritative `vicon_state_data_v2` consumer，提交为 `d9dd20b30ecc8981c7ae8564b84a32ec044bcf26`（`feat: consume authoritative HITTER track ids`）。本任务只运行离线单测和语法编译，没有启动网络、真机或 PD。

## Fix round 1 补充

初始提交经独立审查后已由 `e5ad3369de85db37920be74e7d95b5d0af53a86b`（`fix: harden authoritative HITTER consumer`）修正。累计 Task 7 范围为 `49efb76..e5ad336`；下方初始报告中关于“commit 保持 G1、依赖工作树 G2 overlay”的描述仅记录首版状态，已不再适用于最终提交。

- `ViconConsumerSettings.base_subject` 默认精确为 `G2Pelvis`，支持严格显式 subject 配置；大小写或空格不匹配永久 fail closed。
- conflict 对首次/已见 challenger 和递增/重复/倒序 frame 使用同一语义：消费双方、保留 incumbent active authority、切断 estimator，且不通知 planner。
- invalid end 只有 active 同 ID 或 last+seen 的重复 end 合法；unknown/旧非 last ID 永久 schema fault。
- wire frame、ID、flags、pose/quaternion 与 table valid 合同在刷新 freshness 前完整校验。
- reentry/conflict 会同步 diagnostics snapshot 的 `consumed=True`，不推进 generation、不回调 listener。
- transition queue 容量严格为 `[1,64]`；overflow 原子替换为一个 sticky、永久 `VICON_SCHEMA_ERROR` event，不能递归或被后续事件替换。
- 修复测试按组先得到 `25 failed`、warning case `1 failed`、identity case `4 failed`、capacity case `1 failed`；最终当前工作树 consumer+connection-wait+factory `81 passed`。从最终 HEAD 导出的无 `.git` 干净 archive 中 consumer `67 passed` 且 `py_compile` 通过，证明不依赖用户 overlay。

## Fix round 2 补充

最终提交为 `1580fcb361fcb5d5acca0c47871c2faac26d0065`（`fix: close HITTER schema fault planner gate`），累计 Task 7 范围为 `49efb76..1580fcb`。

- 任意永久 `VICON_SCHEMA_ERROR`（含 transition queue overflow）都在同一状态锁内消费 active ID、同步 latest snapshot 的 consumed 状态并清 estimator/readiness，同时保留 active/visible/last 诊断身份。故障后的同 ID 与新 ID 都不能进入 estimator 或 planner listener。
- seen/unseen challenger conflict 都用 max 语义更新 challenger frame watermark，不推进 challenger generation，也不夺取 incumbent authority；随后较小 frame replay 被拒绝。
- 修复轮 2 定向 RED 为 `3 failed`，GREEN 为 `3 passed`；当前树 consumer+connection-wait+factory 为 `80 passed`，干净 archive consumer 为 `66 passed`，`py_compile` 通过。
- 独立静态复审结论为 PASS，无 Critical/Important。唯一 deferred Minor 是 listener tuple 在锁内捕获、锁外调用之间可能遇到并发 consume；Task 9 的最终 lifecycle admission gate负责再次核验 consumed/fault。

## TDD 证据

- RED：先创建 `deploy/tests/test_real_world_v2_consumer.py`，首次运行得到 `32 failed`。失败点包括缺少 `_ingest_vicon_v2_message()` / status API、坏 fingerprint 异常外泄、仍订阅 v1 channel、缺少 consumer settings。
- GREEN 第一轮：最小实现后 `32 passed`；修正项只有生产初始化缺少 world-pose 默认值、stale 后 `ball_fresh` 诊断语义，以及测试中“被 quarantine 的 active ID 超过 0.40s 仍应 active”的错误预期（合同要求其 ball-stale 结束）。
- 提交前 fresh verification：consumer + 只读 connection-wait + runtime-factory 共 `46 passed, 2 warnings in 0.93s`；`py_compile` 与 `git diff --cached --check` 均成功。两个 warning 是导入链中的既有 invalid-escape deprecation warning。

## 场景矩阵

| 边界 | 离线覆盖与结果 |
| --- | --- |
| schema/channel | 只订阅精确 `vicon_state_data_v2`；坏 fingerprint 被 callback 最外层捕获并永久锁存 schema fault，随后 RC handler 仍可处理 R2；无 v1 fallback、无逐球 print。 |
| subject-ID | ball 只接受正 ID；pelvis/table 只接受 0；0、负数和非零 base/table ID 均 fail closed，estimator sample 保持 0。 |
| frame/generation | 每 ID generation 从 1 开始；同 ID 仅严格递增 frame 增长；duplicate/backward 不增长 generation、不加 sample；外部 estimator reset 不改 active/last/history/seen/consumed。 |
| estimator isolation | 任意首次 ID 在 active/admission 前清旧窗口；ID 7 ready 后 invalid，再经连续 0.50s 接受 ID 8 时 generation=1、sample_count=1、ready=false，拟合窗口只含 ID 8；quarantine ID 同样切窗但不加自己的 sample。 |
| conflict/consumption | overlapping 新 ID 会消费 old+new、切 estimator、锁存并排队 `TRACK_ID_CONFLICT`；consumed snapshot 仍供 diagnostics，但 listener 不收到。 |
| invalid/direct event | 仅当前 active 同 ID 的 invalid 能结束；active 置空、last/history 保留、visible=false、no-ball 起计，并且只产生一次 `TRACK_ENDED`。base valid->invalid 只产生一次 `BASE_POSE_INVALID`。 |
| freshness | stream 与 active-ball 阈值均为严格 `age > 0.40s`；`0.40` 不触发，`0.400001` 触发；stale 结束 active、base fail closed、fault/event 去重，普通恢复包不自行清 latch。 |
| reentry | 未恢复 stream/base 或永久 schema fault 时拒绝且不清 fault；fresh base 且无 schema 时才清可恢复 stale/conflict、开启 session，并 quarantine 当时可见 ID；无球从 reentry 时刻重新计时。 |
| 0.50s admission | ready 严格由 session open、无 latch、stream fresh、base valid、无 active/visible、连续 no-ball >=0.50s 构成；不读取 HitterEnv phase。0.49s 到达的新 ID 立即 consumed，之后不复活；任何 valid ball 都清 no-ball timer。 |
| listener/locks/events | listener 只接 planning-eligible snapshot，首次 admitted 为 `new_track=true`、后续 false，`consumed` 反映真实状态；取 listener 列表后在 ball lock 外回调；event deque sequence 单调，drain 在锁内原子返回并清空。 |

## 配置与接口

- 新增 `ViconConsumerSettings` 与严格解析：channel 必须精确为 v2，`stream_timeout_s` / `ball_timeout_s` / `new_serve_no_ball_s` 必须 finite 且正数；默认分别为 `0.40 / 0.40 / 0.50`。
- 新增 `ViconInputFault`、`ViconEventReason`、`ViconConsumerEvent`、`ViconConsumerStatus`、`begin_hitter_policy_session()`、`hitter_vicon_status()`、`drain_hitter_vicon_events()`、`consume_hitter_track()`。
- 未提前创建 Task 8 的 `LifecycleCancelReason`，也未修改 HitterEnv phase gate。

## 暂存、提交与 dirty-tree 证明

- 提交只含：`M deploy/simulator/real_world.py`、`A deploy/tests/test_real_world_v2_consumer.py`、`M deploy/utils/hitter_runtime_factory.py`。
- 使用交互式 `git add -p`；cached commit 中没有连接等待/R2 helper/calibration、G2 user overlay、velocity-range hunks，也没有未跟踪 `deploy/tests/test_real_world_connection_wait.py` 或本报告。
- 提交后仍为用户保留：`real_world.py` unstaged `31/21`（原连接等待、G2、R2，加上 Task 7 新事件 detail 的 G2 overlay），`hitter_runtime_factory.py` unstaged `4/0` velocity-range，`test_hitter_runtime_factory.py` 仍 modified，`test_real_world_connection_wait.py` 仍 untracked。未 restore/reset/clean，未恢复任何 deletion。

## 自审 concerns

- brief 示例在 pelvis 1.41 后直接检查 1.92 ready，与 0.40s stream freshness 合同矛盾；测试在 1.91 补一帧 pelvis，以 authoritative freshness 为准。
- 旧 `test_hitter_task_input_adapter.py` 的 RealWorld case 仍显式发送 `vicon_state_data`；它属于被本任务“无 v1 fallback”替代的旧合同，且不在 Task 7 files，所以未编辑/暂存/纳入本任务指定回归。
- 提交 index 保持 HEAD 的 G1 subject 名称，工作树继续保留用户既有 G2 overlay；本机 fresh tests 针对实际工作树 G2 运行。后续若要独立检出该提交运行真机，必须同时保留/整合用户的 G2 部署选择。
