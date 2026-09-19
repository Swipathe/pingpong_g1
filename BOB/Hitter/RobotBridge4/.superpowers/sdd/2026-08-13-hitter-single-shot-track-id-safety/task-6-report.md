# Task 6 交付报告：planner completed result 有序队列

## 状态与提交

- 状态：完成。
- 提交：`b5f8c82 feat: queue completed HITTER planner results`
- 提交文件仅有：
  - `deploy/utils/hitter_realtime.py`
  - `deploy/tests/test_hitter_completed_result_queue.py`
  - `deploy/tests/test_hitter_runtime_identity_types.py`

## RED / GREEN 证据

- RED：首次运行 `PYTHONPATH=deploy ... pytest -q deploy/tests/test_hitter_completed_result_queue.py`，得到 `25 failed`。失败明确来自缺少 `drain_completed_results`、`completed_result_queue_capacity`、`CompletedResultBatch`、queue stats、typed reason/error_text 和严格 result invariant。
- GREEN：最小实现后同一命令得到 `25 passed in 0.16s`。
- brief 指定 queue + safety：`30 passed in 1.77s`。

## 合同实现与证据

- pending input 仍是深度 1 的 latest-only：Event 有界测试证明 generation 1 阻塞时提交 2/3，实际执行顺序是 `[1, 3]`，`pending_replaced_total == 1`，`max_pending_depth == 1`。
- completed output 默认容量 64，容量仅接受 `type is int and > 0`；bool、float、0、负数均拒绝。
- completion 按完成顺序 append；队列满时保留旧 pair、丢弃新 pair，并累计 since-drain count、sorted unique 正 track id 及永久 total。
- `drain_completed_results()` 在同一 condition lock 内原子复制全部 `(result, frozen)` pair 和 since-drain overflow 元数据，随后只清空队列、since-drain 字段和 latch；total 不清零。
- `PlannerResultSnapshot` 保证 command/reason 恰为成功或失败二选一，并验证正 track/frame、非负 generation、有限 completion 时间及 deadline 的有限/NaN 边界；保留旧 `error` 只读兼容视图。
- `PlannerRejected` 保留 `exc.reason` 与 `exc.detail`；未知异常转 `INTERNAL_ERROR` 和带异常类型的 `error_text`。测试证明 unknown 后下一条仍成功完成。
- `FrozenPlannerResult` 同步 `failure_reason`；测试证明 frozen command 字段与之后被修改的原 ndarray 解耦且映射不可变。
- 所有 worker 测试的 `finally` 均释放 Event 并断言 `close(timeout_s=1.0)`，未遗留 worker thread。

## 最终验证

- 最终直接影响集合（queue、diagnostics safety、Task 5 planner failure、identity、diagnostics pipeline、runtime factory、strike logging）：`109 passed in 2.72s`。
- `python -m py_compile`：3 个变更 Python 文件通过。
- `git diff --check`：通过。
- 最终测试中的 2 条 `DeprecationWarning: invalid escape sequence` 来自既有动态代码（`<unknown>:4`）；新 queue 测试单独运行无 warning，本 diff 未新增 warning。

## staging 与用户脏改证明

- commit 前 `git diff --cached --name-status` 仅列出上述 3 个任务文件；无 plan、report、safety 用户 hunk。
- `deploy/tests/test_hitter_task_diagnostics_safety.py` 提交前后保持未暂存：blob `97224be4ec1d28495b3d796c2b4efdd75431734a`，`+2/-2`，diff SHA256 `7e77f15ca9eeafb4253aea301d39de728e4f175d102a24b219b56699d593a692`。
- 本报告位于 `.superpowers/` ignored 路径，仅本地保存，未提交。

## 自审 concerns

- `latest_result_bundle()` 仍保留给旧 controller/diagnostics 只读兼容；本任务没有提前迁移 HitterEnv 或 diagnostics 消费路径。
- 单独探索运行用户已修改的 `test_hitter_task_diagnostics_integration.py` 时，其两条 `G1Pelvis -> G2Pelvis` 未提交改动导致 2 个既有场景失败；未修改、未暂存该用户文件。任务最终回归使用 brief 指定 safety 与未受该脏改影响的 diagnostics pipeline 集合。
- 未运行真机、PD 或 `unitree_sdk2/build`。

## Review fix round 1/5

### 状态与提交

- review findings 已修复。
- 独立提交（未 amend）：`808befe fix: make HITTER planner delivery reentrant`。
- fix commit 包含 9 个必要文件；HitterEnv、runtime factory test、diagnostics integration test 均只通过 partial staging 提交本轮 constructor 迁移 hunk。

### RED / GREEN

- 首先新增 3 个定向测试：listener callback 内有界 close、canonical derived `.error`、constructor 禁止 `error=`。当前实现稳定得到 `3 failed`：close 首次为 `False`、INTERNAL_ERROR 的 `.error` 被 detail 覆盖、签名仍暴露 `error`。
- 核心修复后定向测试 `3 passed`，queue 全集 `28 passed`。
- constructor 切换后直接影响调用点首次回归 `9 failed, 50 passed`，失败均为旧 `error=` 或旧测试仍把 `.error` 当 detail；机械迁移后消除。
- replay 扩展回归暴露旧录制合法使用 `source_frame=0`，测试 RED；将 frame 合同修正为非负整数后 queue + replay `64 passed`。
- 首次精确 Git index 快照回归 `150 passed, 1 failed`，定位到 Frozen reason 未进入 replay JSON、diagnostic text 前缀破坏 parity；补齐序列化并恢复 Frozen raw detail 后，最终 index 快照 `151 passed in 7.15s`。

### 锁序修复

- 原闭环：submit dispatcher 持 `_trace_delivery_condition` 执行 listener；listener 调 `close()` join worker；worker 正等待同一 delivery lock 投递 start trace。
- 新实现使用单一 `_trace_delivery_active` dispatcher：producer 只在 lock 内按 seq 入队；active dispatcher 在 lock 内取下一 seq、lock 外执行用户 callback，再在 lock 内推进 seq。
- callback 期间其他 submit/worker/reentrant submit 只入队后返回，因此 worker 可退出并让 callback 内首次 bounded close 成功；dispatcher 随后严格按连续 seq drain，无重复、无丢失。
- 定向覆盖 callback close 与 reentrant submit；既有覆盖同时验证首 callback 阻塞时严格顺序、listener 异常计数和异常后继续交付。

### 单一 typed failure 真相

- `PlannerResultSnapshot` constructor 现在只接受 `failure_reason/error_text`；独立 `error` 字段已删除，签名传 `error=` 直接 `TypeError`。
- `.error` 是只读派生 property：成功为 `None`，所有失败严格返回 `failure_reason.value`。因此 `TRACK_ENDED` 与 HitterEnv 的 `TRACK_ENDED_RESULT_ERROR = PlannerFailureReason.TRACK_ENDED.value` 一致；unknown 稳定为 `INTERNAL_ERROR`，detail 只在 `error_text`。
- worker、HitterEnv MuJoCo failure、diagnostics integration fake、replay 与相关测试 callsite 均迁到 typed constructor；成功路径旧 `error=None` 已移除。
- diagnostics bundle identity 现在比较 typed reason，不再解析 `.error`；Frozen 同步 reason，并在 replay JSON 中序列化/反序列化。PlannerResult unknown `error_text` 保留 `Type: detail`，Frozen 继续保存原 raw diagnostic detail，维持在线/回放 parity。

### 用户 dirty 保护与 concerns

- fix commit 前 cached name-status 仅 9 个必要文件，无 safety、plan、report 或用户其它 hunk。
- safety 文件仍为未暂存 blob `97224be4ec1d28495b3d796c2b4efdd75431734a`、`+2/-2`、diff SHA256 `7e77f15ca9eeafb4253aea301d39de728e4f175d102a24b219b56699d593a692`。
- 工作树中的 HitterEnv observation/G2、runtime factory planner/config、diagnostics integration G2 用户改动保持未暂存；index 快照测试使用原 G1 基线并全绿。
- 最终 4 条 warning 均为既有 invalid escape sequence；本轮测试/实现未新增 warning。未运行真机、PD 或 `unitree_sdk2/build`。

## Review fix round 2/5

### Finding 与 RED

- re-review finding：failure bundle matcher 只比较 typed reason，相同 reason 但不同 `error_type/error_text` 会被误判为同一 pair。
- 在 `test_hitter_task_pipeline.py` 新增确定性 mutation 测试：基准 INTERNAL_ERROR pair 为 `ValueError: expected` / `(ValueError, expected)`；分别修改 reason、error_type、error_text，三项都必须 mismatch。
- RED：单测 `1 failed`；reason mutation 已被拒绝，但 error_type mutation 当前错误返回 True（detail mutation同一缺口）。

### Canonical detail 比较与 GREEN

- worker producer 合同有两种结构：
  - `PlannerRejected`：`result.error_text == frozen.error_text == exc.detail`，Frozen `error_type == "PlannerRejected"`。
  - unknown exception：`result.error_text == "TypeName: message"`，Frozen 分开保存 `error_type == "TypeName"` 与 `error_text == "message"`。
- matcher 先严格要求 typed reason 相同及 failure 结构完整；PlannerRejected 直接比较 detail，unknown 用 Frozen type 构造唯一精确前缀并剥离后比较 raw message，沿用 Frozen 的 2048 截断。没有 contains、模糊匹配或独立 `.error` source。
- 聚焦 mutation、长异常截断、既有 bundle identity：`3 passed`。
- pipeline + queue（含 trace close/error property）+ replay + diagnostics safety：`112 passed`。
- 最终 Git index 快照合理集合：`152 passed in 7.44s`；4 条 warning 均为既有 invalid escape sequence。

### 提交与 staging

- 独立提交（未 amend）：`927a33e fix: compare HITTER planner failure details`。
- commit 仅含：`deploy/diagnostics/hitter_task_pipeline.py`、`deploy/tests/test_hitter_task_pipeline.py`。
- commit 前 `git diff --cached --check` 通过；cached name-status 仅上述两个文件。
- safety blob、`+2/-2` 和 SHA256 仍与 brief 一致且未暂存；HitterEnv/runtime factory/diagnostics integration 用户 dirty hunk、plan 与本地 report 均未提交。

## Review fix round 3/5

### Coverage finding 与首次结果

- round 2 matcher 实现经确认正确，本轮 finding 是测试仍使用 handcrafted pair，缺少真实 producer 集成覆盖。
- 在 `test_hitter_completed_result_queue.py` 通过真实 `LatestOnlyPlannerWorker` 分别产生：
  - `PlannerRejected(NO_FUTURE_CROSSING, "轨迹: 没有未来交点")`；
  - unknown `ValueError`，message 参数覆盖空串、`a:b:c`、非 ASCII、2055 字符超长字符串。
- helper 对每个 worker 使用有界 wait、drain，并在 `finally` 断言 `close(timeout_s=1.0)`，无遗留线程。
- 新增 characterization/mutation tests 首次运行直接 `5 passed in 0.15s`。这是纯 coverage repair，当前 production 已正确，因此诚实记录为直接 GREEN，没有制造 RED，也没有修改 production。

### 真实 pair 合同与 mutation

- 每个真实 pair 首先独立断言 producer 字段并调用 production `_result_bundle_matches`，原 pair 必须 True。
- typed detail 含冒号与中文，真实字段为 result/frozen detail 原样，证明 matcher 不把 detail 冒号误拆成异常类型。
- unknown 独立断言：result 保存 `ValueError: message`；Frozen 分开保存 `error_type="ValueError"` 与 raw message；超长 Frozen detail 精确截断至 2048，而 result 保留完整文本。
- 每个 typed/unknown 真实 pair 分别 mutation Frozen 的 reason、error_type、error_text，任一变化均必须 False。测试 expectation 未调用或复制 production split helper。

### 验证、提交与 dirty 保护

- queue + pipeline + replay + diagnostics safety + Task 5 failure + identity：工作树 `139 passed in 7.21s`；Git index 快照 `139 passed in 7.26s`。
- warning 仅为既有 invalid escape sequence；本轮未新增 warning。py_compile 与 `git diff --check` 通过。
- 独立 test-only commit（未 amend）：`49efb76 test: cover real HITTER planner failure pairs`。
- commit 仅含 `deploy/tests/test_hitter_completed_result_queue.py`，无 production、safety、用户 dirty、plan 或 report。
- safety blob、`+2/-2`、diff SHA256 仍与 brief 一致且未暂存。
