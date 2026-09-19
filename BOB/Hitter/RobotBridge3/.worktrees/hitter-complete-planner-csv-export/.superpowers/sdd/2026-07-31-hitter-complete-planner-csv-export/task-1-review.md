# Task 1 独立代码审查

## Verdict

**CHANGES REQUESTED**

当前实现能在目标真实 session 上重建出正确的 `837 / 1248 / 2085`
完成调用计数，核心事件字段、完成时间来源、重复冲突检查和集合闭合
也基本符合 Task 1 约束；但任意 `attempt_ids` 子集这一公开接口在
cross-attempt pending replacement 上存在可复现的错误，需修复后再批准。

## Findings

### [P1 / Blocking] 只选择被另一 attempt 替换的 attempt 时，pending key 会被误判为未分类

- **位置：**
  `deploy/diagnostics/export_hitter_task_csv.py:93-98`，
  `deploy/diagnostics/export_hitter_task_csv.py:131-148`
- **影响：**
  `scan_planner_events(events_path, attempt_ids)` 接受任意 attempt 子集，
  后续 CLI 也允许用户指定 `--attempts`。当 attempt A 的 pending
  snapshot 被未选择的 attempt B 的新 snapshot 替换时，单独导出 A
  会以
  `submitted keys are neither completed nor pending-replaced`
  失败。也就是说，合法且完整的磁盘日志只有在碰巧把 replacer attempt
  一并选中时才能通过，违反“只处理指定 attempt”和严格分类闭合接口的
  组合语义，也与实现报告中“cross-attempt 替换使用
  `replaced_attempt_id`”的声明不符。
- **理由与复现：**
  生产管线把 primary `planner_trace/pending_replaced` 事件绑定到
  replacer B，并在 payload 中记录 `replaced_attempt_id=A`；同时向 A
  另发 `planner_trace_replaced` companion
  （`deploy/diagnostics/hitter_task_pipeline.py:1422-1500`，
  现有生产测试见
  `deploy/tests/test_hitter_task_pipeline.py:1892-1939`）。
  扫描器却在读取 payload 之前先按 primary 事件的
  `event.attempt_id` 过滤，因此选择 `[A]` 时既跳过 B 的 primary
  replacement，也忽略 A 的 `planner_trace_replaced`。

  使用真实生产字段形状构造：

  1. A/generation 10 有 `planner_submit`；
  2. B/generation 11 有 `planner_submit`；
  3. B 的 `planner_trace/pending_replaced` 指向 A/generation 10，
     `replaced_attempt_id=A`；
  4. B/generation 11 随后 start/complete；
  5. A 收到对应 `planner_trace_replaced`。

  当前结果为：

  ```text
  [A]    ValueError ... missing=[A/generation 10]
  [B]    OK: submitted=1, pending_replaced=0, completed=1
  [A, B] OK: submitted=2, pending_replaced=1, completed=1
  ```

  当前目标真实 session 的 `[1, 2]` 联合扫描不受影响，得到
  `submitted=2625, pending_replaced=540, completed=2085`；这不能覆盖
  单 attempt 或其他合法子集导出。
- **建议：**
  在丢弃未选择 primary attempt 的事件前，先识别
  `planner_trace/pending_replaced`，并在
  `replaced_attempt_id` 属于所选集合时将
  `replaced_snapshot_key` 计入该被替换 attempt；或者等价地解析
  `planner_trace_replaced` companion。仍可保持单遍流式扫描。
  同时增加至少以下真实 JSONL 测试：

  - 只选择被替换的 A，A 的 key 被归入 `pending_replaced_keys`；
  - 只选择 replacer B，B 的完成调用保留且 replacement metadata 正确；
  - 同时选择 A/B，集合继续严格闭合。

## 已核对且符合要求的部分

- `events.jsonl` 按行解析，没有一次性读入 149 MB 的真实事件文件。
- key 正确包含
  `(attempt_id, schema_version, track_epoch, generation)`。
- pending 分类使用的是 `replaced_snapshot_key`，不是 replacer 自身
  `snapshot_key`。
- 完成调用要求同 key 的 submit、start、complete/result 同时存在。
- complete payload 的 key 与 result key 不一致时会失败。
- submit/start/complete 的内容冲突重复会失败；相同内容重复可幂等处理。
- 完成时间取 result 的 `completed_monotonic_s`，而不是稍晚到达的
  trace event 时间；排序键为 `(completed_monotonic_s, key)`。
- completed 与 pending-replaced 互斥，并与 submitted 做严格集合闭合。
- 未修改实时 worker、replay capture 上限或其他 Task 2/3 功能。

## 验证证据

- `PlannerEventScanTests`：4/4 通过。
- 相关回归：
  `tests.test_hitter_task_replay`、
  `tests.test_hitter_task_recording`、
  `tests.test_hitter_task_csv_export`：66/66 通过。
- 真实 session 扫描：
  - Attempt 1：`1050 / 213 / 837`
  - Attempt 2：`1575 / 327 / 1248`
  - Attempt 1+2：`2625 / 540 / 2085`
- 上述 cross-attempt 单侧选择复现：稳定失败。

## Residual Risks

- 当前测试没有直接锁定 complete event 与 result 的 key 冲突分支，
  该生产检查虽已实现，但后续回归仍可能被无意删除。
- 当前测试没有覆盖 `latest_result_replaced` metadata、相同完成时间的
  tie-break 排序，以及非有限或逆序 monotonic 时间；真实目标 session
  未暴露这些问题，但建议在后续 Task 2/3 完整性测试中补齐。
- 扫描是文件级流式的，但仍为所有所选 planner 事件保留 payload；
  当前 149 MB session 表现正常，极长 session 的峰值内存尚无专门测试。
