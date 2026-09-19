# Task 1 审查修复报告

## 状态

已修复 Task 1 review 的唯一 blocking finding：当被替换 attempt A 被单独
选择、而 `pending_replaced` primary 事件属于未选择的 replacer attempt B
时，A 的旧 key 现在仍会进入 `pending_replaced_keys` 并通过严格分类闭合。

## Commit

- `661659c68f480315d75f4395f5b1998ebb88e2b5`
- `fix: retain cross-attempt planner replacements`

## 修改文件

- `deploy/diagnostics/export_hitter_task_csv.py`
- `deploy/tests/test_hitter_task_csv_export.py`

本报告按调度要求在 fix commit 后生成，不包含在上述代码提交中。

## TDD 证据

### RED

先使用真实 JSONL 临时文件加入生产字段形状：

- attempt A/generation 10 submit；
- attempt B/generation 11 submit；
- B 的 `planner_trace/pending_replaced` primary 指向 A/generation 10，
  且包含 `replaced_attempt_id=A`；
- B 随后 start/complete；
- A 收到对应的 `planner_trace_replaced` companion。

只调用 `scan_planner_events(events_path, [A])` 时，目标测试以退出码 1
失败，原始扫描器抛出：

```text
submitted keys are neither completed nor pending-replaced:
missing=[PlannerExportKey(attempt_id=1, schema_version=1,
track_epoch=1, generation=10)], unexpected=[]
```

测试将该异常转为明确的 `AssertionError`，确认失败原因是 selected A 的
pending key 被过滤掉，而不是 fixture 或解析错误。

### GREEN

最小修复后：

- A-only：submitted 仅含 A/10，pending-replaced 仅含 A/10，
  completed 为空；
- B-only：submitted/completed 仅含 B/11，pending 分类为空，
  B 调用的 `pending_replaced_key` 仍正确指向 A/10；
- A+B：submitted 为 A/10 与 B/11，pending 为 A/10，
  completed 为 B/11，严格闭合继续成立。

## 实现语义

- 在常规 primary attempt 过滤之前，仅识别
  `planner_trace/pending_replaced`。
- 当 payload 的 `replaced_attempt_id` 属于 selected attempts 时，
  使用 `replaced_snapshot_key` 构造被替换 attempt 的完整
  `PlannerExportKey` 并记录。
- 随后仍按 primary `event.attempt_id` 执行原过滤，因此未选择 attempt
  的 submit/start/complete 不会进入扫描结果。
- 生产 `planner_trace_replaced` companion 被保留在测试 fixture 中用于
  锁定真实 schema；扫描器继续以 primary 事件为事实源，避免重复维护
  两套 replacement 解析。
- `latest_result_replaced` 不负责 pending 分类。其 metadata 仍在
  replacer primary attempt 被选择时按原逻辑记录，无需改变。

## 验证

- 新增后的 `PlannerEventScanTests`：7/7 通过。
- `python -m py_compile`：退出码 0。
- Task 1、`tests.test_hitter_task_replay` 和
  `tests.test_hitter_task_recording`：共 69 个测试全部通过
  (`Ran 69 tests ... OK`)。
- `git diff --cached --check`：退出码 0、无输出。
- fix commit 仅包含上述两个 Task 1 文件；未触碰 `recordings/`。

## 遗留问题与风险

- 本修复严格限于 review 的 cross-attempt pending subset 语义，没有扩展
  Task 2 的 planner input 对齐或 CSV 构建。
- `latest_result_replaced`、时间 tie-break 和非有限时间的附加覆盖仍属于
  review 中列出的非阻塞 residual risks，可在后续完整性测试中补充。
