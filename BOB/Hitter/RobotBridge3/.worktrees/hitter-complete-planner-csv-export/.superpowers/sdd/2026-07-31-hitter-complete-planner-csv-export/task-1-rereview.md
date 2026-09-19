# Task 1 修复复核

## Verdict

**APPROVED**

原 P1/blocking finding 已关闭。commit
`661659c68f480315d75f4395f5b1998ebb88e2b5` 未引入新的 blocking issue。

## 原 Finding 关闭依据

`deploy/diagnostics/export_hitter_task_csv.py:97-113` 现在会在 primary
attempt 过滤之前识别 `planner_trace/pending_replaced`。当
`replaced_attempt_id` 属于所选 attempts 时，扫描器使用
`replaced_snapshot_key` 和被替换 attempt id 构造完整
`PlannerExportKey`，因此 replacer 未被选择时，被替换 attempt 的 submit
仍能进入 `pending_replaced_keys` 并通过分类闭合。

随后 `deploy/diagnostics/export_hitter_task_csv.py:114-115` 继续过滤未选择
的 primary attempt；所以未选择 B 的 submit/start/complete 不会泄漏到
A-only 结果。B 被选择时，原有 pending replacement 分支仍会把 A 的 key
挂到 B 完成调用的 `pending_replaced_key`，没有破坏 replacement
metadata。

新增测试保留了生产管线的 primary `planner_trace` 与
`planner_trace_replaced` companion 字段形状，并分别锁定三种选择语义：

- A-only：A/10 在 submitted 与 pending-replaced 中，completed 为空；
- B-only：仅 B/11 完成，pending 分类为空，调用 metadata 指向 A/10；
- A+B：两个 submit、一个 pending、一个 completed，集合严格闭合。

## 验证结果

- A-only、B-only、A+B 三项针对性测试：3/3 通过。
- `PlannerEventScanTests`：7/7 通过。
- `tests.test_hitter_task_replay`、
  `tests.test_hitter_task_recording`、
  `tests.test_hitter_task_csv_export`：69/69 通过。
- `git diff --check 45fdc0b..661659c`：退出码 0、无输出。
- 修复 commit 仅改动 Task 1 的扫描器与测试文件；未修改源录制数据。

## Residual Risks

前次审查记录的非阻塞测试空白仍然存在，包括
`latest_result_replaced` metadata、完成时间 tie-break、非有限时间以及
极长 session 峰值内存；本次窄范围 cross-attempt 修复没有扩大这些风险，
也不影响原 finding 的关闭。
