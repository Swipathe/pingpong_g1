# Task 1 实现报告

## 状态

完成。Task 1 已按 TDD 实现、验证并原子提交；未修改或读取
`recordings/` 源数据，未实现 Task 2/3/4 的输入对齐、CSV 写出、CLI
或真实数据重建。

## Commit

- `45fdc0ba90df4abf2ebf0b94c09e12fc63b04357`
- `feat: reconstruct completed HITTER planner calls`

## 修改文件

- `deploy/diagnostics/export_hitter_task_csv.py`
- `deploy/tests/test_hitter_task_csv_export.py`

报告文件按调度要求在提交后生成，不包含在上述 Task 1 代码提交中。

## 测试与 TDD 证据

### RED

1. 首个真实 JSONL 测试：
   `PlannerEventScanTests.test_scan_keeps_only_calls_that_started_and_completed`
   以退出码 1 失败，错误为
   `ModuleNotFoundError: No module named
   'diagnostics.export_hitter_task_csv'`。
2. 最小扫描器通过首测后，新增未分类 submit 测试：
   `test_scan_rejects_unclassified_submitted_key` 以退出码 1 失败，
   错误为 `AssertionError: ValueError not raised`。
3. 新增重复事件冲突测试后，submit/start 分支已拒绝冲突，
   complete 子测试先以退出码 1 失败，证明仅比较 result 会漏掉
   complete payload 的其他冲突字段。
4. 新增完成与 pending-replaced 重叠测试后，先以退出码 1 失败，
   证明仅比较集合 union 会漏掉重复归类。

### GREEN

- `python -m py_compile` 检查新增模块和测试：退出码 0。
- `PlannerEventScanTests`：4 个测试全部通过。
- 相关回归：
  `tests.test_hitter_task_replay`、
  `tests.test_hitter_task_recording` 与 Task 1 测试合计 66 个测试，
  全部通过（`Ran 66 tests ... OK`）。
- `git diff --cached --check`：退出码 0、无输出。

## 关键语义

- `events.jsonl` 使用逐行 `json.loads()` 扫描，不一次性载入完整事件文件。
- 仅处理请求的 attempt，并以
  `(attempt_id, schema_version, track_epoch, generation)` 建立导出 key。
- `planner_submit`、`planner_trace/start` 和
  `planner_trace/complete` 必须同时存在，才生成
  `CompletedPlannerCall`；complete result 的 snapshot key 必须与事件 key
  一致。
- `pending_replaced` 的 `replaced_snapshot_key` 记录为丢弃 key；
  cross-attempt 替换使用 `replaced_attempt_id`。
- `pending_replaced_key` 和 `latest_replaced_key` 均挂在执行替换的调用上。
- 冲突的重复 submit/start/complete 直接抛出 `ValueError`；相同内容的
  重复记录允许幂等处理。
- 完成调用按 `(completed_monotonic_s, key)` 稳定排序。
- 完成 key 与 pending-replaced key 必须互斥，且二者并集必须与全部
  submitted key 完全相等；任何 missing 或 unexpected key 都会失败。

## 遗留问题与风险

- Task 1 只负责事件重建；planner input 严格连接、三张核心 CSV
  行对齐、事务式目录替换和 CLI 留给 Task 2/3。
- 按任务约束未触碰真实 `recordings/`，因此 837/1248/2085 的真实
  session 验收仍由 Task 4 完成。
- 当前扫描器保留完整 planner result mapping；后续 CSV 展平必须继续
  使用同一 `completed_calls` 顺序，避免各表独立排序。
