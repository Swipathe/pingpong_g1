# Task 3 数据 / 规格复审

## Verdict

**APPROVED**

Commit `931ce6d` 已关闭上一轮三个 data findings；本轮窄范围复审未发现剩余阻塞问题。

## Findings

无。

## 修复核对

- Raw source 完整性：
  - `deploy/diagnostics/export_hitter_task_csv.py:1359-1391` 在任何 interval 筛选之前读取全文件 `input_seq`。
  - 仅接受非负 ASCII 十进制整数；首值可任意，后续必须严格逐行 `+1`。
  - 缺号、重复、倒序、负值都会 fail closed，且不会替换已有输出。
- 未知 subject：
  - `deploy/diagnostics/export_hitter_task_csv.py:1399-1406` 只按闭区间筛选，不再使用固定 subject 白名单。
  - 聚焦测试确认 `g2pelvis` 行被原样保留，fixture 的区间内 raw 行数仍为 6。
- Lifecycle tick 身份：
  - `deploy/diagnostics/export_hitter_task_csv.py:1493-1495` 建立所选 attempt 集合。
  - `deploy/diagnostics/export_hitter_task_csv.py:1588-1597` 对“位于所选区间、但声明为另一个所选 attempt”的 tick 报错；未选择 attempt 的 tick 仍按约定忽略。
- 最终源一致性：
  - manifest round-trip 与临时目录 fsync 完成后，`deploy/diagnostics/export_hitter_task_csv.py:2474-2483` 在目录替换前再次 stat/hash 所有源文件。
- 文档已同步说明全 raw 行保留、`input_seq` 规则、tick 身份冲突和已知尾部截断边界。

## 新鲜验证证据

完整三模块回归：

```text
Ran 95 tests in 2.408s
OK
```

聚焦回归：

```text
test_raw_input_sequence_anomalies_fail_before_replacing_output ... ok
test_raw_rows_inside_intervals_are_not_subject_filtered ... ok
test_selected_lifecycle_attempt_conflict_is_rejected ... ok
Ran 3 tests in 0.089s
OK
```

真实 session 使用 `TemporaryDirectory` 重新导出并独立关文件重读：

```text
summary.csv               2
raw_lcm.csv           33894
planner_inputs.csv     2085
planner_calls.csv      2085
planner_results.csv    2085
stage_timeline.csv     2102
policy_ticks.csv       1578
task_observations.csv     2

submitted / pending / completed = 2625 / 540 / 2085
Attempt 1 / Attempt 2 completed = 837 / 1248
raw source input_seq = 0..308509, rows=308510, contiguous=true
三核心表 key 同序且唯一 = true
Attempt 2 generation 11192 = present
manifest round-trip / row counts / all checks / source SHA-256 = true
```

`git diff --check a406253..931ce6d` 退出 0；修复提交仅包含 exporter、对应测试和运行文档三个计划内文件。

## 非阻塞已知边界

相邻 `input_seq` 连续性无法单独识别文件尾部被整体截断。修复报告和运行文档已明确记录：彻底关闭该风险需要 recorder 在 session metadata 中新增 raw row count 或末端 `input_seq` watermark；当前真实 session 的完整连续序列已独立验证。
