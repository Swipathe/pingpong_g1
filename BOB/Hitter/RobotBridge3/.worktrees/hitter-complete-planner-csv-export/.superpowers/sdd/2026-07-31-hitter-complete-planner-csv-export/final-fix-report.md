# HITTER Planner 事件集合最终修复报告

## 结论

已修复最终分支审查中的唯一 Important finding：selected attempts 内不完整
或互相冲突的 planner 事件集合现在会在构造调用记录前 fail closed，不再
通过三集合交集静默丢弃 orphan start/complete，也不再允许 started 或
completed key 被归入 pending-replaced。

## Commit

- `9544c93fe1f86206a3deae01ca545471b1bdbf0b`
- `fix: reject incomplete HITTER planner event sets`

提交只包含：

- `deploy/diagnostics/export_hitter_task_csv.py`
- `deploy/tests/test_hitter_task_csv_export.py`

未修改 Task 3 输出、CLI、文档或 recordings。

## 根因

旧实现先取 `submissions ∩ starts ∩ completions` 构造
`completed_calls`，再只检查 `completed ∪ pending == submitted`。
任何未进入三集合交集的 start/complete key 会在严格闭合前消失；同时，
已有 start 或 complete 的 key 仍可能以 pending-replaced 身份完成分类。

## TDD 证据

### RED

先新增一个基于临时真实 JSONL 文件的参数化测试，覆盖：

1. orphan start + complete，但没有 `planner_submit`；
2. submitted + start + pending-replaced，但没有 complete；
3. submitted + pending-replaced + complete，但没有 start。

定向命令：

```bash
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python \
  -m unittest \
  tests.test_hitter_task_csv_export.PlannerEventScanTests.test_scan_rejects_incomplete_selected_attempt_event_sets \
  -v
```

生产代码修改前退出码为 `1`，三个 subTest 均准确失败：

```text
AssertionError: ValueError not raised
FAILED (failures=3)
```

### GREEN

最小生产修改后，同一命令退出码为 `0`：

```text
Ran 1 test in 0.002s
OK
```

全部 `PlannerEventScanTests` 随后为 `8/8` 通过，其中既有 A-only、
B-only、A+B 三个跨 attempt replacement 用例均保持原语义。

## 修复后的集合不变量

扫描 selected attempts 完成、构造 `CompletedPlannerCall` 前，现在按顺序
验证：

1. start keys 与 complete keys 完全相等；
2. start keys 和 complete keys 都是 submitted keys 的子集；
3. pending-replaced keys 分别与 start keys、complete keys 互斥；
4. completed keys 直接取已经验证相等的 start/complete 集合；
5. 最后仍要求 `completed ∪ pending-replaced == submitted`。

异常文本会分别报告缺失 complete、缺失 start、无 submit 的 start/complete
以及 pending overlap 的具体 typed keys。

## 最终验证

Python 3.8 编译：

```bash
/home/loco1/miniconda3/envs/rb/bin/python -m py_compile \
  diagnostics/export_hitter_task_csv.py \
  tests/test_hitter_task_csv_export.py
```

退出码 `0`。

exporter、replay、recording 完整回归：

```bash
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python \
  -m unittest \
    tests.test_hitter_task_csv_export \
    tests.test_hitter_task_replay \
    tests.test_hitter_task_recording \
    -v
```

结果：

```text
Ran 96 tests in 2.400s
OK
```

对正式源 `events.jsonl` 执行只读 `scan_planner_events(..., [1, 2])`：

```text
submitted=2625
pending_replaced=540
completed=2085
Attempt 1 completed=837
Attempt 2 completed=1248
```

计数与原 oracle 完全一致。

Git 检查：

```text
git diff --check                    PASS
git diff --cached --check           PASS
git diff --check HEAD^ HEAD         PASS
commit file scope                   exporter + test only
post-commit tracked worktree        clean
```
