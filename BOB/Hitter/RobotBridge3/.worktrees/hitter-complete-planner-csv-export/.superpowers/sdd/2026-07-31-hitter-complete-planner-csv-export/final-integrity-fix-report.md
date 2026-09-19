# 最终完整性修复报告

日期：2026-07-31  
基线：`9544c93`  
范围：

- `deploy/diagnostics/export_hitter_task_csv.py`
- `deploy/tests/test_hitter_task_csv_export.py`
- `docs/hitter_complete_planner_csv_export.md`

## 根因

1. `stage_timeline.csv` 和 segment 区间直接使用 attempt detail。在线
   `AttemptTracker` 的 timeline 是 `deque(maxlen=4096)`，长 attempt 会让
   stage、raw LCM 和 policy tick 静默漏掉早期数据。
2. 目录已经 rename + parent fsync 提交后，backup cleanup 的
   `warnings.warn` 仍可能被 `warnings.simplefilter("error")` 转成异常，
   造成“输出已成功、调用却失败”的假失败。
3. `_planner_key` 使用 `int()`，会接受并转换 bool、float、字符串、负数
   和不支持的 schema version；event/replacement/input 的 attempt id
   也可能在进入 key 校验前被转换。

## RED

先加入并运行 6 个定向行为测试。旧实现结果：

- 22 failures
- 1 error
- 长窗口只导出 4096 行而不是 4098 行
- detail timeline/segment mismatch 未拒绝
- 非严格 key 全被接受
- warning-as-error 从 commit 后清理路径抛出

## 修复

### 完整 transition / segment

- 流式读取完整 `events.jsonl` 中所选 attempt 的
  `attempt_transition`。
- 严格校验并规范化为 detail stage row。
- 按 event 顺序形成 full timeline。
- 按在线 monitor 的相同算法从 full timeline 重建 segment 的
  first/last/role。
- `stage_timeline.csv` 写 full timeline。
- `raw_lcm.csv` 和 `policy_ticks.csv` 使用 full segment intervals。
- detail timeline 必须精确等于
  `full[-min(len(full), 4096):]`。
- detail segments 必须能从 detail suffix 精确重建。
- manifest 增加：
  - `full_attempt_transitions_from_events`
  - `attempt_detail_timeline_suffix_verified`
  - `segments_rebuilt_from_full_transitions`

真实 session 中同一 segment 的 transition monotonic timestamp 会因事件
异步入队在局部短暂回退。因此重建严格遵循在线 monitor 的“事件序第一条/
最后一条”语义，不额外要求每相邻 transition 单调；最终 interval 仍要求
`first <= last`。

### commit 后 warning

- `_warn_after_commit` 保留正常 `RuntimeWarning`。
- warning filter 或 hook 抛出任意异常时，吞掉异常并尝试向 stderr
  输出 fallback；stderr 自身失败也不会越过 commit 边界抛出。
- 新 output 保持已提交，cleanup 失败时 backup 保留。

### planner key

- 所有分量必须是严格非 bool integer。
- `attempt_id >= 1`
- `schema_version == 1`
- `track_epoch >= 0`
- `generation >= 0`
- event、replacement、input、result 相关 attempt/key 路径不再使用
  `int()` 转换。

## GREEN 与真实数据

定向测试：

```text
Ran 6 tests
OK
```

完整三模块回归：

```text
Ran 102 tests
OK
```

Python 3.8 `py_compile` 与 `git diff --check` 均通过。

真实只读 session：

```text
submitted / pending / completed = 2625 / 540 / 2085
attempt 1 / attempt 2 completed = 837 / 1248
raw_lcm.csv = 33894
planner_inputs.csv = 2085
planner_calls.csv = 2085
planner_results.csv = 2085
stage_timeline.csv = 2102
policy_ticks.csv = 1578
generation (2, 1, 1, 11192) present = true
```

三项新增 manifest checks 均为 `true`，真实 attempt detail suffix 和
segments 交叉校验通过。
