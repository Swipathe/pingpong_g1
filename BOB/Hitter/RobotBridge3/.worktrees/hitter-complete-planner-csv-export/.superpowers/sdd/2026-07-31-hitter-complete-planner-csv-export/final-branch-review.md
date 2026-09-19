# HITTER Complete Planner CSV Export 最终分支审查

## Verdict

**CHANGES REQUESTED — 当前不应集成。**

最终分支对目标真实 session 生成的正式产物是正确的，95 项回归、
Python 3.8 编译、diff 检查、三张核心表同序、manifest/source hash 和
正式输出独立重读均通过；此前 Task 1 的跨 attempt replacement 修复以及
Task 3 的 raw sequence、lifecycle identity、output overlap、symlink、
目录事务和最终 source hash 修复也未发现跨 Task 回归。

但完整事件分类仍有一个 Important fail-open：selected attempt 中不属于
合法 `submit -> start -> complete` 链的孤立 start/complete 会被静默忽略，
而不是按设计拒绝不完整磁盘日志。因此通用 CLI 尚未完全满足严格完整性
契约，即使当前 Attempt 1/2 的实际源数据没有触发该缺陷。

## Findings

### Critical

无。

### Important

#### 1. Planner 扫描器没有拒绝 orphan start/complete，started pending key 仍可被标记为完整 pending

- **位置：**
  - `deploy/diagnostics/export_hitter_task_csv.py:733-770`
  - `deploy/diagnostics/export_hitter_task_csv.py:777-794`
  - `deploy/diagnostics/export_hitter_task_csv.py:2463-2465`
  - 测试缺口：`deploy/tests/test_hitter_task_csv_export.py:603-855`
- **影响：**
  - `completed_calls` 只由
    `submissions.keys() & starts.keys() & completions.keys()` 构造。
    随后的闭合检查只比较
    `submitted == completed | pending_replaced`，没有检查：
    - `starts <= submissions`
    - `completions <= submissions`
    - `starts == completions`
    - `pending_replaced` 与 `starts/completions` 互斥
  - 因此 selected attempt 中只有 complete、没有 submit/start 的 key 会被
    完全忽略；更严重的是，一个已被归类为 `pending_replaced` 的 key 即使
    又出现 start、但缺 complete，仍会通过导出。命令随后可写出全部
    completeness checks 为 `true` 的 manifest，造成不完整或内部冲突的
    `events.jsonl` 被声明为完整。
  - 这违反设计中“每个 complete key 都存在 submit、start、result 和
    planner input”“只有没有 start/complete 的 pending-replaced 才排除”
    以及任一不完整日志必须 fail closed 的要求。
- **独立复现证据：**

  ```text
  ORPHAN_COMPLETE_ACCEPTED 0 0
  STARTED_PENDING_ACCEPTED [10] [11]
  ```

  第一例仅写入 Attempt 1 / generation 99 的 complete，扫描器返回空结果而
  不报错。第二例 generation 10 已被 generation 11 pending-replace，但
  generation 10 仍有 start 且没有 complete；扫描器仍把 10 归为 pending，
  把 11 归为 completed，并通过计数闭合。
- **建议：**
  - 在构造结果前显式拒绝所有 selected-attempt 事件关系异常：
    `starts - submissions`、`completions - submissions`、
    `starts - completions`、`completions - starts` 均必须为空。
  - 显式要求 `pending_replaced_keys` 与 `starts`、`completions` 均互斥；
    这样“pending-replaced”才确实表示从未开始执行。
  - 增加至少四个真实 JSONL 回归测试：orphan start、orphan complete、
    start-without-complete、pending-replaced-plus-start。测试还应断言
    `export_session_csv(..., overwrite=True)` 失败时旧 output 保持不变，
    且不生成声称完整的 manifest。

### Minor

无。

## 整分支核对

- **要求与非目标：** diff 仅新增
  `deploy/diagnostics/export_hitter_task_csv.py`、
  `deploy/tests/test_hitter_task_csv_export.py` 和
  `docs/hitter_complete_planner_csv_export.md`。未修改 estimator、planner、
  policy、控制参数、latest-only worker 或 replay capture 的 8192 上限。
- **真实 schema / 数据源：** exporter 从 `session.json`、
  `events.jsonl`、`ball_samples.csv` 和 selected attempt details 读取；
  没有导入或读取 `ReplayCaptureStore`、`ReplayInputBundle` 或 replay
  bundle。Planner input、result、raw、stage、tick 和 observation 固定
  header 与当前磁盘 schema 相符。
- **三张核心表：** 同一个 `aligned` tuple 生成 inputs/calls/results，
  落盘后以 typed 四字段 key 重新读取，检查 header、行数、唯一性和同序。
- **辅助表：** raw 在筛选前验证全文件 `input_seq` 连续性，区间内不再按
  subject 白名单静默丢行；stage 保留 detail 顺序；policy tick 来自完整
  event log 并验证 selected-attempt interval identity；无 observation 的
  attempt 仍保留空行。
- **CLI / manifest / 原子安全：** 四参数 CLI、固定九项产物、源
  SHA-256/stat、row counts、core-key hash 和 20 项 checks 均已实现。输出
  写入同级 temp，旧目录经 backup 事务替换；install/fsync/restore/cleanup
  状态与返回语义一致，静态 source overlap 与已有 symlink component
  会拒绝。
- **Python 3.8：** 使用 rb 环境 Python 3.8.20 完成 fresh `py_compile`
  和全部测试；未发现高版本专属 API。
- **文档：** 中文运行文档说明完整数据源、actual completed/pending
  语义、输出 schema、CLI、事务提交点及已知限制。

## Fresh verification

在 HEAD `931ce6da189a0c74f19ebffc257cfe9bfe21e217` 上执行：

```text
python -m py_compile exporter/tests             PASS
unittest export + replay + recording            Ran 95 tests; OK
git diff --check base..head                      PASS
git diff --check                                 PASS
worktree status                                  clean
```

正式输出只读独立重读：

```text
summary.csv                2
raw_lcm.csv            33894
planner_inputs.csv      2085
planner_calls.csv       2085
planner_results.csv     2085
stage_timeline.csv      2102
policy_ticks.csv        1578
task_observations.csv      2
```

- 三张核心表 2085 个 typed key 逐行同序且唯一。
- Attempt 1 / 2 分别为 837 / 1248，generation 11192 存在。
- 正式源事件关系额外验证为：
  `submitted=2625, starts=2085, completed=2085, pending=540`；
  `completed <= starts <= submitted`、pending 与 start/complete 互斥、
  `submitted == completed | pending` 均成立。
- manifest 的 20 项 checks 全为 true，row counts、core key hash 和五个
  source SHA-256 均与磁盘独立重算一致。
- 正式输出父目录无 temp、backup 或 failed-new 残留。

因此本 finding **不否定当前正式 Attempt 1/2 数据的正确性**；它阻塞的是
分支承诺的可重复、通用、严格 fail-closed 导出接口。

## Residual limitations

以下限制已在实现/文档中披露，本次不作为新增 finding：

1. output component 检查和 rename 仍是 path-based；有权限的并发进程可在
   最后检查与 rename 之间制造 TOCTOU。完全关闭需 directory fd /
   `renameat`。
2. source 仍通过路径多次读取；最终 stat/hash 缩小但不能消除
   check-to-rename 窗口，也不能识别修改后恢复为相同字节的并发写者。
3. `input_seq` 相邻连续性不能识别文件尾部整体截断；需要 recorder 持久化
   raw row count 或末端 sequence watermark。
4. 非空目录覆盖需要两次 rename，进程在中间崩溃可能留下可恢复的 backup
   或 failed-new；它是可恢复目录事务，不是单次原子交换。

