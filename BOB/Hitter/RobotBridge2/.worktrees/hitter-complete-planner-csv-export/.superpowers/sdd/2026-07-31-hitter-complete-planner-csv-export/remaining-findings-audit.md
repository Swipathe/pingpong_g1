# 剩余完整性问题只读复核

## 复核范围

- Worktree：`hitter-complete-planner-csv-export`
- 被复核提交：`9544c93fe1f86206a3deae01ca545471b1bdbf0b`
- 复核方式：只读检查生产代码，并在 `/tmp` 中构造最小 session；
  未修改导出器、测试或 recordings。
- 正式源 session：
  `20260730_194528_402494-p740921-f85e25f0`
- 正式 Attempt 1/2 输出：
  `analysis_attempts_1_2_20260730_194528`

## 结论

三项旧审计问题都可以稳定复现：

1. **高严重度**：`stage_timeline.csv` 直接信任有界 attempt
   timeline；`raw_lcm.csv` 和 `policy_ticks.csv` 虽分别扫描完整
   `ball_samples.csv` / `events.jsonl`，但筛选区间仍来自同一个有界
   timeline 派生出的 detail segments。完整事件和 detail 不一致时，
   三张表会静默少导，manifest 仍全部为 `true`。
2. **中严重度**：事务已经提交后，backup 清理失败调用
   `warnings.warn`。在 `PYTHONWARNINGS=error` 或等价过滤器下，
   warning 会变成异常，因此 CLI 会出现“新 output 已提交，但退出码为
   1”的矛盾结果。
3. **高严重度**：planner key 通过裸 `int()` 解析，会接受并转换
   `bool`、浮点数、数字字符串，接受负 epoch/generation，也接受不支持
   的 snapshot schema。浮点截断还会改变 key 身份。

**当前正式 Attempt 1/2 没有受到这三项缺陷影响。** 当前数据中
transition 未触及 4096 上限、events/detail 完全一致，planner key
全部是合法整数，正式覆盖也没有遗留 backup/temp。但这只能证明当前
产物正确，不能消除未来长 attempt 或畸形源数据的完整性风险。

## A. 有界 timeline 可导致三张辅助表静默少导

### 根因证据

1. `AttemptTracker` 默认
   `max_timeline_per_attempt=4096`，每个 attempt 使用
   `deque(maxlen=...)`：
   `deploy/diagnostics/hitter_task_attempts.py:73-92,349-363`。
   超过 4096 时，旧 transition 会由 deque 自动淘汰，没有截断标志。
2. monitor 会把每个 transition 单独发布为
   `attempt_transition`，因此完整 `events.jsonl` 仍可保留它：
   `deploy/diagnostics/hitter_task_monitor.py:2669-2678`。
3. attempt 关闭时，`segments` 和 `stage_timeline` 都只由上述有界
   timeline 生成：
   `deploy/diagnostics/hitter_task_monitor.py:2352-2392`。
4. 离线导出器：

   - `_write_stage_timeline_csv` 直接遍历
     `attempt_details[*].stage_timeline`：
     `deploy/diagnostics/export_hitter_task_csv.py:1806-1910`；
   - `_merged_segment_intervals` 只读取
     `attempt_details[*].segments`：
     `deploy/diagnostics/export_hitter_task_csv.py:1123-1214`；
   - `raw_lcm.csv` 扫描完整 raw 文件，但用上述 detail intervals
     筛选：
     `deploy/diagnostics/export_hitter_task_csv.py:1356-1468`；
   - `policy_ticks.csv` 扫描完整事件文件，但同样用上述 detail
     intervals 筛选：
     `deploy/diagnostics/export_hitter_task_csv.py:1516-1630`。

当前没有把 `events.jsonl` 的 `attempt_transition` 与 detail timeline
逐条核对，也没有 transition 数量/watermark 或 timeline-truncated
标志。

### 最小复现

在完整 fixture 中保留 3 条 `attempt_transition`，然后模拟有界 deque
已淘汰 Attempt 1 的早期 transition：detail 只保留 2 条，并把 Attempt 1
segment 起点同步后移。两次导出结果如下：

```text
full_events_attempt_transitions= 3
truncated_detail_transitions= 2
baseline_counts= {
  raw_lcm.csv: 6,
  stage_timeline.csv: 3,
  policy_ticks.csv: 2,
  planner_calls.csv: 2
}
truncated_counts= {
  raw_lcm.csv: 3,
  stage_timeline.csv: 2,
  policy_ticks.csv: 1,
  planner_calls.csv: 2
}
truncated_export_returned_success=True
truncated_manifest_checks_all_true=True
```

这证明缺失并非只影响网页或 attempt detail：完整 raw/event 文件仍在，
但 detail-derived interval 会让 `raw_lcm` 和 `policy_ticks` 一并静默
少导。核心 planner 表在同一复现中仍为 2 行，说明该问题与本次已修复
的 planner submit/start/complete 重建是两条独立链路。

### 当前正式 1/2 是否受影响

独立读取正式源并把每个 `attempt_transition` 重建为 detail row 后：

```text
Attempt 1: events=845, detail=845, exact_sequence=True
Attempt 2: events=1257, detail=1257, exact_sequence=True
```

两者分别只达到 4096 上限的约 20.6% 和 30.7%，没有发生 deque
淘汰。进一步独立按 detail intervals 扫描完整源：

```text
raw selected from ball_samples.csv = 33894
raw_lcm.csv                      = 33894
policy ticks selected from events = 1578
policy_ticks.csv                  = 1578
stage detail total                = 2102
stage_timeline.csv                = 2102
```

因此当前正式 1/2 不受影响；风险面是未来 transition 超过 4096 的长
attempt，或 detail/events 因其他原因不一致的 session。

### 严重度与建议

- 严重度：**高**
- 置信度：**高**
- 原因：目标明确要求从完整磁盘事实源重建且禁止静默丢行，而当前可以
  成功输出少行并把全部 completeness checks 标为 `true`。

建议：

1. `stage_timeline.csv` 从完整 `attempt_transition` 事件流重建；
2. 从完整 transition 重建 segment intervals，再用于 raw 和 policy
   tick 筛选；不要把有界 detail segments 当完整事实源；
3. 将 detail 只用于交叉验证/兼容，而不是完整性的唯一来源；
4. 增加超过 4096 transition 的回归测试，以及
   events/detail 不一致时禁止静默成功的测试；
5. manifest 增加 transition event count、detail count、重建 segment
   count 和明确的相等/覆盖检查。

## B. warnings-as-errors 会造成提交成功但 CLI 失败

### 根因证据

`replace_output_directory` 在安装新目录并成功 fsync 父目录后才进入
backup 清理；清理失败调用 `warnings.warn(RuntimeWarning)`：
`deploy/diagnostics/export_hitter_task_csv.py:2264-2337`。

`main` 捕获所有 `Exception` 并返回 1：
`deploy/diagnostics/export_hitter_task_csv.py:2546-2561`。
Python warning 在 `error` filter 下会以 `RuntimeWarning` 异常抛出，
因此能越过已经完成的提交点。

### 最小复现

用 mock 只让 backup `shutil.rmtree` 失败，并启用
`warnings.simplefilter("error", RuntimeWarning)`：

```text
raised_type=RuntimeWarning
committed_output_value=new
backup_count=1
backup_value=old
temporary_exists=False
```

即：调用者收到异常，但 output 已经是新版本。相同异常进入 `main`
后会被当作命令失败并返回 1。

### 当前正式 1/2 是否受影响

正式输出目录当前是完整新结果，父目录不存在同名
`backup`、`failed-new` 或 temp 残留；此前运行也未使用
warnings-as-errors。因此当前正式覆盖不受影响。

### 严重度与建议

- 严重度：**中**
- 置信度：**高**
- 原因：不破坏已提交数据，但违反文档所述“提交后 cleanup warning
  不会把成功谎报为失败”，会误导自动化重试、状态判断和人工恢复。

建议：

1. 提交点之后不要使用可能被 filter 提升为异常的
   `warnings.warn`；
2. 将 post-commit 警告作为结构化返回值，或由 CLI 显式写 stderr /
   logger warning，同时保持返回码 0；
3. 新增 `warnings.simplefilter("error")` 回归测试，要求 output 为新
   版本、backup 可保留、函数/CLI 仍报告成功。

## C. planner key 使用 `int()` 造成宽松转换和身份改变

### 根因证据

`_planner_key` 对 attempt/schema/epoch/generation 全部直接调用
`int()`，`PlannerExportKey` 本身没有 `__post_init__` 校验：
`deploy/diagnostics/export_hitter_task_csv.py:201-206,558-574`。

同文件已经有严格 `_require_int`，它会拒绝 bool 和非 int，并支持
下界检查，但 planner key 没有使用它：
`deploy/diagnostics/export_hitter_task_csv.py:850-858`。

运行时 `SnapshotKey` 的真实约束是 epoch/generation 非负，序列化的
schema 固定为当前 schema：
`deploy/diagnostics/hitter_task_models.py:153-167`。

### 最小复现

直接调用当前 `_planner_key`：

```text
bool generation True       -> ACCEPTED, generation=1
float generation 7.9       -> ACCEPTED, generation=7
numeric strings "1","2","7" -> ACCEPTED, coerced to ints
track_epoch=-2,generation=-7 -> ACCEPTED
schema_version=2           -> ACCEPTED
```

端到端 fixture 又把 Attempt 2 的 snapshot schema 改成 2、generation
改成 21.9，并在 submit/start/complete/input/stage 中保持一致。导出
仍成功：

```text
export_returned_success=True
attempt2_output_schema=2
attempt2_output_generation=21
manifest_checks_all_true=True
```

这不是单纯的“允许宽松输入”：`21.9 -> 21` 改变了复合主键，可能把
两个不同原始 key 别名化，进而触发错误冲突或错误对齐。

### 当前正式 1/2 是否受影响

对正式源中所有选中 attempt 的 submit/start/complete、
replacement/result key，以及 detail 中的 planner input/stage key
逐项检查：

```text
invalid_planner_keys=0
```

所有 schema 都是整数 1，epoch/generation 都是非 bool 的非负整数。
正式三张核心表 2085 行 key 序列也已一致，因此当前产物不受影响。

### 严重度与建议

- 严重度：**高**
- 置信度：**高**
- 原因：畸形源数据会被“规范化”为另一个 key，破坏精确身份和
  fail-closed 语义；manifest 仍会声明完整。

建议：

1. planner key 的四个字段统一使用严格整数校验：
   `attempt_id >= 1`、`schema_version == 1`、
   `track_epoch >= 0`、`generation >= 0`；
2. 明确拒绝 bool、float、字符串和不支持的 schema，不做转换；
3. submit/start/complete/result/replacement/input/stage 共用同一严格
   parser；
4. 为每种畸形类型增加 scanner 和端到端测试，另加浮点截断造成 key
   alias 的回归测试。

## 正式产物总体复核摘要

```text
summary.csv             2
raw_lcm.csv         33894
planner_inputs.csv    2085
planner_calls.csv     2085
planner_results.csv   2085
stage_timeline.csv    2102
policy_ticks.csv      1578
task_observations.csv    2
```

- events/detail transition 序列逐条相同；
- raw/policy/stage 独立重算行数与输出一致；
- planner key 畸形数为 0；
- manifest 当前全部 completeness checks 为 `true`；
- 正式输出父目录无 backup/temp/failed-new 残留。

因此可继续使用当前 Attempt 1/2 CSV 做分析，但在上述 A、B、C 修复并
加回归测试前，不应把导出器视为对未来任意完整 session 已完全
fail-closed。
