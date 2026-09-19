# Task 3 独立数据 / 规格审查

## Verdict

**CHANGES REQUESTED**

当前真实 session 的导出结果、三张核心表和既定 oracle 全部吻合，但导出器仍有两条会把不完整或内部冲突的磁盘事实源标记为完整的 fail-open 路径。其中 `ball_samples.csv` 的 `input_seq` 完整性缺口直接影响“完整原始样本”这一主要交付承诺。

## Findings

### High — `ball_samples.csv` 不校验 `input_seq` 连续性 / 唯一性，缺失原始行仍会被声明完整

- 位置：
  - `deploy/diagnostics/export_hitter_task_csv.py:974`
  - `deploy/diagnostics/export_hitter_task_csv.py:1293`
  - `deploy/diagnostics/export_hitter_task_csv.py:2194`
  - `deploy/diagnostics/export_hitter_task_csv.py:2259`
- 影响：
  - `_validate_session_metadata()` 只相信 `session.json` 的 `input_samples_dropped=0`。
  - `_write_raw_lcm_csv()` 校验 header、列数和 `received_monotonic_s`，但从不解析或校验 `input_seq`。
  - `raw_lcm.csv` 的 expected row count 又直接采用同一次扫描实际写出的数量，因此源文件中间缺行、重复行或重排行时，关闭后重读的 count 仍会自洽。
  - SHA-256 只能证明“导出自当前这个文件”，不能证明这个文件保留了 recorder 原本写入的完整有序序列。最终 manifest 仍写入 `input_samples_not_dropped=true`，会让下游把不完整 raw 事实当成完整数据使用。
- 证据：
  - 临时完整 fixture 中删除 `ball_samples.csv` 的 `input_seq=2` 行，保持 `session.json` 的 `input_samples_dropped=0` 不变。
  - `export_session_csv()` 仍成功，输出：

    ```text
    RAW_GAP_ACCEPTED 5 True True
    ```

    即 `raw_lcm.csv` 少一行，但 manifest 的 `input_samples_not_dropped` 和 `source_files_unchanged_during_export` 都为 `true`。
  - 真实 session 的源序列当前确实健康：`input_seq=0..308509`，共 308510 行，重复数 0，相邻非 `+1` 数 0；因此增加该校验不会阻断本次真实导出。
- 建议：
  - 在任何 subject / interval 筛选之前，对 `ball_samples.csv` 全文件的 `input_seq` 做严格整数解析，并要求源文件顺序中唯一且逐行 `+1`；首值可以不是 0，以兼容 recorder 在进程运行中途开始。
  - 为缺号、重复和倒序分别增加失败测试，并验证失败发生在替换已有输出之前。
  - 若要覆盖文件尾部截断，还应在 session 元数据中持久化 raw row count 或末端 `input_seq` watermark，并由导出器核对；仅检查相邻序列无法识别尾部整体被截断。

### Medium — 落入所选闭区间但 `attempt_id` 与区间归属冲突的 lifecycle tick 被静默丢弃

- 位置：`deploy/diagnostics/export_hitter_task_csv.py:1515-1531`
- 影响：
  - Segment union 已为每个时刻建立唯一 attempt 归属，并拒绝所选 attempts 的跨 attempt 区间重叠。
  - lifecycle tick 落入该 union 后，如果事件的 `attempt_id` 与 interval owner 不同，当前代码直接 `continue`；它既不进入 `policy_ticks.csv`，也不触发错误。
  - 这会把磁盘事件与 segment 的身份冲突隐藏成“正常筛选”，同时 manifest 的全部 completeness checks 仍可为 `true`，不符合设计文档“字段冲突或不完整磁盘日志直接报错，禁止静默丢行”的约束。
- 证据：
  - 在临时 fixture 中，把原本位于 Attempt 1 闭区间内的 lifecycle tick 的 `attempt_id` 从 1 改为 2；Attempts 1/2 均为本次所选。
  - 导出仍成功，tick 行数从 2 变为 1，输出：

    ```text
    TICK_MISMATCH_ACCEPTED 1 True True
    ```

    最后两个 `True` 分别表示 `segments_valid_and_non_overlapping=true`、所有 manifest completeness checks 均为 `true`。
- 建议：
  - 当 tick 的 `attempt_id` 属于本次所选集合、时间也落入所选 union，但与 interval owner 不同，直接以明确错误 fail closed。
  - 对不属于所选 attempts 的事件可继续忽略，但应把这两种情况显式区分。
  - 增加 selected-attempt identity mismatch 回归测试；如果产品语义确实允许旧 attempt 的 tick 延迟进入下一 attempt 区间，则必须在设计和 manifest 中明确这种排除规则，不能仍把输出称为完整。

### Low — raw subject 白名单会让未来 pelvis 名称变化静默漏行

- 位置：`deploy/diagnostics/export_hitter_task_csv.py:1347-1350`
- 影响：
  - 当前代码固定只保留 `ball`、`g1pelvis`、`table`。这与当前 pipeline 和本次真实 session 一致。
  - 但设计同时承诺命令可用于后续 session。若后续系统在不改变 CSV header 的情况下把 pelvis canonical subject 改为例如 `g2pelvis`，导出器不会 fail closed，而会静默删除所有 pelvis 行；manifest 也没有记录 accepted / observed subject 集合。
- 证据：
  - 筛选条件是代码内三值 tuple，既不来自 session 元数据，也没有 unexpected-subject 检查。
  - 当前真实 session 未受影响；这是前向兼容性风险，不影响本次 33894 行 oracle。
- 建议：
  - 将 canonical subject 集合纳入明确的 source/export schema，并在不支持的 schema 或 subject role 上报错；或从版本化 session 元数据读取角色到 subject 的映射。
  - manifest 记录 observed subject counts，使 subject 漏失可审计。

## 已核实为正确的关键项

- `submit <= start <= complete` 并未遗漏：校验位于 `scan_planner_events()` 的 `deploy/diagnostics/export_hitter_task_csv.py:732-757`，而不是 `_validate_aligned_rows()`；`export_session_csv()` 必经该扫描路径。
- Segment 使用闭区间；同 attempt 重叠 / 相接区间合并，跨 attempt 重叠拒绝，实际 raw 行无重复。
- stage 按 detail 原始顺序输出并增加 0-based `stage_index`。
- 无 task observation 的 Attempt 1 仍输出一行全空 observation。
- summary 保留两个 attempt 原始 `recording_complete=False`，并分别写入 completed / pending 为 `837/213`、`1248/327`。
- 三张核心表关闭后按四字段 typed key 重读，2085 个 key 同数、唯一、严格同序；pending-replaced 泄漏数为 0。
- 源仅使用 `session.json`、`events.jsonl`、`ball_samples.csv` 和所选 attempt details；实现与文档均未使用 bounded replay capture。
- CLI 四参数和中文运行文档与计划一致。

## 验证证据

完整回归：

```text
Ran 85 tests in 2.209s
OK
```

真实 session 使用 `TemporaryDirectory` 导出并独立关文件重读：

```text
summary.csv             2
raw_lcm.csv         33894
planner_inputs.csv   2085
planner_calls.csv    2085
planner_results.csv  2085
stage_timeline.csv   2102
policy_ticks.csv     1578
task_observations.csv   2
```

额外 oracle：

```text
submitted / pending / completed = 2625 / 540 / 2085
Attempt 1 / Attempt 2 completed = 837 / 1248
三核心表 key 相等、唯一、manifest hash 一致
pending-replaced 与核心 key 交集 = 0
Attempt 2 generation 11192 = present
八张 CSV row count、source SHA-256、manifest round-trip = match
真实 policy tick selected-attempt mismatch = 0
```
