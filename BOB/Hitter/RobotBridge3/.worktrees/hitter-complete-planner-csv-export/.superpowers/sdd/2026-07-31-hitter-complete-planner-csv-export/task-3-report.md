# Task 3 实施报告

## 结果

已实现完整 HITTER planner diagnostics 离线导出、事务式输出目录替换和四参数 CLI。

提交：

```text
a406253 feat: export complete HITTER planner diagnostics
```

提交仅包含：

- `deploy/diagnostics/export_hitter_task_csv.py`
- `deploy/tests/test_hitter_task_csv_export.py`
- `docs/hitter_complete_planner_csv_export.md`

未修改 recordings、实时 monitor、estimator、planner、policy、控制参数或 replay capture 上限。

## TDD 证据

RED：

```text
test_export_writes_complete_aligned_csv_set ... ERROR
AttributeError: module 'diagnostics.export_hitter_task_csv'
has no attribute 'export_session_csv'
Ran 1 test
FAILED (errors=1)
```

GREEN：

```text
test_export_writes_complete_aligned_csv_set ... ok
Ran 1 test
OK
```

随后增加并通过了以下关键失败路径：

- validation failure 保留已有输出 sentinel；
- incomplete source session 拒绝导出；
- 三张核心 CSV 关文件重读后发现 key 顺序篡改；
- 目录安装第二次 rename 失败时恢复旧输出；
- CLI help 包含四个参数，异常返回非零。

## 输出 schema

输出八张固定 header CSV：

1. `summary.csv`
2. `raw_lcm.csv`
3. `planner_inputs.csv`
4. `planner_calls.csv`
5. `planner_results.csv`
6. `stage_timeline.csv`
7. `policy_ticks.csv`
8. `task_observations.csv`

并输出 `export_manifest.json`。

三张核心表共享四字段复合 key：

```text
attempt_id
snapshot_key.schema_version
snapshot_key.track_epoch
snapshot_key.generation
```

它们只包含 actual completed calls，逐行 key、顺序和行数完全一致。`pending_replaced` 仅计数，不进入核心表。

辅助表语义：

- raw 使用 segment 闭区间和 `received_monotonic_s`，保留源 `attempt_id/track_segment_id`，新增 interval 归属列；
- stage 保留 detail 原始顺序并增加 `stage_index`；
- policy tick 来自完整 `events.jsonl`，保留真实 `event_id`；
- task observation 每 attempt 一行，A1 无 observation 时仍输出空值行；
- summary 保留 attempt detail 历史 `recording_complete`，不因离线恢复改写。

## 完整性与原子语义

完整性事实源仅为磁盘：

- `session.json`
- `events.jsonl`
- `ball_samples.csv`
- 所选 `attempt_details/<id>.json`

不读取 `ReplayCaptureStore` 或 replay bundle。

Fail-closed 检查包括：

- session `COMPLETE`、`recording_complete=true`、`recorder_healthy=true`；
- `last_error/failure` 为空、`input_samples_dropped=0`；
- persisted/observed watermark 一致；
- event id 从 1 连续到 watermark，且没有 `RECORDER_EVENT_GAP`；
- segment 时间有限、`first <= last`，同 attempt 重叠区间合并，跨 attempt 重叠拒绝；
- submit/start/complete 时间顺序、result key、input key/source frame；
- completed 与 pending-replaced 互斥且并集等于 submitted；
- 八张 CSV 关闭后重新读取并核验固定 header/行数；
- 三张核心表重新解析 typed key，核验同序、唯一和同数；
- 导出末尾重新 stat/hash 所有源文件，发现竞态即失败。

所有输出先写入目标同级 `.hitter-csv-*` 临时目录。覆盖时执行：

1. old output -> 同级唯一 backup；
2. validated temp -> output；
3. 成功后删除 backup；
4. 第二步失败则 backup -> output；
5. 恢复也失败时保留 backup，并在错误中报告路径和两个异常。

任何源/写入验证失败均发生在替换旧输出之前。

## 验证

Python：

```text
Python 3.8.20
```

完整回归：

```text
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest \
  tests.test_hitter_task_csv_export \
  tests.test_hitter_task_replay \
  tests.test_hitter_task_recording -v

Ran 85 tests in 2.276s
OK
```

额外验证：

```text
python -m py_compile exporter/tests
git diff --check
```

均退出 0。

真实 session 使用 `TemporaryDirectory` 导出，源 recordings 只读，临时输出自动删除：

```text
submitted=2625
pending_replaced=540
completed=2085

summary.csv=2
raw_lcm.csv=33894
planner_inputs.csv=2085
planner_calls.csv=2085
planner_results.csv=2085
stage_timeline.csv=2102
policy_ticks.csv=1578
task_observations.csv=2
```

结果与 Task 3 oracle 完全一致。

## 已知风险

- 非空目录替换依赖两次同文件系统 rename，两个 rename 之间目标路径会短暂不存在；实现提供 rollback，但不能消除进程在窗口内崩溃留下 backup 的可能。
- 导出器对当前 schema 采用严格固定 header/payload 校验。未来持久化 schema 新增字段时会 fail closed，需要显式升级 exporter schema，而不会静默丢列。
- Task observation 当前按 HITTER 已确认的 11 维 schema 展开；维度改变会明确报错。
