# HITTER Complete Planner CSV Export Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 从完整磁盘日志恢复所有实际开始并完成的 HITTER planner 调用，排除 pending replacement，并生成逐 key、逐行严格对齐的完整 CSV。

**Architecture:** 新增一个纯离线导出模块，流式读取 `events.jsonl` 和 `ball_samples.csv`，并结合 `attempt_details/<id>.json` 重建完成调用。实时 monitor、estimator、planner 和有界 replay capture 保持不变；所有输出先写入同文件系统临时目录，完成严格校验后再替换目标分析目录。

**Tech Stack:** Python 3 标准库（`argparse`、`csv`、`dataclasses`、`hashlib`、`json`、`pathlib`、`tempfile`）、现有 `diagnostics.hitter_task_models.SnapshotKey` 数据约定、`unittest`。

## Global Constraints

- 不修改 estimator、planner、policy、真机控制参数或 latest-only worker 行为。
- 不取消或增大实时 monitor 的 8192 条 replay capture 内存保护。
- 完成调用必须同时具有 submit、start、complete/result 和 planner input。
- `pending_replaced` 且未 start/complete 的 key 不得出现在完成调用表。
- `planner_inputs.csv`、`planner_calls.csv`、`planner_results.csv` 必须具有相同 key、相同行数和相同顺序。
- 任一完整性规则失败时必须返回非零状态，且不得破坏已有输出。
- 只提交本计划涉及的明确文件；不得暂存当前脏工作区中的其他用户改动。

---

### Task 1: 从完整事件日志重建实际完成调用

**Files:**
- Create: `deploy/diagnostics/export_hitter_task_csv.py`
- Create: `deploy/tests/test_hitter_task_csv_export.py`

**Interfaces:**
- Consumes: session `events.jsonl` 中的 `planner_submit` 与 `planner_trace` 事件。
- Produces:
  - `PlannerExportKey(attempt_id, schema_version, track_epoch, generation)`
  - `CompletedPlannerCall(key, submitted_monotonic_s, started_monotonic_s, completed_monotonic_s, source_frame, result, pending_replaced_key, latest_replaced_key)`
  - `PlannerEventScan(submitted_keys, pending_replaced_keys, completed_calls)`
  - `scan_planner_events(events_path: Path, attempt_ids: Sequence[int]) -> PlannerEventScan`

- [ ] **Step 1: 写出能抓住“pending replacement 不应成为完成调用”的失败测试**

测试必须使用真实 JSONL 临时文件，不 mock 解析器。用两个 key：generation 10 被 generation 11 替换；只有 generation 11 出现 start 和 complete。

```python
class PlannerEventScanTests(unittest.TestCase):
    def test_scan_keeps_only_calls_that_started_and_completed(self):
        with tempfile.TemporaryDirectory() as directory:
            events_path = Path(directory) / "events.jsonl"
            events = [
                planner_submit_event(attempt_id=1, generation=10, t=1.0),
                planner_submit_event(attempt_id=1, generation=11, t=1.01),
                pending_replaced_event(
                    attempt_id=1,
                    generation=11,
                    replaced_generation=10,
                    t=1.011,
                ),
                planner_start_event(attempt_id=1, generation=11, t=1.02),
                planner_complete_event(
                    attempt_id=1,
                    generation=11,
                    source_frame=1011,
                    completed_t=1.03,
                    error_type="ValueError",
                    error_text="example rejection",
                ),
            ]
            write_json_lines(events_path, events)

            scan = scan_planner_events(events_path, [1])

            self.assertEqual(
                [call.key.generation for call in scan.completed_calls],
                [11],
            )
            self.assertEqual(
                {key.generation for key in scan.pending_replaced_keys},
                {10},
            )
            self.assertEqual(
                {key.generation for key in scan.submitted_keys},
                {10, 11},
            )
```

这项测试要防止的生产回归：把只有 `pending_replaced` 的输入错误计为一次完成调用。

- [ ] **Step 2: 运行单测并确认因导出模块不存在而失败**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python \
  -m unittest tests.test_hitter_task_csv_export.PlannerEventScanTests.test_scan_keeps_only_calls_that_started_and_completed -v
```

Expected: `ModuleNotFoundError: No module named 'diagnostics.export_hitter_task_csv'`。

- [ ] **Step 3: 实现最小事件扫描器**

在新模块中定义：

```python
@dataclass(frozen=True, order=True)
class PlannerExportKey:
    attempt_id: int
    schema_version: int
    track_epoch: int
    generation: int


@dataclass(frozen=True)
class CompletedPlannerCall:
    key: PlannerExportKey
    submitted_monotonic_s: float
    started_monotonic_s: float
    completed_monotonic_s: float
    source_frame: int
    result: Mapping[str, Any]
    pending_replaced_key: Optional[PlannerExportKey]
    latest_replaced_key: Optional[PlannerExportKey]


@dataclass(frozen=True)
class PlannerEventScan:
    submitted_keys: FrozenSet[PlannerExportKey]
    pending_replaced_keys: FrozenSet[PlannerExportKey]
    completed_calls: Tuple[CompletedPlannerCall, ...]
```

`scan_planner_events()` 必须：

1. 逐行 `json.loads()`，不把完整 `events.jsonl` 读入内存；
2. 只处理指定 attempt；
3. 用 `(attempt_id, schema_version, track_epoch, generation)` 建 key；
4. 将 `pending_replaced` 的 `replaced_snapshot_key` 记为被丢弃 key；
5. 只为同时具有 submit、start、complete/result 的 key 创建完成调用；
6. 对重复但内容冲突的 submit/start/complete 抛出 `ValueError`；
7. 按 `(completed_monotonic_s, key)` 排序。

- [ ] **Step 4: 增加计数闭合测试并确认 RED**

```python
def test_scan_rejects_unclassified_submitted_key(self):
    # generation 20 只有 submit，既未完成也未被替换。
    with self.assertRaisesRegex(
        ValueError,
        "submitted keys are neither completed nor pending-replaced",
    ):
        scan_planner_events(events_path, [1])
```

Run:

```bash
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python \
  -m unittest tests.test_hitter_task_csv_export.PlannerEventScanTests.test_scan_rejects_unclassified_submitted_key -v
```

Expected: FAIL，因为最小扫描器尚未校验 `completed + pending_replaced == submitted`。

- [ ] **Step 5: 实现严格计数闭合并运行 Task 1 测试**

严格比较三个 key 集合：

```python
completed_keys = {call.key for call in completed_calls}
classified_keys = completed_keys | pending_replaced_keys
if classified_keys != submitted_keys:
    missing = sorted(submitted_keys - classified_keys)
    unexpected = sorted(classified_keys - submitted_keys)
    raise ValueError(
        "submitted keys are neither completed nor pending-replaced: "
        "missing={!r}, unexpected={!r}".format(missing, unexpected)
    )
```

Run:

```bash
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python \
  -m unittest tests.test_hitter_task_csv_export.PlannerEventScanTests -v
```

Expected: PASS。

- [ ] **Step 6: 仅提交 Task 1 文件**

```bash
git add -- \
  deploy/diagnostics/export_hitter_task_csv.py \
  deploy/tests/test_hitter_task_csv_export.py
git diff --cached --check
git commit -m "feat: reconstruct completed HITTER planner calls"
```

---

### Task 2: 严格连接完成调用与 Planner 输入

**Files:**
- Modify: `deploy/diagnostics/export_hitter_task_csv.py`
- Modify: `deploy/tests/test_hitter_task_csv_export.py`

**Interfaces:**
- Consumes:
  - Task 1 的 `PlannerEventScan.completed_calls`
  - `attempt_details/<id>.json` 中的 `planner_inputs`
- Produces:
  - `AlignedPlannerRow(call: CompletedPlannerCall, input_row: Mapping[str, Any])`
  - `load_attempt_inputs(session_dir: Path, attempt_ids: Sequence[int]) -> Mapping[PlannerExportKey, Mapping[str, Any]]`
  - `align_completed_calls(scan: PlannerEventScan, inputs: Mapping[PlannerExportKey, Mapping[str, Any]]) -> Tuple[AlignedPlannerRow, ...]`
  - `planner_input_csv_rows()`
  - `planner_call_csv_rows()`
  - `planner_result_csv_rows()`

- [ ] **Step 1: 写出逐行对齐的失败测试**

手工提供两个完成调用，attempt detail 中故意以反序保存输入。断言三个输出仍按 complete time 使用完全相同的 key 顺序。

```python
def test_three_core_tables_share_the_same_completed_key_order(self):
    calls = (
        completed_call(generation=31, completed_t=3.1, source_frame=1031),
        completed_call(generation=42, completed_t=4.2, source_frame=1042),
    )
    inputs = {
        export_key(generation=42): planner_input(
            generation=42,
            source_frame=1042,
            velocity_x=-1.2,
        ),
        export_key(generation=31): planner_input(
            generation=31,
            source_frame=1031,
            velocity_x=0.4,
        ),
    }

    aligned = align_completed_calls(planner_scan(calls), inputs)
    input_rows = planner_input_csv_rows(aligned)
    call_rows = planner_call_csv_rows(aligned)
    result_rows = planner_result_csv_rows(aligned)

    expected = [(1, 1, 31), (1, 1, 42)]
    self.assertEqual(csv_keys(input_rows), expected)
    self.assertEqual(csv_keys(call_rows), expected)
    self.assertEqual(csv_keys(result_rows), expected)
```

这项测试要防止的生产回归：三张 CSV 分别排序，导致相同行号不是同一个 snapshot。

- [ ] **Step 2: 运行测试并确认 RED**

Run:

```bash
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python \
  -m unittest tests.test_hitter_task_csv_export.AlignedPlannerRowsTests.test_three_core_tables_share_the_same_completed_key_order -v
```

Expected: FAIL，因为对齐 API 尚未实现。

- [ ] **Step 3: 实现输入加载、严格连接和稳定 JSON/CSV 展平**

`load_attempt_inputs()` 必须：

1. 按 attempt id 打开 `attempt_details/<id>.json`；
2. 读取 `planner_inputs`；
3. 将 detail 中的 `snapshot_key` 转换为 `PlannerExportKey`；
4. 拒绝重复 key；
5. 保留所有向量为紧凑 JSON 字符串，禁止 Python repr。

`align_completed_calls()` 必须：

```python
for call in scan.completed_calls:
    input_row = inputs.get(call.key)
    if input_row is None:
        raise ValueError("completed planner call has no input: {!r}".format(call.key))
    if int(input_row["source_frame"]) != call.source_frame:
        raise ValueError("source_frame mismatch for {!r}".format(call.key))
    rows.append(AlignedPlannerRow(call=call, input_row=input_row))
```

三组 CSV row builder 均遍历同一个 `aligned` tuple，不允许各自重新查询或排序。

- [ ] **Step 4: 写出缺输入、重复 key 和 frame 冲突测试并确认 RED**

```python
def test_completed_call_without_input_is_rejected(self):
    with self.assertRaisesRegex(ValueError, "has no input"):
        align_completed_calls(scan_with_generation(9), {})


def test_duplicate_planner_input_key_is_rejected(self):
    with self.assertRaisesRegex(ValueError, "duplicate planner input key"):
        load_attempt_inputs(session_dir_with_duplicate_inputs(), [1])


def test_source_frame_conflict_is_rejected(self):
    call = completed_call(generation=9, source_frame=1009)
    input_row = planner_input(generation=9, source_frame=9999)
    with self.assertRaisesRegex(ValueError, "source_frame mismatch"):
        align_completed_calls(planner_scan((call,)), {call.key: input_row})
```

Run:

```bash
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python \
  -m unittest \
    tests.test_hitter_task_csv_export.AlignedPlannerRowsTests.test_completed_call_without_input_is_rejected \
    tests.test_hitter_task_csv_export.AlignedPlannerRowsTests.test_duplicate_planner_input_key_is_rejected \
    tests.test_hitter_task_csv_export.AlignedPlannerRowsTests.test_source_frame_conflict_is_rejected \
    -v
```

Expected: 新增测试至少一个 FAIL，直到所有严格校验实现。

- [ ] **Step 5: 完成严格校验并运行 Task 1/2 测试**

Run:

```bash
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python \
  -m unittest \
    tests.test_hitter_task_csv_export.PlannerEventScanTests \
    tests.test_hitter_task_csv_export.AlignedPlannerRowsTests \
    -v
```

Expected: PASS。

- [ ] **Step 6: 仅提交 Task 2 文件**

```bash
git add -- \
  deploy/diagnostics/export_hitter_task_csv.py \
  deploy/tests/test_hitter_task_csv_export.py
git diff --cached --check
git commit -m "feat: align completed HITTER planner CSV rows"
```

---

### Task 3: 完整辅助表、事务式输出和 CLI

**Files:**
- Modify: `deploy/diagnostics/export_hitter_task_csv.py`
- Modify: `deploy/tests/test_hitter_task_csv_export.py`
- Create: `docs/hitter_complete_planner_csv_export.md`

**Interfaces:**
- Consumes:
  - Task 2 的 aligned rows
  - 完整 `ball_samples.csv`
  - 完整 `events.jsonl`
  - attempt detail 的 segments、stage timeline 和 task observation
  - `session.json`
- Produces:
  - `ExportManifest`
  - `export_session_csv(session_dir: Path, attempt_ids: Sequence[int], output_dir: Path, overwrite: bool) -> ExportManifest`
  - `main(argv: Optional[Sequence[str]] = None) -> int`

- [ ] **Step 1: 写出端到端失败测试**

临时 session fixture 必须包含：

- `session.json`，`status=COMPLETE`、`recording_complete=true`；
- 两个 attempt detail；
- 区间内 ball/pelvis/table 原始行和区间外原始行；
- 一个 pending-replaced 输入；
- 两个实际完成调用；
- lifecycle tick、stage 和 task observation。

```python
def test_export_writes_complete_aligned_csv_set(self):
    session_dir = build_complete_session_fixture()
    output_dir = session_dir / "analysis"

    manifest = export_session_csv(
        session_dir=session_dir,
        attempt_ids=[1, 2],
        output_dir=output_dir,
        overwrite=False,
    )

    self.assertEqual(manifest.completed_call_count, 2)
    self.assertEqual(manifest.pending_replaced_count, 1)
    self.assertEqual(csv_row_count(output_dir / "planner_inputs.csv"), 2)
    self.assertEqual(csv_row_count(output_dir / "planner_calls.csv"), 2)
    self.assertEqual(csv_row_count(output_dir / "planner_results.csv"), 2)
    self.assertEqual(
        csv_key_sequence(output_dir / "planner_inputs.csv"),
        csv_key_sequence(output_dir / "planner_calls.csv"),
    )
    self.assertEqual(
        csv_key_sequence(output_dir / "planner_calls.csv"),
        csv_key_sequence(output_dir / "planner_results.csv"),
    )
    self.assertNotIn(
        pending_replaced_key(),
        csv_key_sequence(output_dir / "planner_calls.csv"),
    )
```

这项测试要防止的生产回归：输出文件存在但仍由不同数据源产生不同粒度。

- [ ] **Step 2: 运行端到端测试并确认 RED**

Run:

```bash
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python \
  -m unittest tests.test_hitter_task_csv_export.ExportSessionCsvTests.test_export_writes_complete_aligned_csv_set -v
```

Expected: FAIL，因为 `export_session_csv()` 尚未实现。

- [ ] **Step 3: 实现完整辅助表提取**

实现以下行为：

1. 从 attempt detail 的 segments 建立互不重叠的 monotonic time intervals；
2. 流式扫描 `ball_samples.csv`，将区间内 ball、pelvis、table 写入 `raw_lcm.csv`；
3. 流式扫描 `events.jsonl`，将区间内 lifecycle tick 写入 `policy_ticks.csv`；
4. 从 detail 展平 stage timeline 和 task observations；
5. `summary.csv` 保留原 `recording_complete`，新增：
   - `completed_call_export_complete=True`
   - `completed_call_count`
   - `pending_replaced_count`
6. `export_manifest.json` 写入：
   - schema version
   - source session absolute path
   - attempt ids
   - SHA-256 of `session.json`, `events.jsonl`, `ball_samples.csv`, each attempt detail
   - row counts
   - core key sequence SHA-256
   - completeness checks

- [ ] **Step 4: 实现事务式目录替换**

导出顺序必须是：

```python
temporary_dir = Path(tempfile.mkdtemp(prefix=".hitter-csv-", dir=str(output_dir.parent)))
try:
    write_all_outputs(temporary_dir, ...)
    validate_written_outputs(temporary_dir, ...)
    replace_output_directory(temporary_dir, output_dir, overwrite=overwrite)
except BaseException:
    shutil.rmtree(temporary_dir, ignore_errors=True)
    raise
```

`replace_output_directory()` 在 overwrite 时：

1. 将旧目录重命名为同级唯一 backup；
2. 将完整临时目录重命名为目标目录；
3. 成功后删除 backup；
4. 第二步失败时把 backup 恢复为目标目录。

- [ ] **Step 5: 写出失败不破坏旧文件的测试并确认 RED**

```python
def test_validation_failure_preserves_existing_output(self):
    session_dir = build_session_fixture_missing_completed_input()
    output_dir = session_dir / "analysis"
    output_dir.mkdir()
    sentinel = output_dir / "sentinel.txt"
    sentinel.write_text("keep-me", encoding="utf-8")

    with self.assertRaisesRegex(ValueError, "has no input"):
        export_session_csv(
            session_dir=session_dir,
            attempt_ids=[1],
            output_dir=output_dir,
            overwrite=True,
        )

    self.assertEqual(sentinel.read_text(encoding="utf-8"), "keep-me")
    self.assertEqual(list(output_dir.iterdir()), [sentinel])
```

Run:

```bash
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python \
  -m unittest tests.test_hitter_task_csv_export.ExportSessionCsvTests.test_validation_failure_preserves_existing_output -v
```

Expected: FAIL，直到临时目录验证与替换顺序正确。

- [ ] **Step 6: 实现 CLI 并验证帮助输出**

CLI 参数：

```python
parser.add_argument("--session", type=Path, required=True)
parser.add_argument("--attempts", type=int, nargs="+", required=True)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--overwrite", action="store_true")
```

Run:

```bash
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python \
  -m diagnostics.export_hitter_task_csv --help
```

Expected: 退出码 0，帮助中包含四个参数。

- [ ] **Step 7: 更新运行文档**

在 `docs/hitter_complete_planner_csv_export.md` 写入完整离线导出命令，并明确：

- 实时网页允许有界；
- 完整分析必须使用本导出器；
- 三张核心表只包含 actual completed calls；
- pending-replaced 不属于完成调用。

- [ ] **Step 8: 运行新增测试和现有 diagnostics 回归**

Run:

```bash
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python \
  -m unittest \
    tests.test_hitter_task_csv_export \
    tests.test_hitter_task_replay \
    tests.test_hitter_task_recording \
    -v
```

Expected: PASS。

- [ ] **Step 9: 仅提交 Task 3 文件**

```bash
git add -- \
  deploy/diagnostics/export_hitter_task_csv.py \
  deploy/tests/test_hitter_task_csv_export.py \
  docs/hitter_complete_planner_csv_export.md
git diff --cached --check
git commit -m "feat: export complete HITTER planner diagnostics"
```

---

### Task 4: 重建当前 Attempt 1/2 并执行真实数据验收

**Files:**
- Generate/replace, do not commit: `recordings/hitter_task_diagnostics/20260730_122056_381218-p994970-d50bc89f/analysis_attempts_1_2_20260730_194528/*`
- Verify: `recordings/hitter_task_diagnostics/20260730_194528_402494-p740921-f85e25f0/session.json`
- Verify: `recordings/hitter_task_diagnostics/20260730_194528_402494-p740921-f85e25f0/events.jsonl`
- Verify: `recordings/hitter_task_diagnostics/20260730_194528_402494-p740921-f85e25f0/ball_samples.csv`

**Interfaces:**
- Consumes: Task 3 CLI。
- Produces: 八张完整 CSV、`export_manifest.json` 和验收证据。

- [ ] **Step 1: 执行真实数据导出**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy

PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python \
  -m diagnostics.export_hitter_task_csv \
  --session ../recordings/hitter_task_diagnostics/20260730_194528_402494-p740921-f85e25f0 \
  --attempts 1 2 \
  --output ../recordings/hitter_task_diagnostics/20260730_122056_381218-p994970-d50bc89f/analysis_attempts_1_2_20260730_194528 \
  --overwrite
```

Expected: 退出码 0；输出报告 Attempt 1 完成 837、Attempt 2 完成 1248、合计 2085。

- [ ] **Step 2: 验证三张核心表逐行完全对齐**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2

/home/loco1/miniconda3/envs/rb/bin/python - <<'PY'
import csv
from pathlib import Path

root = Path(
    "recordings/hitter_task_diagnostics/"
    "20260730_122056_381218-p994970-d50bc89f/"
    "analysis_attempts_1_2_20260730_194528"
)

def keys(name):
    with (root / name).open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    values = [
        (
            int(row["attempt_id"]),
            int(row["snapshot_key.schema_version"]),
            int(row["snapshot_key.track_epoch"]),
            int(row["snapshot_key.generation"]),
        )
        for row in rows
    ]
    assert len(values) == len(set(values)), (name, "duplicate key")
    return values

inputs = keys("planner_inputs.csv")
calls = keys("planner_calls.csv")
results = keys("planner_results.csv")
assert len(inputs) == 2085, len(inputs)
assert inputs == calls == results
assert (2, 1, 1, 11192) in inputs
print("ALIGNED_COMPLETED_CALLS=2085")
PY
```

Expected: `ALIGNED_COMPLETED_CALLS=2085`。

- [ ] **Step 3: 验证 attempt 计数和 pending replacement 排除**

Run:

```bash
/home/loco1/miniconda3/envs/rb/bin/python - <<'PY'
import csv
from collections import Counter
from pathlib import Path

root = Path(
    "recordings/hitter_task_diagnostics/"
    "20260730_122056_381218-p994970-d50bc89f/"
    "analysis_attempts_1_2_20260730_194528"
)
with (root / "planner_calls.csv").open(newline="", encoding="utf-8") as stream:
    rows = list(csv.DictReader(stream))
counts = Counter(int(row["attempt_id"]) for row in rows)
assert counts == {1: 837, 2: 1248}, counts
print("ATTEMPT_COUNTS", dict(counts))
PY
```

Expected: `ATTEMPT_COUNTS {1: 837, 2: 1248}`。

- [ ] **Step 4: 验证 manifest、源哈希和完整性状态**

Run:

```bash
/home/loco1/miniconda3/envs/rb/bin/python - <<'PY'
import json
from pathlib import Path

path = Path(
    "recordings/hitter_task_diagnostics/"
    "20260730_122056_381218-p994970-d50bc89f/"
    "analysis_attempts_1_2_20260730_194528/export_manifest.json"
)
manifest = json.loads(path.read_text(encoding="utf-8"))
assert manifest["completed_call_count"] == 2085
assert manifest["pending_replaced_count"] == 540
assert manifest["checks"]["core_key_sequences_equal"] is True
assert manifest["checks"]["source_session_recording_complete"] is True
print("EXPORT_COMPLETE")
PY
```

Expected: `EXPORT_COMPLETE`。

- [ ] **Step 5: 运行最终测试与语法检查**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy

/home/loco1/miniconda3/envs/rb/bin/python -m py_compile \
  diagnostics/export_hitter_task_csv.py \
  tests/test_hitter_task_csv_export.py

PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python \
  -m unittest \
    tests.test_hitter_task_csv_export \
    tests.test_hitter_task_replay \
    tests.test_hitter_task_recording \
    -v
```

Expected: 所有命令退出码 0。

- [ ] **Step 6: 检查提交范围**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2
git status --short
git log -4 --oneline
```

Expected: 代码提交只包含 exporter、对应测试和诊断文档；生成的 `recordings/` 文件保留在本地，不提交，不触碰其他脏工作区改动。
