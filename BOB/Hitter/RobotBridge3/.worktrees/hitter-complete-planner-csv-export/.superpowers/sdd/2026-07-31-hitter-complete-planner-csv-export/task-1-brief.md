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

