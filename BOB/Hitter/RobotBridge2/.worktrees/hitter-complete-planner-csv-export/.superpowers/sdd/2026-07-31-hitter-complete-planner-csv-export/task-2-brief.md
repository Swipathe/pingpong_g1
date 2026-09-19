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

