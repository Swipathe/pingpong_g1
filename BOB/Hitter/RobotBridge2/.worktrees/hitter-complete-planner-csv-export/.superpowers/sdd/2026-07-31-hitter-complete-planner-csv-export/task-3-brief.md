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

