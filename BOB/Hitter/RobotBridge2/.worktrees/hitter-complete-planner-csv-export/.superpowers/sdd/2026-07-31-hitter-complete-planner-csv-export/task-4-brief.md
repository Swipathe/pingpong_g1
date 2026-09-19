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
