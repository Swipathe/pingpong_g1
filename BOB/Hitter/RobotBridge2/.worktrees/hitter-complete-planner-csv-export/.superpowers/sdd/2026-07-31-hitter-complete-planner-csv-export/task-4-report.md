# Task 4 真实导出与验收报告

## 结论

已在指定工作树和指定提交上完成 Attempt 1/2 的正式事务式覆盖导出，并从磁盘独立重读验收。

- 工作树：`/home/loco1/BOB/Hitter/RobotBridge2/.worktrees/hitter-complete-planner-csv-export`
- HEAD：`931ce6da189a0c74f19ebffc257cfe9bfe21e217`
- 源目录（只读）：
  `/home/loco1/BOB/Hitter/RobotBridge2/recordings/hitter_task_diagnostics/20260730_194528_402494-p740921-f85e25f0`
- 正式目标：
  `/home/loco1/BOB/Hitter/RobotBridge2/recordings/hitter_task_diagnostics/20260730_122056_381218-p994970-d50bc89f/analysis_attempts_1_2_20260730_194528`
- 导出、独立验收、语法检查和 95 项回归均退出 0。
- 未手工删除目标目录，未修改源数据，未提交 recordings。
- CLI 没有产生 warning；最终没有 backup、failed-new 或 `.hitter-csv-*` 临时残留。

## 正式导出

在工作树的 `deploy/` 中执行：

```bash
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python \
  -m diagnostics.export_hitter_task_csv \
  --session /home/loco1/BOB/Hitter/RobotBridge2/recordings/hitter_task_diagnostics/20260730_194528_402494-p740921-f85e25f0 \
  --attempts 1 2 \
  --output /home/loco1/BOB/Hitter/RobotBridge2/recordings/hitter_task_diagnostics/20260730_122056_381218-p994970-d50bc89f/analysis_attempts_1_2_20260730_194528 \
  --overwrite
```

退出码：`0`。

CLI manifest 摘要：

```text
submitted_count=2625
pending_replaced_count=540
completed_call_count=2085
core_key_sequence_sha256=83a6f8c272520d35de54bbc4b002e01ab8b50ed236d960a5127fb0ff557c747d
```

目标在导出前已存在。本次由 CLI 的 `--overwrite` 事务流程完成替换，没有预删或手工移动目标。

## 独立磁盘重读

使用独立 Python 脚本直接读取正式目标的 CSV、manifest 和源
`events.jsonl`，不复用导出函数返回的内存对象。脚本退出码：`0`。

### 产物集合和行数

目标目录恰好包含八张 CSV 和一个 `export_manifest.json`，没有额外文件或子目录。

| 文件 | 数据行数 |
|---|---:|
| `summary.csv` | 2 |
| `raw_lcm.csv` | 33894 |
| `planner_inputs.csv` | 2085 |
| `planner_calls.csv` | 2085 |
| `planner_results.csv` | 2085 |
| `stage_timeline.csv` | 2102 |
| `policy_ticks.csv` | 1578 |
| `task_observations.csv` | 2 |

磁盘重算行数与 `export_manifest.json["row_counts"]` 完全一致。

### 三张核心表

重读并按以下四字段构造 typed key：

```text
attempt_id
snapshot_key.schema_version
snapshot_key.track_epoch
snapshot_key.generation
```

验收结果：

```text
planner_inputs == planner_calls == planner_results（逐行、同序）
总 key 数=2085
三表各自 key 唯一
Attempt 1=837
Attempt 2=1248
key (2, 1, 1, 11192) 存在
重算 core key SHA-256=
83a6f8c272520d35de54bbc4b002e01ab8b50ed236d960a5127fb0ff557c747d
```

重算 hash 与 manifest 完全一致。

### submitted/completed/pending 语义

独立扫描源 `events.jsonl` 中 Attempt 1/2 的 `planner_trace`：

```text
submitted unique=2625
completed unique=2085
pending_replaced unique=540
completed 与 pending_replaced 不相交
submitted == completed ∪ pending_replaced
核心表 key 序列 == complete 事件 key 序列
核心表与 pending_replaced key 集合不相交
pending：Attempt 1=213，Attempt 2=327
```

因此 540 个被 pending replacement 丢弃的 key 没有泄漏到三张核心表。

### manifest 和 summary

manifest 磁盘重读结果：

```text
submitted_count=2625
pending_replaced_count=540
completed_call_count=2085
completeness_checks：20/20 均为 true
row_counts：与磁盘重算一致
core_key_sequence_sha256：与独立重算一致
source_sha256：与五个源文件独立重算一致
```

`summary.csv` 保留历史 `recording_complete=False`，并新增完整导出事实：

| Attempt | recording_complete | completed_call_export_complete | completed_call_count | pending_replaced_count |
|---:|---|---|---:|---:|
| 1 | False | True | 837 | 213 |
| 2 | False | True | 1248 | 327 |

最终 manifest 文件自身 SHA-256：

```text
efcf1b9a4cd78e45c5cf87eaba903575a074afb5f94ee99006919ea719818f45
```

## 源文件前后不变

导出前记录 SHA-256；导出、独立验收和测试完成后再次执行：

```bash
sha256sum \
  /home/loco1/BOB/Hitter/RobotBridge2/recordings/hitter_task_diagnostics/20260730_194528_402494-p740921-f85e25f0/session.json \
  /home/loco1/BOB/Hitter/RobotBridge2/recordings/hitter_task_diagnostics/20260730_194528_402494-p740921-f85e25f0/events.jsonl \
  /home/loco1/BOB/Hitter/RobotBridge2/recordings/hitter_task_diagnostics/20260730_194528_402494-p740921-f85e25f0/ball_samples.csv \
  /home/loco1/BOB/Hitter/RobotBridge2/recordings/hitter_task_diagnostics/20260730_194528_402494-p740921-f85e25f0/attempt_details/1.json \
  /home/loco1/BOB/Hitter/RobotBridge2/recordings/hitter_task_diagnostics/20260730_194528_402494-p740921-f85e25f0/attempt_details/2.json
```

退出码：`0`。前后值完全相同，且均与 manifest 一致：

| 源文件 | 导出前/最终 SHA-256 |
|---|---|
| `session.json` | `fa3541b2575a66f2b672b37870057e682a835a21b84eee453b827d7b57f1de8e` |
| `events.jsonl` | `5d3f8c9091296be88789b768d2eac9de3d60aa905f26047b5c2189c6a0cdac3a` |
| `ball_samples.csv` | `1dbf2cf98a20b3fbafa7c2b79c35d20d254f6f12499e7a7eda64d63bec682f19` |
| `attempt_details/1.json` | `9143804af8570abe1878beebe1f9d317e1a6a94e9f2aca7bef7f888ee10545c0` |
| `attempt_details/2.json` | `4af0064ec61c229ca855d4bcfc0e470b269f12e3808a772bb4dc3dd281f3fb83` |

## 事务残留检查

最终执行：

```bash
find /home/loco1/BOB/Hitter/RobotBridge2/recordings/hitter_task_diagnostics/20260730_122056_381218-p994970-d50bc89f \
  -maxdepth 1 \
  \( -name '.hitter-csv-*' \
     -o -name '.analysis_attempts_1_2_20260730_194528-backup-*' \
     -o -name '.analysis_attempts_1_2_20260730_194528-failed-new-*' \) \
  -printf '%f %y\n'
```

退出码：`0`；输出为空。没有 temp/backup/failed-new 残留，也没有 cleanup warning。

## 语法检查和完整回归

在工作树 `deploy/` 中执行：

```bash
/home/loco1/miniconda3/envs/rb/bin/python -m py_compile \
  diagnostics/export_hitter_task_csv.py \
  tests/test_hitter_task_csv_export.py
```

退出码：`0`。

```bash
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python \
  -m unittest \
    tests.test_hitter_task_csv_export \
    tests.test_hitter_task_replay \
    tests.test_hitter_task_recording \
    -v
```

退出码：`0`。

```text
Ran 95 tests in 2.435s
OK
```

## Git 范围检查

最终执行：

```bash
git rev-parse HEAD
git status --short --branch
git diff --check
git diff --cached --check
git ls-files -- recordings/hitter_task_diagnostics/20260730_122056_381218-p994970-d50bc89f/analysis_attempts_1_2_20260730_194528
git show --pretty='' --name-only HEAD
```

上述命令均退出 `0`。证据：

```text
HEAD=931ce6da189a0c74f19ebffc257cfe9bfe21e217
git status 仅显示分支标题，没有任何 staged/unstaged/untracked 项
git diff --check 无输出
git diff --cached --check 无输出
git ls-files 对正式 recordings 目标无输出
HEAD 仅包含：
  deploy/diagnostics/export_hitter_task_csv.py
  deploy/tests/test_hitter_task_csv_export.py
  docs/hitter_complete_planner_csv_export.md
```

源码工作树保持干净；正式导出数据仅保留在本地 recordings 路径中，未进入提交。
