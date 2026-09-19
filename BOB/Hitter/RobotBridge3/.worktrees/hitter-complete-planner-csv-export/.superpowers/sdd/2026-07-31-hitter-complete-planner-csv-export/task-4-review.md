# Task 4 正式输出独立复核

## 结论

**APPROVED**

未发现 finding。正式输出满足 `task-4-brief.md` 的数据验收要求，
并与 `task-4-report.md` 中的输出声明一致。

本复核直接读取磁盘文件；核心键、事件分类、哈希和行数均由独立脚本重算，
未导入或调用 exporter 实现。`events.jsonl` 逐行流式读取，未修改源目录或
正式输出目录。本 review 文件位于 Task 文档目录，不属于正式输出集合。

## 正式产物集合

正式目标恰好包含八张 CSV 和一个 `export_manifest.json`，全部为普通文件；
没有额外文件或子目录。目标父目录没有 `.hitter-csv-*`、
`*-backup-*` 或 `*-failed-new-*` 残留。

| 文件 | 独立重算数据行数 |
|---|---:|
| `summary.csv` | 2 |
| `raw_lcm.csv` | 33894 |
| `planner_inputs.csv` | 2085 |
| `planner_calls.csv` | 2085 |
| `planner_results.csv` | 2085 |
| `stage_timeline.csv` | 2102 |
| `policy_ticks.csv` | 1578 |
| `task_observations.csv` | 2 |

上述行数与 manifest 的 `row_counts` 精确相等。

## 三张核心表

按以下四字段读取为整数 key：

1. `attempt_id`
2. `snapshot_key.schema_version`
3. `snapshot_key.track_epoch`
4. `snapshot_key.generation`

独立验算结果：

- `planner_inputs.csv`、`planner_calls.csv`、`planner_results.csv` 的
  2085 个 key 逐行、同序完全相等。
- 三表各自均无重复 key。
- Attempt 1 为 837 行，Attempt 2 为 1248 行。
- key `(2, 1, 1, 11192)` 存在。
- 独立按 compact JSON key 序列重算 SHA-256 为
  `83a6f8c272520d35de54bbc4b002e01ab8b50ed236d960a5127fb0ff557c747d`，
  与 manifest 精确相等。

## 事件流独立分类

直接逐行流式扫描源 `events.jsonl`，以 `planner_submit` 记录 submitted，
以 `planner_trace/complete` 记录 completed，以
`planner_trace/pending_replaced` 的 `replaced_snapshot_key` 记录 pending：

```text
submitted unique = 2625
completed unique = 2085
pending_replaced unique = 540
completed ∩ pending_replaced = 0
submitted = completed ∪ pending_replaced
核心 CSV key 集合 = completed key 集合
核心 CSV key 序列 = complete 事件 key 序列
```

分 Attempt 计数：

| Attempt | completed | pending_replaced |
|---:|---:|---:|
| 1 | 837 | 213 |
| 2 | 1248 | 327 |

因此 pending replacement key 没有泄漏进三张核心表。

## Manifest 与 summary

`export_manifest.json` 独立重读和重算结果：

- `submitted_count=2625`
- `pending_replaced_count=540`
- `completed_call_count=2085`
- `row_counts` 与八张 CSV 的磁盘重算结果相等
- `core_key_sequence_sha256` 与独立重算值相等
- `source_sha256` 与五个源文件的当前独立重算值逐项相等
- `source_file_stats` 的 device、inode、size、mtime_ns 与当前源文件逐项相等
- `completeness_checks` 恰含预期 20 项，20/20 均为布尔值 `true`，
  没有缺项、额外项或 false 项

manifest 文件自身 SHA-256：

```text
efcf1b9a4cd78e45c5cf87eaba903575a074afb5f94ee99006919ea719818f45
```

`summary.csv` 保留历史状态并正确写入本次完整导出状态：

| Attempt | recording_complete | completed_call_export_complete | completed_call_count | pending_replaced_count |
|---:|---|---|---:|---:|
| 1 | False | True | 837 | 213 |
| 2 | False | True | 1248 | 327 |

## 源文件不变性

复核时从磁盘重算的五个源文件 SHA-256 与 manifest、Task 4 报告完全一致：

| 源文件 | SHA-256 |
|---|---|
| `session.json` | `fa3541b2575a66f2b672b37870057e682a835a21b84eee453b827d7b57f1de8e` |
| `events.jsonl` | `5d3f8c9091296be88789b768d2eac9de3d60aa905f26047b5c2189c6a0cdac3a` |
| `ball_samples.csv` | `1dbf2cf98a20b3fbafa7c2b79c35d20d254f6f12499e7a7eda64d63bec682f19` |
| `attempt_details/1.json` | `9143804af8570abe1878beebe1f9d317e1a6a94e9f2aca7bef7f888ee10545c0` |
| `attempt_details/2.json` | `4af0064ec61c229ca855d4bcfc0e470b269f12e3808a772bb4dc3dd281f3fb83` |

源文件的 stat 四元组也与 manifest 的导出时快照精确相等，未发现源文件变化。
