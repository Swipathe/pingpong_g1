# HITTER Complete Planner CSV Export 最终分支复审（三）

## Verdict

**APPROVED — 建议集成。**

复审范围：

```text
base = ebc14fb7959f8314ea7122713702f3f5401d3076
head = 26f31c7fffe72913564ac4482bdd74ce91b4edee
```

本轮未发现新的 Critical、Important 或 Minor finding。前次复审的两个
Important 和一个 Minor 均已关闭；随后针对 transition snapshot、late
transition、producer queue 丢事件、task observation close-to-freeze 竞态及
event/time 反序发现的边界也已补齐实现、测试、文档和正式输出证据。

## Findings

### Critical

无。

### Important

无。

### Minor

无。

## 前次 Findings 关闭情况

### 1. 有界 4096 条 attempt detail 导致辅助表静默截断

**已关闭。**

- `deploy/diagnostics/export_hitter_task_csv.py:1375-1612` 现在完整扫描
  `events.jsonl`，记录 selected attempts 的全部 transition、其真实 event
  id，以及全部 lifecycle ticks；同时全局拒绝
  `RECORDER_EVENT_GAP` 和 `DIAGNOSTIC_EVENT_DROPPED`。
- `deploy/diagnostics/export_hitter_task_csv.py:1703-1913` 将 detail timeline
  解释为持久化 snapshot：
  - 少于 4096 行时必须是 full timeline 的同长度 prefix；
  - 等于 4096 行时必须唯一匹配某个 complete-event prefix 的 bounded
    suffix；
  - transition 和 segment 使用 canonical JSON 比较，`true` 不再与 `1`
    混同；
  - `raw_lcm.csv` / `policy_ticks.csv` 的区间由 full timeline 到 snapshot
    cutoff 的完整 prefix 重建。
- `deploy/diagnostics/export_hitter_task_csv.py:2699-2764` 的
  `stage_timeline.csv` 使用 full event timeline，因此 snapshot 后合法落盘的
  late transition 仍被完整导出，但不会反向扩大 raw/policy 区间。
- `deploy/tests/test_hitter_task_csv_export.py:2132-2459` 覆盖 producer drop
  marker、late transition、JSON 类型敏感比较和超过 4096 transitions 的
  恢复。

最终 HEAD 的独立造数结果：

```text
DROP_PREFIX_REJECTED True old ['sentinel']
LATE_CUTOFF_OK PLANNER_SUCCEEDED [('2', '2.1')] True
```

第一行复现了旧 fail-open 的精确形状：早期 transition 已丢失、detail 的
4096 行 window 仍可匹配，但事件日志含 producer 持久化的 drop marker。
当前实现明确拒绝，且旧输出 sentinel 保持不变。第二行证明 late stage
进入 Attempt 1 的完整 stage CSV，而同一时刻 policy tick 的区间 owner
仍是 Attempt 2。

### 2. warnings-as-errors 会在提交后制造假失败

**已关闭。**

- `deploy/diagnostics/export_hitter_task_csv.py:3006-3020` 的
  `_warn_after_commit()` 吞掉 warning hook/filter 自身抛出的异常，并以
  stderr 作为最后回退。
- `deploy/diagnostics/export_hitter_task_csv.py:3023-3189` 保持 install rename
  加 parent fsync 为提交点；提交后的 backup 删除或 cleanup fsync 问题只做
  best-effort 通知，不再从 API/CLI 逃逸。
- `deploy/tests/test_hitter_task_csv_export.py:3169-3216` 在
  `warnings.simplefilter("error")` 下验证函数仍成功返回，新输出已安装且旧
  backup 状态真实可见。

### 3. Planner identity 的 `int()` 宽松强制转换

**已关闭。**

- `deploy/diagnostics/export_hitter_task_csv.py:636-687` 对四字段 planner key
  统一使用严格 integer 校验，拒绝 bool、float、字符串、负 epoch /
  generation 和非当前 schema。
- event attempt identity、replacement identity、detail input identity 走同一
  规则。
- `deploy/diagnostics/export_hitter_task_csv.py:546-560`、
  `deploy/diagnostics/export_hitter_task_csv.py:934-938` 还把 input/result
  `source_frame` 收紧为非负 integer，并逐调用精确比较。
- 对应 bool/string/fractional/negative/schema/attempt/source-frame 回归均
  通过。

## 新增边界复核

### Task observation snapshot 与 close-to-freeze 竞态

通过。

- exporter 从 detail window 中唯一的 `ATTEMPT_CLOSED` 映射回真实
  transition event id（`deploy/diagnostics/export_hitter_task_csv.py:
  1831-1838`）。
- selected attempt 的全部 lifecycle ticks 都必须同时满足 event id 顺序和
  monotonic 时间方向；两者反向时 fail closed
  （`deploy/diagnostics/export_hitter_task_csv.py:1840-1873`）。
- detail 等于最后一个 pre-close observation 时采用该值；否则只能精确匹配
  close 后完整事件中真实存在的 observation，以覆盖 close signal 已生成、
  recorder 尚未冻结 detail 的合法竞态。更早的历史 observation 不能被选中，
  null detail 也不能掩盖 pre-close 非空 observation
  （`deploy/diagnostics/export_hitter_task_csv.py:1874-1911`）。
- scope、wall time、payload 字段和 observation 维度在区间筛选之前验证，
  因此区间外 selected tick 不能绕过校验。

最终 HEAD 的独立造数结果：

```text
NULL_TICK_ORDER_REJECTED True True
CLOSE_TO_FREEZE_ACCEPTED 99.0 99.0 9
```

即使 lifecycle tick 的 task observation 为空，event/time 反序也会被拒绝；
符合 producer FIFO 的 close-to-freeze observation 则可恢复为冻结值。

对应定向测试包括：

- final pre-close 值导出；
- detail 不得选择更早历史；
- null detail 对 pre-close 值 fail closed；
- post-freeze tick 不重写 snapshot；
- close-to-freeze tick 可成为 snapshot；
- event id / monotonic 双边界反序拒绝；
- selected tick 必须为 attempt scope。

文档 `docs/hitter_complete_planner_csv_export.md:90-112` 与上述实际规则一致，
并明确披露当前 detail schema 没有 capture event-id watermark。

## Fresh verification

所有验证均在 clean `26f31c7fffe72913564ac4482bdd74ce91b4edee`
上重新执行，未沿用中间候选结果：

```text
20 项关键边界定向测试                         PASS
export + replay + recording                  Ran 115 tests; OK
attempt/model regression                     Ran 20 tests; OK
Python 3.8 syntax compile                    PASS
git diff --check ebc14fb..HEAD               PASS
git diff --check                             PASS
implementation worktree before this report   clean
```

base..HEAD 仅新增：

```text
deploy/diagnostics/export_hitter_task_csv.py
deploy/tests/test_hitter_task_csv_export.py
docs/hitter_complete_planner_csv_export.md
```

未修改实时 estimator、planner、policy、AttemptTracker 上限或 replay capture
上限。

## 真实 session 与正式输出

真实源 session：

```text
/home/loco1/BOB/Hitter/RobotBridge2/recordings/hitter_task_diagnostics/
20260730_194528_402494-p740921-f85e25f0
```

正式输出：

```text
/home/loco1/BOB/Hitter/RobotBridge2/recordings/hitter_task_diagnostics/
20260730_122056_381218-p994970-d50bc89f/
analysis_attempts_1_2_20260730_194528
```

正式 manifest 的 `exported_at_utc` 为
`2026-07-31T06:39:30.036768+00:00`，已由最终 HEAD 覆盖，不再是旧的
suffix-check manifest。

真实 session fresh 临时导出与正式输出共同确认：

```text
submitted                    2625
pending-replaced              540
actual completed             2085

summary.csv                      2
raw_lcm.csv                  33894
planner_inputs.csv            2085
planner_calls.csv             2085
planner_results.csv           2085
stage_timeline.csv            2102
policy_ticks.csv              1578
task_observations.csv            2
```

额外证据：

- 27 项 completeness checks 全部为 `true`；包含
  `no_diagnostic_event_drop`、
  `attempt_detail_timeline_prefix_window_verified`、
  `task_observation_detail_found_in_full_events` 和
  `null_task_observation_preclose_events_absent`，旧 suffix/task check 名称均
  已消失。
- 三张核心表均为 2085 个同序且唯一的四字段 key；Attempt 1/2 分别为
  `837 / 1248`，`(2, 1, 1, 11192)` 存在。
- 完整 transition 数为 `845 / 1257`，与两个 detail snapshot cutoff 和
  detail 长度逐一相等。
- selected lifecycle tick 数为 `1790 / 17066`；其中带 task observation 的
  数量为 `0 / 26`。正式 task CSV 的 Attempt 1 为空值，Attempt 2 的
  `pre_0=-0.5580859780311584`、`post_10=0.00787369254976511`、
  `clip_count=0`。
- 正式 manifest 的所有源 SHA-256 和 stat 与当前源文件一致；重读八张 CSV
  后的行数和 core-key SHA-256 与 manifest 一致。
- 再次 fresh 临时导出后，八张正式 CSV 逐文件 SHA-256 全部相同；除
  `exported_at_utc` 外，两份 manifest 完全一致。
- 正式目录恰有八张 CSV 和一个 manifest；父目录无 temp、backup 或
  failed-new 残留。

## 已知且已披露的非阻塞限制

以下限制不由本轮升级为 finding：

1. 路径式 component 检查和 rename 仍有并发 TOCTOU；完全关闭需要锁定的
   directory fd / `renameat`。
2. source 多次路径读取与最终 fingerprint 之间仍有窄竞态窗。
3. `input_seq` 连续性不能识别 raw 文件尾部整体截断；需要 recorder raw row
   count 或末端 sequence watermark。
4. 非空目录覆盖是可恢复的两次 rename 事务，不是单次原子交换。
5. Attempt detail 尚未持久化 capture event-id watermark；因此现有 schema
   只能要求 fallback 值精确存在于 close 后事件中，不能证明更窄的逐事件
   freeze 点。该限制已在运行文档中明确说明。
