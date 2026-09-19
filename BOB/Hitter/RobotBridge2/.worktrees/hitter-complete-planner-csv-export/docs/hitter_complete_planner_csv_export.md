# HITTER 完整 Planner CSV 离线导出

## 适用场景

实时诊断网页和 replay capture 为保护真机进程内存，允许保留有界数据，当前上限为 8192 条。它们适合在线观察，但不能作为长 attempt 的完整分析数据源。

需要完整分析时，必须使用本离线导出器。导出器只读取 session 磁盘事实源：

- `session.json`
- `events.jsonl`
- `ball_samples.csv`
- `attempt_details/<attempt_id>.json`

它不会读取 `ReplayCaptureStore` 或 replay bundle，也不会修改 estimator、planner、policy、控制参数和 recordings 源文件。

## 命令

在 `deploy` 目录执行：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy

PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python \
  -m diagnostics.export_hitter_task_csv \
  --session ../recordings/hitter_task_diagnostics/20260730_194528_402494-p740921-f85e25f0 \
  --attempts 1 2 \
  --output ../recordings/hitter_task_diagnostics/20260730_122056_381218-p994970-d50bc89f/analysis_attempts_1_2_20260730_194528 \
  --overwrite
```

四个参数的含义：

- `--session`：完整源 session 目录。
- `--attempts`：一个或多个正整数 attempt id。
- `--output`：导出目录。使用绝对规范化路径；它不能与源 session 重合，也就是不能等于、包含源 session 或位于源 session 内部。路径中任何已经存在的分量都不能是符号链接。
- `--overwrite`：允许事务式替换已有输出目录；省略时，已有目标会直接报错。

命令成功返回 0，并在标准输出打印 manifest JSON；提交点之前的完整性、读取、写入或替换错误均返回非零。新目录 rename 并完成父目录 fsync 后即视为已经提交；此后旧 backup 清理失败只发出 warning、保留残留 backup，不会把已成功的提交谎报为失败。即使调用方把 warning 配置成 exception，导出器也会吞掉该异常并向标准错误回退输出诊断，不会让 commit 后清理问题从 API 抛出。

## 完成调用的定义

`planner_inputs.csv`、`planner_calls.csv` 和 `planner_results.csv` 只包含 actual completed calls。每个 key 必须同时存在：

1. `planner_submit`
2. `planner_trace/start`
3. `planner_trace/complete` 及 result
4. attempt detail 中的 planner input

三表统一使用以下复合 key，并按 complete 时间排序：

```text
attempt_id
+ snapshot_key.schema_version
+ snapshot_key.track_epoch
+ snapshot_key.generation
```

四个 key 分量均严格要求 JSON/Python integer，拒绝 bool、float 和字符串：`attempt_id >= 1`，`schema_version` 必须等于当前导出 schema 1，`track_epoch >= 0`，`generation >= 0`。event、跨 attempt replacement、attempt detail input 和 result 都走同一校验，不做 `int()` 截断或字符串转换。

Planner input 与 result 的 `source_frame` 同样必须是非负 integer，并且逐调用完全相等；不会把 float 或字符串通过 `int()` 截断后再比较。

三表关闭后会被重新打开，逐行验证 header、行数、key 顺序和唯一性。仅发生 `pending_replaced`、从未 start/complete 的输入不属于完成调用，不会进入这三张表；其数量会单独记录在 summary 和 manifest 中。

## 输出文件

导出目录包含八张 CSV 和一个 manifest：

- `summary.csv`：每个 attempt 一行，保留 detail 原有的 `recording_complete` 历史值，并增加 `completed_call_export_complete`、`completed_call_count`、`pending_replaced_count`。
- `raw_lcm.csv`：从完整 `ball_samples.csv` 按 segment 闭区间选出的所有源行，不使用固定 subject 白名单。源 `attempt_id`、`track_segment_id` 和任意 subject 原样保留，`interval_attempt_id` 和 `interval_track_segment_id` 表示离线区间归属。
- `planner_inputs.csv`：actual completed calls 对应的完整输入。
- `planner_calls.csv`：submit/start/complete 时间、耗时及 replacement 元数据。
- `planner_results.csv`：真实 command、reason code 或错误结果。
- `stage_timeline.csv`：从完整 `events.jsonl` 流式恢复所选 attempt 的全部 `attempt_transition`，严格按 event 顺序展开并增加 `stage_index`；不会按时间重新排序。它不再受 attempt tracker `maxlen=4096` 的 detail 上限影响。
- `policy_ticks.csv`：从完整 `events.jsonl` 流式提取的 `lifecycle_tick`，使用真实 `event_id`。
- `task_observations.csv`：以持久化 attempt detail 为冻结快照，并用完整 `events.jsonl` 中的 `lifecycle_tick` 双源核对后恢复 11 维 pre/post clip 和 clip count；关闭后的新 tick 不会覆盖旧 attempt 的快照，没有 observation 的 attempt 仍保留空值行。
- `export_manifest.json`：源 session 绝对路径、attempt ids、所有源文件 SHA-256/stat、各表行数、核心 key 序列 SHA-256、submitted/pending/completed 计数及明确的 completeness checks。

所有向量和映射均使用紧凑 JSON，不使用 Python `repr`。

## 完整性与输出安全

导出器采用 fail-closed 语义。源 session 必须同时满足：

- `status=COMPLETE`
- `recording_complete=true`
- `recorder_healthy=true`
- `last_error=null`、`failure=null`
- `input_samples_dropped=0`
- persisted/observed event watermark 相等
- `events.jsonl` 的 event id 连续、末端与 watermark 一致，且没有 `RECORDER_EVENT_GAP` 或 `DIAGNOSTIC_EVENT_DROPPED`；后者表示 producer/publisher queue 已经丢失无法恢复的事件，因此即使 session metadata 仍显示 complete 也必须拒绝导出
- `ball_samples.csv` 全文件的 `input_seq` 是非负十进制整数；首值可任意，之后每行必须严格 `+1`。该校验先于 attempt 区间和 subject 筛选，因此缺号、重复、倒序和负值都会拒绝整个导出。

每个所选 attempt 的 `attempt_transition` payload 会严格规范化成 detail stage row。导出器按完整 event 顺序重建 full timeline；`stage_timeline.csv` 始终写出其中全部 transition，包括 detail 持久化后才落盘的晚到事件。

Attempt detail 仍作为冻结边界的交叉校验源。少于 4096 行时，detail timeline 必须精确等于 full timeline 的同长度 prefix；达到在线 deque 的 4096 行上限时，它必须精确等于某个 complete-event prefix 的 4096 行 bounded suffix，而且匹配边界必须唯一。JSON 采用规范序列化比较，因此 `true` 与 `1` 不会被 Python 相等规则误判为同值。detail 中的 `segments` 还必须能由这个 detail window 按在线 monitor 的同一算法精确重建。

`raw_lcm.csv` 和 `policy_ticks.csv` 的 segment 区间由 full timeline 从开头到上述冻结边界的完整 prefix 重建，而不是使用 detail 的 bounded window，也不会被冻结边界后的晚到 transition 反向延长。任一 timeline/segment 不一致、边界歧义或不同 attempt 区间重叠，都会在替换输出前 fail closed。manifest 对应记录 `full_attempt_transitions_from_events`、`attempt_detail_timeline_prefix_window_verified` 和 `segments_rebuilt_from_full_transitions`。

Task observation 也执行双源核对。所有所选 attempt 的 lifecycle tick 在做区间筛选之前就必须满足 `scope=attempt`、非负 integer `wall_time_us`、精确 payload 字段集合；非空 observation 必须同时具备有限的 11 维 `task_pre_clip`、`task_post_clip` 和非负 integer `clip_count`。导出器用 detail 中唯一 `ATTEMPT_CLOSED` 对应的真实 event id 划分关闭前后，并拒绝 event 顺序与 monotonic 时间方向相反的 malformed log。detail 若等于关闭边界之前 event 顺序中的最后一个 observation，就采用该值；否则它只能匹配关闭边界之后完整事件中真实出现的 observation，以兼容 close signal 已产生、但 recorder 尚未冻结 detail 的并发窗口，不能回退选择更早的历史值。detail 为空时，关闭边界之前不得存在非空 observation。CSV 写入经过该规则匹配的完整事件值，manifest 记录 `task_observations_from_full_events`、`task_observation_detail_found_in_full_events` 和 `null_task_observation_preclose_events_absent`。

当前 attempt detail 没有记录“捕获时对应的 event_id watermark”，因此无法把 close 后并发窗口精确收窄到某一个 event id；上述退化规则是现有 schema 下的 fail-closed 下界。若将来要证明严格的逐事件冻结点，应在 detail schema 中持久化 capture event id/watermark。

快照 prefix 重建出的 segment 时间必须有限且最终 `first_monotonic_s <= last_monotonic_s`；重叠或相接的同 attempt 区间会合并，不同 attempt 的区间重叠会拒绝。位于所选区间内、但声明为另一个所选 attempt 的 lifecycle tick 会被视为身份冲突并拒绝；不属于所选 attempts 的事件仍会忽略。

所有文件先写入目标同级临时目录并完成重读验证。manifest 写完且临时目录 fsync 后，导出器会在最终 replace 之前立即重新 stat 并计算所有源文件的 SHA-256；检测到变化会删除临时输出并保留旧输出。覆盖已有输出时，旧目录先移动到同级唯一 backup，再安装完整临时目录：

- 安装 rename 失败会恢复旧目录；若恢复后的父目录 fsync 失败，错误会准确说明旧目录已回到 output、backup 已被消费。
- 安装 rename 成功但父目录 fsync 失败时，新目录会移到同级 `failed-new`，旧目录恢复到 output 并再次 fsync；调用仍失败，并报告两个目录的真实位置。
- 没有旧输出时，安装后的父目录 fsync 失败会把新目录移回临时路径，交给外层清理。
- 安装 rename 与父目录 fsync 均成功是提交点。之后 backup 删除或清理 fsync 失败只产生 warning；新 output 仍视为成功，可能保留可人工清理的同级 backup。

路径防护会在创建输出父目录前后逐级拒绝所有已经存在的 symlink 分量，并在目录替换前再次检查。不过当前实现仍使用路径式检查和 rename，未使用锁定的 directory fd / `renameat`；攻击者或并发进程若恰好在最后一次检查与 rename 之间替换路径分量，仍存在路径级 TOCTOU。首尾 `input_seq` 连续性也无法单独识别文件尾部被整体截断；若要关闭该风险，录制协议还需持久化 raw row count 或末端 `input_seq` watermark。

## 当前 Attempt 1/2 的完整结果

源 session `20260730_194528_402494-p740921-f85e25f0` 的预期结果为：

| 指标 | 行数 |
|---|---:|
| submitted | 2625 |
| pending-replaced | 540 |
| actual completed | 2085 |
| `planner_inputs.csv` | 2085 |
| `planner_calls.csv` | 2085 |
| `planner_results.csv` | 2085 |
| `raw_lcm.csv` | 33894 |
| `stage_timeline.csv` | 2102 |
| `policy_ticks.csv` | 1578 |
| `task_observations.csv` | 2 |

三张核心表必须具有同一个 2085-key 序列；Attempt 2 的 generation 11192 应存在于导出结果中。
