# HITTER 完整 Planner CSV 导出设计

## 背景

当前逐球分析目录混用了两套不同粒度的数据：

- `planner_inputs.csv` 来自 monitor 独立保存的 planner 输入快照；
- `planner_calls.csv` 和部分 replay 数据来自每个 attempt 的有界内存副本。

实时副本最多保存 8192 个 replay 事件。达到上限后，后续事件不再进入该副本，并将 attempt 标记为 `recording_complete=False`。这项限制用于保护真机监控进程的内存，不能取消；但离线导出不应使用这个有界副本作为完整数据源。

本次源 session 的完整磁盘记录仍然存在：

- `ball_samples.csv` 保存按到达顺序落盘的原始 LCM 样本；
- `events.jsonl` 保存完整的 planner submit、pending replacement、start、complete 和 lifecycle 事件；
- `attempt_details/<id>.json` 保存 planner 输入字段、阶段和 task observation。

Attempt 2 实际有 1575 次 planner submit，其中 327 次 pending 输入在开始执行前被新输入替换，1248 次真正开始并完成。旧派生文件只保留了其中 1069 次完成调用，额外缺失 179 次已完成调用。

## 目标

1. 从完整磁盘日志恢复所有实际开始并完成的 planner 调用。
2. 排除从未开始执行的 `pending_replaced` 输入。
3. 使 `planner_inputs.csv`、`planner_calls.csv`、`planner_results.csv` 具有完全相同的行数、snapshot key 和排序，允许逐行直接比较。
4. 从完整磁盘记录重新生成原始帧、阶段、policy tick 和 task observation CSV。
5. 对缺 key、重复 key、字段冲突或不完整磁盘日志直接报错，禁止静默丢行。
6. 提供可重复运行的命令，既能修复当前 Attempt 1/2，也能用于后续 session。

## 非目标

- 不修改 estimator、planner、policy 或真机控制逻辑。
- 不改变 360 Hz 原始输入、planner 提交节流和 latest-only worker 行为。
- 不取消实时 monitor 的 8192 条内存保护。
- 不把 `pending_replaced` 伪装成已完成 planner call。

## 方案比较

### 方案 1：从完整磁盘日志离线重建（采用）

完整扫描 `events.jsonl` 和 `ball_samples.csv`，结合 attempt detail 重建实际完成调用。该方案能恢复当前数据，不增加真机运行内存，并能严格验证完整性。

### 方案 2：提高实时副本上限

无法恢复已经缺失的派生数据，长时间 attempt 仍可能再次达到新上限，因此不采用。

### 方案 3：取消实时副本上限

会让实时监控内存随运行时间持续增长，影响真机稳定性，因此不采用。

## 数据来源与粒度

### 原始样本

`ball_samples.csv` 是原始 LCM 样本的磁盘事实源。导出器按 attempt 的单个或多个时间区间流式扫描该文件，输出区间内的 ball、pelvis 和 table 样本，不通过 replay input 副本。

### Planner 输入

`attempt_details/<id>.json` 中的 planner input 保存 snapshot 的位置、速度、base pose、可见性和 estimator ready 状态。导出器只选择实际完成调用对应的输入 key。

### Planner 调用与结果

`events.jsonl` 是 planner 事件的磁盘事实源。一次“实际完成调用”必须同时存在：

1. `planner_submit`；
2. 同一 snapshot key 的 `planner_trace/start`；
3. 同一 snapshot key 的 `planner_trace/complete`，且包含冻结后的 result。

只有 `pending_replaced` 而没有 start/complete 的 key 不进入完成调用表。

### 阶段、Policy Tick 和 Task Observation

- 阶段与最终 task observation 从 attempt detail 和完整事件相互核对；
- policy tick 从完整 `events.jsonl` 提取；
- 不使用 replay bundle 中受 8192 上限影响的 tick 或 stage 列表。

## 主键与排序

三张核心表统一使用以下复合主键：

```text
attempt_id
+ snapshot_key.schema_version
+ snapshot_key.track_epoch
+ snapshot_key.generation
```

每个表中的主键必须唯一。三张表均按：

```text
completed_monotonic_s
+ attempt_id
+ track_epoch
+ generation
```

排序。相同行号必须代表同一次实际完成调用。

`source_frame` 作为一致性校验字段，不代替 snapshot key。

## 输出文件

导出器覆盖指定分析目录中的以下文件：

- `summary.csv`
- `raw_lcm.csv`
- `planner_inputs.csv`
- `planner_calls.csv`
- `planner_results.csv`
- `stage_timeline.csv`
- `policy_ticks.csv`
- `task_observations.csv`
- `export_manifest.json`

其中：

- `planner_inputs.csv`：仅包含实际完成调用的输入；
- `planner_calls.csv`：每个实际完成调用的 submit/start/complete、耗时和替换元数据；
- `planner_results.csv`：与 calls 一一对应的成功 command 或真实错误；
- `export_manifest.json`：记录源 session、attempt 列表、源文件哈希、各表行数、完整性检查和导出时间。

`summary.csv` 同时保留原 attempt 的 `recording_complete` 历史值，并新增：

- `completed_call_export_complete`
- `completed_call_count`
- `pending_replaced_count`

这样不会把旧 replay 副本的截断历史改写成“从未发生”，同时可以明确说明本次离线恢复是否完整。

## 完整性规则

导出成功前必须满足：

1. 每个 complete key 都存在 submit、start、result 和 planner input；
2. submit、start、complete、input 中的 attempt、epoch、generation 和 source frame 一致；
3. 三张核心表 key 序列完全相同；
4. 三张核心表不存在重复 key；
5. `completed_count + pending_replaced_count == submitted_count`；
6. 源 session recorder 为 complete，且不存在 recorder gap；
7. 临时文件全部生成并验证后，才原子替换目标 CSV。

任一规则失败时，命令返回非零退出码，保留旧文件并输出明确错误，不生成“看似完整”的部分结果。

## 当前数据的预期结果

对源 session `20260730_194528_402494-p740921-f85e25f0`：

| Attempt | Planner submit | Pending replaced | 实际完成 |
|---|---:|---:|---:|
| 1 | 1050 | 213 | 837 |
| 2 | 1575 | 327 | 1248 |
| 合计 | 2625 | 540 | 2085 |

因此重新生成后：

- `planner_inputs.csv`：2085 行；
- `planner_calls.csv`：2085 行；
- `planner_results.csv`：2085 行；
- 三张表的 2085 个 key 和行序必须完全一致。

## 命令接口

提供可重复运行的模块命令：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy

PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python \
  -m diagnostics.export_hitter_task_csv \
  --session ../recordings/hitter_task_diagnostics/20260730_194528_402494-p740921-f85e25f0 \
  --attempts 1 2 \
  --output ../recordings/hitter_task_diagnostics/20260730_122056_381218-p994970-d50bc89f/analysis_attempts_1_2_20260730_194528 \
  --overwrite
```

默认采用严格模式；不提供允许静默缺行的选项。

## 测试

自动化测试至少覆盖：

1. 完整事件流生成逐 key 对齐的三张核心表；
2. pending-replaced 输入不进入完成表；
3. replay bundle 截断但完整磁盘事件存在时，仍恢复全部完成调用；
4. complete 缺少 input、start 或 result 时导出失败；
5. 重复 key 或 source frame 冲突时导出失败；
6. 原子写入失败时不破坏现有 CSV；
7. 当前真实 session 生成 837、1248、2085 的预期计数。

## 交付判定

只有同时满足以下条件才算完成：

- 当前 Attempt 1/2 的八张 CSV 和 manifest 已重新生成；
- 三张核心表各 2085 行且 key 序列完全相同；
- Attempt 2 的末端完成调用（包括 generation 11192）存在于新文件；
- `pending_replaced` key 不出现在完成调用表；
- 自动化测试和真实数据完整性检查全部通过；
- 未修改真机 planner、policy 或控制参数。
