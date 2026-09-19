# HITTER 任务监控实时计数显示修复实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 当前阶段推进到 incoming/planner 后，页面仍显示真实 estimator 样本计数，并将后端空的预测击球时间和 planner TTS 显示为 `—`。

**Architecture:** `AttemptSummary` 作为 `/api/state` 的稳定传输对象，显式携带 `estimator_sample_count` 和 `estimator_window_size`；monitor 在每次 current-attempt 投影时写入同一时刻的值。前端优先消费这些字段，不再依赖当前 stage 文本恢复 estimator 计数；数值格式化在强制转成 `Number` 前先处理 `null`/`undefined`。

**Tech Stack:** Python 3.8 dataclass、标准库 `unittest`、原生 JavaScript、Chrome headless DOM 测试。

## Global Constraints

- 诊断器保持只读，不向机器人或生产 planner 发布控制命令。
- API 变更只增加字段，保留现有 schema version 和旧 stage 文本兼容回退。
- 不修改 estimator、incoming confirmation、planner 或真机控制语义。
- 只 stage/commit 本计划明确列出的文件。

---

### Task 1: 在 current-attempt API 中投影真实 estimator 计数

**Files:**
- Modify: `deploy/diagnostics/hitter_task_models.py`
- Modify: `deploy/diagnostics/hitter_task_monitor.py`
- Test: `deploy/tests/test_hitter_task_monitor.py`

**Interfaces:**
- Consumes: `HitterTaskMonitor._latest_estimator_sample_count` 与 `HitterTaskMonitor.estimator_window_size`
- Produces: `AttemptSummary.estimator_sample_count: int` 与 `AttemptSummary.estimator_window_size: int`

- [x] **Step 1: 写失败测试**

在 `test_tick_reads_semantic_monotonic_twice_and_projects_stage` 中设置 monitor 当前计数为 `31`，断言 `ATTEMPT_CURRENT` JSON 即使 stage 已是 `INCOMING_CONFIRMING 2/3`，仍显式包含：

```python
self.assertEqual(current.payload["attempt"]["estimator_sample_count"], 31)
self.assertEqual(current.payload["attempt"]["estimator_window_size"], 31)
```

- [x] **Step 2: 确认测试因字段缺失而失败**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest \
  tests.test_hitter_task_monitor.HitterTaskMonitorTest.test_tick_reads_semantic_monotonic_twice_and_projects_stage
```

Expected: `KeyError: 'estimator_sample_count'`。

- [x] **Step 3: 写最小实现**

给 `AttemptSummary` 添加两个向后兼容的默认字段并写入 JSON；`_project_attempt()` 使用 monitor 当前计数与窗口大小覆盖它们。

- [x] **Step 4: 确认后端测试通过**

重复 Step 2 命令，Expected: `OK`。

### Task 2: 渲染真实计数并正确格式化空时间

**Files:**
- Modify: `deploy/diagnostics/static/hitter_task_monitor.html`
- Test: `deploy/tests/test_hitter_task_frontend.py`

**Interfaces:**
- Consumes: `current_attempt.estimator_sample_count`、`current_attempt.estimator_window_size`、nullable `predicted_strike_time_s`、nullable `planner_tts_s`
- Produces: 阶段推进后仍为 `31/31`；两个 nullable 时间为 `—`

- [x] **Step 1: 写失败的 headless 浏览器测试**

构造 stage 为 `INCOMING_CONFIRMED 3/3`、显式 estimator 计数为 `31/31`、两个时间为 `null` 的 current attempt，并断言实际 DOM 包含：

```text
31/31
预测击球 —
planner TTS —
```

同时断言 current-attempt 文本不包含 `预测击球 0.000 s` 或 `planner TTS 0.000 s`。

- [x] **Step 2: 确认测试因当前回退和 null 转零而失败**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest \
  tests.test_hitter_task_frontend.HitterTaskFrontendContractTest.test_headless_advanced_stage_keeps_estimator_count_and_null_tts_empty
```

Expected: DOM 仍包含 `0/31` 或 `0.000 s`。

- [x] **Step 3: 写最小前端实现**

`stageValue(..., "estimating")` 优先组合显式 count/window 字段，并保留旧 stage 正则作为兼容回退。`formatNumber()` 在调用 `Number(value)` 前对 `null`/`undefined` 返回 `—`。

- [x] **Step 4: 运行定向和完整回归**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest \
  tests.test_hitter_task_frontend \
  tests.test_hitter_task_monitor \
  tests.test_hitter_task_web \
  tests.test_hitter_task_events \
  tests.test_hitter_task_recording
```

Expected: 所有测试通过，零 failure、零 error。

- [x] **Step 5: 检查差异并仅提交相关文件**

```bash
git diff --check -- \
  deploy/diagnostics/hitter_task_models.py \
  deploy/diagnostics/hitter_task_monitor.py \
  deploy/diagnostics/static/hitter_task_monitor.html \
  deploy/tests/test_hitter_task_monitor.py \
  deploy/tests/test_hitter_task_frontend.py \
  docs/superpowers/plans/2026-07-27-hitter-task-monitor-live-counter-display.md
```
