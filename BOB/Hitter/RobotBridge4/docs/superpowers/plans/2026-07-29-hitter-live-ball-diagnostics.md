# HITTER Live Ball Diagnostics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让诊断网页只展示后端真实存在且时间语义明确的数据；球的 estimator 和诊断 incoming 计数不依赖 pelvis，同时明确展示 pelvis 缺失会阻塞生产 planner 和 task observation。

**Architecture:** 保持真机生产链路不变，在 diagnostics 内建立一份不可变的 v2 状态快照。底层 reducer、磁盘记录、API 和前端统一传递同一组结构化字段；球诊断新增独立于 pelvis 的只读 incoming candidate，生产 gate 仍忠实反映真实 planner 条件。

**Tech Stack:** Python 3.8、`dataclasses`、现有 diagnostics EventHub/HTTP server、原生 HTML/CSS/JavaScript、`unittest`。

## Global Constraints

- 不修改 `deploy/envs/hitter.py`，不改变真实 policy、planner、R2 或 action 行为。
- 不改变生产 incoming confirmation 的判定顺序和阈值。
- 不把未知值显示成 `0`、`0.000 s`、`PASS` 或 `COMPLETE`。
- 球诊断可以在 pelvis 缺失时继续；生产 planner/task observation 必须显示真实阻塞原因。
- 每个诊断 tick 只发布一份自洽快照，网页不从 stage 文本反推状态。
- 兼容已有 schema v1 记录；缺失字段映射为 `UNKNOWN/NOT_AVAILABLE`，不能补造数值。
- 只提交本计划涉及的明确文件，保留当前混合 worktree 中所有无关改动。

---

## Task 1: 修复 AttemptSummary 字段在 reducer 中丢失

**Files:**

- Modify: `deploy/diagnostics/hitter_task_models.py`
- Modify: `deploy/diagnostics/hitter_task_events.py`
- Test: `deploy/tests/test_hitter_task_events.py`

- [ ] **Step 1: 写 reducer 回归测试**

新增测试，构造含真实计数的 `ATTEMPT_CURRENT` 和 `ATTEMPT_CLOSED`：

```python
payload = {
    "attempt_id": 7,
    "estimator_sample_count": 19,
    "estimator_window_size": 31,
    "incoming_count": 2,
    "incoming_required_count": 3,
}
```

断言 `/api/state` 对应模型中的四个字段仍为 `19/31`、`2/3`。再构造不含这些字段的 v1 payload，断言结果为未知值，而不是 `0/31`、`0/3`。

- [ ] **Step 2: 运行测试确认失败**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
/home/loco1/miniconda3/envs/rb/bin/python -m unittest \
  tests.test_hitter_task_events
```

预期：真实字段被 `_attempt()` 丢失，legacy payload 被默认成假零。

- [ ] **Step 3: 把计数字段改为可空并完整透传**

`AttemptSummary` 的诊断进度字段使用 `Optional[int]`：

```python
estimator_sample_count: Optional[int] = None
estimator_window_size: Optional[int] = None
incoming_count: Optional[int] = None
incoming_required_count: Optional[int] = None
```

`HitterTaskEventReducer._attempt()` 显式读取和保留四个字段。字段缺失时保持 `None`。

- [ ] **Step 4: 运行 reducer 测试**

运行 Step 2 命令，预期全部通过。

- [ ] **Step 5: 提交**

```bash
git add deploy/diagnostics/hitter_task_models.py \
  deploy/diagnostics/hitter_task_events.py \
  deploy/tests/test_hitter_task_events.py
git commit -m "fix: preserve HITTER diagnostic progress fields"
```

---

## Task 2: 让磁盘记录和 API 往返保持真实字段

**Files:**

- Modify: `deploy/diagnostics/hitter_task_recording.py`
- Modify: `deploy/diagnostics/hitter_task_web.py`
- Test: `deploy/tests/test_hitter_task_recording.py`
- Test: `deploy/tests/test_hitter_task_web.py`

- [ ] **Step 1: 写 repository/API 往返失败测试**

覆盖以下场景：

- 写入后重新打开 session，四个计数字段不丢失。
- CSV 表头和每行都包含四个字段，未知值输出为空。
- `/api/attempts` 与 `/api/attempts/<id>` 返回一致的计数。
- legacy JSON 缺失字段时 API 返回 `null`。

- [ ] **Step 2: 运行测试确认失败**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
/home/loco1/miniconda3/envs/rb/bin/python -m unittest \
  tests.test_hitter_task_recording \
  tests.test_hitter_task_web
```

- [ ] **Step 3: 修复 JSON/CSV/API 序列化**

在 `_summary_from_json()` 中显式读取四个可空字段；CSV 增加：

```text
estimator_sample_count
estimator_window_size
incoming_count
incoming_required_count
```

HTTP 层直接序列化 reducer/repository 的值，不使用 `or 0` 一类回退。

- [ ] **Step 4: 运行 Task 2 测试并提交**

```bash
git add deploy/diagnostics/hitter_task_recording.py \
  deploy/diagnostics/hitter_task_web.py \
  deploy/tests/test_hitter_task_recording.py \
  deploy/tests/test_hitter_task_web.py
git commit -m "fix: persist truthful HITTER diagnostic counters"
```

---

## Task 3: 增加独立于 pelvis 的球诊断状态

**Files:**

- Modify: `deploy/diagnostics/hitter_task_models.py`
- Modify: `deploy/diagnostics/hitter_task_pipeline.py`
- Test: `deploy/tests/test_hitter_task_pipeline.py`

- [ ] **Step 1: 为结构化球状态写失败测试**

覆盖：

- 没有 pelvis 时 estimator 仍从 `1` 累加到 window size。
- 满足 `vx <= incoming_speed_threshold` 的唯一球样本使诊断 incoming 从 `1/3` 到 `3/3`。
- 重复处理同一个 `(track_epoch, generation)` 不重复计数。
- 非 incoming 样本重置连续计数。
- 球不可见或 track 结束时状态为明确原因，不伪造零。
- 生产 `incoming_count` 不被诊断计数修改。

- [ ] **Step 2: 运行 pipeline 测试确认失败**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
/home/loco1/miniconda3/envs/rb/bin/python -m unittest \
  tests.test_hitter_task_pipeline
```

- [ ] **Step 3: 定义 v2 结构化状态**

在 models 中增加：

```python
@dataclass(frozen=True)
class BallDiagnosticState:
    status: str
    estimator_sample_count: Optional[int]
    estimator_window_size: Optional[int]
    speed_mps: Optional[float]
    velocity_world_mps: Optional[Tuple[float, float, float]]
    incoming_count: Optional[int]
    incoming_required_count: Optional[int]
    incoming_status: str
    blocker: Optional[str]
```

`status`/`incoming_status` 使用显式枚举字符串，例如 `NOT_SEEN`、`ESTIMATING`、`READY`、`INCOMING`、`NOT_INCOMING`、`TRACK_ENDED`。

- [ ] **Step 4: 实现 BallDiagnosticTracker**

Tracker 只读取球 estimator snapshot；以 `(track_epoch, generation)` 去重，并使用现有 planner 配置的 incoming 速度阈值与 required count。它不读取 pelvis，不调用生产 planner，不产生 action。

- [ ] **Step 5: 运行 Task 3 测试并提交**

```bash
git add deploy/diagnostics/hitter_task_models.py \
  deploy/diagnostics/hitter_task_pipeline.py \
  deploy/tests/test_hitter_task_pipeline.py
git commit -m "feat: add pelvis-independent ball diagnostics"
```

---

## Task 4: 发布自洽的实时状态与生产 gate

**Files:**

- Modify: `deploy/diagnostics/hitter_task_models.py`
- Modify: `deploy/diagnostics/hitter_task_monitor.py`
- Modify: `deploy/diagnostics/hitter_task_events.py`
- Test: `deploy/tests/test_hitter_task_monitor.py`
- Test: `deploy/tests/test_hitter_task_events.py`

- [ ] **Step 1: 写 monitor 到 EventHub 的跨层失败测试**

模拟球输入但不提供 pelvis，断言同一个 state revision 中：

- `ball.status` 为 `ESTIMATING` 或 `READY`，计数为真实值。
- `ball.diagnostic_incoming` 可以到 `3/3`。
- `production_gate.pelvis` 为 `BLOCKED`，原因 `PELVIS_UNAVAILABLE`。
- `production_gate.planner` 为 `BLOCKED`。
- `production_gate.task_observation` 为 `NOT_AVAILABLE`。
- attempt summary 与顶层实时 ball state 数值一致。

- [ ] **Step 2: 写 subject health/TTS 语义测试**

区分 `NEVER_SEEN`、`LIVE`、`STALE`、`INVALID`；未计算出的 predicted/planner TTS 必须为 `None`。配置常量 `arm_tts_s` 标记为 threshold，不作为实时 TTS。

- [ ] **Step 3: 运行测试确认失败**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
/home/loco1/miniconda3/envs/rb/bin/python -m unittest \
  tests.test_hitter_task_monitor \
  tests.test_hitter_task_events
```

- [ ] **Step 4: 构造不可变 v2 快照**

Monitor 每 tick 在锁内读取一次输入并生成：

```python
{
    "schema_version": 2,
    "revision": revision,
    "captured_monotonic_s": now,
    "subjects": {...},
    "ball": {...},
    "production_gate": {...},
    "current_attempt": {...},
}
```

同一 event payload 中所有字段来自这份快照，不再由前端或 reducer 猜测。

- [ ] **Step 5: 保留 v1 兼容**

Reducer 接受 v1/v2；v1 缺失结构化字段时保留 unknown。不能拒绝已有记录，也不能把缺失字段改成零。

- [ ] **Step 6: 运行 Task 4 测试并提交**

```bash
git add deploy/diagnostics/hitter_task_models.py \
  deploy/diagnostics/hitter_task_monitor.py \
  deploy/diagnostics/hitter_task_events.py \
  deploy/tests/test_hitter_task_monitor.py \
  deploy/tests/test_hitter_task_events.py
git commit -m "feat: publish coherent HITTER diagnostic snapshots"
```

---

## Task 5: 网页只渲染结构化真实值

**Files:**

- Modify: `deploy/diagnostics/static/hitter_task_monitor.html`
- Test: `deploy/tests/test_hitter_task_frontend.py`

- [ ] **Step 1: 写 frontend 失败测试**

覆盖页面源代码与 JS 渲染 helper：

- `null` 显示 `—`，不能显示 `0/31`、`0/3`、`0.000 s`。
- estimator 使用 `state.ball.estimator_sample_count/window_size`。
- 球诊断 incoming 使用 `state.ball.incoming_count/required_count`。
- 生产 incoming/planner/task obs 使用 `production_gate`，不能与球诊断计数混为一项。
- pelvis 缺失时球卡仍更新，生产卡显示 `PELVIS_UNAVAILABLE`。
- history 计数来自记录值；翻页游标不在每次刷新时重置。
- 不再通过 stage 子串决定颜色和进度。

- [ ] **Step 2: 运行测试确认失败**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
/home/loco1/miniconda3/envs/rb/bin/python -m unittest \
  tests.test_hitter_task_frontend
```

- [ ] **Step 3: 重写实时卡片绑定**

把当前来球拆成：

1. 球检测/新鲜度；
2. estimator；
3. 球诊断 incoming；
4. 生产 incoming；
5. planner gate；
6. task observation。

统一格式化 helper：

```javascript
const formatOptional = (value, formatter) =>
  value === null || value === undefined ? "—" : formatter(value);
```

TTS 只显示真实计算值；`arm_tts_s` 政名为 `ARM threshold`。

- [ ] **Step 4: 运行 Task 5 测试并提交**

```bash
git add deploy/diagnostics/static/hitter_task_monitor.html \
  deploy/tests/test_hitter_task_frontend.py
git commit -m "fix: render truthful HITTER live diagnostics"
```

---

## Task 6: 文档、全量回归与现场验收准备

**Files:**

- Modify: `docs/hitter_task_observation_diagnostics.md`
- Test: all `deploy/tests/test_hitter_task_*.py`

- [ ] **Step 1: 更新运行说明**

写明：

- 球诊断不需要 pelvis。
- pelvis 缺失会阻塞生产 planner/task observation，这是预期且必须显式显示。
- 各状态字段的真实含义。
- 网页重启命令与不运行机器人的球链路检查命令。

- [ ] **Step 2: 运行全部 diagnostics 测试**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
/home/loco1/miniconda3/envs/rb/bin/python -m unittest discover \
  -s tests -p 'test_hitter_task_*.py'
```

预期：全部通过；环境相关 skip 保持明确。

- [ ] **Step 3: 做静态与范围检查**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2
git diff --check
git diff -- deploy/envs/hitter.py
git status --short
```

确认本任务没有触碰生产 `deploy/envs/hitter.py`；该文件原有用户改动保持不变。

- [ ] **Step 4: 本地提交**

```bash
git add docs/hitter_task_observation_diagnostics.md
git commit -m "docs: explain truthful HITTER ball diagnostics"
```

- [ ] **Step 5: 最终验收输出**

交付内容必须包含：

- 修复的根因。
- 单测总数与结果。
- pelvis 缺失时页面应看到的精确状态。
- 需要用户重启 monitor 才能加载新代码的命令。
- 明确说明没有修改生产 policy/planner/action 链路。
