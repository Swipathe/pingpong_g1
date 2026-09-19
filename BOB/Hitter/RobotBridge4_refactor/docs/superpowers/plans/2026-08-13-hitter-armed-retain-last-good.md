# RobotBridge4 ARMED Retain Last-Good Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让 `HIT_HEIGHT_OUT_OF_RANGE` 和 `OVERRIDE_DISCONTINUITY` 在 commit 前只拒绝新结果并保留 last-good command，不再参与三次软失败撤销。

**Architecture:** 在 `HitterCommandLifecycle` 中把两类 reason 明确分为 retain-only。planner failure 路径提前返回 `retained_failure`；override 连续性失败路径删除计数和阈值 `cancel()`，直接返回 `retained_discontinuity`。其他软失败、硬故障、commit 和一球一拍逻辑不变。

**Tech Stack:** Python 3、现有 `HitterCommandLifecycle` 纯状态机、pytest 契约测试。

## Global Constraints

- 只修改 `/home/loco1/BOB/Hitter/RobotBridge4`，RobotBridge3 不修改。
- retain-only 结果不增加也不清零 `consecutive_failure_count`。
- 不删除通用 `cancel()`，不放宽任何硬故障撤销。
- 按用户要求本轮不执行测试命令；只同步测试契约并核对精确 diff。
- 保留工作树中所有既有未提交改动，暂存时只列出本任务文件。

---

### Task 1: 实现 retain-only 生命周期语义

**Files:**
- Modify: `deploy/utils/hitter_realtime.py:778-793,1323-1403`
- Modify: `deploy/tests/test_hitter_single_shot_lifecycle.py:451-477`

**Interfaces:**
- Consumes: `LifecycleCancelReason`、`HitterCommandLifecycle.ingest()`、`consecutive_failure_count` 和现有 `LifecycleDecision` kinds。
- Produces: commit 前高度越界返回 `retained_failure`；override 不连续返回 `retained_discontinuity`；两者均保持 active command 和现有计数。

- [ ] **Step 1: 同步 override 契约测试**

将 `test_third_override_discontinuity_cancels` 改为重复输入均保留：

```python
def test_override_discontinuity_retains_last_good_without_counting(change):
    item = armed_lifecycle_before_commit()
    active = item.active_result
    for generation in (2, 3, 4):
        decision = item.ingest(
            override_result(active, generation=generation, **change),
            now=1.1,
        )
        assert decision.kind == "retained_discontinuity"
        assert item.consecutive_failure_count == 0
        assert item.active_result is active
        assert item.phase is CommandPhase.ARMED
```

- [ ] **Step 2: 增加高度越界不改变既有计数的契约测试**

先制造一次仍应保留的其他软失败，再穿插多次高度越界：

```python
def test_hit_height_out_of_range_retains_last_good_without_counting():
    item = armed_lifecycle_before_commit()
    active = item.active_result
    assert item.ingest(
        failure(
            generation=2,
            reason=PlannerFailureReason.BALL_NOT_INCOMING,
        ),
        now=1.1,
    ).kind == "retained_failure"
    assert item.consecutive_failure_count == 1

    for generation in (3, 4, 5):
        decision = item.ingest(
            failure(
                generation=generation,
                reason=PlannerFailureReason.HIT_HEIGHT_OUT_OF_RANGE,
            ),
            now=1.1,
        )
        assert decision.kind == "retained_failure"
        assert item.consecutive_failure_count == 1
        assert item.active_result is active
        assert item.phase is CommandPhase.ARMED
```

- [ ] **Step 3: 写入最小状态机修改**

增加 retain-only 分类，在 `_handle_armed_failure()` 中先于计数逻辑返回，同时删除 override 分支中的计数和阈值撤销：

```python
_RETAIN_ONLY_FAILURES = frozenset(
    {
        LifecycleCancelReason.HIT_HEIGHT_OUT_OF_RANGE,
        LifecycleCancelReason.OVERRIDE_DISCONTINUITY,
    }
)

if reason in self._RETAIN_ONLY_FAILURES:
    active_id = (
        None if self.active_result is None else self.active_result.track_id
    )
    return self._decision(
        "retained_failure",
        track_id=active_id,
        cancel_reason=reason,
    )
```

override 不连续分支只保留：

```python
self.last_failure_reason = LifecycleCancelReason.OVERRIDE_DISCONTINUITY
return self._decision(
    "retained_discontinuity",
    track_id=result.track_id,
    cancel_reason=LifecycleCancelReason.OVERRIDE_DISCONTINUITY,
)
```

- [ ] **Step 4: 核对而不执行测试**

运行只读检查：

```bash
git diff --check -- deploy/utils/hitter_realtime.py deploy/tests/test_hitter_single_shot_lifecycle.py
git diff -- deploy/utils/hitter_realtime.py deploy/tests/test_hitter_single_shot_lifecycle.py
```

确认 diff 只改变两类 retain-only 行为，且没有运行 pytest。测试执行留待用户后续允许。

- [ ] **Step 5: 精确提交任务文件**

```bash
git add -- deploy/utils/hitter_realtime.py deploy/tests/test_hitter_single_shot_lifecycle.py
git commit -m "fix: retain armed hitter command on rejected overrides"
```
