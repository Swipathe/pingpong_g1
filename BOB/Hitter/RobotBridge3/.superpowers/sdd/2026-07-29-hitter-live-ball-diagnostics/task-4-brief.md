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

