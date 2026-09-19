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

