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

