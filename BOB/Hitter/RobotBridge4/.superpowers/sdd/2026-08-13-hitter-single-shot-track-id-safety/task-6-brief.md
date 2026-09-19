### Task 6: 将 planner 输出改为容量 64 的有序完成队列

**Files:**
- Modify: `deploy/utils/hitter_realtime.py: PlannerResultSnapshot, PlannerWorkerStats, FrozenPlannerResult, LatestOnlyPlannerWorker`
- Create: `deploy/tests/test_hitter_completed_result_queue.py`

**Interfaces:**
- Consumes: Task 5 的 `PlannerFailureReason` / `PlannerRejected`。
- Produces: `CompletedResultBatch(results, frozen_results, overflowed, overflow_count, overflowed_track_ids)`；`LatestOnlyPlannerWorker.drain_completed_results()`；latest-only pending 输入保持不变。

测试文件内定义：`snapshot(track_id,generation)` 返回 deadline 尚未到期且所有数组有限的 immutable `BallEstimateSnapshot`；`plan_ok(snapshot)` 返回 `PlannerResultSnapshot` 所需的完整有限 `HitterWbcCommand`，deadline 为 `snapshot.received_monotonic_s+1.0`；`plan_in_generation_order` 调用 `plan_ok` 并保留输入 generation；`wait_until(predicate, timeout_s=1.0)` 使用 `time.monotonic()` 的有界轮询并在超时 raise `AssertionError`；`complete_three_sequential_submissions(worker)` 每次 submit 后等待对应 completion 再提交下一条。这样 overflow 测试验证 completed queue，而不是误测 latest-only pending replacement。

- [ ] **Step 1: 写有序、清空、overflow 和 unknown exception 失败测试**

```python
def test_completed_results_are_drained_in_order():
    worker = LatestOnlyPlannerWorker(plan_in_generation_order)
    try:
        for generation in (1, 2, 3):
            worker.submit(snapshot(track_id=7, generation=generation))
            wait_until(lambda: worker.stats.completed + worker.stats.failed == generation)
        batch = worker.drain_completed_results()
        assert [r.source_generation for r in batch.results] == [1, 2, 3]
        assert worker.drain_completed_results().results == ()
    finally:
        assert worker.close(timeout_s=1.0)


def test_capacity_overflow_is_sticky_and_never_replaces_existing_results():
    worker = LatestOnlyPlannerWorker(plan_ok, completed_result_queue_capacity=2)
    complete_three_sequential_submissions(worker)
    batch = worker.drain_completed_results()
    assert [r.source_generation for r in batch.results] == [1, 2]
    assert batch.overflowed is True
    assert batch.overflow_count == 1
    assert batch.overflowed_track_ids == (7,)
    empty = worker.drain_completed_results()
    assert empty.overflowed is False
    assert empty.overflow_count == 0
    assert empty.overflowed_track_ids == ()
    assert worker.stats.completed_result_queue_overflow_total == 1
```

另一个测试用两个 `threading.Event`：planner 在 generation 1 内阻塞，连续 submit 2/3，释放后断言实际执行 generation 为 `[1,3]`、`pending_replaced_total == 1`，证明 pending input 深度仍最多 1 且只保留最新 snapshot；`finally` 必须释放 event 并 close worker。

- [ ] **Step 2: 运行测试，确认当前 `latest_result` 覆盖语义失败**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  deploy/tests/test_hitter_completed_result_queue.py
```

Expected: FAIL，缺少 `completed_result_queue_capacity` 或 `drain_completed_results`。

- [ ] **Step 3: 定义严格的 result 与 batch invariants**

```python
@dataclass(frozen=True)
class PlannerResultSnapshot:
    track_id: int
    source_generation: int
    source_frame: int
    strike_deadline_monotonic_s: float
    completed_monotonic_s: float
    command: object | None
    failure_reason: PlannerFailureReason | None = None
    error_text: str | None = None

    def __post_init__(self) -> None:
        succeeded = self.command is not None and self.failure_reason is None
        failed = self.command is None and self.failure_reason is not None
        if not (succeeded or failed):
            raise ValueError("planner result must be exactly success or failure")


@dataclass(frozen=True)
class CompletedResultBatch:
    results: tuple[PlannerResultSnapshot, ...]
    frozen_results: tuple[FrozenPlannerResult | None, ...]
    overflowed: bool
    overflow_count: int
    overflowed_track_ids: tuple[int, ...]

    def __post_init__(self) -> None:
        if len(self.results) != len(self.frozen_results):
            raise ValueError("results and frozen_results length mismatch")
        if type(self.overflow_count) is not int or self.overflow_count < 0:
            raise ValueError("overflow_count must be a non-negative integer")
        if bool(self.overflowed) != (self.overflow_count > 0):
            raise ValueError("overflowed must match this drain's overflow_count")
        if any(type(track_id) is not int or track_id <= 0
               for track_id in self.overflowed_track_ids):
            raise ValueError("overflowed_track_ids must contain positive integers")
        if self.overflowed_track_ids != tuple(sorted(set(self.overflowed_track_ids))):
            raise ValueError("overflowed_track_ids must be sorted and unique")
        if self.overflowed and not self.overflowed_track_ids:
            raise ValueError("overflow must identify at least one affected track")
        if self.overflow_count < len(self.overflowed_track_ids):
            raise ValueError("overflow count cannot be smaller than unique track ids")
```

- [ ] **Step 4: 在 worker 内 append，不覆盖 completed result**

```python
if len(self._completed_results) >= self._completed_result_queue_capacity:
    self._completed_result_queue_overflow_latched = True
    self._completed_result_queue_overflow_since_drain += 1
    self._completed_result_queue_overflow_total += 1
    self._completed_result_queue_overflow_track_ids_since_drain.add(
        planned.track_id
    )
else:
    self._completed_results.append((planned, frozen))
```

`drain_completed_results()` 在同一 condition lock 内，把 `_completed_result_queue_overflow_since_drain` 和排序后的 `_track_ids_since_drain` 复制进 batch，然后将这两个 since-drain 字段和 sticky bool 清零；`_completed_result_queue_overflow_total` 永不在 drain 时清零，并由 `stats.completed_result_queue_overflow_total` 暴露。禁止用累计 total 构造 batch，否则一次旧 overflow 会污染以后所有 drain。保留 `latest_result_bundle()` 仅供尚未迁移的只读诊断，Task 10 后 production/shadow 均不再依赖它。

- [ ] **Step 5: 对 typed/unknown exception 生成明确 reason**

```python
except PlannerRejected as exc:
    reason = exc.reason
    error_text = exc.detail
except Exception as exc:
    reason = PlannerFailureReason.INTERNAL_ERROR
    error_text = f"{type(exc).__name__}: {exc}"
```

worker 捕获异常后继续线程循环；`FrozenPlannerResult` 同步保存 `failure_reason`，诊断不得再解析 `error_text`。

- [ ] **Step 6: 运行 queue 测试与线程关闭回归**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  deploy/tests/test_hitter_completed_result_queue.py \
  deploy/tests/test_hitter_task_diagnostics_safety.py
```

Expected: PASS；unknown exception 后下一条仍完成；close 不遗留 worker thread。

- [ ] **Step 7: 提交 ordered output 单元**

```bash
git add deploy/utils/hitter_realtime.py \
  deploy/tests/test_hitter_completed_result_queue.py
git add -p -- deploy/tests/test_hitter_task_diagnostics_safety.py
git diff --cached --check
git diff --cached --name-status
git commit -m "feat: queue completed HITTER planner results"
```

---

