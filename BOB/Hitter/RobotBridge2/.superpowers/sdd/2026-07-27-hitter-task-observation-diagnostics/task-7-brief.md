## Task 7: 实现 100 Hz worker、50 Hz lifecycle 与 shadow task tick

**Files:**

- Modify: `deploy/utils/hitter_realtime.py`
- Modify: `deploy/diagnostics/hitter_task_pipeline.py`
- Create: `deploy/tests/test_hitter_task_pipeline.py`

### Worker trace extension

在不改变默认行为的前提下增加：

```python
@dataclass(frozen=True)
class FrozenPlannerResult:
    snapshot_key: SnapshotKey
    source_frame: int
    strike_deadline_monotonic_s: float
    completed_monotonic_s: float
    command_fields: Mapping[str, JsonValue] | None
    error_type: str | None
    error_text: str | None


@dataclass(frozen=True)
class PlannerWorkerTrace:
    kind: str
    monotonic_s: float
    snapshot: BallEstimateSnapshot
    result: FrozenPlannerResult | None
    replaced_snapshot: BallEstimateSnapshot | None
    replaced_result: FrozenPlannerResult | None
    stats: PlannerWorkerStats
```

`LatestOnlyPlannerWorker(..., trace_listener=None)` 仅在 listener 非空时复制并发送 `submit`、`pending_replaced`、`start`、`complete`、`latest_result_replaced`；command 必须在 worker 内转为深度不可变 `FrozenPlannerResult.command_fields`，不得把可变 command 对象交给诊断线程。listener 必须在 worker condition lock 外调用，listener 异常只能增加 trace-listener failure 计数，不能改变 planner result/stats。`close(timeout_s=None) -> bool` 保持生产默认等待行为；诊断传有限 timeout 并检查返回值。`IncomingTrackConfirmation` 新增只读 `snapshot()`，返回 epoch/count/confirmed，不提供新的 mutation API。

### Pipeline interface

```python
@dataclass(frozen=True)
class TaskTickResult:
    lifecycle_now_s: float
    obs_now_s: float
    phase: str
    lifecycle_decision: str
    active_binding: AttemptBinding | None
    cached_binding: AttemptBinding | None
    command_result: FrozenPlannerResult | None
    command_fields_used: Mapping[str, JsonValue] | None
    task_observation: TaskObservationResult | None
    task_pass: bool
    errors: tuple[str, ...]


class ShadowTaskPipeline:
    def __init__(
        self,
        *,
        adapter: MocapFrameAdapter,
        settings: HitterRuntimeSettings,
        event_sink: Callable[[EventDraft], None],
        raw_sink: Callable[[NormalizedMocapSample], RawSubmitResult],
    ) -> None:
        """Own incoming, planner worker, lifecycle and production-reset callback."""

    def ingest_adapter_output(self, output: AdapterOutput) -> None:
        """Raw record, attempt bind, 100 Hz throttle, then worker submit."""

    def tick(
        self,
        *,
        lifecycle_now_s: float,
        obs_now_s: float,
        wall_time_us: int,
    ) -> TaskTickResult:
        """Advance, reset on strike crossing, consume, then assemble task obs."""
```

- 写 worker trace 回归测试：无 listener 时旧 stats/latest-only 行为逐项不变；listener 时 pending/result 覆盖身份准确。
- 写 listener 抛异常不影响 worker、nested command freeze 后不可修改、诊断 `close(timeout_s=5.0)` 有界返回的测试。
- 写 100 Hz throttle 测试：基于 snapshot `received_monotonic_s`，不是 wall/source time。
- 100 Hz 必须复制生产的事件驱动判断：`received_time - last_submit >= 0.01 - 1e-12`，不得实现成独立周期 timer。
- 写 incoming 的 exact production tests：not-ready/base-invalid 在 `observe()` 前拒绝且保留已有 count；non-incoming 调用 `observe()` 后 count 清零；confirmed latch；invisible 只有真正进入 plan_fn 才 reset。
- 写 reason mapper 测试，保留异常原始 type/text/value，并映射到固定 estimator/incoming/planner reason code。
- 写 50 Hz tick 顺序测试：先 `lifecycle_now_s` advance/consume，再 `obs_now_s` policy_tts/latest pelvis；两个时间值不得合并。
- 精确顺序测试固定为：`lifecycle.advance → 若 ARMED crossing 则同步 adapter.reset_estimator_after_strike()/epoch++ 并通知 AttemptTracker production reset → 读取/消费 latest planner result → 捕获 obs_now/latest pelvis → helper`。
- 写当前/latest pelvis 与旧 planner snapshot pelvis 不同的测试，task base-forward/target 必须使用 tick 时 pelvis。
- 写 lifecycle `TRACKING/RECOVERY` 不算首次 PASS、只有 `ARMED` + identity 一致 + 11 finite + clip_count 0 才 PASS。
- 写 `OBS_COMMAND_MISMATCH`：`TaskTickResult.command_result`、`command_fields_used`、active binding、base/racket/TTS 必须来自同一 tick 的 active command，任一字段或 source key 不一致即失败。
- 写 invalid invisible snapshot 可能被 throttle/pending/result 覆盖的测试，证明 pipeline 不提前通知 lifecycle。
- 写 strike deadline 后 estimator reset/epoch advance、RECOVERY/cached next attempt、stale generation 和 result overwrite 测试。
- 每个 online attempt 必须通过 immutable events 完整记录 Task 8 所需事实：raw input_seq、submit/start/complete/replacement、真实 planner duration、50 Hz lifecycle_now/obs_now、decision/phase、active/cached key、实际 recovery duration、submission phase anchor、command fields、pre/post task values、clip count 和 recording completeness。
- 运行失败测试：

  ```bash
  cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
  PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover -s tests -p 'test_hitter_task_pipeline.py' -v
  ```

- 实现所有实时路径；event/raw 提交必须 nonblocking，pipeline 回调立即生成深度不可变 payload，不做 JSON 编码、磁盘 I/O 或 HTTP。
- 运行 Task 1、Task 6、Task 7 与现存 lifecycle 相关测试：

  ```bash
  cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
  for test_file in \
    test_hitter_task_observation.py \
    test_hitter_task_input_adapter.py \
    test_hitter_task_pipeline.py \
    test_hitter_strike_target_logging.py; do
    PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover -s tests -p "$test_file" -v || exit 1
  done
  ```

- 提交，`hitter_realtime.py` 只暂存本任务 hunk：

  ```bash
  git add deploy/diagnostics/hitter_task_pipeline.py deploy/tests/test_hitter_task_pipeline.py
  git add -p deploy/utils/hitter_realtime.py
  git diff --cached --check
  git commit -m "feat: trace HITTER shadow task pipeline"
  ```

