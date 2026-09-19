## Task 8: 实现确定性 baseline replay 与 `1/100` A/B

**Files:**

- Create: `deploy/diagnostics/hitter_task_replay.py`
- Create: `deploy/tests/test_hitter_task_replay.py`
- Create: `deploy/tests/test_hitter_task_replay_process.py`

### Interface

```python
@dataclass(frozen=True)
class ReplayVariant:
    name: str
    planner_rate_hz: float
    incoming_confirmations: int


@dataclass(frozen=True)
class ReplayOutcome:
    variant: str
    terminal_code: str
    task_obs_pass: bool
    boundary_sensitive: bool
    recording_complete: bool
    warnings: tuple[str, ...]
    summary_label: str | None
    metrics: Mapping[str, float | int | str | None]


@dataclass(frozen=True)
class PolicyTickRecord:
    tick_index: int
    lifecycle_now_s: float
    obs_now_s: float
    lifecycle_decision: str
    phase: str
    active_key: SnapshotKey | None
    cached_key: SnapshotKey | None
    command_fields: Mapping[str, JsonValue] | None
    task_pre_clip: tuple[float, ...] | None
    task_post_clip: tuple[float, ...] | None
    task_clip_count: int | None


@dataclass(frozen=True)
class PlannerCallRecord:
    snapshot_key: SnapshotKey
    submitted_monotonic_s: float
    started_monotonic_s: float
    completed_monotonic_s: float
    duration_s: float
    pending_replaced_key: SnapshotKey | None
    latest_replaced_key: SnapshotKey | None
    result: FrozenPlannerResult


@dataclass(frozen=True)
class OnlineAttemptTrace:
    attempt_id: int
    stages: tuple[AttemptTransition, ...]
    planner_calls: tuple[PlannerCallRecord, ...]
    policy_ticks: tuple[PolicyTickRecord, ...]
    terminal_code: str
    recovery_duration_s: float | None
    submission_phase_anchor_s: float | None
    recording_complete: bool


@dataclass(frozen=True)
class ReplayInputBundle:
    attempt_id: int
    inputs: tuple[NormalizedMocapSample, ...]
    online_baseline: OnlineAttemptTrace
    recording_complete: bool


class PlannerProtocol(Protocol):
    def plan_command(
        self,
        ball_position: np.ndarray,
        ball_velocity: np.ndarray,
        *,
        current_base_xy_w: np.ndarray,
        base_forward_xy_w: np.ndarray,
        strike_type: str | None,
    ) -> object:
        """Return one HitterWbcCommand-compatible command."""


class PlannerDurationProvider(Protocol):
    def duration_s(
        self,
        *,
        variant: ReplayVariant,
        snapshot_key: SnapshotKey,
        measured_offline_duration_s: float | None,
    ) -> float:
        """Reuse online duration or return one recorded offline duration."""


@dataclass(frozen=True)
class ReplayTrace:
    variant: ReplayVariant
    stages: tuple[AttemptTransition, ...]
    planner_calls: tuple[PlannerCallRecord, ...]
    policy_ticks: tuple[PolicyTickRecord, ...]
    outcome: ReplayOutcome


@dataclass(frozen=True)
class ReplayParity:
    matches: bool
    mismatches: tuple[str, ...]


@dataclass(frozen=True)
class ReplayComparison:
    baseline: ReplayOutcome
    one_frame: ReplayOutcome | None
    parity: ReplayParity
    summary_label: str
    deltas: Mapping[str, JsonValue]


def replay_attempt(
    bundle: ReplayInputBundle,
    *,
    variant: ReplayVariant,
    duration_provider: PlannerDurationProvider,
    planner_factory: Callable[[], PlannerProtocol],
) -> ReplayTrace:
    """Run one deterministic discrete-event replay without real sleep."""


def compare_baseline_to_online(
    *,
    online: OnlineAttemptTrace,
    replay: ReplayTrace,
    atol: float = 1.0e-6,
) -> ReplayParity:
    """Compare every canonical online fact before counterfactual execution."""


def analyze_attempt_ab(
    bundle: ReplayInputBundle,
    *,
    planner_factory: Callable[[], PlannerProtocol],
    duration_provider: PlannerDurationProvider,
) -> ReplayComparison:
    """Run 3/100, gate on parity, then and only then run fresh 1/100."""


@dataclass(frozen=True)
class ReplayJobRef:
    session_basename: str
    attempt_id: int
    input_path: Path


@dataclass(frozen=True)
class ReplayWorkerReply:
    attempt_id: int
    status: str
    result: Mapping[str, JsonValue] | None
    error: str | None


class ReplayProcessController:
    def set_realtime_busy(
        self,
        *,
        attempt_active: bool,
        reacquire_grace_active: bool,
    ) -> None:
        """Set/clear pause and wait for child PAUSED acknowledgement."""

    def submit_disk_job(self, job: ReplayJobRef) -> None:
        """Append QUEUED to replay_jobs.jsonl before IPC submission."""

    def poll(self) -> tuple[ReplayWorkerReply, ...]:
        """Parent receives replies, records them, and publishes events."""

    def close(self, *, timeout_s: float = 5.0) -> bool:
        """Stop the spawned child with a bounded join."""
```

`analyze_attempt_ab()` 的固定顺序为：

```python
baseline = replay_attempt(
    bundle,
    variant=ReplayVariant("3/100", 100.0, 3),
    duration_provider=duration_provider,
    planner_factory=planner_factory,
)
parity = compare_baseline_to_online(
    online=bundle.online_baseline,
    replay=baseline,
)
if not parity.matches:
    return replay_divergence_comparison(baseline, parity)
one_frame = replay_attempt(
    bundle,
    variant=ReplayVariant("1/100", 100.0, 1),
    duration_provider=duration_provider,
    planner_factory=planner_factory,
)
return compare_completed_variants(baseline, one_frame, parity)
```

每次 `replay_attempt()` 必须调用 `planner_factory()` 得到全新 planner。canonical parity 逐项比较：stage/reason、attempt/segment/key、submit/start/complete/replacement 序列和统计、lifecycle decision/phase、active/cached key、实际 recovery duration、完整 command fields、pre/post task values、clip count、terminal code；浮点绝对容差固定 `1e-6`。
terminal code 固定使用 `TASK_OBS_PASS`、`LATE_SKIP`、`TRACK_ENDED_BEFORE_READY`、`TRACK_ENDED_BEFORE_CONFIRMATION`、`NO_VALID_PLAN`、`PELVIS_UNAVAILABLE`、`MALFORMED_COMMAND`、`RECORDING_INCOMPLETE`、`REPLAY_DIVERGENCE`；禁止同时出现缩写 `TRACK_ENDED_BEFORE_CONFIRM`。

- 实现测试用 heap scheduler，排序键固定为 `(at_s, event_priority, global_order_seq)`；`input_seq` 仅保留在 INPUT payload。
- 写 fake 100 Hz submit phase、capacity-one pending/in-flight/latest-result、planner start/complete、50 Hz `lifecycle_now_s`/`obs_now_s` 的金丝雀测试。
- 固定同一虚拟时刻的事件优先级：`INPUT → PLAN_COMPLETE → PLAN_START → LIFECYCLE_TICK → OBS_TICK → GRACE_EXPIRE`；incoming count 只在 `PLAN_START` 增加，不在 raw input/submit 时增加。idle submit 和 complete 后 pending start 都必须形成 PLAN_START。
- 100 Hz submission 完全使用 recorded input arrival 做事件驱动 throttle：`received_monotonic_s-last_submit >= 0.01-1e-12`，不生成独立周期 timer。
- 写线上 baseline 与 replay 阶段/identity/ARM/task obs 完全一致测试；任一不同必须 `REPLAY_DIVERGENCE + INCONCLUSIVE`。
- 写 `3/100` late、同输入 `1/100` PASS 的 `SAVED_BY_ONE_FRAME` 测试。
- 写 `SAME_PASS`、`BASELINE_ONLY_PASS`、`BOTH_FAIL_SAME`、`BOTH_FAIL_DIFFERENT`、`BOTH_PASS_DIFFERENT_COMMAND`、`INCONCLUSIVE`。
- 写 `ONE_FRAME_UNSTABLE`、距 ARM 边界 `<5 ms` 的 `BOUNDARY_SENSITIVE` 和 delta metrics。
- 写 baseline duration 按 online snapshot identity 复用，`1/100` 新 planner 调用由离线 provider 实测并记录的测试。
- 写 replay supervisor 仅在无 active/reacquire attempt 时派发；新 attempt 发送 pause，父进程是唯一 EventHub/recorder writer。pending job 状态写入独立 `replay_jobs.jsonl`（QUEUED/RUNNING/PAUSED/COMPLETED/FAILED），不是分析输出文件。
- 在 `test_hitter_task_replay_process.py` 使用 `multiprocessing.get_context("spawn")` 和 Event/Queue 同步验证：active/reacquire 时 idle；新 attempt 暂停；磁盘 job 不丢；下一 idle window 从头确定性重放；child failure 只返回 FAILED；close 可在 5 秒内 join。禁止用固定 sleep 猜时序。
- child 在每个 scheduler event 前后检查 pause；PAUSED 后丢弃半执行的 mutable planner 状态，下次 idle 从输入文件开头确定性重放。child 禁止直接写 EventHub 或 session 文件。
- 运行失败测试：

  ```bash
  cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
  PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover -s tests -p 'test_hitter_task_replay.py' -v
  PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover -s tests -p 'test_hitter_task_replay_process.py' -v
  ```

- 实现默认 variants，代码中不得存在实时 `360 Hz` 完整 planner variant；360 Hz raw 数据只作为输入保留。
- 提交：

  ```bash
  git add deploy/diagnostics/hitter_task_replay.py \
    deploy/tests/test_hitter_task_replay.py \
    deploy/tests/test_hitter_task_replay_process.py
  git diff --cached --check
  git commit -m "feat: compare HITTER incoming replay variants"
  ```

