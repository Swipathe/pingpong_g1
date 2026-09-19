## Task 3: 定义不可变诊断协议和逐球身份状态机

**Files:**

- Create: `deploy/diagnostics/__init__.py`
- Create: `deploy/diagnostics/hitter_task_models.py`
- Create: `deploy/diagnostics/hitter_task_attempts.py`
- Create: `deploy/tests/test_hitter_task_attempts.py`

### Core contracts

`hitter_task_models.py` 至少定义：

```python
@dataclass(frozen=True)
class NormalizedMocapSample:
    input_seq: int
    channel: str
    subject: str
    position_w: np.ndarray
    quaternion_xyzw: np.ndarray
    valid: bool
    occluded: bool
    source_frame: int
    source_time_s: float | None
    publish_time_us: int | None
    received_monotonic_s: float
    wall_time_us: int
    payload_size: int


JsonScalar = Union[None, bool, int, float, str]
JsonValue = Union[
    JsonScalar,
    Tuple["JsonValue", ...],
    Mapping[str, "JsonValue"],
]


@dataclass(frozen=True, order=True)
class SnapshotKey:
    track_epoch: int
    generation: int


@dataclass(frozen=True)
class AttemptBinding:
    attempt_id: int
    track_segment_id: int
    role: str
    snapshot_key: SnapshotKey


@dataclass(frozen=True)
class AttemptTransition:
    attempt_id: int
    track_segment_id: int | None
    stage: str
    monotonic_s: float
    snapshot_key: SnapshotKey | None
    reason_code: str | None
    values: Mapping[str, JsonValue]


@dataclass(frozen=True)
class EventDraft:
    kind: str
    monotonic_s: float
    wall_time_us: int
    scope: str
    attempt_id: int | None
    payload: Mapping[str, JsonValue]


@dataclass(frozen=True)
class HealthSnapshot:
    lcm_connected: bool
    message_rate_hz_by_subject: Mapping[str, float]
    message_age_s_by_subject: Mapping[str, float | None]
    source_frame_by_subject: Mapping[str, int | None]
    pelvis_valid: bool
    pelvis_age_s: float | None
    planner_submitted: int
    planner_completed: int
    planner_failed: int
    planner_dropped_pending: int
    planner_results_overwritten_before_consume: int
    raw_samples_dropped: int
    recorder_event_gaps: int
    diagnostic_events_dropped: int
    recorder_healthy: bool
    recording_complete: bool
    config_name: str
    session_basename: str
    warnings: tuple[str, ...]


@dataclass(frozen=True)
class LifecycleSnapshot:
    phase: str
    last_decision: str
    active_key: SnapshotKey | None
    cached_key: SnapshotKey | None
    lifecycle_now_s: float | None
    obs_now_s: float | None


@dataclass(frozen=True)
class AttemptSummary:
    attempt_id: int
    status: str
    stage: str
    primary_blocker: str | None
    ball_speed_mps: float | None
    predicted_strike_time_s: float | None
    planner_tts_s: float | None
    arm_tts_s: float | None
    task_obs_status: str
    ab_summary: str | None
    recording_complete: bool


@dataclass(frozen=True)
class AttemptDetail:
    attempt_id: int
    summary: AttemptSummary
    segments: tuple[Mapping[str, JsonValue], ...]
    stage_timeline: tuple[Mapping[str, JsonValue], ...]
    planner_inputs: tuple[Mapping[str, JsonValue], ...]
    planner_results: tuple[Mapping[str, JsonValue], ...]
    task_observation_pre_clip: tuple[float, ...] | None
    task_observation_post_clip: tuple[float, ...] | None
    task_observation_clip_count: int | None
    variant_outcomes: tuple[Mapping[str, JsonValue], ...]
    ab_deltas: Mapping[str, JsonValue]


@dataclass(frozen=True)
class AttemptPage:
    items: tuple[AttemptSummary, ...]
    has_more: bool
    next_before: int | None
```

所有 ndarray 在 `__post_init__` 中复制并设为只读；跨进程/落盘通过固定 `schema_version=1` 的显式 `to_json_dict()`，禁止 pickle 任意对象作为持久化格式。
所有标准化 mocap position/quaternion 必须保存为只读 `float64`，CSV/JSON 使用足以往返 float64 的表示。上述字段名、`_s`/`_mps` 单位后缀和 `None → JSON null` 是 HTTP/前端/落盘共同契约；`to_json_dict()` 必须逐字段显式输出，禁止直接暴露 `__dict__` 或绝对路径。

`hitter_task_attempts.py` 提供：

```python
class AttemptTracker:
    def observe_ball_sample(
        self,
        sample: NormalizedMocapSample,
        *,
        snapshot_key: SnapshotKey,
    ) -> tuple[AttemptTransition, ...]:
        """Update display grouping only; never mutate estimator/incoming/lifecycle."""

    def observe_production_reset(
        self,
        *,
        previous_track_epoch: int,
        new_track_epoch: int,
        reason: str,
        monotonic_s: float,
    ) -> tuple[AttemptTransition, ...]:
        """End the production segment on invalid or strike-deadline reset."""

    def bind_snapshot(
        self,
        *,
        snapshot_key: SnapshotKey,
    ) -> AttemptBinding | None:
        """Bind async planner identity without guessing from current visibility."""

    def binding_for_result(
        self,
        snapshot_key: SnapshotKey,
    ) -> AttemptBinding | None:
        """Look up immutable async-result ownership."""

    def record_stage(
        self,
        *,
        binding: AttemptBinding,
        stage: str,
        monotonic_s: float,
        reason_code: str | None,
        values: Mapping[str, JsonValue],
    ) -> AttemptTransition:
        """Append a stage/rejection without changing production state."""

    def record_task_pass(
        self,
        *,
        binding: AttemptBinding,
        monotonic_s: float,
    ) -> AttemptTransition:
        """Mark first task PASS and make later reacquisition a post-deadline tail."""

    def record_health_warning(
        self,
        *,
        reason_code: str,
        monotonic_s: float,
        values: Mapping[str, JsonValue],
    ) -> AttemptTransition | None:
        """Record warning only; never open/close attempts."""

    def set_lifecycle_context(
        self,
        *,
        phase: str,
        active_key: SnapshotKey | None,
        cached_key: SnapshotKey | None,
        monotonic_s: float,
    ) -> tuple[AttemptTransition, ...]:
        """Project recovery/cached status onto the owning attempt."""

    def advance(
        self,
        *,
        now_monotonic_s: float,
    ) -> tuple[AttemptTransition, ...]:
        """Close expired reacquire grace without changing production state."""
```

- 先写测试：visible 上升沿创建 attempt/segment；global `attempt_id` 和 `track_segment_id` 单调递增。
- 写 invalid 已推进 epoch 的测试：旧 segment 关闭，新 invisible snapshot 仍绑定正确 attempt，tracker 不调用任何 production reset。
- 将边界例固定为：visible `(epoch=5,generation=31)` 属于 attempt 1 / segment 1；invalid 后的 invisible `(6,32)` 仍绑定 attempt 1 / segment 1；grace 内 visible `(6,33)` 属于 attempt 1 / segment 2。
- 写 `0.20 s` grace 内重捕获同 attempt 新 segment、grace 外新 attempt、PASS 后 `POST_DEADLINE_TAIL`。
- grace 边界使用闭区间：`now <= invalid_time + 0.20` 仍归原 attempt，只有严格大于 deadline 才关闭。
- 写 heartbeat/pelvis stale 只产生 warning、不关闭 attempt、不改变 segment 的测试。
- 写 recovery 中 `WAITING_FOR_PREVIOUS_RECOVERY` / `CACHED_DURING_RECOVERY` 的展示映射测试。
- 写 `(track_epoch, generation)` 绑定和迟到异步结果归属测试。
- 写 attempt close 的 primary blocker 表驱动测试：任一 segment PASS 则成功；否则按 never-ready → never-confirmed → no-valid-plan/last-planner-reason → late/before-arm → explicit obs failure 的最远阶段顺序选择，较早 warning 只留在 timeline。
- 运行并确认失败：

  ```bash
  cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
  PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover -s tests -p 'test_hitter_task_attempts.py' -v
  ```

- 实现 immutable model、deep JSON freeze/copy 和纯展示 tracker；tracker 中禁止 import planner/LCM。
- 运行测试，额外检查源码中不存在展示层直接调用 `.reset()` 或 `.mark_track_ended()`。
- 提交：

  ```bash
  git add deploy/diagnostics/__init__.py \
    deploy/diagnostics/hitter_task_models.py \
    deploy/diagnostics/hitter_task_attempts.py \
    deploy/tests/test_hitter_task_attempts.py
  git diff --cached --check
  git commit -m "feat: model HITTER diagnostic attempts"
  ```

