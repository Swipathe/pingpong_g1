from __future__ import annotations

import threading
import time
from dataclasses import dataclass, fields, is_dataclass, replace
from enum import Enum
from typing import Any, Callable, Mapping, Optional, Tuple

import numpy as np

from diagnostics.hitter_task_models import (
    JsonValue,
    freeze_json_value,
)
from utils.hitter_runtime_types import (
    PlannerFailureReason,
    PlannerRejected,
    SnapshotKey,
)
from utils.hitter_planner import HitterWbcCommand, StrikePlan


def _copied_read_only_array(value) -> np.ndarray:
    array = np.asarray(value).copy()
    immutable = np.frombuffer(
        array.tobytes(order="C"),
        dtype=array.dtype,
    ).reshape(array.shape)
    immutable.setflags(write=False)
    return immutable


@dataclass(frozen=True)
class BallEstimateSnapshot:
    track_id: int
    generation: int
    source_frame: int
    source_time_s: float
    received_monotonic_s: float
    position_w: np.ndarray
    velocity_w: np.ndarray
    base_position_w: np.ndarray
    base_quaternion_xyzw: np.ndarray
    base_valid: bool
    visible: bool
    ready: bool
    consumed: bool = False
    new_track: bool = False

    def __post_init__(self) -> None:
        for name in (
            "position_w",
            "velocity_w",
            "base_position_w",
            "base_quaternion_xyzw",
        ):
            object.__setattr__(
                self,
                name,
                _copied_read_only_array(getattr(self, name)),
            )


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
        failure_reason = self.failure_reason
        error_text = self.error_text

        if failure_reason is not None and not isinstance(
            failure_reason,
            PlannerFailureReason,
        ):
            raise ValueError("failure_reason must be a PlannerFailureReason or None")
        if error_text is not None and type(error_text) is not str:
            raise ValueError("error_text must be a string or None")

        succeeded = self.command is not None and failure_reason is None
        failed = self.command is None and failure_reason is not None
        if not (succeeded or failed):
            raise ValueError("planner result must be exactly success or failure")

        if type(self.track_id) is not int or self.track_id <= 0:
            raise ValueError("track_id must be a positive integer")
        if (
            type(self.source_generation) is not int
            or self.source_generation < 0
        ):
            raise ValueError("source_generation must be a non-negative integer")
        if type(self.source_frame) is not int or self.source_frame < 0:
            raise ValueError("source_frame must be a non-negative integer")
        if (
            not np.isfinite(self.completed_monotonic_s)
            or self.completed_monotonic_s < 0.0
        ):
            raise ValueError("completed_monotonic_s must be finite and non-negative")
        deadline = float(self.strike_deadline_monotonic_s)
        if np.isinf(deadline) or (succeeded and not np.isfinite(deadline)):
            raise ValueError(
                "strike_deadline_monotonic_s must be finite for success "
                "and never infinite"
            )
        if np.isfinite(deadline) and deadline < 0.0:
            raise ValueError(
                "strike_deadline_monotonic_s must be non-negative when finite"
            )

        object.__setattr__(self, "failure_reason", failure_reason)
        object.__setattr__(self, "error_text", error_text)

    @property
    def error(self) -> str | None:
        if self.failure_reason is None:
            return None
        return self.failure_reason.value


@dataclass(frozen=True)
class PlannerWorkerStats:
    submitted: int
    completed: int
    failed: int
    dropped_pending: int
    trace_listener_failures: int = 0
    pending_replaced_total: int = 0
    completed_result_queue_overflow_total: int = 0


@dataclass(frozen=True)
class IncomingTrackSnapshot:
    track_id: Optional[int]
    consecutive_count: int
    confirmed: bool


def _command_json_value(value: Any) -> Any:
    if is_dataclass(value):
        return {
            field.name: _command_json_value(getattr(value, field.name))
            for field in fields(value)
        }
    if isinstance(value, np.ndarray):
        return tuple(_command_json_value(item) for item in value.tolist())
    if isinstance(value, np.generic):
        return _command_json_value(value.item())
    if isinstance(value, Mapping):
        return {
            str(key): _command_json_value(item)
            for key, item in value.items()
        }
    if isinstance(value, (tuple, list)):
        return tuple(_command_json_value(item) for item in value)
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    return repr(value)[:2048]


def freeze_planner_command_fields(
    command: object,
) -> Mapping[str, JsonValue]:
    """Freeze one production command for diagnostic identity checks."""
    converted = _command_json_value(command)
    if not isinstance(converted, Mapping):
        converted = {"value": converted}
    frozen = freeze_json_value(converted)
    if not isinstance(frozen, Mapping):
        raise TypeError("planner command must freeze to a mapping")
    return frozen

@dataclass(frozen=True)
class FrozenPlannerResult:
    snapshot_key: SnapshotKey
    source_frame: int
    strike_deadline_monotonic_s: float
    completed_monotonic_s: float
    command_fields: Optional[Mapping[str, JsonValue]]
    error_type: Optional[str]
    error_text: Optional[str]
    failure_reason: PlannerFailureReason | None = None

    def __post_init__(self) -> None:
        if self.command_fields is not None:
            frozen = freeze_json_value(self.command_fields)
            if not isinstance(frozen, Mapping):
                raise TypeError("command_fields must be a mapping")
            object.__setattr__(self, "command_fields", frozen)
        if self.error_type is not None:
            object.__setattr__(self, "error_type", str(self.error_type))
        if self.error_text is not None:
            object.__setattr__(
                self,
                "error_text",
                str(self.error_text)[:2048],
            )
        if self.failure_reason is not None and not isinstance(
            self.failure_reason,
            PlannerFailureReason,
        ):
            raise ValueError(
                "failure_reason must be a PlannerFailureReason or None"
            )


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
        if type(self.overflowed) is not bool:
            raise ValueError("overflowed must be a boolean")
        if self.overflowed != (self.overflow_count > 0):
            raise ValueError("overflowed must match this drain's overflow_count")
        if any(
            type(track_id) is not int or track_id <= 0
            for track_id in self.overflowed_track_ids
        ):
            raise ValueError(
                "overflowed_track_ids must contain positive integers"
            )
        if self.overflowed_track_ids != tuple(
            sorted(set(self.overflowed_track_ids))
        ):
            raise ValueError(
                "overflowed_track_ids must be sorted and unique"
            )
        if self.overflowed and not self.overflowed_track_ids:
            raise ValueError("overflow must identify at least one affected track")
        if self.overflow_count < len(self.overflowed_track_ids):
            raise ValueError(
                "overflow count cannot be smaller than unique track ids"
            )


@dataclass(frozen=True)
class PlannerWorkerTrace:
    trace_seq: int
    kind: str
    monotonic_s: float
    snapshot: BallEstimateSnapshot
    result: Optional[FrozenPlannerResult]
    replaced_snapshot: Optional[BallEstimateSnapshot]
    replaced_result: Optional[FrozenPlannerResult]
    stats: PlannerWorkerStats


class IncomingTrackConfirmation:
    """Confirm stable fitted incoming motion once per ball track id."""

    def __init__(
        self,
        *,
        minimum_speed_x_mps: float,
        required_consecutive_snapshots: int,
    ):
        self.minimum_speed_x_mps = float(minimum_speed_x_mps)
        self.required_consecutive_snapshots = int(
            required_consecutive_snapshots
        )
        if (
            not np.isfinite(self.minimum_speed_x_mps)
            or self.minimum_speed_x_mps <= 0.0
        ):
            raise ValueError("minimum_speed_x_mps must be finite and positive")
        if self.required_consecutive_snapshots < 1:
            raise ValueError(
                "required_consecutive_snapshots must be positive"
            )
        self._lock = threading.RLock()
        self.reset()

    def reset(self) -> None:
        with self._lock:
            self._track_id = None
            self._consecutive_count = 0
            self._confirmed = False

    def observe(self, *, track_id: int, velocity_x_mps: float) -> bool:
        with self._lock:
            track_id = int(track_id)
            if self._track_id != track_id:
                self._track_id = track_id
                self._consecutive_count = 0
                self._confirmed = False
            if self._confirmed:
                return True

            velocity_x = float(velocity_x_mps)
            if (
                np.isfinite(velocity_x)
                and velocity_x <= -self.minimum_speed_x_mps
            ):
                self._consecutive_count += 1
            else:
                self._consecutive_count = 0
            if (
                self._consecutive_count
                >= self.required_consecutive_snapshots
            ):
                self._confirmed = True
            return self._confirmed

    def snapshot(self) -> IncomingTrackSnapshot:
        with self._lock:
            return IncomingTrackSnapshot(
                track_id=self._track_id,
                consecutive_count=int(self._consecutive_count),
                confirmed=bool(self._confirmed),
            )


class LatestOnlyPlannerWorker:
    """Run a planner off-thread while retaining at most one pending snapshot."""

    def __init__(
        self,
        plan_fn: Callable[[BallEstimateSnapshot], object],
        *,
        monotonic_fn: Callable[[], float] = time.monotonic,
        trace_listener: Optional[
            Callable[[PlannerWorkerTrace], None]
        ] = None,
        completed_result_queue_capacity: int = 64,
    ):
        if (
            type(completed_result_queue_capacity) is not int
            or completed_result_queue_capacity <= 0
        ):
            raise ValueError(
                "completed_result_queue_capacity must be a positive integer"
            )
        self._plan_fn = plan_fn
        self._monotonic_fn = monotonic_fn
        self._trace_listener = trace_listener
        self._condition = threading.Condition()
        self._trace_delivery_condition = threading.Condition(
            threading.RLock()
        )
        self._trace_delivery_pending = {}
        self._next_trace_delivery_seq = 1
        self._trace_delivery_active = False
        self._pending: BallEstimateSnapshot | None = None
        self._latest_result: PlannerResultSnapshot | None = None
        self._latest_frozen_result: FrozenPlannerResult | None = None
        self._completed_result_queue_capacity = completed_result_queue_capacity
        self._completed_results: list[
            tuple[PlannerResultSnapshot, FrozenPlannerResult | None]
        ] = []
        self._completed_result_queue_overflow_latched = False
        self._completed_result_queue_overflow_since_drain = 0
        self._completed_result_queue_overflow_total = 0
        self._completed_result_queue_overflow_track_ids_since_drain: set[
            int
        ] = set()
        self._stop_requested = False
        self._submitted = 0
        self._completed = 0
        self._failed = 0
        self._dropped_pending = 0
        self._trace_listener_failures = 0
        self._trace_seq = 0
        self._max_pending_depth = 0
        self._thread = threading.Thread(
            target=self._run,
            name=f"LatestOnlyPlannerWorker-{id(self):x}",
        )
        self._thread.start()

    @property
    def stats(self) -> PlannerWorkerStats:
        with self._condition:
            return self._stats_locked()

    def _stats_locked(self) -> PlannerWorkerStats:
        return PlannerWorkerStats(
            submitted=self._submitted,
            completed=self._completed,
            failed=self._failed,
            dropped_pending=self._dropped_pending,
            trace_listener_failures=self._trace_listener_failures,
            pending_replaced_total=self._dropped_pending,
            completed_result_queue_overflow_total=(
                self._completed_result_queue_overflow_total
            ),
        )

    @property
    def trace_listener_failures(self) -> int:
        with self._condition:
            return int(self._trace_listener_failures)

    @property
    def max_pending_depth(self) -> int:
        with self._condition:
            return self._max_pending_depth

    def submit(self, snapshot: BallEstimateSnapshot) -> None:
        traces = []
        with self._condition:
            if self._stop_requested:
                raise RuntimeError("planner worker is closed")
            replaced = self._pending
            self._submitted += 1
            if replaced is not None:
                self._dropped_pending += 1
            self._pending = snapshot
            self._max_pending_depth = max(self._max_pending_depth, 1)
            if self._trace_listener is not None:
                traces.append(
                    self._trace_locked(
                        kind="submit",
                        snapshot=snapshot,
                    )
                )
                if replaced is not None:
                    traces.append(
                        self._trace_locked(
                            kind="pending_replaced",
                            snapshot=snapshot,
                            replaced_snapshot=replaced,
                        )
                    )
            self._condition.notify()
        self._notify_traces(traces)

    def latest_result(self) -> PlannerResultSnapshot | None:
        with self._condition:
            return self._latest_result

    def latest_result_bundle(
        self,
    ) -> Tuple[
        Optional[PlannerResultSnapshot],
        Optional[FrozenPlannerResult],
    ]:
        with self._condition:
            return self._latest_result, self._latest_frozen_result

    def drain_completed_results(self) -> CompletedResultBatch:
        with self._condition:
            results = tuple(pair[0] for pair in self._completed_results)
            frozen_results = tuple(
                pair[1] for pair in self._completed_results
            )
            batch = CompletedResultBatch(
                results=results,
                frozen_results=frozen_results,
                overflowed=self._completed_result_queue_overflow_latched,
                overflow_count=(
                    self._completed_result_queue_overflow_since_drain
                ),
                overflowed_track_ids=tuple(
                    sorted(
                        self._completed_result_queue_overflow_track_ids_since_drain
                    )
                ),
            )
            self._completed_results.clear()
            self._completed_result_queue_overflow_latched = False
            self._completed_result_queue_overflow_since_drain = 0
            self._completed_result_queue_overflow_track_ids_since_drain.clear()
            return batch

    def close(self, timeout_s: Optional[float] = None) -> bool:
        if timeout_s is not None and timeout_s < 0.0:
            raise ValueError("timeout_s must be nonnegative")
        with self._condition:
            self._stop_requested = True
            self._pending = None
            self._condition.notify_all()
        if threading.current_thread() is not self._thread:
            self._thread.join(timeout_s)
        return not self._thread.is_alive()

    def _trace_locked(
        self,
        *,
        kind: str,
        snapshot: BallEstimateSnapshot,
        result: Optional[FrozenPlannerResult] = None,
        replaced_snapshot: Optional[BallEstimateSnapshot] = None,
        replaced_result: Optional[FrozenPlannerResult] = None,
    ) -> PlannerWorkerTrace:
        self._trace_seq += 1
        return PlannerWorkerTrace(
            trace_seq=self._trace_seq,
            kind=kind,
            monotonic_s=float(self._monotonic_fn()),
            snapshot=snapshot,
            result=result,
            replaced_snapshot=replaced_snapshot,
            replaced_result=replaced_result,
            stats=self._stats_locked(),
        )

    def _notify_traces(
        self,
        traces: list[PlannerWorkerTrace],
    ) -> None:
        listener = self._trace_listener
        if listener is None:
            return
        with self._trace_delivery_condition:
            for trace in traces:
                self._trace_delivery_pending[trace.trace_seq] = trace
            if self._trace_delivery_active:
                return
            self._trace_delivery_active = True
        while True:
            with self._trace_delivery_condition:
                trace = self._trace_delivery_pending.pop(
                    self._next_trace_delivery_seq,
                    None,
                )
                if trace is None:
                    self._trace_delivery_active = False
                    return
            try:
                listener(trace)
            except Exception:
                with self._condition:
                    self._trace_listener_failures += 1
            except BaseException:
                with self._trace_delivery_condition:
                    self._trace_delivery_active = False
                raise
            with self._trace_delivery_condition:
                self._next_trace_delivery_seq += 1

    def _run(self) -> None:
        while True:
            start_traces = []
            with self._condition:
                while self._pending is None and not self._stop_requested:
                    self._condition.wait()
                if self._stop_requested:
                    return
                snapshot = self._pending
                self._pending = None
                if self._trace_listener is not None:
                    start_traces.append(
                        self._trace_locked(
                            kind="start",
                            snapshot=snapshot,
                        )
                    )
            self._notify_traces(start_traces)

            failed = False
            failure_reason = None
            error_type = None
            error_text = None
            diagnostic_error_text = None
            try:
                command = self._plan_fn(snapshot)
                deadline = snapshot.received_monotonic_s + float(
                    command.time_to_strike
                )
                planned = PlannerResultSnapshot(
                    track_id=snapshot.track_id,
                    source_generation=snapshot.generation,
                    source_frame=snapshot.source_frame,
                    strike_deadline_monotonic_s=deadline,
                    completed_monotonic_s=float(self._monotonic_fn()),
                    command=command,
                )
            except PlannerRejected as exc:
                failed = True
                failure_reason = exc.reason
                error_type = type(exc).__name__
                error_text = exc.detail
                diagnostic_error_text = exc.detail
                planned = PlannerResultSnapshot(
                    track_id=snapshot.track_id,
                    source_generation=snapshot.generation,
                    source_frame=snapshot.source_frame,
                    strike_deadline_monotonic_s=float("nan"),
                    completed_monotonic_s=float(self._monotonic_fn()),
                    command=None,
                    failure_reason=failure_reason,
                    error_text=error_text,
                )
            except Exception as exc:
                failed = True
                failure_reason = PlannerFailureReason.INTERNAL_ERROR
                error_type = type(exc).__name__
                error_text = f"{type(exc).__name__}: {exc}"
                diagnostic_error_text = str(exc)
                planned = PlannerResultSnapshot(
                    track_id=snapshot.track_id,
                    source_generation=snapshot.generation,
                    source_frame=snapshot.source_frame,
                    strike_deadline_monotonic_s=float("nan"),
                    completed_monotonic_s=float(self._monotonic_fn()),
                    command=None,
                    failure_reason=failure_reason,
                    error_text=error_text,
                )

            frozen = None
            if self._trace_listener is not None:
                command_fields = None
                diagnostic_error_type = error_type
                if planned.command is not None:
                    try:
                        command_fields = freeze_planner_command_fields(
                            planned.command
                        )
                    except Exception as exc:
                        diagnostic_error_type = "DiagnosticFreezeError"
                        diagnostic_error_text = "{}: {}".format(
                            type(exc).__name__,
                            exc,
                        )
                frozen = FrozenPlannerResult(
                    snapshot_key=SnapshotKey(
                        int(snapshot.track_id),
                        int(snapshot.generation),
                    ),
                    source_frame=int(snapshot.source_frame),
                    strike_deadline_monotonic_s=float(
                        planned.strike_deadline_monotonic_s
                    ),
                    completed_monotonic_s=float(
                        planned.completed_monotonic_s
                    ),
                    command_fields=command_fields,
                    error_type=diagnostic_error_type,
                    error_text=diagnostic_error_text,
                    failure_reason=failure_reason,
                )
            completion_traces = []
            with self._condition:
                replaced_frozen = self._latest_frozen_result
                self._latest_result = planned
                self._latest_frozen_result = frozen
                if (
                    len(self._completed_results)
                    >= self._completed_result_queue_capacity
                ):
                    self._completed_result_queue_overflow_latched = True
                    self._completed_result_queue_overflow_since_drain += 1
                    self._completed_result_queue_overflow_total += 1
                    self._completed_result_queue_overflow_track_ids_since_drain.add(
                        planned.track_id
                    )
                else:
                    self._completed_results.append((planned, frozen))
                if failed:
                    self._failed += 1
                else:
                    self._completed += 1
                if self._trace_listener is not None:
                    completion_traces.append(
                        self._trace_locked(
                            kind="complete",
                            snapshot=snapshot,
                            result=frozen,
                        )
                    )
                    if replaced_frozen is not None:
                        completion_traces.append(
                            self._trace_locked(
                                kind="latest_result_replaced",
                                snapshot=snapshot,
                                result=frozen,
                                replaced_result=replaced_frozen,
                            )
                        )
                self._condition.notify_all()
            self._notify_traces(completion_traces)


class CommandPhase(str, Enum):
    WAITING = "waiting"
    TRACKING = "tracking"
    ARMED = "armed"
    RECOVERY = "recovery"


class LifecycleCancelReason(str, Enum):
    TRACK_ENDED = "TRACK_ENDED"
    ESTIMATOR_NOT_READY = "ESTIMATOR_NOT_READY"
    BASE_POSE_INVALID = "BASE_POSE_INVALID"
    BALL_NOT_INCOMING = "BALL_NOT_INCOMING"
    NO_FUTURE_CROSSING = "NO_FUTURE_CROSSING"
    HIT_HEIGHT_OUT_OF_RANGE = "HIT_HEIGHT_OUT_OF_RANGE"
    NONFINITE_INPUT_OR_OUTPUT = "NONFINITE_INPUT_OR_OUTPUT"
    INTERNAL_ERROR = "INTERNAL_ERROR"
    VICON_SCHEMA_ERROR = "VICON_SCHEMA_ERROR"
    VICON_STREAM_STALE = "VICON_STREAM_STALE"
    BALL_MESSAGE_STALE = "BALL_MESSAGE_STALE"
    TRACK_ID_CONFLICT = "TRACK_ID_CONFLICT"
    RESULT_QUEUE_OVERFLOW = "RESULT_QUEUE_OVERFLOW"
    OVERRIDE_DISCONTINUITY = "OVERRIDE_DISCONTINUITY"


@dataclass(frozen=True)
class LifecycleDecision:
    kind: str
    track_id: int | None
    command_changed: bool = False
    entered_waiting: bool = False
    cancel_reason: LifecycleCancelReason | None = None
    consumed_track_ids: tuple[int, ...] = ()

    _KINDS = frozenset(
        {
            "none",
            "tracking",
            "armed",
            "overridden",
            "retained_failure",
            "retained_discontinuity",
            "retained_committed",
            "retained_recovery",
            "cancelled",
            "skipped",
            "ignored_consumed",
            "ignored_generation",
            "ignored_track",
            "consumed_during_recovery",
            "cached_recovery",
            "struck",
            "entered_waiting",
            "session_reset",
        }
    )

    def __post_init__(self) -> None:
        if type(self.kind) is not str or self.kind not in self._KINDS:
            raise ValueError("kind is not a recognized lifecycle decision")
        if self.track_id is not None and (
            type(self.track_id) is not int or self.track_id <= 0
        ):
            raise ValueError("track_id must be a positive integer or None")
        if type(self.command_changed) is not bool:
            raise TypeError("command_changed must be a boolean")
        if type(self.entered_waiting) is not bool:
            raise TypeError("entered_waiting must be a boolean")
        if self.cancel_reason is not None and not isinstance(
            self.cancel_reason,
            LifecycleCancelReason,
        ):
            raise TypeError(
                "cancel_reason must be a LifecycleCancelReason or None"
            )
        if type(self.consumed_track_ids) is not tuple:
            raise TypeError("consumed_track_ids must be a tuple")
        canonical = tuple(sorted(set(self.consumed_track_ids)))
        if canonical != self.consumed_track_ids or any(
            type(track_id) is not int or track_id <= 0
            for track_id in canonical
        ):
            raise ValueError(
                "consumed_track_ids must be sorted unique positive ids"
            )


class HitterCommandLifecycle:
    """Pure state machine that binds planner results to one ball track id."""

    _IMMEDIATE_FAILURES = frozenset(
        {
            LifecycleCancelReason.TRACK_ENDED,
            LifecycleCancelReason.BASE_POSE_INVALID,
            LifecycleCancelReason.NONFINITE_INPUT_OR_OUTPUT,
            LifecycleCancelReason.INTERNAL_ERROR,
        }
    )
    _RETAIN_ONLY_FAILURES = frozenset(
        {
            LifecycleCancelReason.ESTIMATOR_NOT_READY,
            LifecycleCancelReason.HIT_HEIGHT_OUT_OF_RANGE,
            LifecycleCancelReason.OVERRIDE_DISCONTINUITY,
        }
    )
    _SOFT_FAILURES = frozenset(
        {
            LifecycleCancelReason.BALL_NOT_INCOMING,
            LifecycleCancelReason.NO_FUTURE_CROSSING,
        }
    )

    def __init__(
        self,
        *,
        waiting_tts: float = 0.92,
        arm_tts: float = 0.92,
        minimum_arm_tts: float = 0.30,
        maximum_policy_tts: float = 0.92,
        armed_cancel_consecutive_failures: int = 3,
        commit_time_to_strike_s: float = 0.30,
        maximum_racket_target_override_delta_m: float = 0.05,
        maximum_racket_velocity_override_delta_mps: float = 0.75,
        maximum_strike_deadline_override_delta_s: float = 0.05,
        swing_duration_sampler: Callable[[], float] = lambda: 1.85,
    ):
        waiting_tts = self._finite_number(waiting_tts, "waiting_tts")
        arm_tts = self._finite_number(arm_tts, "arm_tts")
        minimum_arm_tts = self._finite_number(
            minimum_arm_tts,
            "minimum_arm_tts",
        )
        maximum_policy_tts = self._finite_number(
            maximum_policy_tts,
            "maximum_policy_tts",
        )
        commit_time_to_strike_s = self._finite_number(
            commit_time_to_strike_s,
            "commit_time_to_strike_s",
        )
        maximum_racket_target_override_delta_m = self._finite_number(
            maximum_racket_target_override_delta_m,
            "maximum_racket_target_override_delta_m",
        )
        maximum_racket_velocity_override_delta_mps = self._finite_number(
            maximum_racket_velocity_override_delta_mps,
            "maximum_racket_velocity_override_delta_mps",
        )
        maximum_strike_deadline_override_delta_s = self._finite_number(
            maximum_strike_deadline_override_delta_s,
            "maximum_strike_deadline_override_delta_s",
        )
        if type(armed_cancel_consecutive_failures) is not int:
            raise TypeError(
                "armed_cancel_consecutive_failures must be an integer"
            )
        if armed_cancel_consecutive_failures <= 0:
            raise ValueError(
                "armed_cancel_consecutive_failures must be positive"
            )
        if not callable(swing_duration_sampler):
            raise TypeError("swing_duration_sampler must be callable")
        if not (
            0.0
            <= commit_time_to_strike_s
            <= minimum_arm_tts
            <= arm_tts
            <= maximum_policy_tts
        ):
            raise ValueError(
                "expected 0 <= commit <= minimum_arm <= arm <= maximum_policy"
            )
        if not 0.0 <= waiting_tts <= maximum_policy_tts:
            raise ValueError(
                "expected 0 <= waiting_tts <= maximum_policy_tts"
            )
        for name, value in (
            (
                "maximum_racket_target_override_delta_m",
                maximum_racket_target_override_delta_m,
            ),
            (
                "maximum_racket_velocity_override_delta_mps",
                maximum_racket_velocity_override_delta_mps,
            ),
            (
                "maximum_strike_deadline_override_delta_s",
                maximum_strike_deadline_override_delta_s,
            ),
        ):
            if value < 0.0:
                raise ValueError(f"{name} must be non-negative")

        self.waiting_tts = waiting_tts
        self.arm_tts = arm_tts
        self.minimum_arm_tts = minimum_arm_tts
        self.maximum_policy_tts = maximum_policy_tts
        self.armed_cancel_consecutive_failures = (
            armed_cancel_consecutive_failures
        )
        self.commit_time_to_strike_s = commit_time_to_strike_s
        self.maximum_racket_target_override_delta_m = (
            maximum_racket_target_override_delta_m
        )
        self.maximum_racket_velocity_override_delta_mps = (
            maximum_racket_velocity_override_delta_mps
        )
        self.maximum_strike_deadline_override_delta_s = (
            maximum_strike_deadline_override_delta_s
        )
        self.swing_duration_sampler = swing_duration_sampler

        self.phase = CommandPhase.WAITING
        self.active_result: PlannerResultSnapshot | None = None
        self.recovery_duration_s = 0.0
        self.command_end_deadline_s: float | None = None
        self._current_track_id: int | None = None
        self.consecutive_failure_count = 0
        self._recovery_pending_track_id: int | None = None
        self._recovery_pending_result: PlannerResultSnapshot | None = None

        self.consumed_track_ids: set[int] = set()
        self.last_consumption_reason_by_track_id: dict[int, str] = {}
        self.generation_watermark_by_track_id: dict[int, int] = {}
        self.strike_count_by_track_id: dict[int, int] = {}

        self.locked_track_id: int | None = None
        self.locked_strike_type: str | None = None
        self.locked_base_target_xy: np.ndarray | None = None
        self.locked_strike_deadline_monotonic_s: float | None = None
        self.last_failure_reason: LifecycleCancelReason | None = None
        self.last_cancel_reason: LifecycleCancelReason | None = None

    @staticmethod
    def _finite_number(value: object, name: str) -> float:
        if isinstance(value, (bool, np.bool_)) or not isinstance(
            value,
            (int, float, np.integer, np.floating),
        ):
            raise TypeError(f"{name} must be a real number")
        result = float(value)
        if not np.isfinite(result):
            raise ValueError(f"{name} must be finite")
        return result

    @classmethod
    def _now(cls, value: object) -> float:
        now = cls._finite_number(value, "now")
        if now < 0.0:
            raise ValueError("now must be non-negative")
        return now

    @staticmethod
    def _track_id(value: object) -> int:
        if type(value) is not int or value <= 0:
            raise ValueError("track_id must be a positive integer")
        return value

    @staticmethod
    def _read_only_vector(value: object, *, size: int, name: str) -> np.ndarray:
        try:
            array = np.asarray(value, dtype=np.float64)
        except Exception as exc:
            raise ValueError(f"{name} must be a finite vector") from exc
        if array.shape != (size,) or not np.isfinite(array).all():
            raise ValueError(f"{name} must be a finite {size}-vector")
        copied = array.copy()
        copied.setflags(write=False)
        return copied

    def _freeze_success_result(
        self,
        result: PlannerResultSnapshot,
    ) -> PlannerResultSnapshot:
        command = result.command
        if not isinstance(command, HitterWbcCommand):
            raise TypeError("planner command must be HitterWbcCommand")
        plan = command.strike_plan
        if not isinstance(plan, StrikePlan):
            raise TypeError("planner strike_plan must be StrikePlan")
        if command.strike_type not in ("forehand", "backhand"):
            raise ValueError("planner strike_type is invalid")
        if command.strike_side_source not in {"table_y", "forced"}:
            raise ValueError(
                "planner strike_side_source must be table_y or forced"
            )

        time_to_strike = self._finite_number(
            command.time_to_strike,
            "command.time_to_strike",
        )
        t_strike = self._finite_number(plan.t_strike, "strike_plan.t_strike")
        strike_table_y_w = self._finite_number(
            command.strike_table_y_w,
            "command.strike_table_y_w",
        )
        if time_to_strike < 0.0 or t_strike < 0.0:
            raise ValueError("planner strike timing must be non-negative")

        raw_normal_speed = self._finite_number(
            plan.raw_racket_normal_speed_mps,
            "strike_plan.raw_racket_normal_speed_mps",
        )
        commanded_normal_speed = self._finite_number(
            plan.commanded_racket_normal_speed_mps,
            "strike_plan.commanded_racket_normal_speed_mps",
        )
        minimum_normal_speed = self._finite_number(
            plan.minimum_racket_normal_speed_mps,
            "strike_plan.minimum_racket_normal_speed_mps",
        )
        if minimum_normal_speed < 0.0:
            raise ValueError(
                "strike_plan.minimum_racket_normal_speed_mps "
                "must be non-negative"
            )
        floor_applied = plan.racket_speed_floor_applied
        if type(floor_applied) is not bool:
            raise TypeError(
                "strike_plan.racket_speed_floor_applied must be bool"
            )
        expected_commanded_speed = (
            max(raw_normal_speed, minimum_normal_speed)
            if minimum_normal_speed > 0.0
            else raw_normal_speed
        )
        expected_floor_applied = bool(
            minimum_normal_speed > 0.0
            and raw_normal_speed < minimum_normal_speed
        )
        if not np.isclose(
            commanded_normal_speed,
            expected_commanded_speed,
            rtol=1.0e-9,
            atol=1.0e-9,
        ) or floor_applied is not expected_floor_applied:
            raise ValueError(
                "strike_plan racket speed diagnostics are inconsistent"
            )

        base = self._read_only_vector(
            command.p_base_target_xy,
            size=2,
            name="command.p_base_target_xy",
        )
        position = self._read_only_vector(
            plan.p_racket_target,
            size=3,
            name="strike_plan.p_racket_target",
        )
        command_velocity = self._read_only_vector(
            command.v_racket_target_w,
            size=3,
            name="command.v_racket_target_w",
        )
        plan_velocity = self._read_only_vector(
            plan.v_racket_target,
            size=3,
            name="strike_plan.v_racket_target",
        )
        if not np.array_equal(command_velocity, plan_velocity):
            raise ValueError("planner racket velocity fields disagree")
        if strike_table_y_w != float(position[1]):
            raise ValueError("planner strike table y disagrees with position")
        if command.strike_side_source == "table_y":
            expected_strike_type = (
                "forehand" if strike_table_y_w < 0.0 else "backhand"
            )
            if command.strike_type != expected_strike_type:
                raise ValueError(
                    "planner table-y strike type disagrees with strike table y"
                )
        frozen_plan = replace(
            plan,
            t_strike=t_strike,
            p_racket_target=position,
            v_racket_target=plan_velocity,
            v_ball_in=self._read_only_vector(
                plan.v_ball_in,
                size=3,
                name="strike_plan.v_ball_in",
            ),
            v_ball_out=self._read_only_vector(
                plan.v_ball_out,
                size=3,
                name="strike_plan.v_ball_out",
            ),
            raw_racket_normal_speed_mps=raw_normal_speed,
            commanded_racket_normal_speed_mps=commanded_normal_speed,
            minimum_racket_normal_speed_mps=minimum_normal_speed,
            racket_speed_floor_applied=floor_applied,
        )
        frozen_command = replace(
            command,
            p_base_target_xy=base,
            v_racket_target_w=command_velocity,
            time_to_strike=time_to_strike,
            strike_plan=frozen_plan,
            strike_table_y_w=strike_table_y_w,
        )
        return replace(result, command=frozen_command)

    def _decision(
        self,
        kind: str,
        *,
        track_id: int | None = None,
        command_changed: bool = False,
        entered_waiting: bool = False,
        cancel_reason: LifecycleCancelReason | None = None,
        consumed_track_ids: tuple[int, ...] = (),
    ) -> LifecycleDecision:
        return LifecycleDecision(
            kind=kind,
            track_id=track_id,
            command_changed=command_changed,
            entered_waiting=entered_waiting,
            cancel_reason=cancel_reason,
            consumed_track_ids=consumed_track_ids,
        )

    def consume_track(self, track_id: int, *, reason: str) -> None:
        identity = self._track_id(track_id)
        if type(reason) is not str or not reason:
            raise ValueError("reason must be a non-empty string")
        self.consumed_track_ids.add(identity)
        self.last_consumption_reason_by_track_id[identity] = reason

    def _consume_many(
        self,
        track_ids: set[int],
        *,
        reason: str,
    ) -> tuple[int, ...]:
        ordered = tuple(sorted(track_ids))
        for track_id in ordered:
            self.consume_track(track_id, reason=reason)
        return ordered

    def _clear_transient(self) -> None:
        self.active_result = None
        self.command_end_deadline_s = None
        self.recovery_duration_s = 0.0
        self._current_track_id = None
        self.consecutive_failure_count = 0
        self._clear_recovery_pending()

    def _clear_recovery_pending(self) -> None:
        self._recovery_pending_track_id = None
        self._recovery_pending_result = None

    def reset_for_policy_reentry(self, *, now: float) -> LifecycleDecision:
        self._now(now)
        active_id = (
            None
            if self.active_result is None
            else self.active_result.track_id
        )
        current_id = active_id if active_id is not None else self._current_track_id
        consumed = self._consume_many(
            {
                track_id
                for track_id in (
                    current_id,
                    self._recovery_pending_track_id,
                )
                if track_id is not None
            },
            reason="policy_reentry",
        )
        self._clear_transient()
        self.phase = CommandPhase.WAITING
        return self._decision(
            "session_reset",
            track_id=current_id,
            command_changed=active_id is not None,
            entered_waiting=True,
            consumed_track_ids=consumed,
        )

    def ingest(
        self,
        result: PlannerResultSnapshot,
        *,
        now: float,
    ) -> LifecycleDecision:
        if not isinstance(result, PlannerResultSnapshot):
            raise TypeError("result must be a PlannerResultSnapshot")
        now = self._now(now)
        track_id = self._track_id(result.track_id)
        generation = result.source_generation

        if track_id in self.consumed_track_ids:
            return self._decision(
                "ignored_consumed",
                track_id=track_id,
                consumed_track_ids=(track_id,),
            )

        watermark = self.generation_watermark_by_track_id.get(track_id)
        if watermark is not None and generation <= watermark:
            return self._decision("ignored_generation", track_id=track_id)
        self.generation_watermark_by_track_id[track_id] = generation

        if self.phase is CommandPhase.RECOVERY:
            return self._ingest_recovery(result, now)

        if self._current_track_id is None:
            self._current_track_id = track_id
            if self.phase is CommandPhase.WAITING:
                self.phase = CommandPhase.TRACKING
        elif track_id != self._current_track_id:
            consumed = self._consume_many(
                {track_id},
                reason=f"challenger_during_{self.phase.value}",
            )
            return self._decision(
                "ignored_track",
                track_id=track_id,
                consumed_track_ids=consumed,
            )

        if self.phase is CommandPhase.ARMED:
            return self._ingest_armed(result, now)
        return self._ingest_unarmed(result, now)

    def mark_track_ended(
        self,
        track_id: int,
        *,
        now: float,
    ) -> LifecycleDecision:
        return self.cancel(
            reason=LifecycleCancelReason.TRACK_ENDED,
            now=now,
            track_id=self._track_id(track_id),
        )

    def advance(self, now: float) -> LifecycleDecision:
        now = self._now(now)
        if (
            self.phase is CommandPhase.ARMED
            and self.active_result is not None
            and self.locked_strike_deadline_monotonic_s is not None
            and now >= self.locked_strike_deadline_monotonic_s
        ):
            track_id = self.active_result.track_id
            consumed = self._consume_many({track_id}, reason="struck")
            if self.strike_count_by_track_id.get(track_id, 0) == 0:
                self.strike_count_by_track_id[track_id] = 1
            self.phase = CommandPhase.RECOVERY
            self.consecutive_failure_count = 0
            return self._decision(
                "struck",
                track_id=track_id,
                consumed_track_ids=consumed,
            )

        if (
            self.phase is CommandPhase.RECOVERY
            and self.command_end_deadline_s is not None
            and now >= self.command_end_deadline_s
        ):
            previous_id = (
                None
                if self.active_result is None
                else self.active_result.track_id
            )
            pending_result = self._recovery_pending_result
            pending_consumed = bool(
                pending_result is not None
                and pending_result.track_id in self.consumed_track_ids
            )
            self._clear_transient()
            self.phase = CommandPhase.WAITING
            if pending_result is not None:
                if pending_consumed:
                    return self._decision(
                        "ignored_consumed",
                        track_id=pending_result.track_id,
                        entered_waiting=True,
                        consumed_track_ids=(pending_result.track_id,),
                    )
                self._current_track_id = pending_result.track_id
                continued = self._ingest_unarmed(pending_result, now)
                return replace(continued, entered_waiting=True)
            return self._decision(
                "entered_waiting",
                track_id=previous_id,
                entered_waiting=True,
            )
        return self._decision("none")

    def policy_tts(self, *, now: float) -> float:
        now = self._now(now)
        if self.phase in (CommandPhase.WAITING, CommandPhase.TRACKING):
            return self.waiting_tts
        if self.locked_strike_deadline_monotonic_s is None:
            return self.waiting_tts
        remaining = self.locked_strike_deadline_monotonic_s - now
        return float(np.clip(remaining, 0.0, self.maximum_policy_tts))

    def _ingest_recovery(
        self,
        result: PlannerResultSnapshot,
        now: float,
    ) -> LifecycleDecision:
        track_id = result.track_id
        pending_id = self._recovery_pending_track_id
        if pending_id is not None and track_id != pending_id:
            consumed = self._consume_many(
                {track_id},
                reason="challenger_during_recovery",
            )
            return self._decision(
                "ignored_track",
                track_id=track_id,
                consumed_track_ids=consumed,
            )

        if result.failure_reason is not None:
            reason = LifecycleCancelReason(result.failure_reason.value)
            self.last_failure_reason = reason
            if reason in self._IMMEDIATE_FAILURES:
                return self.cancel(reason=reason, now=now, track_id=track_id)
            return self._decision(
                "retained_recovery",
                track_id=track_id,
                cancel_reason=reason,
            )

        try:
            frozen = self._freeze_success_result(result)
        except Exception:
            reason = LifecycleCancelReason.INTERNAL_ERROR
            self.last_failure_reason = reason
            return self.cancel(reason=reason, now=now, track_id=track_id)
        if pending_id is None:
            self._recovery_pending_track_id = track_id
        self._recovery_pending_result = frozen
        return self._decision("cached_recovery", track_id=track_id)

    def _ingest_unarmed(
        self,
        result: PlannerResultSnapshot,
        now: float,
    ) -> LifecycleDecision:
        track_id = result.track_id
        if result.failure_reason is not None:
            reason = LifecycleCancelReason(result.failure_reason.value)
            self.last_failure_reason = reason
            if reason in self._IMMEDIATE_FAILURES:
                return self.cancel(reason=reason, now=now, track_id=track_id)
            return self._decision(
                "retained_failure",
                track_id=track_id,
                cancel_reason=reason,
            )

        try:
            frozen = self._freeze_success_result(result)
        except Exception:
            reason = LifecycleCancelReason.INTERNAL_ERROR
            self.last_failure_reason = reason
            return self.cancel(reason=reason, now=now, track_id=track_id)

        deadline = frozen.strike_deadline_monotonic_s
        if now > deadline - self.minimum_arm_tts:
            consumed = self._consume_many({track_id}, reason="skipped_late")
            self._clear_transient()
            self.phase = CommandPhase.WAITING
            return self._decision(
                "skipped",
                track_id=track_id,
                entered_waiting=True,
                consumed_track_ids=consumed,
            )
        if now >= deadline - self.arm_tts:
            return self._arm(frozen, now)
        self.phase = CommandPhase.TRACKING
        return self._decision("tracking", track_id=track_id)

    def _ingest_armed(
        self,
        result: PlannerResultSnapshot,
        now: float,
    ) -> LifecycleDecision:
        active = self.active_result
        if active is None:
            return self._decision("ignored_track", track_id=result.track_id)
        if result.track_id != active.track_id:
            return self._decision("ignored_track", track_id=result.track_id)

        if result.failure_reason is not None:
            reason = LifecycleCancelReason(result.failure_reason.value)
            self.last_failure_reason = reason
            if self._is_committed(now):
                return self._decision(
                    "retained_committed",
                    track_id=result.track_id,
                    cancel_reason=reason,
                )
            return self._handle_armed_failure(reason, now=now)

        try:
            candidate = self._freeze_success_result(result)
        except Exception:
            reason = LifecycleCancelReason.INTERNAL_ERROR
            self.last_failure_reason = reason
            return self.cancel(reason=reason, now=now)

        if self._is_committed(now):
            return self._decision(
                "retained_committed",
                track_id=result.track_id,
            )

        active_command = active.command
        candidate_command = candidate.command
        position_delta = float(
            np.linalg.norm(
                candidate_command.strike_plan.p_racket_target
                - active_command.strike_plan.p_racket_target
            )
        )
        velocity_delta = float(
            np.linalg.norm(
                candidate_command.v_racket_target_w
                - active_command.v_racket_target_w
            )
        )
        deadline_delta = abs(
            candidate.strike_deadline_monotonic_s
            - self.locked_strike_deadline_monotonic_s
        )
        continuous = (
            candidate_command.strike_type == self.locked_strike_type
            and candidate_command.strike_side_source
            == active_command.strike_side_source
            and position_delta
            <= self.maximum_racket_target_override_delta_m
            and velocity_delta
            <= self.maximum_racket_velocity_override_delta_mps
            and deadline_delta
            <= self.maximum_strike_deadline_override_delta_s
        )
        if not continuous:
            self.last_failure_reason = (
                LifecycleCancelReason.OVERRIDE_DISCONTINUITY
            )
            return self._decision(
                "retained_discontinuity",
                track_id=result.track_id,
                cancel_reason=LifecycleCancelReason.OVERRIDE_DISCONTINUITY,
            )

        new_position = self._read_only_vector(
            candidate_command.strike_plan.p_racket_target,
            size=3,
            name="strike_plan.p_racket_target",
        )
        new_velocity = self._read_only_vector(
            candidate_command.v_racket_target_w,
            size=3,
            name="command.v_racket_target_w",
        )
        candidate_plan = candidate_command.strike_plan
        locked_plan = replace(
            active_command.strike_plan,
            p_racket_target=new_position,
            v_racket_target=new_velocity,
            v_ball_in=candidate_plan.v_ball_in,
            v_ball_out=candidate_plan.v_ball_out,
            raw_racket_normal_speed_mps=(
                candidate_plan.raw_racket_normal_speed_mps
            ),
            commanded_racket_normal_speed_mps=(
                candidate_plan.commanded_racket_normal_speed_mps
            ),
            minimum_racket_normal_speed_mps=(
                candidate_plan.minimum_racket_normal_speed_mps
            ),
            racket_speed_floor_applied=(
                candidate_plan.racket_speed_floor_applied
            ),
        )
        locked_command = replace(
            active_command,
            v_racket_target_w=new_velocity,
            strike_plan=locked_plan,
            strike_table_y_w=float(new_position[1]),
        )
        self.active_result = replace(
            candidate,
            strike_deadline_monotonic_s=(
                self.locked_strike_deadline_monotonic_s
            ),
            command=locked_command,
        )
        self.consecutive_failure_count = 0
        return self._decision(
            "overridden",
            track_id=result.track_id,
            command_changed=True,
        )

    def _handle_armed_failure(
        self,
        reason: LifecycleCancelReason,
        *,
        now: float,
    ) -> LifecycleDecision:
        if reason in self._IMMEDIATE_FAILURES:
            return self.cancel(reason=reason, now=now)
        if reason in self._RETAIN_ONLY_FAILURES:
            active_id = (
                None
                if self.active_result is None
                else self.active_result.track_id
            )
            return self._decision(
                "retained_failure",
                track_id=active_id,
                cancel_reason=reason,
            )
        if reason not in self._SOFT_FAILURES:
            return self.cancel(
                reason=LifecycleCancelReason.INTERNAL_ERROR,
                now=now,
            )
        self.consecutive_failure_count += 1
        if (
            self.consecutive_failure_count
            >= self.armed_cancel_consecutive_failures
        ):
            return self.cancel(reason=reason, now=now)
        active_id = (
            None if self.active_result is None else self.active_result.track_id
        )
        return self._decision(
            "retained_failure",
            track_id=active_id,
            cancel_reason=reason,
        )

    def _is_committed(self, now: float) -> bool:
        return (
            self.locked_strike_deadline_monotonic_s is not None
            and now
            >= self.locked_strike_deadline_monotonic_s
            - self.commit_time_to_strike_s
        )

    def _arm(
        self,
        result: PlannerResultSnapshot,
        now: float,
    ) -> LifecycleDecision:
        try:
            sampled_duration = self.swing_duration_sampler()
            total_swing_duration = self._finite_number(
                sampled_duration,
                "swing_duration_sampler result",
            )
            if total_swing_duration < 0.0:
                raise ValueError("swing duration must be non-negative")
        except Exception:
            reason = LifecycleCancelReason.INTERNAL_ERROR
            self.last_failure_reason = reason
            return self.cancel(
                reason=reason,
                now=now,
                track_id=result.track_id,
            )

        arm_tts = max(result.strike_deadline_monotonic_s - now, 0.0)
        self.recovery_duration_s = max(
            total_swing_duration - arm_tts,
            0.0,
        )
        self.locked_track_id = result.track_id
        self.locked_strike_type = result.command.strike_type
        self.locked_base_target_xy = self._read_only_vector(
            result.command.p_base_target_xy,
            size=2,
            name="command.p_base_target_xy",
        )
        self.locked_strike_deadline_monotonic_s = (
            result.strike_deadline_monotonic_s
        )
        self.command_end_deadline_s = (
            self.locked_strike_deadline_monotonic_s
            + self.recovery_duration_s
        )
        self.active_result = result
        self.consecutive_failure_count = 0
        self.phase = CommandPhase.ARMED
        return self._decision(
            "armed",
            track_id=result.track_id,
            command_changed=True,
        )

    def cancel(
        self,
        *,
        reason: LifecycleCancelReason,
        now: float,
        track_id: int | None = None,
    ) -> LifecycleDecision:
        if not isinstance(reason, LifecycleCancelReason):
            raise TypeError("reason must be a LifecycleCancelReason")
        now = self._now(now)
        event_id = None if track_id is None else self._track_id(track_id)
        pending_id = self._recovery_pending_track_id
        active_id = (
            None
            if self.active_result is None
            else self.active_result.track_id
        )
        current_id = (
            active_id
            if active_id is not None
            else (
                self._current_track_id
                if self.phase is CommandPhase.TRACKING
                else None
            )
        )
        ids = {
            identity
            for identity in (current_id, event_id)
            if identity is not None
        }
        clear_recovery_pending = bool(
            self.phase is CommandPhase.RECOVERY
            and pending_id is not None
            and (
                reason is not LifecycleCancelReason.TRACK_ENDED
                or event_id == pending_id
            )
        )
        if clear_recovery_pending:
            ids.add(pending_id)
        introduced_new_identity = any(
            identity not in self.consumed_track_ids for identity in ids
        )
        consumed = self._consume_many(ids, reason=reason.value)
        self.last_cancel_reason = reason

        if self.phase is CommandPhase.RECOVERY:
            if clear_recovery_pending:
                self._clear_recovery_pending()
            return self._decision(
                "retained_recovery",
                track_id=(event_id if event_id is not None else active_id),
                cancel_reason=reason,
                consumed_track_ids=consumed,
            )
        if (
            self.phase is CommandPhase.ARMED
            and active_id is not None
            and self._is_committed(now)
        ):
            return self._decision(
                "retained_committed",
                track_id=active_id,
                cancel_reason=reason,
                consumed_track_ids=consumed,
            )

        entered_waiting = (
            self.phase is not CommandPhase.WAITING
            or introduced_new_identity
        )
        command_changed = active_id is not None
        cancelled_id = current_id if current_id is not None else event_id
        self._clear_transient()
        self.phase = CommandPhase.WAITING
        return self._decision(
            "cancelled",
            track_id=cancelled_id,
            command_changed=command_changed,
            entered_waiting=entered_waiting,
            cancel_reason=reason,
            consumed_track_ids=consumed,
        )


__all__ = [
    "BallEstimateSnapshot",
    "CommandPhase",
    "CompletedResultBatch",
    "FrozenPlannerResult",
    "HitterCommandLifecycle",
    "IncomingTrackConfirmation",
    "IncomingTrackSnapshot",
    "LatestOnlyPlannerWorker",
    "LifecycleCancelReason",
    "LifecycleDecision",
    "PlannerResultSnapshot",
    "PlannerWorkerStats",
    "PlannerWorkerTrace",
    "freeze_planner_command_fields",
]
