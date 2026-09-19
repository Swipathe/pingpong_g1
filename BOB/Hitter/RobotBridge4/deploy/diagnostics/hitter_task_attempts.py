from __future__ import annotations

from collections import OrderedDict, deque
from dataclasses import dataclass, field, replace
from functools import wraps
import threading
from typing import Callable, Deque, Dict, List, Mapping, Optional, Tuple, TypeVar

from diagnostics.hitter_task_models import (
    AttemptBinding,
    AttemptSummary,
    AttemptTransition,
    JsonValue,
    NormalizedMocapSample,
    SnapshotKey,
)

_ReturnT = TypeVar("_ReturnT")


def _synchronized(method: Callable[..., _ReturnT]) -> Callable[..., _ReturnT]:
    @wraps(method)
    def locked(self: "AttemptTracker", *args: object, **kwargs: object) -> _ReturnT:
        with self._lock:
            return method(self, *args, **kwargs)

    return locked

_PLANNER_REASONS = {
    "BALL_X_NOT_AHEAD",
    "BALL_NOT_INCOMING",
    "NO_DIRECTED_CROSSING",
    "NO_FUTURE_CROSSING",
    "HIT_HEIGHT_OUT_OF_RANGE",
    "NONFINITE_TRAJECTORY",
    "NONFINITE_INPUT_OR_OUTPUT",
    "ESTIMATOR_NOT_READY",
    "MALFORMED_COMMAND",
    "PLANNER_EXCEPTION",
    "INTERNAL_ERROR",
    "NO_VALID_PLAN",
}

_HARD_FAILURE_REASONS = {
    "MALFORMED_COMMAND",
    "PELVIS_UNAVAILABLE",
    "BASE_POSE_INVALID",
}


@dataclass
class _AttemptState:
    attempt_id: int
    track_id: int
    identity_source: str
    ready: bool = False
    confirmed: bool = False
    planned: bool = False
    armed: bool = False
    passed: bool = False
    late_skip: bool = False
    last_planner_reason: Optional[str] = None
    last_obs_failure: Optional[str] = None
    last_hard_failure: Optional[str] = None
    closed: bool = False
    terminal_status: Optional[str] = None
    terminal_blocker: Optional[str] = None
    terminal_stage: Optional[str] = None
    timeline: Deque[AttemptTransition] = field(default_factory=deque)


@dataclass
class _SegmentState:
    attempt_id: int
    track_segment_id: int
    role: str
    open: bool = True


class AttemptTracker:
    """Pure display grouping for production-owned snapshots and results."""

    def __init__(
        self,
        *,
        reacquire_grace_s: float = 0.20,
        max_attempts: int = 100,
        max_bindings: int = 8192,
        max_timeline_per_attempt: int = 4096,
    ) -> None:
        if reacquire_grace_s < 0.0:
            raise ValueError("reacquire_grace_s must be non-negative")
        if max_attempts <= 0 or max_bindings <= 0 or max_timeline_per_attempt <= 0:
            raise ValueError("attempt tracker bounds must be positive")
        self._lock = threading.RLock()
        self._reacquire_grace_s = float(reacquire_grace_s)
        self._max_attempts = int(max_attempts)
        self._max_bindings = int(max_bindings)
        self._max_timeline_per_attempt = int(max_timeline_per_attempt)
        self._next_attempt_id = 1
        self._next_segment_id = 1
        self._attempts: "OrderedDict[int, _AttemptState]" = OrderedDict()
        self._bindings: "OrderedDict[SnapshotKey, AttemptBinding]" = OrderedDict()
        self._attempt_id_by_track_id: Dict[int, int] = {}
        self._active_attempt_id: Optional[int] = None
        self._current_segment: Optional[_SegmentState] = None
        self._latest_binding: Optional[AttemptBinding] = None
        self._grace_deadline_s: Optional[float] = None
        self._pending_reset_track_id: Optional[int] = None
        self._pending_reset_segment: Optional[_SegmentState] = None
        self._last_lifecycle_stage: Dict[int, str] = {}

    @_synchronized
    def observe_ball_sample(
        self,
        sample: NormalizedMocapSample,
        *,
        snapshot_key: SnapshotKey,
    ) -> Tuple[AttemptTransition, ...]:
        """Update display grouping only; never mutate estimator/incoming/lifecycle."""
        sample = self._sample_with_snapshot_identity(sample, snapshot_key)
        transitions: List[AttemptTransition] = list(
            self._expire_grace(sample.received_monotonic_s)
        )
        visible = bool(sample.valid and not sample.occluded)
        if visible:
            if self._active_attempt_id is None:
                if not self._activate_attempt_for_track(sample):
                    return tuple(transitions)
                transitions.append(
                    self._start_segment(
                        snapshot_key=snapshot_key,
                        monotonic_s=sample.received_monotonic_s,
                    )
                )
            elif self._current_segment is None or not self._current_segment.open:
                transitions.append(
                    self._start_segment(
                        snapshot_key=snapshot_key,
                        monotonic_s=sample.received_monotonic_s,
                    )
                )
            else:
                self._store_binding(snapshot_key, self._current_segment)
            self._grace_deadline_s = None
            self._pending_reset_track_id = None
            self._pending_reset_segment = None
            return tuple(transitions)

        if self._active_attempt_id is None:
            return tuple(transitions)

        segment = self._segment_for_invisible(snapshot_key)
        if segment is not None:
            self._store_binding(snapshot_key, segment)
        if self._current_segment is not None and self._current_segment.open:
            transitions.append(
                self._end_segment(
                    reason_code="BALL_INVALID_OR_OCCLUDED",
                    monotonic_s=sample.received_monotonic_s,
                    snapshot_key=snapshot_key,
                )
            )
        return tuple(transitions)

    @_synchronized
    def observe_production_reset(
        self,
        *,
        previous_track_id: int,
        new_track_id: int,
        reason: str,
        monotonic_s: float,
    ) -> Tuple[AttemptTransition, ...]:
        """End the production segment on invalid or strike-deadline reset."""
        transitions: List[AttemptTransition] = list(self._expire_grace(monotonic_s))
        if self._active_attempt_id is None or self._current_segment is None:
            return tuple(transitions)
        self._pending_reset_track_id = new_track_id
        self._pending_reset_segment = self._current_segment
        if self._current_segment.open:
            transitions.append(
                self._end_segment(
                    reason_code=reason,
                    monotonic_s=monotonic_s,
                    snapshot_key=None,
                    values={
                        "previous_track_id": previous_track_id,
                        "new_track_id": new_track_id,
                    },
                )
            )
        return tuple(transitions)

    @_synchronized
    def bind_snapshot(
        self,
        *,
        snapshot_key: SnapshotKey,
    ) -> Optional[AttemptBinding]:
        """Return exact ownership established when the snapshot was observed."""
        binding = self._bindings.get(snapshot_key)
        if binding is not None:
            self._bindings.move_to_end(snapshot_key)
        return binding

    @_synchronized
    def binding_for_result(
        self,
        snapshot_key: SnapshotKey,
    ) -> Optional[AttemptBinding]:
        """Look up immutable async-result ownership."""
        binding = self._bindings.get(snapshot_key)
        if binding is not None:
            self._bindings.move_to_end(snapshot_key)
        return binding

    @_synchronized
    def record_stage(
        self,
        *,
        binding: AttemptBinding,
        stage: str,
        monotonic_s: float,
        reason_code: Optional[str],
        values: Mapping[str, JsonValue],
    ) -> AttemptTransition:
        """Append a stage/rejection without changing production state."""
        self._validate_binding(binding)
        transition = AttemptTransition(
            attempt_id=binding.attempt_id,
            track_segment_id=binding.track_segment_id,
            stage=stage,
            monotonic_s=monotonic_s,
            snapshot_key=binding.snapshot_key,
            reason_code=reason_code,
            values=values,
        )
        attempt = self._attempts[binding.attempt_id]
        attempt.timeline.append(transition)
        if not attempt.closed:
            self._update_progress(attempt, stage=stage, reason_code=reason_code)
        return transition

    @_synchronized
    def record_task_pass(
        self,
        *,
        binding: AttemptBinding,
        monotonic_s: float,
    ) -> AttemptTransition:
        """Mark first task PASS and make later reacquisition a post-deadline tail."""
        return self.record_stage(
            binding=binding,
            stage="TASK_OBS_PASS",
            monotonic_s=monotonic_s,
            reason_code=None,
            values={},
        )

    @_synchronized
    def record_health_warning(
        self,
        *,
        reason_code: str,
        monotonic_s: float,
        values: Mapping[str, JsonValue],
    ) -> Optional[AttemptTransition]:
        """Record warning only; never open/close attempts."""
        if self._active_attempt_id is None:
            return None
        segment_id = (
            None
            if self._current_segment is None
            else self._current_segment.track_segment_id
        )
        snapshot_key = (
            None if self._latest_binding is None else self._latest_binding.snapshot_key
        )
        transition = AttemptTransition(
            attempt_id=self._active_attempt_id,
            track_segment_id=segment_id,
            stage="WARNING",
            monotonic_s=monotonic_s,
            snapshot_key=snapshot_key,
            reason_code=reason_code,
            values=values,
        )
        self._attempts[self._active_attempt_id].timeline.append(transition)
        return transition

    @_synchronized
    def set_lifecycle_context(
        self,
        *,
        phase: str,
        active_key: Optional[SnapshotKey],
        cached_key: Optional[SnapshotKey],
        monotonic_s: float,
    ) -> Tuple[AttemptTransition, ...]:
        """Project recovery/cached status onto the owning attempt."""
        if phase.upper() != "RECOVERY" or self._active_attempt_id is None:
            return ()
        current_id = self._active_attempt_id
        active = None if active_key is None else self._bindings.get(active_key)
        cached = None if cached_key is None else self._bindings.get(cached_key)
        if active is not None and active.attempt_id == current_id:
            return ()
        if cached is not None and cached.attempt_id == current_id:
            stage = "CACHED_DURING_RECOVERY"
            binding = cached
        else:
            stage = "WAITING_FOR_PREVIOUS_RECOVERY"
            binding = self._latest_binding
        if self._last_lifecycle_stage.get(current_id) == stage:
            return ()
        self._last_lifecycle_stage[current_id] = stage
        transition = AttemptTransition(
            attempt_id=current_id,
            track_segment_id=(
                None if binding is None else binding.track_segment_id
            ),
            stage=stage,
            monotonic_s=monotonic_s,
            snapshot_key=None if binding is None else binding.snapshot_key,
            reason_code=None,
            values={
                "active_key": None if active_key is None else active_key.to_json_dict(),
                "cached_key": None if cached_key is None else cached_key.to_json_dict(),
            },
        )
        self._attempts[current_id].timeline.append(transition)
        return (transition,)

    @_synchronized
    def advance(
        self,
        *,
        now_monotonic_s: float,
    ) -> Tuple[AttemptTransition, ...]:
        """Close expired reacquire grace without changing production state."""
        return self._expire_grace(now_monotonic_s)

    @_synchronized
    def current_summary(self) -> Optional[AttemptSummary]:
        """Return an immutable summary for the display-owned active attempt."""
        if self._active_attempt_id is None:
            return None
        attempt = self._attempts.get(self._active_attempt_id)
        return None if attempt is None else self._summary(attempt)

    @_synchronized
    def summary_for_attempt(self, attempt_id: int) -> Optional[AttemptSummary]:
        """Return an immutable cached summary without exposing tracker state."""
        attempt = self._attempts.get(attempt_id)
        return None if attempt is None else self._summary(attempt)

    @_synchronized
    def timeline_for_attempt(
        self,
        attempt_id: int,
    ) -> Tuple[AttemptTransition, ...]:
        """Return an immutable copy of the bounded attempt timeline."""
        attempt = self._attempts.get(attempt_id)
        return () if attempt is None else tuple(attempt.timeline)

    def _activate_attempt_for_track(self, sample: NormalizedMocapSample) -> bool:
        track_id = int(sample.track_id)
        if track_id <= 0:
            raise ValueError("visible ball samples require a positive track_id")
        attempt_id = self._attempt_id_by_track_id.get(track_id)
        if attempt_id is None or attempt_id not in self._attempts:
            attempt_id = self._next_attempt_id
            self._next_attempt_id += 1
            self._attempts[attempt_id] = _AttemptState(
                attempt_id=attempt_id,
                track_id=track_id,
                identity_source=sample.identity_source,
                timeline=deque(maxlen=self._max_timeline_per_attempt),
            )
            self._attempt_id_by_track_id[track_id] = attempt_id
        else:
            attempt = self._attempts[attempt_id]
            if attempt.closed:
                return False
        self._active_attempt_id = attempt_id
        self._trim_attempts()
        return True

    def _start_segment(
        self,
        *,
        snapshot_key: SnapshotKey,
        monotonic_s: float,
    ) -> AttemptTransition:
        if self._active_attempt_id is None:
            raise RuntimeError("cannot start a segment without an attempt")
        attempt = self._attempts[self._active_attempt_id]
        role = "POST_DEADLINE_TAIL" if attempt.passed else "PRIMARY"
        segment = _SegmentState(
            attempt_id=attempt.attempt_id,
            track_segment_id=self._next_segment_id,
            role=role,
        )
        self._next_segment_id += 1
        self._current_segment = segment
        binding = self._store_binding(snapshot_key, segment)
        stage = "POST_DEADLINE_TAIL" if role == "POST_DEADLINE_TAIL" else "DETECTED"
        transition = AttemptTransition(
            attempt_id=attempt.attempt_id,
            track_segment_id=segment.track_segment_id,
            stage=stage,
            monotonic_s=monotonic_s,
            snapshot_key=snapshot_key,
            reason_code=None,
            values={"role": role},
        )
        attempt.timeline.append(transition)
        self._latest_binding = binding
        return transition

    def _store_binding(
        self,
        snapshot_key: SnapshotKey,
        segment: _SegmentState,
    ) -> AttemptBinding:
        existing = self._bindings.get(snapshot_key)
        if existing is not None:
            self._bindings.move_to_end(snapshot_key)
            return existing
        binding = AttemptBinding(
            attempt_id=segment.attempt_id,
            track_segment_id=segment.track_segment_id,
            track_id=snapshot_key.track_id,
            identity_source=self._attempts[segment.attempt_id].identity_source,
            role=segment.role,
            snapshot_key=snapshot_key,
        )
        self._bindings[snapshot_key] = binding
        while len(self._bindings) > self._max_bindings:
            self._bindings.popitem(last=False)
        self._latest_binding = binding
        return binding

    def _segment_for_invisible(
        self,
        snapshot_key: SnapshotKey,
    ) -> Optional[_SegmentState]:
        if (
            self._pending_reset_track_id == snapshot_key.track_id
            and self._pending_reset_segment is not None
        ):
            return self._pending_reset_segment
        return self._current_segment

    def _end_segment(
        self,
        *,
        reason_code: str,
        monotonic_s: float,
        snapshot_key: Optional[SnapshotKey],
        values: Optional[Mapping[str, JsonValue]] = None,
    ) -> AttemptTransition:
        if self._current_segment is None or self._active_attempt_id is None:
            raise RuntimeError("cannot end a missing segment")
        self._current_segment.open = False
        self._grace_deadline_s = monotonic_s + self._reacquire_grace_s
        transition = AttemptTransition(
            attempt_id=self._active_attempt_id,
            track_segment_id=self._current_segment.track_segment_id,
            stage="REACQUIRE_GRACE",
            monotonic_s=monotonic_s,
            snapshot_key=snapshot_key,
            reason_code=reason_code,
            values={} if values is None else values,
        )
        self._attempts[self._active_attempt_id].timeline.append(transition)
        return transition

    def _expire_grace(self, now_monotonic_s: float) -> Tuple[AttemptTransition, ...]:
        if (
            self._grace_deadline_s is None
            or now_monotonic_s <= self._grace_deadline_s
            or self._active_attempt_id is None
        ):
            return ()
        attempt = self._attempts[self._active_attempt_id]
        blocker = self._primary_blocker(attempt)
        status = "SUCCESS" if blocker is None else "FAILED"
        transition = AttemptTransition(
            attempt_id=attempt.attempt_id,
            track_segment_id=(
                None
                if self._current_segment is None
                else self._current_segment.track_segment_id
            ),
            stage="ATTEMPT_CLOSED",
            monotonic_s=now_monotonic_s,
            snapshot_key=None,
            reason_code=blocker,
            values={"status": status, "primary_blocker": blocker},
        )
        attempt.timeline.append(transition)
        attempt.closed = True
        attempt.terminal_status = status
        attempt.terminal_blocker = blocker
        attempt.terminal_stage = transition.stage
        self._active_attempt_id = None
        self._current_segment = None
        self._latest_binding = None
        self._grace_deadline_s = None
        self._pending_reset_track_id = None
        self._pending_reset_segment = None
        self._trim_attempts()
        return (transition,)

    @staticmethod
    def _sample_with_snapshot_identity(
        sample: NormalizedMocapSample,
        snapshot_key: SnapshotKey,
    ) -> NormalizedMocapSample:
        if sample.track_id == snapshot_key.track_id:
            return sample
        if sample.track_id == 0 or sample.identity_source == "legacy_inferred":
            return replace(
                sample,
                track_id=snapshot_key.track_id,
                identity_source=(
                    "legacy_inferred"
                    if sample.identity_source == "legacy_inferred"
                    else sample.identity_source
                ),
            )
        raise ValueError("sample track_id must match snapshot_key")

    def _validate_binding(self, binding: AttemptBinding) -> None:
        known = self._bindings.get(binding.snapshot_key)
        if known != binding:
            raise ValueError("binding does not match immutable snapshot ownership")
        self._bindings.move_to_end(binding.snapshot_key)

    def _trim_attempts(self) -> None:
        while len(self._attempts) > self._max_attempts:
            evicted_id = next(
                (
                    attempt_id
                    for attempt_id, attempt in self._attempts.items()
                    if attempt.closed and attempt_id != self._active_attempt_id
                ),
                None,
            )
            if evicted_id is None:
                return
            del self._attempts[evicted_id]
            self._last_lifecycle_stage.pop(evicted_id, None)
            for track_id, attempt_id in tuple(self._attempt_id_by_track_id.items()):
                if attempt_id == evicted_id:
                    del self._attempt_id_by_track_id[track_id]
            for key in tuple(self._bindings):
                if self._bindings[key].attempt_id == evicted_id:
                    del self._bindings[key]
            if (
                self._latest_binding is not None
                and self._latest_binding.attempt_id == evicted_id
            ):
                self._latest_binding = None

    @staticmethod
    def _summary(attempt: _AttemptState) -> AttemptSummary:
        if attempt.closed:
            status = attempt.terminal_status or "FAILED"
            blocker = attempt.terminal_blocker
            stage = attempt.terminal_stage or "ATTEMPT_CLOSED"
            task_obs_status = "PASS" if status == "SUCCESS" else "FAILED"
        else:
            status = "ACTIVE"
            blocker = None
            stage = attempt.timeline[-1].stage if attempt.timeline else "WAITING"
            task_obs_status = "PASS" if attempt.passed else "PENDING"
        return AttemptSummary(
            attempt_id=attempt.attempt_id,
            track_id=attempt.track_id,
            identity_source=attempt.identity_source,
            status=status,
            stage=stage,
            primary_blocker=blocker,
            ball_speed_mps=None,
            predicted_strike_time_s=None,
            planner_tts_s=None,
            arm_tts_s=None,
            task_obs_status=task_obs_status,
            ab_summary=None,
            recording_complete=True,
        )

    @staticmethod
    def _update_progress(
        attempt: _AttemptState,
        *,
        stage: str,
        reason_code: Optional[str],
    ) -> None:
        normalized = stage.upper().replace("-", "_").replace(" ", "_")
        if "ESTIMATOR_READY" in normalized or "PLANNER_READY" in normalized:
            attempt.ready = True
        if "INCOMING_CONFIRMED" in normalized:
            attempt.ready = True
            attempt.confirmed = True
        if normalized in ("PLANNED", "PLAN_READY", "PLANNER_SUCCEEDED"):
            attempt.ready = True
            attempt.confirmed = True
            attempt.planned = True
        if normalized == "ARMED":
            attempt.ready = True
            attempt.confirmed = True
            attempt.planned = True
            attempt.armed = True
        if normalized.endswith("TASK_OBS_PASS") or "TASK_OBS_11/11_PASS" in normalized:
            attempt.ready = True
            attempt.confirmed = True
            attempt.planned = True
            attempt.armed = True
            attempt.passed = True
        if normalized == "LATE_SKIP" or reason_code == "LATE_SKIP":
            attempt.late_skip = True
        if reason_code in _HARD_FAILURE_REASONS:
            attempt.last_hard_failure = reason_code
        if (
            reason_code is not None
            and (
                normalized == "PLANNER_REJECTED"
                or reason_code in _PLANNER_REASONS
            )
        ):
            attempt.last_planner_reason = reason_code
        if (
            reason_code is not None
            and attempt.armed
            and (
                normalized in ("TASK_OBS_FAILED", "OBS_VALIDATION_FAILED")
                or reason_code.startswith("OBS_")
            )
        ):
            attempt.last_obs_failure = reason_code

    @staticmethod
    def _primary_blocker(attempt: _AttemptState) -> Optional[str]:
        if attempt.passed:
            return None
        if attempt.last_hard_failure is not None:
            return attempt.last_hard_failure
        if not attempt.ready:
            return "TRACK_ENDED_BEFORE_READY"
        if not attempt.confirmed:
            return "TRACK_ENDED_BEFORE_CONFIRMATION"
        if not attempt.planned:
            return attempt.last_planner_reason or "NO_VALID_PLAN"
        if not attempt.armed:
            if attempt.late_skip:
                return "LATE_SKIP"
            return "TRACK_ENDED_BEFORE_ARM"
        return attempt.last_obs_failure or "TASK_OBS_FAILED"
