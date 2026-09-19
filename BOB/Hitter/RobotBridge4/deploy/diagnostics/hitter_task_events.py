from __future__ import annotations

from collections import OrderedDict, deque
from dataclasses import dataclass, replace
from itertools import islice
import json
import math
import threading
import time
from typing import Any, Callable, Deque, Dict, Mapping, Optional, Tuple
import weakref

from diagnostics.hitter_task_models import (
    LIVE_SNAPSHOT_SCHEMA_VERSION,
    SCHEMA_VERSION,
    AttemptSummary,
    BallDiagnosticState,
    EventDraft,
    HealthSnapshot,
    JsonValue,
    LifecycleSnapshot,
    LiveDiagnosticSnapshot,
    ProductionGateState,
    SnapshotKey,
    SubjectHealth,
    freeze_json_value,
    to_builtin_json,
)


_RECENT_ATTEMPT_LIMIT = 100
_MIN_EVENT_BYTES = 1024
_INT64_MAX = (1 << 63) - 1
_INT64_MIN = -(1 << 63)


class _InvalidDraft(ValueError):
    pass


class _EventTooLarge(ValueError):
    pass


@dataclass(frozen=True)
class PublishedEvent:
    schema_version: int
    event_id: int
    kind: str
    monotonic_s: float
    wall_time_us: int
    scope: str
    attempt_id: Optional[int]
    payload: Mapping[str, JsonValue]

    def __post_init__(self) -> None:
        if self.schema_version != SCHEMA_VERSION:
            raise ValueError("unsupported schema_version")
        if self.event_id <= 0:
            raise ValueError("event_id must be positive")
        if self.scope not in ("state", "attempt"):
            raise ValueError("scope must be state or attempt")
        if self.scope == "attempt" and (
            self.attempt_id is None or self.attempt_id <= 0
        ):
            raise ValueError("attempt scope requires a positive attempt_id")
        frozen = freeze_json_value(self.payload)
        if not isinstance(frozen, Mapping):
            raise TypeError("event payload must be a mapping")
        object.__setattr__(self, "payload", frozen)

    @classmethod
    def _from_frozen(
        cls,
        *,
        event_id: int,
        kind: str,
        monotonic_s: float,
        wall_time_us: int,
        scope: str,
        attempt_id: Optional[int],
        payload: Mapping[str, JsonValue],
    ) -> "PublishedEvent":
        event = object.__new__(cls)
        object.__setattr__(event, "schema_version", SCHEMA_VERSION)
        object.__setattr__(event, "event_id", event_id)
        object.__setattr__(event, "kind", kind)
        object.__setattr__(event, "monotonic_s", monotonic_s)
        object.__setattr__(event, "wall_time_us", wall_time_us)
        object.__setattr__(event, "scope", scope)
        object.__setattr__(event, "attempt_id", attempt_id)
        object.__setattr__(event, "payload", payload)
        return event

    def to_json_dict(self) -> Dict[str, Any]:
        return to_builtin_json(
            {
                "schema_version": self.schema_version,
                "event_id": self.event_id,
                "kind": self.kind,
                "monotonic_s": self.monotonic_s,
                "wall_time_us": self.wall_time_us,
                "scope": self.scope,
                "attempt_id": self.attempt_id,
                "payload": self.payload,
            }
        )


@dataclass(frozen=True)
class CursorBootstrap:
    mode: str
    watermark_event_id: int

    def __post_init__(self) -> None:
        if self.mode not in ("fresh", "resume", "reset"):
            raise ValueError("invalid cursor bootstrap mode")


@dataclass(frozen=True)
class EventRead:
    events: Tuple[PublishedEvent, ...]
    watermark_event_id: int
    lost_event_ids: Optional[Tuple[int, int]]

    def __post_init__(self) -> None:
        object.__setattr__(self, "events", tuple(self.events))


@dataclass(frozen=True)
class DiagnosticState:
    schema_version: int
    watermark_event_id: int
    health: HealthSnapshot
    lifecycle: LifecycleSnapshot
    current_attempt: Optional[AttemptSummary]
    recent_attempts: Tuple[AttemptSummary, ...]
    live_snapshot: Optional[LiveDiagnosticSnapshot] = None

    def __post_init__(self) -> None:
        if self.schema_version not in (
            SCHEMA_VERSION,
            LIVE_SNAPSHOT_SCHEMA_VERSION,
        ):
            raise ValueError("unsupported schema_version")
        if self.watermark_event_id < 0:
            raise ValueError("watermark_event_id must be non-negative")
        if (
            self.live_snapshot is not None
            and self.current_attempt
            != self.live_snapshot.current_attempt
        ):
            raise ValueError(
                "current_attempt mirror must match live snapshot"
            )
        object.__setattr__(
            self,
            "recent_attempts",
            tuple(self.recent_attempts[:_RECENT_ATTEMPT_LIMIT]),
        )

    def to_json_dict(self) -> Dict[str, Any]:
        value = {
            "schema_version": self.schema_version,
            "watermark_event_id": self.watermark_event_id,
            "health": self.health.to_json_dict(),
            "lifecycle": self.lifecycle.to_json_dict(),
            "current_attempt": (
                None
                if self.current_attempt is None
                else self.current_attempt.to_json_dict()
            ),
            "recent_attempts": tuple(
                attempt.to_json_dict() for attempt in self.recent_attempts
            ),
        }
        if (
            self.schema_version == LIVE_SNAPSHOT_SCHEMA_VERSION
            or self.live_snapshot is not None
        ):
            value["live_snapshot"] = (
                None
                if self.live_snapshot is None
                else self.live_snapshot.to_json_dict()
            )
        return to_builtin_json(value)


def _view(payload: Mapping[str, JsonValue], nested_key: str) -> Mapping[str, Any]:
    nested = payload.get(nested_key)
    if isinstance(nested, Mapping):
        return nested
    return payload


def _snapshot_key(value: Any) -> Optional[SnapshotKey]:
    if value is None:
        return None
    if not isinstance(value, Mapping):
        raise _InvalidDraft("snapshot key must be a mapping")
    return SnapshotKey(
        track_id=int(value["track_id"]),
        generation=int(value["generation"]),
    )


def _health(payload: Mapping[str, JsonValue]) -> HealthSnapshot:
    value = _view(payload, "health")
    try:
        return HealthSnapshot(
            lcm_connected=bool(value["lcm_connected"]),
            message_rate_hz_by_subject=value["message_rate_hz_by_subject"],
            message_age_s_by_subject=value["message_age_s_by_subject"],
            source_frame_by_subject=value["source_frame_by_subject"],
            pelvis_valid=bool(value["pelvis_valid"]),
            pelvis_age_s=value["pelvis_age_s"],
            planner_submitted=int(value["planner_submitted"]),
            planner_completed=int(value["planner_completed"]),
            planner_failed=int(value["planner_failed"]),
            planner_dropped_pending=int(value["planner_dropped_pending"]),
            planner_submit_rate_hz=float(value["planner_submit_rate_hz"]),
            completed_result_queue_depth=int(
                value["completed_result_queue_depth"]
            ),
            completed_result_queue_capacity=int(
                value["completed_result_queue_capacity"]
            ),
            completed_result_queue_overflow_count=int(
                value["completed_result_queue_overflow_count"]
            ),
            raw_samples_dropped=int(value["raw_samples_dropped"]),
            recorder_event_gaps=int(value["recorder_event_gaps"]),
            diagnostic_events_dropped=int(value["diagnostic_events_dropped"]),
            recorder_healthy=bool(value["recorder_healthy"]),
            recording_complete=bool(value["recording_complete"]),
            config_name=str(value["config_name"]),
            session_basename=str(value["session_basename"]),
            warnings=tuple(value["warnings"]),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise _InvalidDraft("invalid health snapshot") from exc


def _lifecycle(payload: Mapping[str, JsonValue]) -> LifecycleSnapshot:
    value = _view(payload, "lifecycle")
    try:
        return LifecycleSnapshot(
            phase=str(value["phase"]),
            last_decision=str(value["last_decision"]),
            active_key=_snapshot_key(value["active_key"]),
            active_track_id=value.get("active_track_id"),
            decision_track_id=value.get("decision_track_id"),
            failure_reason=value.get("failure_reason"),
            cancel_reason=value.get("cancel_reason"),
            consumed=bool(value["consumed"]),
            consumed_track_ids=tuple(value["consumed_track_ids"]),
            locked_track_id=value.get("locked_track_id"),
            locked_strike_type=value.get("locked_strike_type"),
            locked_base_target_xy=value.get("locked_base_target_xy"),
            locked_strike_deadline_monotonic_s=value.get(
                "locked_strike_deadline_monotonic_s"
            ),
            in_commit_window=bool(value["in_commit_window"]),
            policy_tts_s=float(value["policy_tts_s"]),
            strike_count=int(value["strike_count"]),
            completed_result_queue_depth=int(
                value["completed_result_queue_depth"]
            ),
            completed_result_queue_capacity=int(
                value["completed_result_queue_capacity"]
            ),
            completed_result_queue_overflow_count=int(
                value["completed_result_queue_overflow_count"]
            ),
            lifecycle_now_s=value["lifecycle_now_s"],
            obs_now_s=value["obs_now_s"],
            wall_time_us=value.get("wall_time_us"),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise _InvalidDraft("invalid lifecycle snapshot") from exc


def _attempt(payload: Mapping[str, JsonValue]) -> AttemptSummary:
    value = _view(payload, "attempt")
    if value is payload:
        value = _view(payload, "summary")
    try:
        attempt_id = int(value["attempt_id"])
        return AttemptSummary(
            attempt_id=attempt_id,
            track_id=int(value.get("track_id", attempt_id)),
            identity_source=str(
                value.get("identity_source", "legacy_inferred")
            ),
            status=str(value["status"]),
            stage=str(value["stage"]),
            primary_blocker=value["primary_blocker"],
            ball_speed_mps=value["ball_speed_mps"],
            predicted_strike_time_s=value["predicted_strike_time_s"],
            planner_tts_s=value["planner_tts_s"],
            arm_tts_s=value["arm_tts_s"],
            task_obs_status=str(value["task_obs_status"]),
            ab_summary=value["ab_summary"],
            recording_complete=bool(value["recording_complete"]),
            consumed=bool(value.get("consumed", False)),
            cancel_reason=value.get("cancel_reason"),
            failure_reason=value.get("failure_reason"),
            locked_strike_type=value.get("locked_strike_type"),
            locked_base_target_xy=value.get("locked_base_target_xy"),
            locked_strike_deadline_monotonic_s=value.get(
                "locked_strike_deadline_monotonic_s"
            ),
            strike_table_y_w=value.get("strike_table_y_w"),
            strike_side_source=value.get("strike_side_source"),
            expected_strike_type_from_table_y=value.get(
                "expected_strike_type_from_table_y"
            ),
            strike_type_consistent=value.get("strike_type_consistent"),
            strike_count=int(value.get("strike_count", 0)),
            estimator_sample_count=value.get("estimator_sample_count"),
            estimator_window_size=value.get("estimator_window_size"),
            incoming_count=value.get("incoming_count"),
            incoming_required_count=value.get("incoming_required_count"),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise _InvalidDraft("invalid attempt snapshot") from exc


def _subject_health(value: object) -> SubjectHealth:
    if not isinstance(value, Mapping):
        raise _InvalidDraft("subject health must be a mapping")
    try:
        return SubjectHealth(
            status=str(value["status"]),
            rate_hz=value.get("rate_hz"),
            age_s=value.get("age_s"),
            source_frame=value.get("source_frame"),
            valid=value.get("valid"),
            occluded=value.get("occluded"),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise _InvalidDraft("invalid subject health") from exc


def _ball_diagnostic(value: object) -> BallDiagnosticState:
    if not isinstance(value, Mapping):
        raise _InvalidDraft("ball diagnostic must be a mapping")
    try:
        return BallDiagnosticState(
            status=str(value["status"]),
            track_id=value.get("track_id"),
            identity_source=value.get("identity_source"),
            consumed=bool(value.get("consumed", False)),
            estimator_sample_count=value.get("estimator_sample_count"),
            estimator_window_size=value.get("estimator_window_size"),
            speed_mps=value.get("speed_mps"),
            velocity_world_mps=value.get(
                "estimated_velocity_w",
                value.get("velocity_world_mps"),
            ),
            incoming_count=value.get(
                "ball_only_incoming_count",
                value.get("incoming_count"),
            ),
            incoming_required_count=value.get(
                "ball_only_incoming_required",
                value.get("incoming_required_count"),
            ),
            incoming_status=str(
                value.get("incoming_status", "UNKNOWN")
            ),
            blocker=value.get("blocker"),
            source_frame=value.get("source_frame"),
            generation=value.get("generation"),
            raw_position_w=value.get("raw_position_w"),
            estimated_position_w=value.get("estimated_position_w"),
            observed_monotonic_s=value.get("observed_monotonic_s"),
            age_s=value.get("age_s"),
            last_estimator_reset_reason=value.get(
                "last_estimator_reset_reason"
            ),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise _InvalidDraft("invalid ball diagnostic") from exc


def _production_gate(value: object) -> ProductionGateState:
    if not isinstance(value, Mapping):
        raise _InvalidDraft("production gate must be a mapping")
    try:
        return ProductionGateState(
            pelvis_status=str(value["pelvis_status"]),
            production_incoming_status=str(
                value["production_incoming_status"]
            ),
            production_incoming_count=value.get(
                "production_incoming_count"
            ),
            production_incoming_required=value.get(
                "production_incoming_required"
            ),
            planner_status=str(value["planner_status"]),
            planner_reason_code=value.get("planner_reason_code"),
            planner_tts_s=value.get("planner_tts_s"),
            arm_status=str(value["arm_status"]),
            arm_trigger_tts_s=value.get("arm_trigger_tts_s"),
            task_observation_status=str(
                value["task_observation_status"]
            ),
            task_observation_valid_dimensions=value.get(
                "task_observation_valid_dimensions"
            ),
            task_observation_total_dimensions=value.get(
                "task_observation_total_dimensions"
            ),
            task_observation_clip_count=value.get(
                "task_observation_clip_count"
            ),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise _InvalidDraft("invalid production gate") from exc


def _live_snapshot(
    payload: Mapping[str, JsonValue],
) -> LiveDiagnosticSnapshot:
    value = _view(payload, "live_snapshot")
    try:
        schema_version = int(value["schema_version"])
        if schema_version != LIVE_SNAPSHOT_SCHEMA_VERSION:
            raise ValueError("unsupported live snapshot schema")
        subjects_value = value["subjects"]
        if not isinstance(subjects_value, Mapping):
            raise TypeError("subjects must be a mapping")
        current_value = value.get("current_attempt")
        current = (
            None
            if current_value is None
            else _attempt(current_value)
        )
        return LiveDiagnosticSnapshot(
            schema_version=schema_version,
            revision=int(value["revision"]),
            captured_monotonic_s=float(
                value["captured_monotonic_s"]
            ),
            subjects={
                str(name): _subject_health(subject)
                for name, subject in subjects_value.items()
            },
            ball=_ball_diagnostic(value["ball"]),
            production_gate=_production_gate(
                value["production_gate"]
            ),
            current_attempt=current,
        )
    except (_InvalidDraft, KeyError, TypeError, ValueError) as exc:
        raise _InvalidDraft("invalid live diagnostic snapshot") from exc


def reduce_diagnostic_state(
    state: DiagnosticState,
    event: PublishedEvent,
) -> DiagnosticState:
    """Apply one full-snapshot event without mutating the input state."""
    if event.kind == "HEALTH_SNAPSHOT":
        return replace(
            state,
            watermark_event_id=event.event_id,
            health=_health(event.payload),
        )
    if event.kind == "LIFECYCLE_SNAPSHOT":
        return replace(
            state,
            watermark_event_id=event.event_id,
            lifecycle=_lifecycle(event.payload),
        )
    if event.kind == "ATTEMPT_CURRENT":
        current = _attempt(event.payload)
        if state.live_snapshot is not None:
            current = state.live_snapshot.current_attempt
        return replace(
            state,
            watermark_event_id=event.event_id,
            current_attempt=current,
        )
    if event.kind == "ATTEMPT_CLOSED":
        closed = _attempt(event.payload)
        recent = (closed,) + tuple(
            item
            for item in state.recent_attempts
            if item.attempt_id != closed.attempt_id
        )
        current = state.current_attempt
        if (
            state.live_snapshot is None
            and current is not None
            and current.attempt_id == closed.attempt_id
        ):
            current = None
        return replace(
            state,
            watermark_event_id=event.event_id,
            current_attempt=current,
            recent_attempts=recent[:_RECENT_ATTEMPT_LIMIT],
        )
    if event.kind == "LIVE_SNAPSHOT":
        live_snapshot = _live_snapshot(event.payload)
        return replace(
            state,
            schema_version=LIVE_SNAPSHOT_SCHEMA_VERSION,
            watermark_event_id=event.event_id,
            current_attempt=live_snapshot.current_attempt,
            live_snapshot=live_snapshot,
        )
    if event.kind == "DIAGNOSTIC_EVENT_DROPPED":
        dropped = event.payload.get("dropped_events", 1)
        if isinstance(dropped, bool) or not isinstance(dropped, int) or dropped <= 0:
            dropped = 1
        return replace(
            state,
            watermark_event_id=event.event_id,
            health=replace(
                state.health,
                diagnostic_events_dropped=(
                    state.health.diagnostic_events_dropped + dropped
                ),
            ),
        )
    return replace(state, watermark_event_id=event.event_id)


Reducer = Callable[[DiagnosticState, PublishedEvent], DiagnosticState]


@dataclass(frozen=True)
class _PreparedDraft:
    kind: str
    monotonic_s: float
    wall_time_us: int
    scope: str
    attempt_id: Optional[int]
    payload: Mapping[str, JsonValue]
    encoded_size_upper: int
    fallback: bool

    def bind(self, event_id: int) -> PublishedEvent:
        return PublishedEvent._from_frozen(
            event_id=event_id,
            kind=self.kind,
            monotonic_s=self.monotonic_s,
            wall_time_us=self.wall_time_us,
            scope=self.scope,
            attempt_id=self.attempt_id,
            payload=self.payload,
        )


def _truncate_utf8(value: object, maximum_bytes: int) -> str:
    encoded = str(value).encode("utf-8", errors="replace")[:maximum_bytes]
    while True:
        try:
            return encoded.decode("utf-8")
        except UnicodeDecodeError:
            encoded = encoded[:-1]


def _canonical_monotonic(value: object) -> float:
    try:
        converted = float(value)
    except (TypeError, ValueError, OverflowError):
        return 0.0
    return converted if math.isfinite(converted) else 0.0


def _canonical_wall_time(value: object) -> int:
    try:
        converted = int(value)
    except (TypeError, ValueError, OverflowError):
        return 0
    return min(_INT64_MAX, max(_INT64_MIN, converted))


class EventCursor:
    def __init__(
        self,
        hub: "EventHub",
        *,
        next_event_id: int,
        bootstrap: CursorBootstrap,
    ) -> None:
        self._hub = hub
        self._next_event_id = next_event_id
        self._bootstrap = bootstrap
        self._closed = False

    @property
    def bootstrap(self) -> CursorBootstrap:
        return self._bootstrap

    def read(
        self,
        *,
        limit: int = 128,
        timeout_s: Optional[float] = None,
    ) -> EventRead:
        return self._hub._read(self, limit=limit, timeout_s=timeout_s)

    def close(self) -> None:
        self._hub._close_cursor(self)


class EventHub:
    def __init__(
        self,
        initial_state: DiagnosticState,
        *,
        capacity: int = 8192,
        max_event_bytes: int = 65536,
        max_ring_bytes: int = 64 * 1024 * 1024,
        reducer: Optional[Reducer] = None,
    ) -> None:
        if capacity <= 0:
            raise ValueError("capacity must be positive")
        if max_event_bytes < _MIN_EVENT_BYTES:
            raise ValueError(
                "max_event_bytes must be at least {}".format(_MIN_EVENT_BYTES)
            )
        if max_ring_bytes < max_event_bytes:
            raise ValueError("max_ring_bytes must be at least max_event_bytes")
        if initial_state.watermark_event_id >= _INT64_MAX:
            raise ValueError("initial watermark leaves no event id space")
        self._condition = threading.Condition(threading.RLock())
        self._capacity = int(capacity)
        self._max_event_bytes = int(max_event_bytes)
        self._max_ring_bytes = int(max_ring_bytes)
        self._reducer = reduce_diagnostic_state if reducer is None else reducer
        self._state = initial_state
        self._ring: Deque[PublishedEvent] = deque()
        self._ring_sizes: Deque[int] = deque()
        self._ring_bytes = 0
        self._next_event_id = initial_state.watermark_event_id + 1
        self._closed = False
        self._cursors: "weakref.WeakSet[EventCursor]" = weakref.WeakSet()

    def publish(self, draft: EventDraft) -> PublishedEvent:
        prepared, invalid_fallback = self._prepare(draft)
        with self._condition:
            if self._closed:
                raise RuntimeError("event hub is closed")
            event_id = self._next_event_id
            event = prepared.bind(event_id)
            fallback = prepared.fallback

            try:
                next_state = self._reducer(self._state, event)
            except _InvalidDraft:
                prepared = invalid_fallback
                event = prepared.bind(event_id)
                fallback = True
                next_state = self._reducer(self._state, event)

            if not isinstance(next_state, DiagnosticState):
                raise TypeError("reducer must return DiagnosticState")
            if fallback:
                wanted = self._state.health.diagnostic_events_dropped + 1
                if next_state.health.diagnostic_events_dropped < wanted:
                    next_state = replace(
                        next_state,
                        health=replace(
                            next_state.health,
                            diagnostic_events_dropped=wanted,
                        ),
                    )
            next_state = replace(
                next_state,
                watermark_event_id=event_id,
                recent_attempts=next_state.recent_attempts[
                    :_RECENT_ATTEMPT_LIMIT
                ],
            )
            while self._ring and (
                len(self._ring) >= self._capacity
                or self._ring_bytes + prepared.encoded_size_upper
                > self._max_ring_bytes
            ):
                self._ring.popleft()
                self._ring_bytes -= self._ring_sizes.popleft()
            self._ring.append(event)
            self._ring_sizes.append(prepared.encoded_size_upper)
            self._ring_bytes += prepared.encoded_size_upper
            self._state = next_state
            self._next_event_id += 1
            self._condition.notify_all()
            return event

    def state_snapshot(self) -> DiagnosticState:
        with self._condition:
            return self._state

    def cursor_count(self) -> int:
        with self._condition:
            return len(self._cursors)

    def ring_bytes(self) -> int:
        with self._condition:
            return self._ring_bytes

    def open_cursor(self, after_event_id: Optional[int]) -> EventCursor:
        with self._condition:
            if self._closed:
                raise RuntimeError("event hub is closed")
            watermark = self._state.watermark_event_id
            oldest = self._ring[0].event_id if self._ring else watermark + 1
            if after_event_id is None:
                mode = "fresh"
                next_event_id = watermark + 1
            elif (
                after_event_id < 0
                or after_event_id > watermark
                or after_event_id < oldest - 1
            ):
                mode = "reset"
                next_event_id = watermark + 1
            else:
                mode = "resume"
                next_event_id = after_event_id + 1
            cursor = EventCursor(
                self,
                next_event_id=next_event_id,
                bootstrap=CursorBootstrap(
                    mode=mode,
                    watermark_event_id=watermark,
                ),
            )
            self._cursors.add(cursor)
            return cursor

    def close(self) -> None:
        with self._condition:
            if self._closed:
                return
            self._closed = True
            for cursor in self._cursors:
                cursor._closed = True
            self._cursors.clear()
            self._condition.notify_all()

    def _prepare(
        self,
        draft: EventDraft,
    ) -> Tuple[_PreparedDraft, _PreparedDraft]:
        invalid_fallback = self._prepare_fallback(
            draft,
            "INVALID_EVENT",
            "recognized event payload failed schema reduction",
        )
        try:
            size = self._encoded_size(
                kind=draft.kind,
                monotonic_s=draft.monotonic_s,
                wall_time_us=draft.wall_time_us,
                scope=draft.scope,
                attempt_id=draft.attempt_id,
                payload=draft.payload,
            )
            if size > self._max_event_bytes:
                raise _EventTooLarge("event exceeds max_event_bytes")
            prepared = _PreparedDraft(
                kind=draft.kind,
                monotonic_s=draft.monotonic_s,
                wall_time_us=draft.wall_time_us,
                scope=draft.scope,
                attempt_id=draft.attempt_id,
                payload=draft.payload,
                encoded_size_upper=size,
                fallback=False,
            )
        except (TypeError, ValueError, OverflowError, RecursionError) as exc:
            prepared = self._prepare_fallback(
                draft,
                type(exc).__name__,
                str(exc),
            )
        return prepared, invalid_fallback

    def _prepare_fallback(
        self,
        draft: EventDraft,
        reason: str,
        error_text: str,
    ) -> _PreparedDraft:
        attempt = (
            None
            if draft.attempt_id is None
            else _truncate_utf8(draft.attempt_id, 96)
        )
        payload = freeze_json_value(
            {
                "reason_code": _truncate_utf8(reason, 64),
                "error_text": _truncate_utf8(error_text, 256),
                "original_kind": _truncate_utf8(draft.kind, 96),
                "original_attempt_id": attempt,
            }
        )
        if not isinstance(payload, Mapping):
            raise RuntimeError("canonical fallback payload is not a mapping")
        monotonic_s = _canonical_monotonic(draft.monotonic_s)
        wall_time_us = _canonical_wall_time(draft.wall_time_us)
        size = self._encoded_size(
            kind="DIAGNOSTIC_EVENT_DROPPED",
            monotonic_s=monotonic_s,
            wall_time_us=wall_time_us,
            scope="state",
            attempt_id=None,
            payload=payload,
        )
        if size > self._max_event_bytes:
            raise RuntimeError("canonical diagnostic fallback is too large")
        return _PreparedDraft(
            kind="DIAGNOSTIC_EVENT_DROPPED",
            monotonic_s=monotonic_s,
            wall_time_us=wall_time_us,
            scope="state",
            attempt_id=None,
            payload=payload,
            encoded_size_upper=size,
            fallback=True,
        )

    @staticmethod
    def _encoded_size(
        *,
        kind: str,
        monotonic_s: float,
        wall_time_us: int,
        scope: str,
        attempt_id: Optional[int],
        payload: Mapping[str, JsonValue],
    ) -> int:
        encoded = json.dumps(
            to_builtin_json(
                {
                    "schema_version": SCHEMA_VERSION,
                    "event_id": _INT64_MAX,
                    "kind": kind,
                    "monotonic_s": monotonic_s,
                    "wall_time_us": wall_time_us,
                    "scope": scope,
                    "attempt_id": attempt_id,
                    "payload": payload,
                }
            ),
            ensure_ascii=False,
            allow_nan=False,
            separators=(",", ":"),
        ).encode("utf-8")
        return len(encoded)

    def _read(
        self,
        cursor: EventCursor,
        *,
        limit: int,
        timeout_s: Optional[float],
    ) -> EventRead:
        if limit <= 0:
            raise ValueError("limit must be positive")
        if timeout_s is not None and timeout_s < 0.0:
            raise ValueError("timeout_s must be non-negative")
        deadline = None if timeout_s is None else time.monotonic() + timeout_s
        with self._condition:
            while True:
                if self._closed:
                    raise RuntimeError("event hub is closed")
                if cursor._closed:
                    raise RuntimeError("event cursor is closed")
                watermark = self._state.watermark_event_id
                if cursor._next_event_id <= watermark:
                    oldest = (
                        self._ring[0].event_id
                        if self._ring
                        else watermark + 1
                    )
                    if cursor._next_event_id < oldest:
                        lost = (cursor._next_event_id, oldest - 1)
                        cursor._next_event_id = watermark + 1
                        return EventRead(
                            events=(),
                            watermark_event_id=watermark,
                            lost_event_ids=lost,
                        )
                    offset = cursor._next_event_id - oldest
                    events = tuple(
                        islice(self._ring, offset, offset + limit)
                    )
                    if events:
                        cursor._next_event_id = events[-1].event_id + 1
                    return EventRead(
                        events=events,
                        watermark_event_id=watermark,
                        lost_event_ids=None,
                    )
                if timeout_s == 0.0:
                    return EventRead(
                        events=(),
                        watermark_event_id=watermark,
                        lost_event_ids=None,
                    )
                if deadline is None:
                    self._condition.wait()
                    continue
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return EventRead(
                        events=(),
                        watermark_event_id=watermark,
                        lost_event_ids=None,
                    )
                self._condition.wait(remaining)

    def _close_cursor(self, cursor: EventCursor) -> None:
        with self._condition:
            if cursor._closed:
                return
            cursor._closed = True
            self._cursors.discard(cursor)
            self._condition.notify_all()


@dataclass
class _ProgressEntry:
    last_emit_s: float
    pending: Optional[EventDraft]


class ProgressMailbox:
    """Bounded producer-side progress coalescing before EventHub.publish()."""

    def __init__(self, *, capacity: int, min_interval_s: float) -> None:
        if capacity <= 0:
            raise ValueError("capacity must be positive")
        if min_interval_s < 0.0:
            raise ValueError("min_interval_s must be non-negative")
        self._capacity = int(capacity)
        self._min_interval_s = float(min_interval_s)
        self._lock = threading.Lock()
        self._entries: "OrderedDict[Tuple[Optional[int], str], _ProgressEntry]" = (
            OrderedDict()
        )

    def offer(self, draft: EventDraft, *, stage: str) -> Optional[EventDraft]:
        if not stage:
            raise ValueError("stage must be non-empty")
        key = (draft.attempt_id, stage)
        with self._lock:
            entry = self._entries.get(key)
            if entry is None:
                self._entries[key] = _ProgressEntry(
                    last_emit_s=draft.monotonic_s,
                    pending=None,
                )
                self._trim()
                return draft
            self._entries.move_to_end(key)
            if (
                draft.monotonic_s
                >= entry.last_emit_s + self._min_interval_s
            ):
                entry.last_emit_s = draft.monotonic_s
                entry.pending = None
                return draft
            entry.pending = draft
            return None

    def drain_ready(
        self,
        *,
        now_monotonic_s: float,
    ) -> Tuple[EventDraft, ...]:
        ready = []
        with self._lock:
            for entry in self._entries.values():
                if (
                    entry.pending is not None
                    and now_monotonic_s
                    >= entry.last_emit_s + self._min_interval_s
                ):
                    ready.append(entry.pending)
                    entry.pending = None
                    entry.last_emit_s = now_monotonic_s
        return tuple(ready)

    def _trim(self) -> None:
        while len(self._entries) > self._capacity:
            self._entries.popitem(last=False)


class EventPublisherLane:
    """Bounded non-blocking producer lane with one EventHub owner thread."""

    def __init__(self, hub: EventHub, *, capacity: int = 4096) -> None:
        if capacity <= 0:
            raise ValueError("capacity must be positive")
        self._hub = hub
        self._capacity = int(capacity)
        self._condition = threading.Condition(threading.Lock())
        self._queue: Deque[EventDraft] = deque()
        self._overflow_count = 0
        self._inflight = False
        self._accepting = True
        self._stop_requested = False
        self._thread = threading.Thread(
            target=self._run,
            name="hitter-diagnostic-event-publisher",
            daemon=True,
        )
        self._thread.start()

    def offer(self, draft: EventDraft) -> bool:
        """Strict put-nowait semantics for realtime event producers."""
        with self._condition:
            if not self._accepting:
                return False
            if len(self._queue) >= self._capacity:
                self._overflow_count += 1
                self._condition.notify()
                return False
            self._queue.append(draft)
            self._condition.notify()
            return True

    def drain(self, *, timeout_s: float) -> bool:
        if timeout_s < 0.0:
            raise ValueError("timeout_s must be non-negative")
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while self._queue or self._overflow_count or self._inflight:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False
                self._condition.wait(remaining)
            return True

    def close(self, *, timeout_s: float) -> bool:
        if timeout_s < 0.0:
            raise ValueError("timeout_s must be non-negative")
        with self._condition:
            self._accepting = False
            self._stop_requested = True
            self._condition.notify_all()
        self._thread.join(timeout_s)
        return not self._thread.is_alive()

    def _run(self) -> None:
        while True:
            with self._condition:
                while (
                    not self._queue
                    and self._overflow_count == 0
                    and not self._stop_requested
                ):
                    self._condition.wait()
                if self._queue:
                    draft = self._queue.popleft()
                elif self._overflow_count:
                    dropped = self._overflow_count
                    self._overflow_count = 0
                    draft = EventDraft(
                        kind="DIAGNOSTIC_EVENT_DROPPED",
                        monotonic_s=time.monotonic(),
                        wall_time_us=0,
                        scope="state",
                        attempt_id=None,
                        payload={
                            "reason_code": "PUBLISHER_QUEUE_FULL",
                            "dropped_events": dropped,
                        },
                    )
                elif self._stop_requested:
                    return
                else:
                    continue
                self._inflight = True
            try:
                self._hub.publish(draft)
            except Exception:
                if draft.kind != "DIAGNOSTIC_EVENT_DROPPED":
                    with self._condition:
                        self._overflow_count += 1
            finally:
                with self._condition:
                    self._inflight = False
                    self._condition.notify_all()
