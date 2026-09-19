from __future__ import annotations

from dataclasses import dataclass
import math
from types import MappingProxyType
from typing import Any, Dict, List, Mapping, Optional, Tuple, Union

import numpy as np


SCHEMA_VERSION = 1

JsonScalar = Union[None, bool, int, float, str]
JsonValue = Union[
    JsonScalar,
    Tuple["JsonValue", ...],
    Mapping[str, "JsonValue"],
]


def freeze_json_value(value: Any) -> JsonValue:
    """Return a detached, recursively immutable JSON-compatible value."""
    if isinstance(value, float):
        if math.isnan(value):
            return "NaN"
        if math.isinf(value):
            return "Infinity" if value > 0.0 else "-Infinity"
        return value
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, np.generic):
        return freeze_json_value(value.item())
    if isinstance(value, np.ndarray):
        return freeze_json_value(value.tolist())
    if isinstance(value, Mapping):
        frozen: Dict[str, JsonValue] = {}
        for key, item in value.items():
            if not isinstance(key, str):
                raise TypeError("JSON mapping keys must be strings")
            frozen[key] = freeze_json_value(item)
        return MappingProxyType(frozen)
    if isinstance(value, (tuple, list)):
        return tuple(freeze_json_value(item) for item in value)
    raise TypeError("value is not JSON-compatible: {!r}".format(type(value).__name__))


def to_builtin_json(value: Any) -> Any:
    """Convert a frozen JSON value to fresh dict/list/scalar builtins."""
    if isinstance(value, float):
        if math.isnan(value):
            return "NaN"
        if math.isinf(value):
            return "Infinity" if value > 0.0 else "-Infinity"
        return value
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, np.generic):
        return to_builtin_json(value.item())
    if isinstance(value, np.ndarray):
        return [to_builtin_json(item) for item in value.tolist()]
    if isinstance(value, Mapping):
        return {str(key): to_builtin_json(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [to_builtin_json(item) for item in value]
    raise TypeError("value is not JSON-compatible: {!r}".format(type(value).__name__))


def _freeze_mapping(value: Mapping[str, Any]) -> Mapping[str, JsonValue]:
    frozen = freeze_json_value(value)
    if not isinstance(frozen, Mapping):
        raise TypeError("expected a mapping")
    return frozen


def _readonly_float64(value: np.ndarray, *, shape: Tuple[int, ...], field: str) -> np.ndarray:
    array = np.array(value, dtype=np.float64, copy=True)
    if array.shape != shape:
        raise ValueError("{} must have shape {}, got {}".format(field, shape, array.shape))
    immutable = np.frombuffer(array.tobytes(order="C"), dtype=np.float64).reshape(shape)
    immutable.setflags(write=False)
    return immutable


def _json_dict(value: Mapping[str, Any]) -> Dict[str, Any]:
    result = to_builtin_json(value)
    if not isinstance(result, dict):
        raise TypeError("expected JSON object")
    return result


def _optional_key_json(value: Optional["SnapshotKey"]) -> Any:
    return None if value is None else value.to_json_dict()


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
    source_time_s: Optional[float]
    publish_time_us: Optional[int]
    received_monotonic_s: float
    wall_time_us: int
    payload_size: int

    def __post_init__(self) -> None:
        if self.input_seq < 0:
            raise ValueError("input_seq must be non-negative")
        if not self.channel or not self.subject:
            raise ValueError("channel and subject must be non-empty")
        if self.payload_size < 0:
            raise ValueError("payload_size must be non-negative")
        object.__setattr__(
            self,
            "position_w",
            _readonly_float64(self.position_w, shape=(3,), field="position_w"),
        )
        object.__setattr__(
            self,
            "quaternion_xyzw",
            _readonly_float64(
                self.quaternion_xyzw,
                shape=(4,),
                field="quaternion_xyzw",
            ),
        )

    def to_json_dict(self) -> Dict[str, Any]:
        return _json_dict({
            "schema_version": SCHEMA_VERSION,
            "input_seq": self.input_seq,
            "channel": self.channel,
            "subject": self.subject,
            "position_w": to_builtin_json(self.position_w),
            "quaternion_xyzw": to_builtin_json(self.quaternion_xyzw),
            "valid": self.valid,
            "occluded": self.occluded,
            "source_frame": self.source_frame,
            "source_time_s": self.source_time_s,
            "publish_time_us": self.publish_time_us,
            "received_monotonic_s": self.received_monotonic_s,
            "wall_time_us": self.wall_time_us,
            "payload_size": self.payload_size,
        })


@dataclass(frozen=True, order=True)
class SnapshotKey:
    track_epoch: int
    generation: int

    def __post_init__(self) -> None:
        if self.track_epoch < 0 or self.generation < 0:
            raise ValueError("snapshot identity values must be non-negative")

    def to_json_dict(self) -> Dict[str, Any]:
        return _json_dict({
            "schema_version": SCHEMA_VERSION,
            "track_epoch": self.track_epoch,
            "generation": self.generation,
        })


@dataclass(frozen=True)
class AttemptBinding:
    attempt_id: int
    track_segment_id: int
    role: str
    snapshot_key: SnapshotKey

    def __post_init__(self) -> None:
        if self.attempt_id <= 0 or self.track_segment_id <= 0:
            raise ValueError("attempt and segment ids must be positive")
        if not self.role:
            raise ValueError("role must be non-empty")

    def to_json_dict(self) -> Dict[str, Any]:
        return _json_dict({
            "schema_version": SCHEMA_VERSION,
            "attempt_id": self.attempt_id,
            "track_segment_id": self.track_segment_id,
            "role": self.role,
            "snapshot_key": self.snapshot_key.to_json_dict(),
        })


@dataclass(frozen=True)
class AttemptTransition:
    attempt_id: int
    track_segment_id: Optional[int]
    stage: str
    monotonic_s: float
    snapshot_key: Optional[SnapshotKey]
    reason_code: Optional[str]
    values: Mapping[str, JsonValue]

    def __post_init__(self) -> None:
        if self.attempt_id <= 0:
            raise ValueError("attempt_id must be positive")
        if self.track_segment_id is not None and self.track_segment_id <= 0:
            raise ValueError("track_segment_id must be positive")
        object.__setattr__(self, "values", _freeze_mapping(self.values))

    def to_json_dict(self) -> Dict[str, Any]:
        return _json_dict({
            "schema_version": SCHEMA_VERSION,
            "attempt_id": self.attempt_id,
            "track_segment_id": self.track_segment_id,
            "stage": self.stage,
            "monotonic_s": self.monotonic_s,
            "snapshot_key": _optional_key_json(self.snapshot_key),
            "reason_code": self.reason_code,
            "values": to_builtin_json(self.values),
        })


@dataclass(frozen=True)
class EventDraft:
    kind: str
    monotonic_s: float
    wall_time_us: int
    scope: str
    attempt_id: Optional[int]
    payload: Mapping[str, JsonValue]

    def __post_init__(self) -> None:
        if not self.kind:
            raise ValueError("kind must be non-empty")
        if self.scope not in ("state", "attempt"):
            raise ValueError("scope must be state or attempt")
        if self.scope == "attempt" and (
            self.attempt_id is None or self.attempt_id <= 0
        ):
            raise ValueError("attempt scope requires a positive attempt_id")
        if self.attempt_id is not None and self.attempt_id <= 0:
            raise ValueError("attempt_id must be positive")
        object.__setattr__(self, "payload", _freeze_mapping(self.payload))

    def to_json_dict(self) -> Dict[str, Any]:
        return _json_dict({
            "schema_version": SCHEMA_VERSION,
            "kind": self.kind,
            "monotonic_s": self.monotonic_s,
            "wall_time_us": self.wall_time_us,
            "scope": self.scope,
            "attempt_id": self.attempt_id,
            "payload": to_builtin_json(self.payload),
        })


@dataclass(frozen=True)
class HealthSnapshot:
    lcm_connected: bool
    message_rate_hz_by_subject: Mapping[str, float]
    message_age_s_by_subject: Mapping[str, Optional[float]]
    source_frame_by_subject: Mapping[str, Optional[int]]
    pelvis_valid: bool
    pelvis_age_s: Optional[float]
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
    warnings: Tuple[str, ...]

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "message_rate_hz_by_subject",
            _freeze_mapping(self.message_rate_hz_by_subject),
        )
        object.__setattr__(
            self,
            "message_age_s_by_subject",
            _freeze_mapping(self.message_age_s_by_subject),
        )
        object.__setattr__(
            self,
            "source_frame_by_subject",
            _freeze_mapping(self.source_frame_by_subject),
        )
        object.__setattr__(self, "warnings", tuple(self.warnings))

    def to_json_dict(self) -> Dict[str, Any]:
        return _json_dict({
            "schema_version": SCHEMA_VERSION,
            "lcm_connected": self.lcm_connected,
            "message_rate_hz_by_subject": to_builtin_json(self.message_rate_hz_by_subject),
            "message_age_s_by_subject": to_builtin_json(self.message_age_s_by_subject),
            "source_frame_by_subject": to_builtin_json(self.source_frame_by_subject),
            "pelvis_valid": self.pelvis_valid,
            "pelvis_age_s": self.pelvis_age_s,
            "planner_submitted": self.planner_submitted,
            "planner_completed": self.planner_completed,
            "planner_failed": self.planner_failed,
            "planner_dropped_pending": self.planner_dropped_pending,
            "planner_results_overwritten_before_consume": self.planner_results_overwritten_before_consume,
            "raw_samples_dropped": self.raw_samples_dropped,
            "recorder_event_gaps": self.recorder_event_gaps,
            "diagnostic_events_dropped": self.diagnostic_events_dropped,
            "recorder_healthy": self.recorder_healthy,
            "recording_complete": self.recording_complete,
            "config_name": self.config_name,
            "session_basename": self.session_basename,
            "warnings": to_builtin_json(self.warnings),
        })


@dataclass(frozen=True)
class LifecycleSnapshot:
    phase: str
    last_decision: str
    active_key: Optional[SnapshotKey]
    cached_key: Optional[SnapshotKey]
    lifecycle_now_s: Optional[float]
    obs_now_s: Optional[float]

    def to_json_dict(self) -> Dict[str, Any]:
        return _json_dict({
            "schema_version": SCHEMA_VERSION,
            "phase": self.phase,
            "last_decision": self.last_decision,
            "active_key": _optional_key_json(self.active_key),
            "cached_key": _optional_key_json(self.cached_key),
            "lifecycle_now_s": self.lifecycle_now_s,
            "obs_now_s": self.obs_now_s,
        })


@dataclass(frozen=True)
class AttemptSummary:
    attempt_id: int
    status: str
    stage: str
    primary_blocker: Optional[str]
    ball_speed_mps: Optional[float]
    predicted_strike_time_s: Optional[float]
    planner_tts_s: Optional[float]
    arm_tts_s: Optional[float]
    task_obs_status: str
    ab_summary: Optional[str]
    recording_complete: bool

    def __post_init__(self) -> None:
        if self.attempt_id <= 0:
            raise ValueError("attempt_id must be positive")

    def to_json_dict(self) -> Dict[str, Any]:
        return _json_dict({
            "schema_version": SCHEMA_VERSION,
            "attempt_id": self.attempt_id,
            "status": self.status,
            "stage": self.stage,
            "primary_blocker": self.primary_blocker,
            "ball_speed_mps": self.ball_speed_mps,
            "predicted_strike_time_s": self.predicted_strike_time_s,
            "planner_tts_s": self.planner_tts_s,
            "arm_tts_s": self.arm_tts_s,
            "task_obs_status": self.task_obs_status,
            "ab_summary": self.ab_summary,
            "recording_complete": self.recording_complete,
        })


@dataclass(frozen=True)
class AttemptDetail:
    attempt_id: int
    summary: AttemptSummary
    segments: Tuple[Mapping[str, JsonValue], ...]
    stage_timeline: Tuple[Mapping[str, JsonValue], ...]
    planner_inputs: Tuple[Mapping[str, JsonValue], ...]
    planner_results: Tuple[Mapping[str, JsonValue], ...]
    task_observation_pre_clip: Optional[Tuple[float, ...]]
    task_observation_post_clip: Optional[Tuple[float, ...]]
    task_observation_clip_count: Optional[int]
    variant_outcomes: Tuple[Mapping[str, JsonValue], ...]
    ab_deltas: Mapping[str, JsonValue]

    def __post_init__(self) -> None:
        if self.attempt_id <= 0 or self.summary.attempt_id != self.attempt_id:
            raise ValueError("attempt detail identity must match its summary")
        for field_name in (
            "segments",
            "stage_timeline",
            "planner_inputs",
            "planner_results",
            "variant_outcomes",
        ):
            values = getattr(self, field_name)
            object.__setattr__(
                self,
                field_name,
                tuple(_freeze_mapping(item) for item in values),
            )
        if self.task_observation_pre_clip is not None:
            object.__setattr__(
                self,
                "task_observation_pre_clip",
                tuple(float(item) for item in self.task_observation_pre_clip),
            )
        if self.task_observation_post_clip is not None:
            object.__setattr__(
                self,
                "task_observation_post_clip",
                tuple(float(item) for item in self.task_observation_post_clip),
            )
        object.__setattr__(self, "ab_deltas", _freeze_mapping(self.ab_deltas))

    def to_json_dict(self) -> Dict[str, Any]:
        return _json_dict({
            "schema_version": SCHEMA_VERSION,
            "attempt_id": self.attempt_id,
            "summary": self.summary.to_json_dict(),
            "segments": to_builtin_json(self.segments),
            "stage_timeline": to_builtin_json(self.stage_timeline),
            "planner_inputs": to_builtin_json(self.planner_inputs),
            "planner_results": to_builtin_json(self.planner_results),
            "task_observation_pre_clip": to_builtin_json(self.task_observation_pre_clip),
            "task_observation_post_clip": to_builtin_json(self.task_observation_post_clip),
            "task_observation_clip_count": self.task_observation_clip_count,
            "variant_outcomes": to_builtin_json(self.variant_outcomes),
            "ab_deltas": to_builtin_json(self.ab_deltas),
        })


@dataclass(frozen=True)
class AttemptPage:
    items: Tuple[AttemptSummary, ...]
    has_more: bool
    next_before: Optional[int]

    def __post_init__(self) -> None:
        object.__setattr__(self, "items", tuple(self.items))
        if self.next_before is not None and self.next_before <= 0:
            raise ValueError("next_before must be positive")

    def to_json_dict(self) -> Dict[str, Any]:
        return _json_dict({
            "schema_version": SCHEMA_VERSION,
            "items": [item.to_json_dict() for item in self.items],
            "has_more": self.has_more,
            "next_before": self.next_before,
        })
