from __future__ import annotations

from dataclasses import dataclass
import math
from types import MappingProxyType
from typing import Any, Dict, List, Mapping, Optional, Tuple, Union

import numpy as np

from utils.hitter_runtime_types import SnapshotKey


SCHEMA_VERSION = 2
LIVE_SNAPSHOT_SCHEMA_VERSION = 3

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
    track_id: int = 0
    identity_source: str = "legacy_inferred"

    def __post_init__(self) -> None:
        if self.input_seq < 0:
            raise ValueError("input_seq must be non-negative")
        if not self.channel or not self.subject:
            raise ValueError("channel and subject must be non-empty")
        if type(self.track_id) is not int or self.track_id < 0:
            raise ValueError("track_id must be a non-negative integer")
        if self.identity_source not in ("wire_v2", "legacy_inferred"):
            raise ValueError(
                "identity_source must be wire_v2 or legacy_inferred"
            )
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
            "track_id": self.track_id,
            "identity_source": self.identity_source,
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


@dataclass(frozen=True)
class AttemptBinding:
    attempt_id: int
    track_segment_id: int
    role: str
    snapshot_key: SnapshotKey
    track_id: Optional[int] = None
    identity_source: str = "legacy_inferred"

    def __post_init__(self) -> None:
        if self.attempt_id <= 0 or self.track_segment_id <= 0:
            raise ValueError("attempt and segment ids must be positive")
        track_id = (
            self.snapshot_key.track_id
            if self.track_id is None
            else self.track_id
        )
        object.__setattr__(self, "track_id", track_id)
        if type(track_id) is not int or track_id <= 0:
            raise ValueError("track_id must be positive")
        if self.identity_source not in ("wire_v2", "legacy_inferred"):
            raise ValueError(
                "identity_source must be wire_v2 or legacy_inferred"
            )
        if self.snapshot_key.track_id != track_id:
            raise ValueError("binding track_id must match snapshot_key")
        if not self.role:
            raise ValueError("role must be non-empty")

    def to_json_dict(self) -> Dict[str, Any]:
        return _json_dict({
            "schema_version": SCHEMA_VERSION,
            "attempt_id": self.attempt_id,
            "track_segment_id": self.track_segment_id,
            "track_id": self.track_id,
            "identity_source": self.identity_source,
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
    raw_samples_dropped: int
    recorder_event_gaps: int
    diagnostic_events_dropped: int
    recorder_healthy: bool
    recording_complete: bool
    config_name: str
    session_basename: str
    warnings: Tuple[str, ...]
    planner_submit_rate_hz: float = 0.0
    completed_result_queue_depth: int = 0
    completed_result_queue_capacity: int = 1
    completed_result_queue_overflow_count: int = 0

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
        if self.completed_result_queue_depth < 0:
            raise ValueError("completed_result_queue_depth must be non-negative")
        if self.completed_result_queue_capacity <= 0:
            raise ValueError("completed_result_queue_capacity must be positive")
        if self.completed_result_queue_overflow_count < 0:
            raise ValueError(
                "completed_result_queue_overflow_count must be non-negative"
            )

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
            "planner_submit_rate_hz": self.planner_submit_rate_hz,
            "completed_result_queue_depth": self.completed_result_queue_depth,
            "completed_result_queue_capacity": self.completed_result_queue_capacity,
            "completed_result_queue_overflow_count": self.completed_result_queue_overflow_count,
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
    lifecycle_now_s: Optional[float]
    obs_now_s: Optional[float]
    active_track_id: Optional[int] = None
    decision_track_id: Optional[int] = None
    failure_reason: Optional[str] = None
    cancel_reason: Optional[str] = None
    consumed: bool = False
    consumed_track_ids: Tuple[int, ...] = ()
    locked_track_id: Optional[int] = None
    locked_strike_type: Optional[str] = None
    locked_base_target_xy: Optional[Tuple[float, float]] = None
    locked_strike_deadline_monotonic_s: Optional[float] = None
    in_commit_window: bool = False
    policy_tts_s: float = 0.0
    strike_count: int = 0
    completed_result_queue_depth: int = 0
    completed_result_queue_capacity: int = 1
    completed_result_queue_overflow_count: int = 0
    wall_time_us: Optional[int] = None

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "consumed_track_ids",
            tuple(self.consumed_track_ids),
        )
        if self.locked_base_target_xy is not None:
            base = tuple(float(value) for value in self.locked_base_target_xy)
            if len(base) != 2 or not all(math.isfinite(value) for value in base):
                raise ValueError("locked_base_target_xy must be a finite 2-vector")
            object.__setattr__(self, "locked_base_target_xy", base)

    def to_json_dict(self) -> Dict[str, Any]:
        return _json_dict({
            "schema_version": SCHEMA_VERSION,
            "phase": self.phase,
            "last_decision": self.last_decision,
            "active_key": _optional_key_json(self.active_key),
            "active_track_id": self.active_track_id,
            "decision_track_id": self.decision_track_id,
            "failure_reason": self.failure_reason,
            "cancel_reason": self.cancel_reason,
            "consumed": self.consumed,
            "consumed_track_ids": self.consumed_track_ids,
            "locked_track_id": self.locked_track_id,
            "locked_strike_type": self.locked_strike_type,
            "locked_base_target_xy": self.locked_base_target_xy,
            "locked_strike_deadline_monotonic_s": (
                self.locked_strike_deadline_monotonic_s
            ),
            "in_commit_window": self.in_commit_window,
            "policy_tts_s": self.policy_tts_s,
            "strike_count": self.strike_count,
            "completed_result_queue_depth": self.completed_result_queue_depth,
            "completed_result_queue_capacity": (
                self.completed_result_queue_capacity
            ),
            "completed_result_queue_overflow_count": (
                self.completed_result_queue_overflow_count
            ),
            "lifecycle_now_s": self.lifecycle_now_s,
            "obs_now_s": self.obs_now_s,
            "wall_time_us": self.wall_time_us,
        })

    @property
    def cached_key(self) -> None:
        return None


@dataclass(frozen=True)
class SubjectHealth:
    status: str
    rate_hz: Optional[float]
    age_s: Optional[float]
    source_frame: Optional[int]
    valid: Optional[bool]
    occluded: Optional[bool]

    def to_json_dict(self) -> Dict[str, Any]:
        return _json_dict({
            "status": self.status,
            "rate_hz": self.rate_hz,
            "age_s": self.age_s,
            "source_frame": self.source_frame,
            "valid": self.valid,
            "occluded": self.occluded,
        })


@dataclass(frozen=True)
class BallDiagnosticState:
    status: str
    estimator_sample_count: Optional[int]
    estimator_window_size: Optional[int]
    speed_mps: Optional[float]
    velocity_world_mps: Optional[Tuple[float, float, float]]
    incoming_count: Optional[int]
    incoming_required_count: Optional[int]
    incoming_status: str
    blocker: Optional[str]
    track_id: Optional[int] = None
    identity_source: Optional[str] = None
    consumed: bool = False
    source_frame: Optional[int] = None
    generation: Optional[int] = None
    raw_position_w: Optional[Tuple[float, float, float]] = None
    estimated_position_w: Optional[Tuple[float, float, float]] = None
    observed_monotonic_s: Optional[float] = None
    age_s: Optional[float] = None
    last_estimator_reset_reason: Optional[str] = None

    def __post_init__(self) -> None:
        for field_name in (
            "velocity_world_mps",
            "raw_position_w",
            "estimated_position_w",
        ):
            value = getattr(self, field_name)
            if value is None:
                continue
            try:
                vector = tuple(float(item) for item in value)
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    "{} must be a finite 3-vector".format(field_name)
                ) from exc
            if (
                len(vector) != 3
                or not all(math.isfinite(item) for item in vector)
            ):
                raise ValueError(
                    "{} must be a finite 3-vector".format(field_name)
                )
            object.__setattr__(self, field_name, vector)
        if self.track_id is not None and (
            type(self.track_id) is not int or self.track_id < 0
        ):
            raise ValueError("track_id must be a non-negative integer or None")
        if self.identity_source is not None and self.identity_source not in (
            "wire_v2",
            "legacy_inferred",
        ):
            raise ValueError(
                "identity_source must be wire_v2, legacy_inferred, or None"
            )
        for field_name in (
            "speed_mps",
            "observed_monotonic_s",
            "age_s",
        ):
            value = getattr(self, field_name)
            if value is not None and not math.isfinite(float(value)):
                raise ValueError("{} must be finite".format(field_name))

    @property
    def estimated_velocity_w(
        self,
    ) -> Optional[Tuple[float, float, float]]:
        return self.velocity_world_mps

    @property
    def velocity_x_mps(self) -> Optional[float]:
        velocity = self.velocity_world_mps
        return None if velocity is None else float(velocity[0])

    @property
    def estimator_ready(self) -> Optional[bool]:
        status = self.status.upper()
        if status == "READY":
            return True
        if status in ("ESTIMATING", "INVALID", "TRACK_ENDED"):
            return False
        return None

    @property
    def ball_only_incoming_count(self) -> Optional[int]:
        return self.incoming_count

    @property
    def ball_only_incoming_required(self) -> Optional[int]:
        return self.incoming_required_count

    @property
    def ball_only_incoming_confirmed(self) -> Optional[bool]:
        if (
            self.incoming_count is None
            or self.incoming_required_count is None
        ):
            return None
        return bool(
            self.incoming_count >= self.incoming_required_count
        )

    def to_live_json_dict(self) -> Dict[str, Any]:
        return _json_dict({
            "status": self.status,
            "source_frame": self.source_frame,
            "track_id": self.track_id,
            "identity_source": self.identity_source,
            "consumed": self.consumed,
            "generation": self.generation,
            "raw_position_w": self.raw_position_w,
            "estimated_position_w": self.estimated_position_w,
            "estimated_velocity_w": self.estimated_velocity_w,
            "speed_mps": self.speed_mps,
            "velocity_x_mps": self.velocity_x_mps,
            "estimator_sample_count": self.estimator_sample_count,
            "estimator_window_size": self.estimator_window_size,
            "estimator_ready": self.estimator_ready,
            "last_estimator_reset_reason": (
                self.last_estimator_reset_reason
            ),
            "ball_only_incoming_count": (
                self.ball_only_incoming_count
            ),
            "ball_only_incoming_required": (
                self.ball_only_incoming_required
            ),
            "ball_only_incoming_confirmed": (
                self.ball_only_incoming_confirmed
            ),
            "incoming_status": self.incoming_status,
            "blocker": self.blocker,
            "observed_monotonic_s": self.observed_monotonic_s,
            "age_s": self.age_s,
        })

    def to_json_dict(self) -> Dict[str, Any]:
        return _json_dict({
            "schema_version": SCHEMA_VERSION,
            "status": self.status,
            "track_id": self.track_id,
            "identity_source": self.identity_source,
            "consumed": self.consumed,
            "source_frame": self.source_frame,
            "generation": self.generation,
            "estimator_sample_count": self.estimator_sample_count,
            "estimator_window_size": self.estimator_window_size,
            "speed_mps": self.speed_mps,
            "velocity_world_mps": self.velocity_world_mps,
            "incoming_count": self.incoming_count,
            "incoming_required_count": self.incoming_required_count,
            "incoming_status": self.incoming_status,
            "blocker": self.blocker,
        })


@dataclass(frozen=True)
class ProductionGateState:
    pelvis_status: str
    production_incoming_status: str
    production_incoming_count: Optional[int]
    production_incoming_required: Optional[int]
    planner_status: str
    planner_reason_code: Optional[str]
    planner_tts_s: Optional[float]
    arm_status: str
    arm_trigger_tts_s: Optional[float]
    task_observation_status: str
    task_observation_valid_dimensions: Optional[int]
    task_observation_total_dimensions: Optional[int]
    task_observation_clip_count: Optional[int]

    def to_json_dict(self) -> Dict[str, Any]:
        return _json_dict({
            "pelvis_status": self.pelvis_status,
            "production_incoming_status": (
                self.production_incoming_status
            ),
            "production_incoming_count": self.production_incoming_count,
            "production_incoming_required": (
                self.production_incoming_required
            ),
            "planner_status": self.planner_status,
            "planner_reason_code": self.planner_reason_code,
            "planner_tts_s": self.planner_tts_s,
            "arm_status": self.arm_status,
            "arm_trigger_tts_s": self.arm_trigger_tts_s,
            "task_observation_status": self.task_observation_status,
            "task_observation_valid_dimensions": (
                self.task_observation_valid_dimensions
            ),
            "task_observation_total_dimensions": (
                self.task_observation_total_dimensions
            ),
            "task_observation_clip_count": (
                self.task_observation_clip_count
            ),
        })


@dataclass(frozen=True)
class LiveDiagnosticSnapshot:
    revision: int
    captured_monotonic_s: float
    subjects: Mapping[str, SubjectHealth]
    ball: BallDiagnosticState
    production_gate: ProductionGateState
    current_attempt: Optional["AttemptSummary"]
    schema_version: int = LIVE_SNAPSHOT_SCHEMA_VERSION

    def __post_init__(self) -> None:
        if self.schema_version != LIVE_SNAPSHOT_SCHEMA_VERSION:
            raise ValueError("unsupported live snapshot schema_version")
        if self.revision <= 0:
            raise ValueError("revision must be positive")
        if not math.isfinite(float(self.captured_monotonic_s)):
            raise ValueError("captured_monotonic_s must be finite")
        subjects = {
            str(name): value
            for name, value in self.subjects.items()
        }
        if not all(
            isinstance(value, SubjectHealth)
            for value in subjects.values()
        ):
            raise TypeError("subjects must contain SubjectHealth values")
        object.__setattr__(
            self,
            "subjects",
            MappingProxyType(subjects),
        )

    def to_json_dict(self) -> Dict[str, Any]:
        return _json_dict({
            "schema_version": self.schema_version,
            "revision": self.revision,
            "captured_monotonic_s": self.captured_monotonic_s,
            "subjects": {
                name: value.to_json_dict()
                for name, value in self.subjects.items()
            },
            "ball": self.ball.to_live_json_dict(),
            "production_gate": self.production_gate.to_json_dict(),
            "current_attempt": (
                None
                if self.current_attempt is None
                else self.current_attempt.to_json_dict()
            ),
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
    track_id: Optional[int] = None
    identity_source: str = "legacy_inferred"
    consumed: bool = False
    cancel_reason: Optional[str] = None
    failure_reason: Optional[str] = None
    locked_strike_type: Optional[str] = None
    locked_base_target_xy: Optional[Tuple[float, float]] = None
    locked_strike_deadline_monotonic_s: Optional[float] = None
    strike_table_y_w: Optional[float] = None
    strike_side_source: Optional[str] = None
    expected_strike_type_from_table_y: Optional[str] = None
    strike_type_consistent: Optional[bool] = None
    strike_count: int = 0
    estimator_sample_count: Optional[int] = None
    estimator_window_size: Optional[int] = None
    incoming_count: Optional[int] = None
    incoming_required_count: Optional[int] = None

    def __post_init__(self) -> None:
        if self.attempt_id <= 0:
            raise ValueError("attempt_id must be positive")
        track_id = self.attempt_id if self.track_id is None else self.track_id
        object.__setattr__(self, "track_id", track_id)
        if type(track_id) is not int or track_id <= 0:
            raise ValueError("track_id must be positive")
        if self.identity_source not in ("wire_v2", "legacy_inferred"):
            raise ValueError(
                "identity_source must be wire_v2 or legacy_inferred"
            )
        if self.locked_base_target_xy is not None:
            base = tuple(float(value) for value in self.locked_base_target_xy)
            if len(base) != 2 or not all(math.isfinite(value) for value in base):
                raise ValueError("locked_base_target_xy must be a finite 2-vector")
            object.__setattr__(self, "locked_base_target_xy", base)

    def to_json_dict(self) -> Dict[str, Any]:
        return _json_dict({
            "schema_version": SCHEMA_VERSION,
            "attempt_id": self.attempt_id,
            "track_id": self.track_id,
            "identity_source": self.identity_source,
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
            "consumed": self.consumed,
            "cancel_reason": self.cancel_reason,
            "failure_reason": self.failure_reason,
            "locked_strike_type": self.locked_strike_type,
            "locked_base_target_xy": self.locked_base_target_xy,
            "locked_strike_deadline_monotonic_s": (
                self.locked_strike_deadline_monotonic_s
            ),
            "strike_table_y_w": self.strike_table_y_w,
            "strike_side_source": self.strike_side_source,
            "expected_strike_type_from_table_y": (
                self.expected_strike_type_from_table_y
            ),
            "strike_type_consistent": self.strike_type_consistent,
            "strike_count": self.strike_count,
            "estimator_sample_count": self.estimator_sample_count,
            "estimator_window_size": self.estimator_window_size,
            "incoming_count": self.incoming_count,
            "incoming_required_count": self.incoming_required_count,
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
