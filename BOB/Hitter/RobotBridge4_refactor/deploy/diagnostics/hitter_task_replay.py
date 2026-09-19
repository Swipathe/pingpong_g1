from __future__ import annotations

from collections import OrderedDict, deque
from dataclasses import dataclass, field, fields, is_dataclass, replace
import heapq
import json
import math
import multiprocessing
import os
from pathlib import Path
import queue
import stat
import threading
import time
from types import SimpleNamespace
from typing import (
    Any,
    Callable,
    Deque,
    Dict,
    Mapping,
    Optional,
    Protocol,
    Sequence,
    Tuple,
)

import numpy as np

from diagnostics.hitter_task_models import (
    AttemptTransition,
    EventDraft,
    NormalizedMocapSample,
    SnapshotKey,
)
from diagnostics.hitter_task_pipeline import (
    AdapterOutput,
    MocapFrameAdapter,
    ShadowTaskPipeline,
    planner_reason_code,
)
from utils.hitter_realtime import (
    BallEstimateSnapshot,
    FrozenPlannerResult,
    PlannerResultSnapshot,
)
from utils.hitter_serialization import JsonValue, freeze_json_value, to_builtin_json
from utils.hitter_runtime_factory import (
    HitterRuntimeSettings,
    build_hitter_system_planner,
)
from utils.hitter_runtime_types import PlannerFailureReason

BASELINE_VARIANT = None
ONE_FRAME_VARIANT = None

_TERMINAL_CODES = frozenset(
    (
        "TASK_OBS_PASS",
        "LATE_SKIP",
        "TRACK_ENDED_BEFORE_READY",
        "TRACK_ENDED_BEFORE_CONFIRMATION",
        "NO_VALID_PLAN",
        "PELVIS_UNAVAILABLE",
        "MALFORMED_COMMAND",
        "RECORDING_INCOMPLETE",
        "REPLAY_DIVERGENCE",
    )
)
_SUMMARY_LABELS = frozenset(
    (
        "SAME_PASS",
        "SAVED_BY_ONE_FRAME",
        "BASELINE_ONLY_PASS",
        "BOTH_FAIL_SAME",
        "BOTH_FAIL_DIFFERENT",
        "BOTH_PASS_DIFFERENT_COMMAND",
        "INCONCLUSIVE",
    )
)
_EVENT_PRIORITIES = {
    "INPUT": 0,
    "PLAN_COMPLETE": 1,
    "PLAN_START": 2,
    "LIFECYCLE_TICK": 3,
    "OBS_TICK": 4,
    "GRACE_EXPIRE": 5,
}
_PARITY_ATOL = 1.0e-6
_BOUNDARY_WARNING_S = 0.005
_TIME_EPSILON_S = 1.0e-12
_MAX_REPLAY_CAPTURE_INPUTS = 8192
_MAX_REPLAY_CAPTURE_EVENTS = 8192


def _frozen_mapping(value: Mapping[str, Any]) -> Mapping[str, JsonValue]:
    frozen = freeze_json_value(value)
    if not isinstance(frozen, Mapping):
        raise TypeError("expected a mapping")
    return frozen


def _optional_float(value: Any) -> Optional[float]:
    if value is None:
        return None
    result = float(value)
    return result if math.isfinite(result) else None


def _has_required_command_fields(
    value: Optional[Mapping[str, Any]],
) -> bool:
    if not isinstance(value, Mapping):
        return False
    strike_plan = value.get("strike_plan")
    return bool(
        "p_base_target_xy" in value
        and "v_racket_target_w" in value
        and "time_to_strike" in value
        and isinstance(strike_plan, Mapping)
        and "p_racket_target" in strike_plan
    )


@dataclass(frozen=True)
class ReplayVariant:
    name: str
    planner_rate_hz: float
    incoming_confirmations: int

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("name must be non-empty")
        rate = float(self.planner_rate_hz)
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError("planner_rate_hz must be finite and positive")
        if abs(rate - 360.0) <= _TIME_EPSILON_S:
            raise ValueError("360 Hz full-planner replay is not supported")
        if isinstance(self.incoming_confirmations, bool) or int(self.incoming_confirmations) < 1:
            raise ValueError("incoming_confirmations must be positive")
        object.__setattr__(self, "planner_rate_hz", rate)
        object.__setattr__(
            self,
            "incoming_confirmations",
            int(self.incoming_confirmations),
        )


BASELINE_VARIANT = ReplayVariant("3/100", 100.0, 3)
ONE_FRAME_VARIANT = ReplayVariant("1/100", 100.0, 1)


@dataclass(frozen=True)
class ReplayOutcome:
    variant: str
    terminal_code: str
    task_obs_pass: bool
    boundary_sensitive: bool
    recording_complete: bool
    warnings: Tuple[str, ...]
    summary_label: Optional[str]
    metrics: Mapping[str, Any]

    def __post_init__(self) -> None:
        if self.terminal_code not in _TERMINAL_CODES:
            raise ValueError("terminal_code must be one of {}".format(sorted(_TERMINAL_CODES)))
        if self.summary_label is not None and self.summary_label not in _SUMMARY_LABELS:
            raise ValueError("invalid summary_label")
        warnings = list(dict.fromkeys(str(item) for item in self.warnings))
        if self.boundary_sensitive and "BOUNDARY_SENSITIVE" not in warnings:
            warnings.append("BOUNDARY_SENSITIVE")
        object.__setattr__(self, "warnings", tuple(warnings))
        object.__setattr__(self, "metrics", _frozen_mapping(self.metrics))


@dataclass(frozen=True)
class PolicyTickRecord:
    tick_index: int
    lifecycle_now_s: float
    obs_now_s: float
    lifecycle_decision: str
    phase: str
    active_key: Optional[SnapshotKey]
    cached_key: Optional[SnapshotKey]
    command_fields: Optional[Mapping[str, JsonValue]]
    task_pre_clip: Optional[Tuple[float, ...]]
    task_post_clip: Optional[Tuple[float, ...]]
    task_clip_count: Optional[int]

    def __post_init__(self) -> None:
        if self.tick_index < 0:
            raise ValueError("tick_index must be non-negative")
        if self.command_fields is not None:
            object.__setattr__(
                self,
                "command_fields",
                _frozen_mapping(self.command_fields),
            )
        for name in ("task_pre_clip", "task_post_clip"):
            value = getattr(self, name)
            if value is not None:
                object.__setattr__(
                    self,
                    name,
                    tuple(float(item) for item in value),
                )


@dataclass(frozen=True)
class PlannerCallRecord:
    snapshot_key: SnapshotKey
    submitted_monotonic_s: float
    started_monotonic_s: float
    completed_monotonic_s: float
    duration_s: float
    pending_replaced_key: Optional[SnapshotKey]
    latest_replaced_key: Optional[SnapshotKey]
    result: FrozenPlannerResult


@dataclass(frozen=True)
class OnlineAttemptTrace:
    attempt_id: int
    stages: Tuple[AttemptTransition, ...]
    planner_calls: Tuple[PlannerCallRecord, ...]
    policy_ticks: Tuple[PolicyTickRecord, ...]
    terminal_code: str
    recovery_duration_s: Optional[float]
    submission_phase_anchor_s: Optional[float]
    recording_complete: bool

    def __post_init__(self) -> None:
        if self.attempt_id <= 0:
            raise ValueError("attempt_id must be positive")
        if self.terminal_code not in _TERMINAL_CODES:
            raise ValueError("invalid terminal_code")
        object.__setattr__(self, "stages", tuple(self.stages))
        object.__setattr__(self, "planner_calls", tuple(self.planner_calls))
        object.__setattr__(self, "policy_ticks", tuple(self.policy_ticks))


@dataclass(frozen=True)
class ReplayInputBundle:
    attempt_id: int
    inputs: Tuple[NormalizedMocapSample, ...]
    online_baseline: OnlineAttemptTrace
    recording_complete: bool
    planner_config: Mapping[str, JsonValue] = field(default_factory=dict)
    runtime_settings: Optional[HitterRuntimeSettings] = None
    forced_strike_type: Optional[str] = None
    capture_metadata: Mapping[str, JsonValue] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.attempt_id <= 0:
            raise ValueError("attempt_id must be positive")
        if self.attempt_id != self.online_baseline.attempt_id:
            raise ValueError("attempt_id must match online_baseline.attempt_id")
        object.__setattr__(self, "inputs", tuple(self.inputs))
        object.__setattr__(
            self,
            "planner_config",
            _frozen_mapping(self.planner_config),
        )
        object.__setattr__(
            self,
            "capture_metadata",
            _frozen_mapping(self.capture_metadata),
        )
        forced = self.forced_strike_type
        if forced is not None:
            forced = str(forced).strip().lower()
            if forced not in ("forehand", "backhand"):
                raise ValueError("forced_strike_type must be forehand/backhand/None")
            object.__setattr__(self, "forced_strike_type", forced)


def _json_number(value: Any) -> float:
    if value == "NaN":
        return float("nan")
    if value == "Infinity":
        return float("inf")
    if value == "-Infinity":
        return float("-inf")
    return float(value)


def _json_optional_number(value: Any) -> Optional[float]:
    return None if value is None else _json_number(value)


def _snapshot_key_from_json(value: Any) -> Optional[SnapshotKey]:
    if value is None:
        return None
    if not isinstance(value, Mapping):
        raise TypeError("snapshot key must be a mapping")
    return SnapshotKey(
        track_id=int(value["track_id"]),
        generation=int(value["generation"]),
    )


def _canonical_stage_identity(
    transition: AttemptTransition,
) -> Optional[Tuple[Any, ...]]:
    stage = str(transition.stage)
    if stage == "ESTIMATOR_READY":
        return (stage,)
    if stage == "INCOMING_CONFIRMING":
        return (
            stage,
            None if transition.snapshot_key is None else transition.snapshot_key.track_id,
            transition.values.get("count"),
        )
    if stage == "INCOMING_CONFIRMED":
        return (stage,)
    if stage in ("PLANNER_SUCCEEDED", "PLANNER_REJECTED"):
        return (stage, transition.snapshot_key)
    if stage in ("ARMED", "LATE_SKIP"):
        return (stage,)
    if stage == "TASK_OBS_FAILED":
        return (stage, transition.reason_code)
    return None


def _deduplicate_canonical_stages(
    transitions: Sequence[AttemptTransition],
) -> Tuple[AttemptTransition, ...]:
    seen = set()
    result = []
    for transition in transitions:
        identity = _canonical_stage_identity(transition)
        if identity is not None:
            if identity in seen:
                continue
            seen.add(identity)
        result.append(transition)
    return tuple(result)


def _sample_from_json(value: Mapping[str, Any]) -> NormalizedMocapSample:
    return NormalizedMocapSample(
        input_seq=int(value["input_seq"]),
        channel=str(value["channel"]),
        subject=str(value["subject"]),
        position_w=np.asarray(value["position_w"], dtype=np.float64),
        quaternion_xyzw=np.asarray(
            value["quaternion_xyzw"],
            dtype=np.float64,
        ),
        valid=bool(value["valid"]),
        occluded=bool(value["occluded"]),
        source_frame=int(value["source_frame"]),
        source_time_s=_json_optional_number(value.get("source_time_s")),
        publish_time_us=(None if value.get("publish_time_us") is None else int(value["publish_time_us"])),
        received_monotonic_s=_json_number(value["received_monotonic_s"]),
        wall_time_us=int(value["wall_time_us"]),
        payload_size=int(value["payload_size"]),
    )


def _transition_from_json(
    value: Mapping[str, Any],
) -> AttemptTransition:
    segment = value.get("track_segment_id")
    reason = value.get("reason_code")
    values = value.get("values", {})
    if not isinstance(values, Mapping):
        raise TypeError("attempt transition values must be a mapping")
    return AttemptTransition(
        attempt_id=int(value["attempt_id"]),
        track_segment_id=None if segment is None else int(segment),
        stage=str(value["stage"]),
        monotonic_s=_json_number(value["monotonic_s"]),
        snapshot_key=_snapshot_key_from_json(value.get("snapshot_key")),
        reason_code=None if reason is None else str(reason),
        values=values,
    )


def _frozen_result_to_json(
    value: FrozenPlannerResult,
) -> Mapping[str, Any]:
    return {
        "snapshot_key": value.snapshot_key.to_json_dict(),
        "source_frame": value.source_frame,
        "strike_deadline_monotonic_s": (value.strike_deadline_monotonic_s),
        "completed_monotonic_s": value.completed_monotonic_s,
        "command_fields": value.command_fields,
        "error_type": value.error_type,
        "error_text": value.error_text,
        "failure_reason": (None if value.failure_reason is None else value.failure_reason.value),
    }


def _frozen_result_from_json(
    value: Mapping[str, Any],
) -> FrozenPlannerResult:
    key = _snapshot_key_from_json(value["snapshot_key"])
    if key is None:
        raise ValueError("planner result requires a snapshot key")
    command_fields = value.get("command_fields")
    if command_fields is not None and not isinstance(command_fields, Mapping):
        raise TypeError("command_fields must be a mapping or null")
    raw_failure_reason = value.get("failure_reason")
    failure_reason = (
        PlannerFailureReason.INTERNAL_ERROR
        if raw_failure_reason is None and (value.get("error_type") is not None or value.get("error_text") is not None)
        else (None if raw_failure_reason is None else PlannerFailureReason(raw_failure_reason))
    )
    return FrozenPlannerResult(
        snapshot_key=key,
        source_frame=int(value["source_frame"]),
        strike_deadline_monotonic_s=_json_number(value["strike_deadline_monotonic_s"]),
        completed_monotonic_s=_json_number(value["completed_monotonic_s"]),
        command_fields=command_fields,
        error_type=(None if value.get("error_type") is None else str(value["error_type"])),
        error_text=(None if value.get("error_text") is None else str(value["error_text"])),
        failure_reason=failure_reason,
    )


def _planner_call_to_json(value: PlannerCallRecord) -> Mapping[str, Any]:
    return {
        "snapshot_key": value.snapshot_key.to_json_dict(),
        "submitted_monotonic_s": value.submitted_monotonic_s,
        "started_monotonic_s": value.started_monotonic_s,
        "completed_monotonic_s": value.completed_monotonic_s,
        "duration_s": value.duration_s,
        "pending_replaced_key": (
            None if value.pending_replaced_key is None else value.pending_replaced_key.to_json_dict()
        ),
        "latest_replaced_key": (
            None if value.latest_replaced_key is None else value.latest_replaced_key.to_json_dict()
        ),
        "result": _frozen_result_to_json(value.result),
    }


def _planner_call_from_json(
    value: Mapping[str, Any],
) -> PlannerCallRecord:
    key = _snapshot_key_from_json(value["snapshot_key"])
    if key is None:
        raise ValueError("planner call requires a snapshot key")
    result = value.get("result")
    if not isinstance(result, Mapping):
        raise TypeError("planner call result must be a mapping")
    return PlannerCallRecord(
        snapshot_key=key,
        submitted_monotonic_s=_json_number(value["submitted_monotonic_s"]),
        started_monotonic_s=_json_number(value["started_monotonic_s"]),
        completed_monotonic_s=_json_number(value["completed_monotonic_s"]),
        duration_s=_json_number(value["duration_s"]),
        pending_replaced_key=_snapshot_key_from_json(value.get("pending_replaced_key")),
        latest_replaced_key=_snapshot_key_from_json(value.get("latest_replaced_key")),
        result=_frozen_result_from_json(result),
    )


def _policy_tick_to_json(value: PolicyTickRecord) -> Mapping[str, Any]:
    return {
        "tick_index": value.tick_index,
        "lifecycle_now_s": value.lifecycle_now_s,
        "obs_now_s": value.obs_now_s,
        "lifecycle_decision": value.lifecycle_decision,
        "phase": value.phase,
        "active_key": (None if value.active_key is None else value.active_key.to_json_dict()),
        "cached_key": (None if value.cached_key is None else value.cached_key.to_json_dict()),
        "command_fields": value.command_fields,
        "task_pre_clip": value.task_pre_clip,
        "task_post_clip": value.task_post_clip,
        "task_clip_count": value.task_clip_count,
    }


def _optional_float_tuple(value: Any) -> Optional[Tuple[float, ...]]:
    if value is None:
        return None
    return tuple(_json_number(item) for item in value)


def _policy_tick_from_json(
    value: Mapping[str, Any],
) -> PolicyTickRecord:
    command_fields = value.get("command_fields")
    if command_fields is not None and not isinstance(command_fields, Mapping):
        raise TypeError("policy command_fields must be a mapping or null")
    clip_count = value.get("task_clip_count")
    return PolicyTickRecord(
        tick_index=int(value["tick_index"]),
        lifecycle_now_s=_json_number(value["lifecycle_now_s"]),
        obs_now_s=_json_number(value["obs_now_s"]),
        lifecycle_decision=str(value["lifecycle_decision"]),
        phase=str(value["phase"]),
        active_key=_snapshot_key_from_json(value.get("active_key")),
        cached_key=_snapshot_key_from_json(value.get("cached_key")),
        command_fields=command_fields,
        task_pre_clip=_optional_float_tuple(value.get("task_pre_clip")),
        task_post_clip=_optional_float_tuple(value.get("task_post_clip")),
        task_clip_count=None if clip_count is None else int(clip_count),
    )


def _runtime_settings_to_json(
    value: Optional[HitterRuntimeSettings],
) -> Optional[Mapping[str, Any]]:
    if value is None:
        return None
    return {item.name: getattr(value, item.name) for item in fields(HitterRuntimeSettings)}


def _runtime_settings_from_json(
    value: Any,
) -> Optional[HitterRuntimeSettings]:
    if value is None:
        return None
    if not isinstance(value, Mapping):
        raise TypeError("runtime_settings must be a mapping or null")
    return HitterRuntimeSettings(
        estimator_sample_rate_hz=_json_number(value["estimator_sample_rate_hz"]),
        planner_update_rate_hz=_json_number(value["planner_update_rate_hz"]),
        planner_update_interval_s=_json_number(value["planner_update_interval_s"]),
        minimum_incoming_speed_x_mps=_json_number(value["minimum_incoming_speed_x_mps"]),
        incoming_confirmation_snapshots=int(value["incoming_confirmation_snapshots"]),
        waiting_tts_s=_json_number(value["waiting_tts_s"]),
        arm_tts_s=_json_number(value["arm_tts_s"]),
        minimum_arm_tts_s=_json_number(value["minimum_arm_tts_s"]),
        maximum_policy_tts_s=_json_number(value["maximum_policy_tts_s"]),
        swing_duration_range_s=tuple(_json_number(item) for item in value["swing_duration_range_s"]),
        hitter_seed=(None if value.get("hitter_seed") is None else int(value["hitter_seed"])),
        control_tick_s=_json_number(value["control_tick_s"]),
        obs_clip_value=_json_optional_number(value.get("obs_clip_value")),
    )


def replay_input_bundle_to_json_dict(
    bundle: ReplayInputBundle,
) -> Dict[str, Any]:
    """Return the complete, canonical spawn-safe replay input envelope."""
    online = bundle.online_baseline
    value = {
        "schema_version": 1,
        "kind": "hitter_task_replay_input",
        "attempt_id": bundle.attempt_id,
        "inputs": [sample.to_json_dict() for sample in bundle.inputs],
        "online_baseline": {
            "attempt_id": online.attempt_id,
            "stages": [stage.to_json_dict() for stage in online.stages],
            "planner_calls": [_planner_call_to_json(call) for call in online.planner_calls],
            "policy_ticks": [_policy_tick_to_json(tick) for tick in online.policy_ticks],
            "terminal_code": online.terminal_code,
            "recovery_duration_s": online.recovery_duration_s,
            "submission_phase_anchor_s": (online.submission_phase_anchor_s),
            "recording_complete": online.recording_complete,
        },
        "recording_complete": bundle.recording_complete,
        "planner_config": bundle.planner_config,
        "runtime_settings": _runtime_settings_to_json(bundle.runtime_settings),
        "forced_strike_type": bundle.forced_strike_type,
        "capture_metadata": bundle.capture_metadata,
    }
    converted = to_builtin_json(value)
    if not isinstance(converted, dict):
        raise TypeError("replay bundle must serialize to a JSON object")
    return converted


def replay_input_bundle_from_json_dict(
    value: Mapping[str, Any],
) -> ReplayInputBundle:
    """Load and validate one canonical replay input envelope."""
    if int(value.get("schema_version", -1)) != 1:
        raise ValueError("unsupported replay input schema_version")
    if value.get("kind") != "hitter_task_replay_input":
        raise ValueError("invalid replay input kind")
    online_value = value.get("online_baseline")
    if not isinstance(online_value, Mapping):
        raise TypeError("online_baseline must be a mapping")
    planner_config = value.get("planner_config", {})
    if not isinstance(planner_config, Mapping):
        raise TypeError("planner_config must be a mapping")
    online = OnlineAttemptTrace(
        attempt_id=int(online_value["attempt_id"]),
        stages=tuple(_transition_from_json(item) for item in online_value.get("stages", ())),
        planner_calls=tuple(_planner_call_from_json(item) for item in online_value.get("planner_calls", ())),
        policy_ticks=tuple(_policy_tick_from_json(item) for item in online_value.get("policy_ticks", ())),
        terminal_code=str(online_value["terminal_code"]),
        recovery_duration_s=_json_optional_number(online_value.get("recovery_duration_s")),
        submission_phase_anchor_s=_json_optional_number(online_value.get("submission_phase_anchor_s")),
        recording_complete=bool(online_value["recording_complete"]),
    )
    return ReplayInputBundle(
        attempt_id=int(value["attempt_id"]),
        inputs=tuple(_sample_from_json(item) for item in value.get("inputs", ())),
        online_baseline=online,
        recording_complete=bool(value["recording_complete"]),
        planner_config=planner_config,
        runtime_settings=_runtime_settings_from_json(value.get("runtime_settings")),
        forced_strike_type=(None if value.get("forced_strike_type") is None else str(value["forced_strike_type"])),
        capture_metadata=value.get("capture_metadata", {}),
    )


def _open_private_directory(path: Path) -> int:
    path.mkdir(mode=0o700, parents=True, exist_ok=True)
    flags = os.O_RDONLY
    flags |= getattr(os, "O_DIRECTORY", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(str(path), flags)
    try:
        if not stat.S_ISDIR(os.fstat(descriptor).st_mode):
            raise OSError("replay input parent is not a directory")
        try:
            os.chmod(str(path), 0o700, follow_symlinks=False)
        except (NotImplementedError, OSError):
            pass
    except BaseException:
        os.close(descriptor)
        raise
    return descriptor


def write_replay_input_bundle(
    path: Path,
    bundle: ReplayInputBundle,
) -> None:
    """Atomically persist a mode-0600 replay file without following links."""
    target = Path(path)
    if target.name in ("", ".", ".."):
        raise ValueError("replay input path requires a file name")
    payload = (
        json.dumps(
            replay_input_bundle_to_json_dict(bundle),
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")
    directory_fd = _open_private_directory(target.parent)
    temporary_name = ".{}.tmp-{}-{}-{}".format(
        target.name,
        os.getpid(),
        threading.get_ident(),
        time.monotonic_ns(),
    )
    descriptor = None
    try:
        try:
            current = os.stat(
                target.name,
                dir_fd=directory_fd,
                follow_symlinks=False,
            )
        except FileNotFoundError:
            current = None
        if current is not None and stat.S_ISLNK(current.st_mode):
            raise OSError("refusing to replace replay input symlink")
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
        flags |= getattr(os, "O_NOFOLLOW", 0)
        descriptor = os.open(
            temporary_name,
            flags,
            0o600,
            dir_fd=directory_fd,
        )
        view = memoryview(payload)
        while view:
            written = os.write(descriptor, view)
            view = view[written:]
        os.fsync(descriptor)
        os.close(descriptor)
        descriptor = None
        os.replace(
            temporary_name,
            target.name,
            src_dir_fd=directory_fd,
            dst_dir_fd=directory_fd,
        )
        os.fsync(directory_fd)
    finally:
        if descriptor is not None:
            os.close(descriptor)
        try:
            os.unlink(temporary_name, dir_fd=directory_fd)
        except FileNotFoundError:
            pass
        os.close(directory_fd)


def load_replay_input_bundle(
    path: Path,
    *,
    maximum_bytes: int = 128 * 1024 * 1024,
) -> ReplayInputBundle:
    """Read one regular replay file without following its final symlink."""
    target = Path(path)
    flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(str(target), flags)
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode):
            raise OSError("replay input is not a regular file")
        if metadata.st_size > int(maximum_bytes):
            raise ValueError("replay input exceeds maximum_bytes")
        chunks = []
        remaining = int(maximum_bytes) + 1
        while remaining > 0:
            chunk = os.read(descriptor, min(1024 * 1024, remaining))
            if not chunk:
                break
            chunks.append(chunk)
            remaining -= len(chunk)
        if remaining <= 0 and os.read(descriptor, 1):
            raise ValueError("replay input exceeds maximum_bytes")
    finally:
        os.close(descriptor)
    decoded = json.loads(b"".join(chunks).decode("utf-8"))
    if not isinstance(decoded, Mapping):
        raise TypeError("replay input root must be a mapping")
    return replay_input_bundle_from_json_dict(decoded)


@dataclass
class _ReplayAttemptCapture:
    inputs: list = field(default_factory=list)
    input_seqs: set = field(default_factory=set)
    event_count: int = 0
    recording_complete: bool = True
    stage_records: list = field(default_factory=list)
    stage_seq: int = 0
    submissions: Dict[SnapshotKey, float] = field(default_factory=dict)
    starts: Dict[SnapshotKey, float] = field(default_factory=dict)
    pending_replaced: Dict[SnapshotKey, SnapshotKey] = field(default_factory=dict)
    latest_replaced: Dict[SnapshotKey, SnapshotKey] = field(default_factory=dict)
    results: Dict[SnapshotKey, FrozenPlannerResult] = field(default_factory=dict)
    track_segments: Dict[SnapshotKey, int] = field(default_factory=dict)
    policy_ticks: list = field(default_factory=list)
    estimator_ready_recorded: bool = False
    planner_stage_keys: set = field(default_factory=set)
    incoming_recorded: bool = False
    armed_recorded: bool = False
    late_skip_recorded: bool = False
    recovery_duration_s: Optional[float] = None
    submission_phase_anchor_s: Optional[float] = None
    last_update_s: float = 0.0


class ReplayCaptureStore:
    """Bounded in-memory capture of canonical online replay facts."""

    def __init__(
        self,
        *,
        input_capacity: int,
        event_capacity: int,
        attempt_capacity: int,
    ) -> None:
        for name, value in (
            ("input_capacity", input_capacity),
            ("event_capacity", event_capacity),
            ("attempt_capacity", attempt_capacity),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError("{} must be a positive integer".format(name))
        self._input_capacity = min(
            int(input_capacity),
            _MAX_REPLAY_CAPTURE_INPUTS,
        )
        self._event_capacity = min(
            int(event_capacity),
            _MAX_REPLAY_CAPTURE_EVENTS,
        )
        self._attempt_capacity = int(attempt_capacity)
        self._lock = threading.RLock()
        self._attempts: "OrderedDict[int, _ReplayAttemptCapture]" = OrderedDict()
        self._latest_pelvis: Optional[NormalizedMocapSample] = None
        self._active_attempt_id: Optional[int] = None
        self._evicted_attempts: Deque[int] = deque(maxlen=self._attempt_capacity)

    @property
    def capacity_metadata(self) -> Dict[str, int]:
        return {
            "input_capacity": self._input_capacity,
            "event_capacity": self._event_capacity,
            "attempt_capacity": self._attempt_capacity,
        }

    def _attempt_locked(self, attempt_id: int) -> _ReplayAttemptCapture:
        attempt = self._attempts.get(attempt_id)
        if attempt is not None:
            self._attempts.move_to_end(attempt_id)
            return attempt
        if len(self._attempts) >= self._attempt_capacity:
            evicted_id, _evicted = self._attempts.popitem(last=False)
            self._evicted_attempts.append(evicted_id)
        attempt = _ReplayAttemptCapture()
        self._attempts[attempt_id] = attempt
        return attempt

    def mark_incomplete(self, attempt_id: Optional[int]) -> None:
        with self._lock:
            if attempt_id is None:
                attempt_id = self._active_attempt_id
            if attempt_id is None:
                return
            self._attempt_locked(int(attempt_id)).recording_complete = False

    def record_raw(
        self,
        sample: NormalizedMocapSample,
        *,
        attempt_id: Optional[int],
        track_segment_id: Optional[int],
    ) -> None:
        del track_segment_id
        with self._lock:
            if sample.subject == "g1pelvis":
                self._latest_pelvis = sample
                if self._active_attempt_id is not None:
                    attempt = self._attempt_locked(self._active_attempt_id)
                    attempt.last_update_s = time.monotonic()
                    if sample.input_seq in attempt.input_seqs:
                        return
                    if len(attempt.inputs) >= self._input_capacity:
                        attempt.recording_complete = False
                        return
                    attempt.inputs.append(sample)
                    attempt.input_seqs.add(sample.input_seq)
                return
            if sample.subject != "ball":
                return
            if attempt_id is None:
                attempt_id = self._active_attempt_id
            if attempt_id is None:
                return
            attempt = self._attempt_locked(int(attempt_id))
            attempt.last_update_s = time.monotonic()
            values = []
            if not attempt.inputs and self._latest_pelvis is None:
                attempt.recording_complete = False
            elif (
                not attempt.inputs
                and self._latest_pelvis is not None
                and self._latest_pelvis.input_seq not in attempt.input_seqs
            ):
                values.append(self._latest_pelvis)
            if sample.input_seq not in attempt.input_seqs:
                values.append(sample)
            if len(attempt.inputs) + len(values) > self._input_capacity:
                attempt.recording_complete = False
                return
            attempt.inputs.extend(values)
            attempt.input_seqs.update(value.input_seq for value in values)

    @staticmethod
    def _key(value: Any) -> Optional[SnapshotKey]:
        return _snapshot_key_from_json(value)

    @staticmethod
    def _segment(value: Any) -> Optional[int]:
        return None if value is None else int(value)

    def _append_stage_locked(
        self,
        attempt: _ReplayAttemptCapture,
        *,
        attempt_id: int,
        track_segment_id: Optional[int],
        stage: str,
        monotonic_s: float,
        snapshot_key: Optional[SnapshotKey],
        reason_code: Optional[str],
        values: Mapping[str, Any],
        priority: int,
        synthetic: bool = False,
    ) -> None:
        attempt.stage_seq += 1
        attempt.stage_records.append(
            (
                float(monotonic_s),
                int(priority),
                attempt.stage_seq,
                int(bool(synthetic)),
                AttemptTransition(
                    attempt_id=int(attempt_id),
                    track_segment_id=track_segment_id,
                    stage=str(stage),
                    monotonic_s=float(monotonic_s),
                    snapshot_key=snapshot_key,
                    reason_code=reason_code,
                    values=values,
                ),
            )
        )

    def _record_transition_locked(
        self,
        attempt_id: int,
        attempt: _ReplayAttemptCapture,
        draft: EventDraft,
    ) -> None:
        payload = draft.payload
        key = self._key(payload.get("snapshot_key"))
        segment = self._segment(payload.get("track_segment_id"))
        values = payload.get("values", {})
        if not isinstance(values, Mapping):
            attempt.recording_complete = False
            return
        if key is not None and segment is not None:
            attempt.track_segments[key] = segment
        stage = str(payload.get("stage", ""))
        priority = (
            0
            if stage in ("DETECTED", "POST_DEADLINE_TAIL")
            else (
                1
                if stage in ("PLANNER_SUCCEEDED", "PLANNER_REJECTED")
                else (
                    2
                    if stage
                    in (
                        "ESTIMATOR_READY",
                        "INCOMING_CONFIRMING",
                        "INCOMING_CONFIRMED",
                    )
                    else (
                        4
                        if stage in ("TASK_OBS_PASS", "TASK_OBS_FAILED")
                        else 5 if stage in ("REACQUIRE_GRACE", "ATTEMPT_CLOSED") else 3
                    )
                )
            )
        )
        reason = payload.get("reason_code")
        self._append_stage_locked(
            attempt,
            attempt_id=attempt_id,
            track_segment_id=segment,
            stage=stage,
            monotonic_s=draft.monotonic_s,
            snapshot_key=key,
            reason_code=None if reason is None else str(reason),
            values=values,
            priority=priority,
        )
        if stage in ("DETECTED", "POST_DEADLINE_TAIL"):
            self._active_attempt_id = attempt_id
        elif stage == "ATTEMPT_CLOSED" and self._active_attempt_id == attempt_id:
            self._active_attempt_id = None

    def _record_planner_locked(
        self,
        attempt_id: int,
        attempt: _ReplayAttemptCapture,
        draft: EventDraft,
    ) -> None:
        payload = draft.payload
        key = self._key(payload.get("snapshot_key"))
        if key is None:
            attempt.recording_complete = False
            return
        segment = self._segment(payload.get("track_segment_id"))
        if segment is not None:
            attempt.track_segments[key] = segment
        kind = str(payload.get("trace_kind", ""))
        if kind == "submit":
            attempt.submissions.setdefault(key, float(draft.monotonic_s))
            if attempt.submission_phase_anchor_s is None:
                attempt.submission_phase_anchor_s = float(draft.monotonic_s)
            return
        if kind == "pending_replaced":
            replaced = self._key(payload.get("replaced_snapshot_key"))
            if replaced is not None:
                attempt.pending_replaced[key] = replaced
            return
        if kind == "start":
            attempt.starts[key] = float(draft.monotonic_s)
            if bool(payload.get("snapshot_ready")) and not attempt.estimator_ready_recorded:
                attempt.estimator_ready_recorded = True
                self._append_stage_locked(
                    attempt,
                    attempt_id=attempt_id,
                    track_segment_id=segment,
                    stage="ESTIMATOR_READY",
                    monotonic_s=draft.monotonic_s,
                    snapshot_key=key,
                    reason_code=None,
                    values={},
                    priority=2,
                    synthetic=True,
                )
            return
        if kind == "latest_result_replaced":
            replaced_result = payload.get("replaced_result")
            if isinstance(replaced_result, Mapping):
                replaced = self._key(replaced_result.get("snapshot_key"))
                if replaced is not None:
                    attempt.latest_replaced[key] = replaced
            return
        if kind != "complete":
            return
        result_value = payload.get("result")
        if not isinstance(result_value, Mapping):
            attempt.recording_complete = False
            return
        result = _frozen_result_from_json(result_value)
        attempt.results[key] = result
        start_s = attempt.starts.get(key, float(draft.monotonic_s))
        if bool(payload.get("incoming_confirmed")) and not (attempt.incoming_recorded):
            attempt.incoming_recorded = True
            self._append_stage_locked(
                attempt,
                attempt_id=attempt_id,
                track_segment_id=segment,
                stage="INCOMING_CONFIRMED",
                monotonic_s=start_s,
                snapshot_key=key,
                reason_code=None,
                values={},
                priority=2,
                synthetic=True,
            )
        if key in attempt.planner_stage_keys:
            return
        attempt.planner_stage_keys.add(key)
        if result.error_type is None and _has_required_command_fields(result.command_fields):
            stage = "PLANNER_SUCCEEDED"
            reason = None
            values = {}
        else:
            stage = "PLANNER_REJECTED"
            reason_value = result_value.get("reason_code")
            reason = (
                "MALFORMED_COMMAND"
                if result.error_type is None
                else (None if reason_value is None else str(reason_value))
            )
            values = {
                "error_type": result.error_type,
                "error_text": result.error_text,
            }
        self._append_stage_locked(
            attempt,
            attempt_id=attempt_id,
            track_segment_id=segment,
            stage=stage,
            monotonic_s=result.completed_monotonic_s,
            snapshot_key=key,
            reason_code=reason,
            values=values,
            priority=1,
            synthetic=True,
        )

    def _record_lifecycle_locked(
        self,
        attempt_id: int,
        attempt: _ReplayAttemptCapture,
        draft: EventDraft,
    ) -> None:
        payload = draft.payload
        command_result = payload.get("command_result")
        command_fields = command_result.get("command_fields") if isinstance(command_result, Mapping) else None
        if command_fields is not None and not isinstance(
            command_fields,
            Mapping,
        ):
            command_fields = None
            attempt.recording_complete = False
        tick = PolicyTickRecord(
            tick_index=len(attempt.policy_ticks),
            lifecycle_now_s=_json_number(payload["lifecycle_now_s"]),
            obs_now_s=_json_number(payload["obs_now_s"]),
            lifecycle_decision=str(payload.get("decision", "none")),
            phase=str(payload.get("phase", "waiting")),
            active_key=self._key(payload.get("active_key")),
            cached_key=self._key(payload.get("cached_key")),
            command_fields=command_fields,
            task_pre_clip=_optional_float_tuple(payload.get("task_pre_clip")),
            task_post_clip=_optional_float_tuple(payload.get("task_post_clip")),
            task_clip_count=(None if payload.get("clip_count") is None else int(payload["clip_count"])),
        )
        attempt.policy_ticks.append(tick)
        recovery = _json_optional_number(payload.get("recovery_duration_s"))
        decision = tick.lifecycle_decision
        if recovery is not None and (
            attempt.armed_recorded or decision == "armed" or tick.phase in ("armed", "recovery")
        ):
            attempt.recovery_duration_s = recovery
        if decision == "armed" and not attempt.armed_recorded:
            attempt.armed_recorded = True
            key = tick.active_key
            segment = None if key is None else attempt.track_segments.get(key)
            arm_tts = None
            if isinstance(command_result, Mapping):
                deadline = _json_optional_number(command_result.get("strike_deadline_monotonic_s"))
                if deadline is not None:
                    arm_tts = deadline - tick.lifecycle_now_s
            self._append_stage_locked(
                attempt,
                attempt_id=attempt_id,
                track_segment_id=segment,
                stage="ARMED",
                monotonic_s=tick.lifecycle_now_s,
                snapshot_key=key,
                reason_code=None,
                values={"arm_tts_s": arm_tts},
                priority=3,
                synthetic=True,
            )
        elif decision == "skipped" and not attempt.late_skip_recorded:
            attempt.late_skip_recorded = True
            key = tick.active_key
            segment = None if key is None else attempt.track_segments.get(key)
            self._append_stage_locked(
                attempt,
                attempt_id=attempt_id,
                track_segment_id=segment,
                stage="LATE_SKIP",
                monotonic_s=tick.lifecycle_now_s,
                snapshot_key=key,
                reason_code="LATE_SKIP",
                values={},
                priority=3,
                synthetic=True,
            )

    def record_event(self, draft: EventDraft) -> None:
        if draft.kind not in (
            "attempt_transition",
            "planner_submit",
            "planner_trace",
            "lifecycle_tick",
        ):
            return
        with self._lock:
            if draft.kind == "lifecycle_tick":
                attempt_id = self._active_attempt_id
                if attempt_id is None:
                    attempt_id = draft.attempt_id
            else:
                attempt_id = draft.attempt_id
            if attempt_id is None:
                return
            attempt_id = int(attempt_id)
            attempt = self._attempt_locked(attempt_id)
            attempt.last_update_s = time.monotonic()
            if attempt.event_count >= self._event_capacity:
                attempt.recording_complete = False
                return
            attempt.event_count += 1
            if draft.kind == "attempt_transition":
                self._record_transition_locked(
                    attempt_id,
                    attempt,
                    draft,
                )
            elif draft.kind == "planner_submit":
                key = self._key(draft.payload.get("snapshot_key"))
                if key is None:
                    attempt.recording_complete = False
                else:
                    submitted_s = float(draft.monotonic_s)
                    attempt.submissions[key] = submitted_s
                    anchor = attempt.submission_phase_anchor_s
                    attempt.submission_phase_anchor_s = submitted_s if anchor is None else min(anchor, submitted_s)
            elif draft.kind == "planner_trace":
                self._record_planner_locked(
                    attempt_id,
                    attempt,
                    draft,
                )
            elif draft.kind == "lifecycle_tick":
                self._record_lifecycle_locked(
                    attempt_id,
                    attempt,
                    draft,
                )

    def ready_to_finalize(
        self,
        attempt_id: int,
        *,
        quiet_period_s: float = 0.005,
    ) -> bool:
        """Report whether async planner traces are complete and quiescent."""
        with self._lock:
            attempt = self._attempts.get(int(attempt_id))
            if attempt is None or not attempt.recording_complete:
                return True
            dropped = set(attempt.pending_replaced.values())
            unresolved = set(attempt.submissions) - set(attempt.results) - dropped
            if unresolved:
                return False
            return time.monotonic() - attempt.last_update_s >= float(quiet_period_s)

    @staticmethod
    def _planner_calls(
        attempt: _ReplayAttemptCapture,
    ) -> Tuple[PlannerCallRecord, ...]:
        calls = []
        for key, result in attempt.results.items():
            submitted = attempt.submissions.get(key)
            started = attempt.starts.get(key)
            if submitted is None or started is None:
                attempt.recording_complete = False
                continue
            completed = float(result.completed_monotonic_s)
            duration = completed - float(started)
            if not math.isfinite(duration) or duration < 0.0:
                attempt.recording_complete = False
                continue
            calls.append(
                PlannerCallRecord(
                    snapshot_key=key,
                    submitted_monotonic_s=float(submitted),
                    started_monotonic_s=float(started),
                    completed_monotonic_s=completed,
                    duration_s=duration,
                    pending_replaced_key=attempt.pending_replaced.get(key),
                    latest_replaced_key=attempt.latest_replaced.get(key),
                    result=result,
                )
            )
        calls.sort(
            key=lambda call: (
                call.completed_monotonic_s,
                call.snapshot_key,
            )
        )
        return tuple(calls)

    def finalize_attempt(
        self,
        *,
        attempt_id: int,
        terminal_code: str,
        recording_complete: bool,
        planner_config: Mapping[str, Any],
        runtime_settings: Optional[HitterRuntimeSettings],
        forced_strike_type: Optional[str],
    ) -> ReplayInputBundle:
        """Detach a bounded attempt snapshot for supervisor-side persistence."""
        with self._lock:
            attempt = self._attempts.pop(int(attempt_id), None)
            if attempt is None:
                attempt = _ReplayAttemptCapture(recording_complete=False)
            dropped = set(attempt.pending_replaced.values())
            unresolved = set(attempt.submissions) - set(attempt.results) - dropped
            if unresolved:
                attempt.recording_complete = False
            planner_calls = self._planner_calls(attempt)
            complete = bool(
                recording_complete and attempt.recording_complete and attempt_id not in self._evicted_attempts
            )
            deduplicated: Dict[Tuple[Any, ...], tuple] = {}
            passthrough = []
            for record in attempt.stage_records:
                transition = record[-1]
                identity = _canonical_stage_identity(transition)
                if identity is None:
                    passthrough.append(record)
                    continue
                previous = deduplicated.get(identity)
                if previous is None or record[-2] < previous[-2]:
                    deduplicated[identity] = record
            stage_records = passthrough + list(deduplicated.values())
            stages = tuple(
                value[-1]
                for value in sorted(
                    stage_records,
                    key=lambda item: item[:3],
                )
            )
            terminal = str(terminal_code) if complete else "RECORDING_INCOMPLETE"
            online = OnlineAttemptTrace(
                attempt_id=int(attempt_id),
                stages=stages,
                planner_calls=planner_calls,
                policy_ticks=tuple(attempt.policy_ticks),
                terminal_code=terminal,
                recovery_duration_s=attempt.recovery_duration_s,
                submission_phase_anchor_s=(attempt.submission_phase_anchor_s),
                recording_complete=complete,
            )
            return ReplayInputBundle(
                attempt_id=int(attempt_id),
                inputs=tuple(attempt.inputs),
                online_baseline=online,
                recording_complete=complete,
                planner_config=planner_config,
                runtime_settings=runtime_settings,
                forced_strike_type=forced_strike_type,
                capture_metadata=self.capacity_metadata,
            )


class ReplayRawCaptureTee:
    def __init__(self, delegate: object, capture: ReplayCaptureStore) -> None:
        self._delegate = delegate
        self._capture = capture

    def submit(
        self,
        sample: NormalizedMocapSample,
        *,
        attempt_id: Optional[int] = None,
        track_segment_id: Optional[int] = None,
    ) -> Any:
        self._capture.record_raw(
            sample,
            attempt_id=attempt_id,
            track_segment_id=track_segment_id,
        )
        submit = getattr(self._delegate, "submit", None)
        try:
            if callable(submit):
                accepted = submit(
                    sample,
                    attempt_id=attempt_id,
                    track_segment_id=track_segment_id,
                )
            elif callable(self._delegate):
                try:
                    accepted = self._delegate(
                        sample,
                        attempt_id=attempt_id,
                        track_segment_id=track_segment_id,
                    )
                except TypeError:
                    accepted = self._delegate(sample)
            else:
                raise TypeError("raw capture delegate is not callable")
        except Exception:
            self._capture.mark_incomplete(attempt_id)
            raise
        accepted_flag = getattr(accepted, "accepted", accepted)
        if accepted_flag is False:
            self._capture.mark_incomplete(attempt_id)
        return accepted


class ReplayEventCaptureTee:
    def __init__(self, delegate: object, capture: ReplayCaptureStore) -> None:
        self._delegate = delegate
        self._capture = capture

    def offer(self, draft: EventDraft) -> Any:
        self._capture.record_event(draft)
        offer = getattr(self._delegate, "offer", None)
        try:
            accepted = offer(draft) if callable(offer) else self._delegate(draft)
        except Exception:
            self._capture.mark_incomplete(draft.attempt_id)
            raise
        if accepted is False:
            self._capture.mark_incomplete(draft.attempt_id)
        return accepted

    def publish(self, draft: EventDraft) -> Any:
        return self.offer(draft)

    def drain(self, *args: Any, **kwargs: Any) -> Any:
        return self._delegate.drain(*args, **kwargs)

    def close(self, *args: Any, **kwargs: Any) -> Any:
        return self._delegate.close(*args, **kwargs)

    def __getattr__(self, name: str) -> Any:
        return getattr(self._delegate, name)


class PlannerProtocol(Protocol):
    def plan_command(
        self,
        ball_position: np.ndarray,
        ball_velocity: np.ndarray,
        *,
        current_base_xy_w: np.ndarray,
        base_forward_xy_w: np.ndarray,
        strike_type: Optional[str],
    ) -> object: ...


class PlannerDurationProvider(Protocol):
    def duration_s(
        self,
        *,
        variant: ReplayVariant,
        snapshot_key: SnapshotKey,
        measured_offline_duration_s: Optional[float],
    ) -> float: ...


@dataclass(frozen=True)
class ReplayTrace:
    variant: ReplayVariant
    stages: Tuple[AttemptTransition, ...]
    planner_calls: Tuple[PlannerCallRecord, ...]
    policy_ticks: Tuple[PolicyTickRecord, ...]
    outcome: ReplayOutcome

    def __post_init__(self) -> None:
        object.__setattr__(self, "stages", tuple(self.stages))
        object.__setattr__(self, "planner_calls", tuple(self.planner_calls))
        object.__setattr__(self, "policy_ticks", tuple(self.policy_ticks))


@dataclass(frozen=True)
class ReplayParity:
    matches: bool
    mismatches: Tuple[str, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "mismatches", tuple(self.mismatches))


@dataclass(frozen=True)
class ReplayComparison:
    baseline: ReplayOutcome
    one_frame: Optional[ReplayOutcome]
    parity: ReplayParity
    summary_label: str
    deltas: Mapping[str, JsonValue]

    def __post_init__(self) -> None:
        if self.summary_label not in _SUMMARY_LABELS:
            raise ValueError("invalid summary_label")
        object.__setattr__(self, "deltas", _frozen_mapping(self.deltas))

    def to_json_dict(self) -> Dict[str, Any]:
        return to_builtin_json(
            {
                "baseline": {
                    "variant": self.baseline.variant,
                    "terminal_code": self.baseline.terminal_code,
                    "task_obs_pass": self.baseline.task_obs_pass,
                    "boundary_sensitive": self.baseline.boundary_sensitive,
                    "recording_complete": self.baseline.recording_complete,
                    "warnings": self.baseline.warnings,
                    "summary_label": self.baseline.summary_label,
                    "metrics": self.baseline.metrics,
                },
                "one_frame": (
                    None
                    if self.one_frame is None
                    else {
                        "variant": self.one_frame.variant,
                        "terminal_code": self.one_frame.terminal_code,
                        "task_obs_pass": self.one_frame.task_obs_pass,
                        "boundary_sensitive": (self.one_frame.boundary_sensitive),
                        "recording_complete": (self.one_frame.recording_complete),
                        "warnings": self.one_frame.warnings,
                        "summary_label": self.one_frame.summary_label,
                        "metrics": self.one_frame.metrics,
                    }
                ),
                "parity": {
                    "matches": self.parity.matches,
                    "mismatches": self.parity.mismatches,
                },
                "summary_label": self.summary_label,
                "deltas": self.deltas,
            }
        )


@dataclass(frozen=True)
class ScheduledEvent:
    at_s: float
    event_type: str
    global_order_seq: int
    payload: Any


class DeterministicEventScheduler:
    """Heap scheduler ordered only by time, fixed priority and push order."""

    def __init__(self) -> None:
        self._heap = []
        self._next_global_order_seq = 0

    def push(self, at_s: float, event_type: str, payload: Any) -> ScheduledEvent:
        when = float(at_s)
        if not math.isfinite(when):
            raise ValueError("event time must be finite")
        if event_type not in _EVENT_PRIORITIES:
            raise ValueError("unknown replay event {!r}".format(event_type))
        sequence = self._next_global_order_seq
        self._next_global_order_seq += 1
        event = ScheduledEvent(when, event_type, sequence, payload)
        heapq.heappush(
            self._heap,
            (
                when,
                _EVENT_PRIORITIES[event_type],
                sequence,
                event,
            ),
        )
        return event

    def pop(self) -> ScheduledEvent:
        if not self._heap:
            raise IndexError("pop from empty replay scheduler")
        return heapq.heappop(self._heap)[3]

    def __bool__(self) -> bool:
        return bool(self._heap)

    def __len__(self) -> int:
        return len(self._heap)


class RecordedPlannerDurationProvider:
    """Reuse online durations and use measured time only for new calls."""

    def __init__(
        self,
        online_calls: Sequence[PlannerCallRecord],
        *,
        minimum_duration_s: float = 0.0,
    ) -> None:
        self._online = {call.snapshot_key: float(call.duration_s) for call in online_calls}
        self._minimum = float(minimum_duration_s)
        if not math.isfinite(self._minimum) or self._minimum < 0.0:
            raise ValueError("minimum_duration_s must be finite and nonnegative")

    def duration_s(
        self,
        *,
        variant: ReplayVariant,
        snapshot_key: SnapshotKey,
        measured_offline_duration_s: Optional[float],
    ) -> float:
        del variant
        if measured_offline_duration_s is not None:
            return max(
                float(measured_offline_duration_s),
                self._minimum,
            )
        recorded = self._online.get(snapshot_key)
        if recorded is not None:
            return recorded
        raise KeyError("no recorded duration for snapshot {}".format(snapshot_key))


@dataclass
class _PendingPlan:
    snapshot: BallEstimateSnapshot
    submitted_s: float
    pending_replaced_key: Optional[SnapshotKey]
    started_s: Optional[float] = None
    duration_s: Optional[float] = None
    command: Optional[object] = None
    command_fields: Optional[Mapping[str, JsonValue]] = None
    error_type: Optional[str] = None
    error_text: Optional[str] = None


class _ReplayWorkerFacade:
    def __init__(self, submit_callback: Callable[[BallEstimateSnapshot], None]):
        self._submit_callback = submit_callback
        self.bundle = (None, None)

    def submit(self, snapshot: BallEstimateSnapshot) -> None:
        self._submit_callback(snapshot)

    def latest_result_bundle(self):
        return self.bundle

    def close(self, timeout_s=None) -> bool:
        del timeout_s
        return True


class _ReplayEventSink:
    def __init__(self) -> None:
        self.transitions = []

    @staticmethod
    def _key(value: Any) -> Optional[SnapshotKey]:
        if value is None:
            return None
        return SnapshotKey(
            int(value["track_id"]),
            int(value["generation"]),
        )

    def offer(self, draft: EventDraft) -> bool:
        if draft.kind != "attempt_transition":
            return True
        payload = draft.payload
        self.transitions.append(
            AttemptTransition(
                attempt_id=int(draft.attempt_id),
                track_segment_id=(None if payload["track_segment_id"] is None else int(payload["track_segment_id"])),
                stage=str(payload["stage"]),
                monotonic_s=float(draft.monotonic_s),
                snapshot_key=self._key(payload["snapshot_key"]),
                reason_code=(None if payload["reason_code"] is None else str(payload["reason_code"])),
                values=payload["values"],
            )
        )
        return True


class _NullRawSink:
    def submit(self, sample, *, attempt_id=None, track_segment_id=None):
        del sample, attempt_id, track_segment_id
        return True


def _command_json_value(value: Any) -> Any:
    if is_dataclass(value):
        return {field.name: _command_json_value(getattr(value, field.name)) for field in fields(value)}
    if isinstance(value, np.ndarray):
        return tuple(_command_json_value(item) for item in value.tolist())
    if isinstance(value, np.generic):
        return _command_json_value(value.item())
    if isinstance(value, Mapping):
        return {str(key): _command_json_value(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return tuple(_command_json_value(item) for item in value)
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    return repr(value)[:2048]


def _freeze_command_fields(command: object) -> Mapping[str, JsonValue]:
    converted = _command_json_value(command)
    if not isinstance(converted, Mapping):
        converted = {"value": converted}
    return _frozen_mapping(converted)


def _snapshot_key(snapshot: BallEstimateSnapshot) -> SnapshotKey:
    return SnapshotKey(int(snapshot.track_id), int(snapshot.generation))


def _result_key(result: PlannerResultSnapshot) -> SnapshotKey:
    return SnapshotKey(
        int(result.track_id),
        int(result.source_generation),
    )


def _adapter_output(
    adapter: MocapFrameAdapter,
    sample: NormalizedMocapSample,
) -> AdapterOutput:
    message = SimpleNamespace(
        name=sample.subject,
        vicon_time_s=0.0 if sample.source_time_s is None else sample.source_time_s,
        publish_time_us=(0 if sample.publish_time_us is None else sample.publish_time_us),
        pos_vicon=sample.position_w,
        quat_vicon=sample.quaternion_xyzw,
        valid=int(sample.valid),
        occluded=int(sample.occluded),
        vicon_frame_number=sample.source_frame,
    )
    output = adapter.ingest_decoded(
        channel=sample.channel,
        message=message,
        payload_size=sample.payload_size,
        received_monotonic_s=sample.received_monotonic_s,
        wall_time_us=sample.wall_time_us,
    )
    return replace(output, sample=sample)


def _inferred_swing_total_s(online: OnlineAttemptTrace) -> float:
    recovery = online.recovery_duration_s
    if recovery is None:
        return 1.85
    call_by_key = {call.snapshot_key: call for call in online.planner_calls}
    for tick in online.policy_ticks:
        if tick.phase != "armed" and tick.lifecycle_decision != "armed":
            continue
        if tick.active_key is None:
            continue
        call = call_by_key.get(tick.active_key)
        if call is None:
            continue
        arm_tts = call.result.strike_deadline_monotonic_s - tick.lifecycle_now_s
        if math.isfinite(arm_tts):
            return max(float(recovery) + max(arm_tts, 0.0), 0.0)
    return max(float(recovery), 0.0) + 0.8


def _initial_online_snapshot_key(
    online: OnlineAttemptTrace,
) -> Optional[SnapshotKey]:
    for transition in online.stages:
        if transition.stage in ("DETECTED", "POST_DEADLINE_TAIL") and transition.snapshot_key is not None:
            return transition.snapshot_key
    for transition in online.stages:
        if transition.snapshot_key is not None:
            return transition.snapshot_key
    if online.planner_calls:
        return min(
            online.planner_calls,
            key=lambda call: call.submitted_monotonic_s,
        ).snapshot_key
    return None


def _runtime_settings(
    variant: ReplayVariant,
    *,
    swing_total_s: float,
    recorded: Optional[HitterRuntimeSettings] = None,
) -> HitterRuntimeSettings:
    if recorded is not None:
        return replace(
            recorded,
            planner_update_rate_hz=variant.planner_rate_hz,
            planner_update_interval_s=1.0 / variant.planner_rate_hz,
            incoming_confirmation_snapshots=(variant.incoming_confirmations),
            swing_duration_range_s=(swing_total_s, swing_total_s),
        )
    return HitterRuntimeSettings(
        estimator_sample_rate_hz=300.0,
        planner_update_rate_hz=variant.planner_rate_hz,
        planner_update_interval_s=1.0 / variant.planner_rate_hz,
        minimum_incoming_speed_x_mps=0.20,
        incoming_confirmation_snapshots=variant.incoming_confirmations,
        waiting_tts_s=0.92,
        arm_tts_s=0.92,
        minimum_arm_tts_s=0.60,
        maximum_policy_tts_s=0.92,
        swing_duration_range_s=(swing_total_s, swing_total_s),
        hitter_seed=0,
        control_tick_s=0.02,
        obs_clip_value=100.0,
    )


def _transition_with_bundle_identity(
    transition: AttemptTransition,
    *,
    bundle: ReplayInputBundle,
) -> AttemptTransition:
    segment_id = transition.track_segment_id
    if transition.snapshot_key is not None:
        for online_stage in bundle.online_baseline.stages:
            if online_stage.snapshot_key == transition.snapshot_key and online_stage.track_segment_id is not None:
                segment_id = online_stage.track_segment_id
                break
    return AttemptTransition(
        attempt_id=bundle.attempt_id,
        track_segment_id=segment_id,
        stage=transition.stage,
        monotonic_s=transition.monotonic_s,
        snapshot_key=transition.snapshot_key,
        reason_code=transition.reason_code,
        values=transition.values,
    )


def _extract_target_metrics(
    metrics: Dict[str, Any],
    fields_value: Optional[Mapping[str, JsonValue]],
) -> None:
    if not isinstance(fields_value, Mapping):
        return
    base = fields_value.get("p_base_target_xy")
    strike_plan = fields_value.get("strike_plan")
    racket = strike_plan.get("p_racket_target") if isinstance(strike_plan, Mapping) else None
    try:
        base_array = np.asarray(base, dtype=np.float64).reshape(-1)
        if base_array.shape == (2,) and np.isfinite(base_array).all():
            metrics["base_target_x"] = float(base_array[0])
            metrics["base_target_y"] = float(base_array[1])
    except (TypeError, ValueError):
        pass
    try:
        racket_array = np.asarray(racket, dtype=np.float64).reshape(-1)
        if racket_array.shape == (3,) and np.isfinite(racket_array).all():
            metrics["racket_target_x"] = float(racket_array[0])
            metrics["racket_target_y"] = float(racket_array[1])
            metrics["racket_target_z"] = float(racket_array[2])
    except (TypeError, ValueError):
        pass
    try:
        metrics["command_fields_json"] = json.dumps(
            to_builtin_json(fields_value),
            ensure_ascii=False,
            allow_nan=False,
            separators=(",", ":"),
            sort_keys=True,
        )
    except (TypeError, ValueError):
        pass


def _array_tuple(value: Any) -> Optional[Tuple[float, ...]]:
    if value is None:
        return None
    return tuple(float(item) for item in np.asarray(value).reshape(-1))


def _replay_attempt(
    bundle: ReplayInputBundle,
    *,
    variant: ReplayVariant,
    duration_provider: PlannerDurationProvider,
    planner_factory: Callable[[], PlannerProtocol],
    checkpoint: Optional[Callable[[], None]] = None,
) -> ReplayTrace:
    planner = planner_factory()
    scheduler = DeterministicEventScheduler()
    for sample in bundle.inputs:
        scheduler.push(sample.received_monotonic_s, "INPUT", sample)
    for tick in bundle.online_baseline.policy_ticks:
        scheduler.push(
            tick.lifecycle_now_s,
            "LIFECYCLE_TICK",
            tick,
        )
        scheduler.push(tick.obs_now_s, "OBS_TICK", tick.tick_index)

    settings = _runtime_settings(
        variant,
        swing_total_s=_inferred_swing_total_s(bundle.online_baseline),
        recorded=bundle.runtime_settings,
    )
    adapter_config = to_builtin_json(bundle.planner_config)
    if not isinstance(adapter_config, Mapping):
        raise TypeError("planner_config must be a mapping")
    adapter = MocapFrameAdapter(
        adapter_config,
        estimator_sample_rate_hz=settings.estimator_sample_rate_hz,
    )
    initial_key = _initial_online_snapshot_key(bundle.online_baseline)
    if initial_key is not None:
        adapter.track_id = int(initial_key.track_id)
        adapter.ball_snapshot_generation = max(
            0,
            int(initial_key.generation) - 1,
        )
    event_sink = _ReplayEventSink()
    pending: Optional[_PendingPlan] = None
    in_flight: Optional[_PendingPlan] = None
    start_scheduled = False
    current_event_s = 0.0
    planner_calls = []
    policy_ticks = []
    deferred_policy_ticks: Dict[int, PolicyTickRecord] = {}
    submitted = 0
    failed = 0
    dropped_pending = 0
    overwritten_latest = 0
    first_ready_time_s = None
    confirm_time_s = None
    plan_success_time_s = None
    arm_time_s = None
    arm_tts_s = None
    task_pass_time_s = None
    saw_late_skip = False
    saw_pelvis_unavailable = False
    saw_malformed = False
    boundary_sensitive = False
    submission_phase_anchor_s = None
    plan_candidates = []
    incoming_confirming_seen = set()
    task_failure_reasons = set()
    online_call_by_key = {call.snapshot_key: call for call in bundle.online_baseline.planner_calls}
    recorded_start_calls: Deque[PlannerCallRecord] = deque(
        sorted(
            bundle.online_baseline.planner_calls,
            key=lambda call: (
                call.started_monotonic_s,
                call.snapshot_key,
            ),
        )
    )
    use_recorded_starts = bool(variant == BASELINE_VARIANT and recorded_start_calls)
    recorded_attempt_close_s = next(
        (
            float(stage.monotonic_s)
            for stage in reversed(bundle.online_baseline.stages)
            if stage.stage == "ATTEMPT_CLOSED"
        ),
        None,
    )

    def check_pause() -> None:
        if checkpoint is not None:
            checkpoint()

    def schedule_submit(snapshot: BallEstimateSnapshot) -> None:
        nonlocal pending
        nonlocal start_scheduled
        nonlocal submitted
        nonlocal dropped_pending
        nonlocal submission_phase_anchor_s
        submitted += 1
        if submission_phase_anchor_s is None:
            submission_phase_anchor_s = float(snapshot.received_monotonic_s)
        replaced_key = None
        if pending is not None:
            replaced_key = _snapshot_key(pending.snapshot)
            dropped_pending += 1
        pending = _PendingPlan(
            snapshot=snapshot,
            submitted_s=float(snapshot.received_monotonic_s),
            pending_replaced_key=replaced_key,
        )
        if in_flight is None and not start_scheduled:
            start_scheduled = True
            if use_recorded_starts:
                if not recorded_start_calls:
                    start_scheduled = False
                    return
                expected = recorded_start_calls[0]
                scheduler.push(
                    max(
                        current_event_s,
                        expected.started_monotonic_s,
                    ),
                    "PLAN_START",
                    expected.snapshot_key,
                )
            else:
                scheduler.push(current_event_s, "PLAN_START", None)

    worker = _ReplayWorkerFacade(schedule_submit)
    pipeline = ShadowTaskPipeline(
        adapter=adapter,
        settings=settings,
        event_sink=event_sink,
        raw_sink=_NullRawSink(),
        planner=planner,
        worker=worker,
        forced_strike_type=bundle.forced_strike_type,
    )

    def append_stage(
        *,
        snapshot: BallEstimateSnapshot,
        stage: str,
        at_s: float,
        reason_code: Optional[str] = None,
        values: Optional[Mapping[str, Any]] = None,
    ) -> None:
        binding = pipeline.attempt_tracker.binding_for_result(_snapshot_key(snapshot))
        if binding is None:
            return
        transition = pipeline.attempt_tracker.record_stage(
            binding=binding,
            stage=stage,
            monotonic_s=at_s,
            reason_code=reason_code,
            values={} if values is None else values,
        )
        event_sink.transitions.append(transition)

    while scheduler:
        check_pause()
        event = scheduler.pop()
        current_event_s = event.at_s
        if event.event_type == "INPUT":
            output = _adapter_output(adapter, event.payload)
            pipeline.ingest_adapter_output(output)
            if output.reset_transition is not None or (
                output.sample.subject == "ball" and (not output.sample.valid or output.sample.occluded)
            ):
                scheduler.push(
                    (
                        recorded_attempt_close_s
                        if (recorded_attempt_close_s is not None and recorded_attempt_close_s > event.at_s)
                        else event.at_s + 0.20 + _TIME_EPSILON_S
                    ),
                    "GRACE_EXPIRE",
                    None,
                )

        elif event.event_type == "PLAN_START":
            start_scheduled = False
            if in_flight is None and pending is not None:
                job = pending
                if use_recorded_starts:
                    if not recorded_start_calls:
                        raise RuntimeError("recorded planner start schedule exhausted")
                    expected = recorded_start_calls.popleft()
                    if _snapshot_key(job.snapshot) != expected.snapshot_key:
                        raise RuntimeError("recorded planner start snapshot mismatch")
                pending = None
                in_flight = job
                job.started_s = event.at_s
                snapshot = job.snapshot
                key = _snapshot_key(snapshot)
                if snapshot.ready and first_ready_time_s is None:
                    first_ready_time_s = event.at_s
                    append_stage(
                        snapshot=snapshot,
                        stage="ESTIMATOR_READY",
                        at_s=event.at_s,
                    )
                incoming_before = pipeline.incoming.snapshot()
                measured_started = time.perf_counter()
                try:
                    job.command = pipeline.plan_snapshot(snapshot)
                    job.command_fields = _freeze_command_fields(job.command)
                    time_to_strike = float(getattr(job.command, "time_to_strike"))
                    if not math.isfinite(time_to_strike):
                        raise ValueError("planner command time_to_strike is non-finite")
                except Exception as exc:
                    job.error_type = type(exc).__name__
                    job.error_text = str(exc)
                measured = max(
                    time.perf_counter() - measured_started,
                    0.0,
                )
                incoming_after = pipeline.incoming.snapshot()
                incoming_required = int(pipeline.incoming.required_consecutive_snapshots)
                incoming_count = int(incoming_after.consecutive_count)
                confirming_identity = (
                    incoming_after.track_id,
                    incoming_count,
                )
                if (
                    incoming_count != int(incoming_before.consecutive_count)
                    and 0 < incoming_count < incoming_required
                    and confirming_identity not in incoming_confirming_seen
                ):
                    incoming_confirming_seen.add(confirming_identity)
                    append_stage(
                        snapshot=snapshot,
                        stage="INCOMING_CONFIRMING",
                        at_s=event.at_s,
                        values={
                            "count": incoming_count,
                            "required": incoming_required,
                        },
                    )
                if incoming_after.confirmed and not incoming_before.confirmed:
                    confirm_time_s = event.at_s
                    append_stage(
                        snapshot=snapshot,
                        stage="INCOMING_CONFIRMED",
                        at_s=event.at_s,
                        values={
                            "count": incoming_count,
                            "required": incoming_required,
                        },
                    )
                online_call = online_call_by_key.get(key)
                equivalent_online_call = False
                if online_call is not None:
                    if variant == BASELINE_VARIANT:
                        equivalent_online_call = True
                    elif (
                        online_call.result.command_fields is not None
                        and job.command_fields is not None
                        and online_call.result.error_type is None
                        and job.error_type is None
                    ):
                        equivalent_online_call = True
                    elif (
                        online_call.result.command_fields is None
                        and job.command_fields is None
                        and online_call.result.error_type == job.error_type
                        and online_call.result.error_text == job.error_text
                    ):
                        equivalent_online_call = True
                measured_for_provider = None if equivalent_online_call else measured
                duration = float(
                    duration_provider.duration_s(
                        variant=variant,
                        snapshot_key=key,
                        measured_offline_duration_s=(measured_for_provider),
                    )
                )
                if not math.isfinite(duration) or duration < 0.0:
                    raise ValueError("planner duration must be finite and nonnegative")
                job.duration_s = duration
                plan_candidates.append(
                    (
                        float(snapshot.velocity_w[0]),
                        bool(job.command is not None and job.error_type is None),
                    )
                )
                scheduler.push(
                    event.at_s + duration,
                    "PLAN_COMPLETE",
                    job,
                )

        elif event.event_type == "PLAN_COMPLETE":
            job = event.payload
            if in_flight is not job:
                raise RuntimeError("planner completion identity mismatch")
            snapshot = job.snapshot
            key = _snapshot_key(snapshot)
            command = job.command
            strike_deadline_s = float("nan")
            if command is not None and job.error_type is None:
                strike_deadline_s = float(snapshot.received_monotonic_s) + float(getattr(command, "time_to_strike"))
            failure_reason = None if job.error_type is None else PlannerFailureReason.INTERNAL_ERROR
            planner_error_text = None if job.error_type is None else "{}: {}".format(job.error_type, job.error_text)
            frozen = FrozenPlannerResult(
                snapshot_key=key,
                source_frame=int(snapshot.source_frame),
                strike_deadline_monotonic_s=strike_deadline_s,
                completed_monotonic_s=event.at_s,
                command_fields=job.command_fields,
                error_type=job.error_type,
                error_text=job.error_text,
                failure_reason=failure_reason,
            )
            previous_result = worker.bundle[1]
            latest_replaced_key = None if previous_result is None else previous_result.snapshot_key
            if previous_result is not None:
                overwritten_latest += 1
            result = PlannerResultSnapshot(
                track_id=int(snapshot.track_id),
                source_generation=int(snapshot.generation),
                source_frame=int(snapshot.source_frame),
                strike_deadline_monotonic_s=strike_deadline_s,
                completed_monotonic_s=event.at_s,
                command=command,
                failure_reason=failure_reason,
                error_text=planner_error_text,
            )
            worker.bundle = (result, frozen)
            planner_calls.append(
                PlannerCallRecord(
                    snapshot_key=key,
                    submitted_monotonic_s=job.submitted_s,
                    started_monotonic_s=float(job.started_s),
                    completed_monotonic_s=event.at_s,
                    duration_s=float(job.duration_s),
                    pending_replaced_key=job.pending_replaced_key,
                    latest_replaced_key=latest_replaced_key,
                    result=frozen,
                )
            )
            if job.error_type is None and _has_required_command_fields(frozen.command_fields):
                if plan_success_time_s is None:
                    plan_success_time_s = event.at_s
                append_stage(
                    snapshot=snapshot,
                    stage="PLANNER_SUCCEEDED",
                    at_s=event.at_s,
                )
            else:
                if job.error_type is not None:
                    failed += 1
                    reason = planner_reason_code(frozen)
                else:
                    reason = "MALFORMED_COMMAND"
                    saw_malformed = True
                if reason == "PELVIS_UNAVAILABLE":
                    saw_pelvis_unavailable = True
                append_stage(
                    snapshot=snapshot,
                    stage="PLANNER_REJECTED",
                    at_s=event.at_s,
                    reason_code=reason,
                    values={
                        "error_type": job.error_type,
                        "error_text": job.error_text,
                    },
                )
            in_flight = None
            if pending is not None and not start_scheduled:
                start_scheduled = True
                if use_recorded_starts:
                    if recorded_start_calls:
                        expected = recorded_start_calls[0]
                        scheduler.push(
                            max(
                                event.at_s,
                                expected.started_monotonic_s,
                            ),
                            "PLAN_START",
                            expected.snapshot_key,
                        )
                    else:
                        start_scheduled = False
                else:
                    scheduler.push(event.at_s, "PLAN_START", None)

        elif event.event_type == "LIFECYCLE_TICK":
            recorded = event.payload
            deferred_policy_ticks[recorded.tick_index] = recorded

        elif event.event_type == "OBS_TICK":
            try:
                recorded = deferred_policy_ticks.pop(int(event.payload))
            except KeyError:
                raise RuntimeError("observation tick has no saved lifecycle clock")
            tick = pipeline.tick(
                lifecycle_now_s=recorded.lifecycle_now_s,
                obs_now_s=recorded.obs_now_s,
                wall_time_us=0,
            )
            active = pipeline.lifecycle.active_result
            cached = pipeline.lifecycle.cached_result
            active_key = None if active is None else _result_key(active)
            cached_key = None if cached is None else _result_key(cached)
            task = tick.task_observation
            policy_record = PolicyTickRecord(
                tick_index=recorded.tick_index,
                lifecycle_now_s=tick.lifecycle_now_s,
                obs_now_s=tick.obs_now_s,
                lifecycle_decision=tick.lifecycle_decision,
                phase=tick.phase,
                active_key=active_key,
                cached_key=cached_key,
                command_fields=tick.command_fields_used,
                task_pre_clip=(None if task is None else _array_tuple(task.pre_clip)),
                task_post_clip=(None if task is None else _array_tuple(task.post_clip)),
                task_clip_count=(None if task is None else int(task.clip_count)),
            )
            policy_ticks.append(policy_record)
            if tick.lifecycle_decision in ("armed", "skipped"):
                relevant = active if active is not None else worker.bundle[0]
                if relevant is not None:
                    remaining = relevant.strike_deadline_monotonic_s - tick.lifecycle_now_s
                    distance = min(
                        abs(remaining - settings.minimum_arm_tts_s),
                        abs(remaining - settings.arm_tts_s),
                    )
                    if distance < _BOUNDARY_WARNING_S:
                        boundary_sensitive = True
            if tick.lifecycle_decision == "armed":
                saw_late_skip = False
                if arm_time_s is None:
                    arm_time_s = tick.lifecycle_now_s
                    if active is not None:
                        arm_tts_s = active.strike_deadline_monotonic_s - tick.lifecycle_now_s
                    binding = tick.active_binding
                    if binding is not None:
                        transition = pipeline.attempt_tracker.record_stage(
                            binding=binding,
                            stage="ARMED",
                            monotonic_s=tick.lifecycle_now_s,
                            reason_code=None,
                            values={"arm_tts_s": arm_tts_s},
                        )
                        event_sink.transitions.append(transition)
            elif tick.lifecycle_decision == "skipped":
                saw_late_skip = True
                latest_snapshot = None if in_flight is None else in_flight.snapshot
                if latest_snapshot is None and planner_calls:
                    latest_key = planner_calls[-1].snapshot_key
                    binding = pipeline.attempt_tracker.binding_for_result(latest_key)
                else:
                    binding = (
                        None
                        if latest_snapshot is None
                        else pipeline.attempt_tracker.binding_for_result(_snapshot_key(latest_snapshot))
                    )
                if binding is not None:
                    transition = pipeline.attempt_tracker.record_stage(
                        binding=binding,
                        stage="LATE_SKIP",
                        monotonic_s=tick.lifecycle_now_s,
                        reason_code="LATE_SKIP",
                        values={},
                    )
                    event_sink.transitions.append(transition)
            if "OBS_COMMAND_MISMATCH" in tick.errors:
                saw_malformed = True
            if tick.errors and tick.active_binding is not None:
                reason = str(tick.errors[0])
                if reason not in task_failure_reasons:
                    task_failure_reasons.add(reason)
                    transition = pipeline.attempt_tracker.record_stage(
                        binding=tick.active_binding,
                        stage="TASK_OBS_FAILED",
                        monotonic_s=tick.obs_now_s,
                        reason_code=reason,
                        values={"errors": tuple(tick.errors)},
                    )
                    event_sink.transitions.append(transition)
            if tick.task_pass and task_pass_time_s is None:
                task_pass_time_s = tick.obs_now_s

        elif event.event_type == "GRACE_EXPIRE":
            for transition in pipeline.attempt_tracker.advance(now_monotonic_s=event.at_s):
                event_sink.transitions.append(transition)
        check_pause()

    metrics: Dict[str, Any] = {
        "attempt_id": bundle.attempt_id,
        "first_ready_time_s": first_ready_time_s,
        "confirm_time_s": confirm_time_s,
        "plan_success_time_s": plan_success_time_s,
        "arm_time_s": arm_time_s,
        "arm_tts_s": arm_tts_s,
        "task_pass_time_s": task_pass_time_s,
        "planner_submitted": submitted,
        "planner_completed": len(planner_calls),
        "planner_failed": failed,
        "planner_dropped_pending": dropped_pending,
        "planner_results_overwritten_before_consume": overwritten_latest,
        "recovery_duration_s": (float(pipeline.lifecycle.recovery_duration_s) if arm_time_s is not None else None),
        "submission_phase_anchor_s": submission_phase_anchor_s,
    }
    active_frozen = worker.bundle[1]
    if pipeline.lifecycle.active_result is not None:
        active_key = _result_key(pipeline.lifecycle.active_result)
        for call in reversed(planner_calls):
            if call.snapshot_key == active_key:
                active_frozen = call.result
                break
    _extract_target_metrics(
        metrics,
        None if active_frozen is None else active_frozen.command_fields,
    )

    warnings = []
    if boundary_sensitive:
        warnings.append("BOUNDARY_SENSITIVE")
    if variant.incoming_confirmations == 1 and plan_candidates:
        first_qualified = None
        for index, (velocity_x, succeeded) in enumerate(plan_candidates):
            if math.isfinite(velocity_x) and velocity_x <= -settings.minimum_incoming_speed_x_mps and succeeded:
                first_qualified = index
                break
        if first_qualified is not None:
            following = plan_candidates[first_qualified + 1 : first_qualified + 3]
            if any(
                (not math.isfinite(velocity_x) or velocity_x > -settings.minimum_incoming_speed_x_mps or not succeeded)
                for velocity_x, succeeded in following
            ):
                warnings.append("ONE_FRAME_UNSTABLE")

    if not bundle.recording_complete:
        terminal_code = "RECORDING_INCOMPLETE"
    elif task_pass_time_s is not None:
        terminal_code = "TASK_OBS_PASS"
    elif saw_late_skip:
        terminal_code = "LATE_SKIP"
    elif first_ready_time_s is None:
        terminal_code = "TRACK_ENDED_BEFORE_READY"
    elif confirm_time_s is None:
        terminal_code = "TRACK_ENDED_BEFORE_CONFIRMATION"
    elif saw_pelvis_unavailable:
        terminal_code = "PELVIS_UNAVAILABLE"
    elif saw_malformed:
        terminal_code = "MALFORMED_COMMAND"
    else:
        terminal_code = "NO_VALID_PLAN"

    stages = _deduplicate_canonical_stages(
        tuple(
            _transition_with_bundle_identity(
                transition,
                bundle=bundle,
            )
            for transition in event_sink.transitions
        )
    )
    return ReplayTrace(
        variant=variant,
        stages=stages,
        planner_calls=tuple(planner_calls),
        policy_ticks=tuple(policy_ticks),
        outcome=ReplayOutcome(
            variant=variant.name,
            terminal_code=terminal_code,
            task_obs_pass=terminal_code == "TASK_OBS_PASS",
            boundary_sensitive=boundary_sensitive,
            recording_complete=bundle.recording_complete,
            warnings=tuple(warnings),
            summary_label=None,
            metrics=metrics,
        ),
    )


def replay_attempt(
    bundle: ReplayInputBundle,
    *,
    variant: ReplayVariant,
    duration_provider: PlannerDurationProvider,
    planner_factory: Callable[[], PlannerProtocol],
    checkpoint: Optional[Callable[[], None]] = None,
) -> ReplayTrace:
    """Run one deterministic discrete-event replay without real sleep."""
    return _replay_attempt(
        bundle,
        variant=variant,
        duration_provider=duration_provider,
        planner_factory=planner_factory,
        checkpoint=checkpoint,
    )


def _compare_value(
    path: str,
    expected: Any,
    actual: Any,
    *,
    atol: float,
    mismatches: list,
) -> None:
    if is_dataclass(expected) and is_dataclass(actual):
        if type(expected) is not type(actual):
            mismatches.append(path)
            return
        for field in fields(expected):
            _compare_value(
                "{}.{}".format(path, field.name),
                getattr(expected, field.name),
                getattr(actual, field.name),
                atol=atol,
                mismatches=mismatches,
            )
        return
    if isinstance(expected, Mapping) and isinstance(actual, Mapping):
        expected_keys = tuple(sorted(expected))
        actual_keys = tuple(sorted(actual))
        if expected_keys != actual_keys:
            mismatches.append("{}.keys".format(path))
            return
        for key in expected_keys:
            _compare_value(
                "{}.{}".format(path, key),
                expected[key],
                actual[key],
                atol=atol,
                mismatches=mismatches,
            )
        return
    if isinstance(expected, (tuple, list)) and isinstance(actual, (tuple, list)):
        if len(expected) != len(actual):
            mismatches.append("{}.length".format(path))
            return
        for index, (expected_item, actual_item) in enumerate(zip(expected, actual)):
            _compare_value(
                "{}[{}]".format(path, index),
                expected_item,
                actual_item,
                atol=atol,
                mismatches=mismatches,
            )
        return
    if isinstance(expected, (float, np.floating)) or isinstance(actual, (float, np.floating)):
        try:
            expected_float = float(expected)
            actual_float = float(actual)
        except (TypeError, ValueError):
            mismatches.append(path)
            return
        if math.isnan(expected_float) and math.isnan(actual_float):
            return
        if (
            not math.isfinite(expected_float)
            or not math.isfinite(actual_float)
            or abs(expected_float - actual_float) > atol
        ):
            mismatches.append(path)
        return
    if expected != actual:
        mismatches.append(path)


def compare_baseline_to_online(
    *,
    online: OnlineAttemptTrace,
    replay: ReplayTrace,
    atol: float = _PARITY_ATOL,
) -> ReplayParity:
    """Compare all canonical online facts before counterfactual execution."""
    tolerance = float(atol)
    if not math.isfinite(tolerance) or tolerance < 0.0:
        raise ValueError("atol must be finite and nonnegative")
    mismatches = []
    values = [
        ("stages", online.stages, replay.stages),
        ("planner_calls", online.planner_calls, replay.planner_calls),
        ("policy_ticks", online.policy_ticks, replay.policy_ticks),
        ("terminal_code", online.terminal_code, replay.outcome.terminal_code),
        (
            "recording_complete",
            online.recording_complete,
            replay.outcome.recording_complete,
        ),
    ]
    if "attempt_id" not in replay.outcome.metrics:
        mismatches.append("attempt_id")
    else:
        values.insert(
            0,
            (
                "attempt_id",
                online.attempt_id,
                replay.outcome.metrics["attempt_id"],
            ),
        )
    for metric_name, online_value in (
        ("recovery_duration_s", online.recovery_duration_s),
        (
            "submission_phase_anchor_s",
            online.submission_phase_anchor_s,
        ),
    ):
        if metric_name not in replay.outcome.metrics:
            mismatches.append(metric_name)
            continue
        values.append(
            (
                metric_name,
                online_value,
                replay.outcome.metrics[metric_name],
            )
        )
    for path, expected, actual in values:
        _compare_value(
            path,
            expected,
            actual,
            atol=tolerance,
            mismatches=mismatches,
        )
    unique = tuple(dict.fromkeys(mismatches))
    return ReplayParity(not unique, unique)


def _metric_delta(
    baseline: ReplayOutcome,
    one_frame: ReplayOutcome,
    key: str,
    *,
    scale: float = 1.0,
) -> Optional[float]:
    baseline_value = _optional_float(baseline.metrics.get(key))
    one_value = _optional_float(one_frame.metrics.get(key))
    if baseline_value is None or one_value is None:
        return None
    return (one_value - baseline_value) * scale


def _target_delta(
    baseline: ReplayOutcome,
    one_frame: ReplayOutcome,
    keys: Tuple[str, ...],
) -> Optional[float]:
    try:
        baseline_values = np.asarray(
            [baseline.metrics[key] for key in keys],
            dtype=np.float64,
        )
        one_values = np.asarray(
            [one_frame.metrics[key] for key in keys],
            dtype=np.float64,
        )
    except (KeyError, TypeError, ValueError):
        return None
    if not np.isfinite(baseline_values).all() or not np.isfinite(one_values).all():
        return None
    return float(np.linalg.norm(one_values - baseline_values))


def _comparison_deltas(
    baseline: ReplayOutcome,
    one_frame: ReplayOutcome,
) -> Mapping[str, JsonValue]:
    return _frozen_mapping(
        {
            "delta_confirm_ms": _metric_delta(
                baseline,
                one_frame,
                "confirm_time_s",
                scale=1000.0,
            ),
            "delta_arm_ms": _metric_delta(
                baseline,
                one_frame,
                "arm_time_s",
                scale=1000.0,
            ),
            "delta_arm_tts": _metric_delta(
                baseline,
                one_frame,
                "arm_tts_s",
            ),
            "base_target_delta_m": _target_delta(
                baseline,
                one_frame,
                ("base_target_x", "base_target_y"),
            ),
            "racket_target_delta_m": _target_delta(
                baseline,
                one_frame,
                (
                    "racket_target_x",
                    "racket_target_y",
                    "racket_target_z",
                ),
            ),
        }
    )


def compare_replay_outcomes(
    baseline: ReplayOutcome,
    one_frame: ReplayOutcome,
    *,
    parity: Optional[ReplayParity] = None,
) -> ReplayComparison:
    parity_value = ReplayParity(True, ()) if parity is None else parity
    deltas = _comparison_deltas(baseline, one_frame)
    if (
        not parity_value.matches
        or not baseline.recording_complete
        or not one_frame.recording_complete
        or baseline.boundary_sensitive
        or one_frame.boundary_sensitive
    ):
        label = "INCONCLUSIVE"
    elif baseline.task_obs_pass and one_frame.task_obs_pass:
        command_same = baseline.metrics.get("command_fields_json") == one_frame.metrics.get("command_fields_json")
        base_delta = deltas.get("base_target_delta_m")
        racket_delta = deltas.get("racket_target_delta_m")
        target_same = all(value is None or abs(float(value)) <= _PARITY_ATOL for value in (base_delta, racket_delta))
        label = "SAME_PASS" if command_same and target_same else "BOTH_PASS_DIFFERENT_COMMAND"
        if (
            "command_fields_json" not in baseline.metrics
            and "command_fields_json" not in one_frame.metrics
            and target_same
        ):
            label = "SAME_PASS"
    elif (
        not baseline.task_obs_pass
        and one_frame.task_obs_pass
        and baseline.terminal_code
        in (
            "LATE_SKIP",
            "TRACK_ENDED_BEFORE_READY",
            "TRACK_ENDED_BEFORE_CONFIRMATION",
        )
    ):
        label = "SAVED_BY_ONE_FRAME"
    elif baseline.task_obs_pass and not one_frame.task_obs_pass:
        label = "BASELINE_ONLY_PASS"
    elif not baseline.task_obs_pass and not one_frame.task_obs_pass:
        label = "BOTH_FAIL_SAME" if baseline.terminal_code == one_frame.terminal_code else "BOTH_FAIL_DIFFERENT"
    else:
        label = "INCONCLUSIVE"
    return ReplayComparison(
        baseline=replace(baseline, summary_label=label),
        one_frame=replace(one_frame, summary_label=label),
        parity=parity_value,
        summary_label=label,
        deltas=deltas,
    )


def _divergence_comparison(
    baseline: ReplayOutcome,
    parity: ReplayParity,
) -> ReplayComparison:
    divergent = ReplayOutcome(
        variant=baseline.variant,
        terminal_code="REPLAY_DIVERGENCE",
        task_obs_pass=False,
        boundary_sensitive=baseline.boundary_sensitive,
        recording_complete=baseline.recording_complete,
        warnings=tuple(baseline.warnings) + ("REPLAY_DIVERGENCE",),
        summary_label="INCONCLUSIVE",
        metrics={
            **to_builtin_json(baseline.metrics),
            "parity_mismatches": ",".join(parity.mismatches),
        },
    )
    return ReplayComparison(
        baseline=divergent,
        one_frame=None,
        parity=parity,
        summary_label="INCONCLUSIVE",
        deltas={},
    )


def analyze_attempt_ab(
    bundle: ReplayInputBundle,
    *,
    planner_factory: Callable[[], PlannerProtocol],
    duration_provider: PlannerDurationProvider,
    checkpoint: Optional[Callable[[], None]] = None,
) -> ReplayComparison:
    """Run 3/100, gate on parity, then and only then run fresh 1/100."""
    baseline = replay_attempt(
        bundle,
        variant=BASELINE_VARIANT,
        duration_provider=duration_provider,
        planner_factory=planner_factory,
        checkpoint=checkpoint,
    )
    if not bundle.recording_complete:
        return ReplayComparison(
            baseline=replace(
                baseline.outcome,
                summary_label="INCONCLUSIVE",
            ),
            one_frame=None,
            parity=ReplayParity(
                False,
                ("recording_complete",),
            ),
            summary_label="INCONCLUSIVE",
            deltas={},
        )
    parity = compare_baseline_to_online(
        online=bundle.online_baseline,
        replay=baseline,
    )
    if not parity.matches:
        return _divergence_comparison(baseline.outcome, parity)
    one_frame = replay_attempt(
        bundle,
        variant=ONE_FRAME_VARIANT,
        duration_provider=duration_provider,
        planner_factory=planner_factory,
        checkpoint=checkpoint,
    )
    return compare_replay_outcomes(
        baseline.outcome,
        one_frame.outcome,
        parity=parity,
    )


@dataclass(frozen=True)
class ReplayJobRef:
    session_basename: str
    attempt_id: int
    input_path: Path

    def __post_init__(self) -> None:
        if not self.session_basename:
            raise ValueError("session_basename must be non-empty")
        if self.attempt_id <= 0:
            raise ValueError("attempt_id must be positive")
        object.__setattr__(self, "input_path", Path(self.input_path))


def run_replay_disk_job(
    job: ReplayJobRef,
    checkpoint: Callable[[], None],
) -> Mapping[str, Any]:
    """Spawn-picklable disk worker using a fresh planner per replay variant."""
    checkpoint()
    bundle = load_replay_input_bundle(job.input_path)
    if bundle.attempt_id != job.attempt_id:
        raise ValueError("replay job attempt_id does not match disk bundle")
    checkpoint()
    duration_provider = RecordedPlannerDurationProvider(bundle.online_baseline.planner_calls)

    def planner_factory() -> PlannerProtocol:
        return build_hitter_system_planner(bundle.planner_config)

    comparison = analyze_attempt_ab(
        bundle,
        planner_factory=planner_factory,
        duration_provider=duration_provider,
        checkpoint=checkpoint,
    )
    checkpoint()
    return comparison.to_json_dict()


@dataclass(frozen=True)
class ReplayWorkerReply:
    attempt_id: int
    status: str
    result: Optional[Mapping[str, JsonValue]]
    error: Optional[str]

    def __post_init__(self) -> None:
        if self.attempt_id <= 0:
            raise ValueError("attempt_id must be positive")
        if self.status not in (
            "RUNNING",
            "PAUSED",
            "COMPLETED",
            "FAILED",
        ):
            raise ValueError("invalid replay worker status")
        if self.result is not None:
            object.__setattr__(
                self,
                "result",
                _frozen_mapping(self.result),
            )
        if self.error is not None:
            object.__setattr__(self, "error", str(self.error)[:2048])


class _PauseRequested(Exception):
    pass


class _StopRequested(Exception):
    pass


def _child_reply(reply_queue, reply_event, value: Mapping[str, Any]) -> None:
    reply_queue.put(dict(value))
    reply_event.set()


def _replay_child_main(
    request_queue,
    reply_queue,
    reply_event,
    pause_event,
    paused_ack_event,
    stop_event,
    worker_fn,
) -> None:
    def checkpoint() -> None:
        if stop_event.is_set():
            raise _StopRequested()
        if pause_event.is_set():
            raise _PauseRequested()

    while not stop_event.is_set():
        try:
            command, job = request_queue.get(timeout=0.1)
        except queue.Empty:
            continue
        if command == "STOP":
            return
        if command != "RUN":
            continue
        _child_reply(
            reply_queue,
            reply_event,
            {
                "attempt_id": job.attempt_id,
                "status": "RUNNING",
                "result": None,
                "error": None,
            },
        )
        try:
            checkpoint()
            result = worker_fn(job, checkpoint)
            checkpoint()
            frozen = freeze_json_value(result)
            if not isinstance(frozen, Mapping):
                raise TypeError("replay worker result must be a mapping")
            _child_reply(
                reply_queue,
                reply_event,
                {
                    "attempt_id": job.attempt_id,
                    "status": "COMPLETED",
                    "result": to_builtin_json(frozen),
                    "error": None,
                },
            )
        except _PauseRequested:
            _child_reply(
                reply_queue,
                reply_event,
                {
                    "attempt_id": job.attempt_id,
                    "status": "PAUSED",
                    "result": None,
                    "error": None,
                },
            )
            paused_ack_event.set()
        except _StopRequested:
            return
        except BaseException as exc:
            _child_reply(
                reply_queue,
                reply_event,
                {
                    "attempt_id": job.attempt_id,
                    "status": "FAILED",
                    "result": None,
                    "error": "{}: {}".format(
                        type(exc).__name__,
                        str(exc),
                    )[:2048],
                },
            )


class ReplayProcessController:
    """Spawn replay work while keeping all persistence in the parent.

    ``set_realtime_busy`` is a supervisor operation because it waits for a
    bounded PAUSED acknowledgement.  It must not be called from the 50 Hz
    lifecycle/policy thread.
    """

    def __init__(
        self,
        *,
        worker_fn: Callable[[ReplayJobRef, Callable[[], None]], Mapping[str, Any]],
        recorder,
        event_sink=None,
        mp_context=None,
        pause_timeout_s: float = 0.25,
        persistence_timeout_s: float = 1.0,
        pending_capacity: int = 1024,
    ) -> None:
        self._context = multiprocessing.get_context("spawn") if mp_context is None else mp_context
        self._recorder = recorder
        self._event_sink = event_sink
        self._pause_timeout_s = float(pause_timeout_s)
        if not math.isfinite(self._pause_timeout_s) or self._pause_timeout_s < 0.0:
            raise ValueError("pause_timeout_s must be finite and nonnegative")
        self._persistence_timeout_s = float(persistence_timeout_s)
        if not math.isfinite(self._persistence_timeout_s) or self._persistence_timeout_s < 0.0:
            raise ValueError("persistence_timeout_s must be finite and nonnegative")
        if isinstance(pending_capacity, bool) or not isinstance(pending_capacity, int) or pending_capacity <= 0:
            raise ValueError("pending_capacity must be a positive integer")
        self._pending_capacity = int(pending_capacity)
        self._worker_fn = worker_fn
        self._reply_event = self._context.Event()
        self._pause_event = self._context.Event()
        self._paused_ack_event = self._context.Event()
        self._stop_event = self._context.Event()
        self._request_queue = None
        self._reply_queue = None
        self._process = None
        self._pending: Deque[ReplayJobRef] = deque()
        self._current_job: Optional[ReplayJobRef] = None
        self._busy = False
        self._closed = False
        self._close_result = True
        self._process_guard = threading.RLock()
        self._reaper_thread: Optional[threading.Thread] = None
        self._start_child()

    def _start_child(self) -> None:
        self._request_queue = self._context.Queue()
        self._reply_queue = self._context.Queue()
        self._reply_event.clear()
        self._paused_ack_event.clear()
        process = self._context.Process(
            target=_replay_child_main,
            args=(
                self._request_queue,
                self._reply_queue,
                self._reply_event,
                self._pause_event,
                self._paused_ack_event,
                self._stop_event,
                self._worker_fn,
            ),
            name="HitterReplayProcess",
        )
        process.start()
        self._process = process

    @staticmethod
    def _close_queue(queue_value, *, cancel_feeder: bool) -> None:
        if queue_value is None:
            return
        if cancel_feeder:
            try:
                queue_value.cancel_join_thread()
            except (AttributeError, OSError, ValueError):
                pass
        try:
            queue_value.close()
        except (OSError, ValueError):
            return
        if not cancel_feeder:
            try:
                queue_value.join_thread()
            except (AssertionError, OSError, ValueError):
                pass

    @staticmethod
    def _close_process_handle(process) -> None:
        if process is None:
            return
        try:
            process.close()
        except (AttributeError, OSError, ValueError):
            pass

    def _reap_process(
        self,
        *,
        timeout_s: float,
        terminate: bool,
    ) -> bool:
        with self._process_guard:
            process = self._process
            reaper = self._reaper_thread
        if process is None:
            return True
        deadline = time.monotonic() + timeout_s
        if reaper is not None and reaper.is_alive() and reaper is not threading.current_thread():
            reaper.join(max(0.0, deadline - time.monotonic()))
            with self._process_guard:
                return self._process is None

        process.join(max(0.0, deadline - time.monotonic()))
        if process.is_alive() and terminate:
            try:
                process.terminate()
            except (OSError, ValueError):
                pass
            process.join(max(0.0, deadline - time.monotonic()))
        if process.is_alive() and terminate:
            kill = getattr(process, "kill", None)
            if callable(kill):
                try:
                    kill()
                except (OSError, ValueError):
                    pass
            process.join(max(0.0, deadline - time.monotonic()))
        if process.is_alive():
            if terminate:
                self._start_background_reaper(process)
            return False
        self._release_process_handle(process)
        return True

    def _release_process_handle(self, process) -> None:
        with self._process_guard:
            if self._process is not process:
                return
            self._close_process_handle(process)
            self._process = None

    def _start_background_reaper(self, process) -> None:
        with self._process_guard:
            existing = self._reaper_thread
            if existing is not None and existing.is_alive():
                return
            reaper = threading.Thread(
                target=self._background_reap,
                args=(process,),
                name="HitterReplayProcessReaper",
                daemon=True,
            )
            self._reaper_thread = reaper
            reaper.start()

    def _background_reap(self, process) -> None:
        try:
            if process.is_alive():
                kill = getattr(process, "kill", None)
                if callable(kill):
                    try:
                        kill()
                    except (OSError, ValueError):
                        pass
            process.join()
        except (AssertionError, OSError, ValueError):
            pass
        finally:
            self._release_process_handle(process)
            with self._process_guard:
                if self._reaper_thread is threading.current_thread():
                    self._reaper_thread = None

    def _offer_job_status(
        self,
        job: ReplayJobRef,
        status: str,
        *,
        error: Optional[str] = None,
    ) -> None:
        value = {
            "session_basename": job.session_basename,
            "attempt_id": job.attempt_id,
            "input_path": str(job.input_path),
            "status": status,
            "error": error,
        }
        offer = getattr(self._recorder, "offer_replay_job_status", None)
        if not callable(offer) or not bool(offer(value)):
            raise RuntimeError("recorder rejected replay job status")

    def _offer_analysis(
        self,
        job: ReplayJobRef,
        result: Mapping[str, JsonValue],
    ) -> None:
        value = {
            "session_basename": job.session_basename,
            "attempt_id": job.attempt_id,
            "result": result,
        }
        offer = getattr(self._recorder, "offer_replay_analysis", None)
        if not callable(offer) or not bool(offer(value)):
            raise RuntimeError("recorder rejected replay analysis")

    def _confirm_persistence(
        self,
        *,
        drain_event_sink: bool,
    ) -> None:
        deadline = time.monotonic() + self._persistence_timeout_s
        if drain_event_sink and self._event_sink is not None:
            drain_events = getattr(self._event_sink, "drain", None)
            if callable(drain_events):
                remaining = max(0.0, deadline - time.monotonic())
                if not bool(drain_events(timeout_s=remaining)):
                    raise RuntimeError("REPLAY_PERSISTENCE_ERROR:" "event publisher did not drain")
        confirm = getattr(
            self._recorder,
            "confirm_replay_persistence",
            None,
        )
        if not callable(confirm):
            raise RuntimeError("REPLAY_PERSISTENCE_ERROR:" "recorder does not support durable confirmation")
        remaining = max(0.0, deadline - time.monotonic())
        status = confirm(timeout_s=remaining)
        healthy = bool(getattr(status, "healthy", False))
        complete = bool(getattr(status, "recording_complete", False))
        if healthy and complete:
            return
        last_error = getattr(status, "last_error", None)
        raise RuntimeError(
            "REPLAY_PERSISTENCE_ERROR:{}".format(
                "recorder reported incomplete persistence" if last_error is None else str(last_error)
            )
        )

    def _mark_persistence_incomplete(self, job: ReplayJobRef) -> None:
        marker = getattr(self._recorder, "mark_incomplete", None)
        if not callable(marker):
            return
        try:
            marker(job.attempt_id, "REPLAY_PERSISTENCE_ERROR")
        except BaseException:
            pass

    def _publish_reply(
        self,
        job: ReplayJobRef,
        reply: ReplayWorkerReply,
    ) -> None:
        if self._event_sink is None:
            return
        draft = EventDraft(
            kind="replay_worker_status",
            monotonic_s=time.monotonic(),
            wall_time_us=int(time.time() * 1.0e6),
            scope="attempt",
            attempt_id=job.attempt_id,
            payload={
                "status": reply.status,
                "error": reply.error,
                "result": reply.result,
            },
        )
        publish = getattr(self._event_sink, "publish", None)
        if callable(publish):
            if publish(draft) is False:
                raise RuntimeError("event sink rejected replay reply")
            return
        offer = getattr(self._event_sink, "offer", None)
        if callable(offer):
            if offer(draft) is False:
                raise RuntimeError("event sink rejected replay reply")
            return
        if callable(self._event_sink):
            if self._event_sink(draft) is False:
                raise RuntimeError("event sink rejected replay reply")

    @staticmethod
    def _persistence_failure_reply(
        job: ReplayJobRef,
        error: BaseException,
    ) -> ReplayWorkerReply:
        return ReplayWorkerReply(
            attempt_id=job.attempt_id,
            status="FAILED",
            result=None,
            error="REPLAY_PERSISTENCE_ERROR:{}: {}".format(
                type(error).__name__,
                str(error),
            )[:2048],
        )

    def _persist_reply(
        self,
        job: ReplayJobRef,
        reply: ReplayWorkerReply,
    ) -> ReplayWorkerReply:
        try:
            self._offer_job_status(
                job,
                reply.status,
                error=reply.error,
            )
            if reply.status == "COMPLETED" and reply.result is not None:
                self._offer_analysis(job, reply.result)
            self._publish_reply(job, reply)
            self._confirm_persistence(drain_event_sink=True)
            return reply
        except BaseException as exc:
            self._mark_persistence_incomplete(job)
            failed = self._persistence_failure_reply(job, exc)
            try:
                self._offer_job_status(
                    job,
                    failed.status,
                    error=failed.error,
                )
            except BaseException:
                pass
            try:
                self._publish_reply(job, failed)
            except BaseException:
                pass
            return failed

    def _dispatch_if_idle(self) -> None:
        if self._closed or self._busy or self._current_job is not None or not self._pending:
            return
        self._paused_ack_event.clear()
        self._pause_event.clear()
        job = self._pending.popleft()
        self._current_job = job
        self._request_queue.put(("RUN", job))

    def set_realtime_busy(
        self,
        *,
        attempt_active: bool,
        reacquire_grace_active: bool,
    ) -> None:
        """Supervisor-only pause; never call this from the 50 Hz tick."""
        if self._closed:
            return
        busy = bool(attempt_active or reacquire_grace_active)
        self._busy = busy
        if busy:
            self._pause_event.set()
            if self._current_job is not None:
                deadline = time.monotonic() + self._pause_timeout_s
                while self._current_job is not None:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        break
                    try:
                        raw = self._reply_queue.get(timeout=remaining)
                    except queue.Empty:
                        break
                    reply = self._consume_raw_reply(raw)
                    if reply is not None and reply.status == "PAUSED":
                        break
            return
        self.poll()
        if self._current_job is not None and self._pause_event.is_set():
            return
        self._pause_event.clear()
        self._dispatch_if_idle()

    def submit_disk_job(self, job: ReplayJobRef) -> None:
        """Record QUEUED before the job can cross the IPC boundary."""
        if self._closed:
            raise RuntimeError("replay controller is closed")
        if not job.input_path.is_file():
            raise FileNotFoundError(str(job.input_path))
        outstanding = len(self._pending) + int(self._current_job is not None)
        if outstanding >= self._pending_capacity:
            raise OverflowError("replay pending window is full")
        try:
            self._offer_job_status(job, "QUEUED")
            self._confirm_persistence(drain_event_sink=False)
        except BaseException:
            self._mark_persistence_incomplete(job)
            raise
        self._pending.append(job)
        self._dispatch_if_idle()

    def wait_for_reply(self, *, timeout_s: float) -> bool:
        if timeout_s < 0.0:
            raise ValueError("timeout_s must be nonnegative")
        return bool(self._reply_event.wait(timeout_s))

    def _consume_raw_reply(
        self,
        raw: Mapping[str, Any],
    ) -> Optional[ReplayWorkerReply]:
        reply = ReplayWorkerReply(
            attempt_id=int(raw["attempt_id"]),
            status=str(raw["status"]),
            result=raw["result"],
            error=raw["error"],
        )
        job = self._current_job
        if job is None or job.attempt_id != reply.attempt_id:
            return None
        if self._pause_event.is_set() and reply.status in (
            "COMPLETED",
            "FAILED",
        ):
            reply = ReplayWorkerReply(
                attempt_id=reply.attempt_id,
                status="PAUSED",
                result=None,
                error=None,
            )
        reply = self._persist_reply(job, reply)
        if reply.status == "PAUSED":
            self._pending.appendleft(job)
            self._current_job = None
        elif reply.status in ("COMPLETED", "FAILED"):
            self._current_job = None
        return reply

    def _recover_unexpected_child_exit(
        self,
    ) -> Tuple[ReplayWorkerReply, ...]:
        process = self._process
        if self._closed or process is None or process.is_alive():
            return ()
        exitcode = process.exitcode
        job = self._current_job
        self._current_job = None
        self._close_queue(self._request_queue, cancel_feeder=True)
        self._close_queue(self._reply_queue, cancel_feeder=True)
        self._close_process_handle(process)
        self._request_queue = None
        self._reply_queue = None
        self._process = None
        replies = []
        if job is not None:
            failure = ReplayWorkerReply(
                attempt_id=job.attempt_id,
                status="FAILED",
                result=None,
                error="REPLAY_CHILD_EXIT: exitcode {}".format(exitcode),
            )
            replies.append(self._persist_reply(job, failure))
        if not self._stop_event.is_set():
            self._start_child()
        return tuple(replies)

    def poll(self) -> Tuple[ReplayWorkerReply, ...]:
        """Receive child replies; persist and publish only in this parent."""
        replies = []
        self._reply_event.clear()
        while True:
            try:
                raw = self._reply_queue.get_nowait()
            except queue.Empty:
                break
            reply = self._consume_raw_reply(raw)
            if reply is not None:
                replies.append(reply)
        replies.extend(self._recover_unexpected_child_exit())
        self._dispatch_if_idle()
        return tuple(replies)

    def close(self, *, timeout_s: float = 5.0) -> bool:
        """Stop the spawned child with a bounded join."""
        if timeout_s < 0.0:
            raise ValueError("timeout_s must be nonnegative")
        if not self._closed:
            self._closed = True
            self._stop_event.set()
            self._pause_event.clear()
            request_queue = self._request_queue
            if request_queue is not None:
                try:
                    request_queue.put(("STOP", None))
                except (OSError, ValueError):
                    pass
            reaped = self._reap_process(
                timeout_s=timeout_s,
                terminate=True,
            )
            request_queue = self._request_queue
            reply_queue = self._reply_queue
            self._request_queue = None
            self._reply_queue = None
            self._close_queue(
                request_queue,
                cancel_feeder=not reaped,
            )
            self._close_queue(
                reply_queue,
                cancel_feeder=not reaped,
            )
        else:
            reaped = self._reap_process(
                timeout_s=timeout_s,
                terminate=True,
            )
        self._close_result = bool(reaped)
        return self._close_result


__all__ = [
    "BASELINE_VARIANT",
    "DeterministicEventScheduler",
    "ONE_FRAME_VARIANT",
    "OnlineAttemptTrace",
    "PlannerCallRecord",
    "PlannerDurationProvider",
    "PlannerProtocol",
    "PolicyTickRecord",
    "RecordedPlannerDurationProvider",
    "ReplayCaptureStore",
    "ReplayComparison",
    "ReplayEventCaptureTee",
    "ReplayInputBundle",
    "ReplayJobRef",
    "ReplayOutcome",
    "ReplayParity",
    "ReplayProcessController",
    "ReplayRawCaptureTee",
    "ReplayTrace",
    "ReplayVariant",
    "ReplayWorkerReply",
    "ScheduledEvent",
    "analyze_attempt_ab",
    "compare_baseline_to_online",
    "compare_replay_outcomes",
    "load_replay_input_bundle",
    "replay_attempt",
    "replay_input_bundle_from_json_dict",
    "replay_input_bundle_to_json_dict",
    "run_replay_disk_job",
    "write_replay_input_bundle",
]
