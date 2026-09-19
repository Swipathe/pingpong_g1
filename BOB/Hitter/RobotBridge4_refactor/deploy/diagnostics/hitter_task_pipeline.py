from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass, replace
import math
import threading
from typing import Any, Callable, Mapping, Optional, Tuple

import numpy as np

from diagnostics.hitter_task_attempts import AttemptTracker
from diagnostics.hitter_task_models import (
    AttemptBinding,
    BallDiagnosticState,
    EventDraft,
    LifecycleSnapshot,
    NormalizedMocapSample,
    SnapshotKey,
)
from utils.hitter_serialization import JsonValue
from utils.hitter_realtime import (
    BallEstimateSnapshot,
    CommandPhase,
    CompletedResultBatch,
    FrozenPlannerResult,
    IncomingTrackConfirmation,
    LatestOnlyPlannerWorker,
    LifecycleCancelReason,
    LifecycleDecision,
    PlannerResultSnapshot,
    PlannerWorkerTrace,
    freeze_planner_command_fields,
)
from utils.hitter_runtime_factory import (
    HitterRuntimeSettings,
    build_ball_state_estimator,
    build_hitter_command_lifecycle,
    build_hitter_system_planner,
)
from utils.hitter_task_observation import (
    TaskObservationAssemblyError,
    TaskObservationResult,
    assemble_active_hitter_task_observation,
)
from utils.transformation import matrix_from_quat


def _readonly_float32(value, *, shape: Tuple[int, ...], field: str) -> np.ndarray:
    array = np.array(value, dtype=np.float32, copy=True)
    if array.shape != shape:
        raise ValueError("{} must have shape {}, got {}".format(field, shape, array.shape))
    immutable = np.frombuffer(
        array.tobytes(order="C"),
        dtype=np.float32,
    ).reshape(shape)
    immutable.setflags(write=False)
    return immutable


@dataclass(frozen=True)
class ProductionReset:
    previous_track_id: int
    new_track_id: int
    reason: str


@dataclass(frozen=True)
class LatestPelvisPose:
    position_w: np.ndarray
    quaternion_xyzw: np.ndarray
    valid: bool
    occluded: bool
    source_frame: int
    source_time_s: Optional[float]
    publish_time_us: Optional[int]
    received_monotonic_s: Optional[float]
    wall_time_us: Optional[int]

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "position_w",
            _readonly_float32(
                self.position_w,
                shape=(3,),
                field="position_w",
            ),
        )
        object.__setattr__(
            self,
            "quaternion_xyzw",
            _readonly_float32(
                self.quaternion_xyzw,
                shape=(4,),
                field="quaternion_xyzw",
            ),
        )


@dataclass(frozen=True)
class AdapterOutput:
    sample: NormalizedMocapSample
    snapshot: Optional[BallEstimateSnapshot]
    warnings: Tuple[str, ...]
    estimator_sample_count: int
    bounce_detected: bool
    reset_transition: Optional[ProductionReset]


class MocapFrameAdapter:
    """Normalize decoded ChingMu frames while preserving RealWorld semantics."""

    _KNOWN_SUBJECTS = frozenset(("ball", "g2pelvis", "table"))

    def __init__(
        self,
        planner_config: Optional[Mapping[str, object]] = None,
        *,
        estimator_sample_rate_hz: Optional[float] = None,
        pelvis_stale_threshold_s: float = 0.05,
        maximum_source_frame_delta: int = 1,
        clock_offset_threshold_s: float = 0.05,
        clock_rate_tolerance_s: float = 0.005,
    ) -> None:
        config = {} if planner_config is None else planner_config
        configured_rate = config.get(
            "state_estimator_sample_rate_hz",
            300.0,
        )
        sample_rate = configured_rate if estimator_sample_rate_hz is None else estimator_sample_rate_hz
        self.estimator_sample_rate_hz = float(sample_rate)
        if not np.isfinite(self.estimator_sample_rate_hz) or self.estimator_sample_rate_hz <= 0.0:
            raise ValueError("estimator_sample_rate_hz must be finite and positive")
        self.pelvis_stale_threshold_s = float(pelvis_stale_threshold_s)
        if not np.isfinite(self.pelvis_stale_threshold_s) or self.pelvis_stale_threshold_s < 0.0:
            raise ValueError("pelvis_stale_threshold_s must be finite and nonnegative")
        self.maximum_source_frame_delta = int(maximum_source_frame_delta)
        if self.maximum_source_frame_delta < 0:
            raise ValueError("maximum_source_frame_delta must be nonnegative")
        self.clock_offset_threshold_s = float(clock_offset_threshold_s)
        self.clock_rate_tolerance_s = float(clock_rate_tolerance_s)
        if (
            not np.isfinite(self.clock_offset_threshold_s)
            or self.clock_offset_threshold_s < 0.0
            or not np.isfinite(self.clock_rate_tolerance_s)
            or self.clock_rate_tolerance_s < 0.0
        ):
            raise ValueError("clock warning thresholds must be nonnegative")

        self.lock = threading.RLock()
        self.ball_state_estimator = build_ball_state_estimator(config)
        self.ball_snapshot_generation = 0
        self.latest_ball_snapshot: Optional[BallEstimateSnapshot] = None
        self.fallback_estimator_time_s: Optional[float] = None
        self._next_input_seq = 0
        self._ball_position_w = np.zeros(3, dtype=np.float32)
        self._ball_velocity_w = np.zeros(3, dtype=np.float32)
        self._ball_visible = False
        self._ball_ready = False
        self._accepted_pelvis_position_w = np.zeros(
            3,
            dtype=np.float32,
        )
        self._accepted_pelvis_quaternion_xyzw = np.array(
            [0.0, 0.0, 0.0, 1.0],
            dtype=np.float32,
        )
        self._pelvis_valid = False
        self._pelvis_occluded = False
        self._pelvis_source_frame = 0
        self._pelvis_source_time_s: Optional[float] = None
        self._pelvis_publish_time_us: Optional[int] = None
        self._pelvis_received_monotonic_s: Optional[float] = None
        self._pelvis_wall_time_us: Optional[int] = None
        self._last_source_frame_by_subject = {}
        self._last_source_publish_pair_by_subject = {}

    @staticmethod
    def _append_warning(warnings, reason: str) -> None:
        if reason not in warnings:
            warnings.append(reason)

    @staticmethod
    def _positive_finite(value: float) -> bool:
        return bool(np.isfinite(value) and value > 0.0)

    def _next_fallback_timestamp_locked(self) -> float:
        if self.fallback_estimator_time_s is None:
            self.fallback_estimator_time_s = 0.0
        else:
            self.fallback_estimator_time_s += 1.0 / self.estimator_sample_rate_hz
        return float(self.fallback_estimator_time_s)

    def _estimator_timestamp_locked(
        self,
        *,
        source_time_s: float,
        publish_time_us: int,
    ) -> float:
        if self._positive_finite(source_time_s):
            return float(source_time_s)
        if publish_time_us > 0:
            publish_time_s = float(publish_time_us) * 1.0e-6
            if self._positive_finite(publish_time_s):
                return publish_time_s
        return self._next_fallback_timestamp_locked()

    def _clear_ball_tracking_locked(self) -> None:
        self.ball_state_estimator.reset()
        self.fallback_estimator_time_s = None
        self._ball_visible = False
        self._ball_ready = False

    def _make_snapshot_locked(
        self,
        sample: NormalizedMocapSample,
    ) -> BallEstimateSnapshot:
        self.ball_snapshot_generation += 1
        snapshot = BallEstimateSnapshot(
            track_id=int(sample.track_id),
            generation=int(self.ball_snapshot_generation),
            source_frame=int(sample.source_frame),
            source_time_s=float(sample.source_time_s or 0.0),
            received_monotonic_s=float(sample.received_monotonic_s),
            position_w=self._ball_position_w.astype(
                np.float32,
                copy=True,
            ),
            velocity_w=self._ball_velocity_w.astype(
                np.float32,
                copy=True,
            ),
            base_position_w=self._accepted_pelvis_position_w.astype(
                np.float32,
                copy=True,
            ),
            base_quaternion_xyzw=(
                self._accepted_pelvis_quaternion_xyzw.astype(
                    np.float32,
                    copy=True,
                )
            ),
            base_valid=bool(self._pelvis_valid),
            visible=bool(self._ball_visible),
            ready=bool(self._ball_ready),
        )
        self.latest_ball_snapshot = snapshot
        return snapshot

    def _source_warnings_locked(
        self,
        sample: NormalizedMocapSample,
        warnings,
    ) -> None:
        previous_frame = self._last_source_frame_by_subject.get(sample.subject)
        if (
            previous_frame is not None
            and abs(int(sample.source_frame) - int(previous_frame)) > self.maximum_source_frame_delta
        ):
            self._append_warning(warnings, "SOURCE_FRAME_GAP")
        self._last_source_frame_by_subject[sample.subject] = int(sample.source_frame)

        source_time_s = sample.source_time_s
        publish_time_us = sample.publish_time_us
        if (
            source_time_s is None
            or publish_time_us is None
            or not self._positive_finite(float(source_time_s))
            or publish_time_us <= 0
        ):
            return
        publish_time_s = float(publish_time_us) * 1.0e-6
        if not self._positive_finite(publish_time_s):
            return
        current = (float(source_time_s), publish_time_s)
        previous = self._last_source_publish_pair_by_subject.get(sample.subject)
        self._last_source_publish_pair_by_subject[sample.subject] = current
        if previous is None:
            return
        source_delta = current[0] - previous[0]
        publish_delta = current[1] - previous[1]
        if (
            source_delta > 0.0
            and publish_delta > 0.0
            and abs(source_delta - publish_delta) <= self.clock_rate_tolerance_s
            and abs(current[1] - current[0]) > self.clock_offset_threshold_s
        ):
            self._append_warning(warnings, "CLOCK_OFFSET_SUSPECTED")

    def _pelvis_warnings_for_ball_locked(
        self,
        sample: NormalizedMocapSample,
        warnings,
    ) -> None:
        if not self._pelvis_valid:
            self._append_warning(warnings, "PELVIS_INVALID")
            return
        mismatch = abs(int(sample.source_frame) - self._pelvis_source_frame) > self.maximum_source_frame_delta
        if (
            self._pelvis_received_monotonic_s is not None
            and float(sample.received_monotonic_s) - self._pelvis_received_monotonic_s > self.pelvis_stale_threshold_s
        ):
            mismatch = True
        if (
            sample.source_time_s is not None
            and self._pelvis_source_time_s is not None
            and np.isfinite(float(sample.source_time_s))
            and np.isfinite(float(self._pelvis_source_time_s))
            and abs(float(sample.source_time_s) - float(self._pelvis_source_time_s)) > self.pelvis_stale_threshold_s
        ):
            mismatch = True
        if mismatch:
            self._append_warning(warnings, "PELVIS_FRAME_MISMATCH")

    def _update_pelvis_locked(
        self,
        sample: NormalizedMocapSample,
        warnings,
    ) -> None:
        effective_valid = bool(sample.valid and not sample.occluded)
        finite_pose = bool(
            np.isfinite(sample.position_w).all()
            and np.isfinite(sample.quaternion_xyzw).all()
            and float(np.linalg.norm(sample.quaternion_xyzw)) >= 1.0e-6
        )
        self._pelvis_valid = bool(effective_valid and finite_pose)
        self._pelvis_occluded = bool(sample.occluded)
        self._pelvis_source_frame = int(sample.source_frame)
        self._pelvis_source_time_s = sample.source_time_s
        self._pelvis_publish_time_us = sample.publish_time_us
        self._pelvis_received_monotonic_s = float(sample.received_monotonic_s)
        self._pelvis_wall_time_us = int(sample.wall_time_us)
        if self._pelvis_valid:
            self._accepted_pelvis_position_w = np.asarray(
                sample.position_w,
                dtype=np.float32,
            ).copy()
            self._accepted_pelvis_quaternion_xyzw = np.asarray(
                sample.quaternion_xyzw,
                dtype=np.float32,
            ).copy()
        else:
            self._append_warning(warnings, "PELVIS_INVALID")

    def _update_ball_locked(
        self,
        sample: NormalizedMocapSample,
        warnings,
    ):
        self._pelvis_warnings_for_ball_locked(sample, warnings)
        effective_valid = bool(sample.valid and not sample.occluded)
        if not effective_valid:
            previous_track_id = int(sample.track_id)
            self._clear_ball_tracking_locked()
            self._append_warning(
                warnings,
                "BALL_INVALID_OR_OCCLUDED",
            )
            reset = ProductionReset(
                previous_track_id=previous_track_id,
                new_track_id=int(sample.track_id),
                reason="BALL_INVALID_OR_OCCLUDED",
            )
            return (
                self._make_snapshot_locked(sample),
                False,
                reset,
            )

        timestamp = self._estimator_timestamp_locked(
            source_time_s=float(sample.source_time_s or 0.0),
            publish_time_us=int(sample.publish_time_us or 0),
        )
        try:
            estimate = self.ball_state_estimator.add_sample(
                sample.position_w,
                timestamp=timestamp,
            )
        except ValueError:
            self._append_warning(warnings, "ESTIMATE_NONFINITE")
            return None, False, None

        self._ball_position_w = estimate.position.astype(
            np.float32,
            copy=True,
        )
        self._ball_velocity_w = estimate.velocity.astype(
            np.float32,
            copy=True,
        )
        self._ball_visible = True
        self._ball_ready = bool(estimate.valid)
        bounce_detected = bool(estimate.bounce_detected)
        if bounce_detected:
            self._append_warning(warnings, "BOUNCE_RESET")
        return (
            self._make_snapshot_locked(sample),
            bounce_detected,
            None,
        )

    def ingest_decoded(
        self,
        *,
        channel: str,
        message: object,
        payload_size: int,
        received_monotonic_s: float,
        wall_time_us: int,
    ) -> AdapterOutput:
        """Process one decoded transformation_t in exact arrival order."""
        with self.lock:
            raw_subject = str(getattr(message, "name", "") or "")
            subject = raw_subject.strip().lower() or "<unnamed>"
            track_id = getattr(message, "track_id", None)
            track_id_valid = type(track_id) is int
            track_id_value = int(track_id) if track_id_valid else 0
            source_time_s = float(getattr(message, "vicon_time_s", 0.0) or 0.0)
            publish_time_us = int(getattr(message, "publish_time_us", 0) or 0)
            sample = NormalizedMocapSample(
                input_seq=self._next_input_seq,
                channel=str(channel),
                subject=subject,
                track_id=track_id_value,
                identity_source="wire_v2",
                position_w=np.asarray(
                    getattr(message, "pos_vicon"),
                    dtype=np.float64,
                ).reshape(
                    -1
                )[:3],
                quaternion_xyzw=np.asarray(
                    getattr(message, "quat_vicon"),
                    dtype=np.float64,
                ).reshape(
                    -1
                )[:4],
                valid=bool(getattr(message, "valid", 1)),
                occluded=bool(getattr(message, "occluded", 0)),
                source_frame=int(getattr(message, "vicon_frame_number", 0) or 0),
                source_time_s=source_time_s,
                publish_time_us=publish_time_us,
                received_monotonic_s=float(received_monotonic_s),
                wall_time_us=int(wall_time_us),
                payload_size=int(payload_size),
            )
            self._next_input_seq += 1
            warnings = []
            if subject in self._KNOWN_SUBJECTS:
                self._source_warnings_locked(sample, warnings)
            contract_valid = True
            if str(channel) != "vicon_state_data_v2":
                contract_valid = False
                self._append_warning(warnings, "VICON_SCHEMA_ERROR")
            if not track_id_valid:
                contract_valid = False
                self._append_warning(warnings, "TRACK_ID_NOT_INTEGER")
            elif subject == "ball" and track_id_value <= 0:
                contract_valid = False
                self._append_warning(warnings, "BALL_TRACK_ID_NOT_POSITIVE")
            elif subject in ("g2pelvis", "table") and track_id_value != 0:
                contract_valid = False
                self._append_warning(warnings, "NON_BALL_TRACK_ID_NOT_ZERO")
            snapshot = None
            bounce_detected = False
            reset_transition = None
            if subject == "g2pelvis" and contract_valid:
                self._update_pelvis_locked(sample, warnings)
            elif subject == "ball" and contract_valid:
                (
                    snapshot,
                    bounce_detected,
                    reset_transition,
                ) = self._update_ball_locked(sample, warnings)
            elif subject != "table":
                self._append_warning(warnings, "UNKNOWN_SUBJECT")

            return AdapterOutput(
                sample=sample,
                snapshot=snapshot,
                warnings=tuple(warnings),
                estimator_sample_count=int(self.ball_state_estimator.sample_count),
                bounce_detected=bounce_detected,
                reset_transition=reset_transition,
            )

    def reset_estimator_after_strike(self) -> int:
        """Clear estimator without changing the publisher-owned track id."""
        with self.lock:
            self._clear_ball_tracking_locked()
            return 0

    def copy_latest_pelvis(self) -> LatestPelvisPose:
        """Atomically copy accepted pose plus latest validity and timing."""
        with self.lock:
            return LatestPelvisPose(
                position_w=self._accepted_pelvis_position_w,
                quaternion_xyzw=(self._accepted_pelvis_quaternion_xyzw),
                valid=bool(self._pelvis_valid),
                occluded=bool(self._pelvis_occluded),
                source_frame=int(self._pelvis_source_frame),
                source_time_s=self._pelvis_source_time_s,
                publish_time_us=self._pelvis_publish_time_us,
                received_monotonic_s=self._pelvis_received_monotonic_s,
                wall_time_us=self._pelvis_wall_time_us,
            )


def planner_reason_code(result: FrozenPlannerResult) -> Optional[str]:
    """Project the typed planner failure reason only."""
    if result.failure_reason is None:
        return None
    return result.failure_reason.value


@dataclass(frozen=True)
class TaskTickResult:
    lifecycle_now_s: float
    obs_now_s: float
    wall_time_us: int
    phase: str
    lifecycle_decision: str
    lifecycle: LifecycleSnapshot
    advance_decision: LifecycleDecision
    result_decisions: Tuple[LifecycleDecision, ...]
    active_binding: Optional[AttemptBinding]
    command_result: Optional[FrozenPlannerResult]
    completed_results: Tuple[PlannerResultSnapshot, ...]
    frozen_results: Tuple[Optional[FrozenPlannerResult], ...]
    completed_result_queue_depth: int
    completed_result_queue_capacity: int
    completed_result_queue_overflow_count: int
    overflowed_track_ids: Tuple[int, ...]
    compromised_track_ids: Tuple[int, ...]
    command_fields_used: Optional[Mapping[str, JsonValue]]
    task_observation: Optional[TaskObservationResult]
    task_pass: bool
    errors: Tuple[str, ...]


class BallDiagnosticTracker:
    """Track ball-only estimator and incoming state without production gates."""

    def __init__(
        self,
        *,
        estimator_window_size: Optional[int],
        minimum_speed_x_mps: float,
        required_consecutive_snapshots: int,
    ) -> None:
        self.estimator_window_size = None if estimator_window_size is None else int(estimator_window_size)
        if self.estimator_window_size is not None and self.estimator_window_size < 1:
            raise ValueError("estimator_window_size must be positive")
        self.minimum_speed_x_mps = float(minimum_speed_x_mps)
        if not np.isfinite(self.minimum_speed_x_mps) or self.minimum_speed_x_mps <= 0.0:
            raise ValueError("minimum_speed_x_mps must be finite and positive")
        self.required_consecutive_snapshots = int(required_consecutive_snapshots)
        if self.required_consecutive_snapshots < 1:
            raise ValueError("required_consecutive_snapshots must be positive")
        self._lock = threading.RLock()
        self._latest_key: Optional[SnapshotKey] = None
        self._incoming_count = 0
        self._last_estimator_reset_reason: Optional[str] = None
        self._state = self._unavailable_state(
            status="NOT_SEEN",
            incoming_status="NOT_EVALUATED",
            blocker="BALL_NOT_SEEN",
        )

    def _unavailable_state(
        self,
        *,
        status: str,
        incoming_status: str,
        blocker: str,
    ) -> BallDiagnosticState:
        return BallDiagnosticState(
            status=status,
            track_id=None,
            identity_source=None,
            consumed=False,
            estimator_sample_count=None,
            estimator_window_size=self.estimator_window_size,
            speed_mps=None,
            velocity_world_mps=None,
            incoming_count=None,
            incoming_required_count=(self.required_consecutive_snapshots),
            incoming_status=incoming_status,
            blocker=blocker,
        )

    def reset(self, *, blocker: str) -> BallDiagnosticState:
        with self._lock:
            self._latest_key = None
            self._incoming_count = 0
            self._last_estimator_reset_reason = str(blocker)
            self._state = self._unavailable_state(
                status="TRACK_ENDED",
                incoming_status="TRACK_ENDED",
                blocker=str(blocker),
            )
            self._state = replace(
                self._state,
                last_estimator_reset_reason=(self._last_estimator_reset_reason),
            )
            return self._state

    def observe(
        self,
        snapshot: BallEstimateSnapshot,
        *,
        estimator_sample_count: Optional[int],
        raw_position_w: Optional[object] = None,
        last_estimator_reset_reason: Optional[str] = None,
    ) -> BallDiagnosticState:
        key = SnapshotKey(
            int(snapshot.track_id),
            int(snapshot.generation),
        )
        raw_position = np.asarray(
            (snapshot.position_w if raw_position_w is None else raw_position_w),
            dtype=np.float64,
        ).reshape(-1)
        estimated_position = np.asarray(
            snapshot.position_w,
            dtype=np.float64,
        ).reshape(-1)
        observation_fields = {
            "source_frame": int(snapshot.source_frame),
            "track_id": int(snapshot.track_id),
            "identity_source": "wire_v2",
            "consumed": bool(snapshot.consumed),
            "generation": int(snapshot.generation),
            "raw_position_w": (
                tuple(float(value) for value in raw_position)
                if (raw_position.shape == (3,) and np.isfinite(raw_position).all())
                else None
            ),
            "estimated_position_w": (
                tuple(float(value) for value in estimated_position)
                if (estimated_position.shape == (3,) and np.isfinite(estimated_position).all())
                else None
            ),
            "observed_monotonic_s": float(snapshot.received_monotonic_s),
        }
        with self._lock:
            latest = self._latest_key
            if latest is not None:
                if key.track_id < latest.track_id:
                    return self._state
                if key.track_id == latest.track_id and key.generation <= latest.generation:
                    return self._state
                if key.track_id != latest.track_id:
                    self._incoming_count = 0
            self._latest_key = key
            if last_estimator_reset_reason is not None:
                self._last_estimator_reset_reason = str(last_estimator_reset_reason)

            if not bool(snapshot.visible):
                self._incoming_count = 0
                self._state = self._unavailable_state(
                    status="TRACK_ENDED",
                    incoming_status="TRACK_ENDED",
                    blocker="BALL_NOT_VISIBLE",
                )
                self._last_estimator_reset_reason = "BALL_NOT_VISIBLE"
                self._state = replace(
                    self._state,
                    **observation_fields,
                    last_estimator_reset_reason=(self._last_estimator_reset_reason),
                )
                return self._state

            sample_count = None if estimator_sample_count is None else int(estimator_sample_count)
            if not bool(snapshot.ready):
                self._state = BallDiagnosticState(
                    status="ESTIMATING",
                    estimator_sample_count=sample_count,
                    estimator_window_size=self.estimator_window_size,
                    speed_mps=None,
                    velocity_world_mps=None,
                    incoming_count=None,
                    incoming_required_count=(self.required_consecutive_snapshots),
                    incoming_status="NOT_EVALUATED",
                    blocker="ESTIMATOR_WARMING",
                    **observation_fields,
                    last_estimator_reset_reason=(self._last_estimator_reset_reason),
                )
                return self._state

            velocity = np.asarray(
                snapshot.velocity_w,
                dtype=np.float64,
            ).reshape(-1)
            if velocity.shape != (3,) or not np.isfinite(velocity).all():
                self._incoming_count = 0
                self._state = BallDiagnosticState(
                    status="INVALID",
                    estimator_sample_count=sample_count,
                    estimator_window_size=self.estimator_window_size,
                    speed_mps=None,
                    velocity_world_mps=None,
                    incoming_count=None,
                    incoming_required_count=(self.required_consecutive_snapshots),
                    incoming_status="NOT_EVALUATED",
                    blocker="ESTIMATE_NONFINITE",
                    **observation_fields,
                    last_estimator_reset_reason=(self._last_estimator_reset_reason),
                )
                return self._state

            velocity_tuple = tuple(float(value) for value in velocity)
            incoming = bool(velocity_tuple[0] <= -self.minimum_speed_x_mps)
            if incoming:
                self._incoming_count = min(
                    self._incoming_count + 1,
                    self.required_consecutive_snapshots,
                )
            else:
                self._incoming_count = 0
            self._state = BallDiagnosticState(
                status="READY",
                estimator_sample_count=sample_count,
                estimator_window_size=self.estimator_window_size,
                speed_mps=float(np.linalg.norm(velocity)),
                velocity_world_mps=velocity_tuple,
                incoming_count=int(self._incoming_count),
                incoming_required_count=(self.required_consecutive_snapshots),
                incoming_status=("INCOMING" if incoming else "NOT_INCOMING"),
                blocker=(None if incoming else "INCOMING_SPEED_REJECTED"),
                **observation_fields,
                last_estimator_reset_reason=(self._last_estimator_reset_reason),
            )
            return self._state

    def snapshot(self) -> BallDiagnosticState:
        with self._lock:
            return self._state


class ShadowTaskPipeline:
    """Production-equivalent estimator-to-task-observation shadow path."""

    def __init__(
        self,
        *,
        adapter: MocapFrameAdapter,
        settings: HitterRuntimeSettings,
        event_sink,
        raw_sink,
        planner=None,
        planner_factory: Optional[Callable[[], object]] = None,
        rng: Optional[np.random.Generator] = None,
        worker=None,
        attempt_tracker: Optional[AttemptTracker] = None,
        forced_strike_type: Optional[str] = None,
    ) -> None:
        self.adapter = adapter
        self.settings = settings
        self.event_sink = event_sink
        self.raw_sink = raw_sink
        self.planner = (
            planner
            if planner is not None
            else (planner_factory() if planner_factory is not None else build_hitter_system_planner({}))
        )
        self.forced_strike_type = forced_strike_type
        self.rng = np.random.default_rng(settings.hitter_seed) if rng is None else rng
        self.incoming = IncomingTrackConfirmation(
            minimum_speed_x_mps=settings.minimum_incoming_speed_x_mps,
            required_consecutive_snapshots=(settings.incoming_confirmation_snapshots),
        )
        estimator = getattr(adapter, "ball_state_estimator", None)
        estimator_window_size = getattr(estimator, "window_size", None)
        self.ball_diagnostics = BallDiagnosticTracker(
            estimator_window_size=estimator_window_size,
            minimum_speed_x_mps=settings.minimum_incoming_speed_x_mps,
            required_consecutive_snapshots=(settings.incoming_confirmation_snapshots),
        )
        self.lifecycle = build_hitter_command_lifecycle(
            settings,
            rng=self.rng,
        )
        self.attempt_tracker = AttemptTracker() if attempt_tracker is None else attempt_tracker
        self._state_lock = threading.RLock()
        self._last_submit_received_s: Optional[float] = None
        self._last_result_key: Optional[SnapshotKey] = None
        self._frozen_results: "OrderedDict[SnapshotKey, FrozenPlannerResult]" = OrderedDict()
        self._max_frozen_results = 8192
        self._passed_attempt_ids: "OrderedDict[int, None]" = OrderedDict()
        self._planner_start_times: "OrderedDict[SnapshotKey, float]" = OrderedDict()
        self._recorded_stage_identities: "OrderedDict[Tuple[Any, ...], None]" = OrderedDict()
        self._max_stage_identities = 8192
        self._sink_failure_lock = threading.Lock()
        self.event_sink_failures = 0
        self.raw_sink_failures = 0
        self.worker = (
            LatestOnlyPlannerWorker(
                self.plan_snapshot,
                trace_listener=self._on_worker_trace,
            )
            if worker is None
            else worker
        )

    @staticmethod
    def _snapshot_key(snapshot: BallEstimateSnapshot) -> SnapshotKey:
        return SnapshotKey(
            int(snapshot.track_id),
            int(snapshot.generation),
        )

    @staticmethod
    def _result_key(result: PlannerResultSnapshot) -> SnapshotKey:
        return SnapshotKey(
            int(result.track_id),
            int(result.source_generation),
        )

    @staticmethod
    def _validated_vector(value, *, name: str, size: int) -> np.ndarray:
        array = np.asarray(value, dtype=np.float64).reshape(-1)
        if array.shape != (size,):
            raise ValueError(
                "{} must have shape ({},), got {}.".format(
                    name,
                    size,
                    array.shape,
                )
            )
        if not np.isfinite(array).all():
            raise ValueError("{} contains non-finite values.".format(name))
        return array.copy()

    def plan_snapshot(self, snapshot: BallEstimateSnapshot):
        """Copy HitterEnv's real-world planner gate in worker order."""
        if not snapshot.visible:
            self.incoming.reset()
            raise RuntimeError("HITTER ball track ended.")
        if not snapshot.ready:
            raise ValueError("HITTER ball estimator is not ready.")
        if not snapshot.base_valid:
            raise ValueError("HITTER base pose is not valid.")

        ball_position = self._validated_vector(
            snapshot.position_w,
            name="snapshot.position_w",
            size=3,
        )
        ball_velocity = self._validated_vector(
            snapshot.velocity_w,
            name="snapshot.velocity_w",
            size=3,
        )
        if not self.incoming.observe(
            track_id=snapshot.track_id,
            velocity_x_mps=float(ball_velocity[0]),
        ):
            raise ValueError(
                "HITTER ball track is not stably incoming: " "vx={:.3f} m/s.".format(float(ball_velocity[0]))
            )
        base_position = self._validated_vector(
            snapshot.base_position_w,
            name="snapshot.base_position_w",
            size=3,
        )
        base_quaternion = self._validated_vector(
            snapshot.base_quaternion_xyzw,
            name="snapshot.base_quaternion_xyzw",
            size=4,
        )
        norm = float(np.linalg.norm(base_quaternion))
        if norm < 1.0e-6:
            raise ValueError("HITTER snapshot base quaternion has near-zero norm.")
        base_quaternion /= norm
        base_forward_xy = matrix_from_quat(base_quaternion)[:, 0][:2]
        return self.planner.plan_command(
            ball_position,
            ball_velocity,
            current_base_xy_w=base_position,
            base_forward_xy_w=base_forward_xy,
            strike_type=self.forced_strike_type,
        )

    def _offer_event(self, draft: EventDraft) -> bool:
        try:
            offer = getattr(self.event_sink, "offer", None)
            if callable(offer):
                return bool(offer(draft))
            return bool(self.event_sink(draft))
        except Exception:
            with self._sink_failure_lock:
                self.event_sink_failures += 1
            return False

    def _event(
        self,
        *,
        kind: str,
        monotonic_s: float,
        wall_time_us: int,
        binding: Optional[AttemptBinding],
        payload: Mapping[str, JsonValue],
    ) -> None:
        self._offer_event(
            EventDraft(
                kind=kind,
                monotonic_s=float(monotonic_s),
                wall_time_us=int(wall_time_us),
                scope="state" if binding is None else "attempt",
                attempt_id=(None if binding is None else binding.attempt_id),
                payload=payload,
            )
        )

    def _offer_raw(
        self,
        sample: NormalizedMocapSample,
        binding: Optional[AttemptBinding],
    ):
        try:
            submit = getattr(self.raw_sink, "submit", None)
            if callable(submit):
                return submit(
                    sample,
                    attempt_id=(None if binding is None else binding.attempt_id),
                    track_segment_id=(None if binding is None else binding.track_segment_id),
                )
            try:
                return self.raw_sink(sample, binding)
            except TypeError:
                return self.raw_sink(sample)
        except Exception:
            with self._sink_failure_lock:
                self.raw_sink_failures += 1
            return False

    def _publish_transition(self, transition, *, wall_time_us: int) -> None:
        self._offer_event(
            EventDraft(
                kind="attempt_transition",
                monotonic_s=float(transition.monotonic_s),
                wall_time_us=int(wall_time_us),
                scope="attempt",
                attempt_id=int(transition.attempt_id),
                payload={
                    "track_segment_id": transition.track_segment_id,
                    "stage": transition.stage,
                    "reason_code": transition.reason_code,
                    "snapshot_key": (
                        None if transition.snapshot_key is None else transition.snapshot_key.to_json_dict()
                    ),
                    "values": transition.values,
                },
            )
        )

    def _record_stage_once(
        self,
        *,
        binding: Optional[AttemptBinding],
        identity: Tuple[Any, ...],
        stage: str,
        monotonic_s: float,
        wall_time_us: int,
        reason_code: Optional[str],
        values: Mapping[str, JsonValue],
    ) -> None:
        if binding is None:
            return
        with self._state_lock:
            if identity in self._recorded_stage_identities:
                self._recorded_stage_identities.move_to_end(identity)
                return
            try:
                transition = self.attempt_tracker.record_stage(
                    binding=binding,
                    stage=stage,
                    monotonic_s=float(monotonic_s),
                    reason_code=reason_code,
                    values=values,
                )
            except ValueError:
                return
            self._recorded_stage_identities[identity] = None
            while len(self._recorded_stage_identities) > self._max_stage_identities:
                self._recorded_stage_identities.popitem(last=False)
            self._publish_transition(
                transition,
                wall_time_us=wall_time_us,
            )

    def ingest_adapter_output(self, output: AdapterOutput) -> None:
        """Raw record, attempt bind, 100 Hz throttle, then worker submit."""
        submission = None
        with self._state_lock:
            sample = output.sample
            snapshot = output.snapshot
            binding = None
            transitions = []
            if sample.subject == "ball" and snapshot is not None:
                key = self._snapshot_key(snapshot)
                reset = output.reset_transition
                if reset is not None:
                    transitions.extend(
                        self.attempt_tracker.observe_production_reset(
                            previous_track_id=(reset.previous_track_id),
                            new_track_id=reset.new_track_id,
                            reason=reset.reason,
                            monotonic_s=sample.received_monotonic_s,
                        )
                    )
                transitions.extend(
                    self.attempt_tracker.observe_ball_sample(
                        sample,
                        snapshot_key=key,
                    )
                )
                binding = self.attempt_tracker.bind_snapshot(snapshot_key=key)

            self._offer_raw(sample, binding)
            for transition in transitions:
                self._publish_transition(
                    transition,
                    wall_time_us=sample.wall_time_us,
                )
            for warning in output.warnings:
                self._event(
                    kind="adapter_warning",
                    monotonic_s=sample.received_monotonic_s,
                    wall_time_us=sample.wall_time_us,
                    binding=binding,
                    payload={
                        "reason_code": warning,
                        "input_seq": sample.input_seq,
                    },
                )

            if snapshot is None:
                return
            reset_reason = (
                "BOUNCE_RESET"
                if output.bounce_detected
                else (None if output.reset_transition is None else output.reset_transition.reason)
            )
            diagnostic_boundary = bool(output.bounce_detected or not bool(snapshot.visible))
            if output.bounce_detected:
                self.ball_diagnostics.reset(blocker="BOUNCE_RESET")
            if diagnostic_boundary:
                self.ball_diagnostics.observe(
                    snapshot,
                    estimator_sample_count=(output.estimator_sample_count),
                    raw_position_w=sample.position_w,
                    last_estimator_reset_reason=reset_reason,
                )
            received = float(snapshot.received_monotonic_s)
            last = self._last_submit_received_s
            if last is not None and received - last < self.settings.planner_update_interval_s - 1.0e-12:
                return
            self._last_submit_received_s = received
            if not diagnostic_boundary:
                self.ball_diagnostics.observe(
                    snapshot,
                    estimator_sample_count=(output.estimator_sample_count),
                    raw_position_w=sample.position_w,
                    last_estimator_reset_reason=reset_reason,
                )
            submission = (
                snapshot,
                received,
                sample.wall_time_us,
                binding,
                sample.input_seq,
                self.lifecycle.phase.value,
            )

        (
            snapshot,
            received,
            wall_time_us,
            binding,
            input_seq,
            submission_phase,
        ) = submission
        self.worker.submit(snapshot)
        self._event(
            kind="planner_submit",
            monotonic_s=received,
            wall_time_us=wall_time_us,
            binding=binding,
            payload={
                "input_seq": input_seq,
                "snapshot_key": self._snapshot_key(snapshot).to_json_dict(),
                "submission_phase": submission_phase,
            },
        )

    @staticmethod
    def _frozen_result_payload(
        result: Optional[FrozenPlannerResult],
    ):
        if result is None:
            return None
        return {
            "snapshot_key": result.snapshot_key.to_json_dict(),
            "track_id": result.snapshot_key.track_id,
            "generation": result.snapshot_key.generation,
            "source_frame": result.source_frame,
            "strike_deadline_monotonic_s": (result.strike_deadline_monotonic_s),
            "completed_monotonic_s": result.completed_monotonic_s,
            "command_fields": result.command_fields,
            "error_type": result.error_type,
            "error_text": result.error_text,
            "reason_code": planner_reason_code(result),
        }

    @staticmethod
    def _command_fields(command: object) -> Mapping[str, JsonValue]:
        fields = dict(freeze_planner_command_fields(command))
        for name in (
            "strike_table_y_w",
            "strike_side_source",
            "expected_strike_type_from_table_y",
            "strike_type_consistent",
        ):
            try:
                value = getattr(command, name)
            except Exception:
                continue
            fields[name] = value
        frozen = freeze_planner_command_fields(fields)
        return frozen

    def _on_worker_trace(self, trace: PlannerWorkerTrace) -> None:
        key = self._snapshot_key(trace.snapshot)
        binding = self.attempt_tracker.binding_for_result(key)
        replaced_key = None
        if trace.replaced_snapshot is not None:
            replaced_key = self._snapshot_key(trace.replaced_snapshot)
        elif trace.replaced_result is not None:
            replaced_key = trace.replaced_result.snapshot_key
        replaced_binding = None if replaced_key is None else self.attempt_tracker.binding_for_result(replaced_key)
        if trace.kind == "start":
            with self._state_lock:
                self._planner_start_times[key] = float(trace.monotonic_s)
                self._planner_start_times.move_to_end(key)
                while len(self._planner_start_times) > self._max_stage_identities:
                    self._planner_start_times.popitem(last=False)
            if bool(trace.snapshot.ready):
                self._record_stage_once(
                    binding=binding,
                    identity=(
                        ("ESTIMATOR_READY", binding.attempt_id) if binding is not None else ("ESTIMATOR_READY", key)
                    ),
                    stage="ESTIMATOR_READY",
                    monotonic_s=trace.monotonic_s,
                    wall_time_us=0,
                    reason_code=None,
                    values={},
                )
        elif trace.kind == "complete":
            with self._state_lock:
                started_s = self._planner_start_times.pop(
                    key,
                    float(trace.monotonic_s),
                )
            incoming = self.incoming.snapshot()
            required = int(self.settings.incoming_confirmation_snapshots)
            if (
                binding is not None
                and bool(trace.snapshot.visible)
                and bool(trace.snapshot.ready)
                and bool(trace.snapshot.base_valid)
                and incoming.track_id == key.track_id
            ):
                count = int(incoming.consecutive_count)
                incoming_values = {
                    "count": count,
                    "required": required,
                }
                if bool(incoming.confirmed):
                    self._record_stage_once(
                        binding=binding,
                        identity=(
                            "INCOMING_CONFIRMED",
                            binding.attempt_id,
                        ),
                        stage="INCOMING_CONFIRMED",
                        monotonic_s=started_s,
                        wall_time_us=0,
                        reason_code=None,
                        values=incoming_values,
                    )
                elif 0 < count < required:
                    self._record_stage_once(
                        binding=binding,
                        identity=(
                            "INCOMING_CONFIRMING",
                            binding.attempt_id,
                            incoming.track_id,
                            count,
                        ),
                        stage="INCOMING_CONFIRMING",
                        monotonic_s=started_s,
                        wall_time_us=0,
                        reason_code=None,
                        values=incoming_values,
                    )
            frozen = trace.result
            if frozen is not None:
                if frozen.error_type is None and self._required_command_fields(frozen.command_fields):
                    stage = "PLANNER_SUCCEEDED"
                    reason = None
                    values = {}
                else:
                    stage = "PLANNER_REJECTED"
                    reason = "MALFORMED_COMMAND" if frozen.error_type is None else planner_reason_code(frozen)
                    values = {
                        "error_type": frozen.error_type,
                        "error_text": frozen.error_text,
                    }
                self._record_stage_once(
                    binding=binding,
                    identity=(stage, key),
                    stage=stage,
                    monotonic_s=frozen.completed_monotonic_s,
                    wall_time_us=0,
                    reason_code=reason,
                    values=values,
                )
        self._event(
            kind="planner_trace",
            monotonic_s=trace.monotonic_s,
            wall_time_us=0,
            binding=binding,
            payload={
                "trace_seq": trace.trace_seq,
                "trace_kind": trace.kind,
                "snapshot_key": key.to_json_dict(),
                "track_segment_id": (None if binding is None else binding.track_segment_id),
                "snapshot_ready": bool(trace.snapshot.ready),
                "incoming_confirmed": bool(self.incoming.snapshot().confirmed),
                "replaced_snapshot_key": (
                    None
                    if trace.replaced_snapshot is None
                    else self._snapshot_key(trace.replaced_snapshot).to_json_dict()
                ),
                "result": self._frozen_result_payload(trace.result),
                "replaced_result": self._frozen_result_payload(trace.replaced_result),
                "replaced_attempt_id": (None if replaced_binding is None else replaced_binding.attempt_id),
                "replaced_track_segment_id": (None if replaced_binding is None else replaced_binding.track_segment_id),
                "stats": {
                    "submitted": trace.stats.submitted,
                    "completed": trace.stats.completed,
                    "failed": trace.stats.failed,
                    "dropped_pending": trace.stats.dropped_pending,
                    "trace_listener_failures": (trace.stats.trace_listener_failures),
                },
            },
        )
        if replaced_binding is not None and replaced_binding != binding:
            self._event(
                kind="planner_trace_replaced",
                monotonic_s=trace.monotonic_s,
                wall_time_us=0,
                binding=replaced_binding,
                payload={
                    "trace_seq": trace.trace_seq,
                    "trace_kind": trace.kind,
                    "replaced_snapshot_key": (None if replaced_key is None else replaced_key.to_json_dict()),
                    "replacing_snapshot_key": key.to_json_dict(),
                    "replacing_attempt_id": (None if binding is None else binding.attempt_id),
                    "replacing_track_segment_id": (None if binding is None else binding.track_segment_id),
                },
            )

    def _store_frozen_result(
        self,
        result: FrozenPlannerResult,
    ) -> None:
        self._frozen_results[result.snapshot_key] = result
        self._frozen_results.move_to_end(result.snapshot_key)
        while len(self._frozen_results) > self._max_frozen_results:
            self._frozen_results.popitem(last=False)

    @staticmethod
    def _required_command_fields(
        fields_value: Optional[Mapping[str, JsonValue]],
    ) -> bool:
        if not isinstance(fields_value, Mapping):
            return False
        strike_plan = fields_value.get("strike_plan")
        return (
            "p_base_target_xy" in fields_value
            and "v_racket_target_w" in fields_value
            and "time_to_strike" in fields_value
            and isinstance(strike_plan, Mapping)
            and "p_racket_target" in strike_plan
        )

    @classmethod
    def _diagnostic_values_match(
        cls,
        production: Any,
        frozen: Any,
    ) -> bool:
        if isinstance(production, bool) or isinstance(frozen, bool):
            return type(production) is type(frozen) and production == frozen
        production_float = isinstance(
            production,
            (float, np.floating),
        )
        frozen_float = isinstance(frozen, (float, np.floating))
        if production_float or frozen_float:
            if not (production_float and frozen_float):
                return False
            production_value = float(production)
            frozen_value = float(frozen)
            if math.isnan(production_value) and math.isnan(frozen_value):
                return True
            return bool(
                math.isfinite(production_value) and math.isfinite(frozen_value) and production_value == frozen_value
            )
        production_integer = isinstance(
            production,
            (int, np.integer),
        )
        frozen_integer = isinstance(frozen, (int, np.integer))
        if production_integer or frozen_integer:
            return bool(production_integer and frozen_integer and int(production) == int(frozen))
        if type(production) is not type(frozen):
            return False
        if isinstance(production, Mapping) and isinstance(frozen, Mapping):
            if set(production) != set(frozen):
                return False
            return all(
                cls._diagnostic_values_match(
                    production[key],
                    frozen[key],
                )
                for key in production
            )
        if isinstance(production, (tuple, list)) and isinstance(
            frozen,
            (tuple, list),
        ):
            return len(production) == len(frozen) and all(
                cls._diagnostic_values_match(
                    production_item,
                    frozen_item,
                )
                for production_item, frozen_item in zip(
                    production,
                    frozen,
                )
            )
        return bool(production == frozen)

    @classmethod
    def _result_bundle_matches(
        cls,
        result: PlannerResultSnapshot,
        frozen: FrozenPlannerResult,
    ) -> bool:
        if cls._result_key(result) != frozen.snapshot_key:
            return False
        if type(result.source_frame) is not type(frozen.source_frame) or result.source_frame != frozen.source_frame:
            return False
        if not cls._diagnostic_values_match(
            result.strike_deadline_monotonic_s,
            frozen.strike_deadline_monotonic_s,
        ):
            return False
        if not cls._diagnostic_values_match(
            result.completed_monotonic_s,
            frozen.completed_monotonic_s,
        ):
            return False
        frozen_has_diagnostic_error = frozen.error_type is not None or frozen.error_text is not None
        if result.failure_reason is not None:
            if (
                result.command is not None
                or frozen.command_fields is not None
                or frozen.failure_reason is not result.failure_reason
                or result.error_text is None
                or frozen.error_type is None
                or frozen.error_text is None
            ):
                return False
            if frozen.error_type == "PlannerRejected":
                production_detail = result.error_text
            else:
                error_prefix = "{}: ".format(frozen.error_type)
                if not result.error_text.startswith(error_prefix):
                    return False
                production_detail = result.error_text[len(error_prefix) :]
            return production_detail[:2048] == frozen.error_text
        if frozen.failure_reason is not None or frozen_has_diagnostic_error:
            return False
        if result.command is None:
            return frozen.command_fields is None
        if frozen.command_fields is None:
            return False
        try:
            production_fields = cls._command_fields(result.command)
        except Exception:
            return False
        return cls._diagnostic_values_match(
            production_fields,
            frozen.command_fields,
        )

    @classmethod
    def _enriched_frozen_result(
        cls,
        result: PlannerResultSnapshot,
        frozen: FrozenPlannerResult,
    ) -> FrozenPlannerResult:
        if result.command is None:
            return frozen
        return FrozenPlannerResult(
            snapshot_key=frozen.snapshot_key,
            source_frame=frozen.source_frame,
            strike_deadline_monotonic_s=(frozen.strike_deadline_monotonic_s),
            completed_monotonic_s=frozen.completed_monotonic_s,
            command_fields=cls._command_fields(result.command),
            error_type=frozen.error_type,
            error_text=frozen.error_text,
            failure_reason=frozen.failure_reason,
        )

    def _consume_completed_result(
        self,
        result: PlannerResultSnapshot,
        frozen: Optional[FrozenPlannerResult],
        *,
        now: float,
    ) -> Tuple[LifecycleDecision, Tuple[str, ...], Optional[FrozenPlannerResult]]:
        none_decision = LifecycleDecision("none", None)
        result_key = self._result_key(result)
        if frozen is not None and not self._result_bundle_matches(result, frozen):
            return none_decision, ("OBS_COMMAND_MISMATCH",), None
        if result_key == self._last_result_key:
            return LifecycleDecision("ignored_generation", result.track_id), (), None
        self._last_result_key = result_key
        if frozen is None:
            command_fields = None if result.command is None else self._command_fields(result.command)
            frozen = FrozenPlannerResult(
                snapshot_key=result_key,
                source_frame=int(result.source_frame),
                strike_deadline_monotonic_s=float(result.strike_deadline_monotonic_s),
                completed_monotonic_s=float(result.completed_monotonic_s),
                command_fields=command_fields,
                error_type=(None if result.failure_reason is None else "PlannerRejected"),
                error_text=result.error_text,
                failure_reason=result.failure_reason,
            )
            enriched = frozen
        else:
            enriched = self._enriched_frozen_result(result, frozen)
        self._store_frozen_result(enriched)
        if result.command is None or result.error is not None:
            reason = planner_reason_code(frozen)
            if reason == "TRACK_ENDED":
                decision = self.lifecycle.mark_track_ended(
                    result.track_id,
                    now=now,
                )
                return decision, (), enriched
            decision = self.lifecycle.ingest(result, now=now)
            return decision, (), enriched
        if not self._required_command_fields(frozen.command_fields):
            return none_decision, ("OBS_COMMAND_MISMATCH",), enriched
        return self.lifecycle.ingest(result, now=now), (), enriched

    def _binding_for_lifecycle_result(
        self,
        result: Optional[PlannerResultSnapshot],
    ) -> Optional[AttemptBinding]:
        if result is None:
            return None
        return self.attempt_tracker.binding_for_result(self._result_key(result))

    def _assemble_active_task(
        self,
        *,
        obs_now_s: float,
        latest_pelvis: LatestPelvisPose,
    ):
        active = self.lifecycle.active_result
        if self.lifecycle.phase != CommandPhase.ARMED or active is None:
            return None, None, None, False, ()
        key = self._result_key(active)
        binding = self.attempt_tracker.binding_for_result(key)
        frozen = self._frozen_results.get(key)
        if (
            binding is None
            or frozen is None
            or frozen.snapshot_key != key
            or not self._required_command_fields(frozen.command_fields)
        ):
            return binding, frozen, None, False, ("OBS_COMMAND_MISMATCH",)
        fields_value = frozen.command_fields
        strike_plan = fields_value["strike_plan"]
        policy_tts = self.lifecycle.policy_tts(now=obs_now_s)
        try:
            task = assemble_active_hitter_task_observation(
                robot_anchor_position_w=latest_pelvis.position_w,
                robot_anchor_quaternion_xyzw=(latest_pelvis.quaternion_xyzw),
                base_target_xy_w=np.asarray(
                    fields_value["p_base_target_xy"],
                    dtype=np.float32,
                ),
                racket_target_position_w=np.asarray(
                    strike_plan["p_racket_target"],
                    dtype=np.float32,
                ),
                racket_target_velocity_w=np.asarray(
                    fields_value["v_racket_target_w"],
                    dtype=np.float32,
                ),
                policy_time_to_strike_s=policy_tts,
                maximum_policy_time_to_strike_s=(self.settings.maximum_policy_tts_s),
                obs_clip_value=self.settings.obs_clip_value,
            )
        except TaskObservationAssemblyError as exc:
            return binding, frozen, None, False, (exc.reason_code,)
        except (TypeError, ValueError, KeyError):
            return binding, frozen, None, False, ("OBS_COMMAND_MISMATCH",)
        errors = tuple(task.errors)
        passed = bool(
            task.pre_clip.shape == (11,)
            and task.pre_clip.dtype == np.dtype(np.float32)
            and np.isfinite(task.pre_clip).all()
            and task.clip_count == 0
            and not errors
            and 0.0 < policy_tts <= self.settings.maximum_policy_tts_s
        )
        return binding, frozen, task, passed, errors

    def _lifecycle_snapshot(
        self,
        *,
        lifecycle_now: float,
        obs_now: float,
        wall_time_us: int,
        active_key: Optional[SnapshotKey],
        decision: LifecycleDecision,
        queue_depth: int,
        queue_overflow_count: int,
    ) -> LifecycleSnapshot:
        active_track_id = None if self.lifecycle.active_result is None else int(self.lifecycle.active_result.track_id)
        locked_base = (
            None
            if self.lifecycle.locked_base_target_xy is None
            else tuple(float(value) for value in self.lifecycle.locked_base_target_xy)
        )
        locked_deadline = self.lifecycle.locked_strike_deadline_monotonic_s
        policy_tts = self.lifecycle.policy_tts(now=obs_now)
        in_commit = bool(
            locked_deadline is not None and lifecycle_now >= locked_deadline - self.lifecycle.commit_time_to_strike_s
        )
        active_or_locked = active_track_id if active_track_id is not None else self.lifecycle.locked_track_id
        strike_count = 0
        if active_or_locked is not None:
            strike_count = int(
                self.lifecycle.strike_count_by_track_id.get(
                    int(active_or_locked),
                    0,
                )
            )
        return LifecycleSnapshot(
            phase=self.lifecycle.phase.value,
            last_decision=decision.kind,
            active_key=active_key,
            active_track_id=active_track_id,
            decision_track_id=decision.track_id,
            failure_reason=(
                None if self.lifecycle.last_failure_reason is None else self.lifecycle.last_failure_reason.value
            ),
            cancel_reason=(None if decision.cancel_reason is None else decision.cancel_reason.value),
            consumed=bool(decision.consumed_track_ids),
            consumed_track_ids=decision.consumed_track_ids,
            locked_track_id=self.lifecycle.locked_track_id,
            locked_strike_type=self.lifecycle.locked_strike_type,
            locked_base_target_xy=locked_base,
            locked_strike_deadline_monotonic_s=locked_deadline,
            in_commit_window=in_commit,
            policy_tts_s=policy_tts,
            strike_count=strike_count,
            completed_result_queue_depth=int(queue_depth),
            completed_result_queue_capacity=int(self.settings.completed_result_queue_capacity),
            completed_result_queue_overflow_count=int(queue_overflow_count),
            lifecycle_now_s=lifecycle_now,
            obs_now_s=obs_now,
            wall_time_us=int(wall_time_us),
        )

    def tick(
        self,
        *,
        lifecycle_now_s: float,
        obs_now_s: float,
        wall_time_us: int,
    ) -> TaskTickResult:
        """Advance lifecycle once, drain one batch, then assemble task obs."""
        with self._state_lock:
            lifecycle_now = float(lifecycle_now_s)
            observation_now = float(obs_now_s)
            wall_time = int(wall_time_us)
            previous_phase = self.lifecycle.phase
            previous_active = self.lifecycle.active_result
            advance_decision = self.lifecycle.advance(lifecycle_now)
            if (
                previous_phase == CommandPhase.ARMED
                and previous_active is not None
                and self.lifecycle.phase != CommandPhase.ARMED
            ):
                self.adapter.reset_estimator_after_strike()
                self.ball_diagnostics.reset(blocker="STRIKE_DEADLINE_RESET")
                transitions = self.attempt_tracker.observe_production_reset(
                    previous_track_id=int(previous_active.track_id),
                    new_track_id=int(previous_active.track_id),
                    reason="STRIKE_DEADLINE_RESET",
                    monotonic_s=lifecycle_now,
                )
                for transition in transitions:
                    self._publish_transition(
                        transition,
                        wall_time_us=wall_time,
                    )

            batch = self.worker.drain_completed_results()
            if not isinstance(batch, CompletedResultBatch):
                raise TypeError("worker must return CompletedResultBatch")
            result_decisions = []
            consume_errors = []
            enriched_frozen_results = list(batch.frozen_results)
            compromised_track_ids: Tuple[int, ...] = ()
            result_binding = None
            command_result = None
            if batch.overflowed:
                compromised = {int(result.track_id) for result in batch.results} | {
                    int(track_id) for track_id in batch.overflowed_track_ids
                }
                compromised_track_ids = tuple(sorted(compromised))
                for track_id in compromised_track_ids:
                    self.lifecycle.consume_track(
                        track_id,
                        reason=LifecycleCancelReason.RESULT_QUEUE_OVERFLOW.value,
                    )
                overflow_decision = self.lifecycle.cancel(
                    reason=LifecycleCancelReason.RESULT_QUEUE_OVERFLOW,
                    now=lifecycle_now,
                )
                result_decisions.append(overflow_decision)
                self._event(
                    kind="planner_result_queue_overflow",
                    monotonic_s=lifecycle_now,
                    wall_time_us=wall_time,
                    binding=None,
                    payload={
                        "phase": self.lifecycle.phase.value,
                        "decision": overflow_decision.kind,
                        "failure_reason": (
                            None if overflow_decision.cancel_reason is None else overflow_decision.cancel_reason.value
                        ),
                        "cancel_reason": (
                            None if overflow_decision.cancel_reason is None else overflow_decision.cancel_reason.value
                        ),
                        "consumed": bool(overflow_decision.consumed_track_ids),
                        "overflow_count": batch.overflow_count,
                        "overflowed_track_ids": (batch.overflowed_track_ids),
                        "compromised_track_ids": compromised_track_ids,
                        "completed_result_queue_depth": len(batch.results),
                        "completed_result_queue_capacity": (self.settings.completed_result_queue_capacity),
                    },
                )
            else:
                for index, result in enumerate(batch.results):
                    frozen = batch.frozen_results[index]
                    result_binding = self._binding_for_lifecycle_result(result) or result_binding
                    decision_item, errors, enriched = self._consume_completed_result(
                        result,
                        frozen,
                        now=lifecycle_now,
                    )
                    result_decisions.append(decision_item)
                    consume_errors.extend(errors)
                    if enriched is not None:
                        enriched_frozen_results[index] = enriched
                        command_result = enriched
                    self._event(
                        kind="planner_result_ingested",
                        monotonic_s=result.completed_monotonic_s,
                        wall_time_us=wall_time,
                        binding=self._binding_for_lifecycle_result(result),
                        payload={
                            "track_id": result.track_id,
                            "generation": result.source_generation,
                            "source_frame": result.source_frame,
                            "phase": self.lifecycle.phase.value,
                            "decision": decision_item.kind,
                            "failure_reason": (None if result.failure_reason is None else result.failure_reason.value),
                            "cancel_reason": (
                                None if decision_item.cancel_reason is None else decision_item.cancel_reason.value
                            ),
                            "consumed": bool(decision_item.consumed_track_ids),
                            "policy_tts_s": self.lifecycle.policy_tts(now=lifecycle_now),
                            "completed_result_queue_depth": (len(batch.results)),
                        },
                    )

            effective_decision = result_decisions[-1] if result_decisions else advance_decision
            decision = effective_decision.kind
            latest_pelvis = self.adapter.copy_latest_pelvis()
            (
                active_binding,
                active_command_result,
                task_observation,
                task_pass,
                task_errors,
            ) = self._assemble_active_task(
                obs_now_s=observation_now,
                latest_pelvis=latest_pelvis,
            )
            if active_command_result is not None:
                command_result = active_command_result
            errors = tuple(dict.fromkeys(tuple(consume_errors) + task_errors))
            if consume_errors:
                task_pass = False
            active = self.lifecycle.active_result
            lifecycle_active_binding = self._binding_for_lifecycle_result(active)
            if active_binding is None:
                active_binding = lifecycle_active_binding
            lifecycle_event_binding = active_binding
            if self.lifecycle.phase in (
                CommandPhase.RECOVERY,
                CommandPhase.TRACKING,
            ):
                lifecycle_event_binding = result_binding or active_binding
            elif lifecycle_event_binding is None:
                lifecycle_event_binding = result_binding
            active_key = None if active is None else self._result_key(active)
            if decision == "skipped":
                late_binding = result_binding or active_binding
                self._record_stage_once(
                    binding=late_binding,
                    identity=(
                        "LATE_SKIP",
                        None if late_binding is None else late_binding.attempt_id,
                    ),
                    stage="LATE_SKIP",
                    monotonic_s=lifecycle_now,
                    wall_time_us=wall_time,
                    reason_code="LATE_SKIP",
                    values={},
                )
            if decision == "armed":
                armed_binding = active_binding or result_binding
                armed_result = command_result if command_result is not None else frozen
                arm_tts = None
                if armed_result is not None:
                    deadline = float(armed_result.strike_deadline_monotonic_s)
                    if math.isfinite(deadline):
                        arm_tts = deadline - lifecycle_now
                self._record_stage_once(
                    binding=armed_binding,
                    identity=(
                        "ARMED",
                        None if armed_binding is None else armed_binding.attempt_id,
                    ),
                    stage="ARMED",
                    monotonic_s=lifecycle_now,
                    wall_time_us=wall_time,
                    reason_code=None,
                    values={"arm_tts_s": arm_tts},
                )
            if self.lifecycle.phase == CommandPhase.ARMED and active_binding is not None and not task_pass:
                task_reason = str(errors[0]) if errors else "TASK_OBS_FAILED"
                self._record_stage_once(
                    binding=active_binding,
                    identity=(
                        "TASK_OBS_FAILED",
                        active_binding.attempt_id,
                        task_reason,
                    ),
                    stage="TASK_OBS_FAILED",
                    monotonic_s=observation_now,
                    wall_time_us=wall_time,
                    reason_code=task_reason,
                    values={"errors": errors},
                )
            for transition in self.attempt_tracker.set_lifecycle_context(
                phase=self.lifecycle.phase.value,
                active_key=active_key,
                cached_key=None,
                monotonic_s=lifecycle_now,
            ):
                self._publish_transition(
                    transition,
                    wall_time_us=wall_time,
                )
            if task_pass and active_binding is not None and active_binding.attempt_id not in self._passed_attempt_ids:
                transition = self.attempt_tracker.record_task_pass(
                    binding=active_binding,
                    monotonic_s=observation_now,
                )
                self._passed_attempt_ids[active_binding.attempt_id] = None
                while len(self._passed_attempt_ids) > 100:
                    self._passed_attempt_ids.popitem(last=False)
                self._publish_transition(
                    transition,
                    wall_time_us=wall_time,
                )
            lifecycle_snapshot = self._lifecycle_snapshot(
                lifecycle_now=lifecycle_now,
                obs_now=observation_now,
                wall_time_us=wall_time,
                active_key=active_key,
                decision=effective_decision,
                queue_depth=len(batch.results),
                queue_overflow_count=batch.overflow_count,
            )
            self._event(
                kind="lifecycle_tick",
                monotonic_s=lifecycle_now,
                wall_time_us=wall_time,
                binding=lifecycle_event_binding,
                payload={
                    "lifecycle_now_s": lifecycle_now,
                    "obs_now_s": observation_now,
                    "wall_time_us": wall_time,
                    "phase": self.lifecycle.phase.value,
                    "decision": decision,
                    "active_key": (None if active_key is None else active_key.to_json_dict()),
                    "command_result": self._frozen_result_payload(command_result),
                    "completed_results": tuple(
                        self._frozen_result_payload(frozen) for frozen in enriched_frozen_results
                    ),
                    "completed_result_queue_depth": len(batch.results),
                    "completed_result_queue_capacity": (self.settings.completed_result_queue_capacity),
                    "completed_result_queue_overflow_count": (batch.overflow_count),
                    "overflowed_track_ids": batch.overflowed_track_ids,
                    "compromised_track_ids": compromised_track_ids,
                    "failure_reason": lifecycle_snapshot.failure_reason,
                    "cancel_reason": lifecycle_snapshot.cancel_reason,
                    "consumed": lifecycle_snapshot.consumed,
                    "locked_strike_type": (lifecycle_snapshot.locked_strike_type),
                    "policy_tts_s": lifecycle_snapshot.policy_tts_s,
                    "task_pre_clip": (None if task_observation is None else task_observation.pre_clip),
                    "task_post_clip": (None if task_observation is None else task_observation.post_clip),
                    "clip_count": (None if task_observation is None else task_observation.clip_count),
                    "task_pass": task_pass,
                    "errors": errors,
                    "recovery_duration_s": (self.lifecycle.recovery_duration_s),
                },
            )
            return TaskTickResult(
                lifecycle_now_s=lifecycle_now,
                obs_now_s=observation_now,
                wall_time_us=wall_time,
                phase=self.lifecycle.phase.value,
                lifecycle_decision=decision,
                lifecycle=lifecycle_snapshot,
                advance_decision=advance_decision,
                result_decisions=tuple(result_decisions),
                active_binding=active_binding,
                command_result=command_result,
                completed_results=batch.results,
                frozen_results=tuple(enriched_frozen_results),
                completed_result_queue_depth=len(batch.results),
                completed_result_queue_capacity=(self.settings.completed_result_queue_capacity),
                completed_result_queue_overflow_count=batch.overflow_count,
                overflowed_track_ids=batch.overflowed_track_ids,
                compromised_track_ids=compromised_track_ids,
                command_fields_used=(None if command_result is None else command_result.command_fields),
                task_observation=task_observation,
                task_pass=task_pass,
                errors=errors,
            )

    def close(self, timeout_s: Optional[float] = 5.0) -> bool:
        return bool(self.worker.close(timeout_s=timeout_s))


__all__ = [
    "AdapterOutput",
    "BallDiagnosticTracker",
    "LatestPelvisPose",
    "MocapFrameAdapter",
    "ProductionReset",
    "ShadowTaskPipeline",
    "TaskTickResult",
    "planner_reason_code",
]
