from __future__ import annotations

from dataclasses import dataclass
import logging
import threading
import time
from typing import Callable, Mapping

import numpy as np

from utils.hitter_realtime import BallEstimateSnapshot
from utils.hitter_runtime_factory import build_ball_state_estimator


logger = logging.getLogger(__name__)

_NON_MONOTONIC_SOURCE_TIMESTAMP = "NON_MONOTONIC_SOURCE_TIMESTAMP"
_SUPERSEDED_OPERATION = "SUPERSEDED_OPERATION"
_PIPELINE_CLOSED = "PIPELINE_CLOSED"
_BALL_INVALID_OR_OCCLUDED = "BALL_INVALID_OR_OCCLUDED"
_BALL_STALE = "BALL_STALE"
_VICON_TIMESTAMP_SOURCE = "vicon"
_PUBLISH_TIMESTAMP_SOURCE = "publish"
_FALLBACK_TIMESTAMP_SOURCE = "fallback"


def _readonly_float32(
    value,
    *,
    shape: tuple[int, ...],
    field: str,
    finite: bool = True,
) -> np.ndarray:
    array = np.array(value, dtype=np.float32, copy=True)
    if array.shape != shape:
        raise ValueError(
            "{} must have shape {}, got {}".format(field, shape, array.shape)
        )
    if finite and not np.isfinite(array).all():
        raise ValueError("{} must contain only finite values".format(field))
    immutable = np.frombuffer(
        array.tobytes(order="C"),
        dtype=np.float32,
    ).reshape(shape)
    immutable.setflags(write=False)
    return immutable


def _finite_float(value, *, field: str) -> float:
    result = float(value)
    if not np.isfinite(result):
        raise ValueError("{} must be finite".format(field))
    return result


@dataclass(frozen=True)
class BasePoseW:
    position_w: np.ndarray
    quaternion_xyzw: np.ndarray
    valid: bool
    simulation_time_s: float
    captured_monotonic_s: float

    def __post_init__(self) -> None:
        position = _readonly_float32(
            self.position_w,
            shape=(3,),
            field="position_w",
        )
        quaternion = _readonly_float32(
            self.quaternion_xyzw,
            shape=(4,),
            field="quaternion_xyzw",
        )
        quaternion_norm = float(np.linalg.norm(quaternion.astype(np.float64)))
        if not np.isclose(
            quaternion_norm,
            1.0,
            rtol=1.0e-5,
            atol=1.0e-6,
        ):
            raise ValueError(
                "quaternion_xyzw must be normalized, got norm {:.9g}".format(
                    quaternion_norm
                )
            )
        object.__setattr__(self, "position_w", position)
        object.__setattr__(self, "quaternion_xyzw", quaternion)
        object.__setattr__(self, "valid", bool(self.valid))
        object.__setattr__(
            self,
            "simulation_time_s",
            _finite_float(
                self.simulation_time_s,
                field="simulation_time_s",
            ),
        )
        object.__setattr__(
            self,
            "captured_monotonic_s",
            _finite_float(
                self.captured_monotonic_s,
                field="captured_monotonic_s",
            ),
        )


@dataclass(frozen=True)
class BallPipelineState:
    position_w: np.ndarray
    velocity_w: np.ndarray
    visible: bool
    ready: bool
    sample_count: int
    track_epoch: int
    generation: int
    latest_snapshot: BallEstimateSnapshot | None
    last_received_monotonic_s: float | None

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
            "velocity_w",
            _readonly_float32(
                self.velocity_w,
                shape=(3,),
                field="velocity_w",
            ),
        )
        object.__setattr__(self, "visible", bool(self.visible))
        object.__setattr__(self, "ready", bool(self.ready))
        for field in ("sample_count", "track_epoch", "generation"):
            value = int(getattr(self, field))
            if value < 0:
                raise ValueError("{} must be nonnegative".format(field))
            object.__setattr__(self, field, value)
        if self.last_received_monotonic_s is not None:
            object.__setattr__(
                self,
                "last_received_monotonic_s",
                _finite_float(
                    self.last_received_monotonic_s,
                    field="last_received_monotonic_s",
                ),
            )


@dataclass(frozen=True)
class BallPipelineUpdate:
    snapshot: BallEstimateSnapshot | None
    sample_count: int
    bounce_detected: bool
    previous_track_epoch: int
    new_track_epoch: int
    reset_reason: str | None
    rejected_reason: str | None

    def __post_init__(self) -> None:
        for field in (
            "sample_count",
            "previous_track_epoch",
            "new_track_epoch",
        ):
            value = int(getattr(self, field))
            if value < 0:
                raise ValueError("{} must be nonnegative".format(field))
            object.__setattr__(self, field, value)
        object.__setattr__(
            self,
            "bounce_detected",
            bool(self.bounce_detected),
        )


@dataclass(frozen=True)
class _TimestampSelection:
    estimator_time_s: float | None
    source_kind: str | None
    raw_time_s: float | None
    fallback_time_s: float | None
    rejected_reason: str | None


class RealtimeViconBallPipeline:
    def __init__(
        self,
        planner_config: Mapping[str, object],
        base_pose_provider: Callable[[], BasePoseW],
        *,
        stale_timeout_s: float | None,
        monotonic_fn: Callable[[], float] = time.monotonic,
    ) -> None:
        if not callable(base_pose_provider):
            raise TypeError("base_pose_provider must be callable")
        if not callable(monotonic_fn):
            raise TypeError("monotonic_fn must be callable")
        if stale_timeout_s is None:
            timeout = None
        else:
            timeout = float(stale_timeout_s)
            if not np.isfinite(timeout) or timeout <= 0.0:
                raise ValueError(
                    "stale_timeout_s must be finite and positive or None"
                )
        sample_rate_hz = float(
            planner_config.get("state_estimator_sample_rate_hz", 300.0)
        )
        if not np.isfinite(sample_rate_hz) or sample_rate_hz <= 0.0:
            raise ValueError(
                "state_estimator_sample_rate_hz must be finite and positive"
            )

        self.stale_timeout_s = timeout
        self.estimator_sample_rate_hz = sample_rate_hz
        self.ball_state_estimator = build_ball_state_estimator(planner_config)
        self._base_pose_provider = base_pose_provider
        self._monotonic_fn = monotonic_fn
        self._lock = threading.RLock()
        self._listeners: list[
            tuple[object, Callable[[BallEstimateSnapshot], None]]
        ] = []
        self._closed = False

        self._position_w = _readonly_float32(
            np.zeros(3, dtype=np.float32),
            shape=(3,),
            field="position_w",
        )
        self._velocity_w = _readonly_float32(
            np.zeros(3, dtype=np.float32),
            shape=(3,),
            field="velocity_w",
        )
        self._visible = False
        self._ready = False
        self._track_epoch = 0
        self._generation = 0
        self._latest_snapshot: BallEstimateSnapshot | None = None
        self._last_received_monotonic_s: float | None = None
        self._last_estimator_timestamp_s: float | None = None
        self._fallback_estimator_time_s: float | None = None
        self._active_timestamp_source: str | None = None
        self._last_raw_timestamp_s_by_source: dict[str, float] = {}
        self._last_source_frame = 0
        self._last_source_time_s = 0.0
        self._invalidation_latched = True
        self._next_operation_ticket = 0
        self._latest_operation_ticket = 0

    @property
    def lock(self):
        return self._lock

    @property
    def fallback_estimator_time_s(self) -> float | None:
        with self._lock:
            return self._fallback_estimator_time_s

    @property
    def last_estimator_timestamp_s(self) -> float | None:
        with self._lock:
            return self._last_estimator_timestamp_s

    def _set_identity_for_compatibility(
        self,
        *,
        track_epoch: int | None = None,
        generation: int | None = None,
    ) -> None:
        with self._lock:
            if track_epoch is not None:
                value = int(track_epoch)
                if value < 0:
                    raise ValueError("track_epoch must be nonnegative")
                self._track_epoch = value
            if generation is not None:
                value = int(generation)
                if value < 0:
                    raise ValueError("generation must be nonnegative")
                self._generation = value

    @staticmethod
    def _normalized_subject(msg: object) -> str:
        return str(getattr(msg, "name", "") or "").strip().lower()

    @staticmethod
    def _source_fields(msg: object) -> tuple[int, float, int]:
        source_frame = int(
            getattr(msg, "vicon_frame_number", 0) or 0
        )
        try:
            source_time_s = float(
                getattr(msg, "vicon_time_s", 0.0) or 0.0
            )
        except (TypeError, ValueError):
            source_time_s = 0.0
        try:
            publish_time_us = int(
                getattr(msg, "publish_time_us", 0) or 0
            )
        except (TypeError, ValueError, OverflowError):
            publish_time_us = 0
        return source_frame, source_time_s, publish_time_us

    @staticmethod
    def _snapshot_source_time(source_time_s: float) -> float:
        if np.isfinite(source_time_s):
            return float(source_time_s)
        return 0.0

    def _capture_base_pose(self) -> BasePoseW:
        provided = self._base_pose_provider()
        if not isinstance(provided, BasePoseW):
            raise TypeError("base_pose_provider must return BasePoseW")
        return BasePoseW(
            position_w=provided.position_w,
            quaternion_xyzw=provided.quaternion_xyzw,
            valid=provided.valid,
            simulation_time_s=provided.simulation_time_s,
            captured_monotonic_s=provided.captured_monotonic_s,
        )

    def _update_locked(
        self,
        *,
        snapshot: BallEstimateSnapshot | None,
        bounce_detected: bool,
        previous_epoch: int,
        reset_reason: str | None = None,
        rejected_reason: str | None = None,
    ) -> BallPipelineUpdate:
        return BallPipelineUpdate(
            snapshot=snapshot,
            sample_count=int(self.ball_state_estimator.sample_count),
            bounce_detected=bounce_detected,
            previous_track_epoch=previous_epoch,
            new_track_epoch=int(self._track_epoch),
            reset_reason=reset_reason,
            rejected_reason=rejected_reason,
        )

    def _issue_operation_ticket_locked(self) -> int:
        self._next_operation_ticket += 1
        self._latest_operation_ticket = self._next_operation_ticket
        return self._next_operation_ticket

    def _operation_is_current_locked(self, ticket: int) -> bool:
        return (
            not self._closed
            and int(ticket) == self._latest_operation_ticket
        )

    def _rejected_update_locked(self, reason: str) -> BallPipelineUpdate:
        epoch = int(self._track_epoch)
        return self._update_locked(
            snapshot=None,
            bounce_detected=False,
            previous_epoch=epoch,
            rejected_reason=reason,
        )

    def _snapshot_locked(
        self,
        *,
        source_frame: int,
        source_time_s: float,
        received_monotonic_s: float,
        base_pose: BasePoseW,
    ) -> BallEstimateSnapshot:
        self._generation += 1
        snapshot = BallEstimateSnapshot(
            track_epoch=int(self._track_epoch),
            generation=int(self._generation),
            source_frame=int(source_frame),
            source_time_s=self._snapshot_source_time(source_time_s),
            received_monotonic_s=float(received_monotonic_s),
            position_w=self._position_w,
            velocity_w=self._velocity_w,
            base_position_w=base_pose.position_w,
            base_quaternion_xyzw=base_pose.quaternion_xyzw,
            base_valid=bool(base_pose.valid),
            visible=bool(self._visible),
            ready=bool(self._ready),
        )
        self._latest_snapshot = snapshot
        return snapshot

    def _listeners_locked(
        self,
    ) -> tuple[Callable[[BallEstimateSnapshot], None], ...]:
        if self._closed:
            return ()
        return tuple(listener for _token, listener in self._listeners)

    @staticmethod
    def _notify(
        snapshot: BallEstimateSnapshot,
        listeners: tuple[Callable[[BallEstimateSnapshot], None], ...],
    ) -> None:
        for listener in listeners:
            try:
                listener(snapshot)
            except Exception:
                logger.exception("HITTER ball pipeline listener failed")

    def _selected_timestamp_locked(
        self,
        *,
        source_time_s: float,
        publish_time_us: int,
    ) -> _TimestampSelection:
        source_kind = _FALLBACK_TIMESTAMP_SOURCE
        raw_time_s: float | None = None
        if np.isfinite(source_time_s) and source_time_s > 0.0:
            source_kind = _VICON_TIMESTAMP_SOURCE
            raw_time_s = float(source_time_s)
        elif publish_time_us > 0:
            publish_time_s = float(publish_time_us) * 1.0e-6
            if np.isfinite(publish_time_s) and publish_time_s > 0.0:
                source_kind = _PUBLISH_TIMESTAMP_SOURCE
                raw_time_s = publish_time_s

        nominal_dt = 1.0 / self.estimator_sample_rate_hz
        if raw_time_s is not None:
            previous_raw_time_s = self._last_raw_timestamp_s_by_source.get(
                source_kind
            )
            if (
                previous_raw_time_s is not None
                and raw_time_s <= previous_raw_time_s
            ):
                return _TimestampSelection(
                    estimator_time_s=None,
                    source_kind=None,
                    raw_time_s=None,
                    fallback_time_s=None,
                    rejected_reason=_NON_MONOTONIC_SOURCE_TIMESTAMP,
                )
            if self._last_estimator_timestamp_s is None:
                timestamp = raw_time_s
            elif (
                self._active_timestamp_source == source_kind
                and previous_raw_time_s is not None
            ):
                timestamp = (
                    self._last_estimator_timestamp_s
                    + (raw_time_s - previous_raw_time_s)
                )
            else:
                timestamp = self._last_estimator_timestamp_s + nominal_dt
            return _TimestampSelection(
                estimator_time_s=float(timestamp),
                source_kind=source_kind,
                raw_time_s=raw_time_s,
                fallback_time_s=self._fallback_estimator_time_s,
                rejected_reason=None,
            )

        if self._last_estimator_timestamp_s is None:
            timestamp = 0.0
        else:
            timestamp = self._last_estimator_timestamp_s + nominal_dt
        if self._fallback_estimator_time_s is None:
            fallback_time_s = 0.0
        else:
            fallback_time_s = self._fallback_estimator_time_s + nominal_dt
        return _TimestampSelection(
            estimator_time_s=float(timestamp),
            source_kind=source_kind,
            raw_time_s=None,
            fallback_time_s=float(fallback_time_s),
            rejected_reason=None,
        )

    def _commit_timestamp_locked(
        self,
        selection: _TimestampSelection,
    ) -> None:
        self._last_estimator_timestamp_s = float(
            selection.estimator_time_s
        )
        self._active_timestamp_source = selection.source_kind
        if selection.raw_time_s is not None:
            self._last_raw_timestamp_s_by_source[
                str(selection.source_kind)
            ] = float(selection.raw_time_s)
        if selection.source_kind == _FALLBACK_TIMESTAMP_SOURCE:
            self._fallback_estimator_time_s = float(
                selection.fallback_time_s
            )

    def _clear_tracking_locked(self, *, advance_epoch: bool) -> int:
        self.ball_state_estimator.reset()
        self._last_estimator_timestamp_s = None
        self._fallback_estimator_time_s = None
        self._active_timestamp_source = None
        self._last_raw_timestamp_s_by_source.clear()
        self._visible = False
        self._ready = False
        if advance_epoch:
            self._track_epoch += 1
        self._invalidation_latched = True
        return int(self._track_epoch)

    def ingest_transformation(
        self,
        msg: object,
        *,
        received_monotonic_s: float | None = None,
    ) -> BallEstimateSnapshot | None:
        return self.ingest_transformation_update(
            msg,
            received_monotonic_s=received_monotonic_s,
        ).snapshot

    def ingest_transformation_update(
        self,
        msg: object,
        *,
        received_monotonic_s: float | None = None,
    ) -> BallPipelineUpdate:
        if self._normalized_subject(msg) != "ball":
            with self._lock:
                return self._update_locked(
                    snapshot=None,
                    bounce_detected=False,
                    previous_epoch=int(self._track_epoch),
                )

        received = (
            self._monotonic_fn()
            if received_monotonic_s is None
            else received_monotonic_s
        )
        received = _finite_float(
            received,
            field="received_monotonic_s",
        )
        source_frame, source_time_s, publish_time_us = self._source_fields(msg)
        effective_valid = bool(
            getattr(msg, "valid", 1)
        ) and not bool(getattr(msg, "occluded", 0))

        with self._lock:
            if self._closed:
                return self._rejected_update_locked(_PIPELINE_CLOSED)
            operation_ticket = self._issue_operation_ticket_locked()
            if not effective_valid and self._invalidation_latched:
                return self._update_locked(
                    snapshot=None,
                    bounce_detected=False,
                    previous_epoch=int(self._track_epoch),
                )

        if not effective_valid:
            base_pose = self._capture_base_pose()
            with self._lock:
                if not self._operation_is_current_locked(operation_ticket):
                    return self._rejected_update_locked(
                        _SUPERSEDED_OPERATION
                    )
                previous_epoch = int(self._track_epoch)
                self._clear_tracking_locked(advance_epoch=True)
                self._last_received_monotonic_s = received
                self._last_source_frame = source_frame
                self._last_source_time_s = self._snapshot_source_time(
                    source_time_s
                )
                snapshot = self._snapshot_locked(
                    source_frame=source_frame,
                    source_time_s=source_time_s,
                    received_monotonic_s=received,
                    base_pose=base_pose,
                )
                listeners = self._listeners_locked()
                update = self._update_locked(
                    snapshot=snapshot,
                    bounce_detected=False,
                    previous_epoch=previous_epoch,
                    reset_reason=_BALL_INVALID_OR_OCCLUDED,
                )
            self._notify(snapshot, listeners)
            return update

        position = _readonly_float32(
            getattr(msg, "pos_vicon"),
            shape=(3,),
            field="pos_vicon",
        )
        base_pose = self._capture_base_pose()
        with self._lock:
            if not self._operation_is_current_locked(operation_ticket):
                return self._rejected_update_locked(
                    _SUPERSEDED_OPERATION
                )
            previous_epoch = int(self._track_epoch)
            timestamp_selection = self._selected_timestamp_locked(
                source_time_s=source_time_s,
                publish_time_us=publish_time_us,
            )
            if timestamp_selection.rejected_reason is not None:
                return self._update_locked(
                    snapshot=None,
                    bounce_detected=False,
                    previous_epoch=previous_epoch,
                    rejected_reason=timestamp_selection.rejected_reason,
                )
            estimate = self.ball_state_estimator.add_sample(
                position,
                timestamp=timestamp_selection.estimator_time_s,
            )
            self._commit_timestamp_locked(timestamp_selection)
            self._last_received_monotonic_s = received
            self._last_source_frame = source_frame
            self._last_source_time_s = self._snapshot_source_time(
                source_time_s
            )
            self._position_w = _readonly_float32(
                estimate.position,
                shape=(3,),
                field="position_w",
            )
            self._velocity_w = _readonly_float32(
                estimate.velocity,
                shape=(3,),
                field="velocity_w",
            )
            self._visible = True
            self._ready = bool(estimate.valid)
            self._invalidation_latched = False
            snapshot = self._snapshot_locked(
                source_frame=source_frame,
                source_time_s=source_time_s,
                received_monotonic_s=received,
                base_pose=base_pose,
            )
            listeners = self._listeners_locked()
            update = self._update_locked(
                snapshot=snapshot,
                bounce_detected=bool(estimate.bounce_detected),
                previous_epoch=previous_epoch,
            )
        self._notify(snapshot, listeners)
        return update

    def expire_stale(
        self,
        *,
        now: float | None = None,
    ) -> BallEstimateSnapshot | None:
        if self.stale_timeout_s is None:
            return None
        checked_now = self._monotonic_fn() if now is None else now
        checked_now = _finite_float(checked_now, field="now")
        with self._lock:
            if self._closed:
                return None
            received = self._last_received_monotonic_s
            if (
                self._invalidation_latched
                or received is None
                or not self._visible
                or checked_now - received <= self.stale_timeout_s
            ):
                return None
            operation_ticket = self._issue_operation_ticket_locked()
        base_pose = self._capture_base_pose()
        with self._lock:
            if not self._operation_is_current_locked(operation_ticket):
                return None
            received = self._last_received_monotonic_s
            if (
                self._invalidation_latched
                or received is None
                or not self._visible
                or checked_now - received <= self.stale_timeout_s
            ):
                return None
            previous_epoch = int(self._track_epoch)
            self._clear_tracking_locked(advance_epoch=True)
            snapshot = self._snapshot_locked(
                source_frame=self._last_source_frame,
                source_time_s=self._last_source_time_s,
                received_monotonic_s=checked_now,
                base_pose=base_pose,
            )
            listeners = self._listeners_locked()
            self._update_locked(
                snapshot=snapshot,
                bounce_detected=False,
                previous_epoch=previous_epoch,
                reset_reason=_BALL_STALE,
            )
        self._notify(snapshot, listeners)
        return snapshot

    def reset_after_strike(self) -> int:
        with self._lock:
            if self._closed:
                return int(self._track_epoch)
            self._issue_operation_ticket_locked()
            return self._clear_tracking_locked(advance_epoch=True)

    def register_listener(
        self,
        listener: Callable[[BallEstimateSnapshot], None],
    ) -> Callable[[], None]:
        if not callable(listener):
            raise TypeError("listener must be callable")
        token = object()
        with self._lock:
            if not self._closed:
                self._listeners.append((token, listener))

        def unregister() -> None:
            with self._lock:
                for index, (registered_token, _listener) in enumerate(
                    self._listeners
                ):
                    if registered_token is token:
                        self._listeners.pop(index)
                        break

        return unregister

    def state(self) -> BallPipelineState:
        with self._lock:
            return BallPipelineState(
                position_w=self._position_w,
                velocity_w=self._velocity_w,
                visible=bool(self._visible),
                ready=bool(self._ready),
                sample_count=int(self.ball_state_estimator.sample_count),
                track_epoch=int(self._track_epoch),
                generation=int(self._generation),
                latest_snapshot=self._latest_snapshot,
                last_received_monotonic_s=self._last_received_monotonic_s,
            )

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._issue_operation_ticket_locked()
            self._listeners.clear()
            self._closed = True
