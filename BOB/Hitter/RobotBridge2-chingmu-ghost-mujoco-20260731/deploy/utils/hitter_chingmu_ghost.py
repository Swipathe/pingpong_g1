from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from types import SimpleNamespace
from typing import Callable

import lcm
import numpy as np
from unitree_sdk2.lcm_types.transformation_t import transformation_t

from utils.hitter_ball_pipeline import BallPipelineState, RealtimeViconBallPipeline
from utils.read_only_lcm import (
    PublicationAudit,
    PublishDenyLcm,
    ReadOnlyLcmSubscription,
)


def _finite_tuple(
    value,
    *,
    length: int,
    field: str,
) -> tuple[float, ...]:
    array = np.asarray(value, dtype=np.float64).reshape(-1)
    if array.shape != (length,):
        raise ValueError("{} must contain {} values".format(field, length))
    if not np.isfinite(array).all():
        raise ValueError("{} must contain finite values".format(field))
    return tuple(float(item) for item in array.tolist())


def _normalize_subject(name) -> str:
    return str(name or "").strip().lower()


def _normalized_quaternion_xyzw(value, *, field: str) -> np.ndarray:
    quat = np.asarray(value, dtype=np.float64).reshape(-1)
    if quat.shape != (4,):
        raise ValueError("{} must contain 4 values".format(field))
    if not np.isfinite(quat).all():
        raise ValueError("{} must contain finite values".format(field))
    norm = float(np.linalg.norm(quat))
    if not np.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError("{} must be nonzero".format(field))
    return quat / norm


@dataclass(frozen=True)
class ChingMuGhostSourceSettings:
    lcm_url: str
    channel: str
    stale_timeout_s: float
    expected_table_center_w: tuple[float, float, float]
    expected_table_quaternion_xyzw: tuple[float, float, float, float]
    table_position_tolerance_m: float = 0.03
    table_angle_tolerance_deg: float = 3.0
    required_table_confirmations: int = 3
    table_length_m: float = 2.730738
    table_width_m: float = 1.512451
    ball_xy_margin_m: float = 0.50
    ball_min_height_offset_m: float = -0.30
    ball_max_height_offset_m: float = 2.50

    def __post_init__(self) -> None:
        object.__setattr__(self, "lcm_url", str(self.lcm_url))
        object.__setattr__(self, "channel", str(self.channel))
        for field in (
            "stale_timeout_s",
            "table_position_tolerance_m",
            "table_angle_tolerance_deg",
            "table_length_m",
            "table_width_m",
            "ball_xy_margin_m",
            "ball_min_height_offset_m",
            "ball_max_height_offset_m",
        ):
            value = float(getattr(self, field))
            if not np.isfinite(value):
                raise ValueError("{} must be finite".format(field))
            object.__setattr__(self, field, value)
        confirmations = int(self.required_table_confirmations)
        if confirmations < 1:
            raise ValueError("required_table_confirmations must be positive")
        object.__setattr__(
            self,
            "required_table_confirmations",
            confirmations,
        )
        object.__setattr__(
            self,
            "expected_table_center_w",
            _finite_tuple(
                self.expected_table_center_w,
                length=3,
                field="expected_table_center_w",
            ),
        )
        object.__setattr__(
            self,
            "expected_table_quaternion_xyzw",
            tuple(
                float(item)
                for item in _normalized_quaternion_xyzw(
                    self.expected_table_quaternion_xyzw,
                    field="expected_table_quaternion_xyzw",
                ).tolist()
            ),
        )


@dataclass(frozen=True)
class ChingMuGhostSourceStatus:
    table_ready: bool
    table_confirmation_count: int
    last_rejection_reason: str | None
    receive_failure: str | None
    thread_alive: bool
    decoded_count: int
    accepted_ball_count: int
    rejected_count: int
    closed: bool
    last_close_joined: bool | None


class ChingMuGhostBallSource:
    def __init__(
        self,
        settings: ChingMuGhostSourceSettings,
        *,
        pipeline: RealtimeViconBallPipeline,
        decoder: Callable[[bytes], object] = transformation_t.decode,
        lcm_factory=lcm.LCM,
        publication_audit: PublicationAudit,
        monotonic_fn: Callable[[], float] = time.monotonic,
    ) -> None:
        if not callable(decoder):
            raise TypeError("decoder must be callable")
        if not callable(lcm_factory):
            raise TypeError("lcm_factory must be callable")
        if not callable(monotonic_fn):
            raise TypeError("monotonic_fn must be callable")
        self.settings = settings
        self._pipeline = pipeline
        self._decoder = decoder
        self._publication_audit = publication_audit
        self._monotonic_fn = monotonic_fn
        self._status_lock = threading.RLock()
        self._table_ready = False
        self._table_confirmation_count = 0
        self._last_rejection_reason: str | None = None
        self._receive_failure: str | None = None
        self._decoded_count = 0
        self._accepted_ball_count = 0
        self._rejected_count = 0
        self._closed = False
        self._last_close_joined: bool | None = None

        def read_only_factory(url: str):
            return PublishDenyLcm(lcm_factory(url), publication_audit)

        self._subscription = ReadOnlyLcmSubscription(
            lcm_url=settings.lcm_url,
            channel=settings.channel,
            handler=self._handle_lcm,
            loop_callback=self._expire_stale_after_lcm_iteration,
            lcm_factory=read_only_factory,
            poll_timeout_ms=10,
        )

    def start(self) -> None:
        with self._status_lock:
            if self._closed:
                raise RuntimeError("cannot start a closed ChingMu source")
        self._subscription.start()

    def register_hitter_ball_listener(self, listener):
        return self._pipeline.register_listener(listener)

    def reset_ball_state_estimator(self) -> int:
        return int(self._pipeline.reset_after_strike())

    def state(self) -> BallPipelineState:
        return self._pipeline.state()

    def source_status(self) -> ChingMuGhostSourceStatus:
        with self._status_lock:
            return ChingMuGhostSourceStatus(
                table_ready=bool(self._table_ready),
                table_confirmation_count=int(
                    self._table_confirmation_count
                ),
                last_rejection_reason=self._last_rejection_reason,
                receive_failure=self._receive_failure,
                thread_alive=self._subscription.thread_alive,
                decoded_count=int(self._decoded_count),
                accepted_ball_count=int(self._accepted_ball_count),
                rejected_count=int(self._rejected_count),
                closed=bool(self._closed),
                last_close_joined=self._last_close_joined,
            )

    def close(self, *, join_timeout_s: float = 1.0) -> bool:
        joined = self._subscription.close(join_timeout_s=join_timeout_s)
        with self._status_lock:
            self._last_close_joined = bool(joined)
            if not joined:
                return False
            if not self._closed:
                self._closed = True
                close = getattr(self._pipeline, "close", None)
                if callable(close):
                    close()
            return True

    def _record_rejection(self, reason: str) -> None:
        with self._status_lock:
            self._last_rejection_reason = str(reason)
            self._rejected_count += 1

    def _handle_lcm(self, channel: str, data: bytes) -> None:
        try:
            message = self._decoder(data)
        except Exception as exc:
            with self._status_lock:
                self._receive_failure = "{}: {}".format(
                    type(exc).__name__,
                    str(exc),
                )
            return

        with self._status_lock:
            self._decoded_count += 1

        subject = _normalize_subject(getattr(message, "name", ""))
        if subject == "table":
            self._handle_table(message)
        elif subject == "ball":
            self._handle_ball(message)

    def _handle_table(self, message: object) -> None:
        ok, reason = self._table_matches(message)
        if ok:
            with self._status_lock:
                self._table_confirmation_count = min(
                    self.settings.required_table_confirmations,
                    self._table_confirmation_count + 1,
                )
                if (
                    self._table_confirmation_count
                    >= self.settings.required_table_confirmations
                ):
                    self._table_ready = True
            return

        with self._status_lock:
            self._table_ready = False
            self._table_confirmation_count = 0
        self._record_rejection(reason)
        self._submit_invalid_ball_proxy(message, reason)

    def _table_matches(self, message: object) -> tuple[bool, str]:
        if not self._effective_valid(message):
            return False, "TABLE_INVALID"
        try:
            position = np.asarray(
                getattr(message, "pos_vicon"),
                dtype=np.float64,
            ).reshape(-1)
            quaternion = _normalized_quaternion_xyzw(
                getattr(message, "quat_vicon"),
                field="quat_vicon",
            )
        except (TypeError, ValueError):
            return False, "TABLE_INVALID"
        if position.shape != (3,) or not np.isfinite(position).all():
            return False, "TABLE_INVALID"
        expected_position = np.asarray(
            self.settings.expected_table_center_w,
            dtype=np.float64,
        )
        if (
            float(np.linalg.norm(position - expected_position))
            > self.settings.table_position_tolerance_m
        ):
            return False, "TABLE_MISMATCH"
        expected_quaternion = np.asarray(
            self.settings.expected_table_quaternion_xyzw,
            dtype=np.float64,
        )
        dot = abs(float(np.dot(quaternion, expected_quaternion)))
        dot = min(1.0, max(-1.0, dot))
        angle_rad = 2.0 * math.acos(dot)
        if (
            math.degrees(angle_rad)
            > self.settings.table_angle_tolerance_deg
        ):
            return False, "TABLE_MISMATCH"
        return True, ""

    @staticmethod
    def _effective_valid(message: object) -> bool:
        return bool(getattr(message, "valid", 1)) and not bool(
            getattr(message, "occluded", 0)
        )

    def _handle_ball(self, message: object) -> None:
        with self._status_lock:
            table_ready = bool(self._table_ready)
        if not table_ready:
            self._record_rejection("TABLE_NOT_READY")
            return

        if not self._effective_valid(message):
            self._record_rejection("BALL_INVALID_OR_OCCLUDED")
            self._submit_invalid_ball_proxy(
                message,
                "BALL_INVALID_OR_OCCLUDED",
            )
            return

        try:
            position = np.asarray(
                getattr(message, "pos_vicon"),
                dtype=np.float64,
            ).reshape(-1)
        except (TypeError, ValueError):
            self._record_rejection("BALL_INVALID_POSITION")
            self._submit_invalid_ball_proxy(message, "BALL_INVALID_POSITION")
            return
        if position.shape != (3,) or not np.isfinite(position).all():
            self._record_rejection("BALL_INVALID_POSITION")
            self._submit_invalid_ball_proxy(message, "BALL_INVALID_POSITION")
            return
        if not self._ball_inside_bounds(position):
            self._record_rejection("BALL_OUT_OF_BOUNDS")
            self._submit_invalid_ball_proxy(message, "BALL_OUT_OF_BOUNDS")
            return

        try:
            update = self._pipeline.ingest_transformation_update(
                message,
                received_monotonic_s=self._monotonic_fn(),
            )
        except ValueError as exc:
            self._record_rejection("BALL_REJECTED")
            with self._status_lock:
                self._receive_failure = "{}: {}".format(
                    type(exc).__name__,
                    str(exc),
                )
            return
        if getattr(update, "rejected_reason", None):
            self._record_rejection(str(update.rejected_reason))
            return
        with self._status_lock:
            self._accepted_ball_count += 1

    def _ball_inside_bounds(self, position: np.ndarray) -> bool:
        x, y, z = [float(item) for item in position.tolist()]
        half_width = self.settings.table_width_m / 2.0
        return (
            -self.settings.ball_xy_margin_m
            <= x
            <= self.settings.table_length_m + self.settings.ball_xy_margin_m
            and -(half_width + self.settings.ball_xy_margin_m)
            <= y
            <= half_width + self.settings.ball_xy_margin_m
            and self.settings.expected_table_center_w[2]
            + self.settings.ball_min_height_offset_m
            <= z
            <= self.settings.expected_table_center_w[2]
            + self.settings.ball_max_height_offset_m
        )

    def _submit_invalid_ball_proxy(
        self,
        source_message: object,
        reason: str,
    ) -> None:
        proxy = SimpleNamespace(
            name="ball",
            valid=0,
            occluded=1,
            pos_vicon=[0.0, 0.0, 0.0],
            quat_vicon=[0.0, 0.0, 0.0, 1.0],
            vicon_frame_number=int(
                getattr(source_message, "vicon_frame_number", 0) or 0
            ),
            vicon_time_s=float(
                getattr(source_message, "vicon_time_s", 0.0) or 0.0
            ),
            publish_time_us=int(
                getattr(source_message, "publish_time_us", 0) or 0
            ),
        )
        try:
            self._pipeline.ingest_transformation_update(
                proxy,
                received_monotonic_s=self._monotonic_fn(),
            )
        except ValueError as exc:
            with self._status_lock:
                self._receive_failure = "{}: {}".format(
                    type(exc).__name__,
                    str(exc),
                )
                self._last_rejection_reason = str(reason)

    def _expire_stale_after_lcm_iteration(self) -> None:
        try:
            snapshot = self._pipeline.expire_stale(now=self._monotonic_fn())
        except ValueError as exc:
            with self._status_lock:
                self._receive_failure = "{}: {}".format(
                    type(exc).__name__,
                    str(exc),
                )
            return
        if snapshot is not None and not bool(getattr(snapshot, "visible", True)):
            with self._status_lock:
                self._last_rejection_reason = "BALL_STALE"
