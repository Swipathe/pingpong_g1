from __future__ import annotations

import time
import lcm
import copy
import select
import threading
from collections import deque
from dataclasses import dataclass
from enum import Enum
from numbers import Real
from typing import Callable
import numpy as np
from simulator.base_sim import BaseSim
from loguru import logger
from scipy.spatial.transform import Rotation as sRot

import sys
sys.path.append('../')
from unitree_sdk2.lcm_types.body_control_data_lcmt import body_control_data_lcmt
from unitree_sdk2.lcm_types.rc_command_lcmt import rc_command_lcmt
from unitree_sdk2.lcm_types.state_estimator_lcmt import state_estimator_lcmt
from unitree_sdk2.lcm_types.pd_tau_targets_lcmt import pd_tau_targets_lcmt
from unitree_sdk2.lcm_types.transformation_t import transformation_t

from utils.helpers import get_gravity, get_rpy
from utils.hitter_realtime import BallEstimateSnapshot
from utils.hitter_runtime_factory import (
    build_ball_state_estimator,
    resolve_vicon_consumer_settings,
)


class ViconInputFault(str, Enum):
    VICON_SCHEMA_ERROR = "VICON_SCHEMA_ERROR"
    VICON_STREAM_STALE = "VICON_STREAM_STALE"
    TRACK_ID_CONFLICT = "TRACK_ID_CONFLICT"
    BALL_MESSAGE_STALE = "BALL_MESSAGE_STALE"


class ViconEventReason(str, Enum):
    TRACK_ENDED = "TRACK_ENDED"
    BASE_POSE_INVALID = "BASE_POSE_INVALID"
    VICON_SCHEMA_ERROR = "VICON_SCHEMA_ERROR"
    VICON_STREAM_STALE = "VICON_STREAM_STALE"
    BALL_MESSAGE_STALE = "BALL_MESSAGE_STALE"
    TRACK_ID_CONFLICT = "TRACK_ID_CONFLICT"


@dataclass(frozen=True)
class ViconConsumerEvent:
    sequence: int
    reason: ViconEventReason
    track_id: int | None
    detail: str


@dataclass(frozen=True)
class ViconConsumerStatus:
    stream_fresh: bool
    ball_fresh: bool
    base_pose_valid: bool
    active_track_id: int | None
    last_track_id: int | None
    visible: bool
    ready_for_new_serve: bool
    latched_fault: ViconInputFault | None

class RealWorld(BaseSim):
    is_real = True
    def __init__(self, config):
        super().__init__(config)
        self._init_communication()
        self.spin()
        self.sync = True
        while True:
            if self.connected():
                break
            time.sleep(0.001)
    
    def _setup(self):
        super()._setup()
        self.lc = lcm.LCM('udpm://239.255.76.67:7667?ttl=255')
    
    def _load_asset(self):
        super()._load_asset()
        self.joint_serial_num = self.cfg.asset.joint_order
        self.policy_joint_order = list(self.joint_serial_num.keys())
        # index for policy to robot
        self.idx_r2p=[self.joint_serial_num[k] for k in self.joint_serial_num]
        # index for robot to policy
        self.idx_p2r=[self.idx_r2p.index(i) for i in range(self.num_dof)]

    def _init_low_state(self):
        super()._init_low_state()

        # Initialize ref_dof_pos for residual control and target_dof_pos
        self.ref_dof_pos = None
        self.target_dof_pos = None

        # remote controller part
        self.mode = 0
        self.ctrlmode_left = 0
        self.ctrlmode_right = 0
        self.left_stick = [0, 0]
        self.right_stick = [0, 0]
        self.left_upper_switch = 0
        self.left_lower_left_switch = 0
        self.left_lower_right_switch = 0
        self.right_upper_switch = 0
        self.right_lower_left_switch = 0
        self.right_lower_right_switch = 0
        self.left_upper_switch_pressed = 0
        self.left_lower_left_switch_pressed = 0
        self.left_lower_right_switch_pressed = 0
        self.right_upper_switch_pressed = 0
        self.right_lower_left_switch_pressed = 0
        self.right_lower_right_switch_pressed = 0

        # LCM callbacks are asynchronous; keep safe defaults so startup can
        # call get_state() before every stream has delivered its first packet.
        self.root_trans_tmp = np.zeros(3, dtype=np.float32)
        self.root_rpy_tmp = np.zeros(3, dtype=np.float32)
        self.root_quat_tmp = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        self.base_lin_vel_tmp = np.zeros(3, dtype=np.float32)
        self.base_ang_vel_tmp = np.zeros(3, dtype=np.float32)
        self.root_trans_world_tmp = np.zeros(3, dtype=np.float32)
        self.root_quat_world_tmp = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        self.dof_pos_tmp = np.zeros(self.num_dof, dtype=np.float32)
        self.dof_vel_tmp = np.zeros(self.num_dof, dtype=np.float32)

        # Heading reset (yaw-only) to avoid large initial yaw correction when IMU world frame
        # (initialized at boot) is different from robot orientation at policy start.
        self._heading_inv_rot = None  # scipy Rotation
        self._reset_heading_on_start = bool(getattr(self.cfg.control, "reset_heading_on_start", True))

        self._init_ball_state()
        self.terminate_on_r2 = bool(getattr(self.cfg.control, "terminate_on_r2", True))
        logger.info(f"[RealWorld] terminate_on_r2={self.terminate_on_r2}")

    def _init_ball_state(self) -> None:
        self._hitter_ball_state_lock = threading.RLock()
        self.ball_pos_world = np.zeros(3, dtype=np.float32)
        self.ball_vel_world = np.zeros(3, dtype=np.float32)
        self.ball_pos_world_tmp = np.zeros(3, dtype=np.float32)
        self.ball_vel_world_tmp = np.zeros(3, dtype=np.float32)
        if not hasattr(self, "root_trans_world_tmp"):
            self.root_trans_world_tmp = np.zeros(3, dtype=np.float32)
        if not hasattr(self, "root_quat_world_tmp"):
            self.root_quat_world_tmp = np.array(
                [0.0, 0.0, 0.0, 1.0],
                dtype=np.float32,
            )
        self.ball_visible = False
        self.ball_visible_tmp = False
        self.base_pose_valid = False
        self.base_pose_valid_tmp = False
        self.firstReceiveBall = False
        self.firstReceiveVicon = False
        self._hitter_ball_listener_lock = threading.Lock()
        self._hitter_ball_listeners: list[
            tuple[object, Callable[[BallEstimateSnapshot], None]]
        ] = []
        self.ball_snapshot_generation = 0
        self.ball_track_id = 0
        self.latest_ball_snapshot = None
        self.active_ball_track_id: int | None = None
        self.last_ball_track_id: int | None = None
        self.seen_ball_track_ids: set[int] = set()
        self.consumed_ball_track_ids: set[int] = set()
        self.ball_track_consumption_reasons: dict[int, str] = {}
        self.admitted_ball_track_ids: set[int] = set()
        self.ball_track_generations: dict[int, int] = {}
        self.ball_track_last_frames: dict[int, int] = {}
        self._pending_ball_track_id: int | None = None
        self._pending_ball_admit_after_monotonic_s: float | None = None
        self._last_any_vicon_monotonic_s: float | None = None
        self._last_ball_vicon_monotonic_s: float | None = None
        self._no_ball_since_monotonic_s: float | None = None
        self._hitter_policy_session_open = False
        self.vicon_fault_latched: ViconInputFault | None = None
        self._vicon_schema_fault_seen = False
        self._vicon_fault_event_emitted: set[ViconInputFault] = set()
        self._vicon_event_sequence = 0
        self._vicon_events: deque[ViconConsumerEvent] = deque()
        self._vicon_event_overflowed = False
        self._last_base_pose_wait_log_monotonic_s = None
        planner_cfg = {}
        motion_cfg = getattr(self.cfg, "motion", None)
        if motion_cfg is not None:
            planner_cfg = motion_cfg.get("ball_planner", {}) or {}
        if motion_cfg is None or "vicon_consumer" not in motion_cfg:
            vicon_cfg = {}
        else:
            vicon_cfg = motion_cfg["vicon_consumer"]
        self.vicon_consumer_settings = resolve_vicon_consumer_settings(
            vicon_cfg
        )
        self.ball_state_estimator_sample_rate_hz = float(planner_cfg.get("state_estimator_sample_rate_hz", 300.0))
        self.ball_state_estimator_time = None
        self.ball_state_estimator_last_host_time = None
        self.ball_state_estimator = build_ball_state_estimator(
            planner_cfg
        )
        self.ball_state_estimator_ready = False
        self.ball_state_estimator_ready_tmp = False
        self.ball_state_estimator_sample_count = 0
        self.ball_state_estimator_sample_count_tmp = 0
        self.ball_state_estimator_min_samples = self.ball_state_estimator.min_samples

    def register_hitter_ball_listener(
        self,
        listener: Callable[[BallEstimateSnapshot], None],
    ) -> Callable[[], None]:
        token = object()
        with self._hitter_ball_listener_lock:
            self._hitter_ball_listeners.append((token, listener))

        def unregister() -> None:
            with self._hitter_ball_listener_lock:
                for index, (registered_token, _listener) in enumerate(
                    self._hitter_ball_listeners
                ):
                    if registered_token is token:
                        self._hitter_ball_listeners.pop(index)
                        break

        return unregister

    @staticmethod
    def _validate_hitter_snapshot(snapshot: object) -> BallEstimateSnapshot:
        if not isinstance(snapshot, BallEstimateSnapshot):
            raise TypeError("snapshot must be a BallEstimateSnapshot")
        if type(snapshot.track_id) is not int or snapshot.track_id <= 0:
            raise ValueError("snapshot track_id must be a positive integer")
        if type(snapshot.generation) is not int or snapshot.generation < 0:
            raise ValueError(
                "snapshot generation must be a non-negative integer"
            )
        if type(snapshot.source_frame) is not int or snapshot.source_frame < 0:
            raise ValueError(
                "snapshot source_frame must be a non-negative integer"
            )
        for name in ("source_time_s", "received_monotonic_s"):
            value = getattr(snapshot, name)
            if (
                isinstance(value, bool)
                or not isinstance(value, Real)
                or not np.isfinite(value)
                or value < 0.0
            ):
                raise ValueError(
                    f"snapshot {name} must be finite and non-negative"
                )
        for name, size in (
            ("position_w", 3),
            ("velocity_w", 3),
            ("base_position_w", 3),
            ("base_quaternion_xyzw", 4),
        ):
            try:
                value = np.asarray(getattr(snapshot, name), dtype=np.float64)
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f"snapshot {name} must contain {size} finite values"
                ) from exc
            if value.shape != (size,) or not np.isfinite(value).all():
                raise ValueError(
                    f"snapshot {name} must contain {size} finite values"
                )
        if float(np.linalg.norm(snapshot.base_quaternion_xyzw)) == 0.0:
            raise ValueError("snapshot base quaternion must be nonzero")
        for name in (
            "base_valid",
            "visible",
            "ready",
            "consumed",
            "new_track",
        ):
            if type(getattr(snapshot, name)) is not bool:
                raise ValueError(f"snapshot {name} must be a bool")
        return snapshot

    def _hitter_snapshot_planning_eligible_locked(
        self,
        snapshot: BallEstimateSnapshot,
    ) -> bool:
        track_id = snapshot.track_id
        latest = self.latest_ball_snapshot
        return bool(
            self._hitter_policy_session_open
            and self.vicon_fault_latched is None
            and track_id in self.admitted_ball_track_ids
            and track_id not in self.consumed_ball_track_ids
            and self.active_ball_track_id == track_id
            and not snapshot.consumed
            and latest is not None
            and not latest.consumed
            and latest.track_id == track_id
            and latest.generation == snapshot.generation
            and latest.source_frame == snapshot.source_frame
            and self.ball_track_generations.get(track_id)
            == snapshot.generation
            and self.ball_track_last_frames.get(track_id)
            == snapshot.source_frame
        )

    def hitter_snapshot_planning_eligible(
        self,
        snapshot: BallEstimateSnapshot,
    ) -> bool:
        snapshot = self._validate_hitter_snapshot(snapshot)
        with self._hitter_ball_state_lock:
            return self._hitter_snapshot_planning_eligible_locked(snapshot)

    def _prepare_ball_snapshot_locked(
        self,
        msg,
        *,
        received_monotonic_s: float,
        consumed: bool | None = None,
        new_track: bool = False,
    ) -> tuple[
        BallEstimateSnapshot,
        tuple[Callable[[BallEstimateSnapshot], None], ...],
    ]:
        source_frame = int(getattr(msg, "vicon_frame_number", 0) or 0)
        source_time_s = float(getattr(msg, "vicon_time_s", 0.0) or 0.0)
        track_id = int(getattr(msg, "track_id", 0) or 0)
        if consumed is None:
            consumed = track_id in self.consumed_ball_track_ids
        with self._hitter_ball_listener_lock:
            snapshot = BallEstimateSnapshot(
                track_id=track_id,
                generation=int(self.ball_snapshot_generation),
                source_frame=source_frame,
                source_time_s=source_time_s,
                received_monotonic_s=float(received_monotonic_s),
                position_w=self.ball_pos_world_tmp,
                velocity_w=self.ball_vel_world_tmp,
                base_position_w=self.root_trans_world_tmp,
                base_quaternion_xyzw=self.root_quat_world_tmp,
                base_valid=bool(self.base_pose_valid_tmp),
                visible=bool(self.ball_visible_tmp),
                ready=bool(self.ball_state_estimator_ready_tmp),
                consumed=bool(consumed),
                new_track=bool(new_track),
            )
            self.latest_ball_snapshot = snapshot
            listeners = (
                tuple(
                    listener
                    for _token, listener in self._hitter_ball_listeners
                )
                if self._hitter_snapshot_planning_eligible_locked(snapshot)
                else ()
            )
        return snapshot, listeners

    @staticmethod
    def _notify_ball_snapshot_listeners(snapshot, listeners) -> None:
        for listener in listeners:
            try:
                listener(snapshot)
            except Exception:
                logger.exception("[RealWorld] HITTER ball listener failed")

    def _publish_ball_snapshot(
        self,
        msg,
        *,
        received_monotonic_s: float,
    ) -> BallEstimateSnapshot:
        with self._hitter_ball_state_lock:
            snapshot, listeners = self._prepare_ball_snapshot_locked(
                msg,
                received_monotonic_s=received_monotonic_s,
            )
        self._notify_ball_snapshot_listeners(snapshot, listeners)
        return snapshot

    def reset_ball_state_estimator(self) -> None:
        self._clear_ball_tracking()
        logger.info("[RealWorld] ball state estimator reset.")

    def _clear_ball_tracking(
        self,
        *,
        emit_snapshot: bool = False,
        source_message=None,
        received_monotonic_s: float | None = None,
        advance_track_id: bool = False,
    ) -> None:
        notification = None
        with self._hitter_ball_state_lock:
            self._clear_ball_tracking_locked()
            if emit_snapshot:
                if source_message is None:
                    raise ValueError("source_message is required when emitting a snapshot")
                if received_monotonic_s is None:
                    received_monotonic_s = time.monotonic()
                notification = self._prepare_ball_snapshot_locked(
                    source_message,
                    received_monotonic_s=received_monotonic_s,
                )
        if notification is not None:
            self._notify_ball_snapshot_listeners(*notification)

    def _clear_ball_tracking_locked(self) -> None:
        self.ball_state_estimator.reset()
        self.ball_state_estimator_time = None
        self.ball_state_estimator_last_host_time = None
        self.ball_state_estimator_ready = False
        self.ball_state_estimator_ready_tmp = False
        self.ball_state_estimator_sample_count = 0
        self.ball_state_estimator_sample_count_tmp = 0

    def _clear_pending_ball_admission_locked(
        self,
        track_id: int | None = None,
    ) -> bool:
        pending_track_id = self._pending_ball_track_id
        if pending_track_id is None:
            return False
        if track_id is not None and pending_track_id != int(track_id):
            return False
        self._pending_ball_track_id = None
        self._pending_ball_admit_after_monotonic_s = None
        return True

    def _reject_pending_ball_admission_locked(
        self,
        track_id: int,
        *,
        reason: str,
    ) -> bool:
        if not self._clear_pending_ball_admission_locked(track_id):
            return False
        self.consumed_ball_track_ids.add(track_id)
        self.ball_track_consumption_reasons[track_id] = (
            f"pending_admission_rejected:{reason}"
        )
        self._mark_latest_ball_snapshot_consumed_locked(track_id)
        self._clear_ball_tracking_locked()
        logger.warning(
            "HITTER Vicon pending-track admission: track_id={} "
            "state=rejected admitted=false reason={}",
            track_id,
            reason,
        )
        return True

    def _quarantine_active_planner_eligibility_locked(self) -> None:
        track_id = self.active_ball_track_id
        if track_id is None:
            return
        if self._reject_pending_ball_admission_locked(
            track_id,
            reason="planner_eligibility_quarantined",
        ):
            return
        self.consumed_ball_track_ids.add(track_id)
        self._mark_latest_ball_snapshot_consumed_locked(track_id)
        self._clear_ball_tracking_locked()

    def _enqueue_vicon_event_locked(
        self,
        reason: ViconEventReason,
        *,
        track_id: int | None,
        detail: str,
    ) -> None:
        if self._vicon_event_overflowed:
            return
        self._vicon_event_sequence += 1
        if (
            len(self._vicon_events)
            >= self.vicon_consumer_settings.event_queue_capacity
        ):
            self._quarantine_active_planner_eligibility_locked()
            self._vicon_events.clear()
            self._vicon_event_overflowed = True
            self._vicon_schema_fault_seen = True
            self.vicon_fault_latched = ViconInputFault.VICON_SCHEMA_ERROR
            self._vicon_fault_event_emitted.add(
                ViconInputFault.VICON_SCHEMA_ERROR
            )
            self._vicon_events.append(
                ViconConsumerEvent(
                    sequence=self._vicon_event_sequence,
                    reason=ViconEventReason.VICON_SCHEMA_ERROR,
                    track_id=None,
                    detail=(
                        "Vicon transition event queue overflow; "
                        "consumer permanently failed closed"
                    ),
                )
            )
            return
        self._vicon_events.append(
            ViconConsumerEvent(
                sequence=self._vicon_event_sequence,
                reason=reason,
                track_id=track_id,
                detail=str(detail),
            )
        )

    def _latch_vicon_fault(
        self,
        fault: ViconInputFault,
        *,
        received_monotonic_s: float,
        detail: str,
        track_id: int | None = None,
    ) -> None:
        with self._hitter_ball_state_lock:
            if fault is ViconInputFault.VICON_SCHEMA_ERROR:
                self._vicon_schema_fault_seen = True
                self._quarantine_active_planner_eligibility_locked()
            if (
                self.vicon_fault_latched is None
                or fault is ViconInputFault.VICON_SCHEMA_ERROR
            ):
                self.vicon_fault_latched = fault
            if fault not in self._vicon_fault_event_emitted:
                self._vicon_fault_event_emitted.add(fault)
                self._enqueue_vicon_event_locked(
                    ViconEventReason(fault.value),
                    track_id=track_id,
                    detail=detail,
                )

    def _end_active_ball_locked(
        self,
        *,
        now_monotonic_s: float,
        event_reason: ViconEventReason | None,
        detail: str,
    ) -> int | None:
        track_id = self.active_ball_track_id
        if track_id is None:
            return None
        if self._pending_ball_track_id == track_id:
            pending_end_reason = (
                event_reason.value
                if event_reason is not None
                else detail.replace(" ", "_")
            )
            self._reject_pending_ball_admission_locked(
                track_id,
                reason=pending_end_reason,
            )
        self.active_ball_track_id = None
        self.ball_visible_tmp = False
        self.ball_visible = False
        self._no_ball_since_monotonic_s = float(now_monotonic_s)
        self._clear_ball_tracking_locked()
        if event_reason is not None:
            self._enqueue_vicon_event_locked(
                event_reason,
                track_id=track_id,
                detail=detail,
            )
        return track_id

    def _stream_fresh_locked(self, now_monotonic_s: float) -> bool:
        last = self._last_any_vicon_monotonic_s
        return bool(
            last is not None
            and float(now_monotonic_s) - last
            <= self.vicon_consumer_settings.stream_timeout_s
        )

    def _ball_fresh_locked(self, now_monotonic_s: float) -> bool:
        last = self._last_ball_vicon_monotonic_s
        if last is None:
            return True
        return bool(
            float(now_monotonic_s) - last
            <= self.vicon_consumer_settings.ball_timeout_s
        )

    def _apply_freshness_locked(self, now_monotonic_s: float) -> None:
        now = float(now_monotonic_s)
        last_any = self._last_any_vicon_monotonic_s
        if (
            last_any is not None
            and now - last_any
            > self.vicon_consumer_settings.stream_timeout_s
        ):
            ended_track = self._end_active_ball_locked(
                now_monotonic_s=now,
                event_reason=None,
                detail="authoritative v2 stream became stale",
            )
            self.base_pose_valid_tmp = False
            self.base_pose_valid = False
            self._latch_vicon_fault(
                ViconInputFault.VICON_STREAM_STALE,
                received_monotonic_s=now,
                detail="authoritative v2 stream age exceeded timeout",
                track_id=ended_track,
            )
            return

        last_ball = self._last_ball_vicon_monotonic_s
        if (
            self.active_ball_track_id is not None
            and last_ball is not None
            and now - last_ball
            > self.vicon_consumer_settings.ball_timeout_s
        ):
            ended_track = self._end_active_ball_locked(
                now_monotonic_s=now,
                event_reason=None,
                detail="active ball message became stale",
            )
            self._latch_vicon_fault(
                ViconInputFault.BALL_MESSAGE_STALE,
                received_monotonic_s=now,
                detail="active ball message age exceeded timeout",
                track_id=ended_track,
            )

    def _status_locked(self, now_monotonic_s: float) -> ViconConsumerStatus:
        now = float(now_monotonic_s)
        stream_fresh = self._stream_fresh_locked(now)
        ball_fresh = self._ball_fresh_locked(now)
        no_ball_long_enough = bool(
            self._no_ball_since_monotonic_s is not None
            and now - self._no_ball_since_monotonic_s
            >= self.vicon_consumer_settings.new_serve_no_ball_s
        )
        visible = bool(self.ball_visible_tmp)
        ready = bool(
            self._hitter_policy_session_open
            and self.vicon_fault_latched is None
            and stream_fresh
            and self.base_pose_valid_tmp
            and self.active_ball_track_id is None
            and not visible
            and no_ball_long_enough
        )
        return ViconConsumerStatus(
            stream_fresh=stream_fresh,
            ball_fresh=ball_fresh,
            base_pose_valid=bool(self.base_pose_valid_tmp),
            active_track_id=self.active_ball_track_id,
            last_track_id=self.last_ball_track_id,
            visible=visible,
            ready_for_new_serve=ready,
            latched_fault=self.vicon_fault_latched,
        )

    def hitter_vicon_status(
        self,
        *,
        now_monotonic_s: float,
    ) -> ViconConsumerStatus:
        now = float(now_monotonic_s)
        if not np.isfinite(now):
            raise ValueError("now_monotonic_s must be finite")
        with self._hitter_ball_state_lock:
            self._apply_freshness_locked(now)
            return self._status_locked(now)

    def drain_hitter_vicon_events(self) -> tuple[ViconConsumerEvent, ...]:
        with self._hitter_ball_state_lock:
            events = tuple(self._vicon_events)
            self._vicon_events.clear()
            return events

    @staticmethod
    def _validate_hitter_consumption_request(
        track_id: object,
        reason: object,
    ) -> tuple[int, str]:
        if type(track_id) is not int or track_id <= 0:
            raise ValueError("track_id must be positive")
        if type(reason) is not str or not reason:
            raise ValueError("reason must be a non-empty string")
        return track_id, reason

    def _consume_hitter_track_locked(self, track_id: int, reason: str) -> bool:
        if track_id not in self.seen_ball_track_ids:
            return False
        was_pending = self._clear_pending_ball_admission_locked(track_id)
        if was_pending:
            logger.warning(
                "HITTER Vicon pending-track admission: track_id={} "
                "state=rejected admitted=false reason=explicit_consume:{}",
                track_id,
                reason,
            )
        already_consumed = track_id in self.consumed_ball_track_ids
        self.consumed_ball_track_ids.add(track_id)
        self.ball_track_consumption_reasons[track_id] = reason
        self._mark_latest_ball_snapshot_consumed_locked(track_id)
        if self.active_ball_track_id == track_id:
            self._clear_ball_tracking_locked()
        return not already_consumed

    def consume_hitter_track(self, track_id: int, *, reason: str) -> bool:
        track_id, reason = self._validate_hitter_consumption_request(
            track_id,
            reason,
        )
        with self._hitter_ball_state_lock:
            return self._consume_hitter_track_locked(track_id, reason)

    def end_hitter_policy_session(self, *, reason: str) -> tuple[int, ...]:
        if type(reason) is not str or not reason:
            raise ValueError("reason must be a non-empty string")
        with self._hitter_ball_state_lock:
            self._hitter_policy_session_open = False
            involved_track_ids = set()
            pending_track_id = self._pending_ball_track_id
            if pending_track_id is not None:
                involved_track_ids.add(pending_track_id)
                self._consume_hitter_track_locked(pending_track_id, reason)
            if self.active_ball_track_id is not None:
                track_id = self.active_ball_track_id
                involved_track_ids.add(track_id)
                self._consume_hitter_track_locked(track_id, reason)
            self._clear_ball_tracking_locked()
            return tuple(sorted(involved_track_ids))

    def _mark_latest_ball_snapshot_consumed_locked(
        self,
        track_id: int,
    ) -> None:
        snapshot = self.latest_ball_snapshot
        if (
            snapshot is None
            or snapshot.track_id != int(track_id)
            or snapshot.consumed
        ):
            return
        self.latest_ball_snapshot = BallEstimateSnapshot(
            track_id=snapshot.track_id,
            generation=snapshot.generation,
            source_frame=snapshot.source_frame,
            source_time_s=snapshot.source_time_s,
            received_monotonic_s=snapshot.received_monotonic_s,
            position_w=snapshot.position_w,
            velocity_w=snapshot.velocity_w,
            base_position_w=snapshot.base_position_w,
            base_quaternion_xyzw=snapshot.base_quaternion_xyzw,
            base_valid=snapshot.base_valid,
            visible=snapshot.visible,
            ready=snapshot.ready,
            consumed=True,
            new_track=snapshot.new_track,
        )

    def begin_hitter_policy_session(
        self,
        *,
        now_monotonic_s: float,
    ) -> bool:
        now = float(now_monotonic_s)
        if not np.isfinite(now):
            raise ValueError("now_monotonic_s must be finite")
        with self._hitter_ball_state_lock:
            self._apply_freshness_locked(now)
            if self._vicon_schema_fault_seen:
                return False
            if not self._stream_fresh_locked(now) or not self.base_pose_valid_tmp:
                return False

            recoverable = {
                ViconInputFault.VICON_STREAM_STALE,
                ViconInputFault.BALL_MESSAGE_STALE,
                ViconInputFault.TRACK_ID_CONFLICT,
            }
            if self.vicon_fault_latched in recoverable:
                self.vicon_fault_latched = None
            self._vicon_fault_event_emitted.difference_update(recoverable)
            recoverable_event_reasons = {
                ViconEventReason.TRACK_ENDED,
                ViconEventReason.BASE_POSE_INVALID,
                ViconEventReason.VICON_STREAM_STALE,
                ViconEventReason.BALL_MESSAGE_STALE,
                ViconEventReason.TRACK_ID_CONFLICT,
            }
            self._vicon_events = deque(
                event
                for event in self._vicon_events
                if event.reason not in recoverable_event_reasons
            )
            self._hitter_policy_session_open = True
            if self.active_ball_track_id is not None and self.ball_visible_tmp:
                if self._clear_pending_ball_admission_locked(
                    self.active_ball_track_id
                ):
                    logger.warning(
                        "HITTER Vicon pending-track admission: track_id={} "
                        "state=rejected admitted=false reason=policy_reentry",
                        self.active_ball_track_id,
                    )
                self.consumed_ball_track_ids.add(self.active_ball_track_id)
                self._mark_latest_ball_snapshot_consumed_locked(
                    self.active_ball_track_id
                )
                self._no_ball_since_monotonic_s = None
                self._clear_ball_tracking_locked()
            else:
                self._clear_pending_ball_admission_locked()
                self._no_ball_since_monotonic_s = now
            return True

    def _ball_estimator_timestamp_from_msg(self, msg, host_time: float) -> float:
        with self._hitter_ball_state_lock:
            vicon_time = float(getattr(msg, "vicon_time_s", 0.0) or 0.0)
            if np.isfinite(vicon_time) and vicon_time > 0.0:
                return vicon_time

            publish_time_us = int(getattr(msg, "publish_time_us", 0) or 0)
            if publish_time_us > 0:
                return float(publish_time_us) * 1.0e-6

            return self._next_ball_estimator_timestamp(host_time)

    def reset_heading(self):
        """Reset heading (yaw only) so that current root_quat becomes heading-zero for policy."""
        quat_xyzw = np.asarray(self.root_quat_tmp, dtype=np.float64).reshape(-1)[:4]
        try:
            yaw = float(sRot.from_quat(quat_xyzw).as_euler("xyz", degrees=False)[2])
        except Exception:
            yaw = 0.0
        self._heading_inv_rot = sRot.from_euler("z", -yaw, degrees=False)
        logger.info(f"[RealWorld] heading reset enabled, yaw0={yaw:.3f} rad")
    
    def _init_communication(self):
        self._poll_stop_event = threading.Event()
        self._communication_close_lock = threading.Lock()
        self._communication_closed = False
        self.firstReceiveAlarm = False
        self.firstReceiveOdometer = False
        self.firstReceiveVicon = False
        self._init_time = time.time()
        
        self.root_state_subscriber = self.lc.subscribe('state_estimator_data', self._root_state_handler)
        self.joint_state_subscriber = self.lc.subscribe('body_control_data', self._joint_state_handler)
        self.vicon_state_subscriber = self.lc.subscribe(
            self.vicon_consumer_settings.channel,
            self._vicon_state_handler,
        )
        self.remote_controller_subscriber = self.lc.subscribe('rc_command_data', self._remote_controller_handler)
        self.teleop_state_subscriber = self.lc.subscribe('camera_reference_data', self._teleop_state_handler)

    def _next_ball_estimator_timestamp(self, host_time):
        with self._hitter_ball_state_lock:
            if self.ball_state_estimator_time is None or self.ball_state_estimator_last_host_time is None:
                self.ball_state_estimator_time = 0.0
            else:
                wall_dt = max(0.0, float(host_time - self.ball_state_estimator_last_host_time))
                nominal_dt = 1.0 / self.ball_state_estimator_sample_rate_hz if self.ball_state_estimator_sample_rate_hz > 0.0 else wall_dt
                self.ball_state_estimator_time += nominal_dt
            self.ball_state_estimator_last_host_time = float(host_time)
            return self.ball_state_estimator_time

    def _root_state_handler(self, channel, data):
        msg = state_estimator_lcmt.decode(data)
        if not self.firstReceiveOdometer:
            self.firstReceiveOdometer = True
            logger.info('State Estimator Information Received!')
            logger.info(f'Root Translation: {np.array(msg.p)}, Root Linear Velocity: {np.array(msg.vBody)}')
        self.root_trans_tmp = np.array(msg.p)
        self.root_rpy_tmp = np.array(msg.rpy)
        self.root_quat_tmp = np.roll(np.array(msg.quat), -1) # (w,x,y,z) -> (x,y,z,w)
        self.base_lin_vel_tmp = np.array(msg.vBody)
        self.base_ang_vel_tmp = np.array(msg.omegaBody)

    def _vicon_state_handler(self, channel, data):
        received_monotonic_s = time.monotonic()
        try:
            if channel != self.vicon_consumer_settings.channel:
                raise ValueError(
                    f"unexpected Vicon channel {channel!r}; expected "
                    f"{self.vicon_consumer_settings.channel!r}"
                )
            msg = transformation_t.decode(data)
            self._ingest_vicon_v2_message(
                msg,
                received_monotonic_s=received_monotonic_s,
            )
        except Exception as exc:
            self._latch_vicon_fault(
                ViconInputFault.VICON_SCHEMA_ERROR,
                received_monotonic_s=received_monotonic_s,
                detail=f"{type(exc).__name__}: {exc}",
            )

    def _ingest_vicon_v2_message(
        self,
        msg,
        *,
        received_monotonic_s: float,
    ) -> None:
        received = float(received_monotonic_s)
        if not np.isfinite(received):
            raise ValueError("received_monotonic_s must be finite")
        try:
            (
                name,
                track_id,
                _source_frame,
                valid,
                pos,
                quat,
            ) = self._validate_vicon_v2_message(msg)
        except Exception as exc:
            self._latch_vicon_fault(
                ViconInputFault.VICON_SCHEMA_ERROR,
                received_monotonic_s=received,
                detail=f"{type(exc).__name__}: {exc}",
            )
            return

        if name == "ball":
            self._update_ball_state_from_vicon(
                msg,
                pos,
                received_monotonic_s=received,
            )
            return

        with self._hitter_ball_state_lock:
            self._last_any_vicon_monotonic_s = received
            if name == "table":
                return

            was_valid = bool(self.base_pose_valid_tmp)
            if not valid:
                self.base_pose_valid_tmp = False
                self.base_pose_valid = False
                pending_track_id = self._pending_ball_track_id
                if pending_track_id is not None:
                    self._reject_pending_ball_admission_locked(
                        pending_track_id,
                        reason="base_pose_invalid",
                    )
                if was_valid:
                    self._enqueue_vicon_event_locked(
                        ViconEventReason.BASE_POSE_INVALID,
                        track_id=None,
                        detail=(
                            f"{self.vicon_consumer_settings.base_subject} "
                            "pose became invalid"
                        ),
                    )
                return

            first_valid_pose = not self.firstReceiveVicon
            self.firstReceiveVicon = True
            self.root_trans_world_tmp = pos.astype(np.float32)
            self.root_quat_world_tmp = quat.astype(np.float32)
            self.base_pose_valid_tmp = True
        if first_valid_pose:
            logger.info('Vicon root information received!')
            logger.info(f'Root Translation World: {pos}')

    def _validate_vicon_v2_message(self, msg):
        name = getattr(msg, "name", None)
        if type(name) is not str:
            raise ValueError("Vicon subject name must be an exact string")

        track_id = getattr(msg, "track_id", None)
        if type(track_id) is not int:
            raise ValueError("Vicon track_id must be a non-bool integer")

        source_frame = getattr(msg, "vicon_frame_number", None)
        if type(source_frame) is not int or source_frame < 0:
            raise ValueError("Vicon source frame must be a non-negative integer")

        valid_flag = getattr(msg, "valid", None)
        occluded_flag = getattr(msg, "occluded", None)
        if (
            type(valid_flag) is not int
            or type(occluded_flag) is not int
            or (valid_flag, occluded_flag) not in {(1, 0), (0, 1)}
        ):
            raise ValueError(
                "Vicon valid/occluded flags must be complementary integer bits"
            )
        valid = valid_flag == 1

        position = np.asarray(
            getattr(msg, "pos_vicon", None),
            dtype=np.float64,
        )
        quaternion = np.asarray(
            getattr(msg, "quat_vicon", None),
            dtype=np.float64,
        )
        if position.shape != (3,) or not np.isfinite(position).all():
            raise ValueError("Vicon position must contain exactly three finite values")
        if (
            quaternion.shape != (4,)
            or not np.isfinite(quaternion).all()
            or float(np.linalg.norm(quaternion)) == 0.0
        ):
            raise ValueError(
                "Vicon quaternion must contain exactly four finite values "
                "with nonzero norm"
            )

        if name == "ball":
            if track_id <= 0:
                raise ValueError(
                    f"ball track_id must be positive, got {track_id}"
                )
        elif name == "table":
            if track_id != 0:
                raise ValueError("table track_id must be zero")
            if not valid:
                raise ValueError("table message must be valid")
        elif name == self.vicon_consumer_settings.base_subject:
            if track_id != 0:
                raise ValueError("base subject track_id must be zero")
        else:
            raise ValueError(
                f"unexpected Vicon subject {name!r}; expected "
                f"{self.vicon_consumer_settings.base_subject!r}"
            )

        return name, track_id, source_frame, valid, position, quaternion

    def _update_ball_state_from_vicon(
        self,
        msg,
        pos: np.ndarray,
        *,
        received_monotonic_s: float | None = None,
    ) -> None:
        received = (
            time.monotonic()
            if received_monotonic_s is None
            else float(received_monotonic_s)
        )
        track_id = int(getattr(msg, "track_id", 0) or 0)
        source_frame = int(getattr(msg, "vicon_frame_number", 0) or 0)
        valid = bool(getattr(msg, "valid", 1)) and not bool(
            getattr(msg, "occluded", 0)
        )
        notification = None
        bounce_detected = False
        first_ball_position = None
        conflict = False
        admission_diagnostic = None
        pending_admission_diagnostic = None
        with self._hitter_ball_state_lock:
            previous_any_vicon_monotonic_s = (
                self._last_any_vicon_monotonic_s
            )
            self._last_any_vicon_monotonic_s = received
            if not valid:
                if self.active_ball_track_id == track_id:
                    self._end_active_ball_locked(
                        now_monotonic_s=received,
                        event_reason=ViconEventReason.TRACK_ENDED,
                        detail="authoritative ball track ended",
                    )
                elif self.active_ball_track_id is not None:
                    self._latch_vicon_fault(
                        ViconInputFault.VICON_SCHEMA_ERROR,
                        received_monotonic_s=received,
                        detail=(
                            "invalid ball end ID does not match active ID: "
                            f"{track_id} != {self.active_ball_track_id}"
                        ),
                        track_id=track_id,
                    )
                elif not (
                    track_id == self.last_ball_track_id
                    and track_id in self.seen_ball_track_ids
                ):
                    self._latch_vicon_fault(
                        ViconInputFault.VICON_SCHEMA_ERROR,
                        received_monotonic_s=received,
                        detail=(
                            "invalid ball end has no matching active or "
                            f"last seen identity: {track_id}"
                        ),
                        track_id=track_id,
                    )
                return

            position = np.asarray(pos, dtype=np.float64).reshape(-1)[:3]
            if position.shape != (3,) or not np.isfinite(position).all():
                self._latch_vicon_fault(
                    ViconInputFault.VICON_SCHEMA_ERROR,
                    received_monotonic_s=received,
                    detail="ball position must contain three finite values",
                    track_id=track_id,
                )
                return

            old_active = self.active_ball_track_id
            no_ball_since_before_packet = self._no_ball_since_monotonic_s
            self._no_ball_since_monotonic_s = None
            conflict = old_active is not None and old_active != track_id
            if conflict:
                if self._pending_ball_track_id is not None:
                    self._reject_pending_ball_admission_locked(
                        self._pending_ball_track_id,
                        reason="track_id_conflict",
                    )
                self._clear_ball_tracking_locked()
                self.consumed_ball_track_ids.update(
                    (old_active, track_id)
                )
                self._mark_latest_ball_snapshot_consumed_locked(old_active)
                previous_challenger_frame = self.ball_track_last_frames.get(
                    track_id
                )
                self.seen_ball_track_ids.add(track_id)
                if (
                    previous_challenger_frame is None
                    or source_frame > previous_challenger_frame
                ):
                    self.ball_track_last_frames[track_id] = source_frame
                self._latch_vicon_fault(
                    ViconInputFault.TRACK_ID_CONFLICT,
                    received_monotonic_s=received,
                    detail=(
                        f"overlapping ball track IDs {old_active} "
                        f"and {track_id}"
                    ),
                    track_id=track_id,
                )
                return

            is_new_track = track_id not in self.seen_ball_track_ids
            if not is_new_track:
                last_frame = self.ball_track_last_frames[track_id]
                if source_frame <= last_frame:
                    return

            if is_new_track:
                self._clear_ball_tracking_locked()
                stream_was_fresh = bool(
                    previous_any_vicon_monotonic_s is not None
                    and received - previous_any_vicon_monotonic_s
                    <= self.vicon_consumer_settings.stream_timeout_s
                )
                no_ball_gap_s = (
                    None
                    if no_ball_since_before_packet is None
                    else max(0.0, received - no_ball_since_before_packet)
                )
                no_ball_long_enough = bool(
                    no_ball_gap_s is not None
                    and no_ball_gap_s
                    >= self.vicon_consumer_settings.new_serve_no_ball_s
                )
                admitted = bool(
                    not conflict
                    and old_active is None
                    and self._hitter_policy_session_open
                    and self.vicon_fault_latched is None
                    and stream_was_fresh
                    and self.base_pose_valid_tmp
                    and no_ball_long_enough
                )
                rejection_reasons = []
                if old_active is not None:
                    rejection_reasons.append("active_track_present")
                if not self._hitter_policy_session_open:
                    rejection_reasons.append("policy_session_closed")
                if self.vicon_fault_latched is not None:
                    rejection_reasons.append("vicon_fault_latched")
                if not stream_was_fresh:
                    rejection_reasons.append("stream_not_fresh")
                if not self.base_pose_valid_tmp:
                    rejection_reasons.append("base_pose_invalid")
                if no_ball_gap_s is None:
                    rejection_reasons.append("no_ball_timer_unset")
                elif not no_ball_long_enough:
                    rejection_reasons.append("no_ball_gap_too_short")
                pending = rejection_reasons == ["no_ball_gap_too_short"]
                self.seen_ball_track_ids.add(track_id)
                if admitted:
                    self.admitted_ball_track_ids.add(track_id)
                elif pending:
                    self._pending_ball_track_id = track_id
                    self._pending_ball_admit_after_monotonic_s = (
                        received
                        + self.vicon_consumer_settings.new_serve_no_ball_s
                        - no_ball_gap_s
                    )
                else:
                    self.consumed_ball_track_ids.add(track_id)
                admission_diagnostic = (
                    track_id,
                    admitted,
                    pending,
                    no_ball_gap_s,
                    self.vicon_consumer_settings.new_serve_no_ball_s,
                    self._hitter_policy_session_open,
                    self.vicon_fault_latched,
                    stream_was_fresh,
                    self.base_pose_valid_tmp,
                    tuple(rejection_reasons),
                )
                if conflict:
                    self.consumed_ball_track_ids.add(old_active)
            elif old_active != track_id:
                self._clear_pending_ball_admission_locked(track_id)
                self._clear_ball_tracking_locked()
                self.consumed_ball_track_ids.add(track_id)

            pending = self._pending_ball_track_id == track_id
            promoted = False
            if pending:
                pending_deadline = self._pending_ball_admit_after_monotonic_s
                hard_rejection_reasons = []
                if not self._hitter_policy_session_open:
                    hard_rejection_reasons.append("policy_session_closed")
                if self.vicon_fault_latched is not None:
                    hard_rejection_reasons.append("vicon_fault_latched")
                if not self._stream_fresh_locked(received):
                    hard_rejection_reasons.append("stream_not_fresh")
                if not self.base_pose_valid_tmp:
                    hard_rejection_reasons.append("base_pose_invalid")
                if hard_rejection_reasons:
                    self._reject_pending_ball_admission_locked(
                        track_id,
                        reason=",".join(hard_rejection_reasons),
                    )
                    pending = False
                elif (
                    pending_deadline is not None
                    and received >= pending_deadline
                ):
                    self._clear_pending_ball_admission_locked(track_id)
                    self.admitted_ball_track_ids.add(track_id)
                    pending = False
                    promoted = True
                    pending_admission_diagnostic = (
                        track_id,
                        pending_deadline,
                        received,
                    )

            self.ball_track_generations[track_id] = (
                self.ball_track_generations.get(track_id, 0) + 1
            )
            self.ball_track_last_frames[track_id] = source_frame
            self.ball_snapshot_generation = self.ball_track_generations[track_id]
            self.ball_track_id = track_id
            self.active_ball_track_id = track_id
            self.last_ball_track_id = track_id
            self.ball_visible_tmp = True
            self._last_ball_vicon_monotonic_s = received

            consumed = track_id in self.consumed_ball_track_ids

            if not consumed:
                sample_time = self._ball_estimator_timestamp_from_msg(
                    msg,
                    received,
                )
                estimate = self.ball_state_estimator.add_sample(
                    position,
                    timestamp=sample_time,
                )
                self.ball_state_estimator_last_host_time = received
                self.ball_pos_world_tmp = estimate.position.astype(np.float32)
                self.ball_vel_world_tmp = estimate.velocity.astype(np.float32)
                self.ball_state_estimator_ready_tmp = bool(estimate.valid)
                self.ball_state_estimator_sample_count_tmp = int(
                    estimate.sample_count
                )
                bounce_detected = bool(estimate.bounce_detected)
                if not self.firstReceiveBall:
                    self.firstReceiveBall = True
                    first_ball_position = self.ball_pos_world_tmp.copy()
            else:
                self.ball_pos_world_tmp = position.astype(np.float32)
                self.ball_vel_world_tmp = np.zeros(3, dtype=np.float32)

            notification = self._prepare_ball_snapshot_locked(
                msg,
                received_monotonic_s=received,
                consumed=consumed,
                new_track=is_new_track or promoted,
            )
            if pending:
                notification = None

        if admission_diagnostic is not None:
            (
                admission_track_id,
                admission_admitted,
                admission_pending,
                admission_no_ball_gap_s,
                admission_required_no_ball_s,
                admission_policy_session_open,
                admission_latched_fault,
                admission_stream_fresh,
                admission_base_pose_valid,
                admission_rejection_reasons,
            ) = admission_diagnostic
            admission_state = (
                "admitted"
                if admission_admitted
                else ("pending" if admission_pending else "rejected")
            )
            admission_log = (
                logger.info
                if admission_admitted or admission_pending
                else logger.warning
            )
            admission_log(
                "HITTER Vicon new-track admission: track_id={} state={} "
                "admitted={} pending={} "
                "no_ball_gap_s={} required_no_ball_s={:.3f} "
                "policy_session_open={} latched_fault={} stream_fresh={} "
                "base_pose_valid={} rejection_reasons={}",
                admission_track_id,
                admission_state,
                str(admission_admitted).lower(),
                str(admission_pending).lower(),
                (
                    "none"
                    if admission_no_ball_gap_s is None
                    else f"{admission_no_ball_gap_s:.3f}"
                ),
                admission_required_no_ball_s,
                str(admission_policy_session_open).lower(),
                (
                    "none"
                    if admission_latched_fault is None
                    else admission_latched_fault.value
                ),
                str(admission_stream_fresh).lower(),
                str(admission_base_pose_valid).lower(),
                (
                    "none"
                    if not admission_rejection_reasons
                    else ",".join(admission_rejection_reasons)
                ),
            )
        if pending_admission_diagnostic is not None:
            (
                pending_track_id,
                pending_deadline,
                promoted_at,
            ) = pending_admission_diagnostic
            logger.info(
                "HITTER Vicon pending-track admission: track_id={} "
                "state=promoted admitted=true deadline_monotonic_s={:.6f} "
                "promoted_at_monotonic_s={:.6f}",
                pending_track_id,
                pending_deadline,
                promoted_at,
            )
        if notification is not None:
            self._notify_ball_snapshot_listeners(*notification)
        if bounce_detected:
            logger.info(
                "[RealWorld] Vicon ball table bounce detected; cleared polynomial fit buffer."
            )
        if first_ball_position is not None:
            logger.info('Vicon ball information received!')
            logger.info(f'Ball Position World: {first_ball_position}')

    def _joint_state_handler(self, channel, data):
        msg = body_control_data_lcmt.decode(data)
        if not self.firstReceiveAlarm:
            self.time_delay = time.time() - self._init_time
            self.firstReceiveAlarm = True
            logger.info("Communication build successfully between the policy and the transition layer!")
            logger.info(f'First signal arrives after {self.time_delay}s!')
        self.dof_pos_tmp = np.array(msg.q)[self.idx_r2p]
        self.dof_vel_tmp = np.array(msg.qd)[self.idx_r2p]
    
    def _remote_controller_handler(self, channel, data):
        msg = rc_command_lcmt.decode(data)
        
        self.left_upper_switch_pressed = ((msg.left_upper_switch and not self.left_upper_switch) or self.left_upper_switch_pressed)
        self.left_lower_left_switch_pressed = ((msg.left_lower_left_switch and not self.left_lower_left_switch) or self.left_lower_left_switch_pressed)
        self.left_lower_right_switch_pressed = ((msg.left_lower_right_switch and not self.left_lower_right_switch) or self.left_lower_right_switch_pressed)
        self.right_upper_switch_pressed = ((msg.right_upper_switch and not self.right_upper_switch) or self.right_upper_switch_pressed)
        self.right_lower_left_switch_pressed = ((msg.right_lower_left_switch and not self.right_lower_left_switch) or self.right_lower_left_switch_pressed)
        self.right_lower_right_switch_pressed = ((msg.right_lower_right_switch) and not self.right_lower_right_switch) or self.right_lower_right_switch_pressed

        self.mode = msg.mode
        self.right_stick = msg.right_stick
        self.left_stick = msg.left_stick
        self.left_upper_switch = msg.left_upper_switch
        self.left_lower_left_switch = msg.left_lower_left_switch
        self.left_lower_right_switch = msg.left_lower_right_switch
        self.right_upper_switch = msg.right_upper_switch
        self.right_lower_left_switch = msg.right_lower_left_switch
        self.right_lower_right_switch = msg.right_lower_right_switch

    def check_teleop_sync(self):
        if self.right_upper_switch_pressed:
            self.sync = not self.sync
            self.right_upper_switch_pressed = False
            print(f"==================== right_upper_switch_pressed ===================")
            print(f" {self.sync} ")

    def get_state(self):
        self.root_trans = self.root_trans_tmp.copy()
        self.base_lin_vel = self.base_lin_vel_tmp.copy()
        self.root_quat = self.root_quat_tmp.copy()
        with self._hitter_ball_state_lock:
            self.root_trans_world = self.root_trans_world_tmp.copy()
            self.root_quat_world = self.root_quat_world_tmp.copy()
            self.ball_pos_world = self.ball_pos_world_tmp.copy()
            self.ball_vel_world = self.ball_vel_world_tmp.copy()
            self.ball_visible = bool(self.ball_visible_tmp)
            self.ball_state_estimator_ready = bool(self.ball_state_estimator_ready_tmp)
            self.ball_state_estimator_sample_count = int(self.ball_state_estimator_sample_count_tmp)
            self.base_pose_valid = bool(self.base_pose_valid_tmp)
        self.root_rpy = self.root_rpy_tmp.copy()
        self.base_ang_vel = self.base_ang_vel_tmp.copy()

        # Apply yaw-only heading reset for policy, if enabled.
        if self._heading_inv_rot is not None:
            try:
                q = self._heading_inv_rot * sRot.from_quat(np.asarray(self.root_quat, dtype=np.float64).reshape(-1)[:4])
                self.root_quat = q.as_quat().astype(np.float32)
                # Keep rpy consistent with the adjusted quaternion
                self.root_rpy = sRot.from_quat(self.root_quat).as_euler("xyz", degrees=False).astype(np.float32)
            except Exception as exc:
                logger.warning(f"[RealWorld] failed to apply heading reset: {exc}")

        self.projected_gravity = get_gravity(self.root_quat, w_last=True)
        self.dof_pos = self.dof_pos_tmp.copy()[self.active_dof_idx]
        self.dof_vel = self.dof_vel_tmp.copy()[self.active_dof_idx]

        if self.cfg.control.update_with_fk:
            fk_info, fk_info_tensor = self.fk()
            self.torso_quat = fk_info[self.torso_name]['quat']
            self.torso_trans = fk_info[self.torso_name]['pos']
            self.robot_fk_info = fk_info_tensor

        if hasattr(self.cfg.control, 'use_teleop'):
            if self._init_teleop:
                if self.cfg.control.use_teleop:
                    self.check_teleop_sync()
                    if self.sync:
                        idx_r2p = [
                            0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10, 16, 23, 5, 11, 17, 24, 18, 25, 19, 26, 20, 27, 21, 28
                        ]
                        self.teleop_dof_pos = self.teleop_dof_pos_tmp.copy()[idx_r2p]
                        self.teleop_quat = self.align_quat(self.teleop_quat_tmp.copy())
                        if self.cfg.control.update_with_fk:
                            fk_info, _ = self.fk_teleop()
                            self.teleop_quat = fk_info[self.torso_name]['quat']

    def apply_action(self, action):
        self.act = action.copy()
        action = action.squeeze(0)
        is_mosaic = getattr(self.cfg.control, 'is_mosaic', False)
        if is_mosaic:
            dof_target_pos = action.copy()
        else:
            # Update default joint angles from reference motion if enabled and available
            current_default_angles = self.default_angles[self.active_dof_idx].copy()
            
            # Check if residual control is enabled in config
            use_residual = getattr(self.cfg.control, 'use_residual', False)
            
            if use_residual and self.ref_dof_pos is not None:
                # Get residual joint indices from config
                if hasattr(self.cfg.control, 'residual_joint_indices') and self.cfg.control.residual_joint_indices is not None:
                    residual_joint_indices = self.cfg.control.residual_joint_indices
                    # Flatten ref_dof_pos if it's 2D
                    ref_dof_pos_flat = self.ref_dof_pos.flatten() if self.ref_dof_pos.ndim > 1 else self.ref_dof_pos
                    for idx in residual_joint_indices:
                        if idx < len(ref_dof_pos_flat) and idx < len(current_default_angles):
                            current_default_angles[idx] = ref_dof_pos_flat[idx]
            
            tgt_dof_pos = current_default_angles + action*self.cfg.control.action_scale
            self.target_dof_pos = tgt_dof_pos  # Store target positions for monitoring
            
            dof_target_pos = self.default_angles.copy()
            dof_target_pos[self.active_dof_idx] = tgt_dof_pos
            dof_target_pos = dof_target_pos[self.idx_p2r]

        cmd = pd_tau_targets_lcmt()
        cmd.q_des = dof_target_pos.copy()
        cmd.qd_des = np.zeros_like(dof_target_pos)
        cmd.kp = self.kps.copy()[self.idx_p2r]
        cmd.kd = self.kds.copy()[self.idx_p2r]
        cmd.tau_ff = np.zeros_like(dof_target_pos)
        cmd.se_contactState = np.zeros(2)
        cmd.timestamp_us = int(time.time()*10**6)
        
        self.lc.publish("pd_plustau_targets", cmd.encode())

    def _calibration_action_from_residual(self, residual, default_pos):
        if getattr(self.cfg.control, 'is_mosaic', False):
            return default_pos + residual
        return residual / self.cfg.control.action_scale

    def _policy_start_base_pose_ready(self, *, now: float | None = None) -> bool:
        if bool(getattr(self, "base_pose_valid_tmp", False)):
            return True
        if now is None:
            now = time.monotonic()
        last_log = getattr(
            self,
            "_last_base_pose_wait_log_monotonic_s",
            None,
        )
        if last_log is None or float(now) - last_log >= 1.0:
            logger.warning(
                "[RealWorld] waiting for a valid {} world pose before "
                "entering the policy loop".format(
                    self.vicon_consumer_settings.base_subject
                )
            )
            self._last_base_pose_wait_log_monotonic_s = float(now)
        return False

    def poll(self, cb=None):
        try:
            while not self._poll_stop_event.is_set():
                timeout = 0.01
                rfds, wfds, efds = select.select([self.lc.fileno()], [], [], timeout)
                if rfds:
                    self.lc.handle()
                else:
                    continue
        except KeyboardInterrupt:
            pass

    def spin(self):
        self._poll_stop_event.clear()
        self.run_thread = threading.Thread(target=self.poll, daemon=True)
        self.run_thread.start()

    def close(self) -> bool:
        with self._communication_close_lock:
            if self._communication_closed:
                return True
            self._poll_stop_event.set()
            run_thread = getattr(self, "run_thread", None)
            if (
                run_thread is not None
                and run_thread is not threading.current_thread()
                and run_thread.is_alive()
            ):
                run_thread.join(timeout=1.0)
                if run_thread.is_alive():
                    logger.warning(
                        "[RealWorld] LCM poll thread did not stop within 1.0 s"
                    )
                    return False
            for subscription in (
                self.root_state_subscriber,
                self.joint_state_subscriber,
                self.vicon_state_subscriber,
                self.remote_controller_subscriber,
                self.teleop_state_subscriber,
            ):
                self.lc.unsubscribe(subscription)
            self._communication_closed = True
            return True

    def connected(self):
        return self.firstReceiveAlarm

    def _wait_for_right_lower_right_switch_press(self):
        while True:
            if (
                getattr(self, 'right_lower_right_switch_pressed', False)
                or getattr(self, 'right_lower_right_switch', False)
            ):
                self.right_lower_right_switch_pressed = False
                break
            time.sleep(0.002)

        while getattr(self, 'right_lower_right_switch', False):
            time.sleep(0.002)
    
    def calibrate(self, refresh, init_ref_dof_pos=None):
        self.get_state()
        if refresh:
            # Handle init_ref_dof_pos if provided
            current_default_angles = self.default_angles.copy()[self.active_dof_idx]
            if init_ref_dof_pos is not None:
                use_residual = getattr(self.cfg.control, 'use_residual', False)    
                if use_residual and init_ref_dof_pos is not None:
                    # Get residual joint indices from config
                    if hasattr(self.cfg.control, 'residual_joint_indices') and self.cfg.control.residual_joint_indices is not None:
                        residual_joint_indices = self.cfg.control.residual_joint_indices
                        # Flatten init_ref_dof_pos if it's 2D
                        ref_dof_pos_flat = init_ref_dof_pos.flatten() if init_ref_dof_pos.ndim > 1 else init_ref_dof_pos
                        for idx in residual_joint_indices:
                            if idx < len(ref_dof_pos_flat) and idx < len(current_default_angles):
                                current_default_angles[idx] = ref_dof_pos_flat[idx]
            
            logger.info('Calibraiting..., Press R2 to continue')
            self._wait_for_right_lower_right_switch_press()
            logger.info('R2 button pressed, Start Calibrating...')

            cur_dof_pos = self.dof_pos
            final_goal = np.zeros_like(current_default_angles)
            default_pos = current_default_angles.copy()
            target = cur_dof_pos - default_pos
            target_seq = []
            while np.max(np.abs(target-final_goal))>0.01:
                target-=np.clip((target-final_goal), -0.05, 0.05)
                target_seq += [copy.deepcopy(target)]
            for tgt in target_seq:
                step_start=time.time()
                next_tgt = self._calibration_action_from_residual(tgt, default_pos)
                self.apply_action(next_tgt[None,])
                self.get_state()
                time_till_next_step = self.high_dt - (time.time()-step_start)
                if time_till_next_step>0:
                    time.sleep(time_till_next_step)
            logger.info('Calibration Done. Press R2 to continue')
            while True:
                self._wait_for_right_lower_right_switch_press()
                if not self._policy_start_base_pose_ready():
                    continue
                logger.info('R2 pressed again, Communication built between policy layer and transition layer!')
                # Reset heading at policy start so "current facing" becomes zero yaw.
                if self._reset_heading_on_start:
                    # Use latest state before resetting heading
                    self.get_state()
                    self.reset_heading()
                    self.reset_teleop()
                break
                
        else:
            raise NotImplementedError
            


    def check_termination(self):
        if not bool(getattr(self, "terminate_on_r2", getattr(self.cfg.control, "terminate_on_r2", True))):
            return False
        return self.right_lower_right_switch_pressed
        # return abs(self.root_rpy_tmp[0])>0.8 or abs(self.root_rpy_tmp[1])>0.8 or self.right_lower_right_switch_pressed
