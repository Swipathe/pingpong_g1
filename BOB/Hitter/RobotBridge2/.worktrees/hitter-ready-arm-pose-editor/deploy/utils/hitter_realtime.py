from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Callable

import numpy as np


def _copied_read_only_array(value) -> np.ndarray:
    array = np.asarray(value).copy()
    array.setflags(write=False)
    return array


@dataclass(frozen=True)
class BallEstimateSnapshot:
    track_epoch: int
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
    track_epoch: int
    source_generation: int
    source_frame: int
    strike_deadline_monotonic_s: float
    completed_monotonic_s: float
    command: object | None
    error: str | None = None


@dataclass(frozen=True)
class PlannerWorkerStats:
    submitted: int
    completed: int
    failed: int
    dropped_pending: int


class IncomingTrackConfirmation:
    """Confirm stable fitted incoming motion once per ball track epoch."""

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
        self.reset()

    def reset(self) -> None:
        self._track_epoch = None
        self._consecutive_count = 0
        self._confirmed = False

    def observe(self, *, track_epoch: int, velocity_x_mps: float) -> bool:
        epoch = int(track_epoch)
        if self._track_epoch != epoch:
            self._track_epoch = epoch
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


class LatestOnlyPlannerWorker:
    """Run a planner off-thread while retaining at most one pending snapshot."""

    def __init__(
        self,
        plan_fn: Callable[[BallEstimateSnapshot], object],
        *,
        monotonic_fn: Callable[[], float] = time.monotonic,
    ):
        self._plan_fn = plan_fn
        self._monotonic_fn = monotonic_fn
        self._condition = threading.Condition()
        self._pending: BallEstimateSnapshot | None = None
        self._latest_result: PlannerResultSnapshot | None = None
        self._stop_requested = False
        self._submitted = 0
        self._completed = 0
        self._failed = 0
        self._dropped_pending = 0
        self._max_pending_depth = 0
        self._thread = threading.Thread(
            target=self._run,
            name=f"LatestOnlyPlannerWorker-{id(self):x}",
        )
        self._thread.start()

    @property
    def stats(self) -> PlannerWorkerStats:
        with self._condition:
            return PlannerWorkerStats(
                submitted=self._submitted,
                completed=self._completed,
                failed=self._failed,
                dropped_pending=self._dropped_pending,
            )

    @property
    def max_pending_depth(self) -> int:
        with self._condition:
            return self._max_pending_depth

    def submit(self, snapshot: BallEstimateSnapshot) -> None:
        with self._condition:
            if self._stop_requested:
                raise RuntimeError("planner worker is closed")
            self._submitted += 1
            if self._pending is not None:
                self._dropped_pending += 1
            self._pending = snapshot
            self._max_pending_depth = max(self._max_pending_depth, 1)
            self._condition.notify()

    def latest_result(self) -> PlannerResultSnapshot | None:
        with self._condition:
            return self._latest_result

    def close(self) -> None:
        with self._condition:
            self._stop_requested = True
            self._pending = None
            self._condition.notify_all()
        if threading.current_thread() is not self._thread:
            self._thread.join()

    def _run(self) -> None:
        while True:
            with self._condition:
                while self._pending is None and not self._stop_requested:
                    self._condition.wait()
                if self._stop_requested:
                    return
                snapshot = self._pending
                self._pending = None

            failed = False
            try:
                command = self._plan_fn(snapshot)
                deadline = snapshot.received_monotonic_s + float(
                    command.time_to_strike
                )
                planned = PlannerResultSnapshot(
                    track_epoch=snapshot.track_epoch,
                    source_generation=snapshot.generation,
                    source_frame=snapshot.source_frame,
                    strike_deadline_monotonic_s=deadline,
                    completed_monotonic_s=float(self._monotonic_fn()),
                    command=command,
                )
            except Exception as exc:
                failed = True
                planned = PlannerResultSnapshot(
                    track_epoch=snapshot.track_epoch,
                    source_generation=snapshot.generation,
                    source_frame=snapshot.source_frame,
                    strike_deadline_monotonic_s=float("nan"),
                    completed_monotonic_s=float(self._monotonic_fn()),
                    command=None,
                    error=f"{type(exc).__name__}: {exc}",
                )

            with self._condition:
                self._latest_result = planned
                if failed:
                    self._failed += 1
                else:
                    self._completed += 1
                self._condition.notify_all()


class CommandPhase(str, Enum):
    WAITING = "waiting"
    TRACKING = "tracking"
    ARMED = "armed"
    RECOVERY = "recovery"


class HitterCommandLifecycle:
    """Pure state machine that binds planner results to one ball track epoch."""

    _EPSILON = 1.0e-12

    def __init__(
        self,
        *,
        waiting_tts: float = 0.92,
        arm_tts: float = 0.90,
        minimum_arm_tts: float = 0.80,
        maximum_policy_tts: float = 0.92,
        swing_duration_sampler: Callable[[], float] = lambda: 1.85,
    ):
        if not 0.0 <= minimum_arm_tts <= arm_tts <= maximum_policy_tts:
            raise ValueError(
                "expected 0 <= minimum_arm_tts <= arm_tts <= maximum_policy_tts"
            )
        self.waiting_tts = float(waiting_tts)
        self.arm_tts = float(arm_tts)
        self.minimum_arm_tts = float(minimum_arm_tts)
        self.maximum_policy_tts = float(maximum_policy_tts)
        self.swing_duration_sampler = swing_duration_sampler

        self.phase = CommandPhase.WAITING
        self.active_result: PlannerResultSnapshot | None = None
        self.cached_result: PlannerResultSnapshot | None = None
        self.recovery_duration_s = 0.0
        self.command_end_deadline_s: float | None = None

        self._current_epoch: int | None = None
        self._latest_generation: int | None = None
        self._skipped_epochs: set[int] = set()
        self._ended_epochs: set[int] = set()

    def ingest(self, result: PlannerResultSnapshot, *, now: float) -> str:
        now = float(now)
        self.advance(now)
        if result.command is None or result.error is not None:
            return "ignored"
        if not np.isfinite(result.strike_deadline_monotonic_s):
            return "ignored"

        if self.phase == CommandPhase.RECOVERY:
            return self._cache_recovery_result(result)
        if self.phase == CommandPhase.ARMED:
            return self._ingest_armed(result, now)
        return self._ingest_unarmed(result, now)

    def mark_track_ended(self, track_epoch: int | None = None) -> None:
        if track_epoch is None:
            if (
                self.phase == CommandPhase.RECOVERY
                and self.cached_result is not None
            ):
                track_epoch = self.cached_result.track_epoch
            elif self.active_result is not None:
                track_epoch = self.active_result.track_epoch
            else:
                track_epoch = self._current_epoch
        if track_epoch is None:
            return
        ended_epoch = int(track_epoch)
        self._ended_epochs.add(ended_epoch)
        if (
            self.cached_result is not None
            and self.cached_result.track_epoch == ended_epoch
        ):
            self.cached_result = None
        if (
            self.phase == CommandPhase.TRACKING
            and self._current_epoch == ended_epoch
        ):
            self.phase = CommandPhase.WAITING

    def advance(self, now: float) -> CommandPhase:
        now = float(now)
        if (
            self.phase == CommandPhase.ARMED
            and self.active_result is not None
            and now + self._EPSILON
            >= self.active_result.strike_deadline_monotonic_s
        ):
            self.phase = CommandPhase.RECOVERY

        if (
            self.phase == CommandPhase.RECOVERY
            and self.command_end_deadline_s is not None
            and now + self._EPSILON >= self.command_end_deadline_s
        ):
            previous = self.active_result
            cached = self.cached_result
            if previous is not None:
                self._ended_epochs.add(previous.track_epoch)
            self.active_result = None
            self.cached_result = None
            self.command_end_deadline_s = None
            self.phase = CommandPhase.WAITING
            if cached is not None:
                self._ingest_unarmed(cached, now)
        return self.phase

    def policy_tts(self, *, now: float) -> float:
        if self.phase in (CommandPhase.WAITING, CommandPhase.TRACKING):
            return self.waiting_tts
        if self.active_result is None:
            return self.waiting_tts
        remaining = self.active_result.strike_deadline_monotonic_s - float(now)
        return float(np.clip(remaining, 0.0, self.maximum_policy_tts))

    def _ingest_unarmed(
        self,
        result: PlannerResultSnapshot,
        now: float,
    ) -> str:
        epoch = int(result.track_epoch)
        generation = int(result.source_generation)
        if epoch in self._skipped_epochs or epoch in self._ended_epochs:
            return "ignored"
        if self._current_epoch is not None:
            if epoch < self._current_epoch:
                return "ignored"
            if epoch == self._current_epoch:
                if (
                    self._latest_generation is not None
                    and generation <= self._latest_generation
                ):
                    return "ignored"
            else:
                self._current_epoch = epoch
                self._latest_generation = None
        else:
            self._current_epoch = epoch

        self._latest_generation = generation
        remaining = result.strike_deadline_monotonic_s - now
        if remaining < self.minimum_arm_tts - self._EPSILON:
            self._skipped_epochs.add(epoch)
            self.phase = CommandPhase.WAITING
            return "skipped"
        if remaining <= self.arm_tts + self._EPSILON:
            self._arm(result, now)
            return "armed"
        self.phase = CommandPhase.TRACKING
        return "tracking"

    def _ingest_armed(
        self,
        result: PlannerResultSnapshot,
        now: float,
    ) -> str:
        active = self.active_result
        if active is None:
            return "ignored"
        if result.track_epoch != active.track_epoch:
            return "ignored"
        if result.track_epoch in self._ended_epochs:
            return "ignored"
        latest_generation = (
            -1 if self._latest_generation is None else self._latest_generation
        )
        if result.source_generation <= latest_generation:
            return "ignored"

        self._latest_generation = int(result.source_generation)
        remaining = result.strike_deadline_monotonic_s - now
        if (
            remaining < -self._EPSILON
            or remaining > self.maximum_policy_tts + self._EPSILON
        ):
            return "retained"
        self.active_result = result
        self.command_end_deadline_s = (
            result.strike_deadline_monotonic_s + self.recovery_duration_s
        )
        return "overridden"

    def _cache_recovery_result(self, result: PlannerResultSnapshot) -> str:
        if self.active_result is None:
            return "ignored"
        if result.track_epoch <= self.active_result.track_epoch:
            return "ignored"
        if result.track_epoch in self._skipped_epochs or result.track_epoch in self._ended_epochs:
            return "ignored"
        cached = self.cached_result
        if cached is not None:
            if result.track_epoch < cached.track_epoch:
                return "ignored"
            if (
                result.track_epoch == cached.track_epoch
                and result.source_generation <= cached.source_generation
            ):
                return "ignored"
        self.cached_result = result
        return "cached"

    def _arm(self, result: PlannerResultSnapshot, now: float) -> None:
        arm_tts = max(result.strike_deadline_monotonic_s - now, 0.0)
        total_swing_duration = float(self.swing_duration_sampler())
        self.recovery_duration_s = max(total_swing_duration - arm_tts, 0.0)
        self.command_end_deadline_s = (
            result.strike_deadline_monotonic_s + self.recovery_duration_s
        )
        self.active_result = result
        self.cached_result = None
        self.phase = CommandPhase.ARMED


__all__ = [
    "BallEstimateSnapshot",
    "CommandPhase",
    "HitterCommandLifecycle",
    "LatestOnlyPlannerWorker",
    "PlannerResultSnapshot",
    "PlannerWorkerStats",
]
