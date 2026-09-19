from __future__ import annotations

import csv
import dataclasses
import json
import os
import time
import unittest
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import List, Optional
from unittest.mock import patch

import numpy as np

from simulator import real_world as real_world_module
from simulator.real_world import RealWorld
from utils.hitter_realtime import (
    BallEstimateSnapshot,
    CommandPhase,
    HitterCommandLifecycle,
    LatestOnlyPlannerWorker,
    PlannerResultSnapshot,
)


RECORDED = Path(
    os.environ.get("CHINGMU_REPLAY_CSV", "/tmp/chingmu_bounce_confirm3.csv")
)
POLICY_DT_S = 0.02
POLICY_RESULT_CONSUME_DELAY_S = 0.001
TRACK_ENDED_RESULT_ERROR = "RuntimeError: HITTER_TRACK_ENDED"


@dataclass(frozen=True)
class ReplayCommand:
    time_to_strike: float


@dataclass
class ReplayStats:
    frame_count: int
    frame_gaps: int
    bounce_count: int
    ready_transitions: int
    ready_reset_count: int
    first_ready_frame: Optional[int]
    worker_submitted: int
    worker_completed: int
    worker_failed: int
    worker_dropped_pending: int
    worker_max_pending_depth: int
    arm_band_candidates: int
    first_arm_tts: Optional[float]
    waiting_policy_tts: float
    max_armed_policy_tts: float
    generations_monotonic: bool
    track_end_events: int
    estimator_samples_after_track_end: int
    visible_after_track_end: bool
    max_policy_result_age_s: float
    active_after_track_end: bool
    active_until_command_end: bool
    phase_after_command_end: Optional[str]
    active_after_command_end: Optional[bool]
    old_epoch_rearm_decision: Optional[str]
    phase_after_old_epoch_rearm: Optional[str]
    active_after_old_epoch_rearm: Optional[bool]
    crossed_hit_plane: bool


class ReplayClock:
    def __init__(self) -> None:
        self.value = 0.0

    def __call__(self) -> float:
        return float(self.value)


class PolicyReplayConsumer:
    def __init__(self) -> None:
        self.lifecycle = HitterCommandLifecycle(
            waiting_tts=1.0,
            arm_tts=0.90,
            minimum_arm_tts=0.80,
            maximum_policy_tts=0.92,
            swing_duration_sampler=lambda: 1.85,
        )
        self.last_result_key = None
        self.consumed_results: List[PlannerResultSnapshot] = []
        self.arm_band_candidates = 0
        self.first_arm_tts: Optional[float] = None
        self.max_armed_policy_tts = 0.0
        self.max_policy_result_age_s = 0.0
        self.track_end_events = 0
        self.waiting_policy_tts = self.lifecycle.policy_tts(now=0.0)

    def consume(self, result: PlannerResultSnapshot, *, now: float) -> str:
        self.lifecycle.advance(now)
        key = (int(result.track_epoch), int(result.source_generation))
        if key == self.last_result_key:
            return "duplicate"
        self.last_result_key = key
        self.consumed_results.append(result)
        self.max_policy_result_age_s = max(
            self.max_policy_result_age_s,
            max(float(now) - float(result.completed_monotonic_s), 0.0),
        )

        if result.command is None or result.error is not None:
            if result.error == TRACK_ENDED_RESULT_ERROR:
                active = self.lifecycle.active_result
                ended_epoch = (
                    int(active.track_epoch)
                    if active is not None
                    else int(result.track_epoch)
                )
                self.lifecycle.mark_track_ended(ended_epoch)
                self.track_end_events += 1
                return "track-ended"
            return "failed"

        remaining = float(result.strike_deadline_monotonic_s) - float(now)
        if 0.80 <= remaining <= 0.90:
            self.arm_band_candidates += 1
        decision = self.lifecycle.ingest(result, now=now)
        if decision == "armed" and self.first_arm_tts is None:
            self.first_arm_tts = self.lifecycle.policy_tts(now=now)
        if (
            self.lifecycle.active_result is not None
            and self.lifecycle.phase in {CommandPhase.ARMED, CommandPhase.RECOVERY}
        ):
            self.max_armed_policy_tts = max(
                self.max_armed_policy_tts,
                self.lifecycle.policy_tts(now=now),
            )
        return decision


def make_ball_message(
    *,
    frame: int,
    source_time: float,
    position,
    valid: int = 1,
    occluded: int = 0,
    publish_time_us: int = 0,
):
    return SimpleNamespace(
        name="ball",
        vicon_frame_number=int(frame),
        vicon_time_s=float(source_time),
        publish_time_us=int(publish_time_us),
        valid=int(valid),
        occluded=int(occluded),
        pos_vicon=np.asarray(position, dtype=np.float64),
        quat_vicon=np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32),
    )


def make_real_world_without_io() -> RealWorld:
    planner_cfg = {
        "state_estimator_sample_rate_hz": 360.0,
        "state_estimator_window_size": 31,
        "state_estimator_min_samples": 31,
        "table_center_xy_w": [1.365369, 0.0],
        "table_height": 0.76,
        "table_length": 2.730738,
        "table_width": 1.512451,
        "ball_radius": 0.02,
        "state_estimator_bounce_height_tolerance": 0.03,
        "state_estimator_bounce_velocity_threshold": 0.10,
        "state_estimator_bounce_min_separation_s": 0.20,
    }
    sim = RealWorld.__new__(RealWorld)
    sim.cfg = SimpleNamespace(motion={"ball_planner": planner_cfg})
    sim.root_trans_world_tmp = np.array(
        [-0.4, 0.0, 0.793], dtype=np.float32
    )
    sim.root_quat_world_tmp = np.array(
        [0.0, 0.0, 0.0, 1.0], dtype=np.float32
    )
    sim.firstReceiveVicon = False
    sim._init_ball_state()
    sim.base_pose_valid_tmp = True
    return sim


def plan_snapshot(snapshot: BallEstimateSnapshot) -> ReplayCommand:
    if not snapshot.visible:
        raise RuntimeError("HITTER_TRACK_ENDED")
    if not snapshot.ready:
        raise ValueError("ball estimator is not ready")
    if not snapshot.base_valid:
        raise ValueError("base pose is not valid")
    velocity_x = float(snapshot.velocity_w[0])
    if not np.isfinite(velocity_x) or velocity_x >= -1.0e-6:
        raise ValueError("ball is not incoming")
    time_to_strike = max(float(snapshot.position_w[0]) / -velocity_x, 0.0)
    return ReplayCommand(time_to_strike=time_to_strike)


def wait_for_generation(
    worker: LatestOnlyPlannerWorker,
    generation: int,
    *,
    timeout_s: float = 2.0,
) -> PlannerResultSnapshot:
    deadline = time.perf_counter() + timeout_s
    while time.perf_counter() < deadline:
        result = worker.latest_result()
        if result is not None and int(result.source_generation) >= int(generation):
            return result
        time.sleep(0.0005)
    raise AssertionError(
        "planner worker did not complete generation {} within {:.1f}s".format(
            generation,
            timeout_s,
        )
    )


class OfflineReplayPipeline:
    def __init__(self) -> None:
        self.sim = make_real_world_without_io()
        self.clock = ReplayClock()
        self.consumer = PolicyReplayConsumer()
        self.snapshots: List[BallEstimateSnapshot] = []
        self.worker = LatestOnlyPlannerWorker(
            plan_snapshot,
            monotonic_fn=self.clock,
        )

        def submit(snapshot: BallEstimateSnapshot) -> None:
            self.snapshots.append(snapshot)
            self.worker.submit(snapshot)

        self.unregister = self.sim.register_hitter_ball_listener(submit)

    def feed(self, message, position, *, received_time: float) -> None:
        self.clock.value = float(received_time)
        with patch.object(
            real_world_module.time,
            "monotonic",
            return_value=float(received_time),
        ):
            self.sim._update_ball_state_from_vicon(
                message,
                np.asarray(position, dtype=np.float64),
            )

    def consume_latest(self, *, now: float) -> str:
        result = wait_for_generation(
            self.worker,
            self.sim.ball_snapshot_generation,
        )
        consume_time = float(now) + POLICY_RESULT_CONSUME_DELAY_S
        self.clock.value = consume_time
        return self.consumer.consume(result, now=consume_time)

    def close(self) -> None:
        self.unregister()
        self.worker.close()


def generations_are_monotonic(results: List[PlannerResultSnapshot]) -> bool:
    generations = [int(result.source_generation) for result in results]
    return all(
        current >= previous
        for previous, current in zip(generations, generations[1:])
    )


def collect_stats(
    pipeline: OfflineReplayPipeline,
    *,
    frame_count: int,
    frame_gaps: int,
    bounce_count: int,
    ready_transitions: int,
    ready_reset_count: int,
    first_ready_frame: Optional[int],
    active_after_track_end: bool,
    active_until_command_end: bool,
    phase_after_command_end: Optional[str],
    active_after_command_end: Optional[bool],
    old_epoch_rearm_decision: Optional[str],
    phase_after_old_epoch_rearm: Optional[str],
    active_after_old_epoch_rearm: Optional[bool],
    crossed_hit_plane: bool,
) -> ReplayStats:
    worker_stats = pipeline.worker.stats
    return ReplayStats(
        frame_count=int(frame_count),
        frame_gaps=int(frame_gaps),
        bounce_count=int(bounce_count),
        ready_transitions=int(ready_transitions),
        ready_reset_count=int(ready_reset_count),
        first_ready_frame=first_ready_frame,
        worker_submitted=int(worker_stats.submitted),
        worker_completed=int(worker_stats.completed),
        worker_failed=int(worker_stats.failed),
        worker_dropped_pending=int(worker_stats.dropped_pending),
        worker_max_pending_depth=int(pipeline.worker.max_pending_depth),
        arm_band_candidates=int(pipeline.consumer.arm_band_candidates),
        first_arm_tts=pipeline.consumer.first_arm_tts,
        waiting_policy_tts=float(pipeline.consumer.waiting_policy_tts),
        max_armed_policy_tts=float(pipeline.consumer.max_armed_policy_tts),
        generations_monotonic=generations_are_monotonic(
            pipeline.consumer.consumed_results
        ),
        track_end_events=int(pipeline.consumer.track_end_events),
        estimator_samples_after_track_end=int(
            pipeline.sim.ball_state_estimator_sample_count_tmp
        ),
        visible_after_track_end=bool(pipeline.sim.ball_visible_tmp),
        max_policy_result_age_s=float(
            pipeline.consumer.max_policy_result_age_s
        ),
        active_after_track_end=bool(active_after_track_end),
        active_until_command_end=bool(active_until_command_end),
        phase_after_command_end=phase_after_command_end,
        active_after_command_end=active_after_command_end,
        old_epoch_rearm_decision=old_epoch_rearm_decision,
        phase_after_old_epoch_rearm=phase_after_old_epoch_rearm,
        active_after_old_epoch_rearm=active_after_old_epoch_rearm,
        crossed_hit_plane=bool(crossed_hit_plane),
    )


def replay_synthetic_trajectory() -> ReplayStats:
    pipeline = OfflineReplayPipeline()
    next_policy_time = 0.0
    frame_count = 0
    active_after_track_end = False
    active_until_command_end = False
    phase_after_command_end = None
    active_after_command_end = None
    old_epoch_rearm_decision = None
    phase_after_old_epoch_rearm = None
    active_after_old_epoch_rearm = None
    crossed_hit_plane = False
    try:
        for frame in range(338):
            source_time = frame / 360.0
            position = np.array(
                [14.0 / 15.0 - source_time, 0.0, 1.0],
                dtype=np.float64,
            )
            pipeline.feed(
                make_ball_message(
                    frame=frame,
                    source_time=source_time,
                    position=position,
                ),
                position,
                received_time=source_time,
            )
            frame_count += 1
            crossed_hit_plane = crossed_hit_plane or float(position[0]) < 0.0
            if source_time + 1.0e-12 >= next_policy_time:
                pipeline.consume_latest(now=source_time)
                next_policy_time += POLICY_DT_S

        first_ready_frame = next(
            (
                int(snapshot.source_frame)
                for snapshot in pipeline.snapshots
                if snapshot.ready
            ),
            None,
        )
        old_result = pipeline.consumer.lifecycle.active_result
        invalid_frame = frame_count
        invalid_time = invalid_frame / 360.0
        invalid_position = np.array([-0.01, 0.0, 1.0], dtype=np.float64)
        pipeline.feed(
            make_ball_message(
                frame=invalid_frame,
                source_time=invalid_time,
                position=invalid_position,
                valid=0,
                occluded=1,
            ),
            invalid_position,
            received_time=invalid_time,
        )
        pipeline.consume_latest(now=invalid_time)
        active_after_track_end = (
            pipeline.consumer.lifecycle.active_result is not None
        )

        command_end = pipeline.consumer.lifecycle.command_end_deadline_s
        if command_end is None or old_result is None:
            raise AssertionError("synthetic replay did not arm a command")
        pipeline.consumer.lifecycle.advance(command_end - 1.0e-9)
        active_until_command_end = (
            pipeline.consumer.lifecycle.active_result is not None
        )
        rearm_time = command_end + 1.0e-9
        pipeline.consumer.lifecycle.advance(rearm_time)
        phase_after_command_end = pipeline.consumer.lifecycle.phase.value
        active_after_command_end = (
            pipeline.consumer.lifecycle.active_result is not None
        )

        ended_epoch_result = PlannerResultSnapshot(
            track_epoch=old_result.track_epoch,
            source_generation=pipeline.sim.ball_snapshot_generation + 1,
            source_frame=invalid_frame + 1,
            strike_deadline_monotonic_s=rearm_time + 0.85,
            completed_monotonic_s=rearm_time,
            command=old_result.command,
        )
        old_epoch_rearm_decision = pipeline.consumer.lifecycle.ingest(
            ended_epoch_result,
            now=rearm_time,
        )
        phase_after_old_epoch_rearm = pipeline.consumer.lifecycle.phase.value
        active_after_old_epoch_rearm = (
            pipeline.consumer.lifecycle.active_result is not None
        )

        return collect_stats(
            pipeline,
            frame_count=frame_count,
            frame_gaps=0,
            bounce_count=0,
            ready_transitions=1,
            ready_reset_count=0,
            first_ready_frame=first_ready_frame,
            active_after_track_end=active_after_track_end,
            active_until_command_end=active_until_command_end,
            phase_after_command_end=phase_after_command_end,
            active_after_command_end=active_after_command_end,
            old_epoch_rearm_decision=old_epoch_rearm_decision,
            phase_after_old_epoch_rearm=phase_after_old_epoch_rearm,
            active_after_old_epoch_rearm=active_after_old_epoch_rearm,
            crossed_hit_plane=crossed_hit_plane,
        )
    finally:
        pipeline.close()


def replay_recorded_csv(path: Path) -> ReplayStats:
    pipeline = OfflineReplayPipeline()
    frame_count = 0
    frame_gaps = 0
    bounce_count = 0
    ready_transitions = 0
    ready_reset_count = 0
    first_ready_frame = None
    previous_frame = None
    previous_ready = False
    previous_bounce_time = None
    first_elapsed = None
    next_policy_time = 0.0
    last_row = None
    active_after_track_end = False
    try:
        with path.open(newline="") as handle:
            for row in csv.DictReader(handle):
                if str(row.get("name", "")).strip().lower() != "ball":
                    continue
                frame = int(row["vicon_frame_number"])
                elapsed = float(row["elapsed_s"])
                if first_elapsed is None:
                    first_elapsed = elapsed
                replay_time = elapsed - first_elapsed
                if previous_frame is not None:
                    frame_gaps += max(frame - previous_frame - 1, 0)
                previous_frame = frame

                pipeline.sim.base_pose_valid_tmp = bool(int(row["base_valid"]))
                pipeline.sim.root_trans_world_tmp = np.array(
                    [
                        float(row["base_x_m"]),
                        float(row["base_y_m"]),
                        float(row["base_z_m"]),
                    ],
                    dtype=np.float32,
                )
                pipeline.sim.root_quat_world_tmp = np.array(
                    [
                        float(row["base_qx"]),
                        float(row["base_qy"]),
                        float(row["base_qz"]),
                        float(row["base_qw"]),
                    ],
                    dtype=np.float32,
                )
                position = np.array(
                    [
                        float(row["ball_x_m"]),
                        float(row["ball_y_m"]),
                        float(row["ball_z_m"]),
                    ],
                    dtype=np.float64,
                )
                pipeline.feed(
                    make_ball_message(
                        frame=frame,
                        source_time=float(row["vicon_time_s"]),
                        position=position,
                        valid=int(row["ball_valid"]),
                        occluded=int(row["ball_occluded"]),
                        publish_time_us=int(row["publish_time_us"]),
                    ),
                    position,
                    received_time=replay_time,
                )
                frame_count += 1

                ready = bool(pipeline.sim.ball_state_estimator_ready_tmp)
                if ready and not previous_ready:
                    ready_transitions += 1
                    if first_ready_frame is None:
                        first_ready_frame = frame
                if previous_ready and not ready:
                    ready_reset_count += 1
                bounce_time = pipeline.sim.ball_state_estimator._last_bounce_time
                if (
                    bounce_time is not None
                    and bounce_time != previous_bounce_time
                ):
                    bounce_count += 1
                    previous_bounce_time = bounce_time
                previous_ready = ready

                if replay_time + 1.0e-12 >= next_policy_time:
                    pipeline.consume_latest(now=replay_time)
                    next_policy_time += POLICY_DT_S
                last_row = row

        if last_row is None or first_elapsed is None:
            raise AssertionError("recorded ChingMu CSV contains no ball rows")

        invalid_frame = int(last_row["vicon_frame_number"]) + 1
        invalid_source_time = float(last_row["vicon_time_s"]) + 1.0 / 360.0
        invalid_time = float(last_row["elapsed_s"]) - first_elapsed + 1.0 / 360.0
        invalid_position = np.array(
            [
                -0.001,
                float(last_row["ball_y_m"]),
                float(last_row["ball_z_m"]),
            ],
            dtype=np.float64,
        )
        pipeline.feed(
            make_ball_message(
                frame=invalid_frame,
                source_time=invalid_source_time,
                position=invalid_position,
                valid=0,
                occluded=1,
            ),
            invalid_position,
            received_time=invalid_time,
        )
        pipeline.consume_latest(now=invalid_time)
        active_after_track_end = (
            pipeline.consumer.lifecycle.active_result is not None
        )

        return collect_stats(
            pipeline,
            frame_count=frame_count,
            frame_gaps=frame_gaps,
            bounce_count=bounce_count,
            ready_transitions=ready_transitions,
            ready_reset_count=ready_reset_count,
            first_ready_frame=first_ready_frame,
            active_after_track_end=active_after_track_end,
            active_until_command_end=False,
            phase_after_command_end=None,
            active_after_command_end=None,
            old_epoch_rearm_decision=None,
            phase_after_old_epoch_rearm=None,
            active_after_old_epoch_rearm=None,
            crossed_hit_plane=float(invalid_position[0]) < 0.0,
        )
    finally:
        pipeline.close()


class HitterRecordedReplayTest(unittest.TestCase):
    def test_synthetic_360hz_replay_drives_full_realtime_chain(self):
        stats = replay_synthetic_trajectory()

        self.assertEqual(stats.first_ready_frame, 30)
        self.assertEqual(stats.worker_max_pending_depth, 1)
        self.assertIsNotNone(stats.first_arm_tts)
        self.assertGreaterEqual(stats.first_arm_tts, 0.80)
        self.assertLessEqual(stats.first_arm_tts, 0.90)
        self.assertEqual(stats.waiting_policy_tts, 1.00)
        self.assertTrue(stats.generations_monotonic)
        self.assertEqual(stats.track_end_events, 1)
        self.assertTrue(stats.crossed_hit_plane)
        self.assertEqual(stats.estimator_samples_after_track_end, 0)
        self.assertFalse(stats.visible_after_track_end)
        self.assertTrue(stats.active_after_track_end)
        self.assertTrue(stats.active_until_command_end)
        self.assertEqual(
            stats.phase_after_command_end,
            CommandPhase.WAITING.value,
        )
        self.assertFalse(stats.active_after_command_end)
        self.assertEqual(stats.old_epoch_rearm_decision, "ignored")
        self.assertEqual(
            stats.phase_after_old_epoch_rearm,
            CommandPhase.WAITING.value,
        )
        self.assertFalse(stats.active_after_old_epoch_rearm)
        self.assertLessEqual(stats.max_armed_policy_tts, 0.92)
        self.assertTrue(np.isfinite(stats.max_policy_result_age_s))
        self.assertGreater(stats.max_policy_result_age_s, 0.0)
        self.assertLessEqual(stats.max_policy_result_age_s, POLICY_DT_S)

    @unittest.skipUnless(RECORDED.exists(), "recorded ChingMu CSV is not available")
    def test_recorded_four_bounce_capture_clears_track(self):
        stats = replay_recorded_csv(RECORDED)

        self.assertEqual(stats.frame_gaps, 0)
        self.assertEqual(stats.bounce_count, 4)
        self.assertEqual(stats.ready_reset_count, 4)
        self.assertGreaterEqual(stats.ready_transitions, 5)
        self.assertEqual(stats.worker_max_pending_depth, 1)
        self.assertTrue(stats.generations_monotonic)
        self.assertEqual(stats.track_end_events, 1)
        self.assertTrue(stats.crossed_hit_plane)
        self.assertEqual(stats.estimator_samples_after_track_end, 0)
        self.assertFalse(stats.visible_after_track_end)
        self.assertLessEqual(stats.max_armed_policy_tts, 0.92)
        self.assertTrue(np.isfinite(stats.max_policy_result_age_s))
        self.assertGreater(stats.max_policy_result_age_s, 0.0)
        self.assertLessEqual(stats.max_policy_result_age_s, POLICY_DT_S)
        print(json.dumps(dataclasses.asdict(stats), sort_keys=True))


if __name__ == "__main__":
    unittest.main()
