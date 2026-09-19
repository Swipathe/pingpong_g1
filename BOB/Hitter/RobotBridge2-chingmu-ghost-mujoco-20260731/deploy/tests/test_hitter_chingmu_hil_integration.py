from __future__ import annotations

import json
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from utils.hitter_ball_pipeline import BasePoseW, RealtimeViconBallPipeline
from utils.hitter_realtime import (
    CommandPhase,
    HitterCommandLifecycle,
    LatestOnlyPlannerWorker,
)
from utils.read_only_lcm import PublicationAudit


FIXTURE = Path(__file__).resolve().parent / "fixtures" / "chingmu_ghost_incoming_ball.jsonl"


def load_fixture():
    return [
        json.loads(line)
        for line in FIXTURE.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def planner_config():
    return {
        "state_estimator_window_size": 31,
        "state_estimator_sample_rate_hz": 360.0,
        "state_estimator_bounce_height_tolerance": 0.03,
        "state_estimator_bounce_velocity_threshold": 0.10,
        "state_estimator_bounce_min_separation_s": 0.20,
        "vertical_restitution": 0.897474,
        "horizontal_restitution": 0.764604,
        "drag_coefficient": 0.098847,
    }


class FakeGhostBall:
    def __init__(self):
        self.visible = False
        self.contact_count = 0
        self.snapshots = []

    def queue_hitter_ghost_snapshot(self, snapshot):
        self.snapshots.append(snapshot)
        self.visible = bool(snapshot.visible and snapshot.ready)

    def hide(self):
        self.visible = False


class DeterministicClock:
    def __init__(self):
        self.value = 0.0

    def set(self, value):
        self.value = float(value)

    def __call__(self):
        return self.value


class HitterChingMuHilIntegrationTests(unittest.TestCase):
    def test_replay_drives_estimator_worker_lifecycle_and_zero_control(self):
        records = load_fixture()
        self.assertGreaterEqual(len(records), 31)
        self.assertTrue(all(record["channel"] == "vicon_state_data" for record in records))
        self.assertTrue(all(record["name"] == "ball" for record in records))

        clock = DeterministicClock()
        base_pose_count = 0

        def copy_mujoco_base_pose_w():
            nonlocal base_pose_count
            base_pose_count += 1
            return BasePoseW(
                position_w=np.asarray([0.0, 0.0, 0.78], dtype=np.float32),
                quaternion_xyzw=np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float32),
                valid=True,
                simulation_time_s=clock.value,
                captured_monotonic_s=clock.value,
            )

        pipeline = RealtimeViconBallPipeline(
            planner_config(),
            copy_mujoco_base_pose_w,
            stale_timeout_s=0.10,
            monotonic_fn=clock,
        )
        ghost = FakeGhostBall()
        unregister = pipeline.register_listener(
            ghost.queue_hitter_ghost_snapshot
        )
        audit = PublicationAudit()
        latest_snapshot = None

        for record in records:
            clock.set(record["received_monotonic_delta_s"])
            msg = SimpleNamespace(
                name=record["name"],
                pos_vicon=np.asarray(record["position_w"], dtype=np.float32),
                vicon_frame_number=record["source_frame"],
                vicon_time_s=record["source_time_s"],
                publish_time_us=record["publish_time_us"],
                valid=int(record["valid"]),
                occluded=int(record["occluded"]),
            )
            update = pipeline.ingest_transformation_update(
                msg,
                received_monotonic_s=clock.value,
            )
            if update.snapshot is not None:
                latest_snapshot = update.snapshot

        state = pipeline.state()
        self.assertTrue(state.visible)
        self.assertTrue(state.ready)
        self.assertGreaterEqual(state.sample_count, 31)
        self.assertIsNotNone(latest_snapshot)
        self.assertTrue(ghost.visible)
        self.assertGreaterEqual(base_pose_count, len(records))
        np.testing.assert_allclose(
            latest_snapshot.base_position_w,
            [0.0, 0.0, 0.78],
        )

        planner_calls = []

        def plan(snapshot):
            planner_calls.append(snapshot)
            return SimpleNamespace(time_to_strike=0.70)

        worker_clock = DeterministicClock()
        worker_clock.set(latest_snapshot.received_monotonic_s)
        worker = LatestOnlyPlannerWorker(plan, monotonic_fn=worker_clock)
        try:
            worker.submit(latest_snapshot)
            deadline = time.time() + 2.0
            result = None
            while time.time() < deadline:
                result = worker.latest_result()
                if result is not None:
                    break
                time.sleep(0.005)
        finally:
            worker.close()

        self.assertEqual(len(planner_calls), 1)
        self.assertIsNotNone(result)
        self.assertEqual(result.track_epoch, state.track_epoch)
        self.assertEqual(result.source_generation, state.generation)

        lifecycle = HitterCommandLifecycle(
            waiting_tts=0.92,
            arm_tts=0.92,
            minimum_arm_tts=0.60,
            maximum_policy_tts=0.92,
            swing_duration_sampler=lambda: 1.85,
        )
        decision = lifecycle.ingest(
            result,
            now=float(result.completed_monotonic_s),
        )
        self.assertEqual(decision, "armed")
        self.assertEqual(lifecycle.phase, CommandPhase.ARMED)

        invalid_msg = SimpleNamespace(
            name="ball",
            pos_vicon=np.asarray([0.0, 0.0, 0.0], dtype=np.float32),
            vicon_frame_number=999,
            vicon_time_s=0.20,
            publish_time_us=1200000,
            valid=0,
            occluded=1,
        )
        clock.set(0.20)
        invalid_update = pipeline.ingest_transformation_update(
            invalid_msg,
            received_monotonic_s=clock.value,
        )
        self.assertIsNotNone(invalid_update.snapshot)
        ghost.hide()
        lifecycle.mark_track_ended(result.track_epoch)
        lifecycle.advance(
            float(result.strike_deadline_monotonic_s) + 2.0
        )
        self.assertEqual(lifecycle.phase, CommandPhase.WAITING)
        self.assertFalse(ghost.visible)

        old_decision = lifecycle.ingest(
            result,
            now=float(result.strike_deadline_monotonic_s) + 2.0,
        )
        self.assertEqual(old_decision, "ignored")

        publish_count, channels = audit.snapshot()
        self.assertEqual(publish_count, 0)
        self.assertEqual(channels, ())
        self.assertEqual(ghost.contact_count, 0)
        unregister()

    def test_integration_does_not_construct_realworld_or_trans_process(self):
        with patch("simulator.real_world.RealWorld") as real_world:
            with patch("subprocess.Popen") as popen:
                records = load_fixture()
                self.assertGreater(len(records), 0)

        real_world.assert_not_called()
        popen.assert_not_called()


if __name__ == "__main__":
    unittest.main()
