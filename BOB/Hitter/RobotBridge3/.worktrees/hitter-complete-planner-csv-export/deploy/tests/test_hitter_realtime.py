from __future__ import annotations

import dataclasses
import threading
import time
import unittest
from types import SimpleNamespace

import numpy as np

from utils.hitter_realtime import (
    BallEstimateSnapshot,
    CommandPhase,
    HitterCommandLifecycle,
    IncomingTrackConfirmation,
    LatestOnlyPlannerWorker,
    PlannerResultSnapshot,
)


def wait_until(predicate, *, timeout_s=1.0, message="condition was not met"):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.002)
    raise AssertionError(message)


def snapshot(*, epoch=1, generation=1, frame=None, received=10.0):
    if frame is None:
        frame = generation
    return BallEstimateSnapshot(
        track_epoch=epoch,
        generation=generation,
        source_frame=frame,
        source_time_s=100.0 + generation,
        received_monotonic_s=received,
        position_w=np.array([1.0, 2.0, 3.0]),
        velocity_w=np.array([-1.0, 0.1, 0.2]),
        base_position_w=np.array([-0.4, 0.0, 0.793]),
        base_quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
        base_valid=True,
        visible=True,
        ready=True,
    )


def fake_command(*, tts=0.9, label="command"):
    return SimpleNamespace(time_to_strike=float(tts), label=label)


def result(
    *,
    epoch=1,
    generation=1,
    deadline=10.9,
    completed=10.01,
    command=None,
    error=None,
):
    if command is None and error is None:
        command = fake_command(label=f"e{epoch}g{generation}")
    return PlannerResultSnapshot(
        track_epoch=epoch,
        source_generation=generation,
        source_frame=generation,
        strike_deadline_monotonic_s=float(deadline),
        completed_monotonic_s=float(completed),
        command=command,
        error=error,
    )


def configured_lifecycle():
    return HitterCommandLifecycle(
        waiting_tts=1.0,
        arm_tts=0.90,
        minimum_arm_tts=0.80,
        maximum_policy_tts=0.92,
        swing_duration_sampler=lambda: 1.85,
    )


class IncomingTrackConfirmationTests(unittest.TestCase):
    def make_confirmation(self):
        return IncomingTrackConfirmation(
            minimum_speed_x_mps=0.20,
            required_consecutive_snapshots=3,
        )

    def test_requires_three_consecutive_stable_incoming_snapshots(self):
        confirmation = self.make_confirmation()

        self.assertFalse(
            confirmation.observe(track_epoch=4, velocity_x_mps=-0.21)
        )
        self.assertFalse(
            confirmation.observe(track_epoch=4, velocity_x_mps=-0.30)
        )
        self.assertTrue(
            confirmation.observe(track_epoch=4, velocity_x_mps=-0.40)
        )

    def test_nonqualifying_snapshot_resets_unconfirmed_count(self):
        confirmation = self.make_confirmation()

        self.assertFalse(
            confirmation.observe(track_epoch=4, velocity_x_mps=-0.30)
        )
        self.assertFalse(
            confirmation.observe(track_epoch=4, velocity_x_mps=-0.05)
        )
        self.assertFalse(
            confirmation.observe(track_epoch=4, velocity_x_mps=-0.30)
        )
        self.assertFalse(
            confirmation.observe(track_epoch=4, velocity_x_mps=-0.30)
        )
        self.assertTrue(
            confirmation.observe(track_epoch=4, velocity_x_mps=-0.30)
        )

    def test_new_epoch_clears_confirmation_and_confirmation_latches(self):
        confirmation = self.make_confirmation()

        for _ in range(3):
            confirmed = confirmation.observe(
                track_epoch=4,
                velocity_x_mps=-0.30,
            )

        self.assertTrue(confirmed)
        self.assertTrue(
            confirmation.observe(track_epoch=4, velocity_x_mps=0.10)
        )
        self.assertFalse(
            confirmation.observe(track_epoch=5, velocity_x_mps=-0.30)
        )

    def test_explicit_reset_clears_confirmation(self):
        confirmation = self.make_confirmation()
        for _ in range(3):
            confirmation.observe(track_epoch=4, velocity_x_mps=-0.30)

        confirmation.reset()

        self.assertFalse(
            confirmation.observe(track_epoch=4, velocity_x_mps=-0.30)
        )


class SnapshotTests(unittest.TestCase):
    def test_lifecycle_default_waiting_tts_matches_training_maximum(self):
        lifecycle = HitterCommandLifecycle()

        self.assertEqual(lifecycle.policy_tts(now=0.0), 0.92)

    def test_ball_snapshot_copies_and_freezes_every_array(self):
        arrays = [
            np.array([1.0, 2.0, 3.0]),
            np.array([4.0, 5.0, 6.0]),
            np.array([7.0, 8.0, 9.0]),
            np.array([0.0, 0.0, 0.0, 1.0]),
        ]
        item = BallEstimateSnapshot(
            track_epoch=3,
            generation=7,
            source_frame=11,
            source_time_s=4.0,
            received_monotonic_s=5.0,
            position_w=arrays[0],
            velocity_w=arrays[1],
            base_position_w=arrays[2],
            base_quaternion_xyzw=arrays[3],
            base_valid=True,
            visible=True,
            ready=True,
        )

        stored = (
            item.position_w,
            item.velocity_w,
            item.base_position_w,
            item.base_quaternion_xyzw,
        )
        arrays[0][0] = 99.0
        self.assertEqual(item.position_w[0], 1.0)
        for value in stored:
            self.assertFalse(value.flags.writeable)
            with self.assertRaises(ValueError):
                value[0] = -1.0
        with self.assertRaises(dataclasses.FrozenInstanceError):
            item.generation = 8

    def test_planner_result_is_frozen(self):
        item = result()
        with self.assertRaises(dataclasses.FrozenInstanceError):
            item.source_generation = 2


class LatestOnlyPlannerWorkerTests(unittest.TestCase):
    def test_pending_slot_keeps_only_latest_generation(self):
        blocker = threading.Event()
        seen = []

        def plan(item):
            seen.append(item.generation)
            if item.generation == 1:
                blocker.wait(1.0)
            return fake_command(tts=0.9)

        worker = LatestOnlyPlannerWorker(plan, monotonic_fn=lambda: 10.0)
        try:
            worker.submit(snapshot(generation=1, received=5.0))
            wait_until(lambda: seen == [1])
            worker.submit(snapshot(generation=2, received=6.0))
            worker.submit(snapshot(generation=3, received=7.0))
            blocker.set()
            wait_until(lambda: seen == [1, 3])
            wait_until(lambda: worker.stats.completed == 2)

            self.assertEqual(worker.stats.submitted, 3)
            self.assertEqual(worker.stats.completed, 2)
            self.assertEqual(worker.stats.failed, 0)
            self.assertEqual(worker.stats.dropped_pending, 1)
            self.assertEqual(worker.max_pending_depth, 1)
            self.assertEqual(worker.latest_result().source_generation, 3)
            with self.assertRaises(dataclasses.FrozenInstanceError):
                worker.stats.submitted = 0
        finally:
            blocker.set()
            worker.close()

    def test_submit_does_not_take_the_plan_function_lock(self):
        planning = threading.Event()
        release = threading.Event()
        submitted = threading.Event()

        def plan(item):
            planning.set()
            release.wait(1.0)
            return fake_command()

        worker = LatestOnlyPlannerWorker(plan)
        submitter = None
        try:
            worker.submit(snapshot(generation=1))
            self.assertTrue(planning.wait(0.5))

            def submit_while_planning():
                worker.submit(snapshot(generation=2))
                submitted.set()

            submitter = threading.Thread(target=submit_while_planning)
            submitter.start()
            self.assertTrue(
                submitted.wait(0.25),
                "submit blocked, so plan_fn was probably called under the condition lock",
            )
            release.set()
            wait_until(lambda: worker.stats.completed == 2)
        finally:
            release.set()
            if submitter is not None:
                submitter.join(0.5)
            worker.close()

    def test_success_deadline_uses_snapshot_receive_time(self):
        worker = LatestOnlyPlannerWorker(
            lambda item: fake_command(tts=0.75),
            monotonic_fn=lambda: 99.0,
        )
        try:
            worker.submit(snapshot(generation=4, received=12.5))
            wait_until(lambda: worker.latest_result() is not None)
            planned = worker.latest_result()
            self.assertAlmostEqual(planned.strike_deadline_monotonic_s, 13.25)
            self.assertAlmostEqual(planned.completed_monotonic_s, 99.0)
        finally:
            worker.close()

    def test_planner_exception_publishes_failure_and_worker_continues(self):
        def plan(item):
            if item.generation == 1:
                raise RuntimeError("planner exploded")
            return fake_command(tts=0.8, label="recovered")

        worker = LatestOnlyPlannerWorker(plan, monotonic_fn=lambda: 20.0)
        try:
            worker.submit(snapshot(generation=1))
            wait_until(
                lambda: worker.latest_result() is not None
                and worker.latest_result().source_generation == 1
            )
            failed = worker.latest_result()
            self.assertIsNone(failed.command)
            self.assertIn("planner exploded", failed.error)
            self.assertEqual(worker.stats.failed, 1)

            worker.submit(snapshot(generation=2))
            wait_until(
                lambda: worker.latest_result() is not None
                and worker.latest_result().source_generation == 2
            )
            recovered = worker.latest_result()
            self.assertIsNone(recovered.error)
            self.assertEqual(recovered.command.label, "recovered")
            self.assertEqual(worker.stats.completed, 1)
        finally:
            worker.close()

    def test_close_is_idempotent_and_leaves_no_live_worker(self):
        worker = LatestOnlyPlannerWorker(lambda item: fake_command())
        worker.close()
        worker.close()
        self.assertFalse(worker._thread.is_alive())
        with self.assertRaises(RuntimeError):
            worker.submit(snapshot())


class HitterCommandLifecycleTests(unittest.TestCase):
    def test_waiting_and_tracking_tts_are_exactly_one_second(self):
        lifecycle = configured_lifecycle()
        self.assertEqual(lifecycle.phase, CommandPhase.WAITING)
        self.assertEqual(lifecycle.policy_tts(now=1.0), 1.0)

        self.assertEqual(
            lifecycle.ingest(result(epoch=1, generation=1, deadline=2.0), now=1.0),
            "tracking",
        )
        self.assertEqual(lifecycle.phase, CommandPhase.TRACKING)
        self.assertEqual(lifecycle.policy_tts(now=1.1), 1.0)

    def test_initial_arm_band_is_inclusive(self):
        for tts in (0.80, 0.90):
            with self.subTest(tts=tts):
                lifecycle = configured_lifecycle()
                self.assertEqual(
                    lifecycle.ingest(result(deadline=10.0 + tts), now=10.0),
                    "armed",
                )
                self.assertEqual(lifecycle.phase, CommandPhase.ARMED)

    def test_waiting_is_one_second_but_armed_tts_is_capped(self):
        lifecycle = configured_lifecycle()
        self.assertEqual(lifecycle.policy_tts(now=1.0), 1.0)
        lifecycle.ingest(result(epoch=1, generation=1, deadline=1.9), now=1.0)
        self.assertAlmostEqual(lifecycle.policy_tts(now=1.0), 0.9)
        self.assertAlmostEqual(lifecycle.policy_tts(now=0.9), 0.92)

    def test_arm_override_and_recovery_follow_latest_deadline(self):
        lifecycle = configured_lifecycle()
        armed = result(epoch=4, generation=1, deadline=10.90)
        self.assertEqual(lifecycle.ingest(armed, now=10.0), "armed")
        self.assertAlmostEqual(lifecycle.recovery_duration_s, 0.95)
        self.assertIs(lifecycle.active_result, armed)

        updated = result(epoch=4, generation=2, deadline=11.00)
        self.assertEqual(lifecycle.ingest(updated, now=10.2), "overridden")
        self.assertIs(lifecycle.active_result, updated)
        self.assertAlmostEqual(lifecycle.recovery_duration_s, 0.95)
        self.assertAlmostEqual(lifecycle.command_end_deadline_s, 11.95)

        lifecycle.advance(11.00)
        self.assertEqual(lifecycle.phase, CommandPhase.RECOVERY)
        self.assertIs(lifecycle.active_result, updated)
        lifecycle.advance(11.95)
        self.assertEqual(lifecycle.phase, CommandPhase.WAITING)
        self.assertIsNone(lifecycle.active_result)

    def test_first_late_result_skips_the_entire_epoch(self):
        lifecycle = configured_lifecycle()
        self.assertEqual(
            lifecycle.ingest(result(epoch=3, generation=1, deadline=10.79), now=10.0),
            "skipped",
        )
        self.assertEqual(lifecycle.phase, CommandPhase.WAITING)
        self.assertEqual(
            lifecycle.ingest(result(epoch=3, generation=2, deadline=10.85), now=10.0),
            "ignored",
        )
        self.assertEqual(
            lifecycle.ingest(result(epoch=4, generation=1, deadline=10.85), now=10.0),
            "armed",
        )

    def test_jump_from_too_early_to_too_late_skips_epoch(self):
        lifecycle = configured_lifecycle()
        self.assertEqual(
            lifecycle.ingest(result(epoch=1, generation=1, deadline=11.2), now=10.0),
            "tracking",
        )
        self.assertEqual(
            lifecycle.ingest(result(epoch=1, generation=2, deadline=10.79), now=10.0),
            "skipped",
        )
        self.assertEqual(lifecycle.phase, CommandPhase.WAITING)

    def test_old_epochs_generations_and_failed_results_are_ignored(self):
        lifecycle = configured_lifecycle()
        current = result(epoch=5, generation=3, deadline=10.9)
        self.assertEqual(lifecycle.ingest(current, now=10.0), "armed")

        failed = result(
            epoch=5,
            generation=4,
            deadline=float("nan"),
            command=None,
            error="bad plan",
        )
        self.assertEqual(lifecycle.ingest(failed, now=10.0), "ignored")
        self.assertEqual(
            lifecycle.ingest(result(epoch=5, generation=3, deadline=10.8), now=10.0),
            "ignored",
        )
        self.assertEqual(
            lifecycle.ingest(result(epoch=4, generation=99, deadline=10.8), now=10.0),
            "ignored",
        )
        self.assertIs(lifecycle.active_result, current)

    def test_generation_zero_duplicate_is_ignored(self):
        lifecycle = configured_lifecycle()
        current = result(epoch=5, generation=0, deadline=10.9)
        self.assertEqual(lifecycle.ingest(current, now=10.0), "armed")

        self.assertEqual(
            lifecycle.ingest(result(epoch=5, generation=0, deadline=10.8), now=10.0),
            "ignored",
        )
        self.assertIs(lifecycle.active_result, current)

    def test_out_of_band_override_retains_active_command(self):
        lifecycle = configured_lifecycle()
        active = result(epoch=2, generation=1, deadline=10.9)
        lifecycle.ingest(active, now=10.0)

        self.assertEqual(
            lifecycle.ingest(result(epoch=2, generation=2, deadline=11.13), now=10.2),
            "retained",
        )
        self.assertIs(lifecycle.active_result, active)
        self.assertAlmostEqual(lifecycle.command_end_deadline_s, 11.85)

        self.assertEqual(
            lifecycle.ingest(result(epoch=2, generation=3, deadline=10.19), now=10.2),
            "retained",
        )
        self.assertIs(lifecycle.active_result, active)

    def test_override_remaining_time_band_is_inclusive(self):
        for remaining in (0.0, 0.92):
            with self.subTest(remaining=remaining):
                lifecycle = configured_lifecycle()
                active = result(epoch=2, generation=1, deadline=10.9)
                lifecycle.ingest(active, now=10.0)
                updated = result(
                    epoch=2,
                    generation=2,
                    deadline=10.2 + remaining,
                )

                self.assertEqual(
                    lifecycle.ingest(updated, now=10.2),
                    "overridden",
                )
                self.assertIs(lifecycle.active_result, updated)
                self.assertAlmostEqual(lifecycle.recovery_duration_s, 0.95)

    def test_mark_track_ended_blocks_override_without_deleting_active(self):
        lifecycle = configured_lifecycle()
        active = result(epoch=8, generation=1, deadline=10.9)
        lifecycle.ingest(active, now=10.0)
        lifecycle.mark_track_ended()

        self.assertEqual(
            lifecycle.ingest(result(epoch=8, generation=2, deadline=10.85), now=10.1),
            "ignored",
        )
        self.assertIs(lifecycle.active_result, active)
        self.assertEqual(lifecycle.phase, CommandPhase.ARMED)

    def test_mark_track_ended_in_recovery_ends_cached_next_epoch(self):
        lifecycle = configured_lifecycle()
        active = result(epoch=1, generation=1, deadline=10.9)
        lifecycle.ingest(active, now=10.0)
        lifecycle.advance(10.9)
        self.assertEqual(lifecycle.phase, CommandPhase.RECOVERY)

        upcoming = result(epoch=2, generation=1, deadline=12.75)
        self.assertEqual(lifecycle.ingest(upcoming, now=11.0), "cached")
        lifecycle.mark_track_ended()

        self.assertEqual(lifecycle.phase, CommandPhase.RECOVERY)
        self.assertIs(lifecycle.active_result, active)
        self.assertIsNone(lifecycle.cached_result)
        self.assertEqual(
            lifecycle.ingest(
                result(epoch=2, generation=2, deadline=12.75),
                now=11.1,
            ),
            "ignored",
        )

        lifecycle.advance(11.85)
        self.assertEqual(lifecycle.phase, CommandPhase.WAITING)
        self.assertIsNone(lifecycle.active_result)
        self.assertEqual(
            lifecycle.ingest(
                result(epoch=2, generation=3, deadline=12.70),
                now=11.85,
            ),
            "ignored",
        )

    def test_explicit_track_epoch_overrides_recovery_default_target(self):
        lifecycle = configured_lifecycle()
        active = result(epoch=1, generation=1, deadline=10.9)
        lifecycle.ingest(active, now=10.0)
        lifecycle.advance(10.9)
        upcoming = result(epoch=2, generation=1, deadline=12.75)
        lifecycle.ingest(upcoming, now=11.0)

        lifecycle.mark_track_ended(track_epoch=1)

        self.assertEqual(lifecycle.phase, CommandPhase.RECOVERY)
        self.assertIs(lifecycle.active_result, active)
        self.assertIs(lifecycle.cached_result, upcoming)
        lifecycle.advance(11.85)
        self.assertEqual(lifecycle.phase, CommandPhase.ARMED)
        self.assertIs(lifecycle.active_result, upcoming)

    def test_recovery_caches_latest_next_epoch_and_arms_at_command_end(self):
        lifecycle = configured_lifecycle()
        lifecycle.ingest(result(epoch=4, generation=1, deadline=10.9), now=10.0)
        lifecycle.advance(10.9)
        self.assertEqual(lifecycle.phase, CommandPhase.RECOVERY)

        self.assertEqual(
            lifecycle.ingest(result(epoch=5, generation=1, deadline=12.82), now=11.0),
            "cached",
        )
        latest = result(epoch=5, generation=2, deadline=12.75)
        self.assertEqual(lifecycle.ingest(latest, now=11.1), "cached")
        self.assertIs(lifecycle.cached_result, latest)
        self.assertEqual(lifecycle.phase, CommandPhase.RECOVERY)

        lifecycle.advance(11.85)
        self.assertEqual(lifecycle.phase, CommandPhase.ARMED)
        self.assertIs(lifecycle.active_result, latest)
        self.assertAlmostEqual(lifecycle.policy_tts(now=11.85), 0.90)

    def test_recovery_rechecks_cached_result_and_skips_if_now_late(self):
        lifecycle = configured_lifecycle()
        lifecycle.ingest(result(epoch=1, generation=1, deadline=10.9), now=10.0)
        lifecycle.advance(10.9)
        lifecycle.ingest(result(epoch=2, generation=1, deadline=12.50), now=11.0)

        lifecycle.advance(11.85)
        self.assertEqual(lifecycle.phase, CommandPhase.WAITING)
        self.assertIsNone(lifecycle.active_result)
        self.assertEqual(
            lifecycle.ingest(result(epoch=2, generation=2, deadline=12.70), now=11.85),
            "ignored",
        )


def tearDownModule():
    live_workers = [
        thread.name
        for thread in threading.enumerate()
        if thread.name.startswith("LatestOnlyPlannerWorker-")
    ]
    if live_workers:
        raise AssertionError(f"live planner workers after tests: {live_workers}")


if __name__ == "__main__":
    unittest.main()
