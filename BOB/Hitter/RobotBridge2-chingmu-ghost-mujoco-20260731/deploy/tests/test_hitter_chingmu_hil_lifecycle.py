from __future__ import annotations

import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from envs.hitter import HitterEnv
from utils.hitter_realtime import (
    BallEstimateSnapshot,
    CommandPhase,
    PlannerResultSnapshot,
)
from utils.hitter_runtime_capabilities import (
    HitterRuntimeCapabilities,
    LoopPacing,
    PlannerFeed,
)


HYBRID_CAPABILITIES = HitterRuntimeCapabilities(
    planner_feed=PlannerFeed.SNAPSHOT_STREAM,
    loop_pacing=LoopPacing.SIMULATOR,
)


class RecordingSource:
    def __init__(self, *, track_epoch=7, generation=11):
        self.events = []
        self._track_epoch = int(track_epoch)
        self._generation = int(generation)
        self._listener = None

    def register_hitter_ball_listener(self, listener):
        self.events.append(("register", listener))
        self._listener = listener

        def unregister():
            self.events.append("unregister")

        return unregister

    def reset_ball_state_estimator(self):
        self.events.append("reset")
        return self._track_epoch

    def state(self):
        self.events.append("state")
        return SimpleNamespace(
            track_epoch=self._track_epoch,
            generation=self._generation,
            visible=True,
            ready=True,
            position_w=np.asarray([1.0, 0.0, 0.9], dtype=np.float64),
            velocity_w=np.asarray([-3.0, 0.0, 0.0], dtype=np.float64),
        )


def build_hybrid_env(source) -> HitterEnv:
    env = HitterEnv.__new__(HitterEnv)
    env.simulator = SimpleNamespace(
        is_real=False,
        hitter_runtime_capabilities=HYBRID_CAPABILITIES,
        hitter_ball_source=source,
        register_hitter_ball_listener=lambda _listener: (_ for _ in ()).throw(
            AssertionError("listener must be registered on hitter_ball_source")
        ),
        reset_ball_state_estimator=lambda: (_ for _ in ()).throw(
            AssertionError("reset must be executed on hitter_ball_source")
        ),
    )
    env.hitter_ball_source = source
    env.hitter_planner_worker = None
    env._unregister_ball_listener = None
    return env


class HitterChingMuHilLifecycleTests(unittest.TestCase):
    def test_snapshot_startup_fails_fast_without_source_contract(self):
        env = HitterEnv.__new__(HitterEnv)
        env.simulator = SimpleNamespace(
            is_real=False,
            hitter_runtime_capabilities=HYBRID_CAPABILITIES,
            hitter_ball_source=SimpleNamespace(state=lambda: object()),
        )

        with self.assertRaisesRegex(
            RuntimeError,
            "register_hitter_ball_listener, reset_ball_state_estimator",
        ):
            env._init_hitter_ball_source_boundary()

    def test_listener_registration_uses_external_source_not_simulator(self):
        source = RecordingSource()
        env = build_hybrid_env(source)
        worker = object()

        with patch(
            "envs.hitter.LatestOnlyPlannerWorker",
            return_value=worker,
        ) as worker_type:
            env._initialize_hitter_realtime_runtime()

        self.assertIs(env.hitter_planner_worker, worker)
        self.assertEqual(source.events[0][0], "register")
        self.assertIs(env._unregister_ball_listener(), None)
        self.assertEqual(source.events[-1], "unregister")
        worker_type.assert_called_once()

    def test_lifecycle_reset_uses_external_source_epoch_and_generation(self):
        source = RecordingSource(track_epoch=7, generation=11)
        env = build_hybrid_env(source)
        env._new_hitter_command_lifecycle = lambda: SimpleNamespace(
            phase=CommandPhase.WAITING
        )
        env.hitter_incoming_track_confirmation = SimpleNamespace(
            reset=lambda: source.events.append("incoming-reset")
        )
        env._hitter_sync_track_epoch = 3

        env._reset_hitter_lifecycle_state()

        self.assertEqual(
            source.events,
            ["reset", "incoming-reset", "state"],
        )
        self.assertEqual(env._hitter_minimum_track_epoch, 7)
        self.assertEqual(env._hitter_minimum_generation, 12)
        self.assertEqual(env._hitter_sync_track_epoch, 3)

    def test_crossing_strike_resets_external_source_once(self):
        source = RecordingSource(track_epoch=3, generation=5)
        env = build_hybrid_env(source)
        active = PlannerResultSnapshot(
            track_epoch=3,
            source_generation=5,
            source_frame=9,
            strike_deadline_monotonic_s=10.0,
            completed_monotonic_s=9.0,
            command=None,
        )

        class CrossingLifecycle:
            def __init__(self):
                self.phase = CommandPhase.ARMED
                self.active_result = active
                self.cached_result = None
                self.command_end_deadline_s = None

            def advance(self, now):
                self.phase = CommandPhase.RECOVERY
                return self.phase

            def policy_tts(self, *, now):
                return 0.0

        env.hitter_command_lifecycle = CrossingLifecycle()
        env.hitter_incoming_track_confirmation = SimpleNamespace(
            reset=lambda: source.events.append("incoming-reset")
        )
        env._hitter_last_logged_phase = CommandPhase.ARMED
        env._hitter_last_result_key = None
        env.hitter_planner_worker = None
        env.hitter_ball_sequence_needs_reset = False

        env._update_hitter_command(now=11.0)

        self.assertEqual(
            source.events,
            ["reset", "incoming-reset"],
        )
        self.assertTrue(env.hitter_command_initialized)

    def test_planner_snapshot_uses_mujoco_base_pose_values(self):
        source = RecordingSource()
        env = build_hybrid_env(source)
        env.hitter_incoming_track_confirmation = SimpleNamespace(
            observe=lambda **_kwargs: True,
            reset=lambda: None,
        )
        planner_calls = []
        env._forced_strike_type_name = lambda: None
        env.hitter_ball_planner = SimpleNamespace(
            plan_command=lambda *args, **kwargs: planner_calls.append(
                (args, kwargs)
            )
            or SimpleNamespace(time_to_strike=0.7)
        )
        snapshot = BallEstimateSnapshot(
            track_epoch=1,
            generation=3,
            source_frame=10,
            source_time_s=0.0,
            received_monotonic_s=5.0,
            position_w=np.asarray([0.5, 0.0, 0.8], dtype=np.float64),
            velocity_w=np.asarray([-3.0, 0.0, 0.0], dtype=np.float64),
            base_position_w=np.asarray([1.0, 2.0, 0.5], dtype=np.float64),
            base_quaternion_xyzw=np.asarray(
                [0.0, 0.0, 0.0, 1.0],
                dtype=np.float64,
            ),
            base_valid=True,
            visible=True,
            ready=True,
        )

        env._plan_hitter_snapshot(snapshot)

        self.assertEqual(len(planner_calls), 1)
        args, kwargs = planner_calls[0]
        np.testing.assert_allclose(
            kwargs["current_base_xy_w"],
            [1.0, 2.0, 0.5],
        )
        np.testing.assert_allclose(kwargs["base_forward_xy_w"], [1.0, 0.0])


if __name__ == "__main__":
    unittest.main()
