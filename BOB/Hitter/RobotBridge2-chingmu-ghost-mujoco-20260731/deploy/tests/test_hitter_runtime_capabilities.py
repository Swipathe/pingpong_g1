from __future__ import annotations

import unittest
from dataclasses import FrozenInstanceError
from types import SimpleNamespace
from unittest.mock import patch

from envs.hitter import HitterEnv
from simulator.mujoco import Mujoco
from simulator.real_world import RealWorld
from utils.hitter_runtime_capabilities import (
    HitterRuntimeCapabilities,
    LoopPacing,
    PlannerFeed,
)


HYBRID_CAPABILITIES = HitterRuntimeCapabilities(
    planner_feed=PlannerFeed.SNAPSHOT_STREAM,
    loop_pacing=LoopPacing.SIMULATOR,
)


class HitterRuntimeProfileTests(unittest.TestCase):
    def test_mujoco_profile_is_inline_state_with_simulator_pacing(self):
        capabilities = Mujoco.hitter_runtime_capabilities

        self.assertEqual(capabilities.planner_feed, PlannerFeed.INLINE_STATE)
        self.assertEqual(capabilities.loop_pacing, LoopPacing.SIMULATOR)
        self.assertFalse(capabilities.uses_snapshot_stream)
        self.assertFalse(capabilities.uses_agent_pacing)

    def test_real_world_profile_is_snapshot_stream_with_agent_pacing(self):
        capabilities = RealWorld.hitter_runtime_capabilities

        self.assertEqual(
            capabilities.planner_feed,
            PlannerFeed.SNAPSHOT_STREAM,
        )
        self.assertEqual(capabilities.loop_pacing, LoopPacing.AGENT)
        self.assertTrue(capabilities.uses_snapshot_stream)
        self.assertTrue(capabilities.uses_agent_pacing)

    def test_hybrid_profile_is_snapshot_stream_with_simulator_pacing(self):
        self.assertEqual(
            HYBRID_CAPABILITIES.planner_feed,
            PlannerFeed.SNAPSHOT_STREAM,
        )
        self.assertEqual(
            HYBRID_CAPABILITIES.loop_pacing,
            LoopPacing.SIMULATOR,
        )
        self.assertTrue(HYBRID_CAPABILITIES.uses_snapshot_stream)
        self.assertFalse(HYBRID_CAPABILITIES.uses_agent_pacing)

    def test_capability_profile_is_immutable(self):
        with self.assertRaises(FrozenInstanceError):
            HYBRID_CAPABILITIES.loop_pacing = LoopPacing.AGENT


class HitterEnvironmentCapabilityTests(unittest.TestCase):
    def test_hybrid_initializes_latest_only_worker_despite_false_is_real(self):
        events = []
        worker = object()
        unregister = object()

        def register_listener(callback):
            events.append(("listener", callback))
            return unregister

        env = HitterEnv.__new__(HitterEnv)
        env.simulator = SimpleNamespace(
            is_real=False,
            hitter_runtime_capabilities=HYBRID_CAPABILITIES,
            register_hitter_ball_listener=register_listener,
            reset_ball_state_estimator=lambda: None,
            state=lambda: SimpleNamespace(track_epoch=0, generation=0),
        )
        env.hitter_planner_worker = None
        env._unregister_ball_listener = None

        with patch(
            "envs.hitter.LatestOnlyPlannerWorker",
            return_value=worker,
        ) as worker_type:
            env._initialize_hitter_realtime_runtime()

        self.assertIs(env.hitter_planner_worker, worker)
        self.assertIs(env._unregister_ball_listener, unregister)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0][0], "listener")
        self.assertEqual(
            events[0][1].__func__,
            env._submit_hitter_planner_snapshot.__func__,
        )
        worker_type.assert_called_once()

    def test_inline_profile_stays_synchronous_despite_true_is_real(self):
        env = HitterEnv.__new__(HitterEnv)
        env.simulator = SimpleNamespace(
            is_real=True,
            hitter_runtime_capabilities=Mujoco.hitter_runtime_capabilities,
        )
        env.hitter_planner_worker = object()

        env._initialize_hitter_realtime_runtime()

        self.assertIsNone(env.hitter_planner_worker)
        self.assertTrue(env._hitter_uses_inline_state())
        self.assertFalse(env._hitter_uses_snapshot_stream())

    def test_snapshot_reset_uses_estimator_epoch_despite_false_is_real(self):
        events = []
        lifecycle = SimpleNamespace(phase="waiting")
        simulator = SimpleNamespace(
            is_real=False,
            hitter_runtime_capabilities=HYBRID_CAPABILITIES,
            register_hitter_ball_listener=lambda _listener: None,
            state=lambda: SimpleNamespace(
                track_epoch=7,
                generation=11,
            ),
            reset_ball_state_estimator=lambda: events.append("reset"),
        )
        env = HitterEnv.__new__(HitterEnv)
        env.simulator = simulator
        env._new_hitter_command_lifecycle = lambda: lifecycle
        env._hitter_sync_track_epoch = 3

        env._reset_hitter_lifecycle_state()

        self.assertEqual(events, ["reset"])
        self.assertEqual(env._hitter_minimum_track_epoch, 7)
        self.assertEqual(env._hitter_minimum_generation, 12)
        self.assertEqual(env._hitter_sync_track_epoch, 3)

    def test_inline_reset_uses_sync_epoch_despite_true_is_real(self):
        events = []
        lifecycle = SimpleNamespace(phase="waiting")
        simulator = SimpleNamespace(
            is_real=True,
            hitter_runtime_capabilities=Mujoco.hitter_runtime_capabilities,
            ball_track_epoch=7,
            ball_snapshot_generation=11,
            reset_ball_state_estimator=lambda: events.append("reset"),
        )
        env = HitterEnv.__new__(HitterEnv)
        env.simulator = simulator
        env._new_hitter_command_lifecycle = lambda: lifecycle
        env._hitter_sync_track_epoch = 3

        env._reset_hitter_lifecycle_state()

        self.assertEqual(events, [])
        self.assertEqual(env._hitter_sync_track_epoch, 4)
        self.assertEqual(env._hitter_minimum_track_epoch, 4)
        self.assertEqual(env._hitter_minimum_generation, 0)


if __name__ == "__main__":
    unittest.main()
