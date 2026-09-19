from __future__ import annotations

import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from envs.hitter import HitterEnv
from utils.hitter_realtime import CommandPhase
from utils.hitter_runtime_capabilities import (
    HitterRuntimeCapabilities,
    LoopPacing,
    PlannerFeed,
)


HYBRID_CAPABILITIES = HitterRuntimeCapabilities(
    planner_feed=PlannerFeed.SNAPSHOT_STREAM,
    loop_pacing=LoopPacing.SIMULATOR,
)


class RecordingLifecycle:
    def __init__(self, events):
        self.events = events
        self.phase = CommandPhase.ARMED
        self.active_result = object()
        self.cached_result = None
        self.command_end_deadline_s = None
        self.maximum_policy_tts = 0.92

    def advance(self, now):
        self.events.append(("advance", float(now)))
        return self.phase

    def policy_tts(self, *, now):
        self.events.append(("policy_tts", float(now)))
        return 10.9 - float(now)


class SourceWithMisleadingBase:
    def register_hitter_ball_listener(self, listener):
        return lambda: None

    def reset_ball_state_estimator(self):
        return 0

    def state(self):
        return SimpleNamespace(
            track_epoch=1,
            generation=1,
            position_w=np.asarray([9.0, 9.0, 9.0], dtype=np.float64),
            velocity_w=np.asarray([-3.0, 0.0, 0.0], dtype=np.float64),
            base_position_w=np.asarray([99.0, 99.0, 99.0], dtype=np.float64),
            base_quaternion_xyzw=np.asarray(
                [0.0, 0.0, 1.0, 0.0],
                dtype=np.float64,
            ),
            visible=True,
            ready=True,
        )


class MinimalMujocoRobotSimulator:
    is_real = False
    hitter_runtime_capabilities = HYBRID_CAPABILITIES

    def __init__(self):
        self.hitter_ball_source = SourceWithMisleadingBase()
        self.root_trans_world = np.asarray(
            [1.0, 2.0, 0.5],
            dtype=np.float32,
        )
        self.root_quat_world = np.asarray(
            [0.0, 0.0, 0.0, 1.0],
            dtype=np.float32,
        )

    def update_obs(self):
        return {}


def build_active_hil_env(events) -> HitterEnv:
    env = HitterEnv.__new__(HitterEnv)
    env.simulator = MinimalMujocoRobotSimulator()
    env.hitter_ball_source = env.simulator.hitter_ball_source
    env.cfg = SimpleNamespace(
        control=SimpleNamespace(obs_clip_value=1000.0)
    )
    env.first_obs_received = True
    env.policy_default_joint_pos = np.zeros(29, dtype=np.float32)
    env.prev_policy_action = np.zeros(29, dtype=np.float32)
    env.dof_pos = np.zeros((1, 29), dtype=np.float32)
    env.dof_vel = np.zeros((1, 29), dtype=np.float32)
    env.base_ang_vel = np.zeros((1, 3), dtype=np.float32)
    env.projected_gravity = np.asarray(
        [[0.0, 0.0, -1.0]],
        dtype=np.float32,
    )
    env._policy_dim = 29
    env._sim_to_policy = np.arange(29, dtype=np.int32)
    env._sim_to_policy_adapter = None
    env.hitter_base_target_xy_w = np.asarray(
        [3.0, 5.0],
        dtype=np.float32,
    )
    env.hitter_base_target_z_w = 0.5
    env.hitter_racket_target_pos_w_fixed = np.asarray(
        [4.0, 6.0, 1.5],
        dtype=np.float32,
    )
    env.hitter_racket_target_vel_w = np.asarray(
        [0.25, -0.5, 0.75],
        dtype=np.float32,
    )
    env.hitter_command_lifecycle = RecordingLifecycle(events)
    env.hitter_command_initialized = True
    env.hitter_planner_worker = None
    env.hitter_ball_sequence_needs_reset = False
    env._hitter_last_logged_phase = CommandPhase.ARMED
    return env


class HitterChingMuHilObservationTests(unittest.TestCase):
    def test_hybrid_observation_shape_stays_104_and_uses_mujoco_robot_base(self):
        events = []
        env = build_active_hil_env(events)

        with patch(
            "envs.hitter.time.monotonic",
            side_effect=[10.0, 10.2],
        ):
            observation = env.refresh_policy_observation()["obs"]

        self.assertEqual(observation.shape, (1, 104))
        self.assertEqual(
            events,
            [
                ("advance", 10.0),
                ("policy_tts", 10.2),
            ],
        )
        np.testing.assert_allclose(
            observation[0, 6:8],
            [1.0, 0.0],
            atol=1.0e-6,
        )
        np.testing.assert_allclose(
            observation[0, 8:10],
            [2.0, 3.0],
            atol=1.0e-6,
        )
        np.testing.assert_allclose(
            observation[0, 10:13],
            [3.0, 4.0, 1.0],
            atol=1.0e-6,
        )
        self.assertAlmostEqual(float(observation[0, 16]), 0.7, places=6)


if __name__ == "__main__":
    unittest.main()
