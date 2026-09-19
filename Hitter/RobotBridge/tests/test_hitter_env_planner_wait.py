import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

DEPLOY_DIR = Path(__file__).resolve().parents[1] / "deploy"
sys.path.insert(0, str(DEPLOY_DIR))

from envs.hitter import HitterEnv


class _PlannerOnlyFailureEnv(HitterEnv):
    def __init__(self):
        self.planner_only = True
        self.hitter_waiting_for_planner_arm = False
        self.hitter_strike_elapsed_s = 0.42
        self.hitter_command_initialized = True
        self.hitter_ball_sequence_needs_reset = False

    def _plan_hitter_command_from_ball(self):
        raise RuntimeError(
            "Ball planner failed in planner_only mode: "
            "ball trajectory does not cross hit plane x=0.000 within prediction horizon"
        )

    def _waiting_for_real_ball_estimator(self):
        return False

    def _apply_hitter_ball_planner_command(self, command, *, reset_elapsed):
        raise AssertionError("invalid planner command should not be applied")


def _planner_command(
    *,
    strike_type="backhand",
    time_to_strike=0.75,
    racket_pos=(0.0, 0.35, 0.9),
    ball_in_vel=(-2.0, 0.2, -0.6),
    racket_vel=(3.0, -0.3, 1.0),
):
    return SimpleNamespace(
        strike_type=strike_type,
        time_to_strike=time_to_strike,
        strike_plan=SimpleNamespace(
            p_racket_target=np.asarray(racket_pos, dtype=np.float64),
            v_ball_in=np.asarray(ball_in_vel, dtype=np.float64),
            v_ball_out=np.asarray([4.0, 0.0, 3.0], dtype=np.float64),
        ),
        v_racket_target_w=np.asarray(racket_vel, dtype=np.float64),
        p_base_target_xy=np.asarray([-0.4, 0.15], dtype=np.float64),
    )


class _RejectedPlannerCommandEnv(HitterEnv):
    def __init__(self, command):
        self.command = command
        self.motion_cfg = {
            "ball_planner": {
                "filter_real_ball_to_wbc_command": True,
                "real_ball_min_speed": 1.5,
                "real_ball_min_incoming_x_speed": 0.5,
                "real_ball_forehand_racket_y_range": [-0.7, 0.0],
                "real_ball_backhand_racket_y_range": [0.0, 0.7],
                "real_ball_racket_z_range_w": [0.8, 1.0],
            }
        }
        self.planner_only = True
        self.arm_planner_command_on_time = True
        self.planner_arm_time_to_strike_range = (0.55, 0.95)
        self.hitter_waiting_for_planner_arm = False
        self.hitter_strike_elapsed_s = 0.42
        self.hitter_command_initialized = True
        self.hitter_has_valid_command = True
        self.hitter_ball_sequence_needs_reset = False

    def _plan_hitter_command_from_ball(self):
        return self.command

    def _waiting_for_real_ball_estimator(self):
        return False

    def _apply_hitter_ball_planner_command(self, command, *, reset_elapsed):
        raise AssertionError("rejected planner command should not be applied")


class _LatePlannerCommandEnv(_RejectedPlannerCommandEnv):
    def __init__(self):
        super().__init__(_planner_command(time_to_strike=0.12))
        self.reset_count = 0
        self.simulator = SimpleNamespace(reset_ball_state_estimator=self._reset_estimator)

    def _reset_estimator(self):
        self.reset_count += 1


class HitterEnvPlannerWaitTest(unittest.TestCase):
    def test_planner_only_waits_for_next_ball_when_current_trajectory_is_invalid(self):
        env = _PlannerOnlyFailureEnv()

        handled = HitterEnv._sample_hitter_command_from_ball_planner(env)

        self.assertTrue(handled)
        self.assertTrue(env.hitter_waiting_for_planner_arm)
        self.assertEqual(env.hitter_strike_elapsed_s, 0.0)
        self.assertFalse(env.hitter_command_initialized)
        self.assertTrue(env.hitter_ball_sequence_needs_reset)

    def test_real_ball_filter_rejects_impossible_racket_height(self):
        env = _RejectedPlannerCommandEnv(_planner_command(racket_pos=(0.0, 0.35, 0.2)))

        valid, reason = HitterEnv._real_ball_command_in_live_range(env, env.command)

        self.assertFalse(valid)
        self.assertIn("racket_z", reason)

    def test_planner_waits_for_next_ball_when_live_command_is_outside_workspace(self):
        env = _RejectedPlannerCommandEnv(_planner_command(racket_pos=(0.0, -1.4, -2.8)))

        handled = HitterEnv._sample_hitter_command_from_ball_planner(env)

        self.assertTrue(handled)
        self.assertTrue(env.hitter_waiting_for_planner_arm)
        self.assertEqual(env.hitter_strike_elapsed_s, 0.0)
        self.assertFalse(env.hitter_command_initialized)
        self.assertFalse(env.hitter_has_valid_command)
        self.assertTrue(env.hitter_ball_sequence_needs_reset)

    def test_too_late_real_ball_command_resets_estimator(self):
        env = _LatePlannerCommandEnv()

        handled = HitterEnv._sample_hitter_command_from_ball_planner(env)

        self.assertTrue(handled)
        self.assertEqual(env.reset_count, 1)
        self.assertTrue(env.hitter_waiting_for_planner_arm)
        self.assertFalse(env.hitter_command_initialized)
        self.assertFalse(env.hitter_has_valid_command)
        self.assertTrue(env.hitter_ball_sequence_needs_reset)

    def test_real_ball_filter_rejects_slow_or_wrong_direction_ball(self):
        env = _RejectedPlannerCommandEnv(_planner_command(ball_in_vel=(0.1, 0.1, 0.0)))

        valid, reason = HitterEnv._real_ball_command_in_live_range(env, env.command)

        self.assertFalse(valid)
        self.assertIn("speed", reason)

    def test_real_ball_filter_accepts_plausible_incoming_command(self):
        env = _RejectedPlannerCommandEnv(_planner_command())

        valid, reason = HitterEnv._real_ball_command_in_live_range(env, env.command)

        self.assertTrue(valid, reason)

    def test_planner_only_waits_without_fake_command_when_no_ball_is_ready(self):
        env = object.__new__(HitterEnv)
        env.use_ball_planner = True
        env.planner_only = True
        env.wait_for_ball_without_crash = True
        env.hitter_command_initialized = False
        env.hitter_waiting_for_planner_arm = False
        env.hitter_strike_elapsed_s = 0.42

        def no_command_available():
            return False

        env._sample_hitter_command_from_ball_planner = no_command_available

        HitterEnv._sample_hitter_command(env)

        self.assertFalse(env.hitter_command_initialized)
        self.assertTrue(env.hitter_waiting_for_planner_arm)
        self.assertEqual(env.hitter_strike_elapsed_s, 0.0)

    def test_observation_keeps_last_command_when_planner_temporarily_unavailable(self):
        class Simulator:
            root_trans_world = np.array([0.1, 0.2, 0.8], dtype=np.float32)
            root_quat_world = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)

            def update_obs(self):
                return {
                    "base_ang_vel": np.array([0.01, 0.02, 0.03], dtype=np.float32),
                    "projected_gravity": np.array([0.0, 0.0, -1.0], dtype=np.float32),
                    "dof_pos": np.zeros(29, dtype=np.float32),
                    "dof_vel": np.zeros(29, dtype=np.float32),
                }

        env = object.__new__(HitterEnv)
        env.cfg = SimpleNamespace(control=SimpleNamespace(obs_clip_value=None))
        env.simulator = Simulator()
        env.first_obs_received = True
        env.base_ang_vel = np.zeros((1, 3), dtype=np.float32)
        env.projected_gravity = np.zeros((1, 3), dtype=np.float32)
        env.dof_pos = np.zeros((1, 29), dtype=np.float32)
        env.dof_vel = np.zeros((1, 29), dtype=np.float32)
        env.policy_default_joint_pos = np.zeros(29, dtype=np.float32)
        env.prev_policy_action = np.zeros(29, dtype=np.float32)
        env._sim_to_policy_adapter = None
        env._sim_to_policy = np.arange(29, dtype=np.int32)
        env._policy_dim = 29

        env.use_ball_planner = True
        env.hitter_command_initialized = False
        env.hitter_base_target_xy_w = np.array([1.2, -0.3], dtype=np.float32)
        env.hitter_base_target_z_w = np.float32(0.9)
        env.hitter_racket_target_pos_w_fixed = np.array([0.7, 0.4, 1.1], dtype=np.float32)
        env.hitter_racket_target_vel_w = np.array([2.8, 0.4, 1.1], dtype=np.float32)
        env.hitter_strike_time_s = 0.7
        env.hitter_strike_elapsed_s = 0.2
        env._sample_hitter_command = lambda: None

        HitterEnv._compute_hitter_observation(env)
        obs = env.obs_buf_dict["obs"].reshape(-1)

        self.assertFalse(env.hitter_command_initialized)
        self.assertFalse(np.allclose(obs, 0.0))
        np.testing.assert_allclose(obs[8:11], np.array([1.1, -0.5, 0.1], dtype=np.float32), atol=1.0e-6)
        np.testing.assert_allclose(obs[11:14], np.array([0.6, 0.2, 0.3], dtype=np.float32), atol=1.0e-6)
        np.testing.assert_allclose(obs[14:17], np.array([2.8, 0.4, 1.1], dtype=np.float32), atol=1.0e-6)
        np.testing.assert_allclose(obs[17:18], np.array([0.5], dtype=np.float32), atol=1.0e-6)

    def test_finished_planner_command_invalidates_policy_until_next_ball(self):
        env = object.__new__(HitterEnv)
        env.hitter_mode = True
        env.hitter_command_initialized = True
        env.hitter_has_valid_command = True
        env.use_ball_planner = True
        env.update_planner_command_while_armed = False
        env.arm_planner_command_on_time = True
        env.hitter_strike_elapsed_s = 1.0
        env.hitter_strike_duration_s = 1.0
        env.playback_speed = 1.0
        env.simulator = SimpleNamespace(high_dt=0.02)
        env.hitter_ball_sequence_needs_reset = False
        env.hitter_waiting_for_planner_arm = False
        env._sample_hitter_command = lambda: None

        HitterEnv._update_hitter_command(env)

        self.assertFalse(env.hitter_command_initialized)
        self.assertFalse(env.hitter_has_valid_command)
        self.assertTrue(env.hitter_waiting_for_planner_arm)
        self.assertTrue(env.hitter_ball_sequence_needs_reset)


if __name__ == "__main__":
    unittest.main()
