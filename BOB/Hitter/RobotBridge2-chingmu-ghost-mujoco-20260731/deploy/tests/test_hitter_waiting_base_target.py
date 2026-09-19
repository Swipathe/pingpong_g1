from __future__ import annotations

import unittest
from types import SimpleNamespace

import numpy as np

from envs.hitter import HitterEnv


class HitterWaitingBaseTargetTests(unittest.TestCase):
    def test_last_planner_target_mode_starts_from_current_base_pose(self):
        env = HitterEnv.__new__(HitterEnv)
        env.motion_cfg = {
            "waiting_base_target_mode": "last_planner_target",
            "waiting_base_target_xy_w": [-0.4, 0.0],
        }
        env.simulator = SimpleNamespace(
            base_pose_valid=True,
            root_trans_world=np.asarray(
                [0.31, -0.14, 0.79],
                dtype=np.float32,
            ),
        )

        try:
            env._capture_hitter_waiting_base_target()
        except ValueError as exc:
            self.fail(f"last_planner_target rejected current-pose fallback: {exc}")

        np.testing.assert_allclose(
            env.hitter_waiting_base_target_pos_w,
            [0.31, -0.14, 0.79],
            rtol=0.0,
            atol=1.0e-6,
        )

    def test_last_planner_target_mode_updates_waiting_target_from_command(self):
        env = HitterEnv.__new__(HitterEnv)
        env.motion_cfg = {
            "waiting_base_target_mode": "last_planner_target",
            "ball_planner": {"target_base_height_w": 0.78},
        }
        env.hitter_command_lifecycle = SimpleNamespace(
            recovery_duration_s=0.95,
        )
        env.hitter_waiting_base_target_pos_w = np.asarray(
            [0.27, -0.18, 0.79],
            dtype=np.float32,
        )
        command = SimpleNamespace(
            strike_type="forehand",
            p_base_target_xy=np.asarray([-0.61, 0.24]),
            v_racket_target_w=np.asarray([3.0, -0.2, 1.5]),
            time_to_strike=0.80,
            strike_plan=SimpleNamespace(
                p_racket_target=np.asarray([-0.21, -0.26, 1.05]),
                v_ball_in=np.asarray([-4.0, 0.1, -1.0]),
            ),
        )

        env._copy_hitter_command(command)

        np.testing.assert_allclose(
            env.hitter_waiting_base_target_pos_w,
            [-0.61, 0.24, 0.79],
            rtol=0.0,
            atol=1.0e-6,
        )

    def test_capture_current_mode_holds_reset_pose_and_corrects_drift(self):
        env = HitterEnv.__new__(HitterEnv)
        env.motion_cfg = {
            "waiting_base_target_mode": "capture_current",
            "waiting_base_target_xy_w": [-0.4, 0.0],
        }
        reset_pose_w = np.asarray(
            [0.27, -0.18, 0.79],
            dtype=np.float32,
        )
        env.simulator = SimpleNamespace(
            base_pose_valid=True,
            root_trans_world=reset_pose_w.copy(),
        )

        env._capture_hitter_waiting_base_target()

        np.testing.assert_allclose(
            env.hitter_waiting_base_target_pos_w,
            [0.27, -0.18, 0.79],
            rtol=0.0,
            atol=1.0e-6,
        )
        identity_quat_xyzw = np.asarray(
            [0.0, 0.0, 0.0, 1.0],
            dtype=np.float32,
        )
        moved_pose_w = np.asarray(
            [0.37, -0.13, 0.79],
            dtype=np.float32,
        )
        np.testing.assert_allclose(
            env._hitter_waiting_base_target_pos_b(
                moved_pose_w,
                identity_quat_xyzw,
            ),
            [-0.10, -0.05, 0.0],
            rtol=0.0,
            atol=1.0e-6,
        )
