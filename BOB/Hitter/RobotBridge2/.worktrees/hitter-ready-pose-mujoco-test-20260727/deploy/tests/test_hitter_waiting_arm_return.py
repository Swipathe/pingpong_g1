from __future__ import annotations

import copy
import unittest
from types import SimpleNamespace

import numpy as np

from envs.hitter import HitterEnv
from utils.hitter_realtime import CommandPhase


ARM_NAMES = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
)
LOWER_BODY_NAMES = tuple(f"lower_body_{index}" for index in range(15))
ARM_TARGET = np.asarray(
    [0.2, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0, -0.5946002267035698, -0.2, 0.0, 0.6, 0.0, 0.0, 0.0],
    dtype=np.float32,
)


def valid_motion_config(*, enabled: bool = True) -> dict:
    return {"waiting_arm_return": {"enabled": enabled, "duration_s": 0.9,
            "target_joint_pos": dict(zip(ARM_NAMES, ARM_TARGET.tolist()))}}


class HitterWaitingArmReturnTests(unittest.TestCase):
    def make_env(self, motion_cfg=None, *, high_dt: float = 0.1, dof_names=None):
        env = HitterEnv.__new__(HitterEnv)
        env.motion_cfg = valid_motion_config() if motion_cfg is None else motion_cfg
        env.simulator = SimpleNamespace(
            dof_names=list(LOWER_BODY_NAMES + ARM_NAMES if dof_names is None else dof_names), high_dt=high_dt)
        env.dof_pos = np.linspace(-1.4, 1.4, 29, dtype=np.float32).reshape(1, 29)
        env.hitter_command_lifecycle = SimpleNamespace(phase=CommandPhase.WAITING)
        return env

    @staticmethod
    def dispatch(env: HitterEnv, policy_target: np.ndarray) -> np.ndarray:
        output = env._apply_waiting_arm_return(policy_target)
        env._last_dispatched_sim_pd_target = output.copy()
        return output

    def test_waiting_smoothstep_changes_only_sim_arm_indices(self):
        env = self.make_env()
        env._configure_waiting_arm_return()
        captured_start = np.arange(29, dtype=np.float32) - 14.0
        policy_target = np.arange(29, dtype=np.float32) + 50.0
        env._last_dispatched_sim_pd_target = captured_start.copy()

        first = self.dispatch(env, policy_target)
        np.testing.assert_array_equal(first[:15], policy_target[:15])
        np.testing.assert_array_equal(first[15:], captured_start[15:])

        self.dispatch(env, policy_target)
        self.dispatch(env, policy_target)
        intermediate = self.dispatch(env, policy_target)
        u = 0.3 / 0.9
        blend = u * u * (3.0 - 2.0 * u)
        np.testing.assert_array_equal(intermediate[:15], policy_target[:15])
        np.testing.assert_allclose(
            intermediate[15:],
            (1.0 - blend) * captured_start[15:] + blend * ARM_TARGET,
            rtol=0.0,
            atol=1e-7,
        )

        for _ in range(5):
            self.dispatch(env, policy_target)
        complete = self.dispatch(env, policy_target)
        held = self.dispatch(env, policy_target + 100.0)
        np.testing.assert_array_equal(complete[:15], policy_target[:15])
        np.testing.assert_array_equal(complete[15:], ARM_TARGET)
        np.testing.assert_array_equal(held[15:], ARM_TARGET)

    def test_armed_cancels_but_recovery_restarts_and_waiting_tracking_continue(self):
        env = self.make_env()
        env._configure_waiting_arm_return()
        waiting_policy = np.arange(29, dtype=np.float32)
        env._last_dispatched_sim_pd_target = np.arange(29, dtype=np.float32) - 20.0
        first = self.dispatch(env, waiting_policy)
        captured_start = first[15:].copy()

        env.hitter_command_lifecycle.phase = CommandPhase.TRACKING
        tracking_policy = waiting_policy + 30.0
        tracking = self.dispatch(env, tracking_policy)
        u = 0.1 / 0.9
        blend = u * u * (3.0 - 2.0 * u)
        np.testing.assert_array_equal(tracking[:15], tracking_policy[:15])
        np.testing.assert_allclose(tracking[15:], (1.0 - blend) * captured_start + blend * ARM_TARGET, rtol=0.0, atol=1e-7)

        armed_policy = tracking_policy + 40.0
        env.hitter_command_lifecycle.phase = CommandPhase.ARMED
        armed = self.dispatch(env, armed_policy)
        np.testing.assert_array_equal(armed, armed_policy)
        self.assertIsNone(env._waiting_arm_return_start_arm)
        self.assertEqual(env._waiting_arm_return_elapsed_s, 0.0)

        recovery_policy = armed_policy + 40.0
        env.hitter_command_lifecycle.phase = CommandPhase.RECOVERY
        recovery_start = self.dispatch(env, recovery_policy)
        np.testing.assert_array_equal(recovery_start[:15], recovery_policy[:15])
        np.testing.assert_array_equal(recovery_start[15:], armed_policy[15:])
        self.assertIsNotNone(env._waiting_arm_return_start_arm)

        recovery_next_policy = recovery_policy + 10.0
        recovery_next = self.dispatch(env, recovery_next_policy)
        u = 0.1 / 0.9
        blend = u * u * (3.0 - 2.0 * u)
        np.testing.assert_array_equal(recovery_next[:15], recovery_next_policy[:15])
        np.testing.assert_allclose(
            recovery_next[15:],
            (1.0 - blend) * armed_policy[15:] + blend * ARM_TARGET,
            rtol=0.0,
            atol=1e-7,
        )

        waiting_policy = recovery_next_policy + 10.0
        env.hitter_command_lifecycle.phase = CommandPhase.WAITING
        waiting = self.dispatch(env, waiting_policy)
        u = 0.2 / 0.9
        blend = u * u * (3.0 - 2.0 * u)
        np.testing.assert_allclose(
            waiting[15:],
            (1.0 - blend) * armed_policy[15:] + blend * ARM_TARGET,
            rtol=0.0,
            atol=1e-7,
        )

        env.hitter_command_lifecycle.phase = CommandPhase.TRACKING
        for _ in range(7):
            tracking = self.dispatch(env, waiting_policy + 20.0)
        np.testing.assert_array_equal(tracking[15:], ARM_TARGET)

    def test_startup_and_reset_capture_current_dof_position(self):
        env = self.make_env()
        env._configure_waiting_arm_return()
        policy_target = np.arange(29, dtype=np.float32) + 10.0
        startup = self.dispatch(env, policy_target)
        np.testing.assert_array_equal(startup[:15], policy_target[:15])
        np.testing.assert_array_equal(startup[15:], env.dof_pos[0, 15:])

        one_dimensional = self.make_env()
        one_dimensional.dof_pos = one_dimensional.dof_pos[0].copy()
        one_dimensional._configure_waiting_arm_return()
        one_dimensional_startup = self.dispatch(one_dimensional, policy_target)
        np.testing.assert_array_equal(
            one_dimensional_startup[15:],
            one_dimensional.dof_pos[15:],
        )

        for invalid_dof_pos in (
            np.zeros((2, 29), dtype=np.float32),
            np.zeros((1, 28), dtype=np.float32),
            np.full((1, 29), np.nan, dtype=np.float32),
        ):
            invalid = self.make_env()
            invalid.dof_pos = invalid_dof_pos
            invalid._configure_waiting_arm_return()
            with self.assertRaises(ValueError):
                invalid._apply_waiting_arm_return(policy_target)

        env._last_dispatched_sim_pd_target = np.arange(29, dtype=np.float32)
        env._waiting_arm_return_start_arm = np.ones(14, dtype=np.float32)
        env._waiting_arm_return_elapsed_s = 0.3
        env._policy_dim = 29
        env._init_hitter_command_state = lambda: None
        env._reset_hitter_lifecycle_state = lambda: None
        env._prepare_hitter_reset_state()
        self.assertIsNone(env._last_dispatched_sim_pd_target)
        self.assertIsNone(env._waiting_arm_return_start_arm)
        self.assertEqual(env._waiting_arm_return_elapsed_s, 0.0)

    def test_calibrating_reset_clears_waiting_transition_before_next_capture(self):
        env = self.make_env()
        env._configure_waiting_arm_return()
        env._last_dispatched_sim_pd_target = np.arange(29, dtype=np.float32) - 50.0
        env._waiting_arm_return_start_arm = np.arange(14, dtype=np.float32) + 100.0
        env._waiting_arm_return_elapsed_s = 0.4
        calibrate_calls = []
        env.simulator.calibrate = lambda refresh, reference: calibrate_calls.append(
            (refresh, reference)
        )
        env.simulator.get_state = lambda: None
        env.simulator.base_pose_valid = True
        env.simulator.root_trans_world = np.asarray([0.0, 0.0, 0.8], dtype=np.float32)
        env.episode_length_buf = np.ones(1, dtype=np.float64)
        env.action = np.ones((1, 29), dtype=np.float32)
        env.history_handler = SimpleNamespace(reset=lambda: None)
        env.collected_traj = {"stale": True}
        env.init_ref_dof_pos = None

        env._reset_envs(True)

        self.assertEqual(calibrate_calls, [(True, None)])
        self.assertIsNone(env._last_dispatched_sim_pd_target)
        self.assertIsNone(env._waiting_arm_return_start_arm)
        self.assertEqual(env._waiting_arm_return_elapsed_s, 0.0)
        post_reset_dof_pos = np.arange(29, dtype=np.float32).reshape(1, 29) + 200.0
        env.dof_pos = post_reset_dof_pos
        next_output = self.dispatch(env, np.arange(29, dtype=np.float32))
        np.testing.assert_array_equal(next_output[15:], post_reset_dof_pos[0, 15:])

    def test_waiting_return_configuration_fails_closed(self):
        valid = valid_motion_config()

        def assert_invalid(raw, *, high_dt=0.1, dof_names=None):
            env = self.make_env({"waiting_arm_return": raw}, high_dt=high_dt, dof_names=dof_names)
            with self.assertRaises((TypeError, ValueError)):
                env._configure_waiting_arm_return()

        assert_invalid([])
        assert_invalid({"enabled": True, "duration_s": 0.9, "target_joint_pos": {}, "extra": 1})
        assert_invalid({"enabled": True, "target_joint_pos": valid["waiting_arm_return"]["target_joint_pos"]})
        for enabled in (1, "true", None):
            raw = copy.deepcopy(valid["waiting_arm_return"])
            raw["enabled"] = enabled
            assert_invalid(raw)
        for duration in (True, "0.9", float("nan"), float("inf"), 0.0, -0.1):
            raw = copy.deepcopy(valid["waiting_arm_return"])
            raw["duration_s"] = duration
            assert_invalid(raw)
        for target_value in (True, "0.2", float("nan"), float("inf")):
            raw = copy.deepcopy(valid["waiting_arm_return"])
            raw["target_joint_pos"][ARM_NAMES[0]] = target_value
            assert_invalid(raw)
        raw = copy.deepcopy(valid["waiting_arm_return"])
        raw["target_joint_pos"].pop(ARM_NAMES[0])
        assert_invalid(raw)
        raw = copy.deepcopy(valid["waiting_arm_return"])
        raw["target_joint_pos"]["unexpected_joint"] = 0.0
        assert_invalid(raw)
        assert_invalid(valid["waiting_arm_return"], dof_names=LOWER_BODY_NAMES + ARM_NAMES[1:])
        assert_invalid(valid["waiting_arm_return"], dof_names=("extra",) + LOWER_BODY_NAMES + ARM_NAMES)
        for high_dt in (True, "0.1", float("nan"), float("inf"), 0.0, -0.1):
            assert_invalid(valid["waiting_arm_return"], high_dt=high_dt)

    def test_missing_or_disabled_block_preserves_policy_target(self):
        policy_target = np.arange(29, dtype=np.float32) - 3.0
        missing = self.make_env({})
        missing._configure_waiting_arm_return()
        self.assertFalse(missing._waiting_arm_return_enabled)
        np.testing.assert_array_equal(missing._apply_waiting_arm_return(policy_target), policy_target)

        disabled = self.make_env(valid_motion_config(enabled=False))
        disabled._configure_waiting_arm_return()
        self.assertFalse(disabled._waiting_arm_return_enabled)
        np.testing.assert_array_equal(disabled._apply_waiting_arm_return(policy_target), policy_target)


if __name__ == "__main__":
    unittest.main()
