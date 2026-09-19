import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from hydra import compose, initialize


DEPLOY_DIR = Path(__file__).resolve().parents[1] / "deploy"
sys.path.insert(0, str(DEPLOY_DIR))

from agents.loco_hitter_agent import LocoHitterAgent, LocoHitterInterpManager
from envs.loco_hitter import LocoHitterEnv


class LocoHitterSwitchTest(unittest.TestCase):
    def make_env(self):
        env = object.__new__(LocoHitterEnv)
        env.policy_mode = "locomotion"
        env.pending_policy_switch = None
        env.hitter_command_initialized = False
        env.hitter_waiting_for_planner_arm = True
        env.hitter_ball_sequence_needs_reset = True
        env.hitter_strike_elapsed_s = 0.0
        env.hitter_strike_time_s = 0.8
        env.hitter_run_steps = 0
        env.hitter_return_steps = 20
        env.switch_back_after_hitter = True
        env.hitter_return_after_strike_s = 0.15
        env.hitter_max_duration_s = 2.4
        env.motion_finished = False
        env.use_ball_planner = True
        env.arm_planner_command_on_time = True
        env.motion_cfg = {"ball_planner": {"target_base_height_w": 0.793}}
        env.loco_default_angles = np.array([0.1, 0.2, 0.3], dtype=np.float32)
        env.hitter_default_angles = np.array([1.0, 1.1, 1.2], dtype=np.float32)
        env.loco_kps = np.array([10.0, 20.0, 30.0], dtype=np.float32)
        env.loco_kds = np.array([1.0, 2.0, 3.0], dtype=np.float32)
        env.hitter_kps = np.array([100.0, 200.0, 300.0], dtype=np.float32)
        env.hitter_kds = np.array([11.0, 22.0, 33.0], dtype=np.float32)
        env.simulator = SimpleNamespace(
            default_angles=env.loco_default_angles.copy(),
            kps=env.loco_kps.copy(),
            kds=env.loco_kds.copy(),
            high_dt=0.02,
            cfg=SimpleNamespace(asset=SimpleNamespace(default_angles=[], kps=[], kds=[])),
        )
        env.compute_observation = lambda: None
        env._reset_hitter_obs_buffers = lambda: None
        return env

    def test_mimic_switch_request_maps_to_hitter_mode(self):
        env = self.make_env()
        cmd = SimpleNamespace(policy_switch="mimic")

        env._handle_policy_switch(cmd)

        self.assertEqual(env.get_pending_policy_switch(), "hitter")

    def test_policy_mode_switches_pd_and_defaults(self):
        env = self.make_env()

        env.set_policy_mode("hitter")
        self.assertEqual(env.policy_mode, "hitter")
        np.testing.assert_allclose(env.simulator.default_angles, env.hitter_default_angles)
        np.testing.assert_allclose(env.simulator.kps, env.hitter_kps)
        np.testing.assert_allclose(env.simulator.kds, env.hitter_kds)

        env.set_policy_mode("locomotion")
        self.assertEqual(env.policy_mode, "locomotion")
        np.testing.assert_allclose(env.simulator.default_angles, env.loco_default_angles)
        np.testing.assert_allclose(env.simulator.kps, env.loco_kps)
        np.testing.assert_allclose(env.simulator.kds, env.loco_kds)

    def test_configured_locomotion_arrays_override_asset_defaults(self):
        env = object.__new__(LocoHitterEnv)
        fallback = np.array([9.0, 9.0, 9.0], dtype=np.float32)
        switch_cfg = {"locomotion_kps": [100, 150, 40]}

        result = env._array_from_switch_cfg(switch_cfg, "locomotion_kps", fallback)

        np.testing.assert_allclose(result, np.array([100.0, 150.0, 40.0], dtype=np.float32))

    def test_auto_switch_only_when_planner_arms_hitter_command(self):
        env = self.make_env()
        env.hitter_ball_planner = object()
        env.auto_switch_on_ball = True

        env._sample_hitter_command_from_ball_planner = lambda: True
        self.assertFalse(env.try_arm_hitter_from_ball())

        def arm():
            env.hitter_command_initialized = True
            env.hitter_waiting_for_planner_arm = False
            return True

        env._sample_hitter_command_from_ball_planner = arm
        self.assertTrue(env.try_arm_hitter_from_ball())

    def test_agent_can_arm_and_log_without_auto_switching_to_hitter(self):
        env = self.make_env()
        env.try_arm_hitter_from_ball = lambda: True
        env.discard_armed_hitter_command = lambda reason: setattr(env, "discard_reason", reason)

        agent = object.__new__(LocoHitterAgent)
        agent.env = env
        agent.enable_auto_hitter_switch = False
        agent.log_armed_hitter_without_switch = True
        agent.interp_manager = SimpleNamespace(idle=True, switch_to_hitter=lambda: setattr(env, "switched", True))

        agent._auto_arm_hitter_if_ready()

        self.assertFalse(hasattr(env, "switched"))
        self.assertIn("auto HITTER switch disabled", env.discard_reason)

    def test_hitter_returns_after_strike_plus_grace_steps(self):
        env = self.make_env()
        env.policy_mode = "hitter"
        env.hitter_strike_elapsed_s = 1.0
        env.hitter_strike_time_s = 0.8
        env.hitter_run_steps = 21

        self.assertTrue(env.hitter_should_return_to_locomotion())

    def test_interp_manager_uses_configured_short_transition(self):
        env = self.make_env()
        env.simulator.active_dof_idx = np.arange(3)
        env.dof_pos = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
        env.loco_to_hitter_steps = 3
        env.hitter_to_loco_steps = 5
        manager = LocoHitterInterpManager(env)

        manager.switch_to_hitter()

        self.assertEqual(manager.interp_durations, [0, 3, 0])

    def test_armed_hitter_timer_advances_during_loco_to_hitter_transition(self):
        env = self.make_env()
        env.simulator.active_dof_idx = np.arange(3)
        env.dof_pos = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
        env.loco_to_hitter_steps = 3
        env.hitter_command_initialized = True
        env.hitter_waiting_for_planner_arm = False
        env.hitter_strike_elapsed_s = 0.0
        env.playback_speed = 1.0
        manager = LocoHitterInterpManager(env)

        manager.switch_to_hitter()
        manager.step()

        self.assertAlmostEqual(env.hitter_strike_elapsed_s, 0.02)

    def test_loco_hitter_config_uses_current_project_locomotion_checkpoint(self):
        with initialize(version_base=None, config_path="../deploy/config"):
            cfg = compose(config_name="loco_hitter", overrides=["robot.control.viewer=false"])

        self.assertEqual(cfg.agent._target_, "agents.loco_hitter_agent.LocoHitterAgent")
        self.assertEqual(cfg.env._target_, "envs.loco_hitter.LocoHitterEnv")
        self.assertEqual(cfg.locomotion.policy.checkpoint, "./data/model/locomotion/loco_level.pt")
        self.assertTrue((DEPLOY_DIR / "data/model/locomotion/loco_level.pt").exists())


if __name__ == "__main__":
    unittest.main()
