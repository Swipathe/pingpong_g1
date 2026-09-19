import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np


DEPLOY_DIR = Path(__file__).resolve().parents[1] / "deploy"
sys.path.insert(0, str(DEPLOY_DIR))

from agents.hitter_agent import HitterAgent


class HitterAgentStartupTest(unittest.TestCase):
    def test_policy_action_for_target_handles_zero_action_scale(self):
        agent = object.__new__(HitterAgent)
        agent.env = SimpleNamespace(
            policy_default_joint_pos=np.array([0.0, 1.0, -2.0], dtype=np.float32),
            policy_action_scales=np.array([0.5, 0.0, 2.0], dtype=np.float32),
        )

        action = HitterAgent._policy_action_for_policy_target(
            agent,
            np.array([1.0, 4.0, 0.0], dtype=np.float32),
        )

        np.testing.assert_allclose(action, np.array([2.0, 0.0, 1.0], dtype=np.float32))

    def test_startup_interp_action_moves_from_current_policy_pose_to_default(self):
        agent = object.__new__(HitterAgent)
        agent.env = SimpleNamespace(
            policy_default_joint_pos=np.array([0.0, 1.0], dtype=np.float32),
            policy_action_scales=np.array([1.0, 2.0], dtype=np.float32),
        )
        agent.startup_interp_steps = 4
        agent.startup_interp_step = 0
        agent._startup_interp_start_policy_q = np.array([1.0, -1.0], dtype=np.float32)

        action = HitterAgent._startup_interp_action(agent)

        np.testing.assert_allclose(action, np.array([0.75, -0.75], dtype=np.float32))

    def test_next_action_holds_default_before_first_planner_command(self):
        class PolicyThatMustNotRun:
            def run(self, *_args, **_kwargs):
                raise AssertionError("ONNX policy should not run before the first planner command is ready.")

        agent = object.__new__(HitterAgent)
        agent.env = SimpleNamespace(
            hitter_mode=True,
            use_ball_planner=True,
            hitter_has_valid_command=False,
            hitter_command_initialized=False,
            policy_default_joint_pos=np.zeros(3, dtype=np.float32),
        )
        agent.policy = PolicyThatMustNotRun()
        agent.startup_interp_steps = 0
        agent.startup_interp_step = 0
        agent._startup_interp_start_policy_q = None
        agent._waiting_for_first_command_logged = False

        action = HitterAgent._next_action(agent, {"obs": np.ones((1, 105), dtype=np.float32)})

        np.testing.assert_allclose(action, np.zeros(3, dtype=np.float32))

    def test_next_action_runs_policy_after_first_planner_command_even_if_current_command_is_waiting(self):
        class PolicyThatRecordsCall:
            def __init__(self):
                self.called = False

            def run(self, _outputs, inputs):
                self.called = True
                np.testing.assert_allclose(inputs["obs"], np.ones((1, 105), dtype=np.float32))
                return [np.array([[0.1, -0.2, 0.3]], dtype=np.float32)]

        agent = object.__new__(HitterAgent)
        agent.env = SimpleNamespace(
            hitter_mode=True,
            use_ball_planner=True,
            hitter_has_valid_command=True,
            hitter_command_initialized=False,
            policy_default_joint_pos=np.zeros(3, dtype=np.float32),
        )
        agent.policy = PolicyThatRecordsCall()
        agent.startup_interp_steps = 0
        agent.startup_interp_step = 0
        agent._startup_interp_start_policy_q = None
        agent._waiting_for_first_command_logged = False

        action = HitterAgent._next_action(agent, {"obs": np.ones((1, 105), dtype=np.float32)})

        self.assertTrue(agent.policy.called)
        np.testing.assert_allclose(action, np.array([[0.1, -0.2, 0.3]], dtype=np.float32))


if __name__ == "__main__":
    unittest.main()
