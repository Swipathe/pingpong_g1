from __future__ import annotations

import unittest

import numpy as np

from agents.hitter_agent import HitterAgent


class FakePolicy:
    def __init__(self, events, *, error=None):
        self.events = events
        self.error = error

    def run(self, output_names, inputs):
        self.events.append("policy")
        if self.error is not None:
            raise self.error
        self.output_names = output_names
        self.inputs = inputs
        return [np.full((1, 29), 0.25, dtype=np.float32)]


class FakeSimulator:
    def __init__(self, *, is_real: bool):
        self.is_real = bool(is_real)
        self.high_dt = 0.02


class FakeEnv:
    def __init__(self, *, is_real: bool, events):
        self.simulator = FakeSimulator(is_real=is_real)
        self.events = events
        self.close_calls = 0
        self.initial_observation = {
            "obs": np.zeros((1, 105), dtype=np.float64),
        }
        self.refreshed_observation = {
            "obs": np.ones((1, 105), dtype=np.float64),
        }

    def reset(self):
        self.events.append("reset")
        return self.initial_observation

    def refresh_policy_observation(self):
        self.events.append("refresh")
        return self.refreshed_observation

    def step(self, action):
        self.events.append("step")
        self.action = np.asarray(action).copy()
        return self.initial_observation

    def close(self):
        self.events.append("close")
        self.close_calls += 1


def make_fake_agent(*, is_real: bool, events, policy_error=None) -> HitterAgent:
    agent = HitterAgent.__new__(HitterAgent)
    agent.env = FakeEnv(is_real=is_real, events=events)
    agent.policy = FakePolicy(events, error=policy_error)
    agent.time = 0.0
    return agent


class HitterAgentTimingTests(unittest.TestCase):
    def test_real_iteration_refreshes_before_policy_inference(self):
        events = []
        agent = make_fake_agent(is_real=True, events=events)

        result = agent._run_iteration(agent.env.initial_observation)

        self.assertIs(result, agent.env.initial_observation)
        self.assertEqual(events, ["refresh", "policy", "step"])
        np.testing.assert_array_equal(
            agent.policy.inputs["obs"],
            agent.env.refreshed_observation["obs"],
        )
        self.assertEqual(agent.policy.inputs["obs"].dtype, np.float32)

    def test_mujoco_iteration_does_not_double_refresh(self):
        events = []
        agent = make_fake_agent(is_real=False, events=events)

        result = agent._run_iteration(agent.env.initial_observation)

        self.assertIs(result, agent.env.initial_observation)
        self.assertEqual(events, ["policy", "step"])
        np.testing.assert_array_equal(
            agent.policy.inputs["obs"],
            agent.env.initial_observation["obs"],
        )
        self.assertEqual(agent.policy.inputs["obs"].dtype, np.float32)

    def test_run_closes_environment_when_policy_raises(self):
        events = []
        failure = RuntimeError("policy failed")
        agent = make_fake_agent(
            is_real=True,
            events=events,
            policy_error=failure,
        )

        with self.assertRaisesRegex(RuntimeError, "policy failed"):
            agent.run()

        self.assertEqual(events, ["reset", "refresh", "policy", "close"])
        self.assertEqual(agent.env.close_calls, 1)

    def test_run_eval_closes_environment_on_keyboard_interrupt(self):
        events = []
        agent = make_fake_agent(
            is_real=False,
            events=events,
            policy_error=KeyboardInterrupt(),
        )

        with self.assertRaises(KeyboardInterrupt):
            agent.run_eval()

        self.assertEqual(events, ["reset", "policy", "close"])
        self.assertEqual(agent.env.close_calls, 1)


if __name__ == "__main__":
    unittest.main()
