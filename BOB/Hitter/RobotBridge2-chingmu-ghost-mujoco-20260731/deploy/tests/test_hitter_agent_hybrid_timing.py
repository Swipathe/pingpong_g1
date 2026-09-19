from __future__ import annotations

import unittest
from contextlib import nullcontext
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from agents.hitter_agent import HitterAgent
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


class StopAgentLoop(RuntimeError):
    pass


class RecordingPolicy:
    def __init__(self, events):
        self.events = events

    def run(self, _outputs, inputs):
        self.events.append(("policy", tuple(inputs)))
        return [np.asarray([[0.25]], dtype=np.float32)]


class RecordingEnvironment:
    def __init__(self, capabilities, *, is_real, stop_after_steps=None):
        self.events = []
        self.simulator = SimpleNamespace(
            is_real=is_real,
            high_dt=0.02,
            hitter_runtime_capabilities=capabilities,
        )
        self.refresh_count = 0
        self.step_count = 0
        self.stop_after_steps = stop_after_steps

    def reset(self):
        self.events.append("reset")
        return {"obs": np.asarray([[0.0]], dtype=np.float32)}

    def refresh_policy_observation(self):
        self.refresh_count += 1
        self.events.append("refresh")
        if self.refresh_count == 2:
            raise StopAgentLoop("test loop complete")
        return {"obs": np.asarray([[1.0]], dtype=np.float32)}

    def step(self, action):
        self.step_count += 1
        self.events.append(("step", float(action[0, 0])))
        if self.step_count == self.stop_after_steps:
            raise StopAgentLoop("test loop complete")
        return {"obs": np.asarray([[2.0]], dtype=np.float32)}

    def close(self):
        self.events.append("close")


class InterruptingEnvironment(RecordingEnvironment):
    def refresh_policy_observation(self):
        self.events.append("refresh")
        raise KeyboardInterrupt()


def build_agent(environment):
    agent = HitterAgent.__new__(HitterAgent)
    agent.env = environment
    agent.policy = RecordingPolicy(environment.events)
    return agent


class HitterAgentHybridTimingTests(unittest.TestCase):
    def test_hybrid_refreshes_observation_before_policy_inference(self):
        environment = RecordingEnvironment(
            HYBRID_CAPABILITIES,
            is_real=False,
        )
        agent = build_agent(environment)

        result = agent._run_iteration(
            {"obs": np.asarray([[0.0]], dtype=np.float32)}
        )

        self.assertEqual(
            environment.events,
            [
                "refresh",
                ("policy", ("obs",)),
                ("step", 0.25),
            ],
        )
        self.assertEqual(float(result["obs"][0, 0]), 2.0)

    def test_hybrid_does_not_execute_agent_sleep_despite_true_is_real(self):
        environment = RecordingEnvironment(
            HYBRID_CAPABILITIES,
            is_real=True,
        )
        agent = build_agent(environment)

        with patch("agents.hitter_agent.time.sleep") as sleep:
            with self.assertRaisesRegex(StopAgentLoop, "test loop complete"):
                agent.run()

        sleep.assert_not_called()
        self.assertEqual(
            environment.events,
            [
                "reset",
                "refresh",
                ("policy", ("obs",)),
                ("step", 0.25),
                "refresh",
                "close",
            ],
        )

    def test_keyboard_interrupt_closes_environment_without_reraising(self):
        environment = InterruptingEnvironment(
            HYBRID_CAPABILITIES,
            is_real=False,
        )
        agent = build_agent(environment)

        agent.run()

        self.assertEqual(
            environment.events,
            [
                "reset",
                "refresh",
                "close",
            ],
        )

    def test_real_world_profile_keeps_agent_pacing_despite_false_is_real(self):
        environment = RecordingEnvironment(
            RealWorld.hitter_runtime_capabilities,
            is_real=False,
        )
        agent = build_agent(environment)

        with patch(
            "agents.hitter_agent.time.time",
            side_effect=[10.0, 10.005, 10.006],
        ):
            with patch("agents.hitter_agent.time.sleep") as sleep:
                with self.assertRaisesRegex(
                    StopAgentLoop,
                    "test loop complete",
                ):
                    agent.run_eval()

        sleep.assert_called_once()
        self.assertAlmostEqual(sleep.call_args.args[0], 0.015)
        self.assertEqual(environment.events[-1], "close")

    def test_mujoco_run_skips_agent_sleep_despite_true_is_real(self):
        environment = RecordingEnvironment(
            Mujoco.hitter_runtime_capabilities,
            is_real=True,
            stop_after_steps=2,
        )
        agent = build_agent(environment)

        with patch("agents.hitter_agent.time.sleep") as sleep:
            with self.assertRaisesRegex(StopAgentLoop, "test loop complete"):
                agent.run()

        sleep.assert_not_called()
        self.assertEqual(environment.refresh_count, 0)
        self.assertEqual(
            environment.events,
            [
                "reset",
                ("policy", ("obs",)),
                ("step", 0.25),
                ("policy", ("obs",)),
                ("step", 0.25),
                "close",
            ],
        )

    def test_mujoco_realtime_pacing_sleeps_inside_simulator(self):
        simulator = Mujoco.__new__(Mujoco)
        simulator.cfg = SimpleNamespace(
            control=SimpleNamespace(
                is_mosaic=True,
                real_time=True,
                torque_clip_value=[100.0],
            )
        )
        simulator.real_start_time = None
        simulator.sim_start_time = 0.0
        simulator.mujoco_model = object()
        simulator.mujoco_data = SimpleNamespace(
            time=0.0,
            qpos=np.zeros(8, dtype=np.float64),
            qvel=np.zeros(7, dtype=np.float64),
            ctrl=np.zeros(1, dtype=np.float64),
        )
        simulator.decimation = 1
        simulator.paused = False
        simulator.viewer = None
        simulator.viewer_enabled = False
        simulator.active_dof_idx = np.asarray([0], dtype=np.int32)
        simulator.frozen_dof_idx = np.asarray([], dtype=np.int32)
        simulator.kps = np.zeros(1, dtype=np.float32)
        simulator.kds = np.zeros(1, dtype=np.float32)
        simulator.num_dof = 1
        simulator._viewer_lock = nullcontext

        def advance_physics(_model, data):
            data.time = 0.02

        with patch(
            "simulator.mujoco.time.perf_counter",
            side_effect=[100.0, 100.005],
        ) as perf_counter:
            with patch(
                "simulator.mujoco.mujoco.mj_step",
                side_effect=advance_physics,
            ) as physics_step:
                with patch("simulator.mujoco.time.sleep") as sleep:
                    simulator.apply_action(
                        np.asarray([0.0], dtype=np.float32)
                    )

        self.assertEqual(perf_counter.call_count, 2)
        physics_step.assert_called_once_with(
            simulator.mujoco_model,
            simulator.mujoco_data,
        )
        sleep.assert_called_once()
        self.assertAlmostEqual(sleep.call_args.args[0], 0.015)
        self.assertEqual(simulator.real_start_time, 100.0)
        self.assertEqual(simulator.sim_start_time, 0.0)


if __name__ == "__main__":
    unittest.main()
