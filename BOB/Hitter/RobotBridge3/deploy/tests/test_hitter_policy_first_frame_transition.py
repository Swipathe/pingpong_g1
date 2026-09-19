import time
from types import SimpleNamespace

import numpy as np
import pytest

from envs.hitter import HitterEnv


class FakeRealWorldSimulator:
    is_real = True
    high_dt = 0.02

    def __init__(self, dof_pos):
        self.dof_pos = np.asarray(dof_pos, dtype=np.float32)
        self.applied_actions = []
        self.get_state_calls = 0

    def get_state(self):
        self.get_state_calls += 1

    def apply_action(self, action):
        self.applied_actions.append(np.asarray(action, dtype=np.float32).copy())


class FakeMujocoSimulator(FakeRealWorldSimulator):
    is_real = False


def make_env(simulator, transition_s=2.0):
    env = HitterEnv.__new__(HitterEnv)
    env.simulator = simulator
    env.policy_model_meta = object()
    env._policy_dim = 3
    env.action_beta = 1.0
    env.prev_policy_action = np.zeros(3, dtype=np.float32)
    env.policy_action_scales = np.array([0.1, 0.2, 0.3], dtype=np.float32)
    env.policy_default_joint_pos = np.array([1.0, 2.0, 3.0], dtype=np.float32)
    env._policy_to_sim_adapter = None
    env._policy_to_sim = np.array([0, 1, 2], dtype=np.int32)
    env._sim_to_policy = np.array([0, 1, 2], dtype=np.int32)
    env.save_video_enabled = False
    env.real_world_first_frame_transition_s = transition_s
    env._real_world_first_frame_transition_pending = False
    env._reset_real_world_first_frame_transition_pending()
    env.cfg = SimpleNamespace(
        control=SimpleNamespace(action_clip_value=10000.0)
    )
    env.episode_length_buf = np.zeros(1, dtype=np.int32)
    env.obs_buf_dict = {"actor_obs": np.zeros((1, 1), dtype=np.float32)}
    env.playback_speed = 1.0
    env.time_step = 0.0
    env._update_hitter_command = lambda: None
    env.compute_observation = lambda: None
    env._check_termination = lambda: None
    return env


def test_real_world_first_step_interpolates_to_frozen_first_policy_target(monkeypatch):
    sim = FakeRealWorldSimulator(dof_pos=[0.0, 0.0, 0.0])
    env = make_env(sim, transition_s=2.0)
    sleeps = []
    now = [100.0]

    monkeypatch.setattr(time, "monotonic", lambda: now[0])

    def fake_sleep(duration):
        sleeps.append(duration)
        now[0] += duration

    monkeypatch.setattr(time, "sleep", fake_sleep)

    obs = env.step(np.array([1.0, -1.0, 0.5], dtype=np.float32))

    target = np.array([1.1, 1.8, 3.15], dtype=np.float32)
    assert obs is env.obs_buf_dict
    assert sim.get_state_calls == 1
    assert len(sim.applied_actions) == 100
    assert all(action.shape == (1, 3) for action in sim.applied_actions)
    assert np.all(np.isfinite(np.stack(sim.applied_actions)))
    assert np.allclose(sim.applied_actions[-1][0], target)

    first_alpha = 3 * (1 / 100) ** 2 - 2 * (1 / 100) ** 3
    assert np.allclose(sim.applied_actions[0][0], first_alpha * target)
    assert np.all(np.diff([action[0, 0] for action in sim.applied_actions]) > 0.0)
    assert env._real_world_first_frame_transition_pending is False
    assert env.episode_length_buf[0] == 1
    assert np.allclose(env.prev_policy_action, [1.0, -1.0, 0.5])
    assert len(sleeps) == 100
    assert all(duration >= 0.0 for duration in sleeps)


def test_real_world_transition_runs_once_then_normal_step(monkeypatch):
    sim = FakeRealWorldSimulator(dof_pos=[0.0, 0.0, 0.0])
    env = make_env(sim, transition_s=2.0)
    monkeypatch.setattr(time, "monotonic", lambda: 0.0)
    monkeypatch.setattr(time, "sleep", lambda duration: None)

    env.step(np.array([1.0, 0.0, 0.0], dtype=np.float32))
    first_count = len(sim.applied_actions)

    env.step(np.array([2.0, 0.0, 0.0], dtype=np.float32))

    assert first_count == 100
    assert len(sim.applied_actions) == 101
    assert np.allclose(sim.applied_actions[-1][0], [1.2, 2.0, 3.0])
    assert sim.get_state_calls == 1


def test_reset_rearms_real_world_transition(monkeypatch):
    sim = FakeRealWorldSimulator(dof_pos=[0.0, 0.0, 0.0])
    env = make_env(sim, transition_s=2.0)
    monkeypatch.setattr(time, "monotonic", lambda: 0.0)
    monkeypatch.setattr(time, "sleep", lambda duration: None)

    env.step(np.array([1.0, 0.0, 0.0], dtype=np.float32))
    env._reset_real_world_first_frame_transition_pending()
    sim.dof_pos = np.array([1.0, 2.0, 3.0], dtype=np.float32)
    env.step(np.array([0.0, 1.0, 0.0], dtype=np.float32))

    assert len(sim.applied_actions) == 200
    assert np.allclose(sim.applied_actions[-1][0], [1.0, 2.2, 3.0])


def test_mujoco_never_runs_first_frame_transition_even_if_configured():
    sim = FakeMujocoSimulator(dof_pos=[0.0, 0.0, 0.0])
    env = make_env(sim, transition_s=2.0)

    env.step(np.array([1.0, 0.0, 0.0], dtype=np.float32))

    assert len(sim.applied_actions) == 1
    assert sim.get_state_calls == 0


@pytest.mark.parametrize("transition_s", [0.0, -0.0])
def test_zero_duration_keeps_old_single_apply_behavior(transition_s):
    sim = FakeRealWorldSimulator(dof_pos=[0.0, 0.0, 0.0])
    env = make_env(sim, transition_s=transition_s)

    env.step(np.array([1.0, 0.0, 0.0], dtype=np.float32))

    assert len(sim.applied_actions) == 1
    assert sim.get_state_calls == 0


@pytest.mark.parametrize("transition_s", [-0.1, float("nan"), float("inf")])
def test_invalid_transition_duration_is_rejected_before_motion(transition_s):
    sim = FakeRealWorldSimulator(dof_pos=[0.0, 0.0, 0.0])
    env = HitterEnv.__new__(HitterEnv)
    env.simulator = sim
    env.real_world_first_frame_transition_s = transition_s

    with pytest.raises(ValueError):
        env._reset_real_world_first_frame_transition_pending()

    assert sim.applied_actions == []


def test_invalid_measured_start_fails_before_motion(monkeypatch):
    sim = FakeRealWorldSimulator(dof_pos=[0.0, float("nan"), 0.0])
    env = make_env(sim, transition_s=2.0)
    monkeypatch.setattr(time, "sleep", lambda duration: None)

    with pytest.raises(ValueError):
        env.step(np.array([1.0, 0.0, 0.0], dtype=np.float32))

    assert sim.applied_actions == []


def test_invalid_first_policy_target_fails_before_motion(monkeypatch):
    sim = FakeRealWorldSimulator(dof_pos=[0.0, 0.0, 0.0])
    env = make_env(sim, transition_s=2.0)
    monkeypatch.setattr(time, "sleep", lambda duration: None)

    with pytest.raises(ValueError):
        env.step(np.array([float("nan"), 0.0, 0.0], dtype=np.float32))

    assert sim.applied_actions == []
