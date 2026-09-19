import time
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from envs.hitter import HitterEnv
from tests.hitter_runtime_test_harness import make_hitter_env_for_test


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
    env._hitter_lifecycle_lock = threading.RLock()
    env._complete_hitter_policy_reentry = lambda *, now: True
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


def test_transition_keeps_session_closed_until_refreshed_anchor_recapture(monkeypatch):
    env = make_hitter_env_for_test(is_real=True)
    env.real_world_first_frame_transition_s = 0.04
    env._reset_real_world_first_frame_transition_pending()
    env._reset_hitter_lifecycle_state(now=10.0)
    env.simulator.root_trans_world = np.asarray(
        [1.0, -0.2, 0.78], dtype=np.float32
    )
    env._capture_hitter_waiting_base_anchor(require_initial=True)
    env.simulator.root_trans_world = np.asarray(
        [2.0, 0.4, 0.78], dtype=np.float32
    )
    seen_session_states = []
    original_apply = env.simulator.apply_action

    def apply_action(action):
        seen_session_states.append(env.simulator.session_open)
        original_apply(action)

    env.simulator.apply_action = apply_action
    monkeypatch.setattr(time, "monotonic", lambda: 11.0)
    monkeypatch.setattr(time, "sleep", lambda _duration: None)

    env.step(np.zeros(29, dtype=np.float32))

    assert seen_session_states == [False, False]
    assert env.simulator.session_open is True
    assert env._hitter_runtime_accepting is True
    assert env.simulator.get_state_calls >= 2
    assert np.allclose(env.waiting_base_anchor_xy_w, [2.0, 0.4])


def test_zero_duration_uses_same_immediate_reentry_flow(monkeypatch):
    env = make_hitter_env_for_test(is_real=True)
    env.real_world_first_frame_transition_s = 0.0
    env._reset_real_world_first_frame_transition_pending()
    env._reset_hitter_lifecycle_state(now=10.0)
    env.simulator.root_trans_world = np.asarray(
        [3.0, -0.7, 0.78], dtype=np.float32
    )
    monkeypatch.setattr(time, "monotonic", lambda: 12.0)

    assert env._complete_hitter_policy_reentry(now=12.0)
    env.step(np.zeros(29, dtype=np.float32))

    assert env.simulator.session_open is True
    assert env._hitter_runtime_accepting is True
    assert len(env.simulator.applied_actions) == 1
    assert np.allclose(env.waiting_base_anchor_xy_w, [3.0, -0.7])


def test_hard_reset_keeps_admission_closed_until_next_transition_step(monkeypatch):
    env = make_hitter_env_for_test(is_real=True)
    env.real_world_first_frame_transition_s = 0.04
    env.simulator.check_termination = lambda: True
    env.simulator.root_trans_world = np.asarray(
        [4.0, -0.8, 0.78], dtype=np.float32
    )

    env._check_termination()

    assert env._real_world_first_frame_transition_pending is True
    assert env.simulator.session_open is False
    assert env._hitter_runtime_accepting is False
    assert np.allclose(env.waiting_base_anchor_xy_w, [4.0, -0.8])

    env.simulator.root_trans_world = np.asarray(
        [4.2, -0.6, 0.78], dtype=np.float32
    )
    env.simulator.check_termination = lambda: False
    monkeypatch.setattr(time, "monotonic", lambda: 21.0)
    monkeypatch.setattr(time, "sleep", lambda _duration: None)

    env.step(np.zeros(29, dtype=np.float32))

    assert env._real_world_first_frame_transition_pending is False
    assert env.simulator.session_open is True
    assert env._hitter_runtime_accepting is True
    assert np.allclose(env.waiting_base_anchor_xy_w, [4.2, -0.6])


def test_zero_duration_hard_reset_reopens_session_immediately(monkeypatch):
    env = make_hitter_env_for_test(is_real=True)
    env.real_world_first_frame_transition_s = 0.0
    env.simulator.check_termination = lambda: True
    env.simulator.root_trans_world = np.asarray(
        [5.0, 0.25, 0.78], dtype=np.float32
    )
    monkeypatch.setattr(time, "monotonic", lambda: 22.0)

    env._check_termination()

    assert env._real_world_first_frame_transition_pending is False
    assert env.simulator.session_open is True
    assert env._hitter_runtime_accepting is True
    assert np.allclose(env.waiting_base_anchor_xy_w, [5.0, 0.25])


def test_reset_rejects_invalid_initial_pose_even_with_old_anchor():
    env = make_hitter_env_for_test(is_real=True)
    old_anchor = env.waiting_base_anchor_xy_w.copy()
    env.real_world_first_frame_transition_s = 0.04
    env.simulator.base_pose_valid = False
    env.simulator.root_trans_world = np.asarray(
        [8.0, 3.0, 0.78], dtype=np.float32
    )

    with pytest.raises(
        RuntimeError,
        match="valid G2Pelvis waiting anchor",
    ):
        env.reset()

    assert env.simulator.session_open is False
    assert env._hitter_runtime_accepting is False
    assert np.array_equal(env.waiting_base_anchor_xy_w, old_anchor)


def test_reset_rejects_stale_cached_base_pose_and_retains_old_anchor(
    monkeypatch,
):
    env = make_hitter_env_for_test(is_real=True)
    old_anchor = env.waiting_base_anchor_xy_w.copy()
    env.real_world_first_frame_transition_s = 0.04
    env.simulator.base_pose_valid = True
    env.simulator.stream_fresh = False
    env.simulator.root_trans_world = np.asarray(
        [9.0, 4.0, 0.78], dtype=np.float32
    )
    monkeypatch.setattr(time, "monotonic", lambda: 40.0)

    with pytest.raises(RuntimeError, match="fresh valid G2Pelvis"):
        env.reset()

    assert env.simulator.status_calls == 1
    assert env.simulator.session_open is False
    assert env._hitter_runtime_accepting is False
    assert np.array_equal(env.waiting_base_anchor_xy_w, old_anchor)


def test_reentry_quarantines_again_after_anchor_recapture_before_begin():
    env = make_hitter_env_for_test(is_real=True)
    order = []
    original_quarantine = env._quarantine_hitter_runtime_locked
    original_capture = env._capture_hitter_waiting_base_anchor
    original_begin = env.simulator.begin_hitter_policy_session

    def quarantine(**kwargs):
        order.append("quarantine")
        return original_quarantine(**kwargs)

    def capture(**kwargs):
        order.append("capture")
        return original_capture(**kwargs)

    def begin(**kwargs):
        order.append("begin")
        return original_begin(**kwargs)

    env._quarantine_hitter_runtime_locked = quarantine
    env._capture_hitter_waiting_base_anchor = capture
    env.simulator.begin_hitter_policy_session = begin

    assert env._complete_hitter_policy_reentry(now=30.0)

    assert order == ["quarantine", "capture", "quarantine", "begin"]
