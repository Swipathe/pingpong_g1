from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from envs.hitter import HitterEnv


def _offline_env() -> HitterEnv:
    simulator = SimpleNamespace(
        get_state=lambda: None,
        mujoco_data=SimpleNamespace(time=2.0),
        ball_pos_world=np.array([1.5, 0.0, 1.0]),
        ball_vel_world=np.array([-1.0, 0.0, 0.0]),
        root_trans_world=np.array([0.0, 0.0, 0.8]),
        root_quat_world=np.array([0.0, 0.0, 0.0, 1.0]),
        base_pose_valid=True,
        ball_visible=True,
        ball_state_estimator_ready=True,
    )
    env = HitterEnv.__new__(HitterEnv)
    env.simulator = simulator
    env._hitter_sync_track_id = 0
    env._hitter_sync_generation = 0
    env.hitter_ball_sequence_needs_reset = True
    env._mujoco_serve_schedule_enabled = lambda: False
    env._reset_hitter_ball_sequence_if_needed = lambda: setattr(
        env,
        "hitter_ball_sequence_needs_reset",
        False,
    )
    env._plan_hitter_snapshot = lambda _snapshot: SimpleNamespace(time_to_strike=0.9)
    return env


def test_mujoco_local_track_ids_start_at_one_and_stay_with_the_serve():
    env = _offline_env()

    first = env._mujoco_planner_result(now=1000.0)
    next_generation = env._mujoco_planner_result(now=1000.01)

    assert first.track_id == 1
    assert first.source_generation == 1
    assert next_generation.track_id == 1
    assert next_generation.source_generation == 2
    assert not hasattr(first, "track_epoch")
