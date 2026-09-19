from __future__ import annotations

import numpy as np
import pytest

from tests.hitter_runtime_test_harness import (
    agent_harness,
    failure_result,
    make_hitter_env_for_test,
    make_snapshot,
    result_batch,
    success_result,
)
from utils.hitter_realtime import CommandPhase
from utils.hitter_runtime_types import PlannerFailureReason


def test_real_waiting_anchor_is_captured_once_and_corrects_drift():
    env = make_hitter_env_for_test(is_real=True)
    env.waiting_base_anchor_xy_w = None
    env.simulator.base_pose_valid = True
    env.simulator.root_trans_world = np.array([1.2, -0.3, 0.78])

    assert env._capture_hitter_waiting_base_anchor(require_initial=True)
    env.simulator.root_trans_world = np.array([1.2, -0.1, 0.78])
    pos, quat = env._hitter_robot_anchor_pose_w()
    target_b = env._hitter_waiting_base_target_pos_b(pos, quat)

    assert np.allclose(env.waiting_base_anchor_xy_w, [1.2, -0.3])
    assert target_b[1] < 0.0


@pytest.mark.parametrize("edge", ["cancel", "recovery_complete"])
def test_each_waiting_edge_refreshes_and_recaptures_anchor_once(edge):
    env = make_hitter_env_for_test(is_real=True)
    worker = env.hitter_planner_worker
    if edge == "cancel":
        worker.queue(
            result_batch(
                failure_result(
                    PlannerFailureReason.TRACK_ENDED,
                    now=10.0,
                )
            )
        )
        now = 10.0
    else:
        worker.queue(result_batch(success_result(now=10.0, deadline=10.90)))
        env._update_hitter_command(now=10.0)
        env._update_hitter_command(now=10.90)
        assert env.hitter_command_lifecycle.phase is CommandPhase.RECOVERY
        now = float(env.hitter_command_lifecycle.command_end_deadline_s)

    env.simulator.root_trans_world = np.asarray(
        [2.0, 0.4, 0.78], dtype=np.float32
    )
    before = env.simulator.get_state_calls
    env._update_hitter_command(now=now)

    assert env.simulator.get_state_calls - before == 1
    assert np.allclose(env.waiting_base_anchor_xy_w, [2.0, 0.4])


def test_invalid_pose_retains_old_anchor_and_latches_fault():
    env = make_hitter_env_for_test(is_real=True)
    old_anchor = env.waiting_base_anchor_xy_w.copy()
    env.simulator.base_pose_valid = False

    assert not env._capture_hitter_waiting_base_anchor(require_initial=False)

    assert np.array_equal(env.waiting_base_anchor_xy_w, old_anchor)
    assert env._waiting_anchor_fault.value == "BASE_POSE_INVALID"


def test_pose_recovery_in_same_session_cannot_submit_or_arm():
    env = make_hitter_env_for_test(is_real=True)
    env.simulator.base_pose_valid = False
    assert not env._capture_hitter_waiting_base_anchor(require_initial=False)
    env.simulator.base_pose_valid = True
    snapshot = make_snapshot(track_id=8, received=10.0)
    env.simulator.present(snapshot)
    env._submit_hitter_planner_snapshot(snapshot)

    assert len(env.hitter_planner_worker.submitted) == 0
    assert env.hitter_command_lifecycle.phase is CommandPhase.WAITING
    assert 8 in env.hitter_command_lifecycle.consumed_track_ids
    assert 8 in env.simulator.consumed_track_ids


def test_only_successful_explicit_reentry_clears_anchor_fault():
    env = make_hitter_env_for_test(is_real=True)
    env.simulator.base_pose_valid = False
    assert not env._capture_hitter_waiting_base_anchor(require_initial=False)
    env.simulator.base_pose_valid = True
    env.simulator.root_trans_world = np.asarray(
        [3.0, -0.6, 0.78], dtype=np.float32
    )

    assert env._complete_hitter_policy_reentry(now=20.0)

    assert env._waiting_anchor_fault is None
    assert env._hitter_runtime_accepting is True
    assert env.simulator.session_open is True
    assert np.allclose(env.waiting_base_anchor_xy_w, [3.0, -0.6])


def test_invalid_pose_without_any_anchor_raises():
    env = make_hitter_env_for_test(is_real=True)
    env.waiting_base_anchor_xy_w = None
    env.simulator.base_pose_valid = False

    with pytest.raises(RuntimeError, match="valid G2Pelvis waiting anchor"):
        env._capture_hitter_waiting_base_anchor(require_initial=True)


def test_mujoco_waiting_keeps_configured_world_target():
    env = make_hitter_env_for_test(is_real=False)
    env.simulator.root_trans_world = np.asarray(
        [0.1, 0.2, 0.78], dtype=np.float32
    )
    pos, quat = env._hitter_robot_anchor_pose_w()

    target_b = env._hitter_waiting_base_target_pos_b(pos, quat)

    assert np.allclose(target_b[:2], [-0.5, -0.2])


def test_waiting_keeps_policy_and_pd_running():
    harness = agent_harness(is_real=True)

    harness.run_ticks(10, phase=CommandPhase.WAITING)

    assert harness.onnx_calls == 10
    assert harness.apply_action_calls == 10
    assert harness.last_observation.shape == (1, 104)
    assert np.isfinite(harness.last_observation).all()
