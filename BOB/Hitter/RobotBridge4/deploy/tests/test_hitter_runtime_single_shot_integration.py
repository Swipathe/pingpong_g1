from __future__ import annotations

import threading
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

import envs.hitter as hitter_module
from envs.hitter import HitterEnv
from tests.hitter_runtime_test_harness import (
    FakePlannerWorker,
    FakeSimulator,
    default_runtime_settings,
    failure_result,
    make_hitter_env_for_test,
    make_snapshot,
    result_batch,
    success_result,
)
from utils.hitter_realtime import CommandPhase, LifecycleCancelReason
from utils.hitter_runtime_types import PlannerFailureReason


def test_reset_reuses_lifecycle_and_worker_and_quarantines_every_known_id():
    env = make_hitter_env_for_test(is_real=True)
    lifecycle = env.hitter_command_lifecycle
    worker = env.hitter_planner_worker
    lifecycle.ingest(success_result(track_id=9), now=10.0)
    env._hitter_submitted_track_ids.add(6)
    worker.queue(result_batch(success_result(track_id=7)))
    env.simulator.active_track_id = 8

    env._reset_hitter_lifecycle_state(now=11.0)

    assert env.hitter_command_lifecycle is lifecycle
    assert env.hitter_planner_worker is worker
    assert {6, 7, 8, 9} <= lifecycle.consumed_track_ids
    assert {6, 7, 8, 9} <= env.simulator.consumed_track_ids
    assert env._hitter_runtime_accepting is False
    assert env.simulator.session_open is False
    worker.queue(result_batch(success_result(track_id=6, generation=2)))
    env._update_hitter_command(now=11.1)
    assert lifecycle.phase is CommandPhase.WAITING
    assert lifecycle.active_result is None


def test_closed_runtime_drains_late_batch_as_consumed_without_ingest():
    env = make_hitter_env_for_test(is_real=True)
    env._reset_hitter_lifecycle_state(now=10.0)
    env.hitter_planner_worker.queue(
        result_batch(success_result(track_id=7, generation=2))
    )

    env._update_hitter_command(now=10.1)

    assert env.hitter_command_lifecycle.phase is CommandPhase.WAITING
    assert env.hitter_command_lifecycle.active_result is None
    assert 7 in env.hitter_command_lifecycle.consumed_track_ids
    assert 7 in env.simulator.consumed_track_ids


def test_mujoco_recovery_completion_requests_next_ball_sequence():
    env = make_hitter_env_for_test(is_real=False)
    armed = success_result(track_id=7, now=10.0, deadline=10.90)
    assert env.hitter_command_lifecycle.ingest(
        armed,
        now=10.0,
    ).kind == "armed"
    env._mujoco_planner_result = lambda *, now: success_result(
        track_id=7,
        generation=2,
        now=now,
        deadline=now + 0.9,
    )

    env._update_hitter_command(now=10.90)
    assert env.hitter_command_lifecycle.phase is CommandPhase.RECOVERY
    env.hitter_ball_sequence_needs_reset = False

    env._update_hitter_command(
        now=float(env.hitter_command_lifecycle.command_end_deadline_s)
    )

    assert env.hitter_command_lifecycle.phase is CommandPhase.WAITING
    assert env.hitter_ball_sequence_needs_reset is True


def test_worker_receives_configured_completed_queue_capacity(monkeypatch):
    captured = {}
    worker = FakePlannerWorker()

    def construct(plan_fn, *, monotonic_fn, completed_result_queue_capacity):
        captured.update(
            plan_fn=plan_fn,
            monotonic_fn=monotonic_fn,
            capacity=completed_result_queue_capacity,
        )
        return worker

    monkeypatch.setattr(hitter_module, "LatestOnlyPlannerWorker", construct)
    env = HitterEnv.__new__(HitterEnv)
    env.simulator = FakeSimulator(is_real=True)
    env.hitter_runtime_settings = default_runtime_settings()
    env.hitter_rng = __import__("numpy").random.default_rng(3)
    env._init_hitter_lifecycle_state()
    env._initialize_hitter_realtime_runtime()

    assert captured["capacity"] == 64
    assert env.hitter_planner_worker is worker


@pytest.mark.parametrize("invalid_height", [float("nan"), 1.0e100])
def test_nonfinite_target_base_height_fails_before_runtime_arming(
    invalid_height,
):
    env = make_hitter_env_for_test(is_real=True)
    env.motion_cfg["ball_planner"]["target_base_height_w"] = invalid_height

    with pytest.raises(ValueError, match="target_base_height_w"):
        env._init_hitter_command_state()

    assert env.hitter_command_lifecycle.phase is CommandPhase.WAITING
    assert env.hitter_command_lifecycle.active_result is None


def test_real_tick_order_is_status_events_advance_batch_then_ordered_ingest():
    env = make_hitter_env_for_test(is_real=True)
    order = env.simulator.call_order
    env.hitter_planner_worker.call_order = order
    original_advance = env.hitter_command_lifecycle.advance
    original_ingest = env.hitter_command_lifecycle.ingest

    def advance(now):
        order.append("advance")
        return original_advance(now)

    def ingest(result, *, now):
        order.append("ingest")
        return original_ingest(result, now=now)

    env.hitter_command_lifecycle.advance = advance
    env.hitter_command_lifecycle.ingest = ingest
    env.hitter_planner_worker.queue(
        result_batch(
            failure_result(
                PlannerFailureReason.ESTIMATOR_NOT_READY,
                generation=1,
            )
        )
    )

    env._update_hitter_command(now=10.0)

    assert order == ["status", "events", "advance", "batch", "ingest"]


def test_all_completed_results_are_ingested_in_order_and_third_soft_failure_cancels():
    env = make_hitter_env_for_test(is_real=True)
    generations = []
    original_ingest = env.hitter_command_lifecycle.ingest

    def ingest(result, *, now):
        generations.append(result.source_generation)
        return original_ingest(result, now=now)

    env.hitter_command_lifecycle.ingest = ingest
    env.hitter_planner_worker.queue(
        result_batch(
            success_result(generation=1),
            failure_result(
                PlannerFailureReason.BALL_NOT_INCOMING, generation=2
            ),
            failure_result(
                PlannerFailureReason.BALL_NOT_INCOMING, generation=3
            ),
            failure_result(
                PlannerFailureReason.BALL_NOT_INCOMING, generation=4
            ),
        )
    )

    env._update_hitter_command(now=10.0)

    assert generations == [1, 2, 3, 4]
    assert env.hitter_command_lifecycle.phase is CommandPhase.WAITING
    assert 7 in env.hitter_command_lifecycle.consumed_track_ids
    assert 7 in env.simulator.consumed_track_ids


def test_overflow_ingests_zero_results_and_consumes_whole_compromised_batch():
    env = make_hitter_env_for_test(is_real=True)
    ingested = []
    original_ingest = env.hitter_command_lifecycle.ingest

    def ingest(result, *, now):
        ingested.append(result.track_id)
        return original_ingest(result, now=now)

    env.hitter_command_lifecycle.ingest = ingest
    env.hitter_planner_worker.queue(
        result_batch(
            success_result(track_id=7),
            overflowed_track_ids=(8,),
        )
    )

    env._update_hitter_command(now=10.0)

    assert ingested == []
    assert env.hitter_command_lifecycle.phase is CommandPhase.WAITING
    assert {7, 8} <= env.hitter_command_lifecycle.consumed_track_ids
    assert {7, 8} <= env.simulator.consumed_track_ids


def test_results_for_every_overflow_compromised_id_remain_ignored_consumed():
    env = make_hitter_env_for_test(is_real=True)
    env.hitter_planner_worker.queue(
        result_batch(
            success_result(track_id=7),
            overflowed_track_ids=(8,),
        )
    )
    env._update_hitter_command(now=10.0)

    for track_id in (7, 8):
        env.hitter_planner_worker.queue(
            result_batch(success_result(track_id=track_id, generation=2))
        )
        env._update_hitter_command(now=10.1 + track_id * 0.001)

    assert env.hitter_command_lifecycle.phase is CommandPhase.WAITING
    assert env.hitter_command_lifecycle.active_result is None


def test_direct_track_end_event_beats_same_tick_success_result():
    env = make_hitter_env_for_test(is_real=True)
    env.simulator.runtime_events.append(
        SimpleNamespace(
            reason=LifecycleCancelReason.TRACK_ENDED,
            track_id=7,
            detail="ended",
        )
    )
    env.hitter_planner_worker.queue(result_batch(success_result(track_id=7)))

    env._update_hitter_command(now=10.0)

    assert env.hitter_command_lifecycle.phase is CommandPhase.WAITING
    assert env.hitter_command_lifecycle.active_result is None
    assert 7 in env.hitter_command_lifecycle.consumed_track_ids


def test_base_invalid_event_latches_anchor_fault_even_when_committed():
    env = make_hitter_env_for_test(is_real=True)
    env.hitter_planner_worker.queue(result_batch(success_result(track_id=7)))
    env._update_hitter_command(now=10.0)
    assert env.hitter_command_lifecycle.phase is CommandPhase.ARMED
    env.simulator.runtime_events.append(
        SimpleNamespace(
            reason=LifecycleCancelReason.BASE_POSE_INVALID,
            track_id=7,
            detail="invalid pelvis",
        )
    )

    env._update_hitter_command(now=10.61)

    assert env.hitter_command_lifecycle.phase is CommandPhase.ARMED
    assert env._waiting_anchor_fault is LifecycleCancelReason.BASE_POSE_INVALID
    assert 7 in env.simulator.consumed_track_ids


def test_central_decision_application_copies_only_lifecycle_active_command():
    env = make_hitter_env_for_test(is_real=True)
    incoming = success_result(track_id=7, generation=1, marker=0.0)
    decision = env.hitter_command_lifecycle.ingest(incoming, now=10.0)
    frozen_active = env.hitter_command_lifecycle.active_result
    assert frozen_active is not incoming

    env._apply_hitter_lifecycle_decision(decision, now=10.0)

    assert env.hitter_command_initialized is True
    assert env.hitter_strike_type == 0
    assert (
        env.hitter_strike_time_s
        == frozen_active.command.time_to_strike
    )


def _result_for_strike_side(side: str):
    result = success_result(track_id=41, generation=1, marker=0.0)
    command = result.command
    y_w = -0.10 if side == "forehand" else 0.10
    position = np.asarray(
        command.strike_plan.p_racket_target,
        dtype=np.float64,
    ).copy()
    position[1] = y_w
    plan = replace(command.strike_plan, p_racket_target=position)
    return replace(
        result,
        command=replace(
            command,
            strike_type=side,
            strike_table_y_w=y_w,
            strike_plan=plan,
        ),
    )


@pytest.mark.parametrize(
    "is_real,side",
    [
        (True, "forehand"),
        (True, "backhand"),
        (False, "forehand"),
        (False, "backhand"),
    ],
)
def test_policy_racket_velocity_magnitude_increment_is_real_world_only(
    is_real,
    side,
):
    env = make_hitter_env_for_test(is_real=is_real)
    env.racket_velocity_magnitude_increment_mps = 1.0
    env.forehand_policy_vx_offset_mps = 0.0
    env.backhand_policy_vx_offset_mps = 0.0
    env.backhand_policy_vy_decrement_mps = 0.0
    result = _result_for_strike_side(side)
    decision = env.hitter_command_lifecycle.ingest(result, now=10.0)
    env._apply_hitter_lifecycle_decision(decision, now=10.0)
    env.compute_observation()

    raw_velocity = np.asarray([1.0, 0.1, 0.2], dtype=np.float64)
    raw_speed = float(np.linalg.norm(raw_velocity))
    expected = (
        raw_velocity * ((raw_speed + 1.0) / raw_speed)
        if is_real
        else raw_velocity
    )
    np.testing.assert_allclose(env.obs_buf_dict["obs"][0, 13:16], expected)


@pytest.mark.parametrize("side", ["forehand", "backhand"])
def test_real_policy_racket_velocity_magnitude_increment_does_not_accumulate(
    side,
):
    env = make_hitter_env_for_test(is_real=True)
    env.racket_velocity_magnitude_increment_mps = 1.0
    env.forehand_policy_vx_offset_mps = 0.0
    env.backhand_policy_vx_offset_mps = 0.0
    env.backhand_policy_vy_decrement_mps = 0.0
    result = _result_for_strike_side(side)
    decision = env.hitter_command_lifecycle.ingest(result, now=10.0)
    env._apply_hitter_lifecycle_decision(decision, now=10.0)
    frozen = env.hitter_command_lifecycle.active_result.command

    env.compute_observation()
    first = env.obs_buf_dict["obs"][0, 13:16].copy()
    env.compute_observation()
    second = env.obs_buf_dict["obs"][0, 13:16].copy()

    raw_velocity = np.asarray([1.0, 0.1, 0.2], dtype=np.float64)
    raw_speed = float(np.linalg.norm(raw_velocity))
    expected = raw_velocity * ((raw_speed + 1.0) / raw_speed)
    np.testing.assert_allclose(first, expected)
    assert np.linalg.norm(first) == pytest.approx(raw_speed + 1.0)
    np.testing.assert_allclose(second, first)
    np.testing.assert_allclose(frozen.v_racket_target_w, raw_velocity)
    np.testing.assert_allclose(
        frozen.strike_plan.v_racket_target,
        raw_velocity,
    )


@pytest.mark.parametrize(
    "side_source,strike_type,strike_table_y_w",
    [
        ("unexpected-source", "forehand", -0.1),
        ("table_y", "backhand", -0.1),
        ("table_y", "forehand", 0.0),
    ],
)
def test_env_incompatible_side_contract_cancels_before_arming(
    side_source,
    strike_type,
    strike_table_y_w,
):
    env = make_hitter_env_for_test(is_real=True)
    result = success_result(track_id=7)
    command = result.command
    position = np.asarray(
        command.strike_plan.p_racket_target,
        dtype=np.float64,
    ).copy()
    position[1] = strike_table_y_w
    plan = replace(command.strike_plan, p_racket_target=position)
    malformed_command = replace(
        command,
        strike_type=strike_type,
        strike_table_y_w=strike_table_y_w,
        strike_side_source=side_source,
        strike_plan=plan,
    )
    env.hitter_planner_worker.queue(
        result_batch(replace(result, command=malformed_command))
    )

    env._update_hitter_command(now=10.0)

    assert env.hitter_command_lifecycle.phase is CommandPhase.WAITING
    assert env.hitter_command_lifecycle.active_result is None
    assert (
        env.hitter_command_lifecycle.last_cancel_reason
        is LifecycleCancelReason.INTERNAL_ERROR
    )
    assert 7 in env.hitter_command_lifecycle.consumed_track_ids
    assert 7 in env.simulator.consumed_track_ids


def test_forced_side_may_differ_from_table_y_rule():
    env = make_hitter_env_for_test(is_real=True)
    result = success_result(track_id=7)
    forced_command = replace(
        result.command,
        strike_type="backhand",
        strike_side_source="forced",
    )
    env.hitter_planner_worker.queue(
        result_batch(replace(result, command=forced_command))
    )

    env._update_hitter_command(now=10.0)

    assert env.hitter_command_lifecycle.phase is CommandPhase.ARMED
    assert env.hitter_command_lifecycle.active_result is not None


def test_invalid_base_status_cancels_precommit_without_an_event():
    env = make_hitter_env_for_test(is_real=True)
    env.hitter_planner_worker.queue(result_batch(success_result(track_id=7)))
    env._update_hitter_command(now=10.0)
    assert env.hitter_command_lifecycle.phase is CommandPhase.ARMED
    env.simulator.base_pose_valid = False

    env._update_hitter_command(now=10.1)

    assert env.hitter_command_lifecycle.phase is CommandPhase.WAITING
    assert env._waiting_anchor_fault is LifecycleCancelReason.BASE_POSE_INVALID
    assert 7 in env.simulator.consumed_track_ids


@pytest.mark.parametrize("new_track", [False, True])
def test_listener_consumes_every_snapshot_during_recovery(new_track):
    env = make_hitter_env_for_test(is_real=True)
    env.hitter_command_lifecycle.phase = CommandPhase.RECOVERY
    snapshot = make_snapshot(track_id=8, new_track=new_track)
    env.simulator.present(snapshot)

    env._submit_hitter_planner_snapshot(snapshot)

    assert env.hitter_planner_worker.submitted == []
    assert 8 in env.hitter_command_lifecycle.consumed_track_ids
    assert 8 in env.simulator.consumed_track_ids


def test_listener_consumes_new_track_when_phase_is_not_waiting():
    env = make_hitter_env_for_test(is_real=True)
    env.hitter_command_lifecycle.phase = CommandPhase.ARMED
    snapshot = make_snapshot(track_id=8, new_track=True)
    env.simulator.present(snapshot)

    env._submit_hitter_planner_snapshot(snapshot)

    assert env.hitter_planner_worker.submitted == []
    assert 8 in env.hitter_command_lifecycle.consumed_track_ids


def test_stale_snapshot_object_cannot_bypass_current_consumed_state():
    env = make_hitter_env_for_test(is_real=True)
    snapshot = make_snapshot(track_id=7, consumed=False, new_track=False)
    env.simulator.present(snapshot)
    env.hitter_command_lifecycle.consume_track(7, reason="already_done")

    env._submit_hitter_planner_snapshot(snapshot)

    assert env.hitter_planner_worker.submitted == []
    assert 7 in env.simulator.consumed_track_ids


def test_consumer_ineligible_snapshot_is_consumed_not_submitted():
    env = make_hitter_env_for_test(is_real=True)
    snapshot = make_snapshot(track_id=7)
    env.simulator.present(snapshot, admitted=False)

    env._submit_hitter_planner_snapshot(snapshot)

    assert env.hitter_planner_worker.submitted == []
    assert 7 in env.hitter_command_lifecycle.consumed_track_ids


def test_listener_rechecks_phase_after_waiting_outside_lifecycle_lock():
    env = make_hitter_env_for_test(is_real=True)
    snapshot = make_snapshot(track_id=8)
    env.simulator.present(snapshot)
    phase_checked = threading.Event()
    allow_submit = threading.Event()
    policy_started = threading.Event()

    def listener():
        phase_checked.set()
        assert allow_submit.wait(1.0)
        env._submit_hitter_planner_snapshot(snapshot)

    thread = threading.Thread(target=listener)
    thread.start()
    assert phase_checked.wait(1.0)
    with env._hitter_lifecycle_lock:
        env.hitter_command_lifecycle.phase = CommandPhase.RECOVERY
        policy_started.set()
    assert policy_started.wait(1.0)
    allow_submit.set()
    thread.join(1.0)

    assert not thread.is_alive()
    assert env.hitter_planner_worker.submitted == []
    assert 8 in env.simulator.consumed_track_ids


def test_submit_is_atomic_with_phase_check_under_lifecycle_lock():
    env = make_hitter_env_for_test(is_real=True)
    snapshot = make_snapshot(track_id=8)
    env.simulator.present(snapshot)
    phase_checked = threading.Event()
    allow_submit = threading.Event()
    policy_attempting = threading.Event()
    policy_started = threading.Event()

    def submit_hook(_snapshot):
        phase_checked.set()
        assert allow_submit.wait(1.0)

    env.hitter_planner_worker.submit_hook = submit_hook

    def policy():
        policy_attempting.set()
        with env._hitter_lifecycle_lock:
            env.hitter_command_lifecycle.phase = CommandPhase.RECOVERY
            policy_started.set()

    listener_thread = threading.Thread(
        target=env._submit_hitter_planner_snapshot,
        args=(snapshot,),
    )
    listener_thread.start()
    assert phase_checked.wait(1.0)
    policy_thread = threading.Thread(target=policy)
    policy_thread.start()
    assert policy_attempting.wait(1.0)
    assert not policy_started.wait(0.05)
    allow_submit.set()
    listener_thread.join(1.0)
    policy_thread.join(1.0)

    assert not listener_thread.is_alive()
    assert not policy_thread.is_alive()
    assert [item.track_id for item in env.hitter_planner_worker.submitted] == [8]
    assert policy_started.is_set()


def test_id_seen_before_no_ball_gate_never_resurrects_and_later_id_submits():
    env = make_hitter_env_for_test(is_real=True)
    too_early = make_snapshot(track_id=7, received=0.49)
    env.simulator.present(too_early, admitted=False)
    env._submit_hitter_planner_snapshot(too_early)
    assert 7 in env.hitter_command_lifecycle.consumed_track_ids

    same_id_later = make_snapshot(track_id=7, generation=2, received=0.50)
    env.simulator.present(same_id_later, admitted=True)
    env._submit_hitter_planner_snapshot(same_id_later)

    later_unseen = make_snapshot(track_id=8, received=0.51)
    env.simulator.present(later_unseen, admitted=True)
    env._submit_hitter_planner_snapshot(later_unseen)

    assert [item.track_id for item in env.hitter_planner_worker.submitted] == [8]


@pytest.mark.parametrize(
    "scenario",
    ["late_skip", "third_soft", "immediate", "third_discontinuity"],
)
def test_every_consuming_decision_is_synchronized_to_consumer(scenario):
    env = make_hitter_env_for_test(is_real=True)
    if scenario == "late_skip":
        results = (success_result(deadline=10.20),)
    elif scenario == "third_soft":
        results = (
            success_result(generation=1),
            failure_result(
                PlannerFailureReason.NO_FUTURE_CROSSING, generation=2
            ),
            failure_result(
                PlannerFailureReason.NO_FUTURE_CROSSING, generation=3
            ),
            failure_result(
                PlannerFailureReason.NO_FUTURE_CROSSING, generation=4
            ),
        )
    elif scenario == "immediate":
        results = (
            success_result(generation=1),
            failure_result(
                PlannerFailureReason.BASE_POSE_INVALID, generation=2
            ),
        )
    else:
        results = (
            success_result(generation=1),
            success_result(generation=2, marker=0.20),
            success_result(generation=3, marker=0.20),
            success_result(generation=4, marker=0.20),
        )
    env.hitter_planner_worker.queue(result_batch(*results))

    env._update_hitter_command(now=10.0)

    assert 7 in env.hitter_command_lifecycle.consumed_track_ids
    assert 7 in env.simulator.consumed_track_ids
    before = len(env.hitter_planner_worker.submitted)
    stale = make_snapshot(track_id=7, generation=99, new_track=False)
    env.simulator.present(stale)
    env._submit_hitter_planner_snapshot(stale)
    assert len(env.hitter_planner_worker.submitted) == before
    assert env.simulator.latest_snapshot.consumed is True


def test_strike_consumption_is_synchronized_before_recovery_completes():
    env = make_hitter_env_for_test(is_real=True)
    env.hitter_planner_worker.queue(result_batch(success_result(deadline=10.90)))
    env._update_hitter_command(now=10.0)

    env._update_hitter_command(now=10.90)

    assert env.hitter_command_lifecycle.phase is CommandPhase.RECOVERY
    assert 7 in env.simulator.consumed_track_ids
    assert env.simulator.reset_estimator_calls == 1


@pytest.mark.parametrize("is_real, expected_ticks", [(True, 0), (False, 1)])
def test_post_physics_has_no_second_real_lifecycle_tick(is_real, expected_ticks):
    env = make_hitter_env_for_test(is_real=is_real)
    ticks = []
    env._update_hitter_command = lambda: ticks.append("tick")
    env.compute_observation = lambda: None
    env._check_termination = lambda: None

    env._post_physics_step()

    assert len(ticks) == expected_ticks
