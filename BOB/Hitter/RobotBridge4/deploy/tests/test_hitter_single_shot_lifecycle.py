from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from simulator.real_world import ViconEventReason
from tests.hitter_test_factories import command, failure, success
from utils.hitter_realtime import (
    CommandPhase,
    HitterCommandLifecycle,
    LifecycleCancelReason,
    LifecycleDecision,
)
from utils.hitter_runtime_types import PlannerFailureReason


def lifecycle(**overrides) -> HitterCommandLifecycle:
    options = {
        "swing_duration_sampler": lambda: 1.85,
    }
    options.update(overrides)
    return HitterCommandLifecycle(**options)


def armed_lifecycle(
    *,
    track_id: int = 7,
    deadline: float = 2.0,
    now: float = 1.1,
    planned_command=None,
) -> HitterCommandLifecycle:
    item = lifecycle()
    decision = item.ingest(
        success(
            track_id=track_id,
            generation=1,
            deadline=deadline,
            planned_command=planned_command,
        ),
        now=now,
    )
    assert decision.kind == "armed"
    return item


def armed_lifecycle_before_commit(*, track_id: int = 7) -> HitterCommandLifecycle:
    return armed_lifecycle(track_id=track_id, deadline=2.0, now=1.1)


def lifecycle_in_recovery(*, track_id: int = 7) -> HitterCommandLifecycle:
    item = armed_lifecycle(track_id=track_id)
    assert item.advance(now=2.0).kind == "struck"
    assert item.phase is CommandPhase.RECOVERY
    return item


def lifecycle_after_terminal(
    *, track_id: int,
    terminal: str,
) -> HitterCommandLifecycle:
    if terminal == "skipped":
        item = lifecycle()
        assert item.ingest(
            success(track_id=track_id, deadline=1.299), now=1.0
        ).kind == "skipped"
    elif terminal == "cancelled":
        item = armed_lifecycle_before_commit(track_id=track_id)
        assert item.cancel(
            reason=LifecycleCancelReason.BASE_POSE_INVALID,
            now=1.1,
        ).kind == "cancelled"
    elif terminal == "struck":
        item = lifecycle_in_recovery(track_id=track_id)
        item.advance(now=3.0)
    elif terminal == "ended":
        item = lifecycle()
        assert item.ingest(
            success(track_id=track_id, deadline=3.0), now=1.0
        ).kind == "tracking"
        assert item.mark_track_ended(track_id, now=1.1).entered_waiting
    else:  # pragma: no cover - test helper guard
        raise AssertionError(terminal)
    return item


def override_result(
    active,
    *,
    generation: int,
    position_delta: float = 0.0,
    velocity_delta: float = 0.0,
    deadline_delta: float = 0.0,
    side: str | None = None,
):
    old_command = active.command
    old_plan = old_command.strike_plan
    new_position = np.asarray(old_plan.p_racket_target).copy()
    new_velocity = np.asarray(old_command.v_racket_target_w).copy()
    new_position[0] += position_delta
    if side is not None and side != old_command.strike_type:
        new_position[1] = 0.2 if side == "backhand" else -0.2
    new_velocity[0] += velocity_delta
    new_plan = replace(
        old_plan,
        t_strike=99.0,
        p_racket_target=new_position,
        v_racket_target=new_velocity.copy(),
        v_ball_in=np.array([91.0, 92.0, 93.0]),
        v_ball_out=np.array([94.0, 95.0, 96.0]),
    )
    new_command = replace(
        old_command,
        strike_type=old_command.strike_type if side is None else side,
        p_base_target_xy=np.array([8.0, 9.0]),
        v_racket_target_w=new_velocity.copy(),
        time_to_strike=0.01,
        strike_plan=new_plan,
        strike_table_y_w=float(new_position[1]),
        strike_side_source=old_command.strike_side_source,
    )
    return success(
        track_id=active.track_id,
        generation=generation,
        deadline=active.strike_deadline_monotonic_s + deadline_delta,
        planned_command=new_command,
    )


def test_cancel_reason_covers_planner_and_vicon_enums():
    for reason in PlannerFailureReason:
        assert LifecycleCancelReason(reason.value).value == reason.value
    for reason in ViconEventReason:
        assert LifecycleCancelReason(reason.value).value == reason.value


@pytest.mark.parametrize(
    "side_source,side,position_y",
    [
        ("unexpected-source", "forehand", -0.2),
        ("table_y", "backhand", -0.2),
        ("table_y", "forehand", 0.0),
    ],
)
def test_invalid_strike_side_contract_is_typed_internal_error_cancel(
    side_source,
    side,
    position_y,
):
    item = lifecycle()
    malformed = command(
        side=side,
        position=(0.0, position_y, 1.0),
        side_source=side_source,
    )

    decision = item.ingest(
        success(planned_command=malformed),
        now=1.1,
    )

    assert decision.kind == "cancelled"
    assert decision.cancel_reason is LifecycleCancelReason.INTERNAL_ERROR
    assert decision.consumed_track_ids == (7,)
    assert item.phase is CommandPhase.WAITING
    assert item.active_result is None


@pytest.mark.parametrize(
    "side_source,side,position_y,expected,consistent",
    [
        ("table_y", "forehand", -0.2, "forehand", True),
        ("table_y", "backhand", 0.0, "backhand", True),
        ("forced", "backhand", -0.2, "forehand", False),
    ],
)
def test_command_exposes_read_only_table_y_side_projection(
    side_source,
    side,
    position_y,
    expected,
    consistent,
):
    planned = command(
        side=side,
        position=(0.0, position_y, 1.0),
        side_source=side_source,
    )

    assert planned.expected_strike_type_from_table_y == expected
    assert planned.strike_type_consistent is consistent

    with pytest.raises(AttributeError):
        planned.expected_strike_type_from_table_y = "backhand"


def test_decision_validates_kind_identity_reason_and_consumed_ids():
    valid = LifecycleDecision(
        kind="cancelled",
        track_id=7,
        cancel_reason=LifecycleCancelReason.TRACK_ENDED,
        consumed_track_ids=(7, 8),
    )
    assert valid.consumed_track_ids == (7, 8)
    for kwargs in (
        {"kind": "invented", "track_id": None},
        {"kind": "none", "track_id": True},
        {"kind": "none", "track_id": None, "consumed_track_ids": (8, 7)},
        {"kind": "none", "track_id": None, "consumed_track_ids": (7, 7)},
        {"kind": "none", "track_id": None, "cancel_reason": "TRACK_ENDED"},
    ):
        with pytest.raises((TypeError, ValueError)):
            LifecycleDecision(**kwargs)


def test_defaults_and_idle_decisions_match_final_contract():
    item = lifecycle()
    assert item.waiting_tts == 0.92
    assert item.arm_tts == 0.92
    assert item.minimum_arm_tts == 0.30
    assert item.maximum_policy_tts == 0.92
    assert item.armed_cancel_consecutive_failures == 3
    assert item.commit_time_to_strike_s == 0.30
    assert item.maximum_racket_target_override_delta_m == 0.05
    assert item.maximum_racket_velocity_override_delta_mps == 0.75
    assert item.maximum_strike_deadline_override_delta_s == 0.05
    assert item.advance(now=0.0).kind == "none"
    assert item.policy_tts(now=0.0) == 0.92


def test_waiting_policy_tts_still_rejects_nonfinite_time():
    with pytest.raises(ValueError):
        lifecycle().policy_tts(now=float("nan"))


@pytest.mark.parametrize(
    "kwargs",
    [
        {"waiting_tts": float("nan")},
        {"maximum_policy_tts": float("inf")},
        {"commit_time_to_strike_s": -0.01},
        {"commit_time_to_strike_s": 0.31},
        {"minimum_arm_tts": 0.93},
        {"arm_tts": 0.29},
        {"armed_cancel_consecutive_failures": True},
        {"armed_cancel_consecutive_failures": 1.5},
        {"armed_cancel_consecutive_failures": 0},
        {"maximum_racket_target_override_delta_m": -0.01},
        {"maximum_racket_velocity_override_delta_mps": float("nan")},
        {"maximum_strike_deadline_override_delta_s": float("inf")},
    ],
)
def test_constructor_rejects_invalid_contract_values(kwargs):
    with pytest.raises((TypeError, ValueError)):
        lifecycle(**kwargs)


def test_ingest_does_not_implicitly_advance():
    item = armed_lifecycle()
    result = success(track_id=8, generation=1, deadline=4.0)
    decision = item.ingest(result, now=3.0)
    assert decision.kind == "ignored_track"
    assert item.phase is CommandPhase.ARMED
    assert item.strike_count_by_track_id.get(7, 0) == 0


@pytest.mark.parametrize("terminal", ["skipped", "cancelled", "struck", "ended"])
def test_terminal_track_id_can_never_arm_again(terminal):
    item = lifecycle_after_terminal(track_id=7, terminal=terminal)
    decision = item.ingest(success(track_id=7, generation=99), now=2.0)
    assert decision.kind == "ignored_consumed"
    assert item.phase is CommandPhase.WAITING
    assert 7 in item.consumed_track_ids


def test_late_boundary_uses_strict_less_than_and_consumes():
    exact = lifecycle()
    assert exact.ingest(
        success(deadline=1.30), now=1.0
    ).kind == "armed"

    late = lifecycle()
    decision = late.ingest(
        success(deadline=np.nextafter(1.30, -np.inf)), now=1.0
    )
    assert decision.kind == "skipped"
    assert decision.entered_waiting
    assert decision.consumed_track_ids == (7,)


def test_failure_and_success_share_generation_watermark():
    item = armed_lifecycle_before_commit()
    first = item.ingest(
        failure(generation=2, reason=PlannerFailureReason.BALL_NOT_INCOMING),
        now=1.1,
    )
    assert first.kind == "retained_failure"
    assert item.consecutive_failure_count == 1
    assert item.ingest(success(generation=2), now=1.1).kind == "ignored_generation"
    assert item.consecutive_failure_count == 1
    assert item.ingest(failure(generation=1), now=1.1).kind == "ignored_generation"
    assert item.consecutive_failure_count == 1
    assert item.ingest(
        success(generation=3, deadline=2.0), now=1.1
    ).kind == "overridden"
    assert item.consecutive_failure_count == 0


def test_recovery_consumes_new_id_without_caching():
    item = lifecycle_in_recovery(track_id=7)
    decision = item.ingest(success(track_id=8, generation=1), now=2.1)
    assert decision.kind == "consumed_during_recovery"
    assert decision.consumed_track_ids == (8,)
    assert 8 in item.consumed_track_ids
    assert not hasattr(item, "cached_result")
    item.advance(now=3.0)
    assert item.phase is CommandPhase.WAITING
    assert item.active_result is None


def test_advance_crosses_only_one_phase_even_when_late_and_recovery_is_zero():
    item = lifecycle(swing_duration_sampler=lambda: 0.30)
    assert item.ingest(success(deadline=1.30), now=1.0).kind == "armed"
    first = item.advance(now=99.0)
    assert first.kind == "struck"
    assert first.consumed_track_ids == (7,)
    assert item.phase is CommandPhase.RECOVERY
    assert item.strike_count_by_track_id[7] == 1
    second = item.advance(now=99.0)
    assert second.kind == "entered_waiting"
    assert second.entered_waiting
    assert second.consumed_track_ids == ()
    assert item.strike_count_by_track_id[7] == 1
    assert item.advance(now=100.0).kind == "none"


@pytest.mark.parametrize(
    "reason",
    [
        LifecycleCancelReason.TRACK_ENDED,
        LifecycleCancelReason.VICON_STREAM_STALE,
        LifecycleCancelReason.TRACK_ID_CONFLICT,
        LifecycleCancelReason.RESULT_QUEUE_OVERFLOW,
    ],
)
def test_recovery_cancel_is_retained_and_consumes_event_id(reason):
    item = lifecycle_in_recovery()
    deadline = item.command_end_deadline_s
    decision = item.cancel(reason=reason, now=2.1, track_id=8)
    assert decision.kind == "retained_recovery"
    assert not decision.entered_waiting
    assert decision.consumed_track_ids == (7, 8)
    assert item.phase is CommandPhase.RECOVERY
    assert item.command_end_deadline_s == deadline


def test_first_arm_locks_side_base_deadline_and_copies_arrays():
    source = command(
        "forehand",
        [-0.4, -0.2],
        [0.0, -0.2, 1.0],
        [1.0, 0.0, 0.5],
    )
    item = armed_lifecycle(planned_command=source)
    source.p_base_target_xy[:] = 99.0
    source.strike_plan.p_racket_target[:] = 99.0
    source.v_racket_target_w[:] = 99.0
    assert item.locked_strike_type == "forehand"
    np.testing.assert_allclose(item.locked_base_target_xy, [-0.4, -0.2])
    assert item.locked_strike_deadline_monotonic_s == 2.0
    np.testing.assert_allclose(
        item.active_result.command.strike_plan.p_racket_target,
        [0.0, -0.2, 1.0],
    )


def test_continuous_override_only_updates_position_and_velocity():
    item = armed_lifecycle_before_commit()
    old = item.active_result
    old_command = old.command
    updated = override_result(
        old,
        generation=2,
        position_delta=0.05,
        velocity_delta=0.75,
        deadline_delta=0.05,
    )
    decision = item.ingest(updated, now=1.1)
    assert decision.kind == "overridden"
    assert decision.command_changed
    active = item.active_result
    assert active.strike_deadline_monotonic_s == 2.0
    assert active.command.strike_type == old_command.strike_type
    np.testing.assert_array_equal(
        active.command.p_base_target_xy,
        old_command.p_base_target_xy,
    )
    np.testing.assert_array_equal(
        active.command.v_racket_target_w,
        active.command.strike_plan.v_racket_target,
    )
    assert active.command.strike_table_y_w == active.command.strike_plan.p_racket_target[1]
    assert active.command.time_to_strike == old_command.time_to_strike
    assert active.command.strike_plan.t_strike == old_command.strike_plan.t_strike
    np.testing.assert_array_equal(
        active.command.strike_plan.v_ball_in,
        old_command.strike_plan.v_ball_in,
    )
    np.testing.assert_array_equal(
        active.command.strike_plan.v_ball_out,
        old_command.strike_plan.v_ball_out,
    )
    assert active.command.strike_side_source == old_command.strike_side_source
    updated.command.strike_plan.p_racket_target[:] = -77.0
    updated.command.v_racket_target_w[:] = -88.0
    assert active.command.strike_plan.p_racket_target[0] != -77.0
    assert active.command.v_racket_target_w[0] != -88.0


def test_override_cannot_change_side_source_and_break_table_y_invariant():
    item = armed_lifecycle(
        planned_command=command(
            side="forehand",
            position=(0.0, -0.01, 1.0),
            side_source="table_y",
        )
    )
    active = item.active_result
    forced_candidate = command(
        side="forehand",
        position=(0.0, 0.01, 1.0),
        side_source="forced",
    )

    decision = item.ingest(
        success(
            generation=2,
            deadline=2.0,
            planned_command=forced_candidate,
        ),
        now=1.1,
    )

    assert decision.kind == "retained_discontinuity"
    assert item.active_result is active
    assert item.active_result.command.strike_side_source == "table_y"
    assert item.active_result.command.strike_type == "forehand"
    assert item.active_result.command.strike_table_y_w < 0.0


@pytest.mark.parametrize(
    "change",
    [
        {"position_delta": 0.050001},
        {"velocity_delta": 0.750001},
        {"deadline_delta": 0.050001},
        {"side": "backhand"},
    ],
)
def test_override_discontinuity_retains_last_good_without_counting(change):
    item = armed_lifecycle_before_commit()
    active = item.active_result
    for generation in (2, 3, 4):
        decision = item.ingest(
            override_result(active, generation=generation, **change), now=1.1
        )
        assert decision.kind == "retained_discontinuity"
        assert item.consecutive_failure_count == 0
        assert item.active_result is active
        assert item.phase is CommandPhase.ARMED


def test_hit_height_out_of_range_retains_last_good_without_counting():
    item = armed_lifecycle_before_commit()
    active = item.active_result
    assert item.ingest(
        failure(
            generation=2,
            reason=PlannerFailureReason.BALL_NOT_INCOMING,
        ),
        now=1.1,
    ).kind == "retained_failure"
    assert item.consecutive_failure_count == 1

    for generation in (3, 4, 5):
        decision = item.ingest(
            failure(
                generation=generation,
                reason=PlannerFailureReason.HIT_HEIGHT_OUT_OF_RANGE,
            ),
            now=1.1,
        )
        assert decision.kind == "retained_failure"
        assert item.consecutive_failure_count == 1
        assert item.active_result is active
        assert item.phase is CommandPhase.ARMED


@pytest.mark.parametrize(
    "reason",
    [
        PlannerFailureReason.TRACK_ENDED,
        PlannerFailureReason.BASE_POSE_INVALID,
        PlannerFailureReason.NONFINITE_INPUT_OR_OUTPUT,
        PlannerFailureReason.INTERNAL_ERROR,
    ],
)
def test_immediate_planner_failure_cancels_precommit(reason):
    item = armed_lifecycle_before_commit()
    decision = item.ingest(failure(generation=2, reason=reason), now=1.1)
    assert decision.kind == "cancelled"
    assert decision.entered_waiting
    assert decision.cancel_reason is LifecycleCancelReason(reason.value)
    assert item.last_failure_reason is LifecycleCancelReason(reason.value)


def test_only_matching_active_soft_failures_accumulate():
    item = armed_lifecycle_before_commit()
    active = item.active_result
    mismatch = item.ingest(
        failure(
            track_id=8,
            generation=1,
            reason=PlannerFailureReason.BALL_NOT_INCOMING,
        ),
        now=1.1,
    )
    assert mismatch.kind == "ignored_track"
    assert mismatch.consumed_track_ids == (8,)
    assert 8 in item.consumed_track_ids
    assert item.consecutive_failure_count == 0
    assert item.active_result is active
    for generation in (2, 3):
        assert item.ingest(
            failure(
                generation=generation,
                reason=PlannerFailureReason.BALL_NOT_INCOMING,
            ),
            now=1.1,
        ).kind == "retained_failure"
    decision = item.ingest(
        failure(generation=4, reason=PlannerFailureReason.BALL_NOT_INCOMING),
        now=1.1,
    )
    assert decision.kind == "cancelled"
    assert decision.cancel_reason is LifecycleCancelReason.BALL_NOT_INCOMING


def test_armed_challenger_success_is_consumed_without_changing_active():
    item = armed_lifecycle_before_commit()
    active = item.active_result
    decision = item.ingest(
        success(track_id=8, generation=1, deadline=2.0),
        now=1.1,
    )
    assert decision.kind == "ignored_track"
    assert decision.consumed_track_ids == (8,)
    assert 8 in item.consumed_track_ids
    assert item.active_result is active
    assert item.consecutive_failure_count == 0


@pytest.mark.parametrize("current_kind", ["soft_failure", "tracking_success"])
@pytest.mark.parametrize("challenger_kind", ["success", "failure"])
def test_tracking_challenger_is_consumed_permanently(
    current_kind,
    challenger_kind,
):
    item = lifecycle()
    if current_kind == "soft_failure":
        first = failure(
            track_id=7,
            generation=1,
            reason=PlannerFailureReason.ESTIMATOR_NOT_READY,
        )
        assert item.ingest(first, now=1.0).kind == "retained_failure"
    else:
        assert item.ingest(
            success(track_id=7, generation=1, deadline=3.0),
            now=1.0,
        ).kind == "tracking"
    assert item.phase is CommandPhase.TRACKING

    challenger = (
        success(track_id=8, generation=1, deadline=3.0)
        if challenger_kind == "success"
        else failure(
            track_id=8,
            generation=1,
            reason=PlannerFailureReason.BALL_NOT_INCOMING,
        )
    )
    decision = item.ingest(challenger, now=1.1)
    assert decision.kind == "ignored_track"
    assert decision.consumed_track_ids == (8,)
    assert item.phase is CommandPhase.TRACKING
    assert item.active_result is None
    assert item.consecutive_failure_count == 0

    assert item.mark_track_ended(7, now=1.2).entered_waiting
    retry = item.ingest(
        success(track_id=8, generation=99, deadline=3.0),
        now=1.3,
    )
    assert retry.kind == "ignored_consumed"
    assert retry.consumed_track_ids == (8,)


def test_conflict_event_consumes_active_and_event_ids_atomically():
    item = armed_lifecycle_before_commit(track_id=7)
    decision = item.cancel(
        reason=LifecycleCancelReason.TRACK_ID_CONFLICT,
        now=1.1,
        track_id=8,
    )
    assert decision.kind == "cancelled"
    assert decision.entered_waiting
    assert decision.consumed_track_ids == (7, 8)
    assert {7, 8} <= item.consumed_track_ids
    assert item.ingest(success(track_id=7, generation=2), now=1.1).kind == "ignored_consumed"
    assert item.ingest(success(track_id=8, generation=1), now=1.1).kind == "ignored_consumed"


def test_tracking_cancel_uses_current_identity_and_duplicate_has_no_waiting_edge():
    item = lifecycle()
    assert item.ingest(success(deadline=3.0), now=1.0).kind == "tracking"
    decision = item.cancel(
        reason=LifecycleCancelReason.VICON_STREAM_STALE,
        now=1.1,
    )
    assert decision.kind == "cancelled"
    assert decision.track_id == 7
    assert decision.entered_waiting
    assert decision.consumed_track_ids == (7,)
    repeated = item.mark_track_ended(7, now=1.2)
    assert not repeated.entered_waiting
    assert repeated.consumed_track_ids == (7,)


def test_first_direct_waiting_track_end_has_one_semantic_waiting_edge():
    item = lifecycle()
    first = item.mark_track_ended(7, now=0.0)
    assert first.kind == "cancelled"
    assert first.entered_waiting
    assert first.consumed_track_ids == (7,)
    repeated = item.mark_track_ended(7, now=0.1)
    assert repeated.kind == "cancelled"
    assert not repeated.entered_waiting
    assert repeated.consumed_track_ids == (7,)


def test_commit_boundary_and_nextafter_choose_exact_sides():
    committed = armed_lifecycle(deadline=2.0, now=1.1)
    before = committed.active_result
    decision = committed.ingest(
        override_result(before, generation=2, position_delta=0.01),
        now=1.70,
    )
    assert decision.kind == "retained_committed"
    assert committed.active_result is before

    precommit = armed_lifecycle(deadline=2.0, now=1.1)
    before = precommit.active_result
    decision = precommit.ingest(
        override_result(before, generation=2, position_delta=0.01),
        now=np.nextafter(1.70, -np.inf),
    )
    assert decision.kind == "overridden"


def test_commit_freezes_success_failure_cancel_and_uses_locked_deadline():
    item = armed_lifecycle(deadline=2.0, now=1.1)
    for generation in (2, 3):
        assert item.ingest(
            failure(
                generation=generation,
                reason=PlannerFailureReason.BALL_NOT_INCOMING,
            ),
            now=1.1,
        ).kind == "retained_failure"
    assert item.consecutive_failure_count == 2
    frozen = item.active_result
    success_decision = item.ingest(
        override_result(frozen, generation=4, position_delta=0.01), now=1.70
    )
    assert success_decision.kind == "retained_committed"
    assert item.active_result is frozen
    assert item.consecutive_failure_count == 2
    failure_decision = item.ingest(
        failure(generation=5, reason=PlannerFailureReason.INTERNAL_ERROR),
        now=1.70,
    )
    assert failure_decision.kind == "retained_committed"
    assert item.active_result is frozen
    assert item.consecutive_failure_count == 2
    assert item.last_failure_reason is LifecycleCancelReason.INTERNAL_ERROR
    cancel_decision = item.cancel(
        reason=LifecycleCancelReason.TRACK_ID_CONFLICT,
        now=1.70,
        track_id=8,
    )
    assert cancel_decision.kind == "retained_committed"
    assert cancel_decision.consumed_track_ids == (7, 8)
    assert item.phase is CommandPhase.ARMED
    assert item.active_result is frozen
    assert item.policy_tts(now=1.75) == pytest.approx(0.25)
    assert item.advance(now=2.0).kind == "struck"
    assert item.strike_count_by_track_id[7] == 1


@pytest.mark.parametrize(
    "malform",
    ["command_type", "nonfinite", "velocity_mismatch", "table_y_mismatch"],
)
def test_malformed_commit_success_is_retained_but_consumes_active(malform):
    item = armed_lifecycle(deadline=2.0, now=1.1)
    active = item.active_result
    valid_command = command()
    if malform == "command_type":
        malformed_command = object()
    elif malform == "nonfinite":
        malformed_command = replace(
            valid_command,
            v_racket_target_w=np.array([np.nan, 0.0, 0.5]),
        )
    elif malform == "velocity_mismatch":
        malformed_command = replace(
            valid_command,
            v_racket_target_w=np.array([9.0, 8.0, 7.0]),
        )
    else:
        malformed_command = replace(valid_command, strike_table_y_w=99.0)
    malformed = success(
        track_id=7,
        generation=2,
        deadline=2.0,
        planned_command=malformed_command,
    )

    decision = item.ingest(malformed, now=1.70)

    assert decision.kind == "retained_committed"
    assert decision.cancel_reason is LifecycleCancelReason.INTERNAL_ERROR
    assert decision.consumed_track_ids == (7,)
    assert item.last_failure_reason is LifecycleCancelReason.INTERNAL_ERROR
    assert item.last_cancel_reason is LifecycleCancelReason.INTERNAL_ERROR
    assert item.phase is CommandPhase.ARMED
    assert item.active_result is active
    assert item.advance(now=2.0).kind == "struck"
    assert item.strike_count_by_track_id[7] == 1
    assert item.advance(now=2.1).kind == "none"
    assert item.strike_count_by_track_id[7] == 1


@pytest.mark.parametrize("state", ["tracking", "armed", "commit", "recovery"])
def test_policy_reentry_reset_clears_transient_state_but_preserves_history(state):
    item = lifecycle()
    assert item.ingest(
        success(track_id=6, deadline=0.5), now=0.0
    ).kind == "armed"
    assert item.advance(now=0.5).kind == "struck"
    assert item.advance(now=2.0).kind == "entered_waiting"
    old_consumption_reason = item.last_consumption_reason_by_track_id[6]

    if state == "tracking":
        assert item.ingest(
            success(track_id=7, deadline=4.0), now=2.0
        ).kind == "tracking"
        reset_now = 2.1
    else:
        assert item.ingest(
            success(track_id=7, deadline=2.9), now=2.0
        ).kind == "armed"
        reset_now = 2.6 if state == "commit" else 2.1
        if state == "recovery":
            assert item.advance(now=2.9).kind == "struck"
            reset_now = 3.0

    locked_side = item.locked_strike_type
    decision = item.reset_for_policy_reentry(now=reset_now)
    assert decision.kind == "session_reset"
    assert decision.entered_waiting
    assert decision.consumed_track_ids == (7,)
    assert item.phase is CommandPhase.WAITING
    assert item.active_result is None
    assert item.command_end_deadline_s is None
    assert item.consecutive_failure_count == 0
    assert item.ingest(
        success(track_id=7, generation=99, deadline=4.0), now=2.1
    ).kind == "ignored_consumed"
    assert item.strike_count_by_track_id[6] == 1
    assert item.last_consumption_reason_by_track_id[6] == old_consumption_reason
    assert item.locked_strike_type == locked_side


def test_empty_policy_reentry_reset_still_reports_waiting_edge():
    item = lifecycle()
    decision = item.reset_for_policy_reentry(now=0.0)
    assert decision.kind == "session_reset"
    assert decision.entered_waiting
    assert decision.consumed_track_ids == ()


def test_policy_reentry_reset_preserves_generation_and_diagnostic_history():
    item = armed_lifecycle(track_id=6)
    item.ingest(
        failure(
            track_id=6,
            generation=2,
            reason=PlannerFailureReason.BALL_NOT_INCOMING,
        ),
        now=1.1,
    )
    item.cancel(
        reason=LifecycleCancelReason.VICON_SCHEMA_ERROR,
        now=1.1,
    )
    previous_reason = item.last_consumption_reason_by_track_id[6]
    assert item.ingest(
        success(track_id=7, generation=4, deadline=2.0), now=1.1
    ).kind == "armed"
    locked_deadline = item.locked_strike_deadline_monotonic_s

    item.reset_for_policy_reentry(now=1.2)

    assert item.generation_watermark_by_track_id == {6: 2, 7: 4}
    assert item.last_failure_reason is LifecycleCancelReason.BALL_NOT_INCOMING
    assert item.last_cancel_reason is LifecycleCancelReason.VICON_SCHEMA_ERROR
    assert item.last_consumption_reason_by_track_id[6] == previous_reason
    assert item.locked_track_id == 7
    assert item.locked_strike_deadline_monotonic_s == locked_deadline


@pytest.mark.parametrize("bad_id", [True, 7.0, "7", 0, -1])
def test_public_track_id_inputs_require_exact_positive_int(bad_id):
    item = lifecycle()
    with pytest.raises((TypeError, ValueError)):
        item.consume_track(bad_id, reason="test")
    with pytest.raises((TypeError, ValueError)):
        item.cancel(
            reason=LifecycleCancelReason.TRACK_ENDED,
            now=0.0,
            track_id=bad_id,
        )
    with pytest.raises((TypeError, ValueError)):
        item.mark_track_ended(bad_id, now=0.0)


@pytest.mark.parametrize(
    "sampler",
    [
        lambda: float("nan"),
        lambda: float("inf"),
        lambda: -0.01,
        lambda: "bad",
    ],
)
def test_invalid_sampler_result_becomes_internal_error_cancel(sampler):
    item = lifecycle(swing_duration_sampler=sampler)
    decision = item.ingest(success(deadline=1.9), now=1.0)
    assert decision.kind == "cancelled"
    assert decision.entered_waiting
    assert decision.cancel_reason is LifecycleCancelReason.INTERNAL_ERROR
    assert decision.consumed_track_ids == (7,)


def test_sampler_exception_does_not_escape_policy_tick():
    def explode():
        raise RuntimeError("sampler exploded")

    item = lifecycle(swing_duration_sampler=explode)
    decision = item.ingest(success(deadline=1.9), now=1.0)
    assert decision.kind == "cancelled"
    assert decision.cancel_reason is LifecycleCancelReason.INTERNAL_ERROR
    assert item.phase is CommandPhase.WAITING
