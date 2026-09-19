from __future__ import annotations

from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from simulator import real_world as real_world_module
from unitree_sdk2.lcm_types.rc_command_lcmt import rc_command_lcmt
from unitree_sdk2.lcm_types.transformation_t import transformation_t
from utils import hitter_runtime_factory


def ball(
    *,
    track_id: int,
    frame: int,
    valid: bool = True,
    position=(1.5, 0.0, 1.0),
) -> transformation_t:
    message = transformation_t()
    message.name = "ball"
    message.track_id = track_id
    message.vicon_frame_number = frame
    message.vicon_time_s = frame / 100.0
    message.publish_time_us = frame * 10_000
    message.valid = int(valid)
    message.occluded = int(not valid)
    message.pos_vicon = list(position)
    message.quat_vicon = [0.0, 0.0, 0.0, 1.0]
    return message


def pelvis(
    *,
    frame: int,
    track_id: int = 0,
    valid: bool = True,
) -> transformation_t:
    message = transformation_t()
    message.name = "G2Pelvis"
    message.track_id = track_id
    message.vicon_frame_number = frame
    message.vicon_time_s = frame / 100.0
    message.publish_time_us = frame * 10_000
    message.valid = int(valid)
    message.occluded = int(not valid)
    message.pos_vicon = [0.2, 0.0, 0.8]
    message.quat_vicon = [0.0, 0.0, 0.0, 1.0]
    return message


def table(*, frame: int, track_id: int = 0) -> transformation_t:
    message = pelvis(frame=frame, track_id=track_id)
    message.name = "table"
    return message


def encoded_rc(*, r2: bool) -> bytes:
    message = rc_command_lcmt()
    message.right_lower_right_switch = int(r2)
    return message.encode()


def ingest_valid_pelvis(world, *, now: float, frame: int = 1) -> None:
    world._ingest_vicon_v2_message(
        pelvis(frame=frame),
        received_monotonic_s=now,
    )


def ingest_valid_pelvis_and_ball(
    world,
    *,
    now: float,
    track_id: int,
    frame: int = 10,
) -> None:
    ingest_valid_pelvis(world, now=now, frame=frame)
    world._ingest_vicon_v2_message(
        ball(track_id=track_id, frame=frame),
        received_monotonic_s=now,
    )


def open_session(world, *, start: float = 0.0) -> None:
    ingest_valid_pelvis(world, now=start, frame=1)
    assert world.begin_hitter_policy_session(now_monotonic_s=start)


def admit_ball(
    world,
    *,
    track_id: int = 7,
    now: float = 0.5,
    frame: int = 10,
) -> None:
    ingest_valid_pelvis(world, now=now - 0.01, frame=frame - 1)
    world._ingest_vicon_v2_message(
        ball(track_id=track_id, frame=frame),
        received_monotonic_s=now,
    )


@pytest.fixture
def world():
    simulator = real_world_module.RealWorld.__new__(real_world_module.RealWorld)
    simulator.cfg = SimpleNamespace(
        motion={
            "ball_planner": {
                "state_estimator_window_size": 3,
                "state_estimator_min_samples": 3,
                "state_estimator_sample_rate_hz": 100.0,
            },
            "vicon_consumer": {"base_subject": "G2Pelvis"},
        }
    )
    simulator._init_ball_state()
    for name in (
        "left_upper_switch",
        "left_lower_left_switch",
        "left_lower_right_switch",
        "right_upper_switch",
        "right_lower_left_switch",
        "right_lower_right_switch",
        "left_upper_switch_pressed",
        "left_lower_left_switch_pressed",
        "left_lower_right_switch_pressed",
        "right_upper_switch_pressed",
        "right_lower_left_switch_pressed",
        "right_lower_right_switch_pressed",
    ):
        setattr(simulator, name, 0)
    simulator.mode = 0
    simulator.left_stick = [0.0, 0.0]
    simulator.right_stick = [0.0, 0.0]
    snapshots = []
    unregister = simulator.register_hitter_ball_listener(snapshots.append)
    simulator.test_listener_snapshots = snapshots
    try:
        yield simulator
    finally:
        unregister()
        close = getattr(simulator, "close", None)
        if callable(close) and hasattr(simulator, "_communication_close_lock"):
            close()


def test_same_wire_id_survives_reset_and_rejects_out_of_order_frames(world):
    open_session(world)
    admit_ball(world, track_id=7, now=0.5, frame=10)
    world._ingest_vicon_v2_message(ball(track_id=7, frame=11), received_monotonic_s=0.51)
    generation = world.ball_snapshot_generation
    samples = world.ball_state_estimator.sample_count

    world._ingest_vicon_v2_message(ball(track_id=7, frame=11), received_monotonic_s=0.52)
    world._ingest_vicon_v2_message(ball(track_id=7, frame=9), received_monotonic_s=0.53)

    assert world.ball_snapshot_generation == generation
    assert world.ball_state_estimator.sample_count == samples
    world.reset_ball_state_estimator()
    assert world.active_ball_track_id == 7
    assert world.last_ball_track_id == 7
    assert world.seen_ball_track_ids == {7}
    assert world.ball_snapshot_generation == generation


def test_overlapping_new_id_consumes_both_and_latches_conflict(world):
    world._ingest_vicon_v2_message(ball(track_id=7, frame=10), received_monotonic_s=1.0)
    world._ingest_vicon_v2_message(ball(track_id=8, frame=11), received_monotonic_s=1.01)

    status = world.hitter_vicon_status(now_monotonic_s=1.01)
    assert {7, 8} <= world.consumed_ball_track_ids
    assert status.latched_fault is real_world_module.ViconInputFault.TRACK_ID_CONFLICT
    assert world.ball_state_estimator.sample_count == 0
    assert [(event.reason, event.track_id) for event in world.drain_hitter_vicon_events()] == [
        (real_world_module.ViconEventReason.TRACK_ID_CONFLICT, 8)
    ]


def test_seen_challenger_conflict_watermark_rejects_later_replay(world):
    world._ingest_vicon_v2_message(ball(track_id=8, frame=10), received_monotonic_s=0.0)
    world._ingest_vicon_v2_message(
        ball(track_id=8, frame=11, valid=False),
        received_monotonic_s=0.01,
    )
    world.drain_hitter_vicon_events()
    ingest_valid_pelvis(world, now=0.02, frame=2)
    assert world.begin_hitter_policy_session(now_monotonic_s=0.02)
    admit_ball(world, track_id=7, now=0.52, frame=20)
    incumbent_snapshot = world.latest_ball_snapshot
    listener_count = len(world.test_listener_snapshots)
    challenger_generation = world.ball_track_generations[8]

    assert 7 in world.admitted_ball_track_ids
    assert 7 not in world.consumed_ball_track_ids
    assert incumbent_snapshot.track_id == 7
    assert not incumbent_snapshot.consumed

    world._ingest_vicon_v2_message(
        ball(track_id=8, frame=12),
        received_monotonic_s=0.53,
    )

    assert {7, 8} <= world.consumed_ball_track_ids
    assert world.active_ball_track_id == 7
    assert world.last_ball_track_id == 7
    assert world.ball_visible_tmp
    assert world._last_ball_vicon_monotonic_s == 0.52
    assert world.ball_state_estimator.sample_count == 0
    assert len(world.test_listener_snapshots) == listener_count
    assert world.ball_track_last_frames[8] == 12
    assert world.ball_track_generations[8] == challenger_generation
    assert world.latest_ball_snapshot.track_id == 7
    assert world.latest_ball_snapshot.generation == incumbent_snapshot.generation
    assert world.latest_ball_snapshot.consumed
    assert world.vicon_fault_latched is real_world_module.ViconInputFault.TRACK_ID_CONFLICT
    assert [(event.reason, event.track_id) for event in world.drain_hitter_vicon_events()] == [
        (real_world_module.ViconEventReason.TRACK_ID_CONFLICT, 8)
    ]

    world._ingest_vicon_v2_message(
        ball(track_id=7, frame=21, valid=False),
        received_monotonic_s=0.54,
    )
    ingest_valid_pelvis(world, now=0.55, frame=22)
    assert world.begin_hitter_policy_session(now_monotonic_s=0.55)
    ingest_valid_pelvis(world, now=1.05, frame=23)
    world._ingest_vicon_v2_message(
        ball(track_id=8, frame=11),
        received_monotonic_s=1.051,
    )

    assert world.active_ball_track_id is None
    assert world.last_ball_track_id == 7
    assert world.ball_track_last_frames[8] == 12
    assert world.ball_track_generations[8] == challenger_generation
    assert len(world.test_listener_snapshots) == listener_count


def test_unseen_challenger_conflict_records_diagnostics_without_taking_authority(world):
    world._ingest_vicon_v2_message(ball(track_id=7, frame=20), received_monotonic_s=1.0)
    incumbent_generation = world.ball_snapshot_generation

    world._ingest_vicon_v2_message(ball(track_id=8, frame=30), received_monotonic_s=1.01)

    assert world.active_ball_track_id == 7
    assert world.last_ball_track_id == 7
    assert world.seen_ball_track_ids == {7, 8}
    assert world.ball_track_last_frames[8] == 30
    assert 8 not in world.ball_track_generations
    assert world.ball_snapshot_generation == incumbent_generation
    assert world.latest_ball_snapshot.track_id == 7
    assert world.latest_ball_snapshot.consumed


def test_bad_fingerprint_latches_schema_error_without_raising(world):
    world._vicon_state_handler("vicon_state_data_v2", b"bad fingerprint")

    assert world.vicon_fault_latched is real_world_module.ViconInputFault.VICON_SCHEMA_ERROR
    world._remote_controller_handler("rc_command_data", encoded_rc(r2=True))
    assert world.right_lower_right_switch_pressed is True


def test_schema_fault_atomically_quarantines_active_planner_eligibility(world):
    open_session(world)
    admit_ball(world, track_id=7, now=0.5, frame=10)
    incumbent_generation = world.ball_track_generations[7]
    incumbent_snapshot = world.latest_ball_snapshot
    listener_count = len(world.test_listener_snapshots)

    assert 7 in world.admitted_ball_track_ids
    assert 7 not in world.consumed_ball_track_ids
    assert world.ball_state_estimator.sample_count == 1
    assert not world.latest_ball_snapshot.consumed
    assert world.hitter_snapshot_planning_eligible(incumbent_snapshot)

    malformed_table = table(frame=11, track_id=1)
    world._ingest_vicon_v2_message(
        malformed_table,
        received_monotonic_s=0.51,
    )

    assert world.active_ball_track_id == 7
    assert world.last_ball_track_id == 7
    assert world.ball_visible_tmp
    assert 7 in world.consumed_ball_track_ids
    assert world.latest_ball_snapshot.track_id == 7
    assert world.latest_ball_snapshot.consumed
    assert world.ball_state_estimator.sample_count == 0
    assert not world.ball_state_estimator_ready_tmp
    assert len(world.test_listener_snapshots) == listener_count
    assert not world.hitter_snapshot_planning_eligible(incumbent_snapshot)
    assert world.vicon_fault_latched is real_world_module.ViconInputFault.VICON_SCHEMA_ERROR

    world._ingest_vicon_v2_message(
        ball(track_id=7, frame=11),
        received_monotonic_s=0.52,
    )
    assert world.active_ball_track_id == 7
    assert world.ball_track_generations[7] == incumbent_generation + 1
    assert world.latest_ball_snapshot.consumed
    assert world.ball_state_estimator.sample_count == 0
    assert len(world.test_listener_snapshots) == listener_count

    world._ingest_vicon_v2_message(
        ball(track_id=8, frame=20),
        received_monotonic_s=0.53,
    )
    assert world.active_ball_track_id == 7
    assert world.ball_state_estimator.sample_count == 0
    assert len(world.test_listener_snapshots) == listener_count


@pytest.mark.parametrize(
    ("message", "expected_active"),
    [
        (ball(track_id=0, frame=1), None),
        (ball(track_id=-1, frame=1), None),
        (pelvis(frame=1, track_id=9), None),
        (table(frame=1, track_id=9), None),
    ],
)
def test_subject_id_contract_fails_closed_without_estimator_sample(
    world,
    message,
    expected_active,
):
    world._ingest_vicon_v2_message(message, received_monotonic_s=1.0)

    assert world.active_ball_track_id is expected_active
    assert world.ball_state_estimator.sample_count == 0
    assert world.vicon_fault_latched is real_world_module.ViconInputFault.VICON_SCHEMA_ERROR
    assert [event.reason for event in world.drain_hitter_vicon_events()] == [
        real_world_module.ViconEventReason.VICON_SCHEMA_ERROR
    ]


def test_configured_base_subject_is_exact_and_mismatch_is_permanent_schema_fault():
    simulator = real_world_module.RealWorld.__new__(real_world_module.RealWorld)
    simulator.cfg = SimpleNamespace(
        motion={
            "ball_planner": {},
            "vicon_consumer": {"base_subject": "RobotPelvis"},
        }
    )
    simulator._init_ball_state()
    matching = pelvis(frame=1)
    matching.name = "RobotPelvis"
    simulator._ingest_vicon_v2_message(
        matching,
        received_monotonic_s=1.0,
    )
    assert simulator.base_pose_valid_tmp

    mismatch = pelvis(frame=2)
    simulator._ingest_vicon_v2_message(
        mismatch,
        received_monotonic_s=1.01,
    )

    assert simulator.vicon_fault_latched is real_world_module.ViconInputFault.VICON_SCHEMA_ERROR
    assert not simulator.begin_hitter_policy_session(now_monotonic_s=1.01)


@pytest.mark.parametrize("wrong_subject", ["g2pelvis", " G2Pelvis", "G2Pelvis "])
def test_base_subject_case_or_whitespace_mismatch_does_not_refresh_stream(
    world,
    wrong_subject,
):
    message = pelvis(frame=1)
    message.name = wrong_subject

    world._ingest_vicon_v2_message(message, received_monotonic_s=1.0)

    assert world._last_any_vicon_monotonic_s is None
    assert not world.base_pose_valid_tmp
    assert world.vicon_fault_latched is real_world_module.ViconInputFault.VICON_SCHEMA_ERROR


def test_base_pose_wait_warning_names_the_configured_subject(monkeypatch):
    simulator = real_world_module.RealWorld.__new__(real_world_module.RealWorld)
    simulator.cfg = SimpleNamespace(
        motion={
            "ball_planner": {},
            "vicon_consumer": {"base_subject": "RobotPelvis"},
        }
    )
    simulator._init_ball_state()
    warnings = []
    monkeypatch.setattr(real_world_module.logger, "warning", warnings.append)

    assert not simulator._policy_start_base_pose_ready(now=1.0)

    assert len(warnings) == 1
    assert "RobotPelvis" in warnings[0]


def test_schema_fault_is_process_lifetime_and_reentry_cannot_clear_it(world):
    world._vicon_state_handler("vicon_state_data_v2", b"bad fingerprint")
    ingest_valid_pelvis(world, now=1.0)

    assert not world.begin_hitter_policy_session(now_monotonic_s=1.0)
    assert world.vicon_fault_latched is real_world_module.ViconInputFault.VICON_SCHEMA_ERROR


def test_stream_stale_is_latched_until_policy_reentry(world):
    ingest_valid_pelvis_and_ball(world, now=1.0, track_id=7)

    status = world.hitter_vicon_status(now_monotonic_s=1.400001)
    assert status.latched_fault is real_world_module.ViconInputFault.VICON_STREAM_STALE
    assert status.active_track_id is None
    assert status.last_track_id == 7
    ingest_valid_pelvis(world, now=1.41)
    assert (
        world.hitter_vicon_status(now_monotonic_s=1.42).latched_fault
        is real_world_module.ViconInputFault.VICON_STREAM_STALE
    )


def test_stream_freshness_boundary_is_strictly_greater_than_point_four(world):
    ingest_valid_pelvis(world, now=1.0)

    assert world.hitter_vicon_status(now_monotonic_s=1.4).stream_fresh
    status = world.hitter_vicon_status(now_monotonic_s=1.400001)
    assert not status.stream_fresh
    assert status.latched_fault is real_world_module.ViconInputFault.VICON_STREAM_STALE


def test_reentry_clears_recovered_stale_fault_and_accepts_next_id(world):
    ingest_valid_pelvis_and_ball(world, now=1.0, track_id=7)
    world.hitter_vicon_status(now_monotonic_s=1.400001)
    ingest_valid_pelvis(world, now=1.41)

    assert world.begin_hitter_policy_session(now_monotonic_s=1.42)
    ingest_valid_pelvis(world, now=1.91, frame=19)
    assert world.hitter_vicon_status(now_monotonic_s=1.92).ready_for_new_serve
    world._ingest_vicon_v2_message(ball(track_id=8, frame=20), received_monotonic_s=1.921)
    assert world.active_ball_track_id == 8
    assert 8 not in world.consumed_ball_track_ids


def test_reentry_rejects_unrecovered_stale_stream(world):
    ingest_valid_pelvis_and_ball(world, now=1.0, track_id=7)
    world.hitter_vicon_status(now_monotonic_s=1.400001)

    assert not world.begin_hitter_policy_session(now_monotonic_s=1.402)
    assert world.vicon_fault_latched is real_world_module.ViconInputFault.VICON_STREAM_STALE


def test_invalid_ball_emits_one_direct_track_end_event_and_preserves_history(world):
    world._ingest_vicon_v2_message(ball(track_id=7, frame=10), received_monotonic_s=1.0)
    world._ingest_vicon_v2_message(
        ball(track_id=7, frame=11, valid=False),
        received_monotonic_s=1.01,
    )
    generation = world.ball_track_generations[7]
    source_frame = world.ball_track_last_frames[7]
    world._ingest_vicon_v2_message(
        ball(track_id=7, frame=12, valid=False),
        received_monotonic_s=1.02,
    )

    status = world.hitter_vicon_status(now_monotonic_s=1.02)
    assert status.active_track_id is None
    assert status.last_track_id == 7
    assert not status.visible
    assert world.ball_track_generations[7] == generation
    assert world.ball_track_last_frames[7] == source_frame
    assert [(event.reason, event.track_id) for event in world.drain_hitter_vicon_events()] == [
        (real_world_module.ViconEventReason.TRACK_ENDED, 7)
    ]
    assert world.drain_hitter_vicon_events() == ()


def test_unknown_invalid_ball_with_no_active_permanently_schema_faults(world):
    world._ingest_vicon_v2_message(
        ball(track_id=99, frame=1, valid=False),
        received_monotonic_s=1.0,
    )

    assert world.vicon_fault_latched is real_world_module.ViconInputFault.VICON_SCHEMA_ERROR
    assert world.seen_ball_track_ids == set()
    assert [event.reason for event in world.drain_hitter_vicon_events()] == [
        real_world_module.ViconEventReason.VICON_SCHEMA_ERROR
    ]


def test_old_non_last_invalid_ball_with_no_active_permanently_schema_faults(world):
    _end_track(world, track_id=7, frame=10, now=1.0)
    _end_track(world, track_id=8, frame=20, now=1.01)
    world.drain_hitter_vicon_events()

    world._ingest_vicon_v2_message(
        ball(track_id=7, frame=12, valid=False),
        received_monotonic_s=1.02,
    )

    assert world.last_ball_track_id == 8
    assert world.vicon_fault_latched is real_world_module.ViconInputFault.VICON_SCHEMA_ERROR
    assert [event.reason for event in world.drain_hitter_vicon_events()] == [
        real_world_module.ViconEventReason.VICON_SCHEMA_ERROR
    ]


def test_invalid_base_transition_emits_once_without_becoming_schema_fault(world):
    ingest_valid_pelvis(world, now=1.0)
    invalid = pelvis(frame=2, valid=False)
    world._ingest_vicon_v2_message(invalid, received_monotonic_s=1.01)
    world._ingest_vicon_v2_message(invalid, received_monotonic_s=1.02)

    assert not world.base_pose_valid_tmp
    assert world.vicon_fault_latched is None
    assert [event.reason for event in world.drain_hitter_vicon_events()] == [
        real_world_module.ViconEventReason.BASE_POSE_INVALID
    ]


def test_per_id_generation_starts_at_one_and_invalid_does_not_advance_it(world):
    world._ingest_vicon_v2_message(ball(track_id=7, frame=10), received_monotonic_s=1.0)
    world._ingest_vicon_v2_message(ball(track_id=7, frame=11, valid=False), received_monotonic_s=1.01)
    world._ingest_vicon_v2_message(ball(track_id=8, frame=20), received_monotonic_s=1.51)

    assert world.ball_track_generations == {7: 1, 8: 1}
    assert world.ball_track_last_frames == {7: 10, 8: 20}


def test_ball_freshness_boundary_ends_active_track_and_emits_once(world):
    open_session(world)
    admit_ball(world, now=0.5)

    ingest_valid_pelvis(world, now=0.9, frame=90)
    assert world.hitter_vicon_status(now_monotonic_s=0.9).ball_fresh
    ingest_valid_pelvis(world, now=0.900001, frame=91)
    status = world.hitter_vicon_status(now_monotonic_s=0.900001)
    assert not status.ball_fresh
    assert status.active_track_id is None
    assert status.latched_fault is real_world_module.ViconInputFault.BALL_MESSAGE_STALE
    assert [event.reason for event in world.drain_hitter_vicon_events()] == [
        real_world_module.ViconEventReason.BALL_MESSAGE_STALE
    ]


def test_startup_visible_id_is_quarantined_and_never_notifies_planner(world):
    world._ingest_vicon_v2_message(ball(track_id=7, frame=10), received_monotonic_s=1.0)

    assert world.active_ball_track_id == 7
    assert world.ball_visible_tmp
    assert world.consumed_ball_track_ids == {7}
    assert world.ball_state_estimator.sample_count == 0
    assert world.latest_ball_snapshot.consumed
    assert world.latest_ball_snapshot.new_track
    assert world.test_listener_snapshots == []


def test_early_new_id_is_consumed_and_does_not_revive_after_gate_time(world):
    open_session(world)
    ingest_valid_pelvis(world, now=0.48, frame=8)
    world._ingest_vicon_v2_message(ball(track_id=7, frame=10), received_monotonic_s=0.49)

    assert world.consumed_ball_track_ids == {7}
    assert world.ball_state_estimator.sample_count == 0
    ingest_valid_pelvis(world, now=0.98, frame=98)
    status = world.hitter_vicon_status(now_monotonic_s=0.99)
    assert not status.ready_for_new_serve
    assert status.active_track_id is None
    assert status.latched_fault is real_world_module.ViconInputFault.BALL_MESSAGE_STALE
    assert world.test_listener_snapshots == []


def test_admitted_listener_snapshots_expose_true_new_and_consumed_flags(world):
    open_session(world)
    admit_ball(world, now=0.5, frame=10)
    world._ingest_vicon_v2_message(ball(track_id=7, frame=11), received_monotonic_s=0.51)

    assert [snapshot.generation for snapshot in world.test_listener_snapshots] == [1, 2]
    assert [snapshot.new_track for snapshot in world.test_listener_snapshots] == [True, False]
    assert [snapshot.consumed for snapshot in world.test_listener_snapshots] == [False, False]

    assert world.consume_hitter_track(7, reason="test_complete")
    world._ingest_vicon_v2_message(ball(track_id=7, frame=12), received_monotonic_s=0.52)
    assert len(world.test_listener_snapshots) == 2
    assert world.latest_ball_snapshot.consumed


@pytest.mark.parametrize("bad_track_id", [True, 7.0, "7", 0, -1])
def test_consume_hitter_track_requires_an_exact_positive_integer_id(
    world,
    bad_track_id,
):
    with pytest.raises(ValueError, match="track_id must be positive"):
        world.consume_hitter_track(bad_track_id, reason="test")


@pytest.mark.parametrize("bad_reason", [None, "", 7])
def test_consume_hitter_track_requires_a_nonempty_string_reason(
    world,
    bad_reason,
):
    with pytest.raises(ValueError, match="reason must be a non-empty string"):
        world.consume_hitter_track(7, reason=bad_reason)


def test_consume_hitter_track_is_idempotent_and_records_latest_reason(world):
    open_session(world)
    admit_ball(world, now=0.5, frame=10)
    world._ingest_vicon_v2_message(ball(track_id=7, frame=11), received_monotonic_s=0.51)
    before = world.latest_ball_snapshot
    assert world.ball_state_estimator.sample_count == 2

    assert world.consume_hitter_track(7, reason="lifecycle_cancel")

    assert world.ball_track_consumption_reasons == {7: "lifecycle_cancel"}
    assert world.latest_ball_snapshot.consumed
    assert world.ball_state_estimator.sample_count == 0
    assert not world.ball_state_estimator_ready_tmp

    assert not world.consume_hitter_track(7, reason="session_end")
    assert world.ball_track_consumption_reasons == {7: "session_end"}
    assert before is not world.latest_ball_snapshot


def test_consume_hitter_track_rejects_an_unseen_valid_id_without_recording_reason(
    world,
):
    assert not world.consume_hitter_track(99, reason="not_seen")
    assert world.ball_track_consumption_reasons == {}


def test_snapshot_eligibility_rejects_an_old_generation_and_delayed_listener(
    world,
):
    open_session(world)
    admit_ball(world, now=0.5, frame=10)
    old_snapshot = world.latest_ball_snapshot
    assert world.hitter_snapshot_planning_eligible(old_snapshot)

    world._ingest_vicon_v2_message(ball(track_id=7, frame=11), received_monotonic_s=0.51)
    current_snapshot = world.latest_ball_snapshot

    assert not world.hitter_snapshot_planning_eligible(old_snapshot)
    assert world.hitter_snapshot_planning_eligible(current_snapshot)

    planned_track_ids = []

    def delayed_listener(snapshot):
        if world.hitter_snapshot_planning_eligible(snapshot):
            planned_track_ids.append(snapshot.track_id)

    assert world.consume_hitter_track(7, reason="lifecycle_cancel")
    delayed_listener(current_snapshot)

    assert planned_track_ids == []
    assert not world.hitter_snapshot_planning_eligible(current_snapshot)


def test_snapshot_eligibility_strictly_validates_the_snapshot_contract(world):
    open_session(world)
    admit_ball(world, now=0.5, frame=10)
    snapshot = world.latest_ball_snapshot

    with pytest.raises(TypeError, match="BallEstimateSnapshot"):
        world.hitter_snapshot_planning_eligible(object())

    malformed_snapshots = [
        replace(snapshot, track_id=True),
        replace(snapshot, generation=True),
        replace(snapshot, generation=-1),
        replace(snapshot, source_frame=True),
        replace(snapshot, source_frame=-1),
        replace(snapshot, source_time_s=float("nan")),
        replace(snapshot, received_monotonic_s=float("inf")),
        replace(snapshot, position_w=np.array([1.0, 2.0])),
        replace(snapshot, velocity_w=np.array([0.0, float("nan"), 0.0])),
        replace(snapshot, base_position_w=np.array([0.0, 0.0])),
        replace(snapshot, base_quaternion_xyzw=np.zeros(4)),
        replace(snapshot, base_valid=1),
        replace(snapshot, visible=1),
        replace(snapshot, ready=1),
        replace(snapshot, consumed=0),
        replace(snapshot, new_track=1),
    ]
    for malformed in malformed_snapshots:
        with pytest.raises(ValueError, match="snapshot"):
            world.hitter_snapshot_planning_eligible(malformed)


def test_end_policy_session_quarantines_active_and_blocks_later_notifications(
    world,
):
    open_session(world)
    admit_ball(world, now=0.5, frame=10)
    before = world.latest_ball_snapshot
    listener_count = len(world.test_listener_snapshots)
    assert world.hitter_snapshot_planning_eligible(before)

    involved = world.end_hitter_policy_session(reason="environment_reset")

    assert involved == (7,)
    assert not world._hitter_policy_session_open
    assert world.active_ball_track_id == 7
    assert world.last_ball_track_id == 7
    assert world.ball_visible_tmp
    assert world.consumed_ball_track_ids == {7}
    assert world.ball_track_consumption_reasons == {7: "environment_reset"}
    assert world.latest_ball_snapshot.consumed
    assert world.ball_state_estimator.sample_count == 0
    assert not world.hitter_snapshot_planning_eligible(before)

    world._ingest_vicon_v2_message(ball(track_id=7, frame=11), received_monotonic_s=0.51)
    world._ingest_vicon_v2_message(ball(track_id=7, frame=12, valid=False), received_monotonic_s=0.52)
    ingest_valid_pelvis(world, now=1.03, frame=103)
    world._ingest_vicon_v2_message(ball(track_id=8, frame=20), received_monotonic_s=1.04)

    assert len(world.test_listener_snapshots) == listener_count
    assert world.latest_ball_snapshot.track_id == 8
    assert world.latest_ball_snapshot.consumed
    assert not world.hitter_snapshot_planning_eligible(world.latest_ball_snapshot)


@pytest.mark.parametrize("bad_reason", [None, "", 7])
def test_end_policy_session_requires_a_nonempty_string_reason(world, bad_reason):
    with pytest.raises(ValueError, match="reason must be a non-empty string"):
        world.end_hitter_policy_session(reason=bad_reason)


def test_end_policy_session_without_an_active_track_returns_empty_tuple(world):
    assert world.end_hitter_policy_session(reason="environment_reset") == ()
    assert not world._hitter_policy_session_open


def test_successful_begin_discards_recoverable_events_without_resetting_sequence(
    world,
):
    open_session(world)
    recoverable_reasons = (
        real_world_module.ViconEventReason.TRACK_ENDED,
        real_world_module.ViconEventReason.BASE_POSE_INVALID,
        real_world_module.ViconEventReason.VICON_STREAM_STALE,
        real_world_module.ViconEventReason.BALL_MESSAGE_STALE,
        real_world_module.ViconEventReason.TRACK_ID_CONFLICT,
    )
    with world._hitter_ball_state_lock:
        for reason in recoverable_reasons:
            world._enqueue_vicon_event_locked(
                reason,
                track_id=(7 if reason is not real_world_module.ViconEventReason.BASE_POSE_INVALID else None),
                detail=f"old session {reason.value}",
            )
        world.vicon_fault_latched = real_world_module.ViconInputFault.TRACK_ID_CONFLICT
        world._vicon_fault_event_emitted.update(
            {
                real_world_module.ViconInputFault.VICON_STREAM_STALE,
                real_world_module.ViconInputFault.BALL_MESSAGE_STALE,
                real_world_module.ViconInputFault.TRACK_ID_CONFLICT,
            }
        )

    assert world.begin_hitter_policy_session(now_monotonic_s=0.01)
    assert world.drain_hitter_vicon_events() == ()
    assert world.vicon_fault_latched is None

    with world._hitter_ball_state_lock:
        world._enqueue_vicon_event_locked(
            real_world_module.ViconEventReason.TRACK_ENDED,
            track_id=8,
            detail="new session track ended",
        )
    assert [event.sequence for event in world.drain_hitter_vicon_events()] == [6]


def test_failed_begin_preserves_permanent_schema_event(world):
    malformed = ball(track_id=7, frame=10)
    malformed.track_id = 0
    world._ingest_vicon_v2_message(
        malformed,
        received_monotonic_s=0.0,
    )
    ingest_valid_pelvis(world, now=0.01, frame=2)
    before = tuple(world._vicon_events)

    assert not world.begin_hitter_policy_session(now_monotonic_s=0.01)
    assert tuple(world._vicon_events) == before
    assert [event.reason for event in world.drain_hitter_vicon_events()] == [
        real_world_module.ViconEventReason.VICON_SCHEMA_ERROR
    ]


def test_failed_begin_preserves_permanent_overflow_event():
    simulator = real_world_module.RealWorld.__new__(real_world_module.RealWorld)
    simulator.cfg = SimpleNamespace(
        motion={
            "ball_planner": {},
            "vicon_consumer": {
                "base_subject": "G2Pelvis",
                "event_queue_capacity": 1,
            },
        }
    )
    simulator._init_ball_state()
    ingest_valid_pelvis(simulator, now=0.0, frame=1)
    with simulator._hitter_ball_state_lock:
        simulator._enqueue_vicon_event_locked(
            real_world_module.ViconEventReason.TRACK_ENDED,
            track_id=7,
            detail="first event",
        )
        simulator._enqueue_vicon_event_locked(
            real_world_module.ViconEventReason.BASE_POSE_INVALID,
            track_id=None,
            detail="overflow trigger",
        )
    before = tuple(simulator._vicon_events)

    assert not simulator.begin_hitter_policy_session(now_monotonic_s=0.01)
    assert tuple(simulator._vicon_events) == before
    assert before[0].reason is real_world_module.ViconEventReason.VICON_SCHEMA_ERROR
    assert "overflow" in before[0].detail.lower()


def test_new_track_estimator_window_never_contains_previous_track_samples(world):
    open_session(world)
    admit_ball(world, track_id=7, now=0.5, frame=10)
    world._ingest_vicon_v2_message(
        ball(track_id=7, frame=11, position=(1.4, 0.0, 0.95)),
        received_monotonic_s=0.51,
    )
    world._ingest_vicon_v2_message(
        ball(track_id=7, frame=12, position=(1.3, 0.0, 0.9)),
        received_monotonic_s=0.52,
    )
    assert world.ball_state_estimator.sample_count == 3
    assert world.ball_state_estimator_ready_tmp
    world._ingest_vicon_v2_message(
        ball(track_id=7, frame=13, valid=False),
        received_monotonic_s=0.53,
    )
    ingest_valid_pelvis(world, now=1.02, frame=102)
    world._ingest_vicon_v2_message(
        ball(track_id=8, frame=20, position=(2.0, 0.2, 1.2)),
        received_monotonic_s=1.03,
    )

    assert world.active_ball_track_id == 8
    assert world.ball_snapshot_generation == 1
    assert world.ball_state_estimator.sample_count == 1
    assert not world.ball_state_estimator_ready_tmp
    np.testing.assert_allclose(
        world.ball_state_estimator._samples[0][1],
        [2.0, 0.2, 1.2],
    )


def test_quarantined_new_track_cuts_old_window_without_adding_own_sample(world):
    open_session(world)
    admit_ball(world, track_id=7, now=0.5, frame=10)
    world._ingest_vicon_v2_message(ball(track_id=7, frame=11), received_monotonic_s=0.51)
    world._ingest_vicon_v2_message(ball(track_id=7, frame=12), received_monotonic_s=0.52)
    world._ingest_vicon_v2_message(ball(track_id=7, frame=13, valid=False), received_monotonic_s=0.53)
    ingest_valid_pelvis(world, now=0.79, frame=79)
    world._ingest_vicon_v2_message(ball(track_id=8, frame=20), received_monotonic_s=0.80)

    assert world.consumed_ball_track_ids == {8}
    assert world.active_ball_track_id == 8
    assert world.ball_state_estimator.sample_count == 0
    assert not world.ball_state_estimator_ready_tmp


def test_seen_and_consumed_sets_survive_estimator_and_policy_reset(world):
    world._ingest_vicon_v2_message(ball(track_id=7, frame=10), received_monotonic_s=1.0)
    world.reset_ball_state_estimator()
    ingest_valid_pelvis(world, now=1.01)
    assert world.begin_hitter_policy_session(now_monotonic_s=1.01)

    assert world.seen_ball_track_ids == {7}
    assert world.consumed_ball_track_ids == {7}


def test_ready_gate_requires_exact_half_second_and_a_fresh_base(world):
    open_session(world)
    ingest_valid_pelvis(world, now=0.49, frame=49)

    assert not world.hitter_vicon_status(now_monotonic_s=0.49).ready_for_new_serve
    ingest_valid_pelvis(world, now=0.50, frame=50)
    assert world.hitter_vicon_status(now_monotonic_s=0.50).ready_for_new_serve


def test_every_valid_ball_clears_no_ball_timer_even_when_quarantined(world):
    open_session(world)
    ingest_valid_pelvis(world, now=0.20, frame=20)
    world._ingest_vicon_v2_message(ball(track_id=7, frame=10), received_monotonic_s=0.20)

    assert world._no_ball_since_monotonic_s is None
    assert 7 in world.consumed_ball_track_ids


@pytest.mark.parametrize("repeated_frame", [10, 9])
def test_duplicate_or_backward_valid_ball_clears_no_ball_timer_before_early_return(
    world,
    repeated_frame,
):
    world._ingest_vicon_v2_message(ball(track_id=7, frame=10), received_monotonic_s=1.0)
    world._ingest_vicon_v2_message(
        ball(track_id=7, frame=11, valid=False),
        received_monotonic_s=1.01,
    )
    generation = world.ball_snapshot_generation
    samples = world.ball_state_estimator.sample_count
    listener_count = len(world.test_listener_snapshots)
    assert world._no_ball_since_monotonic_s == 1.01

    world._ingest_vicon_v2_message(
        ball(track_id=7, frame=repeated_frame),
        received_monotonic_s=1.02,
    )

    assert world._no_ball_since_monotonic_s is None
    assert world.active_ball_track_id is None
    assert not world.ball_visible_tmp
    assert world.ball_snapshot_generation == generation
    assert world.ball_state_estimator.sample_count == samples
    assert len(world.test_listener_snapshots) == listener_count


def test_reentry_quarantines_current_visible_id(world):
    ingest_valid_pelvis_and_ball(world, now=1.0, track_id=7)

    assert world.begin_hitter_policy_session(now_monotonic_s=1.01)
    assert world.active_ball_track_id == 7
    assert 7 in world.consumed_ball_track_ids
    assert world.ball_state_estimator.sample_count == 0
    assert not world.hitter_vicon_status(now_monotonic_s=1.01).ready_for_new_serve


def test_reentry_quarantine_rebuilds_latest_snapshot_as_consumed(world):
    open_session(world)
    admit_ball(world, now=0.5, frame=10)
    before = world.latest_ball_snapshot
    listener_count = len(world.test_listener_snapshots)
    generation = world.ball_snapshot_generation
    assert not before.consumed

    assert world.begin_hitter_policy_session(now_monotonic_s=0.51)

    after = world.latest_ball_snapshot
    assert after.consumed
    assert after.track_id == before.track_id
    assert after.generation == before.generation
    assert after.source_frame == before.source_frame
    assert after.new_track == before.new_track
    assert world.ball_snapshot_generation == generation
    assert len(world.test_listener_snapshots) == listener_count


def test_event_queue_preserves_sequence_and_drains_atomically(world):
    world._ingest_vicon_v2_message(ball(track_id=7, frame=10), received_monotonic_s=1.0)
    world._ingest_vicon_v2_message(ball(track_id=7, frame=11, valid=False), received_monotonic_s=1.01)
    world._ingest_vicon_v2_message(ball(track_id=8, frame=20), received_monotonic_s=1.02)
    world._ingest_vicon_v2_message(ball(track_id=8, frame=21, valid=False), received_monotonic_s=1.03)

    events = world.drain_hitter_vicon_events()
    assert [event.sequence for event in events] == [1, 2]
    assert [(event.reason, event.track_id) for event in events] == [
        (real_world_module.ViconEventReason.TRACK_ENDED, 7),
        (real_world_module.ViconEventReason.TRACK_ENDED, 8),
    ]
    assert world.drain_hitter_vicon_events() == ()


def _end_track(world, *, track_id: int, frame: int, now: float) -> None:
    world._ingest_vicon_v2_message(
        ball(track_id=track_id, frame=frame),
        received_monotonic_s=now,
    )
    world._ingest_vicon_v2_message(
        ball(track_id=track_id, frame=frame + 1, valid=False),
        received_monotonic_s=now + 0.001,
    )


def test_event_queue_allows_exact_capacity_without_dropping_events():
    simulator = real_world_module.RealWorld.__new__(real_world_module.RealWorld)
    simulator.cfg = SimpleNamespace(
        motion={
            "ball_planner": {},
            "vicon_consumer": {
                "base_subject": "G2Pelvis",
                "event_queue_capacity": 2,
            },
        }
    )
    simulator._init_ball_state()

    _end_track(simulator, track_id=7, frame=10, now=1.0)
    _end_track(simulator, track_id=8, frame=20, now=1.01)

    events = simulator.drain_hitter_vicon_events()
    assert [event.sequence for event in events] == [1, 2]
    assert [event.reason for event in events] == [
        real_world_module.ViconEventReason.TRACK_ENDED,
        real_world_module.ViconEventReason.TRACK_ENDED,
    ]
    assert simulator.vicon_fault_latched is None


def test_event_queue_overflow_atomically_replaces_events_and_latches_schema_fault():
    simulator = real_world_module.RealWorld.__new__(real_world_module.RealWorld)
    simulator.cfg = SimpleNamespace(
        motion={
            "ball_planner": {},
            "vicon_consumer": {
                "base_subject": "G2Pelvis",
                "event_queue_capacity": 2,
            },
        }
    )
    simulator._init_ball_state()
    snapshots = []
    unregister = simulator.register_hitter_ball_listener(snapshots.append)

    open_session(simulator)
    admit_ball(simulator, track_id=7, now=0.5, frame=10)
    listener_count = len(snapshots)
    assert 7 in simulator.admitted_ball_track_ids
    assert 7 not in simulator.consumed_ball_track_ids

    simulator._ingest_vicon_v2_message(pelvis(frame=11, valid=False), received_monotonic_s=0.51)
    ingest_valid_pelvis(simulator, now=0.52, frame=12)
    simulator._ingest_vicon_v2_message(pelvis(frame=13, valid=False), received_monotonic_s=0.53)
    ingest_valid_pelvis(simulator, now=0.54, frame=14)
    simulator._ingest_vicon_v2_message(pelvis(frame=15, valid=False), received_monotonic_s=0.55)
    overflow_sequence = simulator._vicon_events[0].sequence
    simulator._ingest_vicon_v2_message(pelvis(frame=16, valid=False), received_monotonic_s=0.56)
    assert len(simulator._vicon_events) == 1
    assert simulator._vicon_events[0].sequence == overflow_sequence
    assert simulator.active_ball_track_id == 7
    assert simulator.last_ball_track_id == 7
    assert simulator.ball_visible_tmp
    assert 7 in simulator.consumed_ball_track_ids
    assert simulator.latest_ball_snapshot.track_id == 7
    assert simulator.latest_ball_snapshot.consumed
    assert simulator.ball_state_estimator.sample_count == 0
    assert not simulator.ball_state_estimator_ready_tmp
    assert len(snapshots) == listener_count

    events = simulator.drain_hitter_vicon_events()
    assert len(events) == 1
    assert events[0].sequence == 3
    assert events[0].reason is real_world_module.ViconEventReason.VICON_SCHEMA_ERROR
    assert "overflow" in events[0].detail.lower()
    assert simulator.vicon_fault_latched is real_world_module.ViconInputFault.VICON_SCHEMA_ERROR
    ingest_valid_pelvis(simulator, now=0.57, frame=17)
    assert not simulator.begin_hitter_policy_session(now_monotonic_s=0.57)
    assert simulator.drain_hitter_vicon_events() == ()
    simulator._ingest_vicon_v2_message(ball(track_id=7, frame=11), received_monotonic_s=0.58)
    simulator._ingest_vicon_v2_message(ball(track_id=8, frame=20), received_monotonic_s=0.59)
    assert simulator.active_ball_track_id == 7
    assert simulator.ball_state_estimator.sample_count == 0
    assert len(snapshots) == listener_count
    assert simulator.drain_hitter_vicon_events() == ()
    unregister()


def test_communication_subscribes_only_to_validated_v2_channel(world):
    class FakeLcm:
        def __init__(self):
            self.channels = []
            self.unsubscribed = []

        def subscribe(self, channel, callback):
            token = (channel, callback)
            self.channels.append(channel)
            return token

        def unsubscribe(self, subscription):
            self.unsubscribed.append(subscription)

    world.lc = FakeLcm()
    world._init_communication()
    try:
        assert "vicon_state_data_v2" in world.lc.channels
        assert "vicon_state_data" not in world.lc.channels
    finally:
        assert world.close()


@pytest.mark.parametrize(
    ("config", "match"),
    [
        ({"channel": "vicon_state_data"}, "exactly vicon_state_data_v2"),
        ({"stream_timeout_s": float("nan")}, "finite and positive"),
        ({"ball_timeout_s": float("inf")}, "finite and positive"),
        ({"new_serve_no_ball_s": 0.0}, "finite and positive"),
    ],
)
def test_vicon_consumer_settings_reject_unsafe_values(config, match):
    with pytest.raises(ValueError, match=match):
        hitter_runtime_factory.resolve_vicon_consumer_settings(config)


@pytest.mark.parametrize("config", [None, [], "vicon settings"])
def test_vicon_consumer_settings_reject_non_mapping_config(config):
    with pytest.raises(TypeError, match="mapping"):
        hitter_runtime_factory.resolve_vicon_consumer_settings(config)


@pytest.mark.parametrize(
    ("name", "value"),
    [
        ("stream_timeout_s", True),
        ("stream_timeout_s", "0.40"),
        ("ball_timeout_s", True),
        ("ball_timeout_s", "0.40"),
        ("new_serve_no_ball_s", True),
        ("new_serve_no_ball_s", "0.50"),
    ],
)
def test_vicon_consumer_settings_reject_coercible_timeout_values(
    name,
    value,
):
    with pytest.raises(TypeError, match="real number"):
        hitter_runtime_factory.resolve_vicon_consumer_settings({name: value})


def test_vicon_consumer_settings_defaults_are_exact():
    settings = hitter_runtime_factory.resolve_vicon_consumer_settings({})

    assert settings.channel == "vicon_state_data_v2"
    assert settings.base_subject == "G2Pelvis"
    assert settings.stream_timeout_s == 0.40
    assert settings.ball_timeout_s == 0.40
    assert settings.new_serve_no_ball_s == 0.50
    assert settings.event_queue_capacity == 64


@pytest.mark.parametrize(
    ("config", "match"),
    [
        ({"base_subject": None}, "base_subject"),
        ({"base_subject": ""}, "base_subject"),
        ({"base_subject": " G2Pelvis"}, "base_subject"),
        ({"base_subject": "BALL"}, "reserved"),
        ({"base_subject": "table"}, "reserved"),
        ({"event_queue_capacity": True}, "positive integer"),
        ({"event_queue_capacity": 0}, "positive integer"),
        ({"event_queue_capacity": 1.5}, "positive integer"),
        ({"event_queue_capacity": 65}, "at most 64"),
    ],
)
def test_vicon_consumer_settings_reject_invalid_identity_or_capacity(config, match):
    with pytest.raises(ValueError, match=match):
        hitter_runtime_factory.resolve_vicon_consumer_settings(config)


@pytest.mark.parametrize("vicon_config", [None, [], "vicon settings"])
def test_real_world_rejects_explicit_invalid_vicon_consumer_config(
    vicon_config,
):
    world = real_world_module.RealWorld.__new__(real_world_module.RealWorld)
    world.cfg = SimpleNamespace(
        motion={"vicon_consumer": vicon_config},
    )

    with pytest.raises(TypeError, match="mapping"):
        world._init_ball_state()


def test_real_world_uses_vicon_defaults_only_when_field_is_missing():
    world = real_world_module.RealWorld.__new__(real_world_module.RealWorld)
    world.cfg = SimpleNamespace(motion={})

    world._init_ball_state()

    assert world.vicon_consumer_settings == (hitter_runtime_factory.resolve_vicon_consumer_settings({}))


def _malformed_wire_messages():
    negative_frame = ball(track_id=7, frame=1)
    negative_frame.vicon_frame_number = -1
    bool_frame = pelvis(frame=1)
    bool_frame.vicon_frame_number = True
    non_integer_track = table(frame=1)
    non_integer_track.track_id = 0.0
    abnormal_flags = ball(track_id=7, frame=1)
    abnormal_flags.valid = 1
    abnormal_flags.occluded = 1
    bool_flags = pelvis(frame=1)
    bool_flags.valid = True
    bool_flags.occluded = False
    short_position = table(frame=1)
    short_position.pos_vicon = [0.0, 0.0]
    nan_position = ball(track_id=7, frame=1)
    nan_position.pos_vicon = [float("nan"), 0.0, 1.0]
    infinite_quaternion = pelvis(frame=1)
    infinite_quaternion.quat_vicon = [0.0, 0.0, 0.0, float("inf")]
    zero_quaternion = table(frame=1)
    zero_quaternion.quat_vicon = [0.0, 0.0, 0.0, 0.0]
    invalid_table = table(frame=1)
    invalid_table.valid = 0
    invalid_table.occluded = 1
    return [
        negative_frame,
        bool_frame,
        non_integer_track,
        abnormal_flags,
        bool_flags,
        short_position,
        nan_position,
        infinite_quaternion,
        zero_quaternion,
        invalid_table,
    ]


@pytest.mark.parametrize("message", _malformed_wire_messages())
def test_full_wire_contract_violations_fail_closed_before_state_mutation(
    world,
    message,
):
    last_any = world._last_any_vicon_monotonic_s

    world._ingest_vicon_v2_message(message, received_monotonic_s=1.0)

    assert world._last_any_vicon_monotonic_s is last_any
    assert world.active_ball_track_id is None
    assert world.ball_state_estimator.sample_count == 0
    assert world.test_listener_snapshots == []
    assert world.vicon_fault_latched is real_world_module.ViconInputFault.VICON_SCHEMA_ERROR
