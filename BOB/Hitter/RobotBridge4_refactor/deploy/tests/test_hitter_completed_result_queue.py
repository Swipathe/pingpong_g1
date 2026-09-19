from __future__ import annotations

import inspect
import threading
import time
from dataclasses import replace

import numpy as np
import pytest

import utils.hitter_realtime as hitter_realtime
from diagnostics.hitter_task_pipeline import ShadowTaskPipeline
from utils.hitter_planner import HitterWbcCommand, StrikePlan
from utils.hitter_realtime import (
    BallEstimateSnapshot,
    LatestOnlyPlannerWorker,
    PlannerResultSnapshot,
)
from utils.hitter_runtime_types import PlannerFailureReason, PlannerRejected


def snapshot(*, track_id: int = 7, generation: int = 1) -> BallEstimateSnapshot:
    received_s = time.monotonic()
    return BallEstimateSnapshot(
        track_id=track_id,
        generation=generation,
        source_frame=generation,
        source_time_s=float(generation) / 360.0,
        received_monotonic_s=received_s,
        position_w=np.array([0.8, 0.0, 1.0], dtype=np.float64),
        velocity_w=np.array([-2.0, 0.0, 0.0], dtype=np.float64),
        base_position_w=np.array([-0.4, 0.0, 0.8], dtype=np.float64),
        base_quaternion_xyzw=np.array(
            [0.0, 0.0, 0.0, 1.0],
            dtype=np.float64,
        ),
        base_valid=True,
        visible=True,
        ready=True,
    )


def plan_ok(value: BallEstimateSnapshot) -> HitterWbcCommand:
    time_to_strike = 1.0
    strike_plan = StrikePlan(
        t_strike=time_to_strike,
        p_racket_target=np.array([0.0, -0.1, 1.0], dtype=np.float64),
        v_racket_target=np.array([1.0, 0.0, 0.5], dtype=np.float64),
        v_ball_in=np.array([-2.0, 0.0, 0.0], dtype=np.float64),
        v_ball_out=np.array([3.0, 0.0, 0.5], dtype=np.float64),
    )
    return HitterWbcCommand(
        strike_type="forehand",
        p_base_target_xy=np.array([-0.4, 0.2], dtype=np.float64),
        v_racket_target_w=strike_plan.v_racket_target.copy(),
        time_to_strike=time_to_strike,
        strike_plan=strike_plan,
        strike_table_y_w=-0.1,
        strike_side_source="table_y",
    )


def plan_in_generation_order(value: BallEstimateSnapshot) -> HitterWbcCommand:
    return plan_ok(value)


def wait_until(predicate, timeout_s: float = 1.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.001)
    raise AssertionError("condition was not satisfied before timeout")


def complete_sequential_submissions(
    worker: LatestOnlyPlannerWorker,
    generations=(1, 2, 3),
) -> None:
    for completion_count, generation in enumerate(generations, start=1):
        worker.submit(snapshot(generation=generation))
        wait_until(lambda: worker.stats.completed + worker.stats.failed == completion_count)


def completed_failure_pair(plan_fn):
    worker = LatestOnlyPlannerWorker(
        plan_fn,
        trace_listener=lambda _trace: None,
    )
    try:
        worker.submit(snapshot())
        wait_until(lambda: worker.stats.failed == 1)
        batch = worker.drain_completed_results()
        assert len(batch.results) == 1
        assert len(batch.frozen_results) == 1
        assert batch.frozen_results[0] is not None
        return batch.results[0], batch.frozen_results[0]
    finally:
        assert worker.close(timeout_s=1.0)


def test_completed_results_are_drained_in_order():
    worker = LatestOnlyPlannerWorker(plan_in_generation_order)
    try:
        complete_sequential_submissions(worker)
        batch = worker.drain_completed_results()
        assert [result.source_generation for result in batch.results] == [1, 2, 3]
        assert batch.frozen_results == (None, None, None)
        assert worker.drain_completed_results().results == ()
    finally:
        assert worker.close(timeout_s=1.0)


def test_capacity_overflow_is_sticky_and_never_replaces_existing_results():
    worker = LatestOnlyPlannerWorker(
        plan_ok,
        completed_result_queue_capacity=2,
    )
    try:
        complete_sequential_submissions(worker)
        batch = worker.drain_completed_results()
        assert [result.source_generation for result in batch.results] == [1, 2]
        assert batch.overflowed is True
        assert batch.overflow_count == 1
        assert batch.overflowed_track_ids == (7,)

        empty = worker.drain_completed_results()
        assert empty.results == ()
        assert empty.frozen_results == ()
        assert empty.overflowed is False
        assert empty.overflow_count == 0
        assert empty.overflowed_track_ids == ()
        assert worker.stats.completed_result_queue_overflow_total == 1
    finally:
        assert worker.close(timeout_s=1.0)


def test_overflow_count_can_exceed_sorted_unique_track_ids():
    worker = LatestOnlyPlannerWorker(
        plan_ok,
        completed_result_queue_capacity=1,
    )
    try:
        complete_sequential_submissions(worker, generations=(1, 2, 3))
        worker.submit(snapshot(track_id=9, generation=4))
        wait_until(lambda: worker.stats.completed == 4)

        batch = worker.drain_completed_results()
        assert [result.source_generation for result in batch.results] == [1]
        assert batch.overflow_count == 3
        assert batch.overflowed_track_ids == (7, 9)
        assert worker.stats.completed_result_queue_overflow_total == 3
    finally:
        assert worker.close(timeout_s=1.0)


@pytest.mark.parametrize("capacity", [True, False, 0, -1, 1.0, 64.0])
def test_completed_queue_capacity_requires_a_positive_plain_integer(capacity):
    with pytest.raises(ValueError, match="positive integer"):
        LatestOnlyPlannerWorker(
            plan_ok,
            completed_result_queue_capacity=capacity,
        )


def test_completed_queue_capacity_defaults_to_64():
    worker = LatestOnlyPlannerWorker(plan_ok)
    try:
        complete_sequential_submissions(worker, generations=range(1, 66))
        batch = worker.drain_completed_results()
        assert len(batch.results) == 64
        assert batch.overflow_count == 1
    finally:
        assert worker.close(timeout_s=1.0)


def test_pending_input_remains_latest_only_while_planner_is_blocked():
    first_entered = threading.Event()
    release_first = threading.Event()
    planned_generations: list[int] = []

    def plan(value: BallEstimateSnapshot) -> HitterWbcCommand:
        planned_generations.append(value.generation)
        if value.generation == 1:
            first_entered.set()
            if not release_first.wait(timeout=1.0):
                raise AssertionError("test did not release first plan")
        return plan_ok(value)

    worker = LatestOnlyPlannerWorker(plan)
    try:
        worker.submit(snapshot(generation=1))
        assert first_entered.wait(timeout=1.0)
        worker.submit(snapshot(generation=2))
        worker.submit(snapshot(generation=3))
        release_first.set()
        wait_until(lambda: worker.stats.completed == 2)

        assert planned_generations == [1, 3]
        assert worker.stats.pending_replaced_total == 1
        assert worker.max_pending_depth == 1
        assert [result.source_generation for result in worker.drain_completed_results().results] == [1, 3]
    finally:
        release_first.set()
        assert worker.close(timeout_s=1.0)


def test_typed_rejection_preserves_reason_detail_and_frozen_identity():
    detail = "ball estimate has no future crossing"

    def reject(_value: BallEstimateSnapshot):
        raise PlannerRejected(
            PlannerFailureReason.NO_FUTURE_CROSSING,
            detail,
        )

    worker = LatestOnlyPlannerWorker(reject, trace_listener=lambda _trace: None)
    try:
        worker.submit(snapshot())
        wait_until(lambda: worker.stats.failed == 1)
        batch = worker.drain_completed_results()
        result = batch.results[0]
        frozen = batch.frozen_results[0]

        assert result.command is None
        assert result.failure_reason is PlannerFailureReason.NO_FUTURE_CROSSING
        assert result.error_text == detail
        assert frozen is not None
        assert frozen.snapshot_key.track_id == 7
        assert frozen.failure_reason is PlannerFailureReason.NO_FUTURE_CROSSING
        assert frozen.error_text == detail
    finally:
        assert worker.close(timeout_s=1.0)


def test_unknown_exception_is_internal_error_and_worker_continues():
    def plan(value: BallEstimateSnapshot) -> HitterWbcCommand:
        if value.generation == 1:
            raise LookupError("missing intercept")
        return plan_ok(value)

    worker = LatestOnlyPlannerWorker(plan, trace_listener=lambda _trace: None)
    try:
        complete_sequential_submissions(worker, generations=(1, 2))
        batch = worker.drain_completed_results()
        failed, succeeded = batch.results

        assert failed.command is None
        assert failed.failure_reason is PlannerFailureReason.INTERNAL_ERROR
        assert failed.error_text == "LookupError: missing intercept"
        assert batch.frozen_results[0].failure_reason is PlannerFailureReason.INTERNAL_ERROR
        assert succeeded.command is not None
        assert succeeded.failure_reason is None
        assert succeeded.error_text is None
        assert worker.stats.failed == 1
        assert worker.stats.completed == 1
    finally:
        assert worker.close(timeout_s=1.0)


def test_real_typed_rejection_pair_rejects_reason_type_or_detail_mutation():
    detail = "轨迹: 没有未来交点"

    def reject(_value: BallEstimateSnapshot):
        raise PlannerRejected(
            PlannerFailureReason.NO_FUTURE_CROSSING,
            detail,
        )

    result, frozen = completed_failure_pair(reject)

    assert result.failure_reason is PlannerFailureReason.NO_FUTURE_CROSSING
    assert result.error_text == "轨迹: 没有未来交点"
    assert frozen.failure_reason is PlannerFailureReason.NO_FUTURE_CROSSING
    assert frozen.error_type == "PlannerRejected"
    assert frozen.error_text == "轨迹: 没有未来交点"
    assert ShadowTaskPipeline._result_bundle_matches(result, frozen)

    mutations = (
        {"failure_reason": PlannerFailureReason.TRACK_ENDED},
        {"error_type": "LookupError"},
        {"error_text": "不同: 详情"},
    )
    for changes in mutations:
        assert not ShadowTaskPipeline._result_bundle_matches(
            result,
            replace(frozen, **changes),
        )


@pytest.mark.parametrize(
    ("message", "expected_result_text", "expected_frozen_text"),
    [
        ("", "ValueError: ", ""),
        ("a:b:c", "ValueError: a:b:c", "a:b:c"),
        ("错误:没有交点", "ValueError: 错误:没有交点", "错误:没有交点"),
        ("x" * 2055, None, "x" * 2048),
    ],
    ids=("empty", "colons", "non_ascii", "over_2048"),
)
def test_real_unknown_failure_pair_rejects_reason_type_or_detail_mutation(
    message,
    expected_result_text,
    expected_frozen_text,
):
    def fail(_value: BallEstimateSnapshot):
        raise ValueError(message)

    result, frozen = completed_failure_pair(fail)

    assert result.failure_reason is PlannerFailureReason.INTERNAL_ERROR
    if expected_result_text is None:
        assert result.error_text.startswith("ValueError: ")
        assert len(result.error_text) == 2067
        assert result.error_text.endswith("x" * 2055)
    else:
        assert result.error_text == expected_result_text
    assert frozen.failure_reason is PlannerFailureReason.INTERNAL_ERROR
    assert frozen.error_type == "ValueError"
    assert frozen.error_text == expected_frozen_text
    assert ShadowTaskPipeline._result_bundle_matches(result, frozen)

    mutations = (
        {"failure_reason": PlannerFailureReason.TRACK_ENDED},
        {"error_type": "LookupError"},
        {"error_text": "different"},
    )
    for changes in mutations:
        assert not ShadowTaskPipeline._result_bundle_matches(
            result,
            replace(frozen, **changes),
        )


def test_frozen_queue_result_is_detached_from_mutable_command_arrays():
    command_holder = {}

    def plan(value: BallEstimateSnapshot) -> HitterWbcCommand:
        command = plan_ok(value)
        command_holder["command"] = command
        return command

    worker = LatestOnlyPlannerWorker(plan, trace_listener=lambda _trace: None)
    try:
        worker.submit(snapshot())
        wait_until(lambda: worker.stats.completed == 1)
        batch = worker.drain_completed_results()
        frozen = batch.frozen_results[0]

        command_holder["command"].p_base_target_xy[0] = 99.0
        assert frozen.command_fields["p_base_target_xy"] == (-0.4, 0.2)
        with pytest.raises(TypeError):
            frozen.command_fields["new"] = "value"
    finally:
        assert worker.close(timeout_s=1.0)


def test_trace_listener_can_close_worker_without_delivery_lock_deadlock():
    start_delivery_attempted = threading.Event()
    close_results: list[bool] = []
    delivered_sequences: list[int] = []

    class CoordinatedWorker(LatestOnlyPlannerWorker):
        def _notify_traces(self, traces):
            if threading.current_thread() is self._thread and any(trace.kind == "start" for trace in traces):
                start_delivery_attempted.set()
            return super()._notify_traces(traces)

    worker = None

    def listener(trace):
        delivered_sequences.append(trace.trace_seq)
        if trace.kind == "submit":
            assert start_delivery_attempted.wait(timeout=1.0)
            close_results.append(worker.close(timeout_s=0.2))

    worker = CoordinatedWorker(plan_ok, trace_listener=listener)
    try:
        worker.submit(snapshot())
        assert close_results == [True]
        assert delivered_sequences == list(range(1, len(delivered_sequences) + 1))
        assert worker.close(timeout_s=0.2)
    finally:
        assert worker.close(timeout_s=1.0)


def test_trace_listener_reentrant_submit_preserves_exact_sequence():
    delivered_sequences: list[int] = []
    submitted_generations: list[int] = []
    worker = None

    def listener(trace):
        delivered_sequences.append(trace.trace_seq)
        if trace.kind == "submit":
            submitted_generations.append(trace.snapshot.generation)
            if trace.snapshot.generation == 1:
                worker.submit(snapshot(generation=2))

    worker = LatestOnlyPlannerWorker(plan_ok, trace_listener=listener)
    try:
        worker.submit(snapshot(generation=1))
        wait_until(lambda: worker.stats.completed + worker.stats.failed >= 1)
        assert worker.close(timeout_s=1.0)
        assert submitted_generations == [1, 2]
        assert delivered_sequences == list(range(1, len(delivered_sequences) + 1))
    finally:
        assert worker.close(timeout_s=1.0)


def test_planner_result_error_is_derived_from_typed_reason():
    common = {
        "track_id": 7,
        "source_generation": 1,
        "source_frame": 1,
        "strike_deadline_monotonic_s": float("nan"),
        "completed_monotonic_s": time.monotonic(),
        "command": None,
        "error_text": "diagnostic detail",
    }

    track_ended = PlannerResultSnapshot(
        **common,
        failure_reason=PlannerFailureReason.TRACK_ENDED,
    )
    internal = PlannerResultSnapshot(
        **common,
        failure_reason=PlannerFailureReason.INTERNAL_ERROR,
    )
    success = PlannerResultSnapshot(
        **{
            **common,
            "strike_deadline_monotonic_s": time.monotonic() + 1.0,
            "command": plan_ok(snapshot()),
            "error_text": None,
        }
    )

    assert track_ended.error == PlannerFailureReason.TRACK_ENDED.value
    assert internal.error == PlannerFailureReason.INTERNAL_ERROR.value
    assert success.error is None


def test_planner_result_constructor_has_no_legacy_error_input():
    assert "error" not in inspect.signature(PlannerResultSnapshot).parameters
    with pytest.raises(TypeError, match="unexpected keyword argument 'error'"):
        PlannerResultSnapshot(
            track_id=7,
            source_generation=1,
            source_frame=1,
            strike_deadline_monotonic_s=float("nan"),
            completed_monotonic_s=time.monotonic(),
            command=None,
            failure_reason=PlannerFailureReason.INTERNAL_ERROR,
            error_text="detail",
            error="TRACK_ENDED",
        )


def test_planner_result_enforces_exact_success_or_failure():
    common = {
        "track_id": 7,
        "source_generation": 1,
        "source_frame": 1,
        "strike_deadline_monotonic_s": time.monotonic() + 1.0,
        "completed_monotonic_s": time.monotonic(),
    }
    command = plan_ok(snapshot())

    success = PlannerResultSnapshot(**common, command=command)
    failure = PlannerResultSnapshot(
        **common,
        command=None,
        failure_reason=PlannerFailureReason.TRACK_ENDED,
        error_text="track ended",
    )
    assert success.failure_reason is None
    assert failure.command is None

    with pytest.raises(ValueError, match="exactly success or failure"):
        PlannerResultSnapshot(**common, command=None)
    with pytest.raises(ValueError, match="exactly success or failure"):
        PlannerResultSnapshot(
            **common,
            command=command,
            failure_reason=PlannerFailureReason.INTERNAL_ERROR,
        )


@pytest.mark.parametrize(
    "changes",
    [
        {"track_id": True},
        {"track_id": 0},
        {"source_generation": True},
        {"source_generation": -1},
        {"source_frame": True},
        {"source_frame": -1},
        {"completed_monotonic_s": float("nan")},
        {"completed_monotonic_s": float("inf")},
        {"strike_deadline_monotonic_s": float("inf")},
    ],
)
def test_planner_result_rejects_invalid_identity_and_time_values(changes):
    values = {
        "track_id": 7,
        "source_generation": 1,
        "source_frame": 1,
        "strike_deadline_monotonic_s": time.monotonic() + 1.0,
        "completed_monotonic_s": time.monotonic(),
        "command": plan_ok(snapshot()),
    }
    values.update(changes)
    with pytest.raises(ValueError):
        PlannerResultSnapshot(**values)


def test_completed_batch_rejects_misaligned_or_inconsistent_metadata():
    batch_type = hitter_realtime.CompletedResultBatch
    result = PlannerResultSnapshot(
        track_id=7,
        source_generation=1,
        source_frame=1,
        strike_deadline_monotonic_s=time.monotonic() + 1.0,
        completed_monotonic_s=time.monotonic(),
        command=plan_ok(snapshot()),
    )

    with pytest.raises(ValueError, match="length mismatch"):
        batch_type((result,), (), False, 0, ())
    with pytest.raises(ValueError, match="overflowed must match"):
        batch_type((), (), True, 0, ())
    with pytest.raises(ValueError, match="sorted and unique"):
        batch_type((), (), True, 2, (9, 7))
    with pytest.raises(ValueError, match="positive integers"):
        batch_type((), (), True, 1, (True,))
    with pytest.raises(ValueError, match="smaller"):
        batch_type((), (), True, 1, (7, 9))
