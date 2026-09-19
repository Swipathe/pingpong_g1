from __future__ import annotations

from dataclasses import FrozenInstanceError, dataclass, replace
import queue
import threading
import unittest

import numpy as np

from diagnostics.hitter_task_models import (
    BallDiagnosticState,
    NormalizedMocapSample,
    SnapshotKey,
)
from diagnostics.hitter_task_pipeline import (
    AdapterOutput,
    LatestPelvisPose,
    ShadowTaskPipeline,
    planner_reason_code,
)
from utils.hitter_planner import HitterWbcCommand, StrikePlan
from utils.hitter_realtime import (
    BallEstimateSnapshot,
    CompletedResultBatch,
    FrozenPlannerResult,
    IncomingTrackConfirmation,
    LatestOnlyPlannerWorker,
    PlannerResultSnapshot,
    PlannerWorkerStats,
    PlannerWorkerTrace,
)
from utils.hitter_runtime_factory import HitterRuntimeSettings
from utils.hitter_runtime_types import PlannerFailureReason


def _command(*, time_to_strike: float = 0.85) -> HitterWbcCommand:
    target_velocity = np.array([4.0, 5.0, 6.0], dtype=np.float64)
    return HitterWbcCommand(
        strike_type="forehand",
        p_base_target_xy=np.array([-0.4, 0.2], dtype=np.float64),
        v_racket_target_w=target_velocity.copy(),
        time_to_strike=time_to_strike,
        strike_plan=StrikePlan(
            t_strike=time_to_strike,
            p_racket_target=np.array([0.1, -0.2, 1.1], dtype=np.float64),
            v_racket_target=target_velocity.copy(),
            v_ball_in=np.array([-1.0, 0.0, 0.0], dtype=np.float64),
            v_ball_out=np.array([1.0, 0.0, 0.0], dtype=np.float64),
        ),
        strike_table_y_w=-0.2,
        strike_side_source="table_y",
    )


def _command_fields(command: HitterWbcCommand):
    return {
        "strike_type": command.strike_type,
        "p_base_target_xy": tuple(command.p_base_target_xy),
        "v_racket_target_w": tuple(command.v_racket_target_w),
        "time_to_strike": command.time_to_strike,
        "strike_table_y_w": command.strike_table_y_w,
        "strike_side_source": command.strike_side_source,
        "expected_strike_type_from_table_y": (command.expected_strike_type_from_table_y),
        "strike_type_consistent": command.strike_type_consistent,
        "strike_plan": {
            "t_strike": command.strike_plan.t_strike,
            "p_racket_target": tuple(command.strike_plan.p_racket_target),
            "v_racket_target": tuple(command.strike_plan.v_racket_target),
            "v_ball_in": tuple(command.strike_plan.v_ball_in),
            "v_ball_out": tuple(command.strike_plan.v_ball_out),
        },
    }


class _RecursiveCommand(dict):
    def __init__(self):
        super().__init__()
        self.time_to_strike = 0.85
        self["loop"] = self


def _snapshot(
    generation: int,
    *,
    received_s: float = 0.0,
    visible: bool = True,
    ready: bool = True,
    base_valid: bool = True,
    velocity=(-1.0, 0.0, 0.0),
    track_id: int = 1,
) -> BallEstimateSnapshot:
    return BallEstimateSnapshot(
        track_id=track_id,
        generation=generation,
        source_frame=generation,
        source_time_s=float(generation) / 360.0,
        received_monotonic_s=received_s,
        position_w=np.array([1.5, 0.0, 1.0], dtype=np.float32),
        velocity_w=np.array(velocity, dtype=np.float32),
        base_position_w=np.array([0.0, 0.0, 0.8], dtype=np.float32),
        base_quaternion_xyzw=np.array(
            [0.0, 0.0, 0.0, 1.0],
            dtype=np.float32,
        ),
        base_valid=base_valid,
        visible=visible,
        ready=ready,
    )


def _sample(
    generation: int,
    *,
    received_s: float,
    visible: bool = True,
    track_id: int = 1,
) -> NormalizedMocapSample:
    return NormalizedMocapSample(
        input_seq=generation,
        channel="vicon_state_data",
        subject="ball",
        position_w=np.array([1.5, 0.0, 1.0]),
        quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
        valid=visible,
        occluded=not visible,
        source_frame=generation,
        source_time_s=generation / 360.0,
        publish_time_us=generation * 1000,
        received_monotonic_s=received_s,
        wall_time_us=1_700_000_000_000_000 + generation,
        payload_size=96,
        track_id=track_id,
        identity_source="wire_v2",
    )


def _output(
    generation: int,
    *,
    received_s: float,
    visible: bool = True,
    ready: bool = True,
    base_valid: bool = True,
    velocity=(-1.0, 0.0, 0.0),
    track_id: int = 1,
    estimator_sample_count=None,
    bounce_detected: bool = False,
) -> AdapterOutput:
    snapshot = _snapshot(
        generation,
        received_s=received_s,
        visible=visible,
        ready=ready,
        base_valid=base_valid,
        velocity=velocity,
        track_id=track_id,
    )
    return AdapterOutput(
        sample=_sample(
            generation,
            received_s=received_s,
            visible=visible,
            track_id=track_id,
        ),
        snapshot=snapshot,
        warnings=(),
        estimator_sample_count=(generation if estimator_sample_count is None else estimator_sample_count),
        bounce_detected=bounce_detected,
        reset_transition=None,
    )


def _settings(*, required: int = 2) -> HitterRuntimeSettings:
    return HitterRuntimeSettings(
        estimator_sample_rate_hz=360.0,
        planner_update_rate_hz=100.0,
        planner_update_interval_s=0.01,
        minimum_incoming_speed_x_mps=0.2,
        incoming_confirmation_snapshots=required,
        waiting_tts_s=0.92,
        arm_tts_s=0.90,
        minimum_arm_tts_s=0.60,
        maximum_policy_tts_s=0.92,
        completed_result_queue_capacity=64,
        armed_cancel_consecutive_failures=3,
        commit_time_to_strike_s=0.30,
        maximum_racket_target_override_delta_m=0.05,
        maximum_racket_velocity_override_delta_mps=0.75,
        maximum_strike_deadline_override_delta_s=0.05,
        swing_duration_range_s=(1.75, 1.95),
        hitter_seed=0,
        control_tick_s=0.02,
        obs_clip_value=100.0,
    )


class _EventSink:
    def __init__(self) -> None:
        self.drafts = []

    def offer(self, draft) -> bool:
        self.drafts.append(draft)
        return True


class _RawSink:
    def __init__(self) -> None:
        self.items = []

    def __call__(self, sample, binding) -> bool:
        self.items.append((sample, binding))
        return True


class _FakeWorker:
    def __init__(self) -> None:
        self.submissions = []
        self.bundle = (None, None)
        self.batch = CompletedResultBatch(
            results=(),
            frozen_results=(),
            overflowed=False,
            overflow_count=0,
            overflowed_track_ids=(),
        )
        self.closed = False

    def submit(self, snapshot) -> None:
        self.submissions.append(snapshot)

    def latest_result_bundle(self):
        return self.bundle

    def drain_completed_results(self):
        result, frozen = self.bundle
        if result is not None or frozen is not None:
            self.batch = CompletedResultBatch(
                results=() if result is None else (result,),
                frozen_results=(() if result is None else (frozen,)),
                overflowed=False,
                overflow_count=0,
                overflowed_track_ids=(),
            )
            self.bundle = (None, None)
        batch = self.batch
        self.batch = CompletedResultBatch(
            results=(),
            frozen_results=(),
            overflowed=False,
            overflow_count=0,
            overflowed_track_ids=(),
        )
        return batch

    def close(self, timeout_s=None) -> bool:
        self.closed = True
        return True


class _LockOrderProbeWorker(_FakeWorker):
    """Model trace delivery calling back while submit waits on its lock."""

    def __init__(self) -> None:
        super().__init__()
        self.pipeline = None
        self.lock_inversion_detected = False
        self._trace_delivery_lock = threading.Lock()
        self._callback_threads = []

    def submit(self, snapshot) -> None:
        callback_started = threading.Event()

        def deliver_trace() -> None:
            with self._trace_delivery_lock:
                callback_started.set()
                with self.pipeline._state_lock:
                    pass

        callback_thread = threading.Thread(
            target=deliver_trace,
            name="pipeline-lock-order-probe",
            daemon=True,
        )
        self._callback_threads.append(callback_thread)
        callback_thread.start()
        if not callback_started.wait(1.0):
            raise AssertionError("trace callback did not start")
        acquired = self._trace_delivery_lock.acquire(timeout=0.25)
        if acquired:
            self._trace_delivery_lock.release()
        else:
            self.lock_inversion_detected = True
        self.submissions.append(snapshot)

    def wait_callbacks(self) -> bool:
        for thread in self._callback_threads:
            thread.join(1.0)
        return all(not thread.is_alive() for thread in self._callback_threads)


class _FakeAdapter:
    def __init__(self) -> None:
        self.track_id = 1
        self.reset_calls = 0
        self.ball_state_estimator = type(
            "_FakeBallStateEstimator",
            (),
            {"window_size": 31},
        )()
        self.pose = LatestPelvisPose(
            position_w=np.array([1.0, 2.0, 0.8], dtype=np.float32),
            quaternion_xyzw=np.array(
                [0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)],
                dtype=np.float32,
            ),
            valid=True,
            occluded=False,
            source_frame=10,
            source_time_s=1.0,
            publish_time_us=1_000_000,
            received_monotonic_s=1.0,
            wall_time_us=1,
        )

    def copy_latest_pelvis(self):
        return self.pose

    def reset_estimator_after_strike(self) -> int:
        self.reset_calls += 1
        self.track_id += 1
        return self.track_id


class PlannerWorkerTraceTests(unittest.TestCase):
    def test_trace_listener_delivery_is_strictly_ordered_when_first_callback_blocks(self) -> None:
        first_entered = threading.Event()
        release_first = threading.Event()
        second_delivered = threading.Event()
        planned = threading.Event()
        delivered = []

        def listener(trace):
            if trace.trace_seq == 1:
                first_entered.set()
                self.assertTrue(release_first.wait(2.0))
            if trace.trace_seq == 2:
                second_delivered.set()
            delivered.append(trace.trace_seq)

        def plan(_snapshot_value):
            planned.set()
            return _command()

        worker = LatestOnlyPlannerWorker(plan, trace_listener=listener)
        submit_thread = threading.Thread(target=lambda: worker.submit(_snapshot(1)))
        submit_thread.start()
        self.assertTrue(first_entered.wait(2.0))
        self.assertFalse(second_delivered.wait(0.1))
        release_first.set()
        submit_thread.join(2.0)
        self.assertFalse(submit_thread.is_alive())
        self.assertTrue(planned.wait(2.0))
        self.assertTrue(worker.close(timeout_s=2.0))
        self.assertEqual(delivered, sorted(delivered))

    def test_no_listener_keeps_result_behavior_and_close_none_returns_true(self) -> None:
        planned = threading.Event()

        def plan(_snapshot_value):
            planned.set()
            return _command()

        worker = LatestOnlyPlannerWorker(plan)
        worker.submit(_snapshot(1))
        self.assertTrue(planned.wait(2.0))
        self.assertTrue(worker.close())
        result, frozen = worker.latest_result_bundle()
        self.assertEqual(result.source_generation, 1)
        self.assertIsNone(frozen)
        self.assertEqual(worker.stats.submitted, 1)
        self.assertEqual(worker.stats.completed, 1)

    def test_no_listener_never_freezes_a_recursive_production_command(self) -> None:
        planned = threading.Event()
        command = _RecursiveCommand()

        def plan(_snapshot_value):
            planned.set()
            return command

        worker = LatestOnlyPlannerWorker(plan)
        worker.submit(_snapshot(1))
        self.assertTrue(planned.wait(2.0))
        self.assertTrue(worker.close(timeout_s=2.0))
        result, frozen = worker.latest_result_bundle()
        self.assertIs(result.command, command)
        self.assertIsNone(result.error)
        self.assertIsNone(frozen)
        self.assertEqual(worker.stats.completed, 1)

    def test_diagnostic_freeze_failure_cannot_kill_listener_worker(self) -> None:
        planned = threading.Event()
        command = _RecursiveCommand()
        traces = queue.Queue()

        def plan(_snapshot_value):
            planned.set()
            return command

        worker = LatestOnlyPlannerWorker(
            plan,
            trace_listener=traces.put,
        )
        worker.submit(_snapshot(1))
        self.assertTrue(planned.wait(2.0))
        self.assertTrue(worker.close(timeout_s=2.0))
        result, frozen = worker.latest_result_bundle()
        self.assertIs(result.command, command)
        self.assertIsNone(result.error)
        self.assertEqual(worker.stats.completed, 1)
        self.assertIsNone(frozen.command_fields)
        self.assertEqual(frozen.error_type, "DiagnosticFreezeError")
        self.assertIn("RecursionError", frozen.error_text)
        delivered = []
        while not traces.empty():
            delivered.append(traces.get_nowait())
        self.assertTrue(
            any(trace.kind == "complete" and trace.result.error_type == "DiagnosticFreezeError" for trace in delivered)
        )

    def test_long_planner_error_keeps_worker_and_frozen_result_identity(self) -> None:
        completed = threading.Event()
        long_message = "x" * 3000

        def plan(_snapshot_value):
            raise ValueError(long_message)

        def listener(trace):
            if trace.kind == "complete":
                completed.set()

        worker = LatestOnlyPlannerWorker(
            plan,
            trace_listener=listener,
        )
        worker.submit(_snapshot(1))
        self.assertTrue(completed.wait(2.0))
        self.assertTrue(worker.close(timeout_s=2.0))
        result, frozen = worker.latest_result_bundle()
        self.assertEqual(result.error, PlannerFailureReason.INTERNAL_ERROR.value)
        self.assertGreater(len(result.error_text), 2048)
        self.assertEqual(len(frozen.error_text), 2048)
        self.assertTrue(ShadowTaskPipeline._result_bundle_matches(result, frozen))

    def test_trace_sequence_replacements_and_frozen_nested_command_are_exact(self) -> None:
        started = threading.Event()
        release = threading.Event()
        completed_latest = threading.Event()
        traces = queue.Queue()

        def plan(_snapshot_value):
            started.set()
            self.assertTrue(release.wait(2.0))
            return _command()

        def listener(trace):
            traces.put(trace)
            if trace.kind == "complete" and trace.result is not None and trace.result.snapshot_key.generation == 3:
                completed_latest.set()

        worker = LatestOnlyPlannerWorker(plan, trace_listener=listener)
        worker.submit(_snapshot(1))
        self.assertTrue(started.wait(2.0))
        worker.submit(_snapshot(2))
        worker.submit(_snapshot(3))
        release.set()
        self.assertTrue(completed_latest.wait(2.0))
        self.assertTrue(worker.close(timeout_s=2.0))

        all_traces = []
        while not traces.empty():
            all_traces.append(traces.get_nowait())
        self.assertEqual(
            sorted(trace.trace_seq for trace in all_traces),
            list(range(1, len(all_traces) + 1)),
        )
        replacement = next(trace for trace in all_traces if trace.kind == "pending_replaced")
        self.assertEqual(replacement.replaced_snapshot.generation, 2)
        self.assertEqual(replacement.snapshot.generation, 3)
        self.assertTrue(
            any(
                trace.kind == "latest_result_replaced"
                and trace.replaced_result.snapshot_key.generation == 1
                and trace.result.snapshot_key.generation == 3
                for trace in all_traces
            )
        )
        result, frozen = worker.latest_result_bundle()
        self.assertEqual(result.source_generation, 3)
        self.assertEqual(frozen.snapshot_key, SnapshotKey(1, 3))
        self.assertEqual(
            frozen.command_fields["strike_plan"]["p_racket_target"],
            (0.1, -0.2, 1.1),
        )
        with self.assertRaises(TypeError):
            frozen.command_fields["other"] = 1
        with self.assertRaises(TypeError):
            frozen.command_fields["strike_plan"]["other"] = 1
        self.assertEqual(worker.stats.dropped_pending, 1)

    def test_listener_failure_does_not_change_result_and_finite_close_can_retry(self) -> None:
        started = threading.Event()
        release = threading.Event()

        def plan(_snapshot_value):
            started.set()
            self.assertTrue(release.wait(2.0))
            return _command()

        def broken_listener(_trace):
            raise RuntimeError("diagnostic sink failed")

        worker = LatestOnlyPlannerWorker(
            plan,
            trace_listener=broken_listener,
        )
        worker.submit(_snapshot(1))
        self.assertTrue(started.wait(2.0))
        self.assertFalse(worker.close(timeout_s=0.0))
        release.set()
        self.assertTrue(worker.close(timeout_s=2.0))
        result, frozen = worker.latest_result_bundle()
        self.assertIsNone(result.error)
        self.assertEqual(frozen.snapshot_key, SnapshotKey(1, 1))
        self.assertGreaterEqual(worker.trace_listener_failures, 2)

    def test_incoming_snapshot_is_immutable_and_tracks_latched_state(self) -> None:
        incoming = IncomingTrackConfirmation(
            minimum_speed_x_mps=0.2,
            required_consecutive_snapshots=2,
        )
        self.assertFalse(incoming.observe(track_id=7, velocity_x_mps=-1.0))
        first = incoming.snapshot()
        self.assertEqual((first.track_id, first.consecutive_count), (7, 1))
        self.assertFalse(first.confirmed)
        self.assertTrue(incoming.observe(track_id=7, velocity_x_mps=-1.0))
        self.assertTrue(incoming.snapshot().confirmed)
        with self.assertRaises(FrozenInstanceError):
            first.confirmed = True


class ShadowTaskPipelineTests(unittest.TestCase):
    def _pipeline(
        self,
        *,
        required: int = 2,
        worker=None,
        adapter=None,
        planner=None,
    ):
        worker = _FakeWorker() if worker is None else worker
        adapter = _FakeAdapter() if adapter is None else adapter
        raw = _RawSink()
        events = _EventSink()
        pipeline = ShadowTaskPipeline(
            adapter=adapter,
            settings=_settings(required=required),
            event_sink=events,
            raw_sink=raw,
            worker=worker,
            planner=planner,
        )
        return pipeline, worker, adapter, raw, events

    @staticmethod
    def _worker_trace(
        *,
        trace_seq,
        kind,
        monotonic_s,
        snapshot,
        result=None,
        replaced_snapshot=None,
        replaced_result=None,
    ):
        return PlannerWorkerTrace(
            trace_seq=trace_seq,
            kind=kind,
            monotonic_s=monotonic_s,
            snapshot=snapshot,
            result=result,
            replaced_snapshot=replaced_snapshot,
            replaced_result=replaced_result,
            stats=PlannerWorkerStats(1, int(result is not None), 0, 0),
        )

    @staticmethod
    def _attempt_stage_drafts(events):
        return [draft for draft in events.drafts if draft.kind == "attempt_transition"]

    def _ball_diagnostics(self, pipeline):
        tracker = getattr(pipeline, "ball_diagnostics", None)
        self.assertIsNotNone(
            tracker,
            "pipeline must expose independent ball diagnostics",
        )
        return tracker

    def test_ball_diagnostic_state_detaches_immutable_velocity_tuple(self) -> None:
        source_velocity = [-1, 2, 3]
        state = BallDiagnosticState(
            status="READY",
            estimator_sample_count=31,
            estimator_window_size=31,
            speed_mps=4.0,
            velocity_world_mps=source_velocity,
            incoming_count=1,
            incoming_required_count=3,
            incoming_status="INCOMING",
            blocker=None,
        )

        source_velocity[0] = 99

        self.assertIsInstance(state.velocity_world_mps, tuple)
        self.assertEqual(
            state.velocity_world_mps,
            (-1.0, 2.0, 3.0),
        )
        with self.assertRaises(FrozenInstanceError):
            state.velocity_world_mps = (0.0, 0.0, 0.0)

    def test_ball_diagnostic_state_rejects_invalid_velocity_tuple(self) -> None:
        for velocity in ((1.0, 2.0), (1.0, 2.0, float("nan"))):
            with self.subTest(velocity=velocity):
                with self.assertRaisesRegex(
                    ValueError,
                    "velocity_world_mps",
                ):
                    BallDiagnosticState(
                        status="READY",
                        estimator_sample_count=31,
                        estimator_window_size=31,
                        speed_mps=None,
                        velocity_world_mps=velocity,
                        incoming_count=None,
                        incoming_required_count=3,
                        incoming_status="NOT_EVALUATED",
                        blocker="ESTIMATE_NONFINITE",
                    )

    def test_ingest_binds_raw_before_event_driven_received_time_throttle(self) -> None:
        pipeline, worker, _adapter_value, raw, _events = self._pipeline()
        for generation, received in ((1, 5.0), (2, 5.005), (3, 5.01)):
            pipeline.ingest_adapter_output(_output(generation, received_s=received))

        self.assertEqual(
            [snapshot.generation for snapshot in worker.submissions],
            [1, 3],
        )
        self.assertEqual(
            [sample.input_seq for sample, _binding in raw.items],
            [1, 2, 3],
        )
        self.assertTrue(all(binding is not None for _, binding in raw.items))

    def test_throttled_ingest_keeps_one_canonical_ball_observation(self) -> None:
        pipeline, _worker, _adapter, _raw, _events = self._pipeline(required=3)
        first = _output(
            11,
            received_s=5.0,
            velocity=(-1.0, 0.1, 0.2),
            track_id=4,
            estimator_sample_count=31,
        )
        first = replace(
            first,
            sample=replace(
                first.sample,
                source_frame=101,
                position_w=np.array([1.1, 1.2, 1.3]),
            ),
            snapshot=replace(
                first.snapshot,
                source_frame=101,
                position_w=np.array([1.4, 1.5, 1.6]),
            ),
        )
        throttled = _output(
            12,
            received_s=5.005,
            velocity=(-2.0, 2.1, 2.2),
            track_id=4,
            estimator_sample_count=32,
        )
        throttled = replace(
            throttled,
            sample=replace(
                throttled.sample,
                source_frame=102,
                position_w=np.array([2.1, 2.2, 2.3]),
            ),
            snapshot=replace(
                throttled.snapshot,
                source_frame=102,
                position_w=np.array([2.4, 2.5, 2.6]),
            ),
        )

        pipeline.ingest_adapter_output(first)
        pipeline.ingest_adapter_output(throttled)

        state = self._ball_diagnostics(pipeline).snapshot()
        self.assertEqual(
            (
                state.source_frame,
                state.track_id,
                state.generation,
                state.estimator_sample_count,
                state.incoming_count,
            ),
            (101, 4, 11, 31, 1),
        )
        np.testing.assert_allclose(
            state.raw_position_w,
            (1.1, 1.2, 1.3),
        )
        np.testing.assert_allclose(
            state.estimated_position_w,
            (1.4, 1.5, 1.6),
        )
        np.testing.assert_allclose(
            state.velocity_world_mps,
            (-1.0, 0.1, 0.2),
        )

    def test_ingest_submits_without_pipeline_trace_lock_inversion(self) -> None:
        worker = _LockOrderProbeWorker()
        pipeline, _worker, _adapter, _raw, _events = self._pipeline(worker=worker)
        worker.pipeline = pipeline

        pipeline.ingest_adapter_output(_output(1, received_s=1.0))

        self.assertTrue(worker.wait_callbacks())
        self.assertFalse(worker.lock_inversion_detected)
        self.assertEqual(
            [snapshot.generation for snapshot in worker.submissions],
            [1],
        )

    def test_ball_diagnostics_reaches_estimator_window_without_pelvis(self) -> None:
        pipeline, _worker, _adapter, _raw, _events = self._pipeline(required=3)

        diagnostics = self._ball_diagnostics(pipeline)
        initial = diagnostics.snapshot()
        self.assertEqual(initial.status, "NOT_SEEN")
        self.assertIsNone(initial.estimator_sample_count)
        self.assertEqual(initial.estimator_window_size, 31)
        self.assertIsNone(initial.incoming_count)
        self.assertEqual(initial.incoming_required_count, 3)
        self.assertEqual(initial.incoming_status, "NOT_EVALUATED")

        for generation in range(1, 32):
            pipeline.ingest_adapter_output(
                _output(
                    generation,
                    received_s=float(generation) * 0.02,
                    ready=generation == 31,
                    base_valid=False,
                    velocity=(-1.0, 0.0, 0.0),
                )
            )
            state = diagnostics.snapshot()
            self.assertEqual(state.estimator_sample_count, generation)
            self.assertEqual(state.estimator_window_size, 31)
            if generation < 31:
                self.assertEqual(state.status, "ESTIMATING")
                self.assertIsNone(state.speed_mps)
                self.assertIsNone(state.velocity_world_mps)
                self.assertIsNone(state.incoming_count)
                self.assertEqual(
                    state.incoming_status,
                    "NOT_EVALUATED",
                )

        ready = diagnostics.snapshot()
        self.assertEqual(ready.status, "READY")
        self.assertEqual(ready.speed_mps, 1.0)
        self.assertEqual(
            ready.velocity_world_mps,
            (-1.0, 0.0, 0.0),
        )
        self.assertEqual(ready.incoming_count, 1)
        self.assertEqual(ready.incoming_status, "INCOMING")
        self.assertIsNone(ready.blocker)

    def test_ball_diagnostics_deduplicates_ready_snapshots_without_touching_production(self) -> None:
        pipeline, _worker, _adapter, _raw, _events = self._pipeline(required=3)
        samples = (
            _output(
                1,
                received_s=0.02,
                base_valid=False,
                velocity=(-0.2, 0.0, 0.0),
            ),
            _output(
                1,
                received_s=0.04,
                base_valid=False,
                velocity=(-0.2, 0.0, 0.0),
            ),
            _output(
                2,
                received_s=0.06,
                base_valid=False,
                velocity=(-0.2, 0.0, 0.0),
            ),
            _output(
                3,
                received_s=0.08,
                base_valid=False,
                velocity=(-0.2, 0.0, 0.0),
            ),
        )

        observed_counts = []
        for output in samples:
            pipeline.ingest_adapter_output(output)
            observed_counts.append(self._ball_diagnostics(pipeline).snapshot().incoming_count)

        self.assertEqual(observed_counts, [1, 1, 2, 3])
        diagnostic = self._ball_diagnostics(pipeline).snapshot()
        self.assertEqual(diagnostic.incoming_status, "INCOMING")
        self.assertEqual(diagnostic.incoming_required_count, 3)
        production = pipeline.incoming.snapshot()
        self.assertIsNone(production.track_id)
        self.assertEqual(production.consecutive_count, 0)
        self.assertFalse(production.confirmed)

    def test_ball_diagnostics_deduplicates_nonadjacent_generation_per_epoch(self) -> None:
        pipeline, _worker, _adapter, _raw, _events = self._pipeline(required=3)
        observed_counts = []
        for track_id, generation, received_s in (
            (1, 1, 0.02),
            (1, 2, 0.04),
            (1, 1, 0.06),
            (2, 1, 0.08),
        ):
            pipeline.ingest_adapter_output(
                _output(
                    generation,
                    received_s=received_s,
                    base_valid=False,
                    velocity=(-1.0, 0.0, 0.0),
                    track_id=track_id,
                )
            )
            observed_counts.append(self._ball_diagnostics(pipeline).snapshot().incoming_count)

        self.assertEqual(observed_counts, [1, 2, 2, 1])

    def test_ball_diagnostics_nonincoming_sample_resets_consecutive_count(self) -> None:
        pipeline, _worker, _adapter, _raw, _events = self._pipeline(required=3)
        pipeline.ingest_adapter_output(
            _output(
                1,
                received_s=0.02,
                base_valid=False,
                velocity=(-1.0, 0.0, 0.0),
            )
        )
        self.assertEqual(
            self._ball_diagnostics(pipeline).snapshot().incoming_count,
            1,
        )

        pipeline.ingest_adapter_output(
            _output(
                2,
                received_s=0.04,
                base_valid=False,
                velocity=(0.1, 0.0, 0.0),
            )
        )

        state = self._ball_diagnostics(pipeline).snapshot()
        self.assertEqual(state.status, "READY")
        self.assertEqual(state.incoming_count, 0)
        self.assertEqual(state.incoming_status, "NOT_INCOMING")
        self.assertEqual(state.blocker, "INCOMING_SPEED_REJECTED")

    def test_ball_diagnostics_bounce_inside_throttle_restarts_confirmation(self) -> None:
        pipeline, worker, _adapter, _raw, _events = self._pipeline(required=3)
        for generation, received_s in ((1, 0.02), (2, 0.04)):
            pipeline.ingest_adapter_output(
                _output(
                    generation,
                    received_s=received_s,
                    base_valid=False,
                    velocity=(-1.0, 0.0, 0.0),
                )
            )
        self.assertEqual(
            self._ball_diagnostics(pipeline).snapshot().incoming_count,
            2,
        )

        pipeline.ingest_adapter_output(
            _output(
                3,
                received_s=0.045,
                ready=False,
                base_valid=False,
                estimator_sample_count=1,
                bounce_detected=True,
            )
        )
        warming = self._ball_diagnostics(pipeline).snapshot()
        self.assertEqual(warming.status, "ESTIMATING")
        self.assertEqual(warming.estimator_sample_count, 1)
        self.assertIsNone(warming.incoming_count)

        pipeline.ingest_adapter_output(
            _output(
                4,
                received_s=0.06,
                base_valid=False,
                velocity=(-1.0, 0.0, 0.0),
                estimator_sample_count=3,
            )
        )
        restarted = self._ball_diagnostics(pipeline).snapshot()
        self.assertEqual(restarted.incoming_count, 1)
        self.assertEqual(restarted.incoming_status, "INCOMING")
        self.assertEqual(
            [snapshot.generation for snapshot in worker.submissions],
            [1, 2, 4],
        )

    def test_ball_diagnostics_track_end_is_explicit_and_next_epoch_restarts_count(self) -> None:
        pipeline, _worker, _adapter, _raw, _events = self._pipeline(required=3)
        pipeline.ingest_adapter_output(
            _output(
                1,
                received_s=0.02,
                base_valid=False,
                velocity=(-1.0, 0.0, 0.0),
            )
        )
        pipeline.ingest_adapter_output(
            _output(
                2,
                received_s=0.04,
                visible=False,
                ready=False,
                base_valid=False,
                track_id=2,
                estimator_sample_count=0,
            )
        )

        ended = self._ball_diagnostics(pipeline).snapshot()
        self.assertEqual(ended.status, "TRACK_ENDED")
        self.assertIsNone(ended.estimator_sample_count)
        self.assertEqual(ended.estimator_window_size, 31)
        self.assertIsNone(ended.speed_mps)
        self.assertIsNone(ended.velocity_world_mps)
        self.assertIsNone(ended.incoming_count)
        self.assertEqual(ended.incoming_status, "TRACK_ENDED")
        self.assertEqual(ended.blocker, "BALL_NOT_VISIBLE")

        pipeline.ingest_adapter_output(
            _output(
                3,
                received_s=0.06,
                base_valid=False,
                velocity=(-1.0, 0.0, 0.0),
                track_id=2,
                estimator_sample_count=1,
            )
        )
        restarted = self._ball_diagnostics(pipeline).snapshot()
        self.assertEqual(restarted.incoming_count, 1)
        self.assertEqual(restarted.incoming_status, "INCOMING")

    def test_ball_diagnostics_observes_final_invisible_sample_inside_throttle(self) -> None:
        pipeline, worker, _adapter, _raw, _events = self._pipeline(required=3)
        pipeline.ingest_adapter_output(
            _output(
                1,
                received_s=1.0,
                base_valid=False,
                velocity=(-1.0, 0.0, 0.0),
            )
        )
        pipeline.ingest_adapter_output(
            _output(
                2,
                received_s=1.005,
                visible=False,
                ready=False,
                base_valid=False,
                track_id=2,
                estimator_sample_count=0,
            )
        )

        terminal = self._ball_diagnostics(pipeline).snapshot()
        self.assertEqual(terminal.status, "TRACK_ENDED")
        self.assertEqual(terminal.incoming_status, "TRACK_ENDED")
        self.assertEqual(terminal.blocker, "BALL_NOT_VISIBLE")
        self.assertIsNone(terminal.incoming_count)
        self.assertEqual(
            [snapshot.generation for snapshot in worker.submissions],
            [1],
        )

    def test_throttled_and_pending_invisible_samples_do_not_reset_early(self) -> None:
        pipeline, worker, _adapter_value, _raw, _events = self._pipeline(required=2)
        with self.assertRaisesRegex(ValueError, "stably incoming"):
            pipeline.plan_snapshot(_snapshot(1))
        self.assertEqual(pipeline.incoming.snapshot().consecutive_count, 1)
        pipeline.ingest_adapter_output(_output(1, received_s=1.0))
        pipeline.ingest_adapter_output(_output(2, received_s=1.005, visible=False))
        self.assertEqual(
            [snapshot.generation for snapshot in worker.submissions],
            [1],
        )
        self.assertEqual(pipeline.incoming.snapshot().consecutive_count, 1)
        self.assertEqual(pipeline.lifecycle.phase.value, "waiting")
        pipeline.ingest_adapter_output(_output(3, received_s=1.01, visible=False))
        self.assertEqual(
            [snapshot.generation for snapshot in worker.submissions],
            [1, 3],
        )
        self.assertEqual(pipeline.incoming.snapshot().consecutive_count, 1)
        with self.assertRaisesRegex(RuntimeError, "track ended"):
            pipeline.plan_snapshot(worker.submissions[-1])
        self.assertEqual(pipeline.incoming.snapshot().consecutive_count, 0)

    def test_plan_fn_preserves_count_across_not_ready_and_base_invalid(self) -> None:
        planner_calls = []
        planned_command = _command()

        class Planner:
            def plan_command(self, *args, **kwargs):
                planner_calls.append((args, kwargs))
                return planned_command

        pipeline, _worker, _adapter_value, _raw, _events = self._pipeline(
            required=2,
            planner=Planner(),
        )
        with self.assertRaisesRegex(ValueError, "stably incoming"):
            pipeline.plan_snapshot(_snapshot(1, velocity=(-1.0, 0.0, 0.0)))
        self.assertEqual(
            pipeline.incoming.snapshot().consecutive_count,
            1,
        )
        with self.assertRaisesRegex(ValueError, "not ready"):
            pipeline.plan_snapshot(_snapshot(2, ready=False))
        with self.assertRaisesRegex(ValueError, "base pose"):
            pipeline.plan_snapshot(_snapshot(3, base_valid=False))
        self.assertEqual(
            pipeline.incoming.snapshot().consecutive_count,
            1,
        )
        with self.assertRaisesRegex(ValueError, "stably incoming"):
            pipeline.plan_snapshot(_snapshot(4, velocity=(0.5, 0.0, 0.0)))
        self.assertEqual(
            pipeline.incoming.snapshot().consecutive_count,
            0,
        )
        with self.assertRaisesRegex(ValueError, "stably incoming"):
            pipeline.plan_snapshot(_snapshot(5))
        self.assertIs(pipeline.plan_snapshot(_snapshot(6)), planned_command)
        self.assertEqual(len(planner_calls), 1)
        with self.assertRaisesRegex(RuntimeError, "track ended"):
            pipeline.plan_snapshot(_snapshot(7, visible=False))
        self.assertEqual(
            pipeline.incoming.snapshot().consecutive_count,
            0,
        )

    def test_reason_mapper_keeps_fixed_codes_without_losing_raw_error(self) -> None:
        cases = (
            (PlannerFailureReason.ESTIMATOR_NOT_READY, "ignored text"),
            (PlannerFailureReason.BASE_POSE_INVALID, "ignored text"),
            (PlannerFailureReason.BALL_NOT_INCOMING, "ignored text"),
            (PlannerFailureReason.NO_FUTURE_CROSSING, "ignored text"),
            (PlannerFailureReason.HIT_HEIGHT_OUT_OF_RANGE, "ignored text"),
            (PlannerFailureReason.NONFINITE_INPUT_OR_OUTPUT, "ignored text"),
            (PlannerFailureReason.TRACK_ENDED, "ignored text"),
            (PlannerFailureReason.INTERNAL_ERROR, "other"),
        )
        for failure_reason, error_text in cases:
            with self.subTest(error_text=error_text):
                frozen = FrozenPlannerResult(
                    snapshot_key=SnapshotKey(1, 2),
                    source_frame=3,
                    strike_deadline_monotonic_s=float("nan"),
                    completed_monotonic_s=4.0,
                    command_fields=None,
                    error_type="PlannerRejected",
                    error_text=error_text,
                    failure_reason=failure_reason,
                )
                self.assertEqual(
                    planner_reason_code(frozen),
                    failure_reason.value,
                )
                self.assertEqual(frozen.error_type, "PlannerRejected")
                self.assertEqual(frozen.error_text, error_text)

        legacy_text_only = FrozenPlannerResult(
            snapshot_key=SnapshotKey(1, 2),
            source_frame=3,
            strike_deadline_monotonic_s=float("nan"),
            completed_monotonic_s=4.0,
            command_fields=None,
            error_type="ValueError",
            error_text="HITTER base pose is not valid.",
            failure_reason=None,
        )
        self.assertIsNone(planner_reason_code(legacy_text_only))

    def test_worker_trace_records_canonical_progress_once_per_identity(self) -> None:
        class Planner:
            def plan_command(self, *_args, **_kwargs):
                return _command()

        pipeline, _worker, _adapter, _raw, events = self._pipeline(
            required=2,
            planner=Planner(),
        )
        first = _snapshot(1, received_s=0.0)
        pipeline.ingest_adapter_output(_output(1, received_s=0.0))
        pipeline._on_worker_trace(
            self._worker_trace(
                trace_seq=1,
                kind="start",
                monotonic_s=0.001,
                snapshot=first,
            )
        )
        with self.assertRaisesRegex(ValueError, "stably incoming"):
            pipeline.plan_snapshot(first)
        first_result = FrozenPlannerResult(
            snapshot_key=SnapshotKey(1, 1),
            source_frame=1,
            strike_deadline_monotonic_s=float("nan"),
            completed_monotonic_s=0.002,
            command_fields=None,
            error_type="PlannerRejected",
            error_text="HITTER ball track is not stably incoming.",
            failure_reason=PlannerFailureReason.BALL_NOT_INCOMING,
        )
        first_complete = self._worker_trace(
            trace_seq=2,
            kind="complete",
            monotonic_s=0.002,
            snapshot=first,
            result=first_result,
        )
        pipeline._on_worker_trace(first_complete)
        pipeline._on_worker_trace(first_complete)

        second = _snapshot(2, received_s=0.02)
        pipeline.ingest_adapter_output(_output(2, received_s=0.02))
        pipeline._on_worker_trace(
            self._worker_trace(
                trace_seq=3,
                kind="start",
                monotonic_s=0.021,
                snapshot=second,
            )
        )
        command = pipeline.plan_snapshot(second)
        second_result = FrozenPlannerResult(
            snapshot_key=SnapshotKey(1, 2),
            source_frame=2,
            strike_deadline_monotonic_s=0.85,
            completed_monotonic_s=0.022,
            command_fields=_command_fields(command),
            error_type=None,
            error_text=None,
        )
        second_complete = self._worker_trace(
            trace_seq=4,
            kind="complete",
            monotonic_s=0.022,
            snapshot=second,
            result=second_result,
        )
        pipeline._on_worker_trace(second_complete)
        pipeline._on_worker_trace(second_complete)

        stages = self._attempt_stage_drafts(events)
        canonical = [
            draft
            for draft in stages
            if draft.payload["stage"]
            in (
                "ESTIMATOR_READY",
                "INCOMING_CONFIRMING",
                "INCOMING_CONFIRMED",
                "PLANNER_REJECTED",
                "PLANNER_SUCCEEDED",
            )
        ]
        self.assertEqual(
            [draft.payload["stage"] for draft in canonical],
            [
                "ESTIMATOR_READY",
                "INCOMING_CONFIRMING",
                "PLANNER_REJECTED",
                "INCOMING_CONFIRMED",
                "PLANNER_SUCCEEDED",
            ],
        )
        self.assertEqual(canonical[0].monotonic_s, 0.001)
        self.assertEqual(
            canonical[1].payload["values"],
            {"count": 1, "required": 2},
        )
        self.assertEqual(canonical[1].monotonic_s, 0.001)
        self.assertEqual(
            canonical[2].payload["reason_code"],
            "BALL_NOT_INCOMING",
        )
        self.assertEqual(canonical[2].monotonic_s, 0.002)
        self.assertEqual(
            canonical[3].payload["values"],
            {"count": 2, "required": 2},
        )
        self.assertEqual(canonical[3].monotonic_s, 0.021)
        self.assertEqual(canonical[4].monotonic_s, 0.022)
        self.assertEqual(
            [draft.payload["snapshot_key"] for draft in canonical[-2:]],
            [
                SnapshotKey(1, 2).to_json_dict(),
                SnapshotKey(1, 2).to_json_dict(),
            ],
        )

    def test_latest_result_replacement_after_tick_consumption_is_not_counted(
        self,
    ) -> None:
        pipeline, worker, _adapter, _raw, _events = self._pipeline(required=1)
        pipeline.ingest_adapter_output(_output(1, received_s=0.0))
        pipeline.ingest_adapter_output(_output(2, received_s=0.02))
        first_command = _command(time_to_strike=2.0)
        first_result = PlannerResultSnapshot(
            track_id=1,
            source_generation=1,
            source_frame=1,
            strike_deadline_monotonic_s=2.0,
            completed_monotonic_s=0.001,
            command=first_command,
        )
        first_frozen = FrozenPlannerResult(
            snapshot_key=SnapshotKey(1, 1),
            source_frame=1,
            strike_deadline_monotonic_s=2.0,
            completed_monotonic_s=0.001,
            command_fields=_command_fields(first_command),
            error_type=None,
            error_text=None,
        )
        worker.bundle = (first_result, first_frozen)
        pipeline.tick(
            lifecycle_now_s=0.0,
            obs_now_s=0.01,
            wall_time_us=1,
        )

        second_frozen = replace(
            first_frozen,
            snapshot_key=SnapshotKey(1, 2),
            source_frame=2,
            completed_monotonic_s=0.021,
        )
        pipeline._on_worker_trace(
            self._worker_trace(
                trace_seq=2,
                kind="latest_result_replaced",
                monotonic_s=0.021,
                snapshot=_snapshot(2, received_s=0.02),
                result=second_frozen,
                replaced_result=first_frozen,
            )
        )

        self.assertEqual(worker.batch.overflow_count, 0)

    def test_two_completions_between_ticks_count_one_unconsumed_replacement(
        self,
    ) -> None:
        pipeline, worker, _adapter, _raw, _events = self._pipeline(required=1)
        pipeline.ingest_adapter_output(_output(1, received_s=0.0))
        pipeline.ingest_adapter_output(_output(2, received_s=0.02))
        first_command = _command(time_to_strike=2.0)
        first_frozen = FrozenPlannerResult(
            snapshot_key=SnapshotKey(1, 1),
            source_frame=1,
            strike_deadline_monotonic_s=2.0,
            completed_monotonic_s=0.001,
            command_fields=_command_fields(first_command),
            error_type=None,
            error_text=None,
        )
        second_frozen = replace(
            first_frozen,
            snapshot_key=SnapshotKey(1, 2),
            source_frame=2,
            completed_monotonic_s=0.021,
        )
        replacement = self._worker_trace(
            trace_seq=2,
            kind="latest_result_replaced",
            monotonic_s=0.021,
            snapshot=_snapshot(2, received_s=0.02),
            result=second_frozen,
            replaced_result=first_frozen,
        )
        pipeline._on_worker_trace(replacement)
        pipeline._on_worker_trace(replacement)
        second_result = PlannerResultSnapshot(
            track_id=1,
            source_generation=2,
            source_frame=2,
            strike_deadline_monotonic_s=2.0,
            completed_monotonic_s=0.021,
            command=first_command,
        )
        worker.bundle = (second_result, second_frozen)
        pipeline.tick(
            lifecycle_now_s=0.02,
            obs_now_s=0.03,
            wall_time_us=2,
        )

        self.assertEqual(worker.batch.overflow_count, 0)

    def test_pelvis_unavailable_trace_is_terminal_primary_blocker(self) -> None:
        pipeline, _worker, _adapter, _raw, _events = self._pipeline(required=3)
        snapshot = _snapshot(1, received_s=1.0, base_valid=False)
        pipeline.ingest_adapter_output(_output(1, received_s=1.0, base_valid=False))
        pipeline._on_worker_trace(
            self._worker_trace(
                trace_seq=1,
                kind="start",
                monotonic_s=1.001,
                snapshot=snapshot,
            )
        )
        with self.assertRaisesRegex(ValueError, "base pose"):
            pipeline.plan_snapshot(snapshot)
        pipeline._on_worker_trace(
            self._worker_trace(
                trace_seq=2,
                kind="complete",
                monotonic_s=1.002,
                snapshot=snapshot,
                result=FrozenPlannerResult(
                    snapshot_key=SnapshotKey(1, 1),
                    source_frame=1,
                    strike_deadline_monotonic_s=float("nan"),
                    completed_monotonic_s=1.002,
                    command_fields=None,
                    error_type="PlannerRejected",
                    error_text="HITTER base pose is not valid.",
                    failure_reason=PlannerFailureReason.BASE_POSE_INVALID,
                ),
            )
        )
        pipeline.ingest_adapter_output(_output(2, received_s=1.1, visible=False))
        pipeline.attempt_tracker.advance(now_monotonic_s=1.300001)

        summary = pipeline.attempt_tracker.summary_for_attempt(1)
        self.assertEqual(summary.primary_blocker, "BASE_POSE_INVALID")

    def test_tick_records_late_skip_once(self) -> None:
        pipeline, worker, _adapter, _raw, events = self._pipeline(required=1)
        pipeline.ingest_adapter_output(_output(1, received_s=0.0))
        command = _command(time_to_strike=0.5)
        worker.bundle = (
            PlannerResultSnapshot(
                track_id=1,
                source_generation=1,
                source_frame=1,
                strike_deadline_monotonic_s=0.5,
                completed_monotonic_s=0.01,
                command=command,
            ),
            FrozenPlannerResult(
                snapshot_key=SnapshotKey(1, 1),
                source_frame=1,
                strike_deadline_monotonic_s=0.5,
                completed_monotonic_s=0.01,
                command_fields=_command_fields(command),
                error_type=None,
                error_text=None,
            ),
        )

        first = pipeline.tick(
            lifecycle_now_s=0.0,
            obs_now_s=0.1,
            wall_time_us=1,
        )
        second = pipeline.tick(
            lifecycle_now_s=0.02,
            obs_now_s=0.12,
            wall_time_us=2,
        )

        self.assertEqual(first.lifecycle_decision, "skipped")
        self.assertEqual(second.lifecycle_decision, "none")
        late = [draft for draft in self._attempt_stage_drafts(events) if draft.payload["stage"] == "LATE_SKIP"]
        self.assertEqual(len(late), 1)
        self.assertEqual(late[0].payload["reason_code"], "LATE_SKIP")
        self.assertEqual(late[0].monotonic_s, 0.0)

    def test_tick_records_armed_then_task_failure_once(self) -> None:
        pipeline, worker, _adapter, _raw, events = self._pipeline(required=1)
        pipeline.ingest_adapter_output(_output(1, received_s=0.0))
        clipped_velocity = np.array(
            [400.0, 500.0, 600.0],
            dtype=np.float64,
        )
        base_command = _command(time_to_strike=0.85)
        command = replace(
            base_command,
            v_racket_target_w=clipped_velocity,
            strike_plan=replace(
                base_command.strike_plan,
                v_racket_target=clipped_velocity,
            ),
        )
        worker.bundle = (
            PlannerResultSnapshot(
                track_id=1,
                source_generation=1,
                source_frame=1,
                strike_deadline_monotonic_s=0.85,
                completed_monotonic_s=0.01,
                command=command,
            ),
            FrozenPlannerResult(
                snapshot_key=SnapshotKey(1, 1),
                source_frame=1,
                strike_deadline_monotonic_s=0.85,
                completed_monotonic_s=0.01,
                command_fields=_command_fields(command),
                error_type=None,
                error_text=None,
            ),
        )

        first = pipeline.tick(
            lifecycle_now_s=0.0,
            obs_now_s=0.1,
            wall_time_us=1,
        )
        second = pipeline.tick(
            lifecycle_now_s=0.02,
            obs_now_s=0.12,
            wall_time_us=2,
        )

        self.assertEqual(first.lifecycle_decision, "armed")
        self.assertFalse(first.task_pass)
        self.assertEqual(first.errors, ("OBS_CLIPPED",))
        self.assertFalse(second.task_pass)
        relevant = [
            draft
            for draft in self._attempt_stage_drafts(events)
            if draft.payload["stage"] in ("ARMED", "TASK_OBS_FAILED")
        ]
        self.assertEqual(
            [draft.payload["stage"] for draft in relevant],
            ["ARMED", "TASK_OBS_FAILED"],
        )
        self.assertAlmostEqual(
            relevant[0].payload["values"]["arm_tts_s"],
            0.85,
        )
        self.assertEqual(
            relevant[1].payload["reason_code"],
            "OBS_CLIPPED",
        )
        self.assertEqual(
            relevant[1].payload["values"]["errors"],
            ("OBS_CLIPPED",),
        )
        self.assertEqual(relevant[1].monotonic_s, 0.1)

    def test_tick_arms_with_same_source_fields_and_latest_pelvis_then_resets_at_strike(self) -> None:
        pipeline, worker, adapter, _raw, events = self._pipeline(required=1)
        output = _output(1, received_s=0.0)
        pipeline.ingest_adapter_output(output)
        command = _command(time_to_strike=0.85)
        result = PlannerResultSnapshot(
            track_id=1,
            source_generation=1,
            source_frame=1,
            strike_deadline_monotonic_s=0.85,
            completed_monotonic_s=0.01,
            command=command,
        )
        frozen = FrozenPlannerResult(
            snapshot_key=SnapshotKey(1, 1),
            source_frame=1,
            strike_deadline_monotonic_s=0.85,
            completed_monotonic_s=0.01,
            command_fields=_command_fields(command),
            error_type=None,
            error_text=None,
        )
        worker.bundle = (result, frozen)

        armed = pipeline.tick(
            lifecycle_now_s=0.0,
            obs_now_s=0.10,
            wall_time_us=100,
        )
        self.assertEqual(armed.phase, "armed")
        self.assertEqual(armed.lifecycle_decision, "armed")
        self.assertTrue(armed.task_pass)
        self.assertEqual(
            armed.active_binding.snapshot_key,
            SnapshotKey(1, 1),
        )
        self.assertEqual(armed.command_result.snapshot_key, frozen.snapshot_key)
        self.assertEqual(armed.command_fields_used, frozen.command_fields)
        np.testing.assert_allclose(
            armed.task_observation.pre_clip[:2],
            [0.0, 1.0],
            atol=1.0e-6,
        )
        np.testing.assert_array_equal(
            armed.task_observation.pre_clip[7:10],
            [4.0, 5.0, 6.0],
        )
        self.assertAlmostEqual(
            float(armed.task_observation.pre_clip[-1]),
            0.75,
        )
        still_armed = pipeline.tick(
            lifecycle_now_s=0.20,
            obs_now_s=0.21,
            wall_time_us=100,
        )
        self.assertTrue(still_armed.task_pass)
        self.assertEqual(
            sum(
                draft.kind == "attempt_transition" and draft.payload["stage"] == "TASK_OBS_PASS"
                for draft in events.drafts
            ),
            1,
        )
        self.assertEqual(
            sum(draft.kind == "attempt_transition" and draft.payload["stage"] == "ARMED" for draft in events.drafts),
            1,
        )

        recovery = pipeline.tick(
            lifecycle_now_s=0.851,
            obs_now_s=0.852,
            wall_time_us=101,
        )
        self.assertEqual(recovery.phase, "recovery")
        self.assertFalse(recovery.task_pass)
        self.assertEqual(adapter.reset_calls, 1)
        reset_diagnostics = self._ball_diagnostics(pipeline).snapshot()
        self.assertEqual(reset_diagnostics.status, "TRACK_ENDED")
        self.assertIsNone(reset_diagnostics.estimator_sample_count)
        self.assertIsNone(reset_diagnostics.incoming_count)
        self.assertEqual(
            reset_diagnostics.incoming_status,
            "TRACK_ENDED",
        )
        self.assertEqual(
            reset_diagnostics.blocker,
            "STRIKE_DEADLINE_RESET",
        )
        self.assertTrue(any(draft.kind == "lifecycle_tick" for draft in events.drafts))

    def test_bundle_identity_mismatch_never_arms_or_passes(self) -> None:
        pipeline, worker, _adapter_value, _raw, _events = self._pipeline(required=1)
        pipeline.ingest_adapter_output(_output(1, received_s=0.0))
        worker.bundle = (
            PlannerResultSnapshot(
                track_id=1,
                source_generation=1,
                source_frame=1,
                strike_deadline_monotonic_s=0.85,
                completed_monotonic_s=0.01,
                command=_command(),
            ),
            FrozenPlannerResult(
                snapshot_key=SnapshotKey(1, 99),
                source_frame=1,
                strike_deadline_monotonic_s=0.85,
                completed_monotonic_s=0.01,
                command_fields={"time_to_strike": 0.85},
                error_type=None,
                error_text=None,
            ),
        )
        tick = pipeline.tick(
            lifecycle_now_s=0.0,
            obs_now_s=0.1,
            wall_time_us=1,
        )
        self.assertFalse(tick.task_pass)
        self.assertIn("OBS_COMMAND_MISMATCH", tick.errors)
        self.assertNotEqual(tick.phase, "armed")

    def test_bundle_requires_all_source_timing_and_command_facts(self) -> None:
        command = _command(time_to_strike=0.85)
        valid_result = PlannerResultSnapshot(
            track_id=1,
            source_generation=1,
            source_frame=1,
            strike_deadline_monotonic_s=0.85,
            completed_monotonic_s=0.01,
            command=command,
        )
        valid_frozen = FrozenPlannerResult(
            snapshot_key=SnapshotKey(1, 1),
            source_frame=1,
            strike_deadline_monotonic_s=0.85,
            completed_monotonic_s=0.01,
            command_fields=_command_fields(command),
            error_type=None,
            error_text=None,
        )
        changed_fields = _command_fields(command)
        changed_fields["p_base_target_xy"] = (-0.399998, 0.2)
        cases = (
            replace(valid_frozen, source_frame=2),
            replace(
                valid_frozen,
                strike_deadline_monotonic_s=0.850002,
            ),
            replace(
                valid_frozen,
                completed_monotonic_s=0.010002,
            ),
            replace(valid_frozen, command_fields=changed_fields),
        )
        for frozen in cases:
            with self.subTest(frozen=frozen):
                pipeline, worker, _adapter, _raw, _events = self._pipeline(required=1)
                pipeline.ingest_adapter_output(_output(1, received_s=0.0))
                worker.bundle = (valid_result, frozen)
                tick = pipeline.tick(
                    lifecycle_now_s=0.0,
                    obs_now_s=0.1,
                    wall_time_us=1,
                )
                self.assertFalse(tick.task_pass)
                self.assertIn("OBS_COMMAND_MISMATCH", tick.errors)
                self.assertNotEqual(tick.phase, "armed")

    def test_mismatched_bundle_does_not_poison_corrected_same_key(self) -> None:
        pipeline, worker, _adapter, _raw, _events = self._pipeline(required=1)
        pipeline.ingest_adapter_output(_output(1, received_s=0.0))
        command = _command(time_to_strike=0.85)
        result = PlannerResultSnapshot(
            track_id=1,
            source_generation=1,
            source_frame=1,
            strike_deadline_monotonic_s=0.85,
            completed_monotonic_s=0.01,
            command=command,
        )
        valid_frozen = FrozenPlannerResult(
            snapshot_key=SnapshotKey(1, 1),
            source_frame=1,
            strike_deadline_monotonic_s=0.85,
            completed_monotonic_s=0.01,
            command_fields=_command_fields(command),
            error_type=None,
            error_text=None,
        )
        worker.bundle = (
            result,
            replace(valid_frozen, source_frame=99),
        )
        mismatch = pipeline.tick(
            lifecycle_now_s=0.0,
            obs_now_s=0.1,
            wall_time_us=1,
        )
        self.assertIn("OBS_COMMAND_MISMATCH", mismatch.errors)

        worker.bundle = (result, valid_frozen)
        corrected = pipeline.tick(
            lifecycle_now_s=0.0,
            obs_now_s=0.1,
            wall_time_us=2,
        )
        self.assertEqual(corrected.lifecycle_decision, "armed")
        self.assertTrue(corrected.task_pass)

    def test_bundle_rejects_any_float_fact_divergence(self) -> None:
        pipeline, worker, _adapter, _raw, _events = self._pipeline(required=1)
        pipeline.ingest_adapter_output(_output(1, received_s=0.0))
        command = _command(time_to_strike=0.85)
        command_fields = _command_fields(command)
        command_fields["p_base_target_xy"] = (-0.3999995, 0.2)
        worker.bundle = (
            PlannerResultSnapshot(
                track_id=1,
                source_generation=1,
                source_frame=1,
                strike_deadline_monotonic_s=0.85,
                completed_monotonic_s=0.01,
                command=command,
            ),
            FrozenPlannerResult(
                snapshot_key=SnapshotKey(1, 1),
                source_frame=1,
                strike_deadline_monotonic_s=0.8500005,
                completed_monotonic_s=0.0100005,
                command_fields=command_fields,
                error_type=None,
                error_text=None,
            ),
        )
        tick = pipeline.tick(
            lifecycle_now_s=0.0,
            obs_now_s=0.1,
            wall_time_us=1,
        )
        self.assertIn("OBS_COMMAND_MISMATCH", tick.errors)
        self.assertFalse(tick.task_pass)

    def test_bundle_requires_exact_success_and_failure_identity(self) -> None:
        command = _command(time_to_strike=0.85)
        success = PlannerResultSnapshot(
            track_id=1,
            source_generation=1,
            source_frame=1,
            strike_deadline_monotonic_s=0.85,
            completed_monotonic_s=0.01,
            command=command,
        )
        success_frozen = FrozenPlannerResult(
            snapshot_key=SnapshotKey(1, 1),
            source_frame=1,
            strike_deadline_monotonic_s=0.85,
            completed_monotonic_s=0.01,
            command_fields=_command_fields(command),
            error_type=None,
            error_text=None,
        )
        failure = PlannerResultSnapshot(
            track_id=1,
            source_generation=1,
            source_frame=1,
            strike_deadline_monotonic_s=float("nan"),
            completed_monotonic_s=0.01,
            command=None,
            failure_reason=PlannerFailureReason.INTERNAL_ERROR,
            error_text="ValueError: scripted failure",
        )
        failure_frozen = FrozenPlannerResult(
            snapshot_key=SnapshotKey(1, 1),
            source_frame=1,
            strike_deadline_monotonic_s=float("nan"),
            completed_monotonic_s=0.01,
            command_fields=None,
            error_type="ValueError",
            error_text="scripted failure",
            failure_reason=PlannerFailureReason.INTERNAL_ERROR,
        )

        self.assertTrue(
            ShadowTaskPipeline._result_bundle_matches(
                success,
                success_frozen,
            )
        )
        self.assertTrue(
            ShadowTaskPipeline._result_bundle_matches(
                failure,
                failure_frozen,
            )
        )
        self.assertFalse(
            ShadowTaskPipeline._result_bundle_matches(
                success,
                replace(
                    success_frozen,
                    error_type="ValueError",
                    error_text="different failure",
                ),
            )
        )
        self.assertFalse(
            ShadowTaskPipeline._result_bundle_matches(
                failure,
                replace(
                    failure_frozen,
                    error_type=None,
                    error_text=None,
                    failure_reason=None,
                ),
            )
        )
        self.assertFalse(
            ShadowTaskPipeline._result_bundle_matches(
                failure,
                replace(
                    failure_frozen,
                    failure_reason=PlannerFailureReason.TRACK_ENDED,
                ),
            )
        )

    def test_bundle_rejects_numeric_string_and_source_frame_coercion(self) -> None:
        command = _command(time_to_strike=0.85)
        result = PlannerResultSnapshot(
            track_id=1,
            source_generation=1,
            source_frame=1,
            strike_deadline_monotonic_s=0.85,
            completed_monotonic_s=0.01,
            command=command,
        )
        fields = _command_fields(command)
        fields["time_to_strike"] = "0.85"
        fields["p_base_target_xy"] = ("-0.4", 0.2)
        frozen = FrozenPlannerResult(
            snapshot_key=SnapshotKey(1, 1),
            source_frame=1.9,
            strike_deadline_monotonic_s="0.85",
            completed_monotonic_s="0.01",
            command_fields=fields,
            error_type=None,
            error_text=None,
        )

        self.assertFalse(ShadowTaskPipeline._result_bundle_matches(result, frozen))

    def test_failure_bundle_rejects_mutated_reason_type_or_detail(self) -> None:
        result = PlannerResultSnapshot(
            track_id=1,
            source_generation=1,
            source_frame=1,
            strike_deadline_monotonic_s=float("nan"),
            completed_monotonic_s=0.01,
            command=None,
            failure_reason=PlannerFailureReason.INTERNAL_ERROR,
            error_text="ValueError: expected",
        )
        frozen = FrozenPlannerResult(
            snapshot_key=SnapshotKey(1, 1),
            source_frame=1,
            strike_deadline_monotonic_s=float("nan"),
            completed_monotonic_s=0.01,
            command_fields=None,
            error_type="ValueError",
            error_text="expected",
            failure_reason=PlannerFailureReason.INTERNAL_ERROR,
        )
        mutations = {
            "reason": {
                "failure_reason": PlannerFailureReason.TRACK_ENDED,
            },
            "type": {"error_type": "LookupError"},
            "detail": {"error_text": "different"},
        }

        self.assertTrue(ShadowTaskPipeline._result_bundle_matches(result, frozen))
        for label, changes in mutations.items():
            with self.subTest(label=label):
                self.assertFalse(
                    ShadowTaskPipeline._result_bundle_matches(
                        result,
                        replace(frozen, **changes),
                    )
                )

    def test_planner_replacement_is_also_scoped_to_replaced_attempt(self) -> None:
        pipeline, _worker, _adapter, _raw, events = self._pipeline()
        old_snapshot = _snapshot(1, received_s=0.0, track_id=1)
        pipeline.ingest_adapter_output(_output(1, received_s=0.0, track_id=1))
        pipeline.ingest_adapter_output(
            _output(
                2,
                received_s=0.01,
                visible=False,
                track_id=1,
            )
        )
        pipeline.attempt_tracker.advance(now_monotonic_s=0.3)
        new_snapshot = _snapshot(3, received_s=0.31, track_id=2)
        pipeline.ingest_adapter_output(_output(3, received_s=0.31, track_id=2))
        events.drafts.clear()

        pipeline._on_worker_trace(
            PlannerWorkerTrace(
                trace_seq=10,
                kind="pending_replaced",
                monotonic_s=0.31,
                snapshot=new_snapshot,
                result=None,
                replaced_snapshot=old_snapshot,
                replaced_result=None,
                stats=PlannerWorkerStats(2, 0, 0, 1),
            )
        )

        primary = next(draft for draft in events.drafts if draft.kind == "planner_trace")
        replaced = next(draft for draft in events.drafts if draft.kind == "planner_trace_replaced")
        self.assertEqual(primary.attempt_id, 2)
        self.assertEqual(primary.payload["replaced_attempt_id"], 1)
        self.assertEqual(replaced.attempt_id, 1)
        self.assertEqual(replaced.payload["replacing_attempt_id"], 2)

    def test_completed_result_batch_ingests_each_result_in_order(self) -> None:
        pipeline, worker, _adapter, _raw, events = self._pipeline(required=1)
        pipeline.ingest_adapter_output(_output(1, received_s=0.0, track_id=1))
        first_command = _command(time_to_strike=2.0)
        second_command = _command(time_to_strike=2.1)
        first = PlannerResultSnapshot(
            track_id=1,
            source_generation=1,
            source_frame=1,
            strike_deadline_monotonic_s=2.0,
            completed_monotonic_s=0.01,
            command=first_command,
        )
        second = PlannerResultSnapshot(
            track_id=1,
            source_generation=2,
            source_frame=2,
            strike_deadline_monotonic_s=2.1,
            completed_monotonic_s=0.02,
            command=second_command,
        )
        worker.batch = CompletedResultBatch(
            results=(first, second),
            frozen_results=(
                FrozenPlannerResult(
                    snapshot_key=SnapshotKey(1, 1),
                    source_frame=1,
                    strike_deadline_monotonic_s=2.0,
                    completed_monotonic_s=0.01,
                    command_fields=_command_fields(first_command),
                    error_type=None,
                    error_text=None,
                ),
                FrozenPlannerResult(
                    snapshot_key=SnapshotKey(1, 2),
                    source_frame=2,
                    strike_deadline_monotonic_s=2.1,
                    completed_monotonic_s=0.02,
                    command_fields=_command_fields(second_command),
                    error_type=None,
                    error_text=None,
                ),
            ),
            overflowed=False,
            overflow_count=0,
            overflowed_track_ids=(),
        )

        tick = pipeline.tick(
            lifecycle_now_s=0.0,
            obs_now_s=0.1,
            wall_time_us=1,
        )

        self.assertEqual(
            [decision.kind for decision in tick.result_decisions],
            ["tracking", "tracking"],
        )
        ingested = [draft for draft in events.drafts if draft.kind == "planner_result_ingested"]
        self.assertEqual(
            [draft.payload["generation"] for draft in ingested],
            [1, 2],
        )
        self.assertEqual(tick.completed_result_queue_depth, 2)

    def test_completed_result_batch_none_frozen_result_is_ingested(self) -> None:
        pipeline, worker, _adapter, _raw, _events = self._pipeline(required=1)
        pipeline.ingest_adapter_output(_output(1, received_s=0.0, track_id=1))
        command = _command(time_to_strike=2.0)
        worker.batch = CompletedResultBatch(
            results=(
                PlannerResultSnapshot(
                    track_id=1,
                    source_generation=1,
                    source_frame=1,
                    strike_deadline_monotonic_s=2.0,
                    completed_monotonic_s=0.01,
                    command=command,
                ),
            ),
            frozen_results=(None,),
            overflowed=False,
            overflow_count=0,
            overflowed_track_ids=(),
        )

        tick = pipeline.tick(
            lifecycle_now_s=0.0,
            obs_now_s=0.1,
            wall_time_us=1,
        )

        self.assertEqual(tick.result_decisions[0].kind, "tracking")
        self.assertEqual(len(tick.frozen_results), 1)
        self.assertEqual(
            tick.frozen_results[0].snapshot_key,
            SnapshotKey(1, 1),
        )

    def test_result_queue_overflow_consumes_all_compromised_ids_without_ingest_events(self) -> None:
        pipeline, worker, _adapter, _raw, events = self._pipeline(required=1)
        pipeline.ingest_adapter_output(_output(1, received_s=0.0, track_id=1))
        command = _command(time_to_strike=2.0)
        worker.batch = CompletedResultBatch(
            results=(
                PlannerResultSnapshot(
                    track_id=1,
                    source_generation=1,
                    source_frame=1,
                    strike_deadline_monotonic_s=2.0,
                    completed_monotonic_s=0.01,
                    command=command,
                ),
            ),
            frozen_results=(None,),
            overflowed=True,
            overflow_count=3,
            overflowed_track_ids=(2, 3),
        )

        tick = pipeline.tick(
            lifecycle_now_s=0.0,
            obs_now_s=0.1,
            wall_time_us=1,
        )

        self.assertEqual(tick.lifecycle_decision, "cancelled")
        self.assertEqual(tick.overflowed_track_ids, (2, 3))
        self.assertEqual(tick.compromised_track_ids, (1, 2, 3))
        self.assertEqual(tick.completed_result_queue_depth, 1)
        self.assertEqual(tick.completed_result_queue_overflow_count, 3)
        self.assertTrue({1, 2, 3} <= pipeline.lifecycle.consumed_track_ids)
        self.assertFalse(any(draft.kind == "planner_result_ingested" for draft in events.drafts))
        overflow_events = [draft for draft in events.drafts if draft.kind == "planner_result_queue_overflow"]
        self.assertEqual(len(overflow_events), 1)
        self.assertEqual(
            overflow_events[0].payload["compromised_track_ids"],
            (1, 2, 3),
        )

    def test_lifecycle_tracking_tick_uses_latest_result_ownership(self) -> None:
        pipeline, worker, _adapter, _raw, events = self._pipeline(required=1)
        pipeline.ingest_adapter_output(_output(1, received_s=0.0))
        command = _command(time_to_strike=2.0)
        worker.bundle = (
            PlannerResultSnapshot(
                track_id=1,
                source_generation=1,
                source_frame=1,
                strike_deadline_monotonic_s=2.0,
                completed_monotonic_s=0.01,
                command=command,
            ),
            FrozenPlannerResult(
                snapshot_key=SnapshotKey(1, 1),
                source_frame=1,
                strike_deadline_monotonic_s=2.0,
                completed_monotonic_s=0.01,
                command_fields=_command_fields(command),
                error_type=None,
                error_text=None,
            ),
        )
        tick = pipeline.tick(
            lifecycle_now_s=0.0,
            obs_now_s=0.1,
            wall_time_us=1,
        )
        self.assertEqual(tick.phase, "tracking")
        lifecycle_event = [draft for draft in events.drafts if draft.kind == "lifecycle_tick"][-1]
        self.assertEqual(lifecycle_event.scope, "attempt")
        self.assertEqual(lifecycle_event.attempt_id, 1)

    def test_recovery_result_consumption_uses_next_attempt_ownership(self) -> None:
        pipeline, worker, _adapter, raw, events = self._pipeline(required=1)
        pipeline.ingest_adapter_output(_output(1, received_s=0.0))
        first_command = _command(time_to_strike=0.85)
        worker.bundle = (
            PlannerResultSnapshot(
                track_id=1,
                source_generation=1,
                source_frame=1,
                strike_deadline_monotonic_s=0.85,
                completed_monotonic_s=0.01,
                command=first_command,
            ),
            FrozenPlannerResult(
                snapshot_key=SnapshotKey(1, 1),
                source_frame=1,
                strike_deadline_monotonic_s=0.85,
                completed_monotonic_s=0.01,
                command_fields=_command_fields(first_command),
                error_type=None,
                error_text=None,
            ),
        )
        pipeline.tick(
            lifecycle_now_s=0.0,
            obs_now_s=0.1,
            wall_time_us=1,
        )
        pipeline.tick(
            lifecycle_now_s=0.851,
            obs_now_s=0.852,
            wall_time_us=2,
        )
        pipeline.attempt_tracker.advance(now_monotonic_s=1.1)
        pipeline.ingest_adapter_output(_output(2, received_s=1.2, track_id=2))
        self.assertEqual(raw.items[-1][1].attempt_id, 2)
        second_command = _command(time_to_strike=0.8)
        worker.bundle = (
            PlannerResultSnapshot(
                track_id=2,
                source_generation=2,
                source_frame=2,
                strike_deadline_monotonic_s=2.0,
                completed_monotonic_s=1.21,
                command=second_command,
            ),
            FrozenPlannerResult(
                snapshot_key=SnapshotKey(2, 2),
                source_frame=2,
                strike_deadline_monotonic_s=2.0,
                completed_monotonic_s=1.21,
                command_fields=_command_fields(second_command),
                error_type=None,
                error_text=None,
            ),
        )
        cached = pipeline.tick(
            lifecycle_now_s=1.2,
            obs_now_s=1.21,
            wall_time_us=3,
        )
        self.assertEqual(cached.lifecycle_decision, "consumed_during_recovery")
        lifecycle_event = [draft for draft in events.drafts if draft.kind == "lifecycle_tick"][-1]
        self.assertEqual(lifecycle_event.attempt_id, 2)

    def test_recovery_result_is_consumed_and_does_not_rearm_after_recovery(self) -> None:
        pipeline, worker, _adapter_value, _raw, _events = self._pipeline(required=1)
        pipeline.ingest_adapter_output(_output(1, received_s=0.0))
        first_command = _command(time_to_strike=0.85)
        worker.bundle = (
            PlannerResultSnapshot(
                track_id=1,
                source_generation=1,
                source_frame=1,
                strike_deadline_monotonic_s=0.85,
                completed_monotonic_s=0.01,
                command=first_command,
            ),
            FrozenPlannerResult(
                snapshot_key=SnapshotKey(1, 1),
                source_frame=1,
                strike_deadline_monotonic_s=0.85,
                completed_monotonic_s=0.01,
                command_fields=_command_fields(first_command),
                error_type=None,
                error_text=None,
            ),
        )
        self.assertEqual(
            pipeline.tick(
                lifecycle_now_s=0.0,
                obs_now_s=0.1,
                wall_time_us=1,
            ).phase,
            "armed",
        )
        pipeline.tick(
            lifecycle_now_s=0.851,
            obs_now_s=0.852,
            wall_time_us=2,
        )
        self.assertEqual(pipeline.lifecycle.phase.value, "recovery")
        recovery_end = float(pipeline.lifecycle.command_end_deadline_s)

        pipeline.ingest_adapter_output(_output(2, received_s=0.9, track_id=2))
        second_deadline = recovery_end + 0.85
        second_command = _command(time_to_strike=second_deadline - 0.9)
        worker.bundle = (
            PlannerResultSnapshot(
                track_id=2,
                source_generation=2,
                source_frame=2,
                strike_deadline_monotonic_s=second_deadline,
                completed_monotonic_s=0.91,
                command=second_command,
            ),
            FrozenPlannerResult(
                snapshot_key=SnapshotKey(2, 2),
                source_frame=2,
                strike_deadline_monotonic_s=second_deadline,
                completed_monotonic_s=0.91,
                command_fields=_command_fields(second_command),
                error_type=None,
                error_text=None,
            ),
        )
        cached = pipeline.tick(
            lifecycle_now_s=0.9,
            obs_now_s=0.91,
            wall_time_us=3,
        )
        self.assertEqual(cached.lifecycle_decision, "consumed_during_recovery")
        self.assertEqual(cached.phase, "recovery")

        rearmed = pipeline.tick(
            lifecycle_now_s=recovery_end,
            obs_now_s=recovery_end + 0.01,
            wall_time_us=4,
        )
        self.assertEqual(rearmed.phase, "waiting")
        self.assertEqual(rearmed.lifecycle_decision, "entered_waiting")
        self.assertIsNone(rearmed.active_binding)

    def test_sink_exceptions_are_contained_on_ingest_trace_and_tick(self) -> None:
        class BrokenSink:
            def offer(self, _draft):
                raise RuntimeError("full")

        class BrokenRaw:
            def __call__(self, *_args):
                raise RuntimeError("disk")

        worker = _FakeWorker()
        pipeline = ShadowTaskPipeline(
            adapter=_FakeAdapter(),
            settings=_settings(required=1),
            event_sink=BrokenSink(),
            raw_sink=BrokenRaw(),
            worker=worker,
            planner=None,
        )
        pipeline.ingest_adapter_output(_output(1, received_s=0.0))
        pipeline.tick(
            lifecycle_now_s=0.0,
            obs_now_s=0.1,
            wall_time_us=1,
        )


if __name__ == "__main__":
    unittest.main()
