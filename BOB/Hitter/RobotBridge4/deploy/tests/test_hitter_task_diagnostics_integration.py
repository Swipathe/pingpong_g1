from __future__ import annotations

from dataclasses import dataclass
from types import SimpleNamespace
import unittest

import numpy as np

from diagnostics.hitter_task_models import EventDraft, SnapshotKey
from diagnostics.hitter_task_pipeline import MocapFrameAdapter, ShadowTaskPipeline
from diagnostics.hitter_task_replay import (
    BASELINE_VARIANT,
    RecordedPlannerDurationProvider,
    ReplayCaptureStore,
    ReplayEventCaptureTee,
    ReplayRawCaptureTee,
    compare_baseline_to_online,
    replay_attempt,
)
from utils.hitter_realtime import (
    FrozenPlannerResult,
    PlannerResultSnapshot,
    PlannerWorkerStats,
    PlannerWorkerTrace,
    freeze_planner_command_fields,
)
from utils.hitter_runtime_factory import HitterRuntimeSettings
from utils.hitter_runtime_types import PlannerFailureReason


@dataclass(frozen=True)
class _StrikePlan:
    t_strike: float
    p_racket_target: np.ndarray
    v_racket_target: np.ndarray


@dataclass(frozen=True)
class _Command:
    strike_type: str
    p_base_target_xy: np.ndarray
    v_racket_target_w: np.ndarray
    time_to_strike: float
    strike_plan: _StrikePlan


class _ScriptedPlanner:
    def __init__(self) -> None:
        self.calls = []

    def plan_command(
        self,
        ball_position,
        ball_velocity,
        *,
        current_base_xy_w,
        base_forward_xy_w,
        strike_type,
    ):
        self.calls.append(
            (
                np.array(ball_position, copy=True),
                np.array(ball_velocity, copy=True),
                np.array(current_base_xy_w, copy=True),
                np.array(base_forward_xy_w, copy=True),
                strike_type,
            )
        )
        return _Command(
            strike_type="forehand",
            p_base_target_xy=np.array([-0.4, 0.2], dtype=np.float64),
            v_racket_target_w=np.array([4.0, 5.0, 6.0], dtype=np.float64),
            time_to_strike=0.85,
            strike_plan=_StrikePlan(
                t_strike=0.85,
                p_racket_target=np.array(
                    [0.1, 0.2, 1.1],
                    dtype=np.float64,
                ),
                v_racket_target=np.array(
                    [4.0, 5.0, 6.0],
                    dtype=np.float64,
                ),
            ),
        )


class _ImmediateWorker:
    """Deterministic worker facade exercising the production plan callback."""

    def __init__(self) -> None:
        self.pipeline = None
        self.bundle = (None, None)
        self.submitted = 0
        self.completed = 0
        self.failed = 0
        self.trace_seq = 0

    @property
    def stats(self):
        return PlannerWorkerStats(
            submitted=self.submitted,
            completed=self.completed,
            failed=self.failed,
            dropped_pending=0,
        )

    def attach(self, pipeline: ShadowTaskPipeline) -> None:
        self.pipeline = pipeline

    def submit(self, snapshot) -> None:
        if self.pipeline is None:
            raise RuntimeError("worker is not attached")
        self.submitted += 1
        self.trace_seq += 1
        self.pipeline._on_worker_trace(
            PlannerWorkerTrace(
                trace_seq=self.trace_seq,
                kind="start",
                monotonic_s=float(snapshot.received_monotonic_s),
                snapshot=snapshot,
                result=None,
                replaced_snapshot=None,
                replaced_result=None,
                stats=self.stats,
            )
        )
        completed_s = float(snapshot.received_monotonic_s) + 0.0001
        error_type = None
        error_text = None
        try:
            command = self.pipeline.plan_snapshot(snapshot)
            deadline_s = (
                float(snapshot.received_monotonic_s)
                + float(command.time_to_strike)
            )
            result = PlannerResultSnapshot(
                track_id=snapshot.track_id,
                source_generation=snapshot.generation,
                source_frame=snapshot.source_frame,
                strike_deadline_monotonic_s=deadline_s,
                completed_monotonic_s=completed_s,
                command=command,
            )
            command_fields = freeze_planner_command_fields(command)
            self.completed += 1
        except Exception as exc:
            error_type = type(exc).__name__
            error_text = str(exc)
            result = PlannerResultSnapshot(
                track_id=snapshot.track_id,
                source_generation=snapshot.generation,
                source_frame=snapshot.source_frame,
                strike_deadline_monotonic_s=float("nan"),
                completed_monotonic_s=completed_s,
                command=None,
                failure_reason=PlannerFailureReason.INTERNAL_ERROR,
                error_text="{}: {}".format(error_type, error_text),
            )
            deadline_s = float("nan")
            command_fields = None
            self.failed += 1
        frozen = FrozenPlannerResult(
            snapshot_key=SnapshotKey(
                int(snapshot.track_id),
                int(snapshot.generation),
            ),
            source_frame=int(snapshot.source_frame),
            strike_deadline_monotonic_s=deadline_s,
            completed_monotonic_s=completed_s,
            command_fields=command_fields,
            error_type=error_type,
            error_text=error_text,
            failure_reason=result.failure_reason,
        )
        previous_frozen = self.bundle[1]
        self.bundle = (result, frozen)
        self.trace_seq += 1
        self.pipeline._on_worker_trace(
            PlannerWorkerTrace(
                trace_seq=self.trace_seq,
                kind="complete",
                monotonic_s=completed_s,
                snapshot=snapshot,
                result=frozen,
                replaced_snapshot=None,
                replaced_result=None,
                stats=self.stats,
            )
        )
        if previous_frozen is not None:
            self.trace_seq += 1
            self.pipeline._on_worker_trace(
                PlannerWorkerTrace(
                    trace_seq=self.trace_seq,
                    kind="latest_result_replaced",
                    monotonic_s=completed_s,
                    snapshot=snapshot,
                    result=frozen,
                    replaced_snapshot=None,
                    replaced_result=previous_frozen,
                    stats=self.stats,
                )
            )

    def latest_result_bundle(self):
        return self.bundle

    def close(self, timeout_s=None) -> bool:
        del timeout_s
        return True


class _EventSink:
    def __init__(self) -> None:
        self.drafts = []

    def offer(self, draft) -> bool:
        self.drafts.append(draft)
        return True


class _RawSink:
    def __init__(self) -> None:
        self.items = []

    def submit(self, sample, *, attempt_id=None, track_segment_id=None):
        self.items.append((sample, attempt_id, track_segment_id))
        return True


def _message(
    name: str,
    *,
    frame: int,
    source_time_s: float,
    position,
    valid: bool = True,
):
    return SimpleNamespace(
        name=name,
        vicon_frame_number=frame,
        vicon_time_s=source_time_s,
        publish_time_us=int(source_time_s * 1.0e6),
        valid=int(valid),
        occluded=int(not valid),
        pos_vicon=list(position),
        quat_vicon=[0.0, 0.0, 0.0, 1.0],
    )


def _ingest(
    adapter: MocapFrameAdapter,
    pipeline: ShadowTaskPipeline,
    message,
    *,
    received_s: float,
):
    output = adapter.ingest_decoded(
        channel="vicon_state_data",
        message=message,
        payload_size=96,
        received_monotonic_s=received_s,
        wall_time_us=1_700_000_000_000_000 + int(received_s * 1.0e6),
    )
    pipeline.ingest_adapter_output(output)
    return output


def _settings() -> HitterRuntimeSettings:
    return HitterRuntimeSettings(
        estimator_sample_rate_hz=360.0,
        planner_update_rate_hz=100.0,
        planner_update_interval_s=0.01,
        minimum_incoming_speed_x_mps=0.2,
        incoming_confirmation_snapshots=3,
        waiting_tts_s=0.92,
        arm_tts_s=0.90,
        minimum_arm_tts_s=0.60,
        maximum_policy_tts_s=0.92,
        swing_duration_range_s=(1.75, 1.95),
        hitter_seed=0,
        control_tick_s=0.02,
        obs_clip_value=100.0,
    )


class HitterTaskDiagnosticsIntegrationTest(unittest.TestCase):
    def test_capture_keeps_post_ball_pelvis_and_baseline_parity(self) -> None:
        planner_config = {
            "state_estimator_window_size": 31,
            "state_estimator_min_samples": 31,
            "state_estimator_sample_rate_hz": 360.0,
        }
        settings = _settings()
        adapter = MocapFrameAdapter(
            planner_config=planner_config,
            estimator_sample_rate_hz=360.0,
        )
        planner = _ScriptedPlanner()
        worker = _ImmediateWorker()
        captured_events = _EventSink()
        captured_raw = _RawSink()
        capture = ReplayCaptureStore(
            input_capacity=256,
            event_capacity=2048,
            attempt_capacity=4,
        )
        events = ReplayEventCaptureTee(captured_events, capture)
        raw = ReplayRawCaptureTee(captured_raw, capture)
        pipeline = ShadowTaskPipeline(
            adapter=adapter,
            settings=settings,
            planner=planner,
            worker=worker,
            event_sink=events,
            raw_sink=raw,
        )
        worker.attach(pipeline)

        _ingest(
            adapter,
            pipeline,
            _message(
                "G2Pelvis",
                frame=1,
                source_time_s=1.0,
                position=(0.0, 0.0, 0.8),
            ),
            received_s=10.0,
        )
        for index in range(41):
            source_time_s = 1.0 + index / 360.0
            received_s = 10.0 + index / 360.0
            _ingest(
                adapter,
                pipeline,
                _message(
                    "ball",
                    frame=index + 1,
                    source_time_s=source_time_s,
                    position=(1.8 - 0.01 * index, 0.0, 1.2),
                ),
                received_s=received_s,
            )
        final_ball_s = 10.0 + 40.0 / 360.0
        final_pelvis_output = _ingest(
            adapter,
            pipeline,
            _message(
                "G2Pelvis",
                frame=42,
                source_time_s=1.115,
                position=(0.02, 0.0, 0.8),
            ),
            received_s=final_ball_s + 0.0005,
        )
        pipeline.tick(
            lifecycle_now_s=final_ball_s + 0.001,
            obs_now_s=final_ball_s + 0.002,
            wall_time_us=1_700_000_000_200_000,
        )
        invalid_s = final_ball_s + 0.01
        _ingest(
            adapter,
            pipeline,
            _message(
                "ball",
                frame=43,
                source_time_s=1.12,
                position=(1.39, 0.0, 1.2),
                valid=False,
            ),
            received_s=invalid_s,
        )
        close_s = invalid_s + 0.201
        pipeline.tick(
            lifecycle_now_s=close_s - 0.001,
            obs_now_s=close_s,
            wall_time_us=1_700_000_000_400_000,
        )
        transitions = pipeline.attempt_tracker.advance(
            now_monotonic_s=close_s
        )
        for transition in transitions:
            events.offer(
                EventDraft(
                    kind="attempt_transition",
                    monotonic_s=transition.monotonic_s,
                    wall_time_us=1_700_000_000_400_000,
                    scope="attempt",
                    attempt_id=transition.attempt_id,
                    payload={
                        "track_segment_id": transition.track_segment_id,
                        "stage": transition.stage,
                        "reason_code": transition.reason_code,
                        "snapshot_key": (
                            None
                            if transition.snapshot_key is None
                            else transition.snapshot_key.to_json_dict()
                        ),
                        "values": transition.values,
                    },
                )
            )
        summary = pipeline.attempt_tracker.summary_for_attempt(1)
        self.assertIsNotNone(summary)
        bundle = capture.finalize_attempt(
            attempt_id=1,
            terminal_code="TASK_OBS_PASS",
            recording_complete=True,
            planner_config=planner_config,
            runtime_settings=settings,
            forced_strike_type=None,
        )
        final_pelvis_seq = final_pelvis_output.sample.input_seq
        self.assertEqual(
            sum(
                sample.input_seq == final_pelvis_seq
                for sample in bundle.inputs
            ),
            1,
        )
        self.assertGreater(
            bundle.inputs[-2].received_monotonic_s,
            bundle.inputs[-3].received_monotonic_s,
        )

        replay = replay_attempt(
            bundle,
            variant=BASELINE_VARIANT,
            duration_provider=RecordedPlannerDurationProvider(
                bundle.online_baseline.planner_calls
            ),
            planner_factory=_ScriptedPlanner,
        )
        parity = compare_baseline_to_online(
            online=bundle.online_baseline,
            replay=replay,
        )
        self.assertTrue(
            parity.matches,
            (
                parity.mismatches,
                bundle.online_baseline.stages,
                replay.stages,
            ),
        )

    def test_360hz_estimator_to_100hz_planner_to_11_field_pass(self) -> None:
        adapter = MocapFrameAdapter(
            planner_config={
                "state_estimator_window_size": 31,
                "state_estimator_min_samples": 31,
                "state_estimator_sample_rate_hz": 360.0,
            },
            estimator_sample_rate_hz=360.0,
        )
        planner = _ScriptedPlanner()
        worker = _ImmediateWorker()
        events = _EventSink()
        raw = _RawSink()
        pipeline = ShadowTaskPipeline(
            adapter=adapter,
            settings=_settings(),
            planner=planner,
            worker=worker,
            event_sink=events,
            raw_sink=raw,
        )
        worker.attach(pipeline)

        _ingest(
            adapter,
            pipeline,
            _message(
                "G2Pelvis",
                frame=1,
                source_time_s=1.0,
                position=(0.0, 0.0, 0.8),
            ),
            received_s=10.0,
        )
        last_output = None
        for index in range(41):
            source_time_s = 1.0 + index / 360.0
            received_s = 10.0 + index / 360.0
            last_output = _ingest(
                adapter,
                pipeline,
                _message(
                    "ball",
                    frame=index + 1,
                    source_time_s=source_time_s,
                    position=(1.8 - 0.01 * index, 0.0, 1.2),
                ),
                received_s=received_s,
            )

        self.assertIsNotNone(last_output)
        self.assertTrue(last_output.snapshot.ready)
        self.assertEqual(last_output.estimator_sample_count, 31)
        self.assertTrue(pipeline.incoming.snapshot().confirmed)
        self.assertEqual(pipeline.incoming.snapshot().consecutive_count, 3)
        self.assertEqual(len(planner.calls), 1)
        self.assertEqual(len(raw.items), 42)

        final_received_s = 10.0 + 40.0 / 360.0
        tick = pipeline.tick(
            lifecycle_now_s=final_received_s + 0.001,
            obs_now_s=final_received_s + 0.002,
            wall_time_us=1_700_000_000_200_000,
        )
        self.assertEqual(tick.phase, "armed")
        self.assertEqual(tick.lifecycle_decision, "armed")
        self.assertTrue(tick.task_pass)
        self.assertEqual(tick.errors, ())
        self.assertEqual(tick.task_observation.pre_clip.shape, (11,))
        self.assertEqual(tick.task_observation.post_clip.shape, (11,))
        self.assertEqual(tick.task_observation.pre_clip.dtype, np.float32)
        self.assertTrue(np.isfinite(tick.task_observation.pre_clip).all())
        self.assertEqual(tick.task_observation.clip_count, 0)
        self.assertEqual(
            pipeline.attempt_tracker.current_summary().task_obs_status,
            "PASS",
        )

        complete_traces = [
            draft
            for draft in events.drafts
            if draft.kind == "planner_trace"
            and draft.payload["trace_kind"] == "complete"
        ]
        self.assertEqual(len(complete_traces), worker.submitted)
        self.assertEqual(
            complete_traces[-1].payload["result"]["reason_code"],
            None,
        )
        self.assertGreater(
            sum(
                draft.payload["result"]["reason_code"]
                == "ESTIMATOR_WARMING"
                for draft in complete_traces
            ),
            0,
        )
        self.assertEqual(
            sum(
                draft.payload["result"]["reason_code"]
                == "INCOMING_SPEED_REJECTED"
                for draft in complete_traces
            ),
            2,
        )

        invalid_received_s = final_received_s + 0.01
        _ingest(
            adapter,
            pipeline,
            _message(
                "ball",
                frame=42,
                source_time_s=1.0 + 41.0 / 360.0,
                position=(0.0, 0.0, 0.0),
                valid=False,
            ),
            received_s=invalid_received_s,
        )
        pipeline.attempt_tracker.advance(
            now_monotonic_s=invalid_received_s + 0.201,
        )
        terminal = pipeline.attempt_tracker.summary_for_attempt(1)
        self.assertEqual(terminal.status, "SUCCESS")
        self.assertIsNone(terminal.primary_blocker)
        self.assertTrue(pipeline.close(timeout_s=0.1))


if __name__ == "__main__":
    unittest.main()
