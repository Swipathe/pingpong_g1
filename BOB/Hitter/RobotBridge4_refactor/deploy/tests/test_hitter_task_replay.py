from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import stat
import tempfile
import unittest
from unittest import mock

import numpy as np

from diagnostics import hitter_task_replay as replay_module
from diagnostics.hitter_task_models import (
    AttemptTransition,
    EventDraft,
    NormalizedMocapSample,
    SnapshotKey,
)
from diagnostics.hitter_task_replay import (
    DeterministicEventScheduler,
    OnlineAttemptTrace,
    PlannerCallRecord,
    PolicyTickRecord,
    RecordedPlannerDurationProvider,
    ReplayInputBundle,
    ReplayCaptureStore,
    ReplayEventCaptureTee,
    ReplayOutcome,
    ReplayRawCaptureTee,
    ReplayJobRef,
    ReplayTrace,
    ReplayVariant,
    analyze_attempt_ab,
    compare_baseline_to_online,
    compare_replay_outcomes,
    load_replay_input_bundle,
    replay_attempt,
    replay_input_bundle_to_json_dict,
    run_replay_disk_job,
    write_replay_input_bundle,
)
from utils.hitter_realtime import FrozenPlannerResult


def _sample(
    seq: int,
    at_s: float,
    *,
    subject: str = "ball",
    x: float = 1.0,
    valid: bool = True,
) -> NormalizedMocapSample:
    return NormalizedMocapSample(
        input_seq=seq,
        channel="vicon_state_data",
        subject=subject,
        position_w=np.array([x, 0.0, 1.0]),
        quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
        valid=valid,
        occluded=not valid,
        source_frame=seq,
        source_time_s=at_s,
        publish_time_us=int(at_s * 1.0e6),
        received_monotonic_s=at_s,
        wall_time_us=int(at_s * 1.0e6),
        payload_size=80,
    )


def _tick(index: int = 0) -> PolicyTickRecord:
    return PolicyTickRecord(
        tick_index=index,
        lifecycle_now_s=1.0 + 0.02 * index,
        obs_now_s=1.001 + 0.02 * index,
        lifecycle_decision="none",
        phase="waiting",
        active_key=None,
        cached_key=None,
        command_fields=None,
        task_pre_clip=None,
        task_post_clip=None,
        task_clip_count=None,
    )


def _online(
    *,
    terminal_code: str = "TRACK_ENDED_BEFORE_READY",
    ticks=(),
) -> OnlineAttemptTrace:
    return OnlineAttemptTrace(
        attempt_id=1,
        stages=(),
        planner_calls=(),
        policy_ticks=tuple(ticks),
        terminal_code=terminal_code,
        recovery_duration_s=None,
        submission_phase_anchor_s=None,
        recording_complete=True,
    )


def _outcome(
    variant: str,
    code: str,
    *,
    passed: bool = False,
    boundary: bool = False,
    complete: bool = True,
    metrics=None,
) -> ReplayOutcome:
    return ReplayOutcome(
        variant=variant,
        terminal_code=code,
        task_obs_pass=passed,
        boundary_sensitive=boundary,
        recording_complete=complete,
        warnings=(),
        summary_label=None,
        metrics={} if metrics is None else metrics,
    )


class _Duration:
    def __init__(self, value: float = 0.002):
        self.value = value
        self.calls = []

    def duration_s(
        self,
        *,
        variant,
        snapshot_key,
        measured_offline_duration_s,
    ):
        self.calls.append((variant.name, snapshot_key, measured_offline_duration_s))
        return self.value


class _Planner:
    def plan_command(
        self,
        ball_position,
        ball_velocity,
        *,
        current_base_xy_w,
        base_forward_xy_w,
        strike_type,
    ):
        raise AssertionError("empty replay must not call planner")


@dataclass(frozen=True)
class _StrikePlan:
    p_racket_target: np.ndarray


@dataclass(frozen=True)
class _Command:
    strike_type: str
    p_base_target_xy: np.ndarray
    v_racket_target_w: np.ndarray
    time_to_strike: float
    strike_plan: _StrikePlan


class _WorkingPlanner:
    def __init__(self, *, time_to_strike: float = 0.8):
        self.time_to_strike = time_to_strike
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
                tuple(ball_position),
                tuple(ball_velocity),
                tuple(current_base_xy_w),
                tuple(base_forward_xy_w),
            )
        )
        return _Command(
            strike_type="forehand",
            p_base_target_xy=np.array([-0.4, 0.2]),
            v_racket_target_w=np.array([4.0, 5.0, 6.0]),
            time_to_strike=self.time_to_strike,
            strike_plan=_StrikePlan(p_racket_target=np.array([0.1, 0.2, 1.1])),
        )


class _DeadlinePlanner(_WorkingPlanner):
    def __init__(self, *, deadline_s: float = 1.928):
        super().__init__()
        self.deadline_s = deadline_s

    def plan_command(self, ball_position, ball_velocity, **kwargs):
        received_s = 1.001 + (1.5 - float(ball_position[0]))
        self.time_to_strike = self.deadline_s - received_s
        return super().plan_command(
            ball_position,
            ball_velocity,
            **kwargs,
        )


class _UnstablePlanner(_WorkingPlanner):
    def plan_command(self, ball_position, ball_velocity, **kwargs):
        if len(self.calls) == 1:
            raise RuntimeError("next qualified candidate rejected")
        return super().plan_command(
            ball_position,
            ball_velocity,
            **kwargs,
        )


def _track_inputs(count: int = 35):
    samples = [
        _sample(
            0,
            1.0,
            subject="g1pelvis",
            x=0.0,
        )
    ]
    for index in range(count):
        at_s = 1.001 + 0.01 * index
        samples.append(
            _sample(
                index + 1,
                at_s,
                x=1.5 - 0.01 * index,
            )
        )
    return tuple(samples)


def _policy_ticks(start_s: float = 1.0, count: int = 30):
    return tuple(
        PolicyTickRecord(
            tick_index=index,
            lifecycle_now_s=start_s + 0.02 * index,
            obs_now_s=start_s + 0.02 * index + 0.001,
            lifecycle_decision="none",
            phase="waiting",
            active_key=None,
            cached_key=None,
            command_fields=None,
            task_pre_clip=None,
            task_post_clip=None,
            task_clip_count=None,
        )
        for index in range(count)
    )


class HitterTaskReplayTest(unittest.TestCase):
    def test_open_private_directory_closes_fd_when_fstat_fails(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            descriptors = []
            real_open = os.open

            def capture_open(*args, **kwargs):
                descriptor = real_open(*args, **kwargs)
                descriptors.append(descriptor)
                return descriptor

            with (
                mock.patch.object(
                    replay_module.os,
                    "open",
                    side_effect=capture_open,
                ),
                mock.patch.object(
                    replay_module.os,
                    "fstat",
                    side_effect=OSError("fstat failed"),
                ),
            ):
                with self.assertRaisesRegex(OSError, "fstat failed"):
                    replay_module._open_private_directory(Path(temp_dir) / "private")

            self.assertEqual(len(descriptors), 1)
            with self.assertRaises(OSError):
                os.fstat(descriptors[0])

    def test_capture_tees_build_canonical_online_bundle_without_disk_io(self):
        class RawSink:
            def __init__(self):
                self.values = []

            def submit(self, sample, **binding):
                self.values.append((sample, binding))
                return True

        class EventSink:
            def __init__(self):
                self.values = []

            def offer(self, draft):
                self.values.append(draft)
                return True

        capture = ReplayCaptureStore(
            input_capacity=16,
            event_capacity=32,
            attempt_capacity=4,
        )
        raw_delegate = RawSink()
        event_delegate = EventSink()
        raw = ReplayRawCaptureTee(raw_delegate, capture)
        events = ReplayEventCaptureTee(event_delegate, capture)
        key = SnapshotKey(1, 31)
        result = {
            "snapshot_key": key.to_json_dict(),
            "source_frame": 31,
            "strike_deadline_monotonic_s": 1.90,
            "completed_monotonic_s": 1.003,
            "command_fields": {
                "p_base_target_xy": [-0.4, 0.0],
                "v_racket_target_w": [1.0, 2.0, 3.0],
                "time_to_strike": 0.9,
                "strike_plan": {
                    "p_racket_target": [0.1, 0.2, 1.0],
                },
            },
            "error_type": None,
            "error_text": None,
            "reason_code": None,
        }
        pelvis = _sample(0, 0.9, subject="g1pelvis", x=0.0)
        first_ball = _sample(1, 1.0)
        second_ball = _sample(2, 1.01, x=0.99)
        final_pelvis = _sample(
            3,
            1.015,
            subject="g1pelvis",
            x=0.01,
        )
        raw.submit(pelvis, attempt_id=None, track_segment_id=None)
        raw.submit(first_ball, attempt_id=1, track_segment_id=3)
        events.offer(
            EventDraft(
                kind="attempt_transition",
                monotonic_s=1.0,
                wall_time_us=1000000,
                scope="attempt",
                attempt_id=1,
                payload={
                    "track_segment_id": 3,
                    "stage": "DETECTED",
                    "reason_code": None,
                    "snapshot_key": key.to_json_dict(),
                    "values": {"role": "PRIMARY"},
                },
            )
        )
        raw.submit(second_ball, attempt_id=1, track_segment_id=3)
        raw.submit(final_pelvis, attempt_id=None, track_segment_id=None)
        events.offer(
            EventDraft(
                kind="planner_submit",
                monotonic_s=1.0,
                wall_time_us=1000000,
                scope="attempt",
                attempt_id=1,
                payload={
                    "input_seq": 1,
                    "snapshot_key": key.to_json_dict(),
                    "submission_phase": "waiting",
                },
            )
        )
        events.offer(
            EventDraft(
                kind="planner_trace",
                monotonic_s=1.001,
                wall_time_us=0,
                scope="attempt",
                attempt_id=1,
                payload={
                    "trace_seq": 2,
                    "trace_kind": "start",
                    "snapshot_key": key.to_json_dict(),
                    "track_segment_id": 3,
                    "snapshot_ready": True,
                    "incoming_confirmed": False,
                    "replaced_snapshot_key": None,
                    "result": None,
                    "replaced_result": None,
                },
            )
        )
        events.offer(
            EventDraft(
                kind="attempt_transition",
                monotonic_s=1.001,
                wall_time_us=1001000,
                scope="attempt",
                attempt_id=1,
                payload={
                    "track_segment_id": 3,
                    "stage": "ESTIMATOR_READY",
                    "reason_code": None,
                    "snapshot_key": key.to_json_dict(),
                    "values": {},
                },
            )
        )
        events.offer(
            EventDraft(
                kind="planner_trace",
                monotonic_s=1.004,
                wall_time_us=0,
                scope="attempt",
                attempt_id=1,
                payload={
                    "trace_seq": 3,
                    "trace_kind": "complete",
                    "snapshot_key": key.to_json_dict(),
                    "track_segment_id": 3,
                    "snapshot_ready": True,
                    "incoming_confirmed": True,
                    "replaced_snapshot_key": None,
                    "result": result,
                    "replaced_result": None,
                },
            )
        )
        events.offer(
            EventDraft(
                kind="lifecycle_tick",
                monotonic_s=1.02,
                wall_time_us=1020000,
                scope="attempt",
                attempt_id=1,
                payload={
                    "lifecycle_now_s": 1.02,
                    "obs_now_s": 1.021,
                    "phase": "armed",
                    "decision": "armed",
                    "active_key": key.to_json_dict(),
                    "cached_key": None,
                    "command_result": result,
                    "task_pre_clip": [0.0] * 11,
                    "task_post_clip": [0.0] * 11,
                    "clip_count": 0,
                    "task_pass": True,
                    "errors": [],
                    "recovery_duration_s": 0.98,
                },
            )
        )

        bundle = capture.finalize_attempt(
            attempt_id=1,
            terminal_code="TASK_OBS_PASS",
            recording_complete=True,
            planner_config={},
            runtime_settings=None,
            forced_strike_type=None,
        )

        self.assertEqual(
            [sample.subject for sample in bundle.inputs],
            ["g1pelvis", "ball", "ball", "g1pelvis"],
        )
        self.assertEqual(
            [stage.stage for stage in bundle.online_baseline.stages],
            [
                "DETECTED",
                "ESTIMATOR_READY",
                "INCOMING_CONFIRMED",
                "PLANNER_SUCCEEDED",
                "ARMED",
            ],
        )
        self.assertEqual(len(bundle.online_baseline.planner_calls), 1)
        self.assertAlmostEqual(
            bundle.online_baseline.planner_calls[0].duration_s,
            0.002,
        )
        self.assertEqual(len(bundle.online_baseline.policy_ticks), 1)
        self.assertAlmostEqual(
            bundle.online_baseline.recovery_duration_s,
            0.98,
        )
        self.assertAlmostEqual(
            bundle.online_baseline.submission_phase_anchor_s,
            1.0,
        )
        self.assertEqual(len(raw_delegate.values), 4)
        self.assertEqual(len(event_delegate.values), 6)

    def test_capture_overflow_marks_bundle_recording_incomplete(self):
        capture = ReplayCaptureStore(
            input_capacity=2,
            event_capacity=4,
            attempt_capacity=1,
        )
        raw = ReplayRawCaptureTee(lambda *_args, **_kwargs: True, capture)
        raw.submit(
            _sample(0, 0.9, subject="g1pelvis", x=0.0),
            attempt_id=None,
            track_segment_id=None,
        )
        raw.submit(
            _sample(1, 1.0),
            attempt_id=1,
            track_segment_id=1,
        )
        raw.submit(
            _sample(2, 1.01),
            attempt_id=1,
            track_segment_id=1,
        )

        bundle = capture.finalize_attempt(
            attempt_id=1,
            terminal_code="TRACK_ENDED_BEFORE_READY",
            recording_complete=True,
            planner_config={},
            runtime_settings=None,
            forced_strike_type=None,
        )

        self.assertFalse(bundle.recording_complete)
        self.assertFalse(bundle.online_baseline.recording_complete)
        self.assertEqual(
            bundle.online_baseline.terminal_code,
            "RECORDING_INCOMPLETE",
        )

    def test_capture_large_requests_use_bounded_actual_capacity_metadata(self):
        capture = ReplayCaptureStore(
            input_capacity=131072,
            event_capacity=65536,
            attempt_capacity=4,
        )
        expected = {
            "input_capacity": 8192,
            "event_capacity": 8192,
            "attempt_capacity": 4,
        }
        self.assertEqual(capture.capacity_metadata, expected)

        bundle = capture.finalize_attempt(
            attempt_id=1,
            terminal_code="TRACK_ENDED_BEFORE_READY",
            recording_complete=True,
            planner_config={},
            runtime_settings=None,
            forced_strike_type=None,
        )

        self.assertEqual(bundle.capture_metadata, expected)
        self.assertEqual(
            replay_input_bundle_to_json_dict(bundle)["capture_metadata"],
            expected,
        )

    def test_capture_clamped_input_boundary_is_fail_closed(self):
        capture = ReplayCaptureStore(
            input_capacity=131072,
            event_capacity=65536,
            attempt_capacity=1,
        )
        capture.record_raw(
            _sample(0, 0.0, subject="g1pelvis"),
            attempt_id=None,
            track_segment_id=None,
        )
        for sequence in range(1, 8192):
            capture.record_raw(
                _sample(sequence, sequence / 360.0),
                attempt_id=1,
                track_segment_id=1,
            )

        attempt = capture._attempts[1]
        self.assertEqual(len(attempt.inputs), 8192)
        self.assertTrue(attempt.recording_complete)

        capture.record_raw(
            _sample(8192, 8192 / 360.0),
            attempt_id=1,
            track_segment_id=1,
        )
        self.assertEqual(len(attempt.inputs), 8192)
        self.assertFalse(attempt.recording_complete)

    def test_capture_clamped_event_boundary_is_fail_closed(self):
        capture = ReplayCaptureStore(
            input_capacity=131072,
            event_capacity=65536,
            attempt_capacity=1,
        )
        for sequence in range(8192):
            capture.record_event(
                EventDraft(
                    kind="attempt_transition",
                    monotonic_s=sequence / 360.0,
                    wall_time_us=sequence,
                    scope="attempt",
                    attempt_id=1,
                    payload={
                        "track_segment_id": 1,
                        "stage": "DETECTED",
                        "reason_code": None,
                        "snapshot_key": None,
                        "values": {},
                    },
                )
            )

        attempt = capture._attempts[1]
        self.assertEqual(attempt.event_count, 8192)
        self.assertTrue(attempt.recording_complete)

        capture.record_event(
            EventDraft(
                kind="attempt_transition",
                monotonic_s=8192 / 360.0,
                wall_time_us=8192,
                scope="attempt",
                attempt_id=1,
                payload={
                    "track_segment_id": 1,
                    "stage": "DETECTED",
                    "reason_code": None,
                    "snapshot_key": None,
                    "values": {},
                },
            )
        )
        self.assertEqual(attempt.event_count, 8192)
        self.assertFalse(attempt.recording_complete)

    def test_replay_bundle_json_round_trip_is_private_and_nofollow_safe(self):
        inputs = _track_inputs()
        seed = replay_attempt(
            ReplayInputBundle(
                attempt_id=1,
                inputs=inputs,
                online_baseline=_online(ticks=_policy_ticks()),
                recording_complete=True,
            ),
            variant=ReplayVariant("3/100", 100.0, 3),
            duration_provider=_Duration(),
            planner_factory=_WorkingPlanner,
        )
        bundle = ReplayInputBundle(
            attempt_id=1,
            inputs=inputs,
            online_baseline=OnlineAttemptTrace(
                attempt_id=1,
                stages=seed.stages,
                planner_calls=seed.planner_calls,
                policy_ticks=seed.policy_ticks,
                terminal_code=seed.outcome.terminal_code,
                recovery_duration_s=seed.outcome.metrics.get("recovery_duration_s"),
                submission_phase_anchor_s=seed.outcome.metrics.get("submission_phase_anchor_s"),
                recording_complete=True,
            ),
            recording_complete=True,
            planner_config={"prediction_dt": 0.005},
            forced_strike_type="forehand",
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "private" / "attempt-1.json"
            write_replay_input_bundle(path, bundle)
            self.assertEqual(
                stat.S_IMODE(os.stat(path).st_mode),
                0o600,
            )
            loaded = load_replay_input_bundle(path)
            self.assertEqual(
                replay_input_bundle_to_json_dict(loaded),
                replay_input_bundle_to_json_dict(bundle),
            )

            link = Path(temp_dir) / "bundle-link.json"
            link.symlink_to(path)
            with self.assertRaises(OSError):
                load_replay_input_bundle(link)

    def test_top_level_disk_worker_runs_real_parity_and_checkpoints_events(self):
        ticks = (_tick(),)
        inputs = (_sample(0, 0.9, subject="g1pelvis", x=0.0),)
        seed = replay_attempt(
            ReplayInputBundle(
                attempt_id=1,
                inputs=inputs,
                online_baseline=_online(ticks=ticks),
                recording_complete=True,
            ),
            variant=ReplayVariant("3/100", 100.0, 3),
            duration_provider=_Duration(),
            planner_factory=_WorkingPlanner,
        )
        bundle = ReplayInputBundle(
            attempt_id=1,
            inputs=inputs,
            online_baseline=OnlineAttemptTrace(
                attempt_id=1,
                stages=seed.stages,
                planner_calls=seed.planner_calls,
                policy_ticks=seed.policy_ticks,
                terminal_code=seed.outcome.terminal_code,
                recovery_duration_s=seed.outcome.metrics.get("recovery_duration_s"),
                submission_phase_anchor_s=seed.outcome.metrics.get("submission_phase_anchor_s"),
                recording_complete=True,
            ),
            recording_complete=True,
            planner_config={},
        )
        checkpoints = []
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "attempt-1.json"
            write_replay_input_bundle(path, bundle)
            result = run_replay_disk_job(
                ReplayJobRef("session", 1, path),
                lambda: checkpoints.append(len(checkpoints)),
            )

        self.assertTrue(result["parity"]["matches"])
        self.assertEqual(result["summary_label"], "BOTH_FAIL_SAME")
        self.assertIsNotNone(result["one_frame"])
        self.assertGreaterEqual(len(checkpoints), 8)

    def test_scheduler_uses_time_priority_then_global_order(self):
        scheduler = DeterministicEventScheduler()
        scheduler.push(1.0, "OBS_TICK", "obs")
        scheduler.push(1.0, "GRACE_EXPIRE", "grace")
        scheduler.push(1.0, "LIFECYCLE_TICK", "lifecycle")
        scheduler.push(1.0, "INPUT", "first-input")
        scheduler.push(1.0, "INPUT", "second-input")
        scheduler.push(1.0, "PLAN_START", "start")
        scheduler.push(1.0, "PLAN_COMPLETE", "complete")

        self.assertEqual(
            [scheduler.pop().payload for _ in range(7)],
            [
                "first-input",
                "second-input",
                "complete",
                "start",
                "lifecycle",
                "obs",
                "grace",
            ],
        )

    def test_terminal_code_uses_only_the_canonical_spelling(self):
        with self.assertRaisesRegex(ValueError, "terminal_code"):
            _outcome("3/100", "TRACK_ENDED_BEFORE_CONFIRM")

    def test_bundle_rejects_attempt_identity_mismatch(self):
        with self.assertRaisesRegex(ValueError, "attempt_id"):
            ReplayInputBundle(
                attempt_id=2,
                inputs=(),
                online_baseline=_online(),
                recording_complete=True,
            )

    def test_recorded_duration_reuses_online_only_without_offline_measurement(
        self,
    ):
        key = SnapshotKey(2, 31)
        result = FrozenPlannerResult(
            snapshot_key=key,
            source_frame=31,
            strike_deadline_monotonic_s=2.0,
            completed_monotonic_s=1.1,
            command_fields={"time_to_strike": 0.9},
            error_type=None,
            error_text=None,
        )
        provider = RecordedPlannerDurationProvider(
            (
                PlannerCallRecord(
                    snapshot_key=key,
                    submitted_monotonic_s=1.0,
                    started_monotonic_s=1.0,
                    completed_monotonic_s=1.1,
                    duration_s=0.1,
                    pending_replaced_key=None,
                    latest_replaced_key=None,
                    result=result,
                ),
            )
        )

        self.assertEqual(
            provider.duration_s(
                variant=ReplayVariant("3/100", 100.0, 3),
                snapshot_key=key,
                measured_offline_duration_s=None,
            ),
            0.1,
        )
        self.assertEqual(
            provider.duration_s(
                variant=ReplayVariant("1/100", 100.0, 1),
                snapshot_key=key,
                measured_offline_duration_s=0.02,
            ),
            0.02,
        )

    def test_parity_compares_all_canonical_tick_fields(self):
        online = _online(ticks=(_tick(),))
        replay = ReplayTrace(
            variant=ReplayVariant("3/100", 100.0, 3),
            stages=(),
            planner_calls=(),
            policy_ticks=(_tick(),),
            outcome=_outcome(
                "3/100",
                "TRACK_ENDED_BEFORE_READY",
                metrics={
                    "attempt_id": 1,
                    "recovery_duration_s": None,
                    "submission_phase_anchor_s": None,
                },
            ),
        )
        self.assertTrue(
            compare_baseline_to_online(
                online=online,
                replay=replay,
            ).matches
        )

        changed = PolicyTickRecord(
            **{
                **_tick().__dict__,
                "obs_now_s": _tick().obs_now_s + 2.0e-6,
            }
        )
        mismatch = compare_baseline_to_online(
            online=online,
            replay=ReplayTrace(
                variant=replay.variant,
                stages=(),
                planner_calls=(),
                policy_ticks=(changed,),
                outcome=replay.outcome,
            ),
        )
        self.assertFalse(mismatch.matches)
        self.assertIn("policy_ticks[0].obs_now_s", mismatch.mismatches)

        missing_metrics = compare_baseline_to_online(
            online=online,
            replay=ReplayTrace(
                variant=replay.variant,
                stages=(),
                planner_calls=(),
                policy_ticks=(_tick(),),
                outcome=_outcome(
                    "3/100",
                    "TRACK_ENDED_BEFORE_READY",
                    metrics={},
                ),
            ),
        )
        self.assertFalse(missing_metrics.matches)
        self.assertIn("recovery_duration_s", missing_metrics.mismatches)

        forged_identity = compare_baseline_to_online(
            online=online,
            replay=ReplayTrace(
                variant=replay.variant,
                stages=(),
                planner_calls=(),
                policy_ticks=(_tick(),),
                outcome=_outcome(
                    "3/100",
                    "TRACK_ENDED_BEFORE_READY",
                    metrics={
                        "attempt_id": 99,
                        "recovery_duration_s": None,
                        "submission_phase_anchor_s": None,
                    },
                ),
            ),
        )
        self.assertFalse(forged_identity.matches)
        self.assertIn("attempt_id", forged_identity.mismatches)

    def test_parity_compares_command_and_planner_replacement_identity(self):
        key = SnapshotKey(2, 31)
        frozen = FrozenPlannerResult(
            snapshot_key=key,
            source_frame=100,
            strike_deadline_monotonic_s=2.0,
            completed_monotonic_s=1.1,
            command_fields={"time_to_strike": 0.9},
            error_type=None,
            error_text=None,
        )
        call = PlannerCallRecord(
            snapshot_key=key,
            submitted_monotonic_s=1.0,
            started_monotonic_s=1.0,
            completed_monotonic_s=1.1,
            duration_s=0.1,
            pending_replaced_key=None,
            latest_replaced_key=None,
            result=frozen,
        )
        online = OnlineAttemptTrace(
            attempt_id=1,
            stages=(
                AttemptTransition(
                    attempt_id=1,
                    track_segment_id=1,
                    stage="PLANNER_COMPLETE",
                    monotonic_s=1.1,
                    snapshot_key=key,
                    reason_code=None,
                    values={},
                ),
            ),
            planner_calls=(call,),
            policy_ticks=(),
            terminal_code="NO_VALID_PLAN",
            recovery_duration_s=1.85,
            submission_phase_anchor_s=1.0,
            recording_complete=True,
        )
        replay = ReplayTrace(
            ReplayVariant("3/100", 100.0, 3),
            online.stages,
            online.planner_calls,
            (),
            _outcome(
                "3/100",
                "NO_VALID_PLAN",
                metrics={
                    "attempt_id": 1,
                    "recovery_duration_s": 1.85,
                    "submission_phase_anchor_s": 1.0,
                },
            ),
        )
        self.assertTrue(compare_baseline_to_online(online=online, replay=replay).matches)

        changed_call = PlannerCallRecord(
            **{
                **call.__dict__,
                "pending_replaced_key": SnapshotKey(2, 30),
            }
        )
        parity = compare_baseline_to_online(
            online=online,
            replay=ReplayTrace(
                replay.variant,
                replay.stages,
                (changed_call,),
                (),
                replay.outcome,
            ),
        )
        self.assertFalse(parity.matches)
        self.assertIn(
            "planner_calls[0].pending_replaced_key",
            parity.mismatches,
        )

    def test_comparison_labels_and_boundaries(self):
        cases = (
            (
                _outcome("3/100", "TASK_OBS_PASS", passed=True),
                _outcome("1/100", "TASK_OBS_PASS", passed=True),
                "SAME_PASS",
            ),
            (
                _outcome("3/100", "LATE_SKIP"),
                _outcome("1/100", "TASK_OBS_PASS", passed=True),
                "SAVED_BY_ONE_FRAME",
            ),
            (
                _outcome("3/100", "TASK_OBS_PASS", passed=True),
                _outcome("1/100", "NO_VALID_PLAN"),
                "BASELINE_ONLY_PASS",
            ),
            (
                _outcome("3/100", "NO_VALID_PLAN"),
                _outcome("1/100", "NO_VALID_PLAN"),
                "BOTH_FAIL_SAME",
            ),
            (
                _outcome("3/100", "LATE_SKIP"),
                _outcome("1/100", "NO_VALID_PLAN"),
                "BOTH_FAIL_DIFFERENT",
            ),
        )
        for baseline, one_frame, expected in cases:
            with self.subTest(expected):
                comparison = compare_replay_outcomes(
                    baseline,
                    one_frame,
                )
                self.assertEqual(comparison.summary_label, expected)

        inconclusive = compare_replay_outcomes(
            _outcome("3/100", "NO_VALID_PLAN"),
            _outcome(
                "1/100",
                "TASK_OBS_PASS",
                passed=True,
                boundary=True,
            ),
        )
        self.assertEqual(inconclusive.summary_label, "INCONCLUSIVE")

    def test_comparison_reports_command_deltas(self):
        baseline = _outcome(
            "3/100",
            "TASK_OBS_PASS",
            passed=True,
            metrics={
                "base_target_x": 0.0,
                "base_target_y": 0.0,
                "racket_target_x": 0.0,
                "racket_target_y": 0.0,
                "racket_target_z": 1.0,
            },
        )
        one_frame = _outcome(
            "1/100",
            "TASK_OBS_PASS",
            passed=True,
            metrics={
                "base_target_x": 0.02,
                "base_target_y": 0.0,
                "racket_target_x": 0.0,
                "racket_target_y": 0.0,
                "racket_target_z": 1.0,
            },
        )
        comparison = compare_replay_outcomes(baseline, one_frame)
        self.assertEqual(
            comparison.summary_label,
            "BOTH_PASS_DIFFERENT_COMMAND",
        )
        self.assertAlmostEqual(
            comparison.deltas["base_target_delta_m"],
            0.02,
        )

    def test_comparison_reports_timing_racket_and_boundary_warnings(self):
        baseline = _outcome(
            "3/100",
            "TASK_OBS_PASS",
            passed=True,
            metrics={
                "confirm_time_s": 1.03,
                "arm_time_s": 1.08,
                "arm_tts_s": 0.601,
                "base_target_x": 0.0,
                "base_target_y": 0.0,
                "racket_target_x": 0.0,
                "racket_target_y": 0.0,
                "racket_target_z": 1.0,
            },
        )
        one_frame = _outcome(
            "1/100",
            "TASK_OBS_PASS",
            passed=True,
            boundary=True,
            metrics={
                "confirm_time_s": 1.01,
                "arm_time_s": 1.06,
                "arm_tts_s": 0.621,
                "base_target_x": 0.0,
                "base_target_y": 0.0,
                "racket_target_x": 0.0,
                "racket_target_y": 0.03,
                "racket_target_z": 1.04,
            },
        )
        comparison = compare_replay_outcomes(baseline, one_frame)
        self.assertEqual(comparison.summary_label, "INCONCLUSIVE")
        self.assertAlmostEqual(comparison.deltas["delta_confirm_ms"], -20.0)
        self.assertAlmostEqual(comparison.deltas["delta_arm_ms"], -20.0)
        self.assertAlmostEqual(comparison.deltas["delta_arm_tts"], 0.02)
        self.assertAlmostEqual(
            comparison.deltas["racket_target_delta_m"],
            0.05,
        )
        self.assertIn("BOUNDARY_SENSITIVE", one_frame.warnings)

    def test_replay_is_deterministic_and_reuses_online_snapshot_durations(self):
        inputs = _track_inputs()
        seed_online = _online(ticks=_policy_ticks())
        seed_bundle = ReplayInputBundle(
            attempt_id=1,
            inputs=inputs,
            online_baseline=seed_online,
            recording_complete=True,
        )
        seed_duration = _Duration(0.002)
        seed_trace = replay_attempt(
            seed_bundle,
            variant=ReplayVariant("3/100", 100.0, 3),
            duration_provider=seed_duration,
            planner_factory=_WorkingPlanner,
        )
        online = OnlineAttemptTrace(
            attempt_id=1,
            stages=seed_trace.stages,
            planner_calls=seed_trace.planner_calls,
            policy_ticks=seed_trace.policy_ticks,
            terminal_code=seed_trace.outcome.terminal_code,
            recovery_duration_s=seed_trace.outcome.metrics.get("recovery_duration_s"),
            submission_phase_anchor_s=seed_trace.outcome.metrics.get("submission_phase_anchor_s"),
            recording_complete=True,
        )
        bundle = ReplayInputBundle(
            attempt_id=1,
            inputs=inputs,
            online_baseline=online,
            recording_complete=True,
        )
        duration = _Duration(0.002)
        replay = replay_attempt(
            bundle,
            variant=ReplayVariant("3/100", 100.0, 3),
            duration_provider=duration,
            planner_factory=_WorkingPlanner,
        )

        self.assertTrue(
            compare_baseline_to_online(
                online=online,
                replay=replay,
            ).matches
        )
        self.assertEqual(seed_trace.stages, replay.stages)
        self.assertEqual(seed_trace.policy_ticks, replay.policy_ticks)
        self.assertEqual(seed_trace.outcome, replay.outcome)
        self.assertTrue(replay.planner_calls)
        online_keys = {call.snapshot_key for call in online.planner_calls}
        self.assertTrue(
            all(
                measured is None
                for variant, key, measured in duration.calls
                if variant == "3/100" and key in online_keys
            )
        )
        self.assertLess(
            replay.outcome.metrics["planner_submitted"],
            len(inputs),
        )

    def test_observation_sees_plan_completed_after_lifecycle_clock(self):
        tick = PolicyTickRecord(
            tick_index=0,
            lifecycle_now_s=1.302,
            obs_now_s=1.304,
            lifecycle_decision="none",
            phase="waiting",
            active_key=None,
            cached_key=None,
            command_fields=None,
            task_pre_clip=None,
            task_post_clip=None,
            task_clip_count=None,
        )
        trace = replay_attempt(
            ReplayInputBundle(
                attempt_id=1,
                inputs=_track_inputs(),
                online_baseline=_online(ticks=(tick,)),
                recording_complete=True,
            ),
            variant=ReplayVariant("1/100", 100.0, 1),
            duration_provider=_Duration(0.002),
            planner_factory=_WorkingPlanner,
        )

        self.assertEqual(
            trace.policy_ticks[0].active_key,
            SnapshotKey(1, 31),
        )
        self.assertIsNotNone(trace.policy_ticks[0].command_fields)

    def test_one_frame_early_full_plan_records_offline_measured_duration(self):
        inputs = _track_inputs()
        seed_bundle = ReplayInputBundle(
            attempt_id=1,
            inputs=inputs,
            online_baseline=_online(ticks=_policy_ticks()),
            recording_complete=True,
        )
        seed_trace = replay_attempt(
            seed_bundle,
            variant=ReplayVariant("3/100", 100.0, 3),
            duration_provider=_Duration(),
            planner_factory=_WorkingPlanner,
        )
        online = OnlineAttemptTrace(
            attempt_id=1,
            stages=seed_trace.stages,
            planner_calls=seed_trace.planner_calls,
            policy_ticks=seed_trace.policy_ticks,
            terminal_code=seed_trace.outcome.terminal_code,
            recovery_duration_s=seed_trace.outcome.metrics.get("recovery_duration_s"),
            submission_phase_anchor_s=seed_trace.outcome.metrics.get("submission_phase_anchor_s"),
            recording_complete=True,
        )
        duration = _Duration()
        replay_attempt(
            ReplayInputBundle(1, inputs, online, True),
            variant=ReplayVariant("1/100", 100.0, 1),
            duration_provider=duration,
            planner_factory=_WorkingPlanner,
        )
        online_keys = {call.snapshot_key for call in online.planner_calls}
        self.assertTrue(
            any(
                variant == "1/100" and key in online_keys and measured is not None
                for variant, key, measured in duration.calls
            )
        )

    def test_one_frame_warns_when_following_candidate_is_unstable(self):
        trace = replay_attempt(
            ReplayInputBundle(
                attempt_id=1,
                inputs=_track_inputs(),
                online_baseline=_online(ticks=_policy_ticks()),
                recording_complete=True,
            ),
            variant=ReplayVariant("1/100", 100.0, 1),
            duration_provider=_Duration(),
            planner_factory=_UnstablePlanner,
        )

        self.assertIn("ONE_FRAME_UNSTABLE", trace.outcome.warnings)

    def test_same_input_late_baseline_is_saved_by_one_frame(self):
        inputs = _track_inputs()
        seed_bundle = ReplayInputBundle(
            attempt_id=1,
            inputs=inputs,
            online_baseline=_online(
                ticks=_policy_ticks(count=40),
            ),
            recording_complete=True,
        )
        seed_trace = replay_attempt(
            seed_bundle,
            variant=ReplayVariant("3/100", 100.0, 3),
            duration_provider=_Duration(),
            planner_factory=_DeadlinePlanner,
        )
        self.assertEqual(seed_trace.outcome.terminal_code, "LATE_SKIP")
        online = OnlineAttemptTrace(
            attempt_id=1,
            stages=seed_trace.stages,
            planner_calls=seed_trace.planner_calls,
            policy_ticks=seed_trace.policy_ticks,
            terminal_code=seed_trace.outcome.terminal_code,
            recovery_duration_s=seed_trace.outcome.metrics.get("recovery_duration_s"),
            submission_phase_anchor_s=seed_trace.outcome.metrics.get("submission_phase_anchor_s"),
            recording_complete=True,
        )
        comparison = analyze_attempt_ab(
            ReplayInputBundle(1, inputs, online, True),
            planner_factory=_DeadlinePlanner,
            duration_provider=_Duration(),
        )
        self.assertTrue(comparison.parity.matches)
        self.assertEqual(comparison.baseline.terminal_code, "LATE_SKIP")
        self.assertEqual(
            comparison.one_frame.terminal_code,
            "TASK_OBS_PASS",
        )
        self.assertEqual(comparison.summary_label, "SAVED_BY_ONE_FRAME")

    def test_analysis_stops_before_one_frame_when_parity_diverges(self):
        factories = []

        def factory():
            factories.append(object())
            return _Planner()

        bundle = ReplayInputBundle(
            attempt_id=1,
            inputs=(),
            online_baseline=_online(terminal_code="NO_VALID_PLAN"),
            recording_complete=True,
        )
        comparison = analyze_attempt_ab(
            bundle,
            planner_factory=factory,
            duration_provider=_Duration(),
        )
        self.assertEqual(len(factories), 1)
        self.assertFalse(comparison.parity.matches)
        self.assertIsNone(comparison.one_frame)
        self.assertEqual(comparison.summary_label, "INCONCLUSIVE")
        self.assertEqual(
            comparison.baseline.terminal_code,
            "REPLAY_DIVERGENCE",
        )

    def test_analysis_uses_fresh_planner_for_each_variant_after_parity(self):
        factories = []

        def factory():
            planner = _Planner()
            factories.append(planner)
            return planner

        bundle = ReplayInputBundle(
            attempt_id=1,
            inputs=(),
            online_baseline=_online(),
            recording_complete=True,
        )
        comparison = analyze_attempt_ab(
            bundle,
            planner_factory=factory,
            duration_provider=_Duration(),
        )
        self.assertEqual(len(factories), 2)
        self.assertTrue(comparison.parity.matches)
        self.assertIsNotNone(comparison.one_frame)
        self.assertEqual(
            comparison.summary_label,
            "BOTH_FAIL_SAME",
        )

    def test_incomplete_recording_never_claims_variant_result(self):
        bundle = ReplayInputBundle(
            attempt_id=1,
            inputs=(_sample(0, 1.0),),
            online_baseline=_online(),
            recording_complete=False,
        )
        comparison = analyze_attempt_ab(
            bundle,
            planner_factory=lambda: _Planner(),
            duration_provider=_Duration(),
        )
        self.assertEqual(comparison.summary_label, "INCONCLUSIVE")
        self.assertEqual(
            comparison.baseline.terminal_code,
            "RECORDING_INCOMPLETE",
        )
        self.assertIsNone(comparison.one_frame)


if __name__ == "__main__":
    unittest.main()
