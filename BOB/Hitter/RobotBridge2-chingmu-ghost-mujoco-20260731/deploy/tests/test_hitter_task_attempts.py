from __future__ import annotations

import json
import math
import threading
import unittest
from dataclasses import FrozenInstanceError, replace

import numpy as np

from diagnostics.hitter_task_attempts import AttemptTracker
from diagnostics.hitter_task_models import (
    AttemptBinding,
    AttemptDetail,
    AttemptPage,
    AttemptSummary,
    AttemptTransition,
    EventDraft,
    HealthSnapshot,
    LifecycleSnapshot,
    NormalizedMocapSample,
    SnapshotKey,
    freeze_json_value,
    to_builtin_json,
)

_nextafter = getattr(math, "nextafter", np.nextafter)


def _sample(
    monotonic_s: float,
    *,
    valid: bool = True,
    occluded: bool = False,
) -> NormalizedMocapSample:
    return NormalizedMocapSample(
        input_seq=int(monotonic_s * 1000),
        channel="vicon_state_data",
        subject="ball",
        position_w=np.array([1.0, 2.0, 3.0], dtype=np.float32),
        quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32),
        valid=valid,
        occluded=occluded,
        source_frame=123,
        source_time_s=None,
        publish_time_us=None,
        received_monotonic_s=monotonic_s,
        wall_time_us=1_700_000_000_000_000,
        payload_size=64,
    )


def _close_attempt(
    tracker: AttemptTracker,
    *,
    invalid_time_s: float = 10.0,
) -> AttemptTransition:
    tracker.observe_ball_sample(
        _sample(invalid_time_s, valid=False),
        snapshot_key=SnapshotKey(2, 2),
    )
    transitions = tracker.advance(now_monotonic_s=invalid_time_s + 0.200001)
    return next(item for item in transitions if item.stage == "ATTEMPT_CLOSED")


class ImmutableModelTests(unittest.TestCase):
    def test_mocap_arrays_are_independent_readonly_float64_and_json_round_trip(self) -> None:
        position = np.array([1.0, 2.0, 3.0], dtype=np.float32)
        quaternion = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        sample = NormalizedMocapSample(
            input_seq=7,
            channel="chan",
            subject="ball",
            position_w=position,
            quaternion_xyzw=quaternion,
            valid=True,
            occluded=False,
            source_frame=9,
            source_time_s=None,
            publish_time_us=None,
            received_monotonic_s=1.25,
            wall_time_us=99,
            payload_size=12,
        )

        position[0] = 99.0
        quaternion[3] = 0.0
        self.assertEqual(sample.position_w.dtype, np.dtype(np.float64))
        self.assertEqual(sample.quaternion_xyzw.dtype, np.dtype(np.float64))
        self.assertFalse(sample.position_w.flags.writeable)
        self.assertFalse(sample.quaternion_xyzw.flags.writeable)
        np.testing.assert_array_equal(sample.position_w, [1.0, 2.0, 3.0])
        np.testing.assert_array_equal(sample.quaternion_xyzw, [0.0, 0.0, 0.0, 1.0])
        with self.assertRaises(ValueError):
            sample.position_w[0] = -1.0
        with self.assertRaises(ValueError):
            sample.position_w.setflags(write=True)
        with self.assertRaises(ValueError):
            sample.quaternion_xyzw.setflags(write=True)

        encoded = sample.to_json_dict()
        self.assertEqual(encoded["schema_version"], 1)
        self.assertIsNone(encoded["source_time_s"])
        self.assertIsNone(encoded["publish_time_us"])
        round_tripped = json.loads(json.dumps(encoded))
        np.testing.assert_array_equal(
            np.asarray(round_tripped["position_w"], dtype=np.float64),
            sample.position_w,
        )

    def test_nested_json_values_are_deep_copied_frozen_and_explicitly_serialized(self) -> None:
        nested = {"metrics": [1, {"reason": "warm"}]}
        draft = EventDraft(
            kind="progress",
            monotonic_s=1.0,
            wall_time_us=2,
            scope="attempt",
            attempt_id=3,
            payload=nested,
        )
        nested["metrics"][1]["reason"] = "mutated"
        nested["metrics"].append(2)

        self.assertEqual(draft.payload["metrics"][1]["reason"], "warm")
        with self.assertRaises(TypeError):
            draft.payload["other"] = 1
        with self.assertRaises(TypeError):
            draft.payload["metrics"][1]["reason"] = "changed"
        self.assertEqual(
            json.loads(json.dumps(draft.to_json_dict()))["payload"],
            {"metrics": [1, {"reason": "warm"}]},
        )
        with self.assertRaises(FrozenInstanceError):
            draft.kind = "changed"

    def test_nonfinite_values_use_fixed_json_tokens_everywhere(self) -> None:
        sample = NormalizedMocapSample(
            input_seq=1,
            channel="chan",
            subject="ball",
            position_w=np.array([float("nan"), float("inf"), float("-inf")]),
            quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
            valid=False,
            occluded=False,
            source_frame=-1,
            source_time_s=float("nan"),
            publish_time_us=None,
            received_monotonic_s=float("inf"),
            wall_time_us=1,
            payload_size=0,
        )
        draft = EventDraft(
            kind="warning",
            monotonic_s=float("-inf"),
            wall_time_us=1,
            scope="state",
            attempt_id=None,
            payload={"values": [float("nan"), float("inf"), float("-inf")]},
        )
        summary = AttemptSummary(
            attempt_id=1,
            status="FAILED",
            stage="CLOSED",
            primary_blocker="OBS_NONFINITE",
            ball_speed_mps=float("nan"),
            predicted_strike_time_s=float("inf"),
            planner_tts_s=float("-inf"),
            arm_tts_s=None,
            task_obs_status="FAILED",
            ab_summary=None,
            recording_complete=True,
        )
        transition = AttemptTransition(
            attempt_id=1,
            track_segment_id=1,
            stage="WARNING",
            monotonic_s=float("nan"),
            snapshot_key=SnapshotKey(1, 1),
            reason_code="OBS_NONFINITE",
            values={"value": float("inf")},
        )
        health = replace(
            HealthSnapshot(
                lcm_connected=True,
                message_rate_hz_by_subject={"ball": float("nan")},
                message_age_s_by_subject={"ball": float("inf")},
                source_frame_by_subject={"ball": 1},
                pelvis_valid=True,
                pelvis_age_s=None,
                planner_submitted=0,
                planner_completed=0,
                planner_failed=0,
                planner_dropped_pending=0,
                planner_results_overwritten_before_consume=0,
                raw_samples_dropped=0,
                recorder_event_gaps=0,
                diagnostic_events_dropped=0,
                recorder_healthy=True,
                recording_complete=True,
                config_name="hitter",
                session_basename="session",
                warnings=(),
            ),
            pelvis_age_s=float("-inf"),
        )
        lifecycle = LifecycleSnapshot(
            phase="TRACKING",
            last_decision="NONE",
            active_key=None,
            cached_key=None,
            lifecycle_now_s=float("nan"),
            obs_now_s=float("inf"),
        )
        detail = AttemptDetail(
            attempt_id=1,
            summary=summary,
            segments=(),
            stage_timeline=(transition.to_json_dict(),),
            planner_inputs=(),
            planner_results=(),
            task_observation_pre_clip=(float("nan"),),
            task_observation_post_clip=(float("inf"),),
            task_observation_clip_count=0,
            variant_outcomes=(),
            ab_deltas={"delta": float("-inf")},
        )
        page = AttemptPage(items=(summary,), has_more=False, next_before=None)

        self.assertEqual(
            freeze_json_value([float("nan"), float("inf"), float("-inf")]),
            ("NaN", "Infinity", "-Infinity"),
        )
        self.assertEqual(
            to_builtin_json((float("nan"), float("inf"), float("-inf"))),
            ["NaN", "Infinity", "-Infinity"],
        )
        self.assertEqual(
            sample.to_json_dict()["position_w"],
            ["NaN", "Infinity", "-Infinity"],
        )
        self.assertEqual(draft.to_json_dict()["monotonic_s"], "-Infinity")
        self.assertEqual(summary.to_json_dict()["ball_speed_mps"], "NaN")
        for model in (
            sample,
            draft,
            transition,
            health,
            lifecycle,
            summary,
            detail,
            page,
        ):
            json.dumps(model.to_json_dict(), allow_nan=False)

    def test_event_draft_validates_scope_and_attempt_identity(self) -> None:
        with self.assertRaises(ValueError):
            EventDraft(
                kind="warning",
                monotonic_s=1.0,
                wall_time_us=1,
                scope="other",
                attempt_id=None,
                payload={},
            )
        with self.assertRaises(ValueError):
            EventDraft(
                kind="warning",
                monotonic_s=1.0,
                wall_time_us=1,
                scope="attempt",
                attempt_id=None,
                payload={},
            )
        EventDraft(
            kind="health",
            monotonic_s=1.0,
            wall_time_us=1,
            scope="state",
            attempt_id=None,
            payload={},
        )

    def test_all_protocol_models_emit_schema_version_one_json_values(self) -> None:
        key = SnapshotKey(4, 5)
        binding = AttemptBinding(1, 2, "PRIMARY", key)
        transition = AttemptTransition(
            attempt_id=1,
            track_segment_id=2,
            stage="PLANNED",
            monotonic_s=3.0,
            snapshot_key=key,
            reason_code=None,
            values={"tts_s": 0.4},
        )
        health = HealthSnapshot(
            lcm_connected=True,
            message_rate_hz_by_subject={"ball": 360.0},
            message_age_s_by_subject={"ball": None},
            source_frame_by_subject={"ball": 5},
            pelvis_valid=True,
            pelvis_age_s=None,
            planner_submitted=1,
            planner_completed=1,
            planner_failed=0,
            planner_dropped_pending=0,
            planner_results_overwritten_before_consume=0,
            raw_samples_dropped=0,
            recorder_event_gaps=0,
            diagnostic_events_dropped=0,
            recorder_healthy=True,
            recording_complete=True,
            config_name="hitter",
            session_basename="session",
            warnings=("SOURCE_FRAME_GAP",),
        )
        lifecycle = LifecycleSnapshot(
            phase="TRACKING",
            last_decision="ACCEPTED",
            active_key=key,
            cached_key=None,
            lifecycle_now_s=1.0,
            obs_now_s=None,
        )
        summary = AttemptSummary(
            attempt_id=1,
            status="ACTIVE",
            stage="PLANNED",
            primary_blocker=None,
            ball_speed_mps=4.5,
            predicted_strike_time_s=None,
            planner_tts_s=0.4,
            arm_tts_s=None,
            task_obs_status="PENDING",
            ab_summary=None,
            recording_complete=True,
        )
        detail = AttemptDetail(
            attempt_id=1,
            summary=summary,
            segments=({"track_segment_id": 2},),
            stage_timeline=(transition.to_json_dict(),),
            planner_inputs=(),
            planner_results=(),
            task_observation_pre_clip=None,
            task_observation_post_clip=None,
            task_observation_clip_count=None,
            variant_outcomes=(),
            ab_deltas={},
        )
        page = AttemptPage(items=(summary,), has_more=False, next_before=None)

        for model in (key, binding, transition, health, lifecycle, summary, detail, page):
            payload = model.to_json_dict()
            self.assertEqual(payload["schema_version"], 1)
            json.dumps(payload, allow_nan=False)


class AttemptTrackerTests(unittest.TestCase):
    def test_visible_edges_allocate_globally_monotonic_attempt_and_segment_ids(self) -> None:
        tracker = AttemptTracker(reacquire_grace_s=0.20)
        key_1 = SnapshotKey(5, 31)
        transitions = tracker.observe_ball_sample(_sample(1.0), snapshot_key=key_1)
        self.assertEqual([item.stage for item in transitions], ["DETECTED"])
        self.assertEqual(tracker.bind_snapshot(snapshot_key=key_1), AttemptBinding(1, 1, "PRIMARY", key_1))

        tracker.observe_ball_sample(
            _sample(1.1, valid=False),
            snapshot_key=SnapshotKey(6, 32),
        )
        tracker.advance(now_monotonic_s=1.300001)
        key_2 = SnapshotKey(7, 40)
        tracker.observe_ball_sample(_sample(1.4), snapshot_key=key_2)
        self.assertEqual(tracker.bind_snapshot(snapshot_key=key_2), AttemptBinding(2, 2, "PRIMARY", key_2))

        tracker.observe_ball_sample(
            _sample(1.5, valid=False),
            snapshot_key=SnapshotKey(8, 41),
        )
        key_3 = SnapshotKey(8, 42)
        tracker.observe_ball_sample(_sample(1.6), snapshot_key=key_3)
        self.assertEqual(tracker.bind_snapshot(snapshot_key=key_3), AttemptBinding(2, 3, "PRIMARY", key_3))

    def test_advanced_epoch_invalid_snapshot_stays_with_closed_segment_then_reacquires(self) -> None:
        tracker = AttemptTracker(reacquire_grace_s=0.20)
        visible_key = SnapshotKey(5, 31)
        invalid_key = SnapshotKey(6, 32)
        reacquired_key = SnapshotKey(6, 33)

        tracker.observe_ball_sample(_sample(10.0), snapshot_key=visible_key)
        invalid_events = tracker.observe_ball_sample(
            _sample(10.1, valid=False),
            snapshot_key=invalid_key,
        )
        self.assertEqual([item.stage for item in invalid_events], ["REACQUIRE_GRACE"])
        self.assertEqual(
            tracker.binding_for_result(invalid_key),
            AttemptBinding(1, 1, "PRIMARY", invalid_key),
        )

        reacquired = tracker.observe_ball_sample(_sample(10.2), snapshot_key=reacquired_key)
        self.assertEqual([item.stage for item in reacquired], ["DETECTED"])
        self.assertEqual(
            tracker.binding_for_result(reacquired_key),
            AttemptBinding(1, 2, "PRIMARY", reacquired_key),
        )

    def test_production_reset_closes_display_segment_without_invoking_production(self) -> None:
        tracker = AttemptTracker(reacquire_grace_s=0.20)
        tracker.observe_ball_sample(_sample(4.0), snapshot_key=SnapshotKey(9, 10))
        events = tracker.observe_production_reset(
            previous_track_epoch=9,
            new_track_epoch=10,
            reason="STRIKE_DEADLINE_RESET",
            monotonic_s=4.1,
        )
        self.assertEqual([item.stage for item in events], ["REACQUIRE_GRACE"])
        invisible_key = SnapshotKey(10, 11)
        self.assertEqual(
            tracker.observe_ball_sample(
                _sample(4.11, valid=False),
                snapshot_key=invisible_key,
            ),
            (),
        )
        self.assertEqual(
            tracker.binding_for_result(invisible_key),
            AttemptBinding(1, 1, "PRIMARY", invisible_key),
        )

    def test_grace_deadline_is_closed_interval_and_strictly_later_closes(self) -> None:
        tracker = AttemptTracker(reacquire_grace_s=0.20)
        tracker.observe_ball_sample(_sample(10.0), snapshot_key=SnapshotKey(1, 1))
        tracker.observe_ball_sample(
            _sample(10.1, valid=False),
            snapshot_key=SnapshotKey(2, 2),
        )
        first_deadline = 10.1 + 0.20
        self.assertEqual(tracker.advance(now_monotonic_s=first_deadline), ())
        boundary_key = SnapshotKey(2, 3)
        tracker.observe_ball_sample(_sample(first_deadline), snapshot_key=boundary_key)
        self.assertEqual(
            tracker.binding_for_result(boundary_key),
            AttemptBinding(1, 2, "PRIMARY", boundary_key),
        )

        tracker.observe_ball_sample(
            _sample(11.0, valid=False),
            snapshot_key=SnapshotKey(3, 4),
        )
        second_deadline = 11.0 + 0.20
        closed = tracker.advance(
            now_monotonic_s=float(_nextafter(second_deadline, math.inf))
        )
        self.assertEqual([item.stage for item in closed], ["ATTEMPT_CLOSED"])
        next_key = SnapshotKey(4, 5)
        tracker.observe_ball_sample(_sample(11.3), snapshot_key=next_key)
        self.assertEqual(
            tracker.binding_for_result(next_key),
            AttemptBinding(2, 3, "PRIMARY", next_key),
        )

    def test_passed_attempt_reacquisition_is_post_deadline_tail(self) -> None:
        tracker = AttemptTracker()
        first_key = SnapshotKey(1, 1)
        tracker.observe_ball_sample(_sample(1.0), snapshot_key=first_key)
        binding = tracker.bind_snapshot(snapshot_key=first_key)
        self.assertIsNotNone(binding)
        tracker.record_task_pass(binding=binding, monotonic_s=1.1)
        tracker.observe_ball_sample(
            _sample(1.2, valid=False),
            snapshot_key=SnapshotKey(2, 2),
        )
        tail_key = SnapshotKey(2, 3)
        events = tracker.observe_ball_sample(_sample(1.3), snapshot_key=tail_key)
        self.assertEqual([item.stage for item in events], ["POST_DEADLINE_TAIL"])
        self.assertEqual(
            tracker.bind_snapshot(snapshot_key=tail_key),
            AttemptBinding(1, 2, "POST_DEADLINE_TAIL", tail_key),
        )

    def test_health_warnings_do_not_open_close_or_change_segment(self) -> None:
        tracker = AttemptTracker()
        self.assertIsNone(
            tracker.record_health_warning(
                reason_code="LCM_HEARTBEAT_STALE",
                monotonic_s=0.0,
                values={},
            )
        )
        key = SnapshotKey(1, 1)
        tracker.observe_ball_sample(_sample(1.0), snapshot_key=key)
        before = tracker.bind_snapshot(snapshot_key=key)
        warning = tracker.record_health_warning(
            reason_code="PELVIS_INVALID",
            monotonic_s=1.1,
            values={"age_s": 0.2},
        )
        self.assertEqual(warning.stage, "WARNING")
        self.assertEqual(warning.reason_code, "PELVIS_INVALID")
        later_key = SnapshotKey(1, 2)
        tracker.observe_ball_sample(_sample(1.2), snapshot_key=later_key)
        after = tracker.bind_snapshot(snapshot_key=later_key)
        self.assertEqual(before.attempt_id, after.attempt_id)
        self.assertEqual(before.track_segment_id, after.track_segment_id)

    def test_recovery_projects_waiting_and_cached_onto_next_attempt(self) -> None:
        tracker = AttemptTracker()
        old_key = SnapshotKey(1, 1)
        tracker.observe_ball_sample(_sample(1.0), snapshot_key=old_key)
        tracker.observe_ball_sample(
            _sample(1.1, valid=False),
            snapshot_key=SnapshotKey(2, 2),
        )
        tracker.advance(now_monotonic_s=1.300001)
        next_key = SnapshotKey(3, 3)
        tracker.observe_ball_sample(_sample(1.4), snapshot_key=next_key)

        waiting = tracker.set_lifecycle_context(
            phase="RECOVERY",
            active_key=old_key,
            cached_key=None,
            monotonic_s=1.5,
        )
        self.assertEqual([item.stage for item in waiting], ["WAITING_FOR_PREVIOUS_RECOVERY"])
        self.assertEqual(waiting[0].attempt_id, 2)

        cached = tracker.set_lifecycle_context(
            phase="RECOVERY",
            active_key=old_key,
            cached_key=next_key,
            monotonic_s=1.6,
        )
        self.assertEqual([item.stage for item in cached], ["CACHED_DURING_RECOVERY"])
        self.assertEqual(cached[0].attempt_id, 2)

    def test_late_result_uses_immutable_snapshot_binding_not_current_attempt(self) -> None:
        tracker = AttemptTracker()
        old_key = SnapshotKey(1, 10)
        tracker.observe_ball_sample(_sample(1.0), snapshot_key=old_key)
        old_binding = tracker.bind_snapshot(snapshot_key=old_key)
        tracker.observe_ball_sample(
            _sample(1.1, valid=False),
            snapshot_key=SnapshotKey(2, 11),
        )
        tracker.advance(now_monotonic_s=1.300001)
        tracker.observe_ball_sample(_sample(1.4), snapshot_key=SnapshotKey(3, 12))

        self.assertEqual(tracker.binding_for_result(old_key), old_binding)
        late = tracker.record_stage(
            binding=old_binding,
            stage="PLANNER_REJECTED",
            monotonic_s=1.5,
            reason_code="NO_DIRECTED_CROSSING",
            values={},
        )
        self.assertEqual(late.attempt_id, 1)
        self.assertEqual(late.track_segment_id, 1)

    def test_primary_blocker_is_selected_from_farthest_stage(self) -> None:
        cases = (
            ("never_ready", (), "TRACK_ENDED_BEFORE_READY", "FAILED"),
            (
                "never_confirmed",
                (("ESTIMATOR_READY", None),),
                "TRACK_ENDED_BEFORE_CONFIRMATION",
                "FAILED",
            ),
            (
                "last_planner_reason",
                (
                    ("ESTIMATOR_READY", None),
                    ("INCOMING_CONFIRMED", None),
                    ("PLANNER_REJECTED", "NO_DIRECTED_CROSSING"),
                ),
                "NO_DIRECTED_CROSSING",
                "FAILED",
            ),
            (
                "fallback_no_plan",
                (("ESTIMATOR_READY", None), ("INCOMING_CONFIRMED", None)),
                "NO_VALID_PLAN",
                "FAILED",
            ),
            (
                "late_before_arm",
                (
                    ("ESTIMATOR_READY", None),
                    ("INCOMING_CONFIRMED", None),
                    ("PLANNED", None),
                    ("LATE_SKIP", "LATE_SKIP"),
                ),
                "LATE_SKIP",
                "FAILED",
            ),
            (
                "before_arm",
                (
                    ("ESTIMATOR_READY", None),
                    ("INCOMING_CONFIRMED", None),
                    ("PLANNED", None),
                ),
                "TRACK_ENDED_BEFORE_ARM",
                "FAILED",
            ),
            (
                "explicit_obs_failure",
                (
                    ("ESTIMATOR_READY", None),
                    ("INCOMING_CONFIRMED", None),
                    ("PLANNED", None),
                    ("ARMED", None),
                    ("TASK_OBS_FAILED", "OBS_CLIPPED"),
                ),
                "OBS_CLIPPED",
                "FAILED",
            ),
            (
                "pass_wins",
                (
                    ("ESTIMATOR_READY", None),
                    ("INCOMING_CONFIRMED", None),
                    ("PLANNED", None),
                    ("ARMED", None),
                    ("TASK_OBS_PASS", None),
                ),
                None,
                "SUCCESS",
            ),
        )
        for name, stages, expected_blocker, expected_status in cases:
            with self.subTest(name=name):
                tracker = AttemptTracker()
                key = SnapshotKey(1, 1)
                tracker.observe_ball_sample(_sample(1.0), snapshot_key=key)
                binding = tracker.bind_snapshot(snapshot_key=key)
                tracker.record_health_warning(
                    reason_code="LCM_HEARTBEAT_STALE",
                    monotonic_s=1.01,
                    values={},
                )
                for stage, reason in stages:
                    if stage == "TASK_OBS_PASS":
                        tracker.record_task_pass(binding=binding, monotonic_s=1.1)
                    else:
                        tracker.record_stage(
                            binding=binding,
                            stage=stage,
                            monotonic_s=1.1,
                            reason_code=reason,
                            values={},
                        )
                closed = _close_attempt(tracker)
                self.assertEqual(closed.reason_code, expected_blocker)
                self.assertEqual(closed.values["primary_blocker"], expected_blocker)
                self.assertEqual(closed.values["status"], expected_status)

    def test_explicit_hard_failure_precedes_only_stage_fallbacks(self) -> None:
        for reason in ("PELVIS_UNAVAILABLE", "MALFORMED_COMMAND"):
            with self.subTest(reason=reason):
                tracker = AttemptTracker()
                key = SnapshotKey(1, 1)
                tracker.observe_ball_sample(
                    _sample(1.0),
                    snapshot_key=key,
                )
                binding = tracker.bind_snapshot(snapshot_key=key)
                tracker.record_stage(
                    binding=binding,
                    stage="ESTIMATOR_READY",
                    monotonic_s=1.01,
                    reason_code=None,
                    values={},
                )
                tracker.record_stage(
                    binding=binding,
                    stage="PLANNER_REJECTED",
                    monotonic_s=1.02,
                    reason_code=reason,
                    values={},
                )

                closed = _close_attempt(tracker)

                self.assertEqual(closed.reason_code, reason)
                self.assertEqual(
                    closed.values["primary_blocker"],
                    reason,
                )

        tracker = AttemptTracker()
        key = SnapshotKey(1, 1)
        tracker.observe_ball_sample(_sample(1.0), snapshot_key=key)
        binding = tracker.bind_snapshot(snapshot_key=key)
        tracker.record_stage(
            binding=binding,
            stage="ESTIMATOR_READY",
            monotonic_s=1.01,
            reason_code=None,
            values={},
        )
        tracker.record_stage(
            binding=binding,
            stage="PLANNER_REJECTED",
            monotonic_s=1.02,
            reason_code="INCOMING_SPEED_REJECTED",
            values={},
        )
        closed = _close_attempt(tracker)
        self.assertEqual(
            closed.reason_code,
            "TRACK_ENDED_BEFORE_CONFIRMATION",
        )

    def test_attempt_binding_and_timeline_state_are_bounded(self) -> None:
        tracker = AttemptTracker(
            max_attempts=3,
            max_bindings=3,
            max_timeline_per_attempt=4,
        )
        first_key = SnapshotKey(1, 1)
        tracker.observe_ball_sample(_sample(1.0), snapshot_key=first_key)
        for generation in range(2, 7):
            tracker.observe_ball_sample(
                _sample(1.0 + generation * 0.001),
                snapshot_key=SnapshotKey(1, generation),
            )
        self.assertIsNone(tracker.binding_for_result(first_key))
        self.assertIsNotNone(tracker.binding_for_result(SnapshotKey(1, 6)))
        for index in range(10):
            tracker.record_health_warning(
                reason_code="SOURCE_FRAME_GAP",
                monotonic_s=1.1 + index * 0.001,
                values={"index": index},
            )
        self.assertEqual(len(tracker.timeline_for_attempt(1)), 4)

        tracker.observe_ball_sample(
            _sample(2.0, valid=False),
            snapshot_key=SnapshotKey(2, 7),
        )
        tracker.advance(now_monotonic_s=float(_nextafter(2.2, math.inf)))
        for attempt_id in range(2, 7):
            epoch = attempt_id * 2
            tracker.observe_ball_sample(
                _sample(float(epoch)),
                snapshot_key=SnapshotKey(epoch, 1),
            )
            tracker.observe_ball_sample(
                _sample(float(epoch) + 0.1, valid=False),
                snapshot_key=SnapshotKey(epoch + 1, 2),
            )
            tracker.advance(
                now_monotonic_s=float(
                    _nextafter(float(epoch) + 0.3, math.inf)
                )
            )

        self.assertIsNone(tracker.summary_for_attempt(1))
        self.assertIsNone(tracker.summary_for_attempt(3))
        self.assertIsNotNone(tracker.summary_for_attempt(4))
        self.assertIsNotNone(tracker.summary_for_attempt(6))

    def test_active_attempt_is_never_evicted_from_attempt_cache(self) -> None:
        tracker = AttemptTracker(max_attempts=1)
        tracker.observe_ball_sample(_sample(1.0), snapshot_key=SnapshotKey(1, 1))
        tracker.observe_ball_sample(
            _sample(1.1, valid=False),
            snapshot_key=SnapshotKey(2, 2),
        )
        tracker.advance(now_monotonic_s=float(_nextafter(1.3, math.inf)))
        tracker.observe_ball_sample(_sample(2.0), snapshot_key=SnapshotKey(3, 3))

        current = tracker.current_summary()
        self.assertIsNotNone(current)
        self.assertEqual(current.attempt_id, 2)
        self.assertIsNone(tracker.summary_for_attempt(1))
        self.assertIsNotNone(tracker.summary_for_attempt(2))

    def test_default_attempt_cache_stays_at_one_hundred_over_long_stream(self) -> None:
        tracker = AttemptTracker()
        for attempt_id in range(1, 106):
            start_s = float(attempt_id)
            tracker.observe_ball_sample(
                _sample(start_s),
                snapshot_key=SnapshotKey(attempt_id * 2, 1),
            )
            invalid_s = start_s + 0.1
            tracker.observe_ball_sample(
                _sample(invalid_s, valid=False),
                snapshot_key=SnapshotKey(attempt_id * 2 + 1, 2),
            )
            deadline = invalid_s + 0.20
            tracker.advance(
                now_monotonic_s=float(_nextafter(deadline, math.inf))
            )

        self.assertIsNone(tracker.summary_for_attempt(5))
        self.assertIsNotNone(tracker.summary_for_attempt(6))
        self.assertIsNotNone(tracker.summary_for_attempt(105))

    def test_terminal_outcome_is_not_rewritten_by_late_result(self) -> None:
        tracker = AttemptTracker()
        key = SnapshotKey(1, 1)
        tracker.observe_ball_sample(_sample(1.0), snapshot_key=key)
        binding = tracker.bind_snapshot(snapshot_key=key)
        tracker.observe_ball_sample(
            _sample(1.1, valid=False),
            snapshot_key=SnapshotKey(2, 2),
        )
        tracker.advance(now_monotonic_s=float(_nextafter(1.3, math.inf)))
        before = tracker.summary_for_attempt(1)
        self.assertEqual(before.status, "FAILED")
        self.assertEqual(before.primary_blocker, "TRACK_ENDED_BEFORE_READY")

        late = tracker.record_task_pass(binding=binding, monotonic_s=2.0)
        after = tracker.summary_for_attempt(1)
        self.assertEqual(late.stage, "TASK_OBS_PASS")
        self.assertEqual(after.status, "FAILED")
        self.assertEqual(after.primary_blocker, "TRACK_ENDED_BEFORE_READY")
        self.assertEqual(tracker.timeline_for_attempt(1)[-1], late)

    def test_public_methods_are_safe_under_concurrent_readers_and_writers(self) -> None:
        tracker = AttemptTracker(
            max_bindings=1024,
            max_timeline_per_attempt=1024,
        )
        tracker.observe_ball_sample(_sample(1.0), snapshot_key=SnapshotKey(1, 1))
        errors = []
        start = threading.Barrier(5)

        def samples(worker_id: int) -> None:
            try:
                start.wait()
                for offset in range(100):
                    key = SnapshotKey(1, worker_id * 100 + offset + 2)
                    tracker.observe_ball_sample(
                        _sample(2.0 + offset * 0.001),
                        snapshot_key=key,
                    )
                    tracker.bind_snapshot(snapshot_key=key)
            except BaseException as exc:
                errors.append(exc)

        def reader() -> None:
            try:
                start.wait()
                for _ in range(200):
                    tracker.current_summary()
                    tracker.summary_for_attempt(1)
                    tracker.timeline_for_attempt(1)
            except BaseException as exc:
                errors.append(exc)

        threads = [
            threading.Thread(target=samples, args=(worker_id,))
            for worker_id in range(4)
        ]
        threads.append(threading.Thread(target=reader))
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

        self.assertEqual(errors, [])
        self.assertEqual(tracker.current_summary().attempt_id, 1)


if __name__ == "__main__":
    unittest.main()
