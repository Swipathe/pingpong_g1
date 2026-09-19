from __future__ import annotations

import gc
import json
import math
import threading
import unittest
from dataclasses import replace
import weakref

from diagnostics.hitter_task_events import (
    DiagnosticState,
    EventHub,
    EventPublisherLane,
    ProgressMailbox,
    reduce_diagnostic_state,
)
from diagnostics.hitter_task_models import (
    AttemptSummary,
    EventDraft,
    HealthSnapshot,
    LifecycleSnapshot,
)


def _health(*, dropped: int = 0) -> HealthSnapshot:
    return HealthSnapshot(
        lcm_connected=True,
        message_rate_hz_by_subject={"ball": 360.0},
        message_age_s_by_subject={"ball": 0.01},
        source_frame_by_subject={"ball": 10},
        pelvis_valid=True,
        pelvis_age_s=0.01,
        planner_submitted=0,
        planner_completed=0,
        planner_failed=0,
        planner_dropped_pending=0,
        planner_results_overwritten_before_consume=0,
        raw_samples_dropped=0,
        recorder_event_gaps=0,
        diagnostic_events_dropped=dropped,
        recorder_healthy=True,
        recording_complete=True,
        config_name="hitter",
        session_basename="test-session",
        warnings=(),
    )


def _lifecycle(*, phase: str = "TRACKING") -> LifecycleSnapshot:
    return LifecycleSnapshot(
        phase=phase,
        last_decision="NONE",
        active_key=None,
        cached_key=None,
        lifecycle_now_s=None,
        obs_now_s=None,
    )


def _summary(attempt_id: int, *, status: str = "ACTIVE") -> AttemptSummary:
    return AttemptSummary(
        attempt_id=attempt_id,
        status=status,
        stage="DETECTED",
        primary_blocker=None,
        ball_speed_mps=None,
        predicted_strike_time_s=None,
        planner_tts_s=None,
        arm_tts_s=None,
        task_obs_status="PENDING",
        ab_summary=None,
        recording_complete=True,
    )


def _state() -> DiagnosticState:
    return DiagnosticState(
        schema_version=1,
        watermark_event_id=0,
        health=_health(),
        lifecycle=_lifecycle(),
        current_attempt=None,
        recent_attempts=(),
    )


def _draft(
    sequence: int,
    *,
    kind: str = "PROGRESS",
    attempt_id: int = 1,
    payload=None,
) -> EventDraft:
    return EventDraft(
        kind=kind,
        monotonic_s=float(sequence),
        wall_time_us=sequence,
        scope="attempt",
        attempt_id=attempt_id,
        payload={"sequence": sequence} if payload is None else payload,
    )


class EventHubTests(unittest.TestCase):
    def test_eight_publishers_allocate_one_contiguous_sequence(self) -> None:
        hub = EventHub(_state(), capacity=256)
        published = []
        published_lock = threading.Lock()

        def worker(worker_id: int) -> None:
            local = [
                hub.publish(_draft(worker_id * 25 + offset))
                for offset in range(25)
            ]
            with published_lock:
                published.extend(local)

        threads = [threading.Thread(target=worker, args=(index,)) for index in range(8)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

        ids = sorted(event.event_id for event in published)
        self.assertEqual(ids, list(range(1, 201)))
        self.assertEqual(hub.state_snapshot().watermark_event_id, 200)
        cursor = hub.open_cursor(0)
        self.assertEqual(
            tuple(event.event_id for event in cursor.read(limit=256).events),
            tuple(range(1, 201)),
        )

    def test_independent_cursors_read_the_same_events(self) -> None:
        hub = EventHub(_state(), capacity=8)
        first = hub.open_cursor(0)
        second = hub.open_cursor(0)
        for index in range(1, 4):
            hub.publish(_draft(index))

        first_events = first.read(limit=8).events
        second_events = second.read(limit=8).events
        self.assertEqual([event.event_id for event in first_events], [1, 2, 3])
        self.assertEqual([event.event_id for event in second_events], [1, 2, 3])

    def test_bootstrap_modes_and_runtime_gap_have_exact_semantics(self) -> None:
        hub = EventHub(_state(), capacity=2)
        fresh = hub.open_cursor(None)
        self.assertEqual(fresh.bootstrap.mode, "fresh")
        self.assertEqual(fresh.bootstrap.watermark_event_id, 0)
        lagging = hub.open_cursor(0)
        for index in range(1, 5):
            hub.publish(_draft(index))

        self.assertEqual(hub.open_cursor(3).bootstrap.mode, "resume")
        self.assertEqual(hub.open_cursor(99).bootstrap.mode, "reset")
        self.assertEqual(hub.open_cursor(0).bootstrap.mode, "reset")

        gap = lagging.read(limit=8)
        self.assertEqual(gap.events, ())
        self.assertEqual(gap.lost_event_ids, (1, 2))
        self.assertEqual(gap.watermark_event_id, 4)
        self.assertEqual(lagging.read(limit=8, timeout_s=0.0).events, ())

    def test_cursor_registry_does_not_retain_abandoned_cursors(self) -> None:
        hub = EventHub(_state())
        cursor = hub.open_cursor(None)
        reference = weakref.ref(cursor)
        self.assertEqual(hub.cursor_count(), 1)
        del cursor
        gc.collect()
        self.assertIsNone(reference())
        self.assertEqual(hub.cursor_count(), 0)

    def test_default_reducer_applies_full_snapshots_and_bounds_recent_attempts(self) -> None:
        hub = EventHub(_state(), capacity=256)
        changed_health = replace(_health(), pelvis_valid=False)
        hub.publish(
            _draft(
                1,
                kind="HEALTH_SNAPSHOT",
                payload=changed_health.to_json_dict(),
            )
        )
        hub.publish(
            _draft(
                2,
                kind="LIFECYCLE_SNAPSHOT",
                payload=_lifecycle(phase="RECOVERY").to_json_dict(),
            )
        )
        hub.publish(
            _draft(3, kind="ATTEMPT_CURRENT", payload=_summary(101).to_json_dict())
        )
        for attempt_id in range(1, 102):
            hub.publish(
                _draft(
                    attempt_id + 3,
                    kind="ATTEMPT_CLOSED",
                    attempt_id=attempt_id,
                    payload=_summary(attempt_id, status="FAILED").to_json_dict(),
                )
            )

        state = hub.state_snapshot()
        self.assertFalse(state.health.pelvis_valid)
        self.assertEqual(state.lifecycle.phase, "RECOVERY")
        self.assertEqual(len(state.recent_attempts), 100)
        self.assertEqual(state.recent_attempts[0].attempt_id, 101)
        self.assertEqual(state.recent_attempts[-1].attempt_id, 2)
        self.assertIsNone(state.current_attempt)
        json.dumps(state.to_json_dict(), allow_nan=False)

    def test_payload_is_detached_deeply_immutable_and_never_rewritten(self) -> None:
        hub = EventHub(_state(), capacity=4)
        source = {"nested": [{"value": 1}]}
        draft = _draft(1, payload=source)
        event = hub.publish(draft)
        source["nested"][0]["value"] = 99
        source["nested"].append({"value": 2})
        with self.assertRaises(TypeError):
            event.payload["other"] = 3
        with self.assertRaises(TypeError):
            event.payload["nested"][0]["value"] = 3
        self.assertEqual(event.payload["nested"][0]["value"], 1)

        mailbox = ProgressMailbox(capacity=4, min_interval_s=1.0)
        mailbox.offer(_draft(2), stage="ESTIMATING")
        mailbox.offer(_draft(3), stage="ESTIMATING")
        self.assertEqual(event.payload["nested"][0]["value"], 1)

    def test_oversize_and_malformed_drafts_become_small_drop_events_without_id_gaps(self) -> None:
        hub = EventHub(_state(), capacity=8, max_event_bytes=1024)
        first = hub.publish(_draft(1))
        oversize = hub.publish(_draft(2, payload={"blob": "球" * 1000}))
        malformed = hub.publish(
            _draft(3, kind="ATTEMPT_CURRENT", payload={"attempt_id": 1})
        )
        last = hub.publish(_draft(4))

        self.assertEqual([first.event_id, oversize.event_id, malformed.event_id, last.event_id], [1, 2, 3, 4])
        self.assertEqual(oversize.kind, "DIAGNOSTIC_EVENT_DROPPED")
        self.assertEqual(malformed.kind, "DIAGNOSTIC_EVENT_DROPPED")
        self.assertEqual(hub.state_snapshot().health.diagnostic_events_dropped, 2)
        json.dumps(oversize.to_json_dict(), allow_nan=False)

    def test_fallback_uses_only_bounded_canonical_values(self) -> None:
        hub = EventHub(_state(), max_event_bytes=1024)
        draft = EventDraft(
            kind="错" * 2000,
            monotonic_s=float("nan"),
            wall_time_us=10**100,
            scope="attempt",
            attempt_id=10**100,
            payload={"blob": "球" * 2000},
        )
        event = hub.publish(draft)
        encoded = json.dumps(
            event.to_json_dict(),
            ensure_ascii=False,
            allow_nan=False,
            separators=(",", ":"),
        ).encode("utf-8")
        self.assertEqual(event.kind, "DIAGNOSTIC_EVENT_DROPPED")
        self.assertEqual(event.monotonic_s, 0.0)
        self.assertTrue(math.isfinite(event.monotonic_s))
        self.assertEqual(event.wall_time_us, (1 << 63) - 1)
        self.assertIsNone(event.attempt_id)
        self.assertIsInstance(event.payload["original_attempt_id"], str)
        self.assertLessEqual(len(encoded), 1024)

    def test_reducer_failure_is_atomic_and_does_not_leave_an_id_hole(self) -> None:
        def reducer(state, event):
            if event.kind == "FAIL_REDUCER":
                raise RuntimeError("no update")
            return reduce_diagnostic_state(state, event)

        hub = EventHub(_state(), capacity=8, reducer=reducer)
        self.assertEqual(hub.publish(_draft(1)).event_id, 1)
        with self.assertRaisesRegex(RuntimeError, "no update"):
            hub.publish(_draft(2, kind="FAIL_REDUCER"))
        self.assertEqual(hub.state_snapshot().watermark_event_id, 1)
        self.assertEqual(hub.publish(_draft(3)).event_id, 2)

    def test_close_is_idempotent_wakes_blocked_read_and_rejects_later_use(self) -> None:
        hub = EventHub(_state())
        cursor = hub.open_cursor(None)
        started = threading.Event()
        finished = threading.Event()
        errors = []

        def blocked_reader() -> None:
            started.set()
            try:
                cursor.read()
            except RuntimeError as exc:
                errors.append(str(exc))
            finally:
                finished.set()

        thread = threading.Thread(target=blocked_reader)
        thread.start()
        self.assertTrue(started.wait(timeout=1.0))
        cursor.close()
        cursor.close()
        self.assertTrue(finished.wait(timeout=1.0))
        thread.join()
        self.assertEqual(errors, ["event cursor is closed"])
        with self.assertRaises(RuntimeError):
            cursor.read(timeout_s=0.0)

        hub.close()
        hub.close()
        with self.assertRaises(RuntimeError):
            hub.publish(_draft(1))
        with self.assertRaises(RuntimeError):
            hub.open_cursor(None)

    def test_hub_close_wakes_all_cursor_reads(self) -> None:
        hub = EventHub(_state())
        cursor = hub.open_cursor(None)
        started = threading.Event()
        finished = threading.Event()

        def blocked_reader() -> None:
            started.set()
            with self.assertRaises(RuntimeError):
                cursor.read()
            finished.set()

        thread = threading.Thread(target=blocked_reader)
        thread.start()
        self.assertTrue(started.wait(timeout=1.0))
        hub.close()
        self.assertTrue(finished.wait(timeout=1.0))
        thread.join()

    def test_ring_total_bytes_bound_evicts_old_events(self) -> None:
        hub = EventHub(
            _state(),
            capacity=10,
            max_event_bytes=1024,
            max_ring_bytes=1800,
        )
        cursor = hub.open_cursor(0)
        for index in range(1, 6):
            hub.publish(_draft(index, payload={"blob": "x" * 500}))
        gap = cursor.read(limit=10)
        self.assertEqual(gap.events, ())
        self.assertIsNotNone(gap.lost_event_ids)
        self.assertLess(hub.ring_bytes(), 1801)


class ProgressMailboxTests(unittest.TestCase):
    def test_rate_gate_coalesces_before_publish_and_keeps_latest_value(self) -> None:
        mailbox = ProgressMailbox(capacity=2, min_interval_s=1.0)
        first = _draft(0, payload={"progress": 1})
        middle = _draft(0, payload={"progress": 2})
        latest = EventDraft(
            kind="PROGRESS",
            monotonic_s=0.5,
            wall_time_us=3,
            scope="attempt",
            attempt_id=1,
            payload={"progress": 3},
        )

        self.assertEqual(mailbox.offer(first, stage="ESTIMATING"), first)
        self.assertIsNone(mailbox.offer(middle, stage="ESTIMATING"))
        self.assertIsNone(mailbox.offer(latest, stage="ESTIMATING"))
        self.assertEqual(mailbox.drain_ready(now_monotonic_s=0.999), ())
        ready = mailbox.drain_ready(now_monotonic_s=1.0)
        self.assertEqual(len(ready), 1)
        self.assertEqual(ready[0].payload["progress"], 3)

    def test_mailbox_pending_keys_are_bounded(self) -> None:
        mailbox = ProgressMailbox(capacity=2, min_interval_s=10.0)
        for attempt_id in (1, 2, 3):
            mailbox.offer(
                _draft(0, attempt_id=attempt_id),
                stage="ESTIMATING",
            )
            mailbox.offer(
                EventDraft(
                    kind="PROGRESS",
                    monotonic_s=1.0,
                    wall_time_us=1,
                    scope="attempt",
                    attempt_id=attempt_id,
                    payload={"attempt_id": attempt_id},
                ),
                stage="ESTIMATING",
            )
        ready = mailbox.drain_ready(now_monotonic_s=10.0)
        self.assertEqual([item.attempt_id for item in ready], [2, 3])

    def test_key_churn_evicts_last_emit_and_pending_as_one_entry(self) -> None:
        mailbox = ProgressMailbox(capacity=2, min_interval_s=10.0)
        mailbox.offer(_draft(0, attempt_id=1), stage="ESTIMATING")
        mailbox.offer(
            EventDraft(
                kind="PROGRESS",
                monotonic_s=1.0,
                wall_time_us=1,
                scope="attempt",
                attempt_id=1,
                payload={"progress": 2},
            ),
            stage="ESTIMATING",
        )
        mailbox.offer(_draft(0, attempt_id=2), stage="ESTIMATING")
        mailbox.offer(_draft(0, attempt_id=3), stage="ESTIMATING")
        self.assertEqual(mailbox.drain_ready(now_monotonic_s=2.0), ())
        self.assertEqual(mailbox.drain_ready(now_monotonic_s=20.0), ())


class EventPublisherLaneTests(unittest.TestCase):
    def test_offer_stays_nonblocking_while_hub_is_blocked_and_drop_is_coalesced(self) -> None:
        entered = threading.Event()
        release = threading.Event()

        def reducer(state, event):
            if event.kind == "BLOCK":
                entered.set()
                release.wait(timeout=1.0)
            return reduce_diagnostic_state(state, event)

        hub = EventHub(_state(), capacity=16, reducer=reducer)
        cursor = hub.open_cursor(0)
        lane = EventPublisherLane(hub, capacity=2)
        self.assertTrue(lane.offer(_draft(1, kind="BLOCK")))
        self.assertTrue(entered.wait(timeout=1.0))
        self.assertTrue(lane.offer(_draft(2)))
        self.assertTrue(lane.offer(_draft(3)))
        self.assertFalse(lane.offer(_draft(4)))
        self.assertFalse(lane.offer(_draft(5)))
        self.assertFalse(lane.offer(_draft(6)))
        release.set()
        self.assertTrue(lane.drain(timeout_s=1.0))
        self.assertTrue(lane.close(timeout_s=1.0))
        self.assertTrue(lane.close(timeout_s=1.0))
        self.assertFalse(lane.offer(_draft(7)))

        kinds = [event.kind for event in cursor.read(limit=16).events]
        self.assertEqual(
            kinds,
            [
                "BLOCK",
                "PROGRESS",
                "PROGRESS",
                "DIAGNOSTIC_EVENT_DROPPED",
            ],
        )
        self.assertEqual(hub.state_snapshot().health.diagnostic_events_dropped, 3)

    def test_closed_hub_does_not_make_owner_retry_forever(self) -> None:
        hub = EventHub(_state())
        lane = EventPublisherLane(hub, capacity=1)
        hub.close()
        self.assertTrue(lane.offer(_draft(1)))
        self.assertTrue(lane.drain(timeout_s=1.0))
        self.assertTrue(lane.close(timeout_s=1.0))


if __name__ == "__main__":
    unittest.main()
