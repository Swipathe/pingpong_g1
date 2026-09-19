from __future__ import annotations

import csv
from datetime import datetime
import json
import os
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest import mock

import numpy as np

from diagnostics import hitter_task_recording as recording
from diagnostics.hitter_task_events import DiagnosticState, EventHub
from diagnostics.hitter_task_models import (
    AttemptDetail,
    AttemptSummary,
    EventDraft,
    HealthSnapshot,
    LifecycleSnapshot,
    NormalizedMocapSample,
)
from diagnostics.hitter_task_recording import (
    AttemptDetailRepository,
    RawRecordLane,
    SessionRecorder,
    create_session_paths,
)


def _sample(sequence: int) -> NormalizedMocapSample:
    return NormalizedMocapSample(
        input_seq=sequence,
        channel="vicon_state_data",
        subject="ball",
        position_w=np.array([sequence + 0.125, 2.0, 3.0], dtype=np.float64),
        quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64),
        valid=True,
        occluded=False,
        source_frame=sequence,
        source_time_s=sequence / 360.0,
        publish_time_us=sequence * 1000,
        received_monotonic_s=sequence / 100.0,
        wall_time_us=sequence * 10000,
        payload_size=64,
    )


def _summary(
    attempt_id: int,
    *,
    ab_summary=None,
    recording_complete: bool = True,
) -> AttemptSummary:
    return AttemptSummary(
        attempt_id=attempt_id,
        status="FAILED",
        stage="ATTEMPT_CLOSED",
        primary_blocker="TRACK_ENDED_BEFORE_READY",
        ball_speed_mps=4.25,
        predicted_strike_time_s=None,
        planner_tts_s=None,
        arm_tts_s=None,
        task_obs_status="FAILED",
        ab_summary=ab_summary,
        recording_complete=recording_complete,
    )


def _detail(
    attempt_id: int,
    *,
    ab_summary=None,
    recording_complete: bool = True,
) -> AttemptDetail:
    return AttemptDetail(
        attempt_id=attempt_id,
        summary=_summary(
            attempt_id,
            ab_summary=ab_summary,
            recording_complete=recording_complete,
        ),
        segments=({"track_segment_id": attempt_id},),
        stage_timeline=(),
        planner_inputs=(),
        planner_results=(),
        task_observation_pre_clip=None,
        task_observation_post_clip=None,
        task_observation_clip_count=None,
        variant_outcomes=(),
        ab_deltas={},
    )


def _hub(capacity: int = 1024) -> EventHub:
    health = HealthSnapshot(
        lcm_connected=True,
        message_rate_hz_by_subject={},
        message_age_s_by_subject={},
        source_frame_by_subject={},
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
        session_basename="test",
        warnings=(),
    )
    lifecycle = LifecycleSnapshot(
        phase="TRACKING",
        last_decision="NONE",
        active_key=None,
        cached_key=None,
        lifecycle_now_s=None,
        obs_now_s=None,
    )
    return EventHub(
        DiagnosticState(1, 0, health, lifecycle, None, ()),
        capacity=capacity,
    )


class RawRecordLaneTests(unittest.TestCase):
    def test_envelopes_are_bounded_and_drop_ranges_end_on_recovery_and_close(self) -> None:
        lane = RawRecordLane(capacity=1)
        self.assertTrue(
            lane.submit(_sample(1), attempt_id=7, track_segment_id=3).accepted
        )
        second = lane.submit(_sample(2), attempt_id=7, track_segment_id=3)
        self.assertEqual(lane.take_drop_ranges(), ((2, 2, 7),))
        third = lane.submit(_sample(3), attempt_id=7, track_segment_id=3)
        self.assertFalse(second.accepted)
        self.assertEqual(second.reason, "FULL")
        self.assertEqual(third.dropped_input_seq, (2, 3))
        envelope = lane.take_many(limit=1)[0]
        self.assertEqual(envelope.attempt_id, 7)
        self.assertEqual(envelope.track_segment_id, 3)
        recovered = lane.submit(_sample(4), attempt_id=8, track_segment_id=4)
        self.assertTrue(recovered.accepted)
        self.assertEqual(recovered.dropped_input_seq, (2, 3))
        self.assertEqual(recovered.attempt_id, 7)

        lane.take_many(limit=1)
        lane.submit(_sample(5), attempt_id=9)
        lane.submit(_sample(6), attempt_id=9)
        lane.close()
        self.assertIn((6, 6, 9), lane.take_drop_ranges())
        closed = lane.submit(_sample(7), attempt_id=9)
        self.assertFalse(closed.accepted)
        self.assertEqual(closed.reason, "CLOSED")
        self.assertEqual(lane.take_many(limit=1)[0].sample.input_seq, 5)

    def test_non_contiguous_drop_metadata_overflow_is_explicit(self) -> None:
        lane = RawRecordLane(capacity=1, drop_metadata_capacity=2)
        lane.submit(_sample(1), attempt_id=1)
        lane.submit(_sample(2), attempt_id=2)
        lane.submit(_sample(4), attempt_id=3)
        lane.submit(_sample(6), attempt_id=4)

        notices = lane.take_drop_ranges()

        self.assertIn((2, 2, 2), notices)
        self.assertIn((4, 4, 3), notices)
        self.assertIn("METADATA_OVERFLOW", notices)


class SessionPathAndRepositoryTests(unittest.TestCase):
    def test_session_paths_are_exclusive_and_files_are_private(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            output_root = Path(temp_dir)
            paths = create_session_paths(
                output_root,
                now=datetime(2026, 7, 27, 12, 34, 56, 123456),
                pid=42,
                short_uuid="abc123",
            )
            self.assertEqual(
                paths.root.name,
                "20260727_123456_123456-p42-abc123",
            )
            self.assertEqual(paths.root.stat().st_mode & 0o777, 0o700)
            self.assertEqual(
                paths.attempt_details_dir.stat().st_mode & 0o777,
                0o700,
            )
            with self.assertRaises(FileExistsError):
                create_session_paths(
                    output_root,
                    now=datetime(2026, 7, 27, 12, 34, 56, 123456),
                    pid=42,
                    short_uuid="abc123",
                )
            for invalid_suffix in ("", ".", "..", "../escape", "slash/name"):
                with self.subTest(invalid_suffix=invalid_suffix):
                    with self.assertRaises(ValueError):
                        create_session_paths(
                            output_root,
                            now=datetime(2026, 7, 27, 12, 34, 56, 123456),
                            pid=42,
                            short_uuid=invalid_suffix,
                        )

            hub = _hub()
            repository = AttemptDetailRepository(paths.attempt_details_dir)
            recorder = SessionRecorder(
                paths=paths,
                raw_lane=RawRecordLane(),
                event_cursor=hub.open_cursor(None),
                attempt_repository=repository,
                session_metadata={"config_name": "hitter"},
            )
            recorder.start()
            running = json.loads(paths.session_json.read_text())
            self.assertEqual(running["status"], "RUNNING")
            for path in (
                paths.session_json,
                paths.ball_samples_csv,
                paths.events_jsonl,
                paths.replay_jobs_jsonl,
                paths.replay_analysis_jsonl,
                paths.attempts_csv,
            ):
                self.assertEqual(path.stat().st_mode & 0o777, 0o600)
            recorder.write_terminal_and_close(
                terminal_session={"reason": "test"},
                timeout_s=0.1,
            )

    def test_detail_repository_uses_safe_ids_lru_and_descending_pagination(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            details_dir = Path(temp_dir) / "details"
            details_dir.mkdir(mode=0o700)
            repository = AttemptDetailRepository(details_dir, cache_size=2)
            for attempt_id in (1, 2, 3):
                repository.commit(_detail(attempt_id, ab_summary="BOTH_FAIL_SAME"))
            self.assertEqual(repository.get(2).attempt_id, 2)
            self.assertEqual(
                [item.attempt_id for item in repository.list_page(limit=2).items],
                [3, 2],
            )
            page = repository.list_page(limit=2, before=3)
            self.assertEqual([item.attempt_id for item in page.items], [2, 1])
            self.assertFalse(page.has_more)
            reopened = AttemptDetailRepository(details_dir, cache_size=1)
            (details_dir / "01.json").write_text("{}")
            (details_dir / "not-an-attempt.json").write_text("{}")
            self.assertEqual(
                [
                    item.attempt_id
                    for item in reopened.list_page(limit=2).items
                ],
                [3, 2],
            )
            self.assertEqual(reopened.get(1).attempt_id, 1)
            for invalid in (True, 0, -1, "../1"):
                with self.subTest(invalid=invalid):
                    with self.assertRaises((TypeError, ValueError)):
                        reopened.get(invalid)
            payload = json.loads((details_dir / "1.json").read_text())
            self.assertEqual(payload["schema_version"], 1)
            self.assertEqual((details_dir / "1.json").stat().st_mode & 0o777, 0o600)

    def test_detail_repository_never_follows_numeric_symlinks(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            details_dir = root / "details"
            details_dir.mkdir(mode=0o700)
            external = root / "external.json"
            external.write_text(
                json.dumps(_detail(7).to_json_dict()),
                encoding="utf-8",
            )
            (details_dir / "7.json").symlink_to(external)
            repository = AttemptDetailRepository(details_dir)

            self.assertIsNone(repository.get(7))
            self.assertEqual(repository.list_page(limit=10).items, ())

    def test_open_private_closes_descriptor_when_fdopen_fails(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            captured = []
            real_open = os.open

            def capture_open(*args, **kwargs):
                descriptor = real_open(*args, **kwargs)
                captured.append(descriptor)
                return descriptor

            with mock.patch.object(
                recording.os,
                "open",
                side_effect=capture_open,
            ), mock.patch.object(
                recording.os,
                "fdopen",
                side_effect=OSError("fdopen failed"),
            ):
                with self.assertRaises(OSError):
                    recording._open_private(Path(temp_dir) / "private.csv")

            self.assertEqual(len(captured), 1)
            with self.assertRaises(OSError):
                os.fstat(captured[0])

    def test_atomic_json_closes_descriptor_when_fdopen_fails(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            captured = []
            real_open = os.open

            def capture_open(*args, **kwargs):
                descriptor = real_open(*args, **kwargs)
                captured.append(descriptor)
                return descriptor

            with mock.patch.object(
                recording.os,
                "open",
                side_effect=capture_open,
            ), mock.patch.object(
                recording.os,
                "fdopen",
                side_effect=OSError("fdopen failed"),
            ):
                with self.assertRaises(OSError):
                    recording._atomic_json(
                        Path(temp_dir) / "atomic.json",
                        {"status": "COMPLETE"},
                    )

            self.assertEqual(len(captured), 1)
            try:
                with self.assertRaises(OSError):
                    os.fstat(captured[0])
            finally:
                try:
                    os.close(captured[0])
                except OSError:
                    pass

    def test_detail_commit_stops_after_fsync_crosses_deadline(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            details_dir = Path(temp_dir) / "details"
            repository = AttemptDetailRepository(details_dir)
            now = [0.0]
            real_fsync = os.fsync

            def expire_after_fsync(descriptor):
                real_fsync(descriptor)
                now[0] = 1.0

            with mock.patch.object(
                recording.time,
                "monotonic",
                side_effect=lambda: now[0],
            ), mock.patch.object(
                recording.os,
                "fsync",
                side_effect=expire_after_fsync,
            ), mock.patch.object(
                recording.os,
                "replace",
                wraps=os.replace,
            ) as replace:
                try:
                    committed = repository.commit(
                        _detail(1),
                        deadline_s=0.5,
                    )
                except TypeError as exc:
                    self.fail(
                        "commit must accept and enforce deadline_s: {}".format(
                            exc
                        )
                    )

            self.assertFalse(committed)
            replace.assert_not_called()
            self.assertEqual(list(details_dir.iterdir()), [])
            self.assertIsNone(repository.get(1))

    def test_detail_commit_fails_if_directory_fsync_crosses_deadline(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            details_dir = Path(temp_dir) / "details"
            repository = AttemptDetailRepository(details_dir)
            now = [0.0]
            fsync_calls = [0]
            real_fsync = os.fsync

            def expire_after_directory_fsync(descriptor):
                real_fsync(descriptor)
                fsync_calls[0] += 1
                if fsync_calls[0] == 2:
                    now[0] = 1.0

            with mock.patch.object(
                recording.time,
                "monotonic",
                side_effect=lambda: now[0],
            ), mock.patch.object(
                recording.os,
                "fsync",
                side_effect=expire_after_directory_fsync,
            ):
                committed = repository.commit(
                    _detail(1),
                    deadline_s=0.5,
                )

            self.assertFalse(committed)
            self.assertEqual(fsync_calls[0], 4)
            self.assertNotIn(1, repository._cache)
            self.assertTrue((details_dir / "1.json").is_file())
            recovered = repository.get(1)
            self.assertIsNotNone(recovered)
            self.assertEqual(
                recovered.summary.ab_summary,
                "INCONCLUSIVE",
            )
            self.assertFalse(recovered.summary.recording_complete)


class SessionRecorderTests(unittest.TestCase):
    def test_public_mark_incomplete_forces_attempt_ab_inconclusive(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder, paths, repository, hub, _raw_lane = self._recorder(
                Path(temp_dir)
            )
            recorder.start()
            recorder.offer_attempt_detail(
                _detail(17, ab_summary="SAME_PASS")
            )

            recorder.mark_incomplete(17, "INPUT_SAMPLES_DROPPED")
            recorder.drain(timeout_s=0.1)
            status = recorder.write_terminal_and_close(
                terminal_session={},
                timeout_s=1.0,
            )

            detail = repository.get(17)
            self.assertIsNotNone(detail)
            self.assertEqual(detail.summary.ab_summary, "INCONCLUSIVE")
            self.assertFalse(detail.summary.recording_complete)
            self.assertFalse(status.recording_complete)
            with paths.attempts_csv.open("r", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(rows[0]["ab_summary"], "INCONCLUSIVE")
            self.assertEqual(rows[0]["recording_complete"], "False")
            hub.close()

    def _recorder(self, root: Path, *, hub=None, raw_lane=None, **kwargs):
        hub = _hub() if hub is None else hub
        raw_lane = RawRecordLane() if raw_lane is None else raw_lane
        paths = create_session_paths(root)
        repository = AttemptDetailRepository(paths.attempt_details_dir)
        recorder = SessionRecorder(
            paths=paths,
            raw_lane=raw_lane,
            event_cursor=hub.open_cursor(0),
            attempt_repository=repository,
            session_metadata={"config_name": "hitter"},
            **kwargs,
        )
        recorder.start()
        return recorder, paths, repository, hub, raw_lane

    def test_one_drain_round_is_fair_at_512_raw_and_128_events(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            hub = _hub(capacity=1024)
            raw_lane = RawRecordLane(capacity=700)
            for sequence in range(1, 601):
                raw_lane.submit(_sample(sequence), attempt_id=1, track_segment_id=1)
            for sequence in range(1, 131):
                hub.publish(
                    EventDraft(
                        kind="PROGRESS",
                        monotonic_s=float(sequence),
                        wall_time_us=sequence,
                        scope="state",
                        attempt_id=None,
                        payload={"sequence": sequence},
                    )
                )
            recorder, paths, _, _, _ = self._recorder(
                Path(temp_dir),
                hub=hub,
                raw_lane=raw_lane,
                flush_row_count=1,
            )
            recorder.drain(timeout_s=0.0)
            with paths.ball_samples_csv.open() as stream:
                raw_rows = list(csv.DictReader(stream))
            event_rows = paths.events_jsonl.read_text().splitlines()
            self.assertEqual(len(raw_rows), 512)
            self.assertEqual(len(event_rows), 128)
            self.assertEqual(
                float(json.loads(raw_rows[0]["position_w"])[0]),
                _sample(1).position_w[0],
            )
            recorder.write_terminal_and_close(
                terminal_session={},
                timeout_s=0.2,
            )

    def test_raw_drop_and_event_gap_make_pending_attempt_inconclusive(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            hub = _hub(capacity=2)
            raw_lane = RawRecordLane(capacity=1)
            recorder, paths, _, _, _ = self._recorder(
                Path(temp_dir),
                hub=hub,
                raw_lane=raw_lane,
                event_sink=hub.publish,
                flush_row_count=1,
            )
            recorder.offer_attempt_detail(_detail(7))
            raw_lane.submit(_sample(1), attempt_id=7, track_segment_id=1)
            raw_lane.submit(_sample(2), attempt_id=7, track_segment_id=1)
            raw_lane.take_many(limit=1)
            raw_lane.submit(_sample(3), attempt_id=7, track_segment_id=1)
            for sequence in range(4):
                hub.publish(
                    EventDraft(
                        kind="PROGRESS",
                        monotonic_s=float(sequence),
                        wall_time_us=sequence,
                        scope="state",
                        attempt_id=None,
                        payload={},
                    )
                )
            status = recorder.drain(timeout_s=0.0)
            self.assertFalse(status.recording_complete)
            recorder.write_terminal_and_close(
                terminal_session={},
                timeout_s=0.2,
            )
            with paths.attempts_csv.open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["attempt_id"], "7")
            self.assertEqual(rows[0]["ab_summary"], "INCONCLUSIVE")
            self.assertEqual(rows[0]["recording_complete"], "False")
            markers = [
                json.loads(line)
                for line in paths.events_jsonl.read_text().splitlines()
                if json.loads(line)["kind"] == "RECORDER_EVENT_GAP"
            ]
            self.assertEqual(len(markers), 1)
            self.assertEqual(markers[0]["payload"]["lost_event_ids"], [1, 3])
            self.assertEqual(markers[0]["payload"]["watermark_event_id"], 5)

    def test_final_attempt_writes_once_and_pending_shutdown_writes_inconclusive(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder, paths, _, _, _ = self._recorder(Path(temp_dir))
            complete = _detail(1, ab_summary="BOTH_FAIL_SAME")
            recorder.offer_attempt_detail(complete)
            recorder.offer_attempt_detail(complete)
            recorder.offer_attempt_detail(_detail(2))
            recorder.offer_replay_analysis(
                {"attempt_id": 1, "summary_label": "BOTH_FAIL_SAME"}
            )
            recorder.offer_replay_job_status(
                {"attempt_id": 1, "status": "COMPLETE"}
            )
            recorder.drain(timeout_s=0.0)
            recorder.write_terminal_and_close(
                terminal_session={"reason": "done"},
                timeout_s=0.2,
            )
            with paths.attempts_csv.open() as stream:
                rows = list(csv.DictReader(stream))
            rows_by_id = {row["attempt_id"]: row for row in rows}
            self.assertEqual(set(rows_by_id), {"1", "2"})
            self.assertEqual(
                rows_by_id["1"]["ab_summary"],
                "BOTH_FAIL_SAME",
            )
            self.assertEqual(
                rows_by_id["2"]["ab_summary"],
                "INCONCLUSIVE",
            )
            self.assertEqual(
                len(paths.replay_analysis_jsonl.read_text().splitlines()),
                1,
            )
            self.assertEqual(
                len(paths.replay_jobs_jsonl.read_text().splitlines()),
                1,
            )
            terminal = json.loads(paths.session_json.read_text())
            self.assertIn(terminal["status"], ("COMPLETE", "INCOMPLETE"))

    def test_offer_queues_are_bounded_and_close_is_idempotent(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder, _, _, _, _ = self._recorder(
                Path(temp_dir),
                offer_capacity=1,
            )
            self.assertTrue(recorder.offer_attempt_detail(_detail(1)))
            self.assertFalse(recorder.offer_attempt_detail(_detail(2)))
            self.assertTrue(recorder.offer_replay_analysis({"attempt_id": 1}))
            self.assertFalse(recorder.offer_replay_analysis({"attempt_id": 2}))
            self.assertTrue(
                recorder.offer_replay_job_status(
                    {"attempt_id": 1, "status": "PENDING"}
                )
            )
            self.assertFalse(
                recorder.offer_replay_job_status(
                    {"attempt_id": 2, "status": "PENDING"}
                )
            )
            first = recorder.write_terminal_and_close(
                terminal_session={},
                timeout_s=0.2,
            )
            second = recorder.write_terminal_and_close(
                terminal_session={},
                timeout_s=0.2,
            )
            self.assertEqual(first, second)
            self.assertFalse(first.recording_complete)

    def test_late_raw_drop_revises_completed_attempt_before_first_csv_row(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            raw_lane = RawRecordLane(capacity=1)
            recorder, paths, repository, _, _ = self._recorder(
                Path(temp_dir),
                raw_lane=raw_lane,
                flush_row_count=1,
            )
            recorder.offer_attempt_detail(
                _detail(7, ab_summary="BOTH_FAIL_SAME")
            )
            recorder.drain(timeout_s=0.0)
            raw_lane.submit(_sample(1), attempt_id=7)
            raw_lane.submit(_sample(2), attempt_id=7)

            status = recorder.write_terminal_and_close(
                terminal_session={},
                timeout_s=0.2,
            )

            self.assertFalse(status.recording_complete)
            detail = repository.get(7)
            self.assertEqual(detail.summary.ab_summary, "INCONCLUSIVE")
            self.assertFalse(detail.summary.recording_complete)
            with paths.attempts_csv.open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["ab_summary"], "INCONCLUSIVE")
            self.assertEqual(rows[0]["recording_complete"], "False")

    def test_terminal_timeout_records_only_persisted_event_watermark(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            hub = _hub(capacity=512)
            for sequence in range(1, 301):
                hub.publish(
                    EventDraft(
                        kind="PROGRESS",
                        monotonic_s=float(sequence),
                        wall_time_us=sequence,
                        scope="state",
                        attempt_id=None,
                        payload={"sequence": sequence},
                    )
                )
            recorder, paths, _, _, _ = self._recorder(
                Path(temp_dir),
                hub=hub,
                flush_row_count=1,
            )

            status = recorder.write_terminal_and_close(
                terminal_session={},
                timeout_s=0.0,
            )

            running = json.loads(paths.session_json.read_text())
            self.assertFalse(status.recording_complete)
            self.assertIn("SHUTDOWN_DEADLINE_EXPIRED", status.last_error)
            self.assertEqual(running["status"], "RUNNING")
            self.assertEqual(
                len(paths.events_jsonl.read_text().splitlines()),
                0,
            )

    def test_expired_terminal_close_discards_buffered_raw_rows(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            raw_lane = RawRecordLane()
            recorder, paths, _, _, _ = self._recorder(
                Path(temp_dir),
                raw_lane=raw_lane,
                flush_interval_s=3600.0,
                flush_row_count=1000,
            )
            baseline = paths.ball_samples_csv.stat().st_size
            raw_lane.submit(_sample(1), attempt_id=1)
            recorder.drain(timeout_s=0.0)
            self.assertEqual(paths.ball_samples_csv.stat().st_size, baseline)

            status = recorder.write_terminal_and_close(
                terminal_session={},
                timeout_s=0.0,
            )

            self.assertFalse(status.recording_complete)
            self.assertEqual(paths.ball_samples_csv.stat().st_size, baseline)

    def test_drop_metadata_overflow_marks_whole_session_incomplete(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            raw_lane = RawRecordLane(
                capacity=1,
                drop_metadata_capacity=1,
            )
            raw_lane.submit(_sample(1), attempt_id=1)
            raw_lane.submit(_sample(2), attempt_id=2)
            raw_lane.submit(_sample(4), attempt_id=3)
            raw_lane.take_many(limit=1)
            recorder, _, _, _, _ = self._recorder(
                Path(temp_dir),
                raw_lane=raw_lane,
            )

            status = recorder.write_terminal_and_close(
                terminal_session={},
                timeout_s=0.0,
            )

            self.assertFalse(status.recording_complete)
            self.assertEqual(
                status.last_error,
                "RAW_DROP_METADATA_OVERFLOW",
            )

    def test_terminal_timeout_marks_attempt_incomplete_before_finalizing(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            raw_lane = RawRecordLane(capacity=600)
            for sequence in range(1, 514):
                raw_lane.submit(_sample(sequence), attempt_id=9)
            recorder, paths, repository, _, _ = self._recorder(
                Path(temp_dir),
                raw_lane=raw_lane,
                flush_row_count=1,
            )
            recorder.offer_attempt_detail(
                _detail(9, ab_summary="BOTH_FAIL_SAME")
            )

            status = recorder.write_terminal_and_close(
                terminal_session={},
                timeout_s=0.0,
            )

            self.assertFalse(status.recording_complete)
            self.assertIn("SHUTDOWN_DEADLINE_EXPIRED", status.last_error)
            queued = recorder._detail_offers[0]
            self.assertEqual(queued.summary.ab_summary, "INCONCLUSIVE")
            self.assertFalse(queued.summary.recording_complete)
            self.assertIsNone(repository.get(9))
            with paths.attempts_csv.open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(rows, [])

    def test_zero_terminal_deadline_skips_large_history_and_marks_incomplete(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder, paths, repository, _, _ = self._recorder(
                Path(temp_dir),
                offer_capacity=256,
            )
            recorder.offer_attempt_detail(_detail(1))
            recorder.drain(timeout_s=0.0)
            for attempt_id in range(2, 152):
                repository.commit(
                    _detail(
                        attempt_id,
                        ab_summary="BOTH_FAIL_SAME",
                    )
                )

            started = time.monotonic()
            status = recorder.write_terminal_and_close(
                terminal_session={},
                timeout_s=0.0,
            )
            elapsed = time.monotonic() - started

            self.assertLess(elapsed, 0.25)
            self.assertFalse(status.recording_complete)
            self.assertIn("SHUTDOWN_DEADLINE_EXPIRED", status.last_error)
            pending = recorder._pending_details[1]
            self.assertEqual(pending.summary.ab_summary, "INCONCLUSIVE")
            self.assertFalse(pending.summary.recording_complete)
            self.assertEqual(
                json.loads(paths.session_json.read_text())["status"],
                "RUNNING",
            )

    def test_terminal_checks_absolute_deadline_before_each_detail_io(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder, _, repository, _, _ = self._recorder(
                Path(temp_dir),
                offer_capacity=64,
            )
            for attempt_id in range(100, 120):
                repository.commit(
                    _detail(
                        attempt_id,
                        ab_summary="BOTH_FAIL_SAME",
                    )
                )
            for attempt_id in range(1, 11):
                self.assertTrue(
                    recorder.offer_attempt_detail(_detail(attempt_id))
                )

            now = [0.0]
            commit_calls = []
            real_commit = repository.commit

            def advance_deadline(detail, **kwargs):
                commit_calls.append(detail.attempt_id)
                committed = real_commit(detail, **kwargs)
                now[0] = 1.0
                return committed

            with mock.patch.object(
                recording.time,
                "monotonic",
                side_effect=lambda: now[0],
            ), mock.patch.object(
                repository,
                "commit",
                side_effect=advance_deadline,
            ):
                status = recorder.write_terminal_and_close(
                    terminal_session={},
                    timeout_s=0.5,
                )

            self.assertEqual(commit_calls, [1])
            self.assertFalse(status.recording_complete)
            self.assertIn("SHUTDOWN_DEADLINE_EXPIRED", status.last_error)
            self.assertEqual(
                recorder._pending_details[1].summary.ab_summary,
                "INCONCLUSIVE",
            )
            self.assertFalse(
                recorder._pending_details[1].summary.recording_complete
            )

    def test_attempt_row_crossing_deadline_is_discarded_and_inconclusive(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder, paths, _, _, _ = self._recorder(Path(temp_dir))
            recorder.offer_attempt_detail(
                _detail(1, ab_summary="BOTH_FAIL_SAME")
            )
            recorder.drain(timeout_s=0.0)
            baseline = paths.attempts_csv.stat().st_size
            now = [0.0]
            real_writerow = recorder._attempt_writer.writerow

            def expire_after_writerow(row):
                result = real_writerow(row)
                now[0] = 1.0
                return result

            with mock.patch.object(
                recording.time,
                "monotonic",
                side_effect=lambda: now[0],
            ), mock.patch.object(
                recorder._attempt_writer,
                "writerow",
                side_effect=expire_after_writerow,
            ):
                status = recorder.write_terminal_and_close(
                    terminal_session={},
                    timeout_s=0.5,
                )

            self.assertFalse(status.recording_complete)
            self.assertEqual(paths.attempts_csv.stat().st_size, baseline)
            with paths.attempts_csv.open() as stream:
                self.assertEqual(list(csv.DictReader(stream)), [])
            pending = recorder._pending_details[1]
            self.assertEqual(pending.summary.ab_summary, "INCONCLUSIVE")
            self.assertFalse(pending.summary.recording_complete)

    def test_attempt_fsync_crossing_deadline_rolls_back_terminal_truth(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder, paths, repository, hub, _ = self._recorder(
                Path(temp_dir)
            )
            recorder.offer_attempt_detail(
                _detail(1, ab_summary="BOTH_FAIL_SAME")
            )
            recorder.drain(timeout_s=0.0)
            attempts_fd = recorder._files["attempts"].fileno()
            now = [0.0]
            real_fsync = os.fsync

            def expire_after_attempt_fsync(descriptor):
                real_fsync(descriptor)
                if descriptor == attempts_fd:
                    now[0] = 1.0

            with mock.patch.object(
                recording.time,
                "monotonic",
                side_effect=lambda: now[0],
            ), mock.patch.object(
                recording.os,
                "fsync",
                side_effect=expire_after_attempt_fsync,
            ):
                status = recorder.write_terminal_and_close(
                    terminal_session={},
                    timeout_s=0.5,
                )

            self.assertFalse(status.recording_complete)
            self.assertIn(1, recorder._pending_details)
            pending = recorder._pending_details[1]
            self.assertEqual(pending.summary.ab_summary, "INCONCLUSIVE")
            self.assertFalse(pending.summary.recording_complete)
            detail = repository.get(1)
            self.assertEqual(detail.summary.ab_summary, "INCONCLUSIVE")
            self.assertFalse(detail.summary.recording_complete)
            with paths.attempts_csv.open() as stream:
                self.assertEqual(list(csv.DictReader(stream)), [])
            hub.close()

    def test_directory_fsync_timeout_keeps_offer_fail_closed_inconclusive(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder, paths, repository, hub, _ = self._recorder(
                Path(temp_dir)
            )
            recorder.offer_attempt_detail(
                _detail(1, ab_summary="BOTH_FAIL_SAME")
            )
            now = [0.0]
            fsync_calls = [0]
            real_fsync = os.fsync

            def expire_after_directory_fsync(descriptor):
                real_fsync(descriptor)
                fsync_calls[0] += 1
                if fsync_calls[0] == 2:
                    now[0] = 1.0

            with mock.patch.object(
                recording.time,
                "monotonic",
                side_effect=lambda: now[0],
            ), mock.patch.object(
                recording.os,
                "fsync",
                side_effect=expire_after_directory_fsync,
            ):
                status = recorder.write_terminal_and_close(
                    terminal_session={},
                    timeout_s=0.5,
                )

            self.assertFalse(status.recording_complete)
            self.assertEqual(fsync_calls[0], 4)
            self.assertEqual(len(recorder._detail_offers), 1)
            queued = recorder._detail_offers[0]
            self.assertEqual(queued.summary.ab_summary, "INCONCLUSIVE")
            self.assertFalse(queued.summary.recording_complete)
            self.assertNotIn(1, recorder._pending_details)
            disk = json.loads(
                (paths.attempt_details_dir / "1.json").read_text()
            )
            self.assertEqual(
                disk["summary"]["ab_summary"],
                "INCONCLUSIVE",
            )
            self.assertFalse(
                disk["summary"]["recording_complete"]
            )
            served = repository.get(1)
            self.assertIsNotNone(served)
            self.assertEqual(
                served.summary.ab_summary,
                "INCONCLUSIVE",
            )
            self.assertFalse(served.summary.recording_complete)
            hub.close()

    def test_identical_shared_detail_is_not_physically_recommitted(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder, _, repository, hub, _ = self._recorder(
                Path(temp_dir)
            )
            detail = _detail(1, ab_summary="BOTH_FAIL_SAME")
            self.assertTrue(repository.commit(detail))
            self.assertTrue(recorder.offer_attempt_detail(detail))

            with mock.patch.object(
                repository,
                "commit",
                wraps=repository.commit,
            ) as commit:
                recorder.drain(timeout_s=0.0)

            commit.assert_not_called()
            self.assertEqual(
                recorder._pending_details[1],
                detail,
            )
            recorder.write_terminal_and_close(
                terminal_session={},
                timeout_s=1.0,
            )
            hub.close()

    def test_confirm_replay_persistence_drains_entry_snapshot_and_fsyncs(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder, paths, _, hub, _ = self._recorder(Path(temp_dir))
            published = hub.publish(
                EventDraft(
                    kind="replay_worker_status",
                    monotonic_s=1.0,
                    wall_time_us=2,
                    scope="attempt",
                    attempt_id=1,
                    payload={"status": "COMPLETED"},
                )
            )
            self.assertTrue(
                recorder.offer_replay_job_status(
                    {
                        "attempt_id": 1,
                        "status": "COMPLETED",
                    }
                )
            )
            self.assertTrue(
                recorder.offer_replay_analysis(
                    {
                        "attempt_id": 1,
                        "result": {"summary_label": "SAME_PASS"},
                    }
                )
            )

            status = recorder.confirm_replay_persistence(timeout_s=1.0)

            self.assertTrue(status.healthy)
            self.assertTrue(status.recording_complete)
            self.assertEqual(
                json.loads(paths.events_jsonl.read_text())["event_id"],
                published.event_id,
            )
            self.assertEqual(
                json.loads(paths.replay_jobs_jsonl.read_text())["status"],
                "COMPLETED",
            )
            self.assertEqual(
                json.loads(
                    paths.replay_analysis_jsonl.read_text()
                )["result"]["summary_label"],
                "SAME_PASS",
            )
            recorder.write_terminal_and_close(
                terminal_session={},
                timeout_s=1.0,
            )
            hub.close()

    def test_confirm_replay_persistence_fails_closed_on_entry_backlog(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder, _, _, hub, _ = self._recorder(Path(temp_dir))
            self.assertTrue(
                recorder.offer_replay_job_status(
                    {
                        "attempt_id": 1,
                        "status": "COMPLETED",
                    }
                )
            )

            status = recorder.confirm_replay_persistence(timeout_s=0.0)

            self.assertFalse(status.recording_complete)
            self.assertIn(
                "REPLAY_PERSISTENCE_DEADLINE",
                status.last_error,
            )
            self.assertEqual(len(recorder._job_offers), 1)
            recorder.write_terminal_and_close(
                terminal_session={},
                timeout_s=1.0,
            )
            hub.close()

    def test_confirm_replay_persistence_fails_closed_on_write_error(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder, _, _, hub, _ = self._recorder(Path(temp_dir))
            self.assertTrue(
                recorder.offer_replay_analysis(
                    {
                        "attempt_id": 1,
                        "result": {"summary_label": "SAME_PASS"},
                    }
                )
            )
            analysis_stream = recorder._files["analysis"]
            real_write = recorder._write_json_line

            def fail_analysis(stream, value):
                if stream is analysis_stream:
                    raise OSError("synthetic replay analysis failure")
                return real_write(stream, value)

            with mock.patch.object(
                recorder,
                "_write_json_line",
                side_effect=fail_analysis,
            ):
                status = recorder.confirm_replay_persistence(
                    timeout_s=1.0
                )

            self.assertFalse(status.healthy)
            self.assertFalse(status.recording_complete)
            self.assertIn(
                "synthetic replay analysis failure",
                status.last_error,
            )
            recorder.write_terminal_and_close(
                terminal_session={},
                timeout_s=1.0,
            )
            hub.close()

    def test_nonterminal_drain_consumes_only_entry_offer_snapshot(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder, _, _, _, _ = self._recorder(Path(temp_dir))
            recorder.offer_attempt_detail(
                _detail(1, ab_summary="BOTH_FAIL_SAME")
            )
            consumed = []
            real_consume = recorder._consume_detail

            def consume_and_replenish(detail):
                real_consume(detail)
                consumed.append(detail.attempt_id)
                if detail.attempt_id == 1:
                    recorder.offer_attempt_detail(
                        _detail(2, ab_summary="BOTH_FAIL_SAME")
                    )

            try:
                with mock.patch.object(
                    recorder,
                    "_consume_detail",
                    side_effect=consume_and_replenish,
                ):
                    recorder.drain(timeout_s=0.0)

                self.assertEqual(consumed, [1])
                self.assertEqual(
                    [detail.attempt_id for detail in recorder._detail_offers],
                    [2],
                )
            finally:
                recorder.write_terminal_and_close(
                    terminal_session={},
                    timeout_s=1.0,
                )

    def test_timed_drain_uses_one_snapshot_across_all_producer_lanes(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            hub = _hub(capacity=1024)
            raw_lane = RawRecordLane(capacity=1024)
            recorder, _, _, _, _ = self._recorder(
                Path(temp_dir),
                hub=hub,
                raw_lane=raw_lane,
                flush_interval_s=3600.0,
                flush_row_count=100000,
            )
            for sequence in range(1, 4):
                raw_lane.submit(_sample(sequence), attempt_id=sequence)
                hub.publish(
                    EventDraft(
                        kind="PROGRESS",
                        monotonic_s=float(sequence),
                        wall_time_us=sequence,
                        scope="state",
                        attempt_id=None,
                        payload={"sequence": sequence},
                    )
                )
                recorder.offer_attempt_detail(
                    _detail(sequence, ab_summary="BOTH_FAIL_SAME")
                )
                recorder.offer_replay_analysis({"sequence": sequence})
                recorder.offer_replay_job_status({"sequence": sequence})

            consumed = {
                "raw": [],
                "event": [],
                "detail": [],
                "analysis": [],
                "job": [],
            }
            next_value = {name: 100 for name in consumed}
            real_write_raw = recorder._write_raw
            real_write_event = recorder._write_event
            real_write_json_line = recorder._write_json_line

            def pause_and_next(name):
                value = next_value[name]
                next_value[name] += 1
                return value

            def write_raw_and_replenish(envelope):
                real_write_raw(envelope)
                consumed["raw"].append(envelope.sample.input_seq)
                value = pause_and_next("raw")
                raw_lane.submit(_sample(value), attempt_id=value)

            def write_event_and_replenish(event):
                real_write_event(event)
                consumed["event"].append(event.event_id)
                value = pause_and_next("event")
                hub.publish(
                    EventDraft(
                        kind="PROGRESS",
                        monotonic_s=float(value),
                        wall_time_us=value,
                        scope="state",
                        attempt_id=None,
                        payload={"sequence": value},
                    )
                )

            def consume_detail_and_replenish(detail, **kwargs):
                del kwargs
                consumed["detail"].append(detail.attempt_id)
                value = pause_and_next("detail")
                recorder.offer_attempt_detail(
                    _detail(value, ab_summary="BOTH_FAIL_SAME")
                )
                return True

            def write_json_and_replenish(stream, value):
                real_write_json_line(stream, value)
                if stream is recorder._files["analysis"]:
                    name = "analysis"
                    recorder.offer_replay_analysis(
                        {"sequence": pause_and_next(name)}
                    )
                elif stream is recorder._files["jobs"]:
                    name = "job"
                    recorder.offer_replay_job_status(
                        {"sequence": pause_and_next(name)}
                    )
                else:
                    return
                consumed[name].append(int(value["sequence"]))

            try:
                with mock.patch.object(
                    recorder,
                    "_write_raw",
                    side_effect=write_raw_and_replenish,
                ), mock.patch.object(
                    recorder,
                    "_write_event",
                    side_effect=write_event_and_replenish,
                ), mock.patch.object(
                    recorder,
                    "_consume_detail",
                    side_effect=consume_detail_and_replenish,
                ), mock.patch.object(
                    recorder,
                    "_write_json_line",
                    side_effect=write_json_and_replenish,
                ):
                    recorder.drain(timeout_s=0.02)

                for name, values in consumed.items():
                    self.assertEqual(
                        len(values),
                        3,
                        "{} consumed beyond entry snapshot: {}".format(
                            name,
                            values,
                        ),
                    )
                self.assertEqual(raw_lane.pending_count(), 3)
                self.assertEqual(len(recorder._detail_offers), 3)
                self.assertEqual(len(recorder._analysis_offers), 3)
                self.assertEqual(len(recorder._job_offers), 3)
            finally:
                recorder.write_terminal_and_close(
                    terminal_session={},
                    timeout_s=1.0,
                )
                hub.close()

    def test_io_error_is_sticky_for_future_attempts_without_offer_exception(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            raw_lane = RawRecordLane(capacity=2)
            raw_lane.submit(_sample(1), attempt_id=11)
            recorder, _, repository, _, _ = self._recorder(
                Path(temp_dir),
                raw_lane=raw_lane,
            )
            with mock.patch.object(
                recorder,
                "_write_raw",
                side_effect=OSError("disk full"),
            ):
                failed = recorder.drain(timeout_s=0.0)
            self.assertFalse(failed.healthy)
            self.assertTrue(
                recorder.offer_attempt_detail(
                    _detail(11, ab_summary="BOTH_FAIL_SAME")
                )
            )

            terminal = recorder.write_terminal_and_close(
                terminal_session={},
                timeout_s=0.2,
            )

            detail = repository.get(11)
            self.assertFalse(terminal.healthy)
            self.assertEqual(detail.summary.ab_summary, "INCONCLUSIVE")
            self.assertFalse(detail.summary.recording_complete)

    def test_flushes_after_one_second_or_one_thousand_rows(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            now = [0.0]
            raw_lane = RawRecordLane(capacity=2)
            recorder, paths, _, _, _ = self._recorder(
                Path(temp_dir),
                raw_lane=raw_lane,
                clock=lambda: now[0],
                flush_interval_s=1.0,
                flush_row_count=1000,
            )
            baseline = paths.ball_samples_csv.stat().st_size
            raw_lane.submit(_sample(1), attempt_id=1)
            recorder.drain(timeout_s=0.0)
            self.assertEqual(paths.ball_samples_csv.stat().st_size, baseline)
            now[0] = 1.01
            recorder.drain(timeout_s=0.0)
            self.assertGreater(
                paths.ball_samples_csv.stat().st_size,
                baseline,
            )
            recorder.write_terminal_and_close(
                terminal_session={},
                timeout_s=0.2,
            )

        with tempfile.TemporaryDirectory() as temp_dir:
            raw_lane = RawRecordLane(capacity=1000)
            for sequence in range(1, 1001):
                raw_lane.submit(_sample(sequence), attempt_id=1)
            recorder, paths, _, _, _ = self._recorder(
                Path(temp_dir),
                raw_lane=raw_lane,
                raw_quota=1000,
                flush_interval_s=3600.0,
                flush_row_count=1000,
            )
            baseline = paths.ball_samples_csv.stat().st_size
            recorder.drain(timeout_s=0.0)
            self.assertGreater(
                paths.ball_samples_csv.stat().st_size,
                baseline,
            )
            recorder.write_terminal_and_close(
                terminal_session={},
                timeout_s=0.2,
            )

    def test_all_session_files_have_parseable_committed_schema(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            hub = _hub()
            raw_lane = RawRecordLane()
            raw_lane.submit(_sample(1), attempt_id=1, track_segment_id=1)
            hub.publish(
                EventDraft(
                    kind="PROGRESS",
                    monotonic_s=1.0,
                    wall_time_us=1,
                    scope="state",
                    attempt_id=None,
                    payload={"sequence": 1},
                )
            )
            recorder, paths, _, _, _ = self._recorder(
                Path(temp_dir),
                hub=hub,
                raw_lane=raw_lane,
                flush_row_count=1,
            )
            recorder.offer_attempt_detail(
                _detail(1, ab_summary="BOTH_FAIL_SAME")
            )
            recorder.offer_replay_job_status(
                {"schema_version": 1, "attempt_id": 1, "status": "COMPLETE"}
            )
            recorder.offer_replay_analysis(
                {"schema_version": 1, "attempt_id": 1, "label": "SAME"}
            )
            recorder.drain(timeout_s=0.0)
            recorder.write_terminal_and_close(
                terminal_session={"reason": "schema-test"},
                timeout_s=0.2,
            )

            session = json.loads(paths.session_json.read_text())
            self.assertEqual(session["schema_version"], 1)
            self.assertIn(session["status"], ("COMPLETE", "INCOMPLETE"))
            with paths.ball_samples_csv.open() as stream:
                raw_rows = list(csv.DictReader(stream))
            self.assertEqual(raw_rows[0]["attempt_id"], "1")
            event = json.loads(paths.events_jsonl.read_text().splitlines()[0])
            self.assertEqual(event["schema_version"], 1)
            self.assertEqual(event["event_id"], 1)
            job = json.loads(
                paths.replay_jobs_jsonl.read_text().splitlines()[0]
            )
            analysis = json.loads(
                paths.replay_analysis_jsonl.read_text().splitlines()[0]
            )
            self.assertEqual(job["schema_version"], 1)
            self.assertEqual(analysis["schema_version"], 1)
            with paths.attempts_csv.open() as stream:
                attempts = list(csv.DictReader(stream))
            self.assertEqual(attempts[0]["attempt_id"], "1")
            detail = json.loads(
                (paths.attempt_details_dir / "1.json").read_text()
            )
            self.assertEqual(detail["schema_version"], 1)

    def test_partial_start_closes_every_file_already_opened(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            hub = _hub()
            paths = create_session_paths(Path(temp_dir))
            repository = AttemptDetailRepository(paths.attempt_details_dir)
            recorder = SessionRecorder(
                paths=paths,
                raw_lane=RawRecordLane(),
                event_cursor=hub.open_cursor(0),
                attempt_repository=repository,
            )
            opened = []
            real_open_private = recording._open_private

            def fail_second_open(path):
                if opened:
                    raise OSError("second file failed")
                stream = real_open_private(path)
                opened.append(stream)
                return stream

            with mock.patch.object(
                recording,
                "_open_private",
                side_effect=fail_second_open,
            ):
                recorder.start()

            self.assertFalse(recorder.status().healthy)
            self.assertTrue(opened[0].closed)
            hub.close()

    def test_concurrent_drain_and_terminal_close_are_serialized(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder, paths, _, _, _ = self._recorder(Path(temp_dir))
            recorder.offer_attempt_detail(
                _detail(1, ab_summary="BOTH_FAIL_SAME")
            )
            barrier = threading.Barrier(3)
            results = []
            errors = []

            def close_recorder():
                try:
                    barrier.wait()
                    results.append(
                        recorder.write_terminal_and_close(
                            terminal_session={},
                            timeout_s=0.2,
                        )
                    )
                except BaseException as exc:
                    errors.append(exc)

            def drain_recorder():
                try:
                    barrier.wait()
                    results.append(recorder.drain(timeout_s=0.2))
                except BaseException as exc:
                    errors.append(exc)

            threads = [
                threading.Thread(target=close_recorder),
                threading.Thread(target=close_recorder),
                threading.Thread(target=drain_recorder),
            ]
            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join(2.0)

            self.assertFalse(errors)
            self.assertEqual(len(results), 3)
            with paths.attempts_csv.open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 1)


if __name__ == "__main__":
    unittest.main()
