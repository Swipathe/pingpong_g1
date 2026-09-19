from __future__ import annotations

import csv
import copy
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock
import warnings

from diagnostics import export_hitter_task_csv as csv_export


scan_planner_events = csv_export.scan_planner_events


def snapshot_key(
    *,
    generation: int,
    schema_version: int = 1,
    track_epoch: int = 1,
):
    return {
        "schema_version": schema_version,
        "track_epoch": track_epoch,
        "generation": generation,
    }


def planner_submit_event(
    *,
    attempt_id: int,
    generation: int,
    t: float,
):
    return {
        "schema_version": 1,
        "kind": "planner_submit",
        "monotonic_s": t,
        "attempt_id": attempt_id,
        "payload": {
            "snapshot_key": snapshot_key(generation=generation),
        },
    }


def planner_trace_event(
    *,
    attempt_id: int,
    generation: int,
    trace_kind: str,
    t: float,
    **payload,
):
    return {
        "schema_version": 1,
        "kind": "planner_trace",
        "monotonic_s": t,
        "attempt_id": attempt_id,
        "payload": {
            "trace_kind": trace_kind,
            "snapshot_key": snapshot_key(generation=generation),
            **payload,
        },
    }


def pending_replaced_event(
    *,
    attempt_id: int,
    generation: int,
    replaced_generation: int,
    replaced_attempt_id=None,
    t: float,
):
    event = planner_trace_event(
        attempt_id=attempt_id,
        generation=generation,
        trace_kind="pending_replaced",
        t=t,
        replaced_snapshot_key=snapshot_key(
            generation=replaced_generation,
        ),
    )
    if replaced_attempt_id is not None:
        event["payload"]["replaced_attempt_id"] = replaced_attempt_id
    return event


def planner_trace_replaced_event(
    *,
    replaced_attempt_id: int,
    replaced_generation: int,
    replacing_attempt_id: int,
    replacing_generation: int,
    t: float,
):
    return {
        "schema_version": 1,
        "kind": "planner_trace_replaced",
        "monotonic_s": t,
        "attempt_id": replaced_attempt_id,
        "payload": {
            "trace_kind": "pending_replaced",
            "replaced_snapshot_key": snapshot_key(
                generation=replaced_generation,
            ),
            "replacing_snapshot_key": snapshot_key(
                generation=replacing_generation,
            ),
            "replacing_attempt_id": replacing_attempt_id,
        },
    }


def planner_start_event(
    *,
    attempt_id: int,
    generation: int,
    t: float,
):
    return planner_trace_event(
        attempt_id=attempt_id,
        generation=generation,
        trace_kind="start",
        t=t,
    )


def planner_complete_event(
    *,
    attempt_id: int,
    generation: int,
    source_frame: int,
    completed_t: float,
    error_type,
    error_text,
):
    key = snapshot_key(generation=generation)
    return planner_trace_event(
        attempt_id=attempt_id,
        generation=generation,
        trace_kind="complete",
        t=completed_t,
        result={
            "snapshot_key": key,
            "source_frame": source_frame,
            "strike_deadline_monotonic_s": completed_t + 0.5,
            "completed_monotonic_s": completed_t,
            "command_fields": None,
            "error_type": error_type,
            "error_text": error_text,
            "reason_code": "PLANNER_EXCEPTION",
        },
    )


def write_json_lines(path: Path, rows) -> None:
    with path.open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row, separators=(",", ":")))
            stream.write("\n")


def cross_attempt_pending_replacement_events():
    return [
        planner_submit_event(
            attempt_id=1,
            generation=10,
            t=1.0,
        ),
        planner_submit_event(
            attempt_id=2,
            generation=11,
            t=1.01,
        ),
        pending_replaced_event(
            attempt_id=2,
            generation=11,
            replaced_generation=10,
            replaced_attempt_id=1,
            t=1.011,
        ),
        planner_start_event(
            attempt_id=2,
            generation=11,
            t=1.02,
        ),
        planner_complete_event(
            attempt_id=2,
            generation=11,
            source_frame=1011,
            completed_t=1.03,
            error_type=None,
            error_text=None,
        ),
        planner_trace_replaced_event(
            replaced_attempt_id=1,
            replaced_generation=10,
            replacing_attempt_id=2,
            replacing_generation=11,
            t=1.011,
        ),
    ]


def export_key(
    *,
    generation: int,
    attempt_id: int = 1,
    schema_version: int = 1,
    track_epoch: int = 1,
):
    return csv_export.PlannerExportKey(
        attempt_id=attempt_id,
        schema_version=schema_version,
        track_epoch=track_epoch,
        generation=generation,
    )


def completed_call(
    *,
    generation: int,
    completed_t: float,
    source_frame: int,
    attempt_id: int = 1,
    command_fields=None,
    error_type="ValueError",
    error_text="example rejection",
    reason_code="PLANNER_EXCEPTION",
    pending_replaced_key=None,
    latest_replaced_key=None,
):
    key = export_key(
        attempt_id=attempt_id,
        generation=generation,
    )
    return csv_export.CompletedPlannerCall(
        key=key,
        submitted_monotonic_s=completed_t - 0.02,
        started_monotonic_s=completed_t - 0.01,
        completed_monotonic_s=completed_t,
        source_frame=source_frame,
        result={
            "snapshot_key": snapshot_key(generation=generation),
            "source_frame": source_frame,
            "strike_deadline_monotonic_s": completed_t + 0.5,
            "completed_monotonic_s": completed_t,
            "command_fields": command_fields,
            "error_type": error_type,
            "error_text": error_text,
            "reason_code": reason_code,
        },
        pending_replaced_key=pending_replaced_key,
        latest_replaced_key=latest_replaced_key,
    )


def planner_scan(calls):
    return csv_export.PlannerEventScan(
        submitted_keys=frozenset(call.key for call in calls),
        pending_replaced_keys=frozenset(),
        completed_calls=tuple(calls),
    )


def planner_input(
    *,
    generation: int,
    source_frame: int,
    velocity_x: float,
    attempt_id: int = 1,
):
    return {
        "attempt_id": attempt_id,
        "snapshot_key": snapshot_key(generation=generation),
        "source_frame": source_frame,
        "source_time_s": 100.0 + generation,
        "received_monotonic_s": 200.0 + generation,
        "position_w": [1.0, 2.0, 3.0],
        "velocity_w": [velocity_x, 0.0, 0.0],
        "base_position_w": [4.0, 5.0, 6.0],
        "base_quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
        "base_valid": True,
        "visible": True,
        "ready": True,
    }


def csv_keys(rows):
    return [
        (
            row["attempt_id"],
            row["snapshot_key.schema_version"],
            row["snapshot_key.track_epoch"],
            row["snapshot_key.generation"],
        )
        for row in rows
    ]


def write_attempt_detail(
    session_dir: Path,
    *,
    attempt_id: int,
    planner_inputs,
    detail_attempt_id=None,
) -> None:
    details_dir = session_dir / "attempt_details"
    details_dir.mkdir(parents=True, exist_ok=True)
    value = {
        "schema_version": 1,
        "attempt_id": (
            attempt_id
            if detail_attempt_id is None
            else detail_attempt_id
        ),
        "planner_inputs": planner_inputs,
    }
    (details_dir / "{}.json".format(attempt_id)).write_text(
        json.dumps(value, separators=(",", ":")),
        encoding="utf-8",
    )


def _complete_recorded_event(event, event_id):
    value = copy.deepcopy(event)
    value["event_id"] = event_id
    value.setdefault("wall_time_us", event_id)
    value.setdefault("scope", "attempt")
    return value


def _lifecycle_tick_event(
    *,
    event_id,
    attempt_id,
    t,
    task_observation=None,
):
    if task_observation is None:
        task_pre_clip = None
        task_post_clip = None
        clip_count = None
    else:
        task_pre_clip, task_post_clip, clip_count = task_observation
    return {
        "schema_version": 1,
        "event_id": event_id,
        "kind": "lifecycle_tick",
        "monotonic_s": t,
        "wall_time_us": event_id,
        "scope": "attempt",
        "attempt_id": attempt_id,
        "payload": {
            "lifecycle_now_s": t,
            "obs_now_s": t + 0.001,
            "phase": "striking",
            "decision": "execute",
            "active_key": snapshot_key(generation=11),
            "cached_key": None,
            "command_result": None,
            "task_pre_clip": task_pre_clip,
            "task_post_clip": task_post_clip,
            "clip_count": clip_count,
            "task_pass": True,
            "errors": [],
            "recovery_duration_s": 0.0,
        },
    }


def _attempt_transition_event(
    *,
    attempt_id,
    track_segment_id,
    stage,
    t,
    generation=None,
    reason_code=None,
    values=None,
):
    return {
        "schema_version": 1,
        "kind": "attempt_transition",
        "monotonic_s": t,
        "wall_time_us": 0,
        "scope": "attempt",
        "attempt_id": attempt_id,
        "payload": {
            "track_segment_id": track_segment_id,
            "stage": stage,
            "reason_code": reason_code,
            "snapshot_key": (
                None
                if generation is None
                else snapshot_key(generation=generation)
            ),
            "values": {} if values is None else values,
        },
    }


def _detail_transition_from_event(event):
    payload = event["payload"]
    return {
        "schema_version": event["schema_version"],
        "attempt_id": event["attempt_id"],
        "track_segment_id": payload["track_segment_id"],
        "stage": payload["stage"],
        "monotonic_s": event["monotonic_s"],
        "snapshot_key": payload["snapshot_key"],
        "reason_code": payload["reason_code"],
        "values": payload["values"],
    }


def _full_attempt_detail(
    *,
    attempt_id,
    generation,
    source_frame,
    first_t,
    last_t,
    recording_complete,
    task_observation,
):
    task_pre, task_post, clip_count = task_observation
    return {
        "schema_version": 1,
        "attempt_id": attempt_id,
        "summary": {
            "schema_version": 1,
            "attempt_id": attempt_id,
            "status": "SUCCESS",
            "stage": "ATTEMPT_CLOSED",
            "primary_blocker": None,
            "ball_speed_mps": 3.5,
            "predicted_strike_time_s": 0.4,
            "planner_tts_s": 0.3,
            "arm_tts_s": 0.2,
            "task_obs_status": "PASS",
            "ab_summary": "INCONCLUSIVE",
            "recording_complete": recording_complete,
            "estimator_sample_count": 31,
            "estimator_window_size": 31,
            "incoming_count": 3,
            "incoming_required_count": 3,
        },
        "segments": [
            {
                "track_segment_id": attempt_id,
                "first_monotonic_s": first_t,
                "last_monotonic_s": last_t,
                "role": "PRIMARY",
            }
        ],
        "stage_timeline": [
            _detail_transition_from_event(
                _attempt_transition_event(
                    attempt_id=attempt_id,
                    track_segment_id=attempt_id,
                    stage="DETECTED",
                    t=first_t,
                    generation=generation,
                    values={"role": "PRIMARY"},
                )
            ),
            _detail_transition_from_event(
                _attempt_transition_event(
                    attempt_id=attempt_id,
                    track_segment_id=attempt_id,
                    stage="ATTEMPT_CLOSED",
                    t=last_t,
                    reason_code="COMPLETE",
                    values={
                        "status": "SUCCESS",
                        "primary_blocker": None,
                    },
                )
            ),
        ],
        "planner_inputs": [
            planner_input(
                attempt_id=attempt_id,
                generation=generation,
                source_frame=source_frame,
                velocity_x=-1.0,
            )
        ],
        "planner_results": [],
        "task_observation_pre_clip": task_pre,
        "task_observation_post_clip": task_post,
        "task_observation_clip_count": clip_count,
        "variant_outcomes": [],
        "ab_deltas": {},
    }


def build_complete_session_fixture(root: Path) -> Path:
    session_dir = root / "session"
    details_dir = session_dir / "attempt_details"
    details_dir.mkdir(parents=True)

    details = (
        _full_attempt_detail(
            attempt_id=1,
            generation=11,
            source_frame=1011,
            first_t=1.0,
            last_t=1.5,
            recording_complete=False,
            task_observation=(None, None, None),
        ),
        _full_attempt_detail(
            attempt_id=2,
            generation=21,
            source_frame=1021,
            first_t=2.0,
            last_t=2.5,
            recording_complete=True,
            task_observation=(
                [float(index) for index in range(11)],
                [float(index) for index in range(11)],
                0,
            ),
        ),
    )
    for detail in details:
        (details_dir / "{}.json".format(detail["attempt_id"])).write_text(
            json.dumps(detail, separators=(",", ":")),
            encoding="utf-8",
        )

    events = [
        _attempt_transition_event(
            attempt_id=1,
            track_segment_id=1,
            stage="DETECTED",
            t=1.0,
            generation=11,
            values={"role": "PRIMARY"},
        ),
        planner_submit_event(attempt_id=1, generation=10, t=1.10),
        planner_submit_event(attempt_id=1, generation=11, t=1.11),
        pending_replaced_event(
            attempt_id=1,
            generation=11,
            replaced_generation=10,
            t=1.12,
        ),
        planner_start_event(attempt_id=1, generation=11, t=1.13),
        planner_complete_event(
            attempt_id=1,
            generation=11,
            source_frame=1011,
            completed_t=1.14,
            error_type=None,
            error_text=None,
        ),
        _lifecycle_tick_event(event_id=0, attempt_id=1, t=1.20),
        _attempt_transition_event(
            attempt_id=1,
            track_segment_id=1,
            stage="ATTEMPT_CLOSED",
            t=1.5,
            reason_code="COMPLETE",
            values={
                "status": "SUCCESS",
                "primary_blocker": None,
            },
        ),
        _attempt_transition_event(
            attempt_id=2,
            track_segment_id=2,
            stage="DETECTED",
            t=2.0,
            generation=21,
            values={"role": "PRIMARY"},
        ),
        planner_submit_event(attempt_id=2, generation=21, t=2.10),
        planner_start_event(attempt_id=2, generation=21, t=2.11),
        planner_complete_event(
            attempt_id=2,
            generation=21,
            source_frame=1021,
            completed_t=2.12,
            error_type=None,
            error_text=None,
        ),
        _lifecycle_tick_event(
            event_id=0,
            attempt_id=2,
            t=2.20,
            task_observation=(
                [float(index) for index in range(11)],
                [float(index) for index in range(11)],
                0,
            ),
        ),
        _attempt_transition_event(
            attempt_id=2,
            track_segment_id=2,
            stage="ATTEMPT_CLOSED",
            t=2.5,
            reason_code="COMPLETE",
            values={
                "status": "SUCCESS",
                "primary_blocker": None,
            },
        ),
        _lifecycle_tick_event(event_id=0, attempt_id=None, t=3.20),
    ]
    recorded_events = [
        _complete_recorded_event(event, event_id)
        for event_id, event in enumerate(events, start=1)
    ]
    write_json_lines(session_dir / "events.jsonl", recorded_events)

    raw_fields = (
        "input_seq",
        "attempt_id",
        "track_segment_id",
        "channel",
        "subject",
        "position_w",
        "quaternion_xyzw",
        "valid",
        "occluded",
        "source_frame",
        "source_time_s",
        "publish_time_us",
        "received_monotonic_s",
        "wall_time_us",
        "payload_size",
    )
    raw_rows = []
    for input_seq, attempt_id, t in (
        (1, 1, 1.05),
        (2, 1, 1.06),
        (3, 1, 1.07),
        (4, 2, 2.05),
        (5, 2, 2.06),
        (6, 2, 2.07),
        (7, None, 3.05),
    ):
        subject = ("ball", "g1pelvis", "table")[(input_seq - 1) % 3]
        raw_rows.append({
            "input_seq": input_seq,
            "attempt_id": attempt_id if subject == "ball" else None,
            "track_segment_id": (
                attempt_id if subject == "ball" else None
            ),
            "channel": "vicon_state_data",
            "subject": subject,
            "position_w": "[1.0,2.0,3.0]",
            "quaternion_xyzw": "[0.0,0.0,0.0,1.0]",
            "valid": True,
            "occluded": False,
            "source_frame": 1000 + input_seq,
            "source_time_s": 100.0 + input_seq,
            "publish_time_us": 10000 + input_seq,
            "received_monotonic_s": t,
            "wall_time_us": 20000 + input_seq,
            "payload_size": 100,
        })
    with (session_dir / "ball_samples.csv").open(
        "w",
        encoding="utf-8",
        newline="",
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=raw_fields)
        writer.writeheader()
        writer.writerows(raw_rows)

    event_count = len(recorded_events)
    (session_dir / "session.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "status": "COMPLETE",
                "recording_complete": True,
                "recorder_healthy": True,
                "last_error": None,
                "failure": None,
                "input_samples_dropped": 0,
                "watermark_event_id": event_count,
                "observed_watermark_event_id": event_count,
            },
            separators=(",", ":"),
        ),
        encoding="utf-8",
    )
    return session_dir


def csv_row_count(path: Path) -> int:
    with path.open("r", encoding="utf-8", newline="") as stream:
        return sum(1 for _row in csv.DictReader(stream))


def csv_key_sequence(path: Path):
    with path.open("r", encoding="utf-8", newline="") as stream:
        return [
            (
                int(row["attempt_id"]),
                int(row["snapshot_key.schema_version"]),
                int(row["snapshot_key.track_epoch"]),
                int(row["snapshot_key.generation"]),
            )
            for row in csv.DictReader(stream)
        ]


class PlannerEventScanTests(unittest.TestCase):
    def test_planner_key_rejects_non_integer_or_out_of_range_components(
        self,
    ):
        valid = snapshot_key(generation=3)
        cases = (
            ("attempt bool", True, valid),
            ("attempt zero", 0, valid),
            ("attempt string", "1", valid),
            ("attempt float", 1.0, valid),
            (
                "schema bool",
                1,
                dict(valid, schema_version=True),
            ),
            (
                "schema unsupported",
                1,
                dict(valid, schema_version=2),
            ),
            (
                "schema string",
                1,
                dict(valid, schema_version="1"),
            ),
            (
                "schema float",
                1,
                dict(valid, schema_version=1.0),
            ),
            (
                "track epoch bool",
                1,
                dict(valid, track_epoch=False),
            ),
            (
                "track epoch negative",
                1,
                dict(valid, track_epoch=-1),
            ),
            (
                "track epoch string",
                1,
                dict(valid, track_epoch="1"),
            ),
            (
                "track epoch float",
                1,
                dict(valid, track_epoch=1.0),
            ),
            (
                "generation bool",
                1,
                dict(valid, generation=True),
            ),
            (
                "generation negative",
                1,
                dict(valid, generation=-1),
            ),
            (
                "generation string",
                1,
                dict(valid, generation="3"),
            ),
            (
                "generation float",
                1,
                dict(valid, generation=3.0),
            ),
        )

        for label, attempt_id, key_value in cases:
            with self.subTest(label=label):
                with self.assertRaises(ValueError):
                    csv_export._planner_key(
                        attempt_id,
                        key_value,
                        field="test snapshot_key",
                    )

    def test_event_and_replacement_attempt_ids_are_not_coerced(self):
        cases = (
            (
                "event attempt",
                [
                    planner_submit_event(
                        attempt_id="1",
                        generation=11,
                        t=1.0,
                    ),
                    planner_start_event(
                        attempt_id="1",
                        generation=11,
                        t=1.01,
                    ),
                    planner_complete_event(
                        attempt_id="1",
                        generation=11,
                        source_frame=1011,
                        completed_t=1.02,
                        error_type=None,
                        error_text=None,
                    ),
                ],
            ),
            (
                "replacement attempt",
                [
                    planner_submit_event(
                        attempt_id=1,
                        generation=10,
                        t=1.0,
                    ),
                    planner_submit_event(
                        attempt_id=2,
                        generation=11,
                        t=1.01,
                    ),
                    pending_replaced_event(
                        attempt_id=2,
                        generation=11,
                        replaced_generation=10,
                        replaced_attempt_id="1",
                        t=1.011,
                    ),
                    planner_start_event(
                        attempt_id=2,
                        generation=11,
                        t=1.02,
                    ),
                    planner_complete_event(
                        attempt_id=2,
                        generation=11,
                        source_frame=1011,
                        completed_t=1.03,
                        error_type=None,
                        error_text=None,
                    ),
                ],
            ),
        )

        for label, events in cases:
            with self.subTest(label=label):
                with tempfile.TemporaryDirectory() as directory:
                    events_path = Path(directory) / "events.jsonl"
                    write_json_lines(events_path, events)

                    with self.assertRaises(ValueError):
                        scan_planner_events(events_path, [1, 2])

    def test_completed_result_source_frame_is_not_coerced(self):
        events = [
            planner_submit_event(
                attempt_id=1,
                generation=11,
                t=1.00,
            ),
            planner_start_event(
                attempt_id=1,
                generation=11,
                t=1.01,
            ),
            planner_complete_event(
                attempt_id=1,
                generation=11,
                source_frame=1011.9,
                completed_t=1.02,
                error_type=None,
                error_text=None,
            ),
        ]
        with tempfile.TemporaryDirectory() as directory:
            events_path = Path(directory) / "events.jsonl"
            write_json_lines(events_path, events)

            with self.assertRaisesRegex(ValueError, "source_frame"):
                scan_planner_events(events_path, [1])

    def test_scan_keeps_cross_attempt_pending_key_when_replacer_is_unselected(
        self,
    ):
        with tempfile.TemporaryDirectory() as directory:
            events_path = Path(directory) / "events.jsonl"
            write_json_lines(
                events_path,
                cross_attempt_pending_replacement_events(),
            )

            try:
                scan = scan_planner_events(events_path, [1])
            except ValueError as exc:
                self.fail(
                    "selected replaced attempt must remain classified: "
                    "{}".format(exc)
                )

            self.assertEqual(
                {
                    (key.attempt_id, key.generation)
                    for key in scan.submitted_keys
                },
                {(1, 10)},
            )
            self.assertEqual(
                {
                    (key.attempt_id, key.generation)
                    for key in scan.pending_replaced_keys
                },
                {(1, 10)},
            )
            self.assertEqual(scan.completed_calls, ())

    def test_scan_keeps_cross_attempt_replacer_when_replaced_is_unselected(
        self,
    ):
        with tempfile.TemporaryDirectory() as directory:
            events_path = Path(directory) / "events.jsonl"
            write_json_lines(
                events_path,
                cross_attempt_pending_replacement_events(),
            )

            scan = scan_planner_events(events_path, [2])

            self.assertEqual(
                {
                    (key.attempt_id, key.generation)
                    for key in scan.submitted_keys
                },
                {(2, 11)},
            )
            self.assertEqual(scan.pending_replaced_keys, frozenset())
            self.assertEqual(
                [
                    (call.key.attempt_id, call.key.generation)
                    for call in scan.completed_calls
                ],
                [(2, 11)],
            )
            self.assertEqual(
                (
                    scan.completed_calls[0].pending_replaced_key.attempt_id,
                    scan.completed_calls[0].pending_replaced_key.generation,
                ),
                (1, 10),
            )

    def test_scan_closes_cross_attempt_replacement_when_both_are_selected(
        self,
    ):
        with tempfile.TemporaryDirectory() as directory:
            events_path = Path(directory) / "events.jsonl"
            write_json_lines(
                events_path,
                cross_attempt_pending_replacement_events(),
            )

            scan = scan_planner_events(events_path, [1, 2])

            self.assertEqual(
                {
                    (key.attempt_id, key.generation)
                    for key in scan.submitted_keys
                },
                {(1, 10), (2, 11)},
            )
            self.assertEqual(
                {
                    (key.attempt_id, key.generation)
                    for key in scan.pending_replaced_keys
                },
                {(1, 10)},
            )
            self.assertEqual(
                [
                    (call.key.attempt_id, call.key.generation)
                    for call in scan.completed_calls
                ],
                [(2, 11)],
            )

    def test_scan_keeps_only_calls_that_started_and_completed(self):
        with tempfile.TemporaryDirectory() as directory:
            events_path = Path(directory) / "events.jsonl"
            events = [
                planner_submit_event(
                    attempt_id=1,
                    generation=10,
                    t=1.0,
                ),
                planner_submit_event(
                    attempt_id=1,
                    generation=11,
                    t=1.01,
                ),
                pending_replaced_event(
                    attempt_id=1,
                    generation=11,
                    replaced_generation=10,
                    t=1.011,
                ),
                planner_start_event(
                    attempt_id=1,
                    generation=11,
                    t=1.02,
                ),
                planner_complete_event(
                    attempt_id=1,
                    generation=11,
                    source_frame=1011,
                    completed_t=1.03,
                    error_type="ValueError",
                    error_text="example rejection",
                ),
            ]
            write_json_lines(events_path, events)

            scan = scan_planner_events(events_path, [1])

            self.assertEqual(
                [call.key.generation for call in scan.completed_calls],
                [11],
            )
            self.assertEqual(
                {key.generation for key in scan.pending_replaced_keys},
                {10},
            )
            self.assertEqual(
                {key.generation for key in scan.submitted_keys},
                {10, 11},
            )

    def test_scan_rejects_unclassified_submitted_key(self):
        with tempfile.TemporaryDirectory() as directory:
            events_path = Path(directory) / "events.jsonl"
            write_json_lines(
                events_path,
                [
                    planner_submit_event(
                        attempt_id=1,
                        generation=20,
                        t=2.0,
                    ),
                ],
            )

            with self.assertRaisesRegex(
                ValueError,
                "submitted keys are neither completed nor pending-replaced",
            ):
                scan_planner_events(events_path, [1])

    def test_scan_rejects_incomplete_selected_attempt_event_sets(self):
        valid_replacer_events = [
            planner_submit_event(
                attempt_id=1,
                generation=11,
                t=1.10,
            ),
            pending_replaced_event(
                attempt_id=1,
                generation=11,
                replaced_generation=10,
                t=1.11,
            ),
            planner_start_event(
                attempt_id=1,
                generation=11,
                t=1.12,
            ),
            planner_complete_event(
                attempt_id=1,
                generation=11,
                source_frame=1011,
                completed_t=1.13,
                error_type=None,
                error_text=None,
            ),
        ]
        cases = (
            (
                "orphan start and complete without submit",
                [
                    planner_start_event(
                        attempt_id=1,
                        generation=10,
                        t=1.00,
                    ),
                    planner_complete_event(
                        attempt_id=1,
                        generation=10,
                        source_frame=1010,
                        completed_t=1.01,
                        error_type=None,
                        error_text=None,
                    ),
                ],
                (
                    r"planner start/complete keys lack submissions: "
                    r"starts=.*generation=10.*completions=.*generation=10"
                ),
            ),
            (
                "started pending replacement without complete",
                [
                    planner_submit_event(
                        attempt_id=1,
                        generation=10,
                        t=1.00,
                    ),
                    planner_start_event(
                        attempt_id=1,
                        generation=10,
                        t=1.01,
                    ),
                ] + valid_replacer_events,
                (
                    r"planner start and complete keys differ: "
                    r"missing_complete=.*generation=10.*missing_start=\[\]"
                ),
            ),
            (
                "completed pending replacement without start",
                [
                    planner_submit_event(
                        attempt_id=1,
                        generation=10,
                        t=1.00,
                    ),
                    planner_complete_event(
                        attempt_id=1,
                        generation=10,
                        source_frame=1010,
                        completed_t=1.01,
                        error_type=None,
                        error_text=None,
                    ),
                ] + valid_replacer_events,
                (
                    r"planner start and complete keys differ: "
                    r"missing_complete=\[\], "
                    r"missing_start=.*generation=10"
                ),
            ),
        )

        for label, events, error_pattern in cases:
            with self.subTest(label=label):
                with tempfile.TemporaryDirectory() as directory:
                    events_path = Path(directory) / "events.jsonl"
                    write_json_lines(events_path, events)

                    with self.assertRaisesRegex(
                        ValueError,
                        error_pattern,
                    ):
                        scan_planner_events(events_path, [1])

    def test_scan_rejects_conflicting_duplicate_planner_events(self):
        base_events = [
            planner_submit_event(
                attempt_id=1,
                generation=30,
                t=3.0,
            ),
            planner_start_event(
                attempt_id=1,
                generation=30,
                t=3.01,
            ),
            planner_complete_event(
                attempt_id=1,
                generation=30,
                source_frame=1030,
                completed_t=3.02,
                error_type=None,
                error_text=None,
            ),
        ]
        conflicts = (
            ("planner submit", 0, "input_seq", 999),
            ("planner start", 1, "snapshot_ready", False),
            ("planner complete", 2, "incoming_confirmed", True),
        )

        for label, event_index, field, value in conflicts:
            with self.subTest(label=label):
                with tempfile.TemporaryDirectory() as directory:
                    events_path = Path(directory) / "events.jsonl"
                    events = copy.deepcopy(base_events)
                    duplicate = copy.deepcopy(events[event_index])
                    duplicate["payload"][field] = value
                    events.insert(event_index + 1, duplicate)
                    write_json_lines(events_path, events)

                    with self.assertRaisesRegex(
                        ValueError,
                        "conflicting duplicate {}".format(label),
                    ):
                        scan_planner_events(events_path, [1])

    def test_scan_rejects_key_classified_as_completed_and_pending_replaced(self):
        with tempfile.TemporaryDirectory() as directory:
            events_path = Path(directory) / "events.jsonl"
            events = [
                planner_submit_event(
                    attempt_id=1,
                    generation=40,
                    t=4.0,
                ),
                planner_start_event(
                    attempt_id=1,
                    generation=40,
                    t=4.01,
                ),
                planner_complete_event(
                    attempt_id=1,
                    generation=40,
                    source_frame=1040,
                    completed_t=4.02,
                    error_type=None,
                    error_text=None,
                ),
                planner_submit_event(
                    attempt_id=1,
                    generation=41,
                    t=4.03,
                ),
                pending_replaced_event(
                    attempt_id=1,
                    generation=41,
                    replaced_generation=40,
                    t=4.031,
                ),
                planner_start_event(
                    attempt_id=1,
                    generation=41,
                    t=4.04,
                ),
                planner_complete_event(
                    attempt_id=1,
                    generation=41,
                    source_frame=1041,
                    completed_t=4.05,
                    error_type=None,
                    error_text=None,
                ),
            ]
            write_json_lines(events_path, events)

            with self.assertRaisesRegex(
                ValueError,
                "completed and pending-replaced keys overlap",
            ):
                scan_planner_events(events_path, [1])


class AlignedPlannerRowsTests(unittest.TestCase):
    def test_planner_input_attempt_id_is_not_coerced(self):
        with tempfile.TemporaryDirectory() as directory:
            session_dir = Path(directory)
            input_row = planner_input(
                generation=61,
                source_frame=1061,
                velocity_x=-0.125,
            )
            input_row["attempt_id"] = "1"
            write_attempt_detail(
                session_dir,
                attempt_id=1,
                planner_inputs=[input_row],
            )

            with self.assertRaises(ValueError):
                csv_export.load_attempt_inputs(session_dir, [1])

    def test_three_core_tables_share_the_same_completed_key_order(self):
        calls = (
            completed_call(
                generation=31,
                completed_t=3.1,
                source_frame=1031,
            ),
            completed_call(
                generation=42,
                completed_t=4.2,
                source_frame=1042,
            ),
        )
        inputs = {
            export_key(generation=42): planner_input(
                generation=42,
                source_frame=1042,
                velocity_x=-1.2,
            ),
            export_key(generation=31): planner_input(
                generation=31,
                source_frame=1031,
                velocity_x=0.4,
            ),
        }

        aligned = csv_export.align_completed_calls(
            planner_scan(calls),
            inputs,
        )
        input_rows = csv_export.planner_input_csv_rows(aligned)
        call_rows = csv_export.planner_call_csv_rows(aligned)
        result_rows = csv_export.planner_result_csv_rows(aligned)

        expected = [(1, 1, 1, 31), (1, 1, 1, 42)]
        self.assertEqual(csv_keys(input_rows), expected)
        self.assertEqual(csv_keys(call_rows), expected)
        self.assertEqual(csv_keys(result_rows), expected)

    def test_core_rows_preserve_values_with_stable_compact_json(self):
        success = completed_call(
            generation=51,
            completed_t=5.1,
            source_frame=1051,
            command_fields={
                "strike_type": "forehand",
                "p_base_target_xy": [0.2, -0.1],
            },
            error_type=None,
            error_text=None,
            reason_code=None,
            pending_replaced_key=export_key(generation=50),
            latest_replaced_key=export_key(generation=49),
        )
        failure = completed_call(
            generation=52,
            completed_t=5.2,
            source_frame=1052,
        )
        failure.result["strike_deadline_monotonic_s"] = "NaN"
        inputs = {
            success.key: planner_input(
                generation=51,
                source_frame=1051,
                velocity_x=-0.25,
            ),
            failure.key: planner_input(
                generation=52,
                source_frame=1052,
                velocity_x=0.75,
            ),
        }

        aligned = csv_export.align_completed_calls(
            planner_scan((success, failure)),
            inputs,
        )
        input_rows = csv_export.planner_input_csv_rows(aligned)
        call_rows = csv_export.planner_call_csv_rows(aligned)
        result_rows = csv_export.planner_result_csv_rows(aligned)

        self.assertEqual(input_rows[0]["velocity_w"], "[-0.25,0.0,0.0]")
        self.assertEqual(input_rows[0]["position_w"], "[1.0,2.0,3.0]")
        self.assertEqual(call_rows[0]["source_frame"], 1051)
        self.assertAlmostEqual(call_rows[0]["duration_s"], 0.01)
        self.assertEqual(
            call_rows[0]["pending_replaced_key"],
            (
                '{"attempt_id":1,"generation":50,"schema_version":1,'
                '"track_epoch":1}'
            ),
        )
        self.assertEqual(
            call_rows[0]["latest_replaced_key"],
            (
                '{"attempt_id":1,"generation":49,"schema_version":1,'
                '"track_epoch":1}'
            ),
        )
        self.assertEqual(
            result_rows[0]["command_fields"],
            (
                '{"p_base_target_xy":[0.2,-0.1],'
                '"strike_type":"forehand"}'
            ),
        )
        self.assertIsNone(result_rows[0]["reason_code"])
        self.assertIsNone(result_rows[0]["error_type"])
        self.assertIsNone(result_rows[0]["error_text"])
        self.assertIsNone(result_rows[1]["command_fields"])
        self.assertEqual(result_rows[1]["reason_code"], "PLANNER_EXCEPTION")
        self.assertEqual(result_rows[1]["error_type"], "ValueError")
        self.assertEqual(result_rows[1]["error_text"], "example rejection")
        self.assertEqual(
            result_rows[1]["strike_deadline_monotonic_s"],
            "NaN",
        )

    def test_load_attempt_inputs_uses_detail_identity_and_compact_json(self):
        with tempfile.TemporaryDirectory() as directory:
            session_dir = Path(directory)
            input_row = planner_input(
                generation=61,
                source_frame=1061,
                velocity_x=-0.125,
            )
            input_row.pop("attempt_id")
            input_row["estimator_metadata"] = {
                "zeta": [3, 2],
                "alpha": True,
            }
            write_attempt_detail(
                session_dir,
                attempt_id=1,
                planner_inputs=[input_row],
            )

            inputs = csv_export.load_attempt_inputs(session_dir, [1])

            key = export_key(generation=61)
            self.assertEqual(set(inputs), {key})
            self.assertEqual(inputs[key]["attempt_id"], 1)
            self.assertEqual(
                inputs[key]["snapshot_key"],
                snapshot_key(generation=61),
            )
            self.assertEqual(
                inputs[key]["velocity_w"],
                "[-0.125,0.0,0.0]",
            )
            self.assertEqual(
                inputs[key]["estimator_metadata"],
                '{"alpha":true,"zeta":[3,2]}',
            )

    def test_completed_call_without_input_is_rejected(self):
        call = completed_call(
            generation=9,
            completed_t=0.9,
            source_frame=1009,
        )

        with self.assertRaisesRegex(ValueError, "has no input"):
            csv_export.align_completed_calls(
                planner_scan((call,)),
                {},
            )

    def test_duplicate_planner_input_key_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            session_dir = Path(directory)
            input_row = planner_input(
                generation=9,
                source_frame=1009,
                velocity_x=0.2,
            )
            input_row.pop("attempt_id")
            write_attempt_detail(
                session_dir,
                attempt_id=1,
                planner_inputs=[input_row, copy.deepcopy(input_row)],
            )

            with self.assertRaisesRegex(
                ValueError,
                "duplicate planner input key",
            ):
                csv_export.load_attempt_inputs(session_dir, [1])

    def test_source_frame_conflict_is_rejected(self):
        call = completed_call(
            generation=9,
            completed_t=0.9,
            source_frame=1009,
        )
        input_row = planner_input(
            generation=9,
            source_frame=9999,
            velocity_x=0.2,
        )

        with self.assertRaisesRegex(ValueError, "source_frame mismatch"):
            csv_export.align_completed_calls(
                planner_scan((call,)),
                {call.key: input_row},
            )

    def test_input_source_frame_is_not_coerced(self):
        call = completed_call(
            generation=9,
            completed_t=0.9,
            source_frame=100,
        )
        input_row = planner_input(
            generation=9,
            source_frame=100.9,
            velocity_x=0.2,
        )

        with self.assertRaisesRegex(ValueError, "source_frame"):
            csv_export.align_completed_calls(
                planner_scan((call,)),
                {call.key: input_row},
            )

    def test_input_snapshot_key_conflict_is_rejected(self):
        call = completed_call(
            generation=9,
            completed_t=0.9,
            source_frame=1009,
        )
        input_row = planner_input(
            generation=10,
            source_frame=1009,
            velocity_x=0.2,
        )

        with self.assertRaisesRegex(
            ValueError,
            "planner input key mismatch",
        ):
            csv_export.align_completed_calls(
                planner_scan((call,)),
                {call.key: input_row},
            )

    def test_input_attempt_id_conflict_is_rejected(self):
        call = completed_call(
            generation=9,
            completed_t=0.9,
            source_frame=1009,
        )
        input_row = planner_input(
            attempt_id=2,
            generation=9,
            source_frame=1009,
            velocity_x=0.2,
        )

        with self.assertRaisesRegex(
            ValueError,
            "planner input attempt_id mismatch",
        ):
            csv_export.align_completed_calls(
                planner_scan((call,)),
                {call.key: input_row},
            )

    def test_attempt_detail_identity_conflict_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            session_dir = Path(directory)
            write_attempt_detail(
                session_dir,
                attempt_id=1,
                detail_attempt_id=2,
                planner_inputs=[],
            )

            with self.assertRaisesRegex(
                ValueError,
                "attempt detail id mismatch",
            ):
                csv_export.load_attempt_inputs(session_dir, [1])

    def test_duplicate_completed_call_key_is_rejected(self):
        call = completed_call(
            generation=9,
            completed_t=0.9,
            source_frame=1009,
        )
        input_row = planner_input(
            generation=9,
            source_frame=1009,
            velocity_x=0.2,
        )

        with self.assertRaisesRegex(
            ValueError,
            "duplicate completed planner call key",
        ):
            csv_export.align_completed_calls(
                planner_scan((call, call)),
                {call.key: input_row},
            )


class ExportSessionCsvTests(unittest.TestCase):
    def test_task_observation_detail_must_match_full_events(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            session_dir = build_complete_session_fixture(root)
            detail_path = session_dir / "attempt_details" / "2.json"
            detail = json.loads(
                detail_path.read_text(encoding="utf-8")
            )
            detail["task_observation_pre_clip"][0] = 100.0
            detail_path.write_text(
                json.dumps(detail, separators=(",", ":")),
                encoding="utf-8",
            )
            output_dir = root / "analysis"
            output_dir.mkdir()
            sentinel = output_dir / "sentinel.txt"
            sentinel.write_text("keep-me", encoding="utf-8")

            with self.assertRaisesRegex(
                ValueError,
                "task observation",
            ):
                csv_export.export_session_csv(
                    session_dir=session_dir,
                    attempt_ids=[1, 2],
                    output_dir=output_dir,
                    overwrite=True,
                )

            self.assertEqual(
                sentinel.read_text(encoding="utf-8"),
                "keep-me",
            )
            self.assertEqual(list(output_dir.iterdir()), [sentinel])

    def test_task_observation_export_uses_final_preclose_full_event(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            session_dir = build_complete_session_fixture(root)
            events_path = session_dir / "events.jsonl"
            events = [
                json.loads(line)
                for line in events_path.read_text(
                    encoding="utf-8"
                ).splitlines()
            ]
            final_tick_index = next(
                index
                for index, event in enumerate(events)
                if (
                    event["kind"] == "lifecycle_tick"
                    and event["attempt_id"] == 2
                )
            )
            earlier = _lifecycle_tick_event(
                event_id=0,
                attempt_id=2,
                t=2.19,
                task_observation=(
                    [-1.0 for _index in range(11)],
                    [-1.0 for _index in range(11)],
                    11,
                ),
            )
            events.insert(
                final_tick_index,
                _complete_recorded_event(earlier, 0),
            )
            recorded_events = [
                _complete_recorded_event(event, event_id)
                for event_id, event in enumerate(events, start=1)
            ]
            write_json_lines(events_path, recorded_events)
            session_path = session_dir / "session.json"
            session = json.loads(
                session_path.read_text(encoding="utf-8")
            )
            session["watermark_event_id"] = len(recorded_events)
            session["observed_watermark_event_id"] = len(
                recorded_events
            )
            session_path.write_text(
                json.dumps(session, separators=(",", ":")),
                encoding="utf-8",
            )

            output_dir = root / "analysis"
            manifest = csv_export.export_session_csv(
                session_dir=session_dir,
                attempt_ids=[1, 2],
                output_dir=output_dir,
                overwrite=False,
            )

            with (output_dir / "task_observations.csv").open(
                "r",
                encoding="utf-8",
                newline="",
            ) as stream:
                rows = {
                    int(row["attempt_id"]): row
                    for row in csv.DictReader(stream)
                }
            self.assertEqual(rows[2]["pre_0"], "0.0")
            self.assertEqual(rows[2]["post_10"], "10.0")
            self.assertEqual(rows[2]["clip_count"], "0")
            self.assertTrue(
                manifest.completeness_checks[
                    "task_observations_from_full_events"
                ]
            )
            self.assertTrue(
                manifest.completeness_checks[
                    "task_observation_detail_found_in_full_events"
                ]
            )

    def test_task_observation_detail_cannot_select_older_history(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            session_dir = build_complete_session_fixture(root)
            events_path = session_dir / "events.jsonl"
            events = [
                json.loads(line)
                for line in events_path.read_text(
                    encoding="utf-8"
                ).splitlines()
            ]
            final_tick_index = next(
                index
                for index, event in enumerate(events)
                if (
                    event["kind"] == "lifecycle_tick"
                    and event["attempt_id"] == 2
                )
            )
            older_observation = (
                [-1.0 for _index in range(11)],
                [-1.0 for _index in range(11)],
                11,
            )
            events.insert(
                final_tick_index,
                _lifecycle_tick_event(
                    event_id=0,
                    attempt_id=2,
                    t=2.19,
                    task_observation=older_observation,
                ),
            )
            recorded_events = [
                _complete_recorded_event(event, event_id)
                for event_id, event in enumerate(events, start=1)
            ]
            write_json_lines(events_path, recorded_events)
            session_path = session_dir / "session.json"
            session = json.loads(
                session_path.read_text(encoding="utf-8")
            )
            session["watermark_event_id"] = len(recorded_events)
            session["observed_watermark_event_id"] = len(
                recorded_events
            )
            session_path.write_text(
                json.dumps(session, separators=(",", ":")),
                encoding="utf-8",
            )
            detail_path = session_dir / "attempt_details" / "2.json"
            detail = json.loads(
                detail_path.read_text(encoding="utf-8")
            )
            detail["task_observation_pre_clip"] = older_observation[0]
            detail["task_observation_post_clip"] = older_observation[1]
            detail["task_observation_clip_count"] = older_observation[2]
            detail_path.write_text(
                json.dumps(detail, separators=(",", ":")),
                encoding="utf-8",
            )

            with self.assertRaisesRegex(
                ValueError,
                "final pre-close lifecycle observation",
            ):
                csv_export.export_session_csv(
                    session_dir=session_dir,
                    attempt_ids=[1, 2],
                    output_dir=root / "analysis",
                    overwrite=False,
                )

            self.assertFalse((root / "analysis").exists())

    def test_null_task_observation_rejects_preclose_event_value(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            session_dir = build_complete_session_fixture(root)
            events_path = session_dir / "events.jsonl"
            events = [
                json.loads(line)
                for line in events_path.read_text(
                    encoding="utf-8"
                ).splitlines()
            ]
            attempt_one_tick = next(
                event
                for event in events
                if (
                    event["kind"] == "lifecycle_tick"
                    and event["attempt_id"] == 1
                )
            )
            attempt_one_tick["payload"]["task_pre_clip"] = [
                float(index) for index in range(11)
            ]
            attempt_one_tick["payload"]["task_post_clip"] = [
                float(index) for index in range(11)
            ]
            attempt_one_tick["payload"]["clip_count"] = 0
            write_json_lines(events_path, events)

            with self.assertRaisesRegex(
                ValueError,
                "has no task observation",
            ):
                csv_export.export_session_csv(
                    session_dir=session_dir,
                    attempt_ids=[1, 2],
                    output_dir=root / "analysis",
                    overwrite=False,
                )

            self.assertFalse((root / "analysis").exists())

    def test_post_close_task_observation_does_not_rewrite_snapshot(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            session_dir = build_complete_session_fixture(root)
            events_path = session_dir / "events.jsonl"
            events = [
                json.loads(line)
                for line in events_path.read_text(
                    encoding="utf-8"
                ).splitlines()
            ]
            events.append(
                _lifecycle_tick_event(
                    event_id=0,
                    attempt_id=2,
                    t=2.6,
                    task_observation=(
                        [99.0 for _index in range(11)],
                        [99.0 for _index in range(11)],
                        9,
                    ),
                )
            )
            events.append(
                _attempt_transition_event(
                    attempt_id=2,
                    track_segment_id=2,
                    stage="POST_CLOSE_DIAGNOSTIC",
                    t=2.7,
                    generation=21,
                    values={"late": True},
                )
            )
            recorded_events = [
                _complete_recorded_event(event, event_id)
                for event_id, event in enumerate(events, start=1)
            ]
            write_json_lines(events_path, recorded_events)
            session_path = session_dir / "session.json"
            session = json.loads(
                session_path.read_text(encoding="utf-8")
            )
            session["watermark_event_id"] = len(recorded_events)
            session["observed_watermark_event_id"] = len(
                recorded_events
            )
            session_path.write_text(
                json.dumps(session, separators=(",", ":")),
                encoding="utf-8",
            )

            output_dir = root / "analysis"
            csv_export.export_session_csv(
                session_dir=session_dir,
                attempt_ids=[1, 2],
                output_dir=output_dir,
                overwrite=False,
            )

            with (output_dir / "task_observations.csv").open(
                "r",
                encoding="utf-8",
                newline="",
            ) as stream:
                rows = {
                    int(row["attempt_id"]): row
                    for row in csv.DictReader(stream)
                }
            self.assertEqual(rows[2]["pre_0"], "0.0")
            self.assertEqual(rows[2]["post_10"], "10.0")
            self.assertEqual(rows[2]["clip_count"], "0")
            with (output_dir / "policy_ticks.csv").open(
                "r",
                encoding="utf-8",
                newline="",
            ) as stream:
                policy_rows = list(csv.DictReader(stream))
            self.assertNotIn(
                "2.6",
                [row["monotonic_s"] for row in policy_rows],
            )

    def test_post_close_task_observation_can_be_the_frozen_snapshot(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            session_dir = build_complete_session_fixture(root)
            events_path = session_dir / "events.jsonl"
            events = [
                json.loads(line)
                for line in events_path.read_text(
                    encoding="utf-8"
                ).splitlines()
            ]
            postclose_observation = (
                [99.0 for _index in range(11)],
                [99.0 for _index in range(11)],
                9,
            )
            events.append(
                _lifecycle_tick_event(
                    event_id=0,
                    attempt_id=2,
                    t=2.6,
                    task_observation=postclose_observation,
                )
            )
            recorded_events = [
                _complete_recorded_event(event, event_id)
                for event_id, event in enumerate(events, start=1)
            ]
            write_json_lines(events_path, recorded_events)
            session_path = session_dir / "session.json"
            session = json.loads(
                session_path.read_text(encoding="utf-8")
            )
            session["watermark_event_id"] = len(recorded_events)
            session["observed_watermark_event_id"] = len(
                recorded_events
            )
            session_path.write_text(
                json.dumps(session, separators=(",", ":")),
                encoding="utf-8",
            )
            detail_path = session_dir / "attempt_details" / "2.json"
            detail = json.loads(
                detail_path.read_text(encoding="utf-8")
            )
            detail["task_observation_pre_clip"] = (
                postclose_observation[0]
            )
            detail["task_observation_post_clip"] = (
                postclose_observation[1]
            )
            detail["task_observation_clip_count"] = (
                postclose_observation[2]
            )
            detail_path.write_text(
                json.dumps(detail, separators=(",", ":")),
                encoding="utf-8",
            )

            output_dir = root / "analysis"
            csv_export.export_session_csv(
                session_dir=session_dir,
                attempt_ids=[1, 2],
                output_dir=output_dir,
                overwrite=False,
            )

            with (output_dir / "task_observations.csv").open(
                "r",
                encoding="utf-8",
                newline="",
            ) as stream:
                rows = {
                    int(row["attempt_id"]): row
                    for row in csv.DictReader(stream)
                }
            self.assertEqual(rows[2]["pre_0"], "99.0")
            self.assertEqual(rows[2]["post_10"], "99.0")
            self.assertEqual(rows[2]["clip_count"], "9")

    def test_task_observation_event_order_and_time_must_agree(self):
        for label in (
            "event_before_close_with_postclose_time",
            "event_after_close_with_preclose_time",
        ):
            with self.subTest(label=label):
                with tempfile.TemporaryDirectory() as directory:
                    root = Path(directory)
                    session_dir = build_complete_session_fixture(root)
                    events_path = session_dir / "events.jsonl"
                    events = [
                        json.loads(line)
                        for line in events_path.read_text(
                            encoding="utf-8"
                        ).splitlines()
                    ]
                    if label.startswith("event_before"):
                        task_tick = next(
                            event
                            for event in events
                            if (
                                event["kind"] == "lifecycle_tick"
                                and event["attempt_id"] == 2
                            )
                        )
                        task_tick["monotonic_s"] = 2.6
                        task_tick["payload"]["lifecycle_now_s"] = 2.6
                    else:
                        events.append(
                            _lifecycle_tick_event(
                                event_id=0,
                                attempt_id=2,
                                t=2.4,
                                task_observation=(
                                    [
                                        float(index)
                                        for index in range(11)
                                    ],
                                    [
                                        float(index)
                                        for index in range(11)
                                    ],
                                    0,
                                ),
                            )
                        )
                    recorded_events = [
                        _complete_recorded_event(event, event_id)
                        for event_id, event in enumerate(events, start=1)
                    ]
                    write_json_lines(events_path, recorded_events)
                    session_path = session_dir / "session.json"
                    session = json.loads(
                        session_path.read_text(encoding="utf-8")
                    )
                    session["watermark_event_id"] = len(recorded_events)
                    session["observed_watermark_event_id"] = len(
                        recorded_events
                    )
                    session_path.write_text(
                        json.dumps(
                            session,
                            separators=(",", ":"),
                        ),
                        encoding="utf-8",
                    )

                    with self.assertRaisesRegex(
                        ValueError,
                        "event order and monotonic time disagree",
                    ):
                        csv_export.export_session_csv(
                            session_dir=session_dir,
                            attempt_ids=[1, 2],
                            output_dir=root / "analysis",
                            overwrite=False,
                        )

                    self.assertFalse((root / "analysis").exists())

    def test_selected_task_observation_requires_attempt_scope(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            session_dir = build_complete_session_fixture(root)
            events_path = session_dir / "events.jsonl"
            events = [
                json.loads(line)
                for line in events_path.read_text(
                    encoding="utf-8"
                ).splitlines()
            ]
            invalid_tick = _lifecycle_tick_event(
                event_id=0,
                attempt_id=2,
                t=3.5,
                task_observation=(
                    [float(index) for index in range(11)],
                    [float(index) for index in range(11)],
                    0,
                ),
            )
            invalid_tick["scope"] = "state"
            events.append(invalid_tick)
            recorded_events = [
                _complete_recorded_event(event, event_id)
                for event_id, event in enumerate(events, start=1)
            ]
            write_json_lines(events_path, recorded_events)
            session_path = session_dir / "session.json"
            session = json.loads(
                session_path.read_text(encoding="utf-8")
            )
            session["watermark_event_id"] = len(recorded_events)
            session["observed_watermark_event_id"] = len(
                recorded_events
            )
            session_path.write_text(
                json.dumps(session, separators=(",", ":")),
                encoding="utf-8",
            )

            with self.assertRaisesRegex(ValueError, "scope"):
                csv_export.export_session_csv(
                    session_dir=session_dir,
                    attempt_ids=[1, 2],
                    output_dir=root / "analysis",
                    overwrite=False,
                )

            self.assertFalse((root / "analysis").exists())

    def test_diagnostic_event_drop_marker_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            session_dir = build_complete_session_fixture(root)
            events_path = session_dir / "events.jsonl"
            events = [
                json.loads(line)
                for line in events_path.read_text(
                    encoding="utf-8"
                ).splitlines()
            ]
            events.insert(
                1,
                {
                    "schema_version": 1,
                    "kind": "DIAGNOSTIC_EVENT_DROPPED",
                    "monotonic_s": 1.001,
                    "wall_time_us": 0,
                    "scope": "state",
                    "attempt_id": None,
                    "payload": {
                        "reason_code": "PUBLISHER_QUEUE_FULL",
                        "dropped_events": 1,
                    },
                },
            )
            recorded_events = [
                _complete_recorded_event(event, event_id)
                for event_id, event in enumerate(events, start=1)
            ]
            write_json_lines(events_path, recorded_events)
            session_path = session_dir / "session.json"
            session = json.loads(
                session_path.read_text(encoding="utf-8")
            )
            session["watermark_event_id"] = len(recorded_events)
            session["observed_watermark_event_id"] = len(
                recorded_events
            )
            session_path.write_text(
                json.dumps(session, separators=(",", ":")),
                encoding="utf-8",
            )
            output_dir = root / "analysis"
            output_dir.mkdir()
            sentinel = output_dir / "sentinel.txt"
            sentinel.write_text("keep-me", encoding="utf-8")

            with self.assertRaisesRegex(
                ValueError,
                "DIAGNOSTIC_EVENT_DROPPED",
            ):
                csv_export.export_session_csv(
                    session_dir=session_dir,
                    attempt_ids=[1, 2],
                    output_dir=output_dir,
                    overwrite=True,
                )

            self.assertEqual(
                sentinel.read_text(encoding="utf-8"),
                "keep-me",
            )
            self.assertEqual(list(output_dir.iterdir()), [sentinel])

    def test_late_transition_after_persisted_detail_is_exported(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            session_dir = build_complete_session_fixture(root)
            events_path = session_dir / "events.jsonl"
            events = [
                json.loads(line)
                for line in events_path.read_text(
                    encoding="utf-8"
                ).splitlines()
            ]
            events.append(
                _attempt_transition_event(
                    attempt_id=1,
                    track_segment_id=1,
                    stage="PLANNER_SUCCEEDED",
                    t=2.10,
                    generation=11,
                    values={"late": True},
                )
            )
            recorded_events = [
                _complete_recorded_event(event, event_id)
                for event_id, event in enumerate(events, start=1)
            ]
            write_json_lines(events_path, recorded_events)
            session_path = session_dir / "session.json"
            session = json.loads(
                session_path.read_text(encoding="utf-8")
            )
            session["watermark_event_id"] = len(recorded_events)
            session["observed_watermark_event_id"] = len(
                recorded_events
            )
            session_path.write_text(
                json.dumps(session, separators=(",", ":")),
                encoding="utf-8",
            )

            output_dir = root / "analysis"
            manifest = csv_export.export_session_csv(
                session_dir=session_dir,
                attempt_ids=[1, 2],
                output_dir=output_dir,
                overwrite=False,
            )

            with (output_dir / "stage_timeline.csv").open(
                "r",
                encoding="utf-8",
                newline="",
            ) as stream:
                rows = list(csv.DictReader(stream))
            attempt_one_stages = [
                row["stage"]
                for row in rows
                if row["attempt_id"] == "1"
            ]
            self.assertEqual(
                attempt_one_stages,
                ["DETECTED", "ATTEMPT_CLOSED", "PLANNER_SUCCEEDED"],
            )
            self.assertTrue(
                manifest.completeness_checks[
                    "attempt_detail_timeline_prefix_window_verified"
                ]
            )

    def test_attempt_detail_transition_or_segment_mismatch_fails_closed(self):
        mutations = (
            (
                "timeline",
                lambda detail: detail["stage_timeline"][0].__setitem__(
                    "stage",
                    "WRONG_STAGE",
                ),
                "timeline",
            ),
            (
                "segments",
                lambda detail: detail["segments"][0].__setitem__(
                    "first_monotonic_s",
                    1.01,
                ),
                "segments",
            ),
        )
        for label, mutate, error_pattern in mutations:
            with self.subTest(label=label):
                with tempfile.TemporaryDirectory() as directory:
                    root = Path(directory)
                    session_dir = build_complete_session_fixture(root)
                    detail_path = (
                        session_dir / "attempt_details" / "1.json"
                    )
                    detail = json.loads(
                        detail_path.read_text(encoding="utf-8")
                    )
                    mutate(detail)
                    detail_path.write_text(
                        json.dumps(detail, separators=(",", ":")),
                        encoding="utf-8",
                    )
                    output_dir = root / "analysis"
                    output_dir.mkdir()
                    sentinel = output_dir / "sentinel.txt"
                    sentinel.write_text("keep-me", encoding="utf-8")

                    with self.assertRaisesRegex(
                        ValueError,
                        error_pattern,
                    ):
                        csv_export.export_session_csv(
                            session_dir=session_dir,
                            attempt_ids=[1, 2],
                            output_dir=output_dir,
                            overwrite=True,
                        )

                    self.assertEqual(
                        sentinel.read_text(encoding="utf-8"),
                        "keep-me",
                    )

    def test_transition_cross_check_is_json_type_sensitive(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            session_dir = build_complete_session_fixture(root)
            events_path = session_dir / "events.jsonl"
            events = [
                json.loads(line)
                for line in events_path.read_text(
                    encoding="utf-8"
                ).splitlines()
            ]
            detected = next(
                event
                for event in events
                if (
                    event["kind"] == "attempt_transition"
                    and event["attempt_id"] == 1
                    and event["payload"]["stage"] == "DETECTED"
                )
            )
            detected["payload"]["values"]["role"] = True
            write_json_lines(events_path, events)

            detail_path = session_dir / "attempt_details" / "1.json"
            detail = json.loads(
                detail_path.read_text(encoding="utf-8")
            )
            detail["stage_timeline"][0]["values"]["role"] = 1
            detail["segments"][0]["role"] = 1
            detail_path.write_text(
                json.dumps(detail, separators=(",", ":")),
                encoding="utf-8",
            )

            with self.assertRaisesRegex(ValueError, "timeline"):
                csv_export.export_session_csv(
                    session_dir=session_dir,
                    attempt_ids=[1, 2],
                    output_dir=root / "analysis",
                    overwrite=False,
                )

            self.assertFalse((root / "analysis").exists())

    def test_more_than_4096_transitions_restore_full_stage_and_intervals(
        self,
    ):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            session_dir = build_complete_session_fixture(root)
            events_path = session_dir / "events.jsonl"
            original_events = [
                json.loads(line)
                for line in events_path.read_text(
                    encoding="utf-8"
                ).splitlines()
            ]
            dense_transitions = [
                _attempt_transition_event(
                    attempt_id=1,
                    track_segment_id=1,
                    stage=(
                        "ATTEMPT_CLOSED"
                        if index == 4097
                        else "DENSE_{}".format(index)
                    ),
                    t=1.0 + index * 0.0001,
                    generation=index,
                    values=(
                        {"role": "PRIMARY"} if index == 0 else {}
                    ),
                )
                for index in range(4098)
            ]
            attempt_two_transitions = [
                event
                for event in original_events
                if (
                    event["kind"] == "attempt_transition"
                    and event["attempt_id"] == 2
                )
            ]
            non_transition_events = [
                event
                for event in original_events
                if event["kind"] != "attempt_transition"
            ]
            attempt_one_tick = next(
                event
                for event in non_transition_events
                if (
                    event["kind"] == "lifecycle_tick"
                    and event["attempt_id"] == 1
                )
            )
            attempt_one_tick["monotonic_s"] = 1.0001
            remaining_non_transition_events = [
                event
                for event in non_transition_events
                if event is not attempt_one_tick
            ]
            recorded_events = [
                _complete_recorded_event(event, event_id)
                for event_id, event in enumerate(
                    (
                        dense_transitions[:2]
                        + [attempt_one_tick]
                        + dense_transitions[2:]
                        + remaining_non_transition_events
                        + attempt_two_transitions
                    ),
                    start=1,
                )
            ]
            write_json_lines(events_path, recorded_events)

            session_path = session_dir / "session.json"
            session = json.loads(
                session_path.read_text(encoding="utf-8")
            )
            session["watermark_event_id"] = len(recorded_events)
            session["observed_watermark_event_id"] = len(recorded_events)
            session_path.write_text(
                json.dumps(session, separators=(",", ":")),
                encoding="utf-8",
            )

            detail_path = session_dir / "attempt_details" / "1.json"
            detail = json.loads(
                detail_path.read_text(encoding="utf-8")
            )
            detail["stage_timeline"] = [
                _detail_transition_from_event(event)
                for event in dense_transitions[-4096:]
            ]
            detail["segments"] = [
                {
                    "track_segment_id": 1,
                    "first_monotonic_s": 1.0002,
                    "last_monotonic_s": 1.4097,
                    "role": None,
                }
            ]
            detail_path.write_text(
                json.dumps(detail, separators=(",", ":")),
                encoding="utf-8",
            )

            raw_path = session_dir / "ball_samples.csv"
            with raw_path.open(
                "r",
                encoding="utf-8",
                newline="",
            ) as stream:
                reader = csv.DictReader(stream)
                fieldnames = reader.fieldnames
                raw_rows = list(reader)
            raw_rows[0]["received_monotonic_s"] = "1.0001"
            with raw_path.open(
                "w",
                encoding="utf-8",
                newline="",
            ) as stream:
                writer = csv.DictWriter(stream, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(raw_rows)

            output_dir = root / "analysis"
            manifest = csv_export.export_session_csv(
                session_dir=session_dir,
                attempt_ids=[1, 2],
                output_dir=output_dir,
                overwrite=False,
            )

            with (output_dir / "stage_timeline.csv").open(
                "r",
                encoding="utf-8",
                newline="",
            ) as stream:
                stage_rows = list(csv.DictReader(stream))
            attempt_one_stages = [
                row for row in stage_rows if row["attempt_id"] == "1"
            ]
            self.assertEqual(len(attempt_one_stages), 4098)
            self.assertEqual(attempt_one_stages[0]["stage"], "DENSE_0")
            with (output_dir / "raw_lcm.csv").open(
                "r",
                encoding="utf-8",
                newline="",
            ) as stream:
                raw_output = list(csv.DictReader(stream))
            self.assertIn(
                "1",
                [row["input_seq"] for row in raw_output],
            )
            with (output_dir / "policy_ticks.csv").open(
                "r",
                encoding="utf-8",
                newline="",
            ) as stream:
                policy_rows = list(csv.DictReader(stream))
            self.assertIn(
                "1.0001",
                [row["monotonic_s"] for row in policy_rows],
            )
            for check in (
                "full_attempt_transitions_from_events",
                "attempt_detail_timeline_prefix_window_verified",
                "segments_rebuilt_from_full_transitions",
            ):
                self.assertTrue(manifest.completeness_checks[check])

    def test_raw_input_sequence_anomalies_fail_before_replacing_output(self):
        mutations = {
            "missing": lambda rows: rows.pop(2),
            "duplicate": lambda rows: rows[2].__setitem__(
                "input_seq",
                rows[1]["input_seq"],
            ),
            "reordered": lambda rows: (
                rows[2].__setitem__("input_seq", "4"),
                rows[3].__setitem__("input_seq", "3"),
            ),
            "negative": lambda rows: rows[2].__setitem__(
                "input_seq",
                "-1",
            ),
        }
        for label, mutate in mutations.items():
            with self.subTest(label=label):
                with tempfile.TemporaryDirectory() as directory:
                    session_dir = build_complete_session_fixture(
                        Path(directory)
                    )
                    raw_path = session_dir / "ball_samples.csv"
                    with raw_path.open(
                        "r",
                        encoding="utf-8",
                        newline="",
                    ) as stream:
                        reader = csv.DictReader(stream)
                        fieldnames = reader.fieldnames
                        rows = list(reader)
                    mutate(rows)
                    with raw_path.open(
                        "w",
                        encoding="utf-8",
                        newline="",
                    ) as stream:
                        writer = csv.DictWriter(
                            stream,
                            fieldnames=fieldnames,
                        )
                        writer.writeheader()
                        writer.writerows(rows)
                    output_dir = session_dir.parent / "analysis"
                    output_dir.mkdir()
                    sentinel = output_dir / "sentinel.txt"
                    sentinel.write_text("keep-me", encoding="utf-8")

                    with self.assertRaisesRegex(
                        ValueError,
                        "input_seq",
                    ):
                        csv_export.export_session_csv(
                            session_dir=session_dir,
                            attempt_ids=[1, 2],
                            output_dir=output_dir,
                            overwrite=True,
                        )

                    self.assertEqual(
                        sentinel.read_text(encoding="utf-8"),
                        "keep-me",
                    )
                    self.assertEqual(
                        list(output_dir.iterdir()),
                        [sentinel],
                    )

    def test_raw_rows_inside_intervals_are_not_subject_filtered(self):
        with tempfile.TemporaryDirectory() as directory:
            session_dir = build_complete_session_fixture(Path(directory))
            raw_path = session_dir / "ball_samples.csv"
            with raw_path.open(
                "r",
                encoding="utf-8",
                newline="",
            ) as stream:
                reader = csv.DictReader(stream)
                fieldnames = reader.fieldnames
                rows = list(reader)
            rows[1]["subject"] = "g2pelvis"
            with raw_path.open(
                "w",
                encoding="utf-8",
                newline="",
            ) as stream:
                writer = csv.DictWriter(stream, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(rows)
            output_dir = session_dir.parent / "analysis"

            csv_export.export_session_csv(
                session_dir=session_dir,
                attempt_ids=[1, 2],
                output_dir=output_dir,
                overwrite=False,
            )

            with (output_dir / "raw_lcm.csv").open(
                "r",
                encoding="utf-8",
                newline="",
            ) as stream:
                subjects = [
                    row["subject"] for row in csv.DictReader(stream)
                ]
            self.assertIn("g2pelvis", subjects)
            self.assertEqual(len(subjects), 6)

    def test_selected_lifecycle_attempt_conflict_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            session_dir = build_complete_session_fixture(Path(directory))
            events_path = session_dir / "events.jsonl"
            events = [
                json.loads(line)
                for line in events_path.read_text(
                    encoding="utf-8"
                ).splitlines()
            ]
            tick = next(
                event
                for event in events
                if (
                    event["kind"] == "lifecycle_tick"
                    and event["attempt_id"] == 1
                )
            )
            tick["attempt_id"] = 2
            write_json_lines(events_path, events)
            output_dir = session_dir.parent / "analysis"
            output_dir.mkdir()
            sentinel = output_dir / "sentinel.txt"
            sentinel.write_text("keep-me", encoding="utf-8")

            with self.assertRaisesRegex(
                ValueError,
                "interval owner",
            ):
                csv_export.export_session_csv(
                    session_dir=session_dir,
                    attempt_ids=[1, 2],
                    output_dir=output_dir,
                    overwrite=True,
                )

            self.assertEqual(
                sentinel.read_text(encoding="utf-8"),
                "keep-me",
            )

        with tempfile.TemporaryDirectory() as directory:
            session_dir = build_complete_session_fixture(Path(directory))
            events_path = session_dir / "events.jsonl"
            events = [
                json.loads(line)
                for line in events_path.read_text(
                    encoding="utf-8"
                ).splitlines()
            ]
            tick = next(
                event
                for event in events
                if (
                    event["kind"] == "lifecycle_tick"
                    and event["attempt_id"] == 1
                )
            )
            tick["attempt_id"] = 3
            write_json_lines(events_path, events)
            output_dir = session_dir.parent / "analysis"

            csv_export.export_session_csv(
                session_dir=session_dir,
                attempt_ids=[1, 2],
                output_dir=output_dir,
                overwrite=False,
            )

            self.assertEqual(
                csv_row_count(output_dir / "policy_ticks.csv"),
                1,
            )

    def test_output_inside_session_cannot_replace_attempt_details(self):
        with tempfile.TemporaryDirectory() as directory:
            session_dir = build_complete_session_fixture(Path(directory))
            details_dir = session_dir / "attempt_details"
            detail_path = details_dir / "1.json"
            original_detail = detail_path.read_bytes()

            try:
                with self.assertRaisesRegex(
                    ValueError,
                    "overlap",
                ):
                    csv_export.export_session_csv(
                        session_dir=session_dir,
                        attempt_ids=[1, 2],
                        output_dir=details_dir,
                        overwrite=True,
                    )
            finally:
                self.assertTrue(detail_path.is_file())
                self.assertEqual(detail_path.read_bytes(), original_detail)
                self.assertFalse(
                    (details_dir / "export_manifest.json").exists()
                )

    def test_output_rejects_an_existing_intermediate_symlink(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            session_dir = build_complete_session_fixture(root)
            real_parent = root / "real-output"
            real_parent.mkdir()
            alias = root / "output-alias"
            alias.symlink_to(real_parent, target_is_directory=True)
            output_dir = alias / "nested" / "analysis"

            with self.assertRaisesRegex(ValueError, "symlink"):
                csv_export.export_session_csv(
                    session_dir=session_dir,
                    attempt_ids=[1, 2],
                    output_dir=output_dir,
                    overwrite=False,
                )

            self.assertFalse((real_parent / "nested").exists())

    def test_export_writes_complete_aligned_csv_set(self):
        with tempfile.TemporaryDirectory() as directory:
            session_dir = build_complete_session_fixture(Path(directory))
            output_dir = session_dir.parent / "analysis"

            manifest = csv_export.export_session_csv(
                session_dir=session_dir,
                attempt_ids=[1, 2],
                output_dir=output_dir,
                overwrite=False,
            )

            self.assertEqual(manifest.completed_call_count, 2)
            self.assertEqual(manifest.pending_replaced_count, 1)
            self.assertEqual(csv_row_count(output_dir / "summary.csv"), 2)
            self.assertEqual(csv_row_count(output_dir / "raw_lcm.csv"), 6)
            self.assertEqual(
                csv_row_count(output_dir / "planner_inputs.csv"),
                2,
            )
            self.assertEqual(
                csv_row_count(output_dir / "planner_calls.csv"),
                2,
            )
            self.assertEqual(
                csv_row_count(output_dir / "planner_results.csv"),
                2,
            )
            self.assertEqual(
                csv_row_count(output_dir / "stage_timeline.csv"),
                4,
            )
            self.assertEqual(
                csv_row_count(output_dir / "policy_ticks.csv"),
                2,
            )
            self.assertEqual(
                csv_row_count(output_dir / "task_observations.csv"),
                2,
            )
            self.assertEqual(
                csv_key_sequence(output_dir / "planner_inputs.csv"),
                csv_key_sequence(output_dir / "planner_calls.csv"),
            )
            self.assertEqual(
                csv_key_sequence(output_dir / "planner_calls.csv"),
                csv_key_sequence(output_dir / "planner_results.csv"),
            )
            self.assertNotIn(
                (1, 1, 1, 10),
                csv_key_sequence(output_dir / "planner_calls.csv"),
            )
            self.assertTrue((output_dir / "export_manifest.json").is_file())
            with (output_dir / "summary.csv").open(
                "r",
                encoding="utf-8",
                newline="",
            ) as stream:
                summary_rows = list(csv.DictReader(stream))
            self.assertEqual(summary_rows[0]["recording_complete"], "False")
            with (output_dir / "raw_lcm.csv").open(
                "r",
                encoding="utf-8",
                newline="",
            ) as stream:
                raw_rows = list(csv.DictReader(stream))
            pelvis_row = next(
                row for row in raw_rows if row["subject"] == "g1pelvis"
            )
            self.assertEqual(pelvis_row["attempt_id"], "")
            self.assertEqual(pelvis_row["interval_attempt_id"], "1")

    def test_validation_failure_preserves_existing_output(self):
        with tempfile.TemporaryDirectory() as directory:
            session_dir = build_complete_session_fixture(Path(directory))
            detail_path = session_dir / "attempt_details" / "1.json"
            detail = json.loads(detail_path.read_text(encoding="utf-8"))
            detail["planner_inputs"] = []
            detail_path.write_text(
                json.dumps(detail, separators=(",", ":")),
                encoding="utf-8",
            )
            output_dir = session_dir.parent / "analysis"
            output_dir.mkdir()
            sentinel = output_dir / "sentinel.txt"
            sentinel.write_text("keep-me", encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "has no input"):
                csv_export.export_session_csv(
                    session_dir=session_dir,
                    attempt_ids=[1, 2],
                    output_dir=output_dir,
                    overwrite=True,
                )

            self.assertEqual(
                sentinel.read_text(encoding="utf-8"),
                "keep-me",
            )
            self.assertEqual(list(output_dir.iterdir()), [sentinel])

    def test_incomplete_source_session_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            session_dir = build_complete_session_fixture(Path(directory))
            session_path = session_dir / "session.json"
            session = json.loads(session_path.read_text(encoding="utf-8"))
            session["status"] = "INCOMPLETE"
            session["recording_complete"] = False
            session_path.write_text(
                json.dumps(session, separators=(",", ":")),
                encoding="utf-8",
            )

            with self.assertRaisesRegex(
                ValueError,
                "status must be COMPLETE",
            ):
                csv_export.export_session_csv(
                    session_dir=session_dir,
                    attempt_ids=[1, 2],
                    output_dir=session_dir.parent / "analysis",
                    overwrite=False,
                )

            self.assertFalse((session_dir.parent / "analysis").exists())

    def test_core_csv_validation_rereads_and_rejects_key_reordering(self):
        with tempfile.TemporaryDirectory() as directory:
            session_dir = build_complete_session_fixture(Path(directory))
            output_dir = session_dir.parent / "analysis"
            manifest = csv_export.export_session_csv(
                session_dir=session_dir,
                attempt_ids=[1, 2],
                output_dir=output_dir,
                overwrite=False,
            )
            result_path = output_dir / "planner_results.csv"
            with result_path.open(
                "r",
                encoding="utf-8",
                newline="",
            ) as stream:
                rows = list(csv.reader(stream))
            with result_path.open(
                "w",
                encoding="utf-8",
                newline="",
            ) as stream:
                writer = csv.writer(stream)
                writer.writerow(rows[0])
                writer.writerows(reversed(rows[1:]))

            with self.assertRaisesRegex(
                ValueError,
                "key sequences do not match",
            ):
                csv_export.validate_written_outputs(
                    output_dir,
                    manifest.row_counts,
                )

    def test_cli_help_lists_all_arguments_and_errors_are_nonzero(self):
        environment = dict(os.environ)
        environment["PYTHONPATH"] = "."
        help_result = subprocess.run(
            [
                sys.executable,
                "-m",
                "diagnostics.export_hitter_task_csv",
                "--help",
            ],
            cwd=str(Path(__file__).resolve().parents[1]),
            env=environment,
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(help_result.returncode, 0, help_result.stderr)
        for option in (
            "--session",
            "--attempts",
            "--output",
            "--overwrite",
        ):
            self.assertIn(option, help_result.stdout)

        error_result = subprocess.run(
            [
                sys.executable,
                "-m",
                "diagnostics.export_hitter_task_csv",
                "--session",
                "/definitely/missing/hitter-session",
                "--attempts",
                "1",
                "--output",
                "/tmp/unused-hitter-export",
            ],
            cwd=str(Path(__file__).resolve().parents[1]),
            env=environment,
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertNotEqual(error_result.returncode, 0)
        self.assertIn("error:", error_result.stderr)

    def test_source_mutation_during_manifest_write_preserves_old_output(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            session_dir = build_complete_session_fixture(root)
            output_dir = root / "analysis"
            output_dir.mkdir()
            sentinel = output_dir / "sentinel.txt"
            sentinel.write_text("keep-me", encoding="utf-8")
            session_path = session_dir / "session.json"
            real_write_manifest = csv_export._write_manifest

            def write_manifest_then_mutate_source(path, manifest):
                real_write_manifest(path, manifest)
                with session_path.open("a", encoding="utf-8") as stream:
                    stream.write("\n")

            with mock.patch.object(
                csv_export,
                "_write_manifest",
                side_effect=write_manifest_then_mutate_source,
            ):
                with self.assertRaisesRegex(
                    ValueError,
                    "source file changed during export",
                ):
                    csv_export.export_session_csv(
                        session_dir=session_dir,
                        attempt_ids=[1, 2],
                        output_dir=output_dir,
                        overwrite=True,
                    )

            self.assertEqual(
                sentinel.read_text(encoding="utf-8"),
                "keep-me",
            )
            self.assertEqual(list(output_dir.iterdir()), [sentinel])

    def test_failed_directory_install_restores_existing_output(self):
        with tempfile.TemporaryDirectory() as directory:
            parent = Path(directory)
            output_dir = parent / "analysis"
            output_dir.mkdir()
            (output_dir / "sentinel.txt").write_text(
                "keep-me",
                encoding="utf-8",
            )
            temporary_dir = parent / ".hitter-csv-test"
            temporary_dir.mkdir()
            (temporary_dir / "new.txt").write_text(
                "new-data",
                encoding="utf-8",
            )
            real_replace = os.replace
            calls = []

            def fail_second_replace(source, target):
                calls.append((source, target))
                if len(calls) == 2:
                    raise OSError("install failed")
                return real_replace(source, target)

            with mock.patch.object(
                csv_export.os,
                "replace",
                side_effect=fail_second_replace,
            ):
                with self.assertRaisesRegex(OSError, "install failed"):
                    csv_export.replace_output_directory(
                        temporary_dir,
                        output_dir,
                        overwrite=True,
                    )

            self.assertEqual(
                (output_dir / "sentinel.txt").read_text(encoding="utf-8"),
                "keep-me",
            )
            self.assertTrue(temporary_dir.is_dir())
            self.assertFalse(any(parent.glob(".analysis-backup-*")))

    def test_install_fsync_failure_restores_existing_output(self):
        with tempfile.TemporaryDirectory() as directory:
            parent = Path(directory)
            output_dir = parent / "analysis"
            output_dir.mkdir()
            (output_dir / "sentinel.txt").write_text(
                "keep-me",
                encoding="utf-8",
            )
            temporary_dir = parent / ".hitter-csv-test"
            temporary_dir.mkdir()
            (temporary_dir / "new.txt").write_text(
                "new-data",
                encoding="utf-8",
            )

            with mock.patch.object(
                csv_export,
                "_fsync_directory",
                side_effect=[OSError("install fsync failed"), None],
            ):
                with self.assertRaisesRegex(
                    RuntimeError,
                    "install fsync failed",
                ):
                    csv_export.replace_output_directory(
                        temporary_dir,
                        output_dir,
                        overwrite=True,
                    )

            self.assertEqual(
                (output_dir / "sentinel.txt").read_text(encoding="utf-8"),
                "keep-me",
            )
            self.assertFalse((output_dir / "new.txt").exists())
            failed_new = list(parent.glob(".analysis-failed-new-*"))
            self.assertEqual(len(failed_new), 1)
            self.assertEqual(
                (failed_new[0] / "new.txt").read_text(encoding="utf-8"),
                "new-data",
            )
            self.assertFalse(any(parent.glob(".analysis-backup-*")))

    def test_install_fsync_failure_without_old_output_withdraws_new(self):
        with tempfile.TemporaryDirectory() as directory:
            parent = Path(directory)
            output_dir = parent / "analysis"
            temporary_dir = parent / ".hitter-csv-test"
            temporary_dir.mkdir()
            (temporary_dir / "new.txt").write_text(
                "new-data",
                encoding="utf-8",
            )

            with mock.patch.object(
                csv_export,
                "_fsync_directory",
                side_effect=[OSError("install fsync failed"), None],
            ):
                with self.assertRaisesRegex(
                    RuntimeError,
                    "install fsync failed",
                ):
                    csv_export.replace_output_directory(
                        temporary_dir,
                        output_dir,
                        overwrite=False,
                    )

            self.assertFalse(output_dir.exists())
            self.assertEqual(
                (temporary_dir / "new.txt").read_text(encoding="utf-8"),
                "new-data",
            )

    def test_backup_cleanup_failure_warns_after_commit(self):
        with tempfile.TemporaryDirectory() as directory:
            parent = Path(directory)
            output_dir = parent / "analysis"
            output_dir.mkdir()
            (output_dir / "sentinel.txt").write_text(
                "old-data",
                encoding="utf-8",
            )
            temporary_dir = parent / ".hitter-csv-test"
            temporary_dir.mkdir()
            (temporary_dir / "new.txt").write_text(
                "new-data",
                encoding="utf-8",
            )

            with mock.patch.object(
                csv_export.shutil,
                "rmtree",
                side_effect=OSError("cleanup failed"),
            ):
                with self.assertWarnsRegex(
                    RuntimeWarning,
                    "backup cleanup failed",
                ):
                    csv_export.replace_output_directory(
                        temporary_dir,
                        output_dir,
                        overwrite=True,
                    )

            self.assertEqual(
                (output_dir / "new.txt").read_text(encoding="utf-8"),
                "new-data",
            )
            self.assertFalse((output_dir / "sentinel.txt").exists())
            backups = list(parent.glob(".analysis-backup-*"))
            self.assertEqual(len(backups), 1)
            self.assertEqual(
                (backups[0] / "sentinel.txt").read_text(encoding="utf-8"),
                "old-data",
            )

    def test_backup_cleanup_warning_never_escapes_after_commit(self):
        with tempfile.TemporaryDirectory() as directory:
            parent = Path(directory)
            output_dir = parent / "analysis"
            output_dir.mkdir()
            (output_dir / "sentinel.txt").write_text(
                "old-data",
                encoding="utf-8",
            )
            temporary_dir = parent / ".hitter-csv-test"
            temporary_dir.mkdir()
            (temporary_dir / "new.txt").write_text(
                "new-data",
                encoding="utf-8",
            )

            with mock.patch.object(
                csv_export.shutil,
                "rmtree",
                side_effect=OSError("cleanup failed"),
            ), warnings.catch_warnings(), mock.patch.object(
                csv_export.sys,
                "stderr",
                io.StringIO(),
            ) as fallback_stderr:
                warnings.simplefilter("error")
                csv_export.replace_output_directory(
                    temporary_dir,
                    output_dir,
                    overwrite=True,
                )

            self.assertEqual(
                (output_dir / "new.txt").read_text(encoding="utf-8"),
                "new-data",
            )
            self.assertFalse((output_dir / "sentinel.txt").exists())
            backups = list(parent.glob(".analysis-backup-*"))
            self.assertEqual(len(backups), 1)
            self.assertEqual(
                (backups[0] / "sentinel.txt").read_text(encoding="utf-8"),
                "old-data",
            )
            self.assertIn(
                "backup cleanup failed",
                fallback_stderr.getvalue(),
            )

    def test_restore_fsync_failure_reports_restored_output_state(self):
        with tempfile.TemporaryDirectory() as directory:
            parent = Path(directory)
            output_dir = parent / "analysis"
            output_dir.mkdir()
            (output_dir / "sentinel.txt").write_text(
                "keep-me",
                encoding="utf-8",
            )
            temporary_dir = parent / ".hitter-csv-test"
            temporary_dir.mkdir()
            (temporary_dir / "new.txt").write_text(
                "new-data",
                encoding="utf-8",
            )
            real_replace = os.replace
            calls = []

            def fail_install_replace(source, target):
                calls.append((source, target))
                if len(calls) == 2:
                    raise OSError("install failed")
                return real_replace(source, target)

            with mock.patch.object(
                csv_export.os,
                "replace",
                side_effect=fail_install_replace,
            ), mock.patch.object(
                csv_export,
                "_fsync_directory",
                side_effect=OSError("restore fsync failed"),
            ):
                with self.assertRaisesRegex(
                    RuntimeError,
                    "old output restored",
                ) as raised:
                    csv_export.replace_output_directory(
                        temporary_dir,
                        output_dir,
                        overwrite=True,
                    )

            self.assertNotIn("backup preserved", str(raised.exception))
            self.assertEqual(
                (output_dir / "sentinel.txt").read_text(encoding="utf-8"),
                "keep-me",
            )
            self.assertFalse(any(parent.glob(".analysis-backup-*")))


if __name__ == "__main__":
    unittest.main()
