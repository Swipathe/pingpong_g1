from __future__ import annotations

import argparse
import bisect
import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import errno
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import stat
import sys
import tempfile
from typing import (
    Any,
    Dict,
    FrozenSet,
    Iterable,
    List,
    Mapping,
    Optional,
    Sequence,
    Tuple,
)
import uuid
import warnings


EXPORT_SCHEMA_VERSION = 1
TASK_OBSERVATION_DIMENSIONS = 11
ATTEMPT_DETAIL_TIMELINE_CAPACITY = 4096

_KEY_FIELDS = (
    "attempt_id",
    "snapshot_key.schema_version",
    "snapshot_key.track_epoch",
    "snapshot_key.generation",
)
_SUMMARY_FIELDS = (
    "schema_version",
    "attempt_id",
    "status",
    "stage",
    "primary_blocker",
    "ball_speed_mps",
    "predicted_strike_time_s",
    "planner_tts_s",
    "arm_tts_s",
    "task_obs_status",
    "ab_summary",
    "recording_complete",
    "estimator_sample_count",
    "estimator_window_size",
    "incoming_count",
    "incoming_required_count",
    "completed_call_export_complete",
    "completed_call_count",
    "pending_replaced_count",
)
_SOURCE_RAW_FIELDS = (
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
_RAW_LCM_FIELDS = (
    "schema_version",
    "interval_attempt_id",
    "interval_track_segment_id",
) + _SOURCE_RAW_FIELDS
_PLANNER_INPUT_FIELDS = _KEY_FIELDS + (
    "source_frame",
    "source_time_s",
    "received_monotonic_s",
    "position_w",
    "velocity_w",
    "base_position_w",
    "base_quaternion_xyzw",
    "base_valid",
    "visible",
    "ready",
)
_PLANNER_CALL_FIELDS = _KEY_FIELDS + (
    "submitted_monotonic_s",
    "started_monotonic_s",
    "completed_monotonic_s",
    "duration_s",
    "source_frame",
    "pending_replaced_key",
    "latest_replaced_key",
)
_PLANNER_RESULT_FIELDS = _KEY_FIELDS + (
    "source_frame",
    "strike_deadline_monotonic_s",
    "completed_monotonic_s",
    "command_fields",
    "reason_code",
    "error_type",
    "error_text",
)
_STAGE_VALUE_FIELDS = (
    "role",
    "error_type",
    "error_text",
    "count",
    "required",
    "previous_track_epoch",
    "new_track_epoch",
    "status",
    "primary_blocker",
    "arm_tts_s",
)
_STAGE_TIMELINE_FIELDS = frozenset({
    "schema_version",
    "attempt_id",
    "track_segment_id",
    "stage",
    "monotonic_s",
    "reason_code",
    "snapshot_key",
    "values",
})
_ATTEMPT_TRANSITION_PAYLOAD_FIELDS = frozenset({
    "track_segment_id",
    "stage",
    "reason_code",
    "snapshot_key",
    "values",
})
_SEGMENT_FIELDS = frozenset({
    "track_segment_id",
    "first_monotonic_s",
    "last_monotonic_s",
    "role",
})
_STAGE_FIELDS = (
    "attempt_id",
    "stage_index",
    "schema_version",
    "track_segment_id",
    "stage",
    "monotonic_s",
    "reason_code",
    "snapshot_key.schema_version",
    "snapshot_key.track_epoch",
    "snapshot_key.generation",
) + tuple("values.{}".format(field) for field in _STAGE_VALUE_FIELDS) + (
    "values_json",
)
_POLICY_TICK_FIELDS = (
    "schema_version",
    "event_id",
    "attempt_id",
    "tick_index",
    "monotonic_s",
    "wall_time_us",
    "scope",
    "lifecycle_now_s",
    "obs_now_s",
    "phase",
    "decision",
    "active_key",
    "cached_key",
    "command_result",
    "task_pre_clip",
    "task_post_clip",
    "clip_count",
    "task_pass",
    "errors",
    "recovery_duration_s",
)
_TASK_OBSERVATION_FIELDS = (
    ("attempt_id", "clip_count")
    + tuple(
        "pre_{}".format(index)
        for index in range(TASK_OBSERVATION_DIMENSIONS)
    )
    + tuple(
        "post_{}".format(index)
        for index in range(TASK_OBSERVATION_DIMENSIONS)
    )
)
_CSV_HEADERS = {
    "summary.csv": _SUMMARY_FIELDS,
    "raw_lcm.csv": _RAW_LCM_FIELDS,
    "planner_inputs.csv": _PLANNER_INPUT_FIELDS,
    "planner_calls.csv": _PLANNER_CALL_FIELDS,
    "planner_results.csv": _PLANNER_RESULT_FIELDS,
    "stage_timeline.csv": _STAGE_FIELDS,
    "policy_ticks.csv": _POLICY_TICK_FIELDS,
    "task_observations.csv": _TASK_OBSERVATION_FIELDS,
}
_REQUIRED_LIFECYCLE_PAYLOAD_FIELDS = frozenset({
    "lifecycle_now_s",
    "obs_now_s",
    "phase",
    "decision",
    "active_key",
    "cached_key",
    "command_result",
    "task_pre_clip",
    "task_post_clip",
    "clip_count",
    "task_pass",
    "errors",
    "recovery_duration_s",
})


@dataclass(frozen=True, order=True)
class PlannerExportKey:
    attempt_id: int
    schema_version: int
    track_epoch: int
    generation: int


@dataclass(frozen=True)
class CompletedPlannerCall:
    key: PlannerExportKey
    submitted_monotonic_s: float
    started_monotonic_s: float
    completed_monotonic_s: float
    source_frame: int
    result: Mapping[str, Any]
    pending_replaced_key: Optional[PlannerExportKey]
    latest_replaced_key: Optional[PlannerExportKey]


@dataclass(frozen=True)
class PlannerEventScan:
    submitted_keys: FrozenSet[PlannerExportKey]
    pending_replaced_keys: FrozenSet[PlannerExportKey]
    completed_calls: Tuple[CompletedPlannerCall, ...]


@dataclass(frozen=True)
class AlignedPlannerRow:
    call: CompletedPlannerCall
    input_row: Mapping[str, Any]


@dataclass(frozen=True)
class ExportManifest:
    schema_version: int
    source_session_dir: str
    attempt_ids: Tuple[int, ...]
    source_sha256: Mapping[str, str]
    source_file_stats: Mapping[str, Mapping[str, int]]
    row_counts: Mapping[str, int]
    core_key_sequence_sha256: str
    submitted_count: int
    pending_replaced_count: int
    completed_call_count: int
    completeness_checks: Mapping[str, bool]
    exported_at_utc: str

    def to_json_dict(self) -> Dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "source_session_dir": self.source_session_dir,
            "attempt_ids": list(self.attempt_ids),
            "source_sha256": dict(self.source_sha256),
            "source_file_stats": {
                name: dict(values)
                for name, values in self.source_file_stats.items()
            },
            "row_counts": dict(self.row_counts),
            "core_key_sequence_sha256": (
                self.core_key_sequence_sha256
            ),
            "submitted_count": self.submitted_count,
            "pending_replaced_count": self.pending_replaced_count,
            "completed_call_count": self.completed_call_count,
            "completeness_checks": dict(self.completeness_checks),
            "exported_at_utc": self.exported_at_utc,
        }


@dataclass(frozen=True)
class _SourceFingerprint:
    sha256: str
    device: int
    inode: int
    size: int
    mtime_ns: int


@dataclass(frozen=True)
class _SegmentInterval:
    attempt_id: int
    track_segment_id: Optional[int]
    first_monotonic_s: float
    last_monotonic_s: float


@dataclass(frozen=True)
class _TaskObservation:
    pre_clip: Tuple[float, ...]
    post_clip: Tuple[float, ...]
    clip_count: int


@dataclass(frozen=True)
class _LifecycleTickEvent:
    event_id: int
    monotonic_s: float
    observation: Optional[_TaskObservation]


@dataclass(frozen=True)
class _EventLogValidation:
    event_count: int
    final_event_id: int
    event_ids_contiguous: bool
    recorder_event_gap_count: int
    diagnostic_event_drop_count: int


@dataclass(frozen=True)
class _AttemptTransitionScan:
    timelines: Mapping[int, Tuple[Mapping[str, Any], ...]]
    transition_event_ids: Mapping[int, Tuple[int, ...]]
    lifecycle_events: Mapping[
        int,
        Tuple[_LifecycleTickEvent, ...],
    ]
    event_count: int
    final_event_id: int


@dataclass(frozen=True)
class _AttemptSnapshotValidation:
    segments: Mapping[int, Tuple[Mapping[str, Any], ...]]
    task_observations: Mapping[int, _TaskObservation]
    transition_cutoff_counts: Mapping[int, int]


def _planner_key_csv_fields(
    key: PlannerExportKey,
) -> Dict[str, int]:
    return {
        "attempt_id": key.attempt_id,
        "snapshot_key.schema_version": key.schema_version,
        "snapshot_key.track_epoch": key.track_epoch,
        "snapshot_key.generation": key.generation,
    }


def _compact_json(value: Any) -> str:
    return json.dumps(
        value,
        allow_nan=False,
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=True,
    )


def _csv_value(value: Any) -> Any:
    if isinstance(value, Mapping) or isinstance(value, (list, tuple)):
        return _compact_json(value)
    return value


def _optional_planner_key_json(
    key: Optional[PlannerExportKey],
) -> Optional[str]:
    if key is None:
        return None
    return _compact_json({
        "attempt_id": key.attempt_id,
        "schema_version": key.schema_version,
        "track_epoch": key.track_epoch,
        "generation": key.generation,
    })


def load_attempt_inputs(
    session_dir: Path,
    attempt_ids: Sequence[int],
) -> Mapping[PlannerExportKey, Mapping[str, Any]]:
    inputs: Dict[PlannerExportKey, Mapping[str, Any]] = {}
    details_dir = Path(session_dir) / "attempt_details"
    for attempt_value in attempt_ids:
        attempt_id = _require_int(
            attempt_value,
            field="attempt_id",
            minimum=1,
        )
        detail_path = details_dir / "{}.json".format(attempt_id)
        with detail_path.open("r", encoding="utf-8") as stream:
            detail = json.load(stream)
        if not isinstance(detail, Mapping):
            raise ValueError(
                "attempt detail must be a mapping: {}".format(
                    detail_path
                )
            )
        try:
            detail_attempt_id = _require_int(
                detail["attempt_id"],
                field="attempt detail attempt_id",
                minimum=1,
            )
        except (KeyError, ValueError) as exc:
            raise ValueError(
                "invalid attempt detail id: {}".format(detail_path)
            ) from exc
        if detail_attempt_id != attempt_id:
            raise ValueError(
                "attempt detail id mismatch: path={}, detail={}".format(
                    attempt_id,
                    detail_attempt_id,
                )
            )
        planner_inputs = detail.get("planner_inputs")
        if not isinstance(planner_inputs, list):
            raise ValueError(
                "planner_inputs must be a list for attempt {}".format(
                    attempt_id
                )
            )
        for index, value in enumerate(planner_inputs):
            if not isinstance(value, Mapping):
                raise ValueError(
                    "planner input must be a mapping: attempt={}, "
                    "index={}".format(attempt_id, index)
                )
            input_row = dict(value)
            input_attempt_value = input_row.get("attempt_id")
            if input_attempt_value is not None:
                try:
                    input_attempt_id = _require_int(
                        input_attempt_value,
                        field="planner input attempt_id",
                        minimum=1,
                    )
                except ValueError as exc:
                    raise ValueError(
                        "invalid planner input attempt_id: "
                        "attempt={}, index={}".format(
                            attempt_id,
                            index,
                        )
                    ) from exc
                if input_attempt_id != attempt_id:
                    raise ValueError(
                        "planner input attempt_id mismatch: "
                        "path={}, input={}".format(
                            attempt_id,
                            input_attempt_id,
                        )
                    )
            input_row["attempt_id"] = attempt_id
            key = _planner_key(
                attempt_id,
                input_row.get("snapshot_key"),
                field="planner input snapshot_key",
            )
            if key in inputs:
                raise ValueError(
                    "duplicate planner input key: {!r}".format(key)
                )
            for field, field_value in tuple(input_row.items()):
                if field in ("attempt_id", "snapshot_key"):
                    continue
                input_row[field] = _csv_value(field_value)
            inputs[key] = input_row
    return inputs


def align_completed_calls(
    scan: PlannerEventScan,
    inputs: Mapping[PlannerExportKey, Mapping[str, Any]],
) -> Tuple[AlignedPlannerRow, ...]:
    rows = []
    seen_keys = set()
    for call in scan.completed_calls:
        if call.key in seen_keys:
            raise ValueError(
                "duplicate completed planner call key: {!r}".format(
                    call.key
                )
            )
        seen_keys.add(call.key)
        input_row = inputs.get(call.key)
        if input_row is None:
            raise ValueError(
                "completed planner call has no input: {!r}".format(
                    call.key
                )
            )
        if not isinstance(input_row, Mapping):
            raise ValueError(
                "planner input must be a mapping for {!r}".format(
                    call.key
                )
            )
        try:
            input_attempt_id = _require_int(
                input_row["attempt_id"],
                field="planner input attempt_id",
                minimum=1,
            )
        except (KeyError, ValueError) as exc:
            raise ValueError(
                "invalid planner input attempt_id for {!r}".format(
                    call.key
                )
            ) from exc
        if input_attempt_id != call.key.attempt_id:
            raise ValueError(
                "planner input attempt_id mismatch for {!r}".format(
                    call.key
                )
            )
        input_key = _planner_key(
            input_attempt_id,
            input_row.get("snapshot_key"),
            field="planner input snapshot_key",
        )
        if input_key != call.key:
            raise ValueError(
                "planner input key mismatch for {!r}: {!r}".format(
                    call.key,
                    input_key,
                )
            )
        try:
            input_source_frame = _require_int(
                input_row["source_frame"],
                field="planner input source_frame",
                minimum=0,
            )
        except (KeyError, ValueError) as exc:
            raise ValueError(
                "invalid planner input source_frame for {!r}".format(
                    call.key
                )
            ) from exc
        if input_source_frame != call.source_frame:
            raise ValueError(
                "source_frame mismatch for {!r}".format(call.key)
            )
        rows.append(
            AlignedPlannerRow(
                call=call,
                input_row=input_row,
            )
        )
    return tuple(rows)


def planner_input_csv_rows(
    aligned: Sequence[AlignedPlannerRow],
) -> Tuple[Mapping[str, Any], ...]:
    rows = []
    for aligned_row in aligned:
        row: Dict[str, Any] = _planner_key_csv_fields(
            aligned_row.call.key
        )
        for field, value in aligned_row.input_row.items():
            if field in ("attempt_id", "snapshot_key"):
                continue
            row[field] = _csv_value(value)
        rows.append(row)
    return tuple(rows)


def planner_call_csv_rows(
    aligned: Sequence[AlignedPlannerRow],
) -> Tuple[Mapping[str, Any], ...]:
    rows = []
    for aligned_row in aligned:
        call = aligned_row.call
        row: Dict[str, Any] = _planner_key_csv_fields(call.key)
        row.update({
            "submitted_monotonic_s": call.submitted_monotonic_s,
            "started_monotonic_s": call.started_monotonic_s,
            "completed_monotonic_s": call.completed_monotonic_s,
            "duration_s": (
                call.completed_monotonic_s
                - call.started_monotonic_s
            ),
            "source_frame": call.source_frame,
            "pending_replaced_key": _optional_planner_key_json(
                call.pending_replaced_key
            ),
            "latest_replaced_key": _optional_planner_key_json(
                call.latest_replaced_key
            ),
        })
        rows.append(row)
    return tuple(rows)


def planner_result_csv_rows(
    aligned: Sequence[AlignedPlannerRow],
) -> Tuple[Mapping[str, Any], ...]:
    rows = []
    for aligned_row in aligned:
        row: Dict[str, Any] = _planner_key_csv_fields(
            aligned_row.call.key
        )
        for field, value in aligned_row.call.result.items():
            if field == "snapshot_key":
                continue
            row[field] = _csv_value(value)
        for field in (
            "command_fields",
            "reason_code",
            "error_type",
            "error_text",
        ):
            row.setdefault(field, None)
        rows.append(row)
    return tuple(rows)


def _planner_key(
    attempt_id: int,
    value: Any,
    *,
    field: str,
) -> PlannerExportKey:
    if not isinstance(value, Mapping):
        raise ValueError("{} must be a mapping".format(field))
    try:
        normalized_attempt_id = _require_int(
            attempt_id,
            field="{} attempt_id".format(field),
            minimum=1,
        )
        schema_version = _require_int(
            value["schema_version"],
            field="{} schema_version".format(field),
            minimum=0,
        )
        track_epoch = _require_int(
            value["track_epoch"],
            field="{} track_epoch".format(field),
            minimum=0,
        )
        generation = _require_int(
            value["generation"],
            field="{} generation".format(field),
            minimum=0,
        )
    except (KeyError, ValueError) as exc:
        raise ValueError("invalid {}: {!r}".format(field, value)) from exc
    if schema_version != EXPORT_SCHEMA_VERSION:
        raise ValueError(
            "invalid {} schema_version: expected={}, actual={}".format(
                field,
                EXPORT_SCHEMA_VERSION,
                schema_version,
            )
        )
    return PlannerExportKey(
        attempt_id=normalized_attempt_id,
        schema_version=schema_version,
        track_epoch=track_epoch,
        generation=generation,
    )


def _remember_unique(
    records: Dict[PlannerExportKey, Any],
    key: PlannerExportKey,
    value: Any,
    *,
    label: str,
) -> None:
    previous = records.get(key)
    if previous is not None and previous != value:
        raise ValueError("conflicting duplicate {} for {!r}".format(label, key))
    records[key] = value


def _replacement_attempt_id(
    payload: Mapping[str, Any],
    fallback_attempt_id: int,
) -> int:
    value = payload.get("replaced_attempt_id")
    if value is None:
        return _require_int(
            fallback_attempt_id,
            field="fallback replaced_attempt_id",
            minimum=1,
        )
    return _require_int(
        value,
        field="replaced_attempt_id",
        minimum=1,
    )


def scan_planner_events(
    events_path: Path,
    attempt_ids: Sequence[int],
) -> PlannerEventScan:
    selected_attempt_ids = frozenset(
        _normalize_attempt_ids(attempt_ids)
    )
    submissions: Dict[PlannerExportKey, Tuple[float, Mapping[str, Any]]] = {}
    starts: Dict[PlannerExportKey, Tuple[float, Mapping[str, Any]]] = {}
    completions: Dict[PlannerExportKey, Tuple[float, Mapping[str, Any]]] = {}
    pending_replacements: Dict[PlannerExportKey, PlannerExportKey] = {}
    latest_replacements: Dict[PlannerExportKey, PlannerExportKey] = {}
    pending_replaced_keys = set()

    with Path(events_path).open("r", encoding="utf-8") as stream:
        for line in stream:
            event = json.loads(line)
            if not isinstance(event, Mapping):
                raise ValueError("planner event must be a mapping")
            attempt_value = event.get("attempt_id")
            if attempt_value is None:
                continue
            attempt_id = _require_int(
                attempt_value,
                field="planner event attempt_id",
                minimum=1,
            )
            kind = event.get("kind")
            payload = event.get("payload")
            if (
                kind == "planner_trace"
                and isinstance(payload, Mapping)
                and payload.get("trace_kind") == "pending_replaced"
                and payload.get("replaced_attempt_id") is not None
            ):
                replaced_attempt_id = _require_int(
                    payload["replaced_attempt_id"],
                    field="replaced_attempt_id",
                    minimum=1,
                )
                if replaced_attempt_id in selected_attempt_ids:
                    pending_replaced_keys.add(
                        _planner_key(
                            replaced_attempt_id,
                            payload.get("replaced_snapshot_key"),
                            field="replaced_snapshot_key",
                        )
                    )
            if attempt_id not in selected_attempt_ids:
                continue
            if kind not in ("planner_submit", "planner_trace"):
                continue
            if not isinstance(payload, Mapping):
                raise ValueError("planner event payload must be a mapping")
            key = _planner_key(
                attempt_id,
                payload.get("snapshot_key"),
                field="snapshot_key",
            )
            monotonic_s = float(event["monotonic_s"])
            event_content = (monotonic_s, dict(payload))

            if kind == "planner_submit":
                _remember_unique(
                    submissions,
                    key,
                    event_content,
                    label="planner submit",
                )
                continue

            trace_kind = payload.get("trace_kind")
            if trace_kind == "start":
                _remember_unique(
                    starts,
                    key,
                    event_content,
                    label="planner start",
                )
                continue
            if trace_kind == "pending_replaced":
                replaced_attempt_id = _replacement_attempt_id(
                    payload,
                    attempt_id,
                )
                replaced_key = _planner_key(
                    replaced_attempt_id,
                    payload.get("replaced_snapshot_key"),
                    field="replaced_snapshot_key",
                )
                previous = pending_replacements.get(key)
                if previous is not None and previous != replaced_key:
                    raise ValueError(
                        "conflicting pending replacement for {!r}".format(key)
                    )
                pending_replacements[key] = replaced_key
                if replaced_attempt_id in selected_attempt_ids:
                    pending_replaced_keys.add(replaced_key)
                continue
            if trace_kind == "latest_result_replaced":
                replaced_result = payload.get("replaced_result")
                if not isinstance(replaced_result, Mapping):
                    raise ValueError("replaced_result must be a mapping")
                replaced_attempt_id = _replacement_attempt_id(
                    payload,
                    attempt_id,
                )
                replaced_key = _planner_key(
                    replaced_attempt_id,
                    replaced_result.get("snapshot_key"),
                    field="replaced_result.snapshot_key",
                )
                previous = latest_replacements.get(key)
                if previous is not None and previous != replaced_key:
                    raise ValueError(
                        "conflicting latest replacement for {!r}".format(key)
                    )
                latest_replacements[key] = replaced_key
                continue
            if trace_kind != "complete":
                continue

            result = payload.get("result")
            if not isinstance(result, Mapping):
                raise ValueError(
                    "planner complete result must be a mapping for {!r}".format(
                        key
                    )
                )
            result_key = _planner_key(
                attempt_id,
                result.get("snapshot_key"),
                field="result.snapshot_key",
            )
            if result_key != key:
                raise ValueError(
                    "planner complete result key mismatch for {!r}".format(key)
                )
            _remember_unique(
                completions,
                key,
                event_content,
                label="planner complete",
            )

    submitted_keys = frozenset(submissions)
    start_keys = frozenset(starts)
    complete_keys = frozenset(completions)
    frozen_pending_replaced_keys = frozenset(pending_replaced_keys)

    missing_complete_keys = sorted(start_keys - complete_keys)
    missing_start_keys = sorted(complete_keys - start_keys)
    if missing_complete_keys or missing_start_keys:
        raise ValueError(
            "planner start and complete keys differ: "
            "missing_complete={!r}, missing_start={!r}".format(
                missing_complete_keys,
                missing_start_keys,
            )
        )

    unsubmitted_start_keys = sorted(start_keys - submitted_keys)
    unsubmitted_complete_keys = sorted(complete_keys - submitted_keys)
    if unsubmitted_start_keys or unsubmitted_complete_keys:
        raise ValueError(
            "planner start/complete keys lack submissions: "
            "starts={!r}, completions={!r}".format(
                unsubmitted_start_keys,
                unsubmitted_complete_keys,
            )
        )

    pending_started_keys = sorted(
        frozen_pending_replaced_keys & start_keys
    )
    pending_completed_keys = sorted(
        frozen_pending_replaced_keys & complete_keys
    )
    if pending_started_keys or pending_completed_keys:
        raise ValueError(
            "completed and pending-replaced keys overlap: {!r}; "
            "started and pending-replaced keys overlap: {!r}".format(
                pending_completed_keys,
                pending_started_keys,
            )
        )

    completed_keys = start_keys
    completed_calls = []
    for key in completed_keys:
        submitted_monotonic_s = _require_finite_float(
            submissions[key][0],
            field="planner submitted_monotonic_s",
        )
        started_monotonic_s = _require_finite_float(
            starts[key][0],
            field="planner started_monotonic_s",
        )
        completion_payload = completions[key][1]
        result = dict(completion_payload["result"])
        completed_monotonic_s = _require_finite_float(
            result.get("completed_monotonic_s"),
            field="planner completed_monotonic_s",
        )
        if not (
            submitted_monotonic_s
            <= started_monotonic_s
            <= completed_monotonic_s
        ):
            raise ValueError(
                "planner timestamps are out of order for {!r}".format(
                    key
                )
            )
        completed_calls.append(
            CompletedPlannerCall(
                key=key,
                submitted_monotonic_s=submitted_monotonic_s,
                started_monotonic_s=started_monotonic_s,
                completed_monotonic_s=completed_monotonic_s,
                source_frame=_require_int(
                    result.get("source_frame"),
                    field="planner result source_frame",
                    minimum=0,
                ),
                result=result,
                pending_replaced_key=pending_replacements.get(key),
                latest_replaced_key=latest_replacements.get(key),
            )
        )
    completed_calls.sort(
        key=lambda call: (
            call.completed_monotonic_s,
            call.key,
        )
    )
    classified_keys = completed_keys | frozen_pending_replaced_keys
    if classified_keys != submitted_keys:
        missing = sorted(submitted_keys - classified_keys)
        unexpected = sorted(classified_keys - submitted_keys)
        raise ValueError(
            "submitted keys are neither completed nor pending-replaced: "
            "missing={!r}, unexpected={!r}".format(missing, unexpected)
        )

    return PlannerEventScan(
        submitted_keys=submitted_keys,
        pending_replaced_keys=frozen_pending_replaced_keys,
        completed_calls=tuple(completed_calls),
    )


def _require_finite_float(value: Any, *, field: str) -> float:
    if isinstance(value, bool):
        raise ValueError("{} must be a finite number".format(field))
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            "{} must be a finite number".format(field)
        ) from exc
    if not math.isfinite(result):
        raise ValueError("{} must be finite".format(field))
    return result


def _require_int(value: Any, *, field: str, minimum: int = 0) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError("{} must be an integer".format(field))
    result = value
    if result < minimum:
        raise ValueError(
            "{} must be at least {}".format(field, minimum)
        )
    return result


def _normalize_attempt_ids(
    attempt_ids: Sequence[int],
) -> Tuple[int, ...]:
    normalized: List[int] = []
    seen = set()
    for value in attempt_ids:
        attempt_id = _require_int(
            value,
            field="attempt_id",
            minimum=1,
        )
        if attempt_id in seen:
            raise ValueError(
                "duplicate attempt_id: {}".format(attempt_id)
            )
        seen.add(attempt_id)
        normalized.append(attempt_id)
    if not normalized:
        raise ValueError("at least one attempt_id is required")
    return tuple(sorted(normalized))


def _read_regular_file_bytes(path: Path) -> Tuple[bytes, os.stat_result]:
    try:
        before = os.stat(str(path), follow_symlinks=False)
    except FileNotFoundError as exc:
        raise ValueError(
            "required source file is missing: {}".format(path)
        ) from exc
    if not stat.S_ISREG(before.st_mode):
        raise ValueError(
            "source path must be a regular file: {}".format(path)
        )
    flags = os.O_RDONLY
    flags |= getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(str(path), flags)
    except OSError as exc:
        if exc.errno == errno.ELOOP:
            raise ValueError(
                "source path must not be a symlink: {}".format(path)
            ) from exc
        raise
    try:
        opened = os.fstat(descriptor)
        if (
            not stat.S_ISREG(opened.st_mode)
            or opened.st_dev != before.st_dev
            or opened.st_ino != before.st_ino
        ):
            raise ValueError(
                "source file changed while opening: {}".format(path)
            )
        chunks = []
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            chunks.append(chunk)
        after = os.fstat(descriptor)
    finally:
        os.close(descriptor)
    if (
        opened.st_dev != after.st_dev
        or opened.st_ino != after.st_ino
        or opened.st_size != after.st_size
        or opened.st_mtime_ns != after.st_mtime_ns
    ):
        raise ValueError(
            "source file changed while reading: {}".format(path)
        )
    return b"".join(chunks), after


def _fingerprint_source_file(path: Path) -> _SourceFingerprint:
    digest = hashlib.sha256()
    try:
        before = os.stat(str(path), follow_symlinks=False)
    except FileNotFoundError as exc:
        raise ValueError(
            "required source file is missing: {}".format(path)
        ) from exc
    if not stat.S_ISREG(before.st_mode):
        raise ValueError(
            "source path must be a regular file: {}".format(path)
        )
    flags = os.O_RDONLY
    flags |= getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(str(path), flags)
    except OSError as exc:
        if exc.errno == errno.ELOOP:
            raise ValueError(
                "source path must not be a symlink: {}".format(path)
            ) from exc
        raise
    try:
        opened = os.fstat(descriptor)
        if (
            not stat.S_ISREG(opened.st_mode)
            or opened.st_dev != before.st_dev
            or opened.st_ino != before.st_ino
        ):
            raise ValueError(
                "source file changed while opening: {}".format(path)
            )
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
        after = os.fstat(descriptor)
    finally:
        os.close(descriptor)
    if (
        opened.st_dev != after.st_dev
        or opened.st_ino != after.st_ino
        or opened.st_size != after.st_size
        or opened.st_mtime_ns != after.st_mtime_ns
    ):
        raise ValueError(
            "source file changed while hashing: {}".format(path)
        )
    return _SourceFingerprint(
        sha256=digest.hexdigest(),
        device=after.st_dev,
        inode=after.st_ino,
        size=after.st_size,
        mtime_ns=after.st_mtime_ns,
    )


def _load_json_mapping(path: Path, *, label: str) -> Mapping[str, Any]:
    raw, _file_stat = _read_regular_file_bytes(path)
    try:
        value = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(
            "{} is not valid UTF-8 JSON: {}".format(label, path)
        ) from exc
    if not isinstance(value, Mapping):
        raise ValueError("{} must be a JSON object".format(label))
    return value


def _validate_session_metadata(
    session: Mapping[str, Any],
) -> int:
    if session.get("schema_version") != EXPORT_SCHEMA_VERSION:
        raise ValueError("unsupported session schema_version")
    if session.get("status") != "COMPLETE":
        raise ValueError("source session status must be COMPLETE")
    if session.get("recording_complete") is not True:
        raise ValueError(
            "source session recording_complete must be true"
        )
    if session.get("recorder_healthy") is not True:
        raise ValueError("source session recorder_healthy must be true")
    if session.get("last_error") is not None:
        raise ValueError("source session last_error must be null")
    if session.get("failure") is not None:
        raise ValueError("source session failure must be null")
    dropped = _require_int(
        session.get("input_samples_dropped"),
        field="input_samples_dropped",
        minimum=0,
    )
    if dropped != 0:
        raise ValueError("source session has dropped input samples")
    persisted = _require_int(
        session.get("watermark_event_id"),
        field="watermark_event_id",
        minimum=0,
    )
    observed = _require_int(
        session.get("observed_watermark_event_id"),
        field="observed_watermark_event_id",
        minimum=0,
    )
    if persisted != observed:
        raise ValueError(
            "source session event watermarks do not match"
        )
    return persisted


def _load_attempt_details(
    session_dir: Path,
    attempt_ids: Sequence[int],
) -> Mapping[int, Mapping[str, Any]]:
    details: Dict[int, Mapping[str, Any]] = {}
    for attempt_id in attempt_ids:
        path = (
            session_dir
            / "attempt_details"
            / "{}.json".format(attempt_id)
        )
        detail = _load_json_mapping(
            path,
            label="attempt detail {}".format(attempt_id),
        )
        if detail.get("schema_version") != EXPORT_SCHEMA_VERSION:
            raise ValueError(
                "unsupported attempt detail schema: {}".format(path)
            )
        detail_id = _require_int(
            detail.get("attempt_id"),
            field="attempt detail attempt_id",
            minimum=1,
        )
        if detail_id != attempt_id:
            raise ValueError(
                "attempt detail id mismatch: path={}, detail={}".format(
                    attempt_id,
                    detail_id,
                )
            )
        summary = detail.get("summary")
        if not isinstance(summary, Mapping):
            raise ValueError(
                "attempt detail summary must be a mapping: {}".format(
                    attempt_id
                )
            )
        summary_id = _require_int(
            summary.get("attempt_id"),
            field="attempt summary attempt_id",
            minimum=1,
        )
        if summary_id != attempt_id:
            raise ValueError(
                "attempt summary id mismatch: detail={}, summary={}".format(
                    attempt_id,
                    summary_id,
                )
            )
        for field in ("segments", "stage_timeline", "planner_inputs"):
            if not isinstance(detail.get(field), list):
                raise ValueError(
                    "{} must be a list for attempt {}".format(
                        field,
                        attempt_id,
                    )
                )
        for field in (
            "task_observation_pre_clip",
            "task_observation_post_clip",
            "task_observation_clip_count",
        ):
            if field not in detail:
                raise ValueError(
                    "{} is missing for attempt {}".format(
                        field,
                        attempt_id,
                    )
                )
        details[attempt_id] = detail
    return details


def _normalize_stage_timeline_row(
    value: Any,
    *,
    expected_attempt_id: int,
    label: str,
) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError("{} must be a mapping".format(label))
    if frozenset(value) != _STAGE_TIMELINE_FIELDS:
        raise ValueError(
            "{} fields mismatch: missing={!r}, unexpected={!r}".format(
                label,
                sorted(_STAGE_TIMELINE_FIELDS - frozenset(value)),
                sorted(frozenset(value) - _STAGE_TIMELINE_FIELDS),
            )
        )
    schema_version = _require_int(
        value.get("schema_version"),
        field="{} schema_version".format(label),
        minimum=0,
    )
    if schema_version != EXPORT_SCHEMA_VERSION:
        raise ValueError(
            "{} has unsupported schema_version {}".format(
                label,
                schema_version,
            )
        )
    attempt_id = _require_int(
        value.get("attempt_id"),
        field="{} attempt_id".format(label),
        minimum=1,
    )
    if attempt_id != expected_attempt_id:
        raise ValueError(
            "{} attempt_id mismatch: expected={}, actual={}".format(
                label,
                expected_attempt_id,
                attempt_id,
            )
        )
    monotonic_s = _require_finite_float(
        value.get("monotonic_s"),
        field="{} monotonic_s".format(label),
    )
    track_segment_id = value.get("track_segment_id")
    if track_segment_id is not None:
        track_segment_id = _require_int(
            track_segment_id,
            field="{} track_segment_id".format(label),
            minimum=1,
        )
    stage = value.get("stage")
    if not isinstance(stage, str) or not stage:
        raise ValueError("{} stage must be a non-empty string".format(label))
    reason_code = value.get("reason_code")
    if reason_code is not None and not isinstance(reason_code, str):
        raise ValueError(
            "{} reason_code must be a string or null".format(label)
        )
    snapshot = value.get("snapshot_key")
    if snapshot is None:
        snapshot_value = None
    else:
        snapshot_key = _planner_key(
            attempt_id,
            snapshot,
            field="{} snapshot_key".format(label),
        )
        snapshot_value = {
            "schema_version": snapshot_key.schema_version,
            "track_epoch": snapshot_key.track_epoch,
            "generation": snapshot_key.generation,
        }
    values = value.get("values")
    if not isinstance(values, Mapping):
        raise ValueError("{} values must be a mapping".format(label))
    return {
        "schema_version": schema_version,
        "attempt_id": attempt_id,
        "track_segment_id": track_segment_id,
        "stage": stage,
        "monotonic_s": monotonic_s,
        "snapshot_key": snapshot_value,
        "reason_code": reason_code,
        "values": dict(values),
    }


def _segments_from_timeline(
    attempt_id: int,
    timeline: Sequence[Mapping[str, Any]],
    *,
    label: str,
) -> Tuple[Mapping[str, Any], ...]:
    segment_values: Dict[int, Dict[str, Any]] = {}
    for index, transition in enumerate(timeline):
        segment_id = transition["track_segment_id"]
        if segment_id is None:
            continue
        monotonic_s = _require_finite_float(
            transition["monotonic_s"],
            field="{} transition {} monotonic_s".format(label, index),
        )
        segment = segment_values.get(segment_id)
        if segment is None:
            segment = {
                "track_segment_id": segment_id,
                "first_monotonic_s": monotonic_s,
                "last_monotonic_s": monotonic_s,
                "role": None,
            }
            segment_values[segment_id] = segment
        segment["last_monotonic_s"] = monotonic_s
        role = transition["values"].get("role")
        if role is not None:
            segment["role"] = role
    return tuple(
        segment_values[key] for key in sorted(segment_values)
    )


def _scan_attempt_transitions(
    events_path: Path,
    attempt_ids: Sequence[int],
    *,
    expected_watermark_event_id: int,
) -> _AttemptTransitionScan:
    selected_attempt_ids = frozenset(
        _normalize_attempt_ids(attempt_ids)
    )
    timelines: Dict[int, List[Mapping[str, Any]]] = {
        attempt_id: [] for attempt_id in selected_attempt_ids
    }
    transition_event_ids: Dict[int, List[int]] = {
        attempt_id: [] for attempt_id in selected_attempt_ids
    }
    lifecycle_events: Dict[
        int,
        List[_LifecycleTickEvent],
    ] = {
        attempt_id: [] for attempt_id in selected_attempt_ids
    }
    expected_event_id = 1
    event_count = 0
    with Path(events_path).open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                raise ValueError(
                    "events.jsonl has a blank line at {}".format(
                        line_number
                    )
                )
            try:
                event = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(
                    "events.jsonl has invalid JSON at line {}".format(
                        line_number
                    )
                ) from exc
            if not isinstance(event, Mapping):
                raise ValueError(
                    "events.jsonl event must be a mapping at line {}".format(
                        line_number
                    )
                )
            kind = event.get("kind")
            if kind == "RECORDER_EVENT_GAP":
                raise ValueError(
                    "events.jsonl contains RECORDER_EVENT_GAP"
                )
            if kind == "DIAGNOSTIC_EVENT_DROPPED":
                raise ValueError(
                    "events.jsonl contains DIAGNOSTIC_EVENT_DROPPED"
                )
            if event.get("schema_version") != EXPORT_SCHEMA_VERSION:
                raise ValueError(
                    "unsupported event schema at line {}".format(
                        line_number
                    )
                )
            event_id = _require_int(
                event.get("event_id"),
                field="event_id at line {}".format(line_number),
                minimum=1,
            )
            if event_id != expected_event_id:
                raise ValueError(
                    "events.jsonl event_id gap or reorder at line {}: "
                    "expected={}, actual={}".format(
                        line_number,
                        expected_event_id,
                        event_id,
                    )
                )
            expected_event_id += 1
            event_count += 1

            if kind == "lifecycle_tick":
                attempt_value = event.get("attempt_id")
                if attempt_value is None:
                    continue
                lifecycle_attempt_id = _require_int(
                    attempt_value,
                    field=(
                        "lifecycle_tick attempt_id at line {}".format(
                            line_number
                        )
                    ),
                    minimum=1,
                )
                if lifecycle_attempt_id not in selected_attempt_ids:
                    continue
                if event.get("scope") != "attempt":
                    raise ValueError(
                        "selected lifecycle_tick scope must be attempt"
                    )
                monotonic_s = _require_finite_float(
                    event.get("monotonic_s"),
                    field=(
                        "lifecycle_tick monotonic_s at line {}".format(
                            line_number
                        )
                    ),
                )
                _require_int(
                    event.get("wall_time_us"),
                    field=(
                        "lifecycle_tick wall_time_us at line {}".format(
                            line_number
                        )
                    ),
                    minimum=0,
                )
                payload = event.get("payload")
                if not isinstance(payload, Mapping):
                    raise ValueError(
                        "lifecycle_tick payload must be a mapping"
                    )
                payload_fields = frozenset(payload)
                if payload_fields != _REQUIRED_LIFECYCLE_PAYLOAD_FIELDS:
                    raise ValueError(
                        "lifecycle_tick payload fields mismatch at line "
                        "{}: missing={!r}, unexpected={!r}".format(
                            line_number,
                            sorted(
                                _REQUIRED_LIFECYCLE_PAYLOAD_FIELDS
                                - payload_fields
                            ),
                            sorted(
                                payload_fields
                                - _REQUIRED_LIFECYCLE_PAYLOAD_FIELDS
                            ),
                        )
                    )
                observation = _normalize_task_observation(
                    payload.get("task_pre_clip"),
                    payload.get("task_post_clip"),
                    payload.get("clip_count"),
                    label=(
                        "lifecycle_tick event_id={}".format(event_id)
                    ),
                )
                lifecycle_events[lifecycle_attempt_id].append(
                    _LifecycleTickEvent(
                        event_id=event_id,
                        monotonic_s=monotonic_s,
                        observation=observation,
                    )
                )
                continue

            if kind != "attempt_transition":
                continue
            attempt_id = _require_int(
                event.get("attempt_id"),
                field=(
                    "attempt_transition attempt_id at line {}".format(
                        line_number
                    )
                ),
                minimum=1,
            )
            if attempt_id not in selected_attempt_ids:
                continue
            if event.get("scope") != "attempt":
                raise ValueError(
                    "selected attempt_transition scope must be attempt"
                )
            payload = event.get("payload")
            if not isinstance(payload, Mapping):
                raise ValueError(
                    "attempt_transition payload must be a mapping"
                )
            if frozenset(payload) != _ATTEMPT_TRANSITION_PAYLOAD_FIELDS:
                raise ValueError(
                    "attempt_transition payload fields mismatch at line "
                    "{}: missing={!r}, unexpected={!r}".format(
                        line_number,
                        sorted(
                            _ATTEMPT_TRANSITION_PAYLOAD_FIELDS
                            - frozenset(payload)
                        ),
                        sorted(
                            frozenset(payload)
                            - _ATTEMPT_TRANSITION_PAYLOAD_FIELDS
                        ),
                    )
                )
            transition = _normalize_stage_timeline_row(
                {
                    "schema_version": event.get("schema_version"),
                    "attempt_id": attempt_id,
                    "track_segment_id": payload.get(
                        "track_segment_id"
                    ),
                    "stage": payload.get("stage"),
                    "monotonic_s": event.get("monotonic_s"),
                    "snapshot_key": payload.get("snapshot_key"),
                    "reason_code": payload.get("reason_code"),
                    "values": payload.get("values"),
                },
                expected_attempt_id=attempt_id,
                label=(
                    "attempt_transition at line {}".format(
                        line_number
                    )
                ),
            )
            timelines[attempt_id].append(transition)
            transition_event_ids[attempt_id].append(event_id)
    final_event_id = expected_event_id - 1
    if final_event_id != expected_watermark_event_id:
        raise ValueError(
            "events.jsonl final event_id does not match session "
            "watermark: final={}, watermark={}".format(
                final_event_id,
                expected_watermark_event_id,
            )
        )
    frozen_timelines = {
        attempt_id: tuple(timelines[attempt_id])
        for attempt_id in sorted(timelines)
    }
    frozen_lifecycle_events = {
        attempt_id: tuple(lifecycle_events[attempt_id])
        for attempt_id in sorted(lifecycle_events)
    }
    frozen_transition_event_ids = {
        attempt_id: tuple(transition_event_ids[attempt_id])
        for attempt_id in sorted(transition_event_ids)
    }
    return _AttemptTransitionScan(
        timelines=frozen_timelines,
        transition_event_ids=frozen_transition_event_ids,
        lifecycle_events=frozen_lifecycle_events,
        event_count=event_count,
        final_event_id=final_event_id,
    )


def _normalize_detail_segments(
    values: Any,
    *,
    attempt_id: int,
) -> Tuple[Mapping[str, Any], ...]:
    if not isinstance(values, list):
        raise ValueError(
            "segments must be a list for attempt {}".format(attempt_id)
        )
    normalized = []
    seen_segment_ids = set()
    for index, value in enumerate(values):
        label = "attempt {} segment {}".format(attempt_id, index)
        if not isinstance(value, Mapping):
            raise ValueError("{} must be a mapping".format(label))
        if frozenset(value) != _SEGMENT_FIELDS:
            raise ValueError(
                "{} fields mismatch: missing={!r}, unexpected={!r}".format(
                    label,
                    sorted(_SEGMENT_FIELDS - frozenset(value)),
                    sorted(frozenset(value) - _SEGMENT_FIELDS),
                )
            )
        segment_id = _require_int(
            value.get("track_segment_id"),
            field="{} track_segment_id".format(label),
            minimum=1,
        )
        if segment_id in seen_segment_ids:
            raise ValueError(
                "duplicate track_segment_id {} for attempt {}".format(
                    segment_id,
                    attempt_id,
                )
            )
        seen_segment_ids.add(segment_id)
        first = _require_finite_float(
            value.get("first_monotonic_s"),
            field="{} first_monotonic_s".format(label),
        )
        last = _require_finite_float(
            value.get("last_monotonic_s"),
            field="{} last_monotonic_s".format(label),
        )
        if first > last:
            raise ValueError(
                "{} first_monotonic_s exceeds last_monotonic_s".format(
                    label
                )
            )
        normalized.append({
            "track_segment_id": segment_id,
            "first_monotonic_s": first,
            "last_monotonic_s": last,
            "role": value.get("role"),
        })
    return tuple(normalized)


def _canonical_sequence_window_ends(
    full_values: Sequence[Mapping[str, Any]],
    window_values: Sequence[Mapping[str, Any]],
) -> Tuple[int, ...]:
    full = tuple(_compact_json(value) for value in full_values)
    window = tuple(_compact_json(value) for value in window_values)
    if not window:
        return ()
    prefix_lengths = [0] * len(window)
    matched = 0
    for index in range(1, len(window)):
        while matched and window[index] != window[matched]:
            matched = prefix_lengths[matched - 1]
        if window[index] == window[matched]:
            matched += 1
            prefix_lengths[index] = matched
    matched = 0
    ends = []
    for index, value in enumerate(full):
        while matched and value != window[matched]:
            matched = prefix_lengths[matched - 1]
        if value == window[matched]:
            matched += 1
            if matched == len(window):
                ends.append(index + 1)
                matched = prefix_lengths[matched - 1]
    return tuple(ends)


def _validate_attempt_detail_transition_windows(
    details: Mapping[int, Mapping[str, Any]],
    transition_scan: _AttemptTransitionScan,
) -> _AttemptSnapshotValidation:
    segments_by_attempt: Dict[
        int,
        Tuple[Mapping[str, Any], ...],
    ] = {}
    task_observations: Dict[int, _TaskObservation] = {}
    transition_cutoff_counts: Dict[int, int] = {}
    for attempt_id in sorted(details):
        full_timeline = transition_scan.timelines[attempt_id]
        detail_timeline = tuple(
            _normalize_stage_timeline_row(
                value,
                expected_attempt_id=attempt_id,
                label="attempt {} detail timeline row {}".format(
                    attempt_id,
                    index,
                ),
            )
            for index, value in enumerate(
                details[attempt_id]["stage_timeline"]
            )
        )
        if not detail_timeline:
            raise ValueError(
                "attempt {} detail stage timeline is empty".format(
                    attempt_id
                )
            )
        if len(detail_timeline) > ATTEMPT_DETAIL_TIMELINE_CAPACITY:
            raise ValueError(
                "attempt {} detail stage timeline exceeds capacity "
                "{}".format(
                    attempt_id,
                    ATTEMPT_DETAIL_TIMELINE_CAPACITY,
                )
            )
        if len(detail_timeline) > len(full_timeline):
            raise ValueError(
                "attempt {} detail stage timeline is longer than the "
                "complete event timeline".format(attempt_id)
            )
        close_transition_indices = tuple(
            index
            for index, value in enumerate(detail_timeline)
            if value["stage"] == "ATTEMPT_CLOSED"
        )
        if not close_transition_indices:
            raise ValueError(
                "attempt {} detail stage timeline has no "
                "ATTEMPT_CLOSED marker".format(attempt_id)
            )
        if len(close_transition_indices) != 1:
            raise ValueError(
                "attempt {} detail stage timeline has an ambiguous "
                "ATTEMPT_CLOSED boundary".format(attempt_id)
            )
        if len(detail_timeline) < ATTEMPT_DETAIL_TIMELINE_CAPACITY:
            expected_prefix = full_timeline[:len(detail_timeline)]
            if (
                tuple(
                    _compact_json(value) for value in detail_timeline
                )
                != tuple(
                    _compact_json(value) for value in expected_prefix
                )
            ):
                cutoff_count = None
            else:
                cutoff_count = len(detail_timeline)
        else:
            matching_ends = _canonical_sequence_window_ends(
                full_timeline,
                detail_timeline,
            )
            if len(matching_ends) > 1:
                raise ValueError(
                    "attempt {} detail stage timeline has an ambiguous "
                    "complete-event prefix boundary: {!r}".format(
                        attempt_id,
                        matching_ends,
                    )
                )
            cutoff_count = (
                None if not matching_ends else matching_ends[0]
            )
        if cutoff_count is None:
            raise ValueError(
                "attempt {} detail stage timeline is not a canonical "
                "bounded suffix of a complete-event prefix: "
                "detail_count={}, full_count={}".format(
                    attempt_id,
                    len(detail_timeline),
                    len(full_timeline),
                )
            )
        detail_segments = _normalize_detail_segments(
            details[attempt_id]["segments"],
            attempt_id=attempt_id,
        )
        expected_detail_segments = _segments_from_timeline(
            attempt_id,
            detail_timeline,
            label="attempt detail timeline prefix window",
        )
        if _compact_json(list(detail_segments)) != _compact_json(
            list(expected_detail_segments)
        ):
            raise ValueError(
                "attempt {} detail segments do not match its timeline "
                "prefix window".format(attempt_id)
            )
        transition_cutoff_counts[attempt_id] = cutoff_count
        segments_by_attempt[attempt_id] = _segments_from_timeline(
            attempt_id,
            full_timeline[:cutoff_count],
            label="complete attempt transition snapshot prefix",
        )
        transition_ids = transition_scan.transition_event_ids[
            attempt_id
        ]
        if len(transition_ids) != len(full_timeline):
            raise ValueError(
                "attempt {} transition event ids are not aligned with "
                "the complete timeline".format(attempt_id)
            )
        detail_window_start = cutoff_count - len(detail_timeline)
        close_timeline_index = (
            detail_window_start + close_transition_indices[0]
        )
        close_event_id = transition_ids[close_timeline_index]

        detail_observation = _detail_task_observation(
            details[attempt_id],
            attempt_id=attempt_id,
        )
        lifecycle_events = transition_scan.lifecycle_events[attempt_id]
        close_monotonic_s = detail_timeline[
            close_transition_indices[0]
        ]["monotonic_s"]
        for lifecycle_event in lifecycle_events:
            if (
                lifecycle_event.event_id < close_event_id
                and lifecycle_event.monotonic_s > close_monotonic_s
            ) or (
                lifecycle_event.event_id > close_event_id
                and lifecycle_event.monotonic_s < close_monotonic_s
            ):
                raise ValueError(
                    "attempt {} lifecycle event order and "
                    "monotonic time disagree around ATTEMPT_CLOSED".format(
                        attempt_id
                    )
                )
            if lifecycle_event.event_id == close_event_id:
                raise ValueError(
                    "attempt {} lifecycle tick reuses the "
                    "ATTEMPT_CLOSED event id".format(attempt_id)
                )
        observation_events = [
            value
            for value in lifecycle_events
            if value.observation is not None
        ]
        preclose_observations = [
            value
            for value in observation_events
            if value.event_id < close_event_id
        ]
        postclose_observations = [
            value
            for value in observation_events
            if value.event_id > close_event_id
        ]
        if detail_observation is None:
            if preclose_observations:
                raise ValueError(
                    "attempt {} detail has no task observation but full "
                    "lifecycle events contain one at or before the "
                    "persisted ATTEMPT_CLOSED boundary".format(attempt_id)
                )
            continue
        if preclose_observations:
            final_preclose_observation = (
                preclose_observations[-1].observation
            )
            if final_preclose_observation == detail_observation:
                task_observations[attempt_id] = (
                    final_preclose_observation
                )
                continue
        matching_observations = [
            value
            for value in postclose_observations
            if value.observation == detail_observation
        ]
        if not matching_observations:
            raise ValueError(
                "attempt {} task observation detail matches neither the "
                "final pre-close lifecycle observation nor an "
                "observation in the close-to-freeze race window".format(
                    attempt_id
                )
            )
        task_observations[attempt_id] = (
            matching_observations[-1].observation
        )
    return _AttemptSnapshotValidation(
        segments=segments_by_attempt,
        task_observations=task_observations,
        transition_cutoff_counts=transition_cutoff_counts,
    )


def _merged_segment_intervals(
    segments_by_attempt: Mapping[
        int,
        Sequence[Mapping[str, Any]],
    ],
) -> Tuple[_SegmentInterval, ...]:
    intervals = []
    for attempt_id, segments in segments_by_attempt.items():
        if not segments:
            raise ValueError(
                "attempt {} has no track segments".format(attempt_id)
            )
        for index, value in enumerate(segments):
            if not isinstance(value, Mapping):
                raise ValueError(
                    "segment must be a mapping: attempt={}, index={}".format(
                        attempt_id,
                        index,
                    )
                )
            segment_id = _require_int(
                value.get("track_segment_id"),
                field="track_segment_id",
                minimum=1,
            )
            first = _require_finite_float(
                value.get("first_monotonic_s"),
                field="segment first_monotonic_s",
            )
            last = _require_finite_float(
                value.get("last_monotonic_s"),
                field="segment last_monotonic_s",
            )
            if first > last:
                raise ValueError(
                    "segment first_monotonic_s exceeds "
                    "last_monotonic_s: attempt={}, segment={}".format(
                        attempt_id,
                        segment_id,
                    )
                )
            intervals.append(
                _SegmentInterval(
                    attempt_id=attempt_id,
                    track_segment_id=segment_id,
                    first_monotonic_s=first,
                    last_monotonic_s=last,
                )
            )
    intervals.sort(
        key=lambda value: (
            value.first_monotonic_s,
            value.last_monotonic_s,
            value.attempt_id,
            (
                -1
                if value.track_segment_id is None
                else value.track_segment_id
            ),
        )
    )
    merged: List[_SegmentInterval] = []
    for interval in intervals:
        if (
            not merged
            or interval.first_monotonic_s
            > merged[-1].last_monotonic_s
        ):
            merged.append(interval)
            continue
        previous = merged[-1]
        if previous.attempt_id != interval.attempt_id:
            raise ValueError(
                "track segment intervals overlap across attempts: "
                "{} and {}".format(
                    previous.attempt_id,
                    interval.attempt_id,
                )
            )
        merged[-1] = _SegmentInterval(
            attempt_id=previous.attempt_id,
            track_segment_id=(
                previous.track_segment_id
                if previous.track_segment_id
                == interval.track_segment_id
                else None
            ),
            first_monotonic_s=previous.first_monotonic_s,
            last_monotonic_s=max(
                previous.last_monotonic_s,
                interval.last_monotonic_s,
            ),
        )
    return tuple(merged)


def _interval_for_time(
    intervals: Sequence[_SegmentInterval],
    interval_starts: Sequence[float],
    monotonic_s: float,
) -> Optional[_SegmentInterval]:
    index = bisect.bisect_right(interval_starts, monotonic_s) - 1
    if index < 0:
        return None
    interval = intervals[index]
    if monotonic_s <= interval.last_monotonic_s:
        return interval
    return None


def _path_is_within(path: Path, parent: Path) -> bool:
    try:
        path.relative_to(parent)
    except ValueError:
        return False
    return True


def _absolute_normalized_path(path: Path) -> Path:
    return Path(os.path.abspath(str(Path(path))))


def _reject_existing_symlink_components(path: Path) -> None:
    absolute = _absolute_normalized_path(path)
    current = Path(absolute.anchor)
    for component in absolute.parts[1:]:
        current = current / component
        try:
            component_stat = os.lstat(str(current))
        except FileNotFoundError:
            break
        if stat.S_ISLNK(component_stat.st_mode):
            raise ValueError(
                "output path contains a symlink component: {}".format(
                    current
                )
            )


def _prepare_output_path(
    session_dir: Path,
    output_dir: Path,
    *,
    overwrite: bool,
) -> Tuple[Path, Path]:
    output = _absolute_normalized_path(output_dir)
    _reject_existing_symlink_components(output)
    resolved_session = session_dir.resolve(strict=True)
    resolved_output = output.resolve(strict=False)
    if (
        resolved_output == resolved_session
        or _path_is_within(resolved_session, resolved_output)
        or _path_is_within(resolved_output, resolved_session)
    ):
        raise ValueError(
            "output directory must not overlap the source session"
        )
    parent = output.parent
    parent.mkdir(parents=True, exist_ok=True)
    _reject_existing_symlink_components(output)
    resolved_output = output.resolve(strict=False)
    if (
        resolved_output == resolved_session
        or _path_is_within(resolved_session, resolved_output)
        or _path_is_within(resolved_output, resolved_session)
    ):
        raise ValueError(
            "output directory must not overlap the source session"
        )
    output = resolved_output
    parent = output.parent
    if os.path.lexists(str(output)):
        output_stat = os.stat(str(output), follow_symlinks=False)
        if not stat.S_ISDIR(output_stat.st_mode):
            raise ValueError(
                "existing output path must be a directory"
            )
        if not overwrite:
            raise FileExistsError(
                "output directory already exists: {}".format(output)
            )
    return output, parent


def _write_csv_rows(
    path: Path,
    fieldnames: Sequence[str],
    rows: Iterable[Mapping[str, Any]],
) -> int:
    count = 0
    with path.open(
        "w",
        encoding="utf-8",
        newline="",
    ) as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=fieldnames,
            extrasaction="raise",
        )
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
            count += 1
        stream.flush()
        os.fsync(stream.fileno())
    return count


def _validate_csv_source_row(
    row: Mapping[Optional[str], Any],
    *,
    fieldnames: Sequence[str],
    line_number: int,
    label: str,
) -> None:
    if None in row:
        raise ValueError(
            "{} row {} has extra columns".format(label, line_number)
        )
    missing = [
        field
        for field in fieldnames
        if row.get(field) is None
    ]
    if missing:
        raise ValueError(
            "{} row {} is missing columns: {!r}".format(
                label,
                line_number,
                missing,
            )
        )


def _write_raw_lcm_csv(
    source_path: Path,
    output_path: Path,
    intervals: Sequence[_SegmentInterval],
) -> int:
    interval_starts = tuple(
        interval.first_monotonic_s for interval in intervals
    )
    count = 0
    previous_input_seq: Optional[int] = None
    with source_path.open(
        "r",
        encoding="utf-8",
        newline="",
    ) as source, output_path.open(
        "w",
        encoding="utf-8",
        newline="",
    ) as target:
        reader = csv.DictReader(source)
        if reader.fieldnames != list(_SOURCE_RAW_FIELDS):
            raise ValueError(
                "ball_samples.csv header mismatch: expected={!r}, "
                "actual={!r}".format(
                    list(_SOURCE_RAW_FIELDS),
                    reader.fieldnames,
                )
            )
        if len(set(reader.fieldnames)) != len(reader.fieldnames):
            raise ValueError("ball_samples.csv has duplicate header fields")
        writer = csv.DictWriter(
            target,
            fieldnames=_RAW_LCM_FIELDS,
            extrasaction="raise",
        )
        writer.writeheader()
        for line_number, row in enumerate(reader, start=2):
            _validate_csv_source_row(
                row,
                fieldnames=_SOURCE_RAW_FIELDS,
                line_number=line_number,
                label="ball_samples.csv",
            )
            input_seq_text = row["input_seq"]
            if (
                not input_seq_text
                or any(
                    character < "0" or character > "9"
                    for character in input_seq_text
                )
            ):
                raise ValueError(
                    "ball_samples.csv input_seq must be a non-negative "
                    "integer at line {}".format(line_number)
                )
            input_seq = int(input_seq_text)
            if (
                previous_input_seq is not None
                and input_seq != previous_input_seq + 1
            ):
                raise ValueError(
                    "ball_samples.csv input_seq must increment by one at "
                    "line {}: previous={}, actual={}".format(
                        line_number,
                        previous_input_seq,
                        input_seq,
                    )
                )
            previous_input_seq = input_seq
            monotonic_s = _require_finite_float(
                row["received_monotonic_s"],
                field=(
                    "ball_samples.csv received_monotonic_s "
                    "at line {}".format(line_number)
                ),
            )
            interval = _interval_for_time(
                intervals,
                interval_starts,
                monotonic_s,
            )
            if interval is None:
                continue
            output_row: Dict[str, Any] = {
                "schema_version": EXPORT_SCHEMA_VERSION,
                "interval_attempt_id": interval.attempt_id,
                "interval_track_segment_id": (
                    interval.track_segment_id
                ),
            }
            output_row.update(row)
            for vector_field in (
                "position_w",
                "quaternion_xyzw",
            ):
                try:
                    vector_value = json.loads(row[vector_field])
                except json.JSONDecodeError as exc:
                    raise ValueError(
                        "ball_samples.csv {} is invalid JSON at line "
                        "{}".format(vector_field, line_number)
                    ) from exc
                if not isinstance(vector_value, list):
                    raise ValueError(
                        "ball_samples.csv {} must be a JSON list at line "
                        "{}".format(vector_field, line_number)
                    )
                output_row[vector_field] = _compact_json(vector_value)
            writer.writerow(output_row)
            count += 1
        target.flush()
        os.fsync(target.fileno())
    return count


def _normalize_task_observation(
    pre_clip: Any,
    post_clip: Any,
    clip_count: Any,
    *,
    label: str,
) -> Optional[_TaskObservation]:
    present = (
        pre_clip is not None,
        post_clip is not None,
        clip_count is not None,
    )
    if not any(present):
        return None
    if not all(present):
        raise ValueError(
            "{} task observation fields must be all null or all "
            "present".format(label)
        )
    if not isinstance(pre_clip, list) or not isinstance(post_clip, list):
        raise ValueError(
            "{} task observation pre/post vectors must both be "
            "lists".format(label)
        )
    if (
        len(pre_clip) != TASK_OBSERVATION_DIMENSIONS
        or len(post_clip) != TASK_OBSERVATION_DIMENSIONS
    ):
        raise ValueError(
            "{} task observation vectors must have {} "
            "dimensions".format(label, TASK_OBSERVATION_DIMENSIONS)
        )
    return _TaskObservation(
        pre_clip=tuple(
            _require_finite_float(
                value,
                field="{} task observation pre_clip".format(label),
            )
            for value in pre_clip
        ),
        post_clip=tuple(
            _require_finite_float(
                value,
                field="{} task observation post_clip".format(label),
            )
            for value in post_clip
        ),
        clip_count=_require_int(
            clip_count,
            field="{} task observation clip_count".format(label),
            minimum=0,
        ),
    )


def _policy_tick_csv_row(
    event: Mapping[str, Any],
    interval: _SegmentInterval,
    tick_index: int,
) -> Mapping[str, Any]:
    payload = event["payload"]
    payload_fields = frozenset(payload)
    if payload_fields != _REQUIRED_LIFECYCLE_PAYLOAD_FIELDS:
        raise ValueError(
            "lifecycle_tick payload fields mismatch: "
            "missing={!r}, unexpected={!r}".format(
                sorted(
                    _REQUIRED_LIFECYCLE_PAYLOAD_FIELDS
                    - payload_fields
                ),
                sorted(
                    payload_fields
                    - _REQUIRED_LIFECYCLE_PAYLOAD_FIELDS
                ),
            )
        )
    return {
        "schema_version": event["schema_version"],
        "event_id": event["event_id"],
        "attempt_id": interval.attempt_id,
        "tick_index": tick_index,
        "monotonic_s": event["monotonic_s"],
        "wall_time_us": event["wall_time_us"],
        "scope": event["scope"],
        "lifecycle_now_s": payload["lifecycle_now_s"],
        "obs_now_s": payload["obs_now_s"],
        "phase": payload["phase"],
        "decision": payload["decision"],
        "active_key": _csv_value(payload["active_key"]),
        "cached_key": _csv_value(payload["cached_key"]),
        "command_result": _csv_value(payload["command_result"]),
        "task_pre_clip": _csv_value(payload["task_pre_clip"]),
        "task_post_clip": _csv_value(payload["task_post_clip"]),
        "clip_count": payload["clip_count"],
        "task_pass": payload["task_pass"],
        "errors": _csv_value(payload["errors"]),
        "recovery_duration_s": payload["recovery_duration_s"],
    }


def _validate_events_and_write_policy_ticks(
    source_path: Path,
    output_path: Path,
    *,
    intervals: Sequence[_SegmentInterval],
    expected_watermark_event_id: int,
) -> Tuple[int, _EventLogValidation]:
    interval_starts = tuple(
        interval.first_monotonic_s for interval in intervals
    )
    selected_attempt_ids = frozenset(
        interval.attempt_id for interval in intervals
    )
    expected_event_id = 1
    event_count = 0
    recorder_event_gap_count = 0
    diagnostic_event_drop_count = 0
    policy_count = 0
    tick_index_by_attempt: Dict[int, int] = {}
    with source_path.open(
        "r",
        encoding="utf-8",
    ) as source, output_path.open(
        "w",
        encoding="utf-8",
        newline="",
    ) as target:
        writer = csv.DictWriter(
            target,
            fieldnames=_POLICY_TICK_FIELDS,
            extrasaction="raise",
        )
        writer.writeheader()
        for line_number, line in enumerate(source, start=1):
            if not line.strip():
                raise ValueError(
                    "events.jsonl has a blank line at {}".format(
                        line_number
                    )
                )
            try:
                event = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(
                    "events.jsonl has invalid JSON at line {}".format(
                        line_number
                    )
                ) from exc
            if not isinstance(event, Mapping):
                raise ValueError(
                    "events.jsonl event must be a mapping at line {}".format(
                        line_number
                    )
                )
            if event.get("kind") == "RECORDER_EVENT_GAP":
                recorder_event_gap_count += 1
                raise ValueError(
                    "events.jsonl contains RECORDER_EVENT_GAP"
                )
            if event.get("kind") == "DIAGNOSTIC_EVENT_DROPPED":
                diagnostic_event_drop_count += 1
                raise ValueError(
                    "events.jsonl contains DIAGNOSTIC_EVENT_DROPPED"
                )
            if event.get("schema_version") != EXPORT_SCHEMA_VERSION:
                raise ValueError(
                    "unsupported event schema at line {}".format(
                        line_number
                    )
                )
            event_id = _require_int(
                event.get("event_id"),
                field="event_id at line {}".format(line_number),
                minimum=1,
            )
            if event_id != expected_event_id:
                raise ValueError(
                    "events.jsonl event_id gap or reorder at line {}: "
                    "expected={}, actual={}".format(
                        line_number,
                        expected_event_id,
                        event_id,
                    )
                )
            expected_event_id += 1
            event_count += 1
            if event.get("kind") != "lifecycle_tick":
                continue
            attempt_value = event.get("attempt_id")
            event_attempt_id = None
            if attempt_value is not None:
                event_attempt_id = _require_int(
                    attempt_value,
                    field="lifecycle_tick attempt_id",
                    minimum=1,
                )
            payload = event.get("payload")
            if (
                event_attempt_id is not None
                and event_attempt_id in selected_attempt_ids
            ):
                if not isinstance(payload, Mapping):
                    raise ValueError(
                        "lifecycle_tick payload must be a mapping"
                    )
                payload_fields = frozenset(payload)
                if payload_fields != _REQUIRED_LIFECYCLE_PAYLOAD_FIELDS:
                    raise ValueError(
                        "lifecycle_tick payload fields mismatch: "
                        "missing={!r}, unexpected={!r}".format(
                            sorted(
                                _REQUIRED_LIFECYCLE_PAYLOAD_FIELDS
                                - payload_fields
                            ),
                            sorted(
                                payload_fields
                                - _REQUIRED_LIFECYCLE_PAYLOAD_FIELDS
                            ),
                        )
                    )
                _normalize_task_observation(
                    payload.get("task_pre_clip"),
                    payload.get("task_post_clip"),
                    payload.get("clip_count"),
                    label=(
                        "lifecycle_tick event_id={}".format(event_id)
                    ),
                )
            monotonic_s = _require_finite_float(
                event.get("monotonic_s"),
                field=(
                    "lifecycle_tick monotonic_s at line {}".format(
                        line_number
                    )
                ),
            )
            interval = _interval_for_time(
                intervals,
                interval_starts,
                monotonic_s,
            )
            if interval is None:
                continue
            if event_attempt_id is None:
                continue
            if event_attempt_id != interval.attempt_id:
                if event_attempt_id in selected_attempt_ids:
                    raise ValueError(
                        "lifecycle_tick attempt_id {} conflicts with "
                        "interval owner {}".format(
                            event_attempt_id,
                            interval.attempt_id,
                        )
                    )
                continue
            if not isinstance(payload, Mapping):
                raise ValueError(
                    "lifecycle_tick payload must be a mapping"
                )
            scope = event.get("scope")
            if scope != "attempt":
                raise ValueError(
                    "selected lifecycle_tick scope must be attempt"
                )
            _require_int(
                event.get("wall_time_us"),
                field="lifecycle_tick wall_time_us",
                minimum=0,
            )
            tick_index = tick_index_by_attempt.get(
                interval.attempt_id,
                0,
            )
            writer.writerow(
                _policy_tick_csv_row(
                    event,
                    interval,
                    tick_index,
                )
            )
            tick_index_by_attempt[interval.attempt_id] = tick_index + 1
            policy_count += 1
        target.flush()
        os.fsync(target.fileno())
    final_event_id = expected_event_id - 1
    if final_event_id != expected_watermark_event_id:
        raise ValueError(
            "events.jsonl final event_id does not match session "
            "watermark: final={}, watermark={}".format(
                final_event_id,
                expected_watermark_event_id,
            )
        )
    return (
        policy_count,
        _EventLogValidation(
            event_count=event_count,
            final_event_id=final_event_id,
            event_ids_contiguous=True,
            recorder_event_gap_count=recorder_event_gap_count,
            diagnostic_event_drop_count=diagnostic_event_drop_count,
        ),
    )


def _write_summary_csv(
    output_path: Path,
    *,
    details: Mapping[int, Mapping[str, Any]],
    scan: PlannerEventScan,
) -> int:
    submitted_by_attempt: Dict[int, int] = {}
    pending_by_attempt: Dict[int, int] = {}
    completed_by_attempt: Dict[int, int] = {}
    for key in scan.submitted_keys:
        submitted_by_attempt[key.attempt_id] = (
            submitted_by_attempt.get(key.attempt_id, 0) + 1
        )
    for key in scan.pending_replaced_keys:
        pending_by_attempt[key.attempt_id] = (
            pending_by_attempt.get(key.attempt_id, 0) + 1
        )
    for call in scan.completed_calls:
        completed_by_attempt[call.key.attempt_id] = (
            completed_by_attempt.get(call.key.attempt_id, 0) + 1
        )
    rows = []
    expected_summary_fields = set(_SUMMARY_FIELDS) - {
        "completed_call_export_complete",
        "completed_call_count",
        "pending_replaced_count",
    }
    for attempt_id in sorted(details):
        summary = details[attempt_id]["summary"]
        if set(summary) != expected_summary_fields:
            raise ValueError(
                "attempt summary fields mismatch for {}: "
                "missing={!r}, unexpected={!r}".format(
                    attempt_id,
                    sorted(expected_summary_fields - set(summary)),
                    sorted(set(summary) - expected_summary_fields),
                )
            )
        recording_complete = summary.get("recording_complete")
        if (
            recording_complete is not True
            and recording_complete is not False
        ):
            raise ValueError(
                "attempt summary recording_complete must be boolean"
            )
        completed_count = completed_by_attempt.get(attempt_id, 0)
        pending_count = pending_by_attempt.get(attempt_id, 0)
        submitted_count = submitted_by_attempt.get(attempt_id, 0)
        if completed_count + pending_count != submitted_count:
            raise ValueError(
                "attempt planner counts do not partition submissions: "
                "attempt={}".format(attempt_id)
            )
        row = dict(summary)
        row.update({
            "completed_call_export_complete": True,
            "completed_call_count": completed_count,
            "pending_replaced_count": pending_count,
        })
        rows.append(row)
    return _write_csv_rows(output_path, _SUMMARY_FIELDS, rows)


def _validate_aligned_rows(
    aligned: Sequence[AlignedPlannerRow],
) -> None:
    expected_input_fields = {
        "attempt_id",
        "snapshot_key",
    } | set(_PLANNER_INPUT_FIELDS[len(_KEY_FIELDS):])
    expected_result_fields = {
        "snapshot_key",
    } | set(_PLANNER_RESULT_FIELDS[len(_KEY_FIELDS):])
    for row in aligned:
        input_fields = set(row.input_row)
        if input_fields != expected_input_fields:
            raise ValueError(
                "planner input fields mismatch for {!r}: "
                "missing={!r}, unexpected={!r}".format(
                    row.call.key,
                    sorted(expected_input_fields - input_fields),
                    sorted(input_fields - expected_input_fields),
                )
            )
        result_fields = set(row.call.result)
        if result_fields != expected_result_fields:
            raise ValueError(
                "planner result fields mismatch for {!r}: "
                "missing={!r}, unexpected={!r}".format(
                    row.call.key,
                    sorted(expected_result_fields - result_fields),
                    sorted(result_fields - expected_result_fields),
                )
            )
        _require_finite_float(
            row.input_row.get("source_time_s"),
            field="planner input source_time_s",
        )
        _require_finite_float(
            row.input_row.get("received_monotonic_s"),
            field="planner input received_monotonic_s",
        )


def _write_core_csvs(
    output_dir: Path,
    aligned: Sequence[AlignedPlannerRow],
) -> Mapping[str, int]:
    _validate_aligned_rows(aligned)
    rows_by_file = {
        "planner_inputs.csv": planner_input_csv_rows(aligned),
        "planner_calls.csv": planner_call_csv_rows(aligned),
        "planner_results.csv": planner_result_csv_rows(aligned),
    }
    counts = {}
    for filename, rows in rows_by_file.items():
        counts[filename] = _write_csv_rows(
            output_dir / filename,
            _CSV_HEADERS[filename],
            rows,
        )
    return counts


def _write_stage_timeline_csv(
    output_path: Path,
    timelines: Mapping[
        int,
        Sequence[Mapping[str, Any]],
    ],
) -> int:
    rows = []
    for attempt_id in sorted(timelines):
        for stage_index, raw_value in enumerate(
            timelines[attempt_id]
        ):
            value = _normalize_stage_timeline_row(
                raw_value,
                expected_attempt_id=attempt_id,
                label="stage timeline row attempt={} index={}".format(
                    attempt_id,
                    stage_index,
                ),
            )
            segment_value = value.get("track_segment_id")
            snapshot = value.get("snapshot_key")
            snapshot_key = None
            if snapshot is not None:
                snapshot_key = _planner_key(
                    attempt_id,
                    snapshot,
                    field="stage timeline snapshot_key",
                )
            values = value.get("values")
            if not isinstance(values, Mapping):
                raise ValueError(
                    "stage timeline values must be a mapping"
                )
            row: Dict[str, Any] = {
                "attempt_id": attempt_id,
                "stage_index": stage_index,
                "schema_version": value["schema_version"],
                "track_segment_id": segment_value,
                "stage": value["stage"],
                "monotonic_s": value["monotonic_s"],
                "reason_code": value["reason_code"],
                "snapshot_key.schema_version": (
                    None
                    if snapshot_key is None
                    else snapshot_key.schema_version
                ),
                "snapshot_key.track_epoch": (
                    None
                    if snapshot_key is None
                    else snapshot_key.track_epoch
                ),
                "snapshot_key.generation": (
                    None
                    if snapshot_key is None
                    else snapshot_key.generation
                ),
                "values_json": _compact_json(values),
            }
            for field in _STAGE_VALUE_FIELDS:
                row["values.{}".format(field)] = _csv_value(
                    values.get(field)
                )
            rows.append(row)
    return _write_csv_rows(output_path, _STAGE_FIELDS, rows)


def _detail_task_observation(
    detail: Mapping[str, Any],
    *,
    attempt_id: int,
) -> Optional[_TaskObservation]:
    return _normalize_task_observation(
        detail["task_observation_pre_clip"],
        detail["task_observation_post_clip"],
        detail["task_observation_clip_count"],
        label="attempt {} detail".format(attempt_id),
    )


def _write_task_observations_csv(
    output_path: Path,
    attempt_ids: Sequence[int],
    task_observations: Mapping[int, _TaskObservation],
) -> int:
    rows = []
    for attempt_id in sorted(attempt_ids):
        observation = task_observations.get(attempt_id)
        if observation is None:
            pre_values = (None,) * TASK_OBSERVATION_DIMENSIONS
            post_values = (None,) * TASK_OBSERVATION_DIMENSIONS
            clip_count = None
        else:
            pre_values = observation.pre_clip
            post_values = observation.post_clip
            clip_count = observation.clip_count
        row: Dict[str, Any] = {
            "attempt_id": attempt_id,
            "clip_count": clip_count,
        }
        for index, value in enumerate(pre_values):
            row["pre_{}".format(index)] = value
        for index, value in enumerate(post_values):
            row["post_{}".format(index)] = value
        rows.append(row)
    return _write_csv_rows(
        output_path,
        _TASK_OBSERVATION_FIELDS,
        rows,
    )


def _read_and_validate_csv(
    path: Path,
    expected_header: Sequence[str],
    *,
    read_keys: bool,
) -> Tuple[int, Tuple[PlannerExportKey, ...]]:
    count = 0
    keys = []
    seen_keys = set()
    with path.open(
        "r",
        encoding="utf-8",
        newline="",
    ) as stream:
        reader = csv.reader(stream)
        try:
            header = next(reader)
        except StopIteration as exc:
            raise ValueError(
                "CSV is empty: {}".format(path.name)
            ) from exc
        if header != list(expected_header):
            raise ValueError(
                "{} header mismatch: expected={!r}, actual={!r}".format(
                    path.name,
                    list(expected_header),
                    header,
                )
            )
        if len(set(header)) != len(header):
            raise ValueError(
                "{} has duplicate header fields".format(path.name)
            )
        key_indexes = (
            tuple(header.index(field) for field in _KEY_FIELDS)
            if read_keys
            else ()
        )
        for record_number, values in enumerate(reader, start=2):
            if values == header:
                raise ValueError(
                    "{} contains a repeated header at record {}".format(
                        path.name,
                        record_number,
                    )
                )
            if len(values) != len(header):
                raise ValueError(
                    "{} has a ragged row at record {}".format(
                        path.name,
                        record_number,
                    )
                )
            count += 1
            if not read_keys:
                continue
            try:
                key = PlannerExportKey(
                    attempt_id=int(values[key_indexes[0]]),
                    schema_version=int(values[key_indexes[1]]),
                    track_epoch=int(values[key_indexes[2]]),
                    generation=int(values[key_indexes[3]]),
                )
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    "{} has an invalid planner key at record {}".format(
                        path.name,
                        record_number,
                    )
                ) from exc
            if key in seen_keys:
                raise ValueError(
                    "{} has a duplicate planner key: {!r}".format(
                        path.name,
                        key,
                    )
                )
            seen_keys.add(key)
            keys.append(key)
    return count, tuple(keys)


def _core_key_sequence_sha256(
    keys: Sequence[PlannerExportKey],
) -> str:
    serialized = _compact_json([
        [
            key.attempt_id,
            key.schema_version,
            key.track_epoch,
            key.generation,
        ]
        for key in keys
    ])
    return hashlib.sha256(serialized.encode("utf-8")).hexdigest()


def validate_written_outputs(
    output_dir: Path,
    expected_row_counts: Mapping[str, int],
) -> Tuple[Mapping[str, int], str]:
    row_counts = {}
    core_keys = {}
    core_names = {
        "planner_inputs.csv",
        "planner_calls.csv",
        "planner_results.csv",
    }
    for filename, header in _CSV_HEADERS.items():
        count, keys = _read_and_validate_csv(
            Path(output_dir) / filename,
            header,
            read_keys=filename in core_names,
        )
        expected_count = expected_row_counts.get(filename)
        if expected_count is None:
            raise ValueError(
                "missing expected row count for {}".format(filename)
            )
        if count != expected_count:
            raise ValueError(
                "{} row count mismatch: expected={}, actual={}".format(
                    filename,
                    expected_count,
                    count,
                )
            )
        row_counts[filename] = count
        if filename in core_names:
            core_keys[filename] = keys
    input_keys = core_keys["planner_inputs.csv"]
    call_keys = core_keys["planner_calls.csv"]
    result_keys = core_keys["planner_results.csv"]
    if not (input_keys == call_keys == result_keys):
        raise ValueError(
            "core planner CSV key sequences do not match"
        )
    if not (
        row_counts["planner_inputs.csv"]
        == row_counts["planner_calls.csv"]
        == row_counts["planner_results.csv"]
    ):
        raise ValueError("core planner CSV row counts do not match")
    return row_counts, _core_key_sequence_sha256(input_keys)


def _write_manifest(path: Path, manifest: ExportManifest) -> None:
    with path.open("w", encoding="utf-8") as stream:
        json.dump(
            manifest.to_json_dict(),
            stream,
            allow_nan=False,
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        )
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    loaded = _load_json_mapping(path, label="export manifest")
    if loaded != manifest.to_json_dict():
        raise ValueError("export manifest round-trip validation failed")


def _fsync_directory(path: Path) -> None:
    flags = os.O_RDONLY
    flags |= getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_DIRECTORY", 0)
    descriptor = os.open(str(path), flags)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _unique_sibling_path(output_dir: Path, label: str) -> Path:
    for _attempt in range(100):
        candidate = output_dir.parent / (
            ".{}-{}-{}".format(
                output_dir.name,
                label,
                uuid.uuid4().hex,
            )
        )
        if not os.path.lexists(str(candidate)):
            return candidate
    raise RuntimeError(
        "could not allocate a unique {} path".format(label)
    )


def _unique_backup_path(output_dir: Path) -> Path:
    return _unique_sibling_path(output_dir, "backup")


def _warn_after_commit(message: str) -> None:
    try:
        warnings.warn(
            message,
            RuntimeWarning,
        )
    except BaseException:
        # Once the replacement directory is durably installed, warning
        # filters or custom warning hooks must not turn cleanup diagnostics
        # into an apparent export failure.
        try:
            sys.stderr.write("RuntimeWarning: {}\n".format(message))
            sys.stderr.flush()
        except BaseException:
            pass


def replace_output_directory(
    temporary_dir: Path,
    output_dir: Path,
    *,
    overwrite: bool,
) -> None:
    temporary = _absolute_normalized_path(temporary_dir)
    output = _absolute_normalized_path(output_dir)
    _reject_existing_symlink_components(temporary)
    _reject_existing_symlink_components(output)
    if temporary.parent != output.parent:
        raise ValueError(
            "temporary and output directories must share a parent"
        )
    if not temporary.is_dir():
        raise ValueError(
            "temporary output must be a real directory"
        )
    if not os.path.lexists(str(output)):
        os.replace(str(temporary), str(output))
        try:
            _fsync_directory(output.parent)
        except BaseException as install_sync_error:
            try:
                os.replace(str(output), str(temporary))
            except BaseException as withdraw_error:
                raise RuntimeError(
                    "new output was installed at {} but parent fsync "
                    "failed ({!r}); withdrawing it also failed ({!r}), "
                    "so output may remain visible".format(
                        output,
                        install_sync_error,
                        withdraw_error,
                    )
                ) from withdraw_error
            try:
                _fsync_directory(output.parent)
            except BaseException as withdraw_sync_error:
                raise RuntimeError(
                    "new output parent fsync failed ({!r}); output was "
                    "moved back to temporary path {}, but rollback fsync "
                    "also failed ({!r})".format(
                        install_sync_error,
                        temporary,
                        withdraw_sync_error,
                    )
                ) from withdraw_sync_error
            raise RuntimeError(
                "new output parent fsync failed ({!r}); uncommitted "
                "output was moved back to temporary path {}".format(
                    install_sync_error,
                    temporary,
                )
            ) from install_sync_error
        return
    output_stat = os.stat(str(output), follow_symlinks=False)
    if not stat.S_ISDIR(output_stat.st_mode):
        raise ValueError("existing output path must be a directory")
    if not overwrite:
        raise FileExistsError(
            "output directory already exists: {}".format(output)
        )
    backup = _unique_backup_path(output)
    os.replace(str(output), str(backup))
    try:
        os.replace(str(temporary), str(output))
    except BaseException as install_error:
        try:
            os.replace(str(backup), str(output))
        except BaseException as restore_rename_error:
            raise RuntimeError(
                "failed to install new output ({!r}) and failed to "
                "restore old output ({!r}); backup preserved at {}".format(
                    install_error,
                    restore_rename_error,
                    backup,
                )
            ) from restore_rename_error
        try:
            _fsync_directory(output.parent)
        except BaseException as restore_sync_error:
            raise RuntimeError(
                "failed to install new output ({!r}); old output "
                "restored at {}, but restore fsync failed ({!r}); "
                "backup was consumed by the restore".format(
                    install_error,
                    output,
                    restore_sync_error,
                )
            ) from restore_sync_error
        raise
    try:
        _fsync_directory(output.parent)
    except BaseException as install_sync_error:
        failed_new = _unique_sibling_path(output, "failed-new")
        try:
            os.replace(str(output), str(failed_new))
        except BaseException as quarantine_error:
            raise RuntimeError(
                "new output rename succeeded but parent fsync failed "
                "({!r}); moving the uncommitted output aside also "
                "failed ({!r}); new output may remain at {} and old "
                "output remains at {}".format(
                    install_sync_error,
                    quarantine_error,
                    output,
                    backup,
                )
            ) from quarantine_error
        try:
            os.replace(str(backup), str(output))
        except BaseException as restore_error:
            raise RuntimeError(
                "new output parent fsync failed ({!r}); uncommitted new "
                "output was preserved at {}, but restoring old output "
                "failed ({!r}); old output remains at {}".format(
                    install_sync_error,
                    failed_new,
                    restore_error,
                    backup,
                )
            ) from restore_error
        try:
            _fsync_directory(output.parent)
        except BaseException as restore_sync_error:
            raise RuntimeError(
                "new output parent fsync failed ({!r}); old output "
                "restored at {} and uncommitted new output preserved at "
                "{}, but restore fsync failed ({!r}); backup was "
                "consumed by the restore".format(
                    install_sync_error,
                    output,
                    failed_new,
                    restore_sync_error,
                )
            ) from restore_sync_error
        raise RuntimeError(
            "new output parent fsync failed ({!r}); old output restored "
            "at {} and uncommitted new output preserved at {}".format(
                install_sync_error,
                output,
                failed_new,
            )
        ) from install_sync_error
    try:
        shutil.rmtree(str(backup))
    except OSError as cleanup_error:
        _warn_after_commit(
            "output committed at {}; backup cleanup failed and residual "
            "backup may remain at {}: {}".format(
                output,
                backup,
                cleanup_error,
            )
        )
        return
    try:
        _fsync_directory(output.parent)
    except OSError as cleanup_sync_error:
        _warn_after_commit(
            "output committed at {}; backup was removed but cleanup "
            "fsync failed: {}".format(output, cleanup_sync_error)
        )


def _source_files(
    session_dir: Path,
    attempt_ids: Sequence[int],
) -> Mapping[str, Path]:
    paths: Dict[str, Path] = {
        "session.json": session_dir / "session.json",
        "events.jsonl": session_dir / "events.jsonl",
        "ball_samples.csv": session_dir / "ball_samples.csv",
    }
    for attempt_id in attempt_ids:
        name = "attempt_details/{}.json".format(attempt_id)
        paths[name] = session_dir / name
    return paths


def _assert_source_files_unchanged(
    paths: Mapping[str, Path],
    initial: Mapping[str, _SourceFingerprint],
) -> None:
    for name, path in paths.items():
        final = _fingerprint_source_file(path)
        if final != initial[name]:
            raise ValueError(
                "source file changed during export: {}".format(name)
            )


def export_session_csv(
    session_dir: Path,
    attempt_ids: Sequence[int],
    output_dir: Path,
    overwrite: bool,
) -> ExportManifest:
    attempts = _normalize_attempt_ids(attempt_ids)
    session = Path(session_dir).resolve(strict=True)
    if not session.is_dir():
        raise ValueError("source session must be a directory")
    output, output_parent = _prepare_output_path(
        session,
        Path(output_dir),
        overwrite=bool(overwrite),
    )
    source_paths = _source_files(session, attempts)
    initial_fingerprints = {
        name: _fingerprint_source_file(path)
        for name, path in source_paths.items()
    }
    session_metadata = _load_json_mapping(
        source_paths["session.json"],
        label="session metadata",
    )
    watermark_event_id = _validate_session_metadata(session_metadata)
    details = _load_attempt_details(session, attempts)
    transition_scan = _scan_attempt_transitions(
        source_paths["events.jsonl"],
        attempts,
        expected_watermark_event_id=watermark_event_id,
    )
    snapshot_validation = _validate_attempt_detail_transition_windows(
        details,
        transition_scan,
    )
    intervals = _merged_segment_intervals(
        snapshot_validation.segments
    )
    scan = scan_planner_events(
        source_paths["events.jsonl"],
        attempts,
    )
    inputs = load_attempt_inputs(session, attempts)
    aligned = align_completed_calls(scan, inputs)
    _validate_aligned_rows(aligned)

    temporary = Path(
        tempfile.mkdtemp(
            prefix=".hitter-csv-",
            dir=str(output_parent),
        )
    )
    try:
        expected_counts: Dict[str, int] = {}
        expected_counts["summary.csv"] = _write_summary_csv(
            temporary / "summary.csv",
            details=details,
            scan=scan,
        )
        expected_counts["raw_lcm.csv"] = _write_raw_lcm_csv(
            source_paths["ball_samples.csv"],
            temporary / "raw_lcm.csv",
            intervals,
        )
        expected_counts.update(_write_core_csvs(temporary, aligned))
        expected_counts["stage_timeline.csv"] = (
            _write_stage_timeline_csv(
                temporary / "stage_timeline.csv",
                transition_scan.timelines,
            )
        )
        (
            expected_counts["policy_ticks.csv"],
            event_validation,
        ) = _validate_events_and_write_policy_ticks(
            source_paths["events.jsonl"],
            temporary / "policy_ticks.csv",
            intervals=intervals,
            expected_watermark_event_id=watermark_event_id,
        )
        if (
            event_validation.event_count
            != transition_scan.event_count
            or event_validation.final_event_id
            != transition_scan.final_event_id
        ):
            raise ValueError(
                "events.jsonl changed between validation passes"
            )
        expected_counts["task_observations.csv"] = (
            _write_task_observations_csv(
                temporary / "task_observations.csv",
                attempts,
                snapshot_validation.task_observations,
            )
        )
        row_counts, core_key_hash = validate_written_outputs(
            temporary,
            expected_counts,
        )
        completed_count = len(scan.completed_calls)
        pending_count = len(scan.pending_replaced_keys)
        submitted_count = len(scan.submitted_keys)
        if completed_count + pending_count != submitted_count:
            raise ValueError(
                "completed and pending-replaced counts do not partition "
                "submitted calls"
            )
        manifest = ExportManifest(
            schema_version=EXPORT_SCHEMA_VERSION,
            source_session_dir=str(session),
            attempt_ids=attempts,
            source_sha256={
                name: fingerprint.sha256
                for name, fingerprint in initial_fingerprints.items()
            },
            source_file_stats={
                name: {
                    "device": fingerprint.device,
                    "inode": fingerprint.inode,
                    "size": fingerprint.size,
                    "mtime_ns": fingerprint.mtime_ns,
                }
                for name, fingerprint in initial_fingerprints.items()
            },
            row_counts=row_counts,
            core_key_sequence_sha256=core_key_hash,
            submitted_count=submitted_count,
            pending_replaced_count=pending_count,
            completed_call_count=completed_count,
            completeness_checks={
                "disk_session_sources_only": True,
                "session_status_complete": True,
                "session_recording_complete": True,
                "recorder_healthy": True,
                "session_last_error_absent": True,
                "session_failure_absent": True,
                "input_samples_not_dropped": True,
                "watermarks_match": True,
                "event_ids_contiguous": (
                    event_validation.event_ids_contiguous
                ),
                "final_event_id_matches_watermark": (
                    event_validation.final_event_id
                    == watermark_event_id
                ),
                "no_recorder_event_gap": (
                    event_validation.recorder_event_gap_count == 0
                ),
                "no_diagnostic_event_drop": (
                    event_validation.diagnostic_event_drop_count == 0
                ),
                "segments_valid_and_non_overlapping": True,
                "full_attempt_transitions_from_events": True,
                "attempt_detail_timeline_prefix_window_verified": True,
                "segments_rebuilt_from_full_transitions": True,
                "task_observations_from_full_events": True,
                "task_observation_detail_found_in_full_events": True,
                "null_task_observation_preclose_events_absent": True,
                "submitted_partition_complete": True,
                "completed_pending_disjoint": True,
                "completed_calls_have_submit_start_complete_input": True,
                "core_csv_headers_exact": True,
                "core_csv_key_sequences_equal": True,
                "core_csv_keys_unique": True,
                "core_csv_row_counts_equal": True,
                "source_files_unchanged_during_export": True,
            },
            exported_at_utc=datetime.now(timezone.utc).isoformat(),
        )
        _write_manifest(
            temporary / "export_manifest.json",
            manifest,
        )
        _fsync_directory(temporary)
        _assert_source_files_unchanged(
            source_paths,
            initial_fingerprints,
        )
        replace_output_directory(
            temporary,
            output,
            overwrite=bool(overwrite),
        )
        return manifest
    except BaseException:
        shutil.rmtree(str(temporary), ignore_errors=True)
        raise


def _argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Export complete HITTER planner diagnostics from on-disk "
            "session sources."
        )
    )
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument(
        "--attempts",
        type=int,
        nargs="+",
        required=True,
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--overwrite", action="store_true")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = _argument_parser()
    arguments = parser.parse_args(argv)
    try:
        manifest = export_session_csv(
            session_dir=arguments.session,
            attempt_ids=arguments.attempts,
            output_dir=arguments.output,
            overwrite=arguments.overwrite,
        )
    except Exception as exc:
        print(
            "error: {}: {}".format(type(exc).__name__, exc),
            file=sys.stderr,
        )
        return 1
    print(
        json.dumps(
            manifest.to_json_dict(),
            allow_nan=False,
            ensure_ascii=False,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
