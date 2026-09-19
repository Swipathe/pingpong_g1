from __future__ import annotations

from collections import OrderedDict, deque
import csv
from dataclasses import dataclass, replace
from datetime import datetime
import errno
import heapq
import json
import os
from pathlib import Path
import re
import stat
import threading
import time
from typing import (
    Any,
    Callable,
    Deque,
    Dict,
    Mapping,
    Optional,
    TextIO,
    Tuple,
    Union,
)
import uuid

from diagnostics.hitter_task_events import EventCursor, PublishedEvent
from diagnostics.hitter_task_models import (
    SCHEMA_VERSION,
    AttemptDetail,
    AttemptPage,
    AttemptSummary,
    EventDraft,
    NormalizedMocapSample,
)
from utils.hitter_serialization import JsonValue, freeze_json_value, to_builtin_json


@dataclass(frozen=True)
class SessionPaths:
    root: Path
    session_json: Path
    ball_samples_csv: Path
    events_jsonl: Path
    replay_jobs_jsonl: Path
    replay_analysis_jsonl: Path
    attempts_csv: Path
    attempt_details_dir: Path


@dataclass(frozen=True)
class RawRecordEnvelope:
    sample: NormalizedMocapSample
    attempt_id: Optional[int]
    track_segment_id: Optional[int]

    def __post_init__(self) -> None:
        if self.attempt_id is not None and self.attempt_id <= 0:
            raise ValueError("attempt_id must be positive")
        if self.track_segment_id is not None and self.track_segment_id <= 0:
            raise ValueError("track_segment_id must be positive")
        if self.track_segment_id is not None and self.attempt_id is None:
            raise ValueError("segment identity requires attempt identity")


@dataclass(frozen=True)
class RawSubmitResult:
    accepted: bool
    dropped_input_seq: Optional[Tuple[int, int]]
    attempt_id: Optional[int]
    reason: Optional[str]


@dataclass(frozen=True)
class RecorderStatus:
    healthy: bool
    recording_complete: bool
    bytes_written: int
    last_error: Optional[str]


RAW_DROP_METADATA_OVERFLOW = "METADATA_OVERFLOW"
RawDropRange = Tuple[int, int, Optional[int]]
RawDropNotice = Union[RawDropRange, str]


@dataclass
class _DrainSnapshot:
    raw_remaining: int
    event_watermark: int
    detail_remaining: int
    analysis_remaining: int
    job_remaining: int
    drop_notices: Tuple[RawDropNotice, ...]

    def exhausted(
        self,
        *,
        next_event_id: int,
        event_gap_seen: bool,
    ) -> bool:
        return bool(
            self.raw_remaining <= 0
            and (event_gap_seen or next_event_id > self.event_watermark)
            and self.detail_remaining <= 0
            and self.analysis_remaining <= 0
            and self.job_remaining <= 0
            and not self.drop_notices
        )


class _RecordingDeadlineExpired(RuntimeError):
    pass


def _deadline_expired(deadline_s: Optional[float]) -> bool:
    return deadline_s is not None and time.monotonic() >= deadline_s


def _check_deadline(deadline_s: Optional[float]) -> None:
    if _deadline_expired(deadline_s):
        raise _RecordingDeadlineExpired()


def _discard_buffered_text_and_close(stream: TextIO) -> None:
    raw = stream.buffer.raw
    raw.close()
    stream.close()


def create_session_paths(
    output_root: Path,
    *,
    now: Optional[datetime] = None,
    pid: Optional[int] = None,
    short_uuid: Optional[str] = None,
) -> SessionPaths:
    output_root = Path(output_root)
    output_root.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now() if now is None else now
    process_id = os.getpid() if pid is None else pid
    if isinstance(process_id, bool) or not isinstance(process_id, int) or process_id <= 0:
        raise ValueError("pid must be a positive integer")
    suffix = uuid.uuid4().hex[:8] if short_uuid is None else str(short_uuid)
    if re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", suffix) is None:
        raise ValueError("short_uuid must be a safe basename suffix")
    basename = "{}-p{}-{}".format(
        timestamp.strftime("%Y%m%d_%H%M%S_%f"),
        process_id,
        suffix,
    )
    root = output_root / basename
    root.mkdir(mode=0o700)
    os.chmod(str(root), 0o700)
    details = root / "attempt_details"
    details.mkdir(mode=0o700)
    os.chmod(str(details), 0o700)
    return SessionPaths(
        root=root,
        session_json=root / "session.json",
        ball_samples_csv=root / "ball_samples.csv",
        events_jsonl=root / "events.jsonl",
        replay_jobs_jsonl=root / "replay_jobs.jsonl",
        replay_analysis_jsonl=root / "replay_analysis.jsonl",
        attempts_csv=root / "attempts.csv",
        attempt_details_dir=details,
    )


def _atomic_json(
    path: Path,
    value: Mapping[str, Any],
    *,
    deadline_s: Optional[float] = None,
    failure_value: Optional[Mapping[str, Any]] = None,
) -> bool:
    payload = (
        json.dumps(
            to_builtin_json(value),
            ensure_ascii=False,
            allow_nan=False,
            separators=(",", ":"),
        )
        + "\n"
    )
    temporary = path.with_name(".{}.{}.tmp".format(path.name, uuid.uuid4().hex))
    descriptor = -1
    stream: Optional[TextIO] = None
    replaced = False

    def restore_fail_closed_value() -> None:
        if replaced and failure_value is not None:
            _atomic_json(path, failure_value)

    try:
        _check_deadline(deadline_s)
        descriptor = os.open(
            str(temporary),
            os.O_WRONLY | os.O_CREAT | os.O_EXCL,
            0o600,
        )
        _check_deadline(deadline_s)
        try:
            stream = os.fdopen(descriptor, "w", encoding="utf-8")
        except BaseException:
            os.close(descriptor)
            descriptor = -1
            raise
        descriptor = -1
        _check_deadline(deadline_s)
        stream.write(payload)
        _check_deadline(deadline_s)
        stream.flush()
        _check_deadline(deadline_s)
        os.fsync(stream.fileno())
        _check_deadline(deadline_s)
        stream.close()
        stream = None
        _check_deadline(deadline_s)
        os.replace(str(temporary), str(path))
        replaced = True
        _check_deadline(deadline_s)
        os.chmod(str(path), 0o600)
        _check_deadline(deadline_s)
        directory_fd = os.open(str(path.parent), os.O_RDONLY)
        try:
            _check_deadline(deadline_s)
            os.fsync(directory_fd)
            _check_deadline(deadline_s)
        finally:
            os.close(directory_fd)
        return True
    except _RecordingDeadlineExpired:
        if stream is not None:
            try:
                _discard_buffered_text_and_close(stream)
            except (OSError, ValueError):
                pass
        elif descriptor >= 0:
            try:
                os.close(descriptor)
            except OSError:
                pass
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
        restore_fail_closed_value()
        return False
    except BaseException as exc:
        if stream is not None:
            try:
                _discard_buffered_text_and_close(stream)
            except (OSError, ValueError):
                pass
        elif descriptor >= 0:
            try:
                os.close(descriptor)
            except OSError:
                pass
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
        try:
            restore_fail_closed_value()
        except BaseException as restore_exc:
            raise restore_exc from exc
        raise


def _open_private(path: Path) -> TextIO:
    descriptor = os.open(
        str(path),
        os.O_WRONLY | os.O_CREAT | os.O_EXCL,
        0o600,
    )
    try:
        os.chmod(str(path), 0o600)
        return os.fdopen(descriptor, "w", encoding="utf-8", newline="")
    except BaseException:
        os.close(descriptor)
        raise


class RawRecordLane:
    def __init__(
        self,
        capacity: int = 65536,
        *,
        drop_metadata_capacity: int = 1024,
    ) -> None:
        for name, value in (
            ("capacity", capacity),
            ("drop_metadata_capacity", drop_metadata_capacity),
        ):
            if isinstance(value, bool) or not isinstance(value, int):
                raise TypeError("{} must be an integer".format(name))
            if value <= 0:
                raise ValueError("{} must be positive".format(name))
        self._capacity = int(capacity)
        self._drop_metadata_capacity = int(drop_metadata_capacity)
        self._condition = threading.Condition(threading.Lock())
        self._records: Deque[RawRecordEnvelope] = deque()
        self._active_drop: Optional[RawDropRange] = None
        self._drop_notices: Deque[RawDropRange] = deque()
        self._drop_metadata_overflow = False
        self._closed = False

    def submit(
        self,
        sample: NormalizedMocapSample,
        *,
        attempt_id: Optional[int] = None,
        track_segment_id: Optional[int] = None,
    ) -> RawSubmitResult:
        envelope = RawRecordEnvelope(sample, attempt_id, track_segment_id)
        with self._condition:
            if self._closed:
                return RawSubmitResult(False, None, attempt_id, "CLOSED")
            if len(self._records) >= self._capacity:
                sequence = sample.input_seq
                pending = self._active_drop
                if pending is not None and pending[2] == attempt_id and sequence == pending[1] + 1:
                    self._active_drop = (pending[0], sequence, attempt_id)
                else:
                    self._active_drop = (sequence, sequence, attempt_id)
                self._record_drop_notice(sequence, attempt_id)
                current = self._active_drop
                return RawSubmitResult(
                    False,
                    (current[0], current[1]),
                    current[2],
                    "FULL",
                )
            completed = self._active_drop
            self._active_drop = None
            self._records.append(envelope)
            self._condition.notify()
            return RawSubmitResult(
                True,
                None if completed is None else (completed[0], completed[1]),
                None if completed is None else completed[2],
                None,
            )

    def take_many(
        self,
        *,
        limit: int,
        timeout_s: float = 0.0,
    ) -> Tuple[RawRecordEnvelope, ...]:
        if limit <= 0:
            raise ValueError("limit must be positive")
        if timeout_s < 0.0:
            raise ValueError("timeout_s must be non-negative")
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while not self._records and not self._closed:
                if timeout_s == 0.0:
                    return ()
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return ()
                self._condition.wait(remaining)
            items = []
            while self._records and len(items) < limit:
                items.append(self._records.popleft())
            return tuple(items)

    def take_drop_ranges(self) -> Tuple[RawDropNotice, ...]:
        with self._condition:
            items = list(self._drop_notices)
            self._drop_notices.clear()
            if self._drop_metadata_overflow:
                items.append(RAW_DROP_METADATA_OVERFLOW)
                self._drop_metadata_overflow = False
            return tuple(items)

    def pending_drop_metadata(self) -> bool:
        with self._condition:
            return bool(self._drop_notices or self._drop_metadata_overflow)

    def pending_count(self) -> int:
        with self._condition:
            return len(self._records)

    def close(self) -> None:
        with self._condition:
            if self._closed:
                return
            self._closed = True
            self._active_drop = None
            self._condition.notify_all()

    def _record_drop_notice(
        self,
        sequence: int,
        attempt_id: Optional[int],
    ) -> None:
        if self._drop_metadata_overflow:
            return
        if self._drop_notices:
            previous = self._drop_notices[-1]
            if previous[2] == attempt_id and sequence == previous[1] + 1:
                self._drop_notices[-1] = (
                    previous[0],
                    sequence,
                    attempt_id,
                )
                return
        if len(self._drop_notices) >= self._drop_metadata_capacity:
            self._drop_metadata_overflow = True
            return
        self._drop_notices.append((sequence, sequence, attempt_id))


def _summary_from_json(value: Mapping[str, Any]) -> AttemptSummary:
    return AttemptSummary(
        attempt_id=int(value["attempt_id"]),
        status=str(value["status"]),
        stage=str(value["stage"]),
        primary_blocker=value["primary_blocker"],
        ball_speed_mps=value["ball_speed_mps"],
        predicted_strike_time_s=value["predicted_strike_time_s"],
        planner_tts_s=value["planner_tts_s"],
        arm_tts_s=value["arm_tts_s"],
        task_obs_status=str(value["task_obs_status"]),
        ab_summary=value["ab_summary"],
        recording_complete=bool(value["recording_complete"]),
        estimator_sample_count=value.get("estimator_sample_count"),
        estimator_window_size=value.get("estimator_window_size"),
        incoming_count=value.get("incoming_count"),
        incoming_required_count=value.get("incoming_required_count"),
    )


def _detail_from_json(value: Mapping[str, Any]) -> AttemptDetail:
    if value.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("unsupported attempt detail schema")
    return AttemptDetail(
        attempt_id=int(value["attempt_id"]),
        summary=_summary_from_json(value["summary"]),
        segments=tuple(value["segments"]),
        stage_timeline=tuple(value["stage_timeline"]),
        planner_inputs=tuple(value["planner_inputs"]),
        planner_results=tuple(value["planner_results"]),
        task_observation_pre_clip=(
            None if value["task_observation_pre_clip"] is None else tuple(value["task_observation_pre_clip"])
        ),
        task_observation_post_clip=(
            None if value["task_observation_post_clip"] is None else tuple(value["task_observation_post_clip"])
        ),
        task_observation_clip_count=value["task_observation_clip_count"],
        variant_outcomes=tuple(value["variant_outcomes"]),
        ab_deltas=value["ab_deltas"],
    )


class AttemptDetailRepository:
    def __init__(self, details_dir: Path, cache_size: int = 100) -> None:
        if isinstance(cache_size, bool) or not isinstance(cache_size, int):
            raise TypeError("cache_size must be an integer")
        if cache_size <= 0:
            raise ValueError("cache_size must be positive")
        self._details_dir = Path(details_dir)
        self._details_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
        os.chmod(str(self._details_dir), 0o700)
        self._cache_size = int(cache_size)
        self._cache: "OrderedDict[int, AttemptDetail]" = OrderedDict()
        self._lock = threading.RLock()

    def commit(
        self,
        detail: AttemptDetail,
        *,
        deadline_s: Optional[float] = None,
    ) -> bool:
        if _deadline_expired(deadline_s):
            return False
        with self._lock:
            if _deadline_expired(deadline_s):
                return False
            path = self._details_dir / "{}.json".format(detail.attempt_id)
            fail_closed_detail = replace(
                detail,
                summary=replace(
                    detail.summary,
                    ab_summary="INCONCLUSIVE",
                    recording_complete=False,
                ),
            )
            if not _atomic_json(
                path,
                detail.to_json_dict(),
                deadline_s=deadline_s,
                failure_value=fail_closed_detail.to_json_dict(),
            ):
                self._cache.pop(detail.attempt_id, None)
                return False
            self._cache[detail.attempt_id] = detail
            self._cache.move_to_end(detail.attempt_id)
            while len(self._cache) > self._cache_size:
                self._cache.popitem(last=False)
            return True

    def get(self, attempt_id: int) -> Optional[AttemptDetail]:
        self._validate_id(attempt_id)
        with self._lock:
            cached = self._cache.get(attempt_id)
            if cached is not None:
                self._cache.move_to_end(attempt_id)
                return cached
            path = self._details_dir / "{}.json".format(attempt_id)
            value = self._read_regular_json(path)
            if value is None:
                return None
            detail = _detail_from_json(value)
            if detail.attempt_id != attempt_id:
                raise ValueError("attempt detail id does not match its filename")
            self._cache[attempt_id] = detail
            self._cache.move_to_end(attempt_id)
            while len(self._cache) > self._cache_size:
                self._cache.popitem(last=False)
            return detail

    @staticmethod
    def _read_regular_json(path: Path) -> Optional[Mapping[str, Any]]:
        try:
            before = os.stat(str(path), follow_symlinks=False)
        except FileNotFoundError:
            return None
        if not stat.S_ISREG(before.st_mode):
            return None
        flags = os.O_RDONLY
        flags |= getattr(os, "O_CLOEXEC", 0)
        flags |= getattr(os, "O_NOFOLLOW", 0)
        try:
            descriptor = os.open(str(path), flags)
        except OSError as exc:
            if exc.errno in (errno.ENOENT, errno.ENOTDIR, errno.ELOOP):
                return None
            raise
        try:
            opened = os.fstat(descriptor)
            if not stat.S_ISREG(opened.st_mode) or opened.st_dev != before.st_dev or opened.st_ino != before.st_ino:
                return None
            with os.fdopen(descriptor, "r", encoding="utf-8") as stream:
                descriptor = -1
                value = json.load(stream)
        finally:
            if descriptor >= 0:
                os.close(descriptor)
        if not isinstance(value, Mapping):
            raise ValueError("attempt detail must be a JSON object")
        return value

    def list_page(
        self,
        *,
        limit: int = 50,
        before: Optional[int] = None,
        deadline_s: Optional[float] = None,
    ) -> AttemptPage:
        if isinstance(limit, bool) or not isinstance(limit, int):
            raise TypeError("limit must be an integer")
        if limit <= 0 or limit > 200:
            raise ValueError("limit must be between 1 and 200")
        if before is not None:
            self._validate_id(before)
        with self._lock:

            def numeric_ids():
                if _deadline_expired(deadline_s):
                    raise _RecordingDeadlineExpired()
                with os.scandir(str(self._details_dir)) as entries:
                    iterator = iter(entries)
                    while True:
                        if _deadline_expired(deadline_s):
                            raise _RecordingDeadlineExpired()
                        try:
                            entry = next(iterator)
                        except StopIteration:
                            break
                        if _deadline_expired(deadline_s):
                            raise _RecordingDeadlineExpired()
                        if not entry.is_file(follow_symlinks=False):
                            continue
                        match = re.fullmatch(
                            r"([1-9][0-9]*)\.json",
                            entry.name,
                        )
                        if match is None:
                            continue
                        attempt_id = int(match.group(1))
                        if before is None or attempt_id < before:
                            yield attempt_id

            ids = heapq.nlargest(limit + 1, numeric_ids())
            details = []
            for attempt_id in ids:
                if _deadline_expired(deadline_s):
                    raise _RecordingDeadlineExpired()
                detail = self.get(attempt_id)
                if detail is not None:
                    details.append(detail)
                if len(details) > limit:
                    break
            has_more = len(details) > limit
            selected = details[:limit]
            return AttemptPage(
                items=tuple(detail.summary for detail in selected),
                has_more=has_more,
                next_before=(selected[-1].attempt_id if has_more and selected else None),
            )

    @staticmethod
    def _validate_id(attempt_id: int) -> None:
        if isinstance(attempt_id, bool) or not isinstance(attempt_id, int):
            raise TypeError("attempt_id must be an integer")
        if attempt_id <= 0:
            raise ValueError("attempt_id must be positive")


_RAW_FIELDS = (
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
_ATTEMPT_FIELDS = (
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
)


class SessionRecorder:
    def __init__(
        self,
        *,
        paths: SessionPaths,
        raw_lane: RawRecordLane,
        event_cursor: EventCursor,
        attempt_repository: AttemptDetailRepository,
        session_metadata: Optional[Mapping[str, JsonValue]] = None,
        clock: Callable[[], float] = time.monotonic,
        event_sink: Optional[Callable[[EventDraft], object]] = None,
        offer_capacity: int = 1024,
        raw_quota: int = 512,
        event_quota: int = 128,
        flush_interval_s: float = 1.0,
        flush_row_count: int = 1000,
    ) -> None:
        for name, value in (
            ("offer_capacity", offer_capacity),
            ("raw_quota", raw_quota),
            ("event_quota", event_quota),
            ("flush_row_count", flush_row_count),
        ):
            if isinstance(value, bool) or not isinstance(value, int):
                raise TypeError("{} must be an integer".format(name))
            if value <= 0:
                raise ValueError("{} must be positive".format(name))
        if flush_interval_s < 0.0:
            raise ValueError("flush_interval_s must be non-negative")
        self._paths = paths
        self._raw_lane = raw_lane
        self._event_cursor = event_cursor
        self._repository = attempt_repository
        metadata = freeze_json_value({} if session_metadata is None else session_metadata)
        if not isinstance(metadata, Mapping):
            raise TypeError("session_metadata must be a mapping")
        self._session_metadata = metadata
        self._clock = clock
        self._event_sink = event_sink
        self._offer_capacity = int(offer_capacity)
        self._raw_quota = int(raw_quota)
        self._event_quota = int(event_quota)
        self._flush_interval_s = float(flush_interval_s)
        self._flush_row_count = int(flush_row_count)
        self._lock = threading.RLock()
        self._consumer_lock = threading.RLock()
        self._detail_offers: Deque[AttemptDetail] = deque()
        self._analysis_offers: Deque[Mapping[str, JsonValue]] = deque()
        self._job_offers: Deque[Mapping[str, JsonValue]] = deque()
        self._pending_details: "OrderedDict[int, AttemptDetail]" = OrderedDict()
        self._written_attempt_ids: "OrderedDict[int, None]" = OrderedDict()
        self._incomplete_attempt_ids: "OrderedDict[int, None]" = OrderedDict()
        self._global_incomplete = False
        self._healthy = True
        self._recording_complete = True
        self._last_error: Optional[str] = None
        self._started = False
        self._accepting = True
        self._closed = False
        self._terminal_status: Optional[RecorderStatus] = None
        self._files: Dict[str, TextIO] = {}
        self._raw_writer: Optional[csv.DictWriter] = None
        self._attempt_writer: Optional[csv.DictWriter] = None
        self._rows_since_flush = 0
        self._last_flush_s = self._clock()
        next_event_id = getattr(event_cursor, "_next_event_id", None)
        if isinstance(next_event_id, int) and not isinstance(next_event_id, bool) and next_event_id > 0:
            event_baseline = next_event_id - 1
        else:
            event_baseline = 0
        self._persisted_event_id = event_baseline
        self._observed_watermark = event_baseline
        self._event_gap_seen = False

    def start(self) -> None:
        with self._consumer_lock:
            with self._lock:
                if self._started:
                    return
            opened: Dict[str, TextIO] = {}
            try:
                running = dict(to_builtin_json(self._session_metadata))
                running.update({"schema_version": SCHEMA_VERSION, "status": "RUNNING"})
                _atomic_json(self._paths.session_json, running)
                for name, path in (
                    ("raw", self._paths.ball_samples_csv),
                    ("events", self._paths.events_jsonl),
                    ("jobs", self._paths.replay_jobs_jsonl),
                    ("analysis", self._paths.replay_analysis_jsonl),
                    ("attempts", self._paths.attempts_csv),
                ):
                    opened[name] = _open_private(path)
                self._files = opened
                self._raw_writer = csv.DictWriter(
                    self._files["raw"],
                    fieldnames=_RAW_FIELDS,
                )
                self._raw_writer.writeheader()
                self._attempt_writer = csv.DictWriter(
                    self._files["attempts"],
                    fieldnames=_ATTEMPT_FIELDS,
                )
                self._attempt_writer.writeheader()
                self._flush_all()
                self._started = True
            except BaseException as exc:
                for stream in opened.values():
                    try:
                        stream.close()
                    except OSError:
                        pass
                self._files = {}
                self._raw_writer = None
                self._attempt_writer = None
                self._started = False
                self._record_error(exc)

    def offer_attempt_detail(self, detail: AttemptDetail) -> bool:
        return self._offer(self._detail_offers, detail)

    def mark_incomplete(
        self,
        attempt_id: Optional[int],
        reason: str,
    ) -> None:
        """Mark the session and, when known, one attempt as incomplete."""
        if attempt_id is not None:
            if isinstance(attempt_id, bool) or not isinstance(attempt_id, int) or attempt_id <= 0:
                raise ValueError("attempt_id must be positive or None")
        normalized_reason = str(reason).strip()
        if not normalized_reason:
            raise ValueError("reason must be non-empty")
        self._mark_incomplete(attempt_id, normalized_reason)

    def offer_replay_analysis(
        self,
        analysis: Mapping[str, JsonValue],
    ) -> bool:
        frozen = freeze_json_value(analysis)
        if not isinstance(frozen, Mapping):
            raise TypeError("analysis must be a mapping")
        return self._offer(self._analysis_offers, frozen)

    def offer_replay_job_status(
        self,
        status: Mapping[str, JsonValue],
    ) -> bool:
        frozen = freeze_json_value(status)
        if not isinstance(frozen, Mapping):
            raise TypeError("job status must be a mapping")
        return self._offer(self._job_offers, frozen)

    def request_stop(self) -> None:
        with self._lock:
            self._accepting = False

    def drain(self, timeout_s: float) -> RecorderStatus:
        if timeout_s < 0.0:
            raise ValueError("timeout_s must be non-negative")
        with self._consumer_lock:
            if self._closed:
                return self.status()
            if timeout_s == 0.0:
                self._drain_round()
                return self.status()
            snapshot = self._capture_drain_snapshot()
            deadline = time.monotonic() + timeout_s
            while not snapshot.exhausted(
                next_event_id=int(
                    getattr(
                        self._event_cursor,
                        "_next_event_id",
                        self._persisted_event_id + 1,
                    )
                ),
                event_gap_seen=self._event_gap_seen,
            ):
                progressed = self._drain_round(snapshot=snapshot)
                if not progressed or time.monotonic() >= deadline:
                    break
            return self.status()

    def confirm_replay_persistence(
        self,
        timeout_s: float,
    ) -> RecorderStatus:
        """Drain one replay-visible snapshot and force it through fsync."""
        if timeout_s < 0.0:
            raise ValueError("timeout_s must be non-negative")
        with self._consumer_lock:
            if not self._started or self._closed:
                self._mark_incomplete(
                    None,
                    "REPLAY_PERSISTENCE_UNAVAILABLE",
                )
                return self._status_snapshot()

            snapshot = self._capture_drain_snapshot()
            deadline = time.monotonic() + timeout_s

            def snapshot_exhausted() -> bool:
                return snapshot.exhausted(
                    next_event_id=int(
                        getattr(
                            self._event_cursor,
                            "_next_event_id",
                            self._persisted_event_id + 1,
                        )
                    ),
                    event_gap_seen=self._event_gap_seen,
                )

            while not snapshot_exhausted() and not _deadline_expired(deadline):
                progressed = self._drain_round(
                    deadline_s=deadline,
                    snapshot=snapshot,
                )
                if not progressed:
                    break

            if not snapshot_exhausted():
                self._mark_incomplete(
                    None,
                    ("REPLAY_PERSISTENCE_DEADLINE" if _deadline_expired(deadline) else "REPLAY_PERSISTENCE_BACKLOG"),
                )
            else:
                try:
                    if not self._flush_all(deadline_s=deadline):
                        self._mark_incomplete(
                            None,
                            "REPLAY_PERSISTENCE_DEADLINE:FLUSH",
                        )
                except BaseException as exc:
                    self._record_error(exc)
            return self._status_snapshot(deadline_s=deadline)

    def _capture_drain_snapshot(self) -> _DrainSnapshot:
        hub = getattr(self._event_cursor, "_hub", None)
        if hub is None:
            event_watermark = self._observed_watermark
        else:
            event_watermark = int(hub.state_snapshot().watermark_event_id)
        with self._lock:
            detail_count = len(self._detail_offers)
            analysis_count = len(self._analysis_offers)
            job_count = len(self._job_offers)
        return _DrainSnapshot(
            raw_remaining=self._raw_lane.pending_count(),
            event_watermark=event_watermark,
            detail_remaining=detail_count,
            analysis_remaining=analysis_count,
            job_remaining=job_count,
            drop_notices=self._raw_lane.take_drop_ranges(),
        )

    def write_terminal_and_close(
        self,
        *,
        terminal_session: Mapping[str, JsonValue],
        timeout_s: float,
    ) -> RecorderStatus:
        if timeout_s < 0.0:
            raise ValueError("timeout_s must be non-negative")
        with self._consumer_lock:
            with self._lock:
                if self._terminal_status is not None:
                    return self._terminal_status
                self._accepting = False
            self._raw_lane.close()
            deadline = time.monotonic() + timeout_s
            self._drain_round(deadline_s=deadline)
            captured_watermark = self._observed_watermark
            while self._terminal_has_backlog(captured_watermark) and not _deadline_expired(deadline):
                event_remaining = captured_watermark - self._persisted_event_id
                progressed = self._drain_round(
                    event_limit=(
                        min(self._event_quota, event_remaining)
                        if event_remaining > 0 and not self._event_gap_seen
                        else 0
                    ),
                    deadline_s=deadline,
                )
                if not progressed:
                    break

            backlog = self._terminal_backlog_reasons(captured_watermark)
            if backlog:
                self._mark_incomplete(
                    None,
                    "{}:{}".format(
                        ("SHUTDOWN_DEADLINE_EXPIRED" if _deadline_expired(deadline) else "SHUTDOWN_DRAIN_TIMEOUT"),
                        ",".join(backlog),
                    ),
                )

            try:
                if not self._finalize_all_attempts(deadline_s=deadline):
                    if self._recording_complete:
                        self._mark_incomplete(
                            None,
                            ("SHUTDOWN_DEADLINE_EXPIRED:" "ATTEMPT_FINALIZATION"),
                        )
                if not _deadline_expired(deadline):
                    if not self._flush_all(deadline_s=deadline):
                        self._mark_incomplete(
                            None,
                            "SHUTDOWN_DEADLINE_EXPIRED:FLUSH",
                        )
                if not _deadline_expired(deadline):
                    terminal = dict(to_builtin_json(self._session_metadata))
                    terminal.update(to_builtin_json(terminal_session))
                    terminal.update(
                        {
                            "schema_version": SCHEMA_VERSION,
                            "status": ("COMPLETE" if self._healthy and self._recording_complete else "INCOMPLETE"),
                            "recording_complete": self._recording_complete,
                            "recorder_healthy": self._healthy,
                            "last_error": self._last_error,
                            "watermark_event_id": self._persisted_event_id,
                            "observed_watermark_event_id": (captured_watermark),
                        }
                    )
                    if not _deadline_expired(deadline):
                        if not _atomic_json(
                            self._paths.session_json,
                            terminal,
                            deadline_s=deadline,
                        ):
                            self._mark_incomplete(
                                None,
                                ("SHUTDOWN_DEADLINE_EXPIRED:" "SESSION_TERMINAL"),
                            )
                elif self._recording_complete:
                    self._mark_incomplete(
                        None,
                        "SHUTDOWN_DEADLINE_EXPIRED:SESSION_TERMINAL",
                    )
            except BaseException as exc:
                self._record_error(exc)
            for stream in self._files.values():
                try:
                    if _deadline_expired(deadline):
                        _discard_buffered_text_and_close(stream)
                    else:
                        stream.close()
                except OSError as exc:
                    self._record_error(exc)
            try:
                self._event_cursor.close()
            except RuntimeError as exc:
                self._record_error(exc)
            with self._lock:
                self._closed = True
                self._terminal_status = self._status_snapshot(
                    deadline_s=deadline,
                )
                return self._terminal_status

    def status(self) -> RecorderStatus:
        return self._status_snapshot()

    def _status_snapshot(
        self,
        *,
        deadline_s: Optional[float] = None,
    ) -> RecorderStatus:
        with self._lock:
            total = 0
            for path in (
                self._paths.session_json,
                self._paths.ball_samples_csv,
                self._paths.events_jsonl,
                self._paths.replay_jobs_jsonl,
                self._paths.replay_analysis_jsonl,
                self._paths.attempts_csv,
            ):
                if _deadline_expired(deadline_s):
                    break
                try:
                    total += path.stat().st_size
                except OSError:
                    pass
            return RecorderStatus(
                healthy=self._healthy,
                recording_complete=self._recording_complete,
                bytes_written=total,
                last_error=self._last_error,
            )

    def _offer(self, queue: Deque[Any], value: Any) -> bool:
        with self._lock:
            if not self._accepting or self._closed:
                return False
            if len(queue) >= self._offer_capacity:
                self._mark_incomplete(None, "RECORDER_OFFER_QUEUE_FULL")
                return False
            queue.append(value)
            return True

    def _drain_round(
        self,
        *,
        event_limit: Optional[int] = None,
        deadline_s: Optional[float] = None,
        snapshot: Optional[_DrainSnapshot] = None,
    ) -> bool:
        if not self._started or self._closed:
            return False
        progressed = False
        with self._lock:
            detail_offer_count = min(
                len(self._detail_offers),
                (snapshot.detail_remaining if snapshot is not None else len(self._detail_offers)),
            )
            analysis_offer_count = min(
                len(self._analysis_offers),
                (snapshot.analysis_remaining if snapshot is not None else len(self._analysis_offers)),
            )
            job_offer_count = min(
                len(self._job_offers),
                (snapshot.job_remaining if snapshot is not None else len(self._job_offers)),
            )
        try:
            if snapshot is None:
                drop_notices = self._raw_lane.take_drop_ranges()
            else:
                drop_notices = snapshot.drop_notices
                snapshot.drop_notices = ()
            for notice in drop_notices:
                progressed = True
                if notice == RAW_DROP_METADATA_OVERFLOW:
                    self._mark_incomplete(
                        None,
                        "RAW_DROP_METADATA_OVERFLOW",
                    )
                    continue
                first, last, attempt_id = notice
                self._mark_incomplete(attempt_id, "RAW_SAMPLES_DROPPED")
                if self._event_sink is not None:
                    draft = EventDraft(
                        kind="RAW_SAMPLES_DROPPED",
                        monotonic_s=self._clock(),
                        wall_time_us=0,
                        scope="attempt" if attempt_id is not None else "state",
                        attempt_id=attempt_id,
                        payload={
                            "first_input_seq": first,
                            "last_input_seq": last,
                        },
                    )
                    if not _deadline_expired(deadline_s):
                        accepted = self._event_sink(draft)
                        if accepted is False:
                            self._mark_incomplete(
                                attempt_id,
                                "DIAGNOSTIC_EVENT_DROPPED",
                            )
            if _deadline_expired(deadline_s):
                return progressed
            raw_limit = 1 if deadline_s is not None else self._raw_quota
            if snapshot is not None:
                raw_limit = min(raw_limit, snapshot.raw_remaining)
            raw = (
                ()
                if raw_limit <= 0
                else self._raw_lane.take_many(
                    limit=raw_limit,
                    timeout_s=0.0,
                )
            )
            if snapshot is not None:
                snapshot.raw_remaining -= len(raw)
            events: Tuple[PublishedEvent, ...] = ()
            effective_event_limit = event_limit
            if snapshot is not None:
                next_event_id = int(
                    getattr(
                        self._event_cursor,
                        "_next_event_id",
                        self._persisted_event_id + 1,
                    )
                )
                snapshot_event_limit = max(
                    0,
                    snapshot.event_watermark - next_event_id + 1,
                )
                effective_event_limit = (
                    snapshot_event_limit
                    if effective_event_limit is None
                    else min(effective_event_limit, snapshot_event_limit)
                )
            if effective_event_limit is None or effective_event_limit > 0:
                if _deadline_expired(deadline_s):
                    return progressed
                event_read = self._event_cursor.read(
                    limit=(
                        1
                        if deadline_s is not None
                        else (
                            self._event_quota
                            if effective_event_limit is None
                            else min(self._event_quota, effective_event_limit)
                        )
                    ),
                    timeout_s=0.0,
                )
                self._observed_watermark = max(
                    self._observed_watermark,
                    event_read.watermark_event_id,
                )
                events = event_read.events
                if event_read.lost_event_ids is not None:
                    progressed = True
                    self._event_gap_seen = True
                    try:
                        if _deadline_expired(deadline_s):
                            return progressed
                        self._write_json_line(
                            self._files["events"],
                            {
                                "schema_version": SCHEMA_VERSION,
                                "kind": "RECORDER_EVENT_GAP",
                                "monotonic_s": self._clock(),
                                "wall_time_us": 0,
                                "scope": "state",
                                "attempt_id": None,
                                "payload": {
                                    "lost_event_ids": (event_read.lost_event_ids),
                                    "watermark_event_id": (event_read.watermark_event_id),
                                },
                            },
                        )
                    finally:
                        self._mark_incomplete(
                            None,
                            "RECORDER_EVENT_GAP",
                        )
            progressed = progressed or bool(raw) or bool(events)
            for envelope in raw:
                if _deadline_expired(deadline_s):
                    break
                self._write_raw(envelope)
            for event in events:
                if _deadline_expired(deadline_s):
                    break
                self._write_event(event)
                self._persisted_event_id = event.event_id

            def consume_offers(queue, consumer, offer_count) -> int:
                nonlocal progressed
                consumed = 0
                for _ in range(offer_count):
                    if _deadline_expired(deadline_s):
                        return consumed
                    with self._lock:
                        if not queue:
                            return consumed
                        value = queue[0]
                    if consumer(value) is False:
                        return consumed
                    with self._lock:
                        if queue and queue[0] is value:
                            queue.popleft()
                            consumed += 1
                    progressed = True
                return consumed

            def consume_detail(value):
                if deadline_s is None:
                    return self._consume_detail(value)
                return self._consume_detail(
                    value,
                    deadline_s=deadline_s,
                )

            consumed_details = consume_offers(
                self._detail_offers,
                consume_detail,
                detail_offer_count,
            )
            consumed_analysis = consume_offers(
                self._analysis_offers,
                lambda value: self._write_json_line(
                    self._files["analysis"],
                    value,
                ),
                analysis_offer_count,
            )
            consumed_jobs = consume_offers(
                self._job_offers,
                lambda value: self._write_json_line(
                    self._files["jobs"],
                    value,
                ),
                job_offer_count,
            )
            if snapshot is not None:
                snapshot.detail_remaining -= consumed_details
                snapshot.analysis_remaining -= consumed_analysis
                snapshot.job_remaining -= consumed_jobs
            self._maybe_flush(deadline_s=deadline_s)
        except BaseException as exc:
            self._record_error(exc)
        return progressed

    def _write_raw(self, envelope: RawRecordEnvelope) -> None:
        sample = envelope.sample
        self._raw_writer.writerow(
            {
                "input_seq": sample.input_seq,
                "attempt_id": envelope.attempt_id,
                "track_segment_id": envelope.track_segment_id,
                "channel": sample.channel,
                "subject": sample.subject,
                "position_w": json.dumps(
                    to_builtin_json(sample.position_w),
                    allow_nan=False,
                    separators=(",", ":"),
                ),
                "quaternion_xyzw": json.dumps(
                    to_builtin_json(sample.quaternion_xyzw),
                    allow_nan=False,
                    separators=(",", ":"),
                ),
                "valid": sample.valid,
                "occluded": sample.occluded,
                "source_frame": sample.source_frame,
                "source_time_s": to_builtin_json(sample.source_time_s),
                "publish_time_us": sample.publish_time_us,
                "received_monotonic_s": to_builtin_json(sample.received_monotonic_s),
                "wall_time_us": sample.wall_time_us,
                "payload_size": sample.payload_size,
            }
        )
        self._rows_since_flush += 1

    def _write_event(self, event: PublishedEvent) -> None:
        self._write_json_line(self._files["events"], event.to_json_dict())

    def _write_json_line(
        self,
        stream: TextIO,
        value: Mapping[str, Any],
    ) -> None:
        stream.write(
            json.dumps(
                to_builtin_json(value),
                ensure_ascii=False,
                allow_nan=False,
                separators=(",", ":"),
            )
        )
        stream.write("\n")
        self._rows_since_flush += 1

    def _consume_detail(
        self,
        detail: AttemptDetail,
        *,
        deadline_s: Optional[float] = None,
    ) -> bool:
        if (
            self._global_incomplete
            or detail.attempt_id in self._incomplete_attempt_ids
            or not detail.summary.recording_complete
        ):
            detail = self._inconclusive(detail, force_incomplete=True)
        existing = self._repository.get(detail.attempt_id)
        if existing != detail:
            if not self._repository.commit(
                detail,
                deadline_s=deadline_s,
            ):
                return False
        self._pending_details[detail.attempt_id] = detail
        self._pending_details.move_to_end(detail.attempt_id)
        while len(self._pending_details) > self._offer_capacity:
            self._pending_details.popitem(last=False)
        return True

    def _finalize_all_attempts(
        self,
        *,
        deadline_s: Optional[float] = None,
    ) -> bool:
        before: Optional[int] = None
        while True:
            if _deadline_expired(deadline_s):
                return False
            try:
                page = self._repository.list_page(
                    limit=200,
                    before=before,
                    deadline_s=deadline_s,
                )
            except _RecordingDeadlineExpired:
                return False
            for summary in page.items:
                if _deadline_expired(deadline_s):
                    return False
                detail = self._repository.get(summary.attempt_id)
                if detail is None:
                    continue
                if (
                    self._global_incomplete
                    or detail.attempt_id in self._incomplete_attempt_ids
                    or not detail.summary.recording_complete
                ):
                    detail = self._inconclusive(
                        detail,
                        force_incomplete=True,
                    )
                elif detail.summary.ab_summary is None:
                    detail = self._inconclusive(detail)
                if _deadline_expired(deadline_s):
                    return False
                if not self._repository.commit(
                    detail,
                    deadline_s=deadline_s,
                ):
                    return False
                if _deadline_expired(deadline_s):
                    return False
                if not self._write_attempt_once(
                    detail.summary,
                    deadline_s=deadline_s,
                ):
                    self._mark_incomplete(
                        detail.attempt_id,
                        "ATTEMPT_FINALIZATION_DEADLINE",
                    )
                    corrected = self._pending_details.get(
                        detail.attempt_id,
                        self._inconclusive(
                            detail,
                            force_incomplete=True,
                        ),
                    )
                    self._repository.commit(corrected)
                    return False
                self._pending_details.pop(detail.attempt_id, None)
                self._incomplete_attempt_ids.pop(
                    detail.attempt_id,
                    None,
                )
            if not page.has_more:
                break
            before = page.next_before
        return True

    def _write_attempt_once(
        self,
        summary: AttemptSummary,
        *,
        deadline_s: Optional[float] = None,
    ) -> bool:
        already_written = self._attempt_already_written(
            summary.attempt_id,
            deadline_s=deadline_s,
        )
        if already_written is None:
            return False
        if already_written:
            return True
        if _deadline_expired(deadline_s):
            return False
        attempts_stream = self._files["attempts"]
        row_start = attempts_stream.tell()
        self._attempt_writer.writerow({field: getattr(summary, field) for field in _ATTEMPT_FIELDS})
        self._rows_since_flush += 1
        if self._flush_all(deadline_s=deadline_s):
            self._remember_written_attempt(summary.attempt_id)
            return True
        attempts_stream.seek(row_start)
        attempts_stream.truncate()
        attempts_stream.flush()
        self._rows_since_flush = max(0, self._rows_since_flush - 1)
        return False

    def _attempt_already_written(
        self,
        attempt_id: int,
        *,
        deadline_s: Optional[float] = None,
    ) -> Optional[bool]:
        if _deadline_expired(deadline_s):
            return None
        if attempt_id in self._written_attempt_ids:
            self._written_attempt_ids.move_to_end(attempt_id)
            return True
        attempts_stream = self._files.get("attempts")
        if attempts_stream is not None:
            if _deadline_expired(deadline_s):
                return None
            attempts_stream.flush()
        if _deadline_expired(deadline_s):
            return None
        try:
            with self._paths.attempts_csv.open(
                "r",
                encoding="utf-8",
                newline="",
            ) as stream:

                def deadline_checked_lines():
                    while True:
                        if _deadline_expired(deadline_s):
                            raise _RecordingDeadlineExpired()
                        line = stream.readline()
                        if not line:
                            return
                        yield line

                for row in csv.DictReader(deadline_checked_lines()):
                    if _deadline_expired(deadline_s):
                        return None
                    if row.get("attempt_id") == str(attempt_id):
                        self._remember_written_attempt(attempt_id)
                        return True
        except _RecordingDeadlineExpired:
            return None
        except FileNotFoundError:
            return False
        return False

    def _remember_written_attempt(self, attempt_id: int) -> None:
        self._written_attempt_ids[attempt_id] = None
        self._written_attempt_ids.move_to_end(attempt_id)
        while len(self._written_attempt_ids) > self._offer_capacity:
            self._written_attempt_ids.popitem(last=False)

    def _inconclusive(
        self,
        detail: AttemptDetail,
        *,
        force_incomplete: bool = False,
    ) -> AttemptDetail:
        incomplete = force_incomplete or self._global_incomplete or detail.attempt_id in self._incomplete_attempt_ids
        summary = replace(
            detail.summary,
            ab_summary="INCONCLUSIVE",
            recording_complete=(False if incomplete else detail.summary.recording_complete),
        )
        return replace(detail, summary=summary)

    def _mark_incomplete(
        self,
        attempt_id: Optional[int],
        reason: str,
    ) -> None:
        with self._lock:
            self._recording_complete = False
            if attempt_id is None:
                self._global_incomplete = True
                self._incomplete_attempt_ids.clear()
            else:
                if (
                    attempt_id not in self._incomplete_attempt_ids
                    and len(self._incomplete_attempt_ids) >= self._offer_capacity
                ):
                    self._global_incomplete = True
                    self._incomplete_attempt_ids.clear()
                elif not self._global_incomplete:
                    self._incomplete_attempt_ids[attempt_id] = None
                    self._incomplete_attempt_ids.move_to_end(attempt_id)
            if self._healthy or self._last_error is None:
                self._last_error = str(reason)[:2048]
            for key, detail in tuple(self._pending_details.items()):
                if attempt_id is None or key == attempt_id:
                    self._pending_details[key] = self._inconclusive(
                        detail,
                        force_incomplete=True,
                    )
            for index, detail in enumerate(self._detail_offers):
                if attempt_id is None or detail.attempt_id == attempt_id:
                    self._detail_offers[index] = self._inconclusive(
                        detail,
                        force_incomplete=True,
                    )

    def _record_error(self, error: BaseException) -> None:
        with self._lock:
            self._healthy = False
            self._recording_complete = False
            self._global_incomplete = True
            self._incomplete_attempt_ids.clear()
            self._last_error = "{}: {}".format(
                type(error).__name__,
                str(error),
            )[:2048]
            for key, detail in tuple(self._pending_details.items()):
                self._pending_details[key] = self._inconclusive(
                    detail,
                    force_incomplete=True,
                )

    def _terminal_has_backlog(self, captured_watermark: int) -> bool:
        if self._raw_lane.pending_count():
            return True
        if self._raw_lane.pending_drop_metadata():
            return True
        with self._lock:
            if self._detail_offers or self._analysis_offers or self._job_offers:
                return True
        return not self._event_gap_seen and self._persisted_event_id < captured_watermark

    def _terminal_backlog_reasons(
        self,
        captured_watermark: int,
    ) -> Tuple[str, ...]:
        reasons = []
        if self._raw_lane.pending_count():
            reasons.append("RAW")
        if self._raw_lane.pending_drop_metadata():
            reasons.append("RAW_DROP_METADATA")
        with self._lock:
            if self._detail_offers:
                reasons.append("ATTEMPT_DETAILS")
            if self._analysis_offers:
                reasons.append("REPLAY_ANALYSIS")
            if self._job_offers:
                reasons.append("REPLAY_JOBS")
        if not self._event_gap_seen and self._persisted_event_id < captured_watermark:
            reasons.append("EVENTS")
        return tuple(reasons)

    def _maybe_flush(
        self,
        *,
        deadline_s: Optional[float] = None,
    ) -> bool:
        if (
            self._rows_since_flush >= self._flush_row_count
            or self._clock() - self._last_flush_s >= self._flush_interval_s
        ):
            return self._flush_all(deadline_s=deadline_s)
        return True

    def _flush_all(
        self,
        *,
        deadline_s: Optional[float] = None,
    ) -> bool:
        for stream in self._files.values():
            if _deadline_expired(deadline_s):
                return False
            stream.flush()
            if _deadline_expired(deadline_s):
                return False
            os.fsync(stream.fileno())
            if _deadline_expired(deadline_s):
                return False
        self._rows_since_flush = 0
        self._last_flush_s = self._clock()
        return True
