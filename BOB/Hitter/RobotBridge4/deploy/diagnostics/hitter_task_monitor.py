from __future__ import annotations

import argparse
from collections import OrderedDict, deque
from dataclasses import dataclass, replace
import hashlib
from importlib import metadata as importlib_metadata
import json
import math
import os
from pathlib import Path
import platform
import select
import signal
import subprocess
import sys
import threading
import time
from types import SimpleNamespace
from typing import (
    Any,
    Callable,
    Deque,
    Dict,
    Mapping,
    Optional,
    Sequence,
    Tuple,
)

import lcm
import numpy as np
from omegaconf import OmegaConf

_IMPORT_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_IMPORT_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_IMPORT_REPO_ROOT))

from unitree_sdk2.lcm_types.transformation_t import transformation_t

from diagnostics.hitter_task_attempts import AttemptTracker
from diagnostics.hitter_task_events import (
    DiagnosticState,
    EventHub,
    EventPublisherLane,
)
from diagnostics.hitter_task_models import (
    LIVE_SNAPSHOT_SCHEMA_VERSION,
    SCHEMA_VERSION,
    AttemptDetail,
    AttemptSummary,
    AttemptTransition,
    BallDiagnosticState,
    EventDraft,
    HealthSnapshot,
    LifecycleSnapshot,
    LiveDiagnosticSnapshot,
    ProductionGateState,
    SnapshotKey,
    SubjectHealth,
    to_builtin_json,
)
from diagnostics.hitter_task_pipeline import (
    MocapFrameAdapter,
    ShadowTaskPipeline,
    planner_reason_code,
)
from diagnostics.hitter_task_recording import (
    AttemptDetailRepository,
    RawRecordLane,
    RecorderStatus,
    SessionPaths,
    SessionRecorder,
    create_session_paths,
)
from diagnostics.hitter_task_replay import (
    ReplayCaptureStore,
    ReplayEventCaptureTee,
    ReplayJobRef,
    ReplayProcessController,
    ReplayRawCaptureTee,
    ReplayWorkerReply,
    run_replay_disk_job,
    write_replay_input_bundle,
)
from diagnostics.hitter_task_web import (
    HitterTaskWebApplication,
    HitterTaskWebServer,
    WebServerConfig,
)
from utils.hitter_runtime_factory import (
    HitterRuntimeSettings,
    build_hitter_system_planner,
    forced_strike_type,
    resolve_hitter_runtime_settings,
)


_REPO_ROOT = _IMPORT_REPO_ROOT
_DEPLOY_ROOT = Path(__file__).resolve().parents[1]
_DEFAULT_OUTPUT_DIR = _REPO_ROOT / "recordings/hitter_task_diagnostics"
_STATIC_PAGE = Path(__file__).resolve().parent / "static/hitter_task_monitor.html"
_SUBJECT_RATE_WINDOW_S = 1.0
_SUBJECT_RATE_CAPACITY = 2048
_RECORDER_POLL_S = 0.02
_RUN_POLL_S = 0.05
_LCM_SELECT_TIMEOUT_S = 0.1
_PACKAGE_NAMES = ("numpy", "scipy", "omegaconf", "lcm")
_SPECIAL_ATTEMPT_STAGES = frozenset(
    (
        "WAITING_FOR_PREVIOUS_RECOVERY",
        "CACHED_DURING_RECOVERY",
        "REACQUIRE_GRACE",
        "POST_DEADLINE_TAIL",
    )
)


def default_output_dir() -> Path:
    """Return the repository-stable default recording root."""
    return _DEFAULT_OUTPUT_DIR


def _finite_nonnegative(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError("{} must be a number".format(name))
    converted = float(value)
    if not math.isfinite(converted) or converted < 0.0:
        raise ValueError("{} must be finite and nonnegative".format(name))
    return converted


def _positive_integer(name: str, value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError("{} must be an integer".format(name))
    if value <= 0:
        raise ValueError("{} must be positive".format(name))
    return int(value)


def _finite_float_or_none(value: object) -> Optional[float]:
    try:
        converted = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return converted if math.isfinite(converted) else None


@dataclass(frozen=True)
class MonitorOptions:
    mimic_config: Path
    control_config: Path
    table_calib: Path
    pelvis_calib: Path
    lcm_url: str
    channel: str
    base_name: str
    ball_name: str
    port: int
    output_dir: Path
    duration_s: float
    reacquire_grace_s: float
    heartbeat_stale_s: float
    pelvis_stale_s: float
    source_frame_gap_threshold: int
    raw_capacity: int
    event_capacity: int
    attempt_cache_size: int
    input_capacity: int = 65536

    def __post_init__(self) -> None:
        for name in (
            "mimic_config",
            "control_config",
            "table_calib",
            "pelvis_calib",
            "output_dir",
        ):
            object.__setattr__(self, name, Path(getattr(self, name)))
        for name in ("lcm_url", "channel", "base_name", "ball_name"):
            value = str(getattr(self, name)).strip()
            if not value:
                raise ValueError("{} must be non-empty".format(name))
            object.__setattr__(self, name, value)
        if isinstance(self.port, bool) or not isinstance(self.port, int):
            raise TypeError("port must be an integer")
        if self.port < 0 or self.port > 65535:
            raise ValueError("port must be between 0 and 65535")
        for name in (
            "duration_s",
            "reacquire_grace_s",
            "heartbeat_stale_s",
            "pelvis_stale_s",
        ):
            object.__setattr__(
                self,
                name,
                _finite_nonnegative(name, getattr(self, name)),
            )
        if (
            isinstance(self.source_frame_gap_threshold, bool)
            or not isinstance(self.source_frame_gap_threshold, int)
        ):
            raise TypeError("source_frame_gap_threshold must be an integer")
        if self.source_frame_gap_threshold < 0:
            raise ValueError(
                "source_frame_gap_threshold must be nonnegative"
            )
        for name in (
            "raw_capacity",
            "event_capacity",
            "attempt_cache_size",
            "input_capacity",
        ):
            object.__setattr__(
                self,
                name,
                _positive_integer(name, getattr(self, name)),
            )


@dataclass(frozen=True)
class _ArrivalSnapshot:
    rates: Mapping[str, float]
    ages: Mapping[str, Optional[float]]
    source_frames: Mapping[str, Optional[int]]
    valid_by_subject: Mapping[str, Optional[bool]]
    occluded_by_subject: Mapping[str, Optional[bool]]
    received_any: bool


@dataclass(frozen=True)
class _DecodedInput:
    channel: str
    message: object
    payload_size: int
    received_monotonic_s: float
    wall_time_us: int


@dataclass(frozen=True)
class _ReplayCloseRequest:
    attempt_id: int
    terminal_code: str
    recording_complete: bool


@dataclass(frozen=True)
class _AttemptCloseSignal:
    attempt_id: int
    monotonic_s: float
    wall_time_us: int
    detail: Optional[AttemptDetail] = None


class _AttemptCloseEventTee:
    """Route terminal attempt transitions to a bounded local mailbox."""

    def __init__(self, delegate: object, *, capacity: int) -> None:
        self._delegate = delegate
        self._capacity = int(capacity)
        self._lock = threading.Lock()
        self._close_signals: Deque[_AttemptCloseSignal] = deque()
        self._queued_attempt_ids = set()
        self._dropped_close_signals = 0
        self._overflow_listener: Optional[Callable[[int], None]] = None

    @staticmethod
    def _close_signal(draft: EventDraft) -> Optional[_AttemptCloseSignal]:
        if str(draft.kind) != "attempt_transition":
            return None
        if str(draft.payload.get("stage", "")).upper() != "ATTEMPT_CLOSED":
            return None
        if draft.attempt_id is None:
            return None
        return _AttemptCloseSignal(
            attempt_id=int(draft.attempt_id),
            monotonic_s=float(draft.monotonic_s),
            wall_time_us=int(draft.wall_time_us),
        )

    def enqueue_close(self, signal: _AttemptCloseSignal) -> bool:
        overflow_listener = None
        with self._lock:
            if signal.attempt_id in self._queued_attempt_ids:
                return True
            if len(self._close_signals) >= self._capacity:
                self._dropped_close_signals += 1
                overflow_listener = self._overflow_listener
            else:
                self._close_signals.append(signal)
                self._queued_attempt_ids.add(signal.attempt_id)
                return True
        if overflow_listener is not None:
            overflow_listener(signal.attempt_id)
        return False

    def set_overflow_listener(
        self,
        listener: Callable[[int], None],
    ) -> None:
        self._overflow_listener = listener

    def take_close_signals(
        self,
        *,
        limit: int,
    ) -> Tuple[_AttemptCloseSignal, ...]:
        values = []
        with self._lock:
            while self._close_signals and len(values) < int(limit):
                signal = self._close_signals.popleft()
                self._queued_attempt_ids.discard(signal.attempt_id)
                values.append(signal)
        return tuple(values)

    @property
    def close_pending_count(self) -> int:
        with self._lock:
            return len(self._close_signals)

    def offer(self, draft: EventDraft) -> Any:
        signal = self._close_signal(draft)
        if signal is not None:
            self.enqueue_close(signal)
        offer = getattr(self._delegate, "offer", None)
        return offer(draft) if callable(offer) else self._delegate(draft)

    def publish(self, draft: EventDraft) -> Any:
        return self.offer(draft)

    def drain(self, *args: Any, **kwargs: Any) -> Any:
        return self._delegate.drain(*args, **kwargs)

    def close(self, *args: Any, **kwargs: Any) -> Any:
        return self._delegate.close(*args, **kwargs)

    def __getattr__(self, name: str) -> Any:
        return getattr(self._delegate, name)


class _ArrivalWindow:
    """Bounded, monitor-owned LCM arrival counters."""

    def __init__(self, subjects: Sequence[str]) -> None:
        self._lock = threading.RLock()
        self._times: Dict[str, Deque[float]] = {
            str(subject): deque(maxlen=_SUBJECT_RATE_CAPACITY)
            for subject in subjects
        }
        self._latest: Dict[str, float] = {}
        self._frames: Dict[str, int] = {}
        self._valid: Dict[str, Optional[bool]] = {}
        self._occluded: Dict[str, Optional[bool]] = {}

    def record(
        self,
        *,
        subject: str,
        received_monotonic_s: float,
        source_frame: int,
        valid: Optional[bool] = None,
        occluded: Optional[bool] = None,
    ) -> None:
        name = str(subject)
        timestamp = float(received_monotonic_s)
        with self._lock:
            times = self._times.setdefault(
                name,
                deque(maxlen=_SUBJECT_RATE_CAPACITY),
            )
            times.append(timestamp)
            self._latest[name] = timestamp
            self._frames[name] = int(source_frame)
            self._valid[name] = (
                None if valid is None else bool(valid)
            )
            self._occluded[name] = (
                None if occluded is None else bool(occluded)
            )

    def snapshot(self, now_monotonic_s: float) -> _ArrivalSnapshot:
        now = float(now_monotonic_s)
        with self._lock:
            rates: Dict[str, float] = {}
            ages: Dict[str, Optional[float]] = {}
            frames: Dict[str, Optional[int]] = {}
            valid: Dict[str, Optional[bool]] = {}
            occluded: Dict[str, Optional[bool]] = {}
            names = tuple(
                dict.fromkeys(
                    tuple(self._times)
                    + tuple(self._latest)
                    + tuple(self._frames)
                    + tuple(self._valid)
                    + tuple(self._occluded)
                )
            )
            for name in names:
                times = self._times.setdefault(
                    name,
                    deque(maxlen=_SUBJECT_RATE_CAPACITY),
                )
                cutoff = now - _SUBJECT_RATE_WINDOW_S
                while len(times) > 1 and times[0] < cutoff:
                    times.popleft()
                if len(times) >= 2 and times[-1] > times[0]:
                    rates[name] = float(
                        (len(times) - 1) / (times[-1] - times[0])
                    )
                else:
                    rates[name] = 0.0
                latest = self._latest.get(name)
                ages[name] = (
                    None if latest is None else max(0.0, now - latest)
                )
                frames[name] = self._frames.get(name)
                valid[name] = self._valid.get(name)
                occluded[name] = self._occluded.get(name)
            return _ArrivalSnapshot(
                rates=rates,
                ages=ages,
                source_frames=frames,
                valid_by_subject=valid,
                occluded_by_subject=occluded,
                received_any=bool(self._latest),
            )


class _CountingRawSink:
    def __init__(self, lane: RawRecordLane) -> None:
        self.lane = lane
        self._lock = threading.Lock()
        self._dropped = 0

    def submit(self, sample, **identity):
        result = self.lane.submit(sample, **identity)
        if not result.accepted:
            with self._lock:
                self._dropped += 1
        return result

    @property
    def dropped(self) -> int:
        with self._lock:
            return int(self._dropped)


def _resolved_path(path: Path) -> Path:
    value = Path(path).expanduser()
    if not value.is_absolute():
        value = Path.cwd() / value
    return value.resolve()


def _resolved_options(options: MonitorOptions) -> MonitorOptions:
    return replace(
        options,
        mimic_config=_resolved_path(options.mimic_config),
        control_config=_resolved_path(options.control_config),
        table_calib=_resolved_path(options.table_calib),
        pelvis_calib=_resolved_path(options.pelvis_calib),
        output_dir=_resolved_path(options.output_dir),
    )


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        while True:
            chunk = stream.read(1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def _git_snapshot(repo_root: Path) -> Tuple[Optional[str], Optional[bool]]:
    def run(*arguments: str) -> Optional[str]:
        try:
            completed = subprocess.run(
                ("git",) + arguments,
                cwd=str(repo_root),
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                timeout=2.0,
                check=False,
            )
        except (OSError, subprocess.SubprocessError):
            return None
        if completed.returncode != 0:
            return None
        return completed.stdout.strip()

    commit = run("rev-parse", "HEAD")
    status = run("status", "--porcelain=v1", "--untracked-files=normal")
    return commit or None, None if status is None else bool(status)


def _package_versions() -> Mapping[str, Optional[str]]:
    versions: Dict[str, Optional[str]] = {}
    for package in _PACKAGE_NAMES:
        try:
            versions[package] = importlib_metadata.version(package)
        except importlib_metadata.PackageNotFoundError:
            versions[package] = None
    return versions


def _config_mapping(path: Path) -> Mapping[str, Any]:
    loaded = OmegaConf.load(str(path))
    value = OmegaConf.to_container(loaded, resolve=True)
    if not isinstance(value, Mapping):
        raise TypeError("{} must contain a mapping".format(path))
    return value


def _session_metadata(
    *,
    options: MonitorOptions,
    mimic_config: Mapping[str, Any],
    control_config: Mapping[str, Any],
    settings: HitterRuntimeSettings,
    estimator_window_size: int,
    argv: Sequence[str],
) -> Mapping[str, Any]:
    commit, dirty = _git_snapshot(_REPO_ROOT)
    packages = _package_versions()
    config_hashes = {
        "mimic": _sha256_file(options.mimic_config),
        "control": _sha256_file(options.control_config),
    }
    calibration_hashes = {
        "table": _sha256_file(options.table_calib),
        "pelvis": _sha256_file(options.pelvis_calib),
    }
    resolved_runtime = {
        "estimator_window_size": int(estimator_window_size),
        "estimator_sample_rate_hz": settings.estimator_sample_rate_hz,
        "planner_update_rate_hz": settings.planner_update_rate_hz,
        "incoming_confirmation_snapshots": (
            settings.incoming_confirmation_snapshots
        ),
        "control_rate_hz": 1.0 / settings.control_tick_s,
        "control_tick_s": settings.control_tick_s,
        "duration_s": options.duration_s,
        "reacquire_grace_s": options.reacquire_grace_s,
        "heartbeat_stale_s": options.heartbeat_stale_s,
        "pelvis_stale_s": options.pelvis_stale_s,
        "source_frame_gap_threshold": options.source_frame_gap_threshold,
        "input_capacity": options.input_capacity,
        "raw_capacity": options.raw_capacity,
        "event_capacity": options.event_capacity,
        "attempt_cache_size": options.attempt_cache_size,
    }
    return {
        "schema_version": SCHEMA_VERSION,
        "monitor_version": 1,
        "config_name": options.mimic_config.stem,
        "lcm_url": options.lcm_url,
        "channel": options.channel,
        "base_name": options.base_name,
        "ball_name": options.ball_name,
        "argv": tuple(str(item) for item in argv),
        "output_dir": str(options.output_dir),
        "config_sha256": config_hashes,
        "calibration_sha256": calibration_hashes,
        "files": {
            "mimic_config": {
                "basename": options.mimic_config.name,
                "sha256": config_hashes["mimic"],
            },
            "control_config": {
                "basename": options.control_config.name,
                "sha256": config_hashes["control"],
            },
            "table_calibration": {
                "basename": options.table_calib.name,
                "sha256": calibration_hashes["table"],
            },
            "pelvis_calibration": {
                "basename": options.pelvis_calib.name,
                "sha256": calibration_hashes["pelvis"],
            },
        },
        "git_commit": commit,
        "git_dirty": dirty,
        "git": {"commit": commit, "dirty": dirty},
        "python_version": platform.python_version(),
        "package_versions": packages,
        "versions": {
            "python": platform.python_version(),
            "packages": packages,
        },
        "resolved_runtime": resolved_runtime,
        "resolved_config": {
            "mimic": mimic_config,
            "control": control_config,
        },
    }


def _initial_state(
    *,
    config_name: str,
    session_basename: str,
    subjects: Sequence[str],
) -> DiagnosticState:
    rates = {subject: 0.0 for subject in subjects}
    ages = {subject: None for subject in subjects}
    frames = {subject: None for subject in subjects}
    return DiagnosticState(
        schema_version=LIVE_SNAPSHOT_SCHEMA_VERSION,
        watermark_event_id=0,
        health=HealthSnapshot(
            lcm_connected=False,
            message_rate_hz_by_subject=rates,
            message_age_s_by_subject=ages,
            source_frame_by_subject=frames,
            pelvis_valid=False,
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
            config_name=config_name,
            session_basename=session_basename,
            warnings=(),
        ),
        lifecycle=LifecycleSnapshot(
            phase="waiting",
            last_decision="none",
            active_key=None,
            cached_key=None,
            lifecycle_now_s=None,
            obs_now_s=None,
        ),
        current_attempt=None,
        recent_attempts=(),
    )


def _snapshot_key(binding: object) -> Optional[SnapshotKey]:
    if binding is None:
        return None
    value = getattr(binding, "snapshot_key", None)
    return value if isinstance(value, SnapshotKey) else None


def _transition_payload(transition: AttemptTransition) -> Mapping[str, Any]:
    return {
        "track_segment_id": transition.track_segment_id,
        "stage": transition.stage,
        "reason_code": transition.reason_code,
        "snapshot_key": (
            None
            if transition.snapshot_key is None
            else transition.snapshot_key.to_json_dict()
        ),
        "values": transition.values,
    }


class HitterTaskMonitor:
    """Independent, read-only diagnostic process composition."""

    def __init__(
        self,
        options: MonitorOptions,
        *,
        lcm_client: object,
        decoder: Callable[[bytes], object],
        adapter: object,
        pipeline: object,
        recorder: object,
        web_server: object,
        event_lane: object,
        event_hub: object,
        attempt_repository: object,
        session_basename: str,
        control_tick_s: float,
        estimator_window_size: int,
        session_paths: Optional[SessionPaths] = None,
        runtime_settings: Optional[HitterRuntimeSettings] = None,
        raw_counter: Optional[_CountingRawSink] = None,
        replay_controller: Optional[object] = None,
        replay_capture: Optional[ReplayCaptureStore] = None,
        replay_planner_config: Optional[Mapping[str, Any]] = None,
        replay_forced_strike_type: Optional[str] = None,
        monotonic_fn: Callable[[], float] = time.monotonic,
        wall_time_fn: Callable[[], float] = time.time,
        select_fn: Callable[..., object] = select.select,
    ) -> None:
        self.options = options
        self.lcm_client = lcm_client
        self.decoder = decoder
        self.adapter = adapter
        self.pipeline = pipeline
        self.recorder = recorder
        self.web_server = web_server
        self.event_lane = (
            event_lane
            if isinstance(event_lane, _AttemptCloseEventTee)
            else _AttemptCloseEventTee(
                event_lane,
                capacity=max(1, int(options.attempt_cache_size)),
            )
        )
        try:
            self.pipeline.event_sink = self.event_lane
        except Exception:
            pass
        self.event_hub = event_hub
        self.attempt_repository = attempt_repository
        self.session_basename = str(session_basename)
        self.session_paths = (
            session_paths
            if session_paths is not None
            else SimpleNamespace(root=Path(self.session_basename))
        )
        self.control_tick_s = float(control_tick_s)
        if (
            not math.isfinite(self.control_tick_s)
            or self.control_tick_s <= 0.0
        ):
            raise ValueError("control_tick_s must be finite and positive")
        self.estimator_window_size = int(estimator_window_size)
        if self.estimator_window_size <= 0:
            raise ValueError("estimator_window_size must be positive")
        self.runtime_settings = (
            runtime_settings
            if runtime_settings is not None
            else getattr(pipeline, "settings", None)
        )
        self.raw_counter = raw_counter
        self.replay_controller = replay_controller
        self.replay_capture = replay_capture
        self.replay_planner_config = (
            {}
            if replay_planner_config is None
            else dict(replay_planner_config)
        )
        self.replay_forced_strike_type = replay_forced_strike_type
        self._monotonic_fn = monotonic_fn
        self._wall_time_fn = wall_time_fn
        self._select_fn = select_fn
        self._arrival = _ArrivalWindow(
            ("ball", "g1pelvis", "table")
        )
        self._state_lock = threading.RLock()
        self._lifecycle_lock = threading.RLock()
        # Lock order: capture -> tick/pipeline -> monitor state. Event and
        # recorder publication must happen after the capture lock is released.
        self._snapshot_capture_lock = threading.RLock()
        self._tick_lock = threading.Lock()
        self._attempt_projection_lock = threading.RLock()
        self._attempt_close_drain_lock = threading.Lock()
        self._close_lock = threading.RLock()
        self._stop_event = threading.Event()
        self._input_condition = threading.Condition(threading.Lock())
        self._input_queue: Deque[_DecodedInput] = deque()
        self._input_inflight = False
        self._input_accepting = True
        self._input_stop_requested = False
        self._input_samples_dropped = 0
        self._recording_global_incomplete = False
        self._recording_incomplete_attempt_ids = set()
        self._started = False
        self._closed = False
        self._close_result = True
        self._subscription = None
        self._deadline_s: Optional[float] = None
        self._failure: Optional[str] = None
        self._warnings: Deque[str] = deque(maxlen=64)
        self._last_adapter_warnings: Tuple[str, ...] = ()
        self._live_snapshot_revision = 0
        self._last_planner_input_received_s: Optional[float] = None
        self._last_tick_result = None
        self._recorder_status = RecorderStatus(
            healthy=True,
            recording_complete=True,
            bytes_written=0,
            last_error=None,
        )
        self._closed_attempt_ids: "OrderedDict[int, None]" = OrderedDict()
        self._attempt_observations: Dict[int, Dict[str, Any]] = {}
        self._replay_attempt_active = False
        self._replay_grace_active = False
        self._replay_state_version = 0
        self._replay_close_requests: Deque[_ReplayCloseRequest] = deque()
        self._replay_request_capacity = max(
            1,
            int(self.options.attempt_cache_size),
        )
        self._lcm_thread: Optional[threading.Thread] = None
        self._input_thread: Optional[threading.Thread] = None
        self._tick_thread: Optional[threading.Thread] = None
        self._recorder_thread: Optional[threading.Thread] = None
        self._replay_thread: Optional[threading.Thread] = None
        self.event_lane.set_overflow_listener(
            self._attempt_close_queue_overflow
        )

    @property
    def address(self) -> Tuple[str, int]:
        return self.web_server.address

    @property
    def page_url(self) -> str:
        host, port = self.address
        display_host = "127.0.0.1" if host in ("127.0.0.1", "::1") else host
        return "http://{}:{}/".format(display_host, port)

    @property
    def background_threads_alive(self) -> bool:
        return any(
            thread is not None and thread.is_alive()
            for thread in (
                self._lcm_thread,
                self._input_thread,
                self._tick_thread,
                self._recorder_thread,
                self._replay_thread,
            )
        )

    @property
    def input_pending_count(self) -> int:
        with self._input_condition:
            return len(self._input_queue) + int(self._input_inflight)

    @property
    def input_samples_dropped(self) -> int:
        with self._input_condition:
            return int(self._input_samples_dropped)

    @property
    def input_recording_complete(self) -> bool:
        return self.input_samples_dropped == 0

    def _add_warning(self, value: str) -> None:
        warning = str(value)[:256]
        with self._state_lock:
            if warning not in self._warnings:
                self._warnings.append(warning)

    def _attempt_close_queue_overflow(self, attempt_id: int) -> None:
        self._add_warning("ATTEMPT_CLOSE_QUEUE_FULL")
        self._mark_recording_incomplete(
            int(attempt_id),
            "ATTEMPT_CLOSE_QUEUE_FULL",
        )

    def _current_attempt_id(self) -> Optional[int]:
        try:
            summary = self.pipeline.attempt_tracker.current_summary()
        except Exception:
            return None
        value = getattr(summary, "attempt_id", None)
        if (
            isinstance(value, int)
            and not isinstance(value, bool)
            and value > 0
        ):
            return int(value)
        return None

    def _attempt_recording_complete(self, attempt_id: int) -> bool:
        with self._state_lock:
            return bool(
                not self._recording_global_incomplete
                and int(attempt_id)
                not in self._recording_incomplete_attempt_ids
            )

    def _cache_recorder_status(self, status: object) -> RecorderStatus:
        cached = RecorderStatus(
            healthy=bool(getattr(status, "healthy", False)),
            recording_complete=bool(
                getattr(status, "recording_complete", False)
            ),
            bytes_written=int(getattr(status, "bytes_written", 0)),
            last_error=getattr(status, "last_error", None),
        )
        with self._state_lock:
            self._recorder_status = cached
        return cached

    def _cached_recorder_status(self) -> RecorderStatus:
        with self._state_lock:
            return self._recorder_status

    @staticmethod
    def _inconclusive_detail(detail: AttemptDetail) -> AttemptDetail:
        return replace(
            detail,
            summary=replace(
                detail.summary,
                ab_summary="INCONCLUSIVE",
                recording_complete=False,
            ),
        )

    def _mark_recording_incomplete(
        self,
        attempt_id: Optional[int],
        reason: str,
    ) -> None:
        if attempt_id is not None:
            attempt_id = int(attempt_id)
        with self._state_lock:
            if attempt_id is None:
                self._recording_global_incomplete = True
                self._recording_incomplete_attempt_ids.clear()
            elif not self._recording_global_incomplete:
                if (
                    len(self._recording_incomplete_attempt_ids)
                    >= self.options.attempt_cache_size
                ):
                    self._recording_global_incomplete = True
                    self._recording_incomplete_attempt_ids.clear()
                else:
                    self._recording_incomplete_attempt_ids.add(attempt_id)
            self._recorder_status = replace(
                self._recorder_status,
                recording_complete=False,
            )
        marker = getattr(self.recorder, "mark_incomplete", None)
        if callable(marker):
            try:
                marker(attempt_id, reason)
            except Exception as exc:
                self._add_warning(
                    "RECORDER_MARK_INCOMPLETE_ERROR:{}:{}".format(
                        type(exc).__name__,
                        str(exc),
                    )
                )
        capture_marker = getattr(
            self.replay_capture,
            "mark_incomplete",
            None,
        )
        if callable(capture_marker):
            try:
                capture_marker(attempt_id)
            except Exception as exc:
                self._add_warning(
                    "REPLAY_MARK_INCOMPLETE_ERROR:{}:{}".format(
                        type(exc).__name__,
                        str(exc),
                    )
                )
    def _set_failure(self, exc: BaseException) -> None:
        with self._state_lock:
            if self._failure is None:
                self._failure = "{}: {}".format(
                    type(exc).__name__,
                    str(exc),
                )[:2048]
        self._stop_event.set()

    def _failure_snapshot(self) -> Optional[str]:
        with self._state_lock:
            return self._failure

    def _duration_expired(self) -> bool:
        deadline = self._deadline_s
        return deadline is not None and self._monotonic_fn() >= deadline

    def start(self) -> None:
        """Start recorder, web, LCM owner and 50 Hz loop in that order."""
        with self._lifecycle_lock:
            if self._closed:
                raise RuntimeError("monitor is closed")
            if self._started:
                return
            self._started = True
            if self.options.duration_s > 0.0:
                self._deadline_s = (
                    self._monotonic_fn() + self.options.duration_s
                )
            try:
                self.recorder.start()
                status = self._cache_recorder_status(
                    self.recorder.status()
                )
                if not bool(status.healthy):
                    raise RuntimeError(
                        status.last_error or "recorder failed to start"
                    )
                self._recorder_thread = threading.Thread(
                    target=self._recorder_loop,
                    name="hitter-task-recorder",
                    daemon=True,
                )
                self._recorder_thread.start()

                self.web_server.start()

                self._input_thread = threading.Thread(
                    target=self._input_loop,
                    name="hitter-task-input-processor",
                    daemon=True,
                )
                self._input_thread.start()

                self._subscription = self.lcm_client.subscribe(
                    self.options.channel,
                    self._handle_lcm,
                )
                self._lcm_thread = threading.Thread(
                    target=self._lcm_loop,
                    name="hitter-task-lcm-owner",
                    daemon=True,
                )
                self._lcm_thread.start()

                if self.replay_controller is not None:
                    self._replay_thread = threading.Thread(
                        target=self._replay_supervisor_loop,
                        name="hitter-task-replay-supervisor",
                        daemon=True,
                    )
                    self._replay_thread.start()

                self._tick_thread = threading.Thread(
                    target=self._tick_loop,
                    name="hitter-task-50hz",
                    daemon=True,
                )
                self._tick_thread.start()
            except BaseException as exc:
                self._set_failure(exc)
                self.close(timeout_s=5.0)
                raise

    def run(self) -> int:
        """Run until duration, signal or failure; no control dependency."""
        previous_handlers: Dict[int, object] = {}

        def stop_for_signal(_signum, _frame) -> None:
            self._stop_event.set()

        if threading.current_thread() is threading.main_thread():
            for number in (signal.SIGINT, signal.SIGTERM):
                previous_handlers[number] = signal.getsignal(number)
                signal.signal(number, stop_for_signal)
        exit_code = 0
        try:
            self.start()
            while not self._stop_event.is_set():
                if self._failure_snapshot() is not None:
                    exit_code = 1
                    break
                if self._duration_expired():
                    self._stop_event.set()
                    break
                self._stop_event.wait(_RUN_POLL_S)
            if self._failure_snapshot() is not None:
                exit_code = 1
        finally:
            closed_cleanly = self.close(timeout_s=5.0)
            if not closed_cleanly:
                exit_code = 1
            for number, handler in previous_handlers.items():
                signal.signal(number, handler)
        return exit_code

    def _handle_lcm(self, channel: str, payload: bytes) -> None:
        received_monotonic_s = self._monotonic_fn()
        wall_time_us = int(self._wall_time_fn() * 1.0e6)
        try:
            message = self.decoder(payload)
        except Exception as exc:
            self._add_warning(
                "LCM_DECODE_ERROR:{}:{}".format(
                    type(exc).__name__,
                    str(exc),
                )
            )
            return
        item = _DecodedInput(
            channel=str(channel),
            message=message,
            payload_size=len(payload),
            received_monotonic_s=received_monotonic_s,
            wall_time_us=wall_time_us,
        )
        dropped = False
        with self._input_condition:
            if (
                not self._input_accepting
                or len(self._input_queue) >= self.options.input_capacity
            ):
                self._input_samples_dropped += 1
                dropped = True
            else:
                self._input_queue.append(item)
                self._input_condition.notify()
        if dropped:
            self._add_warning("INPUT_QUEUE_FULL_RECORDING_INCOMPLETE")
            self._mark_recording_incomplete(
                self._current_attempt_id(),
                "INPUT_SAMPLES_DROPPED",
            )

    def _process_input(self, item: _DecodedInput) -> None:
        try:
            with self._snapshot_capture_lock:
                message = item.message
                raw_name = str(
                    getattr(message, "name", "") or ""
                ).strip()
                if raw_name.casefold() == self.options.base_name.casefold():
                    setattr(message, "name", "g1pelvis")
                elif (
                    raw_name.casefold()
                    == self.options.ball_name.casefold()
                ):
                    setattr(message, "name", "ball")
                output = self.adapter.ingest_decoded(
                    channel=item.channel,
                    message=message,
                    payload_size=item.payload_size,
                    received_monotonic_s=item.received_monotonic_s,
                    wall_time_us=item.wall_time_us,
                )
                self.pipeline.ingest_adapter_output(output)
                sample = output.sample
                self._arrival.record(
                    subject=sample.subject,
                    received_monotonic_s=sample.received_monotonic_s,
                    source_frame=sample.source_frame,
                    valid=getattr(sample, "valid", None),
                    occluded=getattr(sample, "occluded", None),
                )
                with self._state_lock:
                    self._last_adapter_warnings = tuple(output.warnings)
                self._remember_planner_input(output)
        except Exception as exc:
            self._add_warning(
                "LCM_INPUT_ERROR:{}:{}".format(
                    type(exc).__name__,
                    str(exc),
                )
            )
            return

    def _input_loop(self) -> None:
        while True:
            with self._input_condition:
                while (
                    not self._input_queue
                    and not self._input_stop_requested
                ):
                    self._input_condition.wait()
                if self._input_queue:
                    item = self._input_queue.popleft()
                    self._input_inflight = True
                elif self._input_stop_requested:
                    return
                else:
                    continue
            try:
                self._process_input(item)
            finally:
                with self._input_condition:
                    self._input_inflight = False
                    self._input_condition.notify_all()

    def _remember_planner_input(self, output: object) -> None:
        snapshot = getattr(output, "snapshot", None)
        if snapshot is None:
            return
        received = float(snapshot.received_monotonic_s)
        with self._state_lock:
            last = self._last_planner_input_received_s
            interval = float(
                getattr(
                    self.runtime_settings,
                    "planner_update_interval_s",
                    0.01,
                )
            )
            if (
                last is not None
                and received - last < interval - 1.0e-12
            ):
                return
            self._last_planner_input_received_s = received
        key = SnapshotKey(
            int(snapshot.track_id),
            int(snapshot.generation),
        )
        binding = self.pipeline.attempt_tracker.binding_for_result(key)
        if binding is None:
            return
        value = {
            "snapshot_key": key.to_json_dict(),
            "source_frame": int(snapshot.source_frame),
            "source_time_s": float(snapshot.source_time_s),
            "received_monotonic_s": received,
            "position_w": np.asarray(
                snapshot.position_w,
                dtype=np.float64,
            ),
            "velocity_w": np.asarray(
                snapshot.velocity_w,
                dtype=np.float64,
            ),
            "base_position_w": np.asarray(
                snapshot.base_position_w,
                dtype=np.float64,
            ),
            "base_quaternion_xyzw": np.asarray(
                snapshot.base_quaternion_xyzw,
                dtype=np.float64,
            ),
            "base_valid": bool(snapshot.base_valid),
            "visible": bool(snapshot.visible),
            "ready": bool(snapshot.ready),
        }
        with self._state_lock:
            if int(binding.attempt_id) in self._closed_attempt_ids:
                return
            captured = self._attempt_observations.setdefault(
                int(binding.attempt_id),
                {},
            )
            inputs = captured.setdefault("planner_inputs", [])
            if len(inputs) < 8192:
                inputs.append(value)
            else:
                self._add_warning("PLANNER_INPUT_DETAIL_LIMIT")

    def _lcm_loop(self) -> None:
        try:
            descriptor = self.lcm_client.fileno()
            while not self._stop_event.is_set():
                if self._duration_expired():
                    self._stop_event.set()
                    break
                readable, _writable, _exceptional = self._select_fn(
                    [descriptor],
                    [],
                    [],
                    _LCM_SELECT_TIMEOUT_S,
                )
                if self._stop_event.is_set():
                    break
                if self._duration_expired():
                    self._stop_event.set()
                    break
                if readable:
                    self.lcm_client.handle()
        except BaseException as exc:
            if not self._stop_event.is_set():
                self._set_failure(exc)

    def _tick_loop(self) -> None:
        next_tick = time.monotonic()
        try:
            while not self._stop_event.is_set():
                now = time.monotonic()
                if now < next_tick:
                    if self._stop_event.wait(next_tick - now):
                        break
                if self._stop_event.is_set():
                    break
                self._tick_once()
                next_tick += self.control_tick_s
                after = time.monotonic()
                if next_tick < after - self.control_tick_s:
                    next_tick = after
        except BaseException as exc:
            if not self._stop_event.is_set():
                self._set_failure(exc)

    def _recorder_loop(self) -> None:
        while True:
            self._drain_attempt_close_requests()
            try:
                self._cache_recorder_status(
                    self.recorder.drain(0.0)
                )
            except Exception as exc:
                self._add_warning(
                    "RECORDER_DRAIN_ERROR:{}:{}".format(
                        type(exc).__name__,
                        str(exc),
                    )
                )
            if (
                self._stop_event.is_set()
                and self.event_lane.close_pending_count == 0
            ):
                return
            self._stop_event.wait(_RECORDER_POLL_S)

    def _remember_closed_attempt(self, attempt_id: int) -> bool:
        with self._state_lock:
            if attempt_id in self._closed_attempt_ids:
                self._closed_attempt_ids.move_to_end(attempt_id)
                return False
            self._closed_attempt_ids[attempt_id] = None
            while (
                len(self._closed_attempt_ids)
                > self.options.attempt_cache_size
            ):
                self._closed_attempt_ids.popitem(last=False)
            return True

    def _publish_attempt_detail(
        self,
        detail: AttemptDetail,
        *,
        recorder_rejected_reason: str,
        recorder_error_reason: str,
        monotonic_s: Optional[float] = None,
        wall_time_us: Optional[int] = None,
    ) -> AttemptDetail:
        if not self._attempt_recording_complete(detail.attempt_id):
            detail = self._inconclusive_detail(detail)
        rejected_reason = None
        try:
            if not bool(self.recorder.offer_attempt_detail(detail)):
                rejected_reason = recorder_rejected_reason
        except Exception as exc:
            rejected_reason = recorder_error_reason
            self._add_warning(
                "{}:{}:{}".format(
                    recorder_error_reason,
                    type(exc).__name__,
                    str(exc),
                )
            )
        if rejected_reason is not None:
            self._mark_recording_incomplete(
                detail.attempt_id,
                rejected_reason,
            )
            detail = self._inconclusive_detail(detail)
        try:
            self.attempt_repository.commit(detail)
        except Exception as exc:
            self._add_warning(
                "ATTEMPT_REPOSITORY_ERROR:{}:{}".format(
                    type(exc).__name__,
                    str(exc),
                )
            )
            self._mark_recording_incomplete(
                detail.attempt_id,
                "ATTEMPT_REPOSITORY_ERROR",
            )
            detail = self._inconclusive_detail(detail)
            try:
                self.recorder.offer_attempt_detail(detail)
            except Exception:
                pass
            try:
                self.attempt_repository.commit(detail)
            except Exception as retry_exc:
                self._add_warning(
                    "ATTEMPT_REPOSITORY_FAIL_CLOSED_ERROR:{}:{}".format(
                        type(retry_exc).__name__,
                        str(retry_exc),
                    )
                )
        self._offer(
            EventDraft(
                kind="ATTEMPT_CLOSED",
                monotonic_s=(
                    self._monotonic_fn()
                    if monotonic_s is None
                    else float(monotonic_s)
                ),
                wall_time_us=(
                    int(self._wall_time_fn() * 1.0e6)
                    if wall_time_us is None
                    else int(wall_time_us)
                ),
                scope="attempt",
                attempt_id=detail.attempt_id,
                payload={"attempt": detail.summary.to_json_dict()},
            )
        )
        return detail

    def _detail_for_close_signal(
        self,
        signal: _AttemptCloseSignal,
    ) -> Optional[AttemptDetail]:
        if signal.detail is not None:
            return signal.detail
        summary = self.pipeline.attempt_tracker.summary_for_attempt(
            signal.attempt_id
        )
        if summary is None:
            return None
        with self._state_lock:
            captured = self._attempt_observations.get(
                signal.attempt_id,
                {},
            )
            previous_projection = captured.get("projected_summary")
        projected = (
            previous_projection
            if isinstance(previous_projection, AttemptSummary)
            else summary
        )
        projected = replace(
            projected,
            status=summary.status,
            stage="ATTEMPT_CLOSED",
            primary_blocker=summary.primary_blocker,
            task_obs_status=summary.task_obs_status,
            ab_summary=(
                "INCONCLUSIVE"
                if (
                    self.replay_controller is not None
                    and self.replay_capture is not None
                )
                else projected.ab_summary
            ),
            recording_complete=bool(
                projected.recording_complete
                and summary.recording_complete
                and self.input_recording_complete
                and self._attempt_recording_complete(signal.attempt_id)
            ),
        )
        return self._attempt_detail(summary=projected)

    def _drain_attempt_close_requests(
        self,
        *,
        deadline_s: Optional[float] = None,
    ) -> None:
        if not self._acquire_before(
            self._attempt_close_drain_lock,
            deadline_s=deadline_s,
        ):
            return
        try:
            for _index in range(self.options.attempt_cache_size):
                if (
                    deadline_s is not None
                    and time.monotonic() >= deadline_s
                ):
                    return
                if not self._acquire_before(
                    self._attempt_projection_lock,
                    deadline_s=deadline_s,
                ):
                    return
                try:
                    signals = self.event_lane.take_close_signals(limit=1)
                    if not signals:
                        return
                    signal = signals[0]
                    if not self._remember_closed_attempt(
                        signal.attempt_id
                    ):
                        continue
                    detail = self._detail_for_close_signal(signal)
                    if detail is None:
                        with self._state_lock:
                            self._closed_attempt_ids.pop(
                                signal.attempt_id,
                                None,
                            )
                finally:
                    self._attempt_projection_lock.release()
                if detail is None:
                    self._add_warning(
                        "ATTEMPT_CLOSE_DETAIL_MISSING:{}".format(
                            signal.attempt_id
                        )
                    )
                    continue
                persisted = self._publish_attempt_detail(
                    detail,
                    recorder_rejected_reason=(
                        "ATTEMPT_DETAIL_RECORDER_REJECTED"
                    ),
                    recorder_error_reason=(
                        "ATTEMPT_DETAIL_RECORDER_ERROR"
                    ),
                    monotonic_s=signal.monotonic_s,
                    wall_time_us=signal.wall_time_us,
                )
                self._queue_replay_close(summary=persisted.summary)
        finally:
            self._attempt_close_drain_lock.release()

    def _replay_supervisor_loop(self) -> None:
        replay = self.replay_controller
        if replay is None:
            return
        applied_version = -1
        while True:
            with self._state_lock:
                version = self._replay_state_version
                attempt_active = self._replay_attempt_active
                grace_active = self._replay_grace_active
                has_requests = bool(self._replay_close_requests)
            try:
                if version != applied_version:
                    replay.set_realtime_busy(
                        attempt_active=attempt_active,
                        reacquire_grace_active=grace_active,
                    )
                    applied_version = version
                self._submit_replay_close_requests()
                for reply in replay.poll():
                    if reply.status in ("COMPLETED", "FAILED"):
                        self._apply_replay_reply(reply)
            except Exception as exc:
                self._add_warning(
                    "REPLAY_SUPERVISOR_ERROR:{}:{}".format(
                        type(exc).__name__,
                        str(exc),
                    )
                )
            with self._state_lock:
                has_requests = bool(self._replay_close_requests)
            if self._stop_event.is_set() and not has_requests:
                return
            self._stop_event.wait(_RECORDER_POLL_S)

    @staticmethod
    def _replay_terminal_code(summary: AttemptSummary) -> str:
        if str(summary.task_obs_status).upper() == "PASS":
            return "TASK_OBS_PASS"
        blocker = (
            ""
            if summary.primary_blocker is None
            else str(summary.primary_blocker).upper()
        )
        if blocker in (
            "LATE_SKIP",
            "TRACK_ENDED_BEFORE_READY",
            "TRACK_ENDED_BEFORE_CONFIRMATION",
            "NO_VALID_PLAN",
            "PELVIS_UNAVAILABLE",
            "MALFORMED_COMMAND",
        ):
            return blocker
        if blocker == "OBS_COMMAND_MISMATCH":
            return "MALFORMED_COMMAND"
        return "NO_VALID_PLAN"

    def _queue_replay_close(
        self,
        *,
        summary: AttemptSummary,
    ) -> bool:
        capture = self.replay_capture
        if self.replay_controller is None or capture is None:
            return False
        request = _ReplayCloseRequest(
            attempt_id=int(summary.attempt_id),
            terminal_code=self._replay_terminal_code(summary),
            recording_complete=bool(summary.recording_complete),
        )
        with self._state_lock:
            if (
                len(self._replay_close_requests)
                >= self._replay_request_capacity
            ):
                capture.mark_incomplete(summary.attempt_id)
                self._add_warning("REPLAY_CLOSE_QUEUE_FULL")
                return False
            self._replay_close_requests.append(request)
        return True

    def _submit_replay_close_requests(self) -> None:
        replay = self.replay_controller
        capture = self.replay_capture
        if replay is None or capture is None:
            return
        while True:
            with self._state_lock:
                if not self._replay_close_requests:
                    return
                request = self._replay_close_requests.popleft()
            ready = getattr(capture, "ready_to_finalize", None)
            if (
                callable(ready)
                and not self._stop_event.is_set()
                and not bool(ready(request.attempt_id))
            ):
                with self._state_lock:
                    self._replay_close_requests.appendleft(request)
                return
            path = (
                Path(self.session_paths.root)
                / "replay_inputs"
                / "attempt-{}.json".format(request.attempt_id)
            )
            try:
                bundle = capture.finalize_attempt(
                    attempt_id=request.attempt_id,
                    terminal_code=request.terminal_code,
                    recording_complete=request.recording_complete,
                    planner_config=self.replay_planner_config,
                    runtime_settings=self.runtime_settings,
                    forced_strike_type=self.replay_forced_strike_type,
                )
                write_replay_input_bundle(path, bundle)
                if not bundle.recording_complete:
                    self._mark_replay_detail_incomplete(
                        request.attempt_id
                    )
                replay.submit_disk_job(
                    ReplayJobRef(
                        self.session_basename,
                        request.attempt_id,
                        path,
                    )
                )
            except Exception as exc:
                error = "{}: {}".format(
                    type(exc).__name__,
                    str(exc),
                )[:2048]
                offer_status = getattr(
                    self.recorder,
                    "offer_replay_job_status",
                    None,
                )
                if callable(offer_status):
                    try:
                        offer_status(
                            {
                                "session_basename": self.session_basename,
                                "attempt_id": request.attempt_id,
                                "input_path": str(path),
                                "status": "FAILED",
                                "error": error,
                            }
                        )
                    except Exception:
                        self._add_warning(
                            "REPLAY_FAILED_STATUS_REJECTED"
                        )
                self._apply_replay_reply(
                    ReplayWorkerReply(
                        attempt_id=request.attempt_id,
                        status="FAILED",
                        result=None,
                        error=error,
                    )
                )

    def _publish_replay_detail(self, detail: AttemptDetail) -> None:
        self._publish_attempt_detail(
            detail,
            recorder_rejected_reason="REPLAY_DETAIL_RECORDER_REJECTED",
            recorder_error_reason="REPLAY_DETAIL_RECORDER_ERROR",
        )

    def _mark_replay_detail_incomplete(self, attempt_id: int) -> None:
        detail = self.attempt_repository.get(attempt_id)
        if detail is None:
            return
        updated = replace(
            detail,
            summary=replace(
                detail.summary,
                ab_summary="INCONCLUSIVE",
                recording_complete=False,
            ),
        )
        self._publish_replay_detail(updated)

    def _apply_replay_reply(self, reply: ReplayWorkerReply) -> None:
        detail = self.attempt_repository.get(reply.attempt_id)
        if detail is None:
            self._add_warning(
                "REPLAY_DETAIL_MISSING:{}".format(reply.attempt_id)
            )
            return
        if reply.status == "FAILED":
            persistence_failed = bool(
                reply.error
                and "REPLAY_PERSISTENCE_ERROR" in str(reply.error)
            )
            if persistence_failed:
                self._mark_recording_incomplete(
                    reply.attempt_id,
                    "REPLAY_PERSISTENCE_ERROR",
                )
            updated = replace(
                detail,
                summary=replace(
                    detail.summary,
                    ab_summary="INCONCLUSIVE",
                    recording_complete=bool(
                        detail.summary.recording_complete
                        and not persistence_failed
                    ),
                ),
                variant_outcomes=(
                    {
                        "variant": "replay",
                        "status": "FAILED",
                        "error": reply.error,
                    },
                ),
                ab_deltas={},
            )
            self._publish_replay_detail(updated)
            return
        result = reply.result
        if reply.status != "COMPLETED" or not isinstance(result, Mapping):
            return
        baseline = result.get("baseline")
        one_frame = result.get("one_frame")
        parity = result.get("parity")
        deltas = result.get("deltas", {})
        if not isinstance(baseline, Mapping):
            baseline = {}
        if one_frame is not None and not isinstance(one_frame, Mapping):
            one_frame = None
        if not isinstance(parity, Mapping):
            parity = {}
        if not isinstance(deltas, Mapping):
            deltas = {}
        complete = bool(
            detail.summary.recording_complete
            and baseline.get("recording_complete", False)
            and (
                one_frame is None
                or one_frame.get("recording_complete", False)
            )
        )
        parity_matches = bool(parity.get("matches", False))
        label = str(result.get("summary_label", "INCONCLUSIVE"))
        if (
            not complete
            or not parity_matches
            or label not in (
                "SAME_PASS",
                "SAVED_BY_ONE_FRAME",
                "BASELINE_ONLY_PASS",
                "BOTH_FAIL_SAME",
                "BOTH_FAIL_DIFFERENT",
                "BOTH_PASS_DIFFERENT_COMMAND",
            )
        ):
            label = "INCONCLUSIVE"
        outcomes = []
        if baseline:
            outcomes.append(dict(baseline))
        if one_frame is not None:
            outcomes.append(dict(one_frame))
        updated = replace(
            detail,
            summary=replace(
                detail.summary,
                ab_summary=label,
                recording_complete=complete,
            ),
            variant_outcomes=tuple(outcomes),
            ab_deltas=dict(deltas),
        )
        self._publish_replay_detail(updated)

    def _offer(self, draft: EventDraft) -> bool:
        try:
            accepted = bool(self.event_lane.offer(draft))
        except Exception as exc:
            self._add_warning(
                "EVENT_LANE_ERROR:{}:{}".format(
                    type(exc).__name__,
                    str(exc),
                )
            )
            self._mark_recording_incomplete(
                (
                    draft.attempt_id
                    if draft.attempt_id is not None
                    else self._current_attempt_id()
                ),
                "DIAGNOSTIC_EVENT_DROPPED",
            )
            return False
        if not accepted:
            self._add_warning("DIAGNOSTIC_EVENT_QUEUE_FULL")
            self._mark_recording_incomplete(
                (
                    draft.attempt_id
                    if draft.attempt_id is not None
                    else self._current_attempt_id()
                ),
                "DIAGNOSTIC_EVENT_DROPPED",
            )
        return accepted

    def _ball_diagnostic_snapshot(
        self,
        *,
        obs_now_s: float,
        incoming: object,
    ) -> BallDiagnosticState:
        tracker = getattr(self.pipeline, "ball_diagnostics", None)
        snapshot_fn = getattr(tracker, "snapshot", None)
        candidate = snapshot_fn() if callable(snapshot_fn) else None
        if not isinstance(candidate, BallDiagnosticState):
            configured_required = getattr(
                self.runtime_settings,
                "incoming_confirmation_snapshots",
                None,
            )
            required = (
                None
                if configured_required is None
                else int(configured_required)
            )
            candidate = BallDiagnosticState(
                status="UNKNOWN",
                estimator_sample_count=None,
                estimator_window_size=self.estimator_window_size,
                speed_mps=None,
                velocity_world_mps=None,
                incoming_count=None,
                incoming_required_count=required,
                incoming_status="UNKNOWN",
                blocker=None,
            )
        observed = candidate.observed_monotonic_s
        if (
            candidate.last_estimator_reset_reason is None
            and candidate.status.upper() == "TRACK_ENDED"
        ):
            candidate = replace(
                candidate,
                last_estimator_reset_reason=candidate.blocker,
            )
        age_s = (
            None
            if observed is None
            else max(0.0, float(obs_now_s) - float(observed))
        )
        return replace(candidate, age_s=age_s)

    def _subject_health_snapshots(
        self,
        *,
        arrival: _ArrivalSnapshot,
        pelvis: object,
    ) -> Mapping[str, SubjectHealth]:
        def subject(
            name: str,
            *,
            stale_s: float,
            age_s: Optional[float] = None,
            source_frame: Optional[int] = None,
            valid: Optional[bool] = None,
            occluded: Optional[bool] = None,
        ) -> SubjectHealth:
            age = (
                arrival.ages.get(name)
                if age_s is None
                else age_s
            )
            frame = (
                arrival.source_frames.get(name)
                if source_frame is None
                else source_frame
            )
            effective_valid = (
                arrival.valid_by_subject.get(name)
                if valid is None
                else valid
            )
            effective_occluded = (
                arrival.occluded_by_subject.get(name)
                if occluded is None
                else occluded
            )
            if age is None:
                status = "NEVER_SEEN"
            elif age > stale_s:
                status = "STALE"
            elif (
                effective_valid is False
                or effective_occluded is True
            ):
                status = "INVALID"
            else:
                status = "LIVE"
            return SubjectHealth(
                status=status,
                rate_hz=(
                    None
                    if age is None
                    else arrival.rates.get(name)
                ),
                age_s=age,
                source_frame=frame,
                valid=effective_valid,
                occluded=effective_occluded,
            )

        pelvis_received = _finite_float_or_none(
            getattr(pelvis, "received_monotonic_s", None)
        )
        pelvis_age = arrival.ages.get("g1pelvis")
        return {
            "ball": subject(
                "ball",
                stale_s=self.options.heartbeat_stale_s,
            ),
            "g1pelvis": subject(
                "g1pelvis",
                stale_s=self.options.pelvis_stale_s,
                age_s=pelvis_age,
                source_frame=(
                    int(getattr(pelvis, "source_frame", 0))
                    if pelvis_received is not None
                    else None
                ),
                valid=(
                    bool(getattr(pelvis, "valid", False))
                    if pelvis_received is not None
                    else None
                ),
                occluded=(
                    bool(getattr(pelvis, "occluded", False))
                    if pelvis_received is not None
                    else None
                ),
            ),
            "table": subject(
                "table",
                stale_s=self.options.heartbeat_stale_s,
            ),
        }

    def _production_gate_snapshot(
        self,
        *,
        tick_result: object,
        incoming: object,
        ball: BallDiagnosticState,
        subjects: Mapping[str, SubjectHealth],
        obs_now_s: float,
    ) -> ProductionGateState:
        configured_required = getattr(
            self.runtime_settings,
            "incoming_confirmation_snapshots",
            None,
        )
        required = (
            None
            if configured_required is None
            else int(configured_required)
        )
        pelvis = subjects["g1pelvis"]
        pelvis_ready = bool(
            pelvis.status == "LIVE" and pelvis.valid is True
        )
        incoming_track_id = getattr(incoming, "track_id", None)
        incoming_count = int(
            getattr(incoming, "consecutive_count", 0)
        )
        incoming_confirmed = bool(
            getattr(incoming, "confirmed", False)
        )
        if not pelvis_ready or ball.estimator_ready is not True:
            production_incoming_status = "NOT_EVALUATED"
            production_incoming_count = None
        elif incoming_track_id is None:
            production_incoming_status = "NOT_EVALUATED"
            production_incoming_count = None
        elif incoming_confirmed:
            production_incoming_status = "CONFIRMED"
            production_incoming_count = incoming_count
        elif incoming_count > 0:
            production_incoming_status = "CONFIRMING"
            production_incoming_count = incoming_count
        else:
            production_incoming_status = "REJECTED"
            production_incoming_count = 0

        command = getattr(tick_result, "command_result", None)
        planner_tts_s = None
        if command is not None:
            deadline = _finite_float_or_none(
                getattr(
                    command,
                    "strike_deadline_monotonic_s",
                    None,
                )
            )
            if deadline is not None:
                planner_tts_s = deadline - float(obs_now_s)
        if not pelvis_ready:
            planner_status = "BLOCKED"
            planner_reason = "PELVIS_UNAVAILABLE"
            planner_tts_s = None
        elif ball.estimator_ready is not True:
            planner_status = "BLOCKED"
            ball_status = ball.status.upper()
            if ball.blocker is not None:
                planner_reason = ball.blocker
            elif ball_status == "ESTIMATING":
                planner_reason = "ESTIMATOR_WARMING"
            elif ball_status == "NOT_SEEN":
                planner_reason = "BALL_NOT_SEEN"
            elif ball.estimator_ready is None:
                planner_reason = "BALL_STATE_UNKNOWN"
            else:
                planner_reason = "BALL_STATE_UNAVAILABLE"
            planner_tts_s = None
        elif production_incoming_status != "CONFIRMED":
            planner_status = "BLOCKED"
            planner_reason = "INCOMING_NOT_CONFIRMED"
            planner_tts_s = None
        elif command is None:
            planner_status = "PENDING"
            planner_reason = None
        else:
            planner_reason = planner_reason_code(command)
            planner_status = (
                "REJECTED"
                if planner_reason is not None
                else "READY"
            )

        phase = str(getattr(tick_result, "phase", "")).upper()
        if planner_status != "READY":
            arm_status = "NOT_EVALUATED"
        elif phase == "ARMED":
            arm_status = "ARMED"
        else:
            arm_status = "NOT_ARMED"
        arm_trigger_tts_s = _finite_float_or_none(
            getattr(self.runtime_settings, "arm_tts_s", None)
        )

        task = getattr(tick_result, "task_observation", None)
        errors = tuple(getattr(tick_result, "errors", ()) or ())
        valid_dimensions = None
        clip_count = None
        if arm_status != "ARMED":
            task_status = "NOT_AVAILABLE"
        elif task is None:
            task_status = "ERROR" if errors else "NOT_AVAILABLE"
        else:
            values = getattr(task, "post_clip", ())
            try:
                observation = tuple(float(item) for item in values)
            except (TypeError, ValueError):
                observation = ()
            if observation and all(
                math.isfinite(item) for item in observation
            ):
                valid_dimensions = len(observation)
            clip_value = getattr(task, "clip_count", None)
            if (
                isinstance(clip_value, int)
                and not isinstance(clip_value, bool)
                and clip_value >= 0
            ):
                clip_count = int(clip_value)
            task_status = (
                "AVAILABLE"
                if (
                    bool(getattr(tick_result, "task_pass", False))
                    and not errors
                    and valid_dimensions is not None
                )
                else "ERROR"
            )

        return ProductionGateState(
            pelvis_status="READY" if pelvis_ready else "BLOCKED",
            production_incoming_status=production_incoming_status,
            production_incoming_count=production_incoming_count,
            production_incoming_required=required,
            planner_status=planner_status,
            planner_reason_code=planner_reason,
            planner_tts_s=planner_tts_s,
            arm_status=arm_status,
            arm_trigger_tts_s=arm_trigger_tts_s,
            task_observation_status=task_status,
            task_observation_valid_dimensions=valid_dimensions,
            task_observation_total_dimensions=11,
            task_observation_clip_count=clip_count,
        )

    def _next_live_snapshot_revision(self) -> int:
        with self._state_lock:
            self._live_snapshot_revision += 1
            return int(self._live_snapshot_revision)

    def _attempt_stage(
        self,
        summary: AttemptSummary,
        tick_result: object,
        incoming: object,
        *,
        estimator_sample_count: Optional[int],
        estimator_window_size: Optional[int] = None,
        diagnostic_incoming_count: Optional[int] = None,
        diagnostic_incoming_required: Optional[int] = None,
        diagnostic_incoming_confirmed: Optional[bool] = None,
        diagnostic_status: Optional[str] = None,
        production_gate: Optional[ProductionGateState] = None,
    ) -> str:
        original = str(summary.stage).upper()
        if original in _SPECIAL_ATTEMPT_STAGES:
            return original
        if production_gate is not None:
            planner_status = production_gate.planner_status.upper()
            planner_reason = production_gate.planner_reason_code
            if planner_status in ("BLOCKED", "REJECTED"):
                return "PLANNER_{}".format(
                    planner_reason or planner_status
                )
            if planner_status == "PENDING":
                return "PLANNER_PENDING"
            if planner_status == "READY":
                if production_gate.arm_status.upper() != "ARMED":
                    return "PLANNER_READY"
                task_status = (
                    production_gate.task_observation_status.upper()
                )
                if (
                    task_status == "AVAILABLE"
                    and bool(getattr(tick_result, "task_pass", False))
                ):
                    return "TASK_OBS_PASS"
                if task_status == "ERROR":
                    return "TASK_OBS_ERROR"
                return "ARMED"
        if bool(getattr(tick_result, "task_pass", False)):
            return "TASK_OBS_PASS"
        phase = str(getattr(tick_result, "phase", "")).upper()
        if phase == "ARMED":
            return "ARMED"
        command = getattr(tick_result, "command_result", None)
        if command is not None:
            reason = planner_reason_code(command)
            if reason is not None:
                return "PLANNER_{}".format(reason)
            if getattr(command, "command_fields", None) is not None:
                return "PLANNER_READY"
        if (
            diagnostic_status is not None
            and (
                estimator_sample_count is None
                or diagnostic_incoming_count is None
            )
        ):
            return str(diagnostic_status).upper()
        count = (
            int(getattr(incoming, "consecutive_count", 0))
            if diagnostic_incoming_count is None
            else int(diagnostic_incoming_count)
        )
        required = (
            int(
                getattr(
                    self.runtime_settings,
                    "incoming_confirmation_snapshots",
                    3,
                )
            )
            if diagnostic_incoming_required is None
            else int(diagnostic_incoming_required)
        )
        confirmed = (
            bool(getattr(incoming, "confirmed", False))
            if diagnostic_incoming_confirmed is None
            else bool(diagnostic_incoming_confirmed)
        )
        if confirmed:
            return "INCOMING_CONFIRMED {}/{}".format(count, required)
        if count > 0:
            return "INCOMING_CONFIRMING {}/{}".format(count, required)
        if estimator_sample_count is None:
            return (
                "UNKNOWN"
                if diagnostic_status is None
                else str(diagnostic_status).upper()
            )
        samples = int(estimator_sample_count)
        window_size = (
            self.estimator_window_size
            if estimator_window_size is None
            else int(estimator_window_size)
        )
        if samples < window_size:
            return "ESTIMATING {}/{}".format(
                samples,
                window_size,
            )
        return "PLANNER_WAITING"

    def _project_attempt(
        self,
        summary: AttemptSummary,
        *,
        tick_result: object,
        incoming: object,
        obs_now_s: float,
        recorder_status: object,
        ball: Optional[BallDiagnosticState] = None,
        production_gate: Optional[ProductionGateState] = None,
    ) -> AttemptSummary:
        command = getattr(tick_result, "command_result", None)
        predicted = None
        planner_tts = None
        if command is not None:
            deadline = float(
                getattr(command, "strike_deadline_monotonic_s", math.nan)
            )
            if math.isfinite(deadline):
                predicted = deadline
                planner_tts = deadline - float(obs_now_s)
        if (
            production_gate is not None
            and production_gate.planner_status != "READY"
        ):
            predicted = None
            planner_tts = None
        elif production_gate is not None:
            planner_tts = production_gate.planner_tts_s
        if ball is not None:
            estimator_sample_count = ball.estimator_sample_count
            estimator_window_size = ball.estimator_window_size
            ball_speed_mps = ball.speed_mps
            incoming_count = ball.incoming_count
            incoming_required_count = ball.incoming_required_count
        else:
            estimator_sample_count = None
            estimator_window_size = None
            ball_speed_mps = None
            incoming_count = None
            incoming_required_count = None
        diagnostic_confirmed = (
            None
            if ball is None
            else ball.ball_only_incoming_confirmed
        )
        task_obs_status = (
            summary.task_obs_status
            if production_gate is None
            else production_gate.task_observation_status
        )
        primary_blocker = summary.primary_blocker
        if (
            production_gate is not None
            and production_gate.planner_status in ("BLOCKED", "REJECTED")
            and production_gate.planner_reason_code is not None
        ):
            primary_blocker = production_gate.planner_reason_code
        live_arm_tts = (
            planner_tts
            if (
                production_gate is not None
                and production_gate.arm_status == "ARMED"
            )
            else None
        )
        return replace(
            summary,
            stage=self._attempt_stage(
                summary,
                tick_result,
                incoming,
                estimator_sample_count=estimator_sample_count,
                estimator_window_size=estimator_window_size,
                diagnostic_incoming_count=incoming_count,
                diagnostic_incoming_required=incoming_required_count,
                diagnostic_incoming_confirmed=diagnostic_confirmed,
                diagnostic_status=(
                    None if ball is None else ball.status
                ),
                production_gate=production_gate,
            ),
            primary_blocker=primary_blocker,
            ball_speed_mps=ball_speed_mps,
            predicted_strike_time_s=predicted,
            planner_tts_s=planner_tts,
            arm_tts_s=live_arm_tts,
            task_obs_status=task_obs_status,
            recording_complete=bool(
                recorder_status.recording_complete
                and self.input_recording_complete
                and self._attempt_recording_complete(summary.attempt_id)
            ),
            estimator_sample_count=estimator_sample_count,
            estimator_window_size=estimator_window_size,
            incoming_count=incoming_count,
            incoming_required_count=incoming_required_count,
        )

    def _remember_tick_detail(
        self,
        tick_result: object,
    ) -> None:
        binding = getattr(tick_result, "active_binding", None)
        if binding is None:
            return
        attempt_id = int(binding.attempt_id)
        with self._state_lock:
            if attempt_id in self._closed_attempt_ids:
                return
            value = self._attempt_observations.setdefault(attempt_id, {})
            command = getattr(tick_result, "command_result", None)
            if command is not None:
                value["planner_results"] = (
                    {
                        "snapshot_key": command.snapshot_key.to_json_dict(),
                        "source_frame": command.source_frame,
                        "strike_deadline_monotonic_s": (
                            command.strike_deadline_monotonic_s
                        ),
                        "completed_monotonic_s": (
                            command.completed_monotonic_s
                        ),
                        "command_fields": command.command_fields,
                        "reason_code": planner_reason_code(command),
                        "error_type": command.error_type,
                        "error_text": command.error_text,
                    },
                )
            task = getattr(tick_result, "task_observation", None)
            if task is not None:
                value["pre_clip"] = tuple(
                    float(item) for item in task.pre_clip
                )
                value["post_clip"] = tuple(
                    float(item) for item in task.post_clip
                )
                value["clip_count"] = int(task.clip_count)

    def _attempt_detail(
        self,
        *,
        summary: AttemptSummary,
    ) -> AttemptDetail:
        timeline = self.pipeline.attempt_tracker.timeline_for_attempt(
            summary.attempt_id
        )
        segment_values: Dict[int, Dict[str, Any]] = {}
        for transition in timeline:
            segment_id = transition.track_segment_id
            if segment_id is None:
                continue
            segment = segment_values.setdefault(
                int(segment_id),
                {
                    "track_segment_id": int(segment_id),
                    "first_monotonic_s": transition.monotonic_s,
                    "last_monotonic_s": transition.monotonic_s,
                    "role": None,
                },
            )
            segment["last_monotonic_s"] = transition.monotonic_s
            role = transition.values.get("role")
            if role is not None:
                segment["role"] = role
        with self._state_lock:
            captured = self._attempt_observations.pop(
                summary.attempt_id,
                {},
            )
        return AttemptDetail(
            attempt_id=summary.attempt_id,
            summary=summary,
            segments=tuple(
                segment_values[key] for key in sorted(segment_values)
            ),
            stage_timeline=tuple(
                transition.to_json_dict() for transition in timeline
            ),
            planner_inputs=tuple(captured.get("planner_inputs", ())),
            planner_results=tuple(captured.get("planner_results", ())),
            task_observation_pre_clip=captured.get("pre_clip"),
            task_observation_post_clip=captured.get("post_clip"),
            task_observation_clip_count=captured.get("clip_count"),
            variant_outcomes=(),
            ab_deltas={},
        )

    def _close_attempt(
        self,
        *,
        attempt_id: int,
        tick_result: object,
        incoming: object,
        obs_now_s: float,
        wall_time_us: int,
        recorder_status: object,
    ) -> None:
        summary = self.pipeline.attempt_tracker.summary_for_attempt(
            attempt_id
        )
        if summary is None:
            return
        projected = self._project_attempt(
            summary,
            tick_result=tick_result,
            incoming=incoming,
            obs_now_s=obs_now_s,
            recorder_status=recorder_status,
        )
        projected = replace(
            projected,
            stage="ATTEMPT_CLOSED",
            ab_summary=(
                "INCONCLUSIVE"
                if (
                    self.replay_controller is not None
                    and self.replay_capture is not None
                )
                else projected.ab_summary
            ),
        )
        detail = self._attempt_detail(summary=projected)
        if not self.event_lane.enqueue_close(
            _AttemptCloseSignal(
                attempt_id=int(attempt_id),
                monotonic_s=float(obs_now_s),
                wall_time_us=int(wall_time_us),
                detail=detail,
            )
        ):
            self._mark_recording_incomplete(
                attempt_id,
                "ATTEMPT_CLOSE_QUEUE_FULL",
            )

    def _health_snapshot(
        self,
        *,
        obs_now_s: float,
        recorder_status: object,
        arrival: Optional[_ArrivalSnapshot] = None,
        pelvis: Optional[object] = None,
        current_health: Optional[object] = None,
    ) -> HealthSnapshot:
        if arrival is None:
            arrival = self._arrival.snapshot(obs_now_s)
        if pelvis is None:
            pelvis = self.adapter.copy_latest_pelvis()
        pelvis_received = getattr(
            pelvis,
            "received_monotonic_s",
            None,
        )
        pelvis_age = (
            None
            if pelvis_received is None
            else max(0.0, obs_now_s - float(pelvis_received))
        )
        pelvis_valid = bool(getattr(pelvis, "valid", False))
        if pelvis_age is not None and pelvis_age > self.options.pelvis_stale_s:
            pelvis_valid = False
        stats = self.pipeline.worker.stats
        if current_health is None:
            current_health = self.event_hub.state_snapshot().health
        warnings = list(self._last_adapter_warnings)
        if (
            not arrival.received_any
            or all(
                age is None or age > self.options.heartbeat_stale_s
                for age in arrival.ages.values()
            )
        ):
            warnings.append("LCM_HEARTBEAT_STALE")
        if not pelvis_valid:
            warnings.append("PELVIS_INVALID_OR_STALE")
        if not bool(recorder_status.healthy):
            warnings.append("RECORDER_UNHEALTHY")
        with self._state_lock:
            warnings.extend(self._warnings)
        return HealthSnapshot(
            lcm_connected=bool(
                self._subscription is not None
                and not self._stop_event.is_set()
                and arrival.received_any
                and any(
                    age is not None
                    and age <= self.options.heartbeat_stale_s
                    for age in arrival.ages.values()
                )
            ),
            message_rate_hz_by_subject=arrival.rates,
            message_age_s_by_subject=arrival.ages,
            source_frame_by_subject=arrival.source_frames,
            pelvis_valid=pelvis_valid,
            pelvis_age_s=pelvis_age,
            planner_submitted=int(getattr(stats, "submitted", 0)),
            planner_completed=int(getattr(stats, "completed", 0)),
            planner_failed=int(getattr(stats, "failed", 0)),
            planner_dropped_pending=int(
                getattr(stats, "dropped_pending", 0)
            ),
            planner_results_overwritten_before_consume=int(
                getattr(
                    self.pipeline,
                    "planner_results_overwritten_before_consume",
                    0,
                )
            ),
            raw_samples_dropped=(
                (
                    int(self.raw_counter.dropped)
                    if self.raw_counter is not None
                    else int(
                        getattr(self.pipeline, "raw_sink_failures", 0)
                    )
                )
                + self.input_samples_dropped
            ),
            recorder_event_gaps=int(
                getattr(current_health, "recorder_event_gaps", 0)
            ),
            diagnostic_events_dropped=int(
                getattr(current_health, "diagnostic_events_dropped", 0)
            ),
            recorder_healthy=bool(recorder_status.healthy),
            recording_complete=bool(
                recorder_status.recording_complete
                and self.input_recording_complete
            ),
            config_name=self.options.mimic_config.stem,
            session_basename=self.session_basename,
            warnings=tuple(dict.fromkeys(warnings)),
        )

    @staticmethod
    def _tick_attempt_ids(tick_result: object) -> Tuple[int, ...]:
        values = []
        for name in ("active_binding", "cached_binding"):
            binding = getattr(tick_result, name, None)
            attempt_id = getattr(binding, "attempt_id", None)
            if (
                isinstance(attempt_id, int)
                and not isinstance(attempt_id, bool)
                and attempt_id > 0
                and attempt_id not in values
            ):
                values.append(int(attempt_id))
        return tuple(values)

    def _tick_can_project_attempt(
        self,
        tick_result: object,
        summary: AttemptSummary,
    ) -> bool:
        attempt_id = int(summary.attempt_id)
        tick_attempt_ids = self._tick_attempt_ids(tick_result)
        with self._state_lock:
            if attempt_id in self._closed_attempt_ids:
                return False
        return (
            not tick_attempt_ids
            or attempt_id in tick_attempt_ids
        )

    def _tick_once(self) -> None:
        """Run one serialized production-order projection tick."""
        with self._attempt_projection_lock:
            if self._closed:
                return
            self._tick_once_under_projection()

    def _tick_once_under_projection(self) -> None:
        """Run one production-order tick with exactly two semantic clocks."""
        with self._snapshot_capture_lock:
            lifecycle_now_s = self._monotonic_fn()
            obs_now_s = self._monotonic_fn()
            wall_time_us = int(self._wall_time_fn() * 1.0e6)
            with self._tick_lock:
                tick_result = self.pipeline.tick(
                    lifecycle_now_s=lifecycle_now_s,
                    obs_now_s=obs_now_s,
                    wall_time_us=wall_time_us,
                )
                transitions = self.pipeline.attempt_tracker.advance(
                    now_monotonic_s=obs_now_s
                )
            self._last_tick_result = tick_result
            self._remember_tick_detail(tick_result)

            lifecycle = LifecycleSnapshot(
                phase=str(tick_result.phase),
                last_decision=str(tick_result.lifecycle_decision),
                active_key=_snapshot_key(tick_result.active_binding),
                cached_key=_snapshot_key(tick_result.cached_binding),
                lifecycle_now_s=lifecycle_now_s,
                obs_now_s=obs_now_s,
            )
            recorder_status = self._cached_recorder_status()
            incoming = self.pipeline.incoming.snapshot()
            ball = self._ball_diagnostic_snapshot(
                obs_now_s=obs_now_s,
                incoming=incoming,
            )
            arrival = self._arrival.snapshot(obs_now_s)
            pelvis = self.adapter.copy_latest_pelvis()
            current_state = self.event_hub.state_snapshot()
            current_health = current_state.health
            subjects = self._subject_health_snapshots(
                arrival=arrival,
                pelvis=pelvis,
            )
            production_gate = self._production_gate_snapshot(
                tick_result=tick_result,
                incoming=incoming,
                ball=ball,
                subjects=subjects,
                obs_now_s=obs_now_s,
            )
            current = self.pipeline.attempt_tracker.current_summary()
            projected = None
            if (
                current is not None
                and self._tick_can_project_attempt(tick_result, current)
            ):
                projected = self._project_attempt(
                    current,
                    tick_result=tick_result,
                    incoming=incoming,
                    obs_now_s=obs_now_s,
                    recorder_status=recorder_status,
                    ball=ball,
                    production_gate=production_gate,
                )
                with self._state_lock:
                    self._attempt_observations.setdefault(
                        projected.attempt_id,
                        {},
                    )["projected_summary"] = projected

            health = self._health_snapshot(
                obs_now_s=obs_now_s,
                recorder_status=recorder_status,
                arrival=arrival,
                pelvis=pelvis,
                current_health=current_health,
            )
            live_snapshot = LiveDiagnosticSnapshot(
                revision=self._next_live_snapshot_revision(),
                captured_monotonic_s=obs_now_s,
                subjects=subjects,
                ball=ball,
                production_gate=production_gate,
                current_attempt=projected,
            )

        for transition in transitions:
            self._offer(
                EventDraft(
                    kind="attempt_transition",
                    monotonic_s=transition.monotonic_s,
                    wall_time_us=wall_time_us,
                    scope="attempt",
                    attempt_id=transition.attempt_id,
                    payload=_transition_payload(transition),
                )
            )
        self._offer(
            EventDraft(
                kind="LIVE_SNAPSHOT",
                monotonic_s=obs_now_s,
                wall_time_us=wall_time_us,
                scope="state",
                attempt_id=None,
                payload=live_snapshot.to_json_dict(),
            )
        )
        self._offer(
            EventDraft(
                kind="LIFECYCLE_SNAPSHOT",
                monotonic_s=lifecycle_now_s,
                wall_time_us=wall_time_us,
                scope="state",
                attempt_id=None,
                payload={"lifecycle": lifecycle.to_json_dict()},
            )
        )
        if projected is not None:
            self._offer(
                EventDraft(
                    kind="ATTEMPT_CURRENT",
                    monotonic_s=obs_now_s,
                    wall_time_us=wall_time_us,
                    scope="attempt",
                    attempt_id=projected.attempt_id,
                    payload={"attempt": projected.to_json_dict()},
                )
            )
        self._offer(
            EventDraft(
                kind="HEALTH_SNAPSHOT",
                monotonic_s=obs_now_s,
                wall_time_us=wall_time_us,
                scope="state",
                attempt_id=None,
                payload={"health": health.to_json_dict()},
            )
        )

        replay = self.replay_controller
        if replay is not None:
            stage = "" if current is None else str(current.stage).upper()
            attempt_active = (
                current is not None and stage != "REACQUIRE_GRACE"
            )
            grace_active = stage == "REACQUIRE_GRACE"
            with self._state_lock:
                if (
                    attempt_active != self._replay_attempt_active
                    or grace_active != self._replay_grace_active
                ):
                    self._replay_attempt_active = attempt_active
                    self._replay_grace_active = grace_active
                    self._replay_state_version += 1

    @staticmethod
    def _remaining(deadline: float) -> float:
        return max(0.0, deadline - time.monotonic())

    @staticmethod
    def _acquire_before(
        lock: object,
        *,
        deadline_s: Optional[float],
    ) -> bool:
        if deadline_s is None:
            lock.acquire()
            return True
        return bool(
            lock.acquire(
                timeout=max(0.0, deadline_s - time.monotonic())
            )
        )

    def close(self, *, timeout_s: float = 5.0) -> bool:
        """Execute the finite shutdown order once; subsequent calls reuse it."""
        timeout = _finite_nonnegative("timeout_s", timeout_s)
        with self._close_lock:
            if self._closed:
                return self._close_result
            self._closed = True
            deadline = time.monotonic() + timeout
            clean = True

            self._stop_event.set()
            subscription = self._subscription
            self._subscription = None
            if subscription is not None:
                try:
                    self.lcm_client.unsubscribe(subscription)
                except Exception:
                    clean = False
            lcm_thread = self._lcm_thread
            if (
                lcm_thread is not None
                and lcm_thread is not threading.current_thread()
            ):
                lcm_thread.join(self._remaining(deadline))
                clean = clean and not lcm_thread.is_alive()

            with self._input_condition:
                self._input_accepting = False
                self._input_stop_requested = True
                self._input_condition.notify_all()
            input_thread = self._input_thread
            if (
                input_thread is not None
                and input_thread is not threading.current_thread()
            ):
                input_thread.join(self._remaining(deadline))
                clean = clean and not input_thread.is_alive()

            tick_thread = self._tick_thread
            if (
                tick_thread is not None
                and tick_thread is not threading.current_thread()
            ):
                tick_thread.join(self._remaining(deadline))
                clean = clean and not tick_thread.is_alive()

            tick_lock_acquired = False
            try:
                tick_lock_acquired = self._acquire_before(
                    self._tick_lock,
                    deadline_s=deadline,
                )
                if tick_lock_acquired:
                    clean = bool(
                        self.pipeline.close(
                            timeout_s=self._remaining(deadline)
                        )
                    ) and clean
                else:
                    clean = False
            except Exception:
                clean = False
            finally:
                if tick_lock_acquired:
                    self._tick_lock.release()
            try:
                self._drain_attempt_close_requests(deadline_s=deadline)
                clean = (
                    self.event_lane.close_pending_count == 0
                ) and clean
            except Exception:
                clean = False
            if self.replay_controller is not None:
                replay_thread = self._replay_thread
                if (
                    replay_thread is not None
                    and replay_thread is not threading.current_thread()
                ):
                    replay_thread.join(self._remaining(deadline))
                    clean = clean and not replay_thread.is_alive()
                try:
                    clean = bool(
                        self.replay_controller.close(
                            timeout_s=self._remaining(deadline)
                        )
                    ) and clean
                except Exception:
                    clean = False

            try:
                clean = bool(
                    self.event_lane.drain(
                        timeout_s=self._remaining(deadline)
                    )
                ) and clean
                clean = bool(
                    self.event_lane.close(
                        timeout_s=self._remaining(deadline)
                    )
                ) and clean
            except Exception:
                clean = False

            try:
                self.recorder.request_stop()
                self._cache_recorder_status(
                    self.recorder.drain(self._remaining(deadline))
                )
            except Exception:
                clean = False
            try:
                self._cache_recorder_status(
                    self.recorder.write_terminal_and_close(
                        terminal_session={
                            "reason": (
                                "failure"
                                if self._failure_snapshot() is not None
                                else "stopped"
                            ),
                            "failure": self._failure_snapshot(),
                            "ended_wall_time_us": int(
                                self._wall_time_fn() * 1.0e6
                            ),
                            "input_samples_dropped": (
                                self.input_samples_dropped
                            ),
                        },
                        timeout_s=self._remaining(deadline),
                    )
                )
            except Exception:
                clean = False

            try:
                clean = bool(
                    self.web_server.close(
                        timeout_s=self._remaining(deadline)
                    )
                ) and clean
            except Exception:
                clean = False
            try:
                self.event_hub.close()
            except Exception:
                clean = False

            for thread in (self._recorder_thread,):
                if (
                    thread is not None
                    and thread is not threading.current_thread()
                ):
                    thread.join(self._remaining(deadline))
                    clean = clean and not thread.is_alive()
            self._close_result = clean
            return clean


def build_monitor(
    options: MonitorOptions,
    *,
    lcm_factory: Callable[[str], object] = lcm.LCM,
    monotonic_fn: Callable[[], float] = time.monotonic,
    wall_time_fn: Callable[[], float] = time.time,
    decoder: Callable[[bytes], object] = transformation_t.decode,
    argv: Optional[Sequence[str]] = None,
) -> HitterTaskMonitor:
    """Build the dependency-injected, read-only diagnostic process."""
    resolved = _resolved_options(options)
    mimic = _config_mapping(resolved.mimic_config)
    control = _config_mapping(resolved.control_config)
    policy = mimic.get("policy", {})
    motion = mimic.get("motion", {})
    if not isinstance(policy, Mapping) or not isinstance(motion, Mapping):
        raise TypeError("mimic config policy and motion must be mappings")
    planner_config = motion.get("ball_planner", {})
    if not isinstance(planner_config, Mapping):
        raise TypeError("motion.ball_planner must be a mapping")
    settings = resolve_hitter_runtime_settings(
        policy_config=policy,
        motion_config=motion,
        control_config=control,
    )
    estimator_window_size = int(
        planner_config.get("state_estimator_window_size", 31)
    )
    paths = create_session_paths(resolved.output_dir)
    metadata = _session_metadata(
        options=resolved,
        mimic_config=mimic,
        control_config=control,
        settings=settings,
        estimator_window_size=estimator_window_size,
        argv=tuple(sys.argv if argv is None else argv),
    )
    repository = AttemptDetailRepository(
        paths.attempt_details_dir,
        cache_size=resolved.attempt_cache_size,
    )
    initial = _initial_state(
        config_name=resolved.mimic_config.stem,
        session_basename=paths.root.name,
        subjects=("ball", "g1pelvis", "table"),
    )
    hub = EventHub(
        initial,
        capacity=max(1024, resolved.event_capacity * 2),
    )
    event_lane = EventPublisherLane(
        hub,
        capacity=resolved.event_capacity,
    )
    raw_lane = RawRecordLane(capacity=resolved.raw_capacity)
    raw_counter = _CountingRawSink(raw_lane)
    replay_capture = ReplayCaptureStore(
        input_capacity=max(2, resolved.input_capacity * 2),
        event_capacity=max(
            resolved.event_capacity,
            resolved.input_capacity,
        ),
        attempt_capacity=resolved.attempt_cache_size,
    )
    captured_event_lane = ReplayEventCaptureTee(
        event_lane,
        replay_capture,
    )
    captured_raw_sink = ReplayRawCaptureTee(
        raw_counter,
        replay_capture,
    )
    recorder = SessionRecorder(
        paths=paths,
        raw_lane=raw_lane,
        event_cursor=hub.open_cursor(0),
        attempt_repository=repository,
        session_metadata=metadata,
        clock=monotonic_fn,
        event_sink=event_lane.offer,
    )
    adapter = MocapFrameAdapter(
        planner_config,
        estimator_sample_rate_hz=settings.estimator_sample_rate_hz,
        pelvis_stale_threshold_s=resolved.pelvis_stale_s,
        maximum_source_frame_delta=resolved.source_frame_gap_threshold,
    )
    tracker = AttemptTracker(
        reacquire_grace_s=resolved.reacquire_grace_s,
        max_attempts=max(100, resolved.attempt_cache_size),
    )
    pipeline = None
    web_server = None
    replay_controller = None
    try:
        replay_controller = ReplayProcessController(
            worker_fn=run_replay_disk_job,
            recorder=recorder,
            event_sink=captured_event_lane,
            pending_capacity=resolved.attempt_cache_size,
        )
        resolved_forced_strike_type = forced_strike_type(
            planner_config,
            motion,
        )
        pipeline = ShadowTaskPipeline(
            adapter=adapter,
            settings=settings,
            event_sink=captured_event_lane,
            raw_sink=captured_raw_sink,
            planner=build_hitter_system_planner(planner_config),
            attempt_tracker=tracker,
            forced_strike_type=resolved_forced_strike_type,
        )
        application = HitterTaskWebApplication(
            hub=hub,
            attempts=repository,
            static_files={"/": _STATIC_PAGE},
        )
        web_server = HitterTaskWebServer(
            application,
            WebServerConfig(
                host="127.0.0.1",
                port=resolved.port,
            ),
        )
        transport = lcm_factory(resolved.lcm_url)
        return HitterTaskMonitor(
            resolved,
            lcm_client=transport,
            decoder=decoder,
            adapter=adapter,
            pipeline=pipeline,
            recorder=recorder,
            web_server=web_server,
            event_lane=captured_event_lane,
            event_hub=hub,
            attempt_repository=repository,
            session_basename=paths.root.name,
            control_tick_s=settings.control_tick_s,
            estimator_window_size=estimator_window_size,
            session_paths=paths,
            runtime_settings=settings,
            raw_counter=raw_counter,
            replay_controller=replay_controller,
            replay_capture=replay_capture,
            replay_planner_config=planner_config,
            replay_forced_strike_type=resolved_forced_strike_type,
            monotonic_fn=monotonic_fn,
            wall_time_fn=wall_time_fn,
        )
    except BaseException:
        if replay_controller is not None:
            replay_controller.close(timeout_s=1.0)
        if web_server is not None:
            web_server.close(timeout_s=1.0)
        if pipeline is not None:
            pipeline.close(timeout_s=1.0)
        event_lane.close(timeout_s=1.0)
        hub.close()
        raise


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="HITTER 任务观测只读旁路诊断",
    )
    parser.add_argument(
        "--mimic-config",
        type=Path,
        default=Path("config/mimic/hitter.yaml"),
    )
    parser.add_argument(
        "--control-config",
        type=Path,
        default=Path("config/control/g1_hitter_racket.yaml"),
    )
    parser.add_argument(
        "--table-calib",
        type=Path,
        default=Path(
            "mocap_bridge/calibrations/chingmu_table_frame_latest.json"
        ),
    )
    parser.add_argument(
        "--pelvis-calib",
        type=Path,
        default=Path(
            "mocap_bridge/calibrations/"
            "chingmu_g2_pelvis_orientation_latest.json"
        ),
    )
    parser.add_argument(
        "--lcm-url",
        default="udpm://239.255.76.67:7667?ttl=255",
    )
    parser.add_argument("--channel", default="vicon_state_data")
    parser.add_argument("--base-name", default="G2Pelvis")
    parser.add_argument("--ball-name", default="ball")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--output-dir", type=Path, default=default_output_dir())
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--reacquire-grace-s", type=float, default=0.20)
    parser.add_argument("--heartbeat-stale-s", type=float, default=0.10)
    parser.add_argument("--pelvis-stale-s", type=float, default=0.05)
    parser.add_argument(
        "--source-frame-gap-threshold",
        type=int,
        default=1,
    )
    parser.add_argument("--input-capacity", type=int, default=65536)
    parser.add_argument("--raw-capacity", type=int, default=65536)
    parser.add_argument("--event-capacity", type=int, default=4096)
    parser.add_argument("--attempt-cache-size", type=int, default=100)
    return parser


def _options_from_args(arguments: argparse.Namespace) -> MonitorOptions:
    return MonitorOptions(
        mimic_config=arguments.mimic_config,
        control_config=arguments.control_config,
        table_calib=arguments.table_calib,
        pelvis_calib=arguments.pelvis_calib,
        lcm_url=arguments.lcm_url,
        channel=arguments.channel,
        base_name=arguments.base_name,
        ball_name=arguments.ball_name,
        port=arguments.port,
        output_dir=arguments.output_dir,
        duration_s=arguments.duration,
        reacquire_grace_s=arguments.reacquire_grace_s,
        heartbeat_stale_s=arguments.heartbeat_stale_s,
        pelvis_stale_s=arguments.pelvis_stale_s,
        source_frame_gap_threshold=arguments.source_frame_gap_threshold,
        raw_capacity=arguments.raw_capacity,
        event_capacity=arguments.event_capacity,
        attempt_cache_size=arguments.attempt_cache_size,
        input_capacity=arguments.input_capacity,
    )


def main(argv: Optional[Sequence[str]] = None) -> int:
    arguments = _parser().parse_args(argv)
    command_argv = tuple(sys.argv if argv is None else (sys.argv[0],) + tuple(argv))
    monitor = build_monitor(
        _options_from_args(arguments),
        argv=command_argv,
    )
    print("HITTER 只读旁路诊断页面: {}".format(monitor.page_url))
    print("记录会话: {}".format(monitor.session_basename))
    return monitor.run()


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "HitterTaskMonitor",
    "MonitorOptions",
    "build_monitor",
    "default_output_dir",
    "main",
]
