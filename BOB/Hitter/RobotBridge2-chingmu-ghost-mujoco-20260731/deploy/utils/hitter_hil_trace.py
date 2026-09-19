from __future__ import annotations

import json
import queue
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np


def _jsonable(value):
    if isinstance(value, np.ndarray):
        return [_jsonable(item) for item in value.tolist()]
    if isinstance(value, np.generic):
        return _jsonable(value.item())
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    return repr(value)


def _attr(obj, name: str, default=None):
    if obj is None:
        return default
    if isinstance(obj, dict):
        return obj.get(name, default)
    return getattr(obj, name, default)


def _float_or_none(value):
    if value is None:
        return None
    result = float(value)
    if not np.isfinite(result):
        return None
    return result


@dataclass(frozen=True)
class HitterHilTraceConfig:
    queue_capacity: int = 4096
    rotate_bytes: int = 67108864
    retained_files: int = 4
    flush_interval_s: float = 1.0
    status_log_interval_s: float = 1.0

    def __post_init__(self) -> None:
        if int(self.queue_capacity) < 1:
            raise ValueError("trace queue_capacity must be positive")
        if int(self.rotate_bytes) < 1:
            raise ValueError("trace rotate_bytes must be positive")
        if int(self.retained_files) < 1:
            raise ValueError("trace retained_files must be positive")
        if not (0.0 < float(self.flush_interval_s) <= 1.0):
            raise ValueError("trace flush_interval_s must be in (0, 1]")
        if float(self.status_log_interval_s) < 1.0:
            raise ValueError("status_log_interval_s must be at least 1 s")

    @classmethod
    def from_mapping(cls, mapping) -> "HitterHilTraceConfig":
        return cls(
            queue_capacity=int(_attr(mapping, "trace_queue_capacity", 4096)),
            rotate_bytes=int(_attr(mapping, "trace_rotate_bytes", 67108864)),
            retained_files=int(_attr(mapping, "trace_retained_files", 4)),
            flush_interval_s=float(
                _attr(mapping, "trace_flush_interval_s", 1.0)
            ),
            status_log_interval_s=float(
                _attr(mapping, "status_log_interval_s", 1.0)
            ),
        )


class HitterHilRateLimiter:
    def __init__(self, *, interval_s: float):
        self.interval_s = float(interval_s)
        if self.interval_s <= 0.0:
            raise ValueError("interval_s must be positive")
        self._last_emit_s: float | None = None

    def should_emit(self, now_s: float) -> bool:
        now = float(now_s)
        if (
            self._last_emit_s is None
            or now - self._last_emit_s >= self.interval_s - 1.0e-12
        ):
            self._last_emit_s = now
            return True
        return False


class HitterHilTraceWriter:
    def __init__(
        self,
        path,
        *,
        config: HitterHilTraceConfig | None = None,
        start_thread: bool = True,
        monotonic_fn=time.monotonic,
    ) -> None:
        self.path = Path(path)
        self.config = config or HitterHilTraceConfig()
        self._monotonic_fn = monotonic_fn
        self._queue: queue.Queue[dict] = queue.Queue(
            maxsize=self.config.queue_capacity
        )
        self._stop_event = threading.Event()
        self._thread = None
        self._file = None
        self._lock = threading.RLock()
        self.trace_drop_count = 0
        self.status_limiter = HitterHilRateLimiter(
            interval_s=self.config.status_log_interval_s
        )
        if start_thread:
            self.start()

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="HitterHilTraceWriter",
            daemon=True,
        )
        self._thread.start()

    def record(self, kind: str, **fields) -> bool:
        record = {
            "kind": str(kind),
            "trace_monotonic_s": float(self._monotonic_fn()),
        }
        record.update({key: _jsonable(value) for key, value in fields.items()})
        try:
            self._queue.put_nowait(record)
        except queue.Full:
            self.trace_drop_count += 1
            return False
        return True

    def status_should_log(self, now_s: float) -> bool:
        return self.status_limiter.should_emit(now_s)

    def _open_file_locked(self):
        if self._file is None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self._file = self.path.open("a", encoding="utf-8")
        return self._file

    def _rotate_if_needed_locked(self) -> None:
        if self._file is not None:
            self._file.flush()
            if self.path.exists() and self.path.stat().st_size < self.config.rotate_bytes:
                return
            self._file.close()
            self._file = None
        for index in range(self.config.retained_files - 1, 0, -1):
            src = self.path.with_name(f"{self.path.name}.{index}")
            dst = self.path.with_name(f"{self.path.name}.{index + 1}")
            if src.exists():
                if index + 1 > self.config.retained_files:
                    src.unlink()
                else:
                    src.replace(dst)
        if self.path.exists():
            self.path.replace(self.path.with_name(f"{self.path.name}.1"))

    def flush_pending(self) -> int:
        written = 0
        with self._lock:
            while True:
                try:
                    record = self._queue.get_nowait()
                except queue.Empty:
                    break
                self._rotate_if_needed_locked()
                handle = self._open_file_locked()
                handle.write(
                    json.dumps(record, sort_keys=True, separators=(",", ":"))
                    + "\n"
                )
                written += 1
            if self._file is not None:
                self._file.flush()
        return written

    def _run(self) -> None:
        while not self._stop_event.is_set():
            self.flush_pending()
            time.sleep(self.config.flush_interval_s)
        self.flush_pending()

    def close(self, *, join_timeout_s: float = 1.0) -> bool:
        self._stop_event.set()
        thread = self._thread
        ok = True
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=float(join_timeout_s))
            ok = not thread.is_alive()
        self.flush_pending()
        with self._lock:
            if self._file is not None:
                self._file.close()
                self._file = None
        return ok


def build_hitter_hil_status_record(
    *,
    now_monotonic_s: float,
    source_status,
    pipeline_state,
    worker_stats,
    latest_result,
    policy_time_to_strike_s,
    predicted_strike_point_w,
    commanded_base_target_xy_w,
    commanded_racket_velocity_w,
    mujoco_racket_position_w,
    mujoco_racket_velocity_w,
    ghost_to_racket_min_distance_m,
    publication_audit,
) -> dict:
    publish_count, publish_channels = publication_audit.snapshot()
    completed_s = _float_or_none(
        _attr(latest_result, "completed_monotonic_s", None)
    )
    result_age_s = None if completed_s is None else max(
        float(now_monotonic_s) - completed_s,
        0.0,
    )
    return {
        "kind": "status",
        "now_monotonic_s": float(now_monotonic_s),
        "source_decoded_count": int(_attr(source_status, "decoded_count", 0)),
        "source_accepted_ball_count": int(
            _attr(source_status, "accepted_ball_count", 0)
        ),
        "source_rejected_count": int(_attr(source_status, "rejected_count", 0)),
        "source_table_ready": bool(_attr(source_status, "table_ready", False)),
        "source_table_confirmation_count": int(
            _attr(source_status, "table_confirmation_count", 0)
        ),
        "source_thread_alive": bool(_attr(source_status, "thread_alive", False)),
        "source_receive_failure": _attr(source_status, "receive_failure", None),
        "estimator_track_epoch": int(_attr(pipeline_state, "track_epoch", 0)),
        "estimator_generation": int(_attr(pipeline_state, "generation", 0)),
        "estimator_ready": bool(_attr(pipeline_state, "ready", False)),
        "estimator_visible": bool(_attr(pipeline_state, "visible", False)),
        "estimator_sample_count": int(_attr(pipeline_state, "sample_count", 0)),
        "estimator_min_samples": int(_attr(pipeline_state, "min_samples", 0)),
        "last_update_monotonic_s": _float_or_none(
            _attr(pipeline_state, "last_update_monotonic_s", None)
        ),
        "worker_submitted": int(_attr(worker_stats, "submitted", 0)),
        "worker_completed": int(_attr(worker_stats, "completed", 0)),
        "worker_dropped": int(_attr(worker_stats, "dropped_pending", 0)),
        "worker_failed": int(_attr(worker_stats, "failed", 0)),
        "result_age_s": result_age_s,
        "strike_deadline_monotonic_s": _float_or_none(
            _attr(latest_result, "strike_deadline_monotonic_s", None)
        ),
        "policy_time_to_strike_s": _float_or_none(policy_time_to_strike_s),
        "predicted_strike_point_w": _jsonable(predicted_strike_point_w),
        "commanded_base_target_xy_w": _jsonable(commanded_base_target_xy_w),
        "commanded_racket_velocity_w": _jsonable(commanded_racket_velocity_w),
        "mujoco_racket_position_w": _jsonable(mujoco_racket_position_w),
        "mujoco_racket_velocity_w": _jsonable(mujoco_racket_velocity_w),
        "ghost_to_racket_min_distance_m": _float_or_none(
            ghost_to_racket_min_distance_m
        ),
        "control_publication_count": int(publish_count),
        "control_publication_channels": list(publish_channels),
    }


def iter_hitter_hil_trace_records(path) -> Iterable[dict]:
    with Path(path).open("r", encoding="utf-8") as handle:
        for line in handle:
            stripped = line.strip()
            if stripped:
                yield json.loads(stripped)
