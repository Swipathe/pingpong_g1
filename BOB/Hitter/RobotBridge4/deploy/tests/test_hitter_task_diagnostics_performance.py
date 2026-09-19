from __future__ import annotations

from collections import deque
import math
import os
from pathlib import Path
import struct
import tempfile
import threading
import time
from types import SimpleNamespace
from typing import Deque
import unittest
from urllib import request as urllib_request

import numpy as np

from diagnostics.hitter_task_events import DiagnosticState, EventHub
from diagnostics.hitter_task_models import (
    EventDraft,
    HealthSnapshot,
    LifecycleSnapshot,
    NormalizedMocapSample,
)
from diagnostics.hitter_task_monitor import MonitorOptions, build_monitor
from diagnostics.hitter_task_recording import RawRecordLane


DEPLOY_ROOT = Path(__file__).resolve().parents[1]
PERF_SAMPLE_RATE_HZ = 360
PERF_DURATION_S = 60
PERF_SAMPLE_COUNT = PERF_SAMPLE_RATE_HZ * PERF_DURATION_S
HANDLER_BUDGET_MS = 1000.0 / PERF_SAMPLE_RATE_HZ
STATE_GET_PERIOD_S = 0.11
MAX_RSS_GROWTH_BYTES = 64 * 1024 * 1024


def _sample(sequence: int) -> NormalizedMocapSample:
    return NormalizedMocapSample(
        input_seq=sequence,
        channel="vicon_state_data",
        subject="ball",
        position_w=np.array([1.5, 0.0, 1.2], dtype=np.float64),
        quaternion_xyzw=np.array(
            [0.0, 0.0, 0.0, 1.0],
            dtype=np.float64,
        ),
        valid=True,
        occluded=False,
        source_frame=sequence,
        source_time_s=sequence / 360.0,
        publish_time_us=sequence * 2778,
        received_monotonic_s=sequence / 360.0,
        wall_time_us=sequence * 2778,
        payload_size=4,
    )


def _initial_state() -> DiagnosticState:
    return DiagnosticState(
        schema_version=1,
        watermark_event_id=0,
        health=HealthSnapshot(
            lcm_connected=True,
            message_rate_hz_by_subject={"ball": 360.0},
            message_age_s_by_subject={"ball": 0.0},
            source_frame_by_subject={"ball": 0},
            pelvis_valid=True,
            pelvis_age_s=0.0,
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
            session_basename="performance-test",
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


def _percentile_ms(values_s, quantile: float) -> float:
    ordered = sorted(float(value) for value in values_s)
    if not ordered:
        raise AssertionError("no handler timings were recorded")
    index = max(0, int(math.ceil(quantile * len(ordered))) - 1)
    return ordered[index] * 1000.0


def _rss_bytes() -> int:
    statm = Path("/proc/self/statm")
    if statm.exists():
        resident_pages = int(statm.read_text(encoding="ascii").split()[1])
        return resident_pages * int(os.sysconf("SC_PAGE_SIZE"))
    return 0


def _performance_options(root: Path) -> MonitorOptions:
    table = root / "table.json"
    pelvis = root / "pelvis.json"
    table.write_text('{"fixture":"table"}\n', encoding="utf-8")
    pelvis.write_text('{"fixture":"pelvis"}\n', encoding="utf-8")
    return MonitorOptions(
        mimic_config=DEPLOY_ROOT / "config/mimic/hitter.yaml",
        control_config=(
            DEPLOY_ROOT / "config/control/g1_hitter_racket.yaml"
        ),
        table_calib=table,
        pelvis_calib=pelvis,
        lcm_url="memq://diagnostic-performance-test",
        channel="vicon_state_data",
        base_name="G2Pelvis",
        ball_name="ball",
        port=0,
        output_dir=root / "recordings",
        duration_s=0.0,
        reacquire_grace_s=0.20,
        heartbeat_stale_s=0.50,
        pelvis_stale_s=0.05,
        source_frame_gap_threshold=100,
        raw_capacity=65536,
        event_capacity=4096,
        attempt_cache_size=100,
    )


class _TimedNoPublishTransport:
    def __init__(self) -> None:
        self._read_fd, self._write_fd = os.pipe()
        self._pending: Deque[bytes] = deque()
        self._condition = threading.Condition(threading.Lock())
        self._callback = None
        self._subscription = object()
        self.handler_durations_s = []
        self.published_messages = []
        self.handled = 0

    def fileno(self) -> int:
        return self._read_fd

    def subscribe(self, channel, callback):
        self.channel = channel
        self._callback = callback
        return self._subscription

    def send(self, payload: bytes) -> None:
        with self._condition:
            self._pending.append(bytes(payload))
        os.write(self._write_fd, b"x")

    def handle(self) -> int:
        os.read(self._read_fd, 1)
        with self._condition:
            payload = self._pending.popleft()
        if self._callback is None:
            raise RuntimeError("transport was not subscribed")
        started = time.perf_counter()
        self._callback(self.channel, payload)
        elapsed = time.perf_counter() - started
        with self._condition:
            self.handler_durations_s.append(elapsed)
            self.handled += 1
            self._condition.notify_all()
        return 0

    def publish(self, channel, payload) -> None:
        self.published_messages.append((channel, payload))
        raise AssertionError("diagnostics must never publish LCM messages")

    def unsubscribe(self, subscription) -> None:
        if subscription is not self._subscription:
            raise AssertionError("wrong subscription")

    def wait_handled(self, count: int, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while self.handled < count:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False
                self._condition.wait(remaining)
            return True

    def close(self) -> None:
        for descriptor in (self._read_fd, self._write_fd):
            try:
                os.close(descriptor)
            except OSError:
                pass


def _decode_performance_message(payload: bytes):
    frame = struct.unpack("!I", payload)[0]
    pelvis = frame % 12 == 0
    cycle = frame % 720
    return SimpleNamespace(
        name="G2Pelvis" if pelvis else "ball",
        vicon_frame_number=frame + 1,
        vicon_time_s=10.0 + frame / 360.0,
        publish_time_us=10_000_000 + int(frame / 360.0 * 1.0e6),
        valid=1,
        occluded=0,
        pos_vicon=(
            [0.0, 0.0, 0.80]
            if pelvis
            else [2.20 - 0.0025 * cycle, 0.0, 1.20]
        ),
        quat_vicon=[0.0, 0.0, 0.0, 1.0],
    )


class HitterTaskDiagnosticsPerformanceTest(unittest.TestCase):
    def test_default_raw_and_event_buffers_remain_bounded_under_burst(self):
        raw = RawRecordLane(capacity=64)
        for sequence in range(4096):
            raw.submit(_sample(sequence), attempt_id=1)
        self.assertEqual(raw.pending_count(), 64)

        max_ring_bytes = 128 * 1024
        hub = EventHub(
            _initial_state(),
            capacity=64,
            max_event_bytes=2048,
            max_ring_bytes=max_ring_bytes,
        )
        for sequence in range(4096):
            hub.publish(
                EventDraft(
                    kind="PERF_SAMPLE",
                    monotonic_s=sequence / 360.0,
                    wall_time_us=sequence,
                    scope="state",
                    attempt_id=None,
                    payload={"sequence": sequence},
                )
            )
        self.assertEqual(hub.state_snapshot().watermark_event_id, 4096)
        self.assertLessEqual(hub.ring_bytes(), max_ring_bytes)
        self.assertEqual(hub.open_cursor(0).bootstrap.mode, "reset")
        hub.close()

    @unittest.skipUnless(
        os.environ.get("HITTER_DIAGNOSTICS_PERF") == "1",
        "set HITTER_DIAGNOSTICS_PERF=1 for the 60-second acceptance run",
    )
    def test_60_second_360hz_live_monitor_acceptance(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            transport = _TimedNoPublishTransport()
            monitor = build_monitor(
                _performance_options(Path(temp_dir)),
                lcm_factory=lambda _url: transport,
                decoder=_decode_performance_message,
                argv=("diagnostic-performance-test",),
            )
            poll_stop = threading.Event()
            poll_times = []
            poll_errors = []

            def poll_state() -> None:
                url = "{}api/state".format(monitor.page_url)
                while not poll_stop.wait(STATE_GET_PERIOD_S):
                    try:
                        with urllib_request.urlopen(url, timeout=1.0) as response:
                            response.read()
                        poll_times.append(time.monotonic())
                    except Exception as exc:
                        poll_errors.append(
                            "{}: {}".format(type(exc).__name__, str(exc))
                        )

            poll_thread = threading.Thread(
                target=poll_state,
                name="hitter-performance-browser",
                daemon=True,
            )
            rss_start = 0
            rss_mid = 0
            rss_end = 0
            test_started = 0.0
            test_ended = 0.0
            try:
                monitor.start()
                poll_thread.start()
                rss_start = _rss_bytes()
                test_started = time.monotonic()
                for frame in range(PERF_SAMPLE_COUNT):
                    deadline = test_started + frame / PERF_SAMPLE_RATE_HZ
                    remaining = deadline - time.monotonic()
                    if remaining > 0.0:
                        time.sleep(remaining)
                    transport.send(struct.pack("!I", frame))
                    if frame == PERF_SAMPLE_COUNT // 2:
                        rss_mid = _rss_bytes()
                self.assertTrue(
                    transport.wait_handled(PERF_SAMPLE_COUNT, timeout_s=5.0)
                )
                drain_deadline = time.monotonic() + 10.0
                while (
                    monitor.input_pending_count > 0
                    and time.monotonic() < drain_deadline
                ):
                    time.sleep(0.005)
                self.assertEqual(monitor.input_pending_count, 0)
                self.assertEqual(
                    monitor.adapter._next_input_seq,
                    PERF_SAMPLE_COUNT,
                )
                test_ended = time.monotonic()
                rss_end = _rss_bytes()

                durations = tuple(transport.handler_durations_s)
                p50_ms = _percentile_ms(durations, 0.50)
                p95_ms = _percentile_ms(durations, 0.95)
                p99_ms = _percentile_ms(durations, 0.99)
                elapsed_s = test_ended - test_started
                get_rate_hz = len(poll_times) / elapsed_s
                input_dropped = monitor.input_samples_dropped
                raw_dropped = (
                    int(monitor.raw_counter.dropped) + input_dropped
                )
                state = monitor.event_hub.state_snapshot()
                event_dropped = int(
                    state.health.diagnostic_events_dropped
                )
                rss_growth = max(0, rss_end - rss_start)
                first_half_growth = max(0, rss_mid - rss_start)
                second_half_growth = max(0, rss_end - rss_mid)

                print(
                    "HITTER_DIAGNOSTICS_PERF "
                    "samples={} elapsed_s={:.3f} "
                    "handler_p50_ms={:.3f} "
                    "handler_p95_ms={:.3f} "
                    "handler_p99_ms={:.3f} "
                    "input_drop={} raw_drop={} event_drop={} "
                    "rss_growth_bytes={} "
                    "rss_first_half_bytes={} "
                    "rss_second_half_bytes={} "
                    "state_get_count={} state_get_rate_hz={:.3f}".format(
                        len(durations),
                        elapsed_s,
                        p50_ms,
                        p95_ms,
                        p99_ms,
                        input_dropped,
                        raw_dropped,
                        event_dropped,
                        rss_growth,
                        first_half_growth,
                        second_half_growth,
                        len(poll_times),
                        get_rate_hz,
                    )
                )

                self.assertEqual(len(durations), PERF_SAMPLE_COUNT)
                self.assertLess(p99_ms, HANDLER_BUDGET_MS)
                self.assertEqual(input_dropped, 0)
                self.assertEqual(raw_dropped, 0)
                self.assertEqual(event_dropped, 0)
                self.assertFalse(poll_errors, poll_errors[:3])
                self.assertLessEqual(get_rate_hz, 10.0)
                self.assertLessEqual(rss_growth, MAX_RSS_GROWTH_BYTES)
                self.assertLessEqual(
                    second_half_growth,
                    max(
                        16 * 1024 * 1024,
                        first_half_growth + 8 * 1024 * 1024,
                    ),
                )
                self.assertLessEqual(
                    monitor.event_hub.ring_bytes(),
                    64 * 1024 * 1024,
                )
                self.assertEqual(transport.published_messages, [])
            finally:
                poll_stop.set()
                poll_thread.join(2.0)
                monitor.close(timeout_s=5.0)
                transport.close()
            self.assertFalse(poll_thread.is_alive())
            self.assertFalse(monitor.background_threads_alive)


if __name__ == "__main__":
    unittest.main()
