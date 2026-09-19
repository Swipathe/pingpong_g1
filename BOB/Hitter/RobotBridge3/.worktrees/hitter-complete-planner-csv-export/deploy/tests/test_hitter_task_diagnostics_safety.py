from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import multiprocessing
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
from typing import Deque, Optional
import unittest

import numpy as np

from diagnostics.hitter_task_monitor import (
    MonitorOptions,
    build_monitor,
)


DEPLOY_ROOT = Path(__file__).resolve().parents[1]


def _wait_until(predicate, *, timeout_s: float = 2.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return bool(predicate())


def _options(root: Path) -> MonitorOptions:
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
        lcm_url="memq://diagnostic-safety-test",
        channel="vicon_state_data",
        base_name="G1Pelvis",
        ball_name="ball",
        port=0,
        output_dir=root / "recordings",
        duration_s=0.0,
        reacquire_grace_s=0.20,
        heartbeat_stale_s=0.50,
        pelvis_stale_s=0.05,
        source_frame_gap_threshold=1,
        raw_capacity=256,
        event_capacity=256,
        attempt_cache_size=16,
    )


class _NoPublishTransport:
    """Selectable input transport whose publish path is a hard tripwire."""

    def __init__(self) -> None:
        self._read_fd, self._write_fd = os.pipe()
        self._pending: Deque[bytes] = deque()
        self._lock = threading.Lock()
        self._callback = None
        self._subscription = object()
        self.published_messages = []
        self.unsubscribed = False

    def fileno(self) -> int:
        return self._read_fd

    def subscribe(self, channel, callback):
        self.channel = channel
        self._callback = callback
        return self._subscription

    def send(self, payload: bytes) -> None:
        with self._lock:
            self._pending.append(bytes(payload))
        os.write(self._write_fd, b"x")

    def handle(self) -> int:
        os.read(self._read_fd, 1)
        with self._lock:
            payload = self._pending.popleft()
        if self._callback is None:
            raise RuntimeError("transport was not subscribed")
        self._callback(self.channel, payload)
        return 0

    def publish(self, channel, payload) -> None:
        self.published_messages.append((channel, payload))
        raise AssertionError("diagnostics must never publish LCM messages")

    def unsubscribe(self, subscription) -> None:
        if subscription is not self._subscription:
            raise AssertionError("wrong subscription")
        self.unsubscribed = True

    def close(self) -> None:
        for descriptor in (self._read_fd, self._write_fd):
            try:
                os.close(descriptor)
            except OSError:
                pass


def _decoded_message(payload: bytes):
    frame = int(payload.decode("ascii"))
    pelvis = frame == 0
    return SimpleNamespace(
        name="G1Pelvis" if pelvis else "ball",
        vicon_frame_number=max(1, frame),
        vicon_time_s=1.0 + frame / 360.0,
        publish_time_us=1_000_000 + int(frame / 360.0 * 1.0e6),
        valid=1,
        occluded=0,
        pos_vicon=(
            [0.0, 0.0, 0.80]
            if pelvis
            else [1.00 - 0.003 * frame, 0.0, 1.20]
        ),
        quat_vicon=[0.0, 0.0, 0.0, 1.0],
    )


class _DrainFailingRecorder:
    def __init__(self, delegate) -> None:
        self._delegate = delegate
        self.drain_calls = 0

    def start(self) -> None:
        self._delegate.start()

    def status(self):
        return self._delegate.status()

    def drain(self, timeout_s: float):
        del timeout_s
        self.drain_calls += 1
        raise OSError("synthetic recorder storage failure")

    def request_stop(self) -> None:
        self._delegate.request_stop()

    def write_terminal_and_close(self, *, terminal_session, timeout_s):
        return self._delegate.write_terminal_and_close(
            terminal_session=terminal_session,
            timeout_s=timeout_s,
        )

    def offer_attempt_detail(self, detail) -> bool:
        return self._delegate.offer_attempt_detail(detail)


class _OfferFailingEventLane:
    def __init__(self, delegate) -> None:
        self._delegate = delegate
        self.offer_calls = 0

    def __getattr__(self, name):
        return getattr(self._delegate, name)

    def offer(self, draft) -> bool:
        del draft
        self.offer_calls += 1
        raise RuntimeError("synthetic event client failure")

    def drain(self, *, timeout_s: float) -> bool:
        return self._delegate.drain(timeout_s=timeout_s)

    def close(self, *, timeout_s: float) -> bool:
        return self._delegate.close(timeout_s=timeout_s)


@dataclass(frozen=True)
class _StrikePlan:
    t_strike: float
    p_racket_target: np.ndarray
    v_racket_target: np.ndarray


@dataclass(frozen=True)
class _PlannerCommand:
    strike_type: str
    p_base_target_xy: np.ndarray
    v_racket_target_w: np.ndarray
    time_to_strike: float
    strike_plan: _StrikePlan


class _DeterministicPlanner:
    def plan_command(
        self,
        ball_position,
        ball_velocity,
        *,
        current_base_xy_w,
        base_forward_xy_w,
        strike_type,
    ):
        del (
            ball_position,
            ball_velocity,
            current_base_xy_w,
            base_forward_xy_w,
            strike_type,
        )
        return _PlannerCommand(
            strike_type="forehand",
            p_base_target_xy=np.array([-0.4, 0.2], dtype=np.float64),
            v_racket_target_w=np.array([4.0, 5.0, 6.0], dtype=np.float64),
            time_to_strike=0.85,
            strike_plan=_StrikePlan(
                t_strike=0.85,
                p_racket_target=np.array(
                    [0.1, 0.2, 1.1],
                    dtype=np.float64,
                ),
                v_racket_target=np.array(
                    [4.0, 5.0, 6.0],
                    dtype=np.float64,
                ),
            ),
        )


class HitterTaskDiagnosticsSafetyTest(unittest.TestCase):
    def test_import_and_build_do_not_load_policy_or_control_runtime(self):
        script = """
import json
import sys
from diagnostics.hitter_task_monitor import build_monitor
del build_monitor
forbidden = (
    "onnxruntime",
    "agents.hitter_agent",
    "simulator.real_world",
    "envs.base_env",
    "envs.hitter",
)
print(json.dumps([name for name in forbidden if name in sys.modules]))
"""
        environment = dict(os.environ)
        environment["PYTHONPATH"] = str(DEPLOY_ROOT)
        completed = subprocess.run(
            [sys.executable, "-c", script],
            cwd=str(DEPLOY_ROOT),
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=10.0,
            check=True,
        )
        self.assertEqual(json.loads(completed.stdout), [])

    def test_fake_lcm_publish_is_never_called_and_shutdown_releases_workers(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            transport = _NoPublishTransport()
            baseline_children = {
                child.pid for child in multiprocessing.active_children()
            }
            monitor = build_monitor(
                _options(Path(temp_dir)),
                lcm_factory=lambda _url: transport,
                decoder=_decoded_message,
                argv=("diagnostic-safety-test",),
            )
            monitor.pipeline.planner = _DeterministicPlanner()
            host, port = monitor.address
            try:
                monitor.start()
                feed_started = time.monotonic()
                for frame in range(40):
                    deadline = feed_started + frame * 0.011
                    remaining = deadline - time.monotonic()
                    if remaining > 0.0:
                        time.sleep(remaining)
                    transport.send(str(frame).encode("ascii"))
                    self.assertTrue(
                        _wait_until(
                            lambda: (
                                monitor.adapter._next_input_seq
                                >= frame + 1
                            )
                        )
                    )
                    snapshot = monitor.adapter.latest_ball_snapshot
                    if snapshot is None or not snapshot.ready:
                        continue
                    generation = snapshot.generation
                    monitor.pipeline.worker.submit(snapshot)
                    self.assertTrue(
                        _wait_until(
                            lambda: (
                                monitor.pipeline.worker.latest_result_bundle()[
                                    1
                                ]
                                is not None
                                and monitor.pipeline.worker.latest_result_bundle()[
                                    1
                                ].snapshot_key.generation
                                == generation
                            )
                        )
                    )
                    monitor._tick_once()
                    summary = (
                        monitor.pipeline.attempt_tracker.current_summary()
                    )
                    if (
                        summary is not None
                        and summary.task_obs_status == "PASS"
                    ):
                        break
                self.assertEqual(
                    monitor.pipeline.attempt_tracker.current_summary().task_obs_status,
                    "PASS",
                )
                self.assertEqual(transport.published_messages, [])
                self.assertIsNone(monitor._failure_snapshot())
                self.assertTrue(monitor.close(timeout_s=3.0))

                threads = (
                    monitor._lcm_thread,
                    monitor._input_thread,
                    monitor._tick_thread,
                    monitor._recorder_thread,
                    monitor.pipeline.worker._thread,
                    monitor.event_lane._thread,
                    monitor.web_server._accept_thread,
                )
                self.assertTrue(
                    all(thread is None or not thread.is_alive() for thread in threads)
                )
                self.assertFalse(monitor.background_threads_alive)
                remaining_children = {
                    child.pid for child in multiprocessing.active_children()
                }
                self.assertEqual(
                    remaining_children.difference(baseline_children),
                    set(),
                )

                reusable = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                try:
                    reusable.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    reusable.bind((host, port))
                finally:
                    reusable.close()
            finally:
                monitor.close(timeout_s=1.0)
                transport.close()

    def test_recorder_drain_failure_does_not_stop_core_ticks(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            transport = _NoPublishTransport()
            monitor = build_monitor(
                _options(Path(temp_dir)),
                lcm_factory=lambda _url: transport,
                decoder=_decoded_message,
            )
            failing = _DrainFailingRecorder(monitor.recorder)
            monitor.recorder = failing
            try:
                monitor.start()
                self.assertTrue(
                    _wait_until(
                        lambda: failing.drain_calls >= 2
                        and monitor._last_tick_result is not None,
                    )
                )
                first_tick = monitor._last_tick_result.obs_now_s
                self.assertTrue(
                    _wait_until(
                        lambda: (
                            monitor._last_tick_result is not None
                            and monitor._last_tick_result.obs_now_s > first_tick
                        )
                    )
                )
                self.assertIsNone(monitor._failure_snapshot())
            finally:
                self.assertFalse(monitor.close(timeout_s=2.0))
                transport.close()
            self.assertFalse(monitor.background_threads_alive)
            self.assertEqual(transport.published_messages, [])

    def test_event_lane_failure_does_not_stop_core_ticks(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            transport = _NoPublishTransport()
            monitor = build_monitor(
                _options(Path(temp_dir)),
                lcm_factory=lambda _url: transport,
                decoder=_decoded_message,
            )
            failing = _OfferFailingEventLane(monitor.event_lane)
            monitor.event_lane = failing
            try:
                monitor.start()
                self.assertTrue(
                    _wait_until(
                        lambda: failing.offer_calls >= 6
                        and monitor._last_tick_result is not None,
                    )
                )
                first_tick = monitor._last_tick_result.obs_now_s
                self.assertTrue(
                    _wait_until(
                        lambda: (
                            monitor._last_tick_result is not None
                            and monitor._last_tick_result.obs_now_s > first_tick
                        )
                    )
                )
                self.assertIsNone(monitor._failure_snapshot())
                self.assertTrue(monitor.close(timeout_s=2.0))
            finally:
                monitor.close(timeout_s=1.0)
                transport.close()
            self.assertFalse(monitor.background_threads_alive)
            self.assertEqual(transport.published_messages, [])

    def test_disconnected_sse_client_does_not_stop_state_progress(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            transport = _NoPublishTransport()
            monitor = build_monitor(
                _options(Path(temp_dir)),
                lcm_factory=lambda _url: transport,
                decoder=_decoded_message,
            )
            client: Optional[socket.socket] = None
            try:
                monitor.start()
                host, port = monitor.address
                self.assertTrue(
                    _wait_until(
                        lambda: (
                            monitor.event_hub.state_snapshot().watermark_event_id
                            >= 2
                        )
                    )
                )
                client = socket.create_connection((host, port), timeout=2.0)
                client.settimeout(2.0)
                request = (
                    "GET /events HTTP/1.1\r\n"
                    "Host: {}:{}\r\n"
                    "Accept: text/event-stream\r\n"
                    "\r\n"
                ).format(host, port)
                client.sendall(request.encode("ascii"))
                response = client.recv(4096)
                self.assertIn(b"200 OK", response)
                client.setsockopt(
                    socket.SOL_SOCKET,
                    socket.SO_LINGER,
                    struct.pack("ii", 1, 0),
                )
                client.close()
                client = None

                watermark = (
                    monitor.event_hub.state_snapshot().watermark_event_id
                )
                self.assertTrue(
                    _wait_until(
                        lambda: (
                            monitor.event_hub.state_snapshot().watermark_event_id
                            > watermark + 2
                        )
                    )
                )
                self.assertIsNone(monitor._failure_snapshot())
                self.assertTrue(monitor.close(timeout_s=2.0))
            finally:
                if client is not None:
                    client.close()
                monitor.close(timeout_s=1.0)
                transport.close()
            self.assertFalse(monitor.background_threads_alive)
            self.assertEqual(transport.published_messages, [])


if __name__ == "__main__":
    unittest.main()
