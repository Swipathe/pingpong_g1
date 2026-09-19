from __future__ import annotations

from dataclasses import dataclass, replace
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest

from diagnostics.hitter_task_models import (
    AttemptBinding,
    AttemptDetail,
    AttemptSummary,
    AttemptTransition,
    BallDiagnosticState,
    EventDraft,
    SnapshotKey,
    SubjectHealth,
)
from diagnostics.hitter_task_events import EventHub
from diagnostics.hitter_task_attempts import AttemptTracker
from diagnostics.hitter_task_monitor import (
    HitterTaskMonitor,
    MonitorOptions,
    _initial_state,
    _parser,
    build_monitor,
    default_output_dir,
)
from diagnostics.hitter_task_replay import (
    OnlineAttemptTrace,
    ReplayInputBundle,
    ReplayProcessController,
    ReplayWorkerReply,
    run_replay_disk_job,
)


DEPLOY_ROOT = Path(__file__).resolve().parents[1]


def _options(
    root: Path,
    *,
    duration_s: float = 0.0,
    input_capacity: int = 65536,
) -> MonitorOptions:
    return MonitorOptions(
        mimic_config=DEPLOY_ROOT / "config/mimic/hitter.yaml",
        control_config=DEPLOY_ROOT / "config/control/g1_hitter_racket.yaml",
        table_calib=root / "table.json",
        pelvis_calib=root / "pelvis.json",
        lcm_url="memq://diagnostic-test",
        channel="vicon_state_data",
        base_name="G1Pelvis",
        ball_name="ball",
        port=0,
        output_dir=root / "recordings",
        duration_s=duration_s,
        reacquire_grace_s=0.20,
        heartbeat_stale_s=0.50,
        pelvis_stale_s=0.05,
        source_frame_gap_threshold=1,
        raw_capacity=64,
        event_capacity=64,
        attempt_cache_size=8,
        input_capacity=input_capacity,
    )


class _FakeTransport:
    def __init__(self, log=None):
        self._read_fd, self._write_fd = os.pipe()
        self._pending = []
        self._lock = threading.Lock()
        self.callback = None
        self.subscription = object()
        self.unsubscribed = False
        self.log = [] if log is None else log

    def fileno(self):
        return self._read_fd

    def subscribe(self, channel, callback):
        self.log.append("lcm.subscribe")
        self.channel = channel
        self.callback = callback
        return self.subscription

    def send(self, payload):
        with self._lock:
            self._pending.append(payload)
        os.write(self._write_fd, b"x")

    def handle(self):
        os.read(self._read_fd, 1)
        with self._lock:
            payload = self._pending.pop(0)
        self.callback(self.channel, payload)
        return 0

    def unsubscribe(self, subscription):
        self.log.append("lcm.unsubscribe")
        self.unsubscribed = True
        self.asserted_subscription = subscription

    def close_fds(self):
        for descriptor in (self._read_fd, self._write_fd):
            try:
                os.close(descriptor)
            except OSError:
                pass


@dataclass(frozen=True)
class _Pelvis:
    valid: bool = False
    received_monotonic_s: float | None = None
    source_frame: int = 0


class _FakeAdapter:
    def __init__(self, log, ingested):
        self.log = log
        self.ingested = ingested
        self.entered = threading.Event()
        self.release = None

    def ingest_decoded(self, **kwargs):
        self.entered.set()
        if self.release is not None:
            self.release.wait(2.0)
        self.log.append("adapter.ingest")
        output = SimpleNamespace(
            sample=SimpleNamespace(
                subject="ball",
                source_frame=17,
                received_monotonic_s=kwargs["received_monotonic_s"],
            ),
            snapshot=None,
            warnings=(),
            estimator_sample_count=4,
        )
        self.ingested.append(kwargs)
        return output

    def copy_latest_pelvis(self):
        return _Pelvis()


class _FakeTracker:
    def __init__(self, summary=None, transitions=()):
        self.summary = summary
        self.transitions = tuple(transitions)

    def advance(self, *, now_monotonic_s):
        del now_monotonic_s
        transitions = self.transitions
        self.transitions = ()
        return transitions

    def current_summary(self):
        return self.summary

    def summary_for_attempt(self, attempt_id):
        if self.summary is not None and self.summary.attempt_id == attempt_id:
            return self.summary
        return None

    def timeline_for_attempt(self, attempt_id):
        del attempt_id
        return ()


class _ClosingTracker:
    def __init__(self):
        self.summary = AttemptSummary(
            attempt_id=7,
            status="FAILED",
            stage="ATTEMPT_CLOSED",
            primary_blocker="TRACK_ENDED_BEFORE_READY",
            ball_speed_mps=None,
            predicted_strike_time_s=None,
            planner_tts_s=None,
            arm_tts_s=None,
            task_obs_status="FAILED",
            ab_summary=None,
            recording_complete=True,
        )
        self.transition = AttemptTransition(
            attempt_id=7,
            track_segment_id=3,
            stage="ATTEMPT_CLOSED",
            monotonic_s=20.0,
            snapshot_key=SnapshotKey(1, 2),
            reason_code="TRACK_ENDED_BEFORE_READY",
            values={"status": "FAILED"},
        )
        self.closed = False

    def advance(self, *, now_monotonic_s):
        del now_monotonic_s
        if self.closed:
            return ()
        self.closed = True
        return (self.transition,)

    def current_summary(self):
        return None if self.closed else self.summary

    def summary_for_attempt(self, attempt_id):
        return self.summary if attempt_id == 7 else None

    def timeline_for_attempt(self, attempt_id):
        return (self.transition,) if attempt_id == 7 else ()


class _FakePipeline:
    def __init__(self, log, tracker=None):
        self.log = log
        self.attempt_tracker = _FakeTracker() if tracker is None else tracker
        self.incoming = SimpleNamespace(
            snapshot=lambda: SimpleNamespace(
                consecutive_count=2,
                confirmed=False,
            )
        )
        self.worker = SimpleNamespace(
            stats=SimpleNamespace(
                submitted=3,
                completed=2,
                failed=0,
                dropped_pending=1,
            )
        )
        self.raw_sink_failures = 0
        self.event_sink_failures = 0
        self.planner_results_overwritten_before_consume = 7
        self.tick_calls = []
        self.ingest_event = threading.Event()

    def ingest_adapter_output(self, output):
        self.log.append("pipeline.ingest")
        self.last_output = output
        self.ingest_event.set()

    def tick(self, **kwargs):
        self.log.append("pipeline.tick")
        self.tick_calls.append(kwargs)
        return SimpleNamespace(
            phase="tracking",
            lifecycle_decision="none",
            active_binding=None,
            cached_binding=None,
            command_result=None,
            command_fields_used=None,
            task_observation=None,
            task_pass=False,
            errors=(),
        )

    def close(self, timeout_s=5.0):
        self.log.append("pipeline.close")
        self.close_timeout_s = timeout_s
        return True


class _FakeRecorder:
    def __init__(self, log):
        self.log = log
        self.stop_requested = False
        self.details = []
        self.replay_job_statuses = []
        self.replay_analyses = []
        self.incomplete_calls = []
        self.recording_complete = True

    def start(self):
        self.log.append("recorder.start")

    def request_stop(self):
        self.log.append("recorder.request_stop")
        self.stop_requested = True

    def drain(self, timeout_s):
        if self.stop_requested:
            self.log.append("recorder.drain")
        self.drain_timeout_s = timeout_s
        return self.status()

    def write_terminal_and_close(self, *, terminal_session, timeout_s):
        self.log.append("recorder.terminal")
        self.terminal_session = dict(terminal_session)
        self.terminal_timeout_s = timeout_s
        return self.status()

    def status(self):
        return SimpleNamespace(
            healthy=True,
            recording_complete=self.recording_complete,
            bytes_written=0,
            last_error=None,
        )

    def mark_incomplete(self, attempt_id, reason):
        self.incomplete_calls.append((attempt_id, reason))
        self.recording_complete = False

    def offer_attempt_detail(self, detail):
        self.detail = detail
        self.details.append(detail)
        return True

    def offer_replay_job_status(self, value):
        self.replay_job_statuses.append(dict(value))
        return True

    def offer_replay_analysis(self, value):
        self.replay_analyses.append(dict(value))
        return True

    def confirm_replay_persistence(self, timeout_s):
        self.replay_persistence_timeout_s = float(timeout_s)
        return self.status()


class _FakeWeb:
    def __init__(self, log):
        self.log = log
        self.address = ("127.0.0.1", 43210)

    def start(self):
        self.log.append("web.start")

    def close(self, *, timeout_s=5.0):
        self.log.append("web.close")
        self.timeout_s = timeout_s
        return True


class _FakeReplay:
    def __init__(self, log):
        self.log = log

    def set_realtime_busy(self, **kwargs):
        self.busy = kwargs

    def poll(self):
        return ()

    def close(self, *, timeout_s=5.0):
        self.log.append("replay.close")
        self.timeout_s = timeout_s
        return True


class _FakeLane:
    def __init__(self, hub=None):
        self.drafts = []
        self.hub = hub

    def offer(self, draft):
        self.drafts.append(draft)
        if self.hub is not None:
            self.hub.publish(draft)
        return True

    def drain(self, *, timeout_s):
        del timeout_s
        return True

    def close(self, *, timeout_s):
        del timeout_s
        return True


class _FakeHub:
    def __init__(self):
        self.health = SimpleNamespace(diagnostic_events_dropped=0)

    def state_snapshot(self):
        return SimpleNamespace(health=self.health)

    def close(self):
        return None


class _FakeRepository:
    def commit(self, detail):
        self.detail = detail

    def get(self, attempt_id):
        detail = getattr(self, "detail", None)
        if detail is not None and detail.attempt_id == attempt_id:
            return detail
        return None


def _constructed_monitor(
    root: Path,
    *,
    duration_s: float = 0.0,
    input_capacity: int = 65536,
    monotonic_fn=time.monotonic,
    event_hub=None,
):
    log = []
    transport = _FakeTransport(log)
    decoder_log = []
    ingested = []

    def decoder(payload):
        decoder_log.append(payload)
        log.append("decoder")
        return SimpleNamespace(name="ball", payload=payload)

    adapter = _FakeAdapter(log, ingested)
    pipeline = _FakePipeline(log)
    recorder = _FakeRecorder(log)
    web = _FakeWeb(log)
    replay = _FakeReplay(log)
    lane = _FakeLane(event_hub)
    hub = _FakeHub() if event_hub is None else event_hub
    monitor = HitterTaskMonitor(
        _options(
            root,
            duration_s=duration_s,
            input_capacity=input_capacity,
        ),
        lcm_client=transport,
        decoder=decoder,
        adapter=adapter,
        pipeline=pipeline,
        recorder=recorder,
        web_server=web,
        event_lane=lane,
        event_hub=hub,
        attempt_repository=_FakeRepository(),
        session_basename="session-test",
        control_tick_s=0.02,
        estimator_window_size=31,
        replay_controller=replay,
        monotonic_fn=monotonic_fn,
        wall_time_fn=lambda: 1234.5,
    )
    return (
        monitor,
        log,
        transport,
        decoder_log,
        ingested,
        pipeline,
        recorder,
        web,
        lane,
    )


class HitterTaskMonitorTest(unittest.TestCase):
    def test_default_heartbeat_threshold_is_one_tenth_second(self):
        self.assertAlmostEqual(
            _parser().parse_args([]).heartbeat_stale_s,
            0.10,
        )

    def test_default_output_dir_is_repo_relative_not_cwd_relative(self):
        expected = (
            Path(__file__).resolve().parents[2]
            / "recordings"
            / "hitter_task_diagnostics"
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            previous = Path.cwd()
            try:
                os.chdir(temp_dir)
                self.assertEqual(default_output_dir(), expected)
            finally:
                os.chdir(previous)

    def test_fd_owner_decodes_and_ingests_in_exact_arrival_order(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            (
                monitor,
                log,
                transport,
                decoder_log,
                ingested,
                pipeline,
                _recorder,
                _web,
                _lane,
            ) = _constructed_monitor(Path(temp_dir))
            try:
                monitor.start()
                transport.send(b"first-packet")
                self.assertTrue(pipeline.ingest_event.wait(1.0))
                ordered = [
                    item
                    for item in log
                    if item in ("decoder", "adapter.ingest", "pipeline.ingest")
                ]
                self.assertEqual(
                    ordered[:3],
                    ["decoder", "adapter.ingest", "pipeline.ingest"],
                )
                self.assertEqual(decoder_log, [b"first-packet"])
                self.assertEqual(ingested[0]["channel"], "vicon_state_data")
                self.assertEqual(ingested[0]["payload_size"], 12)
                self.assertEqual(ingested[0]["wall_time_us"], 1234500000)
            finally:
                monitor.close(timeout_s=1.0)
                transport.close_fds()

    def test_slow_adapter_never_blocks_lcm_decode_and_fifo_is_preserved(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            (
                monitor,
                _log,
                transport,
                decoder_log,
                ingested,
                pipeline,
                _recorder,
                _web,
                _lane,
            ) = _constructed_monitor(Path(temp_dir))
            release = threading.Event()
            monitor.adapter.release = release
            try:
                monitor.start()
                transport.send(b"first")
                self.assertTrue(monitor.adapter.entered.wait(1.0))
                transport.send(b"second")
                transport.send(b"third")
                deadline = time.monotonic() + 1.0
                while len(decoder_log) < 3 and time.monotonic() < deadline:
                    time.sleep(0.001)
                self.assertEqual(
                    decoder_log,
                    [b"first", b"second", b"third"],
                )
                self.assertFalse(pipeline.ingest_event.is_set())
                self.assertEqual(monitor.input_pending_count, 3)

                release.set()
                deadline = time.monotonic() + 1.0
                while len(ingested) < 3 and time.monotonic() < deadline:
                    time.sleep(0.001)
                self.assertEqual(
                    [item["message"].payload for item in ingested],
                    [b"first", b"second", b"third"],
                )
                self.assertEqual(monitor.input_pending_count, 0)
                self.assertEqual(monitor.input_samples_dropped, 0)
            finally:
                release.set()
                monitor.close(timeout_s=1.0)
                transport.close_fds()

    def test_full_input_fifo_is_explicitly_incomplete_without_stopping_lcm(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            (
                monitor,
                _log,
                transport,
                decoder_log,
                _ingested,
                pipeline,
                recorder,
                _web,
                _lane,
            ) = _constructed_monitor(
                Path(temp_dir),
                input_capacity=1,
            )
            release = threading.Event()
            monitor.adapter.release = release
            pipeline.attempt_tracker.summary = AttemptSummary(
                attempt_id=11,
                status="ACTIVE",
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
            try:
                monitor.start()
                transport.send(b"inflight")
                self.assertTrue(monitor.adapter.entered.wait(1.0))
                transport.send(b"queued")
                transport.send(b"dropped")
                deadline = time.monotonic() + 1.0
                while len(decoder_log) < 3 and time.monotonic() < deadline:
                    time.sleep(0.001)
                self.assertEqual(len(decoder_log), 3)
                self.assertEqual(monitor.input_pending_count, 2)
                self.assertEqual(monitor.input_samples_dropped, 1)
                self.assertFalse(monitor.input_recording_complete)
                self.assertEqual(
                    recorder.incomplete_calls,
                    [(11, "INPUT_SAMPLES_DROPPED")],
                )
                self.assertIsNone(monitor._failure_snapshot())
            finally:
                release.set()
                monitor.close(timeout_s=1.0)
                transport.close_fds()

    def test_close_drains_decoded_fifo_before_pipeline_close(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            (
                monitor,
                log,
                transport,
                _decoder_log,
                ingested,
                _pipeline,
                _recorder,
                _web,
                _lane,
            ) = _constructed_monitor(Path(temp_dir))
            try:
                monitor.start()
                for payload in (b"one", b"two", b"three"):
                    monitor._handle_lcm("vicon_state_data", payload)
                self.assertTrue(monitor.close(timeout_s=1.0))
                self.assertEqual(
                    [item["message"].payload for item in ingested],
                    [b"one", b"two", b"three"],
                )
                self.assertLess(
                    max(
                        index
                        for index, item in enumerate(log)
                        if item == "pipeline.ingest"
                    ),
                    log.index("pipeline.close"),
                )
            finally:
                monitor.close(timeout_s=0.0)
                transport.close_fds()

    def test_zero_message_duration_stops_unsubscribes_and_joins(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            (
                monitor,
                _log,
                transport,
                _decoder_log,
                _ingested,
                _pipeline,
                _recorder,
                _web,
                _lane,
            ) = _constructed_monitor(Path(temp_dir), duration_s=0.2)
            started = time.monotonic()
            try:
                self.assertEqual(monitor.run(), 0)
            finally:
                transport.close_fds()
            elapsed = time.monotonic() - started
            self.assertLess(elapsed, 1.5)
            self.assertTrue(transport.unsubscribed)
            self.assertFalse(monitor.background_threads_alive)

    def test_tick_reads_semantic_monotonic_twice_and_projects_stage(self):
        values = iter((10.0, 10.125))
        with tempfile.TemporaryDirectory() as temp_dir:
            (
                monitor,
                _log,
                transport,
                _decoder_log,
                _ingested,
                pipeline,
                _recorder,
                _web,
                lane,
            ) = _constructed_monitor(
                Path(temp_dir),
                monotonic_fn=lambda: next(values),
            )
            pipeline.attempt_tracker.summary = AttemptSummary(
                attempt_id=1,
                status="ACTIVE",
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
            monitor._latest_estimator_sample_count = 31
            try:
                monitor._tick_once()
            finally:
                monitor.close(timeout_s=0.0)
                transport.close_fds()
            self.assertEqual(
                pipeline.tick_calls[0]["lifecycle_now_s"],
                10.0,
            )
            self.assertEqual(
                pipeline.tick_calls[0]["obs_now_s"],
                10.125,
            )
            current = next(
                draft
                for draft in lane.drafts
                if draft.kind == "ATTEMPT_CURRENT"
            )
            self.assertEqual(
                current.payload["attempt"]["stage"],
                "PLANNER_PELVIS_UNAVAILABLE",
            )
            self.assertEqual(
                (
                    current.payload["attempt"][
                        "estimator_sample_count"
                    ],
                    current.payload["attempt"][
                        "estimator_window_size"
                    ],
                    current.payload["attempt"]["incoming_count"],
                    current.payload["attempt"][
                        "incoming_required_count"
                    ],
                ),
                (None, 31, None, None),
            )
            self.assertTrue(
                any(draft.kind == "HEALTH_SNAPSHOT" for draft in lane.drafts)
            )
            self.assertTrue(
                any(
                    draft.kind == "LIFECYCLE_SNAPSHOT"
                    for draft in lane.drafts
                )
            )

    def test_tick_publishes_one_coherent_v2_live_snapshot_without_pelvis(self):
        values = iter((10.0, 10.125))
        hub = EventHub(
            _initial_state(
                config_name="hitter",
                session_basename="session-test",
                subjects=("ball", "g1pelvis", "table"),
            )
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            (
                monitor,
                _log,
                transport,
                _decoder_log,
                _ingested,
                pipeline,
                _recorder,
                _web,
                lane,
            ) = _constructed_monitor(
                Path(temp_dir),
                monotonic_fn=lambda: next(values),
                event_hub=hub,
            )
            pipeline.attempt_tracker.summary = AttemptSummary(
                attempt_id=1,
                status="ACTIVE",
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
            diagnostic_reads = []
            diagnostic_state = BallDiagnosticState(
                status="READY",
                estimator_sample_count=31,
                estimator_window_size=31,
                speed_mps=0.45,
                velocity_world_mps=(-0.45, 0.0, 0.0),
                incoming_count=3,
                incoming_required_count=3,
                incoming_status="INCOMING",
                blocker=None,
            )

            def diagnostic_snapshot():
                diagnostic_reads.append(True)
                return diagnostic_state

            pipeline.ball_diagnostics = SimpleNamespace(
                snapshot=diagnostic_snapshot
            )
            pipeline.incoming = SimpleNamespace(
                snapshot=lambda: SimpleNamespace(
                    track_epoch=None,
                    consecutive_count=0,
                    confirmed=False,
                )
            )
            monitor.runtime_settings = SimpleNamespace(
                incoming_confirmation_snapshots=3,
                arm_tts_s=0.35,
            )
            monitor._arrival.record(
                subject="ball",
                received_monotonic_s=10.10,
                source_frame=77,
            )
            try:
                monitor._tick_once()
            finally:
                monitor.close(timeout_s=0.0)
                transport.close_fds()

        state = hub.state_snapshot().to_json_dict()
        self.assertIn("live_snapshot", state)
        live = state["live_snapshot"]
        self.assertEqual(live["schema_version"], 2)
        self.assertEqual(live["revision"], 1)
        self.assertEqual(live["captured_monotonic_s"], 10.125)
        self.assertEqual(live["subjects"]["ball"]["status"], "LIVE")
        self.assertEqual(
            live["subjects"]["g1pelvis"]["status"],
            "NEVER_SEEN",
        )
        self.assertEqual(live["ball"]["status"], "READY")
        self.assertEqual(
            (
                live["ball"]["estimator_sample_count"],
                live["ball"]["estimator_window_size"],
                live["ball"]["ball_only_incoming_count"],
                live["ball"]["ball_only_incoming_required"],
            ),
            (31, 31, 3, 3),
        )
        gate = live["production_gate"]
        self.assertEqual(gate["pelvis_status"], "BLOCKED")
        self.assertEqual(
            gate["production_incoming_status"],
            "NOT_EVALUATED",
        )
        self.assertIsNone(gate["production_incoming_count"])
        self.assertEqual(gate["planner_status"], "BLOCKED")
        self.assertEqual(
            gate["planner_reason_code"],
            "PELVIS_UNAVAILABLE",
        )
        self.assertIsNone(gate["planner_tts_s"])
        self.assertEqual(gate["arm_status"], "NOT_EVALUATED")
        self.assertEqual(gate["arm_trigger_tts_s"], 0.35)
        self.assertEqual(
            gate["task_observation_status"],
            "NOT_AVAILABLE",
        )
        self.assertIsNone(
            gate["task_observation_valid_dimensions"]
        )
        attempt = live["current_attempt"]
        self.assertEqual(
            (
                attempt["estimator_sample_count"],
                attempt["estimator_window_size"],
                attempt["incoming_count"],
                attempt["incoming_required_count"],
            ),
            (31, 31, 3, 3),
        )
        self.assertIsNone(attempt["planner_tts_s"])
        self.assertIsNone(attempt["arm_tts_s"])
        self.assertEqual(attempt["task_obs_status"], "NOT_AVAILABLE")
        self.assertEqual(attempt["primary_blocker"], "PELVIS_UNAVAILABLE")
        self.assertEqual(
            sum(draft.kind == "LIVE_SNAPSHOT" for draft in lane.drafts),
            1,
        )
        self.assertEqual(len(diagnostic_reads), 1)

    def test_throttled_input_does_not_relabel_canonical_ball_generation(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            (
                monitor,
                _log,
                transport,
                _decoder_log,
                _ingested,
                pipeline,
                _recorder,
                _web,
                _lane,
            ) = _constructed_monitor(Path(temp_dir))
            canonical = BallDiagnosticState(
                status="READY",
                estimator_sample_count=31,
                estimator_window_size=31,
                speed_mps=1.0,
                velocity_world_mps=(-1.0, 0.0, 0.0),
                incoming_count=1,
                incoming_required_count=3,
                incoming_status="INCOMING",
                blocker=None,
                source_frame=101,
                track_epoch=4,
                generation=11,
                raw_position_w=(1.1, 1.2, 1.3),
                estimated_position_w=(1.4, 1.5, 1.6),
                observed_monotonic_s=5.0,
            )
            pipeline.ball_diagnostics = SimpleNamespace(
                snapshot=lambda: canonical
            )
            pipeline.attempt_tracker.binding_for_result = (
                lambda _key: None
            )
            monitor.adapter.ingest_decoded = lambda **_kwargs: SimpleNamespace(
                sample=SimpleNamespace(
                    subject="ball",
                    source_frame=102,
                    position_w=(2.1, 2.2, 2.3),
                    received_monotonic_s=5.005,
                    valid=True,
                    occluded=False,
                ),
                snapshot=SimpleNamespace(
                    source_frame=102,
                    track_epoch=4,
                    generation=12,
                    position_w=(2.4, 2.5, 2.6),
                    velocity_w=(-2.0, 0.0, 0.0),
                    received_monotonic_s=5.005,
                ),
                warnings=(),
                estimator_sample_count=32,
                bounce_detected=False,
                reset_transition=None,
            )
            item = SimpleNamespace(
                channel="vicon_state_data",
                message=SimpleNamespace(name="ball"),
                payload_size=96,
                received_monotonic_s=5.005,
                wall_time_us=1,
            )
            try:
                monitor._process_input(item)
                captured = monitor._ball_diagnostic_snapshot(
                    obs_now_s=5.006,
                    incoming=pipeline.incoming.snapshot(),
                )
            finally:
                monitor.close(timeout_s=0.0)
                transport.close_fds()

        self.assertEqual(
            (
                captured.source_frame,
                captured.track_epoch,
                captured.generation,
                captured.raw_position_w,
                captured.estimated_position_w,
                captured.velocity_world_mps,
                captured.estimator_sample_count,
                captured.incoming_count,
            ),
            (
                101,
                4,
                11,
                (1.1, 1.2, 1.3),
                (1.4, 1.5, 1.6),
                (-1.0, 0.0, 0.0),
                31,
                1,
            ),
        )

    def test_tick_capture_waits_for_inflight_input_projection_boundary(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            (
                monitor,
                _log,
                transport,
                _decoder_log,
                _ingested,
                pipeline,
                _recorder,
                _web,
                _lane,
            ) = _constructed_monitor(Path(temp_dir))
            release_input = threading.Event()
            monitor.adapter.release = release_input
            tick_started = threading.Event()
            tick_entered_pipeline = threading.Event()
            original_tick = pipeline.tick

            def observed_tick(**kwargs):
                tick_entered_pipeline.set()
                return original_tick(**kwargs)

            pipeline.tick = observed_tick
            item = SimpleNamespace(
                channel="vicon_state_data",
                message=SimpleNamespace(name="ball"),
                payload_size=96,
                received_monotonic_s=5.0,
                wall_time_us=1,
            )
            input_thread = threading.Thread(
                target=monitor._process_input,
                args=(item,),
            )

            def run_tick():
                tick_started.set()
                monitor._tick_once()

            tick_thread = threading.Thread(target=run_tick)
            try:
                input_thread.start()
                self.assertTrue(monitor.adapter.entered.wait(1.0))
                tick_thread.start()
                self.assertTrue(tick_started.wait(1.0))
                self.assertFalse(tick_entered_pipeline.wait(0.1))
                release_input.set()
                input_thread.join(1.0)
                tick_thread.join(1.0)
                self.assertFalse(input_thread.is_alive())
                self.assertFalse(tick_thread.is_alive())
                self.assertTrue(tick_entered_pipeline.is_set())
                self.assertLess(
                    pipeline.log.index("pipeline.ingest"),
                    pipeline.log.index("pipeline.tick"),
                )
            finally:
                release_input.set()
                input_thread.join(1.0)
                tick_thread.join(1.0)
                monitor.close(timeout_s=0.0)
                transport.close_fds()

    def test_tick_clocks_are_sampled_after_waiting_input_commits(self):
        class ObservableRLock:
            def __init__(self):
                self._lock = threading.RLock()
                self._metadata_lock = threading.Lock()
                self._owner = None
                self._depth = 0
                self.contended = threading.Event()

            def acquire(self):
                owner = threading.get_ident()
                with self._metadata_lock:
                    if self._owner is not None and self._owner != owner:
                        self.contended.set()
                acquired = self._lock.acquire()
                with self._metadata_lock:
                    if self._owner == owner:
                        self._depth += 1
                    else:
                        self._owner = owner
                        self._depth = 1
                return acquired

            def release(self):
                owner = threading.get_ident()
                with self._metadata_lock:
                    if self._owner != owner or self._depth < 1:
                        raise RuntimeError("lock released by non-owner")
                    self._depth -= 1
                    if self._depth == 0:
                        self._owner = None
                self._lock.release()

            def held_by_current_thread(self):
                with self._metadata_lock:
                    return self._owner == threading.get_ident()

            def __enter__(self):
                self.acquire()
                return self

            def __exit__(self, _exc_type, _exc, _traceback):
                self.release()

        committed = threading.Event()

        def semantic_clock():
            return 12.0 if committed.is_set() else 10.0

        def wall_clock():
            return 2.0 if committed.is_set() else 1.0

        with tempfile.TemporaryDirectory() as temp_dir:
            (
                monitor,
                _log,
                transport,
                _decoder_log,
                _ingested,
                pipeline,
                _recorder,
                _web,
                lane,
            ) = _constructed_monitor(
                Path(temp_dir),
                monotonic_fn=semantic_clock,
            )
            monitor._wall_time_fn = wall_clock
            capture_lock = ObservableRLock()
            monitor._snapshot_capture_lock = capture_lock
            ball_state = {
                "value": BallDiagnosticState(
                    status="NOT_SEEN",
                    estimator_sample_count=None,
                    estimator_window_size=31,
                    speed_mps=None,
                    velocity_world_mps=None,
                    incoming_count=None,
                    incoming_required_count=3,
                    incoming_status="NOT_EVALUATED",
                    blocker="BALL_NOT_SEEN",
                )
            }
            pipeline.ball_diagnostics = SimpleNamespace(
                snapshot=lambda: ball_state["value"]
            )
            original_ingest = pipeline.ingest_adapter_output

            def ingest_and_commit_ball(output):
                original_ingest(output)
                ball_state["value"] = BallDiagnosticState(
                    status="READY",
                    estimator_sample_count=31,
                    estimator_window_size=31,
                    speed_mps=1.0,
                    velocity_world_mps=(-1.0, 0.0, 0.0),
                    incoming_count=1,
                    incoming_required_count=3,
                    incoming_status="INCOMING",
                    blocker=None,
                    source_frame=17,
                    track_epoch=1,
                    generation=1,
                    raw_position_w=(1.0, 0.0, 1.0),
                    estimated_position_w=(1.0, 0.0, 1.0),
                    observed_monotonic_s=11.0,
                )

            pipeline.ingest_adapter_output = ingest_and_commit_ball
            publication_lock_states = []
            original_offer = lane.offer

            def observe_publication(draft):
                publication_lock_states.append(
                    capture_lock.held_by_current_thread()
                )
                return original_offer(draft)

            lane.offer = observe_publication
            item = SimpleNamespace(
                channel="vicon_state_data",
                message=SimpleNamespace(name="ball"),
                payload_size=96,
                received_monotonic_s=11.0,
                wall_time_us=1,
            )
            tick_errors = []

            def run_tick():
                try:
                    monitor._tick_once()
                except BaseException as exc:
                    tick_errors.append(exc)

            tick_thread = threading.Thread(target=run_tick)
            capture_lock.acquire()
            try:
                tick_thread.start()
                self.assertTrue(capture_lock.contended.wait(1.0))
                monitor._process_input(item)
                committed.set()
            finally:
                capture_lock.release()
            tick_thread.join(1.0)
            try:
                self.assertFalse(tick_thread.is_alive())
                self.assertEqual(tick_errors, [])
                live = next(
                    draft.payload
                    for draft in lane.drafts
                    if draft.kind == "LIVE_SNAPSHOT"
                )
                lifecycle = next(
                    draft.payload["lifecycle"]
                    for draft in lane.drafts
                    if draft.kind == "LIFECYCLE_SNAPSHOT"
                )
                observed_s = live["ball"]["observed_monotonic_s"]
                captured_s = live["captured_monotonic_s"]
                self.assertGreaterEqual(captured_s, observed_s)
                self.assertEqual(captured_s - observed_s, 1.0)
                self.assertEqual(live["ball"]["age_s"], 1.0)
                self.assertEqual(lifecycle["lifecycle_now_s"], 12.0)
                self.assertEqual(lifecycle["obs_now_s"], 12.0)
                self.assertEqual(
                    {draft.wall_time_us for draft in lane.drafts},
                    {2_000_000},
                )
                self.assertTrue(publication_lock_states)
                self.assertFalse(any(publication_lock_states))
            finally:
                monitor.close(timeout_s=0.0)
                transport.close_fds()

    def test_subject_health_distinguishes_never_live_stale_and_invalid(self):
        values = iter(
            (
                10.0,
                10.025,
                10.1,
                10.125,
                10.7,
                10.725,
                10.9,
                10.925,
            )
        )
        hub = EventHub(
            _initial_state(
                config_name="hitter",
                session_basename="session-test",
                subjects=("ball", "g1pelvis", "table"),
            )
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            (
                monitor,
                _log,
                transport,
                _decoder_log,
                _ingested,
                _pipeline,
                _recorder,
                _web,
                _lane,
            ) = _constructed_monitor(
                Path(temp_dir),
                monotonic_fn=lambda: next(values),
                event_hub=hub,
            )
            try:
                monitor._tick_once()
                never = hub.state_snapshot().live_snapshot
                monitor._arrival.record(
                    subject="ball",
                    received_monotonic_s=10.1,
                    source_frame=1,
                    valid=True,
                    occluded=False,
                )
                monitor._tick_once()
                live = hub.state_snapshot().live_snapshot
                monitor._tick_once()
                stale = hub.state_snapshot().live_snapshot
                monitor._arrival.record(
                    subject="ball",
                    received_monotonic_s=10.9,
                    source_frame=2,
                    valid=False,
                    occluded=False,
                )
                monitor._tick_once()
                invalid = hub.state_snapshot().live_snapshot
            finally:
                monitor.close(timeout_s=0.0)
                transport.close_fds()

        self.assertEqual(
            (
                never.subjects["ball"].status,
                live.subjects["ball"].status,
                stale.subjects["ball"].status,
                invalid.subjects["ball"].status,
            ),
            ("NEVER_SEEN", "LIVE", "STALE", "INVALID"),
        )
        self.assertIsNone(never.subjects["ball"].rate_hz)
        self.assertIsNone(never.subjects["ball"].age_s)
        self.assertFalse(invalid.subjects["ball"].valid)

    def test_blocked_pelvis_does_not_leak_stale_command_tts(self):
        values = iter((10.0, 10.125))
        hub = EventHub(
            _initial_state(
                config_name="hitter",
                session_basename="session-test",
                subjects=("ball", "g1pelvis", "table"),
            )
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            (
                monitor,
                _log,
                transport,
                _decoder_log,
                _ingested,
                pipeline,
                _recorder,
                _web,
                _lane,
            ) = _constructed_monitor(
                Path(temp_dir),
                monotonic_fn=lambda: next(values),
                event_hub=hub,
            )
            pipeline.attempt_tracker.summary = AttemptSummary(
                attempt_id=1,
                status="ACTIVE",
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
            pipeline.ball_diagnostics = SimpleNamespace(
                snapshot=lambda: BallDiagnosticState(
                    status="READY",
                    estimator_sample_count=31,
                    estimator_window_size=31,
                    speed_mps=0.45,
                    velocity_world_mps=(-0.45, 0.0, 0.0),
                    incoming_count=3,
                    incoming_required_count=3,
                    incoming_status="INCOMING",
                    blocker=None,
                )
            )
            stale_command = SimpleNamespace(
                strike_deadline_monotonic_s=12.0,
                error_type=None,
                error_text=None,
                command_fields={"strike_type": 0},
            )

            def tick(**kwargs):
                pipeline.tick_calls.append(kwargs)
                return SimpleNamespace(
                    phase="armed",
                    lifecycle_decision="none",
                    active_binding=None,
                    cached_binding=None,
                    command_result=stale_command,
                    command_fields_used=None,
                    task_observation=SimpleNamespace(
                        post_clip=tuple(0.0 for _ in range(11)),
                        clip_count=0,
                    ),
                    task_pass=True,
                    errors=(),
                )

            pipeline.tick = tick
            try:
                monitor._tick_once()
            finally:
                monitor.close(timeout_s=0.0)
                transport.close_fds()

        live = hub.state_snapshot().live_snapshot
        self.assertEqual(
            live.production_gate.planner_reason_code,
            "PELVIS_UNAVAILABLE",
        )
        self.assertIsNone(live.production_gate.planner_tts_s)
        self.assertEqual(
            live.production_gate.arm_status,
            "NOT_EVALUATED",
        )
        self.assertEqual(
            live.production_gate.task_observation_status,
            "NOT_AVAILABLE",
        )
        self.assertIsNone(
            live.production_gate.task_observation_valid_dimensions
        )
        self.assertIsNone(
            live.production_gate.task_observation_clip_count
        )
        self.assertIsNone(live.current_attempt.predicted_strike_time_s)
        self.assertIsNone(live.current_attempt.planner_tts_s)
        self.assertIsNone(live.current_attempt.arm_tts_s)

    def test_upstream_gate_precedence_dominates_stale_downstream_state(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            (
                monitor,
                _log,
                transport,
                _decoder_log,
                _ingested,
                _pipeline,
                recorder,
                _web,
                _lane,
            ) = _constructed_monitor(Path(temp_dir))
            monitor.runtime_settings = SimpleNamespace(
                incoming_confirmation_snapshots=3,
                arm_tts_s=0.35,
            )
            ready_ball = BallDiagnosticState(
                status="READY",
                estimator_sample_count=31,
                estimator_window_size=31,
                speed_mps=1.0,
                velocity_world_mps=(-1.0, 0.0, 0.0),
                incoming_count=3,
                incoming_required_count=3,
                incoming_status="INCOMING",
                blocker=None,
            )
            warming_ball = replace(
                ready_ball,
                status="ESTIMATING",
                estimator_sample_count=10,
                speed_mps=None,
                velocity_world_mps=None,
                incoming_count=None,
                incoming_status="NOT_EVALUATED",
                blocker="ESTIMATOR_WARMING",
            )
            live_pelvis = SubjectHealth(
                status="LIVE",
                rate_hz=360.0,
                age_s=0.001,
                source_frame=1,
                valid=True,
                occluded=False,
            )
            missing_pelvis = replace(
                live_pelvis,
                status="NEVER_SEEN",
                rate_hz=None,
                age_s=None,
                source_frame=None,
                valid=None,
                occluded=None,
            )
            confirmed = SimpleNamespace(
                track_epoch=1,
                consecutive_count=3,
                confirmed=True,
            )
            confirming = SimpleNamespace(
                track_epoch=1,
                consecutive_count=1,
                confirmed=False,
            )
            ready_command = SimpleNamespace(
                strike_deadline_monotonic_s=12.0,
                error_type=None,
                error_text=None,
                command_fields={"strike_type": 0},
            )
            rejected_command = SimpleNamespace(
                strike_deadline_monotonic_s=12.0,
                error_type="ValueError",
                error_text="no directed crossing",
                command_fields=None,
            )
            stale_task = SimpleNamespace(
                post_clip=tuple(0.0 for _ in range(11)),
                clip_count=2,
            )
            summary = AttemptSummary(
                attempt_id=1,
                status="ACTIVE",
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
            cases = (
                (
                    "pelvis unavailable",
                    missing_pelvis,
                    ready_ball,
                    confirmed,
                    "armed",
                    ready_command,
                    "BLOCKED",
                    "PELVIS_UNAVAILABLE",
                    "NOT_EVALUATED",
                    "NOT_AVAILABLE",
                    "PLANNER_PELVIS_UNAVAILABLE",
                    "PELVIS_UNAVAILABLE",
                ),
                (
                    "estimator warming",
                    live_pelvis,
                    warming_ball,
                    confirmed,
                    "armed",
                    ready_command,
                    "BLOCKED",
                    "ESTIMATOR_WARMING",
                    "NOT_EVALUATED",
                    "NOT_AVAILABLE",
                    "PLANNER_ESTIMATOR_WARMING",
                    "ESTIMATOR_WARMING",
                ),
                (
                    "incoming not confirmed",
                    live_pelvis,
                    ready_ball,
                    confirming,
                    "armed",
                    ready_command,
                    "BLOCKED",
                    "INCOMING_NOT_CONFIRMED",
                    "NOT_EVALUATED",
                    "NOT_AVAILABLE",
                    "PLANNER_INCOMING_NOT_CONFIRMED",
                    "INCOMING_NOT_CONFIRMED",
                ),
                (
                    "planner rejected",
                    live_pelvis,
                    ready_ball,
                    confirmed,
                    "armed",
                    rejected_command,
                    "REJECTED",
                    "NO_DIRECTED_CROSSING",
                    "NOT_EVALUATED",
                    "NOT_AVAILABLE",
                    "PLANNER_NO_DIRECTED_CROSSING",
                    "NO_DIRECTED_CROSSING",
                ),
                (
                    "stale armed task result",
                    live_pelvis,
                    ready_ball,
                    confirmed,
                    "tracking",
                    ready_command,
                    "READY",
                    None,
                    "NOT_ARMED",
                    "NOT_AVAILABLE",
                    "PLANNER_READY",
                    None,
                ),
            )
            try:
                for (
                    name,
                    pelvis,
                    ball,
                    incoming,
                    phase,
                    command,
                    expected_planner,
                    expected_reason,
                    expected_arm,
                    expected_task,
                    expected_stage,
                    expected_blocker,
                ) in cases:
                    with self.subTest(name=name):
                        tick_result = SimpleNamespace(
                            phase=phase,
                            command_result=command,
                            task_observation=stale_task,
                            task_pass=True,
                            errors=(),
                        )
                        gate = monitor._production_gate_snapshot(
                            tick_result=tick_result,
                            incoming=incoming,
                            ball=ball,
                            subjects={"g1pelvis": pelvis},
                            obs_now_s=10.0,
                        )
                        projected = monitor._project_attempt(
                            summary,
                            tick_result=tick_result,
                            incoming=incoming,
                            obs_now_s=10.0,
                            recorder_status=recorder.status(),
                            ball=ball,
                            production_gate=gate,
                        )
                        self.assertEqual(
                            (
                                gate.planner_status,
                                gate.planner_reason_code,
                                gate.arm_status,
                                gate.task_observation_status,
                                gate.task_observation_valid_dimensions,
                                gate.task_observation_clip_count,
                                projected.stage,
                                projected.primary_blocker,
                                projected.task_obs_status,
                            ),
                            (
                                expected_planner,
                                expected_reason,
                                expected_arm,
                                expected_task,
                                None,
                                None,
                                expected_stage,
                                expected_blocker,
                                expected_task,
                            ),
                        )
            finally:
                monitor.close(timeout_s=0.0)
                transport.close_fds()

    def test_unknown_ball_projection_preserves_null_progress(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            (
                monitor,
                _log,
                transport,
                _decoder_log,
                _ingested,
                _pipeline,
                recorder,
                _web,
                _lane,
            ) = _constructed_monitor(Path(temp_dir))
            monitor._latest_estimator_sample_count = 17
            monitor._latest_ball_speed_mps = 1.5
            unknown = BallDiagnosticState(
                status="UNKNOWN",
                estimator_sample_count=None,
                estimator_window_size=None,
                speed_mps=None,
                velocity_world_mps=None,
                incoming_count=None,
                incoming_required_count=None,
                incoming_status="UNKNOWN",
                blocker=None,
            )
            pelvis = SubjectHealth(
                status="LIVE",
                rate_hz=360.0,
                age_s=0.001,
                source_frame=1,
                valid=True,
                occluded=False,
            )
            incoming = SimpleNamespace(
                track_epoch=None,
                consecutive_count=0,
                confirmed=False,
            )
            tick_result = SimpleNamespace(
                phase="armed",
                command_result=None,
                task_observation=None,
                task_pass=False,
                errors=(),
            )
            summary = AttemptSummary(
                attempt_id=1,
                status="ACTIVE",
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
            try:
                gate = monitor._production_gate_snapshot(
                    tick_result=tick_result,
                    incoming=incoming,
                    ball=unknown,
                    subjects={"g1pelvis": pelvis},
                    obs_now_s=10.0,
                )
                projected = monitor._project_attempt(
                    summary,
                    tick_result=tick_result,
                    incoming=incoming,
                    obs_now_s=10.0,
                    recorder_status=recorder.status(),
                    ball=unknown,
                    production_gate=gate,
                )
            finally:
                monitor.close(timeout_s=0.0)
                transport.close_fds()

        self.assertEqual(gate.planner_status, "BLOCKED")
        self.assertEqual(gate.planner_reason_code, "BALL_STATE_UNKNOWN")
        self.assertNotEqual(gate.planner_reason_code, "ESTIMATOR_WARMING")
        self.assertEqual(projected.stage, "PLANNER_BALL_STATE_UNKNOWN")
        self.assertEqual(projected.primary_blocker, "BALL_STATE_UNKNOWN")
        self.assertIsNone(projected.ball_speed_mps)
        self.assertIsNone(projected.estimator_sample_count)
        self.assertIsNone(projected.estimator_window_size)
        self.assertIsNone(projected.incoming_count)
        self.assertIsNone(projected.incoming_required_count)

    def test_attempt_projection_uses_same_ball_window_and_required_counts(self):
        values = iter((10.0, 10.125))
        with tempfile.TemporaryDirectory() as temp_dir:
            (
                monitor,
                _log,
                transport,
                _decoder_log,
                _ingested,
                pipeline,
                _recorder,
                _web,
                lane,
            ) = _constructed_monitor(
                Path(temp_dir),
                monotonic_fn=lambda: next(values),
            )
            pipeline.attempt_tracker.summary = AttemptSummary(
                attempt_id=1,
                status="ACTIVE",
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
            pipeline.ball_diagnostics = SimpleNamespace(
                snapshot=lambda: BallDiagnosticState(
                    status="READY",
                    estimator_sample_count=17,
                    estimator_window_size=29,
                    speed_mps=0.30,
                    velocity_world_mps=(-0.30, 0.0, 0.0),
                    incoming_count=2,
                    incoming_required_count=4,
                    incoming_status="INCOMING",
                    blocker=None,
                )
            )
            try:
                monitor._tick_once()
            finally:
                monitor.close(timeout_s=0.0)
                transport.close_fds()

        live = next(
            draft.payload
            for draft in lane.drafts
            if draft.kind == "LIVE_SNAPSHOT"
        )
        attempt = live["current_attempt"]
        self.assertEqual(
            attempt["stage"],
            "PLANNER_PELVIS_UNAVAILABLE",
        )
        self.assertEqual(
            (
                attempt["estimator_sample_count"],
                attempt["estimator_window_size"],
                attempt["incoming_count"],
                attempt["incoming_required_count"],
            ),
            (17, 29, 2, 4),
        )

    def test_attempt_ignores_legacy_count_without_canonical_ball_snapshot(self):
        values = iter((10.0, 10.125))
        with tempfile.TemporaryDirectory() as temp_dir:
            (
                monitor,
                _log,
                transport,
                _decoder_log,
                _ingested,
                pipeline,
                _recorder,
                _web,
                lane,
            ) = _constructed_monitor(
                Path(temp_dir),
                monotonic_fn=lambda: next(values),
            )
            pipeline.attempt_tracker.summary = AttemptSummary(
                attempt_id=1,
                status="ACTIVE",
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
            pipeline.incoming = SimpleNamespace(
                snapshot=lambda: SimpleNamespace(
                    consecutive_count=0,
                    confirmed=False,
                )
            )
            monitor._latest_estimator_sample_count = 17
            try:
                monitor._tick_once()
            finally:
                monitor.close(timeout_s=0.0)
                transport.close_fds()

            current = next(
                draft
                for draft in lane.drafts
                if draft.kind == "ATTEMPT_CURRENT"
            )
            self.assertEqual(
                current.payload["attempt"]["stage"],
                "PLANNER_PELVIS_UNAVAILABLE",
            )
            self.assertIsNone(
                current.payload["attempt"]["estimator_sample_count"]
            )

    def test_health_requires_fresh_heartbeat_and_reports_result_overwrite(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            (
                monitor,
                _log,
                transport,
                _decoder_log,
                _ingested,
                _pipeline,
                recorder,
                _web,
                _lane,
            ) = _constructed_monitor(Path(temp_dir))
            monitor._subscription = transport.subscription
            try:
                never_received = monitor._health_snapshot(
                    obs_now_s=10.0,
                    recorder_status=recorder.status(),
                )
                self.assertFalse(never_received.lcm_connected)
                self.assertIn(
                    "LCM_HEARTBEAT_STALE",
                    never_received.warnings,
                )

                monitor._arrival.record(
                    subject="ball",
                    received_monotonic_s=9.9,
                    source_frame=1,
                )
                fresh = monitor._health_snapshot(
                    obs_now_s=10.0,
                    recorder_status=recorder.status(),
                )
                self.assertTrue(fresh.lcm_connected)
                self.assertEqual(
                    fresh.planner_results_overwritten_before_consume,
                    7,
                )

                stale = monitor._health_snapshot(
                    obs_now_s=10.500001,
                    recorder_status=recorder.status(),
                )
                self.assertFalse(stale.lcm_connected)
                self.assertIn("LCM_HEARTBEAT_STALE", stale.warnings)
            finally:
                monitor._subscription = None
                monitor.close(timeout_s=0.0)
                transport.close_fds()

    def test_event_lane_drop_marks_session_and_attempt_incomplete(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            (
                monitor,
                _log,
                transport,
                _decoder_log,
                _ingested,
                _pipeline,
                recorder,
                _web,
                lane,
            ) = _constructed_monitor(Path(temp_dir))
            lane.offer = lambda _draft: False
            try:
                accepted = monitor._offer(
                    EventDraft(
                        kind="TEST_DROP",
                        monotonic_s=1.0,
                        wall_time_us=1,
                        scope="attempt",
                        attempt_id=13,
                        payload={},
                    )
                )
                self.assertFalse(accepted)
                self.assertEqual(
                    recorder.incomplete_calls,
                    [(13, "DIAGNOSTIC_EVENT_DROPPED")],
                )
                self.assertFalse(recorder.status().recording_complete)
            finally:
                monitor.close(timeout_s=0.0)
                transport.close_fds()

    def test_tick_closes_attempt_into_repository_recorder_and_state(self):
        values = iter((20.0, 20.1))
        with tempfile.TemporaryDirectory() as temp_dir:
            (
                monitor,
                _log,
                transport,
                _decoder_log,
                _ingested,
                pipeline,
                recorder,
                _web,
                lane,
            ) = _constructed_monitor(
                Path(temp_dir),
                monotonic_fn=lambda: next(values),
            )
            pipeline.attempt_tracker = _ClosingTracker()
            repository = monitor.attempt_repository
            try:
                monitor._tick_once()
                monitor._drain_attempt_close_requests()
            finally:
                monitor.close(timeout_s=0.0)
                transport.close_fds()

            self.assertEqual(repository.detail.attempt_id, 7)
            self.assertEqual(recorder.detail.attempt_id, 7)
            self.assertEqual(
                [draft.kind for draft in lane.drafts].count(
                    "ATTEMPT_CLOSED"
                ),
                1,
            )
            self.assertFalse(
                any(draft.kind == "ATTEMPT_CURRENT" for draft in lane.drafts)
            )

    def test_tick_close_never_waits_for_slow_repository_commit(self):
        class SlowRepository(_FakeRepository):
            def __init__(self):
                self.commit_calls = 0

            def commit(self, detail):
                self.commit_calls += 1
                time.sleep(0.12)
                super().commit(detail)

        values = iter((20.0, 20.1))
        with tempfile.TemporaryDirectory() as temp_dir:
            (
                monitor,
                _log,
                transport,
                _decoder_log,
                _ingested,
                pipeline,
                _recorder,
                _web,
                _lane,
            ) = _constructed_monitor(
                Path(temp_dir),
                monotonic_fn=lambda: next(values),
            )
            pipeline.attempt_tracker = _ClosingTracker()
            repository = SlowRepository()
            monitor.attempt_repository = repository
            try:
                started = time.monotonic()
                monitor._tick_once()
                elapsed = time.monotonic() - started

                self.assertLess(elapsed, 0.02)
                self.assertEqual(repository.commit_calls, 0)
            finally:
                monitor.close(timeout_s=0.0)
                transport.close_fds()

    def test_tick_never_scans_recorder_filesystem_status(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            (
                monitor,
                _log,
                transport,
                _decoder_log,
                _ingested,
                _pipeline,
                recorder,
                _web,
                _lane,
            ) = _constructed_monitor(Path(temp_dir))
            status_calls = []
            original_status = recorder.status

            def slow_status():
                status_calls.append(time.monotonic())
                time.sleep(0.12)
                return original_status()

            recorder.status = slow_status
            try:
                started = time.monotonic()
                monitor._tick_once()
                elapsed = time.monotonic() - started

                self.assertLess(elapsed, 0.02)
                self.assertEqual(status_calls, [])
            finally:
                monitor.close(timeout_s=0.0)
                transport.close_fds()

    def test_input_transition_close_is_background_and_exactly_once(self):
        class CountingRepository(_FakeRepository):
            def __init__(self):
                self.commit_calls = 0

            def commit(self, detail):
                self.commit_calls += 1
                super().commit(detail)

        with tempfile.TemporaryDirectory() as temp_dir:
            (
                monitor,
                _log,
                transport,
                _decoder_log,
                _ingested,
                pipeline,
                recorder,
                _web,
                _lane,
            ) = _constructed_monitor(Path(temp_dir))
            tracker = _ClosingTracker()
            tracker.closed = True
            pipeline.attempt_tracker = tracker
            repository = CountingRepository()
            monitor.attempt_repository = repository
            transition = EventDraft(
                kind="attempt_transition",
                monotonic_s=20.3,
                wall_time_us=20300000,
                scope="attempt",
                attempt_id=7,
                payload={
                    "track_segment_id": 3,
                    "stage": "ATTEMPT_CLOSED",
                    "reason_code": "TRACK_ENDED_BEFORE_READY",
                    "snapshot_key": None,
                    "values": {"status": "FAILED"},
                },
            )
            recorder_thread = threading.Thread(
                target=monitor._recorder_loop,
                daemon=True,
            )
            monitor._recorder_thread = recorder_thread
            recorder_thread.start()
            try:
                monitor._offer(transition)
                monitor._offer(transition)
                deadline = time.monotonic() + 1.0
                while not recorder.details and time.monotonic() < deadline:
                    time.sleep(0.005)

                self.assertEqual(repository.commit_calls, 1)
                self.assertEqual(len(recorder.details), 1)
                self.assertEqual(recorder.details[0].attempt_id, 7)
            finally:
                monitor._stop_event.set()
                recorder_thread.join(1.0)
                monitor.close(timeout_s=0.5)
                transport.close_fds()

    def test_background_close_does_not_project_new_attempt_metrics(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            (
                monitor,
                _log,
                transport,
                _decoder_log,
                _ingested,
                pipeline,
                _recorder,
                _web,
                _lane,
            ) = _constructed_monitor(Path(temp_dir))
            tracker = _ClosingTracker()
            tracker.closed = True
            pipeline.attempt_tracker = tracker
            monitor._latest_ball_speed_mps = 99.0
            monitor._last_tick_result = SimpleNamespace(
                phase="armed",
                lifecycle_decision="armed",
                active_binding=None,
                cached_binding=None,
                command_result=SimpleNamespace(
                    strike_deadline_monotonic_s=44.0,
                ),
                task_observation=None,
                task_pass=True,
            )
            try:
                monitor._offer(
                    EventDraft(
                        kind="attempt_transition",
                        monotonic_s=20.3,
                        wall_time_us=20300000,
                        scope="attempt",
                        attempt_id=7,
                        payload={
                            "track_segment_id": 3,
                            "stage": "ATTEMPT_CLOSED",
                            "reason_code": "TRACK_ENDED_BEFORE_READY",
                            "snapshot_key": None,
                            "values": {"status": "FAILED"},
                        },
                    )
                )
                monitor._drain_attempt_close_requests()

                stored = monitor.attempt_repository.get(7)
                self.assertIsNone(stored.summary.ball_speed_mps)
                self.assertIsNone(stored.summary.predicted_strike_time_s)
                self.assertIsNone(stored.summary.planner_tts_s)
                self.assertEqual(
                    stored.summary.primary_blocker,
                    "TRACK_ENDED_BEFORE_READY",
                )
            finally:
                monitor.close(timeout_s=0.0)
                transport.close_fds()

    def test_stale_tick_binding_cannot_recreate_or_project_new_attempt(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            (
                monitor,
                _log,
                transport,
                _decoder_log,
                _ingested,
                pipeline,
                _recorder,
                _web,
                lane,
            ) = _constructed_monitor(Path(temp_dir))
            pipeline.attempt_tracker.summary = AttemptSummary(
                attempt_id=2,
                status="ACTIVE",
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
            stale_result = SimpleNamespace(
                phase="armed",
                lifecycle_decision="armed",
                active_binding=AttemptBinding(
                    attempt_id=1,
                    track_segment_id=1,
                    role="PRIMARY",
                    snapshot_key=SnapshotKey(1, 1),
                ),
                cached_binding=None,
                command_result=None,
                command_fields_used=None,
                task_observation=SimpleNamespace(
                    pre_clip=(1.0, 2.0),
                    post_clip=(1.0, 2.0),
                    clip_count=0,
                ),
                task_pass=True,
                errors=(),
            )

            def stale_tick(**kwargs):
                pipeline.tick_calls.append(kwargs)
                return stale_result

            pipeline.tick = stale_tick
            monitor._remember_closed_attempt(1)
            try:
                monitor._tick_once()
                pipeline.attempt_tracker.binding_for_result = (
                    lambda _key: stale_result.active_binding
                )
                monitor._remember_planner_input(
                    SimpleNamespace(
                        snapshot=SimpleNamespace(
                            track_epoch=1,
                            generation=2,
                            source_frame=2,
                            source_time_s=1.0,
                            received_monotonic_s=1.0,
                            position_w=(1.0, 2.0, 3.0),
                            velocity_w=(4.0, 5.0, 6.0),
                            base_position_w=(0.0, 0.0, 0.8),
                            base_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                            base_valid=True,
                            visible=True,
                            ready=True,
                        )
                    )
                )

                self.assertNotIn(1, monitor._attempt_observations)
                self.assertFalse(
                    any(
                        draft.kind == "ATTEMPT_CURRENT"
                        and draft.attempt_id == 2
                        for draft in lane.drafts
                    )
                )
            finally:
                monitor.close(timeout_s=0.0)
                transport.close_fds()

    def test_new_ball_before_tick_closes_old_attempt_into_replay_once(self):
        class BundleCapture:
            def __init__(self):
                self.calls = []

            def ready_to_finalize(self, attempt_id):
                del attempt_id
                return True

            def finalize_attempt(self, **kwargs):
                self.calls.append(dict(kwargs))
                attempt_id = kwargs["attempt_id"]
                return ReplayInputBundle(
                    attempt_id=attempt_id,
                    inputs=(),
                    online_baseline=OnlineAttemptTrace(
                        attempt_id=attempt_id,
                        stages=(),
                        planner_calls=(),
                        policy_ticks=(),
                        terminal_code=kwargs["terminal_code"],
                        recovery_duration_s=None,
                        submission_phase_anchor_s=None,
                        recording_complete=kwargs["recording_complete"],
                    ),
                    recording_complete=kwargs["recording_complete"],
                    planner_config={},
                )

            def mark_incomplete(self, attempt_id):
                del attempt_id

        class ReplayQueue(_FakeReplay):
            def __init__(self, log):
                super().__init__(log)
                self.jobs = []

            def submit_disk_job(self, job):
                self.jobs.append(job)

        def sample(at_s, *, visible):
            return SimpleNamespace(
                received_monotonic_s=float(at_s),
                valid=bool(visible),
                occluded=not bool(visible),
            )

        def publish_transitions(monitor, transitions):
            for transition in transitions:
                monitor._offer(
                    EventDraft(
                        kind="attempt_transition",
                        monotonic_s=transition.monotonic_s,
                        wall_time_us=int(
                            transition.monotonic_s * 1.0e6
                        ),
                        scope="attempt",
                        attempt_id=transition.attempt_id,
                        payload={
                            "track_segment_id": (
                                transition.track_segment_id
                            ),
                            "stage": transition.stage,
                            "reason_code": transition.reason_code,
                            "snapshot_key": (
                                None
                                if transition.snapshot_key is None
                                else transition.snapshot_key.to_json_dict()
                            ),
                            "values": transition.values,
                        },
                    )
                )

        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            (
                monitor,
                log,
                transport,
                _decoder_log,
                _ingested,
                pipeline,
                _recorder,
                _web,
                _lane,
            ) = _constructed_monitor(root)
            tracker = AttemptTracker(
                reacquire_grace_s=0.20,
                max_attempts=8,
            )
            pipeline.attempt_tracker = tracker
            capture = BundleCapture()
            replay = ReplayQueue(log)
            monitor.replay_capture = capture
            monitor.replay_controller = replay
            monitor.session_paths = SimpleNamespace(root=root / "session")
            try:
                publish_transitions(
                    monitor,
                    tracker.observe_ball_sample(
                        sample(10.0, visible=True),
                        snapshot_key=SnapshotKey(1, 1),
                    ),
                )
                publish_transitions(
                    monitor,
                    tracker.observe_ball_sample(
                        sample(10.1, visible=False),
                        snapshot_key=SnapshotKey(2, 1),
                    ),
                )
                transitions = tracker.observe_ball_sample(
                    sample(10.301, visible=True),
                    snapshot_key=SnapshotKey(2, 2),
                )
                publish_transitions(monitor, transitions)

                self.assertEqual(
                    tracker.current_summary().attempt_id,
                    2,
                )
                monitor._drain_attempt_close_requests()
                monitor._submit_replay_close_requests()

                self.assertEqual(
                    [call["attempt_id"] for call in capture.calls],
                    [1],
                )
                self.assertEqual(
                    [job.attempt_id for job in replay.jobs],
                    [1],
                )
                self.assertEqual(
                    monitor.attempt_repository.get(1).attempt_id,
                    1,
                )
                self.assertNotIn(2, monitor._closed_attempt_ids)
            finally:
                monitor.close(timeout_s=0.0)
                transport.close_fds()

    def test_closed_attempt_identity_cache_is_bounded(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            (
                monitor,
                _log,
                transport,
                _decoder_log,
                _ingested,
                _pipeline,
                _recorder,
                _web,
                _lane,
            ) = _constructed_monitor(Path(temp_dir))
            try:
                for attempt_id in range(1, 101):
                    self.assertTrue(
                        monitor._remember_closed_attempt(attempt_id)
                    )
                self.assertEqual(
                    len(monitor._closed_attempt_ids),
                    monitor.options.attempt_cache_size,
                )
                self.assertEqual(
                    tuple(monitor._closed_attempt_ids),
                    tuple(range(93, 101)),
                )
            finally:
                monitor.close(timeout_s=0.0)
                transport.close_fds()

    def test_completed_replay_recorder_rejection_is_fail_closed_everywhere(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            (
                monitor,
                _log,
                transport,
                _decoder_log,
                _ingested,
                _pipeline,
                recorder,
                _web,
                lane,
            ) = _constructed_monitor(Path(temp_dir))
            summary = _ClosingTracker().summary
            initial = AttemptDetail(
                attempt_id=summary.attempt_id,
                summary=replace(summary, ab_summary="INCONCLUSIVE"),
                segments=(),
                stage_timeline=(),
                planner_inputs=(),
                planner_results=(),
                task_observation_pre_clip=None,
                task_observation_post_clip=None,
                task_observation_clip_count=None,
                variant_outcomes=(),
                ab_deltas={},
            )
            monitor.attempt_repository.commit(initial)
            recorder.offer_attempt_detail = lambda _detail: False
            try:
                monitor._apply_replay_reply(
                    ReplayWorkerReply(
                        attempt_id=7,
                        status="COMPLETED",
                        result={
                            "baseline": {"recording_complete": True},
                            "one_frame": {"recording_complete": True},
                            "parity": {"matches": True},
                            "deltas": {},
                            "summary_label": "BOTH_FAIL_SAME",
                        },
                        error=None,
                    )
                )

                stored = monitor.attempt_repository.get(7)
                self.assertEqual(stored.summary.ab_summary, "INCONCLUSIVE")
                self.assertFalse(stored.summary.recording_complete)
                self.assertIn(
                    (7, "REPLAY_DETAIL_RECORDER_REJECTED"),
                    recorder.incomplete_calls,
                )
                published = [
                    draft
                    for draft in lane.drafts
                    if draft.kind == "ATTEMPT_CLOSED"
                ]
                self.assertTrue(published)
                self.assertEqual(
                    published[-1].payload["attempt"]["ab_summary"],
                    "INCONCLUSIVE",
                )
                self.assertFalse(
                    published[-1].payload["attempt"]["recording_complete"]
                )
            finally:
                monitor.close(timeout_s=0.0)
                transport.close_fds()

    def test_replay_persistence_failure_marks_attempt_recording_incomplete(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            (
                monitor,
                _log,
                transport,
                _decoder_log,
                _ingested,
                _pipeline,
                recorder,
                _web,
                _lane,
            ) = _constructed_monitor(Path(temp_dir))
            summary = _ClosingTracker().summary
            monitor.attempt_repository.commit(
                AttemptDetail(
                    attempt_id=summary.attempt_id,
                    summary=replace(
                        summary,
                        ab_summary="INCONCLUSIVE",
                        recording_complete=True,
                    ),
                    segments=(),
                    stage_timeline=(),
                    planner_inputs=(),
                    planner_results=(),
                    task_observation_pre_clip=None,
                    task_observation_post_clip=None,
                    task_observation_clip_count=None,
                    variant_outcomes=(),
                    ab_deltas={},
                )
            )
            try:
                monitor._apply_replay_reply(
                    ReplayWorkerReply(
                        attempt_id=summary.attempt_id,
                        status="FAILED",
                        result=None,
                        error=(
                            "REPLAY_PERSISTENCE_ERROR:OSError: "
                            "synthetic disk failure"
                        ),
                    )
                )

                stored = monitor.attempt_repository.get(summary.attempt_id)
                self.assertEqual(
                    stored.summary.ab_summary,
                    "INCONCLUSIVE",
                )
                self.assertFalse(stored.summary.recording_complete)
                self.assertIn(
                    (
                        summary.attempt_id,
                        "REPLAY_PERSISTENCE_ERROR",
                    ),
                    recorder.incomplete_calls,
                )
            finally:
                monitor.close(timeout_s=0.0)
                transport.close_fds()

    def test_attempt_close_runs_disk_ab_and_merges_completed_detail(self):
        class BundleCapture:
            def __init__(self, bundle):
                self.bundle = bundle
                self.calls = []

            def finalize_attempt(self, **kwargs):
                self.calls.append(dict(kwargs))
                return self.bundle

            def mark_incomplete(self, attempt_id):
                del attempt_id

        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            (
                monitor,
                _log,
                transport,
                _decoder_log,
                _ingested,
                pipeline,
                recorder,
                _web,
                lane,
            ) = _constructed_monitor(root)
            pipeline.attempt_tracker = _ClosingTracker()
            bundle = ReplayInputBundle(
                attempt_id=7,
                inputs=(),
                online_baseline=OnlineAttemptTrace(
                    attempt_id=7,
                    stages=(),
                    planner_calls=(),
                    policy_ticks=(),
                    terminal_code="TRACK_ENDED_BEFORE_READY",
                    recovery_duration_s=None,
                    submission_phase_anchor_s=None,
                    recording_complete=True,
                ),
                recording_complete=True,
                planner_config={},
            )
            capture = BundleCapture(bundle)
            controller = ReplayProcessController(
                worker_fn=run_replay_disk_job,
                recorder=recorder,
                event_sink=lane,
            )
            monitor.replay_capture = capture
            monitor.replay_controller = controller
            monitor.session_paths = SimpleNamespace(
                root=root / "session"
            )
            try:
                monitor._tick_once()
                monitor._drain_attempt_close_requests()
                self.assertEqual(
                    monitor.attempt_repository.get(7).summary.ab_summary,
                    "INCONCLUSIVE",
                )
                monitor._submit_replay_close_requests()
                self.assertEqual(
                    recorder.replay_job_statuses[0]["status"],
                    "QUEUED",
                )
                replies = []
                deadline = time.monotonic() + 5.0
                while time.monotonic() < deadline:
                    controller.wait_for_reply(timeout_s=0.1)
                    replies.extend(controller.poll())
                    terminal = [
                        reply
                        for reply in replies
                        if reply.status in ("COMPLETED", "FAILED")
                    ]
                    if terminal:
                        break
                self.assertTrue(terminal)
                self.assertEqual(terminal[-1].status, "COMPLETED")
                monitor._apply_replay_reply(terminal[-1])

                detail = monitor.attempt_repository.get(7)
                self.assertEqual(
                    detail.summary.ab_summary,
                    "BOTH_FAIL_SAME",
                )
                self.assertEqual(len(detail.variant_outcomes), 2)
                self.assertIn("delta_confirm_ms", detail.ab_deltas)
                self.assertTrue(
                    (
                        root
                        / "session"
                        / "replay_inputs"
                        / "attempt-7.json"
                    ).is_file()
                )
                self.assertTrue(recorder.replay_analyses)
                self.assertTrue(
                    any(
                        item["status"] == "COMPLETED"
                        for item in recorder.replay_job_statuses
                    )
                )
                self.assertGreaterEqual(len(recorder.details), 2)
                self.assertTrue(
                    any(
                        draft.kind == "ATTEMPT_CLOSED"
                        and draft.payload["attempt"]["ab_summary"]
                        == "BOTH_FAIL_SAME"
                        for draft in lane.drafts
                    )
                )
            finally:
                monitor.close(timeout_s=5.0)
                transport.close_fds()

    def test_start_and_close_follow_exact_bounded_component_order_once(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            (
                monitor,
                log,
                transport,
                _decoder_log,
                _ingested,
                _pipeline,
                recorder,
                web,
                _lane,
            ) = _constructed_monitor(Path(temp_dir))
            try:
                monitor.start()
                self.assertLess(
                    log.index("recorder.start"),
                    log.index("web.start"),
                )
                self.assertLess(
                    log.index("web.start"),
                    log.index("lcm.subscribe"),
                )
                self.assertTrue(monitor.close(timeout_s=1.0))
                self.assertTrue(monitor.close(timeout_s=1.0))
                expected = [
                    "lcm.unsubscribe",
                    "pipeline.close",
                    "replay.close",
                    "recorder.request_stop",
                    "recorder.drain",
                    "recorder.terminal",
                    "web.close",
                ]
                indices = [log.index(item) for item in expected]
                self.assertEqual(indices, sorted(indices))
                for item in expected:
                    self.assertEqual(log.count(item), 1)
                self.assertLessEqual(recorder.drain_timeout_s, 1.0)
                self.assertLessEqual(recorder.terminal_timeout_s, 1.0)
                self.assertLessEqual(web.timeout_s, 1.0)
            finally:
                monitor.close(timeout_s=0.0)
                transport.close_fds()

    def test_close_waits_for_tick_before_final_attempt_mailbox_drain(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            (
                monitor,
                _log,
                transport,
                _decoder_log,
                _ingested,
                pipeline,
                _recorder,
                _web,
                _lane,
            ) = _constructed_monitor(Path(temp_dir))
            pipeline.attempt_tracker = _ClosingTracker()
            tick_waiting = threading.Event()
            release_tick = threading.Event()
            event_drain_started = threading.Event()
            original_offer = monitor._offer
            original_event_drain = monitor.event_lane.drain

            def blocking_offer(draft):
                if (
                    draft.kind == "attempt_transition"
                    and draft.payload.get("stage") == "ATTEMPT_CLOSED"
                ):
                    tick_waiting.set()
                    self.assertTrue(release_tick.wait(1.0))
                return original_offer(draft)

            def observed_event_drain(*, timeout_s):
                event_drain_started.set()
                return original_event_drain(timeout_s=timeout_s)

            def release_after_order_is_observable():
                event_drain_started.wait(0.10)
                release_tick.set()

            monitor._offer = blocking_offer
            monitor.event_lane.drain = observed_event_drain
            tick_thread = threading.Thread(target=monitor._tick_once)
            release_thread = threading.Thread(
                target=release_after_order_is_observable
            )
            monitor._tick_thread = tick_thread
            tick_thread.start()
            release_thread.start()
            try:
                self.assertTrue(tick_waiting.wait(1.0))
                self.assertTrue(monitor.close(timeout_s=1.0))
                self.assertFalse(tick_thread.is_alive())
                self.assertEqual(monitor.event_lane.close_pending_count, 0)
                self.assertIsNotNone(monitor.attempt_repository.get(7))
            finally:
                release_tick.set()
                tick_thread.join(1.0)
                release_thread.join(1.0)
                monitor.close(timeout_s=0.0)
                transport.close_fds()

    def test_build_loads_resolved_values_and_writes_complete_metadata(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            table = root / "table.json"
            pelvis = root / "pelvis.json"
            table.write_text('{"table":"fixture"}\n', encoding="utf-8")
            pelvis.write_text('{"pelvis":"fixture"}\n', encoding="utf-8")
            options = _options(root, duration_s=0.05)
            transport = _FakeTransport()
            monitor = build_monitor(
                options,
                lcm_factory=lambda _url: transport,
                decoder=lambda _payload: SimpleNamespace(name="ball"),
            )
            try:
                self.assertIsNotNone(monitor.replay_controller)
                self.assertEqual(monitor.estimator_window_size, 31)
                self.assertEqual(
                    monitor.runtime_settings.estimator_sample_rate_hz,
                    360.0,
                )
                self.assertEqual(
                    monitor.runtime_settings.planner_update_rate_hz,
                    100.0,
                )
                self.assertEqual(
                    monitor.runtime_settings.incoming_confirmation_snapshots,
                    3,
                )
                self.assertAlmostEqual(
                    1.0 / monitor.runtime_settings.control_tick_s,
                    50.0,
                )
                self.assertEqual(monitor.run(), 0)
                metadata = json.loads(
                    monitor.session_paths.session_json.read_text(
                        encoding="utf-8"
                    )
                )
                self.assertEqual(metadata["schema_version"], 1)
                self.assertEqual(metadata["lcm_url"], options.lcm_url)
                self.assertIn("argv", metadata)
                self.assertIn("git_commit", metadata)
                self.assertIn("git_dirty", metadata)
                self.assertIn("python_version", metadata)
                self.assertIn("package_versions", metadata)
                self.assertEqual(
                    metadata["config_sha256"]["mimic"],
                    hashlib.sha256(
                        options.mimic_config.read_bytes()
                    ).hexdigest(),
                )
                self.assertEqual(
                    metadata["calibration_sha256"]["table"],
                    hashlib.sha256(table.read_bytes()).hexdigest(),
                )
                self.assertEqual(
                    Path(metadata["output_dir"]),
                    options.output_dir.resolve(),
                )
                self.assertEqual(
                    metadata["resolved_runtime"],
                    {
                        "estimator_window_size": 31,
                        "estimator_sample_rate_hz": 360.0,
                        "planner_update_rate_hz": 100.0,
                        "incoming_confirmation_snapshots": 3,
                        "control_rate_hz": 50.0,
                        "control_tick_s": 0.02,
                        "duration_s": options.duration_s,
                        "reacquire_grace_s": options.reacquire_grace_s,
                        "heartbeat_stale_s": options.heartbeat_stale_s,
                        "pelvis_stale_s": options.pelvis_stale_s,
                        "source_frame_gap_threshold": (
                            options.source_frame_gap_threshold
                        ),
                        "input_capacity": options.input_capacity,
                        "raw_capacity": options.raw_capacity,
                        "event_capacity": options.event_capacity,
                        "attempt_cache_size": options.attempt_cache_size,
                    },
                )
            finally:
                monitor.close(timeout_s=1.0)
                transport.close_fds()

    def test_import_does_not_load_runtime_or_inference_modules(self):
        source = Path(
            __import__(
                "diagnostics.hitter_task_monitor",
                fromlist=["__file__"],
            ).__file__
        ).read_text(encoding="utf-8")
        for forbidden in (
            "onnxruntime",
            "InferenceSession",
            "pd_plustau_targets",
            "body_control_data",
            "rc_command_data",
        ):
            self.assertNotIn(forbidden, source)

        completed = subprocess.run(
            [
                "/home/loco1/miniconda3/envs/rb/bin/python",
                "-c",
                (
                    "import sys;"
                    "import diagnostics.hitter_task_monitor;"
                    "banned=('deploy.agents','agents.hitter_agent',"
                    "'onnxruntime','simulator.real_world');"
                    "assert not any(name in sys.modules for name in banned)"
                ),
            ],
            cwd=str(DEPLOY_ROOT),
            env=dict(os.environ, PYTHONPATH="."),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=5.0,
            check=False,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)


if __name__ == "__main__":
    unittest.main()
