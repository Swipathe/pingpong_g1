from __future__ import annotations

import os
import threading
import time
import unittest
from dataclasses import FrozenInstanceError
from types import SimpleNamespace
from unittest import mock

import numpy as np

from simulator.base_sim import BaseSim
from unitree_sdk2.lcm_types.transformation_t import transformation_t
from utils.hitter_ball_pipeline import BasePoseW, RealtimeViconBallPipeline
from utils.hitter_chingmu_ghost import (
    ChingMuGhostBallSource,
    ChingMuGhostSourceSettings,
    ChingMuGhostSourceStatus,
)
from utils.read_only_lcm import (
    PublicationAudit,
    PublishDenyLcm,
    ReadOnlyLcmSubscription,
)


LCM_URL = "memq://"
CHANNEL = "vicon_state_data"
TABLE_CENTER = (1.365369, 0.0, 0.76)
TABLE_QUAT = (0.0, 0.0, 0.0, 1.0)


def _message(
    name: str,
    *,
    position=(1.5, 0.0, 1.0),
    quaternion=(0.0, 0.0, 0.0, 1.0),
    frame: int = 1,
    source_time_s: float = 1.0,
    publish_time_us: int = 1_000_000,
    valid: bool = True,
    occluded: bool = False,
) -> transformation_t:
    message = transformation_t()
    message.name = name
    message.vicon_frame_number = int(frame)
    message.vicon_time_s = float(source_time_s)
    message.publish_time_us = int(publish_time_us)
    message.valid = int(bool(valid))
    message.occluded = int(bool(occluded))
    message.pos_vicon = list(position)
    message.quat_vicon = list(quaternion)
    return transformation_t.decode(message.encode())


def _payload(*args, **kwargs) -> bytes:
    return _message(*args, **kwargs).encode()


class _Clock:
    def __init__(self, start: float = 10.0) -> None:
        self.now = float(start)
        self._lock = threading.Lock()

    def __call__(self) -> float:
        with self._lock:
            return float(self.now)

    def advance(self, dt: float) -> None:
        with self._lock:
            self.now += float(dt)


class _BaseProvider:
    def __call__(self) -> BasePoseW:
        return BasePoseW(
            position_w=np.array([-0.4, 0.0, 0.79], dtype=np.float64),
            quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64),
            valid=True,
            simulation_time_s=1.25,
            captured_monotonic_s=time.monotonic(),
        )


def _pipeline(
    *,
    clock: _Clock | None = None,
    stale_timeout_s: float | None = 0.1,
) -> RealtimeViconBallPipeline:
    return RealtimeViconBallPipeline(
        {
            "state_estimator_window_size": 31,
            "state_estimator_min_samples": 1,
            "state_estimator_sample_rate_hz": 360.0,
            "table_height": TABLE_CENTER[2],
            "table_center_xy_w": [TABLE_CENTER[0], TABLE_CENTER[1]],
            "table_length": 2.730738,
            "table_width": 1.512451,
        },
        _BaseProvider(),
        stale_timeout_s=stale_timeout_s,
        monotonic_fn=time.monotonic if clock is None else clock,
    )


def _settings(**overrides) -> ChingMuGhostSourceSettings:
    values = {
        "lcm_url": LCM_URL,
        "channel": CHANNEL,
        "stale_timeout_s": 0.1,
        "expected_table_center_w": TABLE_CENTER,
        "expected_table_quaternion_xyzw": TABLE_QUAT,
        "table_position_tolerance_m": 0.03,
        "table_angle_tolerance_deg": 3.0,
        "required_table_confirmations": 3,
        "table_length_m": 2.730738,
        "table_width_m": 1.512451,
        "ball_xy_margin_m": 0.50,
        "ball_min_height_offset_m": -0.30,
        "ball_max_height_offset_m": 2.50,
    }
    values.update(overrides)
    return ChingMuGhostSourceSettings(**values)


class _QueuedLcm:
    def __init__(self, url: str = LCM_URL) -> None:
        self.url = url
        self.subscriptions = []
        self.unsubscribed = []
        self.handled_count = 0
        self._queue = []
        self._lock = threading.Lock()
        self._read_fd, self._write_fd = os.pipe()

    def fileno(self):
        return self._read_fd

    def subscribe(self, channel, handler):
        subscription = SimpleNamespace(channel=channel, handler=handler)
        self.subscriptions.append(subscription)
        return subscription

    def unsubscribe(self, subscription):
        self.unsubscribed.append(subscription)

    def push(self, payload: bytes, *, channel: str = CHANNEL) -> None:
        with self._lock:
            self._queue.append((channel, payload))
        try:
            os.write(self._write_fd, b"x")
        except OSError:
            pass

    def handle(self):
        try:
            os.read(self._read_fd, 1)
        except OSError:
            pass
        with self._lock:
            if not self._queue:
                return 0
            channel, payload = self._queue.pop(0)
        self.handled_count += 1
        for subscription in list(self.subscriptions):
            if subscription.channel == channel:
                subscription.handler(channel, payload)
        return 1

    def handle_timeout(self, timeout_ms):
        with self._lock:
            has_item = bool(self._queue)
        if has_item:
            return self.handle()
        time.sleep(min(float(timeout_ms) / 1000.0, 0.002))
        return 0

    def close_pipe(self) -> None:
        for fd in (self._read_fd, self._write_fd):
            try:
                os.close(fd)
            except OSError:
                pass


class _BlockingLcm(_QueuedLcm):
    def __init__(self, url: str = LCM_URL) -> None:
        super().__init__(url)
        self.entered = threading.Event()
        self.release = threading.Event()

    def handle_timeout(self, timeout_ms):
        self.entered.set()
        self.release.wait(timeout=1.0)
        return 0


class _ClosingLcm(_QueuedLcm):
    def __init__(self, url: str = LCM_URL) -> None:
        super().__init__(url)
        self.entered = threading.Event()

    def handle_timeout(self, timeout_ms):
        self.entered.set()
        time.sleep(min(float(timeout_ms) / 1000.0, 0.002))
        raise OSError("lcm_handle_timeout() returned -1")


class _SpyPipeline:
    def __init__(self) -> None:
        self.messages = []
        self.expire_calls = 0
        self.listeners = []
        self.reset_count = 0
        self.closed = False

    def ingest_transformation_update(self, msg, *, received_monotonic_s=None):
        self.messages.append((msg, received_monotonic_s))
        return SimpleNamespace(
            snapshot=None,
            sample_count=0,
            bounce_detected=False,
            previous_track_epoch=0,
            new_track_epoch=0,
            reset_reason=None,
            rejected_reason=None,
        )

    def expire_stale(self, *, now=None):
        self.expire_calls += 1
        return None

    def register_listener(self, listener):
        self.listeners.append(listener)

        def unregister():
            self.listeners.remove(listener)

        return unregister

    def reset_after_strike(self):
        self.reset_count += 1
        return self.reset_count

    def state(self):
        return SimpleNamespace(
            position_w=np.zeros(3, dtype=np.float32),
            velocity_w=np.zeros(3, dtype=np.float32),
            visible=False,
            ready=False,
            sample_count=0,
            track_epoch=self.reset_count,
            generation=0,
            latest_snapshot=None,
            last_received_monotonic_s=None,
        )

    def close(self):
        self.closed = True


def _source(
    *,
    pipeline=None,
    lcm_instance: _QueuedLcm | None = None,
    clock: _Clock | None = None,
    decoder=transformation_t.decode,
    settings: ChingMuGhostSourceSettings | None = None,
    audit: PublicationAudit | None = None,
) -> tuple[ChingMuGhostBallSource, _QueuedLcm, PublicationAudit]:
    lcm_obj = _QueuedLcm() if lcm_instance is None else lcm_instance
    publication_audit = PublicationAudit() if audit is None else audit
    source = ChingMuGhostBallSource(
        _settings() if settings is None else settings,
        pipeline=_pipeline(clock=clock) if pipeline is None else pipeline,
        decoder=decoder,
        lcm_factory=lambda _url: lcm_obj,
        publication_audit=publication_audit,
        monotonic_fn=time.monotonic if clock is None else clock,
    )
    return source, lcm_obj, publication_audit


def _wait_for(predicate, *, timeout_s: float = 1.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("condition was not reached before timeout")


def _unlock_table(source: ChingMuGhostBallSource, lcm_obj: _QueuedLcm) -> None:
    source.start()
    for frame in range(1, 4):
        lcm_obj.push(
            _payload(
                "table",
                position=TABLE_CENTER,
                quaternion=TABLE_QUAT,
                frame=frame,
                source_time_s=float(frame),
            )
        )
    _wait_for(lambda: source.source_status().table_ready)


class ReadOnlyLcmFacadeTests(unittest.TestCase):
    def test_publish_deny_delegates_receive_apis_and_records_publish_attempts(self) -> None:
        inner = _QueuedLcm()
        audit = PublicationAudit()
        facade = PublishDenyLcm(inner, audit)
        received = []

        subscription = facade.subscribe(CHANNEL, lambda channel, data: received.append((channel, data)))
        inner.push(b"payload")
        self.assertEqual(facade.fileno(), inner.fileno())
        self.assertEqual(facade.handle_timeout(1), 1)
        self.assertEqual(received, [(CHANNEL, b"payload")])
        facade.unsubscribe(subscription)

        with self.assertRaises(RuntimeError):
            facade.publish("pd_plustau_targets", b"blocked")
        self.assertEqual(audit.snapshot(), (1, ("pd_plustau_targets",)))
        inner.close_pipe()

    def test_subscription_lifecycle_is_idempotent_and_join_timeout_is_observable(self) -> None:
        lcm_obj = _BlockingLcm()
        handled = []
        subscription = ReadOnlyLcmSubscription(
            lcm_url=LCM_URL,
            channel=CHANNEL,
            handler=lambda channel, data: handled.append((channel, data)),
            lcm_factory=lambda _url: lcm_obj,
            poll_timeout_ms=1,
        )

        self.assertFalse(hasattr(subscription, "publish"))
        subscription.start()
        subscription.start()
        _wait_for(lambda: bool(lcm_obj.subscriptions))
        _wait_for(lambda: lcm_obj.entered.is_set())
        self.assertEqual(len(lcm_obj.subscriptions), 1)
        self.assertFalse(subscription.close(join_timeout_s=0.001))
        self.assertEqual(len(lcm_obj.unsubscribed), 0)
        lcm_obj.release.set()
        self.assertTrue(subscription.close(join_timeout_s=1.0))
        self.assertEqual(len(lcm_obj.unsubscribed), 1)
        self.assertTrue(subscription.close(join_timeout_s=1.0))
        with self.assertRaises(RuntimeError):
            subscription.start()
        lcm_obj.close_pipe()

    def test_subscription_treats_negative_lcm_timeout_as_close_signal(self) -> None:
        lcm_obj = _ClosingLcm()
        subscription = ReadOnlyLcmSubscription(
            lcm_url=LCM_URL,
            channel=CHANNEL,
            handler=lambda _channel, _data: None,
            lcm_factory=lambda _url: lcm_obj,
            poll_timeout_ms=1,
        )

        subscription._lc = lcm_obj
        subscription._run()
        self.assertTrue(lcm_obj.entered.is_set())
        lcm_obj.close_pipe()

    def test_basesim_poll_uses_facade_fileno_and_handle(self) -> None:
        inner = _QueuedLcm()
        facade = PublishDenyLcm(inner, PublicationAudit())
        sim = BaseSim.__new__(BaseSim)
        sim.lc = facade
        handle_calls = []

        def fake_select(reads, _writes, _errors, _timeout):
            self.assertEqual(reads, [inner.fileno()])
            if not handle_calls:
                return reads, [], []
            raise KeyboardInterrupt

        def fake_handle():
            handle_calls.append("handled")
            return 1

        with mock.patch("simulator.base_sim.select.select", side_effect=fake_select):
            with mock.patch.object(facade, "handle", side_effect=fake_handle):
                sim.poll()
        self.assertEqual(handle_calls, ["handled"])
        inner.close_pipe()


class ChingMuGhostBallSourceTests(unittest.TestCase):
    def test_interleaved_table_gate_ignores_pelvis_ball_and_unknown_subjects(self) -> None:
        spy = _SpyPipeline()
        source, lcm_obj, audit = _source(pipeline=spy)
        source.start()
        self.assertFalse(hasattr(source, "publish"))
        self.assertEqual([sub.channel for sub in lcm_obj.subscriptions], [CHANNEL])

        for frame in range(1, 4):
            lcm_obj.push(_payload("G2Pelvis", frame=frame))
            lcm_obj.push(_payload("ball", frame=frame))
            lcm_obj.push(_payload("unknown", frame=frame))
            lcm_obj.push(
                _payload(
                    "table",
                    position=TABLE_CENTER,
                    quaternion=TABLE_QUAT,
                    frame=frame,
                )
            )
        _wait_for(lambda: source.source_status().table_ready)

        status = source.source_status()
        self.assertIsInstance(status, ChingMuGhostSourceStatus)
        self.assertEqual(status.table_confirmation_count, 3)
        self.assertEqual(status.decoded_count, 12)
        self.assertEqual(status.accepted_ball_count, 0)
        self.assertEqual(len(spy.messages), 0)
        self.assertGreaterEqual(spy.expire_calls, 1)
        self.assertEqual(audit.snapshot(), (0, ()))
        with self.assertRaises(FrozenInstanceError):
            status.table_ready = False

        source.close()
        lcm_obj.close_pipe()

    def test_table_mismatch_locks_gate_and_invalidates_active_track_once(self) -> None:
        pipeline = _pipeline()
        source, lcm_obj, _audit = _source(pipeline=pipeline)
        snapshots = []
        source.register_hitter_ball_listener(snapshots.append)
        _unlock_table(source, lcm_obj)

        lcm_obj.push(_payload("ball", position=(1.5, 0.0, 1.0), frame=10, source_time_s=10.0))
        _wait_for(lambda: source.state().visible)
        self.assertEqual(source.state().track_epoch, 0)

        lcm_obj.push(
            _payload(
                "table",
                position=(TABLE_CENTER[0] + 0.5, TABLE_CENTER[1], TABLE_CENTER[2]),
                quaternion=TABLE_QUAT,
                frame=11,
                source_time_s=11.0,
            )
        )
        _wait_for(lambda: source.state().track_epoch == 1)
        status = source.source_status()
        self.assertFalse(status.table_ready)
        self.assertEqual(status.table_confirmation_count, 0)
        self.assertEqual(status.last_rejection_reason, "TABLE_MISMATCH")
        self.assertFalse(source.state().visible)
        self.assertEqual([snapshot.visible for snapshot in snapshots][-1], False)

        lcm_obj.push(
            _payload(
                "table",
                position=(TABLE_CENTER[0] + 0.5, TABLE_CENTER[1], TABLE_CENTER[2]),
                quaternion=TABLE_QUAT,
                frame=12,
                source_time_s=12.0,
            )
        )
        time.sleep(0.05)
        self.assertEqual(source.state().track_epoch, 1)

        source.close()
        lcm_obj.close_pipe()

    def test_out_of_bounds_and_occluded_ball_use_public_invalid_pipeline_path(self) -> None:
        pipeline = _pipeline()
        source, lcm_obj, _audit = _source(pipeline=pipeline)
        snapshots = []
        source.register_hitter_ball_listener(snapshots.append)
        _unlock_table(source, lcm_obj)

        lcm_obj.push(_payload("ball", position=(1.5, 0.0, 1.0), frame=20, source_time_s=20.0))
        _wait_for(lambda: source.state().visible)
        lcm_obj.push(_payload("ball", position=(4.0, 0.0, 1.0), frame=21, source_time_s=21.0))
        _wait_for(lambda: source.state().track_epoch == 1)
        self.assertEqual(source.source_status().last_rejection_reason, "BALL_OUT_OF_BOUNDS")
        self.assertFalse(snapshots[-1].visible)
        self.assertEqual(pipeline.ball_state_estimator.sample_count, 0)

        lcm_obj.push(_payload("ball", position=(4.0, 0.0, 1.0), frame=22, source_time_s=22.0))
        time.sleep(0.05)
        self.assertEqual(source.state().track_epoch, 1)

        lcm_obj.push(_payload("ball", position=(1.4, 0.0, 1.2), frame=23, source_time_s=23.0))
        _wait_for(lambda: source.state().visible)
        lcm_obj.push(
            _payload(
                "ball",
                position=(float("nan"), 0.0, 1.0),
                frame=24,
                source_time_s=24.0,
                valid=False,
                occluded=True,
            )
        )
        _wait_for(lambda: source.state().track_epoch == 2)
        self.assertEqual(source.source_status().last_rejection_reason, "BALL_INVALID_OR_OCCLUDED")

        source.close()
        lcm_obj.close_pipe()

    def test_table_quaternion_sign_equivalence_zero_rejection_and_boundary_acceptance(self) -> None:
        source, lcm_obj, _audit = _source()
        source.start()
        for frame in range(1, 4):
            lcm_obj.push(
                _payload(
                    "table",
                    position=TABLE_CENTER,
                    quaternion=(0.0, 0.0, 0.0, -1.0),
                    frame=frame,
                )
            )
        _wait_for(lambda: source.source_status().table_ready)

        lcm_obj.push(_payload("table", position=TABLE_CENTER, quaternion=(0.0, 0.0, 0.0, 0.0), frame=4))
        _wait_for(lambda: not source.source_status().table_ready)
        self.assertEqual(source.source_status().last_rejection_reason, "TABLE_INVALID")

        for frame in range(5, 8):
            lcm_obj.push(_payload("table", position=TABLE_CENTER, quaternion=TABLE_QUAT, frame=frame))
        _wait_for(lambda: source.source_status().table_ready)
        lcm_obj.push(
            _payload(
                "ball",
                position=(-0.50, -1.2562255, 0.46),
                frame=8,
                source_time_s=8.0,
            )
        )
        _wait_for(lambda: source.source_status().accepted_ball_count == 1)
        np.testing.assert_allclose(source.state().position_w, [-0.50, -1.2562255, 0.46])

        source.close()
        lcm_obj.close_pipe()

    def test_stale_expiration_runs_even_during_continuous_non_ball_traffic(self) -> None:
        clock = _Clock(start=100.0)
        pipeline = _pipeline(clock=clock, stale_timeout_s=0.05)
        source, lcm_obj, _audit = _source(pipeline=pipeline, clock=clock)
        _unlock_table(source, lcm_obj)
        lcm_obj.push(_payload("ball", position=(1.5, 0.0, 1.0), frame=30, source_time_s=30.0))
        _wait_for(lambda: source.state().visible)

        clock.advance(0.20)
        for frame in range(31, 40):
            lcm_obj.push(_payload("G2Pelvis", frame=frame))
        _wait_for(lambda: source.state().track_epoch == 1)
        self.assertFalse(source.state().visible)
        self.assertEqual(source.source_status().last_rejection_reason, "BALL_STALE")

        source.close()
        lcm_obj.close_pipe()

    def test_corrupt_payload_records_failure_and_next_good_payload_is_decoded(self) -> None:
        source, lcm_obj, _audit = _source()
        source.start()
        lcm_obj.push(b"not a transformation")
        _wait_for(lambda: source.source_status().receive_failure is not None)
        failure = source.source_status().receive_failure
        lcm_obj.push(_payload("table", position=TABLE_CENTER, quaternion=TABLE_QUAT, frame=1))
        _wait_for(lambda: source.source_status().decoded_count == 1)
        self.assertEqual(source.source_status().receive_failure, failure)

        source.close()
        lcm_obj.close_pipe()

    def test_source_start_close_status_and_reset_delegate(self) -> None:
        spy = _SpyPipeline()
        source, lcm_obj, audit = _source(pipeline=spy)
        source.start()
        source.start()
        _wait_for(lambda: source.source_status().thread_alive)
        self.assertEqual(len(lcm_obj.subscriptions), 1)
        self.assertEqual(source.reset_ball_state_estimator(), 1)
        self.assertEqual(source.state().track_epoch, 1)
        self.assertTrue(source.close(join_timeout_s=1.0))
        status = source.source_status()
        self.assertFalse(status.thread_alive)
        self.assertTrue(status.closed)
        self.assertTrue(status.last_close_joined)
        self.assertEqual(len(lcm_obj.unsubscribed), 1)
        self.assertTrue(source.close(join_timeout_s=1.0))
        with self.assertRaises(RuntimeError):
            source.start()
        self.assertTrue(spy.closed)
        self.assertEqual(audit.snapshot(), (0, ()))
        lcm_obj.close_pipe()

    def test_static_safety_contract_excludes_realworld_sdk_startup_and_publish_calls(self) -> None:
        root = os.path.dirname(os.path.dirname(__file__))
        production_paths = [
            os.path.join(root, "utils", "read_only_lcm.py"),
            os.path.join(root, "utils", "hitter_chingmu_ghost.py"),
        ]
        combined = "\n".join(open(path, "r", encoding="utf-8").read() for path in production_paths)
        self.assertNotIn("simulator.real_world", combined)
        self.assertNotIn("RealWorld", combined)
        self.assertNotIn("chingmu_sdk", combined)
        self.assertNotIn("pd_plustau_targets", combined)
        publish_occurrences = [
            line.strip()
            for line in combined.splitlines()
            if ".publish(" in line or "def publish(" in line
        ]
        self.assertEqual(publish_occurrences, ["def publish(self, channel, payload):"])


if __name__ == "__main__":
    unittest.main()
