from __future__ import annotations

import math
import threading
import unittest
from dataclasses import FrozenInstanceError
from types import SimpleNamespace
from unittest import mock

import numpy as np

from diagnostics.hitter_task_pipeline import (
    MocapFrameAdapter,
    ProductionReset,
)
from simulator.real_world import RealWorld
from unitree_sdk2.lcm_types.transformation_t import transformation_t
from utils.hitter_ball_pipeline import BallPipelineState


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
    message.vicon_frame_number = frame
    message.vicon_time_s = source_time_s
    message.publish_time_us = publish_time_us
    message.valid = int(valid)
    message.occluded = int(occluded)
    message.pos_vicon = list(position)
    message.quat_vicon = list(quaternion)
    return transformation_t.decode(message.encode())


def _adapter(*, min_samples: int = 3) -> MocapFrameAdapter:
    return MocapFrameAdapter(
        planner_config={
            "state_estimator_window_size": 31,
            "state_estimator_min_samples": min_samples,
            "state_estimator_sample_rate_hz": 360.0,
        },
        estimator_sample_rate_hz=360.0,
    )


def _ingest(
    adapter: MocapFrameAdapter,
    message: transformation_t,
    *,
    received_s: float,
    wall_time_us: int = 1_700_000_000_000_000,
):
    return adapter.ingest_decoded(
        channel="vicon_state_data",
        message=message,
        payload_size=len(message.encode()),
        received_monotonic_s=received_s,
        wall_time_us=wall_time_us,
    )


class MocapFrameAdapterInputTests(unittest.TestCase):
    def test_real_messages_keep_global_arrival_order_and_normalized_subjects(self) -> None:
        adapter = _adapter()
        messages = [
            _message("G1Pelvis", frame=10),
            _message("BALL", frame=10),
            _message("table", frame=10),
            _message("UnknownSubject", frame=10),
        ]

        outputs = [
            _ingest(adapter, message, received_s=2.0 + index * 0.01)
            for index, message in enumerate(messages)
        ]

        self.assertEqual(
            [output.sample.input_seq for output in outputs],
            [0, 1, 2, 3],
        )
        self.assertEqual(
            [output.sample.subject for output in outputs],
            ["g1pelvis", "ball", "table", "unknownsubject"],
        )
        self.assertIsNotNone(outputs[1].snapshot)
        self.assertIsNone(outputs[0].snapshot)
        self.assertIsNone(outputs[2].snapshot)
        self.assertIsNone(outputs[3].snapshot)
        self.assertIn("UNKNOWN_SUBJECT", outputs[3].warnings)

    def test_samples_are_detached_bytes_backed_float64_but_snapshots_are_float32(self) -> None:
        adapter = _adapter(min_samples=1)
        message = _message(
            "ball",
            position=(1.25, -0.5, 0.875),
            quaternion=(0.1, 0.2, 0.3, 0.9),
        )
        output = _ingest(adapter, message, received_s=2.0)

        message.pos_vicon = [99.0, -0.5, 0.875]
        self.assertEqual(output.sample.position_w.dtype, np.dtype(np.float64))
        self.assertEqual(
            output.sample.quaternion_xyzw.dtype,
            np.dtype(np.float64),
        )
        self.assertFalse(output.sample.position_w.flags.writeable)
        with self.assertRaises(ValueError):
            output.sample.position_w.setflags(write=True)
        self.assertEqual(output.snapshot.position_w.dtype, np.dtype(np.float32))
        self.assertEqual(output.snapshot.velocity_w.dtype, np.dtype(np.float32))
        self.assertEqual(
            output.snapshot.base_position_w.dtype,
            np.dtype(np.float32),
        )
        self.assertEqual(
            output.snapshot.base_quaternion_xyzw.dtype,
            np.dtype(np.float32),
        )
        for array in (
            output.snapshot.position_w,
            output.snapshot.velocity_w,
            output.snapshot.base_position_w,
            output.snapshot.base_quaternion_xyzw,
        ):
            self.assertFalse(array.flags.writeable)
            with self.assertRaises(ValueError):
                array.setflags(write=True)
        np.testing.assert_array_equal(
            output.sample.position_w,
            [1.25, -0.5, 0.875],
        )

    def test_unknown_subjects_do_not_grow_or_cross_contaminate_warning_history(self) -> None:
        adapter = _adapter()
        first = _ingest(
            adapter,
            _message(
                "UnknownSubject",
                frame=1,
                source_time_s=10.0,
                publish_time_us=1_010_000_000,
            ),
            received_s=1.0,
        )
        second = _ingest(
            adapter,
            _message(
                "unknownsubject",
                frame=100,
                source_time_s=10.01,
                publish_time_us=1_010_010_000,
            ),
            received_s=1.01,
        )

        self.assertEqual(first.warnings, ("UNKNOWN_SUBJECT",))
        self.assertEqual(second.warnings, ("UNKNOWN_SUBJECT",))

    def test_timestamp_priority_and_independent_zero_based_nominal_fallback(self) -> None:
        adapter = _adapter(min_samples=1)
        messages = [
            _message(
                "ball",
                frame=1,
                source_time_s=10.0,
                publish_time_us=90_000_000,
            ),
            _message(
                "ball",
                frame=2,
                source_time_s=float("nan"),
                publish_time_us=20_000_000,
            ),
            _message(
                "ball",
                frame=3,
                source_time_s=0.0,
                publish_time_us=0,
            ),
            _message(
                "ball",
                frame=4,
                source_time_s=-1.0,
                publish_time_us=-1,
            ),
        ]
        for index, message in enumerate(messages):
            _ingest(adapter, message, received_s=1000.0 + index * 50.0)

        timestamps = [
            float(timestamp)
            for timestamp, _position in adapter.ball_state_estimator._samples
        ]
        np.testing.assert_allclose(
            timestamps,
            [
                10.0,
                10.0 + 1.0 / 360.0,
                10.0 + 2.0 / 360.0,
                10.0 + 3.0 / 360.0,
            ],
            rtol=0.0,
            atol=1.0e-12,
        )
        self.assertAlmostEqual(
            adapter.fallback_estimator_time_s,
            1.0 / 360.0,
        )

    def test_explicit_estimator_sample_rate_override_drives_fallback(self) -> None:
        adapter = MocapFrameAdapter(
            planner_config={
                "state_estimator_window_size": 31,
                "state_estimator_min_samples": 1,
                "state_estimator_sample_rate_hz": 100.0,
            },
            estimator_sample_rate_hz=400.0,
        )
        for frame in (1, 2):
            _ingest(
                adapter,
                _message(
                    "ball",
                    frame=frame,
                    source_time_s=0.0,
                    publish_time_us=0,
                ),
                received_s=float(frame),
            )

        np.testing.assert_allclose(
            [
                float(timestamp)
                for timestamp, _position in adapter.ball_state_estimator._samples
            ],
            [0.0, 1.0 / 400.0],
            rtol=0.0,
            atol=1.0e-12,
        )
        self.assertAlmostEqual(
            adapter.fallback_estimator_time_s,
            1.0 / 400.0,
        )

    def test_duplicate_and_backward_source_times_are_rejected_within_epoch(self) -> None:
        adapter = _adapter(min_samples=1)
        outputs = []
        for frame, source_time in enumerate((5.0, 5.0, 4.0), start=1):
            outputs.append(
                _ingest(
                    adapter,
                    _message(
                        "ball",
                        frame=frame,
                        source_time_s=source_time,
                    ),
                    received_s=float(frame),
                )
            )
        timestamps = [
            float(timestamp)
            for timestamp, _position in adapter.ball_state_estimator._samples
        ]
        np.testing.assert_allclose(timestamps, [5.0])
        self.assertIsNotNone(outputs[0].snapshot)
        self.assertIsNone(outputs[1].snapshot)
        self.assertIsNone(outputs[2].snapshot)
        self.assertIn(
            "NON_MONOTONIC_SOURCE_TIMESTAMP",
            outputs[1].warnings,
        )
        self.assertIn(
            "NON_MONOTONIC_SOURCE_TIMESTAMP",
            outputs[2].warnings,
        )

    def test_ball_snapshot_binds_last_accepted_pose_and_current_validity(self) -> None:
        adapter = _adapter(min_samples=1)
        valid_pose = _message(
            "G1Pelvis",
            position=(0.2, -0.3, 0.8),
            quaternion=(0.0, 0.0, 0.25, 0.9682458365518543),
            frame=100,
            source_time_s=1.0,
        )
        _ingest(adapter, valid_pose, received_s=10.0)
        invalid_pose = _message(
            "g1pelvis",
            position=(9.0, 9.0, 9.0),
            quaternion=(1.0, 0.0, 0.0, 0.0),
            frame=101,
            source_time_s=1.01,
            valid=False,
        )
        invalid_output = _ingest(adapter, invalid_pose, received_s=10.01)
        latest = adapter.copy_latest_pelvis()
        np.testing.assert_allclose(latest.position_w, [0.2, -0.3, 0.8])
        self.assertFalse(latest.valid)
        self.assertEqual(latest.source_frame, 101)
        self.assertEqual(latest.source_time_s, 1.01)
        self.assertEqual(latest.received_monotonic_s, 10.01)
        self.assertIn("PELVIS_INVALID", invalid_output.warnings)

        ball_output = _ingest(
            adapter,
            _message("ball", frame=101, source_time_s=1.01),
            received_s=10.011,
        )
        np.testing.assert_allclose(
            ball_output.snapshot.base_position_w,
            [0.2, -0.3, 0.8],
        )
        self.assertFalse(ball_output.snapshot.base_valid)
        with self.assertRaises(ValueError):
            latest.position_w.setflags(write=True)
        with self.assertRaises(FrozenInstanceError):
            latest.valid = True

    def test_frame_mismatch_staleness_and_source_gap_warn_without_changing_base_valid(self) -> None:
        adapter = _adapter(min_samples=1)
        _ingest(
            adapter,
            _message("g1pelvis", frame=10, source_time_s=1.0),
            received_s=20.0,
        )
        _ingest(
            adapter,
            _message("ball", frame=10, source_time_s=1.0),
            received_s=20.001,
        )
        output = _ingest(
            adapter,
            _message("ball", frame=15, source_time_s=1.2),
            received_s=20.051,
        )

        self.assertTrue(output.snapshot.base_valid)
        self.assertIn("PELVIS_FRAME_MISMATCH", output.warnings)
        self.assertIn("SOURCE_FRAME_GAP", output.warnings)

    def test_clock_offset_is_inferred_from_consecutive_source_publish_pairs_only(self) -> None:
        adapter = _adapter(min_samples=1)
        first = _ingest(
            adapter,
            _message(
                "table",
                frame=1,
                source_time_s=10.0,
                publish_time_us=1_010_000_000,
            ),
            received_s=500.0,
            wall_time_us=9_000_000_000_000_000,
        )
        second = _ingest(
            adapter,
            _message(
                "table",
                frame=2,
                source_time_s=10.01,
                publish_time_us=1_010_010_000,
            ),
            received_s=500.01,
            wall_time_us=9_100_000_000_000_000,
        )

        self.assertNotIn("CLOCK_OFFSET_SUSPECTED", first.warnings)
        self.assertIn("CLOCK_OFFSET_SUSPECTED", second.warnings)
        self.assertFalse(
            any(
                "LATENCY" in warning or "DELAY" in warning
                for warning in second.warnings
            )
        )

    def test_invalid_ball_resets_epoch_emits_invisible_snapshot_and_retains_state(self) -> None:
        adapter = _adapter(min_samples=1)
        visible = _ingest(
            adapter,
            _message("ball", position=(1.4, 0.1, 1.1), frame=1),
            received_s=1.0,
        )
        invalid = _ingest(
            adapter,
            _message(
                "ball",
                position=(0.0, 0.0, 0.0),
                frame=2,
                source_time_s=1.01,
                valid=False,
            ),
            received_s=1.01,
        )

        self.assertEqual(visible.snapshot.track_epoch, 0)
        self.assertEqual(visible.snapshot.generation, 1)
        self.assertEqual(invalid.snapshot.track_epoch, 1)
        self.assertEqual(invalid.snapshot.generation, 2)
        self.assertFalse(invalid.snapshot.visible)
        self.assertFalse(invalid.snapshot.ready)
        np.testing.assert_array_equal(
            invalid.snapshot.position_w,
            visible.snapshot.position_w,
        )
        np.testing.assert_array_equal(
            invalid.snapshot.velocity_w,
            visible.snapshot.velocity_w,
        )
        self.assertEqual(invalid.estimator_sample_count, 0)
        self.assertEqual(
            invalid.reset_transition,
            ProductionReset(0, 1, "BALL_INVALID_OR_OCCLUDED"),
        )
        self.assertIn("BALL_INVALID_OR_OCCLUDED", invalid.warnings)
        self.assertFalse(hasattr(adapter, "incoming"))
        self.assertFalse(hasattr(adapter, "lifecycle"))

    def test_repeated_invalid_ball_preserves_adapter_transition_metadata_exactly_once(self) -> None:
        adapter = _adapter(min_samples=1)
        _ingest(
            adapter,
            _message("ball", frame=1, source_time_s=1.0),
            received_s=1.0,
        )
        first = _ingest(
            adapter,
            _message("ball", frame=2, source_time_s=2.0, valid=False),
            received_s=2.0,
        )
        repeated = _ingest(
            adapter,
            _message("ball", frame=3, source_time_s=3.0, occluded=True),
            received_s=3.0,
        )

        self.assertEqual(
            first.reset_transition,
            ProductionReset(0, 1, "BALL_INVALID_OR_OCCLUDED"),
        )
        self.assertIsNone(repeated.reset_transition)
        self.assertIsNone(repeated.snapshot)
        self.assertEqual(adapter.track_epoch, 1)
        self.assertEqual(adapter.ball_snapshot_generation, 2)

    def test_strike_reset_only_advances_epoch_and_preserves_latest_snapshot(self) -> None:
        adapter = _adapter(min_samples=1)
        first = _ingest(
            adapter,
            _message("ball", frame=1),
            received_s=1.0,
        )

        new_epoch = adapter.reset_estimator_after_strike()

        self.assertEqual(new_epoch, 1)
        self.assertIs(adapter.latest_ball_snapshot, first.snapshot)
        second = _ingest(
            adapter,
            _message("ball", frame=2, source_time_s=2.0),
            received_s=2.0,
        )
        self.assertEqual(second.snapshot.track_epoch, 1)
        self.assertEqual(second.snapshot.generation, 2)
        self.assertEqual(second.estimator_sample_count, 1)

    def test_bounce_resets_fit_buffer_without_advancing_epoch(self) -> None:
        adapter = _adapter(min_samples=3)
        positions = (
            (1.4, 0.0, 0.95),
            (1.4, 0.0, 0.90),
            (1.4, 0.0, 0.79),
        )
        outputs = [
            _ingest(
                adapter,
                _message(
                    "ball",
                    position=position,
                    frame=index,
                    source_time_s=float(index),
                ),
                received_s=float(index),
            )
            for index, position in enumerate(positions, start=1)
        ]

        self.assertTrue(outputs[-1].bounce_detected)
        self.assertEqual(outputs[-1].estimator_sample_count, 1)
        self.assertEqual(outputs[-1].snapshot.track_epoch, 0)
        self.assertIn("BOUNCE_RESET", outputs[-1].warnings)

    def test_estimator_becomes_ready_on_the_thirty_first_sample(self) -> None:
        adapter = _adapter(min_samples=31)
        outputs = []
        for index in range(31):
            outputs.append(
                _ingest(
                    adapter,
                    _message(
                        "ball",
                        position=(1.8 - 0.01 * index, 0.0, 1.2),
                        frame=index + 1,
                        source_time_s=1.0 + index / 360.0,
                    ),
                    received_s=2.0 + index / 360.0,
                )
            )
        self.assertFalse(outputs[-2].snapshot.ready)
        self.assertEqual(outputs[-2].estimator_sample_count, 30)
        self.assertTrue(outputs[-1].snapshot.ready)
        self.assertEqual(outputs[-1].estimator_sample_count, 31)

    def test_near_zero_pelvis_quaternion_is_invalid_for_both_adapters(self) -> None:
        adapter = _adapter(min_samples=1)
        tiny_quaternion = (1.0e-300, -1.0e-300, 1.0e-300, -1.0e-300)
        diagnostic_pelvis = _message(
            "g1pelvis",
            quaternion=tiny_quaternion,
            frame=7,
        )
        pelvis_output = _ingest(
            adapter,
            diagnostic_pelvis,
            received_s=1.0,
        )
        ball_output = _ingest(
            adapter,
            _message("ball", frame=7),
            received_s=1.001,
        )

        real = RealWorld.__new__(RealWorld)
        real.cfg = SimpleNamespace(
            motion={
                "ball_planner": {
                    "state_estimator_window_size": 31,
                    "state_estimator_min_samples": 1,
                    "state_estimator_sample_rate_hz": 360.0,
                }
            }
        )
        real._init_ball_state()
        real.firstReceiveVicon = False
        runtime_pelvis = _message(
            "G2Pelvis",
            quaternion=tiny_quaternion,
            frame=7,
        )
        real._vicon_state_handler(
            "vicon_state_data",
            runtime_pelvis.encode(),
        )

        self.assertEqual(
            pelvis_output.sample.valid,
            bool(diagnostic_pelvis.valid),
        )
        self.assertIn("PELVIS_INVALID", pelvis_output.warnings)
        self.assertFalse(real.base_pose_valid_tmp)
        self.assertFalse(ball_output.snapshot.base_valid)
        np.testing.assert_array_equal(
            ball_output.snapshot.base_quaternion_xyzw,
            real.root_quat_world_tmp,
        )

    def test_replay_identity_setters_and_realworld_public_pipeline_proxies_remain_compatible(self) -> None:
        adapter = _adapter(min_samples=1)
        adapter.track_epoch = 7
        adapter.ball_snapshot_generation = 10
        output = _ingest(
            adapter,
            _message("ball", frame=1, source_time_s=1.0),
            received_s=1.0,
        )
        self.assertEqual(output.snapshot.track_epoch, 7)
        self.assertEqual(output.snapshot.generation, 11)
        self.assertIs(adapter.latest_ball_snapshot, output.snapshot)
        self.assertIs(
            adapter.ball_state_estimator,
            adapter.ball_pipeline.ball_state_estimator,
        )
        self.assertIsInstance(adapter.lock, type(threading.RLock()))

        real = RealWorld.__new__(RealWorld)
        real.cfg = SimpleNamespace(
            motion={
                "ball_planner": {
                    "state_estimator_window_size": 31,
                    "state_estimator_min_samples": 1,
                    "state_estimator_sample_rate_hz": 360.0,
                }
            }
        )
        real.root_trans_world_tmp = np.zeros(3, dtype=np.float32)
        real.root_quat_world_tmp = np.array(
            [0.0, 0.0, 0.0, 1.0],
            dtype=np.float32,
        )
        real._init_ball_state()
        real.firstReceiveVicon = False
        pelvis = _message(
            "G2Pelvis",
            position=(-0.4, -0.06, 0.61),
            quaternion=(0.0, 0.0, 0.2, 0.98),
            frame=1,
        )
        real._vicon_state_handler("vicon_state_data", pelvis.encode())
        self.assertEqual(real.ball_state_estimator_sample_count_tmp, 0)

        received = []
        unregister = real.register_hitter_ball_listener(received.append)
        real._vicon_state_handler(
            "vicon_state_data",
            _message(
                "ball",
                position=(1.4, 0.1, 1.1),
                frame=2,
                source_time_s=2.0,
            ).encode(),
        )
        self.assertIsNone(real._hitter_ball_pipeline.stale_timeout_s)
        self.assertEqual(real.ball_track_epoch, 0)
        self.assertEqual(real.ball_snapshot_generation, 1)
        self.assertIs(real.latest_ball_snapshot, received[-1])
        self.assertTrue(real.ball_visible_tmp)
        self.assertTrue(real.ball_state_estimator_ready_tmp)
        self.assertEqual(real.ball_state_estimator_sample_count_tmp, 1)
        np.testing.assert_allclose(
            real.ball_pos_world_tmp,
            received[-1].position_w,
        )
        np.testing.assert_allclose(
            received[-1].base_position_w,
            [-0.4, -0.06, 0.61],
        )
        np.testing.assert_allclose(
            np.linalg.norm(received[-1].base_quaternion_xyzw),
            1.0,
            rtol=0.0,
            atol=1.0e-6,
        )
        unregister()
        unregister()

    def test_realworld_accepts_g2pelvis_as_the_base_subject(self) -> None:
        real = RealWorld.__new__(RealWorld)
        real.cfg = SimpleNamespace(motion={"ball_planner": {}})
        real._init_ball_state()
        real.firstReceiveVicon = False

        pelvis = _message(
            "G2Pelvis",
            position=(-0.4, -0.06, 0.61),
            quaternion=(0.01, 0.02, -0.03, 0.999),
        )
        real._vicon_state_handler("vicon_state_data", pelvis.encode())

        self.assertTrue(real.base_pose_valid_tmp)
        self.assertTrue(real.firstReceiveVicon)
        np.testing.assert_allclose(
            real.root_trans_world_tmp,
            [-0.4, -0.06, 0.61],
        )

    def test_realworld_reset_supersedes_provider_blocked_valid_ingest(self) -> None:
        real = RealWorld.__new__(RealWorld)
        real.cfg = SimpleNamespace(
            motion={
                "ball_planner": {
                    "state_estimator_window_size": 31,
                    "state_estimator_min_samples": 1,
                    "state_estimator_sample_rate_hz": 360.0,
                }
            }
        )
        real.root_trans_world_tmp = np.zeros(3, dtype=np.float32)
        real.root_quat_world_tmp = np.array(
            [0.0, 0.0, 0.0, 1.0],
            dtype=np.float32,
        )
        provider_entered = threading.Event()
        provider_release = threading.Event()
        provider_lock = threading.Lock()
        provider_calls = {"count": 0}

        def blocking_base_provider():
            with provider_lock:
                provider_calls["count"] += 1
                call = provider_calls["count"]
            if call == 1:
                provider_entered.set()
                if not provider_release.wait(timeout=2.0):
                    raise RuntimeError(
                        "timed out waiting to release RealWorld base provider"
                    )
            return RealWorld._copy_base_pose_for_ball_pipeline(real)

        real._copy_base_pose_for_ball_pipeline = blocking_base_provider
        real._init_ball_state()
        real.ball_state_estimator_ready = True
        real.ball_state_estimator_sample_count = 9
        real.ball_visible = True
        observed = []
        real.register_hitter_ball_listener(observed.append)
        result = {}
        errors = []

        def ingest_provider_blocked_valid() -> None:
            try:
                result["old"] = (
                    real._hitter_ball_pipeline.ingest_transformation_update(
                        _message(
                            "ball",
                            frame=1,
                            source_time_s=67488.0,
                        ),
                        received_monotonic_s=1.0,
                    )
                )
            except Exception as exc:  # pragma: no cover - diagnostic guard
                errors.append(exc)

        thread = threading.Thread(target=ingest_provider_blocked_valid)
        thread.start()
        self.assertTrue(provider_entered.wait(timeout=1.0))

        real.reset_ball_state_estimator()
        reset_state = real._hitter_ball_pipeline.state()
        self.assertEqual(reset_state.track_epoch, 1)
        self.assertEqual(reset_state.generation, 0)
        self.assertEqual(reset_state.sample_count, 0)
        self.assertFalse(reset_state.visible)
        self.assertIsNone(reset_state.latest_snapshot)
        self.assertFalse(real.ball_state_estimator_ready)
        self.assertEqual(real.ball_state_estimator_sample_count, 0)
        self.assertFalse(real.ball_visible)

        provider_release.set()
        thread.join(timeout=1.0)
        self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(result["old"].rejected_reason, "SUPERSEDED_OPERATION")
        self.assertIsNone(result["old"].snapshot)
        self.assertEqual(observed, [])
        after_old = real._hitter_ball_pipeline.state()
        self.assertEqual(after_old.track_epoch, 1)
        self.assertEqual(after_old.generation, 0)
        self.assertEqual(after_old.sample_count, 0)
        self.assertFalse(after_old.visible)
        self.assertIsNone(after_old.latest_snapshot)

        accepted = real._hitter_ball_pipeline.ingest_transformation_update(
            _message("ball", frame=2, source_time_s=1.0),
            received_monotonic_s=2.0,
        )
        self.assertIsNotNone(accepted.snapshot)
        self.assertEqual(accepted.new_track_epoch, 1)
        self.assertEqual(len(observed), 1)
        self.assertIs(observed[0], accepted.snapshot)
        accepted_state = real._hitter_ball_pipeline.state()
        self.assertEqual(accepted_state.track_epoch, 1)
        self.assertEqual(accepted_state.generation, 1)
        self.assertEqual(accepted_state.sample_count, 1)
        self.assertTrue(accepted_state.visible)
        self.assertIs(accepted_state.latest_snapshot, accepted.snapshot)

    def test_realworld_get_state_copies_one_atomic_ball_pipeline_state(self) -> None:
        first = BallPipelineState(
            position_w=[1.0, 2.0, 3.0],
            velocity_w=[4.0, 5.0, 6.0],
            visible=True,
            ready=True,
            sample_count=31,
            track_epoch=7,
            generation=11,
            latest_snapshot=None,
            last_received_monotonic_s=1.0,
        )
        later = BallPipelineState(
            position_w=[9.0, 9.0, 9.0],
            velocity_w=[8.0, 8.0, 8.0],
            visible=False,
            ready=False,
            sample_count=0,
            track_epoch=8,
            generation=12,
            latest_snapshot=None,
            last_received_monotonic_s=2.0,
        )

        class AdvancingPipeline:
            def __init__(self) -> None:
                self.calls = 0

            def state(self):
                result = first if self.calls == 0 else later
                self.calls += 1
                return result

        real = RealWorld.__new__(RealWorld)
        real._hitter_ball_state_lock = threading.RLock()
        real._hitter_ball_pipeline = AdvancingPipeline()
        real.root_trans_tmp = np.zeros(3, dtype=np.float32)
        real.base_lin_vel_tmp = np.zeros(3, dtype=np.float32)
        real.root_quat_tmp = np.array(
            [0.0, 0.0, 0.0, 1.0],
            dtype=np.float32,
        )
        real.root_trans_world_tmp = np.zeros(3, dtype=np.float32)
        real.root_quat_world_tmp = np.array(
            [0.0, 0.0, 0.0, 1.0],
            dtype=np.float32,
        )
        real.base_pose_valid_tmp = False
        real.root_rpy_tmp = np.zeros(3, dtype=np.float32)
        real.base_ang_vel_tmp = np.zeros(3, dtype=np.float32)
        real.dof_pos_tmp = np.zeros(0, dtype=np.float32)
        real.dof_vel_tmp = np.zeros(0, dtype=np.float32)
        real.active_dof_idx = np.zeros(0, dtype=np.int64)
        real._heading_inv_rot = None
        real.cfg = SimpleNamespace(
            control=SimpleNamespace(update_with_fk=False)
        )

        real.get_state()

        self.assertEqual(real._hitter_ball_pipeline.calls, 1)
        np.testing.assert_array_equal(real.ball_pos_world, first.position_w)
        np.testing.assert_array_equal(real.ball_vel_world, first.velocity_w)
        self.assertTrue(real.ball_visible)
        self.assertTrue(real.ball_state_estimator_ready)
        self.assertEqual(real.ball_state_estimator_sample_count, 31)

    def test_realworld_ignores_the_retired_g1pelvis_subject(self) -> None:
        real = RealWorld.__new__(RealWorld)
        real.cfg = SimpleNamespace(motion={"ball_planner": {}})
        real._init_ball_state()
        real.firstReceiveVicon = False
        real.root_trans_world_tmp = np.zeros(3, dtype=np.float32)
        real.root_quat_world_tmp = np.array(
            [0.0, 0.0, 0.0, 1.0],
            dtype=np.float32,
        )
        original_position = real.root_trans_world_tmp.copy()

        pelvis = _message("G1Pelvis", position=(9.0, 9.0, 9.0))
        real._vicon_state_handler("vicon_state_data", pelvis.encode())

        self.assertFalse(real.base_pose_valid_tmp)
        self.assertFalse(real.firstReceiveVicon)
        np.testing.assert_array_equal(
            real.root_trans_world_tmp,
            original_position,
        )

    def test_nonfinite_valid_ball_is_recorded_but_estimate_failure_is_contained(self) -> None:
        adapter = _adapter(min_samples=1)
        before = _ingest(
            adapter,
            _message("ball", frame=1),
            received_s=1.0,
        )
        output = _ingest(
            adapter,
            _message(
                "ball",
                position=(float("nan"), 0.0, 1.0),
                frame=2,
                source_time_s=2.0,
            ),
            received_s=2.0,
        )

        self.assertTrue(math.isnan(output.sample.position_w[0]))
        self.assertIsNone(output.snapshot)
        self.assertIn("ESTIMATE_NONFINITE", output.warnings)
        self.assertEqual(output.estimator_sample_count, 1)
        self.assertIsNone(output.reset_transition)
        self.assertIs(adapter.latest_ball_snapshot, before.snapshot)
        self.assertEqual(adapter.track_epoch, 0)

    def test_adapter_serializes_ingest_reset_and_pelvis_reads_with_one_rlock(self) -> None:
        adapter = _adapter(min_samples=1)
        self.assertIsInstance(adapter.lock, type(threading.RLock()))
        with adapter.lock:
            _ingest(
                adapter,
                _message("g1pelvis", frame=1),
                received_s=1.0,
            )
            adapter.copy_latest_pelvis()
            self.assertEqual(adapter.reset_estimator_after_strike(), 1)


if __name__ == "__main__":
    unittest.main()
