from __future__ import annotations

import threading
import unittest
from dataclasses import FrozenInstanceError
from types import SimpleNamespace

import numpy as np

from utils.hitter_ball_pipeline import (
    BasePoseW,
    BallPipelineUpdate,
    RealtimeViconBallPipeline,
)


def _message(
    name: str = "ball",
    *,
    position=(1.5, 0.0, 1.2),
    frame: int = 1,
    source_time_s: float = 1.0,
    publish_time_us: int = 1_000_000,
    valid: bool = True,
    occluded: bool = False,
) -> SimpleNamespace:
    return SimpleNamespace(
        name=name,
        pos_vicon=list(position),
        quat_vicon=[0.0, 0.0, 0.0, 1.0],
        vicon_frame_number=frame,
        vicon_time_s=source_time_s,
        publish_time_us=publish_time_us,
        valid=int(valid),
        occluded=int(occluded),
    )


class _BaseProvider:
    def __init__(self) -> None:
        self.calls = 0
        self.position = np.array([-0.4, -0.06, 0.61], dtype=np.float64)
        self.quaternion = np.array(
            [0.0, 0.0, 0.0, 1.0],
            dtype=np.float64,
        )
        self.valid = True

    def __call__(self) -> BasePoseW:
        self.calls += 1
        return BasePoseW(
            position_w=self.position,
            quaternion_xyzw=self.quaternion,
            valid=self.valid,
            simulation_time_s=4.0,
            captured_monotonic_s=5.0,
        )


class _BlockingProvider(_BaseProvider):
    def __init__(self, *, block_call: int) -> None:
        super().__init__()
        self.block_call = int(block_call)
        self.entered = threading.Event()
        self.release = threading.Event()
        self._calls_lock = threading.Lock()

    def __call__(self) -> BasePoseW:
        with self._calls_lock:
            self.calls += 1
            call = self.calls
        if call == self.block_call:
            self.entered.set()
            if not self.release.wait(timeout=2.0):
                raise RuntimeError("timed out waiting to release base provider")
        return BasePoseW(
            position_w=self.position,
            quaternion_xyzw=self.quaternion,
            valid=self.valid,
            simulation_time_s=4.0,
            captured_monotonic_s=5.0,
        )


def _pipeline(
    *,
    min_samples: int = 1,
    sample_rate_hz: float = 2.0,
    stale_timeout_s: float | None = 0.1,
    provider: _BaseProvider | None = None,
) -> tuple[RealtimeViconBallPipeline, _BaseProvider]:
    actual_provider = _BaseProvider() if provider is None else provider
    return (
        RealtimeViconBallPipeline(
            {
                "state_estimator_window_size": 31,
                "state_estimator_min_samples": min_samples,
                "state_estimator_sample_rate_hz": sample_rate_hz,
            },
            actual_provider,
            stale_timeout_s=stale_timeout_s,
        ),
        actual_provider,
    )


class RealtimeViconBallPipelineTests(unittest.TestCase):
    def test_only_normalized_ball_input_reaches_estimator_and_base_is_injected(self) -> None:
        pipeline, provider = _pipeline()

        for subject in ("G2Pelvis", "table", "unknown", "  "):
            result = pipeline.ingest_transformation(
                _message(subject, position=(9.0, 8.0, 7.0)),
                received_monotonic_s=1.0,
            )
            self.assertIsNone(result)

        self.assertEqual(provider.calls, 0)
        self.assertEqual(pipeline.state().sample_count, 0)

        snapshot = pipeline.ingest_transformation(
            _message("  BALL  ", position=(1.25, 0.2, 1.1)),
            received_monotonic_s=2.0,
        )

        self.assertEqual(provider.calls, 1)
        np.testing.assert_allclose(snapshot.position_w, [1.25, 0.2, 1.1])
        np.testing.assert_allclose(
            snapshot.base_position_w,
            [-0.4, -0.06, 0.61],
        )
        self.assertTrue(snapshot.base_valid)

    def test_timestamp_priority_fallback_and_nonmonotonic_rejection(self) -> None:
        pipeline, _provider = _pipeline(sample_rate_hz=2.0)

        accepted = [
            pipeline.ingest_transformation(
                _message(
                    frame=1,
                    source_time_s=10.0,
                    publish_time_us=90_000_000,
                ),
                received_monotonic_s=1.0,
            ),
            pipeline.ingest_transformation(
                _message(
                    frame=2,
                    source_time_s=0.0,
                    publish_time_us=11_000_000,
                ),
                received_monotonic_s=2.0,
            ),
            pipeline.ingest_transformation(
                _message(
                    frame=3,
                    source_time_s=float("nan"),
                    publish_time_us=0,
                ),
                received_monotonic_s=3.0,
            ),
        ]
        self.assertTrue(all(snapshot is not None for snapshot in accepted))
        np.testing.assert_allclose(
            [
                float(timestamp)
                for timestamp, _position in pipeline.ball_state_estimator._samples
            ],
            [10.0, 10.5, 11.0],
            rtol=0.0,
            atol=1.0e-12,
        )

        duplicate = pipeline.ingest_transformation_update(
            _message(frame=4, source_time_s=10.0),
            received_monotonic_s=4.0,
        )
        backward = pipeline.ingest_transformation_update(
            _message(frame=5, source_time_s=9.0),
            received_monotonic_s=5.0,
        )
        self.assertIsInstance(duplicate, BallPipelineUpdate)
        self.assertIsNone(duplicate.snapshot)
        self.assertIsNone(backward.snapshot)
        self.assertEqual(
            duplicate.rejected_reason,
            "NON_MONOTONIC_SOURCE_TIMESTAMP",
        )
        self.assertEqual(
            backward.rejected_reason,
            "NON_MONOTONIC_SOURCE_TIMESTAMP",
        )
        self.assertEqual(duplicate.sample_count, 3)
        self.assertEqual(duplicate.previous_track_epoch, 0)
        self.assertEqual(duplicate.new_track_epoch, 0)
        self.assertIsNone(duplicate.reset_reason)
        with self.assertRaises(FrozenInstanceError):
            duplicate.sample_count = 99
        state = pipeline.state()
        self.assertEqual(state.sample_count, 3)
        self.assertEqual(state.generation, 3)
        self.assertEqual(state.track_epoch, 0)

        invalid = pipeline.ingest_transformation(
            _message(frame=6, source_time_s=12.0, valid=False),
            received_monotonic_s=6.0,
        )
        restarted = pipeline.ingest_transformation(
            _message(frame=7, source_time_s=1.0),
            received_monotonic_s=7.0,
        )
        self.assertEqual(invalid.track_epoch, 1)
        self.assertEqual(restarted.track_epoch, 1)
        self.assertEqual(pipeline.state().sample_count, 1)
        self.assertEqual(
            float(pipeline.ball_state_estimator._samples[-1][0]),
            1.0,
        )

    def test_real_timestamp_domains_map_to_one_strictly_increasing_timeline(self) -> None:
        nominal_dt = 1.0 / 300.0
        pipeline, _provider = _pipeline(sample_rate_hz=300.0)
        snapshots = [
            pipeline.ingest_transformation(
                _message(
                    frame=1,
                    source_time_s=67488.226666667,
                    publish_time_us=1_784_095_971_657_880,
                ),
                received_monotonic_s=1.0,
            ),
            pipeline.ingest_transformation(
                _message(
                    frame=2,
                    source_time_s=0.0,
                    publish_time_us=1_784_095_971_661_210,
                ),
                received_monotonic_s=2.0,
            ),
            pipeline.ingest_transformation(
                _message(
                    frame=3,
                    source_time_s=67488.230000000,
                    publish_time_us=1_784_095_971_664_540,
                ),
                received_monotonic_s=3.0,
            ),
            pipeline.ingest_transformation(
                _message(
                    frame=4,
                    source_time_s=67488.233333333,
                    publish_time_us=1_784_095_971_667_870,
                ),
                received_monotonic_s=4.0,
            ),
        ]
        estimator_times = np.array(
            [
                float(timestamp)
                for timestamp, _position in pipeline.ball_state_estimator._samples
            ]
        )

        self.assertAlmostEqual(snapshots[0].source_time_s, 67488.226666667)
        self.assertEqual(snapshots[1].source_time_s, 0.0)
        self.assertAlmostEqual(snapshots[2].source_time_s, 67488.230000000)
        self.assertTrue(np.all(np.diff(estimator_times) > 0.0))
        self.assertTrue(np.all(np.diff(estimator_times) < 0.01))
        self.assertAlmostEqual(
            estimator_times[1] - estimator_times[0],
            nominal_dt,
            places=9,
        )
        self.assertAlmostEqual(
            estimator_times[2] - estimator_times[1],
            nominal_dt,
            places=9,
        )
        self.assertAlmostEqual(
            estimator_times[3] - estimator_times[2],
            0.003333333,
            places=9,
        )

    def test_publish_first_then_vicon_and_publish_only_use_bounded_deltas(self) -> None:
        pipeline, _provider = _pipeline(sample_rate_hz=300.0)
        pipeline.ingest_transformation(
            _message(
                frame=1,
                source_time_s=0.0,
                publish_time_us=1_784_095_971_657_880,
            ),
            received_monotonic_s=1.0,
        )
        pipeline.ingest_transformation(
            _message(
                frame=2,
                source_time_s=67488.226666667,
                publish_time_us=1_784_095_971_661_210,
            ),
            received_monotonic_s=2.0,
        )
        first_pair = [
            float(timestamp)
            for timestamp, _position in pipeline.ball_state_estimator._samples
        ]
        self.assertAlmostEqual(
            first_pair[1] - first_pair[0],
            1.0 / 300.0,
            places=7,
        )

        publish_only, _provider = _pipeline(sample_rate_hz=300.0)
        for frame, publish_time_us in enumerate(
            (1_784_095_971_657_880, 1_784_095_971_660_950),
            start=1,
        ):
            publish_only.ingest_transformation(
                _message(
                    frame=frame,
                    source_time_s=0.0,
                    publish_time_us=publish_time_us,
                ),
                received_monotonic_s=float(frame),
            )
        publish_times = [
            float(timestamp)
            for timestamp, _position in publish_only.ball_state_estimator._samples
        ]
        expected_publish_delta = (
            float(1_784_095_971_660_950) * 1.0e-6
            - float(1_784_095_971_657_880) * 1.0e-6
        )
        self.assertAlmostEqual(
            publish_times[1] - publish_times[0],
            expected_publish_delta,
            places=12,
        )

    def test_fallback_before_and_after_real_sources_uses_nominal_steps(self) -> None:
        pipeline, _provider = _pipeline(sample_rate_hz=100.0)
        messages = (
            _message(frame=1, source_time_s=0.0, publish_time_us=0),
            _message(frame=2, source_time_s=67488.0),
            _message(frame=3, source_time_s=0.0, publish_time_us=0),
            _message(
                frame=4,
                source_time_s=0.0,
                publish_time_us=1_784_095_971_657_880,
            ),
            _message(frame=5, source_time_s=0.0, publish_time_us=0),
        )
        for index, message in enumerate(messages, start=1):
            pipeline.ingest_transformation(
                message,
                received_monotonic_s=float(index),
            )
        estimator_times = [
            float(timestamp)
            for timestamp, _position in pipeline.ball_state_estimator._samples
        ]

        np.testing.assert_allclose(
            estimator_times,
            [0.0, 0.01, 0.02, 0.03, 0.04],
            rtol=0.0,
            atol=1.0e-9,
        )
        self.assertAlmostEqual(pipeline.fallback_estimator_time_s, 0.02)

    def test_duplicate_and_backward_are_rejected_per_real_clock_domain(self) -> None:
        pipeline, _provider = _pipeline(sample_rate_hz=300.0)
        accepted_vicon = pipeline.ingest_transformation_update(
            _message(
                frame=1,
                source_time_s=67488.226,
                publish_time_us=1_784_095_971_657_880,
            ),
            received_monotonic_s=1.0,
        )
        accepted_publish = pipeline.ingest_transformation_update(
            _message(
                frame=2,
                source_time_s=0.0,
                publish_time_us=1_784_095_971_661_210,
            ),
            received_monotonic_s=2.0,
        )
        duplicate_vicon = pipeline.ingest_transformation_update(
            _message(frame=3, source_time_s=67488.226),
            received_monotonic_s=3.0,
        )
        backward_vicon = pipeline.ingest_transformation_update(
            _message(frame=4, source_time_s=67488.225),
            received_monotonic_s=4.0,
        )
        duplicate_publish = pipeline.ingest_transformation_update(
            _message(
                frame=5,
                source_time_s=0.0,
                publish_time_us=1_784_095_971_661_210,
            ),
            received_monotonic_s=5.0,
        )
        backward_publish = pipeline.ingest_transformation_update(
            _message(
                frame=6,
                source_time_s=0.0,
                publish_time_us=1_784_095_971_650_000,
            ),
            received_monotonic_s=6.0,
        )

        self.assertIsNotNone(accepted_vicon.snapshot)
        self.assertIsNotNone(accepted_publish.snapshot)
        for rejected in (
            duplicate_vicon,
            backward_vicon,
            duplicate_publish,
            backward_publish,
        ):
            self.assertEqual(
                rejected.rejected_reason,
                "NON_MONOTONIC_SOURCE_TIMESTAMP",
            )
        self.assertEqual(pipeline.state().sample_count, 2)
        self.assertEqual(pipeline.state().generation, 2)

    def test_thirty_first_sample_is_ready(self) -> None:
        pipeline, _provider = _pipeline(
            min_samples=31,
            sample_rate_hz=360.0,
        )
        snapshots = []
        for index in range(31):
            snapshots.append(
                pipeline.ingest_transformation(
                    _message(
                        position=(1.8 - 0.01 * index, 0.0, 1.2),
                        frame=index + 1,
                        source_time_s=1.0 + index / 360.0,
                    ),
                    received_monotonic_s=2.0 + index / 360.0,
                )
            )

        self.assertFalse(snapshots[-2].ready)
        self.assertTrue(snapshots[-1].ready)
        self.assertEqual(pipeline.state().sample_count, 31)

    def test_invalid_and_occluded_messages_advance_epoch_once_per_track(self) -> None:
        pipeline, _provider = _pipeline()
        visible = pipeline.ingest_transformation(
            _message(frame=1, source_time_s=1.0),
            received_monotonic_s=1.0,
        )
        invalid = pipeline.ingest_transformation_update(
            _message(frame=2, source_time_s=2.0, valid=False),
            received_monotonic_s=2.0,
        )
        occluded = pipeline.ingest_transformation_update(
            _message(frame=3, source_time_s=3.0, occluded=True),
            received_monotonic_s=3.0,
        )
        restarted = pipeline.ingest_transformation(
            _message(frame=4, source_time_s=0.5),
            received_monotonic_s=4.0,
        )

        self.assertEqual(
            [
                visible.track_epoch,
                invalid.snapshot.track_epoch,
                restarted.track_epoch,
            ],
            [0, 1, 1],
        )
        self.assertEqual(
            [
                visible.generation,
                invalid.snapshot.generation,
                restarted.generation,
            ],
            [1, 2, 3],
        )
        self.assertFalse(invalid.snapshot.visible)
        self.assertIsNone(occluded.snapshot)
        self.assertEqual(invalid.previous_track_epoch, 0)
        self.assertEqual(invalid.new_track_epoch, 1)
        self.assertEqual(invalid.reset_reason, "BALL_INVALID_OR_OCCLUDED")
        self.assertFalse(invalid.bounce_detected)
        self.assertEqual(occluded.previous_track_epoch, 1)
        self.assertEqual(occluded.new_track_epoch, 1)
        self.assertIsNone(occluded.reset_reason)
        self.assertIsNone(occluded.rejected_reason)
        self.assertTrue(restarted.visible)
        self.assertEqual(pipeline.state().sample_count, 1)

    def test_stale_transition_is_exactly_once_and_new_track_keeps_generation(self) -> None:
        pipeline, _provider = _pipeline(stale_timeout_s=0.1)
        first = pipeline.ingest_transformation(
            _message(frame=1, source_time_s=5.0),
            received_monotonic_s=10.0,
        )

        self.assertIsNone(pipeline.expire_stale(now=10.1))
        stale = pipeline.expire_stale(now=10.100001)
        self.assertIsNotNone(stale)
        self.assertFalse(stale.visible)
        self.assertFalse(stale.ready)
        self.assertEqual(stale.track_epoch, 1)
        self.assertEqual(stale.generation, 2)
        self.assertIsNone(pipeline.expire_stale(now=20.0))
        self.assertEqual(pipeline.state().generation, 2)

        restarted = pipeline.ingest_transformation(
            _message(frame=2, source_time_s=1.0),
            received_monotonic_s=20.1,
        )
        self.assertEqual(restarted.track_epoch, 1)
        self.assertEqual(restarted.generation, 3)
        self.assertEqual(pipeline.reset_after_strike(), 2)
        self.assertEqual(pipeline.state().generation, 3)
        after_reset = pipeline.ingest_transformation(
            _message(frame=3, source_time_s=0.5),
            received_monotonic_s=21.0,
        )
        self.assertEqual(after_reset.track_epoch, 2)
        self.assertEqual(after_reset.generation, 4)
        self.assertEqual(first.track_epoch, 0)

    def test_update_reports_exact_bounce_transition_without_epoch_reset(self) -> None:
        pipeline, _provider = _pipeline(min_samples=3)
        updates = []
        for index, position in enumerate(
            (
                (1.4, 0.0, 0.95),
                (1.4, 0.0, 0.90),
                (1.4, 0.0, 0.79),
            ),
            start=1,
        ):
            updates.append(
                pipeline.ingest_transformation_update(
                    _message(
                        position=position,
                        frame=index,
                        source_time_s=float(index),
                    ),
                    received_monotonic_s=float(index),
                )
            )

        self.assertFalse(updates[-2].bounce_detected)
        self.assertTrue(updates[-1].bounce_detected)
        self.assertEqual(updates[-1].sample_count, 1)
        self.assertEqual(updates[-1].previous_track_epoch, 0)
        self.assertEqual(updates[-1].new_track_epoch, 0)
        self.assertIsNone(updates[-1].reset_reason)
        self.assertIsNone(updates[-1].rejected_reason)
        self.assertEqual(updates[-1].snapshot.track_epoch, 0)

    def test_arrays_are_detached_read_only_and_inputs_are_validated(self) -> None:
        provider = _BaseProvider()
        base = provider()
        provider.position[0] = 99.0
        provider.quaternion[3] = 0.0
        np.testing.assert_allclose(base.position_w, [-0.4, -0.06, 0.61])
        np.testing.assert_allclose(base.quaternion_xyzw, [0.0, 0.0, 0.0, 1.0])
        for array in (base.position_w, base.quaternion_xyzw):
            self.assertFalse(array.flags.writeable)
            with self.assertRaises(ValueError):
                array.setflags(write=True)
        with self.assertRaises(FrozenInstanceError):
            base.valid = False

        provider = _BaseProvider()
        pipeline, _provider = _pipeline(provider=provider)
        message = _message(position=(1.2, 0.1, 1.0))
        snapshot = pipeline.ingest_transformation(
            message,
            received_monotonic_s=1.0,
        )
        message.pos_vicon[0] = 8.0
        provider.position[0] = 7.0
        state = pipeline.state()
        for array in (
            snapshot.position_w,
            snapshot.velocity_w,
            snapshot.base_position_w,
            snapshot.base_quaternion_xyzw,
            state.position_w,
            state.velocity_w,
            pipeline._position_w,
            pipeline._velocity_w,
        ):
            self.assertFalse(array.flags.writeable)
            with self.assertRaises(ValueError):
                array.setflags(write=True)
        np.testing.assert_allclose(snapshot.position_w, [1.2, 0.1, 1.0])
        np.testing.assert_allclose(snapshot.base_position_w, [-0.4, -0.06, 0.61])

        invalid_base_arguments = (
            {"position_w": [0.0, 0.0]},
            {"position_w": [0.0, float("nan"), 0.0]},
            {"quaternion_xyzw": [0.0, 0.0, 0.0, 2.0]},
            {"quaternion_xyzw": [0.0, 0.0, 0.0, 0.0]},
        )
        for overrides in invalid_base_arguments:
            arguments = {
                "position_w": [0.0, 0.0, 0.0],
                "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                "valid": True,
                "simulation_time_s": 0.0,
                "captured_monotonic_s": 0.0,
            }
            arguments.update(overrides)
            with self.subTest(overrides=overrides):
                with self.assertRaises(ValueError):
                    BasePoseW(**arguments)

        with self.assertRaises(ValueError):
            pipeline.ingest_transformation(
                _message(position=(1.0, 2.0)),
                received_monotonic_s=2.0,
            )
        with self.assertRaises(ValueError):
            pipeline.ingest_transformation(
                _message(position=(1.0, float("inf"), 2.0)),
                received_monotonic_s=2.0,
            )
        for timeout in (0.0, -1.0, float("inf"), float("nan")):
            with self.subTest(timeout=timeout):
                with self.assertRaises(ValueError):
                    RealtimeViconBallPipeline(
                        {},
                        _BaseProvider(),
                        stale_timeout_s=timeout,
                    )

    def test_later_invalid_tombstone_supersedes_slow_valid_update(self) -> None:
        provider = _BlockingProvider(block_call=1)
        pipeline, _provider = _pipeline(provider=provider)
        observed = []
        pipeline.register_listener(observed.append)
        result = {}
        errors = []

        def ingest_slow_valid() -> None:
            try:
                result["valid"] = pipeline.ingest_transformation_update(
                    _message(frame=1, source_time_s=67488.0),
                    received_monotonic_s=1.0,
                )
            except Exception as exc:  # pragma: no cover - diagnostic guard
                errors.append(exc)

        thread = threading.Thread(target=ingest_slow_valid)
        thread.start()
        self.assertTrue(provider.entered.wait(timeout=1.0))
        invalid = pipeline.ingest_transformation_update(
            _message(frame=2, source_time_s=67488.01, valid=False),
            received_monotonic_s=2.0,
        )
        provider.release.set()
        thread.join(timeout=1.0)

        self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])
        self.assertIsNone(invalid.snapshot)
        self.assertIsNone(invalid.reset_reason)
        self.assertEqual(result["valid"].rejected_reason, "SUPERSEDED_OPERATION")
        self.assertIsNone(result["valid"].snapshot)
        self.assertEqual(observed, [])
        state = pipeline.state()
        self.assertFalse(state.visible)
        self.assertEqual(state.sample_count, 0)
        self.assertEqual(state.track_epoch, 0)
        self.assertEqual(state.generation, 0)
        self.assertIsNone(state.latest_snapshot)

    def test_stale_transition_supersedes_slow_valid_update(self) -> None:
        provider = _BlockingProvider(block_call=2)
        pipeline, _provider = _pipeline(
            provider=provider,
            stale_timeout_s=0.1,
        )
        observed = []
        pipeline.register_listener(
            lambda snapshot: observed.append(
                (
                    snapshot.generation,
                    snapshot.track_epoch,
                    snapshot.visible,
                    snapshot.source_frame,
                )
            )
        )
        pipeline.ingest_transformation(
            _message(frame=1, source_time_s=67488.0),
            received_monotonic_s=1.0,
        )
        result = {}
        errors = []

        def ingest_slow_valid() -> None:
            try:
                result["valid"] = pipeline.ingest_transformation_update(
                    _message(frame=2, source_time_s=67488.01),
                    received_monotonic_s=1.05,
                )
            except Exception as exc:  # pragma: no cover - diagnostic guard
                errors.append(exc)

        thread = threading.Thread(target=ingest_slow_valid)
        thread.start()
        self.assertTrue(provider.entered.wait(timeout=1.0))
        stale = pipeline.expire_stale(now=1.100001)
        provider.release.set()
        thread.join(timeout=1.0)

        self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])
        self.assertIsNotNone(stale)
        self.assertEqual(result["valid"].rejected_reason, "SUPERSEDED_OPERATION")
        self.assertEqual(
            observed,
            [
                (1, 0, True, 1),
                (2, 1, False, 1),
            ],
        )
        state = pipeline.state()
        self.assertFalse(state.visible)
        self.assertEqual(state.sample_count, 0)
        self.assertEqual(state.track_epoch, 1)
        self.assertEqual(state.generation, 2)
        self.assertIs(state.latest_snapshot, stale)

    def test_strike_reset_supersedes_slow_valid_then_new_valid_is_accepted(self) -> None:
        provider = _BlockingProvider(block_call=1)
        pipeline, _provider = _pipeline(provider=provider)
        observed = []
        pipeline.register_listener(
            lambda snapshot: observed.append(
                (
                    snapshot.generation,
                    snapshot.track_epoch,
                    snapshot.source_frame,
                )
            )
        )
        result = {}
        errors = []

        def ingest_slow_valid() -> None:
            try:
                result["valid"] = pipeline.ingest_transformation_update(
                    _message(frame=1, source_time_s=67488.0),
                    received_monotonic_s=1.0,
                )
            except Exception as exc:  # pragma: no cover - diagnostic guard
                errors.append(exc)

        thread = threading.Thread(target=ingest_slow_valid)
        thread.start()
        self.assertTrue(provider.entered.wait(timeout=1.0))
        self.assertEqual(pipeline.reset_after_strike(), 1)
        provider.release.set()
        thread.join(timeout=1.0)
        accepted = pipeline.ingest_transformation_update(
            _message(frame=2, source_time_s=1.0),
            received_monotonic_s=2.0,
        )

        self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(result["valid"].rejected_reason, "SUPERSEDED_OPERATION")
        self.assertIsNotNone(accepted.snapshot)
        self.assertEqual(accepted.new_track_epoch, 1)
        self.assertEqual(observed, [(1, 1, 2)])
        state = pipeline.state()
        self.assertTrue(state.visible)
        self.assertEqual(state.sample_count, 1)
        self.assertEqual(state.track_epoch, 1)
        self.assertEqual(state.generation, 1)
        self.assertIs(state.latest_snapshot, accepted.snapshot)

    def test_later_concurrent_invalid_is_the_only_committed_transition(self) -> None:
        provider = _BlockingProvider(block_call=2)
        pipeline, _provider = _pipeline(provider=provider)
        observed = []
        pipeline.register_listener(
            lambda snapshot: observed.append(
                (
                    snapshot.generation,
                    snapshot.track_epoch,
                    snapshot.visible,
                    snapshot.source_frame,
                )
            )
        )
        pipeline.ingest_transformation(
            _message(frame=1, source_time_s=1.0),
            received_monotonic_s=1.0,
        )
        result = {}
        errors = []

        def ingest_slow_invalid() -> None:
            try:
                result["invalid"] = pipeline.ingest_transformation_update(
                    _message(frame=2, source_time_s=2.0, valid=False),
                    received_monotonic_s=2.0,
                )
            except Exception as exc:  # pragma: no cover - diagnostic guard
                errors.append(exc)

        thread = threading.Thread(target=ingest_slow_invalid)
        thread.start()
        self.assertTrue(provider.entered.wait(timeout=1.0))
        winning = pipeline.ingest_transformation_update(
            _message(frame=3, source_time_s=3.0, occluded=True),
            received_monotonic_s=3.0,
        )
        provider.release.set()
        thread.join(timeout=1.0)

        self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(
            result["invalid"].rejected_reason,
            "SUPERSEDED_OPERATION",
        )
        self.assertEqual(winning.reset_reason, "BALL_INVALID_OR_OCCLUDED")
        self.assertEqual(
            observed,
            [
                (1, 0, True, 1),
                (2, 1, False, 3),
            ],
        )
        state = pipeline.state()
        self.assertFalse(state.visible)
        self.assertEqual(state.sample_count, 0)
        self.assertEqual(state.track_epoch, 1)
        self.assertEqual(state.generation, 2)
        self.assertIs(state.latest_snapshot, winning.snapshot)

    def test_close_supersedes_provider_blocked_work(self) -> None:
        provider = _BlockingProvider(block_call=1)
        pipeline, _provider = _pipeline(provider=provider)
        observed = []
        pipeline.register_listener(observed.append)
        result = {}
        errors = []

        def ingest_slow_valid() -> None:
            try:
                result["valid"] = pipeline.ingest_transformation_update(
                    _message(frame=1, source_time_s=1.0),
                    received_monotonic_s=1.0,
                )
            except Exception as exc:  # pragma: no cover - diagnostic guard
                errors.append(exc)

        thread = threading.Thread(target=ingest_slow_valid)
        thread.start()
        self.assertTrue(provider.entered.wait(timeout=1.0))
        pipeline.close()
        provider.release.set()
        thread.join(timeout=1.0)
        after_close = pipeline.ingest_transformation_update(
            _message(frame=2, source_time_s=2.0),
            received_monotonic_s=2.0,
        )

        self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(result["valid"].rejected_reason, "SUPERSEDED_OPERATION")
        self.assertEqual(after_close.rejected_reason, "PIPELINE_CLOSED")
        self.assertEqual(provider.calls, 1)
        self.assertEqual(observed, [])
        self.assertEqual(pipeline.state().generation, 0)
        self.assertEqual(pipeline.state().sample_count, 0)

    def test_provider_and_listeners_run_outside_lock_with_reentrant_callbacks(self) -> None:
        provider_probe = {"pipeline": None, "completed": []}

        def provider() -> BasePoseW:
            completed = threading.Event()

            def read_state() -> None:
                provider_probe["pipeline"].state()
                completed.set()

            thread = threading.Thread(target=read_state)
            thread.start()
            thread.join(timeout=0.5)
            provider_probe["completed"].append(completed.is_set())
            return BasePoseW(
                position_w=[-0.4, -0.06, 0.61],
                quaternion_xyzw=[0.0, 0.0, 0.0, 1.0],
                valid=True,
                simulation_time_s=0.0,
                captured_monotonic_s=0.0,
            )

        pipeline = RealtimeViconBallPipeline(
            {
                "state_estimator_window_size": 31,
                "state_estimator_min_samples": 1,
            },
            provider,
            stale_timeout_s=None,
        )
        provider_probe["pipeline"] = pipeline
        observed = []
        self_unregistered = []

        def broken_listener(_snapshot) -> None:
            raise RuntimeError("listener failure must be isolated")

        def lock_probe(snapshot) -> None:
            completed = threading.Event()

            def read_state() -> None:
                pipeline.state()
                completed.set()

            thread = threading.Thread(target=read_state)
            thread.start()
            thread.join(timeout=0.5)
            observed.append((snapshot.generation, completed.is_set()))

        unregister_holder = {}

        def self_unregister_and_reset(snapshot) -> None:
            unregister_holder["unregister"]()
            self_unregistered.append(
                (snapshot.generation, pipeline.reset_after_strike())
            )

        pipeline.register_listener(broken_listener)
        unregister_holder["unregister"] = pipeline.register_listener(
            self_unregister_and_reset
        )
        unregister = pipeline.register_listener(lock_probe)
        first = pipeline.ingest_transformation(
            _message(frame=1, source_time_s=1.0),
            received_monotonic_s=1.0,
        )
        self.assertEqual(provider_probe["completed"], [True])
        self.assertEqual(self_unregistered, [(first.generation, 1)])
        self.assertEqual(observed, [(first.generation, True)])

        unregister()
        unregister()
        pipeline.ingest_transformation(
            _message(frame=2, source_time_s=0.5),
            received_monotonic_s=2.0,
        )
        self.assertEqual(len(self_unregistered), 1)
        self.assertEqual(observed, [(first.generation, True)])

        post_close_calls = []
        pipeline.register_listener(post_close_calls.append)
        pipeline.close()
        pipeline.close()
        pipeline.ingest_transformation(
            _message(frame=3, source_time_s=3.0),
            received_monotonic_s=3.0,
        )
        self.assertEqual(post_close_calls, [])


if __name__ == "__main__":
    unittest.main()
