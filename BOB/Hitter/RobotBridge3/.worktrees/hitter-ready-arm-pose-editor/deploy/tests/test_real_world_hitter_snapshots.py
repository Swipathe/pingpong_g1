from __future__ import annotations

import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
from hydra import compose, initialize_config_dir

from simulator import real_world as real_world_module
from simulator.real_world import RealWorld
from utils.hitter_realtime import BallEstimateSnapshot


DEPLOY_DIR = Path(__file__).resolve().parents[1]
CONFIG_DIR = DEPLOY_DIR / "config"


class ObservedRLock:
    def __init__(self, observed_thread_name: str):
        self._lock = threading.RLock()
        self.observed_thread_name = observed_thread_name
        self.acquire_attempted = threading.Event()

    def __enter__(self):
        if threading.current_thread().name == self.observed_thread_name:
            self.acquire_attempted.set()
        self._lock.acquire()
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback):
        self._lock.release()


def ball_message(
    *,
    frame: int = 0,
    source_time: float = 0.0,
    valid: int = 1,
    occluded: int = 0,
    position=(1.0, 0.0, 0.90),
):
    return SimpleNamespace(
        name="ball",
        vicon_frame_number=int(frame),
        vicon_time_s=float(source_time),
        publish_time_us=0,
        valid=int(valid),
        occluded=int(occluded),
        pos_vicon=np.asarray(position, dtype=np.float64),
        quat_vicon=np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32),
    )


def base_message(
    *,
    name: str = "G1Pelvis",
    valid: int = 1,
    occluded: int = 0,
    position=(0.2, -0.1, 0.8),
):
    return SimpleNamespace(
        name=name,
        vicon_frame_number=1,
        vicon_time_s=1.0,
        publish_time_us=0,
        valid=int(valid),
        occluded=int(occluded),
        pos_vicon=np.asarray(position, dtype=np.float32),
        quat_vicon=np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32),
    )


def make_real_world_without_io() -> RealWorld:
    sim = RealWorld.__new__(RealWorld)
    sim.cfg = SimpleNamespace(motion={})
    sim.root_trans_world_tmp = np.zeros(3, dtype=np.float32)
    sim.root_quat_world_tmp = np.array(
        [0.0, 0.0, 0.0, 1.0], dtype=np.float32
    )
    sim.firstReceiveVicon = False
    sim._init_ball_state()
    return sim


def deliver_vicon(sim: RealWorld, message) -> None:
    with patch.object(
        real_world_module.transformation_t,
        "decode",
        return_value=message,
    ):
        sim._vicon_state_handler("vicon_state_data", b"")


def feed_positions(
    sim: RealWorld,
    positions,
    *,
    start_frame: int = 0,
    start_time: float = 0.0,
    dt: float = 0.01,
) -> None:
    for offset, position in enumerate(positions):
        frame = start_frame + offset
        source_time = start_time + offset * dt
        sim._update_ball_state_from_vicon(
            ball_message(
                frame=frame,
                source_time=source_time,
                position=position,
            ),
            np.asarray(position, dtype=np.float64),
        )


class RealWorldHitterConfigTests(unittest.TestCase):
    def test_default_hitter_composition_shares_planner_motion_with_real_world(self):
        with initialize_config_dir(
            version_base=None,
            config_dir=str(CONFIG_DIR.resolve()),
        ):
            config = compose(config_name="hitter")

        self.assertIn("motion", config.sim.config)
        sim = RealWorld.__new__(RealWorld)
        sim.cfg = config.sim.config
        sim.root_trans_world_tmp = np.zeros(3, dtype=np.float32)
        sim.root_quat_world_tmp = np.array(
            [0.0, 0.0, 0.0, 1.0], dtype=np.float32
        )
        sim._init_ball_state()

        self.assertEqual(sim.ball_state_estimator_sample_rate_hz, 360.0)
        np.testing.assert_allclose(
            sim.ball_state_estimator.table_center_xy,
            [1.365369, 0.0],
        )
        self.assertEqual(sim.ball_state_estimator.table_length, 2.730738)
        self.assertEqual(sim.ball_state_estimator.table_width, 1.512451)
        self.assertEqual(sim.ball_state_estimator.table_height, 0.76)


class RealWorldHitterSnapshotTests(unittest.TestCase):
    def test_reset_serializes_with_inflight_ball_callback(self):
        sim = make_real_world_without_io()
        observed_lock = ObservedRLock("reset-thread")
        sim._hitter_ball_state_lock = observed_lock
        sim.base_pose_valid_tmp = True
        sim.root_trans_world_tmp[:] = [0.2, -0.1, 0.8]
        sim.root_quat_world_tmp[:] = [0.0, 0.0, 0.1, 0.995]
        received = []
        sim.register_hitter_ball_listener(received.append)

        callback_paused = threading.Event()
        release_callback = threading.Event()
        reset_finished = threading.Event()
        callback_errors = []
        original_add_sample = sim.ball_state_estimator.add_sample

        def blocking_add_sample(position, *, timestamp=None):
            estimate = original_add_sample(position, timestamp=timestamp)
            callback_paused.set()
            if not release_callback.wait(timeout=2.0):
                raise TimeoutError("test did not release ball callback")
            return estimate

        sim.ball_state_estimator.add_sample = blocking_add_sample

        def run_callback():
            try:
                sim._update_ball_state_from_vicon(
                    ball_message(frame=8, source_time=2.0),
                    np.array([0.9, 0.0, 0.9]),
                )
            except BaseException as error:
                callback_errors.append(error)

        def run_reset():
            sim.reset_ball_state_estimator()
            reset_finished.set()

        callback_thread = threading.Thread(target=run_callback)
        reset_thread = threading.Thread(target=run_reset, name="reset-thread")
        callback_thread.start()
        self.assertTrue(callback_paused.wait(timeout=1.0))
        reset_thread.start()
        self.assertTrue(observed_lock.acquire_attempted.wait(timeout=1.0))

        reset_finished_before_release = reset_finished.wait(timeout=0.1)
        release_callback.set()
        callback_thread.join(timeout=1.0)
        reset_thread.join(timeout=1.0)

        self.assertFalse(callback_thread.is_alive())
        self.assertFalse(reset_thread.is_alive())
        self.assertEqual(callback_errors, [])
        self.assertFalse(reset_finished_before_release)
        self.assertEqual(len(received), 1)
        self.assertEqual(received[0].track_epoch, 0)
        self.assertTrue(received[0].visible)
        np.testing.assert_allclose(received[0].base_position_w, [0.2, -0.1, 0.8])
        np.testing.assert_allclose(
            received[0].base_quaternion_xyzw,
            [0.0, 0.0, 0.1, 0.995],
        )
        self.assertEqual(sim.ball_track_epoch, 1)
        self.assertFalse(sim.ball_visible_tmp)
        self.assertFalse(sim.ball_state_estimator_ready_tmp)
        self.assertEqual(sim.ball_state_estimator.sample_count, 0)

    def test_each_valid_message_publishes_a_new_snapshot_generation(self):
        sim = make_real_world_without_io()
        received = []
        sim.register_hitter_ball_listener(received.append)

        for frame in range(31):
            position = np.array([1.0 - 0.002 * frame, 0.0, 0.90])
            sim._update_ball_state_from_vicon(
                ball_message(
                    frame=frame,
                    source_time=frame / 360.0,
                    position=position,
                ),
                position,
            )

        self.assertEqual(len(received), 31)
        self.assertIsInstance(received[-1], BallEstimateSnapshot)
        self.assertEqual(received[-1].generation, 31)
        self.assertEqual(received[-1].source_frame, 30)
        self.assertEqual(received[-1].source_time_s, 30 / 360.0)
        self.assertTrue(received[-1].visible)
        self.assertTrue(received[-1].ready)
        self.assertIs(sim.latest_ball_snapshot, received[-1])

    def test_snapshot_uses_receive_monotonic_and_copies_base_state(self):
        sim = make_real_world_without_io()
        sim.base_pose_valid_tmp = True
        sim.root_trans_world_tmp[:] = [0.2, -0.1, 0.8]
        sim.root_quat_world_tmp[:] = [0.0, 0.0, 0.1, 0.995]
        received = []
        sim.register_hitter_ball_listener(received.append)

        with patch.object(real_world_module.time, "monotonic", return_value=42.5):
            sim._update_ball_state_from_vicon(
                ball_message(frame=7, source_time=12.25),
                np.array([0.9, 0.0, 0.9]),
            )

        item = received[-1]
        sim.root_trans_world_tmp[:] = 99.0
        sim.root_quat_world_tmp[:] = 99.0
        self.assertEqual(item.received_monotonic_s, 42.5)
        self.assertTrue(item.base_valid)
        np.testing.assert_allclose(item.base_position_w, [0.2, -0.1, 0.8])
        np.testing.assert_allclose(
            item.base_quaternion_xyzw,
            [0.0, 0.0, 0.1, 0.995],
        )
        self.assertFalse(item.position_w.flags.writeable)
        self.assertFalse(item.base_position_w.flags.writeable)

    def test_get_state_copies_world_base_and_ball_fields_atomically(self):
        class BlockingCopyArray:
            def __init__(self, value, copy_paused, release_copy):
                self.value = np.asarray(value, dtype=np.float32)
                self.copy_paused = copy_paused
                self.release_copy = release_copy

            def copy(self):
                copied = self.value.copy()
                self.copy_paused.set()
                if not self.release_copy.wait(timeout=2.0):
                    raise TimeoutError("test did not release get_state copy")
                return copied

        sim = make_real_world_without_io()
        observed_lock = ObservedRLock("base-writer")
        sim._hitter_ball_state_lock = observed_lock
        copy_paused = threading.Event()
        release_copy = threading.Event()
        writer_finished = threading.Event()
        reader_errors = []
        writer_errors = []
        sim.root_trans_tmp = np.zeros(3, dtype=np.float32)
        sim.base_lin_vel_tmp = np.zeros(3, dtype=np.float32)
        sim.root_quat_tmp = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        sim.root_rpy_tmp = np.zeros(3, dtype=np.float32)
        sim.base_ang_vel_tmp = np.zeros(3, dtype=np.float32)
        sim._heading_inv_rot = None
        sim.root_trans_world_tmp = BlockingCopyArray(
            [0.2, -0.1, 0.8],
            copy_paused,
            release_copy,
        )
        sim.root_quat_world_tmp = np.array(
            [0.0, 0.0, 0.0, 1.0], dtype=np.float32
        )
        sim.base_pose_valid_tmp = True
        sim.dof_pos_tmp = np.zeros(1, dtype=np.float32)
        sim.dof_vel_tmp = np.zeros(1, dtype=np.float32)
        sim.active_dof_idx = np.array([0], dtype=np.int64)
        sim.cfg.control = SimpleNamespace(update_with_fk=False)

        def run_reader():
            try:
                sim.get_state()
            except BaseException as error:
                reader_errors.append(error)

        def run_writer():
            try:
                message = base_message(position=[0.4, 0.3, 0.82])
                message.quat_vicon = np.array(
                    [0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)],
                    dtype=np.float32,
                )
                deliver_vicon(sim, message)
            except BaseException as error:
                writer_errors.append(error)
            finally:
                writer_finished.set()

        reader_thread = threading.Thread(target=run_reader)
        writer_thread = threading.Thread(target=run_writer, name="base-writer")
        reader_thread.start()
        self.assertTrue(copy_paused.wait(timeout=1.0))
        writer_thread.start()
        self.assertTrue(observed_lock.acquire_attempted.wait(timeout=1.0))

        writer_finished_before_release = writer_finished.wait(timeout=0.1)
        release_copy.set()
        reader_thread.join(timeout=1.0)
        writer_thread.join(timeout=1.0)

        self.assertFalse(reader_thread.is_alive())
        self.assertFalse(writer_thread.is_alive())
        self.assertEqual(reader_errors, [])
        self.assertEqual(writer_errors, [])
        self.assertFalse(writer_finished_before_release)
        np.testing.assert_allclose(sim.root_trans_world, [0.2, -0.1, 0.8])
        np.testing.assert_allclose(
            sim.root_quat_world,
            [0.0, 0.0, 0.0, 1.0],
        )
        self.assertTrue(sim.base_pose_valid)

    def test_listener_unregisters_idempotently_and_callbacks_run_outside_lock(self):
        sim = make_real_world_without_io()
        received = []
        callback_lock_state = []

        def listener(item):
            acquired = sim._hitter_ball_listener_lock.acquire(blocking=False)
            callback_lock_state.append(acquired)
            if acquired:
                sim._hitter_ball_listener_lock.release()
            received.append(item)

        unregister = sim.register_hitter_ball_listener(listener)
        feed_positions(sim, [[1.0, 0.0, 0.9]])
        unregister()
        unregister()
        feed_positions(
            sim,
            [[0.99, 0.0, 0.9]],
            start_frame=1,
            start_time=0.01,
        )

        self.assertEqual(callback_lock_state, [True])
        self.assertEqual(len(received), 1)

    def test_each_duplicate_listener_registration_unregisters_independently(self):
        sim = make_real_world_without_io()
        received = []
        listener = received.append
        first_unregister = sim.register_hitter_ball_listener(listener)
        second_unregister = sim.register_hitter_ball_listener(listener)

        first_unregister()
        first_unregister()
        feed_positions(sim, [[1.0, 0.0, 0.9]])
        self.assertEqual(len(received), 1)

        second_unregister()
        feed_positions(
            sim,
            [[0.99, 0.0, 0.9]],
            start_frame=1,
            start_time=0.01,
        )
        self.assertEqual(len(received), 1)

    def test_raw_x_direction_changes_do_not_advance_epoch_or_reset_estimator(self):
        sim = make_real_world_without_io()
        feed_positions(
            sim,
            [
                [1.00, 0.0, 0.90],
                [1.01, 0.0, 0.90],
            ],
        )
        old_epoch = sim.ball_track_epoch

        feed_positions(
            sim,
            [
                [1.0099999, 0.0, 0.90],
                [1.02, 0.0, 0.90],
                [1.01, 0.0, 0.90],
            ],
            start_frame=2,
            start_time=0.02,
        )

        self.assertEqual(sim.ball_track_epoch, old_epoch)
        self.assertEqual(sim.ball_state_estimator_sample_count_tmp, 5)

    def test_vicon_handler_does_not_advance_epoch_for_raw_x_direction_change(self):
        sim = make_real_world_without_io()
        deliver_vicon(
            sim,
            ball_message(frame=0, source_time=0.0, position=[1.0, 0.0, 0.9]),
        )
        deliver_vicon(
            sim,
            ball_message(
                frame=1,
                source_time=0.01,
                position=[1.0001, 0.0, 0.9],
            ),
        )
        old_epoch = sim.ball_track_epoch

        deliver_vicon(
            sim,
            ball_message(
                frame=2,
                source_time=0.02,
                position=[1.00009999, 0.0, 0.9],
            ),
        )

        self.assertEqual(sim.ball_track_epoch, old_epoch)
        self.assertEqual(sim.ball_state_estimator_sample_count_tmp, 3)

    def test_large_receive_gap_does_not_end_or_restart_track(self):
        sim = make_real_world_without_io()
        received = []
        sim.register_hitter_ball_listener(received.append)

        with patch.object(
            real_world_module.time,
            "monotonic",
            side_effect=[1.0, 1001.0],
        ):
            sim._update_ball_state_from_vicon(
                ball_message(frame=1, source_time=1.0),
                np.array([1.0, 0.0, 0.9]),
            )
            old_epoch = sim.ball_track_epoch
            sim._update_ball_state_from_vicon(
                ball_message(frame=2, source_time=1.01),
                np.array([0.99, 0.0, 0.9]),
            )

        self.assertEqual(sim.ball_track_epoch, old_epoch)
        self.assertEqual(sim.ball_state_estimator_sample_count_tmp, 2)
        self.assertTrue(received[-1].visible)

    def test_invalid_message_publishes_once_and_clears_estimator(self):
        sim = make_real_world_without_io()
        received = []
        sim.register_hitter_ball_listener(received.append)
        feed_positions(sim, [[1.0, 0.0, 0.9], [0.99, 0.0, 0.9]])
        old_epoch = sim.ball_track_epoch
        old_generation = sim.ball_snapshot_generation
        previous_position = sim.ball_pos_world_tmp.copy()

        invalid = ball_message(
            frame=9,
            source_time=4.25,
            valid=0,
            occluded=1,
        )
        before = len(received)
        sim._update_ball_state_from_vicon(invalid, np.zeros(3))

        self.assertEqual(len(received), before + 1)
        item = received[-1]
        self.assertEqual(item.track_epoch, old_epoch + 1)
        self.assertEqual(item.generation, old_generation + 1)
        self.assertEqual(item.source_frame, 9)
        self.assertEqual(item.source_time_s, 4.25)
        self.assertFalse(item.visible)
        self.assertFalse(item.ready)
        np.testing.assert_allclose(item.position_w, previous_position)
        self.assertEqual(sim.ball_state_estimator.sample_count, 0)
        self.assertEqual(sim.ball_state_estimator_sample_count_tmp, 0)

    def test_startup_is_silent_and_public_reset_advances_epoch_without_snapshot(self):
        sim = make_real_world_without_io()
        self.assertEqual(sim.ball_snapshot_generation, 0)
        self.assertEqual(sim.ball_track_epoch, 0)
        self.assertIsNone(sim.latest_ball_snapshot)
        received = []
        sim.register_hitter_ball_listener(received.append)
        feed_positions(sim, [[1.0, 0.0, 0.9]])
        old_epoch = sim.ball_track_epoch
        old_generation = sim.ball_snapshot_generation
        before = len(received)

        sim.reset_ball_state_estimator()

        self.assertEqual(sim.ball_track_epoch, old_epoch + 1)
        self.assertEqual(sim.ball_snapshot_generation, old_generation)
        self.assertEqual(len(received), before)
        self.assertEqual(sim.ball_state_estimator.sample_count, 0)

    def test_ball_message_does_not_fabricate_base_validity(self):
        sim = make_real_world_without_io()
        received = []
        sim.register_hitter_ball_listener(received.append)

        sim._update_ball_state_from_vicon(
            ball_message(frame=1, source_time=1.0),
            np.array([1.0, 0.0, 0.9]),
        )

        self.assertFalse(sim.base_pose_valid_tmp)
        self.assertFalse(received[-1].base_valid)

    def test_only_valid_g1pelvis_controls_base_validity(self):
        sim = make_real_world_without_io()
        initial_position = sim.root_trans_world_tmp.copy()

        deliver_vicon(sim, base_message(name="OtherBody", valid=1))
        self.assertFalse(sim.base_pose_valid_tmp)
        np.testing.assert_allclose(sim.root_trans_world_tmp, initial_position)

        deliver_vicon(sim, base_message(name="G1Pelvis", valid=0, occluded=1))
        self.assertFalse(sim.base_pose_valid_tmp)

        valid = base_message(name="G1Pelvis", valid=1, position=[0.3, 0.2, 0.8])
        deliver_vicon(sim, valid)
        self.assertTrue(sim.base_pose_valid_tmp)
        np.testing.assert_allclose(sim.root_trans_world_tmp, [0.3, 0.2, 0.8])

        deliver_vicon(sim, base_message(name="G1Pelvis", valid=0, occluded=1))
        self.assertFalse(sim.base_pose_valid_tmp)


class RealWorldCommunicationLifecycleTests(unittest.TestCase):
    def test_close_timeout_keeps_subscriptions_retryable(self):
        class FakeLcm:
            def __init__(self):
                self.unsubscribed = []

            def unsubscribe(self, subscription):
                self.unsubscribed.append(subscription)

        class FakeThread:
            def __init__(self):
                self.alive = True
                self.join_timeouts = []

            def is_alive(self):
                return self.alive

            def join(self, timeout=None):
                self.join_timeouts.append(timeout)

        sim = RealWorld.__new__(RealWorld)
        sim.lc = FakeLcm()
        subscriptions = {
            "root_state_subscriber": "root",
            "joint_state_subscriber": "joint",
            "vicon_state_subscriber": "vicon",
            "remote_controller_subscriber": "remote",
            "teleop_state_subscriber": "teleop",
        }
        for attribute, subscription in subscriptions.items():
            setattr(sim, attribute, subscription)
        sim._poll_stop_event = threading.Event()
        sim._communication_close_lock = threading.Lock()
        sim._communication_closed = False
        sim.run_thread = FakeThread()

        with patch.object(real_world_module.logger, "warning") as warning:
            first_close_result = sim.close()

        self.assertIs(first_close_result, False)
        self.assertEqual(sim.run_thread.join_timeouts, [1.0])
        self.assertEqual(warning.call_count, 1)
        self.assertEqual(sim.lc.unsubscribed, [])
        self.assertFalse(sim._communication_closed)

        sim.run_thread.alive = False
        self.assertIs(sim.close(), True)
        self.assertIs(sim.close(), True)

        self.assertCountEqual(sim.lc.unsubscribed, subscriptions.values())
        self.assertEqual(len(sim.lc.unsubscribed), len(subscriptions))
        self.assertTrue(sim._communication_closed)

    def test_close_stops_poll_thread_and_unsubscribes_every_channel_once(self):
        class FakeLcm:
            def __init__(self):
                self.unsubscribed = []
                self.handle_entered = threading.Event()
                self.release_handle = threading.Event()
                self.handling = False
                self.unsubscribed_while_handling = False

            def fileno(self):
                return 123

            def handle(self):
                self.handling = True
                self.handle_entered.set()
                if not self.release_handle.wait(timeout=2.0):
                    raise TimeoutError("test did not release LCM handle")
                self.handling = False

            def unsubscribe(self, subscription):
                self.unsubscribed_while_handling |= self.handling
                self.unsubscribed.append(subscription)

        sim = RealWorld.__new__(RealWorld)
        sim.lc = FakeLcm()
        subscriptions = {
            "root_state_subscriber": "root",
            "joint_state_subscriber": "joint",
            "vicon_state_subscriber": "vicon",
            "remote_controller_subscriber": "remote",
            "teleop_state_subscriber": "teleop",
        }
        for attribute, subscription in subscriptions.items():
            setattr(sim, attribute, subscription)
        sim._poll_stop_event = threading.Event()
        sim._communication_close_lock = threading.Lock()
        sim._communication_closed = False

        def controlled_select(_read, _write, _error, _timeout):
            return [123], [], []

        close_started = threading.Event()
        close_errors = []
        close_results = []

        def run_close():
            close_started.set()
            try:
                close_results.append(sim.close())
            except BaseException as error:
                close_errors.append(error)

        with patch.object(real_world_module.select, "select", controlled_select):
            sim.spin()
            self.assertTrue(sim.lc.handle_entered.wait(timeout=1.0))
            close_thread = threading.Thread(target=run_close)
            close_thread.start()
            self.assertTrue(close_started.wait(timeout=1.0))
            self.assertTrue(sim._poll_stop_event.wait(timeout=1.0))
            try:
                unsubscribed_before_handle_returned = list(sim.lc.unsubscribed)
            finally:
                sim.lc.release_handle.set()
                close_thread.join(timeout=1.0)
                sim.run_thread.join(timeout=1.0)
                already_closed_result = sim.close()

        self.assertFalse(close_thread.is_alive())
        self.assertFalse(sim.run_thread.is_alive())
        self.assertEqual(close_errors, [])
        self.assertEqual(close_results, [True])
        self.assertIs(already_closed_result, True)
        self.assertEqual(unsubscribed_before_handle_returned, [])
        self.assertFalse(sim.lc.unsubscribed_while_handling)
        self.assertCountEqual(sim.lc.unsubscribed, subscriptions.values())
        self.assertEqual(len(sim.lc.unsubscribed), len(subscriptions))

    def test_close_before_spin_succeeds_and_is_idempotent(self):
        class FakeLcm:
            def __init__(self):
                self.unsubscribed = []

            def unsubscribe(self, subscription):
                self.unsubscribed.append(subscription)

        sim = RealWorld.__new__(RealWorld)
        sim.lc = FakeLcm()
        subscriptions = {
            "root_state_subscriber": "root",
            "joint_state_subscriber": "joint",
            "vicon_state_subscriber": "vicon",
            "remote_controller_subscriber": "remote",
            "teleop_state_subscriber": "teleop",
        }
        for attribute, subscription in subscriptions.items():
            setattr(sim, attribute, subscription)
        sim._poll_stop_event = threading.Event()
        sim._communication_close_lock = threading.Lock()
        sim._communication_closed = False

        self.assertIs(sim.close(), True)
        self.assertIs(sim.close(), True)

        self.assertCountEqual(sim.lc.unsubscribed, subscriptions.values())
        self.assertEqual(len(sim.lc.unsubscribed), len(subscriptions))
        self.assertTrue(sim._communication_closed)


class _PolicyGateRealWorld(RealWorld):
    @property
    def right_lower_right_switch_pressed(self):
        return True

    @right_lower_right_switch_pressed.setter
    def right_lower_right_switch_pressed(self, value):
        if not value:
            self._r2_clear_count += 1

    @property
    def base_pose_valid_tmp(self):
        self._base_valid_reads += 1
        return self._base_valid_reads >= 4

    @base_pose_valid_tmp.setter
    def base_pose_valid_tmp(self, value):
        self._base_pose_valid_value = bool(value)


class RealWorldPolicyStartGateTests(unittest.TestCase):
    def test_second_r2_waits_for_base_pose_without_changing_first_calibration(self):
        sim = _PolicyGateRealWorld.__new__(_PolicyGateRealWorld)
        sim._r2_clear_count = 0
        sim._base_valid_reads = 0
        sim._base_pose_valid_value = False
        sim._last_base_pose_wait_log_monotonic_s = None
        sim.cfg = SimpleNamespace(
            control=SimpleNamespace(
                use_residual=False,
                action_scale=1.0,
            )
        )
        sim.active_dof_idx = np.array([0], dtype=np.int64)
        sim.default_angles = np.zeros(1, dtype=np.float32)
        sim.dof_pos = np.zeros(1, dtype=np.float32)
        sim.high_dt = 0.01
        sim._reset_heading_on_start = False
        sim.get_state = lambda: None
        sim.apply_action = lambda action: self.fail(
            "zero-residual first calibration must not publish an action"
        )

        with patch.object(real_world_module.logger, "warning") as warning:
            sim.calibrate(refresh=True)

        self.assertGreaterEqual(sim._base_valid_reads, 4)
        self.assertEqual(sim._r2_clear_count, 2)
        self.assertEqual(warning.call_count, 1)


if __name__ == "__main__":
    unittest.main()
