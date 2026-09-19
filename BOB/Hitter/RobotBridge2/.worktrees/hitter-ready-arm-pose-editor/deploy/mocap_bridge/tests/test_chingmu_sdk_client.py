import threading
import unittest

import numpy as np

from deploy.mocap_bridge import chingmu_sdk_client
from deploy.mocap_bridge.chingmu_sdk_client import (
    ChingMuSdkClient,
    HierarchyReport,
    Timeval,
    TrackerReport,
)


class FakeFunction:
    def __init__(self, implementation=lambda *args: True):
        self.implementation = implementation
        self.argtypes = None
        self.restype = None

    def __call__(self, *args):
        return self.implementation(*args)


class FakeLibrary:
    def __init__(self):
        self.calls = []
        self.hierarchy_callback = None
        self.end_hierarchy_callback = None
        self.tracker_callback = None
        self.CMVrpnStartExtern = FakeFunction(self._start)
        self.CMVrpnQuitExtern = FakeFunction(self._quit)
        self.CMPluginConnectServer = FakeFunction(self._connect)
        self.CMPluginRegisterUpdateHierarchy = FakeFunction(self._register_hierarchy)
        self.CMPluginUnRegisterUpdateHierarchy = FakeFunction(self._unregister_hierarchy)
        self.CMPluginRegisterEndHierarchy = FakeFunction(
            self._register_end_hierarchy
        )
        self.CMPluginRegisterTrackerData = FakeFunction(self._register_tracker)
        self.CMPluginUnRegisterTrackerData = FakeFunction(self._unregister_tracker)
        self.CMTrackerExternTC = FakeFunction(self._tracker_extern_tc)
        self.polled_body_poses = {}

    def _start(self):
        self.calls.append(("start",))

    def _quit(self):
        self.calls.append(("quit",))
        self.hierarchy_callback = None
        self.end_hierarchy_callback = None
        self.tracker_callback = None

    def _connect(self, host):
        self.calls.append(("connect", bytes(host)))
        return True

    def _register_hierarchy(self, host, _userdata, callback):
        self.calls.append(("register_hierarchy", bytes(host), callback))
        self.hierarchy_callback = callback
        return True

    def _unregister_hierarchy(self, host, _userdata, callback):
        self.calls.append(("unregister_hierarchy", bytes(host), callback))
        return self.hierarchy_callback is callback

    def _register_end_hierarchy(self, host, _userdata, callback):
        self.calls.append(("register_end_hierarchy", bytes(host), callback))
        self.end_hierarchy_callback = callback
        return True

    def _register_tracker(self, host, _userdata, callback):
        self.calls.append(("register_tracker", bytes(host), callback))
        self.tracker_callback = callback
        return True

    def _unregister_tracker(self, host, _userdata, callback):
        self.calls.append(("unregister_tracker", bytes(host), callback))
        return self.tracker_callback is callback

    def _tracker_extern_tc(self, host, body_id, timecode_data, body_pos, body_rot, tv):
        host_bytes = bytes(host)
        self.calls.append(("tracker_extern_tc", host_bytes, int(body_id)))
        pose = self.polled_body_poses.get((host_bytes, int(body_id)))
        if pose is None:
            return False
        position, quaternion, source_time = pose
        timecode_data[0] = int(source_time * 1000.0)
        body_pos[:] = position
        body_rot[:] = quaternion
        timeval = tv._obj if hasattr(tv, "_obj") else tv.contents
        timeval.tv_sec = int(source_time)
        timeval.tv_usec = int(round((source_time % 1.0) * 1.0e6))
        return True

    def set_polled_body_pose(
        self,
        body_id,
        position,
        quaternion=(0.0, 0.0, 0.0, 1.0),
        source_time=10.0,
        host=b"G1Pelvis@192.168.2.100",
    ):
        self.polled_body_poses[(bytes(host), int(body_id))] = (
            tuple(position),
            tuple(quaternion),
            float(source_time),
        )

    def emit_hierarchy(self, sensor, name, parent=-1):
        report = HierarchyReport()
        report.sensor = sensor
        report.parent = parent
        report.name = name.encode("utf-8")
        self.hierarchy_callback(None, report)

    def emit_end_hierarchy(self, retarget_flag=0, source_time=10.0):
        report = chingmu_sdk_client.EndHierarchyReport()
        report.msg_time = Timeval(
            int(source_time), int(round((source_time % 1.0) * 1.0e6))
        )
        report.retarget_flag = retarget_flag
        self.end_hierarchy_callback(None, report)

    def emit_tracker(
        self,
        sensor,
        frame,
        position,
        source_time=10.0,
        quaternion=(0.0, 0.0, 0.0, 1.0),
    ):
        report = TrackerReport()
        report.msg_time = Timeval(int(source_time), int(round((source_time % 1.0) * 1.0e6)))
        report.sensor = sensor
        report.frameCounter = frame
        report.pos[:] = position
        report.quat[:] = quaternion
        self.tracker_callback(None, report)


class ChingMuSdkClientTest(unittest.TestCase):
    def make_client(
        self,
        body_name="G1Pelvis",
        body_pose_id=None,
        monotonic_fn=None,
    ):
        self.library = FakeLibrary()
        self.sleeps = []
        client = ChingMuSdkClient(
            "unused.so",
            server_ip="192.168.2.100",
            body_name=body_name,
            body_pose_id=body_pose_id,
            library=self.library,
            sleep_fn=self.sleeps.append,
            monotonic_fn=monotonic_fn or chingmu_sdk_client.time.monotonic,
            discovery_delay_s=1.0,
        )
        return client

    def start_client(self, client, body_id=6, body_name="G1Pelvis"):
        def publish_hierarchy(_delay):
            self.sleeps.append(_delay)
            if self.library.hierarchy_callback is not None:
                self.library.emit_hierarchy(body_id, body_name)
            if self.library.end_hierarchy_callback is not None:
                self.library.emit_end_hierarchy()

        client.sleep_fn = publish_hierarchy
        client.start(resolve_timeout_s=0.1)

    def test_uses_aggregate_server_for_stream_and_subject_server_for_pose(self):
        client = self.make_client()
        self.start_client(client)

        connects = [call for call in self.library.calls if call[0] == "connect"]
        self.assertEqual(connects, [("connect", b"MCAvatar@192.168.2.100")])
        self.assertEqual(client.body_pose_server, b"G1Pelvis@192.168.2.100")
        self.assertEqual(self.sleeps[0], 1.0)
        client.close()

    def test_resolves_exact_body_name_without_hardcoding_id(self):
        client = self.make_client()
        self.start_client(client, body_id=17)

        self.assertEqual(client.body_id, 17)
        self.assertEqual(client.body_marker_sensor_range, range(8650, 8700))
        client.close()

    def test_same_named_non_root_hierarchy_node_is_not_resolved_as_body(self):
        client = self.make_client()

        def publish_same_named_nodes(_delay):
            self.sleeps.append(_delay)
            if self.library.hierarchy_callback is not None:
                self.library.emit_hierarchy(6, "G1Pelvis")
                self.library.emit_hierarchy(7, "G1Pelvis", parent=6)
            if self.library.end_hierarchy_callback is not None:
                self.library.emit_end_hierarchy()

        client.sleep_fn = publish_same_named_nodes
        client.start(resolve_timeout_s=0.1)

        self.assertEqual(client.body_id, 6)
        self.assertEqual(client.body_marker_sensor_range, range(8100, 8150))
        client.close()

    def test_start_waits_for_hierarchy_end_before_tracker_stream(self):
        client = self.make_client()
        first_wait_release = threading.Event()
        second_wait_release = threading.Event()
        state_condition = threading.Condition()
        state = {"resolve_waits": 0, "finished": False}
        start_errors = []
        clock = [0.0]

        def fake_monotonic():
            now = clock[0]
            clock[0] += 0.01
            return now

        def controlled_sleep(delay):
            self.sleeps.append(delay)
            if self.library.hierarchy_callback is None:
                return
            with state_condition:
                state["resolve_waits"] += 1
                wait_number = state["resolve_waits"]
                state_condition.notify_all()
            if wait_number == 1:
                first_wait_release.wait(timeout=1.0)
            elif wait_number == 2:
                second_wait_release.wait(timeout=1.0)

        def run_start():
            try:
                client.start(resolve_timeout_s=1.0)
            except Exception as error:
                start_errors.append(error)
            finally:
                with state_condition:
                    state["finished"] = True
                    state_condition.notify_all()

        client.monotonic_fn = fake_monotonic
        client.sleep_fn = controlled_sleep
        start_thread = threading.Thread(target=run_start, daemon=True)
        start_thread.start()

        try:
            with state_condition:
                entered_first_wait = state_condition.wait_for(
                    lambda: state["resolve_waits"] >= 1, timeout=1.0
                )
            self.assertTrue(entered_first_wait)

            self.library.emit_hierarchy(6, "G1Pelvis")
            first_wait_release.set()

            with state_condition:
                reached_barrier = state_condition.wait_for(
                    lambda: state["resolve_waits"] >= 2 or state["finished"],
                    timeout=1.0,
                )
            self.assertTrue(reached_barrier)
            self.assertIsNone(self.library.tracker_callback)
            self.assertIsNone(client.body_id)

            self.library.emit_hierarchy(7, "OtherBody")
            self.library.emit_end_hierarchy()
            second_wait_release.set()
            start_thread.join(timeout=1.0)

            self.assertFalse(start_thread.is_alive())
            self.assertEqual(start_errors, [])
            self.assertIsNotNone(self.library.tracker_callback)

            self.library.emit_tracker(7, 200, (9.0, 9.0, 9.0))
            self.library.emit_tracker(8100, 201, (1.0, 2.0, 3.0))
            frame = client.next_frame(timeout_s=0.01)

            self.assertEqual(frame.frame_number, 200)
            self.assertEqual(frame.unlabeled_markers_mm.shape, (0, 3))
        finally:
            first_wait_release.set()
            second_wait_release.set()
            start_thread.join(timeout=1.0)
            client.close()

    def test_failed_start_does_not_reuse_discovery_state_on_retry(self):
        client = self.make_client()
        self.addCleanup(client.close)
        clock = [0.0]

        def fake_monotonic():
            now = clock[0]
            clock[0] += 0.01
            return now

        def publish_root_only(delay):
            self.sleeps.append(delay)
            if self.library.hierarchy_callback is not None:
                self.library.emit_hierarchy(6, "G1Pelvis")

        client.monotonic_fn = fake_monotonic
        client.sleep_fn = publish_root_only
        with self.assertRaises(TimeoutError) as first_error:
            client.start(resolve_timeout_s=0.03)

        self.assertIn("hierarchy_complete=False", str(first_error.exception))
        self.assertIsNone(client.body_id)
        self.assertIsNone(self.library.tracker_callback)

        def publish_end_only(delay):
            self.sleeps.append(delay)
            if self.library.end_hierarchy_callback is not None:
                self.library.emit_end_hierarchy()

        client.sleep_fn = publish_end_only
        with self.assertRaises(TimeoutError) as second_error:
            client.start(resolve_timeout_s=0.03)

        self.assertIn("hierarchy_complete=True", str(second_error.exception))
        self.assertIsNone(client.body_id)
        self.assertEqual(client.body_marker_sensor_range, range(0, 0))
        self.assertIsNone(self.library.tracker_callback)
        tracker_registrations = [
            call for call in self.library.calls if call[0] == "register_tracker"
        ]
        self.assertEqual(tracker_registrations, [])

        def publish_complete_hierarchy(delay):
            self.sleeps.append(delay)
            if self.library.hierarchy_callback is not None:
                self.library.emit_hierarchy(6, "G1Pelvis")
            if self.library.end_hierarchy_callback is not None:
                self.library.emit_end_hierarchy()

        client.sleep_fn = publish_complete_hierarchy
        client.start(resolve_timeout_s=0.03)

        self.assertEqual(client.body_id, 6)
        self.assertEqual(client.body_marker_sensor_range, range(8100, 8150))
        self.assertIsNotNone(self.library.tracker_callback)
        client.close()

    def test_resolved_body_root_report_is_preserved_as_xyzw(self):
        client = self.make_client()
        self.start_client(client, body_id=6)
        self.library.emit_tracker(
            6,
            100,
            (410.0, -20.0, 793.0),
            quaternion=(0.0, 0.0, 0.2, 0.98),
        )
        self.library.emit_tracker(8100, 101, (1.0, 2.0, 3.0))

        frame = client.next_frame(timeout_s=0.01)

        np.testing.assert_allclose(frame.body_position_mm, [410.0, -20.0, 793.0])
        np.testing.assert_allclose(
            frame.body_quaternion_xyzw, [0.0, 0.0, 0.2, 0.98]
        )
        self.assertEqual(frame.unlabeled_markers_mm.shape, (0, 3))
        client.close()

    def test_next_frame_polls_body_pose_when_root_callback_is_absent(self):
        client = self.make_client(body_name="G1pelvis")
        self.start_client(client, body_id=6, body_name="G1pelvis")
        self.library.set_polled_body_pose(
            6,
            (410.0, -20.0, 793.0),
            quaternion=(0.0, 0.0, 0.2, 0.98),
            source_time=12.5,
            host=b"G1pelvis@192.168.2.100",
        )
        self.library.emit_tracker(8100, 100, (1.0, 2.0, 3.0), source_time=12.5)
        self.library.emit_tracker(8100, 101, (1.0, 2.0, 3.0), source_time=12.5028)

        frame = client.next_frame(timeout_s=0.01)

        np.testing.assert_allclose(frame.body_position_mm, [410.0, -20.0, 793.0])
        np.testing.assert_allclose(
            frame.body_quaternion_xyzw, [0.0, 0.0, 0.2, 0.98]
        )
        self.assertIn(
            ("tracker_extern_tc", b"G1pelvis@192.168.2.100", 6),
            self.library.calls,
        )
        client.close()

    def test_explicit_body_pose_id_overrides_hierarchy_id_for_polling(self):
        client = self.make_client(body_name="G1pelvis", body_pose_id=300)
        self.start_client(client, body_id=6, body_name="G1pelvis")
        self.library.set_polled_body_pose(
            300,
            (510.0, 20.0, 893.0),
            quaternion=(0.1, 0.2, 0.3, 0.9),
            source_time=12.5,
            host=b"G1pelvis@192.168.2.100",
        )
        self.library.emit_tracker(8100, 100, (1.0, 2.0, 3.0), source_time=12.5)
        self.library.emit_tracker(8100, 101, (1.0, 2.0, 3.0), source_time=12.5028)

        frame = client.next_frame(timeout_s=0.01)

        self.assertEqual(client.body_id, 6)
        self.assertEqual(client.body_pose_id, 300)
        np.testing.assert_allclose(frame.body_position_mm, [510.0, 20.0, 893.0])
        np.testing.assert_allclose(
            frame.body_quaternion_xyzw, [0.1, 0.2, 0.3, 0.9]
        )
        self.assertIn(
            ("tracker_extern_tc", b"G1pelvis@192.168.2.100", 300),
            self.library.calls,
        )
        client.close()

    def test_other_hierarchy_root_is_not_used_as_g1pelvis(self):
        client = self.make_client()
        self.start_client(client, body_id=6)
        self.library.emit_hierarchy(7, "OtherBody")
        self.library.emit_tracker(7, 200, (9.0, 9.0, 9.0))
        self.library.emit_tracker(8100, 201, (1.0, 2.0, 3.0))

        frame = client.next_frame(timeout_s=0.01)

        self.assertIsNone(frame.body_position_mm)
        self.assertIsNone(frame.body_quaternion_xyzw)
        self.assertEqual(frame.unlabeled_markers_mm.shape, (0, 3))
        client.close()

    def test_tracker_before_other_root_hierarchy_is_not_unlabeled(self):
        client = self.make_client()
        self.start_client(client, body_id=6)
        self.library.emit_tracker(7, 200, (9.0, 9.0, 9.0))
        self.library.emit_hierarchy(7, "OtherBody")
        self.library.emit_tracker(8100, 201, (1.0, 2.0, 3.0))

        frame = client.next_frame(timeout_s=0.01)

        self.assertEqual(frame.unlabeled_markers_mm.shape, (0, 3))
        client.close()

    def test_groups_body_and_unlabeled_markers_from_the_same_frame(self):
        client = self.make_client()
        self.start_client(client, body_id=6)

        self.library.emit_tracker(8100, 100, (1799.5, -18.8, 10.8), source_time=10.25)
        self.library.emit_tracker(8101, 100, (1801.0, -20.0, 12.0), source_time=10.25)
        self.library.emit_tracker(102312, 100, (-1362.5, 751.7, 5.5), source_time=10.25)
        self.library.emit_tracker(102317, 100, (-1359.4, -760.0, 8.1), source_time=10.25)
        self.library.emit_tracker(8100, 101, (1799.5, -18.8, 10.8), source_time=10.2528)

        frame = client.next_frame(timeout_s=0.01)

        self.assertEqual(frame.frame_number, 100)
        self.assertAlmostEqual(frame.source_time_s, 10.25)
        self.assertEqual(set(frame.body_markers_mm), {8100, 8101})
        np.testing.assert_allclose(frame.body_markers_mm[8100], [1799.5, -18.8, 10.8])
        np.testing.assert_allclose(
            frame.unlabeled_markers_mm,
            [[-1362.5, 751.7, 5.5], [-1359.4, -760.0, 8.1]],
        )
        client.close()

    def test_does_not_mix_other_rigid_body_markers_into_g1pelvis(self):
        client = self.make_client()
        self.start_client(client, body_id=6)
        self.library.emit_hierarchy(1, "OtherBody")
        self.library.emit_tracker(7850, 200, (9.0, 9.0, 9.0))
        self.library.emit_tracker(8100, 200, (1.0, 2.0, 3.0))
        self.library.emit_tracker(102400, 200, (4.0, 5.0, 6.0))
        self.library.emit_tracker(8100, 201, (1.0, 2.0, 3.0))

        frame = client.next_frame(timeout_s=0.01)

        self.assertEqual(set(frame.body_markers_mm), {8100})
        np.testing.assert_allclose(frame.unlabeled_markers_mm, [[4.0, 5.0, 6.0]])
        client.close()

    def test_accepts_dynamic_unlabeled_sensor_ids_below_old_threshold(self):
        client = self.make_client()
        self.start_client(client, body_id=6)

        self.library.emit_tracker(53170, 300, (10.0, 20.0, 30.0))
        self.library.emit_tracker(8100, 301, (1.0, 2.0, 3.0))

        frame = client.next_frame(timeout_s=0.01)

        np.testing.assert_allclose(frame.unlabeled_markers_mm, [[10.0, 20.0, 30.0]])
        client.close()

    def test_close_uses_vendor_quit_without_racy_callback_unregister(self):
        client = self.make_client()
        self.start_client(client)

        client.close()
        client.close()

        hierarchy_calls = [call for call in self.library.calls if call[0] == "unregister_hierarchy"]
        tracker_calls = [call for call in self.library.calls if call[0] == "unregister_tracker"]
        self.assertEqual(hierarchy_calls, [])
        self.assertEqual(tracker_calls, [])
        self.assertEqual(sum(call[0] == "quit" for call in self.library.calls), 1)

    def test_slow_consumer_gets_latest_frame_instead_of_stale_backlog(self):
        client = self.make_client()
        self.start_client(client, body_id=6)

        self.library.emit_tracker(8100, 100, (1.0, 0.0, 0.0))
        self.library.emit_tracker(8100, 101, (2.0, 0.0, 0.0))
        self.library.emit_tracker(8100, 102, (3.0, 0.0, 0.0))

        frame = client.next_frame(timeout_s=0.01)

        self.assertEqual(frame.frame_number, 101)
        np.testing.assert_allclose(frame.body_markers_mm[8100], [2.0, 0.0, 0.0])
        self.assertEqual(client.dropped_frame_count, 1)
        client.close()


if __name__ == "__main__":
    unittest.main()
