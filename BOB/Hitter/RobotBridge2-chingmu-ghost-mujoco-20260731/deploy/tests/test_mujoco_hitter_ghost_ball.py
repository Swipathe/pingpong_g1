from __future__ import annotations

import threading
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import mujoco
import numpy as np

from simulator.mujoco import Mujoco
from utils.hitter_ball_pipeline import BasePoseW
from utils.hitter_realtime import BallEstimateSnapshot
from utils.hitter_runtime_capabilities import LoopPacing, PlannerFeed


DEPLOY_DIR = Path(__file__).resolve().parents[1]
XML_PATH = (
    DEPLOY_DIR
    / "data"
    / "assets"
    / "g1"
    / "g1_29dof_hitter_racket_table_tennis.xml"
)


def _geom_id(model, name: str) -> int:
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    if geom_id < 0:
        raise AssertionError(f"missing geom: {name}")
    return int(geom_id)


def _body_id(model, name: str) -> int:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if body_id < 0:
        raise AssertionError(f"missing body: {name}")
    return int(body_id)


def _snapshot(
    *,
    visible: bool = True,
    ready: bool = True,
    received_monotonic_s: float = 10.0,
) -> BallEstimateSnapshot:
    return BallEstimateSnapshot(
        track_epoch=3,
        generation=7,
        source_frame=101,
        source_time_s=1.25,
        received_monotonic_s=received_monotonic_s,
        position_w=np.array([1.2, 0.1, 1.1], dtype=np.float32),
        velocity_w=np.array([2.0, 0.0, -1.0], dtype=np.float32),
        base_position_w=np.array([-0.4, 0.0, 0.79], dtype=np.float32),
        base_quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32),
        base_valid=True,
        visible=visible,
        ready=ready,
    )


def _minimal_config(*, mode: str = "mujoco_direct") -> SimpleNamespace:
    return SimpleNamespace(
        table_tennis={
            "enabled": True,
            "randomize_ball_on_reset": False,
            "ball_initial_pos": [1.8, 0.0, 1.05],
            "ball_initial_lin_vel": [-2.0, 0.0, 0.0],
            "ball_initial_ang_vel": [0.0, 0.0, 0.0],
        },
        hitter_ball_input={
            "mode": mode,
            "lcm_url": "memq://",
            "channel": "vicon_state_data",
            "stale_timeout_s": 0.15,
        },
        motion={
            "ball_planner": {
                "state_estimator_window_size": 31,
                "state_estimator_min_samples": 1,
                "state_estimator_sample_rate_hz": 360.0,
                "table_center_xy_w": [1.365369, 0.0],
                "table_height": 0.76,
                "table_length": 2.730738,
                "table_width": 1.512451,
            }
        },
        control=SimpleNamespace(
            update_with_fk=False,
            use_teleop=False,
            use_residual=False,
            action_scale=0.25,
        ),
    )


def _minimal_simulator(*, mode: str = "mujoco_direct") -> Mujoco:
    simulator = Mujoco.__new__(Mujoco)
    simulator.cfg = _minimal_config(mode=mode)
    simulator.low_dt = 0.005
    simulator.decimation = 1
    simulator.high_dt = 0.005
    simulator.mujoco_model = mujoco.MjModel.from_xml_path(str(XML_PATH))
    simulator.mujoco_data = mujoco.MjData(simulator.mujoco_model)
    simulator.default_qpos = simulator.mujoco_data.qpos.copy()
    simulator.default_qvel = simulator.mujoco_data.qvel.copy()
    simulator.viewer = None
    simulator.viewer_enabled = False
    simulator.num_dof = 29
    simulator.num_action = 29
    simulator.default_angles = np.zeros(29, dtype=np.float32)
    simulator.kps = np.ones(29, dtype=np.float32)
    simulator.kds = np.ones(29, dtype=np.float32)
    simulator.active_dof_idx = np.arange(29, dtype=np.int32)
    simulator.frozen_dof_idx = np.zeros(0, dtype=np.int32)
    simulator.foot_body_ids = np.zeros(0, dtype=np.int32)
    simulator.dof_pos = np.zeros(29, dtype=np.float32)
    simulator.dof_vel = np.zeros(29, dtype=np.float32)
    simulator.root_trans_world = np.array([0.0, 0.0, 0.793], dtype=np.float32)
    simulator.root_quat_world = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
    simulator.base_pose_valid = True
    simulator.teleop_quat_tmp = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    simulator.teleop_dof_pos_tmp = np.zeros(29, dtype=np.float64)
    simulator._init_teleop = False
    simulator._base_rot = None
    simulator._base_pos = np.zeros(3, dtype=np.float64)
    simulator._init_table_tennis_state()
    return simulator


class MujocoGhostXmlTests(unittest.TestCase):
    def test_asset_contains_visual_only_mocap_ghost_body(self) -> None:
        root = ET.parse(XML_PATH).getroot()
        ghost_body = root.find(".//body[@name='hitter_ball_ghost']")
        self.assertIsNotNone(ghost_body)
        self.assertEqual(ghost_body.attrib.get("mocap"), "true")
        self.assertEqual(ghost_body.attrib.get("pos"), "0 0 -10")

        ghost_geom = root.find(".//geom[@name='hitter_ball_ghost_geom']")
        self.assertIsNotNone(ghost_geom)
        self.assertEqual(ghost_geom.attrib.get("contype"), "0")
        self.assertEqual(ghost_geom.attrib.get("conaffinity"), "0")
        self.assertEqual(ghost_geom.attrib.get("rgba"), "1 0.35 0.1 0")

        for pair in root.findall(".//contact/pair"):
            self.assertNotIn(
                "hitter_ball_ghost_geom",
                (pair.attrib.get("geom1"), pair.attrib.get("geom2")),
            )

        model = mujoco.MjModel.from_xml_path(str(XML_PATH))
        data = mujoco.MjData(model)
        ghost_body_id = _body_id(model, "hitter_ball_ghost")
        ghost_geom_id = _geom_id(model, "hitter_ball_ghost_geom")
        self.assertGreaterEqual(int(model.body_mocapid[ghost_body_id]), 0)
        np.testing.assert_allclose(data.mocap_pos[0], [0.0, 0.0, -10.0])
        self.assertEqual(int(model.geom_contype[ghost_geom_id]), 0)
        self.assertEqual(int(model.geom_conaffinity[ghost_geom_id]), 0)
        self.assertEqual(float(model.geom_rgba[ghost_geom_id, 3]), 0.0)


class MujocoGhostRuntimeTests(unittest.TestCase):
    def test_queueing_copies_snapshot_without_mutating_mjdata_until_apply(self) -> None:
        simulator = _minimal_simulator()
        simulator.set_hitter_ghost_mode(True)
        qpos_before = simulator.mujoco_data.qpos.copy()
        qvel_before = simulator.mujoco_data.qvel.copy()
        mocap_before = simulator.mujoco_data.mocap_pos.copy()

        snapshot = _snapshot(received_monotonic_s=10.0)
        simulator.queue_hitter_ghost_snapshot(snapshot)
        snapshot.position_w[0]
        np.testing.assert_array_equal(simulator.mujoco_data.qpos, qpos_before)
        np.testing.assert_array_equal(simulator.mujoco_data.qvel, qvel_before)
        np.testing.assert_array_equal(simulator.mujoco_data.mocap_pos, mocap_before)

        simulator.apply_pending_hitter_ghost_snapshot(now_monotonic_s=10.02)
        ghost_geom_id = _geom_id(simulator.mujoco_model, "hitter_ball_ghost_geom")
        np.testing.assert_array_equal(simulator.mujoco_data.qpos, qpos_before)
        np.testing.assert_array_equal(simulator.mujoco_data.qvel, qvel_before)
        np.testing.assert_allclose(
            simulator.mujoco_data.mocap_pos[simulator.hitter_ball_ghost_mocap_id],
            [1.24, 0.1, 1.08],
            atol=1.0e-6,
        )
        np.testing.assert_allclose(
            simulator.mujoco_data.mocap_quat[simulator.hitter_ball_ghost_mocap_id],
            [1.0, 0.0, 0.0, 0.0],
        )
        self.assertGreater(float(simulator.mujoco_model.geom_rgba[ghost_geom_id, 3]), 0.9)

    def test_invalid_snapshot_hides_ghost_and_contacts_never_involve_it(self) -> None:
        simulator = _minimal_simulator()
        simulator.set_hitter_ghost_mode(True)
        simulator.queue_hitter_ghost_snapshot(_snapshot())
        simulator.apply_pending_hitter_ghost_snapshot(now_monotonic_s=10.0)
        simulator.queue_hitter_ghost_snapshot(_snapshot(visible=False, ready=False))
        simulator.apply_pending_hitter_ghost_snapshot(now_monotonic_s=10.1)

        ghost_geom_id = _geom_id(simulator.mujoco_model, "hitter_ball_ghost_geom")
        self.assertEqual(float(simulator.mujoco_model.geom_rgba[ghost_geom_id, 3]), 0.0)
        np.testing.assert_allclose(
            simulator.mujoco_data.mocap_pos[simulator.hitter_ball_ghost_mocap_id],
            [0.0, 0.0, -10.0],
        )

        for _ in range(100):
            mujoco.mj_step(simulator.mujoco_model, simulator.mujoco_data)
        contacts = [
            {
                int(simulator.mujoco_data.contact[index].geom1),
                int(simulator.mujoco_data.contact[index].geom2),
            }
            for index in range(simulator.mujoco_data.ncon)
        ]
        self.assertTrue(all(ghost_geom_id not in pair for pair in contacts))

    def test_ghost_mode_parks_physical_ball_defaults_and_disables_physical_helpers(self) -> None:
        simulator = _minimal_simulator()
        simulator.set_hitter_ghost_mode(True)
        qposadr = simulator.hitter_ball_qposadr
        qveladr = simulator.hitter_ball_qveladr
        np.testing.assert_allclose(simulator.default_qpos[qposadr:qposadr + 3], [0.0, 0.0, -10.0])
        np.testing.assert_allclose(simulator.default_qvel[qveladr:qveladr + 6], np.zeros(6))
        self.assertFalse(simulator.capture_hitter_ball_launch_state())
        self.assertFalse(simulator.restore_hitter_ball_launch_state())

        simulator.reset_hitter_ball(pos=[1.8, 0.0, 1.0], lin_vel=[-2.0, 0.0, 0.0])
        np.testing.assert_allclose(simulator.mujoco_data.qpos[qposadr:qposadr + 3], [0.0, 0.0, -10.0])
        simulator.calibrate(refresh=True)
        np.testing.assert_allclose(simulator.mujoco_data.qpos[qposadr:qposadr + 3], [0.0, 0.0, -10.0])
        np.testing.assert_allclose(simulator.mujoco_data.qvel[qveladr:qveladr + 6], np.zeros(6))
        self.assertFalse(simulator.ball_visible)
        self.assertFalse(simulator.ball_state_estimator_ready)

    def test_copy_base_pose_uses_mujoco_cached_state_with_xyzw_quaternion(self) -> None:
        simulator = _minimal_simulator()
        simulator.root_trans_world = np.array([-0.3, 0.2, 0.8], dtype=np.float64)
        simulator.root_quat_world = np.array([0.0, 0.0, 0.70710678, 0.70710678], dtype=np.float64)
        simulator.base_pose_valid = True

        pose = simulator.copy_hitter_base_pose_w()

        self.assertIsInstance(pose, BasePoseW)
        np.testing.assert_allclose(pose.position_w, [-0.3, 0.2, 0.8])
        np.testing.assert_allclose(pose.quaternion_xyzw, [0.0, 0.0, 0.70710678, 0.70710678])
        self.assertTrue(pose.valid)

    def test_hil_input_constructs_one_source_starts_it_and_sets_hybrid_capability(self) -> None:
        simulator = _minimal_simulator(mode="chingmu_ghost")
        source = mock.Mock()
        source.register_hitter_ball_listener.return_value = lambda: None
        pipeline = mock.Mock()

        with mock.patch("simulator.mujoco.RealtimeViconBallPipeline", return_value=pipeline) as pipeline_type:
            with mock.patch("simulator.mujoco.ChingMuGhostBallSource", return_value=source) as source_type:
                simulator._hitter_lcm_publication_audit = object()
                simulator._init_hitter_ball_input()

        pipeline_type.assert_called_once()
        source_type.assert_called_once()
        source.start.assert_called_once()
        source.register_hitter_ball_listener.assert_called_once_with(
            simulator.queue_hitter_ghost_snapshot
        )
        self.assertIs(simulator.hitter_ball_source, source)
        self.assertTrue(simulator.hitter_runtime_capabilities.uses_snapshot_stream)
        self.assertEqual(simulator.hitter_runtime_capabilities.loop_pacing, LoopPacing.SIMULATOR)

    def test_direct_mode_keeps_inline_capability_and_source_self(self) -> None:
        simulator = _minimal_simulator(mode="mujoco_direct")
        simulator._init_hitter_ball_input()

        self.assertIs(simulator.hitter_ball_source, simulator)
        self.assertEqual(simulator.hitter_runtime_capabilities.planner_feed, PlannerFeed.INLINE_STATE)

    def test_close_orders_source_poll_thread_unsubscribe_and_viewer(self) -> None:
        simulator = _minimal_simulator(mode="chingmu_ghost")
        events = []
        source = SimpleNamespace(close=lambda **_kwargs: events.append("source") or True)
        simulator.hitter_ball_source = source
        simulator._hitter_ghost_unregister_listener = lambda: events.append("unregister")
        simulator._mujoco_poll_stop_event = threading.Event()
        simulator.run_thread = SimpleNamespace(
            is_alive=lambda: False,
            join=lambda timeout=None: events.append("join"),
        )
        simulator.lc = SimpleNamespace(unsubscribe=lambda sub: events.append(("unsubscribe", sub)))
        simulator.teleop_state_subscriber = object()
        simulator.viewer = SimpleNamespace(close=lambda: events.append("viewer"))

        self.assertTrue(simulator.close())

        self.assertEqual(events[0], "source")
        self.assertEqual(events[1], "unregister")
        self.assertIn("join", events)
        self.assertIn(("unsubscribe", simulator.teleop_state_subscriber), events)
        self.assertIn("viewer", events)
        self.assertTrue(simulator._mujoco_poll_stop_event.is_set())


if __name__ == "__main__":
    unittest.main()
