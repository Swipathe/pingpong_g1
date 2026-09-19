from __future__ import annotations

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np
import yaml

from envs.hitter import HitterEnv
from simulator.mujoco import Mujoco

DEPLOY_DIR = Path(__file__).resolve().parents[1]
XML_PATH = DEPLOY_DIR / "data" / "assets" / "g1" / "g1_29dof_hitter_racket_table_tennis.xml"


def _geom_id(model, name: str) -> int:
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    if geom_id < 0:
        raise AssertionError(f"缺少 MuJoCo geom: {name}")
    return int(geom_id)


def _has_contact(data, geom_a: int, geom_b: int) -> bool:
    expected = {geom_a, geom_b}
    return any({int(data.contact[i].geom1), int(data.contact[i].geom2)} == expected for i in range(data.ncon))


def _minimal_table_tennis_simulator(table_tennis_cfg: dict) -> Mujoco:
    simulator = Mujoco.__new__(Mujoco)
    simulator.cfg = SimpleNamespace(table_tennis=table_tennis_cfg)
    simulator.mujoco_model = mujoco.MjModel.from_xml_path(str(XML_PATH))
    simulator.mujoco_data = mujoco.MjData(simulator.mujoco_model)
    simulator.default_qpos = simulator.mujoco_data.qpos.copy()
    simulator.default_qvel = simulator.mujoco_data.qvel.copy()
    simulator.viewer = None
    simulator._init_table_tennis_state()
    return simulator


def _scheduled_serve_env(simulator: Mujoco) -> HitterEnv:
    env = HitterEnv.__new__(HitterEnv)
    env.simulator = simulator
    env.motion_cfg = {"mujoco_serve_times_s": [0.0, 20.0]}
    env.hitter_ball_sequence_needs_reset = True
    env._hitter_sync_track_epoch = 0
    env._hitter_sync_generation = 7
    env._init_mujoco_serve_schedule_state()
    return env


class RecordingSimulator:
    def __init__(self):
        self.analytic_hit_calls = 0

    def set_hitter_analytic_racket_hit(self, _target_pos, _outgoing_vel):
        self.analytic_hit_calls += 1


class BallLaunchStateTests(unittest.TestCase):
    def test_capture_and_restore_reproduces_complete_launch_state(self):
        initial_quat_wxyz = np.array([0.5, 0.5, 0.5, 0.5], dtype=np.float64)
        disturbed_quat_wxyz = np.array([np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)], dtype=np.float64)
        simulator = _minimal_table_tennis_simulator(
            {
                "enabled": True,
                "ball_initial_quat_wxyz": initial_quat_wxyz.tolist(),
            }
        )
        simulator.reset_hitter_ball(
            pos=[1.8, 0.1, 1.05],
            lin_vel=[-2.2, 0.2, 0.3],
        )
        qposadr = simulator.hitter_ball_qposadr
        qveladr = simulator.hitter_ball_qveladr
        body_id = simulator.hitter_ball_body_id
        simulator.mujoco_data.qvel[qveladr + 3 : qveladr + 6] = [1.0, 2.0, 3.0]
        mujoco.mj_forward(simulator.mujoco_model, simulator.mujoco_data)
        simulator._update_hitter_ball_state()

        expected_qpos = simulator.mujoco_data.qpos[qposadr : qposadr + 7].copy()
        expected_qvel = simulator.mujoco_data.qvel[qveladr : qveladr + 6].copy()
        expected_xpos = simulator.mujoco_data.xpos[body_id].copy()
        expected_xquat_wxyz = simulator.mujoco_data.xquat[body_id].copy()
        self.assertTrue(simulator.capture_hitter_ball_launch_state())

        simulator.mujoco_data.qpos[qposadr : qposadr + 7] = np.concatenate(([0.2, -0.4, 0.3], disturbed_quat_wxyz))
        simulator.mujoco_data.qvel[qveladr : qveladr + 6] = [
            0.4,
            -0.5,
            0.6,
            -3.0,
            -2.0,
            -1.0,
        ]
        mujoco.mj_forward(simulator.mujoco_model, simulator.mujoco_data)
        simulator._update_hitter_ball_state()

        self.assertTrue(simulator.restore_hitter_ball_launch_state())
        np.testing.assert_array_equal(simulator.mujoco_data.qpos[qposadr : qposadr + 7], expected_qpos)
        np.testing.assert_array_equal(simulator.mujoco_data.qvel[qveladr : qveladr + 6], expected_qvel)
        np.testing.assert_array_equal(simulator.mujoco_data.xpos[body_id], expected_xpos)
        np.testing.assert_array_equal(simulator.mujoco_data.xquat[body_id], expected_xquat_wxyz)
        np.testing.assert_array_equal(simulator.ball_pos_world, expected_xpos.astype(np.float32))
        np.testing.assert_array_equal(
            simulator.ball_quat_world,
            expected_xquat_wxyz[[1, 2, 3, 0]].astype(np.float32),
        )
        np.testing.assert_array_equal(simulator.ball_vel_world, expected_qvel[:3].astype(np.float32))
        np.testing.assert_array_equal(simulator.ball_ang_vel_world, expected_qvel[3:].astype(np.float32))


class AnalyticOverrideRemovalTests(unittest.TestCase):
    def test_mujoco_source_contains_no_analytic_ball_state_override(self):
        source = (DEPLOY_DIR / "simulator" / "mujoco.py").read_text()
        forbidden = (
            "use_analytic_table_bounce",
            "use_analytic_racket_hit",
            "analytic_racket_hit_armed",
            "set_hitter_analytic_racket_hit",
            "_apply_hitter_ball_analytic_table_bounce",
            "_apply_hitter_ball_analytic_racket_hit",
        )
        for token in forbidden:
            self.assertNotIn(token, source, token)

    def test_hitter_config_contains_no_analytic_ball_override_options(self):
        config = yaml.safe_load((DEPLOY_DIR / "config" / "hitter.yaml").read_text())
        table_tennis = config["sim"]["config"]["table_tennis"]
        for key in (
            "use_analytic_table_bounce",
            "use_analytic_racket_hit",
            "ball_geom_name",
            "racket_face_geom_name",
        ):
            self.assertNotIn(key, table_tennis)

    def test_copying_planner_command_does_not_call_analytic_hit_setter(self):
        env = HitterEnv.__new__(HitterEnv)
        env.simulator = RecordingSimulator()
        env.hitter_command_lifecycle = SimpleNamespace(recovery_duration_s=0.5)
        fields = {
            "strike_type_index": 0,
            "time_to_strike": 0.85,
            "base_target": np.array([-0.4, 0.0], dtype=np.float32),
            "base_height": np.float32(0.793),
            "racket_target": np.array([0.0, 0.1, 1.0], dtype=np.float32),
            "racket_velocity": np.array([2.8, 0.0, 0.5], dtype=np.float32),
        }

        env._copy_hitter_command(SimpleNamespace(), validated=fields)

        self.assertEqual(env.simulator.analytic_hit_calls, 0)


class PurePhysicalContactTests(unittest.TestCase):
    def test_xml_enables_still_standard_air(self):
        root = ET.parse(XML_PATH).getroot()
        option = root.find("./option")
        self.assertIsNotNone(option)
        self.assertEqual(option.attrib.get("gravity"), "0 0 -9.81")
        self.assertEqual(option.attrib.get("wind"), "0 0 0")
        self.assertEqual(option.attrib.get("density"), "1.225")
        self.assertEqual(option.attrib.get("viscosity"), "0.000018")

        model = mujoco.MjModel.from_xml_path(str(XML_PATH))
        np.testing.assert_allclose(model.opt.gravity, [0.0, 0.0, -9.81])
        np.testing.assert_allclose(model.opt.wind, [0.0, 0.0, 0.0])
        self.assertAlmostEqual(float(model.opt.density), 1.225)
        self.assertAlmostEqual(float(model.opt.viscosity), 0.000018)

    def test_standard_air_slows_a_free_flying_ball(self):
        final_speeds = []
        for air_enabled in (True, False):
            model = mujoco.MjModel.from_xml_path(str(XML_PATH))
            model.opt.timestep = 0.005
            model.opt.gravity[:] = 0.0
            if not air_enabled:
                model.opt.density = 0.0
                model.opt.viscosity = 0.0
            data = mujoco.MjData(model)
            ball_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "hitter_ball_freejoint")
            qposadr = int(model.jnt_qposadr[ball_joint])
            qveladr = int(model.jnt_dofadr[ball_joint])
            data.qpos[qposadr : qposadr + 3] = [10.0, 0.0, 10.0]
            data.qvel[qveladr : qveladr + 3] = [4.0, 0.0, 0.0]
            mujoco.mj_forward(model, data)

            for _ in range(200):
                mujoco.mj_step(model, data)

            self.assertTrue(np.isfinite(data.qpos).all())
            self.assertTrue(np.isfinite(data.qvel).all())
            final_speeds.append(float(np.linalg.norm(data.qvel[qveladr : qveladr + 3])))

        speed_with_air, speed_without_air = final_speeds
        self.assertLess(speed_with_air, speed_without_air - 0.5)

    def test_xml_defines_only_intended_ball_contact_pairs(self):
        root = ET.parse(XML_PATH).getroot()
        pair_elements = root.findall("./contact/pair")
        self.assertEqual(len(pair_elements), 3)
        pairs = {frozenset((pair.attrib["geom1"], pair.attrib["geom2"])) for pair in pair_elements}
        expected_pairs = {
            frozenset(("right_racket_face_collision", "hitter_ball_geom")),
            frozenset(("hitter_table_top", "hitter_ball_geom")),
            frozenset(("hitter_table_net", "hitter_ball_geom")),
        }
        self.assertEqual(pairs, expected_pairs)
        expected_attributes = {
            frozenset(("hitter_table_top", "hitter_ball_geom")): {
                "condim": "6",
                "friction": "0.20 0.005 0.0001",
                "solref": "0.04 0.10",
                "solimp": "0.95 0.99 0.001",
            },
            frozenset(("hitter_table_net", "hitter_ball_geom")): {
                "condim": "6",
                "friction": "0.20 0.005 0.0001",
                "solref": "0.004 0.35",
                "solimp": "0.95 0.99 0.001",
            },
            frozenset(("right_racket_face_collision", "hitter_ball_geom")): {
                "condim": "6",
                "friction": "0.20 0.005 0.0001",
                "solref": "0.004 0.35",
                "solimp": "0.95 0.99 0.001",
            },
        }
        for pair in pair_elements:
            pair_key = frozenset((pair.attrib["geom1"], pair.attrib["geom2"]))
            for attribute, expected_value in expected_attributes[pair_key].items():
                self.assertEqual(pair.attrib.get(attribute), expected_value)
        self.assertEqual(root.findall("./contact/exclude"), [])
        center_line = root.find(".//geom[@name='hitter_table_center_line']")
        self.assertIsNotNone(center_line)
        self.assertEqual(center_line.attrib.get("contype"), "0")
        self.assertEqual(center_line.attrib.get("conaffinity"), "0")
        ball = root.find(".//geom[@name='hitter_ball_geom']")
        self.assertIsNotNone(ball)
        self.assertEqual(ball.attrib.get("contype"), "0")
        self.assertEqual(ball.attrib.get("conaffinity"), "0")

    def test_falling_ball_contacts_table_and_rebounds(self):
        model = mujoco.MjModel.from_xml_path(str(XML_PATH))
        model.opt.timestep = 0.005
        data = mujoco.MjData(model)
        ball_geom = _geom_id(model, "hitter_ball_geom")
        table_geom = _geom_id(model, "hitter_table_top")
        ball_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "hitter_ball_freejoint")
        qposadr = int(model.jnt_qposadr[ball_joint])
        qveladr = int(model.jnt_dofadr[ball_joint])
        data.qpos[qposadr : qposadr + 3] = [1.0, 0.25, 0.84]
        data.qvel[qveladr : qveladr + 3] = [0.0, 0.0, -2.0]
        mujoco.mj_forward(model, data)

        contacted = False
        rebound_vz = -np.inf
        for _ in range(120):
            mujoco.mj_step(model, data)
            contacted = contacted or _has_contact(data, ball_geom, table_geom)
            if contacted:
                rebound_vz = max(rebound_vz, float(data.qvel[qveladr + 2]))
            self.assertTrue(np.isfinite(data.qpos).all())
            self.assertTrue(np.isfinite(data.qvel).all())

        self.assertTrue(contacted)
        self.assertGreater(rebound_vz, 0.0)

    def test_repeated_table_bounces_do_not_gain_vertical_speed(self):
        model = mujoco.MjModel.from_xml_path(str(XML_PATH))
        model.opt.timestep = 0.005
        data = mujoco.MjData(model)
        ball_geom = _geom_id(model, "hitter_ball_geom")
        table_geom = _geom_id(model, "hitter_table_top")
        ball_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "hitter_ball_freejoint")
        qposadr = int(model.jnt_qposadr[ball_joint])
        qveladr = int(model.jnt_dofadr[ball_joint])
        data.qpos[qposadr : qposadr + 3] = [1.0, 0.25, 0.84]
        data.qvel[qveladr : qveladr + 3] = [0.0, 0.0, -2.0]
        mujoco.mj_forward(model, data)

        episodes = []
        incoming_speed = None
        was_in_contact = _has_contact(data, ball_geom, table_geom)
        for _ in range(2000):
            previous_vz = float(data.qvel[qveladr + 2])
            mujoco.mj_step(model, data)
            in_contact = _has_contact(data, ball_geom, table_geom)
            if in_contact and not was_in_contact:
                self.assertLess(previous_vz, 0.0)
                incoming_speed = -previous_vz
            elif was_in_contact and not in_contact:
                outgoing_speed = float(data.qvel[qveladr + 2])
                self.assertIsNotNone(incoming_speed)
                self.assertGreater(outgoing_speed, 0.0)
                episodes.append((incoming_speed, outgoing_speed))
                incoming_speed = None
                if len(episodes) == 3:
                    break
            was_in_contact = in_contact

        self.assertEqual(len(episodes), 3)
        for episode, (incoming_speed, outgoing_speed) in enumerate(episodes, 1):
            self.assertLessEqual(
                outgoing_speed,
                incoming_speed + 1e-6,
                f"episode {episode}: incoming={incoming_speed}, " f"outgoing={outgoing_speed}",
            )

    def test_racket_contact_is_physical_across_operating_speeds(self):
        for speed in (2.0, 4.0, 6.0):
            with self.subTest(speed=speed):
                model = mujoco.MjModel.from_xml_path(str(XML_PATH))
                model.opt.timestep = 0.005
                model.opt.gravity[:] = 0.0
                data = mujoco.MjData(model)
                mujoco.mj_forward(model, data)
                ball_geom = _geom_id(model, "hitter_ball_geom")
                racket_geom = _geom_id(model, "right_racket_face_collision")
                ball_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "hitter_ball_freejoint")
                qposadr = int(model.jnt_qposadr[ball_joint])
                qveladr = int(model.jnt_dofadr[ball_joint])
                rotation = data.geom_xmat[racket_geom].reshape(3, 3)
                normal = rotation @ np.array([0.0, -1.0, 0.0])
                normal /= np.linalg.norm(normal)
                data.qpos[qposadr : qposadr + 3] = data.geom_xpos[racket_geom] + 0.15 * normal
                data.qvel[qveladr : qveladr + 3] = -speed * normal
                mujoco.mj_forward(model, data)

                contacted = False
                outgoing_normal_speed = -np.inf
                for _ in range(80):
                    mujoco.mj_step(model, data)
                    contacted = contacted or _has_contact(data, ball_geom, racket_geom)
                    if contacted:
                        outgoing_normal_speed = max(
                            outgoing_normal_speed,
                            float(np.dot(data.qvel[qveladr : qveladr + 3], normal)),
                        )

                self.assertTrue(contacted)
                self.assertGreater(outgoing_normal_speed, 0.0)


class RandomizedBallResetTests(unittest.TestCase):
    @staticmethod
    def _trajectory_config(*, selection: str, seed: int = 17) -> dict:
        return {
            "enabled": True,
            "randomize_ball_on_reset": True,
            "ball_random_seed": seed,
            "ball_initial_pos": [1.55, -0.3, 1.25],
            "ball_initial_lin_vel": [-1.1, 0.0, 0.1],
            "ball_initial_ang_vel": [0.0, 0.0, 0.0],
            "ball_trajectory_selection": selection,
            "ball_trajectory_candidates": [
                {
                    "name": "left",
                    "pos": [1.81, -0.22, 1.04],
                    "lin_vel": [-2.1, 0.15, 0.25],
                    "ang_vel": [1.0, 2.0, 3.0],
                },
                {
                    "name": "center",
                    "pos": [1.92, 0.02, 1.12],
                    "lin_vel": [-2.4, -0.05, 0.35],
                    "ang_vel": [4.0, 5.0, 6.0],
                },
                {
                    "name": "right",
                    "pos": [2.03, 0.24, 1.20],
                    "lin_vel": [-2.7, -0.2, 0.45],
                    "ang_vel": [7.0, 8.0, 9.0],
                },
            ],
        }

    def test_enabled_config_enables_randomized_ball_resets(self):
        simulator = _minimal_table_tennis_simulator(self._trajectory_config(selection="sequence"))

        self.assertTrue(simulator.table_tennis_enabled)
        self.assertTrue(simulator.randomize_hitter_ball)

    def test_sequential_trajectory_resets_choose_different_candidates(self):
        config = self._trajectory_config(selection="sequence")
        simulator = _minimal_table_tennis_simulator(config)
        qposadr = simulator.hitter_ball_qposadr

        simulator.reset_hitter_ball()
        first_position = simulator.mujoco_data.qpos[qposadr : qposadr + 3].copy()
        simulator.reset_hitter_ball()
        second_position = simulator.mujoco_data.qpos[qposadr : qposadr + 3].copy()

        np.testing.assert_allclose(first_position, config["ball_trajectory_candidates"][0]["pos"])
        np.testing.assert_allclose(second_position, config["ball_trajectory_candidates"][1]["pos"])
        self.assertFalse(np.array_equal(first_position, second_position))

    def test_random_trajectory_selection_honors_fixed_seed(self):
        seed = 17
        config = self._trajectory_config(selection="random", seed=seed)
        candidate_count = len(config["ball_trajectory_candidates"])
        expected_indices = np.random.default_rng(seed).integers(candidate_count, size=12)
        self.assertGreater(len(set(expected_indices)), 1)
        actual_position_sequences = []

        for simulator in (
            _minimal_table_tennis_simulator(config),
            _minimal_table_tennis_simulator(config),
        ):
            qposadr = simulator.hitter_ball_qposadr
            qveladr = simulator.hitter_ball_qveladr
            positions = []
            for expected_index in expected_indices:
                expected = config["ball_trajectory_candidates"][expected_index]
                simulator.reset_hitter_ball()
                actual_position = simulator.mujoco_data.qpos[qposadr : qposadr + 3].copy()
                positions.append(actual_position)
                np.testing.assert_allclose(actual_position, expected["pos"])
                np.testing.assert_allclose(
                    simulator.mujoco_data.qvel[qveladr : qveladr + 3],
                    expected["lin_vel"],
                )
                np.testing.assert_allclose(
                    simulator.mujoco_data.qvel[qveladr + 3 : qveladr + 6],
                    expected["ang_vel"],
                )
            actual_position_sequences.append(np.stack(positions))

        np.testing.assert_allclose(actual_position_sequences[0], actual_position_sequences[1])


class TwoServeScheduleTests(unittest.TestCase):
    def test_hitter_config_defines_two_mujoco_serves(self):
        config = yaml.safe_load((DEPLOY_DIR / "config" / "mimic" / "hitter.yaml").read_text())
        self.assertEqual(config["motion"]["mujoco_serve_times_s"], [0.0, 20.0])

    def test_schedule_serves_at_zero_and_twenty_seconds_only(self):
        simulator = _minimal_table_tennis_simulator({"enabled": True})
        simulator.is_real = False
        env = _scheduled_serve_env(simulator)

        simulator.mujoco_data.time = 3.0
        self.assertTrue(env._update_mujoco_serve_schedule())
        qposadr = simulator.hitter_ball_qposadr
        qveladr = simulator.hitter_ball_qveladr
        first_qpos = simulator.mujoco_data.qpos[qposadr : qposadr + 7].copy()
        first_qvel = simulator.mujoco_data.qvel[qveladr : qveladr + 6].copy()

        simulator.mujoco_data.qpos[qposadr : qposadr + 3] = [0.1, 0.2, 0.3]
        simulator.mujoco_data.qvel[qveladr : qveladr + 6] = 0.0
        simulator.mujoco_data.time = 22.999
        self.assertFalse(env._update_mujoco_serve_schedule())
        self.assertEqual(env._mujoco_next_serve_index, 1)
        np.testing.assert_array_equal(simulator.mujoco_data.qpos[qposadr : qposadr + 3], [0.1, 0.2, 0.3])
        np.testing.assert_array_equal(simulator.mujoco_data.qvel[qveladr : qveladr + 6], np.zeros(6))

        simulator.mujoco_data.time = 23.0
        self.assertTrue(env._update_mujoco_serve_schedule())
        np.testing.assert_array_equal(simulator.mujoco_data.qpos[qposadr : qposadr + 7], first_qpos)
        np.testing.assert_array_equal(simulator.mujoco_data.qvel[qveladr : qveladr + 6], first_qvel)
        self.assertEqual(env._hitter_sync_track_epoch, 2)
        self.assertEqual(env._hitter_sync_generation, 0)

        simulator.mujoco_data.qpos[qposadr : qposadr + 3] = [0.4, 0.5, 0.6]
        simulator.mujoco_data.qvel[qveladr : qveladr + 6] = 1.0
        simulator.mujoco_data.time = 43.0
        self.assertFalse(env._update_mujoco_serve_schedule())
        self.assertEqual(env._mujoco_next_serve_index, 2)
        np.testing.assert_array_equal(simulator.mujoco_data.qpos[qposadr : qposadr + 3], [0.4, 0.5, 0.6])
        np.testing.assert_array_equal(simulator.mujoco_data.qvel[qveladr : qveladr + 6], np.ones(6))

    def test_public_reset_serves_before_the_first_physics_step(self):
        simulator = _minimal_table_tennis_simulator({"enabled": True})
        simulator.is_real = False
        qposadr = simulator.hitter_ball_qposadr
        qveladr = simulator.hitter_ball_qveladr
        simulator.num_dof = qposadr - 7
        simulator.default_angles = simulator.default_qpos[7:qposadr].copy()
        simulator.cfg.control = SimpleNamespace(
            update_with_fk=False,
            use_residual=False,
        )
        simulator.active_dof_idx = np.array([], dtype=np.int32)
        simulator.foot_body_ids = np.array([], dtype=np.int32)
        simulator.teleop_quat_tmp = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        simulator.mujoco_data.time = 0.0
        env = _scheduled_serve_env(simulator)
        env.policy_model_meta = object()
        env._policy_dim = 0
        env.waiting_time_to_strike_s = 0.92
        env.hitter_rng = np.random.default_rng(0)
        env.save_video_enabled = False
        env.episode_length_buf = np.zeros(1, dtype=np.float64)
        env.action = np.zeros((1, 0), dtype=np.float32)
        env.history_handler = SimpleNamespace(reset=lambda: None)
        env.init_ref_dof_pos = None

        def compute_observation():
            env.obs_buf_dict = {"obs": np.zeros((1, 1), dtype=np.float32)}

        env.compute_observation = compute_observation

        observation = env.reset()

        time_after_reset = float(simulator.mujoco_data.time)
        index_after_reset = env._mujoco_next_serve_index
        qpos_after_reset = simulator.mujoco_data.qpos[qposadr : qposadr + 7].copy()
        qvel_after_reset = simulator.mujoco_data.qvel[qveladr : qveladr + 6].copy()
        launch_was_captured = bool(
            simulator._hitter_ball_launch_qpos is not None
            and simulator._hitter_ball_launch_qvel is not None
            and np.array_equal(
                simulator._hitter_ball_launch_qpos,
                qpos_after_reset,
            )
            and np.array_equal(
                simulator._hitter_ball_launch_qvel,
                qvel_after_reset,
            )
        )

        disturbed_qpos = np.array([0.7, -0.1, 1.4, 0.5, 0.5, 0.5, 0.5], dtype=np.float64)
        disturbed_qvel = np.array([-1.8, 0.3, -0.2, 3.0, -2.0, 1.0], dtype=np.float64)
        simulator.mujoco_data.qpos[qposadr : qposadr + 7] = disturbed_qpos
        simulator.mujoco_data.qvel[qveladr : qveladr + 6] = disturbed_qvel
        simulator.mujoco_data.time = 0.02
        served_at_0_02 = env._update_mujoco_serve_schedule()

        observed = {
            "sim_time_after_reset": time_after_reset,
            "next_index_after_reset": index_after_reset,
            "launch_was_captured": launch_was_captured,
            "served_at_0_02": served_at_0_02,
            "next_index_after_0_02": env._mujoco_next_serve_index,
            "qpos_preserved_at_0_02": np.array_equal(
                simulator.mujoco_data.qpos[qposadr : qposadr + 7],
                disturbed_qpos,
            ),
            "qvel_preserved_at_0_02": np.array_equal(
                simulator.mujoco_data.qvel[qveladr : qveladr + 6],
                disturbed_qvel,
            ),
        }
        self.assertEqual(observation["obs"].shape, (1, 1))
        self.assertEqual(
            observed,
            {
                "sim_time_after_reset": 0.0,
                "next_index_after_reset": 1,
                "launch_was_captured": True,
                "served_at_0_02": False,
                "next_index_after_0_02": 1,
                "qpos_preserved_at_0_02": True,
                "qvel_preserved_at_0_02": True,
            },
        )

    def test_command_reset_flag_cannot_create_an_unscheduled_serve(self):
        simulator = _minimal_table_tennis_simulator({"enabled": True})
        simulator.is_real = False
        env = _scheduled_serve_env(simulator)
        simulator.mujoco_data.time = 0.0
        self.assertTrue(env._update_mujoco_serve_schedule())

        env.hitter_ball_sequence_needs_reset = True
        simulator.mujoco_data.time = 5.0
        self.assertFalse(env._update_mujoco_serve_schedule())
        self.assertFalse(env.hitter_ball_sequence_needs_reset)
        self.assertEqual(env._mujoco_next_serve_index, 1)

    def test_real_backend_does_not_enable_mujoco_schedule(self):
        simulator = SimpleNamespace(is_real=True)
        env = _scheduled_serve_env(simulator)
        self.assertFalse(env._mujoco_serve_schedule_enabled())
        self.assertFalse(simulator.preserve_hitter_ball_state_on_calibrate)

    def test_reinitializing_schedule_preserves_completed_serve_quota(self):
        simulator = _minimal_table_tennis_simulator({"enabled": True})
        simulator.is_real = False
        env = _scheduled_serve_env(simulator)

        simulator.mujoco_data.time = 0.0
        self.assertTrue(env._update_mujoco_serve_schedule())
        simulator.mujoco_data.time = 20.0
        self.assertTrue(env._update_mujoco_serve_schedule())
        original_origin_time_s = env._mujoco_serve_origin_time_s

        env._init_mujoco_serve_schedule_state()
        simulator.mujoco_data.time = 40.0

        self.assertFalse(env._update_mujoco_serve_schedule())
        self.assertEqual(env._mujoco_next_serve_index, 2)
        self.assertEqual(env._mujoco_serve_origin_time_s, original_origin_time_s)
        self.assertEqual(env._hitter_sync_track_epoch, 2)

    def test_schedule_rejects_a_third_event(self):
        env = HitterEnv.__new__(HitterEnv)
        env.simulator = SimpleNamespace(is_real=False)
        env.motion_cfg = {"mujoco_serve_times_s": [0.0, 20.0, 40.0]}

        with self.assertRaisesRegex(
            ValueError,
            r"mujoco_serve_times_s must be exactly \[0\.0, 20\.0\]",
        ):
            env._init_mujoco_serve_schedule_state()

    def test_calibrate_preserves_current_ball_during_scheduled_run(self):
        simulator = _minimal_table_tennis_simulator({"enabled": True})
        simulator.is_real = False
        _scheduled_serve_env(simulator)
        qposadr = simulator.hitter_ball_qposadr
        qveladr = simulator.hitter_ball_qveladr
        simulator.num_dof = qposadr - 7
        simulator.default_angles = simulator.default_qpos[7:qposadr].copy()
        simulator.cfg.control = SimpleNamespace(use_residual=False)
        simulator.teleop_quat_tmp = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        flying_qpos = np.array([0.7, -0.1, 1.4, 0.5, 0.5, 0.5, 0.5], dtype=np.float64)
        flying_qvel = np.array([-1.8, 0.3, -0.2, 3.0, -2.0, 1.0], dtype=np.float64)
        simulator.mujoco_data.qpos[7] = 0.75
        simulator.mujoco_data.qpos[qposadr : qposadr + 7] = flying_qpos
        simulator.mujoco_data.qvel[qveladr : qveladr + 6] = flying_qvel
        mujoco.mj_forward(simulator.mujoco_model, simulator.mujoco_data)

        simulator.calibrate(refresh=True)

        np.testing.assert_array_equal(simulator.mujoco_data.qpos[qposadr : qposadr + 7], flying_qpos)
        np.testing.assert_array_equal(simulator.mujoco_data.qvel[qveladr : qveladr + 6], flying_qvel)
        self.assertEqual(simulator.mujoco_data.qpos[7], simulator.default_qpos[7])

    def test_missing_schedule_uses_legacy_lifecycle_reset(self):
        simulator = _minimal_table_tennis_simulator({"enabled": True})
        simulator.is_real = False
        simulator.cfg.control = SimpleNamespace(update_with_fk=False)
        simulator.active_dof_idx = np.array([], dtype=np.int32)
        simulator.foot_body_ids = np.array([], dtype=np.int32)
        env = HitterEnv.__new__(HitterEnv)
        env.simulator = simulator
        env.motion_cfg = {}
        env.reset_ball_on_command = True
        env.hitter_ball_sequence_needs_reset = True
        env._hitter_sync_track_epoch = 4
        env._hitter_sync_generation = 7
        env._init_mujoco_serve_schedule_state()
        qposadr = simulator.hitter_ball_qposadr
        expected_default_position = simulator.default_qpos[qposadr : qposadr + 3].copy()
        simulator.mujoco_data.qpos[qposadr : qposadr + 3] = [0.1, 0.2, 0.3]

        result = env._mujoco_planner_result(now=1000.0)

        self.assertEqual(result.track_epoch, 5)
        self.assertEqual(result.source_generation, 1)
        self.assertFalse(env.hitter_ball_sequence_needs_reset)
        np.testing.assert_array_equal(
            simulator.mujoco_data.qpos[qposadr : qposadr + 3],
            expected_default_position,
        )

    def test_mujoco_planner_result_runs_the_serve_schedule(self):
        simulator = _minimal_table_tennis_simulator({"enabled": True})
        simulator.is_real = False
        simulator.cfg.control = SimpleNamespace(update_with_fk=False)
        simulator.active_dof_idx = np.array([], dtype=np.int32)
        simulator.foot_body_ids = np.array([], dtype=np.int32)
        env = _scheduled_serve_env(simulator)

        simulator.mujoco_data.time = 2.0
        first_result = env._mujoco_planner_result(now=1000.0)
        self.assertEqual(env._mujoco_next_serve_index, 1)
        self.assertEqual(first_result.track_epoch, 1)

        qposadr = simulator.hitter_ball_qposadr
        simulator.mujoco_data.qpos[qposadr : qposadr + 3] = [0.1, 0.2, 0.3]
        env.hitter_ball_sequence_needs_reset = True
        simulator.mujoco_data.time = 21.999
        pending_result = env._mujoco_planner_result(now=2000.0)
        self.assertEqual(env._mujoco_next_serve_index, 1)
        self.assertEqual(pending_result.track_epoch, 1)
        self.assertFalse(env.hitter_ball_sequence_needs_reset)
        np.testing.assert_array_equal(simulator.mujoco_data.qpos[qposadr : qposadr + 3], [0.1, 0.2, 0.3])

        simulator.mujoco_data.time = 22.0
        second_result = env._mujoco_planner_result(now=3000.0)
        self.assertEqual(env._mujoco_next_serve_index, 2)
        self.assertEqual(second_result.track_epoch, 2)
