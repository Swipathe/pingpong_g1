import tempfile
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from deploy.mocap_bridge import chingmu_table_lcm_bridge
from deploy.mocap_bridge.chingmu_sdk_client import MocapFrame
from deploy.mocap_bridge.chingmu_table_lcm_bridge import (
    BallTracker,
    BridgeConfig,
    ChingMuTableLcmBridge,
    PelvisOrientationCalibration,
    build_arg_parser,
    calibrate_from_frames,
    emit_messages,
    infer_table_frame,
    load_table_frame,
    make_message,
    raw_to_table_world,
    save_table_frame,
)
from unitree_sdk2.lcm_types.transformation_t import transformation_t


TABLE_CORNERS_MM = np.array(
    [
        [-1362.572, 751.784, 5.331],
        [-1359.357, -760.017, 8.019],
        [1369.078, 756.992, 1.482],
        [1370.427, -756.099, -7.670],
    ],
    dtype=np.float64,
)
ROBOT_RAW_MM = np.array([1799.5, -18.8, 10.8], dtype=np.float64)
EXTRA_MARKER_MM = np.array([-1738.5, -6.8, 1013.8], dtype=np.float64)
CALIBRATION_PATH = (
    Path(__file__).resolve().parents[1]
    / "calibrations"
    / "chingmu_table_frame_latest.json"
)


def reference_body_markers(origin=ROBOT_RAW_MM):
    offsets = np.array(
        [
            [-60.0, -40.0, 20.0],
            [55.0, -35.0, -15.0],
            [-45.0, 50.0, -10.0],
            [50.0, 45.0, 25.0],
        ]
    )
    return {
        8100 + index: np.asarray(origin, dtype=np.float64) + offset
        for index, offset in enumerate(offsets)
    }


def table_world_to_raw(position_world, table, config):
    position = np.asarray(position_world, dtype=np.float64).reshape(3)
    table_coordinates_mm = np.array(
        [
            (position[0] - 0.5 * config.table_length_m) * 1000.0,
            position[1] * 1000.0,
            (position[2] - config.table_height_m) * 1000.0,
        ]
    )
    return (
        table.center_raw_mm
        + table_coordinates_mm[0] * table.x_axis_raw
        + table_coordinates_mm[1] * table.y_axis_raw
        + table_coordinates_mm[2] * table.z_axis_raw
    )


class ChingMuTableGeometryTest(unittest.TestCase):
    def setUp(self):
        self.config = BridgeConfig()
        self.table = infer_table_frame(
            np.vstack([EXTRA_MARKER_MM, TABLE_CORNERS_MM]),
            ROBOT_RAW_MM,
            self.config,
        )

    def test_effective_dimensions_match_the_four_selected_corners(self):
        centered = TABLE_CORNERS_MM - TABLE_CORNERS_MM.mean(axis=0)
        _left, singular_values_mm, _right_t = np.linalg.svd(
            centered, full_matrices=False
        )

        np.testing.assert_allclose(
            singular_values_mm[:2] * 0.001,
            [self.config.table_length_m, self.config.table_width_m],
            atol=1.0e-6,
        )
        self.assertEqual(self.config.table_length_m, 2.730738)
        self.assertEqual(self.config.table_width_m, 1.512451)
        self.assertEqual(self.config.table_height_m, 0.760000)

    def test_table_frame_uses_3d_svd_and_is_right_handed(self):
        selected = self.table.corners_raw_mm
        center = selected.mean(axis=0)
        _left, _singular_values, right_t = np.linalg.svd(
            selected - center, full_matrices=False
        )
        expected_normal = right_t[-1]
        if expected_normal[2] < 0.0:
            expected_normal = -expected_normal

        np.testing.assert_allclose(self.table.center_raw_mm, center, atol=1.0e-12)
        np.testing.assert_allclose(
            self.table.z_axis_raw, expected_normal, atol=1.0e-12
        )
        np.testing.assert_allclose(
            np.cross(self.table.x_axis_raw, self.table.y_axis_raw),
            self.table.z_axis_raw,
            atol=1.0e-12,
        )
        np.testing.assert_allclose(
            np.vstack(
                [
                    self.table.x_axis_raw,
                    self.table.y_axis_raw,
                    self.table.z_axis_raw,
                ]
            )
            @ np.vstack(
                [
                    self.table.x_axis_raw,
                    self.table.y_axis_raw,
                    self.table.z_axis_raw,
                ]
            ).T,
            np.eye(3),
            atol=1.0e-12,
        )
        self.assertLess(
            float(
                np.dot(
                    ROBOT_RAW_MM - self.table.center_raw_mm,
                    self.table.x_axis_raw,
                )
            ),
            0.0,
        )

    def test_maps_table_to_the_robotbridge_world_convention(self):
        corners_world = raw_to_table_world(
            TABLE_CORNERS_MM, self.table, self.config
        )

        np.testing.assert_allclose(
            raw_to_table_world(
                self.table.center_raw_mm, self.table, self.config
            ),
            [1.365369, 0.0, 0.760000],
            atol=1.0e-12,
        )
        self.assertAlmostEqual(float(corners_world[:, 0].min()), 0.0, delta=0.001)
        self.assertAlmostEqual(
            float(corners_world[:, 0].max()), 2.730738, delta=0.001
        )
        self.assertAlmostEqual(
            float(corners_world[:, 1].min()), -0.7562255, delta=0.001
        )
        self.assertAlmostEqual(
            float(corners_world[:, 1].max()), 0.7562255, delta=0.001
        )
        self.assertAlmostEqual(float(corners_world[:, 2].mean()), 0.760000)

    def test_body_pose_uses_table_rotation_and_sdk_xyzw(self):
        quaternion = Rotation.from_euler("xyz", [0.1, -0.2, 0.3]).as_quat()
        sdk_quaternion = -2.0 * quaternion

        position_world, rotation_world = (
            chingmu_table_lcm_bridge.body_pose_to_table_world(
                ROBOT_RAW_MM,
                sdk_quaternion,
                self.table,
                self.config,
            )
        )

        np.testing.assert_allclose(
            position_world,
            raw_to_table_world(ROBOT_RAW_MM, self.table, self.config),
            atol=1.0e-12,
        )
        np.testing.assert_allclose(
            rotation_world,
            np.vstack(
                [
                    self.table.x_axis_raw,
                    self.table.y_axis_raw,
                    self.table.z_axis_raw,
                ]
            )
            @ Rotation.from_quat(quaternion).as_matrix(),
            atol=1.0e-12,
        )

    def test_excludes_table_corners_and_outside_marker_from_ball(self):
        ball_raw = table_world_to_raw(
            [0.5 * self.config.table_length_m, 0.0, 0.96],
            self.table,
            self.config,
        )
        tracker = BallTracker()

        update = tracker.update(
            np.vstack([TABLE_CORNERS_MM, EXTRA_MARKER_MM, ball_raw]),
            frame_number=100,
            table=self.table,
            config=self.config,
        )

        self.assertFalse(update.ended)
        np.testing.assert_allclose(
            update.position_world,
            [0.5 * self.config.table_length_m, 0.0, 0.96],
            atol=1.0e-9,
        )

    def test_message_keeps_lcm_schema_units_xyzw_and_sdk_timestamp(self):
        message = make_message(
            "G1Pelvis",
            [1.0, 2.0, 3.0],
            [0.0, 0.0, 0.0, 1.0],
            frame_number=720,
            source_time_s=123.456,
            valid=True,
            publish_time_us=123456,
        )

        decoded = transformation_t.decode(message.encode())
        self.assertEqual(decoded.name, "G1Pelvis")
        self.assertEqual(decoded.vicon_frame_number, 720)
        self.assertEqual(decoded.vicon_time_s, 123.456)
        self.assertEqual(decoded.publish_time_us, 123456)
        self.assertEqual((decoded.valid, decoded.occluded), (1, 0))
        self.assertEqual(decoded.pos_vicon, (1.0, 2.0, 3.0))
        self.assertEqual(decoded.quat_vicon, (0.0, 0.0, 0.0, 1.0))

    def test_calibration_uses_true_root_for_table_axis_sign(self):
        misleading_body_markers = reference_body_markers(
            np.array([-1800.0, 0.0, 800.0])
        )
        frames = [
            MocapFrame(
                frame_number=100 + index,
                source_time_s=10.0 + index / 360.0,
                body_position_mm=ROBOT_RAW_MM.copy(),
                body_quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
                body_markers_mm={
                    sensor: position + np.array([0.01 * index, 0.0, 0.0])
                    for sensor, position in misleading_body_markers.items()
                },
                unlabeled_markers_mm=np.vstack(
                    [TABLE_CORNERS_MM, EXTRA_MARKER_MM]
                ),
            )
            for index in range(20)
        ]

        calibration = calibrate_from_frames(frames, self.config)

        self.assertLess(
            float(
                np.dot(
                    ROBOT_RAW_MM - calibration.table_frame.center_raw_mm,
                    calibration.table_frame.x_axis_raw,
                )
            ),
            0.0,
        )
        np.testing.assert_allclose(
            raw_to_table_world(
                calibration.table_frame.center_raw_mm,
                calibration.table_frame,
                self.config,
            ),
            [1.365369, 0.0, 0.760000],
            atol=1.0e-12,
        )

    def test_chingmu_table_calibration_json_round_trip(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "chingmu_table.json"
            save_table_frame(path, self.table, self.config)
            loaded = load_table_frame(path, self.config)

        np.testing.assert_allclose(loaded.center_raw_mm, self.table.center_raw_mm)
        np.testing.assert_allclose(loaded.x_axis_raw, self.table.x_axis_raw)
        np.testing.assert_allclose(loaded.y_axis_raw, self.table.y_axis_raw)
        np.testing.assert_allclose(loaded.z_axis_raw, self.table.z_axis_raw)
        np.testing.assert_allclose(loaded.corners_raw_mm, self.table.corners_raw_mm)

    def test_saved_calibration_is_refit_from_its_raw_corners_in_3d(self):
        loaded = load_table_frame(CALIBRATION_PATH, self.config)
        refitted = infer_table_frame(
            loaded.corners_raw_mm, ROBOT_RAW_MM, self.config
        )

        np.testing.assert_allclose(
            loaded.center_raw_mm, refitted.center_raw_mm, atol=1.0e-12
        )
        np.testing.assert_allclose(
            loaded.x_axis_raw, refitted.x_axis_raw, atol=1.0e-12
        )
        np.testing.assert_allclose(
            loaded.y_axis_raw, refitted.y_axis_raw, atol=1.0e-12
        )
        np.testing.assert_allclose(
            loaded.z_axis_raw, refitted.z_axis_raw, atol=1.0e-12
        )
        self.assertAlmostEqual(
            loaded.rectangle_score, refitted.rectangle_score, places=12
        )
        self.assertGreater(np.linalg.norm(loaded.z_axis_raw[:2]), 1.0e-4)

    def test_cli_defaults_use_accepted_effective_geometry(self):
        args = build_arg_parser().parse_args([])

        self.assertEqual(args.host, "192.168.2.100")
        self.assertEqual(args.base_subject, "G1Pelvis")
        self.assertIsNone(args.body_id)
        self.assertEqual(args.table_length, 2.730738)
        self.assertEqual(args.table_width, 1.512451)
        self.assertEqual(args.table_height, 0.760000)
        self.assertEqual(args.source_rate_hz, 360.0)
        self.assertFalse(args.publish)

    def test_cli_accepts_explicit_chingmu_body_pose_id(self):
        args = build_arg_parser().parse_args(["--body-id", "300"])

        self.assertEqual(args.body_id, 300)


class ChingMuBallTrackerTest(unittest.TestCase):
    def setUp(self):
        self.config = BridgeConfig()
        self.table = infer_table_frame(
            TABLE_CORNERS_MM, ROBOT_RAW_MM, self.config
        )
        self.tracker = BallTracker()

    def raw(self, position_world):
        return table_world_to_raw(
            position_world, self.table, self.config
        )

    def test_crossing_x_zero_ends_once_and_resets_track(self):
        active = self.tracker.update(
            [self.raw([0.01, 0.0, 0.90])],
            frame_number=400,
            table=self.table,
            config=self.config,
        )
        ended = self.tracker.update(
            [
                self.raw([0.60, 0.0, 0.90]),
                self.raw([-0.01, 0.0, 0.90]),
            ],
            frame_number=401,
            table=self.table,
            config=self.config,
        )
        next_frame = self.tracker.update(
            [],
            frame_number=402,
            table=self.table,
            config=self.config,
        )

        self.assertIsNotNone(active.position_world)
        self.assertFalse(active.ended)
        self.assertIsNone(ended.position_world)
        self.assertTrue(ended.ended)
        self.assertIsNone(next_frame.position_world)
        self.assertFalse(next_frame.ended)

    def test_current_frame_loss_ends_once_and_resets_track(self):
        self.tracker.update(
            [self.raw([0.50, 0.0, 0.90])],
            frame_number=500,
            table=self.table,
            config=self.config,
        )

        ended = self.tracker.update(
            [],
            frame_number=501,
            table=self.table,
            config=self.config,
        )
        next_frame = self.tracker.update(
            [],
            frame_number=502,
            table=self.table,
            config=self.config,
        )

        self.assertTrue(ended.ended)
        self.assertFalse(next_frame.ended)


class ChingMuTableBridgeTest(unittest.TestCase):
    def setUp(self):
        self.config = BridgeConfig()
        self.table = infer_table_frame(
            TABLE_CORNERS_MM, ROBOT_RAW_MM, self.config
        )
        self.reference = reference_body_markers()
        self.bridge = ChingMuTableLcmBridge(
            config=self.config,
            table_frame=self.table,
            pelvis_orientation_calibration=PelvisOrientationCalibration(
                rotation_rigid_from_pelvis=np.eye(3),
                sample_count=30,
                source_duration_s=1.0,
                position_rms_m=0.0,
                angular_rms_deg=0.0,
            ),
        )

    def make_frame(
        self,
        frame_number,
        unlabeled,
        *,
        source_time_s=None,
        body_position=ROBOT_RAW_MM,
        body_quat=(0.0, 0.0, 0.0, 1.0),
        body_markers=None,
    ):
        return MocapFrame(
            frame_number=frame_number,
            source_time_s=(
                100.0 + frame_number / 360.0
                if source_time_s is None
                else source_time_s
            ),
            body_position_mm=(
                None
                if body_position is None
                else np.asarray(body_position, dtype=np.float64).copy()
            ),
            body_quaternion_xyzw=(
                None
                if body_quat is None
                else np.asarray(body_quat, dtype=np.float64).copy()
            ),
            body_markers_mm=(
                self.reference if body_markers is None else body_markers
            ),
            unlabeled_markers_mm=np.asarray(
                unlabeled, dtype=np.float64
            ).reshape(-1, 3),
        )

    def raw(self, position_world):
        return table_world_to_raw(position_world, self.table, self.config)

    def expected_base_rotation(self, body_quat=(0.0, 0.0, 0.0, 1.0)):
        return Rotation.from_matrix(
            chingmu_table_lcm_bridge.raw_rotation_to_table_world(self.table)
            @ Rotation.from_quat(body_quat).as_matrix()
        )

    def assert_base_rotation_equal(self, quaternion, expected):
        actual = Rotation.from_quat(quaternion)
        self.assertLess(
            (actual.inv() * expected).magnitude(),
            1.0e-10,
        )

    def test_publishes_rigid_origin_in_absolute_base_rotation(self):
        expected_rotation = self.expected_base_rotation()

        messages = self.bridge.process_frame(
            self.make_frame(360, []), publish_time_us=1000
        )
        base = next(message for message in messages if message.name == "G1Pelvis")

        np.testing.assert_allclose(
            base.pos_vicon,
            raw_to_table_world(ROBOT_RAW_MM, self.table, self.config),
            atol=1.0e-9,
        )
        self.assert_base_rotation_equal(base.quat_vicon, expected_rotation)

    def test_rigid_origin_is_unchanged_by_absolute_base_rotation(self):
        self.bridge.process_frame(self.make_frame(360, []), publish_time_us=1000)

        quarter_turn = Rotation.from_euler("z", 0.5 * np.pi)
        expected_rotation = self.expected_base_rotation(
            quarter_turn.as_quat()
        )
        messages = self.bridge.process_frame(
            self.make_frame(
                361,
                [],
                body_quat=quarter_turn.as_quat(),
            ),
            publish_time_us=2000,
        )
        base = next(message for message in messages if message.name == "G1Pelvis")

        np.testing.assert_allclose(
            base.pos_vicon,
            raw_to_table_world(ROBOT_RAW_MM, self.table, self.config),
            atol=1.0e-9,
        )
        self.assert_base_rotation_equal(base.quat_vicon, expected_rotation)

    def test_publishes_only_canonical_objects_in_existing_order(self):
        ball_raw = self.raw(
            [0.5 * self.config.table_length_m, 0.0, 0.96]
        )
        expected_rotation = self.expected_base_rotation()
        frame = self.make_frame(
            360,
            np.vstack([TABLE_CORNERS_MM, ball_raw]),
            source_time_s=321.25,
        )

        messages = self.bridge.process_frame(frame, publish_time_us=1000)
        decoded = [
            transformation_t.decode(message.encode()) for message in messages
        ]

        self.assertEqual(
            [message.name for message in decoded],
            ["G1Pelvis", "ball", "table"],
        )
        np.testing.assert_allclose(
            decoded[0].pos_vicon,
            raw_to_table_world(ROBOT_RAW_MM, self.table, self.config),
            atol=1.0e-9,
        )
        self.assert_base_rotation_equal(
            decoded[0].quat_vicon,
            expected_rotation,
        )
        np.testing.assert_allclose(
            decoded[1].pos_vicon,
            [0.5 * self.config.table_length_m, 0.0, 0.96],
            atol=1.0e-9,
        )
        np.testing.assert_allclose(
            decoded[2].pos_vicon,
            [0.5 * self.config.table_length_m, 0.0, 0.760000],
            atol=1.0e-9,
        )
        self.assertTrue(
            all(message.vicon_time_s == 321.25 for message in decoded)
        )

    def test_root_position_is_not_marker_centroid(self):
        true_root = ROBOT_RAW_MM + np.array([50.0, -25.0, 700.0])
        expected_rotation = self.expected_base_rotation()
        frame = self.make_frame(
            363,
            TABLE_CORNERS_MM,
            body_position=true_root,
            body_markers=self.reference,
        )

        base = next(
            message
            for message in self.bridge.process_frame(frame)
            if message.name == "G1Pelvis"
        )

        np.testing.assert_allclose(
            base.pos_vicon,
            raw_to_table_world(true_root, self.table, self.config),
            atol=1.0e-9,
        )
        self.assert_base_rotation_equal(base.quat_vicon, expected_rotation)

    def test_first_and_later_root_rotations_are_absolute(self):
        first_body_rotation = Rotation.from_euler(
            "xyz",
            [0.10, -0.20, 0.40],
        )
        second_body_rotation = Rotation.from_euler(
            "xyz",
            [-0.05, 0.08, 0.55],
        )
        first = self.make_frame(
            360,
            TABLE_CORNERS_MM,
            body_position=ROBOT_RAW_MM,
            body_quat=first_body_rotation.as_quat(),
        )
        second = self.make_frame(
            361,
            TABLE_CORNERS_MM,
            body_position=ROBOT_RAW_MM,
            body_quat=second_body_rotation.as_quat(),
        )

        first_base = next(
            message
            for message in self.bridge.process_frame(first)
            if message.name == "G1Pelvis"
        )
        second_base = next(
            message
            for message in self.bridge.process_frame(second)
            if message.name == "G1Pelvis"
        )

        self.assert_base_rotation_equal(
            first_base.quat_vicon,
            self.expected_base_rotation(first_body_rotation.as_quat()),
        )
        self.assert_base_rotation_equal(
            second_base.quat_vicon,
            self.expected_base_rotation(second_body_rotation.as_quat()),
        )

    def test_missing_root_has_no_marker_centroid_fallback(self):
        frame = self.make_frame(
            362,
            TABLE_CORNERS_MM,
            body_position=None,
            body_quat=None,
            body_markers=self.reference,
        )

        names = [message.name for message in self.bridge.process_frame(frame)]

        self.assertNotIn("G1Pelvis", names)
        self.assertIn("table", names)

    def test_valid_root_loss_emits_one_invalid_then_recovers(self):
        first = self.bridge.process_frame(
            self.make_frame(
                380,
                TABLE_CORNERS_MM,
                source_time_s=50.0,
            )
        )
        lost = self.bridge.process_frame(
            self.make_frame(
                381,
                [self.raw([0.50, 0.0, 0.90])],
                source_time_s=50.25,
                body_position=None,
                body_quat=None,
                body_markers=self.reference,
            )
        )
        still_missing = self.bridge.process_frame(
            self.make_frame(
                382,
                [self.raw([0.49, 0.0, 0.90])],
                source_time_s=50.50,
                body_position=None,
                body_quat=None,
                body_markers=self.reference,
            )
        )
        restored = self.bridge.process_frame(
            self.make_frame(
                383,
                [self.raw([0.48, 0.0, 0.90])],
                source_time_s=50.75,
            )
        )

        first_base = [message for message in first if message.name == "G1Pelvis"]
        lost_base = [message for message in lost if message.name == "G1Pelvis"]
        repeated_base = [
            message for message in still_missing if message.name == "G1Pelvis"
        ]
        restored_base = [
            message for message in restored if message.name == "G1Pelvis"
        ]

        self.assertEqual(len(first_base), 1)
        self.assertEqual((first_base[0].valid, first_base[0].occluded), (1, 0))
        self.assertEqual(len(lost_base), 1)
        self.assertEqual((lost_base[0].valid, lost_base[0].occluded), (0, 1))
        self.assertEqual(lost_base[0].vicon_frame_number, 381)
        self.assertEqual(lost_base[0].vicon_time_s, 50.25)
        self.assertTrue(np.isfinite(lost_base[0].pos_vicon).all())
        self.assertTrue(np.isfinite(lost_base[0].quat_vicon).all())
        np.testing.assert_allclose(lost_base[0].pos_vicon, [0.0, 0.0, 0.0])
        np.testing.assert_allclose(
            lost_base[0].quat_vicon,
            [0.0, 0.0, 0.0, 1.0],
        )
        marker_centroid_world = raw_to_table_world(
            np.mean(np.vstack(list(self.reference.values())), axis=0),
            self.table,
            self.config,
        )
        self.assertFalse(
            np.allclose(lost_base[0].pos_vicon, marker_centroid_world)
        )
        self.assertEqual(repeated_base, [])
        self.assertEqual(len(restored_base), 1)
        self.assertEqual(
            (restored_base[0].valid, restored_base[0].occluded),
            (1, 0),
        )
        self.assertIn("ball", [message.name for message in lost])
        self.assertIn("table", [message.name for message in lost])
        self.assertIn("ball", [message.name for message in still_missing])
        self.assertIn("table", [message.name for message in still_missing])

    def test_invalid_root_quaternion_is_not_published(self):
        frame = self.make_frame(
            362,
            TABLE_CORNERS_MM,
            body_position=ROBOT_RAW_MM,
            body_quat=[0.0, 0.0, 0.0, 0.0],
        )

        names = [message.name for message in self.bridge.process_frame(frame)]

        self.assertNotIn("G1Pelvis", names)
        self.assertIn("table", names)

    def test_ball_stream_continues_when_root_is_missing(self):
        frame = self.make_frame(
            370,
            [self.raw([0.5, 0.0, 0.90])],
            body_position=None,
            body_quat=None,
        )

        names = [message.name for message in self.bridge.process_frame(frame)]

        self.assertEqual(names, ["ball", "table"])

    def test_crossing_x_zero_publishes_one_invalid_and_closes_track(self):
        inside_raw = self.raw([0.01, 0.0, 0.90])
        crossed_raw = self.raw([-0.01, 0.0, 0.90])
        self.bridge.process_frame(self.make_frame(400, [inside_raw]))

        ended = self.bridge.process_frame(
            self.make_frame(401, [crossed_raw], source_time_s=41.25)
        )
        next_frame = self.bridge.process_frame(self.make_frame(402, []))

        invalid = [message for message in ended if message.name == "ball"]
        self.assertEqual(len(invalid), 1)
        self.assertEqual((invalid[0].valid, invalid[0].occluded), (0, 1))
        self.assertEqual(invalid[0].vicon_time_s, 41.25)
        self.assertFalse(
            any(message.name == "ball" for message in next_frame)
        )

    def test_current_frame_ball_loss_publishes_one_invalid(self):
        self.bridge.process_frame(
            self.make_frame(410, [self.raw([0.50, 0.0, 0.90])])
        )

        ended = self.bridge.process_frame(
            self.make_frame(411, [], source_time_s=42.25)
        )
        next_frame = self.bridge.process_frame(self.make_frame(412, []))

        invalid = [message for message in ended if message.name == "ball"]
        self.assertEqual(len(invalid), 1)
        self.assertEqual((invalid[0].valid, invalid[0].occluded), (0, 1))
        self.assertEqual(invalid[0].vicon_time_s, 42.25)
        self.assertFalse(
            any(message.name == "ball" for message in next_frame)
        )

    def test_lcm_emission_requires_explicit_publish_and_keeps_channel(self):
        messages = self.bridge.process_frame(
            self.make_frame(12, TABLE_CORNERS_MM), publish_time_us=12
        )

        class FakeLcm:
            def __init__(self):
                self.calls = []

            def publish(self, channel, data):
                self.calls.append((channel, transformation_t.decode(data)))

        fake_lcm = FakeLcm()
        self.assertEqual(
            emit_messages(fake_lcm, messages, self.config, False), 0
        )
        self.assertEqual(fake_lcm.calls, [])

        self.assertEqual(
            emit_messages(fake_lcm, messages, self.config, True), 2
        )
        self.assertEqual(
            [(channel, message.name) for channel, message in fake_lcm.calls],
            [
                ("vicon_state_data", "G1Pelvis"),
                ("vicon_state_data", "table"),
            ],
        )


if __name__ == "__main__":
    unittest.main()
