from __future__ import annotations

import itertools
import json
import tempfile
import threading
import unittest
from collections import deque
from pathlib import Path
from unittest import mock

import numpy as np
from scipy.spatial.transform import Rotation

from deploy.mocap_bridge.chingmu_sdk_client import (
    ChingMuSdkClient,
    Timeval,
    TrackerReport,
)
from deploy.mocap_bridge.mocap_types import MocapFrame
from deploy.mocap_bridge.chingmu_table_lcm_bridge import (
    BridgeConfig,
    ChingMuTableLcmBridge,
    PelvisOrientationCalibration,
    TableFrame,
    _collect_calibration_frames,
    _load_runtime_pelvis_orientation,
    _operation_mode,
    _validate_calibration_path_roles,
    _validate_numeric_arguments,
    build_arg_parser,
    calibrate_pelvis_orientation_from_frames,
    load_pelvis_orientation_calibration,
    raw_to_table_world,
    save_pelvis_orientation_calibration,
    save_table_frame,
    main,
)


def identity_table() -> TableFrame:
    return TableFrame(
        center_raw_mm=np.zeros(3, dtype=np.float64),
        x_axis_raw=np.array([1.0, 0.0, 0.0], dtype=np.float64),
        y_axis_raw=np.array([0.0, 1.0, 0.0], dtype=np.float64),
        z_axis_raw=np.array([0.0, 0.0, 1.0], dtype=np.float64),
        corners_raw_mm=np.zeros((4, 3), dtype=np.float64),
        rectangle_score=0.0,
    )


_USE_FRAME_SOURCE_TIME = object()


def mocap_frame(
    index: int,
    rotation_world: Rotation,
    *,
    position_mm=(100.0, 200.0, 300.0),
    source_time_s: float | None = None,
    body_pose_source_time_s=_USE_FRAME_SOURCE_TIME,
) -> MocapFrame:
    effective_source_time_s = index / 40.0 if source_time_s is None else source_time_s
    quaternion = rotation_world.as_quat()
    if index % 2:
        quaternion = -quaternion
    return MocapFrame(
        frame_number=1000 + index,
        source_time_s=effective_source_time_s,
        body_position_mm=np.asarray(position_mm, dtype=np.float64),
        body_quaternion_xyzw=quaternion,
        body_markers_mm={},
        unlabeled_markers_mm=np.empty((0, 3), dtype=np.float64),
        body_pose_source_time_s=(
            effective_source_time_s if body_pose_source_time_s is _USE_FRAME_SOURCE_TIME else body_pose_source_time_s
        ),
    )


def stable_frames(
    rigid_world: Rotation,
    *,
    count: int = 61,
) -> list[MocapFrame]:
    return [mocap_frame(index, rigid_world) for index in range(count)]


def valid_orientation_calibration(
    rotation_rigid_from_pelvis: Rotation | None = None,
) -> PelvisOrientationCalibration:
    rotation = Rotation.identity() if rotation_rigid_from_pelvis is None else rotation_rigid_from_pelvis
    return PelvisOrientationCalibration(
        rotation_rigid_from_pelvis=rotation.as_matrix(),
        sample_count=61,
        source_duration_s=1.5,
        position_rms_m=0.0002,
        angular_rms_deg=0.05,
    )


class PelvisOrientationCalibrationMathTest(unittest.TestCase):
    def setUp(self):
        self.config = BridgeConfig()
        self.table = identity_table()

    def test_recovers_rigid_from_pelvis_rotation_from_aligned_pose(self):
        expected = Rotation.from_euler("xyz", [0.12, -0.08, 0.35])
        rigid_world_in_aligned_pose = expected.inv()

        calibration = calibrate_pelvis_orientation_from_frames(
            stable_frames(rigid_world_in_aligned_pose),
            self.table,
            self.config,
        )

        recovered = Rotation.from_matrix(calibration.rotation_rigid_from_pelvis)
        self.assertLess((recovered.inv() * expected).magnitude(), 1.0e-10)
        self.assertEqual(calibration.sample_count, 61)
        self.assertAlmostEqual(calibration.source_duration_s, 1.5)
        self.assertAlmostEqual(calibration.position_rms_m, 0.0)
        self.assertAlmostEqual(calibration.angular_rms_deg, 0.0)

    def test_quaternion_sign_changes_do_not_change_rotation_mean(self):
        expected = Rotation.from_euler("xyz", [-0.2, 0.1, -0.4])

        calibration = calibrate_pelvis_orientation_from_frames(
            stable_frames(expected.inv()),
            self.table,
            self.config,
        )

        recovered = Rotation.from_matrix(calibration.rotation_rigid_from_pelvis)
        self.assertLess((recovered.inv() * expected).magnitude(), 1.0e-10)

    def test_rejects_insufficient_sample_count(self):
        with self.assertRaisesRegex(ValueError, "at least 30 valid"):
            calibrate_pelvis_orientation_from_frames(
                stable_frames(Rotation.identity(), count=29),
                self.table,
                self.config,
            )

    def test_rejects_insufficient_source_time_coverage(self):
        frames = [
            mocap_frame(
                index,
                Rotation.identity(),
                source_time_s=index / 100.0,
            )
            for index in range(61)
        ]

        with self.assertRaisesRegex(ValueError, "source-time coverage"):
            calibrate_pelvis_orientation_from_frames(
                frames,
                self.table,
                self.config,
            )

    def test_uses_rigid_pose_timestamps_for_source_time_coverage(self):
        pose_times = np.linspace(0.0, 0.9, 61)
        frame_offsets = np.linspace(-0.09, 0.09, 61)
        frames = [
            mocap_frame(
                index,
                Rotation.identity(),
                source_time_s=pose_times[index] + frame_offsets[index],
                body_pose_source_time_s=pose_times[index],
            )
            for index in range(61)
        ]

        with self.assertRaisesRegex(ValueError, "source-time coverage"):
            calibrate_pelvis_orientation_from_frames(
                frames,
                self.table,
                self.config,
            )

    def test_rejects_position_motion_above_two_millimetres_rms(self):
        frames = [
            mocap_frame(
                index,
                Rotation.identity(),
                position_mm=(100.0 + 10.0 * np.sin(index), 200.0, 300.0),
            )
            for index in range(61)
        ]

        with self.assertRaisesRegex(ValueError, "position RMS"):
            calibrate_pelvis_orientation_from_frames(
                frames,
                self.table,
                self.config,
            )

    def test_rejects_angular_motion_above_point_three_degrees_rms(self):
        frames = [
            mocap_frame(
                index,
                Rotation.from_euler(
                    "z",
                    np.linspace(-1.0, 1.0, 61)[index],
                    degrees=True,
                ),
            )
            for index in range(61)
        ]

        with self.assertRaisesRegex(ValueError, "angular RMS"):
            calibrate_pelvis_orientation_from_frames(
                frames,
                self.table,
                self.config,
            )

    def test_rejects_repeated_rigid_pose_timestamps_as_stale_samples(self):
        frames = [
            mocap_frame(
                index,
                Rotation.identity(),
                body_pose_source_time_s=(index // 3) * 3.0 / 40.0,
            )
            for index in range(61)
        ]

        with self.assertRaisesRegex(
            ValueError,
            "at least 30 valid fresh unique",
        ):
            calibrate_pelvis_orientation_from_frames(
                frames,
                self.table,
                self.config,
            )

    def test_rejects_nonfinite_rigid_pose_timestamps(self):
        nonfinite_times = (
            float("nan"),
            float("inf"),
            float("-inf"),
        )
        frames = [
            mocap_frame(
                index,
                Rotation.identity(),
                body_pose_source_time_s=nonfinite_times[index % len(nonfinite_times)],
            )
            for index in range(61)
        ]

        with self.assertRaisesRegex(
            ValueError,
            "at least 30 valid fresh unique",
        ):
            calibrate_pelvis_orientation_from_frames(
                frames,
                self.table,
                self.config,
            )

    def test_rejects_rigid_pose_timestamp_older_than_frame_age_limit(self):
        frames = [
            mocap_frame(
                index,
                Rotation.identity(),
                source_time_s=index / 40.0 + 0.100001,
                body_pose_source_time_s=index / 40.0,
            )
            for index in range(61)
        ]

        with self.assertRaisesRegex(
            ValueError,
            "at least 30 valid fresh unique",
        ):
            calibrate_pelvis_orientation_from_frames(
                frames,
                self.table,
                self.config,
            )


class PelvisOrientationCalibrationJsonTest(unittest.TestCase):
    def setUp(self):
        self.config = BridgeConfig()

    def test_json_round_trip_preserves_rotation_and_quality(self):
        expected = valid_orientation_calibration(Rotation.from_euler("xyz", [0.2, -0.1, 0.4]))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "pelvis_orientation.json"

            save_pelvis_orientation_calibration(
                path,
                expected,
                self.config,
            )
            loaded = load_pelvis_orientation_calibration(
                path,
                self.config,
            )
            payload = json.loads(path.read_text(encoding="utf-8"))

        np.testing.assert_allclose(
            loaded.rotation_rigid_from_pelvis,
            expected.rotation_rigid_from_pelvis,
            atol=1.0e-12,
        )
        self.assertEqual(loaded.sample_count, expected.sample_count)
        self.assertEqual(
            payload["rotation_convention"],
            "R_world_pelvis = R_world_rigid @ R_rigid_from_pelvis",
        )
        self.assertGreaterEqual(
            payload["quaternion_rigid_from_pelvis_xyzw"][3],
            0.0,
        )
        self.assertAlmostEqual(
            np.linalg.norm(payload["quaternion_rigid_from_pelvis_xyzw"]),
            1.0,
        )

    def test_rejects_invalid_json_fields(self):
        valid_payload = {
            "format": "robotbridge2_chingmu_pelvis_orientation_v1",
            "base_subject": "G2Pelvis",
            "quaternion_convention": "xyzw",
            "rotation_convention": ("R_world_pelvis = " "R_world_rigid @ R_rigid_from_pelvis"),
            "quaternion_rigid_from_pelvis_xyzw": [0.0, 0.0, 0.0, 1.0],
            "sample_count": 61,
            "source_duration_s": 1.5,
            "position_rms_m": 0.0002,
            "angular_rms_deg": 0.05,
        }
        mutations = {
            "format": {"format": "wrong"},
            "subject": {"base_subject": "OtherBody"},
            "quaternion_convention": {"quaternion_convention": "wxyz"},
            "rotation_convention": {"rotation_convention": "wrong"},
            "quaternion_non_unit": {"quaternion_rigid_from_pelvis_xyzw": [0.0, 0.0, 0.0, 2.0]},
            "quaternion_non_finite": {
                "quaternion_rigid_from_pelvis_xyzw": [
                    0.0,
                    0.0,
                    float("nan"),
                    1.0,
                ]
            },
            "quaternion_wrong_size": {"quaternion_rigid_from_pelvis_xyzw": [0.0, 0.0, 1.0]},
            "sample_count": {"sample_count": 29},
            "source_duration": {"source_duration_s": 0.5},
            "position_quality": {"position_rms_m": 0.0021},
            "angular_quality": {"angular_rms_deg": 0.31},
            "non_finite_quality": {"position_rms_m": float("inf")},
        }

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "pelvis_orientation.json"
            for label, mutation in mutations.items():
                with self.subTest(label=label):
                    payload = dict(valid_payload)
                    payload.update(mutation)
                    path.write_text(
                        json.dumps(payload),
                        encoding="utf-8",
                    )
                    with self.assertRaises(ValueError):
                        load_pelvis_orientation_calibration(
                            path,
                            self.config,
                        )

            for missing_field in valid_payload:
                with self.subTest(missing_field=missing_field):
                    missing_field_payload = dict(valid_payload)
                    del missing_field_payload[missing_field]
                    path.write_text(
                        json.dumps(missing_field_payload),
                        encoding="utf-8",
                    )
                    with self.assertRaises(ValueError):
                        load_pelvis_orientation_calibration(
                            path,
                            self.config,
                        )

            path.write_text("[]", encoding="utf-8")
            with self.assertRaises(ValueError):
                load_pelvis_orientation_calibration(
                    path,
                    self.config,
                )

            path.write_text("{", encoding="utf-8")
            with self.assertRaises(ValueError):
                load_pelvis_orientation_calibration(
                    path,
                    self.config,
                )

    def test_missing_json_file_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "missing.json"
            with self.assertRaisesRegex(
                ValueError,
                "failed to read pelvis orientation calibration",
            ):
                load_pelvis_orientation_calibration(
                    path,
                    self.config,
                )

    def test_failed_save_does_not_overwrite_existing_file(self):
        invalid = PelvisOrientationCalibration(
            rotation_rigid_from_pelvis=np.eye(3),
            sample_count=61,
            source_duration_s=1.5,
            position_rms_m=0.5,
            angular_rms_deg=0.05,
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "pelvis_orientation.json"
            path.write_text("keep-existing\n", encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "position RMS"):
                save_pelvis_orientation_calibration(
                    path,
                    invalid,
                    self.config,
                )

            self.assertEqual(
                path.read_text(encoding="utf-8"),
                "keep-existing\n",
            )


class PelvisOrientationRuntimeTest(unittest.TestCase):
    def setUp(self):
        self.config = BridgeConfig()
        self.table = identity_table()

    def base_message(
        self,
        bridge: ChingMuTableLcmBridge,
        frame: MocapFrame,
    ):
        return next(
            message
            for message in bridge.process_frame(
                frame,
                publish_time_us=123,
            )
            if message.name == self.config.base_subject
        )

    def test_without_pelvis_origin_translation_keeps_rigid_origin_and_rotation_is_right_multiplied(self):
        rigid_world = Rotation.from_euler("xyz", [0.31, -0.22, 0.47])
        rigid_from_pelvis = Rotation.from_euler(
            "xyz",
            [-0.18, 0.26, -0.33],
        )
        bridge = ChingMuTableLcmBridge(
            config=self.config,
            table_frame=self.table,
            pelvis_orientation_calibration=(valid_orientation_calibration(rigid_from_pelvis)),
        )
        frame = mocap_frame(0, rigid_world)

        base = self.base_message(bridge, frame)

        np.testing.assert_allclose(
            base.pos_vicon,
            raw_to_table_world(
                frame.body_position_mm,
                self.table,
                self.config,
            ),
            atol=1.0e-12,
        )
        published = Rotation.from_quat(base.quat_vicon)
        expected = rigid_world * rigid_from_pelvis
        self.assertLess(
            (published.inv() * expected).magnitude(),
            1.0e-10,
        )

    def test_loaded_pelvis_origin_translation_offsets_position_in_pelvis_frame(self):
        rigid_world = Rotation.from_euler("xyz", [0.31, -0.22, 0.47])
        rigid_from_pelvis = Rotation.from_euler(
            "xyz",
            [-0.18, 0.26, -0.33],
        )
        offset_pelvis_m = np.array([0.01, -0.02, 0.07605])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "pelvis_orientation.json"
            path.write_text(
                json.dumps(
                    {
                        "format": "robotbridge2_chingmu_pelvis_orientation_v1",
                        "base_subject": "G2Pelvis",
                        "quaternion_convention": "xyzw",
                        "rotation_convention": ("R_world_pelvis = " "R_world_rigid @ R_rigid_from_pelvis"),
                        "quaternion_rigid_from_pelvis_xyzw": (rigid_from_pelvis.as_quat().tolist()),
                        "translation_convention": (
                            "p_world_pelvis = p_world_rigid + "
                            "R_world_pelvis @ "
                            "translation_rigid_origin_to_pelvis_origin_pelvis_m"
                        ),
                        "translation_rigid_origin_to_pelvis_origin_pelvis_m": (offset_pelvis_m.tolist()),
                        "sample_count": 61,
                        "source_duration_s": 1.5,
                        "position_rms_m": 0.0002,
                        "angular_rms_deg": 0.05,
                    }
                ),
                encoding="utf-8",
            )
            calibration = load_pelvis_orientation_calibration(
                path,
                self.config,
            )
        bridge = ChingMuTableLcmBridge(
            config=self.config,
            table_frame=self.table,
            pelvis_orientation_calibration=calibration,
        )
        frame = mocap_frame(0, rigid_world)

        base = self.base_message(bridge, frame)

        rigid_position_world = raw_to_table_world(
            frame.body_position_mm,
            self.table,
            self.config,
        )
        pelvis_world = rigid_world * rigid_from_pelvis
        expected_position = rigid_position_world + pelvis_world.apply(offset_pelvis_m)
        np.testing.assert_allclose(
            base.pos_vicon,
            expected_position,
            atol=1.0e-12,
        )
        published = Rotation.from_quat(base.quat_vicon)
        self.assertLess(
            (published.inv() * pelvis_world).magnitude(),
            1.0e-10,
        )

    def test_nonunit_table_basis_preserves_full_transform_and_local_x(self):
        table_rotation = Rotation.from_euler(
            "xyz",
            [0.31, -0.22, 0.47],
        )
        unit_basis = table_rotation.as_matrix()
        table = TableFrame(
            center_raw_mm=np.array([120.0, -80.0, 40.0]),
            x_axis_raw=2.0 * unit_basis[0],
            y_axis_raw=3.0 * unit_basis[1],
            z_axis_raw=4.0 * unit_basis[2],
            corners_raw_mm=np.zeros((4, 3)),
            rectangle_score=0.0,
        )
        rigid_rotation_raw = Rotation.from_euler(
            "xyz",
            [-0.28, 0.41, -0.19],
        )
        rigid_from_pelvis = Rotation.from_euler(
            "xyz",
            [0.17, -0.36, 0.29],
        )
        bridge = ChingMuTableLcmBridge(
            config=self.config,
            table_frame=table,
            pelvis_orientation_calibration=(valid_orientation_calibration(rigid_from_pelvis)),
        )
        raw_position_mm = table.center_raw_mm + np.array([250.0, -400.0, 550.0])
        frame = mocap_frame(
            0,
            rigid_rotation_raw,
            position_mm=raw_position_mm,
        )

        base = self.base_message(bridge, frame)

        expected_position = (
            np.array(
                [
                    0.5 * self.config.table_length_m,
                    0.0,
                    self.config.table_height_m,
                ]
            )
            + unit_basis @ (raw_position_mm - table.center_raw_mm) * 0.001
        )
        np.testing.assert_allclose(
            base.pos_vicon,
            expected_position,
            atol=1.0e-12,
        )
        published = Rotation.from_quat(base.quat_vicon)
        expected_rotation = table_rotation * rigid_rotation_raw * rigid_from_pelvis
        self.assertLess(
            (published.inv() * expected_rotation).magnitude(),
            1.0e-10,
        )
        np.testing.assert_allclose(
            published.apply([1.0, 0.0, 0.0]),
            expected_rotation.apply([1.0, 0.0, 0.0]),
            atol=1.0e-10,
        )

    def test_first_frame_keeps_corrected_absolute_yaw(self):
        correction = Rotation.from_euler("z", -0.4)
        bridge = ChingMuTableLcmBridge(
            config=self.config,
            table_frame=self.table,
            pelvis_orientation_calibration=(valid_orientation_calibration(correction)),
        )

        first = self.base_message(
            bridge,
            mocap_frame(0, Rotation.from_euler("z", 0.75)),
        )
        second = self.base_message(
            bridge,
            mocap_frame(1, Rotation.from_euler("z", 0.90)),
        )

        first_yaw = Rotation.from_quat(first.quat_vicon).as_euler("xyz")[2]
        second_yaw = Rotation.from_quat(second.quat_vicon).as_euler("xyz")[2]
        self.assertAlmostEqual(first_yaw, 0.35, places=10)
        self.assertAlmostEqual(second_yaw, 0.50, places=10)

    def test_large_finite_quaternion_is_stably_normalized(self):
        bridge = ChingMuTableLcmBridge(
            config=self.config,
            table_frame=self.table,
            pelvis_orientation_calibration=(valid_orientation_calibration()),
        )
        frame = MocapFrame(
            frame_number=1000,
            source_time_s=1.0,
            body_position_mm=np.array([100.0, 200.0, 300.0]),
            body_quaternion_xyzw=np.array([1.0e308, -1.0e308, 1.0e308, 1.0e308]),
            body_markers_mm={},
            unlabeled_markers_mm=np.empty((0, 3)),
            body_pose_source_time_s=1.0,
        )

        base = self.base_message(bridge, frame)

        published = Rotation.from_quat(base.quat_vicon)
        expected = Rotation.from_quat([0.5, -0.5, 0.5, 0.5])
        self.assertLess(
            (published.inv() * expected).magnitude(),
            1.0e-10,
        )

    def test_calibrated_left_quarter_turn_points_forward_at_world_y(self):
        correction = Rotation.from_euler(
            "xyz",
            [0.12, -0.08, 0.35],
        )
        desired_pelvis_world = Rotation.from_euler("z", 0.5 * np.pi)
        rigid_world = desired_pelvis_world * correction.inv()
        bridge = ChingMuTableLcmBridge(
            config=self.config,
            table_frame=self.table,
            pelvis_orientation_calibration=(valid_orientation_calibration(correction)),
        )

        base = self.base_message(
            bridge,
            mocap_frame(0, rigid_world),
        )

        published = Rotation.from_quat(base.quat_vicon)
        forward_world = published.apply([1.0, 0.0, 0.0])
        np.testing.assert_allclose(
            forward_world,
            [0.0, 1.0, 0.0],
            atol=1.0e-10,
        )

    def test_valid_pose_loss_still_emits_one_invalid_transition(self):
        bridge = ChingMuTableLcmBridge(
            config=self.config,
            table_frame=self.table,
            pelvis_orientation_calibration=(valid_orientation_calibration()),
        )
        bridge.process_frame(mocap_frame(0, Rotation.identity()))
        missing = mocap_frame(1, Rotation.identity())
        missing = MocapFrame(
            frame_number=missing.frame_number,
            source_time_s=missing.source_time_s,
            body_position_mm=None,
            body_quaternion_xyzw=None,
            body_markers_mm={},
            unlabeled_markers_mm=np.empty((0, 3)),
        )

        first_loss = bridge.process_frame(missing)
        second_loss = bridge.process_frame(missing)

        first_base = [message for message in first_loss if message.name == self.config.base_subject]
        second_base = [message for message in second_loss if message.name == self.config.base_subject]
        self.assertEqual(len(first_base), 1)
        self.assertEqual(first_base[0].valid, 0)
        self.assertEqual(second_base, [])

    def test_normal_runtime_requires_orientation_file(self):
        args = build_arg_parser().parse_args([])

        with self.assertRaisesRegex(
            ValueError,
            "--pelvis-orientation-calib is required",
        ):
            _load_runtime_pelvis_orientation(
                args,
                self.config,
            )


class PelvisOrientationCliTest(unittest.TestCase):
    def setUp(self):
        self.parser = build_arg_parser()

    def test_default_base_subject_is_g2pelvis(self):
        self.assertEqual(BridgeConfig().base_subject, "G2Pelvis")
        self.assertEqual(
            self.parser.parse_args([]).base_subject,
            "G2Pelvis",
        )

    def test_selects_pelvis_calibration_mode(self):
        args = self.parser.parse_args(
            [
                "--table-calib",
                "table.json",
                "--save-pelvis-orientation-calib",
                "pelvis.json",
            ]
        )
        self.assertEqual(_operation_mode(args), "pelvis_calibration")

    def test_selects_runtime_mode(self):
        args = self.parser.parse_args(["--pelvis-orientation-calib", "pelvis.json"])
        self.assertEqual(_operation_mode(args), "runtime")

    def test_preserves_table_only_calibration_mode(self):
        args = self.parser.parse_args(["--save-table-calib", "table.json"])
        self.assertEqual(_operation_mode(args), "table_calibration")

    def test_rejects_all_resolved_calibration_path_role_collisions(self):
        roles = (
            "--table-calib",
            "--save-table-calib",
            "--pelvis-orientation-calib",
            "--save-pelvis-orientation-calib",
        )
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "calibration.json"
            alias = Path(directory) / "missing" / ".." / "calibration.json"
            for first_role, second_role in itertools.combinations(roles, 2):
                with self.subTest(
                    first_role=first_role,
                    second_role=second_role,
                ):
                    args = self.parser.parse_args(
                        [
                            first_role,
                            str(target),
                            second_role,
                            str(alias),
                        ]
                    )
                    with self.assertRaisesRegex(
                        ValueError,
                        f"{first_role}.*{second_role}",
                    ):
                        _validate_calibration_path_roles(args)

    def test_table_calibration_load_and_save_are_mutually_exclusive(self):
        args = self.parser.parse_args(
            [
                "--table-calib",
                "input-table.json",
                "--save-table-calib",
                "output-table.json",
                "--pelvis-orientation-calib",
                "pelvis.json",
            ]
        )

        with self.assertRaisesRegex(
            ValueError,
            "--table-calib.*--save-table-calib",
        ):
            _operation_mode(args)

    def test_pelvis_calibration_requires_existing_table_file_argument(self):
        args = self.parser.parse_args(
            [
                "--save-pelvis-orientation-calib",
                "pelvis.json",
            ]
        )
        with self.assertRaisesRegex(
            ValueError,
            "--table-calib is required",
        ):
            _operation_mode(args)

    def test_pelvis_calibration_forbids_publish(self):
        args = self.parser.parse_args(
            [
                "--table-calib",
                "table.json",
                "--save-pelvis-orientation-calib",
                "pelvis.json",
                "--publish",
            ]
        )
        with self.assertRaisesRegex(
            ValueError,
            "cannot be combined with --publish",
        ):
            _operation_mode(args)

    def test_load_and_save_orientation_are_mutually_exclusive(self):
        args = self.parser.parse_args(
            [
                "--table-calib",
                "table.json",
                "--pelvis-orientation-calib",
                "old.json",
                "--save-pelvis-orientation-calib",
                "new.json",
            ]
        )
        with self.assertRaisesRegex(ValueError, "mutually exclusive"):
            _operation_mode(args)

    def test_missing_orientation_file_is_rejected_outside_table_mode(self):
        args = self.parser.parse_args([])
        with self.assertRaisesRegex(
            ValueError,
            "--pelvis-orientation-calib is required",
        ):
            _operation_mode(args)


class PelvisOrientationNumericCliTest(unittest.TestCase):
    def setUp(self):
        self.parser = build_arg_parser()

    def validate(self, arguments):
        args = self.parser.parse_args(arguments)
        mode = _operation_mode(args)
        _validate_numeric_arguments(args, mode)

    def test_table_collection_requires_finite_positive_calib_sec(self):
        for value in ("nan", "inf", "-inf", "0", "-1"):
            with self.subTest(value=value):
                with self.assertRaisesRegex(ValueError, "--calib-sec"):
                    self.validate(
                        [
                            "--save-table-calib",
                            "table.json",
                            f"--calib-sec={value}",
                        ]
                    )

    def test_runtime_table_collection_requires_finite_positive_calib_sec(self):
        with self.assertRaisesRegex(ValueError, "--calib-sec"):
            self.validate(
                [
                    "--pelvis-orientation-calib",
                    "pelvis.json",
                    "--calib-sec",
                    "nan",
                ]
            )

    def test_loaded_table_runtime_ignores_unused_calib_durations(self):
        self.validate(
            [
                "--table-calib",
                "table.json",
                "--pelvis-orientation-calib",
                "pelvis.json",
                "--calib-sec",
                "nan",
                "--pelvis-calib-sec",
                "inf",
            ]
        )

    def test_pelvis_collection_requires_finite_positive_pelvis_calib_sec(self):
        for value in ("nan", "inf", "-inf", "0", "-1"):
            with self.subTest(value=value):
                with self.assertRaisesRegex(
                    ValueError,
                    "--pelvis-calib-sec",
                ):
                    self.validate(
                        [
                            "--table-calib",
                            "table.json",
                            "--save-pelvis-orientation-calib",
                            "pelvis.json",
                            f"--pelvis-calib-sec={value}",
                        ]
                    )

    def test_runtime_requires_finite_nonnegative_duration(self):
        for value in ("nan", "inf", "-inf", "-1"):
            with self.subTest(value=value):
                with self.assertRaisesRegex(ValueError, "--duration"):
                    self.validate(
                        [
                            "--table-calib",
                            "table.json",
                            "--pelvis-orientation-calib",
                            "pelvis.json",
                            f"--duration={value}",
                        ]
                    )
        self.validate(
            [
                "--table-calib",
                "table.json",
                "--pelvis-orientation-calib",
                "pelvis.json",
                "--duration",
                "0",
            ]
        )

    def test_rates_require_finite_positive_values(self):
        for argument_name in (
            "--source-rate-hz",
            "--body-pose-poll-hz",
        ):
            for value in ("nan", "inf", "-inf", "0", "-1"):
                with self.subTest(
                    argument_name=argument_name,
                    value=value,
                ):
                    with self.assertRaisesRegex(
                        ValueError,
                        argument_name,
                    ):
                        self.validate(
                            [
                                "--table-calib",
                                "table.json",
                                "--pelvis-orientation-calib",
                                "pelvis.json",
                                f"{argument_name}={value}",
                            ]
                        )

    def test_invalid_preflight_never_constructs_sdk_client(self):
        from deploy.mocap_bridge import chingmu_sdk_client

        invalid_cases = (
            (
                "--duration",
                [
                    "--table-calib",
                    "table.json",
                    "--pelvis-orientation-calib",
                    "pelvis.json",
                    "--duration",
                    "nan",
                ],
            ),
            (
                "--table-calib",
                [
                    "--table-calib",
                    "input-table.json",
                    "--save-table-calib",
                    "output-table.json",
                    "--pelvis-orientation-calib",
                    "pelvis.json",
                ],
            ),
        )
        with mock.patch.object(
            chingmu_sdk_client,
            "ChingMuSdkClient",
            side_effect=AssertionError("SDK client must not be constructed"),
        ) as sdk_constructor:
            for expected_error, arguments in invalid_cases:
                with self.subTest(expected_error=expected_error):
                    with self.assertRaisesRegex(
                        ValueError,
                        expected_error,
                    ):
                        main(arguments)
            sdk_constructor.assert_not_called()


class CalibrationFrameCollectionTest(unittest.TestCase):
    def test_nonfinite_duration_is_rejected_before_reading_client(self):
        class ClientThatMustNotBeRead:
            def __init__(self):
                self.calls = 0

            def next_frame(self, *, timeout_s):
                self.calls += 1
                raise AssertionError("next_frame must not be called")

        for duration_s in (float("nan"), float("inf"), float("-inf")):
            with self.subTest(duration_s=duration_s):
                client = ClientThatMustNotBeRead()
                with self.assertRaisesRegex(
                    ValueError,
                    "--pelvis-calib-sec",
                ):
                    _collect_calibration_frames(
                        client,
                        duration_s,
                        argument_name="--pelvis-calib-sec",
                    )
                self.assertEqual(client.calls, 0)


class ChingMuMainInterruptTest(unittest.TestCase):
    class InterruptingSdkClient:
        instances = []

        def __init__(self, *args, **kwargs):
            del args, kwargs
            self.body_id = 6
            self.body_pose_id = 6
            self.dropped_frame_count = 0
            self.closed = False
            self.__class__.instances.append(self)

        def start(self):
            return None

        def next_frame(self, *, timeout_s=0.1):
            del timeout_s
            raise KeyboardInterrupt

        def close(self):
            self.closed = True

    def setUp(self):
        self.InterruptingSdkClient.instances.clear()

    def run_main_with_interrupting_sdk(self, arguments):
        from deploy.mocap_bridge import chingmu_sdk_client

        with mock.patch.object(
            chingmu_sdk_client,
            "ChingMuSdkClient",
            self.InterruptingSdkClient,
        ):
            result = main(arguments)

        self.assertEqual(len(self.InterruptingSdkClient.instances), 1)
        self.assertTrue(self.InterruptingSdkClient.instances[0].closed)
        return result

    def test_table_calibration_interrupt_returns_130_and_closes_client(self):
        with tempfile.TemporaryDirectory() as directory:
            output_path = Path(directory) / "table.json"

            result = self.run_main_with_interrupting_sdk(["--save-table-calib", str(output_path)])

            self.assertEqual(result, 130)
            self.assertFalse(output_path.exists())

    def test_pelvis_calibration_interrupt_returns_130_and_closes_client(self):
        with tempfile.TemporaryDirectory() as directory:
            table_path = Path(directory) / "table.json"
            output_path = Path(directory) / "pelvis.json"
            save_table_frame(table_path, identity_table(), BridgeConfig())

            result = self.run_main_with_interrupting_sdk(
                [
                    "--table-calib",
                    str(table_path),
                    "--save-pelvis-orientation-calib",
                    str(output_path),
                ]
            )

            self.assertEqual(result, 130)
            self.assertFalse(output_path.exists())

    def test_runtime_interrupt_returns_zero_and_closes_client(self):
        with tempfile.TemporaryDirectory() as directory:
            table_path = Path(directory) / "table.json"
            pelvis_path = Path(directory) / "pelvis.json"
            config = BridgeConfig()
            save_table_frame(table_path, identity_table(), config)
            save_pelvis_orientation_calibration(
                pelvis_path,
                valid_orientation_calibration(),
                config,
            )

            result = self.run_main_with_interrupting_sdk(
                [
                    "--table-calib",
                    str(table_path),
                    "--pelvis-orientation-calib",
                    str(pelvis_path),
                ]
            )

            self.assertEqual(result, 0)


class ChingMuSdkPoseTimestampTest(unittest.TestCase):
    def test_sdk_rejects_nonfinite_body_pose_poll_rate(self):
        for body_pose_poll_hz in (
            float("nan"),
            float("inf"),
            float("-inf"),
        ):
            with self.subTest(body_pose_poll_hz=body_pose_poll_hz):
                with mock.patch.object(
                    ChingMuSdkClient,
                    "_bind",
                    return_value=None,
                ):
                    with self.assertRaisesRegex(
                        ValueError,
                        "body_pose_poll_hz",
                    ):
                        ChingMuSdkClient(
                            "unused.so",
                            server_ip="192.168.2.100",
                            body_name="G2Pelvis",
                            body_pose_poll_hz=body_pose_poll_hz,
                            library=object(),
                        )

    def test_mocap_frame_accepts_optional_pose_source_timestamp(self):
        legacy_frame = MocapFrame(
            frame_number=999,
            source_time_s=12.5,
            body_position_mm=np.zeros(3),
            body_quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
            body_markers_mm={},
            unlabeled_markers_mm=np.empty((0, 3)),
        )
        timestamped_frame = MocapFrame(
            frame_number=1000,
            source_time_s=12.55,
            body_position_mm=np.zeros(3),
            body_quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
            body_markers_mm={},
            unlabeled_markers_mm=np.empty((0, 3)),
            body_pose_source_time_s=12.5,
        )

        self.assertIsNone(legacy_frame.body_pose_source_time_s)
        self.assertEqual(timestamped_frame.body_pose_source_time_s, 12.5)

    def test_root_callback_uses_its_own_sdk_timestamp(self):
        client = ChingMuSdkClient.__new__(ChingMuSdkClient)
        client._condition = threading.Condition()
        client._ready = deque(maxlen=1)
        client.dropped_frame_count = 0
        client.body_id = 6
        client.body_marker_sensor_range = range(8100, 8150)
        client._hierarchy_sensor_ids = {6}
        client._rigid_body_ids = {6}
        client._current_frame = None
        client._current_time_s = 0.0
        client._current_body_position = None
        client._current_body_quaternion = None
        client._current_body_markers = {}
        client._current_pending_unlabeled = []

        first_marker = TrackerReport()
        first_marker.msg_time = Timeval(12, 100000)
        first_marker.sensor = 8100
        first_marker.frameCounter = 100
        first_marker.pos[:] = (1.0, 2.0, 3.0)
        root = TrackerReport()
        root.msg_time = Timeval(12, 125000)
        root.sensor = 6
        root.frameCounter = 100
        root.pos[:] = (410.0, -20.0, 793.0)
        root.quat[:] = (0.0, 0.0, 0.2, 0.98)
        next_frame_marker = TrackerReport()
        next_frame_marker.msg_time = Timeval(12, 130000)
        next_frame_marker.sensor = 8100
        next_frame_marker.frameCounter = 101
        next_frame_marker.pos[:] = (1.0, 2.0, 3.0)

        client._on_tracker(None, first_marker)
        client._on_tracker(None, root)
        client._on_tracker(None, next_frame_marker)

        frame = client._ready.popleft()
        self.assertAlmostEqual(frame.source_time_s, 12.1)
        self.assertAlmostEqual(frame.body_pose_source_time_s, 12.125)

    def test_cold_poll_cache_falls_back_once_and_propagates_timestamp(self):
        class PollingLibrary:
            def __init__(self):
                self.calls = 0

            def CMTrackerExternTC(
                self,
                _server,
                _body_id,
                timecode_data,
                body_position,
                body_quaternion,
                timestamp_pointer,
            ):
                self.calls += 1
                timecode_data[0] = 42125
                body_position[:] = (410.0, -20.0, 793.0)
                body_quaternion[:] = (0.0, 0.0, 0.2, 0.98)
                timestamp = timestamp_pointer._obj
                timestamp.tv_sec = 42
                timestamp.tv_usec = 125000
                return True

        client = ChingMuSdkClient.__new__(ChingMuSdkClient)
        client.poll_body_pose = True
        client.body_pose_id = 300
        client._body_pose_servers = (b"G2Pelvis@192.168.2.100",)
        client._body_pose_lock = threading.Lock()
        client._latest_polled_body_pose = None
        client.lib = PollingLibrary()
        empty_frame = MocapFrame(
            frame_number=1000,
            source_time_s=42.13,
            body_position_mm=None,
            body_quaternion_xyzw=None,
            body_markers_mm={},
            unlabeled_markers_mm=np.empty((0, 3)),
        )

        populated = client._frame_with_polled_body_pose(empty_frame)

        self.assertEqual(client.lib.calls, 1)
        np.testing.assert_allclose(
            populated.body_position_mm,
            [410.0, -20.0, 793.0],
        )
        np.testing.assert_allclose(
            populated.body_quaternion_xyzw,
            [0.0, 0.0, 0.2, 0.98],
        )
        self.assertAlmostEqual(
            populated.body_pose_source_time_s,
            42.125,
        )


if __name__ == "__main__":
    unittest.main()
