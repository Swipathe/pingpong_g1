import copy
import math
import threading
import time
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

from tools.hitter_ready_pose_editor.asset_model import (
    AssetSourceChanged,
    HitterAssetModel,
)
from tools.hitter_ready_pose_editor.constants import (
    ARM_JOINT_NAMES,
    ROBOT29_ARM_INDICES,
)
from tools.hitter_ready_pose_editor.pose_validation import (
    PoseInputError,
    PoseValidationError,
    PoseValidator,
)
from utils.kinematics import ForwardKinematicsConfig, MujocoKinematics


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_ROOT_POS = np.array([-0.4, 0.0, 0.793], dtype=np.float64)
FK_LINKS = (
    "left_wrist_yaw_link",
    "right_wrist_yaw_link",
    "right_racket_link",
)


class _AssetModelDouble:
    def __init__(self, manifest, source_error=None):
        self._manifest = manifest
        self._source_error = source_error

    def build_manifest(self):
        return self._manifest

    def assert_source_hashes_unchanged(self):
        if self._source_error is not None:
            raise self._source_error


class _CountingKinematics:
    def __init__(self, body_info):
        self.body_info = body_info
        self.call_count = 0

    def forward(self, *_args, **_kwargs):
        self.call_count += 1
        return copy.deepcopy(self.body_info), None


def _identity_body_info(*link_names):
    result = {
        "pelvis": {
            "pos": np.zeros(3, dtype=np.float64),
            "quat": np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64),
        }
    }
    for index, name in enumerate(link_names, start=1):
        result[name] = {
            "pos": np.array([float(index), 0.0, 0.0], dtype=np.float64),
            "quat": np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64),
        }
    return result


class PoseValidatorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.asset_model = HitterAssetModel.from_defaults(REPO_ROOT)
        cls.manifest = cls.asset_model.build_manifest()
        cls.kinematics = MujocoKinematics(
            ForwardKinematicsConfig(
                xml_path=str(cls.manifest.asset_paths["mjcf"]),
                kinematic_joint_names=list(cls.manifest.active_joint_names),
                debug_viz=False,
            )
        )
        cls.validator = PoseValidator(
            cls.asset_model,
            cls.kinematics,
            DEFAULT_ROOT_POS,
        )
        cls.default_29 = np.asarray(
            cls.manifest.default_joint_pos_rad, dtype=np.float64
        )
        cls.default_arm_pose = {
            name: float(cls.default_29[index])
            for name, index in zip(ARM_JOINT_NAMES, ROBOT29_ARM_INDICES)
        }

    def test_compose_joint_pos_only_replaces_arm_indices(self):
        pose = dict(self.default_arm_pose)
        pose["left_elbow_joint"] = 0.75
        full = self.validator.compose_joint_pos(pose)
        self.assertEqual(full.shape, (29,))
        np.testing.assert_allclose(full[:15], self.default_29[:15])
        self.assertEqual(full[18], 0.75)

    def test_default_fk_matches_current_regression_anchors(self):
        result = self.validator.validate(self.default_arm_pose)
        np.testing.assert_allclose(
            result.fk_robot_base["links"]["left_wrist_yaw_link"]["position_m"],
            [0.097296, 0.214477, -0.024402],
            atol=1e-5,
        )
        np.testing.assert_allclose(
            result.fk_robot_base["links"]["right_wrist_yaw_link"]["position_m"],
            [0.097296, -0.214467, -0.024402],
            atol=1e-5,
        )
        np.testing.assert_allclose(
            result.fk_robot_base["links"]["right_racket_link"]["position_m"],
            [0.288860, -0.227561, -0.078599],
            atol=1e-5,
        )
        self.assertEqual(result.fk_robot_base["frame"], "robot_base_default")
        self.assertEqual(result.fk_robot_base["quaternion_convention"], "xyzw")

    def test_hard_limit_and_nonfinite_values_are_rejected(self):
        for bad in (float("nan"), float("inf"), -float("inf")):
            with self.subTest(bad=bad):
                pose = dict(self.default_arm_pose)
                pose["left_elbow_joint"] = bad
                with self.assertRaises(PoseInputError):
                    self.validator.validate(pose)
        pose = dict(self.default_arm_pose)
        pose["left_elbow_joint"] = 99.0
        with self.assertRaises(PoseInputError):
            self.validator.validate(pose)

    def test_browser_fk_mismatch_blocks_validation(self):
        browser_fk = copy.deepcopy(
            self.validator.validate(self.default_arm_pose).fk_robot_base
        )
        browser_fk["links"]["right_racket_link"]["position_m"][0] += 0.001
        with self.assertRaisesRegex(PoseValidationError, "position"):
            self.validator.validate(self.default_arm_pose, browser_fk)

    def test_browser_orientation_mismatch_blocks_validation(self):
        browser_fk = copy.deepcopy(
            self.validator.validate(self.default_arm_pose).fk_robot_base
        )
        angle = 0.01
        browser_fk["links"]["right_racket_link"]["quaternion_xyzw"] = [
            math.sin(angle / 2.0),
            0.0,
            0.0,
            math.cos(angle / 2.0),
        ]
        with self.assertRaisesRegex(PoseValidationError, "orientation"):
            self.validator.validate(self.default_arm_pose, browser_fk)

    def test_quaternion_sign_does_not_create_orientation_error(self):
        expected = self.validator.validate(self.default_arm_pose).fk_robot_base
        browser_fk = copy.deepcopy(expected)
        quat = browser_fk["links"]["right_racket_link"]["quaternion_xyzw"]
        browser_fk["links"]["right_racket_link"]["quaternion_xyzw"] = [
            -value for value in quat
        ]
        result = self.validator.validate(self.default_arm_pose, browser_fk)
        self.assertLessEqual(result.max_orientation_error_rad, 1e-12)

    def test_browser_fk_requires_finite_unit_quaternion_and_no_bool(self):
        expected = self.validator.validate(self.default_arm_pose).fk_robot_base
        for quaternion in (
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 100.0],
        ):
            with self.subTest(quaternion=quaternion):
                browser_fk = copy.deepcopy(expected)
                browser_fk["links"]["right_racket_link"][
                    "quaternion_xyzw"
                ] = quaternion
                with self.assertRaisesRegex(PoseInputError, "unit quaternion"):
                    self.validator.validate(self.default_arm_pose, browser_fk)
        browser_fk = copy.deepcopy(expected)
        browser_fk["links"]["right_racket_link"]["position_m"][0] = True
        with self.assertRaisesRegex(PoseInputError, "finite number"):
            self.validator.validate(self.default_arm_pose, browser_fk)

    def test_browser_fk_schema_is_exact_and_complete(self):
        expected = self.validator.validate(self.default_arm_pose).fk_robot_base

        def add_top_level(summary):
            summary["extra"] = None

        def change_frame(summary):
            summary["frame"] = "world"

        def remove_link(summary):
            del summary["links"]["left_wrist_yaw_link"]

        def add_link(summary):
            summary["links"]["extra_link"] = copy.deepcopy(
                summary["links"]["left_wrist_yaw_link"]
            )

        def shorten_position(summary):
            summary["links"]["right_racket_link"]["position_m"] = [0.0, 0.0]

        for mutate in (
            add_top_level,
            change_frame,
            remove_link,
            add_link,
            shorten_position,
        ):
            with self.subTest(mutate=mutate.__name__):
                browser_fk = copy.deepcopy(expected)
                mutate(browser_fk)
                with self.assertRaises(PoseInputError):
                    self.validator.validate(self.default_arm_pose, browser_fk)

    def test_name_set_is_exact(self):
        pose = dict(self.default_arm_pose)
        del pose["left_elbow_joint"]
        pose["unknown_joint"] = 0.0
        with self.assertRaisesRegex(
            PoseInputError, r"left_elbow_joint.*unknown_joint"
        ):
            self.validator.validate(pose)

    def test_value_type_is_strict(self):
        accepted_pose = dict(self.default_arm_pose)
        for bad in (True, "0.2", None, float("nan"), float("inf"), -float("inf")):
            with self.subTest(bad=bad):
                pose = dict(accepted_pose)
                pose["left_elbow_joint"] = bad
                with self.assertRaises(PoseInputError):
                    self.validator.validate(pose)
                self.assertEqual(accepted_pose, self.default_arm_pose)

    def test_hard_limit_is_not_clamped(self):
        spec = self.manifest.joint_by_name["left_elbow_joint"]
        lower, upper = spec.hard_limit_rad
        value = upper + 1e-6
        pose = dict(self.default_arm_pose)
        pose["left_elbow_joint"] = value
        pattern = r"left_elbow_joint={}.*\[{}, {}\]".format(
            value, lower, upper
        )
        with self.assertRaisesRegex(PoseInputError, pattern):
            self.validator.validate(pose)

    def test_soft_warning_is_nonblocking(self):
        spec = self.manifest.joint_by_name["left_elbow_joint"]
        value = spec.soft_limit_rad[1] + 1e-6
        self.assertLess(value, spec.hard_limit_rad[1])
        pose = dict(self.default_arm_pose)
        pose["left_elbow_joint"] = value
        result = self.validator.validate(pose)
        self.assertEqual(len(result.soft_limit_warnings), 1)
        self.assertTrue(result.needs_soft_limit_confirmation)

    def test_left_right_chains_are_independent(self):
        baseline = self.validator.validate(
            self.default_arm_pose
        ).fk_robot_base["links"]

        left_pose = dict(self.default_arm_pose)
        left_pose["left_elbow_joint"] += 0.1
        left = self.validator.validate(left_pose).fk_robot_base["links"]

        right_pose = dict(self.default_arm_pose)
        right_pose["right_wrist_pitch_joint"] += 0.1
        right = self.validator.validate(right_pose).fk_robot_base["links"]

        for link_name in ("right_wrist_yaw_link", "right_racket_link"):
            np.testing.assert_allclose(
                left[link_name]["position_m"],
                baseline[link_name]["position_m"],
                atol=1e-10,
            )
            np.testing.assert_allclose(
                left[link_name]["quaternion_xyzw"],
                baseline[link_name]["quaternion_xyzw"],
                atol=1e-10,
            )
        np.testing.assert_allclose(
            right["left_wrist_yaw_link"]["position_m"],
            baseline["left_wrist_yaw_link"]["position_m"],
            atol=1e-10,
        )
        self.assertGreater(
            np.linalg.norm(
                np.asarray(right["right_racket_link"]["position_m"])
                - np.asarray(baseline["right_racket_link"]["position_m"])
            ),
            1e-6,
        )

    def test_pelvis_relative_transform(self):
        root_quat = np.array(
            [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)],
            dtype=np.float64,
        )
        root_rotation = np.array(
            [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        root_position = np.array([1.0, 2.0, 3.0], dtype=np.float64)
        relative_position = np.array([0.2, -0.1, 0.3], dtype=np.float64)
        body_info = {
            "pelvis": {"pos": root_position, "quat": root_quat},
            "child": {
                "pos": root_position + root_rotation.dot(relative_position),
                "quat": root_quat,
            },
        }
        fake_fk = _CountingKinematics(body_info)
        validator = PoseValidator(
            _AssetModelDouble(self.manifest),
            fake_fk,
            DEFAULT_ROOT_POS,
        )
        actual = validator.forward_links_robot_base(
            self.default_29, ("child",)
        )["child"]
        np.testing.assert_allclose(
            actual["position_m"], relative_position, atol=1e-12
        )
        np.testing.assert_allclose(
            actual["quaternion_xyzw"], [0.0, 0.0, 0.0, 1.0], atol=1e-12
        )

    def test_asset_mismatch_precedes_fk(self):
        fake_fk = _CountingKinematics(_identity_body_info(*FK_LINKS))
        validator = PoseValidator(
            _AssetModelDouble(
                self.manifest, AssetSourceChanged("forced source drift")
            ),
            fake_fk,
            DEFAULT_ROOT_POS,
        )
        with self.assertRaisesRegex(AssetSourceChanged, "source drift"):
            validator.validate(self.default_arm_pose)
        self.assertEqual(fake_fk.call_count, 0)

    def test_direct_fk_asset_mismatch_precedes_fk(self):
        fake_fk = _CountingKinematics(_identity_body_info(*FK_LINKS))
        validator = PoseValidator(
            _AssetModelDouble(
                self.manifest, AssetSourceChanged("forced source drift")
            ),
            fake_fk,
            DEFAULT_ROOT_POS,
        )
        with self.assertRaisesRegex(AssetSourceChanged, "source drift"):
            validator.forward_links_robot_base(self.default_29, FK_LINKS)
        self.assertEqual(fake_fk.call_count, 0)

    def test_incompatible_manifest_precedes_fk(self):
        fake_fk = _CountingKinematics(_identity_body_info(*FK_LINKS))
        incompatible = replace(
            self.manifest,
            compatible_for_save=False,
            incompatibilities=("forced mismatch",),
        )
        validator = PoseValidator(
            _AssetModelDouble(incompatible),
            fake_fk,
            DEFAULT_ROOT_POS,
        )
        with self.assertRaisesRegex(PoseValidationError, "forced mismatch"):
            validator.validate(self.default_arm_pose)
        self.assertEqual(fake_fk.call_count, 0)

    def test_direct_fk_incompatible_manifest_precedes_fk(self):
        fake_fk = _CountingKinematics(_identity_body_info(*FK_LINKS))
        incompatible = replace(
            self.manifest,
            compatible_for_save=False,
            incompatibilities=("forced mismatch",),
        )
        validator = PoseValidator(
            _AssetModelDouble(incompatible),
            fake_fk,
            DEFAULT_ROOT_POS,
        )
        with self.assertRaisesRegex(PoseValidationError, "forced mismatch"):
            validator.forward_links_robot_base(self.default_29, FK_LINKS)
        self.assertEqual(fake_fk.call_count, 0)

    def test_missing_requested_body_is_rejected(self):
        fake_fk = _CountingKinematics(_identity_body_info("present"))
        validator = PoseValidator(
            _AssetModelDouble(self.manifest),
            fake_fk,
            DEFAULT_ROOT_POS,
        )
        with self.assertRaisesRegex(PoseValidationError, "missing"):
            validator.forward_links_robot_base(self.default_29, ("missing",))

    def test_fk_lock_serializes_two_threads(self):
        start_barrier = threading.Barrier(3)
        counter_lock = threading.Lock()

        class BarrierBackedKinematics:
            def __init__(self):
                self.active = 0
                self.max_active = 0

            def forward(self, *_args, **_kwargs):
                with counter_lock:
                    self.active += 1
                    self.max_active = max(self.max_active, self.active)
                time.sleep(0.05)
                with counter_lock:
                    self.active -= 1
                return _identity_body_info("child"), None

        fake_fk = BarrierBackedKinematics()
        validator = PoseValidator(
            _AssetModelDouble(self.manifest),
            fake_fk,
            DEFAULT_ROOT_POS,
        )
        errors = []

        def run_forward():
            try:
                start_barrier.wait()
                validator.forward_links_robot_base(
                    self.default_29, ("child",)
                )
            except Exception as exc:  # pragma: no cover - failure evidence
                errors.append(exc)

        threads = [threading.Thread(target=run_forward) for _ in range(2)]
        for thread in threads:
            thread.start()
        start_barrier.wait()
        for thread in threads:
            thread.join(timeout=2.0)

        self.assertEqual((errors, fake_fk.max_active), ([], 1))


if __name__ == "__main__":
    unittest.main()
