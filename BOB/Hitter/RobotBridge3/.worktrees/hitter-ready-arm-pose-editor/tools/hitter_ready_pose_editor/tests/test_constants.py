import math
import unittest

import yaml

from tools.hitter_ready_pose_editor.constants import (
    ARM_JOINT_NAMES,
    MAX_MESH_BYTES,
    MAX_REQUEST_BODY_BYTES,
    MAX_STL_TRIANGLES,
    MOTION_NPZ_ARM_INDICES,
    ROBOT29_ARM_INDICES,
    centered_soft_limit,
)


class ConstantsTest(unittest.TestCase):
    def test_runtime_yaml_version_matches_declared_dependency(self):
        self.assertEqual(yaml.__version__, "6.0.3")

    def test_transport_and_mesh_limits_are_fixed(self):
        self.assertEqual(MAX_REQUEST_BODY_BYTES, 256 * 1024)
        self.assertEqual(MAX_MESH_BYTES, 64 * 1024 * 1024)
        self.assertEqual(MAX_STL_TRIANGLES, 1_000_000)

    def test_arm_order_and_index_contracts_are_exact(self):
        self.assertEqual(len(ARM_JOINT_NAMES), 14)
        self.assertEqual(ROBOT29_ARM_INDICES, tuple(range(15, 29)))
        self.assertEqual(
            MOTION_NPZ_ARM_INDICES,
            (11, 15, 19, 21, 23, 25, 27, 12, 16, 20, 22, 24, 26, 28),
        )
        self.assertEqual(ARM_JOINT_NAMES[0], "left_shoulder_pitch_joint")
        self.assertEqual(ARM_JOINT_NAMES[6], "left_wrist_yaw_joint")
        self.assertEqual(ARM_JOINT_NAMES[7], "right_shoulder_pitch_joint")
        self.assertEqual(ARM_JOINT_NAMES[13], "right_wrist_yaw_joint")

    def test_soft_limit_shrinks_about_range_midpoint(self):
        lower, upper = centered_soft_limit(-1.0, 3.0)
        self.assertTrue(math.isclose(lower, -0.8))
        self.assertTrue(math.isclose(upper, 2.8))

    def test_invalid_hard_limit_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "lower must be smaller"):
            centered_soft_limit(1.0, 1.0)
