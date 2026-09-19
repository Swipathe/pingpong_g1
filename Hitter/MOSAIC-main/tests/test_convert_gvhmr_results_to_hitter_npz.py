import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "tools"))

import convert_gvhmr_results_to_hitter_npz as converter


class RightArmPoseCalibrationTests(unittest.TestCase):
    def test_apply_calibration_preserves_motion_deltas(self):
        dof_pos = np.zeros((3, len(converter.ISAAC_JOINT_NAMES)), dtype=np.float32)
        arm_indices = [
            converter.ISAAC_JOINT_NAMES.index(name)
            for name in converter.FOREHAND_RIGHT_ARM_SCALE_JOINT_NAMES
        ]
        dof_pos[:, arm_indices] = np.asarray(
            [
                [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
                [0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8],
                [0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1],
            ],
            dtype=np.float32,
        )
        original_deltas = np.diff(dof_pos[:, arm_indices], axis=0)
        target_pose = np.asarray([-0.4, -0.3, -0.2, -0.1, 0.0, 0.1, 0.2], dtype=np.float32)

        calibrated, offsets = converter.apply_right_arm_strike_pose_calibration(
            dof_pos,
            reference_strike_frame=1,
            target_arm_pose=target_pose,
        )

        np.testing.assert_allclose(calibrated[1, arm_indices], target_pose, atol=1.0e-6)
        np.testing.assert_allclose(
            np.diff(calibrated[:, arm_indices], axis=0), original_deltas, atol=1.0e-6
        )
        np.testing.assert_allclose(calibrated[:, :12], dof_pos[:, :12], atol=1.0e-6)
        np.testing.assert_allclose(offsets, target_pose - dof_pos[1, arm_indices], atol=1.0e-6)

    def test_load_reference_poses_separates_forehand_and_backhand(self):
        arm_indices = [
            converter.ISAAC_JOINT_NAMES.index(name)
            for name in converter.FOREHAND_RIGHT_ARM_SCALE_JOINT_NAMES
        ]
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            for label, values in (("forehand", (1.0, 3.0)), ("backhand", (5.0, 7.0))):
                output_dir = root / label
                output_dir.mkdir()
                for file_index, value in enumerate(values):
                    joint_pos = np.zeros((4, len(converter.ISAAC_JOINT_NAMES)), dtype=np.float32)
                    joint_pos[2, arm_indices] = value
                    np.savez(output_dir / f"{label}_{file_index}.npz", joint_pos=joint_pos)

            poses = converter.load_reference_strike_arm_poses(root, reference_strike_frame=2)

        np.testing.assert_allclose(poses["forehand"], 2.0)
        np.testing.assert_allclose(poses["backhand"], 6.0)

    def test_reference_target_is_only_selected_for_enabled_strokes(self):
        targets = {
            "forehand": np.ones(7, dtype=np.float32),
            "backhand": np.full(7, 2.0, dtype=np.float32),
        }

        forehand = converter.select_arm_pose_target(
            targets,
            strike_type="forehand",
            enabled_strokes=("forehand",),
        )
        backhand = converter.select_arm_pose_target(
            targets,
            strike_type="backhand",
            enabled_strokes=("forehand",),
        )

        np.testing.assert_allclose(forehand, targets["forehand"])
        self.assertIsNone(backhand)

    def test_named_joint_offsets_change_only_requested_joints(self):
        dof_pos = np.zeros((2, len(converter.ISAAC_JOINT_NAMES)), dtype=np.float32)
        offsets = {
            "right_shoulder_pitch_joint": -0.25,
            "right_elbow_joint": 0.57,
            "right_wrist_pitch_joint": 0.26,
        }

        adjusted = converter.apply_named_joint_offsets(dof_pos, offsets)

        for joint_name, offset in offsets.items():
            joint_index = converter.ISAAC_JOINT_NAMES.index(joint_name)
            np.testing.assert_allclose(adjusted[:, joint_index], offset)
        unchanged_index = converter.ISAAC_JOINT_NAMES.index("left_elbow_joint")
        np.testing.assert_allclose(adjusted[:, unchanged_index], 0.0)


class GMRInitializationTests(unittest.TestCase):
    def test_load_gmr_initial_qpos_maps_isaac_joints_to_gmr_order(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            joint_pos = np.zeros((2, len(converter.ISAAC_JOINT_NAMES)), dtype=np.float32)
            joint_pos[0] = np.arange(len(converter.ISAAC_JOINT_NAMES), dtype=np.float32)
            body_pos_w = np.zeros((2, len(converter.ISAAC_BODY_NAMES), 3), dtype=np.float32)
            body_pos_w[0, 0] = [1.0, 2.0, 3.0]
            body_quat_w = np.zeros((2, len(converter.ISAAC_BODY_NAMES), 4), dtype=np.float32)
            body_quat_w[0, 0] = [1.0, 0.0, 0.0, 0.0]
            np.savez(root / "forehand_example.npz", joint_pos=joint_pos, body_pos_w=body_pos_w, body_quat_w=body_quat_w)
            np.savez(root / "backhand_example.npz", joint_pos=joint_pos, body_pos_w=body_pos_w, body_quat_w=body_quat_w)

            initial = converter.load_gmr_initial_qpos_by_stroke(root)

        qpos = initial["forehand"]
        self.assertEqual(qpos.shape, (7 + len(converter.GMR_JOINT_NAMES),))
        np.testing.assert_allclose(qpos[:3], [1.0, 2.0, 3.0])
        np.testing.assert_allclose(qpos[3:7], [1.0, 0.0, 0.0, 0.0])
        expected = joint_pos[0][converter.ISAAC_TO_GMR_JOINT_INDICES]
        np.testing.assert_allclose(qpos[7:], expected)


if __name__ == "__main__":
    unittest.main()
