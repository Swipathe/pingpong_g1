#!/usr/bin/env python3
"""Convert GVHMR predictions into MOSAIC-compatible HITTER G1 npz files."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_ROOT = REPO_ROOT / "scripts"
DEFAULT_GMR_ROOT = REPO_ROOT.parent / "GMR-master"
DEFAULT_SMPLX_ROOT = DEFAULT_GMR_ROOT / "assets" / "body_models"
DEFAULT_CAPTURE_ROOT = REPO_ROOT / "data" / "hitter_captures" / "mqy_capture"
DEFAULT_INPUT_ROOT = DEFAULT_CAPTURE_ROOT / "gvhmr_results"
DEFAULT_CLIP_ROOT = DEFAULT_CAPTURE_ROOT / "manual_clips_review"
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "data" / "hitter_motions" / "mqy_g1_npz_arm_pose_corrected_v3_pre_align"
DEFAULT_ARM_POSE_REFERENCE_ROOT = (
    REPO_ROOT
    / "data"
    / "hitter_motions"
    / "wechat_20260608_g1_npz_strike43_fixed_racket_roll055_arm155_peak43_aligned"
)

if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

import build_bvh1h_npz as bvh1h_npz  # noqa: E402
from build_bvh1h_npz import (  # noqa: E402
    angular_velocity_from_quaternions_xyzw,
    finite_difference,
    init_worker,
    normalize_quaternions_xyzw,
    save_npz,
    wxyz_to_xyzw,
    xyzw_to_wxyz,
)


GMR_JOINT_NAMES = [
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]

ISAAC_JOINT_NAMES = [
    "left_hip_pitch_joint",
    "right_hip_pitch_joint",
    "waist_yaw_joint",
    "left_hip_roll_joint",
    "right_hip_roll_joint",
    "waist_roll_joint",
    "left_hip_yaw_joint",
    "right_hip_yaw_joint",
    "waist_pitch_joint",
    "left_knee_joint",
    "right_knee_joint",
    "left_shoulder_pitch_joint",
    "right_shoulder_pitch_joint",
    "left_ankle_pitch_joint",
    "right_ankle_pitch_joint",
    "left_shoulder_roll_joint",
    "right_shoulder_roll_joint",
    "left_ankle_roll_joint",
    "right_ankle_roll_joint",
    "left_shoulder_yaw_joint",
    "right_shoulder_yaw_joint",
    "left_elbow_joint",
    "right_elbow_joint",
    "left_wrist_roll_joint",
    "right_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "right_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_wrist_yaw_joint",
]

ISAAC_BODY_NAMES = [
    "pelvis",
    "left_hip_pitch_link",
    "right_hip_pitch_link",
    "waist_yaw_link",
    "left_hip_roll_link",
    "right_hip_roll_link",
    "waist_roll_link",
    "left_hip_yaw_link",
    "right_hip_yaw_link",
    "torso_link",
    "left_knee_link",
    "right_knee_link",
    "left_shoulder_pitch_link",
    "right_shoulder_pitch_link",
    "left_ankle_pitch_link",
    "right_ankle_pitch_link",
    "left_shoulder_roll_link",
    "right_shoulder_roll_link",
    "left_ankle_roll_link",
    "right_ankle_roll_link",
    "left_shoulder_yaw_link",
    "right_shoulder_yaw_link",
    "left_elbow_link",
    "right_elbow_link",
    "left_wrist_roll_link",
    "right_wrist_roll_link",
    "left_wrist_pitch_link",
    "right_wrist_pitch_link",
    "left_wrist_yaw_link",
    "right_wrist_yaw_link",
    "right_racket_link",
]

LOWER_BODY_JOINT_NAMES = [
    name
    for name in ISAAC_JOINT_NAMES
    if "_hip_" in name or "_knee_" in name or "_ankle_" in name
]
WAIST_JOINT_NAMES = ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"]
FOREHAND_RIGHT_ARM_SCALE_JOINT_NAMES = [
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]
DEFAULT_STABLE_JOINT_POS = {
    "left_hip_pitch_joint": -0.312,
    "right_hip_pitch_joint": -0.312,
    "left_knee_joint": 0.669,
    "right_knee_joint": 0.669,
    "left_ankle_pitch_joint": -0.363,
    "right_ankle_pitch_joint": -0.363,
}
DEFAULT_REFERENCE_ROOT = REPO_ROOT / "data" / "hitter_motions" / "iphone_manual_hitter_g1_npz_strike43_clean"
DEFAULT_HITTER_URDF_PATH = (
    REPO_ROOT
    / "source"
    / "whole_body_tracking"
    / "whole_body_tracking"
    / "assets"
    / "unitree_description"
    / "urdf"
    / "g1_hitter_racket"
    / "main.urdf"
)
DEFAULT_URDF_PATH = DEFAULT_HITTER_URDF_PATH
GMR_TO_ISAAC_JOINT_INDICES = [GMR_JOINT_NAMES.index(name) for name in ISAAC_JOINT_NAMES]
ISAAC_TO_GMR_JOINT_INDICES = np.argsort(GMR_TO_ISAAC_JOINT_INDICES)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input-root",
        default=str(DEFAULT_INPUT_ROOT),
        help="Directory containing GVHMR output folders.",
    )
    parser.add_argument(
        "--clip-root",
        default=str(DEFAULT_CLIP_ROOT),
        help="Optional classified clip root for labels and source paths.",
    )
    parser.add_argument(
        "--output-root",
        default=str(DEFAULT_OUTPUT_ROOT),
        help="Directory where HITTER npz files are written.",
    )
    parser.add_argument("--gmr-root", default=str(DEFAULT_GMR_ROOT), help="Path to GMR-master.")
    parser.add_argument("--smplx-root", default=str(DEFAULT_SMPLX_ROOT), help="Path containing smplx/SMPLX_NEUTRAL.npz.")
    parser.add_argument("--urdf-path", default=str(DEFAULT_URDF_PATH), help="G1 URDF for pybullet FK.")
    parser.add_argument("--target-fps", type=float, default=50.0, help="Output fps.")
    parser.add_argument("--target-frames", type=int, default=94, help="Output frame count.")
    parser.add_argument("--reference-strike-frame", type=int, default=43, help="Reference hit frame.")
    parser.add_argument("--robot", default="unitree_g1", help="GMR target robot.")
    parser.add_argument(
        "--root-xy-mode",
        choices=("keep", "zero_start"),
        default="zero_start",
        help="How to normalize reference root x/y translation.",
    )
    parser.add_argument(
        "--target-root-height",
        type=float,
        default=0.78,
        help="Shift root z so the mean pelvis height matches this value. Use a negative value to keep raw height.",
    )
    parser.add_argument(
        "--min-body-z",
        type=float,
        default=0.025,
        help="After FK, shift the whole motion up if any saved body origin is below this height.",
    )
    parser.add_argument(
        "--lower-body-mode",
        choices=("keep", "reference_mean", "default"),
        default="reference_mean",
        help="How to fill hip/knee/ankle joints. GVHMR lower body is often unreliable for cropped racket videos.",
    )
    parser.add_argument(
        "--waist-mode",
        choices=("keep", "reference_mean", "zero"),
        default="keep",
        help="How to fill waist joints.",
    )
    parser.add_argument(
        "--reference-root",
        default=str(DEFAULT_REFERENCE_ROOT),
        help="Existing MOSAIC reference directory used for reference_mean joint baselines.",
    )
    parser.add_argument(
        "--gmr-init-reference-root",
        default="",
        help=(
            "Optional classified MOSAIC NPZ directory used to seed GMR's initial IK pose "
            "per stroke. This helps avoid mirrored/local-minimum arm solutions."
        ),
    )
    parser.add_argument(
        "--arm-pose-reference-root",
        default=str(DEFAULT_ARM_POSE_REFERENCE_ROOT),
        help=(
            "Classified reference motion directory used to match the seven right-arm joints "
            "at reference-strike-frame. Pass an empty string to disable this calibration."
        ),
    )
    parser.add_argument(
        "--arm-pose-calibration-strokes",
        nargs="+",
        choices=("forehand", "backhand"),
        default=("forehand", "backhand"),
        help="Stroke classes that use reference right-arm strike-pose calibration.",
    )
    parser.add_argument(
        "--backhand-right-shoulder-pitch-offset",
        type=float,
        default=0.0,
        help="Additional right shoulder pitch offset in radians applied only to inferred backhand clips.",
    )
    parser.add_argument(
        "--backhand-right-shoulder-roll-offset",
        type=float,
        default=0.0,
        help="Additional right shoulder roll offset in radians applied only to inferred backhand clips.",
    )
    parser.add_argument(
        "--backhand-right-shoulder-yaw-offset",
        type=float,
        default=0.0,
        help="Additional right shoulder yaw offset in radians applied only to inferred backhand clips.",
    )
    parser.add_argument(
        "--backhand-right-elbow-offset",
        type=float,
        default=0.0,
        help="Additional right elbow offset in radians applied only to inferred backhand clips.",
    )
    parser.add_argument(
        "--backhand-right-wrist-roll-offset",
        type=float,
        default=0.0,
        help="Additional right wrist roll offset in radians applied only to inferred backhand clips.",
    )
    parser.add_argument(
        "--backhand-right-wrist-pitch-offset",
        type=float,
        default=0.0,
        help="Additional right wrist pitch offset in radians applied only to inferred backhand clips.",
    )
    parser.add_argument(
        "--backhand-right-wrist-yaw-offset",
        type=float,
        default=0.0,
        help="Additional right wrist yaw offset in radians applied only to inferred backhand clips.",
    )
    parser.add_argument(
        "--forehand-right-arm-motion-scale",
        type=float,
        default=1.0,
        help=(
            "Scale forehand right-arm joint motion around the strike frame. "
            "The strike pose is preserved while backswing/follow-through range changes."
        ),
    )
    parser.add_argument("--compressed", action="store_true", help="Use np.savez_compressed.")
    parser.add_argument("--overwrite", action="store_true", help="Overwrite existing npz files.")
    return parser.parse_args()


def add_gmr_to_path(gmr_root: Path) -> None:
    if str(gmr_root) not in sys.path:
        sys.path.insert(0, str(gmr_root))


def load_gvhmr_frames(gvhmr_result: Path, smplx_root: Path, target_fps: float):
    from general_motion_retargeting.utils.smpl import get_gvhmr_data_offline_fast, load_gvhmr_pred_file

    smplx_data, body_model, smplx_output, human_height = load_gvhmr_pred_file(str(gvhmr_result), smplx_root)
    frames, aligned_fps = get_gvhmr_data_offline_fast(
        smplx_data,
        body_model,
        smplx_output,
        tgt_fps=target_fps,
    )
    return frames, float(aligned_fps), float(human_height)


def resample_rows(values: np.ndarray, target_frames: int) -> np.ndarray:
    values = np.asarray(values, dtype=np.float32)
    if values.shape[0] == target_frames:
        return values.astype(np.float32)
    if values.shape[0] <= 0:
        raise ValueError("Cannot resample an empty sequence.")
    if values.shape[0] == 1:
        return np.repeat(values, target_frames, axis=0).astype(np.float32)

    source_x = np.linspace(0.0, 1.0, values.shape[0], dtype=np.float64)
    target_x = np.linspace(0.0, 1.0, target_frames, dtype=np.float64)
    flat = values.reshape(values.shape[0], -1)
    out = np.empty((target_frames, flat.shape[1]), dtype=np.float32)
    for col in range(flat.shape[1]):
        out[:, col] = np.interp(target_x, source_x, flat[:, col]).astype(np.float32)
    return out.reshape((target_frames,) + values.shape[1:]).astype(np.float32)


def normalize_quat_wxyz(quaternions: np.ndarray) -> np.ndarray:
    quaternions = np.asarray(quaternions, dtype=np.float32)
    norms = np.linalg.norm(quaternions, axis=-1, keepdims=True)
    norms = np.maximum(norms, 1.0e-8)
    return (quaternions / norms).astype(np.float32)


def standardize_qpos(qpos: np.ndarray, target_frames: int) -> np.ndarray:
    qpos = np.asarray(qpos, dtype=np.float32)
    qpos = resample_rows(qpos, target_frames)
    qpos[:, 3:7] = normalize_quat_wxyz(qpos[:, 3:7])
    return qpos.astype(np.float32)


def load_reference_joint_mean(reference_root: Path | None) -> np.ndarray | None:
    if reference_root is None or not reference_root.exists():
        return None
    files = sorted(path for path in reference_root.rglob("*.npz") if path.is_file())
    if not files:
        return None
    joint_arrays = []
    for path in files:
        with np.load(path) as data:
            joint_pos = np.asarray(data["joint_pos"], dtype=np.float32)
        if joint_pos.ndim == 2 and joint_pos.shape[1] == len(ISAAC_JOINT_NAMES):
            joint_arrays.append(joint_pos)
    if not joint_arrays:
        return None
    return np.concatenate(joint_arrays, axis=0).mean(axis=0).astype(np.float32)


def load_gmr_initial_qpos_by_stroke(reference_root: Path | None) -> dict[str, np.ndarray]:
    """Load one validated G1 pose per stroke and convert it to GMR qpos order."""
    if reference_root is None or not reference_root.exists():
        return {}

    paths_by_stroke: dict[str, list[Path]] = {"forehand": [], "backhand": []}
    for path in sorted(reference_root.rglob("*.npz")):
        stroke = infer_label(path.name)
        if stroke in paths_by_stroke:
            paths_by_stroke[stroke].append(path)

    initial_by_stroke: dict[str, np.ndarray] = {}
    for stroke, paths in paths_by_stroke.items():
        if not paths:
            raise ValueError(f"No {stroke} NPZ found under GMR init reference root {reference_root}")
        path = paths[0]
        with np.load(path) as data:
            joint_pos = np.asarray(data["joint_pos"], dtype=np.float32)
            body_pos_w = np.asarray(data["body_pos_w"], dtype=np.float32)
            body_quat_w = np.asarray(data["body_quat_w"], dtype=np.float32)
        if joint_pos.ndim != 2 or joint_pos.shape[1] != len(ISAAC_JOINT_NAMES):
            raise ValueError(f"Invalid joint_pos shape in GMR init reference: {path}: {joint_pos.shape}")
        if body_pos_w.ndim != 3 or body_pos_w.shape[1] <= 0 or body_pos_w.shape[2] != 3:
            raise ValueError(f"Invalid body_pos_w shape in GMR init reference: {path}: {body_pos_w.shape}")
        if body_quat_w.ndim != 3 or body_quat_w.shape[1] <= 0 or body_quat_w.shape[2] != 4:
            raise ValueError(f"Invalid body_quat_w shape in GMR init reference: {path}: {body_quat_w.shape}")

        qpos = np.empty((7 + len(GMR_JOINT_NAMES),), dtype=np.float32)
        qpos[:3] = body_pos_w[0, 0]
        qpos[3:7] = normalize_quat_wxyz(body_quat_w[0, 0][None, :])[0]
        qpos[7:] = joint_pos[0, ISAAC_TO_GMR_JOINT_INDICES]
        initial_by_stroke[stroke] = qpos

    return initial_by_stroke


def stable_joint_value(name: str, reference_mean: np.ndarray | None) -> float:
    joint_index = ISAAC_JOINT_NAMES.index(name)
    if reference_mean is not None:
        return float(reference_mean[joint_index])
    return float(DEFAULT_STABLE_JOINT_POS.get(name, 0.0))


def load_reference_strike_arm_poses(
    reference_root: Path,
    *,
    reference_strike_frame: int,
) -> dict[str, np.ndarray]:
    reference_root = Path(reference_root)
    if not reference_root.is_dir():
        raise FileNotFoundError(f"Invalid arm-pose reference directory: {reference_root}")

    arm_indices = [ISAAC_JOINT_NAMES.index(name) for name in FOREHAND_RIGHT_ARM_SCALE_JOINT_NAMES]
    poses_by_label: dict[str, list[np.ndarray]] = {"forehand": [], "backhand": []}
    for path in sorted(reference_root.rglob("*.npz")):
        label = infer_label(path.name)
        if label not in poses_by_label:
            continue
        with np.load(path) as data:
            joint_pos = np.asarray(data["joint_pos"], dtype=np.float32)
        if joint_pos.ndim != 2 or joint_pos.shape[1] != len(ISAAC_JOINT_NAMES):
            raise ValueError(f"Invalid joint_pos shape in arm-pose reference: {path}: {joint_pos.shape}")
        frame_index = max(0, min(int(reference_strike_frame), joint_pos.shape[0] - 1))
        poses_by_label[label].append(joint_pos[frame_index, arm_indices].copy())

    missing_labels = [label for label, poses in poses_by_label.items() if not poses]
    if missing_labels:
        raise ValueError(f"No arm-pose references found for classes {missing_labels} under {reference_root}")
    return {
        label: np.mean(np.stack(poses, axis=0), axis=0).astype(np.float32)
        for label, poses in poses_by_label.items()
    }


def apply_right_arm_strike_pose_calibration(
    dof_pos: np.ndarray,
    *,
    reference_strike_frame: int,
    target_arm_pose: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    dof_pos = np.asarray(dof_pos, dtype=np.float32).copy()
    target_arm_pose = np.asarray(target_arm_pose, dtype=np.float32)
    arm_indices = [ISAAC_JOINT_NAMES.index(name) for name in FOREHAND_RIGHT_ARM_SCALE_JOINT_NAMES]
    if target_arm_pose.shape != (len(arm_indices),):
        raise ValueError(
            f"Expected target arm pose shape {(len(arm_indices),)}, got {target_arm_pose.shape}"
        )

    strike_frame = max(0, min(int(reference_strike_frame), dof_pos.shape[0] - 1))
    offsets = target_arm_pose - dof_pos[strike_frame, arm_indices]
    dof_pos[:, arm_indices] += offsets[None, :]
    return dof_pos.astype(np.float32), offsets.astype(np.float32)


def select_arm_pose_target(
    targets: dict[str, np.ndarray],
    *,
    strike_type: str,
    enabled_strokes: tuple[str, ...] | list[str],
) -> np.ndarray | None:
    if strike_type not in enabled_strokes:
        return None
    return targets.get(strike_type)


def apply_named_joint_offsets(
    dof_pos: np.ndarray,
    offsets: dict[str, float],
) -> np.ndarray:
    dof_pos = np.asarray(dof_pos, dtype=np.float32).copy()
    for joint_name, offset in offsets.items():
        if offset != 0.0:
            dof_pos[:, ISAAC_JOINT_NAMES.index(joint_name)] += float(offset)
    return dof_pos.astype(np.float32)


def apply_joint_stabilization(
    dof_pos: np.ndarray,
    *,
    lower_body_mode: str,
    waist_mode: str,
    reference_mean: np.ndarray | None,
) -> np.ndarray:
    dof_pos = np.asarray(dof_pos, dtype=np.float32).copy()

    if lower_body_mode != "keep":
        for name in LOWER_BODY_JOINT_NAMES:
            joint_index = ISAAC_JOINT_NAMES.index(name)
            if lower_body_mode == "default":
                dof_pos[:, joint_index] = float(DEFAULT_STABLE_JOINT_POS.get(name, 0.0))
            else:
                dof_pos[:, joint_index] = stable_joint_value(name, reference_mean)

    if waist_mode != "keep":
        for name in WAIST_JOINT_NAMES:
            joint_index = ISAAC_JOINT_NAMES.index(name)
            dof_pos[:, joint_index] = 0.0 if waist_mode == "zero" else stable_joint_value(name, reference_mean)

    return dof_pos.astype(np.float32)


def apply_forehand_right_arm_motion_scale(
    dof_pos: np.ndarray,
    *,
    reference_strike_frame: int,
    motion_scale: float,
) -> np.ndarray:
    dof_pos = np.asarray(dof_pos, dtype=np.float32).copy()
    if motion_scale == 1.0:
        return dof_pos

    anchor_frame = max(0, min(int(reference_strike_frame), dof_pos.shape[0] - 1))
    joint_indices = [ISAAC_JOINT_NAMES.index(name) for name in FOREHAND_RIGHT_ARM_SCALE_JOINT_NAMES]
    anchor = dof_pos[anchor_frame : anchor_frame + 1, joint_indices]
    dof_pos[:, joint_indices] = anchor + (dof_pos[:, joint_indices] - anchor) * float(motion_scale)
    return dof_pos.astype(np.float32)


def compute_body_state_sequences_for_isaac(
    root_pos: np.ndarray,
    root_rot_wxyz: np.ndarray,
    dof_pos_isaac: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    worker_context = bvh1h_npz._WORKER_CONTEXT
    pybullet = worker_context["pybullet"]
    physics_client_id = worker_context["physics_client_id"]
    robot_id = worker_context["robot_id"]

    joint_infos = [
        pybullet.getJointInfo(robot_id, joint_idx, physicsClientId=physics_client_id)
        for joint_idx in range(pybullet.getNumJoints(robot_id, physicsClientId=physics_client_id))
    ]
    joint_name_to_index = {info[1].decode("utf-8"): info[0] for info in joint_infos}
    link_name_to_index = {info[12].decode("utf-8"): info[0] for info in joint_infos}

    missing_joints = [name for name in ISAAC_JOINT_NAMES if name not in joint_name_to_index]
    missing_links = [name for name in ISAAC_BODY_NAMES[1:] if name not in link_name_to_index]
    if missing_joints or missing_links:
        raise KeyError(f"FK URDF is missing joints={missing_joints} links={missing_links}")

    actuated_joint_indices = [joint_name_to_index[name] for name in ISAAC_JOINT_NAMES]
    body_link_indices = [link_name_to_index[name] for name in ISAAC_BODY_NAMES[1:]]
    root_rot_xyzw = normalize_quaternions_xyzw(wxyz_to_xyzw(root_rot_wxyz))

    num_frames = root_pos.shape[0]
    num_bodies = len(ISAAC_BODY_NAMES)
    body_pos_w = np.zeros((num_frames, num_bodies, 3), dtype=np.float32)
    body_quat_xyzw = np.zeros((num_frames, num_bodies, 4), dtype=np.float32)

    for frame_idx in range(num_frames):
        pybullet.resetBasePositionAndOrientation(
            robot_id,
            root_pos[frame_idx].tolist(),
            root_rot_xyzw[frame_idx].tolist(),
            physicsClientId=physics_client_id,
        )
        for joint_index, joint_position in zip(actuated_joint_indices, dof_pos_isaac[frame_idx], strict=True):
            pybullet.resetJointState(
                robot_id,
                joint_index,
                float(joint_position),
                targetVelocity=0.0,
                physicsClientId=physics_client_id,
            )

        body_pos_w[frame_idx, 0] = root_pos[frame_idx]
        body_quat_xyzw[frame_idx, 0] = root_rot_xyzw[frame_idx]
        for body_idx, link_index in enumerate(body_link_indices, start=1):
            link_state = pybullet.getLinkState(
                robot_id,
                link_index,
                computeForwardKinematics=True,
                computeLinkVelocity=False,
                physicsClientId=physics_client_id,
            )
            body_pos_w[frame_idx, body_idx] = np.asarray(link_state[4], dtype=np.float32)
            body_quat_xyzw[frame_idx, body_idx] = np.asarray(link_state[5], dtype=np.float32)

    body_quat_xyzw = normalize_quaternions_xyzw(body_quat_xyzw.reshape(-1, 4)).reshape(num_frames, num_bodies, 4)
    return body_pos_w, body_quat_xyzw


def convert_one(
    gvhmr_result: Path,
    output_path: Path,
    *,
    strike_type: str,
    target_fps: float,
    target_frames: int,
    robot: str,
    smplx_root: Path,
    root_xy_mode: str,
    target_root_height: float,
    min_body_z: float,
    lower_body_mode: str,
    waist_mode: str,
    reference_mean: np.ndarray | None,
    gmr_init_qpos: np.ndarray | None,
    backhand_right_shoulder_pitch_offset: float,
    backhand_right_shoulder_roll_offset: float,
    backhand_right_shoulder_yaw_offset: float,
    backhand_right_elbow_offset: float,
    backhand_right_wrist_roll_offset: float,
    backhand_right_wrist_pitch_offset: float,
    backhand_right_wrist_yaw_offset: float,
    reference_strike_frame: int,
    forehand_right_arm_motion_scale: float,
    target_arm_pose: np.ndarray | None,
    compressed: bool,
) -> dict[str, object]:
    from general_motion_retargeting import GeneralMotionRetargeting as GMR

    frames, aligned_fps, human_height = load_gvhmr_frames(gvhmr_result, smplx_root, target_fps)
    retargeter = GMR(
        actual_human_height=human_height,
        src_human="smplx",
        tgt_robot=robot,
        verbose=False,
    )
    if gmr_init_qpos is not None:
        if gmr_init_qpos.shape != retargeter.configuration.data.qpos.shape:
            raise ValueError(
                f"Invalid GMR init qpos shape {gmr_init_qpos.shape}; "
                f"expected {retargeter.configuration.data.qpos.shape}"
            )
        retargeter.configuration.data.qpos[:] = gmr_init_qpos

    qpos = np.asarray([retargeter.retarget(frame) for frame in frames], dtype=np.float32)
    qpos = standardize_qpos(qpos, target_frames)

    root_pos = qpos[:, :3].astype(np.float32)
    if root_xy_mode == "zero_start":
        root_pos[:, :2] -= root_pos[0:1, :2]
    if target_root_height >= 0.0:
        root_pos[:, 2] += float(target_root_height) - float(np.mean(root_pos[:, 2]))
    root_rot_wxyz = qpos[:, 3:7].astype(np.float32)
    dof_pos_gmr = qpos[:, 7:].astype(np.float32)
    dof_pos = dof_pos_gmr[:, GMR_TO_ISAAC_JOINT_INDICES]
    dof_pos = apply_joint_stabilization(
        dof_pos,
        lower_body_mode=lower_body_mode,
        waist_mode=waist_mode,
        reference_mean=reference_mean,
    )
    backhand_arm_offsets = {
        "right_shoulder_pitch_joint": float(backhand_right_shoulder_pitch_offset),
        "right_shoulder_roll_joint": float(backhand_right_shoulder_roll_offset),
        "right_shoulder_yaw_joint": float(backhand_right_shoulder_yaw_offset),
        "right_elbow_joint": float(backhand_right_elbow_offset),
        "right_wrist_roll_joint": float(backhand_right_wrist_roll_offset),
        "right_wrist_pitch_joint": float(backhand_right_wrist_pitch_offset),
        "right_wrist_yaw_joint": float(backhand_right_wrist_yaw_offset),
    }
    if strike_type == "backhand":
        dof_pos = apply_named_joint_offsets(dof_pos, backhand_arm_offsets)
    if strike_type == "forehand" and forehand_right_arm_motion_scale != 1.0:
        dof_pos = apply_forehand_right_arm_motion_scale(
            dof_pos,
            reference_strike_frame=reference_strike_frame,
            motion_scale=forehand_right_arm_motion_scale,
        )
    arm_pose_offsets: dict[str, float] = {}
    if target_arm_pose is not None:
        dof_pos, offsets = apply_right_arm_strike_pose_calibration(
            dof_pos,
            reference_strike_frame=reference_strike_frame,
            target_arm_pose=target_arm_pose,
        )
        arm_pose_offsets = {
            name: float(offset)
            for name, offset in zip(FOREHAND_RIGHT_ARM_SCALE_JOINT_NAMES, offsets, strict=True)
        }
    dt = 1.0 / target_fps

    body_pos_w, body_quat_xyzw = compute_body_state_sequences_for_isaac(root_pos, root_rot_wxyz, dof_pos)
    min_z_before = float(np.min(body_pos_w[..., 2]))
    z_shift = 0.0
    if min_z_before < float(min_body_z):
        z_shift = float(min_body_z) - min_z_before
        body_pos_w[..., 2] += z_shift

    joint_vel = finite_difference(dof_pos, dt)
    body_lin_vel_w = finite_difference(body_pos_w, dt)
    body_ang_vel_w = np.stack(
        [angular_velocity_from_quaternions_xyzw(body_quat_xyzw[:, body_idx], dt) for body_idx in range(body_quat_xyzw.shape[1])],
        axis=1,
    ).astype(np.float32)
    body_quat_w = xyzw_to_wxyz(body_quat_xyzw)

    save_npz(
        output_path,
        compressed=compressed,
        fps=np.asarray([target_fps], dtype=np.float32),
        joint_pos=dof_pos.astype(np.float32),
        joint_vel=joint_vel.astype(np.float32),
        body_pos_w=body_pos_w.astype(np.float32),
        body_quat_w=body_quat_w.astype(np.float32),
        body_lin_vel_w=body_lin_vel_w.astype(np.float32),
        body_ang_vel_w=body_ang_vel_w.astype(np.float32),
    )
    return {
        "source_gvhmr_result": str(gvhmr_result.resolve()),
        "output_npz": str(output_path.resolve()),
        "source_frames": int(len(frames)),
        "aligned_fps_from_gvhmr_loader": aligned_fps,
        "output_fps": float(target_fps),
        "output_frames": int(target_frames),
        "human_height": human_height,
        "root_xy_mode": root_xy_mode,
        "target_root_height": None if target_root_height < 0.0 else float(target_root_height),
        "min_body_z": float(min_body_z),
        "min_body_z_before_shift": min_z_before,
        "z_shift": z_shift,
        "lower_body_mode": lower_body_mode,
        "waist_mode": waist_mode,
        "backhand_arm_offsets": backhand_arm_offsets if strike_type == "backhand" else {},
        "forehand_right_arm_motion_scale": (
            float(forehand_right_arm_motion_scale) if strike_type == "forehand" else 1.0
        ),
        "right_arm_strike_pose_offsets": arm_pose_offsets,
    }


def collect_results(input_root: Path) -> list[Path]:
    return sorted(path for path in input_root.rglob("hmr4d_results.pt") if path.is_file())


def infer_label(name: str) -> str:
    if name.startswith("forehand_"):
        return "forehand"
    if name.startswith("backhand_"):
        return "backhand"
    return "unknown"


def write_manifest(output_root: Path, rows: list[dict[str, object]]) -> None:
    index_dir = output_root / "_index"
    index_dir.mkdir(parents=True, exist_ok=True)
    payload = {
        "dataset_name": output_root.name,
        "motion_count": len(rows),
        "body_names": ISAAC_BODY_NAMES,
        "joint_names": ISAAC_JOINT_NAMES,
        "motions": rows,
    }
    with (index_dir / "gvhmr_hitter_npz_manifest.json").open("w") as f:
        json.dump(payload, f, indent=2)
    with (index_dir / "gvhmr_hitter_npz_manifest.csv").open("w", newline="") as f:
        fieldnames = sorted({key for row in rows for key in row.keys()})
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    args = parse_args()
    input_root = Path(args.input_root).resolve()
    output_root = Path(args.output_root).resolve()
    gmr_root = Path(args.gmr_root).resolve()
    smplx_root = Path(args.smplx_root).resolve()
    urdf_path = Path(args.urdf_path).resolve()
    reference_root = Path(args.reference_root).resolve() if args.reference_root else None
    gmr_init_reference_root = (
        Path(args.gmr_init_reference_root).resolve() if args.gmr_init_reference_root else None
    )
    arm_pose_reference_root = (
        Path(args.arm_pose_reference_root).resolve() if args.arm_pose_reference_root else None
    )

    add_gmr_to_path(gmr_root)
    init_worker(str(gmr_root), str(urdf_path))
    reference_mean = load_reference_joint_mean(reference_root)
    gmr_init_qpos_by_stroke = load_gmr_initial_qpos_by_stroke(gmr_init_reference_root)
    arm_pose_targets = (
        load_reference_strike_arm_poses(
            arm_pose_reference_root,
            reference_strike_frame=int(args.reference_strike_frame),
        )
        if arm_pose_reference_root is not None
        else {}
    )
    if args.lower_body_mode == "reference_mean" and reference_mean is None:
        print(f"[warn] reference mean unavailable under {reference_root}; falling back to default lower-body joints")
    if args.waist_mode == "reference_mean" and reference_mean is None:
        print(f"[warn] reference mean unavailable under {reference_root}; falling back to zero/default waist joints")

    results = collect_results(input_root)
    if not results:
        raise FileNotFoundError(f"No hmr4d_results.pt files found under {input_root}")

    rows: list[dict[str, object]] = []
    for result_path in results:
        motion_name = result_path.parent.name
        label = infer_label(motion_name)
        output_dir = output_root / label
        output_path = output_dir / f"{motion_name}__unitree_g1.npz"
        if output_path.exists() and not args.overwrite:
            rows.append(
                {
                    "motion_name": motion_name,
                    "strike_type": label,
                    "source_gvhmr_result": str(result_path.resolve()),
                    "output_npz": str(output_path.resolve()),
                    "skipped_existing": True,
                }
            )
            continue

        row = convert_one(
            result_path,
            output_path,
            strike_type=label,
            target_fps=float(args.target_fps),
            target_frames=int(args.target_frames),
            robot=args.robot,
            smplx_root=smplx_root,
            root_xy_mode=str(args.root_xy_mode),
            target_root_height=float(args.target_root_height),
            min_body_z=float(args.min_body_z),
            lower_body_mode=str(args.lower_body_mode),
            waist_mode=str(args.waist_mode),
            reference_mean=reference_mean,
            gmr_init_qpos=gmr_init_qpos_by_stroke.get(label),
            backhand_right_shoulder_pitch_offset=float(args.backhand_right_shoulder_pitch_offset),
            backhand_right_shoulder_roll_offset=float(args.backhand_right_shoulder_roll_offset),
            backhand_right_shoulder_yaw_offset=float(args.backhand_right_shoulder_yaw_offset),
            backhand_right_elbow_offset=float(args.backhand_right_elbow_offset),
            backhand_right_wrist_roll_offset=float(args.backhand_right_wrist_roll_offset),
            backhand_right_wrist_pitch_offset=float(args.backhand_right_wrist_pitch_offset),
            backhand_right_wrist_yaw_offset=float(args.backhand_right_wrist_yaw_offset),
            reference_strike_frame=int(args.reference_strike_frame),
            forehand_right_arm_motion_scale=float(args.forehand_right_arm_motion_scale),
            target_arm_pose=select_arm_pose_target(
                arm_pose_targets,
                strike_type=label,
                enabled_strokes=args.arm_pose_calibration_strokes,
            ),
            compressed=bool(args.compressed),
        )
        row.update(
            {
                "motion_name": motion_name,
                "strike_type": label,
                "reference_strike_frame": int(args.reference_strike_frame),
                "reference_strike_time_s": float(args.reference_strike_frame) / float(args.target_fps),
                "arm_pose_reference_root": (
                    str(arm_pose_reference_root) if arm_pose_reference_root is not None else None
                ),
                "arm_pose_calibration_enabled": label in args.arm_pose_calibration_strokes,
                "gmr_init_reference_root": (
                    str(gmr_init_reference_root) if gmr_init_reference_root is not None else None
                ),
                "gmr_init_enabled": label in gmr_init_qpos_by_stroke,
                "skipped_existing": False,
            }
        )
        rows.append(row)
        print(f"[OK] {motion_name} -> {output_path}")

    write_manifest(output_root, rows)
    print(f"[Done] wrote {len(rows)} motions to {output_root}")


if __name__ == "__main__":
    main()
