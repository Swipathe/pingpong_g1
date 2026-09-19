#!/usr/bin/env python3
"""Convert bvh1h BVH motions into MOSAIC-compatible G1 motion npz files."""

from __future__ import annotations

import argparse
import atexit
import csv
import json
import os
import re
import sys
import time
from collections.abc import Iterable
from concurrent.futures import ProcessPoolExecutor, as_completed
from contextlib import contextmanager
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = REPO_ROOT.parent
DEFAULT_INPUT_ROOT = Path("/home/sijie/Downloads/bvh1h")
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "data" / "motions" / "bvh1h_g1_npz"
DEFAULT_GMR_ROOT = WORKSPACE_ROOT / "GMR-master"
DEFAULT_URDF_PATH = DEFAULT_GMR_ROOT / "assets" / "unitree_g1" / "g1_custom_collision_29dof.urdf"
DEFAULT_DATASET_NAME = "bvh1h_g1_npz"

ACTUATED_JOINT_NAMES = [
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

BODY_LINK_NAMES = [
    "pelvis",
    "left_hip_pitch_link",
    "left_hip_roll_link",
    "left_hip_yaw_link",
    "left_knee_link",
    "left_ankle_pitch_link",
    "left_ankle_roll_link",
    "right_hip_pitch_link",
    "right_hip_roll_link",
    "right_hip_yaw_link",
    "right_knee_link",
    "right_ankle_pitch_link",
    "right_ankle_roll_link",
    "waist_yaw_link",
    "waist_roll_link",
    "torso_link",
    "left_shoulder_pitch_link",
    "left_shoulder_roll_link",
    "left_shoulder_yaw_link",
    "left_elbow_link",
    "left_wrist_roll_link",
    "left_wrist_pitch_link",
    "left_wrist_yaw_link",
    "right_shoulder_pitch_link",
    "right_shoulder_roll_link",
    "right_shoulder_yaw_link",
    "right_elbow_link",
    "right_wrist_roll_link",
    "right_wrist_pitch_link",
    "right_wrist_yaw_link",
]

README_TEXT = """# bvh1h_g1_npz

MOSAIC-compatible Unitree G1 motion npz files converted from `/home/sijie/Downloads/bvh1h`.

Coordinate handling:
- Source BVH is parsed as centimeters with Y-up.
- The conversion applies the same GMR BVH rotation matrix used by `load_bvh_file`: BVH Y-up -> MOSAIC/Isaac Z-up.
- Source 100 fps BVH is downsampled on the time axis to 30 fps before retargeting.
- The bvh1h skeleton uses `LeftToeBase`/`RightToeBase`; those are used for `LeftFootMod`/`RightFootMod`.
- The bvh1h skeleton has `Spine1` but no `Spine2`; `Spine2` is aliased to `Spine1` for the GMR IK config.
- A constant per-motion z shift is applied so the lowest generated G1 body point is on ground z=0.

Output fields:
- `fps`
- `joint_pos`, `joint_vel`
- `body_pos_w`, `body_quat_w`
- `body_lin_vel_w`, `body_ang_vel_w`

`body_quat_w` is saved in Isaac/MOSAIC `wxyz` order.
"""

_WORKER_CONTEXT: dict[str, object] = {}


@contextmanager
def _suppress_native_output():
    if os.environ.get("BVH1H_VERBOSE_NATIVE") == "1":
        yield
        return

    devnull_fd = os.open(os.devnull, os.O_WRONLY)
    saved_fds = []
    try:
        for stream in (sys.stdout, sys.stderr):
            try:
                stream.flush()
            except Exception:
                pass
        for fd in (1, 2):
            saved_fds.append(os.dup(fd))
            os.dup2(devnull_fd, fd)
        yield
    finally:
        for fd, saved_fd in zip((1, 2), saved_fds):
            os.dup2(saved_fd, fd)
            os.close(saved_fd)
        os.close(devnull_fd)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build MOSAIC-compatible G1 npz motions from bvh1h BVH files.")
    parser.add_argument("--input-root", default=str(DEFAULT_INPUT_ROOT), help="Directory containing bvh1h .bvh files.")
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT), help="Directory where npz files are written.")
    parser.add_argument("--gmr-root", default=str(DEFAULT_GMR_ROOT), help="Path to GMR-master.")
    parser.add_argument("--urdf-path", default=str(DEFAULT_URDF_PATH), help="G1 URDF for forward kinematics.")
    parser.add_argument("--dataset-name", default=DEFAULT_DATASET_NAME, help="Dataset name for the manifest.")
    parser.add_argument("--target-fps", type=float, default=30.0, help="Output motion fps.")
    parser.add_argument("--human-height", type=float, default=1.75, help="Human height passed to GMR.")
    parser.add_argument("--workers", type=int, default=min(max(os.cpu_count() or 1, 1), 4), help="Parallel workers.")
    parser.add_argument("--limit", type=int, default=0, help="Convert only first N files for smoke tests.")
    parser.add_argument("--overwrite", action="store_true", help="Overwrite existing npz files.")
    parser.add_argument("--compressed", action="store_true", help="Use np.savez_compressed.")
    parser.add_argument(
        "--no-ground-align",
        action="store_true",
        help="Disable constant z shift that puts the lowest G1 body point on z=0.",
    )
    return parser.parse_args()


def normalize_quaternions_xyzw(quaternions: np.ndarray) -> np.ndarray:
    quaternions = np.asarray(quaternions, dtype=np.float32)
    norms = np.linalg.norm(quaternions, axis=-1, keepdims=True)
    if np.any(norms <= 1e-8):
        raise ValueError("Encountered near-zero quaternion norm.")
    return quaternions / norms


def wxyz_to_xyzw(quaternions: np.ndarray) -> np.ndarray:
    quaternions = np.asarray(quaternions, dtype=np.float32)
    return np.concatenate([quaternions[..., 1:], quaternions[..., :1]], axis=-1).astype(np.float32)


def xyzw_to_wxyz(quaternions: np.ndarray) -> np.ndarray:
    quaternions = np.asarray(quaternions, dtype=np.float32)
    return np.concatenate([quaternions[..., 3:4], quaternions[..., :3]], axis=-1).astype(np.float32)


def finite_difference(values: np.ndarray, dt: float) -> np.ndarray:
    values = np.asarray(values, dtype=np.float32)
    if values.shape[0] <= 1:
        return np.zeros_like(values, dtype=np.float32)
    return np.gradient(values, dt, axis=0, edge_order=1).astype(np.float32)


def quat_multiply_xyzw(quat_a: np.ndarray, quat_b: np.ndarray) -> np.ndarray:
    ax, ay, az, aw = quat_a
    bx, by, bz, bw = quat_b
    return np.asarray(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ],
        dtype=np.float32,
    )


def quat_inverse_xyzw(quaternion: np.ndarray) -> np.ndarray:
    quaternion = np.asarray(quaternion, dtype=np.float32)
    norm_sq = float(np.dot(quaternion, quaternion))
    if norm_sq <= 1e-8:
        raise ValueError("Encountered near-zero quaternion norm.")
    return np.asarray([-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3]], dtype=np.float32) / norm_sq


def quat_to_rotvec_xyzw(quaternion: np.ndarray) -> np.ndarray:
    quaternion = normalize_quaternions_xyzw(np.asarray([quaternion], dtype=np.float32))[0]
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    xyz = quaternion[:3]
    sin_half_angle = float(np.linalg.norm(xyz))
    if sin_half_angle <= 1e-8:
        return (2.0 * xyz).astype(np.float32)
    half_angle = np.arctan2(sin_half_angle, float(quaternion[3]))
    axis = xyz / sin_half_angle
    return (axis * (2.0 * half_angle)).astype(np.float32)


def angular_velocity_from_quaternions_xyzw(quaternions: np.ndarray, dt: float) -> np.ndarray:
    quaternions = normalize_quaternions_xyzw(quaternions)
    num_frames = quaternions.shape[0]
    angular_velocity = np.zeros((num_frames, 3), dtype=np.float32)
    if num_frames <= 1:
        return angular_velocity
    for frame_idx in range(num_frames):
        prev_idx = max(frame_idx - 1, 0)
        next_idx = min(frame_idx + 1, num_frames - 1)
        if prev_idx == next_idx:
            continue
        delta_t = float(next_idx - prev_idx) * dt
        delta_quat = quat_multiply_xyzw(quaternions[next_idx], quat_inverse_xyzw(quaternions[prev_idx]))
        angular_velocity[frame_idx] = quat_to_rotvec_xyzw(delta_quat) / delta_t
    return angular_velocity.astype(np.float32)


def _disconnect_worker() -> None:
    pybullet = _WORKER_CONTEXT.get("pybullet")
    physics_client_id = _WORKER_CONTEXT.get("physics_client_id")
    if pybullet is not None and physics_client_id is not None:
        try:
            pybullet.disconnect(physicsClientId=physics_client_id)
        except Exception:
            pass


def init_worker(gmr_root: str, urdf_path: str) -> None:
    global _WORKER_CONTEXT

    gmr_root_path = Path(gmr_root).resolve()
    if str(gmr_root_path) not in sys.path:
        sys.path.insert(0, str(gmr_root_path))

    with _suppress_native_output():
        import pybullet
        from scipy.spatial.transform import Rotation as Rotation
        from general_motion_retargeting import GeneralMotionRetargeting as GMR
        from general_motion_retargeting.utils.lafan_vendor import utils as lafan_utils
        from general_motion_retargeting.utils.lafan_vendor.extract import Anim, channelmap

        physics_client_id = pybullet.connect(pybullet.DIRECT)
        pybullet.setAdditionalSearchPath(str(Path(urdf_path).resolve().parent), physicsClientId=physics_client_id)
        robot_id = pybullet.loadURDF(
            str(Path(urdf_path).resolve()),
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0, 1.0],
            flags=pybullet.URDF_MAINTAIN_LINK_ORDER + pybullet.URDF_ENABLE_CACHED_GRAPHICS_SHAPES,
            useFixedBase=False,
            physicsClientId=physics_client_id,
        )

    num_joints = pybullet.getNumJoints(robot_id, physicsClientId=physics_client_id)
    joint_infos = [pybullet.getJointInfo(robot_id, joint_idx, physicsClientId=physics_client_id) for joint_idx in range(num_joints)]
    joint_name_to_index = {info[1].decode("utf-8"): info[0] for info in joint_infos}
    link_name_to_index = {info[12].decode("utf-8"): info[0] for info in joint_infos}
    missing_joints = [name for name in ACTUATED_JOINT_NAMES if name not in joint_name_to_index]
    missing_links = [name for name in BODY_LINK_NAMES[1:] if name not in link_name_to_index]
    if missing_joints or missing_links:
        raise KeyError(f"URDF is missing joints={missing_joints} links={missing_links}")

    _WORKER_CONTEXT = {
        "pybullet": pybullet,
        "physics_client_id": physics_client_id,
        "robot_id": robot_id,
        "actuated_joint_indices": [joint_name_to_index[name] for name in ACTUATED_JOINT_NAMES],
        "body_link_indices": [link_name_to_index[name] for name in BODY_LINK_NAMES[1:]],
        "Rotation": Rotation,
        "GMR": GMR,
        "lafan_utils": lafan_utils,
        "Anim": Anim,
        "channelmap": channelmap,
    }
    atexit.register(_disconnect_worker)


def read_bvh_robust(filename: Path, order: str | None = None) -> tuple[object, float]:
    lafan_utils = _WORKER_CONTEXT["lafan_utils"]
    Anim = _WORKER_CONTEXT["Anim"]
    channelmap = _WORKER_CONTEXT["channelmap"]

    frame_idx = 0
    active = -1
    end_site = False
    channels = None
    frametime = None
    names: list[str] = []
    orients = np.array([]).reshape((0, 4))
    offsets = np.array([]).reshape((0, 3))
    parents = np.array([], dtype=int)
    positions = None
    rotations = None

    with open(filename, "r", errors="ignore") as f:
        for line in f:
            if "HIERARCHY" in line or "MOTION" in line:
                continue

            root_match = re.match(r"ROOT (\w+)", line)
            if root_match:
                names.append(root_match.group(1))
                offsets = np.append(offsets, np.array([[0, 0, 0]]), axis=0)
                orients = np.append(orients, np.array([[1, 0, 0, 0]]), axis=0)
                parents = np.append(parents, active)
                active = len(parents) - 1
                continue

            if "{" in line:
                continue
            if "}" in line:
                if end_site:
                    end_site = False
                else:
                    active = parents[active]
                continue

            offset_match = re.match(r"\s*OFFSET\s+([\-\d\.eE]+)\s+([\-\d\.eE]+)\s+([\-\d\.eE]+)", line)
            if offset_match:
                if not end_site:
                    offsets[active] = np.array([list(map(float, offset_match.groups()))])
                continue

            channel_match = re.match(r"\s*CHANNELS\s+(\d+)", line)
            if channel_match:
                channels = int(channel_match.group(1))
                if order is None:
                    channel_start = 0 if channels == 3 else 3
                    channel_end = 3 if channels == 3 else 6
                    parts = line.split()[2 + channel_start : 2 + channel_end]
                    if all(part in channelmap for part in parts):
                        order = "".join(channelmap[part] for part in parts)
                continue

            joint_match = re.match(r"\s*JOINT\s+(\w+)", line)
            if joint_match:
                names.append(joint_match.group(1))
                offsets = np.append(offsets, np.array([[0, 0, 0]]), axis=0)
                orients = np.append(orients, np.array([[1, 0, 0, 0]]), axis=0)
                parents = np.append(parents, active)
                active = len(parents) - 1
                continue

            if "End Site" in line:
                end_site = True
                continue

            frames_match = re.match(r"\s*Frames:\s+(\d+)", line)
            if frames_match:
                frame_count = int(frames_match.group(1))
                positions = offsets[np.newaxis].repeat(frame_count, axis=0)
                rotations = np.zeros((frame_count, len(orients), 3))
                continue

            frametime_match = re.match(r"\s*Frame Time:\s+([\d\.]+)", line)
            if frametime_match:
                frametime = float(frametime_match.group(1))
                continue

            tokens = line.strip().split()
            if not tokens:
                continue
            if positions is None or rotations is None or channels is None:
                continue

            data_block = np.array(list(map(float, tokens)), dtype=np.float64)
            num_joints = len(parents)
            if channels == 3:
                expected = 3 + 3 * num_joints
                if data_block.size != expected:
                    raise ValueError(f"Expected {expected} motion values, got {data_block.size}")
                positions[frame_idx, 0:1] = data_block[0:3]
                rotations[frame_idx, :] = data_block[3:].reshape(num_joints, 3)
            elif channels == 6:
                expected = 6 * num_joints
                if data_block.size != expected:
                    raise ValueError(f"Expected {expected} motion values, got {data_block.size}")
                data_block = data_block.reshape(num_joints, 6)
                positions[frame_idx, :] = data_block[:, 0:3]
                rotations[frame_idx, :] = data_block[:, 3:6]
            else:
                raise ValueError(f"Unsupported BVH channel count: {channels}")
            frame_idx += 1

    if positions is None or rotations is None or frametime is None:
        raise ValueError("BVH is missing motion frames or frame time.")
    if frame_idx != positions.shape[0]:
        raise ValueError(f"Parsed {frame_idx} frames but header declares {positions.shape[0]}.")
    if order is None:
        raise ValueError("Could not infer BVH rotation order.")

    quats = lafan_utils.euler_to_quat(np.radians(rotations), order=order)
    quats = lafan_utils.remove_quat_discontinuities(quats)
    return Anim(quats, positions, offsets, parents, names), frametime


def selected_source_indices(frame_count: int, source_fps: float, target_fps: float) -> np.ndarray:
    if frame_count <= 0:
        raise ValueError("BVH has no frames.")
    if frame_count == 1:
        return np.asarray([0], dtype=np.int64)
    duration = (frame_count - 1) / source_fps
    target_times = np.arange(0.0, duration + 1.0e-9, 1.0 / target_fps, dtype=np.float64)
    indices = np.rint(target_times * source_fps).astype(np.int64)
    indices = np.clip(indices, 0, frame_count - 1)
    return np.unique(indices)


def retarget_bvh_to_qpos(bvh_path: Path, target_fps: float, human_height: float) -> tuple[float, float, np.ndarray]:
    lafan_utils = _WORKER_CONTEXT["lafan_utils"]
    Rotation = _WORKER_CONTEXT["Rotation"]
    GMR = _WORKER_CONTEXT["GMR"]

    anim, frame_time = read_bvh_robust(bvh_path)
    source_fps = 1.0 / frame_time
    indices = selected_source_indices(anim.pos.shape[0], source_fps, target_fps)
    global_quats, global_pos = lafan_utils.quat_fk(anim.quats, anim.pos, anim.parents)

    rotation_matrix = np.asarray([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=np.float32)
    rotation_quat = Rotation.from_matrix(rotation_matrix).as_quat(scalar_first=True)

    bone_names = list(anim.bones)
    required = {"Hips", "Spine1", "LeftFoot", "RightFoot", "LeftToeBase", "RightToeBase"}
    missing = sorted(required - set(bone_names))
    if missing:
        raise KeyError(f"BVH missing required bvh1h joints: {missing}")

    with _suppress_native_output():
        retargeter = GMR(
            actual_human_height=human_height,
            src_human="bvh_nokov",
            tgt_robot="unitree_g1",
            verbose=False,
        )

    qpos_list = []
    for frame_index in indices:
        human_frame = {}
        for bone_index, bone_name in enumerate(bone_names):
            position = global_pos[frame_index, bone_index] @ rotation_matrix.T / 100.0
            orientation = lafan_utils.quat_mul(rotation_quat, global_quats[frame_index, bone_index])
            human_frame[bone_name] = [position, orientation]
        human_frame["LeftFootMod"] = [human_frame["LeftFoot"][0], human_frame["LeftToeBase"][1]]
        human_frame["RightFootMod"] = [human_frame["RightFoot"][0], human_frame["RightToeBase"][1]]
        human_frame["Spine2"] = human_frame["Spine1"]
        qpos_list.append(retargeter.retarget(human_frame).copy())

    qpos = np.asarray(qpos_list, dtype=np.float32)
    if qpos.ndim != 2 or qpos.shape[1] != 7 + len(ACTUATED_JOINT_NAMES):
        raise ValueError(f"Unexpected retargeted qpos shape {qpos.shape}")
    return source_fps, target_fps, qpos


def compute_body_state_sequences(root_pos: np.ndarray, root_rot_wxyz: np.ndarray, dof_pos: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    pybullet = _WORKER_CONTEXT["pybullet"]
    physics_client_id = _WORKER_CONTEXT["physics_client_id"]
    robot_id = _WORKER_CONTEXT["robot_id"]
    actuated_joint_indices = _WORKER_CONTEXT["actuated_joint_indices"]
    body_link_indices = _WORKER_CONTEXT["body_link_indices"]

    root_rot_xyzw = normalize_quaternions_xyzw(wxyz_to_xyzw(root_rot_wxyz))
    num_frames = root_pos.shape[0]
    num_bodies = len(BODY_LINK_NAMES)
    body_pos_w = np.zeros((num_frames, num_bodies, 3), dtype=np.float32)
    body_quat_xyzw = np.zeros((num_frames, num_bodies, 4), dtype=np.float32)

    for frame_idx in range(num_frames):
        pybullet.resetBasePositionAndOrientation(
            robot_id,
            root_pos[frame_idx].tolist(),
            root_rot_xyzw[frame_idx].tolist(),
            physicsClientId=physics_client_id,
        )
        for joint_index, joint_position in zip(actuated_joint_indices, dof_pos[frame_idx], strict=True):
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


def output_path_for(input_root: Path, bvh_path: Path, output_root: Path) -> Path:
    rel_path = bvh_path.relative_to(input_root)
    if len(rel_path.parts) == 1:
        return output_root / f"{bvh_path.stem}__unitree_g1.npz"
    return output_root / rel_path.parent / f"{bvh_path.stem}__unitree_g1.npz"


def save_npz(output_path: Path, compressed: bool, **arrays: np.ndarray) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    saver = np.savez_compressed if compressed else np.savez
    tmp_path = output_path.with_name(output_path.name + ".tmp")
    with tmp_path.open("wb") as f:
        saver(f, **arrays)
    tmp_path.replace(output_path)


def convert_one(task: tuple[str, str, str, float, float, bool, bool, bool]) -> dict[str, object]:
    input_root_str, bvh_path_str, output_root_str, target_fps, human_height, overwrite, compressed, ground_align = task
    input_root = Path(input_root_str)
    bvh_path = Path(bvh_path_str)
    output_root = Path(output_root_str)
    output_path = output_path_for(input_root, bvh_path, output_root)

    if output_path.exists() and not overwrite:
        with np.load(output_path) as data:
            return {
                "source_bvh": str(bvh_path.resolve()),
                "output_npz": str(output_path.resolve()),
                "frames": int(data["joint_pos"].shape[0]),
                "source_fps": None,
                "output_fps": float(data["fps"][0]),
                "ground_shift_z": None,
                "skipped_existing": True,
            }

    source_fps, output_fps, qpos = retarget_bvh_to_qpos(bvh_path, target_fps=target_fps, human_height=human_height)
    root_pos = qpos[:, :3].astype(np.float32)
    root_rot_wxyz = qpos[:, 3:7].astype(np.float32)
    dof_pos = qpos[:, 7:].astype(np.float32)
    frames = int(qpos.shape[0])
    dt = 1.0 / output_fps

    body_pos_w, body_quat_xyzw = compute_body_state_sequences(root_pos, root_rot_wxyz, dof_pos)
    ground_shift_z = 0.0
    if ground_align:
        min_z = float(np.min(body_pos_w[..., 2]))
        ground_shift_z = -min_z
        body_pos_w[..., 2] += ground_shift_z

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
        fps=np.asarray([output_fps], dtype=np.float32),
        joint_pos=dof_pos.astype(np.float32),
        joint_vel=joint_vel.astype(np.float32),
        body_pos_w=body_pos_w.astype(np.float32),
        body_quat_w=body_quat_w.astype(np.float32),
        body_lin_vel_w=body_lin_vel_w.astype(np.float32),
        body_ang_vel_w=body_ang_vel_w.astype(np.float32),
    )
    return {
        "source_bvh": str(bvh_path.resolve()),
        "output_npz": str(output_path.resolve()),
        "frames": frames,
        "source_fps": float(source_fps),
        "output_fps": float(output_fps),
        "ground_shift_z": float(ground_shift_z),
        "skipped_existing": False,
    }


def collect_bvh_files(input_root: Path, limit: int) -> list[Path]:
    files = sorted(path for path in input_root.rglob("*.bvh") if path.is_file())
    if limit > 0:
        files = files[:limit]
    if not files:
        raise FileNotFoundError(f"No .bvh files found under {input_root}")
    return files


def write_manifest(output_root: Path, dataset_name: str, motions: list[dict[str, object]]) -> None:
    index_dir = output_root / "_index"
    index_dir.mkdir(parents=True, exist_ok=True)
    manifest_data = {
        "dataset_name": dataset_name,
        "motion_count": len(motions),
        "body_names": BODY_LINK_NAMES,
        "joint_names": ACTUATED_JOINT_NAMES,
        "motions": motions,
    }
    (index_dir / "manifest.json").write_text(json.dumps(manifest_data, indent=2, ensure_ascii=False), encoding="utf-8")

    fields = ["source_bvh", "output_npz", "frames", "source_fps", "output_fps", "ground_shift_z", "skipped_existing"]
    with (index_dir / "manifest.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(motions)


def write_failures(output_root: Path, failures: Iterable[str]) -> None:
    failures = list(failures)
    index_dir = output_root / "_index"
    index_dir.mkdir(parents=True, exist_ok=True)
    (index_dir / "failures.log").write_text("\n".join(failures) + ("\n" if failures else ""), encoding="utf-8")


def write_readme(output_root: Path) -> None:
    output_root.mkdir(parents=True, exist_ok=True)
    (output_root / "README.md").write_text(README_TEXT, encoding="utf-8")


def write_report(output_root: Path, motions: list[dict[str, object]], failures: list[str], elapsed: float) -> None:
    index_dir = output_root / "_index"
    index_dir.mkdir(parents=True, exist_ok=True)
    if motions:
        frames = [int(item["frames"]) for item in motions]
        shifts = [float(item["ground_shift_z"]) for item in motions if item["ground_shift_z"] is not None]
        report = [
            "# bvh1h NPZ Processing Report",
            "",
            f"- Output root: `{output_root}`",
            f"- Motions converted or present: {len(motions)}",
            f"- Failures: {len(failures)}",
            f"- Frame count min/max/mean: {min(frames)} / {max(frames)} / {sum(frames) / len(frames):.2f}",
            f"- Ground shift z min/max: {min(shifts):.6f} / {max(shifts):.6f}" if shifts else "- Ground shift z min/max: n/a",
            f"- Elapsed seconds: {elapsed:.1f}",
            "",
            "Coordinate alignment:",
            "- BVH centimeters are converted to meters.",
            "- BVH Y-up is rotated to MOSAIC/Isaac Z-up.",
            "- `Spine2` is aliased to `Spine1` for this skeleton.",
            "- `LeftToeBase`/`RightToeBase` provide the foot orientation targets.",
            "- Output quaternions are saved as `wxyz`.",
            "- A constant per-motion z shift puts the lowest generated G1 body point at ground z=0.",
        ]
    else:
        report = ["# bvh1h NPZ Processing Report", "", f"- Failures: {len(failures)}"]
    (index_dir / "processing_report_20260603.md").write_text("\n".join(report) + "\n", encoding="utf-8")


def main() -> None:
    args = parse_args()
    input_root = Path(args.input_root).resolve()
    output_root = Path(args.output_root).resolve()
    gmr_root = Path(args.gmr_root).resolve()
    urdf_path = Path(args.urdf_path).resolve()

    if not input_root.exists():
        raise FileNotFoundError(f"Input root not found: {input_root}")
    if not gmr_root.exists():
        raise FileNotFoundError(f"GMR root not found: {gmr_root}")
    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF path not found: {urdf_path}")
    if args.workers <= 0:
        raise ValueError("--workers must be positive.")
    if args.target_fps <= 0:
        raise ValueError("--target-fps must be positive.")

    bvh_files = collect_bvh_files(input_root, args.limit)
    write_readme(output_root)

    start_time = time.time()
    motions: list[dict[str, object]] = []
    failures: list[str] = []
    tasks = [
        (
            str(input_root),
            str(bvh_path),
            str(output_root),
            float(args.target_fps),
            float(args.human_height),
            bool(args.overwrite),
            bool(args.compressed),
            not bool(args.no_ground_align),
        )
        for bvh_path in bvh_files
    ]

    print(f"[build] input_root={input_root}")
    print(f"[build] output_root={output_root}")
    print(f"[build] gmr_root={gmr_root}")
    print(f"[build] files={len(tasks)} workers={args.workers} target_fps={args.target_fps:g}")

    with ProcessPoolExecutor(max_workers=args.workers, initializer=init_worker, initargs=(str(gmr_root), str(urdf_path))) as executor:
        future_to_path = {executor.submit(convert_one, task): task[1] for task in tasks}
        for idx, future in enumerate(as_completed(future_to_path), start=1):
            source_path = future_to_path[future]
            try:
                motions.append(future.result())
            except Exception as exc:
                failures.append(f"{source_path}\t{type(exc).__name__}: {exc}")
            if idx % 10 == 0 or idx == len(tasks):
                elapsed = time.time() - start_time
                print(
                    "[progress] done={}/{} motions={} failures={} elapsed_s={:.1f}".format(
                        idx, len(tasks), len(motions), len(failures), elapsed
                    ),
                    flush=True,
                )

    motions.sort(key=lambda item: str(item["output_npz"]))
    elapsed = time.time() - start_time
    write_manifest(output_root, args.dataset_name, motions)
    write_failures(output_root, failures)
    write_report(output_root, motions, failures, elapsed)

    print(f"[done] motions={len(motions)} failures={len(failures)} output_root={output_root} elapsed_s={elapsed:.1f}")


if __name__ == "__main__":
    main()
