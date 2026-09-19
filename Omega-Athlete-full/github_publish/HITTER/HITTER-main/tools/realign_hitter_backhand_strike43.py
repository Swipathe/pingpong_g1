#!/usr/bin/env python3
"""Realign new HITTER backhand references so racket peak speed lands on frame 43."""

from __future__ import annotations

import argparse
import csv
import json
import shutil
import sys
from datetime import datetime
from pathlib import Path

import mujoco
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = REPO_ROOT.parent
ROBOTBRIDGE_DEPLOY = WORKSPACE_ROOT / "RobotBridge" / "deploy"
DEFAULT_INPUT = REPO_ROOT / "data" / "hitter_motions" / "hitter_mix_oldiphone_newwechat_20260608_strike43"
DEFAULT_OUTPUT = REPO_ROOT / "data" / "hitter_motions" / "hitter_mix_oldiphone_newwechat_20260609_backhand_strike43_aligned"
DEFAULT_XML = ROBOTBRIDGE_DEPLOY / "data" / "assets" / "g1" / "g1_29dof_hitter_racket_noball_tmp.xml"
FALLBACK_XML = ROBOTBRIDGE_DEPLOY / "data" / "assets" / "g1" / "g1_29dof_hitter_racket.xml"
TARGET_FRAME = 43
REQUIRED_KEYS = (
    "fps",
    "joint_pos",
    "joint_vel",
    "body_pos_w",
    "body_quat_w",
    "body_lin_vel_w",
    "body_ang_vel_w",
)


sys.path.insert(0, str(REPO_ROOT / "scripts"))
sys.path.insert(0, str(ROBOTBRIDGE_DEPLOY))
from build_bvh1h_npz import (  # noqa: E402
    angular_velocity_from_quaternions_xyzw,
    finite_difference,
    wxyz_to_xyzw,
)
from play_hitter_reference_mujoco import (  # noqa: E402
    ISAAC_JOINT_NAMES,
    _joint_reorder_indices,
    _model_joint_names,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-root", default=str(DEFAULT_INPUT), help="Existing mixed dataset root.")
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT), help="Output mixed dataset root.")
    parser.add_argument("--xml", default=None, help="MuJoCo XML used for right_racket_link FK.")
    parser.add_argument("--target-frame", type=int, default=TARGET_FRAME, help="Desired strike/peak frame.")
    parser.add_argument("--overwrite", action="store_true", help="Remove an existing output root first.")
    return parser.parse_args()


def _load_npz(path: Path) -> dict[str, np.ndarray]:
    with np.load(path) as data:
        missing = [key for key in REQUIRED_KEYS if key not in data.files]
        if missing:
            raise KeyError(f"{path} is missing keys: {missing}")
        return {key: data[key].astype(np.float32, copy=True) for key in REQUIRED_KEYS}


def _save_npz(path: Path, arrays: dict[str, np.ndarray]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_name(path.name + ".tmp")
    with tmp_path.open("wb") as f:
        np.savez_compressed(f, **arrays)
    tmp_path.replace(path)


def _shift_array(values: np.ndarray, shift_frames: int) -> np.ndarray:
    frame_count = values.shape[0]
    source_indices = np.arange(frame_count, dtype=np.int64) - int(shift_frames)
    source_indices = np.clip(source_indices, 0, frame_count - 1)
    return values[source_indices].astype(np.float32, copy=True)


def _normalize_quat_wxyz(quat: np.ndarray) -> np.ndarray:
    quat = np.asarray(quat, dtype=np.float64)
    norm = np.linalg.norm(quat)
    if norm <= 1.0e-8:
        raise ValueError("Encountered near-zero root quaternion.")
    return quat / norm


def _racket_positions_from_motion(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    motion: dict[str, np.ndarray],
    reorder_indices: np.ndarray,
) -> np.ndarray:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_racket_link")
    if body_id < 0:
        raise KeyError("MuJoCo model has no body named right_racket_link.")

    frame_count = motion["joint_pos"].shape[0]
    positions = np.zeros((frame_count, 3), dtype=np.float64)
    for frame in range(frame_count):
        data.qpos[:] = 0.0
        data.qvel[:] = 0.0
        data.qpos[:3] = motion["body_pos_w"][frame, 0].astype(np.float64)
        data.qpos[3:7] = _normalize_quat_wxyz(motion["body_quat_w"][frame, 0])
        data.qpos[7 : 7 + len(reorder_indices)] = motion["joint_pos"][frame, reorder_indices].astype(np.float64)
        mujoco.mj_forward(model, data)
        positions[frame] = data.xpos[body_id]
    return positions


def _racket_speed_stats(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    motion: dict[str, np.ndarray],
    reorder_indices: np.ndarray,
) -> tuple[int, np.ndarray]:
    fps = float(np.asarray(motion["fps"]).reshape(-1)[0])
    positions = _racket_positions_from_motion(model, data, motion, reorder_indices)
    velocities = np.gradient(positions, 1.0 / fps, axis=0, edge_order=1)
    speeds = np.linalg.norm(velocities, axis=-1)
    return int(np.argmax(speeds)), speeds.astype(np.float64)


def _recompute_velocities(motion: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    fps = float(np.asarray(motion["fps"]).reshape(-1)[0])
    dt = 1.0 / fps
    motion["joint_vel"] = finite_difference(motion["joint_pos"], dt)
    motion["body_lin_vel_w"] = finite_difference(motion["body_pos_w"], dt)

    quat_xyzw = wxyz_to_xyzw(motion["body_quat_w"])
    frame_count, body_count = quat_xyzw.shape[:2]
    body_ang_vel = np.zeros((frame_count, body_count, 3), dtype=np.float32)
    for body_idx in range(body_count):
        body_ang_vel[:, body_idx] = angular_velocity_from_quaternions_xyzw(quat_xyzw[:, body_idx], dt)
    motion["body_ang_vel_w"] = body_ang_vel
    return motion


def _relative_to_repo(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(REPO_ROOT))
    except ValueError:
        return str(path.resolve())


def _copy_forehands(input_root: Path, output_root: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for source in sorted((input_root / "forehand").glob("*.npz")):
        output = output_root / "forehand" / source.name
        output.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, output)
        with np.load(output) as data:
            rows.append(
                {
                    "class": "forehand",
                    "source": _relative_to_repo(source),
                    "output": _relative_to_repo(output),
                    "frames": int(data["joint_pos"].shape[0]),
                    "fps": float(np.asarray(data["fps"]).reshape(-1)[0]),
                    "copied_unchanged": True,
                }
            )
    return rows


def _realign_backhands(
    input_root: Path,
    output_root: Path,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    reorder_indices: np.ndarray,
    target_frame: int,
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for source in sorted((input_root / "backhand").glob("*.npz")):
        source_motion = _load_npz(source)
        source_peak_frame, source_speeds = _racket_speed_stats(model, data, source_motion, reorder_indices)
        shift_frames = int(target_frame - source_peak_frame)

        shifted_motion = {
            "fps": source_motion["fps"],
            "joint_pos": _shift_array(source_motion["joint_pos"], shift_frames),
            "joint_vel": source_motion["joint_vel"],
            "body_pos_w": _shift_array(source_motion["body_pos_w"], shift_frames),
            "body_quat_w": _shift_array(source_motion["body_quat_w"], shift_frames),
            "body_lin_vel_w": source_motion["body_lin_vel_w"],
            "body_ang_vel_w": source_motion["body_ang_vel_w"],
        }
        shifted_motion = _recompute_velocities(shifted_motion)

        aligned_peak_frame, aligned_speeds = _racket_speed_stats(model, data, shifted_motion, reorder_indices)
        output = output_root / "backhand" / source.name
        _save_npz(output, shifted_motion)

        rows.append(
            {
                "class": "backhand",
                "source": _relative_to_repo(source),
                "output": _relative_to_repo(output),
                "frames": int(shifted_motion["joint_pos"].shape[0]),
                "fps": float(np.asarray(shifted_motion["fps"]).reshape(-1)[0]),
                "target_frame": int(target_frame),
                "source_peak_frame": int(source_peak_frame),
                "applied_shift_frames": int(shift_frames),
                "aligned_peak_frame": int(aligned_peak_frame),
                "aligned_peak_error_frames": int(aligned_peak_frame - target_frame),
                "source_peak_speed_mps": float(source_speeds[source_peak_frame]),
                "source_strike_speed_mps": float(source_speeds[target_frame]),
                "aligned_strike_speed_mps": float(aligned_speeds[target_frame]),
                "edge_hold_start_frames": int(max(shift_frames + 1, 0)),
                "edge_hold_end_frames": int(max(-shift_frames + 1, 0)),
                "copied_unchanged": False,
            }
        )
    return rows


def _write_manifests(output_root: Path, rows: list[dict[str, object]], args: argparse.Namespace, xml_path: Path) -> None:
    index_dir = output_root / "_index"
    index_dir.mkdir(parents=True, exist_ok=True)

    manifest = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "input_root": _relative_to_repo(Path(args.input_root)),
        "output_root": _relative_to_repo(output_root),
        "mujoco_xml": str(xml_path.resolve()),
        "target_frame": int(args.target_frame),
        "method": "shift_with_edge_hold_using_mujoco_right_racket_link_speed_peak",
        "forehand_policy": "copied unchanged from input_root/forehand",
        "backhand_policy": "shifted so each right_racket_link speed peak aligns to target_frame, then velocities recomputed",
        "total_count": len(rows),
        "forehand_count": sum(1 for row in rows if row["class"] == "forehand"),
        "backhand_count": sum(1 for row in rows if row["class"] == "backhand"),
        "rows": rows,
    }
    with (index_dir / "backhand_strike43_realignment_manifest.json").open("w") as f:
        json.dump(manifest, f, indent=2)

    csv_path = index_dir / "backhand_strike43_realignment_manifest.csv"
    fieldnames = sorted({key for row in rows for key in row.keys()})
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    args = parse_args()
    input_root = Path(args.input_root).resolve()
    output_root = Path(args.output_root).resolve()
    xml_path = Path(args.xml).resolve() if args.xml else DEFAULT_XML
    if not xml_path.exists():
        xml_path = FALLBACK_XML
    if not input_root.exists():
        raise FileNotFoundError(input_root)
    if output_root.exists():
        if not args.overwrite:
            raise FileExistsError(f"{output_root} already exists; pass --overwrite to replace it.")
        shutil.rmtree(output_root)

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    model_joint_names = _model_joint_names(model)
    reorder_indices = _joint_reorder_indices(ISAAC_JOINT_NAMES, model_joint_names)

    rows = []
    rows.extend(_copy_forehands(input_root, output_root))
    rows.extend(_realign_backhands(input_root, output_root, model, data, reorder_indices, args.target_frame))
    _write_manifests(output_root, rows, args, xml_path)

    print(f"[realign] output_root={output_root}")
    print(f"[realign] forehand={sum(1 for row in rows if row['class'] == 'forehand')} backhand={sum(1 for row in rows if row['class'] == 'backhand')}")
    for row in rows:
        if row["class"] == "backhand":
            print(
                "[realign] "
                f"{Path(row['output']).name}: source_peak={row['source_peak_frame']} "
                f"shift={row['applied_shift_frames']} aligned_peak={row['aligned_peak_frame']} "
                f"strike_speed={row['aligned_strike_speed_mps']:.3f}m/s"
            )


if __name__ == "__main__":
    main()
