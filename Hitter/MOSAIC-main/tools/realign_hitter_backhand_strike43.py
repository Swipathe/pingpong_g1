#!/usr/bin/env python3
"""Align MQY HITTER forehand and backhand racket-speed peaks to frame 43."""

from __future__ import annotations

import argparse
import csv
import json
import shutil
import sys
from datetime import datetime
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INPUT = REPO_ROOT / "data" / "hitter_motions" / "mqy_g1_npz_arm_pose_corrected_v3_pre_align"
DEFAULT_OUTPUT = REPO_ROOT / "data" / "hitter_motions" / "mqy_g1_npz_arm_pose_corrected_v3_peak43_aligned"
TARGET_FRAME = 43
RIGHT_RACKET_BODY_INDEX = 30
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
from build_bvh1h_npz import (  # noqa: E402
    angular_velocity_from_quaternions_xyzw,
    finite_difference,
    wxyz_to_xyzw,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-root", default=str(DEFAULT_INPUT), help="Pre-alignment MQY dataset root.")
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT), help="Frame-43-aligned MQY dataset root.")
    parser.add_argument("--target-frame", type=int, default=TARGET_FRAME, help="Desired racket-speed peak frame.")
    parser.add_argument(
        "--edge-exclude-frames",
        type=int,
        default=0,
        help="Ignore this many frames at both sequence edges when finding the speed peak.",
    )
    parser.add_argument("--overwrite", action="store_true", help="Replace an existing output root.")
    return parser.parse_args()


def _load_npz(path: Path) -> dict[str, np.ndarray]:
    with np.load(path) as data:
        missing = [key for key in REQUIRED_KEYS if key not in data.files]
        if missing:
            raise KeyError(f"{path} is missing keys: {missing}")
        motion = {key: data[key].astype(np.float32, copy=True) for key in REQUIRED_KEYS}

    frame_count = motion["joint_pos"].shape[0]
    if motion["body_pos_w"].shape[1] <= RIGHT_RACKET_BODY_INDEX:
        raise ValueError(f"{path} does not contain right_racket_link at body index {RIGHT_RACKET_BODY_INDEX}")
    if any(array.shape[0] != frame_count for key, array in motion.items() if key != "fps"):
        raise ValueError(f"{path} has inconsistent frame counts")
    if not all(np.isfinite(array).all() for array in motion.values()):
        raise ValueError(f"{path} contains NaN or Inf")
    return motion


def _save_npz(path: Path, arrays: dict[str, np.ndarray]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_name(path.name + ".tmp")
    with tmp_path.open("wb") as file:
        np.savez_compressed(file, **arrays)
    tmp_path.replace(path)


def _shift_array(values: np.ndarray, shift_frames: int) -> np.ndarray:
    source_indices = np.arange(values.shape[0], dtype=np.int64) - int(shift_frames)
    source_indices = np.clip(source_indices, 0, values.shape[0] - 1)
    return values[source_indices].astype(np.float32, copy=True)


def _racket_speed_stats(
    motion: dict[str, np.ndarray], edge_exclude_frames: int = 0
) -> tuple[int, np.ndarray]:
    fps = float(motion["fps"].reshape(-1)[0])
    racket_positions = motion["body_pos_w"][:, RIGHT_RACKET_BODY_INDEX]
    racket_velocities = np.gradient(racket_positions, 1.0 / fps, axis=0, edge_order=1)
    speeds = np.linalg.norm(racket_velocities, axis=-1)
    edge = max(0, int(edge_exclude_frames))
    if 2 * edge >= speeds.shape[0]:
        raise ValueError(f"edge exclusion {edge} leaves no searchable frames")
    end = speeds.shape[0] - edge if edge else speeds.shape[0]
    return int(np.argmax(speeds[edge:end])) + edge, speeds


def _recompute_velocities(motion: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    fps = float(motion["fps"].reshape(-1)[0])
    dt = 1.0 / fps
    motion["joint_vel"] = finite_difference(motion["joint_pos"], dt)
    motion["body_lin_vel_w"] = finite_difference(motion["body_pos_w"], dt)

    quat_xyzw = wxyz_to_xyzw(motion["body_quat_w"])
    frame_count, body_count = quat_xyzw.shape[:2]
    body_ang_vel = np.zeros((frame_count, body_count, 3), dtype=np.float32)
    for body_index in range(body_count):
        body_ang_vel[:, body_index] = angular_velocity_from_quaternions_xyzw(
            quat_xyzw[:, body_index],
            dt,
        )
    motion["body_ang_vel_w"] = body_ang_vel
    return motion


def _relative_to_repo(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(REPO_ROOT))
    except ValueError:
        return str(path.resolve())


def _align_one(
    source: Path,
    output: Path,
    stroke: str,
    target_frame: int,
    edge_exclude_frames: int,
) -> dict[str, object]:
    source_motion = _load_npz(source)
    frame_count = source_motion["joint_pos"].shape[0]
    if not 0 <= target_frame < frame_count:
        raise ValueError(f"target frame {target_frame} is outside {source} with {frame_count} frames")

    source_peak_frame, source_speeds = _racket_speed_stats(source_motion, edge_exclude_frames)
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
    _recompute_velocities(shifted_motion)
    # The edge-hold operation can create an artificial finite-difference spike
    # immediately after a held prefix/suffix.  Do not re-select that artifact
    # as the motion peak.  The selected source peak is mapped by construction:
    # output_frame = source_frame + shift_frames == target_frame.
    aligned_peak_frame = int(source_peak_frame + shift_frames)
    if aligned_peak_frame != target_frame:
        raise RuntimeError(
            f"{source}: source peak mapping gives frame {aligned_peak_frame}, expected {target_frame}"
        )

    _save_npz(output, shifted_motion)
    return {
        "class": stroke,
        "source": _relative_to_repo(source),
        "output": _relative_to_repo(output),
        "frames": int(frame_count),
        "fps": float(source_motion["fps"].reshape(-1)[0]),
        "target_frame": int(target_frame),
        "edge_exclude_frames": int(edge_exclude_frames),
        "source_peak_frame": int(source_peak_frame),
        "applied_shift_frames": int(shift_frames),
        "aligned_peak_frame": int(aligned_peak_frame),
        "source_peak_speed_mps": float(source_speeds[source_peak_frame]),
        "aligned_peak_speed_mps": float(source_speeds[source_peak_frame]),
        "edge_hold_start_frames": int(max(shift_frames, 0)),
        "edge_hold_end_frames": int(max(-shift_frames, 0)),
    }


def _write_manifests(
    input_root: Path,
    output_root: Path,
    target_frame: int,
    rows: list[dict[str, object]],
) -> None:
    index_dir = output_root / "_index"
    index_dir.mkdir(parents=True, exist_ok=True)
    manifest = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "input_root": _relative_to_repo(input_root),
        "output_root": _relative_to_repo(output_root),
        "target_frame": int(target_frame),
        "racket_body_index": RIGHT_RACKET_BODY_INDEX,
        "method": "shift_with_edge_hold_using_saved_right_racket_link_speed_peak",
        "total_count": len(rows),
        "forehand_count": sum(row["class"] == "forehand" for row in rows),
        "backhand_count": sum(row["class"] == "backhand" for row in rows),
        "rows": rows,
    }
    with (index_dir / "strike43_alignment_manifest.json").open("w") as file:
        json.dump(manifest, file, indent=2)

    with (index_dir / "strike43_alignment_manifest.csv").open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=sorted({key for row in rows for key in row}))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    args = parse_args()
    input_root = Path(args.input_root).resolve()
    output_root = Path(args.output_root).resolve()
    if not input_root.exists():
        raise FileNotFoundError(input_root)
    if output_root.exists():
        if not args.overwrite:
            raise FileExistsError(f"{output_root} already exists; pass --overwrite to replace it")
        shutil.rmtree(output_root)

    rows: list[dict[str, object]] = []
    for stroke in ("forehand", "backhand"):
        sources = sorted((input_root / stroke).glob("*.npz"))
        if not sources:
            raise FileNotFoundError(f"No {stroke} npz files found under {input_root / stroke}")
        for source in sources:
            output = output_root / stroke / source.name
            row = _align_one(
                source,
                output,
                stroke,
                int(args.target_frame),
                int(args.edge_exclude_frames),
            )
            rows.append(row)
            print(
                f"[align] {stroke} {source.name}: "
                f"source_peak={row['source_peak_frame']} "
                f"shift={row['applied_shift_frames']} "
                f"aligned_peak={row['aligned_peak_frame']}"
            )

    _write_manifests(input_root, output_root, int(args.target_frame), rows)
    print(f"[Done] wrote {len(rows)} aligned motions to {output_root}")


if __name__ == "__main__":
    main()
