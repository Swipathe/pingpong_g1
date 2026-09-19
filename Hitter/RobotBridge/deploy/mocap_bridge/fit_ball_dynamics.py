#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class Trajectory:
    source: str
    times: np.ndarray
    positions: np.ndarray


@dataclass(frozen=True)
class LocalKinematics:
    positions: np.ndarray
    velocities: np.ndarray
    accelerations: np.ndarray


@dataclass(frozen=True)
class FitResult:
    drag_coefficient: float
    horizontal_restitution: float
    vertical_restitution: float
    trajectory_count: int
    bounce_count: int
    flight_sample_count: int
    source_files: List[str]

    def to_dict(self) -> dict:
        return {
            "drag_coefficient": self.drag_coefficient,
            "horizontal_restitution": self.horizontal_restitution,
            "vertical_restitution": self.vertical_restitution,
            "trajectory_count": self.trajectory_count,
            "bounce_count": self.bounce_count,
            "flight_sample_count": self.flight_sample_count,
            "source_files": self.source_files,
        }

    def suggested_yaml(self) -> str:
        return (
            f"vertical_restitution: {self.vertical_restitution:.6f}\n"
            f"horizontal_restitution: {self.horizontal_restitution:.6f}\n"
            f"drag_coefficient: {self.drag_coefficient:.6f}"
        )


def _float_or_none(value) -> Optional[float]:
    if value is None or value == "":
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _is_true_field(value) -> bool:
    if value is None:
        return True
    text = str(value).strip().lower()
    return text not in {"", "0", "false", "nan", "none"}


def _row_time(row: dict, index: int, sample_rate_hz: float, *, time_source: str = "frame") -> Optional[float]:
    time_source = str(time_source).lower()
    if time_source in {"frame", "auto"}:
        source_time = _float_or_none(row.get("vicon_time_s"))
        if source_time is not None:
            return source_time
        frame = _float_or_none(row.get("frame_number"))
        if frame is not None and sample_rate_hz > 0.0:
            return frame / sample_rate_hz
        if time_source == "frame":
            return index / sample_rate_hz if sample_rate_hz > 0.0 else None

    if time_source == "elapsed":
        for key in ("elapsed_s", "time_s", "timestamp_s", "t", "host_time_s"):
            value = _float_or_none(row.get(key))
            if value is not None:
                return value
    elif time_source == "host":
        for key in ("host_time_s", "elapsed_s", "time_s", "timestamp_s", "t"):
            value = _float_or_none(row.get(key))
            if value is not None:
                return value
    else:
        raise ValueError("time_source must be 'frame', 'elapsed', 'host', or 'auto'.")

    frame = _float_or_none(row.get("frame_number"))
    if frame is not None and sample_rate_hz > 0.0:
        return frame / sample_rate_hz
    return index / sample_rate_hz if sample_rate_hz > 0.0 else None


def _row_gap_time(row: dict, index: int, sample_rate_hz: float) -> Optional[float]:
    for key in ("vicon_time_s", "host_time_s", "elapsed_s", "time_s", "timestamp_s", "t"):
        value = _float_or_none(row.get(key))
        if value is not None:
            return value
    frame = _float_or_none(row.get("frame_number"))
    if frame is not None and sample_rate_hz > 0.0:
        return frame / sample_rate_hz
    return index / sample_rate_hz if sample_rate_hz > 0.0 else None


def _row_position_m(row: dict) -> Optional[np.ndarray]:
    direct = [_float_or_none(row.get(f"ball_{axis}_m")) for axis in ("x", "y", "z")]
    if all(value is not None for value in direct):
        return np.asarray(direct, dtype=np.float64)

    millimeters = [_float_or_none(row.get(f"ball_{axis}_mm")) for axis in ("x", "y", "z")]
    if all(value is not None for value in millimeters):
        return 0.001 * np.asarray(millimeters, dtype=np.float64)
    return None


def _strictly_increasing_times(times: np.ndarray, sample_rate_hz: float) -> np.ndarray:
    if times.shape[0] <= 1:
        return times
    fixed = times.astype(np.float64, copy=True)
    fallback_dt = 1.0 / sample_rate_hz if sample_rate_hz > 0.0 else 1.0e-3
    for index in range(1, fixed.shape[0]):
        if fixed[index] <= fixed[index - 1]:
            fixed[index] = fixed[index - 1] + fallback_dt
    return fixed


def read_nexus_csv(
    path: Path,
    *,
    sample_rate_hz: float = 300.0,
    max_gap_s: float = 0.10,
    time_source: str = "frame",
) -> List[Trajectory]:
    samples: List[Tuple[int, Optional[int], float, float, np.ndarray]] = []
    with Path(path).open(newline="") as handle:
        reader = csv.DictReader(handle)
        for index, row in enumerate(reader):
            if not _is_true_field(row.get("ball_valid")):
                continue
            if row.get("ball_occluded") is not None and _is_true_field(row.get("ball_occluded")):
                continue
            t = _row_time(row, index, sample_rate_hz, time_source=time_source)
            gap_t = _row_gap_time(row, index, sample_rate_hz)
            p = _row_position_m(row)
            if t is None or gap_t is None or p is None or not np.isfinite(p).all():
                continue
            clean_segment = _float_or_none(row.get("clean_segment_id"))
            clean_segment_id = None if clean_segment is None else int(clean_segment)
            samples.append((index, clean_segment_id, float(gap_t), float(t), p))

    if not samples:
        return []

    segments: List[List[Tuple[int, Optional[int], float, float, np.ndarray]]] = [[]]
    if any(sample[1] is not None for sample in samples):
        samples.sort(key=lambda item: (item[1] if item[1] is not None else -1, item[0]))
        last_segment_id: Optional[int] = None
        for sample in samples:
            segment_id = sample[1]
            if segments[-1] and segment_id != last_segment_id:
                segments.append([])
            segments[-1].append(sample)
            last_segment_id = segment_id
    else:
        samples.sort(key=lambda item: item[2])
        for sample in samples:
            if segments[-1] and sample[2] - segments[-1][-1][2] > max_gap_s:
                segments.append([])
            segments[-1].append(sample)

    trajectories = []
    for seg_index, segment in enumerate(segments):
        if not segment:
            continue
        times = np.asarray([item[3] for item in segment], dtype=np.float64)
        positions = np.asarray([item[4] for item in segment], dtype=np.float64)
        times = _strictly_increasing_times(times, sample_rate_hz)
        times = times - times[0]
        trajectories.append(Trajectory(source=f"{path}#{seg_index}", times=times, positions=positions))
    return trajectories


def local_quadratic_kinematics(times: np.ndarray, positions: np.ndarray, *, fit_window: int = 31) -> LocalKinematics:
    if fit_window < 3:
        raise ValueError("fit_window must be at least 3.")
    if times.ndim != 1 or positions.ndim != 2 or positions.shape[1] != 3:
        raise ValueError("times must be (N,), positions must be (N,3).")
    if times.shape[0] != positions.shape[0]:
        raise ValueError("times and positions must have the same length.")
    if times.shape[0] < fit_window:
        raise ValueError(f"trajectory has {times.shape[0]} samples, fewer than fit_window={fit_window}.")

    count = times.shape[0]
    half = fit_window // 2
    pos_fit = np.empty_like(positions, dtype=np.float64)
    vel_fit = np.empty_like(positions, dtype=np.float64)
    acc_fit = np.empty_like(positions, dtype=np.float64)

    for index in range(count):
        start = max(0, min(index - half, count - fit_window))
        end = start + fit_window
        tau = times[start:end] - times[index]
        design = np.column_stack([np.ones(fit_window, dtype=np.float64), tau, tau * tau])
        coeffs, *_ = np.linalg.lstsq(design, positions[start:end], rcond=None)
        pos_fit[index] = coeffs[0]
        vel_fit[index] = coeffs[1]
        acc_fit[index] = 2.0 * coeffs[2]

    return LocalKinematics(positions=pos_fit, velocities=vel_fit, accelerations=acc_fit)


def detect_bounces(
    times: np.ndarray,
    positions: np.ndarray,
    velocities: np.ndarray,
    *,
    min_vz: float = 0.20,
    contact_z: Optional[float] = None,
    contact_tolerance: float = 0.04,
    min_separation_s: float = 0.20,
) -> List[int]:
    z = positions[:, 2]
    vz = velocities[:, 2]
    candidates = []
    for index in range(1, len(times) - 1):
        sign_turn = vz[index - 1] < -min_vz and vz[index + 1] > min_vz
        local_min = z[index] <= z[index - 1] and z[index] <= z[index + 1]
        if not (sign_turn or local_min):
            continue
        if contact_z is not None and abs(float(z[index]) - float(contact_z)) > contact_tolerance:
            continue
        candidates.append(index)

    if not candidates and contact_z is not None:
        near_contact = np.where(np.abs(z - float(contact_z)) <= contact_tolerance)[0]
        candidates = [int(index) for index in near_contact if 0 < index < len(times) - 1]

    selected = []
    for index in candidates:
        if selected and times[index] - times[selected[-1]] < min_separation_s:
            if z[index] < z[selected[-1]]:
                selected[-1] = index
            continue
        selected.append(index)
    return selected


def _fit_velocity_at(times: np.ndarray, positions: np.ndarray, eval_time: float) -> np.ndarray:
    if len(times) < 3:
        raise ValueError("at least 3 samples are required to fit velocity.")
    tau = times - float(eval_time)
    design = np.column_stack([np.ones(len(times), dtype=np.float64), tau, tau * tau])
    coeffs, *_ = np.linalg.lstsq(design, positions, rcond=None)
    return coeffs[1]


def restitution_from_bounce(
    times: np.ndarray,
    positions: np.ndarray,
    bounce_index: int,
    *,
    window_s: float = 0.035,
) -> Optional[Tuple[float, float]]:
    t_bounce = float(times[bounce_index])
    pre_mask = (times >= t_bounce - window_s) & (times < t_bounce)
    post_mask = (times > t_bounce) & (times <= t_bounce + window_s)
    if int(np.count_nonzero(pre_mask)) < 3 or int(np.count_nonzero(post_mask)) < 3:
        return None

    v_pre = _fit_velocity_at(times[pre_mask], positions[pre_mask], t_bounce)
    v_post = _fit_velocity_at(times[post_mask], positions[post_mask], t_bounce)
    horizontal_pre = float(np.linalg.norm(v_pre[:2]))
    vertical_pre = abs(float(v_pre[2]))
    if horizontal_pre < 1.0e-6 or vertical_pre < 1.0e-6:
        return None
    horizontal = float(np.linalg.norm(v_post[:2])) / horizontal_pre
    vertical = abs(float(v_post[2])) / vertical_pre
    if not (math.isfinite(horizontal) and math.isfinite(vertical)):
        return None
    return horizontal, vertical


def estimate_drag_coefficient(
    times: np.ndarray,
    kinematics: LocalKinematics,
    bounce_indices: Sequence[int],
    *,
    gravity: Sequence[float] = (0.0, 0.0, -9.81),
    exclude_bounce_window_s: float = 0.045,
    min_flight_speed: float = 0.20,
) -> Tuple[float, int]:
    gravity_vec = np.asarray(gravity, dtype=np.float64).reshape(3)
    keep = np.ones(times.shape[0], dtype=bool)
    for index in bounce_indices:
        keep &= np.abs(times - times[index]) > exclude_bounce_window_s

    velocities = kinematics.velocities[keep]
    accelerations = kinematics.accelerations[keep]
    speeds = np.linalg.norm(velocities, axis=1)
    valid = np.isfinite(speeds) & (speeds >= min_flight_speed)
    velocities = velocities[valid]
    accelerations = accelerations[valid]
    if velocities.shape[0] == 0:
        return 0.0, 0

    x = -np.linalg.norm(velocities, axis=1)[:, None] * velocities
    y = accelerations - gravity_vec.reshape(1, 3)
    denom = float(np.sum(x * x))
    if denom <= 1.0e-12:
        return 0.0, 0
    k = float(np.sum(x * y) / denom)
    return max(k, 0.0), int(velocities.shape[0])


def fit_trajectories(
    trajectories: Iterable[Trajectory],
    *,
    fit_window: int = 31,
    min_segment_samples: int = 80,
    gravity: Sequence[float] = (0.0, 0.0, -9.81),
    contact_z: Optional[float] = None,
    contact_tolerance: float = 0.04,
    bounce_window_s: float = 0.035,
    exclude_bounce_window_s: float = 0.045,
    min_flight_speed: float = 0.20,
) -> FitResult:
    ks = []
    k_weights = []
    horizontal = []
    vertical = []
    used_sources = []
    trajectory_count = 0
    bounce_count = 0
    flight_sample_count = 0

    for trajectory in trajectories:
        if trajectory.times.shape[0] < max(fit_window, min_segment_samples):
            continue
        trajectory_count += 1
        used_sources.append(trajectory.source)
        kinematics = local_quadratic_kinematics(trajectory.times, trajectory.positions, fit_window=fit_window)
        bounces = detect_bounces(
            trajectory.times,
            trajectory.positions,
            kinematics.velocities,
            contact_z=contact_z,
            contact_tolerance=contact_tolerance,
        )
        bounce_count += len(bounces)

        k, sample_count = estimate_drag_coefficient(
            trajectory.times,
            kinematics,
            bounces,
            gravity=gravity,
            exclude_bounce_window_s=exclude_bounce_window_s,
            min_flight_speed=min_flight_speed,
        )
        if sample_count > 0:
            ks.append(k)
            k_weights.append(sample_count)
            flight_sample_count += sample_count

        for bounce_index in bounces:
            value = restitution_from_bounce(
                trajectory.times,
                trajectory.positions,
                bounce_index,
                window_s=bounce_window_s,
            )
            if value is None:
                continue
            h, v = value
            horizontal.append(h)
            vertical.append(v)

    if not ks:
        raise RuntimeError("No usable flight samples found for drag fitting.")

    drag = float(np.average(np.asarray(ks, dtype=np.float64), weights=np.asarray(k_weights, dtype=np.float64)))
    horizontal_value = float(np.median(horizontal)) if horizontal else float("nan")
    vertical_value = float(np.median(vertical)) if vertical else float("nan")
    return FitResult(
        drag_coefficient=drag,
        horizontal_restitution=horizontal_value,
        vertical_restitution=vertical_value,
        trajectory_count=trajectory_count,
        bounce_count=bounce_count,
        flight_sample_count=flight_sample_count,
        source_files=used_sources,
    )


def fit_csv_paths(
    paths: Sequence[Path],
    *,
    sample_rate_hz: float = 300.0,
    max_gap_s: float = 0.10,
    fit_window: int = 31,
    min_segment_samples: int = 80,
    gravity: Sequence[float] = (0.0, 0.0, -9.81),
    contact_z: Optional[float] = None,
    contact_tolerance: float = 0.04,
    bounce_window_s: float = 0.035,
    exclude_bounce_window_s: float = 0.045,
    min_flight_speed: float = 0.20,
    time_source: str = "frame",
) -> FitResult:
    trajectories: List[Trajectory] = []
    for path in paths:
        trajectories.extend(
            read_nexus_csv(
                Path(path),
                sample_rate_hz=sample_rate_hz,
                max_gap_s=max_gap_s,
                time_source=time_source,
            )
        )
    if not trajectories:
        raise RuntimeError("No valid ball samples found in input CSV files.")
    return fit_trajectories(
        trajectories,
        fit_window=fit_window,
        min_segment_samples=min_segment_samples,
        gravity=gravity,
        contact_z=contact_z,
        contact_tolerance=contact_tolerance,
        bounce_window_s=bounce_window_s,
        exclude_bounce_window_s=exclude_bounce_window_s,
        min_flight_speed=min_flight_speed,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Fit HITTER ball dynamics from Nexus/Vicon ball CSV files.")
    parser.add_argument(
        "csv",
        nargs="+",
        type=Path,
        help="CSV file(s) recorded by nexus_probe.py or monitor_vicon_lcm.py --csv",
    )
    parser.add_argument("--sample-rate-hz", type=float, default=300.0)
    parser.add_argument(
        "--time-source",
        choices=("frame", "elapsed", "host", "auto"),
        default="frame",
        help="Time base for velocity fitting. Use frame for Vicon/LCM recordings at --sample-rate-hz.",
    )
    parser.add_argument("--max-gap-s", type=float, default=0.10)
    parser.add_argument("--fit-window", type=int, default=31)
    parser.add_argument("--min-segment-samples", type=int, default=80)
    parser.add_argument("--gravity-z", type=float, default=-9.81)
    parser.add_argument("--contact-z", type=float, default=None, help="Optional table contact height in the CSV frame.")
    parser.add_argument("--contact-tolerance", type=float, default=0.04)
    parser.add_argument("--bounce-window-s", type=float, default=0.035)
    parser.add_argument("--exclude-bounce-window-s", type=float, default=0.045)
    parser.add_argument("--min-flight-speed", type=float, default=0.20)
    parser.add_argument("--output-json", type=Path, default=None)
    args = parser.parse_args()

    try:
        result = fit_csv_paths(
            args.csv,
            sample_rate_hz=args.sample_rate_hz,
            max_gap_s=args.max_gap_s,
            fit_window=args.fit_window,
            min_segment_samples=args.min_segment_samples,
            gravity=(0.0, 0.0, args.gravity_z),
            contact_z=args.contact_z,
            contact_tolerance=args.contact_tolerance,
            bounce_window_s=args.bounce_window_s,
            exclude_bounce_window_s=args.exclude_bounce_window_s,
            min_flight_speed=args.min_flight_speed,
            time_source=args.time_source,
        )
    except RuntimeError as exc:
        print(f"error: {exc}")
        return 1

    print(json.dumps(result.to_dict(), indent=2))
    print("\nSuggested config/mimic/hitter.yaml values:")
    print(result.suggested_yaml())
    if args.output_json is not None:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(result.to_dict(), indent=2) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
