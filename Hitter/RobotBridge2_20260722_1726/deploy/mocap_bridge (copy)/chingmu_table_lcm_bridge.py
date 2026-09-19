#!/usr/bin/env python3
from __future__ import annotations

import argparse
import itertools
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional, Sequence

import numpy as np
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[2]
for import_path in (ROOT, ROOT / "deploy"):
    if str(import_path) not in sys.path:
        sys.path.insert(0, str(import_path))

from unitree_sdk2.lcm_types.transformation_t import transformation_t


@dataclass(frozen=True)
class BridgeConfig:
    base_subject: str = "G1Pelvis"
    channel: str = "vicon_state_data"
    lcm_url: str = "udpm://239.255.76.67:7667?ttl=255"
    table_length_m: float = 2.730738
    table_width_m: float = 1.512451
    table_height_m: float = 0.760000
    source_rate_hz: float = 360.0
    corner_exclusion_radius_mm: float = 50.0
    pelvis_offset_heading_m: tuple = (0.003145, 0.044074, 0.048231)


@dataclass(frozen=True)
class TableFrame:
    center_raw_mm: np.ndarray
    x_axis_raw: np.ndarray
    y_axis_raw: np.ndarray
    z_axis_raw: np.ndarray
    corners_raw_mm: np.ndarray
    rectangle_score: float


@dataclass(frozen=True)
class CalibrationResult:
    table_frame: TableFrame


def _stable_marker_centers(
    frames: Sequence,
    *,
    association_radius_mm: float = 80.0,
    maximum_rms_mm: float = 15.0,
) -> np.ndarray:
    """Return stationary unlabeled-marker centers observed during calibration."""
    clusters = []
    for frame in frames:
        used_clusters = set()
        markers = np.asarray(frame.unlabeled_markers_mm, dtype=np.float64).reshape(-1, 3)
        for marker in markers:
            nearest_index = None
            nearest_distance = float("inf")
            for index, cluster in enumerate(clusters):
                if index in used_clusters:
                    continue
                distance = float(np.linalg.norm(marker - cluster["mean"]))
                if distance <= association_radius_mm and distance < nearest_distance:
                    nearest_index = index
                    nearest_distance = distance
            if nearest_index is None:
                clusters.append(
                    {
                        "mean": marker.copy(),
                        "count": 1,
                        "sum_squared_error": 0.0,
                    }
                )
                used_clusters.add(len(clusters) - 1)
                continue

            cluster = clusters[nearest_index]
            old_mean = cluster["mean"].copy()
            cluster["count"] += 1
            cluster["mean"] += (marker - cluster["mean"]) / cluster["count"]
            cluster["sum_squared_error"] += float(
                np.dot(marker - old_mean, marker - cluster["mean"])
            )
            used_clusters.add(nearest_index)

    minimum_observations = max(5, int(np.ceil(0.1 * len(frames))))
    stable = []
    for cluster in clusters:
        if cluster["count"] < minimum_observations:
            continue
        rms = np.sqrt(cluster["sum_squared_error"] / cluster["count"])
        if rms <= maximum_rms_mm:
            stable.append(cluster["mean"].copy())
    return np.asarray(stable, dtype=np.float64).reshape(-1, 3)


def calibrate_from_frames(frames: Iterable, config: BridgeConfig) -> CalibrationResult:
    frames = list(frames)
    if not frames:
        raise ValueError("no ChingMu frames received during calibration")

    robot_raw_mm = None
    for frame in frames:
        if frame.body_position_mm is None:
            continue
        try:
            position = np.asarray(
                frame.body_position_mm, dtype=np.float64
            ).reshape(3)
        except (TypeError, ValueError):
            continue
        if np.isfinite(position).all():
            robot_raw_mm = position.copy()
            break
    if robot_raw_mm is None:
        raise ValueError(
            f"{config.base_subject} needs a valid root position during calibration"
        )
    stable_markers = _stable_marker_centers(frames)
    table = infer_table_frame(stable_markers, robot_raw_mm, config)
    return CalibrationResult(table_frame=table)


def save_table_frame(path, table: TableFrame, config: BridgeConfig) -> None:
    calibration_path = Path(path)
    calibration_path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "format": "robotbridge2_chingmu_table_frame_v1",
        "units": "metres",
        "table_length_m": config.table_length_m,
        "table_width_m": config.table_width_m,
        "table_height_m": config.table_height_m,
        "center_raw_m": (table.center_raw_mm * 0.001).tolist(),
        "x_axis_raw": table.x_axis_raw.tolist(),
        "y_axis_raw": table.y_axis_raw.tolist(),
        "z_axis_raw": table.z_axis_raw.tolist(),
        "corners_raw_m": (table.corners_raw_mm * 0.001).tolist(),
        "rectangle_score": float(table.rectangle_score),
    }
    calibration_path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def load_table_frame(path, config: BridgeConfig) -> TableFrame:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    if payload.get("format") != "robotbridge2_chingmu_table_frame_v1":
        raise ValueError(f"unsupported ChingMu table calibration format in {path}")
    for key, expected in (
        ("table_length_m", config.table_length_m),
        ("table_width_m", config.table_width_m),
        ("table_height_m", config.table_height_m),
    ):
        if not np.isclose(float(payload[key]), float(expected), atol=1.0e-9):
            raise ValueError(
                f"table calibration {key}={payload[key]} does not match configured {expected}"
            )
    return TableFrame(
        center_raw_mm=np.asarray(payload["center_raw_m"], dtype=np.float64) * 1000.0,
        x_axis_raw=np.asarray(payload["x_axis_raw"], dtype=np.float64),
        y_axis_raw=np.asarray(payload["y_axis_raw"], dtype=np.float64),
        z_axis_raw=np.asarray(payload["z_axis_raw"], dtype=np.float64),
        corners_raw_mm=np.asarray(payload["corners_raw_m"], dtype=np.float64) * 1000.0,
        rectangle_score=float(payload["rectangle_score"]),
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Adapt one ChingMu MCAvatar stream to RobotBridge2's existing "
            "vicon_state_data contract"
        )
    )
    parser.add_argument(
        "--sdk-library",
        type=Path,
        default=ROOT / "chingmu_sdk" / "ChingmuDLL" / "libCMVrpn.so",
    )
    parser.add_argument("--host", default="192.168.2.100")
    parser.add_argument("--base-subject", default="G1Pelvis")
    parser.add_argument(
        "--body-id",
        type=int,
        help=(
            "MCAvatar server Person ID used for polling rigid-body pose; "
            "defaults to the resolved hierarchy sensor id"
        ),
    )
    parser.set_defaults(poll_body_pose=True)
    parser.add_argument(
        "--poll-body-pose",
        dest="poll_body_pose",
        action="store_true",
        help="Poll rigid-body pose with CMTrackerExternTC when callback pose is absent.",
    )
    parser.add_argument(
        "--no-poll-body-pose",
        dest="poll_body_pose",
        action="store_false",
        help="Skip CMTrackerExternTC pose polling; useful for throughput/drop diagnostics.",
    )
    parser.add_argument(
        "--body-pose-poll-hz",
        type=float,
        default=60.0,
        help="Background CMTrackerExternTC polling rate for rigid-body pose.",
    )
    parser.add_argument("--table-calib", type=Path)
    parser.add_argument("--save-table-calib", type=Path)
    parser.add_argument("--calib-sec", type=float, default=2.0)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--print-hz", type=float, default=10.0)
    parser.add_argument("--table-length", type=float, default=2.730738)
    parser.add_argument("--table-width", type=float, default=1.512451)
    parser.add_argument("--table-height", type=float, default=0.760000)
    parser.add_argument("--source-rate-hz", type=float, default=360.0)
    parser.add_argument("--corner-exclusion-radius-mm", type=float, default=50.0)
    parser.add_argument(
        "--lcm-url", default="udpm://239.255.76.67:7667?ttl=255"
    )
    parser.add_argument("--channel", default="vicon_state_data")
    parser.set_defaults(publish=False)
    parser.add_argument("--publish", dest="publish", action="store_true")
    parser.add_argument("--no-publish", dest="publish", action="store_false")
    return parser


def _rectangle_score(points_mm: np.ndarray, config: BridgeConfig) -> float:
    distances = sorted(
        float(np.linalg.norm(points_mm[first] - points_mm[second]))
        for first in range(4)
        for second in range(first + 1, 4)
    )
    short = min(config.table_length_m, config.table_width_m) * 1000.0
    long = max(config.table_length_m, config.table_width_m) * 1000.0
    diagonal = float(np.hypot(short, long))
    expected = (short, short, long, long, diagonal, diagonal)
    score = sum(abs(value - target) / target for value, target in zip(distances, expected))
    score += float(np.ptp(points_mm[:, 2])) / 300.0
    return float(score)


def infer_table_frame(
    stable_points_mm,
    robot_raw_mm,
    config: BridgeConfig,
) -> TableFrame:
    points = np.asarray(stable_points_mm, dtype=np.float64).reshape(-1, 3)
    if len(points) < 4:
        raise ValueError(f"need at least four stable table markers, got {len(points)}")

    best_points = None
    best_score = float("inf")
    for indices in itertools.combinations(range(len(points)), 4):
        candidate = points[list(indices)]
        score = _rectangle_score(candidate, config)
        if score < best_score:
            best_points = candidate.copy()
            best_score = score
    if best_points is None:
        raise ValueError("could not infer table rectangle")

    center = best_points.mean(axis=0)
    centered = best_points - center
    _left, _singular_values, right_t = np.linalg.svd(
        centered, full_matrices=False
    )
    z_axis = right_t[-1].copy()
    if z_axis[2] < 0.0:
        z_axis = -z_axis
    z_axis /= np.linalg.norm(z_axis)

    x_axis = right_t[0] - float(np.dot(right_t[0], z_axis)) * z_axis
    x_axis /= np.linalg.norm(x_axis)
    robot_delta = np.asarray(robot_raw_mm, dtype=np.float64).reshape(3) - center
    if float(np.dot(robot_delta, x_axis)) > 0.0:
        x_axis = -x_axis
    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    x_axis = np.cross(y_axis, z_axis)
    x_axis /= np.linalg.norm(x_axis)

    return TableFrame(
        center_raw_mm=center,
        x_axis_raw=x_axis,
        y_axis_raw=y_axis,
        z_axis_raw=z_axis,
        corners_raw_mm=best_points,
        rectangle_score=best_score,
    )


def raw_to_table_world(raw_mm, table: TableFrame, config: BridgeConfig) -> np.ndarray:
    points = np.asarray(raw_mm, dtype=np.float64)
    original_shape = points.shape
    flat = points.reshape(-1, 3)
    relative = flat - table.center_raw_mm
    world = np.column_stack(
        [
            0.5 * config.table_length_m + relative @ table.x_axis_raw * 0.001,
            relative @ table.y_axis_raw * 0.001,
            config.table_height_m + relative @ table.z_axis_raw * 0.001,
        ]
    )
    return world.reshape(original_shape)


def raw_rotation_to_table_world(table: TableFrame) -> np.ndarray:
    return np.vstack(
        [table.x_axis_raw, table.y_axis_raw, table.z_axis_raw]
    )


def normalize_quaternion_xyzw(value) -> Optional[np.ndarray]:
    try:
        quaternion = np.asarray(value, dtype=np.float64).reshape(4)
    except (TypeError, ValueError):
        return None
    norm = float(np.linalg.norm(quaternion))
    if not np.isfinite(quaternion).all() or norm < 1.0e-9:
        return None
    quaternion = quaternion / norm
    return -quaternion if quaternion[3] < 0.0 else quaternion


def body_pose_to_table_world(
    position_mm,
    quaternion_xyzw,
    table: TableFrame,
    config: BridgeConfig,
):
    quaternion = normalize_quaternion_xyzw(quaternion_xyzw)
    try:
        position = np.asarray(position_mm, dtype=np.float64).reshape(3)
    except (TypeError, ValueError):
        return None
    if quaternion is None or not np.isfinite(position).all():
        return None
    rotation_world = (
        raw_rotation_to_table_world(table)
        @ Rotation.from_quat(quaternion).as_matrix()
    )
    return raw_to_table_world(position, table, config), rotation_world


def yaw_quaternion_from_rotation(rotation) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    yaw = float(np.arctan2(matrix[1, 0], matrix[0, 0]))
    half = 0.5 * yaw
    quaternion = np.array([0.0, 0.0, np.sin(half), np.cos(half)], dtype=np.float64)
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    return quaternion


@dataclass(frozen=True)
class BallTrackUpdate:
    position_world: Optional[np.ndarray]
    ended: bool


class BallTracker:
    def __init__(self):
        self.reset()

    def reset(self) -> None:
        self.position_world = None
        self.velocity_world = None
        self.frame_number = None

    def update(
        self,
        unlabeled_markers_mm,
        *,
        frame_number: int,
        table: TableFrame,
        config: BridgeConfig,
    ) -> BallTrackUpdate:
        candidates = []
        for raw_marker in np.asarray(unlabeled_markers_mm, dtype=np.float64).reshape(-1, 3):
            corner_distance = float(
                np.min(np.linalg.norm(table.corners_raw_mm - raw_marker, axis=1))
            )
            if corner_distance <= config.corner_exclusion_radius_mm:
                continue
            world = raw_to_table_world(raw_marker, table, config)
            if not np.isfinite(world).all():
                continue
            if world[0] > config.table_length_m:
                continue
            if abs(float(world[1])) > 0.5 * config.table_width_m:
                continue
            if world[2] <= config.table_height_m:
                continue
            candidates.append(world)

        active = self.position_world is not None and self.frame_number is not None
        if not candidates:
            if active:
                self.reset()
                return BallTrackUpdate(position_world=None, ended=True)
            return BallTrackUpdate(position_world=None, ended=False)

        candidates_array = np.asarray(candidates, dtype=np.float64)
        if not active:
            admitted = candidates_array[candidates_array[:, 0] > 0.0]
            if len(admitted) == 0:
                return BallTrackUpdate(position_world=None, ended=False)
            selected = admitted[0]
        else:
            frame_delta = max(int(frame_number) - int(self.frame_number), 1)
            delta_time = frame_delta / config.source_rate_hz
            predicted = self.position_world
            if self.velocity_world is not None:
                predicted = predicted + delta_time * self.velocity_world
            selected = candidates_array[
                int(np.argmin(np.linalg.norm(candidates_array - predicted, axis=1)))
            ]
            if selected[0] <= 0.0:
                self.reset()
                return BallTrackUpdate(position_world=None, ended=True)
            self.velocity_world = (selected - self.position_world) / delta_time
        self.position_world = selected.copy()
        self.frame_number = int(frame_number)
        return BallTrackUpdate(
            position_world=selected.copy(),
            ended=False,
        )


def make_message(
    name,
    position_world,
    quaternion_xyzw,
    *,
    frame_number: int,
    source_time_s: float,
    valid: bool,
    publish_time_us: Optional[int] = None,
):
    message = transformation_t()
    message.name = str(name)
    message.vicon_frame_number = int(frame_number)
    message.vicon_time_s = float(source_time_s)
    message.publish_time_us = int(
        time.time() * 1.0e6 if publish_time_us is None else publish_time_us
    )
    message.valid = int(bool(valid))
    message.occluded = int(not bool(valid))
    message.pos_vicon = np.asarray(position_world, dtype=np.float64).reshape(3).tolist()
    message.quat_vicon = np.asarray(quaternion_xyzw, dtype=np.float64).reshape(4).tolist()
    return message


class ChingMuTableLcmBridge:
    def __init__(
        self,
        *,
        config: BridgeConfig,
        table_frame: TableFrame,
    ):
        self.config = config
        self.table_frame = table_frame
        self.initial_base_yaw = None
        self._base_pose_was_valid = False
        self.ball_tracker = BallTracker()

    def process_frame(self, frame, *, publish_time_us: Optional[int] = None):
        messages = []
        body_pose = body_pose_to_table_world(
            frame.body_position_mm,
            frame.body_quaternion_xyzw,
            self.table_frame,
            self.config,
        )
        if body_pose is not None:
            base_world, base_rotation = body_pose
            base_yaw = float(
                np.arctan2(base_rotation[1, 0], base_rotation[0, 0])
            )
            if self.initial_base_yaw is None:
                self.initial_base_yaw = base_yaw
            relative_yaw = float(
                np.arctan2(
                    np.sin(base_yaw - self.initial_base_yaw),
                    np.cos(base_yaw - self.initial_base_yaw),
                )
            )
            heading_rotation = Rotation.from_euler(
                "z", relative_yaw
            ).as_matrix()
            pelvis_offset_world = heading_rotation @ np.asarray(
                self.config.pelvis_offset_heading_m,
                dtype=np.float64,
            )
            messages.append(
                make_message(
                    self.config.base_subject,
                    base_world + pelvis_offset_world,
                    yaw_quaternion_from_rotation(heading_rotation),
                    frame_number=frame.frame_number,
                    source_time_s=frame.source_time_s,
                    valid=True,
                    publish_time_us=publish_time_us,
                )
            )
            self._base_pose_was_valid = True
        elif self._base_pose_was_valid:
            # The consumer ignores the pose when valid=0. Keep the transition
            # payload finite and schema-correct without fabricating a marker
            # centroid fallback.
            messages.append(
                make_message(
                    self.config.base_subject,
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0],
                    frame_number=frame.frame_number,
                    source_time_s=frame.source_time_s,
                    valid=False,
                    publish_time_us=publish_time_us,
                )
            )
            self._base_pose_was_valid = False

        ball_update = self.ball_tracker.update(
            frame.unlabeled_markers_mm,
            frame_number=frame.frame_number,
            table=self.table_frame,
            config=self.config,
        )
        if ball_update.position_world is not None:
            messages.append(
                make_message(
                    "ball",
                    ball_update.position_world,
                    [0.0, 0.0, 0.0, 1.0],
                    frame_number=frame.frame_number,
                    source_time_s=frame.source_time_s,
                    valid=True,
                    publish_time_us=publish_time_us,
                )
            )
        elif ball_update.ended:
            messages.append(
                make_message(
                    "ball",
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0],
                    frame_number=frame.frame_number,
                    source_time_s=frame.source_time_s,
                    valid=False,
                    publish_time_us=publish_time_us,
                )
            )
        messages.append(
            make_message(
                "table",
                [
                    0.5 * self.config.table_length_m,
                    0.0,
                    self.config.table_height_m,
                ],
                [0.0, 0.0, 0.0, 1.0],
                frame_number=frame.frame_number,
                source_time_s=frame.source_time_s,
                valid=True,
                publish_time_us=publish_time_us,
            )
        )
        return messages


def emit_messages(lc_client, messages, config: BridgeConfig, enabled: bool) -> int:
    if not enabled:
        return 0
    count = 0
    for message in messages:
        lc_client.publish(config.channel, message.encode())
        count += 1
    return count


def _collect_calibration_frames(client, duration_s: float) -> list:
    if duration_s <= 0.0:
        raise ValueError("--calib-sec must be positive")
    frames = []
    start = time.monotonic()
    while time.monotonic() - start < duration_s:
        frame = client.next_frame(timeout_s=0.1)
        if frame is not None:
            frames.append(frame)
    return frames


def _config_from_args(args) -> BridgeConfig:
    return BridgeConfig(
        base_subject=args.base_subject,
        channel=args.channel,
        lcm_url=args.lcm_url,
        table_length_m=args.table_length,
        table_width_m=args.table_width,
        table_height_m=args.table_height,
        source_rate_hz=args.source_rate_hz,
        corner_exclusion_radius_mm=args.corner_exclusion_radius_mm,
    )


def _describe_table(table: TableFrame, config: BridgeConfig) -> None:
    center_m = table.center_raw_mm * 0.001
    print(
        "Table calibration: "
        f"score={table.rectangle_score:.4f} "
        f"center_raw_m={np.array2string(center_m, precision=4)} "
        f"x_axis_raw={np.array2string(table.x_axis_raw, precision=4)} "
        f"y_axis_raw={np.array2string(table.y_axis_raw, precision=4)}",
        flush=True,
    )
    print(
        "RobotBridge table world: "
        f"robot-side edge x=0.0000 m, far edge x={config.table_length_m:.4f} m, "
        f"center=[{0.5 * config.table_length_m:.4f}, 0.0000, "
        f"{config.table_height_m:.4f}] m",
        flush=True,
    )
    for index, corner_raw_mm in enumerate(table.corners_raw_mm):
        corner_world = raw_to_table_world(corner_raw_mm, table, config)
        print(
            f"  table_corner_{index}_world_m="
            f"{np.array2string(corner_world, precision=4)}",
            flush=True,
        )


def main(argv=None) -> int:
    args = build_arg_parser().parse_args(argv)
    config = _config_from_args(args)
    if config.source_rate_hz <= 0.0:
        raise ValueError("--source-rate-hz must be positive")
    if args.body_pose_poll_hz <= 0.0:
        raise ValueError("--body-pose-poll-hz must be positive")

    from deploy.mocap_bridge.chingmu_sdk_client import ChingMuSdkClient

    client = ChingMuSdkClient(
        args.sdk_library,
        server_ip=args.host,
        body_name=config.base_subject,
        body_pose_id=args.body_id,
        poll_body_pose=args.poll_body_pose,
        body_pose_poll_hz=args.body_pose_poll_hz,
    )
    try:
        print(
            f"Connecting to MCAvatar@{args.host} with {args.sdk_library} ...",
            flush=True,
        )
        client.start()
        print(
            f"Resolved {config.base_subject} body_id={client.body_id}; "
            f"pose_body_id={client.body_pose_id}; "
            f"source rate={config.source_rate_hz:.1f} Hz",
            flush=True,
        )

        if args.table_calib is None:
            print(
                f"Calibrating table from stationary unlabeled markers for "
                f"{args.calib_sec:.2f} s ...",
                flush=True,
            )
            frames = _collect_calibration_frames(client, args.calib_sec)
            calibration = calibrate_from_frames(frames, config)
            table = calibration.table_frame
            if args.save_table_calib is not None:
                save_table_frame(args.save_table_calib, table, config)
                print(f"Saved table calibration to {args.save_table_calib}", flush=True)
        else:
            table = load_table_frame(args.table_calib, config)
            print(f"Loaded table calibration from {args.table_calib}", flush=True)
        _describe_table(table, config)

        lc_client = None
        if args.publish:
            import lcm

            lc_client = lcm.LCM(config.lcm_url)
        print(
            f"{'Publishing' if args.publish else 'Monitoring only'} "
            f"channel={config.channel} via {config.lcm_url}",
            flush=True,
        )

        bridge = ChingMuTableLcmBridge(
            config=config,
            table_frame=table,
        )
        start = time.monotonic()
        last_print = start - 10.0
        frame_count = 0
        while True:
            now = time.monotonic()
            elapsed = now - start
            if args.duration > 0.0 and elapsed >= args.duration:
                break
            frame = client.next_frame(timeout_s=0.1)
            if frame is None:
                continue
            frame_count += 1
            messages = bridge.process_frame(frame)
            emit_messages(lc_client, messages, config, args.publish)

            print_period = 1.0 / max(args.print_hz, 1.0e-6)
            if now - last_print >= print_period:
                by_name = {message.name: message for message in messages}
                base = by_name.get(config.base_subject)
                ball = by_name.get("ball")
                base_text = (
                    "missing"
                    if base is None
                    else np.array2string(
                        np.asarray(base.pos_vicon), precision=4, suppress_small=True
                    )
                )
                ball_text = (
                    "missing"
                    if ball is None
                    else np.array2string(
                        np.asarray(ball.pos_vicon), precision=4, suppress_small=True
                    )
                )
                rate = frame_count / max(elapsed, 1.0e-6)
                print(
                    f"frame={frame.frame_number} hz={rate:.1f} "
                    f"dropped={client.dropped_frame_count} "
                    f"body_markers={len(frame.body_markers_mm)} "
                    f"unlabeled={len(frame.unlabeled_markers_mm)} "
                    f"{config.base_subject}={base_text} ball={ball_text}",
                    flush=True,
                )
                last_print = now
    except KeyboardInterrupt:
        print("Stopped by user", flush=True)
    finally:
        client.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
