#!/usr/bin/env python3
from __future__ import annotations

import argparse
import select
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import lcm
import numpy as np
from omegaconf import OmegaConf


ROOT = Path(__file__).resolve().parents[2]
for import_path in (ROOT, ROOT / "deploy"):
    import_path_text = str(import_path)
    if import_path_text not in sys.path:
        sys.path.insert(0, import_path_text)

from unitree_sdk2.lcm_types.transformation_t import transformation_t
from utils.hitter_planner import (
    BallStateEstimator,
    BallTrajectoryPredictor,
    BaseTargetPlanner,
    HitterSystemPlanner,
    StrikePlanner,
)
from utils.hitter_realtime import (
    HitterCommandLifecycle,
    IncomingTrackConfirmation,
    PlannerResultSnapshot,
)
from utils.transformation import matrix_from_quat


DEFAULT_CONFIG = ROOT / "deploy/config/mimic/hitter.yaml"
DEFAULT_LCM_URL = "udpm://239.255.76.67:7667?ttl=255"


@dataclass(frozen=True)
class PredictionReport:
    frame: int
    track_epoch: int
    generation: int
    status: str
    sample_count: int
    estimator_ready: bool
    bounce_detected: bool
    estimated_position_w: Optional[np.ndarray] = None
    estimated_velocity_w: Optional[np.ndarray] = None
    error: Optional[str] = None
    decision: Optional[str] = None
    time_to_strike_s: Optional[float] = None
    policy_time_to_strike_s: Optional[float] = None
    strike_type: Optional[str] = None
    strike_position_w: Optional[np.ndarray] = None
    ball_in_velocity_w: Optional[np.ndarray] = None
    ball_out_velocity_w: Optional[np.ndarray] = None
    racket_velocity_w: Optional[np.ndarray] = None
    base_target_xy_w: Optional[np.ndarray] = None


def load_motion_config(path: Path) -> dict:
    config = OmegaConf.to_container(OmegaConf.load(path), resolve=True)
    if not isinstance(config, dict) or not isinstance(config.get("motion"), dict):
        raise ValueError(f"{path} does not contain a motion mapping")
    return config["motion"]


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "Monitor HITTER ball estimation and strike prediction from the shared "
            "vicon_state_data LCM contract without loading or running a policy."
        )
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--lcm-url", default=DEFAULT_LCM_URL)
    parser.add_argument("--channel", default="vicon_state_data")
    parser.add_argument("--base-name", default="G1Pelvis")
    parser.add_argument("--ball-name", default="ball")
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Optional run duration in seconds; zero runs until Ctrl+C.",
    )
    return parser.parse_args(argv)


def _message_int(msg, name: str, default: int = 0) -> int:
    try:
        return int(getattr(msg, name))
    except (AttributeError, TypeError, ValueError):
        return default


def _message_float(msg, name: str) -> Optional[float]:
    try:
        value = float(getattr(msg, name))
    except (AttributeError, TypeError, ValueError):
        return None
    return value if np.isfinite(value) else None


def _vector(value, size: int) -> Optional[np.ndarray]:
    try:
        vector = np.asarray(value, dtype=np.float64).reshape(-1)[:size]
    except (TypeError, ValueError):
        return None
    if vector.shape != (size,) or not np.isfinite(vector).all():
        return None
    return vector.copy()


def _format_vector(value: Optional[np.ndarray]) -> str:
    if value is None:
        return "none"
    return "[" + ",".join(f"{float(item):.4f}" for item in value) + "]"


def format_report(report: PredictionReport) -> str:
    fields = [
        f"frame={report.frame}",
        f"epoch={report.track_epoch}",
        f"generation={report.generation}",
        f"status={report.status}",
        f"samples={report.sample_count}",
        f"ready={int(report.estimator_ready)}",
        f"bounce={int(report.bounce_detected)}",
    ]
    if report.estimated_position_w is not None:
        fields.append(f"est_pos_w={_format_vector(report.estimated_position_w)}")
    if report.estimated_velocity_w is not None:
        fields.append(f"est_vel_w={_format_vector(report.estimated_velocity_w)}")
    if report.error is not None:
        fields.append(f"error={report.error!r}")
    if report.decision is not None:
        fields.append(f"decision={report.decision}")
    if report.time_to_strike_s is not None:
        fields.append(f"tts_s={report.time_to_strike_s:.3f}")
    if report.policy_time_to_strike_s is not None:
        fields.append(f"policy_tts_s={report.policy_time_to_strike_s:.3f}")
    if report.strike_type is not None:
        fields.append(f"strike_type={report.strike_type}")
    if report.strike_position_w is not None:
        fields.append(f"hit_w={_format_vector(report.strike_position_w)}")
    if report.ball_in_velocity_w is not None:
        fields.append(f"ball_in_v_w={_format_vector(report.ball_in_velocity_w)}")
    if report.ball_out_velocity_w is not None:
        fields.append(f"ball_out_v_w={_format_vector(report.ball_out_velocity_w)}")
    if report.racket_velocity_w is not None:
        fields.append(f"racket_v_w={_format_vector(report.racket_velocity_w)}")
    if report.base_target_xy_w is not None:
        fields.append(f"base_target_xy_w={_format_vector(report.base_target_xy_w)}")
    return " ".join(fields)


class PredictionMonitor:
    def __init__(
        self,
        motion_config: dict,
        *,
        base_name: str = "G1Pelvis",
        ball_name: str = "ball",
        estimator=None,
        planner=None,
    ):
        self.motion_config = dict(motion_config)
        self.planner_config = dict(self.motion_config.get("ball_planner", {}) or {})
        self.base_name = str(base_name).strip().lower()
        self.ball_name = str(ball_name).strip().lower()

        cfg = self.planner_config
        table_height = float(cfg.get("table_height", 0.76))
        table_center = cfg.get("table_center_xy_w", [1.37, 0.0])
        table_length = float(cfg.get("table_length", 2.74))
        table_width = float(cfg.get("table_width", 1.525))
        ball_radius = float(cfg.get("ball_radius", 0.02))
        estimator_window = int(cfg.get("state_estimator_window_size", 31))

        self.estimator = estimator or BallStateEstimator(
            window_size=estimator_window,
            min_samples=int(cfg.get("state_estimator_min_samples", estimator_window)),
            table_height=table_height,
            table_center_xy=table_center,
            table_length=table_length,
            table_width=table_width,
            ball_radius=ball_radius,
            bounce_height_tolerance=float(
                cfg.get("state_estimator_bounce_height_tolerance", 0.03)
            ),
            bounce_velocity_threshold=float(
                cfg.get("state_estimator_bounce_velocity_threshold", 0.10)
            ),
            bounce_min_separation_s=float(
                cfg.get("state_estimator_bounce_min_separation_s", 0.20)
            ),
        )
        self.planner = planner or self._build_planner()
        self.incoming_confirmation = IncomingTrackConfirmation(
            minimum_speed_x_mps=float(
                cfg.get("minimum_stable_incoming_speed_x_mps", 0.20)
            ),
            required_consecutive_snapshots=int(
                cfg.get("stable_incoming_confirmation_snapshots", 3)
            ),
        )
        waiting_tts = float(self.motion_config.get("waiting_time_to_strike_s", 0.92))
        self.lifecycle = HitterCommandLifecycle(
            waiting_tts=waiting_tts,
            arm_tts=float(cfg.get("arm_time_to_strike_s", 0.92)),
            minimum_arm_tts=float(cfg.get("minimum_arm_time_to_strike_s", 0.60)),
            maximum_policy_tts=float(cfg.get("maximum_policy_time_to_strike_s", 0.92)),
            swing_duration_sampler=lambda: float(
                np.mean(cfg.get("swing_duration_range", [1.75, 1.95]))
            ),
        )
        planner_rate = float(cfg.get("planner_update_rate_hz", 100.0))
        if not np.isfinite(planner_rate) or planner_rate <= 0.0:
            raise ValueError("planner_update_rate_hz must be finite and positive")
        self.planner_interval_s = 1.0 / planner_rate

        self.base_position_w = np.zeros(3, dtype=np.float64)
        self.base_quaternion_xyzw = np.array([0.0, 0.0, 0.0, 1.0])
        self.base_valid = False
        self.track_epoch = 0
        self.generation = 0
        self.last_submit_monotonic_s: Optional[float] = None

    def _build_planner(self) -> HitterSystemPlanner:
        cfg = self.planner_config
        table_height = float(cfg.get("table_height", 0.76))
        predictor = BallTrajectoryPredictor(
            gravity=cfg.get("gravity", [0.0, 0.0, -9.81]),
            drag_coefficient=float(cfg.get("drag_coefficient", 0.0)),
            vertical_restitution=float(cfg.get("vertical_restitution", 0.8)),
            horizontal_restitution=float(cfg.get("horizontal_restitution", 0.9)),
            dt=float(cfg.get("prediction_dt", 0.005)),
            table_height=table_height,
            table_center_xy=cfg.get("table_center_xy_w", [1.37, 0.0]),
            table_length=float(cfg.get("table_length", 2.74)),
            table_width=float(cfg.get("table_width", 1.525)),
            ball_radius=float(cfg.get("ball_radius", 0.02)),
        )
        maximum_hit_height = round(
            table_height + float(cfg.get("maximum_hit_height_above_table_m", 0.50)),
            6,
        )
        strike_planner = StrikePlanner(
            predictor=predictor,
            virtual_hit_plane_x=float(cfg.get("virtual_hit_plane_x", 0.0)),
            desired_landing_point=cfg.get("desired_landing_point_w", [2.05, 0.0, 0.78]),
            post_hit_flight_time=float(cfg.get("post_hit_flight_time", 0.55)),
            racket_restitution=float(cfg.get("racket_restitution", 0.85)),
            prediction_horizon_s=float(cfg.get("prediction_horizon_s", 2.0)),
            maximum_prediction_horizon_s=float(
                cfg.get("maximum_prediction_horizon_s", 5.0)
            ),
            minimum_hit_height=table_height,
            maximum_hit_height=maximum_hit_height,
            require_future_hit_plane_crossing=bool(
                cfg.get("require_future_hit_plane_crossing", False)
            ),
        )
        base_planner = BaseTargetPlanner(
            racket_x_offset_b=float(cfg.get("racket_x_offset_b", 0.40)),
            forehand_nominal_racket_y_b=float(
                cfg.get("forehand_nominal_racket_y_b", -0.5)
            ),
            backhand_nominal_racket_y_b=float(
                cfg.get("backhand_nominal_racket_y_b", 0.22)
            ),
            default_base_z=float(cfg.get("target_base_height_w", 0.793)),
        )
        return HitterSystemPlanner(
            strike_planner=strike_planner,
            base_planner=base_planner,
        )

    def _source_timestamp(self, msg, received_monotonic_s: float) -> float:
        vicon_time = _message_float(msg, "vicon_time_s")
        if vicon_time is not None and vicon_time > 0.0:
            return vicon_time
        publish_time_us = _message_int(msg, "publish_time_us", 0)
        if publish_time_us > 0:
            return publish_time_us * 1.0e-6
        return float(received_monotonic_s)

    def _handle_base(self, msg) -> None:
        position = _vector(getattr(msg, "pos_vicon", None), 3)
        quaternion = _vector(getattr(msg, "quat_vicon", None), 4)
        valid = bool(_message_int(msg, "valid", 1)) and not bool(
            _message_int(msg, "occluded", 0)
        )
        if position is None or quaternion is None:
            valid = False
        if valid:
            quaternion_norm = float(np.linalg.norm(quaternion))
            if quaternion_norm < 1.0e-9:
                valid = False
            else:
                quaternion = quaternion / quaternion_norm
        self.base_valid = bool(valid)
        if self.base_valid:
            self.base_position_w = position
            self.base_quaternion_xyzw = quaternion

    def _report(
        self,
        *,
        msg,
        status: str,
        estimate=None,
        error: Optional[str] = None,
        **kwargs,
    ) -> PredictionReport:
        return PredictionReport(
            frame=_message_int(msg, "vicon_frame_number", 0),
            track_epoch=int(self.track_epoch),
            generation=int(self.generation),
            status=status,
            sample_count=0 if estimate is None else int(estimate.sample_count),
            estimator_ready=False if estimate is None else bool(estimate.valid),
            bounce_detected=False if estimate is None else bool(estimate.bounce_detected),
            estimated_position_w=(
                None if estimate is None else np.asarray(estimate.position).copy()
            ),
            estimated_velocity_w=(
                None if estimate is None else np.asarray(estimate.velocity).copy()
            ),
            error=error,
            **kwargs,
        )

    def handle_message(
        self,
        msg,
        *,
        received_monotonic_s: Optional[float] = None,
    ) -> Optional[PredictionReport]:
        received = (
            time.monotonic()
            if received_monotonic_s is None
            else float(received_monotonic_s)
        )
        name = str(getattr(msg, "name", "") or "").strip().lower()
        if name == self.base_name:
            self._handle_base(msg)
            return None
        if name != self.ball_name:
            return None

        self.generation += 1
        valid = bool(_message_int(msg, "valid", 1)) and not bool(
            _message_int(msg, "occluded", 0)
        )
        position = _vector(getattr(msg, "pos_vicon", None), 3)
        if not valid or position is None:
            report = self._report(msg=msg, status="track-ended")
            self.estimator.reset()
            self.incoming_confirmation.reset()
            self.lifecycle.mark_track_ended(self.track_epoch)
            self.track_epoch += 1
            return report

        estimate = self.estimator.add_sample(
            position,
            timestamp=self._source_timestamp(msg, received),
        )
        last_submit = self.last_submit_monotonic_s
        if (
            last_submit is not None
            and received - last_submit < self.planner_interval_s - 1.0e-12
        ):
            return None
        self.last_submit_monotonic_s = received
        self.lifecycle.advance(received)

        if not estimate.valid:
            return self._report(msg=msg, status="estimating", estimate=estimate)
        if not self.base_valid:
            return self._report(
                msg=msg,
                status="rejected",
                estimate=estimate,
                error="base pose is not valid",
            )
        if not self.incoming_confirmation.observe(
            track_epoch=self.track_epoch,
            velocity_x_mps=float(estimate.velocity[0]),
        ):
            return self._report(
                msg=msg,
                status="rejected",
                estimate=estimate,
                error=(
                    "ball track is not stably incoming: "
                    f"vx={float(estimate.velocity[0]):.3f} m/s"
                ),
            )

        try:
            base_forward_xy = matrix_from_quat(self.base_quaternion_xyzw)[:, 0][:2]
            forced = self.planner_config.get(
                "force_strike_type",
                self.motion_config.get("force_strike_type"),
            )
            command = self.planner.plan_command(
                estimate.position,
                estimate.velocity,
                current_base_xy_w=self.base_position_w,
                base_forward_xy_w=base_forward_xy,
                strike_type=None if forced in (None, "", "none", "null") else forced,
            )
        except Exception as exc:
            return self._report(
                msg=msg,
                status="rejected",
                estimate=estimate,
                error=f"{type(exc).__name__}: {exc}",
            )

        deadline = received + float(command.time_to_strike)
        result = PlannerResultSnapshot(
            track_epoch=self.track_epoch,
            source_generation=self.generation,
            source_frame=_message_int(msg, "vicon_frame_number", 0),
            strike_deadline_monotonic_s=deadline,
            completed_monotonic_s=received,
            command=command,
        )
        decision = self.lifecycle.ingest(result, now=received)
        strike_plan = command.strike_plan
        return self._report(
            msg=msg,
            status="planned",
            estimate=estimate,
            decision=decision,
            time_to_strike_s=float(command.time_to_strike),
            policy_time_to_strike_s=self.lifecycle.policy_tts(now=received),
            strike_type=str(command.strike_type),
            strike_position_w=np.asarray(strike_plan.p_racket_target).copy(),
            ball_in_velocity_w=np.asarray(strike_plan.v_ball_in).copy(),
            ball_out_velocity_w=np.asarray(strike_plan.v_ball_out).copy(),
            racket_velocity_w=np.asarray(command.v_racket_target_w).copy(),
            base_target_xy_w=np.asarray(command.p_base_target_xy).copy(),
        )


def main(argv=None) -> int:
    args = parse_args(argv)
    if not np.isfinite(args.duration) or args.duration < 0.0:
        raise ValueError("--duration must be finite and non-negative")

    motion = load_motion_config(args.config.expanduser().resolve())
    monitor = PredictionMonitor(
        motion,
        base_name=args.base_name,
        ball_name=args.ball_name,
    )
    lcm_client = lcm.LCM(args.lcm_url)
    report_count = 0
    start = time.monotonic()

    def handler(_channel, data):
        nonlocal report_count
        report = monitor.handle_message(transformation_t.decode(data))
        if report is not None:
            report_count += 1
            print(format_report(report), flush=True)

    lcm_client.subscribe(args.channel, handler)
    print(
        f"Listening for base={args.base_name!r} ball={args.ball_name!r} "
        f"on channel={args.channel} via {args.lcm_url}",
        flush=True,
    )
    print(
        "Planner-only monitor active: no ONNX policy and no control commands.",
        flush=True,
    )
    try:
        while True:
            elapsed = time.monotonic() - start
            if args.duration > 0.0 and elapsed >= args.duration:
                break
            ready, _, _ = select.select([lcm_client.fileno()], [], [], 0.1)
            if ready:
                lcm_client.handle()
    except KeyboardInterrupt:
        pass

    print(
        f"Prediction monitor stopped: reports={report_count} "
        f"duration_s={time.monotonic() - start:.3f}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
