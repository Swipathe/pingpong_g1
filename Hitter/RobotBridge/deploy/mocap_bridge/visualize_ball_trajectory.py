#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import select
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, List, Optional, Sequence, Tuple

import numpy as np

ROBOTBRIDGE_DIR = Path(__file__).resolve().parents[2]
DEPLOY_DIR = ROBOTBRIDGE_DIR / "deploy"
for path in (ROBOTBRIDGE_DIR, DEPLOY_DIR):
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.insert(0, path_str)

from utils.hitter_planner import BallStateEstimator, BallTrajectoryPredictor, StrikePlanner
from mocap_bridge.evaluate_planner_paper_style import PlannerEvalRow, build_paper_style_summary


@dataclass(frozen=True)
class TrajectorySample:
    timestamp: float
    position: np.ndarray


@dataclass(frozen=True)
class VisualizerConfig:
    window_s: float = 3.0
    hold_last_trajectory_s: float = 60.0
    estimator_window: int = 31
    estimator_min_samples: Optional[int] = 31
    table_height: float = 0.76
    table_center_xy: Tuple[float, float] = (1.37, 0.0)
    table_length: float = 2.74
    table_width: float = 1.525
    ball_radius: float = 0.02
    bounce_height_tolerance: float = 0.03
    bounce_velocity_threshold: float = 0.10
    bounce_min_separation_s: float = 0.20
    max_sample_gap_s: Optional[float] = 0.25
    prediction_horizon_s: float = 1.2
    prediction_dt: float = 0.005
    virtual_hit_plane_x: float = 0.0
    desired_landing_point: Tuple[float, float, float] = (2.05, 0.0, 0.78)
    post_hit_flight_time: float = 0.48
    racket_restitution: float = 0.85
    drag_coefficient: float = 0.0
    horizontal_restitution: float = 1.0
    vertical_restitution: float = 0.8
    gravity: Tuple[float, float, float] = (0.0, 0.0, -9.81)


@dataclass(frozen=True)
class HitPrediction:
    created_time: float
    time_to_hit_s: float
    absolute_hit_time_s: float
    hit_position: np.ndarray
    incoming_velocity: np.ndarray
    outgoing_velocity: np.ndarray
    racket_velocity: np.ndarray


@dataclass(frozen=True)
class PlannerEvaluation:
    created_time: float
    time_to_hit_s: float
    predicted_hit_time_s: float
    actual_hit_time_s: float
    predicted_position: np.ndarray
    actual_position: np.ndarray
    position_error_m: float
    time_error_s: float


@dataclass(frozen=True)
class LiveStatus:
    elapsed_s: float
    total_messages: int
    base_messages: int
    table_messages: int
    ball_messages: int
    total_hz: float
    ball_hz: float
    last_message_age_s: float
    last_ball_age_s: float
    latest_base_position: Optional[np.ndarray] = None
    latest_table_position: Optional[np.ndarray] = None
    latest_ball_position: Optional[np.ndarray] = None


@dataclass(frozen=True)
class AnalyzerSnapshot:
    observed_times: np.ndarray
    observed_positions: np.ndarray
    estimate_position: np.ndarray
    estimate_velocity: np.ndarray
    estimate_valid: bool
    sample_count: int
    speed_mps: float
    prediction_times: np.ndarray
    predicted_positions: np.ndarray
    hit_prediction: Optional[HitPrediction]
    bounce_positions: np.ndarray
    last_bounce_detected: bool
    new_evaluations: Tuple[PlannerEvaluation, ...]
    latest_evaluation: Optional[PlannerEvaluation]
    evaluation_count: int
    position_error_median_m: float
    position_error_p90_m: float
    time_error_median_s: float
    time_error_p90_s: float
    evaluated_predicted_positions: np.ndarray
    evaluated_actual_positions: np.ndarray
    pending_hit_positions: np.ndarray
    paper_style_summary: Optional[dict]


def _float_or_none(value) -> Optional[float]:
    if value is None or value == "":
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _truthy_field(value) -> bool:
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

    bare = [_float_or_none(row.get(axis)) for axis in ("x", "y", "z")]
    if all(value is not None for value in bare):
        return np.asarray(bare, dtype=np.float64)
    return None


def load_ball_csv_samples(
    path: Path,
    *,
    sample_rate_hz: float = 300.0,
    ball_name: str = "ball",
    segment_id: Optional[int] = None,
    time_source: str = "frame",
) -> List[TrajectorySample]:
    samples: List[TrajectorySample] = []
    with Path(path).open(newline="") as handle:
        reader = csv.DictReader(handle)
        for index, row in enumerate(reader):
            if row.get("name") not in (None, "", ball_name):
                continue
            if segment_id is not None:
                value = _float_or_none(row.get("clean_segment_id"))
                if value is None or int(value) != int(segment_id):
                    continue
            if not _truthy_field(row.get("ball_valid")):
                continue
            if row.get("ball_occluded") is not None and _truthy_field(row.get("ball_occluded")):
                continue
            timestamp = _row_time(row, index, sample_rate_hz, time_source=time_source)
            position = _row_position_m(row)
            if timestamp is None or position is None or not np.isfinite(position).all():
                continue
            samples.append(TrajectorySample(float(timestamp), position))

    samples.sort(key=lambda sample: sample.timestamp)
    return samples


def next_stream_timestamp(
    previous_stream_time: Optional[float],
    previous_host_time: Optional[float],
    current_host_time: float,
    *,
    sample_rate_hz: float,
    max_sample_gap_s: Optional[float],
) -> float:
    if previous_stream_time is None or previous_host_time is None:
        return 0.0
    wall_dt = max(0.0, float(current_host_time - previous_host_time))
    nominal_dt = 1.0 / sample_rate_hz if sample_rate_hz > 0.0 else wall_dt
    if max_sample_gap_s is not None and wall_dt > float(max_sample_gap_s):
        return float(previous_stream_time + wall_dt)
    return float(previous_stream_time + nominal_dt)


def drain_lcm(lc, *, timeout_s: float, max_messages: int) -> int:
    if max_messages <= 0:
        return 0
    handled = 0
    timeout = max(0.0, float(timeout_s))
    while handled < max_messages:
        rfds, _, _ = select.select([lc.fileno()], [], [], timeout)
        if not rfds:
            break
        lc.handle()
        handled += 1
        timeout = 0.0
    return handled


def _msg_int_field(msg, name: str, default: int = 0) -> int:
    try:
        value = int(getattr(msg, name))
    except (AttributeError, TypeError, ValueError):
        return default
    return value


def _msg_float_field(msg, name: str) -> Optional[float]:
    try:
        value = float(getattr(msg, name))
    except (AttributeError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _msg_vicon_time(msg) -> Optional[float]:
    value = _msg_float_field(msg, "vicon_time_s")
    if value is None or value <= 0.0:
        return None
    return value


class BallTrajectoryAnalyzer:
    def __init__(self, config: VisualizerConfig):
        self.config = config
        self._samples: Deque[TrajectorySample] = deque()
        self._bounces: Deque[np.ndarray] = deque()
        self._pending_predictions: Deque[HitPrediction] = deque()
        self._evaluations: List[PlannerEvaluation] = []
        self._paper_style_eval_count = -1
        self._paper_style_summary: Optional[dict] = None
        self._estimator = BallStateEstimator(
            window_size=config.estimator_window,
            min_samples=config.estimator_min_samples,
            table_height=config.table_height,
            table_center_xy=config.table_center_xy,
            table_length=config.table_length,
            table_width=config.table_width,
            ball_radius=config.ball_radius,
            bounce_height_tolerance=config.bounce_height_tolerance,
            bounce_velocity_threshold=config.bounce_velocity_threshold,
            bounce_min_separation_s=config.bounce_min_separation_s,
            max_sample_gap_s=config.max_sample_gap_s,
        )
        self._predictor = BallTrajectoryPredictor(
            gravity=config.gravity,
            drag_coefficient=config.drag_coefficient,
            horizontal_restitution=config.horizontal_restitution,
            vertical_restitution=config.vertical_restitution,
            dt=config.prediction_dt,
            table_height=config.table_height,
            table_center_xy=config.table_center_xy,
            table_length=config.table_length,
            table_width=config.table_width,
            ball_radius=config.ball_radius,
        )
        self._strike_planner = StrikePlanner(
            predictor=self._predictor,
            virtual_hit_plane_x=config.virtual_hit_plane_x,
            desired_landing_point=config.desired_landing_point,
            post_hit_flight_time=config.post_hit_flight_time,
            racket_restitution=config.racket_restitution,
            prediction_horizon_s=config.prediction_horizon_s,
            require_future_hit_plane_crossing=True,
        )

    def reset(self) -> None:
        self._samples.clear()
        self._bounces.clear()
        self._pending_predictions.clear()
        self._evaluations.clear()
        self._paper_style_eval_count = -1
        self._paper_style_summary = None
        self._estimator.reset()

    def _start_new_track(self) -> None:
        self._samples.clear()
        self._bounces.clear()
        self._pending_predictions.clear()
        self._estimator.reset()

    def _trim(self, timestamp: float) -> None:
        cutoff = float(timestamp) - self.config.window_s
        while self._samples and self._samples[0].timestamp < cutoff:
            self._samples.popleft()
        while self._pending_predictions and self._pending_predictions[0].created_time < cutoff:
            self._pending_predictions.popleft()
        while self._bounces and self._samples:
            if len(self._samples) == 0:
                break
            if self._bounces[0][3] >= cutoff:
                break
            self._bounces.popleft()

    def _incoming_plane_crossing(
        self,
        previous: TrajectorySample,
        current: TrajectorySample,
    ) -> Optional[Tuple[float, np.ndarray]]:
        plane_x = self.config.virtual_hit_plane_x
        prev_x = float(previous.position[0] - plane_x)
        curr_x = float(current.position[0] - plane_x)
        if not (prev_x > 0.0 and curr_x <= 0.0 and current.position[0] < previous.position[0]):
            return None
        denom = prev_x - curr_x
        if abs(denom) < 1.0e-12:
            return None
        alpha = float(np.clip(prev_x / denom, 0.0, 1.0))
        timestamp = (1.0 - alpha) * previous.timestamp + alpha * current.timestamp
        position = (1.0 - alpha) * previous.position + alpha * current.position
        position = position.astype(np.float64, copy=True)
        position[0] = plane_x
        return float(timestamp), position

    def _evaluate_crossing(self, actual_time: float, actual_position: np.ndarray) -> Tuple[PlannerEvaluation, ...]:
        if not self._pending_predictions:
            return ()
        evaluations = []
        for prediction in list(self._pending_predictions):
            if prediction.created_time > actual_time:
                continue
            position_error = float(np.linalg.norm(prediction.hit_position[1:3] - actual_position[1:3]))
            time_error = float(abs(prediction.absolute_hit_time_s - actual_time))
            evaluations.append(
                PlannerEvaluation(
                    created_time=prediction.created_time,
                    time_to_hit_s=prediction.time_to_hit_s,
                    predicted_hit_time_s=prediction.absolute_hit_time_s,
                    actual_hit_time_s=float(actual_time),
                    predicted_position=prediction.hit_position.copy(),
                    actual_position=actual_position.copy(),
                    position_error_m=position_error,
                    time_error_s=time_error,
                )
            )
        self._pending_predictions.clear()
        self._evaluations.extend(evaluations)
        return tuple(evaluations)

    def _hit_prediction_from_estimate(self, estimate, current_time: float) -> Optional[HitPrediction]:
        if not estimate.valid:
            return None
        plane_x = self.config.virtual_hit_plane_x
        if not (float(estimate.position[0]) > plane_x and float(estimate.velocity[0]) < 0.0):
            return None
        try:
            strike_plan = self._strike_planner.plan(estimate.position, estimate.velocity)
        except Exception:
            return None
        return HitPrediction(
            created_time=float(current_time),
            time_to_hit_s=float(strike_plan.t_strike),
            absolute_hit_time_s=float(current_time + strike_plan.t_strike),
            hit_position=strike_plan.p_racket_target.copy(),
            incoming_velocity=strike_plan.v_ball_in.copy(),
            outgoing_velocity=strike_plan.v_ball_out.copy(),
            racket_velocity=strike_plan.v_racket_target.copy(),
        )

    def _evaluation_stats(self) -> Tuple[float, float, float, float]:
        if not self._evaluations:
            return float("nan"), float("nan"), float("nan"), float("nan")
        position_errors = np.asarray([item.position_error_m for item in self._evaluations], dtype=np.float64)
        time_errors = np.asarray([item.time_error_s for item in self._evaluations], dtype=np.float64)
        return (
            float(np.median(position_errors)),
            float(np.percentile(position_errors, 90)),
            float(np.median(time_errors)),
            float(np.percentile(time_errors, 90)),
        )

    def _paper_style_eval_rows(self) -> List[PlannerEvalRow]:
        return [
            PlannerEvalRow(
                created_time_s=float(item.created_time),
                predicted_hit_time_s=float(item.predicted_hit_time_s),
                actual_hit_time_s=float(item.actual_hit_time_s),
                position_error_cm=float(item.position_error_m * 100.0),
                time_error_ms=float(item.time_error_s * 1000.0),
            )
            for item in self._evaluations
        ]

    def _current_paper_style_summary(self) -> Optional[dict]:
        if not self._evaluations:
            self._paper_style_eval_count = 0
            self._paper_style_summary = None
            return None
        if self._paper_style_eval_count == len(self._evaluations):
            return self._paper_style_summary
        self._paper_style_summary = build_paper_style_summary(
            self._paper_style_eval_rows(),
            horizons_s=(0.5, 0.3, 0.1),
            bin_width_s=0.05,
            max_lead_s=1.0,
        )
        self._paper_style_eval_count = len(self._evaluations)
        return self._paper_style_summary

    def add_sample(self, position: Sequence[float], *, timestamp: float) -> AnalyzerSnapshot:
        pos = np.asarray(position, dtype=np.float64).reshape(3)
        current_sample = TrajectorySample(float(timestamp), pos.copy())
        previous_sample = self._samples[-1] if self._samples else None
        if (
            previous_sample is not None
            and self.config.max_sample_gap_s is not None
            and float(timestamp) - previous_sample.timestamp > self.config.max_sample_gap_s
        ):
            self._start_new_track()
            previous_sample = None
        estimate = self._estimator.add_sample(pos, timestamp=float(timestamp))
        self._samples.append(current_sample)
        if estimate.bounce_detected:
            self._bounces.append(np.asarray([pos[0], pos[1], pos[2], float(timestamp)], dtype=np.float64))

        new_evaluations: Tuple[PlannerEvaluation, ...] = ()
        if previous_sample is not None:
            crossing = self._incoming_plane_crossing(previous_sample, current_sample)
            if crossing is not None:
                actual_time, actual_position = crossing
                new_evaluations = self._evaluate_crossing(actual_time, actual_position)

        self._trim(float(timestamp))

        prediction_times = np.empty((0,), dtype=np.float64)
        predicted_positions = np.empty((0, 3), dtype=np.float64)
        hit_prediction = self._hit_prediction_from_estimate(estimate, float(timestamp))
        if estimate.valid:
            trajectory = self._predictor.predict(
                estimate.position,
                estimate.velocity,
                self.config.prediction_horizon_s,
            )
            prediction_times = trajectory.times
            predicted_positions = trajectory.positions
        if hit_prediction is not None:
            self._pending_predictions.append(hit_prediction)

        observed_times = np.asarray([sample.timestamp for sample in self._samples], dtype=np.float64)
        observed_positions = np.asarray([sample.position for sample in self._samples], dtype=np.float64)
        if observed_positions.size == 0:
            observed_positions = np.empty((0, 3), dtype=np.float64)

        bounce_positions = np.asarray([bounce[:3] for bounce in self._bounces], dtype=np.float64)
        if bounce_positions.size == 0:
            bounce_positions = np.empty((0, 3), dtype=np.float64)

        evaluated_predicted_positions = np.asarray([item.predicted_position for item in self._evaluations], dtype=np.float64)
        evaluated_actual_positions = np.asarray([item.actual_position for item in self._evaluations], dtype=np.float64)
        if evaluated_predicted_positions.size == 0:
            evaluated_predicted_positions = np.empty((0, 3), dtype=np.float64)
        if evaluated_actual_positions.size == 0:
            evaluated_actual_positions = np.empty((0, 3), dtype=np.float64)

        pending_hit_positions = np.asarray([item.hit_position for item in self._pending_predictions], dtype=np.float64)
        if pending_hit_positions.size == 0:
            pending_hit_positions = np.empty((0, 3), dtype=np.float64)

        pos_median, pos_p90, time_median, time_p90 = self._evaluation_stats()
        return AnalyzerSnapshot(
            observed_times=observed_times,
            observed_positions=observed_positions,
            estimate_position=estimate.position,
            estimate_velocity=estimate.velocity,
            estimate_valid=bool(estimate.valid),
            sample_count=int(estimate.sample_count),
            speed_mps=float(np.linalg.norm(estimate.velocity)),
            prediction_times=prediction_times,
            predicted_positions=predicted_positions,
            hit_prediction=hit_prediction,
            bounce_positions=bounce_positions,
            last_bounce_detected=bool(estimate.bounce_detected),
            new_evaluations=new_evaluations,
            latest_evaluation=self._evaluations[-1] if self._evaluations else None,
            evaluation_count=len(self._evaluations),
            position_error_median_m=pos_median,
            position_error_p90_m=pos_p90,
            time_error_median_s=time_median,
            time_error_p90_s=time_p90,
            evaluated_predicted_positions=evaluated_predicted_positions,
            evaluated_actual_positions=evaluated_actual_positions,
            pending_hit_positions=pending_hit_positions,
            paper_style_summary=self._current_paper_style_summary(),
        )


class TrajectoryPlotter:
    def __init__(self, config: VisualizerConfig):
        import matplotlib.pyplot as plt

        self.config = config
        self.plt = plt
        self.fig = plt.figure(figsize=(13.0, 8.0))
        self.ax3d = self.fig.add_subplot(2, 2, 1, projection="3d")
        self.ax_xy = self.fig.add_subplot(2, 2, 2)
        self.ax_xz = self.fig.add_subplot(2, 2, 3)
        self.ax_zt = self.fig.add_subplot(2, 2, 4)
        self.ax_error_time = self.ax_zt.twinx()
        self.ax_error_time.patch.set_visible(False)
        self.ax_error_time.set_visible(False)
        self.fig.suptitle("HITTER Ball Trajectory")

        (self.obs3d,) = self.ax3d.plot([], [], [], color="#1f77b4", linewidth=2.0, label="observed")
        (self.pred3d,) = self.ax3d.plot([], [], [], color="#d62728", linewidth=1.7, linestyle="--", label="predicted")
        self.bounce3d = self.ax3d.scatter([], [], [], color="#ff7f0e", s=35, label="bounce")
        self.hit3d = self.ax3d.scatter(
            [], [], [],
            color="#8a2be2",
            edgecolors="white",
            linewidths=1.2,
            marker="*",
            s=240,
            label="predicted hit",
            zorder=30,
        )
        self.hit_history3d = self.ax3d.scatter([], [], [], color="#bcbd22", marker=".", s=18, label="hit history")
        self.eval_pred3d = self.ax3d.scatter([], [], [], color="#d62728", marker="x", s=35, label="evaluated pred", zorder=8)
        self.eval_actual3d = self.ax3d.scatter([], [], [], color="#2ca02c", marker="o", s=35, label="actual crossing")
        self.latest_actual3d = self.ax3d.scatter([], [], [], color="#00aa44", marker="*", s=120, label="latest actual")

        (self.obs_xy,) = self.ax_xy.plot([], [], color="#1f77b4", linewidth=1.8)
        (self.pred_xy,) = self.ax_xy.plot([], [], color="#d62728", linewidth=1.5, linestyle="--")
        self.bounce_xy = self.ax_xy.scatter([], [], color="#ff7f0e", s=28)
        self.hit_xy = self.ax_xy.scatter(
            [], [],
            color="#8a2be2",
            edgecolors="white",
            linewidths=1.2,
            marker="*",
            s=220,
            zorder=30,
        )
        self.hit_history_xy = self.ax_xy.scatter([], [], color="#bcbd22", marker=".", s=16)
        self.eval_pred_xy = self.ax_xy.scatter([], [], color="#d62728", marker="x", s=30, zorder=8)
        self.eval_actual_xy = self.ax_xy.scatter([], [], color="#2ca02c", marker="o", s=30)
        self.latest_actual_xy = self.ax_xy.scatter([], [], color="#00aa44", marker="*", s=95)

        (self.obs_xz,) = self.ax_xz.plot([], [], color="#1f77b4", linewidth=1.8)
        (self.pred_xz,) = self.ax_xz.plot([], [], color="#d62728", linewidth=1.5, linestyle="--")
        self.bounce_xz = self.ax_xz.scatter([], [], color="#ff7f0e", s=28)
        self.hit_xz = self.ax_xz.scatter(
            [], [],
            color="#8a2be2",
            edgecolors="white",
            linewidths=1.2,
            marker="*",
            s=220,
            zorder=30,
        )
        self.hit_history_xz = self.ax_xz.scatter([], [], color="#bcbd22", marker=".", s=16)
        self.eval_pred_xz = self.ax_xz.scatter([], [], color="#d62728", marker="x", s=30, zorder=8)
        self.eval_actual_xz = self.ax_xz.scatter([], [], color="#2ca02c", marker="o", s=30)
        self.latest_actual_xz = self.ax_xz.scatter([], [], color="#00aa44", marker="*", s=95)

        (self.z_time,) = self.ax_zt.plot([], [], color="#1f77b4", linewidth=1.5)
        (self.paper_pos_error,) = self.ax_zt.plot(
            [],
            [],
            color="#1f77b4",
            linewidth=2.0,
            label="position error",
            visible=False,
        )
        (self.paper_time_error,) = self.ax_error_time.plot(
            [],
            [],
            color="#ff7f0e",
            linewidth=2.0,
            label="time error",
            visible=False,
        )
        self.paper_pos_ref = self.ax_zt.axhline(
            7.5,
            color="#d62728",
            linestyle="--",
            linewidth=1.2,
            visible=False,
        )
        self.paper_time_ref = self.ax_error_time.axhline(
            20.0,
            color="#d62728",
            linestyle=":",
            linewidth=1.2,
            visible=False,
        )
        self.height_ref = None
        self.paper_pos_band = None
        self.paper_time_band = None
        self.status_text = self.fig.text(0.015, 0.015, "", fontsize=9, family="monospace")
        self._draw_static_table()
        self.fig.tight_layout()

    def _table_outline(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        half_w = 0.5 * self.config.table_width
        length = self.config.table_length
        height = self.config.table_height
        xs = np.asarray([0.0, length, length, 0.0, 0.0], dtype=np.float64)
        ys = np.asarray([-half_w, -half_w, half_w, half_w, -half_w], dtype=np.float64)
        zs = np.full_like(xs, height)
        return xs, ys, zs

    def _draw_static_table(self) -> None:
        xs, ys, zs = self._table_outline()
        half_w = 0.5 * self.config.table_width
        z_min = self.config.table_height - 0.20
        z_max = self.config.table_height + 1.60

        self.ax3d.plot(xs, ys, zs, color="#333333", linewidth=1.2, alpha=0.8)
        self.ax3d.plot(
            [self.config.virtual_hit_plane_x, self.config.virtual_hit_plane_x],
            [-half_w, half_w],
            [self.config.table_height, self.config.table_height],
            color="#9467bd",
            linewidth=1.1,
            linestyle=":",
            alpha=0.9,
        )
        self.ax3d.set_xlim(-0.35, self.config.table_length + 0.45)
        self.ax3d.set_ylim(-half_w - 0.45, half_w + 0.45)
        self.ax3d.set_zlim(z_min, z_max)
        self.ax3d.set_xlabel("x m")
        self.ax3d.set_ylabel("y m")
        self.ax3d.set_zlabel("z m")
        self.ax3d.legend(loc="upper left")

        self.ax_xy.plot(xs, ys, color="#333333", linewidth=1.0)
        self.ax_xy.axvline(self.config.virtual_hit_plane_x, color="#9467bd", linewidth=1.0, linestyle=":")
        self.ax_xy.set_aspect("equal", adjustable="box")
        self.ax_xy.set_xlim(-0.35, self.config.table_length + 0.45)
        self.ax_xy.set_ylim(-half_w - 0.45, half_w + 0.45)
        self.ax_xy.set_xlabel("x m")
        self.ax_xy.set_ylabel("y m")
        self.ax_xy.set_title("top view")

        self.ax_xz.plot([0.0, self.config.table_length], [self.config.table_height, self.config.table_height], color="#333333", linewidth=1.0)
        self.ax_xz.axvline(self.config.virtual_hit_plane_x, color="#9467bd", linewidth=1.0, linestyle=":")
        self.ax_xz.set_xlim(-0.35, self.config.table_length + 0.45)
        self.ax_xz.set_ylim(z_min, z_max)
        self.ax_xz.set_xlabel("x m")
        self.ax_xz.set_ylabel("z m")
        self.ax_xz.set_title("side view")

        self.height_ref = self.ax_zt.axhline(self.config.table_height, color="#333333", linewidth=1.0)
        self.ax_zt.set_ylim(z_min, z_max)
        self.ax_zt.set_xlabel("time s")
        self.ax_zt.set_ylabel("z m")
        self.ax_zt.set_title("height over time")

    @staticmethod
    def _empty_offsets() -> np.ndarray:
        return np.empty((0, 2), dtype=np.float64)

    @staticmethod
    def _format_position(position: Optional[np.ndarray]) -> str:
        if position is None:
            return "none"
        return f"[{position[0]:.3f},{position[1]:.3f},{position[2]:.3f}]"

    @staticmethod
    def _format_age(seconds: float) -> str:
        if not math.isfinite(seconds):
            return "never"
        return f"{seconds:.2f}s"

    @staticmethod
    def _ball_state(status: LiveStatus) -> str:
        if status.ball_messages <= 0:
            return "ball missing"
        if math.isfinite(status.last_ball_age_s) and status.last_ball_age_s > 0.5:
            return "ball stale"
        return "ball ok"

    @staticmethod
    def _finite(value) -> bool:
        try:
            return math.isfinite(float(value))
        except (TypeError, ValueError):
            return False

    @classmethod
    def _format_paper_horizon(cls, row: dict) -> str:
        lead = float(row.get("lead_time_s", float("nan")))
        prefix = f"{lead:.1f}s"
        if int(row.get("sample_count", 0)) <= 0:
            return f"{prefix}=--"
        position = row.get("position_error_mean_cm")
        timing = row.get("time_error_mean_ms")
        if not (cls._finite(position) and cls._finite(timing)):
            return f"{prefix}=--"
        return f"{prefix}={float(position):.1f}cm/{float(timing):.0f}ms"

    @classmethod
    def _paper_status_text(cls, summary: Optional[dict]) -> str:
        if not summary or int(summary.get("event_count", 0)) <= 0:
            return "paper: waiting for hit-plane crossings"
        horizons = " ".join(cls._format_paper_horizon(row) for row in summary.get("horizons", ()))
        return (
            f"paper: events={int(summary.get('event_count', 0))} "
            f"preds={int(summary.get('prediction_count', 0))} {horizons}"
        ).strip()

    def _live_status_text(self, status: Optional[LiveStatus]) -> str:
        if status is None:
            return ""
        return (
            f"live: {self._ball_state(status)} "
            f"msgs={status.total_messages} total_hz={status.total_hz:.1f} "
            f"ball={status.ball_messages} ball_hz={status.ball_hz:.1f} "
            f"last_msg={self._format_age(status.last_message_age_s)} "
            f"last_ball={self._format_age(status.last_ball_age_s)}\n"
            f"base={self._format_position(status.latest_base_position)} "
            f"table={self._format_position(status.latest_table_position)} "
            f"ball_pos={self._format_position(status.latest_ball_position)}"
        )

    def _remove_paper_bands(self) -> None:
        for attr in ("paper_pos_band", "paper_time_band"):
            band = getattr(self, attr)
            if band is None:
                continue
            try:
                band.remove()
            except ValueError:
                pass
            setattr(self, attr, None)

    def _set_paper_error_mode(self, enabled: bool) -> None:
        self.z_time.set_visible(not enabled)
        if self.height_ref is not None:
            self.height_ref.set_visible(not enabled)
        self.paper_pos_error.set_visible(enabled)
        self.paper_time_error.set_visible(enabled)
        self.paper_pos_ref.set_visible(enabled)
        self.paper_time_ref.set_visible(enabled)
        self.ax_error_time.set_visible(enabled)
        if enabled:
            self.ax_zt.set_title("paper-style prediction error")
            self.ax_zt.set_xlabel("time before strike s")
            self.ax_zt.set_ylabel("position error cm", color="#1f77b4")
            self.ax_error_time.set_ylabel("strike time error ms", color="#ff7f0e")
            self.ax_zt.tick_params(axis="y", labelcolor="#1f77b4")
            self.ax_error_time.tick_params(axis="y", labelcolor="#ff7f0e")
            self.ax_zt.grid(True, alpha=0.25)
        else:
            self._remove_paper_bands()
            self.paper_pos_error.set_data([], [])
            self.paper_time_error.set_data([], [])
            self.ax_zt.set_title("height over time")
            self.ax_zt.set_xlabel("time s")
            self.ax_zt.set_ylabel("z m", color="black")
            self.ax_zt.tick_params(axis="y", labelcolor="black")
            self.ax_zt.set_ylim(self.config.table_height - 0.20, self.config.table_height + 1.60)

    def _update_paper_error_axis(self, summary: Optional[dict]) -> None:
        curve = [] if not summary else [
            row for row in summary.get("curve", ())
            if int(row.get("sample_count", 0)) > 0
            and self._finite(row.get("position_error_mean_cm"))
            and self._finite(row.get("time_error_mean_ms"))
        ]
        if not curve:
            self._set_paper_error_mode(False)
            return

        lead = np.asarray([row["lead_time_s"] for row in curve], dtype=np.float64)
        pos_mean = np.asarray([row["position_error_mean_cm"] for row in curve], dtype=np.float64)
        pos_std = np.asarray([row["position_error_std_cm"] for row in curve], dtype=np.float64)
        time_mean = np.asarray([row["time_error_mean_ms"] for row in curve], dtype=np.float64)
        time_std = np.asarray([row["time_error_std_ms"] for row in curve], dtype=np.float64)

        pos_std = np.nan_to_num(pos_std, nan=0.0, posinf=0.0, neginf=0.0)
        time_std = np.nan_to_num(time_std, nan=0.0, posinf=0.0, neginf=0.0)
        self._remove_paper_bands()
        self._set_paper_error_mode(True)

        self.paper_pos_error.set_data(lead, pos_mean)
        self.paper_time_error.set_data(lead, time_mean)
        self.paper_pos_band = self.ax_zt.fill_between(
            lead,
            np.maximum(0.0, pos_mean - pos_std),
            pos_mean + pos_std,
            color="#1f77b4",
            alpha=0.16,
            linewidth=0.0,
            zorder=1,
        )
        self.paper_time_band = self.ax_error_time.fill_between(
            lead,
            np.maximum(0.0, time_mean - time_std),
            time_mean + time_std,
            color="#ff7f0e",
            alpha=0.14,
            linewidth=0.0,
            zorder=1,
        )

        max_lead = max(1.0, float(np.nanmax(lead)))
        self.ax_zt.set_xlim(max_lead, 0.0)
        pos_upper = max(10.0, float(np.nanmax(pos_mean + pos_std)) * 1.2, 7.5 * 1.35)
        time_upper = max(30.0, float(np.nanmax(time_mean + time_std)) * 1.2, 20.0 * 1.5)
        self.ax_zt.set_ylim(0.0, pos_upper)
        self.ax_error_time.set_ylim(0.0, time_upper)

    def _clear_dynamic_artists(self) -> None:
        self.obs3d.set_data([], [])
        self.obs3d.set_3d_properties([])
        self.pred3d.set_data([], [])
        self.pred3d.set_3d_properties([])
        self.bounce3d._offsets3d = ([], [], [])
        self.hit3d._offsets3d = ([], [], [])
        self.hit_history3d._offsets3d = ([], [], [])
        self.eval_pred3d._offsets3d = ([], [], [])
        self.eval_actual3d._offsets3d = ([], [], [])
        self.latest_actual3d._offsets3d = ([], [], [])

        self.obs_xy.set_data([], [])
        self.pred_xy.set_data([], [])
        self.bounce_xy.set_offsets(self._empty_offsets())
        self.hit_xy.set_offsets(self._empty_offsets())
        self.hit_history_xy.set_offsets(self._empty_offsets())
        self.eval_pred_xy.set_offsets(self._empty_offsets())
        self.eval_actual_xy.set_offsets(self._empty_offsets())
        self.latest_actual_xy.set_offsets(self._empty_offsets())

        self.obs_xz.set_data([], [])
        self.pred_xz.set_data([], [])
        self.bounce_xz.set_offsets(self._empty_offsets())
        self.hit_xz.set_offsets(self._empty_offsets())
        self.hit_history_xz.set_offsets(self._empty_offsets())
        self.eval_pred_xz.set_offsets(self._empty_offsets())
        self.eval_actual_xz.set_offsets(self._empty_offsets())
        self.latest_actual_xz.set_offsets(self._empty_offsets())

        self.z_time.set_data([], [])
        self.ax_zt.set_xlim(-self.config.window_s, 0.05)
        self._set_paper_error_mode(False)

    def update_live(self, snapshot: Optional[AnalyzerSnapshot], live_status: LiveStatus) -> None:
        hold_expired = (
            math.isfinite(live_status.last_ball_age_s)
            and self.config.hold_last_trajectory_s >= 0.0
            and live_status.last_ball_age_s > self.config.hold_last_trajectory_s
        )
        if snapshot is None or live_status.ball_messages <= 0 or hold_expired:
            self._clear_dynamic_artists()
            self.ax3d.set_title(
                f"live: {self._ball_state(live_status)} "
                f"total={live_status.total_hz:.1f}Hz ball={live_status.ball_hz:.1f}Hz"
            )
            if snapshot is None or live_status.ball_messages <= 0:
                planner_line = "planner: waiting for ball samples"
            else:
                planner_line = "planner: last trajectory hold expired, waiting for a fresh trajectory"
            self.status_text.set_text(self._live_status_text(live_status) + "\n" + planner_line)
            self.fig.canvas.draw_idle()
            return
        self.update(snapshot, live_status=live_status)

    def update(self, snapshot: AnalyzerSnapshot, live_status: Optional[LiveStatus] = None) -> None:
        obs = snapshot.observed_positions
        pred = snapshot.predicted_positions
        bounces = snapshot.bounce_positions
        hit = snapshot.hit_prediction
        latest_eval = snapshot.latest_evaluation
        hit_history = snapshot.pending_hit_positions[-200:]
        evaluated_pred = snapshot.evaluated_predicted_positions[-200:]
        evaluated_actual = snapshot.evaluated_actual_positions[-200:]

        if obs.shape[0] > 0:
            self.obs3d.set_data(obs[:, 0], obs[:, 1])
            self.obs3d.set_3d_properties(obs[:, 2])
            self.obs_xy.set_data(obs[:, 0], obs[:, 1])
            self.obs_xz.set_data(obs[:, 0], obs[:, 2])
            self.z_time.set_data(snapshot.observed_times - snapshot.observed_times[-1], obs[:, 2])
            self.ax_zt.set_xlim(-self.config.window_s, 0.05)

        if pred.shape[0] > 0:
            self.pred3d.set_data(pred[:, 0], pred[:, 1])
            self.pred3d.set_3d_properties(pred[:, 2])
            self.pred_xy.set_data(pred[:, 0], pred[:, 1])
            self.pred_xz.set_data(pred[:, 0], pred[:, 2])
        else:
            self.pred3d.set_data([], [])
            self.pred3d.set_3d_properties([])
            self.pred_xy.set_data([], [])
            self.pred_xz.set_data([], [])

        if bounces.shape[0] > 0:
            self.bounce3d._offsets3d = (bounces[:, 0], bounces[:, 1], bounces[:, 2])
            self.bounce_xy.set_offsets(bounces[:, :2])
            self.bounce_xz.set_offsets(bounces[:, [0, 2]])
        else:
            self.bounce3d._offsets3d = ([], [], [])
            self.bounce_xy.set_offsets(np.empty((0, 2)))
            self.bounce_xz.set_offsets(np.empty((0, 2)))

        display_hit_position = None
        if hit is not None:
            display_hit_position = hit.hit_position
        elif latest_eval is not None:
            display_hit_position = latest_eval.predicted_position

        if display_hit_position is not None:
            p = display_hit_position
            self.hit3d._offsets3d = ([p[0]], [p[1]], [p[2]])
            self.hit_xy.set_offsets(np.asarray([[p[0], p[1]]], dtype=np.float64))
            self.hit_xz.set_offsets(np.asarray([[p[0], p[2]]], dtype=np.float64))
        else:
            self.hit3d._offsets3d = ([], [], [])
            self.hit_xy.set_offsets(self._empty_offsets())
            self.hit_xz.set_offsets(self._empty_offsets())

        if hit_history.shape[0] > 0:
            self.hit_history3d._offsets3d = (hit_history[:, 0], hit_history[:, 1], hit_history[:, 2])
            self.hit_history_xy.set_offsets(hit_history[:, :2])
            self.hit_history_xz.set_offsets(hit_history[:, [0, 2]])
        else:
            self.hit_history3d._offsets3d = ([], [], [])
            self.hit_history_xy.set_offsets(self._empty_offsets())
            self.hit_history_xz.set_offsets(self._empty_offsets())

        if evaluated_pred.shape[0] > 0:
            self.eval_pred3d._offsets3d = (evaluated_pred[:, 0], evaluated_pred[:, 1], evaluated_pred[:, 2])
            self.eval_pred_xy.set_offsets(evaluated_pred[:, :2])
            self.eval_pred_xz.set_offsets(evaluated_pred[:, [0, 2]])
        else:
            self.eval_pred3d._offsets3d = ([], [], [])
            self.eval_pred_xy.set_offsets(np.empty((0, 2)))
            self.eval_pred_xz.set_offsets(np.empty((0, 2)))

        if evaluated_actual.shape[0] > 0:
            self.eval_actual3d._offsets3d = (evaluated_actual[:, 0], evaluated_actual[:, 1], evaluated_actual[:, 2])
            self.eval_actual_xy.set_offsets(evaluated_actual[:, :2])
            self.eval_actual_xz.set_offsets(evaluated_actual[:, [0, 2]])
        else:
            self.eval_actual3d._offsets3d = ([], [], [])
            self.eval_actual_xy.set_offsets(np.empty((0, 2)))
            self.eval_actual_xz.set_offsets(np.empty((0, 2)))

        if latest_eval is not None:
            p = latest_eval.actual_position
            self.latest_actual3d._offsets3d = ([p[0]], [p[1]], [p[2]])
            self.latest_actual_xy.set_offsets(np.asarray([[p[0], p[1]]], dtype=np.float64))
            self.latest_actual_xz.set_offsets(np.asarray([[p[0], p[2]]], dtype=np.float64))
        else:
            self.latest_actual3d._offsets3d = ([], [], [])
            self.latest_actual_xy.set_offsets(self._empty_offsets())
            self.latest_actual_xz.set_offsets(self._empty_offsets())

        self._update_paper_error_axis(snapshot.paper_style_summary)

        status = "ready" if snapshot.estimate_valid else "warming"
        self.ax3d.set_title(
            f"{status}: samples={snapshot.sample_count} speed={snapshot.speed_mps:.2f} m/s "
            f"v=[{snapshot.estimate_velocity[0]:.2f}, {snapshot.estimate_velocity[1]:.2f}, {snapshot.estimate_velocity[2]:.2f}]"
        )
        if hit is not None:
            hit_line = (
                f"hit: t={hit.time_to_hit_s:.3f}s "
                f"p=[{hit.hit_position[0]:.3f},{hit.hit_position[1]:.3f},{hit.hit_position[2]:.3f}] "
                f"vr=[{hit.racket_velocity[0]:.2f},{hit.racket_velocity[1]:.2f},{hit.racket_velocity[2]:.2f}]"
            )
        elif latest_eval is not None:
            hit_line = (
                "hit: latest crossed "
                f"pred=[{latest_eval.predicted_position[0]:.3f},{latest_eval.predicted_position[1]:.3f},{latest_eval.predicted_position[2]:.3f}] "
                f"actual=[{latest_eval.actual_position[0]:.3f},{latest_eval.actual_position[1]:.3f},{latest_eval.actual_position[2]:.3f}]"
            )
        else:
            hit_line = "hit: waiting for incoming ball crossing the hit plane"
        if snapshot.latest_evaluation is None:
            eval_line = "eval: no crossing evaluated yet"
        else:
            latest = snapshot.latest_evaluation
            eval_line = (
                f"eval: latest={latest.position_error_m * 100.0:.1f}cm/{latest.time_error_s * 1000.0:.0f}ms "
                f"median={snapshot.position_error_median_m * 100.0:.1f}cm/{snapshot.time_error_median_s * 1000.0:.0f}ms "
                f"p90={snapshot.position_error_p90_m * 100.0:.1f}cm/{snapshot.time_error_p90_s * 1000.0:.0f}ms "
                f"n={snapshot.evaluation_count}"
            )
        paper_summary = snapshot.paper_style_summary
        paper_line = self._paper_status_text(paper_summary)
        live_text = self._live_status_text(live_status)
        if paper_summary is not None and int(paper_summary.get("event_count", 0)) > 0:
            lines = [line for line in (live_text, hit_line, paper_line) if line]
        else:
            lines = [line for line in (live_text, hit_line, paper_line, eval_line) if line]
        self.status_text.set_text("\n".join(lines))
        self.fig.canvas.draw_idle()

    def save(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.fig.savefig(path, dpi=180)

    def show(self) -> None:
        self.plt.show()

    def pause(self, seconds: float = 0.001) -> None:
        self.plt.pause(seconds)


def _build_config(args: argparse.Namespace) -> VisualizerConfig:
    return VisualizerConfig(
        window_s=args.window_s,
        hold_last_trajectory_s=args.hold_last_trajectory_s,
        estimator_window=args.estimator_window,
        estimator_min_samples=args.estimator_min_samples,
        table_height=args.table_height,
        table_center_xy=(args.table_center_x, args.table_center_y),
        table_length=args.table_length,
        table_width=args.table_width,
        ball_radius=args.ball_radius,
        bounce_height_tolerance=args.bounce_height_tolerance,
        bounce_velocity_threshold=args.bounce_velocity_threshold,
        bounce_min_separation_s=args.bounce_min_separation_s,
        max_sample_gap_s=args.max_sample_gap_s,
        prediction_horizon_s=args.prediction_horizon_s,
        prediction_dt=args.prediction_dt,
        virtual_hit_plane_x=args.virtual_hit_plane_x,
        desired_landing_point=(args.desired_landing_x, args.desired_landing_y, args.desired_landing_z),
        post_hit_flight_time=args.post_hit_flight_time,
        racket_restitution=args.racket_restitution,
        drag_coefficient=args.drag_coefficient,
        horizontal_restitution=args.horizontal_restitution,
        vertical_restitution=args.vertical_restitution,
        gravity=(0.0, 0.0, args.gravity_z),
    )


def run_offline(args: argparse.Namespace) -> int:
    if args.csv is None:
        print("error: offline mode requires --csv")
        return 2
    samples = load_ball_csv_samples(
        args.csv,
        sample_rate_hz=args.sample_rate_hz,
        ball_name=args.ball_name,
        segment_id=args.segment_id,
        time_source=args.time_source,
    )
    if not samples:
        print(f"error: no valid ball samples found in {args.csv}")
        return 1

    config = _build_config(args)
    analyzer = BallTrajectoryAnalyzer(config)
    snapshot = None
    for sample in samples:
        snapshot = analyzer.add_sample(sample.position, timestamp=sample.timestamp)
    assert snapshot is not None

    output = args.output
    if output is None and not args.show:
        suffix = "" if args.segment_id is None else f"_segment_{args.segment_id:03d}"
        output = args.csv.with_name(f"{args.csv.stem}{suffix}_trajectory.png")

    if output is not None or args.show:
        if output is not None and not args.show:
            import matplotlib

            matplotlib.use("Agg")
        plotter = TrajectoryPlotter(config)
        plotter.update(snapshot)
        if output is not None:
            plotter.save(output)
            print(f"saved={output}")
        if args.show:
            plotter.show()

    duration = samples[-1].timestamp - samples[0].timestamp
    print(
        f"samples={len(samples)} duration_s={duration:.3f} "
        f"bounces={snapshot.bounce_positions.shape[0]} "
        f"speed_mps={snapshot.speed_mps:.3f} estimate_valid={int(snapshot.estimate_valid)}"
    )
    return 0


class LiveRecorder:
    def __init__(self, path: Optional[Path]):
        self.path = path
        self.handle = None
        self.writer = None
        self.count = 0
        self.fields = [
            "host_time_s",
            "elapsed_s",
            "row_index",
            "frame_number",
            "vicon_frame_number",
            "vicon_time_s",
            "publish_time_us",
            "channel",
            "name",
            "ball_valid",
            "ball_occluded",
            "ball_x_m",
            "ball_y_m",
            "ball_z_m",
            "ball_qx",
            "ball_qy",
            "ball_qz",
            "ball_qw",
            "base_valid",
            "base_x_m",
            "base_y_m",
            "base_z_m",
            "base_qx",
            "base_qy",
            "base_qz",
            "base_qw",
        ]

    def __enter__(self):
        if self.path is not None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.handle = self.path.open("w", newline="")
            self.writer = csv.DictWriter(self.handle, fieldnames=self.fields)
            self.writer.writeheader()
            self.handle.flush()
        return self

    def __exit__(self, exc_type, exc, tb):
        if self.handle is not None:
            self.handle.close()

    def write(self, *, channel: str, elapsed: float, host_time: float, ball_msg, ball_pos: np.ndarray, ball_quat: np.ndarray, base_state) -> None:
        if self.writer is None:
            return
        base_pos = None
        base_quat = None
        if base_state is not None:
            base_pos, base_quat = base_state
        vicon_frame_number = _msg_int_field(ball_msg, "vicon_frame_number", 0)
        vicon_time_s = _msg_vicon_time(ball_msg)
        publish_time_us = _msg_int_field(ball_msg, "publish_time_us", 0)
        ball_valid = _msg_int_field(ball_msg, "valid", 1)
        ball_occluded = _msg_int_field(ball_msg, "occluded", 0)
        frame_number = vicon_frame_number if vicon_frame_number > 0 else self.count
        row = {
            "host_time_s": f"{host_time:.9f}",
            "elapsed_s": f"{elapsed:.9f}",
            "row_index": self.count,
            "frame_number": frame_number,
            "vicon_frame_number": "" if vicon_frame_number <= 0 else vicon_frame_number,
            "vicon_time_s": "" if vicon_time_s is None else f"{vicon_time_s:.9f}",
            "publish_time_us": "" if publish_time_us <= 0 else publish_time_us,
            "channel": channel,
            "name": "ball",
            "ball_valid": ball_valid,
            "ball_occluded": ball_occluded,
            "ball_x_m": f"{ball_pos[0]:.9f}",
            "ball_y_m": f"{ball_pos[1]:.9f}",
            "ball_z_m": f"{ball_pos[2]:.9f}",
            "ball_qx": f"{ball_quat[0]:.9f}",
            "ball_qy": f"{ball_quat[1]:.9f}",
            "ball_qz": f"{ball_quat[2]:.9f}",
            "ball_qw": f"{ball_quat[3]:.9f}",
            "base_valid": int(base_pos is not None),
            "base_x_m": "" if base_pos is None else f"{base_pos[0]:.9f}",
            "base_y_m": "" if base_pos is None else f"{base_pos[1]:.9f}",
            "base_z_m": "" if base_pos is None else f"{base_pos[2]:.9f}",
            "base_qx": "" if base_quat is None else f"{base_quat[0]:.9f}",
            "base_qy": "" if base_quat is None else f"{base_quat[1]:.9f}",
            "base_qz": "" if base_quat is None else f"{base_quat[2]:.9f}",
            "base_qw": "" if base_quat is None else f"{base_quat[3]:.9f}",
        }
        self.writer.writerow(row)
        self.count += 1
        if self.handle is not None and self.count % 300 == 0:
            self.handle.flush()


class PlannerEvaluationRecorder:
    def __init__(self, path: Optional[Path]):
        self.path = path
        self.handle = None
        self.writer = None
        self.count = 0
        self.fields = [
            "created_time_s",
            "time_to_hit_s",
            "predicted_hit_time_s",
            "actual_hit_time_s",
            "predicted_x_m",
            "predicted_y_m",
            "predicted_z_m",
            "actual_x_m",
            "actual_y_m",
            "actual_z_m",
            "position_error_cm",
            "time_error_ms",
        ]

    def __enter__(self):
        if self.path is not None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.handle = self.path.open("w", newline="")
            self.writer = csv.DictWriter(self.handle, fieldnames=self.fields)
            self.writer.writeheader()
            self.handle.flush()
        return self

    def __exit__(self, exc_type, exc, tb):
        if self.handle is not None:
            self.handle.close()

    def write_many(self, evaluations: Sequence[PlannerEvaluation]) -> None:
        if self.writer is None:
            return
        for evaluation in evaluations:
            self.writer.writerow(
                {
                    "created_time_s": f"{evaluation.created_time:.9f}",
                    "time_to_hit_s": f"{evaluation.time_to_hit_s:.9f}",
                    "predicted_hit_time_s": f"{evaluation.predicted_hit_time_s:.9f}",
                    "actual_hit_time_s": f"{evaluation.actual_hit_time_s:.9f}",
                    "predicted_x_m": f"{evaluation.predicted_position[0]:.9f}",
                    "predicted_y_m": f"{evaluation.predicted_position[1]:.9f}",
                    "predicted_z_m": f"{evaluation.predicted_position[2]:.9f}",
                    "actual_x_m": f"{evaluation.actual_position[0]:.9f}",
                    "actual_y_m": f"{evaluation.actual_position[1]:.9f}",
                    "actual_z_m": f"{evaluation.actual_position[2]:.9f}",
                    "position_error_cm": f"{evaluation.position_error_m * 100.0:.6f}",
                    "time_error_ms": f"{evaluation.time_error_s * 1000.0:.6f}",
                }
            )
            self.count += 1
        if self.handle is not None and evaluations:
            self.handle.flush()


def run_live(args: argparse.Namespace) -> int:
    try:
        import lcm
        from unitree_sdk2.lcm_types.transformation_t import transformation_t
    except ImportError as exc:
        print(f"error: live mode requires lcm and unitree_sdk2 lcm types: {exc}")
        print("hint: use `conda run -n rb python deploy/mocap_bridge/visualize_ball_trajectory.py --live ...`")
        return 2

    config = _build_config(args)
    analyzer = BallTrajectoryAnalyzer(config)
    plotter = TrajectoryPlotter(config)
    lc = lcm.LCM(args.lcm_url)
    start = time.time()
    latest_base = None
    latest_table = None
    latest_ball_position = None
    latest_snapshot = None
    total_messages = 0
    base_messages = 0
    table_messages = 0
    ball_messages = 0
    last_message_time = None
    last_ball_time = None
    ball_stream_time = None
    last_draw = 0.0
    last_status_print = 0.0

    def build_live_status(now: float) -> LiveStatus:
        elapsed = max(now - start, 1.0e-9)
        return LiveStatus(
            elapsed_s=elapsed,
            total_messages=total_messages,
            base_messages=base_messages,
            table_messages=table_messages,
            ball_messages=ball_messages,
            total_hz=total_messages / elapsed,
            ball_hz=ball_messages / elapsed,
            last_message_age_s=float("nan") if last_message_time is None else now - last_message_time,
            last_ball_age_s=float("nan") if last_ball_time is None else now - last_ball_time,
            latest_base_position=None if latest_base is None else latest_base[0].copy(),
            latest_table_position=None if latest_table is None else latest_table.copy(),
            latest_ball_position=None if latest_ball_position is None else latest_ball_position.copy(),
        )

    def handler(channel, data):
        nonlocal latest_base, latest_table, latest_ball_position, latest_snapshot
        nonlocal total_messages, base_messages, table_messages, ball_messages
        nonlocal last_message_time, last_ball_time, ball_stream_time
        msg = transformation_t.decode(data)
        name = str(msg.name)
        pos = np.asarray(msg.pos_vicon, dtype=np.float64)
        quat = np.asarray(msg.quat_vicon, dtype=np.float64)
        now = time.time()
        elapsed = now - start
        total_messages += 1
        last_message_time = now
        if name == args.base_name:
            base_messages += 1
            latest_base = (pos.copy(), quat.copy())
            return
        if name == args.table_name:
            table_messages += 1
            latest_table = pos.copy()
            return
        if name != args.ball_name:
            return
        ball_messages += 1
        source_time = _msg_vicon_time(msg)
        if source_time is None:
            source_time = next_stream_timestamp(
                ball_stream_time,
                last_ball_time,
                now,
                sample_rate_hz=args.sample_rate_hz,
                max_sample_gap_s=args.max_sample_gap_s,
            )
        ball_stream_time = source_time
        last_ball_time = now
        latest_ball_position = pos.copy()
        latest_snapshot = analyzer.add_sample(pos, timestamp=source_time)
        if latest_snapshot.new_evaluations:
            eval_recorder.write_many(latest_snapshot.new_evaluations)
            latest_eval = latest_snapshot.latest_evaluation
            if latest_eval is not None:
                print(
                    "planner_eval "
                    f"new={len(latest_snapshot.new_evaluations)} total={latest_snapshot.evaluation_count} "
                    f"latest={latest_eval.position_error_m * 100.0:.2f}cm/"
                    f"{latest_eval.time_error_s * 1000.0:.1f}ms "
                    f"median={latest_snapshot.position_error_median_m * 100.0:.2f}cm/"
                    f"{latest_snapshot.time_error_median_s * 1000.0:.1f}ms "
                    f"p90={latest_snapshot.position_error_p90_m * 100.0:.2f}cm/"
                    f"{latest_snapshot.time_error_p90_s * 1000.0:.1f}ms",
                    flush=True,
                )
        recorder.write(
            channel=channel,
            elapsed=elapsed,
            host_time=now,
            ball_msg=msg,
            ball_pos=pos,
            ball_quat=quat,
            base_state=latest_base,
        )

    print(f"listening lcm_url={args.lcm_url} channel={args.channel} ball_name={args.ball_name}")
    with LiveRecorder(args.record_csv) as recorder, PlannerEvaluationRecorder(args.eval_csv) as eval_recorder:
        lc.subscribe(args.channel, handler)
        try:
            while True:
                now = time.time()
                if args.duration > 0.0 and now - start >= args.duration:
                    break
                drain_lcm(lc, timeout_s=0.01, max_messages=args.max_lcm_batch)
                status = build_live_status(now)
                if now - last_draw >= 1.0 / max(args.refresh_hz, 1.0):
                    plotter.update_live(latest_snapshot, status)
                    plotter.pause(0.001)
                    last_draw = now
                if args.status_print_hz > 0.0 and now - last_status_print >= 1.0 / args.status_print_hz:
                    print(
                        "viewer_status "
                        f"{TrajectoryPlotter._ball_state(status)} "
                        f"msgs={status.total_messages} total_hz={status.total_hz:.1f} "
                        f"base={status.base_messages} table={status.table_messages} "
                        f"ball={status.ball_messages} ball_hz={status.ball_hz:.1f} "
                        f"last_ball={TrajectoryPlotter._format_age(status.last_ball_age_s)}",
                        flush=True,
                    )
                    last_status_print = now
        except KeyboardInterrupt:
            pass

    print(
        f"received_messages={total_messages} received_ball={ball_messages} recorded_ball_rows={recorder.count} "
        f"planner_eval_rows={eval_recorder.count}"
    )
    return 0 if ball_messages > 0 or args.duration <= 0.0 else 1


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Realtime/offline 3D visualization for HITTER Vicon ball trajectories.")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--live", action="store_true", help="Subscribe to live LCM ball data and update plots in real time.")
    mode.add_argument("--csv", type=Path, help="Offline CSV recorded by monitor_vicon_lcm.py or this script.")

    parser.add_argument("--output", type=Path, default=None, help="Offline output image path. Defaults to <csv>_trajectory.png.")
    parser.add_argument("--show", action="store_true", help="Show an interactive matplotlib window.")
    parser.add_argument("--segment-id", type=int, default=None, help="Offline: only plot a clean_segment_id from a cleaned CSV.")
    parser.add_argument("--sample-rate-hz", type=float, default=300.0)
    parser.add_argument(
        "--time-source",
        choices=("frame", "elapsed", "host", "auto"),
        default="frame",
        help="Offline CSV time base. Use frame for Vicon/LCM recordings at --sample-rate-hz.",
    )

    parser.add_argument("--lcm-url", default="udpm://239.255.76.67:7667?ttl=255")
    parser.add_argument("--channel", default="vicon_state_data")
    parser.add_argument("--ball-name", default="ball")
    parser.add_argument("--base-name", default="G1Pelvis")
    parser.add_argument("--table-name", default="table")
    parser.add_argument("--record-csv", type=Path, default=None, help="Live mode: record ball samples while plotting.")
    parser.add_argument(
        "--eval-csv",
        type=Path,
        default=Path("deploy/mocap_bridge/logs/ball_dynamics/live_planner_eval.csv"),
        help="Live mode: write planner prediction error rows when the ball crosses the hit plane.",
    )
    parser.add_argument("--duration", type=float, default=0.0, help="Live mode duration in seconds; 0 means until Ctrl-C.")
    parser.add_argument("--refresh-hz", type=float, default=30.0)
    parser.add_argument("--status-print-hz", type=float, default=1.0, help="Live mode console diagnostic print rate; 0 disables it.")
    parser.add_argument("--max-lcm-batch", type=int, default=2000, help="Maximum queued LCM messages handled before one GUI refresh.")

    parser.add_argument("--window-s", type=float, default=3.0)
    parser.add_argument(
        "--hold-last-trajectory-s",
        type=float,
        default=60.0,
        help="Live mode: keep the last plotted ball trajectory this many seconds after ball samples stop; negative keeps it forever.",
    )
    parser.add_argument("--estimator-window", type=int, default=31)
    parser.add_argument("--estimator-min-samples", type=int, default=31)
    parser.add_argument("--max-sample-gap-s", type=float, default=0.25)
    parser.add_argument("--prediction-horizon-s", type=float, default=1.2)
    parser.add_argument("--prediction-dt", type=float, default=0.005)
    parser.add_argument("--virtual-hit-plane-x", type=float, default=0.0)
    parser.add_argument("--desired-landing-x", type=float, default=2.05)
    parser.add_argument("--desired-landing-y", type=float, default=0.0)
    parser.add_argument("--desired-landing-z", type=float, default=0.78)
    parser.add_argument("--post-hit-flight-time", type=float, default=0.48)
    parser.add_argument("--racket-restitution", type=float, default=0.85)

    parser.add_argument("--table-height", type=float, default=0.76)
    parser.add_argument("--table-center-x", type=float, default=1.37)
    parser.add_argument("--table-center-y", type=float, default=0.0)
    parser.add_argument("--table-length", type=float, default=2.74)
    parser.add_argument("--table-width", type=float, default=1.525)
    parser.add_argument("--ball-radius", type=float, default=0.02)
    parser.add_argument("--bounce-height-tolerance", type=float, default=0.03)
    parser.add_argument("--bounce-velocity-threshold", type=float, default=0.10)
    parser.add_argument("--bounce-min-separation-s", type=float, default=0.20)

    parser.add_argument("--gravity-z", type=float, default=-9.81)
    parser.add_argument("--drag-coefficient", type=float, default=0.0)
    parser.add_argument("--horizontal-restitution", type=float, default=1.0)
    parser.add_argument("--vertical-restitution", type=float, default=0.8)
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    if args.live:
        return run_live(args)
    return run_offline(args)


if __name__ == "__main__":
    raise SystemExit(main())
