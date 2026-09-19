from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass

import numpy as np


def _vec2(value, name: str) -> np.ndarray:
    arr = np.asarray(value, dtype=np.float64).reshape(-1)
    if arr.shape != (2,):
        raise ValueError(f"{name} must have shape (2,), got {arr.shape}.")
    if not np.isfinite(arr).all():
        raise ValueError(f"{name} contains non-finite values: {arr}.")
    return arr


def _vec3(value, name: str) -> np.ndarray:
    arr = np.asarray(value, dtype=np.float64).reshape(-1)
    if arr.shape != (3,):
        raise ValueError(f"{name} must have shape (3,), got {arr.shape}.")
    if not np.isfinite(arr).all():
        raise ValueError(f"{name} contains non-finite values: {arr}.")
    return arr


def _base_position(value, default_z: float) -> np.ndarray:
    arr = np.asarray(value, dtype=np.float64).reshape(-1)
    if arr.shape == (2,):
        arr = np.array([arr[0], arr[1], default_z], dtype=np.float64)
    if arr.shape != (3,):
        raise ValueError(f"current_base_xy_w must contain 2 or 3 values, got shape {arr.shape}.")
    if not np.isfinite(arr).all():
        raise ValueError(f"current_base_xy_w contains non-finite values: {arr}.")
    return arr


def _world_from_base_yaw(base_forward_xy_w) -> np.ndarray:
    forward = _vec2(base_forward_xy_w, "base_forward_xy_w")
    norm = np.linalg.norm(forward)
    if norm < 1.0e-9:
        raise ValueError("base_forward_xy_w must be nonzero.")
    forward = forward / norm
    left = np.array([-forward[1], forward[0]], dtype=np.float64)
    return np.column_stack([forward, left])


@dataclass(frozen=True)
class BallTrajectory:
    times: np.ndarray
    positions: np.ndarray
    velocities: np.ndarray


@dataclass(frozen=True)
class BallStateEstimate:
    position: np.ndarray
    velocity: np.ndarray
    sample_count: int
    valid: bool
    bounce_detected: bool


class BallStateEstimator:
    """Online ball state estimator matching the paper's quadratic LS fit."""

    def __init__(
        self,
        *,
        window_size: int = 31,
        min_samples: int | None = None,
        table_height: float = 0.76,
        table_center_xy=(1.37, 0.0),
        table_length: float = 2.74,
        table_width: float = 1.525,
        ball_radius: float = 0.02,
        bounce_height_tolerance: float = 0.03,
        bounce_velocity_threshold: float = 0.10,
        bounce_min_separation_s: float = 0.20,
        max_sample_gap_s: float | None = 0.25,
    ):
        if window_size < 1:
            raise ValueError("window_size must be positive.")
        self.window_size = int(window_size)
        self.min_samples = self.window_size if min_samples is None else int(min_samples)
        if self.min_samples < 1 or self.min_samples > self.window_size:
            raise ValueError("min_samples must be in [1, window_size].")
        self.table_height = float(table_height)
        self.table_center_xy = _vec2(table_center_xy, "table_center_xy")
        self.table_length = float(table_length)
        self.table_width = float(table_width)
        self.ball_radius = float(ball_radius)
        self.bounce_height_tolerance = float(bounce_height_tolerance)
        self.bounce_velocity_threshold = float(bounce_velocity_threshold)
        self.bounce_min_separation_s = float(bounce_min_separation_s)
        self.max_sample_gap_s = None if max_sample_gap_s is None else float(max_sample_gap_s)
        self._samples = deque(maxlen=self.window_size)
        self._last_bounce_time: float | None = None

    @property
    def sample_count(self) -> int:
        return len(self._samples)

    def reset(self, *, clear_bounce_history: bool = True) -> None:
        self._samples.clear()
        if clear_bounce_history:
            self._last_bounce_time = None

    def _inside_table(self, xy: np.ndarray) -> bool:
        half = np.array([0.5 * self.table_length, 0.5 * self.table_width], dtype=np.float64)
        return bool(np.all(np.abs(xy - self.table_center_xy) <= half))

    def _detect_table_bounce(self, position: np.ndarray, timestamp: float) -> bool:
        if len(self._samples) < 2:
            return False

        prev_t, prev_pos = self._samples[-2]
        last_t, last_pos = self._samples[-1]
        dt_pre = max(float(last_t - prev_t), 1.0e-6)
        dt_post = max(float(timestamp - last_t), 1.0e-6)
        pre_vz = float((last_pos[2] - prev_pos[2]) / dt_pre)
        post_vz = float((position[2] - last_pos[2]) / dt_post)

        contact_z = self.table_height + self.ball_radius
        near_contact = min(float(last_pos[2]), float(position[2])) <= contact_z + self.bounce_height_tolerance
        inside_table = self._inside_table(last_pos[:2]) or self._inside_table(position[:2])
        if not (near_contact and inside_table):
            return False

        threshold = self.bounce_velocity_threshold
        rebound_turn = pre_vz < -threshold and post_vz > threshold
        contact_crossing = (
            float(last_pos[2]) > contact_z + self.bounce_height_tolerance
            and float(position[2]) <= contact_z + self.bounce_height_tolerance
            and post_vz < -threshold
        )
        return bool(rebound_turn or contact_crossing)

    def _fit_current_state(self, current_time: float, bounce_detected: bool) -> BallStateEstimate:
        count = len(self._samples)
        times = np.asarray([sample[0] for sample in self._samples], dtype=np.float64)
        positions = np.asarray([sample[1] for sample in self._samples], dtype=np.float64)

        if count == 1:
            position = positions[-1].copy()
            velocity = np.zeros(3, dtype=np.float64)
        elif count == 2:
            dt = max(float(times[-1] - times[-2]), 1.0e-6)
            position = positions[-1].copy()
            velocity = (positions[-1] - positions[-2]) / dt
        else:
            tau = times - float(current_time)
            design = np.column_stack([np.ones(count, dtype=np.float64), tau, tau * tau])
            coeffs, *_ = np.linalg.lstsq(design, positions, rcond=None)
            position = coeffs[0].copy()
            velocity = coeffs[1].copy()

        return BallStateEstimate(
            position=position,
            velocity=velocity,
            sample_count=count,
            valid=count >= self.min_samples,
            bounce_detected=bounce_detected,
        )

    def add_sample(self, position, *, timestamp: float | None = None) -> BallStateEstimate:
        position = _vec3(position, "position")
        now = time.time() if timestamp is None else float(timestamp)
        if not np.isfinite(now):
            raise ValueError(f"timestamp must be finite, got {timestamp!r}.")

        if self._samples:
            last_time = float(self._samples[-1][0])
            if now <= last_time:
                now = last_time + 1.0e-6
            if self.max_sample_gap_s is not None and now - last_time > self.max_sample_gap_s:
                self.reset()

        bounce_detected = self._detect_table_bounce(position, now)
        if bounce_detected:
            if (
                self._last_bounce_time is not None
                and now - self._last_bounce_time < self.bounce_min_separation_s
            ):
                bounce_detected = False
            else:
                self._last_bounce_time = now
                self.reset(clear_bounce_history=False)

        self._samples.append((now, position.copy()))
        return self._fit_current_state(now, bounce_detected)


class BallTrajectoryPredictor:
    def __init__(
        self,
        *,
        gravity=(0.0, 0.0, -9.81),
        drag_coefficient: float = 0.0,
        vertical_restitution: float = 0.8,
        horizontal_restitution: float = 0.9,
        dt: float = 0.005,
        table_height: float = 0.76,
        table_center_xy=(1.37, 0.0),
        table_length: float = 2.74,
        table_width: float = 1.525,
        ball_radius: float = 0.02,
    ):
        if dt <= 0.0:
            raise ValueError("dt must be positive.")
        self.gravity = _vec3(gravity, "gravity")
        self.drag_coefficient = float(drag_coefficient)
        self.vertical_restitution = float(vertical_restitution)
        self.horizontal_restitution = float(horizontal_restitution)
        self.dt = float(dt)
        self.table_height = float(table_height)
        self.table_center_xy = _vec2(table_center_xy, "table_center_xy")
        self.table_length = float(table_length)
        self.table_width = float(table_width)
        self.ball_radius = float(ball_radius)

    def _inside_table(self, xy: np.ndarray) -> bool:
        half = np.array([0.5 * self.table_length, 0.5 * self.table_width], dtype=np.float64)
        return bool(np.all(np.abs(xy - self.table_center_xy) <= half))

    def predict(self, position, velocity, horizon_s: float) -> BallTrajectory:
        if horizon_s < 0.0:
            raise ValueError("horizon_s must be non-negative.")
        count = int(np.ceil(float(horizon_s) / self.dt)) + 1
        times = np.arange(count, dtype=np.float64) * self.dt
        positions = np.empty((count, 3), dtype=np.float64)
        velocities = np.empty((count, 3), dtype=np.float64)

        pos = _vec3(position, "position").copy()
        vel = _vec3(velocity, "velocity").copy()
        contact_z = self.table_height + self.ball_radius
        for index in range(count):
            positions[index] = pos
            velocities[index] = vel
            acc = self.gravity - self.drag_coefficient * np.linalg.norm(vel) * vel
            next_vel = vel + acc * self.dt
            next_pos = pos + vel * self.dt + 0.5 * acc * self.dt * self.dt
            if vel[2] < 0.0 and next_pos[2] <= contact_z and self._inside_table(next_pos[:2]):
                next_pos[2] = contact_z
                next_vel[:2] *= self.horizontal_restitution
                next_vel[2] = -next_vel[2] * self.vertical_restitution
            pos, vel = next_pos, next_vel
        return BallTrajectory(times=times, positions=positions, velocities=velocities)


@dataclass(frozen=True)
class StrikePlan:
    t_strike: float
    p_racket_target: np.ndarray
    v_racket_target: np.ndarray
    v_ball_in: np.ndarray
    v_ball_out: np.ndarray


class StrikePlanner:
    def __init__(
        self,
        *,
        predictor: BallTrajectoryPredictor | None = None,
        virtual_hit_plane_x: float = 0.40,
        desired_landing_point=(2.05, 0.0, 0.78),
        post_hit_flight_time: float = 0.55,
        racket_restitution: float = 0.85,
        prediction_horizon_s: float = 2.0,
        require_future_hit_plane_crossing: bool = False,
    ):
        self.predictor = predictor if predictor is not None else BallTrajectoryPredictor()
        self.virtual_hit_plane_x = float(virtual_hit_plane_x)
        self.desired_landing_point = _vec3(desired_landing_point, "desired_landing_point")
        self.post_hit_flight_time = float(post_hit_flight_time)
        self.racket_restitution = float(racket_restitution)
        self.prediction_horizon_s = float(prediction_horizon_s)
        self.require_future_hit_plane_crossing = bool(require_future_hit_plane_crossing)

    def hit_plane_intersection(self, ball_position, ball_velocity) -> tuple[float, np.ndarray, np.ndarray]:
        trajectory = self.predictor.predict(ball_position, ball_velocity, self.prediction_horizon_s)
        x = trajectory.positions[:, 0]
        signed = x - self.virtual_hit_plane_x
        crossing = np.where(signed[:-1] * signed[1:] <= 0.0)[0]
        if crossing.size == 0:
            if self.require_future_hit_plane_crossing:
                raise ValueError(
                    f"ball trajectory does not cross hit plane x={self.virtual_hit_plane_x:.3f} within prediction horizon"
                )
            nearest = int(np.argmin(np.abs(signed)))
            return float(trajectory.times[nearest]), trajectory.positions[nearest].copy(), trajectory.velocities[nearest].copy()

        index = int(crossing[0])
        x0, x1 = x[index], x[index + 1]
        alpha = 0.0 if abs(x1 - x0) < 1.0e-12 else (self.virtual_hit_plane_x - x0) / (x1 - x0)
        alpha = float(np.clip(alpha, 0.0, 1.0))
        t = (1.0 - alpha) * trajectory.times[index] + alpha * trajectory.times[index + 1]
        pos = (1.0 - alpha) * trajectory.positions[index] + alpha * trajectory.positions[index + 1]
        vel = (1.0 - alpha) * trajectory.velocities[index] + alpha * trajectory.velocities[index + 1]
        pos[0] = self.virtual_hit_plane_x
        return float(t), pos, vel

    def desired_outgoing_ball_velocity(self, strike_position: np.ndarray) -> np.ndarray:
        strike_position = _vec3(strike_position, "strike_position")
        flight_time = max(self.post_hit_flight_time, 1.0e-6)
        return (self.desired_landing_point - strike_position) / flight_time - 0.5 * self.predictor.gravity * flight_time

    def racket_velocity_from_ball_velocities(self, v_ball_in: np.ndarray, v_ball_out: np.ndarray) -> np.ndarray:
        v_in = _vec3(v_ball_in, "v_ball_in")
        v_out = _vec3(v_ball_out, "v_ball_out")
        delta = v_out - v_in
        norm = np.linalg.norm(delta)
        if norm < 1.0e-9:
            return np.zeros(3, dtype=np.float64)
        normal = delta / norm
        scalar = (np.dot(v_out, normal) + self.racket_restitution * np.dot(v_in, normal)) / (
            1.0 + self.racket_restitution
        )
        return scalar * normal

    def plan(self, ball_position, ball_velocity) -> StrikePlan:
        t_strike, p_strike, v_in = self.hit_plane_intersection(ball_position, ball_velocity)
        v_out = self.desired_outgoing_ball_velocity(p_strike)
        v_racket = self.racket_velocity_from_ball_velocities(v_in, v_out)
        return StrikePlan(
            t_strike=t_strike,
            p_racket_target=p_strike,
            v_racket_target=v_racket,
            v_ball_in=v_in,
            v_ball_out=v_out,
        )


class BaseTargetPlanner:
    def __init__(
        self,
        *,
        racket_x_offset_b: float = 0.40,
        forehand_nominal_racket_y_b: float = -0.5,
        backhand_nominal_racket_y_b: float = 0.22,
        default_base_z: float = 0.78,
        strike_plane_x: float | None = None,
    ):
        if strike_plane_x is not None:
            racket_x_offset_b = strike_plane_x
        self.racket_x_offset_b = float(racket_x_offset_b)
        self.forehand_nominal_racket_y_b = float(forehand_nominal_racket_y_b)
        self.backhand_nominal_racket_y_b = float(backhand_nominal_racket_y_b)
        self.default_base_z = float(default_base_z)

    def _select_strike_type(
        self,
        racket_target_w: np.ndarray,
        current_base_w: np.ndarray,
        base_forward_xy_w: np.ndarray,
        strike_type: str | None,
    ) -> str:
        if strike_type is not None:
            strike_type = str(strike_type).lower()
            if strike_type not in {"forehand", "backhand"}:
                raise ValueError("strike_type must be 'forehand', 'backhand', or None.")
            return strike_type
        frame = _world_from_base_yaw(base_forward_xy_w)
        local_xy = frame.T @ (racket_target_w[:2] - current_base_w[:2])
        return "forehand" if local_xy[1] < 0.0 else "backhand"

    def plan(
        self,
        *,
        racket_target_w,
        current_base_xy_w=(0.0, 0.0, 0.78),
        base_forward_xy_w=(1.0, 0.0),
        strike_type: str | None = None,
    ) -> tuple[str, np.ndarray, np.ndarray]:
        racket_target_w = _vec3(racket_target_w, "racket_target_w")
        current_base_w = _base_position(current_base_xy_w, self.default_base_z)
        frame = _world_from_base_yaw(base_forward_xy_w)
        strike_type = self._select_strike_type(racket_target_w, current_base_w, frame[:, 0], strike_type)
        nominal_y = self.forehand_nominal_racket_y_b if strike_type == "forehand" else self.backhand_nominal_racket_y_b
        p_base_target_xy = racket_target_w[:2] - self.racket_x_offset_b * frame[:, 0] - nominal_y * frame[:, 1]
        delta_xy = racket_target_w[:2] - p_base_target_xy
        p_racket_target_b_xy = frame.T @ delta_xy
        p_racket_target_b = np.array(
            [p_racket_target_b_xy[0], p_racket_target_b_xy[1], racket_target_w[2] - current_base_w[2]],
            dtype=np.float64,
        )
        return strike_type, p_base_target_xy, p_racket_target_b


@dataclass(frozen=True)
class HitterWbcCommand:
    strike_type: str
    p_base_target_xy: np.ndarray
    p_racket_target_b: np.ndarray
    v_racket_target_w: np.ndarray
    time_to_strike: float
    strike_plan: StrikePlan


class HitterSystemPlanner:
    def __init__(
        self,
        *,
        strike_planner: StrikePlanner | None = None,
        base_planner: BaseTargetPlanner | None = None,
    ):
        self.strike_planner = strike_planner if strike_planner is not None else StrikePlanner()
        self.base_planner = base_planner if base_planner is not None else BaseTargetPlanner()

    def plan_command(
        self,
        ball_position,
        ball_velocity,
        *,
        current_base_xy_w=(0.0, 0.0, 0.78),
        base_forward_xy_w=(1.0, 0.0),
        strike_type: str | None = None,
    ) -> HitterWbcCommand:
        strike_plan = self.strike_planner.plan(ball_position, ball_velocity)
        strike_type, p_base_target_xy, p_racket_target_b = self.base_planner.plan(
            racket_target_w=strike_plan.p_racket_target,
            current_base_xy_w=current_base_xy_w,
            base_forward_xy_w=base_forward_xy_w,
            strike_type=strike_type,
        )
        return HitterWbcCommand(
            strike_type=strike_type,
            p_base_target_xy=p_base_target_xy,
            p_racket_target_b=p_racket_target_b,
            v_racket_target_w=strike_plan.v_racket_target,
            time_to_strike=strike_plan.t_strike,
            strike_plan=strike_plan,
        )
