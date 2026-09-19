from __future__ import annotations

import time
from collections import deque
from collections.abc import Mapping
from dataclasses import dataclass

import numpy as np

from utils.hitter_runtime_types import PlannerFailureReason, PlannerRejected


_AXIS_INDEX = {"x": 0, "y": 1, "z": 2}


def _vec2(value, name: str, *, reject_nonfinite: bool = False) -> np.ndarray:
    arr = np.asarray(value, dtype=np.float64).reshape(-1)
    if arr.shape != (2,):
        raise ValueError(f"{name} must have shape (2,), got {arr.shape}.")
    if not np.isfinite(arr).all():
        if reject_nonfinite:
            raise PlannerRejected(
                PlannerFailureReason.NONFINITE_INPUT_OR_OUTPUT,
                f"{name} contains non-finite values: {arr}.",
            )
        raise ValueError(f"{name} contains non-finite values: {arr}.")
    return arr


def _vec3(value, name: str, *, reject_nonfinite: bool = False) -> np.ndarray:
    arr = np.asarray(value, dtype=np.float64).reshape(-1)
    if arr.shape != (3,):
        raise ValueError(f"{name} must have shape (3,), got {arr.shape}.")
    if not np.isfinite(arr).all():
        if reject_nonfinite:
            raise PlannerRejected(
                PlannerFailureReason.NONFINITE_INPUT_OR_OUTPUT,
                f"{name} contains non-finite values: {arr}.",
            )
        raise ValueError(f"{name} contains non-finite values: {arr}.")
    return arr


def _finite_float(value, name: str) -> float:
    if isinstance(value, (bool, np.bool_)) or not isinstance(
        value,
        (int, float, np.integer, np.floating),
    ):
        raise TypeError(f"{name} must be a real number.")
    result = float(value)
    if not np.isfinite(result):
        raise ValueError(f"{name} must be finite.")
    return result


def _finite_nonnegative_float(value, name: str) -> float:
    result = _finite_float(value, name)
    if result < 0.0:
        raise ValueError(f"{name} must be non-negative.")
    return result


def _component_ranges_mps(value, name: str) -> dict[int, tuple[float, float]] | None:
    if value is None:
        return None
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be a mapping from axis name to [low, high].")

    ranges: dict[int, tuple[float, float]] = {}
    for axis, index in _AXIS_INDEX.items():
        raw_range = value.get(axis)
        if raw_range is None:
            continue
        range_array = np.asarray(raw_range, dtype=np.float64).reshape(-1)
        if range_array.shape != (2,):
            raise ValueError(
                f"{name}.{axis} must have shape (2,), got {range_array.shape}."
            )
        if not np.isfinite(range_array).all():
            raise ValueError(f"{name}.{axis} contains non-finite values.")
        low, high = float(range_array[0]), float(range_array[1])
        if high < low:
            raise ValueError(f"{name}.{axis} has high < low: {raw_range!r}.")
        ranges[index] = (low, high)

    return ranges or None


def _base_position(value, default_z: float, *, reject_nonfinite: bool = False) -> np.ndarray:
    arr = np.asarray(value, dtype=np.float64).reshape(-1)
    if arr.shape == (2,):
        arr = np.array([arr[0], arr[1], default_z], dtype=np.float64)
    if arr.shape != (3,):
        raise ValueError(f"current_base_xy_w must contain 2 or 3 values, got shape {arr.shape}.")
    if not np.isfinite(arr).all():
        if reject_nonfinite:
            raise PlannerRejected(
                PlannerFailureReason.NONFINITE_INPUT_OR_OUTPUT,
                f"current_base_xy_w contains non-finite values: {arr}.",
            )
        raise ValueError(f"current_base_xy_w contains non-finite values: {arr}.")
    return arr


def _world_from_base_yaw(base_forward_xy_w, *, reject_nonfinite: bool = False) -> np.ndarray:
    forward = _vec2(
        base_forward_xy_w,
        "base_forward_xy_w",
        reject_nonfinite=reject_nonfinite,
    )
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

        if count < self.min_samples:
            return BallStateEstimate(
                position=positions[-1].copy(),
                velocity=np.zeros(3, dtype=np.float64),
                sample_count=count,
                valid=False,
                bounce_detected=bounce_detected,
            )

        tau = times - float(current_time)
        design = np.column_stack([np.ones(count, dtype=np.float64), tau, tau * tau])
        coeffs, *_ = np.linalg.lstsq(design, positions, rcond=None)
        position = coeffs[0].copy()
        velocity = coeffs[1].copy()

        return BallStateEstimate(
            position=position,
            velocity=velocity,
            sample_count=count,
            valid=True,
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

    def _descending_table_contact_time(
        self,
        pos: np.ndarray,
        vel: np.ndarray,
        acc: np.ndarray,
        segment_dt: float,
        contact_z: float,
    ) -> float | None:
        time_tolerance = 1.0e-12
        velocity_tolerance = 1.0e-10
        if segment_dt <= 0.0 or pos[2] < contact_z - 1.0e-10:
            return None

        height = float(pos[2] - contact_z)
        velocity_z = float(vel[2])
        acceleration_z = float(acc[2])
        if abs(acceleration_z) < 1.0e-12:
            if velocity_z == 0.0:
                return None
            roots = (-height / velocity_z,)
        else:
            discriminant = (
                velocity_z * velocity_z
                - 2.0 * acceleration_z * height
            )
            if discriminant < -time_tolerance:
                return None
            root = float(np.sqrt(max(discriminant, 0.0)))
            roots = (
                (-velocity_z - root) / acceleration_z,
                (-velocity_z + root) / acceleration_z,
            )

        valid = []
        for value in roots:
            if not (
                -time_tolerance
                <= value
                <= segment_dt + time_tolerance
            ):
                continue
            contact_time = float(np.clip(value, 0.0, segment_dt))
            contact_velocity_z = velocity_z + acceleration_z * contact_time
            if contact_velocity_z < -velocity_tolerance:
                valid.append(contact_time)
        return min(valid) if valid else None

    def _first_table_exit_time(
        self,
        pos: np.ndarray,
        vel: np.ndarray,
        acc: np.ndarray,
        duration: float,
    ) -> float | None:
        half_extent = np.array(
            [0.5 * self.table_length, 0.5 * self.table_width],
            dtype=np.float64,
        )
        lower = self.table_center_xy - half_extent
        upper = self.table_center_xy + half_extent
        time_tolerance = 1.0e-12
        velocity_tolerance = 1.0e-10
        candidates = []
        for axis in (0, 1):
            for boundary, outward_sign in (
                (lower[axis], -1.0),
                (upper[axis], 1.0),
            ):
                coefficient_a = 0.5 * float(acc[axis])
                coefficient_b = float(vel[axis])
                coefficient_c = float(pos[axis] - boundary)
                if abs(coefficient_a) < 1.0e-12:
                    if abs(coefficient_b) < 1.0e-12:
                        roots = ()
                    else:
                        roots = (-coefficient_c / coefficient_b,)
                else:
                    discriminant = (
                        coefficient_b * coefficient_b
                        - 4.0 * coefficient_a * coefficient_c
                    )
                    if discriminant < -time_tolerance:
                        roots = ()
                    else:
                        root = float(np.sqrt(max(discriminant, 0.0)))
                        roots = (
                            (-coefficient_b - root)
                            / (2.0 * coefficient_a),
                            (-coefficient_b + root)
                            / (2.0 * coefficient_a),
                        )
                for value in roots:
                    if not (
                        -time_tolerance
                        <= value
                        <= duration + time_tolerance
                    ):
                        continue
                    exit_time = float(np.clip(value, 0.0, duration))
                    exit_velocity = vel[axis] + acc[axis] * exit_time
                    if outward_sign * exit_velocity <= velocity_tolerance:
                        continue
                    candidate_xy = (
                        pos[:2]
                        + vel[:2] * exit_time
                        + 0.5 * acc[:2] * exit_time * exit_time
                    )
                    if np.all(candidate_xy >= lower - 1.0e-10) and np.all(
                        candidate_xy <= upper + 1.0e-10
                    ):
                        candidates.append(exit_time)
        return min(candidates) if candidates else None

    def _supported_table_state(
        self,
        pos: np.ndarray,
        vel: np.ndarray,
        duration: float,
        contact_z: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        supported_vel = vel.copy()
        supported_vel[2] = 0.0
        supported_acc = (
            -self.drag_coefficient
            * np.linalg.norm(supported_vel)
            * supported_vel
        )
        supported_acc[2] = 0.0
        exit_time = self._first_table_exit_time(
            pos,
            supported_vel,
            supported_acc,
            duration,
        )
        if exit_time is not None:
            exit_pos = (
                pos
                + supported_vel * exit_time
                + 0.5 * supported_acc * exit_time * exit_time
            )
            exit_pos[2] = contact_z
            exit_vel = supported_vel + supported_acc * exit_time
            exit_vel[2] = 0.0
            free_duration = duration - exit_time
            free_acc = (
                self.gravity
                - self.drag_coefficient
                * np.linalg.norm(exit_vel)
                * exit_vel
            )
            return (
                exit_pos
                + exit_vel * free_duration
                + 0.5 * free_acc * free_duration * free_duration,
                exit_vel + free_acc * free_duration,
            )

        next_pos = (
            pos
            + supported_vel * duration
            + 0.5 * supported_acc * duration * duration
        )
        next_vel = supported_vel + supported_acc * duration
        next_pos[2] = contact_z
        next_vel[2] = 0.0
        return next_pos, next_vel

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
            resting_on_table = bool(
                abs(pos[2] - contact_z) <= 1.0e-10
                and abs(vel[2]) <= 1.0e-10
                and self._inside_table(pos[:2])
            )
            if resting_on_table:
                pos, vel = self._supported_table_state(
                    pos,
                    vel,
                    self.dt,
                    contact_z,
                )
                continue

            acc = self.gravity - self.drag_coefficient * np.linalg.norm(vel) * vel
            next_vel = vel + acc * self.dt
            next_pos = pos + vel * self.dt + 0.5 * acc * self.dt * self.dt
            contact_dt = self._descending_table_contact_time(
                pos,
                vel,
                acc,
                self.dt,
                contact_z,
            )
            if contact_dt is not None:
                contact_pos = (
                    pos
                    + vel * contact_dt
                    + 0.5 * acc * contact_dt * contact_dt
                )
                contact_pos[2] = contact_z
                if self._inside_table(contact_pos[:2]):
                    contact_vel = vel + acc * contact_dt
                    rebound_vel = contact_vel.copy()
                    rebound_vel[:2] *= self.horizontal_restitution
                    rebound_vel[2] = (
                        -rebound_vel[2] * self.vertical_restitution
                    )
                    remaining_dt = self.dt - contact_dt
                    rebound_acc = (
                        self.gravity
                        - self.drag_coefficient
                        * np.linalg.norm(rebound_vel)
                        * rebound_vel
                    )
                    second_contact_dt = self._descending_table_contact_time(
                        contact_pos,
                        rebound_vel,
                        rebound_acc,
                        remaining_dt,
                        contact_z,
                    )
                    if rebound_vel[2] <= 1.0e-10:
                        next_pos, next_vel = self._supported_table_state(
                            contact_pos,
                            rebound_vel,
                            remaining_dt,
                            contact_z,
                        )
                    elif second_contact_dt is not None:
                        second_contact_pos = (
                            contact_pos
                            + rebound_vel * second_contact_dt
                            + 0.5
                            * rebound_acc
                            * second_contact_dt
                            * second_contact_dt
                        )
                        second_contact_pos[2] = contact_z
                        if self._inside_table(second_contact_pos[:2]):
                            second_contact_vel = (
                                rebound_vel
                                + rebound_acc * second_contact_dt
                            )
                            second_contact_vel = second_contact_vel.copy()
                            second_contact_vel[:2] *= (
                                self.horizontal_restitution
                            )
                            next_pos, next_vel = self._supported_table_state(
                                second_contact_pos,
                                second_contact_vel,
                                remaining_dt - second_contact_dt,
                                contact_z,
                            )
                        else:
                            next_pos = (
                                contact_pos
                                + rebound_vel * remaining_dt
                                + 0.5
                                * rebound_acc
                                * remaining_dt
                                * remaining_dt
                            )
                            next_vel = (
                                rebound_vel
                                + rebound_acc * remaining_dt
                            )
                    else:
                        next_pos = (
                            contact_pos
                            + rebound_vel * remaining_dt
                            + 0.5
                            * rebound_acc
                            * remaining_dt
                            * remaining_dt
                        )
                        next_vel = (
                            rebound_vel
                            + rebound_acc * remaining_dt
                        )
            pos, vel = next_pos, next_vel
        return BallTrajectory(times=times, positions=positions, velocities=velocities)


@dataclass(frozen=True)
class StrikePlan:
    t_strike: float
    p_racket_target: np.ndarray
    v_racket_target: np.ndarray
    v_ball_in: np.ndarray
    v_ball_out: np.ndarray
    raw_racket_normal_speed_mps: float = 0.0
    commanded_racket_normal_speed_mps: float = 0.0
    minimum_racket_normal_speed_mps: float = 0.0
    racket_speed_floor_applied: bool = False


class StrikePlanner:
    def __init__(
        self,
        *,
        predictor: BallTrajectoryPredictor | None = None,
        virtual_hit_plane_x: float = 0.0,
        desired_landing_point=(2.05, 0.0, 0.78),
        forehand_desired_landing_point=None,
        backhand_desired_landing_point=None,
        post_hit_flight_time: float = 0.55,
        racket_restitution: float = 0.85,
        prediction_horizon_s: float = 2.0,
        maximum_prediction_horizon_s: float = 5.0,
        minimum_hit_height: float = 0.76,
        maximum_hit_height: float = 1.45,
        require_future_hit_plane_crossing: bool = False,
        minimum_racket_normal_speed_mps: float = 0.0,
        racket_velocity_component_ranges_mps=None,
        backhand_racket_velocity_component_ranges_mps=None,
        backhand_edge_landing_start_y_w_m: float = 0.30,
        backhand_edge_landing_full_y_w_m: float = 0.50,
        backhand_edge_landing_y_decrement_m: float = 0.0,
        backhand_edge_landing_threshold_y_w_m: float = 0.20,
        backhand_edge_landing_target_y_w_m: float | None = None,
    ):
        self.predictor = predictor if predictor is not None else BallTrajectoryPredictor()
        self.virtual_hit_plane_x = float(virtual_hit_plane_x)
        self.desired_landing_point = _vec3(desired_landing_point, "desired_landing_point")
        self.forehand_desired_landing_point = (
            self.desired_landing_point.copy()
            if forehand_desired_landing_point is None
            else _vec3(
                forehand_desired_landing_point,
                "forehand_desired_landing_point",
            )
        )
        self.backhand_desired_landing_point = (
            self.desired_landing_point.copy()
            if backhand_desired_landing_point is None
            else _vec3(
                backhand_desired_landing_point,
                "backhand_desired_landing_point",
            )
        )
        self.post_hit_flight_time = float(post_hit_flight_time)
        self.racket_restitution = float(racket_restitution)
        self.prediction_horizon_s = float(prediction_horizon_s)
        self.maximum_prediction_horizon_s = float(maximum_prediction_horizon_s)
        self.minimum_hit_height = float(minimum_hit_height)
        self.maximum_hit_height = float(maximum_hit_height)
        # Kept for constructor compatibility; planner admission now always
        # requires a finite directed crossing and never falls back to nearest.
        self.require_future_hit_plane_crossing = bool(require_future_hit_plane_crossing)
        self.minimum_racket_normal_speed_mps = _finite_nonnegative_float(
            minimum_racket_normal_speed_mps,
            "minimum_racket_normal_speed_mps",
        )
        self.racket_velocity_component_ranges_mps = _component_ranges_mps(
            racket_velocity_component_ranges_mps,
            "racket_velocity_component_ranges_mps",
        )
        self.backhand_racket_velocity_component_ranges_mps = _component_ranges_mps(
            backhand_racket_velocity_component_ranges_mps,
            "backhand_racket_velocity_component_ranges_mps",
        )
        self.backhand_edge_landing_start_y_w_m = _finite_nonnegative_float(
            backhand_edge_landing_start_y_w_m,
            "backhand_edge_landing_start_y_w_m",
        )
        self.backhand_edge_landing_full_y_w_m = _finite_nonnegative_float(
            backhand_edge_landing_full_y_w_m,
            "backhand_edge_landing_full_y_w_m",
        )
        self.backhand_edge_landing_y_decrement_m = _finite_nonnegative_float(
            backhand_edge_landing_y_decrement_m,
            "backhand_edge_landing_y_decrement_m",
        )
        self.backhand_edge_landing_threshold_y_w_m = _finite_nonnegative_float(
            backhand_edge_landing_threshold_y_w_m,
            "backhand_edge_landing_threshold_y_w_m",
        )
        self.backhand_edge_landing_target_y_w_m = (
            None
            if backhand_edge_landing_target_y_w_m is None
            else _finite_float(
                backhand_edge_landing_target_y_w_m,
                "backhand_edge_landing_target_y_w_m",
            )
        )

        finite_parameters = {
            "virtual_hit_plane_x": self.virtual_hit_plane_x,
            "prediction_horizon_s": self.prediction_horizon_s,
            "maximum_prediction_horizon_s": self.maximum_prediction_horizon_s,
            "minimum_hit_height": self.minimum_hit_height,
            "maximum_hit_height": self.maximum_hit_height,
        }
        for name, value in finite_parameters.items():
            if not np.isfinite(value):
                raise ValueError(f"{name} must be finite, got {value!r}.")
        if self.prediction_horizon_s <= 0.0:
            raise ValueError("prediction_horizon_s must be positive.")
        if self.maximum_prediction_horizon_s < self.prediction_horizon_s:
            raise ValueError(
                "maximum_prediction_horizon_s must be greater than or equal to "
                "prediction_horizon_s."
            )
        if self.minimum_hit_height >= self.maximum_hit_height:
            raise ValueError("minimum_hit_height must be less than maximum_hit_height.")
        if self.backhand_edge_landing_y_decrement_m > 0.0:
            if (
                self.backhand_edge_landing_full_y_w_m
                <= self.backhand_edge_landing_start_y_w_m
            ):
                raise ValueError(
                    "backhand_edge_landing_full_y_w_m must be greater than "
                    "backhand_edge_landing_start_y_w_m when the bias is enabled."
                )
            half_table_width = 0.5 * float(self.predictor.table_width)
            if (
                not np.isfinite(half_table_width)
                or half_table_width <= 0.0
            ):
                raise ValueError("predictor.table_width must be finite and positive.")
            if self.backhand_edge_landing_full_y_w_m > half_table_width:
                raise ValueError(
                    "backhand_edge_landing_full_y_w_m must not exceed half the "
                    "table width."
                )
            base_landing_y = float(self.backhand_desired_landing_point[1])
            final_landing_y = (
                base_landing_y - self.backhand_edge_landing_y_decrement_m
            )
            if not (
                -half_table_width <= base_landing_y <= half_table_width
                and -half_table_width <= final_landing_y <= half_table_width
            ):
                raise ValueError(
                    "backhand edge landing targets must remain inside the table "
                    "width."
                )
        if self.backhand_edge_landing_target_y_w_m is not None:
            if self.backhand_edge_landing_y_decrement_m > 0.0:
                raise ValueError(
                    "direct and smooth backhand edge landing rules cannot both "
                    "be enabled."
                )
            half_table_width = 0.5 * float(self.predictor.table_width)
            if (
                not np.isfinite(half_table_width)
                or half_table_width <= 0.0
            ):
                raise ValueError("predictor.table_width must be finite and positive.")
            table_center_y = float(self.predictor.table_center_xy[1])
            table_min_y = table_center_y - half_table_width
            table_max_y = table_center_y + half_table_width
            if self.backhand_edge_landing_threshold_y_w_m >= table_max_y:
                raise ValueError(
                    "backhand_edge_landing_threshold_y_w_m must be below the "
                    "positive-y table edge."
                )
            base_landing_y = float(self.backhand_desired_landing_point[1])
            override_landing_y = self.backhand_edge_landing_target_y_w_m
            if not (
                table_min_y <= base_landing_y <= table_max_y
                and table_min_y <= override_landing_y <= table_max_y
            ):
                raise ValueError(
                    "backhand edge landing targets must remain inside the table "
                    "width."
                )

    def hit_plane_intersection(self, ball_position, ball_velocity) -> tuple[float, np.ndarray, np.ndarray]:
        ball_position = _vec3(
            ball_position,
            "ball_position",
            reject_nonfinite=True,
        )
        ball_velocity = _vec3(
            ball_velocity,
            "ball_velocity",
            reject_nonfinite=True,
        )
        if not ball_position[0] > self.virtual_hit_plane_x:
            raise PlannerRejected(
                PlannerFailureReason.BALL_NOT_INCOMING,
                f"ball admission requires x > {self.virtual_hit_plane_x:g}, "
                f"got x={ball_position[0]:.3f}."
            )
        if not ball_velocity[0] < 0.0:
            raise PlannerRejected(
                PlannerFailureReason.BALL_NOT_INCOMING,
                f"ball admission requires vx < 0, got vx={ball_velocity[0]:.3f}."
            )

        horizon = self.prediction_horizon_s
        while True:
            trajectory = self.predictor.predict(ball_position, ball_velocity, horizon)
            if not (
                np.isfinite(trajectory.times).all()
                and np.isfinite(trajectory.positions).all()
                and np.isfinite(trajectory.velocities).all()
            ):
                raise PlannerRejected(
                    PlannerFailureReason.NONFINITE_INPUT_OR_OUTPUT,
                    "ball trajectory predictor produced non-finite values.",
                )

            x = trajectory.positions[:, 0]
            signed = x - self.virtual_hit_plane_x
            crossing = np.where(
                (signed[:-1] > 0.0)
                & (signed[1:] <= 0.0)
                & (trajectory.velocities[:-1, 0] < 0.0)
            )[0]
            if crossing.size:
                index = int(crossing[0])
                x0, x1 = x[index], x[index + 1]
                alpha = (
                    0.0
                    if abs(x1 - x0) < 1.0e-12
                    else (self.virtual_hit_plane_x - x0) / (x1 - x0)
                )
                alpha = float(np.clip(alpha, 0.0, 1.0))
                t = (
                    (1.0 - alpha) * trajectory.times[index]
                    + alpha * trajectory.times[index + 1]
                )
                if t <= horizon + 1.0e-12:
                    pos = (
                        (1.0 - alpha) * trajectory.positions[index]
                        + alpha * trajectory.positions[index + 1]
                    )
                    vel = (
                        (1.0 - alpha) * trajectory.velocities[index]
                        + alpha * trajectory.velocities[index + 1]
                    )
                    pos[0] = self.virtual_hit_plane_x
                    if not (self.minimum_hit_height < pos[2] <= self.maximum_hit_height):
                        raise PlannerRejected(
                            PlannerFailureReason.HIT_HEIGHT_OUT_OF_RANGE,
                            f"predicted hit height {pos[2]:.3f} is outside "
                            f"({self.minimum_hit_height:.3f}, {self.maximum_hit_height:.3f}]"
                        )
                    return float(t), pos, vel

            if horizon >= self.maximum_prediction_horizon_s:
                break
            horizon = min(
                horizon + self.prediction_horizon_s,
                self.maximum_prediction_horizon_s,
            )

        raise PlannerRejected(
            PlannerFailureReason.NO_FUTURE_CROSSING,
            f"ball trajectory has no directed crossing of hit plane "
            f"x={self.virtual_hit_plane_x:.3f} within maximum prediction horizon "
            f"{self.maximum_prediction_horizon_s:.3f}s"
        )

    def _free_flight_endpoint(
        self,
        initial_position: np.ndarray,
        initial_velocity: np.ndarray,
        flight_time: float,
    ) -> np.ndarray:
        position = _vec3(
            initial_position,
            "free_flight_initial_position",
            reject_nonfinite=True,
        ).copy()
        velocity = _vec3(
            initial_velocity,
            "free_flight_initial_velocity",
            reject_nonfinite=True,
        ).copy()
        duration = float(flight_time)
        dt = float(self.predictor.dt)
        drag_coefficient = float(self.predictor.drag_coefficient)
        gravity = _vec3(
            self.predictor.gravity,
            "free_flight_gravity",
            reject_nonfinite=True,
        )
        if not np.isfinite([duration, dt, drag_coefficient]).all():
            raise PlannerRejected(
                PlannerFailureReason.NONFINITE_INPUT_OR_OUTPUT,
                "free-flight parameters contain non-finite values.",
            )
        if duration <= 0.0 or dt <= 0.0:
            raise PlannerRejected(
                PlannerFailureReason.INTERNAL_ERROR,
                "free-flight duration and integration dt must be positive.",
            )

        maximum_step_count = 10_000
        required_steps = duration / dt
        if (
            not np.isfinite(required_steps)
            or required_steps > maximum_step_count
        ):
            raise PlannerRejected(
                PlannerFailureReason.INTERNAL_ERROR,
                "free-flight integration requires too many steps; "
                f"maximum is {maximum_step_count}.",
            )
        step_count = int(np.ceil(required_steps))
        elapsed = 0.0
        for _ in range(step_count):
            remaining = duration - elapsed
            if remaining <= 0.0:
                break
            step_dt = min(dt, remaining)
            acceleration = (
                gravity
                - drag_coefficient * np.linalg.norm(velocity) * velocity
            )
            position = (
                position
                + velocity * step_dt
                + 0.5 * acceleration * step_dt * step_dt
            )
            velocity = velocity + acceleration * step_dt
            if not (
                np.isfinite(position).all()
                and np.isfinite(velocity).all()
            ):
                raise PlannerRejected(
                    PlannerFailureReason.NONFINITE_INPUT_OR_OUTPUT,
                    "free-flight integration produced non-finite values.",
                )
            elapsed += step_dt
        return position

    def desired_landing_point_for_strike(
        self,
        strike_position,
        *,
        strike_type: str | None = None,
    ) -> np.ndarray:
        strike_position = _vec3(
            strike_position,
            "strike_position",
            reject_nonfinite=True,
        )
        resolved_strike_type = (
            strike_type_from_table_y(strike_position)
            if strike_type is None
            else validate_explicit_strike_type(strike_type)
        )
        target = (
            self.forehand_desired_landing_point.copy()
            if resolved_strike_type == "forehand"
            else self.backhand_desired_landing_point.copy()
        )
        if (
            resolved_strike_type == "backhand"
            and self.backhand_edge_landing_target_y_w_m is not None
            and strike_position[1] > self.backhand_edge_landing_threshold_y_w_m
        ):
            target[1] = self.backhand_edge_landing_target_y_w_m
        elif (
            resolved_strike_type == "backhand"
            and self.backhand_edge_landing_y_decrement_m > 0.0
        ):
            start = self.backhand_edge_landing_start_y_w_m
            full = self.backhand_edge_landing_full_y_w_m
            u = np.clip((strike_position[1] - start) / (full - start), 0.0, 1.0)
            weight = u * u * (3.0 - 2.0 * u)
            target[1] -= self.backhand_edge_landing_y_decrement_m * weight
        return target

    def desired_outgoing_ball_velocity(
        self,
        strike_position: np.ndarray,
        *,
        strike_type: str | None = None,
    ) -> np.ndarray:
        strike_position = _vec3(
            strike_position,
            "strike_position",
            reject_nonfinite=True,
        )
        flight_time = float(self.post_hit_flight_time)
        if not np.isfinite(flight_time):
            raise PlannerRejected(
                PlannerFailureReason.NONFINITE_INPUT_OR_OUTPUT,
                "post_hit_flight_time is non-finite.",
            )
        flight_time = max(flight_time, 1.0e-6)
        landing_point = self.desired_landing_point_for_strike(
            strike_position,
            strike_type=strike_type,
        )

        # Start from the no-drag closed-form solution, then use bounded
        # shooting corrections against the same discrete free-flight dynamics
        # as the trajectory predictor (gravity plus quadratic drag).
        outgoing_velocity = (
            (landing_point - strike_position) / flight_time
            - 0.5 * self.predictor.gravity * flight_time
        )
        endpoint_tolerance_m = 1.0e-5
        maximum_iterations = 12
        endpoint_error = np.full(3, np.inf, dtype=np.float64)
        for _ in range(maximum_iterations):
            endpoint = self._free_flight_endpoint(
                strike_position,
                outgoing_velocity,
                flight_time,
            )
            endpoint_error = landing_point - endpoint
            if not np.isfinite(endpoint_error).all():
                raise PlannerRejected(
                    PlannerFailureReason.NONFINITE_INPUT_OR_OUTPUT,
                    "outgoing-velocity shooting produced a non-finite error.",
                )
            if float(np.linalg.norm(endpoint_error)) <= endpoint_tolerance_m:
                return _vec3(
                    outgoing_velocity,
                    "desired_outgoing_ball_velocity",
                    reject_nonfinite=True,
                )
            outgoing_velocity = (
                outgoing_velocity + endpoint_error / flight_time
            )

        raise PlannerRejected(
            PlannerFailureReason.INTERNAL_ERROR,
            "outgoing-velocity shooting did not converge; endpoint error "
            f"is {float(np.linalg.norm(endpoint_error)):.6f} m.",
        )

    def _collision_normal_and_raw_racket_speed(
        self,
        v_ball_in: np.ndarray,
        v_ball_out: np.ndarray,
    ) -> tuple[np.ndarray, float]:
        v_in = _vec3(v_ball_in, "v_ball_in", reject_nonfinite=True)
        v_out = _vec3(v_ball_out, "v_ball_out", reject_nonfinite=True)
        delta = v_out - v_in
        norm = float(np.linalg.norm(delta))
        if norm < 1.0e-9:
            return np.zeros(3, dtype=np.float64), 0.0
        normal = delta / norm
        scalar = (np.dot(v_out, normal) + self.racket_restitution * np.dot(v_in, normal)) / (
            1.0 + self.racket_restitution
        )
        return normal, float(scalar)

    def racket_velocity_from_ball_velocities(
        self,
        v_ball_in: np.ndarray,
        v_ball_out: np.ndarray,
    ) -> np.ndarray:
        normal, raw_speed = self._collision_normal_and_raw_racket_speed(
            v_ball_in,
            v_ball_out,
        )
        if not np.any(normal):
            if self.minimum_racket_normal_speed_mps > 0.0:
                raise PlannerRejected(
                    PlannerFailureReason.INTERNAL_ERROR,
                    "collision normal is undefined because "
                    "v_ball_out equals v_ball_in",
                )
            return np.zeros(3, dtype=np.float64)
        commanded_speed = raw_speed
        if self.minimum_racket_normal_speed_mps > 0.0:
            commanded_speed = max(
                raw_speed,
                self.minimum_racket_normal_speed_mps,
            )
        return commanded_speed * normal

    def _align_racket_velocity_components(
        self, v_racket: np.ndarray, *, strike_type: str | None = None
    ) -> np.ndarray:
        aligned = _vec3(v_racket, "v_racket_target").copy()
        ranges = self.racket_velocity_component_ranges_mps
        if strike_type == "backhand" and self.backhand_racket_velocity_component_ranges_mps:
            ranges = dict(ranges or {})
            ranges.update(self.backhand_racket_velocity_component_ranges_mps)
        if ranges is None:
            return aligned
        for index, (low, high) in ranges.items():
            aligned[index] = np.clip(aligned[index], low, high)
        return aligned

    def plan(
        self,
        ball_position,
        ball_velocity,
        *,
        strike_type: str | None = None,
    ) -> StrikePlan:
        t_strike, p_strike, v_in = self.hit_plane_intersection(ball_position, ball_velocity)
        resolved_strike_type = (
            strike_type_from_table_y(p_strike)
            if strike_type is None
            else validate_explicit_strike_type(strike_type)
        )
        v_out = self.desired_outgoing_ball_velocity(
            p_strike,
            strike_type=resolved_strike_type,
        )
        normal, raw_normal_speed = (
            self._collision_normal_and_raw_racket_speed(v_in, v_out)
        )
        v_racket = self.racket_velocity_from_ball_velocities(v_in, v_out)
        v_racket = _vec3(
            v_racket,
            "v_racket_target",
            reject_nonfinite=True,
        )
        commanded_normal_speed = (
            float(np.dot(v_racket, normal))
            if np.any(normal)
            else 0.0
        )
        floor_applied = bool(
            np.any(normal)
            and self.minimum_racket_normal_speed_mps > 0.0
            and raw_normal_speed
            < self.minimum_racket_normal_speed_mps
        )
        v_racket = self._align_racket_velocity_components(
            v_racket, strike_type=resolved_strike_type
        )
        return StrikePlan(
            t_strike=t_strike,
            p_racket_target=p_strike,
            v_racket_target=v_racket,
            v_ball_in=v_in,
            v_ball_out=v_out,
            raw_racket_normal_speed_mps=raw_normal_speed,
            commanded_racket_normal_speed_mps=commanded_normal_speed,
            minimum_racket_normal_speed_mps=(
                self.minimum_racket_normal_speed_mps
            ),
            racket_speed_floor_applied=floor_applied,
        )


class BaseTargetPlanner:
    def __init__(
        self,
        *,
        racket_x_offset_b: float = 0.40,
        forehand_nominal_racket_y_b: float = -0.5,
        backhand_nominal_racket_y_b: float = 0.22,
        default_base_z: float = 0.78,
    ):
        self.racket_x_offset_b = float(racket_x_offset_b)
        self.forehand_nominal_racket_y_b = float(forehand_nominal_racket_y_b)
        self.backhand_nominal_racket_y_b = float(backhand_nominal_racket_y_b)
        self.default_base_z = float(default_base_z)

    def plan(
        self,
        *,
        racket_target_w,
        current_base_xy_w=(0.0, 0.0, 0.78),
        base_forward_xy_w=(1.0, 0.0),
        strike_type: str,
    ) -> tuple[str, np.ndarray]:
        racket_target_w = _vec3(
            racket_target_w,
            "racket_target_w",
            reject_nonfinite=True,
        )
        current_base_w = _base_position(
            current_base_xy_w,
            self.default_base_z,
            reject_nonfinite=True,
        )
        frame = _world_from_base_yaw(
            base_forward_xy_w,
            reject_nonfinite=True,
        )
        strike_type = validate_explicit_strike_type(strike_type)
        nominal_y = self.forehand_nominal_racket_y_b if strike_type == "forehand" else self.backhand_nominal_racket_y_b
        p_base_target_xy = racket_target_w[:2] - self.racket_x_offset_b * frame[:, 0] - nominal_y * frame[:, 1]
        return strike_type, p_base_target_xy


def validate_explicit_strike_type(strike_type: str) -> str:
    resolved = str(strike_type).strip().lower()
    if resolved not in {"forehand", "backhand"}:
        raise ValueError("strike_type must be 'forehand' or 'backhand'.")
    return resolved


def strike_type_from_table_y(racket_target_w) -> str:
    target = _vec3(
        racket_target_w,
        "racket_target_w",
        reject_nonfinite=True,
    )
    return "forehand" if float(target[1]) < 0.0 else "backhand"


@dataclass(frozen=True)
class HitterWbcCommand:
    strike_type: str
    p_base_target_xy: np.ndarray
    v_racket_target_w: np.ndarray
    time_to_strike: float
    strike_plan: StrikePlan
    strike_table_y_w: float
    strike_side_source: str

    @property
    def expected_strike_type_from_table_y(self) -> str:
        return strike_type_from_table_y(
            np.asarray(
                [0.0, float(self.strike_table_y_w), 0.0],
                dtype=np.float64,
            )
        )

    @property
    def strike_type_consistent(self) -> bool:
        return (
            self.strike_type
            == self.expected_strike_type_from_table_y
        )


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
        forced_strike_type = (
            None
            if strike_type is None
            else validate_explicit_strike_type(strike_type)
        )
        if forced_strike_type is None or not isinstance(
            self.strike_planner,
            StrikePlanner,
        ):
            strike_plan = self.strike_planner.plan(ball_position, ball_velocity)
        else:
            strike_plan = self.strike_planner.plan(
                ball_position,
                ball_velocity,
                strike_type=forced_strike_type,
            )
        time_to_strike = float(strike_plan.t_strike)
        if not np.isfinite(time_to_strike):
            raise PlannerRejected(
                PlannerFailureReason.NONFINITE_INPUT_OR_OUTPUT,
                "strike_plan.t_strike is non-finite.",
            )
        for name in ("v_racket_target", "v_ball_in", "v_ball_out"):
            _vec3(
                getattr(strike_plan, name),
                f"strike_plan.{name}",
                reject_nonfinite=True,
            )
        strike_table_y_w = float(
            _vec3(
                strike_plan.p_racket_target,
                "strike_plan.p_racket_target",
                reject_nonfinite=True,
            )[1]
        )
        resolved_strike_type = (
            strike_type_from_table_y(strike_plan.p_racket_target)
            if forced_strike_type is None
            else forced_strike_type
        )
        strike_side_source = "table_y" if forced_strike_type is None else "forced"
        resolved_strike_type, p_base_target_xy = self.base_planner.plan(
            racket_target_w=strike_plan.p_racket_target,
            current_base_xy_w=current_base_xy_w,
            base_forward_xy_w=base_forward_xy_w,
            strike_type=resolved_strike_type,
        )
        _vec2(
            p_base_target_xy,
            "p_base_target_xy",
            reject_nonfinite=True,
        )
        return HitterWbcCommand(
            strike_type=resolved_strike_type,
            p_base_target_xy=p_base_target_xy,
            v_racket_target_w=strike_plan.v_racket_target,
            time_to_strike=time_to_strike,
            strike_plan=strike_plan,
            strike_table_y_w=strike_table_y_w,
            strike_side_source=strike_side_source,
        )
