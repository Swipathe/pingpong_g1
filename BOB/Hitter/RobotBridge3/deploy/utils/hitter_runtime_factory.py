from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

import numpy as np

from utils.hitter_planner import (
    BallStateEstimator,
    BallTrajectoryPredictor,
    BaseTargetPlanner,
    HitterSystemPlanner,
    StrikePlanner,
)
from utils.hitter_realtime import HitterCommandLifecycle


def build_ball_state_estimator(
    planner_config: Mapping[str, object],
) -> BallStateEstimator:
    """Build the exact estimator currently used by RealWorld."""
    estimator_window_size = int(
        planner_config.get("state_estimator_window_size", 31)
    )
    return BallStateEstimator(
        window_size=estimator_window_size,
        min_samples=int(
            planner_config.get(
                "state_estimator_min_samples",
                estimator_window_size,
            )
        ),
        table_height=float(planner_config.get("table_height", 0.76)),
        table_center_xy=planner_config.get(
            "table_center_xy_w",
            [1.37, 0.0],
        ),
        table_length=float(planner_config.get("table_length", 2.74)),
        table_width=float(planner_config.get("table_width", 1.525)),
        ball_radius=float(planner_config.get("ball_radius", 0.02)),
        bounce_height_tolerance=float(
            planner_config.get(
                "state_estimator_bounce_height_tolerance",
                0.03,
            )
        ),
        bounce_velocity_threshold=float(
            planner_config.get(
                "state_estimator_bounce_velocity_threshold",
                0.10,
            )
        ),
        bounce_min_separation_s=float(
            planner_config.get(
                "state_estimator_bounce_min_separation_s",
                0.20,
            )
        ),
    )


def build_hitter_system_planner(
    planner_config: Mapping[str, object],
) -> HitterSystemPlanner:
    """Build the exact planner currently used by HitterEnv."""
    table_center_xy = planner_config.get(
        "table_center_xy_w",
        [1.37, 0.0],
    )
    predictor = BallTrajectoryPredictor(
        gravity=planner_config.get("gravity", [0.0, 0.0, -9.81]),
        drag_coefficient=float(
            planner_config.get("drag_coefficient", 0.0)
        ),
        vertical_restitution=float(
            planner_config.get("vertical_restitution", 0.8)
        ),
        horizontal_restitution=float(
            planner_config.get("horizontal_restitution", 0.9)
        ),
        dt=float(planner_config.get("prediction_dt", 0.005)),
        table_height=float(planner_config.get("table_height", 0.76)),
        table_center_xy=table_center_xy,
        table_length=float(planner_config.get("table_length", 2.74)),
        table_width=float(planner_config.get("table_width", 1.525)),
        ball_radius=float(planner_config.get("ball_radius", 0.02)),
    )
    table_height = float(planner_config.get("table_height", 0.76))
    maximum_hit_height = round(
        table_height
        + float(
            planner_config.get(
                "maximum_hit_height_above_table_m",
                0.69,
            )
        ),
        12,
    )
    strike_planner = StrikePlanner(
        predictor=predictor,
        virtual_hit_plane_x=float(
            planner_config.get("virtual_hit_plane_x", 0.0)
        ),
        desired_landing_point=planner_config.get(
            "desired_landing_point_w",
            [2.05, 0.0, 0.78],
        ),
        post_hit_flight_time=float(
            planner_config.get("post_hit_flight_time", 0.55)
        ),
        racket_restitution=float(
            planner_config.get("racket_restitution", 0.85)
        ),
        prediction_horizon_s=float(
            planner_config.get("prediction_horizon_s", 2.0)
        ),
        maximum_prediction_horizon_s=float(
            planner_config.get("maximum_prediction_horizon_s", 5.0)
        ),
        minimum_hit_height=table_height,
        maximum_hit_height=maximum_hit_height,
        require_future_hit_plane_crossing=bool(
            planner_config.get(
                "require_future_hit_plane_crossing",
                False,
            )
        ),
        racket_velocity_component_ranges_mps=planner_config.get(
            "racket_velocity_component_ranges_mps",
            None,
        ),
    )
    base_planner = BaseTargetPlanner(
        racket_x_offset_b=float(
            planner_config.get("racket_x_offset_b", 0.40)
        ),
        forehand_nominal_racket_y_b=float(
            planner_config.get(
                "forehand_nominal_racket_y_b",
                -0.5,
            )
        ),
        backhand_nominal_racket_y_b=float(
            planner_config.get(
                "backhand_nominal_racket_y_b",
                0.22,
            )
        ),
        default_base_z=float(
            planner_config.get("target_base_height_w", 0.793)
        ),
    )
    return HitterSystemPlanner(
        strike_planner=strike_planner,
        base_planner=base_planner,
    )


def forced_strike_type(
    planner_config: Mapping[str, object],
    motion_config: Mapping[str, object],
) -> str | None:
    """Resolve and validate forehand/backhand/None exactly once."""
    forced = planner_config.get(
        "force_strike_type",
        motion_config.get("force_strike_type", None),
    )
    if (
        forced is None
        or str(forced).strip().lower() in {"", "none", "null"}
    ):
        return None
    forced_name = str(forced).strip().lower()
    if forced_name not in {"forehand", "backhand"}:
        raise ValueError(
            "force_strike_type must be forehand/backhand/None, "
            f"got {forced_name!r}."
        )
    return forced_name


@dataclass(frozen=True)
class HitterRuntimeSettings:
    estimator_sample_rate_hz: float
    planner_update_rate_hz: float
    planner_update_interval_s: float
    minimum_incoming_speed_x_mps: float
    incoming_confirmation_snapshots: int
    waiting_tts_s: float
    arm_tts_s: float
    minimum_arm_tts_s: float
    maximum_policy_tts_s: float
    swing_duration_range_s: tuple[float, float]
    hitter_seed: int | None
    control_tick_s: float
    obs_clip_value: float | None


def _range_from_value(
    name: str,
    value: object,
) -> tuple[float, float]:
    if len(value) != 2:
        raise ValueError(
            f"HITTER motion range `{name}` must contain two values, "
            f"got {value!r}."
        )
    low, high = float(value[0]), float(value[1])
    if high < low:
        raise ValueError(
            f"HITTER motion range `{name}` has high < low: {value!r}."
        )
    return low, high


def resolve_hitter_runtime_settings(
    *,
    policy_config: Mapping[str, object],
    motion_config: Mapping[str, object],
    control_config: Mapping[str, object],
) -> HitterRuntimeSettings:
    """Resolve every estimator/incoming/lifecycle/tick value once."""
    planner_config = motion_config.get("ball_planner", {}) or {}
    planner_update_rate_hz = float(
        planner_config.get(
            "planner_update_rate_hz",
            motion_config.get("planner_update_rate_hz", 100.0),
        )
    )
    if (
        not np.isfinite(planner_update_rate_hz)
        or planner_update_rate_hz <= 0.0
    ):
        raise ValueError(
            "HITTER planner_update_rate_hz must be finite and positive."
        )

    swing_duration_range_s = _range_from_value(
        "swing_duration_range",
        planner_config.get(
            "swing_duration_range",
            motion_config.get(
                "swing_duration_range",
                (1.75, 1.95),
            ),
        ),
    )
    hitter_seed_value = policy_config.get("hitter_seed", None)
    obs_clip_value = control_config.get("obs_clip_value", None)

    return HitterRuntimeSettings(
        estimator_sample_rate_hz=float(
            planner_config.get(
                "state_estimator_sample_rate_hz",
                300.0,
            )
        ),
        planner_update_rate_hz=planner_update_rate_hz,
        planner_update_interval_s=1.0 / planner_update_rate_hz,
        minimum_incoming_speed_x_mps=float(
            planner_config.get(
                "minimum_stable_incoming_speed_x_mps",
                0.20,
            )
        ),
        incoming_confirmation_snapshots=int(
            planner_config.get(
                "stable_incoming_confirmation_snapshots",
                3,
            )
        ),
        waiting_tts_s=float(
            motion_config.get("waiting_time_to_strike_s", 0.92)
        ),
        arm_tts_s=float(
            planner_config.get("arm_time_to_strike_s", 0.90)
        ),
        minimum_arm_tts_s=float(
            planner_config.get(
                "minimum_arm_time_to_strike_s",
                0.80,
            )
        ),
        maximum_policy_tts_s=float(
            planner_config.get(
                "maximum_policy_time_to_strike_s",
                0.92,
            )
        ),
        swing_duration_range_s=swing_duration_range_s,
        hitter_seed=(
            None
            if hitter_seed_value is None
            else int(hitter_seed_value)
        ),
        control_tick_s=(
            float(control_config.get("low_dt", 0.005))
            * float(control_config.get("decimation", 4))
        ),
        obs_clip_value=(
            None
            if obs_clip_value is None
            else float(obs_clip_value)
        ),
    )


def build_hitter_command_lifecycle(
    settings: HitterRuntimeSettings,
    *,
    rng: np.random.Generator,
) -> HitterCommandLifecycle:
    """Use the seeded production-uniform swing-duration sampler."""
    low, high = settings.swing_duration_range_s

    def sample_swing_duration() -> float:
        return float(rng.uniform(low, high))

    return HitterCommandLifecycle(
        waiting_tts=settings.waiting_tts_s,
        arm_tts=settings.arm_tts_s,
        minimum_arm_tts=settings.minimum_arm_tts_s,
        maximum_policy_tts=settings.maximum_policy_tts_s,
        swing_duration_sampler=sample_swing_duration,
    )


__all__ = [
    "HitterRuntimeSettings",
    "build_ball_state_estimator",
    "build_hitter_command_lifecycle",
    "build_hitter_system_planner",
    "forced_strike_type",
    "resolve_hitter_runtime_settings",
]
