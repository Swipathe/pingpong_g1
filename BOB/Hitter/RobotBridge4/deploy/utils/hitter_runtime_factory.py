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
    target_base_height_w = float(
        planner_config.get("target_base_height_w", 0.793)
    )
    racket_target_z_range_b_m = planner_config.get(
        "racket_target_z_range_b_m",
        None,
    )
    if racket_target_z_range_b_m is None:
        minimum_hit_height = table_height
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
    else:
        minimum_z_b, maximum_z_b = racket_target_z_range_b_m
        minimum_hit_height = round(
            target_base_height_w + float(minimum_z_b),
            12,
        )
        maximum_hit_height = round(
            target_base_height_w + float(maximum_z_b),
            12,
        )
    desired_landing_point = planner_config.get(
        "desired_landing_point_w",
        [2.05, 0.0, 0.78],
    )
    strike_planner = StrikePlanner(
        predictor=predictor,
        virtual_hit_plane_x=float(
            planner_config.get("virtual_hit_plane_x", 0.0)
        ),
        desired_landing_point=desired_landing_point,
        forehand_desired_landing_point=planner_config.get(
            "forehand_desired_landing_point_w",
            desired_landing_point,
        ),
        backhand_desired_landing_point=planner_config.get(
            "backhand_desired_landing_point_w",
            desired_landing_point,
        ),
        backhand_edge_landing_start_y_w_m=planner_config.get(
            "backhand_edge_landing_start_y_w_m",
            0.30,
        ),
        backhand_edge_landing_full_y_w_m=planner_config.get(
            "backhand_edge_landing_full_y_w_m",
            0.50,
        ),
        backhand_edge_landing_y_decrement_m=planner_config.get(
            "backhand_edge_landing_y_decrement_m",
            0.0,
        ),
        backhand_edge_landing_threshold_y_w_m=planner_config.get(
            "backhand_edge_landing_threshold_y_w_m",
            0.20,
        ),
        backhand_edge_landing_target_y_w_m=planner_config.get(
            "backhand_edge_landing_target_y_w_m",
            None,
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
        minimum_hit_height=minimum_hit_height,
        maximum_hit_height=maximum_hit_height,
        require_future_hit_plane_crossing=bool(
            planner_config.get(
                "require_future_hit_plane_crossing",
                False,
            )
        ),
        minimum_racket_normal_speed_mps=planner_config.get(
            "minimum_racket_normal_speed_mps",
            0.0,
        ),
        racket_velocity_component_ranges_mps=planner_config.get(
            "racket_velocity_component_ranges_mps",
            None,
        ),
        backhand_racket_velocity_component_ranges_mps=planner_config.get(
            "backhand_racket_velocity_component_ranges_mps",
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
        default_base_z=target_base_height_w,
    )
    return HitterSystemPlanner(
        strike_planner=strike_planner,
        base_planner=base_planner,
    )


def forced_strike_type(
    planner_config: Mapping[str, object],
    motion_config: Mapping[str, object],
    *,
    is_real_world: bool = False,
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
    if is_real_world:
        raise ValueError(
            "force_strike_type is disabled for real-world HITTER runtime."
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
    completed_result_queue_capacity: int
    armed_cancel_consecutive_failures: int
    commit_time_to_strike_s: float
    maximum_racket_target_override_delta_m: float
    maximum_racket_velocity_override_delta_mps: float
    maximum_strike_deadline_override_delta_s: float
    swing_duration_range_s: tuple[float, float]
    hitter_seed: int | None
    control_tick_s: float
    obs_clip_value: float | None


@dataclass(frozen=True)
class ViconConsumerSettings:
    channel: str
    base_subject: str
    stream_timeout_s: float
    ball_timeout_s: float
    new_serve_no_ball_s: float
    event_queue_capacity: int


def resolve_vicon_consumer_settings(
    config: Mapping[str, object],
) -> ViconConsumerSettings:
    """Validate the fail-closed authoritative v2 consumer boundary."""
    if not isinstance(config, Mapping):
        raise TypeError("HITTER Vicon config must be a mapping.")

    channel = config.get("channel", "vicon_state_data_v2")
    if type(channel) is not str or channel != "vicon_state_data_v2":
        raise ValueError(
            "HITTER Vicon channel must be exactly vicon_state_data_v2."
        )

    base_subject = config.get("base_subject", "G2Pelvis")
    if (
        type(base_subject) is not str
        or not base_subject
        or base_subject != base_subject.strip()
    ):
        raise ValueError(
            "HITTER Vicon base_subject must be a non-empty exact string."
        )
    if base_subject.casefold() in {"ball", "table"}:
        raise ValueError(
            "HITTER Vicon base_subject uses a reserved subject name."
        )

    event_queue_capacity = config.get("event_queue_capacity", 64)
    if type(event_queue_capacity) is not int or event_queue_capacity <= 0:
        raise ValueError(
            "HITTER Vicon event_queue_capacity must be a positive integer."
        )
    if event_queue_capacity > 64:
        raise ValueError(
            "HITTER Vicon event_queue_capacity must be at most 64."
        )

    values = {
        name: _finite_float(
            f"Vicon {name}",
            config.get(name, default),
            positive=True,
        )
        for name, default in (
            ("stream_timeout_s", 0.40),
            ("ball_timeout_s", 0.40),
            ("new_serve_no_ball_s", 0.50),
        )
    }

    return ViconConsumerSettings(
        channel=channel,
        base_subject=base_subject,
        event_queue_capacity=event_queue_capacity,
        **values,
    )


def _range_from_value(
    name: str,
    value: object,
) -> tuple[float, float]:
    if isinstance(value, (str, bytes, Mapping)):
        raise TypeError(
            f"HITTER motion range `{name}` must be a two-value sequence."
        )
    try:
        size = len(value)
    except TypeError as exc:
        raise TypeError(
            f"HITTER motion range `{name}` must be a two-value sequence."
        ) from exc
    if size != 2:
        raise ValueError(
            f"HITTER motion range `{name}` must contain two values, "
            f"got {value!r}."
        )
    low = _finite_float(f"{name}[0]", value[0], positive=False)
    high = _finite_float(f"{name}[1]", value[1], positive=False)
    if high < low:
        raise ValueError(
            f"HITTER motion range `{name}` has high < low: {value!r}."
        )
    return low, high


def _finite_float(
    name: str,
    value: object,
    *,
    positive: bool,
) -> float:
    if isinstance(value, (bool, np.bool_)) or not isinstance(
        value,
        (int, float, np.integer, np.floating),
    ):
        raise TypeError(f"HITTER {name} must be a real number.")
    result = float(value)
    invalid_bound = result <= 0.0 if positive else result < 0.0
    if not np.isfinite(result) or invalid_bound:
        constraint = "positive" if positive else "non-negative"
        raise ValueError(
            f"HITTER {name} must be finite and {constraint}."
        )
    return result


def _positive_int(name: str, value: object) -> int:
    if type(value) is not int:
        raise TypeError(f"HITTER {name} must be an exact integer.")
    if value <= 0:
        raise ValueError(f"HITTER {name} must be positive.")
    return value


def resolve_hitter_runtime_settings(
    *,
    policy_config: Mapping[str, object],
    motion_config: Mapping[str, object],
    control_config: Mapping[str, object],
) -> HitterRuntimeSettings:
    """Resolve every estimator/incoming/lifecycle/tick value once."""
    for name, config in (
        ("policy_config", policy_config),
        ("motion_config", motion_config),
        ("control_config", control_config),
    ):
        if not isinstance(config, Mapping):
            raise TypeError(f"HITTER {name} must be a mapping.")
    planner_config = motion_config.get("ball_planner", {})
    if not isinstance(planner_config, Mapping):
        raise TypeError("HITTER ball_planner must be a mapping.")
    planner_update_rate_hz = _finite_float(
        "planner_update_rate_hz",
        planner_config.get(
            "planner_update_rate_hz",
            motion_config.get("planner_update_rate_hz", 50.0),
        ),
        positive=True,
    )
    planner_update_interval_s = 1.0 / planner_update_rate_hz
    if not np.isfinite(planner_update_interval_s):
        raise ValueError(
            "HITTER planner_update_interval_s must be finite and positive."
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
    if hitter_seed_value is not None and type(hitter_seed_value) is not int:
        raise TypeError("HITTER hitter_seed must be an exact integer or None.")
    if hitter_seed_value is not None and hitter_seed_value < 0:
        raise ValueError("HITTER hitter_seed must be non-negative.")

    estimator_sample_rate_hz = _finite_float(
        "state_estimator_sample_rate_hz",
        planner_config.get("state_estimator_sample_rate_hz", 300.0),
        positive=True,
    )
    minimum_incoming_speed_x_mps = _finite_float(
        "minimum_stable_incoming_speed_x_mps",
        planner_config.get("minimum_stable_incoming_speed_x_mps", 0.20),
        positive=True,
    )
    incoming_confirmation_snapshots = _positive_int(
        "stable_incoming_confirmation_snapshots",
        planner_config.get("stable_incoming_confirmation_snapshots", 3),
    )

    waiting_tts_s = _finite_float(
        "waiting_time_to_strike_s",
        motion_config.get("waiting_time_to_strike_s", 0.92),
        positive=False,
    )
    arm_tts_s = _finite_float(
        "arm_time_to_strike_s",
        planner_config.get("arm_time_to_strike_s", 0.92),
        positive=False,
    )
    minimum_arm_tts_s = _finite_float(
        "minimum_arm_time_to_strike_s",
        planner_config.get("minimum_arm_time_to_strike_s", 0.30),
        positive=False,
    )
    maximum_policy_tts_s = _finite_float(
        "maximum_policy_time_to_strike_s",
        planner_config.get("maximum_policy_time_to_strike_s", 0.92),
        positive=False,
    )
    commit_time_to_strike_s = _finite_float(
        "commit_time_to_strike_s",
        planner_config.get("commit_time_to_strike_s", 0.30),
        positive=False,
    )
    if not (
        commit_time_to_strike_s
        <= minimum_arm_tts_s
        <= arm_tts_s
        <= maximum_policy_tts_s
    ):
        raise ValueError(
            "HITTER lifecycle times must satisfy commit <= minimum_arm "
            "<= arm <= maximum_policy."
        )
    if waiting_tts_s > maximum_policy_tts_s:
        raise ValueError(
            "HITTER waiting_time_to_strike_s must not exceed "
            "maximum_policy_time_to_strike_s."
        )

    completed_result_queue_capacity = _positive_int(
        "completed_result_queue_capacity",
        planner_config.get("completed_result_queue_capacity", 64),
    )
    armed_cancel_consecutive_failures = _positive_int(
        "armed_cancel_consecutive_failures",
        planner_config.get("armed_cancel_consecutive_failures", 3),
    )
    maximum_racket_target_override_delta_m = _finite_float(
        "maximum_racket_target_override_delta_m",
        planner_config.get(
            "maximum_racket_target_override_delta_m",
            0.05,
        ),
        positive=False,
    )
    maximum_racket_velocity_override_delta_mps = _finite_float(
        "maximum_racket_velocity_override_delta_mps",
        planner_config.get(
            "maximum_racket_velocity_override_delta_mps",
            0.75,
        ),
        positive=False,
    )
    maximum_strike_deadline_override_delta_s = _finite_float(
        "maximum_strike_deadline_override_delta_s",
        planner_config.get(
            "maximum_strike_deadline_override_delta_s",
            0.05,
        ),
        positive=False,
    )

    low_dt = _finite_float(
        "low_dt",
        control_config.get("low_dt", 0.005),
        positive=True,
    )
    decimation = _positive_int(
        "decimation",
        control_config.get("decimation", 4),
    )
    control_tick_s = low_dt * decimation
    if not np.isfinite(control_tick_s) or control_tick_s <= 0.0:
        raise ValueError("HITTER control_tick_s must be finite and positive.")

    obs_clip_value = control_config.get("obs_clip_value", None)
    resolved_obs_clip_value = (
        None
        if obs_clip_value is None
        else _finite_float(
            "obs_clip_value",
            obs_clip_value,
            positive=True,
        )
    )

    return HitterRuntimeSettings(
        estimator_sample_rate_hz=estimator_sample_rate_hz,
        planner_update_rate_hz=planner_update_rate_hz,
        planner_update_interval_s=planner_update_interval_s,
        minimum_incoming_speed_x_mps=minimum_incoming_speed_x_mps,
        incoming_confirmation_snapshots=incoming_confirmation_snapshots,
        waiting_tts_s=waiting_tts_s,
        arm_tts_s=arm_tts_s,
        minimum_arm_tts_s=minimum_arm_tts_s,
        maximum_policy_tts_s=maximum_policy_tts_s,
        completed_result_queue_capacity=completed_result_queue_capacity,
        armed_cancel_consecutive_failures=(
            armed_cancel_consecutive_failures
        ),
        commit_time_to_strike_s=commit_time_to_strike_s,
        maximum_racket_target_override_delta_m=(
            maximum_racket_target_override_delta_m
        ),
        maximum_racket_velocity_override_delta_mps=(
            maximum_racket_velocity_override_delta_mps
        ),
        maximum_strike_deadline_override_delta_s=(
            maximum_strike_deadline_override_delta_s
        ),
        swing_duration_range_s=swing_duration_range_s,
        hitter_seed=hitter_seed_value,
        control_tick_s=control_tick_s,
        obs_clip_value=resolved_obs_clip_value,
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
        armed_cancel_consecutive_failures=(
            settings.armed_cancel_consecutive_failures
        ),
        commit_time_to_strike_s=settings.commit_time_to_strike_s,
        maximum_racket_target_override_delta_m=(
            settings.maximum_racket_target_override_delta_m
        ),
        maximum_racket_velocity_override_delta_mps=(
            settings.maximum_racket_velocity_override_delta_mps
        ),
        maximum_strike_deadline_override_delta_s=(
            settings.maximum_strike_deadline_override_delta_s
        ),
        swing_duration_sampler=sample_swing_duration,
    )


__all__ = [
    "HitterRuntimeSettings",
    "ViconConsumerSettings",
    "build_ball_state_estimator",
    "build_hitter_command_lifecycle",
    "build_hitter_system_planner",
    "forced_strike_type",
    "resolve_hitter_runtime_settings",
    "resolve_vicon_consumer_settings",
]
