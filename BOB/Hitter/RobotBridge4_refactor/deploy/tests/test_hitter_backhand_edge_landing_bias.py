from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from utils.hitter_planner import (
    BallTrajectoryPredictor,
    HitterSystemPlanner,
    StrikePlanner,
)


class SelectedStrikePlanner(StrikePlanner):
    """Planner with a deterministic selected strike point for target tests."""

    def __init__(self, strike_y: float, **kwargs):
        self._strike_y = float(strike_y)
        super().__init__(**kwargs)

    def hit_plane_intersection(self, _ball_position, _ball_velocity):
        return (
            0.25,
            np.array([0.0, self._strike_y, 1.0], dtype=np.float64),
            np.array([-2.0, 0.0, 0.0], dtype=np.float64),
        )


def predictor(*, table_width: float = 1.525) -> BallTrajectoryPredictor:
    return BallTrajectoryPredictor(
        table_height=0.76,
        table_width=table_width,
        drag_coefficient=0.0,
        dt=0.005,
    )


def edge_planner(strike_y: float, **kwargs) -> SelectedStrikePlanner:
    return SelectedStrikePlanner(
        strike_y,
        predictor=predictor(),
        desired_landing_point=(2.05, 0.0, 0.78),
        post_hit_flight_time=0.48,
        backhand_edge_landing_threshold_y_w_m=0.20,
        backhand_edge_landing_target_y_w_m=-0.30,
        **kwargs,
    )


@pytest.mark.parametrize(
    ("strike_y", "expected_landing"),
    [
        (-0.20, [2.05, 0.3775, 0.78]),
        (0.0, [2.05, -0.3775, 0.78]),
        (0.20, [2.05, -0.3775, 0.78]),
    ],
)
def test_full_plan_selects_landing_target_from_resolved_strike_type(
    strike_y,
    expected_landing,
):
    planner = SelectedStrikePlanner(
        strike_y,
        predictor=predictor(),
        desired_landing_point=(2.05, 0.0, 0.78),
        forehand_desired_landing_point=(2.05, 0.3775, 0.78),
        backhand_desired_landing_point=(2.05, -0.3775, 0.78),
        post_hit_flight_time=0.48,
    )

    plan = planner.plan(
        ball_position=[0.8, 0.0, 1.0],
        ball_velocity=[-2.0, 0.0, 0.0],
    )

    endpoint = planner._free_flight_endpoint(
        plan.p_racket_target,
        plan.v_ball_out,
        0.48,
    )
    np.testing.assert_allclose(endpoint, expected_landing, atol=2.0e-5)


@pytest.mark.parametrize(
    ("strike_y", "strike_type", "expected_y"),
    [
        (-0.50, "forehand", 0.0),
        (0.20, "backhand", 0.0),
        (0.200001, "backhand", -0.30),
        (0.70, "backhand", -0.30),
        (0.70, "forehand", 0.0),
    ],
)
def test_effective_landing_y_uses_strict_backhand_reach_threshold(strike_y, strike_type, expected_y):
    planner = edge_planner(strike_y)

    landing = planner.desired_landing_point_for_strike(
        [0.0, strike_y, 1.0],
        strike_type=strike_type,
    )

    np.testing.assert_allclose(landing, [2.05, expected_y, 0.78])


def test_full_plan_recomputes_outgoing_velocity_for_over_threshold_backhand_target():
    planner = edge_planner(0.50)

    plan = planner.plan(
        ball_position=[0.8, 0.0, 1.0],
        ball_velocity=[-2.0, 0.0, 0.0],
        strike_type="backhand",
    )

    endpoint = planner._free_flight_endpoint(
        plan.p_racket_target,
        plan.v_ball_out,
        0.48,
    )
    np.testing.assert_allclose(endpoint, [2.05, -0.30, 0.78], atol=2.0e-5)


def test_disabled_bias_matches_unmodified_planner_output():
    unmodified = SelectedStrikePlanner(
        0.50,
        predictor=predictor(),
        desired_landing_point=(2.05, 0.0, 0.78),
        post_hit_flight_time=0.48,
    )
    disabled = SelectedStrikePlanner(
        0.50,
        predictor=predictor(),
        desired_landing_point=(2.05, 0.0, 0.78),
        post_hit_flight_time=0.48,
        backhand_edge_landing_threshold_y_w_m=0.20,
        backhand_edge_landing_target_y_w_m=None,
    )

    inputs = dict(ball_position=[0.8, 0.0, 1.0], ball_velocity=[-2.0, 0.0, 0.0])
    unmodified_plan = unmodified.plan(**inputs)
    disabled_plan = disabled.plan(**inputs, strike_type="backhand")

    np.testing.assert_allclose(disabled_plan.v_ball_out, unmodified_plan.v_ball_out)


def test_forced_strike_type_is_passed_to_strike_planner_before_velocity_planning():
    planner = HitterSystemPlanner(strike_planner=edge_planner(0.50))

    command = planner.plan_command(
        [0.8, 0.0, 1.0],
        [-2.0, 0.0, 0.0],
        strike_type="forehand",
    )

    endpoint = planner.strike_planner._free_flight_endpoint(
        command.strike_plan.p_racket_target,
        command.strike_plan.v_ball_out,
        0.48,
    )
    np.testing.assert_allclose(endpoint, [2.05, 0.0, 0.78], atol=2.0e-5)


@pytest.mark.parametrize(
    ("parameter", "value"),
    [
        ("backhand_edge_landing_threshold_y_w_m", np.nan),
        ("backhand_edge_landing_threshold_y_w_m", -0.01),
        ("backhand_edge_landing_target_y_w_m", np.inf),
    ],
)
def test_override_parameters_must_be_finite_and_threshold_nonnegative(parameter, value):
    with pytest.raises(ValueError):
        StrikePlanner(**{parameter: value})


def test_override_threshold_must_be_inside_table_half_width():
    with pytest.raises(ValueError):
        StrikePlanner(
            predictor=predictor(table_width=0.80),
            backhand_edge_landing_threshold_y_w_m=0.41,
            backhand_edge_landing_target_y_w_m=-0.30,
        )


def test_override_landing_y_must_remain_inside_table():
    with pytest.raises(ValueError):
        StrikePlanner(
            predictor=predictor(table_width=0.80),
            backhand_edge_landing_threshold_y_w_m=0.20,
            backhand_edge_landing_target_y_w_m=-0.41,
        )


def test_direct_and_smooth_landing_rules_cannot_both_be_enabled():
    with pytest.raises(ValueError):
        StrikePlanner(
            predictor=predictor(),
            backhand_edge_landing_start_y_w_m=0.30,
            backhand_edge_landing_full_y_w_m=0.50,
            backhand_edge_landing_y_decrement_m=0.10,
            backhand_edge_landing_threshold_y_w_m=0.20,
            backhand_edge_landing_target_y_w_m=-0.30,
        )
