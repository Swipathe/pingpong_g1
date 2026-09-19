from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from envs.hitter import HitterEnv
from utils.hitter_planner import (
    BallTrajectory,
    BallTrajectoryPredictor,
    BaseTargetPlanner,
    HitterSystemPlanner,
    StrikePlan,
    StrikePlanner,
)
from utils.hitter_realtime import BallEstimateSnapshot, IncomingTrackConfirmation
from utils.hitter_runtime_types import PlannerFailureReason, PlannerRejected


class FixedStrikePlanner:
    def __init__(self, point, *, t_strike=0.5, racket_velocity=None):
        self.point = np.asarray(point, dtype=np.float64)
        self.t_strike = t_strike
        self.racket_velocity = (
            np.zeros(3, dtype=np.float64) if racket_velocity is None else np.asarray(racket_velocity, dtype=np.float64)
        )

    def plan(self, _ball_position, _ball_velocity) -> StrikePlan:
        return StrikePlan(
            t_strike=self.t_strike,
            p_racket_target=self.point.copy(),
            v_racket_target=self.racket_velocity.copy(),
            v_ball_in=np.array([-1.0, 0.0, 0.0], dtype=np.float64),
            v_ball_out=np.array([1.0, 0.0, 0.0], dtype=np.float64),
        )


def system_planner_with_fixed_strike_point(point) -> HitterSystemPlanner:
    return HitterSystemPlanner(
        strike_planner=FixedStrikePlanner(point),
        base_planner=BaseTargetPlanner(),
    )


@pytest.fixture
def strike_planner() -> StrikePlanner:
    return StrikePlanner(
        predictor=BallTrajectoryPredictor(table_height=0.76, dt=0.005),
        virtual_hit_plane_x=0.0,
        prediction_horizon_s=2.0,
        maximum_prediction_horizon_s=5.0,
        minimum_hit_height=0.76,
        maximum_hit_height=1.45,
        require_future_hit_plane_crossing=True,
    )


@pytest.mark.parametrize(
    "table_y,expected",
    [(-1.0e-9, "forehand"), (0.0, "backhand"), (1.0e-9, "backhand")],
)
def test_strike_type_uses_absolute_table_y(table_y, expected):
    planner = system_planner_with_fixed_strike_point([0.0, table_y, 1.0])
    for base_y, yaw in [(-0.8, -1.2), (0.0, 0.0), (0.9, 2.1)]:
        forward = [np.cos(yaw), np.sin(yaw)]
        command = planner.plan_command(
            [0.8, 0.0, 1.0],
            [-2.0, 0.0, 0.0],
            current_base_xy_w=[-0.4, base_y],
            base_forward_xy_w=forward,
        )
        assert command.strike_type == expected
        assert command.strike_table_y_w == pytest.approx(table_y)
        assert command.strike_side_source == "table_y"


def test_forced_strike_type_is_explicitly_recorded():
    planner = system_planner_with_fixed_strike_point([0.0, -0.2, 1.0])
    command = planner.plan_command(
        [0.8, 0.0, 1.0],
        [-2.0, 0.0, 0.0],
        strike_type="backhand",
    )
    assert command.strike_type == "backhand"
    assert command.strike_table_y_w == pytest.approx(-0.2)
    assert command.strike_side_source == "forced"


def test_no_crossing_has_typed_reason(strike_planner):
    with pytest.raises(PlannerRejected) as caught:
        strike_planner.hit_plane_intersection(
            [0.8, 0.0, 1.0],
            [-0.01, 0.0, 4.0],
        )
    assert caught.value.reason is PlannerFailureReason.NO_FUTURE_CROSSING


def test_outgoing_ball_has_typed_reason(strike_planner):
    with pytest.raises(PlannerRejected) as caught:
        strike_planner.plan([0.8, 0.0, 1.0], [0.1, 0.0, 0.0])
    assert caught.value.reason is PlannerFailureReason.BALL_NOT_INCOMING


def test_hit_height_has_typed_reason():
    planner = StrikePlanner(
        predictor=BallTrajectoryPredictor(
            gravity=[0.0, 0.0, 0.0],
            table_height=0.76,
            dt=0.005,
        ),
        virtual_hit_plane_x=0.0,
        minimum_hit_height=0.76,
        maximum_hit_height=1.45,
    )
    with pytest.raises(PlannerRejected) as caught:
        planner.plan([0.4, 0.0, 1.46], [-1.0, 0.0, 0.0])
    assert caught.value.reason is PlannerFailureReason.HIT_HEIGHT_OUT_OF_RANGE


class NonfiniteTrajectoryPredictor(BallTrajectoryPredictor):
    def predict(self, _position, _velocity, horizon_s):
        return BallTrajectory(
            times=np.array([0.0, float(horizon_s)]),
            positions=np.array([[0.1, 0.0, 1.0], [np.nan, 0.0, 1.0]]),
            velocities=np.array([[-1.0, 0.0, 0.0], [-1.0, 0.0, 0.0]]),
        )


@pytest.mark.parametrize("source", ["input", "trajectory"])
def test_nonfinite_data_has_typed_reason(source, strike_planner):
    planner = strike_planner
    position = [np.nan, 0.0, 1.0]
    if source == "trajectory":
        planner = StrikePlanner(
            predictor=NonfiniteTrajectoryPredictor(),
            virtual_hit_plane_x=0.0,
            minimum_hit_height=0.76,
            maximum_hit_height=1.45,
        )
        position = [0.1, 0.0, 1.0]
    with pytest.raises(PlannerRejected) as caught:
        planner.plan(position, [-1.0, 0.0, 0.0])
    assert caught.value.reason is PlannerFailureReason.NONFINITE_INPUT_OR_OUTPUT


@pytest.mark.parametrize(
    "strike_planner",
    [
        FixedStrikePlanner([0.0, 0.0, 1.0], t_strike=np.nan),
        FixedStrikePlanner(
            [0.0, 0.0, 1.0],
            racket_velocity=[np.inf, 0.0, 0.0],
        ),
    ],
)
def test_nonfinite_strike_plan_output_has_typed_reason(strike_planner):
    planner = HitterSystemPlanner(
        strike_planner=strike_planner,
        base_planner=BaseTargetPlanner(),
    )
    with pytest.raises(PlannerRejected) as caught:
        planner.plan_command([0.8, 0.0, 1.0], [-2.0, 0.0, 0.0])
    assert caught.value.reason is PlannerFailureReason.NONFINITE_INPUT_OR_OUTPUT


def snapshot(**changes) -> BallEstimateSnapshot:
    values = {
        "track_id": 7,
        "generation": 1,
        "source_frame": 1,
        "source_time_s": 1.0,
        "received_monotonic_s": 2.0,
        "position_w": np.array([0.8, 0.0, 1.0]),
        "velocity_w": np.array([-2.0, 0.0, 0.0]),
        "base_position_w": np.array([-0.4, 0.0, 0.8]),
        "base_quaternion_xyzw": np.array([0.0, 0.0, 0.0, 1.0]),
        "base_valid": True,
        "visible": True,
        "ready": True,
    }
    values.update(changes)
    return BallEstimateSnapshot(**values)


def real_world_planning_env() -> HitterEnv:
    env = HitterEnv.__new__(HitterEnv)
    env.simulator = SimpleNamespace(is_real=True)
    env.hitter_incoming_track_confirmation = IncomingTrackConfirmation(
        minimum_speed_x_mps=0.2,
        required_consecutive_snapshots=2,
    )
    env.hitter_ball_planner = system_planner_with_fixed_strike_point([0.0, 0.0, 1.0])
    env.hitter_forced_strike_type = None
    return env


@pytest.mark.parametrize(
    "changes,reason",
    [
        ({"visible": False}, PlannerFailureReason.TRACK_ENDED),
        ({"ready": False}, PlannerFailureReason.ESTIMATOR_NOT_READY),
        ({"base_valid": False}, PlannerFailureReason.BASE_POSE_INVALID),
        ({}, PlannerFailureReason.BALL_NOT_INCOMING),
    ],
)
def test_snapshot_admission_has_typed_reason(changes, reason):
    env = real_world_planning_env()
    with pytest.raises(PlannerRejected) as caught:
        env._plan_hitter_snapshot(snapshot(**changes))
    assert caught.value.reason is reason


def test_internal_error_reason_is_reserved_for_worker_boundary():
    assert set(PlannerFailureReason) == {
        PlannerFailureReason.TRACK_ENDED,
        PlannerFailureReason.ESTIMATOR_NOT_READY,
        PlannerFailureReason.BASE_POSE_INVALID,
        PlannerFailureReason.BALL_NOT_INCOMING,
        PlannerFailureReason.NO_FUTURE_CROSSING,
        PlannerFailureReason.HIT_HEIGHT_OUT_OF_RANGE,
        PlannerFailureReason.NONFINITE_INPUT_OR_OUTPUT,
        PlannerFailureReason.INTERNAL_ERROR,
    }
