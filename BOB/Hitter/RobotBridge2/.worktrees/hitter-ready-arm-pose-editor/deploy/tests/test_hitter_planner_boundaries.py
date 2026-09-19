import unittest

import numpy as np

from utils.hitter_planner import BallTrajectory, BallTrajectoryPredictor, StrikePlanner


class StrikePlannerBoundaryTest(unittest.TestCase):
    def setUp(self):
        predictor = BallTrajectoryPredictor(table_height=0.76, dt=0.005)
        self.planner = StrikePlanner(
            predictor=predictor,
            virtual_hit_plane_x=0.0,
            prediction_horizon_s=2.0,
            maximum_prediction_horizon_s=5.0,
            minimum_hit_height=0.76,
            maximum_hit_height=1.26,
            require_future_hit_plane_crossing=True,
        )

    @staticmethod
    def make_level_planner(
        *,
        require_crossing=True,
        prediction_horizon_s=2.0,
        maximum_prediction_horizon_s=5.0,
    ):
        predictor = BallTrajectoryPredictor(
            gravity=[0.0, 0.0, 0.0],
            table_height=0.76,
            dt=0.005,
        )
        return StrikePlanner(
            predictor=predictor,
            virtual_hit_plane_x=0.0,
            prediction_horizon_s=prediction_horizon_s,
            maximum_prediction_horizon_s=maximum_prediction_horizon_s,
            minimum_hit_height=0.76,
            maximum_hit_height=1.26,
            require_future_hit_plane_crossing=require_crossing,
        )

    def test_rejects_outgoing_ball(self):
        with self.assertRaisesRegex(ValueError, "vx < 0"):
            self.planner.plan([0.4, 0.0, 0.9], [1.0, 0.0, 0.0])

    def test_rejects_ball_already_past_hit_plane(self):
        with self.assertRaisesRegex(ValueError, "x > 0"):
            self.planner.plan([-0.01, 0.0, 0.9], [-1.0, 0.0, 0.0])

    def test_rejects_ball_exactly_on_hit_plane(self):
        with self.assertRaisesRegex(ValueError, "x > 0"):
            self.planner.plan([0.0, 0.0, 0.9], [-1.0, 0.0, 0.0])

    def test_rejects_zero_incoming_velocity(self):
        with self.assertRaisesRegex(ValueError, "vx < 0"):
            self.planner.plan([0.4, 0.0, 0.9], [0.0, 0.0, 0.0])

    def test_rejects_crossing_above_fifty_centimetres(self):
        with self.assertRaisesRegex(ValueError, "hit height"):
            self.make_level_planner().plan([0.4, 0.0, 1.40], [-1.0, 0.0, 0.0])

    def test_rejects_crossing_at_table_height(self):
        with self.assertRaisesRegex(ValueError, "hit height"):
            self.make_level_planner().plan([0.4, 0.0, 0.76], [-1.0, 0.0, 0.0])

    def test_accepts_crossing_at_maximum_hit_height(self):
        plan = self.make_level_planner().plan([0.4, 0.0, 1.26], [-1.0, 0.0, 0.0])
        self.assertAlmostEqual(plan.p_racket_target[2], 1.26)

    def test_accepts_positive_to_negative_crossing(self):
        plan = self.planner.plan([0.4, 0.0, 0.95], [-1.5, 0.0, 0.0])
        self.assertGreater(plan.t_strike, 0.0)
        self.assertAlmostEqual(plan.p_racket_target[0], 0.0)
        self.assertLess(plan.v_ball_in[0], 0.0)

    def test_two_seconds_is_a_chunk_not_a_business_gate(self):
        predictor = BallTrajectoryPredictor(
            table_height=0.76,
            dt=0.005,
            horizontal_restitution=1.0,
        )
        planner = StrikePlanner(
            predictor=predictor,
            virtual_hit_plane_x=0.0,
            prediction_horizon_s=2.0,
            maximum_prediction_horizon_s=5.0,
            minimum_hit_height=0.76,
            maximum_hit_height=1.26,
            require_future_hit_plane_crossing=True,
        )

        plan = planner.plan([2.4, 0.0, 0.95], [-0.8, 0.0, 0.0])
        self.assertGreater(plan.t_strike, 2.0)
        self.assertLessEqual(plan.t_strike, 5.0)

    def test_rejects_when_no_directed_crossing_exists_by_maximum_horizon(self):
        planner = self.make_level_planner(
            require_crossing=False,
            prediction_horizon_s=0.1,
            maximum_prediction_horizon_s=0.2,
        )

        with self.assertRaisesRegex(ValueError, "directed crossing"):
            planner.plan([1.0, 0.0, 0.9], [-1.0, 0.0, 0.0])

    def test_rejects_geometric_crossing_with_non_incoming_predicted_velocity(self):
        class ReverseVelocityPredictor(BallTrajectoryPredictor):
            def predict(self, position, velocity, horizon_s):
                return BallTrajectory(
                    times=np.asarray([0.0, float(horizon_s)]),
                    positions=np.asarray([[0.1, 0.0, 0.9], [-0.1, 0.0, 0.9]]),
                    velocities=np.asarray([[1.0, 0.0, 0.0], [1.0, 0.0, 0.0]]),
                )

        planner = StrikePlanner(
            predictor=ReverseVelocityPredictor(gravity=[0.0, 0.0, 0.0]),
            virtual_hit_plane_x=0.0,
            prediction_horizon_s=0.1,
            maximum_prediction_horizon_s=0.2,
            minimum_hit_height=0.76,
            maximum_hit_height=1.26,
            require_future_hit_plane_crossing=True,
        )

        with self.assertRaisesRegex(ValueError, "directed crossing"):
            planner.plan([0.1, 0.0, 0.9], [-1.0, 0.0, 0.0])

    def test_prediction_horizon_grows_in_chunks_up_to_finite_maximum(self):
        class RecordingPredictor(BallTrajectoryPredictor):
            def __init__(self):
                super().__init__(gravity=[0.0, 0.0, 0.0])
                self.horizons = []

            def predict(self, position, velocity, horizon_s):
                self.horizons.append(float(horizon_s))
                end_x = -0.1 if horizon_s >= 5.0 else 0.1
                return BallTrajectory(
                    times=np.asarray([0.0, float(horizon_s)]),
                    positions=np.asarray([[0.4, 0.0, 0.9], [end_x, 0.0, 0.9]]),
                    velocities=np.asarray([[-1.0, 0.0, 0.0], [-1.0, 0.0, 0.0]]),
                )

        predictor = RecordingPredictor()
        planner = StrikePlanner(
            predictor=predictor,
            virtual_hit_plane_x=0.0,
            prediction_horizon_s=2.0,
            maximum_prediction_horizon_s=5.0,
            minimum_hit_height=0.76,
            maximum_hit_height=1.26,
            require_future_hit_plane_crossing=True,
        )

        plan = planner.plan([0.4, 0.0, 0.9], [-1.0, 0.0, 0.0])

        self.assertEqual(predictor.horizons, [2.0, 4.0, 5.0])
        self.assertGreater(plan.t_strike, 2.0)
        self.assertLessEqual(plan.t_strike, 5.0)

    def test_predictor_step_cannot_cross_past_maximum_horizon(self):
        predictor = BallTrajectoryPredictor(
            gravity=[0.0, 0.0, 0.0],
            dt=0.1,
        )
        planner = StrikePlanner(
            predictor=predictor,
            virtual_hit_plane_x=0.0,
            prediction_horizon_s=0.2,
            maximum_prediction_horizon_s=0.25,
            minimum_hit_height=0.76,
            maximum_hit_height=1.26,
            require_future_hit_plane_crossing=True,
        )

        with self.assertRaisesRegex(ValueError, "directed crossing"):
            planner.plan([0.26, 0.0, 0.9], [-1.0, 0.0, 0.0])

    def test_rejects_non_finite_ball_state(self):
        with self.assertRaisesRegex(ValueError, "non-finite"):
            self.planner.plan([np.nan, 0.0, 0.9], [-1.0, 0.0, 0.0])
        with self.assertRaisesRegex(ValueError, "non-finite"):
            self.planner.plan([0.4, 0.0, 0.9], [-1.0, np.inf, 0.0])

    def test_rejects_non_finite_or_reversed_prediction_horizons(self):
        with self.assertRaisesRegex(ValueError, "finite"):
            StrikePlanner(maximum_prediction_horizon_s=np.inf)
        with self.assertRaisesRegex(ValueError, "maximum_prediction_horizon_s"):
            StrikePlanner(
                prediction_horizon_s=2.0,
                maximum_prediction_horizon_s=1.0,
            )


if __name__ == "__main__":
    unittest.main()
