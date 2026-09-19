from __future__ import annotations

import unittest

import numpy as np

from utils.hitter_planner import StrikePlanner


class FixedVelocityStrikePlanner(StrikePlanner):
    def hit_plane_intersection(self, ball_position, ball_velocity):
        return (
            0.5,
            np.array([0.0, 0.0, 1.0], dtype=np.float64),
            np.zeros(3, dtype=np.float64),
        )

    def desired_outgoing_ball_velocity(self, strike_position):
        return np.zeros(3, dtype=np.float64)

    def racket_velocity_from_ball_velocities(self, v_ball_in, v_ball_out):
        return np.array([2.0, 3.0, 10.0], dtype=np.float64)


class StrikePlannerRacketVelocityAlignmentTests(unittest.TestCase):
    def test_default_planner_keeps_raw_racket_velocity(self):
        planner = FixedVelocityStrikePlanner()

        plan = planner.plan(
            ball_position=[1.0, 0.0, 1.0],
            ball_velocity=[-1.0, 0.0, 0.0],
        )

        np.testing.assert_allclose(
            plan.v_racket_target,
            np.array([2.0, 3.0, 10.0], dtype=np.float64),
        )

    def test_component_ranges_clip_racket_velocity_before_command_output(self):
        planner = FixedVelocityStrikePlanner(
            racket_velocity_component_ranges_mps={
                "x": [4.0, 8.0],
                "y": [-1.4, 1.4],
                "z": [4.0, 8.0],
            },
        )

        plan = planner.plan(
            ball_position=[1.0, 0.0, 1.0],
            ball_velocity=[-1.0, 0.0, 0.0],
        )

        np.testing.assert_allclose(
            plan.v_racket_target,
            np.array([4.0, 1.4, 8.0], dtype=np.float64),
        )


if __name__ == "__main__":
    unittest.main()
