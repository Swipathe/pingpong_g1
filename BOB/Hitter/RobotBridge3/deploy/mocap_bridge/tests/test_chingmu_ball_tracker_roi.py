import unittest

import numpy as np

from deploy.mocap_bridge.chingmu_table_lcm_bridge import (
    BALL_TRACKING_FAR_EDGE_MARGIN_M,
    BallTracker,
    BridgeConfig,
    TableFrame,
)


class ChingMuBallTrackerRoiTest(unittest.TestCase):
    def setUp(self):
        self.config = BridgeConfig()
        self.table = TableFrame(
            center_raw_mm=np.zeros(3, dtype=np.float64),
            x_axis_raw=np.array([1.0, 0.0, 0.0], dtype=np.float64),
            y_axis_raw=np.array([0.0, 1.0, 0.0], dtype=np.float64),
            z_axis_raw=np.array([0.0, 0.0, 1.0], dtype=np.float64),
            corners_raw_mm=np.zeros((4, 3), dtype=np.float64),
            rectangle_score=0.0,
        )

    def raw_from_world(self, position_world, *, config=None):
        config = self.config if config is None else config
        position = np.asarray(position_world, dtype=np.float64)
        return np.array(
            [
                (position[0] - 0.5 * config.table_length_m) * 1000.0,
                position[1] * 1000.0,
                (position[2] - config.table_height_m) * 1000.0,
            ],
            dtype=np.float64,
        )

    def update_new_track(self, position_world, *, config=None):
        config = self.config if config is None else config
        return BallTracker().update(
            [self.raw_from_world(position_world, config=config)],
            frame_number=1,
            table=self.table,
            config=config,
        )

    def test_far_edge_margin_is_40_cm(self):
        self.assertEqual(BALL_TRACKING_FAR_EDGE_MARGIN_M, 0.40)

    def test_far_edge_cutoff_is_inclusive_for_runtime_table_length(self):
        config = BridgeConfig(table_length_m=3.25)
        cutoff_x = config.table_length_m - 0.40
        epsilon_m = 1.0e-9
        accepted = self.update_new_track(
            [cutoff_x, 0.0, 0.90],
            config=config,
        )
        rejected = self.update_new_track(
            [cutoff_x + epsilon_m, 0.0, 0.90],
            config=config,
        )

        self.assertIsNotNone(accepted.position_world)
        self.assertFalse(accepted.ended)
        self.assertIsNone(rejected.position_world)
        self.assertFalse(rejected.ended)

    def test_new_track_admission_requires_positive_x(self):
        at_near_edge = self.update_new_track([0.0, 0.0, 0.90])
        before_near_edge = self.update_new_track([-0.001, 0.0, 0.90])
        inside_near_edge = self.update_new_track([0.001, 0.0, 0.90])

        self.assertIsNone(at_near_edge.position_world)
        self.assertFalse(at_near_edge.ended)
        self.assertIsNone(before_near_edge.position_world)
        self.assertFalse(before_near_edge.ended)
        self.assertIsNotNone(inside_near_edge.position_world)
        self.assertFalse(inside_near_edge.ended)

    def test_table_width_cutoff_is_inclusive_on_both_sides(self):
        half_width_m = 0.5 * self.config.table_width_m
        epsilon_m = 1.0e-9

        for y_m in (-half_width_m, half_width_m):
            with self.subTest(y_m=y_m, expected="accepted"):
                update = self.update_new_track([0.50, y_m, 0.90])
                self.assertIsNotNone(update.position_world)
                self.assertFalse(update.ended)

        for y_m in (-half_width_m - epsilon_m, half_width_m + epsilon_m):
            with self.subTest(y_m=y_m, expected="rejected"):
                update = self.update_new_track([0.50, y_m, 0.90])
                self.assertIsNone(update.position_world)
                self.assertFalse(update.ended)

    def test_ball_must_be_strictly_above_table_height(self):
        at_table_height = self.update_new_track(
            [0.50, 0.0, self.config.table_height_m]
        )
        above_table_height = self.update_new_track(
            [0.50, 0.0, self.config.table_height_m + 1.0e-9]
        )

        self.assertIsNone(at_table_height.position_world)
        self.assertFalse(at_table_height.ended)
        self.assertIsNotNone(above_table_height.position_world)
        self.assertFalse(above_table_height.ended)

    def test_active_track_ends_once_when_only_candidate_is_beyond_far_edge(self):
        cutoff_x = self.config.table_length_m - 0.40
        tracker = BallTracker()
        accepted = tracker.update(
            [self.raw_from_world([cutoff_x - 0.10, 0.0, 0.90])],
            frame_number=1,
            table=self.table,
            config=self.config,
        )
        ended = tracker.update(
            [self.raw_from_world([cutoff_x + 1.0e-9, 0.0, 0.90])],
            frame_number=2,
            table=self.table,
            config=self.config,
        )
        next_empty = tracker.update(
            [],
            frame_number=3,
            table=self.table,
            config=self.config,
        )

        self.assertIsNotNone(accepted.position_world)
        self.assertFalse(accepted.ended)
        self.assertIsNone(ended.position_world)
        self.assertTrue(ended.ended)
        self.assertIsNone(next_empty.position_world)
        self.assertFalse(next_empty.ended)

    def test_active_track_near_edge_evidence_never_publishes_valid_ball(self):
        tracker = BallTracker()
        admitted = tracker.update(
            [self.raw_from_world([0.001, 0.0, 0.90])],
            frame_number=1,
            table=self.table,
            config=self.config,
        )
        ended = tracker.update(
            [self.raw_from_world([-0.001, 0.0, 0.90])],
            frame_number=2,
            table=self.table,
            config=self.config,
        )

        self.assertIsNotNone(admitted.position_world)
        self.assertFalse(admitted.ended)
        self.assertIsNone(ended.position_world)
        self.assertTrue(ended.ended)


if __name__ == "__main__":
    unittest.main()
