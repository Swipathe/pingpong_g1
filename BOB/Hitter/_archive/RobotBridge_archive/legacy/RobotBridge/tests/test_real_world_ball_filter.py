import sys
import time
import unittest
from pathlib import Path

import numpy as np

DEPLOY_DIR = Path(__file__).resolve().parents[1] / "deploy"
sys.path.insert(0, str(DEPLOY_DIR))

from simulator.real_world import RealWorld
from unitree_sdk2.lcm_types.transformation_t import transformation_t
from utils.hitter_planner import BallStateEstimator


def _make_real_world_for_ball_filter():
    sim = object.__new__(RealWorld)
    sim.ball_pos_world_tmp = np.zeros(3, dtype=np.float32)
    sim.ball_vel_world_tmp = np.zeros(3, dtype=np.float32)
    sim.ball_visible_tmp = False
    sim.ball_state_estimator_ready_tmp = False
    sim.ball_state_estimator_sample_count_tmp = 0
    sim.ball_state_estimator_time = None
    sim.ball_state_estimator_last_host_time = None
    sim.ball_state_estimator_sample_rate_hz = 300.0
    sim.ball_state_estimator_max_gap_s = 0.25
    sim.ball_state_estimator_last_seen_host_time = None
    sim.ball_state_estimator_stale_timeout_s = 0.2
    sim.ball_state_estimator_min_raw_speed_mps = 0.3
    sim.ball_state_estimator_max_raw_speed_mps = 15.0
    sim.ball_state_estimator = BallStateEstimator(
        window_size=3,
        min_samples=2,
        max_sample_gap_s=0.25,
    )
    sim._ball_raw_last_pos = None
    sim._ball_raw_last_sample_time = None
    sim._ball_raw_last_host_time = None
    sim._ball_tracking_active = False
    sim.firstReceiveVicon = False
    sim.root_trans_world_tmp = np.zeros(3, dtype=np.float32)
    sim.root_quat_world_tmp = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
    return sim


def _ball_msg(pos, *, t, valid=1, occluded=0):
    msg = transformation_t()
    msg.name = "ball"
    msg.valid = valid
    msg.occluded = occluded
    msg.vicon_time_s = float(t)
    msg.publish_time_us = int((1_000.0 + float(t)) * 1_000_000)
    msg.pos_vicon = [float(pos[0]), float(pos[1]), float(pos[2])]
    msg.quat_vicon = [0.0, 0.0, 0.0, 1.0]
    return msg.encode()


class RealWorldBallFilterTest(unittest.TestCase):
    def test_stationary_valid_unlabeled_point_does_not_feed_estimator(self):
        sim = _make_real_world_for_ball_filter()

        for i in range(40):
            pos = [-0.34 + 1.0e-5 * (i % 2), 0.08, 0.68]
            RealWorld._vicon_state_handler(sim, "vicon_state_data", _ball_msg(pos, t=10.0 + i / 300.0))

        self.assertFalse(sim.ball_visible_tmp)
        self.assertFalse(sim.ball_state_estimator_ready_tmp)
        self.assertEqual(sim.ball_state_estimator_sample_count_tmp, 0)
        self.assertEqual(sim.ball_state_estimator.sample_count, 0)

    def test_continuous_moving_ball_feeds_estimator(self):
        sim = _make_real_world_for_ball_filter()

        RealWorld._vicon_state_handler(sim, "vicon_state_data", _ball_msg([1.00, 0.0, 1.0], t=20.000))
        RealWorld._vicon_state_handler(sim, "vicon_state_data", _ball_msg([0.99, 0.0, 1.0], t=20.003333))
        RealWorld._vicon_state_handler(sim, "vicon_state_data", _ball_msg([0.98, 0.0, 1.0], t=20.006667))

        self.assertTrue(sim.ball_visible_tmp)
        self.assertTrue(sim.ball_state_estimator_ready_tmp)
        self.assertEqual(sim.ball_state_estimator_sample_count_tmp, 2)

    def test_implausible_jump_resets_tracking_instead_of_crossing_plane(self):
        sim = _make_real_world_for_ball_filter()

        RealWorld._vicon_state_handler(sim, "vicon_state_data", _ball_msg([0.30, 0.62, 0.54], t=30.000))
        RealWorld._vicon_state_handler(sim, "vicon_state_data", _ball_msg([0.29, 0.62, 0.54], t=30.003333))
        RealWorld._vicon_state_handler(sim, "vicon_state_data", _ball_msg([0.28, 0.62, 0.54], t=30.006667))
        self.assertTrue(sim.ball_state_estimator_ready_tmp)

        RealWorld._vicon_state_handler(sim, "vicon_state_data", _ball_msg([-0.34, 0.08, 0.68], t=30.010000))

        self.assertFalse(sim.ball_visible_tmp)
        self.assertFalse(sim.ball_state_estimator_ready_tmp)
        self.assertEqual(sim.ball_state_estimator_sample_count_tmp, 0)
        self.assertEqual(sim.ball_state_estimator.sample_count, 0)

    def test_stale_ball_expires_visible_and_ready_flags(self):
        sim = _make_real_world_for_ball_filter()
        RealWorld._vicon_state_handler(sim, "vicon_state_data", _ball_msg([1.00, 0.0, 1.0], t=40.000))
        RealWorld._vicon_state_handler(sim, "vicon_state_data", _ball_msg([0.99, 0.0, 1.0], t=40.003333))
        RealWorld._vicon_state_handler(sim, "vicon_state_data", _ball_msg([0.98, 0.0, 1.0], t=40.006667))
        self.assertTrue(sim.ball_visible_tmp)

        RealWorld._expire_ball_if_stale(sim, time.time() + 1.0)

        self.assertFalse(sim.ball_visible_tmp)
        self.assertFalse(sim.ball_state_estimator_ready_tmp)
        self.assertEqual(sim.ball_state_estimator_sample_count_tmp, 0)


if __name__ == "__main__":
    unittest.main()
