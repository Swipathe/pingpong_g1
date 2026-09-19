from __future__ import annotations

import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from deploy.mocap_bridge.monitor_hitter_prediction import (
    PredictionMonitor,
    format_report,
    load_motion_config,
    parse_args,
)
from utils.hitter_planner import BallStateEstimate, HitterWbcCommand, StrikePlan


ROOT = Path(__file__).resolve().parents[3]


def message(
    name,
    *,
    frame=1,
    source_time=1.0,
    position=(1.0, 0.0, 1.0),
    quaternion=(0.0, 0.0, 0.0, 1.0),
    valid=1,
    occluded=0,
):
    return SimpleNamespace(
        name=name,
        vicon_frame_number=int(frame),
        vicon_time_s=float(source_time),
        publish_time_us=0,
        pos_vicon=np.asarray(position, dtype=np.float64),
        quat_vicon=np.asarray(quaternion, dtype=np.float64),
        valid=int(valid),
        occluded=int(occluded),
    )


class ReadyEstimator:
    def __init__(self, velocity=(-1.5, 0.0, 0.0)):
        self.velocity = np.asarray(velocity, dtype=np.float64)
        self.reset_calls = 0

    def add_sample(self, position, *, timestamp):
        return BallStateEstimate(
            position=np.asarray(position, dtype=np.float64),
            velocity=self.velocity.copy(),
            sample_count=31,
            valid=True,
            bounce_detected=False,
        )

    def reset(self):
        self.reset_calls += 1


class FixedPlanner:
    def __init__(self, *, tts=0.75):
        self.tts = float(tts)
        self.calls = []

    def plan_command(self, position, velocity, **kwargs):
        self.calls.append(
            (
                np.asarray(position).copy(),
                np.asarray(velocity).copy(),
                kwargs,
            )
        )
        strike_plan = StrikePlan(
            t_strike=self.tts,
            p_racket_target=np.array([0.0, 0.2, 1.0]),
            v_racket_target=np.array([2.0, 3.0, 4.0]),
            v_ball_in=np.array([-1.5, 0.1, -0.2]),
            v_ball_out=np.array([2.5, -0.1, 1.2]),
        )
        return HitterWbcCommand(
            strike_type="backhand",
            p_base_target_xy=np.array([-0.4, -0.2]),
            v_racket_target_w=strike_plan.v_racket_target,
            time_to_strike=self.tts,
            strike_plan=strike_plan,
        )


class MonitorHitterPredictionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config_path = ROOT / "deploy/config/mimic/hitter.yaml"
        cls.motion = load_motion_config(cls.config_path)

    def make_monitor(self, *, estimator=None, planner=None):
        return PredictionMonitor(
            self.motion,
            estimator=ReadyEstimator() if estimator is None else estimator,
            planner=FixedPlanner() if planner is None else planner,
        )

    def test_loads_current_production_rates_and_lifecycle_window(self):
        planner = self.motion["ball_planner"]

        self.assertEqual(planner["state_estimator_sample_rate_hz"], 360.0)
        self.assertEqual(planner["planner_update_rate_hz"], 100.0)
        self.assertEqual(planner["stable_incoming_confirmation_snapshots"], 3)
        self.assertEqual(planner["arm_time_to_strike_s"], 0.92)
        self.assertEqual(planner["minimum_arm_time_to_strike_s"], 0.60)

    def test_default_cli_uses_shared_lcm_contract(self):
        args = parse_args([])

        self.assertEqual(args.channel, "vicon_state_data")
        self.assertEqual(args.base_name, "G1Pelvis")
        self.assertEqual(args.ball_name, "ball")
        self.assertEqual(args.config.resolve(), self.config_path.resolve())

    def test_throttles_prediction_attempts_to_configured_100hz(self):
        monitor = self.make_monitor()
        monitor.handle_message(message("G1Pelvis"), received_monotonic_s=0.0)

        first = monitor.handle_message(
            message("ball", frame=1, source_time=1.000),
            received_monotonic_s=10.000,
        )
        suppressed = monitor.handle_message(
            message("ball", frame=2, source_time=1.003),
            received_monotonic_s=10.003,
        )
        second = monitor.handle_message(
            message("ball", frame=3, source_time=1.010),
            received_monotonic_s=10.010,
        )

        self.assertIsNotNone(first)
        self.assertIsNone(suppressed)
        self.assertIsNotNone(second)

    def test_third_stable_incoming_attempt_plans_and_arms_without_policy(self):
        planner = FixedPlanner(tts=0.75)
        monitor = self.make_monitor(planner=planner)
        monitor.handle_message(
            message("G1Pelvis", position=(-0.4, 0.0, 0.793)),
            received_monotonic_s=0.0,
        )

        reports = [
            monitor.handle_message(
                message(
                    "ball",
                    frame=index,
                    source_time=1.0 + 0.01 * index,
                    position=(1.0 - 0.02 * index, 0.0, 1.0),
                ),
                received_monotonic_s=10.0 + 0.01 * index,
            )
            for index in (1, 2, 3)
        ]

        self.assertEqual(reports[0].status, "rejected")
        self.assertIn("not stably incoming", reports[0].error)
        self.assertEqual(reports[1].status, "rejected")
        self.assertEqual(reports[2].status, "planned")
        self.assertEqual(reports[2].decision, "armed")
        self.assertAlmostEqual(reports[2].time_to_strike_s, 0.75)
        np.testing.assert_allclose(reports[2].strike_position_w, [0.0, 0.2, 1.0])
        np.testing.assert_allclose(reports[2].racket_velocity_w, [2.0, 3.0, 4.0])
        self.assertEqual(len(planner.calls), 1)

        rendered = format_report(reports[2])
        self.assertIn("decision=armed", rendered)
        self.assertIn("tts_s=0.750", rendered)
        self.assertIn("hit_w=[0.0000,0.2000,1.0000]", rendered)
        self.assertIn("racket_v_w=[2.0000,3.0000,4.0000]", rendered)

    def test_invalid_ball_ends_track_and_resets_estimator(self):
        estimator = ReadyEstimator()
        monitor = self.make_monitor(estimator=estimator)

        report = monitor.handle_message(
            message("ball", valid=0, occluded=1),
            received_monotonic_s=10.0,
        )

        self.assertEqual(report.status, "track-ended")
        self.assertEqual(report.track_epoch, 0)
        self.assertEqual(estimator.reset_calls, 1)
        self.assertEqual(monitor.track_epoch, 1)


if __name__ == "__main__":
    unittest.main()
