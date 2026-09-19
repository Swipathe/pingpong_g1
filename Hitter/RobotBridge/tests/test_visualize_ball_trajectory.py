import csv
import os
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

DEPLOY_DIR = Path(__file__).resolve().parents[1] / "deploy"
sys.path.insert(0, str(DEPLOY_DIR))

from mocap_bridge.visualize_ball_trajectory import (
    BallTrajectoryAnalyzer,
    LiveStatus,
    TrajectoryPlotter,
    VisualizerConfig,
    drain_lcm,
    load_ball_csv_samples,
    next_stream_timestamp,
)


class VisualizeBallTrajectoryTest(unittest.TestCase):
    def test_drain_lcm_handles_multiple_ready_messages(self):
        class FakeLCM:
            def __init__(self, count):
                self.read_fd, self.write_fd = os.pipe()
                os.write(self.write_fd, b"x" * count)
                self.handled = 0

            def fileno(self):
                return self.read_fd

            def handle(self):
                data = os.read(self.read_fd, 1)
                if not data:
                    raise RuntimeError("handle called with no data")
                self.handled += 1

            def close(self):
                os.close(self.read_fd)
                os.close(self.write_fd)

        fake = FakeLCM(5)
        try:
            handled = drain_lcm(fake, timeout_s=0.0, max_messages=10)
        finally:
            fake.close()

        self.assertEqual(handled, 5)
        self.assertEqual(fake.handled, 5)

    def test_load_ball_csv_samples_filters_ball_rows_and_uses_frame_time_by_default(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            csv_path = Path(tmpdir) / "samples.csv"
            with csv_path.open("w", newline="") as handle:
                writer = csv.writer(handle)
                writer.writerow(
                    [
                        "host_time_s",
                        "elapsed_s",
                        "frame_number",
                        "name",
                        "ball_valid",
                        "ball_occluded",
                        "ball_x_m",
                        "ball_y_m",
                        "ball_z_m",
                    ]
                )
                writer.writerow([100.0, 0.10, 1, "ball", 1, 0, 0.1, -0.2, 1.0])
                writer.writerow([100.1, 0.20, 2, "G1Pelvis", 1, 0, 9.0, 9.0, 9.0])
                writer.writerow([100.2, 0.30, 3, "ball", 1, 0, 0.3, -0.1, 0.9])

            samples = load_ball_csv_samples(csv_path)
            elapsed_samples = load_ball_csv_samples(csv_path, time_source="elapsed")

        self.assertEqual(len(samples), 2)
        self.assertEqual([sample.timestamp for sample in samples], [1.0 / 300.0, 3.0 / 300.0])
        np.testing.assert_allclose(samples[0].position, [0.1, -0.2, 1.0])
        np.testing.assert_allclose(samples[1].position, [0.3, -0.1, 0.9])
        self.assertEqual([sample.timestamp for sample in elapsed_samples], [0.10, 0.30])

    def test_load_ball_csv_samples_prefers_vicon_time_when_available(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            csv_path = Path(tmpdir) / "samples.csv"
            with csv_path.open("w", newline="") as handle:
                writer = csv.writer(handle)
                writer.writerow(
                    [
                        "host_time_s",
                        "elapsed_s",
                        "frame_number",
                        "vicon_time_s",
                        "name",
                        "ball_valid",
                        "ball_occluded",
                        "ball_x_m",
                        "ball_y_m",
                        "ball_z_m",
                    ]
                )
                writer.writerow([100.0, 0.10, 9000, 12.5, "ball", 1, 0, 0.1, -0.2, 1.0])

            samples = load_ball_csv_samples(csv_path)
            auto_samples = load_ball_csv_samples(csv_path, time_source="auto")

        self.assertEqual([sample.timestamp for sample in samples], [12.5])
        self.assertEqual([sample.timestamp for sample in auto_samples], [12.5])

    def test_next_stream_timestamp_uses_nominal_rate_inside_track_and_wall_gap_between_tracks(self):
        t0 = next_stream_timestamp(
            None,
            None,
            10.0,
            sample_rate_hz=300.0,
            max_sample_gap_s=0.25,
        )
        t1 = next_stream_timestamp(
            t0,
            10.0,
            10.012,
            sample_rate_hz=300.0,
            max_sample_gap_s=0.25,
        )
        t2 = next_stream_timestamp(
            t1,
            10.012,
            11.012,
            sample_rate_hz=300.0,
            max_sample_gap_s=0.25,
        )

        self.assertEqual(t0, 0.0)
        self.assertAlmostEqual(t1, 1.0 / 300.0)
        self.assertAlmostEqual(t2, t1 + 1.0)

    def test_analyzer_estimates_velocity_predicts_and_records_bounce(self):
        config = VisualizerConfig(
            window_s=2.0,
            estimator_window=5,
            estimator_min_samples=3,
            table_height=0.0,
            ball_radius=0.0,
            table_center_xy=(0.0, 0.0),
            table_length=2.0,
            table_width=1.0,
            bounce_height_tolerance=0.01,
            prediction_horizon_s=0.2,
            prediction_dt=0.02,
            drag_coefficient=0.0,
            max_sample_gap_s=0.5,
        )
        analyzer = BallTrajectoryAnalyzer(config)

        analyzer.add_sample([0.0, 0.0, 0.08], timestamp=0.00)
        analyzer.add_sample([0.0, 0.0, 0.03], timestamp=0.01)
        bounce_snapshot = analyzer.add_sample([0.0, 0.0, 0.00], timestamp=0.02)

        self.assertTrue(bounce_snapshot.last_bounce_detected)
        self.assertEqual(len(bounce_snapshot.bounce_positions), 1)

        analyzer.add_sample([0.0, 0.0, 0.035], timestamp=0.03)
        snapshot = analyzer.add_sample([0.0, 0.0, 0.06], timestamp=0.04)

        self.assertTrue(snapshot.estimate_valid)
        self.assertGreater(snapshot.estimate_velocity[2], 0.0)
        self.assertGreater(snapshot.speed_mps, 0.0)
        self.assertGreater(snapshot.predicted_positions.shape[0], 1)
        np.testing.assert_allclose(snapshot.predicted_positions[0], snapshot.estimate_position)

    def test_analyzer_reports_predicted_hit_point(self):
        config = VisualizerConfig(
            window_s=2.0,
            estimator_window=3,
            estimator_min_samples=3,
            table_height=-1.0,
            ball_radius=0.0,
            gravity=(0.0, 0.0, 0.0),
            drag_coefficient=0.0,
            prediction_horizon_s=1.0,
            prediction_dt=0.01,
            virtual_hit_plane_x=0.0,
            max_sample_gap_s=0.5,
        )
        analyzer = BallTrajectoryAnalyzer(config)

        analyzer.add_sample([0.30, 0.10, 1.00], timestamp=0.00)
        analyzer.add_sample([0.29, 0.10, 1.00], timestamp=0.01)
        snapshot = analyzer.add_sample([0.28, 0.10, 1.00], timestamp=0.02)

        self.assertIsNotNone(snapshot.hit_prediction)
        self.assertAlmostEqual(snapshot.hit_prediction.time_to_hit_s, 0.28, places=6)
        np.testing.assert_allclose(snapshot.hit_prediction.hit_position, [0.0, 0.10, 1.00], atol=1.0e-10)
        np.testing.assert_allclose(snapshot.hit_prediction.racket_velocity.shape, (3,))
        self.assertGreaterEqual(snapshot.pending_hit_positions.shape[0], 1)
        np.testing.assert_allclose(snapshot.pending_hit_positions[-1], [0.0, 0.10, 1.00], atol=1.0e-10)

    def test_analyzer_evaluates_pending_predictions_at_incoming_plane_crossing(self):
        config = VisualizerConfig(
            window_s=2.0,
            estimator_window=3,
            estimator_min_samples=3,
            table_height=-1.0,
            ball_radius=0.0,
            gravity=(0.0, 0.0, 0.0),
            drag_coefficient=0.0,
            prediction_horizon_s=1.0,
            prediction_dt=0.01,
            virtual_hit_plane_x=0.0,
            max_sample_gap_s=0.5,
        )
        analyzer = BallTrajectoryAnalyzer(config)

        snapshot = None
        for index in range(34):
            t = index * 0.01
            x = 0.30 - t
            snapshot = analyzer.add_sample([x, 0.10, 1.00], timestamp=t)

        self.assertIsNotNone(snapshot)
        self.assertGreater(snapshot.evaluation_count, 0)
        self.assertIsNotNone(snapshot.latest_evaluation)
        self.assertLess(snapshot.latest_evaluation.position_error_m, 1.0e-9)
        self.assertLess(snapshot.latest_evaluation.time_error_s, 1.0e-9)
        self.assertLess(snapshot.position_error_median_m, 1.0e-9)

    def test_analyzer_does_not_evaluate_crossing_across_sample_gap(self):
        config = VisualizerConfig(
            window_s=2.0,
            estimator_window=3,
            estimator_min_samples=3,
            table_height=-1.0,
            ball_radius=0.0,
            gravity=(0.0, 0.0, 0.0),
            drag_coefficient=0.0,
            prediction_horizon_s=1.0,
            prediction_dt=0.01,
            virtual_hit_plane_x=0.0,
            max_sample_gap_s=0.25,
        )
        analyzer = BallTrajectoryAnalyzer(config)

        analyzer.add_sample([0.30, 0.10, 1.00], timestamp=0.00)
        analyzer.add_sample([0.29, 0.10, 1.00], timestamp=0.01)
        with_prediction = analyzer.add_sample([0.28, 0.10, 1.00], timestamp=0.02)
        self.assertIsNotNone(with_prediction.hit_prediction)

        after_gap = analyzer.add_sample([-0.10, 0.50, 1.20], timestamp=1.00)

        self.assertEqual(after_gap.evaluation_count, 0)
        self.assertEqual(after_gap.observed_positions.shape[0], 1)

    def test_plotter_accepts_hit_prediction_and_evaluation_metrics(self):
        import matplotlib

        matplotlib.use("Agg")
        config = VisualizerConfig(
            window_s=2.0,
            estimator_window=3,
            estimator_min_samples=3,
            table_height=-1.0,
            ball_radius=0.0,
            gravity=(0.0, 0.0, 0.0),
            drag_coefficient=0.0,
            prediction_horizon_s=1.0,
            prediction_dt=0.01,
            virtual_hit_plane_x=0.0,
            max_sample_gap_s=0.5,
        )
        analyzer = BallTrajectoryAnalyzer(config)
        snapshot = None
        for index in range(34):
            t = index * 0.01
            x = 0.30 - t
            snapshot = analyzer.add_sample([x, 0.10, 1.00], timestamp=t)

        plotter = TrajectoryPlotter(config)
        plotter.update(snapshot)

        text = plotter.status_text.get_text()
        self.assertIn("paper:", text)
        self.assertNotIn("eval: latest=", text)
        plotter.plt.close(plotter.fig)

    def test_plotter_shows_paper_style_prediction_error_summary(self):
        import matplotlib

        matplotlib.use("Agg")
        config = VisualizerConfig(
            window_s=2.0,
            estimator_window=3,
            estimator_min_samples=3,
            table_height=-1.0,
            ball_radius=0.0,
            gravity=(0.0, 0.0, 0.0),
            drag_coefficient=0.0,
            prediction_horizon_s=1.0,
            prediction_dt=0.01,
            virtual_hit_plane_x=0.0,
            max_sample_gap_s=0.5,
        )
        analyzer = BallTrajectoryAnalyzer(config)
        snapshot = None
        for index in range(34):
            t = index * 0.01
            x = 0.30 - t
            snapshot = analyzer.add_sample([x, 0.10, 1.00], timestamp=t)

        plotter = TrajectoryPlotter(config)
        plotter.update(snapshot)

        text = plotter.status_text.get_text()
        self.assertIn("paper:", text)
        self.assertIn("0.5s=", text)
        self.assertIn("0.3s=", text)
        self.assertIn("0.1s=", text)
        self.assertIn("prediction error", plotter.ax_zt.get_title())
        plotter.plt.close(plotter.fig)

    def test_plotter_highlights_latest_prediction_and_actual_crossing_after_ball_passes_plane(self):
        import matplotlib

        matplotlib.use("Agg")
        config = VisualizerConfig(
            window_s=2.0,
            estimator_window=3,
            estimator_min_samples=3,
            table_height=-1.0,
            ball_radius=0.0,
            gravity=(0.0, 0.0, 0.0),
            drag_coefficient=0.0,
            prediction_horizon_s=1.0,
            prediction_dt=0.01,
            virtual_hit_plane_x=0.0,
            max_sample_gap_s=0.5,
        )
        analyzer = BallTrajectoryAnalyzer(config)
        snapshot = None
        for index in range(34):
            t = index * 0.01
            x = 0.30 - t
            snapshot = analyzer.add_sample([x, 0.10, 1.00], timestamp=t)

        self.assertIsNotNone(snapshot)
        self.assertIsNone(snapshot.hit_prediction)
        self.assertIsNotNone(snapshot.latest_evaluation)

        plotter = TrajectoryPlotter(config)
        plotter.update(snapshot)

        latest = snapshot.latest_evaluation
        hit_offsets = np.asarray(plotter.hit_xy.get_offsets(), dtype=np.float64)
        latest_actual_offsets = np.asarray(plotter.latest_actual_xy.get_offsets(), dtype=np.float64)
        np.testing.assert_allclose(hit_offsets, latest.predicted_position[:2].reshape(1, 2))
        np.testing.assert_allclose(latest_actual_offsets, latest.actual_position[:2].reshape(1, 2))
        self.assertGreater(plotter.hit_xy.get_zorder(), plotter.eval_pred_xy.get_zorder())
        self.assertGreater(plotter.hit_xz.get_zorder(), plotter.eval_pred_xz.get_zorder())
        self.assertGreater(float(plotter.hit_xy.get_sizes()[0]), float(plotter.eval_pred_xy.get_sizes()[0]))
        self.assertGreater(float(plotter.hit_xz.get_sizes()[0]), float(plotter.eval_pred_xz.get_sizes()[0]))
        text = plotter.status_text.get_text()
        self.assertIn("pred=[", text)
        self.assertIn("actual=[", text)
        plotter.plt.close(plotter.fig)

    def test_plotter_live_status_updates_without_ball_snapshot(self):
        import matplotlib

        matplotlib.use("Agg")
        config = VisualizerConfig()
        plotter = TrajectoryPlotter(config)
        status = LiveStatus(
            elapsed_s=1.5,
            total_messages=12,
            base_messages=6,
            table_messages=6,
            ball_messages=0,
            total_hz=8.0,
            ball_hz=0.0,
            last_message_age_s=0.02,
            last_ball_age_s=float("nan"),
            latest_base_position=np.asarray([-0.85, -0.38, 0.82], dtype=np.float64),
            latest_table_position=np.asarray([1.37, 0.0, 0.76], dtype=np.float64),
        )

        plotter.update_live(None, status)

        text = plotter.status_text.get_text()
        self.assertIn("live:", text)
        self.assertIn("ball missing", text)
        self.assertIn("base=", text)
        plotter.plt.close(plotter.fig)

    def test_plotter_live_status_keeps_last_trajectory_while_ball_is_stale(self):
        import matplotlib

        matplotlib.use("Agg")
        config = VisualizerConfig(
            window_s=1.0,
            estimator_window=3,
            estimator_min_samples=3,
            table_height=-1.0,
            ball_radius=0.0,
            gravity=(0.0, 0.0, 0.0),
            drag_coefficient=0.0,
            prediction_horizon_s=1.0,
            prediction_dt=0.01,
            virtual_hit_plane_x=0.0,
            max_sample_gap_s=0.5,
        )
        analyzer = BallTrajectoryAnalyzer(config)
        snapshot = None
        for index in range(34):
            t = index * 0.01
            x = 0.30 - t
            snapshot = analyzer.add_sample([x, 0.10, 1.00], timestamp=t)

        plotter = TrajectoryPlotter(config)
        status = LiveStatus(
            elapsed_s=3.0,
            total_messages=900,
            base_messages=450,
            table_messages=449,
            ball_messages=1,
            total_hz=300.0,
            ball_hz=0.33,
            last_message_age_s=0.01,
            last_ball_age_s=2.0,
        )
        plotter.update_live(snapshot, status)

        self.assertGreater(len(plotter.hit_xy.get_offsets()), 0)
        self.assertGreater(len(plotter.latest_actual_xy.get_offsets()), 0)
        self.assertIn("ball stale", plotter.status_text.get_text())
        plotter.plt.close(plotter.fig)


if __name__ == "__main__":
    unittest.main()
