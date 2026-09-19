import csv
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

DEPLOY_DIR = Path(__file__).resolve().parents[1] / "deploy"
sys.path.insert(0, str(DEPLOY_DIR))

from mocap_bridge.fit_ball_dynamics import fit_csv_paths, read_nexus_csv


class FitBallDynamicsTest(unittest.TestCase):
    def test_fit_csv_paths_recovers_drag_and_restitution(self):
        dt = 1.0 / 300.0
        true_k = 0.012
        true_ch = 0.88
        true_cv = 0.76
        gravity = np.array([0.0, 0.0, -9.81], dtype=np.float64)
        contact_z = 0.78

        pos = np.array([0.15, -0.22, 1.30], dtype=np.float64)
        vel = np.array([2.25, 0.30, -1.00], dtype=np.float64)
        rows = []
        bounced = False
        for index in range(260):
            t = index * dt
            rows.append((t, pos.copy()))
            acc = gravity - true_k * np.linalg.norm(vel) * vel
            next_vel = vel + acc * dt
            next_pos = pos + vel * dt + 0.5 * acc * dt * dt
            if not bounced and next_pos[2] <= contact_z:
                next_pos[2] = contact_z
                next_vel[:2] *= true_ch
                next_vel[2] = abs(next_vel[2]) * true_cv
                bounced = True
            pos, vel = next_pos, next_vel

        with tempfile.TemporaryDirectory() as tmpdir:
            csv_path = Path(tmpdir) / "ball.csv"
            with csv_path.open("w", newline="") as handle:
                writer = csv.writer(handle)
                writer.writerow(
                    [
                        "host_time_s",
                        "frame_number",
                        "ball_valid",
                        "ball_occluded",
                        "ball_x_m",
                        "ball_y_m",
                        "ball_z_m",
                    ]
                )
                for index, (t, p) in enumerate(rows):
                    writer.writerow([t, index, 1, 0, p[0], p[1], p[2]])

            result = fit_csv_paths(
                [csv_path],
                fit_window=31,
                min_segment_samples=120,
                bounce_window_s=0.035,
                exclude_bounce_window_s=0.045,
                min_flight_speed=0.20,
            )

        self.assertEqual(result.trajectory_count, 1)
        self.assertEqual(result.bounce_count, 1)
        self.assertAlmostEqual(result.drag_coefficient, true_k, delta=0.004)
        self.assertAlmostEqual(result.horizontal_restitution, true_ch, delta=0.06)
        self.assertAlmostEqual(result.vertical_restitution, true_cv, delta=0.06)

    def test_read_nexus_csv_uses_frame_time_but_host_time_for_gaps(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            csv_path = Path(tmpdir) / "bursty.csv"
            with csv_path.open("w", newline="") as handle:
                writer = csv.writer(handle)
                writer.writerow(
                    [
                        "host_time_s",
                        "frame_number",
                        "ball_valid",
                        "ball_occluded",
                        "ball_x_m",
                        "ball_y_m",
                        "ball_z_m",
                    ]
                )
                writer.writerow([10.0000, 0, 1, 0, 0.0, 0.0, 1.0])
                writer.writerow([10.0800, 1, 1, 0, 0.1, 0.0, 1.0])
                writer.writerow([10.0803, 2, 1, 0, 0.2, 0.0, 1.0])
                writer.writerow([11.0000, 3, 1, 0, 1.0, 0.0, 1.0])
                writer.writerow([11.0003, 4, 1, 0, 1.1, 0.0, 1.0])

            trajectories = read_nexus_csv(csv_path, sample_rate_hz=300.0, max_gap_s=0.10)

        self.assertEqual(len(trajectories), 2)
        np.testing.assert_allclose(np.diff(trajectories[0].times), [1.0 / 300.0, 1.0 / 300.0])
        np.testing.assert_allclose(np.diff(trajectories[1].times), [1.0 / 300.0])


if __name__ == "__main__":
    unittest.main()
