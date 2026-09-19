import csv
import sys
import tempfile
import unittest
from pathlib import Path

DEPLOY_DIR = Path(__file__).resolve().parents[1] / "deploy"
sys.path.insert(0, str(DEPLOY_DIR))

from mocap_bridge.evaluate_planner_paper_style import (
    build_paper_style_summary,
    read_planner_eval_csv,
)


class EvaluatePlannerPaperStyleTest(unittest.TestCase):
    def test_summary_uses_actual_time_before_strike_for_horizon_errors(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            csv_path = Path(tmpdir) / "eval.csv"
            with csv_path.open("w", newline="") as handle:
                writer = csv.writer(handle)
                writer.writerow(
                    [
                        "created_time_s",
                        "time_to_hit_s",
                        "predicted_hit_time_s",
                        "actual_hit_time_s",
                        "predicted_x_m",
                        "predicted_y_m",
                        "predicted_z_m",
                        "actual_x_m",
                        "actual_y_m",
                        "actual_z_m",
                        "position_error_cm",
                        "time_error_ms",
                    ]
                )
                writer.writerow([9.50, 0.10, 9.60, 10.00, 0, 0, 0, 0, 0, 0, 5.0, 400.0])
                writer.writerow([9.90, 0.10, 10.00, 10.00, 0, 0, 0, 0, 0, 0, 0.2, 0.0])
                writer.writerow([19.49, 0.10, 19.59, 20.00, 0, 0, 0, 0, 0, 0, 7.0, 410.0])
                writer.writerow([19.90, 0.10, 20.00, 20.00, 0, 0, 0, 0, 0, 0, 0.1, 0.0])

            rows = read_planner_eval_csv(csv_path)
            summary = build_paper_style_summary(rows, horizons_s=[0.5, 0.1], bin_width_s=0.1, max_lead_s=0.6)

        self.assertEqual(summary["event_count"], 2)
        horizon_by_time = {item["lead_time_s"]: item for item in summary["horizons"]}
        self.assertAlmostEqual(horizon_by_time[0.5]["position_error_mean_cm"], 6.0)
        self.assertAlmostEqual(horizon_by_time[0.5]["time_error_mean_ms"], 405.0)
        self.assertAlmostEqual(horizon_by_time[0.1]["position_error_mean_cm"], 0.15)
        self.assertAlmostEqual(horizon_by_time[0.1]["time_error_mean_ms"], 0.0)


if __name__ == "__main__":
    unittest.main()
