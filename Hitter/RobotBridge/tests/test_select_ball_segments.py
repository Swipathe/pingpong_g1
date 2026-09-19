import csv
import sys
import tempfile
import unittest
from pathlib import Path

DEPLOY_DIR = Path(__file__).resolve().parents[1] / "deploy"
sys.path.insert(0, str(DEPLOY_DIR))

from mocap_bridge.select_ball_segments import (
    SelectedSegment,
    export_selected_segments,
    load_recorded_ball_rows,
    nearest_display_point_index,
    split_candidates,
)


class SelectBallSegmentsTest(unittest.TestCase):
    def test_export_selected_segments_preserves_rows_and_adds_clean_segment_ids(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            source = Path(tmpdir) / "session.csv"
            output = Path(tmpdir) / "clean.csv"
            with source.open("w", newline="") as handle:
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
                        "note",
                    ]
                )
                writer.writerow([100.00, 0.00, 0, "ball", 1, 0, 0.0, 0.0, 1.0, "a"])
                writer.writerow([100.01, 0.01, 1, "ball", 1, 0, 0.1, 0.0, 1.1, "b"])
                writer.writerow([100.02, 0.02, 2, "G1Pelvis", 1, 0, 9.0, 9.0, 9.0, "base"])
                writer.writerow([100.03, 0.03, 3, "ball", 1, 0, 0.2, 0.0, 1.0, "c"])
                writer.writerow([100.50, 0.50, 4, "ball", 1, 0, 1.0, 0.0, 0.9, "d"])
                writer.writerow([100.51, 0.51, 5, "ball", 1, 0, 1.1, 0.0, 0.8, "e"])

            rows = load_recorded_ball_rows([source], time_source="elapsed")
            candidates = split_candidates(rows, max_gap_s=0.10, min_samples=1)
            export_selected_segments(
                [
                    SelectedSegment(rows=candidates[0], start_time=0.005, end_time=0.031),
                    SelectedSegment(rows=candidates[1], start_time=0.50, end_time=0.51),
                ],
                output,
            )

            with output.open(newline="") as handle:
                exported = list(csv.DictReader(handle))

        self.assertEqual([row["note"] for row in exported], ["b", "c", "d", "e"])
        self.assertEqual([row["clean_segment_id"] for row in exported], ["1", "1", "2", "2"])
        self.assertEqual([row["source_file"] for row in exported], ["session.csv"] * 4)
        self.assertEqual([row["source_row_index"] for row in exported], ["3", "5", "6", "7"])
        self.assertEqual(exported[0]["ball_x_m"], "0.1")
        self.assertIn("elapsed_s", exported[0])

    def test_nearest_display_point_index_returns_closest_projected_point(self):
        index = nearest_display_point_index(
            [(10.0, 10.0), (100.0, 100.0), (200.0, 50.0)],
            click_xy=(96.0, 103.0),
            max_distance_px=20.0,
        )

        self.assertEqual(index, 1)
        self.assertIsNone(
            nearest_display_point_index(
                [(10.0, 10.0), (100.0, 100.0)],
                click_xy=(400.0, 400.0),
                max_distance_px=20.0,
            )
        )


if __name__ == "__main__":
    unittest.main()
