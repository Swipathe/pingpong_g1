from __future__ import annotations

import csv
import io
import tempfile
import unittest
from datetime import datetime
from pathlib import Path
from unittest.mock import patch

from mocap_bridge.record_vicon_ball import (
    BallCsvWriter,
    BallVelocityTracker,
    default_output_path,
    main,
)


class BallVelocityTrackerTests(unittest.TestCase):
    def test_consecutive_frames_use_source_time_for_vx(self):
        tracker = BallVelocityTracker()

        self.assertIsNone(tracker.update(100, 20.0, 1.0))
        self.assertAlmostEqual(tracker.update(101, 20.01, 0.98), -2.0)

    def test_frame_gap_leaves_vx_empty_and_restarts_from_current_sample(self):
        tracker = BallVelocityTracker()
        tracker.update(100, 20.0, 1.0)

        self.assertIsNone(tracker.update(103, 20.03, 0.94))
        self.assertAlmostEqual(tracker.update(104, 20.04, 0.93), -1.0)
        self.assertEqual(tracker.gap_count, 1)

    def test_invalid_sample_resets_velocity_history(self):
        tracker = BallVelocityTracker()
        tracker.update(100, 20.0, 1.0)

        self.assertIsNone(tracker.update(101, 20.01, 0.99, valid=False))
        self.assertIsNone(tracker.update(102, 20.02, 0.98))

    def test_non_increasing_source_time_leaves_vx_empty(self):
        tracker = BallVelocityTracker()
        tracker.update(100, 20.0, 1.0)

        self.assertIsNone(tracker.update(101, 20.0, 0.99))


class BallCsvWriterTests(unittest.TestCase):
    def test_default_output_uses_timestamped_tmp_path(self):
        output = default_output_path(datetime(2026, 7, 15, 14, 30, 0))

        self.assertEqual(
            output,
            Path("/tmp/hitter_ball_20260715-143000_xyz_vx.csv"),
        )

    def test_writer_emits_unix_newlines_and_blank_vx_after_gap(self):
        output = io.StringIO(newline="")
        writer = BallCsvWriter(output)

        writer.write_sample(
            host_time_s=100.0,
            start_time_s=99.0,
            frame_number=10,
            source_time_s=20.0,
            position_m=(1.0, 2.0, 3.0),
            valid=True,
            occluded=False,
        )
        writer.write_sample(
            host_time_s=100.1,
            start_time_s=99.0,
            frame_number=12,
            source_time_s=20.02,
            position_m=(0.9, 2.1, 3.1),
            valid=True,
            occluded=False,
        )

        text = output.getvalue()
        self.assertNotIn("\r", text)
        rows = list(csv.DictReader(io.StringIO(text)))
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0]["x_m"], "1.000000000")
        self.assertEqual(rows[0]["vx_mps"], "")
        self.assertEqual(rows[1]["vx_mps"], "")
        self.assertEqual(writer.gap_count, 1)


class RecorderLifecycleTests(unittest.TestCase):
    def test_ctrl_c_closes_empty_recording_without_error_exit(self):
        class FakeLcm:
            def subscribe(self, _channel, _handler):
                return None

            def fileno(self):
                return 0

        with tempfile.TemporaryDirectory() as directory:
            output_path = Path(directory) / "recording.csv"
            with patch(
                "mocap_bridge.record_vicon_ball.lcm.LCM",
                return_value=FakeLcm(),
            ):
                with patch(
                    "mocap_bridge.record_vicon_ball.select.select",
                    side_effect=KeyboardInterrupt(),
                ):
                    exit_code = main(["--output", str(output_path)])

            self.assertEqual(exit_code, 0)
            self.assertEqual(output_path.read_text().count("\n"), 1)


if __name__ == "__main__":
    unittest.main()
