from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from utils.hitter_hil_trace import (
    HitterHilRateLimiter,
    HitterHilTraceConfig,
    HitterHilTraceWriter,
    build_hitter_hil_status_record,
    iter_hitter_hil_trace_records,
)
from utils.read_only_lcm import PublicationAudit


class HitterChingMuHilTraceTests(unittest.TestCase):
    def test_default_trace_config_matches_bounded_operational_limits(self):
        config = HitterHilTraceConfig()

        self.assertEqual(config.queue_capacity, 4096)
        self.assertEqual(config.rotate_bytes, 67108864)
        self.assertEqual(config.retained_files, 4)
        self.assertLessEqual(config.flush_interval_s, 1.0)
        self.assertEqual(config.status_log_interval_s, 1.0)

    def test_rate_limiter_allows_status_at_one_hz(self):
        limiter = HitterHilRateLimiter(interval_s=1.0)

        self.assertTrue(limiter.should_emit(10.0))
        self.assertFalse(limiter.should_emit(10.5))
        self.assertTrue(limiter.should_emit(11.0))

    def test_trace_writer_is_bounded_rotating_jsonl(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "trace.jsonl"
            writer = HitterHilTraceWriter(
                path,
                config=HitterHilTraceConfig(
                    queue_capacity=1,
                    rotate_bytes=120,
                    retained_files=2,
                    flush_interval_s=0.1,
                    status_log_interval_s=1.0,
                ),
                start_thread=False,
            )

            self.assertTrue(writer.record("raw_frame", frame=1))
            self.assertFalse(writer.record("raw_frame", frame=2))
            self.assertEqual(writer.trace_drop_count, 1)
            writer.flush_pending()
            self.assertTrue(path.exists())

            for index in range(20):
                writer.record(
                    "planner",
                    index=index,
                    payload="x" * 80,
                )
                writer.flush_pending()

            files = sorted(path.parent.glob("trace.jsonl*"))
            self.assertLessEqual(len(files), 3)
            writer.close()

    def test_status_record_contains_runtime_diagnostics_and_zero_control_count(self):
        audit = PublicationAudit()
        source_status = SimpleNamespace(
            decoded_count=12,
            accepted_ball_count=5,
            rejected_count=2,
            table_ready=True,
            table_confirmation_count=3,
            thread_alive=True,
            receive_failure=None,
        )
        pipeline_state = SimpleNamespace(
            track_epoch=4,
            generation=7,
            ready=True,
            visible=True,
            sample_count=31,
            min_samples=31,
            last_update_monotonic_s=99.5,
        )
        worker_stats = SimpleNamespace(
            submitted=10,
            completed=9,
            dropped_pending=1,
            failed=0,
        )
        result = SimpleNamespace(
            completed_monotonic_s=99.8,
            strike_deadline_monotonic_s=100.7,
        )

        record = build_hitter_hil_status_record(
            now_monotonic_s=100.0,
            source_status=source_status,
            pipeline_state=pipeline_state,
            worker_stats=worker_stats,
            latest_result=result,
            policy_time_to_strike_s=0.7,
            predicted_strike_point_w=np.asarray([0.0, 0.1, 0.9]),
            commanded_base_target_xy_w=np.asarray([-0.4, 0.0]),
            commanded_racket_velocity_w=np.asarray([4.0, 0.0, 2.0]),
            mujoco_racket_position_w=np.asarray([0.1, 0.2, 0.8]),
            mujoco_racket_velocity_w=np.asarray([3.5, 0.0, 1.5]),
            ghost_to_racket_min_distance_m=0.12,
            publication_audit=audit,
        )

        self.assertEqual(record["kind"], "status")
        self.assertEqual(record["source_decoded_count"], 12)
        self.assertEqual(record["estimator_track_epoch"], 4)
        self.assertEqual(record["estimator_generation"], 7)
        self.assertEqual(record["worker_submitted"], 10)
        self.assertEqual(record["worker_completed"], 9)
        self.assertEqual(record["worker_dropped"], 1)
        self.assertAlmostEqual(record["result_age_s"], 0.2)
        self.assertAlmostEqual(record["policy_time_to_strike_s"], 0.7)
        self.assertEqual(record["control_publication_count"], 0)
        self.assertEqual(record["predicted_strike_point_w"], [0.0, 0.1, 0.9])
        self.assertEqual(record["commanded_base_target_xy_w"], [-0.4, 0.0])
        self.assertEqual(record["commanded_racket_velocity_w"], [4.0, 0.0, 2.0])
        self.assertEqual(record["mujoco_racket_position_w"], [0.1, 0.2, 0.8])
        self.assertEqual(record["mujoco_racket_velocity_w"], [3.5, 0.0, 1.5])
        self.assertEqual(record["ghost_to_racket_min_distance_m"], 0.12)

    def test_jsonl_replay_is_deterministic(self):
        records = [
            {"kind": "raw_frame", "source_time_s": 1.0, "frame": 1},
            {"kind": "planner", "received_monotonic_s": 2.0, "epoch": 3},
            {"kind": "lifecycle", "phase": "armed", "policy_tts": 0.7},
        ]
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "trace.jsonl"
            with path.open("w", encoding="utf-8") as handle:
                for record in records:
                    handle.write(
                        json.dumps(record, sort_keys=True) + "\n"
                    )

            replayed = list(iter_hitter_hil_trace_records(path))

        self.assertEqual(replayed, records)


if __name__ == "__main__":
    unittest.main()
