from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from diagnostics.hitter_hil_control_audit import (
    ReceiveOnlyControlChannelAuditor,
)


FIXTURE = Path(__file__).resolve().parent / "fixtures" / "chingmu_ghost_incoming_ball.jsonl"


class FakeLcm:
    def __init__(self, _url):
        self.handlers = {}
        self.unsubscribe_count = 0
        self.publish_count = 0

    def subscribe(self, channel, handler):
        self.handlers[str(channel)] = handler
        return str(channel)

    def unsubscribe(self, subscription):
        self.unsubscribe_count += 1
        self.handlers.pop(str(subscription), None)

    def handle_timeout(self, _timeout_ms):
        return None

    def inject(self, channel, payload):
        handler = self.handlers.get(str(channel))
        if handler is not None:
            handler(str(channel), payload)

    def publish(self, channel, payload):
        self.publish_count += 1
        self.inject(channel, payload)


class Clock:
    def __init__(self):
        self.now = 1.0

    def __call__(self):
        return self.now


class HitterChingMuHilReplayTests(unittest.TestCase):
    def test_fixture_is_ball_only_and_deterministic(self):
        records = [
            json.loads(line)
            for line in FIXTURE.read_text(encoding="utf-8").splitlines()
            if line.strip()
        ]

        self.assertGreaterEqual(len(records), 31)
        self.assertTrue(all(record["channel"] == "vicon_state_data" for record in records))
        self.assertTrue(all(record["name"] == "ball" for record in records))
        self.assertTrue(all(record["valid"] for record in records))
        self.assertEqual(
            [record["source_frame"] for record in records],
            list(range(1, len(records) + 1)),
        )
        self.assertEqual(records, [json.loads(json.dumps(record, sort_keys=True)) for record in records])

    def test_control_auditor_empty_window_stays_zero(self):
        fake_lcm = FakeLcm("memq://")
        clock = Clock()
        auditor = ReceiveOnlyControlChannelAuditor(
            lcm_url="memq://",
            duration_s=0.0,
            lcm_factory=lambda _url: fake_lcm,
            monotonic_fn=clock,
        )

        auditor.start()
        summary = auditor.summary(duration_s=0.0)
        auditor.close()

        self.assertEqual(summary.message_count, 0)
        self.assertEqual(summary.payload_bytes_total, 0)
        self.assertEqual(fake_lcm.publish_count, 0)
        self.assertEqual(fake_lcm.unsubscribe_count, 1)
        self.assertFalse(hasattr(auditor, "publish"))

    def test_control_auditor_counts_payload_without_decoding_or_publishing(self):
        fake_lcm = FakeLcm("memq://")
        clock = Clock()
        auditor = ReceiveOnlyControlChannelAuditor(
            lcm_url="memq://",
            duration_s=0.0,
            lcm_factory=lambda _url: fake_lcm,
            monotonic_fn=clock,
        )

        auditor.start()
        fake_lcm.publish("pd_plustau_targets", b"abc")
        clock.now = 2.0
        fake_lcm.publish("pd_plustau_targets", b"12345")

        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "control-summary.json"
            summary = auditor.write_summary(path, duration_s=0.0)
            loaded = json.loads(path.read_text(encoding="utf-8"))

        auditor.close()

        self.assertEqual(summary.message_count, 2)
        self.assertEqual(summary.payload_bytes_total, 8)
        self.assertEqual(summary.first_message_monotonic_s, 1.0)
        self.assertEqual(summary.last_message_monotonic_s, 2.0)
        self.assertEqual(loaded["message_count"], 2)
        self.assertEqual(loaded["payload_bytes_total"], 8)


if __name__ == "__main__":
    unittest.main()
