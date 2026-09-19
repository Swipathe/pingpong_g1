import json
import os
import time
from pathlib import Path

import lcm
from unitree_sdk2.lcm_types.transformation_t import transformation_t

from utils.read_only_lcm import (
    PublicationAudit,
    PublishDenyLcm,
    ReadOnlyLcmSubscription,
)


out = Path(os.environ["OUT"])
out.mkdir(parents=True, exist_ok=True)
audit = PublicationAudit()
counts = {"ball": 0, "table": 0, "other": 0, "decode_errors": 0}
last_ball = None
ball_intervals = []


def handler(channel, payload):
    global last_ball
    try:
        msg = transformation_t.decode(payload)
        name = str(getattr(msg, "name", "") or "").strip().lower()
    except Exception:
        counts["decode_errors"] += 1
        return
    if name == "ball":
        now = time.monotonic()
        counts["ball"] += 1
        if last_ball is not None:
            ball_intervals.append(now - last_ball)
        last_ball = now
    elif name == "table":
        counts["table"] += 1
    else:
        counts["other"] += 1


subscription = ReadOnlyLcmSubscription(
    lcm_url="udpm://239.255.76.67:7667?ttl=255",
    channel="vicon_state_data",
    handler=handler,
    lcm_factory=lambda url: PublishDenyLcm(lcm.LCM(url), audit),
    poll_timeout_ms=10,
)
subscription.start()
time.sleep(10.0)
subscription.close(join_timeout_s=1.0)
publish_count, channels = audit.snapshot()
intervals = sorted(ball_intervals)
p99 = None
if intervals:
    p99 = intervals[min(len(intervals) - 1, int(0.99 * (len(intervals) - 1)))]
summary = dict(counts)
summary.update(
    {
        "duration_s": 10.0,
        "ball_interval_count": len(ball_intervals),
        "ball_interarrival_p99_s": p99,
        "publication_attempt_count": publish_count,
        "publication_attempt_channels": list(channels),
    }
)
(out / "sensor-monitor-summary.json").write_text(
    json.dumps(summary, indent=2, sort_keys=True) + "\n",
    encoding="utf-8",
)
print(json.dumps(summary, sort_keys=True))
