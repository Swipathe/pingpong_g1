from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Callable

import lcm


@dataclass(frozen=True)
class ControlChannelAuditSummary:
    lcm_url: str
    channel: str
    duration_s: float
    message_count: int
    payload_bytes_total: int
    first_message_monotonic_s: float | None
    last_message_monotonic_s: float | None


class ReceiveOnlyControlChannelAuditor:
    """Receive-only counter for a control channel.

    The auditor subscribes and records timestamp/count/payload-size only. It
    deliberately does not decode payloads and exposes no publish method.
    """

    def __init__(
        self,
        *,
        lcm_url: str,
        channel: str = "pd_plustau_targets",
        duration_s: float = 1.0,
        lcm_factory: Callable[[str], object] = lcm.LCM,
        monotonic_fn: Callable[[], float] = time.monotonic,
    ) -> None:
        if not callable(lcm_factory):
            raise TypeError("lcm_factory must be callable")
        if not callable(monotonic_fn):
            raise TypeError("monotonic_fn must be callable")
        duration = float(duration_s)
        if duration < 0.0:
            raise ValueError("duration_s must be nonnegative")
        self.lcm_url = str(lcm_url)
        self.channel = str(channel)
        if not self.channel:
            raise ValueError("channel must be nonempty")
        self.duration_s = duration
        self._lcm_factory = lcm_factory
        self._monotonic_fn = monotonic_fn
        self._lc = None
        self._subscription = None
        self._message_count = 0
        self._payload_bytes_total = 0
        self._first_message_monotonic_s = None
        self._last_message_monotonic_s = None

    def start(self) -> None:
        if self._lc is not None:
            return
        self._lc = self._lcm_factory(self.lcm_url)
        self._subscription = self._lc.subscribe(self.channel, self._handle)

    def _handle(self, channel, payload) -> None:
        if str(channel) != self.channel:
            return
        now = float(self._monotonic_fn())
        if self._first_message_monotonic_s is None:
            self._first_message_monotonic_s = now
        self._last_message_monotonic_s = now
        self._message_count += 1
        self._payload_bytes_total += len(bytes(payload))

    def poll_once(self, *, timeout_ms: int = 10) -> None:
        if self._lc is None:
            self.start()
        handle_timeout = getattr(self._lc, "handle_timeout", None)
        if callable(handle_timeout):
            handle_timeout(int(timeout_ms))
            return
        handle = getattr(self._lc, "handle", None)
        if callable(handle):
            handle()

    def run_for(self, duration_s: float | None = None) -> ControlChannelAuditSummary:
        self.start()
        duration = self.duration_s if duration_s is None else float(duration_s)
        deadline = float(self._monotonic_fn()) + max(duration, 0.0)
        while float(self._monotonic_fn()) < deadline:
            self.poll_once(timeout_ms=10)
        return self.summary(duration_s=duration)

    def close(self) -> None:
        if self._lc is not None and self._subscription is not None:
            unsubscribe = getattr(self._lc, "unsubscribe", None)
            if callable(unsubscribe):
                unsubscribe(self._subscription)
        self._subscription = None
        self._lc = None

    def summary(self, *, duration_s: float | None = None) -> ControlChannelAuditSummary:
        return ControlChannelAuditSummary(
            lcm_url=self.lcm_url,
            channel=self.channel,
            duration_s=self.duration_s if duration_s is None else float(duration_s),
            message_count=int(self._message_count),
            payload_bytes_total=int(self._payload_bytes_total),
            first_message_monotonic_s=self._first_message_monotonic_s,
            last_message_monotonic_s=self._last_message_monotonic_s,
        )

    def write_summary(self, path, *, duration_s: float | None = None) -> ControlChannelAuditSummary:
        summary = self.summary(duration_s=duration_s)
        output = Path(path)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(asdict(summary), sort_keys=True, indent=2) + "\n",
            encoding="utf-8",
        )
        return summary


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(
        description="Receive-only HITTER HIL control-channel audit."
    )
    parser.add_argument("--lcm-url", required=True)
    parser.add_argument("--channel", default="pd_plustau_targets")
    parser.add_argument("--duration-s", type=float, default=1.0)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    auditor = ReceiveOnlyControlChannelAuditor(
        lcm_url=args.lcm_url,
        channel=args.channel,
        duration_s=args.duration_s,
    )
    try:
        auditor.run_for()
        auditor.write_summary(args.output)
    finally:
        auditor.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
