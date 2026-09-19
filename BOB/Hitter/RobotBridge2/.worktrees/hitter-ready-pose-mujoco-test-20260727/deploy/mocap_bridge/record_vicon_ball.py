#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import select
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import TextIO

import lcm

BRIDGE_DIR = Path(__file__).resolve().parents[2]
for import_path in (BRIDGE_DIR, BRIDGE_DIR / "deploy"):
    import_path_text = str(import_path)
    if import_path_text not in sys.path:
        sys.path.insert(0, import_path_text)

from unitree_sdk2.lcm_types.transformation_t import transformation_t


DEFAULT_LCM_URL = "udpm://239.255.76.67:7667?ttl=255"
CSV_FIELDS = [
    "row_index",
    "host_time_s",
    "elapsed_s",
    "frame_number",
    "vicon_time_s",
    "x_m",
    "y_m",
    "z_m",
    "vx_mps",
    "valid",
    "occluded",
]


def default_output_path(now: datetime | None = None) -> Path:
    now = datetime.now() if now is None else now
    timestamp = now.strftime("%Y%m%d-%H%M%S")
    return Path(f"/tmp/hitter_ball_{timestamp}_xyz_vx.csv")


def _message_int(msg, name: str, default: int = 0) -> int:
    try:
        return int(getattr(msg, name))
    except (AttributeError, TypeError, ValueError):
        return default


def _message_float(msg, name: str) -> float | None:
    try:
        value = float(getattr(msg, name))
    except (AttributeError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


class BallVelocityTracker:
    def __init__(self) -> None:
        self._previous_frame: int | None = None
        self._previous_source_time_s: float | None = None
        self._previous_x_m: float | None = None
        self.gap_count = 0

    def update(
        self,
        frame_number: int,
        source_time_s: float,
        x_m: float,
        *,
        valid: bool = True,
    ) -> float | None:
        frame_number = int(frame_number)
        source_time_s = float(source_time_s)
        x_m = float(x_m)

        if not valid:
            self._previous_frame = None
            self._previous_source_time_s = None
            self._previous_x_m = None
            return None

        vx_mps = None
        if self._previous_frame is not None:
            consecutive = frame_number == self._previous_frame + 1
            increasing_time = source_time_s > self._previous_source_time_s
            if consecutive and increasing_time:
                vx_mps = (x_m - self._previous_x_m) / (
                    source_time_s - self._previous_source_time_s
                )
            elif not consecutive:
                self.gap_count += 1

        self._previous_frame = frame_number
        self._previous_source_time_s = source_time_s
        self._previous_x_m = x_m
        return vx_mps


class BallCsvWriter:
    def __init__(self, output: TextIO) -> None:
        self._output = output
        self._writer = csv.DictWriter(
            output,
            fieldnames=CSV_FIELDS,
            lineterminator="\n",
        )
        self._writer.writeheader()
        self._velocity_tracker = BallVelocityTracker()
        self.row_count = 0

    @property
    def gap_count(self) -> int:
        return self._velocity_tracker.gap_count

    def write_sample(
        self,
        *,
        host_time_s: float,
        start_time_s: float,
        frame_number: int,
        source_time_s: float | None,
        position_m,
        valid: bool,
        occluded: bool,
    ) -> None:
        x_m, y_m, z_m = (float(value) for value in position_m)
        source_metadata_valid = (
            int(frame_number) > 0
            and source_time_s is not None
            and math.isfinite(float(source_time_s))
            and float(source_time_s) > 0.0
        )
        velocity_sample_valid = (
            bool(valid)
            and not bool(occluded)
            and source_metadata_valid
            and all(math.isfinite(value) for value in (x_m, y_m, z_m))
        )
        vx_mps = self._velocity_tracker.update(
            int(frame_number),
            0.0 if source_time_s is None else float(source_time_s),
            x_m,
            valid=velocity_sample_valid,
        )
        self._writer.writerow(
            {
                "row_index": self.row_count,
                "host_time_s": f"{float(host_time_s):.9f}",
                "elapsed_s": f"{float(host_time_s) - float(start_time_s):.9f}",
                "frame_number": "" if int(frame_number) <= 0 else int(frame_number),
                "vicon_time_s": (
                    "" if source_time_s is None else f"{float(source_time_s):.9f}"
                ),
                "x_m": f"{x_m:.9f}",
                "y_m": f"{y_m:.9f}",
                "z_m": f"{z_m:.9f}",
                "vx_mps": "" if vx_mps is None else f"{vx_mps:.9f}",
                "valid": int(bool(valid)),
                "occluded": int(bool(occluded)),
            }
        )
        self.row_count += 1

    def flush(self) -> None:
        self._output.flush()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Record Vicon ball X/Y/Z and consecutive-frame Vx until Ctrl+C."
    )
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--lcm-url", default=DEFAULT_LCM_URL)
    parser.add_argument("--channel", default="vicon_state_data")
    parser.add_argument("--ball-name", default="ball")
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Optional recording duration in seconds; zero records until Ctrl+C.",
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    if not math.isfinite(args.duration) or args.duration < 0.0:
        raise ValueError("--duration must be finite and non-negative")

    output_path = default_output_path() if args.output is None else args.output
    output_path = output_path.expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    lcm_client = lcm.LCM(args.lcm_url)
    start_wall_time_s = time.time()
    start_monotonic_s = time.monotonic()

    with output_path.open("w", newline="") as output:
        recorder = BallCsvWriter(output)

        def handler(_channel, data) -> None:
            msg = transformation_t.decode(data)
            if str(getattr(msg, "name", "")) != args.ball_name:
                return

            position = tuple(float(value) for value in msg.pos_vicon[:3])
            frame_number = _message_int(msg, "vicon_frame_number", 0)
            source_time_s = _message_float(msg, "vicon_time_s")
            valid = bool(_message_int(msg, "valid", 1))
            occluded = bool(_message_int(msg, "occluded", 0))
            recorder.write_sample(
                host_time_s=time.time(),
                start_time_s=start_wall_time_s,
                frame_number=frame_number,
                source_time_s=source_time_s,
                position_m=position,
                valid=valid,
                occluded=occluded,
            )
            if recorder.row_count % 300 == 0:
                recorder.flush()

        lcm_client.subscribe(args.channel, handler)
        print(f"Recording ball data to {output_path}", flush=True)
        print("Press Ctrl+C to stop and close the CSV.", flush=True)
        try:
            while True:
                elapsed_s = time.monotonic() - start_monotonic_s
                if args.duration > 0.0 and elapsed_s >= args.duration:
                    break
                ready, _, _ = select.select([lcm_client.fileno()], [], [], 0.1)
                if ready:
                    lcm_client.handle()
        except KeyboardInterrupt:
            pass
        finally:
            recorder.flush()

        elapsed_s = time.monotonic() - start_monotonic_s
        print(
            f"Recording stopped: rows={recorder.row_count} "
            f"duration_s={elapsed_s:.3f} gaps={recorder.gap_count} "
            f"output={output_path}",
            flush=True,
        )
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
