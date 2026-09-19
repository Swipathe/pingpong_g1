#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import select
import sys
import time
from pathlib import Path
from typing import Optional

import lcm
import numpy as np

BRIDGE_DIR = Path(__file__).resolve().parents[2]
for path in (BRIDGE_DIR, BRIDGE_DIR / "deploy"):
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.insert(0, path_str)

from unitree_sdk2.lcm_types.transformation_t import transformation_t


CSV_FIELDS = [
    "track_id",
    "valid",
    "occluded",
    "source_frame",
    "source_time_s",
    "publish_time_us",
    "x",
    "y",
    "z",
    "received_monotonic_s",
]


def msg_int_field(msg, name: str, default: int = 0) -> int:
    try:
        return int(getattr(msg, name))
    except (AttributeError, TypeError, ValueError):
        return default


def msg_status(msg) -> tuple[int, int]:
    return (
        msg_int_field(msg, "valid", 1),
        msg_int_field(msg, "occluded", 0),
    )


def msg_float_field(msg, name: str):
    try:
        value = float(getattr(msg, name))
    except (AttributeError, TypeError, ValueError):
        return None
    return value if np.isfinite(value) else None


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--lcm-url", default="udpm://239.255.76.67:7667?ttl=255")
    parser.add_argument("--channel", default="vicon_state_data_v2")
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument(
        "--csv",
        type=Path,
        default=None,
        help="Write strict v2 ball samples to CSV.",
    )
    parser.add_argument("--base-name", default="G2Pelvis")
    parser.add_argument("--ball-name", default="ball")
    parser.add_argument("--print-hz", type=float, default=1.0)
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Suppress periodic summaries; contract transitions still print.",
    )
    return parser


def message_contract_error(message) -> Optional[str]:
    try:
        position = np.asarray(message.pos_vicon, dtype=np.float64).reshape(3)
        quaternion = np.asarray(message.quat_vicon, dtype=np.float64).reshape(4)
    except (AttributeError, TypeError, ValueError):
        return "pose_not_finite"
    if not np.isfinite(position).all() or not np.isfinite(quaternion).all():
        return "pose_not_finite"

    name = str(getattr(message, "name", ""))
    try:
        track_id = int(message.track_id)
    except (AttributeError, TypeError, ValueError, OverflowError):
        return "track_id_not_integer"
    if name == "ball" and track_id <= 0:
        return "ball_track_id_not_positive"
    if name != "ball" and track_id != 0:
        return "non_ball_track_id_not_zero"
    return None


def message_csv_row(
    message,
    received_monotonic_s: float,
) -> dict[str, object]:
    position = np.asarray(message.pos_vicon, dtype=np.float64).reshape(3)
    return {
        "track_id": int(message.track_id),
        "valid": int(message.valid),
        "occluded": int(message.occluded),
        "source_frame": int(message.vicon_frame_number),
        "source_time_s": float(message.vicon_time_s),
        "publish_time_us": int(message.publish_time_us),
        "x": float(position[0]),
        "y": float(position[1]),
        "z": float(position[2]),
        "received_monotonic_s": float(received_monotonic_s),
    }


class MonitorCallback:
    def __init__(self, *, csv_writer=None, decoder=None):
        self.csv_writer = csv_writer
        self.decoder = transformation_t.decode if decoder is None else decoder
        self.received_count = 0
        self.accepted_count = 0
        self.csv_count = 0
        self.contract_error_count = 0
        self.last_by_name = {}
        self.error_by_name = {}

    def _record_error_transition(self, name: str, error: Optional[str]) -> None:
        previous = self.error_by_name.get(name)
        if error == previous:
            return
        if error is None:
            self.error_by_name.pop(name, None)
            print(f"contract_error=none name={name}", flush=True)
        else:
            self.error_by_name[name] = error
            print(f"contract_error={error} name={name}", flush=True)

    def __call__(self, channel, data):
        del channel
        self.received_count += 1
        try:
            message = self.decoder(data)
        except Exception:
            self.contract_error_count += 1
            self._record_error_transition("<decode>", "decode_error")
            return
        self._record_error_transition("<decode>", None)

        try:
            name = str(message.name)
        except Exception:
            self.contract_error_count += 1
            self._record_error_transition("<message>", "field_error")
            return

        contract_error = message_contract_error(message)
        if contract_error is not None:
            self.contract_error_count += 1
            self._record_error_transition(name, contract_error)
            return

        row = None
        if name == "ball" and self.csv_writer is not None:
            try:
                row = message_csv_row(message, time.monotonic())
            except (AttributeError, TypeError, ValueError, OverflowError):
                self.contract_error_count += 1
                self._record_error_transition(name, "field_error")
                return

        self._record_error_transition(name, None)
        self.last_by_name[name] = message
        self.accepted_count += 1
        if row is not None:
            self.csv_writer.writerow(row)
            self.csv_count += 1


def main(argv=None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    if args.channel != "vicon_state_data_v2":
        parser.error("--channel must be exactly vicon_state_data_v2")
    if not np.isfinite(args.duration) or args.duration < 0.0:
        parser.error("--duration must be finite and nonnegative")
    if not np.isfinite(args.print_hz) or args.print_hz != 1.0:
        parser.error("--print-hz must be exactly 1.0")

    lc = lcm.LCM(args.lcm_url)
    start = time.monotonic()
    next_summary_s = start + 1.0
    csv_handle = None
    csv_writer = None
    if args.csv is not None:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        csv_handle = args.csv.open("w", newline="")
        csv_writer = csv.DictWriter(csv_handle, fieldnames=CSV_FIELDS)
        csv_writer.writeheader()

    callback = MonitorCallback(csv_writer=csv_writer)
    lc.subscribe(args.channel, callback)
    print(f"Listening on {args.lcm_url} channel={args.channel}", flush=True)
    try:
        while True:
            now = time.monotonic()
            if args.duration > 0.0 and now - start >= args.duration:
                break
            timeout = min(0.1, max(0.0, next_summary_s - now))
            rfds, _, _ = select.select([lc.fileno()], [], [], timeout)
            if rfds:
                lc.handle()
            now = time.monotonic()
            if now >= next_summary_s:
                while next_summary_s <= now:
                    next_summary_s += 1.0
                if not args.quiet:
                    subjects = ",".join(
                        f"{name}:id={msg_int_field(msg, 'track_id', 0)}"
                        f"/valid={msg_int_field(msg, 'valid', 0)}"
                        for name, msg in sorted(callback.last_by_name.items())
                    )
                    print(
                        f"received={callback.received_count} "
                        f"accepted={callback.accepted_count} "
                        f"contract_errors={callback.contract_error_count} "
                        f"last=[{subjects}]",
                        flush=True,
                    )
                if (
                    csv_handle is not None
                    and callback.csv_count > 0
                    and callback.csv_count % 300 == 0
                ):
                    csv_handle.flush()
    finally:
        if csv_handle is not None:
            csv_handle.close()

    print(
        f"received={callback.received_count} "
        f"accepted={callback.accepted_count} "
        f"contract_errors={callback.contract_error_count} "
        f"recorded_ball_rows={callback.csv_count}",
        flush=True,
    )
    return 0 if callback.received_count > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
