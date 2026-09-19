#!/usr/bin/env python3
import argparse
import csv
import select
import sys
import time
from pathlib import Path

import lcm
import numpy as np

BRIDGE_DIR = Path(__file__).resolve().parents[2]
for path in (BRIDGE_DIR, BRIDGE_DIR / "deploy"):
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.insert(0, path_str)

from unitree_sdk2.lcm_types.transformation_t import transformation_t


def msg_int_field(msg, name: str, default: int = 0) -> int:
    try:
        return int(getattr(msg, name))
    except (AttributeError, TypeError, ValueError):
        return default


def msg_float_field(msg, name: str):
    try:
        value = float(getattr(msg, name))
    except (AttributeError, TypeError, ValueError):
        return None
    return value if np.isfinite(value) else None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--lcm-url", default="udpm://239.255.76.67:7667?ttl=255")
    parser.add_argument("--channel", default="vicon_state_data")
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--csv", type=Path, default=None, help="Write selected ball LCM samples to CSV.")
    parser.add_argument("--base-name", default="G1Pelvis")
    parser.add_argument("--ball-name", default="ball")
    parser.add_argument("--quiet", action="store_true", help="Suppress per-message console output.")
    args = parser.parse_args()

    lc = lcm.LCM(args.lcm_url)
    start = time.time()
    count = 0
    csv_count = 0
    latest_base = None
    csv_handle = None
    csv_writer = None
    csv_fields = [
        "host_time_s",
        "elapsed_s",
        "row_index",
        "frame_number",
        "vicon_frame_number",
        "vicon_time_s",
        "publish_time_us",
        "channel",
        "name",
        "ball_valid",
        "ball_occluded",
        "ball_x_m",
        "ball_y_m",
        "ball_z_m",
        "ball_qx",
        "ball_qy",
        "ball_qz",
        "ball_qw",
        "base_valid",
        "base_x_m",
        "base_y_m",
        "base_z_m",
        "base_qx",
        "base_qy",
        "base_qz",
        "base_qw",
    ]
    if args.csv is not None:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        csv_handle = args.csv.open("w", newline="")
        csv_writer = csv.DictWriter(csv_handle, fieldnames=csv_fields)
        csv_writer.writeheader()

    def handler(channel, data):
        nonlocal count, csv_count, latest_base
        msg = transformation_t.decode(data)
        pos = np.asarray(msg.pos_vicon, dtype=np.float64)
        quat = np.asarray(msg.quat_vicon, dtype=np.float64)
        now = time.time()
        elapsed = now - start
        count += 1
        name = str(msg.name)
        if name == args.base_name:
            latest_base = (pos.copy(), quat.copy())

        if csv_writer is not None and name == args.ball_name:
            base_pos = None
            base_quat = None
            if latest_base is not None:
                base_pos, base_quat = latest_base
            vicon_frame_number = msg_int_field(msg, "vicon_frame_number", 0)
            vicon_time_s = msg_float_field(msg, "vicon_time_s")
            if vicon_time_s is not None and vicon_time_s <= 0.0:
                vicon_time_s = None
            publish_time_us = msg_int_field(msg, "publish_time_us", 0)
            frame_number = vicon_frame_number if vicon_frame_number > 0 else csv_count
            row = {
                "host_time_s": f"{now:.9f}",
                "elapsed_s": f"{elapsed:.9f}",
                "row_index": csv_count,
                "frame_number": frame_number,
                "vicon_frame_number": "" if vicon_frame_number <= 0 else vicon_frame_number,
                "vicon_time_s": "" if vicon_time_s is None else f"{vicon_time_s:.9f}",
                "publish_time_us": "" if publish_time_us <= 0 else publish_time_us,
                "channel": channel,
                "name": name,
                "ball_valid": msg_int_field(msg, "valid", 1),
                "ball_occluded": msg_int_field(msg, "occluded", 0),
                "ball_x_m": f"{pos[0]:.9f}",
                "ball_y_m": f"{pos[1]:.9f}",
                "ball_z_m": f"{pos[2]:.9f}",
                "ball_qx": f"{quat[0]:.9f}",
                "ball_qy": f"{quat[1]:.9f}",
                "ball_qz": f"{quat[2]:.9f}",
                "ball_qw": f"{quat[3]:.9f}",
                "base_valid": int(base_pos is not None),
                "base_x_m": "" if base_pos is None else f"{base_pos[0]:.9f}",
                "base_y_m": "" if base_pos is None else f"{base_pos[1]:.9f}",
                "base_z_m": "" if base_pos is None else f"{base_pos[2]:.9f}",
                "base_qx": "" if base_quat is None else f"{base_quat[0]:.9f}",
                "base_qy": "" if base_quat is None else f"{base_quat[1]:.9f}",
                "base_qz": "" if base_quat is None else f"{base_quat[2]:.9f}",
                "base_qw": "" if base_quat is None else f"{base_quat[3]:.9f}",
            }
            csv_writer.writerow(row)
            csv_count += 1
            if csv_handle is not None and csv_count % 300 == 0:
                csv_handle.flush()

        if not args.quiet:
            frame_text = msg_int_field(msg, "vicon_frame_number", 0)
            vicon_time_s = msg_float_field(msg, "vicon_time_s")
            print(
                f"{elapsed:8.3f}s channel={channel} name={name} "
                f"frame={frame_text} "
                f"vicon_time={0.0 if vicon_time_s is None else vicon_time_s:.6f} "
                f"pos=[{pos[0]: .4f}, {pos[1]: .4f}, {pos[2]: .4f}] "
                f"quat=[{quat[0]: .4f}, {quat[1]: .4f}, {quat[2]: .4f}, {quat[3]: .4f}]",
                flush=True,
            )

    lc.subscribe(args.channel, handler)
    print(f"Listening on {args.lcm_url} channel={args.channel}", flush=True)
    try:
        while True:
            if args.duration > 0.0 and time.time() - start >= args.duration:
                break
            timeout = 0.1
            rfds, _, _ = select.select([lc.fileno()], [], [], timeout)
            if rfds:
                lc.handle()
    finally:
        if csv_handle is not None:
            csv_handle.close()

    print(f"received={count} recorded_ball_rows={csv_count}", flush=True)
    return 0 if count > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
