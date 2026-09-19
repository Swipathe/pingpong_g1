from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np

from deploy.mocap_bridge.chingmu_table_lcm_bridge import (
    BridgeConfig,
    ChingMuTableLcmBridge,
    _collect_calibration_frames,
    _config_from_args,
    _describe_pelvis_orientation,
    _describe_table,
    _load_runtime_pelvis_orientation,
    _operation_mode,
    calibrate_from_frames,
    calibrate_pelvis_orientation_from_frames,
    emit_messages,
    load_table_frame,
    save_pelvis_orientation_calibration,
    save_table_frame,
)
from deploy.mocap_bridge.vicon_sdk_client import ViconSdkClient


ROOT = Path(__file__).resolve().parents[2]


class ViconTableLcmBridge(ChingMuTableLcmBridge):
    """Vicon entrypoint using the same table/ball/LCM logic as ChingMu."""


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Adapt Vicon DataStream unlabeled markers to RobotBridge2's "
            "existing vicon_state_data contract using the ChingMu bridge path"
        )
    )
    parser.add_argument(
        "--helper",
        type=Path,
        default=ROOT / "deploy" / "mocap_bridge" / "bin" / "vicon_frame_stream",
        help="Path to the thin Vicon SDK JSON frame helper.",
    )
    parser.add_argument("--host", default="localhost:801")
    parser.add_argument(
        "--tracker-name",
        default=None,
        help=(
            "Vicon Tracker rigid body/object name as exposed by DataStream. "
            "Defaults to --base-subject when omitted."
        ),
    )
    parser.add_argument(
        "--base-subject",
        default="G2Pelvis",
        help=(
            "RobotBridge2 output base message name. real_world.py currently "
            "expects G2Pelvis."
        ),
    )
    parser.add_argument("--table-calib", type=Path)
    parser.add_argument(
        "--pelvis-orientation-calib",
        type=Path,
        help=(
            "Saved Vicon rigid-to-MuJoCo-pelvis orientation calibration JSON; "
            "required for normal bridge runtime."
        ),
    )
    parser.add_argument(
        "--save-pelvis-orientation-calib",
        type=Path,
        help=(
            "Collect one aligned-pose calibration, atomically save the pelvis "
            "orientation JSON, and exit."
        ),
    )
    parser.add_argument("--save-table-calib", type=Path)
    parser.add_argument("--calib-sec", type=float, default=2.0)
    parser.add_argument(
        "--pelvis-calib-sec",
        type=float,
        default=2.0,
        help="Pelvis orientation aligned-pose collection duration.",
    )
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--print-hz", type=float, default=10.0)
    parser.add_argument("--table-length", type=float, default=2.730738)
    parser.add_argument("--table-width", type=float, default=1.512451)
    parser.add_argument("--table-height", type=float, default=0.760000)
    parser.add_argument("--source-rate-hz", type=float, default=300.0)
    parser.add_argument("--corner-exclusion-radius-mm", type=float, default=50.0)
    parser.add_argument(
        "--lcm-url",
        default="udpm://239.255.76.67:7667?ttl=255",
    )
    parser.add_argument("--channel", default="vicon_state_data_v2")
    parser.add_argument(
        "--ball-track-association-radius-m",
        type=float,
        default=0.35,
    )
    parser.add_argument(
        "--ball-track-end-timeout-s",
        type=float,
        default=0.25,
    )
    parser.set_defaults(publish=False)
    parser.add_argument("--publish", dest="publish", action="store_true")
    parser.add_argument("--no-publish", dest="publish", action="store_false")
    return parser


def _validate_numeric_arguments(args, operation_mode: str) -> None:
    for argument_name, value in (
        ("--source-rate-hz", args.source_rate_hz),
        ("--print-hz", args.print_hz),
        ("--table-length", args.table_length),
        ("--table-width", args.table_width),
        ("--table-height", args.table_height),
    ):
        if not np.isfinite(value) or value <= 0.0:
            raise ValueError(f"{argument_name} must be finite and positive")

    if args.table_calib is None:
        if not np.isfinite(args.calib_sec) or args.calib_sec <= 0.0:
            raise ValueError("--calib-sec must be finite and positive")

    if operation_mode == "pelvis_calibration":
        if (
            not np.isfinite(args.pelvis_calib_sec)
            or args.pelvis_calib_sec <= 0.0
        ):
            raise ValueError("--pelvis-calib-sec must be finite and positive")

    if operation_mode == "runtime":
        if not np.isfinite(args.duration) or args.duration < 0.0:
            raise ValueError("--duration must be finite and nonnegative")


def main(argv=None) -> int:
    args = build_arg_parser().parse_args(argv)
    operation_mode = _operation_mode(args)
    _validate_numeric_arguments(args, operation_mode)
    config: BridgeConfig = _config_from_args(args)

    client = ViconSdkClient(
        args.helper,
        server_ip=args.host,
        body_name=args.tracker_name or config.base_subject,
        source_rate_hz=config.source_rate_hz,
    )
    try:
        print(
            f"Connecting to Vicon@{args.host} through {args.helper} ...",
            flush=True,
        )
        client.start()
        print(
            f"Resolved tracker={args.tracker_name or config.base_subject}; "
            f"publishing base={config.base_subject}; "
            f"source rate={config.source_rate_hz:.1f} Hz",
            flush=True,
        )

        if args.table_calib is None:
            print(
                f"Calibrating table from stationary unlabeled markers for "
                f"{args.calib_sec:.2f} s ...",
                flush=True,
            )
            frames = _collect_calibration_frames(client, args.calib_sec)
            calibration = calibrate_from_frames(frames, config)
            table = calibration.table_frame
            if args.save_table_calib is not None:
                save_table_frame(args.save_table_calib, table, config)
                print(
                    f"Saved table calibration to {args.save_table_calib}",
                    flush=True,
                )
        else:
            table = load_table_frame(args.table_calib, config)
            print(f"Loaded table calibration from {args.table_calib}", flush=True)
        _describe_table(table, config)

        if operation_mode == "table_calibration":
            print(
                "Saved table calibration; exiting before pelvis orientation "
                "is required.",
                flush=True,
            )
            return 0

        if operation_mode == "pelvis_calibration":
            print(
                "Pelvis aligned-pose calibration: confirm pelvis "
                "+X/+Y/+Z are parallel to table-world +X/+Y/+Z. "
                f"Collecting for {args.pelvis_calib_sec:.2f} s ...",
                flush=True,
            )
            frames = _collect_calibration_frames(
                client,
                args.pelvis_calib_sec,
                argument_name="--pelvis-calib-sec",
            )
            pelvis_orientation_calibration = (
                calibrate_pelvis_orientation_from_frames(
                    frames,
                    table,
                    config,
                )
            )
            _describe_pelvis_orientation(pelvis_orientation_calibration)
            save_pelvis_orientation_calibration(
                args.save_pelvis_orientation_calib,
                pelvis_orientation_calibration,
                config,
            )
            print(
                "Saved pelvis orientation calibration to "
                f"{args.save_pelvis_orientation_calib}",
                flush=True,
            )
            return 0

        pelvis_orientation_calibration = _load_runtime_pelvis_orientation(
            args,
            config,
        )
        print(
            "Loaded pelvis orientation calibration from "
            f"{args.pelvis_orientation_calib}",
            flush=True,
        )
        _describe_pelvis_orientation(pelvis_orientation_calibration)

        lc_client = None
        if args.publish:
            import lcm

            lc_client = lcm.LCM(config.lcm_url)
        print(
            f"{'Publishing' if args.publish else 'Monitoring only'} "
            f"channel={config.channel} via {config.lcm_url}",
            flush=True,
        )

        bridge = ViconTableLcmBridge(
            config=config,
            table_frame=table,
            pelvis_orientation_calibration=pelvis_orientation_calibration,
        )
        start = time.monotonic()
        last_print = start - 10.0
        frame_count = 0
        while True:
            now = time.monotonic()
            elapsed = now - start
            if args.duration > 0.0 and elapsed >= args.duration:
                break
            frame = client.next_frame(timeout_s=0.1)
            if frame is None:
                continue
            frame_count += 1
            messages = bridge.process_frame(frame)
            emit_messages(lc_client, messages, config, args.publish)

            print_period = 1.0 / max(args.print_hz, 1.0e-6)
            if now - last_print >= print_period:
                by_name = {message.name: message for message in messages}
                base = by_name.get(config.base_subject)
                ball = by_name.get("ball")
                base_text = (
                    "missing"
                    if base is None
                    else np.array2string(
                        np.asarray(base.pos_vicon),
                        precision=4,
                        suppress_small=True,
                    )
                )
                ball_text = (
                    "missing"
                    if ball is None
                    else np.array2string(
                        np.asarray(ball.pos_vicon),
                        precision=4,
                        suppress_small=True,
                    )
                )
                rate = frame_count / max(elapsed, 1.0e-6)
                print(
                    f"frame={frame.frame_number} hz={rate:.1f} "
                    f"dropped={client.dropped_frame_count} "
                    f"body_markers={len(frame.body_markers_mm)} "
                    f"unlabeled={len(frame.unlabeled_markers_mm)} "
                    f"{config.base_subject}={base_text} ball={ball_text}",
                    flush=True,
                )
                last_print = now
    except KeyboardInterrupt:
        print("Stopped by user", flush=True)
        if operation_mode in ("table_calibration", "pelvis_calibration"):
            return 130
    finally:
        client.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
