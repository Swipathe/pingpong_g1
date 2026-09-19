#!/usr/bin/env python3
"""Probe Vicon Nexus online DataStream data.

Stage 1 of the mocap integration:
connect to Nexus, list subjects/markers/segments, and optionally stream a
specified ball marker and robot base segment to the console or a CSV file.
This script intentionally does not publish LCM or drive RobotBridge.
"""

from __future__ import annotations

import argparse
import csv
import math
import signal
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


def _import_vicon_datastream():
    try:
        from vicon_dssdk import ViconDataStream  # type: ignore
    except Exception as exc:  # pragma: no cover - depends on external SDK
        raise RuntimeError(
            "Could not import Vicon DataStream Python SDK (`vicon_dssdk`). "
            "Install the official Vicon DataStream SDK on this machine and "
            "ensure its Python package is on PYTHONPATH/LD_LIBRARY_PATH."
        ) from exc
    return ViconDataStream


def _first(value: Any) -> Any:
    if isinstance(value, tuple) and value:
        return value[0]
    if isinstance(value, list) and value and not isinstance(value[0], (int, float)):
        return value[0]
    return value


def _field(output: Any, name: str, index: int | None = None, default: Any = None) -> Any:
    if hasattr(output, name):
        return getattr(output, name)
    if isinstance(output, dict):
        return output.get(name, default)
    if index is not None and isinstance(output, (tuple, list)) and len(output) > index:
        return output[index]
    return default


def _vector3(output: Any, field_name: str = "Translation") -> tuple[float, float, float] | None:
    vector = _field(output, field_name, 0)
    if vector is None:
        vector = _first(output)
    try:
        values = tuple(float(x) for x in vector[:3])
    except Exception:
        return None
    if len(values) != 3 or not all(math.isfinite(x) for x in values):
        return None
    return values


def _quat4(output: Any) -> tuple[float, float, float, float] | None:
    vector = _field(output, "Rotation", 0)
    if vector is None:
        vector = _first(output)
    try:
        values = tuple(float(x) for x in vector[:4])
    except Exception:
        return None
    if len(values) != 4 or not all(math.isfinite(x) for x in values):
        return None
    return values


def _occluded(output: Any) -> bool | None:
    value = _field(output, "Occluded", 1)
    if value is None:
        return None
    return bool(value)


def _result_ok(output: Any) -> bool:
    if output is None:
        return True
    if isinstance(output, bool):
        return output
    result = _field(output, "Result")
    if result is None:
        return bool(output)
    return str(result).lower().endswith("success")


def _names(values: Iterable[Any]) -> list[str]:
    result: list[str] = []
    for value in values:
        if isinstance(value, (tuple, list)):
            value = value[0] if value else ""
        result.append(str(value))
    return result


@dataclass
class MarkerSample:
    translation_mm: tuple[float, float, float] | None
    occluded: bool | None

    @property
    def valid(self) -> bool:
        return self.translation_mm is not None and self.occluded is not True


@dataclass
class SegmentSample:
    translation_mm: tuple[float, float, float] | None
    rotation_quat: tuple[float, float, float, float] | None
    occluded: bool | None

    @property
    def valid(self) -> bool:
        return self.translation_mm is not None and self.occluded is not True


class NexusProbe:
    def __init__(self, host: str, stream_mode: str):
        ViconDataStream = _import_vicon_datastream()
        self._sdk = ViconDataStream
        self.client = ViconDataStream.Client()
        self.host = host
        self.stream_mode = stream_mode

    def connect(self) -> None:
        output = self.client.Connect(self.host)
        if not _result_ok(output):
            raise RuntimeError(f"Failed to connect to Vicon DataStream server `{self.host}`: {output!r}")

        self.client.EnableMarkerData()
        self.client.EnableSegmentData()

        if hasattr(self.client, "EnableUnlabeledMarkerData"):
            try:
                self.client.EnableUnlabeledMarkerData()
            except Exception:
                pass

        if hasattr(self.client, "SetStreamMode"):
            mode_name = self.stream_mode.strip().lower()
            mode_enum = getattr(self._sdk.Client, "StreamMode", None)
            mode_value = None
            if mode_enum is not None:
                if mode_name == "pull":
                    mode_value = getattr(mode_enum, "EClientPull", None)
                elif mode_name == "pre-fetch":
                    mode_value = getattr(mode_enum, "EClientPullPreFetch", None)
                elif mode_name == "server-push":
                    mode_value = getattr(mode_enum, "EServerPush", None)
            if mode_value is not None:
                self.client.SetStreamMode(mode_value)

    def disconnect(self) -> None:
        try:
            self.client.Disconnect()
        except Exception:
            pass

    def wait_frame(self, timeout_s: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.client.GetFrame():
                return True
            time.sleep(0.001)
        return False

    def list_stream(self) -> None:
        if not self.wait_frame():
            raise RuntimeError("Timed out waiting for the first Vicon frame.")

        subjects = _names(self.client.GetSubjectNames())
        print(f"Subjects ({len(subjects)}):")
        for subject in subjects:
            print(f"  {subject}")
            try:
                root_segment = self.client.GetSubjectRootSegmentName(subject)
                root_name = _first(root_segment)
                print(f"    root segment: {root_name}")
            except Exception as exc:
                print(f"    root segment: unavailable ({exc})")

            try:
                segments = _names(self.client.GetSegmentNames(subject))
                print(f"    segments ({len(segments)}): {', '.join(segments) if segments else '-'}")
            except Exception as exc:
                print(f"    segments: unavailable ({exc})")

            try:
                markers = _names(self.client.GetMarkerNames(subject))
                print(f"    markers ({len(markers)}): {', '.join(markers) if markers else '-'}")
            except Exception as exc:
                print(f"    markers: unavailable ({exc})")

        if hasattr(self.client, "GetUnlabeledMarkerCount"):
            try:
                count = self.client.GetUnlabeledMarkerCount()
                count_value = _field(count, "MarkerCount", 0, count)
                print(f"Unlabeled markers: {count_value}")
            except Exception:
                pass

    def frame_number(self) -> int | None:
        if not hasattr(self.client, "GetFrameNumber"):
            return None
        try:
            output = self.client.GetFrameNumber()
            if isinstance(output, (int, float)):
                return int(output)
            value = _field(output, "FrameNumber", 0)
            return None if value is None else int(value)
        except Exception:
            return None

    def frame_rate(self) -> float | None:
        if not hasattr(self.client, "GetFrameRate"):
            return None
        try:
            output = self.client.GetFrameRate()
            if isinstance(output, (int, float)):
                return float(output)
            value = _field(output, "FrameRateHz", 0)
            return None if value is None else float(value)
        except Exception:
            return None

    def marker_sample(self, subject: str, marker: str) -> MarkerSample:
        try:
            output = self.client.GetMarkerGlobalTranslation(subject, marker)
        except Exception:
            return MarkerSample(None, True)
        return MarkerSample(_vector3(output), _occluded(output))

    def root_segment_name(self, subject: str) -> str:
        output = self.client.GetSubjectRootSegmentName(subject)
        return str(_first(output))

    def segment_sample(self, subject: str, segment: str) -> SegmentSample:
        try:
            trans_output = self.client.GetSegmentGlobalTranslation(subject, segment)
        except Exception:
            trans_output = None
        try:
            rot_output = self.client.GetSegmentGlobalRotationQuaternion(subject, segment)
        except Exception:
            rot_output = None

        translation = _vector3(trans_output) if trans_output is not None else None
        rotation = _quat4(rot_output) if rot_output is not None else None
        occluded = _occluded(trans_output) if trans_output is not None else None
        return SegmentSample(translation, rotation, occluded)


def _format_vec(values: tuple[float, ...] | None, scale: float = 1.0, precision: int = 3) -> str:
    if values is None:
        return "None"
    return "[" + ", ".join(f"{x * scale:.{precision}f}" for x in values) + "]"


def _write_csv_header(writer: csv.writer) -> None:
    writer.writerow(
        [
            "host_time_s",
            "frame_number",
            "ball_valid",
            "ball_occluded",
            "ball_x_mm",
            "ball_y_mm",
            "ball_z_mm",
            "ball_x_m",
            "ball_y_m",
            "ball_z_m",
            "base_valid",
            "base_occluded",
            "base_x_mm",
            "base_y_mm",
            "base_z_mm",
            "base_qx",
            "base_qy",
            "base_qz",
            "base_qw",
        ]
    )


def _write_csv_row(
    writer: csv.writer,
    *,
    host_time_s: float,
    frame_number: int | None,
    ball: MarkerSample | None,
    base: SegmentSample | None,
) -> None:
    ball_pos = ball.translation_mm if ball is not None else None
    base_pos = base.translation_mm if base is not None else None
    base_quat = base.rotation_quat if base is not None else None
    writer.writerow(
        [
            f"{host_time_s:.9f}",
            "" if frame_number is None else frame_number,
            "" if ball is None else int(ball.valid),
            "" if ball is None or ball.occluded is None else int(ball.occluded),
            *(("" if ball_pos is None else f"{x:.6f}") for x in (ball_pos or (None, None, None))),
            *(("" if ball_pos is None else f"{0.001 * x:.9f}") for x in (ball_pos or (None, None, None))),
            "" if base is None else int(base.valid),
            "" if base is None or base.occluded is None else int(base.occluded),
            *(("" if base_pos is None else f"{x:.6f}") for x in (base_pos or (None, None, None))),
            *(("" if base_quat is None else f"{x:.9f}") for x in (base_quat or (None, None, None, None))),
        ]
    )


def run_stream(args: argparse.Namespace) -> None:
    probe = NexusProbe(host=args.host, stream_mode=args.stream_mode)
    probe.connect()
    print(f"Connected to Vicon DataStream at {args.host}")
    frame_rate = probe.frame_rate()
    if frame_rate is not None:
        print(f"Reported frame rate: {frame_rate:.3f} Hz")

    if args.list:
        probe.list_stream()
        probe.disconnect()
        return

    if not args.ball_subject or not args.ball_marker:
        raise ValueError("--ball-subject and --ball-marker are required unless --list is used.")

    base_segment = args.base_segment
    if args.base_subject and not base_segment:
        base_segment = probe.root_segment_name(args.base_subject)
        print(f"Using root segment for base subject `{args.base_subject}`: {base_segment}")

    csv_file = None
    writer = None
    if args.csv:
        csv_path = Path(args.csv).expanduser()
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        csv_file = csv_path.open("w", newline="")
        writer = csv.writer(csv_file)
        _write_csv_header(writer)
        print(f"Writing samples to {csv_path}")

    stop = False

    def _handle_signal(signum, frame):  # noqa: ARG001
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    start = time.monotonic()
    last_print = 0.0
    frames = 0
    try:
        while not stop:
            if args.duration > 0.0 and time.monotonic() - start >= args.duration:
                break
            if not probe.wait_frame(timeout_s=args.frame_timeout):
                print("Timed out waiting for Vicon frame.", file=sys.stderr)
                continue

            host_time_s = time.time()
            frame_number = probe.frame_number()
            ball = probe.marker_sample(args.ball_subject, args.ball_marker)
            base = None
            if args.base_subject and base_segment:
                base = probe.segment_sample(args.base_subject, base_segment)
            frames += 1

            if writer is not None:
                _write_csv_row(
                    writer,
                    host_time_s=host_time_s,
                    frame_number=frame_number,
                    ball=ball,
                    base=base,
                )

            now = time.monotonic()
            if now - last_print >= 1.0 / max(args.print_hz, 1.0e-6):
                elapsed = max(now - start, 1.0e-6)
                print(
                    "frame={} stream_hz={:.1f} ball_valid={} ball_m={} base_valid={} base_m={} base_quat={}".format(
                        "-" if frame_number is None else frame_number,
                        frames / elapsed,
                        ball.valid,
                        _format_vec(ball.translation_mm, scale=0.001, precision=4),
                        "-" if base is None else base.valid,
                        "-" if base is None else _format_vec(base.translation_mm, scale=0.001, precision=4),
                        "-" if base is None else _format_vec(base.rotation_quat, precision=4),
                    )
                )
                last_print = now
    finally:
        if csv_file is not None:
            csv_file.close()
        probe.disconnect()
        print("Disconnected from Vicon DataStream.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="localhost:801", help="Vicon DataStream host, e.g. 192.168.1.10:801")
    parser.add_argument(
        "--stream-mode",
        default="pull",
        choices=["pull", "pre-fetch", "server-push"],
        help="Vicon DataStream stream mode.",
    )
    parser.add_argument("--list", action="store_true", help="List subjects, markers, and segments, then exit.")
    parser.add_argument("--ball-subject", default=None, help="Nexus subject containing the ball marker.")
    parser.add_argument("--ball-marker", default=None, help="Ball marker name inside --ball-subject.")
    parser.add_argument("--base-subject", default=None, help="Optional robot base subject name.")
    parser.add_argument("--base-segment", default=None, help="Optional base segment name. Defaults to subject root segment.")
    parser.add_argument("--duration", type=float, default=0.0, help="Run duration in seconds. 0 means until Ctrl-C.")
    parser.add_argument("--print-hz", type=float, default=10.0, help="Console print rate.")
    parser.add_argument("--frame-timeout", type=float, default=2.0, help="Timeout while waiting for each Vicon frame.")
    parser.add_argument("--csv", default=None, help="Optional CSV output path.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        run_stream(args)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
