#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np

ROBOTBRIDGE_DIR = Path(__file__).resolve().parents[2]
DEPLOY_DIR = ROBOTBRIDGE_DIR / "deploy"
for path in (ROBOTBRIDGE_DIR, DEPLOY_DIR):
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.insert(0, path_str)

from mocap_bridge.visualize_ball_trajectory import (
    _float_or_none,
    _row_gap_time,
    _row_position_m,
    _row_time,
    _truthy_field,
)


@dataclass(frozen=True)
class RecordedBallRow:
    source_path: Path
    source_row_index: int
    timestamp: float
    gap_time: float
    position: np.ndarray
    row: dict
    fieldnames: Tuple[str, ...]


@dataclass(frozen=True)
class SelectedSegment:
    rows: Sequence[RecordedBallRow]
    start_time: float
    end_time: float


def load_recorded_ball_rows(
    paths: Sequence[Path],
    *,
    sample_rate_hz: float = 300.0,
    ball_name: str = "ball",
    time_source: str = "frame",
) -> List[RecordedBallRow]:
    rows: List[RecordedBallRow] = []
    for path in paths:
        source_path = Path(path)
        with source_path.open(newline="") as handle:
            reader = csv.DictReader(handle)
            fieldnames = tuple(reader.fieldnames or ())
            for row_index, row in enumerate(reader, start=2):
                if row.get("name") not in (None, "", ball_name):
                    continue
                if not _truthy_field(row.get("ball_valid")):
                    continue
                if row.get("ball_occluded") is not None and _truthy_field(row.get("ball_occluded")):
                    continue
                timestamp = _row_time(row, row_index - 2, sample_rate_hz, time_source=time_source)
                gap_time = _row_gap_time(row, row_index - 2, sample_rate_hz)
                position = _row_position_m(row)
                if timestamp is None or gap_time is None or position is None:
                    continue
                if not np.isfinite(position).all():
                    continue
                rows.append(
                    RecordedBallRow(
                        source_path=source_path,
                        source_row_index=row_index,
                        timestamp=float(timestamp),
                        gap_time=float(gap_time),
                        position=position,
                        row=dict(row),
                        fieldnames=fieldnames,
                    )
                )
    rows.sort(key=lambda item: (str(item.source_path), item.gap_time, item.source_row_index))
    return rows


def _clean_segment_value(row: RecordedBallRow) -> Optional[int]:
    value = _float_or_none(row.row.get("clean_segment_id"))
    return None if value is None else int(value)


def split_candidates(
    rows: Sequence[RecordedBallRow],
    *,
    max_gap_s: float = 0.25,
    min_samples: int = 20,
) -> List[List[RecordedBallRow]]:
    candidates: List[List[RecordedBallRow]] = []
    current: List[RecordedBallRow] = []
    previous: Optional[RecordedBallRow] = None
    previous_clean_segment: Optional[int] = None

    for row in rows:
        clean_segment = _clean_segment_value(row)
        starts_new = False
        if previous is None:
            starts_new = True
        elif row.source_path != previous.source_path:
            starts_new = True
        elif clean_segment is not None and clean_segment != previous_clean_segment:
            starts_new = True
        elif row.gap_time - previous.gap_time > max_gap_s:
            starts_new = True

        if starts_new:
            if len(current) >= min_samples:
                candidates.append(current)
            current = []
        current.append(row)
        previous = row
        previous_clean_segment = clean_segment

    if len(current) >= min_samples:
        candidates.append(current)
    return candidates


def selected_rows(segment: SelectedSegment) -> List[RecordedBallRow]:
    start = min(float(segment.start_time), float(segment.end_time))
    end = max(float(segment.start_time), float(segment.end_time))
    return [row for row in segment.rows if start <= row.timestamp <= end]


def _output_fieldnames(segments: Sequence[SelectedSegment]) -> List[str]:
    fields: List[str] = []
    seen = set()
    for segment in segments:
        for row in segment.rows:
            for field in row.fieldnames:
                if field not in seen and field not in {"clean_segment_id", "source_file", "source_row_index"}:
                    fields.append(field)
                    seen.add(field)
    for field in ("clean_segment_id", "source_file", "source_row_index"):
        if field not in seen:
            fields.append(field)
            seen.add(field)
    return fields


def export_selected_segments(segments: Sequence[SelectedSegment], output: Path) -> int:
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fields = _output_fieldnames(segments)
    written = 0
    with output.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for segment_index, segment in enumerate(segments, start=1):
            for source_row in selected_rows(segment):
                row = {field: source_row.row.get(field, "") for field in fields}
                row["clean_segment_id"] = segment_index
                row["source_file"] = source_row.source_path.name
                row["source_row_index"] = source_row.source_row_index
                writer.writerow(row)
                written += 1
    return written


def _default_output_path(paths: Sequence[Path]) -> Path:
    if len(paths) == 1:
        path = Path(paths[0])
        return path.with_name(f"{path.stem}_manual_clean.csv")
    first = Path(paths[0])
    return first.with_name("manual_clean_segments.csv")


def _candidate_title(candidate: Sequence[RecordedBallRow], index: int, total: int) -> str:
    duration = candidate[-1].timestamp - candidate[0].timestamp if candidate else 0.0
    return (
        f"candidate {index + 1}/{total}  "
        f"{candidate[0].source_path.name} rows={len(candidate)} duration={duration:.3f}s"
    )


def nearest_display_point_index(
    points_xy: Sequence[Tuple[float, float]],
    *,
    click_xy: Tuple[float, float],
    max_distance_px: float = 24.0,
) -> Optional[int]:
    if not points_xy:
        return None
    points = np.asarray(points_xy, dtype=np.float64)
    click = np.asarray(click_xy, dtype=np.float64)
    distances = np.linalg.norm(points - click.reshape(1, 2), axis=1)
    best = int(np.argmin(distances))
    return best if float(distances[best]) <= float(max_distance_px) else None


class SegmentSelectorApp:
    def __init__(
        self,
        candidates: Sequence[Sequence[RecordedBallRow]],
        *,
        output: Path,
        table_height: float = 0.76,
        table_length: float = 2.74,
        table_width: float = 1.525,
        table_center_x: float = 1.37,
        table_center_y: float = 0.0,
    ):
        self.candidates = [list(candidate) for candidate in candidates]
        self.output = Path(output)
        self.table_height = float(table_height)
        self.table_length = float(table_length)
        self.table_width = float(table_width)
        self.table_center_x = float(table_center_x)
        self.table_center_y = float(table_center_y)
        self.index = 0
        self.pick_times: List[float] = []
        self.accepted: List[SelectedSegment] = []
        self._closed = False

        import matplotlib.pyplot as plt

        self.plt = plt
        self.fig = plt.figure(figsize=(12, 9))
        self.ax_3d = self.fig.add_subplot(1, 1, 1, projection="3d")
        self.status_text = self.fig.text(0.01, 0.01, "", fontsize=9, family="monospace")
        self.fig.canvas.mpl_connect("button_press_event", self._on_click)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self._draw()

    def show(self) -> int:
        self.plt.show()
        if not self._closed and self.accepted:
            return self.save()
        return 0

    def save(self) -> int:
        written = export_selected_segments(self.accepted, self.output)
        print(f"saved={self.output} segments={len(self.accepted)} rows={written}", flush=True)
        return written

    def _candidate(self) -> List[RecordedBallRow]:
        return self.candidates[self.index]

    def _draw_table(self, ax) -> None:
        x0 = self.table_center_x - 0.5 * self.table_length
        x1 = self.table_center_x + 0.5 * self.table_length
        y0 = self.table_center_y - 0.5 * self.table_width
        y1 = self.table_center_y + 0.5 * self.table_width
        ax.plot([x0, x1, x1, x0, x0], [y0, y0, y1, y1, y0], color="0.25", linewidth=1.0)

    def _draw_table_3d(self) -> None:
        x0 = self.table_center_x - 0.5 * self.table_length
        x1 = self.table_center_x + 0.5 * self.table_length
        y0 = self.table_center_y - 0.5 * self.table_width
        y1 = self.table_center_y + 0.5 * self.table_width
        z = self.table_height
        self.ax_3d.plot([x0, x1, x1, x0, x0], [y0, y0, y1, y1, y0], [z, z, z, z, z], color="0.25")

    def _draw(self) -> None:
        self.ax_3d.clear()

        candidate = self._candidate()
        times = np.asarray([row.timestamp for row in candidate], dtype=np.float64)
        positions = np.asarray([row.position for row in candidate], dtype=np.float64)

        self.ax_3d.plot(positions[:, 0], positions[:, 1], positions[:, 2], color="#1f77b4", linewidth=1.8)
        self._draw_table_3d()
        self.ax_3d.set_xlabel("x m")
        self.ax_3d.set_ylabel("y m")
        self.ax_3d.set_zlabel("z m")
        self.ax_3d.set_title("click trajectory start/end directly")

        for pick_time in self.pick_times:
            pick_index = int(np.argmin(np.abs(times - pick_time)))
            pick = positions[pick_index]
            self.ax_3d.scatter([pick[0]], [pick[1]], [pick[2]], s=90, color="#9467bd", marker="*", zorder=5)
        if len(self.pick_times) == 2:
            start, end = sorted(self.pick_times)
            mask = (times >= start) & (times <= end)
            picked = positions[mask]
            if picked.size:
                self.ax_3d.plot(picked[:, 0], picked[:, 1], picked[:, 2], color="#2ca02c", linewidth=4.0)
                self.ax_3d.scatter(picked[:, 0], picked[:, 1], picked[:, 2], s=14, color="#2ca02c")

        self.fig.suptitle(_candidate_title(candidate, self.index, len(self.candidates)))
        self.status_text.set_text(
            "click two trajectory points in 3D | drag/scroll toolbar can rotate/zoom | a accept | r reset | n next | p prev | s save | q save+quit\n"
            f"accepted={len(self.accepted)} output={self.output} picks={len(self.pick_times)}/2"
        )
        self.fig.tight_layout(rect=(0.0, 0.04, 1.0, 0.95))
        self.fig.canvas.draw_idle()

    def _projected_display_points(self, positions: np.ndarray) -> List[Tuple[float, float]]:
        from mpl_toolkits.mplot3d import proj3d

        xs, ys, _ = proj3d.proj_transform(positions[:, 0], positions[:, 1], positions[:, 2], self.ax_3d.get_proj())
        projected = np.column_stack([xs, ys])
        display = self.ax_3d.transData.transform(projected)
        return [(float(x), float(y)) for x, y in display]

    def _on_click(self, event) -> None:
        if event.inaxes != self.ax_3d or event.x is None or event.y is None:
            return
        candidate = self._candidate()
        positions = np.asarray([row.position for row in candidate], dtype=np.float64)
        point_index = nearest_display_point_index(
            self._projected_display_points(positions),
            click_xy=(float(event.x), float(event.y)),
            max_distance_px=28.0,
        )
        if point_index is None:
            print("click closer to the trajectory line", flush=True)
            return
        pick_time = candidate[point_index].timestamp
        if len(self.pick_times) >= 2:
            self.pick_times = []
        self.pick_times.append(pick_time)
        self._draw()

    def _on_key(self, event) -> None:
        key = str(event.key or "").lower()
        if key == "a":
            if len(self.pick_times) != 2:
                print("select two points before pressing a", flush=True)
                return
            self.accepted.append(
                SelectedSegment(
                    rows=self._candidate(),
                    start_time=min(self.pick_times),
                    end_time=max(self.pick_times),
                )
            )
            self.pick_times = []
            self.index = min(self.index + 1, len(self.candidates) - 1)
        elif key == "r":
            self.pick_times = []
        elif key == "n":
            self.pick_times = []
            self.index = min(self.index + 1, len(self.candidates) - 1)
        elif key == "p":
            self.pick_times = []
            self.index = max(self.index - 1, 0)
        elif key == "s":
            self.save()
        elif key == "q":
            if self.accepted:
                self.save()
            self._closed = True
            self.plt.close(self.fig)
            return
        else:
            return
        self._draw()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Manually select clean ball trajectory segments from recorded CSV files.")
    parser.add_argument("csv", nargs="+", type=Path, help="Recorded ball CSV file(s).")
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--sample-rate-hz", type=float, default=300.0)
    parser.add_argument("--time-source", choices=("frame", "elapsed", "host", "auto"), default="frame")
    parser.add_argument("--ball-name", default="ball")
    parser.add_argument("--max-gap-s", type=float, default=0.25)
    parser.add_argument("--min-candidate-samples", type=int, default=20)
    parser.add_argument("--table-height", type=float, default=0.76)
    parser.add_argument("--table-length", type=float, default=2.74)
    parser.add_argument("--table-width", type=float, default=1.525)
    parser.add_argument("--table-center-x", type=float, default=1.37)
    parser.add_argument("--table-center-y", type=float, default=0.0)
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    rows = load_recorded_ball_rows(
        args.csv,
        sample_rate_hz=args.sample_rate_hz,
        ball_name=args.ball_name,
        time_source=args.time_source,
    )
    if not rows:
        print("error: no valid ball rows found")
        return 1
    candidates = split_candidates(rows, max_gap_s=args.max_gap_s, min_samples=args.min_candidate_samples)
    if not candidates:
        print("error: no candidate trajectory has enough samples")
        return 1
    output = args.output if args.output is not None else _default_output_path(args.csv)
    print(f"loaded_ball_rows={len(rows)} candidates={len(candidates)} output={output}")
    app = SegmentSelectorApp(
        candidates,
        output=output,
        table_height=args.table_height,
        table_length=args.table_length,
        table_width=args.table_width,
        table_center_x=args.table_center_x,
        table_center_y=args.table_center_y,
    )
    app.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
