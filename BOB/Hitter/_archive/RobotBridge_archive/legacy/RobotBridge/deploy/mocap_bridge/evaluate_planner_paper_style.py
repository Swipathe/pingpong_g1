#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence

import numpy as np


@dataclass(frozen=True)
class PlannerEvalRow:
    created_time_s: float
    predicted_hit_time_s: float
    actual_hit_time_s: float
    position_error_cm: float
    time_error_ms: float

    @property
    def actual_lead_time_s(self) -> float:
        return float(self.actual_hit_time_s - self.created_time_s)


def _float_or_none(value) -> Optional[float]:
    if value is None or value == "":
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def read_planner_eval_csv(path: Path) -> List[PlannerEvalRow]:
    rows: List[PlannerEvalRow] = []
    with Path(path).open(newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            created = _float_or_none(row.get("created_time_s"))
            predicted_hit = _float_or_none(row.get("predicted_hit_time_s"))
            actual_hit = _float_or_none(row.get("actual_hit_time_s"))
            position_error = _float_or_none(row.get("position_error_cm"))
            time_error = _float_or_none(row.get("time_error_ms"))
            if None in (created, predicted_hit, actual_hit, position_error, time_error):
                continue
            rows.append(
                PlannerEvalRow(
                    created_time_s=float(created),
                    predicted_hit_time_s=float(predicted_hit),
                    actual_hit_time_s=float(actual_hit),
                    position_error_cm=abs(float(position_error)),
                    time_error_ms=abs(float(time_error)),
                )
            )
    return rows


def group_by_actual_hit(rows: Iterable[PlannerEvalRow], *, precision: int = 6) -> Dict[float, List[PlannerEvalRow]]:
    groups: Dict[float, List[PlannerEvalRow]] = {}
    for row in rows:
        if row.actual_lead_time_s < 0.0:
            continue
        key = round(row.actual_hit_time_s, precision)
        groups.setdefault(key, []).append(row)
    for group_rows in groups.values():
        group_rows.sort(key=lambda item: item.created_time_s)
    return groups


def _mean_std(values: Sequence[float]) -> Dict[str, float]:
    if not values:
        return {
            "count": 0,
            "mean": float("nan"),
            "std": float("nan"),
            "median": float("nan"),
            "p90": float("nan"),
        }
    arr = np.asarray(values, dtype=np.float64)
    return {
        "count": int(arr.shape[0]),
        "mean": float(np.mean(arr)),
        "std": float(np.std(arr, ddof=0)),
        "median": float(np.median(arr)),
        "p90": float(np.percentile(arr, 90)),
    }


def _nearest_row_for_lead(rows: Sequence[PlannerEvalRow], lead_time_s: float) -> Optional[PlannerEvalRow]:
    if not rows:
        return None
    return min(rows, key=lambda item: abs(item.actual_lead_time_s - lead_time_s))


def _summary_for_selected(rows: Sequence[PlannerEvalRow], lead_time_s: float) -> dict:
    position = _mean_std([row.position_error_cm for row in rows])
    timing = _mean_std([row.time_error_ms for row in rows])
    return {
        "lead_time_s": float(lead_time_s),
        "sample_count": int(len(rows)),
        "position_error_mean_cm": position["mean"],
        "position_error_std_cm": position["std"],
        "position_error_median_cm": position["median"],
        "position_error_p90_cm": position["p90"],
        "time_error_mean_ms": timing["mean"],
        "time_error_std_ms": timing["std"],
        "time_error_median_ms": timing["median"],
        "time_error_p90_ms": timing["p90"],
    }


def build_paper_style_summary(
    rows: Sequence[PlannerEvalRow],
    *,
    horizons_s: Sequence[float] = (0.5, 0.3, 0.1),
    bin_width_s: float = 0.05,
    max_lead_s: float = 1.0,
) -> dict:
    groups = group_by_actual_hit(rows)
    event_count = len(groups)
    prediction_count = sum(len(group_rows) for group_rows in groups.values())

    horizons = []
    for horizon in horizons_s:
        selected = []
        for group_rows in groups.values():
            nearest = _nearest_row_for_lead(group_rows, float(horizon))
            if nearest is not None:
                selected.append(nearest)
        horizons.append(_summary_for_selected(selected, float(horizon)))

    if bin_width_s <= 0.0:
        raise ValueError("bin_width_s must be positive.")
    if max_lead_s <= 0.0:
        raise ValueError("max_lead_s must be positive.")
    centers = np.arange(0.0, max_lead_s + 0.5 * bin_width_s, bin_width_s)
    curve = []
    for center in centers:
        selected = []
        for group_rows in groups.values():
            nearest = _nearest_row_for_lead(group_rows, float(center))
            if nearest is None:
                continue
            if abs(nearest.actual_lead_time_s - center) <= 0.5 * bin_width_s:
                selected.append(nearest)
        curve.append(_summary_for_selected(selected, float(center)))

    return {
        "event_count": int(event_count),
        "prediction_count": int(prediction_count),
        "horizons": horizons,
        "curve": curve,
        "method": (
            "Group predictions by actual_hit_time_s. For each trajectory and each requested time before strike, "
            "select the prediction whose actual lead time actual_hit_time_s - created_time_s is nearest. "
            "Report mean/std over trajectories, matching the paper-style prediction error curve."
        ),
    }


def write_summary_csv(summary: dict, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "lead_time_s",
        "sample_count",
        "position_error_mean_cm",
        "position_error_std_cm",
        "position_error_median_cm",
        "position_error_p90_cm",
        "time_error_mean_ms",
        "time_error_std_ms",
        "time_error_median_ms",
        "time_error_p90_ms",
    ]
    with Path(path).open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in summary["curve"]:
            writer.writerow({field: row[field] for field in fields})


def save_paper_style_figure(summary: dict, path: Path, *, racket_radius_cm: float = 7.5, control_step_ms: float = 20.0) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    curve = [row for row in summary["curve"] if row["sample_count"] > 0]
    if not curve:
        raise RuntimeError("No curve samples are available to plot.")
    lead = np.asarray([row["lead_time_s"] for row in curve], dtype=np.float64)
    pos_mean = np.asarray([row["position_error_mean_cm"] for row in curve], dtype=np.float64)
    pos_std = np.asarray([row["position_error_std_cm"] for row in curve], dtype=np.float64)
    time_mean = np.asarray([row["time_error_mean_ms"] for row in curve], dtype=np.float64)
    time_std = np.asarray([row["time_error_std_ms"] for row in curve], dtype=np.float64)

    fig, axes = plt.subplots(2, 1, figsize=(7.0, 6.0), sharex=True)
    axes[0].plot(lead, pos_mean, color="#1f77b4", linewidth=2.0)
    axes[0].fill_between(lead, pos_mean - pos_std, pos_mean + pos_std, color="#1f77b4", alpha=0.20)
    axes[0].axhline(racket_radius_cm, color="#d62728", linestyle="--", linewidth=1.4)
    axes[0].set_ylabel("position error (cm)")
    axes[0].set_title("Planner Prediction Error")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(lead, time_mean, color="#ff7f0e", linewidth=2.0)
    axes[1].fill_between(lead, time_mean - time_std, time_mean + time_std, color="#ff7f0e", alpha=0.20)
    axes[1].axhline(control_step_ms, color="#d62728", linestyle="--", linewidth=1.4)
    axes[1].set_ylabel("strike time error (ms)")
    axes[1].set_xlabel("time before strike (s)")
    axes[1].grid(True, alpha=0.3)

    for ax in axes:
        ax.invert_xaxis()

    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=180)
    plt.close(fig)


def _parse_horizons(value: str) -> List[float]:
    return [float(item.strip()) for item in value.split(",") if item.strip()]


def main() -> int:
    parser = argparse.ArgumentParser(description="Evaluate HITTER planner prediction error using the paper-style protocol.")
    parser.add_argument("eval_csv", type=Path, help="Planner evaluation CSV written by visualize_ball_trajectory.py --eval-csv.")
    parser.add_argument("--horizons-s", default="0.5,0.3,0.1", help="Comma-separated lead times before strike to summarize.")
    parser.add_argument("--bin-width-s", type=float, default=0.05)
    parser.add_argument("--max-lead-s", type=float, default=1.0)
    parser.add_argument("--output-json", type=Path, default=None)
    parser.add_argument("--output-csv", type=Path, default=None)
    parser.add_argument("--output-figure", type=Path, default=None)
    parser.add_argument("--racket-radius-cm", type=float, default=7.5)
    parser.add_argument("--control-step-ms", type=float, default=20.0)
    args = parser.parse_args()

    rows = read_planner_eval_csv(args.eval_csv)
    if not rows:
        print(f"error: no planner evaluation rows found in {args.eval_csv}")
        return 1

    summary = build_paper_style_summary(
        rows,
        horizons_s=_parse_horizons(args.horizons_s),
        bin_width_s=args.bin_width_s,
        max_lead_s=args.max_lead_s,
    )
    print(json.dumps({key: summary[key] for key in ("event_count", "prediction_count", "horizons")}, indent=2))

    stem = args.eval_csv.with_suffix("")
    output_json = args.output_json if args.output_json is not None else stem.with_name(f"{stem.name}_paper_style_summary.json")
    output_csv = args.output_csv if args.output_csv is not None else stem.with_name(f"{stem.name}_paper_style_curve.csv")
    output_figure = args.output_figure if args.output_figure is not None else stem.with_name(f"{stem.name}_paper_style.png")

    output_json.parent.mkdir(parents=True, exist_ok=True)
    output_json.write_text(json.dumps(summary, indent=2) + "\n")
    write_summary_csv(summary, output_csv)
    save_paper_style_figure(
        summary,
        output_figure,
        racket_radius_cm=args.racket_radius_cm,
        control_step_ms=args.control_step_ms,
    )
    print(f"wrote_json={output_json}")
    print(f"wrote_csv={output_csv}")
    print(f"wrote_figure={output_figure}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
