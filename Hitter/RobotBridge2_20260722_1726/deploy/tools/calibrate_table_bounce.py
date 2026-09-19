#!/usr/bin/env python3
"""Measure MuJoCo table-bounce ratios against HITTER planner targets."""

from __future__ import annotations

import argparse
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np


DEPLOY_DIR = Path(__file__).resolve().parents[1]
XML_PATH = DEPLOY_DIR / "data/assets/g1/g1_29dof_hitter_racket_table_tennis.xml"

TARGET_VERTICAL_RESTITUTION = 0.897474
TARGET_HORIZONTAL_RESTITUTION = 0.764604


CASES = (
    ("vertical_slow", np.array([0.0, 0.0, -1.5], dtype=np.float64)),
    ("vertical_fast", np.array([0.0, 0.0, -2.5], dtype=np.float64)),
    ("incoming_center", np.array([-2.5, 0.0, -1.8], dtype=np.float64)),
    ("incoming_diagonal", np.array([-3.0, 0.6, -2.0], dtype=np.float64)),
)

SWEEP_SOLREFS = (
    "0.060 0.05",
    "0.060 0.08",
    "0.060 0.10",
    "0.050 0.05",
    "0.050 0.08",
    "0.050 0.10",
    "0.045 0.05",
    "0.045 0.08",
    "0.045 0.10",
    "0.040 0.05",
    "0.040 0.08",
    "0.040 0.10",
    "0.035 0.08",
    "0.035 0.10",
    "0.030 0.12",
    "0.028 0.12",
)

SWEEP_FRICTIONS = (
    "0.20 0.005 0.0001",
    "0.50 0.005 0.0001",
    "1.00 0.005 0.0001",
    "2.00 0.005 0.0001",
)


def geom_id(model: mujoco.MjModel, name: str) -> int:
    geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    if geom < 0:
        raise RuntimeError(f"Missing geom: {name}")
    return int(geom)


def joint_addresses(model: mujoco.MjModel, name: str) -> tuple[int, int]:
    joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if joint < 0:
        raise RuntimeError(f"Missing joint: {name}")
    return int(model.jnt_qposadr[joint]), int(model.jnt_dofadr[joint])


def has_contact(data: mujoco.MjData, geom_a: int, geom_b: int) -> bool:
    expected = {geom_a, geom_b}
    for index in range(data.ncon):
        if {int(data.contact[index].geom1), int(data.contact[index].geom2)} == expected:
            return True
    return False


def xml_with_table_pair_override(xml_path: Path, solref: str | None, friction: str | None) -> Path:
    if solref is None and friction is None:
        return xml_path

    tree = ET.parse(xml_path)
    root = tree.getroot()
    pair = root.find("./contact/pair[@geom1='hitter_table_top'][@geom2='hitter_ball_geom']")
    if pair is None:
        raise RuntimeError("Missing hitter_table_top/hitter_ball_geom contact pair.")
    if solref is not None:
        pair.set("solref", solref)
    if friction is not None:
        pair.set("friction", friction)

    output = tempfile.NamedTemporaryFile(
        mode="w",
        suffix=".xml",
        prefix="hitter_table_bounce_",
        dir=xml_path.parent,
        delete=False,
    )
    output.close()
    tree.write(output.name, encoding="unicode")
    return Path(output.name)


def measure_case(xml_path: Path, initial_velocity: np.ndarray, timestep: float) -> dict[str, float]:
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    model.opt.timestep = timestep
    data = mujoco.MjData(model)

    ball_geom = geom_id(model, "hitter_ball_geom")
    table_geom = geom_id(model, "hitter_table_top")
    qposadr, qveladr = joint_addresses(model, "hitter_ball_freejoint")

    data.qpos[qposadr : qposadr + 3] = [2.25, 0.45, 0.95]
    data.qpos[qposadr + 3 : qposadr + 7] = [1.0, 0.0, 0.0, 0.0]
    data.qvel[qveladr : qveladr + 3] = initial_velocity
    data.qvel[qveladr + 3 : qveladr + 6] = [0.0, 0.0, 0.0]
    mujoco.mj_forward(model, data)

    incoming_velocity = None
    outgoing_velocity = None
    peak_upward_velocity = None
    was_in_contact = has_contact(data, ball_geom, table_geom)
    saw_contact = False
    contact_steps = 0
    min_z = float(data.qpos[qposadr + 2])
    max_vz = float(data.qvel[qveladr + 2])

    for _ in range(2000):
        previous_velocity = data.qvel[qveladr : qveladr + 3].copy()
        mujoco.mj_step(model, data)
        in_contact = has_contact(data, ball_geom, table_geom)
        contact_steps += int(in_contact)
        min_z = min(min_z, float(data.qpos[qposadr + 2]))
        max_vz = max(max_vz, float(data.qvel[qveladr + 2]))

        if in_contact and not was_in_contact:
            incoming_velocity = previous_velocity
            saw_contact = True
        if saw_contact:
            current_velocity = data.qvel[qveladr : qveladr + 3].copy()
            if peak_upward_velocity is None or current_velocity[2] > peak_upward_velocity[2]:
                peak_upward_velocity = current_velocity
            if not in_contact and current_velocity[2] > 0.0:
                outgoing_velocity = current_velocity
                break
        was_in_contact = in_contact

    if outgoing_velocity is None:
        outgoing_velocity = peak_upward_velocity

    if incoming_velocity is None or outgoing_velocity is None or outgoing_velocity[2] <= 0.0:
        raise RuntimeError(
            "No complete table bounce detected for velocity "
            f"{initial_velocity}; saw_contact={saw_contact}, contact_steps={contact_steps}, "
            f"min_z={min_z:.6f}, max_vz={max_vz:.6f}."
        )

    incoming_h = float(np.linalg.norm(incoming_velocity[:2]))
    outgoing_h = float(np.linalg.norm(outgoing_velocity[:2]))
    vertical_ratio = float(outgoing_velocity[2] / max(abs(incoming_velocity[2]), 1.0e-12))
    horizontal_ratio = float("nan") if incoming_h < 1.0e-12 else outgoing_h / incoming_h

    return {
        "vin_x": float(incoming_velocity[0]),
        "vin_y": float(incoming_velocity[1]),
        "vin_z": float(incoming_velocity[2]),
        "vout_x": float(outgoing_velocity[0]),
        "vout_y": float(outgoing_velocity[1]),
        "vout_z": float(outgoing_velocity[2]),
        "vertical_ratio": vertical_ratio,
        "horizontal_ratio": horizontal_ratio,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--xml", type=Path, default=XML_PATH)
    parser.add_argument("--timestep", type=float, default=0.005)
    parser.add_argument("--table-solref", default=None)
    parser.add_argument("--table-friction", default=None)
    parser.add_argument("--sweep", action="store_true")
    args = parser.parse_args()

    if args.sweep:
        rows = []
        for solref in SWEEP_SOLREFS:
            for friction in SWEEP_FRICTIONS:
                xml_path = xml_with_table_pair_override(args.xml, solref, friction)
                try:
                    vertical_errors = []
                    horizontal_errors = []
                    max_vertical_ratio = -np.inf
                    for _name, velocity in CASES:
                        result = measure_case(xml_path, velocity, args.timestep)
                        vertical_ratio = result["vertical_ratio"]
                        horizontal_ratio = result["horizontal_ratio"]
                        vertical_errors.append(abs(vertical_ratio - TARGET_VERTICAL_RESTITUTION))
                        max_vertical_ratio = max(max_vertical_ratio, vertical_ratio)
                        if np.isfinite(horizontal_ratio):
                            horizontal_errors.append(abs(horizontal_ratio - TARGET_HORIZONTAL_RESTITUTION))
                    mean_vertical_error = float(np.mean(vertical_errors))
                    mean_horizontal_error = float(np.mean(horizontal_errors))
                    score = mean_vertical_error + mean_horizontal_error
                    if max_vertical_ratio > 1.0:
                        score += 10.0 * (max_vertical_ratio - 1.0)
                    rows.append((score, mean_vertical_error, mean_horizontal_error, max_vertical_ratio, solref, friction))
                except RuntimeError:
                    continue
                finally:
                    xml_path.unlink(missing_ok=True)

        print("Top sweep candidates, lower score is better:")
        print("score     v_err     h_err     max_v    solref       friction")
        print("-" * 76)
        for row in sorted(rows)[:12]:
            score, v_err, h_err, max_v, solref, friction = row
            print(f"{score:8.4f}  {v_err:8.4f}  {h_err:8.4f}  {max_v:7.4f}  {solref:11s}  {friction}")
        return

    xml_path = xml_with_table_pair_override(args.xml, args.table_solref, args.table_friction)

    print(f"xml: {args.xml}")
    if xml_path != args.xml:
        print(f"override_xml: {xml_path}")
    if args.table_solref is not None:
        print(f"override_table_solref: {args.table_solref}")
    if args.table_friction is not None:
        print(f"override_table_friction: {args.table_friction}")
    print(f"target_vertical_restitution: {TARGET_VERTICAL_RESTITUTION:.6f}")
    print(f"target_horizontal_restitution: {TARGET_HORIZONTAL_RESTITUTION:.6f}")
    print()
    print(
        "case              vin[x,y,z]                  vout[x,y,z]                 "
        "vertical  horizontal"
    )
    print("-" * 102)

    vertical_errors = []
    horizontal_errors = []
    try:
        for name, velocity in CASES:
            result = measure_case(xml_path, velocity, args.timestep)
            vertical_errors.append(result["vertical_ratio"] - TARGET_VERTICAL_RESTITUTION)
            if np.isfinite(result["horizontal_ratio"]):
                horizontal_errors.append(result["horizontal_ratio"] - TARGET_HORIZONTAL_RESTITUTION)
            print(
                f"{name:17s} "
                f"[{result['vin_x']:7.3f}, {result['vin_y']:6.3f}, {result['vin_z']:7.3f}]  "
                f"[{result['vout_x']:7.3f}, {result['vout_y']:6.3f}, {result['vout_z']:7.3f}]  "
                f"{result['vertical_ratio']:8.4f}  "
                f"{result['horizontal_ratio']:10.4f}"
            )
    finally:
        if xml_path != args.xml:
            xml_path.unlink(missing_ok=True)

    print()
    print(f"mean_vertical_error: {float(np.mean(vertical_errors)):+.6f}")
    if horizontal_errors:
        print(f"mean_horizontal_error: {float(np.mean(horizontal_errors)):+.6f}")


if __name__ == "__main__":
    main()
