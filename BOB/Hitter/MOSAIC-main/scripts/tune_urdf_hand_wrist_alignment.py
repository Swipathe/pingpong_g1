#!/usr/bin/env python3
"""Interactive right hand/wrist alignment tuner for g1_hitter_racket URDF.

The tool serves a small local browser UI.  It loads the parent wrist visual and
the child hand/racket visuals, lets the user adjust the fixed joint origin live,
and writes only that origin line back to the URDF on Save.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import shutil
import socketserver
import struct
import sys
import time
import urllib.parse
import webbrowser
from http.server import BaseHTTPRequestHandler
from pathlib import Path
from typing import Iterable
from xml.etree import ElementTree as ET


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
DEFAULT_ASSET_ROOT = (
    REPO_ROOT
    / "source/whole_body_tracking/whole_body_tracking/assets/unitree_description"
)
DEFAULT_URDF = DEFAULT_ASSET_ROOT / "urdf/g1_hitter_racket/main.urdf"


def parse_floats(value: str, count: int) -> list[float]:
    parts = [float(x) for x in value.split()]
    if len(parts) != count:
        raise ValueError(f"expected {count} floats, got {len(parts)} in {value!r}")
    return parts


def rpy_to_matrix(rpy: Iterable[float]) -> list[list[float]]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = ((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr))
    ry = ((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp))
    rz = ((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0))

    def matmul(a: Iterable[Iterable[float]], b: Iterable[Iterable[float]]):
        a_rows = [list(row) for row in a]
        b_rows = [list(row) for row in b]
        return [
            [sum(a_rows[i][k] * b_rows[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)
        ]

    return matmul(matmul(rz, ry), rx)


def transform_points(
    points: list[list[float]], xyz: Iterable[float], rpy: Iterable[float]
) -> list[list[float]]:
    x, y, z = xyz
    rot = rpy_to_matrix(rpy)
    out: list[list[float]] = []
    for px, py, pz in points:
        out.append(
            [
                rot[0][0] * px + rot[0][1] * py + rot[0][2] * pz + x,
                rot[1][0] * px + rot[1][1] * py + rot[1][2] * pz + y,
                rot[2][0] * px + rot[2][1] * py + rot[2][2] * pz + z,
            ]
        )
    return out


def apply_origin(
    points: list[list[float]], origin_elem: ET.Element | None
) -> list[list[float]]:
    if origin_elem is None:
        return points
    xyz = parse_floats(origin_elem.attrib.get("xyz", "0 0 0"), 3)
    rpy = parse_floats(origin_elem.attrib.get("rpy", "0 0 0"), 3)
    return transform_points(points, xyz, rpy)


def parse_stl(path: Path) -> list[list[float]]:
    data = path.read_bytes()
    vertices: list[list[float]] = []

    if len(data) >= 84:
        tri_count = struct.unpack("<I", data[80:84])[0]
        if 84 + 50 * tri_count == len(data):
            offset = 84
            for _ in range(tri_count):
                offset += 12
                for _ in range(3):
                    vertices.append(list(struct.unpack("<fff", data[offset : offset + 12])))
                    offset += 12
                offset += 2
            return vertices

    for raw_line in data.decode("utf-8", "ignore").splitlines():
        parts = raw_line.strip().split()
        if len(parts) == 4 and parts[0].lower() == "vertex":
            vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
    return vertices


def sample_points(points: list[list[float]], limit: int) -> list[list[float]]:
    if len(points) <= limit:
        return [[round(v, 6) for v in point] for point in points]
    step = len(points) / float(limit)
    sampled = []
    index = 0.0
    while len(sampled) < limit:
        sampled.append([round(v, 6) for v in points[int(index)]])
        index += step
    return sampled


def sample_triangle_vertices(points: list[list[float]], limit: int) -> list[float]:
    tri_count = len(points) // 3
    if tri_count == 0:
        return []
    if tri_count <= limit:
        tri_indices = range(tri_count)
    else:
        step = tri_count / float(limit)
        tri_indices = (int(i * step) for i in range(limit))

    sampled: list[float] = []
    for tri_index in tri_indices:
        start = tri_index * 3
        for point in points[start : start + 3]:
            sampled.extend(round(value, 6) for value in point)
    return sampled


def bbox(points: list[list[float]]) -> dict[str, list[float]]:
    mins = [min(point[i] for point in points) for i in range(3)]
    maxs = [max(point[i] for point in points) for i in range(3)]
    return {
        "min": [round(v, 6) for v in mins],
        "max": [round(v, 6) for v in maxs],
        "center": [round((mins[i] + maxs[i]) / 2.0, 6) for i in range(3)],
    }


def resolve_mesh(asset_root: Path, filename: str) -> Path:
    prefix = "package://unitree_description/"
    if filename.startswith(prefix):
        return asset_root / filename.removeprefix(prefix)
    return Path(filename)


def get_required_element(parent: ET.Element, path: str, label: str) -> ET.Element:
    found = parent.find(path)
    if found is None:
        raise ValueError(f"missing {label}: {path}")
    return found


class AlignmentModel:
    def __init__(
        self,
        urdf_path: Path,
        joint_name: str,
        asset_root: Path,
        cuff_width: float,
        insertion: float,
    ) -> None:
        self.urdf_path = urdf_path
        self.joint_name = joint_name
        self.asset_root = asset_root
        self.cuff_width = cuff_width
        self.insertion = insertion
        self._model_cache: dict | None = None

    def invalidate(self) -> None:
        self._model_cache = None

    def build(self) -> dict:
        if self._model_cache is not None:
            return self._model_cache

        root = ET.parse(self.urdf_path).getroot()
        joint = root.find(f".//joint[@name='{self.joint_name}']")
        if joint is None:
            raise ValueError(f"joint not found: {self.joint_name}")
        origin = get_required_element(joint, "origin", "joint origin")
        xyz = parse_floats(origin.attrib.get("xyz", "0 0 0"), 3)
        rpy = parse_floats(origin.attrib.get("rpy", "0 0 0"), 3)
        parent_link_name = get_required_element(joint, "parent", "joint parent").attrib[
            "link"
        ]
        child_link_name = get_required_element(joint, "child", "joint child").attrib[
            "link"
        ]

        links = {link.attrib["name"]: link for link in root.findall("link")}
        parent_link = links[parent_link_name]
        child_link = links[child_link_name]

        visuals = []
        parent_full_points: list[list[float]] = []
        child_full_points: list[list[float]] = []
        grip_full_points: list[list[float]] = []

        def add_visuals(
            link: ET.Element, role: str, max_points: int, max_triangles: int
        ) -> None:
            nonlocal grip_full_points
            link_name = link.attrib["name"]
            for index, visual in enumerate(link.findall("visual")):
                mesh = visual.find("./geometry/mesh")
                if mesh is None or "filename" not in mesh.attrib:
                    continue
                path = resolve_mesh(self.asset_root, mesh.attrib["filename"])
                points = apply_origin(parse_stl(path), visual.find("origin"))
                if role == "parent":
                    parent_full_points.extend(points)
                else:
                    child_full_points.extend(points)
                    if "hand_grip" in path.name:
                        grip_full_points.extend(points)

                material = visual.find("material")
                color = None
                if material is not None:
                    color_elem = material.find("color")
                    if color_elem is not None and "rgba" in color_elem.attrib:
                        rgba = [float(v) for v in color_elem.attrib["rgba"].split()]
                        color = "#{:02x}{:02x}{:02x}".format(
                            int(max(0.0, min(1.0, rgba[0])) * 255),
                            int(max(0.0, min(1.0, rgba[1])) * 255),
                            int(max(0.0, min(1.0, rgba[2])) * 255),
                        )

                visuals.append(
                    {
                        "name": visual.attrib.get("name", f"{link_name}_visual_{index}"),
                        "link": link_name,
                        "role": role,
                        "mesh": str(path.relative_to(self.asset_root)),
                        "color": color,
                        "points": sample_points(points, max_points),
                        "triangles": sample_triangle_vertices(points, max_triangles),
                    }
                )

        add_visuals(parent_link, "parent", 10000, 8000)
        add_visuals(child_link, "child", 18000, 16000)

        if not parent_full_points:
            raise ValueError(f"no parent visual mesh points for {parent_link_name}")
        if not child_full_points:
            raise ValueError(f"no child visual mesh points for {child_link_name}")
        cuff_source = grip_full_points or child_full_points
        min_x = min(point[0] for point in cuff_source)
        cuff_points = [
            point for point in cuff_source if point[0] <= min_x + self.cuff_width
        ]
        if not cuff_points:
            cuff_points = cuff_source
        cuff_y = (
            min(point[1] for point in cuff_points)
            + max(point[1] for point in cuff_points)
        ) / 2.0
        cuff_z = (
            min(point[2] for point in cuff_points)
            + max(point[2] for point in cuff_points)
        ) / 2.0
        wrist_max_x = max(point[0] for point in parent_full_points)
        target = [wrist_max_x - self.insertion, 0.0, 0.0]

        self._model_cache = {
            "urdf": str(self.urdf_path),
            "joint": self.joint_name,
            "parent": parent_link_name,
            "child": child_link_name,
            "origin": {"xyz": xyz, "rpy": rpy},
            "visuals": visuals,
            "metadata": {
                "parent_bbox": bbox(parent_full_points),
                "child_bbox": bbox(child_full_points),
                "cuff_width": self.cuff_width,
                "insertion": self.insertion,
                "wrist_max_x": round(wrist_max_x, 6),
                "cuff_center_local": [
                    round(min_x, 6),
                    round(cuff_y, 6),
                    round(cuff_z, 6),
                ],
                "snap_target_parent": [round(v, 6) for v in target],
            },
        }
        return self._model_cache

    def save_origin(self, xyz: list[float], rpy: list[float]) -> Path:
        if len(xyz) != 3 or len(rpy) != 3:
            raise ValueError("xyz and rpy must each contain 3 values")
        text = self.urdf_path.read_text()
        joint_name = re.escape(self.joint_name)
        pattern = re.compile(
            rf'(<joint\s+name="{joint_name}"[^>]*>.*?'
            rf'<origin\s+xyz=")([^"]*)("'
            rf'\s+rpy=")([^"]*)(".*?/>)',
            re.DOTALL,
        )
        replacement = (
            r"\g<1>"
            + " ".join(f"{value:.6f}".rstrip("0").rstrip(".") for value in xyz)
            + r"\g<3>"
            + " ".join(f"{value:.6f}".rstrip("0").rstrip(".") for value in rpy)
            + r"\g<5>"
        )
        new_text, count = pattern.subn(replacement, text, count=1)
        if count != 1:
            raise ValueError(f"could not replace origin for joint {self.joint_name}")

        timestamp = time.strftime("%Y%m%d-%H%M%S")
        backup_path = self.urdf_path.with_suffix(
            self.urdf_path.suffix + f".bak-{timestamp}"
        )
        shutil.copy2(self.urdf_path, backup_path)
        self.urdf_path.write_text(new_text)
        ET.parse(self.urdf_path)
        self.invalidate()
        return backup_path


HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>URDF Hand/Wrist Alignment Tuner</title>
  <style>
    :root {
      color-scheme: light dark;
      --bg: #101216;
      --panel: #181b21;
      --line: #303641;
      --text: #edf1f5;
      --muted: #9aa6b2;
      --accent: #3da5ff;
      --green: #18b36a;
      --orange: #d99826;
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      background: var(--bg);
      color: var(--text);
      display: grid;
      grid-template-columns: minmax(720px, 1fr) 360px;
      overflow: hidden;
    }
    main {
      min-width: 0;
      height: 100vh;
      display: grid;
      grid-template-rows: 48px 1fr;
    }
    header {
      display: flex;
      align-items: center;
      gap: 14px;
      padding: 0 16px;
      border-bottom: 1px solid var(--line);
      background: #13161b;
      min-width: 0;
    }
    header h1 {
      margin: 0;
      font-size: 16px;
      font-weight: 650;
      white-space: nowrap;
    }
    header .path {
      min-width: 0;
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
      color: var(--muted);
      font-size: 12px;
    }
    .views {
      display: grid;
      grid-template-columns: minmax(520px, 1.45fr) minmax(320px, 1fr);
      grid-template-rows: 1fr 1fr 1fr;
      gap: 1px;
      background: var(--line);
      min-height: 0;
    }
    .view {
      position: relative;
      background: #f7f8fa;
      min-width: 0;
      min-height: 0;
    }
    .view.three { grid-row: 1 / span 3; background: #0b0e13; }
    .view canvas {
      width: 100%;
      height: 100%;
      display: block;
    }
    .view-title {
      position: absolute;
      left: 12px;
      top: 10px;
      color: #1b222b;
      font-size: 12px;
      font-weight: 650;
      background: rgba(255,255,255,0.78);
      border: 1px solid rgba(0,0,0,0.08);
      border-radius: 6px;
      padding: 4px 7px;
      pointer-events: none;
    }
    .view.three .view-title {
      color: #eef4fb;
      background: rgba(8, 12, 18, 0.72);
      border-color: rgba(255,255,255,0.12);
    }
    .orbit-help {
      position: absolute;
      right: 12px;
      bottom: 10px;
      color: rgba(238,244,251,0.78);
      font-size: 11px;
      background: rgba(8, 12, 18, 0.66);
      border: 1px solid rgba(255,255,255,0.12);
      border-radius: 6px;
      padding: 5px 7px;
      pointer-events: none;
    }
    aside {
      height: 100vh;
      overflow: auto;
      border-left: 1px solid var(--line);
      background: var(--panel);
      padding: 14px;
    }
    section {
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 12px;
      margin-bottom: 12px;
      background: #151820;
    }
    h2 {
      margin: 0 0 10px 0;
      font-size: 13px;
      letter-spacing: 0;
    }
    .field {
      display: grid;
      grid-template-columns: 28px 1fr 86px;
      align-items: center;
      gap: 8px;
      margin: 9px 0;
    }
    label {
      color: var(--muted);
      font-size: 12px;
      font-variant-numeric: tabular-nums;
    }
    input[type="range"] { width: 100%; }
    input[type="number"] {
      width: 86px;
      background: #0e1117;
      color: var(--text);
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 5px 6px;
      font-size: 12px;
      font-variant-numeric: tabular-nums;
    }
    .button-row {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
      margin-top: 10px;
    }
    button {
      appearance: none;
      border: 1px solid var(--line);
      border-radius: 7px;
      background: #222833;
      color: var(--text);
      padding: 8px 10px;
      font-size: 12px;
      cursor: pointer;
    }
    button.primary { background: #0f6fbe; border-color: #1a86df; }
    button:hover { filter: brightness(1.08); }
    .meta, .status {
      color: var(--muted);
      font-size: 12px;
      line-height: 1.45;
      word-break: break-word;
    }
    .status {
      margin-top: 10px;
      padding: 8px;
      border: 1px solid var(--line);
      border-radius: 7px;
      background: #10131a;
      min-height: 36px;
    }
    .legend {
      display: grid;
      gap: 7px;
    }
    .legend span {
      display: flex;
      align-items: center;
      gap: 8px;
      color: var(--muted);
      font-size: 12px;
    }
    .swatch {
      width: 12px;
      height: 12px;
      border-radius: 3px;
      display: inline-block;
    }
    .hint {
      color: var(--muted);
      font-size: 11px;
      line-height: 1.4;
      margin-top: 8px;
    }
    @media (max-width: 1100px) {
      body { grid-template-columns: 1fr; overflow: auto; }
      main { height: 70vh; }
      aside { height: auto; border-left: 0; border-top: 1px solid var(--line); }
    }
  </style>
</head>
<body>
  <main>
    <header>
      <h1>URDF Hand/Wrist Alignment Tuner</h1>
      <div class="path" id="urdfPath"></div>
    </header>
    <div class="views">
      <div class="view three">
        <div class="view-title">3D orbit render</div>
        <canvas id="gl3d"></canvas>
        <div class="orbit-help">drag rotate · wheel zoom · double click reset</div>
      </div>
      <div class="view">
        <div class="view-title">X/Z side</div>
        <canvas id="xz"></canvas>
      </div>
      <div class="view">
        <div class="view-title">X/Y top</div>
        <canvas id="xy"></canvas>
      </div>
      <div class="view">
        <div class="view-title">Y/Z front</div>
        <canvas id="yz"></canvas>
      </div>
    </div>
  </main>
  <aside>
    <section>
      <h2>Fixed Joint Origin</h2>
      <div id="controls"></div>
      <div class="button-row">
        <button id="resetBtn">Reset</button>
        <button id="snapBtn">Snap cuff</button>
      </div>
      <div class="button-row">
        <button id="copyBtn">Copy XML</button>
        <button class="primary" id="saveBtn">Save URDF</button>
      </div>
      <div class="status" id="status">Loading...</div>
    </section>
    <section>
      <h2>Alignment Metrics</h2>
      <div class="meta" id="metrics"></div>
    </section>
    <section>
      <h2>Legend</h2>
      <div class="legend">
        <span><i class="swatch" style="background:#757f8a"></i>wrist yaw link</span>
        <span><i class="swatch" style="background:#3974e8"></i>hand grip</span>
        <span><i class="swatch" style="background:#d99826"></i>paddle/racket</span>
        <span><i class="swatch" style="background:#18b36a"></i>wrist x-axis and cuff marker</span>
      </div>
      <div class="hint">Save creates a timestamped .bak file next to the URDF and replaces only the selected joint origin line.</div>
    </section>
  </aside>

<script>
"use strict";

const state = {
  model: null,
  values: { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 },
  saved: null,
  canvases: {},
  gl3d: null,
};

const fields = [
  ["x", "X", -0.1, 0.4, 0.0005],
  ["y", "Y", -0.12, 0.12, 0.0005],
  ["z", "Z", -0.12, 0.12, 0.0005],
  ["roll", "R", -3.1416, 3.1416, 0.002],
  ["pitch", "P", -3.1416, 3.1416, 0.002],
  ["yaw", "Yw", -3.1416, 3.1416, 0.002],
];

function $(id) { return document.getElementById(id); }

function setStatus(text) {
  $("status").textContent = text;
}

function fmt(v) {
  return Number(v).toFixed(6).replace(/0+$/, "").replace(/\.$/, "") || "0";
}

function makeControls() {
  const controls = $("controls");
  controls.innerHTML = "";
  for (const [key, label, min, max, step] of fields) {
    const row = document.createElement("div");
    row.className = "field";
    const lab = document.createElement("label");
    lab.textContent = label;
    const range = document.createElement("input");
    range.type = "range";
    range.min = min;
    range.max = max;
    range.step = step;
    range.id = `${key}Range`;
    const num = document.createElement("input");
    num.type = "number";
    num.step = step;
    num.id = `${key}Number`;
    row.append(lab, range, num);
    controls.append(row);
    const update = (raw) => {
      state.values[key] = Number(raw);
      range.value = state.values[key];
      num.value = fmt(state.values[key]);
      drawAll();
    };
    range.addEventListener("input", () => update(range.value));
    num.addEventListener("input", () => update(num.value));
  }
}

function setValues(xyz, rpy) {
  const next = {
    x: xyz[0], y: xyz[1], z: xyz[2],
    roll: rpy[0], pitch: rpy[1], yaw: rpy[2],
  };
  state.values = next;
  for (const [key] of fields) {
    $(`${key}Range`).value = next[key];
    $(`${key}Number`).value = fmt(next[key]);
  }
  drawAll();
}

function matFromRpy(roll, pitch, yaw) {
  const cr = Math.cos(roll), sr = Math.sin(roll);
  const cp = Math.cos(pitch), sp = Math.sin(pitch);
  const cy = Math.cos(yaw), sy = Math.sin(yaw);
  return [
    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
    [-sp, cp * sr, cp * cr],
  ];
}

function transformPoint(p) {
  const v = state.values;
  const m = matFromRpy(v.roll, v.pitch, v.yaw);
  return [
    m[0][0] * p[0] + m[0][1] * p[1] + m[0][2] * p[2] + v.x,
    m[1][0] * p[0] + m[1][1] * p[1] + m[1][2] * p[2] + v.y,
    m[2][0] * p[0] + m[2][1] * p[1] + m[2][2] * p[2] + v.z,
  ];
}

function currentClouds() {
  const out = [];
  for (const visual of state.model.visuals) {
    const transformed = visual.role === "child"
      ? visual.points.map(transformPoint)
      : visual.points;
    out.push({ visual, points: transformed });
  }
  return out;
}

function cuffWorld() {
  const c = state.model.metadata.cuff_center_local;
  return transformPoint(c);
}

function metrics() {
  const c = cuffWorld();
  const wristMaxX = state.model.metadata.wrist_max_x;
  const insertion = wristMaxX - c[0];
  const yzError = Math.hypot(c[1], c[2]);
  $("metrics").innerHTML = [
    `joint: ${state.model.joint}`,
    `parent: ${state.model.parent}`,
    `child: ${state.model.child}`,
    `origin xyz: ${fmt(state.values.x)} ${fmt(state.values.y)} ${fmt(state.values.z)}`,
    `origin rpy: ${fmt(state.values.roll)} ${fmt(state.values.pitch)} ${fmt(state.values.yaw)}`,
    `cuff center: ${c.map(fmt).join(" ")}`,
    `cuff y/z axis error: ${fmt(yzError)} m`,
    `cuff insertion by center x: ${fmt(insertion)} m`,
  ].join("<br>");
}

function fitBounds(clouds, axes) {
  let minA = Infinity, minB = Infinity, maxA = -Infinity, maxB = -Infinity;
  for (const cloud of clouds) {
    for (const p of cloud.points) {
      const a = p[axes[0]], b = p[axes[1]];
      if (a < minA) minA = a;
      if (a > maxA) maxA = a;
      if (b < minB) minB = b;
      if (b > maxB) maxB = b;
    }
  }
  const pad = 0.04;
  minA -= pad; maxA += pad; minB -= pad; maxB += pad;
  const spanA = maxA - minA;
  const spanB = maxB - minB;
  const span = Math.max(spanA, spanB, 0.1);
  const midA = (minA + maxA) / 2;
  const midB = (minB + maxB) / 2;
  return { minA: midA - span / 2, maxA: midA + span / 2, minB: midB - span / 2, maxB: midB + span / 2 };
}

function drawView(canvas, axes) {
  const ctx = canvas.getContext("2d");
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.round(rect.width * dpr));
  canvas.height = Math.max(1, Math.round(rect.height * dpr));
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const w = rect.width, h = rect.height;
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = "#f7f8fa";
  ctx.fillRect(0, 0, w, h);

  const clouds = currentClouds();
  const bounds = fitBounds(clouds, axes);
  const sx = w / (bounds.maxA - bounds.minA);
  const sy = h / (bounds.maxB - bounds.minB);
  function project(p) {
    const x = (p[axes[0]] - bounds.minA) * sx;
    const y = h - (p[axes[1]] - bounds.minB) * sy;
    return [x, y];
  }

  function line(pa, pb, color, width = 1) {
    const a = project(pa), b = project(pb);
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.beginPath();
    ctx.moveTo(a[0], a[1]);
    ctx.lineTo(b[0], b[1]);
    ctx.stroke();
  }

  // Grid and origin axes.
  ctx.strokeStyle = "rgba(28, 36, 46, 0.10)";
  ctx.lineWidth = 1;
  for (let i = 0; i <= 10; i++) {
    const x = (w * i) / 10;
    const y = (h * i) / 10;
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
  }
  const axisA = [0, 0, 0], axisB = [0, 0, 0];
  axisA[axes[0]] = bounds.minA; axisB[axes[0]] = bounds.maxA;
  line(axisA, axisB, "rgba(0,0,0,0.30)", 1);
  const axisC = [0, 0, 0], axisD = [0, 0, 0];
  axisC[axes[1]] = bounds.minB; axisD[axes[1]] = bounds.maxB;
  line(axisC, axisD, "rgba(0,0,0,0.30)", 1);

  for (const cloud of clouds) {
    const isPaddle = cloud.visual.name.toLowerCase().includes("paddle") || cloud.visual.name.toLowerCase().includes("racket");
    const isChild = cloud.visual.role === "child";
    const color = isPaddle ? "#d99826" : (isChild ? "#3974e8" : "#6f7782");
    ctx.fillStyle = color;
    ctx.globalAlpha = isPaddle ? 0.62 : (isChild ? 0.42 : 0.34);
    const stride = Math.max(1, Math.floor(cloud.points.length / 9000));
    for (let i = 0; i < cloud.points.length; i += stride) {
      const p = project(cloud.points[i]);
      ctx.fillRect(p[0], p[1], 1.2, 1.2);
    }
  }
  ctx.globalAlpha = 1;

  // Wrist x-axis in parent frame.
  line([0, 0, 0], [0.12, 0, 0], "#18b36a", 2);
  const c = cuffWorld();
  const pc = project(c);
  ctx.strokeStyle = "#18b36a";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(pc[0] - 6, pc[1] - 6);
  ctx.lineTo(pc[0] + 6, pc[1] + 6);
  ctx.moveTo(pc[0] + 6, pc[1] - 6);
  ctx.lineTo(pc[0] - 6, pc[1] + 6);
  ctx.stroke();
}

function hexToRgb(hex, fallback) {
  const raw = (hex || "").replace("#", "");
  if (raw.length !== 6) return fallback;
  return [
    parseInt(raw.slice(0, 2), 16) / 255,
    parseInt(raw.slice(2, 4), 16) / 255,
    parseInt(raw.slice(4, 6), 16) / 255,
  ];
}

function mat4Identity() {
  return new Float32Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]);
}

function mat4Multiply(a, b) {
  const out = new Float32Array(16);
  for (let col = 0; col < 4; col++) {
    for (let row = 0; row < 4; row++) {
      out[col * 4 + row] =
        a[0 * 4 + row] * b[col * 4 + 0] +
        a[1 * 4 + row] * b[col * 4 + 1] +
        a[2 * 4 + row] * b[col * 4 + 2] +
        a[3 * 4 + row] * b[col * 4 + 3];
    }
  }
  return out;
}

function perspective(fovy, aspect, near, far) {
  const f = 1 / Math.tan(fovy / 2);
  const out = new Float32Array(16);
  out[0] = f / aspect;
  out[5] = f;
  out[10] = (far + near) / (near - far);
  out[11] = -1;
  out[14] = (2 * far * near) / (near - far);
  return out;
}

function normalize(v) {
  const len = Math.hypot(v[0], v[1], v[2]) || 1;
  return [v[0] / len, v[1] / len, v[2] / len];
}

function cross(a, b) {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

function dot(a, b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function lookAt(eye, target, up) {
  const z = normalize([eye[0] - target[0], eye[1] - target[1], eye[2] - target[2]]);
  const x = normalize(cross(up, z));
  const y = cross(z, x);
  return new Float32Array([
    x[0], y[0], z[0], 0,
    x[1], y[1], z[1], 0,
    x[2], y[2], z[2], 0,
    -dot(x, eye), -dot(y, eye), -dot(z, eye), 1,
  ]);
}

function childModelMatrix() {
  const v = state.values;
  const r = matFromRpy(v.roll, v.pitch, v.yaw);
  return new Float32Array([
    r[0][0], r[1][0], r[2][0], 0,
    r[0][1], r[1][1], r[2][1], 0,
    r[0][2], r[1][2], r[2][2], 0,
    v.x, v.y, v.z, 1,
  ]);
}

function childNormalMatrix() {
  const v = state.values;
  const r = matFromRpy(v.roll, v.pitch, v.yaw);
  return new Float32Array([
    r[0][0], r[1][0], r[2][0],
    r[0][1], r[1][1], r[2][1],
    r[0][2], r[1][2], r[2][2],
  ]);
}

function buildNormals(vertices) {
  const normals = new Float32Array(vertices.length);
  for (let i = 0; i + 8 < vertices.length; i += 9) {
    const ax = vertices[i], ay = vertices[i + 1], az = vertices[i + 2];
    const bx = vertices[i + 3], by = vertices[i + 4], bz = vertices[i + 5];
    const cx = vertices[i + 6], cy = vertices[i + 7], cz = vertices[i + 8];
    const ux = bx - ax, uy = by - ay, uz = bz - az;
    const vx = cx - ax, vy = cy - ay, vz = cz - az;
    const n = normalize([uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx]);
    for (let j = 0; j < 3; j++) {
      normals[i + j * 3] = n[0];
      normals[i + j * 3 + 1] = n[1];
      normals[i + j * 3 + 2] = n[2];
    }
  }
  return normals;
}

function compileShader(gl, type, source) {
  const shader = gl.createShader(type);
  gl.shaderSource(shader, source);
  gl.compileShader(shader);
  if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
    throw new Error(gl.getShaderInfoLog(shader) || "shader compile failed");
  }
  return shader;
}

function createProgram(gl, vertexSource, fragmentSource) {
  const program = gl.createProgram();
  gl.attachShader(program, compileShader(gl, gl.VERTEX_SHADER, vertexSource));
  gl.attachShader(program, compileShader(gl, gl.FRAGMENT_SHADER, fragmentSource));
  gl.linkProgram(program);
  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
    throw new Error(gl.getProgramInfoLog(program) || "program link failed");
  }
  return program;
}

function createBuffer(gl, data) {
  const buffer = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
  gl.bufferData(gl.ARRAY_BUFFER, data, gl.STATIC_DRAW);
  return buffer;
}

function init3D() {
  const canvas = $("gl3d");
  const gl = canvas.getContext("webgl", { antialias: true });
  if (!gl) {
    setStatus("WebGL is not available in this browser.");
    return;
  }
  const meshProgram = createProgram(gl, `
    attribute vec3 a_position;
    attribute vec3 a_normal;
    uniform mat4 u_viewProj;
    uniform mat4 u_model;
    uniform mat3 u_normalMatrix;
    varying float v_light;
    void main() {
      vec3 normal = normalize(u_normalMatrix * a_normal);
      vec3 lightDir = normalize(vec3(-0.45, -0.55, 0.72));
      v_light = max(dot(normal, lightDir), 0.18);
      gl_Position = u_viewProj * u_model * vec4(a_position, 1.0);
    }
  `, `
    precision mediump float;
    uniform vec3 u_color;
    varying float v_light;
    void main() {
      gl_FragColor = vec4(u_color * v_light, 1.0);
    }
  `);
  const lineProgram = createProgram(gl, `
    attribute vec3 a_position;
    uniform mat4 u_viewProj;
    void main() {
      gl_Position = u_viewProj * vec4(a_position, 1.0);
    }
  `, `
    precision mediump float;
    uniform vec3 u_color;
    void main() {
      gl_FragColor = vec4(u_color, 1.0);
    }
  `);

  const meshes = [];
  for (const visual of state.model.visuals) {
    if (!visual.triangles || visual.triangles.length < 9) continue;
    const vertices = new Float32Array(visual.triangles);
    const normals = buildNormals(vertices);
    const fallback = visual.role === "parent" ? [0.46, 0.50, 0.55] : [0.22, 0.45, 0.90];
    const isPaddle = visual.name.toLowerCase().includes("paddle") || visual.name.toLowerCase().includes("racket");
    meshes.push({
      role: visual.role,
      name: visual.name,
      count: vertices.length / 3,
      color: isPaddle ? [0.85, 0.58, 0.14] : hexToRgb(visual.color, fallback),
      position: createBuffer(gl, vertices),
      normal: createBuffer(gl, normals),
    });
  }

  const axisData = new Float32Array([0, 0, 0, 0.14, 0, 0]);
  const axisBuffer = createBuffer(gl, axisData);
  const cuffBuffer = gl.createBuffer();

  const metadata = state.model.metadata;
  const bb0 = metadata.parent_bbox.min.concat(metadata.child_bbox.min);
  const bb1 = metadata.parent_bbox.max.concat(metadata.child_bbox.max);
  const target = [
    (Math.min(bb0[0], bb0[3]) + Math.max(bb1[0], bb1[3])) / 2,
    (Math.min(bb0[1], bb0[4]) + Math.max(bb1[1], bb1[4])) / 2,
    (Math.min(bb0[2], bb0[5]) + Math.max(bb1[2], bb1[5])) / 2,
  ];

  state.gl3d = {
    canvas, gl, meshProgram, lineProgram, meshes, axisBuffer, cuffBuffer,
    camera: { yaw: -0.82, pitch: 0.34, distance: 0.48, target },
    dragging: false,
    last: [0, 0],
  };

  canvas.addEventListener("pointerdown", (event) => {
    state.gl3d.dragging = true;
    state.gl3d.last = [event.clientX, event.clientY];
    canvas.setPointerCapture(event.pointerId);
  });
  canvas.addEventListener("pointermove", (event) => {
    const view = state.gl3d;
    if (!view.dragging) return;
    const dx = event.clientX - view.last[0];
    const dy = event.clientY - view.last[1];
    view.last = [event.clientX, event.clientY];
    view.camera.yaw -= dx * 0.008;
    view.camera.pitch = Math.max(-1.35, Math.min(1.35, view.camera.pitch - dy * 0.008));
    draw3D();
  });
  canvas.addEventListener("pointerup", () => { state.gl3d.dragging = false; });
  canvas.addEventListener("wheel", (event) => {
    event.preventDefault();
    const camera = state.gl3d.camera;
    camera.distance = Math.max(0.12, Math.min(1.6, camera.distance * Math.exp(event.deltaY * 0.001)));
    draw3D();
  }, { passive: false });
  canvas.addEventListener("dblclick", () => {
    state.gl3d.camera.yaw = -0.82;
    state.gl3d.camera.pitch = 0.34;
    state.gl3d.camera.distance = 0.48;
    draw3D();
  });
}

function drawLineBuffer(points) {
  const view = state.gl3d;
  const gl = view.gl;
  gl.bindBuffer(gl.ARRAY_BUFFER, view.cuffBuffer);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(points), gl.DYNAMIC_DRAW);
}

function draw3D() {
  const view = state.gl3d;
  if (!view) return;
  const { gl, canvas, meshProgram, lineProgram } = view;
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const width = Math.max(1, Math.round(rect.width * dpr));
  const height = Math.max(1, Math.round(rect.height * dpr));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  gl.viewport(0, 0, width, height);
  gl.clearColor(0.045, 0.055, 0.075, 1);
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
  gl.enable(gl.DEPTH_TEST);
  gl.disable(gl.CULL_FACE);

  const camera = view.camera;
  const cp = Math.cos(camera.pitch);
  const eye = [
    camera.target[0] + camera.distance * cp * Math.cos(camera.yaw),
    camera.target[1] + camera.distance * cp * Math.sin(camera.yaw),
    camera.target[2] + camera.distance * Math.sin(camera.pitch),
  ];
  const viewMat = lookAt(eye, camera.target, [0, 0, 1]);
  const projMat = perspective(Math.PI / 4.2, width / height, 0.01, 5.0);
  const viewProj = mat4Multiply(projMat, viewMat);
  const identity = mat4Identity();
  const identityNormal = new Float32Array([1,0,0,0,1,0,0,0,1]);
  const childModel = childModelMatrix();
  const childNormal = childNormalMatrix();

  gl.useProgram(meshProgram);
  const posLoc = gl.getAttribLocation(meshProgram, "a_position");
  const normalLoc = gl.getAttribLocation(meshProgram, "a_normal");
  gl.uniformMatrix4fv(gl.getUniformLocation(meshProgram, "u_viewProj"), false, viewProj);

  for (const mesh of view.meshes) {
    gl.bindBuffer(gl.ARRAY_BUFFER, mesh.position);
    gl.enableVertexAttribArray(posLoc);
    gl.vertexAttribPointer(posLoc, 3, gl.FLOAT, false, 0, 0);
    gl.bindBuffer(gl.ARRAY_BUFFER, mesh.normal);
    gl.enableVertexAttribArray(normalLoc);
    gl.vertexAttribPointer(normalLoc, 3, gl.FLOAT, false, 0, 0);
    gl.uniformMatrix4fv(gl.getUniformLocation(meshProgram, "u_model"), false, mesh.role === "child" ? childModel : identity);
    gl.uniformMatrix3fv(gl.getUniformLocation(meshProgram, "u_normalMatrix"), false, mesh.role === "child" ? childNormal : identityNormal);
    gl.uniform3fv(gl.getUniformLocation(meshProgram, "u_color"), new Float32Array(mesh.color));
    gl.drawArrays(gl.TRIANGLES, 0, mesh.count);
  }

  gl.useProgram(lineProgram);
  const linePosLoc = gl.getAttribLocation(lineProgram, "a_position");
  gl.uniformMatrix4fv(gl.getUniformLocation(lineProgram, "u_viewProj"), false, viewProj);
  gl.uniform3fv(gl.getUniformLocation(lineProgram, "u_color"), new Float32Array([0.08, 0.75, 0.42]));
  gl.bindBuffer(gl.ARRAY_BUFFER, view.axisBuffer);
  gl.enableVertexAttribArray(linePosLoc);
  gl.vertexAttribPointer(linePosLoc, 3, gl.FLOAT, false, 0, 0);
  gl.lineWidth(2);
  gl.drawArrays(gl.LINES, 0, 2);

  const c = cuffWorld();
  const s = 0.014;
  drawLineBuffer([
    c[0] - s, c[1], c[2], c[0] + s, c[1], c[2],
    c[0], c[1] - s, c[2], c[0], c[1] + s, c[2],
    c[0], c[1], c[2] - s, c[0], c[1], c[2] + s,
  ]);
  gl.bindBuffer(gl.ARRAY_BUFFER, view.cuffBuffer);
  gl.vertexAttribPointer(linePosLoc, 3, gl.FLOAT, false, 0, 0);
  gl.drawArrays(gl.LINES, 0, 6);
}

function drawAll() {
  if (!state.model) return;
  draw3D();
  drawView(state.canvases.xz, [0, 2]);
  drawView(state.canvases.xy, [0, 1]);
  drawView(state.canvases.yz, [1, 2]);
  metrics();
}

function snapCuff() {
  const c = state.model.metadata.cuff_center_local;
  const target = state.model.metadata.snap_target_parent;
  const v = state.values;
  const m = matFromRpy(v.roll, v.pitch, v.yaw);
  const rc = [
    m[0][0] * c[0] + m[0][1] * c[1] + m[0][2] * c[2],
    m[1][0] * c[0] + m[1][1] * c[1] + m[1][2] * c[2],
    m[2][0] * c[0] + m[2][1] * c[1] + m[2][2] * c[2],
  ];
  setValues([target[0] - rc[0], target[1] - rc[1], target[2] - rc[2]], [v.roll, v.pitch, v.yaw]);
  setStatus("Snapped cuff center to wrist axis target.");
}

async function save() {
  const xyz = [state.values.x, state.values.y, state.values.z];
  const rpy = [state.values.roll, state.values.pitch, state.values.yaw];
  const res = await fetch("/api/save", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ xyz, rpy }),
  });
  const payload = await res.json();
  if (!res.ok) throw new Error(payload.error || "save failed");
  state.saved = { xyz, rpy };
  setStatus(`Saved. Backup: ${payload.backup}`);
}

async function init() {
  state.canvases = { xz: $("xz"), xy: $("xy"), yz: $("yz") };
  makeControls();
  const res = await fetch("/api/model");
  state.model = await res.json();
  $("urdfPath").textContent = state.model.urdf;
  state.saved = {
    xyz: state.model.origin.xyz.slice(),
    rpy: state.model.origin.rpy.slice(),
  };
  init3D();
  setValues(state.saved.xyz, state.saved.rpy);
  setStatus("Ready.");
  window.addEventListener("resize", drawAll);
  $("resetBtn").addEventListener("click", () => {
    setValues(state.saved.xyz, state.saved.rpy);
    setStatus("Reset to last saved URDF origin.");
  });
  $("snapBtn").addEventListener("click", snapCuff);
  $("copyBtn").addEventListener("click", async () => {
    const text = `<origin xyz="${fmt(state.values.x)} ${fmt(state.values.y)} ${fmt(state.values.z)}" rpy="${fmt(state.values.roll)} ${fmt(state.values.pitch)} ${fmt(state.values.yaw)}"/>`;
    await navigator.clipboard.writeText(text);
    setStatus("Copied origin XML.");
  });
  $("saveBtn").addEventListener("click", () => save().catch(err => setStatus(`Error: ${err.message}`)));
}

init().catch(err => {
  console.error(err);
  setStatus(`Error: ${err.message}`);
});
</script>
</body>
</html>
"""


class ThreadingHTTPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True


def make_handler(model: AlignmentModel):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *args) -> None:
            sys.stderr.write(
                f"{self.address_string()} - - [{self.log_date_time_string()}] {fmt % args}\n"
            )

        def send_json(self, payload: dict, status: int = 200) -> None:
            data = json.dumps(payload).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self) -> None:
            parsed = urllib.parse.urlparse(self.path)
            if parsed.path == "/":
                data = HTML.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return
            if parsed.path == "/api/model":
                try:
                    self.send_json(model.build())
                except Exception as exc:
                    self.send_json({"error": str(exc)}, 500)
                return
            self.send_error(404)

        def do_POST(self) -> None:
            parsed = urllib.parse.urlparse(self.path)
            if parsed.path != "/api/save":
                self.send_error(404)
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                body = self.rfile.read(length)
                payload = json.loads(body.decode("utf-8"))
                backup = model.save_origin(
                    [float(value) for value in payload["xyz"]],
                    [float(value) for value in payload["rpy"]],
                )
                self.send_json({"ok": True, "backup": str(backup)})
            except Exception as exc:
                self.send_json({"error": str(exc)}, 500)

    return Handler


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Serve a browser UI for tuning g1_hitter_racket hand/wrist alignment."
    )
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--asset-root", type=Path, default=DEFAULT_ASSET_ROOT)
    parser.add_argument("--joint", default="right_racket_fixed_joint")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--cuff-width", type=float, default=0.010)
    parser.add_argument("--insertion", type=float, default=0.010)
    parser.add_argument("--open", action="store_true", help="open the browser automatically")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    urdf = args.urdf.resolve()
    asset_root = args.asset_root.resolve()
    if not urdf.exists():
        raise FileNotFoundError(urdf)
    if not asset_root.exists():
        raise FileNotFoundError(asset_root)

    model = AlignmentModel(
        urdf_path=urdf,
        joint_name=args.joint,
        asset_root=asset_root,
        cuff_width=args.cuff_width,
        insertion=args.insertion,
    )
    # Build once at startup so path/mesh problems fail before the server starts.
    model.build()

    url = f"http://{args.host}:{args.port}/"
    with ThreadingHTTPServer((args.host, args.port), make_handler(model)) as httpd:
        print(f"Serving {url}", flush=True)
        print(f"URDF: {urdf}", flush=True)
        print(f"Joint: {args.joint}", flush=True)
        if args.open:
            webbrowser.open(url)
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nStopped.", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
