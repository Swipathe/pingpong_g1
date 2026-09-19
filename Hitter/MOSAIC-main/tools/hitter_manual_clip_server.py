#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import mimetypes
import os
import re
import subprocess
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RAW_ROOT = REPO_ROOT / "data" / "hitter_captures" / "mqy_capture"
DEFAULT_OUTPUT_ROOT = DEFAULT_RAW_ROOT / "manual_clips_review"


HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>HITTER Manual Clip Review</title>
  <style>
    :root { color-scheme: dark; font-family: system-ui, -apple-system, Segoe UI, sans-serif; }
    body { margin: 0; background: #101214; color: #eceff3; }
    header { padding: 12px 18px; border-bottom: 1px solid #2a2f36; display: flex; gap: 12px; align-items: center; }
    main { display: grid; grid-template-columns: minmax(640px, 1fr) 420px; gap: 16px; padding: 16px; }
    video { width: 100%; max-height: calc(100vh - 120px); background: #000; border: 1px solid #30363d; }
    button, input, select { background: #1c2128; color: #eceff3; border: 1px solid #3a424d; border-radius: 6px; padding: 8px 10px; font-size: 14px; }
    button { cursor: pointer; }
    button.primary { background: #2563eb; border-color: #3b82f6; }
    button.danger { background: #7f1d1d; border-color: #991b1b; }
    button:disabled { opacity: .5; cursor: not-allowed; }
    .panel { border: 1px solid #2a2f36; padding: 12px; background: #15191f; border-radius: 8px; }
    .row { display: flex; gap: 8px; align-items: center; margin: 8px 0; flex-wrap: wrap; }
    .row label { width: 92px; color: #aeb7c2; }
    .row input[type="number"] { width: 110px; }
    .status { white-space: pre-wrap; font-family: ui-monospace, SFMono-Regular, Menlo, monospace; color: #aeb7c2; font-size: 12px; }
    .clips { max-height: 220px; overflow: auto; font-size: 12px; line-height: 1.5; }
    .muted { color: #9aa4b2; }
    .time { font-variant-numeric: tabular-nums; color: #93c5fd; }
  </style>
</head>
<body>
  <header>
    <strong>HITTER Manual Clip Review</strong>
    <span id="videoTitle" class="muted"></span>
  </header>
  <main>
    <section>
      <video id="video" controls preload="metadata"></video>
    </section>
    <aside class="panel">
      <div class="row">
        <button id="prevBtn">Prev</button>
        <button id="nextBtn">Next</button>
        <select id="videoSelect"></select>
      </div>
      <div class="row">
        <label>Current</label>
        <span id="currentTime" class="time">0.000</span>
        <button id="markStart">Set start [I]</button>
        <button id="markEnd">Set end [O]</button>
      </div>
      <div class="row">
        <label>Start</label>
        <input id="startInput" type="number" step="0.001" min="0" value="0" />
        <button id="jumpStart">Jump</button>
      </div>
      <div class="row">
        <label>End</label>
        <input id="endInput" type="number" step="0.001" min="0" value="5" />
        <button id="jumpEnd">Jump</button>
      </div>
      <div class="row">
        <label>Stroke</label>
        <select id="strokeSelect">
          <option value="auto">auto from folder</option>
          <option value="forehand">forehand</option>
          <option value="backhand">backhand</option>
        </select>
      </div>
      <div class="row">
        <label>Clip note</label>
        <input id="noteInput" type="text" placeholder="optional" />
      </div>
      <div class="row">
        <button id="playRange">Play range [P]</button>
        <button id="saveBtn" class="primary">Save clip [S]</button>
        <button id="skipBtn">Skip video</button>
      </div>
      <div class="row">
        <button id="minusStart">start -0.1</button>
        <button id="plusStart">start +0.1</button>
        <button id="minusEnd">end -0.1</button>
        <button id="plusEnd">end +0.1</button>
      </div>
      <p class="muted">流程：播放视频，按 I 标开始，按 O 标结束，按 P 预览区间，按 S 保存。保存后可以继续在同一个原视频里选下一段。</p>
      <div class="panel">
        <strong>Saved clips</strong>
        <div id="clips" class="clips muted"></div>
      </div>
      <pre id="status" class="status"></pre>
    </aside>
  </main>
  <script>
    let state = null;
    let activeIndex = 0;
    let rangeTimer = null;
    const video = document.getElementById('video');
    const videoSelect = document.getElementById('videoSelect');
    const currentTime = document.getElementById('currentTime');
    const startInput = document.getElementById('startInput');
    const endInput = document.getElementById('endInput');
    const statusBox = document.getElementById('status');
    const clipsBox = document.getElementById('clips');

    function fmt(x) { return Number(x || 0).toFixed(3); }
    function setStatus(msg) { statusBox.textContent = msg; }
    function currentVideo() { return state.videos[activeIndex]; }
    function defaultEnd() {
      const dur = video.duration || currentVideo().duration || 0;
      endInput.value = fmt(Math.min(dur, Number(startInput.value) + 5.0));
    }
    async function loadState() {
      const res = await fetch('/api/state');
      state = await res.json();
      videoSelect.innerHTML = '';
      state.videos.forEach((v, i) => {
        const opt = document.createElement('option');
        opt.value = i;
        opt.textContent = `${String(i + 1).padStart(2, '0')} ${v.stroke}/${v.name} (${v.duration.toFixed(2)}s)`;
        videoSelect.appendChild(opt);
      });
      renderClips();
      loadVideo(activeIndex);
    }
    function renderClips() {
      if (!state.clips.length) {
        clipsBox.textContent = 'No clips saved yet.';
        return;
      }
      clipsBox.innerHTML = state.clips.map((c, i) =>
        `${i + 1}. ${c.stroke} ${c.source_name} ${Number(c.start).toFixed(3)}-${Number(c.end).toFixed(3)} -> ${c.output_name}`
      ).join('<br>');
    }
    function loadVideo(i) {
      activeIndex = Math.max(0, Math.min(i, state.videos.length - 1));
      videoSelect.value = activeIndex;
      const v = currentVideo();
      document.getElementById('videoTitle').textContent = `${activeIndex + 1}/${state.videos.length}: ${v.rel_path}`;
      video.src = `/video?i=${activeIndex}&t=${Date.now()}`;
      startInput.value = '0.000';
      endInput.value = fmt(Math.min(5, v.duration));
      setStatus(`Loaded ${v.rel_path}\nRaw source is read-only. Output root:\n${state.output_root}`);
    }
    async function saveClip() {
      const start = Number(startInput.value);
      const end = Number(endInput.value);
      if (!(end > start)) {
        setStatus('End must be greater than start.');
        return;
      }
      document.getElementById('saveBtn').disabled = true;
      setStatus('Saving clip with ffmpeg...');
      try {
        const res = await fetch('/api/save', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({
            index: activeIndex,
            start,
            end,
            stroke: document.getElementById('strokeSelect').value,
            note: document.getElementById('noteInput').value || ''
          })
        });
        const data = await res.json();
        if (!res.ok) throw new Error(data.error || 'save failed');
        setStatus(`Saved:\n${data.output}\nframes=${data.frames}, fps=${data.fps}`);
        document.getElementById('noteInput').value = '';
        await loadState();
        loadVideo(activeIndex);
      } catch (e) {
        setStatus(`Save failed: ${e.message}`);
      } finally {
        document.getElementById('saveBtn').disabled = false;
      }
    }
    function playRange() {
      clearInterval(rangeTimer);
      const start = Number(startInput.value);
      const end = Number(endInput.value);
      video.currentTime = start;
      video.play();
      rangeTimer = setInterval(() => {
        if (video.currentTime >= end) {
          video.pause();
          clearInterval(rangeTimer);
        }
      }, 40);
    }
    video.addEventListener('timeupdate', () => { currentTime.textContent = fmt(video.currentTime); });
    video.addEventListener('loadedmetadata', () => defaultEnd());
    videoSelect.addEventListener('change', () => loadVideo(Number(videoSelect.value)));
    document.getElementById('prevBtn').onclick = () => loadVideo(activeIndex - 1);
    document.getElementById('nextBtn').onclick = () => loadVideo(activeIndex + 1);
    document.getElementById('markStart').onclick = () => { startInput.value = fmt(video.currentTime); defaultEnd(); };
    document.getElementById('markEnd').onclick = () => { endInput.value = fmt(video.currentTime); };
    document.getElementById('jumpStart').onclick = () => { video.currentTime = Number(startInput.value); };
    document.getElementById('jumpEnd').onclick = () => { video.currentTime = Number(endInput.value); };
    document.getElementById('playRange').onclick = playRange;
    document.getElementById('saveBtn').onclick = saveClip;
    document.getElementById('skipBtn').onclick = () => loadVideo(activeIndex + 1);
    function adjust(id, delta) {
      const el = document.getElementById(id);
      el.value = fmt(Math.max(0, Number(el.value) + delta));
    }
    document.getElementById('minusStart').onclick = () => adjust('startInput', -0.1);
    document.getElementById('plusStart').onclick = () => adjust('startInput', 0.1);
    document.getElementById('minusEnd').onclick = () => adjust('endInput', -0.1);
    document.getElementById('plusEnd').onclick = () => adjust('endInput', 0.1);
    window.addEventListener('keydown', (e) => {
      if (['INPUT', 'SELECT'].includes(document.activeElement.tagName)) return;
      const k = e.key.toLowerCase();
      if (k === 'i') document.getElementById('markStart').click();
      if (k === 'o') document.getElementById('markEnd').click();
      if (k === 'p') playRange();
      if (k === 's') saveClip();
      if (k === 'n') loadVideo(activeIndex + 1);
      if (k === 'b') loadVideo(activeIndex - 1);
      if (k === ' ') { e.preventDefault(); video.paused ? video.play() : video.pause(); }
    });
    loadState().catch(e => setStatus(e.message));
  </script>
</body>
</html>
"""


def _probe_duration(path: Path) -> float:
    cmd = [
        "ffprobe",
        "-v",
        "error",
        "-show_entries",
        "format=duration",
        "-of",
        "default=noprint_wrappers=1:nokey=1",
        str(path),
    ]
    try:
        return float(subprocess.check_output(cmd, text=True).strip())
    except Exception:
        return 0.0


def _probe_frames_fps(path: Path) -> tuple[int, float]:
    cmd = [
        "ffprobe",
        "-v",
        "error",
        "-select_streams",
        "v:0",
        "-show_entries",
        "stream=nb_frames,r_frame_rate",
        "-of",
        "json",
        str(path),
    ]
    try:
        data = json.loads(subprocess.check_output(cmd, text=True))
        stream = data["streams"][0]
        rate = stream.get("r_frame_rate", "0/1").split("/")
        fps = float(rate[0]) / max(float(rate[1]), 1.0)
        frames = int(stream.get("nb_frames") or 0)
        return frames, fps
    except Exception:
        return 0, 0.0


def _load_manifest(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="") as fp:
        return list(csv.DictReader(fp))


def _write_manifest(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "clip_id",
        "stroke",
        "source_name",
        "source_path",
        "start",
        "end",
        "duration",
        "output_name",
        "output_path",
        "frames",
        "fps",
        "note",
        "created_at",
    ]
    with path.open("w", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    path.with_suffix(".json").write_text(json.dumps(rows, indent=2, ensure_ascii=False), encoding="utf-8")


def _safe_label(value: str) -> str:
    label = re.sub(r"[^A-Za-z0-9_-]+", "_", value.strip()).strip("_").lower()
    if not label:
        raise ValueError("empty clip label")
    return label[:64]


class ClipApp:
    def __init__(self, raw_root: Path, output_root: Path) -> None:
        self.raw_root = raw_root
        self.output_root = output_root
        self.manifest_path = output_root / "_index" / "manual_clips_manifest.csv"
        self.videos = self._discover_videos()
        self.output_root.mkdir(parents=True, exist_ok=True)

    def _discover_videos(self) -> list[dict[str, object]]:
        videos: list[dict[str, object]] = []
        for path in sorted(self.raw_root.iterdir()):
            if not path.is_file() or path.suffix.lower() not in {".mp4", ".mov", ".m4v"}:
                continue
            source_stem = path.stem.replace("_raw", "").lower()
            if source_stem.startswith("forehand"):
                stroke = "forehand"
            elif source_stem.startswith("backhand"):
                stroke = "backhand"
            else:
                stroke = "unclassified"
            videos.append(
                {
                    "path": str(path),
                    "rel_path": str(path.relative_to(self.raw_root)),
                    "name": path.name,
                    "stem": source_stem,
                    "stroke": stroke,
                    "duration": _probe_duration(path),
                }
            )
        return videos

    def state(self) -> dict[str, object]:
        rows = _load_manifest(self.manifest_path)
        return {"raw_root": str(self.raw_root), "output_root": str(self.output_root), "videos": self.videos, "clips": rows}

    def save_clip(self, payload: dict[str, object]) -> dict[str, object]:
        index = int(payload["index"])
        start = float(payload["start"])
        end = float(payload["end"])
        if index < 0 or index >= len(self.videos):
            raise ValueError("video index out of range")
        if not (end > start >= 0):
            raise ValueError("invalid start/end")

        video = self.videos[index]
        source = Path(str(video["path"]))
        duration = float(video["duration"])
        if end > duration + 0.05:
            raise ValueError(f"end exceeds source duration {duration:.3f}s")

        stroke = str(payload.get("stroke") or "auto")
        if stroke == "auto":
            stroke = str(video["stroke"])
        stroke = _safe_label(stroke)

        rows = _load_manifest(self.manifest_path)
        clip_id = len(rows) + 1
        output_name = f"{stroke}_manual_{clip_id:03d}_{str(video['stem'])[:8]}_{start:.3f}_{end:.3f}_50fps.mp4"
        output_path = self.output_root / stroke / output_name
        output_path.parent.mkdir(parents=True, exist_ok=True)
        cmd = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "error",
            "-y",
            "-ss",
            f"{start:.3f}",
            "-i",
            str(source),
            "-t",
            f"{end - start:.3f}",
            "-vf",
            "fps=50,format=yuv420p",
            "-an",
            str(output_path),
        ]
        subprocess.run(cmd, check=True)
        frames, fps = _probe_frames_fps(output_path)
        row = {
            "clip_id": clip_id,
            "stroke": stroke,
            "source_name": str(video["name"]),
            "source_path": str(source),
            "start": f"{start:.3f}",
            "end": f"{end:.3f}",
            "duration": f"{end - start:.3f}",
            "output_name": output_name,
            "output_path": str(output_path),
            "frames": frames,
            "fps": f"{fps:.3f}",
            "note": str(payload.get("note") or ""),
            "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        rows.append(row)
        _write_manifest(self.manifest_path, rows)
        return {"output": str(output_path), "frames": frames, "fps": fps, "row": row}


def make_handler(app: ClipApp):
    class Handler(BaseHTTPRequestHandler):
        server_version = "HitterClipServer/0.1"

        def log_message(self, fmt: str, *args: object) -> None:
            print(f"{self.address_string()} - {fmt % args}")

        def _send_json(self, data: object, status: int = 200) -> None:
            body = json.dumps(data, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _send_text(self, text: str, content_type: str = "text/html; charset=utf-8") -> None:
            body = text.encode("utf-8")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:  # noqa: N802
            parsed = urlparse(self.path)
            if parsed.path == "/":
                self._send_text(HTML)
                return
            if parsed.path == "/api/state":
                self._send_json(app.state())
                return
            if parsed.path == "/video":
                qs = parse_qs(parsed.query)
                index = int(qs.get("i", ["0"])[0])
                if index < 0 or index >= len(app.videos):
                    self.send_error(404)
                    return
                self._serve_file(Path(str(app.videos[index]["path"])))
                return
            self.send_error(404)

        def do_POST(self) -> None:  # noqa: N802
            parsed = urlparse(self.path)
            length = int(self.headers.get("Content-Length", "0"))
            try:
                payload = json.loads(self.rfile.read(length).decode("utf-8") or "{}")
                if parsed.path == "/api/save":
                    self._send_json(app.save_clip(payload))
                    return
            except Exception as exc:  # noqa: BLE001 - return useful UI error.
                self._send_json({"error": str(exc)}, status=400)
                return
            self.send_error(404)

        def _serve_file(self, path: Path) -> None:
            file_size = path.stat().st_size
            range_header = self.headers.get("Range")
            content_type = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
            start = 0
            end = file_size - 1
            status = HTTPStatus.OK
            if range_header:
                units, rng = range_header.split("=", 1)
                if units == "bytes":
                    start_s, end_s = rng.split("-", 1)
                    if start_s:
                        start = int(start_s)
                    if end_s:
                        end = int(end_s)
                    status = HTTPStatus.PARTIAL_CONTENT
            start = max(0, min(start, file_size - 1))
            end = max(start, min(end, file_size - 1))
            length = end - start + 1
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Accept-Ranges", "bytes")
            self.send_header("Content-Length", str(length))
            if status == HTTPStatus.PARTIAL_CONTENT:
                self.send_header("Content-Range", f"bytes {start}-{end}/{file_size}")
            self.end_headers()
            with path.open("rb") as fp:
                fp.seek(start)
                remaining = length
                while remaining > 0:
                    chunk = fp.read(min(1024 * 1024, remaining))
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    remaining -= len(chunk)

    return Handler


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Local manual clipping UI for HITTER iPhone videos.")
    parser.add_argument("--raw-root", type=Path, default=DEFAULT_RAW_ROOT)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    app = ClipApp(args.raw_root, args.output_root)
    handler = make_handler(app)
    server = ThreadingHTTPServer((args.host, args.port), handler)
    print(f"Raw videos: {args.raw_root}")
    print(f"Output root: {args.output_root}")
    print(f"Found videos: {len(app.videos)}")
    print(f"Open: http://{args.host}:{args.port}")
    server.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
