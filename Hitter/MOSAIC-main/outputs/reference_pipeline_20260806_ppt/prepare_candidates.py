"""Extract traceable video frames and contact sheets for presentation review."""

import json
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
CAPTURE = ROOT / "data/hitter_captures/20260806_mqy_capture"
MOTIONS = ROOT / "data/hitter_motions"
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"


def run(*args):
    return subprocess.run(args, check=True, capture_output=True, text=True).stdout


def main():
    alignment = json.loads((MOTIONS / "20260806_mqy_g1_npz_raw_peak43_aligned/_index/strike43_alignment_manifest.json").read_text())
    converted = json.loads((MOTIONS / "20260806_mqy_g1_npz_raw_pre_align/_index/gvhmr_hitter_npz_manifest.json").read_text())
    by_name = {row["motion_name"]: row for row in converted["motions"]}
    crop_records = json.loads((CAPTURE / "manual_clips_review/_index/manual_clips_manifest.json").read_text())
    crop_by_name = {row["output_name"]: row for row in crop_records}
    frames_dir = OUT / "frames"
    frames_dir.mkdir(exist_ok=True)
    records = []
    for stroke, prefix in [("forehand", "F"), ("backhand", "B")]:
        rows = sorted((r for r in alignment["rows"] if r["class"] == stroke), key=lambda r: r["source"])
        panels = []
        for row_index, row in enumerate(rows):
            name = Path(row["source"]).stem.removesuffix("__unitree_g1")
            conversion = by_name[name]
            video = CAPTURE / "manual_clips_review" / stroke / (name + ".mp4")
            probe = json.loads(run("ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries", "stream=nb_frames,avg_frame_rate,width,height", "-of", "json", str(video)))["streams"][0]
            count = int(probe["nb_frames"])
            assert count == conversion["source_frames"], (video, count, conversion["source_frames"])
            assert probe["avg_frame_rate"] == "50/1", probe
            assert probe["width"] == 1080 and probe["height"] == 1920, probe
            for column, (delta, phase) in enumerate([(-12, "Before peak"), (0, "Near peak"), (12, "After peak")]):
                candidate_id = f"{prefix}{row_index * 3 + column + 1}"
                pre_frame = max(0, min(row["frames"] - 1, row["source_peak_frame"] + delta))
                source_float = pre_frame * (count - 1) / (row["frames"] - 1)
                frame = round(source_float)
                # Extract the exact decoded source frame; all annotation is separate.
                output = frames_dir / f"{candidate_id}_{name}_frame{frame:03d}.png"
                run("ffmpeg", "-v", "error", "-i", str(video), "-vf", f"select=eq(n\\,{frame})", "-frames:v", "1", "-update", "1", "-n", str(output))
                original = crop_by_name[name + ".mp4"]
                record = {
                    "id": candidate_id,
                    "stroke": stroke,
                    "phase_relative_to_robot_speed_peak": phase,
                    "video": str(video),
                    "video_frame_zero_based": frame,
                    "video_time_seconds": frame / 50,
                    "original_recording": str(ROOT / original["source_path"]),
                    "original_time_seconds_approx": float(original["start"]) + frame / 50,
                    "gvhmr_result": conversion["source_gvhmr_result"],
                    "gvhmr_frame_zero_based": frame,
                    "npz_pre_align": str(ROOT / row["source"]),
                    "npz_pre_align_frame_nearest_video_frame": frame * (row["frames"] - 1) / (count - 1),
                    "npz_aligned": str(ROOT / row["output"]),
                    "npz_alignment_shift_frames": row["applied_shift_frames"],
                    "candidate_target_pre_align_frame": pre_frame,
                    "unannotated_image": str(output),
                    "contact_sheet_crop_xywh": [0, 320, 1080, 1600],
                    "note": "Human source frame is exact. NPZ time mapping may be fractional after resampling. Robot speed peak is not a measured ball contact event.",
                }
                records.append(record)
                clip_id = name.split("_manual_")[1].split("_")[0]
                panels.append((output, f"{candidate_id}  |  Clip {clip_id}  |  {phase}", f"Video frame {frame}  -  {frame/50:.2f} s"))
        command = ["ffmpeg", "-v", "error"]
        for path, _, _ in panels:
            command += ["-i", str(path)]
        filters = []
        for i, (_, heading, subheading) in enumerate(panels):
            filters.append(f"[{i}:v]crop=1080:1600:0:320,scale=320:474,pad=344:546:12:60:color=0xf3f6fa,drawtext=fontfile={FONT}:text='{heading}':fontcolor=0x172b45:fontsize=15:x=12:y=12,drawtext=fontfile={FONT}:text='{subheading}':fontcolor=0x52637a:fontsize=13:x=12:y=35[p{i}]")
        layout = "|".join(f"{(i%3)*344}_{(i//3)*546}" for i in range(len(panels)))
        filters.append("".join(f"[p{i}]" for i in range(len(panels))) + f"xstack=inputs=9:layout={layout}[sheet]")
        sheet = OUT / f"{stroke}_candidates.png"
        command += ["-filter_complex_threads", "1", "-filter_complex", ";".join(filters), "-map", "[sheet]", "-frames:v", "1", "-update", "1", "-n", str(sheet)]
        run(*command)
        print(f"Created {sheet}", flush=True)
    (OUT / "candidate_manifest.json").write_text(json.dumps({"dataset": "20260806_mqy_capture", "purpose": "Frame selection only; not the final pipeline figure", "candidates": records}, indent=2) + "\n")
    print(f"Saved {len(records)} exact source frames and traceability records.")


if __name__ == "__main__":
    main()
