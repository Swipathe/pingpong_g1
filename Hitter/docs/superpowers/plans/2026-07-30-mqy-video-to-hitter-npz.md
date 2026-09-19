# MQY Video to HITTER NPZ Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Provide four independently runnable MQY processing steps with consistent default directories: manual clipping, GVHMR inference, HITTER NPZ conversion, and frame-43 racket-speed alignment.

**Architecture:** Existing MOSAIC tools remain separate command-line programs. Each tool receives MQY-specific defaults derived from the repository location, while command-line arguments remain the explicit override mechanism. Generated files flow through new directories without modifying the two source MP4 files or existing HITTER datasets.

**Tech Stack:** Python 3.10, argparse, NumPy, ffmpeg/ffprobe, GVHMR, GMR, PyBullet, SMPL-X.

## Global Constraints

- Address YHL before every user-facing response.
- Ask YHL before acting on uncertain decisions.
- Do not add compatibility code unless YHL requests it.
- Do not automatically run GVHMR inference or NPZ conversion.
- Do not overwrite generated output unless YHL explicitly passes an overwrite option.
- Preserve `/home/yhl/Desktop/Hitter/MOSAIC-main/data/hitter_captures/mqy_capture/forehand.mp4`.
- Preserve `/home/yhl/Desktop/Hitter/MOSAIC-main/data/hitter_captures/mqy_capture/backhand.mp4`.

## File Structure

- Modify: `MOSAIC-main/tools/hitter_manual_clip_server.py` — discover root-level MQY videos and save classified clips.
- Create: `MOSAIC-main/tools/run_mqy_gvhmr.py` — invoke GVHMR folder inference separately for forehand and backhand.
- Modify: `MOSAIC-main/tools/convert_gvhmr_results_to_hitter_npz.py` — set MQY defaults and classify results by directory.
- Create: `MOSAIC-main/tools/align_mqy_hitter_strike43.py` — align both stroke classes using the saved racket-link trajectory.
- Create: `MOSAIC-main/tools/validate_mqy_hitter_npz.py` — validate NPZ structure and optional frame-43 alignment.
- Create: `MOSAIC-main/tests/test_mqy_data_pipeline.py` — focused unit tests for path flow, discovery, classification, alignment, and validation.
- Create: `MOSAIC-main/MQY_VIDEO_TO_NPZ.md` — commands and checkpoints for YHL.

---

### Task 1: Root-Level Video Discovery and Classified Clip Output

**Files:**

- Modify: `MOSAIC-main/tools/hitter_manual_clip_server.py`
- Create: `MOSAIC-main/tests/test_mqy_data_pipeline.py`

**Interfaces:**

- Consumes: root-level `mqy_capture/*.mp4`.
- Produces: `manual_clips_review/<stroke>/*.mp4` and `_index/manual_clips_manifest.{csv,json}`.

- [ ] **Step 1: Write failing path and discovery tests**

Add tests that import the tool by file path, create `forehand.mp4`,
`backhand.mp4`, and a nested generated MP4, then assert only the two root
files are discovered:

```python
def test_clip_app_discovers_only_root_level_videos(tmp_path, monkeypatch):
    raw = tmp_path / "mqy_capture"
    raw.mkdir()
    (raw / "forehand.mp4").touch()
    (raw / "backhand.mp4").touch()
    nested = raw / "manual_clips_review" / "forehand"
    nested.mkdir(parents=True)
    (nested / "generated.mp4").touch()
    monkeypatch.setattr(clip_module, "_probe_duration", lambda _: 1.0)

    app = clip_module.ClipApp(raw, raw / "manual_clips_review")

    assert [Path(row["path"]).name for row in app.videos] == [
        "backhand.mp4",
        "forehand.mp4",
    ]
```

Add a save test with `subprocess.run` and probe functions patched. Assert a
selected `forehand` clip is written under `manual_clips_review/forehand/`,
and assert `auto` is rejected.

- [ ] **Step 2: Run tests and verify failure**

Run:

```bash
cd /home/yhl/Desktop/Hitter/MOSAIC-main
python -m pytest tests/test_mqy_data_pipeline.py -k clip -v
```

Expected: discovery returns no root-level videos, or save output is not
classified.

- [ ] **Step 3: Implement MQY defaults and strict classification**

Use repository-relative defaults:

```python
DEFAULT_RAW_ROOT = REPO_ROOT / "data" / "hitter_captures" / "mqy_capture"
DEFAULT_OUTPUT_ROOT = DEFAULT_RAW_ROOT / "manual_clips_review"
```

Change discovery to:

```python
for path in sorted(self.raw_root.iterdir()):
    if not path.is_file() or path.suffix.lower() not in {".mp4", ".mov", ".m4v"}:
        continue
```

Record the initial stroke as `unclassified`. Reject `auto` in `save_clip`;
accept only `forehand` or `backhand`. Save to:

```python
output_path = self.output_root / stroke / output_name
```

Move the manifest to:

```python
self.manifest_path = output_root / "_index" / "manual_clips_manifest.csv"
```

Update the page wording so YHL explicitly selects the class.

- [ ] **Step 4: Run focused tests**

Run:

```bash
python -m pytest tests/test_mqy_data_pipeline.py -k clip -v
```

Expected: all clip tests pass.

- [ ] **Step 5: Perform a read-only default-path smoke check**

Run:

```bash
python -c "from tools.hitter_manual_clip_server import ClipApp, DEFAULT_RAW_ROOT, DEFAULT_OUTPUT_ROOT; app=ClipApp(DEFAULT_RAW_ROOT, DEFAULT_OUTPUT_ROOT); print(DEFAULT_RAW_ROOT); print(DEFAULT_OUTPUT_ROOT); print([v['name'] for v in app.videos])"
```

Expected: paths point to `mqy_capture` and the names are `backhand.mp4` and
`forehand.mp4`.

### Task 2: Stepwise GVHMR Folder Runner

**Files:**

- Create: `MOSAIC-main/tools/run_mqy_gvhmr.py`
- Modify: `MOSAIC-main/tests/test_mqy_data_pipeline.py`

**Interfaces:**

- Consumes: `manual_clips_review/{forehand,backhand}/*.mp4`.
- Produces: `gvhmr_results/{forehand,backhand}/<clip-name>/hmr4d_results.pt`.
- Executes: `/home/yhl/Desktop/Hitter/GVHMR/tools/demo/demo_folder.py`.

- [ ] **Step 1: Write failing command-construction tests**

Test these constants and the command builder:

```python
assert gvhmr_module.DEFAULT_GVHMR_ROOT == Path("/home/yhl/Desktop/Hitter/GVHMR")
assert gvhmr_module.DEFAULT_CLIP_ROOT.name == "manual_clips_review"
assert gvhmr_module.DEFAULT_OUTPUT_ROOT.name == "gvhmr_results"
assert gvhmr_module.build_command(
    Path("/repo/GVHMR"),
    Path("/clips/forehand"),
    Path("/results/forehand"),
    "python",
) == [
    "python",
    "/repo/GVHMR/tools/demo/demo_folder.py",
    "-f",
    "/clips/forehand",
    "-d",
    "/results/forehand",
    "-s",
]
```

Test that missing clip files or a missing GVHMR entry point raises a
`FileNotFoundError` containing the exact missing path.

- [ ] **Step 2: Run tests and verify failure**

Run:

```bash
python -m pytest tests/test_mqy_data_pipeline.py -k gvhmr -v
```

Expected: import fails because the runner does not exist.

- [ ] **Step 3: Implement the runner**

Define these defaults:

```python
HITTER_ROOT = REPO_ROOT.parent
DEFAULT_GVHMR_ROOT = HITTER_ROOT / "GVHMR"
DEFAULT_CLIP_ROOT = REPO_ROOT / "data" / "hitter_captures" / "mqy_capture" / "manual_clips_review"
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "data" / "hitter_captures" / "mqy_capture" / "gvhmr_results"
```

Support:

```text
--stroke {all,forehand,backhand}
--python <GVHMR environment Python>
--dry-run
```

For each selected class, validate that at least one MP4 exists, build the
official static-camera folder-demo command, print it, and run it with
`cwd=DEFAULT_GVHMR_ROOT`. `--dry-run` performs validation and printing
without inference.

- [ ] **Step 4: Run focused tests and syntax check**

Run:

```bash
python -m pytest tests/test_mqy_data_pipeline.py -k gvhmr -v
python -m py_compile tools/run_mqy_gvhmr.py
```

Expected: all GVHMR runner tests pass and compilation succeeds.

### Task 3: MQY Defaults and Directory-Based Classification in the Converter

**Files:**

- Modify: `MOSAIC-main/tools/convert_gvhmr_results_to_hitter_npz.py`
- Modify: `MOSAIC-main/tests/test_mqy_data_pipeline.py`

**Interfaces:**

- Consumes: `gvhmr_results/<stroke>/<motion>/hmr4d_results.pt`.
- Produces: `mqy_g1_npz_pre_align/<stroke>/<motion>__unitree_g1.npz`.

- [ ] **Step 1: Write failing default and classification tests**

Assert that `parse_args()` without path arguments yields:

```python
input_root = data/hitter_captures/mqy_capture/gvhmr_results
clip_root = data/hitter_captures/mqy_capture/manual_clips_review
output_root = data/hitter_motions/mqy_g1_npz_pre_align
gmr_root = /home/yhl/Desktop/Hitter/GMR-master
smplx_root = /home/yhl/Desktop/Hitter/GMR-master/assets/body_models
backhand_right_wrist_roll_offset = 0.55
forehand_right_arm_motion_scale = 1.55
compressed = True
overwrite = False
```

Add:

```python
def test_infer_label_from_result_path():
    root = Path("/results")
    assert converter.infer_label(root / "forehand" / "clip_001" / "hmr4d_results.pt", root) == "forehand"
    assert converter.infer_label(root / "backhand" / "clip_002" / "hmr4d_results.pt", root) == "backhand"
```

Assert an unknown class raises `ValueError`; it must not create an
`unknown/` output directory.

- [ ] **Step 2: Run tests and verify failure**

Run:

```bash
python -m pytest tests/test_mqy_data_pipeline.py -k converter -v
```

Expected: required arguments prevent parsing and the old label function does
not classify by relative directory.

- [ ] **Step 3: Implement defaults and classification**

Add:

```python
DEFAULT_CAPTURE_ROOT = REPO_ROOT / "data" / "hitter_captures" / "mqy_capture"
DEFAULT_INPUT_ROOT = DEFAULT_CAPTURE_ROOT / "gvhmr_results"
DEFAULT_CLIP_ROOT = DEFAULT_CAPTURE_ROOT / "manual_clips_review"
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "data" / "hitter_motions" / "mqy_g1_npz_pre_align"
```

Make `--input-root`, `--clip-root`, and `--output-root` optional with these
defaults. Set the approved motion defaults, including compressed output.
Classify each result using its first relative directory component and reject
anything outside `forehand` or `backhand`.

Before initializing GMR or PyBullet, validate the input root, GMR root,
SMPL-X model, URDF, and reference root. Each exception must display the
missing path.

- [ ] **Step 4: Run focused tests**

Run:

```bash
python -m pytest tests/test_mqy_data_pipeline.py -k converter -v
python -m py_compile tools/convert_gvhmr_results_to_hitter_npz.py
```

Expected: converter tests pass and compilation succeeds.

### Task 4: Align Both Stroke Classes to Frame 43

**Files:**

- Create: `MOSAIC-main/tools/align_mqy_hitter_strike43.py`
- Modify: `MOSAIC-main/tests/test_mqy_data_pipeline.py`

**Interfaces:**

- Consumes: the seven-field, 94-frame NPZ files under `mqy_g1_npz_pre_align`.
- Produces: aligned NPZ files and `_index/strike43_alignment_manifest.{json,csv}`.

- [ ] **Step 1: Write failing alignment tests**

Create a synthetic 94-frame motion with 31 body trajectories and a
right-racket speed peak at frame 38. Assert:

```python
aligned, row = align_module.align_motion(motion, target_frame=43)
assert row["source_peak_frame"] == 38
assert row["aligned_peak_frame"] == 43
assert row["applied_shift_frames"] == 5
assert aligned["joint_pos"].shape == (94, 29)
assert aligned["body_pos_w"].shape == (94, 31, 3)
assert np.isfinite(aligned["body_ang_vel_w"]).all()
```

Run the dataset function on one forehand and one backhand NPZ and assert
both outputs exist and both manifest rows report frame 43.

- [ ] **Step 2: Run tests and verify failure**

Run:

```bash
python -m pytest tests/test_mqy_data_pipeline.py -k align -v
```

Expected: import fails because the MQY alignment script does not exist.

- [ ] **Step 3: Implement alignment**

Use:

```python
DEFAULT_INPUT_ROOT = REPO_ROOT / "data" / "hitter_motions" / "mqy_g1_npz_pre_align"
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "data" / "hitter_motions" / "mqy_g1_npz_peak43_aligned"
TARGET_FRAME = 43
RIGHT_RACKET_BODY_INDEX = 30
```

Load and validate the seven fields. Compute the source peak from the
finite-difference velocity of `body_pos_w[:, 30, :]`, shift all position and
orientation sequences with edge hold, then recompute all three velocity
fields. Process both class directories identically.

Reject an existing output root unless `--overwrite` is explicitly supplied.
Write source peak, shift, aligned peak, speeds, class, source, and output to
the JSON and CSV manifests.

- [ ] **Step 4: Run focused tests**

Run:

```bash
python -m pytest tests/test_mqy_data_pipeline.py -k align -v
python -m py_compile tools/align_mqy_hitter_strike43.py
```

Expected: alignment tests pass and compilation succeeds.

### Task 5: NPZ Validation Checkpoint

**Files:**

- Create: `MOSAIC-main/tools/validate_mqy_hitter_npz.py`
- Modify: `MOSAIC-main/tests/test_mqy_data_pipeline.py`

**Interfaces:**

- Consumes: a dataset root containing forehand/backhand NPZ files.
- Produces: console PASS/FAIL rows and a nonzero process exit code on failure.

- [ ] **Step 1: Write failing validation tests**

Create a valid synthetic NPZ and assert zero errors. Then independently
alter the frame count, joint count, body count, FPS, quaternion norm, and a
value to NaN; assert each defect produces a specific error containing the
file path.

- [ ] **Step 2: Run tests and verify failure**

Run:

```bash
python -m pytest tests/test_mqy_data_pipeline.py -k validate -v
```

Expected: import fails because the validator does not exist.

- [ ] **Step 3: Implement validation**

Validate exact keys and shapes:

```text
fps                 (1,)
joint_pos           (94, 29)
joint_vel           (94, 29)
body_pos_w          (94, 31, 3)
body_quat_w         (94, 31, 4)
body_lin_vel_w      (94, 31, 3)
body_ang_vel_w      (94, 31, 3)
```

Require FPS 50, finite values, quaternion norms within `1e-3` of one, and
minimum body Z at least `0.025 - 1e-4`. When `--require-peak43` is passed,
require the recomputed right-racket speed peak at frame 43.

- [ ] **Step 4: Run focused tests**

Run:

```bash
python -m pytest tests/test_mqy_data_pipeline.py -k validate -v
python -m py_compile tools/validate_mqy_hitter_npz.py
```

Expected: validation tests pass and compilation succeeds.

### Task 6: YHL Step-by-Step Operating Guide

**Files:**

- Create: `MOSAIC-main/MQY_VIDEO_TO_NPZ.md`

**Interfaces:**

- Consumes: the completed four-step tools.
- Produces: exact commands and checkpoints YHL can execute sequentially.

- [ ] **Step 1: Write the guide**

Include:

1. GVHMR and GMR repository locations.
2. Official model-file directory trees.
3. environment creation and dependency checks.
4. clip-server command and UI keys.
5. GVHMR dry-run and real commands.
6. expected `hmr4d_results.pt` discovery command.
7. converter command using defaults.
8. pre-alignment validation command.
9. alignment command.
10. final validation command with `--require-peak43`.

Every step must state its working directory, expected output, and the
condition required before moving to the next step.

- [ ] **Step 2: Check every documented path**

Run a small Python path audit that imports all four tools and prints their
default input/output paths. Compare the output with the guide.

- [ ] **Step 3: Check all documented CLI options**

Run:

```bash
python tools/hitter_manual_clip_server.py --help
python tools/run_mqy_gvhmr.py --help
python tools/convert_gvhmr_results_to_hitter_npz.py --help
python tools/align_mqy_hitter_strike43.py --help
python tools/validate_mqy_hitter_npz.py --help
```

Expected: every command exits successfully and matches the guide.

### Task 7: Full Non-Inference Verification

**Files:**

- Test: `MOSAIC-main/tests/test_mqy_data_pipeline.py`

**Interfaces:**

- Consumes: all implementation tasks.
- Produces: evidence that path flow and local transformations work without running GVHMR/GMR.

- [ ] **Step 1: Run the complete focused test file**

Run:

```bash
cd /home/yhl/Desktop/Hitter/MOSAIC-main
python -m pytest tests/test_mqy_data_pipeline.py -v
```

Expected: all tests pass.

- [ ] **Step 2: Compile every modified or created Python tool**

Run:

```bash
python -m py_compile \
  tools/hitter_manual_clip_server.py \
  tools/run_mqy_gvhmr.py \
  tools/convert_gvhmr_results_to_hitter_npz.py \
  tools/align_mqy_hitter_strike43.py \
  tools/validate_mqy_hitter_npz.py
```

Expected: no output and exit code zero.

- [ ] **Step 3: Confirm original videos are unchanged**

Record and compare size, modification time, and SHA-256 for the two input
MP4 files before and after implementation. Expected: identical values.

- [ ] **Step 4: Confirm no processing jobs were launched**

Verify that no new manual clips, GVHMR results, or NPZ datasets were produced
by implementation tests. Tests must use temporary directories only.

