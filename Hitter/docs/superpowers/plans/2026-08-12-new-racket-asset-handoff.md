# New Racket Asset Handoff Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build verified `.tar.gz` and `.zip` handoff archives containing only the new rigid racket assets, minimal configuration patch, Chinese deployment guide, manifest, and checksums.

**Architecture:** Assemble a new staging directory outside both source projects while preserving project-relative paths below `payload/`. Add human-readable patch/reference material rather than overwriting project code, then validate the staged assets with MuJoCo, USD reference checks, Hydra configuration composition, file hashing, and archive extraction.

**Tech Stack:** Bash/coreutils, `apply_patch`, Python 3.10, MuJoCo 3.11, Hydra, USD text references, `tar`, `zip`, SHA-256.

## Global Constraints

- Do not modify existing MOSAIC or RobotBridge asset defaults.
- Do not include motion NPZ data, checkpoints, ONNX files, logs, caches, or temporary conversion outputs.
- Preserve the relative layout rooted at `payload/MOSAIC-main` and `payload/RobotBridge2`.
- Isaac runtime selection must use `HITTER_G1_HITTER_RACKET_USD_PATH`.
- MuJoCo runtime selection must use `robot.asset.asset_file=g1_29dof_hitter_racket_table_tennis_cad_v1.xml`.
- Document that `playback_slowdown` is unsupported in the current RobotBridge directory.
- Preserve the known missing `right_wrist_yaw_link` visual geom; do not repair it in this handoff.
- Build both `hitter_new_racket_assets_v1_20260812.tar.gz` and `hitter_new_racket_assets_v1_20260812.zip`.
- The workspace root is not a Git repository, so this packaging task produces files without commits or worktrees.

---

### Task 1: Assemble the asset payload

**Files:**
- Create: `hitter_new_racket_assets_v1_20260812/payload/MOSAIC-main/source/whole_body_tracking/whole_body_tracking/assets/unitree_description/usd/g1_hitter_racket_cad_v1/main_motion_compatible.usda`
- Create: `hitter_new_racket_assets_v1_20260812/payload/MOSAIC-main/source/whole_body_tracking/whole_body_tracking/assets/unitree_description/usd/g1_hitter_racket_cad_v1/cad_geometry.usd`
- Create: `hitter_new_racket_assets_v1_20260812/payload/MOSAIC-main/source/whole_body_tracking/whole_body_tracking/assets/unitree_description/urdf/g1_hitter_racket_cad_v1/main.urdf`
- Create: `hitter_new_racket_assets_v1_20260812/payload/MOSAIC-main/source/whole_body_tracking/whole_body_tracking/assets/unitree_description/meshes/g1_hitter_racket/{connector_visual,racket_holder_visual,racket_visual}.stl`
- Create: `hitter_new_racket_assets_v1_20260812/payload/RobotBridge2/deploy/data/assets/g1/g1_29dof_hitter_racket_table_tennis_cad_v1.xml`
- Create: `hitter_new_racket_assets_v1_20260812/payload/RobotBridge2/deploy/data/assets/g1/meshes/{connector_visual,racket_holder_visual,racket_visual}.stl`
- Create: `hitter_new_racket_assets_v1_20260812/source_stl/assembled_fully.STL`

**Interfaces:**
- Consumes: Verified source files from the current `MOSAIC-main`, `RobotBridge2_20260722_1726`, and `/home/yhl/Downloads/assembled_fully.STL`.
- Produces: A runtime payload whose files can be merged into the colleague's existing project without replacing legacy assets.

- [ ] **Step 1: Assert every source file exists and record its size**

Run `test -f` for all eleven source paths and `stat -c '%n %s'` for the resulting list. Expected: every assertion succeeds and no size is zero.

- [ ] **Step 2: Create the exact staging directories**

Use `mkdir -p` only below `/home/yhl/Desktop/Hitter/hitter_new_racket_assets_v1_20260812` for the paths listed in this task.

- [ ] **Step 3: Copy assets without changing the sources**

Use `cp -p` for each explicit source/destination pair. Expected: three STL hashes match across MOSAIC, RobotBridge, and `new_racket_v1` source copies.

- [ ] **Step 4: Verify payload scope**

Run `find` and reject any file matching `*.npz`, `*.pt`, `*.onnx`, `*.log`, `*.pyc`, `.asset_hash`, or files below a `configuration/` directory.

### Task 2: Create the configuration patch and deployment guide

**Files:**
- Create: `hitter_new_racket_assets_v1_20260812/patches/mosaic_g1_asset_switch.patch`
- Create: `hitter_new_racket_assets_v1_20260812/patches/g1_asset_switch_reference.txt`
- Create: `hitter_new_racket_assets_v1_20260812/README_部署说明.md`

**Interfaces:**
- Consumes: Current `robots/g1.py`, the asset layout from Task 1, and the approved design specification.
- Produces: A minimal optional code change plus commands parameterized by `$MOSAIC_ROOT`, `$ROBOTBRIDGE_ROOT`, `$MOTION_PATH`, `$CHECKPOINT_PATH`, and `$ONNX_PATH`.

- [ ] **Step 1: Write a minimal `g1.py` patch**

The patch must retain the old default `g1_hitter_racket/main.usda` and add only an environment-variable-selected USD path for `G1_HITTER_RACKET_CFG`. It must not replace the colleague's complete `g1.py`.

- [ ] **Step 2: Write the exact reference block**

The reference file must show `_G1_HITTER_RACKET_USD_PATH = os.environ.get(...)` and `_make_usd_spawn_cfg(usd_path=_G1_HITTER_RACKET_USD_PATH)` so a failed patch can be applied manually and reviewed.

- [ ] **Step 3: Write the Chinese deployment guide**

Cover prerequisites, paths, copy commands, patch check/application, Isaac reference replay, Isaac checkpoint playback, single-GPU training, multi-GPU training, MuJoCo standalone viewing, RobotBridge ONNX launch, restoration to legacy assets, validation, and known limitations. Do not include YHL-machine absolute paths in colleague commands.

- [ ] **Step 4: Validate guide commands and forbidden claims**

Confirm every referenced payload file exists, every shell continuation is syntactically complete, the guide contains no active `playback_slowdown` argument, and all model/motion locations use the five declared variables.

### Task 3: Validate assets and build reproducible archives

**Files:**
- Create: `hitter_new_racket_assets_v1_20260812/MANIFEST.txt`
- Create: `hitter_new_racket_assets_v1_20260812/SHA256SUMS`
- Create: `hitter_new_racket_assets_v1_20260812.tar.gz`
- Create: `hitter_new_racket_assets_v1_20260812.zip`

**Interfaces:**
- Consumes: Complete staging directory from Tasks 1-2.
- Produces: Two transport archives with identical logical contents and independently verifiable checksums.

- [ ] **Step 1: Validate and compile the MuJoCo XML overlay**

Assert that the staged XML hash matches the installed source XML, then run MuJoCo 3.11 `MjModel.from_xml_path()` against the installed XML in the complete RobotBridge asset directory. Assert the existence of body `right_racket_link` and geoms `right_connector_visual`, `right_racket_holder_visual`, `right_racket_visual`, and `right_racket_face_collision`. The compact overlay intentionally does not duplicate legacy G1 mesh files.

- [ ] **Step 2: Validate Isaac references and configuration source**

Assert that `main_motion_compatible.usda` references `../g1_hitter_racket/main.usda` and `./cad_geometry.usd`, and compile the current `robots/g1.py` with `py_compile` as the reference implementation.

- [ ] **Step 3: Validate RobotBridge Hydra selection**

Run `run.py --config-name=hitter sim=mujoco robot.asset.asset_file=g1_29dof_hitter_racket_table_tennis_cad_v1.xml --cfg job` with an existing local ONNX path, and assert that the composed config selects the staged filename while `viewer` and `real_time` remain enabled.

- [ ] **Step 4: Generate manifest and staged-file checksums**

Generate a sorted `MANIFEST.txt`, then generate `SHA256SUMS` for every staged file except `SHA256SUMS` itself. Run `sha256sum -c SHA256SUMS`; expected: all entries report `OK`.

- [ ] **Step 5: Build both archives**

From `/home/yhl/Desktop/Hitter`, create the `.tar.gz` and `.zip` using the single staging directory as their root entry.

- [ ] **Step 6: Extract archives into fresh temporary directories and compare**

Use `mktemp -d` for each extraction, compare sorted file lists and SHA-256 values against the staging directory, and confirm neither archive contains excluded extensions or directories.

- [ ] **Step 7: Record final sizes and delivery paths**

Run `ls -lh` and `sha256sum` on both archives and report their absolute paths, sizes, hashes, verification results, and the unchanged-default behavior.
