# HITTER model56600 ONNX Deployment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Convert `/home/loco1/Downloads/model_56600.pt` into a metadata-complete RobotBridge4 ONNX artifact and switch the HITTER production model pointer to it.

**Architecture:** Reuse the exact model17500 MOSAIC source/config/motion snapshot and project exporter so the checkpoint normalizer and deployment metadata are preserved. Keep the old ONNX untouched, deploy the new artifact under a unique filename, then change only the configured checkpoint pointer.

**Tech Stack:** PyTorch, Isaac Lab, RSL-RL, ONNX opset 18, ONNX Runtime, Hydra YAML.

## Global Constraints

- Work in `/home/loco1/BOB/Hitter/RobotBridge4` and preserve all unrelated dirty-worktree changes.
- Do not overwrite `hitter.onnx` or the current model17500 artifact.
- Do not start, stop, or restart Vicon, `trans`, or the real-world policy process.
- Do not run the project test suite; only run mandatory artifact-format and load checks.
- The deployed contract must remain float32 `obs[1,104] -> actions[1,29]` with all 11 existing deployment metadata fields.
- The target filename is `hitter_model56600_8x4090_poswin001_velstd18_velwin003_104_20260814.onnx`.

---

### Task 1: Export the model56600 ONNX artifact

**Files:**
- Read: `/home/loco1/Downloads/model_56600.pt`
- Read: `/home/loco1/BOB/Hitter/MOSAIC-main/imports/8x4090_20260808_model17500/`
- Create: `/home/loco1/BOB/Hitter/MOSAIC-main/logs/rsl_rl/hitter_striking_g1/8x4090_20260814_model56600_export/model_56600.pt`
- Create: `/home/loco1/BOB/Hitter/MOSAIC-main/logs/rsl_rl/hitter_striking_g1/8x4090_20260814_model56600_export/params/agent.yaml`
- Create: `/home/loco1/BOB/Hitter/MOSAIC-main/logs/rsl_rl/hitter_striking_g1/8x4090_20260814_model56600_export/exported/policy.onnx`

**Interfaces:**
- Consumes: checkpoint actor `104 -> 512 -> 256 -> 128 -> 29` and policy normalizer `[1,104]`.
- Produces: opset 18 ONNX with input `obs`, output `actions`, and attached RobotBridge metadata.

- [ ] **Step 1: Validate immutable source identities and target non-existence**

Run SHA256 on the checkpoint, inspect the source import snapshot, and abort if the target export directory already contains a model artifact with a different hash.

- [ ] **Step 2: Prepare the exporter run directory**

Copy `model_56600.pt` and the exact model17500 `params/agent.yaml` into the unique run directory, preserving the source import snapshot read-only.

- [ ] **Step 3: Run the project exporter**

Run `scripts/rsl_rl/play.py` with task `Hitter-Striking-PlannerDomain-Flat-G1-v0`, the imported six-motion dataset, `--num_envs=1`, `--skip_critic`, `--headless`, `--device=cuda:0`, `--video`, and `--video_length=1`, using the isolated whole-body-tracking source path.

- [ ] **Step 4: Rename the exported artifact**

Copy `exported/policy.onnx` to the approved unique model56600 filename inside the import export directory without deleting the original exporter output.

### Task 2: Check, deploy, and select the artifact

**Files:**
- Create: `deploy/data/model/hitter/hitter_model56600_8x4090_poswin001_velstd18_velwin003_104_20260814.onnx`
- Create: `deploy/data/model/hitter/hitter_model56600_8x4090_poswin001_velstd18_velwin003_104_20260814.provenance.txt`
- Modify: `deploy/config/mimic/hitter.yaml:2`

**Interfaces:**
- Consumes: Task 1 ONNX artifact.
- Produces: RobotBridge4 deployment artifact and production model pointer.

- [ ] **Step 1: Run mandatory artifact checks**

Use `onnx.checker` and ONNX Runtime to verify opset 18, float32 input `[1,104]`, float32 output `[1,29]`, the exact 11 metadata keys, and finite inference for one zero observation. Do not run any project test module.

- [ ] **Step 2: Copy the verified ONNX into RobotBridge4**

Confirm the destination does not exist, then copy the binary artifact and verify source/destination SHA256 equality.

- [ ] **Step 3: Record provenance**

Create the companion text file with checkpoint and ONNX hashes, iteration, exporter snapshot, contract, metadata count, validation commands, and the explicit boundary that no MuJoCo or real-robot behavior test was run.

- [ ] **Step 4: Switch only the production checkpoint pointer**

Use a targeted patch to change `policy.checkpoint` in `deploy/config/mimic/hitter.yaml` from model17500 to the unique model56600 filename. Preserve every other line exactly as found.

- [ ] **Step 5: Verify the final selection**

Resolve the configured path, load that exact file with ONNX Runtime, print its hash and contract, inspect the targeted YAML diff, and confirm the running real-world process PID is unchanged.
