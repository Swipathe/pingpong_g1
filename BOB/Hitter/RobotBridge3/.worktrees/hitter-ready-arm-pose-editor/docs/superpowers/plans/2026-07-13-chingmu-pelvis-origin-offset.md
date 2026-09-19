# ChingMu Pelvis Origin Offset Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish the training URDF pelvis origin instead of the ChingMu rigid-body origin.

**Architecture:** Store the calibrated heading-frame translation in `BridgeConfig`. After deriving the existing relative-yaw rotation, rotate and add the translation before constructing the `G1Pelvis` LCM message. Leave every other object and interface untouched.

**Tech Stack:** Python 3, NumPy, SciPy `Rotation`, standard-library `unittest`, LCM.

## Global Constraints

- Use offset `[0.003145, 0.044074, 0.048231]` metres.
- Interpret the offset in startup heading axes: forward, left, up.
- Preserve ball, table, LCM, planner, and policy behavior.
- Preserve unrelated user changes in the dirty worktree; do not commit them.

---

### Task 1: Apply the calibrated pelvis offset

**Files:**
- Modify: `deploy/mocap_bridge/chingmu_table_lcm_bridge.py`
- Test: `deploy/mocap_bridge/tests/test_chingmu_table_lcm_bridge.py`

**Interfaces:**
- Consumes: the existing relative-yaw rotation computed in `ChingMuTableLcmBridge.process_frame`.
- Produces: `BridgeConfig.pelvis_offset_heading_m: tuple[float, float, float]` and corrected `G1Pelvis.pos_vicon`.

- [ ] **Step 1: Write a failing zero-yaw test**

Add a test that processes the first valid body frame and expects
`raw_to_table_world(body_position) + pelvis_offset_heading_m`.

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
conda run --no-capture-output -n rb python -m unittest \
  deploy.mocap_bridge.tests.test_chingmu_table_lcm_bridge.ChingMuTableBridgeTest.test_applies_pelvis_offset_in_initial_heading
```

Expected: failure because the bridge still publishes the uncorrected rigid
origin.

- [ ] **Step 3: Write a failing yaw-rotation test**

Initialize the bridge with a zero-yaw frame, then process a frame whose rigid
yaw is 90 degrees. Expect the XY offset to rotate by 90 degrees and Z to stay
unchanged.

- [ ] **Step 4: Run the focused yaw test and verify RED**

Run the new test directly with `python -m unittest`. Expected: failure because
the offset is not implemented.

- [ ] **Step 5: Implement the minimal correction**

Add the calibrated tuple to `BridgeConfig`. Reuse the existing relative-yaw
matrix and publish:

```python
base_world + heading_rotation @ np.asarray(
    self.config.pelvis_offset_heading_m, dtype=np.float64
)
```

- [ ] **Step 6: Run both focused tests and verify GREEN**

Run both test methods directly. Expected: two tests pass.

- [ ] **Step 7: Run the full bridge regression suite**

Run:

```bash
conda run --no-capture-output -n rb python -m unittest discover \
  -s deploy/mocap_bridge/tests -p 'test_*.py' -v
```

Expected: all tests pass with zero failures and zero errors.

- [ ] **Step 8: Review the scoped diff**

Run `git diff --` for the two implementation files and confirm that only the
pelvis-position correction and its tests changed. Do not commit the existing
dirty worktree.

### Task 2: Restart and observe the live bridge

**Files:**
- No source changes.

**Interfaces:**
- Consumes: the corrected ChingMu bridge executable path and existing table calibration.
- Produces: a live `vicon_state_data/G1Pelvis` stream using the corrected origin.

- [ ] **Step 1: Stop duplicate ChingMu bridge processes only**

Match `deploy/mocap_bridge/chingmu_table_lcm_bridge.py` exactly. Do not stop
Unitree transport or the policy process.

- [ ] **Step 2: Start one corrected bridge**

Run with host `192.168.2.100`, base subject `G1Pelvis`, the existing
`chingmu_table_frame_latest.json`, and `--publish`.

- [ ] **Step 3: Verify the corrected stream before robot motion**

Observe `G1Pelvis` LCM samples and confirm finite, valid, changing frame data.
Report the corrected position and leave physical motion to the explicit
real-robot launch state.
