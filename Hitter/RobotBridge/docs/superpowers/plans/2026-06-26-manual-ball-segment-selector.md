# Manual Ball Segment Selector Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an offline matplotlib tool for manually selecting clean ball trajectory segments from historical CSV logs.

**Architecture:** Keep data loading/export logic testable and independent from the GUI. The script owns `RecordedBallRow`, `load_recorded_ball_rows`, `split_candidates`, and `export_selected_segments`; the matplotlib UI only calls these helpers and records selected intervals.

**Tech Stack:** Python 3, csv, dataclasses, numpy, matplotlib, unittest.

---

### Task 1: Test Export Core

**Files:**
- Create: `tests/test_select_ball_segments.py`
- Create: `deploy/mocap_bridge/select_ball_segments.py`

- [ ] **Step 1: Write the failing test**

Create `tests/test_select_ball_segments.py` with tests that build a small CSV, load ball rows, export two selected time ranges, and assert the output preserves original columns plus `clean_segment_id`, `source_file`, and `source_row_index`.

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
conda run --no-capture-output -n rb python -m unittest tests.test_select_ball_segments
```

Expected: fail because `mocap_bridge.select_ball_segments` does not exist.

- [ ] **Step 3: Implement minimal export helpers**

Create `deploy/mocap_bridge/select_ball_segments.py` with:

- `RecordedBallRow`
- `SelectedSegment`
- `load_recorded_ball_rows`
- `split_candidates`
- `export_selected_segments`

- [ ] **Step 4: Run test to verify it passes**

Run:

```bash
conda run --no-capture-output -n rb python -m unittest tests.test_select_ball_segments
```

Expected: pass.

### Task 2: Add Matplotlib Selector

**Files:**
- Modify: `deploy/mocap_bridge/select_ball_segments.py`

- [ ] **Step 1: Add interactive plotting shell**

Implement `SegmentSelectorApp` with four axes: 3D, top view, side view, and height over time. Clicks on height-over-time collect start/end times.

- [ ] **Step 2: Add CLI**

Add arguments:

```text
csv...
--output
--sample-rate-hz
--time-source
--max-gap-s
--min-candidate-samples
--table-height
--table-length
--table-width
--table-center-x
--table-center-y
```

- [ ] **Step 3: Verify syntax and tests**

Run:

```bash
conda run --no-capture-output -n rb python -m compileall deploy/mocap_bridge/select_ball_segments.py
conda run --no-capture-output -n rb python -m unittest tests.test_select_ball_segments tests.test_visualize_ball_trajectory tests.test_fit_ball_dynamics
```

Expected: all pass.
