# Vicon Ball Recorder Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a standalone Python command that records Vicon ball X/Y/Z and frame-to-frame Vx from startup until Ctrl+C.

**Architecture:** Subscribe directly to the existing `vicon_state_data` LCM channel and write only `name=ball` messages to a timestamped CSV. Keep velocity calculation in a small stateful helper so frame gaps, invalid samples, and non-increasing timestamps are independently testable.

**Tech Stack:** Python 3, `lcm`, generated `transformation_t`, standard-library `csv`, `argparse`, and `unittest`.

## Global Constraints

- Do not modify `monitor_vicon_lcm.py` or any real-world policy behavior.
- Default recording continues until Ctrl+C.
- Calculate Vx only for consecutive Vicon source frames with increasing source time.
- Leave Vx empty for the first frame and after any tracking gap.
- Close and flush the CSV on Ctrl+C.

---

### Task 1: Velocity tracking behavior

**Files:**
- Create: `deploy/tests/test_record_vicon_ball.py`
- Create: `deploy/mocap_bridge/record_vicon_ball.py`

**Interfaces:**
- Produces: `BallVelocityTracker.update(frame_number, source_time_s, x_m, valid=True) -> float | None`

- [ ] Write tests showing that consecutive frames produce `(x1-x0)/(t1-t0)`, while the first frame, a frame gap, an invalid frame, and a non-increasing timestamp produce `None`.
- [ ] Run `PYTHONPATH=deploy conda run --no-capture-output -n rb python -m unittest -v tests.test_record_vicon_ball` and verify it fails because the module does not exist.
- [ ] Implement the minimal tracker and rerun the test to green.

### Task 2: Standalone CSV recorder

**Files:**
- Modify: `deploy/mocap_bridge/record_vicon_ball.py`
- Modify: `deploy/tests/test_record_vicon_ball.py`

**Interfaces:**
- Consumes: `vicon_state_data` messages decoded as `transformation_t`.
- Produces: CSV columns `row_index,host_time_s,elapsed_s,frame_number,vicon_time_s,x_m,y_m,z_m,vx_mps,valid,occluded`.

- [ ] Add tests for timestamped default output naming and newline-safe CSV rows.
- [ ] Implement CLI options `--output`, `--lcm-url`, `--channel`, `--ball-name`, and optional `--duration`.
- [ ] Catch Ctrl+C, close the CSV, and print the recorded row count, duration, gap count, and output path.
- [ ] Run the focused unittest, then perform a short live `--duration 2` recording.
- [ ] Run `git diff --check` and confirm no whitespace errors.
