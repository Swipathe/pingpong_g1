# Vicon Handler Timing Diagnostics Design

## Goal

Add diagnostic timing to `deploy/simulator/real_world.py` so each handled Vicon message can be correlated with its source frame gap and the time spent in each handler stage. The diagnostics must not change ball estimation, snapshot publication, locking, or control semantics.

## Scope

Instrument the existing `G1Pelvis` and `ball` branches of `_vicon_state_handler()`. Do not change the C++ bridge, LCM message schema, subscription queue, estimator algorithm, planner behavior, or control loop.

## Clock and units

Use `time.perf_counter_ns()` for all timing boundaries. Convert durations to milliseconds only when formatting the final log line. This monotonic high-resolution clock is independent of wall-clock changes.

## Frame-gap tracking

Track the most recently handled source frame independently for `ball` and `G1Pelvis`.

For each message:

- `frame` is `msg.vicon_frame_number`.
- `delta` is `frame - previous_frame` when both frame numbers are positive; the first observation reports `delta=NA`.
- `dropped` is `max(delta - 1, 0)` for a positive delta; the first observation reports `dropped=NA`.
- A duplicate or backwards frame reports its actual non-positive delta and `dropped=0`.

The metric is named `dropped` for operational convenience, but it represents missing source-frame numbers observed by this subscriber; it does not by itself identify where those frames were lost.

## Ball timing stages

Emit one `[VICON_TIMING] name=ball` line per handled ball message containing:

- `frame`, `delta`, and `dropped`;
- `decode_ms`: LCM payload decode time;
- `parse_ms`: name, position, quaternion, source-time, and frame extraction;
- `prelock_ms`: validity and pre-lock preparation inside `_update_ball_state_from_vicon()`;
- `lock_wait_ms`: time waiting to acquire `_hitter_ball_state_lock`;
- `timestamp_ms`: estimator timestamp selection;
- `estimator_ms`: `BallStateEstimator.add_sample()` time, or zero for an invalid message;
- `state_update_ms`: copying estimate values and updating temporary state;
- `snapshot_ms`: building the immutable snapshot and copying the listener list;
- `locked_ms`: total time holding `_hitter_ball_state_lock`;
- `listener_ms`: listener notification after releasing the state lock;
- `update_total_ms`: complete `_update_ball_state_from_vicon()` time;
- `total_ms`: complete `_vicon_state_handler()` time from before decode through notification.

The existing per-ball `[VICON]` data log remains unchanged so timing results can be compared with the current diagnostic output.

## Pelvis timing stages

Emit one `[VICON_TIMING] name=g1pelvis` line per handled pelvis message containing:

- `frame`, `delta`, and `dropped`;
- `decode_ms` and `parse_ms`;
- `lock_wait_ms`;
- `state_update_ms` for validation and temporary-state assignment;
- `locked_ms`;
- `total_ms`.

Logging of the first valid pelvis pose remains outside the state lock.

## Implementation structure

Keep timing data local to the handler call. `_update_ball_state_from_vicon()` will return a small timing record to `_vicon_state_handler()`; it will continue returning only after the same estimator update, snapshot creation, listener notification, and informational logging it performs today.

Use a small formatting helper for durations and frame-gap fields so ball and pelvis output remain consistent. Initialize the two previous-frame counters with the other Vicon/ball state.

## Locking constraints

Instrumentation must preserve the current critical-section boundaries:

- estimator mutation, temporary ball state, pelvis state, and snapshot creation remain protected by `_hitter_ball_state_lock`;
- listener callbacks remain outside `_hitter_ball_state_lock`;
- no logging occurs while `_hitter_ball_state_lock` is held;
- no new lock is introduced.

## Testing

Add focused tests that fail before implementation and prove:

1. frame-gap calculation for first, consecutive, skipped, duplicate, and backwards frames;
2. ball timing output contains every required stage and a correct missing-frame count;
3. pelvis timing output contains every required stage and an independent frame history;
4. listener notification remains outside `_hitter_ball_state_lock`;
5. existing real-world snapshot and estimator tests continue to pass.

## Expected operational impact

Per-message console output at approximately 300 ball messages and 300 pelvis messages per second is intentionally diagnostic and may itself increase handler latency or frame loss. The timing is suitable for a short profiling run, not as a permanent low-overhead production telemetry mode. Results should be collected with unbuffered output, such as `conda run --no-capture-output`.
