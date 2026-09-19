# ChingMu Realtime Planner and HITTER Policy Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the marker-centroid base approximation with the real ChingMu `G1Pelvis` root pose and connect 360 Hz ball estimates to a latest-only planner whose results drive the existing 50 Hz HITTER policy through the approved TTS and swing lifecycle.

**Architecture:** The ChingMu bridge publishes the real rigid-body root and explicit ball lifecycle messages on the existing LCM schema. `RealWorld` converts ball samples into immutable estimate snapshots and submits them to a capacity-one planner worker; a pure command lifecycle consumes completed results, while `HitterEnv` only maps the selected immutable command into the existing 105-value observation. MuJoCo keeps synchronous state acquisition but shares the same planner boundaries and command lifecycle.

**Tech Stack:** Python 3, standard-library `threading` and `unittest`, NumPy, SciPy Rotation, MuJoCo, ONNX Runtime, Hydra/OmegaConf, LCM, ChingMu `libCMVrpn.so`.

## Global Constraints

- Preserve the ONNX model, 105-value observation order, 29-value action order, and 50 Hz policy/action frequency.
- ChingMu source rate and real-world estimator submission rate are `360.0 Hz`.
- Use a capacity-one latest-only planner request; overwrite pending old frames and never build a queue.
- Waiting observation remains always-on and is not action-gated.
- Waiting `time_to_strike` is the user-selected `1.00 s`; do not clamp it to the trained `0.92 s` ceiling.
- Arm the first valid incoming result in `0.80 s <= TTS <= 0.90 s`; skip an unarmed track once `TTS < 0.80 s`.
- After arm, accept atomic same-track overrides only when planned TTS is in `[0.0, 0.92] s`.
- Preserve the arm-time recovery duration when an override changes the strike deadline.
- Planner admission requires `x > 0`, `vx < 0`, a positive-to-negative crossing of `x=0`, and `table_height < hit_z <= table_height + 0.50 m`.
- Estimator admission requires the ball inside the accepted table x/y bounds and above the tabletop; outgoing `vx >= 0` remains observable but cannot arm the planner.
- Do not add a `0.20 s` or any other wall-clock stale-data timeout.
- Crossing `x=0` or an explicit current-frame ball loss clears the estimator/planner track but does not truncate an already armed recovery.
- Use accepted effective geometry: length `2.730738 m`, width `1.512451 m`, center `[1.365369, 0.0] m`, tabletop `z=0.760000 m`, hit plane `x=0`.
- Never start `deploy/run.py --config-name=hitter` against real hardware during automated verification.
- The worktree already contains user-owned modified and untracked files. Stage only the exact files named by each task; never use a broad `git add`.

## File Map

- `deploy/mocap_bridge/chingmu_sdk_client.py`: preserve the official SDK rigid-body root report in each frame.
- `deploy/mocap_bridge/chingmu_table_lcm_bridge.py`: table transform, true base pose, relative yaw, ball tracking, and explicit invalid messages.
- `deploy/mocap_bridge/calibrations/chingmu_table_frame_latest.json`: accepted effective dimensions with the current measured raw corners.
- `deploy/utils/hitter_planner.py`: incoming crossing and hit-height validation.
- `deploy/utils/hitter_realtime.py`: immutable snapshots, latest-only worker, and pure command lifecycle.
- `deploy/simulator/real_world.py`: estimator snapshots, track epochs, direction resets, and listener publication.
- `deploy/envs/hitter.py`: worker integration and mapping the lifecycle's active command into policy observations.
- `deploy/agents/hitter_agent.py`: refresh real-world observation immediately before ONNX inference.
- `deploy/config/hitter.yaml`: pass shared HITTER table-tennis values to the selected simulator.
- `deploy/config/mimic/hitter.yaml`: final timing, geometry, estimator, and planner boundary values.
- `deploy/data/assets/g1/g1_29dof_hitter_racket_table_tennis.xml`: accepted table geometry for MuJoCo.
- `deploy/mocap_bridge/tests/*.py`, `deploy/tests/*.py`: standard-library regression tests.

---

### Task 1: Preserve the ChingMu `G1Pelvis` Root Report

**Files:**
- Modify: `deploy/mocap_bridge/chingmu_sdk_client.py`
- Modify: `deploy/mocap_bridge/tests/test_chingmu_sdk_client.py`

**Interfaces:**
- Produces: `MocapFrame.body_position_mm: Optional[np.ndarray]`
- Produces: `MocapFrame.body_quaternion_xyzw: Optional[np.ndarray]`
- Preserves: `MocapFrame.body_markers_mm` for diagnostics only

- [ ] **Step 1: Write failing root-report tests**

Extend `FakeLibrary.emit_tracker()` with a `quaternion` argument and add:

```python
def test_resolved_body_root_report_is_preserved_as_xyzw(self):
    client = self.make_client()
    self.start_client(client, body_id=6)
    self.library.emit_tracker(6, 100, (410.0, -20.0, 793.0), quaternion=(0.0, 0.0, 0.2, 0.98))
    self.library.emit_tracker(8100, 101, (1.0, 2.0, 3.0))

    frame = client.next_frame(timeout_s=0.01)

    np.testing.assert_allclose(frame.body_position_mm, [410.0, -20.0, 793.0])
    np.testing.assert_allclose(frame.body_quaternion_xyzw, [0.0, 0.0, 0.2, 0.98])
    self.assertEqual(frame.unlabeled_markers_mm.shape, (0, 3))
    client.close()

def test_other_hierarchy_root_is_not_used_as_g1pelvis(self):
    client = self.make_client()
    self.start_client(client, body_id=6)
    self.library.emit_hierarchy(7, "OtherBody")
    self.library.emit_tracker(7, 200, (9.0, 9.0, 9.0))
    self.library.emit_tracker(8100, 201, (1.0, 2.0, 3.0))

    frame = client.next_frame(timeout_s=0.01)

    self.assertIsNone(frame.body_position_mm)
    self.assertIsNone(frame.body_quaternion_xyzw)
    self.assertEqual(frame.unlabeled_markers_mm.shape, (0, 3))
    client.close()
```

- [ ] **Step 2: Run the tests and verify the missing fields fail**

Run:

```bash
conda run -n rb python -m unittest \
  deploy.mocap_bridge.tests.test_chingmu_sdk_client.ChingMuSdkClientTest.test_resolved_body_root_report_is_preserved_as_xyzw \
  deploy.mocap_bridge.tests.test_chingmu_sdk_client.ChingMuSdkClientTest.test_other_hierarchy_root_is_not_used_as_g1pelvis -v
```

Expected: errors showing `MocapFrame` has no `body_position_mm` / `body_quaternion_xyzw`.

- [ ] **Step 3: Add root fields and capture only `sensor == body_id`**

Implement these exact frame fields and per-frame buffers:

```python
@dataclass(frozen=True)
class MocapFrame:
    frame_number: int
    source_time_s: float
    body_position_mm: Optional[np.ndarray]
    body_quaternion_xyzw: Optional[np.ndarray]
    body_markers_mm: Dict[int, np.ndarray]
    unlabeled_markers_mm: np.ndarray
```

In `_on_tracker()`, classify reports in this order:

```python
if self.body_id is not None and sensor == self.body_id:
    self._current_body_position = position.copy()
    self._current_body_quaternion = np.asarray(tuple(report.quat), dtype=np.float64)
elif sensor in self.body_marker_sensor_range:
    self._current_body_markers[sensor] = position
elif sensor not in self._hierarchy_sensor_ids and not self._is_labelled_rigid_body_marker_locked(sensor):
    self._current_unlabeled.append(position)
```

Reset both root buffers in `_begin_frame_locked()` and copy them into `MocapFrame` in `_finish_frame_locked()`.

- [ ] **Step 4: Run the complete SDK client test module**

Run: `conda run -n rb python -m unittest deploy.mocap_bridge.tests.test_chingmu_sdk_client -v`

Expected: all SDK client tests pass, including the existing latest-frame drop test.

- [ ] **Step 5: Commit only Task 1 files**

```bash
git add -- deploy/mocap_bridge/chingmu_sdk_client.py deploy/mocap_bridge/tests/test_chingmu_sdk_client.py
git commit -m "feat: preserve ChingMu rigid body root pose"
```

---

### Task 2: Publish the True Base Pose and Explicit Ball-End Events

**Files:**
- Modify: `deploy/mocap_bridge/chingmu_table_lcm_bridge.py`
- Modify: `deploy/mocap_bridge/calibrations/chingmu_table_frame_latest.json`
- Modify: `deploy/mocap_bridge/tests/test_chingmu_table_lcm_bridge.py`

**Interfaces:**
- Consumes: `MocapFrame.body_position_mm`, `MocapFrame.body_quaternion_xyzw`
- Produces: `body_pose_to_table_world(...) -> Optional[tuple[np.ndarray, np.ndarray]]`
- Produces: `BallTrackUpdate(position_world: Optional[np.ndarray], ended: bool)`
- Preserves: existing `transformation_t` channel, names, metres, and quaternion `xyzw`

- [ ] **Step 1: Replace marker-fit expectations with root-pose and ball-end tests**

Add focused tests:

```python
def test_first_root_yaw_is_identity_and_later_yaw_is_relative(self):
    first = self.make_frame(360, TABLE_CORNERS_MM, body_position=ROBOT_RAW_MM,
                            body_quat=Rotation.from_euler("z", 0.40).as_quat())
    second = self.make_frame(361, TABLE_CORNERS_MM, body_position=ROBOT_RAW_MM,
                             body_quat=Rotation.from_euler("z", 0.55).as_quat())
    first_base = next(message for message in self.bridge.process_frame(first) if message.name == "G1Pelvis")
    second_base = next(message for message in self.bridge.process_frame(second) if message.name == "G1Pelvis")
    np.testing.assert_allclose(first_base.quat_vicon, [0.0, 0.0, 0.0, 1.0], atol=1e-8)
    np.testing.assert_allclose(
        second_base.quat_vicon,
        Rotation.from_euler("z", 0.15).as_quat(),
        atol=1e-6,
    )

def test_missing_root_has_no_marker_centroid_fallback(self):
    frame = self.make_frame(362, TABLE_CORNERS_MM, body_position=None, body_quat=None)
    names = [message.name for message in self.bridge.process_frame(frame)]
    self.assertNotIn("G1Pelvis", names)
    self.assertIn("table", names)

def test_crossing_x_zero_publishes_one_invalid_and_closes_track(self):
    inside_raw = table_world_to_raw([0.01, 0.0, 0.90], self.table, self.config)
    crossed_raw = table_world_to_raw([-0.01, 0.0, 0.90], self.table, self.config)
    self.bridge.process_frame(self.make_frame(400, [inside_raw]))
    ended = self.bridge.process_frame(self.make_frame(401, [crossed_raw]))
    next_frame = self.bridge.process_frame(self.make_frame(402, []))
    invalid = [message for message in ended if message.name == "ball"]
    self.assertEqual(len(invalid), 1)
    self.assertEqual((invalid[0].valid, invalid[0].occluded), (0, 1))
    self.assertFalse(any(message.name == "ball" for message in next_frame))
```

Update every `MocapFrame(...)` test factory to pass the two new root fields.

- [ ] **Step 2: Run the bridge tests and verify old marker-fit behavior fails**

Run: `conda run -n rb python -m unittest deploy.mocap_bridge.tests.test_chingmu_table_lcm_bridge -v`

Expected: root-pose, effective-dimension, and explicit-invalid tests fail against the current implementation.

- [ ] **Step 3: Implement table rotation and true body pose conversion**

Use the accepted dimensions as `BridgeConfig` and CLI defaults. Form the raw-to-table rotation with table axes as rows:

```python
def raw_rotation_to_table_world(table: TableFrame) -> np.ndarray:
    return np.vstack([table.x_axis_raw, table.y_axis_raw, table.z_axis_raw])

def normalize_quaternion_xyzw(value) -> Optional[np.ndarray]:
    quat = np.asarray(value, dtype=np.float64).reshape(4)
    if not np.isfinite(quat).all() or np.linalg.norm(quat) < 1.0e-9:
        return None
    quat = quat / np.linalg.norm(quat)
    return -quat if quat[3] < 0.0 else quat

def body_pose_to_table_world(position_mm, quaternion_xyzw, table, config):
    quat = normalize_quaternion_xyzw(quaternion_xyzw)
    position = np.asarray(position_mm, dtype=np.float64).reshape(3)
    if quat is None or not np.isfinite(position).all():
        return None
    rotation_wb = raw_rotation_to_table_world(table) @ Rotation.from_quat(quat).as_matrix()
    return raw_to_table_world(position, table, config), rotation_wb
```

`ChingMuTableLcmBridge` records the first valid yaw and publishes only relative yaw. Remove normal-operation dependencies on `fit_rigid_pose()`, reference marker centroids, and `capture_reference_body_markers()`.

- [ ] **Step 4: Fit the table plane in 3D and update calibration metadata**

Use SVD over the four selected corners to obtain the normal, project the long-axis estimate onto the plane, choose its sign from the true root position, and construct a right-handed basis. Recompute the saved center and `x/y/z` axes from the saved raw corners with this same function, then update `chingmu_table_frame_latest.json` dimensions to `2.730738`, `1.512451`, and `0.760000`; do not merely change the dimension metadata while leaving old fixed-z axes.

- [ ] **Step 5: Implement one-shot ball-end updates**

Add:

```python
@dataclass(frozen=True)
class BallTrackUpdate:
    position_world: Optional[np.ndarray]
    ended: bool
```

`BallTracker.update()` must associate the closest continuity candidate before applying the `x>0` admission bound. If an active track selects `x<=0`, or an active track has no current candidate, return `ended=True`, call `reset()`, and make `process_frame()` emit exactly one invalid ball message. Do not inspect wall-clock elapsed time.

- [ ] **Step 6: Use SDK source timestamps in LCM messages**

Change `make_message()` to receive `source_time_s` and set `message.vicon_time_s = float(source_time_s)`. Pass `frame.source_time_s` for base, ball, invalid ball, and table messages.

- [ ] **Step 7: Run all mocap tests**

Run:

```bash
conda run -n rb python -m unittest \
  deploy.mocap_bridge.tests.test_chingmu_sdk_client \
  deploy.mocap_bridge.tests.test_chingmu_table_lcm_bridge -v
```

Expected: all tests pass; no test expects marker-centroid fallback or silent active-ball loss.

- [ ] **Step 8: Commit only Task 2 files**

```bash
git add -- deploy/mocap_bridge/chingmu_table_lcm_bridge.py \
  deploy/mocap_bridge/calibrations/chingmu_table_frame_latest.json \
  deploy/mocap_bridge/tests/test_chingmu_table_lcm_bridge.py
git commit -m "feat: publish table-frame ChingMu root and ball lifecycle"
```

---

### Task 3: Enforce Incoming and Hit-Height Planner Boundaries

**Files:**
- Modify: `deploy/utils/hitter_planner.py`
- Create: `deploy/tests/__init__.py`
- Create: `deploy/tests/test_hitter_planner_boundaries.py`

**Interfaces:**
- Extends: `StrikePlanner(..., minimum_hit_height: float, maximum_hit_height: float, maximum_prediction_horizon_s: float)`
- Preserves: `HitterSystemPlanner.plan_command(...) -> HitterWbcCommand`

- [ ] **Step 1: Write failing boundary tests**

```python
class StrikePlannerBoundaryTest(unittest.TestCase):
    def setUp(self):
        predictor = BallTrajectoryPredictor(table_height=0.76, dt=0.005)
        self.planner = StrikePlanner(
            predictor=predictor,
            virtual_hit_plane_x=0.0,
            prediction_horizon_s=2.0,
            maximum_prediction_horizon_s=5.0,
            minimum_hit_height=0.76,
            maximum_hit_height=1.26,
            require_future_hit_plane_crossing=True,
        )

    def test_rejects_outgoing_ball(self):
        with self.assertRaisesRegex(ValueError, "vx < 0"):
            self.planner.plan([0.4, 0.0, 0.9], [1.0, 0.0, 0.0])

    def test_rejects_ball_already_past_hit_plane(self):
        with self.assertRaisesRegex(ValueError, "x > 0"):
            self.planner.plan([-0.01, 0.0, 0.9], [-1.0, 0.0, 0.0])

    def test_rejects_crossing_above_fifty_centimetres(self):
        with self.assertRaisesRegex(ValueError, "hit height"):
            self.planner.plan([0.4, 0.0, 1.40], [-1.0, 0.0, 0.0])

    def test_accepts_positive_to_negative_crossing(self):
        plan = self.planner.plan([0.4, 0.0, 0.95], [-1.5, 0.0, 0.0])
        self.assertGreater(plan.t_strike, 0.0)
        self.assertAlmostEqual(plan.p_racket_target[0], 0.0)
        self.assertLess(plan.v_ball_in[0], 0.0)

    def test_two_seconds_is_a_chunk_not_a_business_gate(self):
        plan = self.planner.plan([2.4, 0.0, 0.95], [-0.8, 0.0, 0.0])
        self.assertGreater(plan.t_strike, 2.0)
        self.assertLessEqual(plan.t_strike, 5.0)
```

- [ ] **Step 2: Run tests and verify reverse crossings currently pass incorrectly**

Run: `PYTHONPATH=deploy:. conda run -n rb python -m unittest deploy.tests.test_hitter_planner_boundaries -v`

Expected: at least outgoing, past-plane, and height tests fail.

- [ ] **Step 3: Add explicit admission and directed-crossing checks**

At the start of `hit_plane_intersection()` validate finite vectors, `position[0] > plane`, and `velocity[0] < 0`. Evaluate the predictor first to `prediction_horizon_s`, then increase the horizon in chunks up to `maximum_prediction_horizon_s`; stop at the first directed crossing. This makes `2.0 s` a numerical chunk rather than a business admission gate. Select crossings only where:

```python
crossing = np.where(
    (signed[:-1] > 0.0)
    & (signed[1:] <= 0.0)
    & (trajectory.velocities[:-1, 0] < 0.0)
)[0]
```

After interpolation, reject unless:

```python
if not (self.minimum_hit_height < pos[2] <= self.maximum_hit_height):
    raise ValueError(
        f"predicted hit height {pos[2]:.3f} is outside "
        f"({self.minimum_hit_height:.3f}, {self.maximum_hit_height:.3f}]"
    )
```

Retain a finite numerical predictor horizon as a safety bound, but remove the old nearest-point fallback when a directed crossing is required.

- [ ] **Step 4: Run boundary tests and a planner regression smoke test**

Run:

```bash
PYTHONPATH=deploy:. conda run -n rb python -m unittest deploy.tests.test_hitter_planner_boundaries -v
PYTHONPATH=deploy conda run -n rb python - <<'PY'
from utils.hitter_planner import HitterSystemPlanner
command = HitterSystemPlanner().plan_command([0.4, 0.0, 0.95], [-1.5, 0.0, 0.0])
print(command.strike_type, command.time_to_strike)
PY
```

Expected: tests pass and smoke output has a positive TTS.

- [ ] **Step 5: Commit Task 3**

```bash
git add -- deploy/utils/hitter_planner.py deploy/tests/__init__.py deploy/tests/test_hitter_planner_boundaries.py
git commit -m "feat: enforce HITTER incoming strike boundaries"
```

---

### Task 4: Add Immutable Snapshots, Latest-Only Worker, and Pure Command Lifecycle

**Files:**
- Create: `deploy/utils/hitter_realtime.py`
- Create: `deploy/tests/test_hitter_realtime.py`

**Interfaces:**
- Produces: `BallEstimateSnapshot`
- Produces: `PlannerResultSnapshot`
- Produces: `LatestOnlyPlannerWorker.submit()`, `.latest_result()`, `.close()`
- Produces: `HitterCommandLifecycle.ingest()`, `.mark_track_ended()`, `.advance()`, `.policy_tts()`

- [ ] **Step 1: Write worker and lifecycle tests with a fake clock**

Include tests that prove capacity one, non-starvation, arm, override, skip, max-TTS retention, and recovery preservation:

```python
def test_pending_slot_keeps_only_latest_generation(self):
    blocker = threading.Event()
    seen = []
    def plan(snapshot):
        seen.append(snapshot.generation)
        if snapshot.generation == 1:
            blocker.wait(1.0)
        return fake_command(tts=0.9)
    worker = LatestOnlyPlannerWorker(plan, monotonic_fn=lambda: 10.0)
    worker.submit(snapshot(generation=1))
    wait_until(lambda: seen == [1])
    worker.submit(snapshot(generation=2))
    worker.submit(snapshot(generation=3))
    blocker.set()
    wait_until(lambda: seen == [1, 3])
    self.assertEqual(worker.stats.dropped_pending, 1)
    worker.close()

def test_arm_override_and_recovery_follow_latest_deadline(self):
    lifecycle = HitterCommandLifecycle(
        waiting_tts=1.0,
        arm_tts=0.90,
        minimum_arm_tts=0.80,
        maximum_policy_tts=0.92,
        swing_duration_sampler=lambda: 1.85,
    )
    armed = result(epoch=4, generation=1, deadline=10.90)
    self.assertEqual(lifecycle.ingest(armed, now=10.0), "armed")
    self.assertAlmostEqual(lifecycle.recovery_duration_s, 0.95)
    updated = result(epoch=4, generation=2, deadline=11.00)
    self.assertEqual(lifecycle.ingest(updated, now=10.2), "overridden")
    self.assertAlmostEqual(lifecycle.command_end_deadline_s, 11.95)
    lifecycle.advance(11.00)
    self.assertEqual(lifecycle.phase, CommandPhase.RECOVERY)
    lifecycle.advance(11.95)
    self.assertEqual(lifecycle.phase, CommandPhase.WAITING)

def test_waiting_is_one_second_but_armed_tts_is_capped(self):
    lifecycle = configured_lifecycle()
    self.assertEqual(lifecycle.policy_tts(now=1.0), 1.0)
    lifecycle.ingest(result(epoch=1, generation=1, deadline=1.9), now=1.0)
    self.assertAlmostEqual(lifecycle.policy_tts(now=1.0), 0.9)
```

Also cover: a first unarmed result below `0.80` skips the epoch; a jump from above `0.90` to below `0.80` skips; old generations and old epochs are ignored; a same-track result above `0.92` retains the active command; `mark_track_ended()` stops overrides without deleting an active command; a planner exception creates a failed result and the worker continues.

Add one recovery-overlap test: submit a newer epoch while the current command is in `RECOVERY`, assert it is cached without replacing the active command, then advance past command end and arm it only if its then-current TTS remains in `[0.80, 0.90]`.

- [ ] **Step 2: Run tests and verify the module is missing**

Run: `PYTHONPATH=deploy:. conda run -n rb python -m unittest deploy.tests.test_hitter_realtime -v`

Expected: import failure for `utils.hitter_realtime`.

- [ ] **Step 3: Implement immutable snapshot dataclasses**

Use copied read-only arrays in `BallEstimateSnapshot.__post_init__()` and define:

```python
@dataclass(frozen=True)
class BallEstimateSnapshot:
    track_epoch: int
    generation: int
    source_frame: int
    source_time_s: float
    received_monotonic_s: float
    position_w: np.ndarray
    velocity_w: np.ndarray
    base_position_w: np.ndarray
    base_quaternion_xyzw: np.ndarray
    base_valid: bool
    visible: bool
    ready: bool

@dataclass(frozen=True)
class PlannerResultSnapshot:
    track_epoch: int
    source_generation: int
    source_frame: int
    strike_deadline_monotonic_s: float
    completed_monotonic_s: float
    command: object | None
    error: str | None = None
```

- [ ] **Step 4: Implement the capacity-one worker**

The condition-protected state contains one `_pending` snapshot and one `_latest_result`. `submit()` increments `submitted` and increments `dropped_pending` when overwriting an existing pending item. `_run()` copies and clears pending under lock, runs `plan_fn` outside the lock, increments `completed` or `failed`, publishes success or failure, then immediately checks the newest pending item. Expose immutable `PlannerWorkerStats(submitted, completed, failed, dropped_pending)` and `max_pending_depth`, which must never exceed one. `close()` sets a stop flag, notifies, and joins idempotently.

Compute a successful result deadline as:

```python
deadline = snapshot.received_monotonic_s + float(command.time_to_strike)
```

- [ ] **Step 5: Implement the pure command lifecycle**

Use `CommandPhase(WAITING, TRACKING, ARMED, RECOVERY)`. On arm, sample total swing duration and store:

```python
arm_tts = max(result.strike_deadline_monotonic_s - now, 0.0)
self.recovery_duration_s = max(self.swing_duration_sampler() - arm_tts, 0.0)
self.command_end_deadline_s = result.strike_deadline_monotonic_s + self.recovery_duration_s
```

On an accepted same-track override, replace the entire active result and set:

```python
self.command_end_deadline_s = result.strike_deadline_monotonic_s + self.recovery_duration_s
```

`policy_tts()` returns exactly `1.0` in WAITING/TRACKING and clamps active TTS to `[0.0, 0.92]`. `mark_track_ended()` prevents further results from the ended epoch but leaves an active result until recovery completes. During `RECOVERY`, retain only the latest next-epoch result; when recovery ends, re-evaluate its deadline against the normal arm/late-skip band instead of applying it blindly.

- [ ] **Step 6: Run all realtime utility tests repeatedly**

Run twice to expose thread cleanup races:

```bash
PYTHONPATH=deploy:. conda run -n rb python -m unittest deploy.tests.test_hitter_realtime -v
PYTHONPATH=deploy:. conda run -n rb python -m unittest deploy.tests.test_hitter_realtime -v
```

Expected: both runs pass and exit without a live worker thread.

- [ ] **Step 7: Commit Task 4**

```bash
git add -- deploy/utils/hitter_realtime.py deploy/tests/test_hitter_realtime.py
git commit -m "feat: add latest-only HITTER planner runtime"
```

---

### Task 5: Publish 360 Hz Estimate Snapshots from `RealWorld`

**Files:**
- Modify: `deploy/simulator/real_world.py`
- Create: `deploy/tests/test_real_world_hitter_snapshots.py`

**Interfaces:**
- Consumes: `BallEstimateSnapshot`
- Produces: `register_hitter_ball_listener(listener) -> Callable[[], None]`
- Produces: one snapshot generation per current ball observation and explicit invalid event

- [ ] **Step 1: Write isolated callback tests without opening LCM**

Construct `RealWorld` with `__new__`, call `_init_ball_state()`, set root temporary values, and use a fake `transformation_t` object. Add tests:

```python
def test_each_valid_message_publishes_a_new_snapshot_generation(self):
    sim = make_real_world_without_io()
    received = []
    sim.register_hitter_ball_listener(received.append)
    for frame in range(31):
        sim._update_ball_state_from_vicon(
            ball_message(frame=frame, source_time=frame / 360.0),
            np.array([1.0 - 0.002 * frame, 0.0, 0.90]),
        )
    self.assertEqual(received[-1].generation, 31)
    self.assertEqual(received[-1].source_frame, 30)
    self.assertTrue(received[-1].ready)

def test_outgoing_to_incoming_direction_change_starts_new_epoch(self):
    sim = make_real_world_without_io()
    feed_positions(sim, [1.0, 1.01, 1.02])
    old_epoch = sim.ball_track_epoch
    feed_positions(sim, [1.01, 1.00, 0.99])
    self.assertEqual(sim.ball_track_epoch, old_epoch + 1)
    self.assertLessEqual(sim.ball_state_estimator_sample_count_tmp, 3)

def test_invalid_message_publishes_invalid_snapshot_and_clears_estimator(self):
    sim = make_real_world_without_io()
    received = []
    sim.register_hitter_ball_listener(received.append)
    sim._update_ball_state_from_vicon(invalid_ball_message(), np.zeros(3))
    self.assertFalse(received[-1].visible)
    self.assertFalse(received[-1].ready)
    self.assertEqual(sim.ball_state_estimator.sample_count, 0)

def test_ball_message_does_not_fabricate_base_validity(self):
    sim = make_real_world_without_io()
    sim._update_ball_state_from_vicon(
        ball_message(frame=1, source_time=1.0),
        np.array([1.0, 0.0, 0.9]),
    )
    self.assertFalse(sim.base_pose_valid_tmp)
    self.assertFalse(sim.latest_ball_snapshot.base_valid)
```

- [ ] **Step 2: Run tests and verify listener/epoch support is absent**

Run: `PYTHONPATH=deploy:. conda run -n rb python -m unittest deploy.tests.test_real_world_hitter_snapshots -v`

Expected: failures for missing listener and epoch attributes.

- [ ] **Step 3: Add thread-safe listener and snapshot publication**

Initialize a small listener lock, listener list, generation, epoch, last raw sample/time, previous direction, and a separate `base_pose_valid_tmp` flag. `register_hitter_ball_listener()` appends under lock and returns an unregister closure. Publish to a copied listener list outside the lock so callbacks cannot block state mutation.

Only a valid non-ball/non-table `G1Pelvis` message may set `base_pose_valid_tmp=True`; receiving a ball must not reuse `firstReceiveVicon` as base validity. Each snapshot copies `root_trans_world_tmp`, `root_quat_world_tmp`, and `base_pose_valid_tmp` together with the estimate. Stamp `received_monotonic_s=time.monotonic()` and preserve the source frame/time from the LCM message.

- [ ] **Step 4: Reset the fit on outgoing-to-incoming reversal**

Use consecutive raw positions and source timestamps to classify horizontal direction. When the prior classified direction is non-incoming and the new finite difference has `vx < 0`, increment the track epoch, reset the estimator, and add the current sample as sample one of the new epoch. Do not add a minimum-speed threshold or wall-clock timeout.

- [ ] **Step 5: Publish invalid snapshots during `_clear_ball_tracking()`**

Add an `emit_snapshot` parameter so startup resets can be silent, but a current `valid=false` LCM message emits one invalid snapshot and advances the epoch. Preserve the existing public `reset_ball_state_estimator()` behavior while ensuring old worker results are rejected by the new epoch.

At the second R2 transition into the policy loop, require `base_pose_valid_tmp=True`; if it is false, keep waiting and log a rate-limited message instead of running policy observations with a fabricated zero world pose. This does not change the first joint-only calibration phase.

- [ ] **Step 6: Run RealWorld snapshot and estimator regression tests**

Run:

```bash
PYTHONPATH=deploy:. conda run -n rb python -m unittest \
  deploy.tests.test_real_world_hitter_snapshots \
  deploy.tests.test_hitter_realtime -v
```

Expected: all pass without opening a real LCM policy/action path.

- [ ] **Step 7: Commit Task 5**

```bash
git add -- deploy/simulator/real_world.py deploy/tests/test_real_world_hitter_snapshots.py
git commit -m "feat: publish real-world HITTER ball snapshots"
```

---

### Task 6: Integrate Worker and Command Lifecycle into `HitterEnv`

**Files:**
- Modify: `deploy/envs/hitter.py`
- Modify: `deploy/config/mimic/hitter.yaml`
- Create: `deploy/tests/test_hitter_env_lifecycle.py`

**Interfaces:**
- Consumes: `LatestOnlyPlannerWorker`, `HitterCommandLifecycle`, `PlannerResultSnapshot`
- Produces: `refresh_policy_observation()` for the real-world agent
- Preserves: existing observation concatenation order and action mapping

- [ ] **Step 1: Write lifecycle-integration tests with fake simulator and commands**

Cover these externally visible behaviors:

```python
def test_waiting_observation_uses_user_selected_one_second(self):
    env = make_minimal_hitter_env(is_real=True)
    env._update_hitter_command(now=10.0)
    obs = env.refresh_policy_observation()["obs"]
    self.assertEqual(float(obs[0, 17]), 1.0)

def test_result_above_arm_threshold_does_not_replace_waiting(self):
    env = make_minimal_hitter_env(is_real=True)
    env.inject_result(result(deadline=10.95))
    env._update_hitter_command(now=10.0)
    self.assertFalse(env.hitter_command_initialized)

def test_same_track_override_updates_every_command_field_atomically(self):
    env = make_minimal_hitter_env(is_real=True)
    env.inject_result(result(epoch=2, generation=1, deadline=10.90, marker=1.0))
    env._update_hitter_command(now=10.0)
    env.inject_result(result(epoch=2, generation=2, deadline=10.85, marker=2.0))
    env._update_hitter_command(now=10.1)
    self.assertCommandMarker(env, 2.0)

def test_track_end_clears_planner_but_keeps_command_through_recovery(self):
    env = armed_env(strike_deadline=10.9, end_deadline=11.85)
    env.inject_invalid_track(epoch=3)
    env._update_hitter_command(now=10.5)
    self.assertTrue(env.hitter_command_initialized)
    env._update_hitter_command(now=11.86)
    self.assertFalse(env.hitter_command_initialized)
```

Also assert that old generations, old epochs, failed results, TTS `>0.92`, and late unarmed TTS `<0.80` do not mutate active command fields.

- [ ] **Step 2: Run tests and verify current synchronous state fails**

Run: `PYTHONPATH=deploy:. conda run -n rb python -m unittest deploy.tests.test_hitter_env_lifecycle -v`

Expected: failures for missing lifecycle/worker integration and waiting TTS still `0.86`.

- [ ] **Step 3: Instantiate the asynchronous path only for real-world**

When `simulator.is_real` and `register_hitter_ball_listener` exists:

```python
self.hitter_planner_worker = LatestOnlyPlannerWorker(
    self._plan_hitter_snapshot,
    monotonic_fn=time.monotonic,
)
self._unregister_ball_listener = self.simulator.register_hitter_ball_listener(
    self.hitter_planner_worker.submit
)
```

`_plan_hitter_snapshot()` must reject a snapshot unless `visible`, `ready`, and `base_valid` are all true. It then uses only the snapshot's ball and base arrays; it must not call `simulator.get_state()` from the worker thread. Convert base quaternion to forward XY and call the existing `HitterSystemPlanner`. Pass `minimum_hit_height=table_height`, `maximum_hit_height=table_height + maximum_hit_height_above_table_m`, and `maximum_prediction_horizon_s` when building the planner.

For MuJoCo, keep synchronous planning on each 50 Hz policy update but wrap the result in the same `PlannerResultSnapshot` and feed the same lifecycle.

- [ ] **Step 4: Replace elapsed-step command mutation with lifecycle decisions**

Delete the real-world `elapsed += high_dt` deadline path. `_update_hitter_command(now=None)` obtains `now=time.monotonic()`, consumes at most the latest completed result, calls lifecycle `ingest()`/`advance()`, and copies a command only for `armed` or `overridden` decisions.

Copy all command fields in one helper after validation:

```python
def _copy_hitter_command(self, command) -> None:
    self.hitter_strike_type = 0 if command.strike_type == "forehand" else 1
    self.hitter_base_target_xy_w = np.asarray(command.p_base_target_xy, dtype=np.float32).copy()
    self.hitter_racket_target_pos_w_fixed = np.asarray(command.strike_plan.p_racket_target, dtype=np.float32).copy()
    self.hitter_racket_target_vel_w = np.asarray(command.v_racket_target_w, dtype=np.float32).copy()
    self.hitter_ball_out_vel_w = np.asarray(command.strike_plan.v_ball_out, dtype=np.float32).copy()
```

Observation TTS comes only from `lifecycle.policy_tts(time.monotonic())`; waiting returns exactly `1.0`.

Log every lifecycle transition (`WAITING`, `TRACKING`, `ARMED`, `RECOVERY`), arm, override, late-skip, and track-end event once at the event. Add a rate-limited status log containing worker submitted/completed/failed/dropped counts, latest result age, track epoch, generation, and policy TTS; never print one log per 360 Hz frame.

- [ ] **Step 5: Add explicit close/reset invalidation**

On env reset, increment/clear lifecycle state and invalidate old worker results. Add `close()` that unregisters the listener, closes the worker, then closes the simulator. Make both paths idempotent.

- [ ] **Step 6: Update final HITTER configuration**

Set:

```yaml
motion:
  waiting_time_to_strike_s: 1.00
  ball_planner:
    table_center_xy_w: [1.365369, 0.0]
    table_length: 2.730738
    table_width: 1.512451
    table_height: 0.760000
    virtual_hit_plane_x: 0.0
    state_estimator_sample_rate_hz: 360.0
    arm_time_to_strike_s: 0.90
    minimum_arm_time_to_strike_s: 0.80
    maximum_policy_time_to_strike_s: 0.92
    maximum_hit_height_above_table_m: 0.50
    maximum_prediction_horizon_s: 5.0
    swing_duration_range: [1.75, 1.95]
```

- [ ] **Step 7: Run lifecycle, worker, and planner tests**

Run:

```bash
PYTHONPATH=deploy:. conda run -n rb python -m unittest \
  deploy.tests.test_hitter_planner_boundaries \
  deploy.tests.test_hitter_realtime \
  deploy.tests.test_real_world_hitter_snapshots \
  deploy.tests.test_hitter_env_lifecycle -v
```

Expected: all tests pass and no worker thread remains alive.

- [ ] **Step 8: Commit Task 6**

```bash
git add -- deploy/envs/hitter.py deploy/config/mimic/hitter.yaml deploy/tests/test_hitter_env_lifecycle.py
git commit -m "feat: drive HITTER commands from realtime planner results"
```

---

### Task 7: Refresh Observation Before ONNX and Align MuJoCo Geometry

**Files:**
- Modify: `deploy/agents/hitter_agent.py`
- Modify: `deploy/config/hitter.yaml`
- Modify: `deploy/data/assets/g1/g1_29dof_hitter_racket_table_tennis.xml`
- Create: `deploy/tests/test_hitter_agent_timing.py`
- Create: `deploy/tests/test_hitter_geometry_config.py`

**Interfaces:**
- Consumes: `HitterEnv.refresh_policy_observation()`
- Preserves: one ONNX inference and one action application per 20 ms tick

- [ ] **Step 1: Write a real-vs-MuJoCo observation-order test**

Extract a finite `_run_iteration(obs_buf_dict)` helper and test with fakes:

```python
def test_real_iteration_refreshes_before_policy_inference(self):
    events = []
    agent = make_fake_agent(is_real=True, events=events)
    agent._run_iteration({"obs": np.zeros((1, 105), dtype=np.float32)})
    self.assertEqual(events, ["refresh", "policy", "step"])

def test_mujoco_iteration_does_not_double_refresh(self):
    events = []
    agent = make_fake_agent(is_real=False, events=events)
    agent._run_iteration({"obs": np.zeros((1, 105), dtype=np.float32)})
    self.assertEqual(events, ["policy", "step"])
```

- [ ] **Step 2: Run timing tests and verify the helper is absent**

Run: `PYTHONPATH=deploy:. conda run -n rb python -m unittest deploy.tests.test_hitter_agent_timing -v`

Expected: failure for missing `_run_iteration` / `refresh_policy_observation`.

- [ ] **Step 3: Refresh real observation immediately before inference**

Implement:

```python
def _run_iteration(self, obs_buf_dict):
    if self.env.simulator.is_real:
        obs_buf_dict = self.env.refresh_policy_observation()
    inputs = {key: value.astype(np.float32) for key, value in obs_buf_dict.items()}
    action = self.policy.run(None, inputs)[0]
    return self.env.step(action)
```

Call this helper once per loop in both `run()` and `run_eval()`. Keep the existing 20 ms pacing and do not add a second action publication. Wrap each loop in `try/finally` and call `env.close()` in the `finally` block so Ctrl-C cannot leave the planner worker alive.

- [ ] **Step 4: Write and run geometry parity tests**

Compose the HITTER Hydra config with `sim=mujoco`, then parse the MuJoCo XML. Assert the composed `sim.config.table_tennis` values match `mimic.motion.ball_planner`, and assert table top/center line/net half-length and half-width use the accepted effective dimensions.

Run: `PYTHONPATH=deploy:. conda run -n rb python -m unittest deploy.tests.test_hitter_geometry_config -v`

Expected before implementation: standard `2.74 / 1.525` XML dimensions fail the exact accepted-value assertions.

- [ ] **Step 5: Update only HITTER MuJoCo table geometry**

Add a HITTER-scoped `sim.config.table_tennis` block in `deploy/config/hitter.yaml`, interpolating geometry, ball radius, drag, and restitution from `mimic.motion.ball_planner`; generic `sim/mujoco.yaml` remains unchanged. Set XML table center x to `1.365369`, top and center-line half-length to `1.365369`, half-width to `0.7562255`, and net x to `1.365369`. Keep tabletop surface at `z=0.760000` and adjust leg x/y symmetrically inside the new bounds. Do not change racket or foot collision geometry.

- [ ] **Step 6: Run timing and geometry tests**

Run:

```bash
PYTHONPATH=deploy:. conda run -n rb python -m unittest \
  deploy.tests.test_hitter_agent_timing \
  deploy.tests.test_hitter_geometry_config -v
```

Expected: all pass; fake real events are exactly refresh-policy-step.

- [ ] **Step 7: Commit Task 7**

```bash
git add -- deploy/agents/hitter_agent.py \
  deploy/config/hitter.yaml \
  deploy/data/assets/g1/g1_29dof_hitter_racket_table_tennis.xml \
  deploy/tests/test_hitter_agent_timing.py deploy/tests/test_hitter_geometry_config.py
git commit -m "feat: refresh HITTER timing and align table geometry"
```

---

### Task 8: Full Offline Regression and Recorded-Trajectory Replay

**Files:**
- Create: `deploy/tests/test_hitter_recorded_replay.py`
- No production-file edit is expected in this task; if a regression fails, return to the task that owns that component before changing code.

**Interfaces:**
- Consumes: `/tmp/chingmu_bounce_confirm3.csv` when present
- Verifies: source snapshot -> estimator -> latest-only planner -> command lifecycle

- [ ] **Step 1: Add a deterministic synthetic 360 Hz replay test**

Generate a table-bound incoming trajectory with known timestamps and assert:

```python
self.assertEqual(first_ready_frame, 30)
self.assertEqual(worker.max_pending_depth, 1)
self.assertGreaterEqual(first_arm_tts, 0.80)
self.assertLessEqual(first_arm_tts, 0.90)
self.assertEqual(lifecycle.policy_tts(waiting_time), 1.00)
self.assertFalse(any(result.source_generation < previous for result in consumed_results))
```

Cross `x=0`, send the explicit invalid event, and assert estimator sample count becomes zero while the active command remains until its command-end deadline.

In the same file, add an optional recorded diagnostic that is skipped only when the CSV is absent:

```python
RECORDED = Path(os.environ.get("CHINGMU_REPLAY_CSV", "/tmp/chingmu_bounce_confirm3.csv"))

@unittest.skipUnless(RECORDED.exists(), "recorded ChingMu CSV is not available")
def test_recorded_four_bounce_capture_clears_track(self):
    stats = replay_recorded_csv(RECORDED)
    self.assertEqual(stats.frame_gaps, 0)
    self.assertEqual(stats.bounce_count, 4)
    self.assertFalse(stats.visible_after_track_end)
    print(json.dumps(dataclasses.asdict(stats), sort_keys=True))
```

- [ ] **Step 2: Run the replay test and correct only demonstrated integration defects**

Run: `PYTHONPATH=deploy:. conda run -n rb python -m unittest deploy.tests.test_hitter_recorded_replay -v`

Expected: pass after Tasks 1-7; any failure must identify the exact broken boundary before a code change.

- [ ] **Step 3: Run every HITTER and ChingMu unittest**

Run:

```bash
PYTHONPATH=deploy:. conda run -n rb python -m unittest discover -s deploy/tests -v
conda run -n rb python -m unittest discover -s deploy/mocap_bridge/tests -v
```

Expected: all tests pass with no skipped core lifecycle tests.

- [ ] **Step 4: Run static and import checks**

Run:

```bash
conda run -n rb python -m compileall -q \
  deploy/mocap_bridge/chingmu_sdk_client.py \
  deploy/mocap_bridge/chingmu_table_lcm_bridge.py \
  deploy/utils/hitter_planner.py deploy/utils/hitter_realtime.py \
  deploy/simulator/real_world.py deploy/envs/hitter.py deploy/agents/hitter_agent.py
git diff --check
```

Expected: exit code zero and no whitespace errors.

- [ ] **Step 5: Replay the recorded four-bounce capture when available**

Run without creating a real simulator or publishing actions:

```bash
CHINGMU_REPLAY_CSV=/tmp/chingmu_bounce_confirm3.csv \
  PYTHONPATH=deploy:. conda run -n rb python -m unittest \
  deploy.tests.test_hitter_recorded_replay.HitterRecordedReplayTest.test_recorded_four_bounce_capture_clears_track -v
```

The printed `ReplayStats` must report:

- frame count and source gaps;
- estimator ready transitions and four bounce resets;
- worker submitted/completed/dropped counts;
- whether any candidate enters the `0.80-0.90 s` arm band;
- explicit track end and estimator clear;
- maximum planner-result age consumed by the 50 Hz policy simulation.

The previously captured four-bounce trajectory is diagnostic and may legitimately skip arm if no valid result enters the arm band.

- [ ] **Step 6: Inspect final scope and process safety**

Run:

```bash
git status --short
pgrep -af '[r]un.py --config-name=hitter' || true
```

Expected: no real HITTER policy process was started. Confirm unrelated user-owned files were neither staged nor overwritten.

- [ ] **Step 7: Commit the replay test and any narrowly required integration correction**

```bash
git add -- deploy/tests/test_hitter_recorded_replay.py
git commit -m "test: cover HITTER realtime planner replay"
```

---

## Final Acceptance Evidence

The implementation is complete only when the handoff contains all of the following:

- the exact test commands and passing counts;
- proof that waiting observation carries `TTS=1.00 s` while armed TTS never exceeds `0.92 s`;
- proof that a same-track result overrides all command fields atomically;
- proof that pending planner depth never exceeds one and old epochs cannot re-arm;
- proof that `G1Pelvis` position comes from the SDK root report rather than marker centroid;
- proof that `x=0` emits one explicit ball end and clears estimator/planner state;
- proof that recovery duration is preserved when strike deadline changes;
- proof that real-world observation refresh occurs immediately before ONNX while action stays 50 Hz;
- confirmation that no real robot policy was launched.
