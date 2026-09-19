# Vicon Tracer DataStream Runtime Switch Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the existing Vicon bridge consume Tracer's `G1Pelvis` root-segment centroid directly, rebuild the table frame from the four installed corner markers, and preserve the current `vicon_state_data` contract for HITTER.

**Architecture:** Keep `vicon_table_lcm_bridge.cpp` as the only Vicon/Tracer producer and add a minimal same-translation-unit C++ test harness around its pure helpers. Replace marker-centroid/Horn base reconstruction with the DataStream root segment, construct a full 3D table frame from exactly four stable corners, exclude saved corners before ball tracking, and leave every downstream Python component unchanged.

**Tech Stack:** C++17, Vicon DataStream SDK, LCM, standard-library C++ test executable, Python 3 `unittest`, RobotBridge2 `rb` Conda environment, NetworkManager.

## Global Constraints

- Modify the existing `deploy/mocap_bridge/vicon_table_lcm_bridge.cpp`; do not create a second Tracer bridge.
- Keep the ChingMu bridge installed and unchanged; never run it concurrently with the Vicon bridge.
- The exact, case-sensitive subject name is `G1Pelvis`.
- Use the Tracer root-segment translation directly as the pelvis origin: no labelled-marker fit, marker-centroid fallback, ChingMu offset, or configured base anchor.
- Publish only relative startup yaw from mocap; roll and pitch remain robot-IMU inputs.
- Preserve the `vicon_state_data` channel, `transformation_t` schema, message names, units, and downstream consumers.
- Table geometry is exactly `length=2.730738 m`, `width=1.512451 m`, `height=0.760000 m`.
- Robot-side table edge is `x=0`, far edge is `x=2.730738`, table centre is `y=0`, and tabletop is `z=0.760000`.
- Require exactly four stable unlabeled clusters during calibration and a valid direct `G1Pelvis` root pose.
- Reject any table candidate whose best one-to-one transformed-corner assignment has maximum Euclidean error greater than `0.050 m`.
- Exclude runtime unlabeled markers at distance `<=50.0 mm` from a saved table corner before ball selection.
- Prefer the finite positive SDK `GetFrameRate()` result; `--vicon-frame-rate-hz` is fallback only.
- Do not modify `real_world.py`, the estimator, planner, policy, WAITING target, TTS, or swing lifecycle.
- Calibration, probe, bridge, and LCM validation steps must not send robot commands.
- Preserve all unrelated dirty-worktree files and stage only files named in each task.

---

## File map

- Modify: `deploy/mocap_bridge/vicon_table_lcm_bridge.cpp` — root-segment input, pose math, table calibration, persistence, ball filtering, LCM publication.
- Modify: `deploy/mocap_bridge/nexus_probe_cpp.cpp` — base-only root-pose probe and SDK frame-rate output.
- Modify: `deploy/mocap_bridge/build_cpp_probe.sh` — build the deterministic bridge test executable.
- Create: `deploy/mocap_bridge/tests/test_vicon_table_lcm_bridge.cpp` — C++ unit coverage without live Vicon hardware.
- Modify: `deploy/mocap_bridge/monitor_vicon_lcm.py` — expose actual `valid` and `occluded` fields during LCM checks.
- Create: `deploy/mocap_bridge/tests/test_monitor_vicon_lcm.py` — status-field regression coverage.
- Generate live, do not hand-edit: `deploy/mocap_bridge/calibrations/vicon_table_frame_candidate.json`.
- Replace only after explicit live acceptance: `deploy/mocap_bridge/calibrations/table_frame_latest.json`.

---

### Task 1: Add the C++ test seam and pure root-pose helpers

**Files:**
- Modify: `deploy/mocap_bridge/vicon_table_lcm_bridge.cpp:27-146,392-515,769-806,821-1080`
- Modify: `deploy/mocap_bridge/build_cpp_probe.sh:64-78`
- Create: `deploy/mocap_bridge/tests/test_vicon_table_lcm_bridge.cpp`

**Interfaces:**
- Consumes: existing `Vec3`, `Quat`, `TableFrame`, `QuatToMatrix`, and `RawToTableWorld` helpers.
- Produces: `SegmentPoseRaw`, `DecodeSegmentPose(...)`, `SelectFrameRateHz(...)`, and `RelativeTableYawQuat(...)` for Task 2.

- [ ] **Step 1: Add a failing same-translation-unit C++ test**

Create `deploy/mocap_bridge/tests/test_vicon_table_lcm_bridge.cpp` with the test macro before including the production source:

```cpp
#define VICON_TABLE_LCM_BRIDGE_TESTING
#include "../vicon_table_lcm_bridge.cpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <string>

namespace {

int failures = 0;
constexpr double kPi = 3.14159265358979323846;

void Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << "\n";
    ++failures;
  }
}

void ExpectNear(double actual, double expected, double tolerance, const std::string& message) {
  Expect(std::abs(actual - expected) <= tolerance, message);
}

Quat YawQuat(double yaw) {
  return {0.0, 0.0, std::sin(0.5 * yaw), std::cos(0.5 * yaw)};
}

TableFrame IdentityTable() {
  TableFrame table;
  table.valid = true;
  table.x_axis_raw = {1.0, 0.0, 0.0};
  table.y_axis_raw = {0.0, 1.0, 0.0};
  table.z_axis_raw = {0.0, 0.0, 1.0};
  return table;
}

void TestSegmentPoseValidation() {
  const auto valid = DecodeSegmentPose(
      true, false, {100.0, 200.0, 300.0},
      true, false, {0.0, 0.0, 0.0, 2.0});
  Expect(valid.valid, "finite non-occluded root pose is valid");
  ExpectNear(valid.quat_xyzw.w, 1.0, 1.0e-12, "root quaternion is normalized");

  const auto occluded = DecodeSegmentPose(
      true, true, {100.0, 200.0, 300.0},
      true, false, {0.0, 0.0, 0.0, 1.0});
  Expect(!occluded.valid && occluded.occluded, "translation occlusion invalidates root pose");

  const auto nan_pose = DecodeSegmentPose(
      true, false, {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
      true, false, {0.0, 0.0, 0.0, 1.0});
  Expect(!nan_pose.valid, "non-finite root pose is invalid");
}

void TestRelativeYaw() {
  const TableFrame table = IdentityTable();
  const Quat relative = RelativeTableYawQuat(
      YawQuat(30.0 * kPi / 180.0),
      YawQuat(40.0 * kPi / 180.0), table);
  ExpectNear(relative.z, std::sin(5.0 * kPi / 180.0), 1.0e-9,
             "relative yaw is current minus startup heading");
  ExpectNear(relative.w, std::cos(5.0 * kPi / 180.0), 1.0e-9,
             "relative yaw quaternion has expected scalar component");

  const Quat wrapped = RelativeTableYawQuat(
      YawQuat(170.0 * kPi / 180.0),
      YawQuat(-170.0 * kPi / 180.0), table);
  ExpectNear(2.0 * std::atan2(wrapped.z, wrapped.w), 20.0 * kPi / 180.0,
             1.0e-9, "relative yaw wraps through pi correctly");
}

void TestFrameRateSelection() {
  const auto sdk = SelectFrameRateHz(true, 360.0, 300.0);
  Expect(sdk.has_value(), "valid SDK rate is selected");
  ExpectNear(*sdk, 360.0, 0.0, "SDK rate wins over fallback");

  const auto fallback = SelectFrameRateHz(false, 0.0, 300.0);
  Expect(fallback.has_value(), "finite positive fallback is accepted");
  ExpectNear(*fallback, 300.0, 0.0, "fallback rate is used after SDK failure");

  Expect(!SelectFrameRateHz(false, 0.0, 0.0).has_value(),
         "invalid SDK and fallback rates fail closed");
}

}  // namespace

int main() {
  TestSegmentPoseValidation();
  TestRelativeYaw();
  TestFrameRateSelection();
  if (failures == 0) std::cout << "test_vicon_table_lcm_bridge: PASS\n";
  return failures == 0 ? 0 : 1;
}
```

- [ ] **Step 2: Add the test build command and prove it fails**

Append this build rule after the production bridge rule in `build_cpp_probe.sh`:

```bash
g++ -std=c++17 -O2 \
  -I"${SDK_DIR}" \
  -I"${ROBOTBRIDGE_DIR}" \
  "${LCM_CFLAGS[@]}" \
  "${SCRIPT_DIR}/tests/test_vicon_table_lcm_bridge.cpp" \
  -L"${SDK_DIR}" \
  -Wl,-rpath,"${SDK_DIR}" \
  -lViconDataStreamSDK_CPP \
  "${LCM_LIBS[@]}" \
  -o "${SCRIPT_DIR}/bin/test_vicon_table_lcm_bridge"
```

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2
bash deploy/mocap_bridge/build_cpp_probe.sh
```

Expected: compilation fails because the production `main` is still visible and the three new helper interfaces are undefined.

- [ ] **Step 3: Implement the minimal pure helpers**

Add `<optional>`, guard the production entry point, and implement these exact contracts inside the existing anonymous namespace:

```cpp
struct SegmentPoseRaw {
  Vec3 translation_mm;
  Quat quat_xyzw;
  bool valid = false;
  bool occluded = true;
};

SegmentPoseRaw DecodeSegmentPose(
    bool translation_success,
    bool translation_occluded,
    const std::array<double, 3>& translation_mm,
    bool rotation_success,
    bool rotation_occluded,
    const std::array<double, 4>& rotation_xyzw) {
  SegmentPoseRaw pose;
  pose.translation_mm = {translation_mm[0], translation_mm[1], translation_mm[2]};
  pose.quat_xyzw = Normalize(
      {rotation_xyzw[0], rotation_xyzw[1], rotation_xyzw[2], rotation_xyzw[3]});
  pose.occluded = translation_occluded || rotation_occluded;
  const double raw_quat_norm = std::sqrt(
      rotation_xyzw[0] * rotation_xyzw[0] +
      rotation_xyzw[1] * rotation_xyzw[1] +
      rotation_xyzw[2] * rotation_xyzw[2] +
      rotation_xyzw[3] * rotation_xyzw[3]);
  pose.valid = translation_success && rotation_success && !pose.occluded &&
      std::isfinite(pose.translation_mm.x) &&
      std::isfinite(pose.translation_mm.y) &&
      std::isfinite(pose.translation_mm.z) &&
      std::isfinite(raw_quat_norm) && raw_quat_norm > 1.0e-12;
  return pose;
}

std::optional<double> SelectFrameRateHz(
    bool sdk_success, double sdk_hz, double fallback_hz) {
  if (sdk_success && std::isfinite(sdk_hz) && sdk_hz > 0.0) return sdk_hz;
  if (std::isfinite(fallback_hz) && fallback_hz > 0.0) return fallback_hz;
  return std::nullopt;
}

Quat RelativeTableYawQuat(
    const Quat& initial_raw_q,
    const Quat& current_raw_q,
    const TableFrame& table) {
  double initial_raw[3][3];
  double current_raw[3][3];
  QuatToMatrix(initial_raw_q, initial_raw);
  QuatToMatrix(current_raw_q, current_raw);
  const double table_from_raw[3][3] = {
      {table.x_axis_raw.x, table.x_axis_raw.y, table.x_axis_raw.z},
      {table.y_axis_raw.x, table.y_axis_raw.y, table.y_axis_raw.z},
      {table.z_axis_raw.x, table.z_axis_raw.y, table.z_axis_raw.z},
  };
  double initial_table[3][3]{};
  double current_table[3][3]{};
  double relative[3][3]{};
  Multiply3(table_from_raw, initial_raw, initial_table);
  Multiply3(table_from_raw, current_raw, current_table);
  MultiplyByTranspose3(current_table, initial_table, relative);
  const double yaw = std::atan2(relative[1][0], relative[0][0]);
  return Normalize({0.0, 0.0, std::sin(0.5 * yaw), std::cos(0.5 * yaw)});
}
```

Define `Multiply3` and `MultiplyByTranspose3` as fixed-size three-loop matrix
helpers immediately above `RelativeTableYawQuat`. Insert
`#ifndef VICON_TABLE_LCM_BRIDGE_TESTING` immediately before the existing
production `int main(...)` definition and `#endif` immediately after its final
closing brace. Do not copy, abbreviate, or replace the existing main body.

- [ ] **Step 4: Build and run the new test**

Run:

```bash
bash deploy/mocap_bridge/build_cpp_probe.sh
./deploy/mocap_bridge/bin/test_vicon_table_lcm_bridge
```

Expected: all C++ tools build and the test prints `test_vicon_table_lcm_bridge: PASS`.

- [ ] **Step 5: Commit the test seam and pose helpers**

```bash
git add -- \
  deploy/mocap_bridge/vicon_table_lcm_bridge.cpp \
  deploy/mocap_bridge/build_cpp_probe.sh \
  deploy/mocap_bridge/tests/test_vicon_table_lcm_bridge.cpp
git commit -m "test: add Vicon bridge pose harness"
```

---

### Task 2: Replace marker-centroid base reconstruction with the Tracer root segment

**Files:**
- Modify: `deploy/mocap_bridge/vicon_table_lcm_bridge.cpp:50-70,217-228,463-515,517-570,840-1050`
- Modify: `deploy/mocap_bridge/nexus_probe_cpp.cpp:24-65,112-142,145-290`
- Test: `deploy/mocap_bridge/tests/test_vicon_table_lcm_bridge.cpp`

**Interfaces:**
- Consumes: `SegmentPoseRaw`, `DecodeSegmentPose`, `SelectFrameRateHz`, and `RelativeTableYawQuat` from Task 1.
- Produces: `BasePoseWorld`, `TransformRootPose(...)`, `ReadRootSegmentPose(Client*, const std::string&, const std::string&) -> SegmentPoseRaw`, and direct `G1Pelvis` LCM messages for later live tasks.

- [ ] **Step 1: Extend the failing test for direct centroid and invalid-pose behavior**

Add tests proving that the root translation passes directly through the table transform and no anchor is involved:

```cpp
void TestDirectRootTranslation() {
  Args args;
  args.table_length_m = 2.730738;
  args.table_height_m = 0.760000;
  TableFrame table = IdentityTable();
  table.center_raw_mm = {1365.369, 0.0, 760.0};
  SegmentPoseRaw raw;
  raw.translation_mm = {-400.0, 0.0, 793.0};
  raw.quat_xyzw = {0.0, 0.0, 0.0, 1.0};
  raw.valid = true;
  raw.occluded = false;
  const BasePoseWorld world = TransformRootPose(
      raw, raw.quat_xyzw, table, args);
  Expect(world.valid, "valid direct root transforms successfully");
  ExpectNear(world.position_m.x, -0.4, 1.0e-9,
             "root centroid x is not anchored or zeroed");
  ExpectNear(world.position_m.y, 0.0, 1.0e-9,
             "root centroid y is direct table position");
  ExpectNear(world.position_m.z, 0.793, 1.0e-9,
             "root centroid z is direct table position");
}

void TestSourceFrameTiming() {
  ExpectNear(FrameDeltaSeconds(101, 100, 360.0), 1.0 / 360.0, 1.0e-12,
             "one source frame uses SDK rate");
  ExpectNear(FrameDeltaSeconds(103, 100, 360.0), 3.0 / 360.0, 1.0e-12,
             "frame gaps preserve SDK timing");
  Args args;
  lcm_types::transformation_t message;
  FillMessage(&message, "G1Pelvis", {-0.4, 0.0, 0.793}, Quat{},
              720, args, 360.0, true, false);
  ExpectNear(message.vicon_time_s, 2.0, 1.0e-12,
             "LCM source time uses SDK rate");
  Expect(message.valid == 1 && message.occluded == 0,
         "LCM validity is preserved");
}
```

Add both new tests to the test `main` and run once before changing production
flow. Expected: compilation fails because `BasePoseWorld`, `TransformRootPose`,
and the source-rate `FillMessage` overload do not exist.

- [ ] **Step 2: Add the direct transform and root-segment SDK reader**

Implement the pure direct transform first:

```cpp
struct BasePoseWorld {
  Vec3 position_m;
  Quat relative_yaw_xyzw;
  bool valid = false;
};

BasePoseWorld TransformRootPose(
    const SegmentPoseRaw& raw,
    const Quat& initial_raw_q,
    const TableFrame& table,
    const Args& args) {
  BasePoseWorld world;
  if (!raw.valid || !table.valid) return world;
  world.position_m = RawToTableWorld(raw.translation_mm, table, args);
  world.relative_yaw_xyzw =
      RelativeTableYawQuat(initial_raw_q, raw.quat_xyzw, table);
  world.valid = std::isfinite(world.position_m.x) &&
      std::isfinite(world.position_m.y) &&
      std::isfinite(world.position_m.z);
  return world;
}
```

Then implement the SDK adapter without embedding fallback policy:

```cpp
SegmentPoseRaw ReadRootSegmentPose(
    Client* client,
    const std::string& subject,
    const std::string& root_segment) {
  const auto translation = client->GetSegmentGlobalTranslation(subject, root_segment);
  const auto rotation = client->GetSegmentGlobalRotationQuaternion(subject, root_segment);
  return DecodeSegmentPose(
      Ok(translation.Result), translation.Occluded,
      {translation.Translation[0], translation.Translation[1], translation.Translation[2]},
      Ok(rotation.Result), rotation.Occluded,
      {rotation.Rotation[0], rotation.Rotation[1], rotation.Rotation[2], rotation.Rotation[3]});
}
```

After the first DataStream frame, resolve the exact root once and fail if it is unavailable:

```cpp
const auto root = client.GetSubjectRootSegmentName(args.base_subject);
if (!Ok(root.Result) || std::string(root.SegmentName).empty()) {
  std::cerr << "Could not resolve root segment for " << args.base_subject << "\n";
  return 1;
}
const std::string base_segment = root.SegmentName;
```

Wait up to five seconds for the first valid direct root pose. Preserve its raw translation for table-axis orientation and its normalized quaternion as `initial_base_raw_q`. Do not fall back to marker data or an old table axis during a new calibration.

- [ ] **Step 3: Use the SDK frame rate with an explicit fallback**

Immediately after a successful first frame, select the source rate:

```cpp
const auto sdk_rate = client.GetFrameRate();
const auto selected_rate = SelectFrameRateHz(
    Ok(sdk_rate.Result), sdk_rate.FrameRateHz, args.vicon_frame_rate_hz);
if (!selected_rate.has_value()) {
  std::cerr << "No finite positive Vicon frame rate or fallback\n";
  return 1;
}
const double source_frame_rate_hz = *selected_rate;
std::cout << "Vicon frame rate=" << source_frame_rate_hz << " Hz source="
          << (Ok(sdk_rate.Result) && std::isfinite(sdk_rate.FrameRateHz) &&
                      sdk_rate.FrameRateHz > 0.0
                  ? "sdk"
                  : "fallback")
          << "\n";
```

Pass `source_frame_rate_hz` explicitly to `SelectBall`, `UpdateBallTrack`, and `FillMessage`; use it for `FrameDeltaSeconds` and `vicon_time_s`. Update `--help` so `--vicon-frame-rate-hz` is described as fallback only.

- [ ] **Step 4: Remove the marker-template and base-anchor data path**

Delete `VisibleSubjectMarkers`, `FitRigidHorn`, the startup reference-marker capture, per-frame Horn reconstruction, and the configured anchor calculation. Remove `base_anchor_x_m`, `base_anchor_y_m`, `base_anchor_z_m`, `calibrate_base_anchor`, and their CLI options. Keep `expected_base_edge_distance_m` only for diagnostics.

After that deletion, also remove helpers used only by the Horn path (`Mean`,
`Det3`, `MatrixToQuat`, and `Rotate`) and the unused `<map>` include. Do not call
`EnableMarkerData()`; the new bridge requires only segment data and unlabeled
marker data.

Each runtime frame must instead do exactly this:

```cpp
const SegmentPoseRaw base_raw =
    ReadRootSegmentPose(&client, args.base_subject, base_segment);
const BasePoseWorld transformed =
    TransformRootPose(base_raw, initial_base_raw_q, table, args);
const Vec3 base_world = transformed.valid ? transformed.position_m : Vec3{};
const Quat base_world_q =
    transformed.valid ? transformed.relative_yaw_xyzw : Quat{};
const bool base_valid = transformed.valid;
```

Continue publishing a `G1Pelvis` message on invalid frames, but set
`valid=0`, `occluded=1`, position zero, and identity quaternion. Preserve the existing safety gate that suppresses ball publication when `base_valid` is false.

- [ ] **Step 5: Make the probe support base-only inspection and print SDK rate**

In `nexus_probe_cpp.cpp`, accept either a complete ball pair or a base subject:

```cpp
const bool ball_enabled = !args.ball_subject.empty() && !args.ball_marker.empty();
if (!ball_enabled && args.base_subject.empty()) {
  std::cerr << "provide --base-subject or both --ball-subject/--ball-marker\n";
  return 2;
}
```

Only request the ball marker when `ball_enabled`. After the first successful frame, print `client.GetFrameRate().FrameRateHz` when valid. Update usage with this supported command:

```text
nexus_probe_cpp --host HOST --base-subject G1Pelvis --duration 5
```

- [ ] **Step 6: Build, test, and prove the old base path is absent**

Run:

```bash
bash deploy/mocap_bridge/build_cpp_probe.sh
./deploy/mocap_bridge/bin/test_vicon_table_lcm_bridge
rg -n 'FitRigidHorn|VisibleSubjectMarkers|calibrate-base-anchor|base_anchor_' \
  deploy/mocap_bridge/vicon_table_lcm_bridge.cpp
```

Expected: build and test pass; `rg` returns no matches.

- [ ] **Step 7: Commit direct Tracer root consumption**

```bash
git add -- \
  deploy/mocap_bridge/vicon_table_lcm_bridge.cpp \
  deploy/mocap_bridge/nexus_probe_cpp.cpp \
  deploy/mocap_bridge/tests/test_vicon_table_lcm_bridge.cpp
git commit -m "feat: consume Tracer root segment pose"
```

---

### Task 3: Build and validate the full 3D table frame

**Files:**
- Modify: `deploy/mocap_bridge/vicon_table_lcm_bridge.cpp:73-86,573-688,895-955`
- Test: `deploy/mocap_bridge/tests/test_vicon_table_lcm_bridge.cpp`

**Interfaces:**
- Consumes: first valid direct root translation from Task 2.
- Produces: `TableDimensions`, `CalibrationResult`, `BuildTableFrameFromCorners(...)`, `ValidateTableFrame(...)`, and an ordered, right-handed `TableFrame` for Tasks 4 and 5.

- [ ] **Step 1: Write failing synthetic tilted-table tests**

Add `TableDimensions`-based tests that generate the raw corners from a known tilted orthonormal frame, shuffle them, and reconstruct the frame:

```cpp
void TestTiltedTableCalibration() {
  const TableDimensions dims{2.730738, 1.512451, 0.760000};
  const Vec3 z = NormalizeVec({0.10, -0.20, 0.974679434});
  Vec3 x_seed{0.98, 0.18, -0.06};
  const Vec3 x = NormalizeVec(x_seed - Dot(x_seed, z) * z);
  const Vec3 y = NormalizeVec(Cross(z, x));
  const Vec3 center{900.0, 2100.0, 750.0};
  const double hl = dims.length_m * 500.0;
  const double hw = dims.width_m * 500.0;
  const std::array<Vec3, 4> shuffled = {
      center + hl * x + hw * y,
      center - hl * x - hw * y,
      center + hl * x - hw * y,
      center - hl * x + hw * y,
  };
  const Vec3 pelvis = center - (hl + 400.0) * x + 40.0 * z;
  const CalibrationResult result = BuildTableFrameFromCorners(shuffled, pelvis, dims);
  Expect(result.frame.valid, "tilted table calibration succeeds");
  Expect(result.validation.valid, "tilted table passes validation");
  Expect(result.validation.max_corner_error_m <= 0.050,
         "tilted table corner residual is within 50 mm");
  Expect(Dot(Cross(result.frame.x_axis_raw, result.frame.y_axis_raw),
             result.frame.z_axis_raw) > 0.999,
         "table basis is right handed");
  Args args;
  args.table_length_m = dims.length_m;
  args.table_width_m = dims.width_m;
  args.table_height_m = dims.height_m;
  Expect(RawToTableWorld(pelvis, result.frame, args).x < 0.0,
         "pelvis selects the robot side");
}

void TestCalibrationClusterCount() {
  const TableDimensions dims{2.730738, 1.512451, 0.760000};
  std::vector<Cluster> three(3);
  std::vector<Cluster> five(5);
  Expect(!BuildTableFrame(three, {}, dims).validation.valid,
         "three corners are rejected");
  Expect(!BuildTableFrame(five, {}, dims).validation.valid,
         "five corners are rejected");
}
```

Run the C++ test. Expected: compilation fails because the new structures and `Cross`/3D calibration interfaces do not exist.

- [ ] **Step 2: Add calibration result types and a symmetric 3x3 eigensolver**

Add:

```cpp
struct TableDimensions {
  double length_m = 2.730738;
  double width_m = 1.512451;
  double height_m = 0.760000;
};

struct CalibrationValidation {
  bool valid = false;
  std::string reason;
  double max_corner_error_m = std::numeric_limits<double>::infinity();
  double rms_corner_error_m = std::numeric_limits<double>::infinity();
  std::array<size_t, 4> assignment{};
};

struct CalibrationResult {
  TableFrame frame;
  CalibrationValidation validation;
};

Vec3 Cross(const Vec3& a, const Vec3& b) {
  return {a.y * b.z - a.z * b.y,
          a.z * b.x - a.x * b.z,
          a.x * b.y - a.y * b.x};
}
```

Extend the existing `TableFrame` with:

```cpp
double corner_max_error_m = std::numeric_limits<double>::infinity();
double corner_rms_error_m = std::numeric_limits<double>::infinity();
```

```cpp
bool SymmetricEigenvectors3(
    const double covariance[3][3],
    std::array<double, 3>* eigenvalues,
    std::array<Vec3, 3>* eigenvectors) {
  double a[3][3]{};
  double v[3][3] = {{1.0, 0.0, 0.0},
                    {0.0, 1.0, 0.0},
                    {0.0, 0.0, 1.0}};
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      if (!std::isfinite(covariance[row][col])) return false;
      a[row][col] = covariance[row][col];
    }
  }

  for (int iteration = 0; iteration < 32; ++iteration) {
    int p = 0;
    int q = 1;
    double largest = std::abs(a[0][1]);
    if (std::abs(a[0][2]) > largest) {
      p = 0;
      q = 2;
      largest = std::abs(a[0][2]);
    }
    if (std::abs(a[1][2]) > largest) {
      p = 1;
      q = 2;
      largest = std::abs(a[1][2]);
    }
    if (largest < 1.0e-12) break;

    const double app = a[p][p];
    const double aqq = a[q][q];
    const double apq = a[p][q];
    const double phi = 0.5 * std::atan2(2.0 * apq, aqq - app);
    const double c = std::cos(phi);
    const double s = std::sin(phi);

    for (int k = 0; k < 3; ++k) {
      if (k == p || k == q) continue;
      const double akp = a[k][p];
      const double akq = a[k][q];
      a[k][p] = a[p][k] = c * akp - s * akq;
      a[k][q] = a[q][k] = s * akp + c * akq;
    }
    a[p][p] = c * c * app - 2.0 * s * c * apq + s * s * aqq;
    a[q][q] = s * s * app + 2.0 * s * c * apq + c * c * aqq;
    a[p][q] = a[q][p] = 0.0;

    for (int k = 0; k < 3; ++k) {
      const double vkp = v[k][p];
      const double vkq = v[k][q];
      v[k][p] = c * vkp - s * vkq;
      v[k][q] = s * vkp + c * vkq;
    }
  }

  std::array<size_t, 3> order = {0, 1, 2};
  std::sort(order.begin(), order.end(),
            [&a](size_t lhs, size_t rhs) { return a[lhs][lhs] < a[rhs][rhs]; });
  for (size_t output = 0; output < order.size(); ++output) {
    const size_t source = order[output];
    Vec3 vector{v[0][source], v[1][source], v[2][source]};
    const double norm = Norm(vector);
    if (!std::isfinite(a[source][source]) || !std::isfinite(norm) ||
        norm < 1.0e-12) {
      return false;
    }
    (*eigenvalues)[output] = a[source][source];
    (*eigenvectors)[output] = vector / norm;
  }
  return true;
}
```

The smallest returned eigenvector is the plane normal and the largest is the
long axis. Return false for non-finite inputs/eigenpairs or a vector norm below
`1e-12`.

- [ ] **Step 3: Implement full 3D construction and 4! validation**

`BuildTableFrameFromCorners` must:

1. reject non-finite points or non-positive dimensions;
2. compute the four-point centre and covariance;
3. choose the smallest eigenvector as `z` and the largest as `x`;
4. project `x` onto the plane and normalize it;
5. flip `z` when `Dot(pelvis-centre, z) < 0`;
6. flip `x` when `Dot(pelvis-centre, x) > 0`;
7. set `y = NormalizeVec(Cross(z, x))`, then `x = NormalizeVec(Cross(y, z))`;
8. call `ValidateTableFrame` with a `0.050 m` maximum error.

`ValidateTableFrame` must enumerate all 24 assignments from raw saved corners to these expected table-frame positions:

```cpp
std::array<Vec3, 4> ExpectedWorldCorners(const TableDimensions& dims) {
  return {{{0.0, -0.5 * dims.width_m, dims.height_m},
           {0.0,  0.5 * dims.width_m, dims.height_m},
           {dims.length_m, -0.5 * dims.width_m, dims.height_m},
           {dims.length_m,  0.5 * dims.width_m, dims.height_m}}};
}
```

Use this validation interface and assignment loop:

```cpp
CalibrationValidation ValidateTableFrame(
    TableFrame* frame,
    const TableDimensions& dims,
    const Vec3* pelvis_raw_mm,
    double allowed_max_error_m) {
  CalibrationValidation result;
  const auto finite_vec = [](const Vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) &&
           std::isfinite(value.z);
  };
  if (!finite_vec(frame->center_raw_mm) ||
      !finite_vec(frame->x_axis_raw) || !finite_vec(frame->y_axis_raw) ||
      !finite_vec(frame->z_axis_raw)) {
    result.reason = "non-finite table frame";
    return result;
  }
  if (std::abs(Norm(frame->x_axis_raw) - 1.0) > 1.0e-6 ||
      std::abs(Norm(frame->y_axis_raw) - 1.0) > 1.0e-6 ||
      std::abs(Norm(frame->z_axis_raw) - 1.0) > 1.0e-6 ||
      std::abs(Dot(frame->x_axis_raw, frame->y_axis_raw)) > 1.0e-6 ||
      std::abs(Dot(frame->x_axis_raw, frame->z_axis_raw)) > 1.0e-6 ||
      std::abs(Dot(frame->y_axis_raw, frame->z_axis_raw)) > 1.0e-6 ||
      Dot(Cross(frame->x_axis_raw, frame->y_axis_raw), frame->z_axis_raw) <
          1.0 - 1.0e-6) {
    result.reason = "table basis is not orthonormal and right handed";
    return result;
  }

  Args transform_args;
  transform_args.table_length_m = dims.length_m;
  transform_args.table_width_m = dims.width_m;
  transform_args.table_height_m = dims.height_m;
  if (pelvis_raw_mm != nullptr &&
      RawToTableWorld(*pelvis_raw_mm, *frame, transform_args).x >= 0.0) {
    result.reason = "G1Pelvis is not on the robot side";
    return result;
  }

  const auto expected = ExpectedWorldCorners(dims);
  std::array<size_t, 4> permutation = {0, 1, 2, 3};
  do {
    double max_error = 0.0;
    double sum_squared = 0.0;
    for (size_t raw_index = 0; raw_index < 4; ++raw_index) {
      const Vec3 world = RawToTableWorld(
          frame->corners_raw_mm[raw_index], *frame, transform_args);
      const double error = Distance(world, expected[permutation[raw_index]]);
      max_error = std::max(max_error, error);
      sum_squared += error * error;
    }
    const double rms = std::sqrt(sum_squared / 4.0);
    if (max_error < result.max_corner_error_m ||
        (max_error == result.max_corner_error_m &&
         rms < result.rms_corner_error_m)) {
      result.max_corner_error_m = max_error;
      result.rms_corner_error_m = rms;
      result.assignment = permutation;
    }
  } while (std::next_permutation(permutation.begin(), permutation.end()));

  if (!std::isfinite(result.max_corner_error_m) ||
      result.max_corner_error_m > allowed_max_error_m) {
    result.reason = "corner residual exceeds limit";
    return result;
  }
  std::array<Vec3, 4> ordered{};
  for (size_t raw_index = 0; raw_index < 4; ++raw_index) {
    ordered[result.assignment[raw_index]] = frame->corners_raw_mm[raw_index];
  }
  frame->corners_raw_mm = ordered;
  frame->corner_max_error_m = result.max_corner_error_m;
  frame->corner_rms_error_m = result.rms_corner_error_m;
  result.valid = true;
  return result;
}
```

Implement construction with the same exact types:

```cpp
CalibrationResult BuildTableFrameFromCorners(
    const std::array<Vec3, 4>& corners_raw_mm,
    const Vec3& pelvis_raw_mm,
    const TableDimensions& dims) {
  CalibrationResult result;
  if (!(std::isfinite(dims.length_m) && dims.length_m > 0.0 &&
        std::isfinite(dims.width_m) && dims.width_m > 0.0 &&
        std::isfinite(dims.height_m) && dims.height_m > 0.0)) {
    result.validation.reason = "invalid table dimensions";
    return result;
  }
  for (const Vec3& corner : corners_raw_mm) {
    if (!(std::isfinite(corner.x) && std::isfinite(corner.y) &&
          std::isfinite(corner.z))) {
      result.validation.reason = "non-finite table corner";
      return result;
    }
  }

  TableFrame frame;
  frame.corners_raw_mm = corners_raw_mm;
  frame.center_raw_mm =
      (corners_raw_mm[0] + corners_raw_mm[1] +
       corners_raw_mm[2] + corners_raw_mm[3]) / 4.0;
  double covariance[3][3]{};
  for (const Vec3& corner : corners_raw_mm) {
    const Vec3 d = corner - frame.center_raw_mm;
    const double values[3] = {d.x, d.y, d.z};
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        covariance[row][col] += values[row] * values[col];
      }
    }
  }
  std::array<double, 3> eigenvalues{};
  std::array<Vec3, 3> eigenvectors{};
  if (!SymmetricEigenvectors3(covariance, &eigenvalues, &eigenvectors)) {
    result.validation.reason = "degenerate table covariance";
    return result;
  }

  Vec3 z = eigenvectors[0];
  Vec3 x = NormalizeVec(eigenvectors[2] - Dot(eigenvectors[2], z) * z);
  const Vec3 pelvis_delta = pelvis_raw_mm - frame.center_raw_mm;
  if (Dot(pelvis_delta, z) < 0.0) z = -1.0 * z;
  if (Dot(pelvis_delta, x) > 0.0) x = -1.0 * x;
  Vec3 y = NormalizeVec(Cross(z, x));
  x = NormalizeVec(Cross(y, z));
  frame.x_axis_raw = x;
  frame.y_axis_raw = y;
  frame.z_axis_raw = z;
  frame.valid = true;
  result.frame = frame;
  result.validation = ValidateTableFrame(
      &result.frame, dims, &pelvis_raw_mm, 0.050);
  result.frame.valid = result.validation.valid;
  return result;
}
```

Replace the old combination-search overload with this exact production wrapper:

```cpp
CalibrationResult BuildTableFrame(
    const std::vector<Cluster>& clusters,
    const Vec3& pelvis_raw_mm,
    const TableDimensions& dims) {
  if (clusters.size() != 4) {
    CalibrationResult rejected;
    rejected.validation.reason = "expected exactly four stable corners";
    return rejected;
  }
  std::array<Vec3, 4> corners{};
  for (size_t index = 0; index < corners.size(); ++index) {
    corners[index] = clusters[index].mean;
  }
  return BuildTableFrameFromCorners(corners, pelvis_raw_mm, dims);
}
```

Store corners in the accepted near-left, near-right, far-left, far-right order and store both maximum and RMS errors on `TableFrame`.

- [ ] **Step 4: Require exactly four stable clusters in production**

Replace the current “sort up to 12 and choose the best four” behavior with:

```cpp
if (clusters.size() != 4) {
  std::cerr << "Expected exactly 4 stable unlabeled table corners, got "
            << clusters.size() << "\n";
  return 1;
}
const CalibrationResult calibrated = BuildTableFrame(
    clusters, initial_base_raw.translation_mm,
    {args.table_length_m, args.table_width_m, args.table_height_m});
if (!calibrated.validation.valid) {
  std::cerr << "Table calibration rejected: "
            << calibrated.validation.reason << "\n";
  return 1;
}
table = calibrated.frame;
```

- [ ] **Step 5: Run the C++ tests**

```bash
bash deploy/mocap_bridge/build_cpp_probe.sh
./deploy/mocap_bridge/bin/test_vicon_table_lcm_bridge
```

Expected: `PASS`, including tilted-table, right-handedness, robot-side, and 3/5-cluster rejection checks.

- [ ] **Step 6: Commit 3D table calibration**

```bash
git add -- \
  deploy/mocap_bridge/vicon_table_lcm_bridge.cpp \
  deploy/mocap_bridge/tests/test_vicon_table_lcm_bridge.cpp
git commit -m "feat: calibrate Vicon table frame in 3D"
```

---

### Task 4: Make calibration persistence authoritative and atomic

**Files:**
- Modify: `deploy/mocap_bridge/vicon_table_lcm_bridge.cpp:267-382,895-940`
- Test: `deploy/mocap_bridge/tests/test_vicon_table_lcm_bridge.cpp`

**Interfaces:**
- Consumes: `TableDimensions`, `TableFrame`, and `ValidateTableFrame` from Task 3.
- Produces: `LoadTableCalibration(path, dimensions, table, error)` and `SaveTableCalibrationAtomic(path, dimensions, table, error)` for runtime and candidate generation.

- [ ] **Step 1: Add failing JSON round-trip and rejection tests**

Use a temporary directory and verify saved dimensions override deliberately incorrect runtime defaults:

```cpp
CalibrationResult MakeSyntheticValidCalibration(const TableDimensions& dims) {
  const double length_mm = dims.length_m * 1000.0;
  const double half_width_mm = dims.width_m * 500.0;
  const std::array<Vec3, 4> corners = {
      Vec3{0.0, -half_width_mm, 0.0},
      Vec3{length_mm, half_width_mm, 0.0},
      Vec3{length_mm, -half_width_mm, 0.0},
      Vec3{0.0, half_width_mm, 0.0},
  };
  return BuildTableFrameFromCorners(corners, {-400.0, 0.0, 40.0}, dims);
}

void TestCalibrationRoundTrip() {
  const TableDimensions expected{2.730738, 1.512451, 0.760000};
  const auto built = MakeSyntheticValidCalibration(expected);
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() / "vicon_table_bridge_test.json";
  std::string error;
  Expect(SaveTableCalibrationAtomic(path.string(), expected, built.frame, &error),
         "valid calibration saves atomically");

  TableDimensions loaded{9.0, 8.0, 7.0};
  TableFrame frame;
  Expect(LoadTableCalibration(path.string(), &loaded, &frame, &error),
         "valid calibration reloads");
  ExpectNear(loaded.length_m, expected.length_m, 1.0e-12,
             "saved length overrides runtime value");
  ExpectNear(loaded.width_m, expected.width_m, 1.0e-12,
             "saved width overrides runtime value");
  ExpectNear(loaded.height_m, expected.height_m, 1.0e-12,
             "saved height overrides runtime value");
  std::filesystem::remove(path);
}
```

Add `<filesystem>` and `<fstream>` to the test source and add this fixture
writer plus the three exact malformed documents:

```cpp
void WriteFixture(const std::filesystem::path& path, const std::string& text) {
  std::ofstream output(path, std::ios::trunc);
  output << text;
}

void TestMalformedCalibrationFiles() {
  const auto directory = std::filesystem::temp_directory_path();
  const auto missing_width = directory / "vicon_missing_width.json";
  const auto three_corners = directory / "vicon_three_corners.json";
  const auto left_handed = directory / "vicon_left_handed.json";
  const std::string prefix =
      R"({"center_raw_m":[1.365369,0,0],"x_axis_raw":[1,0,0],)"
      R"("y_axis_raw":[0,1,0],"z_axis_raw":[0,0,1],)"
      R"("height_m_estimate":0.76,"table_length_m":2.730738,)";
  const std::string four =
      R"("corners_raw_m":[[0,-0.7562255,0],[0,0.7562255,0],)"
      R"([2.730738,-0.7562255,0],[2.730738,0.7562255,0]]})";
  WriteFixture(missing_width, prefix + four);
  WriteFixture(
      three_corners,
      prefix + R"("table_width_m":1.512451,)"
      R"("corners_raw_m":[[0,-0.7562255,0],[0,0.7562255,0],)"
      R"([2.730738,-0.7562255,0]]})");
  WriteFixture(
      left_handed,
      R"({"center_raw_m":[1.365369,0,0],"x_axis_raw":[1,0,0],)"
      R"("y_axis_raw":[0,1,0],"z_axis_raw":[0,0,-1],)"
      R"("height_m_estimate":0.76,"table_length_m":2.730738,)"
      R"("table_width_m":1.512451,)" + four);

  for (const auto& path : {missing_width, three_corners, left_handed}) {
    TableDimensions dimensions;
    TableFrame frame;
    std::string error;
    Expect(!LoadTableCalibration(path.string(), &dimensions, &frame, &error),
           "malformed calibration is rejected: " + path.string());
    std::filesystem::remove(path);
  }
}
```

Run the test and expect compile failure until the new persistence signatures
exist.

- [ ] **Step 2: Load all dimensions and validate all saved geometry**

Change loading so `table_length_m`, `table_width_m`, and `height_m_estimate` are all mandatory, finite, and positive. Require exactly four `corners_raw_m`; do not synthesize missing corners. Require all three saved axes and validate the loaded frame using the saved dimensions and the same finite/orthonormal/right-handed/corner-residual rules as Task 3.

Only after the full document validates, assign the temporary parsed dimensions and frame to the output pointers. This prevents partial mutation on failure.

- [ ] **Step 3: Save through a temporary file and atomic rename**

Reuse the existing `WriteVec3Json` helper and implement the complete writer:

```cpp
bool WriteCalibrationJson(
    std::ostream& output,
    const TableDimensions& dims,
    const TableFrame& table) {
  output << std::fixed << std::setprecision(10);
  output << "{\n  \"center_raw_m\": ";
  WriteVec3Json(output, table.center_raw_mm, 0.001);
  output << ",\n  \"x_axis_raw\": ";
  WriteVec3Json(output, table.x_axis_raw, 1.0);
  output << ",\n  \"y_axis_raw\": ";
  WriteVec3Json(output, table.y_axis_raw, 1.0);
  output << ",\n  \"z_axis_raw\": ";
  WriteVec3Json(output, table.z_axis_raw, 1.0);
  output << ",\n  \"height_m_estimate\": " << dims.height_m;
  output << ",\n  \"table_length_m\": " << dims.length_m;
  output << ",\n  \"table_width_m\": " << dims.width_m;
  output << ",\n  \"corner_max_error_m\": " << table.corner_max_error_m;
  output << ",\n  \"corner_rms_error_m\": " << table.corner_rms_error_m;
  output << ",\n  \"corners_raw_m\": [\n";
  for (size_t i = 0; i < table.corners_raw_mm.size(); ++i) {
    output << "    ";
    WriteVec3Json(output, table.corners_raw_mm[i], 0.001);
    output << (i + 1 == table.corners_raw_mm.size() ? "\n" : ",\n");
  }
  output << "  ]\n}\n";
  return static_cast<bool>(output);
}

bool SaveTableCalibrationAtomic(
    const std::string& path,
    const TableDimensions& dims,
    const TableFrame& table,
    std::string* error) {
  const std::string temporary = path + ".tmp";
  TableFrame checked = table;
  const CalibrationValidation validation =
      ValidateTableFrame(&checked, dims, nullptr, 0.050);
  if (!validation.valid) {
    if (error != nullptr) *error = validation.reason;
    return false;
  }
  std::remove(temporary.c_str());
  std::ofstream output(temporary, std::ios::trunc);
  if (!output || !WriteCalibrationJson(output, dims, checked)) {
    output.close();
    std::remove(temporary.c_str());
    if (error != nullptr) *error = "could not write complete calibration";
    return false;
  }
  output.flush();
  const bool complete = static_cast<bool>(output);
  output.close();
  if (!complete || std::rename(temporary.c_str(), path.c_str()) != 0) {
    std::remove(temporary.c_str());
    if (error != nullptr) *error = "could not atomically install calibration";
    return false;
  }
  return true;
}
```

Add `<cstdio>` for `std::remove`/`std::rename`. Use `std::rename` only after the
stream has closed successfully.

- [ ] **Step 4: Integrate candidate load/save semantics**

When `--table-calib` is supplied, load dimensions from that file and use them for transformation, table messages, and validation. When `--save-table-calib` is supplied during new calibration, save only the requested candidate path. Never copy or rename the candidate to `table_frame_latest.json` inside the bridge.

After loading and after the first direct root pose is available, run one more
`ValidateTableFrame(&table, dimensions, &initial_base_raw.translation_mm,
0.050)` call. Exit nonzero if the current standing pelvis does not transform to
`x < 0`; this prevents using a geometrically valid file with reversed x on a
changed Tracer world.

- [ ] **Step 5: Run tests and commit**

```bash
bash deploy/mocap_bridge/build_cpp_probe.sh
./deploy/mocap_bridge/bin/test_vicon_table_lcm_bridge
git add -- \
  deploy/mocap_bridge/vicon_table_lcm_bridge.cpp \
  deploy/mocap_bridge/tests/test_vicon_table_lcm_bridge.cpp
git commit -m "fix: validate Vicon table calibration files"
```

Expected: C++ test prints `PASS`; malformed files are rejected and valid dimensions round-trip.

---

### Task 5: Exclude saved corners and configured ignore spheres from ball candidates

**Files:**
- Modify: `deploy/mocap_bridge/vicon_table_lcm_bridge.cpp:57-70,172-205,690-768,1017-1075`
- Test: `deploy/mocap_bridge/tests/test_vicon_table_lcm_bridge.cpp`

**Interfaces:**
- Consumes: ordered saved `TableFrame.corners_raw_mm` from Tasks 3-4.
- Produces: `IsSavedTableCorner(...)` and filtered `BallCandidates(...)` diagnostics.

- [ ] **Step 1: Write failing 49.9/50.1 mm boundary tests**

Add:

```cpp
void TestBallCandidateExclusions() {
  Args args;
  args.corner_exclusion_radius_mm = 50.0;
  TableFrame table = IdentityTable();
  table.center_raw_mm = {0.0, 0.0, 0.0};
  table.corners_raw_mm = {{{0.0, 0.0, 0.0}, {1000.0, 0.0, 0.0},
                           {0.0, 1000.0, 0.0}, {1000.0, 1000.0, 0.0}}};
  args.raw_ignore_spheres.push_back({{500.0, 500.0, 500.0}, 10.0});
  BallSelectionDiagnostics diagnostics;
  const auto candidates = BallCandidates(
      {{49.9, 0.0, 0.0}, {50.1, 0.0, 0.0},
       {500.0, 500.0, 500.0}, {300.0, 300.0, 900.0}},
      table, args, &diagnostics);
  Expect(candidates.size() == 2, "corner and raw-sphere markers are excluded");
  Expect(diagnostics.corner_ignored_count == 1,
         "49.9 mm corner candidate is excluded");
  Expect(diagnostics.raw_ignored_count == 1,
         "configured raw ignore sphere is effective");
}
```

Run the C++ test. Expected: compilation fails because the radius argument and diagnostic counts do not exist.

- [ ] **Step 2: Apply both filters before world conversion**

Add `double corner_exclusion_radius_mm = 50.0` to `Args` and parse `--corner-exclusion-radius-mm`. Implement:

```cpp
bool IsSavedTableCorner(
    const Vec3& raw_mm, const TableFrame& table, double radius_mm) {
  for (const Vec3& corner : table.corners_raw_mm) {
    if (Distance(raw_mm, corner) <= radius_mm) return true;
  }
  return false;
}
```

In `BallCandidates`, first reject `IgnoredRawMarker`, then reject `IsSavedTableCorner`, and only then call `RawToTableWorld`. Extend diagnostics with `raw_ignored_count`, `corner_ignored_count`, and remaining `candidate_count`; print them in the bridge status line.

- [ ] **Step 3: Run tests and commit**

```bash
bash deploy/mocap_bridge/build_cpp_probe.sh
./deploy/mocap_bridge/bin/test_vicon_table_lcm_bridge
git add -- \
  deploy/mocap_bridge/vicon_table_lcm_bridge.cpp \
  deploy/mocap_bridge/tests/test_vicon_table_lcm_bridge.cpp
git commit -m "fix: exclude Vicon table markers from ball tracking"
```

Expected: C++ test prints `PASS`; exactly the 49.9 mm and explicit-sphere markers are excluded while the 50.1 mm and moving-ball markers remain.

---

### Task 6: Expose validity in the LCM monitor

**Files:**
- Modify: `deploy/mocap_bridge/monitor_vicon_lcm.py:14-31,65-151`
- Create: `deploy/mocap_bridge/tests/test_monitor_vicon_lcm.py`

**Interfaces:**
- Consumes: existing `transformation_t.valid` and `.occluded` fields.
- Produces: `msg_status(msg) -> tuple[int, int]` and truthful console/CSV base validity for live acceptance.

- [ ] **Step 1: Write the failing Python test**

```python
import unittest

from deploy.mocap_bridge.monitor_vicon_lcm import msg_status


class Message:
    valid = 0
    occluded = 1


class MonitorViconLcmTest(unittest.TestCase):
    def test_reads_valid_and_occluded(self):
        self.assertEqual(msg_status(Message()), (0, 1))


if __name__ == "__main__":
    unittest.main()
```

Run:

```bash
conda run -n rb python -m unittest \
  deploy.mocap_bridge.tests.test_monitor_vicon_lcm -v
```

Expected: import fails because `msg_status` does not exist.

- [ ] **Step 2: Implement and use truthful status extraction**

Add:

```python
def msg_status(msg) -> tuple[int, int]:
    return (
        msg_int_field(msg, "valid", 1),
        msg_int_field(msg, "occluded", 0),
    )
```

Store the latest base as `(pos, quat, valid, occluded)`, write the actual base
validity into CSV, and include `valid=0|1 occluded=0|1` in every console line.
Do not infer base validity from the mere presence of a prior message.

Use this output order so the live acceptance expressions are stable:

```python
print(
    f"{elapsed:8.3f}s channel={channel} name={name} "
    f"valid={valid} occluded={occluded} frame={frame_text} "
    f"vicon_time={0.0 if vicon_time_s is None else vicon_time_s:.6f} "
    f"pos=[{pos[0]: .4f}, {pos[1]: .4f}, {pos[2]: .4f}] "
    f"quat=[{quat[0]: .4f}, {quat[1]: .4f}, {quat[2]: .4f}, {quat[3]: .4f}]",
    flush=True,
)
```

- [ ] **Step 3: Run and commit**

```bash
conda run -n rb python -m unittest \
  deploy.mocap_bridge.tests.test_monitor_vicon_lcm -v
git add -- \
  deploy/mocap_bridge/monitor_vicon_lcm.py \
  deploy/mocap_bridge/tests/test_monitor_vicon_lcm.py
git commit -m "test: expose Vicon LCM validity"
```

Expected: one test passes.

---

### Task 7: Run the complete no-hardware regression gate

**Files:**
- Verify only; do not edit unrelated files.

**Interfaces:**
- Consumes: Tasks 1-6.
- Produces: build/test evidence required before connecting to Tracer.

- [ ] **Step 1: Rebuild from source and run the C++ test**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2
rm -f deploy/mocap_bridge/bin/test_vicon_table_lcm_bridge
bash deploy/mocap_bridge/build_cpp_probe.sh
./deploy/mocap_bridge/bin/test_vicon_table_lcm_bridge
```

Expected: three tools plus the test binary build; test prints `PASS`.

- [ ] **Step 2: Run mocap and HITTER Python regressions**

```bash
conda run --no-capture-output -n rb python -m unittest \
  deploy.mocap_bridge.tests.test_chingmu_sdk_client \
  deploy.mocap_bridge.tests.test_chingmu_table_lcm_bridge \
  deploy.mocap_bridge.tests.test_monitor_vicon_lcm \
  deploy.tests.test_real_world_hitter_snapshots \
  deploy.tests.test_hitter_env_lifecycle \
  deploy.tests.test_hitter_realtime \
  deploy.tests.test_hitter_recorded_replay -v
```

Expected: all tests pass, with only previously documented skips allowed.

- [ ] **Step 3: Check scope and artifacts**

```bash
git diff --check
git status --short
git log -6 --oneline
```

Expected: no whitespace errors; no downstream simulator/planner/policy file was changed by this plan; commits contain only their named files.

---

### Task 8: Connect to Tracer, generate the candidate calibration, and validate LCM without robot commands

**Files:**
- Generate: `deploy/mocap_bridge/calibrations/vicon_table_frame_candidate.json`
- Replace after explicit acceptance: `deploy/mocap_bridge/calibrations/table_frame_latest.json`

**Interfaces:**
- Consumes: built binaries, physical Tracer connection, `G1Pelvis`, and the four installed corner markers.
- Produces: accepted Vicon table calibration and live `G1Pelvis`/`ball`/`table` LCM evidence.

- [ ] **Step 1: Prove no competing mocap publisher is running**

```bash
pgrep -af '[c]hingmu_table_lcm_bridge.py|[v]icon_table_lcm_bridge' || true
```

If the check lists an existing bridge from this deployment, stop only those
matching processes and wait for clean exit:

```bash
for process_id in $(pgrep -f '[c]hingmu_table_lcm_bridge.py|[v]icon_table_lcm_bridge'); do
  kill -INT "${process_id}"
done
for attempt in {1..50}; do
  pgrep -f '[c]hingmu_table_lcm_bridge.py|[v]icon_table_lcm_bridge' >/dev/null || break
  sleep 0.1
done
! pgrep -af '[c]hingmu_table_lcm_bridge.py|[v]icon_table_lcm_bridge'
```

Expected: final command succeeds with no output. Do not use a broad Python kill.

- [ ] **Step 2: Activate a separate Vicon network profile**

The current machine reserves `enx0c3d5e614921` for the mocap cable and retains the existing ChingMu profile `有线连接 2`. After the Vicon cable has carrier, create the Vicon profile once:

```bash
nmcli -t -f NAME connection show | rg -q '^vicon-tracer$' || \
  nmcli connection add type ethernet \
    ifname enx0c3d5e614921 \
    con-name vicon-tracer \
    ipv4.method manual \
    ipv4.addresses 192.168.10.2/24 \
    ipv4.never-default yes \
    ipv6.method disabled
nmcli connection up vicon-tracer
ip -br -4 addr show dev enx0c3d5e614921
ping -c 3 -W 1 192.168.10.1
```

Expected: interface has `192.168.10.2/24` and all three pings succeed. If the device still reports `NO-CARRIER`, stop here and fix the physical connection rather than changing code.

- [ ] **Step 3: Probe the Tracer subject, root segment, pose, and source rate**

```bash
./deploy/mocap_bridge/bin/nexus_probe_cpp \
  --host 192.168.10.1:801 \
  --list

./deploy/mocap_bridge/bin/nexus_probe_cpp \
  --host 192.168.10.1:801 \
  --base-subject G1Pelvis \
  --duration 5 \
  --print-hz 10
```

Expected: exact subject `G1Pelvis`, a non-empty root segment, finite non-occluded centroid/quaternion samples, and a finite positive SDK frame rate. Record the actual reported rate; do not force 300 or 360 Hz.

- [ ] **Step 4: Generate a candidate from the four stationary corners**

Keep `G1Pelvis` visible in the normal robot-side stance. Keep exactly the four installed table-corner markers unlabeled and remove the ball and all other unlabeled points. Run:

```bash
./deploy/mocap_bridge/bin/vicon_table_lcm_bridge \
  --host 192.168.10.1:801 \
  --base-subject G1Pelvis \
  --calib-sec 2 \
  --table-length 2.730738 \
  --table-width 1.512451 \
  --table-height 0.760000 \
  --expected-base-edge-distance 0.40 \
  --save-table-calib deploy/mocap_bridge/calibrations/vicon_table_frame_candidate.json \
  --duration 0.1 \
  --no-publish
```

Expected: `Stable unlabeled clusters (4)`, accepted right-handed axes, maximum corner error `<=0.0500 m`, corners at `(0, +/-0.7562255, 0.76)` and `(2.730738, +/-0.7562255, 0.76)`, and `G1Pelvis x < 0`. The x/y transform should be close to Tracer's already aligned origin; z is intentionally normalized so the tabletop is `0.76 m`.

- [ ] **Step 5: Inspect and explicitly promote the accepted candidate**

```bash
sed -n '1,220p' \
  deploy/mocap_bridge/calibrations/vicon_table_frame_candidate.json
```

Pause for explicit human acceptance of the printed candidate and bridge diagnostics. Then preserve the old file once and promote:

```bash
test ! -e deploy/mocap_bridge/calibrations/table_frame_pre_tracer_20260714.json && \
  cp -a deploy/mocap_bridge/calibrations/table_frame_latest.json \
        deploy/mocap_bridge/calibrations/table_frame_pre_tracer_20260714.json
cp deploy/mocap_bridge/calibrations/vicon_table_frame_candidate.json \
   deploy/mocap_bridge/calibrations/table_frame_latest.json
```

Expected: accepted and candidate files are byte-identical; the backup remains unchanged on later runs.

- [ ] **Step 6: Publish Vicon only and inspect LCM**

Terminal A:

```bash
./deploy/mocap_bridge/bin/vicon_table_lcm_bridge \
  --host 192.168.10.1:801 \
  --base-subject G1Pelvis \
  --table-calib deploy/mocap_bridge/calibrations/table_frame_latest.json \
  --publish
```

Terminal B:

```bash
set -o pipefail
conda run --no-capture-output -n rb python \
  deploy/mocap_bridge/monitor_vicon_lcm.py \
  --duration 10 \
  2>&1 | tee /tmp/vicon_tracer_lcm.log
rg 'name=G1Pelvis .*valid=1 .*occluded=0' /tmp/vicon_tracer_lcm.log
rg 'name=table .*valid=1 .*occluded=0' /tmp/vicon_tracer_lcm.log
```

Expected while the robot stands normally:

- `G1Pelvis valid=1 occluded=0`, x/y near `(-0.4, 0)`, finite measured pelvis z, and startup yaw quaternion near identity;
- `table valid=1`, position `[1.365369, 0, 0.76]`;
- corner markers produce no `ball` messages;
- moving a ball produces continuous table-frame `ball` messages;
- manually moving/turning the unpowered robot changes x/y/relative-yaw with the expected signs.

While moving the ball, run another ten-second capture and require a ball row:

```bash
conda run --no-capture-output -n rb python \
  deploy/mocap_bridge/monitor_vicon_lcm.py \
  --duration 10 \
  2>&1 | tee /tmp/vicon_tracer_ball_lcm.log
rg 'name=ball .*valid=1 .*occluded=0' /tmp/vicon_tracer_ball_lcm.log
```

Temporarily occlude the `G1Pelvis` rigid body. Expected: `G1Pelvis valid=0 occluded=1` and no ball message capable of triggering planning for those frames. Stop Terminal A with Ctrl-C after the check.

- [ ] **Step 7: Commit only the accepted physical calibration**

```bash
git add -- deploy/mocap_bridge/calibrations/table_frame_latest.json
git commit -m "config: calibrate Vicon Tracer table frame"
```

Do not commit the candidate or pre-Tracer backup. Do not start `deploy/run.py` in this task; hand the validated real-world launch command back to the user for the separately authorized robot test.
