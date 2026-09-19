#define VICON_TABLE_LCM_BRIDGE_TESTING
#include "../vicon_table_lcm_bridge.cpp"

#include <cmath>
#include <filesystem>
#include <fstream>
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

  const auto blocked_target =
      std::filesystem::temp_directory_path() /
      "vicon_atomic_target_directory";
  std::filesystem::remove_all(blocked_target);
  std::filesystem::create_directory(blocked_target);
  Expect(!SaveTableCalibrationAtomic(
              blocked_target.string(), expected, built.frame, &error),
         "failed atomic rename is reported");
  Expect(!std::filesystem::exists(blocked_target.string() + ".tmp"),
         "failed atomic rename removes temporary file");
  std::filesystem::remove_all(blocked_target);
}

void WriteFixture(const std::filesystem::path& path, const std::string& text) {
  std::ofstream output(path, std::ios::trunc);
  output << text;
}

void TestMalformedCalibrationFiles() {
  const auto directory = std::filesystem::temp_directory_path();
  const auto missing_width = directory / "vicon_missing_width.json";
  const auto three_corners = directory / "vicon_three_corners.json";
  const auto left_handed = directory / "vicon_left_handed.json";
  const auto trailing_garbage = directory / "vicon_trailing_garbage.json";
  const auto redirected_corners = directory / "vicon_redirected_corners.json";
  const auto duplicate_width = directory / "vicon_duplicate_width.json";
  const auto escaped_duplicate_width =
      directory / "vicon_escaped_duplicate_width.json";
  const auto quoted_width = directory / "vicon_quoted_width.json";
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
  WriteFixture(
      trailing_garbage,
      prefix + R"("table_width_m":1.512451,)" + four + " garbage");
  WriteFixture(
      redirected_corners,
      prefix + R"("table_width_m":1.512451,)"
      R"("corners_raw_m":null,"other":[[0,-0.7562255,0],)"
      R"([0,0.7562255,0],[2.730738,-0.7562255,0],)"
      R"([2.730738,0.7562255,0]]})");
  WriteFixture(
      duplicate_width,
      prefix + R"("table_width_m":1.512451,)"
      R"("table_width_m":1.512451,)" + four);
  WriteFixture(
      escaped_duplicate_width,
      prefix + R"("table_width_m":1.512451,)"
      R"("table_\u0077idth_m":1.512451,)" + four);
  WriteFixture(
      quoted_width,
      prefix + R"("table_width_m":"1.512451",)" + four);

  for (const auto& path : {missing_width, three_corners, left_handed,
                           trailing_garbage, redirected_corners,
                           duplicate_width, escaped_duplicate_width,
                           quoted_width}) {
    TableDimensions dimensions{9.0, 8.0, 7.0};
    TableFrame frame;
    frame.center_raw_mm = {123.0, 456.0, 789.0};
    std::string error;
    Expect(!LoadTableCalibration(path.string(), &dimensions, &frame, &error),
           "malformed calibration is rejected: " + path.string());
    ExpectNear(dimensions.length_m, 9.0, 0.0,
               "rejected load leaves dimensions unchanged");
    ExpectNear(frame.center_raw_mm.x, 123.0, 0.0,
               "rejected load leaves frame unchanged");
    std::filesystem::remove(path);
  }
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

  raw.valid = false;
  Expect(!TransformRootPose(raw, raw.quat_xyzw, table, args).valid,
         "invalid direct root pose stays invalid");
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

void TestRuntimeFrameTimeoutDecision() {
  Args args;
  const RuntimeFrameWaitDecision timeout = DecideRuntimeFrameWait(
      false, false, true, 720, args, 360.0);
  Expect(timeout.exit_loop && timeout.exit_nonzero,
         "stream timeout exits the runtime loop with failure");
  Expect(timeout.publish_invalid_base,
         "publishing stream timeout emits one synthetic invalid base");
  Expect(timeout.invalid_base.name == "G1Pelvis",
         "timeout invalid message names the pelvis subject");
  Expect(timeout.invalid_base.valid == 0 &&
             timeout.invalid_base.occluded == 1,
         "timeout invalid message fails closed");
  Expect(timeout.invalid_base.vicon_frame_number == 720,
         "timeout invalid message retains the last source frame");
  ExpectNear(timeout.invalid_base.vicon_time_s, 2.0, 1.0e-12,
             "timeout invalid message retains the source frame rate");
  ExpectNear(timeout.invalid_base.pos_vicon[0], 0.0, 0.0,
             "timeout invalid message has zero position");
  ExpectNear(timeout.invalid_base.pos_vicon[1], 0.0, 0.0,
             "timeout invalid message has zero position");
  ExpectNear(timeout.invalid_base.pos_vicon[2], 0.0, 0.0,
             "timeout invalid message has zero position");
  ExpectNear(timeout.invalid_base.quat_vicon[0], 0.0, 0.0,
             "timeout invalid message has identity quaternion");
  ExpectNear(timeout.invalid_base.quat_vicon[1], 0.0, 0.0,
             "timeout invalid message has identity quaternion");
  ExpectNear(timeout.invalid_base.quat_vicon[2], 0.0, 0.0,
             "timeout invalid message has identity quaternion");
  ExpectNear(timeout.invalid_base.quat_vicon[3], 1.0, 0.0,
             "timeout invalid message has identity quaternion");

  const RuntimeFrameWaitDecision no_publish = DecideRuntimeFrameWait(
      false, false, false, 0, args, 360.0);
  Expect(no_publish.exit_loop && no_publish.exit_nonzero &&
             !no_publish.publish_invalid_base,
         "monitor-only timeout exits nonzero without synthetic publish");
  Expect(no_publish.invalid_base.vicon_frame_number == 0,
         "timeout before any source frame uses frame zero");

  const RuntimeFrameWaitDecision shutdown = DecideRuntimeFrameWait(
      false, true, true, 720, args, 360.0);
  Expect(shutdown.exit_loop && !shutdown.exit_nonzero &&
             !shutdown.publish_invalid_base,
         "requested shutdown exits cleanly without synthetic invalid base");

  const RuntimeFrameWaitDecision received = DecideRuntimeFrameWait(
      true, false, true, 720, args, 360.0);
  Expect(!received.exit_loop && !received.exit_nonzero &&
             !received.publish_invalid_base,
         "received frame continues normally");
}

SegmentPoseRaw ValidCalibrationPose(double x_mm) {
  return DecodeSegmentPose(
      true, false, {x_mm, 20.0, 800.0},
      true, false, {0.0, 0.0, 0.0, 1.0});
}

void TestCalibrationRootTracking() {
  CalibrationRootTracker tracker;
  Expect(AcceptCalibrationRootSample(ValidCalibrationPose(-410.0), &tracker),
         "first valid calibration root is accepted");
  Expect(AcceptCalibrationRootSample(ValidCalibrationPose(-405.0), &tracker),
         "later valid calibration root is accepted");
  const auto final = FinalCalibrationRootPose(tracker);
  Expect(final.has_value(), "valid root sequence has a final pose");
  ExpectNear(final->translation_mm.x, -405.0, 0.0,
             "calibration selects the latest valid root translation");

  CalibrationRootTracker invalid_intermediate;
  Expect(AcceptCalibrationRootSample(
             ValidCalibrationPose(-410.0), &invalid_intermediate),
         "valid root before an invalid intermediate is accepted");
  SegmentPoseRaw invalid = ValidCalibrationPose(-407.0);
  invalid.valid = false;
  invalid.occluded = true;
  Expect(!AcceptCalibrationRootSample(invalid, &invalid_intermediate),
         "invalid intermediate root rejects calibration immediately");
  Expect(!FinalCalibrationRootPose(invalid_intermediate).has_value(),
         "rejected calibration has no final root pose");

  CalibrationRootTracker invalid_final;
  Expect(AcceptCalibrationRootSample(
             ValidCalibrationPose(-410.0), &invalid_final),
         "valid root before invalid final is accepted");
  Expect(!AcceptCalibrationRootSample(invalid, &invalid_final),
         "invalid final root rejects calibration");
  Expect(!FinalCalibrationRootPose(invalid_final).has_value(),
         "invalid final root cannot be selected");

  CalibrationRootTracker empty;
  Expect(!FinalCalibrationRootPose(empty).has_value(),
         "calibration requires at least one valid root sample");

  CalibrationRootTracker stalled;
  Expect(AcceptCalibrationRootSample(
             ValidCalibrationPose(-410.0), &stalled),
         "valid root before calibration stream stall is accepted");
  Expect(DecideCalibrationFrame(false, false, &stalled) ==
             CalibrationFrameDecision::RejectTimeout,
         "missing calibration frame is a rejecting timeout");
  Expect(!FinalCalibrationRootPose(stalled).has_value(),
         "missing calibration frame invalidates the prior root sample");

  CalibrationRootTracker shutdown;
  Expect(AcceptCalibrationRootSample(
             ValidCalibrationPose(-410.0), &shutdown),
         "valid root before requested shutdown is accepted");
  Expect(DecideCalibrationFrame(false, true, &shutdown) ==
             CalibrationFrameDecision::CleanShutdown,
         "requested shutdown is distinguished from a stream timeout");
  Expect(FinalCalibrationRootPose(shutdown).has_value(),
         "requested shutdown does not mislabel the sampled root as invalid");
}

Cluster MakeCalibrationCluster(
    const Vec3& center, int accepted_frames,
    const std::vector<Vec3>& offsets,
    int first_frame = 1) {
  std::vector<Cluster> clusters;
  for (int index = 0; index < accepted_frames; ++index) {
    const Vec3 offset = offsets[static_cast<size_t>(index) % offsets.size()];
    AddClusterSample(
        &clusters, center + offset, 80.0, first_frame + index);
  }
  return clusters.front();
}

void TestStableCalibrationClusterFiltering() {
  const int frames = 10;
  const std::vector<Vec3> stationary_offsets = {
      {-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
  std::vector<Cluster> stationary = {
      MakeCalibrationCluster({0.0, 0.0, 0.0}, frames, stationary_offsets),
      MakeCalibrationCluster({2730.738, 0.0, 0.0}, frames, stationary_offsets),
      MakeCalibrationCluster({0.0, 1512.451, 0.0}, frames, stationary_offsets),
      MakeCalibrationCluster(
          {2730.738, 1512.451, 0.0}, frames, stationary_offsets),
  };
  const auto retained = StableCalibrationClusters(stationary, frames);
  Expect(retained.size() == 4,
         "four stationary full-coverage clusters are retained");
  for (const Cluster& cluster : retained) {
    const ClusterStability stats = EvaluateClusterStability(cluster, frames);
    Expect(stats.stable && stats.coverage == 1.0,
           "stationary cluster reports full stable coverage");
    Expect(stats.rms_deviation_mm <= 5.0 &&
               stats.max_deviation_mm <= 15.0,
           "stationary cluster is within deviation limits");
  }

  Cluster low_coverage = MakeCalibrationCluster(
      {500.0, 500.0, 0.0}, 7, stationary_offsets);
  const ClusterStability low_coverage_stats =
      EvaluateClusterStability(low_coverage, frames);
  Expect(!low_coverage_stats.stable,
         "cluster below 80 percent coverage is rejected");
  ExpectNear(low_coverage_stats.coverage, 0.7, 1.0e-12,
             "rejected cluster still reports its measured coverage");
  Expect(std::isfinite(low_coverage_stats.rms_deviation_mm) &&
             std::isfinite(low_coverage_stats.max_deviation_mm),
         "rejected low-coverage cluster still reports deviation diagnostics");

  Cluster noisy = MakeCalibrationCluster(
      {500.0, 500.0, 0.0}, frames,
      {{-8.0, 0.0, 0.0}, {8.0, 0.0, 0.0}});
  Expect(!EvaluateClusterStability(noisy, frames).stable,
         "cluster above 5 mm RMS is rejected");

  Cluster moving = MakeCalibrationCluster(
      {500.0, 500.0, 0.0}, frames,
      {{-20.0, 0.0, 0.0}, {-15.0, 0.0, 0.0}, {-10.0, 0.0, 0.0},
       {-5.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {5.0, 0.0, 0.0},
       {10.0, 0.0, 0.0}, {15.0, 0.0, 0.0}, {20.0, 0.0, 0.0},
       {25.0, 0.0, 0.0}});
  Expect(!EvaluateClusterStability(moving, frames).stable,
         "moving cluster above maximum deviation is rejected");

  Cluster non_finite = stationary.front();
  non_finite.samples.push_back(
      {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0});
  ++non_finite.count;
  Expect(!EvaluateClusterStability(non_finite, frames).stable,
         "non-finite cluster samples are rejected");
}

void TestSaveCalibrationHelpUsesCandidatePath() {
  const std::string usage = UsageText("vicon_table_lcm_bridge");
  Expect(usage.find(
             "[--save-table-calib vicon_table_frame_candidate.json]") !=
             std::string::npos,
         "help advertises an explicit candidate calibration output");
  Expect(usage.find("[--save-table-calib table_frame_latest.json]") ==
             std::string::npos,
         "help does not advertise implicit latest-file overwrite");
}

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
  const CalibrationResult result =
      BuildTableFrameFromCorners(shuffled, pelvis, dims);
  Expect(result.frame.valid, "tilted table calibration succeeds");
  Expect(result.validation.valid, "tilted table passes validation");
  Expect(result.validation.max_corner_error_m <= 0.050,
         "tilted table corner residual is within 50 mm");
  Expect(Dot(Cross(result.frame.x_axis_raw, result.frame.y_axis_raw),
             result.frame.z_axis_raw) > 0.999,
         "table basis is right handed");
  Expect(Dot(result.frame.z_axis_raw, z) > 0.999,
         "table normal points toward the known upward side");
  Args args;
  args.table_length_m = dims.length_m;
  args.table_width_m = dims.width_m;
  args.table_height_m = dims.height_m;
  Expect(RawToTableWorld(pelvis, result.frame, args).x < 0.0,
         "pelvis selects the robot side");
  const auto expected = ExpectedWorldCorners(dims);
  for (size_t index = 0; index < expected.size(); ++index) {
    const Vec3 actual =
        RawToTableWorld(result.frame.corners_raw_mm[index], result.frame, args);
    ExpectNear(actual.x, expected[index].x, 1.0e-9,
               "ordered corner x matches canonical index");
    ExpectNear(actual.y, expected[index].y, 1.0e-9,
               "ordered corner y matches canonical index");
    ExpectNear(actual.z, expected[index].z, 1.0e-9,
               "ordered corner z matches canonical index");
  }
  ExpectNear(result.frame.corner_max_error_m,
             result.validation.max_corner_error_m, 0.0,
             "frame stores validated maximum residual");
  ExpectNear(result.frame.corner_rms_error_m,
             result.validation.rms_corner_error_m, 0.0,
             "frame stores validated RMS residual");
}

void TestTableUpAxisDoesNotFollowPelvis() {
  const TableDimensions dims{2.730738, 1.512451, 0.760000};
  const double hl = dims.length_m * 500.0;
  const double hw = dims.width_m * 500.0;
  const std::array<Vec3, 4> corners = {{{-hl, -hw, 0.0},
                                        {-hl, hw, 0.0},
                                        {hl, -hw, 0.0},
                                        {hl, hw, 0.0}}};
  const Vec3 pelvis{-hl - 400.0, 0.0, -100.0};

  const CalibrationResult result =
      BuildTableFrameFromCorners(corners, pelvis, dims);

  Expect(result.validation.valid,
         "pelvis below the tabletop does not invalidate calibration");
  Expect(Dot(result.frame.z_axis_raw, {0.0, 0.0, 1.0}) > 0.999,
         "table positive z follows Tracker native positive z");
  Args args;
  args.table_length_m = dims.length_m;
  args.table_width_m = dims.width_m;
  args.table_height_m = dims.height_m;
  ExpectNear(RawToTableWorld(pelvis, result.frame, args).z,
             dims.height_m - 0.100, 1.0e-12,
             "pelvis below the tabletop remains below table height");
}

void TestCalibrationClusterCount() {
  const TableDimensions dims{2.730738, 1.512451, 0.760000};
  const Args defaults;
  ExpectNear(defaults.table_length_m, dims.length_m, 0.0,
             "runtime default table length is authoritative");
  ExpectNear(defaults.table_width_m, dims.width_m, 0.0,
             "runtime default table width is authoritative");
  ExpectNear(defaults.table_height_m, dims.height_m, 0.0,
             "runtime default table height is authoritative");
  std::vector<Cluster> three(3);
  std::vector<Cluster> five(5);
  Expect(!BuildTableFrame(three, {}, dims).validation.valid,
         "three corners are rejected");
  Expect(!BuildTableFrame(five, {}, dims).validation.valid,
         "five corners are rejected");

  const double hl = dims.length_m * 500.0;
  const double hw = dims.width_m * 500.0;
  std::vector<Cluster> four(4);
  four[0].mean = {hl, hw, 0.0};
  four[1].mean = {-hl, -hw, 0.0};
  four[2].mean = {hl, -hw, 0.0};
  four[3].mean = {-hl, hw, 0.0};
  Expect(BuildTableFrame(four, {-hl - 400.0, 0.0, 40.0}, dims)
             .validation.valid,
         "four valid clusters calibrate successfully");
}

void TestCornerResidualLimit() {
  const TableDimensions dims{2.730738, 1.512451, 0.760000};
  const double hl = dims.length_m * 500.0;
  const double hw = dims.width_m * 500.0;
  const Vec3 pelvis{-hl - 400.0, 0.0, 40.0};
  const auto make_frame = [hl, hw](double corner_z_offset_mm) {
    TableFrame frame = IdentityTable();
    frame.center_raw_mm = {};
    frame.corners_raw_mm = {{{-hl, -hw, corner_z_offset_mm},
                             {-hl, hw, 0.0},
                             {hl, -hw, 0.0},
                             {hl, hw, 0.0}}};
    return frame;
  };

  TableFrame inside = make_frame(49.9);
  const CalibrationValidation accepted =
      ValidateTableFrame(&inside, dims, &pelvis, 0.050);
  Expect(accepted.valid, "49.9 mm corner residual is accepted");

  TableFrame outside = make_frame(50.1);
  const CalibrationValidation rejected =
      ValidateTableFrame(&outside, dims, &pelvis, 0.050);
  Expect(!rejected.valid, "50.1 mm corner residual is rejected");

  TableFrame exact = make_frame(50.0);
  Expect(ValidateTableFrame(&exact, dims, &pelvis, 0.050).valid,
         "exactly 50.0 mm corner residual is accepted");
}

void TestCalibrationValidationFailsClosed() {
  const TableDimensions dims{2.730738, 1.512451, 0.760000};
  const double hl = dims.length_m * 500.0;
  const double hw = dims.width_m * 500.0;
  const std::array<Vec3, 4> corners = {{{-hl, -hw, 0.0},
                                        {-hl, hw, 0.0},
                                        {hl, -hw, 0.0},
                                        {hl, hw, 0.0}}};
  const Vec3 pelvis{-hl - 400.0, 0.0, 40.0};

  const Vec3 non_finite_pelvis{
      std::numeric_limits<double>::quiet_NaN(), 0.0, 40.0};
  Expect(!BuildTableFrameFromCorners(corners, non_finite_pelvis, dims)
              .validation.valid,
         "non-finite pelvis is rejected");

  TableFrame non_finite_corner = IdentityTable();
  non_finite_corner.corners_raw_mm = corners;
  non_finite_corner.corners_raw_mm[0].x =
      std::numeric_limits<double>::quiet_NaN();
  Expect(!ValidateTableFrame(
              &non_finite_corner, dims, &pelvis, 0.050).valid,
         "non-finite saved corner is rejected");

  TableFrame inverted = IdentityTable();
  inverted.y_axis_raw = {0.0, -1.0, 0.0};
  inverted.z_axis_raw = {0.0, 0.0, -1.0};
  inverted.corners_raw_mm = corners;
  Expect(!ValidateTableFrame(&inverted, dims, &pelvis, 0.050).valid,
         "table positive z opposite Tracker positive z is rejected");

  TableFrame non_orthonormal = IdentityTable();
  non_orthonormal.y_axis_raw = {1.0, 0.0, 0.0};
  non_orthonormal.corners_raw_mm = corners;
  Expect(!ValidateTableFrame(
              &non_orthonormal, dims, &pelvis, 0.050).valid,
         "non-orthonormal basis is rejected");

  TableFrame wrong_side = IdentityTable();
  wrong_side.corners_raw_mm = corners;
  const Vec3 far_side{hl + 400.0, 0.0, 40.0};
  Expect(!ValidateTableFrame(
              &wrong_side, dims, &far_side, 0.050).valid,
         "pelvis on the far side is rejected");
}

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
      {{49.9, 0.0, 0.0}, {50.0, 0.0, 0.0}, {50.1, 0.0, 0.0},
       {500.0, 500.0, 500.0}, {300.0, 300.0, 900.0}},
      table, args, &diagnostics);
  Expect(candidates.size() == 2, "corner and raw-sphere markers are excluded");
  Expect(diagnostics.corner_ignored_count == 2,
         "49.9 mm and exactly 50.0 mm corner candidates are excluded");
  Expect(diagnostics.raw_ignored_count == 1,
         "configured raw ignore sphere is effective");
  ExpectNear(candidates[0].raw.x, 50.1, 0.0,
             "50.1 mm corner-adjacent marker is retained");
  ExpectNear(candidates[0].world.x,
             args.table_length_m * 0.5 + 0.0501, 1.0e-12,
             "retained 50.1 mm marker has the expected world position");
  ExpectNear(candidates[1].raw.x, 300.0, 0.0,
             "moving-ball raw marker identity is retained");
  ExpectNear(candidates[1].world.z,
             args.table_height_m + 0.900, 1.0e-12,
             "moving-ball marker is converted after filtering");
}

bool ParseCornerExclusionRadius(const std::string& value, Args* args) {
  std::array<std::string, 3> storage = {
      "vicon_table_lcm_bridge", "--corner-exclusion-radius-mm", value};
  std::array<char*, 3> argv = {
      storage[0].data(), storage[1].data(), storage[2].data()};
  return ParseArgs(static_cast<int>(argv.size()), argv.data(), args);
}

void TestCornerExclusionRadiusValidation() {
  for (const std::string& invalid : {"-0.1", "nan", "inf", "-inf"}) {
    Args args;
    Expect(!ParseCornerExclusionRadius(invalid, &args),
           "negative and non-finite corner exclusion radius is rejected: " +
               invalid);
  }

  Args zero;
  Expect(ParseCornerExclusionRadius("0", &zero),
         "zero corner exclusion radius is accepted");
  ExpectNear(zero.corner_exclusion_radius_mm, 0.0, 0.0,
             "zero corner exclusion radius is preserved");

  Args positive;
  Expect(ParseCornerExclusionRadius("50.1", &positive),
         "finite positive corner exclusion radius is accepted");
  ExpectNear(positive.corner_exclusion_radius_mm, 50.1, 0.0,
             "finite positive corner exclusion radius is preserved");
}

}  // namespace

int main() {
  TestCalibrationRoundTrip();
  TestMalformedCalibrationFiles();
  TestSegmentPoseValidation();
  TestRelativeYaw();
  TestFrameRateSelection();
  TestDirectRootTranslation();
  TestSourceFrameTiming();
  TestRuntimeFrameTimeoutDecision();
  TestCalibrationRootTracking();
  TestStableCalibrationClusterFiltering();
  TestSaveCalibrationHelpUsesCandidatePath();
  TestTiltedTableCalibration();
  TestTableUpAxisDoesNotFollowPelvis();
  TestCalibrationClusterCount();
  TestCornerResidualLimit();
  TestCalibrationValidationFailsClosed();
  TestBallCandidateExclusions();
  TestCornerExclusionRadiusValidation();
  if (failures == 0) std::cout << "test_vicon_table_lcm_bridge: PASS\n";
  return failures == 0 ? 0 : 1;
}
