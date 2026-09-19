#define main hitter_vicon_table_lcm_bridge_main
#include "../../deploy/mocap_bridge/vicon_table_lcm_bridge.cpp"
#undef main

#include <cmath>
#include <iostream>
#include <vector>

namespace {

Vec3 RawFromWorldForTest(const Vec3& world, const TableFrame& table, const Args& args) {
  return table.center_raw_mm +
         ((world.x - args.table_length_m * 0.5) * 1000.0) * table.x_axis_raw +
         (world.y * 1000.0) * table.y_axis_raw +
         ((world.z - args.table_height_m) * 1000.0) * table.z_axis_raw;
}

TableFrame MakeTableForTest() {
  TableFrame table;
  table.valid = true;
  table.center_raw_mm = {0.0, 0.0, 0.0};
  table.x_axis_raw = {1.0, 0.0, 0.0};
  table.y_axis_raw = {0.0, 1.0, 0.0};
  table.z_axis_raw = {0.0, 0.0, 1.0};
  table.corners_raw_mm = {
      Vec3{-3000.0, -2000.0, -1000.0},
      Vec3{-3000.0, 2000.0, -1000.0},
      Vec3{3000.0, -2000.0, -1000.0},
      Vec3{3000.0, 2000.0, -1000.0},
  };
  return table;
}

}  // namespace

int main() {
  Args args;

  const Vec3 ghost_raw_mm{60100.0, 80900.0, 2104.0};
  if (!IgnoredRawMarker(ghost_raw_mm, args)) {
    std::cerr << "expected known raw ghost marker to be ignored\n";
    return 1;
  }

  const Vec3 table_near_raw_mm{40.0, 2868.0, 760.0};
  if (IgnoredRawMarker(table_near_raw_mm, args)) {
    std::cerr << "did not expect table-near raw marker to be ignored\n";
    return 2;
  }

  RawIgnoreSphere custom;
  if (!ParseRawIgnoreSphereM("1.0,2.0,3.0,0.25", &custom)) {
    std::cerr << "expected raw ignore sphere parser to accept comma-separated meters\n";
    return 3;
  }
  if (std::abs(custom.center_mm.x - 1000.0) > 1.0e-9 ||
      std::abs(custom.center_mm.y - 2000.0) > 1.0e-9 ||
      std::abs(custom.center_mm.z - 3000.0) > 1.0e-9 ||
      std::abs(custom.radius_mm - 250.0) > 1.0e-9) {
    std::cerr << "raw ignore sphere parser did not convert meters to millimeters\n";
    return 4;
  }

  const TableFrame table = MakeTableForTest();
  BallTrackState track;
  Vec3 ball_raw;
  Vec3 ball_world;
  BallSelectionDiagnostics diagnostics;

  if (SelectBall(std::vector<Vec3>{}, table, args, &track, 50, &ball_raw, &ball_world, &diagnostics)) {
    std::cerr << "empty unlabeled set should not be selected as ball\n";
    return 5;
  }
  if (diagnostics.reason != BallRejectReason::NoUnlabeled) {
    std::cerr << "expected no_unlabeled reject reason\n";
    return 6;
  }

  const Vec3 bootstrap_raw = RawFromWorldForTest(Vec3{1.20, 0.05, 1.05}, table, args);
  diagnostics = BallSelectionDiagnostics{};
  if (SelectBall(std::vector<Vec3>{bootstrap_raw}, table, args, &track, 60, &ball_raw, &ball_world, &diagnostics)) {
    std::cerr << "first moving candidate should wait for bootstrap history\n";
    return 7;
  }
  if (diagnostics.reason != BallRejectReason::BootstrapWait || diagnostics.candidate_count != 1) {
    std::cerr << "expected bootstrap_wait reject reason with one candidate\n";
    return 8;
  }
  track.bootstrap_observations.clear();

  std::vector<Vec3> static_residual;
  for (int frame = 100; frame < 140; ++frame) {
    const double jitter = (frame % 2 == 0) ? 1.0e-5 : -1.0e-5;
    const Vec3 raw = RawFromWorldForTest(Vec3{-0.34 + jitter, 0.08, 0.68}, table, args);
    static_residual = {raw};
    if (SelectBall(static_residual, table, args, &track, frame, &ball_raw, &ball_world)) {
      std::cerr << "static residual unlabeled point should not be selected as ball\n";
      return 9;
    }
    NoteBallMiss(&track, args);
  }

  for (int frame = 150; frame < 190; ++frame) {
    const double jitter = (frame % 2 == 0) ? 0.003 : -0.003;
    const Vec3 raw = RawFromWorldForTest(Vec3{-0.34 + jitter, 0.08, 0.68}, table, args);
    static_residual = {raw};
    if (SelectBall(static_residual, table, args, &track, frame, &ball_raw, &ball_world)) {
      std::cerr << "jittering static residual unlabeled point should not be selected as ball\n";
      return 10;
    }
    NoteBallMiss(&track, args);
  }

  bool selected_moving_ball = false;
  for (int frame = 200; frame < 210; ++frame) {
    const double x = 1.20 - 0.02 * static_cast<double>(frame - 200);
    const Vec3 raw = RawFromWorldForTest(Vec3{x, 0.05, 1.05}, table, args);
    if (SelectBall(std::vector<Vec3>{raw}, table, args, &track, frame, &ball_raw, &ball_world)) {
      selected_moving_ball = true;
      UpdateBallTrack(&track, ball_world, frame, args);
      break;
    }
    NoteBallMiss(&track, args);
  }
  if (!selected_moving_ball) {
    std::cerr << "moving incoming unlabeled point should bootstrap ball track\n";
    return 11;
  }

  return 0;
}
