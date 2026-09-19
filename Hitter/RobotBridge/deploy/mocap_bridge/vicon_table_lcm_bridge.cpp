#include "DataStreamClient.h"

#include <lcm/lcm-cpp.hpp>
#include "unitree_sdk2/lcm_types/transformation_t.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <numeric>
#include <regex>
#include <string>
#include <thread>
#include <vector>

using namespace ViconDataStreamSDK::CPP;

namespace {

volatile std::sig_atomic_t g_stop = 0;
void HandleSignal(int) { g_stop = 1; }

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct RawIgnoreSphere {
  Vec3 center_mm;
  double radius_mm = 0.0;
};

struct Quat {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
};

struct Args {
  std::string host = "192.168.10.1:801";
  std::string base_subject = "G1Pelvis";
  std::string lcm_url = "udpm://239.255.76.67:7667?ttl=255";
  std::string table_calib_path;
  std::string save_table_calib_path;
  double duration_s = 0.0;
  double print_hz = 10.0;
  double calib_s = 2.0;
  double table_length_m = 2.74;
  double table_width_m = 1.525;
  double table_height_m = 0.76;
  double expected_base_edge_distance_m = 0.40;
  double base_anchor_x_m = std::numeric_limits<double>::quiet_NaN();
  double base_anchor_y_m = std::numeric_limits<double>::quiet_NaN();
  double base_anchor_z_m = 0.793;
  double vicon_frame_rate_hz = 300.0;
  double ball_max_speed_mps = 12.0;
  double ball_min_bootstrap_speed_mps = 0.30;
  double ball_min_bootstrap_displacement_m = 0.01;
  double ball_min_gate_m = 0.06;
  double ball_gate_margin_m = 0.03;
  int ball_track_reset_frames = 15;
  std::vector<RawIgnoreSphere> raw_ignore_spheres = {
      {Vec3{60100.0, 80900.0, 2104.0}, 500.0},
  };
  bool calibrate_base_anchor = false;
  bool publish = false;
};

struct Cluster {
  Vec3 mean;
  Vec3 sum;
  Vec3 sum_sq;
  int count = 0;
};

struct TableFrame {
  bool valid = false;
  Vec3 center_raw_mm;
  Vec3 x_axis_raw;
  Vec3 y_axis_raw;
  Vec3 z_axis_raw{0.0, 0.0, 1.0};
  std::array<Vec3, 4> corners_raw_mm;
  double rectangle_score = 0.0;
};

struct BallCandidate {
  Vec3 raw;
  Vec3 world;
};

enum class BallRejectReason {
  None = 0,
  BaseInvalid,
  NoUnlabeled,
  NoCandidates,
  BootstrapWait,
  TrackingGateFail,
  Count,
};

constexpr size_t kBallRejectReasonCount = static_cast<size_t>(BallRejectReason::Count);

const char* BallRejectReasonName(BallRejectReason reason) {
  switch (reason) {
    case BallRejectReason::None:
      return "none";
    case BallRejectReason::BaseInvalid:
      return "base_invalid";
    case BallRejectReason::NoUnlabeled:
      return "no_unlabeled";
    case BallRejectReason::NoCandidates:
      return "no_candidates";
    case BallRejectReason::BootstrapWait:
      return "bootstrap_wait";
    case BallRejectReason::TrackingGateFail:
      return "tracking_gate_fail";
    case BallRejectReason::Count:
      break;
  }
  return "unknown";
}

size_t BallRejectReasonIndex(BallRejectReason reason) {
  const size_t index = static_cast<size_t>(reason);
  if (index >= kBallRejectReasonCount) return static_cast<size_t>(BallRejectReason::NoCandidates);
  return index;
}

struct BallSelectionDiagnostics {
  BallRejectReason reason = BallRejectReason::None;
  size_t candidate_count = 0;
  size_t ignored_raw_count = 0;
  size_t near_corner_count = 0;
  size_t implausible_world_count = 0;
  size_t bootstrap_observation_count = 0;
  double tracking_dt_s = 0.0;
  double tracking_gate_m = 0.0;
  double tracking_best_pred_dist_m = std::numeric_limits<double>::infinity();
  double tracking_best_direct_speed_mps = 0.0;
};

struct BallBootstrapObservation {
  Vec3 world;
  int64_t frame_number = 0;
};

struct BallTrackState {
  bool have_position = false;
  bool have_velocity = false;
  Vec3 position;
  Vec3 velocity;
  int64_t frame_number = 0;
  int missed_frames = 0;
  std::vector<BallBootstrapObservation> bootstrap_observations;
};

Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 operator*(double s, const Vec3& a) { return {s * a.x, s * a.y, s * a.z}; }
Vec3 operator/(const Vec3& a, double s) { return {a.x / s, a.y / s, a.z / s}; }

double Dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
double Norm(const Vec3& a) { return std::sqrt(Dot(a, a)); }
double Distance(const Vec3& a, const Vec3& b) { return Norm(a - b); }
double Clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

int64_t NowUnixMicros() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::microseconds>(now).count();
}

double FrameDeltaSeconds(int64_t current_frame, int64_t previous_frame, double frame_rate_hz) {
  if (frame_rate_hz <= 0.0) return 0.0;
  const int64_t frame_delta = current_frame - previous_frame;
  if (frame_delta > 0) return static_cast<double>(frame_delta) / frame_rate_hz;
  return 1.0 / frame_rate_hz;
}

Vec3 Mean(const std::vector<Vec3>& pts) {
  Vec3 sum;
  for (const auto& p : pts) sum = sum + p;
  return pts.empty() ? Vec3{} : sum / static_cast<double>(pts.size());
}

Vec3 NormalizeVec(const Vec3& p) {
  const double n = Norm(p);
  if (n < 1.0e-12) return {};
  return p / n;
}

bool Ok(Result::Enum result) { return result == Result::Success; }

bool ParseRawIgnoreSphereM(const std::string& spec, RawIgnoreSphere* sphere) {
  const std::regex pattern(
      "^\\s*([-+0-9.eE]+)\\s*,\\s*([-+0-9.eE]+)\\s*,\\s*([-+0-9.eE]+)\\s*,\\s*([-+0-9.eE]+)\\s*$");
  std::smatch match;
  if (!std::regex_match(spec, match, pattern)) return false;
  sphere->center_mm = {
      std::stod(match[1].str()) * 1000.0,
      std::stod(match[2].str()) * 1000.0,
      std::stod(match[3].str()) * 1000.0,
  };
  sphere->radius_mm = std::stod(match[4].str()) * 1000.0;
  return std::isfinite(sphere->center_mm.x) &&
         std::isfinite(sphere->center_mm.y) &&
         std::isfinite(sphere->center_mm.z) &&
         std::isfinite(sphere->radius_mm) &&
         sphere->radius_mm >= 0.0;
}

bool IgnoredRawMarker(const Vec3& raw_mm, const Args& args) {
  for (const auto& sphere : args.raw_ignore_spheres) {
    if (Distance(raw_mm, sphere.center_mm) <= sphere.radius_mm) return true;
  }
  return false;
}

size_t CountIgnoredRawMarkers(const std::vector<Vec3>& unlabeled_raw, const Args& args) {
  size_t count = 0;
  for (const auto& raw : unlabeled_raw) {
    if (IgnoredRawMarker(raw, args)) ++count;
  }
  return count;
}

bool WaitFrame(Client* client, double timeout_s) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < deadline && !g_stop) {
    if (Ok(client->GetFrame().Result)) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}

std::map<std::string, Vec3> VisibleSubjectMarkers(Client* client, const std::string& subject) {
  std::map<std::string, Vec3> out;
  const auto marker_count = client->GetMarkerCount(subject).MarkerCount;
  for (unsigned int i = 0; i < marker_count; ++i) {
    const std::string name = client->GetMarkerName(subject, i).MarkerName;
    const auto marker = client->GetMarkerGlobalTranslation(subject, name);
    if (Ok(marker.Result) && !marker.Occluded) {
      out[name] = {marker.Translation[0], marker.Translation[1], marker.Translation[2]};
    }
  }
  return out;
}

std::vector<Vec3> UnlabeledMarkers(Client* client) {
  std::vector<Vec3> markers;
  const unsigned int count = client->GetUnlabeledMarkerCount().MarkerCount;
  markers.reserve(count);
  for (unsigned int i = 0; i < count; ++i) {
    const auto marker = client->GetUnlabeledMarkerGlobalTranslation(i);
    Vec3 p{marker.Translation[0], marker.Translation[1], marker.Translation[2]};
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) markers.push_back(p);
  }
  return markers;
}

bool ReadFile(const std::string& path, std::string* out) {
  std::ifstream input(path);
  if (!input) return false;
  out->assign(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
  return true;
}

bool ExtractNumber(const std::string& text, const std::string& key, double* out) {
  const std::regex pattern("\"" + key + "\"\\s*:\\s*([-+0-9.eE]+)");
  std::smatch match;
  if (!std::regex_search(text, match, pattern)) return false;
  *out = std::stod(match[1].str());
  return true;
}

bool ExtractVec3(const std::string& text, const std::string& key, double scale, Vec3* out) {
  const std::regex pattern(
      "\"" + key + "\"\\s*:\\s*\\[\\s*([-+0-9.eE]+)\\s*,\\s*([-+0-9.eE]+)\\s*,\\s*([-+0-9.eE]+)\\s*\\]");
  std::smatch match;
  if (!std::regex_search(text, match, pattern)) return false;
  *out = {
      std::stod(match[1].str()) * scale,
      std::stod(match[2].str()) * scale,
      std::stod(match[3].str()) * scale,
  };
  return true;
}

bool ExtractVec3Array(const std::string& text, const std::string& key, double scale, std::vector<Vec3>* out) {
  const std::string quoted_key = "\"" + key + "\"";
  const size_t key_pos = text.find(quoted_key);
  if (key_pos == std::string::npos) return false;
  const size_t start = text.find('[', key_pos + quoted_key.size());
  if (start == std::string::npos) return false;

  int depth = 0;
  size_t end = std::string::npos;
  for (size_t index = start; index < text.size(); ++index) {
    if (text[index] == '[') {
      ++depth;
    } else if (text[index] == ']') {
      --depth;
      if (depth == 0) {
        end = index;
        break;
      }
    }
  }
  if (end == std::string::npos || end <= start) return false;

  const std::string block = text.substr(start, end - start + 1);
  const std::regex vec_pattern(
      "\\[\\s*([-+0-9.eE]+)\\s*,\\s*([-+0-9.eE]+)\\s*,\\s*([-+0-9.eE]+)\\s*\\]");
  for (std::sregex_iterator it(block.begin(), block.end(), vec_pattern), done; it != done; ++it) {
    out->push_back({
        std::stod((*it)[1].str()) * scale,
        std::stod((*it)[2].str()) * scale,
        std::stod((*it)[3].str()) * scale,
    });
  }
  return !out->empty();
}

std::array<Vec3, 4> InferTableCornersRawMm(const TableFrame& table, const Args& args) {
  const double half_length_mm = args.table_length_m * 500.0;
  const double half_width_mm = args.table_width_m * 500.0;
  const Vec3 x_half = half_length_mm * table.x_axis_raw;
  const Vec3 y_half = half_width_mm * table.y_axis_raw;
  return {
      table.center_raw_mm - x_half - y_half,
      table.center_raw_mm - x_half + y_half,
      table.center_raw_mm + x_half - y_half,
      table.center_raw_mm + x_half + y_half,
  };
}

bool LoadTableCalibration(const std::string& path, Args* args, TableFrame* table) {
  std::string text;
  if (!ReadFile(path, &text)) {
    std::cerr << "Could not read table calibration: " << path << "\n";
    return false;
  }
  if (!ExtractVec3(text, "center_raw_m", 1000.0, &table->center_raw_mm) ||
      !ExtractVec3(text, "x_axis_raw", 1.0, &table->x_axis_raw) ||
      !ExtractVec3(text, "y_axis_raw", 1.0, &table->y_axis_raw)) {
    std::cerr << "Table calibration missing center_raw_m/x_axis_raw/y_axis_raw: " << path << "\n";
    return false;
  }
  Vec3 z_axis;
  if (ExtractVec3(text, "z_axis_raw", 1.0, &z_axis)) table->z_axis_raw = z_axis;
  double height = 0.0;
  if (ExtractNumber(text, "height_m_estimate", &height)) args->table_height_m = height;
  std::vector<Vec3> corners;
  if (ExtractVec3Array(text, "corners_raw_m", 1000.0, &corners) && corners.size() >= table->corners_raw_mm.size()) {
    for (size_t i = 0; i < table->corners_raw_mm.size(); ++i) table->corners_raw_mm[i] = corners[i];
  } else {
    table->corners_raw_mm = InferTableCornersRawMm(*table, *args);
  }
  table->valid = true;
  table->rectangle_score = 0.0;
  return true;
}

void WriteVec3Json(std::ostream& out, const Vec3& p, double scale) {
  out << "[" << p.x * scale << ", " << p.y * scale << ", " << p.z * scale << "]";
}

bool SaveTableCalibration(const std::string& path, const Args& args, const TableFrame& table) {
  std::ofstream output(path);
  if (!output) {
    std::cerr << "Could not write table calibration: " << path << "\n";
    return false;
  }

  output << std::fixed << std::setprecision(10);
  output << "{\n";
  output << "  \"center_raw_m\": ";
  WriteVec3Json(output, table.center_raw_mm, 0.001);
  output << ",\n  \"x_axis_raw\": ";
  WriteVec3Json(output, table.x_axis_raw, 1.0);
  output << ",\n  \"y_axis_raw\": ";
  WriteVec3Json(output, table.y_axis_raw, 1.0);
  output << ",\n  \"z_axis_raw\": ";
  WriteVec3Json(output, table.z_axis_raw, 1.0);
  output << ",\n  \"height_m_estimate\": " << args.table_height_m;
  output << ",\n  \"table_length_m\": " << args.table_length_m;
  output << ",\n  \"table_width_m\": " << args.table_width_m;
  output << ",\n  \"rectangle_score\": " << table.rectangle_score;
  output << ",\n  \"corners_raw_m\": [\n";
  for (size_t i = 0; i < table.corners_raw_mm.size(); ++i) {
    output << "    ";
    WriteVec3Json(output, table.corners_raw_mm[i], 0.001);
    output << (i + 1 == table.corners_raw_mm.size() ? "\n" : ",\n");
  }
  output << "  ]\n";
  output << "}\n";
  if (!output) {
    std::cerr << "Could not finish writing table calibration: " << path << "\n";
    return false;
  }
  std::cout << "Saved table calibration to " << path << "\n";
  return true;
}

double Det3(const double r[3][3]) {
  return r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1]) -
         r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0]) +
         r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]);
}

Quat Normalize(Quat q) {
  const double n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (n < 1.0e-12) return {};
  q.x /= n;
  q.y /= n;
  q.z /= n;
  q.w /= n;
  if (q.w < 0.0) {
    q.x = -q.x;
    q.y = -q.y;
    q.z = -q.z;
    q.w = -q.w;
  }
  return q;
}

void QuatToMatrix(const Quat& q_in, double r[3][3]) {
  const Quat q = Normalize(q_in);
  const double xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
  const double xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
  const double wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
  r[0][0] = 1.0 - 2.0 * (yy + zz);
  r[0][1] = 2.0 * (xy - wz);
  r[0][2] = 2.0 * (xz + wy);
  r[1][0] = 2.0 * (xy + wz);
  r[1][1] = 1.0 - 2.0 * (xx + zz);
  r[1][2] = 2.0 * (yz - wx);
  r[2][0] = 2.0 * (xz - wy);
  r[2][1] = 2.0 * (yz + wx);
  r[2][2] = 1.0 - 2.0 * (xx + yy);
}

Quat MatrixToQuat(const double r[3][3]) {
  Quat q;
  const double trace = r[0][0] + r[1][1] + r[2][2];
  if (trace > 0.0) {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    q.w = 0.25 * s;
    q.x = (r[2][1] - r[1][2]) / s;
    q.y = (r[0][2] - r[2][0]) / s;
    q.z = (r[1][0] - r[0][1]) / s;
  } else if (r[0][0] > r[1][1] && r[0][0] > r[2][2]) {
    const double s = std::sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0;
    q.w = (r[2][1] - r[1][2]) / s;
    q.x = 0.25 * s;
    q.y = (r[0][1] + r[1][0]) / s;
    q.z = (r[0][2] + r[2][0]) / s;
  } else if (r[1][1] > r[2][2]) {
    const double s = std::sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0;
    q.w = (r[0][2] - r[2][0]) / s;
    q.x = (r[0][1] + r[1][0]) / s;
    q.y = 0.25 * s;
    q.z = (r[1][2] + r[2][1]) / s;
  } else {
    const double s = std::sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0;
    q.w = (r[1][0] - r[0][1]) / s;
    q.x = (r[0][2] + r[2][0]) / s;
    q.y = (r[1][2] + r[2][1]) / s;
    q.z = 0.25 * s;
  }
  return Normalize(q);
}

Vec3 Rotate(const double r[3][3], const Vec3& p) {
  return {
      r[0][0] * p.x + r[0][1] * p.y + r[0][2] * p.z,
      r[1][0] * p.x + r[1][1] * p.y + r[1][2] * p.z,
      r[2][0] * p.x + r[2][1] * p.y + r[2][2] * p.z,
  };
}

bool FitRigidHorn(const std::vector<Vec3>& ref, const std::vector<Vec3>& cur, Vec3* t, Quat* q, double* rms_mm) {
  if (ref.size() != cur.size() || ref.size() < 3) return false;

  const Vec3 ref_mean = Mean(ref);
  const Vec3 cur_mean = Mean(cur);

  double sxx = 0, sxy = 0, sxz = 0;
  double syx = 0, syy = 0, syz = 0;
  double szx = 0, szy = 0, szz = 0;
  for (size_t i = 0; i < ref.size(); ++i) {
    const Vec3 a = ref[i] - ref_mean;
    const Vec3 b = cur[i] - cur_mean;
    sxx += a.x * b.x; sxy += a.x * b.y; sxz += a.x * b.z;
    syx += a.y * b.x; syy += a.y * b.y; syz += a.y * b.z;
    szx += a.z * b.x; szy += a.z * b.y; szz += a.z * b.z;
  }

  double n[4][4] = {
      {sxx + syy + szz, syz - szy, szx - sxz, sxy - syx},
      {syz - szy, sxx - syy - szz, sxy + syx, sxz + szx},
      {szx - sxz, sxy + syx, -sxx + syy - szz, syz + szy},
      {sxy - syx, sxz + szx, syz + szy, -sxx - syy + szz},
  };

  std::array<double, 4> v = {1.0, 0.0, 0.0, 0.0};
  for (int iter = 0; iter < 80; ++iter) {
    std::array<double, 4> next = {0.0, 0.0, 0.0, 0.0};
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) next[row] += n[row][col] * v[col];
    }
    const double len = std::sqrt(std::inner_product(next.begin(), next.end(), next.begin(), 0.0));
    if (len < 1.0e-12) return false;
    for (double& value : next) value /= len;
    v = next;
  }

  *q = Normalize({v[1], v[2], v[3], v[0]});
  double r[3][3];
  QuatToMatrix(*q, r);
  if (Det3(r) < 0.0) return false;

  *t = cur_mean - Rotate(r, ref_mean);

  double sum_sq = 0.0;
  for (size_t i = 0; i < ref.size(); ++i) {
    const Vec3 pred = Rotate(r, ref[i]) + *t;
    const Vec3 err = cur[i] - pred;
    sum_sq += Dot(err, err);
  }
  *rms_mm = std::sqrt(sum_sq / ref.size());
  return std::isfinite(*rms_mm);
}

bool ParseArgs(int argc, char** argv, Args* args) {
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for " << key << "\n";
        std::exit(2);
      }
      return argv[++i];
    };
    if (key == "--host") args->host = next();
    else if (key == "--base-subject") args->base_subject = next();
    else if (key == "--duration") args->duration_s = std::stod(next());
    else if (key == "--print-hz") args->print_hz = std::stod(next());
    else if (key == "--calib-sec") args->calib_s = std::stod(next());
    else if (key == "--table-calib") args->table_calib_path = next();
    else if (key == "--save-table-calib") args->save_table_calib_path = next();
    else if (key == "--lcm-url") args->lcm_url = next();
    else if (key == "--table-length") args->table_length_m = std::stod(next());
    else if (key == "--table-width") args->table_width_m = std::stod(next());
    else if (key == "--table-height") args->table_height_m = std::stod(next());
    else if (key == "--expected-base-edge-distance") args->expected_base_edge_distance_m = std::stod(next());
    else if (key == "--base-anchor-x") args->base_anchor_x_m = std::stod(next());
    else if (key == "--base-anchor-y") args->base_anchor_y_m = std::stod(next());
    else if (key == "--base-anchor-z") args->base_anchor_z_m = std::stod(next());
    else if (key == "--vicon-frame-rate-hz") args->vicon_frame_rate_hz = std::stod(next());
    else if (key == "--ball-max-speed") args->ball_max_speed_mps = std::stod(next());
    else if (key == "--ball-min-bootstrap-speed") args->ball_min_bootstrap_speed_mps = std::stod(next());
    else if (key == "--ball-min-bootstrap-displacement") args->ball_min_bootstrap_displacement_m = std::stod(next());
    else if (key == "--ball-min-gate") args->ball_min_gate_m = std::stod(next());
    else if (key == "--ball-gate-margin") args->ball_gate_margin_m = std::stod(next());
    else if (key == "--ball-track-reset-frames") args->ball_track_reset_frames = std::stoi(next());
    else if (key == "--clear-raw-ignore-spheres") args->raw_ignore_spheres.clear();
    else if (key == "--ignore-raw-sphere-m") {
      RawIgnoreSphere sphere;
      const std::string spec = next();
      if (!ParseRawIgnoreSphereM(spec, &sphere)) {
        std::cerr << "Invalid --ignore-raw-sphere-m value, expected x,y,z,r in meters: " << spec << "\n";
        return false;
      }
      args->raw_ignore_spheres.push_back(sphere);
    }
    else if (key == "--raw-base-anchor") args->calibrate_base_anchor = false;
    else if (key == "--calibrate-base-anchor") args->calibrate_base_anchor = true;
    else if (key == "--publish") args->publish = true;
    else if (key == "--no-publish") args->publish = false;
    else if (key == "--help" || key == "-h") {
      std::cout << "Usage: " << argv[0]
                << " [--host 192.168.10.1:801] [--base-subject G1Pelvis]"
                << " [--calib-sec 2] [--table-calib table_frame_latest.json]"
                << " [--save-table-calib table_frame_latest.json]"
                << " [--ignore-raw-sphere-m x,y,z,r] [--clear-raw-ignore-spheres]"
                << " [--vicon-frame-rate-hz 300] [--ball-max-speed 12]"
                << " [--ball-min-bootstrap-speed 0.30]"
                << " [--ball-min-bootstrap-displacement 0.01]"
                << " [--ball-min-gate 0.06] [--ball-gate-margin 0.03]"
                << " [--duration 0] [--publish]\n";
      std::exit(0);
    } else {
      std::cerr << "Unknown argument: " << key << "\n";
      return false;
    }
  }
  return true;
}

void AddClusterSample(std::vector<Cluster>* clusters, const Vec3& p, double gate_mm) {
  int best = -1;
  double best_dist = gate_mm;
  for (size_t i = 0; i < clusters->size(); ++i) {
    const double dist = Distance(p, (*clusters)[i].mean);
    if (dist < best_dist) {
      best_dist = dist;
      best = static_cast<int>(i);
    }
  }

  if (best < 0) {
    Cluster c;
    c.mean = p;
    c.sum = p;
    c.sum_sq = {p.x * p.x, p.y * p.y, p.z * p.z};
    c.count = 1;
    clusters->push_back(c);
    return;
  }

  Cluster& c = (*clusters)[best];
  c.sum = c.sum + p;
  c.sum_sq = c.sum_sq + Vec3{p.x * p.x, p.y * p.y, p.z * p.z};
  c.count += 1;
  c.mean = c.sum / static_cast<double>(c.count);
}

double RectScore(const std::array<Vec3, 4>& pts, double length_mm, double width_mm) {
  std::vector<double> distances;
  distances.reserve(6);
  for (int i = 0; i < 4; ++i) {
    for (int j = i + 1; j < 4; ++j) distances.push_back(Distance(pts[i], pts[j]));
  }
  std::sort(distances.begin(), distances.end());

  const double a = std::min(length_mm, width_mm);
  const double b = std::max(length_mm, width_mm);
  const double d = std::sqrt(a * a + b * b);
  const std::array<double, 6> expected = {a, a, b, b, d, d};

  double score = 0.0;
  for (int i = 0; i < 6; ++i) score += std::abs(distances[i] - expected[i]) / expected[i];

  double z_min = pts[0].z;
  double z_max = pts[0].z;
  for (const auto& p : pts) {
    z_min = std::min(z_min, p.z);
    z_max = std::max(z_max, p.z);
  }
  score += (z_max - z_min) / 300.0;
  return score;
}

TableFrame BuildTableFrame(const std::vector<Cluster>& clusters_in, const Vec3& robot_raw_mm, const Args& args) {
  std::vector<Cluster> clusters = clusters_in;
  std::sort(clusters.begin(), clusters.end(), [](const Cluster& a, const Cluster& b) {
    return a.count > b.count;
  });
  if (clusters.size() > 12) clusters.resize(12);

  TableFrame frame;
  if (clusters.size() < 4) return frame;

  double best_score = std::numeric_limits<double>::infinity();
  std::array<Vec3, 4> best{};
  const double length_mm = args.table_length_m * 1000.0;
  const double width_mm = args.table_width_m * 1000.0;

  for (size_t a = 0; a + 3 < clusters.size(); ++a) {
    for (size_t b = a + 1; b + 2 < clusters.size(); ++b) {
      for (size_t c = b + 1; c + 1 < clusters.size(); ++c) {
        for (size_t d = c + 1; d < clusters.size(); ++d) {
          std::array<Vec3, 4> pts = {clusters[a].mean, clusters[b].mean, clusters[c].mean, clusters[d].mean};
          const double score = RectScore(pts, length_mm, width_mm);
          if (score < best_score) {
            best_score = score;
            best = pts;
          }
        }
      }
    }
  }

  frame.valid = std::isfinite(best_score);
  frame.corners_raw_mm = best;
  frame.rectangle_score = best_score;
  frame.center_raw_mm = (best[0] + best[1] + best[2] + best[3]) / 4.0;

  double cxx = 0.0;
  double cxy = 0.0;
  double cyy = 0.0;
  for (const auto& p : best) {
    const Vec3 d = p - frame.center_raw_mm;
    cxx += d.x * d.x;
    cxy += d.x * d.y;
    cyy += d.y * d.y;
  }
  const double theta = 0.5 * std::atan2(2.0 * cxy, cxx - cyy);
  Vec3 x_axis{std::cos(theta), std::sin(theta), 0.0};
  Vec3 robot_delta = robot_raw_mm - frame.center_raw_mm;
  if (Dot(robot_delta, x_axis) > 0.0) x_axis = -1.0 * x_axis;
  frame.x_axis_raw = NormalizeVec(x_axis);
  frame.y_axis_raw = NormalizeVec(Vec3{-frame.x_axis_raw.y, frame.x_axis_raw.x, 0.0});
  return frame;
}

Vec3 RawToTableWorld(const Vec3& raw_mm, const TableFrame& table, const Args& args) {
  const Vec3 rel = raw_mm - table.center_raw_mm;
  return {
      args.table_length_m * 0.5 + Dot(rel, table.x_axis_raw) * 0.001,
      Dot(rel, table.y_axis_raw) * 0.001,
      args.table_height_m + Dot(rel, table.z_axis_raw) * 0.001,
  };
}

bool NearTableCorner(const Vec3& raw_mm, const TableFrame& table, double gate_mm) {
  for (const auto& corner : table.corners_raw_mm) {
    if (Distance(raw_mm, corner) < gate_mm) return true;
  }
  return false;
}

bool PlausibleBallWorld(const Vec3& p, const Args& args) {
  return p.x > -0.8 && p.x < args.table_length_m + 0.8 &&
         p.y > -1.4 && p.y < 1.4 &&
         p.z > args.table_height_m - 0.25 && p.z < args.table_height_m + 1.6;
}

std::vector<BallCandidate> BallCandidates(
    const std::vector<Vec3>& unlabeled_raw,
    const TableFrame& table,
    const Args& args,
    BallSelectionDiagnostics* diagnostics = nullptr) {
  std::vector<BallCandidate> candidates;
  for (const auto& raw : unlabeled_raw) {
    if (IgnoredRawMarker(raw, args)) {
      if (diagnostics != nullptr) ++diagnostics->ignored_raw_count;
      continue;
    }
    if (NearTableCorner(raw, table, 120.0)) {
      if (diagnostics != nullptr) ++diagnostics->near_corner_count;
      continue;
    }
    const Vec3 world = RawToTableWorld(raw, table, args);
    if (!PlausibleBallWorld(world, args)) {
      if (diagnostics != nullptr) ++diagnostics->implausible_world_count;
      continue;
    }
    candidates.push_back(BallCandidate{raw, world});
  }
  if (diagnostics != nullptr) diagnostics->candidate_count = candidates.size();
  return candidates;
}

void PruneBallBootstrap(BallTrackState* track, int64_t frame_number, const Args& args) {
  const int64_t max_age_frames = std::max<int64_t>(1, args.ball_track_reset_frames);
  track->bootstrap_observations.erase(
      std::remove_if(
          track->bootstrap_observations.begin(),
          track->bootstrap_observations.end(),
          [&](const BallBootstrapObservation& obs) {
            return frame_number <= obs.frame_number ||
                   frame_number - obs.frame_number > max_age_frames;
          }),
      track->bootstrap_observations.end());
}

void RememberBallBootstrapCandidates(
    BallTrackState* track,
    const std::vector<BallCandidate>& candidates,
    int64_t frame_number) {
  constexpr size_t kMaxBootstrapObservations = 16;
  for (const auto& candidate : candidates) {
    track->bootstrap_observations.push_back(BallBootstrapObservation{candidate.world, frame_number});
  }
  if (track->bootstrap_observations.size() > kMaxBootstrapObservations) {
    track->bootstrap_observations.erase(
        track->bootstrap_observations.begin(),
        track->bootstrap_observations.end() -
            static_cast<std::ptrdiff_t>(kMaxBootstrapObservations));
  }
}

bool TryBootstrapBallTrack(
    const std::vector<BallCandidate>& candidates,
    const Args& args,
    BallTrackState* track,
    int64_t frame_number,
    size_t* best_index,
    BallSelectionDiagnostics* diagnostics = nullptr) {
  PruneBallBootstrap(track, frame_number, args);
  if (diagnostics != nullptr) {
    diagnostics->bootstrap_observation_count = track->bootstrap_observations.size();
  }

  bool found = false;
  double best_distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < candidates.size(); ++i) {
    for (const auto& obs : track->bootstrap_observations) {
      const double dt = FrameDeltaSeconds(frame_number, obs.frame_number, args.vicon_frame_rate_hz);
      if (dt <= 1.0e-6) continue;
      const double distance = Distance(candidates[i].world, obs.world);
      if (distance < args.ball_min_bootstrap_displacement_m) continue;
      const double speed = distance / dt;
      if (speed < args.ball_min_bootstrap_speed_mps || speed > args.ball_max_speed_mps) continue;
      if (distance < best_distance) {
        *best_index = i;
        best_distance = distance;
        found = true;
      }
    }
  }

  if (found) {
    track->bootstrap_observations.clear();
    return true;
  }

  RememberBallBootstrapCandidates(track, candidates, frame_number);
  if (diagnostics != nullptr) {
    diagnostics->bootstrap_observation_count = track->bootstrap_observations.size();
    diagnostics->reason = BallRejectReason::BootstrapWait;
  }
  return false;
}

bool SelectBall(
    const std::vector<Vec3>& unlabeled_raw,
    const TableFrame& table,
    const Args& args,
    BallTrackState* track,
    int64_t frame_number,
    Vec3* ball_raw,
    Vec3* ball_world,
    BallSelectionDiagnostics* diagnostics = nullptr) {
  if (diagnostics != nullptr) *diagnostics = BallSelectionDiagnostics{};
  if (unlabeled_raw.empty()) {
    if (diagnostics != nullptr) diagnostics->reason = BallRejectReason::NoUnlabeled;
    return false;
  }

  const std::vector<BallCandidate> candidates = BallCandidates(unlabeled_raw, table, args, diagnostics);
  if (candidates.empty()) {
    if (diagnostics != nullptr) diagnostics->reason = BallRejectReason::NoCandidates;
    return false;
  }

  const bool tracking = track->have_position && track->missed_frames <= args.ball_track_reset_frames;
  size_t best = 0;
  if (tracking) {
    const double dt = FrameDeltaSeconds(frame_number, track->frame_number, args.vicon_frame_rate_hz);
    const Vec3 predicted = track->have_velocity ? track->position + dt * track->velocity : track->position;
    const double gate_m = std::max(args.ball_min_gate_m, args.ball_max_speed_mps * dt + args.ball_gate_margin_m);
    if (diagnostics != nullptr) {
      diagnostics->tracking_dt_s = dt;
      diagnostics->tracking_gate_m = gate_m;
    }
    double best_pred_dist = std::numeric_limits<double>::infinity();
    double diagnostic_best_pred_dist = std::numeric_limits<double>::infinity();
    double best_direct_speed = 0.0;
    bool found = false;
    for (size_t i = 0; i < candidates.size(); ++i) {
      const double direct_speed = dt > 1.0e-6 ? Distance(candidates[i].world, track->position) / dt : 0.0;
      if (direct_speed > args.ball_max_speed_mps) continue;
      const double pred_dist = Distance(candidates[i].world, predicted);
      if (pred_dist < diagnostic_best_pred_dist) {
        diagnostic_best_pred_dist = pred_dist;
        best_direct_speed = direct_speed;
      }
      if (pred_dist <= gate_m && pred_dist < best_pred_dist) {
        best = i;
        best_pred_dist = pred_dist;
        found = true;
      }
    }
    if (diagnostics != nullptr) {
      diagnostics->tracking_best_pred_dist_m = diagnostic_best_pred_dist;
      diagnostics->tracking_best_direct_speed_mps = best_direct_speed;
    }
    if (!found) {
      if (diagnostics != nullptr) diagnostics->reason = BallRejectReason::TrackingGateFail;
      return false;
    }
  } else {
    if (!TryBootstrapBallTrack(candidates, args, track, frame_number, &best, diagnostics)) return false;
  }
  *ball_raw = candidates[best].raw;
  *ball_world = candidates[best].world;
  if (diagnostics != nullptr) diagnostics->reason = BallRejectReason::None;
  return true;
}

void UpdateBallTrack(BallTrackState* track, const Vec3& ball_world, int64_t frame_number, const Args& args) {
  if (track->have_position) {
    const double dt = FrameDeltaSeconds(frame_number, track->frame_number, args.vicon_frame_rate_hz);
    if (dt > 1.0e-6) {
      track->velocity = (ball_world - track->position) / dt;
      track->have_velocity = true;
    }
  }
  track->position = ball_world;
  track->frame_number = frame_number;
  track->have_position = true;
  track->missed_frames = 0;
  track->bootstrap_observations.clear();
}

void NoteBallMiss(BallTrackState* track, const Args& args) {
  if (!track->have_position) return;
  ++track->missed_frames;
  if (track->missed_frames > args.ball_track_reset_frames) {
    track->have_position = false;
    track->have_velocity = false;
    track->missed_frames = 0;
  }
}

Quat TableQuatFromRelativeBase(const Quat& base_q) {
  double r[3][3];
  QuatToMatrix(base_q, r);
  const double yaw = std::atan2(r[1][0], r[0][0]);
  const double half = 0.5 * yaw;
  return Normalize({0.0, 0.0, std::sin(half), std::cos(half)});
}

void FillMessage(
    lcm_types::transformation_t* msg,
    const std::string& name,
    const Vec3& pos,
    const Quat& quat,
    int64_t frame_number,
    const Args& args,
    bool valid,
    bool occluded) {
  msg->name = name;
  msg->vicon_frame_number = frame_number;
  msg->vicon_time_s = args.vicon_frame_rate_hz > 0.0
                          ? static_cast<double>(frame_number) / args.vicon_frame_rate_hz
                          : 0.0;
  msg->publish_time_us = NowUnixMicros();
  msg->valid = valid ? 1 : 0;
  msg->occluded = occluded ? 1 : 0;
  msg->pos_vicon[0] = pos.x;
  msg->pos_vicon[1] = pos.y;
  msg->pos_vicon[2] = pos.z;
  msg->quat_vicon[0] = quat.x;
  msg->quat_vicon[1] = quat.y;
  msg->quat_vicon[2] = quat.z;
  msg->quat_vicon[3] = quat.w;
}

void PublishTransform(lcm::LCM* lcm, const std::string& channel, const lcm_types::transformation_t& msg) {
  std::vector<unsigned char> buffer(static_cast<size_t>(msg.getEncodedSize()));
  const int encoded = msg.encode(buffer.data(), 0, static_cast<int>(buffer.size()));
  if (encoded > 0) lcm->publish(channel, buffer.data(), static_cast<unsigned int>(encoded));
}

void PrintRejectCounts(
    std::ostream& out,
    const std::array<unsigned long long, kBallRejectReasonCount>& counts) {
  out << "{";
  bool first = true;
  for (size_t i = 0; i < counts.size(); ++i) {
    if (counts[i] == 0) continue;
    if (!first) out << ",";
    first = false;
    out << BallRejectReasonName(static_cast<BallRejectReason>(i)) << ":" << counts[i];
  }
  if (first) out << "none:0";
  out << "}";
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);

  Args args;
  if (!ParseArgs(argc, argv, &args)) return 2;

  Client client;
  client.SetStreamMode(StreamMode::ClientPull);

  std::cout << "Connecting to " << args.host << " ...\n";
  const auto connect = client.Connect(args.host);
  if (!Ok(connect.Result)) {
    std::cerr << "Connect failed: " << static_cast<int>(connect.Result) << "\n";
    return 1;
  }

  client.EnableMarkerData();
  client.EnableSegmentData();
  client.EnableUnlabeledMarkerData();

  if (!WaitFrame(&client, 5.0)) {
    std::cerr << "Timed out waiting for initial Vicon frame.\n";
    return 1;
  }

  const bool calibrating_table =
      args.table_calib_path.empty() && !args.save_table_calib_path.empty();

  std::map<std::string, Vec3> ref_markers;
  const auto base_capture_start = std::chrono::steady_clock::now();
  while (!g_stop) {
    WaitFrame(&client, 1.0);
    ref_markers = VisibleSubjectMarkers(&client, args.base_subject);
    if (ref_markers.size() >= 3) break;
    std::cout << "Waiting for at least 3 visible " << args.base_subject
              << " markers; currently " << ref_markers.size() << "\n";
    if (calibrating_table) {
      const double waited =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - base_capture_start).count();
      if (waited >= 2.0) break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  if (ref_markers.size() < 3 && !calibrating_table) {
    std::cerr << "Could not capture rigid template for " << args.base_subject << "\n";
    return 1;
  }

  std::vector<Vec3> ref_marker_points;
  for (const auto& kv : ref_markers) ref_marker_points.push_back(kv.second);
  Vec3 base_ref_centroid = Mean(ref_marker_points);
  if (ref_markers.size() >= 3) {
    std::cout << "Captured " << args.base_subject << " template with " << ref_markers.size() << " markers:";
    for (const auto& kv : ref_markers) std::cout << " " << kv.first;
    std::cout << "\n";
  } else {
    std::cout << "Could not capture " << args.base_subject
              << " template; table calibration will use previous table x-axis for orientation.\n";
    TableFrame previous_table;
    Args previous_args = args;
    if (LoadTableCalibration(args.save_table_calib_path, &previous_args, &previous_table)) {
      base_ref_centroid = previous_table.center_raw_mm - 1000.0 * previous_table.x_axis_raw;
    } else {
      std::cout << "Previous table calibration unavailable; table x-axis sign may need manual verification.\n";
    }
  }

  TableFrame table;
  if (!args.table_calib_path.empty()) {
    std::cout << "Loading table calibration from " << args.table_calib_path << " ...\n";
    if (!LoadTableCalibration(args.table_calib_path, &args, &table)) return 1;
  } else {
    std::cout << "Calibrating table from unlabeled markers for " << args.calib_s << " s ...\n";
    std::vector<Cluster> clusters;
    const auto calib_start = std::chrono::steady_clock::now();
    int calib_frames = 0;
    while (!g_stop) {
      const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - calib_start).count();
      if (elapsed >= args.calib_s) break;
      if (!WaitFrame(&client, 1.0)) continue;
      ++calib_frames;
      for (const auto& p : UnlabeledMarkers(&client)) AddClusterSample(&clusters, p, 80.0);
    }

    clusters.erase(
        std::remove_if(clusters.begin(), clusters.end(), [calib_frames](const Cluster& c) {
          return c.count < std::max(5, calib_frames / 10);
        }),
        clusters.end());
    std::sort(clusters.begin(), clusters.end(), [](const Cluster& a, const Cluster& b) {
      return a.count > b.count;
    });

    std::cout << "Stable unlabeled clusters (" << clusters.size() << "):\n";
    for (size_t i = 0; i < clusters.size(); ++i) {
      const auto& c = clusters[i];
      std::cout << std::fixed << std::setprecision(1)
                << "  #" << i << " count=" << c.count
                << " raw_mm=[" << c.mean.x << ", " << c.mean.y << ", " << c.mean.z << "]\n";
    }

    table = BuildTableFrame(clusters, base_ref_centroid, args);
  }
  if (!table.valid) {
    std::cerr << "Could not infer table corners from unlabeled clusters.\n";
    return 1;
  }
  if (!args.save_table_calib_path.empty() &&
      !SaveTableCalibration(args.save_table_calib_path, args, table)) {
    return 1;
  }

  std::cout << std::fixed << std::setprecision(4)
            << "Table rectangle score=" << table.rectangle_score
            << " center_raw_m=[" << table.center_raw_mm.x * 0.001 << ", "
            << table.center_raw_mm.y * 0.001 << ", " << table.center_raw_mm.z * 0.001 << "]"
            << " x_axis_raw=[" << table.x_axis_raw.x << ", " << table.x_axis_raw.y << ", " << table.x_axis_raw.z << "]"
            << " y_axis_raw=[" << table.y_axis_raw.x << ", " << table.y_axis_raw.y << ", " << table.y_axis_raw.z << "]\n";
  std::cout << "Table-world convention: robot-side edge x=0.0000 m, far edge x="
            << args.table_length_m << " m, expected " << args.base_subject
            << " x ~= -" << args.expected_base_edge_distance_m << " m\n";
  if (args.table_calib_path.empty()) {
    for (size_t i = 0; i < table.corners_raw_mm.size(); ++i) {
      const Vec3 w = RawToTableWorld(table.corners_raw_mm[i], table, args);
      std::cout << "  table_corner_" << i << "_world_m=[" << w.x << ", " << w.y << ", " << w.z << "]\n";
    }
  }

  const Vec3 base_initial_measured_world = RawToTableWorld(base_ref_centroid, table, args);
  Vec3 base_anchor_initial_world = base_initial_measured_world;
  if (args.calibrate_base_anchor) {
    base_anchor_initial_world.x = std::isfinite(args.base_anchor_x_m)
                                      ? args.base_anchor_x_m
                                      : -args.expected_base_edge_distance_m;
    if (std::isfinite(args.base_anchor_y_m)) base_anchor_initial_world.y = args.base_anchor_y_m;
    base_anchor_initial_world.z = args.base_anchor_z_m;
  }
  std::cout << "Base initial measured_world_m=[" << base_initial_measured_world.x << ", "
            << base_initial_measured_world.y << ", " << base_initial_measured_world.z << "]"
            << " anchor_initial_world_m=[" << base_anchor_initial_world.x << ", "
            << base_anchor_initial_world.y << ", " << base_anchor_initial_world.z << "]"
            << (args.calibrate_base_anchor ? " calibrated\n" : " raw\n");

  lcm::LCM lcm(args.lcm_url);
  if (args.publish && !lcm.good()) {
    std::cerr << "Could not initialize LCM: " << args.lcm_url << "\n";
    return 1;
  }
  std::cout << (args.publish ? "Publishing" : "Monitoring only")
            << " on vicon_state_data via " << args.lcm_url << "\n";

  const auto start = std::chrono::steady_clock::now();
  auto last_print = start - std::chrono::seconds(10);
  unsigned long frames = 0;
  BallTrackState ball_track;
  std::array<unsigned long long, kBallRejectReasonCount> ball_reject_counts{};

  while (!g_stop) {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - start).count();
    if (args.duration_s > 0.0 && elapsed >= args.duration_s) break;
    if (!WaitFrame(&client, 2.0)) continue;
    ++frames;
    const int64_t frame_number = static_cast<int64_t>(client.GetFrameNumber().FrameNumber);

    const auto cur_markers_map = VisibleSubjectMarkers(&client, args.base_subject);
    std::vector<Vec3> ref;
    std::vector<Vec3> cur;
    for (const auto& kv : ref_markers) {
      const auto found = cur_markers_map.find(kv.first);
      if (found != cur_markers_map.end()) {
        ref.push_back(kv.second);
        cur.push_back(found->second);
      }
    }

    Vec3 base_t;
    Quat base_q;
    double base_rms = 0.0;
    const bool base_valid = FitRigidHorn(ref, cur, &base_t, &base_q, &base_rms);
    Vec3 base_raw = base_ref_centroid;
    if (base_valid) {
      double r[3][3];
      QuatToMatrix(base_q, r);
      base_raw = Rotate(r, base_ref_centroid) + base_t;
    }
    const Vec3 base_measured_world = RawToTableWorld(base_raw, table, args);
    const Vec3 base_world = base_anchor_initial_world + (base_measured_world - base_initial_measured_world);
    const Quat base_world_q = TableQuatFromRelativeBase(base_q);

    const std::vector<Vec3> unlabeled = UnlabeledMarkers(&client);
    const size_t raw_ignore_count = CountIgnoredRawMarkers(unlabeled, args);
    Vec3 ball_raw;
    Vec3 ball_world;
    BallSelectionDiagnostics ball_diag;
    bool ball_valid = false;
    if (!base_valid) {
      ball_diag.reason = BallRejectReason::BaseInvalid;
    } else {
      ball_valid = SelectBall(unlabeled, table, args, &ball_track, frame_number, &ball_raw, &ball_world, &ball_diag);
    }
    if (ball_valid) {
      UpdateBallTrack(&ball_track, ball_world, frame_number, args);
    } else {
      NoteBallMiss(&ball_track, args);
      ++ball_reject_counts[BallRejectReasonIndex(ball_diag.reason)];
    }

    if (args.publish) {
      lcm_types::transformation_t msg;
      FillMessage(&msg, args.base_subject, base_world, base_world_q, frame_number, args, base_valid, !base_valid);
      PublishTransform(&lcm, "vicon_state_data", msg);
      if (ball_valid) {
        FillMessage(&msg, "ball", ball_world, Quat{}, frame_number, args, true, false);
        PublishTransform(&lcm, "vicon_state_data", msg);
      }
      FillMessage(&msg, "table", Vec3{args.table_length_m * 0.5, 0.0, args.table_height_m}, Quat{}, frame_number, args, true, false);
      PublishTransform(&lcm, "vicon_state_data", msg);
    }

    const double print_period = 1.0 / std::max(args.print_hz, 1.0e-6);
    if (std::chrono::duration<double>(now - last_print).count() >= print_period) {
      const double hz = frames / std::max(elapsed, 1.0e-6);
      std::cout << std::fixed << std::setprecision(4)
                << "frame=" << frame_number
                << " hz=" << std::setprecision(1) << hz
                << " base_valid=" << base_valid
                << " base_markers=" << cur.size()
                << " base_rms_mm=" << std::setprecision(2) << base_rms
                << " base_measured_m=[" << std::setprecision(4)
                << base_measured_world.x << ", " << base_measured_world.y << ", " << base_measured_world.z << "]"
                << " base_world_m=["
                << base_world.x << ", " << base_world.y << ", " << base_world.z << "]"
                << " base_edge_error_m=" << base_world.x + args.expected_base_edge_distance_m
                << " base_quat_xyzw=[" << base_world_q.x << ", " << base_world_q.y << ", "
                << base_world_q.z << ", " << base_world_q.w << "]"
                << " unlabeled_count=" << unlabeled.size()
                << " raw_ignore_count=" << raw_ignore_count
                << " ball_valid=" << ball_valid
                << " ball_missed_frames=" << ball_track.missed_frames;
      if (ball_valid) {
        std::cout << " ball_world_m=[" << ball_world.x << ", " << ball_world.y << ", " << ball_world.z << "]"
                  << " ball_raw_m=[" << ball_raw.x * 0.001 << ", " << ball_raw.y * 0.001 << ", " << ball_raw.z * 0.001 << "]";
      } else {
        std::cout << " ball_reject=" << BallRejectReasonName(ball_diag.reason)
                  << " ball_reject_counts=";
        PrintRejectCounts(std::cout, ball_reject_counts);
        std::cout << " ball_candidates=" << ball_diag.candidate_count
                  << " ball_filter_counts=[ignored_raw:" << ball_diag.ignored_raw_count
                  << ",near_corner:" << ball_diag.near_corner_count
                  << ",implausible_world:" << ball_diag.implausible_world_count << "]"
                  << " bootstrap_obs=" << ball_diag.bootstrap_observation_count;
        if (ball_diag.reason == BallRejectReason::TrackingGateFail) {
          std::cout << " track_dt_s=" << ball_diag.tracking_dt_s
                    << " track_gate_m=" << ball_diag.tracking_gate_m
                    << " best_pred_dist_m=" << ball_diag.tracking_best_pred_dist_m
                    << " best_direct_speed_mps=" << ball_diag.tracking_best_direct_speed_mps;
        }
      }
      std::cout << "\n";
      ball_reject_counts.fill(0);
      last_print = now;
    }
  }

  client.Disconnect();
  return 0;
}
