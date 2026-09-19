#include "DataStreamClient.h"

#include <lcm/lcm-cpp.hpp>
#include "unitree_sdk2/lcm_types/transformation_t.hpp"
#include <unitree/common/json/jsonize.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace ViconDataStreamSDK::CPP;

namespace {

volatile std::sig_atomic_t g_stop = 0;
void HandleSignal(int) { g_stop = 1; }

constexpr const char* kBaseSubject = "G1Pelvis";

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

constexpr Vec3 kTrackerNativeUpRaw{0.0, 0.0, 1.0};

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
  std::string base_subject = kBaseSubject;
  std::string lcm_url = "udpm://239.255.76.67:7667?ttl=255";
  std::string table_calib_path;
  std::string save_table_calib_path;
  double duration_s = 0.0;
  double print_hz = 10.0;
  double calib_s = 2.0;
  double table_length_m = 2.730738;
  double table_width_m = 1.512451;
  double table_height_m = 0.760000;
  double expected_base_edge_distance_m = 0.40;
  double vicon_frame_rate_hz = 300.0;
  double corner_exclusion_radius_mm = 50.0;
  std::vector<RawIgnoreSphere> raw_ignore_spheres;
  bool publish = false;
};

struct Cluster {
  Vec3 mean;
  Vec3 sum;
  Vec3 sum_sq;
  int count = 0;
  int observed_frames = 0;
  int last_observed_frame = -1;
  std::vector<Vec3> samples;
};

struct ClusterStability {
  bool stable = false;
  double coverage = 0.0;
  double rms_deviation_mm = std::numeric_limits<double>::infinity();
  double max_deviation_mm = std::numeric_limits<double>::infinity();
};

struct TableFrame {
  bool valid = false;
  Vec3 center_raw_mm;
  Vec3 x_axis_raw;
  Vec3 y_axis_raw;
  Vec3 z_axis_raw{0.0, 0.0, 1.0};
  std::array<Vec3, 4> corners_raw_mm;
  double rectangle_score = 0.0;
  double corner_max_error_m = std::numeric_limits<double>::infinity();
  double corner_rms_error_m = std::numeric_limits<double>::infinity();
};

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

CalibrationValidation ValidateTableFrame(
    TableFrame* frame,
    const TableDimensions& dims,
    const Vec3* pelvis_raw_mm,
    double allowed_max_error_m);

struct BallCandidate {
  Vec3 raw;
  Vec3 world;
};

enum class BallRejectReason {
  None = 0,
  BaseInvalid,
  NoCandidates,
  Count,
};

constexpr size_t kBallRejectReasonCount = static_cast<size_t>(BallRejectReason::Count);

const char* BallRejectReasonName(BallRejectReason reason) {
  switch (reason) {
    case BallRejectReason::None:
      return "none";
    case BallRejectReason::BaseInvalid:
      return "base_invalid";
    case BallRejectReason::NoCandidates:
      return "no_candidates";
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
  size_t raw_ignored_count = 0;
  size_t corner_ignored_count = 0;
  size_t candidate_count = 0;
  double tracking_dt_s = 0.0;
  double tracking_best_pred_dist_m = std::numeric_limits<double>::infinity();
  double tracking_best_direct_speed_mps = 0.0;
};

struct BallTrackState {
  bool have_position = false;
  bool have_velocity = false;
  Vec3 position;
  Vec3 velocity;
  int64_t frame_number = 0;
  int missed_frames = 0;
};

Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 operator*(double s, const Vec3& a) { return {s * a.x, s * a.y, s * a.z}; }
Vec3 operator/(const Vec3& a, double s) { return {a.x / s, a.y / s, a.z / s}; }

double Dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3 Cross(const Vec3& a, const Vec3& b) {
  return {a.y * b.z - a.z * b.y,
          a.z * b.x - a.x * b.z,
          a.x * b.y - a.y * b.x};
}
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

Vec3 NormalizeVec(const Vec3& p) {
  const double n = Norm(p);
  if (n < 1.0e-12) return {};
  return p / n;
}

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
            [&a](size_t lhs, size_t rhs) {
              return a[lhs][lhs] < a[rhs][rhs];
            });
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

bool IsSavedTableCorner(
    const Vec3& raw_mm, const TableFrame& table, double radius_mm) {
  for (const Vec3& corner : table.corners_raw_mm) {
    if (Distance(raw_mm, corner) <= radius_mm) return true;
  }
  return false;
}

bool WaitFrame(Client* client, double timeout_s) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < deadline && !g_stop) {
    if (Ok(client->GetFrame().Result)) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
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

bool HasExactlyOneJsonMember(
    const std::string& text, const std::string& key) {
  const std::regex pattern("\"" + key + "\"\\s*:");
  return std::distance(
      std::sregex_iterator(text.begin(), text.end(), pattern),
      std::sregex_iterator()) == 1;
}

bool JsonNumber(const unitree::common::Any& value, double* out) {
  if (out == nullptr || unitree::common::IsBool(value) ||
      !unitree::common::IsNumber(value)) {
    return false;
  }
  try {
    unitree::common::FromAny(value, *out);
  } catch (const std::exception&) {
    return false;
  }
  return std::isfinite(*out);
}

bool JsonVec3(
    const unitree::common::Any& value, double scale, Vec3* out) {
  if (out == nullptr || !unitree::common::IsJsonArray(value)) return false;
  const auto& array =
      unitree::common::AnyCast<unitree::common::JsonArray>(value);
  if (array.size() != 3) return false;
  double coordinates[3]{};
  for (size_t index = 0; index < array.size(); ++index) {
    if (!JsonNumber(array[index], &coordinates[index])) return false;
  }
  *out = {coordinates[0] * scale,
          coordinates[1] * scale,
          coordinates[2] * scale};
  return std::isfinite(out->x) && std::isfinite(out->y) &&
      std::isfinite(out->z);
}

bool RequiredJsonMember(
    const unitree::common::JsonMap& object,
    const std::string& key,
    const unitree::common::Any** value) {
  const auto found = object.find(key);
  if (found == object.end()) return false;
  *value = &found->second;
  return true;
}

bool LoadTableCalibration(
    const std::string& path,
    TableDimensions* dimensions,
    TableFrame* table,
    std::string* error) {
  if (error != nullptr) error->clear();
  if (dimensions == nullptr || table == nullptr) {
    if (error != nullptr) *error = "invalid calibration output";
    return false;
  }
  std::string text;
  if (!ReadFile(path, &text)) {
    if (error != nullptr) *error = "could not read table calibration";
    return false;
  }
  if (text.find('\\') != std::string::npos) {
    if (error != nullptr) {
      *error = "calibration schema does not permit escaped strings";
    }
    return false;
  }

  static const std::array<const char*, 8> required_keys = {
      "table_length_m", "table_width_m", "height_m_estimate",
      "center_raw_m", "x_axis_raw", "y_axis_raw", "z_axis_raw",
      "corners_raw_m"};
  for (const char* key : required_keys) {
    if (!HasExactlyOneJsonMember(text, key)) {
      if (error != nullptr) {
        *error = "calibration must contain each required member exactly once";
      }
      return false;
    }
  }

  unitree::common::JsonMap object;
  try {
    const unitree::common::Any document =
        unitree::common::FromJsonString(text);
    if (!unitree::common::IsJsonMap(document)) {
      if (error != nullptr) *error = "calibration root must be an object";
      return false;
    }
    object = unitree::common::AnyCast<unitree::common::JsonMap>(document);
  } catch (const std::exception&) {
    if (error != nullptr) *error = "calibration is not valid JSON";
    return false;
  }

  TableDimensions parsed_dimensions;
  TableFrame parsed_table;
  const unitree::common::Any* length = nullptr;
  const unitree::common::Any* width = nullptr;
  const unitree::common::Any* height = nullptr;
  const unitree::common::Any* center = nullptr;
  const unitree::common::Any* x_axis = nullptr;
  const unitree::common::Any* y_axis = nullptr;
  const unitree::common::Any* z_axis = nullptr;
  const unitree::common::Any* corners_value = nullptr;
  if (!RequiredJsonMember(object, "table_length_m", &length) ||
      !RequiredJsonMember(object, "table_width_m", &width) ||
      !RequiredJsonMember(object, "height_m_estimate", &height) ||
      !RequiredJsonMember(object, "center_raw_m", &center) ||
      !RequiredJsonMember(object, "x_axis_raw", &x_axis) ||
      !RequiredJsonMember(object, "y_axis_raw", &y_axis) ||
      !RequiredJsonMember(object, "z_axis_raw", &z_axis) ||
      !RequiredJsonMember(object, "corners_raw_m", &corners_value) ||
      !JsonNumber(*length, &parsed_dimensions.length_m) ||
      !JsonNumber(*width, &parsed_dimensions.width_m) ||
      !JsonNumber(*height, &parsed_dimensions.height_m) ||
      !JsonVec3(*center, 1000.0, &parsed_table.center_raw_mm) ||
      !JsonVec3(*x_axis, 1.0, &parsed_table.x_axis_raw) ||
      !JsonVec3(*y_axis, 1.0, &parsed_table.y_axis_raw) ||
      !JsonVec3(*z_axis, 1.0, &parsed_table.z_axis_raw) ||
      !unitree::common::IsJsonArray(*corners_value)) {
    if (error != nullptr) *error = "calibration has invalid member types";
    return false;
  }

  const auto& corners =
      unitree::common::AnyCast<unitree::common::JsonArray>(*corners_value);

  if (!(std::isfinite(parsed_dimensions.length_m) &&
        parsed_dimensions.length_m > 0.0 &&
        std::isfinite(parsed_dimensions.width_m) &&
        parsed_dimensions.width_m > 0.0 &&
        std::isfinite(parsed_dimensions.height_m) &&
        parsed_dimensions.height_m > 0.0)) {
    if (error != nullptr) *error = "calibration dimensions must be finite and positive";
    return false;
  }
  if (corners.size() != parsed_table.corners_raw_mm.size()) {
    if (error != nullptr) *error = "calibration must contain exactly four corners";
    return false;
  }
  for (size_t index = 0; index < corners.size(); ++index) {
    if (!JsonVec3(
            corners[index], 1000.0,
            &parsed_table.corners_raw_mm[index])) {
      if (error != nullptr) *error = "calibration corner is not a numeric vec3";
      return false;
    }
  }
  const CalibrationValidation validation =
      ValidateTableFrame(&parsed_table, parsed_dimensions, nullptr, 0.050);
  if (!validation.valid) {
    if (error != nullptr) *error = validation.reason;
    return false;
  }
  parsed_table.valid = true;
  *dimensions = parsed_dimensions;
  *table = parsed_table;
  return true;
}

void WriteVec3Json(std::ostream& out, const Vec3& p, double scale) {
  out << "[" << p.x * scale << ", " << p.y * scale << ", " << p.z * scale << "]";
}

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
  if (error != nullptr) error->clear();
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
  output.close();
  const bool complete = static_cast<bool>(output);
  if (!complete || std::rename(temporary.c_str(), path.c_str()) != 0) {
    std::remove(temporary.c_str());
    if (error != nullptr) *error = "could not atomically install calibration";
    return false;
  }
  return true;
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

struct SegmentPoseRaw {
  Vec3 translation_mm;
  Quat quat_xyzw;
  bool valid = false;
  bool occluded = true;
};

struct CalibrationRootTracker {
  SegmentPoseRaw latest;
  bool have_valid_sample = false;
  bool rejected = false;
};

enum class CalibrationFrameDecision {
  FrameReady,
  CleanShutdown,
  RejectTimeout,
};

CalibrationFrameDecision DecideCalibrationFrame(
    bool frame_received,
    bool shutdown_requested,
    CalibrationRootTracker* tracker) {
  if (frame_received) return CalibrationFrameDecision::FrameReady;
  if (shutdown_requested) return CalibrationFrameDecision::CleanShutdown;
  if (tracker != nullptr) tracker->rejected = true;
  return CalibrationFrameDecision::RejectTimeout;
}

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

bool AcceptCalibrationRootSample(
    const SegmentPoseRaw& pose, CalibrationRootTracker* tracker) {
  if (tracker == nullptr || !pose.valid || pose.occluded) {
    if (tracker != nullptr) tracker->rejected = true;
    return false;
  }
  tracker->latest = pose;
  tracker->have_valid_sample = true;
  return true;
}

std::optional<SegmentPoseRaw> FinalCalibrationRootPose(
    const CalibrationRootTracker& tracker) {
  if (tracker.rejected || !tracker.have_valid_sample ||
      !tracker.latest.valid || tracker.latest.occluded) {
    return std::nullopt;
  }
  return tracker.latest;
}

SegmentPoseRaw ReadRootSegmentPose(
    Client* client,
    const std::string& subject,
    const std::string& root_segment) {
  const auto translation =
      client->GetSegmentGlobalTranslation(subject, root_segment);
  const auto rotation =
      client->GetSegmentGlobalRotationQuaternion(subject, root_segment);
  return DecodeSegmentPose(
      Ok(translation.Result), translation.Occluded,
      {translation.Translation[0], translation.Translation[1],
       translation.Translation[2]},
      Ok(rotation.Result), rotation.Occluded,
      {rotation.Rotation[0], rotation.Rotation[1],
       rotation.Rotation[2], rotation.Rotation[3]});
}

std::optional<double> SelectFrameRateHz(
    bool sdk_success, double sdk_hz, double fallback_hz) {
  if (sdk_success && std::isfinite(sdk_hz) && sdk_hz > 0.0) return sdk_hz;
  if (std::isfinite(fallback_hz) && fallback_hz > 0.0) return fallback_hz;
  return std::nullopt;
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

void Multiply3(const double a[3][3], const double b[3][3], double out[3][3]) {
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      out[row][col] = 0.0;
      for (int k = 0; k < 3; ++k) out[row][col] += a[row][k] * b[k][col];
    }
  }
}

void MultiplyByTranspose3(
    const double a[3][3], const double b[3][3], double out[3][3]) {
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      out[row][col] = 0.0;
      for (int k = 0; k < 3; ++k) out[row][col] += a[row][k] * b[col][k];
    }
  }
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

std::string UsageText(const std::string& program) {
  std::ostringstream out;
  out << "Usage: " << program
      << " [--host 192.168.10.1:801] [--base-subject G1Pelvis]"
      << " [--calib-sec 2] [--table-calib table_frame_latest.json]"
      << " [--save-table-calib vicon_table_frame_candidate.json]"
      << " [--ignore-raw-sphere-m x,y,z,r] [--clear-raw-ignore-spheres]"
      << " [--corner-exclusion-radius-mm 50]"
      << " [--vicon-frame-rate-hz FALLBACK_HZ]"
      << " [--duration 0] [--publish]\n"
      << "  --vicon-frame-rate-hz is fallback only when the SDK rate is unavailable.\n";
  return out.str();
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
    else if (key == "--vicon-frame-rate-hz") args->vicon_frame_rate_hz = std::stod(next());
    else if (key == "--corner-exclusion-radius-mm") {
      const std::string value = next();
      const double radius_mm = std::stod(value);
      if (!std::isfinite(radius_mm) || radius_mm < 0.0) {
        std::cerr << "Invalid --corner-exclusion-radius-mm value, expected a finite non-negative radius: "
                  << value << "\n";
        return false;
      }
      args->corner_exclusion_radius_mm = radius_mm;
    }
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
    else if (key == "--publish") args->publish = true;
    else if (key == "--no-publish") args->publish = false;
    else if (key == "--help" || key == "-h") {
      std::cout << UsageText(argv[0]);
      std::exit(0);
    } else {
      std::cerr << "Unknown argument: " << key << "\n";
      return false;
    }
  }
  if (args->base_subject != kBaseSubject) {
    std::cerr << "--base-subject must be exactly " << kBaseSubject << "\n";
    return false;
  }
  return true;
}

void AddClusterSample(
    std::vector<Cluster>* clusters,
    const Vec3& p,
    double gate_mm,
    int calibration_frame = -1) {
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
    c.observed_frames = 1;
    c.last_observed_frame = calibration_frame;
    c.samples.push_back(p);
    clusters->push_back(c);
    return;
  }

  Cluster& c = (*clusters)[best];
  c.sum = c.sum + p;
  c.sum_sq = c.sum_sq + Vec3{p.x * p.x, p.y * p.y, p.z * p.z};
  c.count += 1;
  c.mean = c.sum / static_cast<double>(c.count);
  if (calibration_frame < 0 || calibration_frame != c.last_observed_frame) {
    ++c.observed_frames;
    c.last_observed_frame = calibration_frame;
  }
  c.samples.push_back(p);
}

ClusterStability EvaluateClusterStability(
    const Cluster& cluster, int accepted_frames) {
  ClusterStability result;
  if (accepted_frames <= 0 ||
      cluster.samples.empty() ||
      cluster.samples.size() != static_cast<size_t>(cluster.count) ||
      cluster.observed_frames < 0) {
    return result;
  }
  result.coverage = static_cast<double>(cluster.observed_frames) /
                    static_cast<double>(accepted_frames);

  double sum_squared_deviation = 0.0;
  double max_deviation = 0.0;
  for (const Vec3& sample : cluster.samples) {
    const double deviation = Distance(sample, cluster.mean);
    if (!std::isfinite(deviation)) return result;
    sum_squared_deviation += deviation * deviation;
    max_deviation = std::max(max_deviation, deviation);
  }
  result.rms_deviation_mm = std::sqrt(
      sum_squared_deviation / static_cast<double>(cluster.samples.size()));
  result.max_deviation_mm = max_deviation;
  result.stable = cluster.count >= 5 &&
      std::isfinite(result.coverage) && result.coverage >= 0.8 &&
      std::isfinite(result.rms_deviation_mm) &&
      std::isfinite(result.max_deviation_mm) &&
      result.rms_deviation_mm <= 5.0 &&
      result.max_deviation_mm <= 15.0;
  return result;
}

std::vector<Cluster> StableCalibrationClusters(
    const std::vector<Cluster>& clusters, int accepted_frames) {
  std::vector<Cluster> stable;
  for (const Cluster& cluster : clusters) {
    if (EvaluateClusterStability(cluster, accepted_frames).stable) {
      stable.push_back(cluster);
    }
  }
  return stable;
}

Vec3 RawToTableWorld(
    const Vec3& raw_mm, const TableFrame& table, const Args& args);

std::array<Vec3, 4> ExpectedWorldCorners(const TableDimensions& dims) {
  return {{{0.0, -0.5 * dims.width_m, dims.height_m},
           {0.0, 0.5 * dims.width_m, dims.height_m},
           {dims.length_m, -0.5 * dims.width_m, dims.height_m},
           {dims.length_m, 0.5 * dims.width_m, dims.height_m}}};
}

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
  if (frame == nullptr ||
      !(std::isfinite(dims.length_m) && dims.length_m > 0.0 &&
        std::isfinite(dims.width_m) && dims.width_m > 0.0 &&
        std::isfinite(dims.height_m) && dims.height_m > 0.0) ||
      !std::isfinite(allowed_max_error_m) || allowed_max_error_m < 0.0) {
    result.reason = "invalid table validation inputs";
    return result;
  }
  if (!finite_vec(frame->center_raw_mm) ||
      !finite_vec(frame->x_axis_raw) || !finite_vec(frame->y_axis_raw) ||
      !finite_vec(frame->z_axis_raw)) {
    result.reason = "non-finite table frame";
    return result;
  }
  for (const Vec3& corner : frame->corners_raw_mm) {
    if (!finite_vec(corner)) {
      result.reason = "non-finite table corner";
      return result;
    }
  }
  if (std::abs(Norm(frame->x_axis_raw) - 1.0) > 1.0e-6 ||
      std::abs(Norm(frame->y_axis_raw) - 1.0) > 1.0e-6 ||
      std::abs(Norm(frame->z_axis_raw) - 1.0) > 1.0e-6 ||
      std::abs(Dot(frame->x_axis_raw, frame->y_axis_raw)) > 1.0e-6 ||
      std::abs(Dot(frame->x_axis_raw, frame->z_axis_raw)) > 1.0e-6 ||
      std::abs(Dot(frame->y_axis_raw, frame->z_axis_raw)) > 1.0e-6 ||
      Dot(Cross(frame->x_axis_raw, frame->y_axis_raw),
          frame->z_axis_raw) < 1.0 - 1.0e-6) {
    result.reason = "table basis is not orthonormal and right handed";
    return result;
  }
  if (Dot(frame->z_axis_raw, kTrackerNativeUpRaw) <= 0.0) {
    result.reason = "table positive z does not align with Tracker positive z";
    return result;
  }

  Args transform_args;
  transform_args.table_length_m = dims.length_m;
  transform_args.table_width_m = dims.width_m;
  transform_args.table_height_m = dims.height_m;
  if (pelvis_raw_mm != nullptr) {
    if (!finite_vec(*pelvis_raw_mm)) {
      result.reason = "non-finite G1Pelvis position";
      return result;
    }
    const Vec3 pelvis_world =
        RawToTableWorld(*pelvis_raw_mm, *frame, transform_args);
    if (!finite_vec(pelvis_world) || pelvis_world.x >= 0.0) {
      result.reason = "G1Pelvis is not on the robot side";
      return result;
    }
  }

  const auto expected = ExpectedWorldCorners(dims);
  std::array<size_t, 4> permutation = {0, 1, 2, 3};
  do {
    double max_error = 0.0;
    double sum_squared = 0.0;
    bool finite_assignment = true;
    for (size_t raw_index = 0; raw_index < 4; ++raw_index) {
      const Vec3 world = RawToTableWorld(
          frame->corners_raw_mm[raw_index], *frame, transform_args);
      const double error =
          Distance(world, expected[permutation[raw_index]]);
      if (!finite_vec(world) || !std::isfinite(error)) {
        finite_assignment = false;
        break;
      }
      max_error = std::max(max_error, error);
      sum_squared += error * error;
    }
    if (!finite_assignment) continue;
    const double rms = std::sqrt(sum_squared / 4.0);
    if (!std::isfinite(rms)) continue;
    if (max_error < result.max_corner_error_m ||
        (max_error == result.max_corner_error_m &&
         rms < result.rms_corner_error_m)) {
      result.max_corner_error_m = max_error;
      result.rms_corner_error_m = rms;
      result.assignment = permutation;
    }
  } while (std::next_permutation(permutation.begin(), permutation.end()));

  const double comparison_tolerance =
      64.0 * std::numeric_limits<double>::epsilon() *
      std::max(1.0, std::abs(allowed_max_error_m));
  if (!std::isfinite(result.max_corner_error_m) ||
      result.max_corner_error_m >
          allowed_max_error_m + comparison_tolerance) {
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
  if (!(std::isfinite(pelvis_raw_mm.x) &&
        std::isfinite(pelvis_raw_mm.y) &&
        std::isfinite(pelvis_raw_mm.z))) {
    result.validation.reason = "non-finite G1Pelvis position";
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
       corners_raw_mm[2] + corners_raw_mm[3]) /
      4.0;
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
  Vec3 x =
      NormalizeVec(eigenvectors[2] - Dot(eigenvectors[2], z) * z);
  const Vec3 pelvis_delta = pelvis_raw_mm - frame.center_raw_mm;
  if (Dot(z, kTrackerNativeUpRaw) < 0.0) z = -1.0 * z;
  if (Dot(pelvis_delta, x) > 0.0) x = -1.0 * x;
  Vec3 y = NormalizeVec(Cross(z, x));
  x = NormalizeVec(Cross(y, z));
  frame.x_axis_raw = x;
  frame.y_axis_raw = y;
  frame.z_axis_raw = z;
  frame.valid = true;
  result.frame = frame;
  result.validation =
      ValidateTableFrame(&result.frame, dims, &pelvis_raw_mm, 0.050);
  result.frame.valid = result.validation.valid;
  return result;
}

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

Vec3 RawToTableWorld(const Vec3& raw_mm, const TableFrame& table, const Args& args) {
  const Vec3 rel = raw_mm - table.center_raw_mm;
  return {
      args.table_length_m * 0.5 + Dot(rel, table.x_axis_raw) * 0.001,
      Dot(rel, table.y_axis_raw) * 0.001,
      args.table_height_m + Dot(rel, table.z_axis_raw) * 0.001,
  };
}

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

std::vector<BallCandidate> BallCandidates(
    const std::vector<Vec3>& unlabeled_raw,
    const TableFrame& table,
    const Args& args,
    BallSelectionDiagnostics* diagnostics = nullptr) {
  std::vector<BallCandidate> candidates;
  for (const auto& raw : unlabeled_raw) {
    if (IgnoredRawMarker(raw, args)) {
      if (diagnostics != nullptr) ++diagnostics->raw_ignored_count;
      continue;
    }
    if (IsSavedTableCorner(raw, table, args.corner_exclusion_radius_mm)) {
      if (diagnostics != nullptr) ++diagnostics->corner_ignored_count;
      continue;
    }
    const Vec3 world = RawToTableWorld(raw, table, args);
    candidates.push_back(BallCandidate{raw, world});
  }
  if (diagnostics != nullptr) diagnostics->candidate_count = candidates.size();
  return candidates;
}

bool SelectBall(
    const std::vector<Vec3>& unlabeled_raw,
    const TableFrame& table,
    const Args& args,
    BallTrackState* track,
    int64_t frame_number,
    double source_frame_rate_hz,
    Vec3* ball_raw,
    Vec3* ball_world,
    BallSelectionDiagnostics* diagnostics = nullptr) {
  if (diagnostics != nullptr) *diagnostics = BallSelectionDiagnostics{};

  const std::vector<BallCandidate> candidates = BallCandidates(unlabeled_raw, table, args, diagnostics);
  if (candidates.empty()) {
    if (diagnostics != nullptr) diagnostics->reason = BallRejectReason::NoCandidates;
    return false;
  }

  size_t best = 0;
  if (track->have_position) {
    const double dt = FrameDeltaSeconds(
        frame_number, track->frame_number, source_frame_rate_hz);
    const Vec3 predicted = track->have_velocity ? track->position + dt * track->velocity : track->position;
    if (diagnostics != nullptr) {
      diagnostics->tracking_dt_s = dt;
    }
    double best_pred_dist = std::numeric_limits<double>::infinity();
    double best_direct_speed = 0.0;
    for (size_t i = 0; i < candidates.size(); ++i) {
      const double direct_speed = dt > 1.0e-6 ? Distance(candidates[i].world, track->position) / dt : 0.0;
      const double pred_dist = Distance(candidates[i].world, predicted);
      if (pred_dist < best_pred_dist) {
        best = i;
        best_pred_dist = pred_dist;
        best_direct_speed = direct_speed;
      }
    }
    if (diagnostics != nullptr) {
      diagnostics->tracking_best_pred_dist_m = best_pred_dist;
      diagnostics->tracking_best_direct_speed_mps = best_direct_speed;
    }
  }
  *ball_raw = candidates[best].raw;
  *ball_world = candidates[best].world;
  if (diagnostics != nullptr) diagnostics->reason = BallRejectReason::None;
  return true;
}

void UpdateBallTrack(
    BallTrackState* track,
    const Vec3& ball_world,
    int64_t frame_number,
    double source_frame_rate_hz) {
  if (track->have_position) {
    const double dt = FrameDeltaSeconds(
        frame_number, track->frame_number, source_frame_rate_hz);
    if (dt > 1.0e-6) {
      track->velocity = (ball_world - track->position) / dt;
      track->have_velocity = true;
    }
  }
  track->position = ball_world;
  track->frame_number = frame_number;
  track->have_position = true;
  track->missed_frames = 0;
}

void NoteBallMiss(BallTrackState* track, const Args& args) {
  (void)args;
  if (!track->have_position) return;
  ++track->missed_frames;
}

void FillMessage(
    lcm_types::transformation_t* msg,
    const std::string& name,
    const Vec3& pos,
    const Quat& quat,
    int64_t frame_number,
    const Args& args,
    double source_frame_rate_hz,
    bool valid,
    bool occluded) {
  (void)args;
  msg->name = name;
  msg->vicon_frame_number = frame_number;
  msg->vicon_time_s = source_frame_rate_hz > 0.0
                          ? static_cast<double>(frame_number) / source_frame_rate_hz
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

struct RuntimeFrameWaitDecision {
  bool exit_loop = false;
  bool exit_nonzero = false;
  bool publish_invalid_base = false;
  lcm_types::transformation_t invalid_base;
};

RuntimeFrameWaitDecision DecideRuntimeFrameWait(
    bool frame_received,
    bool shutdown_requested,
    bool publishing_enabled,
    int64_t last_frame_number,
    const Args& args,
    double source_frame_rate_hz) {
  RuntimeFrameWaitDecision decision;
  if (frame_received) return decision;
  decision.exit_loop = true;
  if (shutdown_requested) return decision;

  decision.exit_nonzero = true;
  FillMessage(
      &decision.invalid_base, kBaseSubject, Vec3{}, Quat{},
      last_frame_number, args, source_frame_rate_hz, false, true);
  decision.publish_invalid_base = publishing_enabled;
  return decision;
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

#ifndef VICON_TABLE_LCM_BRIDGE_TESTING
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

  client.EnableSegmentData();
  client.EnableUnlabeledMarkerData();

  if (!WaitFrame(&client, 5.0)) {
    std::cerr << "Timed out waiting for initial Vicon frame.\n";
    return 1;
  }

  const auto sdk_rate = client.GetFrameRate();
  const auto selected_rate = SelectFrameRateHz(
      Ok(sdk_rate.Result), sdk_rate.FrameRateHz, args.vicon_frame_rate_hz);
  if (!selected_rate.has_value()) {
    std::cerr << "No finite positive Vicon frame rate or fallback\n";
    return 1;
  }
  const double source_frame_rate_hz = *selected_rate;
  const bool sdk_rate_valid = Ok(sdk_rate.Result) &&
      std::isfinite(sdk_rate.FrameRateHz) && sdk_rate.FrameRateHz > 0.0;
  std::cout << "Vicon frame rate=" << source_frame_rate_hz << " Hz source="
            << (sdk_rate_valid ? "sdk" : "fallback") << "\n";

  const auto root = client.GetSubjectRootSegmentName(args.base_subject);
  if (!Ok(root.Result) || std::string(root.SegmentName).empty()) {
    std::cerr << "Could not resolve root segment for " << args.base_subject << "\n";
    return 1;
  }
  const std::string base_segment = root.SegmentName;
  std::cout << "Using root segment for " << args.base_subject << ": "
            << base_segment << "\n";

  SegmentPoseRaw initial_base_raw;
  const auto initial_base_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!g_stop && std::chrono::steady_clock::now() < initial_base_deadline) {
    initial_base_raw =
        ReadRootSegmentPose(&client, args.base_subject, base_segment);
    if (initial_base_raw.valid) break;
    const double remaining_s = std::chrono::duration<double>(
        initial_base_deadline - std::chrono::steady_clock::now()).count();
    if (remaining_s <= 0.0) break;
    WaitFrame(&client, std::min(1.0, remaining_s));
  }
  if (!initial_base_raw.valid) {
    std::cerr << "Could not read a valid root pose for " << args.base_subject
              << " within 5 seconds\n";
    return 1;
  }
  const Quat initial_base_raw_q = initial_base_raw.quat_xyzw;

  TableDimensions dimensions{
      args.table_length_m, args.table_width_m, args.table_height_m};
  TableFrame table;
  if (!args.table_calib_path.empty()) {
    std::cout << "Loading table calibration from " << args.table_calib_path << " ...\n";
    std::string error;
    if (!LoadTableCalibration(
            args.table_calib_path, &dimensions, &table, &error)) {
      std::cerr << "Table calibration rejected: " << error << "\n";
      return 1;
    }
    args.table_length_m = dimensions.length_m;
    args.table_width_m = dimensions.width_m;
    args.table_height_m = dimensions.height_m;
    const CalibrationValidation current_world_validation =
        ValidateTableFrame(
            &table, dimensions, &initial_base_raw.translation_mm, 0.050);
    if (!current_world_validation.valid) {
      std::cerr << "Table calibration rejected for current G1Pelvis: "
                << current_world_validation.reason << "\n";
      return 1;
    }
  } else {
    std::cout << "Calibrating table from unlabeled markers for " << args.calib_s << " s ...\n";
    std::vector<Cluster> clusters;
    CalibrationRootTracker calibration_root;
    const auto calib_start = std::chrono::steady_clock::now();
    int calib_frames = 0;
    while (!g_stop) {
      const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - calib_start).count();
      if (elapsed >= args.calib_s) break;
      const CalibrationFrameDecision frame_decision = DecideCalibrationFrame(
          WaitFrame(&client, 1.0), g_stop != 0, &calibration_root);
      if (frame_decision == CalibrationFrameDecision::CleanShutdown) {
        client.Disconnect();
        return 0;
      }
      if (frame_decision == CalibrationFrameDecision::RejectTimeout) {
        std::cerr << "Table calibration rejected: no Vicon frame received "
                     "for 1 second\n";
        return 1;
      }
      ++calib_frames;
      const SegmentPoseRaw calibration_pose =
          ReadRootSegmentPose(&client, args.base_subject, base_segment);
      if (!AcceptCalibrationRootSample(calibration_pose, &calibration_root)) {
        std::cerr << "Table calibration rejected: current G1Pelvis root pose "
                     "is invalid or occluded\n";
        return 1;
      }
      for (const auto& p : UnlabeledMarkers(&client)) {
        if (IgnoredRawMarker(p, args)) continue;
        AddClusterSample(&clusters, p, 80.0, calib_frames);
      }
    }

    if (g_stop) {
      client.Disconnect();
      return 0;
    }

    const auto final_calibration_pose =
        FinalCalibrationRootPose(calibration_root);
    if (!final_calibration_pose.has_value()) {
      std::cerr << "Table calibration rejected: no current valid G1Pelvis "
                   "root pose was sampled\n";
      return 1;
    }

    std::cout << "Calibration unlabeled clusters (" << clusters.size()
              << "):\n";
    for (size_t i = 0; i < clusters.size(); ++i) {
      const auto& c = clusters[i];
      const ClusterStability stats =
          EvaluateClusterStability(c, calib_frames);
      std::cout << std::fixed << std::setprecision(3)
                << "  #" << i << " count=" << c.count
                << " coverage=" << stats.coverage
                << " rms_mm=" << stats.rms_deviation_mm
                << " max_mm=" << stats.max_deviation_mm
                << " stable=" << stats.stable
                << " raw_mm=[" << c.mean.x << ", " << c.mean.y << ", "
                << c.mean.z << "]\n";
    }

    clusters = StableCalibrationClusters(clusters, calib_frames);
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

    if (clusters.size() != 4) {
      std::cerr << "Expected exactly 4 stable unlabeled table corners, got "
                << clusters.size() << "\n";
      return 1;
    }
    const CalibrationResult calibrated = BuildTableFrame(
        clusters, final_calibration_pose->translation_mm,
        {args.table_length_m, args.table_width_m, args.table_height_m});
    if (!calibrated.validation.valid) {
      std::cerr << "Table calibration rejected: "
                << calibrated.validation.reason << "\n";
      return 1;
    }
    table = calibrated.frame;
    if (!args.save_table_calib_path.empty()) {
      std::string error;
      if (!SaveTableCalibrationAtomic(
              args.save_table_calib_path, dimensions, table, &error)) {
        std::cerr << "Could not save table calibration: " << error << "\n";
        return 1;
      }
      std::cout << "Saved table calibration candidate to "
                << args.save_table_calib_path << "\n";
    }
  }
  if (!table.valid) {
    std::cerr << "Could not infer table corners from unlabeled clusters.\n";
    return 1;
  }

  std::cout << std::fixed << std::setprecision(4)
            << "Table corner max error=" << table.corner_max_error_m << " m"
            << " rms error=" << table.corner_rms_error_m << " m"
            << " center_raw_m=[" << table.center_raw_mm.x * 0.001 << ", "
            << table.center_raw_mm.y * 0.001 << ", " << table.center_raw_mm.z * 0.001 << "]"
            << " x_axis_raw=[" << table.x_axis_raw.x << ", " << table.x_axis_raw.y << ", " << table.x_axis_raw.z << "]"
            << " y_axis_raw=[" << table.y_axis_raw.x << ", " << table.y_axis_raw.y << ", " << table.y_axis_raw.z << "]"
            << " z_axis_raw=[" << table.z_axis_raw.x << ", " << table.z_axis_raw.y << ", " << table.z_axis_raw.z << "]\n";
  std::cout << "Table-world convention: robot-side edge x=0.0000 m, far edge x="
            << args.table_length_m << " m, expected " << args.base_subject
            << " x ~= -" << args.expected_base_edge_distance_m << " m\n";
  if (args.table_calib_path.empty()) {
    for (size_t i = 0; i < table.corners_raw_mm.size(); ++i) {
      const Vec3 w = RawToTableWorld(table.corners_raw_mm[i], table, args);
      std::cout << "  table_corner_" << i << "_world_m=[" << w.x << ", " << w.y << ", " << w.z << "]\n";
    }
  }

  const BasePoseWorld initial_base_world =
      TransformRootPose(initial_base_raw, initial_base_raw_q, table, args);
  if (!initial_base_world.valid) {
    std::cerr << "Could not transform initial root pose for "
              << args.base_subject << "\n";
    return 1;
  }
  std::cout << "Base initial direct_world_m=[" << initial_base_world.position_m.x
            << ", " << initial_base_world.position_m.y << ", "
            << initial_base_world.position_m.z << "]\n";

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
  const auto latest_source_frame = client.GetFrameNumber();
  int64_t last_frame_number = Ok(latest_source_frame.Result)
      ? static_cast<int64_t>(latest_source_frame.FrameNumber)
      : 0;
  int runtime_exit_code = 0;
  BallTrackState ball_track;
  std::array<unsigned long long, kBallRejectReasonCount> ball_reject_counts{};

  while (!g_stop) {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - start).count();
    if (args.duration_s > 0.0 && elapsed >= args.duration_s) break;
    const bool frame_received = WaitFrame(&client, 2.0);
    const RuntimeFrameWaitDecision wait_decision = DecideRuntimeFrameWait(
        frame_received, g_stop != 0, args.publish, last_frame_number,
        args, source_frame_rate_hz);
    if (wait_decision.exit_loop) {
      if (wait_decision.exit_nonzero) {
        std::cerr << "Vicon stream timeout: no frame received for 2 seconds; "
                     "exiting fail-closed.\n";
        if (wait_decision.publish_invalid_base) {
          PublishTransform(
              &lcm, "vicon_state_data", wait_decision.invalid_base);
        }
        runtime_exit_code = 1;
      }
      break;
    }
    ++frames;
    const int64_t frame_number = static_cast<int64_t>(client.GetFrameNumber().FrameNumber);
    last_frame_number = frame_number;

    const SegmentPoseRaw base_raw =
        ReadRootSegmentPose(&client, args.base_subject, base_segment);
    const BasePoseWorld transformed =
        TransformRootPose(base_raw, initial_base_raw_q, table, args);
    const Vec3 base_world = transformed.valid ? transformed.position_m : Vec3{};
    const Quat base_world_q =
        transformed.valid ? transformed.relative_yaw_xyzw : Quat{};
    const bool base_valid = transformed.valid;

    const std::vector<Vec3> unlabeled = UnlabeledMarkers(&client);
    Vec3 ball_raw;
    Vec3 ball_world;
    BallSelectionDiagnostics ball_diag;
    bool ball_valid = false;
    if (!base_valid) {
      ball_diag.reason = BallRejectReason::BaseInvalid;
    } else {
      ball_valid = SelectBall(
          unlabeled, table, args, &ball_track, frame_number,
          source_frame_rate_hz, &ball_raw, &ball_world, &ball_diag);
    }
    if (ball_valid) {
      UpdateBallTrack(
          &ball_track, ball_world, frame_number, source_frame_rate_hz);
    } else {
      NoteBallMiss(&ball_track, args);
      ++ball_reject_counts[BallRejectReasonIndex(ball_diag.reason)];
    }

    if (args.publish) {
      lcm_types::transformation_t msg;
      FillMessage(
          &msg, kBaseSubject, base_world, base_world_q, frame_number,
          args, source_frame_rate_hz, base_valid, !base_valid);
      PublishTransform(&lcm, "vicon_state_data", msg);
      if (ball_valid) {
        FillMessage(
            &msg, "ball", ball_world, Quat{}, frame_number,
            args, source_frame_rate_hz, true, false);
        PublishTransform(&lcm, "vicon_state_data", msg);
      }
      FillMessage(
          &msg, "table",
          Vec3{args.table_length_m * 0.5, 0.0, args.table_height_m},
          Quat{}, frame_number, args, source_frame_rate_hz, true, false);
      PublishTransform(&lcm, "vicon_state_data", msg);
    }

    const double print_period = 1.0 / std::max(args.print_hz, 1.0e-6);
    if (std::chrono::duration<double>(now - last_print).count() >= print_period) {
      const double hz = frames / std::max(elapsed, 1.0e-6);
      std::cout << std::fixed << std::setprecision(4)
                << "frame=" << frame_number
                << " hz=" << std::setprecision(1) << hz
                << " base_valid=" << base_valid
                << " base_occluded=" << base_raw.occluded
                << " base_raw_m=[" << std::setprecision(4)
                << base_raw.translation_mm.x * 0.001 << ", "
                << base_raw.translation_mm.y * 0.001 << ", "
                << base_raw.translation_mm.z * 0.001 << "]"
                << " base_world_m=["
                << base_world.x << ", " << base_world.y << ", " << base_world.z << "]"
                << " base_edge_error_m=" << base_world.x + args.expected_base_edge_distance_m
                << " base_quat_xyzw=[" << base_world_q.x << ", " << base_world_q.y << ", "
                << base_world_q.z << ", " << base_world_q.w << "]"
                << " unlabeled_count=" << unlabeled.size()
                << " raw_ignored_count=" << ball_diag.raw_ignored_count
                << " corner_ignored_count=" << ball_diag.corner_ignored_count
                << " ball_candidates=" << ball_diag.candidate_count
                << " ball_valid=" << ball_valid
                << " ball_missed_frames=" << ball_track.missed_frames;
      if (ball_valid) {
        std::cout << " ball_world_m=[" << ball_world.x << ", " << ball_world.y << ", " << ball_world.z << "]"
                  << " ball_raw_m=[" << ball_raw.x * 0.001 << ", " << ball_raw.y * 0.001 << ", " << ball_raw.z * 0.001 << "]";
      } else {
        std::cout << " ball_reject=" << BallRejectReasonName(ball_diag.reason)
                  << " ball_reject_counts=";
        PrintRejectCounts(std::cout, ball_reject_counts);
        if (std::isfinite(ball_diag.tracking_best_pred_dist_m)) {
          std::cout << " track_dt_s=" << ball_diag.tracking_dt_s
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
  return runtime_exit_code;
}
#endif
