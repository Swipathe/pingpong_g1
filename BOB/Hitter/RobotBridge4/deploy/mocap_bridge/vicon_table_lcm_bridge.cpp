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
#include <filesystem>
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

constexpr const char* kBaseSubject = "G2Pelvis";
constexpr const char* kDefaultTrackerSubject = "G1Pelvis";
constexpr const char* kViconV2Channel = "vicon_state_data_v2";
constexpr double kDefaultViconFrameRateHz = 300.0;
constexpr double kBallTrackingFarEdgeMarginM = 0.40;
constexpr const char* kTableCalibrationFormat =
    "robotbridge2_chingmu_table_frame_v1";
constexpr const char* kPelvisExtrinsicsFormat =
    "robotbridge2_chingmu_pelvis_orientation_v1";
constexpr const char* kPelvisRotationConvention =
    "R_world_pelvis = R_world_rigid @ R_rigid_from_pelvis";
constexpr const char* kPelvisTranslationConvention =
    "p_world_pelvis = p_world_rigid + R_world_pelvis @ "
    "translation_rigid_origin_to_pelvis_origin_pelvis_m";

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
  std::string tracker_subject = kDefaultTrackerSubject;
  std::string base_subject = kBaseSubject;
  std::string lcm_url = "udpm://239.255.76.67:7667?ttl=255";
  std::string table_calib_path;
  std::string pelvis_extrinsics_path;
  std::string save_table_calib_path;
  double duration_s = 0.0;
  double print_hz = 1.0;
  double calib_s = 2.0;
  double table_length_m = 2.730738;
  double table_width_m = 1.512451;
  double table_height_m = 0.760000;
  double expected_base_edge_distance_m = 0.40;
  double vicon_frame_rate_hz = 300.0;
  double corner_exclusion_radius_mm = 50.0;
  std::vector<RawIgnoreSphere> raw_ignore_spheres;
  bool publish = false;
  std::string channel = kViconV2Channel;
  double ball_track_association_radius_m = 0.35;
  double ball_track_end_timeout_s = 0.25;
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

struct PelvisExtrinsics {
  bool valid = false;
  Quat rotation_rigid_from_pelvis;
  Vec3 translation_pelvis_m;
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

enum class BallTrackPhase { Inactive, Active, MissingGrace };

enum class BallTrackError { None, AllocatorExhausted };

const char* BallTrackPhaseName(BallTrackPhase phase) {
  switch (phase) {
    case BallTrackPhase::Inactive:
      return "inactive";
    case BallTrackPhase::Active:
      return "active";
    case BallTrackPhase::MissingGrace:
      return "missing_grace";
  }
  return "unknown";
}

const char* BallTrackErrorName(BallTrackError error) {
  switch (error) {
    case BallTrackError::None:
      return "none";
    case BallTrackError::AllocatorExhausted:
      return "allocator_exhausted";
  }
  return "unknown";
}

struct BallTrackState {
  BallTrackPhase phase = BallTrackPhase::Inactive;
  int64_t track_id = 0;
  int64_t last_allocated_track_id = 0;
  Vec3 position;
  Vec3 velocity;
  bool have_velocity = false;
  int64_t last_observed_frame = 0;
  double last_observed_source_time_s = 0.0;
};

struct BallTrackUpdate {
  BallTrackPhase phase = BallTrackPhase::Inactive;
  int64_t track_id = 0;
  Vec3 position;
  bool publish_valid = false;
  bool publish_end = false;
  BallTrackError error = BallTrackError::None;
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

bool JsonQuat(const unitree::common::Any& value, Quat* out) {
  if (out == nullptr || !unitree::common::IsJsonArray(value)) return false;
  const auto& array =
      unitree::common::AnyCast<unitree::common::JsonArray>(value);
  if (array.size() != 4) return false;
  double components[4]{};
  for (size_t index = 0; index < array.size(); ++index) {
    if (!JsonNumber(array[index], &components[index])) return false;
  }
  const double norm = std::sqrt(
      components[0] * components[0] + components[1] * components[1] +
      components[2] * components[2] + components[3] * components[3]);
  if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1.0e-6) return false;
  *out = {components[0], components[1], components[2], components[3]};
  return true;
}

bool JsonString(const unitree::common::Any& value, std::string* out) {
  if (out == nullptr || !unitree::common::IsString(value)) return false;
  try {
    unitree::common::FromAny(value, *out);
  } catch (const std::exception&) {
    return false;
  }
  return true;
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

bool LoadPelvisExtrinsics(
    const std::string& path,
    const std::string& base_subject,
    PelvisExtrinsics* extrinsics,
    std::string* error) {
  if (error != nullptr) error->clear();
  if (extrinsics == nullptr) {
    if (error != nullptr) *error = "invalid pelvis extrinsics output";
    return false;
  }
  std::string text;
  if (!ReadFile(path, &text)) {
    if (error != nullptr) *error = "could not read pelvis extrinsics";
    return false;
  }
  if (text.find('\\') != std::string::npos) {
    if (error != nullptr) {
      *error = "pelvis extrinsics schema does not permit escaped strings";
    }
    return false;
  }
  static const std::array<const char*, 8> required_keys = {
      "format", "base_subject", "quaternion_convention",
      "rotation_convention", "quaternion_rigid_from_pelvis_xyzw",
      "translation_convention",
      "translation_rigid_origin_to_pelvis_origin_pelvis_m", "sample_count"};
  for (const char* key : required_keys) {
    if (!HasExactlyOneJsonMember(text, key)) {
      if (error != nullptr) {
        *error = std::string("pelvis extrinsics must contain required member ") +
            key + " exactly once";
      }
      return false;
    }
  }
  for (const char* key :
       {"source_duration_s", "position_rms_m", "angular_rms_deg"}) {
    if (!HasExactlyOneJsonMember(text, key)) {
      if (error != nullptr) {
        *error = std::string("pelvis extrinsics must contain required member ") +
            key + " exactly once";
      }
      return false;
    }
  }

  unitree::common::JsonMap object;
  try {
    const unitree::common::Any document =
        unitree::common::FromJsonString(text);
    if (!unitree::common::IsJsonMap(document)) {
      if (error != nullptr) *error = "pelvis extrinsics root must be an object";
      return false;
    }
    object = unitree::common::AnyCast<unitree::common::JsonMap>(document);
  } catch (const std::exception&) {
    if (error != nullptr) *error = "pelvis extrinsics is not valid JSON";
    return false;
  }

  const unitree::common::Any* format_value = nullptr;
  const unitree::common::Any* subject_value = nullptr;
  const unitree::common::Any* quaternion_convention_value = nullptr;
  const unitree::common::Any* rotation_convention_value = nullptr;
  const unitree::common::Any* quaternion_value = nullptr;
  const unitree::common::Any* translation_convention_value = nullptr;
  const unitree::common::Any* translation_value = nullptr;
  const unitree::common::Any* sample_count_value = nullptr;
  const unitree::common::Any* duration_value = nullptr;
  const unitree::common::Any* position_rms_value = nullptr;
  const unitree::common::Any* angular_rms_value = nullptr;
  if (!RequiredJsonMember(object, "format", &format_value) ||
      !RequiredJsonMember(object, "base_subject", &subject_value) ||
      !RequiredJsonMember(
          object, "quaternion_convention", &quaternion_convention_value) ||
      !RequiredJsonMember(
          object, "rotation_convention", &rotation_convention_value) ||
      !RequiredJsonMember(
          object, "quaternion_rigid_from_pelvis_xyzw", &quaternion_value) ||
      !RequiredJsonMember(
          object, "translation_convention", &translation_convention_value) ||
      !RequiredJsonMember(
          object, "translation_rigid_origin_to_pelvis_origin_pelvis_m",
          &translation_value) ||
      !RequiredJsonMember(object, "sample_count", &sample_count_value) ||
      !RequiredJsonMember(object, "source_duration_s", &duration_value) ||
      !RequiredJsonMember(object, "position_rms_m", &position_rms_value) ||
      !RequiredJsonMember(object, "angular_rms_deg", &angular_rms_value)) {
    if (error != nullptr) *error = "pelvis extrinsics is missing fields";
    return false;
  }

  std::string format;
  std::string subject;
  std::string quaternion_convention;
  std::string rotation_convention;
  std::string translation_convention;
  PelvisExtrinsics parsed;
  double sample_count = 0.0;
  double source_duration_s = 0.0;
  double position_rms_m = 0.0;
  double angular_rms_deg = 0.0;
  if (!JsonString(*format_value, &format) ||
      !JsonString(*subject_value, &subject) ||
      !JsonString(*quaternion_convention_value, &quaternion_convention) ||
      !JsonString(*rotation_convention_value, &rotation_convention) ||
      !JsonQuat(*quaternion_value, &parsed.rotation_rigid_from_pelvis) ||
      !JsonString(*translation_convention_value, &translation_convention) ||
      !JsonVec3(*translation_value, 1.0, &parsed.translation_pelvis_m) ||
      !JsonNumber(*sample_count_value, &sample_count) ||
      !JsonNumber(*duration_value, &source_duration_s) ||
      !JsonNumber(*position_rms_value, &position_rms_m) ||
      !JsonNumber(*angular_rms_value, &angular_rms_deg)) {
    if (error != nullptr) *error = "pelvis extrinsics has invalid member types";
    return false;
  }
  if (format != kPelvisExtrinsicsFormat || subject != base_subject ||
      quaternion_convention != "xyzw" ||
      rotation_convention != kPelvisRotationConvention ||
      translation_convention != kPelvisTranslationConvention) {
    if (error != nullptr) *error = "pelvis extrinsics conventions do not match";
    return false;
  }
  if (std::floor(sample_count) != sample_count || sample_count < 30.0 ||
      source_duration_s < 1.0 || position_rms_m < 0.0 ||
      position_rms_m > 0.002 || angular_rms_deg < 0.0 ||
      angular_rms_deg > 0.3 || Norm(parsed.translation_pelvis_m) > 0.5) {
    if (error != nullptr) *error = "pelvis extrinsics quality checks failed";
    return false;
  }
  parsed.valid = true;
  *extrinsics = parsed;
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

  static const std::array<const char*, 7> required_keys = {
      "table_length_m", "table_width_m",
      "center_raw_m", "x_axis_raw", "y_axis_raw", "z_axis_raw",
      "corners_raw_m"};
  for (const char* key : required_keys) {
    if (!HasExactlyOneJsonMember(text, key)) {
      if (error != nullptr) {
        *error = std::string("calibration must contain required member ")
            + key + " exactly once";
      }
      return false;
    }
  }
  const bool has_height_m_estimate =
      HasExactlyOneJsonMember(text, "height_m_estimate");
  const bool has_table_height_m =
      HasExactlyOneJsonMember(text, "table_height_m");
  if (!has_height_m_estimate && !has_table_height_m) {
    if (error != nullptr) {
      *error = "calibration must contain height_m_estimate or table_height_m";
    }
    return false;
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
  const unitree::common::Any* height_estimate = nullptr;
  const unitree::common::Any* table_height = nullptr;
  const unitree::common::Any* center = nullptr;
  const unitree::common::Any* x_axis = nullptr;
  const unitree::common::Any* y_axis = nullptr;
  const unitree::common::Any* z_axis = nullptr;
  const unitree::common::Any* corners_value = nullptr;
  if (!RequiredJsonMember(object, "table_length_m", &length) ||
      !RequiredJsonMember(object, "table_width_m", &width) ||
      !RequiredJsonMember(object, "center_raw_m", &center) ||
      !RequiredJsonMember(object, "x_axis_raw", &x_axis) ||
      !RequiredJsonMember(object, "y_axis_raw", &y_axis) ||
      !RequiredJsonMember(object, "z_axis_raw", &z_axis) ||
      !RequiredJsonMember(object, "corners_raw_m", &corners_value) ||
      !JsonNumber(*length, &parsed_dimensions.length_m) ||
      !JsonNumber(*width, &parsed_dimensions.width_m) ||
      !JsonVec3(*center, 1000.0, &parsed_table.center_raw_mm) ||
      !JsonVec3(*x_axis, 1.0, &parsed_table.x_axis_raw) ||
      !JsonVec3(*y_axis, 1.0, &parsed_table.y_axis_raw) ||
      !JsonVec3(*z_axis, 1.0, &parsed_table.z_axis_raw) ||
      !unitree::common::IsJsonArray(*corners_value)) {
    if (error != nullptr) *error = "calibration has invalid member types";
    return false;
  }
  double height_m = std::numeric_limits<double>::quiet_NaN();
  double alternate_height_m = std::numeric_limits<double>::quiet_NaN();
  if (has_height_m_estimate) {
    if (!RequiredJsonMember(object, "height_m_estimate", &height_estimate) ||
        !JsonNumber(*height_estimate, &height_m)) {
      if (error != nullptr) {
        *error = "calibration height_m_estimate has invalid type";
      }
      return false;
    }
  }
  if (has_table_height_m) {
    if (!RequiredJsonMember(object, "table_height_m", &table_height) ||
        !JsonNumber(*table_height, &alternate_height_m)) {
      if (error != nullptr) {
        *error = "calibration table_height_m has invalid type";
      }
      return false;
    }
    if (!has_height_m_estimate) {
      height_m = alternate_height_m;
    } else if (std::abs(height_m - alternate_height_m) > 1.0e-9) {
      if (error != nullptr) {
        *error = "calibration height_m_estimate and table_height_m differ";
      }
      return false;
    }
  }
  parsed_dimensions.height_m = height_m;

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
  output << "{\n  \"format\": "
         << "\"" << kTableCalibrationFormat << "\"";
  output << ",\n  \"units\": \"metres\"";
  output << ",\n  \"center_raw_m\": ";
  WriteVec3Json(output, table.center_raw_mm, 0.001);
  output << ",\n  \"x_axis_raw\": ";
  WriteVec3Json(output, table.x_axis_raw, 1.0);
  output << ",\n  \"y_axis_raw\": ";
  WriteVec3Json(output, table.y_axis_raw, 1.0);
  output << ",\n  \"z_axis_raw\": ";
  WriteVec3Json(output, table.z_axis_raw, 1.0);
  output << ",\n  \"height_m_estimate\": " << dims.height_m;
  output << ",\n  \"table_height_m\": " << dims.height_m;
  output << ",\n  \"table_length_m\": " << dims.length_m;
  output << ",\n  \"table_width_m\": " << dims.width_m;
  output << ",\n  \"rectangle_score\": " << table.rectangle_score;
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

Vec3 Multiply3(const double matrix[3][3], const Vec3& vector) {
  return {
      matrix[0][0] * vector.x + matrix[0][1] * vector.y +
          matrix[0][2] * vector.z,
      matrix[1][0] * vector.x + matrix[1][1] * vector.y +
          matrix[1][2] * vector.z,
      matrix[2][0] * vector.x + matrix[2][1] * vector.y +
          matrix[2][2] * vector.z,
  };
}

Quat MatrixToQuat(const double rotation[3][3]) {
  Quat quaternion;
  const double trace =
      rotation[0][0] + rotation[1][1] + rotation[2][2];
  if (trace > 0.0) {
    const double scale = std::sqrt(trace + 1.0) * 2.0;
    quaternion.w = 0.25 * scale;
    quaternion.x = (rotation[2][1] - rotation[1][2]) / scale;
    quaternion.y = (rotation[0][2] - rotation[2][0]) / scale;
    quaternion.z = (rotation[1][0] - rotation[0][1]) / scale;
  } else if (rotation[0][0] > rotation[1][1] &&
             rotation[0][0] > rotation[2][2]) {
    const double scale =
        std::sqrt(1.0 + rotation[0][0] - rotation[1][1] - rotation[2][2]) *
        2.0;
    quaternion.w = (rotation[2][1] - rotation[1][2]) / scale;
    quaternion.x = 0.25 * scale;
    quaternion.y = (rotation[0][1] + rotation[1][0]) / scale;
    quaternion.z = (rotation[0][2] + rotation[2][0]) / scale;
  } else if (rotation[1][1] > rotation[2][2]) {
    const double scale =
        std::sqrt(1.0 + rotation[1][1] - rotation[0][0] - rotation[2][2]) *
        2.0;
    quaternion.w = (rotation[0][2] - rotation[2][0]) / scale;
    quaternion.x = (rotation[0][1] + rotation[1][0]) / scale;
    quaternion.y = 0.25 * scale;
    quaternion.z = (rotation[1][2] + rotation[2][1]) / scale;
  } else {
    const double scale =
        std::sqrt(1.0 + rotation[2][2] - rotation[0][0] - rotation[1][1]) *
        2.0;
    quaternion.w = (rotation[1][0] - rotation[0][1]) / scale;
    quaternion.x = (rotation[0][2] + rotation[2][0]) / scale;
    quaternion.y = (rotation[1][2] + rotation[2][1]) / scale;
    quaternion.z = 0.25 * scale;
  }
  return Normalize(quaternion);
}

std::string UsageText(const std::string& program) {
  std::ostringstream out;
  out << "Usage: " << program
      << " [--host 192.168.10.1:801] [--tracker-name G1Pelvis]"
      << " [--base-subject G2Pelvis]"
      << " [--calib-sec 2] [--table-calib table_frame_latest.json]"
      << " [--pelvis-orientation-calib pelvis_orientation.json]"
      << " [--save-table-calib vicon_table_frame_candidate.json]"
      << " [--ignore-raw-sphere-m x,y,z,r] [--clear-raw-ignore-spheres]"
      << " [--corner-exclusion-radius-mm 50]"
      << " [--vicon-frame-rate-hz FALLBACK_HZ]"
      << " [--channel vicon_state_data_v2]"
      << " [--ball-track-association-radius-m 0.35]"
      << " [--ball-track-end-timeout-s 0.25]"
      << " [--duration 0] [--publish]\n"
      << "  --vicon-frame-rate-hz is fallback only when the SDK rate is unavailable.\n";
  return out.str();
}

bool ConsumeRequiredArgValue(
    int argc,
    char** argv,
    int* index,
    const std::string& key,
    std::string* value) {
  if (*index + 1 >= argc) {
    std::cerr << "Missing value for " << key << "\n";
    return false;
  }
  const std::string candidate = argv[*index + 1];
  if (candidate.rfind("--", 0) == 0 || candidate == "-h") {
    std::cerr << "Missing value for " << key << "\n";
    return false;
  }
  *value = candidate;
  ++(*index);
  return true;
}

bool ParseFinitePositiveDoubleStrict(
    const std::string& text, double* value) {
  try {
    size_t consumed = 0;
    const double parsed = std::stod(text, &consumed);
    if (consumed != text.size() || !std::isfinite(parsed) || parsed <= 0.0) {
      return false;
    }
    *value = parsed;
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

bool ParseFiniteNonNegativeDoubleStrict(
    const std::string& text, double* value) {
  try {
    size_t consumed = 0;
    const double parsed = std::stod(text, &consumed);
    if (consumed != text.size() || !std::isfinite(parsed) || parsed < 0.0) {
      return false;
    }
    *value = parsed;
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

bool ParseArgs(int argc, char** argv, Args* args) {
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    auto parse_positive_value = [&](double* value) -> bool {
      std::string text;
      if (!ConsumeRequiredArgValue(argc, argv, &i, key, &text)) return false;
      if (!ParseFinitePositiveDoubleStrict(text, value)) {
        std::cerr << "Invalid " << key
                  << " value, expected a finite positive number: "
                  << text << "\n";
        return false;
      }
      return true;
    };
    auto parse_non_negative_value = [&](double* value) -> bool {
      std::string text;
      if (!ConsumeRequiredArgValue(argc, argv, &i, key, &text)) return false;
      if (!ParseFiniteNonNegativeDoubleStrict(text, value)) {
        std::cerr << "Invalid " << key
                  << " value, expected a finite non-negative number: "
                  << text << "\n";
        return false;
      }
      return true;
    };
    if (key == "--host") {
      if (!ConsumeRequiredArgValue(argc, argv, &i, key, &args->host)) {
        return false;
      }
    }
    else if (key == "--tracker-name") {
      std::string value;
      if (!ConsumeRequiredArgValue(argc, argv, &i, key, &value)) return false;
      args->tracker_subject = value;
    }
    else if (key == "--base-subject") {
      if (!ConsumeRequiredArgValue(
              argc, argv, &i, key, &args->base_subject)) {
        return false;
      }
    }
    else if (key == "--duration") {
      if (!parse_non_negative_value(&args->duration_s)) return false;
    }
    else if (key == "--print-hz") {
      if (!parse_positive_value(&args->print_hz)) return false;
    }
    else if (key == "--calib-sec") {
      if (!parse_positive_value(&args->calib_s)) return false;
    }
    else if (key == "--table-calib") {
      if (!ConsumeRequiredArgValue(
              argc, argv, &i, key, &args->table_calib_path)) {
        return false;
      }
    }
    else if (key == "--pelvis-orientation-calib") {
      std::string value;
      if (!ConsumeRequiredArgValue(argc, argv, &i, key, &value)) return false;
      args->pelvis_extrinsics_path = value;
    }
    else if (key == "--save-table-calib") {
      if (!ConsumeRequiredArgValue(
              argc, argv, &i, key, &args->save_table_calib_path)) {
        return false;
      }
    }
    else if (key == "--lcm-url") {
      if (!ConsumeRequiredArgValue(argc, argv, &i, key, &args->lcm_url)) {
        return false;
      }
    }
    else if (key == "--channel") {
      std::string value;
      if (!ConsumeRequiredArgValue(argc, argv, &i, key, &value)) return false;
      args->channel = value;
    }
    else if (key == "--ball-track-association-radius-m") {
      std::string value;
      if (!ConsumeRequiredArgValue(argc, argv, &i, key, &value)) return false;
      if (!ParseFinitePositiveDoubleStrict(
              value, &args->ball_track_association_radius_m)) {
        std::cerr << "Invalid " << key
                  << " value, expected a finite positive number: "
                  << value << "\n";
        return false;
      }
    }
    else if (key == "--ball-track-end-timeout-s") {
      std::string value;
      if (!ConsumeRequiredArgValue(argc, argv, &i, key, &value)) return false;
      if (!ParseFinitePositiveDoubleStrict(
              value, &args->ball_track_end_timeout_s)) {
        std::cerr << "Invalid " << key
                  << " value, expected a finite positive number: "
                  << value << "\n";
        return false;
      }
    }
    else if (key == "--table-length") {
      if (!parse_positive_value(&args->table_length_m)) return false;
    }
    else if (key == "--table-width") {
      if (!parse_positive_value(&args->table_width_m)) return false;
    }
    else if (key == "--table-height") {
      if (!parse_positive_value(&args->table_height_m)) return false;
    }
    else if (key == "--expected-base-edge-distance") {
      if (!parse_non_negative_value(&args->expected_base_edge_distance_m)) {
        return false;
      }
    }
    else if (key == "--vicon-frame-rate-hz") {
      if (!parse_positive_value(&args->vicon_frame_rate_hz)) return false;
    }
    else if (key == "--corner-exclusion-radius-mm") {
      if (!parse_non_negative_value(&args->corner_exclusion_radius_mm)) {
        return false;
      }
    }
    else if (key == "--clear-raw-ignore-spheres") args->raw_ignore_spheres.clear();
    else if (key == "--ignore-raw-sphere-m") {
      RawIgnoreSphere sphere;
      std::string spec;
      if (!ConsumeRequiredArgValue(argc, argv, &i, key, &spec)) return false;
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
  if (args->tracker_subject != kDefaultTrackerSubject) {
    std::cerr << "--tracker-name must be exactly "
              << kDefaultTrackerSubject
              << " because the deployed pelvis calibration is bound to it\n";
    return false;
  }
  if (!args->table_calib_path.empty() &&
      !args->save_table_calib_path.empty()) {
    std::cerr << "--table-calib and --save-table-calib are mutually exclusive\n";
    return false;
  }
  std::vector<std::pair<std::string, std::string>> calibration_roles;
  if (!args->table_calib_path.empty()) {
    calibration_roles.emplace_back("--table-calib", args->table_calib_path);
  }
  if (!args->save_table_calib_path.empty()) {
    calibration_roles.emplace_back(
        "--save-table-calib", args->save_table_calib_path);
  }
  if (!args->pelvis_extrinsics_path.empty()) {
    calibration_roles.emplace_back(
        "--pelvis-orientation-calib", args->pelvis_extrinsics_path);
  }
  for (size_t first = 0; first < calibration_roles.size(); ++first) {
    std::error_code first_error;
    const std::filesystem::path first_path =
        std::filesystem::weakly_canonical(
            std::filesystem::absolute(calibration_roles[first].second),
            first_error);
    if (first_error) {
      std::cerr << "Could not resolve calibration path for "
                << calibration_roles[first].first << ": "
                << first_error.message() << "\n";
      return false;
    }
    for (size_t second = first + 1;
         second < calibration_roles.size(); ++second) {
      std::error_code second_error;
      const std::filesystem::path second_path =
          std::filesystem::weakly_canonical(
              std::filesystem::absolute(calibration_roles[second].second),
              second_error);
      if (second_error) {
        std::cerr << "Could not resolve calibration path for "
                  << calibration_roles[second].first << ": "
                  << second_error.message() << "\n";
        return false;
      }
      if (first_path == second_path) {
        std::cerr << calibration_roles[first].first << " and "
                  << calibration_roles[second].first
                  << " resolve to the same calibration file: "
                  << first_path.string() << "\n";
        return false;
      }
    }
  }
  if (args->channel != kViconV2Channel) {
    std::cerr << "--channel must be exactly " << kViconV2Channel << "\n";
    return false;
  }
  if (!std::isfinite(args->ball_track_association_radius_m) ||
      args->ball_track_association_radius_m <= 0.0) {
    std::cerr << "--ball-track-association-radius-m must be finite and positive\n";
    return false;
  }
  if (!std::isfinite(args->ball_track_end_timeout_s) ||
      args->ball_track_end_timeout_s <= 0.0) {
    std::cerr << "--ball-track-end-timeout-s must be finite and positive\n";
    return false;
  }
  return true;
}

bool IsTableOnlyCalibration(const Args& args) {
  return args.table_calib_path.empty() &&
      args.pelvis_extrinsics_path.empty() &&
      !args.save_table_calib_path.empty() && !args.publish;
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
      result.reason = "non-finite tracker rigid-body position";
      return result;
    }
    const Vec3 pelvis_world =
        RawToTableWorld(*pelvis_raw_mm, *frame, transform_args);
    if (!finite_vec(pelvis_world) || pelvis_world.x >= 0.0) {
      result.reason = "tracker rigid body is not on the robot side";
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

double TableRectangleScore(
    const std::array<Vec3, 4>& corners_raw_mm,
    const TableDimensions& dims) {
  std::array<double, 6> distances{};
  size_t distance_index = 0;
  double minimum_raw_z_mm = std::numeric_limits<double>::infinity();
  double maximum_raw_z_mm = -std::numeric_limits<double>::infinity();
  for (size_t first = 0; first < corners_raw_mm.size(); ++first) {
    minimum_raw_z_mm =
        std::min(minimum_raw_z_mm, corners_raw_mm[first].z);
    maximum_raw_z_mm =
        std::max(maximum_raw_z_mm, corners_raw_mm[first].z);
    for (size_t second = first + 1;
         second < corners_raw_mm.size(); ++second) {
      distances[distance_index++] =
          Distance(corners_raw_mm[first], corners_raw_mm[second]);
    }
  }
  std::sort(distances.begin(), distances.end());
  const double short_mm = std::min(dims.length_m, dims.width_m) * 1000.0;
  const double long_mm = std::max(dims.length_m, dims.width_m) * 1000.0;
  const double diagonal_mm = std::hypot(short_mm, long_mm);
  const std::array<double, 6> expected{
      short_mm, short_mm, long_mm, long_mm, diagonal_mm, diagonal_mm};
  double score = 0.0;
  for (size_t index = 0; index < distances.size(); ++index) {
    score += std::abs(distances[index] - expected[index]) / expected[index];
  }
  score += (maximum_raw_z_mm - minimum_raw_z_mm) / 300.0;
  return score;
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
    result.validation.reason = "non-finite tracker rigid-body position";
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
  frame.rectangle_score = TableRectangleScore(corners_raw_mm, dims);
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
  Quat orientation_xyzw;
  bool valid = false;
};

BasePoseWorld TransformRootPose(
    const SegmentPoseRaw& raw,
    const TableFrame& table,
    const Args& args,
    const PelvisExtrinsics& extrinsics) {
  BasePoseWorld world;
  if (!raw.valid || !table.valid || !extrinsics.valid) return world;

  const Vec3 x_axis = NormalizeVec(table.x_axis_raw);
  const Vec3 y_axis = NormalizeVec(table.y_axis_raw);
  const Vec3 z_axis = NormalizeVec(table.z_axis_raw);
  const double table_from_raw[3][3] = {
      {x_axis.x, x_axis.y, x_axis.z},
      {y_axis.x, y_axis.y, y_axis.z},
      {z_axis.x, z_axis.y, z_axis.z},
  };
  double raw_rigid[3][3]{};
  double world_rigid[3][3]{};
  double rigid_pelvis[3][3]{};
  double world_pelvis[3][3]{};
  QuatToMatrix(raw.quat_xyzw, raw_rigid);
  QuatToMatrix(extrinsics.rotation_rigid_from_pelvis, rigid_pelvis);
  Multiply3(table_from_raw, raw_rigid, world_rigid);
  Multiply3(world_rigid, rigid_pelvis, world_pelvis);

  world.orientation_xyzw = MatrixToQuat(world_pelvis);
  world.position_m =
      RawToTableWorld(raw.translation_mm, table, args) +
      Multiply3(world_pelvis, extrinsics.translation_pelvis_m);
  world.valid = std::isfinite(world.position_m.x) &&
      std::isfinite(world.position_m.y) &&
      std::isfinite(world.position_m.z) &&
      std::isfinite(world.orientation_xyzw.x) &&
      std::isfinite(world.orientation_xyzw.y) &&
      std::isfinite(world.orientation_xyzw.z) &&
      std::isfinite(world.orientation_xyzw.w);
  return world;
}

bool FiniteVec(const Vec3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) &&
      std::isfinite(value.z);
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
    if (!FiniteVec(world)) continue;
    if (world.x > args.table_length_m - kBallTrackingFarEdgeMarginM) {
      continue;
    }
    if (std::abs(world.y) > 0.5 * args.table_width_m) continue;
    if (world.z <= args.table_height_m) continue;
    candidates.push_back(BallCandidate{raw, world});
  }
  if (diagnostics != nullptr) diagnostics->candidate_count = candidates.size();
  return candidates;
}

bool VecLexicographicLess(const Vec3& lhs, const Vec3& rhs) {
  if (lhs.x != rhs.x) return lhs.x < rhs.x;
  if (lhs.y != rhs.y) return lhs.y < rhs.y;
  return lhs.z < rhs.z;
}

bool BallCandidateLess(const BallCandidate& lhs, const BallCandidate& rhs) {
  if (VecLexicographicLess(lhs.world, rhs.world)) return true;
  if (VecLexicographicLess(rhs.world, lhs.world)) return false;
  return VecLexicographicLess(lhs.raw, rhs.raw);
}

double SourceDeltaSeconds(
    const BallTrackState& state,
    int64_t source_frame,
    double source_time_s,
    double source_frame_rate_hz) {
  if (std::isfinite(source_time_s) &&
      source_time_s > state.last_observed_source_time_s) {
    return source_time_s - state.last_observed_source_time_s;
  }
  if (!std::isfinite(source_frame_rate_hz) || source_frame_rate_hz <= 0.0) {
    return 0.0;
  }
  const int64_t frame_delta = source_frame - state.last_observed_frame;
  return frame_delta > 0
      ? static_cast<double>(frame_delta) / source_frame_rate_hz
      : 0.0;
}

double NormalizedObservedSourceTime(
    const BallTrackState& state,
    int64_t source_frame,
    double source_time_s,
    double source_frame_rate_hz) {
  if (state.phase == BallTrackPhase::Inactive) {
    if (std::isfinite(source_time_s)) return source_time_s;
    if (std::isfinite(source_frame_rate_hz) && source_frame_rate_hz > 0.0) {
      return static_cast<double>(source_frame) / source_frame_rate_hz;
    }
    return 0.0;
  }
  return state.last_observed_source_time_s + SourceDeltaSeconds(
      state, source_frame, source_time_s, source_frame_rate_hz);
}

void UpdateVelocityFromSourceTime(
    BallTrackState* state,
    const BallCandidate& matched,
    int64_t source_frame,
    double source_time_s,
    double source_frame_rate_hz) {
  const bool had_observation = state->phase != BallTrackPhase::Inactive;
  const double dt = had_observation
      ? SourceDeltaSeconds(
            *state, source_frame, source_time_s, source_frame_rate_hz)
      : 0.0;
  const double normalized_time = NormalizedObservedSourceTime(
      *state, source_frame, source_time_s, source_frame_rate_hz);
  if (had_observation && dt > 0.0) {
    state->velocity = (matched.world - state->position) / dt;
    state->have_velocity = true;
  }
  state->position = matched.world;
  state->last_observed_frame = source_frame;
  state->last_observed_source_time_s = normalized_time;
}

void ResetActiveTrackPreservingAllocator(BallTrackState* state) {
  const int64_t last_allocated_track_id = state->last_allocated_track_id;
  *state = BallTrackState{};
  state->last_allocated_track_id = last_allocated_track_id;
}

BallTrackUpdate CurrentBallTrackUpdate(const BallTrackState& state) {
  return {state.phase, state.track_id, state.position, false, false};
}

BallTrackUpdate AdvanceBallTrackWithFrameRate(
    BallTrackState* state,
    const std::vector<BallCandidate>& candidates,
    int64_t source_frame,
    double source_time_s,
    double association_radius_m,
    double end_timeout_s,
    int64_t allocation_unix_time_us,
    double source_frame_rate_hz) {
  if (state == nullptr || !std::isfinite(association_radius_m) ||
      association_radius_m <= 0.0 || !std::isfinite(end_timeout_s) ||
      end_timeout_s <= 0.0 || !std::isfinite(source_frame_rate_hz) ||
      source_frame_rate_hz <= 0.0) {
    return {};
  }

  std::optional<BallCandidate> matched;
  if (state->phase == BallTrackPhase::Inactive) {
    for (const BallCandidate& candidate : candidates) {
      if (FiniteVec(candidate.world) && candidate.world.x > 0.0 &&
          (!matched.has_value() || BallCandidateLess(candidate, *matched))) {
        matched = candidate;
      }
    }
    if (!matched.has_value()) return CurrentBallTrackUpdate(*state);

    if (state->last_allocated_track_id ==
        std::numeric_limits<int64_t>::max()) {
      return {BallTrackPhase::Inactive, 0, state->position, false, false,
              BallTrackError::AllocatorExhausted};
    }
    const int64_t next_id = std::max<int64_t>(
        1, state->last_allocated_track_id + 1);
    state->track_id = std::max(next_id, allocation_unix_time_us);
    state->last_allocated_track_id = state->track_id;
  } else {
    const double dt = SourceDeltaSeconds(
        *state, source_frame, source_time_s, source_frame_rate_hz);
    const Vec3 predicted = state->have_velocity
        ? state->position + dt * state->velocity
        : state->position;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (const BallCandidate& candidate : candidates) {
      if (!FiniteVec(candidate.world)) continue;
      const double distance = Distance(candidate.world, predicted);
      if (distance < nearest_distance ||
          (distance == nearest_distance &&
           (!matched.has_value() || BallCandidateLess(candidate, *matched)))) {
        nearest_distance = distance;
        matched = candidate;
      }
    }
    if (nearest_distance > association_radius_m) matched.reset();
  }

  if (matched.has_value()) {
    UpdateVelocityFromSourceTime(
        state, *matched, source_frame, source_time_s, source_frame_rate_hz);
    state->phase = BallTrackPhase::Active;
    return {state->phase, state->track_id, state->position, true, false};
  }

  const double missing_s = std::max(
      0.0, SourceDeltaSeconds(
               *state, source_frame, source_time_s, source_frame_rate_hz));
  if (missing_s < end_timeout_s) {
    state->phase = BallTrackPhase::MissingGrace;
    return CurrentBallTrackUpdate(*state);
  }
  const BallTrackUpdate ended{
      BallTrackPhase::Inactive, state->track_id, state->position, false, true};
  ResetActiveTrackPreservingAllocator(state);
  return ended;
}

BallTrackUpdate AdvanceBallTrack(
    BallTrackState* state,
    const std::vector<BallCandidate>& candidates,
    int64_t source_frame,
    double source_time_s,
    double association_radius_m,
    double end_timeout_s,
    int64_t allocation_unix_time_us) {
  return AdvanceBallTrackWithFrameRate(
      state, candidates, source_frame, source_time_s, association_radius_m,
      end_timeout_s, allocation_unix_time_us, kDefaultViconFrameRateHz);
}

BallTrackUpdate AdvanceBallTrackForBaseFrame(
    BallTrackState* state,
    const std::vector<BallCandidate>& candidates,
    bool base_valid,
    int64_t source_frame,
    double source_time_s,
    double association_radius_m,
    double end_timeout_s,
    int64_t allocation_unix_time_us,
    double source_frame_rate_hz) {
  if (state == nullptr) return {};
  if (!base_valid) return CurrentBallTrackUpdate(*state);
  return AdvanceBallTrackWithFrameRate(
      state, candidates, source_frame, source_time_s, association_radius_m,
      end_timeout_s, allocation_unix_time_us, source_frame_rate_hz);
}

void FillMessage(
    lcm_types::transformation_t* msg,
    const std::string& name,
    const Vec3& pos,
    const Quat& quat,
    int64_t frame_number,
    const Args& args,
    double source_frame_rate_hz,
    int64_t publish_time_us,
    bool valid,
    bool occluded,
    int64_t track_id) {
  (void)args;
  msg->name = name;
  msg->vicon_frame_number = frame_number;
  msg->vicon_time_s = source_frame_rate_hz > 0.0
                          ? static_cast<double>(frame_number) / source_frame_rate_hz
                          : 0.0;
  msg->publish_time_us = publish_time_us;
  msg->track_id = track_id;
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

struct BasePublishState {
  bool was_valid = false;
};

struct BasePublishDecision {
  bool publish = false;
  bool valid = false;
};

BasePublishDecision AdvanceBasePublishState(
    BasePublishState* state, bool base_valid) {
  if (state == nullptr) return {};
  if (base_valid) {
    state->was_valid = true;
    return {true, true};
  }
  const bool publish_invalid_transition = state->was_valid;
  state->was_valid = false;
  return {publish_invalid_transition, false};
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
    double source_frame_rate_hz,
    int64_t publish_time_us) {
  RuntimeFrameWaitDecision decision;
  if (frame_received) return decision;
  decision.exit_loop = true;
  if (shutdown_requested) return decision;

  decision.exit_nonzero = true;
  FillMessage(
      &decision.invalid_base, args.base_subject, Vec3{}, Quat{},
      last_frame_number, args, source_frame_rate_hz, publish_time_us,
      false, true, 0);
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

  const bool table_only_calibration = IsTableOnlyCalibration(args);
  if (!table_only_calibration && args.pelvis_extrinsics_path.empty()) {
    std::cerr << "--pelvis-orientation-calib is required for runtime; "
                 "refusing to publish an uncalibrated pelvis pose\n";
    return 2;
  }
  PelvisExtrinsics pelvis_extrinsics;
  if (!table_only_calibration) {
    std::string pelvis_extrinsics_error;
    if (!LoadPelvisExtrinsics(
            args.pelvis_extrinsics_path, args.base_subject,
            &pelvis_extrinsics, &pelvis_extrinsics_error)) {
      std::cerr << "Pelvis orientation calibration rejected: "
                << pelvis_extrinsics_error << "\n";
      return 1;
    }
    std::cout << "Loaded pelvis orientation calibration from "
              << args.pelvis_extrinsics_path << "\n";
  }

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

  const auto root = client.GetSubjectRootSegmentName(args.tracker_subject);
  if (!Ok(root.Result) || std::string(root.SegmentName).empty()) {
    std::cerr << "Could not resolve root segment for Vicon tracker "
              << args.tracker_subject << "\n";
    return 1;
  }
  const std::string base_segment = root.SegmentName;
  std::cout << "Using Vicon tracker " << args.tracker_subject
            << " root segment " << base_segment
            << "; publishing base subject " << args.base_subject << "\n";

  SegmentPoseRaw initial_base_raw;
  const auto initial_base_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!g_stop && std::chrono::steady_clock::now() < initial_base_deadline) {
    initial_base_raw =
        ReadRootSegmentPose(&client, args.tracker_subject, base_segment);
    if (initial_base_raw.valid) break;
    const double remaining_s = std::chrono::duration<double>(
        initial_base_deadline - std::chrono::steady_clock::now()).count();
    if (remaining_s <= 0.0) break;
    WaitFrame(&client, std::min(1.0, remaining_s));
  }
  if (!initial_base_raw.valid) {
    std::cerr << "Could not read a valid root pose for Vicon tracker "
              << args.tracker_subject
              << " within 5 seconds\n";
    return 1;
  }
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
      std::cerr << "Table calibration rejected for current Vicon tracker "
                << args.tracker_subject << ": "
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
          ReadRootSegmentPose(&client, args.tracker_subject, base_segment);
      if (!AcceptCalibrationRootSample(calibration_pose, &calibration_root)) {
        std::cerr << "Table calibration rejected: current Vicon tracker "
                  << args.tracker_subject
                  << " root pose is invalid or occluded\n";
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
      std::cerr << "Table calibration rejected: no current valid Vicon tracker "
                << args.tracker_subject << " root pose was sampled\n";
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

  if (table_only_calibration) {
    std::cout << "Saved table calibration; exiting before pelvis orientation "
                 "is required.\n";
    client.Disconnect();
    return 0;
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
      TransformRootPose(
          initial_base_raw, table, args, pelvis_extrinsics);
  if (!initial_base_world.valid) {
    std::cerr << "Could not transform initial root pose for "
              << args.base_subject << " from Vicon tracker "
              << args.tracker_subject << "\n";
    return 1;
  }
  std::cout << "Base initial pelvis_world_m=[" << initial_base_world.position_m.x
            << ", " << initial_base_world.position_m.y << ", "
            << initial_base_world.position_m.z << "]\n";

  lcm::LCM lcm(args.lcm_url);
  if (args.publish && !lcm.good()) {
    std::cerr << "Could not initialize LCM: " << args.lcm_url << "\n";
    return 1;
  }
  std::cout << (args.publish ? "Publishing" : "Monitoring only")
            << " on " << args.channel << " via " << args.lcm_url << "\n";

  const auto start = std::chrono::steady_clock::now();
  auto last_print = start - std::chrono::seconds(10);
  unsigned long frames = 0;
  const auto latest_source_frame = client.GetFrameNumber();
  int64_t last_frame_number = Ok(latest_source_frame.Result)
      ? static_cast<int64_t>(latest_source_frame.FrameNumber)
      : 0;
  int runtime_exit_code = 0;
  BallTrackState ball_track;
  BasePublishState base_publish_state;
  std::array<unsigned long long, kBallRejectReasonCount> ball_reject_counts{};

  while (!g_stop) {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - start).count();
    if (args.duration_s > 0.0 && elapsed >= args.duration_s) break;
    const bool frame_received = WaitFrame(&client, 2.0);
    const int64_t frame_publish_time_us = NowUnixMicros();
    const RuntimeFrameWaitDecision wait_decision = DecideRuntimeFrameWait(
        frame_received, g_stop != 0, args.publish, last_frame_number,
        args, source_frame_rate_hz, frame_publish_time_us);
    if (wait_decision.exit_loop) {
      if (wait_decision.exit_nonzero) {
        std::cerr << "Vicon stream timeout: no frame received for 2 seconds; "
                     "exiting fail-closed.\n";
        if (wait_decision.publish_invalid_base) {
          PublishTransform(
              &lcm, args.channel, wait_decision.invalid_base);
        }
        runtime_exit_code = 1;
      }
      break;
    }
    ++frames;
    const int64_t frame_number = static_cast<int64_t>(client.GetFrameNumber().FrameNumber);
    last_frame_number = frame_number;

    const SegmentPoseRaw base_raw =
        ReadRootSegmentPose(&client, args.tracker_subject, base_segment);
    const BasePoseWorld transformed =
        TransformRootPose(base_raw, table, args, pelvis_extrinsics);
    const Vec3 base_world = transformed.valid ? transformed.position_m : Vec3{};
    const Quat base_world_q =
        transformed.valid ? transformed.orientation_xyzw : Quat{};
    const bool base_valid = transformed.valid;

    const std::vector<Vec3> unlabeled = UnlabeledMarkers(&client);
    Vec3 ball_raw;
    BallSelectionDiagnostics ball_diag;
    const std::vector<BallCandidate> candidates =
        BallCandidates(unlabeled, table, args, &ball_diag);
    const BallTrackPhase previous_phase = ball_track.phase;
    const int64_t previous_track_id = ball_track.track_id;
    const double source_time_s = source_frame_rate_hz > 0.0
        ? static_cast<double>(frame_number) / source_frame_rate_hz
        : std::numeric_limits<double>::quiet_NaN();
    if (!base_valid) {
      ball_diag.reason = BallRejectReason::BaseInvalid;
    } else if (candidates.empty()) {
      ball_diag.reason = BallRejectReason::NoCandidates;
    }
    const BallTrackUpdate ball_update = AdvanceBallTrackForBaseFrame(
        &ball_track, candidates, base_valid, frame_number, source_time_s,
        args.ball_track_association_radius_m,
        args.ball_track_end_timeout_s, frame_publish_time_us,
        source_frame_rate_hz);
    if (ball_update.error != BallTrackError::None) {
      std::cerr << "ball_track protocol_error="
                << BallTrackErrorName(ball_update.error)
                << " source_frame=" << frame_number << "\n";
      runtime_exit_code = 1;
      break;
    }
    const bool ball_valid = ball_update.publish_valid;
    const Vec3 ball_world = ball_update.position;
    if (ball_valid) {
      double nearest = std::numeric_limits<double>::infinity();
      for (const BallCandidate& candidate : candidates) {
        const double distance = Distance(candidate.world, ball_world);
        if (distance < nearest) {
          nearest = distance;
          ball_raw = candidate.raw;
        }
      }
      ball_diag.reason = BallRejectReason::None;
    } else {
      ++ball_reject_counts[BallRejectReasonIndex(ball_diag.reason)];
    }
    if (ball_update.phase != previous_phase ||
        (ball_update.publish_valid &&
         ball_update.track_id != previous_track_id)) {
      std::cout << "ball_track transition="
                << BallTrackPhaseName(previous_phase) << "->"
                << BallTrackPhaseName(ball_update.phase)
                << " track_id=" << ball_update.track_id
                << " publish_valid=" << ball_update.publish_valid
                << " publish_end=" << ball_update.publish_end
                << " source_frame=" << frame_number << "\n";
    }

    const BasePublishDecision base_publish =
        AdvanceBasePublishState(&base_publish_state, base_valid);
    if (args.publish) {
      lcm_types::transformation_t msg;
      if (base_publish.publish) {
        FillMessage(
            &msg, args.base_subject, base_world, base_world_q, frame_number,
            args, source_frame_rate_hz, frame_publish_time_us,
            base_publish.valid, !base_publish.valid, 0);
        PublishTransform(&lcm, args.channel, msg);
      }
      if (ball_update.publish_valid || ball_update.publish_end) {
        FillMessage(
            &msg, "ball", ball_world, Quat{}, frame_number,
            args, source_frame_rate_hz, frame_publish_time_us,
            ball_update.publish_valid, ball_update.publish_end,
            ball_update.track_id);
        PublishTransform(&lcm, args.channel, msg);
      }
      FillMessage(
          &msg, "table",
          Vec3{args.table_length_m * 0.5, 0.0, args.table_height_m},
          Quat{}, frame_number, args, source_frame_rate_hz,
          frame_publish_time_us, true, false, 0);
      PublishTransform(&lcm, args.channel, msg);
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
                << " ball_track_phase=" << BallTrackPhaseName(ball_update.phase)
                << " ball_track_id=" << ball_update.track_id;
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
