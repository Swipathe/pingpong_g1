#include "DataStreamClient.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
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

struct Quat {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
};

Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 operator*(double s, const Vec3& a) { return {s * a.x, s * a.y, s * a.z}; }

double Dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

double Norm(const Vec3& a) { return std::sqrt(Dot(a, a)); }

double Distance(const Vec3& a, const Vec3& b) { return Norm(a - b); }

bool IgnoredRawMarker(const Vec3& raw_mm) {
  return Distance(raw_mm, Vec3{60100.0, 80900.0, 2104.0}) <= 500.0;
}

Vec3 Mean(const std::vector<Vec3>& pts) {
  Vec3 sum;
  for (const auto& p : pts) sum = sum + p;
  return (1.0 / std::max<size_t>(pts.size(), 1)) * sum;
}

bool Ok(Result::Enum result) { return result == Result::Success; }

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
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) next[r] += n[r][c] * v[c];
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

  const Vec3 rotated_ref_mean = Rotate(r, ref_mean);
  *t = cur_mean - rotated_ref_mean;

  double sum_sq = 0.0;
  for (size_t i = 0; i < ref.size(); ++i) {
    const Vec3 pred = Rotate(r, ref[i]) + *t;
    const Vec3 err = cur[i] - pred;
    sum_sq += Dot(err, err);
  }
  *rms_mm = std::sqrt(sum_sq / ref.size());
  return std::isfinite(*rms_mm);
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);

  std::string host = "192.168.10.1:801";
  std::string base_subject = "G1Pelvis";
  double duration_s = 20.0;
  double print_hz = 10.0;
  if (argc > 1) host = argv[1];
  if (argc > 2) base_subject = argv[2];
  if (argc > 3) duration_s = std::stod(argv[3]);

  Client client;
  client.SetStreamMode(StreamMode::ClientPull);

  std::cout << "Connecting to " << host << " ...\n";
  const auto connect = client.Connect(host);
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

  std::map<std::string, Vec3> ref_markers;
  while (!g_stop) {
    WaitFrame(&client, 1.0);
    ref_markers = VisibleSubjectMarkers(&client, base_subject);
    if (ref_markers.size() >= 3) break;
    std::cout << "Waiting for at least 3 visible " << base_subject
              << " markers; currently " << ref_markers.size() << "\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  if (ref_markers.size() < 3) {
    std::cerr << "Could not capture rigid template for " << base_subject << "\n";
    return 1;
  }

  std::cout << "Captured rigid template with " << ref_markers.size() << " markers:";
  for (const auto& kv : ref_markers) std::cout << " " << kv.first;
  std::cout << "\n";

  const auto start = std::chrono::steady_clock::now();
  auto last_print = start - std::chrono::seconds(10);
  unsigned long frames = 0;
  bool have_ball_track = false;
  Vec3 tracked_ball;

  while (!g_stop) {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - start).count();
    if (duration_s > 0.0 && elapsed >= duration_s) break;
    if (!WaitFrame(&client, 2.0)) continue;
    ++frames;

    const auto cur_markers_map = VisibleSubjectMarkers(&client, base_subject);
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

    const unsigned int unlabeled_count = client.GetUnlabeledMarkerCount().MarkerCount;
    std::vector<Vec3> unlabeled_markers;
    unlabeled_markers.reserve(unlabeled_count);
    for (unsigned int i = 0; i < unlabeled_count; ++i) {
      const auto u = client.GetUnlabeledMarkerGlobalTranslation(i);
      unlabeled_markers.push_back({u.Translation[0], u.Translation[1], u.Translation[2]});
    }

    std::vector<size_t> ball_candidates;
    size_t raw_ignore_count = 0;
    for (size_t i = 0; i < unlabeled_markers.size(); ++i) {
      const Vec3& p = unlabeled_markers[i];
      if (IgnoredRawMarker(p)) {
        ++raw_ignore_count;
        continue;
      }
      // Current lab layout: the moving ping-pong ball lives in this Vicon region.
      // This rejects the persistent stray point near y ~= 1.58 m.
      if (p.y > 2000.0 && p.z > 500.0 && p.z < 2000.0) {
        ball_candidates.push_back(i);
      }
    }
    if (ball_candidates.empty()) {
      for (size_t i = 0; i < unlabeled_markers.size(); ++i) {
        if (!IgnoredRawMarker(unlabeled_markers[i])) ball_candidates.push_back(i);
      }
    }

    Vec3 ball;
    int ball_index = -1;
    bool ball_valid = !ball_candidates.empty();
    if (ball_valid) {
      size_t best_idx = ball_candidates.front();
      if (have_ball_track) {
        double best_dist = Distance(unlabeled_markers[best_idx], tracked_ball);
        for (const size_t idx : ball_candidates) {
          const double dist = Distance(unlabeled_markers[idx], tracked_ball);
          if (dist < best_dist) {
            best_dist = dist;
            best_idx = idx;
          }
        }
      }
      ball = unlabeled_markers[best_idx];
      tracked_ball = ball;
      have_ball_track = true;
      ball_index = static_cast<int>(best_idx);
    }

    const double print_period = 1.0 / std::max(print_hz, 1.0e-6);
    if (std::chrono::duration<double>(now - last_print).count() >= print_period) {
      const auto frame_number = client.GetFrameNumber().FrameNumber;
      const double hz = frames / std::max(elapsed, 1.0e-6);
      std::cout << std::fixed << std::setprecision(4)
                << "frame=" << frame_number
                << " hz=" << std::setprecision(1) << hz
                << " base_fit_valid=" << base_valid
                << " base_markers=" << cur.size()
                << " base_rms_mm=" << std::setprecision(2) << base_rms
                << " base_m=[" << std::setprecision(4)
                << base_t.x * 0.001 << ", " << base_t.y * 0.001 << ", " << base_t.z * 0.001 << "]"
                << " base_quat_xyzw=[" << base_q.x << ", " << base_q.y << ", "
                << base_q.z << ", " << base_q.w << "]"
                << " unlabeled_count=" << unlabeled_count
                << " raw_ignore_count=" << raw_ignore_count
                << " ball_index=" << ball_index
                << " ball_unlabeled_valid=" << ball_valid
                << " ball_m=[" << ball.x * 0.001 << ", " << ball.y * 0.001 << ", " << ball.z * 0.001 << "]"
                << "\n";
      last_print = now;
    }
  }

  client.Disconnect();
  return 0;
}
