### Task 2: 实现 C++ Vicon 三态物理球轨迹与 v2 发布

**Files:**
- Modify: `deploy/mocap_bridge/vicon_table_lcm_bridge.cpp: Args, BallTrackState, SelectBall/UpdateBallTrack/NoteBallMiss, FillMessage, main`
- Create: `deploy/mocap_bridge/tests/test_vicon_ball_track_v2.cpp`
- Modify: `deploy/mocap_bridge/build_v2_mocap.sh`

**Interfaces:**
- Consumes: Task 1 的 `transformation_t.track_id`。
- Produces: `BallTrackUpdate AdvanceBallTrack(BallTrackState*, const std::vector<BallCandidate>&, int64_t, double, double, double, int64_t)`、严格 v2 CLI、同一轨迹一次 invalid end、测试 binary 的 `--emit-track-fixture` parity 输出。

- [ ] **Step 1: 写纯状态机失败测试，固定 source time 与分配时钟**

```cpp
#define VICON_TABLE_LCM_BRIDGE_TESTING
#include "../vicon_table_lcm_bridge.cpp"

void TestTrackIdentityAndGrace() {
  BallTrackState state;
  const auto first = AdvanceBallTrack(
      &state, {Candidate(0.80, -0.10, 1.00)}, 100, 1.00,
      0.35, 0.25, 1000000);
  Expect(first.publish_valid && first.track_id == 1000000,
         "first eligible marker allocates injected positive id");
  const auto approach = AdvanceBallTrack(
      &state, {Candidate(0.50, -0.10, 1.00)}, 101, 1.01,
      0.35, 0.25, 1000001);
  Expect(approach.publish_valid && approach.track_id == first.track_id,
         "in-radius approach establishes velocity without changing id");
  const auto reverse = AdvanceBallTrack(
      &state, {Candidate(-0.02, -0.10, 1.00)}, 102, 1.02,
      0.35, 0.25, 1000002);
  Expect(reverse.publish_valid && reverse.track_id == first.track_id,
         "x<=0 retains id when constant-velocity association stays in radius");
  const auto short_miss = AdvanceBallTrack(
      &state, {}, 110, 1.20, 0.35, 0.25, 1000003);
  Expect(!short_miss.publish_valid && !short_miss.publish_end,
         "short miss stays in grace");
  const auto end = AdvanceBallTrack(
      &state, {}, 116, 1.28, 0.35, 0.25, 1000004);
  Expect(end.publish_end && end.track_id == first.track_id,
         "timeout emits old id once");
  const auto end_again = AdvanceBallTrack(
      &state, {}, 117, 1.29, 0.35, 0.25, 1000005);
  Expect(!end_again.publish_end, "end is not repeated");
}
```

同一文件增加表驱动测试 `TrackCase{name,candidates,frame,time,expected_phase,expected_id_relation,publish_valid,publish_end}`，依次覆盖：距离恰为 `0.35 m` 仍重关联、`0.350001 m` 不抢占、bounce 后 x/速度反号且每一步 CV 预测距离都在 `0.35 m` 内时仍保 ID、结束后下一轨迹 ID严格增加、pelvis/table ID 为 0、valid/end ball ID 相同。每行都断言 phase、publish flags、ID 关系和有限位置，而不是只检查进程退出；不得因越过 `x=0` 或 bounce 放宽关联半径。

- [ ] **Step 2: 运行新 C++ target，确认缺少接口而编译失败**

Run: `bash deploy/mocap_bridge/build_v2_mocap.sh`

Expected: FAIL，包含 `AdvanceBallTrack was not declared`。

- [ ] **Step 3: 加入明确的 C++ 状态与结果类型**

```cpp
constexpr const char* kViconV2Channel = "vicon_state_data_v2";

enum class BallTrackPhase { Inactive, Active, MissingGrace };

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
};
```

- [ ] **Step 4: 实现 source-time 状态推进与严格关联门限**

```cpp
const int64_t allocated = std::max(
    state->last_allocated_track_id + 1, allocation_unix_time_us);

// INACTIVE only admits x > 0. ACTIVE/MISSING_GRACE use constant-velocity
// prediction and accept only the nearest candidate within radius.
const double missing_s = std::max(
    0.0, source_time_s - state->last_observed_source_time_s);
if (matched.has_value()) {
  UpdateVelocityFromSourceTime(state, *matched, source_frame, source_time_s);
  state->phase = BallTrackPhase::Active;
  return {state->phase, state->track_id, state->position, true, false};
}
if (missing_s < end_timeout_s) {
  state->phase = BallTrackPhase::MissingGrace;
  return {state->phase, state->track_id, state->position, false, false};
}
const BallTrackUpdate ended{
    BallTrackPhase::Inactive, state->track_id, state->position, false, true};
ResetActiveTrackPreservingAllocator(state);
return ended;
```

当 source time 非有限或不递增时，使用 `(source_frame-last_frame)/source_frame_rate_hz`；host steady/system clock 不参与连续性判定。

- [ ] **Step 5: 配置化 v2 channel、默认参数与消息 ID**

```cpp
struct Args {
  std::string host = "192.168.10.1:801";
  std::string base_subject = kBaseSubject;
  std::string lcm_url = "udpm://239.255.76.67:7667?ttl=255";
  std::string table_calib_path;
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

void FillMessage(
    lcm_types::transformation_t* msg,
    const std::string& name,
    const Vec3& pos,
    const Quat& quat,
    int64_t frame_number,
    const Args& args,
    double source_frame_rate_hz,
    bool valid,
    bool occluded,
    int64_t track_id) {
  (void)args;
  msg->name = name;
  msg->vicon_frame_number = frame_number;
  msg->vicon_time_s = source_frame_rate_hz > 0.0
                          ? static_cast<double>(frame_number) / source_frame_rate_hz
                          : 0.0;
  msg->publish_time_us = NowUnixMicros();
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
```

CLI `--channel` 只接受 `vicon_state_data_v2`；两个数值必须有限且正。pelvis invalid 不能结束球轨迹；base/table 发 0，valid/end ball 发正 ID；打印默认降为 `1 Hz`，状态转换/协议错误即时输出。

- [ ] **Step 6: 运行 C++ 状态机、消息合同和 bridge 构建**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
bash deploy/mocap_bridge/build_v2_mocap.sh
deploy/mocap_bridge/.build-v2/test_vicon_ball_track_v2
deploy/mocap_bridge/.build-v2/vicon_table_lcm_bridge_v2 --help | \
  grep -F 'vicon_state_data_v2'
```

Expected: test exit 0；help 包含 v2 channel 与两个轨迹参数。

- [ ] **Step 7: 提交 C++ publisher 单元**

```bash
git add deploy/mocap_bridge/tests/test_vicon_ball_track_v2.cpp \
  deploy/mocap_bridge/build_v2_mocap.sh
git add -p -- deploy/mocap_bridge/vicon_table_lcm_bridge.cpp
git diff --cached --check
git diff --cached --name-status
git commit -m "feat: track physical balls in Vicon publisher"
```

---
