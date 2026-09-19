# HITTER 单拍轨迹 ID 与安全撤拍实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 RobotBridge4 中建立发布端权威 `track_id`、每个物理球最多击打一拍、撞网/失效可安全撤拍、真机 WAITING 锁存当前位置，以及球桌绝对 y 决定正反手的完整 v2 链路。

**Architecture:** `transformation_t` 在独立的 `vicon_state_data_v2` 频道携带发布端分配的正整数球轨迹 ID，C++ Vicon 与 Python ChingMu 发布端实现相同的三态关联器。RealWorld 负责 v2 解码、身份/帧/freshness 校验和新发准入；planner worker 保留 latest-only 输入但输出有序完成队列；纯 lifecycle 负责消费 ID、锁定命令、撤拍和承诺窗口。Diagnostics 只投影生产状态，历史 `track_epoch` 只能通过 replay-only 适配器进入离线回放。

**Tech Stack:** Python 3、NumPy、pytest/unittest、LCM/lcm-gen、C++17、Vicon DataStream SDK、ChingMu SDK、OmegaConf/Hydra、ONNX Runtime、Loguru。

## Global Constraints

- 所有读写、测试和 Git 命令的工作目录固定为 `/home/loco1/BOB/Hitter/RobotBridge4`；不得修改 RobotBridge3、RobotBridge2、MOSAIC-main 或 Omega-Athlete。
- 当前工作树已有大量用户修改、删除和未跟踪文件；不得执行 `git restore`、`git checkout --`、`git reset --hard`、清理命令或覆盖已有删除状态。
- 每次提交只使用明确路径的 `git add path/to/file1 path/to/file2`；禁止 `git add .` 和 `git add -A`。
- 对实施开始前已经是 `M` 的路径必须用 `git add -p -- path/to/file` 只暂存本任务新 hunk；新建且不覆盖用户未跟踪文件的路径才可直接 `git add`。每次 commit 前运行 `git diff --cached --check` 和 `git diff --cached --name-status`。
- 不恢复已删除的 `deploy/mocap_bridge/tests/test_vicon_table_lcm_bridge.cpp`、`deploy/mocap_bridge/tests/test_chingmu_table_lcm_bridge.py`、`deploy/mocap_bridge/tests/test_monitor_vicon_lcm.py` 或已删除的旧 lifecycle 测试；新功能使用新的 v2 聚焦测试文件。
- `transformation_t.hpp` 和 `transformation_t.py` 必须从唯一的 `.lcm` 源文件用 `lcm-gen` 生成，不得手工编辑。
- 真机控制和在线诊断只允许 `vicon_state_data_v2`；不得实现 v1/v2 双解码 fallback。
- 有效/结束 ball 均携带同一个正整数 `track_id`；pelvis/table 的 `track_id` 必须为 0；ball `track_id <= 0` 一律 fail closed。
- 自动手型的唯一规则是预测击球点球桌/world 绝对坐标 `y < 0 -> forehand`、`y >= 0 -> backhand`；`y == 0` 固定归反手。
- 第一次进入 `ARMED` 后锁定 `track_id`、手型、base target 和首个 deadline；`TTS <= 0.30 s` 后冻结普通 planner 命令。
- 真机 WAITING 只在进入状态的边沿锁存一次有效 pelvis 世界 x/y；WAITING 期间不得随机器人位置重写 anchor。
- 不改变 ONNX `obs [1,104] -> actions [1,29]`、PD 接口、Unitree `trans` 协议、MuJoCo 发球行为或现有 5 秒真机首帧平滑过渡。
- 实施期间只做离线测试、构建和只读进程检查；没有用户针对当次实验的明确授权与现场 readiness 确认，不启动真机 policy、不发 PD 命令、不执行击球验收。
- `unitree_sdk2/build` 的现有 CMake cache 指向 RobotBridge2，禁止读取它作为 RobotBridge4 构建依据或向其中构建；所有 Unitree target 验证必须从 RobotBridge4 源树配置到 `unitree_sdk2/.build-robotbridge4-v2`，并核对 `CMAKE_HOME_DIRECTORY` 精确指向 RobotBridge4。
- 默认参数固定为：association `0.35 m`、publisher end timeout `0.25 s`、stream/ball stale `0.40 s`、inter-serve no-ball `0.50 s`、planner `50 Hz`、completed queue `64`、连续失败 `3`、commit `0.30 s`、racket position delta `0.05 m`、racket velocity delta `0.75 m/s`、deadline delta `0.05 s`。

---

## 文件结构与职责

| 文件 | 最终职责 |
|---|---|
| `unitree_sdk2/lcm_types/transformation_t.{lcm,hpp,py}` | v2 wire schema 及生成绑定 |
| `deploy/mocap_bridge/vicon_table_lcm_bridge.cpp` | C++ Vicon 候选筛选、三态轨迹关联、ID 分配、v2 发布 |
| `deploy/mocap_bridge/chingmu_table_lcm_bridge.py` | 与 C++ 等价的 ChingMu 轨迹状态机和 v2 发布 |
| `deploy/mocap_bridge/monitor_vicon_lcm.py` | v2 合同检查、1 Hz 摘要和带 ID 的 CSV |
| `deploy/utils/hitter_runtime_types.py` | runtime/diagnostics 共用的 `SnapshotKey`、typed failure 类型，避免 runtime 从 diagnostics 反向取得身份类型 |
| `deploy/utils/hitter_planner.py` | typed planner rejection、table-y 手型选择、显式手型 base 规划 |
| `deploy/utils/hitter_realtime.py` | immutable snapshot/result、有序 completed queue、单拍 lifecycle |
| `deploy/simulator/real_world.py` | v2 callback、防 decode 崩线程、wire ID/帧/freshness/准入状态 |
| `deploy/envs/hitter.py` | 50 Hz 提交、全量结果 drain、lifecycle 决策应用、WAITING anchor 和 104-D observation |
| `deploy/utils/hitter_runtime_factory.py` | 全部安全参数解析、数值校验、真机配置约束 |
| `deploy/diagnostics/hitter_task_{models,events,attempts,pipeline}.py` | v2 canonical diagnostics 状态与 shadow projection |
| `deploy/diagnostics/hitter_task_{monitor,recording}.py` | 在线 v2 monitor、录制和 CSV 字段 |
| `deploy/diagnostics/hitter_task_legacy_identity.py` | 仅离线 replay/export 可用的确定性历史 ID 适配器 |
| `deploy/diagnostics/hitter_task_replay.py` | v2 replay 和显式 legacy 适配路径 |
| `deploy/diagnostics/export_hitter_task_csv.py` | 以 `(attempt_id, identity_source, track_id, generation)` 为键的原子 CSV 导出 |
| `docs/hitter_single_shot_track_id_real_world_acceptance.md` | 只读监测到低风险单拍的分阶段人工验收门禁 |

---

### Task 1: 建立 `transformation_t` v2 schema 与跨语言 wire 证明

**Files:**
- Modify: `unitree_sdk2/lcm_types/transformation_t.lcm`
- Regenerate: `unitree_sdk2/lcm_types/transformation_t.hpp`
- Regenerate: `unitree_sdk2/lcm_types/transformation_t.py`
- Create: `deploy/mocap_bridge/tests/test_transformation_t_v2.py`
- Create: `deploy/mocap_bridge/tests/test_transformation_t_v2.cpp`
- Create: `deploy/mocap_bridge/build_v2_mocap.sh`
- Local-only modify (do not stage): `.git/info/exclude`

**Interfaces:**
- Consumes: 现有 `lcm_types.transformation_t` 字段布局。
- Produces: `int64_t track_id`；C++/Python 相同 fingerprint；`.build-v2/test_transformation_t_v2 --emit-hex` 输出可由 Python v2 decoder 解码的固定 payload。

- [ ] **Step 1: 写 Python 失败测试，覆盖字段、round-trip、v1 拒绝和 C++ fixture**

```python
def test_python_round_trip_preserves_track_id():
    msg = transformation_t()
    msg.name = "ball"
    msg.vicon_frame_number = 41
    msg.vicon_time_s = 0.125
    msg.publish_time_us = 1_700_000_000_000_000
    msg.track_id = 9001
    msg.valid, msg.occluded = 1, 0
    msg.pos_vicon = [0.7, -0.2, 1.0]
    msg.quat_vicon = [0.0, 0.0, 0.0, 1.0]
    decoded = transformation_t.decode(msg.encode())
    assert decoded.track_id == 9001


def test_v1_payload_is_rejected():
    v1_payload_bytes = bytes.fromhex("71f936e3b20f1df5") + (b"\0" * 96)
    with pytest.raises(ValueError, match="Decode error"):
        transformation_t.decode(v1_payload_bytes)
```

测试中的 v1 fixture 使用当前设计文档提交前实测 packed fingerprint `71f936e3b20f1df5` 构造，不从 v2 class 重新编码。

- [ ] **Step 2: 写 C++ 失败测试并把独立测试 target 接入构建脚本**

```cpp
int main(int argc, char** argv) {
  lcm_types::transformation_t msg;
  msg.name = "ball";
  msg.vicon_frame_number = 41;
  msg.vicon_time_s = 0.125;
  msg.publish_time_us = 1700000000000000LL;
  msg.track_id = 9001;
  msg.valid = 1;
  msg.occluded = 0;
  msg.pos_vicon[0] = 0.7;
  msg.pos_vicon[1] = -0.2;
  msg.pos_vicon[2] = 1.0;
  msg.quat_vicon[3] = 1.0;
  std::vector<unsigned char> wire(msg.getEncodedSize());
  if (msg.encode(wire.data(), 0, wire.size()) < 0) return 2;
  if (argc == 2 && std::string(argv[1]) == "--emit-hex") {
    for (unsigned char byte : wire) std::cout << std::hex << std::setw(2)
                                               << std::setfill('0') << int(byte);
    std::cout << "\n";
  }
  return msg.track_id == 9001 ? 0 : 3;
}
```

`build_v2_mocap.sh` 必须单独编译这个不依赖 Vicon runtime 的 target；Task 2 加入状态机测试后，同一脚本再构建 active C++ bridge 与新状态机测试。不得修改工作树中用户已有的未跟踪 `build_cpp_probe.sh`，也不得恢复已删除的旧 C++ test。

脚本内容固定为独立、fail-fast 的 v2 build：

```bash
#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOTBRIDGE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SDK_DIR="${ROBOTBRIDGE_DIR}/vicon_datastream_sdk/linux64/Linux64"
UNITREE_INCLUDE_DIR="${ROBOTBRIDGE_DIR}/unitree_sdk2/include"
UNITREE_SDK_LIB="${ROBOTBRIDGE_DIR}/unitree_sdk2/lib/x86_64/libunitree_sdk2.a"
BUILD_DIR="${SCRIPT_DIR}/.build-v2"
mkdir -p "${BUILD_DIR}"

mapfile -t LCM_CFLAGS < <(pkg-config --cflags-only-I lcm | tr ' ' '\n' | sed '/^$/d')
mapfile -t LCM_LIBS < <(pkg-config --libs lcm | tr ' ' '\n' | sed '/^$/d')

g++ -std=c++17 -O2 -I"${ROBOTBRIDGE_DIR}" \
  "${LCM_CFLAGS[@]}" \
  "${SCRIPT_DIR}/tests/test_transformation_t_v2.cpp" \
  "${LCM_LIBS[@]}" \
  -o "${BUILD_DIR}/test_transformation_t_v2"

g++ -std=c++17 -O2 -I"${SDK_DIR}" -I"${ROBOTBRIDGE_DIR}" \
  -I"${UNITREE_INCLUDE_DIR}" "${LCM_CFLAGS[@]}" \
  "${SCRIPT_DIR}/vicon_table_lcm_bridge.cpp" \
  -L"${SDK_DIR}" -Wl,-rpath,"${SDK_DIR}" -lViconDataStreamSDK_CPP \
  "${UNITREE_SDK_LIB}" "${LCM_LIBS[@]}" \
  -o "${BUILD_DIR}/vicon_table_lcm_bridge_v2"

if [[ -f "${SCRIPT_DIR}/tests/test_vicon_ball_track_v2.cpp" ]]; then
  g++ -std=c++17 -O2 -I"${SDK_DIR}" -I"${ROBOTBRIDGE_DIR}" \
    -I"${UNITREE_INCLUDE_DIR}" "${LCM_CFLAGS[@]}" \
    "${SCRIPT_DIR}/tests/test_vicon_ball_track_v2.cpp" \
    -L"${SDK_DIR}" -Wl,-rpath,"${SDK_DIR}" -lViconDataStreamSDK_CPP \
    "${UNITREE_SDK_LIB}" "${LCM_LIBS[@]}" \
    -o "${BUILD_DIR}/test_vicon_ball_track_v2"
fi
```

`.build-v2/` 是本任务唯一的新构建输出目录。创建脚本时先只读确认该目录不存在；随后用 `apply_patch` 在 `.git/info/exclude` 追加且只追加一行 `deploy/mocap_bridge/.build-v2/`（已有完全相同行则不重复），本地排除文件不得 stage/commit。首次构建后运行 `git check-ignore -v deploy/mocap_bridge/.build-v2/test_transformation_t_v2`，Expected: 命中 `.git/info/exclude` 的精确行。不得写入实施前已有的未跟踪 `deploy/mocap_bridge/bin/`，也不得覆盖其中任何 binary。

- [ ] **Step 3: 运行测试，确认因 schema 尚无字段而失败**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. \
  /home/loco1/miniconda3/envs/rb/bin/python -m pytest -p no:cacheprovider \
  deploy/mocap_bridge/tests/test_transformation_t_v2.py -q
```

Expected: FAIL，包含 `AttributeError: 'transformation_t' object has no attribute 'track_id'` 或 C++ `no member named 'track_id'`。

- [ ] **Step 4: 修改唯一 schema 源并重新生成两种绑定**

```text
package lcm_types;

struct transformation_t
{
    string name;
    int64_t vicon_frame_number;
    double vicon_time_s;
    int64_t publish_time_us;
    int64_t track_id;
    int8_t valid;
    int8_t occluded;
    double pos_vicon[3];
    double quat_vicon[4];
}
```

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
lcm-gen -x --cpp-hpath unitree_sdk2 unitree_sdk2/lcm_types/transformation_t.lcm
lcm-gen -p --ppath unitree_sdk2 unitree_sdk2/lcm_types/transformation_t.lcm
```

- [ ] **Step 5: 构建并运行跨语言 round-trip**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
bash deploy/mocap_bridge/build_v2_mocap.sh
deploy/mocap_bridge/.build-v2/test_transformation_t_v2
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. \
  /home/loco1/miniconda3/envs/rb/bin/python -m pytest -p no:cacheprovider \
  deploy/mocap_bridge/tests/test_transformation_t_v2.py -q
```

Expected: C++ exit 0；pytest `4 passed`，包括 Python 解码 C++ hex payload 后 `track_id == 9001`。

- [ ] **Step 6: 明确提交 schema 与新测试，不带入其他工作树内容**

```bash
git add unitree_sdk2/lcm_types/transformation_t.lcm \
  unitree_sdk2/lcm_types/transformation_t.hpp \
  unitree_sdk2/lcm_types/transformation_t.py \
  deploy/mocap_bridge/tests/test_transformation_t_v2.py \
  deploy/mocap_bridge/tests/test_transformation_t_v2.cpp \
  deploy/mocap_bridge/build_v2_mocap.sh
git diff --cached --check
git diff --cached --name-status
git commit -m "feat: add HITTER v2 track id schema"
```

---

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
上述 7 参数 `AdvanceBallTrack(...)` 是固定 `300 Hz` 的测试/parity 契约；translation unit 内部允许增加接收 `source_frame_rate_hz` 的 helper，main 必须把 SDK/fallback 实际选中的帧率传入该 helper。两条入口必须复用同一 source-time 规范化与状态推进逻辑，并分别测试 300 Hz 与非 300 Hz fallback。

- [ ] **Step 5: 配置化 v2 channel、默认参数与消息 ID**

```cpp
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
```

CLI `--channel` 只接受 `vicon_state_data_v2`；轨迹参数必须有限且正，并通过
`--pelvis-orientation-calib` 加载完整的 rigid-to-pelvis 旋转和平移外参。C++
运行时按 `R_world_pelvis = R_world_rigid @ R_rigid_from_pelvis` 发布完整姿态，
不再使用启动首帧相对 yaw；同一源帧的 base/ball/table 共用一个
`publish_time_us`。pelvis invalid 不能结束球轨迹，base 只在 valid->invalid
转换时发一条 invalid；base/table 发 0，valid/end ball 发正 ID；打印默认降为
`1 Hz`，状态转换/协议错误即时输出。部署外参绑定
`--tracker-name G1Pelvis --base-subject G2Pelvis`，其他 subject fail fast；纯
`--save-table-calib --no-publish` 模式可不提供 pelvis 外参，但保存后立即退出。

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

### Task 3: 对齐 ChingMu 状态机、跨发布端 parity 与 v2 monitor

**Files:**
- Modify: `deploy/mocap_bridge/chingmu_table_lcm_bridge.py: BridgeConfig, BallTracker, make_message, ChingMuTableLcmBridge, CLI`
- Modify: `deploy/mocap_bridge/monitor_vicon_lcm.py`
- Modify: `deploy/mocap_bridge/tests/test_chingmu_ball_tracker_roi.py`
- Create: `deploy/mocap_bridge/tests/test_chingmu_ball_track_v2.py`
- Create: `deploy/mocap_bridge/tests/test_mocap_v2_monitor.py`

**Interfaces:**
- Consumes: Task 2 的 C++ canonical fixture 与 Task 1 schema。
- Produces: Python `BallTrackPhase` / `BallTrackUpdate`、与 C++ 一致的决策序列、只接受 v2 的 monitor/CSV。

测试文件内同时定义以下确定性 helper，避免依赖 SDK 或在线 LCM：`identity_table` 为单位旋转/零平移的 table transform；`raw([x,y,z])` 把米转换为 ChingMu 原始毫米 marker；`python_contract_fixture()` 逐项执行与 C++ `--emit-track-fixture` 完全相同的已筛选 candidate/frame/source-time 序列；`ball_message()` / `message()` 直接构造 Task 1 的生成类型。fixture JSON 固定只包含 `phase,track_id,publish_valid,publish_end,position_world`，并按输入顺序比较。

跨发布端 parity 同时覆盖原始 marker 到候选的公共 ROI 和“收到同一候选序列后”的关联状态机/ID/end 决策。两端共同要求有限坐标、保存桌角排除、`x <= table_length_m - 0.40`、`|y| <= table_width_m / 2`、`z > table_height_m`，新轨迹另要求 `x > 0`；C++ Vicon 的 raw ignore sphere 是允许的额外排除项。已有轨迹跨过近端 `x=0` 时仍允许按公共关联半径保持同一 ID。

- [ ] **Step 1: 写 Python 生命周期与跨语言 parity 失败测试**

```python
def test_short_miss_and_return_keep_the_same_id(identity_table):
    tracker = BallTracker()
    first = tracker.update(
        [raw([0.8, -0.1, 1.0])], frame_number=100,
        source_time_s=1.0, table=identity_table, config=BridgeConfig(),
        allocation_time_us=1_000_000,
    )
    approach = tracker.update(
        [raw([0.5, -0.1, 1.0])], frame_number=101,
        source_time_s=1.01, table=identity_table, config=BridgeConfig(),
        allocation_time_us=1_000_001,
    )
    miss = tracker.update(
        [], frame_number=102, source_time_s=1.02,
        table=identity_table, config=BridgeConfig(),
    )
    returned = tracker.update(
        [raw([-0.02, -0.1, 1.0])], frame_number=103,
        source_time_s=1.03, table=identity_table, config=BridgeConfig(),
    )
    assert first.track_id == approach.track_id == returned.track_id == 1_000_000
    assert not miss.publish_end


def test_cpp_and_python_contract_fixtures_match():
    cpp = json.loads(subprocess.check_output([
        "deploy/mocap_bridge/.build-v2/test_vicon_ball_track_v2",
        "--emit-track-fixture",
    ]))
    assert python_contract_fixture() == cpp
```

- [ ] **Step 2: 写 monitor 合同失败测试**

```python
def test_monitor_defaults_to_v2_and_exports_track_id():
    args = build_arg_parser().parse_args([])
    assert args.channel == "vicon_state_data_v2"
    row = message_csv_row(ball_message(track_id=77), received_monotonic_s=3.0)
    assert row["track_id"] == 77


@pytest.mark.parametrize("name,track_id", [("ball", 0), ("table", 7), ("G2Pelvis", 7)])
def test_monitor_rejects_invalid_subject_id_contract(name, track_id):
    assert message_contract_error(message(name, track_id)) is not None
```

- [ ] **Step 3: 运行测试，确认旧的一帧结束语义和 v1 默认导致失败**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. \
  /home/loco1/miniconda3/envs/rb/bin/python -m pytest -p no:cacheprovider \
  deploy/mocap_bridge/tests/test_chingmu_ball_track_v2.py \
  deploy/mocap_bridge/tests/test_mocap_v2_monitor.py \
  deploy/mocap_bridge/tests/test_chingmu_ball_tracker_roi.py -q
```

Expected: FAIL，至少包含缺少 `track_id` / `publish_end` 或默认 channel 为 v1。

- [ ] **Step 4: 实现与 C++ 同名同值的 Python 合同**

```python
class BallTrackPhase(str, Enum):
    INACTIVE = "inactive"
    ACTIVE = "active"
    MISSING_GRACE = "missing_grace"


@dataclass(frozen=True)
class BallTrackUpdate:
    phase: BallTrackPhase
    track_id: int
    position_world: Optional[np.ndarray]
    publish_valid: bool
    publish_end: bool
```

`BallTracker.update(unlabeled_markers_mm, *, frame_number, source_time_s, table, config, allocation_time_us=None)` 严格复现 Task 2：仅 INACTIVE 接纳 `x>0`，active 允许 `x<=0`，0.35 m 外只 miss，`<0.25 s` 保持 grace，达到阈值只返回一次旧 ID/最后位置的 end。

- [ ] **Step 5: 让 bridge 对每一种 subject 显式赋 ID**

```python
def make_message(name, position_world, quaternion_xyzw, *, track_id: int,
                 frame_number: int, source_time_s: float, valid: bool,
                 publish_time_us: Optional[int] = None):
    message = transformation_t()
    message.name = str(name)
    message.vicon_frame_number = int(frame_number)
    message.vicon_time_s = float(source_time_s)
    message.publish_time_us = int(
        time.time() * 1.0e6 if publish_time_us is None else publish_time_us
    )
    message.track_id = int(track_id)
    message.valid = int(bool(valid))
    message.occluded = int(not bool(valid))
    message.pos_vicon = np.asarray(
        position_world, dtype=np.float64
    ).reshape(3).tolist()
    message.quat_vicon = np.asarray(
        quaternion_xyzw, dtype=np.float64
    ).reshape(4).tolist()
    return message
```

`process_frame()` 对 pelvis/table 传 0；valid/end ball 传 tracker 返回的正 ID。结束消息使用 tracker 的最后有限位置。`BridgeConfig` 和 CLI 默认/校验值与 C++ 完全一致，日志摘要默认 `1 Hz`。

- [ ] **Step 6: 重构 monitor 为可测 helper，并删除逐包输出**

实现三个精确 helper：`build_arg_parser() -> argparse.ArgumentParser` 默认 v2 channel/1 Hz；`message_contract_error(message) -> str | None` 对 ball 非正 ID、非 ball 非零 ID和非有限 pose 返回稳定 reason code；`message_csv_row(message,received_monotonic_s) -> dict[str,object]` 固定写 `track_id,valid,occluded,source_frame,source_time_s,publish_time_us,x,y,z,received_monotonic_s`。callback 只累计 count/last/error transition，主循环以 `next_summary_s += 1.0` 最多 1 Hz输出；仅 contract error 的首次/变化时立即输出。

- [ ] **Step 7: 运行 publisher parity 与 monitor 测试**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
bash deploy/mocap_bridge/build_v2_mocap.sh
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. \
  /home/loco1/miniconda3/envs/rb/bin/python -m pytest -p no:cacheprovider \
  deploy/mocap_bridge/tests/test_chingmu_ball_track_v2.py \
  deploy/mocap_bridge/tests/test_mocap_v2_monitor.py \
  deploy/mocap_bridge/tests/test_chingmu_ball_tracker_roi.py -q
```

Expected: PASS；对相同已筛选 candidate 序列，C++/Python fixture JSON 决策语义相同；两端各自 ROI 回归仍 PASS。

- [ ] **Step 8: 提交 Python publisher 与 monitor 单元**

```bash
git add deploy/mocap_bridge/tests/test_chingmu_ball_track_v2.py \
  deploy/mocap_bridge/tests/test_mocap_v2_monitor.py
git add -p -- deploy/mocap_bridge/tests/test_chingmu_ball_tracker_roi.py
git add -p -- deploy/mocap_bridge/chingmu_table_lcm_bridge.py
git add -p -- deploy/mocap_bridge/monitor_vicon_lcm.py
git diff --cached --check
git diff --cached --name-status
git commit -m "feat: align ChingMu v2 ball identity semantics"
```

---

### Task 4: 建立 canonical runtime 身份类型并完成 `track_epoch -> track_id` 迁移

**Files:**
- Create: `deploy/utils/hitter_runtime_types.py`
- Modify: `deploy/utils/hitter_realtime.py`
- Modify: `deploy/simulator/real_world.py`
- Modify: `deploy/envs/hitter.py`
- Modify: `deploy/diagnostics/hitter_task_models.py`
- Modify: `deploy/diagnostics/hitter_task_attempts.py`
- Modify: `deploy/diagnostics/hitter_task_events.py`
- Modify: `deploy/diagnostics/hitter_task_pipeline.py`
- Modify: `deploy/diagnostics/hitter_task_monitor.py`
- Modify: `deploy/diagnostics/hitter_task_replay.py`
- Modify: `deploy/diagnostics/static/hitter_task_monitor.html`
- Modify: `deploy/tests/test_hitter_runtime_factory.py`
- Modify: `deploy/tests/test_hitter_strike_target_logging.py`
- Modify: `deploy/tests/test_hitter_task_diagnostics_integration.py`
- Modify: `deploy/tests/test_hitter_task_events.py`
- Modify: `deploy/tests/test_hitter_task_frontend.py`
- Modify: `deploy/tests/test_hitter_task_input_adapter.py`
- Modify: `deploy/tests/test_hitter_task_monitor.py`
- Modify: `deploy/tests/test_hitter_task_pipeline.py`
- Create: `deploy/tests/test_hitter_runtime_identity_types.py`
- Create: `deploy/tests/test_mujoco_hitter_track_id_v2.py`

**Interfaces:**
- Consumes: Task 1 wire name `track_id`。
- Produces: `SnapshotKey(track_id, generation)`、`BallEstimateSnapshot.track_id`、`BallEstimateSnapshot.consumed/new_track`、`PlannerResultSnapshot.track_id`；active runtime 不再从 diagnostics 导入身份 key。

`snapshot` fixture 在新测试文件中构造一条 `track_id=9,generation=1,source_frame=1` 的完整 `BallEstimateSnapshot`：position `[1.5,0,1]`、velocity `[-1,0,0]`、有限 base pose、`visible/ready=True`、`consumed=False`、`new_track=True`；它只用于验证字段合同，不依赖 RealWorld。

本任务只前置 `consumed: bool = False` 与 `new_track: bool = False` 的 immutable 字段合同，不实现任何 consume/admission gate。现有生产构造点保持默认 `False`；RealWorld 在 Task 7 才赋 wire-aware 真值，HitterEnv 在 Task 9 才依据这些字段做最终 phase gate。MuJoCo 不得把当前 generation/serve 逻辑提前解释成 `new_track`。

- [ ] **Step 1: 写 canonical type 失败测试**

```python
def test_snapshot_key_requires_positive_track_id():
    assert SnapshotKey(track_id=9, generation=0).to_json_dict() == {
        "schema_version": 2,
        "track_id": 9,
        "generation": 0,
    }
    with pytest.raises(ValueError, match="track_id must be positive"):
        SnapshotKey(track_id=0, generation=0)


def test_runtime_snapshots_expose_no_epoch_attribute(snapshot):
    assert snapshot.track_id == 9
    assert not hasattr(snapshot, "track_epoch")
```

- [ ] **Step 2: 运行测试并做全树旧名称盘点**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  deploy/tests/test_hitter_runtime_identity_types.py
rg -n '\btrack_epoch\b' deploy --glob '*.py' --glob '*.html'
```

Expected: 测试 import/字段失败；`rg` 显示 runtime、diagnostics 和 fixture 的旧名字。

- [ ] **Step 3: 创建不依赖 diagnostics 的 canonical key**

```python
@dataclass(frozen=True, order=True)
class SnapshotKey:
    track_id: int
    generation: int

    def __post_init__(self) -> None:
        if type(self.track_id) is not int or self.track_id <= 0:
            raise ValueError("track_id must be positive")
        if type(self.generation) is not int or self.generation < 0:
            raise ValueError("generation must be a non-negative integer")

    def to_json_dict(self) -> dict[str, int]:
        return {"schema_version": 2, "track_id": self.track_id,
                "generation": self.generation}
```

`hitter_task_models.py` 从该模块 import 并 re-export `SnapshotKey`，避免一次性破坏外部 import；不得复制第二份 key class。

- [ ] **Step 4: 机械迁移 Python 属性、构造参数、JSON key 和日志标签**

把生产路径全部迁为 `track_id`。MuJoCo 的本地测试 ID 从 1 开始且只服务于离线 simulation；RealWorld 暂时保留原行为到 Task 7，但字段名已与 wire 契约一致。当前 v2 录制只写 `track_id`；旧 `track_epoch` 解析在 Task 13 放入隔离适配器，不能留在 active model。

- [ ] **Step 5: 更新现存 fixture 并证明 active tree 没有旧名称**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  deploy/tests/test_hitter_runtime_identity_types.py \
  deploy/tests/test_mujoco_hitter_track_id_v2.py \
  deploy/tests/test_hitter_task_events.py \
  deploy/tests/test_hitter_task_input_adapter.py
test -z "$(rg -l '\btrack_epoch\b' \
  deploy/simulator deploy/envs deploy/utils deploy/diagnostics \
  --glob '*.py' --glob '*.html')"
```

Expected: 指定测试 PASS；`rg` 返回空。

- [ ] **Step 6: 提交 canonical identity rename**

```bash
git add deploy/utils/hitter_runtime_types.py \
  deploy/utils/hitter_realtime.py deploy/diagnostics/hitter_task_models.py \
  deploy/diagnostics/hitter_task_attempts.py \
  deploy/diagnostics/hitter_task_events.py \
  deploy/diagnostics/hitter_task_pipeline.py \
  deploy/diagnostics/hitter_task_replay.py \
  deploy/diagnostics/static/hitter_task_monitor.html \
  deploy/tests/test_hitter_runtime_identity_types.py \
  deploy/tests/test_hitter_strike_target_logging.py \
  deploy/tests/test_hitter_task_events.py \
  deploy/tests/test_hitter_task_frontend.py \
  deploy/tests/test_hitter_task_pipeline.py \
  deploy/tests/test_mujoco_hitter_track_id_v2.py
git add -p -- deploy/simulator/real_world.py deploy/envs/hitter.py
git add -p -- deploy/diagnostics/hitter_task_monitor.py
git add -p -- deploy/tests/test_hitter_runtime_factory.py
git add -p -- deploy/tests/test_hitter_task_diagnostics_integration.py
git add -p -- deploy/tests/test_hitter_task_input_adapter.py
git add -p -- deploy/tests/test_hitter_task_monitor.py
git diff --cached --check
git diff --cached --name-status
git commit -m "refactor: make track id the HITTER runtime identity"
```

在执行该提交前，必须先用 `git diff --cached --name-status` 确认没有把 pre-existing deleted 文件 staged；如有，执行 `git restore --staged <明确路径>` 只取消暂存，不改变工作树。
现有 `deploy/tests/test_mujoco_physical_table_tennis.py` 是实施前用户未跟踪文件；只读参考其中 epoch case 并移植到新建的 v2 测试，禁止编辑或暂存该用户文件。

---

### Task 5: 引入 typed planner failure 与球桌绝对 y 手型规则

**Files:**
- Modify: `deploy/utils/hitter_runtime_types.py`
- Modify: `deploy/utils/hitter_planner.py: vector validation, StrikePlanner.hit_plane_intersection, BaseTargetPlanner.plan, HitterWbcCommand, HitterSystemPlanner.plan_command`
- Modify: `deploy/envs/hitter.py: _plan_hitter_snapshot, _mujoco_planner_result`
- Modify: `deploy/utils/hitter_runtime_factory.py: forced_strike_type`
- Create: `deploy/tests/test_hitter_planner_failure_reasons.py`
- Modify: `deploy/tests/test_hitter_runtime_factory.py`
- Modify: `deploy/tests/test_hitter_strike_target_logging.py`

**Interfaces:**
- Consumes: Task 4 的 canonical `track_id` types。
- Produces: `PlannerFailureReason`、`PlannerRejected`、`strike_type_from_table_y()`；`BaseTargetPlanner.plan(*, racket_target_w, current_base_xy_w, base_forward_xy_w, strike_type: str)` 不再自行判断手型；command 携带 `strike_table_y_w` 与 `strike_side_source="table_y"`。

测试文件内定义 `FixedStrikePlanner`，其 `plan()` 返回指定的有限 `StrikePlan`；`system_planner_with_fixed_strike_point(point)` 用该 stub 和真实 `BaseTargetPlanner` 构造 `HitterSystemPlanner`。`strike_planner` fixture 使用生产边界配置，不 monkeypatch `strike_type_from_table_y()`，因此三点边界测试会走真实自动手型路径。

- [ ] **Step 1: 写 typed reason 和 table-y 三点边界失败测试**

```python
@pytest.mark.parametrize(
    "table_y,expected",
    [(-1.0e-9, "forehand"), (0.0, "backhand"), (1.0e-9, "backhand")],
)
def test_strike_type_uses_absolute_table_y(table_y, expected):
    planner = system_planner_with_fixed_strike_point([0.0, table_y, 1.0])
    for base_y, yaw in [(-0.8, -1.2), (0.0, 0.0), (0.9, 2.1)]:
        forward = [np.cos(yaw), np.sin(yaw)]
        command = planner.plan_command(
            [0.8, 0.0, 1.0], [-2.0, 0.0, 0.0],
            current_base_xy_w=[-0.4, base_y],
            base_forward_xy_w=forward,
        )
        assert command.strike_type == expected
        assert command.strike_table_y_w == pytest.approx(table_y)
        assert command.strike_side_source == "table_y"


def test_no_crossing_has_typed_reason(strike_planner):
    with pytest.raises(PlannerRejected) as caught:
        strike_planner.hit_plane_intersection([0.8, 0.0, 1.0], [-0.01, 0.0, 4.0])
    assert caught.value.reason is PlannerFailureReason.NO_FUTURE_CROSSING
```

同文件以 `pytest.mark.parametrize("scenario,reason", [...])` 逐项覆盖八个 planner reason：ended、estimator-not-ready、base-invalid、not-incoming、no-crossing、height、nonfinite 和 monkeypatched unexpected exception；前七项断言具体 `PlannerRejected.reason`，unexpected exception 在 Task 6 worker 测试断言转 `INTERNAL_ERROR`。控制断言只比较 enum，不比较异常文本。

- [ ] **Step 2: 写真机 override 启动失败测试**

```python
def test_real_world_rejects_forced_strike_type():
    with pytest.raises(ValueError, match="force_strike_type is disabled"):
        forced_strike_type(
            {"force_strike_type": "forehand"}, {}, is_real_world=True
        )


def test_mujoco_may_force_strike_type():
    assert forced_strike_type(
        {"force_strike_type": "forehand"}, {}, is_real_world=False
    ) == "forehand"
```

- [ ] **Step 3: 运行测试，确认当前异常字符串和相对 base lateral y 规则失败**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  deploy/tests/test_hitter_planner_failure_reasons.py \
  deploy/tests/test_hitter_runtime_factory.py \
  deploy/tests/test_hitter_strike_target_logging.py
```

Expected: FAIL，包含缺少 `PlannerRejected` 或零点/不同 base pose 得到错误手型。

- [ ] **Step 4: 定义 planner failure enum 与带 detail 的异常**

```python
class PlannerFailureReason(str, Enum):
    TRACK_ENDED = "TRACK_ENDED"
    ESTIMATOR_NOT_READY = "ESTIMATOR_NOT_READY"
    BASE_POSE_INVALID = "BASE_POSE_INVALID"
    BALL_NOT_INCOMING = "BALL_NOT_INCOMING"
    NO_FUTURE_CROSSING = "NO_FUTURE_CROSSING"
    HIT_HEIGHT_OUT_OF_RANGE = "HIT_HEIGHT_OUT_OF_RANGE"
    NONFINITE_INPUT_OR_OUTPUT = "NONFINITE_INPUT_OR_OUTPUT"
    INTERNAL_ERROR = "INTERNAL_ERROR"


class PlannerRejected(RuntimeError):
    def __init__(self, reason: PlannerFailureReason, detail: str):
        self.reason = PlannerFailureReason(reason)
        self.detail = str(detail)
        super().__init__(f"{self.reason.value}: {self.detail}")
```

- [ ] **Step 5: 把 planner 边界映射为 typed rejection**

`StrikePlanner` 对 admission 失败发 `BALL_NOT_INCOMING`，没有 directed crossing 发 `NO_FUTURE_CROSSING`，高度区间失败发 `HIT_HEIGHT_OUT_OF_RANGE`，输入、trajectory 或输出非有限发 `NONFINITE_INPUT_OR_OUTPUT`。`HitterEnv._plan_hitter_snapshot()` 对 visible/ready/base/incoming confirmation 分别发 `TRACK_ENDED`、`ESTIMATOR_NOT_READY`、`BASE_POSE_INVALID`、`BALL_NOT_INCOMING`。

- [ ] **Step 6: 将手型判定从 BaseTargetPlanner 上移到 system planner**

```python
def strike_type_from_table_y(racket_target_w) -> str:
    target = _vec3(racket_target_w, "racket_target_w")
    return "forehand" if float(target[1]) < 0.0 else "backhand"


resolved_strike_type = (
    strike_type_from_table_y(strike_plan.p_racket_target)
    if strike_type is None
    else validate_explicit_strike_type(strike_type)
)
strike_side_source = "table_y" if strike_type is None else "forced"
resolved_strike_type, p_base_target_xy = self.base_planner.plan(
    racket_target_w=strike_plan.p_racket_target,
    current_base_xy_w=current_base_xy_w,
    base_forward_xy_w=base_forward_xy_w,
    strike_type=resolved_strike_type,
)
```

`BaseTargetPlanner.plan` 的 `strike_type` 改成必填并只校验/使用。`HitterWbcCommand` 增加 `strike_table_y_w` 和 `strike_side_source`；自动模式固定记录 `table_y`，仅 MuJoCo/离线显式 override 记录 `forced`。strike log 同时输出这两个字段。

```python
@dataclass(frozen=True)
class HitterWbcCommand:
    strike_type: str
    p_base_target_xy: np.ndarray
    v_racket_target_w: np.ndarray
    time_to_strike: float
    strike_plan: StrikePlan
    strike_table_y_w: float
    strike_side_source: str
```

- [ ] **Step 7: 在 HitterEnv 初始化时一次性校验真机 override**

缓存 `self.hitter_forced_strike_type`；构造时以 `is_real_world=bool(self.simulator.is_real)` 解析，真机非空在 worker/ONNX 启动前抛错，MuJoCo/离线继续允许。

- [ ] **Step 8: 运行 planner、factory 与日志测试**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  deploy/tests/test_hitter_planner_failure_reasons.py \
  deploy/tests/test_hitter_runtime_factory.py \
  deploy/tests/test_hitter_strike_target_logging.py
```

Expected: PASS；`y=-epsilon/0/+epsilon` 为 forehand/backhand/backhand，改变 base y/yaw 不改变结果。

- [ ] **Step 9: 提交 typed planner 单元**

```bash
git add deploy/utils/hitter_runtime_types.py \
  deploy/tests/test_hitter_planner_failure_reasons.py \
  deploy/tests/test_hitter_strike_target_logging.py
git add -p -- deploy/utils/hitter_planner.py deploy/envs/hitter.py
git add -p -- deploy/utils/hitter_runtime_factory.py
git add -p -- deploy/tests/test_hitter_runtime_factory.py
git diff --cached --check
git diff --cached --name-status
git commit -m "feat: type HITTER failures and select side from table y"
```

---

### Task 6: 将 planner 输出改为容量 64 的有序完成队列

**Files:**
- Modify: `deploy/utils/hitter_realtime.py: PlannerResultSnapshot, PlannerWorkerStats, FrozenPlannerResult, LatestOnlyPlannerWorker`
- Create: `deploy/tests/test_hitter_completed_result_queue.py`

**Interfaces:**
- Consumes: Task 5 的 `PlannerFailureReason` / `PlannerRejected`。
- Produces: `CompletedResultBatch(results, frozen_results, overflowed, overflow_count, overflowed_track_ids)`；`LatestOnlyPlannerWorker.drain_completed_results()`；latest-only pending 输入保持不变。

测试文件内定义：`snapshot(track_id,generation)` 返回 deadline 尚未到期且所有数组有限的 immutable `BallEstimateSnapshot`；`plan_ok(snapshot)` 返回 `PlannerResultSnapshot` 所需的完整有限 `HitterWbcCommand`，deadline 为 `snapshot.received_monotonic_s+1.0`；`plan_in_generation_order` 调用 `plan_ok` 并保留输入 generation；`wait_until(predicate, timeout_s=1.0)` 使用 `time.monotonic()` 的有界轮询并在超时 raise `AssertionError`；`complete_three_sequential_submissions(worker)` 每次 submit 后等待对应 completion 再提交下一条。这样 overflow 测试验证 completed queue，而不是误测 latest-only pending replacement。

- [ ] **Step 1: 写有序、清空、overflow 和 unknown exception 失败测试**

```python
def test_completed_results_are_drained_in_order():
    worker = LatestOnlyPlannerWorker(plan_in_generation_order)
    try:
        for generation in (1, 2, 3):
            worker.submit(snapshot(track_id=7, generation=generation))
            wait_until(lambda: worker.stats.completed + worker.stats.failed == generation)
        batch = worker.drain_completed_results()
        assert [r.source_generation for r in batch.results] == [1, 2, 3]
        assert worker.drain_completed_results().results == ()
    finally:
        assert worker.close(timeout_s=1.0)


def test_capacity_overflow_is_sticky_and_never_replaces_existing_results():
    worker = LatestOnlyPlannerWorker(plan_ok, completed_result_queue_capacity=2)
    complete_three_sequential_submissions(worker)
    batch = worker.drain_completed_results()
    assert [r.source_generation for r in batch.results] == [1, 2]
    assert batch.overflowed is True
    assert batch.overflow_count == 1
    assert batch.overflowed_track_ids == (7,)
    empty = worker.drain_completed_results()
    assert empty.overflowed is False
    assert empty.overflow_count == 0
    assert empty.overflowed_track_ids == ()
    assert worker.stats.completed_result_queue_overflow_total == 1
```

另一个测试用两个 `threading.Event`：planner 在 generation 1 内阻塞，连续 submit 2/3，释放后断言实际执行 generation 为 `[1,3]`、`pending_replaced_total == 1`，证明 pending input 深度仍最多 1 且只保留最新 snapshot；`finally` 必须释放 event 并 close worker。

- [ ] **Step 2: 运行测试，确认当前 `latest_result` 覆盖语义失败**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  deploy/tests/test_hitter_completed_result_queue.py
```

Expected: FAIL，缺少 `completed_result_queue_capacity` 或 `drain_completed_results`。

- [ ] **Step 3: 定义严格的 result 与 batch invariants**

```python
@dataclass(frozen=True)
class PlannerResultSnapshot:
    track_id: int
    source_generation: int
    source_frame: int
    strike_deadline_monotonic_s: float
    completed_monotonic_s: float
    command: object | None
    failure_reason: PlannerFailureReason | None = None
    error_text: str | None = None

    def __post_init__(self) -> None:
        succeeded = self.command is not None and self.failure_reason is None
        failed = self.command is None and self.failure_reason is not None
        if not (succeeded or failed):
            raise ValueError("planner result must be exactly success or failure")


@dataclass(frozen=True)
class CompletedResultBatch:
    results: tuple[PlannerResultSnapshot, ...]
    frozen_results: tuple[FrozenPlannerResult | None, ...]
    overflowed: bool
    overflow_count: int
    overflowed_track_ids: tuple[int, ...]

    def __post_init__(self) -> None:
        if len(self.results) != len(self.frozen_results):
            raise ValueError("results and frozen_results length mismatch")
        if type(self.overflow_count) is not int or self.overflow_count < 0:
            raise ValueError("overflow_count must be a non-negative integer")
        if bool(self.overflowed) != (self.overflow_count > 0):
            raise ValueError("overflowed must match this drain's overflow_count")
        if any(type(track_id) is not int or track_id <= 0
               for track_id in self.overflowed_track_ids):
            raise ValueError("overflowed_track_ids must contain positive integers")
        if self.overflowed_track_ids != tuple(sorted(set(self.overflowed_track_ids))):
            raise ValueError("overflowed_track_ids must be sorted and unique")
        if self.overflowed and not self.overflowed_track_ids:
            raise ValueError("overflow must identify at least one affected track")
        if self.overflow_count < len(self.overflowed_track_ids):
            raise ValueError("overflow count cannot be smaller than unique track ids")
```

- [ ] **Step 4: 在 worker 内 append，不覆盖 completed result**

```python
if len(self._completed_results) >= self._completed_result_queue_capacity:
    self._completed_result_queue_overflow_latched = True
    self._completed_result_queue_overflow_since_drain += 1
    self._completed_result_queue_overflow_total += 1
    self._completed_result_queue_overflow_track_ids_since_drain.add(
        planned.track_id
    )
else:
    self._completed_results.append((planned, frozen))
```

`drain_completed_results()` 在同一 condition lock 内，把 `_completed_result_queue_overflow_since_drain` 和排序后的 `_track_ids_since_drain` 复制进 batch，然后将这两个 since-drain 字段和 sticky bool 清零；`_completed_result_queue_overflow_total` 永不在 drain 时清零，并由 `stats.completed_result_queue_overflow_total` 暴露。禁止用累计 total 构造 batch，否则一次旧 overflow 会污染以后所有 drain。保留 `latest_result_bundle()` 仅供尚未迁移的只读诊断，Task 10 后 production/shadow 均不再依赖它。

- [ ] **Step 5: 对 typed/unknown exception 生成明确 reason**

```python
except PlannerRejected as exc:
    reason = exc.reason
    error_text = exc.detail
except Exception as exc:
    reason = PlannerFailureReason.INTERNAL_ERROR
    error_text = f"{type(exc).__name__}: {exc}"
```

worker 捕获异常后继续线程循环；`FrozenPlannerResult` 同步保存 `failure_reason`，诊断不得再解析 `error_text`。

- [ ] **Step 6: 运行 queue 测试与线程关闭回归**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  deploy/tests/test_hitter_completed_result_queue.py \
  deploy/tests/test_hitter_task_diagnostics_safety.py
```

Expected: PASS；unknown exception 后下一条仍完成；close 不遗留 worker thread。

- [ ] **Step 7: 提交 ordered output 单元**

```bash
git add deploy/utils/hitter_realtime.py \
  deploy/tests/test_hitter_completed_result_queue.py
git add -p -- deploy/tests/test_hitter_task_diagnostics_safety.py
git diff --cached --check
git diff --cached --name-status
git commit -m "feat: queue completed HITTER planner results"
```

---

### Task 7: 实现 RealWorld v2 identity、freshness 与下一发门禁

**Files:**
- Modify: `deploy/simulator/real_world.py: _init_ball_state, _init_communication, _vicon_state_handler, _update_ball_state_from_vicon, reset estimator, poll/status APIs`
- Modify: `deploy/utils/hitter_runtime_factory.py`
- Create: `deploy/tests/test_real_world_v2_consumer.py`

**Interfaces:**
- Consumes: Task 1 v2 schema、Task 4 `BallEstimateSnapshot.track_id`。
- Produces: `ViconInputFault`、`ViconConsumerEvent`、`ViconConsumerStatus`、`begin_hitter_policy_session()`、`hitter_vicon_status()`、`drain_hitter_vicon_events()`、`consume_hitter_track()`；listener 只接收 planning-eligible snapshot。

`test_real_world_v2_consumer.py` 不启动网络线程：`world` fixture 以 `RealWorld.__new__()` 加生产初始化 helper 建立锁、estimator 和 v2 状态；`ball()` / `pelvis()` 构造 Task 1 生成类型；`ingest_valid_pelvis[_and_ball]()` 只调用 `_ingest_vicon_v2_message()`；`encoded_rc()` 使用现有 `wireless_controller_t` 生成合法 payload。测试结束显式关闭任何由 fixture 创建的线程。

- [ ] **Step 1: 写 v2 ID/帧/reset/conflict 失败测试**

```python
def test_same_wire_id_survives_reset_and_rejects_out_of_order_frames(world):
    world._ingest_vicon_v2_message(ball(track_id=7, frame=10), received_monotonic_s=1.0)
    world._ingest_vicon_v2_message(ball(track_id=7, frame=11), received_monotonic_s=1.01)
    generation = world.ball_snapshot_generation
    samples = world.ball_state_estimator.sample_count
    world._ingest_vicon_v2_message(ball(track_id=7, frame=11), received_monotonic_s=1.02)
    world._ingest_vicon_v2_message(ball(track_id=7, frame=9), received_monotonic_s=1.03)
    assert world.ball_snapshot_generation == generation
    assert world.ball_state_estimator.sample_count == samples
    world.reset_ball_state_estimator()
    assert world.active_ball_track_id == 7
    assert world.last_ball_track_id == 7
    assert world.seen_ball_track_ids == {7}


def test_overlapping_new_id_consumes_both_and_latches_conflict(world):
    world._ingest_vicon_v2_message(ball(track_id=7, frame=10), received_monotonic_s=1.0)
    world._ingest_vicon_v2_message(ball(track_id=8, frame=11), received_monotonic_s=1.01)
    status = world.hitter_vicon_status(now_monotonic_s=1.01)
    assert {7, 8} <= world.consumed_ball_track_ids
    assert status.latched_fault is ViconInputFault.TRACK_ID_CONFLICT
```

- [ ] **Step 2: 写 decode/poll-liveness 与 freshness 失败测试**

```python
def test_bad_fingerprint_latches_schema_error_without_raising(world):
    world._vicon_state_handler("vicon_state_data_v2", b"bad fingerprint")
    assert world.vicon_fault_latched is ViconInputFault.VICON_SCHEMA_ERROR
    world._remote_controller_handler("rc_command_data", encoded_rc(r2=True))
    assert world.right_lower_right_switch_pressed is True


def test_stream_stale_is_latched_until_policy_reentry(world):
    ingest_valid_pelvis_and_ball(world, now=1.0, track_id=7)
    status = world.hitter_vicon_status(now_monotonic_s=1.401)
    assert status.latched_fault is ViconInputFault.VICON_STREAM_STALE
    ingest_valid_pelvis(world, now=1.41)
    assert world.hitter_vicon_status(now_monotonic_s=1.42).latched_fault \
        is ViconInputFault.VICON_STREAM_STALE


def test_reentry_clears_recovered_stale_fault_and_accepts_next_id(world):
    ingest_valid_pelvis_and_ball(world, now=1.0, track_id=7)
    world.hitter_vicon_status(now_monotonic_s=1.401)
    ingest_valid_pelvis(world, now=1.41)
    assert world.begin_hitter_policy_session(now_monotonic_s=1.42)
    assert world.hitter_vicon_status(now_monotonic_s=1.92).ready_for_new_serve
    world._ingest_vicon_v2_message(
        ball(track_id=8, frame=20), received_monotonic_s=1.921
    )
    assert world.active_ball_track_id == 8


def test_reentry_rejects_unrecovered_stale_stream(world):
    ingest_valid_pelvis_and_ball(world, now=1.0, track_id=7)
    world.hitter_vicon_status(now_monotonic_s=1.401)
    assert not world.begin_hitter_policy_session(now_monotonic_s=1.402)
    assert world.vicon_fault_latched is ViconInputFault.VICON_STREAM_STALE


def test_invalid_ball_emits_one_direct_track_end_event(world):
    world._ingest_vicon_v2_message(ball(track_id=7, frame=10), received_monotonic_s=1.0)
    world._ingest_vicon_v2_message(
        ball(track_id=7, frame=11, valid=False),
        received_monotonic_s=1.01,
    )
    events = world.drain_hitter_vicon_events()
    assert [(event.reason, event.track_id) for event in events] == [
        (ViconEventReason.TRACK_ENDED, 7)
    ]
    assert world.drain_hitter_vicon_events() == ()
```

同文件用参数化 subject/message case 精确覆盖：ball ID 0/-1 不进 estimator；pelvis/table 非零 ID fail closed；invalid 保留 ID 且只结束一次；新 ID generation 从 1 开始；age `0.400001 s` 触发 ball stale而 `0.40 s` 不触发；startup visible ID进入 quarantine；0.49 s 到达的新 ID立即 consumed且 0.50 s 后不复活；任意有效 ball 清零 no-ball timer；seen/consumed 在 estimator reset 和 policy reset 后保持。每个 case 断言 estimator count、active/last ID、event tuple、fault 和 listener submit count。

新增 estimator 隔离测试：先用 ID 7 填满窗口直至 ready，再收 invalid 7、完整 0.50 s no-ball 和 ID 8 首帧；此时 ID 8 的 generation 必须为 1、`sample_count == 1`、`ready is False`，任何拟合输入都不能含 ID 7 sample。未通过 admission 而被 quarantine/consume 的新 ID 同样先切断旧 estimator window，但不得把自身 sample 加入 planning estimator。

- [ ] **Step 3: 运行测试，确认 v1 hardcode、本地 ID 与无 freshness 导致失败**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  deploy/tests/test_real_world_v2_consumer.py
```

Expected: FAIL，缺少 v2 status API 或仍订阅 `vicon_state_data`。

- [ ] **Step 4: 增加 v2 状态类型和 process-lifetime 集合**

```python
class ViconInputFault(str, Enum):
    VICON_SCHEMA_ERROR = "VICON_SCHEMA_ERROR"
    VICON_STREAM_STALE = "VICON_STREAM_STALE"
    TRACK_ID_CONFLICT = "TRACK_ID_CONFLICT"
    BALL_MESSAGE_STALE = "BALL_MESSAGE_STALE"


class ViconEventReason(str, Enum):
    TRACK_ENDED = "TRACK_ENDED"
    BASE_POSE_INVALID = "BASE_POSE_INVALID"
    VICON_SCHEMA_ERROR = "VICON_SCHEMA_ERROR"
    VICON_STREAM_STALE = "VICON_STREAM_STALE"
    BALL_MESSAGE_STALE = "BALL_MESSAGE_STALE"
    TRACK_ID_CONFLICT = "TRACK_ID_CONFLICT"


@dataclass(frozen=True)
class ViconConsumerEvent:
    sequence: int
    reason: ViconEventReason
    track_id: int | None
    detail: str


@dataclass(frozen=True)
class ViconConsumerStatus:
    stream_fresh: bool
    ball_fresh: bool
    base_pose_valid: bool
    active_track_id: int | None
    last_track_id: int | None
    visible: bool
    ready_for_new_serve: bool
    latched_fault: ViconInputFault | None
```

`ViconInputFault` 只表示 status 中需要跨包锁存的 fault；`ViconEventReason` 表示送往 lifecycle 的一次性 transition，因而额外包含 `TRACK_ENDED` 与 `BASE_POSE_INVALID`。两者不得混作一个 enum或裸字符串。Task 8 的 `LifecycleCancelReason` 必须覆盖 `ViconEventReason` 的全部同名 value；参数化 contract test 对每个 `ViconEventReason` 断言 `LifecycleCancelReason(reason.value)` 成功。

初始化 `active_ball_track_id`、`last_ball_track_id`、per-ID generation/frame、seen/consumed/admitted sets、last-any/last-ball monotonic、no-ball start、policy-session flag，以及按 sequence 排序、容量固定为 64 的 transition-event deque；这些集合不得在 estimator reset 或 HitterEnv lifecycle reset 清空。队列满时禁止静默丢弃单个 transition：必须把队列原子替换成一个永久锁存的 `VICON_SCHEMA_ERROR` overflow event，使 lifecycle fail closed。`BallEstimateSnapshot` 增加 `consumed: bool` 与 `new_track: bool`，供 diagnostics 和 HitterEnv 在 listener 边界做最终 phase gate。

`ViconConsumerSettings` 还必须解析严格的 `base_subject`，默认值与两个 active publisher 一致为精确字符串 `G2Pelvis`；空值、保留名 `ball/table` 或非字符串在进入真机前拒绝。RealWorld 只接受配置指定的 pelvis subject，subject 不匹配按永久 `VICON_SCHEMA_ERROR` fail closed。测试 fixture 不能依赖工作树中的 G1/G2 未提交覆盖，提交后必须从 Git archive 复测干净快照。

`ready_for_new_serve` 的生产表达式固定为：policy session 已开启、没有 latched fault、stream fresh、pelvis valid、当前没有 visible/active ID，且 `now-no_ball_since >= 0.50 s`。这里不检查 HitterEnv phase；phase 在 Task 9 listener 临界区做最终 gate。一个 ID 只有首次包可被加入 admitted set；admitted 后该 ID 的后续递增 frame 可在 `TRACKING/ARMED` 继续细化，不能再次当作“新发球”检查。

- [ ] **Step 5: 用 callback 最外层异常边界隔离 LCM poll thread**

```python
def _vicon_state_handler(self, channel, data):
    received = time.monotonic()
    try:
        msg = transformation_t.decode(data)
    except Exception as exc:
        self._latch_vicon_fault(
            ViconInputFault.VICON_SCHEMA_ERROR,
            received_monotonic_s=received,
            detail=f"{type(exc).__name__}: {exc}",
        )
        return
    self._ingest_vicon_v2_message(msg, received_monotonic_s=received)
```

删除逐 ball `print`。subscription channel 来自已校验 settings，且只能为 `vicon_state_data_v2`。

- [ ] **Step 6: 实现 wire ID/帧/invalid/conflict 规则**

先校验完整 wire contract：subject-ID 合法，source frame 为非负整数，`valid/occluded` 只能是互补的 `1/0` 或 `0/1`，三维位置与四元数均有限且四元数非零；table 只允许 valid。ball 必须正 ID，配置指定的 pelvis/table 必须 0。每个新 ID 的 generation 从 1 开始；同 ID 仅严格递增 source frame 增加 generation。任何未见 wire ID 到达时，必须在把它设为 active 或评估其 sample 之前 reset 单一 `BallStateEstimator` 的 samples/readiness，确保旧 ID 的位置永不参与新 ID 速度拟合；然后用该包到达前的 no-ball 时长、stream/base/session 状态决定是否加入 admitted set，再清零 no-ball timer。任何通过 wire contract 的有效 ball（包括重复或乱序 frame）都必须在 frame 早退前清零 no-ball timer，但重复/乱序 frame 不推进 generation、estimator 或 planner。未满足条件立即加入 consumed set，且不把该包加入 planning estimator。外部 estimator reset 仍只清 samples/readiness，不改 identity/history。invalid ball 必须带当前正 ID：校验它等于 `active_ball_track_id` 后，令 active 变 `None`、保留 `last_ball_track_id` 和 per-ID history，清 visible并 enqueue 一次 `TRACK_ENDED`，同时开始 no-ball 计时。这里“不改身份”指结束消息、history 和诊断仍归旧 ID，不是让旧 ID继续占据 active slot。active 时出现另一个 ID，无论该 ID 是否已在 process lifetime 见过，都先切断 estimator window，再消费双方、latch 并 enqueue `TRACK_ID_CONFLICT`。pelvis 从 valid 变 invalid 时 enqueue `BASE_POSE_INVALID`。已消费 ID 保留 diagnostics 可见性但不通知 planner listener；reentry quarantine 也必须同步把 latest snapshot 的 `consumed` 标志改为真。

新增测试固定结束后的双字段语义：ID 7 invalid 后 `status.active_track_id is None`、`status.last_track_id == 7`，per-ID generation/frame 仍保留；连续 no-ball 到 `0.50 s` 且其余 gate 有效时 `ready_for_new_serve is True`，随后首次出现的未见 ID 8 才能成为 active。

- [ ] **Step 7: 实现 tick 驱动的 freshness 与 policy reentry**

```python
def hitter_vicon_status(self, *, now_monotonic_s: float) -> ViconConsumerStatus:
    with self._hitter_ball_state_lock:
        self._apply_freshness_locked(float(now_monotonic_s))
        return self._status_locked(float(now_monotonic_s))
```

任意 v2 包 age `>0.40`：base invalid、active ball ended、latch 并 enqueue 一次 stream stale。active ball 包 age `>0.40`：结束该 ID并 enqueue 一次 ball stale。`drain_hitter_vicon_events()` 在锁内按 sequence 返回并清空这些 transition；普通数据恢复不清 latch。`begin_hitter_policy_session()` 先重新计算 freshness/subject contract：若 stream 仍 stale、pelvis invalid，或存在不可恢复的 `VICON_SCHEMA_ERROR`（当前进程收到过错误 fingerprint，无法证明部署类型一致），返回 false/raise 且不清 fault；只有 stream fresh、pelvis valid 且没有协议错误时，才原子清上一 policy session 的可恢复 `VICON_STREAM_STALE/BALL_MESSAGE_STALE/TRACK_ID_CONFLICT` latch 与 event-dedupe flags、开启新 session，并消费当时可见 ID。若当时无球，则从 reentry 时刻开始连续 no-ball 计时。schema fault 需要修复发布/接收部署并重启进程，不能用 R2/reentry 清除。

任何永久 `VICON_SCHEMA_ERROR`（包括 transition queue overflow）必须在锁存 fault 的同一临界区立即 quarantine 当前 active ID：加入 consumed set、把 matching latest snapshot 重建为 `consumed=True`、切断 estimator samples/readiness，但保留 active/visible 身份仅供物理可见性诊断。此后同 ID 或任何新 ID 的有效包均不得再进入 estimator 或 planner listener。conflict challenger 无论是否已见，都必须把 `ball_track_last_frames[challenger]` 更新为已接收 frame 的最大值，但不得推进 generation 或取得 active authority；这样 conflict 后的旧帧 replay 仍被 watermark 拒绝。

- [ ] **Step 8: 运行 consumer tests 和通信线程关闭回归**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  deploy/tests/test_real_world_v2_consumer.py \
  deploy/tests/test_real_world_connection_wait.py
```

Expected: PASS；坏 payload 后 RC handler 仍可执行；恢复包不能自行解除 stale/schema latch。

- [ ] **Step 9: 提交 RealWorld v2 consumer 单元**

```bash
git add deploy/tests/test_real_world_v2_consumer.py
git add -p -- deploy/simulator/real_world.py
git add -p -- deploy/utils/hitter_runtime_factory.py
git diff --cached --check
git diff --cached --name-status
git commit -m "feat: consume authoritative HITTER track ids"
```

---

### Task 8: 实现单拍 lifecycle、命令锁定、撤拍与 commit window

**Files:**
- Modify: `deploy/utils/hitter_realtime.py: CommandPhase, HitterCommandLifecycle`
- Create: `deploy/tests/test_hitter_single_shot_lifecycle.py`
- Create: `deploy/tests/hitter_test_factories.py`

**Interfaces:**
- Consumes: Task 5 typed reason/command、Task 6 ordered results、Task 7 consumed ID 语义。
- Produces: `LifecycleCancelReason`、`LifecycleDecision`、`ingest()` / `advance()` / `cancel()`；每 ID 最多一个 strike transition。

跨 Task 复用的 `command(side,base,position,velocity)`、`success()` 与 `failure()` 放在 `hitter_test_factories.py`：前者构造完整 `HitterWbcCommand`，后两者构造满足 Task 6 invariant 的结果。只属于 lifecycle 场景的 `armed_lifecycle*()`、`lifecycle_after_terminal()` 与 `lifecycle_in_recovery()` 放在 `test_hitter_single_shot_lifecycle.py`；它们只通过公开 `ingest/advance/cancel` 建立状态，不直接篡改私有字段。Task 10 复用 factory，不复制命令/result 构造逻辑。

Task 8 是纯状态机迁移：`ingest()` 不得隐式调用 `advance()`，caller 必须先显式调用 `advance(now=...)` 并应用其 decision，之后才按序 ingest completed results；否则 strike 消费 transition 会被吞掉。`advance()`、`ingest()`、`cancel()` 和 `mark_track_ended()` 始终返回 `LifecycleDecision`，无状态变化时也返回 `kind="none"`。Task 9 负责在同一后续提交中迁移 `HitterEnv` 与 `test_hitter_strike_target_logging.py`；Task 10 负责迁移 diagnostics pipeline 与 `test_hitter_task_pipeline.py`。禁止给 `LifecycleDecision` 增加和裸字符串相等的兼容行为。

实现前合同裁决：

- `cancel()` 的当前身份先取 `active_result.track_id`，没有 active command 时取正在 `TRACKING` 的 `_current_track_id`；pre-commit cancel 原子消费当前 ID 与 event ID，避免同 tick queued success 重 arm。
- success 和 failure 共用每 ID generation watermark。只接受严格递增 generation；duplicate/older result 不改变 failure streak，matching active ID 的递增 soft failure 才能累计，continuous success 才清零。
- 每次 `advance()` 最多跨一个 phase。即使 `now` 已同时越过 strike 与 recovery deadline，第一次也只产生 `struck` 并停在 `RECOVERY`，下一次才产生 `entered_waiting`；zero-duration recovery 亦然。
- commit、`policy_tts()` 与 strike crossing 只使用第一次 ARMED 锁定的 deadline。`TTS <= 0.30` 时整条 command 冻结；合法 result 仍推进 generation watermark并可记录 failure，但不改变 command、failure streak 或 recovery deadline。commit 内 cancel 仍消费相关 ID，但保留已承诺 command，随后仍恰好产生一次 strike。
- continuous override 必须重建 frozen command/result：只采用新 position 与 velocity；同一 velocity 同步写入 `HitterWbcCommand.v_racket_target_w` 和 `StrikePlan.v_racket_target`，`strike_table_y_w` 同步为新 position y；强制保留 locked side/base/deadline、recovery deadline、side source、time-to-strike、plan t_strike 与 ball in/out，并防御性复制数组。
- `entered_waiting` 表示需要 Task 9 捕获 WAITING anchor 的语义边沿，而不只是 enum 发生变化。late skip、首次未 arm track end、安全撤拍、recovery complete 为 true；重复/已消费事件和 recovery 内 cancel 为 false。
- `LifecycleCancelReason` 必须与全部 `PlannerFailureReason` 及 `ViconEventReason` 同值覆盖；公开输入采用 exact positive `int`，拒绝 bool/float/string；decision kind 使用固定 vocabulary；`consumed_track_ids` 返回本 decision 涉及的全部相关 ID，即使此前已消费。
- 默认和约束采用最终配置：`waiting_tts=0.92`、`arm_tts=0.92`、`minimum_arm_tts=0.30`、`maximum_policy_tts=0.92`、连续 failure 数 `3`、commit `0.30`、position/velocity/deadline override 阈值 `0.05/0.75/0.05`、swing duration `1.85`。所有数值必须 finite 且满足 `0 <= commit <= minimum_arm <= arm <= maximum_policy`、`0 <= waiting_tts <= maximum_policy`；failure count 为 exact positive int，阈值为 finite nonnegative。sampler 异常或非法结果转为 typed `INTERNAL_ERROR` 撤拍，不得从 policy tick 外泄。
- process-lifetime 的 consumed IDs、消费原因、generation watermark、strike count、最后锁定字段和最后 failure/cancel 原因不能因普通 transition 清除。Task 9 不得在同一 policy 进程内重新创建 lifecycle；只有进程启动时创建一次，普通 env reset 必须复用该实例。若未来确需重建，必须显式迁移这些身份状态。
- Task 8 提供公开 `reset_for_policy_reentry() -> LifecycleDecision`（或等价明确命名），供 Task 9 在普通 env reset/显式 policy reentry 时复用同一实例。该 transition 不受普通 commit/RECOVERY cancel freeze 限制：原子消费 active 或 TRACKING current ID，清 active command、recovery deadline、failure streak与当前瞬态身份，进入 WAITING并返回 `kind="session_reset"` / `entered_waiting=True`；纯初始状态也产生一次初始 WAITING anchor 边沿。它必须保留 consumed IDs/原因、每 ID generation watermark、strike count与最后诊断字段，旧 ID 后续 result 只能 `ignored_consumed`。

- [ ] **Step 1: 写 consumed、recovery 与 late skip 失败测试**

```python
@pytest.mark.parametrize("terminal", ["skipped", "cancelled", "struck", "ended"])
def test_terminal_track_id_can_never_arm_again(terminal):
    lifecycle = lifecycle_after_terminal(track_id=7, terminal=terminal)
    decision = lifecycle.ingest(success(track_id=7, generation=99), now=2.0)
    assert decision.kind == "ignored_consumed"
    assert lifecycle.phase is CommandPhase.WAITING
    assert 7 in lifecycle.consumed_track_ids


def test_recovery_consumes_new_id_without_caching():
    lifecycle = lifecycle_in_recovery(track_id=7)
    decision = lifecycle.ingest(success(track_id=8, generation=1), now=1.1)
    assert decision.kind == "consumed_during_recovery"
    assert 8 in lifecycle.consumed_track_ids
    lifecycle.advance(now=3.0)
    assert lifecycle.phase is CommandPhase.WAITING
    assert lifecycle.active_result is None
```

- [ ] **Step 2: 写锁定、override、连续失败与 commit 边界失败测试**

```python
def test_first_arm_locks_side_base_and_deadline():
    lifecycle = armed_lifecycle(command=command("forehand", base=[-0.4, -0.2]),
                                 deadline=2.0, now=1.1)
    changed = success(track_id=7, generation=2, strike_type="backhand",
                      base=[-0.4, 0.4], deadline=2.01)
    lifecycle.ingest(changed, now=1.2)
    assert lifecycle.locked_strike_type == "forehand"
    assert np.allclose(lifecycle.locked_base_target_xy, [-0.4, -0.2])
    assert lifecycle.locked_strike_deadline_monotonic_s == 2.0


@pytest.mark.parametrize(
    "field,delta",
    [
        ("position_m", 0.050001),
        ("velocity_mps", 0.750001),
        ("deadline_s", 0.050001),
    ],
)
def test_third_override_discontinuity_cancels(field, delta):
    lifecycle = armed_lifecycle_before_commit()
    for expected_count in (1, 2):
        decision = lifecycle.ingest(
            discontinuous_result(field=field, delta=delta), now=1.0
        )
        assert decision.kind == "retained_discontinuity"
        assert lifecycle.consecutive_failure_count == expected_count
    decision = lifecycle.ingest(
        discontinuous_result(field=field, delta=delta), now=1.0
    )
    assert decision.entered_waiting
    assert decision.cancel_reason is LifecycleCancelReason.OVERRIDE_DISCONTINUITY


def test_conflict_event_consumes_active_and_event_ids_atomically():
    lifecycle = armed_lifecycle_before_commit(track_id=7)
    decision = lifecycle.cancel(
        reason=LifecycleCancelReason.TRACK_ID_CONFLICT,
        now=1.0,
        track_id=8,
    )
    assert decision.entered_waiting
    assert decision.consumed_track_ids == (7, 8)
    assert {7, 8} <= lifecycle.consumed_track_ids
    assert lifecycle.ingest(success(track_id=7, generation=2), now=1.0).kind \
        == "ignored_consumed"
    assert lifecycle.ingest(success(track_id=8, generation=1), now=1.0).kind \
        == "ignored_consumed"
```

`discontinuous_result(field,delta)` 从当前已接受命令复制所有字段，只修改 `field` 指定的一个量，因此三种单位不会混用。另测 TTS 恰好 `0.30` 时 success/failure 均不改变 frozen command。

- [ ] **Step 3: 运行 lifecycle 测试，确认现有 recovery cache/全命令覆盖行为失败**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  deploy/tests/test_hitter_single_shot_lifecycle.py
```

Expected: FAIL，显示新 ID 被 cached、failure ignored 或 active command 整体被覆盖。

- [ ] **Step 4: 定义 decision/cancel reason 和锁定状态**

```python
class LifecycleCancelReason(str, Enum):
    TRACK_ENDED = "TRACK_ENDED"
    ESTIMATOR_NOT_READY = "ESTIMATOR_NOT_READY"
    BASE_POSE_INVALID = "BASE_POSE_INVALID"
    BALL_NOT_INCOMING = "BALL_NOT_INCOMING"
    NO_FUTURE_CROSSING = "NO_FUTURE_CROSSING"
    HIT_HEIGHT_OUT_OF_RANGE = "HIT_HEIGHT_OUT_OF_RANGE"
    NONFINITE_INPUT_OR_OUTPUT = "NONFINITE_INPUT_OR_OUTPUT"
    INTERNAL_ERROR = "INTERNAL_ERROR"
    VICON_SCHEMA_ERROR = "VICON_SCHEMA_ERROR"
    VICON_STREAM_STALE = "VICON_STREAM_STALE"
    BALL_MESSAGE_STALE = "BALL_MESSAGE_STALE"
    TRACK_ID_CONFLICT = "TRACK_ID_CONFLICT"
    RESULT_QUEUE_OVERFLOW = "RESULT_QUEUE_OVERFLOW"
    OVERRIDE_DISCONTINUITY = "OVERRIDE_DISCONTINUITY"


@dataclass(frozen=True)
class LifecycleDecision:
    kind: str
    track_id: int | None
    command_changed: bool = False
    entered_waiting: bool = False
    cancel_reason: LifecycleCancelReason | None = None
    consumed_track_ids: tuple[int, ...] = ()

    def __post_init__(self) -> None:
        canonical = tuple(sorted(set(self.consumed_track_ids)))
        if canonical != self.consumed_track_ids or any(
            type(track_id) is not int or track_id <= 0 for track_id in canonical
        ):
            raise ValueError("consumed_track_ids must be sorted unique positive ids")
```

state 保存 process-lifetime `consumed_track_ids`、current/last generation、locked fields、failure streak、strike count；删除 `cached_result` 和恢复后自动 arm。所有公开 transition 返回本次新消费涉及的 `consumed_track_ids`，HitterEnv 不根据 `kind` 猜测消费语义。

所有 deadline crossing 只由显式 `advance()` 产生：`ARMED -> RECOVERY` 返回 `kind="struck"`、消费 active ID并把该 ID 的 strike count 加一；`RECOVERY -> WAITING` 返回 `kind="entered_waiting"` 且不得重复报告消费；其他 tick 返回 `kind="none"`。同一 ID 的 `strike_count_by_track_id` 永远不超过 1。

- [ ] **Step 5: 实现 unarmed、late、recovery 和 deadline transitions**

late 是 `remaining < minimum_arm_tts(0.30)`：立即 consume，并在 decision 中返回该 ID 与 `entered_waiting=True`。deadline crossing 先 consume active ID，再进入 RECOVERY、把该 ID strike count 加一并返回在 `consumed_track_ids`。RECOVERY 收到任意新 ID立即 consume并在 decision 中返回；结束时只进入 WAITING，不能重复报告消费。

```python
def consume_track(self, track_id: int, *, reason: str) -> None:
    identity = int(track_id)
    if identity <= 0:
        raise ValueError("track_id must be positive")
    self.consumed_track_ids.add(identity)
    self.last_consumption_reason_by_track_id[identity] = str(reason)


def mark_track_ended(self, track_id: int, *, now: float) -> LifecycleDecision:
    return self.cancel(
        reason=LifecycleCancelReason.TRACK_ENDED,
        now=now,
        track_id=int(track_id),
    )
```

- [ ] **Step 6: 实现 pre-commit override continuity**

```python
same_track = result.track_id == active.track_id
same_side = result.command.strike_type == self.locked_strike_type
position_delta = np.linalg.norm(
    result.command.strike_plan.p_racket_target
    - active.command.strike_plan.p_racket_target
)
velocity_delta = np.linalg.norm(
    result.command.v_racket_target_w - active.command.v_racket_target_w
)
deadline_delta = abs(
    result.strike_deadline_monotonic_s
    - self.locked_strike_deadline_monotonic_s
)
continuous = (
    same_track and same_side
    and position_delta <= 0.05
    and velocity_delta <= 0.75
    and deadline_delta <= 0.05
)
```

合格更新只替换 racket position/velocity；强制写回 locked side/base/deadline。成功清 soft failure streak。超界只计 `OVERRIDE_DISCONTINUITY`，不覆盖 active。

- [ ] **Step 7: 实现 immediate/third-soft cancel 与原子清理**

immediate set：`TRACK_ENDED, BASE_POSE_INVALID, NONFINITE_INPUT_OR_OUTPUT, INTERNAL_ERROR` 加外部传入的 schema/stream-stale/ball-stale/conflict/overflow。soft set：`ESTIMATOR_NOT_READY, BALL_NOT_INCOMING, NO_FUTURE_CROSSING, HIT_HEIGHT_OUT_OF_RANGE, OVERRIDE_DISCONTINUITY`。pre-commit immediate 一次 cancel，soft 第三个有序 result cancel；commit window 只记录，不改命令。`cancel()` 在 `TTS <= 0.30` 时返回 `retained_committed`；否则在同一临界区清 active、consume ID、进入 WAITING并返回 reason。遥控器停止/下电继续走现有底层路径，不调用这个可冻结的普通 planner cancel API。

```python
def cancel(self, *, reason: LifecycleCancelReason, now: float,
           track_id: int | None = None) -> LifecycleDecision:
    active = self.active_result
    active_id = None if active is None else int(active.track_id)
    event_id = None if track_id is None else int(track_id)
    if event_id is not None and event_id <= 0:
        raise ValueError("track_id must be positive")
    ids_to_consume = {identity for identity in (active_id, event_id)
                      if identity is not None}
    self.consumed_track_ids.update(ids_to_consume)
    if self.phase is CommandPhase.RECOVERY:
        self.last_cancel_reason = reason
        return LifecycleDecision(
            kind="retained_recovery",
            track_id=event_id,
            cancel_reason=reason,
            consumed_track_ids=tuple(sorted(ids_to_consume)),
        )
    if active is not None and (
        active.strike_deadline_monotonic_s - float(now)
        <= self.commit_time_to_strike_s + self._EPSILON
    ):
        return LifecycleDecision(
            kind="retained_committed", track_id=active.track_id,
            cancel_reason=reason,
            consumed_track_ids=tuple(sorted(ids_to_consume)),
        )
    cancelled_id = active_id if active_id is not None else event_id
    was_waiting = self.phase is CommandPhase.WAITING
    self.active_result = None
    self.command_end_deadline_s = None
    self.phase = CommandPhase.WAITING
    self.last_cancel_reason = reason
    return LifecycleDecision(
        kind="cancelled", track_id=cancelled_id,
        entered_waiting=not was_waiting,
        cancel_reason=reason,
        consumed_track_ids=tuple(sorted(ids_to_consume)),
    )
```

当 conflict event 携带 ID 8 而 active ID 为 7 时，上述原子操作必须同时消费 `{7,8}`；即使同 tick completed queue 后面还有 ID 7/8 的成功结果，也只能得到 `ignored_consumed`。进入 commit window 后不修改 active command，但仍消费 event ID 和 active ID，从身份入口阻止任何重 arm。

RECOVERY 参数化测试以 `reason` 取 `TRACK_ENDED,VICON_STREAM_STALE,TRACK_ID_CONFLICT,RESULT_QUEUE_OVERFLOW`，记录调用前 recovery deadline；每次断言 decision.kind=`retained_recovery`、phase 仍 RECOVERY、deadline 不变、事件 ID进入 consumed。新 ID success case 断言 `consumed_during_recovery`。只有 `advance(now=deadline)` 产生一次 `entered_waiting`。

- [ ] **Step 8: 运行完整 lifecycle 边界测试**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  deploy/tests/test_hitter_single_shot_lifecycle.py
```

Expected: PASS；每个测试中的 `strike_count_by_track_id[track_id] <= 1`。

- [ ] **Step 9: 提交 single-shot lifecycle 单元**

```bash
git add deploy/utils/hitter_realtime.py \
  deploy/tests/test_hitter_single_shot_lifecycle.py \
  deploy/tests/hitter_test_factories.py
git diff --cached --check
git diff --cached --name-status
git commit -m "feat: enforce one strike per HITTER track id"
```

---

### Task 9: 集成 HitterEnv、50 Hz 配置与真机 WAITING 固定 anchor

**Files:**
- Modify: `deploy/simulator/real_world.py`
- Modify: `deploy/utils/hitter_runtime_factory.py`
- Modify: `deploy/config/mimic/hitter.yaml`
- Modify: `deploy/config/hitter.yaml`
- Modify: `deploy/config/sim/real_world.yaml`
- Modify: `deploy/envs/hitter.py`
- Modify: `deploy/agents/hitter_agent.py`
- Modify: `deploy/tests/test_hitter_runtime_factory.py`
- Modify: `deploy/tests/test_hitter_strike_target_logging.py`
- Create: `deploy/tests/test_hitter_waiting_anchor.py`
- Create: `deploy/tests/test_hitter_runtime_single_shot_integration.py`
- Modify: `deploy/tests/test_hitter_policy_first_frame_transition.py`
- Modify: `deploy/tests/test_hitter_task_observation.py`
- Modify: `deploy/tests/test_real_world_v2_consumer.py`
- Create: `deploy/tests/hitter_runtime_test_harness.py`

**Interfaces:**
- Consumes: Task 6 batch drain、Task 7 status/consume API、Task 8 decisions。
- Produces: 每 policy tick 的固定处理顺序、全参数校验、真机 waiting anchor edge capture；MuJoCo 行为不变。

实现前合同裁决：

- Task 7 已落地的 nested `motion.vicon_consumer.{channel,base_subject,stream_timeout_s,ball_timeout_s,new_serve_no_ball_s,event_queue_capacity}` 是唯一生产配置源；禁止再增加 motion 顶层或 ball_planner 下的同义字段。`ViconConsumerSettings` 单独解析/测试这些字段，`HitterRuntimeSettings` 只保存 planner/lifecycle 参数。下方示例中的旧长字段名按此裁决替换。
- Task 9 扩展 RealWorld consumer API：`end_hitter_policy_session()` 关闭 admission 并原子 quarantine 当前 active；成功的 `begin_hitter_policy_session()` 同一临界区清除上个 session 遗留的可恢复 transition events；`hitter_snapshot_planning_eligible(snapshot)` 在 consumer lock 内二次检查 session/fault/active/admitted/consumed/latest key；`consume_hitter_track(track_id, reason=...)` 保存诊断原因。锁顺序固定为 HitterEnv lifecycle RLock 后取 consumer state lock，RealWorld 始终锁外调用 listener。
- 同一进程仅创建一次 lifecycle 和 worker。reset/reentry 先关闭 snapshot admission 与 consumer session，调用 Task 8 `reset_for_policy_reentry()`，把 consumer active、所有曾 submitted/outstanding ID、已排队 batch/overflow ID 全部同步为 consumed，并丢弃 reset 前结果；in-flight 结果稍后完成也只能 `ignored_consumed`。禁止普通 reset `new HitterCommandLifecycle()`。
- 真机每个 agent iteration 只在 ONNX 前执行一次固定 lifecycle tick；real-world `_post_physics_step()` 不再第二次 drain/advance，MuJoCo 保留原 post-physics 路径。事件、advance、batch 的顺序严格为 status/freshness -> drain events/cancel -> one `advance()` -> one completed batch drain/ordered ingest。
- production result path 完全移除 `latest_result()`/字符串错误/缓存 recovery 兼容路径。每条 decision 后只从 `lifecycle.active_result.command` 更新 policy command，不能从被拒绝或被重建前的 incoming command 复制。
- 首帧平滑期间 consumer session 和 planner admission 保持关闭。reset 后先捕获初始 WAITING anchor 供第一帧 observation 使用；现有 5 秒 PD transition 完成后再刷新 pelvis、重新捕获当前位置 anchor、丢弃/消费旧 worker 结果并成功 begin session，从该时刻开始 0.50 s no-ball 计时。transition 为 0 时在 reset 完成后立即执行相同 reentry 流程。
- `BASE_POSE_INVALID` 无论 lifecycle 当前是 WAITING、pre-commit ARMED 还是 committed，都立即锁存 `_waiting_anchor_fault` 并保留已有 anchor；同 session 内 pose 恢复不能重新 admission，只有显式成功 reentry 才清 fault。每个 `entered_waiting` 边沿捕获前先刷新 simulator state，且每个 decision 只捕获一次。
- listener 在 lifecycle lock 内依次检查 runtime accepting、anchor fault、process-lifetime consumed、phase 和 consumer eligibility；RECOVERY 的旧/新 snapshot 都消费，非 WAITING 的 `new_track` 也消费。rate gate 与非阻塞 `worker.submit()` 在同一 lifecycle lock 内完成，提交前登记 process-lifetime submitted ID，消除 phase-check/submit TOCTOU 与 Task 7 deferred stale-listener race。
- shared test harness 固定放在 `deploy/tests/hitter_runtime_test_harness.py`，不得通过恢复已删除旧测试取得 fixture。`test_hitter_strike_target_logging.py` 必须随本任务一并迁移和暂存。

新增测试文件共享一个纯离线 harness：`make_hitter_env_for_test(is_real, settings)` 只分配 HitterEnv 的 lifecycle/planner/anchor 字段并注入 fake simulator；fake simulator 实现 Task 7 的 status/event/consume API；`agent_harness.run_ticks()` 调用真实 observation 组装、mock ONNX session 和 spy `apply_action()`。所有时钟由显式 `now` 驱动，不使用 sleep，也不创建 Unitree publisher。

- [ ] **Step 1: 写所有默认值和非法配置失败测试**

```python
def test_single_shot_defaults_are_exact(settings):
    assert settings.planner_update_rate_hz == 50.0
    assert settings.waiting_tts_s == 0.92
    assert settings.arm_tts_s == 0.92
    assert settings.minimum_arm_tts_s == 0.30
    assert settings.maximum_policy_tts_s == 0.92
    assert settings.completed_result_queue_capacity == 64
    assert settings.armed_cancel_consecutive_failures == 3
    assert settings.commit_time_to_strike_s == 0.30
    assert settings.maximum_racket_target_override_delta_m == 0.05
    assert settings.maximum_racket_velocity_override_delta_mps == 0.75
    assert settings.maximum_strike_deadline_override_delta_s == 0.05


def test_vicon_defaults_are_exact(vicon_settings):
    assert vicon_settings.channel == "vicon_state_data_v2"
    assert vicon_settings.base_subject == "G2Pelvis"
    assert vicon_settings.stream_timeout_s == 0.40
    assert vicon_settings.ball_timeout_s == 0.40
    assert vicon_settings.new_serve_no_ball_s == 0.50
    assert vicon_settings.event_queue_capacity == 64
```

参数化拒绝 NaN/inf/0/negative；queue/count 的 bool、float、0 和负数均拒绝；v1 channel 拒绝。

- [ ] **Step 2: 写 anchor 边沿、104-D 与 ONNX/PD 连续调用失败测试**

```python
def test_real_waiting_anchor_is_captured_once_and_corrects_drift(env):
    env.simulator.base_pose_valid = True
    env.simulator.root_trans_world = np.array([1.2, -0.3, 0.78])
    assert env._capture_hitter_waiting_base_anchor(require_initial=True)
    env.simulator.root_trans_world = np.array([1.2, -0.1, 0.78])
    pos, quat = env._hitter_robot_anchor_pose_w()
    target_b = env._hitter_waiting_base_target_pos_b(pos, quat)
    assert np.allclose(env.waiting_base_anchor_xy_w, [1.2, -0.3])
    assert target_b[1] < 0.0


def test_waiting_keeps_policy_and_pd_running(agent_harness):
    agent_harness.run_ticks(10, phase=CommandPhase.WAITING)
    assert agent_harness.onnx_calls == 10
    assert agent_harness.apply_action_calls == 10
    assert agent_harness.last_observation.shape == (1, 104)
    assert np.isfinite(agent_harness.last_observation).all()
```

在 `test_hitter_waiting_anchor.py` 增加 `pytest.mark.parametrize("edge", ["cancel","recovery_complete"])` 验证每个 WAITING edge 只重新捕获一次；另设四个独立测试断言：(1) invalid+old anchor 保留旧值并 block；(2) pose 在同一 policy session 恢复后新 ID 的 planner submit 计数仍为 0且不进入 ARMED；(3) 显式 reentry 成功后才清 fault；(4) invalid+no anchor 抛 RuntimeError。MuJoCo case 断言仍使用 YAML configured waiting target。

- [ ] **Step 3: 写 ordered batch 和 next-serve 集成失败测试**

同一 tick 注入 `success -> failure -> failure -> failure`，断言四条都被消费且第三个 failure 撤拍；注入 overflow 断言 pre-commit 立即 cancel、整批结果均不 ingest、batch 内 ID 和被溢出 ID 全部 consume；0.49 s 前出现的新 ID被 consume，0.50 s 后不能复活，只有之后出现的未见 ID可提交 planner。

两个 barrier 测试各使用 `threading.Event(phase_checked,allow_submit,policy_started)` 和 1 秒 timeout：(1) listener 在锁外准备 ID 8 后阻塞，policy 先进入 RECOVERY，释放 listener 后断言 ID 8 consumed/submit 0；(2) fake worker.submit 设置 `phase_checked` 后等 `allow_submit`，policy thread 在此期间不能取得 lifecycle lock或改变 phase，释放后 submit 完成才允许 RECOVERY。另一个测试在同一 tick 预置 `TRACK_ENDED` event 和成功 result，断言 event 先撤拍、后到 result 因 ID 已 consumed 被忽略，证明直接安全事件在一个 20 ms tick 内生效。

再预先创建一条 `consumed=False,new_track=False` 的旧 active snapshot，然后先完成 strike/进入 RECOVERY、最后才调用 listener；锁内 lifecycle consumed/RECOVERY gate 必须令 submit 计数保持不变，证明 snapshot 创建时的旧标志不能绕过消费状态。

队列完整性用两个独立测试：(1) lifecycle 尚在 WAITING，`CompletedResultBatch(results=(success7,),overflow_count=1,overflowed_track_ids=(8,))`，tick 后保持 WAITING且 worker result ingest spy 为 0；(2)随后分别排队 ID 7/8 success，均返回 ignored consumed、armed count 0。

消费状态同步以 scenario 参数化 `late_skip,third_soft,immediate,third_discontinuity`；每个 scenario 走真实 tick 得到含 ID 7 的 `decision.consumed_track_ids`，随后断言 RealWorld consumed set 含 7。注入 frame+1 后 listener submit spy 不增加，最后 diagnostics snapshot 的 `consumed is True`。

- [ ] **Step 4: 运行配置、anchor 和 integration tests，确认旧 fixed target/latest-result 路径失败**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  deploy/tests/test_hitter_runtime_factory.py \
  deploy/tests/test_hitter_waiting_anchor.py \
  deploy/tests/test_hitter_runtime_single_shot_integration.py \
  deploy/tests/test_hitter_policy_first_frame_transition.py \
  deploy/tests/test_hitter_task_observation.py
```

Expected: FAIL，至少显示 planner 默认仍 100 Hz、waiting target 仍 `[-0.4,0]` 或 controller 只取 latest result。

- [ ] **Step 5: 扩展并严格校验 runtime settings**

`HitterRuntimeSettings` 增加 Global Constraints 中全部参数；单独创建 `ViconConsumerSettings(channel, stream_stale_timeout_s, ball_message_stale_timeout_s)`。浮点要求 finite 且按规范正/非负；整数必须 `type(value) is int and value > 0`。

- [ ] **Step 6: 更新 YAML 默认值并明确 sim 边界**

```yaml
motion:
  vicon_consumer:
    channel: vicon_state_data_v2
    base_subject: G2Pelvis
    stream_timeout_s: 0.40
    ball_timeout_s: 0.40
    new_serve_no_ball_s: 0.50
    event_queue_capacity: 64
  waiting_base_target_xy_w: [-0.4, 0.0]  # MuJoCo only
  ball_planner:
    planner_update_rate_hz: 50.0
    completed_result_queue_capacity: 64
    armed_cancel_consecutive_failures: 3
    commit_time_to_strike_s: 0.30
    maximum_racket_target_override_delta_m: 0.05
    maximum_racket_velocity_override_delta_mps: 0.75
    maximum_strike_deadline_override_delta_s: 0.05
```

- [ ] **Step 7: 按固定顺序整合每个 policy tick**

listener 先做新轨迹的 lifecycle phase gate；未通过的 ID 当场消费，不能等 WAITING 后复活：

```python
def _submit_hitter_planner_snapshot(self, snapshot: BallEstimateSnapshot) -> None:
    if snapshot.consumed:
        return
    with self._hitter_lifecycle_lock:
        if self._waiting_anchor_fault is not None:
            self.simulator.consume_hitter_track(
                snapshot.track_id,
                reason=f"waiting_anchor_fault:{self._waiting_anchor_fault.value}",
            )
            self.hitter_command_lifecycle.consume_track(
                snapshot.track_id,
                reason="waiting_anchor_fault",
            )
            return
        phase = self.hitter_command_lifecycle.phase
        if snapshot.track_id in self.hitter_command_lifecycle.consumed_track_ids:
            self.simulator.consume_hitter_track(
                snapshot.track_id, reason="lifecycle_already_consumed"
            )
            return
        if phase is CommandPhase.RECOVERY:
            self.simulator.consume_hitter_track(
                snapshot.track_id, reason="snapshot_during_recovery"
            )
            self.hitter_command_lifecycle.consume_track(
                snapshot.track_id, reason="snapshot_during_recovery"
            )
            return
        if snapshot.new_track and phase is not CommandPhase.WAITING:
            self.simulator.consume_hitter_track(
                snapshot.track_id,
                reason=f"new_track_during_{phase.value}",
            )
            self.hitter_command_lifecycle.consume_track(
                snapshot.track_id,
                reason="serve_gate_closed",
            )
            return
        if not self._planner_submit_interval_elapsed(snapshot.received_monotonic_s):
            return
        # submit() is non-blocking; keeping it inside this lock makes the
        # phase check and publication to the worker one atomic admission step.
        self.hitter_planner_worker.submit(snapshot)

def _planner_submit_interval_elapsed(self, received_monotonic_s: float) -> bool:
    received = float(received_monotonic_s)
    previous = self._hitter_last_planner_submit_monotonic_s
    if previous is not None and (
        received - previous
        < self.hitter_runtime_settings.planner_update_interval_s - 1.0e-12
    ):
        return False
    self._hitter_last_planner_submit_monotonic_s = received
    return True
```

`_init_hitter_lifecycle_state()` 创建 `threading.RLock()`；listener 的 anchor-fault/consumed/phase check、rate gate、非阻塞 submit/consume 与 policy tick 的 `advance/cancel/ingest` 都在该锁下执行，消除 phase-check 到 submit 的 TOCTOU。即使 snapshot 是 consume 前创建的旧对象，也会在锁内通过 lifecycle process-lifetime consumed set 被挡住；RECOVERY 对 `new_track=False` 的旧 active snapshot 同样 fail closed。`_waiting_anchor_fault` 一旦在同一 policy session 锁存，即使 pelvis 后来恢复为 valid，新 ID 也要被双方消费且不能提交 planner；只有显式 policy reentry 成功重捕 anchor 后清除。首次 policy reset 完成后调用 `begin_hitter_policy_session(now_monotonic_s=now)`；若已有球可见，该 ID 已被 quarantine，只有它结束并连续无球 0.50 s 后出现的未见 ID才可能通过。

```python
status = self.simulator.hitter_vicon_status(now_monotonic_s=now)
for event in self.simulator.drain_hitter_vicon_events():
    reason = LifecycleCancelReason(event.reason.value)
    self._apply_hitter_lifecycle_decision(
        self.hitter_command_lifecycle.cancel(
            reason=reason, now=now, track_id=event.track_id
        ),
        now=now,
    )
advance_decision = self.hitter_command_lifecycle.advance(now=now)
self._apply_hitter_lifecycle_decision(advance_decision, now=now)

batch = self.hitter_planner_worker.drain_completed_results()
if batch.overflowed:
    compromised_ids = {
        result.track_id for result in batch.results
    } | set(batch.overflowed_track_ids)
    for track_id in sorted(compromised_ids):
        self.simulator.consume_hitter_track(
            track_id, reason="completed_result_queue_overflow"
        )
        self.hitter_command_lifecycle.consume_track(
            track_id, reason="completed_result_queue_overflow"
        )
    self._apply_hitter_lifecycle_decision(
        self.hitter_command_lifecycle.cancel(
            reason=LifecycleCancelReason.RESULT_QUEUE_OVERFLOW, now=now
        ),
        now=now,
    )
else:
    for result in batch.results:
        decision = self.hitter_command_lifecycle.ingest(result, now=now)
        self._apply_hitter_lifecycle_decision(decision, now=now)
```

`_apply_hitter_lifecycle_decision()` 必须遍历 `decision.consumed_track_ids`，对每个 ID 在同一 `_hitter_lifecycle_lock` 内调用 `simulator.consume_hitter_track(track_id, reason=decision.kind)`；late skip、cancel、third-soft、strike、recovery consume 和 track end 不再靠 `kind` 分支猜测。overflow 在构造 decision 前已逐 ID同步 batch/丢失 ID。这样 RealWorld 不再向 listener 发送该 ID 的 planning-eligible snapshot。这个同步必须幂等，不能重置 generation 或 no-ball timer。

overflow 表示整批顺序证据已不完整，因此禁止 ingest 该 batch 中任何 success/failure；即使当时没有 active command，也要保持 WAITING。若已有 active ID 且不在 `compromised_ids`，`cancel()` 仍会原子消费它。在 ingest 前 drain 的 schema/stale/ball-stale/conflict/base/track-ended transition 进入 pre-commit immediate cancel；同一 event 只出现一次。strike 后只 reset estimator samples；ID 消费由统一 decision-application 路径同步。不缓存 recovery 新球。

- [ ] **Step 8: 实现 WAITING entry edge capture**

```python
def _capture_hitter_waiting_base_anchor(
    self, *, require_initial: bool, policy_reentry: bool = False
) -> bool:
    if not self.simulator.is_real:
        return True
    if self._waiting_anchor_fault is not None and not policy_reentry:
        return False
    if not bool(self.simulator.base_pose_valid):
        if self.waiting_base_anchor_xy_w is None and require_initial:
            raise RuntimeError(
                "Cannot enter HITTER policy without a valid G2Pelvis waiting anchor"
            )
        self._waiting_anchor_fault = LifecycleCancelReason.BASE_POSE_INVALID
        return False
    position = np.asarray(self.simulator.root_trans_world, dtype=np.float64)[:2]
    if position.shape != (2,) or not np.isfinite(position).all():
        raise RuntimeError("G2Pelvis waiting anchor is not a finite xy vector")
    self.waiting_base_anchor_xy_w = position.astype(np.float32, copy=True)
    if policy_reentry:
        self._waiting_anchor_fault = None
    return True
```

reset/policy reentry 用 `policy_reentry=True`；未 arm end、late skip、cancel、recovery complete 的 `entered_waiting` edge 用默认 false。invalid edge 后即使 pose 数据恢复也保持 fault 和旧 anchor，直到下一次显式 policy reentry。WAITING observation 每帧计算 `anchor-current pelvis`，racket 为当前 FK、velocity 0、TTS 0.92；不增加 action gating。

- [ ] **Step 9: 把 planner summary 限到 1 Hz 并移除控制循环负载噪声**

状态转换/撤拍/协议错误即时一次；ball/planner/queue 汇总最多每秒一次。保留 agent 每 tick ONNX 与 `env.step()->apply_action()` 调用。

- [ ] **Step 10: 运行核心 runtime 回归**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  deploy/tests/test_hitter_runtime_factory.py \
  deploy/tests/test_hitter_waiting_anchor.py \
  deploy/tests/test_hitter_runtime_single_shot_integration.py \
  deploy/tests/test_hitter_policy_first_frame_transition.py \
  deploy/tests/test_hitter_task_observation.py
```

Expected: PASS；WAITING observation 为 `(1,104)` 且 10 tick 对应 10 次 ONNX/PD；MuJoCo tests 不变。

- [ ] **Step 11: 提交 runtime integration 单元**

```bash
git add deploy/config/sim/real_world.yaml \
  deploy/simulator/real_world.py \
  deploy/tests/test_hitter_waiting_anchor.py \
  deploy/tests/test_hitter_runtime_single_shot_integration.py \
  deploy/tests/hitter_runtime_test_harness.py \
  deploy/tests/test_hitter_policy_first_frame_transition.py \
  deploy/tests/test_hitter_task_observation.py \
  deploy/tests/test_real_world_v2_consumer.py \
  deploy/tests/test_hitter_strike_target_logging.py
git add -p -- deploy/utils/hitter_runtime_factory.py
git add -p -- deploy/config/mimic/hitter.yaml deploy/config/hitter.yaml
git add -p -- deploy/envs/hitter.py deploy/agents/hitter_agent.py
git add -p -- deploy/tests/test_hitter_runtime_factory.py
git diff --cached --check
git diff --cached --name-status
git commit -m "feat: integrate safe single-shot HITTER runtime"
```

---

### Task 10: 将 diagnostics canonical model 与 shadow pipeline 切到生产 v2 语义

**Files:**
- Modify: `deploy/diagnostics/hitter_task_models.py`
- Modify: `deploy/diagnostics/hitter_task_events.py`
- Modify: `deploy/diagnostics/hitter_task_attempts.py`
- Modify: `deploy/diagnostics/hitter_task_pipeline.py`
- Modify: `deploy/tests/test_hitter_task_attempts.py`
- Modify: `deploy/tests/test_hitter_task_events.py`
- Modify: `deploy/tests/test_hitter_task_input_adapter.py`
- Modify: `deploy/tests/test_hitter_task_pipeline.py`

**Interfaces:**
- Consumes: Task 4 `SnapshotKey(track_id,generation)`、Task 6 batch、Task 8 lifecycle 权威状态。
- Produces: recording schema 2、live schema 3；`NormalizedMocapSample(track_id, identity_source)`；shadow pipeline 全量 drain 并直接投影 typed reason。

实现前合同裁决：现有生产类型名 `ShadowTaskPipeline` 保持不变，下方 `HitterTaskPipeline` 均视为笔误；不新建同义 pipeline。保留现有 `tick(lifecycle_now_s, obs_now_s, wall_time_us)` 三时钟接口，给 `TaskTickResult` 增加 typed lifecycle snapshot、完整 `completed_results` 与 batch metadata，禁止把 lifecycle/observation monotonic 时钟和 wall timestamp 合并。deterministic worker 的 `frozen_results` 与 results 等长，测试可填 `None`；canonical lifecycle 只消费 `PlannerResultSnapshot`，frozen mirror 只做诊断一致性检查。`completed_result_queue_depth` 定义为本次 drain 返回的 `len(batch.results)`，capacity 来自 Task 9 settings，health overflow 使用累计 total，单次 overflow event 使用 batch 的本次 count；`planner_submit_rate_hz` 表示配置的提交上限，不伪装为实测窗口速率。online adapter 只能生成 `identity_source="wire_v2"`；`legacy_inferred` 仅允许 canonical model/Task 13 replay loader 读写，Task 10 runtime 不得生成或 fallback。

测试中的 `pipeline` fixture 使用生产 `ShadowTaskPipeline` 和一个只实现 `submit/drain_completed_results/stats` 的 deterministic worker；`inject_completed()` 按给定顺序 append `PlannerResultSnapshot`，并生成等长的 `None` frozen entries。`success()` / `failure()` 复用 Task 8 的 test factory（在 Task 8 创建 `deploy/tests/hitter_test_factories.py` 并随该 task 提交），不得建立另一套简化 lifecycle 类型。

- [ ] **Step 1: 写 schema 2/live 3 model round-trip 失败测试**

```python
def test_v2_models_preserve_identity_and_safety_state():
    sample = NormalizedMocapSample(
        input_seq=1, channel="vicon_state_data_v2", subject="ball",
        track_id=7, identity_source="wire_v2",
        position_w=np.array([0.8, -0.1, 1.0]),
        quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
        valid=True, occluded=False, source_frame=10,
        source_time_s=1.0, publish_time_us=1_000_000,
        received_monotonic_s=1.0, wall_time_us=1_000_000,
        payload_size=128,
    )
    assert sample.to_json_dict()["track_id"] == 7
    assert sample.to_json_dict()["identity_source"] == "wire_v2"
    assert SCHEMA_VERSION == 2
    assert LIVE_SNAPSHOT_SCHEMA_VERSION == 3
```

`LifecycleSnapshot` round-trip 还必须断言 `failure_reason, consumed, cancel_reason, locked_strike_type, locked_base_target_xy, locked_strike_deadline_monotonic_s, in_commit_window, policy_tts_s, completed_result_queue_depth`。

- [ ] **Step 2: 写 shadow pipeline ordered-result 失败测试**

```python
def test_one_tick_projects_every_completed_result_in_order(pipeline):
    pipeline.worker.inject_completed([
        success(track_id=7, generation=1),
        failure(track_id=7, generation=2,
                reason=PlannerFailureReason.BALL_NOT_INCOMING),
        failure(track_id=7, generation=3,
                reason=PlannerFailureReason.BALL_NOT_INCOMING),
        failure(track_id=7, generation=4,
                reason=PlannerFailureReason.BALL_NOT_INCOMING),
    ])
    tick = pipeline.tick(now=1.0)
    assert [item.source_generation for item in tick.completed_results] == [1, 2, 3, 4]
    assert tick.lifecycle.cancel_reason == "BALL_NOT_INCOMING"
    assert tick.lifecycle.consumed is True
```

增加 `test_reason_code_ignores_error_text()`：相同 typed reason 分别配空文本与 `"completely unrelated"`，两次 lifecycle decision 相同；增加 `test_pending_drop_and_completed_overflow_are_separate_health_counters()`，分别只触发一种情况并断言另一个计数仍为零。

- [ ] **Step 3: 运行 model/events/pipeline tests，确认旧 schema/单结果逻辑失败**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4/deploy
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  tests/test_hitter_task_attempts.py tests/test_hitter_task_events.py \
  tests/test_hitter_task_input_adapter.py tests/test_hitter_task_pipeline.py
```

Expected: FAIL，显示 schema 仍为 1/2、sample 缺 ID 或 shadow 只看 latest result。

- [ ] **Step 4: 升级 canonical model，不在 diagnostics 重新推导控制语义**

`SCHEMA_VERSION=2`、`LIVE_SNAPSHOT_SCHEMA_VERSION=3`。`NormalizedMocapSample` 增加正/零 ID 合同和 `Literal["wire_v2","legacy_inferred"]`。`BallDiagnosticState` 增加 `track_id, identity_source, consumed`。`HealthSnapshot` 用 `completed_result_queue_depth/capacity/overflow_count` 和 `planner_submit_rate_hz` 替代旧 completed overwrite 指标。

- [ ] **Step 5: 扩展 lifecycle/attempt 投影字段**

`AttemptSummary` 和 `AttemptDetail` 持久化：`track_id, identity_source, consumed, cancel_reason, failure_reason, locked_strike_type, locked_base_target_xy, locked_strike_deadline_monotonic_s, strike_table_y_w, strike_side_source, strike_count`。`AttemptTracker` 以 wire `track_id` 绑定 attempt；`track_segment_id` 只作展示分段，estimator reset 不换 attempt 身份。

- [ ] **Step 6: 让 adapter 强制 v2 subject-ID 合同**

`MocapFrameAdapter` 的在线 subject 集合固定为 `ball, g2pelvis, table`；对 ball 要求正 ID，对 pelvis/table 要求 0；invalid ball 保留原 ID。不分配本地 ID，不因 bounce/reset/strike 改 ID。在线输入的 `identity_source` 恒为 `wire_v2`。

- [ ] **Step 7: 迁移 shadow tick 到 batch drain**

```python
batch = self.worker.drain_completed_results()
decisions = []
if batch.overflowed:
    compromised_ids = {
        result.track_id for result in batch.results
    } | set(batch.overflowed_track_ids)
    for track_id in sorted(compromised_ids):
        self.lifecycle.consume_track(
            track_id, reason="completed_result_queue_overflow"
        )
    decisions.append(self.lifecycle.cancel(
        reason=LifecycleCancelReason.RESULT_QUEUE_OVERFLOW,
        now=now,
    ))
else:
    for result, frozen in zip(batch.results, batch.frozen_results):
        decision = self._consume_completed_result(result, frozen, now=now)
        decisions.append(decision)
```

非 overflow batch 的每个 completed result 都产生有序 event；overflow batch 产生一条含 `overflow_count,overflowed_track_ids,compromised_track_ids` 的完整性故障 event，不再投影其中单条结果。`planner_reason_code()` 删除字符串解析，直接使用 `result.failure_reason.value`。tick detail 保存 tuple，不保存单个 bundle。

- [ ] **Step 8: 运行 diagnostics core tests**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4/deploy
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  tests/test_hitter_task_attempts.py tests/test_hitter_task_events.py \
  tests/test_hitter_task_input_adapter.py tests/test_hitter_task_pipeline.py
```

Expected: PASS；关键 event 均含 `track_id,generation,source_frame,phase,decision,failure_reason,consumed,locked_strike_type,policy_tts_s,completed_result_queue_depth`。

- [ ] **Step 9: 提交 diagnostics core 单元**

```bash
git add deploy/diagnostics/hitter_task_models.py \
  deploy/diagnostics/hitter_task_events.py \
  deploy/diagnostics/hitter_task_attempts.py \
  deploy/diagnostics/hitter_task_pipeline.py \
  deploy/tests/test_hitter_task_attempts.py \
  deploy/tests/test_hitter_task_events.py \
  deploy/tests/test_hitter_task_pipeline.py
git add -p -- deploy/tests/test_hitter_task_input_adapter.py
git diff --cached --check
git diff --cached --name-status
git commit -m "feat: project HITTER v2 identity diagnostics"
```

---

### Task 11: 升级在线 monitor、recording 与 Web 状态展示

**Files:**
- Modify: `deploy/diagnostics/hitter_task_monitor.py`
- Modify: `deploy/diagnostics/hitter_task_recording.py`
- Modify: `deploy/diagnostics/static/hitter_task_monitor.html`
- Modify: `deploy/tests/test_hitter_task_monitor.py`
- Modify: `deploy/tests/test_hitter_task_recording.py`
- Modify: `deploy/tests/test_hitter_task_frontend.py`
- Modify: `deploy/tests/test_hitter_task_diagnostics_safety.py`

**Interfaces:**
- Consumes: Task 10 schema 2/live 3 models。
- Produces: v2-only online monitor、坏 fingerprint 后线程存活、带完整 ID/撤拍/手型/队列字段的 session/CSV/UI。

测试 helper 合同：`encode_valid_ball(track_id)` 用 Task 1 生成类型编码；`wait_until()` 是 1 秒有界轮询；`record_one_cancelled_attempt()` 通过 recorder 的公开 sample/event API 写入一个完整 attempt 并正常 close；`session_paths` 由 `tmp_path` 创建，测试不读取仓库中的历史录制。monitor fixture 必须在 `finally` 中 `close()` 并断言所有后台线程退出。

- [ ] **Step 1: 写 v2-only monitor 与 decode isolation 失败测试**

```python
def test_monitor_rejects_v1_channel_before_start():
    with pytest.raises(ValueError, match="vicon_state_data_v2"):
        MonitorOptions(channel="vicon_state_data")


def test_bad_fingerprint_does_not_stop_monitor_threads(monitor):
    monitor._handle_lcm("vicon_state_data_v2", b"bad fingerprint")
    monitor._handle_lcm("vicon_state_data_v2", encode_valid_ball(track_id=7))
    wait_until(lambda: monitor.pipeline.adapter.latest_ball_snapshot is not None)
    assert monitor.background_threads_alive()
    assert "VICON_SCHEMA_ERROR" in monitor.warnings
```

- [ ] **Step 2: 写录制 CSV 和 live v3 UI 失败测试**

```python
def test_recording_persists_v2_identity_and_cancel_fields(session_paths):
    record_one_cancelled_attempt(session_paths, track_id=7)
    with session_paths.ball_samples_csv.open() as stream:
        raw = list(csv.DictReader(stream))
    with session_paths.attempts_csv.open() as stream:
        attempts = list(csv.DictReader(stream))
    assert raw[0]["track_id"] == "7"
    assert raw[0]["identity_source"] == "wire_v2"
    assert attempts[0]["consumed"] == "True"
    assert attempts[0]["cancel_reason"] == "TRACK_ENDED"
    assert attempts[0]["strike_side_source"] == "table_y"
```

frontend fixture 固定包含 `track_id=7,consumed=true,cancel_reason=TRACK_ENDED,failure_reason=BALL_NOT_INCOMING,locked_strike_type=forehand,strike_table_y_w=-0.1,in_commit_window=false,queue_depth=3,overflow_count=1`，逐个断言对应 DOM text；另把 `schema_version=2` 送入 reducer，断言产生 `incompatible live schema` 状态且不更新 v3 panel。

- [ ] **Step 3: 运行 monitor/recording/frontend tests，确认旧默认和字段失败**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4/deploy
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  tests/test_hitter_task_monitor.py tests/test_hitter_task_recording.py \
  tests/test_hitter_task_frontend.py \
  tests/test_hitter_task_diagnostics_safety.py
```

Expected: FAIL，默认 channel 仍 v1、live schema 仍 2 或 CSV 缺安全字段。

- [ ] **Step 4: 强制 online monitor v2 并隔离 decode exception**

`MonitorOptions.channel` 默认且只允许 `vicon_state_data_v2`。`_handle_lcm()` 捕获 decode/fingerprint 异常，增加 schema-error warning/event 后 return；LCM/input/tick/recorder 线程继续。ball 正常包不 stdout；汇总最多 1 Hz。

- [ ] **Step 5: 扩展 session metadata 与 CSV 字段**

`session.json` 写 `wire_schema="transformation_t_v2"`、`channel="vicon_state_data_v2"`、`identity_source="wire_v2"`。`_RAW_FIELDS` 加 `track_id,identity_source`；`_ATTEMPT_FIELDS` 加 Task 10 的 consumed/cancel/failure/locked/side/strike-count 字段。新录制对 v1 payload 不写 raw row。

- [ ] **Step 6: 更新 live v3 JSON reducer 与 Web 展示**

frontend 不自行重算手型；展示 production 提供的 `strike_table_y_w` 与固定 `strike_side_source=table_y`，并显示 `expected side` 只作为一致性标志。health 分别显示 pending drop 和 completed queue overflow。

- [ ] **Step 7: 运行 online diagnostics tests**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4/deploy
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  tests/test_hitter_task_monitor.py tests/test_hitter_task_recording.py \
  tests/test_hitter_task_frontend.py \
  tests/test_hitter_task_diagnostics_safety.py
```

Expected: PASS；bad fingerprint 后合法消息仍进入 pipeline；CSV 与 UI 包含全部字段。

- [ ] **Step 8: 提交 monitor/recording/UI 单元**

```bash
git add deploy/diagnostics/hitter_task_recording.py \
  deploy/diagnostics/static/hitter_task_monitor.html \
  deploy/tests/test_hitter_task_recording.py \
  deploy/tests/test_hitter_task_frontend.py
git add -p -- deploy/diagnostics/hitter_task_monitor.py
git add -p -- deploy/tests/test_hitter_task_monitor.py
git add -p -- deploy/tests/test_hitter_task_diagnostics_safety.py
git diff --cached --check
git diff --cached --name-status
git commit -m "feat: record and display HITTER v2 safety state"
```

---

### Task 12: 恢复并迁移原子 CSV exporter 到 v2 复合身份

**Files:**
- Create: `deploy/diagnostics/export_hitter_task_csv.py`
- Create: `deploy/tests/test_hitter_task_csv_export.py`

**Interfaces:**
- Consumes: Task 11 schema 2 recording；本地只读参考分支 `local/complete-planner-csv-export-20260731` 的 exporter/tests。
- Produces: 八张 CSV 与 `export_manifest.json`；主键 `(attempt_id, identity_source, track_id, generation)`；失败时保留旧输出目录。

测试中的 `recorded_session` 由 `tmp_path` 写出 schema-2 `session.json`、完成标记和最小 raw/planner/stage/policy/event 文件，并保存每个输入文件 SHA-256；`read_csv(path)` 直接返回 `list(csv.DictReader(...))`；`output` 是尚不存在的同一临时父目录子路径。legacy 输入必须先经 Task 13 replay loader 规范化成显式 bundle，exporter 本身不得 import legacy adapter 或猜测 ID。

- [ ] **Step 1: 只读检查参考实现并写 v2 contract 失败测试**

先用以下命令查看参考，不直接 checkout 或覆盖工作树：

```bash
git show local/complete-planner-csv-export-20260731:deploy/diagnostics/export_hitter_task_csv.py
git show local/complete-planner-csv-export-20260731:deploy/tests/test_hitter_task_csv_export.py
```

新测试核心断言：

```python
def test_export_uses_v2_composite_key_and_safety_columns(recorded_session, output):
    export_session_csv(recorded_session, output)
    rows = read_csv(output / "planner_results.csv")
    assert set(("attempt_id", "identity_source", "track_id", "generation")) \
        <= rows[0].keys()
    assert rows[0]["strike_side_source"] == "table_y"
    manifest = json.loads((output / "export_manifest.json").read_text())
    assert manifest["recording_schema_version"] == 2
    assert manifest["wire_channel"] == "vicon_state_data_v2"
    assert manifest["track_id_strike_count_le_one"] is True
    assert manifest["strike_side_rule_consistent"] is True
```

- [ ] **Step 2: 运行 exporter 测试，确认当前文件不存在**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4/deploy
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  tests/test_hitter_task_csv_export.py
```

Expected: FAIL，`ModuleNotFoundError: diagnostics.export_hitter_task_csv`。

- [ ] **Step 3: 用 apply_patch 创建 exporter，保留已验证的原子/完整性边界**

公开类型/入口固定为：`PlannerExportKey(attempt_id,identity_source,track_id,generation)`、`load_attempt_inputs()`、`scan_planner_events()`、`validate_written_outputs()`、`replace_output_directory()`、`export_session_csv()` 和 `main()`。复用参考分支的 CSV quoting、fsync、hash 和 CLI 退出码实现，但把 join key 全部升级为 `PlannerExportKey`。顶层事务必须按以下代码组织，不允许直接在旧 output 内增量写：

```python
@dataclass(frozen=True, order=True)
class PlannerExportKey:
    attempt_id: int
    identity_source: str
    track_id: int
    generation: int


def export_session_csv(
    session_root: Path, output_root: Path
) -> Mapping[str, object]:
    session_root = Path(session_root).resolve(strict=True)
    output_argument = Path(output_root)
    output_parent = output_argument.parent.resolve(strict=True)
    output_root = output_parent / output_argument.name
    if output_argument.is_symlink() or output_root.is_symlink():
        raise ValueError("output_root must not be a symbolic link")
    inputs = load_attempt_inputs(session_root)
    event_index = scan_planner_events(session_root)
    staging_root = Path(tempfile.mkdtemp(
        prefix=f".{output_root.name}.staging-", dir=output_root.parent
    ))
    try:
        tables = build_export_tables(inputs, event_index)
        for filename in EXPORT_FILENAMES:
            write_csv_fsync(staging_root / filename, tables[filename])
        manifest = build_manifest(session_root, tables)
        write_json_fsync(staging_root / "export_manifest.json", manifest)
        validate_written_outputs(staging_root, manifest)
        replace_output_directory(staging_root, output_root)
        return manifest
    except BaseException:
        if staging_root.exists():
            remove_owned_export_tree(
                staging_root,
                expected_parent=output_root.parent,
                required_prefix=f".{output_root.name}.staging-",
            )
        raise


def replace_output_directory(staging_root: Path, output_root: Path) -> None:
    backup_root = output_root.with_name(f".{output_root.name}.previous")
    validate_export_swap_paths(staging_root, output_root, backup_root)
    if backup_root.exists():
        if output_root.exists():
            raise FileExistsError(backup_root)
        os.replace(backup_root, output_root)  # recover interrupted prior swap
    had_output = output_root.exists()
    if had_output:
        os.replace(output_root, backup_root)
    try:
        os.replace(staging_root, output_root)
    except BaseException:
        if had_output:
            os.replace(backup_root, output_root)
        raise
    if had_output:
        remove_owned_export_tree(
            backup_root,
            expected_parent=output_root.parent,
            required_name=f".{output_root.name}.previous",
        )
```

`validate_export_swap_paths()` 要求三条路径同一已解析父目录，output/backup 不是 symlink，staging 是 `tempfile.mkdtemp()` 新建的真实目录且名字具有精确 prefix；拒绝 `/`、home、repo root 或父目录本身。`remove_owned_export_tree()` 在 `shutil.rmtree()` 前重复验证 resolved parent、精确 name/prefix、目录类型、非 symlink 和单一 target；任一不符立即抛错，不升级删除手段。`load_attempt_inputs()` 必须先验证 completion marker、schema 版本和 session manifest 中每个输入 hash；`scan_planner_events()` 对每个 `PlannerExportKey` 只允许一个 input/call/result，并拒绝 source-frame 冲突；`build_export_tables()` 固定产出下面八张 CSV。`validate_written_outputs()` 重读 CSV，验证 manifest row count/file hash、所有非 summary 行的复合键、跨表 referential integrity 和每 ID strike count。具体输出固定为 `summary.csv, raw_lcm.csv, planner_inputs.csv, planner_calls.csv, planner_results.csv, stage_timeline.csv, policy_ticks.csv, task_observations.csv, export_manifest.json`。任一缺 key、重复 key、混合 identity source、source-frame 冲突或 incomplete recording 抛异常且不动旧输出。

- [ ] **Step 4: 写所有 v2 安全列和 manifest gate**

每张适用 CSV 加：`track_id,identity_source,consumed,cancel_reason,failure_reason,locked_strike_type,locked_base_target_xy,locked_strike_deadline_monotonic_s,in_commit_window,policy_tts_s,completed_result_queue_depth,strike_table_y_w,strike_side_source,expected_strike_type_from_table_y,strike_type_consistent,strike_count`。manifest 加 `recording_schema_version,wire_channel,identity_source,v2_wire_compatible,track_id_strike_count_le_one,strike_side_rule_consistent,completed_queue_overflow_absent`。

- [ ] **Step 5: 运行 exporter 测试，包括失败原子性**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4/deploy
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  tests/test_hitter_task_csv_export.py
```

Expected: PASS；人为损坏输入时 CLI 非零且旧 output checksum 不变。

- [ ] **Step 6: 提交 exporter 单元**

```bash
git add deploy/diagnostics/export_hitter_task_csv.py \
  deploy/tests/test_hitter_task_csv_export.py
git diff --cached --check
git diff --cached --name-status
git commit -m "feat: export HITTER v2 safety evidence"
```

---

### Task 13: 增加 replay-only 历史 ID 适配并对齐生产队列/lifecycle

**Files:**
- Create: `deploy/diagnostics/hitter_task_legacy_identity.py`
- Modify: `deploy/diagnostics/hitter_task_replay.py`
- Modify: `deploy/tests/test_hitter_task_replay.py`
- Modify: `deploy/tests/test_hitter_task_replay_process.py`
- Modify: `deploy/tests/test_hitter_task_diagnostics_safety.py`

**Interfaces:**
- Consumes: v2 replay schema、Task 6 queue、Task 8 lifecycle。
- Produces: `LegacyTrackIdAdapter(session_sha256).track_id_for_epoch()`；legacy 输出恒 `identity_source="legacy_inferred"` 且 `v2_wire_compatible=False`。

- [ ] **Step 1: 写确定性、隔离和碰撞失败测试**

```python
def test_legacy_id_is_deterministic_positive_int64():
    a = LegacyTrackIdAdapter("ab" * 32)
    b = LegacyTrackIdAdapter("ab" * 32)
    assert a.track_id_for_epoch(4) == b.track_id_for_epoch(4)
    assert 0 < a.track_id_for_epoch(4) <= (1 << 63) - 1
    assert a.track_id_for_epoch(4) != a.track_id_for_epoch(5)


def test_active_runtime_never_imports_legacy_adapter():
    script = "import simulator.real_world, utils.hitter_realtime, envs.hitter; " \
             "import sys; print('diagnostics.hitter_task_legacy_identity' in sys.modules)"
    output = subprocess.check_output([sys.executable, "-c", script], text=True)
    assert output.strip() == "False"
```

通过 monkeypatch digest 构造碰撞时，adapter 必须 raise，不能重新取随机 ID。

- [ ] **Step 2: 写历史事故与 v2 下一发 replay 失败测试**

`test_hitter_task_replay.py` 的历史 bundle 固定构造 generation 1 success、2/3/4 `BALL_NOT_INCOMING`、5 同 epoch returning success，断言 generation 4 pre-commit cancel、5 ignored-consumed且无第二次 arm。v2 bundle 固定构造 ID 7 return、invalid 7、0.50 s no-ball、ID 8 generation 1，断言 `strike_count(7)<=1`、ID 7 return submit 为 0、ID 8 才开始下一拍。

- [ ] **Step 3: 运行 replay tests，确认旧 epoch 与 latest-result facade 失败**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4/deploy
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  tests/test_hitter_task_replay.py tests/test_hitter_task_replay_process.py \
  tests/test_hitter_task_diagnostics_safety.py
```

Expected: FAIL，缺少 adapter 或 replay worker 仍只提供 `latest_result_bundle()`。

- [ ] **Step 4: 实现 domain-separated deterministic adapter**

```python
class LegacyTrackIdAdapter:
    def __init__(self, session_sha256: str):
        if not re.fullmatch(r"[0-9a-fA-F]{64}", session_sha256):
            raise ValueError("session_sha256 must contain exactly 64 hex digits")
        self._session_sha256 = session_sha256.lower()
        self._epoch_to_id: dict[int, int] = {}
        self._id_to_epoch: dict[int, int] = {}

    def track_id_for_epoch(self, track_epoch: int) -> int:
        payload = (
            b"HITTER_LEGACY_TRACK_ID_V1\0"
            + bytes.fromhex(self._session_sha256)
            + int(track_epoch).to_bytes(8, "big", signed=True)
        )
        value = int.from_bytes(hashlib.sha256(payload).digest()[:8], "big") \
            & ((1 << 63) - 1)
        value = value or 1
        previous = self._id_to_epoch.get(value)
        if previous is not None and previous != int(track_epoch):
            raise RuntimeError("legacy track id collision")
        self._epoch_to_id[int(track_epoch)] = value
        self._id_to_epoch[value] = int(track_epoch)
        return value
```

- [ ] **Step 5: 建立严格分流的 v2/legacy loader**

schema 2 只读 `track_id`，不推断。schema 1 必须拿 session.json 原始字节 SHA-256；standalone bundle 要求显式 `legacy_session_sha256`。将 raw sample、stage key、planner call/result key、policy tick key 全部经同一个 adapter 映射，并标记 `legacy_inferred`。禁止使用 Python `hash()`、路径或随机值。

- [ ] **Step 6: 把 replay worker facade 改成 production batch 语义**

facade 保存 completed deque 和 overflow state，暴露 `drain_completed_results()`；deterministic scheduler 在一个 lifecycle tick 前完成的每条 result 都按顺序 drain。Replay bundle/outcome/A-B metrics 写 `identity_source` 和 `v2_wire_compatible`，legacy 永远 false。

- [ ] **Step 7: 运行 replay 与 import-graph tests**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4/deploy
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  tests/test_hitter_task_replay.py tests/test_hitter_task_replay_process.py \
  tests/test_hitter_task_diagnostics_safety.py
```

Expected: PASS；历史事故撤拍、同轨迹不二次 arm；v2 下一 ID 可在 gate 后开始；runtime import graph 不含 legacy module。

- [ ] **Step 8: 提交 replay compatibility 单元**

```bash
git add deploy/diagnostics/hitter_task_legacy_identity.py \
  deploy/diagnostics/hitter_task_replay.py \
  deploy/tests/test_hitter_task_replay.py \
  deploy/tests/test_hitter_task_replay_process.py
git add -p -- deploy/tests/test_hitter_task_diagnostics_safety.py
git diff --cached --check
git diff --cached --name-status
git commit -m "feat: replay legacy HITTER tracks with inferred ids"
```

---

### Task 14: 完成端到端、安全性能、构建与分阶段验收手册

**Files:**
- Modify: `deploy/tests/test_hitter_task_diagnostics_integration.py`
- Modify: `deploy/tests/test_hitter_task_diagnostics_performance.py`
- Modify: `deploy/tests/test_hitter_task_diagnostics_safety.py`
- Create: `deploy/tests/test_hitter_v2_end_to_end.py`
- Modify: `docs/hitter_task_observation_diagnostics.md`
- Create: `docs/hitter_single_shot_track_id_real_world_acceptance.md`
- Local-only modify (do not stage): `.git/info/exclude`

**Interfaces:**
- Consumes: Tasks 1–13 的完整链路。
- Produces: 离线 completion evidence、C++ build evidence、真机只能人工执行的逐级 gate；没有本任务自动启动真机。

- [ ] **Step 1: 写跨 publisher/consumer/planner/lifecycle 的端到端测试**

测试序列固定为：ID 7 来球成功规划；pre-commit 撞网后 `BALL_NOT_INCOMING` 连续三条；同 ID 7 返回；invalid 7；0.50 s 无球；ID 8 新发球。断言：

```python
assert strike_count_by_track_id[7] <= 1
assert armed_track_ids.count(7) == 1
assert cancel_reason_by_track_id[7] == "BALL_NOT_INCOMING"
assert returned_track_7_planner_submissions == 0
assert first_accepted_next_track_id == 8
assert every_side_matches_table_y
assert completed_queue_overflow_count == 0
```

- [ ] **Step 2: 更新 60 秒性能/日志失败测试**

360 Hz v2 input、50 Hz policy/planner 场景要求 `submitted <= floor(elapsed*50)+1`、正常 completed overflow 0、depth ≤64、summary log ≤1 Hz、每 ball stdout 0；保留既有 callback p99、drop、RSS、HTTP 和线程退出门槛。

- [ ] **Step 3: 运行端到端/性能测试，并只修正测试揭示的本计划接口不一致**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4/deploy
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  tests/test_hitter_v2_end_to_end.py \
  tests/test_hitter_task_diagnostics_integration.py \
  tests/test_hitter_task_diagnostics_safety.py
HITTER_DIAGNOSTICS_PERF=1 PYTHONPATH=. \
  /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  tests/test_hitter_task_diagnostics_performance.py
```

Expected: PASS；性能测试报告 planner ≤50 Hz、queue overflow 0、逐包 stdout 0。

- [ ] **Step 4: 编写中文人工验收手册，明确禁止自动越级**

手册按以下固定次序：离线测试 → C++ build → v2 monitor → 单 publisher 进程检查 → 显式 `sim=real_world` 且 R2 前 freshness/G2Pelvis/标定/5 秒过渡检查 → 30 秒 WAITING → 无击球轨迹 → 低风险单拍 → 撞网/失效。记录命令：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
/home/loco1/miniconda3/envs/rb/bin/python \
  deploy/mocap_bridge/monitor_vicon_lcm.py \
  --channel vicon_state_data_v2 --duration 30 --quiet \
  --csv recordings/v2-wire-monitor.csv
pgrep -af 'vicon_table_lcm_bridge|chingmu_table_lcm_bridge'

cd /home/loco1/BOB/Hitter/RobotBridge4/deploy
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python \
  -m diagnostics.hitter_task_monitor \
  --channel vicon_state_data_v2 --port 8766
```

文档明确：这些真机步骤只由用户现场确认并授权后执行；本计划的自动执行到离线/构建验证即停止。

- [ ] **Step 5: 运行 C++、Python 核心回归和 Unitree target build**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
bash deploy/mocap_bridge/build_v2_mocap.sh
deploy/mocap_bridge/.build-v2/test_transformation_t_v2
deploy/mocap_bridge/.build-v2/test_vicon_ball_track_v2
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. \
  /home/loco1/miniconda3/envs/rb/bin/python -m pytest -p no:cacheprovider -q \
  deploy/mocap_bridge/tests/test_transformation_t_v2.py \
  deploy/mocap_bridge/tests/test_chingmu_ball_track_v2.py \
  deploy/mocap_bridge/tests/test_mocap_v2_monitor.py \
  deploy/mocap_bridge/tests/test_chingmu_ball_tracker_roi.py

cd /home/loco1/BOB/Hitter/RobotBridge4/deploy
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  tests/test_hitter_planner_failure_reasons.py \
  tests/test_hitter_completed_result_queue.py \
  tests/test_hitter_single_shot_lifecycle.py \
  tests/test_real_world_v2_consumer.py \
  tests/test_hitter_waiting_anchor.py \
  tests/test_hitter_runtime_single_shot_integration.py \
  tests/test_hitter_task_observation.py \
  tests/test_hitter_policy_first_frame_transition.py \
  tests/test_mujoco_hitter_track_id_v2.py

cd /home/loco1/BOB/Hitter/RobotBridge4
cmake -S unitree_sdk2 -B unitree_sdk2/.build-robotbridge4-v2
rg -n -F \
  'CMAKE_HOME_DIRECTORY:INTERNAL=/home/loco1/BOB/Hitter/RobotBridge4/unitree_sdk2' \
  unitree_sdk2/.build-robotbridge4-v2/CMakeCache.txt
cmake --build unitree_sdk2/.build-robotbridge4-v2 --target trans -j2
plan_commit=$(git log -1 --format=%H -- \
  docs/superpowers/plans/2026-08-13-hitter-single-shot-track-id-safety.md)
test -n "${plan_commit:?}"
git diff --check "${plan_commit}..HEAD"
```

在运行上述 CMake 命令前，用 `apply_patch` 在 `.git/info/exclude` 追加且只追加一行 `unitree_sdk2/.build-robotbridge4-v2/`（已有完全相同行则不重复），不得 stage/commit；随后用 `git check-ignore -v unitree_sdk2/.build-robotbridge4-v2/CMakeCache.txt` 验证命中该精确行。不得运行 `cmake --build unitree_sdk2/build`。

Expected: 所有 binary exit 0；所有 pytest PASS；`CMAKE_HOME_DIRECTORY` 精确指向 RobotBridge4；`trans` target 在专用 build 目录中构建成功；从本计划提交到当前 HEAD 的实现 diff 无 whitespace error。不得用未限定的 working-tree `git diff --check` 把用户实施前改动误判为本任务失败。

- [ ] **Step 6: 运行 diagnostics/replay/export 全套回归**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4/deploy
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover \
  -s tests -p 'test_hitter_task_*.py' -v
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python \
  tests/test_hitter_task_csv_export.py -v
```

Expected: 全部 PASS；不存在 `track_epoch` active-runtime 字段、v1 online channel 或 latest-result controller 消费路径。

- [ ] **Step 7: 提交最终测试与文档**

```bash
git add deploy/tests/test_hitter_v2_end_to_end.py \
  docs/hitter_single_shot_track_id_real_world_acceptance.md
git add -p -- deploy/tests/test_hitter_task_diagnostics_integration.py
git add -p -- deploy/tests/test_hitter_task_diagnostics_performance.py
git add -p -- deploy/tests/test_hitter_task_diagnostics_safety.py
git add -p -- docs/hitter_task_observation_diagnostics.md
git diff --cached --check
git diff --cached --name-status
git commit -m "test: verify HITTER v2 single-shot safety"
```

- [ ] **Step 8: 最终只读 completion audit**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
git status --short
git log --oneline --decorate -20
rg -n 'vicon_state_data(?!_v2)' deploy \
  --glob '*.py' --glob '*.cpp' --glob '*.yaml' --pcre2
rg -n '\btrack_epoch\b' deploy/simulator deploy/envs deploy/utils
rg -n 'latest_result\(' deploy/envs/hitter.py deploy/diagnostics/hitter_task_pipeline.py
```

Expected: status 只剩实施前已记录的用户修改/删除/未跟踪项；active publisher/controller/config 没有 v1 channel；active runtime 没有 `track_epoch`；controller/shadow 不调用 `latest_result()`。任何命中先逐条确认是否只属于明确的 legacy replay fixture，再决定 completion。
