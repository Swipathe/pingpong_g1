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

