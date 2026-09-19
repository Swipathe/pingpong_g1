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

这里的跨发布端 parity 只证明“收到同一已筛选 candidate 序列后”的关联状态机/ID/end 决策相同，不宣称 Vicon 和 ChingMu 原始 marker ROI 完全相同。两端保留各自已验证的 SDK/ROI（坐标、far-edge、corner/raw-ignore）并分别跑现有 ROI 回归；计划不得为追求 fixture 一致而改变现场 ROI。若未来要求原始 marker 级 parity，需另立设计统一候选筛选参数和标定，不能在本任务中暗改。

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
