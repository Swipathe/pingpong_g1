# Task 3 报告：ChingMu 三态 Track ID parity 与严格 v2 monitor

## 状态与提交

- 状态：完成。
- 提交：`3d8014b70aa1c94b6f0ba59d583ec598d526fe2a feat: align ChingMu v2 ball identity semantics`
- 作用域：仅 RobotBridge4；未启动 publisher、真机或 PD，未修改 RobotBridge3/2、MOSAIC-main、Omega-Athlete、`unitree_sdk2/build`。

## 实现

- `deploy/mocap_bridge/chingmu_table_lcm_bridge.py`
  - 新增 `BallTrackPhase` 与五字段 `BallTrackUpdate`。
  - ChingMu ROI 筛选后，INACTIVE 按 world x/y/z、raw x/y/z 全序选择；ACTIVE 按预测距离优先并以同一坐标全序破平局，不依赖 marker 输入顺序。
  - 严格 0.35 m 关联；短 miss 进入 `missing_grace`；0.25 s 时只发一次旧 ID/最后有限位置的 end；active 允许 x<=0，未放宽 ROI/关联门限。
  - source time 非有限/不递增时使用 frame delta / `source_rate_hz`，默认 300 Hz；host clock 仅用于分配 ID，不参与连续性。
  - allocator 保证正且严格单调：0/负 clock 从 1，回退为 last+1，INT64_MAX 时保持 inactive、0 ID、无 valid/end（fail closed）。
  - pelvis/table 显式 `track_id=0`；valid/end ball 使用 tracker 正 ID；CLI/channel/summary 默认对齐 v2、300 Hz、0.35 m、0.25 s、1 Hz。
- `deploy/mocap_bridge/monitor_vicon_lcm.py`
  - 新增可测 `build_arg_parser()`、`message_contract_error()`、`message_csv_row()`；默认且只接受 `vicon_state_data_v2`、1 Hz。
  - ball 非正 ID、非 ball 非零 ID、非有限 pose 返回稳定 reason code。
  - CSV 固定字段：`track_id,valid,occluded,source_frame,source_time_s,publish_time_us,x,y,z,received_monotonic_s`。
  - 删除逐包输出；callback 只累计 count/last/error transition，主循环用 `next_summary_s += 1.0` 最多 1 Hz，contract error 首次/变化立即输出。
- 测试：新增 lifecycle/parity/monitor 测试；更新既有 ChingMu ROI 回归以断言三态语义，未统一或改变 Vicon/ChingMu 各自 ROI。

## TDD 证据

RED 命令（生产实现修改前）：

```text
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m pytest -p no:cacheprovider \
  deploy/mocap_bridge/tests/test_chingmu_ball_track_v2.py \
  deploy/mocap_bridge/tests/test_mocap_v2_monitor.py \
  deploy/mocap_bridge/tests/test_chingmu_ball_tracker_roi.py -q
```

结果：`21 failed, 1 passed`，exit 1。失败由旧接口不接收 `source_time_s`/`allocation_time_us`、缺少 `track_id`、v1 channel 默认、缺少 monitor helpers，以及 ROI 测试仍依赖旧 `ended`/单帧结束语义导致。

GREEN（brief 精确命令，先重建 C++）：

```text
bash deploy/mocap_bridge/build_v2_mocap.sh
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m pytest -p no:cacheprovider \
  deploy/mocap_bridge/tests/test_chingmu_ball_track_v2.py \
  deploy/mocap_bridge/tests/test_mocap_v2_monitor.py \
  deploy/mocap_bridge/tests/test_chingmu_ball_tracker_roi.py -q
```

结果：`22 passed in 0.13s`，exit 0。Git index 导出快照复测：`22 passed in 0.12s`，证明提交内容不依赖未暂存用户 hunks。

补充回归：

- `pytest deploy/mocap_bridge/tests -q`：`78 passed in 0.60s`。
- `deploy/mocap_bridge/.build-v2/test_transformation_t_v2`：exit 0。
- `deploy/mocap_bridge/.build-v2/test_vicon_ball_track_v2`：`PASS: Vicon v2 ball track contract`，exit 0（期间预期打印严格参数拒绝信息）。
- `test_chingmu_pelvis_orientation.py`：`47 passed in 0.18s`。

## C++ / Python fixture parity 与 ROI

重建后的真实 C++ binary `--emit-track-fixture` 与 `python_contract_fixture()` 各返回 6 项，比较结果：`PARITY=True`。序列为：同 ID 的 active(0.8)、active(0.5)、missing_grace、active(-0.02)、一次 inactive/end、随后更大 ID 的 active(0.9)。JSON 每项固定仅 `phase,track_id,publish_valid,publish_end,position_world`。

parity 只覆盖相同的已筛选 candidate/frame/source-time 序列；没有声称 marker 级 ROI 一致。ChingMu far-edge、width、height、corner/raw 逻辑保留，ROI 测试包含 x>0 新轨准入、active 跨 x=0、far-edge grace/end；C++ 自有 ROI/track contract binary 同时 PASS。

## 精确暂存与用户脏 hunk 保留

- 新测试先确认不存在；ROI 测试修改前 clean。
- 使用显式 `git add` 与 `git add -p`/manual edit；未使用 `git add .` / `git add -A`。
- commit 前 `git diff --cached --check` 无输出；`git diff --cached --name-status` 精确为：
  - M `deploy/mocap_bridge/chingmu_table_lcm_bridge.py`
  - M `deploy/mocap_bridge/monitor_vicon_lcm.py`
  - A `deploy/mocap_bridge/tests/test_chingmu_ball_track_v2.py`
  - M `deploy/mocap_bridge/tests/test_chingmu_ball_tracker_roi.py`
  - A `deploy/mocap_bridge/tests/test_mocap_v2_monitor.py`
- plan doc cached diff exit 0（未暂存）；cached 中检索 G2/pelvis translation 与 monitor G2 均无命中。
- 提交后用户修改仍在未暂存区：
  - `chingmu_table_lcm_bridge.py`：`88 insertions, 5 deletions`；包含 `dataclass, field`、G2 默认、translation schema/校验/保存/加载、`pelvis_position_world` 应用与日志。
  - `monitor_vicon_lcm.py`：`1 insertion, 1 deletion`；仅 G1Pelvis -> G2Pelvis 默认。
  - 当前未暂存 SHA-256 分别为 `b11abc26d2fba6a9e7f802dd94a001dae0fe16acc4a9e617c267ee622dc2f95d`、`2dcefac6f033c58a1a65a0ceea6cbbefe661d1287135ac0fdec1a256ed1270f6`。它们相对新 HEAD 改变是 Task 3 正常提交后的预期；语义逐项仍在。

## 自审与 concerns

- 自审：commit 仅 5 个目标文件；`git show --check HEAD` clean；staged AST 与 index 快照测试通过；没有恢复旧删除测试，也未修改 `bin/build_cpp_probe.sh`。
- 现场安全：本任务未做在线 LCM/SDK/真机验证，符合只读离线边界；上线前仍需由后续任务验证真实 v2 producer/consumer 组合。
- 边界说明：按裁决，用户 G2/pelvis hunks没有进入独立 commit，所以 commit 单独检出时保留原 HEAD 的 G1/base-world 默认；当前工作树叠加未暂存用户 hunks后是 G2/pelvis-translation 实际组合。

## Fix round 1：base-invalid 冻结与 monitor fail-closed

### Findings 与实现

1. CRITICAL base invalid：根因是 Python `process_frame()` 无条件调用 `BallTracker.update()`，缺失 C++ `AdvanceBallTrackForBaseFrame(..., base_valid)` 的冻结边界。修复为只有 `body_pose is not None` 才推进 tracker；否则返回当前 phase/ID/最后位置的非发布 update，不更新 source frame/time、不 end、不分配。
2. IMPORTANT callback fail-closed：根因是 decode 无保护且合同验证前先更新 `last_by_name`，合同错误也写 CSV。新增可测 `MonitorCallback`，顺序固定为 decode -> name/contract -> CSV field extraction -> accepted last/CSV；任一步错误均记录 stable reason 并 return。坏包之后合法 generated message 仍可处理。
3. IMPORTANT subject error transition：根因是单一 `last_contract_error` 被正常 table/pelvis 清空。修复为 `error_by_name` 按 subject 保存状态；同错误只首次输出，reason 变化输出一次，subject 恢复输出一次；其它健康 subject 不干扰 ball error。
4. 同 finding 自审补充：`track_id=inf` 会使 `int()` 抛 `OverflowError`；加入真实失败测试后将其归入 `track_id_not_integer`，callback 保持存活。

### TDD RED / GREEN

第一轮 RED（生产修复前）：聚焦 Task 3 命令结果 `5 failed, 22 passed in 0.24s`，exit 1。失败精确覆盖 ACTIVE 冻结、MISSING_GRACE 冻结、缺少可测 fail-closed callback 的 decode/invalid-state/subject-transition 三项。

最小实现后的中间 GREEN 为 `28 passed in 0.18s`。自审新增 overflow 测试先取得第二次 RED：`test_mocap_v2_monitor.py` 为 `1 failed, 11 passed in 0.13s`，`OverflowError: cannot convert float infinity to integer`；最小捕获后 monitor `12 passed in 0.06s`、最终聚焦套件 `29 passed in 0.19s`。

完整 GREEN：

- 重建 `bash deploy/mocap_bridge/build_v2_mocap.sh`：exit 0。
- 聚焦 publisher/parity/monitor/ROI：`29 passed in 0.19s`。
- 最终 `pytest deploy/mocap_bridge/tests -q`：`85 passed in 0.65s`。
- C++ `test_transformation_t_v2`：exit 0。
- C++ `test_vicon_ball_track_v2`：`PASS: Vicon v2 ball track contract`，exit 0。
- 重建后二进制与 Python fixture：`FIX_ROUND_FIXTURE_PARITY=PASS rows=6`。
- Git index 导出快照聚焦复测：`29 passed in 0.19s`，证明 fix commit 不依赖未暂存用户 hunks。

### 精确 staging、提交与用户 hunk 证明

- 独立 fix commit（未 amend）：`72a549b23e68e87f7940e65ba68a4333fe230e8a fix: freeze ChingMu tracks and harden v2 monitor`。
- cached 仅 4 文件：两个生产文件与两个覆盖测试文件；`git diff --cached --check` 无输出，staged AST 均 PASS。
- cached numstat：ChingMu `+11/-7`、monitor `+82/-43`、track test `+128/-4`、monitor test `+118/-0`。
- plan cached exit 0；cached 检索 ChingMu G2/pelvis translation 和 monitor G2 均 exit 1，无用户 hunk。
- 提交后用户修改仍未暂存：ChingMu `+88/-5`（G2、translation schema/保存/加载/应用/日志），monitor `+1/-1`（G2 默认）；controller plan doc仍未暂存。

### Concerns

- 没有在线 publisher、LCM 网络、SDK、真机或 PD 验证；本 fix round 仅离线 generated-message callback 与状态机行为验证。
- 评审记录的 0.35 浮点测试 minor 按 controller 裁决 deferred，本轮未扩大范围或更改关联边界。
