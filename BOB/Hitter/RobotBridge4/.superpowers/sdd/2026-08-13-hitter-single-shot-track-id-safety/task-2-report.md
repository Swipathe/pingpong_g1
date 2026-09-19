# Task 2 报告：C++ Vicon 三态物理球轨迹与 v2 发布

## 状态与提交

- Status: PASS
- Commit: `9c58d69f905040ac505225c60619b88fb6d66138 feat: track physical balls in Vicon publisher`
- 修改范围严格限于 RobotBridge4 的三个 Task 2 文件：
  - `deploy/mocap_bridge/vicon_table_lcm_bridge.cpp`
  - `deploy/mocap_bridge/tests/test_vicon_ball_track_v2.cpp`
  - `deploy/mocap_bridge/build_v2_mocap.sh`
- 未运行或写入 `unitree_sdk2/build`，未启动真机、Vicon 在线发布或 PD。

## 开始态与用户脏 hunk

开始前核验：

- 分支：`local/robotbridge4-single-shot-track-id-20260813`
- BASE/HEAD：`8bc598a feat: add HITTER v2 track id schema`
- 新测试路径不存在。
- `build_v2_mocap.sh` clean，且仅输出到 `deploy/mocap_bridge/.build-v2`。
- 用户 cpp worktree blob：`006c74243e04937576325ce301194dee96fca524`
- 用户 cpp diff：`+9/-8`
- 用户 cpp diff SHA-256：`c57c6c0bf26f48e8c9c0428261bb638a00ee61002682f20efcec6ae0f2653108`
- 说明：brief 中的 `c57c...` 是 `git diff` 内容的 SHA-256；文件本身的 SHA-256 是 `b9af9d9e...`，只读复核后不存在开始态歧义。

既有用户 hunks 全部保留：

1. 默认 base subject `G1Pelvis -> G2Pelvis`。
2. help 中 base subject `G1Pelvis -> G2Pelvis`。
3. table frame 校验的两个 pelvis 错误文本改为 G2。
4. corner 建帧的 pelvis 错误文本改为 G2。
5. 当前 calibration pelvis 错误文本改为 G2。
6. calibration root 的两个错误文本改为 G2。
7. calibration marker loop 增加 `IgnoredRawMarker(p, args)`。

## 契约裁决

实现前发现并上报两处 plan/brief 冲突，均先取得主代理裁决：

1. brief 指定的 7 参数 `AdvanceBallTrack(...)` 没有 frame rate，但非有限/不递增 source time 的 fallback 需要 frame rate。裁决为保留 7 参数公共测试/parity 接口，内部增加 translation-unit 私有 `AdvanceBallTrackWithFrameRate(..., source_frame_rate_hz)`；公共接口固定委托 300 Hz，main 传 SDK/fallback 实际选中的 rate。两条路径复用同一个 source-time 规范化实现。
2. brief 初始示例从 `x=0.80` 无速度直接跳到 `x=-0.02`，距离 `0.82 m`，与严格 `0.35 m` 关联门限冲突。裁决为绝不放宽安全门限：先增加门限内 approach 帧建立速度，再在 CV 预测半径内覆盖 `x<=0`；另保留独立三帧 bounce 测试并显式断言每步预测距离不超过 `0.35 m`。

## 实现

- 增加 `BallTrackPhase::{Inactive, Active, MissingGrace}`、`BallTrackState`、`BallTrackUpdate`。
- `INACTIVE` 只接受有限且 `x > 0` 的候选；`ACTIVE/MISSING_GRACE` 使用恒速预测，仅接受预测点 `0.35 m` 内的最近候选，边界使用 `<=`。
- 使用 source time 判定速度和结束；source time 非有限或不递增时使用 source frame delta / frame rate。host steady/system clock 不参与连续性判定；system clock 只用于 Track ID 分配和消息发布时间。
- Track ID 使用 `max(last_allocated + 1, allocation_unix_time_us)`，始终为正且即使分配时钟回退也严格单调。
- 短暂丢球进入 `MissingGrace`；达到 `0.25 s` 后只发布一次同 ID 的 invalid end，随后 reset 但保留 allocator。
- pelvis invalid 通过 `AdvanceBallTrackForBaseFrame` 返回只读快照，不推进、不丢失也不结束物理球轨迹。
- base/table 消息 `track_id=0`；valid/end ball 使用相同正 ID。
- 默认/唯一 channel 为 `vicon_state_data_v2`；`--channel` 拒绝 v1，两个轨迹参数必须有限且为正；默认打印频率降为 1 Hz，状态转换立即输出。
- main 仅向 `args.channel` 发布，Active 发布 valid ball，结束沿只发布一次 invalid/occluded ball。
- 新测试 binary 支持 `--emit-track-fixture`，JSON 每项严格只有 `phase, track_id, publish_valid, publish_end, position_world`。
- 构建脚本无条件构建新 C++ 测试 target，所有输出仍在 `.build-v2`。

## TDD 证据

### RED

先仅添加测试和构建 target，运行：

```bash
bash deploy/mocap_bridge/build_v2_mocap.sh
```

结果：exit 1；关键输出：

```text
error: ‘AdvanceBallTrack’ was not declared in this scope; did you mean ‘UpdateBallTrack’?
```

同时明确失败于缺少 `BallTrackPhase`、`BallTrackUpdate`、带 Track ID 的 `FillMessage` 和严格 v2 Args，确认测试命中了缺失行为。

### 首次 GREEN 调试与安全裁决

最小生产实现后 build exit 0，但单测真实失败：

```text
FAIL: x<=0 and reversal retain id
```

原因是原示例无先验速度时距离为 `0.82 m > 0.35 m`。没有放宽门限；按裁决加入 approach 帧后继续 GREEN。

### 最终 GREEN / 提交后验证

精确运行：

```bash
bash deploy/mocap_bridge/build_v2_mocap.sh
deploy/mocap_bridge/.build-v2/test_vicon_ball_track_v2
deploy/mocap_bridge/.build-v2/vicon_table_lcm_bridge_v2 --help | \
  grep -F 'vicon_state_data_v2'
```

结果：全部 exit 0；关键输出：

```text
PASS: Vicon v2 ball track contract
Usage: ... [--channel vicon_state_data_v2] \
  [--ball-track-association-radius-m 0.35] \
  [--ball-track-end-timeout-s 0.25] ...
```

回归：

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. \
  /home/loco1/miniconda3/envs/rb/bin/python -m pytest -p no:cacheprovider \
  deploy/mocap_bridge/tests/test_transformation_t_v2.py \
  deploy/mocap_bridge/tests/test_vicon_table_lcm_bridge.py \
  deploy/mocap_bridge/tests/test_vicon_sdk_client.py -q
```

结果：`9 passed in 0.42s`。

post-commit 还运行了 `git show --check --oneline --stat HEAD`、cached-empty、未暂存 `+9/-8` 和 commit 用户-hunk排除断言，最终输出 `POST_COMMIT_VERIFY=PASS`。

## 测试覆盖

- 首次合格候选分配注入的正 ID。
- approach 建速后 `x<=0` 仍保持 ID。
- 短 miss 保持 grace；超时只 end 一次。
- `0.35 m` 精确边界仍重关联；`0.350001 m` 不抢占。
- bounce 后 x 和速度反号仍保 ID，并显式验证每步门限距离。
- end 后新轨迹 ID 严格增加；分配时钟回退时精确等于旧 ID + 1。
- 非有限和不递增 source time 的 300 Hz fallback。
- internal/main helper 的非 300 Hz（100 Hz）fallback。
- pelvis invalid 不结束轨迹。
- base/table ID 为 0；valid/end ball ID 相同且为正；end message invalid。
- strict v2 channel；association/timeout 的零、NaN、Inf 拒绝。
- canonical fixture 为有效 JSON，共 6 行且 key schema 精确。

## 精确暂存与用户 hunk 保留证明

按 brief 执行：

```bash
git add deploy/mocap_bridge/tests/test_vicon_ball_track_v2.cpp \
  deploy/mocap_bridge/build_v2_mocap.sh
git add -p -- deploy/mocap_bridge/vicon_table_lcm_bridge.cpp
git diff --cached --check
git diff --cached --name-status
```

`git add -p` 对两个与用户 G2 行相邻的 hunk 使用 manual edit，只保留 Task 2 新增行；其余用户 hunks全部选择不暂存。提交前结果：

```text
M  deploy/mocap_bridge/build_v2_mocap.sh
A  deploy/mocap_bridge/tests/test_vicon_ball_track_v2.cpp
M  deploy/mocap_bridge/vicon_table_lcm_bridge.cpp
```

- `git diff --cached --check`：无输出，exit 0。
- cached cpp 搜索 `G2Pelvis|IgnoredRawMarker(p, args)`：无命中。
- 提交后 cached 为空。
- 提交后 cpp 仍为未暂存 `+9/-8`，7 个用户 hunks逐项存在。
- 提交后 `git show HEAD -- cpp` 搜索用户 G2/ignore 行：无命中。
- 提交后用户 diff SHA-256 为 `97a63749cff897ec1ad4882ee740d679e5540855b1d1b3c1c56fca82a712c77c`。它因 Task 2 新增上下文、index blob 和行号改变而不同于开始态 `c57c...`，但 `+9/-8` 与全部用户语义行保持不变。

## 自审

- 安全门限没有过台特判或任何隐式放宽；inclusive 边界和超界均有独立测试。
- 连续性逻辑没有 steady/system host clock；实际 SDK/fallback rate 从 main 显式传入内部 helper。
- invalid pelvis 路径不会调用推进逻辑；ball end 只有状态机 timeout 边沿产生。
- allocator reset 保留 `last_allocated_track_id`，并有时钟回退 mutation 覆盖。
- 发布协议中没有 v1 channel 字面量；所有 runtime publish 都使用 strict `args.channel`。
- base/table/ball ID 合同由真实生成的 `transformation_t` 断言，而不是 source grep。
- 未恢复已删除旧测试，未修改已有未跟踪 `build_cpp_probe.sh` 或 `bin/`。

## Concerns

- 本任务只做离线 C++/Python 构建与单测，没有连接真实 Vicon、在线 LCM 或真机；真实 SDK 帧率/marker 数据的现场验证留给受控集成阶段。
- 当前 worktree 运行时仍叠加用户未提交的 G2Pelvis/校准 ignore hunks；Task 2 commit 本身刻意不包含它们，因此 checkout 该 commit 单独运行时仍继承 BASE 的 G1Pelvis 默认值。
- Task 3 必须复制本报告中的 6 行 canonical fixture（包含 approach 帧）；不得恢复计划中无速度直接 `0.80 -> -0.02` 的冲突序列，也不得放宽 `0.35 m` 门限。

---

## Fix round 1/5：review Important findings

### 状态与提交

- Status: PASS
- 独立 fix commit：`c55b5929327cf389799273e418719ffba6464cec fix: harden Vicon ball track contracts`
- 只提交：
  - `deploy/mocap_bridge/vicon_table_lcm_bridge.cpp`
  - `deploy/mocap_bridge/tests/test_vicon_ball_track_v2.cpp`
- 未 amend Task 2 原提交；未修改当前未提交 plan doc。

### Finding 1：候选顺序确定性

核验 reviewer 结论：`UnlabeledMarkers` 逐 SDK index push，状态机原实现 INACTIVE 在首个合格候选处 `break`，ACTIVE 对等距候选只保留首个，因此同一候选集合的排列会改变结果。

先只添加 INACTIVE 与 ACTIVE 等距候选的 forward/reverse permutation 测试，运行：

```bash
bash deploy/mocap_bridge/build_v2_mocap.sh && \
  deploy/mocap_bridge/.build-v2/test_vicon_ball_track_v2
```

RED：exit 1，关键输出：

```text
FAIL: inactive permutations choose the same candidate
```

实现稳定全序：

1. 候选主序为 `world.x, world.y, world.z`。
2. world 完全相同时继续按 `raw.x, raw.y, raw.z`。
3. INACTIVE 在所有有限且 `world.x > 0` 的候选中选择上述全序最小项。
4. ACTIVE/MISSING_GRACE 仍以预测距离为第一主键；距离相等时使用同一候选全序。

GREEN：同一命令 exit 0，输出 `PASS: Vicon v2 ball track contract`。置换测试断言 forward/reverse 输出位置一致，并分别固定 INACTIVE 选择 `(0.40, 0.20, 1.00)`、ACTIVE 等距选择 `(0.75, 0.00, 1.00)`。

### Finding 2：Track ID allocator 耗尽

核验 reviewer 结论：原 `last_allocated_track_id + 1` 在 `INT64_MAX` 上有有符号溢出 UB；普通正值 clock rollback 测试不能捕获。

先增加 `INT64_MAX`、重复耗尽、allocation time 为 0 和负数测试。RED：

```bash
bash deploy/mocap_bridge/build_v2_mocap.sh
```

exit 1，关键输出：

```text
error: ‘const struct BallTrackUpdate’ has no member named ‘error’
error: ‘BallTrackError’ has not been declared
```

实现：

- 新增 `BallTrackError::{None, AllocatorExhausted}`，作为 `BallTrackUpdate.error` 的显式结果。
- 在任何 `+1` 前检查 `last_allocated_track_id == INT64_MAX`。
- 耗尽返回 `phase=Inactive, track_id=0, publish_valid=false, publish_end=false, error=AllocatorExhausted`；state 保留已耗尽 allocator，不饱和、不复用。
- allocation time 为 0/负数时，未使用 allocator 从 ID 1 开始。
- main 收到非 None error 立即输出 `ball_track protocol_error=allocator_exhausted`、设置非零退出并在发布分支之前 break。

GREEN：build/test exit 0，contract PASS；测试覆盖首次耗尽、重复耗尽、零/负分配时钟，以及普通 clock rollback 的严格 `old_id + 1`。

### Finding 3：strict v2 CLI 不抛出、不接受脏尾缀

核验 reviewer 结论：两个数值参数直接 `std::stod(next())`；缺值从 `next()` 调 `std::exit(2)`，`abc` 抛异常，`0.35junk` 被前缀解析接受。

先增加表驱动用例：Task 2 的 channel/两个数值参数缺值；两个数值参数 `abc`；`0.35junk` / `0.25junk`。RED：

```bash
bash deploy/mocap_bridge/build_v2_mocap.sh && \
  deploy/mocap_bridge/.build-v2/test_vicon_ball_track_v2
```

结果：build 成功但 test binary exit 2，最后输出：

```text
Missing value for --channel
```

证明 parser 内部退出，后续用例无法继续。

实现保持旧 CLI 参数不变，只为 Task 2 三个参数增加：

- `ConsumeRequiredArgValue`：缺值打印明确错误并返回 false，不 exit。
- `ParseFinitePositiveDoubleStrict`：捕获 `std::stod` 异常；要求 consumed 等于完整字符串长度；拒绝非有限、零和负数。
- association/timeout 的无效错误包含参数名与原值。

GREEN：build/test exit 0，contract PASS；stderr 逐项显示 missing、`abc`、NaN/Inf 与 trailing-character 的明确拒绝信息，没有异常或 parser 内部 exit。

### Fix round 1 完整 GREEN

提交前与提交后均运行：

```bash
bash deploy/mocap_bridge/build_v2_mocap.sh
deploy/mocap_bridge/.build-v2/test_vicon_ball_track_v2
deploy/mocap_bridge/.build-v2/vicon_table_lcm_bridge_v2 --help | \
  grep -F 'vicon_state_data_v2'
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. \
  /home/loco1/miniconda3/envs/rb/bin/python -m pytest -p no:cacheprovider \
  deploy/mocap_bridge/tests/test_transformation_t_v2.py \
  deploy/mocap_bridge/tests/test_vicon_table_lcm_bridge.py \
  deploy/mocap_bridge/tests/test_vicon_sdk_client.py -q
```

结果：

```text
PASS: Vicon v2 ball track contract
Usage: ... [--channel vicon_state_data_v2] ...
......... [100%]
9 passed in 0.43s
```

canonical fixture 仍为 6 行，JSON schema 仍精确为 `phase,track_id,publish_valid,publish_end,position_world`。

### Fix round 1 精确暂存与脏树证明

```bash
git add deploy/mocap_bridge/tests/test_vicon_ball_track_v2.cpp
git add -p -- deploy/mocap_bridge/vicon_table_lcm_bridge.cpp
git diff --cached --check
git diff --cached --name-status
```

- cpp 中 7 个用户 hunks 全部在 `git add -p` 选择不暂存。
- cached cpp 搜索 `G2Pelvis|IgnoredRawMarker(p, args)` 无命中。
- cached 只有两个 fix 文件；`git diff --cached --check` exit 0。
- commit 后 cached 为空。
- commit 后用户 cpp 仍为未暂存 `+9/-8`，当前 worktree blob `cdbfe2cda7fc303f6e19a79575066966106ede7c`，diff SHA-256 `3d187c64373c3a5b6d6c693750fa8afc4b5b6d926e351086a8c4eb33e35cca82`；7 个用户语义 hunks均存在。
- fix commit 的 cpp diff 搜索用户 G2/ignore 行无命中。
- 未提交 plan doc 在暂存前、提交前、提交后的 diff SHA-256 均为 `8a548b3444e3cf984efa1f4980ba635a5d1ea3245c7a95fc0a6f0b17d57ba67a`。

### Fix round 1 自审与 concerns

- 稳定选择只依赖候选 world/raw 数据与 ACTIVE 预测状态，不依赖 vector/SDK index。
- allocator 耗尽不做饱和或复用；显式错误沿 main 传播并在任何 ball publish 前 fail closed。
- strict parser 只扩展 Task 2 新参数，未顺带改变旧 CLI 的兼容行为。
- fixture schema 未增加 error 字段，保持 Task 3 跨语言 parity 合同不变。
- 仍未进行在线 Vicon/LCM 或真机验证；本轮验证范围为离线 C++/Python contract 与构建回归。
