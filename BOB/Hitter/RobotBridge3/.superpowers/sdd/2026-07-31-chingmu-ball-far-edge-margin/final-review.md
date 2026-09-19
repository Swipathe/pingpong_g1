# Chingmu 球追踪远端 40 cm 留白最终审查

## 结论

**Ready verdict: NOT READY**

| Severity | 数量 |
|---|---:|
| Critical | 0 |
| Important | 2 |
| Minor | 1 |

目标提交的远端 X 上界实现是包含边界：`x = table_length_m - 0.40` 可追踪，`x` 大于该值会被排除；真实桌长、Y/Z 门控、标定、planner、CLI 和配置均未被该提交修改。提交范围也没有纳入当前 dirty worktree 的无关改动。

但按本次审查明确给出的全局约束“候选范围必须为 `0 < x <= table_length_m - 0.40 m`”逐字核对，活动轨迹的内部候选集合仍允许 `x <= 0` 点参与最近邻关联；同时，设计要求的 X 下界、Y/Z 边界和活动轨迹远端退出行为没有形成完整的持久回归测试。因此当前不建议进入合并。

## Findings

### Important 1：活动轨迹的候选集合不满足严格的 `0 < x` 下界

证据：

- `15f1915:deploy/mocap_bridge/chingmu_table_lcm_bridge.py:860-879` 构造 `candidates` 时仅过滤新的 X 上界、Y 边界和 Z 边界；`x <= 0` 仍会加入该集合。
- 非活动轨迹直到 `:888-893` 才以 `candidates_array[:, 0] > 0.0` 做 admission。
- 活动轨迹在 `:894-905` 先让全部候选参与最近邻选择，选中 `x <= 0` 后才结束轨迹。
- 既有测试 `15f1915:deploy/mocap_bridge/tests/test_chingmu_table_lcm_bridge.py:353-379` 明确构造了同时存在的合法 `x=0.60` 点和非法 `x=-0.01` 点；负 X 点因更接近预测位置而被选中并结束轨迹。

影响：

- 有效发布位置不会出现 `x <= 0`，但内部关联候选范围并不满足本次全局约束的字面要求。
- 活动轨迹附近若同时出现负 X 未标记点和合法正 X 点，负 X 点仍可能抢占关联并触发一次无效球状态。

这也是一个既有语义冲突：当前负 X 点同时承担“越过机器人侧边界后结束轨迹”的作用。修复前应明确二选一：

1. 严格执行候选范围，在关联前排除 `x <= 0`；或
2. 若负 X 点必须作为轨迹结束证据，需将其与“可关联候选”分开建模，并在规格中明确该例外。

在未消除该规格与运行语义冲突前，本项阻塞 ready。

### Important 2：回归测试没有覆盖设计明确要求的其余边界

证据：

- 新测试 `15f1915:deploy/mocap_bridge/tests/test_chingmu_ball_tracker_roi.py:35-53` 只检查非活动轨迹的两个点：
  - `x = table_length_m - 0.40` 被接受；
  - `x = table_length_m - 0.40 + 0.001` 被拒绝。
- 它没有验证：
  - `x = 0` 拒绝和 `x > 0` 接受；
  - `y = ±table_width_m / 2` 包含及越界排除；
  - `z = table_height_m` 排除及 `z > table_height_m` 接受；
  - 已有活动轨迹收到超出新 X 上界的点时 `ended=True`，且只结束一次。
- 既有 `test_crossing_x_zero_ends_once_and_resets_track` 覆盖了活动轨迹越过 X 下界的旧语义，但不能替代上述完整边界矩阵；现有提交中也没有直接锁定 Y/Z 等号语义的测试。

实际生产代码当前的 Y/Z 语义正确且本提交未改动：`|y| = table_width_m / 2` 可接受，`z = table_height_m` 被拒绝。最终审查在目标提交的隔离归档中做的边界探针也全部通过，但临时审查探针不是仓库内的持久回归保护，不能满足设计文档“增加最小边界回归测试”的交付要求。

建议在专用 ROI 测试中补齐上述边界矩阵和活动轨迹结束路径。本项属于明确测试验收项缺失，阻塞 ready。

### Minor 1：上界测试不能精确锁定 0.40 m 常量和运行时桌长派生关系

证据：

- 测试在 `:36` 重复硬编码 `0.40`，没有断言或复用生产常量。
- 测试只使用默认 `BridgeConfig.table_length_m`，因此若生产实现错误地硬编码默认 cutoff，该测试仍可能通过。
- 越界样本只取 `cutoff + 0.001`。例如实现误用 `0.3995 m` 留白时，两个现有断言仍会通过。

建议：

- 增加一个非默认 `table_length_m` 的配置用例，证明 cutoff 由运行时真实桌长派生；
- 直接断言命名常量等于 `0.40`；
- 使用紧邻边界的可表示值（或显式小于 1 mm 的差值）锁定“任意 `x > cutoff` 均拒绝”的比较语义。

本项单独不阻塞合并，但应与 Important 2 一并修正。

## 已确认正确的部分

- `BALL_TRACKING_FAR_EDGE_MARGIN_M = 0.40` 是命名常量。
- cutoff 由 `config.table_length_m - BALL_TRACKING_FAR_EDGE_MARGIN_M` 计算，没有修改真实 `table_length_m`。
- 过滤条件使用 `world[0] > maximum_ball_x_m`，所以等于 cutoff 时包含，大于 cutoff 时排除。
- `abs(world[1]) > 0.5 * table_width_m` 和 `world[2] <= table_height_m` 两条生产门控在提交中未改动。
- 目标范围只有 1 个提交：
  - `15f1915 fix: delay Chingmu ball admission by 40 cm`
- 范围只包含 3 个文件：
  - `deploy/mocap_bridge/chingmu_table_lcm_bridge.py`
  - `deploy/mocap_bridge/tests/test_chingmu_ball_tracker_roi.py`
  - `docs/superpowers/plans/2026-07-31-chingmu-ball-far-edge-margin.md`
- 未修改标定 JSON、planner、CLI、配置或桌面发布中心。
- 审查包中的 patch 与精确 Git 范围的 `git diff --unified=10 --abbrev=7` 完全一致。

## 独立验证证据

所有运行验证均在 `15f19158ad47098812668fb418dbe71bdd81c7ef` 的临时 `git archive` 中完成，没有使用当前 dirty worktree 的生产文件，也没有改动索引、HEAD 或分支。

1. 精确范围检查：

   - `git rev-list --count c4399c5..15f1915`：`1`
   - `git diff --name-status c4399c5..15f1915`：仅上述 3 个文件
   - `git diff --check c4399c5..15f1915`：退出码 `0`

2. 目标提交测试：

   - 运行新 ROI、Chingmu pelvis orientation 和完整 Chingmu table bridge 三个测试模块；
   - `Ran 71 tests in 0.105s`
   - `OK`

3. 新回归场景的 RED/GREEN 证据：

   - 基线 `c4399c5` 对 `cutoff + 0.001`：仍接受，证明该场景在旧实现上为 RED；
   - 目标 `15f1915` 对相同点：拒绝。

4. 目标提交临时边界矩阵：

   - `x=0`：拒绝；
   - `x>0`：接受；
   - `x=cutoff`：接受；
   - `x=cutoff+0.001`：拒绝；
   - `y=±width/2`：接受；
   - `|y|>width/2`：拒绝；
   - `z=height`：拒绝；
   - `z>height`：接受；
   - 活动轨迹越过远端 cutoff：本帧 `ended=True`，下一空帧不重复结束。

这些探针证明当前输出行为和新远端上界正确，但不抵消 Findings 中的严格候选集合语义及持久测试覆盖缺口。
