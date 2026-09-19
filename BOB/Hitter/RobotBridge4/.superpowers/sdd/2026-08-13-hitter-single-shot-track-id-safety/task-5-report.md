# Task 5 报告：typed planner failure 与绝对球桌 y 手型规则

## 状态

- 实现完成；提交为 `15063ca88fd2e6ff7ea120201631c40c82f1953d`。
- 变更限于 RobotBridge4；未修改 RobotBridge3/2、MOSAIC、Omega、`unitree_sdk2` 或 `build`，未启动真机、PD、viewer 或网络运行时。
- Task 8 的首次 `ARMED` 后锁定未实现；Task 6 的 unexpected exception 到 `INTERNAL_ERROR` 转换也未提前实现。

## TDD：RED / GREEN

- 首次 RED：先创建真实 planner/env 测试并扩充 factory/log 测试，运行 brief 三文件命令；收集期按预期因缺少 `PlannerFailureReason/PlannerRejected` 失败（exit 2）。
- 第二次 RED：补充 system planner 非有限输出边界，两个 case 均得到 `DID NOT RAISE PlannerRejected`。
- GREEN：最小实现后 brief 三文件 fresh 结果为 `32 passed, 2 warnings`；warnings 是现有 `invalid escape sequence` deprecation warning。
- 测试只断言 `PlannerRejected.reason is PlannerFailureReason.*`，不依赖异常 detail 文本；factory 的真机禁用测试只校验独立配置错误消息。

## 八个 reason 的边界

- `_plan_hitter_snapshot()`：`TRACK_ENDED`、`ESTIMATOR_NOT_READY`、`BASE_POSE_INVALID`、`BALL_NOT_INCOMING`。
- `StrikePlanner` / `HitterSystemPlanner`：`NO_FUTURE_CROSSING`、`HIT_HEIGHT_OUT_OF_RANGE`、`NONFINITE_INPUT_OR_OUTPUT`；后者覆盖输入、trajectory 和 command 输出。
- `INTERNAL_ERROR` 已定义并由枚举全集测试固定，但 unexpected exception 仍走原有通用异常结果；按 brief 留给 Task 6 worker 边界测试与实现。
- construction/config 的非有限与形状校验仍默认抛普通 `ValueError`；只有 planner 调用边界传入 `reject_nonfinite=True`，没有异常字符串分类回退。

## 绝对 table-y 与 override

- 自动手型唯一规则：预测 racket target 的 world/table `y < 0` 为 `forehand`，`y >= 0` 为 `backhand`；`-1e-9 / 0 / +1e-9` 分别验证为 forehand/backhand/backhand。
- 每个边界点循环 `base_y=-0.8/0/0.9` 与 `yaw=-1.2/0/2.1` 共 9 组真实 `HitterSystemPlanner + BaseTargetPlanner` 路径，结果不随 base y/yaw 改变。
- `BaseTargetPlanner.plan()` 现在要求显式 `strike_type` 并仅校验/使用 forehand/backhand；自动判定上移到 system planner。
- command 无默认值地携带 `strike_table_y_w` 和 `strike_side_source`；自动为 `table_y`，显式 override 为 `forced`，strike log 同时输出两项。
- `forced_strike_type(..., is_real_world=True)` 只在解析到实际 forehand/backhand override 时启动失败；None/空/null 不是 force。`HitterEnv` 在 planner worker 初始化前缓存并校验，MuJoCo/offline 仍可 override。

## 测试

- brief 聚焦：`32 passed, 2 warnings`。
- 从 `git archive HEAD` 导出的纯提交快照重复 brief 聚焦：`32 passed, 4 warnings`，证明提交不依赖 unstaged 用户 hunks。
- 直接影响回归：MuJoCo track-id v2、task pipeline、task replay、diagnostics safety 合计 `72 passed, 2 warnings`。
- `python -m py_compile` 覆盖 4 个生产文件与 3 个测试文件，通过；`git show --check` 和 cached `diff --check` 通过。
- 扩大但非验收基线：加入 diagnostics integration 时为 `72 passed, 2 failed`，失败是当前 dirty G2/diagnostics capture/incoming parity；加入用户未跟踪物理测试时为 `88 passed, 7 failed`，包括当前 XML friction 与旧 `_hitter_sync_track_id`/first-frame fixture。均不在 Task 5 diff 中，未扩大范围修复。

## staging 与 dirty 保护

- clean 的 runtime types、新 planner test、strike log 精确 add；planner/env/runtime factory/factory test 使用逐 hunk staging，两个与用户 hunk相邻的 planner 签名单行用 cached patch 精确加入。
- 提交前 `git diff --cached --name-status` 只有 7 个 Task 5 文件；无 D、无 `.superpowers` controller plan/report、无 docs。
- cached diff 明确不含 maximum-height、racket velocity range/clip、G2/104-D、minimum-arm/config 测试等用户 hunks。
- 提交后 index 为空，四个交叠文件继续为 unstaged `M`；逐项检查仍含用户的 `maximum_hit_height=1.45`、velocity component range/clip、G2/104-D 与 factory velocity/config test。
- 本报告命中 `.git/info/exclude:7:.superpowers/`，只保留本地且未提交。

## 自审 / Concerns

- `PlannerRejected` 唯一定义在 `utils/hitter_runtime_types.py`；planner/env 直接使用 enum identity，没有字符串反解析。
- generic MuJoCo exception 仍保留原有 `TypeName: detail`，没有提前映射 `INTERNAL_ERROR`。
- 当前工作树整体存在大量用户修改、删除及未跟踪文件；本提交只包含上述 Task 5 范围，未 restore/reset/clean，也未改 controller plan。
