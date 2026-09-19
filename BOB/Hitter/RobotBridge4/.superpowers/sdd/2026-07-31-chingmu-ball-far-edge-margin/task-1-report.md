# Task 1 实施报告：Chingmu BallTracker 远端 40 cm 留白

## 状态

DONE

## 实现范围

- 在 `deploy/mocap_bridge/chingmu_table_lcm_bridge.py` 的常量区新增
  `BALL_TRACKING_FAR_EDGE_MARGIN_M = 0.40`。
- 在 `BallTracker.update()` 中计算
  `maximum_ball_x_m = config.table_length_m - BALL_TRACKING_FAR_EDGE_MARGIN_M`。
- 将球候选 X 上界过滤改为 `world[0] > maximum_ball_x_m` 时拒绝，因此
  `x == maximum_ball_x_m` 仍被接收。
- 新增
  `deploy/mocap_bridge/tests/test_chingmu_ball_tracker_roi.py`，覆盖恰好位于
  上界和超过上界 `0.001 m` 的两个真实行为。
- 提交已批准计划
  `docs/superpowers/plans/2026-07-31-chingmu-ball-far-edge-margin.md`。

没有修改 `table_length_m`、桌面标定、Y/Z 门控、planner 参数、命令行参数或
配置项。

## TDD 证据

### RED

在写入生产代码前运行：

```bash
/home/loco1/miniconda3/envs/rb/bin/python -m unittest \
  deploy.mocap_bridge.tests.test_chingmu_ball_tracker_roi -v
```

结果：退出码 `1`，`Ran 1 test`，`FAILED (failures=1)`。失败符合预期：

```text
AssertionError: array([2.331738, 0.      , 0.9     ]) is not None
```

旧实现错误接收了 `x = table_length_m - 0.399 m` 的候选；失败不是导入、
语法或测试装配错误。

### GREEN：专用测试

写入最小生产实现后运行同一命令。

结果：退出码 `0`，`Ran 1 test`，`OK`。

### GREEN：相关回归

运行：

```bash
/home/loco1/miniconda3/envs/rb/bin/python -m unittest \
  deploy.mocap_bridge.tests.test_chingmu_ball_tracker_roi \
  deploy.mocap_bridge.tests.test_chingmu_pelvis_orientation -v
```

首次结果：退出码 `0`，`Ran 47 tests in 0.080s`，`OK`。

提交后的新鲜复验结果：退出码 `0`，`Ran 47 tests in 0.128s`，`OK`。

## 语法和差异检查

运行：

```bash
/home/loco1/miniconda3/envs/rb/bin/python -m py_compile \
  deploy/mocap_bridge/chingmu_table_lcm_bridge.py \
  deploy/mocap_bridge/tests/test_chingmu_ball_tracker_roi.py
git diff --check -- \
  deploy/mocap_bridge/chingmu_table_lcm_bridge.py \
  deploy/mocap_bridge/tests/test_chingmu_ball_tracker_roi.py
```

结果：两条命令均退出码 `0`，无错误输出；提交后又执行一次，结果相同。

提交前执行 `git diff --cached --check`，退出码 `0`，无错误输出。提交后执行
`git show --check --oneline --stat HEAD`，退出码 `0`。

## 提交

- Commit：`15f19158ad47098812668fb418dbe71bdd81c7ef`
- Message：`fix: delay Chingmu ball admission by 40 cm`
- Base：`c4399c5c8749130d85785b39d3555d5a5a0d26fe`

`git diff-tree --no-commit-id --name-status -r HEAD` 只列出：

```text
M  deploy/mocap_bridge/chingmu_table_lcm_bridge.py
A  deploy/mocap_bridge/tests/test_chingmu_ball_tracker_roi.py
A  docs/superpowers/plans/2026-07-31-chingmu-ball-far-edge-margin.md
```

生产文件在任务开始前已有用户的两个 `G1Pelvis -> G2Pelvis` 未提交 hunk。
提交时使用交互式精确暂存，明确跳过这两个 hunk；提交后
`git diff HEAD -- deploy/mocap_bridge/chingmu_table_lcm_bridge.py` 仍只显示
这两个 G2Pelvis hunk，证明它们被保留在工作区且未进入本任务提交。提交后
暂存区为空。

## 自审

- 逐项对照唯一需求 brief：常量名、常量值 `0.40`、局部变量名、计算表达式、
  过滤比较符和测试内容均一致。
- 包含边界正确：过滤使用严格大于 `>`；恰好等于上界时不触发拒绝。
- 测试直接调用真实 `BallTracker.update()`，没有 mock；期望值使用显式
  `0.40` 和 `0.001`，没有复用生产常量或生产计算逻辑。
- 变异检查：若把生产比较恢复为 `world[0] > config.table_length_m`，RED
  证据表明测试会失败；若把 `>` 错改为 `>=`，上界接受断言会失败。
- 原有 Y、Z、桌角排除、初始 `x > 0` 和 active track 逻辑未改。
- 没有暂存、提交、回退或清理 dirty worktree 中任何无关文件。

## 独立只读审查

独立 reviewer 以 `c4399c5..15f1915` 为范围，逐项检查 brief、提交差异和周边
代码。结论：

- Critical：无。
- Important：无。
- Minor：无。
- Assessment：`Ready to merge: Yes`。

Reviewer 确认常量值、运行时 cutoff 计算、严格大于比较形成的包含边界、真实
行为测试、Y/Z 门控不变以及三文件提交范围均符合要求；只读
`git diff --check` 和 AST 解析通过。无需追加代码修改。

## Concerns

无实现 blocker 或已知正确性问题。当前仓库仍保留大量用户的未提交改动，
包括生产文件中的两个 G2Pelvis hunk；这是任务开始前的既有状态，本任务按
要求完整保留且没有纳入提交。
