# Chingmu Ball Far-Edge Margin Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 Chingmu 有效球接纳/发布区域的 X 上界从真实桌长收紧为真实桌长减去固定的 `0.40 m`，且不改变桌面标定、真实桌长和其他 Y/Z 门控。

**Architecture:** 在 `BallTracker` 模块中定义固定远端留白常量，并仅用它计算球候选点的最大 X。通过独立的边界测试同时证明新上界是包含边界的：等于上界时接受，略大于上界时拒绝。

**Tech Stack:** Python 3.8、`unittest`、NumPy、RobotBridge2 Chingmu mocap bridge。

## Global Constraints

- 修改后的新轨迹 admission 和 valid ball 发布范围必须是
  `0 < x <= table_length_m - 0.40 m`。
- 活动轨迹可继续关联内部 `x <= 0` 点作为越过机器人侧边界的结束证据；选中
  后只结束轨迹并发布一次 invalid ball，绝不把该点作为 valid ball 发布。
- `|y| <= table_width_m / 2` 保持不变。
- `z > table_height_m` 保持不变。
- 不修改 `table_length_m`、桌面标定 JSON、桌面发布位置或 planner 参数。
- 不增加命令行参数或配置项。
- 保留当前 dirty worktree 中所有无关改动。

---

### Task 1: 收紧 BallTracker 远端 X 边界

**Files:**
- Create: `deploy/mocap_bridge/tests/test_chingmu_ball_tracker_roi.py`
- Modify: `deploy/mocap_bridge/chingmu_table_lcm_bridge.py:54-63`
- Modify: `deploy/mocap_bridge/chingmu_table_lcm_bridge.py:859-875`

**Interfaces:**
- Consumes: `BridgeConfig.table_length_m: float`、`BallTracker.update(...) -> BallTrackUpdate`。
- Produces: `BALL_TRACKING_FAR_EDGE_MARGIN_M: float = 0.40`，以及包含边界 `x <= table_length_m - BALL_TRACKING_FAR_EDGE_MARGIN_M` 的有效球接纳/发布行为；活动轨迹既有的 `x <= 0` 内部结束证据语义保持不变。

- [ ] **Step 1: 写入失败的包含边界测试**

创建 `deploy/mocap_bridge/tests/test_chingmu_ball_tracker_roi.py`：

```python
import unittest

import numpy as np

from deploy.mocap_bridge.chingmu_table_lcm_bridge import (
    BallTracker,
    BridgeConfig,
    TableFrame,
)


class ChingMuBallTrackerRoiTest(unittest.TestCase):
    def setUp(self):
        self.config = BridgeConfig()
        self.table = TableFrame(
            center_raw_mm=np.zeros(3, dtype=np.float64),
            x_axis_raw=np.array([1.0, 0.0, 0.0], dtype=np.float64),
            y_axis_raw=np.array([0.0, 1.0, 0.0], dtype=np.float64),
            z_axis_raw=np.array([0.0, 0.0, 1.0], dtype=np.float64),
            corners_raw_mm=np.zeros((4, 3), dtype=np.float64),
            rectangle_score=0.0,
        )

    def raw_from_world(self, position_world):
        position = np.asarray(position_world, dtype=np.float64)
        return np.array(
            [
                (position[0] - 0.5 * self.config.table_length_m) * 1000.0,
                position[1] * 1000.0,
                (position[2] - self.config.table_height_m) * 1000.0,
            ],
            dtype=np.float64,
        )

    def test_far_edge_margin_cutoff_is_inclusive(self):
        cutoff_x = self.config.table_length_m - 0.40
        accepted = BallTracker().update(
            [self.raw_from_world([cutoff_x, 0.0, 0.90])],
            frame_number=1,
            table=self.table,
            config=self.config,
        )
        rejected = BallTracker().update(
            [self.raw_from_world([cutoff_x + 0.001, 0.0, 0.90])],
            frame_number=1,
            table=self.table,
            config=self.config,
        )

        self.assertIsNotNone(accepted.position_world)
        self.assertFalse(accepted.ended)
        self.assertIsNone(rejected.position_world)
        self.assertFalse(rejected.ended)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 运行测试并确认旧实现正确失败**

Run:

```bash
/home/loco1/miniconda3/envs/rb/bin/python -m unittest \
  deploy.mocap_bridge.tests.test_chingmu_ball_tracker_roi -v
```

Expected: `FAIL`，因为旧实现仍接受 `x = table_length_m - 0.399 m` 的候选球，`rejected.position_world` 不是 `None`。

- [ ] **Step 3: 写入最小生产实现**

在桥接模块常量区加入：

```python
BALL_TRACKING_FAR_EDGE_MARGIN_M = 0.40
```

在 `BallTracker.update()` 的候选遍历前计算：

```python
maximum_ball_x_m = (
    config.table_length_m - BALL_TRACKING_FAR_EDGE_MARGIN_M
)
```

将原过滤条件：

```python
if world[0] > config.table_length_m:
```

替换为：

```python
if world[0] > maximum_ball_x_m:
```

- [ ] **Step 4: 运行专用边界测试并确认通过**

Run:

```bash
/home/loco1/miniconda3/envs/rb/bin/python -m unittest \
  deploy.mocap_bridge.tests.test_chingmu_ball_tracker_roi -v
```

Expected: `Ran 1 test` 和 `OK`。

- [ ] **Step 5: 运行 Chingmu 相关回归测试**

Run:

```bash
/home/loco1/miniconda3/envs/rb/bin/python -m unittest \
  deploy.mocap_bridge.tests.test_chingmu_ball_tracker_roi \
  deploy.mocap_bridge.tests.test_chingmu_pelvis_orientation -v
```

Expected: 全部测试通过，无 traceback。

- [ ] **Step 6: 检查差异范围和 Python 语法**

Run:

```bash
/home/loco1/miniconda3/envs/rb/bin/python -m py_compile \
  deploy/mocap_bridge/chingmu_table_lcm_bridge.py \
  deploy/mocap_bridge/tests/test_chingmu_ball_tracker_roi.py
git diff --check -- \
  deploy/mocap_bridge/chingmu_table_lcm_bridge.py \
  deploy/mocap_bridge/tests/test_chingmu_ball_tracker_roi.py
```

Expected: 两条命令均退出码 `0`，没有输出错误。

- [ ] **Step 7: 仅提交本任务文件**

```bash
git add -- \
  deploy/mocap_bridge/chingmu_table_lcm_bridge.py \
  deploy/mocap_bridge/tests/test_chingmu_ball_tracker_roi.py \
  docs/superpowers/plans/2026-07-31-chingmu-ball-far-edge-margin.md
git diff --cached --check
git commit -m "fix: delay Chingmu ball admission by 40 cm"
```

提交前确认暂存区不包含当前 dirty worktree 的其他文件。

---

## Final Review Fix Wave

最终审查不扩大生产代码范围：不修改活动轨迹利用 `x <= 0` 点结束轨迹的既有
逻辑，只将本文中的 `0 < x <= cutoff` 明确为新轨迹 admission 和 valid ball
发布范围。

专用 ROI 测试持久覆盖：

- 远端留白命名常量严格等于 `0.40`；
- 使用非默认 `table_length_m`，并以 `1e-9 m` 的稳定 epsilon 验证运行时
  cutoff 派生和包含边界；
- `x = 0`、`x < 0` 拒绝 admission，`x > 0` 接受；
- `y = ±table_width_m / 2` 接受，越界拒绝；
- `z = table_height_m` 拒绝，`z > table_height_m` 接受；
- 活动轨迹只有超新远端上界候选时 `ended=True`，下一空帧不重复结束；
- 活动轨迹可使用 `x <= 0` 作为内部结束证据，但不会发布 valid ball。
