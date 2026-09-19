### Task 1: 收紧 BallTracker 远端 X 边界

**Files:**
- Create: `deploy/mocap_bridge/tests/test_chingmu_ball_tracker_roi.py`
- Modify: `deploy/mocap_bridge/chingmu_table_lcm_bridge.py:54-63`
- Modify: `deploy/mocap_bridge/chingmu_table_lcm_bridge.py:859-875`

**Interfaces:**
- Consumes: `BridgeConfig.table_length_m: float`、`BallTracker.update(...) -> BallTrackUpdate`。
- Produces: `BALL_TRACKING_FAR_EDGE_MARGIN_M: float = 0.40`，以及包含边界 `x <= table_length_m - BALL_TRACKING_FAR_EDGE_MARGIN_M` 的球候选过滤行为。

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
