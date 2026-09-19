## Task 3: 增加独立于 pelvis 的球诊断状态

**Files:**

- Modify: `deploy/diagnostics/hitter_task_models.py`
- Modify: `deploy/diagnostics/hitter_task_pipeline.py`
- Test: `deploy/tests/test_hitter_task_pipeline.py`

- [ ] **Step 1: 为结构化球状态写失败测试**

覆盖：

- 没有 pelvis 时 estimator 仍从 `1` 累加到 window size。
- 满足 `vx <= incoming_speed_threshold` 的唯一球样本使诊断 incoming 从 `1/3` 到 `3/3`。
- 重复处理同一个 `(track_epoch, generation)` 不重复计数。
- 非 incoming 样本重置连续计数。
- 球不可见或 track 结束时状态为明确原因，不伪造零。
- 生产 `incoming_count` 不被诊断计数修改。

- [ ] **Step 2: 运行 pipeline 测试确认失败**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
/home/loco1/miniconda3/envs/rb/bin/python -m unittest \
  tests.test_hitter_task_pipeline
```

- [ ] **Step 3: 定义 v2 结构化状态**

在 models 中增加：

```python
@dataclass(frozen=True)
class BallDiagnosticState:
    status: str
    estimator_sample_count: Optional[int]
    estimator_window_size: Optional[int]
    speed_mps: Optional[float]
    velocity_world_mps: Optional[Tuple[float, float, float]]
    incoming_count: Optional[int]
    incoming_required_count: Optional[int]
    incoming_status: str
    blocker: Optional[str]
```

`status`/`incoming_status` 使用显式枚举字符串，例如 `NOT_SEEN`、`ESTIMATING`、`READY`、`INCOMING`、`NOT_INCOMING`、`TRACK_ENDED`。

- [ ] **Step 4: 实现 BallDiagnosticTracker**

Tracker 只读取球 estimator snapshot；以 `(track_epoch, generation)` 去重，并使用现有 planner 配置的 incoming 速度阈值与 required count。它不读取 pelvis，不调用生产 planner，不产生 action。

- [ ] **Step 5: 运行 Task 3 测试并提交**

```bash
git add deploy/diagnostics/hitter_task_models.py \
  deploy/diagnostics/hitter_task_pipeline.py \
  deploy/tests/test_hitter_task_pipeline.py
git commit -m "feat: add pelvis-independent ball diagnostics"
```

---

