## Task 6: 复制生产 LCM/estimator 输入语义

**Files:**

- Modify: `deploy/diagnostics/hitter_task_pipeline.py`（首次创建）
- Create: `deploy/tests/test_hitter_task_input_adapter.py`

### Interface

```python
@dataclass(frozen=True)
class AdapterOutput:
    sample: NormalizedMocapSample
    snapshot: BallEstimateSnapshot | None
    warnings: tuple[str, ...]


class MocapFrameAdapter:
    def ingest_decoded(
        self,
        *,
        channel: str,
        message: object,
        payload_size: int,
        received_monotonic_s: float,
        wall_time_us: int,
    ) -> AdapterOutput:
        """Process one decoded transformation_t in exact arrival order."""

    def reset_estimator_after_strike(self) -> int:
        """Match RealWorld reset: clear estimator and advance track epoch."""
```

- 写 `input_seq` 严格按 `ball/pelvis/table` 实际到达顺序增长的测试。
- 写大小写无关 `ball` / `g1pelvis` / `table` 和 unknown subject 测试。
- 写 estimator timestamp 优先级测试：valid positive `vicon_time_s` → positive `publish_time_us` → configured 360 Hz nominal fallback；重复/倒退时间继续由 `BallStateEstimator` 的 `+1e-6` 规则处理。
- 写 ball snapshot 使用“该 ball 到达时 latest valid pelvis”的测试；同时记录 source frame/time delta warning，但 baseline `base_valid` 不因 mismatch 改变。
- 写固定 source/host clock 偏移测试：只能产生 `CLOCK_OFFSET_SUSPECTED`，不得把 `wall_time-source_time` 报成真实网络延迟。
- 写 invalid 测试：立即 estimator reset、epoch `+1`、仍生成 invisible snapshot；不得直接 reset incoming 或 lifecycle。
- 写 bounce、31 帧 sample count/ready、pelvis invalid、near-zero quaternion、nonfinite payload 测试。
- 运行失败测试：

  ```bash
  cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
  PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover -s tests -p 'test_hitter_task_input_adapter.py' -v
  ```

- 用 Task 2 factory 实现 adapter；LCM decode 与 `lcm.LCM.handle()` 留给 CLI owner thread，adapter 本身可纯测试。
- adapter 不创建/猜测 `attempt_id`；Task 7 的 pipeline 先把 sample 交给 `AttemptTracker`，再把 `(track_epoch,generation)` 组成 `SnapshotKey` 并绑定成 `AttemptBinding`。
- 与 `RealWorld._update_ball_state_from_vicon()` 的现有行为做表驱动 characterization 对照。
- 提交：

  ```bash
  git add deploy/diagnostics/hitter_task_pipeline.py deploy/tests/test_hitter_task_input_adapter.py
  git diff --cached --check
  git commit -m "feat: normalize HITTER mocap inputs"
  ```

