## Task 1: 锁定并共享 11 维 task observation

**Files:**

- Create: `deploy/utils/hitter_task_observation.py`
- Create: `deploy/tests/test_hitter_task_observation.py`
- Modify: `deploy/envs/hitter.py`

### Interface

实现以下稳定接口：

```python
@dataclass(frozen=True)
class TaskObservationResult:
    pre_clip: np.ndarray
    post_clip: np.ndarray
    clip_mask: np.ndarray
    clip_count: int
    errors: tuple[str, ...]


class TaskObservationAssemblyError(ValueError):
    reason_code: str


def assemble_active_hitter_task_observation(
    *,
    robot_anchor_position_w: np.ndarray,
    robot_anchor_quaternion_xyzw: np.ndarray,
    base_target_xy_w: np.ndarray,
    racket_target_position_w: np.ndarray,
    racket_target_velocity_w: np.ndarray,
    policy_time_to_strike_s: float,
    maximum_policy_time_to_strike_s: float,
    obs_clip_value: float | None,
) -> TaskObservationResult:
    """Return active HITTER task fields in full-observation indices 6:17."""
```

`pre_clip` 和 `post_clip` 都必须是只读 `float32`、shape `(11,)`，顺序固定为：

```text
base_forward_xy_w[2]
base_target_xy_b[2]
racket_target_pos_b[3]
racket_target_vel_w[3]
policy_tts[1]
```

错误 shape 时立即抛 `TaskObservationAssemblyError(reason_code="OBS_WRONG_SHAPE")`，不伪造 11 维数组。成功返回时 `errors` 只允许 `OBS_NONFINITE`、`OBS_CLIPPED`、`OBS_TTS_OUT_OF_RANGE`。生产 `HitterEnv` 取 `pre_clip` 拼入 104 维 observation，继续沿用末尾一次整向量 clip；诊断进程使用完整 result 判定 PASS。
`clip_count` 只统计 finite 且实际被裁剪的元素；NaN/Inf 只由 `OBS_NONFINITE` 报告，不能重复计入 clip。
`clip_mask` 固定为只读 `bool`、shape `(11,)`。

- 在测试中先写一个不依赖新 helper 的 `legacy_active_task_slice()`，逐项复制当前 `hitter.py` active branch 的公式。
- 添加 yaw 为 `0`、`+90°`、`-90°` 的表驱动测试，断言 world `base_forward`、base-yaw target 和 world racket velocity。
- 添加 `float32`、shape、只读数组、pre/post clip、nonfinite、TTS 的边界测试。
- 构造最小 `HitterEnv.__new__()`，通过 `refresh_policy_observation()` mock 两次 monotonic 返回值，断言 lifecycle update 使用第一次、`obs[6:17]` 的 TTS 使用第二次；单独调用 `_compute_hitter_observation()` 不能冒充“两次取时”测试。
- 运行测试并确认因模块/接口尚不存在而失败：

  ```bash
  cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
  PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover -s tests -p 'test_hitter_task_observation.py' -v
  ```

- 实现 helper；所有入参数组先复制，四元数按当前 `_hitter_robot_anchor_pose_w()` 规则归一化，yaw-only inverse 保持当前 SciPy 计算语义。
- 将 `HitterEnv._compute_hitter_observation()` 的 active task 计算机械替换为 helper；`policy_tts(now=time.monotonic())` 仍在调用 helper 前求值一次，禁止移动到 helper 内。
- 运行新增测试和当前日志测试：

  ```bash
  cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
  PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover -s tests -p 'test_hitter_task_observation.py' -v
  PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover -s tests -p 'test_hitter_strike_target_logging.py' -v
  ```

- 检查只改 active branch，waiting FK 分支和 104 维拼接顺序未改变。
- 仅暂存本任务文件；`hitter.py` 使用交互式 hunk：

  ```bash
  git add deploy/utils/hitter_task_observation.py deploy/tests/test_hitter_task_observation.py
  git add -p deploy/envs/hitter.py
  git diff --cached --check
  git commit -m "refactor: share HITTER task observation assembly"
  ```

