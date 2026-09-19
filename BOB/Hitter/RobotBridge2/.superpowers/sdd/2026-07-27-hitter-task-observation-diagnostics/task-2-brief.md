## Task 2: 共享 estimator / planner 配置构造

**Files:**

- Create: `deploy/utils/hitter_runtime_factory.py`
- Create: `deploy/tests/test_hitter_runtime_factory.py`
- Modify: `deploy/envs/hitter.py`
- Modify: `deploy/simulator/real_world.py`

### Interface

```python
def build_ball_state_estimator(
    planner_config: Mapping[str, object],
) -> BallStateEstimator:
    """Build the exact estimator currently used by RealWorld."""


def build_hitter_system_planner(
    planner_config: Mapping[str, object],
) -> HitterSystemPlanner:
    """Build the exact planner currently used by HitterEnv."""


def forced_strike_type(
    planner_config: Mapping[str, object],
    motion_config: Mapping[str, object],
) -> str | None:
    """Resolve and validate forehand/backhand/None exactly once."""


@dataclass(frozen=True)
class HitterRuntimeSettings:
    estimator_sample_rate_hz: float
    planner_update_rate_hz: float
    planner_update_interval_s: float
    minimum_incoming_speed_x_mps: float
    incoming_confirmation_snapshots: int
    waiting_tts_s: float
    arm_tts_s: float
    minimum_arm_tts_s: float
    maximum_policy_tts_s: float
    swing_duration_range_s: tuple[float, float]
    hitter_seed: int | None
    control_tick_s: float
    obs_clip_value: float | None


def resolve_hitter_runtime_settings(
    *,
    policy_config: Mapping[str, object],
    motion_config: Mapping[str, object],
    control_config: Mapping[str, object],
) -> HitterRuntimeSettings:
    """Resolve every estimator/incoming/lifecycle/tick value once."""


def build_hitter_command_lifecycle(
    settings: HitterRuntimeSettings,
    *,
    rng: np.random.Generator,
) -> HitterCommandLifecycle:
    """Use the seeded production-uniform swing-duration sampler."""
```

- 在测试中保留当前生产构造代码作为 reference builder，加载 `config/mimic/hitter.yaml`，逐字段比较 estimator、predictor、strike planner 和 base planner。
- 对照 production reference 覆盖 waiting `0.92`、arm `0.92`、minimum arm `0.60`、maximum policy `0.92`、swing `[1.75,1.95]`、seed `0`、planner `100 Hz`/`0.01 s`、control `0.005*4=0.02 s`、incoming `3`/`0.20 m/s`、estimator `360 Hz`。
- 用两个相同 seed 的 RNG 连续采样 recovery duration，断言 factory lifecycle 与现有 `_new_hitter_command_lifecycle()` 序列一致。
- 覆盖空配置默认值、`maximum_hit_height` 的 12 位 round、invalid forced strike type。
- 先运行并确认新 factory 测试失败：

  ```bash
  cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
  PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover -s tests -p 'test_hitter_runtime_factory.py' -v
  ```

- 实现纯 factory；不得读取环境变量、时钟或文件。
- 将 `HitterEnv._build_hitter_ball_planner()`、`_forced_strike_type_name()`、incoming 初始化和 `_new_hitter_command_lifecycle()` 改为委托 shared resolver/factory；保持同一个 `self.hitter_rng` 的调用顺序。
- 将 `RealWorld._init_ball_state()` 的 `BallStateEstimator(...)` 构造改为委托 factory，保留现有 sample-rate 和状态字段初始化。
- 运行：

  ```bash
  cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
  PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover -s tests -p 'test_hitter_runtime_factory.py' -v
  PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover -s tests -p 'test_real_world_connection_wait.py' -v
  PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover -s tests -p 'test_hitter_strike_target_logging.py' -v
  ```

- 对三个 dirty 生产文件逐 hunk 暂存：

  ```bash
  git add deploy/utils/hitter_runtime_factory.py deploy/tests/test_hitter_runtime_factory.py
  git add -p deploy/envs/hitter.py deploy/simulator/real_world.py
  git diff --cached --check
  git commit -m "refactor: share HITTER runtime factories"
  ```

