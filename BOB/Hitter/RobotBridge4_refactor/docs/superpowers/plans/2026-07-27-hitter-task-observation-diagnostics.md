# HITTER 真机任务 Observation 逐球诊断 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 新增一个只读、旁路、可在浏览器查看进度的逐球诊断进程；它从 ChingMu 的 `ball` 与 `G2Pelvis` LCM 消息开始，严格复刻当前 estimator、100 Hz latest-only planner、连续 3 次 incoming 与 50 Hz lifecycle 的时序语义，最终判断 shadow 链路是否形成 11 维 HITTER task observation，并对同一份输入离线比较 `3/100` 与 `1/100`。

**Architecture:** 诊断进程拥有自己的 LCM 接收线程、生产等价的 estimator、instrumented latest-only planner worker、50 Hz lifecycle/task-observation tick、不可变事件总线、后台记录器、空闲期 replay 子进程以及 localhost HTTP/SSE 页面。生产 `HitterEnv` 和诊断进程共享 task-observation、estimator 与 planner 构造函数；诊断进程不实例化 `RealWorld`、`HitterAgent`、ONNX session 或任何控制发布者。

**Tech Stack:** Python 3.8（当前 `rb` 环境）、NumPy、SciPy、OmegaConf、LCM、Python 标准库 `unittest` / `threading` / `multiprocessing` / `http.server` / `http.client` / `json` / `csv`，原生 HTML/CSS/JavaScript。

## Global Constraints

- 当前工作树包含大量用户自有修改、删除和未跟踪文件；禁止 reset、restore 或清理。
- 行为规范以 `docs/superpowers/specs/2026-07-27-hitter-task-observation-diagnostics-design.md` 为 canonical source；本计划中的命名统一使用 `TRACK_ENDED_BEFORE_CONFIRMATION`。
- 不恢复已删除的旧 monitor、recorder 或测试文件。
- 对已经是 dirty 的现有文件使用 `git add -p`，只暂存本任务 hunk；新文件使用显式路径 `git add`。禁止 `git add .` 和 `git add -A`。
- 所有测试从 `deploy/` 目录运行；当前 `rb` 环境没有 pytest，统一使用：

  ```bash
  cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
  PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover -s tests -p 'TEST_FILE.py' -v
  ```

- 所有新增 Python 模块第一行必须是 `from __future__ import annotations`；Task 12 必须 `py_compile` 并逐模块 import，防止 Python 3.8 在 `X | None`、`tuple[T, ...]` 处启动失败。
- 单元和集成测试使用 fake clock / scripted planner duration；只有显式性能验收使用真实线程和真实耗时。
- 在线基线固定为 incoming `3`、planner `100 Hz`、lifecycle/task tick `50 Hz`。第一阶段禁止实时运行完整 `360 Hz` planner。
- invalid ball 的生产语义必须保持：

  ```text
  LCM owner 线程立即 reset estimator 并推进 epoch
  → 形成 invisible snapshot
  → 仍经过 100 Hz throttle
  → worker 的 plan_fn 看到 invisible 后 reset incoming
  → 50 Hz tick 实际消费 TRACK_ENDED result 后再通知 lifecycle
  ```

  展示层、heartbeat 和 reacquire grace 不得提前 reset incoming 或 lifecycle。

- 一个 50 Hz tick 必须先读取 `lifecycle_now_s`，推进 lifecycle/消费结果；再读取 `obs_now_s`，复制 latest pelvis 并计算 task observation。共享 helper 内部不得读取时钟。
- 页面成功文案固定为 `SHADOW 3/100 TASK OBS 11/11 PASS`；不得声称生产 policy、ONNX 或完整 104 维 observation 已消费/通过。
- 诊断代码不得 import 或实例化 `HitterAgent` / `onnxruntime.InferenceSession`，不得发布 `pd_plustau_targets`、action 或任何控制频道。

## File Map

### 新增生产共享代码

- `deploy/utils/hitter_task_observation.py`
  - 纯函数组装并验证 active-command 的 11 维 task observation。
- `deploy/utils/hitter_runtime_factory.py`
  - 从现有 `ball_planner` 配置构造 `BallStateEstimator` 与 `HitterSystemPlanner`，供生产和诊断共同使用。

### 新增诊断代码

- `deploy/diagnostics/__init__.py`
- `deploy/diagnostics/hitter_task_models.py`
  - 跨线程、跨进程的不可变数据契约和 JSON 编解码。
- `deploy/diagnostics/hitter_task_attempts.py`
  - `attempt_id` / `track_segment_id`、reacquire grace、primary blocker。
- `deploy/diagnostics/hitter_task_events.py`
  - 原子 `publish()`、immutable state reducer、有界 ring、独立 subscriber cursor。
- `deploy/diagnostics/hitter_task_recording.py`
  - raw lane、会话目录、CSV/JSONL/detail 持久化和公平 drain。
- `deploy/diagnostics/hitter_task_pipeline.py`
  - LCM 规范化、estimator、100 Hz worker、50 Hz lifecycle 和 11 维 task tick。
- `deploy/diagnostics/hitter_task_replay.py`
  - 离散事件 `3/100` baseline replay、`1/100` 反事实和低优先级子进程。
- `deploy/diagnostics/hitter_task_web.py`
  - localhost HTTP/SSE 只读服务。
- `deploy/diagnostics/hitter_task_monitor.py`
  - CLI、组件装配和有界 shutdown。
- `deploy/diagnostics/static/hitter_task_monitor.html`
  - 无构建步骤的只读前端。

### 修改现有生产代码

- `deploy/envs/hitter.py`
  - 使用共享 planner factory 和 task-observation helper；保持现有时钟顺序与数值。
- `deploy/simulator/real_world.py`
  - 使用共享 estimator factory；行为不变。
- `deploy/utils/hitter_realtime.py`
  - 为 latest-only worker 增加可选、默认关闭的只读 trace hook；为 incoming 增加只读状态快照。
- `.gitignore`
  - 只增加 `recordings/hitter_task_diagnostics/`。

### 新增测试与文档

- `deploy/tests/test_hitter_task_observation.py`
- `deploy/tests/test_hitter_runtime_factory.py`
- `deploy/tests/test_hitter_task_attempts.py`
- `deploy/tests/test_hitter_task_events.py`
- `deploy/tests/test_hitter_task_recording.py`
- `deploy/tests/test_hitter_task_input_adapter.py`
- `deploy/tests/test_hitter_task_pipeline.py`
- `deploy/tests/test_hitter_task_replay.py`
- `deploy/tests/test_hitter_task_replay_process.py`
- `deploy/tests/test_hitter_task_web.py`
- `deploy/tests/test_hitter_task_frontend.py`
- `deploy/tests/test_hitter_task_monitor.py`
- `deploy/tests/test_hitter_task_diagnostics_integration.py`
- `deploy/tests/test_hitter_task_diagnostics_safety.py`
- `deploy/tests/test_hitter_task_diagnostics_performance.py`
- `docs/hitter_task_observation_diagnostics.md`

---

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

## Task 3: 定义不可变诊断协议和逐球身份状态机

**Files:**

- Create: `deploy/diagnostics/__init__.py`
- Create: `deploy/diagnostics/hitter_task_models.py`
- Create: `deploy/diagnostics/hitter_task_attempts.py`
- Create: `deploy/tests/test_hitter_task_attempts.py`

### Core contracts

`hitter_task_models.py` 至少定义：

```python
@dataclass(frozen=True)
class NormalizedMocapSample:
    input_seq: int
    channel: str
    subject: str
    position_w: np.ndarray
    quaternion_xyzw: np.ndarray
    valid: bool
    occluded: bool
    source_frame: int
    source_time_s: float | None
    publish_time_us: int | None
    received_monotonic_s: float
    wall_time_us: int
    payload_size: int


JsonScalar = Union[None, bool, int, float, str]
JsonValue = Union[
    JsonScalar,
    Tuple["JsonValue", ...],
    Mapping[str, "JsonValue"],
]


@dataclass(frozen=True, order=True)
class SnapshotKey:
    track_epoch: int
    generation: int


@dataclass(frozen=True)
class AttemptBinding:
    attempt_id: int
    track_segment_id: int
    role: str
    snapshot_key: SnapshotKey


@dataclass(frozen=True)
class AttemptTransition:
    attempt_id: int
    track_segment_id: int | None
    stage: str
    monotonic_s: float
    snapshot_key: SnapshotKey | None
    reason_code: str | None
    values: Mapping[str, JsonValue]


@dataclass(frozen=True)
class EventDraft:
    kind: str
    monotonic_s: float
    wall_time_us: int
    scope: str
    attempt_id: int | None
    payload: Mapping[str, JsonValue]


@dataclass(frozen=True)
class HealthSnapshot:
    lcm_connected: bool
    message_rate_hz_by_subject: Mapping[str, float]
    message_age_s_by_subject: Mapping[str, float | None]
    source_frame_by_subject: Mapping[str, int | None]
    pelvis_valid: bool
    pelvis_age_s: float | None
    planner_submitted: int
    planner_completed: int
    planner_failed: int
    planner_dropped_pending: int
    planner_results_overwritten_before_consume: int
    raw_samples_dropped: int
    recorder_event_gaps: int
    diagnostic_events_dropped: int
    recorder_healthy: bool
    recording_complete: bool
    config_name: str
    session_basename: str
    warnings: tuple[str, ...]


@dataclass(frozen=True)
class LifecycleSnapshot:
    phase: str
    last_decision: str
    active_key: SnapshotKey | None
    cached_key: SnapshotKey | None
    lifecycle_now_s: float | None
    obs_now_s: float | None


@dataclass(frozen=True)
class AttemptSummary:
    attempt_id: int
    status: str
    stage: str
    primary_blocker: str | None
    ball_speed_mps: float | None
    predicted_strike_time_s: float | None
    planner_tts_s: float | None
    arm_tts_s: float | None
    task_obs_status: str
    ab_summary: str | None
    recording_complete: bool


@dataclass(frozen=True)
class AttemptDetail:
    attempt_id: int
    summary: AttemptSummary
    segments: tuple[Mapping[str, JsonValue], ...]
    stage_timeline: tuple[Mapping[str, JsonValue], ...]
    planner_inputs: tuple[Mapping[str, JsonValue], ...]
    planner_results: tuple[Mapping[str, JsonValue], ...]
    task_observation_pre_clip: tuple[float, ...] | None
    task_observation_post_clip: tuple[float, ...] | None
    task_observation_clip_count: int | None
    variant_outcomes: tuple[Mapping[str, JsonValue], ...]
    ab_deltas: Mapping[str, JsonValue]


@dataclass(frozen=True)
class AttemptPage:
    items: tuple[AttemptSummary, ...]
    has_more: bool
    next_before: int | None
```

所有 ndarray 在 `__post_init__` 中复制并设为只读；跨进程/落盘通过固定 `schema_version=1` 的显式 `to_json_dict()`，禁止 pickle 任意对象作为持久化格式。
所有标准化 mocap position/quaternion 必须保存为只读 `float64`，CSV/JSON 使用足以往返 float64 的表示。上述字段名、`_s`/`_mps` 单位后缀和 `None → JSON null` 是 HTTP/前端/落盘共同契约；`to_json_dict()` 必须逐字段显式输出，禁止直接暴露 `__dict__` 或绝对路径。

`hitter_task_attempts.py` 提供：

```python
class AttemptTracker:
    def observe_ball_sample(
        self,
        sample: NormalizedMocapSample,
        *,
        snapshot_key: SnapshotKey,
    ) -> tuple[AttemptTransition, ...]:
        """Update display grouping only; never mutate estimator/incoming/lifecycle."""

    def observe_production_reset(
        self,
        *,
        previous_track_epoch: int,
        new_track_epoch: int,
        reason: str,
        monotonic_s: float,
    ) -> tuple[AttemptTransition, ...]:
        """End the production segment on invalid or strike-deadline reset."""

    def bind_snapshot(
        self,
        *,
        snapshot_key: SnapshotKey,
    ) -> AttemptBinding | None:
        """Bind async planner identity without guessing from current visibility."""

    def binding_for_result(
        self,
        snapshot_key: SnapshotKey,
    ) -> AttemptBinding | None:
        """Look up immutable async-result ownership."""

    def record_stage(
        self,
        *,
        binding: AttemptBinding,
        stage: str,
        monotonic_s: float,
        reason_code: str | None,
        values: Mapping[str, JsonValue],
    ) -> AttemptTransition:
        """Append a stage/rejection without changing production state."""

    def record_task_pass(
        self,
        *,
        binding: AttemptBinding,
        monotonic_s: float,
    ) -> AttemptTransition:
        """Mark first task PASS and make later reacquisition a post-deadline tail."""

    def record_health_warning(
        self,
        *,
        reason_code: str,
        monotonic_s: float,
        values: Mapping[str, JsonValue],
    ) -> AttemptTransition | None:
        """Record warning only; never open/close attempts."""

    def set_lifecycle_context(
        self,
        *,
        phase: str,
        active_key: SnapshotKey | None,
        cached_key: SnapshotKey | None,
        monotonic_s: float,
    ) -> tuple[AttemptTransition, ...]:
        """Project recovery/cached status onto the owning attempt."""

    def advance(
        self,
        *,
        now_monotonic_s: float,
    ) -> tuple[AttemptTransition, ...]:
        """Close expired reacquire grace without changing production state."""
```

- 先写测试：visible 上升沿创建 attempt/segment；global `attempt_id` 和 `track_segment_id` 单调递增。
- 写 invalid 已推进 epoch 的测试：旧 segment 关闭，新 invisible snapshot 仍绑定正确 attempt，tracker 不调用任何 production reset。
- 将边界例固定为：visible `(epoch=5,generation=31)` 属于 attempt 1 / segment 1；invalid 后的 invisible `(6,32)` 仍绑定 attempt 1 / segment 1；grace 内 visible `(6,33)` 属于 attempt 1 / segment 2。
- 写 `0.20 s` grace 内重捕获同 attempt 新 segment、grace 外新 attempt、PASS 后 `POST_DEADLINE_TAIL`。
- grace 边界使用闭区间：`now <= invalid_time + 0.20` 仍归原 attempt，只有严格大于 deadline 才关闭。
- 写 heartbeat/pelvis stale 只产生 warning、不关闭 attempt、不改变 segment 的测试。
- 写 recovery 中 `WAITING_FOR_PREVIOUS_RECOVERY` / `CACHED_DURING_RECOVERY` 的展示映射测试。
- 写 `(track_epoch, generation)` 绑定和迟到异步结果归属测试。
- 写 attempt close 的 primary blocker 表驱动测试：任一 segment PASS 则成功；否则按 never-ready → never-confirmed → no-valid-plan/last-planner-reason → late/before-arm → explicit obs failure 的最远阶段顺序选择，较早 warning 只留在 timeline。
- 运行并确认失败：

  ```bash
  cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
  PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover -s tests -p 'test_hitter_task_attempts.py' -v
  ```

- 实现 immutable model、deep JSON freeze/copy 和纯展示 tracker；tracker 中禁止 import planner/LCM。
- 运行测试，额外检查源码中不存在展示层直接调用 `.reset()` 或 `.mark_track_ended()`。
- 提交：

  ```bash
  git add deploy/diagnostics/__init__.py \
    deploy/diagnostics/hitter_task_models.py \
    deploy/diagnostics/hitter_task_attempts.py \
    deploy/tests/test_hitter_task_attempts.py
  git diff --cached --check
  git commit -m "feat: model HITTER diagnostic attempts"
  ```

## Task 4: 实现原子 StateStore / EventHub

**Files:**

- Create: `deploy/diagnostics/hitter_task_events.py`
- Create: `deploy/tests/test_hitter_task_events.py`

### Interface

```python
@dataclass(frozen=True)
class PublishedEvent:
    schema_version: int
    event_id: int
    kind: str
    monotonic_s: float
    wall_time_us: int
    scope: str
    attempt_id: int | None
    payload: Mapping[str, JsonValue]


@dataclass(frozen=True)
class CursorBootstrap:
    mode: str
    watermark_event_id: int


@dataclass(frozen=True)
class EventRead:
    events: tuple[PublishedEvent, ...]
    watermark_event_id: int
    lost_event_ids: tuple[int, int] | None


class EventCursor:
    @property
    def bootstrap(self) -> CursorBootstrap:
        """Return fresh, resume, or reset plus current watermark."""

    def read(
        self,
        *,
        limit: int = 128,
        timeout_s: float | None = None,
    ) -> EventRead:
        """Read this cursor only; a ring gap is explicit in lost_event_ids."""

    def close(self) -> None:
        """Release subscription state and wake any blocked read."""


class EventHub:
    def __init__(
        self,
        initial_state: DiagnosticState,
        *,
        capacity: int = 8192,
        max_event_bytes: int = 65536,
    ) -> None:
        """Own one bounded immutable ring and one atomic state reference."""

    def publish(self, draft: EventDraft) -> PublishedEvent:
        """Allocate id, reduce immutable state, and append ring under one lock."""

    def state_snapshot(self) -> DiagnosticState:
        """Return the current immutable state by atomic reference."""

    def open_cursor(self, after_event_id: int | None) -> EventCursor:
        """Create an independent cursor; consumers never compete on queue.get()."""

    def close(self) -> None:
        """Wake and close all cursors without blocking a producer."""
```

`DiagnosticState` 的 reader-facing 结构固定为：

```python
@dataclass(frozen=True)
class DiagnosticState:
    schema_version: int
    watermark_event_id: int
    health: HealthSnapshot
    lifecycle: LifecycleSnapshot
    current_attempt: AttemptSummary | None
    recent_attempts: tuple[AttemptSummary, ...]
```

默认 ring 容量 `8192`，内存 attempt detail cache `100`，单 event 序列化上限 `64 KiB`，异常文本截断到 `2048` 字符。

- 写 8 个 publisher 线程的测试：event id 必须唯一、连续，state watermark 必须等于 ring 尾。
- 写两个独立 cursor 读取同一事件的测试，证明 recorder 和 SSE 不竞争消费。
- 写落后 cursor 的 ring-gap reset 测试。
- 写 event 发布后不可修改、嵌套 payload 不可变、超大 event 被拒绝为结构化 diagnostics error 的测试。
- 写 progress producer mailbox/rate-gate 测试：只允许在调用 `publish()` 之前合并；已有 event id 的事件永不原位改写。
- 运行失败测试：

  ```bash
  cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
  PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover -s tests -p 'test_hitter_task_events.py' -v
  ```

- 实现纯 reducer、原子 publish、condition-based cursor wait 和关闭唤醒；任何 cursor 停滞不得阻塞 publish。
- 用 fake clock 测 heartbeat/invalidation，不在 unit test 中 `sleep()`。
- 提交：

  ```bash
  git add deploy/diagnostics/hitter_task_events.py deploy/tests/test_hitter_task_events.py
  git diff --cached --check
  git commit -m "feat: add HITTER diagnostic event hub"
  ```

## Task 5: 实现有界 raw lane 与持久化记录器

**Files:**

- Create: `deploy/diagnostics/hitter_task_recording.py`
- Create: `deploy/tests/test_hitter_task_recording.py`
- Modify: `.gitignore`

### Interface

```python
@dataclass(frozen=True)
class SessionPaths:
    root: Path
    session_json: Path
    ball_samples_csv: Path
    events_jsonl: Path
    replay_jobs_jsonl: Path
    replay_analysis_jsonl: Path
    attempts_csv: Path
    attempt_details_dir: Path


@dataclass(frozen=True)
class RawSubmitResult:
    accepted: bool
    dropped_input_seq: tuple[int, int] | None
    attempt_id: int | None


@dataclass(frozen=True)
class RecorderStatus:
    healthy: bool
    recording_complete: bool
    bytes_written: int
    last_error: str | None


class RawRecordLane:
    def __init__(self, capacity: int = 65536) -> None:
        """Create a bounded queue with no producer-side blocking."""

    def submit(self, sample: NormalizedMocapSample) -> RawSubmitResult:
        """Non-blocking put; report exact lost input_seq ranges on overflow."""

    def take_many(
        self,
        *,
        limit: int,
        timeout_s: float = 0.0,
    ) -> tuple[NormalizedMocapSample, ...]:
        """Take up to limit records for the recorder only."""

    def close(self) -> None:
        """Reject new records and wake the recorder."""


class AttemptDetailRepository:
    def commit(self, detail: AttemptDetail) -> None:
        """Atomically replace a numeric detail file and refresh the LRU cache."""

    def get(self, attempt_id: int) -> AttemptDetail | None:
        """Read cache first, then only the fixed <positive-id>.json path."""

    def list_page(
        self,
        *,
        limit: int = 50,
        before: int | None = None,
    ) -> AttemptPage:
        """Return attempt_id-desc, exclusive-cursor pagination."""


class SessionRecorder:
    def __init__(
        self,
        *,
        paths: SessionPaths,
        raw_lane: RawRecordLane,
        event_cursor: EventCursor,
        attempt_repository: AttemptDetailRepository,
        raw_quota: int = 512,
        event_quota: int = 128,
        flush_interval_s: float = 1.0,
        flush_row_count: int = 1000,
    ) -> None:
        """Bind the two independent input lanes and all output files."""

    def start(self) -> None:
        """Open exclusive 0600 files inside an exclusive 0700 session dir."""

    def offer_attempt_detail(self, detail: AttemptDetail) -> bool:
        """Queue one immutable detail/summary update."""

    def offer_replay_analysis(
        self,
        analysis: Mapping[str, JsonValue],
    ) -> bool:
        """Queue a parent-received replay result; child never writes files."""

    def request_stop(self) -> None:
        """Stop accepting new work while retaining queued records."""

    def drain(self, timeout_s: float) -> RecorderStatus:
        """Drain raw<=512/event<=128 batches; keep session.json open."""

    def write_terminal_and_close(
        self,
        *,
        terminal_session: Mapping[str, JsonValue],
        timeout_s: float,
    ) -> RecorderStatus:
        """Write terminal metadata, fsync/flush, close files and cursor."""

    def status(self) -> RecorderStatus:
        """Return an immutable health snapshot."""
```

`SessionPaths.root` 采用
`YYYYMMDD_HHMMSS_ffffff-p<PID>-<short_uuid>`，exclusive create 后目录 0700、文件 0600。

- 使用 `tempfile.TemporaryDirectory()` 写目录名格式、exclusive create、0700/0600 权限测试。
- 写 raw queue `65536` 默认、有界 nonblocking、连续 drop 合并成 `[first_input_seq,last_input_seq]` 的测试。
- 写 raw drop 通过 EventHub 报告但不写回 raw lane、相关 attempt/session 标记 `RECORDING_INCOMPLETE`、A/B 禁用测试。
- 写 recorder 每轮 raw 512 / event 128 公平 drain 测试，任一路高压不能饿死另一路。
- 写 recorder event cursor 落后并被 ring 覆盖的测试：session/受影响 attempt 标记 `RECORDING_INCOMPLETE`，A/B 禁用；不得阻塞 EventHub publisher。
- 写 `session.json`、`ball_samples.csv`、`events.jsonl`、`replay_jobs.jsonl`、`replay_analysis.jsonl`、`attempts.csv` 和 numeric `attempt_details/<id>.json` schema 测试。
- 写 attempt 正常时只在 A/B 完成后落一行 `attempts.csv`；退出时 pending attempt 落 `INCONCLUSIVE`，不能留下无终态半行。
- 写每 1 秒或 1000 行 flush、attempt/session terminal 立即 flush、磁盘异常降级不抛到 realtime producer 的测试。
- 运行失败测试：

  ```bash
  cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
  PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover -s tests -p 'test_hitter_task_recording.py' -v
  ```

- 实现 recorder；detail 查询只能接受正整数 id，先内存 cache，再拼接固定 `<id>.json`，禁止用户输入路径。
- 在 `.gitignore` 只添加一行：

  ```text
  recordings/hitter_task_diagnostics/
  ```

- 提交；`.gitignore` 使用 hunk staging：

  ```bash
  git add deploy/diagnostics/hitter_task_recording.py deploy/tests/test_hitter_task_recording.py
  git add -p .gitignore
  git diff --cached --check
  git commit -m "feat: record HITTER diagnostic sessions"
  ```

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

## Task 7: 实现 100 Hz worker、50 Hz lifecycle 与 shadow task tick

**Files:**

- Modify: `deploy/utils/hitter_realtime.py`
- Modify: `deploy/diagnostics/hitter_task_pipeline.py`
- Create: `deploy/tests/test_hitter_task_pipeline.py`

### Worker trace extension

在不改变默认行为的前提下增加：

```python
@dataclass(frozen=True)
class FrozenPlannerResult:
    snapshot_key: SnapshotKey
    source_frame: int
    strike_deadline_monotonic_s: float
    completed_monotonic_s: float
    command_fields: Mapping[str, JsonValue] | None
    error_type: str | None
    error_text: str | None


@dataclass(frozen=True)
class PlannerWorkerTrace:
    kind: str
    monotonic_s: float
    snapshot: BallEstimateSnapshot
    result: FrozenPlannerResult | None
    replaced_snapshot: BallEstimateSnapshot | None
    replaced_result: FrozenPlannerResult | None
    stats: PlannerWorkerStats
```

`LatestOnlyPlannerWorker(..., trace_listener=None)` 仅在 listener 非空时复制并发送 `submit`、`pending_replaced`、`start`、`complete`、`latest_result_replaced`；command 必须在 worker 内转为深度不可变 `FrozenPlannerResult.command_fields`，不得把可变 command 对象交给诊断线程。listener 必须在 worker condition lock 外调用，listener 异常只能增加 trace-listener failure 计数，不能改变 planner result/stats。`close(timeout_s=None) -> bool` 保持生产默认等待行为；诊断传有限 timeout 并检查返回值。`IncomingTrackConfirmation` 新增只读 `snapshot()`，返回 epoch/count/confirmed，不提供新的 mutation API。

### Pipeline interface

```python
@dataclass(frozen=True)
class TaskTickResult:
    lifecycle_now_s: float
    obs_now_s: float
    phase: str
    lifecycle_decision: str
    active_binding: AttemptBinding | None
    cached_binding: AttemptBinding | None
    command_result: FrozenPlannerResult | None
    command_fields_used: Mapping[str, JsonValue] | None
    task_observation: TaskObservationResult | None
    task_pass: bool
    errors: tuple[str, ...]


class ShadowTaskPipeline:
    def __init__(
        self,
        *,
        adapter: MocapFrameAdapter,
        settings: HitterRuntimeSettings,
        event_sink: Callable[[EventDraft], None],
        raw_sink: Callable[[NormalizedMocapSample], RawSubmitResult],
    ) -> None:
        """Own incoming, planner worker, lifecycle and production-reset callback."""

    def ingest_adapter_output(self, output: AdapterOutput) -> None:
        """Raw record, attempt bind, 100 Hz throttle, then worker submit."""

    def tick(
        self,
        *,
        lifecycle_now_s: float,
        obs_now_s: float,
        wall_time_us: int,
    ) -> TaskTickResult:
        """Advance, reset on strike crossing, consume, then assemble task obs."""
```

- 写 worker trace 回归测试：无 listener 时旧 stats/latest-only 行为逐项不变；listener 时 pending/result 覆盖身份准确。
- 写 listener 抛异常不影响 worker、nested command freeze 后不可修改、诊断 `close(timeout_s=5.0)` 有界返回的测试。
- 写 100 Hz throttle 测试：基于 snapshot `received_monotonic_s`，不是 wall/source time。
- 100 Hz 必须复制生产的事件驱动判断：`received_time - last_submit >= 0.01 - 1e-12`，不得实现成独立周期 timer。
- 写 incoming 的 exact production tests：not-ready/base-invalid 在 `observe()` 前拒绝且保留已有 count；non-incoming 调用 `observe()` 后 count 清零；confirmed latch；invisible 只有真正进入 plan_fn 才 reset。
- 写 reason mapper 测试，保留异常原始 type/text/value，并映射到固定 estimator/incoming/planner reason code。
- 写 50 Hz tick 顺序测试：先 `lifecycle_now_s` advance/consume，再 `obs_now_s` policy_tts/latest pelvis；两个时间值不得合并。
- 精确顺序测试固定为：`lifecycle.advance → 若 ARMED crossing 则同步 adapter.reset_estimator_after_strike()/epoch++ 并通知 AttemptTracker production reset → 读取/消费 latest planner result → 捕获 obs_now/latest pelvis → helper`。
- 写当前/latest pelvis 与旧 planner snapshot pelvis 不同的测试，task base-forward/target 必须使用 tick 时 pelvis。
- 写 lifecycle `TRACKING/RECOVERY` 不算首次 PASS、只有 `ARMED` + identity 一致 + 11 finite + clip_count 0 才 PASS。
- 写 `OBS_COMMAND_MISMATCH`：`TaskTickResult.command_result`、`command_fields_used`、active binding、base/racket/TTS 必须来自同一 tick 的 active command，任一字段或 source key 不一致即失败。
- 写 invalid invisible snapshot 可能被 throttle/pending/result 覆盖的测试，证明 pipeline 不提前通知 lifecycle。
- 写 strike deadline 后 estimator reset/epoch advance、RECOVERY/cached next attempt、stale generation 和 result overwrite 测试。
- 每个 online attempt 必须通过 immutable events 完整记录 Task 8 所需事实：raw input_seq、submit/start/complete/replacement、真实 planner duration、50 Hz lifecycle_now/obs_now、decision/phase、active/cached key、实际 recovery duration、submission phase anchor、command fields、pre/post task values、clip count 和 recording completeness。
- 运行失败测试：

  ```bash
  cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
  PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover -s tests -p 'test_hitter_task_pipeline.py' -v
  ```

- 实现所有实时路径；event/raw 提交必须 nonblocking，pipeline 回调立即生成深度不可变 payload，不做 JSON 编码、磁盘 I/O 或 HTTP。
- 运行 Task 1、Task 6、Task 7 与现存 lifecycle 相关测试：

  ```bash
  cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
  for test_file in \
    test_hitter_task_observation.py \
    test_hitter_task_input_adapter.py \
    test_hitter_task_pipeline.py \
    test_hitter_strike_target_logging.py; do
    PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover -s tests -p "$test_file" -v || exit 1
  done
  ```

- 提交，`hitter_realtime.py` 只暂存本任务 hunk：

  ```bash
  git add deploy/diagnostics/hitter_task_pipeline.py deploy/tests/test_hitter_task_pipeline.py
  git add -p deploy/utils/hitter_realtime.py
  git diff --cached --check
  git commit -m "feat: trace HITTER shadow task pipeline"
  ```

## Task 8: 实现确定性 baseline replay 与 `1/100` A/B

**Files:**

- Create: `deploy/diagnostics/hitter_task_replay.py`
- Create: `deploy/tests/test_hitter_task_replay.py`
- Create: `deploy/tests/test_hitter_task_replay_process.py`

### Interface

```python
@dataclass(frozen=True)
class ReplayVariant:
    name: str
    planner_rate_hz: float
    incoming_confirmations: int


@dataclass(frozen=True)
class ReplayOutcome:
    variant: str
    terminal_code: str
    task_obs_pass: bool
    boundary_sensitive: bool
    recording_complete: bool
    warnings: tuple[str, ...]
    summary_label: str | None
    metrics: Mapping[str, float | int | str | None]


@dataclass(frozen=True)
class PolicyTickRecord:
    tick_index: int
    lifecycle_now_s: float
    obs_now_s: float
    lifecycle_decision: str
    phase: str
    active_key: SnapshotKey | None
    cached_key: SnapshotKey | None
    command_fields: Mapping[str, JsonValue] | None
    task_pre_clip: tuple[float, ...] | None
    task_post_clip: tuple[float, ...] | None
    task_clip_count: int | None


@dataclass(frozen=True)
class PlannerCallRecord:
    snapshot_key: SnapshotKey
    submitted_monotonic_s: float
    started_monotonic_s: float
    completed_monotonic_s: float
    duration_s: float
    pending_replaced_key: SnapshotKey | None
    latest_replaced_key: SnapshotKey | None
    result: FrozenPlannerResult


@dataclass(frozen=True)
class OnlineAttemptTrace:
    attempt_id: int
    stages: tuple[AttemptTransition, ...]
    planner_calls: tuple[PlannerCallRecord, ...]
    policy_ticks: tuple[PolicyTickRecord, ...]
    terminal_code: str
    recovery_duration_s: float | None
    submission_phase_anchor_s: float | None
    recording_complete: bool


@dataclass(frozen=True)
class ReplayInputBundle:
    attempt_id: int
    inputs: tuple[NormalizedMocapSample, ...]
    online_baseline: OnlineAttemptTrace
    recording_complete: bool


class PlannerProtocol(Protocol):
    def plan_command(
        self,
        ball_position: np.ndarray,
        ball_velocity: np.ndarray,
        *,
        current_base_xy_w: np.ndarray,
        base_forward_xy_w: np.ndarray,
        strike_type: str | None,
    ) -> object:
        """Return one HitterWbcCommand-compatible command."""


class PlannerDurationProvider(Protocol):
    def duration_s(
        self,
        *,
        variant: ReplayVariant,
        snapshot_key: SnapshotKey,
        measured_offline_duration_s: float | None,
    ) -> float:
        """Reuse online duration or return one recorded offline duration."""


@dataclass(frozen=True)
class ReplayTrace:
    variant: ReplayVariant
    stages: tuple[AttemptTransition, ...]
    planner_calls: tuple[PlannerCallRecord, ...]
    policy_ticks: tuple[PolicyTickRecord, ...]
    outcome: ReplayOutcome


@dataclass(frozen=True)
class ReplayParity:
    matches: bool
    mismatches: tuple[str, ...]


@dataclass(frozen=True)
class ReplayComparison:
    baseline: ReplayOutcome
    one_frame: ReplayOutcome | None
    parity: ReplayParity
    summary_label: str
    deltas: Mapping[str, JsonValue]


def replay_attempt(
    bundle: ReplayInputBundle,
    *,
    variant: ReplayVariant,
    duration_provider: PlannerDurationProvider,
    planner_factory: Callable[[], PlannerProtocol],
) -> ReplayTrace:
    """Run one deterministic discrete-event replay without real sleep."""


def compare_baseline_to_online(
    *,
    online: OnlineAttemptTrace,
    replay: ReplayTrace,
    atol: float = 1.0e-6,
) -> ReplayParity:
    """Compare every canonical online fact before counterfactual execution."""


def analyze_attempt_ab(
    bundle: ReplayInputBundle,
    *,
    planner_factory: Callable[[], PlannerProtocol],
    duration_provider: PlannerDurationProvider,
) -> ReplayComparison:
    """Run 3/100, gate on parity, then and only then run fresh 1/100."""


@dataclass(frozen=True)
class ReplayJobRef:
    session_basename: str
    attempt_id: int
    input_path: Path


@dataclass(frozen=True)
class ReplayWorkerReply:
    attempt_id: int
    status: str
    result: Mapping[str, JsonValue] | None
    error: str | None


class ReplayProcessController:
    def set_realtime_busy(
        self,
        *,
        attempt_active: bool,
        reacquire_grace_active: bool,
    ) -> None:
        """Set/clear pause and wait for child PAUSED acknowledgement."""

    def submit_disk_job(self, job: ReplayJobRef) -> None:
        """Append QUEUED to replay_jobs.jsonl before IPC submission."""

    def poll(self) -> tuple[ReplayWorkerReply, ...]:
        """Parent receives replies, records them, and publishes events."""

    def close(self, *, timeout_s: float = 5.0) -> bool:
        """Stop the spawned child with a bounded join."""
```

`analyze_attempt_ab()` 的固定顺序为：

```python
baseline = replay_attempt(
    bundle,
    variant=ReplayVariant("3/100", 100.0, 3),
    duration_provider=duration_provider,
    planner_factory=planner_factory,
)
parity = compare_baseline_to_online(
    online=bundle.online_baseline,
    replay=baseline,
)
if not parity.matches:
    return replay_divergence_comparison(baseline, parity)
one_frame = replay_attempt(
    bundle,
    variant=ReplayVariant("1/100", 100.0, 1),
    duration_provider=duration_provider,
    planner_factory=planner_factory,
)
return compare_completed_variants(baseline, one_frame, parity)
```

每次 `replay_attempt()` 必须调用 `planner_factory()` 得到全新 planner。canonical parity 逐项比较：stage/reason、attempt/segment/key、submit/start/complete/replacement 序列和统计、lifecycle decision/phase、active/cached key、实际 recovery duration、完整 command fields、pre/post task values、clip count、terminal code；浮点绝对容差固定 `1e-6`。
terminal code 固定使用 `TASK_OBS_PASS`、`LATE_SKIP`、`TRACK_ENDED_BEFORE_READY`、`TRACK_ENDED_BEFORE_CONFIRMATION`、`NO_VALID_PLAN`、`PELVIS_UNAVAILABLE`、`MALFORMED_COMMAND`、`RECORDING_INCOMPLETE`、`REPLAY_DIVERGENCE`；禁止同时出现缩写 `TRACK_ENDED_BEFORE_CONFIRM`。

- 实现测试用 heap scheduler，排序键固定为 `(at_s, event_priority, global_order_seq)`；`input_seq` 仅保留在 INPUT payload。
- 写 fake 100 Hz submit phase、capacity-one pending/in-flight/latest-result、planner start/complete、50 Hz `lifecycle_now_s`/`obs_now_s` 的金丝雀测试。
- 固定同一虚拟时刻的事件优先级：`INPUT → PLAN_COMPLETE → PLAN_START → LIFECYCLE_TICK → OBS_TICK → GRACE_EXPIRE`；incoming count 只在 `PLAN_START` 增加，不在 raw input/submit 时增加。idle submit 和 complete 后 pending start 都必须形成 PLAN_START。
- 100 Hz submission 完全使用 recorded input arrival 做事件驱动 throttle：`received_monotonic_s-last_submit >= 0.01-1e-12`，不生成独立周期 timer。
- 写线上 baseline 与 replay 阶段/identity/ARM/task obs 完全一致测试；任一不同必须 `REPLAY_DIVERGENCE + INCONCLUSIVE`。
- 写 `3/100` late、同输入 `1/100` PASS 的 `SAVED_BY_ONE_FRAME` 测试。
- 写 `SAME_PASS`、`BASELINE_ONLY_PASS`、`BOTH_FAIL_SAME`、`BOTH_FAIL_DIFFERENT`、`BOTH_PASS_DIFFERENT_COMMAND`、`INCONCLUSIVE`。
- 写 `ONE_FRAME_UNSTABLE`、距 ARM 边界 `<5 ms` 的 `BOUNDARY_SENSITIVE` 和 delta metrics。
- 写 baseline duration 按 online snapshot identity 复用，`1/100` 新 planner 调用由离线 provider 实测并记录的测试。
- 写 replay supervisor 仅在无 active/reacquire attempt 时派发；新 attempt 发送 pause，父进程是唯一 EventHub/recorder writer。pending job 状态写入独立 `replay_jobs.jsonl`（QUEUED/RUNNING/PAUSED/COMPLETED/FAILED），不是分析输出文件。
- 在 `test_hitter_task_replay_process.py` 使用 `multiprocessing.get_context("spawn")` 和 Event/Queue 同步验证：active/reacquire 时 idle；新 attempt 暂停；磁盘 job 不丢；下一 idle window 从头确定性重放；child failure 只返回 FAILED；close 可在 5 秒内 join。禁止用固定 sleep 猜时序。
- child 在每个 scheduler event 前后检查 pause；PAUSED 后丢弃半执行的 mutable planner 状态，下次 idle 从输入文件开头确定性重放。child 禁止直接写 EventHub 或 session 文件。
- 运行失败测试：

  ```bash
  cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
  PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover -s tests -p 'test_hitter_task_replay.py' -v
  PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover -s tests -p 'test_hitter_task_replay_process.py' -v
  ```

- 实现默认 variants，代码中不得存在实时 `360 Hz` 完整 planner variant；360 Hz raw 数据只作为输入保留。
- 提交：

  ```bash
  git add deploy/diagnostics/hitter_task_replay.py \
    deploy/tests/test_hitter_task_replay.py \
    deploy/tests/test_hitter_task_replay_process.py
  git diff --cached --check
  git commit -m "feat: compare HITTER incoming replay variants"
  ```

## Task 9: 实现只读 localhost HTTP/SSE

**Files:**

- Create: `deploy/diagnostics/hitter_task_web.py`
- Create: `deploy/tests/test_hitter_task_web.py`

### Interface

```python
@dataclass(frozen=True)
class WebServerConfig:
    host: str = "127.0.0.1"
    port: int = 8765
    max_handlers: int = 8
    max_sse_clients: int = 4
    request_timeout_s: float = 5.0
    heartbeat_s: float = 15.0


class HitterTaskWebApplication:
    def __init__(
        self,
        *,
        hub: EventHub,
        attempts: AttemptDetailRepository,
        static_files: Mapping[str, Path],
    ) -> None:
        """Build immutable JSON payloads from the canonical Task 3 schema."""


class HitterTaskWebServer:
    def __init__(
        self,
        app: HitterTaskWebApplication,
        config: WebServerConfig,
        *,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        """Create a bounded ThreadingHTTPServer; port 0 is valid in tests."""

    @property
    def address(self) -> tuple[str, int]:
        """Return the actual bound loopback address."""

    def start(self) -> None:
        """Start one daemon accept thread."""

    def close(self, *, timeout_s: float = 5.0) -> bool:
        """Close SSE cursors, shutdown the server, and join finitely."""
```

### Routes

```text
GET /
GET /api/state
GET /api/attempts?limit=50&before=<exclusive_positive_attempt_id>
GET /api/attempts/<positive_attempt_id>
GET /events
```

SSE 只发送：

```text
ready      {"watermark_event_id": 123}
invalidate {"scope": "state|attempt", "attempt_id": 7}
reset      {"watermark_event_id": 456}
heartbeat  {"watermark_event_id": 456}
```

- 用 `ThreadingHTTPServer` 绑定 ephemeral port，使用 `http.client` 测 state schema 和 attempt desc/exclusive pagination；默认 limit 50，最大 200。
- 测 invalid id/query/method 返回 JSON 400/404/405，包含 `Content-Type`、`Content-Length`、`Cache-Control: no-store`。
- 测只映射 `/` 和明确静态资源，`..`、绝对路径、encoded traversal 均 404。
- 测 Host 仅允许 `127.0.0.1`、`localhost` 和实际端口；无 wildcard CORS；所有响应带 `nosniff`。
- 测 fresh SSE `ready` 后客户端 GET state；Last-Event-ID ring 内补发 invalidate；ring gap 发 reset；旧 event 从不携带状态 patch。
- 用 fake heartbeat clock 测 15 秒 heartbeat；测试断开释放 cursor/thread。
- 测总 handler semaphore 8、SSE semaphore 4、超限 503、普通 socket 5 秒 timeout。
- 测 detail 只从 cache 或 numeric fixed file 读取，API JSON 不含绝对路径和下载字段。
- 运行失败测试：

  ```bash
  cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
  PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover -s tests -p 'test_hitter_task_web.py' -v
  ```

- 实现 HTTP/SSE；SSE slow client 只落后自身 cursor，不持有 EventHub publish lock。
- 提交：

  ```bash
  git add deploy/diagnostics/hitter_task_web.py deploy/tests/test_hitter_task_web.py
  git diff --cached --check
  git commit -m "feat: serve HITTER diagnostic state"
  ```

## Task 10: 实现前端逐球进度页

**Files:**

- Create: `deploy/diagnostics/static/hitter_task_monitor.html`
- Modify: `deploy/tests/test_hitter_task_web.py`
- Create: `deploy/tests/test_hitter_task_frontend.py`

- 先添加静态契约测试：必须包含 6 个主阶段、固定 shadow 声明、健康栏、历史表、detail 区域和 A/B 区域。
- 测 HTML/JS 不含 `innerHTML`、`document.write`、外部 CDN、POST/fetch mutation；动态文本必须通过 `textContent`。
- 实现顶部健康栏：LCM Hz/age、ball/pelvis frame、pelvis age/valid、planner stats、phase、diagnostic drops、config/session basename。
- 实现当前 attempt 的：

  ```text
  球检测
  → ESTIMATING n/31
  → PLANNER-READY INCOMING n/3
  → PLANNER
  → ARMED
  → SHADOW 3/100 TASK OBS 11/11 PASS
  ```

- 实现 `WAITING_FOR_PREVIOUS_RECOVERY`、`CACHED_DURING_RECOVERY`、`REACQUIRE_GRACE` 与 `POST_DEADLINE_TAIL`。
- 实现历史分页、展开 detail、11 维 pre/post clip、primary blocker、planner vectors、A/B outcome/deltas。
- SSE 仅作 invalidator：收到 ready/invalidate/reset 后调度完整 GET `/api/state`；断线指数退避并保持页面可读。
- GET 使用单 token-bucket 规则：事件可立即调度，但距上次 request 不足 100 ms 时合并到下一个 100 ms 边界；任意 1 秒滑窗不超过 10 次。阶段/终态不会等待超过 100 ms。
- 前端丢弃 watermark 小于当前已渲染 watermark 的旧 GET 响应，避免多个异步请求倒序覆盖新状态。
- 运行：

  ```bash
  cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
  PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover -s tests -p 'test_hitter_task_web.py' -v
  PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover -s tests -p 'test_hitter_task_frontend.py' -v
  ```

- 在 `test_hitter_task_frontend.py` 用 `html.parser` 验证 landmarks；若存在 `google-chrome-stable`，以临时 profile、`--headless=new --dump-dom` 打开真实测试 server，注入唯一 runtime marker 和形似 HTML 的恶意文本，断言 marker 已渲染且恶意文本没有成为 DOM element。调用固定为 `subprocess.run(..., timeout=10, check=True)`，测试 server 和 temp profile 在 `finally` 关闭；无 Chrome 时只跳过该一项，不跳过静态契约。
- 用浏览器手工确认 1280×720 和 1920×1080 下无横向溢出，断开 SSE 时黄灯而非空白页。
- 提交：

  ```bash
  git add deploy/diagnostics/static/hitter_task_monitor.html \
    deploy/tests/test_hitter_task_web.py \
    deploy/tests/test_hitter_task_frontend.py
  git diff --cached --check
  git commit -m "feat: show HITTER task diagnostic progress"
  ```

## Task 11: 装配 CLI、配置快照与有界关闭

**Files:**

- Create: `deploy/diagnostics/hitter_task_monitor.py`
- Create: `deploy/tests/test_hitter_task_monitor.py`
- Create: `docs/hitter_task_observation_diagnostics.md`

### Interface

```python
@dataclass(frozen=True)
class MonitorOptions:
    mimic_config: Path
    control_config: Path
    table_calib: Path
    pelvis_calib: Path
    lcm_url: str
    channel: str
    base_name: str
    ball_name: str
    port: int
    output_dir: Path
    duration_s: float
    reacquire_grace_s: float
    heartbeat_stale_s: float
    pelvis_stale_s: float
    source_frame_gap_threshold: int
    raw_capacity: int
    event_capacity: int
    attempt_cache_size: int


class HitterTaskMonitor:
    def start(self) -> None:
        """Start recorder, web, LCM owner and 50 Hz loop in that order."""

    def run(self) -> int:
        """Run until duration, signal or failure; no hardware-control dependency."""

    def close(self, *, timeout_s: float = 5.0) -> bool:
        """Execute the exact finite shutdown order once; idempotent."""


def build_monitor(
    options: MonitorOptions,
    *,
    lcm_factory: Callable[[str], object] = lcm.LCM,
    monotonic_fn: Callable[[], float] = time.monotonic,
    wall_time_fn: Callable[[], float] = time.time,
) -> HitterTaskMonitor:
    """Dependency-injected composition root used by CLI and safety tests."""
```

### CLI contract

默认命令：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m diagnostics.hitter_task_monitor \
  --mimic-config config/mimic/hitter.yaml \
  --control-config config/control/g1_hitter_racket.yaml \
  --table-calib mocap_bridge/calibrations/chingmu_table_frame_latest.json \
  --pelvis-calib mocap_bridge/calibrations/chingmu_pelvis_orientation_latest.json \
  --lcm-url 'udpm://239.255.76.67:7667?ttl=255' \
  --channel vicon_state_data \
  --base-name G2Pelvis \
  --port 8765
```

CLI 只允许 `127.0.0.1` 监听，不提供远程 bind 参数。可覆盖：

```text
--output-dir
--reacquire-grace-s
--heartbeat-stale-s
--pelvis-stale-s
--source-frame-gap-threshold
--raw-capacity
--event-capacity
--attempt-cache-size
--duration
```

- 默认 `output_dir` 固定为 `Path(__file__).resolve().parents[2] / "recordings/hitter_task_diagnostics"`，不依赖启动 cwd；显式相对 `--output-dir` 按调用时 cwd resolve，一并写入 session metadata，但网页/API 只显示 basename。
- 测 `OmegaConf.load()` 读取 mimic/control，不需要 Hydra instantiate；resolved 配置值为 31 / 360 / 100 / 3 / 50 Hz。
- 测 session metadata 包含 config/calibration SHA-256、LCM URL、argv、git commit/dirty、Python/包版本、schema version；HTTP/API 不暴露绝对路径。
- 测 CLI 初始化期间不 import agent、ONNX 或 simulator，不等待 `body_control_data`/R2。
- 用 fake LCM fd/decoder 测 owner loop 的 exact arrival order 和 stop；真实 code 使用 `select.select([lc.fileno()], [], [], 0.1)`，只有 fd ready 才调用 `handle()`，每轮检查 stop/duration。只调用 `subscribe`/`handle`/`unsubscribe`，不得调用 `publish`。
- 写零消息 owner-loop 测试：`--duration 0.2` 仍能停止、unsubscribe、join；禁止直接永久阻塞在 `lcm.LCM.handle()`。
- 测 50 Hz loop 每 tick 调用 monotonic 两次并分别传给 lifecycle/obs。
- 测 shutdown 顺序和每步有限 timeout：

  ```text
  stop LCM receive
  → close/join planner and replay
  → recorder.request_stop() + drain()（文件保持打开）
  → recorder.write_terminal_and_close()（terminal + fsync/flush）
  → shutdown HTTP
  → join remaining threads
  ```

- 写中文 runbook，包含两个终端：

  终端 A：

  ```bash
  cd /home/loco1/BOB/Hitter/RobotBridge2
  /home/loco1/miniconda3/envs/rb/bin/python deploy/mocap_bridge/chingmu_table_lcm_bridge.py \
    --host 192.168.2.100 \
    --base-subject G2Pelvis \
    --table-calib deploy/mocap_bridge/calibrations/chingmu_table_frame_latest.json \
    --pelvis-orientation-calib deploy/mocap_bridge/calibrations/chingmu_pelvis_orientation_latest.json \
    --publish
  ```

  终端 B 使用上面的 monitor 命令，然后浏览器打开 `http://127.0.0.1:8765/`。

- 运行失败测试，再实现：

  ```bash
  cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
  PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover -s tests -p 'test_hitter_task_monitor.py' -v
  ```

- `--duration 0.2` + fake LCM smoke 必须无悬挂退出。
- 提交：

  ```bash
  git add deploy/diagnostics/hitter_task_monitor.py \
    deploy/tests/test_hitter_task_monitor.py \
    docs/hitter_task_observation_diagnostics.md
  git diff --cached --check
  git commit -m "feat: run HITTER task diagnostic monitor"
  ```

## Task 12: 端到端、性能、安全和当前回归验收

**Files:**

- Create: `deploy/tests/test_hitter_task_diagnostics_integration.py`
- Create: `deploy/tests/test_hitter_task_diagnostics_safety.py`
- Create: `deploy/tests/test_hitter_task_diagnostics_performance.py`
- Modify: `docs/hitter_task_observation_diagnostics.md`

- 构造 360 Hz synthetic input + scripted planner：31 帧 ready → incoming 3/3 → plan → ARMED → 11/11 PASS。
- 构造高速边界球：online/replay `3/100` LATE，`1/100` PASS，summary `SAVED_BY_ONE_FRAME`。
- 覆盖非来球、无 directed crossing、高度越界、pelvis invalid、track ended before ready/confirm/arm。
- 覆盖单帧 invalid 后 grace 内重捕获同 attempt 多 segment、grace 外新 attempt、post-deadline tail。
- 覆盖连续两球、上一球 recovery、下一球 cached，验证 attempt/result/obs identity 不串线。
- 同一录制 bundle 连续 replay 两次，要求 events、outcome、task obs 数值完全相同。
- 模拟 raw/event/file/HTTP/replay 任一路故障，实时 adapter/pipeline 仍继续，受影响 attempt 必须 INCONCLUSIVE。
- 安全测试通过 Task 11 的 `lcm_factory` 注入 fake instance；fake 的 `publish()` 立即记录并抛错，完整诊断仍 PASS 且 `published_messages == []`。禁止 patch 不可变的 C-extension `lcm.LCM.publish` 属性。
- 在 `test_hitter_task_diagnostics_safety.py` 另外验证：recorder failure 不停止 pipeline、web client 断开不停止 state/event 推进、fake LCM 的 `published_messages` 始终为空、shutdown 后端口可立即复用且所有 worker thread/process 已结束。
- 运行全部新增功能测试：

  ```bash
  cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
  for test_file in \
    test_hitter_task_observation.py \
    test_hitter_runtime_factory.py \
    test_hitter_task_attempts.py \
    test_hitter_task_events.py \
    test_hitter_task_recording.py \
    test_hitter_task_input_adapter.py \
    test_hitter_task_pipeline.py \
    test_hitter_task_replay.py \
    test_hitter_task_replay_process.py \
    test_hitter_task_web.py \
    test_hitter_task_frontend.py \
    test_hitter_task_monitor.py \
    test_hitter_task_diagnostics_integration.py \
    test_hitter_task_diagnostics_safety.py; do
    PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover -s tests -p "$test_file" -v || exit 1
  done
  ```

- 运行当前工作树中仍存在的相关回归：

  ```bash
  cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
  for test_file in \
    test_hitter_strike_target_logging.py \
    test_real_world_connection_wait.py \
    test_mujoco_physical_table_tennis.py; do
    PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover -s tests -p "$test_file" -v || exit 1
  done
  ```

- 运行 opt-in 60 秒真实性能验收；测试内部生成 360 Hz 输入，报告 handler p50/p95/p99、raw drop、event drop、RSS 增量和 browser refresh rate：

  ```bash
  cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
  HITTER_DIAGNOSTICS_PERF=1 PYTHONPATH=. \
    /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover \
    -s tests -p 'test_hitter_task_diagnostics_performance.py' -v
  ```

  验收要求：`raw_samples_dropped == 0`、LCM owner handler p99 `< 2.78 ms`、页面完整 state GET `<=10 Hz`、内存不随事件数线性增长；新 attempt 触发 replay pause 后，实时 raw/event backlog 不增长。

- 运行 Python 3.8 compile/import smoke：

  ```bash
  cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
  /home/loco1/miniconda3/envs/rb/bin/python -m py_compile \
    utils/hitter_task_observation.py \
    utils/hitter_runtime_factory.py \
    diagnostics/hitter_task_models.py \
    diagnostics/hitter_task_attempts.py \
    diagnostics/hitter_task_events.py \
    diagnostics/hitter_task_recording.py \
    diagnostics/hitter_task_pipeline.py \
    diagnostics/hitter_task_replay.py \
    diagnostics/hitter_task_web.py \
    diagnostics/hitter_task_monitor.py
  PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python - <<'PY'
  import diagnostics.hitter_task_attempts
  import diagnostics.hitter_task_events
  import diagnostics.hitter_task_models
  import diagnostics.hitter_task_monitor
  import diagnostics.hitter_task_pipeline
  import diagnostics.hitter_task_recording
  import diagnostics.hitter_task_replay
  import diagnostics.hitter_task_web
  import utils.hitter_runtime_factory
  import utils.hitter_task_observation
  PY
  ```

- 运行静态安全扫描：

  ```bash
  cd /home/loco1/BOB/Hitter/RobotBridge2
  if rg -n "onnxruntime|InferenceSession|pd_plustau_targets|body_control_data|rc_command_data" \
    deploy/diagnostics; then
    echo "forbidden policy/control dependency" >&2
    exit 1
  fi
  if rg -n "\\blc(m|_client)?\\.publish\\(|\\bpublish\\([^)]*pd_plustau_targets" \
    deploy/diagnostics; then
    echo "forbidden LCM/control publish" >&2
    exit 1
  fi
  if rg -n "innerHTML|document\\.write|Access-Control-Allow-Origin:\\s*\\*" \
    deploy/diagnostics/static deploy/diagnostics/hitter_task_web.py; then
    echo "forbidden frontend/web pattern" >&2
    exit 1
  fi
  ```

  前两条只允许测试中的禁止项断言，不允许运行路径 import/调用；第三条必须无命中。零 LCM publish 的主要证据仍是 fake-LCM 自动化 safety test，静态扫描只是补充。

- 手工只启动 bridge + monitor，不启动 policy/trans；确认页面实时显示 ball/pelvis、31/31、incoming 3/3，并生成会话文件。
- 现场检查 HTTP 安全与 loopback：

  ```bash
  ss -ltnH 'sport = :8765' | rg -q '127\\.0\\.0\\.1:8765'
  if ss -ltnH 'sport = :8765' | rg -q '0\\.0\\.0\\.0:8765|\\[::\\]:8765'; then
    echo "diagnostic server is not loopback-only" >&2
    exit 1
  fi
  check_dir=$(mktemp -d)
  curl -sS -D "$check_dir/state.headers" \
    http://127.0.0.1:8765/api/state \
    -o "$check_dir/state.json"
  /home/loco1/miniconda3/envs/rb/bin/python -m json.tool \
    "$check_dir/state.json" >/dev/null
  rg -qi '^X-Content-Type-Options: nosniff' "$check_dir/state.headers"
  rg -qi '^Cache-Control: no-store' "$check_dir/state.headers"
  if rg -qi '^Access-Control-Allow-Origin:' "$check_dir/state.headers"; then
    echo "unexpected CORS header" >&2
    exit 1
  fi
  test "$(curl -sS -o /dev/null -w '%{http_code}' \
    -H 'Host: evil.test' http://127.0.0.1:8765/api/state)" = 400
  test "$(curl -sS -o /dev/null -w '%{http_code}' \
    -X POST http://127.0.0.1:8765/api/state)" = 405
  test "$(curl --path-as-is -sS -o /dev/null -w '%{http_code}' \
    http://127.0.0.1:8765/../../etc/passwd)" = 404
  ```

  `ss` 结果必须只有 `127.0.0.1:8765`，不能是 `0.0.0.0` 或 `[::]`；headers 中不得出现 `Access-Control-Allow-Origin`。
- 用户连续投掷至少一颗正常球和一颗高速球；逐球核对页面、`attempts.csv`、detail JSON 和 A/B 结果一致。
- 在 runbook 记录实际性能数值、现场会话目录 basename、已知 shadow 限制，不写绝对路径到网页输出。
- 最终暂存并提交测试/文档：

  ```bash
  git add deploy/tests/test_hitter_task_diagnostics_integration.py \
    deploy/tests/test_hitter_task_diagnostics_safety.py \
    deploy/tests/test_hitter_task_diagnostics_performance.py \
    docs/hitter_task_observation_diagnostics.md
  git diff --cached --check
  git commit -m "test: verify HITTER task diagnostics end to end"
  ```

## Final Review Gate

- 使用 `superpowers:requesting-code-review` 做独立 review，重点检查 invalid/incoming/lifecycle 时序和“shadow 非生产事实”措辞。
- 使用 `superpowers:verification-before-completion` 重新运行 Task 12 的全部命令，不引用旧输出。
- `git status --short` 核对没有意外暂存用户原有 dirty 文件，没有恢复删除文件。
- `git diff --check` 与 `git diff --cached --check` 都通过。
- 向用户提供：
  - bridge 命令；
  - monitor 命令；
  - 页面地址；
  - 新增/修改文件清单；
  - 测试和性能证据；
  - 明确的 shadow 能力边界；
  - 下一步现场逐球诊断方法。
