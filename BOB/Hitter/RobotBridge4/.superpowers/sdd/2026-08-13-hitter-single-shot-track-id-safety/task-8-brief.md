### Task 8: 实现单拍 lifecycle、命令锁定、撤拍与 commit window

**Files:**
- Modify: `deploy/utils/hitter_realtime.py: CommandPhase, HitterCommandLifecycle`
- Create: `deploy/tests/test_hitter_single_shot_lifecycle.py`
- Create: `deploy/tests/hitter_test_factories.py`

**Interfaces:**
- Consumes: Task 5 typed reason/command、Task 6 ordered results、Task 7 consumed ID 语义。
- Produces: `LifecycleCancelReason`、`LifecycleDecision`、`ingest()` / `advance()` / `cancel()`；每 ID 最多一个 strike transition。

跨 Task 复用的 `command(side,base,position,velocity)`、`success()` 与 `failure()` 放在 `hitter_test_factories.py`：前者构造完整 `HitterWbcCommand`，后两者构造满足 Task 6 invariant 的结果。只属于 lifecycle 场景的 `armed_lifecycle*()`、`lifecycle_after_terminal()` 与 `lifecycle_in_recovery()` 放在 `test_hitter_single_shot_lifecycle.py`；它们只通过公开 `ingest/advance/cancel` 建立状态，不直接篡改私有字段。Task 10 复用 factory，不复制命令/result 构造逻辑。

Task 8 是纯状态机迁移：`ingest()` 不得隐式调用 `advance()`，caller 必须先显式调用 `advance(now=...)` 并应用其 decision，之后才按序 ingest completed results；否则 strike 消费 transition 会被吞掉。`advance()`、`ingest()`、`cancel()` 和 `mark_track_ended()` 始终返回 `LifecycleDecision`，无状态变化时也返回 `kind="none"`。Task 9 负责在同一后续提交中迁移 `HitterEnv` 与 `test_hitter_strike_target_logging.py`；Task 10 负责迁移 diagnostics pipeline 与 `test_hitter_task_pipeline.py`。禁止给 `LifecycleDecision` 增加和裸字符串相等的兼容行为。

实现前合同裁决：

- `cancel()` 的当前身份先取 `active_result.track_id`，没有 active command 时取正在 `TRACKING` 的 `_current_track_id`；pre-commit cancel 原子消费当前 ID 与 event ID，避免同 tick queued success 重 arm。
- success 和 failure 共用每 ID generation watermark。只接受严格递增 generation；duplicate/older result 不改变 failure streak，matching active ID 的递增 soft failure 才能累计，continuous success 才清零。
- 每次 `advance()` 最多跨一个 phase。即使 `now` 已同时越过 strike 与 recovery deadline，第一次也只产生 `struck` 并停在 `RECOVERY`，下一次才产生 `entered_waiting`；zero-duration recovery 亦然。
- commit、`policy_tts()` 与 strike crossing 只使用第一次 ARMED 锁定的 deadline。`TTS <= 0.30` 时整条 command 冻结；合法 result 仍推进 generation watermark并可记录 failure，但不改变 command、failure streak 或 recovery deadline。commit 内 cancel 仍消费相关 ID，但保留已承诺 command，随后仍恰好产生一次 strike。
- continuous override 必须重建 frozen command/result：只采用新 position 与 velocity；同一 velocity 同步写入 `HitterWbcCommand.v_racket_target_w` 和 `StrikePlan.v_racket_target`，`strike_table_y_w` 同步为新 position y；强制保留 locked side/base/deadline、recovery deadline、side source、time-to-strike、plan t_strike 与 ball in/out，并防御性复制数组。
- `entered_waiting` 表示需要 Task 9 捕获 WAITING anchor 的语义边沿，而不只是 enum 发生变化。late skip、首次未 arm track end、安全撤拍、recovery complete 为 true；重复/已消费事件和 recovery 内 cancel 为 false。
- `LifecycleCancelReason` 必须与全部 `PlannerFailureReason` 及 `ViconEventReason` 同值覆盖；公开输入采用 exact positive `int`，拒绝 bool/float/string；decision kind 使用固定 vocabulary；`consumed_track_ids` 返回本 decision 涉及的全部相关 ID，即使此前已消费。
- 默认和约束采用最终配置：`waiting_tts=0.92`、`arm_tts=0.92`、`minimum_arm_tts=0.30`、`maximum_policy_tts=0.92`、连续 failure 数 `3`、commit `0.30`、position/velocity/deadline override 阈值 `0.05/0.75/0.05`、swing duration `1.85`。所有数值必须 finite 且满足 `0 <= commit <= minimum_arm <= arm <= maximum_policy`、`0 <= waiting_tts <= maximum_policy`；failure count 为 exact positive int，阈值为 finite nonnegative。sampler 异常或非法结果转为 typed `INTERNAL_ERROR` 撤拍，不得从 policy tick 外泄。
- process-lifetime 的 consumed IDs、消费原因、generation watermark、strike count、最后锁定字段和最后 failure/cancel 原因不能因普通 transition 清除。Task 9 不得在同一 policy 进程内重新创建 lifecycle；只有进程启动时创建一次，普通 env reset 必须复用该实例。若未来确需重建，必须显式迁移这些身份状态。
- Task 8 提供公开 `reset_for_policy_reentry() -> LifecycleDecision`（或等价明确命名），供 Task 9 在普通 env reset/显式 policy reentry 时复用同一实例。该 transition 不受普通 commit/RECOVERY cancel freeze 限制：原子消费 active 或 TRACKING current ID，清 active command、recovery deadline、failure streak与当前瞬态身份，进入 WAITING并返回 `kind="session_reset"` / `entered_waiting=True`；纯初始状态也产生一次初始 WAITING anchor 边沿。它必须保留 consumed IDs/原因、每 ID generation watermark、strike count与最后诊断字段，旧 ID 后续 result 只能 `ignored_consumed`。

- [ ] **Step 1: 写 consumed、recovery 与 late skip 失败测试**

```python
@pytest.mark.parametrize("terminal", ["skipped", "cancelled", "struck", "ended"])
def test_terminal_track_id_can_never_arm_again(terminal):
    lifecycle = lifecycle_after_terminal(track_id=7, terminal=terminal)
    decision = lifecycle.ingest(success(track_id=7, generation=99), now=2.0)
    assert decision.kind == "ignored_consumed"
    assert lifecycle.phase is CommandPhase.WAITING
    assert 7 in lifecycle.consumed_track_ids


def test_recovery_consumes_new_id_without_caching():
    lifecycle = lifecycle_in_recovery(track_id=7)
    decision = lifecycle.ingest(success(track_id=8, generation=1), now=1.1)
    assert decision.kind == "consumed_during_recovery"
    assert 8 in lifecycle.consumed_track_ids
    lifecycle.advance(now=3.0)
    assert lifecycle.phase is CommandPhase.WAITING
    assert lifecycle.active_result is None
```

- [ ] **Step 2: 写锁定、override、连续失败与 commit 边界失败测试**

```python
def test_first_arm_locks_side_base_and_deadline():
    lifecycle = armed_lifecycle(command=command("forehand", base=[-0.4, -0.2]),
                                 deadline=2.0, now=1.1)
    changed = success(track_id=7, generation=2, strike_type="backhand",
                      base=[-0.4, 0.4], deadline=2.01)
    lifecycle.ingest(changed, now=1.2)
    assert lifecycle.locked_strike_type == "forehand"
    assert np.allclose(lifecycle.locked_base_target_xy, [-0.4, -0.2])
    assert lifecycle.locked_strike_deadline_monotonic_s == 2.0


@pytest.mark.parametrize(
    "field,delta",
    [
        ("position_m", 0.050001),
        ("velocity_mps", 0.750001),
        ("deadline_s", 0.050001),
    ],
)
def test_third_override_discontinuity_cancels(field, delta):
    lifecycle = armed_lifecycle_before_commit()
    for expected_count in (1, 2):
        decision = lifecycle.ingest(
            discontinuous_result(field=field, delta=delta), now=1.0
        )
        assert decision.kind == "retained_discontinuity"
        assert lifecycle.consecutive_failure_count == expected_count
    decision = lifecycle.ingest(
        discontinuous_result(field=field, delta=delta), now=1.0
    )
    assert decision.entered_waiting
    assert decision.cancel_reason is LifecycleCancelReason.OVERRIDE_DISCONTINUITY


def test_conflict_event_consumes_active_and_event_ids_atomically():
    lifecycle = armed_lifecycle_before_commit(track_id=7)
    decision = lifecycle.cancel(
        reason=LifecycleCancelReason.TRACK_ID_CONFLICT,
        now=1.0,
        track_id=8,
    )
    assert decision.entered_waiting
    assert decision.consumed_track_ids == (7, 8)
    assert {7, 8} <= lifecycle.consumed_track_ids
    assert lifecycle.ingest(success(track_id=7, generation=2), now=1.0).kind \
        == "ignored_consumed"
    assert lifecycle.ingest(success(track_id=8, generation=1), now=1.0).kind \
        == "ignored_consumed"
```

`discontinuous_result(field,delta)` 从当前已接受命令复制所有字段，只修改 `field` 指定的一个量，因此三种单位不会混用。另测 TTS 恰好 `0.30` 时 success/failure 均不改变 frozen command。

- [ ] **Step 3: 运行 lifecycle 测试，确认现有 recovery cache/全命令覆盖行为失败**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  deploy/tests/test_hitter_single_shot_lifecycle.py
```

Expected: FAIL，显示新 ID 被 cached、failure ignored 或 active command 整体被覆盖。

- [ ] **Step 4: 定义 decision/cancel reason 和锁定状态**

```python
class LifecycleCancelReason(str, Enum):
    TRACK_ENDED = "TRACK_ENDED"
    ESTIMATOR_NOT_READY = "ESTIMATOR_NOT_READY"
    BASE_POSE_INVALID = "BASE_POSE_INVALID"
    BALL_NOT_INCOMING = "BALL_NOT_INCOMING"
    NO_FUTURE_CROSSING = "NO_FUTURE_CROSSING"
    HIT_HEIGHT_OUT_OF_RANGE = "HIT_HEIGHT_OUT_OF_RANGE"
    NONFINITE_INPUT_OR_OUTPUT = "NONFINITE_INPUT_OR_OUTPUT"
    INTERNAL_ERROR = "INTERNAL_ERROR"
    VICON_SCHEMA_ERROR = "VICON_SCHEMA_ERROR"
    VICON_STREAM_STALE = "VICON_STREAM_STALE"
    BALL_MESSAGE_STALE = "BALL_MESSAGE_STALE"
    TRACK_ID_CONFLICT = "TRACK_ID_CONFLICT"
    RESULT_QUEUE_OVERFLOW = "RESULT_QUEUE_OVERFLOW"
    OVERRIDE_DISCONTINUITY = "OVERRIDE_DISCONTINUITY"


@dataclass(frozen=True)
class LifecycleDecision:
    kind: str
    track_id: int | None
    command_changed: bool = False
    entered_waiting: bool = False
    cancel_reason: LifecycleCancelReason | None = None
    consumed_track_ids: tuple[int, ...] = ()

    def __post_init__(self) -> None:
        canonical = tuple(sorted(set(self.consumed_track_ids)))
        if canonical != self.consumed_track_ids or any(
            type(track_id) is not int or track_id <= 0 for track_id in canonical
        ):
            raise ValueError("consumed_track_ids must be sorted unique positive ids")
```

state 保存 process-lifetime `consumed_track_ids`、current/last generation、locked fields、failure streak、strike count；删除 `cached_result` 和恢复后自动 arm。所有公开 transition 返回本次新消费涉及的 `consumed_track_ids`，HitterEnv 不根据 `kind` 猜测消费语义。

所有 deadline crossing 只由显式 `advance()` 产生：`ARMED -> RECOVERY` 返回 `kind="struck"`、消费 active ID并把该 ID 的 strike count 加一；`RECOVERY -> WAITING` 返回 `kind="entered_waiting"` 且不得重复报告消费；其他 tick 返回 `kind="none"`。同一 ID 的 `strike_count_by_track_id` 永远不超过 1。

- [ ] **Step 5: 实现 unarmed、late、recovery 和 deadline transitions**

late 是 `remaining < minimum_arm_tts(0.30)`：立即 consume，并在 decision 中返回该 ID 与 `entered_waiting=True`。deadline crossing 先 consume active ID，再进入 RECOVERY、把该 ID strike count 加一并返回在 `consumed_track_ids`。RECOVERY 收到任意新 ID立即 consume并在 decision 中返回；结束时只进入 WAITING，不能重复报告消费。

```python
def consume_track(self, track_id: int, *, reason: str) -> None:
    identity = int(track_id)
    if identity <= 0:
        raise ValueError("track_id must be positive")
    self.consumed_track_ids.add(identity)
    self.last_consumption_reason_by_track_id[identity] = str(reason)


def mark_track_ended(self, track_id: int, *, now: float) -> LifecycleDecision:
    return self.cancel(
        reason=LifecycleCancelReason.TRACK_ENDED,
        now=now,
        track_id=int(track_id),
    )
```

- [ ] **Step 6: 实现 pre-commit override continuity**

```python
same_track = result.track_id == active.track_id
same_side = result.command.strike_type == self.locked_strike_type
position_delta = np.linalg.norm(
    result.command.strike_plan.p_racket_target
    - active.command.strike_plan.p_racket_target
)
velocity_delta = np.linalg.norm(
    result.command.v_racket_target_w - active.command.v_racket_target_w
)
deadline_delta = abs(
    result.strike_deadline_monotonic_s
    - self.locked_strike_deadline_monotonic_s
)
continuous = (
    same_track and same_side
    and position_delta <= 0.05
    and velocity_delta <= 0.75
    and deadline_delta <= 0.05
)
```

合格更新只替换 racket position/velocity；强制写回 locked side/base/deadline。成功清 soft failure streak。超界只计 `OVERRIDE_DISCONTINUITY`，不覆盖 active。

- [ ] **Step 7: 实现 immediate/third-soft cancel 与原子清理**

immediate set：`TRACK_ENDED, BASE_POSE_INVALID, NONFINITE_INPUT_OR_OUTPUT, INTERNAL_ERROR` 加外部传入的 schema/stream-stale/ball-stale/conflict/overflow。soft set：`ESTIMATOR_NOT_READY, BALL_NOT_INCOMING, NO_FUTURE_CROSSING, HIT_HEIGHT_OUT_OF_RANGE, OVERRIDE_DISCONTINUITY`。pre-commit immediate 一次 cancel，soft 第三个有序 result cancel；commit window 只记录，不改命令。`cancel()` 在 `TTS <= 0.30` 时返回 `retained_committed`；否则在同一临界区清 active、consume ID、进入 WAITING并返回 reason。遥控器停止/下电继续走现有底层路径，不调用这个可冻结的普通 planner cancel API。

```python
def cancel(self, *, reason: LifecycleCancelReason, now: float,
           track_id: int | None = None) -> LifecycleDecision:
    active = self.active_result
    active_id = None if active is None else int(active.track_id)
    event_id = None if track_id is None else int(track_id)
    if event_id is not None and event_id <= 0:
        raise ValueError("track_id must be positive")
    ids_to_consume = {identity for identity in (active_id, event_id)
                      if identity is not None}
    self.consumed_track_ids.update(ids_to_consume)
    if self.phase is CommandPhase.RECOVERY:
        self.last_cancel_reason = reason
        return LifecycleDecision(
            kind="retained_recovery",
            track_id=event_id,
            cancel_reason=reason,
            consumed_track_ids=tuple(sorted(ids_to_consume)),
        )
    if active is not None and (
        active.strike_deadline_monotonic_s - float(now)
        <= self.commit_time_to_strike_s + self._EPSILON
    ):
        return LifecycleDecision(
            kind="retained_committed", track_id=active.track_id,
            cancel_reason=reason,
            consumed_track_ids=tuple(sorted(ids_to_consume)),
        )
    cancelled_id = active_id if active_id is not None else event_id
    was_waiting = self.phase is CommandPhase.WAITING
    self.active_result = None
    self.command_end_deadline_s = None
    self.phase = CommandPhase.WAITING
    self.last_cancel_reason = reason
    return LifecycleDecision(
        kind="cancelled", track_id=cancelled_id,
        entered_waiting=not was_waiting,
        cancel_reason=reason,
        consumed_track_ids=tuple(sorted(ids_to_consume)),
    )
```

当 conflict event 携带 ID 8 而 active ID 为 7 时，上述原子操作必须同时消费 `{7,8}`；即使同 tick completed queue 后面还有 ID 7/8 的成功结果，也只能得到 `ignored_consumed`。进入 commit window 后不修改 active command，但仍消费 event ID 和 active ID，从身份入口阻止任何重 arm。

RECOVERY 参数化测试以 `reason` 取 `TRACK_ENDED,VICON_STREAM_STALE,TRACK_ID_CONFLICT,RESULT_QUEUE_OVERFLOW`，记录调用前 recovery deadline；每次断言 decision.kind=`retained_recovery`、phase 仍 RECOVERY、deadline 不变、事件 ID进入 consumed。新 ID success case 断言 `consumed_during_recovery`。只有 `advance(now=deadline)` 产生一次 `entered_waiting`。

- [ ] **Step 8: 运行完整 lifecycle 边界测试**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  deploy/tests/test_hitter_single_shot_lifecycle.py
```

Expected: PASS；每个测试中的 `strike_count_by_track_id[track_id] <= 1`。

- [ ] **Step 9: 提交 single-shot lifecycle 单元**

```bash
git add deploy/utils/hitter_realtime.py \
  deploy/tests/test_hitter_single_shot_lifecycle.py \
  deploy/tests/hitter_test_factories.py
git diff --cached --check
git diff --cached --name-status
git commit -m "feat: enforce one strike per HITTER track id"
```
