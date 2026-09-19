### Task 9: 集成 HitterEnv、50 Hz 配置与真机 WAITING 固定 anchor

**Files:**
- Modify: `deploy/simulator/real_world.py`
- Modify: `deploy/utils/hitter_runtime_factory.py`
- Modify: `deploy/config/mimic/hitter.yaml`
- Modify: `deploy/config/hitter.yaml`
- Modify: `deploy/config/sim/real_world.yaml`
- Modify: `deploy/envs/hitter.py`
- Modify: `deploy/agents/hitter_agent.py`
- Modify: `deploy/tests/test_hitter_runtime_factory.py`
- Modify: `deploy/tests/test_hitter_strike_target_logging.py`
- Create: `deploy/tests/test_hitter_waiting_anchor.py`
- Create: `deploy/tests/test_hitter_runtime_single_shot_integration.py`
- Modify: `deploy/tests/test_hitter_policy_first_frame_transition.py`
- Modify: `deploy/tests/test_hitter_task_observation.py`
- Modify: `deploy/tests/test_real_world_v2_consumer.py`
- Create: `deploy/tests/hitter_runtime_test_harness.py`

**Interfaces:**
- Consumes: Task 6 batch drain、Task 7 status/consume API、Task 8 decisions。
- Produces: 每 policy tick 的固定处理顺序、全参数校验、真机 waiting anchor edge capture；MuJoCo 行为不变。

实现前合同裁决：

- Task 7 已落地的 nested `motion.vicon_consumer.{channel,base_subject,stream_timeout_s,ball_timeout_s,new_serve_no_ball_s,event_queue_capacity}` 是唯一生产配置源；禁止再增加 motion 顶层或 ball_planner 下的同义字段。`ViconConsumerSettings` 单独解析/测试这些字段，`HitterRuntimeSettings` 只保存 planner/lifecycle 参数。下方示例中的旧长字段名按此裁决替换。
- Task 9 扩展 RealWorld consumer API：`end_hitter_policy_session()` 关闭 admission 并原子 quarantine 当前 active；成功的 `begin_hitter_policy_session()` 同一临界区清除上个 session 遗留的可恢复 transition events；`hitter_snapshot_planning_eligible(snapshot)` 在 consumer lock 内二次检查 session/fault/active/admitted/consumed/latest key；`consume_hitter_track(track_id, reason=...)` 保存诊断原因。锁顺序固定为 HitterEnv lifecycle RLock 后取 consumer state lock，RealWorld 始终锁外调用 listener。
- 同一进程仅创建一次 lifecycle 和 worker。reset/reentry 先关闭 snapshot admission 与 consumer session，调用 Task 8 `reset_for_policy_reentry()`，把 consumer active、所有曾 submitted/outstanding ID、已排队 batch/overflow ID 全部同步为 consumed，并丢弃 reset 前结果；in-flight 结果稍后完成也只能 `ignored_consumed`。禁止普通 reset `new HitterCommandLifecycle()`。
- 真机每个 agent iteration 只在 ONNX 前执行一次固定 lifecycle tick；real-world `_post_physics_step()` 不再第二次 drain/advance，MuJoCo 保留原 post-physics 路径。事件、advance、batch 的顺序严格为 status/freshness -> drain events/cancel -> one `advance()` -> one completed batch drain/ordered ingest。
- production result path 完全移除 `latest_result()`/字符串错误/缓存 recovery 兼容路径。每条 decision 后只从 `lifecycle.active_result.command` 更新 policy command，不能从被拒绝或被重建前的 incoming command 复制。
- 首帧平滑期间 consumer session 和 planner admission 保持关闭。reset 后先捕获初始 WAITING anchor 供第一帧 observation 使用；现有 5 秒 PD transition 完成后再刷新 pelvis、重新捕获当前位置 anchor、丢弃/消费旧 worker 结果并成功 begin session，从该时刻开始 0.50 s no-ball 计时。transition 为 0 时在 reset 完成后立即执行相同 reentry 流程。
- `BASE_POSE_INVALID` 无论 lifecycle 当前是 WAITING、pre-commit ARMED 还是 committed，都立即锁存 `_waiting_anchor_fault` 并保留已有 anchor；同 session 内 pose 恢复不能重新 admission，只有显式成功 reentry 才清 fault。每个 `entered_waiting` 边沿捕获前先刷新 simulator state，且每个 decision 只捕获一次。
- listener 在 lifecycle lock 内依次检查 runtime accepting、anchor fault、process-lifetime consumed、phase 和 consumer eligibility；RECOVERY 的旧/新 snapshot 都消费，非 WAITING 的 `new_track` 也消费。rate gate 与非阻塞 `worker.submit()` 在同一 lifecycle lock 内完成，提交前登记 process-lifetime submitted ID，消除 phase-check/submit TOCTOU 与 Task 7 deferred stale-listener race。
- shared test harness 固定放在 `deploy/tests/hitter_runtime_test_harness.py`，不得通过恢复已删除旧测试取得 fixture。`test_hitter_strike_target_logging.py` 必须随本任务一并迁移和暂存。

新增测试文件共享一个纯离线 harness：`make_hitter_env_for_test(is_real, settings)` 只分配 HitterEnv 的 lifecycle/planner/anchor 字段并注入 fake simulator；fake simulator 实现 Task 7 的 status/event/consume API；`agent_harness.run_ticks()` 调用真实 observation 组装、mock ONNX session 和 spy `apply_action()`。所有时钟由显式 `now` 驱动，不使用 sleep，也不创建 Unitree publisher。

- [ ] **Step 1: 写所有默认值和非法配置失败测试**

```python
def test_single_shot_defaults_are_exact(settings):
    assert settings.planner_update_rate_hz == 50.0
    assert settings.waiting_tts_s == 0.92
    assert settings.arm_tts_s == 0.92
    assert settings.minimum_arm_tts_s == 0.30
    assert settings.maximum_policy_tts_s == 0.92
    assert settings.completed_result_queue_capacity == 64
    assert settings.armed_cancel_consecutive_failures == 3
    assert settings.commit_time_to_strike_s == 0.30
    assert settings.maximum_racket_target_override_delta_m == 0.05
    assert settings.maximum_racket_velocity_override_delta_mps == 0.75
    assert settings.maximum_strike_deadline_override_delta_s == 0.05


def test_vicon_defaults_are_exact(vicon_settings):
    assert vicon_settings.channel == "vicon_state_data_v2"
    assert vicon_settings.base_subject == "G2Pelvis"
    assert vicon_settings.stream_timeout_s == 0.40
    assert vicon_settings.ball_timeout_s == 0.40
    assert vicon_settings.new_serve_no_ball_s == 0.50
    assert vicon_settings.event_queue_capacity == 64
```

参数化拒绝 NaN/inf/0/negative；queue/count 的 bool、float、0 和负数均拒绝；v1 channel 拒绝。

- [ ] **Step 2: 写 anchor 边沿、104-D 与 ONNX/PD 连续调用失败测试**

```python
def test_real_waiting_anchor_is_captured_once_and_corrects_drift(env):
    env.simulator.base_pose_valid = True
    env.simulator.root_trans_world = np.array([1.2, -0.3, 0.78])
    assert env._capture_hitter_waiting_base_anchor(require_initial=True)
    env.simulator.root_trans_world = np.array([1.2, -0.1, 0.78])
    pos, quat = env._hitter_robot_anchor_pose_w()
    target_b = env._hitter_waiting_base_target_pos_b(pos, quat)
    assert np.allclose(env.waiting_base_anchor_xy_w, [1.2, -0.3])
    assert target_b[1] < 0.0


def test_waiting_keeps_policy_and_pd_running(agent_harness):
    agent_harness.run_ticks(10, phase=CommandPhase.WAITING)
    assert agent_harness.onnx_calls == 10
    assert agent_harness.apply_action_calls == 10
    assert agent_harness.last_observation.shape == (1, 104)
    assert np.isfinite(agent_harness.last_observation).all()
```

在 `test_hitter_waiting_anchor.py` 增加 `pytest.mark.parametrize("edge", ["cancel","recovery_complete"])` 验证每个 WAITING edge 只重新捕获一次；另设四个独立测试断言：(1) invalid+old anchor 保留旧值并 block；(2) pose 在同一 policy session 恢复后新 ID 的 planner submit 计数仍为 0且不进入 ARMED；(3) 显式 reentry 成功后才清 fault；(4) invalid+no anchor 抛 RuntimeError。MuJoCo case 断言仍使用 YAML configured waiting target。

- [ ] **Step 3: 写 ordered batch 和 next-serve 集成失败测试**

同一 tick 注入 `success -> failure -> failure -> failure`，断言四条都被消费且第三个 failure 撤拍；注入 overflow 断言 pre-commit 立即 cancel、整批结果均不 ingest、batch 内 ID 和被溢出 ID 全部 consume；0.49 s 前出现的新 ID被 consume，0.50 s 后不能复活，只有之后出现的未见 ID可提交 planner。

两个 barrier 测试各使用 `threading.Event(phase_checked,allow_submit,policy_started)` 和 1 秒 timeout：(1) listener 在锁外准备 ID 8 后阻塞，policy 先进入 RECOVERY，释放 listener 后断言 ID 8 consumed/submit 0；(2) fake worker.submit 设置 `phase_checked` 后等 `allow_submit`，policy thread 在此期间不能取得 lifecycle lock或改变 phase，释放后 submit 完成才允许 RECOVERY。另一个测试在同一 tick 预置 `TRACK_ENDED` event 和成功 result，断言 event 先撤拍、后到 result 因 ID 已 consumed 被忽略，证明直接安全事件在一个 20 ms tick 内生效。

再预先创建一条 `consumed=False,new_track=False` 的旧 active snapshot，然后先完成 strike/进入 RECOVERY、最后才调用 listener；锁内 lifecycle consumed/RECOVERY gate 必须令 submit 计数保持不变，证明 snapshot 创建时的旧标志不能绕过消费状态。

队列完整性用两个独立测试：(1) lifecycle 尚在 WAITING，`CompletedResultBatch(results=(success7,),overflow_count=1,overflowed_track_ids=(8,))`，tick 后保持 WAITING且 worker result ingest spy 为 0；(2)随后分别排队 ID 7/8 success，均返回 ignored consumed、armed count 0。

消费状态同步以 scenario 参数化 `late_skip,third_soft,immediate,third_discontinuity`；每个 scenario 走真实 tick 得到含 ID 7 的 `decision.consumed_track_ids`，随后断言 RealWorld consumed set 含 7。注入 frame+1 后 listener submit spy 不增加，最后 diagnostics snapshot 的 `consumed is True`。

- [ ] **Step 4: 运行配置、anchor 和 integration tests，确认旧 fixed target/latest-result 路径失败**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  deploy/tests/test_hitter_runtime_factory.py \
  deploy/tests/test_hitter_waiting_anchor.py \
  deploy/tests/test_hitter_runtime_single_shot_integration.py \
  deploy/tests/test_hitter_policy_first_frame_transition.py \
  deploy/tests/test_hitter_task_observation.py
```

Expected: FAIL，至少显示 planner 默认仍 100 Hz、waiting target 仍 `[-0.4,0]` 或 controller 只取 latest result。

- [ ] **Step 5: 扩展并严格校验 runtime settings**

`HitterRuntimeSettings` 增加 Global Constraints 中全部参数；单独创建 `ViconConsumerSettings(channel, stream_stale_timeout_s, ball_message_stale_timeout_s)`。浮点要求 finite 且按规范正/非负；整数必须 `type(value) is int and value > 0`。

- [ ] **Step 6: 更新 YAML 默认值并明确 sim 边界**

```yaml
motion:
  vicon_consumer:
    channel: vicon_state_data_v2
    base_subject: G2Pelvis
    stream_timeout_s: 0.40
    ball_timeout_s: 0.40
    new_serve_no_ball_s: 0.50
    event_queue_capacity: 64
  waiting_base_target_xy_w: [-0.4, 0.0]  # MuJoCo only
  ball_planner:
    planner_update_rate_hz: 50.0
    completed_result_queue_capacity: 64
    armed_cancel_consecutive_failures: 3
    commit_time_to_strike_s: 0.30
    maximum_racket_target_override_delta_m: 0.05
    maximum_racket_velocity_override_delta_mps: 0.75
    maximum_strike_deadline_override_delta_s: 0.05
```

- [ ] **Step 7: 按固定顺序整合每个 policy tick**

listener 先做新轨迹的 lifecycle phase gate；未通过的 ID 当场消费，不能等 WAITING 后复活：

```python
def _submit_hitter_planner_snapshot(self, snapshot: BallEstimateSnapshot) -> None:
    if snapshot.consumed:
        return
    with self._hitter_lifecycle_lock:
        if self._waiting_anchor_fault is not None:
            self.simulator.consume_hitter_track(
                snapshot.track_id,
                reason=f"waiting_anchor_fault:{self._waiting_anchor_fault.value}",
            )
            self.hitter_command_lifecycle.consume_track(
                snapshot.track_id,
                reason="waiting_anchor_fault",
            )
            return
        phase = self.hitter_command_lifecycle.phase
        if snapshot.track_id in self.hitter_command_lifecycle.consumed_track_ids:
            self.simulator.consume_hitter_track(
                snapshot.track_id, reason="lifecycle_already_consumed"
            )
            return
        if phase is CommandPhase.RECOVERY:
            self.simulator.consume_hitter_track(
                snapshot.track_id, reason="snapshot_during_recovery"
            )
            self.hitter_command_lifecycle.consume_track(
                snapshot.track_id, reason="snapshot_during_recovery"
            )
            return
        if snapshot.new_track and phase is not CommandPhase.WAITING:
            self.simulator.consume_hitter_track(
                snapshot.track_id,
                reason=f"new_track_during_{phase.value}",
            )
            self.hitter_command_lifecycle.consume_track(
                snapshot.track_id,
                reason="serve_gate_closed",
            )
            return
        if not self._planner_submit_interval_elapsed(snapshot.received_monotonic_s):
            return
        # submit() is non-blocking; keeping it inside this lock makes the
        # phase check and publication to the worker one atomic admission step.
        self.hitter_planner_worker.submit(snapshot)

def _planner_submit_interval_elapsed(self, received_monotonic_s: float) -> bool:
    received = float(received_monotonic_s)
    previous = self._hitter_last_planner_submit_monotonic_s
    if previous is not None and (
        received - previous
        < self.hitter_runtime_settings.planner_update_interval_s - 1.0e-12
    ):
        return False
    self._hitter_last_planner_submit_monotonic_s = received
    return True
```

`_init_hitter_lifecycle_state()` 创建 `threading.RLock()`；listener 的 anchor-fault/consumed/phase check、rate gate、非阻塞 submit/consume 与 policy tick 的 `advance/cancel/ingest` 都在该锁下执行，消除 phase-check 到 submit 的 TOCTOU。即使 snapshot 是 consume 前创建的旧对象，也会在锁内通过 lifecycle process-lifetime consumed set 被挡住；RECOVERY 对 `new_track=False` 的旧 active snapshot 同样 fail closed。`_waiting_anchor_fault` 一旦在同一 policy session 锁存，即使 pelvis 后来恢复为 valid，新 ID 也要被双方消费且不能提交 planner；只有显式 policy reentry 成功重捕 anchor 后清除。首次 policy reset 完成后调用 `begin_hitter_policy_session(now_monotonic_s=now)`；若已有球可见，该 ID 已被 quarantine，只有它结束并连续无球 0.50 s 后出现的未见 ID才可能通过。

```python
status = self.simulator.hitter_vicon_status(now_monotonic_s=now)
for event in self.simulator.drain_hitter_vicon_events():
    reason = LifecycleCancelReason(event.reason.value)
    self._apply_hitter_lifecycle_decision(
        self.hitter_command_lifecycle.cancel(
            reason=reason, now=now, track_id=event.track_id
        ),
        now=now,
    )
advance_decision = self.hitter_command_lifecycle.advance(now=now)
self._apply_hitter_lifecycle_decision(advance_decision, now=now)

batch = self.hitter_planner_worker.drain_completed_results()
if batch.overflowed:
    compromised_ids = {
        result.track_id for result in batch.results
    } | set(batch.overflowed_track_ids)
    for track_id in sorted(compromised_ids):
        self.simulator.consume_hitter_track(
            track_id, reason="completed_result_queue_overflow"
        )
        self.hitter_command_lifecycle.consume_track(
            track_id, reason="completed_result_queue_overflow"
        )
    self._apply_hitter_lifecycle_decision(
        self.hitter_command_lifecycle.cancel(
            reason=LifecycleCancelReason.RESULT_QUEUE_OVERFLOW, now=now
        ),
        now=now,
    )
else:
    for result in batch.results:
        decision = self.hitter_command_lifecycle.ingest(result, now=now)
        self._apply_hitter_lifecycle_decision(decision, now=now)
```

`_apply_hitter_lifecycle_decision()` 必须遍历 `decision.consumed_track_ids`，对每个 ID 在同一 `_hitter_lifecycle_lock` 内调用 `simulator.consume_hitter_track(track_id, reason=decision.kind)`；late skip、cancel、third-soft、strike、recovery consume 和 track end 不再靠 `kind` 分支猜测。overflow 在构造 decision 前已逐 ID同步 batch/丢失 ID。这样 RealWorld 不再向 listener 发送该 ID 的 planning-eligible snapshot。这个同步必须幂等，不能重置 generation 或 no-ball timer。

overflow 表示整批顺序证据已不完整，因此禁止 ingest 该 batch 中任何 success/failure；即使当时没有 active command，也要保持 WAITING。若已有 active ID 且不在 `compromised_ids`，`cancel()` 仍会原子消费它。在 ingest 前 drain 的 schema/stale/ball-stale/conflict/base/track-ended transition 进入 pre-commit immediate cancel；同一 event 只出现一次。strike 后只 reset estimator samples；ID 消费由统一 decision-application 路径同步。不缓存 recovery 新球。

- [ ] **Step 8: 实现 WAITING entry edge capture**

```python
def _capture_hitter_waiting_base_anchor(
    self, *, require_initial: bool, policy_reentry: bool = False
) -> bool:
    if not self.simulator.is_real:
        return True
    if self._waiting_anchor_fault is not None and not policy_reentry:
        return False
    if not bool(self.simulator.base_pose_valid):
        if self.waiting_base_anchor_xy_w is None and require_initial:
            raise RuntimeError(
                "Cannot enter HITTER policy without a valid G2Pelvis waiting anchor"
            )
        self._waiting_anchor_fault = LifecycleCancelReason.BASE_POSE_INVALID
        return False
    position = np.asarray(self.simulator.root_trans_world, dtype=np.float64)[:2]
    if position.shape != (2,) or not np.isfinite(position).all():
        raise RuntimeError("G2Pelvis waiting anchor is not a finite xy vector")
    self.waiting_base_anchor_xy_w = position.astype(np.float32, copy=True)
    if policy_reentry:
        self._waiting_anchor_fault = None
    return True
```

reset/policy reentry 用 `policy_reentry=True`；未 arm end、late skip、cancel、recovery complete 的 `entered_waiting` edge 用默认 false。invalid edge 后即使 pose 数据恢复也保持 fault 和旧 anchor，直到下一次显式 policy reentry。WAITING observation 每帧计算 `anchor-current pelvis`，racket 为当前 FK、velocity 0、TTS 0.92；不增加 action gating。

- [ ] **Step 9: 把 planner summary 限到 1 Hz 并移除控制循环负载噪声**

状态转换/撤拍/协议错误即时一次；ball/planner/queue 汇总最多每秒一次。保留 agent 每 tick ONNX 与 `env.step()->apply_action()` 调用。

- [ ] **Step 10: 运行核心 runtime 回归**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  deploy/tests/test_hitter_runtime_factory.py \
  deploy/tests/test_hitter_waiting_anchor.py \
  deploy/tests/test_hitter_runtime_single_shot_integration.py \
  deploy/tests/test_hitter_policy_first_frame_transition.py \
  deploy/tests/test_hitter_task_observation.py
```

Expected: PASS；WAITING observation 为 `(1,104)` 且 10 tick 对应 10 次 ONNX/PD；MuJoCo tests 不变。

- [ ] **Step 11: 提交 runtime integration 单元**

```bash
git add deploy/config/sim/real_world.yaml \
  deploy/simulator/real_world.py \
  deploy/tests/test_hitter_waiting_anchor.py \
  deploy/tests/test_hitter_runtime_single_shot_integration.py \
  deploy/tests/hitter_runtime_test_harness.py \
  deploy/tests/test_hitter_policy_first_frame_transition.py \
  deploy/tests/test_hitter_task_observation.py \
  deploy/tests/test_real_world_v2_consumer.py \
  deploy/tests/test_hitter_strike_target_logging.py
git add -p -- deploy/utils/hitter_runtime_factory.py
git add -p -- deploy/config/mimic/hitter.yaml deploy/config/hitter.yaml
git add -p -- deploy/envs/hitter.py deploy/agents/hitter_agent.py
git add -p -- deploy/tests/test_hitter_runtime_factory.py
git diff --cached --check
git diff --cached --name-status
git commit -m "feat: integrate safe single-shot HITTER runtime"
```

---

