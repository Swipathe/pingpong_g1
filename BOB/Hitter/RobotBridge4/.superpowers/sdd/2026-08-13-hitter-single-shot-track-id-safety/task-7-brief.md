### Task 7: 实现 RealWorld v2 identity、freshness 与下一发门禁

**Files:**
- Modify: `deploy/simulator/real_world.py: _init_ball_state, _init_communication, _vicon_state_handler, _update_ball_state_from_vicon, reset estimator, poll/status APIs`
- Modify: `deploy/utils/hitter_runtime_factory.py`
- Create: `deploy/tests/test_real_world_v2_consumer.py`

**Interfaces:**
- Consumes: Task 1 v2 schema、Task 4 `BallEstimateSnapshot.track_id`。
- Produces: `ViconInputFault`、`ViconConsumerEvent`、`ViconConsumerStatus`、`begin_hitter_policy_session()`、`hitter_vicon_status()`、`drain_hitter_vicon_events()`、`consume_hitter_track()`；listener 只接收 planning-eligible snapshot。

`test_real_world_v2_consumer.py` 不启动网络线程：`world` fixture 以 `RealWorld.__new__()` 加生产初始化 helper 建立锁、estimator 和 v2 状态；`ball()` / `pelvis()` 构造 Task 1 生成类型；`ingest_valid_pelvis[_and_ball]()` 只调用 `_ingest_vicon_v2_message()`；`encoded_rc()` 使用现有 `wireless_controller_t` 生成合法 payload。测试结束显式关闭任何由 fixture 创建的线程。

- [ ] **Step 1: 写 v2 ID/帧/reset/conflict 失败测试**

```python
def test_same_wire_id_survives_reset_and_rejects_out_of_order_frames(world):
    world._ingest_vicon_v2_message(ball(track_id=7, frame=10), received_monotonic_s=1.0)
    world._ingest_vicon_v2_message(ball(track_id=7, frame=11), received_monotonic_s=1.01)
    generation = world.ball_snapshot_generation
    samples = world.ball_state_estimator.sample_count
    world._ingest_vicon_v2_message(ball(track_id=7, frame=11), received_monotonic_s=1.02)
    world._ingest_vicon_v2_message(ball(track_id=7, frame=9), received_monotonic_s=1.03)
    assert world.ball_snapshot_generation == generation
    assert world.ball_state_estimator.sample_count == samples
    world.reset_ball_state_estimator()
    assert world.active_ball_track_id == 7
    assert world.last_ball_track_id == 7
    assert world.seen_ball_track_ids == {7}


def test_overlapping_new_id_consumes_both_and_latches_conflict(world):
    world._ingest_vicon_v2_message(ball(track_id=7, frame=10), received_monotonic_s=1.0)
    world._ingest_vicon_v2_message(ball(track_id=8, frame=11), received_monotonic_s=1.01)
    status = world.hitter_vicon_status(now_monotonic_s=1.01)
    assert {7, 8} <= world.consumed_ball_track_ids
    assert status.latched_fault is ViconInputFault.TRACK_ID_CONFLICT
```

- [ ] **Step 2: 写 decode/poll-liveness 与 freshness 失败测试**

```python
def test_bad_fingerprint_latches_schema_error_without_raising(world):
    world._vicon_state_handler("vicon_state_data_v2", b"bad fingerprint")
    assert world.vicon_fault_latched is ViconInputFault.VICON_SCHEMA_ERROR
    world._remote_controller_handler("rc_command_data", encoded_rc(r2=True))
    assert world.right_lower_right_switch_pressed is True


def test_stream_stale_is_latched_until_policy_reentry(world):
    ingest_valid_pelvis_and_ball(world, now=1.0, track_id=7)
    status = world.hitter_vicon_status(now_monotonic_s=1.401)
    assert status.latched_fault is ViconInputFault.VICON_STREAM_STALE
    ingest_valid_pelvis(world, now=1.41)
    assert world.hitter_vicon_status(now_monotonic_s=1.42).latched_fault \
        is ViconInputFault.VICON_STREAM_STALE


def test_reentry_clears_recovered_stale_fault_and_accepts_next_id(world):
    ingest_valid_pelvis_and_ball(world, now=1.0, track_id=7)
    world.hitter_vicon_status(now_monotonic_s=1.401)
    ingest_valid_pelvis(world, now=1.41)
    assert world.begin_hitter_policy_session(now_monotonic_s=1.42)
    assert world.hitter_vicon_status(now_monotonic_s=1.92).ready_for_new_serve
    world._ingest_vicon_v2_message(
        ball(track_id=8, frame=20), received_monotonic_s=1.921
    )
    assert world.active_ball_track_id == 8


def test_reentry_rejects_unrecovered_stale_stream(world):
    ingest_valid_pelvis_and_ball(world, now=1.0, track_id=7)
    world.hitter_vicon_status(now_monotonic_s=1.401)
    assert not world.begin_hitter_policy_session(now_monotonic_s=1.402)
    assert world.vicon_fault_latched is ViconInputFault.VICON_STREAM_STALE


def test_invalid_ball_emits_one_direct_track_end_event(world):
    world._ingest_vicon_v2_message(ball(track_id=7, frame=10), received_monotonic_s=1.0)
    world._ingest_vicon_v2_message(
        ball(track_id=7, frame=11, valid=False),
        received_monotonic_s=1.01,
    )
    events = world.drain_hitter_vicon_events()
    assert [(event.reason, event.track_id) for event in events] == [
        (ViconEventReason.TRACK_ENDED, 7)
    ]
    assert world.drain_hitter_vicon_events() == ()
```

同文件用参数化 subject/message case 精确覆盖：ball ID 0/-1 不进 estimator；pelvis/table 非零 ID fail closed；invalid 保留 ID 且只结束一次；新 ID generation 从 1 开始；age `0.400001 s` 触发 ball stale而 `0.40 s` 不触发；startup visible ID进入 quarantine；0.49 s 到达的新 ID立即 consumed且 0.50 s 后不复活；任意有效 ball 清零 no-ball timer；seen/consumed 在 estimator reset 和 policy reset 后保持。每个 case 断言 estimator count、active/last ID、event tuple、fault 和 listener submit count。

新增 estimator 隔离测试：先用 ID 7 填满窗口直至 ready，再收 invalid 7、完整 0.50 s no-ball 和 ID 8 首帧；此时 ID 8 的 generation 必须为 1、`sample_count == 1`、`ready is False`，任何拟合输入都不能含 ID 7 sample。未通过 admission 而被 quarantine/consume 的新 ID 同样先切断旧 estimator window，但不得把自身 sample 加入 planning estimator。

- [ ] **Step 3: 运行测试，确认 v1 hardcode、本地 ID 与无 freshness 导致失败**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  deploy/tests/test_real_world_v2_consumer.py
```

Expected: FAIL，缺少 v2 status API 或仍订阅 `vicon_state_data`。

- [ ] **Step 4: 增加 v2 状态类型和 process-lifetime 集合**

```python
class ViconInputFault(str, Enum):
    VICON_SCHEMA_ERROR = "VICON_SCHEMA_ERROR"
    VICON_STREAM_STALE = "VICON_STREAM_STALE"
    TRACK_ID_CONFLICT = "TRACK_ID_CONFLICT"
    BALL_MESSAGE_STALE = "BALL_MESSAGE_STALE"


class ViconEventReason(str, Enum):
    TRACK_ENDED = "TRACK_ENDED"
    BASE_POSE_INVALID = "BASE_POSE_INVALID"
    VICON_SCHEMA_ERROR = "VICON_SCHEMA_ERROR"
    VICON_STREAM_STALE = "VICON_STREAM_STALE"
    BALL_MESSAGE_STALE = "BALL_MESSAGE_STALE"
    TRACK_ID_CONFLICT = "TRACK_ID_CONFLICT"


@dataclass(frozen=True)
class ViconConsumerEvent:
    sequence: int
    reason: ViconEventReason
    track_id: int | None
    detail: str


@dataclass(frozen=True)
class ViconConsumerStatus:
    stream_fresh: bool
    ball_fresh: bool
    base_pose_valid: bool
    active_track_id: int | None
    last_track_id: int | None
    visible: bool
    ready_for_new_serve: bool
    latched_fault: ViconInputFault | None
```

`ViconInputFault` 只表示 status 中需要跨包锁存的 fault；`ViconEventReason` 表示送往 lifecycle 的一次性 transition，因而额外包含 `TRACK_ENDED` 与 `BASE_POSE_INVALID`。两者不得混作一个 enum或裸字符串。Task 8 的 `LifecycleCancelReason` 必须覆盖 `ViconEventReason` 的全部同名 value；参数化 contract test 对每个 `ViconEventReason` 断言 `LifecycleCancelReason(reason.value)` 成功。

初始化 `active_ball_track_id`、`last_ball_track_id`、per-ID generation/frame、seen/consumed/admitted sets、last-any/last-ball monotonic、no-ball start、policy-session flag，以及按 sequence 排序、容量固定为 64 的 transition-event deque；这些集合不得在 estimator reset 或 HitterEnv lifecycle reset 清空。队列满时禁止静默丢弃单个 transition：必须把队列原子替换成一个永久锁存的 `VICON_SCHEMA_ERROR` overflow event，使 lifecycle fail closed。`BallEstimateSnapshot` 增加 `consumed: bool` 与 `new_track: bool`，供 diagnostics 和 HitterEnv 在 listener 边界做最终 phase gate。

`ViconConsumerSettings` 还必须解析严格的 `base_subject`，默认值与两个 active publisher 一致为精确字符串 `G2Pelvis`；空值、保留名 `ball/table` 或非字符串在进入真机前拒绝。RealWorld 只接受配置指定的 pelvis subject，subject 不匹配按永久 `VICON_SCHEMA_ERROR` fail closed。测试 fixture 不能依赖工作树中的 G1/G2 未提交覆盖，提交后必须从 Git archive 复测干净快照。

`ready_for_new_serve` 的生产表达式固定为：policy session 已开启、没有 latched fault、stream fresh、pelvis valid、当前没有 visible/active ID，且 `now-no_ball_since >= 0.50 s`。这里不检查 HitterEnv phase；phase 在 Task 9 listener 临界区做最终 gate。一个 ID 只有首次包可被加入 admitted set；admitted 后该 ID 的后续递增 frame 可在 `TRACKING/ARMED` 继续细化，不能再次当作“新发球”检查。

- [ ] **Step 5: 用 callback 最外层异常边界隔离 LCM poll thread**

```python
def _vicon_state_handler(self, channel, data):
    received = time.monotonic()
    try:
        msg = transformation_t.decode(data)
    except Exception as exc:
        self._latch_vicon_fault(
            ViconInputFault.VICON_SCHEMA_ERROR,
            received_monotonic_s=received,
            detail=f"{type(exc).__name__}: {exc}",
        )
        return
    self._ingest_vicon_v2_message(msg, received_monotonic_s=received)
```

删除逐 ball `print`。subscription channel 来自已校验 settings，且只能为 `vicon_state_data_v2`。

- [ ] **Step 6: 实现 wire ID/帧/invalid/conflict 规则**

先校验完整 wire contract：subject-ID 合法，source frame 为非负整数，`valid/occluded` 只能是互补的 `1/0` 或 `0/1`，三维位置与四元数均有限且四元数非零；table 只允许 valid。ball 必须正 ID，配置指定的 pelvis/table 必须 0。每个新 ID 的 generation 从 1 开始；同 ID 仅严格递增 source frame 增加 generation。任何未见 wire ID 到达时，必须在把它设为 active 或评估其 sample 之前 reset 单一 `BallStateEstimator` 的 samples/readiness，确保旧 ID 的位置永不参与新 ID 速度拟合；然后用该包到达前的 no-ball 时长、stream/base/session 状态决定是否加入 admitted set，再清零 no-ball timer。任何通过 wire contract 的有效 ball（包括重复或乱序 frame）都必须在 frame 早退前清零 no-ball timer，但重复/乱序 frame 不推进 generation、estimator 或 planner。未满足条件立即加入 consumed set，且不把该包加入 planning estimator。外部 estimator reset 仍只清 samples/readiness，不改 identity/history。invalid ball 必须带当前正 ID：校验它等于 `active_ball_track_id` 后，令 active 变 `None`、保留 `last_ball_track_id` 和 per-ID history，清 visible并 enqueue 一次 `TRACK_ENDED`，同时开始 no-ball 计时。这里“不改身份”指结束消息、history 和诊断仍归旧 ID，不是让旧 ID继续占据 active slot。active 时出现另一个 ID，无论该 ID 是否已在 process lifetime 见过，都先切断 estimator window，再消费双方、latch 并 enqueue `TRACK_ID_CONFLICT`。pelvis 从 valid 变 invalid 时 enqueue `BASE_POSE_INVALID`。已消费 ID 保留 diagnostics 可见性但不通知 planner listener；reentry quarantine 也必须同步把 latest snapshot 的 `consumed` 标志改为真。

新增测试固定结束后的双字段语义：ID 7 invalid 后 `status.active_track_id is None`、`status.last_track_id == 7`，per-ID generation/frame 仍保留；连续 no-ball 到 `0.50 s` 且其余 gate 有效时 `ready_for_new_serve is True`，随后首次出现的未见 ID 8 才能成为 active。

- [ ] **Step 7: 实现 tick 驱动的 freshness 与 policy reentry**

```python
def hitter_vicon_status(self, *, now_monotonic_s: float) -> ViconConsumerStatus:
    with self._hitter_ball_state_lock:
        self._apply_freshness_locked(float(now_monotonic_s))
        return self._status_locked(float(now_monotonic_s))
```

任意 v2 包 age `>0.40`：base invalid、active ball ended、latch 并 enqueue 一次 stream stale。active ball 包 age `>0.40`：结束该 ID并 enqueue 一次 ball stale。`drain_hitter_vicon_events()` 在锁内按 sequence 返回并清空这些 transition；普通数据恢复不清 latch。`begin_hitter_policy_session()` 先重新计算 freshness/subject contract：若 stream 仍 stale、pelvis invalid，或存在不可恢复的 `VICON_SCHEMA_ERROR`（当前进程收到过错误 fingerprint，无法证明部署类型一致），返回 false/raise 且不清 fault；只有 stream fresh、pelvis valid 且没有协议错误时，才原子清上一 policy session 的可恢复 `VICON_STREAM_STALE/BALL_MESSAGE_STALE/TRACK_ID_CONFLICT` latch 与 event-dedupe flags、开启新 session，并消费当时可见 ID。若当时无球，则从 reentry 时刻开始连续 no-ball 计时。schema fault 需要修复发布/接收部署并重启进程，不能用 R2/reentry 清除。

任何永久 `VICON_SCHEMA_ERROR`（包括 transition queue overflow）必须在锁存 fault 的同一临界区立即 quarantine 当前 active ID：加入 consumed set、把 matching latest snapshot 重建为 `consumed=True`、切断 estimator samples/readiness，但保留 active/visible 身份仅供物理可见性诊断。此后同 ID 或任何新 ID 的有效包均不得再进入 estimator 或 planner listener。conflict challenger 无论是否已见，都必须把 `ball_track_last_frames[challenger]` 更新为已接收 frame 的最大值，但不得推进 generation 或取得 active authority；这样 conflict 后的旧帧 replay 仍被 watermark 拒绝。

- [ ] **Step 8: 运行 consumer tests 和通信线程关闭回归**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  deploy/tests/test_real_world_v2_consumer.py \
  deploy/tests/test_real_world_connection_wait.py
```

Expected: PASS；坏 payload 后 RC handler 仍可执行；恢复包不能自行解除 stale/schema latch。

- [ ] **Step 9: 提交 RealWorld v2 consumer 单元**

```bash
git add deploy/tests/test_real_world_v2_consumer.py
git add -p -- deploy/simulator/real_world.py
git add -p -- deploy/utils/hitter_runtime_factory.py
git diff --cached --check
git diff --cached --name-status
git commit -m "feat: consume authoritative HITTER track ids"
```

---
