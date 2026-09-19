# RobotBridge4 反弹保留与最低击球速度 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让已有合法击球命令的 `ARMED` 轨迹在桌面反弹后继续保留命令，同时把弱逆碰撞解提升到至少 1.0 m/s 的裁剪前球拍法向速度，并修复近端桌沿的离散碰撞漏判。

**Architecture:** 生命周期只调整 `ESTIMATOR_NOT_READY` 的 ARMED 分类，不删除通用撤销路径；`StrikePlanner` 在现有逆碰撞标量上施加可配置下限，再保留当前工作树已有的分量裁剪作为最终边界；`BallTrajectoryPredictor` 在单个预测步内求连续接触时刻和接触点。诊断字段随 `StrikePlan` 冻结和连续 override 传播，INFO 日志同时输出原始法向标量、下限后的裁剪前标量、最终目标向量和目标位置。

**Tech Stack:** Python 3.10、NumPy、pytest、OmegaConf/Hydra、Loguru、RobotBridge4 HITTER lifecycle/planner/runtime。

## Global Constraints

- 工作目录固定为 `/home/loco1/BOB/Hitter/RobotBridge4`，当前分支为 `local/robotbridge4-single-shot-track-id-20260813`。
- 当前工作树包含大量用户改动；禁止 reset、restore、checkout 覆盖或清理现有内容。
- `deploy/utils/hitter_planner.py`、`deploy/utils/hitter_runtime_factory.py`、`deploy/config/mimic/hitter.yaml` 和 `deploy/tests/test_hitter_runtime_factory.py` 在计划开始前已经是 dirty；每次提交必须使用 `git add -p` 或等价的精确 index patch，仅暂存本计划新增的行。
- 不暂存现有的 component velocity alignment、model17500、5 秒 transition、最大击球高度、站位、Vicon、标定或其他用户改动。
- 每次提交前运行 `git diff --cached --check` 和 `git diff --cached --stat`，再逐行检查 `git diff --cached -- <本任务文件>`。
- 虚拟击球面始终保持 `virtual_hit_plane_x: 0.0`；不得恢复 `x=-0.2`。
- 生产配置使用 `minimum_racket_normal_speed_mps: 1.0`；构造器默认值使用 `0.0`，以保持未显式配置调用方的旧公式。
- 速度下限作用于分量裁剪前的碰撞法向标量；当前工作树已有的分量范围裁剪继续作为最终边界。
- 不修改 ONNX、104 维 observation、29 维 action、PD 增益、Unitree `trans` 协议或 `post_hit_flight_time`。
- `BALL_NOT_INCOMING` 和 `NO_FUTURE_CROSSING` 继续在第三次连续失败时撤销；hard failure 在 precommit 继续立即撤销，commit window 内继续按现有合同 `retained_committed`。
- 自动化测试只能证明命令和预测语义，不能声称已经证明真机实际球拍速度或实际触球。
- 测试统一使用：`PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -p no:cacheprovider`。

---

## File Map

- Modify: `deploy/utils/hitter_realtime.py` — ARMED failure 分类、`StrikePlan` 诊断字段冻结、连续 override 的诊断字段传播。
- Modify: `deploy/utils/hitter_planner.py` — 连续桌面接触、最低法向速度、`StrikePlan` 诊断字段。
- Modify: `deploy/utils/hitter_runtime_factory.py` — 将原始最低速度配置传给 `StrikePlanner`，由构造器统一校验。
- Modify: `deploy/config/mimic/hitter.yaml` — 生产最低法向速度设为 1.0 m/s，保持 `virtual_hit_plane_x: 0.0`。
- Modify: `docs/superpowers/specs/2026-08-13-hitter-bounce-retention-and-minimum-racket-speed-design.md` — 更正 floor 后落点估算，并写明 drag 模型敏感性。
- Modify: `deploy/envs/hitter.py` — 校验并记录目标位置和速度下限诊断。
- Modify: `deploy/tests/test_hitter_single_shot_lifecycle.py` — retain-only、hard failure 和 override 诊断传播单元测试。
- Modify: `deploy/tests/test_hitter_runtime_single_shot_integration.py` — 反弹后 estimator refill 集成语义，并修正过期 discontinuity 预期。
- Modify: `deploy/tests/test_real_world_v2_consumer.py` — 用生产 RealWorld estimator 触发真实 bounce，验证同 ID 的 ready=False snapshot 到 typed failure 再到 lifecycle retain 的完整路径。
- Create: `deploy/tests/test_hitter_minimum_racket_speed.py` — 两条真机弱解、强解不变、回退值和非法配置测试。
- Create: `deploy/tests/test_hitter_predictor_continuous_contact.py` — 桌沿内接触、桌沿外接触和单次反弹测试。
- Modify: `deploy/tests/test_hitter_strike_target_logging.py` — 新 INFO 诊断字段及非有限值保护测试。

## Task 1: ARMED estimator refill 保留 last-good command

**Files:**

- Modify: `deploy/tests/test_hitter_single_shot_lifecycle.py:474-548`
- Modify: `deploy/tests/test_hitter_runtime_single_shot_integration.py:159-190,505-551`
- Modify: `deploy/tests/test_real_world_v2_consumer.py:1-13,150-174`
- Modify: `deploy/utils/hitter_realtime.py:786-798`

**Interfaces:**

- Consumes: `HitterCommandLifecycle.ingest(result: PlannerResultSnapshot, *, now: float) -> LifecycleDecision`、`LifecycleCancelReason`、`PlannerFailureReason`。
- Produces: ARMED 中 `ESTIMATOR_NOT_READY` 的 `retained_failure` 语义；active command、锁存字段、失败计数和 track consumption 均不改变。

- [ ] **Step 1: 写生命周期失败测试**

在 `test_hitter_single_shot_lifecycle.py` 的 retain-only 测试旁加入：

```python
def test_estimator_not_ready_retains_last_good_without_counting_until_hard_failure():
    item = armed_lifecycle_before_commit()
    active = item.active_result

    for generation in range(2, 7):
        decision = item.ingest(
            failure(
                generation=generation,
                reason=PlannerFailureReason.ESTIMATOR_NOT_READY,
            ),
            now=1.1,
        )
        assert decision.kind == "retained_failure"
        assert (
            decision.cancel_reason
            is LifecycleCancelReason.ESTIMATOR_NOT_READY
        )
        assert item.phase is CommandPhase.ARMED
        assert item.active_result is active
        assert item.consecutive_failure_count == 0
        assert 7 not in item.consumed_track_ids

    decision = item.ingest(
        failure(
            generation=7,
            reason=PlannerFailureReason.BASE_POSE_INVALID,
        ),
        now=1.1,
    )
    assert decision.kind == "cancelled"
    assert decision.cancel_reason is LifecycleCancelReason.BASE_POSE_INVALID
    assert item.phase is CommandPhase.WAITING
    assert item.active_result is None
    assert 7 in item.consumed_track_ids
```

保留现有 `test_only_matching_active_soft_failures_accumulate`；它继续用 `BALL_NOT_INCOMING` 证明第三次失败会撤销。

- [ ] **Step 2: 运行新测试，确认旧实现会在第三次 estimator failure 撤销**

Run:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -p no:cacheprovider deploy/tests/test_hitter_single_shot_lifecycle.py::test_estimator_not_ready_retains_last_good_without_counting_until_hard_failure -q
```

Expected: FAIL；第三次 `ESTIMATOR_NOT_READY` 后 `decision.kind` 为 `cancelled`，而不是 `retained_failure`。

- [ ] **Step 3: 最小修改 failure 分类**

在 `HitterCommandLifecycle` 中将枚举项改为：

```python
    _RETAIN_ONLY_FAILURES = frozenset(
        {
            LifecycleCancelReason.ESTIMATOR_NOT_READY,
            LifecycleCancelReason.HIT_HEIGHT_OUT_OF_RANGE,
            LifecycleCancelReason.OVERRIDE_DISCONTINUITY,
        }
    )
    _SOFT_FAILURES = frozenset(
        {
            LifecycleCancelReason.BALL_NOT_INCOMING,
            LifecycleCancelReason.NO_FUTURE_CROSSING,
        }
    )
```

不修改 `_handle_armed_failure` 的其他分支；WAITING/TRACKING 仍无法用未 ready 的估计生成命令。

- [ ] **Step 4: 运行生命周期聚焦测试**

Run:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -p no:cacheprovider \
  deploy/tests/test_hitter_single_shot_lifecycle.py::test_estimator_not_ready_retains_last_good_without_counting_until_hard_failure \
  deploy/tests/test_hitter_single_shot_lifecycle.py::test_only_matching_active_soft_failures_accumulate \
  deploy/tests/test_hitter_single_shot_lifecycle.py::test_immediate_planner_failure_cancels_precommit \
  deploy/tests/test_hitter_single_shot_lifecycle.py::test_commit_freezes_success_failure_cancel_and_uses_locked_deadline -q
```

Expected: PASS；ESTIMATOR 不计数，`BALL_NOT_INCOMING` 合同不变；hard failure 在 precommit 取消，在 commit window 内继续保留冻结命令。

- [ ] **Step 5: 写 runtime batch 集成测试并修正过期 discontinuity 场景**

在 `test_hitter_runtime_single_shot_integration.py` 加入：

```python
def test_estimator_refill_batch_keeps_active_then_hard_failure_cancels():
    env = make_hitter_env_for_test(is_real=True)
    env.hitter_planner_worker.queue(
        result_batch(
            success_result(generation=1),
            *(
                failure_result(
                    PlannerFailureReason.ESTIMATOR_NOT_READY,
                    generation=generation,
                )
                for generation in range(2, 7)
            ),
        )
    )

    env._update_hitter_command(now=10.0)

    assert env.hitter_command_lifecycle.phase is CommandPhase.ARMED
    assert env.hitter_command_lifecycle.active_result.source_generation == 1
    assert env.hitter_command_lifecycle.consecutive_failure_count == 0
    assert 7 not in env.hitter_command_lifecycle.consumed_track_ids
    assert 7 not in env.simulator.consumed_track_ids

    env.hitter_planner_worker.queue(
        result_batch(
            failure_result(
                PlannerFailureReason.BASE_POSE_INVALID,
                generation=7,
            )
        )
    )
    env._update_hitter_command(now=10.0)

    assert env.hitter_command_lifecycle.phase is CommandPhase.WAITING
    assert env.hitter_command_lifecycle.active_result is None
    assert 7 in env.hitter_command_lifecycle.consumed_track_ids
    assert 7 in env.simulator.consumed_track_ids
```

把 `test_every_consuming_decision_is_synchronized_to_consumer` 的参数改为：

```python
@pytest.mark.parametrize(
    "scenario",
    ["late_skip", "third_soft", "immediate"],
)
```

删除该测试的 `third_discontinuity` else 数据分支，另加 retain-only 集成断言：

```python
def test_repeated_override_discontinuities_do_not_consume_active_track():
    env = make_hitter_env_for_test(is_real=True)
    env.hitter_planner_worker.queue(
        result_batch(
            success_result(generation=1),
            success_result(generation=2, marker=0.20),
            success_result(generation=3, marker=0.20),
            success_result(generation=4, marker=0.20),
        )
    )

    env._update_hitter_command(now=10.0)

    assert env.hitter_command_lifecycle.phase is CommandPhase.ARMED
    assert env.hitter_command_lifecycle.active_result.source_generation == 1
    assert 7 not in env.hitter_command_lifecycle.consumed_track_ids
    assert 7 not in env.simulator.consumed_track_ids
```

- [ ] **Step 6: 写生产 RealWorld bounce-to-retain 路径测试**

在 `deploy/tests/test_real_world_v2_consumer.py` 增加 imports：

```python
from envs.hitter import HitterEnv
from tests.hitter_test_factories import failure, success
from utils.hitter_realtime import (
    CommandPhase,
    HitterCommandLifecycle,
    LifecycleCancelReason,
)
from utils.hitter_runtime_types import PlannerFailureReason, PlannerRejected
```

加入完整路径测试；它必须真实触发 estimator bounce，而不是手造 failure batch：

```python
def test_real_world_bounce_refill_maps_to_armed_retain_only(world):
    world.ball_state_estimator = (
        hitter_runtime_factory.build_ball_state_estimator(
            {
                "state_estimator_window_size": 6,
                "state_estimator_min_samples": 6,
                "table_height": 0.76,
                "ball_radius": 0.02,
            }
        )
    )
    open_session(world)
    ingest_valid_pelvis(world, now=0.49, frame=49)

    for frame, now, position in (
        (10, 0.50, (1.4, 0.0, 0.95)),
        (11, 0.51, (1.4, 0.0, 0.90)),
    ):
        world._ingest_vicon_v2_message(
            ball(track_id=7, frame=frame, position=position),
            received_monotonic_s=now,
        )
    assert world.ball_state_estimator.sample_count == 2

    world._ingest_vicon_v2_message(
        ball(
            track_id=7,
            frame=12,
            position=(1.4, 0.0, 0.79),
        ),
        received_monotonic_s=0.52,
    )
    bounce_snapshot = world.latest_ball_snapshot
    assert world.ball_state_estimator.sample_count == 1
    assert bounce_snapshot.track_id == 7
    assert bounce_snapshot.new_track is False
    assert bounce_snapshot.visible is True
    assert bounce_snapshot.ready is False
    assert bounce_snapshot.consumed is False

    refill_snapshots = [bounce_snapshot]
    for frame, now, position in (
        (13, 0.53, (1.4, 0.0, 0.82)),
        (14, 0.54, (1.4, 0.0, 0.86)),
        (15, 0.55, (1.4, 0.0, 0.89)),
        (16, 0.56, (1.4, 0.0, 0.91)),
    ):
        world._ingest_vicon_v2_message(
            ball(track_id=7, frame=frame, position=position),
            received_monotonic_s=now,
        )
        refill_snapshots.append(world.latest_ball_snapshot)

    assert world.ball_state_estimator.sample_count == 5
    assert [snapshot.track_id for snapshot in refill_snapshots] == [7] * 5
    assert all(snapshot.ready is False for snapshot in refill_snapshots)

    item = HitterCommandLifecycle(
        swing_duration_sampler=lambda: 1.85,
    )
    assert item.ingest(
        success(
            track_id=7,
            generation=1,
            deadline=2.0,
            completed=1.0,
        ),
        now=1.1,
    ).kind == "armed"
    active = item.active_result
    planner_env = HitterEnv.__new__(HitterEnv)

    for snapshot in refill_snapshots:
        with pytest.raises(PlannerRejected) as caught:
            planner_env._plan_hitter_snapshot(snapshot)
        assert (
            caught.value.reason
            is PlannerFailureReason.ESTIMATOR_NOT_READY
        )
        decision = item.ingest(
            failure(
                track_id=7,
                generation=snapshot.generation,
                reason=caught.value.reason,
            ),
            now=1.1,
        )
        assert decision.kind == "retained_failure"
        assert (
            decision.cancel_reason
            is LifecycleCancelReason.ESTIMATOR_NOT_READY
        )
        assert item.phase is CommandPhase.ARMED
        assert item.active_result is active
        assert item.consecutive_failure_count == 0
        assert 7 not in item.consumed_track_ids
        assert 7 not in world.consumed_ball_track_ids
```

这个测试同时锁定四个生产事实：bounce 清窗、track ID 不变、refill snapshot `ready=False`、`HitterEnv._plan_hitter_snapshot` 映射为 `ESTIMATOR_NOT_READY`；连续五个真实 refill snapshot 在 ARMED 中均不消费命令。

- [ ] **Step 7: 运行三套 lifecycle/consumer 测试**

Run:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -p no:cacheprovider \
  deploy/tests/test_hitter_single_shot_lifecycle.py \
  deploy/tests/test_hitter_runtime_single_shot_integration.py \
  deploy/tests/test_real_world_v2_consumer.py::test_real_world_bounce_refill_maps_to_armed_retain_only -q
```

Expected: PASS，且不再存在把第三次 `OVERRIDE_DISCONTINUITY` 当作 consuming decision 的断言。

- [ ] **Step 8: 精确暂存并提交**

```bash
git add deploy/utils/hitter_realtime.py \
  deploy/tests/test_hitter_single_shot_lifecycle.py \
  deploy/tests/test_hitter_runtime_single_shot_integration.py \
  deploy/tests/test_real_world_v2_consumer.py
git diff --cached --check
git diff --cached --stat
git diff --cached -- deploy/utils/hitter_realtime.py \
  deploy/tests/test_hitter_single_shot_lifecycle.py \
  deploy/tests/test_hitter_runtime_single_shot_integration.py \
  deploy/tests/test_real_world_v2_consumer.py
git commit -m "fix: retain armed hitter command during estimator refill"
```

## Task 2: 最低球拍法向速度和配置

**Files:**

- Create: `deploy/tests/test_hitter_minimum_racket_speed.py`
- Modify: `deploy/utils/hitter_planner.py:16-68,300-360,459-497`
- Modify: `deploy/utils/hitter_runtime_factory.py:99-132`
- Modify: `deploy/config/mimic/hitter.yaml:28-33,44-62`
- Modify: `docs/superpowers/specs/2026-08-13-hitter-bounce-retention-and-minimum-racket-speed-design.md:74-82`

**Interfaces:**

- Consumes: `StrikePlanner.racket_velocity_from_ball_velocities(v_ball_in, v_ball_out) -> np.ndarray` 和当前工作树已有的 `_align_racket_velocity_components`。
- Produces: `StrikePlanner(..., minimum_racket_normal_speed_mps: float = 0.0)`；`StrikePlan.raw_racket_normal_speed_mps`、`commanded_racket_normal_speed_mps`、`minimum_racket_normal_speed_mps`、`racket_speed_floor_applied`。
- Diagnostic contract: `commanded_racket_normal_speed_mps` 是分量裁剪前、应用 floor 后的有符号法向标量；最终裁剪后的三维目标继续由 `v_racket_target` 表示。

- [ ] **Step 1: 新建真机弱解和配置失败测试**

创建 `deploy/tests/test_hitter_minimum_racket_speed.py`：

```python
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
from omegaconf import OmegaConf

from utils.hitter_planner import BallTrajectoryPredictor, StrikePlanner
from utils.hitter_runtime_factory import build_hitter_system_planner
from utils.hitter_runtime_types import PlannerFailureReason, PlannerRejected


DEPLOY_DIR = Path(__file__).resolve().parents[1]


class FixedCollisionStrikePlanner(StrikePlanner):
    def __init__(self, v_ball_in, v_ball_out, **kwargs):
        super().__init__(
            predictor=BallTrajectoryPredictor(gravity=[0.0, 0.0, 0.0]),
            **kwargs,
        )
        self._fixed_v_ball_in = np.asarray(v_ball_in, dtype=np.float64)
        self._fixed_v_ball_out = np.asarray(v_ball_out, dtype=np.float64)

    def hit_plane_intersection(self, _ball_position, _ball_velocity):
        return (
            0.5,
            np.array([0.0, 0.1, 1.0], dtype=np.float64),
            self._fixed_v_ball_in.copy(),
        )

    def desired_outgoing_ball_velocity(self, _strike_position):
        return self._fixed_v_ball_out.copy()


WEAK_LOG_CASES = (
    (
        [-2.5882, -0.2554, -3.7658],
        [4.2708, -0.1772, 2.3027],
        0.514781058321,
        [0.748919238787, 0.008538487312, 0.662606269220],
    ),
    (
        [-2.7734, -0.2100, -2.8767],
        [4.2708, -0.2486, 1.0232],
        0.533658861907,
        [0.874860412136, -0.004793959841, 0.484351398497],
    ),
)


@pytest.mark.parametrize(
    "v_ball_in,v_ball_out,expected_raw_speed,expected_velocity",
    WEAK_LOG_CASES,
)
def test_logged_weak_collision_solution_is_floored_to_one_mps(
    v_ball_in,
    v_ball_out,
    expected_raw_speed,
    expected_velocity,
):
    planner = FixedCollisionStrikePlanner(
        v_ball_in,
        v_ball_out,
        minimum_racket_normal_speed_mps=1.0,
    )

    plan = planner.plan([1.0, 0.0, 1.0], [-1.0, 0.0, 0.0])

    assert plan.raw_racket_normal_speed_mps == pytest.approx(
        expected_raw_speed,
        abs=1.0e-10,
    )
    assert plan.commanded_racket_normal_speed_mps == pytest.approx(1.0)
    assert plan.minimum_racket_normal_speed_mps == pytest.approx(1.0)
    assert plan.racket_speed_floor_applied is True
    np.testing.assert_allclose(
        plan.v_racket_target,
        expected_velocity,
        atol=1.0e-10,
    )
    assert 0.0 <= plan.v_racket_target[0] <= 6.0
    assert -0.5 <= plan.v_racket_target[1] <= 0.5
    assert 0.0 <= plan.v_racket_target[2] <= 6.0


@pytest.mark.parametrize(
    "v_ball_in,v_ball_out,expected_touchdown_x",
    [
        (
            [-2.5882, -0.2554, -3.7658],
            [4.2708, -0.1772, 2.3027],
            2.4717,
        ),
        (
            [-2.7734, -0.2100, -2.8767],
            [4.2708, -0.2486, 1.0232],
            2.4008,
        ),
    ],
)
def test_floored_weak_solution_stays_inside_far_edge_in_drag_model(
    v_ball_in,
    v_ball_out,
    expected_touchdown_x,
):
    gravity = np.array([0.0, 0.0, -9.81], dtype=np.float64)
    flight_time = 0.48
    desired_landing = np.array([2.05, 0.0, 0.78], dtype=np.float64)
    predictor = BallTrajectoryPredictor(
        gravity=gravity,
        drag_coefficient=0.098847,
        dt=0.005,
        table_height=0.76,
        table_center_xy=[100.0, 0.0],
        table_length=1.0,
        table_width=1.0,
        ball_radius=0.02,
    )
    planner = StrikePlanner(
        predictor=predictor,
        desired_landing_point=desired_landing,
        post_hit_flight_time=flight_time,
        racket_restitution=0.85,
        minimum_racket_normal_speed_mps=1.0,
    )
    v_in = np.asarray(v_ball_in, dtype=np.float64)
    desired_v_out = np.asarray(v_ball_out, dtype=np.float64)
    strike_position = desired_landing - (
        desired_v_out + 0.5 * gravity * flight_time
    ) * flight_time
    normal = desired_v_out - v_in
    normal /= np.linalg.norm(normal)
    v_racket = planner.racket_velocity_from_ball_velocities(
        v_in,
        desired_v_out,
    )
    actual_v_out = v_in + (1.0 + planner.racket_restitution) * (
        np.dot(v_racket - v_in, normal)
    ) * normal

    trajectory = predictor.predict(
        strike_position,
        actual_v_out,
        horizon_s=1.0,
    )
    positions = trajectory.positions
    descending_crossings = np.flatnonzero(
        (positions[:-1, 2] >= 0.78)
        & (positions[1:, 2] <= 0.78)
        & (trajectory.velocities[:-1, 2] < 0.0)
    )
    assert descending_crossings.size == 1
    index = int(descending_crossings[0])
    fraction = (
        (positions[index, 2] - 0.78)
        / (positions[index, 2] - positions[index + 1, 2])
    )
    touchdown = positions[index] + fraction * (
        positions[index + 1] - positions[index]
    )

    assert touchdown[0] == pytest.approx(expected_touchdown_x, abs=0.005)
    assert 0.0 <= touchdown[0] <= 2.730738
    assert abs(touchdown[1]) <= 0.7562255
    assert touchdown[2] == pytest.approx(0.78)


def test_one_mps_floor_is_not_landing_safe_if_drag_is_removed():
    v_in = np.array(
        [-2.5882, -0.2554, -3.7658],
        dtype=np.float64,
    )
    desired_v_out = np.array(
        [4.2708, -0.1772, 2.3027],
        dtype=np.float64,
    )
    gravity = np.array([0.0, 0.0, -9.81], dtype=np.float64)
    flight_time = 0.48
    desired_landing = np.array([2.05, 0.0, 0.78], dtype=np.float64)
    strike_position = desired_landing - (
        desired_v_out + 0.5 * gravity * flight_time
    ) * flight_time
    planner = StrikePlanner(
        predictor=BallTrajectoryPredictor(
            gravity=gravity,
            drag_coefficient=0.0,
        ),
        racket_restitution=0.85,
        minimum_racket_normal_speed_mps=1.0,
    )
    normal = desired_v_out - v_in
    normal /= np.linalg.norm(normal)
    v_racket = planner.racket_velocity_from_ball_velocities(
        v_in,
        desired_v_out,
    )
    actual_v_out = v_in + (1.0 + planner.racket_restitution) * (
        np.dot(v_racket - v_in, normal)
    ) * normal
    roots = np.roots(
        [
            0.5 * gravity[2],
            actual_v_out[2],
            strike_position[2] - 0.78,
        ]
    )
    touchdown_time = max(
        float(root.real)
        for root in roots
        if abs(root.imag) < 1.0e-12 and root.real > 0.0
    )
    touchdown_x = (
        strike_position[0]
        + actual_v_out[0] * touchdown_time
    )

    assert touchdown_x == pytest.approx(2.9617, abs=0.001)
    assert touchdown_x > 2.730738


def test_floor_does_not_change_solution_already_above_one_mps():
    v_ball_in = np.array([-1.0, 0.0, 0.0], dtype=np.float64)
    v_ball_out = np.array([4.0, 0.0, 0.0], dtype=np.float64)
    baseline = StrikePlanner(minimum_racket_normal_speed_mps=0.0)
    floored = StrikePlanner(minimum_racket_normal_speed_mps=1.0)

    expected = baseline.racket_velocity_from_ball_velocities(
        v_ball_in,
        v_ball_out,
    )
    actual = floored.racket_velocity_from_ball_velocities(
        v_ball_in,
        v_ball_out,
    )

    np.testing.assert_array_equal(actual, expected)
    assert np.linalg.norm(actual) > 1.0


def test_zero_floor_exactly_restores_original_signed_formula():
    v_ball_in = np.array([2.0, 0.0, 0.0], dtype=np.float64)
    v_ball_out = np.array([-1.0, 0.0, 0.0], dtype=np.float64)
    delta = v_ball_out - v_ball_in
    normal = delta / np.linalg.norm(delta)
    expected_scalar = (
        np.dot(v_ball_out, normal)
        + 0.85 * np.dot(v_ball_in, normal)
    ) / 1.85
    planner = StrikePlanner(minimum_racket_normal_speed_mps=0.0)

    actual = planner.racket_velocity_from_ball_velocities(
        v_ball_in,
        v_ball_out,
    )

    np.testing.assert_allclose(actual, expected_scalar * normal, rtol=0.0, atol=0.0)


def test_positive_floor_rejects_undefined_collision_normal():
    planner = StrikePlanner(minimum_racket_normal_speed_mps=1.0)
    velocity = np.array([1.0, -0.2, 0.3], dtype=np.float64)

    with pytest.raises(PlannerRejected) as caught:
        planner.racket_velocity_from_ball_velocities(
            velocity,
            velocity,
        )

    assert caught.value.reason is PlannerFailureReason.INTERNAL_ERROR
    assert "collision normal is undefined" in caught.value.detail


@pytest.mark.parametrize(
    "value,expected_error",
    [
        (True, TypeError),
        (-0.1, ValueError),
        (float("nan"), ValueError),
        (float("inf"), ValueError),
    ],
)
def test_invalid_minimum_normal_speed_fails_fast(value, expected_error):
    with pytest.raises(expected_error, match="minimum_racket_normal_speed_mps"):
        StrikePlanner(minimum_racket_normal_speed_mps=value)
    with pytest.raises(expected_error, match="minimum_racket_normal_speed_mps"):
        build_hitter_system_planner(
            {"minimum_racket_normal_speed_mps": value}
        )


def test_production_yaml_keeps_hit_plane_zero_and_enables_one_mps_floor():
    config = OmegaConf.to_container(
        OmegaConf.load(DEPLOY_DIR / "config/mimic/hitter.yaml"),
        resolve=True,
    )["motion"]["ball_planner"]

    planner = build_hitter_system_planner(config).strike_planner

    assert config["virtual_hit_plane_x"] == pytest.approx(0.0)
    assert config["minimum_racket_normal_speed_mps"] == pytest.approx(1.0)
    assert planner.virtual_hit_plane_x == pytest.approx(0.0)
    assert planner.minimum_racket_normal_speed_mps == pytest.approx(1.0)
```

这里故意同时固定两条事实：生产 drag 模型下两条样本的预测首次落台仍在桌内；去掉 drag 后第一条会越过远端桌沿。验收只能声明“在当前生产模型和这两条记录上通过”，不能把 1.0 m/s floor 宣称为与空气模型无关的落点保证。

- [ ] **Step 2: 运行新测试，确认参数和诊断字段尚不存在**

Run:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -p no:cacheprovider deploy/tests/test_hitter_minimum_racket_speed.py -q
```

Expected: FAIL，首个错误为 `StrikePlanner.__init__()` 不接受 `minimum_racket_normal_speed_mps`，或 `StrikePlan` 缺少诊断字段。

- [ ] **Step 3: 添加严格的非负有限值解析**

在 `deploy/utils/hitter_planner.py` 的向量 helper 旁加入：

```python
def _finite_nonnegative_float(value, name: str) -> float:
    if isinstance(value, (bool, np.bool_)) or not isinstance(
        value,
        (int, float, np.integer, np.floating),
    ):
        raise TypeError(f"{name} must be a real number.")
    result = float(value)
    if not np.isfinite(result) or result < 0.0:
        raise ValueError(f"{name} must be finite and non-negative.")
    return result
```

给 `StrikePlanner.__init__` 增加参数并保存：

```python
        minimum_racket_normal_speed_mps: float = 0.0,
```

```python
        self.minimum_racket_normal_speed_mps = _finite_nonnegative_float(
            minimum_racket_normal_speed_mps,
            "minimum_racket_normal_speed_mps",
        )
```

不要在 runtime factory 中先做 `float(...)`，否则 `True` 会被静默转换成 `1.0`。

- [ ] **Step 4: 提取原始碰撞标量并施加可回退 floor**

将当前公式拆为私有 helper，公共方法保持原签名：

```python
    def _collision_normal_and_raw_racket_speed(
        self,
        v_ball_in: np.ndarray,
        v_ball_out: np.ndarray,
    ) -> tuple[np.ndarray, float]:
        v_in = _vec3(v_ball_in, "v_ball_in", reject_nonfinite=True)
        v_out = _vec3(v_ball_out, "v_ball_out", reject_nonfinite=True)
        delta = v_out - v_in
        norm = float(np.linalg.norm(delta))
        if norm < 1.0e-9:
            return np.zeros(3, dtype=np.float64), 0.0
        normal = delta / norm
        scalar = (
            np.dot(v_out, normal)
            + self.racket_restitution * np.dot(v_in, normal)
        ) / (1.0 + self.racket_restitution)
        return normal, float(scalar)

    def racket_velocity_from_ball_velocities(
        self,
        v_ball_in: np.ndarray,
        v_ball_out: np.ndarray,
    ) -> np.ndarray:
        normal, raw_speed = self._collision_normal_and_raw_racket_speed(
            v_ball_in,
            v_ball_out,
        )
        if not np.any(normal):
            if self.minimum_racket_normal_speed_mps > 0.0:
                raise PlannerRejected(
                    PlannerFailureReason.INTERNAL_ERROR,
                    "collision normal is undefined because "
                    "v_ball_out equals v_ball_in",
                )
            return np.zeros(3, dtype=np.float64)
        commanded_speed = raw_speed
        if self.minimum_racket_normal_speed_mps > 0.0:
            commanded_speed = max(
                raw_speed,
                self.minimum_racket_normal_speed_mps,
            )
        return commanded_speed * normal
```

`minimum_racket_normal_speed_mps == 0.0` 时保留原始有符号标量，不对负值额外夹到零。
`minimum_racket_normal_speed_mps > 0.0` 且碰撞法向退化时不得静默输出零速度；必须 typed reject。只有显式回退值 `0.0` 保留旧版退化向量返回零的兼容行为。

- [ ] **Step 5: 扩展 StrikePlan 并在 plan() 填入裁剪前诊断**

给 dataclass 尾部添加带默认值字段，以免破坏现有测试 factory：

```python
@dataclass(frozen=True)
class StrikePlan:
    t_strike: float
    p_racket_target: np.ndarray
    v_racket_target: np.ndarray
    v_ball_in: np.ndarray
    v_ball_out: np.ndarray
    raw_racket_normal_speed_mps: float = 0.0
    commanded_racket_normal_speed_mps: float = 0.0
    minimum_racket_normal_speed_mps: float = 0.0
    racket_speed_floor_applied: bool = False
```

在 `plan()` 中，保留公共速度方法和当前分量裁剪调用，并填入诊断：

```python
        normal, raw_normal_speed = (
            self._collision_normal_and_raw_racket_speed(v_in, v_out)
        )
        v_racket = self.racket_velocity_from_ball_velocities(v_in, v_out)
        v_racket = _vec3(
            v_racket,
            "v_racket_target",
            reject_nonfinite=True,
        )
        commanded_normal_speed = (
            float(np.dot(v_racket, normal))
            if np.any(normal)
            else 0.0
        )
        floor_applied = bool(
            np.any(normal)
            and self.minimum_racket_normal_speed_mps > 0.0
            and raw_normal_speed
            < self.minimum_racket_normal_speed_mps
        )
        v_racket = self._align_racket_velocity_components(v_racket)
        return StrikePlan(
            t_strike=t_strike,
            p_racket_target=p_strike,
            v_racket_target=v_racket,
            v_ball_in=v_in,
            v_ball_out=v_out,
            raw_racket_normal_speed_mps=raw_normal_speed,
            commanded_racket_normal_speed_mps=commanded_normal_speed,
            minimum_racket_normal_speed_mps=(
                self.minimum_racket_normal_speed_mps
            ),
            racket_speed_floor_applied=floor_applied,
        )
```

这里的 `_align_racket_velocity_components` 及其调用是计划开始前已有的用户改动：工作树中继续保留并验证它，但 Task 2 的 index/commit 必须排除 method、constructor 参数、factory/config 以及这行调用。导出的 staged tree 将按 HEAD 基线直接返回 floor 后向量；当前整合工作树则继续在 floor 后执行用户已有的 component alignment。

- [ ] **Step 6: 连接 factory 和生产 YAML**

在 `build_hitter_system_planner` 的 `StrikePlanner(...)` 参数中加入原始配置值：

```python
        minimum_racket_normal_speed_mps=planner_config.get(
            "minimum_racket_normal_speed_mps",
            0.0,
        ),
```

在 `deploy/config/mimic/hitter.yaml` 的 `ball_planner` 下加入：

```yaml
    minimum_racket_normal_speed_mps: 1.0
```

确认同一段仍是：

```yaml
    virtual_hit_plane_x: 0.0
```

- [ ] **Step 7: 更正设计说明中的落点模型表述**

把设计文档中“固定 `post_hit_flight_time` 时的 x 位移”称为落点的句子替换为：

```markdown
对本次两条弱命令，1.0 m/s floor 预计分别生成约
`[0.749, 0.009, 0.663]` 和 `[0.875, -0.005, 0.484]` m/s，
均在 model17500 的训练范围 `x/z=[0,6]`、`y=[-0.5,0.5]` 内。
按当前生产 drag=0.098847 和 5 ms predictor 反算，首次回到球心接触高度
`z=0.78` 时约为 `x=2.47` 和 `x=2.40`，低于 2.730738 m
远端桌沿；但无阻力闭式模型下第一条约落在 `x=2.96`，因此该落点结论依赖
当前空气模型，必须由自动化测试和真机单球实际落点共同验收。
```

- [ ] **Step 8: 运行最低速度测试和既有 planner/factory 测试**

Run:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -p no:cacheprovider \
  deploy/tests/test_hitter_minimum_racket_speed.py \
  deploy/tests/test_hitter_planner_failure_reasons.py \
  deploy/tests/test_hitter_runtime_factory.py -q
```

Expected: PASS。另运行当前未跟踪的 component alignment 测试，仅作为工作树组合验证，不将该文件加入本任务提交：

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -p no:cacheprovider deploy/tests/test_hitter_planner_velocity_alignment.py -q
```

Expected: PASS；生产范围裁剪仍发生在 floor 之后。

- [ ] **Step 9: 只暂存本任务 hunk**

```bash
git add deploy/tests/test_hitter_minimum_racket_speed.py \
  docs/superpowers/specs/2026-08-13-hitter-bounce-retention-and-minimum-racket-speed-design.md
git add -p deploy/utils/hitter_planner.py
git add -p deploy/utils/hitter_runtime_factory.py
git add -p deploy/config/mimic/hitter.yaml
git diff --cached --check
git diff --cached --stat
git diff --cached -- deploy/utils/hitter_planner.py \
  deploy/utils/hitter_runtime_factory.py \
  deploy/config/mimic/hitter.yaml \
  deploy/tests/test_hitter_minimum_racket_speed.py \
  docs/superpowers/specs/2026-08-13-hitter-bounce-retention-and-minimum-racket-speed-design.md
```

在 staged diff 中必须不存在 `_AXIS_INDEX`、`_component_ranges_mps`、`racket_velocity_component_ranges_mps`、checkpoint、transition、`maximum_hit_height`、站位或 Vicon 的新增/修改。再检查导出的 index 文件：

```bash
if git show :deploy/utils/hitter_planner.py \
  | rg -q "_align_racket_velocity_components|racket_velocity_component_ranges_mps"
then
  printf '%s\n' "staged planner unexpectedly contains user component-alignment work"
  exit 1
fi
```

- [ ] **Step 10: 从 Git index 导出临时干净树并运行测试**

这一步验证 Task 2 的提交自身可运行，而不是只验证含用户 unstaged component-alignment 改动的工作树：

```bash
staged_tree_dir="$(mktemp -d /tmp/robotbridge4-staged-tree.XXXXXX)" || exit 1
staged_tree_dir="$(realpath -e "$staged_tree_dir")" || exit 1
case "$staged_tree_dir" in
  /tmp/robotbridge4-staged-tree.*) ;;
  *) printf '%s\n' "unexpected staged-tree path: $staged_tree_dir"; exit 1 ;;
esac
test -d "$staged_tree_dir" || exit 1
test ! -L "$staged_tree_dir" || exit 1
git checkout-index --all --prefix="${staged_tree_dir:?}/" || exit 1
(
  cd "$staged_tree_dir" || exit 1
  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=deploy \
    /home/loco1/miniconda3/envs/rb/bin/python -m pytest \
    -p no:cacheprovider \
    deploy/tests/test_hitter_minimum_racket_speed.py \
    deploy/tests/test_hitter_planner_failure_reasons.py \
    deploy/tests/test_hitter_runtime_factory.py -q
)
staged_test_status=$?
if [ "$staged_test_status" -ne 0 ]
then
  printf '%s\n' "staged tree kept for inspection: $staged_tree_dir"
  exit "$staged_test_status"
fi
resolved_staged_tree="$(realpath -e "$staged_tree_dir")" || exit 1
case "$resolved_staged_tree" in
  /tmp/robotbridge4-staged-tree.*) ;;
  *) printf '%s\n' "refusing cleanup: $resolved_staged_tree"; exit 1 ;;
esac
test -d "$resolved_staged_tree" || exit 1
test ! -L "$resolved_staged_tree" || exit 1
find "$resolved_staged_tree" -maxdepth 0 -type d -print
rm -rf -- "$resolved_staged_tree"
```

Expected: index 导出的干净树三套测试全部 PASS；只有随后整合工作树的额外测试才依赖未提交的 component alignment。

- [ ] **Step 11: 提交已独立验证的 index**

```bash
git commit -m "feat: enforce minimum hitter racket normal speed"
```

## Task 3: 步长内连续桌面接触

**Files:**

- Create: `deploy/tests/test_hitter_predictor_continuous_contact.py`
- Modify: `deploy/utils/hitter_planner.py:243-297`

**Interfaces:**

- Consumes: `BallTrajectoryPredictor.predict(position, velocity, horizon_s) -> BallTrajectory`、`_inside_table(xy) -> bool`。
- Produces: `_descending_table_contact_time(pos, vel, acc, segment_dt, contact_z) -> float | None`、`_first_table_exit_time(pos, vel, acc, duration) -> float | None`、`_supported_table_state(pos, vel, duration, contact_z) -> tuple[np.ndarray, np.ndarray]`；只依据接触时刻的 `xy` 判定桌面，把同一步长内的第二次微小回桌视为 settling，并在 supported 段离开桌沿后恢复自由飞行。

- [ ] **Step 1: 写桌沿内外对称测试**

创建 `deploy/tests/test_hitter_predictor_continuous_contact.py`：

```python
from __future__ import annotations

import numpy as np

from utils.hitter_planner import BallTrajectoryPredictor


def predictor() -> BallTrajectoryPredictor:
    return BallTrajectoryPredictor(
        gravity=[0.0, 0.0, 0.0],
        drag_coefficient=0.0,
        vertical_restitution=0.8,
        horizontal_restitution=0.5,
        dt=0.01,
        table_height=0.76,
        table_center_xy=[0.5, 0.0],
        table_length=1.0,
        table_width=1.0,
        ball_radius=0.02,
    )


def test_contact_inside_near_edge_bounces_even_when_step_endpoint_is_outside():
    trajectory = predictor().predict(
        position=[0.012, 0.0, 0.785],
        velocity=[-2.0, 0.0, -1.0],
        horizon_s=0.01,
    )

    np.testing.assert_allclose(
        trajectory.positions[1],
        [-0.003, 0.0, 0.784],
        atol=1.0e-12,
    )
    np.testing.assert_allclose(
        trajectory.velocities[1],
        [-1.0, 0.0, 0.8],
        atol=1.0e-12,
    )


def test_contact_outside_near_edge_remains_free_flight():
    trajectory = predictor().predict(
        position=[0.008, 0.0, 0.785],
        velocity=[-2.0, 0.0, -1.0],
        horizon_s=0.01,
    )

    np.testing.assert_allclose(
        trajectory.positions[1],
        [-0.012, 0.0, 0.775],
        atol=1.0e-12,
    )
    np.testing.assert_allclose(
        trajectory.velocities[1],
        [-2.0, 0.0, -1.0],
        atol=1.0e-12,
    )


def test_interior_contact_rebounds_once_and_stays_finite():
    trajectory = predictor().predict(
        position=[0.5, 0.0, 0.785],
        velocity=[0.0, 0.0, -1.0],
        horizon_s=0.02,
    )

    assert np.isfinite(trajectory.positions).all()
    assert np.isfinite(trajectory.velocities).all()
    assert trajectory.velocities[1, 2] == 0.8
    assert trajectory.velocities[2, 2] == 0.8
    assert trajectory.positions[2, 2] > trajectory.positions[1, 2]


def test_upward_micro_arc_returns_and_settles_without_table_penetration():
    item = BallTrajectoryPredictor(
        gravity=[0.0, 0.0, -9.81],
        drag_coefficient=0.0,
        vertical_restitution=0.5,
        horizontal_restitution=0.5,
        dt=0.01,
        table_height=0.76,
        table_center_xy=[0.5, 0.0],
        table_length=1.0,
        table_width=1.0,
        ball_radius=0.02,
    )

    trajectory = item.predict(
        position=[0.5, 0.0, 0.78],
        velocity=[0.0, 0.0, 0.02],
        horizon_s=0.02,
    )

    assert np.isfinite(trajectory.positions).all()
    assert np.isfinite(trajectory.velocities).all()
    assert np.all(trajectory.positions[:, 2] >= 0.78 - 1.0e-12)
    np.testing.assert_allclose(
        trajectory.positions[1:, 2],
        [0.78, 0.78],
        atol=1.0e-12,
    )
    np.testing.assert_allclose(
        trajectory.velocities[1:, 2],
        [0.0, 0.0],
        atol=1.0e-12,
    )


def test_supported_motion_starts_falling_at_exact_table_exit_time():
    item = BallTrajectoryPredictor(
        gravity=[0.0, 0.0, -10.0],
        drag_coefficient=0.0,
        vertical_restitution=0.5,
        horizontal_restitution=0.5,
        dt=0.01,
        table_height=0.76,
        table_center_xy=[0.5, 0.0],
        table_length=1.0,
        table_width=1.0,
        ball_radius=0.02,
    )

    trajectory = item.predict(
        position=[0.999, 0.0, 0.78],
        velocity=[1.0, 0.0, 0.0],
        horizon_s=0.01,
    )

    np.testing.assert_allclose(
        trajectory.positions[1],
        [1.009, 0.0, 0.779595],
        atol=1.0e-12,
    )
    np.testing.assert_allclose(
        trajectory.velocities[1],
        [1.0, 0.0, -0.09],
        atol=1.0e-12,
    )
```

- [ ] **Step 2: 运行桌沿测试，确认旧 endpoint 判定漏掉第一条**

Run:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -p no:cacheprovider deploy/tests/test_hitter_predictor_continuous_contact.py -q
```

Expected: 第一条、非零重力 settling 和 supported 出桌测试 FAIL；旧实现第一条得到自由飞行终点 `[-0.008, 0.0, 0.775]` 和速度 `[-2.0, 0.0, -1.0]`，微反弹测试会落到桌面以下，朴素 supported 实现则会把越过桌沿的球错误固定在 `z=0.78`。

- [ ] **Step 3: 加入下降穿越的精确接触时刻求解**

在 `BallTrajectoryPredictor` 中加入：

```python
    def _descending_table_contact_time(
        self,
        pos: np.ndarray,
        vel: np.ndarray,
        acc: np.ndarray,
        segment_dt: float,
        contact_z: float,
    ) -> float | None:
        time_tolerance = 1.0e-12
        velocity_tolerance = 1.0e-10
        if (
            segment_dt <= 0.0
            or pos[2] < contact_z - 1.0e-10
        ):
            return None

        height = float(pos[2] - contact_z)
        velocity_z = float(vel[2])
        acceleration_z = float(acc[2])
        if abs(acceleration_z) < 1.0e-12:
            if velocity_z == 0.0:
                return None
            roots = (-height / velocity_z,)
        else:
            discriminant = (
                velocity_z * velocity_z
                - 2.0 * acceleration_z * height
            )
            if discriminant < -time_tolerance:
                return None
            root = float(np.sqrt(max(discriminant, 0.0)))
            roots = (
                (-velocity_z - root) / acceleration_z,
                (-velocity_z + root) / acceleration_z,
            )

        valid = []
        for value in roots:
            if not (
                -time_tolerance
                <= value
                <= segment_dt + time_tolerance
            ):
                continue
            contact_time = float(np.clip(value, 0.0, segment_dt))
            contact_velocity_z = (
                velocity_z + acceleration_z * contact_time
            )
            if contact_velocity_z < -velocity_tolerance:
                valid.append(contact_time)
        return min(valid) if valid else None
```

这里不能用 `vel[2] < 0` 或 `next_pos[2] <= contact_z` 做提前返回；步首向上的短弧线也可能在同一步内产生合法下降根。

- [ ] **Step 4: 加入 supported 段的连续出桌时刻**

在 `BallTrajectoryPredictor` 中加入：

```python
    def _first_table_exit_time(
        self,
        pos: np.ndarray,
        vel: np.ndarray,
        acc: np.ndarray,
        duration: float,
    ) -> float | None:
        half_extent = np.array(
            [0.5 * self.table_length, 0.5 * self.table_width],
            dtype=np.float64,
        )
        lower = self.table_center_xy - half_extent
        upper = self.table_center_xy + half_extent
        time_tolerance = 1.0e-12
        velocity_tolerance = 1.0e-10
        candidates = []
        for axis in (0, 1):
            for boundary, outward_sign in (
                (lower[axis], -1.0),
                (upper[axis], 1.0),
            ):
                coefficient_a = 0.5 * float(acc[axis])
                coefficient_b = float(vel[axis])
                coefficient_c = float(pos[axis] - boundary)
                if abs(coefficient_a) < 1.0e-12:
                    if abs(coefficient_b) < 1.0e-12:
                        roots = ()
                    else:
                        roots = (-coefficient_c / coefficient_b,)
                else:
                    discriminant = (
                        coefficient_b * coefficient_b
                        - 4.0 * coefficient_a * coefficient_c
                    )
                    if discriminant < -time_tolerance:
                        roots = ()
                    else:
                        root = float(
                            np.sqrt(max(discriminant, 0.0))
                        )
                        roots = (
                            (-coefficient_b - root)
                            / (2.0 * coefficient_a),
                            (-coefficient_b + root)
                            / (2.0 * coefficient_a),
                        )
                for value in roots:
                    if not (
                        -time_tolerance
                        <= value
                        <= duration + time_tolerance
                    ):
                        continue
                    exit_time = float(
                        np.clip(value, 0.0, duration)
                    )
                    exit_velocity = (
                        vel[axis] + acc[axis] * exit_time
                    )
                    if (
                        outward_sign * exit_velocity
                        > velocity_tolerance
                    ):
                        candidates.append(exit_time)
        return min(candidates) if candidates else None
```

该 helper 只接受在边界处速度朝桌外的根；触及边界后又返回桌内的切点不能被误判为出桌。

- [ ] **Step 5: 加入微反弹 settling helper**

在 `BallTrajectoryPredictor` 中加入：

```python
    def _supported_table_state(
        self,
        pos: np.ndarray,
        vel: np.ndarray,
        duration: float,
        contact_z: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        supported_vel = vel.copy()
        supported_vel[2] = 0.0
        supported_acc = (
            -self.drag_coefficient
            * np.linalg.norm(supported_vel)
            * supported_vel
        )
        supported_acc[2] = 0.0
        exit_time = self._first_table_exit_time(
            pos,
            supported_vel,
            supported_acc,
            duration,
        )
        if exit_time is not None:
            exit_pos = (
                pos
                + supported_vel * exit_time
                + 0.5 * supported_acc * exit_time * exit_time
            )
            exit_pos[2] = contact_z
            exit_vel = supported_vel + supported_acc * exit_time
            exit_vel[2] = 0.0
            free_duration = duration - exit_time
            free_acc = (
                self.gravity
                - self.drag_coefficient
                * np.linalg.norm(exit_vel)
                * exit_vel
            )
            return (
                exit_pos
                + exit_vel * free_duration
                + 0.5 * free_acc * free_duration * free_duration,
                exit_vel + free_acc * free_duration,
            )
        next_pos = (
            pos
            + supported_vel * duration
            + 0.5 * supported_acc * duration * duration
        )
        next_vel = supported_vel + supported_acc * duration
        next_pos[2] = contact_z
        next_vel[2] = 0.0
        return next_pos, next_vel
```

该 helper 只用于球已经位于桌内接触面且垂直运动在当前 5 ms 分辨率内完成第二次回桌的情况；不是把任意桌外低球吸附到桌面。若 supported 水平段在剩余时间内越过矩形边界，先精确积分到边界，再用 gravity/drag 自由飞行剩余时间。

- [ ] **Step 6: 在接触点施加 restitution 并积分剩余步长**

用以下块替换 `predict()` 中基于 `next_pos[:2]` 的反弹块：

```python
            resting_on_table = bool(
                abs(pos[2] - contact_z) <= 1.0e-10
                and abs(vel[2]) <= 1.0e-10
                and self._inside_table(pos[:2])
            )
            if resting_on_table:
                pos, vel = self._supported_table_state(
                    pos,
                    vel,
                    self.dt,
                    contact_z,
                )
                continue

            contact_dt = self._descending_table_contact_time(
                pos,
                vel,
                acc,
                self.dt,
                contact_z,
            )
            if contact_dt is not None:
                contact_pos = (
                    pos
                    + vel * contact_dt
                    + 0.5 * acc * contact_dt * contact_dt
                )
                contact_pos[2] = contact_z
                if self._inside_table(contact_pos[:2]):
                    contact_vel = vel + acc * contact_dt
                    rebound_vel = contact_vel.copy()
                    rebound_vel[:2] *= self.horizontal_restitution
                    rebound_vel[2] = (
                        -rebound_vel[2] * self.vertical_restitution
                    )
                    remaining_dt = self.dt - contact_dt
                    rebound_acc = (
                        self.gravity
                        - self.drag_coefficient
                        * np.linalg.norm(rebound_vel)
                        * rebound_vel
                    )
                    second_contact_dt = (
                        self._descending_table_contact_time(
                            contact_pos,
                            rebound_vel,
                            rebound_acc,
                            remaining_dt,
                            contact_z,
                        )
                    )
                    if (
                        rebound_vel[2] <= 1.0e-10
                        or second_contact_dt is not None
                    ):
                        settle_pos = contact_pos
                        settle_vel = rebound_vel
                        settle_remaining_dt = remaining_dt
                        if second_contact_dt is not None:
                            candidate_settle_pos = (
                                contact_pos
                                + rebound_vel * second_contact_dt
                                + 0.5
                                * rebound_acc
                                * second_contact_dt
                                * second_contact_dt
                            )
                            candidate_settle_pos[2] = contact_z
                            if self._inside_table(
                                candidate_settle_pos[:2]
                            ):
                                settle_pos = candidate_settle_pos
                                settle_vel = (
                                    rebound_vel
                                    + rebound_acc * second_contact_dt
                                )
                                settle_vel = settle_vel.copy()
                                settle_vel[:2] *= (
                                    self.horizontal_restitution
                                )
                                settle_remaining_dt -= second_contact_dt
                            else:
                                second_contact_dt = None
                        if (
                            rebound_vel[2] <= 1.0e-10
                            or second_contact_dt is not None
                        ):
                            next_pos, next_vel = (
                                self._supported_table_state(
                                    settle_pos,
                                    settle_vel,
                                    settle_remaining_dt,
                                    contact_z,
                                )
                            )
                        else:
                            next_pos = (
                                contact_pos
                                + rebound_vel * remaining_dt
                                + 0.5
                                * rebound_acc
                                * remaining_dt
                                * remaining_dt
                            )
                            next_vel = (
                                rebound_vel
                                + rebound_acc * remaining_dt
                            )
                    else:
                        next_pos = (
                            contact_pos
                            + rebound_vel * remaining_dt
                            + 0.5
                            * rebound_acc
                            * remaining_dt
                            * remaining_dt
                        )
                        next_vel = (
                            rebound_vel
                            + rebound_acc * remaining_dt
                        )
```

若 `contact_pos[:2]` 在桌外，不改变前面已算好的自由飞行 `next_pos/next_vel`。

- [ ] **Step 7: 运行 predictor 和 planner 回归测试**

Run:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -p no:cacheprovider \
  deploy/tests/test_hitter_predictor_continuous_contact.py \
  deploy/tests/test_hitter_planner_failure_reasons.py \
  deploy/tests/test_hitter_minimum_racket_speed.py -q
```

Expected: PASS；桌沿内发生一次反弹，桌沿外保持自由飞行，上升后同一步回桌的微反弹稳定为 supported state 且下一步不穿透，supported 球越过桌沿后立即恢复下落，最低速度计算不受影响。

- [ ] **Step 8: 精确暂存 predictor hunk 并提交**

```bash
git add deploy/tests/test_hitter_predictor_continuous_contact.py
git add -p deploy/utils/hitter_planner.py
git diff --cached --check
git diff --cached --stat
git diff --cached -- deploy/utils/hitter_planner.py \
  deploy/tests/test_hitter_predictor_continuous_contact.py
```

staged planner diff 只能包含 `BallTrajectoryPredictor` 的接触时刻和积分改动；确认后：

```bash
git commit -m "fix: resolve hitter table contact within prediction step"
```

## Task 4: 目标速度诊断、冻结和 override 传播

**Files:**

- Modify: `deploy/tests/test_hitter_strike_target_logging.py:38-67,113-164`
- Modify: `deploy/tests/test_hitter_single_shot_lifecycle.py:88-128,358-418`
- Modify: `deploy/utils/hitter_realtime.py:956-1041,1337-1351`
- Modify: `deploy/envs/hitter.py:902-1004`

**Interfaces:**

- Consumes: Task 2 的四个 `StrikePlan` 诊断字段。
- Produces: lifecycle 对这些标量和布尔值的冻结及关系校验；连续 override 将候选 `v_ball_in/v_ball_out` 和四个速度诊断作为同一诊断快照传播，同时继续锁住 base、side、deadline 和 `t_strike`；INFO 日志输出内部一致的 planner target，明确不是实测。

- [ ] **Step 1: 写连续 override 的诊断传播测试**

给 `override_result` 增加可选参数：

```python
    plan_diagnostics: dict | None = None,
```

在创建 `new_plan` 后应用它：

```python
    if plan_diagnostics is not None:
        new_plan = replace(new_plan, **plan_diagnostics)
```

将 `test_continuous_override_only_updates_position_and_velocity` 改名为 `test_continuous_override_updates_control_targets_and_speed_diagnostics`，调用时传入：

```python
        plan_diagnostics={
            "raw_racket_normal_speed_mps": 0.52,
            "commanded_racket_normal_speed_mps": 1.0,
            "minimum_racket_normal_speed_mps": 1.0,
            "racket_speed_floor_applied": True,
        },
```

保留现有 base、side、deadline 和 `t_strike` 锁存断言。把原先要求 `v_ball_in/v_ball_out` 保留旧值的断言替换为候选诊断快照：

```python
    plan = active.command.strike_plan
    np.testing.assert_array_equal(
        plan.v_ball_in,
        [91.0, 92.0, 93.0],
    )
    np.testing.assert_array_equal(
        plan.v_ball_out,
        [94.0, 95.0, 96.0],
    )
    assert plan.raw_racket_normal_speed_mps == 0.52
    assert plan.commanded_racket_normal_speed_mps == 1.0
    assert plan.minimum_racket_normal_speed_mps == 1.0
    assert plan.racket_speed_floor_applied is True
```

同时加入 malformed planner output 测试：

```python
@pytest.mark.parametrize(
    "plan_changes",
    [
        {"raw_racket_normal_speed_mps": float("nan")},
        {"commanded_racket_normal_speed_mps": float("inf")},
        {"minimum_racket_normal_speed_mps": -0.1},
        {"racket_speed_floor_applied": 1},
        {
            "raw_racket_normal_speed_mps": 0.52,
            "commanded_racket_normal_speed_mps": 0.2,
            "minimum_racket_normal_speed_mps": 1.0,
            "racket_speed_floor_applied": True,
        },
        {
            "raw_racket_normal_speed_mps": 0.52,
            "commanded_racket_normal_speed_mps": 1.0,
            "minimum_racket_normal_speed_mps": 1.0,
            "racket_speed_floor_applied": False,
        },
        {
            "raw_racket_normal_speed_mps": 1.2,
            "commanded_racket_normal_speed_mps": 1.0,
            "minimum_racket_normal_speed_mps": 1.0,
            "racket_speed_floor_applied": False,
        },
    ],
)
def test_invalid_racket_speed_diagnostics_cancel_before_arm(plan_changes):
    planned_command = command()
    malformed_plan = replace(
        planned_command.strike_plan,
        **plan_changes,
    )
    malformed_command = replace(
        planned_command,
        strike_plan=malformed_plan,
    )
    item = lifecycle()

    decision = item.ingest(
        success(planned_command=malformed_command),
        now=1.1,
    )

    assert decision.kind == "cancelled"
    assert decision.cancel_reason is LifecycleCancelReason.INTERNAL_ERROR
    assert item.phase is CommandPhase.WAITING
    assert item.active_result is None
    assert 7 in item.consumed_track_ids
```

- [ ] **Step 2: 写日志 payload 测试**

给 `fake_command` 增加参数：

```python
    raw_normal_speed: float = 0.5148,
    commanded_normal_speed: float = 1.0,
    minimum_normal_speed: float = 1.0,
    floor_applied: bool = True,
```

创建 `StrikePlan` 时填入：

```python
        raw_racket_normal_speed_mps=raw_normal_speed,
        commanded_racket_normal_speed_mps=commanded_normal_speed,
        minimum_racket_normal_speed_mps=minimum_normal_speed,
        racket_speed_floor_applied=floor_applied,
```

在 `test_logs_all_world_velocity_vectors_and_norms` 追加：

```python
        self.assertIn(
            "p_racket_target_w_m=[0.0000,0.1000,1.0000]",
            message,
        )
        self.assertIn(
            "raw_racket_normal_speed_mps=0.5148 "
            "commanded_racket_normal_speed_mps=1.0000 "
            "minimum_racket_normal_speed_mps=1.0000 "
            "racket_speed_floor_applied=true",
            message,
        )
        self.assertIn("command_kind=planner_target_not_measured", message)
```

再加入 direct logger 防护测试：

```python
    def test_nonfinite_speed_diagnostic_warns_without_target_log(self):
        env = minimal_env()
        command = fake_command(raw_normal_speed=float("nan"))

        with captured_log_messages() as messages:
            env._log_hitter_strike_target(
                planner_result(command=command)
            )

        self.assertFalse(
            any(
                message.startswith("HITTER strike target:")
                for message in messages
            )
        )
        warnings = [
            message
            for message in messages
            if message.startswith("Failed to log HITTER strike target:")
        ]
        self.assertEqual(len(warnings), 1)
        self.assertIn("raw_racket_normal_speed_mps", warnings[0])

    def test_inconsistent_speed_diagnostics_warn_without_target_log(self):
        env = minimal_env()
        command = fake_command(
            raw_normal_speed=0.5148,
            commanded_normal_speed=0.2,
            minimum_normal_speed=1.0,
            floor_applied=True,
        )

        with captured_log_messages() as messages:
            env._log_hitter_strike_target(
                planner_result(command=command)
            )

        self.assertFalse(
            any(
                message.startswith("HITTER strike target:")
                for message in messages
            )
        )
        warnings = [
            message
            for message in messages
            if message.startswith("Failed to log HITTER strike target:")
        ]
        self.assertEqual(len(warnings), 1)
        self.assertIn("speed diagnostics are inconsistent", warnings[0])
```

- [ ] **Step 3: 运行五组新断言，确认 metadata 尚未传播、校验和记录**

Run:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -p no:cacheprovider \
  deploy/tests/test_hitter_single_shot_lifecycle.py::test_continuous_override_updates_control_targets_and_speed_diagnostics \
  deploy/tests/test_hitter_single_shot_lifecycle.py::test_invalid_racket_speed_diagnostics_cancel_before_arm \
  deploy/tests/test_hitter_strike_target_logging.py::StrikeTargetPayloadTests::test_logs_all_world_velocity_vectors_and_norms \
  deploy/tests/test_hitter_strike_target_logging.py::StrikeTargetPayloadTests::test_nonfinite_speed_diagnostic_warns_without_target_log \
  deploy/tests/test_hitter_strike_target_logging.py::StrikeTargetPayloadTests::test_inconsistent_speed_diagnostics_warn_without_target_log -q
```

Expected: FAIL；override 后仍混用旧球速与新 metadata，malformed/inconsistent metadata 未被拒绝，日志也没有新字段。

- [ ] **Step 4: lifecycle 冻结时严格校验诊断值**

在 `_freeze_success_result` 中读取：

```python
        raw_normal_speed = self._finite_number(
            plan.raw_racket_normal_speed_mps,
            "strike_plan.raw_racket_normal_speed_mps",
        )
        commanded_normal_speed = self._finite_number(
            plan.commanded_racket_normal_speed_mps,
            "strike_plan.commanded_racket_normal_speed_mps",
        )
        minimum_normal_speed = self._finite_number(
            plan.minimum_racket_normal_speed_mps,
            "strike_plan.minimum_racket_normal_speed_mps",
        )
        if minimum_normal_speed < 0.0:
            raise ValueError(
                "strike_plan.minimum_racket_normal_speed_mps "
                "must be non-negative"
            )
        floor_applied = plan.racket_speed_floor_applied
        if type(floor_applied) is not bool:
            raise TypeError(
                "strike_plan.racket_speed_floor_applied must be bool"
            )
        expected_commanded_speed = (
            max(raw_normal_speed, minimum_normal_speed)
            if minimum_normal_speed > 0.0
            else raw_normal_speed
        )
        expected_floor_applied = bool(
            minimum_normal_speed > 0.0
            and raw_normal_speed < minimum_normal_speed
        )
        if not np.isclose(
            commanded_normal_speed,
            expected_commanded_speed,
            rtol=1.0e-9,
            atol=1.0e-9,
        ) or floor_applied is not expected_floor_applied:
            raise ValueError(
                "strike_plan racket speed diagnostics are inconsistent"
            )
```

在 `frozen_plan = replace(...)` 中显式写回标准化值：

```python
            raw_racket_normal_speed_mps=raw_normal_speed,
            commanded_racket_normal_speed_mps=commanded_normal_speed,
            minimum_racket_normal_speed_mps=minimum_normal_speed,
            racket_speed_floor_applied=floor_applied,
```

- [ ] **Step 5: 连续 override 复制候选诊断字段**

保留 `active_command.strike_plan` 作为 locked plan 基础，将候选球速和四个速度字段作为不可拆分的诊断快照复制：

```python
        candidate_plan = candidate_command.strike_plan
        locked_plan = replace(
            active_command.strike_plan,
            p_racket_target=new_position,
            v_racket_target=new_velocity,
            v_ball_in=candidate_plan.v_ball_in,
            v_ball_out=candidate_plan.v_ball_out,
            raw_racket_normal_speed_mps=(
                candidate_plan.raw_racket_normal_speed_mps
            ),
            commanded_racket_normal_speed_mps=(
                candidate_plan.commanded_racket_normal_speed_mps
            ),
            minimum_racket_normal_speed_mps=(
                candidate_plan.minimum_racket_normal_speed_mps
            ),
            racket_speed_floor_applied=(
                candidate_plan.racket_speed_floor_applied
            ),
        )
```

不得解锁 `t_strike`、base target、side 或 deadline。`v_ball_in/v_ball_out` 不是控制锁存字段；它们必须与同一 candidate generation 的 raw/commanded/minimum/floor metadata 一起更新，禁止混合新旧计划。

- [ ] **Step 6: 在 HitterEnv 校验并记录 target-only 诊断**

在 `_validated_hitter_command_fields` 中加入：

```python
        speed_diagnostics = {}
        for name in (
            "raw_racket_normal_speed_mps",
            "commanded_racket_normal_speed_mps",
            "minimum_racket_normal_speed_mps",
        ):
            raw_value = getattr(strike_plan, name)
            if isinstance(raw_value, (bool, np.bool_)) or not isinstance(
                raw_value,
                (int, float, np.integer, np.floating),
            ):
                raise TypeError(
                    f"HITTER strike_plan.{name} must be a real number."
                )
            value = float(raw_value)
            if not np.isfinite(value):
                raise ValueError(f"HITTER strike_plan.{name} must be finite.")
            speed_diagnostics[name] = value
        if speed_diagnostics["minimum_racket_normal_speed_mps"] < 0.0:
            raise ValueError(
                "HITTER strike_plan.minimum_racket_normal_speed_mps "
                "must be non-negative."
            )
        floor_applied = strike_plan.racket_speed_floor_applied
        if type(floor_applied) is not bool:
            raise TypeError(
                "HITTER strike_plan.racket_speed_floor_applied must be bool."
            )
        raw_normal_speed = speed_diagnostics[
            "raw_racket_normal_speed_mps"
        ]
        commanded_normal_speed = speed_diagnostics[
            "commanded_racket_normal_speed_mps"
        ]
        minimum_normal_speed = speed_diagnostics[
            "minimum_racket_normal_speed_mps"
        ]
        expected_commanded_speed = (
            max(raw_normal_speed, minimum_normal_speed)
            if minimum_normal_speed > 0.0
            else raw_normal_speed
        )
        expected_floor_applied = bool(
            minimum_normal_speed > 0.0
            and raw_normal_speed < minimum_normal_speed
        )
        if not np.isclose(
            commanded_normal_speed,
            expected_commanded_speed,
            rtol=1.0e-9,
            atol=1.0e-9,
        ) or floor_applied is not expected_floor_applied:
            raise ValueError(
                "HITTER strike_plan racket speed diagnostics "
                "are inconsistent."
            )
```

在现有返回字典中加入：

```python
            "raw_racket_normal_speed_mps": speed_diagnostics[
                "raw_racket_normal_speed_mps"
            ],
            "commanded_racket_normal_speed_mps": speed_diagnostics[
                "commanded_racket_normal_speed_mps"
            ],
            "minimum_racket_normal_speed_mps": speed_diagnostics[
                "minimum_racket_normal_speed_mps"
            ],
            "racket_speed_floor_applied": floor_applied,
```

扩展 `logger.info` 格式串和参数：

```python
                "p_racket_target_w_m={} "
                "raw_racket_normal_speed_mps={:.4f} "
                "commanded_racket_normal_speed_mps={:.4f} "
                "minimum_racket_normal_speed_mps={:.4f} "
                "racket_speed_floor_applied={} "
                "command_kind=planner_target_not_measured "
```

对应参数为：

```python
                self._format_hitter_velocity_log_vector(
                    fields["racket_target"]
                ),
                fields["raw_racket_normal_speed_mps"],
                fields["commanded_racket_normal_speed_mps"],
                fields["minimum_racket_normal_speed_mps"],
                str(fields["racket_speed_floor_applied"]).lower(),
```

这些字段描述 planner target；不要在日志文案中称其为 measured、actual 或 FK racket speed。

- [ ] **Step 7: 运行 lifecycle 和 logging 测试**

Run:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -p no:cacheprovider \
  deploy/tests/test_hitter_single_shot_lifecycle.py \
  deploy/tests/test_hitter_strike_target_logging.py -q
```

Expected: PASS；连续 override 保留控制锁存合同并更新四个诊断字段，日志明确是未测量的 planner target。

- [ ] **Step 8: 暂存并提交诊断改动**

```bash
git add deploy/utils/hitter_realtime.py \
  deploy/envs/hitter.py \
  deploy/tests/test_hitter_single_shot_lifecycle.py \
  deploy/tests/test_hitter_strike_target_logging.py
git diff --cached --check
git diff --cached --stat
git diff --cached -- deploy/utils/hitter_realtime.py \
  deploy/envs/hitter.py \
  deploy/tests/test_hitter_single_shot_lifecycle.py \
  deploy/tests/test_hitter_strike_target_logging.py
git commit -m "feat: log hitter racket speed floor diagnostics"
```

## Task 5: 软件回归、配置验收和真机前交接

**Files:**

- Verify only: `deploy/utils/hitter_realtime.py`
- Verify only: `deploy/utils/hitter_planner.py`
- Verify only: `deploy/utils/hitter_runtime_factory.py`
- Verify only: `deploy/config/mimic/hitter.yaml`
- Verify only: 本计划涉及的测试文件

**Interfaces:**

- Consumes: Tasks 1–4 的完整工作树结果。
- Produces: 可复查的自动化测试结果、resolved 配置值和 staged/unstaged 边界证明；不自动启动真机。

- [ ] **Step 1: 运行全部聚焦回归**

Run:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -p no:cacheprovider \
  deploy/tests/test_hitter_single_shot_lifecycle.py \
  deploy/tests/test_hitter_runtime_single_shot_integration.py \
  deploy/tests/test_hitter_minimum_racket_speed.py \
  deploy/tests/test_hitter_predictor_continuous_contact.py \
  deploy/tests/test_hitter_strike_target_logging.py \
  deploy/tests/test_hitter_planner_failure_reasons.py \
  deploy/tests/test_hitter_runtime_factory.py \
  deploy/tests/test_hitter_completed_result_queue.py \
  deploy/tests/test_hitter_task_pipeline.py -q
```

Expected: 全部 PASS。若出现失败，先区分本计划新增失败与计划开始前 dirty worktree 的既有失败；不得通过回退用户文件消除失败。

- [ ] **Step 2: 单独验证当前工作树已有的分量裁剪组合**

Run:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -p no:cacheprovider deploy/tests/test_hitter_planner_velocity_alignment.py -q
```

Expected: PASS。该未跟踪测试文件仍保持未跟踪，不加入本计划任何提交。

- [ ] **Step 3: 解析生产 YAML 并断言关键值**

Run:

```bash
cd deploy
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python - <<'PY'
from pathlib import Path
from omegaconf import OmegaConf
from utils.hitter_runtime_factory import build_hitter_system_planner

config = OmegaConf.to_container(
    OmegaConf.load(Path("config/mimic/hitter.yaml")),
    resolve=True,
)["motion"]["ball_planner"]
planner = build_hitter_system_planner(config).strike_planner
assert config["virtual_hit_plane_x"] == 0.0
assert config["minimum_racket_normal_speed_mps"] == 1.0
assert planner.virtual_hit_plane_x == 0.0
assert planner.minimum_racket_normal_speed_mps == 1.0
print("virtual_hit_plane_x=0.0")
print("minimum_racket_normal_speed_mps=1.0")
PY
cd ..
```

Expected:

```text
virtual_hit_plane_x=0.0
minimum_racket_normal_speed_mps=1.0
```

- [ ] **Step 4: 检查提交范围和 dirty worktree 保留情况**

Run:

```bash
git diff --check
git status --short
git log -4 --oneline
git show --stat --oneline HEAD~4..HEAD
```

Expected:

- 四个功能提交依次覆盖 estimator retain、minimum speed、continuous contact 和 diagnostics。
- 用户原有 dirty/untracked 文件仍存在；没有被提交、删除或覆盖。
- `git diff --check` 不报告本计划引入的 whitespace error。

- [ ] **Step 5: 真机前只交付验收条件，不自动运行**

在用户明确准备好机器人、Vicon、底层 `trans` 和急停后，从 `deploy` 启动既有真机入口：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4/deploy
LOGURU_LEVEL=INFO PYTHONUNBUFFERED=1 conda run --no-capture-output -n rb \
  python run.py --config-name=hitter device=cpu
```

现场验收必须同时满足：

- 同一 track ID 在 bounce 后连续 `ESTIMATOR_NOT_READY` 时仍保持 `ARMED`，且没有因此进入 `WAITING`。
- `BALL_NOT_INCOMING`/`NO_FUTURE_CROSSING` 三次累计以及 precommit hard failure 仍能撤销；commit window 内 hard failure 继续返回 `retained_committed`。
- INFO 行显示 `virtual_hit_plane_x=0.0` 对应的目标位置、`raw_racket_normal_speed_mps`、`commanded_racket_normal_speed_mps`、minimum 和 floor flag。
- 弱解的裁剪前 commanded normal speed 为 1.0 m/s；最终三维目标仍满足当前 component ranges。
- 每个 track ID 最多一次 `ARMED -> RECOVERY`。
- 记录实际落点；不要把 `command_kind=planner_target_not_measured` 当作实际球拍速度或触球证明。

若真机回球过深，软件回退只需把：

```yaml
minimum_racket_normal_speed_mps: 0.0
```

生命周期回退则只恢复 `ESTIMATOR_NOT_READY` 的分类；不得删除 hard failure 或通用 `cancel()`。
