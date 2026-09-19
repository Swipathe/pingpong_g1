# HITTER 单次击球期望速度日志实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 每个 HITTER 规划击球周期在 strike deadline 处只记录一条最终期望速度日志，包含入球、出球和球拍速度向量及模长。

**Architecture:** 直接复用 `HitterCommandLifecycle` 在 `advance()` 前暴露的 `previous_active`，不增加第二份 command 缓存。新增一个旁路日志辅助方法负责诊断校验、复制、格式化和异常吞吐，再在现有 `crossed_strike` 分支调用一次。

**Tech Stack:** Python 3.8、NumPy、Loguru、`unittest`、RobotBridge2 HITTER lifecycle。

## Global Constraints

- 只修改当前工作区，不重置、恢复或覆盖任何已有 dirty change。
- 不恢复当前已删除的 `deploy/tests/__init__.py`、`test_hitter_env_lifecycle.py` 或其他旧测试。
- 不修改 observation 内容、顺序或 104 维形状。
- 不修改 planner 数学、ONNX action、command 接受规则、生命周期状态转移、LCM 或 R2 流程。
- 所有速度向量均为桌面世界坐标系，单位为 `m/s`。
- `v_ball_out` 只做日志诊断校验，禁止加入 command ingestion 的主验收条件。
- 每个 strike 边界只允许一条 `HITTER strike target:`。
- 诊断失败只能尽力写 WARNING，不得向控制循环传播异常。
- Loguru sink 为同步写入；接受每球一条 INFO 带来的很小但非零 I/O 成本。
- Git 只显式暂存本计划列出的文件，禁止 `git add .` 或 `git add -A`。

---

## File Map

- Create: `deploy/tests/test_hitter_strike_target_logging.py`
  - 独立、无真机依赖的日志内容和生命周期边界测试。
- Modify: `deploy/envs/hitter.py`
  - 新增速度向量格式化和单次 strike target 日志辅助方法。
  - 在现有 `crossed_strike` 分支调用该辅助方法。
- Reference only: `deploy/utils/hitter_realtime.py`
  - `PlannerResultSnapshot`、`HitterCommandLifecycle` 和 final override 语义；不修改。
- Reference only: `docs/superpowers/specs/2026-07-25-hitter-strike-target-logging-design.md`
  - 已批准设计；不修改。

---

### Task 1: 实现旁路 strike target 日志内容

**Files:**
- Create: `deploy/tests/test_hitter_strike_target_logging.py`
- Modify: `deploy/envs/hitter.py:610-681`

**Interfaces:**
- Consumes: `PlannerResultSnapshot`，字段为 `track_epoch`、`source_generation` 和 `command`。
- Consumes: `command.strike_plan.v_ball_in`、`command.strike_plan.v_ball_out`、`command.v_racket_target_w`。
- Produces: `HitterEnv._format_hitter_velocity_log_vector(value: np.ndarray) -> str`。
- Produces: `HitterEnv._log_hitter_strike_target(active_result: PlannerResultSnapshot | None) -> None`。
- Produces: 单行 `HITTER strike target:` INFO，或失败时的 `Failed to log HITTER strike target:` WARNING。

- [ ] **Step 1: 创建测试夹具和“完整日志内容”失败测试**

创建 `deploy/tests/test_hitter_strike_target_logging.py`：

```python
from __future__ import annotations

import unittest
from contextlib import contextmanager
from types import SimpleNamespace

import numpy as np
from loguru import logger

from envs.hitter import HitterEnv
from utils.hitter_realtime import (
    CommandPhase,
    HitterCommandLifecycle,
    PlannerResultSnapshot,
)


@contextmanager
def captured_log_messages():
    messages = []
    sink_id = logger.add(
        lambda message: messages.append(message.record["message"]),
        level="INFO",
    )
    try:
        yield messages
    finally:
        logger.remove(sink_id)


def fake_command(
    *,
    marker: float = 1.0,
    strike_type: str = "backhand",
    ball_in=None,
    ball_out=None,
    racket_velocity=None,
):
    if ball_in is None:
        ball_in = [-marker, -marker, -marker]
    if ball_out is None:
        ball_out = [marker, marker, marker]
    if racket_velocity is None:
        racket_velocity = [marker, marker, marker]
    return SimpleNamespace(
        strike_type=strike_type,
        p_base_target_xy=np.array([-0.4, 0.0], dtype=np.float64),
        v_racket_target_w=np.asarray(racket_velocity, dtype=np.float64),
        time_to_strike=0.90,
        strike_plan=SimpleNamespace(
            p_racket_target=np.array([0.0, 0.1, 1.0], dtype=np.float64),
            v_ball_in=np.asarray(ball_in, dtype=np.float64),
            v_ball_out=np.asarray(ball_out, dtype=np.float64),
        ),
    )


def planner_result(
    *,
    epoch: int = 7,
    generation: int = 11,
    deadline: float = 10.90,
    command=None,
):
    if command is None:
        command = fake_command()
    return PlannerResultSnapshot(
        track_epoch=epoch,
        source_generation=generation,
        source_frame=generation,
        strike_deadline_monotonic_s=deadline,
        completed_monotonic_s=10.01,
        command=command,
        error=None,
    )


def configured_lifecycle() -> HitterCommandLifecycle:
    return HitterCommandLifecycle(
        waiting_tts=0.92,
        arm_tts=0.90,
        minimum_arm_tts=0.80,
        maximum_policy_tts=0.92,
        swing_duration_sampler=lambda: 1.85,
    )


def minimal_env() -> HitterEnv:
    env = HitterEnv.__new__(HitterEnv)
    env.motion_cfg = {
        "ball_planner": {
            "target_base_height_w": 0.78,
        }
    }
    env.hitter_command_lifecycle = configured_lifecycle()
    env._hitter_last_logged_phase = CommandPhase.ARMED
    env._hitter_last_result_key = None
    env._hitter_minimum_track_epoch = 0
    env._hitter_minimum_generation = 0
    env._hitter_observed_track_epoch = None
    return env


class StrikeTargetPayloadTests(unittest.TestCase):
    def test_logs_all_world_velocity_vectors_and_norms(self):
        env = minimal_env()
        ball_in = np.array([-3.0, 0.4, -1.2], dtype=np.float64)
        ball_out = np.array([4.2708, -0.5, 1.8], dtype=np.float64)
        racket_velocity = np.array([1.4, -0.1, 0.7], dtype=np.float64)
        result = planner_result(
            epoch=27,
            generation=8103,
            command=fake_command(
                ball_in=ball_in,
                ball_out=ball_out,
                racket_velocity=racket_velocity,
            ),
        )

        with captured_log_messages() as messages:
            env._log_hitter_strike_target(result)

        target_messages = [
            message
            for message in messages
            if message.startswith("HITTER strike target:")
        ]
        self.assertEqual(len(target_messages), 1)
        message = target_messages[0]
        self.assertIn("epoch=27 generation=8103 type=backhand", message)
        self.assertIn(
            "v_ball_in_w_mps=[-3.0000,0.4000,-1.2000]",
            message,
        )
        self.assertIn(
            f"speed_ball_in_mps={np.linalg.norm(ball_in):.4f}",
            message,
        )
        self.assertIn(
            "v_ball_out_w_mps=[4.2708,-0.5000,1.8000]",
            message,
        )
        self.assertIn(
            f"speed_ball_out_mps={np.linalg.norm(ball_out):.4f}",
            message,
        )
        self.assertIn(
            "v_racket_target_w_mps=[1.4000,-0.1000,0.7000]",
            message,
        )
        self.assertIn(
            f"speed_racket_mps={np.linalg.norm(racket_velocity):.4f}",
            message,
        )
```

- [ ] **Step 2: 运行测试并确认 RED**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2
PYTHONPATH=deploy conda run --no-capture-output -n rb \
  python -m unittest discover \
  -s deploy/tests \
  -p 'test_hitter_strike_target_logging.py' \
  -v
```

Expected:

```text
ERROR: test_logs_all_world_velocity_vectors_and_norms
AttributeError: 'HitterEnv' object has no attribute '_log_hitter_strike_target'
```

- [ ] **Step 3: 增加“非法出球速度只 warning”失败测试**

在同一个 `StrikeTargetPayloadTests` 中增加：

```python
    def test_invalid_ball_out_only_warns_and_does_not_change_main_validation(self):
        env = minimal_env()
        command = fake_command(ball_out=[np.nan, 0.0, 0.0])

        decision = env._consume_hitter_planner_result(
            planner_result(command=command),
            now=10.0,
        )
        self.assertEqual(decision, "armed")
        self.assertIs(
            env.hitter_command_lifecycle.active_result.command,
            command,
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
            if message.startswith(
                "Failed to log HITTER strike target:"
            )
        ]
        self.assertEqual(len(warnings), 1)
        self.assertIn("strike_plan.v_ball_out", warnings[0])

    def test_missing_active_result_only_warns(self):
        env = minimal_env()

        with captured_log_messages() as messages:
            env._log_hitter_strike_target(None)

        warnings = [
            message
            for message in messages
            if message.startswith(
                "Failed to log HITTER strike target:"
            )
        ]
        self.assertEqual(len(warnings), 1)
        self.assertIn("missing a command", warnings[0])
```

- [ ] **Step 4: 再次运行并确认三个测试因缺失日志方法而失败**

Run:

```bash
PYTHONPATH=deploy conda run --no-capture-output -n rb \
  python -m unittest discover \
  -s deploy/tests \
  -p 'test_hitter_strike_target_logging.py' \
  -v
```

Expected:

```text
FAILED (errors=3)
```

三个错误都必须指向缺失的 `_log_hitter_strike_target`，而不是 import、
Conda 或夹具错误。

- [ ] **Step 5: 实现最小旁路日志方法**

在 `deploy/envs/hitter.py` 的
`_validated_hitter_command_fields()` 后、`_copy_hitter_command()` 前新增：

```python
    @staticmethod
    def _format_hitter_velocity_log_vector(value: np.ndarray) -> str:
        return "[" + ",".join(
            f"{float(component):.4f}" for component in value
        ) + "]"

    def _log_hitter_strike_target(
        self,
        active_result: PlannerResultSnapshot | None,
    ) -> None:
        try:
            if active_result is None or active_result.command is None:
                raise ValueError("active planner result is missing a command")
            command = active_result.command
            fields = self._validated_hitter_command_fields(command)
            ball_in_velocity = fields["ball_in_velocity"].copy()
            ball_out_velocity = self._validated_vector(
                command.strike_plan.v_ball_out,
                name="strike_plan.v_ball_out",
                size=3,
            )
            racket_velocity = fields["racket_velocity"].copy()
            logger.info(
                "HITTER strike target: epoch={} generation={} type={} "
                "v_ball_in_w_mps={} speed_ball_in_mps={:.4f} "
                "v_ball_out_w_mps={} speed_ball_out_mps={:.4f} "
                "v_racket_target_w_mps={} speed_racket_mps={:.4f}",
                int(active_result.track_epoch),
                int(active_result.source_generation),
                fields["strike_type"],
                self._format_hitter_velocity_log_vector(
                    ball_in_velocity
                ),
                float(np.linalg.norm(ball_in_velocity)),
                self._format_hitter_velocity_log_vector(
                    ball_out_velocity
                ),
                float(np.linalg.norm(ball_out_velocity)),
                self._format_hitter_velocity_log_vector(
                    racket_velocity
                ),
                float(np.linalg.norm(racket_velocity)),
            )
        except Exception as exc:
            try:
                logger.warning(
                    "Failed to log HITTER strike target: {}",
                    exc,
                )
            except Exception:
                pass
```

不要把 `ball_out_velocity` 加入
`_validated_hitter_command_fields()` 的返回值或主验收路径。

- [ ] **Step 6: 运行 targeted test 并确认 GREEN**

Run:

```bash
PYTHONPATH=deploy conda run --no-capture-output -n rb \
  python -m unittest discover \
  -s deploy/tests \
  -p 'test_hitter_strike_target_logging.py' \
  -v
```

Expected:

```text
Ran 3 tests
OK
```

- [ ] **Step 7: 检查 Task 1 diff 并提交**

Run:

```bash
git diff --check -- \
  deploy/envs/hitter.py \
  deploy/tests/test_hitter_strike_target_logging.py
git diff -- \
  deploy/envs/hitter.py \
  deploy/tests/test_hitter_strike_target_logging.py
git add -- deploy/tests/test_hitter_strike_target_logging.py
git add -p -- deploy/envs/hitter.py
git diff --cached -- \
  deploy/envs/hitter.py \
  deploy/tests/test_hitter_strike_target_logging.py
git diff --cached --name-only
git diff --cached --check
git commit -m "feat: add HITTER strike target log payload"
```

Expected:

- `git add -p` 只接受 Task 1 新增的两个日志方法所在 hunk；
- 必须拒绝 observation 105→104 维的用户已有 hunk；
- 暂存区只包含新测试文件和 Task 1 日志方法；
- commit 成功；
- commit 后用户已有的 104 维 observation diff 仍保持未暂存状态。

---

### Task 2: 在最终 strike 边界恰好记录一次

**Files:**
- Modify: `deploy/tests/test_hitter_strike_target_logging.py`
- Modify: `deploy/envs/hitter.py:882-930`

**Interfaces:**
- Consumes: Task 1 的
  `HitterEnv._log_hitter_strike_target(active_result) -> None`。
- Consumes: `HitterCommandLifecycle.active_result` 的 final override 语义。
- Produces: `crossed_strike` 分支中每个击球周期恰好一次的目标日志。

- [ ] **Step 1: 增加 final override 和跨完整 recovery 的失败测试**

在测试文件中增加：

```python
class StrikeTargetBoundaryTests(unittest.TestCase):
    def test_late_advance_logs_latest_override_exactly_once(self):
        env = minimal_env()
        lifecycle = configured_lifecycle()
        env.hitter_command_lifecycle = lifecycle

        first = planner_result(
            epoch=27,
            generation=1,
            deadline=10.90,
            command=fake_command(marker=1.0),
        )
        self.assertEqual(
            lifecycle.ingest(first, now=10.00),
            "armed",
        )

        final_override = planner_result(
            epoch=27,
            generation=2,
            deadline=10.88,
            command=fake_command(marker=2.0),
        )
        self.assertEqual(
            lifecycle.ingest(final_override, now=10.02),
            "overridden",
        )

        previous_phase = lifecycle.phase
        previous_active = lifecycle.active_result
        previous_end = lifecycle.command_end_deadline_s
        late_now = float(previous_end) + 0.10
        lifecycle.advance(late_now)
        self.assertEqual(lifecycle.phase, CommandPhase.WAITING)

        with captured_log_messages() as messages:
            env._log_hitter_advance_transitions(
                previous_phase,
                previous_active,
                previous_end,
                now=late_now,
            )
            env._log_hitter_advance_transitions(
                lifecycle.phase,
                lifecycle.active_result,
                lifecycle.command_end_deadline_s,
                now=late_now + 0.01,
            )

        target_messages = [
            message
            for message in messages
            if message.startswith("HITTER strike target:")
        ]
        self.assertEqual(len(target_messages), 1)
        message = target_messages[0]
        self.assertIn("epoch=27 generation=2", message)
        self.assertIn(
            "v_ball_out_w_mps=[2.0000,2.0000,2.0000]",
            message,
        )
        self.assertNotIn(
            "v_ball_out_w_mps=[1.0000,1.0000,1.0000]",
            message,
        )

    def test_non_strike_transition_does_not_log_target(self):
        env = minimal_env()
        env.hitter_command_lifecycle = SimpleNamespace(
            phase=CommandPhase.TRACKING
        )

        with captured_log_messages() as messages:
            env._log_hitter_advance_transitions(
                CommandPhase.WAITING,
                None,
                None,
                now=10.0,
            )

        self.assertFalse(
            any(
                message.startswith("HITTER strike target:")
                for message in messages
            )
        )
```

- [ ] **Step 2: 运行测试并确认 RED**

Run:

```bash
PYTHONPATH=deploy conda run --no-capture-output -n rb \
  python -m unittest discover \
  -s deploy/tests \
  -p 'test_hitter_strike_target_logging.py' \
  -v
```

Expected:

```text
FAIL: test_late_advance_logs_latest_override_exactly_once
AssertionError: 0 != 1
```

`test_non_strike_transition_does_not_log_target` 和 Task 1 的三个 payload
测试必须通过。

- [ ] **Step 3: 把日志方法接入现有 crossed_strike 分支**

修改 `deploy/envs/hitter.py::_log_hitter_advance_transitions()`：

```python
        if crossed_strike:
            self._log_hitter_strike_target(previous_active)
            logger.info(
                "HITTER lifecycle transition: {} -> {}",
                CommandPhase.ARMED.value,
                CommandPhase.RECOVERY.value,
            )
```

不要把调用条件写成 `current_phase == CommandPhase.RECOVERY`，也不要在
`armed`、`overridden` 或每个 policy step 中调用。

- [ ] **Step 4: 运行全部新测试并确认 GREEN**

Run:

```bash
PYTHONPATH=deploy conda run --no-capture-output -n rb \
  python -m unittest discover \
  -s deploy/tests \
  -p 'test_hitter_strike_target_logging.py' \
  -v
```

Expected:

```text
Ran 5 tests
OK
```

- [ ] **Step 5: 检查 Task 2 diff 并提交**

Run:

```bash
git diff --check -- \
  deploy/envs/hitter.py \
  deploy/tests/test_hitter_strike_target_logging.py
git diff -- \
  deploy/envs/hitter.py \
  deploy/tests/test_hitter_strike_target_logging.py
git add -- deploy/tests/test_hitter_strike_target_logging.py
git add -p -- deploy/envs/hitter.py
git diff --cached -- \
  deploy/envs/hitter.py \
  deploy/tests/test_hitter_strike_target_logging.py
git diff --cached --name-only
git diff --cached --check
git commit -m "feat: log final HITTER strike target once"
```

Expected:

- `git add -p` 只接受 Task 2 在 `crossed_strike` 中新增调用的 hunk；
- observation 105→104 维 hunk继续拒绝；
- commit 成功，且没有暂存其他 dirty 文件。

---

### Task 3: 回归验证与现场日志检查

**Files:**
- Verify: `deploy/envs/hitter.py`
- Verify: `deploy/tests/test_hitter_strike_target_logging.py`
- Verify: `deploy/tests/test_mujoco_physical_table_tennis.py`

**Interfaces:**
- Consumes: Tasks 1-2 的最终代码。
- Produces: 可供最终 review 的测试、语法、diff 和日志格式证据。

- [ ] **Step 1: 运行新日志测试**

Run:

```bash
PYTHONPATH=deploy conda run --no-capture-output -n rb \
  python -m unittest discover \
  -s deploy/tests \
  -p 'test_hitter_strike_target_logging.py' \
  -v
```

Expected:

```text
Ran 5 tests
OK
```

- [ ] **Step 2: 运行当前仍存在的 HITTER/MuJoCo 相关测试**

Run:

```bash
PYTHONPATH=deploy conda run --no-capture-output -n rb \
  python -m unittest discover \
  -s deploy/tests \
  -p 'test_mujoco_physical_table_tennis.py' \
  -v
```

Expected baseline in the current dirty worktree:

```text
Ran 23 tests
FAILED (failures=1)
```

唯一已知既有失败：

```text
test_xml_defines_only_intended_ball_contact_pairs
```

原因是当前 XML pair friction 为
`0.20 0.20 0.005 0.0001 0.0001`，测试仍期望旧的
`0.20 0.005 0.0001`。验收条件是没有新增失败；不能删除、修改或回退
用户的 contact/测试改动来制造绿色结果。

- [ ] **Step 3: 运行语法和 whitespace 验证**

Run:

```bash
PYTHONPATH=deploy conda run --no-capture-output -n rb \
  python -m py_compile \
  deploy/envs/hitter.py \
  deploy/tests/test_hitter_strike_target_logging.py
git diff --check
```

Expected: 两条命令退出码均为 0。

- [ ] **Step 4: 验证日志字段只在 strike 边界出现**

Run:

```bash
rg -n \
  "_log_hitter_strike_target|HITTER strike target:|crossed_strike" \
  deploy/envs/hitter.py \
  deploy/tests/test_hitter_strike_target_logging.py
```

Expected:

- 生产代码只有一个 `HITTER strike target:` 格式定义；
- 生产代码只有一个 `_log_hitter_strike_target(previous_active)` 调用；
- 该调用位于 `_log_hitter_advance_transitions()` 的
  `if crossed_strike:` 内。

- [ ] **Step 5: 最终 dirty-worktree review**

Run:

```bash
git status --short --branch
git log -3 --oneline
git show --stat --oneline HEAD
git diff -- deploy/envs/hitter.py
```

Expected:

- 新测试文件和日志代码均已提交；
- 用户原有的其他修改、删除和未跟踪文件仍保留；
- 最终汇报明确列出本功能 commit、测试数量和任何既有失败；
- 不 push、不创建 PR，除非用户另外明确要求。
