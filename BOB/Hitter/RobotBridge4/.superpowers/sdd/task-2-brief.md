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

