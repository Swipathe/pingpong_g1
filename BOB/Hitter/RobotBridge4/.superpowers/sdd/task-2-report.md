# Task 2 实施报告：在最终 strike 边界恰好记录一次目标日志

## 状态

已完成 Task 2。仅新增两个 boundary tests，并在既有
`_log_hitter_advance_transitions()` 的 `if crossed_strike:` 分支中加入一次
`self._log_hitter_strike_target(previous_active)` 调用。

## RED 证据

先只新增 final override + late advance 与 non-strike 两个测试，尚未修改生产代码，
执行：

```bash
PYTHONPATH=deploy conda run --no-capture-output -n rb \
  python -m unittest discover \
  -s deploy/tests \
  -p 'test_hitter_strike_target_logging.py' \
  -v
```

实际结果：

```text
test_late_advance_logs_latest_override_exactly_once ... FAIL
test_non_strike_transition_does_not_log_target ... ok
test_invalid_ball_out_only_warns_and_does_not_change_main_validation ... ok
test_logs_all_world_velocity_vectors_and_norms ... ok
test_missing_active_result_only_warns ... ok

FAIL: test_late_advance_logs_latest_override_exactly_once
AssertionError: 0 != 1

Ran 5 tests in 0.005s
FAILED (failures=1)
```

失败原因正是 strike target 日志尚未接入，且 Task 1 的三个 payload tests 与
non-strike test 均通过。

## GREEN 证据

在既有 `if crossed_strike:` 分支加入唯一一行 hook 后，执行同一命令，实际结果：

```text
test_late_advance_logs_latest_override_exactly_once ... ok
test_non_strike_transition_does_not_log_target ... ok
test_invalid_ball_out_only_warns_and_does_not_change_main_validation ... ok
test_logs_all_world_velocity_vectors_and_norms ... ok
test_missing_active_result_only_warns ... ok

Ran 5 tests in 0.005s
OK
```

提交后于 2026-07-25 17:34:45 +08:00 再次执行相同命令，exit code 为 0：

```text
Ran 5 tests in 0.006s
OK
```

late advance 的实际 INFO 使用
`epoch=27 generation=2` 和
`v_ball_out_w_mps=[2.0000,2.0000,2.0000]`。测试同时断言日志总数为 1，并排除
`v_ball_out_w_mps=[1.0000,1.0000,1.0000]`。

## 改动文件

- `deploy/envs/hitter.py`
  - 仅在现有 `if crossed_strike:` 分支新增
    `self._log_hitter_strike_target(previous_active)`。
  - 未使用 current active，未新增缓存，未改成
    `current_phase == CommandPhase.RECOVERY` 条件。
- `deploy/tests/test_hitter_strike_target_logging.py`
  - 新增 late advance 用例：一次 `advance()` 跨过 strike 和完整 recovery 到
    `WAITING`，仍恰好记录一次最后 accepted override。
  - 新增 non-strike 用例：`WAITING -> TRACKING` 不记录 target 日志。

未修改 Task 1 helper、command 接受规则、生命周期状态转移、observation、action、
planner、LCM 或 R2。

## 自查与独立审查

- `git diff --check -- deploy/envs/hitter.py
  deploy/tests/test_hitter_strike_target_logging.py` 通过。
- 提交前全局 `git diff --cached --name-only` 仅包含：

```text
deploy/envs/hitter.py
deploy/tests/test_hitter_strike_target_logging.py
```

- `git diff --cached --check` 无输出，exit code 为 0。
- 完整 staged diff 中，生产代码只有 crossed-strike 分支新增的 1 行；测试新增
  88 行。
- 独立只读代码审查结论：Critical、Important、Minor 均无，Task 2 为 READY。

## 暂存与提交

使用 `git add -- deploy/tests/test_hitter_strike_target_logging.py` 暂存测试；
使用 `git add -p -- deploy/envs/hitter.py` 只接受 crossed-strike hook，并拒绝
全部 observation 104 维 hunk。

提交：

```text
479e97fbea33d9e7aba8cd1b533cdec78a4da5c7
feat: log final HITTER strike target once
2 files changed, 89 insertions(+)
```

提交后 index 为空。用户已有的 observation 104 维改动仍作为
`deploy/envs/hitter.py` 的未暂存 hunk 保留。

## Concerns

- 本任务按 brief 仅运行 targeted
  `test_hitter_strike_target_logging.py`，没有宣称全仓测试套件通过。
- 工作树中仍有大量用户既有修改、删除和未跟踪文件；本任务未 reset、restore、
  clean、修改或暂存它们。
