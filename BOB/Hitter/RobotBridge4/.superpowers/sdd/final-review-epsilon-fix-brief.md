# Final review fix: strike deadline epsilon 边界不漏记

## Root cause

`HitterCommandLifecycle.advance()` 使用
`now + HitterCommandLifecycle._EPSILON >= strike_deadline` 判定
`ARMED -> RECOVERY`，而
`HitterEnv._log_hitter_advance_transitions()` 当前重新用严格
`now >= strike_deadline` 推断同一事件。两套比较规则不一致。

当 `now = np.nextafter(deadline, -np.inf)` 时：

- lifecycle 已进入 `RECOVERY`；
- 日志条件仍为 false；
- 下一帧的 `previous_phase` 已是 `RECOVERY`，因此该周期永久漏记。

## Required change

- 先在 `deploy/tests/test_hitter_strike_target_logging.py` 增加回归测试。
- 测试用真实 lifecycle，令
  `boundary_now = np.nextafter(deadline, -np.inf)`。
- 明确断言 `boundary_now < deadline`，但 `advance(boundary_now)` 后 phase 为
  `RECOVERY`。
- 调用 `_log_hitter_advance_transitions()`，再模拟下一帧调用，最终
  `HITTER strike target:` 总数必须恰好为 1。
- 日志必须仍来自原 `previous_active`。
- 在写生产修复前运行目标文件，确认新增测试 RED，且原因是 target count 为 0。
- 最小生产修复：日志的 `crossed_strike` 应依据 lifecycle 已发生的 phase
  transition，而不是复制 deadline 比较规则：

```python
crossed_strike = bool(
    previous_phase == CommandPhase.ARMED
    and previous_active is not None
    and current_phase != CommandPhase.ARMED
)
```

- 不访问或复制 `_EPSILON`，不修改 lifecycle，不改其他日志或控制语义。
- 修复后目标文件应为 6/6 GREEN。
- 再运行 MuJoCo 23 项回归，允许且仅允许既有 friction 失败。
- `py_compile`、`git diff --check` 必须通过。

## Git constraints

- 当前工作区有大量用户 dirty changes，全部保留。
- `deploy/envs/hitter.py` 的 observation 104 维 hunk必须保持未暂存。
- 只提交本 fix 的 predicate hunk和一个测试。
- 禁止 reset/restore/clean、`git add .`、`git add -A`。
- 提交前检查全局 staged diff、name-only 和 `--check`。
