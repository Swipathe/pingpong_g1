# Strike deadline epsilon 边界修复报告

## 范围与根因

- 工作目录：`/home/loco1/BOB/Hitter/RobotBridge2`
- 基线：`479e97fbea33d9e7aba8cd1b533cdec78a4da5c7`
- 根因：`HitterCommandLifecycle.advance()` 已按自身边界语义完成
  `ARMED -> RECOVERY`，但 `_log_hitter_advance_transitions()` 又用严格
  `now >= strike_deadline` 重新推断事件。在
  `np.nextafter(deadline, -np.inf)` 处，前者为真、后者为假；下一帧旧 phase
  已是 `RECOVERY`，因此该周期的 strike target 永久漏记。
- 语义核验：`_update_hitter_command()` 会在 `advance()` 前保存
  `previous_phase` 和 `previous_active`，并在 `advance()` 后调用日志函数；
  因此 `previous_phase == ARMED` 且当前 phase 已离开 `ARMED` 就是实际发生的
  strike phase transition。此判定也与该函数内既有的真实后端 reset 判定一致。

## RED

在 `deploy/tests/test_hitter_strike_target_logging.py` 新增
`test_epsilon_boundary_logs_original_target_exactly_once`：

- 使用真实 `HitterCommandLifecycle`；
- 使用 `boundary_now = np.nextafter(deadline, -np.inf)`；
- 明确断言 `boundary_now < deadline`；
- 调用 `advance(boundary_now)` 后明确断言 phase 为 `RECOVERY`；
- 使用 advance 前保存的 `previous_active` 调用日志函数，再模拟下一帧；
- 断言 `HITTER strike target:` 总数恰好为 1，并核对原
  `epoch=31 generation=401` 及 marker=3.0 的载荷。

生产代码修改前运行：

```bash
PYTHONPATH=deploy conda run --no-capture-output -n rb \
  python -m unittest discover \
  -s deploy/tests \
  -p 'test_hitter_strike_target_logging.py' \
  -v
```

结果：`Ran 6 tests`，仅新增测试失败；失败为
`AssertionError: 0 != 1`，即 `target_messages` 数量为 0。输出同时显示
lifecycle transition 已是 `armed -> recovery`，符合预期复现。

## 最小生产改动

只修改 `deploy/envs/hitter.py` 中日志函数的 `crossed_strike` predicate：

```python
crossed_strike = bool(
    previous_phase == CommandPhase.ARMED
    and previous_active is not None
    and current_phase != CommandPhase.ARMED
)
```

日志仍调用 `_log_hitter_strike_target(previous_active)`。未修改 lifecycle，未在
日志代码中访问或复制 lifecycle 的 epsilon，也未改变其他日志或控制语义。

## GREEN 与回归

同一目标测试命令在修复后结果：

- `Ran 6 tests in 0.006s`
- `OK`
- 新边界日志恰好一次，来自原 `previous_active`。

MuJoCo 回归命令：

```bash
PYTHONPATH=deploy conda run --no-capture-output -n rb \
  python -m unittest discover \
  -s deploy/tests \
  -p 'test_mujoco_physical_table_tennis.py' \
  -v
```

结果：`Ran 23 tests in 2.097s`，仅有 brief 允许的既有失败：
`test_xml_defines_only_intended_ball_contact_pairs`。实际 friction 为
`0.20 0.20 0.005 0.0001 0.0001`，测试期望
`0.20 0.005 0.0001`；没有新增失败。

以下检查均退出 0：

```bash
PYTHONPATH=deploy conda run --no-capture-output -n rb \
  python -m py_compile \
  deploy/envs/hitter.py \
  deploy/tests/test_hitter_strike_target_logging.py
git diff --check
git diff --cached --check
```

## Git 与工作树保护

- 提交：
  `c7e1e85a6004184c9397098ae9217971e1b883e0`
- 消息：`fix: log HITTER strike at epsilon boundary`
- 提交仅包含：
  - `deploy/envs/hitter.py` 的单个 predicate hunk；
  - `deploy/tests/test_hitter_strike_target_logging.py` 的单个测试增量。
- 提交前全局 `git diff --cached --name-only` 恰好为上述两个文件，
  `git diff --cached` 和 `git diff --cached --check` 已逐项核对。
- `deploy/envs/hitter.py` 的 observation 104 维用户 hunk仍完整保留为未暂存；
  其余大量用户 dirty changes均未清理、回退或暂存。

## Concerns

- 唯一已知回归 concern 是既有 MuJoCo friction 期望不一致；本修复没有触碰
  XML 或该测试。
- 当前仍是包含大量用户改动的 dirty worktree；后续操作仍需显式暂存。
- 本次 fix 没有发现新的 lifecycle、日志或控制语义风险。
