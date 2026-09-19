# Task 1 实施报告：单次击球目标日志函数

## 状态

已完成 Task 1。仅新增 payload 级单元测试、速度向量格式化 helper 和
strike target 日志 helper；未接入 lifecycle hook。

## RED 证据

第一次仅加入夹具和完整 payload 测试后执行：

```bash
PYTHONPATH=deploy conda run --no-capture-output -n rb \
  python -m unittest discover \
  -s deploy/tests \
  -p 'test_hitter_strike_target_logging.py' \
  -v
```

实际结果：

```text
test_logs_all_world_velocity_vectors_and_norms ... ERROR
AttributeError: 'HitterEnv' object has no attribute '_log_hitter_strike_target'
Ran 1 test in 0.001s
FAILED (errors=1)
```

加入非法 `v_ball_out` 和缺失 active result 两个测试后再次执行同一命令。
实际结果：

```text
test_invalid_ball_out_only_warns_and_does_not_change_main_validation ... ERROR
test_logs_all_world_velocity_vectors_and_norms ... ERROR
test_missing_active_result_only_warns ... ERROR
Ran 3 tests in 0.003s
FAILED (errors=3)
```

三个错误都来自缺失 `_log_hitter_strike_target`；没有 import、Conda 或夹具错误。
非法 `v_ball_out` 用例在报缺失日志方法之前，主 ingestion 已返回 `armed`。

## GREEN 证据

加入最小实现后执行同一 targeted 命令，实际结果：

```text
Ran 3 tests in 0.003s
OK
```

提交后于 2026-07-25 17:28:02 +08:00 再次执行相同命令，exit code 为 0：

```text
test_invalid_ball_out_only_warns_and_does_not_change_main_validation ... ok
test_logs_all_world_velocity_vectors_and_norms ... ok
test_missing_active_result_only_warns ... ok
Ran 3 tests in 0.003s
OK
```

## 改动文件

- `deploy/envs/hitter.py`
  - 新增 `_format_hitter_velocity_log_vector()`。
  - 新增 `_log_hitter_strike_target()`。
  - INFO 单行包含 epoch、generation、type、三组世界系 m/s 速度向量及模长，
    数值固定 4 位小数。
  - validation、copy、norm、format 和 INFO logger 均位于外层保护内；
    失败只在内层保护中尝试 WARNING。
  - `v_ball_out` 仅在日志 helper 中诊断校验，未加入
    `_validated_hitter_command_fields()` 或 command ingestion。
- `deploy/tests/test_hitter_strike_target_logging.py`
  - 覆盖完整 payload。
  - 覆盖非法 `v_ball_out` 不改变主 command 接受集合、只产生 WARNING。
  - 覆盖缺失 active result 只产生 WARNING。

## 自查与独立审查

- `git diff --cached --check` 无输出，exit code 为 0。
- 提交前全局 `git diff --cached --name-only` 仅包含：

```text
deploy/envs/hitter.py
deploy/tests/test_hitter_strike_target_logging.py
```

- 全仓生产代码没有 `_log_hitter_strike_target()` 调用点，因此 Task 1 未接入
  lifecycle。
- 独立只读代码审查结论：Critical、Important、Minor 均为无，可以提交。

## 暂存与提交

使用 `git add -- deploy/tests/test_hitter_strike_target_logging.py` 暂存新测试；
使用 `git add -p -- deploy/envs/hitter.py` 只接受日志 helper hunk，并拒绝后续
observation hunks。

提交：

```text
c65a543d6b262822bc07d82ecf2629db13638120
feat: add HITTER strike target log payload
2 files changed, 257 insertions(+)
```

提交后 index 为空。用户已有的 observation 105→104 diff 仍作为
`deploy/envs/hitter.py` 的未暂存改动保留。

## Concerns

- 本任务按 brief 仅运行 targeted payload tests，没有宣称全仓测试套件通过。
- 工作树中仍有大量用户既有修改、删除和未跟踪文件；本任务未 reset、restore、
  clean 或暂存它们。
