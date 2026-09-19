# Task 1 报告：保留 AttemptSummary 诊断进度字段

## 实现

- 将 `AttemptSummary` 的四个诊断进度字段改为 `Optional[int]`，默认值统一为 `None`，使旧版 v1 payload 的缺失字段表示“未知”，而不是伪造的 `0/31`、`0/3`。
- 在 `deploy/diagnostics/hitter_task_events.py::_attempt()` 中显式读取并透传：
  - `estimator_sample_count`
  - `estimator_window_size`
  - `incoming_count`
  - `incoming_required_count`
- 新增 reducer 回归测试：
  - 含真实进度 `19/31`、`2/3` 的 payload 分别通过 `ATTEMPT_CURRENT` 和 `ATTEMPT_CLOSED` 后保持原值。
  - 不含四个字段的完整 v1 legacy payload 通过 reducer 后四项均为 `None`。

## 文件

- `deploy/diagnostics/hitter_task_models.py`
- `deploy/diagnostics/hitter_task_events.py`
- `deploy/tests/test_hitter_task_events.py`

未修改 `deploy/envs/hitter.py`；该文件已有的工作树改动保持原样。

## 提交

- SHA：`5d88016db37d9956177b9ddd0bd278481436998e`
- Subject：`fix: preserve HITTER diagnostic progress fields`
- 提交内容仅包含上述三个任务文件。

## RED

命令：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
/home/loco1/miniconda3/envs/rb/bin/python -m unittest \
  tests.test_hitter_task_events
```

退出码：`1`

输出：

```text
F.................
======================================================================
FAIL: test_attempt_reducer_keeps_legacy_progress_fields_unknown (tests.test_hitter_task_events.EventHubTests)
----------------------------------------------------------------------
Traceback (most recent call last):
  File "/home/loco1/BOB/Hitter/RobotBridge2/deploy/tests/test_hitter_task_events.py", line 289, in test_attempt_reducer_keeps_legacy_progress_fields_unknown
    self.assertEqual(
AssertionError: Tuples differ: (0, 31, 0, 3) != (None, None, None, None)

First differing element 0:
0
None

- (0, 31, 0, 3)
+ (None, None, None, None)

======================================================================
FAIL: test_attempt_reducer_preserves_live_progress_fields (tests.test_hitter_task_events.EventHubTests) (kind='ATTEMPT_CURRENT')
----------------------------------------------------------------------
Traceback (most recent call last):
  File "/home/loco1/BOB/Hitter/RobotBridge2/deploy/tests/test_hitter_task_events.py", line 252, in test_attempt_reducer_preserves_live_progress_fields
    self.assertEqual(
AssertionError: Tuples differ: (0, 31, 0, 3) != (19, 31, 2, 3)

First differing element 0:
0
19

- (0, 31, 0, 3)
?  ^      ^

+ (19, 31, 2, 3)
?  ^^      ^

======================================================================
FAIL: test_attempt_reducer_preserves_live_progress_fields (tests.test_hitter_task_events.EventHubTests) (kind='ATTEMPT_CLOSED')
----------------------------------------------------------------------
Traceback (most recent call last):
  File "/home/loco1/BOB/Hitter/RobotBridge2/deploy/tests/test_hitter_task_events.py", line 252, in test_attempt_reducer_preserves_live_progress_fields
    self.assertEqual(
AssertionError: Tuples differ: (0, 31, 0, 3) != (19, 31, 2, 3)

First differing element 0:
0
19

- (0, 31, 0, 3)
?  ^      ^

+ (19, 31, 2, 3)
?  ^^      ^

----------------------------------------------------------------------
Ran 19 tests in 0.026s

FAILED (failures=3)
```

失败原因符合预期：`_attempt()` 丢弃真实字段，同时 `AttemptSummary` 默认值把 legacy 缺失字段变成假进度。

## GREEN

命令：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
/home/loco1/miniconda3/envs/rb/bin/python -m unittest \
  tests.test_hitter_task_events
```

退出码：`0`

输出：

```text
...................
----------------------------------------------------------------------
Ran 19 tests in 0.034s

OK
```

## 完整测试套件

命令：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
/home/loco1/miniconda3/envs/rb/bin/python -m unittest discover \
  -s tests -p 'test_hitter_task_*.py'
```

退出码：`0`

输出：

```text
2026-07-29 12:19:42.233 | INFO     | utils.motion_lib.torch_humanoid_batch:<module>:41 - Using Humanoid Batch
......................s.............................s................2026-07-29 12:19:55.588 | INFO     | simulator.real_world:_vicon_state_handler:349 - Vicon root information received!
2026-07-29 12:19:55.588 | INFO     | simulator.real_world:_vicon_state_handler:350 - Root Translation World: [1.5 0.  1. ]
...............................................................................................................................................................
----------------------------------------------------------------------
Ran 228 tests in 22.813s

OK (skipped=2)
```

## 自审

- `git diff --check -- deploy/diagnostics/hitter_task_models.py deploy/diagnostics/hitter_task_events.py deploy/tests/test_hitter_task_events.py`：退出码 `0`，无空白错误。
- 补丁仅改变简报指定的三个任务文件；提交将使用显式路径暂存，不会包含当前工作树中的其他修改或删除。
- 测试使用完整真实 payload，不依赖 mock，也没有用生产逻辑计算期望值。
- 变异检查：
  - 删除任一 `_attempt()` 字段透传时，真实进度测试对应字段会失败。
  - 把任一模型默认值恢复为假进度时，legacy 未知值测试对应字段会失败。
  - `ATTEMPT_CURRENT` 与 `ATTEMPT_CLOSED` 两条 reducer 分支均被覆盖。
- 实现没有额外重构或扩大 schema；`to_json_dict()` 已有字段序列化逻辑，会把缺失值按 JSON `null` 输出。

## 关注点

- 本任务仅修复 reducer 层。`deploy/diagnostics/hitter_task_recording.py::_summary_from_json()` 的持久化恢复仍未透传这四个字段，按总任务拆分留给后续 persistence 任务处理。
- 本任务范围内无其他已知问题。
