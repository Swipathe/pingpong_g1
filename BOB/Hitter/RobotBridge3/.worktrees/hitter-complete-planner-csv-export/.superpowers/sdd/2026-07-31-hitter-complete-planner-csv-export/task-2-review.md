# Task 2 独立代码审查报告

## Verdict

**APPROVED**

审查范围为 commit
`8366f2b3701056859ba713fe5d219225a02e303f` 相对
`661659c68f480315d75f4395f5b1998ebb88e2b5` 的 Task 2 修改。
未发现 blocking、major 或 minor 问题。

## Findings

### Blocking

无。

### Major

无。

### Minor

无。

## 逐项审查结论

- 四字段 key：`_planner_key_csv_fields()` 在
  `deploy/diagnostics/export_hitter_task_csv.py:42-50` 统一生成
  `attempt_id + schema_version + track_epoch + generation`；三组 builder
  均从 `AlignedPlannerRow.call.key` 生成相同 key。
- 严格 join：`align_completed_calls()` 在
  `deploy/diagnostics/export_hitter_task_csv.py:164-235` 仅按
  `scan.completed_calls` 的既有顺序遍历，并拒绝重复 completed key、
  缺 input、input attempt id 冲突、snapshot key 冲突和 source frame
  冲突。
- 输入加载：`load_attempt_inputs()` 在
  `deploy/diagnostics/export_hitter_task_csv.py:82-161` 直接只读
  `attempt_details/<id>.json`，校验 detail/input attempt identity，并以
  完整四字段 key 拒绝跨文件或文件内重复 input。
- 三表同序：三个 row builder 位于
  `deploy/diagnostics/export_hitter_task_csv.py:238-301`，都只顺序遍历
  传入的 `aligned`，没有重新查询、过滤或排序。
- JSON 与字段保真：`_compact_json()` 在
  `deploy/diagnostics/export_hitter_task_csv.py:53-60` 使用
  `sort_keys=True`、紧凑分隔符和 `allow_nan=False`；字符串 `"NaN"`
  不经数值转换而原样保留。replacement key 在
  `deploy/diagnostics/export_hitter_task_csv.py:69-79` 明确保留
  `attempt_id`。result builder 在
  `deploy/diagnostics/export_hitter_task_csv.py:289-299` 保留完整 result
  字段，并为 `command_fields`、`reason_code`、`error_type`、
  `error_text` 补齐缺省列。
- pending extra inputs：alignment 只要求每个 completed call 唯一命中
  input，不把未完成的合法 pending-replaced inputs 误判为错误。真实数据
  中 540 个 extra inputs 与 540 个 pending-replaced keys 精确相等。
- 数据源与边界：新增模块没有引用 `ReplayCaptureStore`、
  `ReplayInputBundle`、`load_replay_input_bundle()` 或 replay bundle；
  完成调用来自完整 `events.jsonl`，input 来自 attempt detail。提交未修改
  replay capture 上限，也未实现 Task 3 的 CSV 文件写出、CLI、manifest
  或目录交换。
- Python 3.8：在 `/home/loco1/miniconda3/envs/rb` 的 Python 3.8.20
  下通过编译与测试；新增代码未使用高版本专属 API。

## 验证证据

执行：

```bash
cd deploy
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python \
  -m py_compile \
  diagnostics/export_hitter_task_csv.py \
  tests/test_hitter_task_csv_export.py
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python \
  -m unittest \
  tests.test_hitter_task_csv_export \
  tests.test_hitter_task_recording \
  tests.test_hitter_task_replay -v
```

结果：`Ran 79 tests in 2.195s`，`OK`，退出码 0。

对真实 session
`recordings/hitter_task_diagnostics/20260730_194528_402494-p740921-f85e25f0`
只读执行 `scan -> load -> align -> three row builders`：

- inputs / submitted / pending / completed-aligned：
  `2625 / 2625 / 540 / 2085`
- Attempt 1 / 2 completed：`837 / 1248`
- 三表四字段 key：序列完全相同，`2085` 个 key 全部唯一
- extra inputs：`540`，与 pending-replaced key 集合完全相同
- command / error rows：`233 / 1852`
- 字符串 `"NaN"` deadline：`1852`
- replacement JSON：pending `315`、latest `2084`；所有非空值都包含完整
  四字段，其中跨 attempt latest replacement `1` 条并正确保留
  `attempt_id`
- 已校验 input vectors、command mappings 和 replacement mappings
  均为可回读的稳定紧凑 JSON

`git diff --check 661659c..8366f2b` 无输出；提交范围仅为计划指定的两个
Task 2 文件。

## Residual risks

- attempt detail 的 planner input 采集仍受现有实时上限约束；未来 session
  若 completed call 超过已持久化 input 范围，本实现会按设计 fail closed，
  不会生成貌似完整但缺行的数据。当前目标 session 未触发该边界。
- 三个 builder 保证自身不重排，但 Task 3 的调用方仍需把同一个
  `aligned` tuple 传给三者，并在落盘后再次验证 key 序列；该文件写出逻辑
  不属于本次审查范围。
