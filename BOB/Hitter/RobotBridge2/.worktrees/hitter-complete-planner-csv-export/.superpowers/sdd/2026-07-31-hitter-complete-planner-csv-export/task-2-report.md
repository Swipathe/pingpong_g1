# Task 2 实现报告

## 状态

DONE

- Commit: `8366f2b3701056859ba713fe5d219225a02e303f`
- 提交消息：`feat: align completed HITTER planner CSV rows`
- 工作树在提交后 clean。

## 修改文件

- `deploy/diagnostics/export_hitter_task_csv.py`
- `deploy/tests/test_hitter_task_csv_export.py`

未实现 Task 3 的文件写出、CLI、manifest 或原子目录交换；未修改 estimator、planner、policy、控制参数、replay capture 上限或 replay bundle。

## 实现摘要

- 新增 `AlignedPlannerRow`、`load_attempt_inputs()`、`align_completed_calls()`。
- `load_attempt_inputs()` 只直接读取 `attempt_details/<id>.json`，不实例化 `AttemptDetailRepository`，不使用 `ReplayCaptureStore` 或 replay bundle。
- loader 校验 detail attempt id、可选 input attempt id、snapshot key，并拒绝重复 planner input key。
- alignment 只按 `scan.completed_calls` 的既有顺序遍历一次；拒绝重复 completed key、缺 input、input attempt/key 冲突和 `source_frame` 冲突。
- 三个 row builder 都直接遍历同一个 aligned sequence，不查询 inputs、不重新排序。

## RED / GREEN 证据

1. 三表同序 RED：
   - 命令：`PYTHONPATH=. ... -m unittest tests.test_hitter_task_csv_export.AlignedPlannerRowsTests.test_three_core_tables_share_the_same_completed_key_order -v`
   - 结果：因缺少 `align_completed_calls` 报 `AttributeError`，符合预期。
2. 最小 alignment GREEN：
   - 同一测试通过，`Ran 1 test ... OK`。
3. 字段保真 RED：
   - `test_core_rows_preserve_values_with_stable_compact_json`
   - 结果：`KeyError: 'velocity_w'`，证明旧的仅 key 投影无法满足字段输出。
4. 字段保真 GREEN：
   - 同序与字段保真测试共同通过，`Ran 2 tests ... OK`。
5. 严格连接 RED：
   - missing input 暴露裸 `KeyError`；
   - duplicate loader 因 API 未实现报 `AttributeError`；
   - source frame 冲突测试因未抛异常而失败。
6. 严格连接 GREEN：
   - `AlignedPlannerRowsTests` 共 10 个测试全部通过。

## 最终验证

- Task 1/2：17/17 通过。
- replay/recording 回归：62/62 通过。
- fresh 最终命令同时运行 `py_compile` 和三组测试：79/79 通过，退出码 0。
- `git diff --check` 与 `git diff --cached --check` 均无输出。
- 暂存与提交范围仅包含两个指定文件。

真实 session 的只读 `scan -> load -> align -> row builders` 验证：

- planner inputs：2625；
- pending-replaced：540；
- completed/aligned：2085；
- Attempt 1/2：837 / 1248；
- command/error：233 / 1852；
- 三表四字段 key 序列完全相同且唯一；
- 跨 attempt latest replacement：1 条，replacement JSON 保留 `attempt_id`；
- result 保留字符串 `"NaN"` deadline。

## 输出字段与对齐不变量

三张核心表每行均包含：

- `attempt_id`
- `snapshot_key.schema_version`
- `snapshot_key.track_epoch`
- `snapshot_key.generation`

其他字段：

- inputs：保留 snapshot 之外的全部顶层输入字段；向量和映射使用 `sort_keys=True`、无多余空格的紧凑 JSON。
- calls：提交、开始、完成时间，执行耗时，source frame，以及包含四字段 key（含 attempt id）的 pending/latest replacement JSON。
- results：保留 result 的全部非 snapshot 顶层字段；明确保留 `command_fields`、`reason_code`、`error_type`、`error_text`，嵌套 command mapping 使用稳定紧凑 JSON。

关键不变量：唯一排序源是 `scan.completed_calls`；`align_completed_calls()` 生成一个 tuple，三个 row builder 只顺序遍历该 tuple，因此相同行号始终对应同一完成调用。

## 风险与后续边界

- Task 2 只产生内存 row mappings；CSV header 汇总、文件写出和事务式目录替换留给 Task 3。
- 嵌套 JSON 中的非有限数值会被 `allow_nan=False` fail closed；源数据中的 deadline 字符串 `"NaN"` 原样保留。
- extra inputs 不被错误拒绝，因为它们包括合法 pending-replaced 输入；严格规则只要求每个 completed key 必须唯一命中 input。
