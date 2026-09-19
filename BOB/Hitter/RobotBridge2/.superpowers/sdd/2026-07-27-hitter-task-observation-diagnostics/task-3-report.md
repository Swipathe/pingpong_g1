# Task 3 实现报告

## 结果

- 状态：完成
- 分支：`local/hitter-task-diagnostics-20260727`
- 提交：`d33061a feat: model HITTER diagnostic attempts`
- 提交范围严格为：
  - `deploy/diagnostics/__init__.py`
  - `deploy/diagnostics/hitter_task_models.py`
  - `deploy/diagnostics/hitter_task_attempts.py`
  - `deploy/tests/test_hitter_task_attempts.py`

## 实现

- 定义了 Task 3 brief 中全部不可变协议模型。
- `NormalizedMocapSample` 的 position/quaternion 均在 `__post_init__` 中复制为只读 `float64`。
- 所有嵌套 JSON mapping/sequence 均深复制并冻结；公开
  `freeze_json_value()` 与 `to_builtin_json()` 供后续 EventHub/recorder 复用。
- 每个协议模型使用逐字段 `to_json_dict()`，固定输出
  `schema_version=1`，没有使用 `__dict__`、pickle 或绝对路径。
- `AttemptTracker` 仅维护展示身份和时间线，不 import planner/LCM，也不调用
  production `.reset()` / `.mark_track_ended()`。
- 实现了 visible 上升沿、全局单调 attempt/segment ID、invalid 已推进 epoch
  的旧 segment 绑定、0.20 s 闭区间 grace、PASS 后
  `POST_DEADLINE_TAIL`、health warning、RECOVERY active/cached 映射、
  snapshot/result 迟到归属和表驱动 primary blocker。
- 为避免二进制浮点把数学上的 grace 边界提前关闭，deadline 比较使用
  `1e-12` 边界容差；明显严格超过 deadline 的值仍立即关闭。

## RED → GREEN 证据

RED 命令：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python \
  -m unittest discover -s tests -p 'test_hitter_task_attempts.py' -v
```

首次结果：退出码 1，测试模块因
`ModuleNotFoundError: No module named 'diagnostics'` 按预期失败。

最小实现后的首轮结果：12 项中 11 项通过；唯一失败是
`10.1 + 0.20` 的浮点表示令闭区间边界被提前判为过期。加入最小边界容差后，
同一命令 12/12 通过。

## 最终验证

- Python 3.8 `py_compile`：通过。
- Python 3.8 逐模块 import：通过。
- Task 3 unittest：`Ran 12 tests ... OK`。
- tracker 禁止依赖/调用扫描：无 planner/LCM import，无 `.reset()`，
  无 `.mark_track_ended()`，无 `__dict__`。
- `git diff --cached --check`：通过。
- commit 前 cached name-status：恰好四个 brief 文件，全部为新增文件。

## 风险与边界

- 本任务只运行 brief 指定的 Task 3 测试，没有运行被用户删除的旧测试，也没有
  恢复任何既有删除文件。
- 深冻结后的 `MappingProxyType` 不用于 pickle；跨进程/落盘必须继续调用显式
  JSON 转换函数，这与设计契约一致。
- `task-3-report.md` 是 SDD 编排元数据，按上级指令不纳入 Task 3 commit。

## Review 修复

- 修复提交：`7b46140 fix: bound HITTER diagnostic attempt state`
- 提交范围严格为：
  - `deploy/diagnostics/hitter_task_models.py`
  - `deploy/diagnostics/hitter_task_attempts.py`
  - `deploy/tests/test_hitter_task_attempts.py`
- 新增默认 `max_attempts=100`、`max_bindings=8192`、
  `max_timeline_per_attempt=4096`；只淘汰最老 closed attempt，并同步清理
  binding/lifecycle projection。active attempt 永不因 attempt cache 上限被淘汰。
- 所有 tracker public 方法共享同一个 `RLock`；新增 immutable
  `current_summary()`、`summary_for_attempt()`、`timeline_for_attempt()`。
- attempt terminal outcome 在关闭时固定；late result 仍进入有界 timeline，
  但不再改写 ready/planned/armed/pass 或 primary blocker。
- 非 finite JSON 值统一编码为 `NaN`、`Infinity`、`-Infinity`；所有模型均通过
  `json.dumps(..., allow_nan=False)`。
- mocap ndarray 改为 immutable bytes-backed `float64` view，调用
  `.setflags(write=True)` 会抛 `ValueError`。
- grace 恢复严格 `now <= deadline`；Python 3.8 缺少
  `math.nextafter`，测试优先使用它并在当前环境回退到等价的
  `numpy.nextafter`。
- `EventDraft` 校验 `state|attempt` scope，attempt scope 必须携带正
  `attempt_id`。

Review RED：新增测试首次运行出现 3 个 assertion failure 和 5 个 expected
API/error failure，分别覆盖无界构造参数/公开快照缺失、可重新写 ndarray、
非 finite 未转换、scope 未校验及严格 grace 边界。

Review GREEN：

```text
Ran 19 tests in 0.022s
OK
```

同时重新通过 Python 3.8 `py_compile`、三文件 `git diff --check` 和
`git diff --cached --check`。未跟踪的 Task 4 测试文件没有进入修复提交。
