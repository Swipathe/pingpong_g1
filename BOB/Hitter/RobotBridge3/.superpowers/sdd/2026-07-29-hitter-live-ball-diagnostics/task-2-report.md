# Task 2 交付报告：磁盘记录与 API 保持真实进度字段

## 状态

已完成实现、TDD RED/GREEN、完整 HITTER task 测试套件与提交前自审。

## 实现

- `AttemptDetailRepository` 从 JSON 重建 `AttemptSummary` 时，使用
  `Mapping.get()` 显式读取以下四个可空字段：
  - `estimator_sample_count`
  - `estimator_window_size`
  - `incoming_count`
  - `incoming_required_count`
- `attempts.csv` 的表头和每行增加同样四列。
  - 有真实值时原样写入，例如 `19,31,2,3`。
  - 值为 `None` 时由 `csv.DictWriter` 写为空单元格，不合成 `0`。
- HTTP 层已有正确的直接序列化路径：
  - `/api/attempts` 直接序列化 repository 返回的 `AttemptPage`。
  - `/api/attempts/<id>` 直接序列化 repository 返回的 `AttemptDetail`。
  - 两条路径都没有 `or 0` 或其他数值回退，因此生产
    `hitter_task_web.py` 无需修改；新增真实 HTTP 往返测试锁定该行为。
- legacy schema v1 JSON 缺失四个字段时，repository 使用 `None`，
  HTTP 列表与详情均输出 JSON `null`。

## 文件

- 修改 `deploy/diagnostics/hitter_task_recording.py`
  - JSON 反序列化透传四个 nullable 字段。
  - CSV 增加四个字段。
- 检查但未修改 `deploy/diagnostics/hitter_task_web.py`
  - 现有实现已直接序列化 repository/model 值。
- 修改 `deploy/tests/test_hitter_task_recording.py`
  - repository 写入并重开后保留 `19/31`、`2/3`。
  - CSV 验证表头、每行、真实值与未知空值。
- 修改 `deploy/tests/test_hitter_task_web.py`
  - `/api/attempts` 与 `/api/attempts/7` 返回一致真实计数。
  - legacy JSON 在两条 API 中均返回四个 `null`。
- 未触碰 `deploy/envs/hitter.py`；其工作树修改是任务开始前已有的无关改动。

## TDD：RED

命令：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
/home/loco1/miniconda3/envs/rb/bin/python -m unittest \
  tests.test_hitter_task_recording \
  tests.test_hitter_task_web
```

生产代码尚未修改时的关键输出：

```text
......F....F.......................................
FAIL: test_detail_repository_reopen_preserves_progress_counts
AssertionError: (None, None, None, None) != (19, 31, 2, 3)

FAIL: test_attempt_csv_preserves_progress_counts_and_blanks_unknowns
AssertionError: False is not true

FAIL: test_attempt_list_and_detail_preserve_truthful_progress_counts
  estimator_sample_count: None != 19
  estimator_window_size: None != 31
  incoming_count: None != 2
  incoming_required_count: None != 3

Ran 52 tests in 3.176s
FAILED (failures=6)
```

失败原因与预期一致：repository 反序列化丢失真实计数，CSV 没有四列，
HTTP 从重开的 repository 得到 `null`。legacy `null` 保护测试在 RED
阶段已通过，证明现有兼容语义，并约束修复不得引入假零。

## TDD：GREEN

使用与 RED 相同的聚焦命令，修复后的完整输出：

```text
....................................................
----------------------------------------------------------------------
Ran 52 tests in 3.171s

OK
```

## 完整测试套件

按要求在提交前运行一次全部 `test_hitter_task_*.py`：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
/home/loco1/miniconda3/envs/rb/bin/python -m unittest discover \
  -s tests -p 'test_hitter_task_*.py'
```

结果：

```text
...................................................................
...................................................................
...................................................................
----------------------------------------------------------------------
Ran 232 tests in 23.541s

OK (skipped=2)
```

退出码为 `0`。两个 skip 为套件既有条件跳过，没有失败或错误。

## 自审

- 范围：生产改动只有 `hitter_task_recording.py` 的 8 行；没有改
  planner、policy、incoming confirmation、真实 action 或
  `deploy/envs/hitter.py`。
- 兼容性：`.get()` 让 legacy 缺字段保持 `None`；未引入 `0`、
  `31` 或 `3` 的默认值。
- API：列表与详情都经过真实 HTTP server 和磁盘重开的 repository，
  测试不是 mock，也没有从实现 helper 计算期望值。
- CSV：测试直接读取实际文件，验证列存在于表头和每一行，并分别检查
  已知值与未知空字符串。
- mutation 检查：
  - 删除任一 JSON 字段读取会使 repository/API 真实值测试失败。
  - 删除任一 CSV 字段会使表头/行测试失败。
  - 把 legacy 缺字段默认成 `0` 会使两个 API 的 `null` 测试失败。
- `git diff --check` 对 Task 2 目标路径无输出。
- 大量既有 dirty-worktree 内容保持原样；暂存将只使用明确路径。

## 关注点

- `deploy/diagnostics/hitter_task_web.py` 列在简报的检查范围内，但无需
  生产修改：其现有两个 API 方法已经直接序列化 canonical model。
  为避免无意义重构，本任务只为它增加行为级 HTTP 回归测试。
- 没有功能性阻塞或已知缺陷。

## 提交

- SHA：`d48a771e2fd358b912dc783707f90db20362114c`
- Subject：`fix: persist truthful HITTER diagnostic counters`
- 提交只包含：
  - `deploy/diagnostics/hitter_task_recording.py`
  - `deploy/tests/test_hitter_task_recording.py`
  - `deploy/tests/test_hitter_task_web.py`
