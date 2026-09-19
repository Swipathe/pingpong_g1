# HITTER Complete Planner CSV Export 最终分支复审（二）

## Verdict

**CHANGES REQUESTED — 暂不建议集成。**

复审范围：

```text
base = ebc14fb7959f8314ea7122713702f3f5401d3076
head = 9544c93fe1f86206a3deae01ca545471b1bdbf0b
```

`9544c93` 已正确关闭前次审查发现的 planner 事件集合 fail-open：
selected attempt 的 start/complete 集必须相等，二者必须都有 submit，
pending-replaced 与 start/complete 必须互斥，最后再验证
submitted/completed/pending 分区。原 orphan complete 和 started-pending
复现均已由静默接受变为明确 `ValueError`。

但完整分支仍有两个 Important 和一个 Minor 未解决问题。前两个分别会让
辅助表静默截断、以及让已提交成功的目录被报告为失败，均与本分支公开的
完整性/事务语义冲突。

## Findings

### Critical

无。

### Important

#### 1. 辅助表继续依赖 4096 条有界 attempt detail，未与完整 attempt_transition 事件交叉核对

- **位置：**
  - 有界事实源：
    `deploy/diagnostics/hitter_task_attempts.py:82`、
    `deploy/diagnostics/hitter_task_attempts.py:349-363`
  - detail 从有界 timeline 派生：
    `deploy/diagnostics/hitter_task_monitor.py:2357-2391`
  - 完整 transition 已落入事件日志：
    `deploy/diagnostics/hitter_task_pipeline.py:1086-1105`
  - exporter 只相信 detail：
    `deploy/diagnostics/export_hitter_task_csv.py:1099-1120`、
    `deploy/diagnostics/export_hitter_task_csv.py:1123-1208`、
    `deploy/diagnostics/export_hitter_task_csv.py:1806-1910`
  - event 扫描跳过 transition：
    `deploy/diagnostics/export_hitter_task_csv.py:1596-1597`
  - 被截断 interval 用于 raw/tick：
    `deploy/diagnostics/export_hitter_task_csv.py:2415-2435`
- **影响：**
  - `AttemptTracker` 的每 attempt timeline 是
    `deque(maxlen=4096)`。超过上限时最早 transition 自动丢弃。
  - attempt closure 时，`AttemptDetail.segments` 和
    `AttemptDetail.stage_timeline` 都只从这个有界 tail 生成。早期 segment
    或同一 segment 的早期边界因此可能消失。
  - 完整 `events.jsonl` 中已经持久化全部 `attempt_transition`，但 exporter
    的完整 event pass 对所有非 `lifecycle_tick` 直接 `continue`。
    `stage_timeline.csv` 直接展开有界 detail；`raw_lcm.csv` 和
    `policy_ticks.csv` 又使用 detail 派生的 segment intervals 筛选。
  - 结果是长 attempt 可以同时静默缺失早期 stage、raw 样本和 policy
    ticks，而 manifest 的 20 项 checks 仍全部为 `true`。这重复了本功能
    原本要避开的“完整离线导出依赖有界在线副本”问题，也违反设计中“阶段
    与最终 task observation 从 attempt detail 和完整事件相互核对”的明确
    要求。
- **独立复现：**
  - 构造一个 recorder metadata、event id/watermark、planner 三联事件均
    完整的 session。
  - 完整 `events.jsonl` 保留 Attempt 1 的早期
    `TRACK_SEGMENT_STARTED` transition；detail 模拟有界 timeline 已丢失
    该 transition，并把 segment 起点推进到后期。
  - 当前 HEAD 成功导出：

    ```text
    AUX_TRUNCATION_ACCEPTED True 3 1 2 True
    ```

    含义依次为：早期 transition 未进入 stage CSV、raw 从正常 6 行降到
    3 行、policy tick 从 2 行降到 1 行、stage CSV 只有 detail 的 2 行，
    且全部 manifest checks 仍为 true。
- **当前真实数据：**
  - Attempt 1/2 的完整 transition 数分别为 `845 / 1257`，均低于 4096。
  - 本次独立逐字段比较确认完整 events 与两个 detail timeline 精确相等，
    从 events 重建的 segments 也与 detail 精确相等。因此当前正式
    Attempt 1/2 的 `2102` 行 stage、`33894` 行 raw 和 `1578` 行 policy
    tick 未受该边界影响；问题阻塞的是可重复用于后续长 session 的“完整”
    接口承诺。
- **建议：**
  - 在同一次完整 event pass 中解析 selected attempts 的全部
    `attempt_transition`，并把它作为 stage timeline 和 segment interval
    的完整事实源。
  - detail 可继续用于 summary/planner input/final observation，但必须与
    完整 event transition 的对应 tail/最终值交叉核对；不一致时 fail
    closed。
  - `task_observations.csv` 也应与完整 lifecycle tick 的最终 task
    observation 交叉核对，而不是只相信 detail。
  - 增加一个超过 4096 transitions 的端到端 fixture，断言 early stage、
    raw 和 policy tick 仍完整；另加 detail/event 冲突时保留旧 output 的
    回归测试和明确的 manifest check。

#### 2. warnings-as-errors 会把 commit 后的 cleanup warning 变成“失败”，但新 output 已经安装

- **位置：**
  - post-commit warning：
    `deploy/diagnostics/export_hitter_task_csv.py:2317-2337`
  - 异常继续向外传播：
    `deploy/diagnostics/export_hitter_task_csv.py:2516-2524`
  - CLI 返回非零：
    `deploy/diagnostics/export_hitter_task_csv.py:2546-2561`
  - 现有测试只覆盖普通 warning：
    `deploy/tests/test_hitter_task_csv_export.py:1864-1905`
- **影响：**
  - 实现把 install rename + parent fsync 定义为提交点；其后 backup
    `rmtree` 或 cleanup fsync 失败应是 warning-only。
  - `warnings.warn(..., RuntimeWarning)` 会受进程 warning filter 控制。
    在 `PYTHONWARNINGS=error`、`python -W error` 或
    `warnings.simplefilter("error")` 下，它会抛出 `RuntimeWarning`。
  - 此时新 output 已经安装并持久化，旧 output 仍在 backup；但 API 抛异常，
    CLI 返回 1。调用方收到“失败”后重试，会面对已经改变的目标状态，正是
    前次事务修复要避免的假失败语义。
- **独立复现：**

  ```text
  WARNING_AS_ERROR_FALSE_FAILURE True True 1 True
  ```

  即捕获到 `RuntimeWarning` 异常时，新文件已在 output、旧文件已不在
  output、旧 backup 仍存在，异常文本为 backup cleanup failure。
- **建议：**
  - post-commit 通知不能依赖可被升级为异常的 `warnings.warn`；使用明确的
    stderr/logger 通道，并确保通知本身的任何异常不会越过 commit point。
  - 或把 cleanup 状态作为结构化成功结果返回，但不得让 API/CLI 报告整个
    export 失败。
  - 为 backup `rmtree` 和 cleanup fsync 两条分支增加
    `warnings.simplefilter("error")` / `PYTHONWARNINGS=error` 测试，断言
    函数正常返回、CLI 退出 0、新 output 可用且残留状态被明确报告。

### Minor

#### 1. Planner key 使用 int() 宽松强制转换，非整数和负身份会被接受或改写

- **位置：**
  - `deploy/diagnostics/export_hitter_task_csv.py:558-574`
  - event attempt id 同样被转换：
    `deploy/diagnostics/export_hitter_task_csv.py:615-618`
  - 关闭后 CSV 重读只能看到已改写的整数：
    `deploy/diagnostics/export_hitter_task_csv.py:2034-2047`
- **影响：**
  - `int()` 会接受布尔值、数字字符串、负数，并把非整数浮点向零截断。
    核心四字段 identity 因此可能和源 JSON 不同，甚至与另一真实 key
    碰撞；落盘后的 typed-key 重读无法发现，因为 CSV 中已经写入转换后的
    值。
  - 当前 recorder 正常产生整数且真实 session 未触发；这是损坏/非规范
    source 的 fail-closed 缺口。
- **独立复现：**

  ```text
  FRACTIONAL_KEY_COERCED 1 7
  NEGATIVE_KEY_ACCEPTED PlannerExportKey(
      attempt_id=1, schema_version=1, track_epoch=-2, generation=-3
  )
  ```

  源 key 中 `schema_version=true, generation=7.9` 被接受为 `1, 7`；
  负 epoch/generation 也直接接受。
- **建议：**
  - 使用现有 `_require_int()` 做严格类型检查，拒绝 bool、float 和字符串。
  - `attempt_id >= 1`，`track_epoch/generation >= 0`，并要求
    `schema_version == EXPORT_SCHEMA_VERSION`。
  - 最好在 `PlannerExportKey.__post_init__` 集中维护不变量，同时让所有
    source parser 给出字段级错误；增加 bool/string/fractional/negative/
    unsupported-schema 测试。

## 9544c93 事件集合修复结论

**通过。原 finding 已关闭。**

- `deploy/diagnostics/export_hitter_task_csv.py:733-758`：
  `start_keys == complete_keys`，且两者均为 submitted。
- `deploy/diagnostics/export_hitter_task_csv.py:760-773`：
  pending 与 start/complete 均互斥。
- `deploy/diagnostics/export_hitter_task_csv.py:775-827`：
  只从已验证集合构造 completed calls，再验证
  `submitted == completed | pending`。
- `deploy/tests/test_hitter_task_csv_export.py:777-881` 新增了 orphan
  start+complete、started pending、completed pending 三组真实 JSONL
  形状测试。
- 前次两项最小复现现在输出：

  ```text
  ORPHAN_COMPLETE_REJECTED planner start and complete keys differ
  STARTED_PENDING_REJECTED planner start and complete keys differ
  ```

跨 attempt A-only/B-only/A+B 行为和真实 Attempt 1/2 计数未回归。

## Fresh verification

在 `9544c93fe1f86206a3deae01ca545471b1bdbf0b` 上执行：

```text
Python 3.8.20
py_compile exporter/tests                         PASS
export + replay + recording                       Ran 96 tests; OK
attempt/model regression                          Ran 20 tests; OK
git diff --check ebc14fb..HEAD                    PASS
git diff --check                                  PASS
worktree                                          clean
```

使用当前 HEAD 对真实源 session 重新导出到 `TemporaryDirectory`，未覆盖
正式输出：

```text
summary.csv                2
raw_lcm.csv            33894
planner_inputs.csv      2085
planner_calls.csv       2085
planner_results.csv     2085
stage_timeline.csv      2102
policy_ticks.csv        1578
task_observations.csv      2
```

额外只读核验：

- planner 事件关系：
  `submitted=2625, start=2085, complete=2085, pending=540`；
  start/complete 相等、均为 submitted、与 pending 互斥，且完整分区。
- 三张核心表 2085 个四字段 key 逐行同序且唯一；Attempt 1/2 为
  `837 / 1248`，generation 11192 存在。
- 当前 HEAD 临时导出的 row counts 和 core key SHA-256 与正式 manifest
  完全一致，全部 checks 为 true。
- 完整 events 的 Attempt 1/2 transition 分别为 `845 / 1257`，与 detail
  逐字段精确相等；events 重建 segments 与 detail 相等。
- 正式输出父目录没有 temp、backup 或 failed-new 残留。
- base..HEAD 仍只涉及 exporter、对应测试和运行文档；未修改实时
  estimator/planner/policy、AttemptTracker 上限或 replay capture 上限。

## 已知且已披露的非阻塞限制

以下沿用前次复审判断，本报告不重复升级：

1. 路径式 component 检查和 rename 仍有并发 TOCTOU；完全关闭需要
   directory fd / `renameat`。
2. source 多次路径读取与最终 hash 之间仍有窄竞态窗。
3. `input_seq` 连续性无法识别文件尾部整体截断，需要 recorder raw
   watermark。
4. 非空目录覆盖是可恢复的两次 rename 事务，不是单次原子交换。

