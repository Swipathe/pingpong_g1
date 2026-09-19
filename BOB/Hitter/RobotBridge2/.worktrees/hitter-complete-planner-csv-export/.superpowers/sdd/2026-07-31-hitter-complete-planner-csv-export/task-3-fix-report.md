# Task 3 评审修复报告

## 结果

- 修复提交：`931ce6da189a0c74f19ebffc257cfe9bfe21e217`
- 提交说明：`fix: make HITTER CSV export fail closed`
- 交付文件：
  - `deploy/diagnostics/export_hitter_task_csv.py`
  - `deploy/tests/test_hitter_task_csv_export.py`
  - `docs/hitter_complete_planner_csv_export.md`
- 未修改真实 recording；真实导出仅写入 `TemporaryDirectory`。

## RED / GREEN 证据

### 完整 raw source

RED：

- 删除中间 `input_seq`、制造重复、倒序或负值时，旧实现仍接受部分情况或没有严格拒绝。
- 将区间内 subject 改为 `g2pelvis` 时，旧实现因三值白名单静默丢行。

GREEN：

- 全文件在任何区间或 subject 筛选之前验证 `input_seq`。
- 首值可任意；后续必须是非负 ASCII 整数并严格逐行 `+1`。
- 缺号、重复、倒序和负值均 fail closed，已有 output sentinel 不变。
- 所有区间内 source rows 均保留，不再使用 subject 白名单。

### Lifecycle attempt 身份

RED：

- 位于 Attempt 1 区间内、但改写为所选 Attempt 2 的 tick 被静默丢弃。

GREEN：

- tick 声明另一个所选 attempt 时抛出 interval owner 冲突。
- tick 声明未选择的 Attempt 3 时继续忽略，导出正常完成。

### 输出路径与 symlink

RED：

- `--output=session/attempt_details --overwrite` 会替换并删除源 detail。
- 中间路径 `alias -> real` 会被跟随并把结果写入真实目标。

GREEN：

- output 与 session 相等、output 包含 session、output 位于 session 内部均拒绝。
- 路径先转为绝对规范化路径。
- 创建父目录之前、之后以及最终目录替换之前，逐级拒绝所有已存在 symlink 分量。
- `session/attempt_details --overwrite` 测试验证原始 `1.json` 字节不变，且没有安装 manifest。

### 目录事务提交点

RED：

- 安装 rename 成功后 parent fsync 失败会暴露新 output 并丢失旧 output。
- backup 清理失败会让函数报错，但新 output 已经提交。
- 安装失败、restore rename 成功、restore fsync 失败时，错误消息错误声称 backup 仍存在。

GREEN：

- 无旧 output：安装后的 parent fsync 失败会把新目录移回 temp，供外层清理。
- 有旧 output：安装后的 parent fsync 失败会把新目录移到同级 `failed-new`，恢复旧 output 并 fsync；测试验证旧 sentinel、`failed-new/new.txt` 和无残留 backup。
- 安装 rename 失败后恢复 fsync 失败时，错误准确说明旧 output 已恢复、backup 已被消费。
- 安装 rename 与 parent fsync 成功是提交点；之后 backup `rmtree` 或 cleanup fsync 失败只发出 warning，不把成功提交报告成失败。

### 最终 source 检查

RED：

- 在真实 `_write_manifest()` 返回后向 `session.json` 追加内容，旧实现仍成功替换 output。

GREEN：

- 最终 stat/SHA-256 检查移动到 manifest 写入、round-trip 和 temp directory fsync 之后，紧邻 `replace_output_directory()` 之前。
- 同一注入现在抛出 `source file changed during export`，旧 output sentinel 和目录内容保持不变。

## 验证

Python 版本与语法：

```text
Python 3.8.20
py_compile: PASS
git diff --check: PASS
```

完整回归：

```text
Ran 95 tests in 2.432s
OK
```

其中 exporter 测试为 33 个；相对既有 85 个三模块测试新增 10 个测试方法。

真实 session：

```text
source:
  20260730_194528_402494-p740921-f85e25f0
raw source:
  rows=308510
  input_seq=0..308509
  contiguous=true
submitted / pending-replaced / completed:
  2625 / 540 / 2085
summary.csv:
  2
raw_lcm.csv:
  33894
planner_inputs.csv:
  2085
planner_calls.csv:
  2085
planner_results.csv:
  2085
stage_timeline.csv:
  2102
policy_ticks.csv:
  1578
task_observations.csv:
  2
manifest counts == closed-file reread counts:
  true
all completeness checks:
  true
Attempt 2 generation 11192:
  present
```

## 已知剩余边界

- 路径检查和 rename 仍是 path-based，没有使用锁定的 directory fd / `renameat`。有权限的并发写者仍可能在最后一次组件检查与 rename 之间替换路径，形成路径级 TOCTOU。
- 源文件仍通过路径多次读取；最终 stat/hash 能缩小但不能彻底消除 check-to-rename 竞态，也无法阻止并发写者修改后恢复为相同内容。
- `input_seq` 相邻连续性无法识别文件尾部被整体截断。彻底关闭该风险需要 recorder 在 session metadata 中持久化 raw row count 或末端 `input_seq` watermark。
- 覆盖非空目录需要两次 rename；两次 rename 之间 output 会短暂不存在，进程崩溃仍可能留下 backup 或 `failed-new`，因此它是可恢复目录事务，不是单次原子交换。
