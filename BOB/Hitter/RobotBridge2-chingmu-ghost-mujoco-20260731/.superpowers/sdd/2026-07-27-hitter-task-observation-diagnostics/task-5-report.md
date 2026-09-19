# Task 5 实现报告

## 结果

- 状态：完成
- 分支：`local/hitter-task-diagnostics-20260727`
- 提交：`33fabd6 feat: record HITTER diagnostic sessions`
- 提交范围严格为：
  - `.gitignore` 中新增的单个 recordings 忽略规则
  - `deploy/diagnostics/hitter_task_recording.py`
  - `deploy/tests/test_hitter_task_recording.py`

## 实现

- `RawRecordLane` 保存 immutable sample/attempt/segment envelope，默认容量
  65536；producer 满载时立即返回并合并连续 input sequence drop 区间，
  recovery、attempt 切换和 close 都会封口区间。
- 会话目录使用带微秒、PID、短 UUID 的 exclusive 0700 目录；会话文件为
  exclusive 0600，`session.json` 和 attempt detail 都通过
  temp + fsync + atomic replace 提交。
- `AttemptDetailRepository` 仅接受正整数 ID，使用固定 numeric JSON 文件名，
  维护最多 100 条的 LRU detail cache；分页按严格
  `[1-9][0-9]*.json` 文件索引倒序读取，不保留无限增长的 summary 容器。
- `SessionRecorder` 对 raw/event 两条输入每轮分别限制为 512/128，event
  cursor 始终以零超时读取；attempt detail、replay analysis 和 replay job
  status 都使用有界 offer queue。
- raw drop 会按 envelope 精确归因；无法归因或 cursor ring gap 会 sticky
  标记 session 和未终态 attempt 为 `RECORDING_INCOMPLETE`，A/B 结果固定为
  `INCONCLUSIVE`。
- cursor gap 会先向 `events.jsonl` 写入包含 lost range/watermark 的本地
  `RECORDER_EVENT_GAP` marker，再置 session incomplete。
- 正常 attempt 只在 A/B 有终态后写一行 `attempts.csv`；shutdown 时 pending
  attempt 固定写 `INCONCLUSIVE`，terminal attempt/session 会立即
  flush + fsync。
- JSON 使用 `allow_nan=False`，raw float64 数组以 JSON CSV 单元格保存，
  CSV 由标准 writer 负责 quoting；I/O 错误转为 sticky recorder status，
  不抛回实时 producer。

## RED → GREEN

初始 RED：

```text
ModuleNotFoundError: No module named 'diagnostics.hitter_task_recording'
Ran 1 test
FAILED (errors=1)
```

边界补强 RED 覆盖了 fresh repository pagination、unsafe short UUID 和
recorder gap marker；实现后 GREEN：

```text
Ran 7 tests in 0.206s
OK
```

## 最终验证

- Python 3.8 `py_compile`：通过。
- Task 5 unittest：7/7 通过。
- 使用 `-W error::ResourceWarning` 运行，无未关闭测试文件句柄。
- `git diff --cached --check`：通过。
- commit 前 cached name-status：恰好两个 Task 5 新文件和 `.gitignore`
  单个目标 hunk。

## 风险与边界

- recorder 负责持久化与污染传播；实际后台线程生命周期和 producer 接线由
  后续集成任务完成。
- `task-5-report.md` 是 SDD 编排元数据，不纳入 Task 5 commit。

## Review 修复

- 修复提交：`1364d29 fix: make HITTER recording terminal-safe`
- 仅修改并提交 `hitter_task_recording.py` 与
  `test_hitter_task_recording.py`。
- raw overflow 的首个 drop 立即进入有界 metadata lane；连续区间在尚未消费
  时原位合并，消费后按新 suffix 增量报告。非连续区间超出 metadata 容量时
  显式返回 `METADATA_OVERFLOW` sentinel，recorder 将其升级为全 session
  incomplete，不再由 `deque(maxlen=...)` 静默淘汰。
- attempt detail 在 raw lane 关闭、drop metadata 消费、terminal backlog
  判断完成前不写 `attempts.csv`；late raw drop、513 条 raw 的 timeout=0
  都会先污染 detail，再以唯一一行 `INCONCLUSIVE` / `recording_complete=false`
  落盘。
- recorder 分离 `observed_watermark` 与 `persisted_event_id`。terminal 捕获
  watermark 后只追到该固定目标；300 events、quota=128、timeout=0 时明确
  incomplete，并仅提交 watermark 128。
- I/O failure 会 sticky 设置 unhealthy、global incomplete，并污染当前和未来
  attempt；partial start 会关闭所有已打开文件，`_open_private()` 的
  `fdopen` 异常也会关闭原始 fd。
- shutdown 使用单 consumer `RLock` 串行化 drain/terminal；顺序固定为停止
  offer、关闭 raw、deadline 内有界 drain、检查所有 backlog、先置污染、再
  terminalize attempt、flush/fsync、atomic session replace 和 close。
- written attempt 仅保留 bounded recent LRU，cache miss 时流式扫描 CSV 做
  exact 去重；specific incomplete IDs bounded，溢出升级为 global；detail
  分页使用 `heapq.nlargest(limit + 1)` + `os.scandir()`，不再全量排序历史。
- 新增 I/O fault、1 秒/1000 行 flush、全文件 schema、partial-start cleanup、
  terminal backlog、concurrent drain/close 和 metadata overflow 测试。

Review RED 包含：

```text
Ran 11 tests
FAILED (failures=4, errors=1)
```

Review GREEN：

```text
Task 3: Ran 19 tests ... OK
Task 4: Ran 17 tests ... OK
Task 5: Ran 18 tests ... OK
```

最终同时通过 Python 3.8 `py_compile`、
`-W error::ResourceWarning` 和精确两文件 cached diff check。
