# Task 4 实现报告

## 结果

- 状态：完成
- 分支：`local/hitter-task-diagnostics-20260727`
- 提交：`e0eff65 feat: add HITTER diagnostic event hub`
- 提交范围严格为：
  - `deploy/diagnostics/hitter_task_events.py`
  - `deploy/tests/test_hitter_task_events.py`

## 实现

- `EventHub.publish()` 在同一个 `Condition(RLock)` 临界区内完成 candidate
  event ID、严格 JSON/UTF-8 大小校验、纯 reducer、immutable state 原子替换、
  bounded ring append、ID 提交和 cursor 唤醒。
- 默认 reducer 支持完整 `HEALTH_SNAPSHOT`、`LIFECYCLE_SNAPSHOT`、
  `ATTEMPT_CURRENT`、`ATTEMPT_CLOSED` payload；其他 kind 只推进
  watermark。
- `recent_attempts` 固定最多 100 条并按最新关闭 attempt 在前去重。
- cursor bootstrap 固定为 fresh/resume/reset；运行时落后 ring 时返回精确
  `lost_event_ids`、空 events，并跳到当前 `watermark + 1`。
- 每个 cursor 独立维护读取位置，recorder/SSE 不竞争消费；慢 cursor 不参与
  publish 临界路径。
- 超大或默认 schema 不完整的 draft 在原 candidate ID 上原子替换为小型
  `DIAGNOSTIC_EVENT_DROPPED`，同时增加
  `health.diagnostic_events_dropped`；reducer 失败不会提交状态、ring 或 ID。
- event/state/payload 均深冻结；大小按 `ensure_ascii=False` 的 UTF-8 bytes
  且 `allow_nan=False` 计算。`max_event_bytes` 最小 1024 bytes，保证 fallback
  能表达。
- hub/cursor close 幂等；阻塞 read 会被 cursor 或 hub close 唤醒，closed 后
  publish/read/open cursor 明确抛 `RuntimeError`。
- `ProgressMailbox` 是 publish 前独立 bounded coalescer，按
  `(attempt_id, stage)` rate gate 保留最新 draft；已发布 event 永不原位修改。
- 公共注解使用 Python 3.8 的 `Optional` / `Tuple`，没有依赖
  `get_type_hints()`。

## RED → GREEN

RED：

```text
ModuleNotFoundError: No module named 'diagnostics.hitter_task_events'
Ran 1 test
FAILED (errors=1)
```

GREEN：

```text
Ran 11 tests in 0.016s
OK
```

测试覆盖 8 publisher 连续 ID、两独立 cursor、bootstrap/runtime gap、默认
reducer 与 recent=100、nested immutable、超大/畸形 fallback、reducer 原子
失败、cursor/hub close 唤醒，以及 bounded progress mailbox。

## 最终验证

- Python 3.8 `py_compile`：通过。
- Task 4 unittest：11/11 通过。
- `git diff --check`：通过。
- `git diff --cached --check`：通过。
- commit 前 cached name-status：恰好两个 Task 4 新文件。

## 风险与边界

- 本任务只实现内存 state/ring/cursor；磁盘 recorder gap 污染和 SSE reset
  响应由后续 Task 5/9 消费 `lost_event_ids` 实现。
- 非 finite 数值已由 Task 3 canonical JSON codec 固定转为字符串 token；
  Task 4 的非法事件测试因此使用缺字段的 recognized snapshot，而不是把已合法
  canonicalize 的 token 再判非法。
- `task-4-report.md` 是 SDD 编排元数据，不纳入 Task 4 commit。

## Review 修复

- 修复提交：`6b8dc59 fix: harden HITTER diagnostic event delivery`
- 仅修改并提交 `hitter_task_events.py` 与 `test_hitter_task_events.py`。
- 新增 `EventPublisherLane(capacity=4096)`：producer `offer()` 严格
  put-nowait，唯一后台 owner 调用 `hub.publish()`；overflow=N 合并为一个
  drop event，但 health counter 精确增加 N。hub 关闭/异常不会形成无限重试。
- EventHub 的深冻结输入复用、19 位 event-id 上界 UTF-8 JSON 预编码和 fallback
  准备全部移到 condition 锁外；锁内只绑定 candidate ID、执行纯 reducer 并
  原子提交 state/ring。
- `PublishedEvent` public 构造仍深冻结；hub 内部使用私有 trusted constructor
  避免第二次递归复制。
- cursor registry 改为 `WeakSet`，外部 cursor 引用释放后不会被 hub 保活。
- fallback 使用 finite monotonic、int64 wall time、截断的 kind/error/attempt
  文本，并保证自身小于 1024 bytes。
- `ProgressMailbox` 每 key 使用一个同时包含 last-emit/pending 的 bounded
  entry，key churn 同步淘汰，不会提前 drain 遗留 pending。
- cursor read 使用 `islice` 至多构造 `limit` 条；ring 同时受 count 与默认
  64 MiB 总 bytes 上限约束。

Review GREEN：

```text
Task 3: Ran 19 tests ... OK
Task 4: Ran 17 tests ... OK
```

同时通过 Python 3.8 `py_compile` 与精确两文件 cached check。未跟踪的 Task 5
测试文件没有进入修复提交。
