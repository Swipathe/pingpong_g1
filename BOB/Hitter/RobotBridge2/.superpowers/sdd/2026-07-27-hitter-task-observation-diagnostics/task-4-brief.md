## Task 4: 实现原子 StateStore / EventHub

**Files:**

- Create: `deploy/diagnostics/hitter_task_events.py`
- Create: `deploy/tests/test_hitter_task_events.py`

### Interface

```python
@dataclass(frozen=True)
class PublishedEvent:
    schema_version: int
    event_id: int
    kind: str
    monotonic_s: float
    wall_time_us: int
    scope: str
    attempt_id: int | None
    payload: Mapping[str, JsonValue]


@dataclass(frozen=True)
class CursorBootstrap:
    mode: str
    watermark_event_id: int


@dataclass(frozen=True)
class EventRead:
    events: tuple[PublishedEvent, ...]
    watermark_event_id: int
    lost_event_ids: tuple[int, int] | None


class EventCursor:
    @property
    def bootstrap(self) -> CursorBootstrap:
        """Return fresh, resume, or reset plus current watermark."""

    def read(
        self,
        *,
        limit: int = 128,
        timeout_s: float | None = None,
    ) -> EventRead:
        """Read this cursor only; a ring gap is explicit in lost_event_ids."""

    def close(self) -> None:
        """Release subscription state and wake any blocked read."""


class EventHub:
    def __init__(
        self,
        initial_state: DiagnosticState,
        *,
        capacity: int = 8192,
        max_event_bytes: int = 65536,
    ) -> None:
        """Own one bounded immutable ring and one atomic state reference."""

    def publish(self, draft: EventDraft) -> PublishedEvent:
        """Allocate id, reduce immutable state, and append ring under one lock."""

    def state_snapshot(self) -> DiagnosticState:
        """Return the current immutable state by atomic reference."""

    def open_cursor(self, after_event_id: int | None) -> EventCursor:
        """Create an independent cursor; consumers never compete on queue.get()."""

    def close(self) -> None:
        """Wake and close all cursors without blocking a producer."""
```

`DiagnosticState` 的 reader-facing 结构固定为：

```python
@dataclass(frozen=True)
class DiagnosticState:
    schema_version: int
    watermark_event_id: int
    health: HealthSnapshot
    lifecycle: LifecycleSnapshot
    current_attempt: AttemptSummary | None
    recent_attempts: tuple[AttemptSummary, ...]
```

默认 ring 容量 `8192`，内存 attempt detail cache `100`，单 event 序列化上限 `64 KiB`，异常文本截断到 `2048` 字符。

- 写 8 个 publisher 线程的测试：event id 必须唯一、连续，state watermark 必须等于 ring 尾。
- 写两个独立 cursor 读取同一事件的测试，证明 recorder 和 SSE 不竞争消费。
- 写落后 cursor 的 ring-gap reset 测试。
- 写 event 发布后不可修改、嵌套 payload 不可变、超大 event 被拒绝为结构化 diagnostics error 的测试。
- 写 progress producer mailbox/rate-gate 测试：只允许在调用 `publish()` 之前合并；已有 event id 的事件永不原位改写。
- 运行失败测试：

  ```bash
  cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
  PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover -s tests -p 'test_hitter_task_events.py' -v
  ```

- 实现纯 reducer、原子 publish、condition-based cursor wait 和关闭唤醒；任何 cursor 停滞不得阻塞 publish。
- 用 fake clock 测 heartbeat/invalidation，不在 unit test 中 `sleep()`。
- 提交：

  ```bash
  git add deploy/diagnostics/hitter_task_events.py deploy/tests/test_hitter_task_events.py
  git diff --cached --check
  git commit -m "feat: add HITTER diagnostic event hub"
  ```

