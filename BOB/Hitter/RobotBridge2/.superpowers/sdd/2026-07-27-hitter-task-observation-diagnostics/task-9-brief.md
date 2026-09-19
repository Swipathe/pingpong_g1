## Task 9: 实现只读 localhost HTTP/SSE

**Files:**

- Create: `deploy/diagnostics/hitter_task_web.py`
- Create: `deploy/tests/test_hitter_task_web.py`

### Interface

```python
@dataclass(frozen=True)
class WebServerConfig:
    host: str = "127.0.0.1"
    port: int = 8765
    max_handlers: int = 8
    max_sse_clients: int = 4
    request_timeout_s: float = 5.0
    heartbeat_s: float = 15.0


class HitterTaskWebApplication:
    def __init__(
        self,
        *,
        hub: EventHub,
        attempts: AttemptDetailRepository,
        static_files: Mapping[str, Path],
    ) -> None:
        """Build immutable JSON payloads from the canonical Task 3 schema."""


class HitterTaskWebServer:
    def __init__(
        self,
        app: HitterTaskWebApplication,
        config: WebServerConfig,
        *,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        """Create a bounded ThreadingHTTPServer; port 0 is valid in tests."""

    @property
    def address(self) -> tuple[str, int]:
        """Return the actual bound loopback address."""

    def start(self) -> None:
        """Start one daemon accept thread."""

    def close(self, *, timeout_s: float = 5.0) -> bool:
        """Close SSE cursors, shutdown the server, and join finitely."""
```

### Routes

```text
GET /
GET /api/state
GET /api/attempts?limit=50&before=<exclusive_positive_attempt_id>
GET /api/attempts/<positive_attempt_id>
GET /events
```

SSE 只发送：

```text
ready      {"watermark_event_id": 123}
invalidate {"scope": "state|attempt", "attempt_id": 7}
reset      {"watermark_event_id": 456}
heartbeat  {"watermark_event_id": 456}
```

- 用 `ThreadingHTTPServer` 绑定 ephemeral port，使用 `http.client` 测 state schema 和 attempt desc/exclusive pagination；默认 limit 50，最大 200。
- 测 invalid id/query/method 返回 JSON 400/404/405，包含 `Content-Type`、`Content-Length`、`Cache-Control: no-store`。
- 测只映射 `/` 和明确静态资源，`..`、绝对路径、encoded traversal 均 404。
- 测 Host 仅允许 `127.0.0.1`、`localhost` 和实际端口；无 wildcard CORS；所有响应带 `nosniff`。
- 测 fresh SSE `ready` 后客户端 GET state；Last-Event-ID ring 内补发 invalidate；ring gap 发 reset；旧 event 从不携带状态 patch。
- 用 fake heartbeat clock 测 15 秒 heartbeat；测试断开释放 cursor/thread。
- 测总 handler semaphore 8、SSE semaphore 4、超限 503、普通 socket 5 秒 timeout。
- 测 detail 只从 cache 或 numeric fixed file 读取，API JSON 不含绝对路径和下载字段。
- 运行失败测试：

  ```bash
  cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
  PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover -s tests -p 'test_hitter_task_web.py' -v
  ```

- 实现 HTTP/SSE；SSE slow client 只落后自身 cursor，不持有 EventHub publish lock。
- 提交：

  ```bash
  git add deploy/diagnostics/hitter_task_web.py deploy/tests/test_hitter_task_web.py
  git diff --cached --check
  git commit -m "feat: serve HITTER diagnostic state"
  ```

