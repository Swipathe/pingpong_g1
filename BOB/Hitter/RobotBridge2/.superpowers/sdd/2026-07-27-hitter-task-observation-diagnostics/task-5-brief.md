## Task 5: 实现有界 raw lane 与持久化记录器

**Files:**

- Create: `deploy/diagnostics/hitter_task_recording.py`
- Create: `deploy/tests/test_hitter_task_recording.py`
- Modify: `.gitignore`

### Interface

```python
@dataclass(frozen=True)
class SessionPaths:
    root: Path
    session_json: Path
    ball_samples_csv: Path
    events_jsonl: Path
    replay_jobs_jsonl: Path
    replay_analysis_jsonl: Path
    attempts_csv: Path
    attempt_details_dir: Path


@dataclass(frozen=True)
class RawSubmitResult:
    accepted: bool
    dropped_input_seq: tuple[int, int] | None
    attempt_id: int | None


@dataclass(frozen=True)
class RecorderStatus:
    healthy: bool
    recording_complete: bool
    bytes_written: int
    last_error: str | None


class RawRecordLane:
    def __init__(self, capacity: int = 65536) -> None:
        """Create a bounded queue with no producer-side blocking."""

    def submit(self, sample: NormalizedMocapSample) -> RawSubmitResult:
        """Non-blocking put; report exact lost input_seq ranges on overflow."""

    def take_many(
        self,
        *,
        limit: int,
        timeout_s: float = 0.0,
    ) -> tuple[NormalizedMocapSample, ...]:
        """Take up to limit records for the recorder only."""

    def close(self) -> None:
        """Reject new records and wake the recorder."""


class AttemptDetailRepository:
    def commit(self, detail: AttemptDetail) -> None:
        """Atomically replace a numeric detail file and refresh the LRU cache."""

    def get(self, attempt_id: int) -> AttemptDetail | None:
        """Read cache first, then only the fixed <positive-id>.json path."""

    def list_page(
        self,
        *,
        limit: int = 50,
        before: int | None = None,
    ) -> AttemptPage:
        """Return attempt_id-desc, exclusive-cursor pagination."""


class SessionRecorder:
    def __init__(
        self,
        *,
        paths: SessionPaths,
        raw_lane: RawRecordLane,
        event_cursor: EventCursor,
        attempt_repository: AttemptDetailRepository,
        raw_quota: int = 512,
        event_quota: int = 128,
        flush_interval_s: float = 1.0,
        flush_row_count: int = 1000,
    ) -> None:
        """Bind the two independent input lanes and all output files."""

    def start(self) -> None:
        """Open exclusive 0600 files inside an exclusive 0700 session dir."""

    def offer_attempt_detail(self, detail: AttemptDetail) -> bool:
        """Queue one immutable detail/summary update."""

    def offer_replay_analysis(
        self,
        analysis: Mapping[str, JsonValue],
    ) -> bool:
        """Queue a parent-received replay result; child never writes files."""

    def request_stop(self) -> None:
        """Stop accepting new work while retaining queued records."""

    def drain(self, timeout_s: float) -> RecorderStatus:
        """Drain raw<=512/event<=128 batches; keep session.json open."""

    def write_terminal_and_close(
        self,
        *,
        terminal_session: Mapping[str, JsonValue],
        timeout_s: float,
    ) -> RecorderStatus:
        """Write terminal metadata, fsync/flush, close files and cursor."""

    def status(self) -> RecorderStatus:
        """Return an immutable health snapshot."""
```

`SessionPaths.root` 采用
`YYYYMMDD_HHMMSS_ffffff-p<PID>-<short_uuid>`，exclusive create 后目录 0700、文件 0600。

- 使用 `tempfile.TemporaryDirectory()` 写目录名格式、exclusive create、0700/0600 权限测试。
- 写 raw queue `65536` 默认、有界 nonblocking、连续 drop 合并成 `[first_input_seq,last_input_seq]` 的测试。
- 写 raw drop 通过 EventHub 报告但不写回 raw lane、相关 attempt/session 标记 `RECORDING_INCOMPLETE`、A/B 禁用测试。
- 写 recorder 每轮 raw 512 / event 128 公平 drain 测试，任一路高压不能饿死另一路。
- 写 recorder event cursor 落后并被 ring 覆盖的测试：session/受影响 attempt 标记 `RECORDING_INCOMPLETE`，A/B 禁用；不得阻塞 EventHub publisher。
- 写 `session.json`、`ball_samples.csv`、`events.jsonl`、`replay_jobs.jsonl`、`replay_analysis.jsonl`、`attempts.csv` 和 numeric `attempt_details/<id>.json` schema 测试。
- 写 attempt 正常时只在 A/B 完成后落一行 `attempts.csv`；退出时 pending attempt 落 `INCONCLUSIVE`，不能留下无终态半行。
- 写每 1 秒或 1000 行 flush、attempt/session terminal 立即 flush、磁盘异常降级不抛到 realtime producer 的测试。
- 运行失败测试：

  ```bash
  cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
  PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover -s tests -p 'test_hitter_task_recording.py' -v
  ```

- 实现 recorder；detail 查询只能接受正整数 id，先内存 cache，再拼接固定 `<id>.json`，禁止用户输入路径。
- 在 `.gitignore` 只添加一行：

  ```text
  recordings/hitter_task_diagnostics/
  ```

- 提交；`.gitignore` 使用 hunk staging：

  ```bash
  git add deploy/diagnostics/hitter_task_recording.py deploy/tests/test_hitter_task_recording.py
  git add -p .gitignore
  git diff --cached --check
  git commit -m "feat: record HITTER diagnostic sessions"
  ```

