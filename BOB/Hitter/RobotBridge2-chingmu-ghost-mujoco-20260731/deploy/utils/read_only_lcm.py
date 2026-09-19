from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Callable

import lcm


@dataclass
class PublicationAudit:
    attempted_publish_count: int = 0
    attempted_channels: tuple[str, ...] = ()
    _lock: threading.RLock = field(
        default_factory=threading.RLock,
        init=False,
        repr=False,
        compare=False,
    )

    def record_attempt(self, channel: str) -> None:
        with self._lock:
            self.attempted_publish_count += 1
            self.attempted_channels = (
                self.attempted_channels + (str(channel),)
            )

    def snapshot(self) -> tuple[int, tuple[str, ...]]:
        with self._lock:
            return (
                int(self.attempted_publish_count),
                tuple(self.attempted_channels),
            )


class PublishDenyLcm:
    """Delegate receive APIs and fail closed on every publish attempt."""

    def __init__(self, inner: object, audit: PublicationAudit) -> None:
        self._inner = inner
        self._audit = audit

    def subscribe(self, channel, handler):
        return self._inner.subscribe(channel, handler)

    def unsubscribe(self, subscription):
        return self._inner.unsubscribe(subscription)

    def fileno(self):
        return self._inner.fileno()

    def handle(self):
        return self._inner.handle()

    def handle_timeout(self, timeout_ms):
        return self._inner.handle_timeout(timeout_ms)

    def publish(self, channel, payload):
        self._audit.record_attempt(str(channel))
        raise RuntimeError(
            "read-only LCM facade blocked publish to {!r}".format(channel)
        )


class ReadOnlyLcmSubscription:
    def __init__(
        self,
        *,
        lcm_url: str,
        channel: str,
        handler: Callable[[str, bytes], None],
        loop_callback: Callable[[], None] | None = None,
        lcm_factory: Callable[[str], object] = lcm.LCM,
        poll_timeout_ms: int = 10,
    ) -> None:
        if not callable(handler):
            raise TypeError("handler must be callable")
        if loop_callback is not None and not callable(loop_callback):
            raise TypeError("loop_callback must be callable or None")
        if not callable(lcm_factory):
            raise TypeError("lcm_factory must be callable")
        timeout = int(poll_timeout_ms)
        if timeout < 0:
            raise ValueError("poll_timeout_ms must be nonnegative")

        self._lcm_url = str(lcm_url)
        self._channel = str(channel)
        self._handler = handler
        self._loop_callback = loop_callback
        self._lcm_factory = lcm_factory
        self._poll_timeout_ms = timeout
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._lc = None
        self._subscription = None
        self._close_completed = False
        self._last_close_joined: bool | None = None

    @property
    def thread_alive(self) -> bool:
        with self._lock:
            thread = self._thread
        return bool(thread is not None and thread.is_alive())

    @property
    def last_close_joined(self) -> bool | None:
        with self._lock:
            return self._last_close_joined

    def start(self) -> None:
        with self._lock:
            if self._close_completed:
                raise RuntimeError("cannot start a closed LCM subscription")
            if self._thread is not None and self._thread.is_alive():
                return
            self._stop_event.clear()
            self._lc = self._lcm_factory(self._lcm_url)
            self._subscription = self._lc.subscribe(
                self._channel,
                self._handler,
            )
            self._thread = threading.Thread(
                target=self._run,
                name="ReadOnlyLcmSubscription-{}".format(self._channel),
                daemon=True,
            )
            self._thread.start()

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                with self._lock:
                    lc_client = self._lc
                if lc_client is None:
                    return
                try:
                    lc_client.handle_timeout(self._poll_timeout_ms)
                except OSError:
                    return
            finally:
                if self._loop_callback is not None:
                    self._loop_callback()

    def close(self, *, join_timeout_s: float = 1.0) -> bool:
        timeout = float(join_timeout_s)
        if timeout < 0.0:
            raise ValueError("join_timeout_s must be nonnegative")

        with self._lock:
            if self._close_completed:
                self._last_close_joined = True
                return True
            thread = self._thread
            if thread is None:
                self._close_completed = True
                self._last_close_joined = True
                return True
            self._stop_event.set()

        thread.join(timeout=timeout)
        joined = not thread.is_alive()
        with self._lock:
            self._last_close_joined = bool(joined)
            if not joined:
                return False
            if self._lc is not None and self._subscription is not None:
                self._lc.unsubscribe(self._subscription)
            self._subscription = None
            self._close_completed = True
            return True
