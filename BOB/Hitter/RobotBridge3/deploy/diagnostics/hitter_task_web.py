from __future__ import annotations

from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
import math
from pathlib import Path
import re
import select
import socket
import threading
import time
from typing import (
    Any,
    Callable,
    Dict,
    Mapping,
    Optional,
    Set,
    Tuple,
    Type,
)
from urllib.parse import parse_qsl, unquote_to_bytes, urlsplit

from diagnostics.hitter_task_events import EventCursor, EventHub
from diagnostics.hitter_task_models import SCHEMA_VERSION
from diagnostics.hitter_task_recording import AttemptDetailRepository


_POSITIVE_INTEGER = re.compile(r"[1-9][0-9]*\Z")
_NONNEGATIVE_INTEGER = re.compile(r"(?:0|[1-9][0-9]*)\Z")
_PERCENT_ESCAPE = re.compile(r"%[0-9A-Fa-f]{2}")
_ABSOLUTE_PATH_FRAGMENT = re.compile(
    r"(?<![A-Za-z0-9])/(?!/)(?=\S)"
)
_UNC_PATH_FRAGMENT = re.compile(
    r"(?<![A-Za-z0-9])(?:\\\\|//)(?=\S)"
)
_WINDOWS_PATH_FRAGMENT = re.compile(
    r"(?<![A-Za-z0-9])[A-Za-z]:[\\/](?=\S)"
)
_RESERVED_ROUTES = {
    "/api/state",
    "/api/attempts",
    "/events",
}
_SSE_DISCONNECT_POLL_S = 0.1


def _require_integer(name: str, value: object, *, minimum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError("{} must be an integer".format(name))
    if value < minimum:
        raise ValueError("{} must be at least {}".format(name, minimum))
    return int(value)


def _require_finite_positive(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError("{} must be a number".format(name))
    converted = float(value)
    if not math.isfinite(converted) or converted <= 0.0:
        raise ValueError("{} must be finite and positive".format(name))
    return converted


@dataclass(frozen=True)
class WebServerConfig:
    host: str = "127.0.0.1"
    port: int = 8765
    max_handlers: int = 8
    max_sse_clients: int = 4
    request_timeout_s: float = 5.0
    heartbeat_s: float = 15.0

    def __post_init__(self) -> None:
        if self.host not in ("127.0.0.1", "::1", "localhost"):
            raise ValueError("host must be an explicit loopback address")
        port = _require_integer("port", self.port, minimum=0)
        if port > 65535:
            raise ValueError("port must be at most 65535")
        handlers = _require_integer(
            "max_handlers",
            self.max_handlers,
            minimum=1,
        )
        sse_clients = _require_integer(
            "max_sse_clients",
            self.max_sse_clients,
            minimum=1,
        )
        if sse_clients > handlers:
            raise ValueError("max_sse_clients cannot exceed max_handlers")
        object.__setattr__(
            self,
            "request_timeout_s",
            _require_finite_positive(
                "request_timeout_s",
                self.request_timeout_s,
            ),
        )
        object.__setattr__(
            self,
            "heartbeat_s",
            _require_finite_positive("heartbeat_s", self.heartbeat_s),
        )


@dataclass(frozen=True)
class _StaticResource:
    body: bytes
    content_type: str


@dataclass(frozen=True)
class _RequestTarget:
    path: str
    query: str


def _strict_unquote(value: str) -> str:
    index = 0
    while True:
        index = value.find("%", index)
        if index < 0:
            break
        if _PERCENT_ESCAPE.match(value, index) is None:
            raise ValueError("invalid percent escape")
        index += 3
    return unquote_to_bytes(value).decode("utf-8", errors="strict")


def _unsafe_path(path: str) -> bool:
    if (
        not path.startswith("/")
        or path.startswith("//")
        or "\\" in path
        or "\x00" in path
        or any(ord(character) < 0x20 for character in path)
    ):
        return True
    return any(component in (".", "..") for component in path.split("/"))


def _parse_request_target(raw_target: str) -> Optional[_RequestTarget]:
    try:
        if raw_target.startswith("//"):
            return None
        split = urlsplit(raw_target)
        if split.scheme or split.netloc or split.fragment:
            return None
        path = _strict_unquote(split.path)
        if _unsafe_path(path):
            return None
        if "%" in path:
            try:
                second_pass = _strict_unquote(path)
            except (UnicodeDecodeError, ValueError):
                second_pass = path
            if _unsafe_path(second_pass):
                return None
        return _RequestTarget(path=path, query=split.query)
    except (UnicodeDecodeError, ValueError):
        return None


def _content_type(path: Path) -> str:
    suffix = path.suffix.lower()
    if suffix in (".html", ".htm"):
        return "text/html; charset=utf-8"
    if suffix == ".css":
        return "text/css; charset=utf-8"
    if suffix in (".js", ".mjs"):
        return "text/javascript; charset=utf-8"
    if suffix == ".json":
        return "application/json; charset=utf-8"
    if suffix == ".svg":
        return "image/svg+xml"
    if suffix == ".png":
        return "image/png"
    if suffix in (".jpg", ".jpeg"):
        return "image/jpeg"
    return "application/octet-stream"


def _json_bytes(value: Mapping[str, object]) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        allow_nan=False,
        separators=(",", ":"),
    ).encode("utf-8")


def _safe_api_value(value: Any) -> Any:
    def contains_path(text: str) -> bool:
        lowered = text.lower()
        return bool(
            "file://" in lowered
            or _ABSOLUTE_PATH_FRAGMENT.search(text) is not None
            or _UNC_PATH_FRAGMENT.search(text) is not None
            or _WINDOWS_PATH_FRAGMENT.search(text) is not None
        )

    if isinstance(value, Mapping):
        safe: Dict[str, Any] = {}
        for key, item in value.items():
            rendered_key = str(key)
            normalized = rendered_key.lower()
            compact = re.sub(r"[^a-z0-9]", "", normalized)
            if (
                contains_path(rendered_key)
                or
                "download" in compact
                or compact in ("path", "root", "directory", "dir")
                or compact.endswith("path")
                or compact.endswith("directory")
                or compact.endswith("dir")
            ):
                continue
            safe[rendered_key] = _safe_api_value(item)
        return safe
    if isinstance(value, (tuple, list)):
        return [_safe_api_value(item) for item in value]
    if isinstance(value, str):
        if contains_path(value):
            return "[redacted]"
    return value


def _safe_api_bytes(value: Mapping[str, object]) -> bytes:
    safe = _safe_api_value(value)
    if not isinstance(safe, Mapping):
        raise TypeError("API payload must be an object")
    return _json_bytes(safe)


def _error_bytes(status: int, code: str, message: str) -> bytes:
    return _json_bytes(
        {
            "schema_version": SCHEMA_VERSION,
            "error": {
                "status": int(status),
                "code": str(code),
                "message": str(message),
            },
        }
    )


def _error_code(status: int) -> str:
    return {
        HTTPStatus.BAD_REQUEST: "BAD_REQUEST",
        HTTPStatus.NOT_FOUND: "NOT_FOUND",
        HTTPStatus.METHOD_NOT_ALLOWED: "METHOD_NOT_ALLOWED",
        HTTPStatus.SERVICE_UNAVAILABLE: "SERVICE_UNAVAILABLE",
    }.get(status, "SERVICE_UNAVAILABLE")


class HitterTaskWebApplication:
    def __init__(
        self,
        *,
        hub: EventHub,
        attempts: AttemptDetailRepository,
        static_files: Mapping[str, Path],
    ) -> None:
        """Build detached response payloads from the canonical diagnostic schema."""
        self._hub = hub
        self._attempts = attempts
        resources: Dict[str, _StaticResource] = {}
        for route, source in static_files.items():
            if not isinstance(route, str):
                raise TypeError("static route must be a string")
            if (
                _parse_request_target(route)
                != _RequestTarget(path=route, query="")
                or route in _RESERVED_ROUTES
                or route.startswith("/api/attempts/")
            ):
                raise ValueError("static route must be a safe exact URL path")
            path = Path(source)
            resources[route] = _StaticResource(
                body=path.read_bytes(),
                content_type=_content_type(path),
            )
        if "/" not in resources:
            raise ValueError("static_files must explicitly map /")
        self._static_files = resources

    def state_bytes(self) -> bytes:
        return _safe_api_bytes(self._hub.state_snapshot().to_json_dict())

    def attempts_bytes(self, *, limit: int, before: Optional[int]) -> bytes:
        return _safe_api_bytes(
            self._attempts.list_page(limit=limit, before=before).to_json_dict()
        )

    def attempt_bytes(self, attempt_id: int) -> Optional[bytes]:
        detail = self._attempts.get(attempt_id)
        return (
            None
            if detail is None
            else _safe_api_bytes(detail.to_json_dict())
        )

    def static_resource(self, route: str) -> Optional[_StaticResource]:
        return self._static_files.get(route)


class _BoundedThreadingHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    block_on_close = False

    def __init__(
        self,
        server_address: Tuple[str, int],
        handler_class: Type[BaseHTTPRequestHandler],
        *,
        max_handlers: int,
        request_timeout_s: float,
    ) -> None:
        self._handler_slots = threading.BoundedSemaphore(max_handlers)
        self._request_timeout_s = request_timeout_s
        self._requests_lock = threading.Lock()
        self._handler_threads: Set[threading.Thread] = set()
        self._active_requests: Set[socket.socket] = set()
        super().__init__(server_address, handler_class)

    def get_request(self) -> Tuple[socket.socket, object]:
        request, client_address = super().get_request()
        request.settimeout(self._request_timeout_s)
        return request, client_address

    def process_request(
        self,
        request: socket.socket,
        client_address: object,
    ) -> None:
        if not self._handler_slots.acquire(blocking=False):
            self._reject_overloaded(request)
            self.shutdown_request(request)
            return
        thread = threading.Thread(
            target=self._process_request_bounded,
            args=(request, client_address),
            name="hitter-task-http-handler",
            daemon=True,
        )
        with self._requests_lock:
            self._handler_threads.add(thread)
            self._active_requests.add(request)
        try:
            thread.start()
        except BaseException:
            with self._requests_lock:
                self._handler_threads.discard(thread)
                self._active_requests.discard(request)
            self._handler_slots.release()
            self.shutdown_request(request)
            raise

    def _process_request_bounded(
        self,
        request: socket.socket,
        client_address: object,
    ) -> None:
        try:
            try:
                self.finish_request(request, client_address)
            except Exception:
                self.handle_error(request, client_address)
            finally:
                self.shutdown_request(request)
        finally:
            with self._requests_lock:
                self._active_requests.discard(request)
                self._handler_threads.discard(threading.current_thread())
            self._handler_slots.release()

    def _reject_overloaded(self, request: socket.socket) -> None:
        body = _error_bytes(
            HTTPStatus.SERVICE_UNAVAILABLE,
            "SERVICE_UNAVAILABLE",
            "maximum concurrent handlers reached",
        )
        response = (
            "HTTP/1.1 503 Service Unavailable\r\n"
            "Content-Type: application/json; charset=utf-8\r\n"
            "Content-Length: {}\r\n"
            "Cache-Control: no-store\r\n"
            "X-Content-Type-Options: nosniff\r\n"
            "Connection: close\r\n"
            "\r\n"
        ).format(len(body)).encode("ascii") + body
        try:
            request.sendall(response)
        except OSError:
            pass

    def active_threads(self) -> Tuple[threading.Thread, ...]:
        with self._requests_lock:
            return tuple(self._handler_threads)

    def close_active_requests(self) -> None:
        with self._requests_lock:
            requests = tuple(self._active_requests)
        for request in requests:
            try:
                request.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass


class _IPv6BoundedThreadingHTTPServer(_BoundedThreadingHTTPServer):
    address_family = socket.AF_INET6


class HitterTaskWebServer:
    def __init__(
        self,
        app: HitterTaskWebApplication,
        config: WebServerConfig,
        *,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        """Create a loopback-only bounded ThreadingHTTPServer."""
        self._app = app
        self._config = config
        self._clock = clock
        self._closing = threading.Event()
        self._sse_slots = threading.BoundedSemaphore(config.max_sse_clients)
        self._streams_lock = threading.Lock()
        self._streams: Set[Tuple[EventCursor, socket.socket]] = set()
        self._lifecycle_lock = threading.RLock()
        self._accept_thread: Optional[threading.Thread] = None
        self._shutdown_called = False
        self._socket_closed = False
        server_type: Type[_BoundedThreadingHTTPServer]
        server_type = (
            _IPv6BoundedThreadingHTTPServer
            if config.host == "::1"
            else _BoundedThreadingHTTPServer
        )
        handler = self._handler_class()
        self._httpd = server_type(
            (config.host, config.port),
            handler,
            max_handlers=config.max_handlers,
            request_timeout_s=config.request_timeout_s,
        )
        bound_host = str(self._httpd.server_address[0])
        try:
            loopback = ipaddress.ip_address(bound_host).is_loopback
        except ValueError:
            loopback = False
        if not loopback:
            self._httpd.server_close()
            raise RuntimeError("resolved server address is not loopback")

    @property
    def address(self) -> Tuple[str, int]:
        """Return the actual bound loopback address and actual port."""
        address = self._httpd.server_address
        return str(address[0]), int(address[1])

    def start(self) -> None:
        """Start one daemon accept thread."""
        with self._lifecycle_lock:
            if self._closing.is_set():
                raise RuntimeError("web server is closed")
            if self._accept_thread is not None:
                return
            thread = threading.Thread(
                target=self._httpd.serve_forever,
                kwargs={"poll_interval": 0.05},
                name="hitter-task-http-accept",
                daemon=True,
            )
            self._accept_thread = thread
            thread.start()

    def close(self, *, timeout_s: float = 5.0) -> bool:
        """Close SSE cursors, shutdown sockets, and join all threads finitely."""
        if isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float)):
            raise TypeError("timeout_s must be a number")
        timeout = float(timeout_s)
        if not math.isfinite(timeout) or timeout < 0.0:
            raise ValueError("timeout_s must be finite and non-negative")
        deadline = time.monotonic() + timeout
        self._closing.set()
        with self._streams_lock:
            streams = tuple(self._streams)
        for cursor, connection in streams:
            cursor.close()
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        self._httpd.close_active_requests()

        with self._lifecycle_lock:
            accept_thread = self._accept_thread
            should_shutdown = (
                accept_thread is not None and not self._shutdown_called
            )
            if should_shutdown:
                self._shutdown_called = True
        if should_shutdown:
            self._httpd.shutdown()
        with self._lifecycle_lock:
            if not self._socket_closed:
                self._httpd.server_close()
                self._socket_closed = True

        if accept_thread is not None and accept_thread is not threading.current_thread():
            accept_thread.join(max(0.0, deadline - time.monotonic()))
        for thread in self._httpd.active_threads():
            if thread is threading.current_thread():
                continue
            thread.join(max(0.0, deadline - time.monotonic()))
        accept_stopped = accept_thread is None or not accept_thread.is_alive()
        handlers_stopped = not any(
            thread.is_alive() for thread in self._httpd.active_threads()
        )
        return accept_stopped and handlers_stopped

    def _register_stream(
        self,
        cursor: EventCursor,
        connection: socket.socket,
    ) -> bool:
        with self._streams_lock:
            if self._closing.is_set():
                return False
            self._streams.add((cursor, connection))
            return True

    def _unregister_stream(
        self,
        cursor: EventCursor,
        connection: socket.socket,
    ) -> None:
        with self._streams_lock:
            self._streams.discard((cursor, connection))

    def _handler_class(self) -> Type[BaseHTTPRequestHandler]:
        owner = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"
            server_version = "HitterTaskDiagnostics/1"
            sys_version = ""

            def log_message(self, format_string: str, *args: object) -> None:
                return

            def send_error(
                self,
                code: int,
                message: Optional[str] = None,
                explain: Optional[str] = None,
            ) -> None:
                del explain
                if code == HTTPStatus.NOT_IMPLEMENTED:
                    self._method_not_allowed()
                    return
                status = code
                if status not in (
                    HTTPStatus.BAD_REQUEST,
                    HTTPStatus.NOT_FOUND,
                    HTTPStatus.METHOD_NOT_ALLOWED,
                    HTTPStatus.SERVICE_UNAVAILABLE,
                ):
                    status = HTTPStatus.SERVICE_UNAVAILABLE
                self._send_json_error(
                    status,
                    message or HTTPStatus(status).phrase,
                )

            def do_GET(self) -> None:
                if not self._valid_host():
                    self._send_json_error(
                        HTTPStatus.BAD_REQUEST,
                        "invalid Host header",
                    )
                    return
                target = _parse_request_target(self._raw_request_target())
                if target is None:
                    self._send_json_error(
                        HTTPStatus.NOT_FOUND,
                        "route not found",
                    )
                    return
                try:
                    self._route_get(target)
                except (BrokenPipeError, ConnectionResetError, socket.timeout):
                    self.close_connection = True
                except Exception:
                    self._send_json_error(
                        HTTPStatus.SERVICE_UNAVAILABLE,
                        "diagnostic data is temporarily unavailable",
                    )

            def _raw_request_target(self) -> str:
                words = self.requestline.split()
                if len(words) not in (2, 3):
                    return ""
                return words[1]

            def do_HEAD(self) -> None:
                self._method_not_allowed(send_body=False)

            def do_POST(self) -> None:
                self._method_not_allowed()

            def do_PUT(self) -> None:
                self._method_not_allowed()

            def do_DELETE(self) -> None:
                self._method_not_allowed()

            def do_PATCH(self) -> None:
                self._method_not_allowed()

            def do_OPTIONS(self) -> None:
                self._method_not_allowed()

            def do_TRACE(self) -> None:
                self._method_not_allowed()

            def do_CONNECT(self) -> None:
                self._method_not_allowed()

            def _method_not_allowed(self, *, send_body: bool = True) -> None:
                if not self._valid_host():
                    self._send_json_error(
                        HTTPStatus.BAD_REQUEST,
                        "invalid Host header",
                        send_body=send_body,
                    )
                    return
                self._send_json_error(
                    HTTPStatus.METHOD_NOT_ALLOWED,
                    "only GET is supported",
                    send_body=send_body,
                    extra_headers={"Allow": "GET"},
                )

            def _valid_host(self) -> bool:
                values = self.headers.get_all("Host", failobj=[])
                if len(values) != 1:
                    return False
                port = owner.address[1]
                return values[0] in {
                    "127.0.0.1:{}".format(port),
                    "localhost:{}".format(port),
                    "[::1]:{}".format(port),
                }

            def _route_get(self, target: _RequestTarget) -> None:
                if target.path == "/api/state":
                    if target.query:
                        self._send_json_error(
                            HTTPStatus.BAD_REQUEST,
                            "query parameters are not accepted",
                        )
                        return
                    self._send_bytes(
                        HTTPStatus.OK,
                        owner._app.state_bytes(),
                        "application/json; charset=utf-8",
                    )
                    return
                if target.path == "/api/attempts":
                    parameters = self._attempt_parameters(target.query)
                    if parameters is None:
                        return
                    self._send_bytes(
                        HTTPStatus.OK,
                        owner._app.attempts_bytes(
                            limit=parameters[0],
                            before=parameters[1],
                        ),
                        "application/json; charset=utf-8",
                    )
                    return
                detail_match = re.fullmatch(
                    r"/api/attempts/([1-9][0-9]*)",
                    target.path,
                )
                if detail_match is not None:
                    if target.query:
                        self._send_json_error(
                            HTTPStatus.BAD_REQUEST,
                            "query parameters are not accepted",
                        )
                        return
                    body = owner._app.attempt_bytes(int(detail_match.group(1)))
                    if body is None:
                        self._send_json_error(
                            HTTPStatus.NOT_FOUND,
                            "attempt not found",
                        )
                    else:
                        self._send_bytes(
                            HTTPStatus.OK,
                            body,
                            "application/json; charset=utf-8",
                        )
                    return
                if target.path == "/events":
                    if target.query:
                        self._send_json_error(
                            HTTPStatus.BAD_REQUEST,
                            "query parameters are not accepted",
                        )
                        return
                    self._serve_events()
                    return
                resource = owner._app.static_resource(target.path)
                if resource is None:
                    self._send_json_error(
                        HTTPStatus.NOT_FOUND,
                        "route not found",
                    )
                    return
                self._send_bytes(
                    HTTPStatus.OK,
                    resource.body,
                    resource.content_type,
                )

            def _attempt_parameters(
                self,
                query: str,
            ) -> Optional[Tuple[int, Optional[int]]]:
                if not query:
                    return 50, None
                try:
                    _strict_unquote(query)
                    pairs = parse_qsl(
                        query,
                        keep_blank_values=True,
                        strict_parsing=True,
                        encoding="utf-8",
                        errors="strict",
                        max_num_fields=8,
                    )
                except (UnicodeDecodeError, ValueError):
                    self._send_json_error(
                        HTTPStatus.BAD_REQUEST,
                        "invalid attempt query",
                    )
                    return None
                values: Dict[str, str] = {}
                for key, value in pairs:
                    if key not in ("limit", "before") or key in values:
                        self._send_json_error(
                            HTTPStatus.BAD_REQUEST,
                            "invalid attempt query",
                        )
                        return None
                    values[key] = value
                limit_text = values.get("limit", "50")
                before_text = values.get("before")
                if _POSITIVE_INTEGER.fullmatch(limit_text) is None:
                    self._send_json_error(
                        HTTPStatus.BAD_REQUEST,
                        "limit must be an integer from 1 through 200",
                    )
                    return None
                limit = int(limit_text)
                if limit > 200:
                    self._send_json_error(
                        HTTPStatus.BAD_REQUEST,
                        "limit must be an integer from 1 through 200",
                    )
                    return None
                if (
                    before_text is not None
                    and _POSITIVE_INTEGER.fullmatch(before_text) is None
                ):
                    self._send_json_error(
                        HTTPStatus.BAD_REQUEST,
                        "before must be a positive integer",
                    )
                    return None
                return (
                    limit,
                    None if before_text is None else int(before_text),
                )

            def _serve_events(self) -> None:
                values = self.headers.get_all("Last-Event-ID", failobj=[])
                if len(values) > 1 or (
                    values
                    and _NONNEGATIVE_INTEGER.fullmatch(values[0]) is None
                ):
                    self._send_json_error(
                        HTTPStatus.BAD_REQUEST,
                        "Last-Event-ID must be a non-negative integer",
                    )
                    return
                if owner._closing.is_set() or not owner._sse_slots.acquire(
                    blocking=False
                ):
                    self._send_json_error(
                        HTTPStatus.SERVICE_UNAVAILABLE,
                        "maximum SSE clients reached",
                    )
                    return
                cursor: Optional[EventCursor] = None
                try:
                    after_event_id = None if not values else int(values[0])
                    try:
                        cursor = owner._app._hub.open_cursor(after_event_id)
                    except Exception:
                        self._send_json_error(
                            HTTPStatus.SERVICE_UNAVAILABLE,
                            "event stream is temporarily unavailable",
                        )
                        return
                    if not owner._register_stream(cursor, self.connection):
                        cursor.close()
                        self._send_json_error(
                            HTTPStatus.SERVICE_UNAVAILABLE,
                            "server is shutting down",
                        )
                        return
                    self.send_response(HTTPStatus.OK)
                    self.send_header(
                        "Content-Type",
                        "text/event-stream; charset=utf-8",
                    )
                    self.send_header("Cache-Control", "no-cache")
                    self.send_header("X-Content-Type-Options", "nosniff")
                    self.send_header("Connection", "keep-alive")
                    self.end_headers()
                    self.wfile.flush()
                    next_heartbeat = owner._clock() + owner._config.heartbeat_s
                    bootstrap = cursor.bootstrap
                    if bootstrap.mode == "fresh":
                        self._write_sse(
                            bootstrap.watermark_event_id,
                            "ready",
                            {
                                "watermark_event_id": (
                                    bootstrap.watermark_event_id
                                )
                            },
                        )
                    elif bootstrap.mode == "reset":
                        self._write_sse(
                            bootstrap.watermark_event_id,
                            "reset",
                            {
                                "watermark_event_id": (
                                    bootstrap.watermark_event_id
                                )
                            },
                        )

                    while not owner._closing.is_set():
                        now = owner._clock()
                        read = cursor.read(
                            limit=128,
                            timeout_s=min(
                                _SSE_DISCONNECT_POLL_S,
                                max(0.0, next_heartbeat - now),
                            ),
                        )
                        if self._sse_peer_closed():
                            self.close_connection = True
                            break
                        if read.lost_event_ids is not None:
                            self._write_sse(
                                read.watermark_event_id,
                                "reset",
                                {
                                    "watermark_event_id": (
                                        read.watermark_event_id
                                    )
                                },
                            )
                        else:
                            for event in read.events:
                                self._write_sse(
                                    event.event_id,
                                    "invalidate",
                                    {
                                        "scope": event.scope,
                                        "attempt_id": event.attempt_id,
                                    },
                                )
                        now = owner._clock()
                        if now >= next_heartbeat:
                            watermark = (
                                owner._app._hub.state_snapshot().watermark_event_id
                            )
                            self._write_sse(
                                watermark,
                                "heartbeat",
                                {"watermark_event_id": watermark},
                            )
                            next_heartbeat = now + owner._config.heartbeat_s
                except (
                    BrokenPipeError,
                    ConnectionResetError,
                    RuntimeError,
                    socket.timeout,
                ):
                    self.close_connection = True
                finally:
                    if cursor is not None:
                        owner._unregister_stream(cursor, self.connection)
                        cursor.close()
                    owner._sse_slots.release()

            def _sse_peer_closed(self) -> bool:
                try:
                    readable, _writable, _exceptional = select.select(
                        (self.connection,),
                        (),
                        (),
                        0.0,
                    )
                    if not readable:
                        return False
                    return (
                        self.connection.recv(
                            1,
                            socket.MSG_PEEK | socket.MSG_DONTWAIT,
                        )
                        == b""
                    )
                except BlockingIOError:
                    return False
                except (OSError, ValueError):
                    return True

            def _write_sse(
                self,
                event_id: int,
                event_name: str,
                data: Mapping[str, object],
            ) -> None:
                encoded = _json_bytes(data)
                frame = (
                    "id: {}\n"
                    "event: {}\n"
                    "data: ".format(event_id, event_name).encode("utf-8")
                    + encoded
                    + b"\n\n"
                )
                self.wfile.write(frame)
                self.wfile.flush()

            def _send_json_error(
                self,
                status: int,
                message: str,
                *,
                send_body: bool = True,
                extra_headers: Optional[Mapping[str, str]] = None,
            ) -> None:
                self._send_bytes(
                    status,
                    _error_bytes(status, _error_code(status), message),
                    "application/json; charset=utf-8",
                    send_body=send_body,
                    extra_headers=extra_headers,
                )

            def _send_bytes(
                self,
                status: int,
                body: bytes,
                content_type: str,
                *,
                send_body: bool = True,
                extra_headers: Optional[Mapping[str, str]] = None,
            ) -> None:
                self.send_response(status)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Content-Type-Options", "nosniff")
                if extra_headers is not None:
                    for name, value in extra_headers.items():
                        self.send_header(name, value)
                self.send_header("Connection", "close")
                self.end_headers()
                if send_body:
                    self.wfile.write(body)
                    self.wfile.flush()
                self.close_connection = True

        return Handler
