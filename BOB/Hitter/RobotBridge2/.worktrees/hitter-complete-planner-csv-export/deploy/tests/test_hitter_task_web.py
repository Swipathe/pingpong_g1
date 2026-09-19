from __future__ import annotations

import http.client
import json
from dataclasses import replace
from pathlib import Path
import socket
import struct
import tempfile
import threading
from typing import Dict, Iterable, Optional, Tuple
import unittest

from diagnostics.hitter_task_events import DiagnosticState, EventHub
from diagnostics.hitter_task_models import (
    AttemptDetail,
    AttemptSummary,
    EventDraft,
    HealthSnapshot,
    LifecycleSnapshot,
)
from diagnostics.hitter_task_recording import AttemptDetailRepository
from diagnostics.hitter_task_web import (
    HitterTaskWebApplication,
    HitterTaskWebServer,
    WebServerConfig,
)


def _health() -> HealthSnapshot:
    return HealthSnapshot(
        lcm_connected=True,
        message_rate_hz_by_subject={"ball": 360.0},
        message_age_s_by_subject={"ball": 0.01},
        source_frame_by_subject={"ball": 7},
        pelvis_valid=True,
        pelvis_age_s=0.02,
        planner_submitted=2,
        planner_completed=1,
        planner_failed=0,
        planner_dropped_pending=0,
        planner_results_overwritten_before_consume=0,
        raw_samples_dropped=0,
        recorder_event_gaps=0,
        diagnostic_events_dropped=0,
        recorder_healthy=True,
        recording_complete=True,
        config_name="hitter",
        session_basename="test-session",
        warnings=(),
    )


def _lifecycle() -> LifecycleSnapshot:
    return LifecycleSnapshot(
        phase="TRACKING",
        last_decision="NONE",
        active_key=None,
        cached_key=None,
        lifecycle_now_s=12.5,
        obs_now_s=12.51,
    )


def _hub(*, capacity: int = 32) -> EventHub:
    return EventHub(
        DiagnosticState(
            schema_version=1,
            watermark_event_id=0,
            health=_health(),
            lifecycle=_lifecycle(),
            current_attempt=None,
            recent_attempts=(),
        ),
        capacity=capacity,
    )


def _summary(attempt_id: int) -> AttemptSummary:
    return AttemptSummary(
        attempt_id=attempt_id,
        status="FAILED",
        stage="ATTEMPT_CLOSED",
        primary_blocker="TRACK_ENDED_BEFORE_READY",
        ball_speed_mps=4.25,
        predicted_strike_time_s=None,
        planner_tts_s=None,
        arm_tts_s=None,
        task_obs_status="FAILED",
        ab_summary=None,
        recording_complete=True,
    )


def _detail(attempt_id: int) -> AttemptDetail:
    return AttemptDetail(
        attempt_id=attempt_id,
        summary=_summary(attempt_id),
        segments=({"track_segment_id": attempt_id},),
        stage_timeline=({"stage": "DETECTED"},),
        planner_inputs=(),
        planner_results=(),
        task_observation_pre_clip=None,
        task_observation_post_clip=None,
        task_observation_clip_count=None,
        variant_outcomes=(),
        ab_deltas={},
    )


def _draft(
    sequence: int,
    *,
    scope: str = "attempt",
    attempt_id: Optional[int] = 7,
) -> EventDraft:
    return EventDraft(
        kind="PROGRESS",
        monotonic_s=float(sequence),
        wall_time_us=sequence,
        scope=scope,
        attempt_id=attempt_id,
        payload={
            "private_state_patch": {
                "stage": "this must never be sent through SSE",
            }
        },
    )


class _ObservedHub(EventHub):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.cursor_closed = threading.Event()

    def _close_cursor(self, cursor) -> None:
        super()._close_cursor(cursor)
        self.cursor_closed.set()


class _SteppingClock:
    def __init__(self, values: Iterable[float]) -> None:
        self._values = iter(values)
        self._last = 0.0
        self._lock = threading.Lock()

    def __call__(self) -> float:
        with self._lock:
            try:
                self._last = float(next(self._values))
            except StopIteration:
                pass
            return self._last


def _read_sse(response: http.client.HTTPResponse) -> Tuple[int, str, Dict[str, object]]:
    fields: Dict[str, str] = {}
    while True:
        line = response.readline()
        if line == b"":
            raise EOFError("SSE stream closed before a complete event")
        if line in (b"\n", b"\r\n"):
            break
        name, separator, value = line.decode("utf-8").rstrip("\r\n").partition(":")
        if separator:
            fields[name] = value.lstrip(" ")
    return int(fields["id"]), fields["event"], json.loads(fields["data"])


class HitterTaskWebTests(unittest.TestCase):
    def setUp(self) -> None:
        self._temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self._temporary.cleanup)
        self._root = Path(self._temporary.name)
        self._index = self._root / "index.html"
        self._script = self._root / "monitor.js"
        self._index.write_bytes(b"<!doctype html><title>HITTER</title>")
        self._script.write_bytes(b"window.HITTER = true;\n")

    def _server(
        self,
        *,
        hub: Optional[EventHub] = None,
        repository: Optional[AttemptDetailRepository] = None,
        config: Optional[WebServerConfig] = None,
        clock=None,
    ) -> Tuple[EventHub, AttemptDetailRepository, HitterTaskWebServer]:
        selected_hub = _hub() if hub is None else hub
        selected_repository = (
            AttemptDetailRepository(self._root / "details")
            if repository is None
            else repository
        )
        app = HitterTaskWebApplication(
            hub=selected_hub,
            attempts=selected_repository,
            static_files={
                "/": self._index,
                "/monitor.js": self._script,
            },
        )
        kwargs = {} if clock is None else {"clock": clock}
        server = HitterTaskWebServer(
            app,
            WebServerConfig(port=0) if config is None else config,
            **kwargs,
        )
        server.start()
        self.addCleanup(server.close)
        return selected_hub, selected_repository, server

    @staticmethod
    def _host_header(server: HitterTaskWebServer, name: str = "127.0.0.1") -> str:
        return "{}:{}".format(name, server.address[1])

    def _request(
        self,
        server: HitterTaskWebServer,
        target: str,
        *,
        method: str = "GET",
        host: Optional[str] = None,
    ) -> Tuple[int, http.client.HTTPMessage, bytes]:
        connection = http.client.HTTPConnection(
            server.address[0],
            server.address[1],
            timeout=2.0,
        )
        connection.request(
            method,
            target,
            headers={
                "Host": (
                    self._host_header(server)
                    if host is None
                    else host
                )
            },
        )
        response = connection.getresponse()
        body = response.read()
        result = response.status, response.headers, body
        connection.close()
        return result

    def _open_sse(
        self,
        server: HitterTaskWebServer,
        *,
        last_event_id: Optional[str] = None,
    ) -> Tuple[http.client.HTTPConnection, http.client.HTTPResponse]:
        connection = http.client.HTTPConnection(
            server.address[0],
            server.address[1],
            timeout=2.0,
        )
        headers = {"Host": self._host_header(server)}
        if last_event_id is not None:
            headers["Last-Event-ID"] = last_event_id
        connection.request("GET", "/events", headers=headers)
        return connection, connection.getresponse()

    def _raw_request(
        self,
        server: HitterTaskWebServer,
        target: str,
    ) -> Tuple[int, http.client.HTTPMessage, bytes]:
        client = socket.create_connection(server.address, timeout=2.0)
        try:
            client.sendall(
                (
                    "GET {} HTTP/1.1\r\n"
                    "Host: {}\r\n"
                    "Connection: close\r\n"
                    "\r\n"
                ).format(target, self._host_header(server)).encode("ascii")
            )
            response = http.client.HTTPResponse(client)
            response.begin()
            body = response.read()
            return response.status, response.headers, body
        finally:
            client.close()

    def assert_common_headers(
        self,
        headers: http.client.HTTPMessage,
        body: bytes,
        *,
        cache_control: str = "no-store",
    ) -> None:
        self.assertEqual(headers["Content-Length"], str(len(body)))
        self.assertEqual(headers["Cache-Control"], cache_control)
        self.assertEqual(headers["X-Content-Type-Options"], "nosniff")
        self.assertIsNone(headers["Access-Control-Allow-Origin"])

    def assert_json_error(
        self,
        status: int,
        headers: http.client.HTTPMessage,
        body: bytes,
    ) -> None:
        self.assertIn(status, (400, 404, 405, 503))
        self.assertEqual(headers.get_content_type(), "application/json")
        self.assert_common_headers(headers, body)
        value = json.loads(body)
        self.assertEqual(value["schema_version"], 1)
        self.assertEqual(set(value), {"schema_version", "error"})
        self.assertEqual(
            set(value["error"]),
            {"status", "code", "message"},
        )
        self.assertEqual(value["error"]["status"], status)
        self.assertIsInstance(value["error"]["code"], str)
        self.assertIsInstance(value["error"]["message"], str)

    def test_binds_only_loopback_and_reports_the_ephemeral_address(self) -> None:
        self.assertEqual(WebServerConfig().request_timeout_s, 5.0)
        self.assertEqual(WebServerConfig().max_handlers, 8)
        self.assertEqual(WebServerConfig().max_sse_clients, 4)
        for host in ("0.0.0.0", "::", "", "example.test"):
            with self.subTest(host=host):
                with self.assertRaises(ValueError):
                    WebServerConfig(host=host)

        _hub_value, _repository, server = self._server()
        self.assertEqual(server.address[0], "127.0.0.1")
        self.assertGreater(server.address[1], 0)
        self.assertTrue(server.close(timeout_s=1.0))
        self.assertTrue(server.close(timeout_s=0.0))

    def test_state_and_fixed_static_files_have_complete_safe_headers(self) -> None:
        _hub_value, _repository, server = self._server()
        status, headers, body = self._request(server, "/api/state")
        self.assertEqual(status, 200)
        self.assertEqual(headers.get_content_type(), "application/json")
        self.assert_common_headers(headers, body)
        state = json.loads(body)
        self.assertEqual(
            set(state),
            {
                "schema_version",
                "watermark_event_id",
                "health",
                "lifecycle",
                "current_attempt",
                "recent_attempts",
            },
        )
        self.assertEqual(state["schema_version"], 1)
        self.assertEqual(state["health"]["config_name"], "hitter")

        for target, expected, content_type in (
            ("/", self._index.read_bytes(), "text/html"),
            ("/monitor.js", self._script.read_bytes(), "text/javascript"),
        ):
            with self.subTest(target=target):
                status, headers, body = self._request(server, target)
                self.assertEqual(status, 200)
                self.assertEqual(body, expected)
                self.assertEqual(headers.get_content_type(), content_type)
                self.assert_common_headers(headers, body)

    def test_real_monitor_page_is_served_only_from_the_fixed_root_route(self) -> None:
        monitor = (
            Path(__file__).resolve().parents[1]
            / "diagnostics"
            / "static"
            / "hitter_task_monitor.html"
        )
        app = HitterTaskWebApplication(
            hub=_hub(),
            attempts=AttemptDetailRepository(self._root / "monitor-details"),
            static_files={"/": monitor},
        )
        server = HitterTaskWebServer(app, WebServerConfig(port=0))
        server.start()
        self.addCleanup(server.close)

        status, headers, body = self._request(server, "/")
        self.assertEqual(status, 200)
        self.assertEqual(body, monitor.read_bytes())
        self.assertEqual(headers.get_content_type(), "text/html")
        self.assert_common_headers(headers, body)
        hidden = self._request(
            server,
            "/diagnostics/static/hitter_task_monitor.html",
        )
        self.assertEqual(hidden[0], 404)
        self.assert_json_error(*hidden)

    def test_attempts_are_descending_exclusive_and_strictly_paginated(self) -> None:
        repository = AttemptDetailRepository(self._root / "paged-details")
        for attempt_id in range(1, 56):
            repository.commit(_detail(attempt_id))
        _hub_value, _repository, server = self._server(repository=repository)

        status, _headers, body = self._request(server, "/api/attempts")
        self.assertEqual(status, 200)
        page = json.loads(body)
        self.assertEqual(
            [item["attempt_id"] for item in page["items"]],
            list(range(55, 5, -1)),
        )
        self.assertTrue(page["has_more"])
        self.assertEqual(page["next_before"], 6)

        status, _headers, body = self._request(
            server,
            "/api/attempts?before=6&limit=3",
        )
        self.assertEqual(status, 200)
        page = json.loads(body)
        self.assertEqual(
            [item["attempt_id"] for item in page["items"]],
            [5, 4, 3],
        )
        self.assertTrue(page["has_more"])
        self.assertEqual(page["next_before"], 3)

        status, _headers, body = self._request(
            server,
            "/api/attempts?limit=200",
        )
        self.assertEqual(status, 200)
        self.assertEqual(len(json.loads(body)["items"]), 55)

        for query in (
            "limit=0",
            "limit=201",
            "limit=-1",
            "limit=+1",
            "limit=1.0",
            "limit=true",
            "limit=01",
            "before=0",
            "before=-1",
            "before=true",
            "limit=1&limit=2",
            "unknown=1",
            "limit=",
        ):
            with self.subTest(query=query):
                result = self._request(server, "/api/attempts?" + query)
                self.assertEqual(result[0], 400)
                self.assert_json_error(*result)

    def test_attempt_list_and_detail_preserve_truthful_progress_counts(self) -> None:
        writer = AttemptDetailRepository(self._root / "progress-details")
        detail = _detail(7)
        writer.commit(
            replace(
                detail,
                summary=replace(
                    detail.summary,
                    estimator_sample_count=19,
                    estimator_window_size=31,
                    incoming_count=2,
                    incoming_required_count=3,
                ),
            )
        )
        reader = AttemptDetailRepository(self._root / "progress-details")
        _hub_value, _repository, server = self._server(repository=reader)

        list_status, _list_headers, list_body = self._request(
            server,
            "/api/attempts",
        )
        detail_status, _detail_headers, detail_body = self._request(
            server,
            "/api/attempts/7",
        )

        self.assertEqual(list_status, 200)
        self.assertEqual(detail_status, 200)
        listed = json.loads(list_body)["items"][0]
        detailed = json.loads(detail_body)["summary"]
        for field, expected in (
            ("estimator_sample_count", 19),
            ("estimator_window_size", 31),
            ("incoming_count", 2),
            ("incoming_required_count", 3),
        ):
            with self.subTest(field=field):
                self.assertEqual(listed[field], expected)
                self.assertEqual(detailed[field], expected)

    def test_legacy_attempt_progress_is_null_in_list_and_detail(self) -> None:
        details_dir = self._root / "legacy-details"
        details_dir.mkdir()
        payload = _detail(8).to_json_dict()
        progress_fields = (
            "estimator_sample_count",
            "estimator_window_size",
            "incoming_count",
            "incoming_required_count",
        )
        for field in progress_fields:
            payload["summary"].pop(field)
        (details_dir / "8.json").write_text(
            json.dumps(payload),
            encoding="utf-8",
        )
        repository = AttemptDetailRepository(details_dir)
        _hub_value, _repository, server = self._server(repository=repository)

        list_status, _list_headers, list_body = self._request(
            server,
            "/api/attempts",
        )
        detail_status, _detail_headers, detail_body = self._request(
            server,
            "/api/attempts/8",
        )

        self.assertEqual(list_status, 200)
        self.assertEqual(detail_status, 200)
        listed = json.loads(list_body)["items"][0]
        detailed = json.loads(detail_body)["summary"]
        for field in progress_fields:
            with self.subTest(field=field):
                self.assertIsNone(listed[field])
                self.assertIsNone(detailed[field])

    def test_detail_uses_only_numeric_repository_ids_and_leaks_no_paths(self) -> None:
        writer = AttemptDetailRepository(self._root / "disk-details")
        writer.commit(
            replace(
                _detail(7),
                segments=(
                    {
                        "track_segment_id": 7,
                        "session_path": str(self._root / "private-session"),
                        "download_url": "file:///private-session/7.json",
                        "filepath": "/private/replay/input.json",
                        "error": "replay failed /private/replay/input.json",
                        "/private/key.json": "secret",
                        "arrow_error": "failed->/private/edge.json",
                        "backtick_error": "`/private/backtick.json`",
                        "angle_error": "</private/angle.json>",
                        "pipe_error": "failed|/private/pipe.json",
                        "unc_error": r"failed->\\private\share\input.json",
                        "safe_ratio": "3/100",
                    },
                ),
            )
        )
        (self._root / "disk-details" / "not-an-id.json").write_text(
            json.dumps({"attempt_id": 999}),
            encoding="utf-8",
        )
        reader = AttemptDetailRepository(self._root / "disk-details")
        _hub_value, _repository, server = self._server(repository=reader)

        status, headers, body = self._request(server, "/api/attempts/7")
        self.assertEqual(status, 200)
        self.assert_common_headers(headers, body)
        value = json.loads(body)
        self.assertEqual(value["attempt_id"], 7)
        self.assertNotIn(str(self._root), body.decode("utf-8"))
        self.assertNotIn("/private/", body.decode("utf-8"))
        self.assertNotIn("download", body.decode("utf-8").lower())
        self.assertNotIn("session_path", body.decode("utf-8").lower())
        self.assertNotIn("filepath", body.decode("utf-8").lower())
        self.assertNotIn("/private/key.json", body.decode("utf-8"))
        self.assertNotIn("/private/edge.json", body.decode("utf-8"))
        self.assertNotIn("/private/backtick.json", body.decode("utf-8"))
        self.assertNotIn("/private/angle.json", body.decode("utf-8"))
        self.assertNotIn("/private/pipe.json", body.decode("utf-8"))
        self.assertNotIn("private\\\\share", body.decode("utf-8"))
        self.assertIn("3/100", body.decode("utf-8"))

        for target, expected_status in (
            ("/api/attempts/8", 404),
            ("/api/attempts/0", 404),
            ("/api/attempts/-1", 404),
            ("/api/attempts/true", 404),
            ("/api/attempts/7/extra", 404),
            ("/api/attempts/%2e%2e", 404),
        ):
            with self.subTest(target=target):
                result = self._request(server, target)
                self.assertEqual(result[0], expected_status)
                self.assert_json_error(*result)

    def test_errors_methods_hosts_and_paths_use_one_json_contract(self) -> None:
        _hub_value, _repository, server = self._server()
        port = server.address[1]
        for host in (
            "127.0.0.1",
            "127.0.0.1:1",
            "localhost",
            "LOCALHOST:{}".format(port),
            "evil.test:{}".format(port),
            "[::1]",
        ):
            with self.subTest(host=host):
                result = self._request(server, "/api/state", host=host)
                self.assertEqual(result[0], 400)
                self.assert_json_error(*result)

        for host in (
            "127.0.0.1:{}".format(port),
            "localhost:{}".format(port),
            "[::1]:{}".format(port),
        ):
            with self.subTest(host=host):
                self.assertEqual(
                    self._request(server, "/api/state", host=host)[0],
                    200,
                )

        for target in (
            "/missing",
            "/etc/passwd",
            "/../secret",
            "/%2e%2e/secret",
            "/%252e%252e/secret",
            "//etc/passwd",
            "http://localhost:{}/".format(port),
            r"/..\secret",
        ):
            with self.subTest(target=target):
                result = self._request(server, target)
                self.assertEqual(result[0], 404)
                self.assert_json_error(*result)

        for target in ("//api/state", "///api/state"):
            with self.subTest(raw_target=target):
                result = self._raw_request(server, target)
                self.assertEqual(result[0], 404)
                self.assert_json_error(*result)

        for method in ("POST", "PUT", "DELETE", "PATCH", "OPTIONS"):
            with self.subTest(method=method):
                result = self._request(server, "/api/state", method=method)
                self.assertEqual(result[0], 405)
                self.assert_json_error(*result)
                self.assertEqual(result[1]["Allow"], "GET")

        valid_unknown = self._request(
            server,
            "/api/state",
            method="BREW",
        )
        self.assertEqual(valid_unknown[0], 405)
        self.assert_json_error(*valid_unknown)
        self.assertEqual(valid_unknown[1]["Allow"], "GET")
        invalid_unknown = self._request(
            server,
            "/api/state",
            method="BREW",
            host="evil.test:{}".format(port),
        )
        self.assertEqual(invalid_unknown[0], 400)
        self.assert_json_error(*invalid_unknown)
        self.assertIsNone(invalid_unknown[1]["Allow"])

    def test_sse_initialization_failure_is_complete_json_503(self) -> None:
        hub, _repository, server = self._server()
        hub.close()

        status, headers, body = self._request(server, "/events")

        self.assertEqual(status, 503)
        self.assert_json_error(status, headers, body)

    def test_fresh_sse_sends_ready_then_only_invalidates_full_get_state(self) -> None:
        hub, _repository, server = self._server()
        hub.publish(_draft(1))
        connection, response = self._open_sse(server)
        self.addCleanup(connection.close)
        self.assertEqual(response.status, 200)
        self.assertEqual(response.headers.get_content_type(), "text/event-stream")
        self.assertEqual(response.headers["Cache-Control"], "no-cache")
        self.assertEqual(response.headers["X-Content-Type-Options"], "nosniff")
        self.assertIsNone(response.headers["Access-Control-Allow-Origin"])

        event_id, event_name, data = _read_sse(response)
        self.assertEqual((event_id, event_name), (1, "ready"))
        self.assertEqual(data, {"watermark_event_id": 1})
        status, _headers, body = self._request(server, "/api/state")
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body)["watermark_event_id"], 1)

        published = hub.publish(_draft(2, scope="state", attempt_id=None))
        event_id, event_name, data = _read_sse(response)
        self.assertEqual((event_id, event_name), (published.event_id, "invalidate"))
        self.assertEqual(data, {"scope": "state", "attempt_id": None})
        self.assertNotIn("private_state_patch", data)

    def test_sse_resume_replays_invalidators_and_gap_resets_to_watermark(self) -> None:
        hub = _hub(capacity=2)
        for sequence in range(1, 5):
            hub.publish(_draft(sequence))
        _hub_value, _repository, server = self._server(hub=hub)

        reset_connection, reset_response = self._open_sse(
            server,
            last_event_id="1",
        )
        self.addCleanup(reset_connection.close)
        self.assertEqual(
            _read_sse(reset_response),
            (4, "reset", {"watermark_event_id": 4}),
        )
        published = hub.publish(_draft(5))
        self.assertEqual(
            _read_sse(reset_response),
            (
                published.event_id,
                "invalidate",
                {"scope": "attempt", "attempt_id": 7},
            ),
        )
        reset_connection.close()

        resume_connection, resume_response = self._open_sse(
            server,
            last_event_id="4",
        )
        self.addCleanup(resume_connection.close)
        self.assertEqual(
            _read_sse(resume_response),
            (
                published.event_id,
                "invalidate",
                {"scope": "attempt", "attempt_id": 7},
            ),
        )

        invalid_connection, invalid_response = self._open_sse(
            server,
            last_event_id="true",
        )
        self.addCleanup(invalid_connection.close)
        invalid_body = invalid_response.read()
        self.assertEqual(invalid_response.status, 400)
        self.assert_json_error(
            invalid_response.status,
            invalid_response.headers,
            invalid_body,
        )

    def test_heartbeat_uses_injected_clock_without_wall_clock_sleep(self) -> None:
        clock = _SteppingClock((0.0, 15.0, 15.0, 15.0))
        _hub_value, _repository, server = self._server(clock=clock)
        connection, response = self._open_sse(server)
        self.addCleanup(connection.close)
        self.assertEqual(
            _read_sse(response),
            (0, "ready", {"watermark_event_id": 0}),
        )
        self.assertEqual(
            _read_sse(response),
            (0, "heartbeat", {"watermark_event_id": 0}),
        )

    def test_handler_and_sse_limits_return_503_and_release_on_close(self) -> None:
        total_config = WebServerConfig(
            port=0,
            max_handlers=1,
            max_sse_clients=1,
            heartbeat_s=60.0,
        )
        _hub_value, _repository, total_server = self._server(config=total_config)
        first_connection, first_response = self._open_sse(total_server)
        self.addCleanup(first_connection.close)
        self.assertEqual(_read_sse(first_response)[1], "ready")
        overloaded = self._request(total_server, "/api/state")
        self.assertEqual(overloaded[0], 503)
        self.assert_json_error(*overloaded)
        self.assertTrue(total_server.close(timeout_s=1.0))

        sse_config = WebServerConfig(
            port=0,
            max_handlers=3,
            max_sse_clients=1,
            heartbeat_s=60.0,
        )
        _hub_value, _repository, sse_server = self._server(config=sse_config)
        held_connection, held_response = self._open_sse(sse_server)
        self.addCleanup(held_connection.close)
        self.assertEqual(_read_sse(held_response)[1], "ready")
        rejected_connection, rejected_response = self._open_sse(sse_server)
        rejected_body = rejected_response.read()
        rejected_connection.close()
        self.assertEqual(rejected_response.status, 503)
        self.assert_json_error(
            rejected_response.status,
            rejected_response.headers,
            rejected_body,
        )
        self.assertEqual(self._request(sse_server, "/api/state")[0], 200)

    def test_disconnect_and_server_close_release_cursor_without_polling(self) -> None:
        initial = DiagnosticState(1, 0, _health(), _lifecycle(), None, ())
        hub = _ObservedHub(initial, capacity=8)
        _hub_value, _repository, server = self._server(hub=hub)
        connection, response = self._open_sse(server)
        self.assertEqual(_read_sse(response)[1], "ready")
        self.assertEqual(hub.cursor_count(), 1)

        stream_socket = response.fp.raw._sock
        stream_socket.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_LINGER,
            struct.pack("ii", 1, 0),
        )
        stream_socket.shutdown(socket.SHUT_RDWR)
        stream_socket.close()
        response.close()
        connection.close()
        self.assertTrue(hub.cursor_closed.wait(1.0))
        self.assertEqual(hub.cursor_count(), 0)
        self.assertEqual(self._request(server, "/api/state")[0], 200)

        second_connection, second_response = self._open_sse(server)
        self.addCleanup(second_connection.close)
        self.assertEqual(_read_sse(second_response)[1], "ready")
        self.assertTrue(server.close(timeout_s=1.0))
        self.assertEqual(hub.cursor_count(), 0)

    def test_ordinary_connection_uses_the_configured_socket_timeout(self) -> None:
        config = WebServerConfig(
            port=0,
            request_timeout_s=0.05,
            heartbeat_s=60.0,
        )
        _hub_value, _repository, server = self._server(config=config)
        client = socket.create_connection(server.address, timeout=1.0)
        self.addCleanup(client.close)
        client.settimeout(1.0)
        client.sendall(
            (
                "GET /api/state HTTP/1.1\r\n"
                "Host: {}\r\n".format(self._host_header(server))
            ).encode("ascii")
        )
        self.assertEqual(client.recv(1), b"")


if __name__ == "__main__":
    unittest.main()
