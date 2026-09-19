from __future__ import annotations

from html.parser import HTMLParser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import threading
import unittest


STATIC_HTML = (
    Path(__file__).resolve().parents[1]
    / "diagnostics"
    / "static"
    / "hitter_task_monitor.html"
)


class _LandmarkParser(HTMLParser):
    def __init__(self):
        super().__init__()
        self.ids = set()
        self.tags = []

    def handle_starttag(self, tag, attrs):
        values = dict(attrs)
        if "id" in values:
            self.ids.add(values["id"])
        self.tags.append((tag, values))


class _TextByIdParser(HTMLParser):
    def __init__(self):
        super().__init__()
        self._active_ids = []
        self._text = {}

    def handle_starttag(self, _tag, attrs):
        values = dict(attrs)
        inherited_id = self._active_ids[-1] if self._active_ids else None
        active_id = values.get("id", inherited_id)
        self._active_ids.append(active_id)
        if active_id is not None:
            self._text.setdefault(active_id, [])

    def handle_endtag(self, _tag):
        if self._active_ids:
            self._active_ids.pop()

    def handle_data(self, data):
        if self._active_ids and self._active_ids[-1] is not None:
            self._text[self._active_ids[-1]].append(data)

    def text_for(self, element_id):
        return "".join(self._text.get(element_id, ())).strip()


def _frontend_state(*, lcm_connected=True, warnings=()):
    return {
        "schema_version": 1,
        "watermark_event_id": 7,
        "health": {
            "lcm_connected": lcm_connected,
            "message_rate_hz_by_subject": {},
            "message_age_s_by_subject": {},
            "source_frame_by_subject": {},
            "pelvis_valid": True,
            "planner_submitted": 0,
            "planner_completed": 0,
            "planner_failed": 0,
            "warnings": list(warnings),
            "config_name": "fixture",
            "session_basename": "fixture",
        },
        "lifecycle": {"phase": "waiting"},
        "current_attempt": None,
        "recent_attempts": [],
    }


class _FrontendFixtureHandler(BaseHTTPRequestHandler):
    html = b""
    marker = "runtime-fixture-73b41"
    malicious = '<img id="diagnostic-xss-sentinel">'

    def log_message(self, _format, *_args):
        return

    def _send(self, status, content_type, body):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(body)
        self.close_connection = True

    def do_GET(self):
        if self.path == "/":
            self._send(200, "text/html; charset=utf-8", self.html)
            return
        if self.path == "/api/state":
            payload = {
                "schema_version": 1,
                "watermark_event_id": 7,
                "health": {
                    "lcm_connected": True,
                    "message_rate_hz_by_subject": {},
                    "message_age_s_by_subject": {},
                    "source_frame_by_subject": {},
                    "pelvis_valid": True,
                    "planner_submitted": 0,
                    "planner_completed": 0,
                    "planner_failed": 0,
                    "config_name": self.marker + self.malicious,
                    "session_basename": "fixture",
                },
                "lifecycle": {"phase": "waiting"},
                "current_attempt": None,
                "recent_attempts": [],
            }
            body = json.dumps(payload).encode("utf-8")
            self._send(200, "application/json", body)
            return
        if self.path.startswith("/api/attempts?"):
            body = json.dumps(
                {
                    "schema_version": 1,
                    "items": [],
                    "has_more": False,
                    "next_before": None,
                }
            ).encode("utf-8")
            self._send(200, "application/json", body)
            return
        if self.path == "/events":
            # 204 tells EventSource not to reconnect, so headless DOM dumping
            # reaches a finite network-idle state.
            self._send(204, "text/event-stream", b"")
            return
        self._send(404, "application/json", b'{"error":"not found"}')


class HitterTaskFrontendContractTest(unittest.TestCase):
    def setUp(self):
        self.source = STATIC_HTML.read_text(encoding="utf-8")
        self.parser = _LandmarkParser()
        self.parser.feed(self.source)

    def _render_state_in_headless_browser(self, state):
        chrome = shutil.which("google-chrome-stable")
        if chrome is None:
            self.skipTest("google-chrome-stable is not installed")
        startup = (
            "    scheduleStateRefresh();\n"
            "    loadHistory(null, false).catch(() => {});\n"
            "    connectEvents();"
        )
        fixture_source = self.source.replace(
            startup,
            "    renderState({});".format(
                json.dumps(state, ensure_ascii=False)
            ),
        )
        self.assertNotEqual(fixture_source, self.source)
        with tempfile.TemporaryDirectory(
            prefix="hitter-task-frontend-state-"
        ) as directory:
            fixture_path = Path(directory) / "index.html"
            fixture_path.write_text(fixture_source, encoding="utf-8")
            result = subprocess.run(
                [
                    chrome,
                    "--headless=new",
                    "--disable-gpu",
                    "--disable-background-networking",
                    "--disable-component-update",
                    "--disable-default-apps",
                    "--disable-dev-shm-usage",
                    "--disable-extensions",
                    "--disable-sync",
                    "--metrics-recording-only",
                    "--no-first-run",
                    "--no-sandbox",
                    "--no-proxy-server",
                    "--proxy-bypass-list=*",
                    "--dump-dom",
                    fixture_path.as_uri(),
                ],
                timeout=10,
                check=True,
                capture_output=True,
                text=True,
            )
        parser = _TextByIdParser()
        parser.feed(result.stdout)
        return result.stdout, parser

    def test_page_has_required_health_progress_history_and_detail_regions(self):
        required = {
            "connection-status",
            "health-grid",
            "current-attempt",
            "progress-stages",
            "attempt-history",
            "attempt-detail",
            "ab-comparison",
            "load-more",
        }
        self.assertTrue(required.issubset(self.parser.ids))
        self.assertTrue(
            any(tag == "main" for tag, _attrs in self.parser.tags)
        )
        self.assertTrue(
            any(tag == "table" for tag, _attrs in self.parser.tags)
        )

    def test_six_canonical_stages_and_shadow_disclaimer_are_literal(self):
        for text in (
            "球检测",
            "ESTIMATING",
            "PLANNER-READY INCOMING",
            "PLANNER",
            "ARMED",
            "SHADOW 3/100 TASK OBS",
        ):
            self.assertIn(text, self.source)
        self.assertIn("只读旁路诊断", self.source)
        self.assertIn("不会向机器人发布控制命令", self.source)

    def test_frontend_has_no_unsafe_or_external_mutation_paths(self):
        lowered = self.source.lower()
        self.assertNotIn("innerhtml", lowered)
        self.assertNotIn("document.write", lowered)
        self.assertNotRegex(
            lowered,
            r"<(?:script|link)[^>]+https?://",
        )
        self.assertNotRegex(
            lowered,
            r"(?:method\s*:\s*['\"](?:post|put|delete)|"
            r"fetch\s*\([^)]*,[^)]*(?:post|put|delete))",
        )
        self.assertNotIn("access-control-allow-origin", lowered)

    def test_dynamic_rendering_uses_text_content_and_sse_as_invalidator(self):
        self.assertIn(".textContent", self.source)
        self.assertIn("new EventSource('/events')", self.source)
        self.assertIn("addEventListener('ready'", self.source)
        self.assertIn("addEventListener('invalidate'", self.source)
        self.assertIn("addEventListener('reset'", self.source)
        self.assertIn("fetch('/api/state'", self.source)
        self.assertNotRegex(
            self.source,
            re.compile(
                r"addEventListener\([^)]*(?:ready|invalidate|reset)"
                r"[\s\S]{0,240}JSON\.parse",
            ),
        )

    def test_state_refresh_is_rate_limited_and_rejects_stale_watermarks(self):
        self.assertIn("MIN_REQUEST_INTERVAL_MS = 100", self.source)
        self.assertIn("REQUEST_WINDOW_MS = 1000", self.source)
        self.assertIn("MAX_REQUESTS_PER_WINDOW = 10", self.source)
        self.assertIn("renderedWatermark", self.source)
        self.assertRegex(
            self.source,
            r"watermark_event_id\)?\s*<\s*renderedWatermark",
        )
        self.assertIn("refreshPending", self.source)

    def test_recovery_reacquire_and_tail_states_are_supported(self):
        for state in (
            "WAITING_FOR_PREVIOUS_RECOVERY",
            "CACHED_DURING_RECOVERY",
            "REACQUIRE_GRACE",
            "POST_DEADLINE_TAIL",
        ):
            self.assertIn(state, self.source)

    def test_live_counts_can_be_read_from_attempt_stage_projection(self):
        self.assertIn(
            "stageText.match(/(\\d+)\\s*\\/\\s*31/)",
            self.source,
        )
        self.assertIn(
            "stageText.match(/(\\d+)\\s*\\/\\s*3/)",
            self.source,
        )

    def test_detail_renders_observation_vectors_and_ab_deltas(self):
        for token in (
            "task_observation_pre_clip",
            "task_observation_post_clip",
            "primary_blocker",
            "planner_inputs",
            "planner_results",
            "variant_outcomes",
            "ab_deltas",
        ):
            self.assertIn(token, self.source)

    def test_headless_health_warnings_are_rendered_as_text(self):
        malicious = '<img id="health-warning-xss-sentinel">'
        output, parser = self._render_state_in_headless_browser(
            _frontend_state(
                warnings=("SOURCE_FRAME_GAP", malicious),
            )
        )

        warning_text = parser.text_for("health-warnings")
        self.assertIn("SOURCE_FRAME_GAP", warning_text)
        self.assertIn(malicious, warning_text)
        landmarks = _LandmarkParser()
        landmarks.feed(output)
        self.assertNotIn("health-warning-xss-sentinel", landmarks.ids)

    def test_headless_connection_is_offline_when_lcm_flag_is_false(self):
        _output, parser = self._render_state_in_headless_browser(
            _frontend_state(lcm_connected=False)
        )

        self.assertIn(
            "LCM 数据离线",
            parser.text_for("connection-status"),
        )

    def test_headless_connection_is_offline_when_heartbeat_is_stale(self):
        _output, parser = self._render_state_in_headless_browser(
            _frontend_state(
                lcm_connected=True,
                warnings=("LCM_HEARTBEAT_STALE",),
            )
        )

        status_text = parser.text_for("connection-status")
        self.assertIn("LCM 数据离线", status_text)
        self.assertIn("心跳超时", status_text)

    def test_headless_browser_renders_marker_without_creating_injected_node(self):
        chrome = shutil.which("google-chrome-stable")
        if chrome is None:
            self.skipTest("google-chrome-stable is not installed")

        fixture_state = {
            "schema_version": 1,
            "watermark_event_id": 7,
            "health": {
                "lcm_connected": True,
                "message_rate_hz_by_subject": {},
                "message_age_s_by_subject": {},
                "source_frame_by_subject": {},
                "pelvis_valid": True,
                "planner_submitted": 0,
                "planner_completed": 0,
                "planner_failed": 0,
                "config_name": (
                    _FrontendFixtureHandler.marker
                    + _FrontendFixtureHandler.malicious
                ),
                "session_basename": "fixture",
            },
            "lifecycle": {"phase": "waiting"},
            "current_attempt": None,
            "recent_attempts": [],
        }
        fixture_source = self.source.replace(
            (
                "    scheduleStateRefresh();\n"
                "    loadHistory(null, false).catch(() => {});\n"
                "    connectEvents();"
            ),
            "    renderState({});".format(
                json.dumps(fixture_state, ensure_ascii=False)
            ),
        )
        _FrontendFixtureHandler.html = fixture_source.encode("utf-8")
        server = ThreadingHTTPServer(
            ("127.0.0.1", 0),
            _FrontendFixtureHandler,
        )
        thread = threading.Thread(
            target=server.serve_forever,
            name="frontend-fixture",
            daemon=True,
        )
        thread.start()
        try:
            profile = tempfile.mkdtemp(prefix="hitter-task-frontend-")
            try:
                try:
                    result = subprocess.run(
                        [
                            chrome,
                            "--headless=new",
                            "--disable-gpu",
                            "--disable-background-networking",
                            "--disable-component-update",
                            "--disable-default-apps",
                            "--disable-dev-shm-usage",
                            "--disable-extensions",
                            "--disable-sync",
                            "--metrics-recording-only",
                            "--no-first-run",
                            "--no-sandbox",
                            "--no-zygote",
                            "--no-proxy-server",
                            "--proxy-bypass-list=*",
                            "--remote-debugging-port=0",
                            "--single-process",
                            "--user-data-dir=" + profile,
                            "--dump-dom",
                            "http://127.0.0.1:{}/".format(
                                server.server_address[1]
                            ),
                        ],
                        timeout=10,
                        check=True,
                        capture_output=True,
                        text=True,
                    )
                except subprocess.TimeoutExpired:
                    self.skipTest(
                        "installed Chrome cannot finish localhost "
                        "headless navigation within 10 seconds"
                    )
            finally:
                shutil.rmtree(profile, ignore_errors=True)
            self.assertIn(_FrontendFixtureHandler.marker, result.stdout)
            parser = _LandmarkParser()
            parser.feed(result.stdout)
            self.assertNotIn("diagnostic-xss-sentinel", parser.ids)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(2.0)


if __name__ == "__main__":
    unittest.main()
