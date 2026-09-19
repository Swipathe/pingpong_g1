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

STATIC_HTML = Path(__file__).resolve().parents[1] / "diagnostics" / "static" / "hitter_task_monitor.html"


class _LandmarkParser(HTMLParser):
    def __init__(self):
        super().__init__()
        self.ids = set()
        self.attrs_by_id = {}
        self.tags = []

    def handle_starttag(self, tag, attrs):
        values = dict(attrs)
        if "id" in values:
            self.ids.add(values["id"])
            self.attrs_by_id[values["id"]] = values
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


def _subject(
    *,
    status,
    rate_hz=None,
    age_s=None,
    source_frame=None,
    valid=None,
    occluded=None,
):
    return {
        "status": status,
        "rate_hz": rate_hz,
        "age_s": age_s,
        "source_frame": source_frame,
        "valid": valid,
        "occluded": occluded,
    }


def _ball_state(**overrides):
    value = {
        "status": "NOT_SEEN",
        "source_frame": None,
        "track_id": None,
        "generation": None,
        "raw_position_w": None,
        "estimated_position_w": None,
        "estimated_velocity_w": None,
        "speed_mps": None,
        "velocity_x_mps": None,
        "estimator_sample_count": None,
        "estimator_window_size": None,
        "estimator_ready": None,
        "last_estimator_reset_reason": None,
        "ball_only_incoming_count": None,
        "ball_only_incoming_required": None,
        "ball_only_incoming_confirmed": None,
        "incoming_status": "NOT_EVALUATED",
        "blocker": None,
        "observed_monotonic_s": None,
        "age_s": None,
    }
    value.update(overrides)
    return value


def _production_gate(**overrides):
    value = {
        "pelvis_status": "UNKNOWN",
        "production_incoming_status": "NOT_EVALUATED",
        "production_incoming_count": None,
        "production_incoming_required": None,
        "planner_status": "BLOCKED",
        "planner_reason_code": None,
        "planner_tts_s": None,
        "arm_status": "NOT_EVALUATED",
        "arm_trigger_tts_s": None,
        "task_observation_status": "NOT_AVAILABLE",
        "task_observation_valid_dimensions": None,
        "task_observation_total_dimensions": None,
        "task_observation_clip_count": None,
    }
    value.update(overrides)
    return value


def _attempt(**overrides):
    value = {
        "schema_version": 1,
        "attempt_id": 22,
        "status": "ACTIVE",
        "stage": "DETECTED",
        "primary_blocker": None,
        "ball_speed_mps": None,
        "predicted_strike_time_s": None,
        "planner_tts_s": None,
        "arm_tts_s": None,
        "task_obs_status": "NOT_AVAILABLE",
        "ab_summary": None,
        "recording_complete": True,
        "estimator_sample_count": None,
        "estimator_window_size": None,
        "incoming_count": None,
        "incoming_required_count": None,
    }
    value.update(overrides)
    return value


def _frontend_state(
    *,
    lcm_connected=True,
    warnings=(),
    subjects=None,
    ball=None,
    production_gate=None,
    current_attempt=None,
    recent_attempts=(),
):
    if subjects is None:
        subjects = {
            "ball": _subject(status="NEVER_SEEN"),
            "g1pelvis": _subject(status="NEVER_SEEN"),
            "table": _subject(status="NEVER_SEEN"),
        }
    if ball is None:
        ball = _ball_state()
    if production_gate is None:
        production_gate = _production_gate()
    return {
        "schema_version": 2,
        "watermark_event_id": 7,
        "health": {
            "schema_version": 1,
            "lcm_connected": lcm_connected,
            "message_rate_hz_by_subject": {},
            "message_age_s_by_subject": {},
            "source_frame_by_subject": {},
            "pelvis_valid": True,
            "pelvis_age_s": None,
            "planner_submitted": 0,
            "planner_completed": 0,
            "planner_failed": 0,
            "planner_dropped_pending": 0,
            "planner_results_overwritten_before_consume": 0,
            "raw_samples_dropped": 0,
            "recorder_event_gaps": 0,
            "diagnostic_events_dropped": 0,
            "recorder_healthy": True,
            "recording_complete": True,
            "warnings": list(warnings),
            "config_name": "fixture",
            "session_basename": "fixture",
        },
        "lifecycle": {
            "schema_version": 1,
            "phase": "waiting",
            "last_decision": "none",
            "active_key": None,
            "cached_key": None,
            "lifecycle_now_s": None,
            "obs_now_s": None,
        },
        "current_attempt": current_attempt,
        "recent_attempts": list(recent_attempts),
        "live_snapshot": {
            "schema_version": 2,
            "revision": 4,
            "captured_monotonic_s": 10.125,
            "subjects": subjects,
            "ball": ball,
            "production_gate": production_gate,
            "current_attempt": current_attempt,
        },
    }


def _legacy_frontend_state():
    state = _frontend_state(
        current_attempt=_attempt(
            stage="TASK_OBS_PASS 11/11",
            estimator_sample_count=31,
            estimator_window_size=31,
            incoming_count=3,
            incoming_required_count=3,
            planner_tts_s=0.0,
            arm_tts_s=0.0,
        )
    )
    state["schema_version"] = 1
    state.pop("live_snapshot")
    return state


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

    def _run_in_headless_browser(self, javascript):
        chrome = shutil.which("google-chrome-stable")
        if chrome is None:
            self.skipTest("google-chrome-stable is not installed")
        startup = (
            "    scheduleStateRefresh();\n" "    loadHistory(null, false).catch(() => {});\n" "    connectEvents();"
        )
        fixture_source = self.source.replace(
            startup,
            "\n".join("    " + line if line else "" for line in javascript.splitlines()),
        )
        self.assertNotEqual(fixture_source, self.source)
        with tempfile.TemporaryDirectory(prefix="hitter-task-frontend-state-") as directory:
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

    def _render_state_in_headless_browser(self, state):
        return self._run_in_headless_browser("renderState({});".format(json.dumps(state, ensure_ascii=False)))

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
        self.assertTrue(any(tag == "main" for tag, _attrs in self.parser.tags))
        self.assertTrue(any(tag == "table" for tag, _attrs in self.parser.tags))

    def test_ball_and_production_gate_regions_and_shadow_disclaimer_exist(self):
        required = {
            "live-schema-notice",
            "progress-stages",
            "ball-telemetry",
            "production-gate-stages",
            "ball-status-value",
            "estimator-value",
            "ball-incoming-value",
            "gate-pelvis-value",
            "gate-production-incoming-value",
            "gate-planner-value",
            "gate-arm-value",
            "gate-task-value",
        }
        self.assertTrue(required.issubset(self.parser.ids))
        for text in (
            "球检测",
            "ESTIMATOR",
            "BALL-ONLY INCOMING",
            "PRODUCTION INCOMING",
            "PLANNER",
            "ARMED",
            "TASK OBS",
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
            r"(?:method\s*:\s*['\"](?:post|put|delete)|" r"fetch\s*\([^)]*,[^)]*(?:post|put|delete))",
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
                r"addEventListener\([^)]*(?:ready|invalidate|reset)" r"[\s\S]{0,240}JSON\.parse",
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
        _output, parser = self._render_state_in_headless_browser(_frontend_state(lcm_connected=False))

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

    def test_headless_v2_uses_canonical_ball_and_production_gate_fields(self):
        attempt = _attempt(
            stage="TASK_OBS_PASS 11/11",
            ball_speed_mps=99.0,
            predicted_strike_time_s=None,
            planner_tts_s=99.0,
            arm_tts_s=99.0,
            task_obs_status="PASS",
            estimator_sample_count=1,
            estimator_window_size=31,
            incoming_count=1,
            incoming_required_count=3,
        )
        state = _frontend_state(
            subjects={
                "ball": _subject(
                    status="LIVE",
                    rate_hz=119.5,
                    age_s=0.012,
                    source_frame=901,
                    valid=True,
                    occluded=False,
                ),
                "g1pelvis": _subject(status="NEVER_SEEN"),
                "table": _subject(
                    status="LIVE",
                    rate_hz=119.4,
                    age_s=0.013,
                    source_frame=901,
                    valid=True,
                    occluded=False,
                ),
            },
            ball=_ball_state(
                status="READY",
                source_frame=901,
                track_id=8,
                generation=31,
                raw_position_w=[1.0, 2.0, 3.0],
                estimated_position_w=[1.1, 2.1, 3.1],
                estimated_velocity_w=[-0.45, 0.02, 0.03],
                speed_mps=0.452,
                velocity_x_mps=-0.45,
                estimator_sample_count=31,
                estimator_window_size=31,
                estimator_ready=True,
                ball_only_incoming_count=2,
                ball_only_incoming_required=3,
                ball_only_incoming_confirmed=False,
                incoming_status="CONFIRMING",
                observed_monotonic_s=10.113,
                age_s=0.012,
            ),
            production_gate=_production_gate(
                pelvis_status="BLOCKED",
                production_incoming_status="NOT_EVALUATED",
                planner_status="BLOCKED",
                planner_reason_code="PELVIS_UNAVAILABLE",
                arm_status="NOT_EVALUATED",
                arm_trigger_tts_s=0.35,
                task_observation_status="NOT_AVAILABLE",
            ),
            current_attempt=attempt,
        )

        output, parser = self._render_state_in_headless_browser(state)

        self.assertEqual(parser.text_for("ball-status-value"), "LIVE")
        self.assertIn("31/31", parser.text_for("estimator-value"))
        self.assertIn("2/3", parser.text_for("ball-incoming-value"))
        self.assertIn(
            "NOT_EVALUATED",
            parser.text_for("gate-production-incoming-value"),
        )
        self.assertNotIn(
            "0/3",
            parser.text_for("gate-production-incoming-value"),
        )
        self.assertEqual(parser.text_for("gate-planner-value"), "BLOCKED")
        self.assertIn(
            "PELVIS_UNAVAILABLE",
            parser.text_for("gate-planner-detail"),
        )
        self.assertIn("0.350 s", parser.text_for("gate-arm-detail"))
        self.assertIn("ARM threshold", parser.text_for("gate-arm-detail"))
        self.assertNotIn("ARM TTS", parser.text_for("attempt-meta"))
        self.assertIn("NOT_AVAILABLE", parser.text_for("gate-task-value"))
        self.assertNotIn("0/11", parser.text_for("gate-task-value"))
        self.assertIn(
            "[1.100, 2.100, 3.100] m",
            parser.text_for("ball-estimated-position-value"),
        )
        self.assertIn(
            "[-0.450, 0.020, 0.030] m/s",
            parser.text_for("ball-velocity-value"),
        )
        self.assertIn(
            "0.452 m/s · vx -0.450 m/s",
            parser.text_for("ball-speed-value"),
        )
        landmarks = _LandmarkParser()
        landmarks.feed(output)
        self.assertIn(
            "pass",
            landmarks.attrs_by_id["ball-estimator-card"]["class"],
        )
        self.assertIn(
            "warn",
            landmarks.attrs_by_id["gate-planner-card"]["class"],
        )

    def test_headless_null_live_values_render_dashes_without_fake_zeroes(self):
        state = _frontend_state(current_attempt=_attempt())

        _output, parser = self._render_state_in_headless_browser(state)

        progress_text = " ".join(
            parser.text_for(element_id)
            for element_id in (
                "estimator-value",
                "ball-incoming-value",
                "gate-production-incoming-value",
                "gate-planner-detail",
                "gate-arm-detail",
                "gate-task-value",
                "gate-task-detail",
                "ball-raw-position-value",
                "ball-speed-value",
                "attempt-meta",
            )
        )
        self.assertIn("—", progress_text)
        self.assertNotIn("0/31", progress_text)
        self.assertNotIn("0/3", progress_text)
        self.assertNotIn("0/11", progress_text)
        self.assertNotIn("0.000 s", progress_text)

    def test_headless_v1_does_not_infer_live_progress_from_stage_or_defaults(self):
        _output, parser = self._render_state_in_headless_browser(_legacy_frontend_state())

        notice = parser.text_for("live-schema-notice")
        self.assertIn("旧版后端", notice)
        self.assertIn("重启", notice)
        progress_text = " ".join(
            parser.text_for(element_id)
            for element_id in (
                "ball-status-value",
                "estimator-value",
                "ball-incoming-value",
                "gate-pelvis-value",
                "gate-production-incoming-value",
                "gate-planner-value",
                "gate-arm-value",
                "gate-task-value",
                "attempt-meta",
            )
        )
        self.assertIn("UNKNOWN", progress_text)
        self.assertIn("—", progress_text)
        self.assertNotIn("31/31", progress_text)
        self.assertNotIn("3/3", progress_text)
        self.assertNotIn("11/11", progress_text)
        self.assertNotIn("0.000 s", progress_text)

    def test_headless_newer_schema_stops_instead_of_guessing_values(self):
        state = _frontend_state()
        state["schema_version"] = 3
        state["live_snapshot"]["schema_version"] = 3

        _output, parser = self._render_state_in_headless_browser(state)

        notice = parser.text_for("live-schema-notice")
        self.assertIn("UNSUPPORTED_SCHEMA", notice)
        self.assertIn("停止猜测", notice)
        self.assertEqual(parser.text_for("estimator-value"), "UNKNOWN · —")
        self.assertEqual(
            parser.text_for("gate-production-incoming-value"),
            "UNKNOWN · —",
        )

    def test_headless_subject_health_uses_structured_subject_values(self):
        state = _frontend_state(
            subjects={
                "ball": _subject(
                    status="LIVE",
                    rate_hz=119.5,
                    age_s=0.012,
                    source_frame=901,
                    valid=True,
                    occluded=False,
                ),
                "g1pelvis": _subject(status="NEVER_SEEN"),
                "table": _subject(
                    status="STALE",
                    rate_hz=60.0,
                    age_s=0.8,
                    source_frame=900,
                    valid=True,
                    occluded=False,
                ),
            },
        )

        _output, parser = self._render_state_in_headless_browser(state)

        health_text = parser.text_for("health-grid")
        self.assertIn("BALL", health_text)
        self.assertIn("LIVE", health_text)
        self.assertIn("119.5 Hz", health_text)
        self.assertIn("0.012 s", health_text)
        self.assertIn("frame 901", health_text)
        self.assertIn("PELVIS", health_text)
        self.assertIn("NEVER_SEEN", health_text)
        self.assertIn("TABLE", health_text)
        self.assertIn("STALE", health_text)

    def test_headless_history_uses_recorded_counts_without_defaults(self):
        rows = (
            _attempt(
                attempt_id=31,
                stage="PLANNER_READY",
                estimator_sample_count=19,
                estimator_window_size=31,
                incoming_count=2,
                incoming_required_count=3,
            ),
            _attempt(
                attempt_id=30,
                stage="ESTIMATING 0/31",
                estimator_sample_count=None,
                estimator_window_size=31,
                incoming_count=None,
                incoming_required_count=3,
            ),
        )
        state = _frontend_state(recent_attempts=rows)

        _output, parser = self._render_state_in_headless_browser(state)

        self.assertEqual(parser.text_for("history-estimator-31"), "19/31")
        self.assertEqual(parser.text_for("history-incoming-31"), "2/3")
        self.assertEqual(parser.text_for("history-estimator-30"), "—")
        self.assertEqual(parser.text_for("history-incoming-30"), "—")

    def test_headless_state_refresh_preserves_loaded_history_and_cursor(self):
        state = _frontend_state(recent_attempts=(_attempt(attempt_id=33),))
        existing_rows = [
            _attempt(attempt_id=32),
            _attempt(attempt_id=31),
        ]
        script = """
renderHistoryRows({}, false);
nextBefore = 31;
renderState({});
const sentinel = makeElement('div', '', (
  historyRows.map((row) => String(row.attempt_id)).join(',')
  + '|cursor=' + String(nextBefore)
));
sentinel.id = 'history-refresh-sentinel';
document.body.appendChild(sentinel);
""".format(
            json.dumps(existing_rows, ensure_ascii=False),
            json.dumps(state, ensure_ascii=False),
        )

        _output, parser = self._run_in_headless_browser(script)

        self.assertEqual(
            parser.text_for("history-refresh-sentinel"),
            "33,32,31|cursor=31",
        )

    def test_headless_advanced_stage_keeps_estimator_count_and_null_tts_empty(self):
        attempt = _attempt(
            stage="INCOMING_CONFIRMED 3/3",
            ball_speed_mps=1.39,
            predicted_strike_time_s=None,
            planner_tts_s=None,
            arm_tts_s=None,
            estimator_sample_count=31,
            estimator_window_size=31,
            incoming_count=3,
            incoming_required_count=3,
        )
        state = _frontend_state(
            ball=_ball_state(
                status="READY",
                speed_mps=1.39,
                estimator_sample_count=31,
                estimator_window_size=31,
                estimator_ready=True,
                ball_only_incoming_count=3,
                ball_only_incoming_required=3,
                ball_only_incoming_confirmed=True,
                incoming_status="CONFIRMED",
            ),
            production_gate=_production_gate(
                planner_tts_s=None,
                arm_trigger_tts_s=0.92,
            ),
            current_attempt=attempt,
        )

        _output, parser = self._render_state_in_headless_browser(state)

        progress_text = parser.text_for("estimator-value")
        meta_text = parser.text_for("attempt-meta")
        self.assertIn("31/31", progress_text)
        self.assertIn("预测击球 —", meta_text)
        self.assertIn("planner TTS —", meta_text)
        self.assertNotIn("预测击球 0.000 s", meta_text)
        self.assertNotIn("planner TTS 0.000 s", meta_text)

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
                "config_name": (_FrontendFixtureHandler.marker + _FrontendFixtureHandler.malicious),
                "session_basename": "fixture",
            },
            "lifecycle": {"phase": "waiting"},
            "current_attempt": None,
            "recent_attempts": [],
        }
        fixture_source = self.source.replace(
            ("    scheduleStateRefresh();\n" "    loadHistory(null, false).catch(() => {});\n" "    connectEvents();"),
            "    renderState({});".format(json.dumps(fixture_state, ensure_ascii=False)),
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
                            "http://127.0.0.1:{}/".format(server.server_address[1]),
                        ],
                        timeout=10,
                        check=True,
                        capture_output=True,
                        text=True,
                    )
                except subprocess.TimeoutExpired:
                    self.skipTest("installed Chrome cannot finish localhost " "headless navigation within 10 seconds")
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
