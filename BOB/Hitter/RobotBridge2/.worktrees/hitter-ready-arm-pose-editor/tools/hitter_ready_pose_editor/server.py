"""Loopback-only authenticated HTTP API for the HITTER pose editor."""

import argparse
import copy
import hmac
import json
import logging
import math
import os
import re
import secrets
import stat
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Dict, Mapping, Optional, Tuple
from urllib.parse import parse_qsl, urlsplit

from .asset_model import AssetSourceChanged, HitterAssetModel
from .constants import MAX_MESH_BYTES, MAX_REQUEST_BODY_BYTES
from .pose_serialization import (
    PoseStore,
    PoseStoreError,
    SavedPosePaths,
    build_pose_record,
)
from .pose_validation import (
    PoseInputError,
    PoseValidationError,
    PoseValidator,
    ValidationResult,
)


LOGGER = logging.getLogger(__name__)
ACCESS_LOGGER = logging.getLogger(__name__ + ".access")
API_SCHEMA = "hitter_ready_pose_editor_api/v1"
TICKET_TTL_SECONDS = 600
TOKEN_HEADER = "X-Hitter-Editor-Token"
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
HTTP_METHOD_TOKEN_RE = re.compile(
    r"^[!#$%&'*+\-.^_`|~0-9A-Za-z]+$"
)
MAX_STATIC_FILE_BYTES = MAX_REQUEST_BODY_BYTES
POSE_ROUTE_RE = re.compile(
    r"^/api/poses/"
    r"(hitter_ready_arm_pose_[0-9]{8}_[0-9]{6}(?:_[0-9]{3})?)$"
)
ASSET_ROUTE_RE = re.compile(
    r"^/assets/([0-9a-f]{64})/([0-9a-f]{64})$"
)
STATIC_BASENAMES = (
    "index.html",
    "styles.css",
    "matrix.js",
    "fk.js",
    "stl.js",
    "renderer.js",
    "pose_state.js",
    "api.js",
    "app.js",
)
STATIC_ROUTES = {
    "/": "index.html",
    **{
        "/static/" + basename: basename
        for basename in STATIC_BASENAMES
    },
}
STATIC_CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
}
SECURITY_HEADERS = (
    ("X-Content-Type-Options", "nosniff"),
    ("Cache-Control", "no-store"),
    ("Referrer-Policy", "no-referrer"),
    ("X-Frame-Options", "DENY"),
    (
        "Content-Security-Policy",
        "default-src 'self'; script-src 'self'; style-src 'self'; "
        "connect-src 'self'; object-src 'none'; frame-ancestors 'none'; "
        "base-uri 'none'",
    ),
)


class InvalidRequest(ValueError):
    """The HTTP request does not match the public schema."""


class TicketConflict(RuntimeError):
    """The validation ticket is absent, expired, or stale."""


class SoftLimitConfirmationRequired(RuntimeError):
    """A warning-bearing ticket needs the exact JSON boolean true."""


class ReadOnlyApplication(RuntimeError):
    """Validation and saving are disabled for an incompatible manifest."""


class AssetChanged(RuntimeError):
    """The displayed or source asset no longer matches the ticket."""


class StoreFailure(RuntimeError):
    """A pre-commit store failure left the ticket retryable."""


@dataclass
class PreparedPose:
    validation: ValidationResult
    record: Dict[str, object]
    expires_monotonic: float
    asset_signature_sha256: str


def parse_loopback_host(value: str) -> str:
    """Argparse type that accepts only the v1 literal IPv4 loopback host."""
    if value != "127.0.0.1":
        raise argparse.ArgumentTypeError(
            "--host 仅允许字面值 127.0.0.1"
        )
    return value


def startup_asset_hash_report(asset_model: HitterAssetModel) -> str:
    """Return the complete current public asset hash set for startup logs."""
    hashes = asset_model.public_manifest().get("assetHashes")
    if not isinstance(hashes, Mapping):
        raise RuntimeError("public asset manifest is missing asset hashes")
    return json.dumps(
        dict(hashes),
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    )


def _parse_port(value: str) -> int:
    try:
        port = int(value)
    except ValueError:
        raise argparse.ArgumentTypeError("--port 必须是整数")
    if port < 1 or port > 65535:
        raise argparse.ArgumentTypeError("--port 必须在 1 到 65535 之间")
    return port


def generate_session_token() -> str:
    return secrets.token_urlsafe(32)


def _exact_object(
    value: object,
    expected_keys,
    field: str,
) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise InvalidRequest("%s must be an object" % field)
    actual = set(value.keys())
    expected = set(expected_keys)
    if actual != expected:
        raise InvalidRequest("%s fields are invalid" % field)
    if any(type(key) is not str for key in value.keys()):
        raise InvalidRequest("%s fields are invalid" % field)
    return value


def _sha256(value: object, field: str) -> str:
    if type(value) is not str or SHA256_RE.fullmatch(value) is None:
        raise InvalidRequest("%s must be a lowercase SHA-256" % field)
    return value


def _reject_duplicate_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise InvalidRequest("duplicate JSON field")
        result[key] = value
    return result


def _reject_json_constant(_value):
    raise InvalidRequest("non-finite JSON number")


def _plain_json_value(value: object):
    if isinstance(value, Mapping):
        return {
            str(key): _plain_json_value(item)
            for key, item in value.items()
        }
    if isinstance(value, (list, tuple)):
        return [_plain_json_value(item) for item in value]
    if value is None or type(value) in (bool, str, int):
        return value
    if type(value) is float and math.isfinite(value):
        return value
    raise InvalidRequest("value is not finite JSON data")


def _safe_static_paths(web_root: Path) -> Dict[str, Path]:
    resolved_root = web_root.resolve()
    result = {}
    for basename in STATIC_BASENAMES:
        candidate = resolved_root / basename
        if candidate.is_symlink():
            continue
        if candidate.parent.resolve() == resolved_root:
            result[basename] = candidate
    return result


class EditorApplication:
    """Thread-safe application state shared by HTTP request handlers."""

    def __init__(
        self,
        asset_model: HitterAssetModel,
        validator: PoseValidator,
        store: PoseStore,
        web_root: Path,
        session_token: str,
    ):
        if type(session_token) is not str or not session_token:
            raise ValueError("session_token must be a nonempty string")
        self.asset_model = asset_model
        self.validator = validator
        self.store = store
        self.web_root = Path(web_root).resolve()
        self.session_token = session_token
        self._manifest = asset_model.build_manifest()
        self._public_manifest = copy.deepcopy(asset_model.public_manifest())
        self._static_paths = _safe_static_paths(self.web_root)
        self._tickets: Dict[str, PreparedPose] = {}
        self._ticket_lock = threading.Lock()

    @property
    def compatible_for_save(self) -> bool:
        return bool(self._manifest.compatible_for_save)

    @property
    def display_mesh_set_sha256(self) -> str:
        value = self._public_manifest["assetHashes"][
            "displayMeshSetSha256"
        ]
        return str(value)

    def health(self) -> Dict[str, object]:
        incompatibilities = list(self._manifest.incompatibilities)
        source_ok = True
        try:
            self.asset_model.assert_source_hashes_unchanged()
        except (AssetSourceChanged, OSError):
            source_ok = False
            incompatibilities.append("资产在启动后发生变化")
        return {
            "status": (
                "ok"
                if self.compatible_for_save and source_ok
                else "read_only"
            ),
            "schema": API_SCHEMA,
            "asset_signature_sha256": (
                self._manifest.asset_signature_sha256
            ),
            "incompatibilities": incompatibilities,
        }

    def public_manifest(self) -> Dict[str, object]:
        return copy.deepcopy(self._public_manifest)

    def prepare_pose(self, request: object) -> Dict[str, object]:
        if not self.compatible_for_save:
            raise ReadOnlyApplication()
        body = _exact_object(
            request,
            (
                "joint_pos_by_name",
                "browser_fk",
                "asset_signature_sha256",
            ),
            "validate request",
        )
        submitted_signature = _sha256(
            body["asset_signature_sha256"],
            "asset_signature_sha256",
        )
        current_signature = self._manifest.asset_signature_sha256
        if not hmac.compare_digest(
            submitted_signature,
            current_signature,
        ):
            raise AssetChanged()
        try:
            self.asset_model.assert_source_hashes_unchanged()
        except (AssetSourceChanged, OSError):
            raise AssetChanged()
        validation = self.validator.validate(
            body["joint_pos_by_name"],
            browser_fk=body["browser_fk"],
        )
        if not hmac.compare_digest(
            validation.asset_signature_sha256,
            current_signature,
        ):
            raise AssetChanged()
        record = build_pose_record(validation, self._manifest)
        validation_id = secrets.token_urlsafe(32)
        prepared = PreparedPose(
            validation=validation,
            record=copy.deepcopy(record),
            expires_monotonic=time.monotonic() + TICKET_TTL_SECONDS,
            asset_signature_sha256=current_signature,
        )
        with self._ticket_lock:
            self._tickets[validation_id] = prepared
        return {
            "validation_id": validation_id,
            "expires_in_s": TICKET_TTL_SECONDS,
            "joint_pos_by_name": {
                name: float(value)
                for name, value in validation.joint_pos_by_name.items()
            },
            "joint_pos_29_rad": [
                float(value) for value in validation.joint_pos_29_rad
            ],
            "fk": _plain_json_value(validation.fk_robot_base),
            "soft_limit_warnings": list(
                validation.soft_limit_warnings
            ),
            "needs_soft_limit_confirmation": (
                validation.needs_soft_limit_confirmation
            ),
            "max_position_error_m": float(
                validation.max_position_error_m
            ),
            "max_orientation_error_rad": float(
                validation.max_orientation_error_rad
            ),
            "asset_hashes": copy.deepcopy(
                self._public_manifest["assetHashes"]
            ),
        }

    def consume_ticket(self, request: object) -> SavedPosePaths:
        if not self.compatible_for_save:
            raise ReadOnlyApplication()
        body = _exact_object(
            request,
            ("validation_id", "confirm_soft_limit"),
            "save request",
        )
        validation_id = body["validation_id"]
        confirm_soft_limit = body["confirm_soft_limit"]
        if (
            type(validation_id) is not str
            or not validation_id
            or len(validation_id) > 128
        ):
            raise InvalidRequest("validation_id is invalid")
        if type(confirm_soft_limit) is not bool:
            raise InvalidRequest("confirm_soft_limit must be a boolean")

        with self._ticket_lock:
            prepared = self._tickets.get(validation_id)
            if (
                prepared is None
                or time.monotonic() >= prepared.expires_monotonic
            ):
                self._tickets.pop(validation_id, None)
                raise TicketConflict()
            if (
                prepared.validation.needs_soft_limit_confirmation
                and confirm_soft_limit is not True
            ):
                raise SoftLimitConfirmationRequired()
            try:
                self.asset_model.assert_source_hashes_unchanged()
            except (AssetSourceChanged, OSError):
                raise AssetChanged()
            current = (
                self.asset_model.build_manifest().asset_signature_sha256
            )
            if not hmac.compare_digest(
                current,
                prepared.asset_signature_sha256,
            ):
                raise AssetChanged()
            record = copy.deepcopy(prepared.record)
            record["validation"]["soft_limits"] = (
                "warning_confirmed"
                if prepared.validation.needs_soft_limit_confirmation
                else "passed"
            )
            try:
                saved = self.store.save(record)
            except Exception:
                raise StoreFailure()
            del self._tickets[validation_id]
            return saved

    def static_path(self, basename: str) -> Optional[Path]:
        return self._static_paths.get(basename)


def _saved_response(saved: SavedPosePaths) -> Dict[str, object]:
    return {
        "pose_id": saved.pose_id,
        "created_at": saved.created_at,
        "json_path": str(saved.json_path),
        "yaml_path": str(saved.yaml_path),
        "storage_warning": saved.storage_warning,
    }


def make_handler(app: EditorApplication):
    """Build a fixed-route handler bound to one application instance."""

    class EditorRequestHandler(BaseHTTPRequestHandler):
        server_version = "HitterReadyPoseEditor/1"
        sys_version = ""

        def do_GET(self):
            self._dispatch()

        def do_POST(self):
            self._dispatch()

        def do_HEAD(self):
            self._dispatch()

        def do_PUT(self):
            self._dispatch()

        def do_PATCH(self):
            self._dispatch()

        def do_DELETE(self):
            self._dispatch()

        def do_OPTIONS(self):
            self._dispatch()

        def __getattr__(self, name):
            method_name = "do_" + getattr(self, "command", "")
            if (
                name == method_name
                and HTTP_METHOD_TOKEN_RE.fullmatch(self.command) is not None
            ):
                return self._dispatch
            raise AttributeError(name)

        def log_message(self, _format, *_args):
            try:
                path = urlsplit(self.path).path
            except ValueError:
                path = "<invalid-target>"
            ACCESS_LOGGER.info("%s %s", self.command, path)

        def _security_headers(self):
            for name, value in SECURITY_HEADERS:
                self.send_header(name, value)

        def _send_bytes(
            self,
            status_code: int,
            payload: bytes,
            content_type: str,
            extra_headers=(),
        ):
            self.send_response(status_code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(payload)))
            for name, value in extra_headers:
                self.send_header(name, value)
            self._security_headers()
            self.end_headers()
            if self.command != "HEAD":
                self.wfile.write(payload)

        def send_json(
            self,
            status_code: int,
            body: Mapping[str, object],
            extra_headers=(),
        ):
            try:
                payload = json.dumps(
                    body,
                    allow_nan=False,
                    ensure_ascii=False,
                    separators=(",", ":"),
                ).encode("utf-8")
            except (TypeError, ValueError):
                payload = (
                    b'{"error":"\\u670d\\u52a1\\u5668\\u8fd4\\u56de'
                    b'\\u6570\\u636e\\u65e0\\u6548","code":"internal_error"}'
                )
                status_code = 500
            self._send_bytes(
                status_code,
                payload,
                "application/json; charset=utf-8",
                extra_headers=extra_headers,
            )

        def _error(
            self,
            status_code: int,
            message: str,
            code: str,
            extra_headers=(),
        ):
            self.send_json(
                status_code,
                {"error": message[:96], "code": code},
                extra_headers=extra_headers,
            )

        def _target(self) -> Tuple[str, str]:
            try:
                split = urlsplit(self.path)
            except ValueError:
                raise InvalidRequest()
            return split.path, split.query

        def _is_protected(self, path: str) -> bool:
            return (
                path == "/api"
                or path.startswith("/api/")
                or path == "/assets"
                or path.startswith("/assets/")
            )

        def _authenticate(self) -> bool:
            provided = self.headers.get(TOKEN_HEADER, "")
            if not hmac.compare_digest(provided, app.session_token):
                self._error(403, "会话令牌无效", "forbidden")
                return False
            return True

        def _allowed_method(self, path: str) -> Optional[str]:
            if path in STATIC_ROUTES:
                return "GET"
            if path in (
                "/api/health",
                "/api/model",
                "/api/poses",
            ):
                return "GET"
            if path in ("/api/validate", "/api/save"):
                return "POST"
            if (
                path.startswith("/api/poses/")
                and "/" not in path[len("/api/poses/") :]
            ):
                return "GET"
            if POSE_ROUTE_RE.fullmatch(path) is not None:
                return "GET"
            if ASSET_ROUTE_RE.fullmatch(path) is not None:
                return "GET"
            return None

        def _dispatch(self):
            try:
                path, query = self._target()
                if self._is_protected(path) and not self._authenticate():
                    return
                allowed = self._allowed_method(path)
                if allowed is not None and self.command != allowed:
                    self._error(
                        405,
                        "请求方法不允许",
                        "method_not_allowed",
                        extra_headers=(("Allow", allowed),),
                    )
                    return
                if allowed is None:
                    self._error(404, "请求路径不存在", "not_found")
                    return
                if path == "/":
                    self._serve_root(query)
                elif path in STATIC_ROUTES:
                    if query:
                        self._error(404, "静态文件不存在", "not_found")
                    else:
                        self._serve_static(STATIC_ROUTES[path])
                elif query:
                    self._error(404, "请求路径不存在", "not_found")
                elif path == "/api/health":
                    self.send_json(200, app.health())
                elif path == "/api/model":
                    self.send_json(200, app.public_manifest())
                elif path == "/api/poses":
                    self.send_json(
                        200,
                        {"poses": app.store.list_records()},
                    )
                elif path == "/api/validate":
                    self._validate()
                elif path == "/api/save":
                    self._save()
                elif path.startswith("/api/poses/"):
                    self._load_pose(path)
                elif ASSET_ROUTE_RE.fullmatch(path) is not None:
                    self._serve_asset(path)
                else:
                    self._error(404, "请求路径不存在", "not_found")
            except (BrokenPipeError, ConnectionResetError):
                return
            except Exception:
                LOGGER.error("ready pose request failed")
                try:
                    self._error(500, "服务器内部错误", "internal_error")
                except (BrokenPipeError, ConnectionResetError):
                    return

        def _serve_root(self, query: str):
            if query:
                try:
                    pairs = parse_qsl(
                        query,
                        keep_blank_values=True,
                        strict_parsing=True,
                    )
                except ValueError:
                    self._error(403, "会话令牌无效", "forbidden")
                    return
                if (
                    len(pairs) != 1
                    or pairs[0][0] != "token"
                    or not hmac.compare_digest(
                        pairs[0][1],
                        app.session_token,
                    )
                ):
                    self._error(403, "会话令牌无效", "forbidden")
                    return
            self._serve_static("index.html")

        def _serve_static(self, basename: str):
            path = app.static_path(basename)
            if path is None:
                self._error(404, "静态文件不存在", "not_found")
                return
            try:
                flags = (
                    os.O_RDONLY
                    | getattr(os, "O_CLOEXEC", 0)
                    | getattr(os, "O_NOFOLLOW", 0)
                )
                fd = os.open(str(path), flags)
                try:
                    before = os.fstat(fd)
                    if (
                        before.st_uid != os.geteuid()
                        or not stat.S_ISREG(before.st_mode)
                        or before.st_size < 0
                        or before.st_size > MAX_STATIC_FILE_BYTES
                    ):
                        raise OSError("unsafe static file")
                    chunks = []
                    remaining = before.st_size
                    while remaining:
                        chunk = os.read(
                            fd,
                            min(64 * 1024, remaining),
                        )
                        if not chunk:
                            raise OSError("short static read")
                        chunks.append(chunk)
                        remaining -= len(chunk)
                    after = os.fstat(fd)
                    if (
                        after.st_dev != before.st_dev
                        or after.st_ino != before.st_ino
                        or after.st_size != before.st_size
                        or after.st_mtime_ns != before.st_mtime_ns
                    ):
                        raise OSError("static file changed during read")
                    payload = b"".join(chunks)
                finally:
                    os.close(fd)
            except OSError:
                self._error(404, "静态文件不存在", "not_found")
                return
            content_type = STATIC_CONTENT_TYPES[Path(basename).suffix]
            self._send_bytes(200, payload, content_type)

        def _read_json(self) -> Mapping[str, object]:
            if self.headers.get("Content-Type") != "application/json":
                raise TypeError("unsupported media type")
            content_length = self.headers.get("Content-Length")
            if content_length is None:
                raise InvalidRequest("Content-Length is required")
            try:
                length = int(content_length)
            except ValueError:
                raise InvalidRequest("Content-Length is invalid")
            if length < 0:
                raise InvalidRequest("Content-Length is invalid")
            if length > MAX_REQUEST_BODY_BYTES:
                raise OverflowError("request body is too large")
            raw = self.rfile.read(length)
            if len(raw) != length:
                raise InvalidRequest("request body is incomplete")
            try:
                decoded = raw.decode("utf-8")
                value = json.loads(
                    decoded,
                    object_pairs_hook=_reject_duplicate_keys,
                    parse_constant=_reject_json_constant,
                )
            except UnicodeDecodeError:
                raise InvalidRequest("request body is not UTF-8")
            except json.JSONDecodeError:
                raise InvalidRequest("request body is not JSON")
            if not isinstance(value, Mapping):
                raise InvalidRequest("JSON root must be an object")
            return value

        def _json_request_or_error(self):
            try:
                return self._read_json()
            except TypeError:
                self._error(
                    415,
                    "仅接受 application/json",
                    "unsupported_media_type",
                )
            except OverflowError:
                self._error(
                    413,
                    "请求体超过 256 KiB 限制",
                    "payload_too_large",
                )
            except InvalidRequest:
                self._error(400, "JSON 请求无效", "invalid_json")
            return None

        def _validate(self):
            request = self._json_request_or_error()
            if request is None:
                return
            try:
                response = app.prepare_pose(request)
            except ReadOnlyApplication:
                self._error(
                    409,
                    "当前资产不兼容，验证和保存已禁用",
                    "read_only",
                )
                return
            except AssetChanged:
                self._error(
                    409,
                    "资产已变化，请刷新页面",
                    "asset_changed",
                )
                return
            except (InvalidRequest, PoseInputError, PoseStoreError):
                self._error(400, "姿态请求字段无效", "invalid_request")
                return
            except PoseValidationError:
                self._error(
                    409,
                    "浏览器与服务端姿态验证不一致",
                    "validation_failed",
                )
                return
            self.send_json(200, response)

        def _save(self):
            request = self._json_request_or_error()
            if request is None:
                return
            try:
                saved = app.consume_ticket(request)
            except InvalidRequest:
                self._error(400, "保存请求字段无效", "invalid_request")
                return
            except ReadOnlyApplication:
                self._error(
                    409,
                    "当前资产不兼容，验证和保存已禁用",
                    "read_only",
                )
                return
            except TicketConflict:
                self._error(
                    409,
                    "验证票据不存在、已过期或已使用",
                    "ticket_conflict",
                )
                return
            except SoftLimitConfirmationRequired:
                self._error(
                    409,
                    "软限位警告需要明确二次确认",
                    "soft_limit_confirmation_required",
                )
                return
            except AssetChanged:
                self._error(
                    409,
                    "资产已变化，请重新验证",
                    "asset_changed",
                )
                return
            except StoreFailure:
                self._error(
                    500,
                    "姿态尚未提交，可使用同一票据重试",
                    "store_failure",
                )
                return
            self.send_json(201, _saved_response(saved))

        def _load_pose(self, path: str):
            match = POSE_ROUTE_RE.fullmatch(path)
            if match is None:
                self._error(404, "保存姿态不存在", "pose_not_found")
                return
            pose_id = match.group(1)
            try:
                allowlist = {
                    item["pose_id"] for item in app.store.list_records()
                }
                if pose_id not in allowlist:
                    raise PoseStoreError("not listed")
                record = app.store.load(pose_id)
            except PoseStoreError:
                self._error(404, "保存姿态不存在", "pose_not_found")
                return
            self.send_json(200, {"pose": record})

        def _serve_asset(self, path: str):
            if self.headers.get("Range") is not None:
                self._error(
                    400,
                    "不支持 Range 请求",
                    "range_not_supported",
                )
                return
            match = ASSET_ROUTE_RE.fullmatch(path)
            if match is None:
                self._error(404, "网格不存在", "asset_not_found")
                return
            mesh_set_hash, mesh_id = match.groups()
            if not hmac.compare_digest(
                mesh_set_hash,
                app.display_mesh_set_sha256,
            ):
                self._error(
                    409,
                    "网格集合签名已失效",
                    "asset_changed",
                )
                return
            if mesh_id not in set(app.asset_model.mesh_ids()):
                self._error(404, "网格不存在", "asset_not_found")
                return
            try:
                app.asset_model.assert_source_hashes_unchanged()
                mesh_path = Path(app.asset_model.mesh_path(mesh_id))
                if (
                    mesh_path.is_symlink()
                    or mesh_path.resolve() != mesh_path
                ):
                    raise OSError("unsafe mesh path")
                flags = (
                    os.O_RDONLY
                    | getattr(os, "O_CLOEXEC", 0)
                    | getattr(os, "O_NOFOLLOW", 0)
                )
                fd = os.open(str(mesh_path), flags)
                try:
                    info = os.fstat(fd)
                    if (
                        not stat.S_ISREG(info.st_mode)
                        or info.st_size > MAX_MESH_BYTES
                    ):
                        raise OSError("unsafe mesh file")
                    chunks = []
                    remaining = info.st_size
                    while remaining:
                        chunk = os.read(fd, min(64 * 1024, remaining))
                        if not chunk:
                            raise OSError("short mesh read")
                        chunks.append(chunk)
                        remaining -= len(chunk)
                    payload = b"".join(chunks)
                finally:
                    os.close(fd)
            except (AssetSourceChanged, OSError, KeyError):
                self._error(404, "网格不存在", "asset_not_found")
                return
            self._send_bytes(200, payload, "model/stl")

    return EditorRequestHandler


class EditorHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = False


def _build_application(
    repo_root: Path,
    output_dir: Path,
    web_root: Path,
    session_token: str,
) -> EditorApplication:
    import numpy as np
    from utils.kinematics import (
        ForwardKinematicsConfig,
        MujocoKinematics,
    )

    asset_model = HitterAssetModel.from_defaults(repo_root)
    manifest = asset_model.build_manifest()
    kinematics = MujocoKinematics(
        ForwardKinematicsConfig(
            xml_path=str(manifest.asset_paths["mjcf"]),
            debug_viz=False,
            kinematic_joint_names=list(manifest.active_joint_names),
        )
    )
    validator = PoseValidator(
        asset_model,
        kinematics,
        np.asarray(asset_model._default_root_pos, dtype=np.float64),
    )
    store = PoseStore(output_dir, asset_manifest=manifest)
    return EditorApplication(
        asset_model,
        validator,
        store,
        web_root,
        session_token,
    )


def main(argv=None) -> int:
    repo_root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(
        description="HITTER ready-arm pose editor loopback server"
    )
    parser.add_argument(
        "--host",
        type=parse_loopback_host,
        default="127.0.0.1",
    )
    parser.add_argument("--port", type=_parse_port, default=8765)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=repo_root / "deploy/data/hitter_ready_poses",
    )
    parser.add_argument(
        "--web-root",
        type=Path,
        default=Path(__file__).resolve().parent / "web",
    )
    args = parser.parse_args(argv)

    session_token = generate_session_token()
    app = _build_application(
        repo_root,
        args.output_dir,
        args.web_root,
        session_token,
    )
    server = EditorHTTPServer(
        (args.host, args.port),
        make_handler(app),
    )
    print(
        "HITTER ready pose editor: "
        "http://127.0.0.1:%d/?token=%s"
        % (server.server_port, session_token),
        flush=True,
    )
    print(
        "HITTER ready pose asset hashes: "
        + startup_asset_hash_report(app.asset_model),
        flush=True,
    )
    if not app.compatible_for_save:
        print(
            "READ ONLY: asset compatibility mismatch; "
            "validation/save disabled",
            flush=True,
        )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        LOGGER.info("HITTER ready pose editor stopped by operator")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
