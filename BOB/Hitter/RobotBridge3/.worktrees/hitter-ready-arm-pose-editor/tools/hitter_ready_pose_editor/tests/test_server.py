import argparse
import copy
import io
import json
import logging
import os
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
from pathlib import Path
from types import MappingProxyType
from unittest import mock

from tools.hitter_ready_pose_editor.asset_model import (
    AssetManifest,
    AssetSourceChanged,
)
from tools.hitter_ready_pose_editor.constants import (
    ARM_JOINT_NAMES,
    MAX_REQUEST_BODY_BYTES,
    ROBOT29_ARM_INDICES,
)
from tools.hitter_ready_pose_editor.pose_serialization import PoseStore
from tools.hitter_ready_pose_editor.pose_validation import (
    PoseInputError,
    PoseValidationError,
    ValidationResult,
)
from tools.hitter_ready_pose_editor.server import (
    EditorApplication,
    make_handler,
    parse_loopback_host,
    startup_asset_hash_report,
)


SESSION_TOKEN = "test-session-token"
SHA_A = "a" * 64
SHA_B = "b" * 64
SHA_C = "c" * 64
SHA_D = "d" * 64
SHA_E = "e" * 64
SHA_F = "f" * 64
BASE_JOINT_NAMES = tuple("base_joint_%02d" % index for index in range(15))
ACTIVE_JOINT_NAMES = BASE_JOINT_NAMES + ARM_JOINT_NAMES
STATIC_FILES = {
    "index.html": b"<!doctype html><title>HITTER Ready Pose</title>",
    "styles.css": b"body { color: #fff; }",
    "matrix.js": b"export const matrix = {};",
    "fk.js": b"export const fk = {};",
    "stl.js": b"export const stl = {};",
    "renderer.js": b"export const renderer = {};",
    "pose_state.js": b"export const poseState = {};",
    "api.js": b"export const api = {};",
    "app.js": b"export const app = {};",
}
SAVE_CRITICAL_LINKS = (
    "left_wrist_yaw_link",
    "right_wrist_yaw_link",
    "right_racket_link",
)


def make_manifest(compatible=True, incompatibilities=()):
    hashes = MappingProxyType(
        {
            "display_urdf_sha256": SHA_A,
            "display_mesh_set_sha256": SHA_B,
            "validation_mjcf_sha256": SHA_C,
            "asset_yaml_sha256": SHA_D,
            "urdf_kinematic_sha256": SHA_E,
            "mjcf_kinematic_sha256": SHA_F,
            "asset_signature_sha256": SHA_A,
        }
    )
    return AssetManifest(
        root_link="pelvis",
        active_joint_names=ACTIVE_JOINT_NAMES,
        arm_joint_names=ARM_JOINT_NAMES,
        joints_topological=(),
        joint_by_name=MappingProxyType({}),
        visuals=(),
        table_boxes=(),
        default_joint_pos_rad=tuple(0.0 for _ in range(29)),
        asset_paths=MappingProxyType(
            {
                "urdf": "/approved/display/main.urdf",
                "mjcf": "/approved/validation/model.xml",
                "asset_yaml": "/approved/config/asset.yaml",
                "allowed_asset_root": "/approved/display",
            }
        ),
        asset_hashes=hashes,
        urdf_kinematic_sha256=SHA_E,
        mjcf_kinematic_sha256=SHA_F,
        asset_signature_sha256=SHA_A,
        compatible_for_save=compatible,
        incompatibilities=tuple(incompatibilities),
        right_racket_link="right_racket_link",
    )


def public_manifest(manifest):
    return {
        "schema": "hitter_asset_manifest/v1",
        "units": {"length": "m", "angle": "rad"},
        "limits": {
            "maxRequestBodyBytes": MAX_REQUEST_BODY_BYTES,
            "maxMeshBytes": 67108864,
            "maxStlTriangles": 1000000,
        },
        "frame": {
            "name": "robot_base_default",
            "rootLink": "pelvis",
            "rootTransform": "identity",
            "handedness": "right",
            "matrixLayout": "column-major",
            "quaternionConvention": "xyzw",
            "groundZRobotBaseM": -0.793,
        },
        "rootLink": "pelvis",
        "activeJointNames": list(ACTIVE_JOINT_NAMES),
        "armJointNames": list(ARM_JOINT_NAMES),
        "defaultJointPosRad": [0.0] * 29,
        "joints": [],
        "visuals": [],
        "tableVisuals": [],
        "assetHashes": {
            "displayUrdfSha256": SHA_A,
            "displayMeshSetSha256": SHA_B,
            "validationMjcfSha256": SHA_C,
            "assetYamlSha256": SHA_D,
            "urdfKinematicSha256": SHA_E,
            "mjcfKinematicSha256": SHA_F,
            "assetSignatureSha256": SHA_A,
        },
        "compatibleForSave": manifest.compatible_for_save,
        "incompatibilities": list(manifest.incompatibilities),
        "rightRacketLink": "right_racket_link",
    }


def make_fk():
    return {
        "frame": "robot_base_default",
        "quaternion_convention": "xyzw",
        "links": {
            name: {
                "position_m": [float(index), 0.0, 0.0],
                "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            }
            for index, name in enumerate(SAVE_CRITICAL_LINKS)
        },
    }


class FakeAssetModel:
    def __init__(self, manifest, mesh_path):
        self._manifest = manifest
        self._mesh_path = Path(mesh_path)
        self.drifted = False

    def build_manifest(self):
        return self._manifest

    def public_manifest(self):
        return copy.deepcopy(public_manifest(self._manifest))

    def mesh_ids(self):
        return (SHA_C,)

    def mesh_path(self, mesh_id):
        if mesh_id != SHA_C:
            raise KeyError(mesh_id)
        return self._mesh_path

    def assert_source_hashes_unchanged(self):
        if self.drifted:
            raise AssetSourceChanged("/secret/asset/path changed")


class FakeValidator:
    def __init__(self, asset_model):
        self.asset_model = asset_model
        self.calls = 0

    def validate(self, joint_pos_by_name, browser_fk=None):
        self.calls += 1
        self.asset_model.assert_source_hashes_unchanged()
        if set(joint_pos_by_name) != set(ARM_JOINT_NAMES):
            raise PoseInputError("joint names mismatch")
        if browser_fk != make_fk():
            raise PoseValidationError("/secret/mjcf/path FK mismatch")
        values = {}
        joint_pos_29 = [float(index) / 100.0 for index in range(29)]
        for name, robot_index in zip(ARM_JOINT_NAMES, ROBOT29_ARM_INDICES):
            value = joint_pos_by_name[name]
            if type(value) not in (int, float):
                raise PoseInputError("joint value must be numeric")
            values[name] = float(value)
            joint_pos_29[robot_index] = float(value)
        warnings = (
            ("right_wrist_yaw_joint is near hard limit",)
            if values["right_wrist_yaw_joint"] > 0.9
            else ()
        )
        return ValidationResult(
            joint_pos_by_name=MappingProxyType(values),
            joint_pos_29_rad=tuple(joint_pos_29),
            soft_limit_warnings=warnings,
            needs_soft_limit_confirmation=bool(warnings),
            fk_robot_base=MappingProxyType(make_fk()),
            max_position_error_m=0.0001,
            max_orientation_error_rad=0.001,
            asset_signature_sha256=SHA_A,
        )


class InjectedStoreFailure(RuntimeError):
    """Fault injected by the server storage tests."""


class MutableFault:
    def __init__(self):
        self.phase = None

    def __call__(self, phase):
        if phase == self.phase:
            self.phase = None
            raise InjectedStoreFailure(
                "/secret/output/path injected failure"
            )


class ServerTest(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        root = Path(self.temp_dir.name)
        self.web_root = root / "web"
        self.web_root.mkdir()
        for basename, content in STATIC_FILES.items():
            (self.web_root / basename).write_bytes(content)
        self.mesh_root = root / "meshes"
        self.mesh_root.mkdir()
        self.mesh_path = self.mesh_root / "mesh.stl"
        self.mesh_path.write_bytes(b"solid mesh\nendsolid mesh\n")
        self.manifest = make_manifest()
        self.asset = FakeAssetModel(self.manifest, self.mesh_path)
        self.validator = FakeValidator(self.asset)
        self.fault = MutableFault()
        self.store = PoseStore(
            root / "poses",
            asset_manifest=self.manifest,
            fault_hook=self.fault,
        )
        self.app = EditorApplication(
            self.asset,
            self.validator,
            self.store,
            self.web_root,
            SESSION_TOKEN,
        )
        self.server = self._start_server(self.app)
        self.base_url = "http://127.0.0.1:%d" % self.server.server_port
        self.valid_pose_payload = {
            "joint_pos_by_name": {
                name: float(index) / 100.0
                for index, name in enumerate(ARM_JOINT_NAMES)
            },
            "browser_fk": make_fk(),
            "asset_signature_sha256": SHA_A,
        }

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.server_thread.join(timeout=5.0)
        self.temp_dir.cleanup()

    def _start_server(self, app):
        from http.server import ThreadingHTTPServer

        server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(app))
        self.server_thread = threading.Thread(
            target=server.serve_forever,
            daemon=True,
        )
        self.server_thread.start()
        return server

    def request(self, method, path, body=None, headers=None):
        request_headers = dict(headers or {})
        request = urllib.request.Request(
            self.base_url + path,
            data=body,
            headers=request_headers,
            method=method,
        )
        try:
            with urllib.request.urlopen(request, timeout=5.0) as response:
                return (
                    response.status,
                    response.read(),
                    dict(response.headers.items()),
                )
        except urllib.error.HTTPError as exc:
            return exc.code, exc.read(), dict(exc.headers.items())

    def request_json(
        self,
        method,
        path,
        body=None,
        authenticated=False,
        headers=None,
    ):
        request_headers = dict(headers or {})
        if authenticated:
            request_headers["X-Hitter-Editor-Token"] = SESSION_TOKEN
        raw = None
        if body is not None:
            raw = json.dumps(
                body,
                allow_nan=False,
                separators=(",", ":"),
            ).encode("utf-8")
            request_headers.setdefault("Content-Type", "application/json")
        status, payload, response_headers = self.request(
            method,
            path,
            body=raw,
            headers=request_headers,
        )
        return status, json.loads(payload.decode("utf-8")), response_headers

    def validate_once(self, soft=False):
        payload = copy.deepcopy(self.valid_pose_payload)
        if soft:
            payload["joint_pos_by_name"]["right_wrist_yaw_joint"] = 0.95
        status, body, _ = self.request_json(
            "POST",
            "/api/validate",
            body=payload,
            authenticated=True,
        )
        self.assertEqual(status, 200, body)
        return body["validation_id"]

    def save_ticket(self, validation_id, confirmation=False):
        status, body, _ = self.request_json(
            "POST",
            "/api/save",
            body={
                "validation_id": validation_id,
                "confirm_soft_limit": confirmation,
            },
            authenticated=True,
        )
        return status, body

    def assert_security_headers(self, headers):
        self.assertEqual(headers["X-Content-Type-Options"], "nosniff")
        self.assertEqual(headers["Cache-Control"], "no-store")
        self.assertEqual(headers["Referrer-Policy"], "no-referrer")
        self.assertEqual(headers["X-Frame-Options"], "DENY")
        self.assertIn("default-src 'self'", headers["Content-Security-Policy"])

    def test_static_shell_is_public_but_contains_no_data(self):
        for path in ("/", "/static/index.html", "/static/app.js"):
            status, body, headers = self.request("GET", path)
            self.assertEqual(status, 200)
            self.assertNotIn(SESSION_TOKEN.encode("ascii"), body)
            self.assertNotIn(b"/approved/", body)
            self.assertNotIn(SHA_A.encode("ascii"), body)
            self.assertNotIn(b"hitter_ready_arm_pose_", body)
            self.assert_security_headers(headers)

    def test_wrong_query_token_is_rejected(self):
        status, _, _ = self.request("GET", "/?token=wrong")
        self.assertEqual(status, 403)
        status, shell, _ = self.request(
            "GET",
            "/?token=" + SESSION_TOKEN,
        )
        self.assertEqual(status, 200)
        status, refreshed, _ = self.request("GET", "/")
        self.assertEqual(status, 200)
        self.assertEqual(shell, refreshed)

    def test_api_requires_header_token(self):
        for path in ("/api/health", "/api/model", "/api/poses"):
            for headers in ({}, {"X-Hitter-Editor-Token": "wrong"}):
                status, _, _ = self.request("GET", path, headers=headers)
                self.assertEqual(status, 403)

    def test_methods_and_content_types_are_exact(self):
        auth = {"X-Hitter-Editor-Token": SESSION_TOKEN}
        for method, path in (
            ("POST", "/api/health"),
            ("POST", "/api/model"),
            ("POST", "/api/poses"),
            ("GET", "/api/validate"),
            ("GET", "/api/save"),
            ("POST", "/static/index.html"),
        ):
            status, _, _ = self.request(method, path, headers=auth)
            self.assertEqual(status, 405, (method, path))
        for path in ("/api/validate", "/api/save"):
            status, body, _ = self.request_json(
                "POST",
                path,
                body={},
                authenticated=True,
                headers={"Content-Type": "text/plain"},
            )
            self.assertEqual(status, 415)
            self.assertEqual(body["code"], "unsupported_media_type")

    def test_trace_connect_and_unknown_methods_use_bounded_dispatch(self):
        for method in ("TRACE", "CONNECT", "BREW"):
            status, body, headers = self.request(
                method,
                "/api/model",
            )
            self.assertEqual(status, 403, method)
            self.assertEqual(
                json.loads(body.decode("utf-8"))["code"],
                "forbidden",
            )
            self.assertEqual(
                headers["Content-Type"],
                "application/json; charset=utf-8",
            )
            self.assert_security_headers(headers)

            status, body, headers = self.request(
                method,
                "/api/model",
                headers={"X-Hitter-Editor-Token": SESSION_TOKEN},
            )
            self.assertEqual(status, 405, method)
            self.assertEqual(headers["Allow"], "GET")
            self.assertEqual(
                json.loads(body.decode("utf-8")),
                {
                    "error": "请求方法不允许",
                    "code": "method_not_allowed",
                },
            )
            self.assertEqual(
                headers["Content-Type"],
                "application/json; charset=utf-8",
            )
            self.assert_security_headers(headers)

            status, body, headers = self.request(
                method,
                "/static/index.html",
            )
            self.assertEqual(status, 405, method)
            self.assertEqual(headers["Allow"], "GET")
            self.assertEqual(
                json.loads(body.decode("utf-8"))["code"],
                "method_not_allowed",
            )
            self.assertEqual(
                headers["Content-Type"],
                "application/json; charset=utf-8",
            )
            self.assert_security_headers(headers)

    def test_health_model_poses_and_saved_pose_schemas(self):
        status, health, headers = self.request_json(
            "GET",
            "/api/health",
            authenticated=True,
        )
        self.assertEqual(
            health,
            {
                "status": "ok",
                "schema": "hitter_ready_pose_editor_api/v1",
                "asset_signature_sha256": SHA_A,
                "incompatibilities": [],
            },
        )
        self.assert_security_headers(headers)
        status, model, _ = self.request_json(
            "GET",
            "/api/model",
            authenticated=True,
        )
        self.assertEqual(status, 200)
        self.assertEqual(model, public_manifest(self.manifest))
        validation_id = self.validate_once()
        status, saved = self.save_ticket(validation_id)
        self.assertEqual(status, 201)
        status, poses, _ = self.request_json(
            "GET",
            "/api/poses",
            authenticated=True,
        )
        self.assertEqual(
            poses,
            {
                "poses": [
                    {
                        "pose_id": saved["pose_id"],
                        "created_at": saved["created_at"],
                    }
                ]
            },
        )
        status, loaded, _ = self.request_json(
            "GET",
            "/api/poses/" + saved["pose_id"],
            authenticated=True,
        )
        self.assertEqual(status, 200)
        self.assertEqual(loaded["pose"]["pose_name"], saved["pose_id"])
        status, body, _ = self.request_json(
            "GET",
            "/api/poses/not-server-issued",
            authenticated=True,
        )
        self.assertEqual(status, 404)
        self.assertEqual(body["code"], "pose_not_found")

    def test_validate_then_save_uses_immutable_ticket(self):
        payload = copy.deepcopy(self.valid_pose_payload)
        status, validation, _ = self.request_json(
            "POST",
            "/api/validate",
            body=payload,
            authenticated=True,
        )
        self.assertEqual(status, 200)
        self.assertEqual(validation["expires_in_s"], 600)
        self.assertEqual(
            set(validation),
            {
                "validation_id",
                "expires_in_s",
                "joint_pos_by_name",
                "joint_pos_29_rad",
                "fk",
                "soft_limit_warnings",
                "needs_soft_limit_confirmation",
                "max_position_error_m",
                "max_orientation_error_rad",
                "asset_hashes",
            },
        )
        payload["joint_pos_by_name"][ARM_JOINT_NAMES[0]] = 999.0
        status, saved = self.save_ticket(validation["validation_id"])
        self.assertEqual(status, 201)
        self.assertTrue(saved["json_path"].endswith(".json"))
        self.assertTrue(saved["yaml_path"].endswith(".yaml"))
        with Path(saved["json_path"]).open("r", encoding="utf-8") as stream:
            record = json.load(stream)
        self.assertNotEqual(
            record["joint_pos_by_name"][ARM_JOINT_NAMES[0]],
            999.0,
        )

    def test_save_cannot_choose_path_or_reuse_ticket(self):
        validation_id = self.validate_once()
        status, body, _ = self.request_json(
            "POST",
            "/api/save",
            body={
                "validation_id": validation_id,
                "confirm_soft_limit": False,
                "output_path": "../../config/mimic/hitter.yaml",
            },
            authenticated=True,
        )
        self.assertEqual(status, 400)
        self.assertEqual(body["code"], "invalid_request")
        status, _ = self.save_ticket(validation_id)
        self.assertEqual(status, 201)
        status, body = self.save_ticket(validation_id)
        self.assertEqual(status, 409)
        self.assertEqual(body["code"], "ticket_conflict")

    def test_two_concurrent_saves_publish_exactly_once(self):
        validation_id = self.validate_once()
        barrier = threading.Barrier(3)
        statuses = []

        def save_once():
            barrier.wait()
            status, _ = self.save_ticket(validation_id)
            statuses.append(status)

        threads = [threading.Thread(target=save_once) for _ in range(2)]
        for thread in threads:
            thread.start()
        barrier.wait()
        for thread in threads:
            thread.join(timeout=5.0)
        self.assertEqual(sorted(statuses), [201, 409])
        self.assertEqual(len(self.store.list_records()), 1)

    def test_model_can_be_read_only(self):
        self.server.shutdown()
        self.server.server_close()
        self.server_thread.join(timeout=5.0)
        manifest = make_manifest(False, ("active joint order differs",))
        asset = FakeAssetModel(manifest, self.mesh_path)
        app = EditorApplication(
            asset,
            FakeValidator(asset),
            PoseStore(Path(self.temp_dir.name) / "readonly", manifest),
            self.web_root,
            SESSION_TOKEN,
        )
        self.server = self._start_server(app)
        self.base_url = "http://127.0.0.1:%d" % self.server.server_port
        status, health, _ = self.request_json(
            "GET",
            "/api/health",
            authenticated=True,
        )
        self.assertEqual(status, 200)
        self.assertEqual(health["status"], "read_only")
        status, model, _ = self.request_json(
            "GET",
            "/api/model",
            authenticated=True,
        )
        self.assertFalse(model["compatibleForSave"])
        self.assertEqual(model["incompatibilities"], ["active joint order differs"])
        status, error, _ = self.request_json(
            "POST",
            "/api/validate",
            body=self.valid_pose_payload,
            authenticated=True,
        )
        self.assertEqual(status, 409)
        self.assertEqual(error["code"], "read_only")

    def test_mesh_hash_and_id_are_both_whitelisted(self):
        auth = {"X-Hitter-Editor-Token": SESSION_TOKEN}
        good = "/assets/%s/%s" % (SHA_B, SHA_C)
        status, body, headers = self.request("GET", good, headers=auth)
        self.assertEqual(status, 200)
        self.assertEqual(body, b"solid mesh\nendsolid mesh\n")
        self.assertEqual(headers["Content-Type"], "model/stl")
        for path in (
            "/assets/%s/%s" % (SHA_A, SHA_C),
            "/assets/%s/%s" % (SHA_B, SHA_D),
            "/assets/%s/.." % SHA_B,
            "/assets/%s/%%2e%%2e" % SHA_B,
        ):
            status, _, _ = self.request("GET", path, headers=auth)
            self.assertIn(status, (400, 404, 409))
        status, _, _ = self.request(
            "GET",
            good,
            headers={
                "X-Hitter-Editor-Token": SESSION_TOKEN,
                "Range": "bytes=0-1",
            },
        )
        self.assertEqual(status, 400)
        outside = Path(self.temp_dir.name) / "outside.stl"
        outside.write_bytes(self.mesh_path.read_bytes())
        self.mesh_path.unlink()
        self.mesh_path.symlink_to(outside)
        status, _, _ = self.request("GET", good, headers=auth)
        self.assertEqual(status, 404)

    def test_static_files_are_allowlisted(self):
        for path in (
            "/static/unknown.js",
            "/static/../index.html",
            "/static/%2e%2e/index.html",
            "/static/subdir/app.js",
            "/static/index.html?cache=1",
        ):
            status, _, _ = self.request("GET", path)
            self.assertEqual(status, 404, path)

    def test_static_setup_rejects_same_root_symlink_alias(self):
        self.server.shutdown()
        self.server.server_close()
        self.server_thread.join(timeout=5.0)
        styles_path = self.web_root / "styles.css"
        styles_path.unlink()
        styles_path.symlink_to(self.web_root / "app.js")
        self.app = EditorApplication(
            self.asset,
            self.validator,
            self.store,
            self.web_root,
            SESSION_TOKEN,
        )
        self.server = self._start_server(self.app)
        self.base_url = "http://127.0.0.1:%d" % self.server.server_port

        status, body, headers = self.request(
            "GET",
            "/static/styles.css",
        )
        self.assertEqual(status, 404)
        self.assertEqual(
            json.loads(body.decode("utf-8"))["code"],
            "not_found",
        )
        self.assert_security_headers(headers)
        status, app_js, headers = self.request(
            "GET",
            "/static/app.js",
        )
        self.assertEqual(status, 200)
        self.assertEqual(app_js, STATIC_FILES["app.js"])
        self.assertEqual(
            headers["Content-Type"],
            "text/javascript; charset=utf-8",
        )

    def test_static_descriptor_does_not_follow_swap_after_path_check(self):
        styles_path = self.web_root / "styles.css"
        styles_path.unlink()
        styles_path.symlink_to(self.web_root / "app.js")
        with mock.patch.object(
            self.app,
            "static_path",
            return_value=styles_path,
        ):
            status, body, headers = self.request(
                "GET",
                "/static/styles.css",
            )
        self.assertEqual(status, 404)
        self.assertEqual(
            json.loads(body.decode("utf-8"))["code"],
            "not_found",
        )
        self.assert_security_headers(headers)

    def test_static_descriptor_requires_current_owner_and_size(self):
        with mock.patch(
            "tools.hitter_ready_pose_editor.server.os.geteuid",
            return_value=os.geteuid() + 1,
        ):
            status, _, _ = self.request("GET", "/static/styles.css")
        self.assertEqual(status, 404)

        styles_path = self.web_root / "styles.css"
        styles_path.write_bytes(b"x" * (MAX_REQUEST_BODY_BYTES + 1))
        status, _, _ = self.request("GET", "/static/styles.css")
        self.assertEqual(status, 404)

    def test_json_boundary_is_strict(self):
        auth = {
            "X-Hitter-Editor-Token": SESSION_TOKEN,
            "Content-Type": "application/json",
        }
        cases = (
            (
                b'{"joint_pos_by_name":{},"browser_fk":{},'
                b'"asset_signature_sha256":"%s",'
                b'"asset_signature_sha256":"%s"}' % (
                    SHA_A.encode("ascii"),
                    SHA_A.encode("ascii"),
                ),
                400,
            ),
            (b"\xff", 400),
            (b"[]", 400),
            (b'{"unknown":1}', 400),
            (b'{"joint_pos_by_name":{"x":NaN}}', 400),
        )
        for raw, expected in cases:
            status, body, _ = self.request(
                "POST",
                "/api/validate",
                body=raw,
                headers=auth,
            )
            self.assertEqual(status, expected)
            error = json.loads(body.decode("utf-8"))
            self.assertIn(error["code"], ("invalid_json", "invalid_request"))
        status, body, _ = self.request(
            "POST",
            "/api/validate",
            body=b"x" * (MAX_REQUEST_BODY_BYTES + 1),
            headers=auth,
        )
        self.assertEqual(status, 413)
        self.assertEqual(
            json.loads(body.decode("utf-8"))["code"],
            "payload_too_large",
        )
        status, _, _ = self.request(
            "POST",
            "/api/validate",
            body=b"{}",
            headers={
                "X-Hitter-Editor-Token": SESSION_TOKEN,
                "Content-Type": "application/json; charset=utf-8",
            },
        )
        self.assertEqual(status, 415)

    def test_ticket_expiry_and_asset_drift(self):
        expired = self.validate_once()
        self.app._tickets[expired].expires_monotonic = 0.0
        status, body = self.save_ticket(expired)
        self.assertEqual(status, 409)
        self.assertEqual(body["code"], "ticket_conflict")
        drifted = self.validate_once()
        self.asset.drifted = True
        status, body = self.save_ticket(drifted)
        self.assertEqual(status, 409)
        self.assertEqual(body["code"], "asset_changed")
        self.assertEqual(self.store.list_records(), [])

    def test_soft_confirmation_is_exact_bool(self):
        for bad_body in (
            {},
            {"confirm_soft_limit": False},
            {"confirm_soft_limit": 1},
            {"confirm_soft_limit": "true"},
        ):
            validation_id = self.validate_once(soft=True)
            body = {"validation_id": validation_id}
            body.update(bad_body)
            status, error, _ = self.request_json(
                "POST",
                "/api/save",
                body=body,
                authenticated=True,
            )
            expected = 409 if bad_body.get("confirm_soft_limit") is False else 400
            self.assertEqual(status, expected, (bad_body, error))
        validation_id = self.validate_once(soft=True)
        status, body = self.save_ticket(validation_id, confirmation=True)
        self.assertEqual(status, 201)
        with Path(body["json_path"]).open("r", encoding="utf-8") as stream:
            saved = json.load(stream)
        self.assertEqual(
            saved["validation"]["soft_limits"],
            "warning_confirmed",
        )

    def test_precommit_store_failure_keeps_ticket_retryable(self):
        validation_id = self.validate_once()
        self.fault.phase = "before_marker_rename"
        status, body = self.save_ticket(validation_id)
        self.assertEqual(status, 500)
        self.assertEqual(body["code"], "store_failure")
        self.assertEqual(self.store.list_records(), [])
        status, _ = self.save_ticket(validation_id)
        self.assertEqual(status, 201)
        self.assertEqual(len(self.store.list_records()), 1)

    def test_postcommit_warning_consumes_ticket_once(self):
        validation_id = self.validate_once()
        self.fault.phase = "after_marker_rename"
        with self.assertLogs(
            "tools.hitter_ready_pose_editor.pose_serialization",
            level="ERROR",
        ):
            status, body = self.save_ticket(validation_id)
        self.assertEqual(status, 201)
        self.assertIsNotNone(body["storage_warning"])
        self.assertEqual(len(self.store.list_records()), 1)
        status, body = self.save_ticket(validation_id)
        self.assertEqual(status, 409)
        self.assertEqual(body["code"], "ticket_conflict")
        self.assertEqual(len(self.store.list_records()), 1)

    def test_error_and_log_redaction(self):
        logger = logging.getLogger(
            "tools.hitter_ready_pose_editor.server.access"
        )
        stream = io.StringIO()
        handler = logging.StreamHandler(stream)
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)
        try:
            status, body, _ = self.request_json(
                "POST",
                "/api/validate?token=" + SESSION_TOKEN,
                body=self.valid_pose_payload,
                authenticated=True,
            )
            self.assertEqual(status, 404)
            payload = json.dumps(body, ensure_ascii=False)
            logged = stream.getvalue()
            for secret in (
                SESSION_TOKEN,
                "/secret/",
                "/approved/",
                "Traceback",
                "?token=",
            ):
                self.assertNotIn(secret, payload)
                self.assertNotIn(secret, logged)
        finally:
            logger.removeHandler(handler)

    def test_loopback_parser(self):
        self.assertEqual(parse_loopback_host("127.0.0.1"), "127.0.0.1")
        for value in ("localhost", "::1", "0.0.0.0", "g1-hostB-loco1"):
            with self.assertRaises(argparse.ArgumentTypeError):
                parse_loopback_host(value)

    def test_startup_asset_hash_report_contains_every_live_hash(self):
        report = json.loads(
            startup_asset_hash_report(self.asset)
        )
        self.assertEqual(
            report,
            {
                "assetSignatureSha256": SHA_A,
                "assetYamlSha256": SHA_D,
                "displayMeshSetSha256": SHA_B,
                "displayUrdfSha256": SHA_A,
                "mjcfKinematicSha256": SHA_F,
                "urdfKinematicSha256": SHA_E,
                "validationMjcfSha256": SHA_C,
            },
        )


if __name__ == "__main__":
    unittest.main()
