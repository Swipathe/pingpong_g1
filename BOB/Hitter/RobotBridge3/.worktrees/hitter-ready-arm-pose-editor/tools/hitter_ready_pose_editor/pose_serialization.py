"""Crash-consistent storage for validated HITTER ready-arm poses."""

import copy
import datetime
import errno
import fcntl
import hashlib
import json
import logging
import math
import os
import re
import secrets
import stat
import threading
from contextlib import contextmanager
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

import yaml

from .constants import (
    ARM_JOINT_NAMES,
    MOTION_NPZ_ARM_INDICES,
    ORIENTATION_TOLERANCE_RAD,
    POSE_SCHEMA,
    POSITION_TOLERANCE_M,
    ROBOT29_ARM_INDICES,
)
from .pose_validation import SAVE_CRITICAL_LINKS, ValidationResult


LOGGER = logging.getLogger(__name__)
CHINA_TZ = datetime.timezone(datetime.timedelta(hours=8))
STORAGE_WARNING = "姿态已保存，但目录持久化确认出现警告；请勿重复保存"
POSE_ID_RE = re.compile(
    r"^hitter_ready_arm_pose_[0-9]{8}_[0-9]{6}(?:_[0-9]{3})?$"
)
MARKER_RE = re.compile(
    r"^\.(hitter_ready_arm_pose_[0-9]{8}_[0-9]{6}"
    r"(?:_[0-9]{3})?)\.commit\.json$"
)
FINAL_RE = re.compile(
    r"^(hitter_ready_arm_pose_[0-9]{8}_[0-9]{6}"
    r"(?:_[0-9]{3})?)\.(?:json|yaml)$"
)
TEMP_RE = re.compile(
    r"^\.(hitter_ready_arm_pose_[0-9]{8}_[0-9]{6}"
    r"(?:_[0-9]{3})?)\.(?:json|yaml|commit\.json)"
    r"\.[0-9a-f]+\.tmp$"
)
RESERVATION_RE = re.compile(
    r"^\.(hitter_ready_arm_pose_[0-9]{8}_[0-9]{6}"
    r"(?:_[0-9]{3})?)\.reserve$"
)
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")

BASE_RECORD_KEYS = frozenset(
    (
        "schema",
        "unit",
        "joint_pos_by_name",
        "joint_names",
        "joint_pos_rad",
        "robot29",
        "motion_npz",
        "fk",
        "asset",
        "validation",
    )
)
COMPLETE_RECORD_KEYS = BASE_RECORD_KEYS | frozenset(("pose_name", "created_at"))
ROBOT29_KEYS = frozenset(("indices", "joint_names", "joint_pos_rad"))
MOTION_KEYS = frozenset(("indices", "joint_names", "joint_pos_rad"))
FK_KEYS = frozenset(
    ("frame", "root_link", "quaternion_convention", "links")
)
FK_LINK_KEYS = frozenset(("position_m", "quaternion_xyzw"))
ASSET_KEYS = frozenset(
    (
        "display_urdf_path",
        "display_urdf_sha256",
        "display_mesh_set_sha256",
        "validation_mjcf_path",
        "validation_mjcf_sha256",
        "asset_yaml_path",
        "asset_yaml_sha256",
        "urdf_kinematic_sha256",
        "mjcf_kinematic_sha256",
        "asset_signature_sha256",
    )
)
VALIDATION_KEYS_WITHOUT_SOFT = frozenset(
    (
        "hard_limits",
        "browser_mujoco_fk",
        "max_position_error_m",
        "max_orientation_error_rad",
        "soft_limit_warnings",
    )
)
VALIDATION_KEYS = VALIDATION_KEYS_WITHOUT_SOFT | frozenset(("soft_limits",))
MARKER_KEYS = frozenset(("pose_id", "json_sha256", "yaml_sha256"))


class PoseStoreError(RuntimeError):
    """A pose record or fixed pose-store generation is invalid."""


@dataclass(frozen=True)
class SavedPosePaths:
    pose_id: str
    created_at: str
    json_path: Path
    yaml_path: Path
    storage_warning: Optional[str] = None


@dataclass(frozen=True)
class _AssetContract:
    active_joint_names: Tuple[str, ...]
    asset_provenance: Mapping[str, str]


def _exact_mapping(
    value: object,
    keys: frozenset,
    field: str,
) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise PoseStoreError("%s must be an object" % field)
    actual = set(value.keys())
    if actual != keys:
        raise PoseStoreError("%s has invalid fields" % field)
    return value


def _finite_number(value: object, field: str) -> float:
    if type(value) not in (int, float) or not math.isfinite(float(value)):
        raise PoseStoreError("%s must be a finite number" % field)
    return float(value)


def _finite_vector(value: object, length: int, field: str) -> List[float]:
    if not isinstance(value, list) or len(value) != length:
        raise PoseStoreError("%s must contain %d finite numbers" % (field, length))
    return [
        _finite_number(item, "%s[%d]" % (field, index))
        for index, item in enumerate(value)
    ]


def _string_list(value: object, field: str) -> List[str]:
    if not isinstance(value, list) or any(type(item) is not str for item in value):
        raise PoseStoreError("%s must be a string array" % field)
    return list(value)


def _sha256_text(value: object, field: str) -> str:
    if type(value) is not str or SHA256_RE.fullmatch(value) is None:
        raise PoseStoreError("%s must be a lowercase SHA-256" % field)
    return value


def _absolute_path(value: object, field: str) -> str:
    if type(value) is not str or not value or not Path(value).is_absolute():
        raise PoseStoreError("%s must be an approved absolute path" % field)
    return value


def _exact_integer_indices(
    value: object,
    expected: Sequence[int],
    field: str,
) -> List[int]:
    if (
        not isinstance(value, list)
        or len(value) != len(expected)
        or any(type(item) is not int for item in value)
        or value != list(expected)
    ):
        raise PoseStoreError("%s mapping is invalid" % field)
    return list(value)


def _asset_contract(manifest) -> _AssetContract:
    active_joint_names = tuple(manifest.active_joint_names)
    if (
        len(active_joint_names) != 29
        or len(set(active_joint_names)) != 29
        or any(type(name) is not str or not name for name in active_joint_names)
    ):
        raise PoseStoreError(
            "startup manifest must contain 29 unique active joint names"
        )
    if tuple(
        active_joint_names[index] for index in ROBOT29_ARM_INDICES
    ) != ARM_JOINT_NAMES:
        raise PoseStoreError("startup manifest arm joint mapping is invalid")
    hashes = manifest.asset_hashes
    asset_provenance = {
        "display_urdf_path": _absolute_path(
            manifest.asset_paths["urdf"], "manifest display URDF path"
        ),
        "display_urdf_sha256": _sha256_text(
            hashes["display_urdf_sha256"], "manifest display URDF hash"
        ),
        "display_mesh_set_sha256": _sha256_text(
            hashes["display_mesh_set_sha256"], "manifest display mesh hash"
        ),
        "validation_mjcf_path": _absolute_path(
            manifest.asset_paths["mjcf"], "manifest validation MJCF path"
        ),
        "validation_mjcf_sha256": _sha256_text(
            hashes["validation_mjcf_sha256"], "manifest validation MJCF hash"
        ),
        "asset_yaml_path": _absolute_path(
            manifest.asset_paths["asset_yaml"], "manifest asset YAML path"
        ),
        "asset_yaml_sha256": _sha256_text(
            hashes["asset_yaml_sha256"], "manifest asset YAML hash"
        ),
        "urdf_kinematic_sha256": _sha256_text(
            manifest.urdf_kinematic_sha256, "manifest URDF kinematic hash"
        ),
        "mjcf_kinematic_sha256": _sha256_text(
            manifest.mjcf_kinematic_sha256, "manifest MJCF kinematic hash"
        ),
        "asset_signature_sha256": _sha256_text(
            manifest.asset_signature_sha256, "manifest asset signature"
        ),
    }
    return _AssetContract(
        active_joint_names=active_joint_names,
        asset_provenance=asset_provenance,
    )


def _ordered_joint_mapping(value: object) -> Dict[str, float]:
    if not isinstance(value, Mapping):
        raise PoseStoreError("joint_pos_by_name must be an object")
    if tuple(value.keys()) != ARM_JOINT_NAMES:
        raise PoseStoreError(
            "joint_pos_by_name must contain ARM_JOINT_NAMES in order"
        )
    return {
        name: _finite_number(value[name], "joint_pos_by_name.%s" % name)
        for name in ARM_JOINT_NAMES
    }


def _normalize_links(value: object) -> Dict[str, Dict[str, List[float]]]:
    links = _exact_mapping(
        value, frozenset(SAVE_CRITICAL_LINKS), "fk.links"
    )
    normalized: Dict[str, Dict[str, List[float]]] = {}
    for name in SAVE_CRITICAL_LINKS:
        link = _exact_mapping(links[name], FK_LINK_KEYS, "fk.links.%s" % name)
        position = _finite_vector(
            link["position_m"], 3, "fk.links.%s.position_m" % name
        )
        quaternion = _finite_vector(
            link["quaternion_xyzw"],
            4,
            "fk.links.%s.quaternion_xyzw" % name,
        )
        norm = math.sqrt(sum(item * item for item in quaternion))
        if abs(norm - 1.0) > 1e-6:
            raise PoseStoreError(
                "fk.links.%s.quaternion_xyzw must be unit length" % name
            )
        normalized[name] = {
            "position_m": position,
            "quaternion_xyzw": quaternion,
        }
    return normalized


def _normalize_base_record(
    record: object,
    require_soft_limits: bool,
    contract: _AssetContract,
) -> Dict[str, object]:
    expected_keys = BASE_RECORD_KEYS
    value = _exact_mapping(record, expected_keys, "pose record")
    if value["schema"] != POSE_SCHEMA:
        raise PoseStoreError("unsupported pose schema")
    if value["unit"] != "rad":
        raise PoseStoreError("pose unit must be rad")

    authoritative = _ordered_joint_mapping(value["joint_pos_by_name"])
    joint_names = _string_list(value["joint_names"], "joint_names")
    if tuple(joint_names) != ARM_JOINT_NAMES:
        raise PoseStoreError("joint_names must equal ARM_JOINT_NAMES")
    joint_values = _finite_vector(
        value["joint_pos_rad"], len(ARM_JOINT_NAMES), "joint_pos_rad"
    )
    authoritative_values = [authoritative[name] for name in ARM_JOINT_NAMES]
    if joint_values != authoritative_values:
        raise PoseStoreError("joint_pos_rad disagrees with joint_pos_by_name")

    robot = _exact_mapping(value["robot29"], ROBOT29_KEYS, "robot29")
    robot_indices = _exact_integer_indices(
        robot["indices"], tuple(range(29)), "robot29.indices"
    )
    robot_names = _string_list(robot["joint_names"], "robot29.joint_names")
    if tuple(robot_names) != contract.active_joint_names:
        raise PoseStoreError(
            "robot29.joint_names must equal the startup manifest"
        )
    robot_values = _finite_vector(
        robot["joint_pos_rad"], 29, "robot29.joint_pos_rad"
    )
    if [robot_values[index] for index in ROBOT29_ARM_INDICES] != (
        authoritative_values
    ):
        raise PoseStoreError(
            "robot29.joint_pos_rad disagrees with joint_pos_by_name"
        )

    motion = _exact_mapping(value["motion_npz"], MOTION_KEYS, "motion_npz")
    motion_indices = _exact_integer_indices(
        motion["indices"],
        MOTION_NPZ_ARM_INDICES,
        "motion_npz.indices",
    )
    motion_names = _string_list(
        motion["joint_names"], "motion_npz.joint_names"
    )
    if tuple(motion_names) != ARM_JOINT_NAMES:
        raise PoseStoreError("motion_npz.joint_names mapping is invalid")
    motion_values = _finite_vector(
        motion["joint_pos_rad"],
        len(ARM_JOINT_NAMES),
        "motion_npz.joint_pos_rad",
    )
    if motion_values != authoritative_values:
        raise PoseStoreError(
            "motion_npz.joint_pos_rad disagrees with joint_pos_by_name"
        )

    fk = _exact_mapping(value["fk"], FK_KEYS, "fk")
    if fk["frame"] != "robot_base_default":
        raise PoseStoreError("fk.frame must be robot_base_default")
    if fk["root_link"] != "pelvis":
        raise PoseStoreError("fk.root_link must be pelvis")
    if fk["quaternion_convention"] != "xyzw":
        raise PoseStoreError("fk.quaternion_convention must be xyzw")
    links = _normalize_links(fk["links"])

    asset = _exact_mapping(value["asset"], ASSET_KEYS, "asset")
    normalized_asset = {
        "display_urdf_path": _absolute_path(
            asset["display_urdf_path"], "asset.display_urdf_path"
        ),
        "display_urdf_sha256": _sha256_text(
            asset["display_urdf_sha256"], "asset.display_urdf_sha256"
        ),
        "display_mesh_set_sha256": _sha256_text(
            asset["display_mesh_set_sha256"],
            "asset.display_mesh_set_sha256",
        ),
        "validation_mjcf_path": _absolute_path(
            asset["validation_mjcf_path"], "asset.validation_mjcf_path"
        ),
        "validation_mjcf_sha256": _sha256_text(
            asset["validation_mjcf_sha256"],
            "asset.validation_mjcf_sha256",
        ),
        "asset_yaml_path": _absolute_path(
            asset["asset_yaml_path"], "asset.asset_yaml_path"
        ),
        "asset_yaml_sha256": _sha256_text(
            asset["asset_yaml_sha256"], "asset.asset_yaml_sha256"
        ),
        "urdf_kinematic_sha256": _sha256_text(
            asset["urdf_kinematic_sha256"],
            "asset.urdf_kinematic_sha256",
        ),
        "mjcf_kinematic_sha256": _sha256_text(
            asset["mjcf_kinematic_sha256"],
            "asset.mjcf_kinematic_sha256",
        ),
        "asset_signature_sha256": _sha256_text(
            asset["asset_signature_sha256"],
            "asset.asset_signature_sha256",
        ),
    }
    if normalized_asset != contract.asset_provenance:
        raise PoseStoreError(
            "asset provenance does not match the startup manifest"
        )

    validation_keys = (
        VALIDATION_KEYS if require_soft_limits else VALIDATION_KEYS_WITHOUT_SOFT
    )
    validation = _exact_mapping(
        value["validation"], validation_keys, "validation"
    )
    if validation["hard_limits"] != "passed":
        raise PoseStoreError("validation.hard_limits must be passed")
    if validation["browser_mujoco_fk"] != "passed":
        raise PoseStoreError("validation.browser_mujoco_fk must be passed")
    max_position = _finite_number(
        validation["max_position_error_m"],
        "validation.max_position_error_m",
    )
    if max_position < 0.0 or max_position > POSITION_TOLERANCE_M:
        raise PoseStoreError("validation.max_position_error_m is out of range")
    max_orientation = _finite_number(
        validation["max_orientation_error_rad"],
        "validation.max_orientation_error_rad",
    )
    if (
        max_orientation < 0.0
        or max_orientation > ORIENTATION_TOLERANCE_RAD
    ):
        raise PoseStoreError(
            "validation.max_orientation_error_rad is out of range"
        )
    soft_warnings = _string_list(
        validation["soft_limit_warnings"],
        "validation.soft_limit_warnings",
    )
    normalized_validation: Dict[str, object] = {
        "hard_limits": "passed",
        "browser_mujoco_fk": "passed",
        "max_position_error_m": max_position,
        "max_orientation_error_rad": max_orientation,
        "soft_limit_warnings": soft_warnings,
    }
    if require_soft_limits:
        soft_limits = validation["soft_limits"]
        if soft_limits not in ("passed", "warning_confirmed"):
            raise PoseStoreError("validation.soft_limits is invalid")
        normalized_validation["soft_limits"] = soft_limits

    return {
        "schema": POSE_SCHEMA,
        "unit": "rad",
        "joint_pos_by_name": authoritative,
        "joint_names": list(ARM_JOINT_NAMES),
        "joint_pos_rad": authoritative_values,
        "robot29": {
            "indices": robot_indices,
            "joint_names": robot_names,
            "joint_pos_rad": robot_values,
        },
        "motion_npz": {
            "indices": motion_indices,
            "joint_names": list(ARM_JOINT_NAMES),
            "joint_pos_rad": authoritative_values,
        },
        "fk": {
            "frame": "robot_base_default",
            "root_link": "pelvis",
            "quaternion_convention": "xyzw",
            "links": links,
        },
        "asset": normalized_asset,
        "validation": normalized_validation,
    }


def _parse_created_at(value: object) -> str:
    if type(value) is not str:
        raise PoseStoreError("created_at must be an ISO-8601 string")
    try:
        parsed = datetime.datetime.fromisoformat(value)
    except ValueError:
        raise PoseStoreError("created_at must be an ISO-8601 string")
    if parsed.utcoffset() != datetime.timedelta(hours=8):
        raise PoseStoreError("created_at must use the +08:00 offset")
    if parsed.microsecond != 0:
        raise PoseStoreError("created_at must have whole-second precision")
    return parsed.isoformat()


def _normalize_complete_record(
    record: object,
    expected_pose_id: str,
    contract: _AssetContract,
) -> Dict[str, object]:
    value = _exact_mapping(record, COMPLETE_RECORD_KEYS, "saved pose record")
    if value["pose_name"] != expected_pose_id:
        raise PoseStoreError("pose_name does not match pose_id")
    created_at = _parse_created_at(value["created_at"])
    base = {key: copy.deepcopy(value[key]) for key in BASE_RECORD_KEYS}
    normalized = _normalize_base_record(
        base,
        require_soft_limits=True,
        contract=contract,
    )
    normalized["pose_name"] = expected_pose_id
    normalized["created_at"] = created_at
    return normalized


def build_pose_record(
    validation: ValidationResult,
    manifest,
) -> Dict[str, object]:
    """Build the user-confirmation-independent portion of PoseRecordV1."""
    contract = _asset_contract(manifest)
    if (
        validation.asset_signature_sha256
        != contract.asset_provenance["asset_signature_sha256"]
    ):
        raise PoseStoreError("validation asset signature is stale")
    record: Dict[str, object] = {
        "schema": POSE_SCHEMA,
        "unit": "rad",
        "joint_pos_by_name": {
            name: float(validation.joint_pos_by_name[name])
            for name in ARM_JOINT_NAMES
        },
        "joint_names": list(ARM_JOINT_NAMES),
        "joint_pos_rad": [
            float(validation.joint_pos_by_name[name])
            for name in ARM_JOINT_NAMES
        ],
        "robot29": {
            "indices": list(range(29)),
            "joint_names": list(contract.active_joint_names),
            "joint_pos_rad": [
                float(value) for value in validation.joint_pos_29_rad
            ],
        },
        "motion_npz": {
            "indices": list(MOTION_NPZ_ARM_INDICES),
            "joint_names": list(ARM_JOINT_NAMES),
            "joint_pos_rad": [
                float(validation.joint_pos_by_name[name])
                for name in ARM_JOINT_NAMES
            ],
        },
        "fk": {
            "frame": validation.fk_robot_base["frame"],
            "root_link": "pelvis",
            "quaternion_convention": validation.fk_robot_base[
                "quaternion_convention"
            ],
            "links": {
                name: {
                    "position_m": list(
                        validation.fk_robot_base["links"][name]["position_m"]
                    ),
                    "quaternion_xyzw": list(
                        validation.fk_robot_base["links"][name][
                            "quaternion_xyzw"
                        ]
                    ),
                }
                for name in SAVE_CRITICAL_LINKS
            },
        },
        "asset": dict(contract.asset_provenance),
        "validation": {
            "hard_limits": "passed",
            "browser_mujoco_fk": "passed",
            "max_position_error_m": float(
                validation.max_position_error_m
            ),
            "max_orientation_error_rad": float(
                validation.max_orientation_error_rad
            ),
            "soft_limit_warnings": list(validation.soft_limit_warnings),
        },
    }
    return _normalize_base_record(
        record,
        require_soft_limits=False,
        contract=contract,
    )


def _sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


class PoseStore:
    def __init__(self, output_dir, asset_manifest, fault_hook=None):
        self._lock = threading.RLock()
        self._fault_hook = fault_hook if fault_hook is not None else lambda _phase: None
        self._asset_contract = _asset_contract(asset_manifest)
        self.output_dir = self._prepare_directory(Path(output_dir), "output")
        self._incomplete_dir = self._prepare_directory(
            self.output_dir / ".incomplete", "quarantine"
        )
        self._store_lock_fd = self._open_store_lock()
        with self._lock:
            self.recover_incomplete()

    @staticmethod
    def _existing_components(path: Path) -> Sequence[Path]:
        components = []
        current = path
        while True:
            components.append(current)
            if current == current.parent:
                break
            current = current.parent
        return tuple(reversed(components))

    @classmethod
    def _prepare_directory(cls, path: Path, field: str) -> Path:
        absolute = Path(os.path.abspath(str(path)))
        for component in cls._existing_components(absolute):
            if os.path.lexists(str(component)) and component.is_symlink():
                raise PoseStoreError("%s directory cannot contain a symlink" % field)
        if os.path.lexists(str(absolute)):
            info = absolute.lstat()
            if not stat.S_ISDIR(info.st_mode):
                raise PoseStoreError("%s path must be a directory" % field)
        else:
            try:
                absolute.mkdir(parents=True, mode=0o700)
            except OSError as exc:
                raise PoseStoreError(
                    "cannot create %s directory" % field
                ) from exc
            os.chmod(str(absolute), 0o700)
        info = absolute.stat()
        if (
            info.st_uid != os.geteuid()
            or stat.S_IMODE(info.st_mode) != 0o700
        ):
            raise PoseStoreError(
                "%s directory must be owner-only mode 0700" % field
            )
        return absolute

    @staticmethod
    def _write_new_file(path: Path, payload: bytes) -> None:
        flags = os.O_CREAT | os.O_EXCL | os.O_WRONLY
        fd = os.open(str(path), flags, 0o600)
        try:
            with os.fdopen(fd, "wb") as stream:
                fd = -1
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
        finally:
            if fd >= 0:
                os.close(fd)

    def _open_store_lock(self) -> int:
        path = self.output_dir / ".store.lock"
        common_flags = (
            os.O_RDWR
            | getattr(os, "O_CLOEXEC", 0)
            | getattr(os, "O_NOFOLLOW", 0)
        )
        try:
            fd = os.open(
                str(path),
                common_flags | os.O_CREAT | os.O_EXCL,
                0o600,
            )
        except OSError as exc:
            if exc.errno != errno.EEXIST:
                raise PoseStoreError("cannot create the store lock") from exc
            try:
                fd = os.open(str(path), common_flags)
            except OSError as open_exc:
                raise PoseStoreError(
                    "cannot open the store lock safely"
                ) from open_exc
        try:
            info = os.fstat(fd)
            if (
                info.st_uid != os.geteuid()
                or not stat.S_ISREG(info.st_mode)
                or stat.S_IMODE(info.st_mode) != 0o600
            ):
                raise PoseStoreError(
                    "store lock must be owner-only mode 0600"
                )
            return fd
        except BaseException:
            os.close(fd)
            raise

    @contextmanager
    def _process_store_lock(self):
        fcntl.flock(self._store_lock_fd, fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(self._store_lock_fd, fcntl.LOCK_UN)

    @staticmethod
    def _fsync_directory(path: Path) -> None:
        flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
        fd = os.open(str(path), flags)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)

    def _marker_path(self, pose_id: str) -> Path:
        return self.output_dir / (".%s.commit.json" % pose_id)

    def _reservation_path(self, pose_id: str) -> Path:
        return self.output_dir / (".%s.reserve" % pose_id)

    def _generation_paths(self, pose_id: str) -> Tuple[Path, Path, Path]:
        return (
            self.output_dir / ("%s.json" % pose_id),
            self.output_dir / ("%s.yaml" % pose_id),
            self._marker_path(pose_id),
        )

    def _candidate_is_free(self, pose_id: str) -> bool:
        json_path, yaml_path, marker_path = self._generation_paths(pose_id)
        if any(
            os.path.lexists(str(path))
            for path in (json_path, yaml_path, marker_path)
        ):
            return False
        prefix = ".%s." % pose_id
        return not any(
            child.name.startswith(prefix) for child in self.output_dir.iterdir()
        )

    def _try_reserve_pose_id(
        self,
        pose_id: str,
    ) -> Optional[Tuple[Path, int]]:
        if not self._candidate_is_free(pose_id):
            return None
        reservation_path = self._reservation_path(pose_id)
        flags = (
            os.O_CREAT
            | os.O_EXCL
            | os.O_WRONLY
            | getattr(os, "O_CLOEXEC", 0)
        )
        try:
            fd = os.open(str(reservation_path), flags, 0o600)
        except OSError as exc:
            if exc.errno == errno.EEXIST:
                return None
            raise
        try:
            self._fault_hook("after_reservation_create_before_lock")
            info = os.fstat(fd)
            if (
                info.st_uid != os.geteuid()
                or not stat.S_ISREG(info.st_mode)
                or stat.S_IMODE(info.st_mode) != 0o600
            ):
                raise PoseStoreError("pose reservation is not owner-only")
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            os.write(fd, ("%d\n" % os.getpid()).encode("ascii"))
            os.fsync(fd)
            if any(
                os.path.lexists(str(path))
                for path in self._generation_paths(pose_id)
            ):
                self._release_reservation(reservation_path, fd)
                return None
            return reservation_path, fd
        except BaseException:
            try:
                os.close(fd)
            finally:
                if os.path.lexists(str(reservation_path)):
                    os.unlink(str(reservation_path))
            raise

    @staticmethod
    def _release_reservation(path: Path, fd: int) -> None:
        error = None
        try:
            os.unlink(str(path))
        except BaseException as exc:
            error = exc
        try:
            fcntl.flock(fd, fcntl.LOCK_UN)
        finally:
            os.close(fd)
        if error is not None:
            raise error

    def _reserve_pose_id(
        self,
        now: datetime.datetime,
    ) -> Tuple[str, Path, int]:
        base = now.strftime("hitter_ready_arm_pose_%Y%m%d_%H%M%S")
        candidates = [base] + [
            "%s_%03d" % (base, suffix) for suffix in range(1, 1000)
        ]
        with self._process_store_lock():
            for candidate in candidates:
                reservation = self._try_reserve_pose_id(candidate)
                if reservation is not None:
                    return candidate, reservation[0], reservation[1]
        raise PoseStoreError("no free pose ID remains for this second")

    @staticmethod
    def _local_time(now: Optional[datetime.datetime]) -> datetime.datetime:
        if now is None:
            value = datetime.datetime.now(CHINA_TZ)
        elif not isinstance(now, datetime.datetime):
            raise PoseStoreError("now must be a datetime")
        elif now.tzinfo is None:
            value = now.replace(tzinfo=CHINA_TZ)
        else:
            value = now.astimezone(CHINA_TZ)
        return value.replace(microsecond=0)

    def save(
        self,
        record: Mapping[str, object],
        now: Optional[datetime.datetime] = None,
    ) -> SavedPosePaths:
        with self._lock:
            normalized = _normalize_base_record(
                copy.deepcopy(record),
                require_soft_limits=True,
                contract=self._asset_contract,
            )
            local_time = self._local_time(now)
            pose_id, reservation_path, reservation_fd = (
                self._reserve_pose_id(local_time)
            )
            created_at = local_time.isoformat()
            normalized["pose_name"] = pose_id
            normalized["created_at"] = created_at
            complete = _normalize_complete_record(
                normalized,
                pose_id,
                self._asset_contract,
            )

            json_payload = (
                json.dumps(
                    complete,
                    allow_nan=False,
                    ensure_ascii=False,
                    indent=2,
                )
                + "\n"
            ).encode("utf-8")
            yaml_payload = yaml.safe_dump(
                complete,
                sort_keys=False,
                allow_unicode=True,
            ).encode("utf-8")

            token = secrets.token_hex(8)
            json_path, yaml_path, marker_path = self._generation_paths(pose_id)
            json_temp = self.output_dir / (
                ".%s.json.%s.tmp" % (pose_id, token)
            )
            yaml_temp = self.output_dir / (
                ".%s.yaml.%s.tmp" % (pose_id, token)
            )
            marker_temp = self.output_dir / (
                ".%s.commit.json.%s.tmp" % (pose_id, token)
            )

            try:
                self._write_new_file(json_temp, json_payload)
                self._write_new_file(yaml_temp, yaml_payload)
                os.replace(str(json_temp), str(json_path))
                self._fault_hook("after_json_rename")
                os.replace(str(yaml_temp), str(yaml_path))
                self._fault_hook("after_yaml_rename")
                self._fsync_directory(self.output_dir)

                saved = SavedPosePaths(
                    pose_id=pose_id,
                    created_at=created_at,
                    json_path=json_path,
                    yaml_path=yaml_path,
                )
                marker_payload = (
                    json.dumps(
                        {
                            "pose_id": pose_id,
                            "json_sha256": _sha256_bytes(json_payload),
                            "yaml_sha256": _sha256_bytes(yaml_payload),
                        },
                        allow_nan=False,
                        ensure_ascii=False,
                        indent=2,
                    )
                    + "\n"
                ).encode("utf-8")
                self._write_new_file(marker_temp, marker_payload)
                self._fault_hook("before_marker_rename")
                os.replace(str(marker_temp), str(marker_path))
            except BaseException:
                try:
                    self._release_reservation(
                        reservation_path,
                        reservation_fd,
                    )
                except BaseException:
                    LOGGER.exception(
                        "failed to release an uncommitted pose reservation"
                    )
                raise

            storage_warning = None
            try:
                self._fsync_directory(self.output_dir)
                self._fault_hook("after_marker_rename")
            except BaseException:
                LOGGER.exception(
                    "pose generation committed but directory confirmation failed"
                )
                storage_warning = STORAGE_WARNING
            try:
                self._release_reservation(
                    reservation_path,
                    reservation_fd,
                )
                self._fsync_directory(self.output_dir)
            except BaseException:
                LOGGER.exception(
                    "pose generation committed but reservation cleanup failed"
                )
                storage_warning = STORAGE_WARNING
            if storage_warning is not None:
                return replace(saved, storage_warning=storage_warning)
            return saved

    @staticmethod
    def _read_regular_bytes(path: Path) -> bytes:
        flags = (
            os.O_RDONLY
            | getattr(os, "O_CLOEXEC", 0)
            | getattr(os, "O_NOFOLLOW", 0)
        )
        try:
            fd = os.open(str(path), flags)
        except OSError as exc:
            raise PoseStoreError(
                "saved pose generation cannot be opened safely"
            ) from exc
        try:
            info = os.fstat(fd)
            if (
                info.st_uid != os.geteuid()
                or not stat.S_ISREG(info.st_mode)
                or stat.S_IMODE(info.st_mode) != 0o600
            ):
                raise PoseStoreError(
                    "saved pose generation has unsafe file metadata"
                )
            chunks = []
            while True:
                chunk = os.read(fd, 64 * 1024)
                if not chunk:
                    return b"".join(chunks)
                chunks.append(chunk)
        except OSError as exc:
            raise PoseStoreError(
                "saved pose generation cannot be read"
            ) from exc
        finally:
            os.close(fd)

    def _read_committed(self, pose_id: str) -> Dict[str, object]:
        json_path, yaml_path, marker_path = self._generation_paths(pose_id)
        marker_payload = self._read_regular_bytes(marker_path)
        try:
            marker = json.loads(marker_payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise PoseStoreError("saved pose marker is invalid") from exc
        marker = _exact_mapping(marker, MARKER_KEYS, "commit marker")
        if marker["pose_id"] != pose_id:
            raise PoseStoreError("commit marker pose_id is invalid")
        json_hash = _sha256_text(marker["json_sha256"], "commit json hash")
        yaml_hash = _sha256_text(marker["yaml_sha256"], "commit yaml hash")
        json_payload = self._read_regular_bytes(json_path)
        yaml_payload = self._read_regular_bytes(yaml_path)
        if (
            _sha256_bytes(json_payload) != json_hash
            or _sha256_bytes(yaml_payload) != yaml_hash
        ):
            raise PoseStoreError("saved pose hash mismatch")
        try:
            json_record = json.loads(json_payload.decode("utf-8"))
            yaml_record = yaml.safe_load(yaml_payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError, yaml.YAMLError) as exc:
            raise PoseStoreError("saved pose data is invalid") from exc
        normalized_json = _normalize_complete_record(
            json_record,
            pose_id,
            self._asset_contract,
        )
        normalized_yaml = _normalize_complete_record(
            yaml_record,
            pose_id,
            self._asset_contract,
        )
        if normalized_json != normalized_yaml:
            raise PoseStoreError("saved JSON and YAML semantics differ")
        return normalized_json

    def _quarantine_path(self, path: Path) -> None:
        if not os.path.lexists(str(path)):
            return
        target = self._incomplete_dir / path.name
        if os.path.lexists(str(target)):
            for suffix in range(1, 10000):
                candidate = self._incomplete_dir / (
                    "%s.%03d" % (path.name, suffix)
                )
                if not os.path.lexists(str(candidate)):
                    target = candidate
                    break
            else:
                raise PoseStoreError("quarantine has no collision-free name")
        os.replace(str(path), str(target))

    def _quarantine_generation(self, pose_id: str) -> None:
        for path in self._generation_paths(pose_id):
            self._quarantine_path(path)
        prefix = ".%s." % pose_id
        for path in tuple(self.output_dir.iterdir()):
            if path != self._incomplete_dir and path.name.startswith(prefix):
                self._quarantine_path(path)
        self._fsync_directory(self._incomplete_dir)
        self._fsync_directory(self.output_dir)

    @staticmethod
    def _reservation_is_active(path: Path) -> bool:
        flags = (
            os.O_RDWR
            | getattr(os, "O_CLOEXEC", 0)
            | getattr(os, "O_NOFOLLOW", 0)
        )
        try:
            fd = os.open(str(path), flags)
        except OSError:
            return False
        try:
            info = os.fstat(fd)
            if (
                info.st_uid != os.geteuid()
                or not stat.S_ISREG(info.st_mode)
                or stat.S_IMODE(info.st_mode) != 0o600
            ):
                return False
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError as exc:
                if exc.errno in (errno.EACCES, errno.EAGAIN):
                    return True
                raise
            fcntl.flock(fd, fcntl.LOCK_UN)
            return False
        finally:
            os.close(fd)

    def recover_incomplete(self) -> None:
        with self._lock:
            with self._process_store_lock():
                self._recover_incomplete_locked()

    def _recover_incomplete_locked(self) -> None:
        committed = set()
        active_reservations = set()
        for path in self.output_dir.iterdir():
            match = MARKER_RE.fullmatch(path.name)
            if match is not None:
                committed.add(match.group(1))
            reservation_match = RESERVATION_RE.fullmatch(path.name)
            if (
                reservation_match is not None
                and self._reservation_is_active(path)
            ):
                active_reservations.add(reservation_match.group(1))

        moved = False
        for path in tuple(self.output_dir.iterdir()):
            if path == self._incomplete_dir:
                continue
            final_match = FINAL_RE.fullmatch(path.name)
            temp_match = TEMP_RE.fullmatch(path.name)
            reservation_match = RESERVATION_RE.fullmatch(path.name)
            pose_id = None
            if final_match is not None:
                pose_id = final_match.group(1)
            elif temp_match is not None:
                pose_id = temp_match.group(1)
            elif reservation_match is not None:
                pose_id = reservation_match.group(1)
            if pose_id in active_reservations:
                continue
            if reservation_match is not None or temp_match is not None:
                self._quarantine_path(path)
                moved = True
            elif (
                final_match is not None
                and final_match.group(1) not in committed
            ):
                self._quarantine_path(path)
                moved = True
        if moved:
            self._fsync_directory(self._incomplete_dir)
            self._fsync_directory(self.output_dir)

    def _marker_pose_ids(self) -> List[str]:
        result = []
        for path in self.output_dir.iterdir():
            match = MARKER_RE.fullmatch(path.name)
            if match is not None:
                result.append(match.group(1))
        return result

    def _read_or_quarantine(self, pose_id: str) -> Dict[str, object]:
        try:
            return self._read_committed(pose_id)
        except Exception:
            LOGGER.exception("quarantining invalid saved pose generation")
            self._quarantine_generation(pose_id)
            raise PoseStoreError("invalid saved pose generation")

    def list_records(self) -> List[Dict[str, object]]:
        with self._lock:
            records = []
            for pose_id in self._marker_pose_ids():
                record = self._read_or_quarantine(pose_id)
                records.append(
                    {
                        "pose_id": pose_id,
                        "created_at": record["created_at"],
                    }
                )
            records.sort(
                key=lambda item: (item["created_at"], item["pose_id"]),
                reverse=True,
            )
            return records

    def load(self, pose_id: str) -> Dict[str, object]:
        if type(pose_id) is not str or POSE_ID_RE.fullmatch(pose_id) is None:
            raise PoseStoreError("pose_id is not server-listed")
        with self._lock:
            if pose_id not in set(self._marker_pose_ids()):
                raise PoseStoreError("pose_id is not server-listed")
            return self._read_or_quarantine(pose_id)
