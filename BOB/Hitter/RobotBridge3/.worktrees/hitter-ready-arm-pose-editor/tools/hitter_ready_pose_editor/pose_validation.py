"""Strict ready-arm pose validation backed by locked MuJoCo FK."""

import math
import threading
from dataclasses import dataclass
from types import MappingProxyType
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

import mujoco
import numpy as np

from .asset_model import HitterAssetModel
from .constants import (
    ARM_JOINT_NAMES,
    ORIENTATION_TOLERANCE_RAD,
    POSITION_TOLERANCE_M,
    ROBOT29_ARM_INDICES,
)
from utils.kinematics import MujocoKinematics


SAVE_CRITICAL_LINKS = (
    "left_wrist_yaw_link",
    "right_wrist_yaw_link",
    "right_racket_link",
)
FK_SUMMARY_KEYS = frozenset(("frame", "quaternion_convention", "links"))
FK_LINK_KEYS = frozenset(("position_m", "quaternion_xyzw"))


class PoseInputError(ValueError):
    """The submitted name-keyed pose violates the input contract."""


class PoseValidationError(RuntimeError):
    """The pose could not be proven against the current asset/FK contract."""


@dataclass(frozen=True)
class ValidationResult:
    joint_pos_by_name: Mapping[str, float]
    joint_pos_29_rad: Tuple[float, ...]
    soft_limit_warnings: Tuple[str, ...]
    needs_soft_limit_confirmation: bool
    fk_robot_base: Mapping[str, object]
    max_position_error_m: float
    max_orientation_error_rad: float
    asset_signature_sha256: str


def quat_xyzw_to_matrix(quaternion_xyzw: Sequence[float]) -> np.ndarray:
    """Convert a finite, nonzero XYZW quaternion into a rotation matrix."""
    quaternion = np.asarray(quaternion_xyzw, dtype=np.float64)
    if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
        raise PoseValidationError("FK quaternion must contain four finite values")
    norm = float(np.linalg.norm(quaternion))
    if norm == 0.0:
        raise PoseValidationError("FK quaternion must be nonzero")
    x, y, z, w = quaternion / norm
    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )


def matrix_to_canonical_quat_xyzw(
    rotation_matrix: Sequence[Sequence[float]],
) -> np.ndarray:
    """Convert a rotation matrix to a unit XYZW quaternion with nonnegative w."""
    matrix = np.asarray(rotation_matrix, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise PoseValidationError("FK rotation must be a finite 3x3 matrix")
    quaternion_wxyz = np.zeros(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quaternion_wxyz, matrix.reshape(9))
    quaternion_xyzw = quaternion_wxyz[[1, 2, 3, 0]]
    norm = float(np.linalg.norm(quaternion_xyzw))
    if norm == 0.0 or not math.isfinite(norm):
        raise PoseValidationError("FK rotation produced an invalid quaternion")
    quaternion_xyzw = quaternion_xyzw / norm
    if quaternion_xyzw[3] < 0.0:
        quaternion_xyzw = -quaternion_xyzw
    return quaternion_xyzw


def _exact_keys(value: object, expected, field: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise PoseInputError("%s must be an object" % field)
    actual = set(value.keys())
    if actual != expected:
        raise PoseInputError(
            "%s keys mismatch: missing=%r, unknown=%r"
            % (field, sorted(expected - actual), sorted(actual - expected))
        )
    return value


def _strict_finite_vector(
    value: object,
    length: int,
    field: str,
) -> np.ndarray:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise PoseInputError("%s must contain %d finite numbers" % (field, length))
    for item in value:
        if type(item) not in (int, float) or not math.isfinite(float(item)):
            raise PoseInputError("%s must contain finite numbers" % field)
    return np.asarray(value, dtype=np.float64)


class PoseValidator:
    def __init__(
        self,
        asset_model: HitterAssetModel,
        kinematics: MujocoKinematics,
        default_root_pos: np.ndarray,
    ):
        self._asset_model = asset_model
        self._manifest = asset_model.build_manifest()
        self._kinematics = kinematics
        root = np.asarray(default_root_pos, dtype=np.float64)
        if root.shape != (3,) or not np.all(np.isfinite(root)):
            raise PoseInputError(
                "default_root_pos must contain three finite numbers"
            )
        self._default_root_pos = root.copy()
        self._fk_lock = threading.Lock()

    def _compose_and_warnings(
        self,
        joint_pos_by_name: Mapping[str, object],
    ) -> Tuple[np.ndarray, Tuple[str, ...]]:
        if not isinstance(joint_pos_by_name, Mapping):
            raise PoseInputError("joint_pos_by_name must be an object")
        actual = set(joint_pos_by_name.keys())
        expected = set(ARM_JOINT_NAMES)
        if actual != expected:
            missing = sorted(expected - actual)
            unknown = sorted(actual - expected)
            raise PoseInputError(
                "joint names mismatch: missing={!r}, unknown={!r}".format(
                    missing, unknown
                )
            )

        full = np.asarray(
            self._manifest.default_joint_pos_rad, dtype=np.float64
        ).copy()
        warnings: List[str] = []
        for name, robot_index in zip(ARM_JOINT_NAMES, ROBOT29_ARM_INDICES):
            raw = joint_pos_by_name[name]
            if type(raw) not in (int, float) or not math.isfinite(float(raw)):
                raise PoseInputError("{} must be a finite number".format(name))
            value = float(raw)
            spec = self._manifest.joint_by_name[name]
            if spec.hard_limit_rad is None:
                raise PoseValidationError("{} has no hard limit".format(name))
            lower, upper = spec.hard_limit_rad
            if value < lower or value > upper:
                raise PoseInputError(
                    "{}={} outside hard limit [{}, {}]".format(
                        name, value, lower, upper
                    )
                )
            if spec.soft_limit_rad is not None:
                soft_lower, soft_upper = spec.soft_limit_rad
                if value < soft_lower or value > soft_upper:
                    warnings.append(
                        "{}={} outside soft limit [{}, {}]".format(
                            name, value, soft_lower, soft_upper
                        )
                    )
            full[robot_index] = value
        return full, tuple(warnings)

    def compose_joint_pos(
        self,
        joint_pos_by_name: Mapping[str, object],
    ) -> np.ndarray:
        full, _ = self._compose_and_warnings(joint_pos_by_name)
        return full

    def _assert_fk_ready(self) -> None:
        self._asset_model.assert_source_hashes_unchanged()
        if not self._manifest.compatible_for_save:
            raise PoseValidationError(
                "asset is incompatible for save: {}".format(
                    ", ".join(self._manifest.incompatibilities)
                )
            )

    @staticmethod
    def _body_vector(
        body_info: Mapping[str, object],
        body_name: str,
        field: str,
        length: int,
    ) -> np.ndarray:
        body = body_info[body_name]
        if not isinstance(body, Mapping) or field not in body:
            raise PoseValidationError(
                "MuJoCo body {} is missing {}".format(body_name, field)
            )
        value = np.asarray(body[field], dtype=np.float64)
        if value.shape != (length,) or not np.all(np.isfinite(value)):
            raise PoseValidationError(
                "MuJoCo body {} {} must contain {} finite values".format(
                    body_name, field, length
                )
            )
        return value

    def forward_links_robot_base(
        self,
        joint_pos_29,
        link_names,
    ) -> Dict[str, Dict[str, List[float]]]:
        self._assert_fk_ready()
        joint_pos = np.asarray(joint_pos_29, dtype=np.float64)
        if joint_pos.shape != (29,) or not np.all(np.isfinite(joint_pos)):
            raise PoseInputError(
                "joint_pos_29 must contain 29 finite numbers"
            )
        requested = tuple(link_names)
        if any(type(name) is not str or not name for name in requested):
            raise PoseInputError("link names must be nonempty strings")
        if len(set(requested)) != len(requested):
            raise PoseInputError("link names must be unique")

        with self._fk_lock:
            body_info, _ = self._kinematics.forward(
                joint_pos,
                base_pos=self._default_root_pos,
                base_quat=np.array(
                    [0.0, 0.0, 0.0, 1.0], dtype=np.float64
                ),
            )

        required = set(requested)
        required.add("pelvis")
        missing = sorted(required - set(body_info.keys()))
        if missing:
            raise PoseValidationError(
                "MuJoCo FK missing requested bodies: {!r}".format(missing)
            )

        root_position = self._body_vector(
            body_info, "pelvis", "pos", 3
        )
        root_quaternion = self._body_vector(
            body_info, "pelvis", "quat", 4
        )
        root_rotation = quat_xyzw_to_matrix(root_quaternion)

        result: Dict[str, Dict[str, List[float]]] = {}
        for link_name in requested:
            link_position = self._body_vector(
                body_info, link_name, "pos", 3
            )
            link_quaternion = self._body_vector(
                body_info, link_name, "quat", 4
            )
            link_rotation = quat_xyzw_to_matrix(link_quaternion)
            position_m = root_rotation.T.dot(
                link_position - root_position
            )
            rotation_pelvis = root_rotation.T.dot(link_rotation)
            quaternion_xyzw = matrix_to_canonical_quat_xyzw(
                rotation_pelvis
            )
            result[link_name] = {
                "position_m": [float(value) for value in position_m],
                "quaternion_xyzw": [
                    float(value) for value in quaternion_xyzw
                ],
            }
        return result

    @staticmethod
    def _validated_browser_fk(
        browser_fk: object,
    ) -> Dict[str, Dict[str, np.ndarray]]:
        summary = _exact_keys(browser_fk, FK_SUMMARY_KEYS, "browser FK")
        if summary["frame"] != "robot_base_default":
            raise PoseInputError(
                "browser FK frame must be robot_base_default"
            )
        if summary["quaternion_convention"] != "xyzw":
            raise PoseInputError(
                "browser FK quaternion convention must be xyzw"
            )
        links = _exact_keys(
            summary["links"], frozenset(SAVE_CRITICAL_LINKS), "browser FK links"
        )
        validated: Dict[str, Dict[str, np.ndarray]] = {}
        for link_name in SAVE_CRITICAL_LINKS:
            link = _exact_keys(
                links[link_name],
                FK_LINK_KEYS,
                "browser FK link {}".format(link_name),
            )
            position = _strict_finite_vector(
                link["position_m"],
                3,
                "{} position_m".format(link_name),
            )
            quaternion = _strict_finite_vector(
                link["quaternion_xyzw"],
                4,
                "{} quaternion_xyzw".format(link_name),
            )
            if abs(float(np.linalg.norm(quaternion)) - 1.0) > 1e-6:
                raise PoseInputError(
                    "{} quaternion_xyzw must be a unit quaternion".format(
                        link_name
                    )
                )
            validated[link_name] = {
                "position_m": position,
                "quaternion_xyzw": quaternion,
            }
        return validated

    @staticmethod
    def _compare_fk(
        browser_fk: object,
        mujoco_links: Mapping[str, Mapping[str, Sequence[float]]],
    ) -> Tuple[float, float]:
        browser_links = PoseValidator._validated_browser_fk(browser_fk)
        max_position_error = 0.0
        max_orientation_error = 0.0
        for link_name in SAVE_CRITICAL_LINKS:
            client_position = browser_links[link_name]["position_m"]
            client_quaternion = browser_links[link_name][
                "quaternion_xyzw"
            ]
            mujoco_position = np.asarray(
                mujoco_links[link_name]["position_m"], dtype=np.float64
            )
            mujoco_quaternion = np.asarray(
                mujoco_links[link_name]["quaternion_xyzw"],
                dtype=np.float64,
            )
            position_error = float(
                np.linalg.norm(client_position - mujoco_position)
            )
            dot = float(
                abs(np.dot(client_quaternion, mujoco_quaternion))
            )
            orientation_error = 2.0 * math.acos(min(1.0, dot))
            max_position_error = max(max_position_error, position_error)
            max_orientation_error = max(
                max_orientation_error, orientation_error
            )

        if max_position_error > POSITION_TOLERANCE_M:
            raise PoseValidationError(
                "browser FK position error {} exceeds {}".format(
                    max_position_error, POSITION_TOLERANCE_M
                )
            )
        if max_orientation_error > ORIENTATION_TOLERANCE_RAD:
            raise PoseValidationError(
                "browser FK orientation error {} exceeds {}".format(
                    max_orientation_error, ORIENTATION_TOLERANCE_RAD
                )
            )
        return max_position_error, max_orientation_error

    def validate(
        self,
        joint_pos_by_name: Mapping[str, object],
        browser_fk: Optional[object] = None,
    ) -> ValidationResult:
        self._assert_fk_ready()

        joint_pos_29, warnings = self._compose_and_warnings(
            joint_pos_by_name
        )
        links = self.forward_links_robot_base(
            joint_pos_29, SAVE_CRITICAL_LINKS
        )
        fk_robot_base = {
            "frame": "robot_base_default",
            "quaternion_convention": "xyzw",
            "links": links,
        }
        if browser_fk is None:
            max_position_error = 0.0
            max_orientation_error = 0.0
        else:
            max_position_error, max_orientation_error = self._compare_fk(
                browser_fk, links
            )

        normalized = {
            name: float(joint_pos_29[index])
            for name, index in zip(ARM_JOINT_NAMES, ROBOT29_ARM_INDICES)
        }
        return ValidationResult(
            joint_pos_by_name=MappingProxyType(normalized),
            joint_pos_29_rad=tuple(float(value) for value in joint_pos_29),
            soft_limit_warnings=warnings,
            needs_soft_limit_confirmation=bool(warnings),
            fk_robot_base=fk_robot_base,
            max_position_error_m=max_position_error,
            max_orientation_error_rad=max_orientation_error,
            asset_signature_sha256=self._manifest.asset_signature_sha256,
        )
