"""Read-only HITTER display assets and MuJoCo compatibility manifest."""

import hashlib
import json
import math
import os
from dataclasses import dataclass, replace
from pathlib import Path
from types import MappingProxyType
from typing import Dict, List, Mapping, Optional, Sequence, Tuple
from xml.etree import ElementTree

import mujoco
import numpy as np
import yaml

from .constants import (
    ARM_JOINT_NAMES,
    MAX_MESH_BYTES,
    MAX_REQUEST_BODY_BYTES,
    MAX_STL_TRIANGLES,
    centered_soft_limit,
)


DISPLAY_URDF = Path(
    "/home/loco1/BOB/Hitter/MOSAIC-main/source/whole_body_tracking/"
    "whole_body_tracking/assets/unitree_description/urdf/"
    "g1_hitter_racket/main.urdf"
)
DISPLAY_ASSET_ROOT = Path(
    "/home/loco1/BOB/Hitter/MOSAIC-main/source/whole_body_tracking/"
    "whole_body_tracking/assets/unitree_description"
)
COMPATIBILITY_SCHEMA = "hitter_asset_compatibility/v1"
POSITION_TOLERANCE_M = 1e-6
AXIS_TOLERANCE = 1e-8
ORIENTATION_TOLERANCE_RAD = 1e-5
LIMIT_TOLERANCE_RAD = 1e-5
DEFAULT_COLOR_RGBA = (0.7, 0.7, 0.7, 1.0)


class AssetSourceChanged(RuntimeError):
    """Raised when a source used to build a cached manifest changed on disk."""


@dataclass(frozen=True)
class JointSpec:
    name: str
    joint_type: str
    parent_link: str
    child_link: str
    origin_xyz: Tuple[float, float, float]
    origin_rpy: Tuple[float, float, float]
    axis_xyz: Optional[Tuple[float, float, float]]
    hard_limit_rad: Optional[Tuple[float, float]]
    soft_limit_rad: Optional[Tuple[float, float]]
    default_rad: Optional[float]
    robot29_index: Optional[int]


@dataclass(frozen=True)
class VisualSpec:
    link_name: str
    origin_xyz: Tuple[float, float, float]
    origin_rpy: Tuple[float, float, float]
    mesh_scale_xyz: Tuple[float, float, float]
    mesh_id: str
    color_rgba: Tuple[float, float, float, float]


@dataclass(frozen=True)
class SceneBoxSpec:
    name: str
    frame: str
    center_xyz_m: Tuple[float, float, float]
    quat_xyzw: Tuple[float, float, float, float]
    half_size_xyz_m: Tuple[float, float, float]
    color_rgba: Tuple[float, float, float, float]


@dataclass(frozen=True)
class AssetManifest:
    root_link: str
    active_joint_names: Tuple[str, ...]
    arm_joint_names: Tuple[str, ...]
    joints_topological: Tuple[JointSpec, ...]
    joint_by_name: Mapping[str, JointSpec]
    visuals: Tuple[VisualSpec, ...]
    table_boxes: Tuple[SceneBoxSpec, ...]
    default_joint_pos_rad: Tuple[float, ...]
    asset_paths: Mapping[str, str]
    asset_hashes: Mapping[str, object]
    urdf_kinematic_sha256: str
    mjcf_kinematic_sha256: str
    asset_signature_sha256: str
    compatible_for_save: bool
    incompatibilities: Tuple[str, ...]
    right_racket_link: str


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _canonical_sha256(value: object) -> str:
    payload = json.dumps(
        value,
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _finite_float(value: object, field: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("%s must be finite" % field)
    return result


def _vector(
    text: Optional[str],
    length: int,
    default: Sequence[float],
    field: str,
) -> Tuple[float, ...]:
    values = default if text is None else text.split()
    if len(values) != length:
        raise ValueError("%s must contain %d values" % (field, length))
    return tuple(_finite_float(value, field) for value in values)


def _normalize_axis(axis: Sequence[float]) -> Tuple[float, float, float]:
    norm = math.sqrt(sum(value * value for value in axis))
    if not math.isfinite(norm) or norm == 0.0:
        raise ValueError("joint axis must be finite and nonzero")
    return tuple(value / norm for value in axis)  # type: ignore


def _rpy_to_wxyz(rpy: Sequence[float]) -> Tuple[float, float, float, float]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def _quat_angle_wxyz(
    left: Sequence[float],
    right: Sequence[float],
) -> float:
    left_norm = math.sqrt(sum(value * value for value in left))
    right_norm = math.sqrt(sum(value * value for value in right))
    if left_norm == 0.0 or right_norm == 0.0:
        raise ValueError("quaternion must be nonzero")
    dot = abs(
        sum(a * b for a, b in zip(left, right)) / (left_norm * right_norm)
    )
    dot = min(1.0, max(-1.0, dot))
    return 2.0 * math.acos(dot)


def _distance(left: Sequence[float], right: Sequence[float]) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(left, right)))


def _matrix_to_xyzw(matrix: Sequence[float]) -> Tuple[float, float, float, float]:
    quat = np.zeros(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quat, np.asarray(matrix, dtype=np.float64))
    return (float(quat[1]), float(quat[2]), float(quat[3]), float(quat[0]))


class UrdfSceneModel:
    """A cached, read-only URDF tree and whitelisted mesh registry."""

    @classmethod
    def from_urdf(
        cls,
        urdf_path: Path,
        allowed_asset_root: Path,
    ) -> "UrdfSceneModel":
        return cls(urdf_path.resolve(), allowed_asset_root.resolve())

    def __init__(self, urdf_path: Path, allowed_asset_root: Path):
        self.urdf_path = urdf_path
        self.allowed_asset_root = allowed_asset_root
        if not self.urdf_path.is_file():
            raise FileNotFoundError(str(self.urdf_path))
        if not self.allowed_asset_root.is_dir():
            raise FileNotFoundError(str(self.allowed_asset_root))
        parsed = self._parse()
        self._manifest = parsed[0]
        self._mesh_paths = MappingProxyType(parsed[1])
        self._mesh_hashes = MappingProxyType(parsed[2])
        self._mesh_relative_hashes = tuple(parsed[3])

    def build_manifest(self) -> AssetManifest:
        return self._manifest

    def mesh_path(self, mesh_id: str) -> Path:
        try:
            return self._mesh_paths[mesh_id]
        except KeyError:
            raise KeyError(mesh_id)

    def mesh_ids(self) -> Tuple[str, ...]:
        return tuple(sorted(self._mesh_paths))

    def _resolve_mesh(self, filename: str) -> Path:
        prefix = "package://unitree_description/"
        if filename.startswith(prefix):
            path = self.allowed_asset_root / filename[len(prefix) :]
        elif filename.startswith("package://"):
            raise ValueError("unsupported mesh package: %s" % filename)
        else:
            path = self.urdf_path.parent / filename
        resolved = path.resolve()
        try:
            common = os.path.commonpath(
                (str(resolved), str(self.allowed_asset_root))
            )
        except ValueError:
            raise ValueError("mesh is outside approved asset root")
        if common != str(self.allowed_asset_root):
            raise ValueError("mesh is outside approved asset root")
        if not resolved.is_file():
            raise FileNotFoundError(str(resolved))
        if resolved.stat().st_size > MAX_MESH_BYTES:
            raise ValueError("mesh exceeds MAX_MESH_BYTES: %s" % resolved.name)
        return resolved

    def _parse(self):
        try:
            root = ElementTree.parse(str(self.urdf_path)).getroot()
        except ElementTree.ParseError as exc:
            raise ValueError("invalid URDF XML: %s" % exc)
        if root.tag != "robot":
            raise ValueError("URDF root must be robot")

        link_elements = root.findall("link")
        link_names = [element.get("name") for element in link_elements]
        if any(not name for name in link_names):
            raise ValueError("every link needs a name")
        if len(set(link_names)) != len(link_names):
            raise ValueError("duplicate link name")
        links = set(link_names)

        material_colors: Dict[str, Tuple[float, float, float, float]] = {}
        for material in root.findall("material"):
            name = material.get("name")
            color = material.find("color")
            if name and color is not None:
                material_colors[name] = _vector(
                    color.get("rgba"),
                    4,
                    DEFAULT_COLOR_RGBA,
                    "material rgba",
                )  # type: ignore

        raw_joints = []
        joint_names = []
        child_links = set()
        children: Dict[str, List[Tuple[str, JointSpec]]] = {}
        for element in root.findall("joint"):
            name = element.get("name")
            joint_type = element.get("type")
            if not name:
                raise ValueError("every joint needs a name")
            joint_names.append(name)
            if joint_type not in ("revolute", "fixed"):
                raise ValueError("unsupported joint type: %s" % joint_type)
            parent_element = element.find("parent")
            child_element = element.find("child")
            if parent_element is None or child_element is None:
                raise ValueError("joint %s needs parent and child" % name)
            parent = parent_element.get("link")
            child = child_element.get("link")
            if parent not in links or child not in links:
                raise ValueError("joint %s references unknown link" % name)
            if child in child_links:
                raise ValueError("link %s has multiple parents" % child)
            child_links.add(child)
            origin = element.find("origin")
            origin_xyz = _vector(
                None if origin is None else origin.get("xyz"),
                3,
                (0.0, 0.0, 0.0),
                "joint origin xyz",
            )
            origin_rpy = _vector(
                None if origin is None else origin.get("rpy"),
                3,
                (0.0, 0.0, 0.0),
                "joint origin rpy",
            )
            if joint_type == "revolute":
                axis_element = element.find("axis")
                axis = _normalize_axis(
                    _vector(
                        None if axis_element is None else axis_element.get("xyz"),
                        3,
                        (1.0, 0.0, 0.0),
                        "joint axis",
                    )
                )
                limit = element.find("limit")
                if (
                    limit is None
                    or limit.get("lower") is None
                    or limit.get("upper") is None
                ):
                    raise ValueError("revolute joint %s needs limits" % name)
                lower = _finite_float(limit.get("lower"), "joint lower limit")
                upper = _finite_float(limit.get("upper"), "joint upper limit")
                if not lower < upper:
                    raise ValueError("joint lower limit must be smaller")
                hard_limit = (lower, upper)
                soft_limit = centered_soft_limit(lower, upper)
            else:
                axis = None
                hard_limit = None
                soft_limit = None
            spec = JointSpec(
                name=name,
                joint_type=joint_type,
                parent_link=parent,
                child_link=child,
                origin_xyz=origin_xyz,  # type: ignore
                origin_rpy=origin_rpy,  # type: ignore
                axis_xyz=axis,
                hard_limit_rad=hard_limit,
                soft_limit_rad=soft_limit,
                default_rad=None,
                robot29_index=None,
            )
            raw_joints.append(spec)
            children.setdefault(parent, []).append((child, spec))
        if len(set(joint_names)) != len(joint_names):
            raise ValueError("duplicate joint name")

        roots = links - child_links
        if len(roots) != 1:
            raise ValueError("URDF must have exactly one root link")
        root_link = next(iter(roots))
        topological: List[JointSpec] = []
        visited_links = {root_link}

        def visit(link_name: str) -> None:
            for child_name, spec in children.get(link_name, []):
                if child_name in visited_links:
                    raise ValueError("URDF joint graph contains a cycle")
                visited_links.add(child_name)
                topological.append(spec)
                visit(child_name)

        visit(root_link)
        if visited_links != links:
            raise ValueError("URDF contains disconnected links")

        mesh_paths: Dict[str, Path] = {}
        mesh_hashes: Dict[str, str] = {}
        relative_hashes: Dict[str, str] = {}
        visuals: List[VisualSpec] = []
        for link in link_elements:
            link_name = link.get("name")
            for visual in link.findall("visual"):
                geometry = visual.find("geometry")
                mesh = None if geometry is None else geometry.find("mesh")
                if mesh is None:
                    continue
                filename = mesh.get("filename")
                if not filename:
                    raise ValueError("mesh visual needs filename")
                mesh_path = self._resolve_mesh(filename)
                file_hash = _sha256_file(mesh_path)
                mesh_id = hashlib.sha256(
                    (str(mesh_path) + "\0" + file_hash).encode("utf-8")
                ).hexdigest()
                mesh_paths[mesh_id] = mesh_path
                mesh_hashes[mesh_id] = file_hash
                relative = os.path.relpath(
                    str(mesh_path), str(self.allowed_asset_root)
                )
                relative_hashes[relative] = file_hash
                origin = visual.find("origin")
                origin_xyz = _vector(
                    None if origin is None else origin.get("xyz"),
                    3,
                    (0.0, 0.0, 0.0),
                    "visual origin xyz",
                )
                origin_rpy = _vector(
                    None if origin is None else origin.get("rpy"),
                    3,
                    (0.0, 0.0, 0.0),
                    "visual origin rpy",
                )
                scale = _vector(
                    mesh.get("scale"),
                    3,
                    (1.0, 1.0, 1.0),
                    "mesh scale",
                )
                if any(value <= 0.0 for value in scale):
                    raise ValueError("mesh scale must be positive")
                material = visual.find("material")
                color = DEFAULT_COLOR_RGBA
                if material is not None:
                    inline = material.find("color")
                    if inline is not None:
                        color = _vector(
                            inline.get("rgba"),
                            4,
                            DEFAULT_COLOR_RGBA,
                            "visual rgba",
                        )  # type: ignore
                    elif material.get("name") in material_colors:
                        color = material_colors[material.get("name")]  # type: ignore
                visuals.append(
                    VisualSpec(
                        link_name=link_name,  # type: ignore
                        origin_xyz=origin_xyz,  # type: ignore
                        origin_rpy=origin_rpy,  # type: ignore
                        mesh_scale_xyz=scale,  # type: ignore
                        mesh_id=mesh_id,
                        color_rgba=color,
                    )
                )

        active = tuple(
            joint.name for joint in topological if joint.joint_type == "revolute"
        )
        indexed = []
        default_values = tuple(0.0 for _ in active)
        active_index = {name: index for index, name in enumerate(active)}
        for joint in topological:
            if joint.joint_type == "revolute":
                index = active_index[joint.name]
                indexed.append(
                    replace(joint, default_rad=0.0, robot29_index=index)
                )
            else:
                indexed.append(joint)
        joint_by_name = MappingProxyType(
            {joint.name: joint for joint in indexed}
        )
        urdf_kinematic = self._urdf_kinematic_payload(indexed)
        urdf_hash = _sha256_file(self.urdf_path)
        mesh_set_hash = _canonical_sha256(sorted(relative_hashes.items()))
        empty_hash = _canonical_sha256({"source": "not_evaluated"})
        manifest = AssetManifest(
            root_link=root_link,
            active_joint_names=active,
            arm_joint_names=tuple(
                name for name in ARM_JOINT_NAMES if name in active_index
            ),
            joints_topological=tuple(indexed),
            joint_by_name=joint_by_name,
            visuals=tuple(visuals),
            table_boxes=(),
            default_joint_pos_rad=default_values,
            asset_paths=MappingProxyType(
                {
                    "urdf": str(self.urdf_path),
                    "allowed_asset_root": str(self.allowed_asset_root),
                }
            ),
            asset_hashes=MappingProxyType(
                {
                    "display_urdf_sha256": urdf_hash,
                    "display_mesh_set_sha256": mesh_set_hash,
                }
            ),
            urdf_kinematic_sha256=_canonical_sha256(
                {"source": "urdf", "joints": urdf_kinematic}
            ),
            mjcf_kinematic_sha256=empty_hash,
            asset_signature_sha256=_canonical_sha256(
                (urdf_hash, mesh_set_hash, "not_evaluated")
            ),
            compatible_for_save=False,
            incompatibilities=("not_evaluated",),
            right_racket_link="right_racket_link",
        )
        return (
            manifest,
            mesh_paths,
            mesh_hashes,
            sorted(relative_hashes.items()),
        )

    @staticmethod
    def _urdf_kinematic_payload(joints: Sequence[JointSpec]):
        return [
            {
                "name": joint.name,
                "type": joint.joint_type,
                "parent": joint.parent_link,
                "child": joint.child_link,
                "origin_xyz": list(joint.origin_xyz),
                "origin_rpy": list(joint.origin_rpy),
                "axis": None if joint.axis_xyz is None else list(joint.axis_xyz),
                "limit": (
                    None
                    if joint.hard_limit_rad is None
                    else list(joint.hard_limit_rad)
                ),
            }
            for joint in joints
        ]


class HitterAssetModel:
    """Composes a read-only UrdfSceneModel with the live YAML/MJCF contract."""

    @classmethod
    def from_defaults(cls, repo_root: Path) -> "HitterAssetModel":
        repo_root = repo_root.resolve()
        return cls.from_paths(
            urdf_path=DISPLAY_URDF,
            mjcf_path=repo_root
            / "deploy/data/assets/g1/g1_29dof_hitter_racket_table_tennis.xml",
            asset_yaml_path=repo_root
            / "deploy/config/asset/g1_hitter_racket.yaml",
            allowed_asset_root=DISPLAY_ASSET_ROOT,
        )

    @classmethod
    def from_paths(
        cls,
        urdf_path: Path,
        mjcf_path: Path,
        asset_yaml_path: Path,
        allowed_asset_root: Path,
    ) -> "HitterAssetModel":
        return cls(
            urdf_path.resolve(),
            mjcf_path.resolve(),
            asset_yaml_path.resolve(),
            allowed_asset_root.resolve(),
        )

    def __init__(
        self,
        urdf_path: Path,
        mjcf_path: Path,
        asset_yaml_path: Path,
        allowed_asset_root: Path,
    ):
        for path in (mjcf_path, asset_yaml_path):
            if not path.is_file():
                raise FileNotFoundError(str(path))
        self.urdf_path = urdf_path
        self.mjcf_path = mjcf_path
        self.asset_yaml_path = asset_yaml_path
        self.allowed_asset_root = allowed_asset_root
        self._urdf = UrdfSceneModel.from_urdf(
            urdf_path, allowed_asset_root
        )
        with asset_yaml_path.open("r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        if not isinstance(config, dict):
            raise ValueError("asset YAML must be a mapping")
        names = tuple(config.get("kinematic_joint_names", ()))
        defaults = tuple(
            _finite_float(value, "default angle")
            for value in config.get("default_angles", ())
        )
        if not names or len(names) != len(defaults):
            raise ValueError("joint names and default angles must be nonempty and equal")
        if len(set(names)) != len(names):
            raise ValueError("duplicate kinematic joint name")
        joint_order = config.get("joint_order")
        if (
            not isinstance(joint_order, dict)
            or set(joint_order) != set(names)
        ):
            raise ValueError("joint_order and kinematic_joint_names differ")
        joint_indices = tuple(joint_order[name] for name in names)
        if (
            any(type(index) is not int for index in joint_indices)
            or joint_indices != tuple(range(len(names)))
        ):
            raise ValueError(
                "joint_order values must be exactly 0..N-1 "
                "in kinematic_joint_names order"
            )
        root_pos = tuple(
            _finite_float(value, "default root position")
            for value in config.get("default_root_pos", (0.0, 0.0, 0.0))
        )
        if len(root_pos) != 3:
            raise ValueError("default_root_pos must contain three values")

        self._active_joint_names = names
        self._default_joint_pos_rad = defaults
        self._joint_order = MappingProxyType(
            {name: joint_order[name] for name in names}
        )
        self._default_root_pos = root_pos
        self._mjcf_model = mujoco.MjModel.from_xml_path(str(mjcf_path))
        self._manifest = self._build_bundle_manifest()
        self._startup_hashes = MappingProxyType(
            {
                "urdf": _sha256_file(self.urdf_path),
                "mjcf": _sha256_file(self.mjcf_path),
                "asset_yaml": _sha256_file(self.asset_yaml_path),
                **{
                    "mesh:" + mesh_id: _sha256_file(
                        self._urdf.mesh_path(mesh_id)
                    )
                    for mesh_id in self._urdf.mesh_ids()
                },
            }
        )

    def build_manifest(self) -> AssetManifest:
        return self._manifest

    def mesh_path(self, mesh_id: str) -> Path:
        return self._urdf.mesh_path(mesh_id)

    def mesh_ids(self) -> Tuple[str, ...]:
        return self._urdf.mesh_ids()

    def assert_source_hashes_unchanged(self) -> None:
        current = {
            "urdf": _sha256_file(self.urdf_path),
            "mjcf": _sha256_file(self.mjcf_path),
            "asset_yaml": _sha256_file(self.asset_yaml_path),
            **{
                "mesh:" + mesh_id: _sha256_file(self.mesh_path(mesh_id))
                for mesh_id in self.mesh_ids()
            },
        }
        for name, expected in self._startup_hashes.items():
            if current.get(name) != expected:
                raise AssetSourceChanged("%s changed after startup" % name)

    def _mjcf_name(self, kind, object_id: int) -> str:
        name = mujoco.mj_id2name(self._mjcf_model, kind, object_id)
        if name is None:
            return ""
        return name

    def _mjcf_joint_records(self):
        records = []
        model = self._mjcf_model
        for name in self._active_joint_names:
            joint_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT, name
            )
            if joint_id < 0:
                raise ValueError("MJCF missing selected joint %s" % name)
            if int(model.jnt_type[joint_id]) != int(
                mujoco.mjtJoint.mjJNT_HINGE
            ):
                raise ValueError("selected joint %s is not a hinge" % name)
            body_id = int(model.jnt_bodyid[joint_id])
            parent_id = int(model.body_parentid[body_id])
            records.append(
                {
                    "name": name,
                    "parent": self._mjcf_name(
                        mujoco.mjtObj.mjOBJ_BODY, parent_id
                    ),
                    "child": self._mjcf_name(
                        mujoco.mjtObj.mjOBJ_BODY, body_id
                    ),
                    "origin_xyz": tuple(float(v) for v in model.body_pos[body_id]),
                    "origin_wxyz": tuple(
                        float(v) for v in model.body_quat[body_id]
                    ),
                    "joint_pos": tuple(
                        float(v) for v in model.jnt_pos[joint_id]
                    ),
                    "axis": _normalize_axis(
                        tuple(float(v) for v in model.jnt_axis[joint_id])
                    ),
                    "limit": tuple(
                        float(v) for v in model.jnt_range[joint_id]
                    ),
                }
            )
        return records

    def _fixed_racket_record(self):
        model = self._mjcf_model
        body_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "right_racket_link"
        )
        if body_id < 0:
            raise ValueError("MJCF missing right_racket_link")
        parent_id = int(model.body_parentid[body_id])
        return {
            "name": "right_racket_fixed_joint",
            "parent": self._mjcf_name(mujoco.mjtObj.mjOBJ_BODY, parent_id),
            "child": "right_racket_link",
            "origin_xyz": tuple(float(v) for v in model.body_pos[body_id]),
            "origin_wxyz": tuple(float(v) for v in model.body_quat[body_id]),
            "joint_count": int(model.body_jntnum[body_id]),
        }

    def _compatibility(self, joints: Mapping[str, JointSpec], records):
        issues: List[str] = []
        if tuple(record["name"] for record in records) != self._active_joint_names:
            issues.append("active joint order differs")
        for record in records:
            name = record["name"]
            urdf = joints.get(name)
            if urdf is None:
                issues.append("%s missing from URDF" % name)
                continue
            if (
                urdf.parent_link != record["parent"]
                or urdf.child_link != record["child"]
            ):
                issues.append("%s parent/child differs" % name)
            if _distance(urdf.origin_xyz, record["origin_xyz"]) > POSITION_TOLERANCE_M:
                issues.append("%s origin position differs" % name)
            if (
                _quat_angle_wxyz(
                    _rpy_to_wxyz(urdf.origin_rpy), record["origin_wxyz"]
                )
                > ORIENTATION_TOLERANCE_RAD
            ):
                issues.append("%s origin orientation differs" % name)
            if (
                _distance(record["joint_pos"], (0.0, 0.0, 0.0))
                > POSITION_TOLERANCE_M
            ):
                issues.append("%s joint position differs" % name)
            if urdf.axis_xyz is None or (
                _distance(urdf.axis_xyz, record["axis"]) > AXIS_TOLERANCE
            ):
                issues.append("%s axis differs" % name)
            if urdf.hard_limit_rad is None or any(
                abs(left - right) > LIMIT_TOLERANCE_RAD
                for left, right in zip(
                    urdf.hard_limit_rad, record["limit"]
                )
            ):
                issues.append("%s hard limit differs" % name)
        racket_urdf = joints.get("right_racket_fixed_joint")
        racket_mjcf = self._fixed_racket_record()
        if racket_mjcf["joint_count"] != 0:
            issues.append(
                "right_racket_fixed_joint body contains joints"
            )
        if racket_urdf is None:
            issues.append("right_racket_fixed_joint missing from URDF")
        else:
            if (
                racket_urdf.parent_link != racket_mjcf["parent"]
                or racket_urdf.child_link != racket_mjcf["child"]
            ):
                issues.append("right_racket_fixed_joint parent/child differs")
            if (
                _distance(
                    racket_urdf.origin_xyz, racket_mjcf["origin_xyz"]
                )
                > POSITION_TOLERANCE_M
            ):
                issues.append("right_racket_fixed_joint origin position differs")
            if (
                _quat_angle_wxyz(
                    _rpy_to_wxyz(racket_urdf.origin_rpy),
                    racket_mjcf["origin_wxyz"],
                )
                > ORIENTATION_TOLERANCE_RAD
            ):
                issues.append(
                    "right_racket_fixed_joint origin orientation differs"
                )
        return tuple(issues)

    def _table_boxes(self) -> Tuple[SceneBoxSpec, ...]:
        model = self._mjcf_model
        data = mujoco.MjData(model)
        free_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, "floating_base_joint"
        )
        if free_id >= 0:
            address = int(model.jnt_qposadr[free_id])
            data.qpos[address : address + 3] = self._default_root_pos
            data.qpos[address + 3 : address + 7] = (1.0, 0.0, 0.0, 0.0)
        for name, value in zip(
            self._active_joint_names, self._default_joint_pos_rad
        ):
            joint_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT, name
            )
            data.qpos[int(model.jnt_qposadr[joint_id])] = value
        mujoco.mj_forward(model, data)
        pelvis_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "pelvis"
        )
        if pelvis_id < 0:
            pelvis_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_BODY, self._urdf.build_manifest().root_link
            )
        pelvis_position = tuple(float(v) for v in data.xpos[pelvis_id])
        pelvis_rotation = tuple(float(v) for v in data.xmat[pelvis_id])

        def rotate_transpose(vector):
            return tuple(
                pelvis_rotation[0 * 3 + index] * vector[0]
                + pelvis_rotation[1 * 3 + index] * vector[1]
                + pelvis_rotation[2 * 3 + index] * vector[2]
                for index in range(3)
            )

        boxes = []
        for geom_id in range(model.ngeom):
            name = self._mjcf_name(mujoco.mjtObj.mjOBJ_GEOM, geom_id)
            if not name.startswith("hitter_table_"):
                continue
            if int(model.geom_type[geom_id]) != int(
                mujoco.mjtGeom.mjGEOM_BOX
            ):
                raise ValueError("%s must be an MJCF box geom" % name)
            delta = tuple(
                float(data.geom_xpos[geom_id][index]) - pelvis_position[index]
                for index in range(3)
            )
            center = rotate_transpose(delta)
            world_rotation = tuple(float(v) for v in data.geom_xmat[geom_id])
            relative_rotation = tuple(
                sum(
                    pelvis_rotation[k * 3 + row]
                    * world_rotation[k * 3 + column]
                    for k in range(3)
                )
                for row in range(3)
                for column in range(3)
            )
            boxes.append(
                SceneBoxSpec(
                    name=name,
                    frame="robot_base_default",
                    center_xyz_m=center,  # type: ignore
                    quat_xyzw=_matrix_to_xyzw(relative_rotation),
                    half_size_xyz_m=tuple(
                        float(v) for v in model.geom_size[geom_id]
                    ),
                    color_rgba=tuple(
                        float(v) for v in model.geom_rgba[geom_id]
                    ),
                )
            )
        return tuple(boxes)

    def _build_bundle_manifest(self) -> AssetManifest:
        generic = self._urdf.build_manifest()
        if tuple(generic.active_joint_names) != self._active_joint_names:
            raise ValueError(
                "URDF active joint order differs from asset YAML: %r != %r"
                % (generic.active_joint_names, self._active_joint_names)
            )
        active_index = dict(self._joint_order)
        enriched = []
        for joint in generic.joints_topological:
            if joint.joint_type == "revolute":
                index = active_index[joint.name]
                enriched.append(
                    replace(
                        joint,
                        default_rad=self._default_joint_pos_rad[index],
                        robot29_index=index,
                    )
                )
            else:
                enriched.append(joint)
        by_name = MappingProxyType({joint.name: joint for joint in enriched})
        records = self._mjcf_joint_records()
        issues = self._compatibility(by_name, records)
        urdf_kinematic = UrdfSceneModel._urdf_kinematic_payload(enriched)
        mjcf_kinematic = {
            "active": records,
            "right_racket_fixed": self._fixed_racket_record(),
        }
        urdf_kinematic_hash = _canonical_sha256(
            {"source": "urdf", "joints": urdf_kinematic}
        )
        mjcf_kinematic_hash = _canonical_sha256(
            {"source": "mjcf", "kinematics": mjcf_kinematic}
        )
        urdf_hash = _sha256_file(self.urdf_path)
        mjcf_hash = _sha256_file(self.mjcf_path)
        yaml_hash = _sha256_file(self.asset_yaml_path)
        mesh_set_hash = generic.asset_hashes["display_mesh_set_sha256"]
        signature = _canonical_sha256(
            {
                "compatibility_schema": COMPATIBILITY_SCHEMA,
                "display_urdf_sha256": urdf_hash,
                "display_mesh_set_sha256": mesh_set_hash,
                "validation_mjcf_sha256": mjcf_hash,
                "asset_yaml_sha256": yaml_hash,
            }
        )
        hashes = MappingProxyType(
            {
                "display_urdf_sha256": urdf_hash,
                "display_mesh_set_sha256": mesh_set_hash,
                "validation_mjcf_sha256": mjcf_hash,
                "asset_yaml_sha256": yaml_hash,
                "urdf_kinematic_sha256": urdf_kinematic_hash,
                "mjcf_kinematic_sha256": mjcf_kinematic_hash,
                "asset_signature_sha256": signature,
            }
        )
        return AssetManifest(
            root_link=generic.root_link,
            active_joint_names=self._active_joint_names,
            arm_joint_names=tuple(
                name for name in ARM_JOINT_NAMES if name in active_index
            ),
            joints_topological=tuple(enriched),
            joint_by_name=by_name,
            visuals=generic.visuals,
            table_boxes=self._table_boxes(),
            default_joint_pos_rad=self._default_joint_pos_rad,
            asset_paths=MappingProxyType(
                {
                    "urdf": str(self.urdf_path),
                    "mjcf": str(self.mjcf_path),
                    "asset_yaml": str(self.asset_yaml_path),
                    "allowed_asset_root": str(self.allowed_asset_root),
                }
            ),
            asset_hashes=hashes,
            urdf_kinematic_sha256=urdf_kinematic_hash,
            mjcf_kinematic_sha256=mjcf_kinematic_hash,
            asset_signature_sha256=signature,
            compatible_for_save=not issues,
            incompatibilities=issues,
            right_racket_link="right_racket_link",
        )

    def public_manifest(self) -> Dict[str, object]:
        manifest = self._manifest
        hashes = manifest.asset_hashes
        return {
            "schema": "hitter_asset_manifest/v1",
            "units": {"length": "m", "angle": "rad"},
            "limits": {
                "maxRequestBodyBytes": MAX_REQUEST_BODY_BYTES,
                "maxMeshBytes": MAX_MESH_BYTES,
                "maxStlTriangles": MAX_STL_TRIANGLES,
            },
            "frame": {
                "name": "robot_base_default",
                "rootLink": manifest.root_link,
                "rootTransform": "identity",
                "handedness": "right",
                "matrixLayout": "column-major",
                "quaternionConvention": "xyzw",
                "groundZRobotBaseM": -float(self._default_root_pos[2]),
            },
            "rootLink": manifest.root_link,
            "activeJointNames": list(manifest.active_joint_names),
            "armJointNames": list(manifest.arm_joint_names),
            "defaultJointPosRad": list(manifest.default_joint_pos_rad),
            "joints": [
                {
                    "name": joint.name,
                    "type": joint.joint_type,
                    "parentLink": joint.parent_link,
                    "childLink": joint.child_link,
                    "originXyzM": list(joint.origin_xyz),
                    "originRpyRad": list(joint.origin_rpy),
                    "axisXyz": (
                        None if joint.axis_xyz is None else list(joint.axis_xyz)
                    ),
                    "hardLimitRad": (
                        None
                        if joint.hard_limit_rad is None
                        else list(joint.hard_limit_rad)
                    ),
                    "softLimitRad": (
                        None
                        if joint.soft_limit_rad is None
                        else list(joint.soft_limit_rad)
                    ),
                    "defaultRad": joint.default_rad,
                    "robot29Index": joint.robot29_index,
                }
                for joint in manifest.joints_topological
            ],
            "visuals": [
                {
                    "linkName": visual.link_name,
                    "originXyzM": list(visual.origin_xyz),
                    "originRpyRad": list(visual.origin_rpy),
                    "meshScaleXyz": list(visual.mesh_scale_xyz),
                    "meshId": visual.mesh_id,
                    "colorRgba": list(visual.color_rgba),
                }
                for visual in manifest.visuals
            ],
            "tableVisuals": [
                {
                    "name": box.name,
                    "frame": box.frame,
                    "centerXyzM": list(box.center_xyz_m),
                    "quaternionXyzw": list(box.quat_xyzw),
                    "halfSizeXyzM": list(box.half_size_xyz_m),
                    "colorRgba": list(box.color_rgba),
                }
                for box in manifest.table_boxes
            ],
            "assetHashes": {
                "displayUrdfSha256": hashes["display_urdf_sha256"],
                "displayMeshSetSha256": hashes["display_mesh_set_sha256"],
                "validationMjcfSha256": hashes["validation_mjcf_sha256"],
                "assetYamlSha256": hashes["asset_yaml_sha256"],
                "urdfKinematicSha256": hashes["urdf_kinematic_sha256"],
                "mjcfKinematicSha256": hashes["mjcf_kinematic_sha256"],
                "assetSignatureSha256": hashes["asset_signature_sha256"],
            },
            "compatibleForSave": manifest.compatible_for_save,
            "incompatibilities": list(manifest.incompatibilities),
            "rightRacketLink": manifest.right_racket_link,
        }
