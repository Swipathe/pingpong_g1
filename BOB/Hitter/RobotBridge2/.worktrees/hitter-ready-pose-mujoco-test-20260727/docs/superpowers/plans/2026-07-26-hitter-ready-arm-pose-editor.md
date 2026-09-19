# HITTER Ready Arm Pose Editor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 `loco1` 上实现一个只读加载当前 HITTER G1 资产、可视化调整双臂 14 个关节、经 RobotBridge2 MuJoCo 权威复核后仅保存独立 YAML/JSON 姿态文件的本地网页工具。

**Architecture:** 新工具完全放在 `tools/hitter_ready_pose_editor/`，不接入 RobotBridge2 运行链。Python 服务端负责当前 URDF/MJCF 资产解析、会话鉴权、29 维组装、MuJoCo FK、两阶段保存与 commit-marker 原子可见性；无外网依赖的原生 JavaScript/WebGL 前端负责实时全身渲染、14 关节交互和独立浏览器 FK。浏览器 FK 与 MuJoCo FK 在每次保存前比较，资产签名不一致或误差超阈值时阻断保存。

**Tech Stack:** Python 3.8、标准库 `http.server`/`unittest`、PyYAML 6、NumPy、MuJoCo 3.2、RobotBridge2 `utils.kinematics.MujocoKinematics`、原生 ES modules、WebGL 1、Node.js 18 内置 `node:test`。

## Global Constraints

- 以已确认设计文档 `docs/superpowers/specs/2026-07-26-hitter-ready-arm-pose-editor-design.md` 为需求基线；如实现发现需求冲突，先更新并重新确认设计，不静默改变行为。
- 不修改或重启训练、RobotBridge2 真机/MuJoCo 进程、LCM、DDS、Vicon、ONNX、motion NPZ、URDF、USD、MJCF 或活跃 Hydra 配置。
- v1 只提供 14 个关节滑杆/数值输入，不实现末端拖拽、IK、自碰撞或球桌碰撞安全证明。
- 服务只允许绑定 `127.0.0.1`，Mac 通过 SSH tunnel 访问；首版不提供公网监听开关。
- 资产路径和保存目录由服务端固定。客户端不能提交任意路径，mesh 和已保存 pose 都通过启动时建立的白名单 ID 访问。
- UI 输入以度显示；API、FK、JSON/YAML 全部使用弧度。所有四元数统一为 `xyzw`，所有矩阵统一为右手系、列主序、列向量。
- 软限位按 hard range 中点收缩 90%：`mid ± 0.9 * (upper - lower) / 2`。soft range 外、hard limit 内只警告并要求二次确认；hard limit 外、NaN、Inf、空值、未知或缺失关节直接拒绝。
- 只覆盖 RobotBridge2 29 维中的索引 `15..28`；腿和腰始终从当前 `g1_hitter_racket.yaml` 默认 29 维复制。
- Python 必须兼容 3.8，不使用 `dict[str, ...]`、`str.removeprefix()`、`match` 或 `zoneinfo`。
- `MujocoKinematics` 共享可变 `MjData`，所有共享实例的 `forward()` 必须由 `threading.Lock` 串行保护。
- 浏览器与 MuJoCo 的最大位置误差不得超过 `0.0005 m`，最大姿态角误差不得超过 `math.radians(0.1)`。
- 每个任务遵守 Red → Green → Refactor；只 stage 当前任务列出的文件，绝不把当前脏工作树中的用户文件带入提交。
- 每个任务开始前必须执行 `git diff --cached --quiet`；若已有 staged 内容立即停止。修改 `requirements.txt` 或 `.gitignore` 前还要确认该文件没有未提交用户改动。每次 commit 前将 `git diff --cached --name-only` 与该任务 allowlist 逐项比对。

## File Map

### Create

```text
tools/__init__.py
tools/hitter_ready_pose_editor/__init__.py
tools/hitter_ready_pose_editor/constants.py
tools/hitter_ready_pose_editor/asset_model.py
tools/hitter_ready_pose_editor/pose_validation.py
tools/hitter_ready_pose_editor/pose_serialization.py
tools/hitter_ready_pose_editor/server.py
tools/hitter_ready_pose_editor/README.md
tools/hitter_ready_pose_editor/package.json
tools/hitter_ready_pose_editor/scripts/generate_live_fk_contract.py
tools/hitter_ready_pose_editor/tests/fixtures/minimal_robot.urdf
tools/hitter_ready_pose_editor/tests/fixtures/minimal_robot.xml
tools/hitter_ready_pose_editor/tests/fixtures/minimal_asset.yaml
tools/hitter_ready_pose_editor/tests/fixtures/minimal_ascii.stl
tools/hitter_ready_pose_editor/tests/__init__.py
tools/hitter_ready_pose_editor/tests/test_constants.py
tools/hitter_ready_pose_editor/tests/test_asset_model.py
tools/hitter_ready_pose_editor/tests/test_pose_validation.py
tools/hitter_ready_pose_editor/tests/test_pose_serialization.py
tools/hitter_ready_pose_editor/tests/test_server.py
tools/hitter_ready_pose_editor/web/index.html
tools/hitter_ready_pose_editor/web/styles.css
tools/hitter_ready_pose_editor/web/matrix.js
tools/hitter_ready_pose_editor/web/fk.js
tools/hitter_ready_pose_editor/web/stl.js
tools/hitter_ready_pose_editor/web/renderer.js
tools/hitter_ready_pose_editor/web/pose_state.js
tools/hitter_ready_pose_editor/web/api.js
tools/hitter_ready_pose_editor/web/app.js
tools/hitter_ready_pose_editor/web/tests/matrix.test.mjs
tools/hitter_ready_pose_editor/web/tests/fk.test.mjs
tools/hitter_ready_pose_editor/web/tests/stl.test.mjs
tools/hitter_ready_pose_editor/web/tests/pose_state.test.mjs
tools/hitter_ready_pose_editor/web/tests/api.test.mjs
tools/hitter_ready_pose_editor/web/tests/live_fk.test.mjs
```

### Modify

```text
requirements.txt
.gitignore
```

`requirements.txt` 只增加已经存在于 `rb` 环境中的显式运行依赖 `PyYAML==6.0.3`；`.gitignore` 只忽略运行时生成的 `deploy/data/hitter_ready_poses/`。不引入 pytest、Vite、Three.js 或浏览器 CDN。

Before every task:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2
git diff --cached --quiet || {
  echo "STOP: pre-existing staged changes must be resolved by their owner"
  exit 1
}
git status --short
```

Before Tasks 1 and 4 respectively, also run:

```bash
git diff --quiet -- requirements.txt || {
  echo "STOP: requirements.txt already has user changes"
  exit 1
}
git diff --quiet -- .gitignore || {
  echo "STOP: .gitignore already has user changes"
  exit 1
}
```

---

## Task 1: Lock the joint, index, unit, and limit contracts

**Files:**

- Create: `tools/__init__.py`
- Create: `tools/hitter_ready_pose_editor/__init__.py`
- Create: `tools/hitter_ready_pose_editor/tests/__init__.py`
- Create: `tools/hitter_ready_pose_editor/constants.py`
- Create: `tools/hitter_ready_pose_editor/tests/test_constants.py`
- Modify: `requirements.txt`

**Interfaces:** Consumes no feature code. Produces the one authoritative 14-joint order, RobotBridge2 and motion indices, schema/version constants, FK tolerances, and centered soft-limit function used by every later task.

- [ ] **Step 1: Write the failing constants tests**

```python
# tools/hitter_ready_pose_editor/tests/test_constants.py
import math
import unittest

import yaml

from tools.hitter_ready_pose_editor.constants import (
    ARM_JOINT_NAMES,
    MAX_MESH_BYTES,
    MAX_REQUEST_BODY_BYTES,
    MAX_STL_TRIANGLES,
    MOTION_NPZ_ARM_INDICES,
    ROBOT29_ARM_INDICES,
    centered_soft_limit,
)


class ConstantsTest(unittest.TestCase):
    def test_runtime_yaml_version_matches_declared_dependency(self):
        self.assertEqual(yaml.__version__, "6.0.3")

    def test_transport_and_mesh_limits_are_fixed(self):
        self.assertEqual(MAX_REQUEST_BODY_BYTES, 256 * 1024)
        self.assertEqual(MAX_MESH_BYTES, 64 * 1024 * 1024)
        self.assertEqual(MAX_STL_TRIANGLES, 1_000_000)

    def test_arm_order_and_index_contracts_are_exact(self):
        self.assertEqual(len(ARM_JOINT_NAMES), 14)
        self.assertEqual(ROBOT29_ARM_INDICES, tuple(range(15, 29)))
        self.assertEqual(
            MOTION_NPZ_ARM_INDICES,
            (11, 15, 19, 21, 23, 25, 27, 12, 16, 20, 22, 24, 26, 28),
        )
        self.assertEqual(ARM_JOINT_NAMES[0], "left_shoulder_pitch_joint")
        self.assertEqual(ARM_JOINT_NAMES[6], "left_wrist_yaw_joint")
        self.assertEqual(ARM_JOINT_NAMES[7], "right_shoulder_pitch_joint")
        self.assertEqual(ARM_JOINT_NAMES[13], "right_wrist_yaw_joint")

    def test_soft_limit_shrinks_about_range_midpoint(self):
        lower, upper = centered_soft_limit(-1.0, 3.0)
        self.assertTrue(math.isclose(lower, -0.8))
        self.assertTrue(math.isclose(upper, 2.8))

    def test_invalid_hard_limit_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "lower must be smaller"):
            centered_soft_limit(1.0, 1.0)
```

- [ ] **Step 2: Run the focused test and verify RED**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2
PYTHONDONTWRITEBYTECODE=1 \
  /home/loco1/miniconda3/envs/rb/bin/python \
  -m unittest tools.hitter_ready_pose_editor.tests.test_constants -v
```

Expected: `ModuleNotFoundError` for `tools.hitter_ready_pose_editor.constants`.

- [ ] **Step 3: Implement the immutable contracts**

```python
# tools/hitter_ready_pose_editor/constants.py
from typing import Tuple

ARM_JOINT_NAMES: Tuple[str, ...] = (
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)
ROBOT29_ARM_INDICES: Tuple[int, ...] = tuple(range(15, 29))
MOTION_NPZ_ARM_INDICES: Tuple[int, ...] = (
    11, 15, 19, 21, 23, 25, 27,
    12, 16, 20, 22, 24, 26, 28,
)
SOFT_LIMIT_FACTOR = 0.9
POSITION_TOLERANCE_M = 0.0005
ORIENTATION_TOLERANCE_RAD = 0.0017453292519943296
MAX_REQUEST_BODY_BYTES = 262144
MAX_MESH_BYTES = 67108864
MAX_STL_TRIANGLES = 1000000
POSE_SCHEMA = "hitter_ready_arm_pose/v1"


def centered_soft_limit(
    lower: float,
    upper: float,
    factor: float = SOFT_LIMIT_FACTOR,
) -> Tuple[float, float]:
    if not lower < upper:
        raise ValueError("lower must be smaller than upper")
    if not 0.0 < factor <= 1.0:
        raise ValueError("factor must be in (0, 1]")
    midpoint = (lower + upper) / 2.0
    half_range = (upper - lower) * factor / 2.0
    return midpoint - half_range, midpoint + half_range
```

Add `PyYAML==6.0.3` once to `requirements.txt`. Keep all three package/test `__init__.py` files empty.

- [ ] **Step 4: Run the focused test and verify GREEN**

Run the Step 2 command. Expected: 5 tests pass.

- [ ] **Step 5: Commit only Task 1 files**

```bash
git add requirements.txt tools/__init__.py tools/hitter_ready_pose_editor/__init__.py \
  tools/hitter_ready_pose_editor/tests/__init__.py \
  tools/hitter_ready_pose_editor/constants.py \
  tools/hitter_ready_pose_editor/tests/test_constants.py
/home/loco1/miniconda3/envs/rb/bin/python -c \
  'import subprocess,sys; a=set(subprocess.check_output(["git","diff","--cached","--name-only"],text=True).splitlines()); e=set(sys.argv[1:]); assert a==e,(sorted(e),sorted(a))' \
  requirements.txt tools/__init__.py \
  tools/hitter_ready_pose_editor/__init__.py \
  tools/hitter_ready_pose_editor/tests/__init__.py \
  tools/hitter_ready_pose_editor/constants.py \
  tools/hitter_ready_pose_editor/tests/test_constants.py
git diff --cached --check
git commit -m "feat: define HITTER ready pose contracts"
```

---

## Task 2: Build a read-only live asset model and whitelisted mesh manifest

**Files:**

- Create: `tools/hitter_ready_pose_editor/asset_model.py`
- Create: `tools/hitter_ready_pose_editor/tests/fixtures/minimal_robot.urdf`
- Create: `tools/hitter_ready_pose_editor/tests/fixtures/minimal_robot.xml`
- Create: `tools/hitter_ready_pose_editor/tests/fixtures/minimal_asset.yaml`
- Create: `tools/hitter_ready_pose_editor/tests/fixtures/minimal_ascii.stl`
- Create: `tools/hitter_ready_pose_editor/tests/test_asset_model.py`

**Interfaces:** Consumes Task 1 constants plus the approved URDF, STL, asset YAML, and MJCF paths. Produces an immutable internal `AssetManifest`, a fully specified camelCase public manifest, an opaque mesh registry, current source-hash guard, and no write capability.

- [ ] **Step 1: Write failing unit and live-contract tests**

The fixture URDF contains one revolute joint, the named left-palm/right-racket fixed branches, a nonzero visual origin, a non-unit mesh scale, and one STL visual. Tests cover parsing independently of the live HITTER asset, then add a read-only live integration assertion:

`setUp()` copies all fixture files into a fresh `TemporaryDirectory`; drift tests modify only those copies, never the checked-in fixture or live asset.

```python
class AssetModelTest(unittest.TestCase):
    def test_fixture_manifest_contains_tree_limit_and_opaque_mesh_id(self):
        model = UrdfSceneModel.from_urdf(
            urdf_path=self.fixture_urdf,
            allowed_asset_root=self.fixture_dir,
        )
        manifest = model.build_manifest()
        joint = manifest.joint_by_name["arm_joint"]
        self.assertEqual(joint.parent_link, "base_link")
        self.assertEqual(joint.child_link, "arm_link")
        self.assertEqual(joint.axis_xyz, (0.0, 1.0, 0.0))
        self.assertEqual(joint.hard_limit_rad, (-1.0, 2.0))
        mesh_id = manifest.visuals[0].mesh_id
        self.assertNotIn("/", mesh_id)
        self.assertEqual(model.mesh_path(mesh_id), self.fixture_stl.resolve())

    def test_unknown_or_path_like_mesh_id_is_rejected(self):
        model = self.build_fixture_model()
        with self.assertRaises(KeyError):
            model.mesh_path("../../etc/passwd")

    def test_fixed_joints_have_no_active_value_contract(self):
        manifest = self.build_fixture_model().build_manifest()
        for name in ("left_hand_palm_joint", "right_racket_fixed_joint"):
            spec = manifest.joint_by_name[name]
            self.assertEqual(spec.joint_type, "fixed")
            self.assertIsNone(spec.axis_xyz)
            self.assertIsNone(spec.hard_limit_rad)
            self.assertIsNone(spec.default_rad)
            self.assertIsNone(spec.robot29_index)

    def test_live_hitter_contract_has_current_29_joint_order(self):
        model = HitterAssetModel.from_defaults(self.repo_root)
        manifest = model.build_manifest()
        self.assertEqual(tuple(manifest.active_joint_names), tuple(self.asset_joint_order))
        self.assertEqual(tuple(manifest.arm_joint_names), ARM_JOINT_NAMES)
        self.assertEqual(manifest.right_racket_link, "right_racket_link")
        self.assertEqual(
            {visual.mesh_id for visual in manifest.visuals},
            set(model.mesh_ids()),
        )

    def test_live_wrist_limit_rounding_is_compatible(self):
        model = HitterAssetModel.from_defaults(self.repo_root)
        manifest = model.build_manifest()
        self.assertTrue(manifest.compatible_for_save, manifest.incompatibilities)
        self.assertLess(
            abs(-1.97222205 - -1.97222),
            1e-5,
        )
        self.assertEqual(
            manifest.table_boxes[0].frame,
            "robot_base_default",
        )

    def test_asset_yaml_drift_is_detected(self):
        model = self.build_fixture_bundle()
        self.fixture_asset_yaml.write_text(
            self.fixture_asset_yaml.read_text() + "\n# drift\n",
            encoding="utf-8",
        )
        with self.assertRaises(AssetSourceChanged):
            model.assert_source_hashes_unchanged()
```

Also assert:

- the live URDF and MJCF paths equal the two approved absolute paths;
- all resolved meshes remain under the approved `unitree_description` root using `os.path.commonpath`;
- duplicate link/joint names, multiple roots, unsupported joint types, missing mesh files, non-finite origins/axes, and missing limits fail fast;
- `build_manifest()` includes the current 29-D default angle mapped by name;
- all 37 URDF joints are emitted in topological order: 29 revolute joints consume a named angle, while 8 fixed joints consume no angle and preserve the full robot/racket transform tree;
- `right_racket_fixed_joint` and `left_hand_palm_joint` fixture chains reach the expected transforms without consuming an active angle;
- the live URDF and MJCF are compatible for all 29 selected joints and the right-racket fixed mount using position `1e-6 m`, normalized-axis `1e-8`, orientation `1e-5 rad`, and limit `1e-5 rad` tolerances;
- wrist-roll regression explicitly accepts URDF `[-1.97222205, 1.97222205]` versus MJCF `[-1.97222, 1.97222]`, whose `2.054e-6 rad` representation difference is below the limit tolerance;
- URDF and MJCF keep separate kinematic hashes; normal text/float-rounding differences never require those hashes to be equal;
- changing one copied bundle-fixture origin makes `compatible_for_save=False`, while pure `UrdfSceneModel` parsing reports compatibility as `not_evaluated`;
- the compiled table center is pelvis-relative: the current top center is approximately `[1.765369, 0.0, -0.058]`, not its MJCF world coordinate `[1.365369, 0.0, 0.735]`;
- the asset YAML hash participates in source drift and bundle signature checks.

- [ ] **Step 2: Run and verify RED**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2
env -u DISPLAY \
  PYTHONDONTWRITEBYTECODE=1 \
  MUJOCO_GL=egl \
  PYTHONPATH=deploy:. \
  /home/loco1/miniconda3/envs/rb/bin/python \
  -m unittest tools.hitter_ready_pose_editor.tests.test_asset_model -v
```

Expected: import failure because `asset_model.py` does not exist.

- [ ] **Step 3a: Implement immutable types and the generic URDF tree**

Use frozen dataclasses and public dictionaries containing JSON-safe lists only:

```python
@dataclass(frozen=True)
class JointSpec:
    name: str
    joint_type: str
    parent_link: str
    child_link: str
    origin_xyz: Tuple[float, float, float]
    origin_rpy: Tuple[float, float, float]
    axis_xyz: Tuple[float, float, float]
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
```

Implement:

```python
class UrdfSceneModel:
    @classmethod
    def from_urdf(
        cls,
        urdf_path: Path,
        allowed_asset_root: Path,
    ) -> "UrdfSceneModel":
        return cls(urdf_path.resolve(), allowed_asset_root.resolve())


class HitterAssetModel:
    """Composes a read-only UrdfSceneModel with the live YAML/MJCF contract."""
```

- [ ] **Step 3b: Add the live YAML/MJCF bundle constructor**

Implement these exact `HitterAssetModel` methods:

- `from_defaults(repo_root: Path) -> HitterAssetModel`;
- `from_paths(urdf_path: Path, mjcf_path: Path, asset_yaml_path: Path, allowed_asset_root: Path) -> HitterAssetModel`;
- `build_manifest() -> AssetManifest`;
- `public_manifest() -> Dict[str, object]`;
- `mesh_path(mesh_id: str) -> Path`;
- `mesh_ids() -> Tuple[str, ...]`;
- `assert_source_hashes_unchanged() -> None`.

- [ ] **Step 3c: Add the opaque mesh registry, compatibility comparison, and source guards**

Implement the mesh/path/hash and tolerance rules listed below, then rerun only the fixture, traversal, compatibility, and drift tests until they pass.

- [ ] **Step 3d: Freeze the Python→JavaScript public schema**

`public_manifest()` is the only Python→JavaScript conversion boundary and must return this exact camelCase shape:

```text
PublicManifestV1 = {
  schema: "hitter_asset_manifest/v1",
  units: {length: "m", angle: "rad"},
  limits: {
    maxRequestBodyBytes: 262144,
    maxMeshBytes: 67108864,
    maxStlTriangles: 1000000
  },
  frame: {
    name: "robot_base_default",
    rootLink: "pelvis",
    rootTransform: "identity",
    handedness: "right",
    matrixLayout: "column-major",
    quaternionConvention: "xyzw",
    groundZRobotBaseM: finite number
  },
  rootLink: string,
  activeJointNames: string[29],
  armJointNames: string[14],
  defaultJointPosRad: finite number[29],
  joints: Array<{
    name: string,
    type: "revolute" | "fixed",
    parentLink: string,
    childLink: string,
    originXyzM: finite number[3],
    originRpyRad: finite number[3],
    axisXyz: finite number[3] | null,
    hardLimitRad: finite number[2] | null,
    softLimitRad: finite number[2] | null,
    defaultRad: finite number | null,
    robot29Index: integer | null
  }>,
  visuals: Array<{
    linkName: string,
    originXyzM: finite number[3],
    originRpyRad: finite number[3],
    meshScaleXyz: positive finite number[3],
    meshId: lowercase SHA-256 string,
    colorRgba: finite number[4]
  }>,
  tableVisuals: Array<{
    name: string,
    frame: "robot_base_default",
    centerXyzM: finite number[3],
    quaternionXyzw: finite number[4],
    halfSizeXyzM: positive finite number[3],
    colorRgba: finite number[4]
  }>,
  assetHashes: {
    displayUrdfSha256: lowercase SHA-256 string,
    displayMeshSetSha256: lowercase SHA-256 string,
    validationMjcfSha256: lowercase SHA-256 string,
    assetYamlSha256: lowercase SHA-256 string,
    urdfKinematicSha256: lowercase SHA-256 string,
    mjcfKinematicSha256: lowercase SHA-256 string,
    assetSignatureSha256: lowercase SHA-256 string
  },
  compatibleForSave: boolean,
  incompatibilities: string[],
  rightRacketLink: "right_racket_link"
}
```

Add one Python golden-contract test that calls `public_manifest()`, asserts the exact top-level and nested key sets above, and round-trips `public` through `json.loads(json.dumps(public, allow_nan=False))`. Task 6’s JS fixture must use this camelCase shape verbatim, preventing a separate snake_case adapter.

Implementation requirements:

- parse XML with `xml.etree.ElementTree`;
- resolve `package://unitree_description/` from the approved asset root;
- derive opaque mesh IDs as SHA-256 of canonical path plus file SHA-256, never expose the filesystem path in an asset URL;
- reject any whitelisted source mesh larger than `MAX_MESH_BYTES` during startup before the HTTP server binds;
- compute a deterministic mesh-set hash from sorted `(relative_path, file_sha256)` pairs;
- inspect MJCF joint/body transforms through `mujoco.MjModel`, selecting exactly the 29 names from `g1_hitter_racket.yaml:kinematic_joint_names`; explicitly ignore `floating_base_joint`, `hitter_ball_freejoint`, table/ball bodies, and every unrelated extra body;
- generate independent canonical `urdf_kinematic_sha256` and `mjcf_kinematic_sha256`; determine compatibility by field-wise tolerances rather than hash equality;
- compute `asset_signature_sha256` from the raw URDF hash, raw MJCF hash, raw asset-YAML hash, mesh-set hash, and a versioned compatibility-schema string;
- keep approved resolved URDF/MJCF/asset-YAML absolute paths in internal `asset_paths` for the saved audit record, but omit them from `public_manifest()`;
- compare names, order, parent/child, joint origin, normalized axis, hard limit and racket fixed transform with position `1e-6 m`, axis `1e-8`, orientation `1e-5 rad`, and limit `1e-5 rad` tolerances;
- represent fixed joints with `None` limit/default/index and include them in the public topological tree, so the browser computes the racket and every fixed-link visual correctly;
- include visual mesh scale even though the current asset uses unit scale, and resolve named URDF materials before applying the default color;
- obtain the optional table directly from current MJCF geoms named `hitter_table_*`; use one MuJoCo forward result to compute `P_T_geom = inverse(W_T_pelvis) * W_T_geom`, then export `frame="robot_base_default"`, center, `xyzw` orientation, half-size and material RGBA; never hard-code table dimensions in JavaScript;
- cache the immutable manifest and mesh map after startup; provide no invalidation or write method;
- re-hash the startup URDF, every registered mesh, MJCF, and `g1_hitter_racket.yaml` in `assert_source_hashes_unchanged()` before validation and ticket consumption; any disk drift disables validation/save until a deliberate server restart rebuilds the manifest.

Do not import `AlignmentModel`, `save_origin`, or the old tuner HTTP handler from `MOSAIC-main/scripts/tune_urdf_hand_wrist_alignment.py`.

- [ ] **Step 4: Run and verify GREEN**

Run the Step 2 command. Expected: fixture and live-contract tests pass without opening a viewer or creating a RobotBridge process.

- [ ] **Step 5: Commit only Task 2 files**

```bash
git add tools/hitter_ready_pose_editor/asset_model.py \
  tools/hitter_ready_pose_editor/tests/fixtures/minimal_robot.urdf \
  tools/hitter_ready_pose_editor/tests/fixtures/minimal_robot.xml \
  tools/hitter_ready_pose_editor/tests/fixtures/minimal_asset.yaml \
  tools/hitter_ready_pose_editor/tests/fixtures/minimal_ascii.stl \
  tools/hitter_ready_pose_editor/tests/test_asset_model.py
/home/loco1/miniconda3/envs/rb/bin/python -c \
  'import subprocess,sys; a=set(subprocess.check_output(["git","diff","--cached","--name-only"],text=True).splitlines()); e=set(sys.argv[1:]); assert a==e,(sorted(e),sorted(a))' \
  tools/hitter_ready_pose_editor/asset_model.py \
  tools/hitter_ready_pose_editor/tests/fixtures/minimal_robot.urdf \
  tools/hitter_ready_pose_editor/tests/fixtures/minimal_robot.xml \
  tools/hitter_ready_pose_editor/tests/fixtures/minimal_asset.yaml \
  tools/hitter_ready_pose_editor/tests/fixtures/minimal_ascii.stl \
  tools/hitter_ready_pose_editor/tests/test_asset_model.py
git diff --cached --check
git commit -m "feat: expose read-only HITTER asset manifest"
```

---

## Task 3: Validate 14-D input, compose 29-D pose, and run locked MuJoCo FK

**Files:**

- Create: `tools/hitter_ready_pose_editor/pose_validation.py`
- Create: `tools/hitter_ready_pose_editor/tests/test_pose_validation.py`

**Interfaces:** Consumes `HitterAssetModel`, the current 29-name order/defaults, and `MujocoKinematics`. Produces a normalized `ValidationResult` whose only external FK representation is `robot_base_default` (the default pelvis frame) `{position_m, quaternion_xyzw}` for the two wrists and racket.

- [ ] **Step 1: Write failing input, mapping, FK, and error tests**

```python
class PoseValidatorTest(unittest.TestCase):
    def test_compose_joint_pos_only_replaces_arm_indices(self):
        pose = dict(self.default_arm_pose)
        pose["left_elbow_joint"] = 0.75
        full = self.validator.compose_joint_pos(pose)
        self.assertEqual(full.shape, (29,))
        np.testing.assert_allclose(full[:15], self.default_29[:15])
        self.assertEqual(full[18], 0.75)

    def test_default_fk_matches_current_regression_anchors(self):
        result = self.validator.validate(self.default_arm_pose)
        np.testing.assert_allclose(
            result.fk_robot_base["links"]["left_wrist_yaw_link"]["position_m"],
            [0.097296, 0.214477, -0.024402],
            atol=1e-5,
        )
        np.testing.assert_allclose(
            result.fk_robot_base["links"]["right_wrist_yaw_link"]["position_m"],
            [0.097296, -0.214467, -0.024402],
            atol=1e-5,
        )
        np.testing.assert_allclose(
            result.fk_robot_base["links"]["right_racket_link"]["position_m"],
            [0.288860, -0.227561, -0.078599],
            atol=1e-5,
        )
        self.assertEqual(result.fk_robot_base["frame"], "robot_base_default")
        self.assertEqual(result.fk_robot_base["quaternion_convention"], "xyzw")

    def test_hard_limit_and_nonfinite_values_are_rejected(self):
        for bad in (float("nan"), float("inf"), -float("inf")):
            pose = dict(self.default_arm_pose)
            pose["left_elbow_joint"] = bad
            with self.assertRaises(PoseInputError):
                self.validator.validate(pose)
        pose = dict(self.default_arm_pose)
        pose["left_elbow_joint"] = 99.0
        with self.assertRaises(PoseInputError):
            self.validator.validate(pose)

    def test_browser_fk_mismatch_blocks_validation(self):
        browser_fk = copy.deepcopy(
            self.validator.validate(self.default_arm_pose).fk_robot_base
        )
        browser_fk["links"]["right_racket_link"]["position_m"][0] += 0.001
        with self.assertRaisesRegex(PoseValidationError, "position"):
            self.validator.validate(self.default_arm_pose, browser_fk)

    def test_quaternion_sign_does_not_create_orientation_error(self):
        expected = self.validator.validate(self.default_arm_pose).fk_robot_base
        browser_fk = copy.deepcopy(expected)
        quat = browser_fk["links"]["right_racket_link"]["quaternion_xyzw"]
        browser_fk["links"]["right_racket_link"]["quaternion_xyzw"] = [
            -value for value in quat
        ]
        result = self.validator.validate(self.default_arm_pose, browser_fk)
        self.assertLessEqual(result.max_orientation_error_rad, 1e-12)

    def test_browser_fk_requires_finite_unit_quaternion_and_no_bool(self):
        expected = self.validator.validate(self.default_arm_pose).fk_robot_base
        for quaternion in ([0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 100.0]):
            browser_fk = copy.deepcopy(expected)
            browser_fk["links"]["right_racket_link"]["quaternion_xyzw"] = quaternion
            with self.assertRaisesRegex(PoseInputError, "unit quaternion"):
                self.validator.validate(self.default_arm_pose, browser_fk)
        browser_fk = copy.deepcopy(expected)
        browser_fk["links"]["right_racket_link"]["position_m"][0] = True
        with self.assertRaisesRegex(PoseInputError, "finite number"):
            self.validator.validate(self.default_arm_pose, browser_fk)
```

- [ ] **Step 1a: Add the exact edge-case test matrix**

Add these named test cases, with one assertion per row:

| Test | Input | Required result |
|---|---|---|
| `test_name_set_is_exact` | remove one required name; add `unknown_joint` | `PoseInputError` naming the missing/unknown key |
| `test_value_type_is_strict` | `True`, `"0.2"`, `None`, NaN, ±Inf | `PoseInputError`; accepted pose remains unchanged |
| `test_hard_limit_is_not_clamped` | `hard_max + 1e-6` | `PoseInputError` containing joint name/value/range |
| `test_soft_warning_is_nonblocking` | `soft_max + 1e-6`, still below hard max | one warning and `needs_soft_limit_confirmation is True` |
| `test_left_right_chains_are_independent` | change one left elbow, then one right wrist | opposite wrist unchanged at `1e-10`; racket changes only for right chain |
| `test_pelvis_relative_transform` | synthetic rotated/translated pelvis and child | exact `R_pelvis.T @ (p_child-p_pelvis)` and `R_pelvis.T @ R_child` |
| `test_asset_mismatch_precedes_fk` | fake source guard raises | fake FK call count remains zero |
| `test_fk_lock_serializes_two_threads` | two threads and a barrier-backed fake FK | maximum simultaneous fake-FK entries equals one |

- [ ] **Step 2: Run and verify RED**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2
env -u DISPLAY \
  PYTHONDONTWRITEBYTECODE=1 \
  MUJOCO_GL=egl \
  PYTHONPATH=deploy:. \
  /home/loco1/miniconda3/envs/rb/bin/python \
  -m unittest tools.hitter_ready_pose_editor.tests.test_pose_validation -v
```

Expected: import failure because `pose_validation.py` does not exist.

- [ ] **Step 3: Implement the validator around the existing FK utility**

```python
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
        self._default_root_pos = default_root_pos.copy()
        self._fk_lock = threading.Lock()
```

Use this single external FK JSON contract everywhere:

```text
FkSummaryV1 = {
  frame: "robot_base_default",
  quaternion_convention: "xyzw",
  links: {
    left_wrist_yaw_link: {
      position_m: finite number[3],
      quaternion_xyzw: normalized finite number[4]
    },
    right_wrist_yaw_link: {
      position_m: finite number[3],
      quaternion_xyzw: normalized finite number[4]
    },
    right_racket_link: {
      position_m: finite number[3],
      quaternion_xyzw: normalized finite number[4]
    }
  }
}
```

- [ ] **Step 3a: Implement strict input validation and 14→29 composition**

```python
def compose_joint_pos(
    self,
    joint_pos_by_name: Mapping[str, object],
) -> np.ndarray:
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
    for name, robot_index in zip(ARM_JOINT_NAMES, ROBOT29_ARM_INDICES):
        raw = joint_pos_by_name[name]
        if type(raw) not in (int, float) or not math.isfinite(float(raw)):
            raise PoseInputError("{} must be a finite number".format(name))
        value = float(raw)
        spec = self._manifest.joint_by_name[name]
        lower, upper = spec.hard_limit_rad
        if value < lower or value > upper:
            raise PoseInputError(
                "{}={} outside hard limit [{}, {}]".format(
                    name, value, lower, upper
                )
            )
        full[robot_index] = value
    return full
```

In the same pass, compare every accepted value against `soft_limit_rad` and append deterministic warning strings in `ARM_JOINT_NAMES` order.

- [ ] **Step 3b: Implement locked MuJoCo FK and pelvis conversion**

Instantiate only:

```python
MujocoKinematics(
    ForwardKinematicsConfig(
        xml_path=str(mjcf_path),
        kinematic_joint_names=list(manifest.active_joint_names),
        debug_viz=False,
    )
)
```

Never instantiate `simulator.mujoco.Mujoco` or `RealWorld`. Call:

```python
with self._fk_lock:
    body_info, _ = self._kinematics.forward(
        joint_pos_29,
        base_pos=self._default_root_pos,
        base_quat=np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64),
)
```

Read bodies by name (`pelvis`, `left_wrist_yaw_link`, `right_wrist_yaw_link`, `right_racket_link`), never by tensor row number. Convert each target with:

```python
root_rotation = quat_xyzw_to_matrix(body_info["pelvis"]["quat"])
root_position = body_info["pelvis"]["pos"]
link_rotation = quat_xyzw_to_matrix(link_info["quat"])
position_m = root_rotation.T.dot(link_info["pos"] - root_position)
rotation_pelvis = root_rotation.T.dot(link_rotation)
quaternion_xyzw = matrix_to_canonical_quat_xyzw(rotation_pelvis)
```

Canonicalize quaternion sign to non-negative `w` before serialization. The browser root matrix is identity at `pelvis`, so its FK summary is already in this same frame.

Expose one reusable non-HTTP method `forward_links_robot_base(joint_pos_29, link_names) -> Dict[str, Dict[str, List[float]]]`. It owns the same `_fk_lock`, always includes `pelvis`, validates every requested body name, and returns canonical `position_m`/`quaternion_xyzw`. `validate()` calls it for the three save-critical links; Task 10 calls it for the 31-link live parity contract. There must be only one MuJoCo-to-`robot_base_default` conversion implementation.

- [ ] **Step 3c: Implement schema validation and FK error comparison**

Validate the browser object against `FkSummaryV1` with exact keys and finite lengths. Reject booleans even though Python treats them as integers. Each quaternion must contain four finite non-boolean numbers and satisfy `abs(np.linalg.norm(q) - 1.0) <= 1e-6`; reject zero, scaled, or non-finite quaternions before computing a dot product. For each of the three links:

```python
position_error = float(
    np.linalg.norm(client_position - mujoco_position)
)
dot = float(abs(np.dot(client_quaternion, mujoco_quaternion)))
orientation_error = 2.0 * math.acos(min(1.0, dot))
```

Reject when either maximum exceeds the constants. `validate()` must:

1. call `assert_source_hashes_unchanged()`;
2. reject `compatible_for_save=False`;
3. compose 29-D input and soft warnings;
4. run one locked FK;
5. convert MuJoCo output to `FkSummaryV1`;
6. compare the optional browser summary;
7. return the frozen `ValidationResult`.

`validate()` must call `self._asset_model.assert_source_hashes_unchanged()` before composing the 29-D vector. Convert browser root-relative FK and MuJoCo world FK into the same pelvis-relative frame before comparing; world regression anchors remain tests only and are never serialized as the authoritative pose.

- [ ] **Step 4: Run and verify GREEN**

Run the Step 2 command. Expected: all validation tests pass under `env -u DISPLAY`, with no window, LCM, DDS, background thread, or GPU process.

- [ ] **Step 5: Commit only Task 3 files**

```bash
git add tools/hitter_ready_pose_editor/pose_validation.py \
  tools/hitter_ready_pose_editor/tests/test_pose_validation.py
/home/loco1/miniconda3/envs/rb/bin/python -c \
  'import subprocess,sys; a=set(subprocess.check_output(["git","diff","--cached","--name-only"],text=True).splitlines()); e=set(sys.argv[1:]); assert a==e,(sorted(e),sorted(a))' \
  tools/hitter_ready_pose_editor/pose_validation.py \
  tools/hitter_ready_pose_editor/tests/test_pose_validation.py
git diff --cached --check
git commit -m "feat: validate HITTER ready arm poses"
```

---

## Task 4: Serialize a crash-consistent YAML/JSON generation and reload safely

**Files:**

- Create: `tools/hitter_ready_pose_editor/pose_serialization.py`
- Create: `tools/hitter_ready_pose_editor/tests/test_pose_serialization.py`
- Modify: `.gitignore`

**Interfaces:** Consumes a successful `ValidationResult` and its exact startup asset manifest. Produces a schema-v1 record, a committed YAML/JSON generation, opaque `pose_id`, safe list/load operations, and absolute success paths; it accepts no client path.

- [ ] **Step 1: Write failing schema, mapping, crash-consistency, and traversal tests**

```python
class PoseStoreTest(unittest.TestCase):
    def test_saved_yaml_and_json_have_identical_semantics(self):
        record = build_pose_record(self.validation_result, self.asset_manifest)
        record["validation"]["soft_limits"] = "passed"
        saved = self.store.save(record, now=self.fixed_time)
        with saved.json_path.open("r", encoding="utf-8") as stream:
            json_data = json.load(stream)
        with saved.yaml_path.open("r", encoding="utf-8") as stream:
            yaml_data = yaml.safe_load(stream)
        self.assertEqual(json_data, yaml_data)
        self.assertEqual(json_data["schema"], "hitter_ready_arm_pose/v1")
        self.assertEqual(json_data["unit"], "rad")
        self.assertEqual(len(json_data["joint_names"]), 14)
        self.assertEqual(len(json_data["robot29"]["joint_pos_rad"]), 29)
        self.assertEqual(
            json_data["motion_npz"]["indices"],
            list(MOTION_NPZ_ARM_INDICES),
        )
        self.assertEqual(json_data["pose_name"], saved.pose_id)
        self.assertEqual(
            json_data["robot29"]["indices"], list(range(29))
        )
        self.assertEqual(json_data["fk"]["frame"], "robot_base_default")
        self.assertEqual(json_data["fk"]["root_link"], "pelvis")
        self.assertEqual(
            json_data["fk"]["quaternion_convention"], "xyzw"
        )
        self.assertEqual(json_data["validation"]["hard_limits"], "passed")
        self.assertEqual(
            json_data["validation"]["browser_mujoco_fk"], "passed"
        )

    def test_same_second_creates_suffix_without_overwrite(self):
        first = self.store.save(self.record, now=self.fixed_time)
        second = self.store.save(self.record, now=self.fixed_time)
        self.assertNotEqual(first.json_path, second.json_path)
        self.assertTrue(first.json_path.exists())
        self.assertTrue(second.json_path.exists())

    def test_load_accepts_only_server_listed_pose_id(self):
        with self.assertRaises(PoseStoreError):
            self.store.load("../../config/mimic/hitter.yaml")
```

- [ ] **Step 1a: Add exact schema and crash-recovery tests**

The completed record must have exactly this semantic shape:

```text
PoseRecordV1 = {
  schema: "hitter_ready_arm_pose/v1",
  pose_name: same string as pose_id and both filename stems,
  created_at: ISO-8601 with +08:00 offset,
  unit: "rad",
  joint_pos_by_name: object with exactly 14 names in ARM_JOINT_NAMES order,
  joint_names: ARM_JOINT_NAMES,
  joint_pos_rad: finite number[14],
  robot29: {
    indices: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
              15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28],
    joint_names: current activeJointNames[29],
    joint_pos_rad: finite number[29]
  },
  motion_npz: {
    indices: MOTION_NPZ_ARM_INDICES,
    joint_names: ARM_JOINT_NAMES,
    joint_pos_rad: same finite number[14]
  },
  fk: {
    frame: "robot_base_default",
    root_link: "pelvis",
    quaternion_convention: "xyzw",
    links: FkSummaryV1.links
  },
  asset: {
    display_urdf_path: approved absolute path,
    display_urdf_sha256: SHA-256,
    display_mesh_set_sha256: SHA-256,
    validation_mjcf_path: approved absolute path,
    validation_mjcf_sha256: SHA-256,
    asset_yaml_path: approved absolute path,
    asset_yaml_sha256: SHA-256,
    urdf_kinematic_sha256: SHA-256,
    mjcf_kinematic_sha256: SHA-256,
    asset_signature_sha256: SHA-256
  },
  validation: {
    hard_limits: "passed",
    soft_limits: "passed" | "warning_confirmed",
    browser_mujoco_fk: "passed",
    max_position_error_m: finite number <= 0.0005,
    max_orientation_error_rad: finite number <= radians(0.1),
    soft_limit_warnings: string[]
  }
}
```

Add these named tests with the exact expected result:

| Test | Fault/input | Required result |
|---|---|---|
| `test_joint_name_is_authoritative` | swap two anonymous-array values only | record validation rejects disagreement with `joint_pos_by_name` |
| `test_robot_and_motion_mappings_are_exact` | normal record | all three arrays reconstruct from the name-keyed field |
| `test_output_dir_is_created_securely` | absent nested temp path | directory created mode `0700`; file/dir symlink target is rejected |
| `test_failure_after_json_rename_is_not_visible` | injected hook raises after JSON rename | `list_records()` returns none; restart moves orphan to `.incomplete/` |
| `test_failure_after_yaml_rename_is_not_visible` | injected hook raises after YAML rename | same result; neither file is loadable by pose ID |
| `test_commit_marker_is_visibility_point` | normal marker publication | complete pair becomes listable only after marker rename |
| `test_post_commit_fault_returns_committed_result` | hook raises after marker rename | `save()` returns the one committed pose with `storage_warning`; it does not raise a retryable error |
| `test_marker_hash_mismatch_is_rejected` | alter one committed byte | list/load quarantine generation and raise bounded error |
| `test_list_is_newest_first` | save two fixed timestamps | two opaque pose IDs ordered descending |
| `test_schema_and_values_are_strict` | schema mismatch, duplicate/unknown name, NaN/Inf | reject before publication |

Implement the two transaction-boundary tests as runnable code:

```python
def test_failure_after_yaml_rename_is_not_visible(self):
    store = PoseStore(self.output_dir, fault_hook=RaiseAt("after_yaml_rename"))
    with self.assertRaises(InjectedStoreFailure):
        store.save(self.valid_record, now=self.fixed_time)
    restarted = PoseStore(self.output_dir)
    self.assertEqual(restarted.list_records(), [])
    self.assertGreater(len(list((self.output_dir / ".incomplete").iterdir())), 0)


def test_post_commit_fault_returns_committed_result(self):
    store = PoseStore(self.output_dir, fault_hook=RaiseAt("after_marker_rename"))
    saved = store.save(self.valid_record, now=self.fixed_time)
    self.assertIsNotNone(saved.storage_warning)
    self.assertEqual(
        [item["pose_id"] for item in store.list_records()],
        [saved.pose_id],
    )
    self.assertEqual(store.load(saved.pose_id)["pose_name"], saved.pose_id)
```

- [ ] **Step 2: Run and verify RED**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2
PYTHONDONTWRITEBYTECODE=1 \
  PYTHONPATH=deploy:. \
  /home/loco1/miniconda3/envs/rb/bin/python \
  -m unittest tools.hitter_ready_pose_editor.tests.test_pose_serialization -v
```

Expected: import failure because `pose_serialization.py` does not exist.

- [ ] **Step 3a: Implement deterministic record construction**

Use China Standard Time without Python 3.9 `zoneinfo`:

```python
CHINA_TZ = datetime.timezone(datetime.timedelta(hours=8))
```

Implement:

- frozen `SavedPosePaths(pose_id, created_at, json_path, yaml_path, storage_warning=None)`;
- `build_pose_record(validation, manifest) -> Dict[str, object]`, which builds every semantic field except `pose_name`, `created_at`, and the user-confirmation-dependent `validation.soft_limits`;
- `PoseStore(output_dir, fault_hook=None)`, where production passes no hook and tests inject a callable;
- `PoseStore.save(record, now=None) -> SavedPosePaths`;
- `PoseStore.list_records() -> List[Dict[str, object]]`;
- `PoseStore.load(pose_id) -> Dict[str, object]`;
- private `PoseStore.recover_incomplete()` and injected no-op `_fault_hook(phase)` used only by tests.

`PoseStore.__init__` must create the fixed directory with parents and mode `0700`, reject a symlink/non-directory, confirm it is writable by the current owner, create a mode-`0700` `.incomplete/` quarantine, and run recovery under the store lock.

- [ ] **Step 3b: Implement marker-based committed-generation visibility**

Two independent `os.replace` calls are not cross-file atomic. The reader-visible transaction point is therefore a hidden commit marker:

```text
<pose_id>.json
<pose_id>.yaml
.<pose_id>.commit.json
```

Under `PoseStore._lock`:

1. validate and normalize before creating files;
2. select `hitter_ready_arm_pose_YYYYMMDD_HHMMSS` or free `_001` suffix;
3. inject matching `pose_name` and `created_at`;
4. create each temporary file with `os.open(temp_path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)`;
5. write, flush, and `os.fsync` JSON/YAML;
6. rename JSON then YAML, call `_fault_hook("after_json_rename")` / `_fault_hook("after_yaml_rename")`, then fsync the output directory;
7. preconstruct `SavedPosePaths`, write marker JSON containing pose ID plus both final SHA-256 values, fsync it, call `_fault_hook("before_marker_rename")`, then rename it to `.<pose_id>.commit.json`; this rename is the commit point;
8. after the commit point, attempt directory fsync and `_fault_hook("after_marker_rename")` inside a catch-all that logs the detailed exception server-side but returns the already committed `SavedPosePaths` with the fixed Chinese `storage_warning="姿态已保存，但目录持久化确认出现警告；请勿重复保存"`; no exception after marker publication may be reported as a retryable failed save;
9. `list_records()` and `load()` recognize only generations with all three files and matching hashes;
10. `recover_incomplete()` moves this tool’s uncommitted final/temp files into `.incomplete/` with a collision-free suffix. It never removes or overwrites an older committed generation.

This provides atomic visibility to the application and recoverable crash consistency while preserving the confirmed flat YAML/JSON success paths.

Serialize JSON with `allow_nan=False`, `ensure_ascii=False`, `indent=2`; serialize YAML with `yaml.safe_dump(sort_keys=False, allow_unicode=True)`.

Add only this generated-artifact pattern to `.gitignore`:

```gitignore
deploy/data/hitter_ready_poses/
```

- [ ] **Step 4: Run and verify GREEN**

Run the Step 2 command. Expected: all serializer tests pass using a temporary output directory.

- [ ] **Step 5: Commit only Task 4 files**

```bash
git add .gitignore \
  tools/hitter_ready_pose_editor/pose_serialization.py \
  tools/hitter_ready_pose_editor/tests/test_pose_serialization.py
/home/loco1/miniconda3/envs/rb/bin/python -c \
  'import subprocess,sys; a=set(subprocess.check_output(["git","diff","--cached","--name-only"],text=True).splitlines()); e=set(sys.argv[1:]); assert a==e,(sorted(e),sorted(a))' \
  .gitignore \
  tools/hitter_ready_pose_editor/pose_serialization.py \
  tools/hitter_ready_pose_editor/tests/test_pose_serialization.py
git diff --cached --check
git commit -m "feat: store versioned HITTER ready poses"
```

---

## Task 5: Add the loopback-only HTTP API and two-stage save ticket

**Files:**

- Create: `tools/hitter_ready_pose_editor/server.py`
- Create: `tools/hitter_ready_pose_editor/tests/test_server.py`

**Interfaces:** Consumes Tasks 2–4 application objects. Produces the exact authenticated HTTP JSON contract and single-use validation tickets; it does not import robot-control, simulator, planner, LCM, DDS, or Vicon modules.

- [ ] **Step 1: Write failing route, auth, request-boundary, and save-ticket tests**

Start an ephemeral `ThreadingHTTPServer(("127.0.0.1", 0), handler)` in the test class. Use a temporary pose directory and fake asset/validator objects for HTTP behavior tests.

```python
class ServerTest(unittest.TestCase):
    def test_api_rejects_missing_or_wrong_session_token(self):
        for headers in ({}, {"X-Hitter-Editor-Token": "wrong"}):
            status, _ = self.request("GET", "/api/model", headers=headers)
            self.assertEqual(status, 403)

    def test_validate_then_save_uses_immutable_ticket(self):
        status, validation = self.request_json(
            "POST",
            "/api/validate",
            body=self.valid_pose_payload,
            authenticated=True,
        )
        self.assertEqual(status, 200)
        status, saved = self.request_json(
            "POST",
            "/api/save",
            body={
                "validation_id": validation["validation_id"],
                "confirm_soft_limit": False,
            },
            authenticated=True,
        )
        self.assertEqual(status, 201)
        self.assertTrue(saved["json_path"].endswith(".json"))
        self.assertTrue(saved["yaml_path"].endswith(".yaml"))

    def test_save_cannot_choose_path_or_reuse_ticket(self):
        validation_id = self.validate_once()
        status, _ = self.request_json(
            "POST",
            "/api/save",
            body={
                "validation_id": validation_id,
                "output_path": "../../config/mimic/hitter.yaml",
            },
            authenticated=True,
        )
        self.assertEqual(status, 400)
        self.save_ticket(validation_id)
        status, _ = self.save_ticket(validation_id)
        self.assertEqual(status, 409)

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
            thread.join()
        self.assertEqual(sorted(statuses), [201, 409])

    def test_postcommit_fault_is_success_and_ticket_is_consumed(self):
        validation_id = self.validate_once()
        self.store.fault_phase = "after_marker_rename"
        status, body = self.save_ticket(validation_id)
        self.assertEqual(status, 201)
        self.assertIn("storage_warning", body)
        self.assertEqual(len(self.real_store.list_records()), 1)
        status, _ = self.save_ticket(validation_id)
        self.assertEqual(status, 409)
        self.assertEqual(len(self.real_store.list_records()), 1)
```

- [ ] **Step 1a: Lock the exact HTTP schema**

```text
GET /api/health
200 {
  status: "ok" | "read_only",
  schema: "hitter_ready_pose_editor_api/v1",
  asset_signature_sha256: SHA-256,
  incompatibilities: string[]
}

GET /api/model
200 PublicManifestV1

GET /api/poses
200 {poses: Array<{pose_id: string, created_at: ISO-8601}>}

GET /api/poses/{pose_id}
200 {pose: PoseRecordV1}

POST /api/validate
request {
  joint_pos_by_name: exact name-keyed finite 14-D radians,
  browser_fk: FkSummaryV1,
  asset_signature_sha256: SHA-256 from the displayed manifest
}
response 200 {
  validation_id: random opaque string,
  expires_in_s: 600,
  joint_pos_by_name: normalized name-keyed 14-D,
  joint_pos_29_rad: finite number[29],
  fk: FkSummaryV1,
  soft_limit_warnings: string[],
  needs_soft_limit_confirmation: boolean,
  max_position_error_m: finite number,
  max_orientation_error_rad: finite number,
  asset_hashes: PublicManifestV1.assetHashes
}

POST /api/save
request {
  validation_id: opaque string,
  confirm_soft_limit: boolean
}
response 201 {
  pose_id: string,
  created_at: ISO-8601,
  json_path: fixed absolute success path,
  yaml_path: fixed absolute success path,
  storage_warning: string | null
}

error 4xx/5xx {
  error: bounded Chinese message,
  code: stable lowercase identifier
}
```

Every JSON object rejects extra fields. `pose_id` is not a client filename: it must exactly match an ID returned by the current `PoseStore.list_records()` allowlist.

- [ ] **Step 1b: Add the exact route and security test matrix**

| Test | Required result |
|---|---|
| `test_static_shell_is_public_but_contains_no_data` | `/` and fixed static files load without token; shell contains no path/hash/pose |
| `test_wrong_query_token_is_rejected` | `/?token=wrong` returns 403; no-query refresh still returns the empty shell |
| `test_api_requires_header_token` | missing/wrong `X-Hitter-Editor-Token` returns 403 |
| `test_methods_and_content_types_are_exact` | every route above accepts only its declared method/type |
| `test_model_can_be_read_only` | model exposes incompatibilities; validate returns 409 |
| `test_mesh_hash_and_id_are_both_whitelisted` | stale hash, unknown ID, traversal, symlink escape, and any Range request are rejected |
| `test_static_files_are_allowlisted` | encoded traversal and unknown basename return 404; no filesystem translation helper is used |
| `test_json_boundary_is_strict` | >256 KiB, duplicate keys, invalid UTF-8, array root, extra field, wrong content type return 400/413/415 |
| `test_ticket_expiry_and_asset_drift` | expired or source-drifted ticket returns 409 without saving |
| `test_soft_confirmation_is_exact_bool` | warning ticket rejects missing, `1`, and `"true"`; accepts only JSON `true` |
| `test_precommit_store_failure_keeps_ticket_retryable` | injected failure before marker rename returns 500; second call with same ticket succeeds 201 |
| `test_postcommit_warning_consumes_ticket_once` | injected post-marker fault returns 201 with warning and one generation; second call is 409 |
| `test_error_and_log_redaction` | response and access log contain no traceback, token, absolute asset path, or request query |
| `test_loopback_parser` | only literal `127.0.0.1` accepted; `localhost`, `::1`, `0.0.0.0`, hostnames rejected |

- [ ] **Step 2: Run and verify RED**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2
env -u DISPLAY \
  PYTHONDONTWRITEBYTECODE=1 \
  MUJOCO_GL=egl \
  PYTHONPATH=deploy:. \
  /home/loco1/miniconda3/envs/rb/bin/python \
  -m unittest tools.hitter_ready_pose_editor.tests.test_server -v
```

Expected: import failure because `server.py` does not exist.

- [ ] **Step 3a: Implement the application and retry-safe ticket transaction**

```python
@dataclass
class PreparedPose:
    validation: ValidationResult
    record: Dict[str, object]
    expires_monotonic: float
    asset_signature_sha256: str


class EditorApplication:
    def __init__(
        self,
        asset_model: HitterAssetModel,
        validator: PoseValidator,
        store: PoseStore,
        web_root: Path,
        session_token: str,
    ):
        self.asset_model = asset_model
        self.validator = validator
        self.store = store
        self.web_root = web_root.resolve()
        self.session_token = session_token
        self._tickets: Dict[str, PreparedPose] = {}
        self._ticket_lock = threading.Lock()
```

`prepare_pose()` validates the exact request schema, compares the client asset signature, runs `PoseValidator.validate()`, builds the semantic record, stores an opaque 600-second `PreparedPose`, and returns the response schema above.

`consume_ticket()` must hold `_ticket_lock` through asset recheck and `PoseStore.save()`:

```python
with self._ticket_lock:
    prepared = self._tickets.get(validation_id)
    if prepared is None or time.monotonic() >= prepared.expires_monotonic:
        self._tickets.pop(validation_id, None)
        raise TicketConflict("validation ticket is missing or expired")
    if prepared.validation.needs_soft_limit_confirmation:
        if confirm_soft_limit is not True:
            raise SoftLimitConfirmationRequired()
    self.asset_model.assert_source_hashes_unchanged()
    current = self.asset_model.build_manifest().asset_signature_sha256
    if not hmac.compare_digest(current, prepared.asset_signature_sha256):
        raise TicketConflict("asset signature changed")
    record = copy.deepcopy(prepared.record)
    record["validation"]["soft_limits"] = (
        "warning_confirmed"
        if prepared.validation.needs_soft_limit_confirmation
        else "passed"
    )
    saved = self.store.save(record)
    del self._tickets[validation_id]
    return saved
```

Delete the ticket only after a successful committed generation. Holding the lock guarantees concurrent consumers produce exactly one save; an I/O exception leaves the unexpired ticket available for a safe retry.

- [ ] **Step 3b: Implement fixed routes, authentication, and redacted logging**

`make_handler(app)` creates one `BaseHTTPRequestHandler` subclass with exact route dispatch:

```text
GET  /, /static/index.html, /static/styles.css, and the seven known JS basenames
GET  /api/health
GET  /api/model
GET  /api/poses
GET  /api/poses/{pose_id}
GET  /assets/{displayMeshSetSha256}/{meshId}
POST /api/validate
POST /api/save
```

Static files use a fixed basename→resolved-Path dictionary built from `web_root`; never use `SimpleHTTPRequestHandler.translate_path`. API/assets/pose routes call:

```python
provided = self.headers.get("X-Hitter-Editor-Token", "")
if not hmac.compare_digest(provided, app.session_token):
    self.send_json(403, {"error": "会话令牌无效", "code": "forbidden"})
    return False
return True
```

If `/` has a `token` query, reject a wrong value; a correct value and no query both return the same empty static shell. Frontend Task 8 stores the correct query token in `sessionStorage`, removes it from history, and survives refresh in the same tab.

- [ ] **Step 3c: Implement request and response boundaries**

The implementation must use:

- `secrets.token_urlsafe(32)` for both startup session token and validation IDs;
- `hmac.compare_digest()` for session-token comparison;
- `json.loads(raw.decode("utf-8"), object_pairs_hook=reject_duplicate_keys)`;
- a route table with exact path/method matching rather than substring checks;
- `X-Content-Type-Options: nosniff`, `Cache-Control: no-store`, `Referrer-Policy: no-referrer`, `X-Frame-Options: DENY`, and `Content-Security-Policy: default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; object-src 'none'; frame-ancestors 'none'; base-uri 'none'`;
- `ThreadingHTTPServer`, with immutable asset data shared read-only and MuJoCo guarded by the validator lock;
- `--host` accepts only `127.0.0.1` in v1;
- fixed defaults for the approved URDF, asset YAML, MJCF-derived config, and output directory;
- startup fail-fast for missing/unreadable assets, but an asset signature mismatch starts read-only and prints that validation/save are disabled.

Override `log_message()` to omit the query string and token-bearing headers. Static HTML/JS/CSS contain no server data and are safe as a public loopback shell; `/api/*`, `/assets/*`, and saved-pose reads require `X-Hitter-Editor-Token`.

- [ ] **Step 4: Run and verify GREEN**

Run the Step 2 command. Expected: all HTTP tests pass and every test server is shut down in `tearDownClass`.

- [ ] **Step 5: Commit only Task 5 files**

```bash
git add tools/hitter_ready_pose_editor/server.py \
  tools/hitter_ready_pose_editor/tests/test_server.py
/home/loco1/miniconda3/envs/rb/bin/python -c \
  'import subprocess,sys; a=set(subprocess.check_output(["git","diff","--cached","--name-only"],text=True).splitlines()); e=set(sys.argv[1:]); assert a==e,(sorted(e),sorted(a))' \
  tools/hitter_ready_pose_editor/server.py \
  tools/hitter_ready_pose_editor/tests/test_server.py
git diff --cached --check
git commit -m "feat: serve authenticated ready pose API"
```

---

## Task 6: Implement independent browser matrix, FK, and STL modules

**Files:**

- Create: `tools/hitter_ready_pose_editor/package.json`
- Create: `tools/hitter_ready_pose_editor/web/matrix.js`
- Create: `tools/hitter_ready_pose_editor/web/fk.js`
- Create: `tools/hitter_ready_pose_editor/web/stl.js`
- Create: `tools/hitter_ready_pose_editor/web/tests/matrix.test.mjs`
- Create: `tools/hitter_ready_pose_editor/web/tests/fk.test.mjs`
- Create: `tools/hitter_ready_pose_editor/web/tests/stl.test.mjs`

**Interfaces:** Consumes only the Task 2 public-manifest JSON schema and ArrayBuffer STL bytes. Produces column-major link matrices and the canonical `robot_base_default` browser FK summary; no Python or MuJoCo data is imported.

- [ ] **Step 1: Write failing pure-JavaScript tests**

`package.json` contains only:

```json
{
  "name": "hitter-ready-pose-editor",
  "private": true,
  "type": "module",
  "scripts": {
    "test": "npm run test:unit",
    "test:unit": "node --test $(find web/tests -name '*.test.mjs' ! -name 'live_fk.test.mjs' -print)",
    "test:live": "node --test web/tests/live_fk.test.mjs"
  }
}
```

Test matrix multiplication, arbitrary-axis rotation, RPY order, quaternion conversion and a two-joint chain:

```javascript
test("FK applies parent, joint origin, axis rotation, then child", () => {
  const model = {
    rootLink: "base",
    activeJointNames: ["joint_a"],
    joints: [
      {
        name: "joint_a",
        type: "revolute",
        parentLink: "base",
        childLink: "arm",
        originXyzM: [1, 0, 0],
        originRpyRad: [0, 0, 0],
        axisXyz: [0, 0, 1],
        hardLimitRad: [-Math.PI, Math.PI],
        defaultRad: 0,
        robot29Index: 0
      },
      {
        name: "tool_fixed",
        type: "fixed",
        parentLink: "arm",
        childLink: "tool",
        originXyzM: [0.25, 0, 0],
        originRpyRad: [0, 0, 0],
        axisXyz: null,
        hardLimitRad: null,
        defaultRad: null,
        robot29Index: null
      }
    ]
  };
  const result = computeForwardKinematics(model, {joint_a: Math.PI / 2});
  assertVectorClose(transformPoint(result.linkWorldMatrices.arm, [1, 0, 0]), [1, 1, 0]);
  assertVectorClose(transformPoint(result.linkWorldMatrices.tool, [0, 0, 0]), [1, 0.25, 0]);
});

test("FK rejects missing, unknown, nonfinite, and out-of-range values", () => {
  assert.throws(() => computeForwardKinematics(model, {}), /missing joint_a/);
  assert.throws(() => computeForwardKinematics(model, {joint_a: 0, other: 0}), /unknown other/);
  assert.throws(() => computeForwardKinematics(model, {joint_a: NaN}), /finite/);
  assert.throws(() => computeForwardKinematics(model, {joint_a: 4}), /hard limit/);
});
```

Add exact STL tests: the checked-in one-triangle ASCII fixture shorter than 84 bytes returns 9 finite position floats and bounds `[0,0,0]→[1,1,0]` without reading a binary count; a binary two-triangle square assembled with `DataView` returns 18 floats; headers containing `solid` still use binary detection when byte length matches; truncated binary, zero triangles, NaN coordinates, and ASCII without a complete `facet` each throw `StlParseError`. Test pure `validateStlLimits()` at exactly `67,108,864` bytes / `1,000,000` triangles and at each limit plus one, avoiding giant test allocations.

- [ ] **Step 2: Run and verify RED**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/tools/hitter_ready_pose_editor
node --test web/tests/matrix.test.mjs web/tests/fk.test.mjs web/tests/stl.test.mjs
```

Expected: module-not-found failures for `matrix.js`, `fk.js`, and `stl.js`.

- [ ] **Step 3a: Implement the matrix primitives**

`matrix.js` exports `identity4`, `multiply4`, `translation4`, `rotationRpy4`, `rotationAxisAngle4`, `transformPoint`, `matrixToPoseXyzw`, and `inverseRigid4`.

Use `Float64Array(16)`, column-major indices `column * 4 + row`, column vectors, and `T(xyz) * Rz(yaw) * Ry(pitch) * Rx(roll)` for URDF RPY. `rotationAxisAngle4` normalizes a finite nonzero axis and applies Rodrigues’ formula. `matrixToPoseXyzw` normalizes the quaternion and flips all four values when `w < 0`. Tests assert identity, non-commuting multiplication, 90° rotations about X/Y/Z and `[1,1,1]`, RPY order, rigid inverse, and `q/-q` canonicalization.

- [ ] **Step 3b: Implement full active/fixed-joint FK**

```javascript
export function computeForwardKinematics(model, jointPositionsRad) {
  const expected = new Set(model.activeJointNames);
  for (const name of Object.keys(jointPositionsRad)) {
    if (!expected.has(name)) throw new Error(`unknown ${name}`);
  }
  for (const name of expected) {
    if (!Object.hasOwn(jointPositionsRad, name)) {
      throw new Error(`missing ${name}`);
    }
  }
  const linkWorldMatrices = {[model.rootLink]: identity4()};
  for (const joint of model.joints) {
    const parent = linkWorldMatrices[joint.parentLink];
    if (!parent) throw new Error(`unresolved parent ${joint.parentLink}`);
    const origin = multiply4(
      translation4(joint.originXyzM),
      rotationRpy4(joint.originRpyRad)
    );
    let local = origin;
    if (joint.type === "revolute") {
      const q = jointPositionsRad[joint.name];
      if (!Number.isFinite(q)) throw new Error(`${joint.name} must be finite`);
      const [lower, upper] = joint.hardLimitRad;
      if (q < lower || q > upper) throw new Error(`${joint.name} outside hard limit`);
      local = multiply4(origin, rotationAxisAngle4(joint.axisXyz, q));
    } else if (joint.type !== "fixed") {
      throw new Error(`unsupported joint type ${joint.type}`);
    }
    linkWorldMatrices[joint.childLink] = multiply4(parent, local);
  }
  return {linkWorldMatrices};
}

export function extractFkSummary(result, linkNames) {
  const links = {};
  for (const name of linkNames) {
    const pose = matrixToPoseXyzw(result.linkWorldMatrices[name]);
    links[name] = {
      position_m: pose.position,
      quaternion_xyzw: pose.quaternion
    };
  }
  return {
    frame: "robot_base_default",
    quaternion_convention: "xyzw",
    links
  };
}
```

- [ ] **Step 3c: Implement bounded ASCII/binary STL parsing**

`stl.js` exports `StlParseError`, pure `validateStlLimits(byteLength, triangleCountOrNull, limits)`, `parseStl(arrayBuffer, limits)`, `computeMeshBounds(positions)`, and `buildVertexNormals(positions)`. `limits` comes from `PublicManifestV1.limits` and must equal `maxMeshBytes=67108864` and `maxStlTriangles=1000000`. Reject zero length or bytes above the cap first. Only construct/read the binary `DataView` triangle-count field when `byteLength >= 84`; reject count above the cap before allocation. Treat input as binary only when `84 + triangleCount * 50 === byteLength`. If a binary-looking buffer contains NUL/non-UTF-8 bytes but is shorter than that computed length, raise `StlParseError("truncated binary STL")`; otherwise strict-decode ASCII and parse complete `facet`/`vertex` groups. Reject non-finite values and return `{positions: Float32Array, normals: Float32Array, bounds}`.

`jointPositionsRad` must contain all 29 active names and no unknown names; fixed joints consume no value. No frontend module may import generated MuJoCo values or Python math. This keeps browser FK independently testable.

- [ ] **Step 4: Run and verify GREEN**

Run the Step 2 command. Expected: all pure-JS tests pass with Node 18 and no npm install.

- [ ] **Step 5: Commit only Task 6 files**

```bash
git add tools/hitter_ready_pose_editor/package.json \
  tools/hitter_ready_pose_editor/web/matrix.js \
  tools/hitter_ready_pose_editor/web/fk.js \
  tools/hitter_ready_pose_editor/web/stl.js \
  tools/hitter_ready_pose_editor/web/tests/matrix.test.mjs \
  tools/hitter_ready_pose_editor/web/tests/fk.test.mjs \
  tools/hitter_ready_pose_editor/web/tests/stl.test.mjs
/home/loco1/miniconda3/envs/rb/bin/python -c \
  'import subprocess,sys; a=set(subprocess.check_output(["git","diff","--cached","--name-only"],text=True).splitlines()); e=set(sys.argv[1:]); assert a==e,(sorted(e),sorted(a))' \
  tools/hitter_ready_pose_editor/package.json \
  tools/hitter_ready_pose_editor/web/matrix.js \
  tools/hitter_ready_pose_editor/web/fk.js \
  tools/hitter_ready_pose_editor/web/stl.js \
  tools/hitter_ready_pose_editor/web/tests/matrix.test.mjs \
  tools/hitter_ready_pose_editor/web/tests/fk.test.mjs \
  tools/hitter_ready_pose_editor/web/tests/stl.test.mjs
git diff --cached --check
git commit -m "feat: add browser HITTER kinematics core"
```

---

## Task 7: Render the complete live robot, racket, table, grid, and ghost pose

**Files:**

- Create: `tools/hitter_ready_pose_editor/web/index.html`
- Create: `tools/hitter_ready_pose_editor/web/styles.css`
- Create: `tools/hitter_ready_pose_editor/web/renderer.js`
- Extend: `tools/hitter_ready_pose_editor/web/tests/matrix.test.mjs`

**Interfaces:** Consumes public manifest, authenticated mesh loader, and link matrices from Task 6. Produces pixels and camera interaction only; it does not own or mutate pose state.

- [ ] **Step 1: Add failing camera/bounds tests before WebGL code**

Add exact tests: two transformed unit boxes union to the expected six-number bounds; `computeFitCamera([-1,-2,-3,4,5,6], Math.PI/4, 16/9)` returns a finite positive distance whose projected eight corners all lie inside NDC; preset directions are front `[1,0,0]`, back `[-1,0,0]`, left `[0,1,0]`, right `[0,-1,0]` with up `[0,0,1]`; zero extent receives a finite minimum radius; every NaN/Inf bound throws.

```javascript
test("camera presets use the robot_base_default axes", () => {
  assert.deepEqual(cameraPreset("front").direction, [1, 0, 0]);
  assert.deepEqual(cameraPreset("back").direction, [-1, 0, 0]);
  assert.deepEqual(cameraPreset("left").direction, [0, 1, 0]);
  assert.deepEqual(cameraPreset("right").direction, [0, -1, 0]);
  assert.deepEqual(cameraPreset("front").up, [0, 0, 1]);
});

test("fit camera contains every transformed bound corner", () => {
  const camera = computeFitCamera([-1, -2, -3, 4, 5, 6], Math.PI / 4, 16 / 9);
  assert.ok(Number.isFinite(camera.distance) && camera.distance > 0);
  for (const corner of boundsCorners([-1, -2, -3, 4, 5, 6])) {
    const ndc = projectToNdc(camera, corner);
    assert.ok(Math.abs(ndc[0]) <= 1 && Math.abs(ndc[1]) <= 1);
  }
});
```

Expected initial failure: `computeFitCamera` and `cameraPreset` are not exported.

- [ ] **Step 2: Implement testable camera helpers, then run GREEN**

Put pure camera helpers in `matrix.js`, run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/tools/hitter_ready_pose_editor
node --test web/tests/matrix.test.mjs
```

Expected: the added bounds/camera tests pass.

- [ ] **Step 3: Build the exact three-column page structure**

`index.html` must contain:

- left panel `#left-arm-panel`;
- center `#robot-canvas`, view preset buttons, ghost toggle, table toggle, reset-all button, status/error region;
- right panel `#right-arm-panel`;
- save button labeled `检查并保存独立姿态`;
- hidden confirmation dialog used in Task 9;
- no inline third-party script, CDN, remote font, analytics, iframe, or external image.

`styles.css` must keep both 7-joint panels visible on a 1440-pixel-wide desktop, provide a stacked fallback below 1000 pixels, make warnings yellow and blocking errors red, and keep the center canvas at least 640×640 CSS pixels when space allows.

- [ ] **Step 4a: Implement renderer construction and immutable mesh buffers**

Export class `HitterRenderer` with methods `constructor(canvas, statusCallback)`, `load(manifest, meshLoader)`, `setPose(linkWorldMatrices)`, `setGhostPose(linkWorldMatricesOrNull)`, `setTableVisible(visible)`, `setCameraPreset(name)`, `fitToRobot()`, `render()`, and `dispose()`.

`load()` calls `meshLoader(manifest.assetHashes.displayMeshSetSha256, meshId)` once per unique ID, parses with `parseStl`, uploads immutable position/normal buffers, and reuses them for every visual. It compiles local vertex/fragment shaders with explicit attribute/uniform lookup failure checks. A missing WebGL context calls the supplied status callback with `当前浏览器无法创建 WebGL 上下文` but does not disable pose controls.

- [ ] **Step 4b: Implement robot, scale-correct normals, table, grid, and ghost passes**

For every visual:

```text
M_visual = W_link
         * T(originXyzM)
         * R(originRpyRad)
         * S(meshScaleXyz)
N_visual = transpose(inverse(mat3(M_visual)))
```

Use the same scaled transform for camera bounds. The fixture includes a nonzero visual origin and non-unit/non-uniform mesh scale, and a pure test asserts transformed bounds and inverse-transpose normal behavior.

Convert FK’s `Float64Array` matrices to `Float32Array` immediately before WebGL uniform upload. Render current robot opaque first. Render default ghost second with alpha blending and depth writes disabled, then restore depth state. Render `tableVisuals` as `robot_base_default` oriented boxes using their current MJCF-derived centers/quaternions/half-sizes. Render the ground grid exactly at `manifest.frame.groundZRobotBaseM` (currently approximately `-0.793 m`). No table or ground number is hard-coded in JavaScript.

- [ ] **Step 4c: Implement camera controls and lifecycle**

Add mouse orbit, wheel zoom with bounded distance, double-click fit, and four preset views. Recalculate canvas backing size as CSS size times `min(devicePixelRatio, 2)`. `dispose()` removes every registered DOM listener and deletes every created buffer, shader, program, and animation frame. Add a repeated create/dispose smoke test with a fake WebGL object to assert no listener/resource count grows.

- [ ] **Step 5: Run static and JS checks**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/tools/hitter_ready_pose_editor
node --check web/renderer.js
node --test web/tests/matrix.test.mjs
if grep -RInE "https?://|cdn|unpkg|jsdelivr" web; then
  exit 1
fi
```

Expected: syntax/tests pass; the URL scan returns no matches.

- [ ] **Step 6: Commit only Task 7 files**

```bash
git add tools/hitter_ready_pose_editor/web/index.html \
  tools/hitter_ready_pose_editor/web/styles.css \
  tools/hitter_ready_pose_editor/web/renderer.js \
  tools/hitter_ready_pose_editor/web/matrix.js \
  tools/hitter_ready_pose_editor/web/tests/matrix.test.mjs
/home/loco1/miniconda3/envs/rb/bin/python -c \
  'import subprocess,sys; a=set(subprocess.check_output(["git","diff","--cached","--name-only"],text=True).splitlines()); e=set(sys.argv[1:]); assert a==e,(sorted(e),sorted(a))' \
  tools/hitter_ready_pose_editor/web/index.html \
  tools/hitter_ready_pose_editor/web/styles.css \
  tools/hitter_ready_pose_editor/web/renderer.js \
  tools/hitter_ready_pose_editor/web/matrix.js \
  tools/hitter_ready_pose_editor/web/tests/matrix.test.mjs
git diff --cached --check
git commit -m "feat: render complete HITTER robot in browser"
```

---

## Task 8: Add deterministic pose state and the two seven-joint control panels

**Files:**

- Create: `tools/hitter_ready_pose_editor/web/pose_state.js`
- Create: `tools/hitter_ready_pose_editor/web/app.js`
- Create: `tools/hitter_ready_pose_editor/web/tests/pose_state.test.mjs`
- Modify: `tools/hitter_ready_pose_editor/web/index.html`
- Modify: `tools/hitter_ready_pose_editor/web/styles.css`

**Interfaces:** Consumes the public manifest and Task 6 FK. Produces the sole editable 14-name radian state, a full 29-name pose for browser FK, and the exact 14-name validation payload; DOM inputs are views, not state.

- [ ] **Step 1: Write failing state tests**

```javascript
test("slider is soft-limited while numeric input can enter hard-only range", () => {
  const state = createPoseState(manifest);
  assert.deepEqual(state.control("left_elbow_joint").sliderRangeRad, softRange);
  state.setDegrees("left_elbow_joint", hardOnlyDegrees);
  assert.equal(state.warningFor("left_elbow_joint").kind, "soft-limit");
  assert.throws(() => state.setDegrees("left_elbow_joint", beyondHardDegrees), /hard limit/);
});

test("left reset never changes right arm or lower body", () => {
  const state = createPoseState(manifest);
  state.setDegrees("left_elbow_joint", 70);
  state.setDegrees("right_elbow_joint", 80);
  const rightBefore = state.radians("right_elbow_joint");
  state.resetArm("left");
  assert.equal(state.radians("right_elbow_joint"), rightBefore);
  assert.deepEqual(state.composeRobot29().slice(0, 15), manifest.defaultJointPosRad.slice(0, 15));
});

test("invalid saved pose is all-or-nothing", () => {
  const state = createPoseState(manifest);
  const before = state.armPose();
  const invalid = {...before};
  delete invalid.left_elbow_joint;
  invalid.unknown_joint = 0;
  assert.throws(() => state.loadJointPose(invalid), /joint names mismatch/);
  assert.deepEqual(state.armPose(), before);
});
```

- [ ] **Step 1a: Add the exact state edge-case matrix**

| Test | Required result |
|---|---|
| `test_degree_radian_round_trip` | `-180,-45,0,45,180` round-trip within `1e-12` |
| `test_asymmetric_shoulder_roll_uses_manifest_ranges` | left/right slider bounds equal their distinct `softLimitRad` values |
| `test_dirty_and_reset_all` | first accepted change sets dirty; reset-all restores all 14 defaults and clears dirty |
| `test_load_is_all_or_nothing` | missing/extra/non-finite/out-of-hard saved pose throws and changes no joint |
| `test_full_29_name_map_preserves_lower_body` | first 15 values/names exactly equal manifest defaults |
| `test_validation_payload_has_exact_contract` | only `joint_pos_by_name`, `browser_fk`, `asset_signature_sha256` keys |
| `test_beforeunload_eligibility` | true only when dirty and not after a matching successful save |

- [ ] **Step 2: Run and verify RED**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/tools/hitter_ready_pose_editor
node --test web/tests/pose_state.test.mjs
```

Expected: module-not-found failure for `pose_state.js`.

- [ ] **Step 3a: Implement conversions and immutable input validation**

```javascript
export const degreesToRadians = value => value * Math.PI / 180;
export const radiansToDegrees = value => value * 180 / Math.PI;

export function createPoseState(manifest) {
  const specs = new Map(manifest.joints.map(spec => [spec.name, spec]));
  const initial = Object.fromEntries(
    manifest.armJointNames.map(name => [name, specs.get(name).defaultRad])
  );
  let current = {...initial};
  let saved = {...initial};

  function checkedValue(name, value) {
    if (!manifest.armJointNames.includes(name)) {
      throw new Error(`unknown ${name}`);
    }
    if (!Number.isFinite(value)) throw new Error(`${name} must be finite`);
    const [lower, upper] = specs.get(name).hardLimitRad;
    if (value < lower || value > upper) {
      throw new Error(`${name} outside hard limit`);
    }
    return value;
  }

  return {
    setDegrees(name, degrees) {
      current = {...current, [name]: checkedValue(name, degreesToRadians(degrees))};
    },
    setSliderRadians(name, radians) {
      const [lower, upper] = specs.get(name).softLimitRad;
      if (radians < lower || radians > upper) throw new Error(`${name} outside slider range`);
      current = {...current, [name]: checkedValue(name, radians)};
    },
    radians(name) {
      return current[name];
    },
    armPose() {
      return Object.fromEntries(manifest.armJointNames.map(name => [name, current[name]]));
    },
    jointPosByName29() {
      return Object.fromEntries(manifest.activeJointNames.map((name, index) => [
        name,
        current[name] ?? manifest.defaultJointPosRad[index]
      ]));
    },
    composeRobot29() {
      const byName = this.jointPosByName29();
      return manifest.activeJointNames.map(name => byName[name]);
    },
    isDirty() {
      return manifest.armJointNames.some(name => current[name] !== saved[name]);
    }
  };
}
```

- [ ] **Step 3b: Add warning, reset, atomic load, payload, and save acknowledgment**

Extend the returned object with:

- `control(name)` returning hard/soft/default/current radians from the manifest;
- `warningFor(name)` returning `{kind:"soft-limit", message}` only outside soft and inside hard;
- `resetArm("left"|"right")` and `resetAll()` using `initial`;
- `loadJointPose(jointPosByName)`, which validates an exact 14-name temporary copy before assigning it;
- `validationPayload(browserFk)` returning exactly `{joint_pos_by_name: armPose(), browser_fk: browserFk, asset_signature_sha256: manifest.assetHashes.assetSignatureSha256}`;
- `markSaved(validatedJointPosByName)`, which updates `saved` only if the acknowledged 14-name values equal current state.

The state layer is the only owner of editable radians. DOM elements render state; they do not become a second source of truth.

- [ ] **Step 4: Render controls dynamically from the manifest**

In `app.js`:

- bootstrap by reading `token` from the initial URL; if present, store it in `sessionStorage["hitter-ready-editor-token"]` and remove it with `history.replaceState`; on refresh, reuse the session value; if neither exists, render `请使用服务启动时打印的带 token 链接打开页面` and make no API call;
- fetch `/api/model`;
- create exactly seven controls in the fixed shoulder→elbow→wrist order on each side;
- set slider min/max from per-joint soft range and numeric min/max from hard limit, while displaying both in degrees;
- build the full 29-name map, recompute browser FK, extract `FkSummaryV1`, and call `renderer.setPose()` on every accepted change;
- preserve the last accepted value and show a Chinese inline error for blank, non-numeric, non-finite, or hard-limit input;
- wire left/right/all reset, table, ghost, view presets and `beforeunload`;
- keep save integration disabled until Task 9, but render the save button disabled with reason text.

- [ ] **Step 5: Run and verify GREEN**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/tools/hitter_ready_pose_editor
node --check web/app.js
node --test web/tests/pose_state.test.mjs web/tests/fk.test.mjs
```

Expected: syntax and state/FK tests pass.

- [ ] **Step 6: Commit only Task 8 files**

```bash
git add tools/hitter_ready_pose_editor/web/pose_state.js \
  tools/hitter_ready_pose_editor/web/app.js \
  tools/hitter_ready_pose_editor/web/tests/pose_state.test.mjs \
  tools/hitter_ready_pose_editor/web/index.html \
  tools/hitter_ready_pose_editor/web/styles.css
/home/loco1/miniconda3/envs/rb/bin/python -c \
  'import subprocess,sys; a=set(subprocess.check_output(["git","diff","--cached","--name-only"],text=True).splitlines()); e=set(sys.argv[1:]); assert a==e,(sorted(e),sorted(a))' \
  tools/hitter_ready_pose_editor/web/pose_state.js \
  tools/hitter_ready_pose_editor/web/app.js \
  tools/hitter_ready_pose_editor/web/tests/pose_state.test.mjs \
  tools/hitter_ready_pose_editor/web/index.html \
  tools/hitter_ready_pose_editor/web/styles.css
git diff --cached --check
git commit -m "feat: edit HITTER arm pose in browser"
```

---

## Task 9: Connect validation, two-stage confirmation, save, list, and reload

**Files:**

- Create: `tools/hitter_ready_pose_editor/web/api.js`
- Create: `tools/hitter_ready_pose_editor/web/tests/api.test.mjs`
- Modify: `tools/hitter_ready_pose_editor/web/app.js`
- Modify: `tools/hitter_ready_pose_editor/web/index.html`
- Modify: `tools/hitter_ready_pose_editor/web/styles.css`

**Interfaces:** Consumes the Task 5 API and Task 8 pose snapshot. Produces two-stage validation/save interaction, soft-limit confirmation, success-path display, pose-ID list/reload, and never a filename/path chosen by the browser.

- [ ] **Step 1: Write failing API-client tests with a fake fetch**

```javascript
test("save sends only validation ticket and explicit soft confirmation", async () => {
  const calls = [];
  const api = createEditorApi("secret", async (url, options) => {
    calls.push({url, options});
    return jsonResponse(201, {json_path: "/fixed/a.json", yaml_path: "/fixed/a.yaml"});
  });
  await api.save("ticket-1", true);
  assert.equal(calls[0].url, "/api/save");
  assert.deepEqual(JSON.parse(calls[0].options.body), {
    validation_id: "ticket-1",
    confirm_soft_limit: true
  });
  assert.equal(calls[0].options.headers["X-Hitter-Editor-Token"], "secret");
});

test("non-2xx response preserves structured Chinese error", async () => {
  const api = createEditorApi("secret", async () =>
    jsonResponse(409, {error: "资产签名不一致", code: "asset_mismatch"})
  );
  await assert.rejects(api.model(), /资产签名不一致/);
});
```

- [ ] **Step 1a: Add the exact client test matrix**

| Test | Required result |
|---|---|
| `test_model_health_and_poses_are_gets` | exact URLs, no body, auth header present |
| `test_mesh_uses_hash_and_opaque_id` | exact encoded `/assets/{hash}/{id}`, returns ArrayBuffer |
| `test_validate_sends_exact_snapshot` | body keys exactly match Task 5 request schema |
| `test_pose_uses_only_listed_pose_id` | client rejects slash/dot ID before fetch |
| `test_timeout_aborts` | fake timer reaches 15 s, fetch signal is aborted, Chinese timeout error |
| `test_invalid_json_and_network_failure` | both become `EditorApiError` without losing status/code |
| `test_no_request_can_send_path` | recursive request-body scan finds no `path`, `filename`, or `output_dir` key |

- [ ] **Step 2: Run and verify RED**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/tools/hitter_ready_pose_editor
node --test web/tests/api.test.mjs
```

Expected: module-not-found failure for `api.js`.

- [ ] **Step 3a: Implement one bounded authenticated JSON request primitive**

```javascript
export class EditorApiError extends Error {
  constructor(message, status, code) {
    super(message);
    this.status = status;
    this.code = code;
  }
}

export function createEditorApi(sessionToken, fetchImpl = fetch) {
  async function requestJson(url, {method = "GET", body} = {}) {
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 15000);
    try {
      const response = await fetchImpl(url, {
        method,
        cache: "no-store",
        signal: controller.signal,
        headers: {
          "X-Hitter-Editor-Token": sessionToken,
          ...(body === undefined ? {} : {"Content-Type": "application/json"})
        },
        ...(body === undefined ? {} : {body: JSON.stringify(body)})
      });
      let payload;
      try {
        payload = await response.json();
      } catch (parseError) {
        throw new EditorApiError(
          "服务器返回无效 JSON",
          response.status,
          "invalid_response"
        );
      }
      if (!response.ok) {
        throw new EditorApiError(
          payload.error || "服务器请求失败",
          response.status,
          payload.code || "request_failed"
        );
      }
      return payload;
    } catch (error) {
      if (error.name === "AbortError") {
        throw new EditorApiError("请求超时，请检查 SSH 隧道", 0, "timeout");
      }
      if (error instanceof EditorApiError) throw error;
      throw new EditorApiError("无法连接姿态编辑服务", 0, "network");
    } finally {
      clearTimeout(timeout);
    }
  }
```

- [ ] **Step 3b: Implement exact endpoint methods**

First add a complete binary-request sibling inside `createEditorApi`:

```javascript
async function requestArrayBuffer(url) {
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 15000);
  try {
    const response = await fetchImpl(url, {
      cache: "no-store",
      signal: controller.signal,
      headers: {"X-Hitter-Editor-Token": sessionToken}
    });
    if (!response.ok) {
      let payload = {};
      try {
        payload = await response.json();
      } catch (parseError) {
        payload = {};
      }
      throw new EditorApiError(
        payload.error || "网格请求失败",
        response.status,
        payload.code || "mesh_failed"
      );
    }
    return await response.arrayBuffer();
  } catch (error) {
    if (error.name === "AbortError") {
      throw new EditorApiError("网格请求超时", 0, "timeout");
    }
    if (error instanceof EditorApiError) throw error;
    throw new EditorApiError("无法读取机器人网格", 0, "network");
  } finally {
    clearTimeout(timeout);
  }
}
```

Then return these methods:

```javascript
return {
  health: () => requestJson("/api/health"),
  model: () => requestJson("/api/model"),
  poses: () => requestJson("/api/poses"),
  pose: poseId => {
    if (!/^hitter_ready_arm_pose_\d{8}_\d{6}(?:_\d{3})?$/.test(poseId)) {
      throw new EditorApiError("非法姿态 ID", 0, "invalid_pose_id");
    }
    return requestJson(`/api/poses/${encodeURIComponent(poseId)}`);
  },
  validate: payload => requestJson("/api/validate", {
    method: "POST",
    body: payload
  }),
  save: (validationId, confirmSoftLimit) => requestJson("/api/save", {
    method: "POST",
    body: {
      validation_id: validationId,
      confirm_soft_limit: confirmSoftLimit
    }
  }),
  mesh: (meshSetSha256, meshId) => {
    const sha256 = /^[0-9a-f]{64}$/;
    if (!sha256.test(meshSetSha256) || !sha256.test(meshId)) {
      throw new EditorApiError("非法网格标识", 0, "invalid_mesh_id");
    }
    return requestArrayBuffer(`/assets/${meshSetSha256}/${meshId}`);
  }
};
}
```

Every API/mesh call sends the header, uses `cache: "no-store"`, enforces the same timeout, and converts structured server errors to `EditorApiError`.

- [ ] **Step 4: Implement exact save UX**

On `检查并保存独立姿态`:

1. create an immutable deep copy of `pose_state.armPose()`, the full 29-name map, and browser FK;
2. send name-keyed 14-D radians and browser FK for both wrists/racket to `/api/validate`;
3. display 14 radians, 29-D mapping identifier, three MuJoCo `robot_base_default` link poses, asset hashes, maximum position/orientation errors, and every soft-limit warning;
4. if warnings exist, require a dedicated unchecked checkbox labeled `我确认该姿态超出 90% soft range，但仍在 hard limit 内`;
5. enable final `确认保存独立 YAML + JSON` only after validation succeeds and required checkbox is checked;
6. immediately before final save, compare the current 14-name state with the validated snapshot; if any value changed, invalidate the dialog locally and require validation again; otherwise send only `validation_id` and `confirm_soft_limit`;
7. show both returned absolute paths, display any `storage_warning` without offering a retry, and refresh the saved-pose list;
8. mark clean only if the currently displayed pose still equals the validated snapshot;
9. on any error, close no editor state, preserve all controls, show the server’s Chinese error, and allow retry.

The saved-pose list displays creation time and opaque `pose_id`, loads only through `/api/poses/{pose_id}`, checks schema/hash server-side, compares the record asset signature with the displayed manifest, and updates the browser state only after the entire record passes.

- [ ] **Step 5: Run and verify GREEN**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/tools/hitter_ready_pose_editor
node --check web/api.js
node --check web/app.js
node --test web/tests/api.test.mjs web/tests/pose_state.test.mjs web/tests/fk.test.mjs
```

Expected: syntax and all listed tests pass.

- [ ] **Step 6: Commit only Task 9 files**

```bash
git add tools/hitter_ready_pose_editor/web/api.js \
  tools/hitter_ready_pose_editor/web/tests/api.test.mjs \
  tools/hitter_ready_pose_editor/web/app.js \
  tools/hitter_ready_pose_editor/web/index.html \
  tools/hitter_ready_pose_editor/web/styles.css
/home/loco1/miniconda3/envs/rb/bin/python -c \
  'import subprocess,sys; a=set(subprocess.check_output(["git","diff","--cached","--name-only"],text=True).splitlines()); e=set(sys.argv[1:]); assert a==e,(sorted(e),sorted(a))' \
  tools/hitter_ready_pose_editor/web/api.js \
  tools/hitter_ready_pose_editor/web/tests/api.test.mjs \
  tools/hitter_ready_pose_editor/web/app.js \
  tools/hitter_ready_pose_editor/web/index.html \
  tools/hitter_ready_pose_editor/web/styles.css
git diff --cached --check
git commit -m "feat: validate and save ready poses from UI"
```

---

## Task 10: Prove live browser/MuJoCo parity and document isolated operation

**Files:**

- Create: `tools/hitter_ready_pose_editor/scripts/generate_live_fk_contract.py`
- Create: `tools/hitter_ready_pose_editor/web/tests/live_fk.test.mjs`
- Create: `tools/hitter_ready_pose_editor/README.md`
- Modify only if a test exposes a defect: files created in Tasks 1–9

**Interfaces:** Consumes the complete production manifest/validator/browser FK stack. Produces a temporary deterministic 129-pose `robot_base_default` parity contract, final operator documentation, focused-test evidence, and no runtime configuration change.

- [ ] **Step 1: Write the failing live parity test**

`generate_live_fk_contract.py` must emit one temporary JSON document containing:

- the exact public browser manifest built from current URDF/STL;
- the current URDF, mesh set, MJCF, asset YAML, both kinematic hashes, and combined asset signature;
- default pose;
- for each arm joint, one pose at `soft_min + 1e-6` and one at `soft_max - 1e-6`, with every other joint at default;
- 100 poses from `numpy.random.default_rng(20260726)`, sampling every arm joint uniformly from `[hard_min + 1e-5, hard_max - 1e-5]`, with the lower-body/waist values at defaults;
- `comparisonLinkNames` containing `pelvis`, the 29 active joints’ child links, and `right_racket_link`;
- MuJoCo `robot_base_default` poses for every comparison link.

`live_fk.test.mjs` must fail, not skip, when `HITTER_LIVE_FK_CONTRACT` is absent. Its central loop is:

```javascript
const contractPath = process.env.HITTER_LIVE_FK_CONTRACT;
assert.ok(contractPath, "HITTER_LIVE_FK_CONTRACT is required");
const contract = JSON.parse(await readFile(contractPath, "utf8"));
assert.equal(contract.poses.length, 129);

let maxPositionErrorM = 0;
let maxOrientationErrorRad = 0;
for (const sample of contract.poses) {
  const browser = computeForwardKinematics(
    contract.manifest,
    sample.jointPosByName29
  );
  for (const linkName of contract.comparisonLinkNames) {
    const actual = matrixToPoseXyzw(browser.linkWorldMatrices[linkName]);
    const expected = sample.mujocoRobotBaseDefault[linkName];
    const positionError = Math.hypot(
      actual.position[0] - expected.position_m[0],
      actual.position[1] - expected.position_m[1],
      actual.position[2] - expected.position_m[2]
    );
    const dot = Math.abs(
      actual.quaternion[0] * expected.quaternion_xyzw[0] +
      actual.quaternion[1] * expected.quaternion_xyzw[1] +
      actual.quaternion[2] * expected.quaternion_xyzw[2] +
      actual.quaternion[3] * expected.quaternion_xyzw[3]
    );
    const orientationError = 2 * Math.acos(Math.min(1, dot));
    maxPositionErrorM = Math.max(maxPositionErrorM, positionError);
    maxOrientationErrorRad = Math.max(
      maxOrientationErrorRad, orientationError
    );
  }
}
assert.ok(maxPositionErrorM <= 0.0005, `${maxPositionErrorM}`);
assert.ok(maxOrientationErrorRad <= Math.PI / 1800, `${maxOrientationErrorRad}`);
```

It also asserts exact 29-joint order, exact 14-arm order, 37-joint topological tree, `robot_base_default`, `xyzw`, radians, and matching asset signature. Fixed branches absent as independent MJCF bodies remain covered by Task 2 fixture tests.

- [ ] **Step 2: Run the parity test and verify RED**

```bash
bash -c '
set -euo pipefail
cd /home/loco1/BOB/Hitter/RobotBridge2
contract_file="$(mktemp /tmp/hitter-ready-fk.XXXXXX)"
trap "rm -f \"$contract_file\"" EXIT
env -u DISPLAY \
  PYTHONDONTWRITEBYTECODE=1 \
  MUJOCO_GL=egl \
  PYTHONPATH=deploy:. \
  /home/loco1/miniconda3/envs/rb/bin/python \
  tools/hitter_ready_pose_editor/scripts/generate_live_fk_contract.py \
  --output "$contract_file"
HITTER_LIVE_FK_CONTRACT="$contract_file" \
  node --test tools/hitter_ready_pose_editor/web/tests/live_fk.test.mjs
'
```

Expected before the generator/test are implemented: missing-file or module failure.

- [ ] **Step 3: Implement the generator and make live parity GREEN**

The generator must call the production `HitterAssetModel` and the same locked MuJoCo path as `PoseValidator`, never duplicate expected FK math in Python. Use the actual MuJoCo `pelvis` pose from each forward call to transform every comparison body into `robot_base_default`; do not subtract configured root position as a shortcut. Write only the explicit `--output` temporary file, never the repository or pose directory.

Repeat the Step 2 command. Expected: all default, 28 near-boundary, and 100 seeded-random poses pass `0.5 mm / 0.1°`.

- [ ] **Step 4: Write the operator README**

Document these exact commands.

On `loco1`:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2
env -u DISPLAY \
  PYTHONDONTWRITEBYTECODE=1 \
  MUJOCO_GL=egl \
  PYTHONPATH=deploy:. \
  /home/loco1/miniconda3/envs/rb/bin/python \
  -m tools.hitter_ready_pose_editor.server \
  --host 127.0.0.1 \
  --port 8765
```

On the Mac:

```bash
ssh -J baai-omega-sunyu1997 \
  -N \
  -L 8765:127.0.0.1:8765 \
  g1-hostB-loco1
```

Then open the tokenized URL printed by the `loco1` server, for example:

```text
http://127.0.0.1:8765/?token=<server-printed-session-token>
```

The README must also state:

- the two authoritative live asset paths;
- the authoritative `g1_hitter_racket.yaml` path;
- the fixed output directory;
- YAML/JSON are candidates only and do not affect `waiting_arm_return` until a later reviewed integration;
- how to stop only the editor with `Ctrl-C`;
- the editor has no LCM/DDS/Vicon/robot control imports;
- the UI-to-radians, 14→29, and motion NPZ mappings;
- the soft/hard-limit semantics and FK thresholds;
- v1 does not prove self-collision, table collision, torque, transition smoothness, or true-robot safety; a candidate cannot be deployed without later MuJoCo dynamics and controlled real-robot review;
- how to run Python, Node, live parity, and full baseline tests.

- [ ] **Step 5: Run every focused test**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2
env -u DISPLAY \
  PYTHONDONTWRITEBYTECODE=1 \
  MUJOCO_GL=egl \
  PYTHONPATH=deploy:. \
  /home/loco1/miniconda3/envs/rb/bin/python \
  -m unittest discover \
  -s tools/hitter_ready_pose_editor/tests \
  -p 'test_*.py' \
  -v

cd /home/loco1/BOB/Hitter/RobotBridge2/tools/hitter_ready_pose_editor
npm run test:unit

bash -c '
set -euo pipefail
cd /home/loco1/BOB/Hitter/RobotBridge2
contract_file="$(mktemp /tmp/hitter-ready-fk.XXXXXX)"
trap "rm -f \"$contract_file\"" EXIT
env -u DISPLAY \
  PYTHONDONTWRITEBYTECODE=1 \
  MUJOCO_GL=egl \
  PYTHONPATH=deploy:. \
  /home/loco1/miniconda3/envs/rb/bin/python \
  tools/hitter_ready_pose_editor/scripts/generate_live_fk_contract.py \
  --output "$contract_file"
HITTER_LIVE_FK_CONTRACT="$contract_file" \
  npm --prefix tools/hitter_ready_pose_editor run test:live
'
```

Expected: all new Python and Node tests pass.

- [ ] **Step 6: Re-run the existing deploy baseline and classify the known failure**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
env -u DISPLAY \
  PYTHONDONTWRITEBYTECODE=1 \
  MUJOCO_GL=egl \
  /home/loco1/miniconda3/envs/rb/bin/python \
  -m unittest discover \
  -s tests \
  -p 'test_*.py' \
  -v
```

Current pre-implementation baseline is 31 tests: 30 pass, with one unrelated existing failure:

```text
test_xml_defines_only_intended_ball_contact_pairs
```

The test expects `0.20 0.005 0.0001`, while the current user-owned MJCF at `deploy/data/assets/g1/g1_29dof_hitter_racket_table_tennis.xml:319` contains `0.20 0.20 0.005 0.0001 0.0001`. Do not edit that MJCF or test as part of this feature. Final acceptance requires no additional deploy-test failures.

- [ ] **Step 7: Run a temporary HTTP and browser smoke test**

Start the editor only, using the README command. Verify:

1. startup prints the tokenized loopback URL and reports current asset hashes;
2. `/api/health` is healthy through the tunnel;
3. the full robot, both wrists, and right racket render;
4. each of the 14 controls moves only the expected chain;
5. front/back/left/right, orbit, zoom, double-click fit, ghost, and table toggle work;
6. hard-limit input is rejected without changing the pose;
7. a soft-only pose requires the extra checkbox;
8. the confirmation summary and final-save button become available, but do not create a production candidate before the user has chosen a real pose;
9. list/reload behavior is already exercised end-to-end against a temporary `PoseStore` by `test_server.py`;
10. `git status --short` shows no changed asset/config/model/generated-pose file and no RobotBridge process was started/stopped.

Stop only the temporary editor with `Ctrl-C`. The first file in the fixed production pose directory must come from the user’s deliberately selected posture, not an implementation smoke test.

- [ ] **Step 8: Scan for placeholders and unsafe capabilities**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2
if grep -RInE "TODO|TBD|FIXME|NotImplemented|pass$" tools/hitter_ready_pose_editor; then
  exit 1
fi
grep -RInE "pd_plustau_targets|LCM|DDS|Vicon|RealWorld|hitter\\.onnx|save_origin|0\\.0\\.0\\.0" \
  tools/hitter_ready_pose_editor || true
git diff --check
```

Expected:

- placeholder scan has no matches;
- unsafe-capability scan matches only explanatory README/tests where explicitly asserted absent, never a runtime import or endpoint;
- no whitespace errors.

- [ ] **Step 9: Commit only Task 10 files and any test-driven fixes**

```bash
git add tools/hitter_ready_pose_editor/scripts/generate_live_fk_contract.py \
  tools/hitter_ready_pose_editor/web/tests/live_fk.test.mjs \
  tools/hitter_ready_pose_editor/README.md
/home/loco1/miniconda3/envs/rb/bin/python -c \
  'import subprocess,sys; a=set(subprocess.check_output(["git","diff","--cached","--name-only"],text=True).splitlines()); e=set(sys.argv[1:]); assert a==e,(sorted(e),sorted(a))' \
  tools/hitter_ready_pose_editor/scripts/generate_live_fk_contract.py \
  tools/hitter_ready_pose_editor/web/tests/live_fk.test.mjs \
  tools/hitter_ready_pose_editor/README.md
git diff --cached --check
git diff --cached --name-only
git commit -m "test: verify HITTER ready pose editor end to end"
```

If Tasks 1–9 files required test-driven fixes, list every such path explicitly in both `git add` and the inline Python staged-allowlist command; never use `git add -A` or `git add .`.

---

## Final Review Checklist

- [ ] Compare every section of the confirmed design against Tasks 1–10 and record the matching test or manual check.
- [ ] Confirm exact types and conventions across Python and JavaScript: radians, `xyzw`, right-handed coordinates, column-major matrices, name-keyed 14-D source of truth, full 29-D export.
- [ ] Confirm no placeholder, stale static robot snapshot, CDN, arbitrary-path API, URDF writer, active-config writer, robot-control import, or public bind exists.
- [ ] Confirm server tickets prevent validation/save payload drift and asset-hash drift.
- [ ] Confirm all new tests pass, live 129-pose × 31-link FK parity passes, and the existing deploy suite has no new failures beyond the documented baseline failure.
- [ ] Confirm the final worktree diff contains only intended editor/dependency files; preserve every pre-existing user change.
- [ ] Report the exact launch command, tunnel command, test counts, live asset hashes, and known unrelated baseline failure; report saved paths only after the user deliberately saves a posture.

## Execution Choice

After this plan is committed, do not begin implementation until the user chooses one:

1. **Subagent-Driven (recommended):** use `superpowers:subagent-driven-development`, execute one task at a time with a fresh worker and review gate after every task.
2. **Inline Execution:** use `superpowers:executing-plans`, execute this plan in the current thread with explicit checkpoints.
