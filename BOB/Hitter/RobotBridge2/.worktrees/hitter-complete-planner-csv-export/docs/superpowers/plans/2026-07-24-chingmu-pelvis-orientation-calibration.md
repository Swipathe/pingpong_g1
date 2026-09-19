# ChingMu Pelvis 旋转外参标定 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为 ChingMu `G1Pelvis` 增加一次性、可持久化的三维旋转外参标定，使真机发布的 pelvis 朝向和 `base_forward_xy` 与 MuJoCo 一致，同时位置直接使用 ChingMu 刚体原点。

**Architecture:** 桌面标定继续负责把 ChingMu 房间坐标转换到 RobotBridge 桌面世界系；新增的 `PelvisOrientationCalibration` 只描述 MuJoCo pelvis 系相对 ChingMu 刚体系的固定旋转。标定姿态下 pelvis 三轴与桌面世界系对齐，通过 SO(3) 多帧平均求 `R_rigid_from_pelvis`；运行时统一使用 `R_world_pelvis = R_world_rigid @ R_rigid_from_pelvis`，位置保持 `p_world_rigid`。

**Tech Stack:** Python 3、NumPy、SciPy `Rotation`、标准库 `argparse/json/tempfile/unittest`、RobotBridge2 LCM 类型、conda 环境 `rb`。

## Global Constraints

- 实现必须符合已批准设计：`docs/superpowers/specs/2026-07-24-chingmu-pelvis-orientation-calibration-design.md`。
- 唯一允许的旋转公式是 `R_world_pelvis = R_world_rigid @ R_rigid_from_pelvis`。
- 唯一允许的位置公式是 `p_publish = p_world_rigid`。
- 删除 `BridgeConfig.pelvis_offset_heading_m`，不得再使用 `[0.003145, 0.044074, 0.048231]`。
- 不重新引入启动首帧 yaw、相对 yaw 或 yaw-only 四元数。
- 四元数顺序保持 `xyzw`；写文件前归一化并选择 `w >= 0`。
- 标定门限固定为：至少 30 个有效样本、source-time 覆盖至少 `1.0 s`、位置 RMS 不超过 `0.002 m`、旋转测地 RMS 不超过 `0.3 deg`。
- 标定文件缺失、格式错误、subject 不匹配或质量不合格时必须失败，不得使用单位旋转兜底。
- 不修改 `real_world.py`、`hitter.py`、MuJoCo、planner、policy、训练代码、球追踪和 LCM schema。
- 不恢复当前工作树中已删除的旧测试套件；只新建本计划指定的聚焦测试文件。
- 当前 `chingmu_table_lcm_bridge.py` 已有未提交的“移除首帧 yaw、发布完整绝对旋转”相关改动。Task 1 必须先审查该文件当前 diff，并将这些与本功能直接相关的既有改动和 Task 1 新改动一起纳入第一笔代码提交。
- 工作树包含大量无关修改和删除。每次提交前必须执行 `git diff --cached --name-status`；暂存区只允许出现本任务明确列出的文件。禁止 `git add .` 和 `git add -A`。
- 真实 `chingmu_pelvis_orientation_latest.json` 必须现场摆正机器人后生成；实现阶段不得创建或提交单位四元数占位文件。

终审批准的增补约束（2026-07-24）：

- 四种 calibration 路径角色必须按 `Path.resolve()` 后的路径比较；任意两个
  不同角色不得共用同一文件。`--table-calib` 与
  `--save-table-calib` 即使指向不同文件也必须明确拒绝。
- 所有 CLI 路径/模式/数值拒绝必须发生在 SDK 构造前。
  `--calib-sec` 和 `--pelvis-calib-sec` 只在对应采集路径要求 finite 且
  `> 0`；runtime `--duration` 要求 finite 且 `>= 0`；
  `--source-rate-hz` 与 `--body-pose-poll-hz` 始终要求 finite 且
  `> 0`。
- pelvis 标定只计入带 finite、严格递增且唯一
  `body_pose_source_time_s` 的 fresh 样本；该时间与外层 frame 时间差
  不得超过 `0.1 s`，`1.0 s` coverage 必须由 pose 时间计算。
- 终审允许修改 `chingmu_sdk_client.py` 的窄范围例外：只为 callback
  root report 和 `CMTrackerExternTC` polling pose 传播其自身 SDK
  timestamp，并保留 polling cache 冷启动的一次同步 fallback；不得把
  标定数学下沉到 SDK client。

---

## File Map

- Modify: `deploy/mocap_bridge/chingmu_table_lcm_bridge.py`
  - 定义旋转标定数据结构、质量门限、标定数学、JSON 保存/加载、运行时组合和 CLI 模式。
- Modify: `deploy/mocap_bridge/chingmu_sdk_client.py`
  - 终审批准的窄范围例外：在兼容旧 `MocapFrame` 构造的前提下，传播
    callback/polling 刚体姿态自身的 SDK timestamp，并保留后台 polling
    与 cold-cache 同步 fallback。
- Create: `deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py`
  - 除原有数学、持久化、运行时和 CLI 回归外，覆盖路径角色、finite
    参数、pose freshness/timestamp 传播、稳定四元数归一化和非单位
    table basis；不恢复其他已删除测试。
- Modify: `docs/superpowers/specs/2026-07-24-chingmu-pelvis-orientation-calibration-design.md`
  - 保留已批准的需求和验收标准，并追加终审确认的 quality-sample
    freshness 定义与 SDK timestamp 窄范围例外。
- Runtime-generated, do not create offline:
  `deploy/mocap_bridge/calibrations/chingmu_pelvis_orientation_latest.json`

---

### Task 1: 建立旋转标定数据模型和 SO(3) 求解

**Files:**
- Modify: `deploy/mocap_bridge/chingmu_table_lcm_bridge.py:25-53,325-363`
- Create: `deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py`

**Interfaces:**
- Consumes:
  - `body_pose_to_table_world(position_mm, quaternion_xyzw, table, config)`
  - `MocapFrame.body_position_mm`
  - `MocapFrame.body_quaternion_xyzw`
  - `MocapFrame.source_time_s`
- Produces:
  - `PelvisOrientationCalibration`
  - `calibrate_pelvis_orientation_from_frames(frames, table, config) -> PelvisOrientationCalibration`
  - `_validate_pelvis_orientation_calibration(calibration) -> None`

- [ ] **Step 1: 记录并审查目标文件的现有相关 diff**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2
git diff -- deploy/mocap_bridge/chingmu_table_lcm_bridge.py
```

Expected: diff 只包含当前已知的 ChingMu root 方向链路改动，包括删除 `initial_base_yaw`、由 yaw-only 改为完整绝对四元数，以及现有 pelvis offset 的旋转方式。若出现与本功能无关的新增改动，停止并先隔离该改动。

- [ ] **Step 2: 新建测试夹具和成功标定的失败测试**

Create `deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py` with:

```python
from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from deploy.mocap_bridge.chingmu_sdk_client import MocapFrame
from deploy.mocap_bridge.chingmu_table_lcm_bridge import (
    BridgeConfig,
    PelvisOrientationCalibration,
    TableFrame,
    calibrate_pelvis_orientation_from_frames,
)


def identity_table() -> TableFrame:
    return TableFrame(
        center_raw_mm=np.zeros(3, dtype=np.float64),
        x_axis_raw=np.array([1.0, 0.0, 0.0], dtype=np.float64),
        y_axis_raw=np.array([0.0, 1.0, 0.0], dtype=np.float64),
        z_axis_raw=np.array([0.0, 0.0, 1.0], dtype=np.float64),
        corners_raw_mm=np.zeros((4, 3), dtype=np.float64),
        rectangle_score=0.0,
    )


def mocap_frame(
    index: int,
    rotation_world: Rotation,
    *,
    position_mm=(100.0, 200.0, 300.0),
    source_time_s: float | None = None,
) -> MocapFrame:
    quaternion = rotation_world.as_quat()
    if index % 2:
        quaternion = -quaternion
    return MocapFrame(
        frame_number=1000 + index,
        source_time_s=index / 40.0 if source_time_s is None else source_time_s,
        body_position_mm=np.asarray(position_mm, dtype=np.float64),
        body_quaternion_xyzw=quaternion,
        body_markers_mm={},
        unlabeled_markers_mm=np.empty((0, 3), dtype=np.float64),
    )


def stable_frames(
    rigid_world: Rotation,
    *,
    count: int = 61,
) -> list[MocapFrame]:
    return [mocap_frame(index, rigid_world) for index in range(count)]


def valid_orientation_calibration(
    rotation_rigid_from_pelvis: Rotation | None = None,
) -> PelvisOrientationCalibration:
    rotation = (
        Rotation.identity()
        if rotation_rigid_from_pelvis is None
        else rotation_rigid_from_pelvis
    )
    return PelvisOrientationCalibration(
        rotation_rigid_from_pelvis=rotation.as_matrix(),
        sample_count=61,
        source_duration_s=1.5,
        position_rms_m=0.0002,
        angular_rms_deg=0.05,
    )


class PelvisOrientationCalibrationMathTest(unittest.TestCase):
    def setUp(self):
        self.config = BridgeConfig()
        self.table = identity_table()

    def test_recovers_rigid_from_pelvis_rotation_from_aligned_pose(self):
        expected = Rotation.from_euler("xyz", [0.12, -0.08, 0.35])
        rigid_world_in_aligned_pose = expected.inv()

        calibration = calibrate_pelvis_orientation_from_frames(
            stable_frames(rigid_world_in_aligned_pose),
            self.table,
            self.config,
        )

        recovered = Rotation.from_matrix(
            calibration.rotation_rigid_from_pelvis
        )
        self.assertLess((recovered.inv() * expected).magnitude(), 1.0e-10)
        self.assertEqual(calibration.sample_count, 61)
        self.assertAlmostEqual(calibration.source_duration_s, 1.5)
        self.assertAlmostEqual(calibration.position_rms_m, 0.0)
        self.assertAlmostEqual(calibration.angular_rms_deg, 0.0)

    def test_quaternion_sign_changes_do_not_change_rotation_mean(self):
        expected = Rotation.from_euler("xyz", [-0.2, 0.1, -0.4])

        calibration = calibrate_pelvis_orientation_from_frames(
            stable_frames(expected.inv()),
            self.table,
            self.config,
        )

        recovered = Rotation.from_matrix(
            calibration.rotation_rigid_from_pelvis
        )
        self.assertLess((recovered.inv() * expected).magnitude(), 1.0e-10)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 3: 运行成功标定测试并确认 RED**

Run:

```bash
conda run --no-capture-output -n rb python -m unittest \
  deploy.mocap_bridge.tests.test_chingmu_pelvis_orientation.PelvisOrientationCalibrationMathTest \
  -v
```

Expected: import error，指出 `PelvisOrientationCalibration` 或 `calibrate_pelvis_orientation_from_frames` 尚不存在。

- [ ] **Step 4: 增加不稳定和样本不足的失败测试**

Append to `PelvisOrientationCalibrationMathTest`:

```python
    def test_rejects_insufficient_sample_count(self):
        with self.assertRaisesRegex(ValueError, "at least 30 valid"):
            calibrate_pelvis_orientation_from_frames(
                stable_frames(Rotation.identity(), count=29),
                self.table,
                self.config,
            )

    def test_rejects_insufficient_source_time_coverage(self):
        frames = [
            mocap_frame(
                index,
                Rotation.identity(),
                source_time_s=index / 100.0,
            )
            for index in range(61)
        ]

        with self.assertRaisesRegex(ValueError, "source-time coverage"):
            calibrate_pelvis_orientation_from_frames(
                frames,
                self.table,
                self.config,
            )

    def test_rejects_position_motion_above_two_millimetres_rms(self):
        frames = [
            mocap_frame(
                index,
                Rotation.identity(),
                position_mm=(100.0 + 10.0 * np.sin(index), 200.0, 300.0),
            )
            for index in range(61)
        ]

        with self.assertRaisesRegex(ValueError, "position RMS"):
            calibrate_pelvis_orientation_from_frames(
                frames,
                self.table,
                self.config,
            )

    def test_rejects_angular_motion_above_point_three_degrees_rms(self):
        frames = [
            mocap_frame(
                index,
                Rotation.from_euler(
                    "z",
                    np.linspace(-1.0, 1.0, 61)[index],
                    degrees=True,
                ),
            )
            for index in range(61)
        ]

        with self.assertRaisesRegex(ValueError, "angular RMS"):
            calibrate_pelvis_orientation_from_frames(
                frames,
                self.table,
                self.config,
            )
```

- [ ] **Step 5: 在 bridge 中实现数据结构、门限和求解**

Add near the existing calibration dataclasses:

```python
PELVIS_ORIENTATION_FORMAT = "robotbridge2_chingmu_pelvis_orientation_v1"
PELVIS_ROTATION_CONVENTION = (
    "R_world_pelvis = R_world_rigid @ R_rigid_from_pelvis"
)
MIN_PELVIS_CALIBRATION_SAMPLES = 30
MIN_PELVIS_CALIBRATION_DURATION_S = 1.0
MAX_PELVIS_CALIBRATION_POSITION_RMS_M = 0.002
MAX_PELVIS_CALIBRATION_ANGULAR_RMS_DEG = 0.3


@dataclass(frozen=True)
class PelvisOrientationCalibration:
    rotation_rigid_from_pelvis: np.ndarray
    sample_count: int
    source_duration_s: float
    position_rms_m: float
    angular_rms_deg: float
```

Add after `quaternion_xyzw_from_rotation()`:

```python
def _validate_pelvis_orientation_calibration(
    calibration: PelvisOrientationCalibration,
) -> None:
    matrix = np.asarray(
        calibration.rotation_rigid_from_pelvis,
        dtype=np.float64,
    )
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError(
            "pelvis orientation rotation must be a finite 3x3 matrix"
        )
    if not np.allclose(matrix.T @ matrix, np.eye(3), atol=1.0e-8):
        raise ValueError("pelvis orientation rotation must be orthogonal")
    if not np.isclose(np.linalg.det(matrix), 1.0, atol=1.0e-8):
        raise ValueError(
            "pelvis orientation rotation determinant must be +1"
        )
    if (
        not isinstance(calibration.sample_count, (int, np.integer))
        or isinstance(calibration.sample_count, bool)
        or calibration.sample_count < MIN_PELVIS_CALIBRATION_SAMPLES
    ):
        raise ValueError(
            "pelvis orientation calibration needs at least "
            f"{MIN_PELVIS_CALIBRATION_SAMPLES} valid samples"
        )
    if (
        not np.isfinite(calibration.source_duration_s)
        or calibration.source_duration_s
        < MIN_PELVIS_CALIBRATION_DURATION_S
    ):
        raise ValueError(
            "pelvis orientation source-time coverage must be at least "
            f"{MIN_PELVIS_CALIBRATION_DURATION_S:.1f} s"
        )
    if (
        not np.isfinite(calibration.position_rms_m)
        or calibration.position_rms_m < 0.0
        or calibration.position_rms_m
        > MAX_PELVIS_CALIBRATION_POSITION_RMS_M
    ):
        raise ValueError(
            "pelvis orientation position RMS "
            f"{calibration.position_rms_m!r} m exceeds "
            f"{MAX_PELVIS_CALIBRATION_POSITION_RMS_M:.3f} m"
        )
    if (
        not np.isfinite(calibration.angular_rms_deg)
        or calibration.angular_rms_deg < 0.0
        or calibration.angular_rms_deg
        > MAX_PELVIS_CALIBRATION_ANGULAR_RMS_DEG
    ):
        raise ValueError(
            "pelvis orientation angular RMS "
            f"{calibration.angular_rms_deg!r} deg exceeds "
            f"{MAX_PELVIS_CALIBRATION_ANGULAR_RMS_DEG:.1f} deg"
        )


def calibrate_pelvis_orientation_from_frames(
    frames: Iterable,
    table: TableFrame,
    config: BridgeConfig,
) -> PelvisOrientationCalibration:
    positions_world = []
    rotation_matrices_world = []
    source_times_s = []
    for frame in frames:
        pose = body_pose_to_table_world(
            frame.body_position_mm,
            frame.body_quaternion_xyzw,
            table,
            config,
        )
        try:
            source_time_s = float(frame.source_time_s)
        except (TypeError, ValueError):
            continue
        if pose is None or not np.isfinite(source_time_s):
            continue
        position_world, rotation_world = pose
        positions_world.append(
            np.asarray(position_world, dtype=np.float64).reshape(3)
        )
        rotation_matrices_world.append(
            np.asarray(rotation_world, dtype=np.float64).reshape(3, 3)
        )
        source_times_s.append(source_time_s)

    sample_count = len(positions_world)
    if sample_count < MIN_PELVIS_CALIBRATION_SAMPLES:
        raise ValueError(
            "pelvis orientation calibration needs at least "
            f"{MIN_PELVIS_CALIBRATION_SAMPLES} valid samples; "
            f"got {sample_count}"
        )

    source_duration_s = float(
        max(source_times_s) - min(source_times_s)
    )
    positions = np.stack(positions_world)
    position_mean = positions.mean(axis=0)
    position_rms_m = float(
        np.sqrt(
            np.mean(
                np.sum((positions - position_mean) ** 2, axis=1)
            )
        )
    )

    rotations_world = Rotation.from_matrix(
        np.stack(rotation_matrices_world)
    )
    mean_rotation_world_rigid = rotations_world.mean()
    angular_errors_rad = (
        mean_rotation_world_rigid.inv() * rotations_world
    ).magnitude()
    angular_rms_deg = float(
        np.degrees(np.sqrt(np.mean(angular_errors_rad ** 2)))
    )

    calibration = PelvisOrientationCalibration(
        rotation_rigid_from_pelvis=(
            mean_rotation_world_rigid.inv().as_matrix()
        ),
        sample_count=sample_count,
        source_duration_s=source_duration_s,
        position_rms_m=position_rms_m,
        angular_rms_deg=angular_rms_deg,
    )
    _validate_pelvis_orientation_calibration(calibration)
    return calibration
```

- [ ] **Step 6: 运行 Task 1 测试并确认 GREEN**

Run:

```bash
conda run --no-capture-output -n rb python -m unittest \
  deploy.mocap_bridge.tests.test_chingmu_pelvis_orientation.PelvisOrientationCalibrationMathTest \
  -v
```

Expected: 6 tests pass。

- [ ] **Step 7: 检查语法和 diff**

Run:

```bash
conda run --no-capture-output -n rb python -m py_compile \
  deploy/mocap_bridge/chingmu_table_lcm_bridge.py \
  deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py
git diff --check -- \
  deploy/mocap_bridge/chingmu_table_lcm_bridge.py \
  deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py
git diff -- \
  deploy/mocap_bridge/chingmu_table_lcm_bridge.py \
  deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py
```

Expected: compilation succeeds, `git diff --check` has no output, diff contains only the current ChingMu absolute-orientation baseline plus Task 1 calibration math/tests。

- [ ] **Step 8: 显式暂存并提交 Task 1**

Run:

```bash
git add -p -- deploy/mocap_bridge/chingmu_table_lcm_bridge.py
git add -- deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py
git diff --cached --name-status
git diff --cached --check
git commit -m "feat: add ChingMu pelvis orientation calibration math"
```

Expected before commit: staged paths are exactly:

```text
M  deploy/mocap_bridge/chingmu_table_lcm_bridge.py
A  deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py
```

Do not stage any deletion or any other dirty file。

---

### Task 2: 增加严格且原子化的 JSON 保存与加载

**Files:**
- Modify: `deploy/mocap_bridge/chingmu_table_lcm_bridge.py`
- Modify: `deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py`

**Interfaces:**
- Consumes:
  - `PelvisOrientationCalibration`
  - `_validate_pelvis_orientation_calibration(calibration)`
  - `quaternion_xyzw_from_rotation(rotation)`
- Produces:
  - `save_pelvis_orientation_calibration(path, calibration, config) -> None`
  - `load_pelvis_orientation_calibration(path, config) -> PelvisOrientationCalibration`

- [ ] **Step 1: 写 JSON round-trip 和四元数规范化失败测试**

Extend imports from `chingmu_table_lcm_bridge`:

```python
    load_pelvis_orientation_calibration,
    save_pelvis_orientation_calibration,
```

Append:

```python
class PelvisOrientationCalibrationJsonTest(unittest.TestCase):
    def setUp(self):
        self.config = BridgeConfig()

    def test_json_round_trip_preserves_rotation_and_quality(self):
        expected = valid_orientation_calibration(
            Rotation.from_euler("xyz", [0.2, -0.1, 0.4])
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "pelvis_orientation.json"

            save_pelvis_orientation_calibration(
                path,
                expected,
                self.config,
            )
            loaded = load_pelvis_orientation_calibration(
                path,
                self.config,
            )
            payload = json.loads(path.read_text(encoding="utf-8"))

        np.testing.assert_allclose(
            loaded.rotation_rigid_from_pelvis,
            expected.rotation_rigid_from_pelvis,
            atol=1.0e-12,
        )
        self.assertEqual(loaded.sample_count, expected.sample_count)
        self.assertEqual(
            payload["rotation_convention"],
            "R_world_pelvis = R_world_rigid @ R_rigid_from_pelvis",
        )
        self.assertGreaterEqual(
            payload["quaternion_rigid_from_pelvis_xyzw"][3],
            0.0,
        )
        self.assertAlmostEqual(
            np.linalg.norm(
                payload["quaternion_rigid_from_pelvis_xyzw"]
            ),
            1.0,
        )
```

- [ ] **Step 2: 写损坏文件和失败保存不覆盖旧文件的测试**

Append to `PelvisOrientationCalibrationJsonTest`:

```python
    def test_rejects_invalid_json_fields(self):
        valid_payload = {
            "format": "robotbridge2_chingmu_pelvis_orientation_v1",
            "base_subject": "G1Pelvis",
            "quaternion_convention": "xyzw",
            "rotation_convention": (
                "R_world_pelvis = "
                "R_world_rigid @ R_rigid_from_pelvis"
            ),
            "quaternion_rigid_from_pelvis_xyzw": [0.0, 0.0, 0.0, 1.0],
            "sample_count": 61,
            "source_duration_s": 1.5,
            "position_rms_m": 0.0002,
            "angular_rms_deg": 0.05,
        }
        mutations = {
            "format": {"format": "wrong"},
            "subject": {"base_subject": "OtherBody"},
            "convention": {"rotation_convention": "wrong"},
            "quaternion": {
                "quaternion_rigid_from_pelvis_xyzw": [0.0, 0.0, 0.0, 2.0]
            },
            "quality": {"angular_rms_deg": 0.31},
        }

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "pelvis_orientation.json"
            for label, mutation in mutations.items():
                with self.subTest(label=label):
                    payload = dict(valid_payload)
                    payload.update(mutation)
                    path.write_text(
                        json.dumps(payload),
                        encoding="utf-8",
                    )
                    with self.assertRaises(ValueError):
                        load_pelvis_orientation_calibration(
                            path,
                            self.config,
                        )

            missing_field_payload = dict(valid_payload)
            del missing_field_payload["sample_count"]
            path.write_text(
                json.dumps(missing_field_payload),
                encoding="utf-8",
            )
            with self.assertRaises(ValueError):
                load_pelvis_orientation_calibration(
                    path,
                    self.config,
                )

            path.write_text("{", encoding="utf-8")
            with self.assertRaises(ValueError):
                load_pelvis_orientation_calibration(
                    path,
                    self.config,
                )

    def test_missing_json_file_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "missing.json"
            with self.assertRaisesRegex(
                ValueError,
                "failed to read pelvis orientation calibration",
            ):
                load_pelvis_orientation_calibration(
                    path,
                    self.config,
                )

    def test_failed_save_does_not_overwrite_existing_file(self):
        invalid = PelvisOrientationCalibration(
            rotation_rigid_from_pelvis=np.eye(3),
            sample_count=61,
            source_duration_s=1.5,
            position_rms_m=0.5,
            angular_rms_deg=0.05,
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "pelvis_orientation.json"
            path.write_text("keep-existing\n", encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "position RMS"):
                save_pelvis_orientation_calibration(
                    path,
                    invalid,
                    self.config,
                )

            self.assertEqual(
                path.read_text(encoding="utf-8"),
                "keep-existing\n",
            )
```

- [ ] **Step 3: 运行 JSON 测试并确认 RED**

Run:

```bash
conda run --no-capture-output -n rb python -m unittest \
  deploy.mocap_bridge.tests.test_chingmu_pelvis_orientation.PelvisOrientationCalibrationJsonTest \
  -v
```

Expected: import error，指出保存和加载函数尚不存在。

- [ ] **Step 4: 实现原子保存**

Add imports:

```python
import os
import tempfile
```

Add near the existing table calibration persistence functions:

```python
def save_pelvis_orientation_calibration(
    path,
    calibration: PelvisOrientationCalibration,
    config: BridgeConfig,
) -> None:
    _validate_pelvis_orientation_calibration(calibration)
    quaternion = quaternion_xyzw_from_rotation(
        calibration.rotation_rigid_from_pelvis
    )
    payload = {
        "format": PELVIS_ORIENTATION_FORMAT,
        "base_subject": config.base_subject,
        "quaternion_convention": "xyzw",
        "rotation_convention": PELVIS_ROTATION_CONVENTION,
        "quaternion_rigid_from_pelvis_xyzw": quaternion.tolist(),
        "sample_count": int(calibration.sample_count),
        "source_duration_s": float(calibration.source_duration_s),
        "position_rms_m": float(calibration.position_rms_m),
        "angular_rms_deg": float(calibration.angular_rms_deg),
    }

    calibration_path = Path(path)
    calibration_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=calibration_path.parent,
            prefix=f".{calibration_path.name}.",
            suffix=".tmp",
            delete=False,
        ) as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
            temporary_path = Path(handle.name)
        temporary_path.replace(calibration_path)
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)
```

- [ ] **Step 5: 实现严格加载**

Add:

```python
def load_pelvis_orientation_calibration(
    path,
    config: BridgeConfig,
) -> PelvisOrientationCalibration:
    calibration_path = Path(path)
    try:
        payload = json.loads(
            calibration_path.read_text(encoding="utf-8")
        )
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(
            f"failed to read pelvis orientation calibration "
            f"{calibration_path}: {exc}"
        ) from exc

    if payload.get("format") != PELVIS_ORIENTATION_FORMAT:
        raise ValueError(
            f"unsupported pelvis orientation calibration format in "
            f"{calibration_path}"
        )
    if payload.get("base_subject") != config.base_subject:
        raise ValueError(
            f"pelvis orientation calibration base_subject="
            f"{payload.get('base_subject')!r} does not match "
            f"{config.base_subject!r}"
        )
    if payload.get("quaternion_convention") != "xyzw":
        raise ValueError(
            "pelvis orientation quaternion_convention must be 'xyzw'"
        )
    if (
        payload.get("rotation_convention")
        != PELVIS_ROTATION_CONVENTION
    ):
        raise ValueError(
            "pelvis orientation rotation_convention does not match "
            f"{PELVIS_ROTATION_CONVENTION!r}"
        )

    try:
        quaternion = np.asarray(
            payload["quaternion_rigid_from_pelvis_xyzw"],
            dtype=np.float64,
        ).reshape(4)
        sample_count = payload["sample_count"]
        source_duration_s = float(payload["source_duration_s"])
        position_rms_m = float(payload["position_rms_m"])
        angular_rms_deg = float(payload["angular_rms_deg"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(
            f"invalid pelvis orientation fields in {calibration_path}: "
            f"{exc}"
        ) from exc

    quaternion_norm = float(np.linalg.norm(quaternion))
    if (
        not np.isfinite(quaternion).all()
        or not np.isclose(quaternion_norm, 1.0, atol=1.0e-6)
    ):
        raise ValueError(
            "pelvis orientation quaternion must be finite and unit length"
        )
    if (
        not isinstance(sample_count, int)
        or isinstance(sample_count, bool)
    ):
        raise ValueError(
            "pelvis orientation sample_count must be an integer"
        )

    calibration = PelvisOrientationCalibration(
        rotation_rigid_from_pelvis=(
            Rotation.from_quat(quaternion).as_matrix()
        ),
        sample_count=sample_count,
        source_duration_s=source_duration_s,
        position_rms_m=position_rms_m,
        angular_rms_deg=angular_rms_deg,
    )
    _validate_pelvis_orientation_calibration(calibration)
    return calibration
```

- [ ] **Step 6: 运行 Task 1 和 Task 2 测试并确认 GREEN**

Run:

```bash
conda run --no-capture-output -n rb python -m unittest \
  deploy.mocap_bridge.tests.test_chingmu_pelvis_orientation.PelvisOrientationCalibrationMathTest \
  deploy.mocap_bridge.tests.test_chingmu_pelvis_orientation.PelvisOrientationCalibrationJsonTest \
  -v
```

Expected: 10 tests pass。

- [ ] **Step 7: 检查并提交 Task 2**

Run:

```bash
git diff --check -- \
  deploy/mocap_bridge/chingmu_table_lcm_bridge.py \
  deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py
git add -p -- deploy/mocap_bridge/chingmu_table_lcm_bridge.py
git add -p -- deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py
git diff --cached --name-status
git diff --cached --check
git commit -m "feat: persist ChingMu pelvis orientation calibration"
```

Expected: only the bridge and focused test file are staged。

---

### Task 3: 接入正常运行链路并删除位置 offset

**Files:**
- Modify: `deploy/mocap_bridge/chingmu_table_lcm_bridge.py:25-37,178-236,464-500,628-690`
- Modify: `deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py`

**Interfaces:**
- Consumes:
  - `load_pelvis_orientation_calibration(path, config)`
  - `PelvisOrientationCalibration.rotation_rigid_from_pelvis`
- Produces:
  - `ChingMuTableLcmBridge(..., pelvis_orientation_calibration=...)`
  - CLI `--pelvis-orientation-calib PATH`
  - runtime publication with rigid-origin position and corrected absolute pelvis rotation

- [ ] **Step 1: 写非交换旋转和刚体原点位置的失败测试**

Extend bridge imports:

```python
    ChingMuTableLcmBridge,
    raw_to_table_world,
```

Append:

```python
class PelvisOrientationRuntimeTest(unittest.TestCase):
    def setUp(self):
        self.config = BridgeConfig()
        self.table = identity_table()

    def base_message(
        self,
        bridge: ChingMuTableLcmBridge,
        frame: MocapFrame,
    ):
        return next(
            message
            for message in bridge.process_frame(
                frame,
                publish_time_us=123,
            )
            if message.name == self.config.base_subject
        )

    def test_position_is_rigid_origin_and_rotation_is_right_multiplied(self):
        rigid_world = Rotation.from_euler("xyz", [0.31, -0.22, 0.47])
        rigid_from_pelvis = Rotation.from_euler(
            "xyz",
            [-0.18, 0.26, -0.33],
        )
        bridge = ChingMuTableLcmBridge(
            config=self.config,
            table_frame=self.table,
            pelvis_orientation_calibration=(
                valid_orientation_calibration(rigid_from_pelvis)
            ),
        )
        frame = mocap_frame(0, rigid_world)

        base = self.base_message(bridge, frame)

        np.testing.assert_allclose(
            base.pos_vicon,
            raw_to_table_world(
                frame.body_position_mm,
                self.table,
                self.config,
            ),
            atol=1.0e-12,
        )
        published = Rotation.from_quat(base.quat_vicon)
        expected = rigid_world * rigid_from_pelvis
        self.assertLess(
            (published.inv() * expected).magnitude(),
            1.0e-10,
        )

    def test_first_frame_keeps_corrected_absolute_yaw(self):
        correction = Rotation.from_euler("z", -0.4)
        bridge = ChingMuTableLcmBridge(
            config=self.config,
            table_frame=self.table,
            pelvis_orientation_calibration=(
                valid_orientation_calibration(correction)
            ),
        )

        first = self.base_message(
            bridge,
            mocap_frame(0, Rotation.from_euler("z", 0.75)),
        )
        second = self.base_message(
            bridge,
            mocap_frame(1, Rotation.from_euler("z", 0.90)),
        )

        first_yaw = Rotation.from_quat(first.quat_vicon).as_euler("xyz")[2]
        second_yaw = Rotation.from_quat(second.quat_vicon).as_euler("xyz")[2]
        self.assertAlmostEqual(first_yaw, 0.35, places=10)
        self.assertAlmostEqual(second_yaw, 0.50, places=10)

    def test_calibrated_left_quarter_turn_points_forward_at_world_y(self):
        correction = Rotation.from_euler(
            "xyz",
            [0.12, -0.08, 0.35],
        )
        desired_pelvis_world = Rotation.from_euler("z", 0.5 * np.pi)
        rigid_world = desired_pelvis_world * correction.inv()
        bridge = ChingMuTableLcmBridge(
            config=self.config,
            table_frame=self.table,
            pelvis_orientation_calibration=(
                valid_orientation_calibration(correction)
            ),
        )

        base = self.base_message(
            bridge,
            mocap_frame(0, rigid_world),
        )

        published = Rotation.from_quat(base.quat_vicon)
        forward_world = published.apply([1.0, 0.0, 0.0])
        np.testing.assert_allclose(
            forward_world,
            [0.0, 1.0, 0.0],
            atol=1.0e-10,
        )

    def test_valid_pose_loss_still_emits_one_invalid_transition(self):
        bridge = ChingMuTableLcmBridge(
            config=self.config,
            table_frame=self.table,
            pelvis_orientation_calibration=(
                valid_orientation_calibration()
            ),
        )
        bridge.process_frame(mocap_frame(0, Rotation.identity()))
        missing = mocap_frame(1, Rotation.identity())
        missing = MocapFrame(
            frame_number=missing.frame_number,
            source_time_s=missing.source_time_s,
            body_position_mm=None,
            body_quaternion_xyzw=None,
            body_markers_mm={},
            unlabeled_markers_mm=np.empty((0, 3)),
        )

        first_loss = bridge.process_frame(missing)
        second_loss = bridge.process_frame(missing)

        first_base = [
            message
            for message in first_loss
            if message.name == self.config.base_subject
        ]
        second_base = [
            message
            for message in second_loss
            if message.name == self.config.base_subject
        ]
        self.assertEqual(len(first_base), 1)
        self.assertEqual(first_base[0].valid, 0)
        self.assertEqual(second_base, [])
```

- [ ] **Step 2: 运行 runtime 测试并确认 RED**

Run:

```bash
conda run --no-capture-output -n rb python -m unittest \
  deploy.mocap_bridge.tests.test_chingmu_pelvis_orientation.PelvisOrientationRuntimeTest \
  -v
```

Expected: constructor 不接受 `pelvis_orientation_calibration`，测试失败。

- [ ] **Step 3: 删除硬编码 pelvis offset 并接入旋转外参**

Delete from `BridgeConfig`:

```python
    pelvis_offset_heading_m: tuple = (0.003145, 0.044074, 0.048231)
```

Change constructor:

```python
class ChingMuTableLcmBridge:
    def __init__(
        self,
        *,
        config: BridgeConfig,
        table_frame: TableFrame,
        pelvis_orientation_calibration: PelvisOrientationCalibration,
    ):
        self.config = config
        self.table_frame = table_frame
        self.pelvis_orientation_calibration = (
            pelvis_orientation_calibration
        )
        self._base_pose_was_valid = False
        self.ball_tracker = BallTracker()
```

Replace the valid body branch with:

```python
        if body_pose is not None:
            base_world, rigid_rotation_world = body_pose
            pelvis_rotation_world = (
                rigid_rotation_world
                @ self.pelvis_orientation_calibration.rotation_rigid_from_pelvis
            )
            messages.append(
                make_message(
                    self.config.base_subject,
                    base_world,
                    quaternion_xyzw_from_rotation(
                        pelvis_rotation_world
                    ),
                    frame_number=frame.frame_number,
                    source_time_s=frame.source_time_s,
                    valid=True,
                    publish_time_us=publish_time_us,
                )
            )
            self._base_pose_was_valid = True
```

- [ ] **Step 4: 增加正常运行加载参数**

Add to `build_arg_parser()`:

```python
    parser.add_argument(
        "--pelvis-orientation-calib",
        type=Path,
        help=(
            "Saved ChingMu rigid-to-MuJoCo-pelvis orientation "
            "calibration JSON; required for normal bridge runtime."
        ),
    )
```

Add:

```python
def _load_runtime_pelvis_orientation(
    args,
    config: BridgeConfig,
) -> PelvisOrientationCalibration:
    if args.pelvis_orientation_calib is None:
        raise ValueError(
            "--pelvis-orientation-calib is required for normal runtime"
        )
    return load_pelvis_orientation_calibration(
        args.pelvis_orientation_calib,
        config,
    )
```

In `main()`, after config validation and before constructing the SDK client:

```python
    pelvis_orientation_calibration = (
        _load_runtime_pelvis_orientation(args, config)
    )
```

Pass it to the bridge:

```python
        bridge = ChingMuTableLcmBridge(
            config=config,
            table_frame=table,
            pelvis_orientation_calibration=(
                pelvis_orientation_calibration
            ),
        )
```

- [ ] **Step 5: 为 normal runtime 缺失文件写测试**

Extend imports:

```python
    _load_runtime_pelvis_orientation,
    build_arg_parser,
```

Append to `PelvisOrientationRuntimeTest`:

```python
    def test_normal_runtime_requires_orientation_file(self):
        args = build_arg_parser().parse_args([])

        with self.assertRaisesRegex(
            ValueError,
            "--pelvis-orientation-calib is required",
        ):
            _load_runtime_pelvis_orientation(args, self.config)
```

- [ ] **Step 6: 运行全部聚焦测试并确认 GREEN**

Run:

```bash
conda run --no-capture-output -n rb python -m unittest \
  deploy.mocap_bridge.tests.test_chingmu_pelvis_orientation \
  -v
```

Expected: 15 tests pass。

- [ ] **Step 7: 确认旧 offset 和首帧状态均不存在**

Run:

```bash
rg -n "pelvis_offset_heading_m|initial_base_yaw|relative_yaw|yaw_quaternion_from_rotation" \
  deploy/mocap_bridge/chingmu_table_lcm_bridge.py
```

Expected: no output。

- [ ] **Step 8: 检查并提交 Task 3**

Run:

```bash
git diff --check -- \
  deploy/mocap_bridge/chingmu_table_lcm_bridge.py \
  deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py
git add -p -- deploy/mocap_bridge/chingmu_table_lcm_bridge.py
git add -p -- deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py
git diff --cached --name-status
git diff --cached --check
git commit -m "feat: apply calibrated ChingMu pelvis orientation"
```

Expected: only the bridge and focused test file are staged。

---

### Task 4: 增加一次性标定 CLI 模式并保留桌面标定入口

**Files:**
- Modify: `deploy/mocap_bridge/chingmu_table_lcm_bridge.py:178-236,577-690`
- Modify: `deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py`

**Interfaces:**
- Consumes:
  - `_collect_calibration_frames(client, duration_s, argument_name=...)`
  - `calibrate_pelvis_orientation_from_frames(...)`
  - `save_pelvis_orientation_calibration(...)`
  - `_load_runtime_pelvis_orientation(...)`
- Produces:
  - CLI `--save-pelvis-orientation-calib PATH`
  - CLI `--pelvis-calib-sec FLOAT`
  - `_operation_mode(args) -> str`
  - operation modes `table_calibration`, `pelvis_calibration`, `runtime`

- [ ] **Step 1: 写 CLI 模式解析失败测试**

Extend imports:

```python
    _operation_mode,
```

Append:

```python
class PelvisOrientationCliTest(unittest.TestCase):
    def setUp(self):
        self.parser = build_arg_parser()

    def test_selects_pelvis_calibration_mode(self):
        args = self.parser.parse_args(
            [
                "--table-calib",
                "table.json",
                "--save-pelvis-orientation-calib",
                "pelvis.json",
            ]
        )
        self.assertEqual(_operation_mode(args), "pelvis_calibration")

    def test_selects_runtime_mode(self):
        args = self.parser.parse_args(
            ["--pelvis-orientation-calib", "pelvis.json"]
        )
        self.assertEqual(_operation_mode(args), "runtime")

    def test_preserves_table_only_calibration_mode(self):
        args = self.parser.parse_args(
            ["--save-table-calib", "table.json"]
        )
        self.assertEqual(_operation_mode(args), "table_calibration")

    def test_pelvis_calibration_requires_existing_table_file_argument(self):
        args = self.parser.parse_args(
            [
                "--save-pelvis-orientation-calib",
                "pelvis.json",
            ]
        )
        with self.assertRaisesRegex(
            ValueError,
            "--table-calib is required",
        ):
            _operation_mode(args)

    def test_pelvis_calibration_forbids_publish(self):
        args = self.parser.parse_args(
            [
                "--table-calib",
                "table.json",
                "--save-pelvis-orientation-calib",
                "pelvis.json",
                "--publish",
            ]
        )
        with self.assertRaisesRegex(
            ValueError,
            "cannot be combined with --publish",
        ):
            _operation_mode(args)

    def test_load_and_save_orientation_are_mutually_exclusive(self):
        args = self.parser.parse_args(
            [
                "--table-calib",
                "table.json",
                "--pelvis-orientation-calib",
                "old.json",
                "--save-pelvis-orientation-calib",
                "new.json",
            ]
        )
        with self.assertRaisesRegex(ValueError, "mutually exclusive"):
            _operation_mode(args)

    def test_missing_orientation_file_is_rejected_outside_table_mode(self):
        args = self.parser.parse_args([])
        with self.assertRaisesRegex(
            ValueError,
            "--pelvis-orientation-calib is required",
        ):
            _operation_mode(args)
```

- [ ] **Step 2: 运行 CLI 测试并确认 RED**

Run:

```bash
conda run --no-capture-output -n rb python -m unittest \
  deploy.mocap_bridge.tests.test_chingmu_pelvis_orientation.PelvisOrientationCliTest \
  -v
```

Expected: import error，指出 `_operation_mode` 或新 CLI 参数不存在。

- [ ] **Step 3: 增加 CLI 参数和模式解析**

Add to `build_arg_parser()`:

```python
    parser.add_argument(
        "--save-pelvis-orientation-calib",
        type=Path,
        help=(
            "Collect one aligned-pose calibration, atomically save "
            "the pelvis orientation JSON, and exit."
        ),
    )
    parser.add_argument(
        "--pelvis-calib-sec",
        type=float,
        default=2.0,
        help="Pelvis orientation aligned-pose collection duration.",
    )
```

Add:

```python
def _operation_mode(args) -> str:
    saving_pelvis = args.save_pelvis_orientation_calib is not None
    loading_pelvis = args.pelvis_orientation_calib is not None
    if saving_pelvis and loading_pelvis:
        raise ValueError(
            "--pelvis-orientation-calib and "
            "--save-pelvis-orientation-calib are mutually exclusive"
        )
    if saving_pelvis:
        if args.table_calib is None:
            raise ValueError(
                "--table-calib is required when saving pelvis "
                "orientation calibration"
            )
        if args.publish:
            raise ValueError(
                "--save-pelvis-orientation-calib cannot be combined "
                "with --publish"
            )
        return "pelvis_calibration"
    if loading_pelvis:
        return "runtime"
    if (
        args.save_table_calib is not None
        and args.table_calib is None
        and not args.publish
    ):
        return "table_calibration"
    raise ValueError(
        "--pelvis-orientation-calib is required outside "
        "table-only calibration mode"
    )
```

- [ ] **Step 4: 让帧采集错误信息区分两个 duration 参数**

Change:

```python
def _collect_calibration_frames(
    client,
    duration_s: float,
    *,
    argument_name: str = "--calib-sec",
) -> list:
    if duration_s <= 0.0:
        raise ValueError(f"{argument_name} must be positive")
    frames = []
    start = time.monotonic()
    while time.monotonic() - start < duration_s:
        frame = client.next_frame(timeout_s=0.1)
        if frame is not None:
            frames.append(frame)
    return frames
```

- [ ] **Step 5: 增加标定结果打印函数**

Add:

```python
def _describe_pelvis_orientation(
    calibration: PelvisOrientationCalibration,
) -> None:
    quaternion = quaternion_xyzw_from_rotation(
        calibration.rotation_rigid_from_pelvis
    )
    print(
        "Pelvis orientation calibration: "
        f"samples={calibration.sample_count} "
        f"source_duration_s={calibration.source_duration_s:.3f} "
        f"position_rms_m={calibration.position_rms_m:.6f} "
        f"angular_rms_deg={calibration.angular_rms_deg:.4f} "
        "quaternion_rigid_from_pelvis_xyzw="
        f"{np.array2string(quaternion, precision=7)}",
        flush=True,
    )
```

- [ ] **Step 6: 按模式重排 `main()`**

Immediately after parsing and config validation:

```python
    operation_mode = _operation_mode(args)
```

Remove the unconditional Task 3 call to `_load_runtime_pelvis_orientation` before SDK construction。

After the existing table load/calibration and `_describe_table(...)`:

```python
        if operation_mode == "table_calibration":
            print(
                "Saved table calibration; exiting before pelvis "
                "orientation is required.",
                flush=True,
            )
            return 0

        if operation_mode == "pelvis_calibration":
            print(
                "Pelvis aligned-pose calibration: confirm pelvis "
                "+X/+Y/+Z are parallel to table-world +X/+Y/+Z. "
                f"Collecting for {args.pelvis_calib_sec:.2f} s ...",
                flush=True,
            )
            frames = _collect_calibration_frames(
                client,
                args.pelvis_calib_sec,
                argument_name="--pelvis-calib-sec",
            )
            pelvis_orientation_calibration = (
                calibrate_pelvis_orientation_from_frames(
                    frames,
                    table,
                    config,
                )
            )
            _describe_pelvis_orientation(
                pelvis_orientation_calibration
            )
            save_pelvis_orientation_calibration(
                args.save_pelvis_orientation_calib,
                pelvis_orientation_calibration,
                config,
            )
            print(
                "Saved pelvis orientation calibration to "
                f"{args.save_pelvis_orientation_calib}",
                flush=True,
            )
            return 0

        pelvis_orientation_calibration = (
            _load_runtime_pelvis_orientation(args, config)
        )
        print(
            "Loaded pelvis orientation calibration from "
            f"{args.pelvis_orientation_calib}",
            flush=True,
        )
        _describe_pelvis_orientation(
            pelvis_orientation_calibration
        )
```

Keep LCM publisher construction and `ChingMuTableLcmBridge` construction after this block, so invalid/missing orientation files fail before publishing。

- [ ] **Step 7: 运行全部聚焦测试并确认 GREEN**

Run:

```bash
conda run --no-capture-output -n rb python -m unittest \
  deploy.mocap_bridge.tests.test_chingmu_pelvis_orientation \
  -v
```

Expected: 22 tests pass。

- [ ] **Step 8: 验证 CLI help**

Run:

```bash
conda run --no-capture-output -n rb python \
  deploy/mocap_bridge/chingmu_table_lcm_bridge.py --help
```

Expected: output includes:

```text
--pelvis-orientation-calib
--save-pelvis-orientation-calib
--pelvis-calib-sec
```

- [ ] **Step 9: 检查并提交 Task 4**

Run:

```bash
git diff --check -- \
  deploy/mocap_bridge/chingmu_table_lcm_bridge.py \
  deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py
git add -p -- deploy/mocap_bridge/chingmu_table_lcm_bridge.py
git add -p -- deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py
git diff --cached --name-status
git diff --cached --check
git commit -m "feat: add ChingMu pelvis orientation calibration mode"
```

Expected: only the bridge and focused test file are staged。

---

### Task 5: 离线总验证与现场交接

**Files:**
- Verify: `deploy/mocap_bridge/chingmu_table_lcm_bridge.py`
- Verify: `deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py`
- Do not create offline:
  `deploy/mocap_bridge/calibrations/chingmu_pelvis_orientation_latest.json`

**Interfaces:**
- Consumes: completed Tasks 1–4。
- Produces: verified offline implementation plus exact one-time calibration and normal runtime commands。

- [ ] **Step 1: 运行完整聚焦测试**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2
conda run --no-capture-output -n rb python -m unittest \
  deploy.mocap_bridge.tests.test_chingmu_pelvis_orientation \
  -v
```

Expected: 22 tests pass with zero failures and zero errors。

- [ ] **Step 2: 运行语法、帮助和静态不变量检查**

Run:

```bash
conda run --no-capture-output -n rb python -m py_compile \
  deploy/mocap_bridge/chingmu_table_lcm_bridge.py \
  deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py
conda run --no-capture-output -n rb python \
  deploy/mocap_bridge/chingmu_table_lcm_bridge.py --help |
  rg -n -- "--pelvis-orientation-calib|--save-pelvis-orientation-calib|--pelvis-calib-sec"
if rg -n "pelvis_offset_heading_m|initial_base_yaw|relative_yaw|yaw_quaternion_from_rotation" \
  deploy/mocap_bridge/chingmu_table_lcm_bridge.py; then
  exit 1
fi
git diff --check -- \
  deploy/mocap_bridge/chingmu_table_lcm_bridge.py \
  deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py
```

Expected:

- both files compile；
- help contains all three new flags；
- forbidden-name `rg` prints nothing；
- `git diff --check` prints nothing。

- [ ] **Step 3: 审查提交范围和保留的脏工作树**

Run:

```bash
git log --oneline -5
git status --short
git diff HEAD~4..HEAD --name-status
```

Expected:

- implementation commits only add/modify the bridge and focused test file；
- pre-existing unrelated modifications, deletions and untracked files remain untouched；
- no generated pelvis calibration JSON has been committed。

- [ ] **Step 4: 现场运行一次性标定，必须等待操作者摆正机器人**

Do not run this step until the operator confirms:

```text
pelvis +X = table world +X
pelvis +Y = table world +Y
pelvis +Z = table world +Z
robot is stationary
policy output is not running
```

Then run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2
conda run --no-capture-output -n rb python \
  deploy/mocap_bridge/chingmu_table_lcm_bridge.py \
  --host 192.168.2.100 \
  --base-subject G1Pelvis \
  --table-calib deploy/mocap_bridge/calibrations/chingmu_table_frame_latest.json \
  --save-pelvis-orientation-calib deploy/mocap_bridge/calibrations/chingmu_pelvis_orientation_latest.json \
  --pelvis-calib-sec 2.0
```

Expected:

- command does not publish LCM；
- at least 30 valid samples and at least `1.0 s` source duration；
- `position_rms_m <= 0.002`；
- `angular_rms_deg <= 0.3`；
- JSON is written only after all checks pass。

- [ ] **Step 5: 使用保存文件启动正常 bridge**

Run only after Step 4 succeeds:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2
conda run --no-capture-output -n rb python \
  deploy/mocap_bridge/chingmu_table_lcm_bridge.py \
  --host 192.168.2.100 \
  --base-subject G1Pelvis \
  --table-calib deploy/mocap_bridge/calibrations/chingmu_table_frame_latest.json \
  --pelvis-orientation-calib deploy/mocap_bridge/calibrations/chingmu_pelvis_orientation_latest.json \
  --publish
```

Expected:

- bridge prints the loaded calibration and quality metrics before publishing；
- position equals the table-world ChingMu rigid origin；
- aligned pose has pelvis roll/pitch/yaw within about `1 deg` of zero；
- aligned pose `base_forward_xy` is within about `1 deg` of `[1, 0]`；
- left/right `90 deg` checks approach `[0, 1]` and `[0, -1]`；
- restarting with the same JSON does not redefine yaw zero。

- [ ] **Step 6: Report the honest completion boundary**

If only Steps 1–3 are complete, report:

```text
离线代码和自动化测试已完成；真实 orientation JSON 尚未生成，
需要操作者摆正机器人后执行现场标定命令。
```

Only after Steps 4–5 pass may the implementation be reported as live-calibrated。
