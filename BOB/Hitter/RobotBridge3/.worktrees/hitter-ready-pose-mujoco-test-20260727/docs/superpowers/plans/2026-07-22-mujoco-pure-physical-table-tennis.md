# MuJoCo 纯物理乒乓球实施计划

> **供智能体执行：** 必须逐任务使用 `superpowers:subagent-driven-development`（推荐）或 `superpowers:executing-plans`。所有步骤均使用复选框跟踪。

**目标：** 彻底删除 Python 对仿真球位置和速度的脚本化覆写，让桌面反弹、球网接触和球拍击球全部由 MuJoCo 接触动力学计算。

**架构：** 规划器只生成策略观测，不再向仿真器注入出球速度。MuJoCo 每个低层子步只计算 PD 力矩、执行 `mujoco.mj_step()` 并读取状态；XML 通过显式接触对限定球只与桌面、球网和球拍发生物理接触。

**技术栈：** Python 3.8、`unittest`、MuJoCo 3.2.3、Hydra/OmegaConf、MJCF XML、NumPy。

## 全局约束

- 目标仓库固定为 `/home/loco1/BOB/Hitter/RobotBridge2`，不修改 Omega 项目。
- 所有新增文档和测试说明使用中文。
- 不恢复用户已经删除的旧测试或文档。
- 不修改 ONNX、105 维观测布局、action gating、真机后端或现有 50 Hz 策略周期。
- 初版保持 `low_dt=0.005`、`decimation=4`；只有目标速度范围测试证明漏碰时才另行设计 2 ms 子步修改。
- 不允许任何 fallback 直接写入规划出球速度。
- `deploy/envs/hitter.py` 已含用户未提交修改；本轮实施代码保持未提交，只检查目标 diff，避免把用户改动混入自动提交。

---

### 任务 1：切断规划器到仿真球状态的注入接口

**文件：**
- 新建：`deploy/tests/test_mujoco_physical_table_tennis.py`
- 修改：`deploy/config/hitter.yaml:15-30`
- 修改：`deploy/envs/hitter.py:280-290, 570-627`

**接口：**
- 输入：`HitterEnv._copy_hitter_command(command, validated=fields)`。
- 输出：只更新策略所需的击球类型、底座目标、球拍目标、球拍目标速度和 TTS；不调用仿真器的球状态接口。

- [ ] **步骤 1：先写失败测试**

在新测试文件中加入：

```python
from __future__ import annotations

import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import yaml

from envs.hitter import HitterEnv


DEPLOY_DIR = Path(__file__).resolve().parents[1]


class RecordingSimulator:
    def __init__(self):
        self.analytic_hit_calls = 0

    def set_hitter_analytic_racket_hit(self, _target_pos, _outgoing_vel):
        self.analytic_hit_calls += 1


class AnalyticOverrideRemovalTests(unittest.TestCase):
    def test_hitter_config_contains_no_analytic_ball_override_options(self):
        config = yaml.safe_load((DEPLOY_DIR / "config" / "hitter.yaml").read_text())
        table_tennis = config["sim"]["config"]["table_tennis"]
        for key in (
            "use_analytic_table_bounce",
            "use_analytic_racket_hit",
            "ball_geom_name",
            "racket_face_geom_name",
        ):
            self.assertNotIn(key, table_tennis)

    def test_copying_planner_command_does_not_call_analytic_hit_setter(self):
        env = HitterEnv.__new__(HitterEnv)
        env.simulator = RecordingSimulator()
        env.hitter_command_lifecycle = SimpleNamespace(recovery_duration_s=0.5)
        fields = {
            "strike_type_index": 0,
            "time_to_strike": 0.85,
            "base_target": np.array([-0.4, 0.0], dtype=np.float32),
            "base_height": np.float32(0.793),
            "racket_target": np.array([0.0, 0.1, 1.0], dtype=np.float32),
            "racket_velocity": np.array([2.8, 0.0, 0.5], dtype=np.float32),
            "ball_out_velocity": np.array([4.0, 0.0, 1.0], dtype=np.float32),
        }

        env._copy_hitter_command(SimpleNamespace(), validated=fields)

        self.assertEqual(env.simulator.analytic_hit_calls, 0)
```

- [ ] **步骤 2：运行测试并确认按预期失败**

运行：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
conda run --no-capture-output -n rb \
  python -m unittest discover -s tests \
  -p 'test_mujoco_physical_table_tennis.py' -v
```

预期：两个测试失败；配置仍含四个字段，且旧 setter 被调用一次。

- [ ] **步骤 3：做最小生产修改**

修改 `deploy/config/hitter.yaml`，使 `table_tennis` 只保留运行时仍消费的字段：

```yaml
table_tennis:
  enabled: true
  ball_body_name: hitter_ball
  ball_joint_name: hitter_ball_freejoint
```

规划器参数继续保留在 `mimic.motion.ball_planner`，不再复制到 MuJoCo 脚本配置。

修改 `HitterEnv`：

- 删除 `self.hitter_ball_out_vel_w` 初始化；
- `_validated_hitter_command_fields()` 不再返回只为仿真注入服务的 `ball_out_velocity`；
- `_copy_hitter_command()` 不再保存该字段；
- 完整删除 `set_hitter_analytic_racket_hit()` 调用块。

同时把测试中的 `fields["ball_out_velocity"]` 删除，确保测试描述最终接口。

- [ ] **步骤 4：重新运行测试并确认通过**

运行与步骤 2 相同的命令。

预期：`Ran 2 tests`，结果 `OK`。

- [ ] **步骤 5：检查本任务补丁（不提交）**

```bash
git diff -- deploy/tests/test_mujoco_physical_table_tennis.py \
  deploy/config/hitter.yaml deploy/envs/hitter.py
```

预期：只包含本任务改动和 `hitter.py` 原有的最高击球高度修改；不执行 `git add` 或 `git commit`。

---

### 任务 2：删除 MuJoCo 解析式反弹与击球运行时代码

**文件：**
- 修改：`deploy/tests/test_mujoco_physical_table_tennis.py`
- 修改：`deploy/simulator/mujoco.py:98-316, 370-400, 604-622`

**接口：**
- 输入：策略生成的绝对关节目标和 MuJoCo 当前状态。
- 输出：每个子步仅执行物理求解，不存在解析式球状态后处理。

- [ ] **步骤 1：追加失败测试**

在 `AnalyticOverrideRemovalTests` 中加入：

```python
    def test_mujoco_source_contains_no_analytic_ball_state_override(self):
        source = (DEPLOY_DIR / "simulator" / "mujoco.py").read_text()
        forbidden = (
            "use_analytic_table_bounce",
            "use_analytic_racket_hit",
            "analytic_racket_hit_armed",
            "set_hitter_analytic_racket_hit",
            "_apply_hitter_ball_analytic_table_bounce",
            "_apply_hitter_ball_analytic_racket_hit",
        )
        for token in forbidden:
            self.assertNotIn(token, source, token)
```

- [ ] **步骤 2：运行并确认失败原因正确**

运行任务 1 的 unittest 命令。

预期：新测试因 `mujoco.py` 中仍存在禁用标识符而失败；任务 1 的两个测试保持通过。

- [ ] **步骤 3：删除解析式运行时实现**

在 `Mujoco._init_table_tennis_state()` 中只保留：

```python
self.table_tennis_cfg = getattr(self.cfg, "table_tennis", None)
self.table_tennis_enabled = bool(
    self.table_tennis_cfg and self.table_tennis_cfg.get("enabled", False)
)
self.hitter_ball_body_id = -1
self.hitter_ball_qposadr = None
self.hitter_ball_qveladr = None
self.ball_pos_world = np.zeros(3, dtype=np.float32)
self.ball_quat_world = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
self.ball_vel_world = np.zeros(3, dtype=np.float32)
self.ball_ang_vel_world = np.zeros(3, dtype=np.float32)
self.ball_visible = False
self.ball_state_estimator_ready = False
self.ball_state_estimator_sample_count = 0
self.ball_state_estimator_min_samples = 31
self.randomize_hitter_ball = False
self.table_tennis_rng = np.random.default_rng()
self.table_tennis_ball_trajectory_index = 0
```

启用后先保留与解析式物理无关的随机重置配置：

```python
self.randomize_hitter_ball = bool(
    self.table_tennis_cfg.get("randomize_ball_on_reset", False)
)
self.table_tennis_rng = np.random.default_rng(
    self.table_tennis_cfg.get("ball_random_seed", None)
)
```

随后解析 `ball_body_name` 和 `ball_joint_name`，保留现有 body/joint 有效性检查、qpos/qvel 地址、轨迹候选选择和 `reset_hitter_ball(update_default=True)`。删除解析式物理时不得使随机/顺序轨迹重置分支失效。

删除：

- `_hitter_ball_inside_table()`；
- `_apply_hitter_ball_analytic_table_bounce()`；
- 所有 analytic racket 状态和辅助方法；
- `reset_hitter_ball()` 中的 analytic clear；
- `apply_action()` 中 `prev_ball_pos/prev_ball_vel` 缓存与两次后处理调用。

低层循环最终形态必须是：

```python
mujoco.mj_step(self.mujoco_model, self.mujoco_data)
```

- [ ] **步骤 4：运行单测和语法检查**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
conda run --no-capture-output -n rb \
  python -m unittest discover -s tests \
  -p 'test_mujoco_physical_table_tennis.py' -v
conda run --no-capture-output -n rb \
  python -m py_compile simulator/mujoco.py envs/hitter.py
```

预期：`Ran 3 tests` 且 `OK`；`py_compile` 无输出。

- [ ] **步骤 5：检查本任务补丁（不提交）**

```bash
git diff -- deploy/tests/test_mujoco_physical_table_tennis.py \
  deploy/simulator/mujoco.py
```

---

### 任务 3：启用桌面、球网和球拍纯物理接触

**文件：**
- 修改：`deploy/tests/test_mujoco_physical_table_tennis.py`
- 修改：`deploy/data/assets/g1/g1_29dof_hitter_racket_table_tennis.xml:302-319`

**接口：**
- 输入：球的 MJCF 自由关节状态。
- 输出：MuJoCo `contact` 中出现指定接触对，接触求解器决定反弹速度。

- [ ] **步骤 1：增加 XML 拓扑与动态失败测试**

在测试文件顶部增加：

```python
import xml.etree.ElementTree as ET

import mujoco
```

增加以下测试类和辅助函数：

```python
XML_PATH = (
    DEPLOY_DIR
    / "data"
    / "assets"
    / "g1"
    / "g1_29dof_hitter_racket_table_tennis.xml"
)


def _geom_id(model, name: str) -> int:
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    if geom_id < 0:
        raise AssertionError(f"缺少 MuJoCo geom: {name}")
    return int(geom_id)


def _has_contact(data, geom_a: int, geom_b: int) -> bool:
    expected = {geom_a, geom_b}
    return any(
        {int(data.contact[i].geom1), int(data.contact[i].geom2)} == expected
        for i in range(data.ncon)
    )


class PurePhysicalContactTests(unittest.TestCase):
    def test_xml_defines_only_intended_ball_contact_pairs(self):
        root = ET.parse(XML_PATH).getroot()
        pairs = {
            frozenset((pair.attrib["geom1"], pair.attrib["geom2"]))
            for pair in root.findall("./contact/pair")
        }
        self.assertIn(
            frozenset(("right_racket_face_collision", "hitter_ball_geom")),
            pairs,
        )
        self.assertIn(
            frozenset(("hitter_table_top", "hitter_ball_geom")),
            pairs,
        )
        self.assertIn(
            frozenset(("hitter_table_net", "hitter_ball_geom")),
            pairs,
        )
        self.assertEqual(root.findall("./contact/exclude"), [])
        center_line = root.find(".//geom[@name='hitter_table_center_line']")
        self.assertIsNotNone(center_line)
        self.assertEqual(center_line.attrib.get("contype"), "0")
        self.assertEqual(center_line.attrib.get("conaffinity"), "0")

    def test_falling_ball_contacts_table_and_rebounds(self):
        model = mujoco.MjModel.from_xml_path(str(XML_PATH))
        model.opt.timestep = 0.005
        data = mujoco.MjData(model)
        ball_geom = _geom_id(model, "hitter_ball_geom")
        table_geom = _geom_id(model, "hitter_table_top")
        ball_joint = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, "hitter_ball_freejoint"
        )
        qposadr = int(model.jnt_qposadr[ball_joint])
        qveladr = int(model.jnt_dofadr[ball_joint])
        data.qpos[qposadr:qposadr + 3] = [1.0, 0.25, 0.84]
        data.qvel[qveladr:qveladr + 3] = [0.0, 0.0, -2.0]
        mujoco.mj_forward(model, data)

        contacted = False
        rebound_vz = -np.inf
        for _ in range(120):
            mujoco.mj_step(model, data)
            contacted = contacted or _has_contact(data, ball_geom, table_geom)
            if contacted:
                rebound_vz = max(rebound_vz, float(data.qvel[qveladr + 2]))
            self.assertTrue(np.isfinite(data.qpos).all())
            self.assertTrue(np.isfinite(data.qvel).all())

        self.assertTrue(contacted)
        self.assertGreater(rebound_vz, 0.0)

    def test_racket_contact_is_physical_across_operating_speeds(self):
        for speed in (2.0, 4.0, 6.0):
            with self.subTest(speed=speed):
                model = mujoco.MjModel.from_xml_path(str(XML_PATH))
                model.opt.timestep = 0.005
                model.opt.gravity[:] = 0.0
                data = mujoco.MjData(model)
                mujoco.mj_forward(model, data)
                ball_geom = _geom_id(model, "hitter_ball_geom")
                racket_geom = _geom_id(model, "right_racket_face_collision")
                ball_joint = mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_JOINT, "hitter_ball_freejoint"
                )
                qposadr = int(model.jnt_qposadr[ball_joint])
                qveladr = int(model.jnt_dofadr[ball_joint])
                rotation = data.geom_xmat[racket_geom].reshape(3, 3)
                normal = rotation @ np.array([0.0, -1.0, 0.0])
                normal /= np.linalg.norm(normal)
                data.qpos[qposadr:qposadr + 3] = (
                    data.geom_xpos[racket_geom] + 0.15 * normal
                )
                data.qvel[qveladr:qveladr + 3] = -speed * normal
                mujoco.mj_forward(model, data)

                contacted = False
                outgoing_normal_speed = -np.inf
                for _ in range(80):
                    mujoco.mj_step(model, data)
                    contacted = contacted or _has_contact(
                        data, ball_geom, racket_geom
                    )
                    if contacted:
                        outgoing_normal_speed = max(
                            outgoing_normal_speed,
                            float(np.dot(data.qvel[qveladr:qveladr + 3], normal)),
                        )

                self.assertTrue(contacted)
                self.assertGreater(outgoing_normal_speed, 0.0)
```

- [ ] **步骤 2：运行并确认桌面相关测试失败**

运行任务 1 的 unittest 命令。

预期：XML 缺少桌面、球网接触对且仍有 `exclude`；下落球不会产生桌面接触。球拍物理接触子测试应通过。

- [ ] **步骤 3：修改 MJCF 接触配置**

把中心线设置为视觉几何体：

```xml
<geom name="hitter_table_center_line" ... contype="0" conaffinity="0" ... />
```

保持球为：

```xml
<geom name="hitter_ball_geom" ... contype="0" conaffinity="0" ... />
```

删除：

```xml
<exclude body1="hitter_table" body2="hitter_ball" />
```

在 `<contact>` 中设置三个显式物理接触对：

```xml
<pair geom1="hitter_table_top" geom2="hitter_ball_geom"
      condim="6" friction="0.20 0.005 0.0001"
      solref="0.04 0.10" solimp="0.95 0.99 0.001" />
<pair geom1="hitter_table_net" geom2="hitter_ball_geom"
      condim="6" friction="0.20 0.005 0.0001"
      solref="0.004 0.35" solimp="0.95 0.99 0.001" />
<pair geom1="right_racket_face_collision" geom2="hitter_ball_geom"
      condim="6" friction="0.20 0.005 0.0001"
      solref="0.004 0.35" solimp="0.95 0.99 0.001" />
```

`solref="0.04 0.10"` 是在 5 ms 子步下经过连续三次反弹不增能回归验证的稳定初值；本任务不声明其与真机恢复系数完全相同。首轮候选 `0.03 0.10` 会在第二次反弹产生数值增能，因此已弃用。

- [ ] **步骤 4：运行动态接触测试**

运行任务 1 的 unittest 命令。

预期：全部测试通过；连续三次桌面反弹均不增能，2、4、6 m/s 球拍子测试均检测到真实 contact 和正向法向出球速度。

- [ ] **步骤 5：检查本任务补丁（不提交）**

```bash
git diff -- deploy/tests/test_mujoco_physical_table_tennis.py \
  deploy/data/assets/g1/g1_29dof_hitter_racket_table_tennis.xml
```

---

### 任务 4：全量验证与物理参数报告

**文件：**
- 验证：`deploy/tests/test_mujoco_physical_table_tennis.py`
- 验证：`deploy/config/hitter.yaml`
- 验证：`deploy/envs/hitter.py`
- 验证：`deploy/simulator/mujoco.py`
- 验证：`deploy/data/assets/g1/g1_29dof_hitter_racket_table_tennis.xml`

**接口：**
- 输入：最终工作树。
- 输出：无脚本状态覆写、物理接触通过、语法和配置可加载的证据。

- [ ] **步骤 1：运行专用测试**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
conda run --no-capture-output -n rb \
  python -m unittest discover -s tests \
  -p 'test_mujoco_physical_table_tennis.py' -v
```

预期：全部测试为 `OK`。

- [ ] **步骤 2：运行语法与配置组合检查**

```bash
conda run --no-capture-output -n rb \
  python -m py_compile simulator/mujoco.py envs/hitter.py
conda run --no-capture-output -n rb python - <<'PY'
from pathlib import Path
from hydra import compose, initialize_config_dir
from omegaconf import OmegaConf

config_dir = Path("config").resolve()
with initialize_config_dir(version_base=None, config_dir=str(config_dir)):
    cfg = compose(config_name="hitter", overrides=["sim=mujoco"])
table_tennis = OmegaConf.to_container(
    cfg.sim.config.table_tennis, resolve=True
)
assert table_tennis == {
    "enabled": True,
    "ball_body_name": "hitter_ball",
    "ball_joint_name": "hitter_ball_freejoint",
}
print(table_tennis)
PY
```

预期：`py_compile` 无输出，配置脚本打印只含三个键的字典。

- [ ] **步骤 3：确认不存在脚本状态覆写残留**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2
rg -n "analytic_(table|racket)|use_analytic|set_hitter_analytic" \
  deploy/config/hitter.yaml deploy/envs/hitter.py deploy/simulator/mujoco.py
```

预期：无输出。

- [ ] **步骤 4：检查补丁完整性和脏工作区隔离**

```bash
git diff --check
git status --short
```

预期：`git diff --check` 无输出；状态中原有用户修改仍保持，实施代码未被自动提交。

- [ ] **步骤 5：记录标定边界**

最终交付中明确报告：

```text
桌面和球拍均已改为 MuJoCo 真实接触；
当前桌面 solref="0.04 0.10" 是 5 ms 下不连续增能的稳定初值，尚不能声称与真实桌面恢复系数完全等价；
5 ms 下已覆盖 2、4、6 m/s 球拍接触；
若现场速度超出范围出现漏碰，再单独评估 2 ms MuJoCo 子步。
```
