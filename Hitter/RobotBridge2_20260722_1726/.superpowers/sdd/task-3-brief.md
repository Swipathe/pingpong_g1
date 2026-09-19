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
      solref="0.03 0.10" solimp="0.95 0.99 0.001" />
<pair geom1="hitter_table_net" geom2="hitter_ball_geom"
      condim="6" friction="0.20 0.005 0.0001"
      solref="0.004 0.35" solimp="0.95 0.99 0.001" />
<pair geom1="right_racket_face_collision" geom2="hitter_ball_geom"
      condim="6" friction="0.20 0.005 0.0001"
      solref="0.004 0.35" solimp="0.95 0.99 0.001" />
```

`solref="0.03 0.10"` 仅作为稳定的初始桌面参数；本任务只声明纯物理接触和正向反弹，不声明与真机恢复系数完全相同。

- [ ] **步骤 4：运行动态接触测试**

运行任务 1 的 unittest 命令。

预期：全部测试通过；2、4、6 m/s 球拍子测试均检测到真实 contact 和正向法向出球速度。

- [ ] **步骤 5：检查本任务补丁（不提交）**

```bash
git diff -- deploy/tests/test_mujoco_physical_table_tennis.py \
  deploy/data/assets/g1/g1_29dof_hitter_racket_table_tennis.xml
```

---

