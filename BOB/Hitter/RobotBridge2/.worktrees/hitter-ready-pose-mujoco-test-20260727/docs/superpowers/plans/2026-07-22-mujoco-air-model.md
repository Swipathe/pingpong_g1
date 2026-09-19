# MuJoCo 空气模型实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: 使用 `superpowers:subagent-driven-development`（推荐）或 `superpowers:executing-plans`，按任务逐项实施并保留 RED/GREEN 证据。

**目标：** 在 HITTER 的 MuJoCo 模型中启用静止标准空气，使乒乓球飞行受到 MuJoCo 内置流体力影响。

**架构：** 空气参数只在乒乓球专用 MJCF 的 `<option>` 中声明，由 MuJoCo 在 `mj_step()` 内统一计算流体力；不添加 Python 状态覆写或自定义阻力。运行时 `Mujoco._load_asset()` 继续从控制配置写入 `low_dt=0.005 s`，因此本次只改变空气模型。

**技术栈：** MuJoCo MJCF、MuJoCo Python API、Python 3.8、`unittest`、NumPy。

## 全局约束

- 所有新增文档和注释使用中文。
- 使用 `wind="0 0 0"`、`density="1.225"`、`viscosity="0.000018"`。
- 重力显式保持 `gravity="0 0 -9.81"`。
- 不修改 5 ms 运行时子步长、发球状态、接触参数、planner 或真机路径。
- 不加入 Python 自定义空气阻力、轨迹覆写或解析式修正。
- 保留工作区全部无关修改，不覆盖现有脏文件。

---

### Task 1：启用并验证 MuJoCo 内置空气模型

**文件：**
- 修改：`deploy/data/assets/g1/g1_29dof_hitter_racket_table_tennis.xml:1-6`
- 修改：`deploy/tests/test_mujoco_physical_table_tennis.py:186-250`

**接口：**
- 产生：MJCF 全局 `option.gravity = [0, 0, -9.81]`
- 产生：MJCF 全局 `option.wind = [0, 0, 0]`
- 产生：MJCF 全局 `option.density = 1.225`
- 产生：MJCF 全局 `option.viscosity = 0.000018`

- [ ] **步骤 1：先写失败测试**

在 `PurePhysicalContactTests` 中增加一个测试，先读取 XML 的 `<option>` 属性，再通过 `mujoco.MjModel.from_xml_path()` 验证编译值：

```python
def test_xml_enables_still_standard_air(self):
    root = ET.parse(XML_PATH).getroot()
    option = root.find("./option")
    self.assertIsNotNone(option)
    self.assertEqual(option.attrib.get("gravity"), "0 0 -9.81")
    self.assertEqual(option.attrib.get("wind"), "0 0 0")
    self.assertEqual(option.attrib.get("density"), "1.225")
    self.assertEqual(option.attrib.get("viscosity"), "0.000018")

    model = mujoco.MjModel.from_xml_path(str(XML_PATH))
    np.testing.assert_allclose(model.opt.gravity, [0.0, 0.0, -9.81])
    np.testing.assert_allclose(model.opt.wind, [0.0, 0.0, 0.0])
    self.assertAlmostEqual(float(model.opt.density), 1.225)
    self.assertAlmostEqual(float(model.opt.viscosity), 0.000018)
```

- [ ] **步骤 2：运行测试并确认 RED**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
conda run --no-capture-output -n rb \
  python -m unittest \
  tests.test_mujoco_physical_table_tennis.PurePhysicalContactTests.test_xml_enables_still_standard_air -v
```

预期：因当前 XML 没有 `<option>` 而失败，失败点是 `option is None`。

- [ ] **步骤 3：写入最小模型配置**

在 `<compiler>` 后增加：

```xml
<option gravity="0 0 -9.81" wind="0 0 0" density="1.225" viscosity="0.000018" />
```

- [ ] **步骤 4：增加实际减速测试**

构造两个相同模型，把重力关闭并让球远离所有接触面，以相同初速度自由飞行；对照模型额外将 `density` 和 `viscosity` 设为 0。运行相同数量的 `mj_step()` 后，断言启用空气模型的球速小于对照球速，同时两条轨迹全部为有限数值。

- [ ] **步骤 5：运行定向与完整测试并确认 GREEN**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
conda run --no-capture-output -n rb \
  python -m unittest tests.test_mujoco_physical_table_tennis -v
```

预期：本文件全部测试通过，空气配置和物理接触测试均无数值不稳定告警。

- [ ] **步骤 6：检查最终差异**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2
git diff --check -- \
  deploy/data/assets/g1/g1_29dof_hitter_racket_table_tennis.xml \
  deploy/tests/test_mujoco_physical_table_tennis.py \
  docs/superpowers/specs/2026-07-22-mujoco-pure-physical-table-tennis-design.md \
  docs/superpowers/plans/2026-07-22-mujoco-air-model.md
```

预期：没有空白错误；最终差异只包含空气配置、空气测试和对应中文文档。
