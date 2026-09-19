# 任务 3 实施报告：启用桌面、球网和球拍纯 MuJoCo 物理接触

## 状态

- 实施状态：GREEN，完成，未提交。
- TDD：先添加 XML 拓扑和真实 MuJoCo 动态测试并观察预期 RED，再只修改 MJCF 接触配置并得到 GREEN。
- 本任务源码修改范围：`deploy/tests/test_mujoco_physical_table_tennis.py` 与 `deploy/data/assets/g1/g1_29dof_hitter_racket_table_tennis.xml`。
- 未执行 `git add` 或 `git commit`；`git diff --cached --name-only` 无输出。

## RED 证据

命令（工作目录 `deploy/`）：

```bash
conda run --no-capture-output -n rb \
  python -m unittest discover -s tests \
  -p 'test_mujoco_physical_table_tennis.py' -v
```

退出码：`1`

输出：

```text
2026-07-22 14:27:46.191 | INFO     | utils.motion_lib.torch_humanoid_batch:<module>:41 - Using Humanoid Batch
test_copying_planner_command_does_not_call_analytic_hit_setter (test_mujoco_physical_table_tennis.AnalyticOverrideRemovalTests) ... ok
test_hitter_config_contains_no_analytic_ball_override_options (test_mujoco_physical_table_tennis.AnalyticOverrideRemovalTests) ... ok
test_mujoco_source_contains_no_analytic_ball_state_override (test_mujoco_physical_table_tennis.AnalyticOverrideRemovalTests) ... ok
test_falling_ball_contacts_table_and_rebounds (test_mujoco_physical_table_tennis.PurePhysicalContactTests) ... FAIL
test_racket_contact_is_physical_across_operating_speeds (test_mujoco_physical_table_tennis.PurePhysicalContactTests) ... ok
test_xml_defines_only_intended_ball_contact_pairs (test_mujoco_physical_table_tennis.PurePhysicalContactTests) ... FAIL

======================================================================
FAIL: test_falling_ball_contacts_table_and_rebounds (test_mujoco_physical_table_tennis.PurePhysicalContactTests)
----------------------------------------------------------------------
Traceback (most recent call last):
  File "/home/loco1/BOB/Hitter/RobotBridge2/deploy/tests/test_mujoco_physical_table_tennis.py", line 141, in test_falling_ball_contacts_table_and_rebounds
    self.assertTrue(contacted)
AssertionError: False is not true

======================================================================
FAIL: test_xml_defines_only_intended_ball_contact_pairs (test_mujoco_physical_table_tennis.PurePhysicalContactTests)
----------------------------------------------------------------------
Traceback (most recent call last):
  File "/home/loco1/BOB/Hitter/RobotBridge2/deploy/tests/test_mujoco_physical_table_tennis.py", line 102, in test_xml_defines_only_intended_ball_contact_pairs
    self.assertIn(
AssertionError: frozenset({'hitter_ball_geom', 'hitter_table_top'}) not found in {frozenset({'right_racket_face_collision', 'hitter_ball_geom'})}

----------------------------------------------------------------------
Ran 6 tests in 0.469s

FAILED (failures=2)
ERROR conda.cli.main_run:execute(127): `conda run python -m unittest discover -s tests -p test_mujoco_physical_table_tennis.py -v` failed. (See above for error)
```

RED 判定：失败原因与 brief 完全一致。旧 XML 缺少桌面（以及随后会检查的球网）显式 pair，桌面自由落体没有产生 `data.contact`；已有球拍 pair 的 2/4/6 m/s 真实动态子测试通过。任务 1、2 的三个回归测试继续通过。

## GREEN 证据

最终 fresh 验证命令（工作目录 `deploy/`）：

```bash
conda run --no-capture-output -n rb \
  python -m unittest discover -s tests \
  -p 'test_mujoco_physical_table_tennis.py' -v
```

退出码：`0`

输出：

```text
2026-07-22 14:29:15.572 | INFO     | utils.motion_lib.torch_humanoid_batch:<module>:41 - Using Humanoid Batch
test_copying_planner_command_does_not_call_analytic_hit_setter (test_mujoco_physical_table_tennis.AnalyticOverrideRemovalTests) ... ok
test_hitter_config_contains_no_analytic_ball_override_options (test_mujoco_physical_table_tennis.AnalyticOverrideRemovalTests) ... ok
test_mujoco_source_contains_no_analytic_ball_state_override (test_mujoco_physical_table_tennis.AnalyticOverrideRemovalTests) ... ok
test_falling_ball_contacts_table_and_rebounds (test_mujoco_physical_table_tennis.PurePhysicalContactTests) ... ok
test_racket_contact_is_physical_across_operating_speeds (test_mujoco_physical_table_tennis.PurePhysicalContactTests) ... ok
test_xml_defines_only_intended_ball_contact_pairs (test_mujoco_physical_table_tennis.PurePhysicalContactTests) ... ok

----------------------------------------------------------------------
Ran 6 tests in 0.439s

OK
```

## 动态物理接触结果

使用与测试相同的真实 `mujoco.MjModel` / `mujoco.MjData` 初始化和无 mock 的 `data.contact` pair 检查，再执行只读测量。每次 `mj_step` 为 `0.005 s`。下列结果来自首轮 `solref="0.03 0.10"`，只验证接触与正向反弹；后续连续反弹回归发现其数值增能，最终稳定参数和三次恢复比见报告末尾的后续修复记录。

输出：

```text
TABLE contacted=True first_step=7 max_rebound_vz=2.833904269
RACKET speed=2.0 contacted=True first_step=6 max_outgoing_normal_speed=0.408439941
RACKET speed=4.0 contacted=True first_step=4 max_outgoing_normal_speed=3.608797113
RACKET speed=6.0 contacted=True first_step=3 max_outgoing_normal_speed=3.207873248
```

结论：桌面检测到指定 geom pair 的真实 contact 且 `vz > 0`；球拍在 2、4、6 m/s 三个覆盖点均检测到指定 geom pair 的真实 contact，随后沿球拍正法向的速度均 `> 0`。测试在每个桌面子步还检查了全部 qpos/qvel 为有限值。

## 修改内容

- `deploy/tests/test_mujoco_physical_table_tennis.py`
  - 新增 XML 解析、geom id、真实 contact pair 辅助函数。
  - 新增仅允许目标球接触拓扑测试。
  - 新增桌面自由落体接触与正向反弹动态测试。
  - 新增球拍 2/4/6 m/s 接触与正向法向出球动态测试。
- `deploy/data/assets/g1/g1_29dof_hitter_racket_table_tennis.xml`
  - 中心白线改为视觉-only：`contype="0" conaffinity="0"`。
  - 删除 `hitter_table` / `hitter_ball` body exclude。
  - 增加桌面顶面与球、球网与球的显式 pair；保留并精确配置球拍与球 pair。
  - 桌面 pair 最终参数：`condim="6" friction="0.20 0.005 0.0001" solref="0.04 0.10" solimp="0.95 0.99 0.001"`。
  - 球网和球拍 pair：`condim="6" friction="0.20 0.005 0.0001" solref="0.004 0.35" solimp="0.95 0.99 0.001"`。

## 自审与额外验证

- 独立 XML 断言：`XML_ASSERTIONS_OK pairs=3 excludes=0 ball=0/0 center_line=0/0`。
- 球仍为 `contype="0" conaffinity="0"`；接触区恰好三个显式 pair，因此球不会通过 bitmask 与机器人全身、桌腿、地面或中心线意外接触。
- 控制周期只读核验：`CONTROL_TIMING_OK low_dt=0.005 decimation=4`；未修改控制配置。
- `git diff --check -- deploy/data/assets/g1/g1_29dof_hitter_racket_table_tennis.xml`：退出码 `0`。
- `conda run --no-capture-output -n rb python -m py_compile tests/test_mujoco_physical_table_tennis.py`：退出码 `0`，无输出。
- 未修改 Python 生产文件、ONNX、105D observation、action gating、真机后端或策略/控制周期。
- 任务 1/2 的既有未提交改动、工作树内其他用户改动、已删除测试与文档均保持原状；未执行 reset、checkout、restore、清理、暂存或提交。

## Concerns

- 无已知实现问题。
- 最终桌面 `solref="0.04 0.10"` 在回归覆盖的连续三次反弹中不增能；本任务不声称其恢复系数与真机完全相同。

## 后续数值稳定性回归修复

最终验证发现首轮桌面 `solref="0.03 0.10"` 在 `0.005 s` 子步下会在连续反弹中数值增能。因此严格按 TDD 新增一个真实 MuJoCo 状态机测试：在 `无接触→接触` 时记录上一步的向下速度，在 `接触→无接触` 时记录离开瞬间的向上速度；连续收集同一球的前三个独立 table contact episode，并逐次断言 `outgoing_speed <= incoming_speed + 1e-6`。

同时强化 XML 拓扑测试：显式 pair 集合必须恰好等于桌面、球网、球拍三项，且 `hitter_ball_geom` 的 `contype`、`conaffinity` 必须均为 `0`。

### 后续 RED 证据

命令（工作目录 `deploy/`）：

```bash
conda run --no-capture-output -n rb \
  python -m unittest discover -s tests \
  -p 'test_mujoco_physical_table_tennis.py' -v
```

在仍为 `solref="0.03 0.10"` 时退出码为 `1`，输出：

```text
2026-07-22 14:36:24.962 | INFO     | utils.motion_lib.torch_humanoid_batch:<module>:41 - Using Humanoid Batch
test_copying_planner_command_does_not_call_analytic_hit_setter (test_mujoco_physical_table_tennis.AnalyticOverrideRemovalTests) ... ok
test_hitter_config_contains_no_analytic_ball_override_options (test_mujoco_physical_table_tennis.AnalyticOverrideRemovalTests) ... ok
test_mujoco_source_contains_no_analytic_ball_state_override (test_mujoco_physical_table_tennis.AnalyticOverrideRemovalTests) ... ok
test_falling_ball_contacts_table_and_rebounds (test_mujoco_physical_table_tennis.PurePhysicalContactTests) ... ok
test_racket_contact_is_physical_across_operating_speeds (test_mujoco_physical_table_tennis.PurePhysicalContactTests) ... ok
test_repeated_table_bounces_do_not_gain_vertical_speed (test_mujoco_physical_table_tennis.PurePhysicalContactTests) ... FAIL
test_xml_defines_only_intended_ball_contact_pairs (test_mujoco_physical_table_tennis.PurePhysicalContactTests) ... ok

======================================================================
FAIL: test_repeated_table_bounces_do_not_gain_vertical_speed (test_mujoco_physical_table_tennis.PurePhysicalContactTests)
----------------------------------------------------------------------
Traceback (most recent call last):
  File "/home/loco1/BOB/Hitter/RobotBridge2/deploy/tests/test_mujoco_physical_table_tennis.py", line 179, in test_repeated_table_bounces_do_not_gain_vertical_speed
    self.assertLessEqual(
AssertionError: 2.7848542690621114 not less than or equal to 1.3787698083318072 : episode 2: incoming=1.3787688083318073, outgoing=2.7848542690621114

----------------------------------------------------------------------
Ran 7 tests in 0.590s

FAILED (failures=1)
ERROR conda.cli.main_run:execute(127): `conda run python -m unittest discover -s tests -p test_mujoco_physical_table_tennis.py -v` failed. (See above for error)
```

RED 判定：新增测试不是语法错误或装置错误；它精确捕获第二次独立反弹 episode 的离开速度约为入射速度的 2.02 倍。其余 6 个测试保持通过。

### 最小生产修复

仅把 `hitter_table_top` / `hitter_ball_geom` pair 的 `solref` 从 `0.03 0.10` 改为 `0.04 0.10`。球网和球拍 pair 均继续使用 `0.004 0.35`，其他属性及生产文件未改。

### 后续 GREEN 与最终 fresh 验证

同一完整 unittest 命令退出码为 `0`，输出：

```text
2026-07-22 14:37:43.112 | INFO     | utils.motion_lib.torch_humanoid_batch:<module>:41 - Using Humanoid Batch
test_copying_planner_command_does_not_call_analytic_hit_setter (test_mujoco_physical_table_tennis.AnalyticOverrideRemovalTests) ... ok
test_hitter_config_contains_no_analytic_ball_override_options (test_mujoco_physical_table_tennis.AnalyticOverrideRemovalTests) ... ok
test_mujoco_source_contains_no_analytic_ball_state_override (test_mujoco_physical_table_tennis.AnalyticOverrideRemovalTests) ... ok
test_falling_ball_contacts_table_and_rebounds (test_mujoco_physical_table_tennis.PurePhysicalContactTests) ... ok
test_racket_contact_is_physical_across_operating_speeds (test_mujoco_physical_table_tennis.PurePhysicalContactTests) ... ok
test_repeated_table_bounces_do_not_gain_vertical_speed (test_mujoco_physical_table_tennis.PurePhysicalContactTests) ... ok
test_xml_defines_only_intended_ball_contact_pairs (test_mujoco_physical_table_tennis.PurePhysicalContactTests) ... ok

----------------------------------------------------------------------
Ran 7 tests in 0.569s

OK
```

前三次独立桌面接触 episode 的只读测量结果：

```text
EPISODE 1 incoming=2.294300000 outgoing=1.650788708 ratio=0.719517373
EPISODE 2 incoming=1.684611292 outgoing=1.153686626 ratio=0.684838474
EPISODE 3 incoming=1.200713374 outgoing=0.928882269 ratio=0.773608664
```

三次恢复比均严格小于 `1`。

其他要求验证：

- `conda run --no-capture-output -n rb python -m py_compile tests/test_mujoco_physical_table_tennis.py`：退出码 `0`，无输出。
- `git diff --check -- deploy/data/assets/g1/g1_29dof_hitter_racket_table_tennis.xml`：退出码 `0`，无输出。
- 参数只读断言：`CONTACT_CONFIG_OK pairs=3 table_solref=0.04/0.10 net_solref=0.004/0.35 racket_solref=0.004/0.35 ball=0/0`。
- `git diff --cached --name-only`：无输出；仍未暂存或提交。
