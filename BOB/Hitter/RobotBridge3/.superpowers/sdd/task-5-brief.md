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
