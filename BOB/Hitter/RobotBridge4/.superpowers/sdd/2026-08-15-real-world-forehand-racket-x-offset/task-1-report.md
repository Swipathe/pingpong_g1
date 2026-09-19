# Task 1 交付报告：真机正手 policy 速度适配与诊断

## 完成内容

- `HitterEnv` 从 `policy.real_world_forehand_racket_velocity_x_offset_mps` 读取偏置，默认 `0.0`，并拒绝 bool、非数值、非有限值和负值。
- 新增 `_policy_racket_target_velocity_w()`：先验证并复制 planner 的 `(3,)` 速度为独立 `float32` 数组；仅当 `simulator.is_real` 且最终 `strike_type == "forehand"` 时，对世界/球桌 X 增加 `0.25 m/s`。
- active observation 使用该副本；冻结生命周期命令、`self.hitter_racket_target_vel_w` 和 planner 原始 `v_racket_target_w` 均未原地写入。
- 一次性目标日志保留 `v_racket_target_w_mps` 与 `speed_racket_mps`，并追加 `v_racket_policy_w_mps`、`speed_racket_policy_mps`、`policy_vx_offset_mps`。
- 真机配置保持当前 model7100 checkpoint，并配置 `real_world_forehand_racket_velocity_x_offset_mps: 0.25`。

## TDD 记录

### RED：真实生命周期到 observation

命令：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=deploy \
  /home/loco1/miniconda3/envs/rb/bin/python -m pytest \
  -p no:cacheprovider -q \
  deploy/tests/test_hitter_runtime_single_shot_integration.py::test_policy_racket_velocity_x_offset_is_real_forehand_only \
  deploy/tests/test_hitter_runtime_single_shot_integration.py::test_real_forehand_policy_racket_velocity_x_offset_does_not_accumulate
```

输出（关键失败与汇总）：

```text
F..F
FAILED ...test_policy_racket_velocity_x_offset_is_real_forehand_only[True-forehand-expected0]
x: array([1. , 0.1, 0.2], dtype=float32)
y: array([1.25, 0.1 , 0.2 ])
FAILED ...test_real_forehand_policy_racket_velocity_x_offset_does_not_accumulate
x: array([1. , 0.1, 0.2], dtype=float32)
y: array([1.25, 0.1 , 0.2 ])
2 failed, 2 passed, 2 warnings in 0.98s
```

原因正确：尚未实现真机正手 policy 输入 X 偏置；反手和 MuJoCo 正手均通过。

### RED：日志

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=deploy \
  /home/loco1/miniconda3/envs/rb/bin/python -m pytest \
  -p no:cacheprovider -q \
  deploy/tests/test_hitter_strike_target_logging.py::StrikeTargetPayloadTests::test_real_forehand_logs_raw_and_policy_racket_velocity
```

输出：`AssertionError: 'v_racket_policy_w_mps=[1.6500,-0.1000,0.7000]' not found ...`，`1 failed, 2 warnings in 0.95s`。原因正确：原日志尚未输出 policy 输入速度和偏置。

### GREEN：新增行为

观测测试：`4 passed, 2 warnings in 0.97s`；日志测试：`1 passed, 2 warnings in 0.97s`。

最终指定回归命令：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=deploy \
  /home/loco1/miniconda3/envs/rb/bin/python -m pytest \
  -p no:cacheprovider -q \
  deploy/tests/test_hitter_runtime_single_shot_integration.py \
  deploy/tests/test_hitter_strike_target_logging.py \
  deploy/tests/test_hitter_task_observation.py
```

最终输出：`57 passed, 3 failed, 2 warnings in 1.10s`。三项失败均是任务开始前已知的 RECOVERY prewarm 语义变化与旧断言冲突：

- `test_listener_consumes_every_snapshot_during_recovery[False]`
- `test_listener_consumes_every_snapshot_during_recovery[True]`
- `test_listener_rechecks_phase_after_waiting_outside_lifecycle_lock`

它们断言 RECOVERY 不提交 snapshot，而当前既有实现允许 RECOVERY prewarm；本任务未改动该路径。

## 配置与差异校验

```bash
git diff --check -- deploy/envs/hitter.py deploy/config/mimic/hitter.yaml \
  deploy/tests/test_hitter_runtime_single_shot_integration.py \
  deploy/tests/test_hitter_strike_target_logging.py
```

退出码 0，无输出。

```text
2:  checkpoint: ./data/model/hitter/hitter_model7100_A100_20260815_resume6700_posstd012_104.onnx
4:  real_world_forehand_racket_velocity_x_offset_mps: 0.25
```

## 本任务文件与精确位置

- `deploy/envs/hitter.py:81-88,885-921,1089-1139,1700-1734`
- `deploy/config/mimic/hitter.yaml:4`
- `deploy/tests/test_hitter_runtime_single_shot_integration.py:292-354`
- `deploy/tests/test_hitter_strike_target_logging.py:166-197`

`deploy/envs/hitter.py` 和 `deploy/config/mimic/hitter.yaml` 在开始前已含用户未提交修改；没有执行暂存、提交、重置、还原或清理。

## 自检

- 条件只匹配真机正手；真机反手与 MuJoCo 正手测试保持原速度。
- 每次 observation 和日志均通过同一 helper 生成独立副本；连续 observation 不累计，冻结原始 command 两个速度字段仍为 `[1.0, 0.1, 0.2]`。
- 日志测试确认原始 X `1.4000` 仍存在，policy X `1.6500`、policy 速度模长 `1.7951`、偏置 `0.2500` 均可诊断。

## 真机三终端命令（仅在现场确认与明确授权后执行；本任务未执行）

终端 A：单一 ChingMu v2 发布器（现场确认 host、标定文件与 LCM URL 后再启动）。

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
/home/loco1/miniconda3/envs/rb/bin/python deploy/mocap_bridge/chingmu_table_lcm_bridge.py \
  --host 192.168.2.100 --base-subject G2Pelvis \
  --table-calib deploy/mocap_bridge/calibrations/chingmu_table_frame_latest.json \
  --pelvis-orientation-calib deploy/mocap_bridge/calibrations/chingmu_g2_pelvis_orientation_latest.json \
  --lcm-url 'udpm://239.255.76.67:7667?ttl=255' \
  --channel vicon_state_data_v2 --publish
```

终端 B：机器人 transition layer（先用 `ifconfig` 确认承载 `192.168.123.164` 的真实网卡，替换 `<robot_interface>`）。

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4/unitree_sdk2/build/bin
./trans_wo_lock <robot_interface>
```

终端 C：HITTER policy（确认 A 的 v2 freshness、有效 G2Pelvis、标定与第二次 R2 前安全条件后）。

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4/deploy
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python run.py \
  --config-name=hitter sim=real_world device=cpu
```

## 关注项

- 指定回归集仍有上述 3 个既有 RECOVERY 旧断言失败；新增行为的 5 个直接测试均通过。
- pytest 输出含 2 个既有 `DeprecationWarning: invalid escape sequence \\*`；并非本任务新增。
- 真机命令未自动运行，且终端 A 的 host/LCM/标定、终端 B 的网卡必须以现场已验证值为准。

## Fix Round 1：float32 偏置边界与配置接线

### 修复内容

- `HitterEnv._forehand_policy_vx_offset_from_policy_cfg()` 集中从精确键 `policy.real_world_forehand_racket_velocity_x_offset_mps` 取值，缺省为 `0.0`；`__init__` 仅通过该 helper 初始化偏置。
- validator 除原有类型、有限、非负检查外，拒绝任何有限但不能表示为 `float32` 的偏置（包括 `1e100`）。
- 真机正手相加在 `float64` 中计算，转换回 `float32` 前检查和是否有限且不超过 `float32` 最大值；溢出抛出 `ValueError`，不会向 policy 注入 `inf`。

### RED

新增覆盖测试：

- `test_forehand_policy_vx_offset_reads_deployment_key_and_defaults_to_zero`
- `test_forehand_policy_vx_offset_rejects_invalid_or_unrepresentable_values`
- `test_real_forehand_policy_velocity_rejects_float32_addition_overflow`

命令：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=deploy \
  /home/loco1/miniconda3/envs/rb/bin/python -m pytest \
  -p no:cacheprovider -q \
  deploy/tests/test_hitter_forehand_policy_vx_offset.py
```

输出：`FFFFFFFFF`，`9 failed, 2 warnings in 0.96s`。前 8 项的正确 RED 原因是 `_forehand_policy_vx_offset_from_policy_cfg` 尚未实现；最后一项为 `Failed: DID NOT RAISE <class 'ValueError'>`，证明确有未拦截的 float32 加法溢出路径。

### GREEN

覆盖命令：

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=deploy \
  /home/loco1/miniconda3/envs/rb/bin/python -m pytest \
  -p no:cacheprovider -q \
  deploy/tests/test_hitter_forehand_policy_vx_offset.py \
  deploy/tests/test_hitter_runtime_single_shot_integration.py::test_policy_racket_velocity_x_offset_is_real_forehand_only \
  deploy/tests/test_hitter_runtime_single_shot_integration.py::test_real_forehand_policy_racket_velocity_x_offset_does_not_accumulate \
  deploy/tests/test_hitter_strike_target_logging.py::StrikeTargetPayloadTests::test_real_forehand_logs_raw_and_policy_racket_velocity
```

输出：`14 passed, 2 warnings in 0.97s`。

三文件回归命令与初始报告相同，输出：`57 passed, 3 failed, 2 warnings in 1.08s`。失败仍严格为既有的三个 RECOVERY prewarm 旧断言：

- `test_listener_consumes_every_snapshot_during_recovery[False]`
- `test_listener_consumes_every_snapshot_during_recovery[True]`
- `test_listener_rechecks_phase_after_waiting_outside_lifecycle_lock`

### 改动位置与自检

- `deploy/envs/hitter.py:81-86,886-944`：配置接线、float32 可表示性检查、以高精度相加并在窄化前检查。
- `deploy/tests/test_hitter_forehand_policy_vx_offset.py:1-64`：真实部署 YAML key/default、bool/字符串/NaN/Inf/负值/两种超 float32 有限值，以及加法溢出覆盖。

自检：部署 YAML 经 `OmegaConf.load` 后实际读取 `0.25`，空 policy 读取 `0.0`；validator 覆盖所有复审列出的非法类别；溢出测试以 `float32 max + float32 max` 确认检查的是加法结果而非仅检查 offset。未暂存、提交、reset、restore 或 clean；未修改 RECOVERY 路径。

## Fix Round 2：接受 OmegaConf `DictConfig` policy 节点

### 修复内容

- resolver 的 mapping 边界从内建 `dict` 改为 `collections.abc.Mapping`，因此真实 `OmegaConf.load(...).policy` 的 `DictConfig` 可以进入同一精确 key/default/value validator；非 mapping 仍会被拒绝。
- 保留并扩展未跟踪的 `deploy/tests/test_hitter_forehand_policy_vx_offset.py`，没有删除、移动或替换它。前一轮 review package 漏掉该文件仅因其未跟踪；下一轮 package 必须显式包含此路径。

### RED

新增测试：`test_forehand_policy_vx_offset_accepts_dictconfig_policy_node`。

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=deploy \
  /home/loco1/miniconda3/envs/rb/bin/python -m pytest \
  -p no:cacheprovider -q \
  deploy/tests/test_hitter_forehand_policy_vx_offset.py::test_forehand_policy_vx_offset_accepts_dictconfig_policy_node
```

输出：`1 failed, 2 warnings in 1.00s`，失败为 `TypeError: policy configuration must be a mapping.`；这证明真实 `DictConfig` 被原先的 `isinstance(..., dict)` 错误拒绝。

### GREEN 与覆盖

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=deploy \
  /home/loco1/miniconda3/envs/rb/bin/python -m pytest \
  -p no:cacheprovider -q \
  deploy/tests/test_hitter_forehand_policy_vx_offset.py \
  deploy/tests/test_hitter_runtime_single_shot_integration.py::test_policy_racket_velocity_x_offset_is_real_forehand_only \
  deploy/tests/test_hitter_runtime_single_shot_integration.py::test_real_forehand_policy_racket_velocity_x_offset_does_not_accumulate \
  deploy/tests/test_hitter_strike_target_logging.py::StrikeTargetPayloadTests::test_real_forehand_logs_raw_and_policy_racket_velocity
```

输出：`15 passed, 2 warnings in 0.99s`。其中覆盖了 YAML 容器转换后的 dict、真实 `DictConfig` policy node、空 `DictConfig` default、全部此前非法数值与正手速度适配/日志行为。

### 改动位置与自检

- `deploy/envs/hitter.py:2,907-909`：引入 `Mapping` 并用于 resolver 边界。
- `deploy/tests/test_hitter_forehand_policy_vx_offset.py:30-43`：真实 `DictConfig` 的部署值 `0.25` 与空节点 default `0.0`。

自检：`DictConfig` 只是接受的 mapping 形态，不绕过 key 名、缺省或 `_validated_forehand_policy_vx_offset()` 的类型/有限/float32 上界检查。未触碰 RECOVERY 行为；未暂存、提交、reset、restore 或 clean。
