MOSAIC 负责“把策略训练出来”，RobotBridge 负责“把策略跑起来，并接到仿真或真机上”。
再展开一点：
MOSAIC 是训练侧。
它主要在 Isaac Lab / Isaac Sim 里做 whole-body motion tracking 这件事，输入是各种 motion 数据、teleop 数据、适配器训练数据，输出是训练好的 policy checkpoint。它关心的是：
怎么定义 tracking 任务
怎么设计 reward / observation / MDP
怎么做 residual adaptation / distillation
怎么把人类动作跟踪能力学出来
所以你可以把它看成一个 “策略工厂”。
RobotBridge 是部署侧。
它不负责把策略从零训练出来，而是负责把已有策略接入运行环境。它关心的是：
在 MuJoCo 里怎么跑这个策略
在真机上怎么跑这个策略
locomotion 和 mimic policy 怎么切换
joystick / keyboard / teleop 指令怎么接进来
机器人底层通信怎么打通
所以它更像一个 “运行时桥接器” 或 “部署中间层”。
如果把两者连起来看，链路基本是：
在 MOSAIC 里训练出一个 motion tracking / mimic policy  
导出模型 checkpoint  
在 RobotBridge 里把这个 checkpoint 配进部署环境  
用 MuJoCo 先测，或者直接接真机跑  
如果需要，再叠加 locomotion policy，做 locomotion 和 mimic 的切换
从 README 里的表述看，官方关系其实也很明确：
MOSAIC README 说：这个仓库聚焦 teleoperation policy training
部署相关请看 RobotBridge
RobotBridge README 里又直接举了 mosaic 配置作为 motion mimic 的运行方式
所以它们不是并列重复，而是 上下游关系：
MOSAIC：上游，产出策略
RobotBridge：下游，消费策略并部署
如果你更偏工程视角，可以这么记：
MOSAIC 解决的是 “学什么、怎么学”
RobotBridge 解决的是 “学完以后怎么接系统、怎么落地跑”
如果你更偏机器人系统视角，可以这么记：
MOSAIC 更像 policy / learning stack
RobotBridge 更像 runtime / deployment stack
再直白一点，假设你现在要让 G1 模仿一段全身动作：
没有 MOSAIC，你可能没有一个像样的 tracking policy
没有 RobotBridge，你就算有 policy，也很难顺畅接到 MuJoCo、摇杆、真机通信链路里
所以这两个组件的关系不是“谁包含谁”，而是：
MOSAIC 产出能力，RobotBridge 承载能力。

## 本次最小闭环 smoke test 记录

这次已经把 `MOSAIC -> RobotBridge` 的最小使用流程完整走通了一遍。注意，这次目标不是训练出高质量策略，而是验证工程链路是否能从 motion 数据一路走到 RobotBridge/MuJoCo 中运行。

最终跑通的链路是：

```text
MOSAIC_Dataset G1 motion npz
  -> MOSAIC replay_npz.py 验证 motion
  -> MOSAIC train.py 最小训练
  -> 生成 model_4.pt checkpoint
  -> MOSAIC play.py 导出 policy.onnx
  -> 检查 ONNX metadata
  -> RobotBridge 加载 policy.onnx + motion
  -> MuJoCo 中出现机器人并开始运行
```

机器人在 MuJoCo 中很快摔倒，这是预期现象，因为本次只训练了 `5` 个 iteration，策略质量几乎没有收敛。本次成功点是：数据、训练、导出、部署接口都已经打通。

## 环境与数据

使用的 conda 环境：

```bash
conda activate isaaclab
```

由于机器是 RTX 5090，原始 Isaac Lab 推荐的 `torch==2.5.1+cu121` 不支持 `sm_120`，会出现类似：

```text
NVIDIA GeForce RTX 5090 with CUDA capability sm_120 is not compatible
```

因此实际升级到了：

```text
torch 2.9.1+cu128
```

验证命令：

```bash
python -c "import torch; print(torch.__version__); print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0)); print(torch.cuda.get_device_capability(0))"
```

实际结果：

```text
2.9.1+cu128
True
NVIDIA GeForce RTX 5090
(12, 0)
```

使用的 motion 数据来自：

```text
MOSAIC_Dataset/G1/optical_mocap/g1_W4_stageii.npz
```

下载的数据目录在：

```text
/home/yhl/Desktop/BAAI-Humanoid/MOSAIC_Dataset/G1/optical_mocap/
```

## 1. 验证 Isaac Lab

先验证 Isaac Lab 本身可以启动：

```bash
cd ~/Desktop/IsaacLab
./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py --headless
```

看到：

```text
[INFO]: Setup complete...
```

说明 Isaac Lab/Isaac Sim 基础环境可用。

## 2. 安装 MOSAIC 本地包

进入 MOSAIC：

```bash
cd ~/Desktop/BAAI-Humanoid/MOSAIC
```

安装本地包：

```bash
python -m pip install -e source/whole_body_tracking
python -m pip install -e source/rsl_rl
```

注意：安装 `whole_body_tracking` 时，pip 会把部分依赖升高，例如 `onnx`、`protobuf`、`wandb`。这和 Isaac Lab 的官方 pin 存在版本冲突风险。本次 smoke test 没有因此阻塞，但后续如果遇到奇怪问题，需要优先排查这些依赖版本。

## 3. 用 replay_npz.py 验证 motion

命令：

```bash
cd ~/Desktop/BAAI-Humanoid/MOSAIC

python scripts/replay_npz.py \
  --motion_file ../MOSAIC_Dataset/G1/optical_mocap/g1_W4_stageii.npz \
  --robot g1
```

作用：

`replay_npz.py` 不是训练脚本，也不会跑 policy。它的作用是把 `.npz` motion 文件加载进 Isaac Sim，然后逐帧把 G1 机器人按照 motion 中的关节轨迹和身体位姿摆出来，用来检查：

- `.npz` 字段是否完整
- motion 是否和 `g1` 机器人匹配
- G1 资产是否能正常加载
- 动作是否明显异常，例如乱飞、穿地、关节爆炸

本次 GUI 最后可以渲染出画面，说明 motion 数据、G1 资产、MOSAIC/Isaac Lab 基础回放链路正常。

## 4. 最小训练 smoke test

命令：

```bash
cd ~/Desktop/BAAI-Humanoid/MOSAIC

CUDA_VISIBLE_DEVICES=0 python scripts/rsl_rl/train.py \
  --task=Tracking-Flat-G1-v0 \
  --motion ../MOSAIC_Dataset/G1/optical_mocap/g1_W4_stageii.npz \
  --num_envs=256 \
  --headless \
  --logger tensorboard \
  --run_name smoke_g1_w4 \
  --max_iterations 5
```

说明：

- `CUDA_VISIBLE_DEVICES=0`：只用一张 5090，减少多卡/IOMMU/P2P 干扰
- `--task=Tracking-Flat-G1-v0`：单 motion tracking 任务
- `--num_envs=256`：小规模并行环境
- `--max_iterations 5`：只验证训练链路，不追求效果

训练成功后产物：

```text
logs/rsl_rl/g1_flat_mosaic_hybrid/2026-07-16_15-20-27_smoke_g1_w4/model_0.pt
logs/rsl_rl/g1_flat_mosaic_hybrid/2026-07-16_15-20-27_smoke_g1_w4/model_4.pt
```

这一步说明：

```text
motion npz -> MOSAIC train.py -> checkpoint .pt
```

已经跑通。

## 5. 导出 policy.onnx

使用 `model_4.pt` 导出 ONNX：

```bash
cd ~/Desktop/BAAI-Humanoid/MOSAIC

python scripts/rsl_rl/play.py \
  --task=Tracking-Flat-G1-v0 \
  --motion ../MOSAIC_Dataset/G1/optical_mocap/g1_W4_stageii.npz \
  --num_envs=1 \
  --headless \
  --resume_student_checkpoint logs/rsl_rl/g1_flat_mosaic_hybrid/2026-07-16_15-20-27_smoke_g1_w4/model_4.pt \
  --skip_critic
```

导出结果：

```text
logs/rsl_rl/g1_flat_mosaic_hybrid/2026-07-16_15-20-27_smoke_g1_w4/exported/policy.onnx
```

检查文件：

```bash
ls -lh logs/rsl_rl/g1_flat_mosaic_hybrid/2026-07-16_15-20-27_smoke_g1_w4/exported/policy.onnx
```

本次文件大小约：

```text
9.7M
```

检查 ONNX metadata：

```bash
python - <<'PY'
import onnx
p = 'logs/rsl_rl/g1_flat_mosaic_hybrid/2026-07-16_15-20-27_smoke_g1_w4/exported/policy.onnx'
m = onnx.load(p)
print('metadata keys:')
for item in m.metadata_props:
    print('-', item.key)
PY
```

成功看到：

```text
metadata keys:
- run_path
- joint_names
- joint_stiffness
- joint_damping
- default_joint_pos
- command_names
- observation_names
- observation_history_lengths
- action_scale
- anchor_body_name
- body_names
```

这些 metadata 是 RobotBridge 正确加载 MOSAIC policy 的关键。

## 6. 安装 RobotBridge 依赖

RobotBridge 的依赖文件在：

```text
RobotBridge/requirements.txt
```

为了避免 pip 重新改动已经调好的 `torch 2.9.1+cu128`，本次没有直接执行完整：

```bash
python -m pip install -r requirements.txt
```

而是安装除 `torch/numpy` 外的运行依赖：

```bash
cd ~/Desktop/BAAI-Humanoid/RobotBridge

python -m pip install \
  numpy-stl mujoco omegaconf hydra-core easydict scipy joblib lxml open3d \
  loguru rich pygame transforms3d pyzmq onnxruntime lcm imageio
```

## 7. 用 RobotBridge 跑 MuJoCo

进入部署目录：

```bash
cd ~/Desktop/BAAI-Humanoid/RobotBridge/deploy
```

运行：

```bash
python run.py \
  --config-name=mosaic \
  sim=mujoco \
  device=cpu \
  robot.control.use_teleop=false \
  mimic.motion.motion_path=../../MOSAIC_Dataset/G1/optical_mocap/g1_W4_stageii.npz \
  mimic.policy.checkpoint=../../MOSAIC/logs/rsl_rl/g1_flat_mosaic_hybrid/2026-07-16_15-20-27_smoke_g1_w4/exported/policy.onnx
```

这里建议显式加上：

```bash
robot.control.use_teleop=false
```

目的是让 RobotBridge 走纯 `.npz` motion tracking 分支，而不是 teleop command 分支。

本次结果：

- MuJoCo 画面成功出现
- RobotBridge 成功加载 motion
- RobotBridge 成功加载 `policy.onnx`
- RobotBridge 成功读取 MOSAIC ONNX metadata
- 机器人开始运行，但很快摔倒

摔倒是预期现象，因为 policy 只训练了 `5` 个 iteration。这个结果说明工程链路通了，不代表策略已经可用。

## 关键修复点 1：MOSAIC exporter.py metadata 兼容

第一次导出 ONNX 时，`policy.onnx` 模型本体已经生成，但写入 RobotBridge 需要的 metadata 时失败：

```text
AttributeError: 'ArticulationData' object has no attribute 'default_joint_pos_nominal'
```

出错位置：

```text
MOSAIC/source/whole_body_tracking/whole_body_tracking/utils/exporter.py
```

原因：

MOSAIC 的 exporter 原本使用：

```python
env.scene["robot"].data.default_joint_pos_nominal
```

但当前 Isaac Lab 的 `ArticulationData` 没有这个字段。标准字段是：

```python
default_joint_pos
```

修复方式：

给 `attach_onnx_metadata()` 增加 fallback：

```python
robot_data = env.scene["robot"].data

default_joint_pos = getattr(robot_data, "default_joint_pos_nominal", None)
if default_joint_pos is None:
    default_joint_pos = robot_data.default_joint_pos[0]
```

然后 metadata 中写：

```python
"default_joint_pos": default_joint_pos.cpu().tolist(),
```

修复后，ONNX metadata 成功写入。

## 关键修复点 2：RobotBridge MosaicEnv observation 维度补齐

第一次在 RobotBridge 中加载 ONNX 时，MuJoCo 窗口一闪而过并报错：

```text
Got invalid dimensions for input: obs
index: 1 Got: 770 Expected: 800
```

含义：

MOSAIC 导出的 ONNX policy 需要 `800` 维 observation，但 RobotBridge 的 `MosaicEnv` 只拼出了 `770` 维。

出错位置：

```text
RobotBridge/deploy/envs/mosaic.py
```

原因：

MOSAIC 训练时的 policy observation 里包含这些历史项：

```text
command: 58 * 5 = 290
motion_anchor_pos_b: 3 * 5 = 15
motion_anchor_ori_b: 6 * 5 = 30
base_lin_vel: 3 * 5 = 15
base_ang_vel: 3 * 5 = 15
joint_pos: 29 * 5 = 145
joint_vel: 29 * 5 = 145
actions: 29 * 5 = 145
```

总计：

```text
290 + 15 + 30 + 15 + 15 + 145 + 145 + 145 = 800
```

RobotBridge 原本少拼了：

```text
motion_anchor_pos_b: 15
base_lin_vel: 15
```

所以只有：

```text
800 - 30 = 770
```

修复方式：

在 `RobotBridge/deploy/envs/mosaic.py` 中增加两个 history buffer：

```python
self.obs_motion_anchor_pos_b_buffer = collections.deque(
    [np.zeros(3) for _ in range(self.history_length)],
    maxlen=self.history_length,
)
self.obs_base_lin_vel_buffer = collections.deque(
    [np.zeros(3) for _ in range(self.history_length)],
    maxlen=self.history_length,
)
```

在 `compute_observation()` 里 append：

```python
self.obs_motion_anchor_pos_b_buffer.append(obs_motion_anchor_pos_b)
self.obs_base_lin_vel_buffer.append(obs_base_lin_vel)
```

在 `obs_prop` 拼接中加入：

```python
np.array(self.obs_motion_anchor_pos_b_buffer).reshape(1, -1),
np.array(self.obs_base_lin_vel_buffer).reshape(1, -1),
```

修复后，RobotBridge 可以正常把 `800` 维 observation 喂给 ONNX policy。

## 追加实验：训练 1000 iteration 后再接 RobotBridge

在完成 5 iteration smoke test 之后，又对同一个单 motion 进行了更长一点的训练，用来观察部署到 MuJoCo 后的效果。

使用的 motion 仍然是：

```text
../MOSAIC_Dataset/G1/optical_mocap/g1_W4_stageii.npz
```

训练目录为：

```text
logs/rsl_rl/g1_flat_mosaic_hybrid/2026-07-16_19-09-45_g1_w4_single_motion_1k/
```

最终使用的 checkpoint：

```text
logs/rsl_rl/g1_flat_mosaic_hybrid/2026-07-16_19-09-45_g1_w4_single_motion_1k/model_999.pt
```

### 导出 1000 iteration 的 ONNX

导出命令：

```bash
cd ~/Desktop/BAAI-Humanoid/MOSAIC

python scripts/rsl_rl/play.py \
  --task=Tracking-Flat-G1-v0 \
  --motion ../MOSAIC_Dataset/G1/optical_mocap/g1_W4_stageii.npz \
  --num_envs=1 \
  --headless \
  --resume_student_checkpoint logs/rsl_rl/g1_flat_mosaic_hybrid/2026-07-16_19-09-45_g1_w4_single_motion_1k/model_999.pt \
  --skip_critic
```

输出中可以看到 checkpoint 正确加载：

```text
[CLI] Simplified resume from CLI: /home/yhl/Desktop/BAAI-Humanoid/MOSAIC/logs/rsl_rl/g1_flat_mosaic_hybrid/2026-07-16_19-09-45_g1_w4_single_motion_1k/model_999.pt
[CLI]   - load_run: 2026-07-16_19-09-45_g1_w4_single_motion_1k
[CLI]   - checkpoint: model_999.pt
```

导出时 observation 维度正确：

```text
Active Observation Terms in Group: 'policy' (shape: (800,))
Actor MLP: Linear(in_features=800, ...)
```

ONNX 导出阶段会出现类似日志：

```text
onnx_ir.passes.common.unused_removal
onnxscript.optimizer._constant_folding
Removed unused nodes
```

这些是 ONNX 导出器的图优化日志，不是错误。导出完成后，`play.py` 会继续进入 playback loop，所以终端看起来可能像“卡住”。确认 ONNX 已生成后可以 `Ctrl+C` 停止。

导出结果：

```text
logs/rsl_rl/g1_flat_mosaic_hybrid/2026-07-16_19-09-45_g1_w4_single_motion_1k/exported/policy.onnx
```

建议检查 metadata：

```bash
python - <<'PY'
import onnx
p = 'logs/rsl_rl/g1_flat_mosaic_hybrid/2026-07-16_19-09-45_g1_w4_single_motion_1k/exported/policy.onnx'
m = onnx.load(p)
print('metadata keys:')
for item in m.metadata_props:
    print('-', item.key)
PY
```

期望至少包含：

```text
joint_names
joint_stiffness
joint_damping
default_joint_pos
action_scale
anchor_body_name
body_names
```

### 用 RobotBridge 跑 1000 iteration 的 ONNX

运行命令：

```bash
cd ~/Desktop/BAAI-Humanoid/RobotBridge/deploy

python run.py \
  --config-name=mosaic \
  sim=mujoco \
  device=cpu \
  robot.control.use_teleop=false \
  mimic.motion.motion_path=../../MOSAIC_Dataset/G1/optical_mocap/g1_W4_stageii.npz \
  mimic.policy.checkpoint=../../MOSAIC/logs/rsl_rl/g1_flat_mosaic_hybrid/2026-07-16_19-09-45_g1_w4_single_motion_1k/exported/policy.onnx
```

输出中可以看到 RobotBridge 成功加载 motion 和 ONNX：

```text
Loading Mosaic motion: ../../MOSAIC_Dataset/G1/optical_mocap/g1_W4_stageii.npz
Loading ONNX Checkpoint from ../../MOSAIC/logs/rsl_rl/g1_flat_mosaic_hybrid/2026-07-16_19-09-45_g1_w4_single_motion_1k/exported/policy.onnx
Mosaic policy metadata configured, 29 degrees of freedom.
Mosaic policy metadata loaded. Joints: 29 Anchor body: torso_link
```

MuJoCo 画面可以正常出现并运行，说明：

```text
1000 iteration checkpoint
  -> policy.onnx
  -> RobotBridge
  -> MuJoCo runtime
```

这条链路也已经跑通。

### 关键对齐点：关闭 teleop command 模式

一开始用 RobotBridge 跑 1000 iteration 的 ONNX 时，如果不显式指定：

```bash
robot.control.use_teleop=false
```

RobotBridge 会沿用 G1 control 配置中的默认值：

```yaml
use_teleop: True
```

这会让 `RobotBridge/deploy/envs/mosaic.py` 进入 teleop command 分支：

```python
if getattr(self.cfg.control, "use_teleop", False):
    command, robot_anchor_pos_w, robot_anchor_quat_w, anchor_pos_w, anchor_quat_w = self._get_command_teleop()
else:
    command, robot_anchor_pos_w, robot_anchor_quat_w, anchor_pos_w, anchor_quat_w = self._get_command()
```

也就是说，即使命令里传入了 `.npz` motion，observation 的 command 来源也可能不是纯粹的 `.npz` reference motion，而是 teleop 路径。这会导致 RobotBridge/MuJoCo 中的动作和 MOSAIC `play.py`/`replay_npz.py` 看到的不一致。

加入：

```bash
robot.control.use_teleop=false
```

之后，RobotBridge 会走：

```python
self._get_command()
```

也就是从 `mimic.motion.motion_path` 指定的 `.npz` 中取 reference command。实际观察结果是：MuJoCo 中的动作明显更接近 Isaac Lab `play.py`，关键动作也能做出来，基本和 `replay_npz.py` 看到的参考动作一致。

因此，如果目标是验证：

```text
同一个 npz motion + 同一个 MOSAIC policy
在 RobotBridge/MuJoCo 中能否做 motion tracking
```

那么 RobotBridge 命令中应当显式加：

```bash
robot.control.use_teleop=false
```

这个点是本次从 “Isaac Lab 里像，RobotBridge 里不像” 定位出来的关键配置差异。

### 为什么 MuJoCo 运行一段时间后自动关闭

1000 iteration 版本在 MuJoCo 中运行一段时间后窗口自动关闭，日志里关键行是：

```text
All motion files processed
段错误 (核心已转储)
```

这里真正的原因是：

```text
单个 motion 文件播放结束了。
```

RobotBridge 当前 `mosaic` 模式会顺序播放 motion 文件。当传入的是单个 `.npz` 文件时：

```text
播放 g1_W4_stageii.npz
  -> motion 播放到最后一帧
  -> 尝试切换到下一个 motion
  -> 没有下一个文件
  -> 打印 All motion files processed
  -> 退出程序
```

所以这不是 ONNX 加载失败，也不是 observation 维度问题。后面的 `段错误` 更像是 MuJoCo/GLFW 在窗口关闭清理时的副作用，不是主要原因。

如果想让 MuJoCo 跑久一点，可以：

1. 传入一个 motion 目录，而不是单个 `.npz`：

```bash
mimic.motion.motion_path=../../MOSAIC_Dataset/G1/optical_mocap
```

这样 RobotBridge 会递归读取目录中的 `.npz` 文件，按顺序播放。

2. 换一个更长的 motion 文件。

3. 修改 RobotBridge，让单个 motion 播完后循环播放。

配置里虽然有：

```yaml
motion:
  loop: false
```

但当前 `RobotBridge/deploy/envs/mosaic.py` 中 `loop_motion` 没有真正接入 `next_motion()` 逻辑，所以只设置 `mimic.motion.loop=true` 不一定生效。如果需要单文件循环，需要补对应逻辑。

## 当前结论

本次已经完成 MOSAIC 到 RobotBridge 的最小闭环：

```text
数据 OK
MOSAIC replay OK
MOSAIC training OK
checkpoint OK
ONNX export OK
metadata OK
RobotBridge ONNX loading OK
MuJoCo runtime OK
```

进一步提高效果则需要更多 motion、更长训练，以及根据目标选择 MOSAIC 的完整训练流程。
