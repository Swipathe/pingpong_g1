# HITTER 乒乓球复现流程记录

记录时间：2026-07-20

本记录对应仓库：

```bash
/home/yhl/Desktop/Omega-Athlete-full
```

HITTER 的训练侧在：

```bash
/home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/HITTER-main
```

RobotBridge2/MuJoCo 部署侧在：

```bash
/home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/RobotBridge2
```

整体链路是：

```text
motions 参考动作数据
-> IsaacLab/RSL-RL 训练或加载 policy checkpoint
-> 导出 policy.onnx
-> RobotBridge2 读取 ONNX metadata
-> MuJoCo 仿真验证击球部署
```

## 0. 环境和 LFS 检查

先进入 IsaacLab 环境：

```bash
conda activate isaaclab
```

确认 Python、PyTorch、GPU：

```bash
python -c "import torch; print(torch.__version__); print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'cpu')"
```

确认 Git LFS 文件已经拉下来：

```bash
cd /home/yhl/Desktop/Omega-Athlete-full

git lfs pull

find . -type f -size -200c -exec sh -c 'head -1 "$1" | grep -q "version https://git-lfs.github.com/spec/v1" && echo "$1"' sh {} \;
```

如果最后一条没有输出，说明没有残留 LFS pointer 文件。之前确认过关键 USD 资产已经是正常文件：

```bash
ls -lh github_publish/HITTER/HITTER-main/source/whole_body_tracking/whole_body_tracking/assets/unitree_description/usd/g1_hitter_racket/main.usda
```

正常大小约为 `4.5M`。

## 1. 安装 HITTER-main 本地包

```bash
cd /home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/HITTER-main

python -m pip install -e source/rsl_rl
python -m pip install -e source/whole_body_tracking
```

这一步很重要。HITTER-main 里改过 `whole_body_tracking.tasks.hitter`，如果环境里之前装过 MOSAIC 的同名包，需要用这里的 editable install 覆盖到当前项目版本。

## 2. 数据和参考动作检查

HITTER 使用的参考动作数据在：

```bash
/home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/HITTER-main/motions
```

里面包含 forehand/backhand 的 `.npz` motion 文件。先不跑 policy，只回放参考动作，检查 motion、G1+球拍资产、IsaacLab 是否正常：

```bash
cd /home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/HITTER-main

python scripts/rsl_rl/play_reference_motion.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion motions \
  --num_envs=4
```

成功现象：IsaacLab 里会出现 4 个机器人同步或分别回放击球参考动作。这里有 4 个机器人是因为 `--num_envs=4`，想只看一个可以改成：

```bash
python scripts/rsl_rl/play_reference_motion.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion motions \
  --num_envs=1
```

如果这一步能看到正手/反手击球动作，说明训练侧 motion 和资产链路基本 OK。

## 3. 训练 policy

从头训练 PPO policy 的入口是：

```bash
cd /home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/HITTER-main

python scripts/rsl_rl/train.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0
```

任务名：

```text
Hitter-Striking-PlannerDomain-Flat-G1-v0
```

训练代码会使用 HITTER task config、motion command、motion rewards、随机化和 RSL-RL runner。当前仓库已经带了一个打包好的 checkpoint，所以复现流程里不一定要重新训练到 20400 iteration，可以直接从下一步加载：

```bash
checkpoints/hitter_m20400/model_20400.pt
```

打包 checkpoint 目录包含：

```text
checkpoints/hitter_m20400/model_20400.pt
checkpoints/hitter_m20400/exported/policy.onnx
checkpoints/hitter_m20400/params/agent.yaml
checkpoints/hitter_m20400/params/env.yaml
```

## 4. 播放 checkpoint 并导出 ONNX

```bash
cd /home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/HITTER-main

python scripts/rsl_rl/play.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion motions \
  --resume_student_checkpoint checkpoints/hitter_m20400/model_20400.pt \
  --disable_obs_noise \
  --skip_critic
```

这一步会加载 `model_20400.pt`，并通常会导出或刷新：

```bash
checkpoints/hitter_m20400/exported/policy.onnx
```

已经修过的关键点：

- `play.py` 现在会直接使用 `--resume_student_checkpoint` 指定的 checkpoint 路径，而不是再去 `logs/rsl_rl/hitter_striking_g1` 查找训练日志目录。
- 播放时显式传 `--motion motions`，因为这个 evaluation 脚本需要 motion 输入。
- 播放和导出主要用 actor，所以加 `--skip_critic`，避免 critic 结构不匹配时影响部署验证。

确认 ONNX 输入输出维度：

```bash
cd /home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/HITTER-main

python - <<'PY'
import onnx
m = onnx.load("checkpoints/hitter_m20400/exported/policy.onnx")
for x in m.graph.input:
    print("input:", x.name, [d.dim_value for d in x.type.tensor_type.shape.dim])
for y in m.graph.output:
    print("output:", y.name, [d.dim_value for d in y.type.tensor_type.shape.dim])
PY
```

已经确认的正确输出是：

```text
input: obs [1, 105]
output: actions [1, 29]
```

含义：

- `105` 是 policy 每次推理吃进去的观测向量维度。
- `29` 是 policy 每次输出的动作维度，对应 G1 HITTER 的 29 个控制关节。

## 5. 确认 ONNX metadata

RobotBridge2 依赖 ONNX metadata 读取关节名、PD 参数、默认关节角、action scale、body names 和 observation 结构。

```bash
cd /home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/HITTER-main

python - <<'PY'
import onnx
p = "checkpoints/hitter_m20400/exported/policy.onnx"
m = onnx.load(p)
print("metadata keys:")
for item in m.metadata_props:
    print("-", item.key)
PY
```

期望至少有：

```text
joint_names
joint_stiffness
joint_damping
default_joint_pos
action_scale
anchor_body_name
body_names
observation_names
observation_history_lengths
```

## 6. RobotBridge2 MuJoCo 仿真

RobotBridge2 默认可能找：

```bash
RobotBridge2/deploy/data/model/hitter/hitter.onnx
```

当前流程直接用 Hydra override 指向 HITTER-main 导出的 ONNX：

```bash
cd /home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/RobotBridge2/deploy

python run.py \
  --config-name=hitter \
  sim=mujoco \
  mimic.policy.checkpoint=../../HITTER-main/checkpoints/hitter_m20400/exported/policy.onnx
```

成功日志里应该看到类似：

```text
Total Number of dof: 29
Number of Action: 29
HITTER ball planner configured.
Loading ONNX Checkpoint from ../../HITTER-main/checkpoints/hitter_m20400/exported/policy.onnx
HITTER policy metadata configured, 29 degrees of freedom.
HITTER policy metadata loaded. Joints: 29 Anchor body: pelvis
```

这些日志说明：

- MuJoCo 里的机器人模型是 29 DoF。
- ONNX 输出动作也是 29 维。
- RobotBridge2 已经读到 HITTER metadata。
- ball planner 已经生成击球命令。

## 7. MuJoCo 慢放观察

MuJoCo 里击球动作很快。已经给 `RobotBridge2/deploy/simulator/mujoco.py` 增加了：

```text
robot.control.playback_slowdown
```

并在 HITTER control 配置里加了默认值：

```text
playback_slowdown: 1.0
```

4 倍慢放命令：

```bash
cd /home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/RobotBridge2/deploy

python run.py \
  --config-name=hitter \
  sim=mujoco \
  mimic.policy.checkpoint=../../HITTER-main/checkpoints/hitter_m20400/exported/policy.onnx \
  robot.control.playback_slowdown=4
```

如果仍然太快，可以改成：

```bash
robot.control.playback_slowdown=8
```

## 8. 关于 MuJoCo 里只看到击球、看不到完整来球

这是当前部署演示的默认逻辑。RobotBridge2/MuJoCo 这一步主要验证的是：

```text
读取球状态
-> planner 生成击球命令
-> ONNX policy 控制机器人
-> 机器人把球打出去
```

它不是完整比赛回合可视化。之前日志里出现：

```text
Armed HITTER command from ball ... tts=0.000s
```

`tts` 是 `time to strike`，即距离击球还有多久。`0.000s` 说明一开始就接近击球时刻，所以画面里主要能看到机器人把球打出去。

如果想人为看更长的来球过程，可以尝试改球的初始位置和速度：

```bash
cd /home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/RobotBridge2/deploy

python run.py \
  --config-name=hitter \
  sim=mujoco \
  mimic.policy.checkpoint=../../HITTER-main/checkpoints/hitter_m20400/exported/policy.onnx \
  robot.control.playback_slowdown=4 \
  +sim.config.table_tennis.ball_initial_pos='[2.4,0.0,1.1]' \
  +sim.config.table_tennis.ball_initial_lin_vel='[-1.2,0.0,0.0]'
```

这属于改演示球路，不保证策略一定击得漂亮。

## 9. 离屏或非实时运行

如果只想跑链路、不看窗口：

```bash
cd /home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/RobotBridge2/deploy

python run.py \
  --config-name=hitter \
  sim=mujoco \
  robot.control.viewer=false \
  robot.control.real_time=false \
  mimic.policy.checkpoint=../../HITTER-main/checkpoints/hitter_m20400/exported/policy.onnx
```

## 10. 本次踩坑和修复记录

### Git LFS 资产缺失

旧的 `Omega-Athlete-main` 里部分大文件是 Git LFS pointer，导致 IsaacLab 资产加载失败。换到：

```bash
/home/yhl/Desktop/Omega-Athlete-full
```

并执行 `git lfs pull` 后，关键 USD 文件恢复正常。

### HITTER-main 本地包未安装

曾出现 `ModuleNotFoundError: No module named 'rsl_rl'`。解决方式是在 HITTER-main 下执行：

```bash
python -m pip install -e source/rsl_rl
python -m pip install -e source/whole_body_tracking
```

### 缺少 USD cache 路径

`g1_hitter_racket/main.usda` 引用了：

```bash
HITTER-main/cache/g1_usd_hitter/main.usd
```

已通过 symlink 指向实际资产目录：

```bash
/home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/HITTER-main/cache/g1_usd_hitter
-> /home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/HITTER-main/source/whole_body_tracking/whole_body_tracking/assets/unitree_description/usd/g1_hitter_racket_from_urdf
```

### motion body index 不匹配

参考动作回放曾出现 body index 越界。原因是当前机器人 USD body ordering 和 31-body motion npz ordering 不完全一致。

已修：

```bash
/home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/HITTER-main/source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/commands.py
```

增加 `motion_body_names`，让 motion 侧索引和 robot 侧索引分开。

同时在：

```bash
/home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/HITTER-main/source/whole_body_tracking/whole_body_tracking/tasks/hitter/config/g1/flat_env_cfg.py
```

设置了 `G1_HITTER_MOTION_BODY_NAMES`，对应：

```text
pelvis + 29 actuated child links + right_racket_link
```

### play.py 打包 checkpoint 路径解析错误

曾出现：

```text
FileNotFoundError: logs/rsl_rl/hitter_striking_g1
```

原因是 `play.py` 收到 `--resume_student_checkpoint` 后仍然调用 `get_checkpoint_path(log_root_path, ...)` 去日志目录找 checkpoint。

已修为：显式传 `--resume_student_checkpoint` 时直接使用该 checkpoint 路径。

### Hydra 不允许覆盖 playback_slowdown

第一次使用：

```bash
robot.control.playback_slowdown=4
```

报：

```text
Key 'playback_slowdown' is not in struct
```

已在：

```bash
/home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/RobotBridge2/deploy/config/control/g1_hitter_racket.yaml
```

加入：

```yaml
playback_slowdown: 1.0
```

之后可以直接通过 Hydra override 调慢播放。
