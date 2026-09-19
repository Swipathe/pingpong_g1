# HITTER 在 MOSAIC 框架上的训练流程

本文记录当前移动后项目的训练流程，对应目录：

```text
/home/yhl/Desktop/Hitter
```

训练侧在：

```text
/home/yhl/Desktop/Hitter/MOSAIC-main
```

部署/仿真侧主要在：

```text
/home/yhl/Desktop/Hitter/RobotBridge2_20260722_1726
```

备份部署目录在：

```text
/home/yhl/Desktop/Hitter/RobotBridge2
```

直白说，HITTER 是把 MOSAIC 的“全身动作跟踪训练框架”改造成“乒乓球击球策略训练框架”。它仍然使用：

```text
IsaacLab 仿真
whole_body_tracking 任务系统
RSL-RL PPO
motion npz 加载
policy checkpoint
ONNX 导出
ONNX metadata
```

但任务从“模仿一段 motion”变成了“参考正手/反手挥拍动作，同时学会根据击球目标控制 G1+球拍”。

## 1. 环境准备

进入 IsaacLab 环境：

```bash
conda activate isaaclab
```

进入训练仓：

```bash
cd /home/yhl/Desktop/Hitter/MOSAIC-main
```

安装本地包：

```bash
python -m pip install -e source/rsl_rl
python -m pip install -e source/whole_body_tracking
```

这一步很重要。`MOSAIC-main` 里包含 HITTER 任务代码，如果环境里之前装过基础 MOSAIC 的同名包，需要用这里的 editable install 覆盖到当前项目版本。

确认 GPU：

```bash
python -c "import torch; print(torch.__version__); print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'cpu')"
```

## 2. 当前默认 motion 数据

本文默认使用的 motion 目录是：

```text
/home/yhl/Desktop/Hitter/MOSAIC-main/data/hitter_motions/wechat_20260608_g1_npz_strike43_fixed_racket_roll055_arm155_peak43_aligned
```

训练命令里写相对路径：

```text
data/hitter_motions/wechat_20260608_g1_npz_strike43_fixed_racket_roll055_arm155_peak43_aligned
```

当前这个目录里有 9 条 `.npz`：

```text
backhand/backhand_manual_005_cb108680_26.000_27.203_strike43_94f_50fps__unitree_g1.npz
backhand/backhand_manual_006_cb108680_27.200_29.000_strike43_94f_50fps__unitree_g1.npz
backhand/backhand_manual_007_cb108680_29.000_30.500_strike43_94f_50fps__unitree_g1.npz
backhand/backhand_manual_008_cb108680_30.500_31.800_strike43_94f_50fps__unitree_g1.npz
backhand/backhand_manual_009_cb108680_32.000_33.500_strike43_94f_50fps__unitree_g1.npz
forehand/forehand_manual_001_cb108680_15.754_17.418_strike43_94f_50fps__unitree_g1.npz
forehand/forehand_manual_002_cb108680_17.506_19.000_strike43_94f_50fps__unitree_g1.npz
forehand/forehand_manual_003_cb108680_19.003_21.000_strike43_94f_50fps__unitree_g1.npz
forehand/forehand_manual_004_cb108680_21.000_23.000_strike43_94f_50fps__unitree_g1.npz
```

这些 `.npz` 是已经处理好的 G1 击球参考动作。文件名中的关键信息：

```text
strike43
94f
50fps
unitree_g1
```

含义是：50Hz、约 94 帧、击球关键帧在第 43 帧，已经重定向到 Unitree G1。

## 3. 参考动作检查

训练前先回放 motion 本身，确认数据和资产正常：

```bash
cd /home/yhl/Desktop/Hitter/MOSAIC-main

python scripts/rsl_rl/play_reference_motion.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion data/hitter_motions/wechat_20260608_g1_npz_strike43_fixed_racket_roll055_arm155_peak43_aligned \
  --num_envs=4
```

只看一个机器人：

```bash
python scripts/rsl_rl/play_reference_motion.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion data/hitter_motions/wechat_20260608_g1_npz_strike43_fixed_racket_roll055_arm155_peak43_aligned \
  --num_envs=1
```

这一步不加载 checkpoint，也不跑 policy，只把 `.npz` 中的参考状态写到 IsaacLab 机器人上。如果这里不正常，后面训练没有意义。

## 4. HITTER 训练任务

基础 MOSAIC 常用 motion tracking 任务是：

```text
Tracking-Flat-G1-v0
```

HITTER 使用的任务是：

```text
Hitter-Striking-PlannerDomain-Flat-G1-v0
```

注册位置：

```text
source/whole_body_tracking/whole_body_tracking/tasks/hitter/config/g1/__init__.py
```

它指向：

```text
G1HitterStrikingPlannerDomainFlatEnvCfg
G1HitterStrikingPlannerDomainPPORunnerCfg
```

也就是：

```text
环境配置：机器人、仿真、观测、奖励、command、终止条件
PPO 配置：网络、学习率、迭代数、保存间隔
```

## 5. G1+球拍和 body 对齐

HITTER 训练时用的是 G1+球拍资产：

```python
G1_HITTER_RACKET_CFG
```

关键 body：

```text
anchor body: pelvis
racket body: right_racket_link
```

HITTER 单独维护了 motion body names 和 robot body names。原因是带球拍后的机器人 body 顺序和 `.npz` 里的 body 顺序不能按 index 直接对齐。

正确理解是：

```text
motion npz 是老师示范轨迹；
训练奖励不是逐个 link 全量照抄；
奖励只检查关键身体姿态 + 击球目标 + 稳定性。
```

## 6. command 逻辑

HITTER 的 command 类是：

```text
HitterStrikingCommand
```

它继承了 MOSAIC 多 motion 加载能力，同时增加击球任务变量：

```text
strike_type
strike_time_s
time_to_strike_s
base_target_pos
racket_target_pos
racket_target_vel
racket_normal
racket_strike_face_normal
```

直白说：

```text
MOSAIC command：这一帧参考动作长这样，你去跟。
HITTER command：参考动作是正手/反手挥拍，同时要在指定时间把球拍送到目标点、目标速度、目标方向。
```

### 当前 command 调参

当前关键配置在：

```text
source/whole_body_tracking/whole_body_tracking/tasks/hitter/config/g1/flat_env_cfg.py
```

当前调参结果：

```python
command.forehand_racket_y_offset_range = (-0.50, -0.50)
command.backhand_racket_y_offset_range = (0.20, 0.20)
command.forehand_racket_y_range = (-table_half_width, -1.0e-4)
command.backhand_racket_y_range = (0.0, table_half_width)
```

含义：

```text
forehand 时，球拍相对 base 的 y 偏移固定为 -0.50 m
backhand 时，球拍相对 base 的 y 偏移固定为 +0.20 m
球桌 -Y 半边采样 forehand，不包含中线
球桌 +Y 半边采样 backhand，包含中线
```

注意：这里的左右取决于当前训练坐标系约定。代码里实际是按 `y` 正负切分。

### racket_y_offset_b 是什么

`racket_y_offset_b` 是球拍相对机器人 base/pelvis 在 body frame 下的 y 方向偏移。

在 `commands.py` 中：

```python
base_y = desired_racket_y_b - racket_y_offset_b
```

也就是说，它和目标击球 y 位置一起决定机器人 base 应该站在哪里。

## 7. 正手/反手采样

当前正手/反手 motion group 采样比例：

```python
command.motion_group_sampling_ratios = {"forehand": 0.5, "backhand": 0.5}
```

所以训练中正手、反手各占一半。

强制只练一种：

```bash
python scripts/rsl_rl/train.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion data/hitter_motions/wechat_20260608_g1_npz_strike43_fixed_racket_roll055_arm155_peak43_aligned \
  --force_strike_type forehand
```

或：

```bash
python scripts/rsl_rl/train.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion data/hitter_motions/wechat_20260608_g1_npz_strike43_fixed_racket_roll055_arm155_peak43_aligned \
  --force_strike_type backhand
```

控制 motion group 比例：

```bash
python scripts/rsl_rl/train.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion data/hitter_motions/wechat_20260608_g1_npz_strike43_fixed_racket_roll055_arm155_peak43_aligned \
  --motion_group_sampling_ratios forehand=1.0,backhand=0.0
```

## 8. Policy 看到什么

HITTER actor observation 是 105 维。

主要包含：

```text
base_ang_vel              3
projected_gravity         3
base_forward_xy           2
base_target_pos           3
racket_target_pos         3
racket_target_vel         3
time_to_strike            1
joint_pos                 29
joint_vel                 29
last_action               29
```

合计：

```text
105
```

Policy 输出：

```text
29 维 action
```

对应 G1 的 29 个控制关节目标。

## 9. Critic 和奖励

HITTER 是非对称 actor-critic。actor 只看部署时可用的 105 维观测；critic 训练时可以看更多 privileged 信息，比如 reference joint state、body pose、base lin vel 等。

HITTER 保留一部分 motion imitation 奖励，但真正重点是：

```text
base_pos：身体/骨盆是否到目标站位
racket_pos：球拍是否到目标击球点
racket_vel：球拍速度是否对
racket_ori：拍面方向是否对
```

稳定性相关惩罚包括：

```text
脚滑
击球时支撑不稳
躯干过度前倾
base 角速度过大
关节限位
动作变化过快
力矩过大
```

## 10. 单卡基础训练

基础命令：

```bash
cd /home/yhl/Desktop/Hitter/MOSAIC-main

python scripts/rsl_rl/train.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion data/hitter_motions/wechat_20260608_g1_npz_strike43_fixed_racket_roll055_arm155_peak43_aligned \
  --headless \
  --logger tensorboard \
  --run_name peak43_aligned_single_gpu
```

默认训练配置：

```text
num_envs = 4096
num_steps_per_env = 24
max_iterations = 200000
save_interval = 100
experiment_name = hitter_striking_g1
```

默认不会因为“收敛”自动停止。它会跑到 `--max_iterations`，或者手动 `Ctrl+C` 停止。

## 11. 双卡训练

当前机器是双 5090。双卡训练要用 `torchrun`，并且必须加：

```text
--distributed
--headless
```

`--headless` 很重要。否则 Isaac Sim/Omniverse 会尝试创建 GUI 渲染窗口，多进程时可能因为 GPU 无法 present 到屏幕而段错误。

当前推荐长训命令是每张卡 32000 env：

```bash
cd /home/yhl/Desktop/Hitter/MOSAIC-main

torchrun --standalone --nnodes=1 --nproc_per_node=2 \
  scripts/rsl_rl/train.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion data/hitter_motions/wechat_20260608_g1_npz_strike43_fixed_racket_roll055_arm155_peak43_aligned \
  --num_envs 32000 \
  --distributed \
  --headless \
  --logger tensorboard \
  --run_name 202607232000_env32000_m20400 \
  --max_iterations 20400
```

这里：

```text
32000 env/GPU
双卡总 64000 env
每个 iteration 采样量 = 64000 * 24 = 1,536,000 timesteps
```

短测命令：

```bash
torchrun --standalone --nnodes=1 --nproc_per_node=2 \
  scripts/rsl_rl/train.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion data/hitter_motions/wechat_20260608_g1_npz_strike43_fixed_racket_roll055_arm155_peak43_aligned \
  --num_envs 32000 \
  --distributed \
  --headless \
  --logger tensorboard \
  --run_name env32000_smoke \
  --max_iterations 20
```

曾测试过 `50000 env/GPU`，可以进入训练循环，但每个 iteration 约 19-21 秒，吞吐约 115k-125k steps/s。它能跑不代表长期最划算。当前更稳的长期配置是 `32000 env/GPU`。

## 12. TensorBoard 看训练进度

训练时加：

```text
--logger tensorboard
--run_name <容易识别的名字>
```

TensorBoard：

```bash
cd /home/yhl/Desktop/Hitter/MOSAIC-main

tensorboard --logdir logs/rsl_rl/hitter_striking_g1
```

浏览器打开：

```text
http://localhost:6006
```

重点看：

```text
Train/mean_reward
Train/mean_episode_length
Loss/value_function
Loss/surrogate
Loss/learning_rate
Policy/mean_noise_std
Perf/total_fps
Metrics/motion/error_anchor_pos
Metrics/motion/error_joint_pos
Metrics/motion/hitter_strike_racket_pos_error
Metrics/motion/hitter_strike_racket_vel_error
Metrics/motion/hitter_strike_racket_ori_error
```

判断是否可以停：

```text
reward 连续几百 iteration 提升很小
episode length 稳定
motion error 不再明显下降
play.py 里动作不摔、不乱抖，击球时机接近 reference
RobotBridge2/MuJoCo 里能稳定把球打出去
```

训练不会自动判断收敛后停止。

## 13. 当前训练产物

当前这轮 command 修改后的训练 run：

```text
logs/rsl_rl/hitter_striking_g1/2026-07-23_20-01-33_202607232000_env32000_m20400/
```

已看到 checkpoint 保存到：

```text
model_4500.pt
```

常用测试 checkpoint：

```text
model_4100.pt
```

保存间隔仍是：

```text
save_interval = 100
```

所以目录中会有：

```text
model_100.pt
model_200.pt
...
model_4100.pt
...
```

## 14. IsaacLab 播放 checkpoint

播放当前训练出的 checkpoint：

```bash
cd /home/yhl/Desktop/Hitter/MOSAIC-main

python scripts/rsl_rl/play.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion data/hitter_motions/wechat_20260608_g1_npz_strike43_fixed_racket_roll055_arm155_peak43_aligned \
  --resume_student_checkpoint /home/yhl/Desktop/Hitter/MOSAIC-main/logs/rsl_rl/hitter_striking_g1/2026-07-23_20-01-33_202607232000_env32000_m20400/model_4100.pt \
  --num_envs=1 \
  --disable_obs_noise \
  --skip_critic
```

只看正手：

```bash
python scripts/rsl_rl/play.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion data/hitter_motions/wechat_20260608_g1_npz_strike43_fixed_racket_roll055_arm155_peak43_aligned \
  --resume_student_checkpoint /home/yhl/Desktop/Hitter/MOSAIC-main/logs/rsl_rl/hitter_striking_g1/2026-07-23_20-01-33_202607232000_env32000_m20400/model_4100.pt \
  --num_envs=1 \
  --disable_obs_noise \
  --skip_critic \
  --force_strike_type forehand
```

只看反手：

```bash
python scripts/rsl_rl/play.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion data/hitter_motions/wechat_20260608_g1_npz_strike43_fixed_racket_roll055_arm155_peak43_aligned \
  --resume_student_checkpoint /home/yhl/Desktop/Hitter/MOSAIC-main/logs/rsl_rl/hitter_striking_g1/2026-07-23_20-01-33_202607232000_env32000_m20400/model_4100.pt \
  --num_envs=1 \
  --disable_obs_noise \
  --skip_critic \
  --force_strike_type backhand
```

`play.py` 和 `train.py` 加载同一个 task/env config，所以会使用当前代码里的 command 配置。如果 checkpoint 是旧 command 训练出来的，而播放时用新 command，效果可能不匹配。

## 15. IsaacLab 录制视频

HITTER 任务里：

```text
sim.dt = 0.005
decimation = 4
env step_dt = 0.02s
```

25 秒仿真视频对应：

```text
25 / 0.02 = 1250 steps
```

录制 25 秒视频：

```bash
cd /home/yhl/Desktop/Hitter/MOSAIC-main

python scripts/rsl_rl/play.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion data/hitter_motions/wechat_20260608_g1_npz_strike43_fixed_racket_roll055_arm155_peak43_aligned \
  --resume_student_checkpoint /home/yhl/Desktop/Hitter/MOSAIC-main/logs/rsl_rl/hitter_striking_g1/2026-07-23_20-01-33_202607232000_env32000_m20400/model_4100.pt \
  --num_envs=1 \
  --disable_obs_noise \
  --skip_critic \
  --video \
  --video_length 1250
```

视频保存位置：

```text
logs/rsl_rl/hitter_striking_g1/2026-07-23_20-01-33_202607232000_env32000_m20400/videos/play/
```

IsaacLab GUI 播放可能看起来像慢放，这是 GUI 渲染跑不到实时导致的。`play.py` 当前没有现成的倍速播放参数。看最终部署效果时建议用 MuJoCo。

## 16. 导出 ONNX

`play.py` 会导出或刷新：

```text
logs/rsl_rl/hitter_striking_g1/<run>/exported/policy.onnx
```

如果只想导出，不看 GUI，可以加 `--headless`：

```bash
cd /home/yhl/Desktop/Hitter/MOSAIC-main

python scripts/rsl_rl/play.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion data/hitter_motions/wechat_20260608_g1_npz_strike43_fixed_racket_roll055_arm155_peak43_aligned \
  --resume_student_checkpoint /home/yhl/Desktop/Hitter/MOSAIC-main/logs/rsl_rl/hitter_striking_g1/2026-07-23_20-01-33_202607232000_env32000_m20400/model_4100.pt \
  --num_envs=1 \
  --disable_obs_noise \
  --skip_critic \
  --headless
```

检查 ONNX：

```bash
ls -lh logs/rsl_rl/hitter_striking_g1/2026-07-23_20-01-33_202607232000_env32000_m20400/exported/policy.onnx
```

检查维度：

```bash
python - <<'PY'
import onnx
p = "logs/rsl_rl/hitter_striking_g1/2026-07-23_20-01-33_202607232000_env32000_m20400/exported/policy.onnx"
m = onnx.load(p)
for x in m.graph.input:
    print("input:", x.name, [d.dim_value for d in x.type.tensor_type.shape.dim])
for y in m.graph.output:
    print("output:", y.name, [d.dim_value for d in y.type.tensor_type.shape.dim])
PY
```

期望：

```text
input: obs [1, 105]
output: actions [1, 29]
```

检查 metadata：

```bash
python - <<'PY'
import onnx
p = "logs/rsl_rl/hitter_striking_g1/2026-07-23_20-01-33_202607232000_env32000_m20400/exported/policy.onnx"
m = onnx.load(p)
print("metadata keys:")
for item in m.metadata_props:
    print("-", item.key)
PY
```

RobotBridge2 至少需要：

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

## 17. 接到 RobotBridge2/MuJoCo

用当前主部署目录：

```bash
cd /home/yhl/Desktop/Hitter/RobotBridge2_20260722_1726/deploy

python run.py \
  --config-name=hitter \
  sim=mujoco \
  mimic.policy.checkpoint=../../MOSAIC-main/logs/rsl_rl/hitter_striking_g1/2026-07-23_20-01-33_202607232000_env32000_m20400/exported/policy.onnx
```

用备份部署目录：

```bash
cd /home/yhl/Desktop/Hitter/RobotBridge2/deploy

python run.py \
  --config-name=hitter \
  sim=mujoco \
  mimic.policy.checkpoint=../../MOSAIC-main/logs/rsl_rl/hitter_striking_g1/2026-07-23_20-01-33_202607232000_env32000_m20400/exported/policy.onnx
```

MuJoCo 慢放：

```bash
python run.py \
  --config-name=hitter \
  sim=mujoco \
  mimic.policy.checkpoint=../../MOSAIC-main/logs/rsl_rl/hitter_striking_g1/2026-07-23_20-01-33_202607232000_env32000_m20400/exported/policy.onnx \
  robot.control.playback_slowdown=4
```

成功日志应包含：

```text
Total Number of dof: 29
Number of Action: 29
HITTER ball planner configured.
Loading ONNX Checkpoint from ...
HITTER policy metadata configured, 29 degrees of freedom.
HITTER policy metadata loaded. Joints: 29 Anchor body: pelvis
```

## 18. 常见问题

### 双卡启动段错误

如果报：

```text
Failed to find a graphics and/or presenting queue
Fatal Python error: Segmentation fault
```

通常是双卡训练没加 `--headless`。训练时加：

```text
--headless
```

### TensorBoard 警告

如果看到：

```text
TensorFlow installation not found - running with reduced feature set.
```

这是正常信息。TensorBoard 可以不装完整 TensorFlow，看 PyTorch/RSL-RL scalar 曲线不受影响。

### 训练日志目录找不到 run_name

日志目录不是单独的 `run_name`，而是：

```text
logs/rsl_rl/hitter_striking_g1/<时间戳>_<run_name>/
```

例如：

```text
2026-07-23_20-01-33_202607232000_env32000_m20400
```

### 旧打包 checkpoint 目录

当前移动后的训练仓没有旧式打包目录：

```text
checkpoints/hitter_m20400/
```

已有 checkpoint 主要在：

```text
MOSAIC-main/logs/rsl_rl/hitter_striking_g1/
```

### Git LFS 资产

如果 USD 资产加载失败，先检查是否残留 LFS pointer：

```bash
cd /home/yhl/Desktop/Hitter

find . -type f -size -200c -exec sh -c 'head -1 "$1" | grep -q "version https://git-lfs.github.com/spec/v1" && echo "$1"' sh {} \;
```

如有需要：

```bash
git lfs pull
```
