# HITTER 乒乓球复现流程记录

记录时间：2026-07-22

本记录对应移动后的项目目录：

```bash
/home/yhl/Desktop/Hitter
```

HITTER 的 IsaacLab/RSL-RL 训练侧现在在：

```bash
/home/yhl/Desktop/Hitter/MOSAIC-main
```

MuJoCo/部署侧主要看：

```bash
/home/yhl/Desktop/Hitter/RobotBridge2
```

目录里也有：

```bash
/home/yhl/Desktop/Hitter/RobotBridge
```

但本文沿用旧复现记录中的 RobotBridge2 部署链路，因为它包含 `--config-name=hitter`、`hitter_agent.py`、`hitter.py` 环境、G1+球拍+乒乓球 MuJoCo 资产等 HITTER 相关逻辑。

整体链路是：

```text
乒乓球参考动作 npz
-> MOSAIC-main / IsaacLab / RSL-RL 训练或加载 policy checkpoint
-> 导出 policy.onnx
-> RobotBridge2 读取 ONNX metadata
-> HITTER ball planner 生成击球命令
-> MuJoCo 中验证 G1 持拍击球
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
cd /home/yhl/Desktop/Hitter

git lfs pull

find . -type f -size -200c -exec sh -c 'head -1 "$1" | grep -q "version https://git-lfs.github.com/spec/v1" && echo "$1"' sh {} \;
```

如果最后一条没有输出，说明没有残留 LFS pointer 文件。关键 G1+球拍 USD 资产应当是正常大文件：

```bash
ls -lh MOSAIC-main/source/whole_body_tracking/whole_body_tracking/assets/unitree_description/usd/g1_hitter_racket/main.usda
```

## 1. 安装 MOSAIC-main 本地包

```bash
cd /home/yhl/Desktop/Hitter/MOSAIC-main

python -m pip install -e source/rsl_rl
python -m pip install -e source/whole_body_tracking
```

这一步很重要。`MOSAIC-main` 里包含 HITTER 任务代码，如果环境里之前装过基础 MOSAIC 的同名包，需要用这里的 editable install 覆盖到当前项目版本。

## 2. 数据和参考动作检查

当前 `play_reference_motion.py` 的默认 motion 路径是：

```bash
data/hitter_motions/wechat_20260608_g1_npz_strike43_fixed_racket_roll055_arm155_peak43_aligned
```

也可以使用混合正手/反手数据目录：

```bash
data/hitter_motions/hitter_mix_oldiphone_newwechat_20260608_strike43
data/hitter_motions/wechat_20260608_g1_npz_strike43_fixed_racket_roll055_arm155
data/hitter_motions/wechat_20260608_g1_npz_strike43_fixed_racket_roll055_arm155_peak43_aligned
```

先不跑 policy，只回放参考动作，检查 motion、G1+球拍资产、IsaacLab 是否正常：

```bash
cd /home/yhl/Desktop/Hitter/MOSAIC-main

python scripts/rsl_rl/play_reference_motion.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion data/hitter_motions/wechat_20260608_g1_npz_strike43_fixed_racket_roll055_arm155_peak43_aligned \
  --num_envs=4
```

想只看一个机器人可以改成：

```bash
python scripts/rsl_rl/play_reference_motion.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion data/hitter_motions/wechat_20260608_g1_npz_strike43_fixed_racket_roll055_arm155_peak43_aligned \
  --num_envs=1
```

如果这一步能看到正手/反手击球动作，说明训练侧 motion 和资产链路基本 OK。

## 3. 训练 policy

从头训练 PPO policy 的入口是：

```bash
cd /home/yhl/Desktop/Hitter/MOSAIC-main

python scripts/rsl_rl/train.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion data/hitter_motions/wechat_20260608_g1_npz_strike43_fixed_racket_roll055_arm155_peak43_aligned
```

任务名：

```text
Hitter-Striking-PlannerDomain-Flat-G1-v0
```

`--motion` 是必填参数。本文默认使用：

```text
data/hitter_motions/wechat_20260608_g1_npz_strike43_fixed_racket_roll055_arm155_peak43_aligned
```

上面的命令会递归加载这个目录下所有 `.npz`，当前共包含 9 条 reference motions：

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

训练代码会使用 HITTER task config、motion command、motion rewards、随机化和 RSL-RL runner。当前移动后的目录里没有旧版打包 checkpoint 目录：

```text
checkpoints/hitter_m20400/
```

已有 checkpoint 主要在：

```text
MOSAIC-main/logs/rsl_rl/hitter_striking_g1/
```

当前能找到的 `model_20400.pt` 有：

```text
logs/rsl_rl/hitter_striking_g1/2026-06-17_08-12-16_hitter_connector_m13300_skipopt_lr1e4_2xa100_env60000_20260617_081201/model_20400.pt
logs/rsl_rl/hitter_striking_g1/2026-06-26_13-25-48_fh_bh_uniform_peak43_swapped_normal_resume_m18800_reward_tuned_16000env_31body/model_20400.pt
```

### 训练时用 TensorBoard 看进度

建议训练时显式加上 `--logger tensorboard` 和一个容易识别的 `--run_name`：

```bash
cd /home/yhl/Desktop/Hitter/MOSAIC-main

python scripts/rsl_rl/train.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion data/hitter_motions/wechat_20260608_g1_npz_strike43_fixed_racket_roll055_arm155_peak43_aligned \
  --logger tensorboard \
  --run_name peak43_aligned_tb
```

训练日志会写到：

```text
logs/rsl_rl/hitter_striking_g1/<时间戳_peak43_aligned_tb>/
```

另开一个终端启动 TensorBoard：

```bash
cd /home/yhl/Desktop/Hitter/MOSAIC-main

tensorboard --logdir logs/rsl_rl/hitter_striking_g1
```

然后在浏览器打开终端显示的地址，通常是：

```text
http://localhost:6006
```

重点看这些曲线：

```text
Train/mean_reward
Train/mean_episode_length
Loss/value_function
Loss/surrogate
Loss/learning_rate
Policy/mean_noise_std
Perf/total_fps
```

判断训练是否在变好时，优先看 `Train/mean_reward` 是否整体上升、`Train/mean_episode_length` 是否变长或稳定。PPO 的 loss 曲线会有波动，不需要期待单调下降。最终还是要定期用 `play.py` 播放 checkpoint，看机器人是否稳定、不乱抖、击球时机是否接近 reference。

## 4. 播放 checkpoint 并导出 ONNX

以 `2026-06-26_13-25-48.../model_20400.pt` 为例：

```bash
cd /home/yhl/Desktop/Hitter/MOSAIC-main

python scripts/rsl_rl/play.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion data/hitter_motions/wechat_20260608_g1_npz_strike43_fixed_racket_roll055_arm155_peak43_aligned \
  --resume_student_checkpoint logs/rsl_rl/hitter_striking_g1/2026-06-26_13-25-48_fh_bh_uniform_peak43_swapped_normal_resume_m18800_reward_tuned_16000env_31body/model_20400.pt \
  --disable_obs_noise \
  --skip_critic
```

这一步会加载指定 checkpoint，并通常导出或刷新同一 run 目录下的：

```text
logs/rsl_rl/hitter_striking_g1/<run>/exported/policy.onnx
```

如果只是要先跑通 RobotBridge2/MuJoCo，也可以使用当前已经存在的 ONNX，例如：

```text
logs/rsl_rl/hitter_striking_g1/2026-06-26_11-36-52_fh_bh_uniform_peak43_swapped_normal_resume_m17900_userparams_16000env_31body/exported/policy.onnx
```

确认 ONNX 输入输出维度：

```bash
cd /home/yhl/Desktop/Hitter/MOSAIC-main

python - <<'PY'
import onnx
p = "logs/rsl_rl/hitter_striking_g1/2026-06-26_11-36-52_fh_bh_uniform_peak43_swapped_normal_resume_m17900_userparams_16000env_31body/exported/policy.onnx"
m = onnx.load(p)
for x in m.graph.input:
    print("input:", x.name, [d.dim_value for d in x.type.tensor_type.shape.dim])
for y in m.graph.output:
    print("output:", y.name, [d.dim_value for d in y.type.tensor_type.shape.dim])
PY
```

旧复现里确认过 HITTER policy 的典型维度是：

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
cd /home/yhl/Desktop/Hitter/MOSAIC-main

python - <<'PY'
import onnx
p = "logs/rsl_rl/hitter_striking_g1/2026-06-26_11-36-52_fh_bh_uniform_peak43_swapped_normal_resume_m17900_userparams_16000env_31body/exported/policy.onnx"
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

进入部署目录：

```bash
cd /home/yhl/Desktop/Hitter/RobotBridge2/deploy
```

RobotBridge2 默认可能找配置内置的 hitter ONNX。为了确保使用训练侧导出的 policy，建议直接用 Hydra override 指向 `MOSAIC-main` 中的 ONNX：

```bash
python run.py \
  --config-name=hitter \
  sim=mujoco \
  mimic.policy.checkpoint=../../MOSAIC-main/logs/rsl_rl/hitter_striking_g1/2026-06-26_11-36-52_fh_bh_uniform_peak43_swapped_normal_resume_m17900_userparams_16000env_31body/exported/policy.onnx
```

如果刚刚从 `model_20400.pt` 导出了新的 ONNX，把 `mimic.policy.checkpoint` 换成新导出的 `exported/policy.onnx` 即可。

成功日志里应该看到类似：

```text
Total Number of dof: 29
Number of Action: 29
HITTER ball planner configured.
Loading ONNX Checkpoint from ...
HITTER policy metadata configured, 29 degrees of freedom.
HITTER policy metadata loaded. Joints: 29 Anchor body: pelvis
```

这些日志说明：

- MuJoCo 里的机器人模型是 29 DoF。
- ONNX 输出动作也是 29 维。
- RobotBridge2 已经读到 HITTER metadata。
- ball planner 已经生成击球命令。

## 7. MuJoCo 慢放观察

MuJoCo 里击球动作很快。当前 RobotBridge2 的 HITTER control 配置支持：

```text
robot.control.playback_slowdown
```

4 倍慢放命令：

```bash
cd /home/yhl/Desktop/Hitter/RobotBridge2/deploy

python run.py \
  --config-name=hitter \
  sim=mujoco \
  mimic.policy.checkpoint=../../MOSAIC-main/logs/rsl_rl/hitter_striking_g1/2026-06-26_11-36-52_fh_bh_uniform_peak43_swapped_normal_resume_m17900_userparams_16000env_31body/exported/policy.onnx \
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

它不是完整比赛回合可视化。如果日志里出现：

```text
Armed HITTER command from ball ... tts=0.000s
```

`tts` 是 `time to strike`，即距离击球还有多久。`0.000s` 说明一开始就接近击球时刻，所以画面里主要能看到机器人把球打出去。

如果想人为看更长的来球过程，可以尝试改球的初始位置和速度：

```bash
cd /home/yhl/Desktop/Hitter/RobotBridge2/deploy

python run.py \
  --config-name=hitter \
  sim=mujoco \
  mimic.policy.checkpoint=../../MOSAIC-main/logs/rsl_rl/hitter_striking_g1/2026-06-26_11-36-52_fh_bh_uniform_peak43_swapped_normal_resume_m17900_userparams_16000env_31body/exported/policy.onnx \
  robot.control.playback_slowdown=4 \
  +sim.config.table_tennis.ball_initial_pos='[2.4,0.0,1.1]' \
  +sim.config.table_tennis.ball_initial_lin_vel='[-1.2,0.0,0.0]'
```

这属于改演示球路，不保证策略一定击得漂亮。

## 9. 离屏或非实时运行

如果只想跑链路、不看窗口：

```bash
cd /home/yhl/Desktop/Hitter/RobotBridge2/deploy

python run.py \
  --config-name=hitter \
  sim=mujoco \
  robot.control.viewer=false \
  robot.control.real_time=false \
  mimic.policy.checkpoint=../../MOSAIC-main/logs/rsl_rl/hitter_striking_g1/2026-06-26_11-36-52_fh_bh_uniform_peak43_swapped_normal_resume_m17900_userparams_16000env_31body/exported/policy.onnx
```

## 10. 当前目录对应关系

旧路径：

```text
/home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/HITTER-main
```

现在对应：

```text
/home/yhl/Desktop/Hitter/MOSAIC-main
```

旧路径：

```text
/home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/RobotBridge2
```

现在对应：

```text
/home/yhl/Desktop/Hitter/RobotBridge2
```

旧路径：

```text
/home/yhl/Desktop/Omega-Athlete-full
```

现在对应：

```text
/home/yhl/Desktop/Hitter
```

## 11. 踩坑和修复记录

### Git LFS 资产缺失

如果部分大文件是 Git LFS pointer，会导致 IsaacLab 资产加载失败。进入新根目录执行：

```bash
cd /home/yhl/Desktop/Hitter
git lfs pull
```

再用 pointer 检查命令确认没有残留小文件。

### MOSAIC-main 本地包未安装

如果出现：

```text
ModuleNotFoundError: No module named 'rsl_rl'
```

或 HITTER task 注册不到，通常是在 `MOSAIC-main` 下没有安装本地包：

```bash
cd /home/yhl/Desktop/Hitter/MOSAIC-main
python -m pip install -e source/rsl_rl
python -m pip install -e source/whole_body_tracking
```

### 缺少 USD cache 路径

`g1_hitter_racket/main.usda` 可能引用：

```bash
MOSAIC-main/cache/g1_usd_hitter/main.usd
```

当前目录已经存在：

```bash
/home/yhl/Desktop/Hitter/MOSAIC-main/cache/g1_usd_hitter
```

如果资产加载失败，优先检查这个 cache 目录是否仍然指向或包含实际 USD 资产。

### motion body index 不匹配

参考动作回放如果出现 body index 越界，原因通常是机器人 USD body ordering 和 motion npz ordering 不一致。

相关修复点在：

```bash
/home/yhl/Desktop/Hitter/MOSAIC-main/source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/commands.py
/home/yhl/Desktop/Hitter/MOSAIC-main/source/whole_body_tracking/whole_body_tracking/tasks/hitter/config/g1/flat_env_cfg.py
```

思路是区分 motion 侧 body names 和 robot 侧 body names，让 motion 侧索引按：

```text
pelvis + 29 actuated child links + right_racket_link
```

来对齐。

### play.py checkpoint 路径解析

如果显式传：

```bash
--resume_student_checkpoint <checkpoint.pt>
```

`play.py` 应该直接使用该 checkpoint 路径，而不是再去 `logs/rsl_rl/hitter_striking_g1` 自动查找。若再次出现找日志目录失败，优先检查 `scripts/rsl_rl/play.py` 的 checkpoint 解析逻辑。

### Hydra 不允许覆盖 playback_slowdown

如果使用：

```bash
robot.control.playback_slowdown=4
```

时报：

```text
Key 'playback_slowdown' is not in struct
```

检查：

```bash
/home/yhl/Desktop/Hitter/RobotBridge2/deploy/config/control/g1_hitter_racket.yaml
```

确保里面有：

```yaml
playback_slowdown: 1.0
```

之后就可以用 Hydra override 调慢播放。
