# HITTER 在 MOSAIC 框架上的训练流程

直白说，HITTER 是把 MOSAIC 的“全身动作跟踪训练框架”改造成“乒乓球击球策略训练框架”。

它不是从零写一个 RL 框架。它仍然使用：

```text
IsaacLab 仿真
whole_body_tracking 任务系统
RSL-RL PPO
motion npz 加载
policy checkpoint
ONNX 导出
ONNX metadata
```

但它把原来“跟踪一段 motion”的任务，改成了“参考正手/反手挥拍动作，同时学会根据击球目标控制 G1+球拍”。

## 1. 数据准备

HITTER 的训练数据已经打包好了，在：

```text
/home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/HITTER-main/motions
```

里面分成两类：

```text
motions/forehand/
motions/backhand/
```

目前有 9 段 `.npz`：

```text
4 段 forehand
5 段 backhand
```

这些 `.npz` 不是普通走路动作，而是已经处理好的 G1 击球参考动作。文件名里能看到关键信息：

```text
strike43
94f
50fps
unitree_g1
```

意思大致是：这段 motion 是 50Hz、94 帧左右，击球关键帧在第 43 帧，已经重定向到 Unitree G1。

所以 HITTER 的数据起点不是“原始视频”，而是：

```text
已经重定向好的 G1 forehand/backhand 击球 motion npz
```

## 2. 数据检查：先回放参考动作

训练前先不要跑 policy，只看 motion 本身对不对：

```bash
cd /home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/HITTER-main

python scripts/rsl_rl/play_reference_motion.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion motions \
  --num_envs=4
```

这一步的作用很简单：

```text
把 npz 里的身体姿态、关节角、速度直接写到 IsaacLab 机器人上
```

它不加载 checkpoint，也不让 policy 控制机器人。

如果这一步正常，说明：

```text
motion 文件能读
forehand/backhand 分组能读
G1+球拍资产能加载
right_racket_link 存在
motion body 顺序基本对齐
IsaacLab 环境能启动
```

如果这里都不对，后面训练肯定没意义。

## 3. HITTER 在 MOSAIC 上新增了什么任务

基础 MOSAIC 训练任务通常是：

```text
Tracking-Flat-G1-v0
```

意思是：给一段 motion，机器人尽量模仿。

HITTER 新增任务是：

```text
Hitter-Striking-PlannerDomain-Flat-G1-v0
```

注册位置是：

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
环境配置：怎么建仿真、机器人、观测、奖励、终止
PPO 配置：网络多大、训练多少步、学习率是多少
```

## 4. 机器人从普通 G1 变成 G1+球拍

基础 MOSAIC 用普通 G1。

HITTER 在训练配置里用的是：

```python
G1_HITTER_RACKET_CFG
```

并且把 anchor body 设成：

```text
pelvis
```

把击球末端设成：

```text
right_racket_link
```

直白说，HITTER 训练时真正关心的末端不是手腕，而是球拍。

它还单独定义了：

```text
G1_HITTER_BODY_NAMES
G1_HITTER_MOTION_BODY_NAMES
```

原因是：带球拍后的机器人 body 顺序和 motion npz 里的 body 顺序不完全一样，不能粗暴按 index 对齐。

更具体地说，这里有两个不同的问题：

```text
1. 数量不一样：训练奖励只挑一部分关键 body 做 imitation。
2. 顺序不一样：robot 模型里的 body index 和 npz 里的 body index 不能默认一一对应。
```

`G1_HITTER_MOTION_BODY_NAMES` 对应 motion npz 里保存的 body 顺序。它更完整，包含腿、腰、躯干、双臂、手腕，以及：

```text
right_racket_link
```

也就是说，motion 文件像一份完整的老师示范轨迹，里面保存了很多 body 的位置、姿态和速度。

但 `G1_HITTER_BODY_NAMES` 是训练奖励里实际拿来做 body imitation 的列表，数量更少，主要覆盖：

```text
pelvis
腰/躯干
双臂
双腕
```

它没有要求所有腿部 link、脚部 link、球拍 link 都逐帧模仿 motion。原因是 HITTER 的目标不是“整个人每个 link 都像参考动作”，而是“机器人稳定地完成击球”。

如果把 motion 里的所有 body 都强行逐帧模仿，policy 会被参考动作绑得太死。比如腿、脚、某些细小 link 一旦和参考 motion 有偏差，就会被惩罚；但打球任务里，机器人可能需要为了站稳、调整站位、追球拍目标而做一点自己的动作调整。

所以 HITTER 的奖励设计是分层的：

```text
部分 body imitation：让动作有参考风格，不要乱挥。
击球任务 reward：让球拍在击球时刻到正确位置、速度、方向。
稳定性 reward/penalty：让身体别塌、脚别滑、关节别爆。
```

HITTER 真正权重很大的奖励是：

```text
racket_pos：球拍到没到击球点
racket_vel：球拍速度对不对
racket_ori：拍面方向对不对
base_pos：身体站位对不对
```

因此可以这么理解：

```text
motion 是完整老师示范录像；
训练奖励不是逐像素照抄老师；
训练奖励只检查关键身体姿态 + 击球结果 + 稳定性。
```

顺序不同则是另一个工程问题。motion npz 里的 `body_pos_w[:, i]` 第 `i` 个 body，不一定等于 IsaacLab robot 模型里的第 `i` 个 body。如果直接按 index 取，就可能出现：

```text
拿机器人的 right_hip_pitch_link 去对齐 motion 里的 left_hip_roll_link
```

这种错位会让 reward 变乱，严重时还会 body index 越界。

所以 HITTER 通过 `motion_body_names` 做名字映射：

```text
训练奖励关心哪些 body
-> 去 motion_body_names 里按名字查它们在 npz 中的 index
-> 去 robot 模型里按名字查它们在 IsaacLab 中的 index
-> 两边按 body name 对齐，而不是按 index 猜
```

一句话总结：

```text
body 数量不同，是因为奖励只模仿关键 body；
body 顺序不同，是因为 npz 和 robot 模型的内部 body index 不一致；
HITTER 用 body name 显式映射来避免错位。
```

## 5. motion 数据怎么进入训练

训练命令里必须传：

```bash
--motion motions
```

脚本会把它写进：

```python
env_cfg.commands.motion.motion = args_cli.motion
```

也就是说，`motions` 目录会交给 HITTER 的 command 模块加载。

HITTER 的 command 类是：

```text
HitterStrikingCommand
```

它继承自 MOSAIC 的：

```text
MultiMotionCommand
```

所以它仍然保留“加载多个 motion、从 motion 里取参考身体姿态”的能力。

但 HITTER 在这个基础上又加了击球相关变量：

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
HITTER command：参考动作是正手/反手挥拍，同时你要在这个时间把球拍送到这个点、这个速度、这个方向。
```

## 6. 正手/反手怎么采样

HITTER 默认设置：

```python
command.motion_group_sampling_ratios = {"forehand": 0.5, "backhand": 0.5}
```

也就是训练时一半环境练正手，一半环境练反手。

也可以强制只练一种，比如只练正手：

```bash
python scripts/rsl_rl/train.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion motions \
  --force_strike_type forehand
```

或者控制采样比例：

```bash
python scripts/rsl_rl/train.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion motions \
  --motion_group_sampling_ratios forehand=1.0,backhand=0.0
```

## 7. 训练时 policy 看到什么

HITTER 的 actor observation 是 105 维。

它不是 MOSAIC 那种 800 维历史 motion tracking observation。

HITTER actor 主要看到：

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

直白说，policy 每一步看到的是：

```text
我现在身体状态怎么样；
我现在面朝哪里；
身体目标在哪；
球拍目标在哪；
球拍应该多快；
离击球还有多久；
当前关节状态是什么；
上一步动作是什么。
```

这里要把 observation 拆成两类来源：

```text
任务目标 command：
base_target_pos
racket_target_pos
racket_target_vel
time_to_strike

机器人自身状态：
base_ang_vel
projected_gravity
base_forward_xy
joint_pos
joint_vel
last_action
```

训练时随机采样的是任务目标 command，不是机器人自身状态。机器人自身状态来自当前 IsaacLab 仿真里的机器人；部署时则来自 RobotBridge2 的 MuJoCo 或真机接口。

然后输出：

```text
29 维 action
```

对应 G1 的 29 个控制关节目标。

## 8. 训练时 critic 看到更多信息

HITTER 是非对称 actor-critic。

actor 是以后部署要用的，所以观测比较克制，只给 105 维任务输入。

critic 训练时可以看到更多 privileged 信息，比如：

```text
motion_anchor_pos_b
motion_anchor_ori_b
body_pose
time_left
reference_joint_state
base_lin_vel
```

直白说：

```text
actor 学实际能部署的控制；
critic 训练时多看一点答案，帮助 PPO 学得更稳。
```

## 9. 奖励怎么设计

基础 MOSAIC 奖励主要是：

```text
机器人越像参考 motion，奖励越高
```

HITTER 仍然保留了一部分 motion imitation 奖励，让动作不要完全乱掉。

但 HITTER 真正的大头奖励是击球目标：

```text
base_pos：骨盆/身体有没有到目标站位
racket_pos：球拍有没有到目标击球点
racket_vel：球拍速度对不对
racket_ori：拍面方向对不对
```

其中球拍位置、速度、方向权重很高。

直白说：

```text
参考动作只是老师示范；
真正考试是击球那一下球拍有没有到位。
```

它还加了稳定性惩罚：

```text
脚滑惩罚
击球时支撑不稳惩罚
躯干过度前倾惩罚
base 角速度惩罚
关节限位惩罚
动作变化过快惩罚
力矩惩罚
```

所以训练目标是：

```text
挥拍要像；
球拍要准；
速度要对；
身体不能塌；
脚不能乱滑。
```

## 10. 训练命令

基础训练命令是：

```bash
conda activate isaaclab

cd /home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/HITTER-main

python -m pip install -e source/rsl_rl
python -m pip install -e source/whole_body_tracking

python scripts/rsl_rl/train.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion motions
```

如果想先小规模 smoke test：

```bash
python scripts/rsl_rl/train.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion motions \
  --num_envs=256 \
  --headless \
  --max_iterations 10
```

正式配置里默认是：

```text
num_envs = 4096
num_steps_per_env = 24
max_iterations = 200000
save_interval = 100
experiment_name = hitter_striking_g1
```

训练日志会写到：

```text
logs/rsl_rl/hitter_striking_g1/<timestamp>/
```

里面会有：

```text
model_*.pt
params/env.yaml
params/agent.yaml
```

## 11. 已有 checkpoint 不需要重新训练也能验证

仓库已经带了一个训练好的包：

```text
checkpoints/hitter_m20400/model_20400.pt
```

所以复现部署时可以直接用它，不必从零训练。

播放并导出 ONNX：

```bash
cd /home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/HITTER-main

python scripts/rsl_rl/play.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion motions \
  --resume_student_checkpoint checkpoints/hitter_m20400/model_20400.pt \
  --disable_obs_noise \
  --skip_critic
```

这一步会导出或刷新：

```text
checkpoints/hitter_m20400/exported/policy.onnx
```

## 12. 为什么 play.py 还要传 motion

因为 HITTER 的 policy 虽然已经训练好了，但 IsaacLab 里 playback/evaluation 仍然需要构建同一个 HITTER 环境。

这个环境里的 command 依赖 motion 数据来生成：

```text
正手/反手类型
参考相位
racket target
base target
time_to_strike
```

所以 `--motion motions` 不能省。

## 13. 导出的 ONNX 为什么能给 RobotBridge2 用

`play.py` 不只是导出网络本体，还会调用 metadata 写入逻辑。

RobotBridge2 依赖 ONNX metadata 读取：

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

这样部署端才能知道：

```text
29 维 action 分别对应哪些关节；
默认关节角是多少；
action_scale 怎么还原成 PD target；
PD stiffness/damping 用什么；
anchor body 是 pelvis。
```

检查 ONNX 维度：

```bash
python - <<'PY'
import onnx
m = onnx.load("checkpoints/hitter_m20400/exported/policy.onnx")
for x in m.graph.input:
    print("input:", x.name, [d.dim_value for d in x.type.tensor_type.shape.dim])
for y in m.graph.output:
    print("output:", y.name, [d.dim_value for d in y.type.tensor_type.shape.dim])
PY
```

期望是：

```text
input: obs [1, 105]
output: actions [1, 29]
```

## 14. 最后接到 MuJoCo

训练侧产物是：

```text
policy.onnx
```

部署命令是：

```bash
cd /home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/RobotBridge2/deploy

python run.py \
  --config-name=hitter \
  sim=mujoco \
  mimic.policy.checkpoint=../../HITTER-main/checkpoints/hitter_m20400/exported/policy.onnx
```

这里和训练不一样：部署时不再直接用 `motions` 逐帧驱动。

MuJoCo 部署时 command 来自：

```text
球状态 -> ball planner -> base/racket/time_to_strike command
```

而不是：

```text
motion npz -> reference frame command
```

所以训练时 motion 是老师，部署时 planner 是出题人。

这里需要明确一个容易混淆的点：HITTER 训练时本来没有部署侧这个“真实球路 planner”。训练阶段不是从 MuJoCo 里的真实乒乓球状态去预测球路，而是由 IsaacLab 里的 `HitterStrikingCommand` 随机采样任务目标 command：

```text
base_target_pos
racket_target_pos
racket_target_vel
time_to_strike
```

注意，随机采样的是上面这些任务目标，不是机器人自身状态。机器人自身状态，例如 `base_ang_vel`、`projected_gravity`、`joint_pos`、`joint_vel`、`last_action`，都是从当前仿真里的机器人实时读出来的。

也就是说，训练时 policy 学到的不是“直接看球的位置和速度，然后自己决定怎么打”的端到端能力。它学到的是：给我身体目标、球拍目标、球拍速度目标、击球时间，以及当前机器人状态，我输出 29DoF 全身动作去完成这一拍。

所以部署时真正必须对齐的不是“必须有某个复杂球路 planner”，而是必须有人生成和训练时语义一致的 HITTER command：

```text
base_target / racket_target / racket_vel / time_to_strike
```

RobotBridge2 当前选择用 `ball planner` 从 MuJoCo 球状态生成这些 command。这是部署侧的一种实现方式，不是训练时存在的模块。

因此可以这样理解：

```text
训练时：随机 command sampler 生成击球目标，IsaacLab 实时读取机器人自身状态。
部署时：需要 command generator 生成同语义击球目标，RobotBridge2/MuJoCo 或真机接口实时读取机器人自身状态。
RobotBridge2 默认 command generator：ball planner。
```

如果只是想在 MuJoCo 里看连续多次击球演示，不一定必须追求非常真实、复杂的球路规划；也可以用更简单、更可控的固定球路或固定 command 序列。但无论如何，policy 输入里的 `base_target_pos / racket_target_pos / racket_target_vel / time_to_strike` 这些目标都必须被生成出来。

### 14.1 MuJoCo 中为什么会一闪而过

默认 HITTER MuJoCo 演示更像“单次击球验证器”，不是完整连续对打系统。它主要验证：

```text
球状态 -> ball planner 生成击球目标 -> policy 挥拍 -> 球被打出去
```

如果启动日志里看到类似：

```text
Armed HITTER command from ball ... tts=0.000s
```

说明 planner 一启动就认为“现在已经是击球时刻”。这时画面里会看到机器人很快挥一下，然后就没有明显连续来球过程。

`tts` 是 `time to strike`，也就是距离击球还剩多久。`tts=0.000s` 的直白含义是：球已经在击球平面附近，policy 没有准备和来球观察时间。

`robot.control.playback_slowdown` 只能让 MuJoCo 按墙钟慢放，它不会改变 planner 算出来的 `time_to_strike`。真正决定是否一闪而过的是：

```text
球初始位置 ball_initial_pos
球初始速度 ball_initial_lin_vel
mimic.motion.ball_planner.prediction_horizon_s
mimic.motion.ball_planner.virtual_hit_plane_x
```

### 14.2 单次来球慢放观察命令

如果只是想把一拍击球看清楚，可以把球放得稍微近一点、速度设得合理一点，同时增大 planner 的预测窗口：

```bash
cd /home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/RobotBridge2/deploy

python run.py \
  --config-name=hitter \
  sim=mujoco \
  mimic.policy.checkpoint=../../HITTER-main/checkpoints/hitter_m20400/exported/policy.onnx \
  robot.control.playback_slowdown=6 \
  mimic.motion.ball_planner.prediction_horizon_s=4.0 \
  +sim.config.table_tennis.enabled=true \
  +sim.config.table_tennis.ball_initial_pos='[1.8,0.0,1.1]' \
  +sim.config.table_tennis.ball_initial_lin_vel='[-1.6,0.0,0.0]' \
  +sim.config.table_tennis.use_analytic_table_bounce=true \
  +sim.config.table_tennis.use_analytic_racket_hit=true
```

这里建议不要用太慢太远的球路，例如：

```text
ball_initial_pos='[2.4,0.0,1.1]'
ball_initial_lin_vel='[-1.2,0.0,0.0]'
```

这组参数从 `x=2.4` 飞到击球平面 `x=0.0` 理论上就要约 `2.0s`，很贴近默认 `prediction_horizon_s=2.0` 的边界；考虑阻力和离散步长后，planner 容易报：

```text
ball trajectory does not cross hit plane x=0.000 within prediction horizon
```

更稳的做法是：

```text
球更近：x = 1.8
球更快：vx = -1.6
预测更长：prediction_horizon_s = 4.0
```

### 14.3 连续击球观察命令

HITTER 的 `HitterEnv` 里已经有 command 结束后重新 reset ball、再 sample 新 command 的逻辑。MuJoCo simulator 也支持：

```text
randomize_ball_on_reset
ball_trajectory_candidates
ball_trajectory_selection
```

所以可以给它一组球路，让它每次 reset ball 时按顺序发下一球。命令行里复杂 list/dict 必须用 Hydra override 语法，不要写 JSON 风格的双引号字典。

正确写法示例：

```bash
cd /home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/RobotBridge2/deploy

python run.py \
  --config-name=hitter \
  sim=mujoco \
  mimic.policy.checkpoint=../../HITTER-main/checkpoints/hitter_m20400/exported/policy.onnx \
  robot.control.playback_slowdown=4 \
  mimic.motion.ball_planner.prediction_horizon_s=4.0 \
  +sim.config.table_tennis.enabled=true \
  +sim.config.table_tennis.randomize_ball_on_reset=true \
  +sim.config.table_tennis.ball_trajectory_selection=sequence \
  '+sim.config.table_tennis.ball_trajectory_candidates=[{name:center_slow,pos:[1.8,0.0,1.1],lin_vel:[-1.6,0.0,0.0]},{name:left_slow,pos:[1.8,0.25,1.1],lin_vel:[-1.6,-0.05,0.0]},{name:right_slow,pos:[1.8,-0.25,1.1],lin_vel:[-1.6,0.05,0.0]}]' \
  +sim.config.table_tennis.use_analytic_table_bounce=true \
  +sim.config.table_tennis.use_analytic_racket_hit=true
```

注意这里的 list/dict 是 Hydra 风格：

```text
+sim.config.table_tennis.ball_trajectory_candidates=[{name:center_slow,pos:[1.8,0.0,1.1],lin_vel:[-1.6,0.0,0.0]}]
```

不要写成 JSON 风格：

```text
+sim.config.table_tennis.ball_trajectory_candidates=[{"name":"center_slow","pos":[1.8,0.0,1.1],"lin_vel":[-1.6,0.0,0.0]}]
```

否则 Hydra 可能报：

```text
LexerNoViableAltException
```

### 14.4 更推荐：写进 mujoco.yaml

命令行传复杂 `ball_trajectory_candidates` 很容易被 shell/Hydra 解析坑到。长期建议把连续击球球路写进：

```text
/home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/RobotBridge2/deploy/config/sim/mujoco.yaml
```

可以在 `config:` 下面加入：

```yaml
table_tennis:
  enabled: true
  randomize_ball_on_reset: true
  ball_trajectory_selection: sequence
  ball_initial_ang_vel: [0.0, 0.0, 0.0]
  use_analytic_table_bounce: true
  use_analytic_racket_hit: true
  ball_trajectory_candidates:
    - name: center_slow
      pos: [1.8, 0.0, 1.1]
      lin_vel: [-1.6, 0.0, 0.0]
    - name: left_slow
      pos: [1.8, 0.25, 1.1]
      lin_vel: [-1.6, -0.05, 0.0]
    - name: right_slow
      pos: [1.8, -0.25, 1.1]
      lin_vel: [-1.6, 0.05, 0.0]
```

然后运行命令可以简化成：

```bash
python run.py \
  --config-name=hitter \
  sim=mujoco \
  mimic.policy.checkpoint=../../HITTER-main/checkpoints/hitter_m20400/exported/policy.onnx \
  robot.control.playback_slowdown=4 \
  mimic.motion.ball_planner.prediction_horizon_s=4.0
```

### 14.5 当前连续击球的真实含义

这里的“连续击球”不是完整乒乓球 rally，也不是对面真实连续发球。它更准确地说是：

```text
来球 1 -> planner 生成击球目标 -> 机器人击球
reset 下一条球路
来球 2 -> planner 生成击球目标 -> 机器人击球
reset 下一条球路
```

也就是连续播放多次单拍击球验证。当前系统重点验证的是：

```text
球状态 -> planner -> 105 维 HITTER observation -> ONNX policy -> G1+球拍挥拍
```

不是完整比赛回合系统。

## 总结

一句话总结：

```text
HITTER 在 MOSAIC 上训练时，用 forehand/backhand motion 教机器人学会稳定挥拍和击球末端控制；
训练出的 policy 不只是模仿动作，而是学会根据 base_target、racket_target、racket_velocity、time_to_strike 这些目标条件输出 29DoF 全身动作；
最后导出 ONNX，交给 RobotBridge2，在 MuJoCo 里由 ball planner 根据球状态实时生成这些目标条件，让机器人完成击球。
```
