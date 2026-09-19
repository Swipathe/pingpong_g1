# HITTER 相比基础 MOSAIC + RobotBridge 的添加和修改

本文档说明：

```text
/home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER
```

中的 HITTER 乒乓球 MuJoCo 仿真，相比：

```text
/home/yhl/Desktop/BAAI-Humanoid
```

里的基础 MOSAIC 训练框架和 RobotBridge 部署框架，针对“机器人打乒乓球”任务做了哪些添加和修改。

## 1. 总体区别

基础 MOSAIC + RobotBridge 的核心目标是：

```text
motion npz
-> MOSAIC 训练 whole-body motion tracking policy
-> 导出 policy.onnx
-> RobotBridge 加载 policy
-> MuJoCo 或真机中跟踪参考动作
```

HITTER 的核心目标是：

```text
forehand/backhand 击球参考动作 + 乒乓球状态
-> HITTER IsaacLab/RSL-RL 训练 striking policy
-> 导出 policy.onnx
-> RobotBridge2 读取 ONNX metadata
-> ball planner 生成击球 command
-> MuJoCo 中 G1 持拍完成击球
```

所以，HITTER 不是简单换一个 motion 文件，而是在 MOSAIC 的训练框架上增加了乒乓球击球任务，在 RobotBridge 的部署框架上增加了持拍机器人、球状态、球路预测、击球规划和 hitter 专用运行路径。

一句话概括：

```text
基础 MOSAIC + RobotBridge：通用全身动作跟踪和部署。
HITTER：任务化的乒乓球击球训练和 MuJoCo 部署。
```

## 2. 训练侧：从通用 tracking 变成 HITTER striking task

基础 MOSAIC 的代表任务是：

```text
Tracking-Flat-G1-v0
```

它主要训练 G1 做 whole-body motion tracking，也就是让机器人跟踪给定 motion 数据。

HITTER 新增的任务是：

```text
Hitter-Striking-PlannerDomain-Flat-G1-v0
```

对应训练侧目录：

```text
/home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/HITTER-main
```

新增或重点使用的任务代码集中在：

```text
source/whole_body_tracking/whole_body_tracking/tasks/hitter/
```

主要包括：

```text
hitter_env_cfg.py
config/g1/flat_env_cfg.py
mdp/commands.py
mdp/observations.py
mdp/rewards.py
mdp/terminations.py
```

这些文件把普通 motion tracking 扩展为面向乒乓球击球的 MDP，增加了击球任务需要的 command、observation、reward 和 termination 逻辑。

### 2.1 HITTER 不是直接复用 tracking task，而是另写 hitter task

直白地说，HITTER 的训练入口已经不是基础 MOSAIC 的通用 tracking task：

```text
tasks/tracking/
```

而是新增了一套击球任务：

```text
tasks/hitter/
```

不过它不是从零重写机器人强化学习框架。HITTER 仍然复用 MOSAIC/whole_body_tracking/RSL-RL 的底层基础设施，并且在：

```text
tasks/hitter/mdp/__init__.py
```

中导入了通用 tracking 的 MDP 函数：

```python
from whole_body_tracking.tasks.tracking.mdp import *
from .commands import *
from .observations import *
from .rewards import *
from .terminations import *
```

所以更准确的理解是：

```text
HITTER 没有直接继续跑原来的 Tracking-Flat-G1-v0。
它另写了 Hitter-Striking-PlannerDomain-Flat-G1-v0。
底层 motion loading、RSL-RL、IsaacLab manager 机制等仍然复用。
但是 command、observation、reward、termination 的任务语义已经换成了击球。
```

也就是说：

```text
tracking：通用 motion imitation。
hitter：target-conditioned striking policy training。
```

### 2.2 env_cfg 的区别

基础 tracking 的配置主要在：

```text
tasks/tracking/tracking_env_cfg.py
tasks/tracking/config/g1/flat_env_cfg.py
```

它定义的是“跟踪参考 motion”的环境：

```text
场景：普通 G1
命令：MotionCommand 或 MultiMotionCommand
观测：参考 motion + 当前机器人状态
奖励：机器人越像参考 motion 越好
终止：偏离参考 motion 太远就结束
```

HITTER 的配置主要在：

```text
tasks/hitter/hitter_env_cfg.py
tasks/hitter/config/g1/flat_env_cfg.py
```

它定义的是“击球任务环境”：

```text
场景：G1 + racket
命令：HitterStrikingCommand
观测：base 目标 + racket 目标 + 击球时间 + 当前机器人状态
奖励：既要像参考动作，又要在击球窗口把球拍送到正确位置、速度和朝向
终止：主要看身体高度/姿态是否崩掉
```

基础 tracking 里普通 G1 的典型设置是：

```python
self.scene.robot = G1_CYLINDER_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
self.commands.motion.anchor_body_name = "torso_link"
```

HITTER 中变成：

```python
self.scene.robot = G1_HITTER_RACKET_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
self.commands.motion.anchor_body_name = "pelvis"
self.commands.motion.racket_body_name = "right_racket_link"
```

这个变化很关键。tracking 的主要参考 anchor 是 `torso_link`，HITTER 改成 `pelvis`，并额外指定 `right_racket_link` 作为击球任务末端。

直白理解：

```text
tracking 的环境问：机器人整体像不像参考 motion？
HITTER 的环境问：机器人能不能在稳定站位下，把球拍末端送到正确击球目标？
```

### 2.3 commands 的区别

基础 tracking 的 command 本质是：

```text
给机器人一段参考 motion，
告诉它当前时刻参考 motion 的身体位置、姿态、速度、关节角，
让机器人跟着做。
```

对应类是：

```text
MotionCommand
MultiMotionCommand
```

它主要关心：

```text
anchor_pos
anchor_ori
body_pos
body_ori
body_lin_vel
body_ang_vel
joint_pos
motion phase
```

HITTER 的 command 是：

```text
HitterStrikingCommand(MultiMotionCommand)
```

它继承 `MultiMotionCommand`，所以仍然能加载多段正手/反手 motion。但它额外维护击球任务变量：

```text
strike_type                 正手/反手
strike_time_s               击球时刻
time_to_strike_s            距离击球还有多久
base_target_pos             身体/骨盆应该去哪里
racket_target_pos           球拍应该到哪里
racket_target_vel           球拍应该以什么速度击球
racket_normal               当前球拍法向
racket_strike_face_normal   当前击球面法向
```

HITTER command 最核心的输出可以理解为：

```text
base_target_pos_b
racket_target_pos_b
racket_target_vel_w
time_to_strike_s
```

所以两者的区别是：

```text
tracking command：你现在应该模仿参考动作的这一帧。
HITTER command：你不但要参考挥拍动作，还要在指定时间把球拍送到指定击球点，并给出指定速度和拍面方向。
```

### 2.4 observations 的区别

基础 tracking 的 policy observation 主要是：

```text
command
motion_anchor_pos_b
motion_anchor_ori_b
base_lin_vel
base_ang_vel
joint_pos
joint_vel
actions
history_length = 5
```

它表达的是：

```text
参考动作现在在哪里？
机器人现在在哪里？
机器人和参考动作之间差多少？
```

HITTER 的 policy observation 主要是：

```text
base_ang_vel
projected_gravity
base_forward_xy
base_target_pos
racket_target_pos
racket_target_vel
time_to_strike
joint_pos
joint_vel
actions
```

新增的核心项是：

```text
base_target_pos
racket_target_pos
racket_target_vel
time_to_strike
```

这些直接对应击球任务。它告诉 policy：

```text
身体该站到哪里？
球拍该去哪里？
球拍该以多快速度挥过去？
离击球还剩多久？
```

所以两者的区别是：

```text
tracking observation：参考人在这，机器人在这，你去追。
HITTER observation：你身体该站哪，球拍该去哪，球拍该多快，离击球还剩多久，你安排全身动作。
```

HITTER 的 critic 还会看到更多 privileged 信息，例如：

```text
motion_anchor_pos_b
motion_anchor_ori_b
body_pose
time_left
reference_joint_state
```

这表示训练时 critic 可以利用更多参考动作和身体状态信息；部署时 actor 使用的是更任务化、更紧凑的 policy observation。

### 2.5 rewards 的区别

基础 tracking 的奖励主要是“像不像参考 motion”：

```text
motion_global_anchor_pos
motion_global_anchor_ori
motion_body_pos
motion_body_ori
motion_body_lin_vel
motion_body_ang_vel
motion_anchor_lin_vel
```

再加通用正则：

```text
undesired_contacts
action_rate_l2
joint_limit
joint_acc
joint_torque
```

直白地说：

```text
骨盆/躯干位置像参考，给分。
身体各 link 位置姿态像参考，给分。
速度像参考，给分。
动作平滑、不乱撞、不超关节限位，少扣分。
```

HITTER 仍然保留了一部分 motion imitation 奖励：

```text
motion_global_anchor_pos
motion_global_anchor_ori
motion_body_pos
motion_body_ori
motion_body_lin_vel
motion_body_ang_vel
motion_anchor_lin_vel
```

但真正针对乒乓球任务新增的是：

```text
hitter_base_position_error_exp
hitter_racket_position_error_exp
hitter_racket_velocity_error_exp
hitter_racket_orientation_error_exp
```

它们分别对应：

```text
base_pos：身体/骨盆到目标站位了吗？
racket_pos：击球窗口时球拍到目标点了吗？
racket_vel：球拍速度对了吗？
racket_ori：拍面方向对了吗？
```

HITTER 中球拍相关奖励权重很大，例如：

```text
base_pos weight = 30
racket_pos weight = 150
racket_vel weight = 120
racket_ori weight = 50
```

这说明训练目标已经明显从“全身像参考动作”转向：

```text
为了击球，球拍末端必须对。
```

HITTER 还新增了一批稳定性和支撑相关惩罚：

```text
hitter_torso_forward_lean_l2
hitter_base_ang_vel_xy_l2
hitter_foot_slip_l2
hitter_hit_unstable_support
hitter_foot_edge_drag
hip_yaw_default
```

这些用于防止机器人为了挥拍乱扭、脚滑、击球时单脚乱飞、躯干过度翻滚。

所以两者的奖励逻辑可以概括为：

```text
tracking reward：动作像不像。
HITTER reward：动作要像，但更重要的是击球那一瞬间球拍位置、速度、朝向要对，同时身体不能塌、脚不能乱滑。
```

### 2.6 terminations 的区别

基础 tracking 的终止条件比较“严格跟踪参考”：

```text
motion_end
time_out
anchor_pos
anchor_ori
ee_body_pos
```

含义是：

```text
motion 播完了，结束。
时间到了，结束。
anchor 高度偏太多，结束。
anchor 姿态偏太多，结束。
手脚等关键 body 偏离参考太多，结束。
```

也就是说，tracking 会因为“你不像参考动作”而结束。

HITTER 的 termination 更少、更任务化：

```text
time_out
anchor_pos = hitter_bad_anchor_height
anchor_ori = bad_anchor_ori
base_bad_orientation = None
base_height = None
```

其中 `hitter_bad_anchor_height` 的逻辑是：

```text
如果 pelvis/root 高度离目标高度 0.793 太远，超过 0.35，就终止。
```

HITTER 没有强行保留 `motion_end` 和 `bad_motion_body_pos_z_only` 这类“身体 link 偏离参考动作太远就结束”的终止项。原因是 HITTER 不希望 policy 被逐帧 motion tracking 绑死，而是要给它空间完成击球目标。

直白理解：

```text
tracking：你不像参考动作了，结束。
HITTER：你可以为了击球调整动作，但不能摔得太离谱。
```

### 2.7 body names 和 motion body names 的区别

基础 G1 tracking 通常只跟踪较少关键身体：

```text
pelvis
left/right hip_roll
left/right knee
left/right ankle_roll
torso
left/right shoulder_roll
left/right elbow
left/right wrist_yaw
```

HITTER 分成了两个列表：

```text
G1_HITTER_BODY_NAMES
G1_HITTER_MOTION_BODY_NAMES
```

`G1_HITTER_BODY_NAMES` 更偏训练奖励和身体控制，主要覆盖：

```text
pelvis
waist
torso
left/right shoulders
left/right elbows
left/right wrists
```

`G1_HITTER_MOTION_BODY_NAMES` 则对应 motion npz 的 body ordering，包含更完整的 31 个 body：

```text
pelvis
腿部 links
腰/躯干
双臂 links
right_racket_link
```

这个分开很重要。HITTER 因为多了 racket，而且 USD/body ordering 更复杂，所以不能简单假设 motion 文件里的 body 顺序和机器人模型里的 body 顺序完全一致。

直白说：

```text
tracking 默认更接近“motion body 和 robot body 比较一致”。
HITTER 必须单独对齐 motion body names，否则容易出现 body index 越界或参考动作错位。
```

### 2.8 scene 和随机化的区别

基础 tracking 的场景更偏通用鲁棒跟踪训练，地形配置里包含：

```text
flat
slightly_rough
```

摩擦和 restitution 随机范围也比较大：

```text
static_friction_range: 0.3 - 1.6
dynamic_friction_range: 0.3 - 1.2
restitution_range: 0.0 - 0.5
```

HITTER 使用更专门的平地击球训练场景：

```text
HitterFlatSceneCfg
terrain_type = plane
```

事件随机化也更保守：

```text
static_friction_range: 1.0 - 1.0
dynamic_friction_range: 1.0 - 1.0
restitution_range: 0.0 - 0.0
```

直白说：

```text
tracking 更像通用鲁棒动作跟踪训练。
HITTER 当前更关注击球动作本身，不希望地面随机性过大干扰挥拍和站位学习。
```

### 2.9 训练侧核心区别总结

最直白的区别是：

```text
原来的 tracking：
给一段 motion，机器人整个人尽量像那段 motion。

HITTER：
给正手/反手参考动作，再给击球目标；
机器人不只是“像人挥拍”，而是要在指定击球时刻，
让 right_racket_link 到指定位置、指定速度、指定拍面方向，
同时身体站位和稳定性不能崩。
```

所以 HITTER 的 `tasks/hitter/` 可以理解成：

```text
把通用 motion tracking 改造成了 target-conditioned striking policy training。
```

## 3. 参考动作：从普通 motion 变成 forehand/backhand 击球 motion

基础 MOSAIC 通常使用普通全身 motion 数据，例如：

```text
MOSAIC_Dataset/G1/optical_mocap/*.npz
```

HITTER 使用打包好的正手和反手击球参考动作：

```text
HITTER-main/motions
```

可通过以下命令只回放参考动作，验证 motion、G1+球拍资产和 IsaacLab 是否正常：

```bash
cd /home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/HITTER-main

python scripts/rsl_rl/play_reference_motion.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion motions \
  --num_envs=4
```

这里的参考动作不只是为了“看起来像挥拍”，而是服务于训练击球策略，让机器人学到正手/反手挥拍、身体协同和球拍末端运动。

## 4. 机器人模型：从普通 G1 变成 G1 + racket

基础 MOSAIC/RobotBridge 常用普通 G1 29DoF：

```text
g1_29dof
```

HITTER 使用带球拍的 G1：

```text
g1_hitter_racket
```

RobotBridge2 里新增了对应配置：

```text
RobotBridge2/deploy/config/asset/g1_hitter_racket.yaml
RobotBridge2/deploy/config/control/g1_hitter_racket.yaml
RobotBridge2/deploy/config/robot/g1_hitter_racket.yaml
```

训练侧也使用 G1 racket 相关 USD 资产。motion body names 中额外考虑：

```text
right_racket_link
```

因此，HITTER 的关键末端不是普通手腕或手部 link，而是球拍 link。击球任务需要让这个球拍 link 在正确时间、正确空间位置和正确方向接触乒乓球。

## 5. Policy 输入输出：observation 结构不同

基础 MOSAIC smoke test 中导出的 ONNX 形状曾是：

```text
input: obs [1, 800]
output: actions [1, 29]
```

其中 800 维 observation 主要来自：

```text
motion command
motion anchor pose
base velocity
joint position
joint velocity
action history
```

HITTER 打包 checkpoint 导出的 ONNX 形状是：

```text
input: obs [1, 105]
output: actions [1, 29]
```

这说明 HITTER 没有直接照搬基础 MOSAIC 的 800 维历史 motion observation，而是使用面向击球部署的 105 维 observation。其语义围绕：

```text
机器人当前状态
球状态
planner 生成的击球 command
球拍/身体相关目标
```

动作输出仍然是：

```text
29
```

对应 G1 HITTER 的 29 个控制关节。

## 6. Command 来源：从 motion 文件驱动变成 ball planner 驱动

基础 RobotBridge 的 mosaic 模式通常这样运行：

```bash
cd /home/yhl/Desktop/BAAI-Humanoid/RobotBridge/deploy

python run.py \
  --config-name=mosaic \
  sim=mujoco \
  device=cpu \
  robot.control.use_teleop=false \
  mimic.motion.motion_path=../../MOSAIC_Dataset/G1/optical_mocap/g1_W4_stageii.npz \
  mimic.policy.checkpoint=../../MOSAIC/logs/rsl_rl/.../exported/policy.onnx
```

此时 command 主要来自：

```text
mimic.motion.motion_path 指定的 npz reference motion
```

HITTER 的 RobotBridge2 模式通常这样运行：

```bash
cd /home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/RobotBridge2/deploy

python run.py \
  --config-name=hitter \
  sim=mujoco \
  mimic.policy.checkpoint=../../HITTER-main/checkpoints/hitter_m20400/exported/policy.onnx
```

HITTER 部署时，command 不是简单从单个 motion npz 逐帧读取，而是由 hitter 环境和 ball planner 根据球状态生成：

```text
读取或模拟球状态
-> 预测球路
-> 判断击球时机
-> 选择正手/反手等击球类型
-> 生成 policy 所需 command
-> policy 输出 29DoF action
```

对应新增核心文件：

```text
RobotBridge2/deploy/utils/hitter_planner.py
RobotBridge2/deploy/envs/hitter.py
RobotBridge2/deploy/agents/hitter_agent.py
```

## 7. RobotBridge2 新增 hitter 专用配置

基础 RobotBridge 里主要有 mosaic 相关部署路径：

```text
deploy/agents/mosaic_agent.py
deploy/envs/mosaic.py
deploy/config/mosaic.yaml
deploy/config/mimic/mosaic.yaml
deploy/config/obs/mosaic.yaml
```

HITTER 的 RobotBridge2 新增了一整套 hitter 配置和模块：

```text
deploy/agents/hitter_agent.py
deploy/envs/hitter.py
deploy/utils/hitter_planner.py
deploy/config/hitter.yaml
deploy/config/agent/hitter.yaml
deploy/config/env/hitter.yaml
deploy/config/mimic/hitter.yaml
deploy/config/obs/hitter.yaml
deploy/config/asset/g1_hitter_racket.yaml
deploy/config/control/g1_hitter_racket.yaml
deploy/config/robot/g1_hitter_racket.yaml
```

`deploy/config/hitter.yaml` 使用的默认组合是：

```yaml
defaults:
  - robot: g1_hitter_racket
  - obs: hitter
  - sim: real_world
  - env: hitter
  - agent: hitter
  - mimic: hitter
  - _self_

device: cpu
```

运行 MuJoCo 时通过命令行覆盖：

```bash
sim=mujoco
```

### 7.1 RobotBridge2 的核心变化：从 motion deployment 变成 table-tennis striking runtime

直白地说，HITTER 的 RobotBridge2 不是“基础 RobotBridge 换一个 ONNX”这么简单，而是新增了一条专门服务乒乓球击球的 runtime 路径。

基础 RobotBridge 的 `mosaic` 模式做的是：

```text
读 motion npz
-> 拼 MOSAIC tracking observation
-> ONNX policy 输出 29 维动作
-> MuJoCo 里让机器人跟踪参考动作
```

HITTER 的 RobotBridge2 做的是：

```text
读/模拟乒乓球状态
-> ball planner 预测球路和击球点
-> 生成 base/racket/time_to_strike command
-> 拼 HITTER 105 维 observation
-> ONNX policy 输出 29 维动作
-> MuJoCo 里 G1 持拍击球
```

所以两者的核心区别是：

```text
基础 RobotBridge：motion deployment bridge。
HITTER RobotBridge2：table-tennis striking runtime。
```

### 7.2 新增 HitterAgent

基础 RobotBridge 有：

```text
deploy/agents/mosaic_agent.py
```

HITTER 新增：

```text
deploy/agents/hitter_agent.py
```

`HitterAgent` 的作用是：

```text
加载 HITTER ONNX
读取 ONNX metadata
把 joint_names / default_joint_pos / action_scale / PD 参数交给 HitterEnv
循环执行 policy 推理和 env.step
```

它和基础 agent 的最大不同不是推理循环，而是它明确要求环境支持：

```text
configure_from_modelmeta()
```

并且日志里会标明：

```text
HITTER policy metadata loaded.
```

为什么要新增 agent？

```text
因为 HITTER 不走基础 MosaicEnv 的 motion 播放逻辑，
而是走 HitterEnv 的击球 command 逻辑。
```

agent 本身很薄，主要负责把 HITTER policy 和 HITTER env 接起来。

### 7.3 新增 HitterEnv，替代 MosaicEnv 的 motion_loader 逻辑

基础 `MosaicEnv` 的核心是：

```python
self.motion_loader = MotionDataset(self.motion_cfg, self.simulator)
```

每一步它从 motion 数据里拿：

```text
command
robot_anchor_pos_w
robot_anchor_quat_w
anchor_pos_w
anchor_quat_w
```

然后拼接 tracking observation。也就是说，基础 `MosaicEnv` 的驱动源是：

```text
motion npz
```

HITTER 的 `HitterEnv` 不创建 `MotionDataset`。它创建的是：

```python
self.hitter_ball_planner = self._build_hitter_ball_planner()
```

这个 planner 由几部分组成：

```text
BallTrajectoryPredictor
StrikePlanner
BaseTargetPlanner
HitterSystemPlanner
```

HITTER 每一步问的不是“motion 下一帧是什么”，而是：

```text
球现在在哪？
球速度是多少？
球什么时候经过击球平面？
球拍应该去哪里？
球拍应该多快？
机器人 base 应该站哪？
```

为什么要这么做？

```text
乒乓球不是固定 motion replay。
球的位置和速度决定击球时机和目标。
打球部署时，reference motion 只是训练时的先验，
真正运行时必须根据球状态生成 command。
```

### 7.4 HITTER 的部署 observation 换成 105 维击球输入

基础 `MosaicEnv` 拼的是 motion tracking observation，典型内容是：

```text
command
motion_anchor_pos_b
motion_anchor_ori_b
base_lin_vel
base_ang_vel
joint_pos_rel
joint_vel_rel
previous_action
history buffer
```

它表达的是：

```text
参考 motion 在哪，我和它差多少。
```

HITTER 的 `HitterEnv` 拼的是固定 105 维 observation：

```text
base_ang_vel              3
projected_gravity         3
base_forward_xy           2
base_target_pos           3
racket_target_pos         3
racket_target_vel         3
time_to_strike            1
joint_pos_rel             29
joint_vel_rel             29
prev_policy_action        29
```

合计：

```text
3 + 3 + 2 + 3 + 3 + 3 + 1 + 29 + 29 + 29 = 105
```

为什么要这么改？

```text
HITTER policy 不需要逐帧读一大串 motion history。
它需要知道当前身体状态、身体目标、球拍目标、球拍速度目标和离击球还剩多久。
```

直白理解：

```text
基础 MosaicEnv observation：我该怎么追这段 motion？
HitterEnv observation：球来了，我该怎么站、怎么挥拍、什么时候打？
```

### 7.5 新增 hitter_planner.py：把球状态变成 policy command

HITTER RobotBridge2 新增的关键模块是：

```text
deploy/utils/hitter_planner.py
```

里面主要包括：

```text
BallStateEstimator
BallTrajectoryPredictor
StrikePlanner
BaseTargetPlanner
HitterSystemPlanner
```

直白解释：

```text
BallStateEstimator：根据球的位置采样估计球速。
BallTrajectoryPredictor：根据重力、阻力、桌面反弹预测球轨迹。
StrikePlanner：计算球什么时候到击球平面，球拍应该怎么打出去。
BaseTargetPlanner：根据击球点计算机器人 base 应该站哪。
HitterSystemPlanner：把上面几件事串起来，输出完整 HITTER command。
```

为什么要加这个 planner？

```text
基础 RobotBridge 没有“球”这个概念。
它只会部署 motion policy。
HITTER 要打球，就必须在 policy 外面加一个 planner，
把球状态转换成 policy 能理解的 base/racket/time_to_strike 目标。
```

### 7.6 MuJoCo simulator 新增乒乓球状态和球物理辅助逻辑

基础 MuJoCo simulator 主要管理：

```text
机器人模型
关节状态
PD 控制
viewer
teleop
foot contact
```

HITTER 的 MuJoCo simulator 增加了乒乓球相关状态：

```text
table_tennis_enabled
hitter_ball_body_id
hitter_ball_qposadr
hitter_ball_qveladr
ball_pos_world
ball_vel_world
ball_ang_vel_world
reset_hitter_ball()
_update_hitter_ball_state()
ball trajectory candidates
ball randomization
```

这些新增内容让 `HitterEnv` 和 `hitter_planner.py` 能实时拿到：

```text
球的位置
球的线速度
球的角速度
球是否需要 reset
```

为什么要改 simulator？

```text
policy 和 planner 都需要球状态。
基础 MuJoCo 只知道机器人，不知道乒乓球在哪里、速度是多少、什么时候反弹。
```

### 7.7 新增 analytic table bounce 和 analytic racket hit

HITTER MuJoCo 里新增了两类解析逻辑：

```text
_apply_hitter_ball_analytic_table_bounce()
_apply_hitter_ball_analytic_racket_hit()
```

直白解释：

```text
解析桌面反弹：
球碰到桌面高度附近时，按照 restitution 修改速度。

解析球拍击球：
球接近 planner 目标击球点，并且球拍也在附近时，
直接给球设置 planner 算好的出球速度。
```

为什么不完全依赖 MuJoCo 的自然接触？

```text
乒乓球和球拍的接触非常快、非常轻、非常敏感。
纯物理接触对时间步、碰撞几何、接触参数都很挑，
容易出现没打上、乱弹、能量不对。
```

解析击球的目的不是证明真实接触物理完全准确，而是稳定验证：

```text
planner 是否生成了合理击球目标；
policy 是否把球拍送到了目标附近；
球能否按照 planner 预期被打出去。
```

### 7.8 新增 G1 + racket + table tennis 资产配置

HITTER 使用的资产配置是：

```text
deploy/config/asset/g1_hitter_racket.yaml
```

其中模型文件是：

```text
g1_29dof_hitter_racket_table_tennis.xml
```

这和基础 RobotBridge 的普通 G1 模型不同。HITTER 的 MuJoCo XML 需要包含：

```text
G1 机器人
右手球拍
乒乓球
球桌或相关几何
球 freejoint
球拍碰撞几何
```

HITTER 还设置了默认根位置：

```yaml
default_root_pos: [-0.4, 0.0, 0.793]
```

为什么要这么做？

```text
机器人需要站在球桌前，并且右手需要带球拍。
普通 motion tracking 里的 G1 站在原点做动作即可，
但打乒乓球需要明确的球桌坐标系、球坐标系和击球空间。
```

### 7.9 HITTER control 配置的适配

HITTER 的控制配置在：

```text
deploy/config/control/g1_hitter_racket.yaml
```

关键项包括：

```yaml
is_mosaic: True
use_teleop: False
update_with_fk: True
viewer: True
real_time: True
playback_slowdown: 1.0
```

其中：

```text
is_mosaic: True
```

表示 policy 输出经过 `action_scale` 和 `default_joint_pos` 后，直接作为目标关节位置 PD target。

```text
use_teleop: False
```

表示 HITTER 部署默认不走 teleop command，而是走 ball planner 生成的击球 command。

```text
playback_slowdown
```

用于慢放 MuJoCo。因为击球发生很快，不慢放很难看清球拍是否到位、球什么时候被打出去。

### 7.10 ONNX metadata 机制仍然沿用

这一点和基础 RobotBridge 类似。HITTER 仍然从 ONNX metadata 读取：

```text
joint_names
default_joint_pos
joint_stiffness
joint_damping
action_scale
anchor_body_name
```

然后部署端根据 metadata 对齐 simulator joint order，配置默认关节角和 PD 参数。

为什么继续沿用 metadata？

```text
这样部署端不用手写 policy 的关节顺序和 PD 参数，
避免训练和部署之间出现关节顺序、默认姿态、action scale 不一致。
```

### 7.11 RobotBridge2 部署侧核心区别总结

基础 RobotBridge 的逻辑是：

```text
我有一段 motion，你照着做。
```

HITTER RobotBridge2 的逻辑是：

```text
球来了；
先算球会飞到哪；
再算机器人该站哪、球拍该去哪、该多快挥；
最后让 policy 控制全身去完成这一拍。
```

所以 HITTER 在 RobotBridge2 里新增和修改的东西，本质上都围绕一个目的：

```text
把“球状态”转换成“机器人能执行的击球目标”，
再把这个目标喂给训练好的 HITTER policy。
```

## 8. 新增乒乓球 ball planner 参数

HITTER 的部署配置中新增了乒乓球规划相关参数，位于：

```text
RobotBridge2/deploy/config/mimic/hitter.yaml
```

其中包括：

```text
table_center_xy_w
table_height
table_length
table_width
ball_radius
virtual_hit_plane_x
desired_landing_point_w
post_hit_flight_time
racket_restitution
prediction_horizon_s
prediction_dt
vertical_restitution
horizontal_restitution
drag_coefficient
target_base_height_w
racket_x_offset_b
swing_duration_range
force_strike_type
forehand_nominal_racket_y_b
backhand_nominal_racket_y_b
```

这些参数用于描述：

```text
球桌尺寸
球半径
球路预测
落点目标
球拍反弹模型
空气阻力
正手/反手 nominal racket 位置
击球持续时间
```

这些内容是基础 MOSAIC/RobotBridge 没有的，是 HITTER 面向乒乓球任务新增的关键逻辑。

## 9. MuJoCo 仿真场景不同

基础 RobotBridge/MuJoCo 主要验证：

```text
机器人是否能加载 policy
observation/action 维度是否对齐
是否能跟踪 reference motion
```

HITTER RobotBridge2/MuJoCo 主要验证：

```text
G1 + racket 模型是否正确
乒乓球状态是否能被读取或模拟
ball planner 是否生成击球 command
ONNX policy 是否能根据 command 控制机器人
机器人是否能把球打出去
```

也就是说，HITTER 的 MuJoCo 演示重点是：

```text
球状态 -> planner -> policy -> 持拍击球
```

它不是完整比赛回合仿真，也不一定展示完整来球过程。当前默认演示中，球可能一开始就接近击球时刻，所以画面里主要看到机器人击球。

如果需要慢放观察，可以使用：

```bash
cd /home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/RobotBridge2/deploy

python run.py \
  --config-name=hitter \
  sim=mujoco \
  mimic.policy.checkpoint=../../HITTER-main/checkpoints/hitter_m20400/exported/policy.onnx \
  robot.control.playback_slowdown=4
```

## 10. ONNX metadata 仍然是部署关键

基础 MOSAIC -> RobotBridge 中，RobotBridge 依赖 ONNX metadata 读取：

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

HITTER 仍然沿用这个机制。RobotBridge2 启动时会读取 HITTER policy 的 metadata，并配置 29 个 DoF。

成功日志通常包含：

```text
Total Number of dof: 29
Number of Action: 29
HITTER ball planner configured.
Loading ONNX Checkpoint from ../../HITTER-main/checkpoints/hitter_m20400/exported/policy.onnx
HITTER policy metadata configured, 29 degrees of freedom.
HITTER policy metadata loaded. Joints: 29 Anchor body: pelvis
```

这些日志说明：

```text
MuJoCo 机器人 DoF 对齐
ONNX action 维度对齐
RobotBridge2 成功读取 HITTER metadata
ball planner 已经配置
```

## 11. 已记录的修复和适配

当前 HITTER 复现过程中，已经做过或确认过以下修复和适配。

### 11.1 Git LFS 资产完整性

旧目录中部分大文件曾是 Git LFS pointer，导致 IsaacLab 资产加载失败。切换到：

```text
/home/yhl/Desktop/Omega-Athlete-full
```

并执行：

```bash
git lfs pull
```

后，关键 USD 资产恢复正常。

### 11.2 G1 hitter racket USD cache 路径

`g1_hitter_racket/main.usda` 引用了：

```text
HITTER-main/cache/g1_usd_hitter/main.usd
```

已通过 symlink 指向实际资产目录：

```text
/home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/HITTER-main/cache/g1_usd_hitter
-> /home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/HITTER-main/source/whole_body_tracking/whole_body_tracking/assets/unitree_description/usd/g1_hitter_racket_from_urdf
```

### 11.3 motion body index 不匹配

参考动作回放曾出现 body index 越界。原因是当前机器人 USD body ordering 和 31-body motion npz ordering 不完全一致。

已修复为：motion 侧 body names 和 robot 侧 body names 分开处理。

相关文件：

```text
HITTER-main/source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/commands.py
HITTER-main/source/whole_body_tracking/whole_body_tracking/tasks/hitter/config/g1/flat_env_cfg.py
```

其中设置了：

```text
G1_HITTER_MOTION_BODY_NAMES
```

对应：

```text
pelvis + 29 actuated child links + right_racket_link
```

### 11.4 play.py 打包 checkpoint 路径解析

曾出现：

```text
FileNotFoundError: logs/rsl_rl/hitter_striking_g1
```

原因是 `play.py` 收到：

```text
--resume_student_checkpoint
```

后仍然调用日志目录搜索逻辑。

已修为：显式传入 `--resume_student_checkpoint` 时直接使用该 checkpoint 路径。

### 11.5 RobotBridge2 playback_slowdown

为了方便观察 MuJoCo 中较快的击球动作，已给：

```text
RobotBridge2/deploy/simulator/mujoco.py
```

增加：

```text
robot.control.playback_slowdown
```

并在：

```text
RobotBridge2/deploy/config/control/g1_hitter_racket.yaml
```

加入默认值：

```yaml
playback_slowdown: 1.0
```

因此可以直接通过 Hydra override 调慢播放：

```bash
robot.control.playback_slowdown=4
```

## 12. 最小复现命令

### 12.1 安装训练侧本地包

```bash
conda activate isaaclab

cd /home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/HITTER-main

python -m pip install -e source/rsl_rl
python -m pip install -e source/whole_body_tracking
```

注意：如果环境里之前安装过基础 MOSAIC 的同名 `whole_body_tracking` 包，需要用 HITTER-main 里的 editable install 覆盖到当前项目版本。

### 12.2 回放参考动作

```bash
cd /home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/HITTER-main

python scripts/rsl_rl/play_reference_motion.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion motions \
  --num_envs=4
```

### 12.3 播放 checkpoint 并导出 ONNX

```bash
cd /home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/HITTER-main

python scripts/rsl_rl/play.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion motions \
  --resume_student_checkpoint checkpoints/hitter_m20400/model_20400.pt \
  --disable_obs_noise \
  --skip_critic
```

导出结果：

```text
checkpoints/hitter_m20400/exported/policy.onnx
```

### 12.4 MuJoCo 部署

```bash
cd /home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/RobotBridge2/deploy

python run.py \
  --config-name=hitter \
  sim=mujoco \
  mimic.policy.checkpoint=../../HITTER-main/checkpoints/hitter_m20400/exported/policy.onnx
```

### 12.5 MuJoCo 慢放

```bash
cd /home/yhl/Desktop/Omega-Athlete-full/github_publish/HITTER/RobotBridge2/deploy

python run.py \
  --config-name=hitter \
  sim=mujoco \
  mimic.policy.checkpoint=../../HITTER-main/checkpoints/hitter_m20400/exported/policy.onnx \
  robot.control.playback_slowdown=4
```

## 13. 结论

HITTER 对基础 MOSAIC + RobotBridge 的主要扩展可以归纳为：

```text
1. 新增 Hitter-Striking-PlannerDomain-Flat-G1-v0 训练任务。
2. 新增 forehand/backhand 乒乓球击球参考动作。
3. 新增 G1 + racket 机器人资产和部署配置。
4. 新增 hitter 专用 observation、command、reward、termination。
5. 新增 RobotBridge2 hitter agent/env/config。
6. 新增 ball planner，用于球路预测和击球 command 生成。
7. MuJoCo 场景从动作跟踪扩展为持拍击球验证。
8. 保留 ONNX metadata 部署机制，但 policy observation 从基础 MOSAIC 的 800 维 tracking 输入变成 HITTER 的 105 维击球输入。
9. 修复和适配了 LFS 资产、USD cache、motion body index、checkpoint 路径和慢放观察等工程问题。
```
