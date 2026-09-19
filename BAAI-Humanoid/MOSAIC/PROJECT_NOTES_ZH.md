# MOSAIC 中文项目说明

本文档用于从零开始学习 `MOSAIC` 项目时做持续积累。目标不是一次性写全，而是随着阅读代码、跑脚本、理解训练链路，逐步补成一份可复习的中文说明。

## 1. 项目一句话定位

`MOSAIC` 是一个面向 `humanoid whole-body tracking` 和 `teleoperation policy` 的训练仓，重点在：

- 基于 `Isaac Lab / Isaac Sim` 进行策略训练
- 使用运动数据驱动 imitation / tracking 学习
- 训练 general motion tracker
- 为不同遥操作接口训练 adaptor
- 通过 residual adaptation / distillation 做快速适配与能力迁移

它更偏“训练系统”，不是部署端或机器人运行时总控仓。  
如果关注 sim-to-sim 或 sim-to-real 的部署，README 里明确指向另一个仓库：`RobotBridge`。

## 2. 我当前对项目主线的理解

按 README 的描述，MOSAIC 的完整训练流程大致分成 3 段：

1. 先准备和预处理 motion 数据
2. 训练一个通用 motion tracker / GMT
3. 针对具体 teleoperation 接口训练 adaptor，并结合 residual adaptation / distillation 做快速适配

更具体地说：

- motion 数据先从 retarget 后的 `.csv` 开始
- 通过脚本转成带有更多运动学信息的本地 `.npz`
- 在 Isaac Sim 里把 motion 当作 reference，让机器人学会 tracking
- 多源数据训练得到 general tracker
- 再引入 adaptor 和 distillation / residual 训练，让系统适配不同输入接口

## 3. 推荐的学习顺序

如果是第一次接触这个仓，我建议按下面顺序学：

1. `README.md`
   先建立项目目标、训练阶段和常用命令的整体印象
2. `scripts/`
   先看数据处理、训练、评估入口脚本，建立“怎么跑”的感觉
3. `tasks/tracking/config/`
   看任务是怎么注册的、不同 task id 分别对应什么环境和 runner
4. `tasks/tracking/mdp/`
   看 observation / reward / command / termination / event 这些 MDP 原子组件
5. `flat_env_cfg.py` 和 `tracking_env_cfg.py`
   看环境级配置是怎么把 MDP、机器人、传感、奖励拼起来的
6. `source/rsl_rl/`
   看训练算法、runner、网络结构，理解 PPO、distillation、MOSAIC 扩展部分
7. `run/*.sh`
   最后回到官方训练脚本，理解完整实验流程如何落地

## 4. 当前能确认的目录分工

### `source/whole_body_tracking/`

这是任务定义和训练环境主体，偏“任务侧 / 仿真侧”。

- `tasks/tracking/`
  tracking 任务定义核心位置
- `tasks/tracking/mdp/`
  MDP 原子组件，包括 observations、rewards、events、commands、terminations
- `tasks/tracking/config/`
  不同机器人和不同训练模式对应的环境与算法配置
- `robots/`
  机器人相关参数与封装
- `utils/`
  motion 数据加载、runner 封装、模型导出等辅助逻辑
- `collection/`
  expert trajectory 采集、student observation 构造等，偏数据采集与蒸馏准备
- `assets/`
  机器人模型、urdf/mjcf、mesh 资源

### `source/rsl_rl/`

这是算法库，偏“RL 训练器 / 网络 / 存储 / runner”。

- `algorithms/`
  包含 `ppo.py`、`distillation.py`、`mosaic.py`
- `modules/`
  actor-critic、student-teacher、velocity estimator 等网络模块
- `runners/`
  on-policy runner
- `storage/`
  rollout storage

可以先把它理解成：`whole_body_tracking` 定义任务，`rsl_rl` 负责训练。

### `scripts/`

这是最直接的学习入口，里面放了很多“拿来就能跑”的脚本：

- 数据预处理：`csv_to_npz.py`、`batch_csv_to_npz.py`
- 数据分析/回放：`analyze_dataset.py`、`replay_npz.py`
- 训练入口：`scripts/rsl_rl/train.py`
- 评估入口：`scripts/rsl_rl/play.py`
- expert 数据采集：`scripts/rsl_rl/collect_expert_trajectories.py`
- 速度估计器训练：`train_ref_vel_estimator.py`

### `run/`

这是更接近“论文/官方流程复现”的 shell 脚本集合，例如：

- `run_mosaic_gmt.sh`
- `run_mosaic_adaptor.sh`
- `run_mosaic_residual_adaptation.sh`
- `run_mosaic_pure_distillation.sh`

这些脚本体现的是“完整训练阶段怎么串起来”，不是最底层实现。

## 5. 任务注册层目前看到的关键信息

在 `source/whole_body_tracking/whole_body_tracking/tasks/tracking/config/g1/__init__.py` 中，项目把很多 Isaac Lab task 注册成了 gym task id。

我现在先把它们粗分成几类：

- 基础 tracking
  - `Tracking-Flat-G1-v0`
  - `General-Tracking-Flat-G1-v0`
- 去掉 state estimation 的 tracking 变体
  - `Tracking-Flat-G1-Wo-State-Estimation-v0`
  - `General-Tracking-Flat-G1-Wo-State-Estimation-v0`
- 低控制频率变体
  - `Tracking-Flat-G1-Low-Freq-v0`
  - `General-Tracking-Flat-G1-Low-Freq-v0`
- expert / distillation / MOSAIC 专用任务
  - `Expert-General-Tracking-Flat-G1-v0`
  - `Expert-General-Tracking-Flat-G1-MOSAIC-v0`
  - `Distillation-General-Tracking-Flat-G1-v0`
  - `MOSAIC-Distill-General-Tracking-Flat-G1-v0`
  - `MOSAIC-Pure-Distill-General-Tracking-Flat-G1-v0`
  - `MOSAIC-RL-Continue-General-Tracking-Flat-G1-v0`
  - `MOSAIC-Residual-General-Tracking-Flat-G1-v0`
  - `MOSAIC-MultiTeacher-Residual-Tracking-Flat-G1-v0`
- one-stage / world-coordinate reward 相关任务
  - `General-Tracking-Flat-G1-Wo-State-Estimation-v0-World-Coordinate-Reward`

这说明仓库不是只有一个 tracking 任务，而是围绕同一个主体问题，派生出了一系列训练阶段和消融/变体配置。

## 6. README 里能直接读出的训练主线

### 阶段 A：motion preprocessing

输入通常是 retarget 后的 motion `.csv`。  
通过脚本转成带有更多运动学信息的 `.npz`，供后续训练和回放使用。

相关脚本：

- `scripts/csv_to_npz.py`
- `scripts/batch_csv_to_npz.py`
- `scripts/replay_npz.py`

### 阶段 B：tracking policy training

在 Isaac Lab 里把 reference motion 作为目标，让机器人学习跟踪。

分成两种常见模式：

- single motion tracking
- multi motion / general tracking

训练入口是：

- `scripts/rsl_rl/train.py`

评估入口是：

- `scripts/rsl_rl/play.py`

### 阶段 C：MOSAIC 全流程训练

README 给出的顺序是：

1. 训练 GMT policy
2. 训练 teleoperation adaptor
3. 更新配置中的 motion path / model path
4. 做 residual adaptation / multi-teacher distillation

其中 `run/run_mosaic_gmt.sh` 当前使用的 task 是：

- `General-Tracking-Flat-G1-Wo-State-Estimation-v0-World-Coordinate-Reward`

这至少说明一件事：GMT 阶段并不是直接用最普通的 tracking task，而是用了一个专门的 one-stage / world-coordinate reward 变体。

## 7. 几个重要术语的当前版本理解

这部分先写“学习中版本”，后面可以随着读代码不断修正。

### whole-body tracking

给定 reference motion，训练 humanoid 在仿真中尽可能跟踪全身姿态与运动趋势，而不是只完成单个局部动作。

### teleoperation policy

面向遥操作接口的策略。它不只是“会模仿动作”，还要能稳定地接收某种操作输入，并产出机器人全身动作。

### GMT

从 README 和脚本命名看，`GMT` 更像 general motion tracker，也就是多源、多动作数据上训练得到的通用 tracking 策略。

### adaptor

用于把某种 teleoperation 接口信号映射到 tracker 更容易消费的表示，或者补偿接口差异。

### residual adaptation

在已有通用能力之上增加残差修正，让系统针对具体接口或场景快速适配，而不用从头重训全部策略。

### distillation

把 teacher 侧能力蒸馏到 student，常用于压缩、统一、迁移，或者把多教师能力收敛到一个更可部署/更高效的策略里。

## 8. 初学者现在最值得先回答的几个问题

后面学习时，我建议优先把这些问题逐个打通：

1. `scripts/rsl_rl/train.py` 最终是怎样把 task id 映射到 env cfg 和 runner cfg 的？
2. `flat_env_cfg.py` 里到底定义了哪些 observation、reward、command 和 randomization？
3. motion `.csv -> .npz` 时具体补充了哪些字段？
4. `General-Tracking` 和 `Expert-General-Tracking` 的核心区别是什么？
5. `MOSAIC` 相比普通 PPO / 普通 tracking，多出来的算法逻辑放在哪？
6. adaptor 的输入输出各是什么？训练监督来自哪里？
7. residual / distillation 阶段分别冻结和训练哪些模块？

## 9. 接下来建议怎么学

最自然的下一步不是直接扎进所有源码，而是先做一轮“入口导读”：

1. 看 `scripts/rsl_rl/train.py`
2. 看 `scripts/rsl_rl/play.py`
3. 看 `tasks/tracking/config/g1/__init__.py`
4. 看 `tasks/tracking/config/g1/flat_env_cfg.py`

这样能先建立“命令行参数 -> task 注册 -> env cfg -> runner cfg”的主链路。

---

## 学习记录

### 第 1 轮

- 已确认项目定位：训练端，目标是 humanoid whole-body tracking 与 teleoperation policy 学习
- 已确认主要模块：`whole_body_tracking` 负责任务与环境，`rsl_rl` 负责算法与训练
- 已确认 README 给出的训练大流程：motion preprocessing -> tracking / GMT -> adaptor -> residual adaptation / distillation
- 已确认 G1 任务注册里存在基础 tracking、general tracking、expert、distillation、residual 等多种 task 变体

### 第 2 轮：训练与评估入口

#### 2.1 `scripts/rsl_rl/train.py` 在做什么

这个文件可以先当成“训练总入口”。

它做的事情按顺序大致是：

1. 解析命令行参数
2. 启动 Isaac Sim / Isaac Lab app
3. 通过 `@hydra_task_config(args_cli.task, "rsl_rl_cfg_entry_point")` 根据 task id 取回：
   - `env_cfg`
   - `agent_cfg`
4. 用 CLI 参数覆盖默认配置
5. 把 `--motion` 写进 `env_cfg.commands.motion.motion`
6. `gym.make(task, cfg=env_cfg)` 创建环境
7. 用 `RslRlVecEnvWrapper` 包装环境
8. 用 `MotionOnPolicyRunner` 创建训练 runner
9. 写日志、保存配置、可选恢复 checkpoint
10. 调 `runner.learn(...)` 正式训练

可以把它记成一句话：

`train.py = 参数入口 + task 配置装配 + 环境创建 + runner 启动`

#### 2.2 `train.py` 最关键的几件事

##### A. task id 决定训练的“任务模板”

例如：

- `Tracking-Flat-G1-v0`
- `General-Tracking-Flat-G1-v0`
- `MOSAIC-MultiTeacher-Residual-Tracking-Flat-G1-v0`

这些 task id 并不是字符串标签而已，它们会通过 task registry 映射到：

- 一个 `env cfg`
- 一个 `rsl_rl runner cfg`

所以改 task，通常就意味着在切换训练阶段或实验配置。

##### B. `--motion` 是强制参数

这点很重要：训练入口要求必须传 `--motion`。

说明这个仓的大多数 tracking 训练都不是“空环境自发学习”，而是显式依赖 reference motion 数据驱动的。

##### C. 分布式训练时会做 rank 级随机种子偏移

`train.py` 里专门处理了：

- `WORLD_SIZE`
- `RANK`
- `LOCAL_RANK`

并按 rank 计算 `env_seed`。  
这说明项目认真考虑了多卡并行时各 rank 采样完全一致的问题。

##### D. 实际 runner 不是 Isaac Lab 默认 runner，而是自定义 runner

训练入口里导入的是：

- `from whole_body_tracking.utils.my_on_policy_runner import MotionOnPolicyRunner as OnPolicyRunner`

这很值得后面重点看。  
通常这意味着项目在标准 on-policy runner 基础上加了 motion-tracking 相关的自定义逻辑。

#### 2.3 `scripts/rsl_rl/play.py` 在做什么

`play.py` 是评估 / 回放入口，但它不只是“加载模型然后显示一下”。

它做了几类很有用的事情：

1. 加载 checkpoint
   - 可以从本地日志目录加载
   - 也可以通过 `--wandb_path` 从 WandB run 下载
2. 用 checkpoint 自带的 `params/agent.yaml` 回填一部分 policy 配置
3. 用 CLI 提供的 `--motion` 覆盖当前评估用的 motion
4. 在评估时主动关掉一些训练期随机性
   - motion randomization
   - observation noise
   - event randomization
   - timeout termination
5. 创建环境并加载 policy
6. 导出 `onnx`

所以可以把它理解成：

`play.py = checkpoint 评估入口 + 复现实验配置 + 导出部署模型`

#### 2.4 `play.py` 里值得特别记住的点

##### A. 评估时默认更“干净”

默认会倾向关闭：

- 观测噪声
- event randomization
- motion randomization

这说明训练和评估的设定并不完全一样。  
训练更偏鲁棒性，评估更偏可观察、可回放、可对齐。

##### B. `--motion` 在评估时依然很关键

`play.py` 里如果没传 `--motion` 会直接报错。  
这再次说明，这套系统的评估通常是围绕“让当前 policy 去跟踪某段给定 motion”展开的。

##### C. 评估脚本会导出 ONNX

脚本中调用了：

- `export_motion_policy_as_onnx(...)`
- `attach_onnx_metadata(...)`

这表示训练仓虽然不以部署为主，但已经考虑了把策略导出成可下游使用的模型格式。

#### 2.5 `scripts/rsl_rl/cli_args.py` 的作用

这个文件主要负责把训练/评估脚本中的 RSL-RL 相关参数统一起来，并把 CLI 参数覆盖到 runner 配置上。

它主要做三件事：

1. 定义通用参数
   - `experiment_name`
   - `run_name`
   - `resume`
   - `load_run`
   - `checkpoint`
   - `logger`
   - `log_project_name`
   - `wandb_path`
   - `distributed`
2. 提供 `parse_rsl_rl_cfg(...)`
   - 从 task registry 读取默认 runner 配置
3. 提供 `update_rsl_rl_cfg(...)`
   - 用 CLI 参数覆盖默认配置
   - 处理 student checkpoint resume
   - 处理 teacher checkpoint 覆盖

#### 2.6 现在可以先记住的主链路

从学习角度，我建议先背住下面这条链：

`命令行参数`
-> `train.py / play.py`
-> `cli_args.py`
-> `task id`
-> `g1/__init__.py` 里的 registry
-> `env cfg + runner cfg`
-> `gym.make(...)`
-> `RslRlVecEnvWrapper`
-> `MotionOnPolicyRunner / OnPolicyRunner`
-> `policy training or playback`

如果这条链清楚，后面再去看 reward、observation、distillation、adaptor，心里就不会散。
