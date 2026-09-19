# RobotBridge 中文学习说明

本文档是面向“从零开始学习 RobotBridge”的中文说明文档。它不是对 README 的逐句翻译，而是把项目的定位、体系结构、运行链路、各模块输入输出、基本使用方式整理成一份更适合学习和持续补充的笔记。

后续可以在这个文档上继续同步新的学习结论。

## 1. 项目定位

`RobotBridge` 不是训练仓，而是整个体系里的部署与运行时桥接层。

它的核心职责是：

- 统一 `sim2sim / sim2real` 的运行接口
- 承接 locomotion policy 与 mimic policy 的部署
- 支持 MuJoCo 仿真和真机控制两种执行后端
- 使用 Hydra 进行模块化配置装配
- 通过 Unitree transition layer 对接真机底层控制链路

可以把它理解为：

**Policy Runtime + Env Adapter + Simulator/Robot Backend + Config Assembler**

更直白一点说，它负责把“训练好的策略”接到“仿真器或真机”上，让同一套上层运行逻辑尽量同时适用于仿真和真实机器人。

## 2. 整体结构

项目最重要的主链路是：

`run.py -> agent -> env -> simulator`

也就是：

- `run.py` 负责读取配置、组装系统并启动
- `agent` 负责加载策略并做推理
- `env` 负责组织观测、处理动作、维护任务逻辑
- `simulator` 负责真正与 MuJoCo 或真机通信

如果是真机部署，还会多一层：

`Python policy layer <-> LCM <-> C++ transition layer <-> Unitree SDK <-> Robot`

## 3. 目录结构理解

项目中最关键的目录如下：

```text
RobotBridge/
├── deploy/
│   ├── run.py
│   ├── agents/
│   ├── envs/
│   ├── simulator/
│   ├── config/
│   └── utils/
├── unitree_sdk2/
└── README.md
```

各部分职责如下。

### 3.1 `deploy/run.py`

入口文件，负责：

- 读取 Hydra 配置
- 实例化 `agent`
- 调用 `agent.run()`

它本身不实现任务逻辑，也不直接处理 MuJoCo 或真机控制，而是“装配器”。

### 3.2 `deploy/agents/`

策略执行层。主要负责：

- 加载策略模型
- 从环境拿 observation
- 做推理得到 action
- 把 action 送回环境执行

典型文件：

- `level_agent.py`：locomotion 策略执行器
- `mosaic_agent.py`：mimic 策略执行器
- `loco_mimic_agent.py`：双策略切换执行器

### 3.3 `deploy/envs/`

环境与接口适配层。主要负责：

- 从 simulator 获取机器人状态
- 组装成 policy 需要的 observation
- 接收 action 并做缩放、裁剪、映射
- 管理 reset、termination、motion 播放、teleop、模式切换等任务逻辑

这层是整个项目最像“桥”的地方。

典型文件：

- `base_env.py`：统一的环境基类
- `level_locomotion.py`：locomotion 运行逻辑
- `mosaic.py`：mimic 运行逻辑
- `loco_mimic_switch.py`：locomotion 与 mimic 的切换逻辑

### 3.4 `deploy/simulator/`

执行后端层。主要负责：

- 把环境给出的目标动作真正执行到 MuJoCo 或真机
- 读取机器人状态
- 向上暴露统一接口

典型文件：

- `base_sim.py`：统一仿真/真机接口
- `mujoco.py`：MuJoCo 后端
- `real_world.py`：真机后端

### 3.5 `deploy/config/`

配置装配层。使用 Hydra 管理各类配置，包括：

- 机器人配置
- observation 配置
- control 配置
- 仿真后端配置
- agent 配置
- env 配置
- locomotion/mimic 任务配置
- teleop 配置

这个目录决定了系统如何被“拼起来”。

### 3.6 `unitree_sdk2/`

真机 transition layer 所在目录。核心作用是：

- 与 Unitree SDK 通信
- 接收来自 policy layer 的控制消息
- 转发控制命令给真机
- 将底层状态发布回上层

其中 `trans.cpp` 是关键桥接实现。

## 4. 各模块输入输出

这一节非常重要。学习 RobotBridge 时，最好始终从“这一层吃什么、吐什么”来理解代码。

### 4.1 `run.py`

输入：

- Hydra 配置
- 命令行 override 参数

输出：

- 一个被正确实例化并启动的 agent

### 4.2 `agent`

输入：

- 来自 `env` 的 observation
- policy checkpoint，例如 `.onnx`、`.pt`、`.jit`

输出：

- action 向量
- 某些 agent 还会输出模式切换控制

### 4.3 `env`

输入：

- 来自 `simulator` 的机器人状态
- 来自 `agent` 的 action
- 来自 teleop / motion reference / joystick 的附加输入

输出：

- 给 `agent` 的 observation 字典
- 给 `simulator` 的目标动作或目标关节位置

### 4.4 `simulator`

输入：

- 来自 `env` 的动作或目标关节位置

输出：

- 最新机器人状态
- 如果是 MuJoCo，还包括可视化结果
- 如果是真机，还包括向 transition layer 发出的控制消息

### 4.5 `unitree transition layer`

输入：

- 来自 Python policy layer 的 LCM 控制消息

输出：

- 给真机底层控制接口的命令
- 回传给上层的机器人状态与遥控器状态

## 5. 关键运行流程

一次完整运行的主数据流如下：

1. 启动 `python deploy/run.py --config-name=...`
2. Hydra 读取配置并实例化 `agent`
3. `agent` 内部持有对应的 `env`
4. `env` 内部实例化对应的 `simulator`
5. `env.reset()` 触发初始化和第一帧状态获取
6. `env` 组织 observation
7. `agent` 用 policy 对 observation 做推理，得到 action
8. `env.step(action)` 对 action 做处理
9. `simulator.apply_action(action)` 执行到 MuJoCo 或真机
10. `simulator.get_state()` 产生新状态
11. `env` 重新组织下一帧 observation
12. 进入下一轮循环

可以压缩成一句话：

`state -> env组obs -> agent推理 -> env处理action -> simulator执行 -> new state`

## 6. 三类典型运行模式

RobotBridge 目前可以先从三类典型使用方式来理解。

### 6.1 只跑 locomotion

入口配置：

- `deploy/config/level_locomotion.yaml`

特点：

- policy 是 locomotion
- env 是 `level`
- simulator 默认是 `mujoco`

适合理解：

- 键盘/摇杆命令是如何进入系统的
- `actor_obs` 是如何拼接出来的
- locomotion action 如何变成关节控制目标

### 6.2 只跑 mimic

入口配置：

- `deploy/config/mosaic.yaml`

特点：

- policy 是 mimic ONNX 模型
- env 是 `mosaic`
- observation 往往是多输入字典，而不是单个向量

适合理解：

- motion reference 如何进入系统
- ONNX metadata 如何决定 joint order、PD 参数等配置
- mimic action 如何映射回 simulator 的关节顺序

### 6.3 locomotion 与 mimic 切换

入口配置：

- `deploy/config/loco_mimic.yaml`

特点：

- 同时加载两个策略
- 同一个 env 管理两种模式
- 切换时需要插值过渡，避免动作硬切

适合理解：

- deployment bridge 的核心价值
- 不同 DoF 策略如何共存
- 模式切换时如何维持动作连续性

## 7. 主要组件概念说明

### 7.1 `LevelAgent`

面向 locomotion 策略，通常加载 TorchScript 模型。

大致逻辑：

- 从 `env` 获取 `actor_obs`
- 调用 policy 输出 action
- 把 action 传回 env

### 7.2 `MosaicAgent`

面向 mimic 策略，通常加载 ONNX 模型。

它除了做推理，还会读取模型 metadata，把以下信息同步给 env：

- joint names
- default joint positions
- stiffness / damping
- action scale
- anchor body 信息

### 7.3 `LocoMimicAgent`

同时管理 locomotion 与 mimic 两套策略。

它的关键价值在于：

- 选择当前 active policy
- 在两种策略之间切换
- 通过插值平滑过渡，避免直接跳变

### 7.4 `BaseEnv`

统一环境基类，定义了标准流程：

- `reset()`
- `step(action)`
- `compute_observation()`
- `_pre_physics_step()`
- `_physics_step()`
- `_post_physics_step()`

从学习角度看，`BaseEnv` 是“上层策略运行循环”和“下层执行后端”之间的总接口。

### 7.5 `Mujoco`

MuJoCo 后端负责：

- 加载 XML 机器人模型
- 读取仿真状态
- 将目标动作转为 PD 控制
- 推进物理步进
- 渲染画面

### 7.6 `RealWorld`

真机后端负责：

- 接收 LCM 状态消息
- 解码遥控器输入
- 将目标关节位置发布为控制命令
- 在运行周期上与真机同步

### 7.7 `trans.cpp`

真机通信桥。它负责：

- 与 Unitree SDK 通信
- 将底层状态转换成上层可用消息
- 接收上层策略命令并发送给机器人

## 8. Hydra 配置系统怎么理解

RobotBridge 采用 Hydra 的模块化配置装配方式，而不是所有内容堆在一个文件里。

例如一个主配置通常会组合：

- `robot`
- `obs`
- `sim`
- `agent`
- `env`
- `mimic`
- `locomotion`
- `teleop`

这意味着运行系统本质上是由多个配置片段拼装出来的。

Hydra 的价值在这个项目里主要体现为：

- 切机器人方便
- 切策略方便
- 切仿真/真机后端方便
- 切模型 checkpoint 和动作文件方便

从学习角度，可以把 Hydra 理解成“运行时装配菜单”。

## 9. 项目整体怎么使用

如果是第一次接触，推荐按下面顺序学习和尝试。

### 第一步：先跑仿真 locomotion

```bash
cd deploy
python run.py --config-name=level_locomotion
```

重点关注：

- teleop 命令从哪里来
- observation 如何组成
- policy 输出如何进入 MuJoCo

### 第二步：再看 mimic

```bash
cd deploy
python run.py --config-name=mosaic \
    env.config.motion.motion_path=data/motion/your_motion.npz \
    env.config.policy.checkpoint=data/model/your_model.onnx
```

重点关注：

- motion dataset 如何喂给策略
- observation 为什么是字典
- ONNX metadata 如何影响运行时参数

### 第三步：最后看 loco_mimic

```bash
cd deploy
python run.py --config-name=loco_mimic
```

重点关注：

- 两个 policy 各自的作用域
- 切换时插值为何必要
- DoF 不一致时如何适配

### 第四步：再看真机部署

真机部署时，核心思想不是重写上层逻辑，而是把执行后端从 MuJoCo 切到 `real_world`，并在前面增加 `transition layer`。

也就是说，这个仓库真正想实现的是：

**尽量保持上层 policy runtime 不变，只替换底层执行后端。**

这正是它作为 deployment/runtime bridge 的核心价值。

## 10. 推荐阅读顺序

建议按以下顺序阅读源码：

1. `README.md`
2. `deploy/run.py`
3. `deploy/config/level_locomotion.yaml`
4. `deploy/agents/level_agent.py`
5. `deploy/envs/level_locomotion.py`
6. `deploy/envs/base_env.py`
7. `deploy/simulator/base_sim.py`
8. `deploy/simulator/mujoco.py`
9. `deploy/envs/mosaic.py`
10. `deploy/envs/loco_mimic_switch.py`
11. `deploy/agents/loco_mimic_agent.py`
12. `unitree_sdk2/trans.cpp`

这个顺序适合先建立最简单的闭环，再逐步进入 mimic、策略切换和真机桥接逻辑。

## 11. 当前学习阶段最该记住的三件事

### 11.1 policy 不直接碰 MuJoCo 或真机

policy 只负责：

- 吃 observation
- 吐 action

### 11.2 env 是翻译层

env 负责：

- 把底层机器人状态翻译成 policy 能理解的 observation
- 把 policy 输出的 action 翻译成执行端能落地的目标动作

### 11.3 simulator 是执行层

simulator 负责：

- 执行动作
- 获取状态
- 屏蔽“MuJoCo vs 真机”的实现差异

## 12. 后续补充建议

后续学习时，可以继续在本文档中追加以下内容：

- `level_locomotion` 逐文件源码笔记
- `mosaic` observation 详细拆解
- `loco_mimic_switch` 状态机与插值逻辑
- `real_world` 中 LCM 消息流整理
- `unitree_sdk2/trans.cpp` 通信链路说明
- 机器人配置文件之间的关系
- DoF 映射与 joint order 适配机制

---

这份文档目前先作为 RobotBridge 的中文学习入口。后续每学到一个模块，都可以继续把内容同步进来，逐渐把它变成一份完整的项目学习手册。
