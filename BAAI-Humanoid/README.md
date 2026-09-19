# 总的
这是一个做 人形机器人 whole-body mobile manipulation 的项目，重点不是单独做行走或单独做抓取，而是把 移动、全身控制、接触交互、操作任务 结合起来，做能在真实环境里稳定工作的系统。
当前 README 里强调了 5 个研究主题：
Whole-body mobile manipulation：让 locomotion 和 manipulation 协同工作
Contact-rich whole-body control：平衡、抗扰动、恢复、安全执行
Dexterous manipulation：双手灵巧操作，结合视觉/触觉/本体感知
Teleoperation & data：人类示教采集、动作重定向、数据集整理
Sim-to-real deployment：域适配、残差学习、系统辨识，以及可复现训练/评测/部署链路
整体长期目标是：做出基于学习的人形机器人系统，具备 move / reach / interact / recover 能力，并且能做好 sim-to-real transfer 和 规模化数据训练管线。

# 每个仓库是干什么的
这个目录本身更像一个“总工作区”，真正的 7 个独立仓库 是这几个：
.github
DECO
DECO-webpage
MOSAIC
MOSAIC-teleop
Omega-EVA
RobotBridge
我是根据各目录里独立的 .git 判断的；根目录自己的 .git 更像是把这些仓库聚到一起管理的外层工作区。
下面是我基于各自 README/入口文件做的初步职责说明。
.github
这是组织/项目门面的资料仓，不是算法代码仓。
目前主要内容是 [README.md](/home/yhl/Desktop/BAAI-Humanoid/.github/profile/README.md)，用来说明 BAAI-Humanoid 的总体研究方向：whole-body mobile manipulation、contact-rich control、dexterous manipulation、teleop/data、sim-to-real。

DECO
这是一个 双手灵巧操作策略训练与部署仓，核心是论文里的 DECO 模型。
从 [README.MD](/home/yhl/Desktop/BAAI-Humanoid/DECO/README.MD) 看，它覆盖：
数据集 DECO-50
训练配置与统计量计算
多模态输入（视觉、触觉、本体）
双臂/双手 manipulation policy 训练
H1-2 上的部署流程
所以它更偏 manipulation policy / imitation learning / tactile-enhanced dexterous manipulation。

DECO-webpage
这是 DECO 项目的展示网页仓，不是研究代码。
从 [index.html](/home/yhl/Desktop/BAAI-Humanoid/DECO-webpage/index.html) 和目录内容看，它就是一个已经构建好的静态站点，里面有图片、视频、JS/CSS 构建产物，用来展示论文、方法和 demo。

MOSAIC
这是 人形机器人全身动作跟踪 / teleoperation policy 训练仓。
从 [README.md](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/README.md) 看，它主要做：
在 Isaac Lab / Isaac Sim 里训练 tracking policy
处理 motion 数据
训练 general motion tracker
做 adaptor / residual adaptation / distillation
它的定位很明确：训练端，尤其是面向 whole-body tracking 和 teleoperation policy 的学习系统。

MOSAIC-teleop
这是 遥操作输入侧与动作重定向仓。
从 [README.md](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC-teleop/README.md) 看，它负责：
接 Noitom 动捕、PICO 4 VR
做 human-to-robot motion retargeting
实时流式发送控制命令
记录数据用于离线训练
简单说，MOSAIC 偏训练，MOSAIC-teleop 偏 人类输入采集 + 在线遥操作桥接。

Omega-EVA
这是一个 新方法/新论文仓，现在还没正式开源代码。
从 [README.md](/home/yhl/Desktop/BAAI-Humanoid/Omega-EVA/README.md) 看，它的核心思想是：
先 proposed action
再用 latent world model “envision” 未来
再 verify/refine 后输出最终动作
也就是一个 world-model-in-the-loop 的 embodied action generation 项目。当前更多是论文占位与发布计划，暂时不是可直接跑的代码仓。

RobotBridge
这是 仿真到真机部署桥接仓，也是运行时最像“系统集成层”的仓库。
从 [README.md](/home/yhl/Desktop/BAAI-Humanoid/RobotBridge/README.md) 看，它负责：
sim2sim / sim2real 统一接口
locomotion policy 与 mimic policy 切换
MuJoCo 仿真
真机控制接口
Hydra 配置管理
Unitree 相关 transition layer
所以它是整个体系里的 deployment/runtime bridge。

如果把这 7 个仓库放在一张图里理解，大致是：
.github：项目总介绍
DECO：双手灵巧操作学习
DECO-webpage：DECO 展示页
MOSAIC：全身 tracking / teleop policy 训练
MOSAIC-teleop：动捕/VR 遥操作与重定向
RobotBridge：仿真和真机部署桥
Omega-EVA：下一代 embodied policy / latent world model 方向
按工程链路再压缩一下，可以理解成：
人类输入采集 (MOSAIC-teleop) -> 策略训练 (MOSAIC / DECO) -> 仿真与真机部署 (RobotBridge)
而 .github、DECO-webpage、Omega-EVA 更偏 对外说明/论文发布层。

# mosaic和robotbridge这两个仓库的关系
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
如果你愿意，我下一步可以继续帮你把这两个仓库画成一张更具体的“接口关系图”，比如：
motion data -> MOSAIC training -> checkpoint/onnx -> RobotBridge agent/env/simulator -> sim or real robot。