# MOSAIC 训练主线通俗讲解

这份文档的目标不是替代源码，而是先帮我快速建立对 `MOSAIC` 训练主线的直觉理解。  
写法尽量直白，但不跳过背后的数理逻辑。

---

## 1. 先用一句话理解整个项目

`MOSAIC` 做的事情，本质上可以概括成四步：

1. 准备参考动作 `motion`
2. 把参考动作变成训练时可读取的标准数据
3. 在仿真里让机器人反复看这个动作并学习跟踪
4. 在通用 tracking 能力上，再做 adaptor / distillation / residual adaptation

如果用更数理一点的话说，它在学一个策略：

`a_t = pi(o_t)`

这里：

- `o_t` 是时刻 `t` 的观测
- `a_t` 是时刻 `t` 要输出的动作
- 策略 `pi` 的目标，是让机器人产生的运动轨迹尽量接近 reference motion

也就是希望真实轨迹和参考轨迹之间的误差尽量小。通常可以抽象成：

`min Σ_t L(state_t, ref_t)`

或者在 RL 里等价地写成：

`max Σ_t r_t`

其中 reward `r_t` 本质上就是：

- 跟踪得越像，奖励越高
- 越不稳，惩罚越大

---

## 第零步：选task id

task id 决定的不是“起个名字”，而是：
你到底要训练哪一种环境、哪一种观测、哪一种奖励、哪一种训练配置。

所以严格说，流程更准确是：
准备/选定要做的训练任务类型 task id
准备与之匹配的 motion npz
跑 train.py
1. task id 本质上是什么
你可以先把它理解成：
一套训练配方的入口名

比如：
Tracking-Flat-G1-v0
General-Tracking-Flat-G1-v0
MOSAIC-MultiTeacher-Residual-Tracking-Flat-G1-v0
这些字符串背后，不只是标签。它们会在 task registry 里映射到：
一个 env cfg
一个 runner cfg
也就是：
环境怎么搭
机器人是谁
观测有哪些
reward 怎么算
reset / termination 怎么做
PPO 或 MOSAIC 的训练超参数是什么
2. 为什么在准备 motion npz 前就要想 task id
因为不同 task id 对 motion 的“使用方式”和“训练目的”可能不同。
直白说，motion 不是脱离任务独立存在的，而是要被某个 task 当作 reference 来解释。
例如：
Tracking-Flat-G1-v0
更像单动作/基础 tracking。
它关注的是：
G1 机器人
平地环境
标准 tracking 配置
这时你的 motion 更像“某一段想让它模仿的动作”。
General-Tracking-Flat-G1-v0
更像多动作/general motion tracking。
这时通常不是一条 motion 文件，而是一个 motion 文件夹，代表一个动作分布。
General-Tracking-Flat-G1-Wo-State-Estimation-v0-World-Coordinate-Reward
这已经不是普通 tracking 了，而是一个更特殊的 GMT 训练设定。
它意味着：
observation 可能少了 state estimation 相关输入
reward 更强调 world-coordinate consistency
所以同样是一批 motion，放到不同 task 里，训练目标其实不一样。
3. 数理上，task id 决定了什么
从 RL 角度，一个训练任务本质上定义了一个 MDP 或近似 MDP。
而 task id 基本就在选这套 MDP 配方。
它决定至少这几件事：
1. observation space
policy 能看到什么：
o_t
2. action space
policy 输出什么动作：
a_t
3. reward function
什么叫“学得好”：
r_t
4. reset / termination
从哪里开始、何时结束
5. reference motion 的解释方式
reference 是单动作、多动作、未来窗口、teacher 命令，还是别的形式
所以 task id 本质上是在决定你优化的目标：
max E[Σ_t gamma^t r_t]
这里的 r_t、o_t、状态转移方式，都会受 task 影响。
4. 工程上它具体做了什么
在这个仓里，train.py 里传入 --task=... 后，会通过 task registry 找到对应配置。
也就是大致这条链：
task id -> env cfg + runner cfg -> gym.make(...) -> 开始训练
比如一个 task id 会对应：
某个 G1Flat...EnvCfg
某个 G1Flat...RunnerCfg
所以你选 task，其实是在选：
训练阶段
训练模板
算法配置
5. 那为什么 README 常常先讲 motion，再讲 task？
因为从直觉上，motion 更容易理解：
“我拿一段动作来训练机器人模仿它。”
但从系统结构上，其实应该先有：
我要做哪类 tracking 任务？

然后才有：
我拿什么 motion 来喂这个任务？

所以更完整的理解是：
task id 定义“怎么学”
motion npz 提供“学什么动作”
6. 你现在可以这样记
一句最实用的话：
task id 负责定义训练规则，motion npz 负责提供参考动作内容。

或者更形式化一点：
task id 决定训练环境 Env
motion npz 决定 reference trajectory ref_t
两者一起定义 tracking 问题。
7. 对你现在学习最重要的结论
在 Tracking Policy Training 这一段里，不是简单地：
给个 motion -> 开始训练
而是：
选 task id -> task 决定训练配方 -> 把 motion 作为 reference 塞进这个配方
所以后面我们讲 commands.py / observations.py / rewards.py 时，你会更清楚：
这些模块不是凭空存在
它们都是 task 配方的一部分


## 2. 第一步：Motion Preprocessing 到底在干嘛

这是最先要理解的一步，因为这个项目不是“无目标探索”，而是“拿着参考动作学跟踪”。

输入是 retarget 后的 `.csv` motion。  
这里的 “retarget” 可以先直白理解成：

- 原始动作数据可能来自人、MoCap、别的骨架
- 先把它映射到目标机器人可理解的关节定义上
- 于是得到和机器人骨架语义一致的动作序列

但 `.csv` 还不够直接拿来训练。为什么？

因为训练时不只是要知道“某一帧关节角是什么”，通常还要知道：

- 根部位姿
- 各 body 的位姿
- 速度
- 角速度
- 可能还有加速度
- 时间序列中的帧关系

所以 preprocessing 的目标是：

把“原始动作表格”变成“训练时可快速读取、信息更完整的 reference motion 文件”。

也就是：

`.csv -> .npz`

你可以把它理解成把数据从“记录格式”变成“训练格式”。

### 准备的原始motion，也就是'.csv'文件，是什么格式的？是从哪里来的，这个框架关心这件事吗？
1. 这个仓要求的原始 .csv 是什么格式？
从 [scripts/csv_to_npz.py (line 149)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/scripts/csv_to_npz.py:149) 可以直接看出来，它把每一行当作一帧，并按固定列切分：
前 3 列：base position
接着 4 列：base rotation quaternion
剩下所有列：joint dof positions
也就是单帧格式大致是：
[root_pos(3), root_quat(4), dof_pos(N)]
再具体一点：
motion[:, :3] -> 根部位置
motion[:, 3:7] -> 根部四元数
motion[:, 7:] -> 机器人各关节角
而且脚本里还做了一步四元数重排：
输入看起来是 xyzw
读进来后会转成内部使用的 wxyz
这一点在 [scripts/csv_to_npz.py (line 163)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/scripts/csv_to_npz.py:163) 也能看到。
所以，这个 .csv 不是随便一张动作表，而是一个已经被整理成“每帧 = 根位姿 + 根朝向 + 全部关节位置”的时序矩阵。
2. 这些 .csv 通常从哪里来？
README 说得比较明确，项目假设你先拿到的是 retarget 后的 reference motion，并且采用与 Unitree 数据集类似的 .csv 约定，见 [README.md (line 86)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/README.md:86)。
常见来源有几类：
公共 motion 数据集README 里明确提到 AMASS、OMOMO 等

MOSAIC 自带公开数据optical MoCap
inertial MoCap
GENMO 生成动作

你自己采集的数据但前提是你要先把它 retarget 到目标机器人骨架

所以原始来源可以很多样：
人体动作捕捉
仿真生成动作
其他骨架系统的数据
自采 teleop 数据
但进入这个仓训练之前，都必须先变成它认得的机器人动作序列。
3. 这个框架“关不关心来源”？
不太关心的部分
它通常不太关心：
你是光学 MoCap 还是惯性 MoCap
你是 AMASS 还是 OMOMO
你是人动作还是生成动作
你数据采集时用的上游骨架长什么样
非常关心的部分
它非常关心以下几点：
是否已经 retarget 到目标机器人
是否使用 generalized coordinates
列顺序是否符合它的读取假设
关节数量和顺序是否和当前 robot 配置一致
四元数格式是否正确
fps 是否明确
动作是否物理上合理、连续、足够平滑
README 里有一句很关键的话：reference motion 应该是 retargeted，并且使用 generalized coordinates，见 [README.md (line 86)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/README.md:86)。
这句话翻成大白话就是：
这个框架不负责替你理解“原始人体动作数据”；它希望你喂进来的已经是“机器人能读懂的动作”。
4. 什么叫它关心“语义”？
这里的“语义”不是自然语言那个语义，而是：
这 3 列真的是根位置吗？
这 4 列真的是根旋转吗？
后面的每一列是不是对应正确的机器人关节？
第 10 列到底是左膝还是右肩？
四元数是不是归一化的？
这一帧和下一帧是不是同一个动作序列的连续状态？
因为一旦这些语义错了，后面虽然代码还能跑，但训练目标就变味了。
从数学上讲，训练假设参考轨迹是：
ref_t = (p_root, q_root, q_joint)
如果你喂进去的列语义错位了，那么 reward 计算的“误差”就不再对应真实动作误差，策略学到的东西会直接跑偏。
5. 这个仓会不会帮你做“来源到 csv”的转换？
会帮一点，但不是全包。
仓里已经有一些辅助转换脚本，例如：
[scripts/batch_pkl_to_csv.py (line 64)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/scripts/batch_pkl_to_csv.py:64)
[scripts/batch_npy_to_csv.py (line 70)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/scripts/batch_npy_to_csv.py:70)
从这些脚本也能看出来，它们最后都在拼同一种结构：
root_pos/root_trans
root_rot/root_ori
dof_pos
也就是说，这个项目内部其实已经默认了一种“动作中间表示”。
所以更准确地说：
它不关心你最早的数据源
但它强依赖一个统一的动作中间格式
6. 最实用的理解方式
你现在可以把这个训练仓想成一个“下游消费者”。
它对上游来源的态度是：
来源随你，但你送进来之前，必须先翻译成我能吃的格式。

所以这条链应该这样理解：
原始数据源 -> retarget -> 标准 csv -> csv_to_npz -> 训练
其中真正属于这个仓强约束的，是中间这段：
标准 csv -> npz -> training
7. 给你一个当前阶段最该记住的结论
一句话总结：
MOSAIC 不太关心 motion 最早从哪里来，但非常关心它进入训练前是否已经被整理成“目标机器人上的根位姿 + 根旋转 + 关节位置”的标准时序格式。

### 2.1 这一步的数理意义

训练时每个时刻都要比较：

- 当前机器人状态 `s_t`
- 参考动作状态 `s_t^ref`

如果 reference 里只有部分关节值，没有速度、根部朝向、全身 link 信息，那 reward 和 observation 都没法完整定义。

所以 preprocessing 本质是在构造一个更完整的参考信号：

`ref_t = {q_t, dq_t, pose_t, vel_t, ...}`

其中：

- `q_t` 是关节位置
- `dq_t` 是关节速度
- `pose_t` 是 body pose
- `vel_t` 是 body velocity

这样训练时才能计算 tracking error。

### 为什么它不能直接拿 csv 训练，而要先变成 npz
不是不能直接用 csv，而是 直接用会缺信息、计算慢、接口不统一，所以项目先把它变成更完整的 npz。

可以拆成 4 个原因。
1. csv 只给了基础位姿，不够训练直接吃
这个仓读 csv 时，本质上拿到的是每一帧：
root position
root quaternion
joint positions
但训练时通常还要用：
root linear velocity
root angular velocity
joint velocity
各 body/link 的世界坐标位姿
各 body/link 的速度
可能还有 anchor/body 局部表示
也就是说，训练里参考信号更像：
ref_t = {q_t, dq_t, body_pose_t, body_vel_t, ...}
而不是只有：
ref_t = {root_pos, root_rot, dof_pos}
所以 npz 其实是在把 reference 补全。
2. reward 计算需要“派生量”
tracking reward 不是只比关节角。
它往往会比：
根部位置误差
朝向误差
关节误差
body pose 误差
速度误差
全局运动一致性
这些量很多都不能直接从原始 csv 一眼拿来，得先算。
从数学上说，如果 reward 是：
r_t = r(q_t, dq_t, body_t, body_vel_t, ref_t)
那 ref_t 就必须是完整的。
预处理就是把这些参考量预先算好，而不是训练时每一步临时现算。
3. 训练时反复现算会很慢
RL 训练不是读一遍数据就完了，而是：
成千上万个并行环境
上百万甚至更多步
每一步都可能访问 reference motion
如果每一步都从 csv 现读、现插值、现求速度、现做运动学展开，代价会很高。
所以工程上更合理的是：
预处理阶段算一次
存成 npz
训练时直接高速读取
这就是典型的“离线预计算换在线效率”。
4. npz 更适合作为统一训练格式
csv 更像“原始动作交换格式”，可读性强，但结构弱。
npz 更像“训练缓存格式”，适合存多个数组键值，比如：
root_states
joint_pos
joint_vel
body_pos_w
body_quat_w
body_lin_vel_w
body_ang_vel_w
这样后面的训练、回放、采样、分析脚本都能按统一接口取数据。
所以不是单纯“换后缀”，而是把数据从：
文本表格
变成：
结构化 reference state package
你可以先记住这句最关键的话：
csv 更像“动作描述”，npz 更像“训练时可直接调用的参考轨迹缓存”。

再往前一步说，csv_to_npz.py 做的事情本质是：
稀一点、原始一点的动作表示 -> 稠密且训练友好的参考状态表示



### 2.2 从“怎么用项目”的角度怎么理解这一步

先不用急着搞清所有字段。使用层面只要先记住：

- `.csv` 更像原始参考动作
- `.npz` 更像训练消费格式
- 训练和回放主要吃 `.npz`

所以如果以后“想训练一段新动作”，第一反应应该是：

1. 准备 retarget 后的 motion
2. 跑 `csv_to_npz.py` 或 `batch_csv_to_npz.py`
3. 再把生成的 motion 路径喂给训练脚本

### csv_to_npz.py 到底具体算出了哪些字段，以及这些字段后面分别被谁用。明确输入是哪些字段，输出是哪些字段。
一句话先概括：
csv_to_npz.py 的作用不是“简单换格式”，而是把 csv 里的根位姿 + 关节角序列，扩展成后续训练/回放/数据加载统一使用的 reference motion package (.npz)。

下面我按你要的方式来：明确输入字段、输出字段、以及每个字段后面被谁用。
1. 输入到底是什么
从 [scripts/csv_to_npz.py (line 149)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/scripts/csv_to_npz.py:149) 的 _load_motion() 可以直接看出来，输入 csv 每一行代表一帧，字段切分方式是：
[base_pos(3), base_quat(4), dof_pos(N)]
也就是：
前 3 列：根位置 base position
接着 4 列：根旋转四元数 base quaternion
剩下所有列：关节位置 joint dof positions
对应代码是：
motion[:, :3] -> motion_base_poss_input
motion[:, 3:7] -> motion_base_rots_input
motion[:, 7:] -> motion_dof_poss_input
另外它还做了一步四元数顺序转换：
输入 csv 里的四元数顺序看起来是 xyzw
内部转成 wxyz
见 [scripts/csv_to_npz.py (line 163)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/scripts/csv_to_npz.py:163)。
所以输入字段可以写成：
root_pos: [T, 3]
root_quat: [T, 4]
dof_pos: [T, N]
这里 T 是帧数，N 是当前机器人关节数。
2. 脚本中间额外算了什么
在写出 npz 之前，这个脚本先做了三类处理。
第一类：时间重采样 / 插值
scripts/csv_to_npz.py 会把输入 fps 的动作插值到输出 fps：
位置、关节角：线性插值 lerp
四元数：球面插值 slerp
见 [scripts/csv_to_npz.py (line 172)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/scripts/csv_to_npz.py:172)。
所以如果原始 csv 是 30Hz，训练想统一成 50Hz，它会先补帧。
第二类：速度计算
脚本会从插值后的序列里算出：
根线速度 motion_base_lin_vels
关节速度 motion_dof_vels
根角速度 motion_base_ang_vels
见 [scripts/csv_to_npz.py (line 217)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/scripts/csv_to_npz.py:217)。
这一步很关键，因为输入 csv 里本来没有速度。
第三类：通过 Isaac Sim/robot model 展开成 body 级状态
这一步是最容易忽略、但最重要的。
脚本不是直接把 root + joint 存起来完事，而是：
把当前帧的 root state 写进机器人
把当前帧的 joint state 写进机器人
调 scene.update(...)
从机器人当前状态里读取所有 body/link 的世界系信息
见 [scripts/csv_to_npz.py (line 308)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/scripts/csv_to_npz.py:308)。
这相当于做了一次基于机器人骨架的前向运动学展开，得到全身每个 body 的状态。
3. 输出 .npz 里到底有哪些字段
真正写入 .npz 的字段在 [scripts/csv_to_npz.py (line 281)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/scripts/csv_to_npz.py:281) 这段 log 里已经定义好了，一共 7 个：
fps
joint_pos
joint_vel
body_pos_w
body_quat_w
body_lin_vel_w
body_ang_vel_w
后面在 reset 时 stack 并 np.savez(...) 存盘，见 [scripts/csv_to_npz.py (line 339)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/scripts/csv_to_npz.py:339)。
4. 每个输出字段分别是什么意思
我给你翻成最直白的版本。
fps
形状大致是 [1]
表示这个 npz 动作最终采用的帧率
作用：
主要是元信息，方便后续知道时间步长
joint_pos
形状：[T, num_joints]
每一帧机器人的关节位置
注意：
这已经不是原始 csv 里“裸的 dof_pos”
而是映射到当前机器人 joint layout 后的完整关节位置
joint_vel
形状：[T, num_joints]
每一帧机器人的关节速度
由插值后的关节位置数值微分得到
body_pos_w
形状：[T, num_bodies, 3]
每一帧每个 body/link 在世界坐标系下的位置
body_quat_w
形状：[T, num_bodies, 4]
每一帧每个 body/link 在世界坐标系下的朝向四元数
body_lin_vel_w
形状：[T, num_bodies, 3]
每一帧每个 body/link 在世界坐标系下的线速度
body_ang_vel_w
形状：[T, num_bodies, 3]
每一帧每个 body/link 在世界坐标系下的角速度
5. 这些字段后面分别被谁用
这部分是重点。你可以把后续使用者分成三类：
replay / 可视化
tracking command / reward / observation
其他训练辅助模块
5.1 被 replay_npz.py 用
在 [scripts/replay_npz.py (line 141)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/scripts/replay_npz.py:141) 可以直接看到：
body_pos_w[:, 0] 用来设置 root position
body_quat_w[:, 0] 用来设置 root orientation
body_lin_vel_w[:, 0] 用来设置 root linear velocity
body_ang_vel_w[:, 0] 用来设置 root angular velocity
joint_pos / joint_vel 用来设置关节状态
也就是说：
回放脚本会直接拿这些 npz 字段，把 reference 动作重新播给机器人。
5.2 被 tracking 的 commands.py 用
这个是训练主线最核心的使用者。
在 [source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/commands.py (line 247)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/commands.py:247) 可以看到，单个 motion 文件加载时直接读：
joint_pos
joint_vel
body_pos_w
body_quat_w
这里很关键：
它没有直接读 csv，而是默认 reference motion 已经是 npz。
这些字段后面被用来构造当前时刻的 reference command：
joint_pos() 返回当前参考关节位置
joint_vel() 返回当前参考关节速度
body_pos_w() 返回当前参考 body 位置
body_quat_w() 返回当前参考 body 朝向
见 [commands.py (line 319)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/commands.py:319)。
然后这些 reference 值会继续参与：
reset 时把机器人摆到 motion 初始状态
计算 tracking error
构造 observation 里的 command 部分
给 reward / termination 用
5.3 被 reward / termination / metric 间接用
在 commands.py 里，参考 motion 和真实机器人状态会被直接比较，形成误差指标：
error_body_pos
error_body_rot
error_joint_pos
error_joint_vel
见 [commands.py (line 404)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/commands.py:404)。
这些误差后面就是 reward、termination、日志 metric 的基础。
也就是说，npz 输出字段本质上在支撑这个量：
error = robot_state - reference_state
如果没有 joint_vel、body_pos_w、body_quat_w 这些字段，很多 tracking reward 根本没法定义。
5.4 被 motion_data_loader.py 用于辅助训练
在 [motion_data_loader.py (line 36)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/source/whole_body_tracking/whole_body_tracking/utils/motion_data_loader.py:36) 可以看到，它会从 npz 中拿：
joint_pos
joint_vel
body_quat_w
body_lin_vel_w
然后抽取 anchor body（默认 G1 的 torso link）来构造额外训练数据，比如：
projected gravity
anchor frame 下的线速度
带历史的 velocity estimator 输入
这说明：
npz 不只是给 tracking policy 用，也给辅助网络/估计器训练用。
5.5 被 vq_similarity.py 这类分析脚本用
在 [scripts/vq_similarity.py (line 129)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/scripts/vq_similarity.py:129) 可以看到，它直接从 npz 里取：
joint_pos
joint_vel
来组成 motion features。
所以这类分析脚本也依赖 npz 作为统一输入格式。
6. 一个特别值得注意的点：谁没被直接存？
你可能会注意到：
输入里有 root pos / root quat
中间也算了 root velocities
但输出里没有单独的 root_pos、root_quat、root_vel
为什么？
因为它们已经被包含进：
body_pos_w[:, 0]
body_quat_w[:, 0]
body_lin_vel_w[:, 0]
body_ang_vel_w[:, 0]
也就是：
root 被当成 body 列表里的第 0 个 body 来统一存储了。
这是一种很好的工程统一方式。
7. 你现在可以记住的“输入输出总表”
输入 csv：
root_pos [T, 3]
root_quat [T, 4]
dof_pos [T, N]
中间计算：
插值到目标 fps
root_lin_vel
root_ang_vel
dof_vel
基于机器人模型展开出全 body 状态
输出 npz：
fps
joint_pos [T, J]
joint_vel [T, J]
body_pos_w [T, B, 3]
body_quat_w [T, B, 4]
body_lin_vel_w [T, B, 3]
body_ang_vel_w [T, B, 3]
8. 最后的本质理解
所以 csv_to_npz.py 真正做的不是：
“把 csv 改成 npz”

而是：
“把只包含根位姿和关节角的原始动作序列，变成训练系统可直接消费的全身 reference state 库。”

也就是把：
root + dof
扩展成：
joint + body-level kinematic state
这就是为什么后面训练、回放、蒸馏、辅助估计器都统一吃 npz。


---

## 3. 第二步：Tracking Policy Training 在干嘛

这是整个项目最核心的一层。

一句话说就是：

让策略学会根据当前观测，输出控制动作，使机器人尽量跟上 reference motion。

还是写成 RL 形式：

- 环境给 observation：`o_t`
- policy 输出 action：`a_t = pi(o_t)`
- 环境推进到下一时刻
- 计算 reward：看这一步跟 reference 有多像、是否稳定
- PPO 之类算法不断更新策略参数

### 一、外层：怎么把 Tracking Policy Training 启动起来
这一层先不碰复杂 reward 和 observation，只讲“训练是怎么组织起来的”。
最直白地说，外层流程是：
选 task id
准备 motion npz
跑 scripts/rsl_rl/train.py
创建环境 env
创建训练器 runner
开始 PPO 训练
这条链你可以先记成：
task + motion + train.py -> env -> runner -> learn()
下面一个个拆。
1. 先选 task id：决定你在做哪一种 tracking
task id 不是名字，而是一套训练配方的入口。
比如：
Tracking-Flat-G1-v0
General-Tracking-Flat-G1-v0
General-Tracking-Flat-G1-Wo-State-Estimation-v0-World-Coordinate-Reward
这些 task 会决定：
用哪种机器人
是单动作还是多动作
observation 长什么样
reward 怎么算
reset 怎么做
PPO / MOSAIC 的配置是什么
所以 task id 决定的是：
“你按什么规则训练”

从 RL 角度，它其实是在决定一套任务定义，也就是近似在决定一个 MDP：
状态如何被观测
动作如何作用环境
奖励如何给出
episode 如何开始和结束
也就是决定：
(o_t, a_t, r_t, termination)
2. 再准备 motion npz：决定你在学什么动作
如果说 task id 决定“怎么学”，那么 motion npz 决定“学什么”。
motion npz 里装的是 reference motion，也就是策略想要逼近的目标轨迹。
所以这一步不是给环境“喂数据集”那么简单，而是给任务提供：
ref_t
也就是每个时刻的参考状态。
训练时，机器人会一直被拿来和这个参考轨迹做比较。
所以更准确地说，tracking 问题不是：
“机器人随便动，环境给奖励”
而是：
“机器人每一步都在试图跟上 reference motion”
3. 跑 train.py：把 task 和 motion 接起来
train.py 做的事情，本质上就是把这两样东西接起来：
--task 指定训练配方
--motion 指定参考动作
然后脚本会：
从 task registry 取出对应的 env cfg
从 task registry 取出对应的 runner cfg
把 motion 写进环境配置里
创建环境
创建 PPO runner
开始训练
所以 train.py 不是在定义 tracking 本身，而是在做“装配”。
它像一个总控入口。
4. 创建 env：把 tracking 任务实例化
env 可以先理解成“训练现场”。
里面包括：
机器人
场景
当前 reference motion
当前 step 的 observation 逻辑
reward 逻辑
reset / termination 逻辑
当 gym.make(...) 被调用时，这个 tracking 任务就真正活起来了。
这一步之后，环境已经知道：
当前要模仿哪段 motion
机器人当前状态是什么
应该返回什么 observation
应该怎么算 reward
5. 创建 runner：让 PPO 开始工作
有了 env 以后，runner 才能工作。
runner 可以先理解成：
“负责采样、存轨迹、算优势、更新策略”的训练器

它不断重复下面的循环：
用当前策略和环境交互
收集轨迹
根据 reward 计算回报和优势
更新 policy / value 网络
再继续采样
所以在外层上，Tracking Policy Training 的本质就是：
用 PPO 在 tracking 环境里反复试错，让策略逐渐更会跟 reference motion。

6. 外层这部分的数理骨架
外层虽然工程味更重，但它背后的优化目标很清楚：
策略是：
a_t = pi(o_t)
PPO 想做的是最大化累计回报：
J(pi) = E[Σ_t gamma^t r_t]
这里：
o_t 从 env 来
r_t 从 env 来
task id 决定 env 的定义
motion npz 决定 reference 是什么
所以外层其实是在搭一个优化问题。

### 二、内层：训练每一步到底在发生什么

你可以把内层理解成一句话：
每一步训练，环境都要回答三个问题：
这一刻目标动作是什么？
policy 现在看到了什么？
policy 这一步做得好不好？

这三个问题正好对应：
commands.py
observations.py
rewards.py
这三者合起来，决定了 tracking training 的“语义”。
三、第一步：commands.py 在干嘛
最直白地说：
commands.py 负责把 motion npz 变成“当前时刻要跟踪的目标”。

也就是把整段 reference motion，变成这一时刻的：
ref_t
比如当前环境时间步走到了第 k 帧，那么 command 模块会取出：
当前参考 joint position
当前参考 joint velocity
当前参考 body position
当前参考 body rotation
有时还会取未来若干帧，或者做相对坐标变换、随机扰动、重采样。
所以 commands.py 不是“发命令给机器人”，而是：
给训练环境定义“此刻应该对齐的参考目标是什么”。

如果没有这一步，tracking 就没目标了。
1. commands.py 的数学角色
它定义的是目标信号：
ref_t
这是一个时变参考轨迹的一部分。
所以训练里比较的不是“机器人和一个固定姿态的距离”，而是：
robot_state_t 和 ref_t 的距离
也就是说，tracking 是一个随时间变化的目标跟踪问题，而不是静态姿态拟合问题。
2. 为什么 command 很重要
因为如果 reference 组织方式不同，训练任务就会变。
例如：
只给当前帧 reference
给当前帧加未来帧
用 world frame 表示
用 anchor/body-relative frame 表示
这些都会改变 policy 学习的难度和归纳偏好。
所以 command 不只是“取数据”，而是在定义：
policy 到底在追什么目标、以什么形式追。

四、第二步：observations.py 在干嘛
最直白地说：
observations.py 负责把“机器人现在的状态”和“当前参考目标”拼成 policy 输入。

也就是构造：
o_t
policy 不是直接看到完整世界真值，而是看到一组被设计好的 observation。
通常里面会有两类信息：
机器人自身当前状态
关节位置
关节速度
base 朝向
projected gravity
上一步动作等

reference 相关信息
当前 command
参考关节信息
参考 body 相对信息
可能的未来目标片段

所以可以粗略写成：
o_t = concat(robot_state_features_t, reference_features_t)
1. observations.py 的数学角色
它定义的是策略真正可见的信息：
a_t = pi(o_t)
注意，policy 学的不是：
a_t = pi(full_state_t)
而是：
a_t = pi(observation_t)
这差别很大。
因为 observation 的设计，决定了：
policy 有多容易理解当前状态
policy 是否知道未来目标趋势
policy 是否需要自己“猜”更多信息
所以 observation 设计会直接影响学习难度。
2. 为什么 observation 不是越多越好
很多人会直觉觉得“给越多信息越好”，但不完全对。
如果 observation：
太冗余
太噪
太依赖某种训练期特权信息
就可能：
训练不稳定
泛化变差
部署时不成立
所以 tracking 里的 observation 设计，本质是在做一个平衡：
给足够信息让 policy 能跟踪
但不要给到失去实际意义或造成过拟合

五、第三步：rewards.py 在干嘛
最直白地说：
rewards.py 负责告诉策略：你这一步跟 reference 跟得好不好。

也就是定义：
r_t
tracking reward 通常不是单一一项，而是多项加权和。
直觉上会包括：
姿态跟踪得像不像
body 位置跟踪得像不像
朝向跟踪得像不像
速度跟踪得像不像
动作稳不稳
有没有出现不合理行为
所以可以抽象成：
r_t = w1 * r_pose + w2 * r_vel + w3 * r_body + w4 * r_stability + ...
这里不同 task 的差异，很多就体现在这些权重和项的定义上。
1. reward 的数学角色
reward 是策略优化的直接依据。
PPO 不是在直接最小化“和参考动作的欧氏距离”，而是在最大化累计 reward：
J(pi) = E[Σ_t gamma^t r_t]
所以 reward 其实是在定义：
什么样的 tracking 行为叫“好”

如果 reward 更强调 joint pose，policy 可能更会对齐关节角。
如果 reward 更强调 global motion consistency，policy 可能更重视整体运动方向和长期稳定。
所以 reward 不只是打分器，它实际上在塑造策略的行为风格。
2. 为什么同一个 motion，不同 reward 会学出不同策略
因为目标函数不同。
哪怕 reference motion 一样，只要 reward 设计不同，最优策略就可能不同。
比如：
一种 reward 偏向局部姿态精度
一种 reward 偏向整体平衡和长时间稳定
一种 reward 偏向世界坐标下一致性
那么最后学到的行为差异可能很大。
所以你在 README 里看到 GMT 用 World-Coordinate-Reward，这不是小修饰，而是在换优化偏好。
六、把三步串起来：单步训练到底发生了什么
现在把内层完整串起来。
在某个时刻 t：
commands.py 从 motion 里取出当前参考目标 ref_t
observations.py 把机器人当前状态和 ref_t 相关信息拼成 o_t
policy 根据 o_t 输出动作：
a_t = pi(o_t)
动作施加到机器人，环境推进到下一时刻
rewards.py 比较机器人新状态和 ref_t，算出 r_t
这一条 (o_t, a_t, r_t, o_{t+1}) 被存进 rollout
PPO 用很多这样的样本更新参数
所以 tracking 的学习循环，本质就是：
ref_t -> o_t -> a_t -> next state -> r_t -> update pi
七、这套 tracking training 到底在学什么
这时可以给一个非常准确、但仍然直白的定义：
它在学一个闭环控制策略。

“闭环”这两个字很重要。
因为 policy 不是把整段 motion 背下来逐帧播放，而是每一步都要根据当前真实状态重新决策。
也就是说，它学的是：
当我现在偏了、晃了、慢了、快了、姿态不准了，我该怎么出动作把自己拉回参考轨迹。

从数学上说，它学的不是静态映射，而是反馈控制律：
a_t = pi(o_t)
其中 o_t 已经包含了“当前状态”和“当前目标”的关系。
这就是为什么 tracking policy 比“直接播放动作序列”强得多。
八、你现在最该记住的结构图
把 Tracking Policy Training 压缩成下面这张脑图就够了：
外层：怎么启动
task id：定义训练配方
motion npz：提供参考动作
train.py：装配任务和数据
env：实例化 tracking 环境
runner：用 PPO 训练策略
内层：每步怎么学
commands.py：定义当前目标 ref_t
observations.py：构造观测 o_t
policy：输出动作 a_t = pi(o_t)
rewards.py：计算奖励 r_t
PPO：最大化 E[Σ gamma^t r_t]
九、如果再用一句话收尾
一句最硬核但也最直白的话：
Tracking Policy Training 的本质，是在一个由 task id 定义好的 tracking MDP 里，用 motion npz 提供时变参考目标，通过 command -> observation -> action -> reward 的闭环，让 PPO 学出一个能稳定跟踪全身动作的反馈控制策略。

### commands.py的作用，输入输出，直白讲解。
把 commands.py 先非常直白地理解成：
它是 tracking 任务里的“参考动作调度器 + 对齐器”。

它不负责训练算法本身，也不直接产生 policy 的动作。
它负责的是：
从 motion npz 里取出“这一刻应该跟踪的参考动作”
把这段参考动作和当前机器人状态对齐
提供给 observation、reward、reset 使用
所以它更像：
“训练环境内部，专门管理 reference motion 的模块”

而不是“给机器人发控制命令的模块”。
1. commands.py 的作用
一句话版：
把整段参考动作 motion，变成每个环境、每个时刻都能直接使用的 ref_t。

也就是把一整个动作序列：
motion = {ref_0, ref_1, ref_2, ...}
变成当前时刻的参考目标：
ref_t
然后训练里就能围绕这个 ref_t 做三件事：
observation: 告诉 policy “你现在要跟谁对齐”
reward: 判断 policy “你这一步跟得像不像”
reset: 环境重置时，把机器人摆到某个参考动作附近
2. 它的输入是什么
commands.py 的输入，不是单一一个东西，而是几类输入一起用。
2.1 参考动作数据：motion npz
最核心输入就是 npz 文件里的 motion 数据。
在单动作版本 MotionLoader 里直接读这些字段，见 [commands.py (line 245)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/commands.py:245)：
fps
joint_pos
joint_vel
body_pos_w
body_quat_w
body_lin_vel_w
body_ang_vel_w
所以它吃进去的是：
“预处理好的 reference motion 状态库”

而不是原始 csv。
2.2 环境配置 cfg
MotionCommandCfg / MultiMotionCommandCfg 提供了很多控制逻辑，见 [commands.py (line 575)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/commands.py:575) 和 [commands.py (line 1749)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/commands.py:1749)：
比如：
motion: 单个文件或 motion 目录
anchor_body_name
body_names
pose_range
velocity_range
joint_position_range
start_from_beginning
start_frame
这些决定：
用哪段 motion
用哪些 body 参与 tracking
reset 时是否加随机扰动
是从头播，还是随机采样某个时间点开始
2.3 当前机器人真实状态
它还不断读取当前机器人在仿真里的真实状态：
robot_joint_pos
robot_joint_vel
robot_body_pos_w
robot_body_quat_w
robot_anchor_pos_w
...
这些在 MotionCommand / MultiMotionCommand 里都是 property，见 [commands.py (line 359)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/commands.py:359) 和 [commands.py (line 1259)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/commands.py:1259)。
也就是说，它同时知道：
参考动作此刻应该是什么
机器人现在实际上是什么
所以它能做“对齐”和“误差计算”。
3. 它的输出是什么
这是最容易搞混的地方。
commands.py 的输出不是“最终控制动作”，而是：
供训练环境内部使用的 reference command / reference state。

最直接的输出有这几类。
3.1 当前时刻的参考关节命令
最直接的是：
joint_pos
joint_vel
以及合起来的：
command = concat(joint_pos, joint_vel)
见 [commands.py (line 316)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/commands.py:316)。
这个 command 可以先理解成：
“这一刻 reference motion 对机器人提出的关节级目标描述”

3.2 当前时刻的参考 body 状态
还会输出当前参考动作的全身状态：
body_pos_w
body_quat_w
body_lin_vel_w
body_ang_vel_w
以及 anchor body 的：
anchor_pos_w
anchor_quat_w
anchor_lin_vel_w
anchor_ang_vel_w
这些不是 policy 直接输出的东西，而是 reference side 的目标状态。
3.3 对齐后的相对目标
还有两个很关键的中间量：
body_pos_relative_w
body_quat_relative_w
它们在 _update_command() 里计算，见 [commands.py (line 495)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/commands.py:495)。
它们的作用是：
把 reference motion 按当前机器人 anchor 的位置/朝向做一个相对对齐

直白说就是：
reference 原本是一段“世界坐标系里的动作”
训练时不一定直接拿绝对坐标硬比
会先根据 anchor（通常是 torso / root）做对齐
得到更适合 tracking 的相对目标
3.4 reset 时写回仿真器的初始状态
在 _resample_command() 里，它还会把采样到的参考状态写回机器人，见 [commands.py (line 452)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/commands.py:452) 和 [commands.py (line 1619)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/commands.py:1619)。
也就是说它还会输出一类“隐式输出”：
用于环境 reset 的 root state 和 joint state

这不是返回值形式的输出，但它确实影响环境状态。
4. 它在训练里到底怎么工作
你可以把它想成一个循环播放、按需取帧的 reference 管理器。
单动作版 MotionCommand
单动作版逻辑比较简单：
加载一个 motion npz
每个环境维护一个 time_steps
每步 time_steps += 1
用 time_steps 从 motion 里取当前帧
如果到结尾了，就重新采样起点并 reset
所以它像：
“在一段 reference motion 上按时间往前走，走到头再重采样”

多动作版 MultiMotionCommand
多动作版更复杂一些：
先加载一个 motion 文件夹
每个环境先分配一条 motion
每个环境都有：当前是哪条 motion
当前走到那条 motion 的第几帧

到结尾时可以：重新分配 motion
从某个时间点继续
按失败率/新颖度做自适应采样

所以多动作版更像：
“每个环境都在播自己那条动作，还会动态换片源”

5. 为什么它叫 commands.py
这里的 “command” 不是控制器动作 action，而是：
reference command / target command

也就是训练目标信号。
从 RL 视角，你可以把它看成定义了：
ref_t
然后：
observations.py 用它构造 o_t
rewards.py 用它计算 r_t
所以在这个项目里：
action 是 policy 输出的
command 是 reference motion 提供的目标
这两个千万别混。
6. 用最直白的话重新总结“输入输出”
输入
commands.py 主要吃三样东西：
motion npz参考动作序列

cfg采样、随机化、anchor、body 列表等规则

当前机器人真实状态用于对齐、误差计算、reset

输出
commands.py 主要产出四类东西：
当前参考命令joint_pos, joint_vel, command

当前参考全身状态body_pos_w, body_quat_w, body_lin_vel_w, body_ang_vel_w

对齐后的相对参考目标body_pos_relative_w, body_quat_relative_w

reset / 统计信息给环境重置用的状态
error metrics
sampling metrics

7. 用一句类比帮助你记
如果把整个 tracking training 想成“学生跟着老师做动作”，那么：
motion npz 是老师整套动作录像
commands.py 是助教
助教每一秒都会说：“现在该做到第 128 帧了”
“现在重点看 torso 和 limbs 的姿态”
“你先站到这个起始姿态附近”
“你现在和标准动作差多少”

所以 commands.py 的本质就是：
把静态存储的动作数据，变成训练每一步都能用的动态目标。

8. 最简数学版
如果只压缩成最核心的数理角色，它做的是：
motion sequence -> current reference ref_t
并提供：
ref_joint_t
ref_body_t
ref_anchor_t
再结合机器人当前真实状态 state_t，产生：
对齐后的目标
误差指标
reset 采样状态
所以它站在训练链里的位置就是：
motion -> commands.py -> ref_t -> observations/rewards

### observations.py的作用，输入输出，直白讲解。
这里有个很关键的认知先立住：
observations.py 不是“整套 observation 配方”，
它更像是一个 observation 零件库 / 取值函数库。

真正决定 “policy 最终看到什么、按什么顺序拼起来” 的，是
[tracking_env_cfg.py (line 170)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/source/whole_body_tracking/whole_body_tracking/tasks/tracking/tracking_env_cfg.py:170)。
所以如果用最直白的话说：
commands.py：先准备当前参考目标 ref_t
observations.py：提供一些“怎么从环境里提取特征”的函数
tracking_env_cfg.py：决定哪些特征真的喂给 policy
1. observations.py 的作用
一句话版：
把机器人当前状态、以及 reference motion 的当前目标，提取成神经网络能直接吃的向量特征。

也就是把环境里那些原始物理量，比如：
机器人 torso 在世界里的姿态
身体各个 link 的位置
当前参考动作的 anchor 位姿
当前参考动作的线速度
转成适合 policy 输入的数值向量：
o_t
你可以把它理解成一个“特征提取层”。
2. 它的输入是什么
observations.py 的函数普遍长这样：
def xxx(env, command_name):
    command = env.command_manager.get_term(command_name)
    ...
    return feature
所以它的输入本质上有两类。
2.1 环境 env
环境里有两大块信息：
当前机器人真实状态
当前 command term（也就是 reference motion 当前时刻的目标）
所以 observations.py 并不直接读 npz 文件。
它是通过 env 去间接拿数据。
2.2 command_name
通常是 "motion"。
它的作用是告诉 observation 函数：
“你去环境里的哪个 command term 取 reference 信息”

也就是它会去拿 commands.py 产出的那套 reference 数据。
3. 它的输出是什么
输出很简单：
一个 tensor 特征向量

一般形状是：
[num_envs, D]
也就是每个并行环境一行特征。
这些输出不会直接控制机器人，而是会被 observation group 拼起来，成为 policy 输入：
o_t = concat(feature_1, feature_2, ..., feature_k)
4. observations.py 里到底有哪些特征
我按直白语义给你分组。
A. 机器人自身 anchor 状态
robot_anchor_ori_w
见 [observations.py (line 12)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/observations.py:12)
作用：
取机器人 anchor body 的朝向，并转成旋转矩阵的前两列表示

直白理解：
anchor 通常是 torso / base 一类核心 body
它想告诉 policy：机器人身体主干现在朝哪边
为什么不是直接给四元数？
因为很多 RL 里会更喜欢用旋转矩阵的一部分来表示方向，数值上更规整一点。
输出大致是：
每个环境一个朝向特征向量
robot_anchor_lin_vel_w
作用：
取机器人 anchor 的线速度

直白理解：
告诉 policy：机器人主干现在在往哪里移动、多快
robot_anchor_ang_vel_w
作用：
取机器人 anchor 的角速度

直白理解：
告诉 policy：机器人主干现在转得有多快
B. 机器人全身相对 anchor 的姿态
robot_body_pos_b
见 [observations.py (line 29)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/observations.py:29)
作用：
把机器人各 body 的位置，转换到机器人自己的 anchor 坐标系下

直白理解：
不直接看“世界坐标里手在 x=1.2”
而是看“手相对 torso 在哪”
这非常重要，因为 tracking 更关心身体内部构型，而不只是绝对世界位置。
输出：
所有 body 相对 anchor 的位置拼平后的向量
robot_body_ori_b
作用：
把机器人各 body 的朝向，也转换到机器人 anchor 坐标系下

直白理解：
告诉 policy：四肢、身体各部分相对主干的姿态关系
这比只给 joint angle 更“几何化”。
C. 参考动作相对当前机器人的目标位置/朝向
这组是最关键的，因为它们把 commands.py 产生的 reference 真正变成了 observation。
motion_anchor_pos_b
见 [observations.py (line 54)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/observations.py:54)
作用：
把 reference motion 的 anchor 位置，表达成“相对当前机器人 anchor”的坐标

直白理解：
不是单纯告诉 policy “参考动作 anchor 在世界坐标哪”
而是告诉它：
“目标 anchor 相对你现在 anchor 偏到哪里去了”
这个非常像控制里的误差信号。
可以理解成一部分：
target - current
motion_anchor_ori_b
作用：
把 reference motion 的 anchor 朝向，表达成“相对当前机器人 anchor”的朝向差

直白理解：
告诉 policy：
“目标朝向和你当前朝向差多少”
这也是 tracking 里非常核心的输入。
D. 参考动作自身的动态信息
ref_base_lin_vel_b
见 [observations.py (line 78)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/observations.py:78)
作用：
取 reference anchor 的线速度，并转到参考动作自己的 base frame 里

直白理解：
告诉 policy：参考动作此刻希望身体怎么动
不是只看姿态，还看运动趋势。
这很重要，因为 tracking 不是静态摆 pose，而是动态跟随。
ref_projected_gravity
作用：
把重力向量投到 reference motion 的 base frame 下

直白理解：
这个特征本质上在描述：
参考动作的姿态相对于“竖直方向”是什么样的
很多 locomotion / humanoid policy 里都会用 projected gravity 来表达姿态信息，因为它比欧拉角稳一点。
E. 已经对齐好的 reference body 目标
body_pos_relative_w
见 [observations.py (line 101)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/observations.py:101)
作用：
直接取 commands.py 中已经计算好的、相对对齐后的 reference body 位置

直白理解：
commands.py 已经把 reference body 做过 anchor 对齐
这里 observation 直接把这些“目标 body 位置”拿出来用
body_quat_relative_w
作用：
直接取 commands.py 中已经对齐好的 reference body 朝向

直白理解：
给 policy 一个更完整的“全身目标姿态”
selected_keypoints_pos_w_heading
作用：
取一部分被选中的关键点位置特征

这个更像是某些特定配置/实验里才会用的特征，不是最基础主线。
你现在先不用把它放在最核心位置记。
5. 它和 commands.py 的关系
这是你现在最该抓住的一点。
commands.py 做什么
它先准备：
当前参考关节位置
当前参考关节速度
当前参考 body 状态
当前参考 anchor 状态
对齐后的目标
也就是先定义：
ref_t
observations.py 做什么
它从这些 reference 里抽出 policy 真正该看的特征，再和机器人当前状态特征一起拼装。
也就是定义：
o_t
所以关系是：
motion -> commands.py -> ref_t -> observations.py -> o_t
6. 它在默认 policy observation 里怎么被用
在 [tracking_env_cfg.py (line 170)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/source/whole_body_tracking/whole_body_tracking/tasks/tracking/tracking_env_cfg.py:170) 默认 PolicyCfg 里，policy 看到的是：
command
motion_anchor_pos_b
motion_anchor_ori_b
base_lin_vel
base_ang_vel
joint_pos
joint_vel
actions
这里面：
来自 reference / command 的
command
motion_anchor_pos_b
motion_anchor_ori_b
来自机器人当前状态的
base_lin_vel
base_ang_vel
joint_pos
joint_vel
actions
所以默认 policy observation 的直白理解是：
“我现在身体什么状态 + 目标动作相对我差在哪 + 我上一步做了什么”

这就足够让策略做闭环控制了。
7. 为什么 observations.py 很重要
因为 policy 学的是：
a_t = pi(o_t)
不是：
a_t = pi(full_state_t)
所以 observation 设计其实在决定：
policy 能不能看懂当前自己在哪
policy 能不能知道目标在哪
policy 是否知道该怎么从当前状态往目标收敛
如果 observation 没有“当前状态”和“目标误差”这两类信息，tracking 就会很难学。
8. 用最直白的话总结输入输出
输入
observations.py 吃的是：
环境当前状态 env
command term（通常是 "motion"）
间接包含：当前机器人状态
当前 reference motion 状态

输出
observations.py 吐的是：
一项项 observation feature tensor
每项通常是 [num_envs, D]
后面再由 tracking_env_cfg.py 拼成最终的 policy observation 向量
9. 用一句类比记住它
如果说：
motion npz 是老师整套标准动作
commands.py 是助教告诉你“现在该做到哪一帧”
那么：
observations.py 就是在把“你现在的动作”和“老师当前这一帧的目标”翻译成学生大脑真正能用的数字提示。

它不决定目标本身，它决定：
目标和当前状态，以什么形式被 policy 看见。

10. 最简数学版
它做的事情可以抽象成：
(state_t, ref_t) -> o_t
其中：
state_t 是机器人当前真实状态
ref_t 是 commands.py 给出的当前参考目标
o_t 是 policy 最终看到的输入

### rewards.py的作用，输入和输出是什么，直白讲解。
rewards.py 这块如果只说一句话，那就是：
它负责给 policy 打分：这一刻你跟 reference motion 跟得好不好。

也就是在训练里定义每一步的奖励：
r_t
前面我们已经有了：
commands.py：准备当前目标 ref_t
observations.py：构造 policy 看到的输入 o_t
那么 rewards.py 做的就是：
把 机器人当前状态
和 当前参考动作状态
拿来比较，然后输出一个分数。
1. rewards.py 的作用
最直白理解：
policy 做了一个动作，机器人动了一下。
rewards.py 来判断：这一下动得是不是更像目标动作了。

所以它回答的是：
身体位置像不像？
身体朝向像不像？
速度像不像？
anchor / root 动得像不像？
有没有出现不希望的接触？
脚部接触节奏对不对？
然后把这些判断变成数值奖励。
2. 它的输入是什么
rewards.py 里的函数一般都长这样：
def xxx(env, command_name, std, ...):
    command = env.command_manager.get_term(command_name)
    ...
    return reward
所以它的输入主要有三类。
2.1 环境 env
环境里包含：
当前机器人真实状态
当前 reference command
传感器状态（比如接触传感器）
所以 reward 函数并不直接读文件，而是通过环境拿当前时刻的信息。
2.2 command_name
通常是 "motion"。
作用是：
去环境里拿当前这条 reference motion 的 command term

也就是去拿 commands.py 产出的：
anchor_pos_w
anchor_quat_w
body_pos_w
body_quat_w
joint_pos
joint_vel
机器人对应的真实状态
2.3 附加参数
不同 reward 会有不同参数，比如：
std
body_names
threshold
sensor_cfg
这些参数决定：
对误差有多敏感
比较哪些 body
接触阈值是多少
用哪个传感器
3. 它的输出是什么
每个 reward 函数的输出本质上都是：
每个环境一个标量 reward

也就是一个 tensor：
形状通常是 [num_envs]
比如 4096 个并行环境，就返回 4096 个奖励值。
这点很重要：
rewards.py 里的每个函数通常只负责一项局部打分，
最终训练用的总 reward 不是单个函数，而是多项加权和。

这个加权和是在
[tracking_env_cfg.py (line 306)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/source/whole_body_tracking/whole_body_tracking/tasks/tracking/tracking_env_cfg.py:306)
里配的。
所以结构是：
总奖励 = w1*r1 + w2*r2 + w3*r3 + ...
4. rewards.py 里到底在比什么
我按最重要的几类给你拆。
A. Anchor / 根部 跟踪奖励
motion_global_anchor_position_error_exp
见 [rewards.py (line 15)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/rewards.py:15)
作用：
比较 reference anchor 位置 和 机器人当前 anchor 位置

直白理解：
看机器人主干 / 根部有没有跟到目标位置
公式大意是：
error = ||ref_anchor_pos - robot_anchor_pos||^2
然后变成指数奖励：
reward = exp(-error / std^2)
特点：
完全对齐时，reward 接近 1
误差越大，reward 越接近 0
motion_global_anchor_orientation_error_exp
作用：
比较 reference anchor 朝向 和 机器人当前 anchor 朝向

直白理解：
看机器人主干转向是不是对的
也是同样套路：
reward = exp(-orientation_error^2 / std^2)
B. 全身 body 跟踪奖励
motion_relative_body_position_error_exp
见 [rewards.py (line 25)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/rewards.py:25)
作用：
比较 reference 各 body 的相对位置 和 机器人各 body 的当前位置

直白理解：
看四肢、躯干、手脚这些 body 的整体构型像不像
它默认不是直接硬比世界绝对位置，而是用 commands.py 里已经对齐过的 body_pos_relative_w。
这说明它更关心：
身体动作构型是否对，而不只是世界坐标下碰巧到某处

motion_relative_body_orientation_error_exp
作用：
比较 reference 各 body 朝向 和 机器人各 body 朝向

直白理解：
不只是位置对不对，还看身体各部分转得对不对
比如：
手臂是不是抬对角度
torso 姿态是不是一致
腿的朝向是不是像 reference
C. 速度跟踪奖励
motion_global_body_linear_velocity_error_exp
作用：
比较 reference 的 body 线速度 和 机器人真实 body 线速度

直白理解：
看它不只是“站得像”，还要“动得像”
这点很重要，因为 tracking 是动态任务，不是静态摆 pose。
motion_global_body_angular_velocity_error_exp
作用：
比较 reference 的 body 角速度 和 机器人真实 body 角速度

直白理解：
看身体各部分旋转的节奏是不是像 reference
motion_anchor_linear_velocity_error_exp
作用：
专门比较 anchor / base 的线速度

直白理解：
看主干整体移动速度跟没跟上
这个在 expert / MOSAIC 风格 reward 里也有单独强化。
D. 接触 / 步态相关奖励
feet_contact_time
作用：
根据接触传感器，奖励合适的脚接触时间

这是比较偏 locomotion 风格的一个奖励项。
contact_feet
见 [rewards.py (line 287)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/rewards.py:287)
作用：
比较当前双脚的接触状态，是否和 reference 一致

直白理解：
参考动作里左脚该着地时，你是不是也着地
参考动作里右脚该腾空时，你是不是也腾空
它的判断方式很简单：
看脚的高度是否低于某个阈值
低于阈值就认为在 contact
当前脚状态和 reference 脚状态一致就给奖励
这本质上是在鼓励：
接触节奏和 gait phase 对齐

E. MOSAIC / teleop 风格奖励
这些函数名里带 teleop_，更偏向 MOSAIC 的遥操作场景。
teleop_body_position_extend
作用：
把 upper body 和 lower body 分开打分

直白理解：
上半身和下半身的重要性可以分开控制
更细粒度地要求全身动作一致
teleop_vr_3point
作用：
专门比较头和双手这 3 个关键点

直白理解：
对 VR / teleop 场景很自然
因为很多接口最直接关心的就是 head + hands
teleop_body_position_feet
作用：
专门加强双脚位置的精确跟踪

直白理解：
对步态稳定、落脚准确很重要
teleop_body_rotation_extend
作用：
比较全身 body 的旋转一致性

teleop_body_velocity_extend
作用：
比较全身 body 的线速度一致性

teleop_body_ang_velocity_extend
作用：
比较全身 body 的角速度一致性

5. 为什么很多 reward 都长成 exp(-error / std^2)
这是这个文件里最核心的数理模式。
很多 reward 都是先算误差，再过一个指数函数：
reward = exp(-error / std^2)
直白理解：
误差很小时，reward 接近 1
误差逐渐变大时，reward 平滑下降
不会像硬阈值那样一下子断掉
这里的 std 可以理解成“容忍尺度”：
std 小：要求更严格，一点误差就掉分
std 大：要求更宽松，误差大一点也还能给分
所以 std 决定了 reward 的敏感度。
6. 它和 commands.py 的关系
这是必须连起来看的。
commands.py 给了什么
它给了：
当前参考 anchor 状态
当前参考 body 状态
当前参考 joint 状态
当前机器人真实状态
对齐后的 relative target
rewards.py 做什么
它拿这些量做差，计算误差：
ref - robot
再把误差变成 reward。
所以它本质上在做：
(state_t, ref_t) -> r_t
更展开一点是：
robot_state_t 和 reference_state_t
经过误差函数
变成
reward_t
7. 真正训练时 reward 怎么组成
这一点很重要：
rewards.py 只是定义“单项评分函数”。
真正训练时，哪些项启用、每项权重多大，是在
[tracking_env_cfg.py (line 306)](/home/yhl/Desktop/BAAI-Humanoid/MOSAIC/source/whole_body_tracking/whole_body_tracking/tasks/tracking/tracking_env_cfg.py:306)
里配置的。
比如基础 RewardsCfg 里有：
motion_global_anchor_pos
motion_global_anchor_ori
motion_body_pos
motion_body_ori
motion_body_lin_vel
motion_body_ang_vel
undesired_contacts
以及 action/joint/torque 等 penalty
而 RewardsExpertCfg 里又加了：
motion_anchor_lin_vel
teleop_body_position_extend
teleop_vr_3point
teleop_body_position_feet
...
所以训练的总 reward 更像：
r_total = tracking_reward + teleop_reward + stability_penalty + contact_penalty + ...
这就是为什么不同 task 能学出不同行为风格。
8. 用最直白的话总结输入输出
输入
rewards.py 吃的是：
环境 env
当前 command term（通常是 motion）
可能的附加配置：std
body_names
threshold
sensor_cfg

它间接使用的信息包括：
当前机器人状态
当前 reference motion 状态
接触传感器状态
输出
rewards.py 吐的是：
每个环境一个 reward 标量
形状通常是 [num_envs]
这些是局部 reward 项，后面会被加权合成总 reward。
9. 用一句类比记住它
如果说：
motion npz 是老师的标准动作
commands.py 是助教告诉你“现在应该做到哪一帧”
observations.py 是把“你当前状态 + 目标差异”翻译给大脑
那么：
rewards.py 就是裁判，专门负责在每一步打分：
“你这一下做得像不像、稳不稳、节奏对不对。”

10. 最简数学版
它做的核心事情可以写成：
r_t = R(state_t, ref_t)
其中：
state_t 是机器人当前真实状态
ref_t 是 commands.py 给出的当前参考目标
R 是由多个 reward term 加权组成的总评分函数
一句压轴总结：
rewards.py 的本质，就是把“当前机器人状态和参考动作的差异”翻译成一个可优化的数值信号，让 PPO 知道什么叫更像 reference，什么叫更差。

### 3.1 为什么这是 RL，不是普通监督学习

因为动作跟踪不是单步映射问题。

不是说“这一帧参考动作长这样，就直接回归一个关节命令”这么简单。  
原因是：

1. 机器人动力学是时序耦合的
2. 当前动作会影响未来状态
3. 稳定性、平衡、接触、惯性这些都跨时间累积
4. 同样的目标姿态，在不同当前状态下需要不同控制

所以更合理的目标不是单步拟合，而是最大化长时段累计回报：

`J(pi) = E[Σ_t gamma^t r_t]`

这里：

- `pi` 是策略
- `r_t` 是每一步 tracking + stability reward
- `gamma` 是折扣因子

这就是为什么项目把训练放在 `Isaac Lab / Isaac Sim` 里做，而不是简单做离线回归。

### 3.2 single motion 和 multi motion 的区别

这个区别很重要。

#### single motion tracking

- 只学一段或一类动作
- 目标相对集中
- 容易训起来
- 更像“专项模仿”

#### multi motion / general tracking

- 同时学很多动作
- 策略必须学会更一般的 tracking 能力
- 难度更高
- 但泛化更强

从函数视角看：

- single motion 更像在拟合一个较窄分布 `p_ref`
- multi motion 是在更大的动作分布上优化期望性能

也就是：

`max E_{motion ~ D, t}[r_t]`

其中 `D` 是 motion 数据集分布。

所以 `General-Tracking-Flat-G1-v0` 这类 task，本质是在训练“通用 tracker”，而不是只记住一段动作。

---

## 4. 第三步：为什么 README 里先训练 GMT

GMT 可以先理解成这个项目里的“通用 tracking 底座”。

也就是说，项目不是一开始就直接学“适配各种 teleop 输入的最终策略”，而是先学一个：

“只要给我标准 reference motion，我就尽量能跟上的通用全身动作跟踪器”。

这一步特别像先训练一个基础模型。

### 4.1 为什么这样设计是合理的

因为 teleoperation 的难点其实有两层：

1. 怎么把输入接口信号变成合理的运动目标
2. 怎么让机器人真的稳定执行这个目标

MOSAIC 把这两件事拆开了。

先解决第 2 件事：

- 我先训一个强的 tracker
- 只要 reference 足够合理，它就会尽量跟上

然后再解决第 1 件事：

- 不同 teleop 接口来时，我只要想办法把接口信息转换成 tracker 能接受的目标或修正信号

这种拆法在工程上很强，因为它降低了问题耦合。

---

## 5. 第四步：Adaptor 在干嘛

直白说，adaptor 就是“翻译层”或“接口适配层”。

因为不同 teleoperation 接口给你的输入形式可能很不一样，比如：

- VR 姿态
- 惯性动作捕捉
- 稀疏末端输入
- 带噪声、缺失或偏差的观测

这些输入通常不能直接当成干净、完整的 reference motion。

所以 adaptor 的工作就是：

把“接口侧输入”变成“tracker 更容易利用的表示”。

你可以把它理解成一个映射：

`z_t = f(interface_t)`

或者更具体一点：

`ref'_t = f(interface_obs_t)`

这个 `f` 就是 adaptor。

### 5.1 数理上它为什么必要

因为 interface distribution 和 training motion distribution 往往不一致。

也就是：

`p(interface input) != p(clean reference motion)`

如果直接把前者塞给 tracker，性能通常会掉。  
adaptor 的意义就是缩小这个 distribution gap。

---

## 6. 第五步：Residual Adaptation 在干嘛

这一步可以理解成：

“通用 tracker 已经很强了，但面对具体接口时还差一点，于是加一个残差修正”。

形式上很像：

`a_t = a_t^base + Δa_t`

或者：

`target_t = target_t^base + Δtarget_t`

这里：

- `base` 是 GMT 或已有策略给出的基础输出
- `Δ` 是针对特定接口、特定场景学出来的修正量

### 6.1 为什么残差学习常有效

因为如果基础策略已经不错，那么新任务不需要重学全部控制规律，只需要学“偏差”。

这在优化上更容易，因为学习目标从“大范围函数拟合”变成“小修正”：

`learn residual instead of relearn everything`

数学上相当于把复杂函数分解成：

`f_total = f_base + f_residual`

如果 `f_base` 已经解释了大部分行为，`f_residual` 的学习负担就小很多。

这也就是 README 里说 “rapid residual adaptation” 的直观含义。

---

## 7. 第六步：Distillation 在干嘛

`distillation` 可以先理解成“把复杂老师的能力压到学生里”。

比如：

- teacher 可能更强、更大、信息更多
- student 可能更轻、更统一、部署更方便

训练时让 student 去模仿 teacher 的输出或中间行为。

形式上常见是最小化：

`L_distill = ||pi_student(o) - pi_teacher(o)||`

当然在 RL 里还可能混合 environment reward，一起优化。

### 7.1 在 MOSAIC 里的直觉

这里的 distillation 很可能不只是做模型压缩，更像做“能力整合”：

- 把专家策略能力
- 把多 teacher 能力
- 把适配阶段的能力

收敛到一个更统一的 student policy 里。

所以可以把它看成训练流程后段的“整合器”。

---

## 8. 第七步：为什么 GMT 阶段用特殊 task，而不是普通 tracking task

`run/run_mosaic_gmt.sh` 用的不是最普通的 `General-Tracking-Flat-G1-v0`，而是：

`General-Tracking-Flat-G1-Wo-State-Estimation-v0-World-Coordinate-Reward`

这说明 GMT 阶段的目标函数和观测设计不是随便选的。

直白理解：

- `Wo-State-Estimation` 说明这个阶段可能刻意去掉某类状态估计输入
- `World-Coordinate-Reward` 说明 reward 更强调世界坐标系下的全局一致性

这和 README 里那句 “emphasize global motion consistency for stable, long-horizon behaviors” 是对得上的。

也就是说，GMT 阶段追求的不是“局部帧像就行”，而是更看重：

- 全局运动方向
- 身体整体一致性
- 长时间稳定跟踪

从数理上说，就是 reward 设计会影响最优策略的归纳偏好。  
如果 reward 更强调 world-frame consistency，那么策略更可能学到稳定、整体协调的动作，而不是只在局部关节误差上刷分。

---

## 9. 现在真正需要掌握的“项目使用框架”

如果从“怎么用这个仓”角度，先记住这条操作链就够了：

1. 先准备 motion 数据
2. 用脚本转成 `.npz`
3. 选一个 task id
4. 用 `scripts/rsl_rl/train.py` 开训
5. 用 `scripts/rsl_rl/play.py` 回放评估
6. 如果走 MOSAIC 完整流程，再按 `run/*.sh` 的阶段化脚本继续

也就是：

`motion -> task -> train.py -> checkpoint -> play.py -> 更高级训练阶段`

---

## 10. 下一步最自然应该学什么

如果目标是“尽快真的会用这个项目”，下一轮最值得看的三个文件是：

- `scripts/csv_to_npz.py`
- `scripts/rsl_rl/train.py`
- `source/whole_body_tracking/whole_body_tracking/tasks/tracking/config/g1/flat_env_cfg.py`

分别对应三个关键问题：

1. `.csv` 到 `.npz` 具体多了什么字段
2. 训练脚本到底怎么把 motion 塞进环境
3. 环境到底给 policy 什么 observation、什么 reward

这样就能从“知道流程”进入“真的会用”。
