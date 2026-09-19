# 青瞳实时规划与 HITTER Policy 接入设计

## 目标

在不改变 HITTER ONNX 模型、105 维 observation 顺序、29 维 action
接口和 50 Hz 真机控制频率的前提下，完成以下四项改造：

1. 使用青瞳 360 Hz 球数据持续更新估计器，并由后台 planner 始终处理最新状态；
2. 使用 Avatar SDK 输出的 `G1Pelvis` 刚体根位姿，不再用刚体 marker 质心代替 pelvis；
3. 按训练时序在约 `0.90 s` arm，并在 arm 后继续用同一来球的最新有效规划覆盖击球目标；
4. 明确球过 `x=0`、出界、丢失以及连续回合时的球状态生命周期，避免旧球重复触发。

本设计延续
`2026-07-11-chingmu-table-calibration-design.md` 中已经确认的桌面坐标系和实测桌面尺寸，
但覆盖其中以下旧结论：

- `G1Pelvis` 位置不再由 marker 模板刚体拟合得到；
- `simulator/real_world.py` 不再保持完全不变，而是提供线程安全的球状态快照；
- planner/policy 接入不再属于非目标，而是本设计的核心范围。

## 已确认的原则

- 青瞳 SDK、球估计器和 planner 输入以 360 Hz 更新。
- planner 使用容量为 1 的 latest-only 通道；忙不过来时丢旧帧，不积压。
- ONNX policy 和真机 action 保持 50 Hz，不提高电机命令频率。
- `TTS > 0.90 s` 时，policy 继续接收默认 waiting observation，其中用户指定
  waiting TTS 固定为训练上限 `0.92 s`。
- 首个有效规划落入 `0.80 s <= TTS <= 0.90 s` 时 arm。
- 首个有效规划已经 `TTS < 0.80 s` 时，整颗球跳过，不进行晚启动。
- arm 后不锁死目标；同一颗来球的更新可以继续原子覆盖完整击球命令。
- policy 始终运行，不添加 action gating 或隐藏的动作抑制。
- 球过机器人侧桌边即击球平面 `x=0` 后，对该球停止规划并清理球状态。
- 不增加 `0.20 s` 或其他墙钟缺包超时。

## 坐标系与桌面几何

所有进入 estimator、planner 和 policy 目标计算的数据都使用桌面固定世界系：

- 机器人侧桌边中心：`x=0, y=0`；
- `+x`：由机器人侧指向对手侧；
- `+y`：机器人面向球桌时的左侧；
- `+z`：竖直向上；
- 桌面高度：`z=0.760000 m`；
- 击球平面：`x=0`。

沿用已确认的实测有效桌面尺寸：

```text
table_length = 2.730738 m
table_width  = 1.512451 m
table_center = [1.365369, 0.0] m
y_bounds     = [-0.7562255, 0.7562255] m
```

桌角 marker 只负责启动时拟合并冻结桌面坐标变换；运行中不随帧重新标定。
planner、MuJoCo HITTER 配置和青瞳 bridge 必须引用同一组桌面几何值。

## `G1Pelvis` 真刚体位姿

### SDK 数据保留

青瞳 hierarchy 将名称 `G1Pelvis` 解析为动态 numeric body ID。SDK tracker
回调中，`sensor == body_id` 的 report 就是该刚体根位姿。当前代码把 hierarchy
sensor report 排除，随后用 `7800 + 50 * body_id` 范围内的 marker 拟合刚体，
导致发布位置成为 marker 质心而不是 Avatar 刚体原点。

新的 `MocapFrame` 必须同时携带：

- `body_position_mm`；
- `body_quaternion_xyzw`；
- 对应 frame number 和 source timestamp；
- unlabeled markers；
- 可选的 body markers 仅用于诊断，不再参与正常 base pose 计算。

青瞳示例明确使用 quaternion `x, y, z, w` 顺序；进入 LCM 前仍需做归一化和
有限值检查。

### 位置与朝向转换

刚体根位置直接通过冻结的桌面变换转换到 RobotBridge 世界系，不添加 marker
质心偏移，也不添加隐藏 base anchor。这样 `root_trans_world` 表示 Avatar 中
实际定义的 `G1Pelvis` 原点。

设 `R_WQ` 为青瞳原始坐标到桌面世界系的旋转，`R_QB` 为 SDK 刚体旋转，则：

```text
R_WB = R_WQ * R_QB
```

为保持原 Vicon bridge 的 policy 语义，bridge 在第一帧有效 `G1Pelvis` 上记录
初始世界 yaw，发布相对于该初始 yaw 的 yaw-only quaternion。初始朝向因此为
单位四元数，后续机器人绕竖直轴的真实变化仍被保留。roll/pitch 继续由机器人
自身 state estimator 提供给 projected gravity，不从 mocap 替代。

若根位姿无效或缺失，不发布伪造 base pose；球数据流可以继续用于监控，但真机
policy 启动仍要求有效 base pose。

## 球输入与 planner 准入边界

### Estimator 输入

只有满足以下桌面空间约束的当前球样本进入 estimator：

```text
0 < x <= table_length
abs(y) <= table_width / 2
z > table_height
```

桌角 marker 先按保存的桌角位置排除，再按时间连续性选择移动球。方向不用于
阻止 estimator 工作：出球 `vx >= 0` 仍可被跟踪，只是不允许触发 planner。

### Planner 准入

planner 仅为满足全部条件的来球生成候选：

- 当前球尚未越过击球平面：`x > 0`；
- 来球方向：`vx < 0`，不额外设置最小来球速度；
- 预测轨迹从桌面正侧向机器人侧穿过 `x=0`；
- 预测击球点高度满足：
  `table_height < hit_z <= table_height + 0.50 m`；
- 预测值和完整命令均为有限数。

固定 `2 s` 不再作为独立的业务准入条件。数值 predictor 可以保留有限的安全
积分上限以避免无限计算，但成功条件由“正确方向穿过 `x=0`”定义。

当水平方向由出球 `vx >= 0` 反转为新来球 `vx < 0` 时，开始新的 incoming
track epoch，并清空旧方向样本，避免把对手击球前后的两段运动拟合在一起。

## 360 Hz latest-only 规划

### 线程边界

LCM/estimator 回调只完成以下轻量工作：

1. 接收并变换当前球样本；
2. 更新 `BallStateEstimator`；
3. 在锁内写入一个不可变 `BallEstimateSnapshot`；
4. 增加 generation 并唤醒 planner worker。

回调线程不运行轨迹 predictor 或 strike planner，以免 planner 计算阻塞后续
360 Hz SDK/LCM 数据。

`BallEstimateSnapshot` 至少包含：

- track epoch；
- source frame 和 estimator generation；
- 球位置、速度及 estimator ready 状态；
- 对应的本机 monotonic 接收时间；
- 同一时刻可用的 `G1Pelvis` 世界位置与朝向。

### 容量为 1 的请求槽

planner worker 阻塞等待 snapshot。若 worker 计算期间到达多个新 snapshot，
pending slot 只保留最后一个，覆盖中间旧状态。完成当前计算后，worker 立即处理
当时最新的 pending snapshot。

完成的规划结果也以不可变、容量为 1 的 latest-result snapshot 发布，包含：

- track epoch 和 source generation；
- 完整 `HitterSystemPlanner` command；
- 绝对 `strike_deadline_monotonic_s`；
- planner 完成时间和结果 age；
- 成功或失败原因。

worker 即使发现计算期间又来了新帧，也可以发布刚完成的结果，然后继续处理
最新 pending 状态；否则当 planner 平均耗时接近 360 Hz 帧周期时会发生永久
饥饿。policy 只接受比当前已应用 generation 更新的结果。

planner 异常只使当前 generation 失败，不杀死 SDK 回调、policy 主循环或 planner
线程。下一份最新 snapshot 到来后继续计算。

## TTS 与 policy 命令状态机

### 训练语义

当前部署 ONNX 与训练导出的模型一致。训练时：

- 初始 TTS 从 `[0.80, 0.92] s` 采样；
- actor observation 直接包含 TTS；
- TTS 每个 50 Hz policy step 递减并 clamp 到 `0`；
- 击球目标在一次 strike 内保持有效到总 swing duration
  `[1.75, 1.95] s` 结束。

真实击球命令和 waiting observation 都不把 `TTS > 0.92 s` 或负 TTS 送入
actor；waiting observation 使用训练范围上限 `0.92 s`。

### Waiting observation

在没有已 arm 命令时，使用用户指定的默认输入：

```text
waiting_target_w    = [-0.4, 0, z_home]
base_target_pos     = R_yaw^-1 * (waiting_target_w - pelvis_position_w)
racket_target_pos   = 当前 FK 球拍位置（pelvis/yaw 相对坐标）
racket_target_vel   = [0, 0, 0]
time_to_strike      = 0.92 s
```

`z_home` 在第二次 R2 通过有效 `G1Pelvis` 检查并完成校准后捕获；每帧和每颗球结束
时都不刷新，只在下一次完整校准/硬重置后重新捕获。世界目标和 planner 的击球
base target 独立保存，避免 `RECOVERY -> WAITING` 后继续追上一球的击球目标。

这只是 observation 状态，不停止 ONNX 推理，也不拦截或替换 policy action。

### 状态定义

命令生命周期包含四个状态：

1. `WAITING`：无当前球候选，policy 使用默认输入；
2. `TRACKING`：360 Hz estimator/planner 已跟踪球，但尚未达到 arm 时间，policy
   仍使用默认输入；
3. `ARMED`：policy 使用 planner 命令，同一来球允许最新有效结果持续覆盖；
4. `RECOVERY`：球已消费，停止该球规划，TTS 保持 `0`，保留最后命令完成收拍。

这四个状态描述的是当前 policy command。球输入侧的 track epoch 独立存在，
因此 `RECOVERY` 期间可以在后台跟踪下一颗球；它不会替换仍在恢复的当前命令，
只会保存为下一击候选。

### Arm 规则

对每个新的 incoming track：

- 延迟修正后的最新 TTS `> 0.90 s`：保持 `TRACKING`；
- 首个有效结果进入 `0.80 s <= TTS <= 0.90 s`：进入 `ARMED`；
- 尚未 arm 时，最新结果已经 `< 0.80 s`：将该 track 标记为 skipped，直到
  下一 track 都不 arm；这也覆盖 TTS 从 `> 0.90 s` 一次跳过整个 arm 区间的情况；
- 不要求采样恰好等于 `0.900 s`，例如从 `0.904 s` 跳到 `0.897 s` 应正常 arm。

arm 时一次性写入完整命令，禁止只更新部分字段。完整命令包括：

- strike type；
- base target；
- 世界系球拍击球点；
- 世界系球拍目标速度；
- desired outgoing ball velocity；
- absolute strike deadline；
- track epoch 和 source generation。

### Arm 后持续覆盖

`ARMED` 后接受更新必须满足：

- 属于同一 track epoch；
- source generation 更新；
- 仍满足 `x>0`、`vx<0`、穿越方向和击球高度边界；
- 延迟修正后的 TTS 在 `[0, 0.92] s`；
- 完整命令有限且有效。

满足条件时，位置、速度、base target、strike type 和 deadline 作为一个原子
snapshot 一起覆盖。允许 TTS 随新预测小幅增加或减少，这是用户明确选择的
实时修正行为。

以下情况保留上一个有效命令，不退回 waiting：

- 当前 planner generation 失败；
- 结果属于旧 generation 或旧 track；
- 新 TTS 又跳到 `>0.92 s`；
- 新命令不满足 planner 边界。

### TTS 时钟与 50 Hz policy 读取

不再用 `elapsed += 0.02` 作为真机击球 deadline 的唯一依据。worker 根据
snapshot 的 monotonic 时间和相对 TTS 生成绝对 deadline：

```text
strike_deadline = snapshot_monotonic_time + planned_time_to_strike
policy_tts      = clamp(strike_deadline - policy_read_monotonic_time, 0, 0.92)
```

这样 planner 排队、计算和等待下一个 policy tick 的时间都会自然扣除。
上述 clamp 用于已经 arm 的真实击球命令；waiting observation 直接使用固定
`0.92 s`。

当前 agent 是先用上一轮 observation 推理、再在 `env.step()` 后生成下一轮
observation，导致真机输入约落后一整个 20 ms 周期。HITTER 真机 loop 必须在每次
ONNX 推理前刷新一次最新 observation；MuJoCo loop 不做额外刷新。ONNX 和 action
发布频率仍为 50 Hz。

## 球过平面、丢失与连续回合

### 显式结束当前球

bridge 在已经跟踪球的情况下，如果当前 SDK frame 明确没有有效候选，发布一次
`valid=false` 的 ball message；这属于当前 source frame 的显式状态，不是墙钟
超时。

BallTracker 在做桌内过滤前保留跨越 `x=0` 的关联判断。检测到当前球从 `x>0`
越过到 `x<=0` 时，发布显式无效/结束消息并关闭该 track。`x<=0` 样本本身不再
进入 estimator 或 planner。

### 消费球与恢复动作

球结束对命令状态的影响分开处理：

- 对 estimator/planner：立即清空该 track 的样本、pending request 和 candidate；
- 对已经 `ARMED` 的 policy command：不在过平面瞬间撤销；
- 到达 strike deadline 后，TTS 保持 `0` 并进入 `RECOVERY`；
- arm 时从 `[1.75, 1.95] s` 采样总 swing duration，并计算
  `recovery_duration = sampled_swing_duration - arm_tts`；
- arm 后 planner 每次覆盖 strike deadline 时，同步设置
  `command_end_deadline = latest_strike_deadline + recovery_duration`；
- 到达 command end deadline 后，命令回到 `WAITING`。

这避免旧球继续重规划，同时保留 policy 训练过的击球后收拍/恢复阶段。

如果球在 arm 后短暂丢失，停止接受新覆盖，但已有绝对 deadline 和命令继续
执行；如果从未 arm 就丢失，则直接回到 `WAITING`。

### 连续击球

完成机器人侧击球后，重新进入桌内且 `vx>=0` 的球可以继续由 estimator 跟踪，
但 planner 不 arm。对手将球打回、检测到方向从非负变为负时，重建 incoming
fit window 和 track epoch。上一击处于 `RECOVERY` 时仍可缓存新球候选；恢复
结束时只有仍处于 `[0.80, 0.90] s` arm 区间的候选可以成为下一击，已经晚于
`0.80 s` 的候选跳过。

## Real-world 与 MuJoCo 的一致性

共享以下命令语义和 planner 边界：

- waiting observation；
- `0.90 s` arm、`0.80 s` late-skip 和 `0.92 s` actor 上限；
- `vx<0`、正确方向穿平面和击球高度边界；
- arm 后完整命令覆盖；
- strike deadline、TTS clamp 和 recovery 生命周期。

数据生产频率按 backend 保持不同：

- real-world：青瞳/estimator 360 Hz，后台 latest-only planner，policy 50 Hz；
- MuJoCo：从精确仿真状态在 policy tick 同步规划，policy 50 Hz。

MuJoCo 不需要模拟青瞳 estimator 或启动额外 planner 线程，但它看到的 policy
命令状态机与真机相同。

## 配置项

HITTER 配置显式保存以下值，不把它们散落成代码常量：

```text
source_rate_hz                  = 360.0
state_estimator_sample_rate_hz  = 360.0
waiting_time_to_strike_s        = 0.92
waiting_base_target_xy_w        = [-0.4, 0.0]
arm_time_to_strike_s            = 0.90
minimum_arm_time_to_strike_s    = 0.80
maximum_policy_time_to_strike_s = 0.92
swing_duration_range_s          = [1.75, 1.95]
```

桌面几何、反弹系数、drag、ball radius 和击球平面继续由共享
`motion.ball_planner` 配置提供。没有 stale-data timeout 配置。

## 并发与关闭

- snapshot/request/result 均通过一个小型锁和 condition 管理；锁内只复制小型
  NumPy array 和元数据，不运行 planner。
- 所有跨线程 array 在发布前复制并设为不可变语义，消费者不得原地修改。
- worker 使用显式 stop event；env 关闭或进程退出时先唤醒再 join。
- 重置 estimator、track epoch 或 env 时，同时失效旧 pending/result generation，
  防止上一颗球的异步结果晚到后重新 arm。

## 诊断信息

运行时以限频日志和计数器暴露：

- SDK source frame、实际接收频率和 frame gap；
- estimator sample count、ready、bounce 和 direction reset；
- planner submitted/completed/failed/dropped-pending 数量；
- latest result age、track epoch 和 generation；
- `WAITING/TRACKING/ARMED/RECOVERY` 状态转换；
- arm、override、late-skip、hit-plane-consume 的原因；
- policy 实际读取的 TTS 和 absolute deadline。

诊断不得在 360 Hz 每帧打印；周期状态限频，状态转换即时打印。

## 错误处理

- `G1Pelvis` root report 缺失：不回退到 marker 质心，不发布伪造 base pose。
- SDK quaternion 非有限或近零：丢弃该 root sample 并报告。
- estimator 未 ready：保持 waiting，不调用 planner。
- planner 单帧失败：保留已 arm 命令或保持 waiting，下一 generation 继续。
- worker 异常退出：真机 policy 不自动切换到同步 360 Hz 规划；记录明确错误并保持
  waiting/最后有效命令。
- track reset：所有旧 epoch 结果必须被拒绝。
- 未 arm 的 TTS 小于 `0.80 s`：跳过整颗球，而不是临时放宽窗口；即使它此前
  曾经大于 `0.90 s` 但一次更新跳过 arm 区间，也执行相同规则。

## 自动化验证

使用标准库 `unittest` 覆盖以下行为。

### 青瞳 SDK 与坐标

- hierarchy 动态解析 `G1Pelvis` body ID；
- tracker 的 root report 被保存，不再进入 unlabeled 或被丢弃；
- SDK `xyzw` quaternion 正确转换到桌面世界系；
- 第一帧发布相对 yaw 为 0，后续 yaw 变化正确；
- base position 来自 root report，而不是 marker centroid；
- root 缺失时不做 marker-centroid fallback；
- 实测桌角转换到确认的 x/y 边界和 `z=0.76`。

### 球边界与生命周期

- 只有桌面 x/y 范围内且高于桌面的球进入 estimator；
- `vx>=0` 仍更新 estimator，但不能产生 planner candidate；
- `vx<0`、正确方向穿越和击球高度合法时才能规划；
- 击球高度高于桌面且不超过 `0.50 m`；
- 方向反转建立新 incoming epoch，不混用旧拟合样本；
- 过 `x=0` 或当前 frame 显式无球时发布 invalid 并清 estimator；
- 不依赖 `0.20 s` 墙钟超时。

### Latest-only worker

- worker 忙时 pending slot 只保留最新 generation；
- 完成结果可发布且随后立即处理最新 pending，不发生饥饿；
- planner 异常不终止 worker；
- reset/epoch change 后迟到结果不会被 policy 接受；
- 关闭流程可重复且不遗留线程。

### Policy 状态机与时序

- `TTS>0.90` 使用 waiting observation；
- 首次 `[0.80,0.90]` 结果 arm；
- 未 arm 时 `<0.80` 标记整 track skipped，包括一次更新跳过 arm 区间；
- arm 后同一 track 新结果完整原子覆盖；
- planner 失败、旧 generation、非法边界或 `TTS>0.92` 保留上次命令；
- absolute deadline 会扣除 planner 和 policy 等待时间；
- waiting policy TTS 精确为 `0.92`，ARMED/RECOVERY policy TTS 位于 `[0,0.92]`；
- planner 覆盖 strike deadline 时保持 arm 时确定的完整 recovery duration；
- 球过平面清 planner，但已 arm command 保留到 recovery 结束；
- 真机每次 ONNX 前刷新 observation，action 发布仍为 50 Hz；
- waiting 状态不抑制 ONNX action。

## 非驱动现场验证

在启动 policy 前完成：

1. 仅运行青瞳 bridge，确认发布的是 Avatar `G1Pelvis` 根位置，pelvis 高度接近
   实际刚体原点而不是 marker 质心；
2. 静止和小角度转动机器人，确认相对 yaw 连续且初始为 0；
3. 打一颗不启动机器人的球，确认 360 Hz frame、estimator 和 latest-only
   planner 计数正常；
4. 确认 `TTS>0.90` 时为 `TRACKING`，进入约 `0.90` 时 arm；
5. 确认 arm 后 override generation 增加，policy result slot 不积压；
6. 确认球过 `x=0` 后 estimator/pending candidate 清空，不再用旧球重规划；
7. 记录但不执行一轮完整 observation，确认 TTS 没有一周期约 20 ms 的固定滞后。

只有以上检查通过并确认 `base_valid=1` 后，才进入真机 policy 和击球测试。

## 预计修改边界

实现阶段预计只修改以下相关区域：

- `deploy/mocap_bridge/chingmu_sdk_client.py` 及其测试；
- `deploy/mocap_bridge/chingmu_table_lcm_bridge.py` 及其测试；
- `deploy/simulator/real_world.py`；
- `deploy/utils/hitter_planner.py`；
- `deploy/envs/hitter.py`；
- `deploy/agents/hitter_agent.py`；
- HITTER 专用 Hydra 配置和对应测试。

不会修改 ONNX 模型、policy observation 顺序、action 维度、Unitree `trans`
协议或其他任务的通用控制频率。

## 非目标

- 不把 policy 或电机控制提高到 360 Hz；
- 不加入 action gating；
- 不加入墙钟缺包超时；
- 不修改训练代码或重新训练模型；
- 不用 marker centroid 作为 `G1Pelvis` fallback；
- 不增加 spin/Magnus 等当前 planner 未建模的物理；
- 不在本设计阶段自动启动真机 policy 或执行击球。
