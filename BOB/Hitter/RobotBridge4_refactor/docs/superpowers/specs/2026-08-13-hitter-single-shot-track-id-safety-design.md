# HITTER 单拍轨迹 ID 与安全撤拍设计

## 背景

RobotBridge4 是从 RobotBridge3 当前工作树完整复制出的独立 Git 根。本设计只作用于 RobotBridge4，不修改 RobotBridge3、RobotBridge2、MOSAIC-main 或 Omega-Athlete。

当前真机链路把每一帧球位置交给本地 estimator，再用本地 `track_epoch` 区分球轨迹。这个边界存在四个已经确认的问题：

1. `ARMED -> RECOVERY` 时会重置 estimator 并递增本地 epoch。同一颗球撞网返回或继续可见时，可能被伪装成“下一颗球”。
2. C++ Vicon bridge 不会真正结束 `BallTrackState`，而 Python ChingMu bridge 又会在单帧丢失或球经过近端边界时过早结束轨迹；两个发布端的轨迹语义不一致。
3. 一次成功规划已经进入 `ARMED` 后，后续普通 planner failure 不会撤销旧命令。球撞网、停止或改变方向后，机器人仍可能执行撞网前的旧击球目标。
4. 同一轨迹的成功结果可以不断覆盖 active command；自动正反手选择没有锁定，目标或手型可能在短时间内改变。

此外，真机 WAITING 当前使用固定世界坐标 `[-0.4, 0.0]` 作为 base target。若希望“打完后停在当前地点”，直接把当前坐标每帧更新为目标会失去位置恢复能力；正确语义应是在进入 WAITING 的瞬间锁存一次世界坐标，并在整个 WAITING 期间保持不变。

## 目标

- 发布端为一段连续物理球轨迹提供稳定、明确的正整数 `track_id`。
- 同一个 `track_id` 在一个控制进程生命周期内最多触发一次击球。
- 球撞网、反向、越过近端边界、打回后继续可见或短暂丢帧时，不生成新 ID。
- 球失效后撤销尚未进入最后承诺窗口的旧命令，不再盲目执行撞网前的目标。
- 按球桌坐标系预测击球点的绝对 y 确定手型：`y < 0` 固定正手，`y >= 0` 固定反手。
- 第一次进入 `ARMED` 后锁定正反手和 base target，限制后续目标突变；进入承诺窗口后冻结整条命令。
- 打完、撤拍或无球时，真机在进入 WAITING 的当前位置保持，而不是返回固定桌边坐标，也不是让目标随机器人漂移。
- WAITING 仍然每个 policy tick 组装 104 维 observation、执行 ONNX、下发 PD 目标，不引入 action gating。
- 将 planner 负载和日志开销限制在不会争抢 50 Hz policy 控制周期的范围内。
- 新旧 LCM schema 在不同频道上隔离，禁止混合解码。

## 非目标

- 不把 Vicon 无标签点变成可跨遮挡、跨进程重启永久识别的物理球序列号。
- 不支持同一时刻多颗球的完整多目标跟踪；单拍模式要求场上只存在一颗候选球。
- 不直接判断“是否过网”或识别球网碰撞；安全决策使用轨迹连续性、方向、预测可达性和 freshness。
- 不修改 ONNX 模型、104 维 observation 布局、29 维 action、PD 增益或底层 Unitree `trans` 协议。
- 不允许 RobotBridge3 的 v1 publisher 与 RobotBridge4 的 v2 consumer 在同一频道滚动混跑。
- 不保证 mocap publisher 热重启后自动恢复击球。运行中 bridge 断流属于需要重新进入 policy 的故障。

## 方案选择

采用“发布端权威 `track_id` + v2 独立频道 + 接收端单拍消费状态机”。

没有采用以下方案：

- 继续由 RobotBridge 本地根据 `valid`、速度方向或 estimator reset 推断 epoch：这正是同一物理球被误认为新球的根因。
- 在原 `vicon_state_data` 频道直接替换 schema：LCM fingerprint 会改变，旧发布端或旧订阅端会持续 `Decode error`，并可能终止共享 poll 线程。
- 只增加 `track_id` 而不增加撤拍：它只能阻止第二次击球，不能阻止机器人执行已经保存的撞网前旧命令。

## LCM v2 契约

### Schema

在 `unitree_sdk2/lcm_types/transformation_t.lcm` 中增加：

```text
int64_t track_id;
```

随后从这一份 `.lcm` 源文件重新生成并提交 C++ `transformation_t.hpp` 和 Python `transformation_t.py`。生成文件不得手工修改。

字段约定如下：

| 消息 | `track_id` | `valid/occluded` | 含义 |
|---|---:|---|---|
| 有效 ball | `> 0` | `1/0` | 当前连续轨迹 |
| 轨迹结束 ball | 原轨迹的正整数 ID | `0/1` | 该 ID 明确结束，只发布一次 |
| pelvis/table | `0` | 保持现有语义 | 不参与球轨迹身份 |

ball 消息中的 `track_id <= 0` 是协议错误，接收端必须 fail closed：不送入 estimator、不提交 planner，也不自动生成本地 ID。

### 频道隔离

- RobotBridge4 的 active Vicon/ChingMu publisher、`RealWorld`、monitor 和在线 diagnostics 全部使用 `vicon_state_data_v2`。
- `vicon_state_data` 保留为 v1 历史频道；RobotBridge4 真机控制入口不订阅它。
- RobotBridge4 不提供双解码 fallback。频道名和 schema 必须同时匹配。
- 同一套真机运行只能启动一个 v2 mocap publisher，不能同时启动 C++ Vicon bridge 和 Python ChingMu bridge。
- `RealWorld` 必须在 callback 最外层捕获 decode/fingerprint 异常；异常不能退出共享 LCM poll 线程。任意 v2 decode error 都锁存 `VICON_SCHEMA_ERROR`、禁止下一拍，并要求重新进入 policy。

### ID 分配

发布端只在从无 active track 接纳第一颗候选球时分配 ID：

```text
track_id = max(last_allocated_track_id + 1, unix_time_us)
```

ID 在进程内严格递增，使用微秒 Unix 时间降低重启碰撞概率。接收端仍保存本进程见过和消费过的全部 ID；ID 重复时按旧轨迹处理，不重新击球。

publisher 热重启无法证明新 ID 对应新物理球，因此任意 v2 流断流都会在 controller 侧锁存通信故障。恢复发布后仍不自动重新接球，必须重新走真机 policy 进入流程。

## 发布端轨迹状态机

C++ Vicon bridge 与 Python ChingMu bridge 使用同一套状态和默认参数：

- `ball_track_association_radius_m = 0.35`
- `ball_track_end_timeout_s = 0.25`

两端的运行时球候选 ROI 也使用同一套球桌/world 条件：排除保存桌角
`corner_exclusion_radius_mm` 内的点，只保留
`x <= table_length_m - 0.40`、`|y| <= table_width_m / 2` 且
`z > table_height_m` 的有限点；新轨迹另要求 `x > 0`。C++ Vicon
允许额外配置 raw ignore sphere，但这些排除项只能缩小候选集合，不能绕过
上述公共 ROI。已有轨迹跨过近端 `x=0` 时仍可在关联半径内保持同一 ID。

C++ Vicon runtime 与 Python bridge 共用桌面标定 JSON，并通过
`--pelvis-orientation-calib` 强制加载同一套 rigid-to-pelvis 外参。发布姿态按
`R_world_pelvis = R_world_rigid @ R_rigid_from_pelvis` 计算，发布位置再加
`R_world_pelvis @ translation_rigid_origin_to_pelvis_origin_pelvis_m`；不能把
启动首帧归零或只发布相对 yaw。同一 Vicon 源帧的 pelvis、ball、table 消息
必须共用一次采样的 `publish_time_us`。pelvis 从 valid 变 invalid 时只发一条
invalid 转换消息，连续 invalid 帧不重复发布；ball track 在 pelvis invalid
期间冻结而不超时结束。

当前部署外参是 `G1Pelvis` rigid body 到发布名 `G2Pelvis` 的专用标定，C++
runtime 对这两个名称都做 fail-fast 校验，禁止把该外参静默套到其他 Vicon
subject。唯一不要求 pelvis JSON 的模式是 `--save-table-calib`、未加载旧桌面、
且 `--no-publish` 的纯桌面标定；保存完成后必须立即退出，不能继续 runtime。

关联预测和丢失时长都使用 mocap source frame/source time；source time 不可用时才按已校验的 source frame rate 换算。host wall clock 不参与球轨迹连续性判定。

状态只有 `INACTIVE`、`ACTIVE` 和 `MISSING_GRACE`：

1. `INACTIVE`：沿用现有 table/corner/高度 ROI；只有世界坐标 `x > 0` 的候选点可以创建新轨迹。接纳后分配正整数 ID。
2. `ACTIVE`：按上一位置和速度做常速度预测，在预测点 0.35 m 内选最近候选点。命中后保持原 ID 并更新位置、速度和帧号。
3. `MISSING_GRACE`：没有可关联候选时保留原 ID 和最后状态，不发布 invalid。0.25 s 内重新关联成功就回到 `ACTIVE`，ID 不变。
4. 连续缺失达到 0.25 s：发布一次 `valid=0, occluded=1`、携带原 ID 和最后有限位置的 ball 消息，然后清除 active track 并回到 `INACTIVE`。

以下事件都不能单独结束轨迹或分配新 ID：

- `x <= 0`；
- `vx` 改变正负；
- 台面 bounce；
- 球撞网后返回；
- planner success/failure；
- strike deadline；
- RobotBridge 本地 estimator reset。

当 active track 存在时，关联半径外的其他无标签点只能记为 miss，不能立即抢占当前轨迹。这样做优先保证“不误击第二颗/错误标记”，代价是复杂多球场景可能漏掉一次发球。

## 接收端身份与发球准入

`BallEstimateSnapshot`、planner result、lifecycle 和 diagnostics 中的身份字段统一命名为 `track_id`。`generation` 继续表示同一 ID 内的本地消息序号，不再承担轨迹身份。

`RealWorld` 保存：

- 当前 wire `track_id`；
- 当前 ID 内最后接受的 source frame；
- 本进程见过的 ID；
- 已消费 ID；
- 最后一个 v2 消息和最后一个有效 ball 消息的 monotonic 时间；
- `ready_for_new_serve` 与 no-ball 起始时间。

处理规则：

1. 同一 ID、递增 source frame 才能继续更新 estimator；重复或乱序帧被丢弃。
2. estimator reset 只清空拟合窗口和 readiness，不改变 ID，不创建新 generation 体系，也不清除 consumed 状态。
3. 已消费 ID 的后续有效帧可以进入 diagnostics，但不得提交 planner。
4. 当前轨迹尚未结束时收到另一个正整数 ID，视为 publisher 关联冲突。旧 ID 和新 ID 都不允许击球；新 ID 被加入 consumed 集合。
5. `RECOVERY` 期间出现的新 ID 可以继续进入 planner；只缓存该 ID 的最新成功结果，不改变上一拍的 recovery 命令。recovery 结束时再按当前时间和 `minimum_arm_time_to_strike_s` 重新判定，过晚则跳过。
6. controller 启动或重新进入 policy 时，如果已有球可见，则先消费这个 ID；必须等它结束后才能进入下一拍。
7. 若新 ID 只差 0.20 s no-ball 条件，则保存固定的 admit-after deadline，期间不喂 estimator、不通知 planner；到时重新检查 session、stream、pelvis 与故障条件，通过后从当前帧开始准入。其他硬条件失败仍立即加入 consumed 集合。

单拍下一发准入要求同时满足：

- lifecycle 已在 `WAITING`，或处于允许缓存下一球结果的 `RECOVERY`；
- 没有 active/visible ball；
- 已连续无球至少 `new_serve_no_ball_s = 0.20`；
- v2 数据流 freshness 正常；
- pelvis pose 有效。

只有在这些条件之后出现的未见过正整数 ID 才能启动新一拍。这个 no-ball 间隔是无标签跟踪完全丢失时的第二层保护；它不能替代真正的物理球 ID。

2026-08-14 真机日志表明，原来的 0.50 s 与 publisher 的 0.25 s 轨迹结束等待叠加后，会过滤正常的快速第二球。因此生产配置缩短为 0.20 s；只差该间隔的新轨迹改为延迟准入，`RECOVERY` 缓存下一球结果，同时保留同 ID 单拍和硬故障保护。

## Planner 结果传递与失败分类

输入侧继续使用 latest-only pending snapshot，避免 planner 落后时积压旧球位置。输出侧不再只保存一个 `latest_result`，而是使用容量 64 的有序 completed-result 队列。policy 每个 tick 按 generation 顺序 drain 全部完成结果，避免短暂 success/failure 在下一次 policy tick 前互相覆盖。

队列溢出表示 policy 已无法完整观察安全事件：记录 `RESULT_QUEUE_OVERFLOW`，在承诺窗口前立即撤拍并消费当前 ID。队列不会静默丢结果。

planner 不再依赖异常字符串决定控制语义，而是给 failure 标注明确原因：

- `TRACK_ENDED`
- `ESTIMATOR_NOT_READY`
- `BASE_POSE_INVALID`
- `BALL_NOT_INCOMING`
- `NO_FUTURE_CROSSING`
- `HIT_HEIGHT_OUT_OF_RANGE`
- `NONFINITE_INPUT_OR_OUTPUT`
- `INTERNAL_ERROR`

异常文本继续保留用于诊断，但 lifecycle 只使用枚举原因。

出球目标速度必须用与来球预测相同的重力、二次空气阻力系数和积分步长反解，使球从 `x=0` 击球面在 `post_hit_flight_time=0.48 s` 后到达配置落点。无阻力解析式只作为 shooting 初值，不能直接作为最终出球速度。

## 单拍命令生命周期

### 手型判定坐标

自动手型只使用预测击球点 `strike_plan.p_racket_target` 在球桌/world frame 中的绝对 y，不使用“击球点减当前 base”得到的相对 lateral y，也不受机器人当前 y 或 yaw 影响：

```text
table_y < 0  -> forehand
table_y >= 0 -> backhand
```

零点归入反手，保证边界是确定的。`BaseTargetPlanner` 必须接收已经算好的显式 `strike_type`，不能在内部根据相对 base 坐标重新选择。RobotBridge4 真机单拍模式下该规则是权威规则；`force_strike_type` 只允许 MuJoCo 和离线测试使用，真机配置为非空时必须在进入 policy 前报错。

### WAITING 与 TRACKING

- 未接受 active command 时继续使用 WAITING observation。
- 普通 planner failure 只更新诊断，不产生动作命令。
- 过晚结果仍按现有 `minimum_arm_time_to_strike_s = 0.30` 跳过，但同时消费该 ID，后续同 ID 不得重新 arm。

### 第一次 ARMED

第一次成功进入 `ARMED` 时锁存：

- `track_id`；
- `strike_type`；
- `p_base_target_xy`；
- 首个有效 strike deadline。

因此同一轨迹不会在正手和反手之间切换，base target 也不会因后续预测点左右变化而跳动。

在 `time_to_strike > 0.30 s` 时，只允许同一 `track_id`、同一 `strike_type` 的新结果细化 racket position 和 racket velocity。每次更新相对当前已接受值必须同时满足：

- racket position 三维欧氏变化不超过 `0.05 m`；
- racket velocity 三维欧氏变化不超过 `0.75 m/s`；
- 新结果仍指向已锁存的 strike deadline，允许的数值误差不超过 `0.05 s`。

超出任一边界的结果标记为 `OVERRIDE_DISCONTINUITY`，只丢弃该次更新并继续保持 last-good active command。

### 撤拍

当 `time_to_strike > 0.30 s` 时，以下原因立即撤拍：

- `TRACK_ENDED`
- `BASE_POSE_INVALID`
- `NONFINITE_INPUT_OR_OUTPUT`
- `INTERNAL_ERROR`
- `VICON_SCHEMA_ERROR`
- v2 stream stale
- track ID 冲突
- completed-result queue overflow

以下原因只丢弃本次新规划，继续保持 last-good active command，不递增撤拍计数：

- `ESTIMATOR_NOT_READY`
- `HIT_HEIGHT_OUT_OF_RANGE`
- `OVERRIDE_DISCONTINUITY`

以下原因对同一 ID 连续出现 3 个 completed result 时撤拍；任一通过连续性检查的成功结果会把计数清零：

- `BALL_NOT_INCOMING`
- `NO_FUTURE_CROSSING`

撤拍是一个原子 lifecycle 操作：清除 active/cached result、消费 ID、进入 `WAITING`、锁存当前真机 base 世界坐标，并记录原因。撤拍不会递增或伪造 track ID。

### 承诺窗口、击球与恢复

当 `time_to_strike <= 0.30 s` 时进入承诺窗口：冻结整条 active command，忽略该 ID 的所有普通 planner 更新和 failure，直到 strike/recovery 完成。这个边界避免在动作已经进入最后阶段时突然切回另一套 policy 目标。遥控器终止和现有底层安全机制不受该冻结规则限制。

跨过 strike deadline 时立即把 ID 标记为 consumed，然后进入 `RECOVERY`。同一 ID 即使继续可见、反向或再次满足 incoming 条件，也不能重新进入 `TRACKING/ARMED`。恢复期间允许首个获得成功规划的新 ID 缓存最新结果；恢复结束时先锁存当时的 base 位置，再按当前时间重新执行正常的 late/arm 判定。

## WAITING 当前地点语义

该变化只作用于 `sim=real_world`；MuJoCo 保持已有发球和 waiting 配置。

进入 WAITING 的边沿包括：

- 首次 policy reset 完成；
- 未 arm 的轨迹结束；
- 过晚跳过；
- 安全撤拍；
- RECOVERY 完成。

在边沿上读取一次有效 `G2Pelvis` 世界位置，把当前 `x/y` 锁存为 `waiting_base_anchor_xy_w`。WAITING 的每一帧都用“固定 anchor 减当前 pelvis 位置”构造 body-frame base target；不能每帧重写 anchor，否则机器人在 y 方向漂移时 target error 会永远接近零。

WAITING 的其余语义保持现状：

- racket target 由当前 FK 计算；
- racket target velocity 为零；
- time-to-strike 使用 `waiting_time_to_strike_s = 0.92`；
- observation 仍严格为 104 维；
- ONNX 与 `apply_action()`/PD 发布持续执行；
- 不冻结 `q_des`，不增加隐藏 action window。

如果进入 WAITING 时 pelvis pose 无效，不得使用零坐标或旧固定桌边坐标替代。已有有效 waiting anchor 时保持原值，同时锁存 `BASE_POSE_INVALID`，禁止下一拍 arm；首次进入时若还没有有效 anchor，则 policy 入口直接失败。pose 恢复并重新进入 policy 后才能接下一发。

## Freshness、频率与日志

- `planner_update_rate_hz` 为 100 Hz，用于尽早形成稳定来球的首个可用规划；它是提交上限，不改变 policy 的 20 ms 周期。
- `RealWorld` 删除每个 ball LCM 包的同步 `print`。
- ball、planner、queue 和 lifecycle 统计最多每秒汇总一次；状态转换、撤拍和协议错误仍立即记录一次。
- 任意 v2 消息连续 `0.40 s` 未到达时，把 base pose 置为 invalid、结束当前球、在承诺窗口前撤拍，并锁存 `VICON_STREAM_STALE`。数据恢复不会自动解除该锁存，必须重新进入 policy。
- active ball 在 `0.40 s` 内既没有同 ID valid 包也没有同 ID invalid 包时，按 track stale 处理；该阈值大于 publisher 的 0.25 s end timeout，为发布端正常结束消息留出余量。

每条关键诊断至少包含：`track_id`、generation、source frame、phase、decision、failure reason、consumed、locked strike type、policy TTS 和 queue depth。
自动手型结果还必须记录 `strike_table_y_w` 和固定值 `strike_side_source=table_y`，避免日志把相对 base lateral y 误当成判定依据。

## 组件边界

### LCM 与发布端

- `unitree_sdk2/lcm_types/transformation_t.lcm`
- 重新生成的 `transformation_t.hpp`、`transformation_t.py`
- `deploy/mocap_bridge/vicon_table_lcm_bridge.cpp`
- `deploy/mocap_bridge/chingmu_table_lcm_bridge.py`
- `deploy/mocap_bridge/monitor_vicon_lcm.py`

发布端只负责候选关联、ID 生命周期和 wire 消息，不知道 planner phase，也不接收击球完成反馈。

### 接收、规划与 lifecycle

- `deploy/simulator/real_world.py`：v2 解码、freshness、ID/帧校验、estimator 生命周期和 snapshot。
- `deploy/utils/hitter_planner.py`：带稳定 reason 的 typed planner rejection。
- `deploy/utils/hitter_realtime.py`：typed planner result、有序结果队列、单拍 lifecycle、消费/锁定/撤拍。
- `deploy/envs/hitter.py`：planner 输入、结果 drain、WAITING anchor、observation 和日志。
- `deploy/utils/hitter_runtime_factory.py`、`deploy/config/hitter.yaml`、`deploy/config/sim/real_world.yaml` 与 `deploy/config/mimic/hitter.yaml`：校验并注入频道、freshness 和 lifecycle 参数。

active runtime 不使用 `deploy/simulator/real_world_new.py` 或 `deploy/mocap_bridge (copy)`；它们不是 RobotBridge4 真机入口，也不能作为 v2 启动脚本。

### Diagnostics 与历史录制

在线 monitor、recording、CSV/export 增加 `track_id`、consumed 和 cancel reason。新录制只接受 v2 payload。

现有 RobotBridge3 历史录制没有 wire `track_id`。离线 replay loader 可以把 session hash 和旧 `track_epoch` 组合成确定性的正 `int64` 测试 ID，并在输出中标记 `identity_source=legacy_inferred`。该适配器只能存在于 diagnostics/replay 路径，不能被 `RealWorld` 或真机 lifecycle 调用。历史回放可以验证命令状态机，不能作为 v2 wire compatibility 证明。

## 配置默认值

| 配置 | 默认值 | 所属边界 |
|---|---:|---|
| `vicon_lcm_channel` | `vicon_state_data_v2` | publisher / RealWorld |
| `vicon_base_subject` | `G2Pelvis` | publisher / RealWorld |
| `ball_track_association_radius_m` | `0.35` | publisher |
| `ball_track_end_timeout_s` | `0.25` | publisher |
| `vicon_stream_stale_timeout_s` | `0.40` | RealWorld |
| `ball_message_stale_timeout_s` | `0.40` | RealWorld |
| `new_serve_no_ball_s` | `0.20` | RealWorld |
| `vicon_event_queue_capacity` | `64` | RealWorld |
| `planner_update_rate_hz` | `100.0` | planner submit |
| `completed_result_queue_capacity` | `64` | planner worker |
| `armed_cancel_consecutive_failures` | `3` | lifecycle |
| `commit_time_to_strike_s` | `0.30` | lifecycle |
| `maximum_racket_target_override_delta_m` | `0.05` | lifecycle |
| `maximum_racket_velocity_override_delta_mps` | `0.75` | lifecycle |
| `maximum_strike_deadline_override_delta_s` | `0.05` | lifecycle |

所有时间、距离和频率配置必须有限且满足正值/非负值约束；queue capacity 和 failure count 必须是正整数。非法配置在进入真机控制前报错。

## 测试设计

### Schema 和 wire

1. Python v2 encode/decode 保留 `track_id`；C++/Python 生成类型的 fingerprint 一致。
2. pelvis/table 的 ID 为 0，valid/invalid ball 均携带正确正整数 ID。
3. v1 payload 不会进入 v2 controller；频道配置错误在启动检查中被发现。
4. 向 v2 callback 注入坏 fingerprint 会锁存 `VICON_SCHEMA_ERROR`，但共享 poll 线程仍能处理后续遥控器和状态消息。

### 两个 publisher

1. 首帧分配正 ID，连续帧、方向反转、`x <= 0`、bounce 和返回均保持同一 ID。
2. 丢失 `< 0.25 s` 后恢复仍使用原 ID。
3. 丢失达到 `0.25 s` 只发布一条带原 ID 的 invalid；下一条合格轨迹获得不同 ID。
4. active 时关联半径外 marker 不抢占轨迹。
5. C++ 与 Python 对同一候选序列产生相同的轨迹开始/保持/结束决策。

### 接收端和单拍

1. strike 后 estimator reset 不改变 ID；同 ID 返回不产生 planner submission。
2. skipped、cancelled、struck、ended ID 都不能第二次 arm。
3. recovery 期间的新 ID 只缓存最新成功结果，不覆盖上一拍；恢复结束后仍满足 `minimum_arm_time_to_strike_s` 才能 arm。
4. 新 ID 只有在 WAITING/RECOVERY + 0.20 s no-ball + stream/base valid 后才被接纳。
5. 0.20 s 计时未满时出现的新 ID 保持 pending；到 deadline 后重新检查硬条件并从当前帧开始 estimator/planner，期间不得提前提交。
6. 启动时已经可见的球被 quarantine；乱序帧、非正 ID、ID 冲突全部 fail closed。
7. publisher 断流取消 pre-commit command 并锁存故障，恢复数据不会自动 re-arm。

### Lifecycle 和目标稳定性

1. 预测击球点 table y 为负时只产生 forehand，为零或正时只产生 backhand；改变当前 base y/yaw 不改变结果。
2. 真机 `force_strike_type` 非空时在进入 policy 前失败。
3. 第一次 ARMED 后 strike type、base target 和 deadline 保持锁定。
4. 合格的同手型小幅 racket 更新可以在 TTS `> 0.30 s` 被接受。
5. 超界更新不覆盖 active command，也不递增撤拍计数。
6. immediate failure 在一个 policy tick 内撤拍；`BALL_NOT_INCOMING` 或 `NO_FUTURE_CROSSING` 连续出现时在第 3 个有序结果撤拍。
7. TTS `<= 0.30 s` 后所有普通结果都不能改变 active command。
8. completed-result 队列保持顺序；溢出不会静默丢失并触发 fail-closed 路径。
9. 每个 `track_id` 的 strike transition 计数永远不大于 1。

### WAITING 和 policy 合同

1. 真机进入 WAITING 时只捕获一次当前 pelvis `x/y`；随后 pelvis 偏移会产生指向固定 anchor 的非零恢复误差。
2. cancel 和 recovery 完成会重新捕获各自当时位置。
3. pelvis invalid 不会把 anchor 改成零或固定桌边坐标；首次没有有效 anchor 时 policy 入口失败。
4. MuJoCo waiting 和发球时序保持现有行为。
5. WAITING、TRACKING、ARMED、RECOVERY 的 observation 都保持 104 维且为有限值。
6. WAITING 期间 ONNX 推理和 PD 发布调用次数不被抑制。

### 回放与性能

1. 用历史事故 recording 的 replay-only legacy ID 适配重放“先成功、后撞网失败、球返回”序列，证明旧命令在 pre-commit 被撤销且同轨迹不再 arm。
2. 构造 v2 recording 重放同 ID 返回和下一 ID 新发球，证明每 ID 最多一次 strike。
3. planner submit 长时间不超过配置的 100 Hz 容差，completed queue 无正常负载溢出。
4. LCM callback 不含逐 ball 同步 stdout；1 Hz 汇总日志包含完整计数和状态。

## 真机上线顺序与验收

实现完成后必须按以下顺序验收，不能直接进入击球测试：

1. 离线运行 schema、publisher、consumer、lifecycle、observation 和 replay 测试。
2. 编译 C++ bridge，单独运行 v2 monitor，确认 pelvis/table ID 为 0，ball ID 在连续轨迹中稳定且结束消息只出现一次。
3. 停止所有其他 v1/v2 mocap publisher，确认只有一个 RobotBridge4 v2 publisher。
4. 启动 RobotBridge4 `trans` 和显式 `sim=real_world` policy；第二次 R2 前确认 v2 freshness、有效 `G2Pelvis`、正确标定和现有第一帧 5 秒平滑过渡。
5. 无球保持测试：进入 WAITING 后记录锁存 anchor，至少观察 30 s，确认 target 不随 pelvis y 漂移而重写，且 ONNX/PD 持续运行。
6. 不击球轨迹测试：只运行跟踪/monitor，手动让同一球反向、越过近端边界和短暂遮挡，确认 ID 不变。
7. 低风险单拍测试：每次只发一球，确认每个 ID 最多一个 `ARMED -> RECOVERY`；球继续可见或返回时不能再次 ARMED。
8. 撞网/失效测试只有在前述步骤通过后进行；pre-commit failure 必须在规定结果数内撤拍，不能继续执行旧目标。

完成标准：

- 在线 v2 链路没有 `Decode error`，共享 LCM poll 线程持续存活。
- 对所有测试和真机记录，`strike_count(track_id) <= 1`。
- 自动模式严格满足 table y `< 0` 为正手、table y `>= 0` 为反手，且不依赖机器人 base pose。
- 同一 ID 内 strike type、base target、deadline 不发生切换。
- pre-commit 的 immediate failure 最迟一个 20 ms policy tick 生效，连续 failure 在第 3 个有序结果生效。
- WAITING 的 world anchor 在状态期间保持常量，进入新的 WAITING 时才更新为当时位置。
- WAITING 不改变 104-D ONNX 合同，也不抑制 action/PD 发布。
- planner submit 不超过配置的 100 Hz，正常运行没有 completed queue overflow，也没有逐球包 stdout。

## 已知限制

Vicon/ChingMu 提供的是无标签点，不是带永久身份的球。若同一物理球完全丢失超过 0.25 s，publisher 必须结束旧轨迹；它再次出现时只能作为新候选。0.20 s no-ball 准入间隔和单拍消费状态可以降低误接纳概率，但不能在数学上证明它还是不是同一颗实体球。若未来需要跨长遮挡或多球的永久身份，必须由上游提供真正的目标跟踪 ID 或增加可辨识标记，而不能继续扩展本地 epoch 猜测。
