# RobotBridge4 反弹后命令保留与最低击球速度设计

## 背景

2026-08-13 21:42 真机日志中，共有 8 个球轨迹进入 `ARMED`：5 个执行到击球时刻，3 个在 commit 前撤拍。三次撤拍均发生在桌面反弹清空 31 点拟合窗口之后，并在 54–62 ms 内因第三个 `ESTIMATOR_NOT_READY` 结果触发。当前估计器按 360 Hz 收集 31 点，理论上需要约 83 ms 才能重新 ready，因此现有 50 Hz planner、三次失败阈值必然形成时序竞争。

同一批日志的 5 个击球命令中，3 个球拍目标速度为 1.62–1.66 m/s，另外 2 个只有 0.5148 m/s 和 0.5337 m/s。用当前逆碰撞公式复算可以精确得到这两个低速结果，说明它们是 planner 允许的低速“挡回”解，并非 ONNX 或 PD 层把强命令削弱。当前日志只记录目标速度，尚不能证明实际球拍速度或实际触球结果。

RobotBridge3 在 `ARMED` 后直接忽略失败 planner result，因此反弹后的 estimator 暂时未就绪不会撤销 active command；但它也缺少 RobotBridge4 的 typed hard failure、单球 ID、commit freeze 和连续性安全边界。本设计只引入必要的保留语义，不整体退回 RobotBridge3。

## 目标

- 已有合法 last-good command 的 `ARMED` 轨迹，不再因桌面反弹后的 `ESTIMATOR_NOT_READY` 暂态撤拍。
- 保留 RobotBridge4 对轨迹结束、Vicon/底盘故障、非有限值、内部错误、身份冲突和队列溢出的撤销能力。
- 低速逆碰撞解至少给 policy 一个 1.0 m/s 的主动球拍法向速度目标。
- 保持虚拟击球面 `x=0.0`，不通过移动击球面制造额外挥拍速度。
- 修正 predictor 在离散步长跨越近端桌沿时可能漏掉合法桌面接触的数值边界。
- 增加足够日志，区分原始 planner 速度、速度下限后的目标以及预测击球点。

## 非目标

- 不修改 ONNX 模型、104 维 observation、29 维 action、PD 增益或 Unitree `trans` 协议。
- 不删除通用 `cancel()`，不忽略全部 planner failure。
- 不改变 `BALL_NOT_INCOMING`、`NO_FUTURE_CROSSING` 的累计撤拍语义。
- 不把 planner 目标速度当作实际球拍速度或实际触球证明。
- 不通过伪造 restitution、全局缩短 `post_hit_flight_time` 或恢复 `x=-0.2` 增强动作。

## 方案选择

采用三个相互独立、按顺序验证的变化：

1. `ARMED` 中将 `ESTIMATOR_NOT_READY` 设为 retain-only。
2. 在逆碰撞标量上增加可配置的 1.0 m/s 最低主动法向速度。
3. predictor 使用步长内接触点判断桌面反弹，而不是只判断步长终点。

没有采用以下替代方案：

- 将连续失败阈值从 3 提高到 6：这仍依赖调度时序，同时会延迟真正的 `BALL_NOT_INCOMING` 和 `NO_FUTURE_CROSSING` 撤拍。
- 将 estimator 窗口从 31 降低：会直接增加速度估计噪声，且当前真机记录没有原始逐帧轨迹可证明精度。
- 只把世界 x 分量下限改成 1.0 m/s：虽然简单，但会改变原碰撞法向。最低法向速度能保持原始击球方向，组件安全范围仍在最后拥有最高优先级。
- 只缩短 `post_hit_flight_time`：它会同时增强所有正常击球并改变所有落点，影响范围过大。

## 设计一：反弹后保留 last-good command

`HitterCommandLifecycle` 将 `ESTIMATOR_NOT_READY` 从 `_SOFT_FAILURES` 移入 `_RETAIN_ONLY_FAILURES`。该规则只影响已经进入 `ARMED`、已经拥有合法 active result 的轨迹：

- 任意次数 `ESTIMATOR_NOT_READY` 返回 `retained_failure`；
- 不增加、不清零 `consecutive_failure_count`；
- 不替换 active command；
- 不改变锁存的 track ID、手型、base target 或 strike deadline；
- 不消费 track ID，也不进入 `WAITING`。

WAITING/TRACKING 阶段仍不能用未就绪 estimator 生成命令。`BALL_NOT_INCOMING` 和 `NO_FUTURE_CROSSING` 继续共享三次累计失败撤拍。以下故障继续沿现有路径在 pre-commit 立即撤拍：

- `TRACK_ENDED`
- `BASE_POSE_INVALID`
- `NONFINITE_INPUT_OR_OUTPUT`
- `INTERNAL_ERROR`
- Vicon schema/stream/ball stale
- track ID 冲突
- completed-result queue overflow

现有 commit freeze、遥控停止和底层安全机制均不改变。该设计可以确定消除本次日志中三次以 `ESTIMATOR_NOT_READY` 为唯一 cancel reason 的撤拍，但不能单独保证实际触球。

## 设计二：最低主动法向速度

`StrikePlanner` 新增非负有限参数：

```text
minimum_racket_normal_speed_mps: 1.0
```

逆碰撞计算仍先得到：

```text
n = (v_ball_out - v_ball_in) / ||v_ball_out - v_ball_in||
s = (v_ball_out·n + restitution * v_ball_in·n) / (1 + restitution)
```

随后使用：

```text
s_command = max(s, minimum_racket_normal_speed_mps)
v_racket = s_command * n
```

最后继续执行现有分量范围裁剪。分量范围是最终安全边界，优先级高于最低法向速度；因此通用合同是“裁剪前法向标量至少 1.0 m/s”，不是在所有异常方向上强行保证最终向量模长。

对本次两条弱命令，1.0 m/s floor 预计分别生成约 `[0.749, 0.009, 0.663]` 和 `[0.875, -0.005, 0.484]` m/s，均在 model17500 的训练范围 `x/z=[0,6]`、`y=[-0.5,0.5]` 内。按当前 `drag_coefficient=0.098847` 与 5 ms predictor 复算，两条轨迹首次下降到 `z=0.78` 时的 `x` 约为 2.47 m 和 2.40 m，在当前模型内仍位于桌内；若忽略 drag，第一条约为 `x=2.96 m`，已经越过远端桌沿。这说明落点结论对模型敏感，必须通过真机单球落点记录验收。

已有原始法向速度大于等于 1.0 m/s 的命令逐位保持不变。配置解析必须拒绝布尔值、负数、NaN 和无穷值。

## 设计三：近端桌沿连续接触

当前 predictor 在每个 5 ms 步长后检查 `next_pos.z <= contact_z`，并使用 `next_pos.xy` 判断是否仍在桌面内。高速球可能在步长中途于桌内接触桌面，但步长终点已经越过 `x=0`，从而漏掉合法反弹。

当一个下降步长从接触高度上方跨到接触高度下方时，predictor 必须：

1. 在该步长内求接触比例和接触时刻；
2. 在接触时刻插值得到 `contact_xy`；
3. 只在 `contact_xy` 位于桌面矩形内时施加水平/垂直 restitution；
4. 使用反弹后速度积分该步长的剩余时间；
5. 若接触点已在桌外，则保持自由飞行，禁止用桌沿外的终点伪造反弹。

该变化是数值鲁棒性修复，不作为两条弱命令的唯一根因解释；第二条弱命令的预测击球高度约 1.419 m，不属于低桌沿接触。

## 日志与诊断

击球时的 INFO 日志补充：

- `p_racket_target_w_m`
- `raw_racket_normal_speed_mps`
- `commanded_racket_normal_speed_mps`
- `minimum_racket_normal_speed_mps`
- 是否应用最低速度 `racket_speed_floor_applied`

继续明确该日志是目标命令，不是实际 FK/Vicon 球拍速度。实际球拍速度和接触结果属于后续真机诊断，不作为本次纯 planner/lifecycle 修改的完成声明。

## 测试设计

### 生命周期

- pre-commit 连续 5 次 `ESTIMATOR_NOT_READY`：始终 `ARMED`，active command 和全部锁存字段不变，计数不增加，ID 未消费。
- 紧接一个 `BASE_POSE_INVALID` 或 `NONFINITE_INPUT_OR_OUTPUT`：立即取消、进入 `WAITING`、清 active、消费 ID。
- 三次 `BALL_NOT_INCOMING`：仍按现有合同取消。
- commit 内既有冻结行为不变。
- 修正现有 integration test 中仍把第三个 `OVERRIDE_DISCONTINUITY` 断言为取消的过期预期。

### 最低速度

- 用本次两条日志数值复算，原始速度分别约 0.5148 和 0.5337 m/s，启用 floor 后裁剪前法向速度精确为 1.0 m/s。
- 原始法向速度大于 1.0 m/s 的计划不变化。
- floor 后三个分量仍满足生产配置范围。
- `minimum_racket_normal_speed_mps=0` 精确恢复原公式，便于回归和回退。
- 非法配置 fail fast。

### 桌沿反弹

- 构造一个步长起点在桌内、终点在桌外、但步长内接触点仍在桌内的轨迹；必须发生一次反弹。
- 构造接触点已在桌外的镜像轨迹；不得反弹。
- 非桌沿普通反弹与自由飞行结果保持现有误差容限。
- 不允许重复反弹或反弹后数值非有限。

## 验收顺序

1. 先运行生命周期聚焦测试，确认只改变 `ESTIMATOR_NOT_READY`。
2. 再运行 planner 公式与桌沿碰撞聚焦测试，单独证明两个 planner 变化。
3. 运行 RobotBridge4 相关 lifecycle、runtime factory、planner、real-world snapshot 集成测试。
4. 真机验收前确认 resolved Hydra 配置仍为 `virtual_hit_plane_x=0.0`、minimum normal speed 为 1.0 m/s。
5. 真机从低速单球开始，要求同一 track ID 在 bounce 后不因 `ESTIMATOR_NOT_READY` 进入 WAITING，每球最多一次 `ARMED -> RECOVERY`；同时记录目标速度和实际落点。

## 风险与回退

- retain-only 会在 estimator 长时间不恢复时继续执行较早命令；该风险由有界 strike deadline 和仍然生效的 hard failure 缓解。
- 最低速度会牺牲原逆碰撞模型的精确落点，可能使回球更深；1.0 m/s 是处于训练域内的保守起点，不在本设计中继续提高。
- 连续接触会改变少量桌沿边界轨迹；必须由内外两个对称测试固定边界，不能通过扩大桌面尺寸制造反弹。
- 软件测试只能证明命令语义，不能证明真机实际球拍速度或触球。

配置回退分别为：将 `minimum_racket_normal_speed_mps` 设为 `0.0` 恢复原速度公式；生命周期回退必须恢复 `ESTIMATOR_NOT_READY` 的原分类，不能删除 hard failure；击球面始终保持 `x=0.0`。
