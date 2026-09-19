# 真机正手目标球拍 X 速度偏置设计

## 目标

只在 RobotBridge4 真机部署中，当最终规划命令的 `strike_type` 为
`forehand` 时，在目标球拍速度送入 HITTER policy 之前，将世界/球桌
坐标系 X 分量增加 `0.25 m/s`：

```text
v_policy_w = v_planner_w.copy()
v_policy_w.x += 0.25 m/s
```

Y、Z 分量保持不变。反手和 MuJoCo 不应用该偏置。

## 已确认语义

- 使用 planner 已经确定的 `command.strike_type`，不重新根据姿态或球桌 Y
  推断正反手。
- X 指当前直接进入 policy 观测的世界/球桌坐标系
  `v_racket_target_w[0]`。
- 偏置在 policy 输入边界应用，不改写 planner 原始命令。
- 每次构造观测都从原始速度的副本开始，因此不会累积成
  `+0.50 m/s`、`+0.75 m/s`。
- 配置项为
  `policy.real_world_forehand_racket_velocity_x_offset_mps: 0.25`；默认值为
  `0.0`，以保持未配置场景的原行为。

## 方案比较

### 方案一：在 policy 观测边界添加偏置（采用）

保留 `HitterWbcCommand.v_racket_target_w` 和生命周期内部数据不变，在
`HitterEnv` 构造 active observation 时生成策略速度副本并应用条件偏置。
这样 planner 连续性判断、撤销逻辑和原始目标日志仍基于真实 planner 输出，
同时可以准确限制为真机 policy 输入。

### 方案二：修改 planner 返回的命令

实现位置集中，但会把补偿值混入 planner 原始结果，影响速度覆盖比较、诊断
和 MuJoCo/离线复现语义，需要在多个入口额外区分真机，因此不采用。

### 方案三：修改碰撞速度计算公式

会改变所有击球类型和所有运行模式，而且偏置会参与后续分量裁剪；不符合
“只在真机正手送入 policy 前补偿”的边界，因此不采用。

## 数据流

1. planner 生成原始 `v_racket_target_w` 和 `strike_type`。
2. 生命周期保留、比较并锁定原始命令。
3. `HitterEnv` 构造 active policy observation。
4. 若 `simulator.is_real` 且 `strike_type == forehand`，复制原始速度并对
   X 加配置偏置；否则复制后原样返回。
5. 调整后的向量进入 104 维观测中的 racket velocity 段（完整观测索引
   `13:16`）。
6. 每次新击球目标只记录一次原始速度、policy 速度和实际应用的 X 偏置，
   便于真机对比。

## 校验与错误处理

- X 偏置必须是有限浮点数；非有限配置在环境初始化时直接拒绝。
- 速度转换函数必须返回独立 `float32 (3,)` 副本，不允许修改 planner
  原数组。
- 测试覆盖真机正手 `+0.25`、真机反手不变、MuJoCo 正手不变，以及连续
  两次构造不累计。
- 运行现有 observation 与 strike-target logging 测试，防止 104 维观测布局
  和日志回归。

## 范围

只修改 RobotBridge4 的 policy 输入适配、对应配置、日志和测试。不修改
planner 碰撞公式、正反手判定、RobotBridge3、ONNX 文件或真机启动方式。
