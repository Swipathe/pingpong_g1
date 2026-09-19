# 真机反手目标球拍 Y 速度减量设计

## 目标

在 RobotBridge4 真机部署中，当最终规划命令的 `strike_type` 为
`backhand` 时，在目标球拍速度送入 HITTER policy 之前，将世界/球桌
坐标系 Y 分量减去 `0.3 m/s`：

```text
v_policy_w = v_planner_w.copy()
v_policy_w.y -= 0.3 m/s
```

X、Z 分量保持不变。现有真机正手 X `+0.25 m/s` 规则继续保留；MuJoCo
不应用任何真机速度补偿。

## 语义与边界

- 使用 planner 已确定的 `command.strike_type`，不重新推断正反手。
- Y 指直接进入 policy 观测的世界/球桌坐标系
  `v_racket_target_w[1]`。
- 补偿仅在 policy 输入边界应用，不修改 planner 命令、生命周期缓存或
  连续性判断所使用的原始速度。
- 每次构造观测都从 planner 原始速度的独立副本开始，不允许连续帧累减。
- 配置项为
  `policy.real_world_backhand_racket_velocity_y_decrement_mps: 0.3`；默认值
  为 `0.0`。

## 方案

采用现有 `_policy_racket_target_velocity_w()` 适配边界：真机正手对 X
应用正偏置，真机反手对 Y 应用负偏置，其余情况返回原始速度副本。相比
修改 planner 输出或在 observation 拼接处硬编码，这一方案能保留原始规划
语义，并让配置、数值校验、日志和非累积行为集中在同一处。

## 数据流与日志

1. planner 生成原始 `v_racket_target_w` 和最终 `strike_type`。
2. 生命周期保存原始命令。
3. 构造 active policy observation 时复制速度向量。
4. 真机正手执行 X `+0.25`；真机反手执行 Y `-0.3`；MuJoCo 不调整。
5. 调整后的向量进入 104 维观测的 racket velocity 段 `13:16`。
6. 击球目标日志保留 `v_racket_target_w_mps` 原值，并输出
   `v_racket_policy_w_mps`、`policy_vx_offset_mps` 和
   `policy_vy_offset_mps`；反手实际 Y 偏置记录为 `-0.3000`。

## 校验与测试

- 减量配置必须是有限、非负且可由 `float32` 表示的实数。
- 应用减量后若 Y 分量不能由 `float32` 表示，则拒绝生成 policy 输入。
- 测试覆盖真机反手 Y `-0.3`、真机正手仅 X `+0.25`、MuJoCo 不变、
  连续观测不累减、planner 原命令不变，以及原始/调整后日志字段。

## 范围

只修改 RobotBridge4 的 policy 输入适配、对应配置、日志和测试。不修改
planner 物理公式、正反手判定、RobotBridge3、ONNX 文件或真机启动方式。
