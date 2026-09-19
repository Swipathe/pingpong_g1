# RobotBridge4 反手边缘落点偏置设计

## 目标

解决机器人侧左边缘（桌面世界系 `+Y`）反手回球容易继续向左出界的问题。生产配置不再在 planner 完成后把球拍目标速度 `Y` 固定减去 `0.3 m/s`，而是在 planner 内把边缘反手的期望落点向 `-Y` 平滑移动，并用新的落点完整重算期望出球速度、碰撞法向和三维球拍目标速度。

## 行为

基础落点继续使用 `desired_landing_point_w: [2.05, 0.0, 0.78]`。只对 `strike_type=backhand` 应用下列规则：

- `strike_y <= 0.30 m`：落点不变；
- `0.30 m < strike_y < 0.50 m`：用 `smoothstep(u)=3u^2-2u^3` 从 `0` 平滑过渡到最大偏置；
- `strike_y >= 0.50 m`：落点 `Y` 最大减少 `0.10 m`，即当前基础配置下为 `-0.10 m`；
- 正手始终不应用该落点偏置。

配置字段放在 `motion.ball_planner`：

```yaml
backhand_edge_landing_start_y_w_m: 0.30
backhand_edge_landing_full_y_w_m: 0.50
backhand_edge_landing_y_decrement_m: 0.10
```

构造器默认 `backhand_edge_landing_y_decrement_m=0.0`，保持未显式配置的调用方行为不变。启用时要求三个数有限且非负、`full > start`、`full` 不超过桌面半宽、偏置后的落点仍在桌面横向边界内。

## 数据流

planner 顺序固定为：

```text
预测 x=0 击球点
→ 解析正手/反手
→ 计算该击球点的有效落点
→ shooting 求期望出球速度
→ 碰撞模型求法向与球拍速度
→ 分量限幅
→ 生成 StrikePlan/HitterWbcCommand
```

不得在 `StrikePlan` 生成后只修改 `v_ball_out` 或 policy observation。显式强制击球类型存在时，`HitterSystemPlanner` 必须把它传给 `StrikePlanner`，避免强制正手却使用反手落点。

## 旧偏置与回退

保留 `real_world_backhand_racket_velocity_y_decrement_mps` 的代码入口，生产 YAML 将其设为 `0.0`，因此不会与 planner 落点偏置叠加。快速回退方法是把 `backhand_edge_landing_y_decrement_m` 改为 `0.0`；如需恢复旧实验，再单独恢复 policy 侧速度减量，但两者不得同时启用。

## 验证

自动测试覆盖：未启用时兼容旧行为、正手不变、阈值以下不变、过渡中点为 `-0.05 m`、`0.50 m` 及以上饱和为 `-0.10 m`、完整 `plan()` 的出球速度确实落到有效目标、非法配置被拒绝、工厂从生产 YAML 正确传参、真机 policy 侧反手速度减量为零。

真机日志仍应以 `v_ball_out_w_mps` 和 `v_racket_target_w_mps` 判断 planner 目标；这些字段不是实测拍速或实测出球速度。
