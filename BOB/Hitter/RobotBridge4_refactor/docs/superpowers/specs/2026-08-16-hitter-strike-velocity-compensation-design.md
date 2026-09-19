# HITTER 真机正反手速度补偿调整设计

## 目标

仅在 RobotBridge4 中调整当前 HITTER 生产配置：

- 真机正手向 policy 提供的世界系目标球拍速度改为 `vx + 0.20 m/s`。
- 真机反手向 policy 提供的世界系目标球拍速度改为 `vy - 0.30 m/s`。
- 当前共享配置不再启用“反手击球点世界 `y > 0.20 m` 时，把目标落点世界 `y` 改为 `-0.30 m`”的规则。

RobotBridge2、RobotBridge3、model10595、Vicon 标定、正反手站位参数和 planner 的通用能力均不在修改范围内。

## 方案选择

采用最小配置方案：修改 `deploy/config/mimic/hitter.yaml` 的两项真机 policy 速度补偿，并删除共享 `ball_planner` 中两项反手边缘落点覆盖配置。保留 `StrikePlanner` 对可选边缘落点规则的通用实现；配置项缺失时，factory 的 `backhand_edge_landing_target_y_w_m=None` 默认值会禁用该规则。

该方案的影响边界为：

- `+0.20/-0.30 m/s` 只在 `simulator.is_real` 分支生效，MuJoCo 不施加这两项 policy 速度补偿。
- 反手边缘落点覆盖位于真机与 MuJoCo 共用配置中；删除后两种运行模式都使用通用目标落点 `[2.05, 0.0, 0.78] m`，不再按反手击球点 `y` 改写目标落点。
- 自动分侧仍保持世界系击球点 `y < 0` 为正手、`y >= 0` 为反手。
- `forehand_nominal_racket_y_b=-0.50 m` 和 `backhand_nominal_racket_y_b=+0.22 m` 保持不变，它们只参与 base target 计算。

## 数据流

planner 继续生成并锁存原始世界系目标球拍速度 `v_planner_w`。构造真机 ONNX observation 时按击球类型生成：

```text
forehand: v_policy_w = v_planner_w + [0.20, 0.00, 0.00]
backhand: v_policy_w = v_planner_w + [0.00, -0.30, 0.00]
```

补偿不回写 planner command，不逐 tick 累加，也不直接修改关节命令。日志继续同时记录 raw planner velocity、policy velocity 和实际 offset。

## 测试与验收

1. 先修改配置合同测试，使其期望正手 `0.20`、反手 `0.30`，以及生产 planner 未配置 direct backhand landing target；在生产配置未改时确认测试按预期失败。
2. 最小修改生产 YAML 后，重新运行配置合同和正反手 policy velocity 专项测试。
3. 运行反手落点测试，确认通用 feature 仍可由显式参数启用，同时生产配置的反手边缘击球仍保留通用落点 `y=0.0`。
4. 用 Hydra `sim=real_world device=cpu --cfg job --resolve` 验证最终真机值。
5. 检查 Git diff，保证只包含本需求对应的 YAML 行和测试行，不覆盖现有未提交改动。

当前正在运行的 policy 不会热加载 YAML；新配置只有在用户按真机安全流程重启 policy 后才生效。本任务只修改和验证代码，不自动停止或重启真机进程。
