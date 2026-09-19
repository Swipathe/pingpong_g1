# RobotBridge4 击球高度上限扩展至 3 米设计

## 目标

仅调整 RobotBridge4 的弹道规划准入上限，将世界坐标中的预测击球高度范围从
`(0.760, 1.450] m` 扩展为 `(0.760, 3.000] m`。RobotBridge3 不做任何修改。

## 方案

RobotBridge4 当前通过以下公式生成绝对击球高度上限：

```text
maximum_hit_height = table_height + maximum_hit_height_above_table_m
```

桌面高度为 `0.760 m`，因此将
`motion.ball_planner.maximum_hit_height_above_table_m` 从 `0.69` 改为 `2.24`：

```text
0.760 + 2.240 = 3.000 m
```

不修改 planner 公式、Track ID 生命周期、正反手选择、速度限制、击球时间门限或其他配置。

## 验证

1. 先增加一个针对当前 RobotBridge4 配置的回归测试，并确认它在旧配置下因上限仍为
   `1.45 m` 而失败。
2. 修改唯一配置项后，确认运行时 factory 解析出的
   `planner.strike_planner.maximum_hit_height` 精确为 `3.0`。
3. 运行 runtime factory 与 planner 边界相关测试，确认 `z=3.0 m` 被接受、`z>3.0 m`
   仍被拒绝，且其他准入条件不变。

## 安全边界

该改动只放宽弹道规划的高度拒绝条件，不证明 `3 m` 的拍面目标处于机器人实际可达空间。
现有 racket 速度范围、时间门限和生命周期取消逻辑保持不变，但它们不能替代机械臂可达性
检查。真机复测时仍需先低速、单球，并观察生成的拍面目标和 policy 动作。
