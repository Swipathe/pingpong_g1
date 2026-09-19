# Chingmu 球追踪远端边界收紧设计

## 目标

只收紧 Chingmu 桥接层对未标记球候选点的有效接纳/发布 X 方向范围：

```text
修改前有效接纳/发布范围：0 < x <= table_length_m
修改后有效接纳/发布范围：0 < x <= table_length_m - 0.40 m
```

这表示球从桌子远端进入桌面投影后的前 40 cm 仍视为“未进入有效接纳/发布
区域”，进入至少 40 cm 后才允许发布 `ball`。

上述范围约束描述新轨迹 admission 和 `valid ball` 发布，不要求从活动轨迹的
内部结束证据中删除 `x <= 0` 点。活动轨迹仍可观察并关联 `x <= 0` 点，用它
判断球已越过机器人侧边界；一旦选中该结束证据，tracker 会清空轨迹并只发布
一次 invalid ball，绝不会把该点作为 valid ball 发布。

## 实现范围

- 仅修改 `deploy/mocap_bridge/chingmu_table_lcm_bridge.py` 中 `BallTracker.update()` 的候选点 X 上界。
- 使用命名常量表示固定的 `0.40 m` 远端留白。
- 保持新轨迹 admission 和 valid ball 发布的 X 下界 `x > 0` 不变。
- 保持活动轨迹使用 `x <= 0` 内部点作为结束证据的既有逻辑不变。
- 保持 `|y| <= table_width_m / 2` 不变。
- 保持 `z > table_height_m` 不变。
- 不修改 `table_length_m` 的真实值。
- 不修改桌面标定文件、桌面坐标系、发布的桌面中心或 planner 的桌面参数。
- 不增加新的命令行参数或配置项。

## 数据链路

```text
Chingmu 未标记点
  -> 转换到桌面世界坐标系
  -> 按新的 X 上界、原 Y/Z 边界过滤
  -> BallTracker 关联（活动轨迹可用 x <= 0 点作为结束证据）
  -> 仅 x > 0 时发布有效 ball
```

超过新 X 上界的点不会进入 `BallTracker` 候选集合。若尚未建立轨迹，则不会发布球；若已有轨迹且所有候选都被该边界过滤，则沿用现有逻辑结束该轨迹并发布一次无效球状态。

## 测试

增加最小边界回归测试：

- `x = table_length_m - 0.40 m`：允许追踪。
- `x > table_length_m - 0.40 m`：拒绝追踪。
- 以非默认 `table_length_m` 和紧邻上界的稳定 epsilon 验证 cutoff 在运行时
  派生且为包含边界。
- 验证既有 admission/publish X 下界、Y 包含边界和 Z 严格下界不因本次修改
  而改变。
- 验证活动轨迹只有超新上界候选时只结束一次，以及 `x <= 0` 结束证据从不
  作为 valid ball 发布。

远端上界行为测试先在旧实现上失败，再修改生产代码使其通过；既有 X/Y/Z
边界和 active-track 结束语义测试用于锁定未变行为，不人为制造 RED。
