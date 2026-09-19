# MuJoCo 两次发球时序设计

## 目标

在一次 MuJoCo 仿真中只发两次完全相同的球：仿真开始时发第一球，MuJoCo 仿真时间经过 20 秒后发第二球。暂停仿真时，MuJoCo 时间不前进，因此第二次发球也必须暂停等待。

## 行为定义

- 发球时刻固定为相对仿真时间 `0.0 s` 和 `20.0 s`。
- 第二球完整复用第一球的初始球状态，包括位置、姿态、线速度和角速度。
- 第二球发出后不再自动发第三球。
- 击球指令结束、规划失败或球飞出场地时，都不得提前触发额外发球。
- 该行为只作用于 MuJoCo 后端，不改变真机链路。

## 选定方案

发球时序由 `HitterEnv` 管理，物理状态的保存与恢复由 `Mujoco` 提供。这样既能使用 `mujoco_data.time` 作为唯一时间源，也能在发出第二球时同步更新 planner 的轨迹代次。

配置在 `deploy/config/mimic/hitter.yaml` 的 `motion` 节点下增加：

```yaml
mujoco_serve_times_s: [0.0, 20.0]
```

不使用墙钟时间、后台线程或定时器。

## 数据流

第一次进入 MuJoCo planner 更新时：

```text
mujoco_data.time
  -> 到达发球时刻 0.0 s
  -> reset_hitter_ball()
  -> 保存球的完整初始 qpos/qvel
  -> track epoch 加一
  -> planner 处理第一球
```

后续每次 planner 更新都读取 MuJoCo 时间。首次达到 20 秒时：

```text
mujoco_data.time - 第一次发球时间 >= 20.0 s
  -> 恢复保存的初始 qpos/qvel
  -> mujoco.mj_forward()
  -> track epoch 加一、generation 清零
  -> planner 将其作为第二条独立来球轨迹处理
```

第二个发球事件消费后，发球索引到达列表末尾，后续不再重置球。

## 与现有生命周期的关系

当前 `HitterEnv` 会在击球指令结束或 planner 跳过结果后设置 `hitter_ball_sequence_needs_reset`。启用 `mujoco_serve_times_s` 后，这个标志不得直接调用 `reset_hitter_ball()`；实际发球只由计划中的两个时刻触发。

非计划模式继续保留原来的按指令重置行为，避免影响其他配置。真机后端继续使用现有 estimator、planner 和指令生命周期，不读取 MuJoCo 发球计划。

## 纯物理边界

发球时写入球的初始状态属于场景重置，是允许的离散事件。球一旦发出，其飞行、桌面反弹、球网接触和球拍击球仍全部由 MuJoCo 接触动力学计算；不得加入任何飞行过程中的位置或速度覆写。

## 测试

实现必须按测试先行完成，并至少覆盖：

1. 仿真时间小于 20 秒时只发生第一次发球。
2. 仿真时间首次达到或超过 20 秒时恰好发生第二次发球。
3. 20 秒之后继续运行不会发生第三次发球。
4. 两次发球的球 `qpos` 和 `qvel` 完全一致。
5. 第二次发球会推进 track epoch 并清零 generation。
6. 指令结束和 planner 跳过结果不会在计划时刻之外重置球。
7. 未配置 `mujoco_serve_times_s` 时保持现有行为。
8. 真机路径不受影响。

## 完成标准

- 可视化运行时只能观察到两次发球，间隔为 20 秒 MuJoCo 仿真时间。
- 暂停窗口不会消耗这 20 秒。
- 第二球与第一球的初始状态相同。
- 第二球能够作为新轨迹进入 planner 和 policy 数据链路。
- 聚焦测试和现有纯物理乒乓球测试全部通过。
