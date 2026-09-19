# HITTER 稳定来球确认设计

## 目标

消除 Vicon 球位置在 300 Hz 下的亚毫米抖动对 real-world HITTER 的两个影响：

1. raw 相邻帧 `Vx` 符号翻转错误推进 `ball_track_epoch`，反复清空 31 帧估计窗口；
2. 单个拟合结果只要 `Vx < 0` 就可能被当作来球，缺少稳定性确认。

修改不得引入 action gating，不得改变 waiting observation，不得修改无关文件。

## 实测依据

`recordings/hitter_ball_20260715-143511_xyz_vx.csv` 中，真实来球在可规划阶段的
31 帧拟合 `Vx` 约为 `-2.15` 至 `-3.19 m/s`；静止阶段拟合速度接近零。最后一次
raw 误 reset 后约 0.10 秒 estimator 重新 ready。

按 100 Hz planner snapshot 重放，前三个满足阈值的结果为：

| 确认 | 录制时间 | 拟合 Vx | 预测 TTS |
|---|---:|---:|---:|
| 1 | 0.271 s | -2.153 m/s | 2.428 s |
| 2 | 0.281 s | -2.635 m/s | 1.633 s |
| 3 | 0.291 s | -3.129 m/s | 1.218 s |

第三次确认仍早于 `0.90 s` arm 边界约 0.318 秒，不会让该实测球错过击球。

## 考虑过的方案

### 方案 A：只增加速度阈值

把来球条件改为拟合 `Vx <= -0.20 m/s`。实现简单，但单个异常拟合仍可接纳来球。

### 方案 B：拟合速度阈值加连续确认（采用）

要求 31 帧拟合 `Vx <= -0.20 m/s`，并连续 3 个 100 Hz planner snapshot 成立。
确认只用于接纳一条球轨迹；一旦确认，在该 epoch 内锁定。新 epoch 会清除确认状态。
额外延迟约 20 ms，同时能拒绝零速附近的短暂负向波动。

### 方案 C：保留 raw 方向 epoch，增加 raw 滞回

对 raw 两帧速度增加阈值和连续帧计数。它仍把高噪声两帧差分用于轨迹生命周期，
并与 recovery/deadline 已有的显式 reset 重复，因此不采用。

## 设计

### epoch 生命周期

删除 `RealWorld` 中 raw 相邻帧方向检测及其状态：

- 不再因 `outgoing -> incoming` raw 符号翻转推进 epoch；
- 不再因此清空 estimator；
- 有效球帧直接进入现有 31 帧 `BallStateEstimator`。

保留现有明确边界：

- strike deadline 进入 recovery 时，real-world 显式 reset estimator 并推进 epoch；
- lifecycle/env reset 时推进 epoch；
- invalid 或 occluded 球消息仍结束当前 track，并按现有语义推进 epoch。

### 稳定来球确认

新增一个与 action lifecycle 分离的纯状态组件 `IncomingTrackConfirmation`，由 real-world
planner worker 串行使用。配置为：

```yaml
minimum_stable_incoming_speed_x_mps: 0.20
stable_incoming_confirmation_snapshots: 3
```

每个可见、estimator-ready 的 planner snapshot 使用已经由最近 31 个 Vicon 帧拟合的
`velocity_w[0]`：

1. epoch 改变时，计数清零且取消锁定；
2. 未锁定时，`Vx <= -0.20 m/s` 令计数加一；
3. 任一 snapshot 不满足阈值，计数清零；
4. 计数达到 3 时，当前 epoch 锁定为稳定来球；
5. 锁定后不因单个速度波动取消确认，后续仍由现有未来 `x=0` 定向穿越、击球高度和
   预测 horizon 检查决定是否产生 command；
6. invalid/track-ended 或新 epoch 清除确认。

前三个 snapshot 未确认时，planner result 为普通失败结果，不产生 command 或 deadline。
这不会改变 policy action；WAITING/TRACKING 阶段仍使用当前 waiting observation。

### deadline 与迟到处理

稳定确认完成后，继续使用现有逻辑：

- TTS `> 0.90 s`：进入或保持 TRACKING；
- TTS 在 `[0.80, 0.90] s`：ARMED；
- 首次可用结果 TTS `< 0.80 s`：该 epoch 标记为 skipped。

不为迟到球绕过稳定确认，也不缩短现有最小 arm 时间。

## 代码范围

计划只修改以下相关文件：

- `deploy/simulator/real_world.py`
- `deploy/utils/hitter_realtime.py`
- `deploy/envs/hitter.py`
- `deploy/config/mimic/hitter.yaml`
- 对应的 real-world snapshot、realtime 和 env lifecycle 测试

不修改 policy action、observation schema、waiting 默认输入、ONNX 模型或 mocap bridge。

## 测试与验收

测试先行，必须先看到新测试按预期失败，再实现最小修改：

1. raw X 方向从正变负不推进 epoch，也不清空 estimator 样本；
2. 两个连续拟合 `Vx <= -0.20 m/s` 不确认，第三个才确认；
3. 中间一个不满足阈值会清零计数；
4. epoch 改变会清除计数和锁定状态；
5. 当前 epoch 确认后，单个速度波动不取消锁定；
6. 确认前不生成 command/deadline，确认后沿用现有轨迹边界检查；
7. 实测 CSV 重放在 TTS 大于 0.90 秒时完成确认，并能进入 arm 区间；
8. 现有 91 项定向回归测试全部通过；
9. `git diff --check` 无输出。

现场验收时，静止球不应造成 epoch 快速增长；真实来球应出现
`waiting -> tracking -> armed`，并且 `submitted` 约每秒增加 100。
