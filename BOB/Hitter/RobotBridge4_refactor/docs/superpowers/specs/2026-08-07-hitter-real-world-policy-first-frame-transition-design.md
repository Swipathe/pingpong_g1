# HITTER 真机 Policy 第一帧平滑过渡设计

## 背景

RobotBridge3 当前真机流程在第二次按下 R2 后立即退出 `RealWorld.calibrate()`，随后 HITTER policy 的第一帧关节目标被直接下发。当前模型在理想默认站姿下也可能产生接近 `0.5 rad` 的单关节首帧目标跳变，因此机器人会剧烈地冲向 policy 站姿。

本设计只解决“第二次 R2 后，从当前默认姿态进入 policy 第一帧”的一次性过渡。正常 policy 循环、击球阶段、MuJoCo 行为和底层 `trans` 均保持不变。

## 目标

- 仅在 `sim=real_world` 时启用。
- 第二次 R2 放行后，照常执行一次 ONNX 推理并得到 policy 第一帧目标。
- 冻结该第一帧目标，用 `2.0 s` 将机器人从当前实测关节姿态平滑移动到该目标。
- 过渡完成后恢复现有逐帧 policy 推理和控制逻辑，不增加长期平滑、限速或 action gating。
- 每次 reset 后重新进入 policy 时，只执行一次同样的过渡。
- 所有修改均限制在 RobotBridge3 内。

## 非目标

- 不修改第二次 R2 前的 Vicon `G2Pelvis` 有效性检查。
- 不修改第一次 R2 的默认姿态校准。
- 不修改全局 `action_beta`、模型 action scale、PD 增益或正常击球动作。
- 不在 `RealWorld.apply_action()` 或 `trans` 中增加全局关节限速。
- 不改变 MuJoCo 的 reset、serve 或 policy 时序。
- 不修改 RobotBridge2、MOSAIC-main 或其他仓库。

## 行为时序

```text
第一次 R2 后的 HITTER 默认姿态
    -> 第二次 R2 且 G2Pelvis 有效
    -> HitterAgent 执行 policy 第一帧推理
    -> HitterEnv 冻结第一帧绝对关节目标
    -> 从最新实测关节角平滑插值 2.0 s
    -> 到达第一帧目标
    -> 后续 policy 帧按现有逻辑直接执行
```

过渡期间不请求第二帧 ONNX 输出。LCM 接收线程和真机状态更新仍继续运行；过渡结束后，下一次正常迭代会重新刷新观测并执行第二次 ONNX 推理。

## 组件设计

### 配置

在 `deploy/config/mimic/hitter.yaml` 的 `policy` 段增加：

```yaml
real_world_first_frame_transition_s: 2.0
```

该配置只在 `simulator.is_real == true` 时生效。缺省值为 `0.0`，从而保证其他 HITTER 配置未显式启用时保持旧行为。配置值必须是有限且非负的秒数。

### HitterEnv 状态

`HitterEnv` 增加以下内部状态：

- 真机第一帧过渡时长；
- 本轮 reset 是否仍等待执行第一帧过渡；
- 可测试的第一帧目标插值辅助函数。

`_prepare_hitter_reset_state()` 在每次 reset 时重新设置 pending 状态。只有真机且配置时长大于零时，pending 才为真。

### 第一帧目标

`HitterEnv.step(action)` 仍先执行现有 action 处理：

```text
smoothed_action = (1 - action_beta) * prev_action + action_beta * action
q_first = policy_default_joint_pos + policy_action_scales * smoothed_action
```

`q_first` 转换到 simulator 关节顺序后被冻结，作为本轮唯一的过渡终点。`prev_policy_action` 保持为这次真正采用的第一帧 action，使过渡结束后的下一次 policy 观测与已应用目标一致。

### 两秒插值

开始插值前调用 `simulator.get_state()`，以最新实测 `dof_pos` 作为起点 `q_start`，不依赖第一次 R2 校准时保存的旧状态。

步数按真机 policy 周期计算：

```text
steps = max(1, ceil(transition_s / simulator.high_dt))
```

当前 `high_dt = 0.02 s`，因此 `2.0 s` 对应 100 步。每步使用 smoothstep：

```text
u = step / steps
alpha = 3*u^2 - 2*u^3
q_cmd = (1 - alpha) * q_start + alpha * q_first
```

每步调用一次 `simulator.apply_action(q_cmd)`，并按照 `high_dt` 补足周期；计算或通信超时时不执行负数 sleep。最后一步严格等于 `q_first`。

插值作为首次 `HitterEnv.step()` 内的一次阻塞式、仅真机操作完成，因此 HitterAgent 在这两秒内不会推进到第二个 policy 帧。完成后 pending 置为假，并沿用现有 post-step 观测更新流程。

## 故障处理

- 时长为 NaN、无穷或负数：初始化时立即报错，不允许进入真机控制。
- 第一帧 action 或转换后的关节目标包含非有限值：在下发前报错。
- 实测起点维度不匹配或包含非有限值：在下发前报错。
- `apply_action()`、`get_state()` 或通信异常：保留异常并退出现有 Agent `try/finally`，由现有关闭流程停止环境；不静默跳过过渡。
- 过渡只有完整完成后才清除 pending 标志。

## 测试设计

新增独立测试覆盖：

1. 真机、`2.0 s`、`high_dt=0.02 s` 时产生 100 个插值目标。
2. 第一条命令接近起点，最后一条命令严格等于冻结的 policy 第一帧目标。
3. 插值系数单调，首尾采用 smoothstep，所有命令均为有限值。
4. 过渡期间仅使用传入的第一帧 action，不请求或采用后续 policy action。
5. 过渡结束后的下一次 `step()` 走原有单帧直接控制路径。
6. reset 后 pending 恢复，下一次进入 policy 再执行一次过渡。
7. MuJoCo 即使配置为 `2.0 s` 也不执行该过渡。
8. `0.0 s` 保持旧行为。
9. 非法时长、非法起点和非法第一帧目标在任何命令下发前失败。
10. mock 时间验证每步只补足剩余周期，不产生负 sleep。

## 成功标准

- 第二次 R2 后不再把完整 policy 第一帧目标一次性下发。
- 真机在 2.0 秒内以平滑曲线到达被冻结的 policy 第一帧目标。
- 两秒后 HITTER 原有 policy 循环、waiting observation、击球规划和动作语义保持不变。
- MuJoCo 输出和时序保持不变。
- RobotBridge3 相关单元测试与离线验证通过。
