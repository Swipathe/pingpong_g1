# RobotBridge4 ARMED 阶段保留 last-good 命令设计

## 背景与决策

RobotBridge4 当前在首次成功规划后进入 `ARMED`。在 commit 之前，如果同一轨迹连续三次返回软失败，状态机会撤销 active command 并回到 `WAITING`。实机日志已经出现两类会过早触发该撤销的结果：

- `HIT_HEIGHT_OUT_OF_RANGE`
- `OVERRIDE_DISCONTINUITY`

本次变更采用以下明确决策：这两类结果只拒绝新的规划结果，继续保留同一轨迹最近一次已接受的 last-good command，不再触发“三次后撤拍”。其他软失败和全部硬故障的现有撤销语义保持不变。

这是对 RobotBridge4 现有单拍安全设计的一项有意例外：只要轨迹身份和硬安全条件仍有效，即使后续高度判断或候选目标连续性检查失败，也允许机器人继续执行此前已经接受的击球命令。

## 范围

只修改 RobotBridge4 的 `HitterCommandLifecycle` 及其相关测试。RobotBridge3 不修改。

本次不包含：

- 改变击球面配置；击球面 `x=0` 属于已经单独恢复的配置事实。
- 改变首次进入 `ARMED` 的条件。
- 增加预先连续多帧成功才允许 arm 的门槛。
- 调整位置、速度、deadline 连续性阈值。
- 改变 commit、`RECOVERY`、一球一拍或 consumed track ID 语义。
- 放宽 Vicon、底盘位姿、轨迹身份、非有限值或内部异常的安全检查。

## 状态机合同

### ARMED 且尚未 commit

对 active track 的 completed result 按下表处理：

| 结果 | active command | 连续可撤销失败计数 | 状态与诊断 |
| --- | --- | --- | --- |
| 连续性检查通过的成功结果 | 按现有规则更新 last-good | 清零 | 保持 `ARMED`，`overridden` |
| `HIT_HEIGHT_OUT_OF_RANGE` | 不覆盖，保留 last-good | 不增、不清零 | 保持 `ARMED`，`retained_failure` |
| `OVERRIDE_DISCONTINUITY` | 不覆盖，保留 last-good | 不增、不清零 | 保持 `ARMED`，`retained_discontinuity` |
| `ESTIMATOR_NOT_READY` | 不覆盖，保留 last-good | 加一 | 前两次保持 `ARMED`，第三次按现状撤销 |
| `BALL_NOT_INCOMING` | 不覆盖，保留 last-good | 加一 | 前两次保持 `ARMED`，第三次按现状撤销 |
| `NO_FUTURE_CROSSING` | 不覆盖，保留 last-good | 加一 | 前两次保持 `ARMED`，第三次按现状撤销 |
| 立即失败或运行时硬故障 | 撤销 | 不适用 | 按现状回到 `WAITING` 并记录原因 |

两类“仅保留”结果不参与连续可撤销失败计数：既不增加，也不清零。这是对现有逻辑的最小修改，并保留其他软失败原有的累计撤销能力。例如，两次 `BALL_NOT_INCOMING` 之后即使穿插 `HIT_HEIGHT_OUT_OF_RANGE`，下一次可撤销软失败仍会达到第三次并触发撤销。

`HIT_HEIGHT_OUT_OF_RANGE` 与 `OVERRIDE_DISCONTINUITY` 可以任意连续出现；只要没有其他撤销条件，active track、锁存的 strike type、base target、strike deadline 和 last-good racket command 均保持不变。

### 保持不变的撤销条件

下列条件继续立即撤销，不受本次例外影响：

- `TRACK_ENDED`
- `BASE_POSE_INVALID`
- `NONFINITE_INPUT_OR_OUTPUT`
- `INTERNAL_ERROR`
- Vicon schema 错误或输入流超时
- 底盘位姿无效或超时
- active track ID 冲突
- completed-result queue 溢出

任何未被识别的 failure reason 仍按 `INTERNAL_ERROR` 处理，不得退化为保留命令。

### commit 之后

commit 窗口内的现有 `retained_committed` 行为完全不变。本次修改只影响 commit 前的两类结果，不能改变 strike deadline，也不能延后或重新计算 commit 边界。

### 轨迹结束与一球一拍

`TRACK_ENDED` 仍立即终止 active command。命令进入执行与恢复后，原有 consumed track ID 和一球最多一拍约束保持不变；“仅保留”结果本身既不消费新 ID，也不创建第二条命令。

## 数据与可观测性

- 被拒绝结果仍按现有入口规则推进 completed-result generation 水位，避免重复处理同一结果。
- 两类结果都必须写入 `last_failure_reason`，并沿用现有 `retained_failure` 或 `retained_discontinuity` decision kind，便于从日志确认新结果被拒绝而 active command 被保留。
- 两类结果不得修改 `active_result`、锁存字段或 strike deadline。
- 本次不新增配置项；`armed_cancel_consecutive_failures` 继续只控制其余三类可撤销软失败。

## 风险边界

该选择保留了一个明确风险：如果新的高度判断或连续性检查持续失败，机器人仍可能按较早的 last-good command 挥拍。该风险由本次产品决策接受，但只限于这两个 reason；轨迹结束、身份冲突、输入失效、底盘失效、非有限值、内部异常和队列溢出仍具有撤销优先权。

本次设计不允许将“保留 last-good”扩展成永久忽略所有 planner failure，也不允许删除通用 `cancel()` 路径。

## 验收标准

1. 在 commit 前，同一 active track 连续返回任意次数 `HIT_HEIGHT_OUT_OF_RANGE`，生命周期始终保持 `ARMED`，且 active command、锁存字段和 deadline 不变。
2. 在 commit 前，同一 active track 连续产生任意次数 `OVERRIDE_DISCONTINUITY`，生命周期始终保持 `ARMED`，且候选结果不覆盖 last-good command。
3. 任一“仅保留”结果都不会增加或清零先前的可撤销软失败计数。
4. `ESTIMATOR_NOT_READY`、`BALL_NOT_INCOMING`、`NO_FUTURE_CROSSING` 三类结果累计达到现有阈值时，仍按现状撤销；三类之间切换或穿插两类“仅保留”结果均不清零，只有通过连续性检查的成功更新才清零。
5. 所有立即失败和运行时硬故障仍立即撤销；未知 reason 仍 fail closed。
6. commit 后行为、一球一拍、track 结束、恢复流程和 generation 去重行为保持原样。
7. RobotBridge3 文件无任何改动。

## 验证要求

实现阶段需要新增或更新聚焦单元测试，覆盖：

- 重复高度越界保留 last-good；
- 重复 override 不连续保留 last-good；
- 两类仅保留结果不改变其他软失败的连续计数；
- 其余三类软失败的第三次撤销；
- 所有立即失败仍撤销；
- commit、track end、一球一拍和 decision kind 回归。

完成聚焦测试后，还需运行 RobotBridge4 的相关生命周期与运行时集成测试。验证只证明软件合同；真机再次发球前仍应先以日志确认 `ARMED` 中上述两类结果显示为 retained，而不是 `armed -> waiting`。
