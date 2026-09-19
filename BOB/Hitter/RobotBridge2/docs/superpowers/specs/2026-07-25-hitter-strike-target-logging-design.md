# HITTER 单次击球期望速度日志设计

## 1. 目标

在真机和 MuJoCo 共用的 HITTER command 生命周期中，每个进入
`armed -> recovery` 的规划击球周期只记录一条日志，保存该周期最终被
policy 采用的：

- 入球预测速度 `v_ball_in_w`；
- 期望出球速度 `v_ball_out_w`；
- 期望球拍线速度 `v_racket_target_w`；
- 上述三个三维速度向量各自的欧氏模长。

所有向量均使用当前 planner 和 observation 已使用的桌面世界坐标系，
单位统一为 `m/s`。

## 2. 已确认的范围

### 2.1 包含

- 同一个 `epoch` 中持续到来的 `overridden` command 继续由现有
  `HitterCommandLifecycle.active_result` 管理。
- 在 `armed -> recovery` 边界读取 transition 前的
  `previous_active`；它就是最后一次成功接受的 command。
- 每个击球周期只向 Loguru 写一条单行、可搜索、可解析的 INFO 日志。
- 日志包含 `epoch`、`generation`、`strike_type`、三个速度向量和三个
  模长。
- 日志随现有 `eval.log` 自动落盘，同时显示在 policy 终端。
- 新增独立单元测试，验证“最后一次 override 胜出”和“一次击球只记录
  一次”。

### 2.2 不包含

- 不修改 observation 内容、顺序或维度。
- 不修改 planner 数学、落点、飞行时间、恢复系数或 command 生命周期。
- 不修改 ONNX action、PD 控制、LCM 消息或真机 R2 流程。
- 不记录实际球拍接触速度或实际出球速度；本日志只记录 planner 期望值。
- 不恢复此前每个 policy step 都执行的 `print()` 调试输出。
- 不创建独立 CSV/JSONL 录制进程。

现有 Loguru 文件和终端 sink 是同步写入。每个 strike 边界新增一条 INFO
日志会增加一次很小但非零的同步格式化和 I/O；它不会改变控制数值或状态
机语义，但不能宣称对循环耗时绝对为零。该成本相对同一边界已经存在的
transition INFO 日志可接受。

## 3. 方案比较

### 3.1 方案 A：在 `armed -> recovery` 记录最终 active result，采用

生命周期进入 recovery 时，直接读取现有 `previous_active` 中的速度
字段及 command 标识并输出一次。无需新增第二份 command 缓存。

优点：

- 与“最终被 policy 采用”的语义一致；
- 每球仅一行，不会被 100 Hz planner override 淹没；
- 不改变实时控制数据流；
- 现有 `eval.log` 即可保存。

### 3.2 方案 B：每次 `armed/overridden` 都记录，不采用

该方案可以保留 command 演化过程，但一次球会产生几十行甚至上百行，
影响现场阅读，也无法直接指出最终采用值。

### 3.3 方案 C：新增结构化 JSONL 录制器，不采用

该方案更适合完整离线分析，但需要额外文件生命周期、异常处理和同步
逻辑。当前需求只有每球最终期望速度，因此属于过度设计。

## 4. 数据流

现有 planner command 已携带：

```text
command.strike_plan.v_ball_in
command.strike_plan.v_ball_out
command.v_racket_target_w
```

修改后的数据流：

```text
planner result
  -> validate command vectors
  -> lifecycle ingest
  -> decision is armed/overridden
  -> lifecycle replaces active_result
  -> copy command into policy-facing state
  -> lifecycle reaches armed -> recovery
  -> log previous_active once
```

只有成功通过 command 校验且 lifecycle 接受为 `armed` 或
`overridden` 的 command 才能成为 `active_result`。失败、过期、重复、
太晚或畸形结果不会成为 `previous_active`，因此也不会进入击球日志。

## 5. 数据来源与辅助方法

`_update_hitter_command()` 在调用 lifecycle `advance()` 前已经保存：

```text
previous_phase
previous_active
```

当 `_log_hitter_advance_transitions()` 判定本帧跨过 strike deadline
时，`previous_active` 正是 recovery 前的最后一个 active planner
result，其中包含：

```text
previous_active.track_epoch
previous_active.source_generation
previous_active.command
```

因此不新增持久诊断状态。只新增一个窄范围日志辅助方法，负责：

1. 复用 `_validated_hitter_command_fields()` 读取并复制当前已经参与
   command 验收的 `v_ball_in` 和 `v_racket_target_w`；
2. 仅在诊断路径中读取、复制并校验 `strike_plan.v_ball_out`；
3. 计算三个欧氏模长；
4. 写出单行 Loguru 日志。

不得把 `v_ball_out` 加入 command ingestion 的主验收条件。它当前不参与
policy observation 或控制；如果为了日志而扩大主验收条件，会改变真机
能够接受的 command 集合，违背“诊断旁路不改变控制”的要求。

日志辅助方法在 transition 时复制三个向量，不保留
`previous_active.command` 内部 NumPy 数组的可变引用。

触发条件必须复用 `_log_hitter_advance_transitions()` 已有的
`crossed_strike` 分支，不得改成 `current_phase == RECOVERY`。这样即使
一次较晚的 `advance()` 同时越过 strike 和 recovery deadline，仍然只
记录一次正确的 strike 边界。

## 6. 日志格式

日志保持单行，建议格式：

```text
HITTER strike target: epoch=27 generation=8103 type=backhand
v_ball_in_w_mps=[-3.1000,0.2000,-1.0000] speed_ball_in_mps=3.2634
v_ball_out_w_mps=[4.2708,-0.1000,1.9000] speed_ball_out_mps=4.6760
v_racket_target_w_mps=[1.8000,0.0500,0.7000] speed_racket_mps=1.9310
```

实际输出为一行；上例换行只用于文档阅读。向量保留四位小数，模长保留
四位小数。字段名显式包含 `_w` 和 `_mps`，避免把世界系向量、速度模长
和 `tts` 混淆。

## 7. 异常处理

- `v_ball_in` 和 `v_racket_target_w` 复用现有 finite/shape 校验；
  `v_ball_out` 只做诊断校验，不改变 command 接受结果。
- 若跨过 strike deadline 时 `previous_active` 缺失或其 command 无法
  重新校验，只写一条 WARNING，不得中断 policy 或真机控制。
- 日志格式化异常不得改变 command 生命周期；诊断功能必须保持旁路。
- 日志辅助方法用 `try/except` 包住诊断校验、向量复制、模长计算、
  格式化和目标 INFO 调用；失败时尽力输出简短 WARNING。
- 不因日志需求在控制循环内执行文件打开、JSON 序列化或同步磁盘刷新。

## 8. 测试设计

由于当前工作区中的旧生命周期测试文件属于用户已有删除，不恢复或覆盖
它们。新增一个窄范围测试文件，至少覆盖：

1. `armed -> recovery` 只输出一次；
2. 日志使用 `previous_active` 的 epoch、generation、类型和三个向量；
3. 同一 epoch 被 override 后，transition 的 `previous_active` 指向最后
   一次 accepted override；
4. 三个模长等于对应向量的 `numpy.linalg.norm()`；
5. 非 strike transition 不输出 `HITTER strike target:`；
6. 缺失或非法 `previous_active` 只 warning，不抛异常；
7. 非法 `v_ball_out` 只让诊断日志退化为 warning，不改变 command
   ingestion 行为；
8. `now` 同时越过 strike deadline 和 command end deadline 时，仍恰好
   记录一次旧 `previous_active`，不能漏记或改记下一球。

测试不连接 ChingMu、LCM 或真机，不运行 ONNX，只对 active result 和
日志边界做单元验证。

## 9. 验收标准

- 每个规划击球周期在 `eval.log` 中恰好出现一条
  `HITTER strike target:`。
- 同一 epoch 的多次 override 不产生额外期望速度日志。
- 记录值来自 recovery 前最后一次成功接受的 command。
- 日志中的三个模长与向量一致，单位和坐标系明确。
- targeted test、相关 HITTER 测试、Python 语法检查和
  `git diff --check` 全部通过。
- 现有 observation、action、command 接受规则和生命周期状态转移没有
  行为变化。
