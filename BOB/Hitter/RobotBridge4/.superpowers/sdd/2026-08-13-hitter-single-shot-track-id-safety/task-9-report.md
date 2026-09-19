# Task 9 主运行时集成报告

## 结论

Task 9 已按严格 TDD 完成 RobotBridge4 HITTER 单拍运行时集成。实现只在纯离线 fake 上验证；未启动 Unitree publisher、LCM 网络、PD 真机控制或真实机器人进程，也未修改本任务明确排除的 diagnostics、replay、`deploy/config/hitter.yaml`、`deploy/config/mimic/hitter.yaml` 和 `deploy/simulator/real_world.py`。

基线提交：`70c91ad6c416ebc839792def0016ab38a529ccad`。

## 实现摘要

- lifecycle、planner worker、listener 与 lifecycle `threading.RLock` 每进程只创建一次；普通 reset/reentry 复用对象并通过 typed `reset_for_policy_reentry()` 隔离旧状态。
- 维护进程生命周期内所有 submitted track ID；reset/reentry 关闭 admission/session，消费 active/submitted/completed/overflow ID，且晚到旧结果只能 ignored-consumed。
- listener 在 lifecycle RLock 内完成最终 eligibility 复核；锁顺序固定为 lifecycle RLock 后 RealWorld consumer lock。RECOVERY、新轨迹非 WAITING、当前已消费或 consumer-ineligible 的 snapshot 均 fail closed。
- 真机 policy tick 固定为 status/freshness、direct events/cancel、一次 advance、一次 completed batch drain、顺序 ingest；overflow 整批零 ingest 并消费所有受影响 ID。
- 所有 typed `LifecycleDecision` 由单一入口应用；消费同步到 simulator，policy command 只从 lifecycle 的 post-decision `active_result.command` 复制。
- 真机 agent 既有代码已在 ONNX 前调用 `refresh_policy_observation()`，无需产生新的 agent diff；真机 `_post_physics_step()` 不再二次 tick，MuJoCo 保留同步更新。
- `HitterEnv._physics_step()` 自包含执行一次 `simulator.apply_action(self.action)`，不依赖用户未暂存的全局 `BaseEnv` hunk，也不会在脏树中重复下发。
- 真机 WAITING 使用锁存的当前 G2Pelvis 世界 xy anchor；reset/reentry 先用同一显式 monotonic 时间执行 freshness status，再刷新状态和捕获 anchor。stale/cached pose、invalid pose 和 permanent fault 均保持 session/admission 关闭。
- 5 秒 transition 全程关闭 session/admission；结束时再次刷新/捕获、再次 quarantine transition 期间结果，再 begin session。零时长 reset 与 hard reset 走同一即时 reentry。
- MuJoCo 的 YAML waiting target 与 RECOVERY 到下一球 reset 语义保持不变；WAITING observation 固定 `[1,104]`，每 tick 继续 ONNX 与 `apply_action()`。
- 正常 per-ball callback 不打印；状态汇总最多 1 Hz；ARMED/WAITING transition、撤拍原因即时且同 session 去重；strike target 保持 exactly once。

## 审查中发现并补强的边界

- hard reset 不经过 `env.reset()`：补齐 transition 后重开与零时长立即重开。
- invalid old anchor 不能继续生成首帧 target；reset 必须取得本次新鲜有效 anchor。
- typed migration 曾遗漏 MuJoCo `RECOVERY -> WAITING` 的下一球 reset 副作用，现绑定到 `decision.entered_waiting`。
- Task 8 lifecycle 合同补强：`strike_side_source` 仅允许 `table_y|forced`；`table_y` 必须遵循 y<0 forehand、y>=0 backhand；malformed command 在 ARMED 前 typed `INTERNAL_ERROR` cancel/consume。
- override 不能改变首拍锁定的 side source，避免合法候选合并后生成内部不一致 command。
- `HitterWbcCommand` 暴露只读 `expected_strike_type_from_table_y` 与 `strike_type_consistent`，供后续 diagnostics 直接投影生产判定。
- 非有限或无法表示为 float32 的 `target_base_height_w` 在 runtime 初始化时 fail fast，避免 lifecycle 已 ARMED 后才在 Env copy 抛错。

## TDD 证据

初始 RED：新 waiting-anchor/runtime integration 组合为 `29 failed, 2 passed`，明确暴露缺少 anchor API、旧 latest-result 控制路径、reset 重建、队列容量未注入、listener 缺锁内 gate，以及真机 post-step 二次 tick。

后续逐项 RED 还包括：

- 零时长 hard reset 留下关闭 session：`1 failed, 1 passed`。
- invalid old anchor reset 与 MuJoCo recovery reset：`2 failed`。
- ARMED transition 与重复 cancel 即时日志：`2 failed`。
- 同 tick status 与 identityless event 的同原因撤拍重复日志：`1 failed`。
- stale status 与 ended-track event 的同原因撤拍重复日志已归并，但 event track ID 仍同步消费。
- 非法 side source/table-y 手型不一致：`3 failed, 1 passed`。
- override side-source 跨零不变量：`1 failed, 1 passed`。
- reentry 缺少 recapture 后第二次 quarantine：`1 failed`。
- 非有限 base height 初始化未拒绝：`1 failed`。
- stale 但 cached-valid pelvis reset 未拒绝：`1 failed`，且 `status_calls=0`。

最终 GREEN：

```text
working tree combined focused + adjacent: 333 passed in 2.43s
clean staged-index Git archive combined: 333 passed in 2.43s
clean commit `0fe2f06e077b922f639a64b4fffdfa4a81561adb` Git archive combined: 333 passed in 2.43s
py_compile: PASS
git diff --check: PASS
```

focused 文件：runtime factory、waiting anchor、runtime single-shot integration、first-frame transition、task observation、strike-target logging。

adjacent 文件：single-shot lifecycle、completed-result queue、RealWorld v2 consumer、runtime identity types、planner failure reasons、MuJoCo track-id v2。

测试通过 `PYTHONPYCACHEPREFIX=$(mktemp -d)` 隔离仓库旧 `__pycache__`，并仅忽略既有 Torch JIT docstring `DeprecationWarning`；没有删除用户缓存或文件。

## 精确脏树边界

本任务提交仅包含以下文件：

- `deploy/envs/hitter.py`
- `deploy/utils/hitter_realtime.py`
- `deploy/utils/hitter_planner.py`（只包含两个只读派生属性；用户既有 velocity/height diff 未暂存）
- `deploy/tests/hitter_runtime_test_harness.py`
- `deploy/tests/test_hitter_waiting_anchor.py`
- `deploy/tests/test_hitter_runtime_single_shot_integration.py`
- `deploy/tests/test_hitter_policy_first_frame_transition.py`
- `deploy/tests/test_hitter_task_observation.py`
- `deploy/tests/test_hitter_strike_target_logging.py`
- `deploy/tests/test_hitter_single_shot_lifecycle.py`
- `deploy/tests/test_hitter_completed_result_queue.py`

`deploy/envs/hitter.py` 基线已有 G2 wording、104-D xy slice 与 per-tick print removal；它们分别是本任务的 G2 anchor、104-D observation 和日志负载要求，因此随 Task 9 一并纳入。`deploy/agents/hitter_agent.py` 的用户既有 per-tick print removal 未暂存；其余所有既有修改、删除和未跟踪文件均保留原状。

## 剩余限制

- 本任务没有进行真机、LCM、Unitree publisher 或 PD 验证；上线前仍需按真实硬件 preflight 验证 Vicon freshness、G2Pelvis 对齐、网络和 resolved policy config。
- diagnostics/replay 的 settings 构造与投影属于 Tasks 10/13，本任务未运行或修改那些套件。
