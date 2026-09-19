# Task 7 实现报告

## 结果

- 状态：完成
- 分支：`local/hitter-task-diagnostics-20260727`
- Task 6 review 修复：`bcfcfe3 fix: harden HITTER mocap snapshots`
- Task 7 提交：`1ccf3f6 feat: trace HITTER shadow task pipeline`
- Task 7 commit 恰好包含：
  - `deploy/utils/hitter_realtime.py`
  - `deploy/diagnostics/hitter_task_pipeline.py`
  - `deploy/tests/test_hitter_task_pipeline.py`

## Worker trace

- `LatestOnlyPlannerWorker` 在同一 condition 临界区原子保存 production
  `PlannerResultSnapshot` 与 `FrozenPlannerResult`，公开
  `latest_result_bundle()`。
- trace 序号在 worker condition 内全局递增；独立 delivery condition 保证
  多线程 callback 仍严格按 `trace_seq` 交付，同时 listener 从不在 worker
  condition 内执行。
- 覆盖 `submit`、pending replacement、start、complete 和 latest-result
  replacement；每条 trace 携带变化后的 stats。
- command 通过 dataclass/ndarray 递归转换并深冻结；错误 type/text 分离，
  text 限制为 2048 字符，production result/error 契约保持不变。
- listener 异常只增加 `trace_listener_failures`；有限 close 可重试，
  `close(None)` 保留无限 join 并返回成功状态。
- `IncomingTrackConfirmation.snapshot()` 返回 immutable epoch/count/latch。

## Shadow pipeline

- ingest 顺序固定为 attempt observe/bind、nonblocking raw offer、基于
  `received_monotonic_s` 的事件驱动 100 Hz throttle、worker submit。
- plan function 精确保留 invisible reset、not-ready/base-invalid 保留 count、
  incoming observe、base quaternion normalize/forward 和完整 planner 顺序。
- tick 顺序固定为 lifecycle advance、strike crossing 同步 adapter reset 和
  tracker production reset、单次 atomic result bundle、latest pelvis copy、
  `policy_tts(obs_now)`、共享 11 维 helper。
- active/cached binding、production result、frozen command fields 和 task
  observation 强制同源；使用 `command.v_racket_target_w`，不误用
  `strike_plan.v_racket_target`。
- 只有 ARMED、identity 一致、11 维 float32 finite、无 clip、TTS 合法时 PASS；
  每个 attempt 只发布首次 PASS transition。
- recovery cached result 在 `advance()` 内重新 arm 时保留 advance decision，
  不被持久化 latest bundle 的 duplicate 覆盖。
- event/raw sink 异常被隔离并计数，不反抛到 ingest、worker callback 或 tick；
  所有 AttemptTransition 即使没有 snapshot key 也保持 attempt scope。

## RED → GREEN

首次 RED：

```text
ImportError: cannot import name 'ShadowTaskPipeline'
```

随后分阶段 RED 覆盖 bytes-backed snapshot、严格 trace delivery、
首次 PASS 去重、cached recovery decision 和 sink 异常隔离。

最终 Task 7：

```text
Ran 13 tests in 0.117s
OK
```

## 最终验证

- Task 1 observation：11/11
- Task 6 input adapter：15/15
- Task 7 pipeline：13/13
- strike target logging：6/6
- Python 3.8 `py_compile`：通过
- `import envs.hitter`、`import utils.hitter_realtime`、
  `import diagnostics.hitter_task_pipeline`：通过
- `git diff --cached --check`：通过

## 边界

- 未恢复任何已删除旧测试。
- `hitter_realtime.py` 使用交互式 hunk staging；Task 7 commit 未包含用户
  dirty 文件或其他任务文件。
- 本报告为 SDD 编排元数据，未纳入 Task 7 commit。
