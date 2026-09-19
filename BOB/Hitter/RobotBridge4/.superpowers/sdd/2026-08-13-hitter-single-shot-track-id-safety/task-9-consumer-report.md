# Task 9 子任务 A：RealWorld consumer session/eligibility API

日期：2026-08-13

## 范围

仅修改并计划提交：

- `deploy/simulator/real_world.py` 的 consumer 状态/API hunks；
- `deploy/tests/test_real_world_v2_consumer.py`。

明确未暂存 `real_world.py` 既有的 connection wait 和 R2 校准 hunks；未修改 HitterEnv、settings 或 YAML。未启动网络、真机、PD publisher。

## TDD 证据

基线：

```text
PYTHONPATH=deploy .../python -m pytest -q deploy/tests/test_real_world_v2_consumer.py
66 passed in 0.23s
```

RED 1（consume reason/validation/idempotency）：

```text
10 failed, 66 deselected
原因：现有 consume_hitter_track() 不接受 reason；因此精确 ID、非空 reason、原因记录和 active estimator quarantine 合同均失败。
```

GREEN 1：

```text
10 passed, 66 deselected
```

RED 2（session end 与 authoritative snapshot eligibility）：

```text
7 failed, 76 deselected
原因：end_hitter_policy_session() 和 hitter_snapshot_planning_eligible() 尚不存在。
```

GREEN 2：

```text
7 passed, 76 deselected
```

RED 3（成功 reentry 清理可恢复事件）：

```text
1 failed, 2 passed, 83 deselected
原因：begin_hitter_policy_session() 清 fault/dedupe flag，但未清上一 session 的可恢复 event deque。
```

GREEN 3：

```text
3 passed, 83 deselected
```

最终相关验证：

```text
PYTHONPATH=deploy .../python -m pytest -q \
  deploy/tests/test_real_world_v2_consumer.py \
  deploy/tests/test_hitter_runtime_identity_types.py
92 passed in 0.30s

PYTHONPATH=deploy .../python -m py_compile \
  deploy/simulator/real_world.py \
  deploy/tests/test_real_world_v2_consumer.py
PASS

git diff --check -- deploy/simulator/real_world.py \
  deploy/tests/test_real_world_v2_consumer.py
PASS
```

## 落地合同

- `end_hitter_policy_session(reason=...)` 在 consumer RLock 内先关闭 session，再消费/quarantine active，清 estimator；返回 sorted involved-ID tuple，同时保留 active/last/visible 物理诊断身份。
- `begin_hitter_policy_session()` 只有成功时过滤上一 session 的五类可恢复事件；永久 schema/overflow 仍返回 false，事件不动；event sequence 不重置。
- `hitter_snapshot_planning_eligible(snapshot)` 严格验证 canonical snapshot，并在 consumer RLock 内二次核对 session、fault、admitted/consumed、active、latest identity/generation/frame 及 process watermarks。
- consumer 生成 listener 通知前也使用同一 authoritative gate；delayed listener 在 consume/end/schema 后再次 gate 会返回 false。
- `consume_hitter_track(track_id, reason=...)` 要求 exact positive int 与非空 string；首次消费返回 true，幂等重复返回 false，但 `ball_track_consumption_reasons[ID]` 更新为最近原因；active 消费同步 latest snapshot consumed 并清 estimator。

## 提交边界

共享 index 一度同时包含另一 Task 9 子任务的三个文件。本子任务等待对方先提交；之后必须复核 index 仅剩：

```text
M deploy/simulator/real_world.py
M deploy/tests/test_real_world_v2_consumer.py
```

已提交：

```text
1c61ceb feat: gate HITTER consumer policy sessions
```

提交后复核：commit 仅含上述两个文件；index 为空；`real_world.py` 的 connection wait/R2 hunks仍为 unstaged，`test_real_world_v2_consumer.py` 已 clean。提交后再次运行相关测试为 `92 passed in 0.25s`，`py_compile` 通过。
