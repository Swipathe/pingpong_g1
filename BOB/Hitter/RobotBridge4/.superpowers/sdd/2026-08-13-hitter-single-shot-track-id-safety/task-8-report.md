# Task 8 实施报告

## 修复轮 1

- 基线：`a90d82f`
- RED：TRACKING challenger 的 success/failure 四种组合均未消费；commit
  畸形 success 四种结构均绕过校验。聚焦运行得到 `8 failed, 1 passed`。
- 修复：任何已有 current identity 的异轨 completed result 都立即消费
  challenger；ARMED success 在 commit 判断前完成 typed command 校验和防御性
  复制，非法 command 通过 commit-aware `INTERNAL_ERROR` cancel 保留已承诺命令。
- 测试纯通过公开 `ingest()` 用两个递增 soft failure 建立 streak=2，不再直接
  写内部计数。
- GREEN：lifecycle `70 passed`；邻接 queue/failure/type/velocity 回归
  `58 passed, 2` 个既有 invalid-escape warning；三个文件 `py_compile` 及
  `git diff --check` 通过。
- 用户删除的两个旧 lifecycle 测试仍保持 `D`，未恢复、未修改、未暂存。

## 范围

- 基线：`1580fcb`
- 生产文件：`deploy/utils/hitter_realtime.py`
- 新测试：`deploy/tests/test_hitter_single_shot_lifecycle.py`
- 共享 fixture：`deploy/tests/hitter_test_factories.py`
- 未修改、未恢复用户已删除的：
  - `deploy/tests/test_hitter_realtime.py`
  - `deploy/tests/test_hitter_env_lifecycle.py`

## RED 证据

1. 首次运行新 lifecycle suite：collection 因生产模块尚无
   `LifecycleCancelReason` / `LifecycleDecision` 失败。
2. 有限时间补充回归：WAITING 下 `policy_tts(now=nan)` 未拒绝，得到
   `1 failed, 58 deselected`。
3. 安全边界补充回归：ARMED challenger success/failure 未消费、WAITING
   首个直接 track-end 未产生语义边沿，得到 `3 failed, 59 deselected`。

## GREEN 证据

```text
PYTHONPATH=deploy .../python -m pytest -q \
  deploy/tests/test_hitter_single_shot_lifecycle.py
62 passed in 0.21s
```

```text
PYTHONPATH=deploy .../python -m pytest -q \
  deploy/tests/test_hitter_completed_result_queue.py \
  deploy/tests/test_hitter_planner_failure_reasons.py \
  deploy/tests/test_hitter_runtime_identity_types.py \
  deploy/tests/test_hitter_planner_velocity_alignment.py
58 passed, 2 pre-existing invalid-escape DeprecationWarnings in 1.14s
```

`py_compile` 覆盖三个任务文件并通过。

## 自审结论

- `ingest()` 不再隐式推进 deadline；每次 `advance()` 最多跨一个 phase。
- process-lifetime 保存 consumed ID、消费原因、per-ID generation watermark、
  strike count、最后锁定字段和最后 failure/cancel 原因。
- `reset_for_policy_reentry()` 强制清理当前安全会话的瞬态状态，但不绕过或
  清除上述历史。
- first ARMED 锁定 side/base/deadline；连续 override 只重建 position 和
  velocity，并同步两个 velocity 字段及 `strike_table_y_w`。
- commit 使用精确边界，冻结整条命令；pre-commit immediate/third-soft
  failure 和 third discontinuity 均消费 ID 并进入 WAITING。
- RECOVERY 不缓存新结果；challenger ID 当场消费；每 ID strike count 最大为 1。
- sampler 异常/非法值转换为 typed `INTERNAL_ERROR` decision，不向 policy tick
  抛出。
- 三个允许路径之外没有 stage；两个用户删除测试仍保持 `D` 状态。
