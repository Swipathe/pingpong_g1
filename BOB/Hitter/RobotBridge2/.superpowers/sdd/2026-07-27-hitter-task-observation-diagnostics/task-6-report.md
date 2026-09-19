# Task 6 实现报告

## 结果

- 状态：完成
- 分支：`local/hitter-task-diagnostics-20260727`
- 提交：`9bff193 feat: normalize HITTER mocap inputs`
- 提交范围严格为：
  - `deploy/diagnostics/hitter_task_pipeline.py`
  - `deploy/tests/test_hitter_task_input_adapter.py`

## 实现

- 使用 Task 2 factory 构造生产等价的 `BallStateEstimator`，adapter 不负责
  LCM decode/handle，也不创建 attempt、incoming 或 lifecycle 状态。
- 所有成功交给 adapter 的 decoded message 按到达顺序分配全局单调
  `input_seq`；subject 大小写无关地规范化并保留 unknown 输入。
- 标准化 sample 由已有 model 复制为 bytes-backed、只读 `float64`；
  `BallEstimateSnapshot` 中 ball/base 数组保持生产 `float32`。
- estimator 时间优先级为 positive finite Vicon time、positive publish time、
  独立从 0 开始的配置采样率 fallback；重复/倒退时间仍由生产 estimator
  执行 `last + 1e-6`。
- invalid ball 同步 reset estimator、epoch 加一、generation 加一并形成保留
  旧 position/velocity 的 invisible snapshot；strike reset 只推进 epoch，
  不生成 snapshot、不增加 generation、不覆盖 latest snapshot。
- 新增 immutable `ProductionReset`、`LatestPelvisPose` 和扩展
  `AdapterOutput`；同一个 `RLock` 串行 ingest、strike reset 和 pelvis copy。
- invalid pelvis 只更新 latest validity/frame/time，保留最后一次 accepted pose；
  ball snapshot 使用到达时 accepted pose 与当前 production `base_valid`。
- frame gap、pelvis mismatch/stale、clock offset 只产生 warning，不修改生产
  validity；clock warning 由同一 subject 连续 source/publish pair 推断，不用
  `wall_time-source_time` 伪装网络延迟。
- valid nonfinite ball 保留 raw sample，捕获 estimator `ValueError`，返回
  `snapshot=None` 与 `ESTIMATE_NONFINITE`，不擅自 reset 状态或 epoch。

## RED → GREEN

RED 命令：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python \
  -m unittest discover -s tests -p 'test_hitter_task_input_adapter.py' -v
```

首次结果：退出码 1，按预期因
`ModuleNotFoundError: No module named 'diagnostics.hitter_task_pipeline'`
失败。

GREEN：同一命令运行 14 项，`Ran 14 tests ... OK`。测试使用真实
`transformation_t.encode/decode`，并通过 `RealWorld.__new__()` 对照
near-zero pelvis quaternion 的 production validity/float32 行为。

## 最终验证

- Task 6 unittest：14/14 通过。
- Task 2 runtime factory 回归：8/8 通过。
- Python 3.8 `py_compile`：通过。
- `git diff --cached --check`：通过。
- commit 前 cached name-status：恰好两个 Task 6 新文件。

## 边界

- 未修改或恢复用户 dirty 文件及已删除旧测试。
- 本报告属于 SDD 编排元数据，未纳入 Task 6 commit。
