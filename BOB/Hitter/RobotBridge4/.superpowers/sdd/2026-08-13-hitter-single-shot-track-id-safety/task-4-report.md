# Task 4 报告：canonical runtime identity 与 `track_id` 迁移

## 状态

- 实现完成；实现提交为 `0ad658dc9179b28e6376fc7b6e76467a22e4d289`。
- 变更限于 RobotBridge4；未修改 RobotBridge3/2、MOSAIC、Omega、`unitree_sdk2` 或 `build`。
- 未启动真机、PD、仿真 viewer 或网络运行时。

## TDD：RED / GREEN

- RED：先创建 `deploy/tests/test_hitter_runtime_identity_types.py`，运行
  `PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q deploy/tests/test_hitter_runtime_identity_types.py`。
  收集期按预期失败：`ModuleNotFoundError: No module named 'utils.hitter_runtime_types'`。
- GREEN：新增唯一 canonical `utils.hitter_runtime_types.SnapshotKey`，diagnostics 仅 import/re-export；身份合同测试变为 6 passed。
- 字段合同：真实构造 `track_id=9,generation=1,source_frame=1` 的完整 `BallEstimateSnapshot`，显式断言 `consumed=False,new_track=True`，且 snapshot/result 均无 `track_epoch` 属性。

## 接口与迁移文件

- 新接口：`SnapshotKey(track_id, generation)`，严格拒绝非正整数 `track_id`、负数/非整数 `generation`；JSON 为 schema v2 的 `track_id`。
- `BallEstimateSnapshot.track_id`、`PlannerResultSnapshot.track_id`、`IncomingTrackSnapshot.track_id`；`consumed/new_track` 只加入保守默认 `False`，未增加准入或消费行为。
- runtime：`deploy/utils/hitter_realtime.py`、`deploy/simulator/real_world.py`、`deploy/envs/hitter.py`。
- diagnostics/UI：models、attempts、events、pipeline、monitor、replay、monitor HTML 全部机械迁移为 `track_id`。
- tests/fixtures：identity、MuJoCo v2、runtime factory、strike log、attempts/events/frontend/input adapter/monitor/pipeline/replay/integration。
- MuJoCo/本地 adapter ID 从 1 开始；generation、发球调度、生命周期判断、ONNX 104-D、PD 和 5 秒过渡逻辑未改变。

## `rg` 盘点

- 迁移前：`rg -o '\btrack_epoch\b' deploy --glob '*.py' --glob '*.html' | wc -l` 为 164 个精确 token，分布在 19 个文件（含只读用户未跟踪物理测试）。
- 迁移后：`rg -n '\btrack_epoch\b' deploy/simulator deploy/envs deploy/utils deploy/diagnostics --glob '*.py' --glob '*.html'` 无输出。
- 用户未跟踪 `deploy/tests/test_mujoco_physical_table_tennis.py` 未编辑、未暂存；其中旧词不属于 active dirs 验收范围。

## 测试

- 聚焦/代表性 GREEN：187 passed, 1 skipped, 2 warnings（identity、MuJoCo v2、events、pipeline、strike logging、frontend、attempts、replay、runtime factory、monitor）。
- brief 指定四文件：identity/MuJoCo/events 通过；input adapter 44 个相关/其余测试通过，但全文件仍有 3 个与 Task 4 无关的既有 G2 fixture 失败（期望仍写 G1 或一条 “retired G1” 测试实际发送 G2）。未夹带修正用户 hunk。
- diagnostics integration + factory + monitor：44 passed，2 个既有 G2/diagnostics parity 失败；失败在用户已改 G2 fixture，非 identity 断言。
- `python -m py_compile` 覆盖全部迁移生产模块通过；`git diff --check` 与 cached check 通过。

## staging 与用户 dirty 保护

- clean paths 精确 `git add <paths>`；7 个交叠文件先 `git restore --staged <exact paths>`（仅取消暂存）后，从 `git diff HEAD` 按明确身份 hunk 生成 patch 加入 index。
- `git diff --cached --name-status` 仅有 Task 4 的 24 个 M/A 文件（含本报告），无 D；无 pre-existing deletion、G2/104-D/R2/connected/planner config hunk。
- 交叠文件提交前均为 `MM`，证明 identity hunk 在 index、用户既有 hunk仍留在 worktree；用户物理测试仍为 `??`。

## 自审

- canonical class 只定义一次；active runtime 直接从 utils import，未从 diagnostics 导入身份 key。
- 没有 legacy fallback、alias 或 `track_epoch` JSON；v2 decode/freshness/admission 未提前实现。
- `consumed/new_track` 无 gate/集合/状态机使用；RealWorld 暂不赋 wire-aware 真值。
- lifecycle 的比较、去重、late/ended/cache 语义仅变量与字段 rename，无分支变化。

## Concerns

- 当前工作树整体存在大量用户修改/删除；本提交只包含上述身份迁移。
- 两组非 identity 基线失败来自实施前用户 G2 fixture 与 diagnostics 对 G1 subject 的现有假设冲突，报告保留证据但未扩大范围修复。

## SDD 本地报告清理

- 只读确认 `.git/info/exclude:7` 仍为 `.superpowers/`。
- 使用 `git rm --cached --` 仅解除本报告的 Git 跟踪，磁盘文件保持存在且内容完整。
- 清理提交：`b37ede9` (`chore: keep SDD task report local`)；提交前 cached name-status 仅为本报告路径 `D`，cached check 通过。
- 提交后 `test -f` 通过，`git check-ignore -v` 命中 `.git/info/exclude:7:.superpowers/`，报告不出现在 `git status --short`，index 为空。
- 七个用户交叠 dirty 文件与 controller plan doc 状态未改变；今后不再暂存或提交 `.superpowers/sdd` artifact。
