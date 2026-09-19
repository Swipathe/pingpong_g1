# Task 4 实施报告

## 状态

- 基线提交：
  `aab6d66ce0ac4b077d2111be1371a310e65999ac`
- Task 4 提交：
  `6fede45ca1a029ec58fa35269a21775b811fc16b`
- 提交信息：
  `feat: add ChingMu pelvis orientation calibration mode`
- 提交范围只有：
  - `deploy/mocap_bridge/chingmu_table_lcm_bridge.py`
  - `deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py`
- 用户原有脏文件和未暂存删除的 legacy 测试均未恢复、覆盖或暂存。

## 实现

- 新增 CLI：
  - `--save-pelvis-orientation-calib PATH`
  - `--pelvis-calib-sec FLOAT`
- 新增 `_operation_mode(args)`，区分：
  - `table_calibration`
  - `pelvis_calibration`
  - `runtime`
- pelvis 标定要求已有 `--table-calib` 参数，禁止 `--publish`，并与
  `--pelvis-orientation-calib` 互斥。
- 缺少 orientation 只允许严格的 table-only 标定模式。
- `_collect_calibration_frames()` 可按调用方输出
  `--calib-sec` 或 `--pelvis-calib-sec` 的正确错误信息。
- pelvis 标定打印四元数、样本数、source duration、位置 RMS 和角度 RMS，
  原子保存成功后立即返回。
- runtime 在 LCM publisher、bridge 和发布循环之前严格加载并校验
  orientation 文件，无 identity fallback。

## TDD 证据

### RED

先只向 focused test 增加 7 个 `PelvisOrientationCliTest` 测试，然后运行：

```bash
conda run --no-capture-output -n rb python -m unittest \
  deploy.mocap_bridge.tests.test_chingmu_pelvis_orientation.PelvisOrientationCliTest \
  -v
```

结果退出码为 `1`，测试模块按预期失败：

```text
ImportError: cannot import name '_operation_mode'
Ran 1 test in 0.000s
FAILED (errors=1)
```

失败原因是待实现接口不存在，不是测试拼写或环境错误。

### GREEN

完成最小实现后运行：

```bash
conda run --no-capture-output -n rb python -m unittest \
  deploy.mocap_bridge.tests.test_chingmu_pelvis_orientation \
  -v
```

结果：

```text
Ran 22 tests in 0.043s
OK
```

其中新 CLI mode 测试为 `7/7`，原 focused 测试为 `15/15`。

## Legacy 回归

工作树中的
`deploy/mocap_bridge/tests/test_chingmu_table_lcm_bridge.py`
保持删除状态。测试源码直接来自
`HEAD:deploy/mocap_bridge/tests/test_chingmu_table_lcm_bridge.py`，通过 stdin
内存编译，并把 `__file__` 设置为其原始仓库路径，确保标定 fixture 的相对
路径语义与真实文件运行一致。

结果：

```text
Ran 25 tests in 0.037s
OK
```

最初的普通 stdin 试跑通过 `24/25`，唯一错误是
`__file__=<stdin>` 导致测试把 fixture 路径解析到仓库外；修正内存执行的
逻辑文件名后完整 `25/25` 通过。全过程未恢复 legacy 文件。

## `main()` 分支顺序证据

提交中的行号顺序：

```text
1056  operation_mode = _operation_mode(args)
1098  table_calibration 分支
1104  table_calibration return 0
1106  pelvis_calibration 分支
1113  使用 --pelvis-calib-sec 采集
1125  打印 quaternion 与质量指标
1128  原子保存 pelvis orientation
1138  pelvis_calibration return 0
1140  runtime 严格加载 orientation
1152  lc_client 初始化
1156  lcm.LCM publisher 构造
1163  ChingMuTableLcmBridge 构造
1173  发布循环
```

因此两个 calibration 分支都不会创建 LCM publisher 或进入发布循环；
runtime 的 orientation 加载校验严格早于 publisher、bridge 和循环。

## 其他验证

- `py_compile`：两个目标文件退出码 `0`，无输出。
- CLI help：退出码 `0`，同时命中：
  - `--pelvis-orientation-calib`
  - `--save-pelvis-orientation-calib`
  - `--pelvis-calib-sec`
- duration 错误检查：
  `--pelvis-calib-sec must be positive`。
- 禁用旧名称检查无输出：
  `pelvis_offset_heading_m|initial_base_yaw|relative_yaw|yaw_quaternion_from_rotation`。
- 两个目标文件的 `git diff --check` 和提交前
  `git diff --cached --check` 均为退出码 `0`。
- 提交前 `git diff --cached --name-status` 只包含两个目标文件。
- `deploy/mocap_bridge/calibrations` 下没有生成
  `*pelvis*orientation*.json`，提交也没有包含 calibration JSON。

## 只读 Review

- Critical：无。
- Important：无。
- 检查了模式优先级、互斥条件、严格 table-only 条件、错误参数名、原子保存
  调用、提前返回，以及 runtime 加载与 publisher/loop 的先后关系。

## Concerns

- 无已知代码问题。
- 按任务边界未执行 Task 5 现场标定，也未生成真实 pelvis calibration
  JSON；现场仍需在操作者确认机器人姿态与策略停机后单独完成。
- 本报告按要求不提交。
