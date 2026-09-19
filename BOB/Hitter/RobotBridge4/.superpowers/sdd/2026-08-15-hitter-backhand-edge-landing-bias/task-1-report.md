# Task 1 Report: 反手边缘落点偏置

## 范围

只修改了 Task 1 brief 指定的 planner 和新增测试；没有暂存、提交、还原或覆盖现有工作树改动。

## RED

先创建 `deploy/tests/test_hitter_backhand_edge_landing_bias.py`，覆盖：平滑且有界的有效落点 Y、完整 `plan()` 的 0.48 s 终点、默认/禁用输出一致性、强制击球类型透传，以及参数和落点校验。

初次按 brief 运行时，当前脏工作树因已删除 `deploy/tests/__init__.py` 导致 `ModuleNotFoundError: No module named 'utils'`；这与新增功能无关。仅在新测试中显式加入 deploy 根目录后，重新运行相同命令得到预期 RED：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4/deploy
conda run --no-capture-output -n rb pytest -q tests/test_hitter_backhand_edge_landing_bias.py
```

关键输出：`14 failed`，全部失败原因为 `TypeError: __init__() got an unexpected keyword argument 'backhand_edge_landing_start_y_w_m'`（其他两个新参数同理），证明测试在功能接口尚未存在时失败。

## 实现

- 为 `StrikePlanner` 添加三个 brief 规定的构造参数，默认 decrement 为 `0.0`。
- 新增 `desired_landing_point_for_strike()`：使用既有 table-Y 判定或既有显式类型校验；反手按 brief 的 smoothstep 计算并下调有效目标 Y。
- 启用偏置时校验阈值、半桌宽、基准和最终落点 Y；所有三个值始终校验为有限且非负。
- `plan()` 在计算出球速度前解析击球类型，并将同一有效落点用于 shooting 初值和每轮端点误差。
- `HitterSystemPlanner.plan_command()` 在实际 `StrikePlanner` 接到强制类型时透传该类型，同时保留旧式 duck-type planner 的两参 `plan()` 兼容性。

## GREEN 与回归

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4/deploy
conda run --no-capture-output -n rb pytest -q tests/test_hitter_backhand_edge_landing_bias.py
```

关键输出：`14 passed in 0.06s`。

```bash
PYTHONPATH=. conda run --no-capture-output -n rb pytest -q \
  tests/test_hitter_planner_velocity_alignment.py \
  tests/test_hitter_planner_failure_reasons.py
```

关键输出：`18 passed, 2 warnings in 0.97s`。两条 warning 是既有测试导入时的 `DeprecationWarning: invalid escape sequence \\*`，不由本改动产生。

另外执行：

```bash
git diff --check -- deploy/utils/hitter_planner.py
PYTHONPATH=. conda run --no-capture-output -n rb python -m py_compile \
  utils/hitter_planner.py tests/test_hitter_backhand_edge_landing_bias.py
```

两条命令退出码均为 0。

## 文件列表

- 新增：`deploy/tests/test_hitter_backhand_edge_landing_bias.py`
- 修改：`deploy/utils/hitter_planner.py`
- 新增：`.superpowers/sdd/2026-08-15-hitter-backhand-edge-landing-bias/task-1-report.md`

## 自审

- literal 期望覆盖 `-0.50/0.30/0.40/0.50/0.70` 与 `0/-0.05/-0.10`；0.48 s 全计划终点误差限制为 `2e-5 m`。
- 禁用默认值 `0.0` 与未提供新参数的 planner 输出一致。
- 非有限/负值、`full <= start`、`full > table_width/2`、最终 Y 超出桌面全部有行为测试。
- 强制 forehand 会在 selected backhand-Y 位置保持未偏置落点，证明类型在速度规划前已透传。
- 未运行 git add 或 commit。

## 顾虑

当前工作树已经删除 `deploy/tests/__init__.py`，使既有测试按 brief 命令直接运行时无法导入 deploy 包；新测试为保证 brief 命令可执行而局部设置路径，既有回归用命令环境 `PYTHONPATH=.` 验证。另为避免破坏已有只实现两参 `plan()` 的测试替身，`HitterSystemPlanner` 仅向 `StrikePlanner`（含其子类）透传新关键字；真实 planner 路径已由新增测试覆盖。
