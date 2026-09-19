# Task 2 实施报告：生产配置、factory 与旧策略偏置禁用

## 完成内容

- 生产 YAML 将 `policy.real_world_backhand_racket_velocity_y_decrement_mps` 设为 `0.0`。
- 在 `motion.ball_planner.desired_landing_point_w` 后新增生产反手边缘落点参数：`0.30`、`0.50`、`0.10`。
- `build_hitter_system_planner()` 将三项配置传给 `StrikePlanner`；配置缺失时保持 Task 1 构造器同值默认数 `0.30`、`0.50`、`0.0`。
- factory 参考构造器/等值断言覆盖三项字段；新增生产 YAML 字面值断言。
- 更新策略 decrement 的生产期望为零，并通过真实后端 adapter 断言反手输入 `[1.0, -0.2, 0.3]`（float32 表示）不被改写。

## RED

brief 原命令：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4/deploy
conda run --no-capture-output -n rb pytest -q \
  tests/test_hitter_runtime_factory.py \
  tests/test_hitter_forehand_policy_vx_offset.py
```

该工作树中已有删除的 `deploy/tests/__init__.py`，原命令在收集阶段报 `ModuleNotFoundError: No module named 'simulator'` / `envs`，与 Task 2 无关。因此未修改该既有脏改动，而只为测试进程补充当前 deploy 根目录：

```bash
PYTHONPATH=. conda run --no-capture-output -n rb pytest -q \
  tests/test_hitter_runtime_factory.py \
  tests/test_hitter_forehand_policy_vx_offset.py
```

关键 RED 输出：`3 failed, 39 passed`。失败分别证明：factory 解析到 `(0.3, 0.5, 0.0)` 而非 `(0.3, 0.5, 0.1)`；YAML 仍为旧策略 decrement `0.3`；真实反手 adapter 将 Y 从 `-0.2` 改为 `-0.5`。

## GREEN

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4/deploy
PYTHONPATH=. conda run --no-capture-output -n rb pytest -q \
  tests/test_hitter_backhand_edge_landing_bias.py \
  tests/test_hitter_runtime_factory.py \
  tests/test_hitter_forehand_policy_vx_offset.py
```

关键输出：`56 passed, 2 warnings in 0.99s`。两条 warning 均为已有的 `DeprecationWarning: invalid escape sequence \\*`。

## 配置解析核验

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4/deploy
conda run --no-capture-output -n rb python - <<'PY'
from omegaconf import OmegaConf
cfg = OmegaConf.load('config/mimic/hitter.yaml')
print(cfg.policy.real_world_backhand_racket_velocity_y_decrement_mps)
print(cfg.motion.ball_planner.backhand_edge_landing_start_y_w_m)
print(cfg.motion.ball_planner.backhand_edge_landing_full_y_w_m)
print(cfg.motion.ball_planner.backhand_edge_landing_y_decrement_m)
PY
```

实际输出四行：`0.0`、`0.3`、`0.5`、`0.1`。

## 文件列表

- `deploy/config/mimic/hitter.yaml`：Task 2 的四个生产数值。
- `deploy/utils/hitter_runtime_factory.py`：三项 factory 转发与缺省值。
- `deploy/tests/test_hitter_runtime_factory.py`：参考构造器、等值检查与生产配置断言。
- `deploy/tests/test_hitter_forehand_policy_vx_offset.py`：零 decrement 期望和真实 adapter 不变性断言。
- `.superpowers/sdd/2026-08-15-hitter-backhand-edge-landing-bias/task-2-report.md`：本报告。

## 自审与顾虑

- 已运行 `git diff --check --` 并限定 brief 列出的六条路径；无空白错误。未运行 `git add`、未创建 commit。
- 限定 diff 仍包含用户先前在 YAML、factory、runtime-factory test、Task 1 planner/test 中的脏改动；本任务只写入上述 Task 2 hunk，未回退或重排它们。
- `deploy/tests/test_hitter_forehand_policy_vx_offset.py` 与 `deploy/tests/test_hitter_backhand_edge_landing_bias.py` 在当前工作树为未跟踪文件，故普通 `git diff --stat` 不会列出它们；pytest 已执行其实际内容。
- 原 brief pytest 命令因既有 `tests/__init__.py` 删除而无法收集；GREEN 使用了非持久的 `PYTHONPATH=.`，未更改测试包布局。
