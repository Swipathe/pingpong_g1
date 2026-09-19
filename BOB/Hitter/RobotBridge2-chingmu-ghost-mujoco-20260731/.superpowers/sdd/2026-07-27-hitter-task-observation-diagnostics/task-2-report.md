# Task 2 实施报告：共享 HITTER runtime factories

## 状态

已按严格 TDD 完成 Task 2，并仅提交 estimator、planner、runtime
settings、lifecycle factory 及 `HitterEnv` / `RealWorld` 的行为等价委托。

## 修改文件

- `deploy/utils/hitter_runtime_factory.py`
  - 新增纯 `BallStateEstimator` factory，逐参数复刻
    `RealWorld._init_ball_state()` 的既有构造。
  - 新增纯 `HitterSystemPlanner` factory，逐参数复刻
    `HitterEnv._build_hitter_ball_planner()` 的 predictor、strike planner
    和 base planner 构造。
  - 新增统一的 forced strike type 解析和校验。
  - 新增冻结的 `HitterRuntimeSettings` 及 resolver，统一 estimator、
    incoming、planner interval、lifecycle、seed、control tick 和 observation
    clip 配置。
  - 新增使用调用方同一个 RNG 的 lifecycle factory。
  - 模块不读取环境变量、时钟或文件。
- `deploy/tests/test_hitter_runtime_factory.py`
  - 保留提取前的 estimator 和 planner 生产构造作为独立 reference
    builder。
  - 从 `config/mimic/hitter.yaml` 和 HITTER control YAML 加载当前配置，
    逐字段比较 estimator、predictor、strike planner 和 base planner。
  - 覆盖当前 runtime 数值、空配置默认值、冻结 settings、12 位 hit
    height 舍入、forced strike type 优先级/校验、同 seed recovery
    sequence 和 `RealWorld` 状态字段初始化。
- `deploy/envs/hitter.py`
  - 初始化时统一解析 runtime settings。
  - planner、forced strike type、incoming 参数和 lifecycle 构造委托
    shared factory/resolver。
  - 继续把同一个 `self.hitter_rng` 传给每次新 lifecycle，不增加随机
    采样，也不改变采样顺序。
- `deploy/simulator/real_world.py`
  - 仅把 `BallStateEstimator(...)` 构造替换为 shared factory。
  - sample rate、时间字段、ready 标志、sample count 和 min-samples
    状态初始化保持原顺序与数值。

## RED 证据

在创建生产 factory 前执行：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python \
  -m unittest discover -s tests \
  -p 'test_hitter_runtime_factory.py' -v
```

结果：exit `1`，失败原因与预期一致：

```text
ModuleNotFoundError: No module named 'utils.hitter_runtime_factory'
Ran 1 test in 0.000s
FAILED (errors=1)
```

## GREEN 与回归证据

提交前 fresh verification：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python \
  -m unittest discover -s tests \
  -p 'test_hitter_runtime_factory.py' -v
```

结果：exit `0`，`Ran 8 tests in 0.009s`，`OK`。

```bash
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python \
  -m unittest discover -s tests \
  -p 'test_real_world_connection_wait.py' -v
```

结果：exit `0`，`Ran 2 tests in 0.003s`，`OK`。

```bash
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python \
  -m unittest discover -s tests \
  -p 'test_hitter_strike_target_logging.py' -v
```

结果：exit `0`，`Ran 6 tests in 0.007s`，`OK`。

附加回归和静态检查：

```text
test_hitter_task_observation.py: Ran 11 tests, OK
git diff --cached --check: exit 0
py_compile(factory, factory test, hitter.py, real_world.py): exit 0
factory environment/time/file static scan: no matches
```

## 提交

```text
81d4225 refactor: share HITTER runtime factories
```

提交仅包含：

```text
deploy/envs/hitter.py
deploy/simulator/real_world.py
deploy/tests/test_hitter_runtime_factory.py
deploy/utils/hitter_runtime_factory.py
```

## Self-review

- estimator factory 的 window/min-samples、table geometry、ball radius 和
  三个 bounce 参数与原生产构造逐项一致。
- planner factory 的 gravity/drag/restitution/dt/table geometry、
  strike target 参数、maximum horizon、future-crossing flag 和 base
  planner 参数与原生产构造逐项一致。
- `maximum_hit_height` 仍使用
  `round(table_height + maximum_hit_height_above_table_m, 12)`。
- forced strike type 保留 planner 配置覆盖 motion 配置的优先级；
  `None`、空字符串、`none`、`null` 仍归一为 `None`。
- YAML settings 固定验证了 estimator `360 Hz`、planner `100 Hz` /
  `0.01 s`、incoming `3` / `0.20 m/s`、waiting/arm/max `0.92 s`、
  minimum arm `0.60 s`、swing `[1.75, 1.95]`、seed `0` 和 control
  tick `0.02 s`。
- 空配置保持生产默认 estimator `300 Hz`、planner `100 Hz`、
  lifecycle `0.92/0.90/0.80/0.92`、incoming `3/0.20`、swing
  `[1.75, 1.95]`、control tick `0.02 s` 和无 observation clip。
- resolver 和 factory 不读取环境变量、时钟或文件；只有测试负责加载
  YAML。
- lifecycle factory 的 sampler 闭包复用传入 RNG；构造 lifecycle
  本身不采样，首次 arm 时才按既有 uniform 顺序采样。
- 没有删除原 `HitterEnv` 的 range helper，避免把无关清理混入本任务。
- 交互式暂存明确拒绝了 `hitter.py` 的 waiting/104D 用户 hunks和
  `real_world.py` 的 connection-wait 用户 hunk。

## Concerns

- 当前工作树在 Task 2 开始前已有大量用户修改、删除和未跟踪文件；
  均未恢复、清理、暂存或提交。
- 指定的 `test_real_world_connection_wait.py` 及其对应
  `RealWorld.__init__` 修复是开始前已有的未跟踪/未暂存用户工作。
  本任务按简报运行该回归并得到 GREEN，但没有把测试或修复纳入
  `81d4225`。
- 当前 `hitter.py` 的 waiting/104D 用户修改仍留在未暂存区。因此和
  Task 1 一样，整个分支的完整 104D observation 行为仍依赖该已记录的
  dirty baseline；Task 2 没有扩大这项既有依赖。
- 除上述既有工作树边界外，没有发现 Task 2 范围内的未解决问题。
