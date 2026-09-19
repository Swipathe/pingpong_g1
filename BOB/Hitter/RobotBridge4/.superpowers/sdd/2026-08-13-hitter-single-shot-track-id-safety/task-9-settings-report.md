# Task 9-B 报告：runtime settings、50 Hz 与 nested Vicon 配置

## Fix round 1

- 基线：`1c61ceb`。
- RED：新增配置类型回归后聚焦集合为 `13 failed, 15 passed`；失败分别证明
  resolver 未先检查 Mapping、timeout 接受 bool/string、RealWorld 把显式
  null/list 吞为默认值，以及 negative seed 被接受。
- 修复：Vicon resolver 先要求 Mapping，并复用 strict finite-positive real-number
  validator；RealWorld 仅在字段缺失时使用 `{}`；hitter seed 要求 exact
  non-negative int 或 None。
- GREEN：当前树两个完整测试文件 `117 passed`，四文件 `py_compile` 通过；从
  staged tree 导出的干净 archive 复测同样 `117 passed` 且 `py_compile` 通过。
- 精确暂存只含四个 fix 文件与本轮 hunks；排除了 factory/test velocity-range
  passthrough，以及 RealWorld connection/R2 用户 hunks。

## 范围

- 生产：`deploy/utils/hitter_runtime_factory.py`
- 测试：`deploy/tests/test_hitter_runtime_factory.py`
- 配置：`deploy/config/mimic/hitter.yaml`
- 未修改 `deploy/config/hitter.yaml` 或 `deploy/config/sim/real_world.yaml`；Vicon
  唯一配置源为 `motion.vicon_consumer`，由既有 Hydra 插值传给 RealWorld。

## RED / GREEN

- RED：新测试先运行，得到 `9 failed, 8 passed`。失败精确来自：新 settings
  字段缺失、默认仍为 100 Hz、nested Vicon YAML 缺失、严格安全校验与 lifecycle
  构造注入缺失。
- GREEN：focused factory suite 为 `18 passed`，两个文件 `py_compile` 通过。
- staged tree 干净导出验证：factory `18 passed`；加 lifecycle 与 completed queue
  为 `122 passed`；RealWorld v2 consumer 为 `66 passed`。

## 实现合同

- 默认 planner 50 Hz；waiting/arm/min/max 为 `0.92/0.92/0.30/0.92`；queue
  64、failure count 3、commit 0.30、position/velocity/deadline override 为
  `0.05/0.75/0.05`。
- estimator/planner/control tick/obs clip 和 incoming speed 使用 finite positive；
  lifecycle timing、override 阈值与 swing range 使用 finite nonnegative，并验证
  `commit <= minimum_arm <= arm <= maximum_policy` 和 waiting 上界。
- count/capacity/decimation 为 exact positive int，拒绝 bool/float/string；seed 为
  exact int 或 None；配置 section 必须是 Mapping。
- lifecycle factory 注入全部 Task 8 参数。
- YAML 增加唯一 nested `motion.vicon_consumer`，planner 改为 50 Hz，并明确
  `waiting_base_target_xy_w` 仅用于 MuJoCo。

## 用户 dirty 保护

- `minimum_arm_time_to_strike_s: 0.30` 与 Task 9 最终目标重叠，纳入提交。
- 精确交互暂存排除了用户既有的 model17500、5 秒首帧、MuJoCo serve、virtual
  plane、base/max height、racket velocity range、fore/back 参数以及 factory/test
  velocity-range passthrough。
- 未暂存任何删除项、真机/PD/网络启动或 `unitree_sdk2/build` 内容。
