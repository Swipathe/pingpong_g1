# 最终审查修复报告

时间：2026-07-22（Asia/Shanghai）

## 修复范围

- 在 `Mujoco._init_table_tennis_state()` 的 `table_tennis_enabled` 检查之后恢复 `randomize_ball_on_reset` 与 `ball_random_seed` 初始化。未恢复或新增任何 analytic 球状态覆写。
- 新增真实 MuJoCo `MjModel` / `MjData` 加 `Mujoco.__new__` 的最小 simulator 行为测试，覆盖随机 reset 开关、sequence 连续候选与 fixed-seed random 候选。
- XML 拓扑测试先保留原始 pair 元素并断言数量恰为 3；逐 pair 锁定 `condim`、`friction`、`solref`、`solimp`。

## TDD RED 证据

生产代码修复前执行：

```bash
cd deploy
conda run --no-capture-output -n rb \
  python -m unittest discover -s tests \
  -p 'test_mujoco_physical_table_tennis.py' -v
```

结果：退出码 1，`Ran 10 tests`，`FAILED (failures=3)`。三个新增行为测试均按缺陷原因失败：

- `randomize_hitter_ball` 实际为 `False`；
- sequence 两次 reset 均落到固定 `ball_initial_pos`，未选择第 0/1 个 trajectory；
- random reset 未按 `np.random.default_rng(seed)` 选择 trajectory，而是落到固定初值。

## GREEN 证据

仅恢复两项配置初始化后再次执行同一完整专用 unittest；随后将 seed 用例强化为 `seed=17` 的 12 次期望序列，并用两个真实 simulator 分别复现该序列后再次执行。

最终结果：退出码 0，`Ran 10 tests in 1.047s`，`OK`。fixed-seed 用例逐次对照 `np.random.default_rng(seed)` 的位置、线速度和角速度，并确认两实例序列一致；sequence 连续选择 `left`、`center`。

## 其他验证与自审

- `conda run --no-capture-output -n rb python -m py_compile simulator/mujoco.py tests/test_mujoco_physical_table_tennis.py`：退出码 0。
- `git diff --check -- deploy/simulator/mujoco.py deploy/tests/test_mujoco_physical_table_tennis.py`：退出码 0；新测试文件当前未跟踪，另行检查新增文件尾随空白无匹配。
- 对 `deploy/simulator/mujoco.py` 执行 analytic override 标识 `rg`：退出码 1、无匹配，表示无回归。
- 自审确认生产改动仅为 enabled 分支内两项初始化；测试使用真实 model/data，不以源码字符串断言替代 random reset 行为。
- 只读复审指出单次 random 抽样可能随机假通过；已改为两实例各 12 次完整期望序列并重新跑绿。
- 未执行 `git add`、commit 或 push。

## 后续标定范围

移动/旋转球拍、斜入射、自旋、球网动态以及不同步进相位不在本轮有限工况冒烟与稳定初值声明内；本轮未新增这些场景，也未修改生产接触参数，留待后续物理标定。
