### 任务 4：全量验证与物理参数报告

**文件：**
- 验证：`deploy/tests/test_mujoco_physical_table_tennis.py`
- 验证：`deploy/config/hitter.yaml`
- 验证：`deploy/envs/hitter.py`
- 验证：`deploy/simulator/mujoco.py`
- 验证：`deploy/data/assets/g1/g1_29dof_hitter_racket_table_tennis.xml`

**接口：**
- 输入：最终工作树。
- 输出：无脚本状态覆写、物理接触通过、语法和配置可加载的证据。

- [ ] **步骤 1：运行专用测试**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
conda run --no-capture-output -n rb \
  python -m unittest discover -s tests \
  -p 'test_mujoco_physical_table_tennis.py' -v
```

预期：全部测试为 `OK`。

- [ ] **步骤 2：运行语法与配置组合检查**

```bash
conda run --no-capture-output -n rb \
  python -m py_compile simulator/mujoco.py envs/hitter.py
conda run --no-capture-output -n rb python - <<'PY'
from pathlib import Path
from hydra import compose, initialize_config_dir
from omegaconf import OmegaConf

config_dir = Path("config").resolve()
with initialize_config_dir(version_base=None, config_dir=str(config_dir)):
    cfg = compose(config_name="hitter", overrides=["sim=mujoco"])
table_tennis = OmegaConf.to_container(
    cfg.sim.config.table_tennis, resolve=True
)
assert table_tennis == {
    "enabled": True,
    "ball_body_name": "hitter_ball",
    "ball_joint_name": "hitter_ball_freejoint",
}
print(table_tennis)
PY
```

预期：`py_compile` 无输出，配置脚本打印只含三个键的字典。

- [ ] **步骤 3：确认不存在脚本状态覆写残留**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2
rg -n "analytic_(table|racket)|use_analytic|set_hitter_analytic" \
  deploy/config/hitter.yaml deploy/envs/hitter.py deploy/simulator/mujoco.py
```

预期：无输出。

- [ ] **步骤 4：检查补丁完整性和脏工作区隔离**

```bash
git diff --check
git status --short
```

预期：`git diff --check` 无输出；状态中原有用户修改仍保持，实施代码未被自动提交。

- [ ] **步骤 5：记录标定边界**

最终交付中明确报告：

```text
桌面和球拍均已改为 MuJoCo 真实接触；
当前 solref/solimp 是稳定初值，尚不能声称与真实桌面恢复系数完全等价；
5 ms 下已覆盖 2、4、6 m/s 球拍接触；
若现场速度超出范围出现漏碰，再单独评估 2 ms MuJoCo 子步。
```
