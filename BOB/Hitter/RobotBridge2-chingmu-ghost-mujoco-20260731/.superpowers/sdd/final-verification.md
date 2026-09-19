# 最终验证记录

验证时间：2026-07-22 15:03（Asia/Shanghai）。

## 专用测试

工作目录：`deploy/`

```bash
conda run --no-capture-output -n rb \
  python -m unittest discover -s tests \
  -p 'test_mujoco_physical_table_tennis.py' -v
```

结果：退出码 0，`Ran 10 tests in 0.991s`，`OK`。

覆盖内容：

- 配置与 planner 注入接口清理；
- `mujoco.py` 不含解析式球状态覆写；
- XML 精确球接触拓扑；
- 5 ms 下桌面真实接触与正向反弹；
- 连续三次桌面反弹不增能；
- 球拍 2、4、6 m/s 真实接触与正向法向出球。
- 随机 reset 开关、顺序轨迹候选以及固定 seed 的随机轨迹序列。

## 语法与 Hydra 配置

`py_compile` 对 `simulator/mujoco.py`、`envs/hitter.py` 和专用测试退出码均为 0。

Hydra `config_name=hitter`、`sim=mujoco` 组合结果：

```text
CONFIG_OK table_tennis={'enabled': True, 'ball_body_name': 'hitter_ball', 'ball_joint_name': 'hitter_ball_freejoint'} low_dt=0.005 decimation=4
```

## 编译后 MuJoCo 接触配置

```text
MODEL_CONTACT_OK npair=3 table_solref=0.04/0.10 net=0.004/0.35 racket=0.004/0.35 ball_mask=0/0 line_mask=0/0
```

## 源码与工作树检查

```text
NO_SCRIPTED_BALL_OVERRIDE_STATE
DIFF_CHECK_OK
INDEX_CLEAN
```

说明：工作树仍保留用户原有未提交修改和删除项；`INDEX_CLEAN` 仅表示暂存区为空。
