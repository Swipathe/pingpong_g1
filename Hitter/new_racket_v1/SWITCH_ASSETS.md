# 旧/新握拍资产切换

## Isaac Lab / MOSAIC

默认加载旧手握球拍资产：

```text
MOSAIC-main/source/whole_body_tracking/whole_body_tracking/assets/unitree_description/usd/g1_hitter_racket/main.usda
```

在原训练或播放命令前添加以下环境变量，可仅对该条命令切换到新 CAD 组件：

```bash
HITTER_G1_HITTER_RACKET_USD_PATH=/home/yhl/Desktop/Hitter/MOSAIC-main/source/whole_body_tracking/whole_body_tracking/assets/unitree_description/usd/g1_hitter_racket_cad_v1/main_motion_compatible.usda \
<原 Isaac Lab/MOSAIC 命令>
```

`main_motion_compatible.usda` 保留旧资产的 31 个刚体及其顺序，只替换新连接件、拍套、球拍、球拍拍面碰撞和惯性参数，因此兼容现有 31-body 动作 NPZ。原始 URDF 转换结果 `main.usd` 保留作几何来源和检查用途，不直接用于现有动作回放或训练。

## RobotBridge / MuJoCo

默认加载旧手握球拍资产：

```text
g1_29dof_hitter_racket_table_tennis.xml
```

在原 RobotBridge 命令末尾添加以下 Hydra 参数，可切换到新 CAD 组件：

```bash
robot.asset.asset_file=g1_29dof_hitter_racket_table_tennis_cad_v1.xml
```

例如：

```bash
python run.py --config-name=hitter sim=mujoco \
  robot.asset.asset_file=g1_29dof_hitter_racket_table_tennis_cad_v1.xml
```

不添加覆盖参数时自动恢复默认旧组件。
