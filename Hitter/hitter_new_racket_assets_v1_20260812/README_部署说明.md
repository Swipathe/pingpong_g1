# Hitter 新刚性握拍组件部署说明

## 1. 交付内容与目的

本包用于把原来的“机器人手握球拍”资产切换为：

```text
right_wrist_yaw_link
└── 刚性固定的 right_racket_link
    ├── 连接件 connector
    ├── 球拍套 racket holder
    └── 球拍 racket
```

连接件、球拍套和球拍视为一个刚性总成。旧 Isaac USD 和旧 MuJoCo XML不会被覆盖；是否使用新组件由启动命令显式控制。

本包不包含动作 NPZ、训练 checkpoint、ONNX policy、训练日志或 Python/Conda 环境。

本包是覆盖安装包，不是脱离原项目运行的完整机器人资产包。新 MuJoCo XML 仍会使用同事原项目 `deploy/data/assets/g1/meshes/` 中已有的 G1 身体网格，因此必须先合并到完整 RobotBridge 项目，再由 MuJoCo 打开。

## 2. 主要参数

- `right_wrist_yaw_link → right_racket_link` 平移：`[0.22279, 0.00685, -0.00291] m`。
- 旋转：绕 X 轴 180°；MuJoCo 四元数 `quat="0 1 0 0"`。
- 总成质量：`0.36624 kg`。
- 质心（相对于 `right_racket_link_frame`）：`[-0.04789, 0.00685, -0.00083] m`。
- 质量和惯性来自 SolidWorks 均匀密度质量属性，实物部署前应复核实际材料或称重结果。

## 3. 设置本机路径

解压后进入本包根目录，然后把下面路径改成同事电脑上的实际目录：

```bash
export PACKAGE_ROOT="$(pwd -P)"
export MOSAIC_ROOT=/path/to/MOSAIC-main
export ROBOTBRIDGE_ROOT=/path/to/RobotBridge2
export MOTION_PATH=/path/to/motion_directory
export CHECKPOINT_PATH=/path/to/model.pt
export ONNX_PATH=/path/to/policy.onnx
```

要求：

- `$MOSAIC_ROOT` 是包含 `scripts/rsl_rl/` 和 `source/whole_body_tracking/` 的目录。
- `$ROBOTBRIDGE_ROOT` 是包含 `deploy/run.py` 的目录。
- `$MOTION_PATH` 应指向动作目录，而不是单个 `.npz` 文件。若只看反手，可指向包含反手 NPZ 的 `backhand/` 目录。
- `$CHECKPOINT_PATH` 和 `$ONNX_PATH` 使用同事已有文件；它们不在本包中。

先检查：

```bash
test -d "$MOSAIC_ROOT/source/whole_body_tracking"
test -f "$MOSAIC_ROOT/source/whole_body_tracking/whole_body_tracking/assets/unitree_description/usd/g1_hitter_racket/main.usda"
test -f "$ROBOTBRIDGE_ROOT/deploy/run.py"
```

## 4. 安装新资产

在交付包根目录执行：

```bash
cp -a "$PACKAGE_ROOT/payload/MOSAIC-main/." "$MOSAIC_ROOT/"
cp -a "$PACKAGE_ROOT/payload/RobotBridge2/." "$ROBOTBRIDGE_ROOT/"
```

这些命令新增 `g1_hitter_racket_cad_v1` 和新 MuJoCo XML，不替换旧 `g1_hitter_racket/main.usda` 或旧 `g1_29dof_hitter_racket_table_tennis.xml`。

安装后检查：

```bash
test -f "$MOSAIC_ROOT/source/whole_body_tracking/whole_body_tracking/assets/unitree_description/usd/g1_hitter_racket_cad_v1/main_motion_compatible.usda"
test -f "$MOSAIC_ROOT/source/whole_body_tracking/whole_body_tracking/assets/unitree_description/usd/g1_hitter_racket_cad_v1/cad_geometry.usd"
test -f "$ROBOTBRIDGE_ROOT/deploy/data/assets/g1/g1_29dof_hitter_racket_table_tennis_cad_v1.xml"
```

## 5. 让 MOSAIC 支持命令行切换资产

检查 `g1.py` 是否已经支持环境变量：

```bash
G1_PY="$MOSAIC_ROOT/source/whole_body_tracking/whole_body_tracking/robots/g1.py"
rg -n 'HITTER_G1_HITTER_RACKET_USD_PATH' "$G1_PY"
```

如果能找到该变量，跳过本节剩余操作。

如果找不到，先测试补丁能否适配同事当前版本：

```bash
cd "$MOSAIC_ROOT"
patch -p1 --dry-run < "$PACKAGE_ROOT/patches/mosaic_g1_asset_switch.patch"
```

只有 dry-run 成功才正式应用：

```bash
patch -p1 < "$PACKAGE_ROOT/patches/mosaic_g1_asset_switch.patch"
```

如果 dry-run 失败，不要强制覆盖整个 `g1.py`。参照：

```text
patches/g1_asset_switch_reference.txt
```

手动把最小代码块合并到同事自己的 `g1.py`，然后执行：

```bash
python -m py_compile "$G1_PY"
```

## 6. Isaac Lab：播放参考动作

激活同事原来可正常运行 MOSAIC/Isaac Lab 的环境，然后执行：

```bash
cd "$MOSAIC_ROOT"

HITTER_G1_HITTER_RACKET_USD_PATH="$MOSAIC_ROOT/source/whole_body_tracking/whole_body_tracking/assets/unitree_description/usd/g1_hitter_racket_cad_v1/main_motion_compatible.usda" \
python scripts/rsl_rl/play_reference_motion.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion="$MOTION_PATH" \
  --motion_index=0 \
  --num_envs=1
```

这一步不加载 checkpoint，只显示动作数据中的参考姿态。`main_motion_compatible.usda` 保持原项目所需的 31 刚体拓扑。

如果多 GPU GUI 出现渲染或 PhysX 错误，先限制为单卡：

```bash
cd "$MOSAIC_ROOT"

CUDA_VISIBLE_DEVICES=0 \
HITTER_G1_HITTER_RACKET_USD_PATH="$MOSAIC_ROOT/source/whole_body_tracking/whole_body_tracking/assets/unitree_description/usd/g1_hitter_racket_cad_v1/main_motion_compatible.usda" \
python scripts/rsl_rl/play_reference_motion.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion="$MOTION_PATH" \
  --motion_index=0 \
  --num_envs=1 \
  --no_loop
```

## 7. Isaac Lab：播放已有 checkpoint

```bash
cd "$MOSAIC_ROOT"

CUDA_VISIBLE_DEVICES=0 \
HITTER_G1_HITTER_RACKET_USD_PATH="$MOSAIC_ROOT/source/whole_body_tracking/whole_body_tracking/assets/unitree_description/usd/g1_hitter_racket_cad_v1/main_motion_compatible.usda" \
python scripts/rsl_rl/play.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion="$MOTION_PATH" \
  --resume_student_checkpoint="$CHECKPOINT_PATH" \
  --num_envs=1 \
  --disable_obs_noise \
  --skip_critic
```

旧 checkpoint 是在旧手握拍几何和质量条件下训练的，只适合检查程序链路，不保证新组件下仍有相同控制效果。

## 8. MOSAIC：新组件单卡训练

先用较小环境数做 smoke test：

```bash
cd "$MOSAIC_ROOT"

CUDA_VISIBLE_DEVICES=0 \
HITTER_G1_HITTER_RACKET_USD_PATH="$MOSAIC_ROOT/source/whole_body_tracking/whole_body_tracking/assets/unitree_description/usd/g1_hitter_racket_cad_v1/main_motion_compatible.usda" \
python scripts/rsl_rl/train.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion="$MOTION_PATH" \
  --num_envs=4096 \
  --headless \
  --logger=tensorboard \
  --run_name=new_racket_cad_v1_smoke \
  --max_iterations=20
```

Smoke test 成功后，再根据显存和原项目训练配置提高 `--num_envs` 和 `--max_iterations`。

## 9. MOSAIC：新组件多卡训练

下面以两张 GPU 为例：

```bash
cd "$MOSAIC_ROOT"

CUDA_VISIBLE_DEVICES=0,1 \
HITTER_G1_HITTER_RACKET_USD_PATH="$MOSAIC_ROOT/source/whole_body_tracking/whole_body_tracking/assets/unitree_description/usd/g1_hitter_racket_cad_v1/main_motion_compatible.usda" \
python -m torch.distributed.run \
  --standalone \
  --nnodes=1 \
  --nproc_per_node=2 \
  scripts/rsl_rl/train.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion="$MOTION_PATH" \
  --num_envs=32000 \
  --distributed \
  --headless \
  --logger=tensorboard \
  --run_name=new_racket_cad_v1_2gpu \
  --max_iterations=20400
```

`--num_envs` 在该项目中通常是每个进程/每张卡的环境数，必须根据同事机器显存调整。多卡训练必须使用 `--headless`。

## 10. MuJoCo：只查看新资产

激活安装了 `mujoco` Python 包的环境，然后执行：

```bash
python -m mujoco.viewer \
  --mjcf="$ROBOTBRIDGE_ROOT/deploy/data/assets/g1/g1_29dof_hitter_racket_table_tennis_cad_v1.xml"
```

查看器中可以暂停仿真，再通过鼠标旋转、平移和滚轮缩放检查右手腕连接位置。

## 11. RobotBridge：使用新组件运行 MuJoCo

```bash
cd "$ROBOTBRIDGE_ROOT/deploy"

python run.py \
  --config-name=hitter \
  sim=mujoco \
  robot.asset.asset_file=g1_29dof_hitter_racket_table_tennis_cad_v1.xml \
  mimic.policy.checkpoint="$ONNX_PATH"
```

当前这版 RobotBridge 没有实现 `robot.control.playback_slowdown`。不要在命令中添加该参数；即使使用 Hydra 的 `+` 语法添加，当前代码也不会读取它。

## 12. 切回旧资产

Isaac Lab/MOSAIC：不要设置新 USD 环境变量，或者先执行：

```bash
unset HITTER_G1_HITTER_RACKET_USD_PATH
```

然后使用原来的播放或训练命令，程序会继续加载：

```text
assets/unitree_description/usd/g1_hitter_racket/main.usda
```

RobotBridge/MuJoCo：从命令中删除：

```text
robot.asset.asset_file=g1_29dof_hitter_racket_table_tennis_cad_v1.xml
```

程序会继续使用原配置中的旧 XML。

## 13. 校验交付包

在交付包根目录执行：

```bash
sha256sum -c SHA256SUMS
```

所有文件应显示 `OK`。`MANIFEST.txt` 记录完整文件列表和字节数。

## 14. 已知限制

1. MuJoCo 新 XML 与当前旧 Hitter 专用 XML 都声明了 `right_wrist_yaw_link.STL`，但没有把它绑定成 `right_wrist_yaw_link` 下的可视 `geom`。所以 MuJoCo 里末端 yaw 手腕外壳不显示；刚体、关节、新连接件、拍套和球拍仍然存在。
2. 本包按当前项目状态原样交付，没有修复上述手腕显示问题。
3. `main_motion_compatible.usda` 会通过相对路径引用同事项目已有的 `usd/g1_hitter_racket/main.usda`，不能删除或移动旧资产。
4. 新球拍总成的质量、惯性、碰撞范围和实物差异会影响训练与 Sim2Real。完成真实材料和称重标定后，应同步更新 Isaac USD 与 MuJoCo XML。
5. `assembled_fully.STL` 只用于几何对照；运行时使用三个分开的 STL 和相应 USD/XML。
