# 新握拍组件资产交付包设计

## 目标

生成一个可直接发送给同事的精简压缩包，使仍在使用旧手握球拍资产的 Hitter 项目能够在不覆盖旧资产的前提下安装、切换和验证新的刚性连接件、球拍套与球拍资产。

## 交付范围

压缩包只包含：

- Isaac Lab/MOSAIC 运行新组件所需的兼容 USD、URDF 源文件和三个视觉 STL。
- RobotBridge/MuJoCo 运行新组件所需的新 XML 和三个视觉 STL。
- SolidWorks 完整装配导出的 `assembled_fully.STL`，用于几何对照。
- 使 MOSAIC `robots/g1.py` 支持环境变量资产切换的最小补丁和参考片段。
- 中文部署说明、文件清单与 SHA-256 校验文件。

压缩包不包含：

- 动作 NPZ 数据。
- PyTorch checkpoint、ONNX policy 或训练日志。
- Isaac 转换缓存、测试脚本、`__pycache__` 或临时输出。
- 当前工作区内与新握拍组件无关的修改。

## 目录设计

压缩包根目录命名为 `hitter_new_racket_assets_v1_20260812`，内部结构如下：

```text
hitter_new_racket_assets_v1_20260812/
├── README_部署说明.md
├── MANIFEST.txt
├── SHA256SUMS
├── patches/
│   ├── mosaic_g1_asset_switch.patch
│   └── g1_asset_switch_reference.txt
├── payload/
│   ├── MOSAIC-main/
│   │   └── source/whole_body_tracking/whole_body_tracking/assets/unitree_description/
│   │       ├── meshes/g1_hitter_racket/
│   │       │   ├── connector_visual.stl
│   │       │   ├── racket_holder_visual.stl
│   │       │   └── racket_visual.stl
│   │       ├── urdf/g1_hitter_racket_cad_v1/main.urdf
│   │       └── usd/g1_hitter_racket_cad_v1/
│   │           ├── cad_geometry.usd
│   │           └── main_motion_compatible.usda
│   └── RobotBridge2/
│       └── deploy/data/assets/g1/
│           ├── g1_29dof_hitter_racket_table_tennis_cad_v1.xml
│           └── meshes/
│               ├── connector_visual.stl
│               ├── racket_holder_visual.stl
│               └── racket_visual.stl
└── source_stl/
    └── assembled_fully.STL
```

`payload` 保留项目相对路径。同事可以把 `payload/MOSAIC-main/` 合并到自己的 MOSAIC 根目录，把 `payload/RobotBridge2/` 合并到自己的 RobotBridge2 根目录；项目根目录名称不同不影响使用。

## 资产切换逻辑

### Isaac Lab/MOSAIC

默认仍加载原来的：

```text
assets/unitree_description/usd/g1_hitter_racket/main.usda
```

只有在启动命令前设置以下环境变量时才加载新资产：

```bash
HITTER_G1_HITTER_RACKET_USD_PATH="$MOSAIC_ROOT/source/whole_body_tracking/whole_body_tracking/assets/unitree_description/usd/g1_hitter_racket_cad_v1/main_motion_compatible.usda"
```

`main_motion_compatible.usda` 引用同事项目中已有的旧 31 刚体资产作为基础，只替换 `right_racket_link` 的几何、惯性和固定关节变换，因此保持现有 31-body 动作数据和任务配置所需的刚体拓扑。

### RobotBridge/MuJoCo

默认仍加载旧 XML。只有在 Hydra 命令中显式添加以下参数时才加载新资产：

```text
robot.asset.asset_file=g1_29dof_hitter_racket_table_tennis_cad_v1.xml
```

不修改 RobotBridge 默认 YAML，从而保证省略该参数时仍使用旧手握球拍组件。

## 部署文档内容

中文部署说明必须包含：

1. 安装前提和项目根目录变量示例。
2. 解压与逐项复制位置。
3. `g1.py` 已支持和未支持环境变量两种情况下的检查及最小补丁方法。
4. Isaac 新资产静态/参考动作播放命令。
5. 使用现有 checkpoint 的策略播放命令。
6. 单卡训练与 `torchrun` 多卡训练命令，动作路径和运行参数使用占位变量，不打包动作或模型。
7. MuJoCo 独立查看器命令。
8. RobotBridge 使用现有 ONNX policy 启动新 XML 的命令。
9. 恢复旧资产的方法。
10. 常见错误：错误使用 `playback_slowdown`、缺少环境变量支持、路径不存在、USD 引用缺失以及 MuJoCo 手腕末端外壳不显示。

所有命令以 `$MOSAIC_ROOT`、`$ROBOTBRIDGE_ROOT`、`$MOTION_PATH`、`$CHECKPOINT_PATH` 和 `$ONNX_PATH` 表示同事本机路径，避免写入 YHL 电脑的绝对路径。

## 已知限制

- 当前质量与惯性来自 SolidWorks 均匀密度质量属性，质量为 `0.36624 kg`；正式实物部署前应使用实际材料或称重数据复核。
- 当前 MuJoCo 新 XML 与旧 Hitter 专用 XML 都没有把 `right_wrist_yaw_link.STL` 绑定为可视 `geom`，因此 MuJoCo 中末端 yaw 手腕外壳不显示；其刚体、关节及后续连接组件仍存在。本交付包按当前状态原样交付，不修复该问题。
- `main_motion_compatible.usda` 依赖目标项目已经存在 `usd/g1_hitter_racket/main.usda`；同事既然在使用旧手握球拍版本，应满足该依赖。
- 本交付不保证旧 checkpoint 在新质量和新球拍几何下仍有相同策略效果；新资产训练应生成新的 checkpoint。

## 验证标准

打包前必须完成：

- 三份 STL 在 MOSAIC、RobotBridge 和源目录之间 SHA-256 一致。
- 交付 XML 与当前已安装的新 XML 哈希一致；在包含旧 G1 通用网格的完整 RobotBridge 资产目录中，MuJoCo 3.11 能编译新 XML，并能找到 `right_racket_link`、三个新视觉 geom 和球拍拍面碰撞 geom。精简覆盖包本身不重复携带旧机器人网格。
- USD 兼容层及其 `cad_geometry.usd` 引用存在，且目标旧 `main.usda` 引用保持相对路径。
- 通过 Hydra `--cfg job` 验证 RobotBridge 明确选中新 XML；不使用无效的 `playback_slowdown` 参数。
- 通过 Python 语法检查验证 `g1.py` 参考配置。
- 解压测试后逐项对比 `SHA256SUMS`，确认压缩包没有漏文件或多余临时文件。

最终同时生成 `.tar.gz` 和 `.zip`，方便 Linux 环境直接解压以及跨平台传输。
