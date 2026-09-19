# HITTER 手机视频转 NPZ 脚本包

这个轻量脚本包用于把手机录制的人体击球视频处理成 MOSAIC/HITTER 可读取的 Unitree G1 `.npz` 动作文件。

完整数据流：

```text
手机视频
  → 人工裁剪、标注 forehand/backhand
  → GVHMR 人体动作恢复
  → hmr4d_results.pt
  → GMR Unitree G1 动作重定向
  → HITTER NPZ
```

## 包里有什么

```text
hitter_phone_video_to_npz_scripts/
├── README.md
├── config.example.env
├── check_environment.sh
├── run_pipeline.sh
├── PACKAGE_MANIFEST.txt
├── SHA256SUMS
└── scripts/
    ├── hitter_manual_clip_server.py
    ├── convert_gvhmr_results_to_hitter_npz.py
    └── build_bvh1h_npz.py
```

- `hitter_manual_clip_server.py`：浏览器中的手机视频裁剪和正手、反手标注工具。
- `convert_gvhmr_results_to_hitter_npz.py`：读取 GVHMR 的 `hmr4d_results.pt`，通过 GMR 重定向并写出 HITTER NPZ。
- `build_bvh1h_npz.py`：转换器依赖的速度、四元数、前向运动学和 NPZ 保存函数。
- `check_environment.sh`：在运行推理前检查路径、模型和 Python 依赖。
- `run_pipeline.sh`：批量执行 GVHMR 和 NPZ 转换。

本包不包含手机原视频、历史动作、输出结果、虚拟环境、GVHMR/GMR 仓库和模型权重。

## 外部依赖

需要自行准备：

1. Linux、Bash、`ffmpeg`、`ffprobe`、`tar` 和 `sha256sum`。
2. 可运行的 GVHMR 仓库及其模型。
3. General Motion Retargeting（GMR）仓库。
4. SMPL-X `SMPLX_NEUTRAL.npz`。
5. HITTER Unitree G1 球拍 URDF。
6. 同一个 Python 环境中可以导入：
   - `cv2`
   - `torch`
   - `hydra`
   - `pytorch3d`
   - `numpy`
   - `pybullet`
   - `smplx`
   - `general_motion_retargeting`

脚本不会自动下载或安装这些大型依赖。

## 当前本机环境状态

截至 2026-07-30，本机仍有：

- GVHMR：`/home/sijie/Stage1_recovery_20260408/GVHMR`
- GMR：`/home/sijie/Stage1_recovery_20260408/GMR-master`
- GVHMR 权重和 SMPL-X 模型
- HITTER 球拍 URDF和参考动作

但当前找到的 Python 解释器都没有同时包含上面列出的全部依赖，因此必须先准备或恢复联合 GVHMR/GMR 环境，再运行主流水线。`check_environment.sh` 会准确列出缺项，不会在环境不完整时启动 GPU 推理。

## 1. 解压并校验

```bash
tar -xzf hitter_phone_video_to_npz_scripts.tar.gz
cd hitter_phone_video_to_npz_scripts
sha256sum -c SHA256SUMS
```

校验成功时，每个文件都会显示 `OK`。

## 2. 配置路径

复制示例配置：

```bash
cp config.example.env config.env
```

修改 `config.env`：

```bash
GVHMR_ROOT=/home/sijie/Stage1_recovery_20260408/GVHMR
GMR_ROOT=/home/sijie/Stage1_recovery_20260408/GMR-master
PIPELINE_PYTHON=/absolute/path/to/gvhmr-gmr/bin/python
SMPLX_ROOT=/home/sijie/Stage1_recovery_20260408/GMR-master/assets/body_models
HITTER_URDF=/home/sijie/Hitter/MOSAIC-main/source/whole_body_tracking/whole_body_tracking/assets/unitree_description/urdf/g1_hitter_racket/main.urdf
REFERENCE_ROOT=/home/sijie/Hitter/MOSAIC-main/data/hitter_motions/iphone_manual_hitter_g1_npz_strike43_clean
VIDEO_INPUT_ROOT=/absolute/path/to/labeled_clips
GVHMR_OUTPUT_ROOT=/absolute/path/to/work/gvhmr_outputs
NPZ_OUTPUT_ROOT=/absolute/path/to/output/hitter_npz
```

这些变量的含义：

- `GVHMR_ROOT`：GVHMR 仓库根目录。
- `GMR_ROOT`：GMR 仓库根目录。
- `PIPELINE_PYTHON`：同时具备 GVHMR、GMR 和转换依赖的 Python。
- `SMPLX_ROOT`：包含 `smplx/SMPLX_NEUTRAL.npz` 的目录。
- `HITTER_URDF`：带球拍的 Unitree G1 URDF。
- `REFERENCE_ROOT`：用于稳定下肢关节的已有 HITTER NPZ 参考目录。
- `VIDEO_INPUT_ROOT`：裁剪并完成正反手命名的视频目录。
- `GVHMR_OUTPUT_ROOT`：GVHMR 中间结果目录。
- `NPZ_OUTPUT_ROOT`：最终 HITTER NPZ 目录。

`config.env` 会被 Bash 加载，只能使用你自己创建和信任的配置文件。

## 3. 可选：人工裁剪手机视频

如果原始视频中包含多个击球动作，可启动本地裁剪界面：

```bash
python3 scripts/hitter_manual_clip_server.py \
  --raw-root /absolute/path/to/raw_phone_videos \
  --output-root /absolute/path/to/labeled_clips \
  --host 127.0.0.1 \
  --port 8765
```

然后在本机浏览器打开：

```text
http://127.0.0.1:8765
```

界面会调用 `ffmpeg` 输出 50 FPS 裁剪视频，生成类似文件名：

```text
forehand_manual_001_xxxxxxxx_1.000_2.880_50fps.mp4
backhand_manual_002_xxxxxxxx_4.200_6.080_50fps.mp4
```

## 4. 输入命名要求

每个待处理视频的文件名必须以以下前缀之一开始：

- `forehand_`
- `backhand_`

扩展名支持 `.mp4`、`.mov` 和 `.m4v`，大小写均可；脚本会递归查找 `VIDEO_INPUT_ROOT`。

转换器根据 GVHMR 结果目录名判断正手或反手。没有正确前缀的结果只能归入 `unknown/`，所以主流水线会在推理前直接拒绝命名不正确的视频。

## 5. 运行环境检查

```bash
./check_environment.sh --config ./config.env
```

只有看到：

```text
[PASS] 环境检查通过。
```

才应继续运行。检查失败不会创建输出，也不会启动推理。

## 6. 执行完整流水线

默认启用静态相机模式并跳过预览渲染：

```bash
./run_pipeline.sh --config ./config.env
```

常用选项：

```bash
# 手机在拍摄过程中有明显移动
./run_pipeline.sh --config ./config.env --no-static-cam

# 生成 GVHMR 预览视频
./run_pipeline.sh --config ./config.env --render

# 重跑每个视频的 GVHMR，并覆盖已有 NPZ
./run_pipeline.sh --config ./config.env --overwrite
```

注意：`--overwrite` 会删除 `GVHMR_OUTPUT_ROOT` 下与当前视频同名的中间结果目录，然后重新推理；它不会删除原始视频或整个输出根目录。

## 输出

GVHMR 中间结果：

```text
GVHMR_OUTPUT_ROOT/
└── forehand_example/
    └── hmr4d_results.pt
```

最终动作：

```text
NPZ_OUTPUT_ROOT/
├── forehand/
│   └── forehand_example__unitree_g1.npz
├── backhand/
│   └── backhand_example__unitree_g1.npz
└── _index/
    ├── gvhmr_hitter_npz_manifest.json
    └── gvhmr_hitter_npz_manifest.csv
```

转换默认值：

- 输出 FPS：50
- 输出帧数：94
- 参考击球帧：43
- 根节点水平位置：从零开始
- 平均骨盆高度：0.78 米
- 下肢模式：使用参考动作均值

每个 NPZ 包含：

- `fps`
- `joint_pos`
- `joint_vel`
- `body_pos_w`
- `body_quat_w`
- `body_lin_vel_w`
- `body_ang_vel_w`

## 单独运行转换器

如果已经有 GVHMR 输出，可以跳过视频推理：

```bash
"$PIPELINE_PYTHON" scripts/convert_gvhmr_results_to_hitter_npz.py \
  --input-root "$GVHMR_OUTPUT_ROOT" \
  --output-root "$NPZ_OUTPUT_ROOT" \
  --gmr-root "$GMR_ROOT" \
  --smplx-root "$SMPLX_ROOT" \
  --urdf-path "$HITTER_URDF" \
  --reference-root "$REFERENCE_ROOT" \
  --compressed
```

转换器会递归查找 `hmr4d_results.pt`。已有 NPZ 默认跳过；确实要覆盖时添加 `--overwrite`。

## 常见问题

### `ModuleNotFoundError`

`PIPELINE_PYTHON` 不是完整的联合环境。用该解释器逐个测试 README 外部依赖列表中的模块，并补齐缺失包。

### 找不到 `SMPLX_NEUTRAL.npz`

确认以下文件存在：

```text
$SMPLX_ROOT/smplx/SMPLX_NEUTRAL.npz
```

### 视频被判定为命名错误

将文件名改为 `forehand_...` 或 `backhand_...`。只放在同名目录下不够，脚本检查的是视频文件名。

### 没有生成 `hmr4d_results.pt`

查看 GVHMR 终端输出，并确认模型权重、GPU、视频编码和人体检测正常。流水线会在该文件缺失时停止，不会继续生成伪 NPZ。

### 已有结果会不会被删除

默认不会。GVHMR 和 NPZ 结果会被跳过。只有显式使用 `--overwrite` 时，当前视频对应的 GVHMR 中间目录和已有 NPZ 才会重建。

### 裁剪界面能打开但保存失败

检查 `ffmpeg` 和 `ffprobe` 是否在 `PATH` 中，并确认输出目录可写。

## 完整性与来源

`PACKAGE_MANIFEST.txt` 是包内文件清单，`SHA256SUMS` 用于确认文件未损坏。`scripts/` 中的三个 Python 文件是本机现有 HITTER 处理脚本的原样快照。
