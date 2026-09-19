# 从新拍视频制作 MOSAIC/HITTER G1 动作 NPZ

更新时间：2026-08-06

本文面向第一次处理 HITTER 动作数据的人，说明如何把一段固定机位拍摄的正手或反手全身视频，处理成 MOSAIC/IsaacLab 可以加载的 Unitree G1+球拍动作 NPZ。

本文对应本机项目：

```text
/home/yhl/Desktop/Hitter/MOSAIC-main
```

外部 GMR 项目：

```text
/home/yhl/Desktop/Hitter/GMR-master
```

本文只处理“视频到参考动作 NPZ”。训练 policy、导出 ONNX 和 RobotBridge 部署属于后续步骤。

## 1. 最终目标与完整数据流

最终需要得到下面的目录结构：

```text
<dataset_root>/
├── backhand/
│   └── *.npz
├── forehand/
│   └── *.npz
└── _index/
    ├── *.json
    └── *.csv
```

完整流程是：

```text
原始正手/反手视频
  -> 手动裁剪为一次完整挥拍的 50 FPS 片段
  -> GVHMR 从单目视频恢复 SMPL-X 人体运动
  -> GMR 将人体运动重定向到 Unitree G1
  -> 生成 MOSAIC/HITTER 七字段 NPZ
  -> IsaacLab 全身回放检查姿态
  -> 必要时校正肩、肘、腕关节
  -> 将球拍速度峰值对齐到第 43 帧
  -> 形成最终训练数据目录和 manifest
```

不要把这些步骤合并成一次不可检查的一键处理。每一步完成后都要检查结果。

## 2. 关键约定

当前 HITTER 数据约定为：

| 项目 | 约定 |
|---|---:|
| 帧率 | 50 FPS |
| 总帧数 | 94 |
| 动作时长 | 约 1.88 秒 |
| 击球/挥拍速度峰值帧 | 43 |
| G1 关节数 | 29 |
| 保存的刚体数 | 31 |
| 球拍刚体 | `right_racket_link`，body index 30 |

第 43 帧不是 NPZ 或 MuJoCo 的硬性格式要求，而是当前 HITTER 任务的时间约定。`43 / 50 = 0.86` 秒，与任务的击球倒计时区间约 0.80–0.92 秒相匹配。

NPZ 必须包含且只依赖以下七个核心字段：

```text
fps
joint_pos
joint_vel
body_pos_w
body_quat_w
body_lin_vel_w
body_ang_vel_w
```

## 3. 拍摄要求

推荐按下面方式拍摄：

- 使用固定机位和三脚架，不要跟随人物移动相机。
- 保证头、双手、球拍、双脚在完整挥拍期间始终位于画面中。
- 人物不要离镜头太远，人体应占画面主要高度，但四肢不能被裁掉。
- 使用均匀光照，人物与背景有明显颜色差异。
- 尽量避免宽松衣物、动态模糊和其他人进入画面。
- 球拍不要长期被躯干或另一只手遮挡。
- 每段视频尽量只包含一种动作类别。
- 正手和反手分开拍摄，文件名建议以 `forehand` 或 `backhand` 开头。
- 原视频 50 FPS 或更高更合适；裁剪脚本最终会输出 50 FPS。

重要限制：GVHMR 恢复的是人体 SMPL-X 姿态，它不理解乒乓球拍这个物体。因此，即使视频里拿着球拍，恢复出来的手腕方向仍可能不准。球拍是后续固定安装到 G1 手腕上的，手腕小角度误差会被球拍长度放大。

## 4. 为一次新采集建立独立目录

下面用 `demo_capture_20260806` 作为示例采集名。处理另一批视频时，请换成新的名字，不要复用已有输出目录。

```bash
cd /home/yhl/Desktop/Hitter/MOSAIC-main

mkdir -p data/hitter_captures/demo_capture_20260806
```

把原视频复制进去，并统一命名。例如：

```text
data/hitter_captures/demo_capture_20260806/forehand.mp4
data/hitter_captures/demo_capture_20260806/backhand.mp4
```

原视频只作为输入使用。不要在后续步骤中覆盖或移动原视频。

建议的完整目录流：

```text
data/hitter_captures/demo_capture_20260806/
├── forehand.mp4
├── backhand.mp4
├── manual_clips_review/
│   ├── forehand/
│   ├── backhand/
│   └── _index/
└── gvhmr_results/
    ├── forehand/
    └── backhand/

data/hitter_motions/
├── demo_capture_20260806_g1_npz_v1_pre_align/
├── demo_capture_20260806_g1_npz_v1_peak43_aligned/
├── demo_capture_20260806_g1_npz_v2_pre_align/
└── demo_capture_20260806_g1_npz_v2_peak43_aligned/
```

版本目录的原则是：

- `v1` 保存无手臂校正的基线结果。
- `v2`、`v3` 用于后续不同校正方案。
- `pre_align` 保存峰值对齐前的数据。
- `peak43_aligned` 保存峰值已经位于第 43 帧的数据。
- 不使用 `--overwrite`；需要重做时换一个新版本目录。

## 5. 环境和模型预检查

### 5.1 GVHMR 环境

```bash
conda activate gvhmr
cd /home/yhl/Desktop/Hitter/MOSAIC-main/GVHMR

python -c "import torch; print('torch:', torch.__version__); print('CUDA:', torch.cuda.is_available()); print('GPU:', torch.cuda.get_device_name(0)); print('arch:', torch.cuda.get_arch_list())"
```

本机 RTX 5090 已验证的环境为：

```text
torch 2.7.1+cu128
CUDA available = True
arch 包含 sm_120
```

如果 PyTorch 警告 RTX 5090 的 `sm_120` 不受支持，就不能继续沿用旧的 PyTorch 2.3/CUDA 12.1 环境。

检查 GVHMR 模型：

```bash
find inputs/checkpoints -type f -printf '%p  %s bytes\n' | sort
```

至少应包含：

```text
inputs/checkpoints/body_models/smplx/SMPLX_NEUTRAL.npz
inputs/checkpoints/gvhmr/gvhmr_siga24_release.ckpt
inputs/checkpoints/hmr2/epoch=10-step=25000.ckpt
inputs/checkpoints/vitpose/vitpose-h-multi-coco.pth
inputs/checkpoints/yolo/yolov8x.pt
```

本流程使用 `SMPLX_NEUTRAL.npz`，不要求额外存在 `SMPL_NEUTRAL.pkl`。

### 5.2 GMR 环境

```bash
conda activate gmr

test -f /home/yhl/Desktop/Hitter/GMR-master/assets/body_models/smplx/SMPLX_NEUTRAL.npz
```

如果文件不存在，可从 GVHMR 已下载的模型复制一次：

```bash
mkdir -p /home/yhl/Desktop/Hitter/GMR-master/assets/body_models/smplx

cp -n \
  /home/yhl/Desktop/Hitter/MOSAIC-main/GVHMR/inputs/checkpoints/body_models/smplx/SMPLX_NEUTRAL.npz \
  /home/yhl/Desktop/Hitter/GMR-master/assets/body_models/smplx/SMPLX_NEUTRAL.npz
```

### 5.3 IsaacLab 环境

```bash
conda activate isaaclab
cd /home/yhl/Desktop/Hitter/MOSAIC-main

python -c "import torch; print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0))"
```

IsaacLab 用于最终全身回放和训练，不参与 GVHMR 推理。

## 6. 第一步：手动裁剪正手和反手

启动本地裁剪页面：

```bash
cd /home/yhl/Desktop/Hitter/MOSAIC-main

python tools/hitter_manual_clip_server.py \
  --raw-root data/hitter_captures/demo_capture_20260806 \
  --output-root data/hitter_captures/demo_capture_20260806/manual_clips_review
```

然后打开：

```text
http://127.0.0.1:8765
```

页面操作：

- `I`：设置开始时间。
- `O`：设置结束时间。
- `P`：预览当前区间。
- `S`：保存片段。
- `N`：下一个原视频。
- `B`：上一个原视频。
- `Stroke` 下拉框：明确选择 `forehand` 或 `backhand`。

即使文件名可以自动推断类别，也建议保存前明确检查 `Stroke`。

### 裁剪区间怎么选

一条片段应包含：

1. 击球前的准备和引拍。
2. 完整加速挥拍。
3. 击球或最大速度时刻。
4. 击球后的随挥和短暂稳定。

目标长度约 1.88 秒。理想情况下，让击球或最快挥拍出现在片段开始后的约 0.86 秒，也就是第 43 帧附近。

不要只保留击球附近的几帧。过短片段缺少准备姿态和随挥，后续重采样会严重扭曲速度。

输出示例：

```text
manual_clips_review/forehand/forehand_manual_001_..._50fps.mp4
manual_clips_review/backhand/backhand_manual_002_..._50fps.mp4
manual_clips_review/_index/manual_clips_manifest.csv
manual_clips_review/_index/manual_clips_manifest.json
```

裁剪脚本通过 ffmpeg 输出 50 FPS、无音频的 MP4。

### 裁剪后检查

```bash
find data/hitter_captures/demo_capture_20260806/manual_clips_review \
  -maxdepth 2 -type f -name '*.mp4' -printf '%p\n' | sort
```

对每段视频执行：

```bash
ffprobe -v error \
  -select_streams v:0 \
  -show_entries stream=nb_frames,r_frame_rate,duration \
  -of default=noprint_wrappers=1 \
  data/hitter_captures/demo_capture_20260806/manual_clips_review/forehand/你的正手片段.mp4
```

重点确认：

- `r_frame_rate=50/1`
- 帧数接近 94
- 画面中全身和球拍没有被裁掉
- 每条片段只包含一次主要挥拍

## 7. 第二步：运行 GVHMR

固定机位使用 `-s`。RTX 5090 环境使用 `--skip_render`，避免加载与当前 PyTorch 不匹配的 PyTorch3D CUDA 扩展。

### 7.1 正手

```bash
conda activate gvhmr
cd /home/yhl/Desktop/Hitter/MOSAIC-main/GVHMR

export TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1

python tools/demo/demo_folder.py \
  --folder /home/yhl/Desktop/Hitter/MOSAIC-main/data/hitter_captures/demo_capture_20260806/manual_clips_review/forehand \
  --output_root /home/yhl/Desktop/Hitter/MOSAIC-main/data/hitter_captures/demo_capture_20260806/gvhmr_results/forehand \
  -s \
  --skip_render
```

### 7.2 反手

```bash
python tools/demo/demo_folder.py \
  --folder /home/yhl/Desktop/Hitter/MOSAIC-main/data/hitter_captures/demo_capture_20260806/manual_clips_review/backhand \
  --output_root /home/yhl/Desktop/Hitter/MOSAIC-main/data/hitter_captures/demo_capture_20260806/gvhmr_results/backhand \
  -s \
  --skip_render
```

如果相机确实在移动，不应使用 `-s`，但移动相机增加视觉里程计误差。新数据采集应优先使用固定机位。

### GVHMR 输出检查

```bash
find /home/yhl/Desktop/Hitter/MOSAIC-main/data/hitter_captures/demo_capture_20260806/gvhmr_results \
  -name hmr4d_results.pt -printf '%p  %s bytes\n' | sort
```

每条输入片段必须有一个：

```text
gvhmr_results/<stroke>/<clip_name>/hmr4d_results.pt
```

同时通常会有：

```text
0_input_video.mp4
preprocess/bbx.pt
preprocess/vitpose.pt
preprocess/vit_features.pt
```

如果某一条失败，只重跑对应片段，不要删除其他已完成输出。

`TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD` 的警告表示当前加载的是可信本地 checkpoint，不代表推理失败。

## 8. 第三步：先生成“无手臂校正”的基线 NPZ

第一版必须尽量少加人为修正。先看 GMR 原始重定向结果，再决定是否校正。

注意：当前转换脚本有 MQY 专用默认参考目录，因此处理一批全新视频时，应显式传入所有路径，并用空字符串关闭右臂参考姿态校正：

```text
--arm-pose-reference-root ""
```

运行：

```bash
conda activate gmr
cd /home/yhl/Desktop/Hitter/MOSAIC-main

export TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1

python tools/convert_gvhmr_results_to_hitter_npz.py \
  --input-root data/hitter_captures/demo_capture_20260806/gvhmr_results \
  --clip-root data/hitter_captures/demo_capture_20260806/manual_clips_review \
  --output-root data/hitter_motions/demo_capture_20260806_g1_npz_v1_pre_align \
  --gmr-root /home/yhl/Desktop/Hitter/GMR-master \
  --smplx-root /home/yhl/Desktop/Hitter/GMR-master/assets/body_models \
  --urdf-path source/whole_body_tracking/whole_body_tracking/assets/unitree_description/urdf/g1_hitter_racket/main.urdf \
  --target-fps 50 \
  --target-frames 94 \
  --reference-strike-frame 43 \
  --root-xy-mode zero_start \
  --target-root-height 0.78 \
  --min-body-z 0.025 \
  --lower-body-mode reference_mean \
  --waist-mode keep \
  --reference-root data/hitter_motions/iphone_manual_hitter_g1_npz_strike43_clean \
  --arm-pose-reference-root "" \
  --forehand-right-arm-motion-scale 1.0 \
  --compressed
```

不要加 `--overwrite`。

转换脚本会完成：

- 读取 GVHMR 的 SMPL-X 动作。
- 使用 GMR 重定向到 Unitree G1。
- 把 GMR 关节顺序转换为 Isaac/HITTER 的 29 关节顺序。
- 使用现有参考动作稳定下肢。
- 把平均 pelvis 高度移到约 0.78 m。
- 使用 G1+球拍 URDF 做正向运动学。
- 计算关节速度、刚体线速度和角速度。
- 保存七字段 NPZ 和 `_index/gvhmr_hitter_npz_manifest.*`。

## 9. 第四步：检查 NPZ 结构

```bash
conda activate gmr
cd /home/yhl/Desktop/Hitter/MOSAIC-main

python - <<'PY'
from pathlib import Path
import numpy as np

root = Path("data/hitter_motions/demo_capture_20260806_g1_npz_v1_pre_align")
required = {
    "fps", "joint_pos", "joint_vel", "body_pos_w", "body_quat_w",
    "body_lin_vel_w", "body_ang_vel_w",
}

for path in sorted(root.glob("*/*.npz")):
    data = np.load(path)
    fps = float(data["fps"].reshape(-1)[0])
    speed = np.linalg.norm(
        np.gradient(data["body_pos_w"][:, 30], 1.0 / fps, axis=0),
        axis=1,
    )
    quat_error = np.max(
        np.abs(np.linalg.norm(data["body_quat_w"], axis=-1) - 1.0)
    )
    print(
        path.parent.name,
        path.name,
        "keys_ok=", set(data.files) == required,
        "joint_shape=", data["joint_pos"].shape,
        "body_shape=", data["body_pos_w"].shape,
        "fps=", fps,
        "peak_frame=", int(speed.argmax()),
        "finite=", all(np.isfinite(data[key]).all() for key in data.files),
        "quat_error=", float(quat_error),
    )
PY
```

期望结果：

```text
keys_ok=True
joint_shape=(94, 29)
body_shape=(94, 31, 3)
fps=50.0
finite=True
quat_error 接近 0
```

此时峰值不一定是 43，下一步再对齐。

## 10. 第五步：峰值对齐到第 43 帧

脚本文件名保留了早期的 `backhand` 字样，但当前实现会同时处理正手和反手。

```bash
conda activate gmr
cd /home/yhl/Desktop/Hitter/MOSAIC-main

python tools/realign_hitter_backhand_strike43.py \
  --input-root data/hitter_motions/demo_capture_20260806_g1_npz_v1_pre_align \
  --output-root data/hitter_motions/demo_capture_20260806_g1_npz_v1_peak43_aligned \
  --target-frame 43
```

不要加 `--overwrite`。

该脚本会：

- 从 `body_pos_w[:, 30, :]` 计算球拍速度。
- 找到最大速度帧。
- 将整段动作移动，使峰值落到第 43 帧。
- 在边界处保持端点帧。
- 重新计算所有速度字段。
- 生成 `_index/strike43_alignment_manifest.json` 和 CSV。

对齐只改变时间位置，不改变关节姿态。

## 11. 第六步：在 IsaacLab 中回放

训练前必须回放参考动作。不要只看数值。

`--motion` 必须传动作目录，不能直接传单个 `.npz` 文件。传文件会触发：

```text
AssertionError: Invalid directory path
```

### 反手

当目录中只有一条反手和一条正手时，排序后通常是：

```text
motion_index 0 = backhand
motion_index 1 = forehand
```

```bash
conda activate isaaclab
cd /home/yhl/Desktop/Hitter/MOSAIC-main

python scripts/rsl_rl/play_reference_motion.py \
  --motion data/hitter_motions/demo_capture_20260806_g1_npz_v1_peak43_aligned \
  --motion_index 0 \
  --num_envs 1 \
  --speed 0.5 \
  --camera_body torso_link \
  --camera_eye 2.5 -2.5 1.2 \
  --camera_lookat 0 0 0
```

### 正手

```bash
python scripts/rsl_rl/play_reference_motion.py \
  --motion data/hitter_motions/demo_capture_20260806_g1_npz_v1_peak43_aligned \
  --motion_index 1 \
  --num_envs 1 \
  --speed 0.5 \
  --camera_body torso_link \
  --camera_eye 2.5 -2.5 1.2 \
  --camera_lookat 0 0 0
```

如果目录中动作数量更多，先查看真实排序：

```bash
find data/hitter_motions/demo_capture_20260806_g1_npz_v1_peak43_aligned \
  -type f -name '*.npz' | sort
```

### 回放检查清单

必须从全身视角检查：

- 双脚是否稳定，是否明显穿地或悬空。
- 骨盆和躯干方向是否正确。
- 肩膀、肘部和手腕是否符合人体关节习惯。
- 球拍是否位于合理高度。
- 球拍是否穿过躯干、另一只手或手臂。
- 球拍朝向和牌面是否合理。
- 挥拍是否连续，有没有单帧跳变。
- 循环播放首尾是否出现严重跳跃。

Isaac Sim 日志中的 OmniHub、Iray、IOMMU 或部分渲染警告通常不是动作加载失败。判断是否失败应看日志最后的 traceback。

## 12. 第七步：只有回放异常时才校正

不要一开始就照抄其他人的关节偏移。不同拍摄者、机位和动作的 GMR 误差不同。

### 12.1 先判断问题来自哪里

| 现象 | 优先检查 |
|---|---|
| 整个机器人都过高或过低 | `target-root-height`、脚底高度、root 姿态 |
| 肩膀和手位置正常，但球拍高到头部 | 手腕 pitch/yaw/roll 与球拍固定安装方向 |
| 手和球拍都过高 | 肩膀、肘部和手腕共同影响 |
| 球拍穿过另一只手 | 肩/肘轨迹、手腕角度、左右手间距 |
| 全身方向错误 | GVHMR root 朝向或相机设置 |
| 手臂单帧跳动 | GVHMR 遮挡、跟踪失败或裁剪质量 |

### 12.2 校正原则

1. 每次只做一种明确校正，并写入新的 `v2`、`v3` 目录。
2. 保留 `v1` 无校正基线，便于回退和比较。
3. 肩膀和肘部决定整条手臂位置；手腕主要决定球拍方向和牌面。
4. 不要只优化球拍高度。仅追求位置可能得到数值正确但手腕严重弯折的姿态。
5. 校正必须同时满足：关节角自然、球拍高度合理、牌面合理、与另一只手有足够距离。
6. 手腕角度应尽量接近一条已经人工确认正常的真实参考动作。
7. 自动参考校正会对同类别参考关节角取平均。旋转关节的平均姿态不一定对应任何真实动作，反手尤其需要谨慎。

### 12.3 使用参考动作校正

转换脚本支持：

```text
--arm-pose-reference-root <包含 forehand/backhand 子目录的参考数据集>
--arm-pose-calibration-strokes forehand
```

它会让指定类别第 43 帧的七个右臂关节匹配参考均值，同时保留原动作的帧间变化。

建议先只对正手使用，并回放确认。不要默认对正手和反手同时开启。

### 12.4 手动偏移参数

反手支持以下弧度偏移：

```text
--backhand-right-shoulder-pitch-offset
--backhand-right-shoulder-roll-offset
--backhand-right-shoulder-yaw-offset
--backhand-right-elbow-offset
--backhand-right-wrist-roll-offset
--backhand-right-wrist-pitch-offset
--backhand-right-wrist-yaw-offset
```

调参时先用约 0.05–0.15 rad 的小步长观察，不要一次给手腕增加接近 1 rad 的角度，除非有明确参考姿态和关节范围验证。

正手还支持：

```text
--forehand-right-arm-motion-scale
```

它以第 43 帧为锚点，放大或缩小正手右臂在击球前后的运动范围，但不改变第 43 帧本身的姿态。

### 12.5 校正后的正确顺序

```text
重新转换到 v2_pre_align
  -> 检查结构和峰值
  -> 回放 v2_pre_align 看姿态
  -> 姿态通过后，再对齐到 v2_peak43_aligned
  -> 最后再回放一次
```

如果校正改变了球拍轨迹，最大速度帧也可能改变，因此校正后必须重新做峰值对齐。

## 13. 本次 MQY 数据处理得到的经验

本次数据包含一条正手和一条反手。处理过程中得到以下结论：

- 原始 root、躯干和肩膀高度基本正常。
- 初始球拍过高主要来自手腕姿态误差，而不是 `target-root-height=0.78`。
- 正手使用参考右臂姿态校正后可以接受。
- 反手对多条参考动作的关节角取平均，产生了球拍穿过另一只手的问题。
- 只按球拍高度和避碰距离优化三个手腕关节，又产生了数值正确但手腕姿态怪异的问题。
- 更合理的方法是约束手腕接近一条真实参考，同时由肩膀和肘部承担整条手臂的位置调整。
- 最终选择没有强行统一两个类别的校正版本，而是分别挑选视觉上更可靠的动作。

最终使用：

```text
反手：data/hitter_motions/mqy_g1_npz_peak43_aligned/backhand/
正手：data/hitter_motions/mqy_g1_npz_arm_pose_corrected_v3_peak43_aligned/forehand/
```

合并后的最终目录：

```text
data/hitter_motions/mqy_g1_npz_selected_peak43_aligned
```

该目录已经验证：

```text
motion count = 2
forehand count = 1
backhand count = 1
fps = 50
frames = 94
joint count = 29
body count = 31
peak frame = 43
finite values = true
```

对应 manifest：

```text
data/hitter_motions/mqy_g1_npz_selected_peak43_aligned/_index/selected_motion_manifest.json
data/hitter_motions/mqy_g1_npz_selected_peak43_aligned/_index/selected_motion_manifest.csv
```

## 14. 形成最终数据集

如果不同类别最终选用了不同版本，可以新建一个选择目录。使用复制而不是移动，保留所有来源：

```bash
mkdir -p \
  data/hitter_motions/demo_capture_20260806_g1_npz_selected_peak43_aligned/backhand \
  data/hitter_motions/demo_capture_20260806_g1_npz_selected_peak43_aligned/forehand

cp -n <选中的反手NPZ> \
  data/hitter_motions/demo_capture_20260806_g1_npz_selected_peak43_aligned/backhand/

cp -n <选中的正手NPZ> \
  data/hitter_motions/demo_capture_20260806_g1_npz_selected_peak43_aligned/forehand/
```

尖括号只是说明占位符，不要原样复制到终端；应替换为实际文件路径。

### `_index` 是否必须

MOSAIC 的 `MultiMotionLoader` 会递归查找 `*.npz`，训练和回放并不依赖 `_index`。

但最终数据集强烈建议生成新的 manifest，用来记录：

- 每条动作的类别和文件名。
- 最终文件来自哪个版本。
- FPS、帧数和峰值帧。
- 关节数和刚体数。
- SHA-256。
- 是否含 NaN/Inf。
- 四元数范数误差。

不能直接复制任一来源目录的旧 manifest，因为混合目录里的文件集合和来源已经改变。

## 15. 用最终数据训练 MOSAIC/HITTER

先回放最终选择目录：

```bash
conda activate isaaclab
cd /home/yhl/Desktop/Hitter/MOSAIC-main

python scripts/rsl_rl/play_reference_motion.py \
  --task Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion data/hitter_motions/demo_capture_20260806_g1_npz_selected_peak43_aligned \
  --num_envs 2 \
  --show_all
```

确认正手和反手都正常后再训练：

```bash
python scripts/rsl_rl/train.py \
  --task Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion data/hitter_motions/demo_capture_20260806_g1_npz_selected_peak43_aligned \
  --logger tensorboard \
  --run_name demo_capture_20260806
```

参考动作回放不正常时不要开始训练。训练不会自动修复错误的参考关节姿态。

## 16. 常见错误

### 16.1 RTX 5090 与旧 PyTorch 不兼容

症状：

```text
NVIDIA GeForce RTX 5090 with CUDA capability sm_120 is not compatible
```

处理：使用包含 `sm_120` 的 PyTorch/CUDA 构建。本机已验证 `torch 2.7.1+cu128`。

### 16.2 PyTorch3D undefined symbol

症状：

```text
ImportError: pytorch3d/_C...so: undefined symbol ...
```

原因是 PyTorch3D 扩展与当前 PyTorch ABI 不匹配。数据恢复阶段使用：

```text
--skip_render
```

只保存 `hmr4d_results.pt`，不导入 PyTorch3D CUDA renderer。

### 16.3 `Invalid directory path: ...npz`

`play_reference_motion.py --motion` 实际要求目录。改为传包含 NPZ 的数据集根目录，再用 `--motion_index` 选择动作。

### 16.4 球拍高度正确但手腕很怪

这是“只优化末端位置、没有约束人体关节自然性”的典型结果。回退到上一版本，用真实参考限制手腕角度，再由肩、肘调整手臂位置。

### 16.5 球拍穿过另一只手

不要只检查 `right_racket_link` 的高度。必须同时回放并检查左右手、肘部、躯干的几何关系。自动平均多条旋转参考也可能造成这种问题。

### 16.6 对齐以后姿态变了

峰值对齐脚本只移动帧并重新计算速度，不会修改关节角。如果视觉姿态变化，应确认是否播放了不同版本目录或选择了不同 `motion_index`。

### 16.7 输出目录已经存在

对齐脚本会拒绝覆盖已有目录。不要加 `--overwrite`；创建新的版本目录，保留旧结果。

## 17. 最终验收清单

只有全部满足后，数据才适合进入训练：

- [ ] 原始视频仍完整保留。
- [ ] 正手和反手分类正确。
- [ ] 每条裁剪片段约 1.88 秒、50 FPS、一次完整挥拍。
- [ ] 每条片段都有对应 `hmr4d_results.pt`。
- [ ] 每条 NPZ 含七个必需字段。
- [ ] `joint_pos.shape == (94, 29)`。
- [ ] `body_pos_w.shape == (94, 31, 3)`。
- [ ] 所有数组均无 NaN/Inf。
- [ ] 四元数范数接近 1。
- [ ] 球拍速度峰值位于第 43 帧。
- [ ] 全身回放时脚底、骨盆和躯干正常。
- [ ] 肩、肘和腕姿态自然。
- [ ] 球拍高度和牌面方向合理。
- [ ] 球拍不穿过躯干、另一只手或手臂。
- [ ] 最终选择目录有准确的新 manifest。
- [ ] 最终数据集使用新目录，没有覆盖任何来源数据。

## 18. 最重要的实践原则

```text
先保留无校正基线，再做小步校正；
先看全身视觉，再相信数值指标；
先保证关节自然，再追求球拍位置；
每个版本另存，永远不要覆盖唯一结果。
```
