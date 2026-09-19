# MQY 视频到 HITTER NPZ 流程设计

## 目标

将以下两段原始视频作为 MQY 数据集入口：

- `/home/yhl/Desktop/Hitter/MOSAIC-main/data/hitter_captures/mqy_capture/forehand.mp4`
- `/home/yhl/Desktop/Hitter/MOSAIC-main/data/hitter_captures/mqy_capture/backhand.mp4`

YHL 手动、逐步执行视频裁剪、GVHMR 动作恢复、GMR/机器人重定向与
HITTER NPZ 峰值对齐。每一步使用固定的 MQY 默认目录，也保留命令行参数，
以便显式覆盖默认目录。流程不得自动执行耗时任务。

## 总体数据流

```text
mqy_capture/*.mp4
  -> manual_clips_review/{forehand,backhand}/*.mp4
  -> gvhmr_results/{forehand,backhand}/<clip>/hmr4d_results.pt
  -> mqy_g1_npz_pre_align/{forehand,backhand}/*.npz
  -> mqy_g1_npz_peak43_aligned/{forehand,backhand}/*.npz
```

动作类别由 YHL 在裁剪页面的 `Stroke` 下拉框中手动选择。后续步骤依据
裁剪结果的 `forehand` 或 `backhand` 父目录传播类别，不根据原始文件名猜测。

## 固定目录

### 项目和外部依赖

```text
/home/yhl/Desktop/Hitter/MOSAIC-main
/home/yhl/Desktop/Hitter/GVHMR
/home/yhl/Desktop/Hitter/GMR-master
```

GVHMR 的模型文件按照 GVHMR 官方目录要求放置。GMR 使用的 SMPL-X 文件为：

```text
/home/yhl/Desktop/Hitter/GMR-master/assets/body_models/smplx/SMPLX_NEUTRAL.npz
```

### 捕获数据与中间结果

```text
/home/yhl/Desktop/Hitter/MOSAIC-main/data/hitter_captures/mqy_capture/
├── forehand.mp4
├── backhand.mp4
├── manual_clips_review/
│   ├── forehand/
│   ├── backhand/
│   └── _index/
└── gvhmr_results/
    ├── forehand/
    └── backhand/
```

### 最终动作数据

```text
/home/yhl/Desktop/Hitter/MOSAIC-main/data/hitter_motions/
├── mqy_g1_npz_pre_align/
│   ├── forehand/
│   ├── backhand/
│   └── _index/
└── mqy_g1_npz_peak43_aligned/
    ├── forehand/
    ├── backhand/
    └── _index/
```

## 分步组件

### 第一步：手动裁剪

修改 `MOSAIC-main/tools/hitter_manual_clip_server.py`：

- 默认输入为 `data/hitter_captures/mqy_capture`。
- 只扫描默认输入根目录的 `*.mp4` 原始视频，避免把输出片段再次当成输入。
- 默认输出为 `data/hitter_captures/mqy_capture/manual_clips_review`。
- YHL 必须在页面中明确选择 `forehand` 或 `backhand` 后保存。
- 输出保持 50 FPS，并记录源视频、起止时间、帧数、类别和输出路径。

检查点：

- 页面列出 `forehand.mp4` 与 `backhand.mp4`。
- 保存的片段位于所选类别目录。
- 每个目标动作片段包含一次完整挥拍。
- 清单中的 FPS、帧数和实际输出一致。

### 第二步：GVHMR 动作恢复

在 GVHMR 安装完成后，提供 MQY 文件夹批处理脚本。该脚本：

- 读取 `manual_clips_review/forehand/*.mp4` 和
  `manual_clips_review/backhand/*.mp4`。
- 对固定机位视频调用 GVHMR 的静态相机模式 `-s`。
- 将每个片段的完整 GVHMR 输出放到
  `gvhmr_results/<类别>/<片段名>/`。
- 不覆盖已有且完整的 `hmr4d_results.pt`，除非 YHL 显式传入覆盖选项。

检查点：

- 每个输入片段均有对应的 `hmr4d_results.pt`。
- GVHMR 可视化结果中人物身份稳定、朝向正确、手臂动作没有明显跳变。
- 若某个片段失败，只重跑该片段。

### 第三步：转换为 HITTER NPZ

修改 `MOSAIC-main/tools/convert_gvhmr_results_to_hitter_npz.py` 的默认目录：

- 默认输入：
  `data/hitter_captures/mqy_capture/gvhmr_results`
- 默认裁剪根目录：
  `data/hitter_captures/mqy_capture/manual_clips_review`
- 默认输出：
  `data/hitter_motions/mqy_g1_npz_pre_align`
- 默认 GMR：
  `/home/yhl/Desktop/Hitter/GMR-master`
- 默认 SMPL-X：
  `/home/yhl/Desktop/Hitter/GMR-master/assets/body_models`
- 默认 HITTER URDF 保持项目内的 `g1_hitter_racket/main.urdf`。
- 默认参考动作保持
  `data/hitter_motions/iphone_manual_hitter_g1_npz_strike43_clean`。

转换参数固定匹配现有 HITTER 数据：

```text
target_fps=50
target_frames=94
reference_strike_frame=43
root_xy_mode=zero_start
target_root_height=0.78
min_body_z=0.025
lower_body_mode=reference_mean
waist_mode=keep
backhand_right_wrist_roll_offset=0.55
forehand_right_arm_motion_scale=1.55
compressed=true
```

检查点：

- NPZ 包含 `fps`、`joint_pos`、`joint_vel`、`body_pos_w`、
  `body_quat_w`、`body_lin_vel_w`、`body_ang_vel_w`。
- 帧数为 94，关节数为 29，刚体数为 31。
- `fps=50`，所有数组均为有限值。
- 四元数范数正常，最低刚体高度不低于设定阈值。
- `right_racket_link` 为第 31 个刚体。

### 第四步：峰值对齐

新增面向 MQY 的峰值对齐脚本，不复用“仅处理反手”的旧行为：

- 默认输入为 `data/hitter_motions/mqy_g1_npz_pre_align`。
- 默认输出为 `data/hitter_motions/mqy_g1_npz_peak43_aligned`。
- 同时处理 `forehand` 和 `backhand`。
- 以 `right_racket_link` 的线速度峰值为击球峰值。
- 将峰值移动到第 43 帧，边界使用端点保持。
- 位移完成后重新计算所有速度。
- 为每个文件记录原峰值帧、位移量、对齐后峰值帧和输出路径。

检查点：

- 每个输入 NPZ 恰好对应一个输出 NPZ。
- 正手和反手的球拍速度峰值均位于第 43 帧。
- 对齐后仍保持 94 帧、50 FPS 和原有七个字段。
- 输出数组无 NaN 或 Inf。

## 操作说明

新增一份 MQY 专用说明文档，逐步列出：

1. 安装并准备 GVHMR、GMR 和模型文件。
2. 启动裁剪页面并保存片段。
3. 批量运行 GVHMR。
4. 检查 GVHMR 输出。
5. 转换 HITTER NPZ。
6. 检查预对齐 NPZ。
7. 执行峰值对齐。
8. 检查最终 NPZ。

每一步均包含工作目录、完整命令、预期输出目录和只读检查命令。说明不把
安装、推理、转换或训练合并为一键命令。

## 错误处理与安全边界

- 原始 `forehand.mp4` 和 `backhand.mp4` 只读使用。
- 所有生成结果写入新目录，不覆盖现有 HITTER 数据集。
- 缺少仓库、模型、Python 包或输入文件时立即报出具体路径。
- 默认不覆盖已有输出；需要覆盖时必须由 YHL 显式指定。
- 类别必须由裁剪阶段明确写入目录，后续不得静默猜测类别。
- 不增加旧目录或旧参数的兼容性分支。

## 验收标准

- 四个步骤可以由 YHL 独立运行和检查。
- 所有脚本默认值组成一致的数据流，不需要反复手写绝对路径。
- 命令行参数仍可显式覆盖默认值。
- 至少一个正手片段和一个反手片段能够从裁剪结果映射到最终 HITTER NPZ。
- 最终 NPZ 的结构、帧率、帧数、关节数和刚体数与现有示例一致。
