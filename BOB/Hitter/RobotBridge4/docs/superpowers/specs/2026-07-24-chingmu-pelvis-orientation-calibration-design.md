# ChingMu 刚体到 MuJoCo Pelvis 的旋转外参标定设计

## 1. 目标

让真机 HITTER 链路发布的 pelvis 朝向与 MuJoCo、训练环境使用的
pelvis 坐标系一致，同时保持位置直接使用 ChingMu `G2Pelvis` 刚体
原点。

本设计解决以下问题：

- AvatarPro 中 `G2Pelvis` 刚体坐标轴与 MuJoCo pelvis 坐标轴存在固定
  roll、pitch、yaw 偏差。
- 该固定偏差会直接改变 observation 中的 `base_forward_xy`。
- 当前 bridge 已经发布桌面世界系中的绝对刚体朝向，但仍隐含假设
  “ChingMu 刚体系与 MuJoCo pelvis 系完全共轴”。
- 外参应只标定一次并保存，之后跨进程启动复用，不能依赖每次启动的
  第一帧。

## 2. 已确认的范围

### 2.1 包含

- 标定并保存 ChingMu 刚体系到 MuJoCo pelvis 系的固定三维旋转外参。
- 标定时使用已经加载的桌面世界坐标系。
- 运行时对实时刚体旋转右乘该固定外参。
- 发布位置严格等于转换到桌面世界系后的 ChingMu 刚体原点。
- 删除当前 `pelvis_offset_heading_m` 位置补偿。
- 保存采样数量、稳定性指标和明确的坐标变换约定。
- 标定文件缺失、格式错误或标定不稳定时明确失败。

### 2.2 不包含

- 不标定刚体原点到 MuJoCo pelvis 原点的平移。
- 不再使用现有 `[0.003145, 0.044074, 0.048231] m` 固定偏移。
- 不修改 AvatarPro 中的刚体定义。
- 不修改桌面坐标系标定算法或标定文件。
- 不修改球追踪、LCM 消息结构、planner、policy、MuJoCo 或训练代码。
- 不恢复启动首帧 yaw 归零。
- 不修改 `real_world.py` 的本体 IMU heading reset；HITTER 的
  `base_forward_xy` 使用 `root_quat_world`，不经过该 reset。

位置误差是本设计明确接受的取舍。发布的 pelvis 位置在语义上实际是
ChingMu `G2Pelvis` 刚体原点，而发布的朝向是经过外参修正后的 MuJoCo
pelvis 朝向。

## 3. 坐标系与数学约定

定义：

- \(W\)：RobotBridge 桌面世界系。
- \(R\)：AvatarPro/ChingMu 的 `G2Pelvis` 刚体系。
- \(P\)：MuJoCo 与训练环境的 pelvis 系。
- \(R_{WR}\)：把刚体系向量变换到桌面世界系的实时旋转。
- \(R_{RP}\)：把 pelvis 系向量变换到刚体系的固定旋转外参。
- \(R_{WP}\)：发布给下游的 pelvis 世界旋转。
- \(p_{WR}\)：ChingMu 刚体原点在桌面世界系中的位置。
- \(p_{\text{publish}}\)：发布给下游的位置。

运行时唯一允许的组合公式为：

```text
R_world_pelvis = R_world_rigid @ R_rigid_from_pelvis
p_publish = p_world_rigid
```

即：

\[
R_{WP}=R_{WR}R_{RP}
\]

\[
p_{\text{publish}}=p_{WR}
\]

外参命名必须写出方向，禁止使用含糊的
`rigid_to_pelvis_rotation`。代码和 JSON 统一使用
`rotation_rigid_from_pelvis` 或
`quaternion_rigid_from_pelvis_xyzw`。

## 4. 标定姿态与算法

### 4.1 现场姿态

标定前先加载已有桌面标定。让真实 pelvis 坐标轴满足：

- pelvis `+X` 指向桌面世界 `+X`，也就是机器人正前方指向桌内。
- pelvis `+Y` 指向桌面世界 `+Y`，也就是机器人左侧。
- pelvis `+Z` 指向桌面世界 `+Z`，也就是竖直向上。

应使用 pelvis 上可重复辨认的机械面或水平仪检查 roll、pitch。只让机器
人视觉上“正对桌子”只能可靠约束 yaw。

### 4.2 多帧采集

标定模式连续采集默认 `2.0 s`：

1. 丢弃刚体位置或四元数缺失、非有限、四元数模长过小的帧。
2. 使用当前桌面变换把每个有效刚体姿态转换为
   \((p_{WR,i}, R_{WR,i})\)。
3. 使用 `scipy.spatial.transform.Rotation.mean()` 在 SO(3) 上计算
   平均旋转 \(\bar R_{WR}\)，不得平均欧拉角。
4. 计算每帧相对平均旋转的测地角，并计算角度 RMS。
5. 计算刚体位置相对平均位置的三维距离 RMS，用于判断机器人是否静止。
6. 只有质量门限全部通过后才能生成和保存外参。

终审补充的刚体姿态 freshness 约束如下：

- 每个候选姿态必须带有刚体姿态自身的 SDK 时间
  `MocapFrame.body_pose_source_time_s`，不能用外层 marker frame 的
  `source_time_s` 冒充刚体姿态时间。
- 只接受有限且相对上一个已接受样本严格递增的刚体姿态时间；重复或回退
  的时间代表重复 cache/陈旧姿态，不计入有效样本。
- 刚体姿态时间与所属外层 frame 时间的绝对差不得超过 `0.1 s`。
- 样本数、位置/旋转质量和 coverage 都只基于通过上述 freshness 门限的
  样本；`source_duration_s` 使用刚体姿态自身时间的首尾差。
- freshness 不得通过比较位置或四元数数值是否变化来推断；静止标定时，
  不同时间的正确姿态本来就可以数值相同。

因为标定姿态期望 \(R_{WP}^{*}=I\)，所以：

\[
R_{RP}=\bar R_{WR}^{T}
\]

四元数保存为 `xyzw`，必须归一化；为获得稳定序列化结果，保存前统一
选择 `w >= 0` 的等价符号。

### 4.3 初始质量门限

- 采集时长必须大于零。
- 有效样本不少于 30 帧。
- 有效帧覆盖的 source time 不少于 `1.0 s`。
- 刚体位置 RMS 不超过 `0.002 m`。
- 刚体旋转测地 RMS 不超过 `0.3 deg`。
- 平均旋转矩阵必须有限、正交且行列式接近 `+1`。

任一条件不满足时，不创建也不覆盖标定文件。

这里的“有效样本”明确指 finite、fresh、唯一且按刚体姿态 SDK 时间严格
递增的样本；“source time 覆盖”明确指
`body_pose_source_time_s` 的覆盖，不是 marker frame 时间覆盖。

## 5. 软件结构

所有功能集中在：

```text
deploy/mocap_bridge/chingmu_table_lcm_bridge.py
```

终审批准一个窄范围 SDK 例外：

```text
deploy/mocap_bridge/chingmu_sdk_client.py
```

该例外只负责时间来源和传递，不承载标定数学：

- `MocapFrame` 末尾增加默认 `None` 的
  `body_pose_source_time_s`，保持旧构造兼容。
- callback 根刚体姿态使用该 root report 自身的 `msg_time`。
- `CMTrackerExternTC` 的 `Timeval` 与 position/quaternion 一起进入
  polling cache，并在注入 frame 时一并传播。
- polling cache 冷启动时允许一次同步 poll fallback；后台 polling
  仍是常规运行路径。

新增一个不可变数据结构：

```text
PelvisOrientationCalibration
```

职责：

- 保存 `rotation_rigid_from_pelvis`。
- 保存标定质量元数据。
- 不包含任何平移字段。

新增独立函数，分别负责：

- 从 ChingMu 多帧数据求旋转外参。
- 保存标定 JSON。
- 加载并严格校验标定 JSON。
- 把实时 `R_world_rigid` 与固定外参组合。

`ChingMuTableLcmBridge` 构造时必须显式接收
`PelvisOrientationCalibration`。`process_frame()` 中：

```text
body_pose_to_table_world()
  -> p_world_rigid, R_world_rigid
  -> p_publish = p_world_rigid
  -> R_world_pelvis = R_world_rigid @ R_rigid_from_pelvis
  -> transformation_t
```

球和桌面的消息路径保持不变。刚体无效帧从 valid 到 invalid 的一次性
转换消息行为保持不变。

## 6. 标定文件

默认建议路径：

```text
deploy/mocap_bridge/calibrations/chingmu_pelvis_orientation_latest.json
```

格式：

```json
{
  "format": "robotbridge2_chingmu_pelvis_orientation_v1",
  "base_subject": "G2Pelvis",
  "quaternion_convention": "xyzw",
  "rotation_convention": "R_world_pelvis = R_world_rigid @ R_rigid_from_pelvis",
  "quaternion_rigid_from_pelvis_xyzw": [0.0, 0.0, 0.0, 1.0],
  "sample_count": 120,
  "source_duration_s": 2.0,
  "position_rms_m": 0.0004,
  "angular_rms_deg": 0.08
}
```

保存过程应先完成全部计算和校验，再原子替换目标文件，避免失败标定
破坏已有可用文件。

加载时必须校验：

- `format` 完全匹配。
- `base_subject` 与当前配置一致。
- 旋转约定字符串完全匹配。
- 四元数恰好四个有限数值且模长有效。
- 质量元数据存在、有限且满足门限。

不允许把损坏或不匹配的文件静默退化成单位旋转。

## 7. 命令行行为

新增参数：

```text
--pelvis-orientation-calib PATH
--save-pelvis-orientation-calib PATH
--pelvis-calib-sec 2.0
```

### 7.1 一次性标定模式

必须提供：

```text
--table-calib EXISTING_TABLE_JSON
--save-pelvis-orientation-calib OUTPUT_JSON
```

流程：

1. 加载桌面标定。
2. 提示操作者确认 pelvis 三轴已经与桌面世界系对齐。
3. 采集并校验多帧刚体姿态。
4. 打印求得的修正四元数与质量指标。
5. 原子保存标定文件。
6. 成功后退出，不进入 LCM 发布循环。

标定模式禁止同时 `--publish`，避免在操作者摆正机器人期间向策略发布
可能错误的姿态。

### 7.2 正常运行模式

正常 bridge 运行必须提供：

```text
--pelvis-orientation-calib SAVED_JSON
```

程序必须在创建 LCM publisher 和进入主循环前完成加载校验。路径缺失、
文件不存在或校验失败时直接退出并报告原因，不使用单位旋转兜底。

## 8. 下游数据链路

修正后的链路为：

```text
ChingMu SDK rigid pose
  -> 现有桌面世界系变换
  -> 保留刚体原点位置
  -> 右乘一次性 pelvis 旋转外参
  -> vicon_state_data / G2Pelvis
  -> real_world.py root_trans_world / root_quat_world
  -> hitter.py base_forward_xy 与世界系目标变换
```

下游不需要新增特例：

- `real_world.py` 继续直接接收 `pos_vicon` 和 `quat_vicon`。
- `hitter.py` 继续用 `root_quat_world` 的局部 `+X` 计算
  `base_forward_xy`。
- MuJoCo 继续直接使用 free root 的绝对 pelvis pose。

## 9. 错误处理

- 无有效刚体帧：标定失败，不写文件。
- 有效样本或 source-time 覆盖不足：标定失败。
- 机器人在采集期间移动：根据位置或旋转 RMS 拒绝保存。
- 输入桌面标定缺失或无效：在 pelvis 标定前失败。
- JSON 字段缺失、非有限、四元数非法或 subject 不匹配：正常运行失败。
- 实时运行中刚体暂时丢失：保留现有 invalid transition 语义。
- 已有目标标定文件只在新标定完全成功后才被替换。

所有错误信息必须指出具体文件、字段或超出的质量指标，方便现场判断。

## 10. 测试设计

当前工作树中旧 `deploy/mocap_bridge/tests` 已被删除，实施时只新增本功能
所需的聚焦测试，不恢复或覆盖用户删除的其他测试文件。

自动化测试至少覆盖：

1. 使用合成刚体旋转恢复已知 `R_rigid_from_pelvis`。
2. 含 roll、pitch、yaw 的非交换旋转，锁定右乘顺序。
3. 四元数符号混合时仍得到相同 SO(3) 平均结果。
4. 发布位置与 `p_world_rigid` 完全一致，不再添加旧 offset。
5. 标定姿态输出接近单位 pelvis 旋转。
6. 标定后机器人左转 `90 deg`，发布 pelvis `+X` 接近世界 `+Y`。
7. 保存和加载 JSON round trip。
8. 缺失、损坏、错误 subject、非法四元数和不合格质量元数据均被拒绝。
9. 样本不足、source-time 覆盖不足、位置不稳定和旋转不稳定时不写文件。
10. 第一帧不是零 yaw 时仍输出校正后的绝对 yaw，不做启动归零。
11. invalid 根姿态消息的既有转换行为不变。

## 11. 现场验收

完成一次真实标定后：

- 标定姿态下发布的 pelvis roll、pitch、yaw 各自误差不超过约 `1 deg`。
- `base_forward_xy` 与 `[1, 0]` 的夹角不超过约 `1 deg`。
- 原地左转约 `90 deg` 后，`base_forward_xy` 接近 `[0, 1]`。
- 原地右转约 `90 deg` 后，`base_forward_xy` 接近 `[0, -1]`。
- bridge 发布的位置等于 ChingMu 刚体原点经过桌面变换后的坐标。
- 使用相同 JSON 重启 bridge，位置和朝向语义保持一致。
- bridge 启动时机器人朝向不同，不会重新定义 yaw 零点。

四元数 \(q\) 与 \(-q\) 表示同一旋转，验收应比较旋转或轴向，不直接比较
四元数符号。

## 12. 重新标定条件

以下情况需要重新生成 pelvis 旋转标定：

- AvatarPro 重新建立或重新定向 `G2Pelvis` 刚体。
- 刚体 marker 布局或安装关系改变。
- ChingMu SDK 输出的刚体坐标轴约定改变。

普通 bridge 重启、机器人移动、桌面重新摆放不要求重做该外参。桌面坐标
系改变时只需重新做桌面标定；pelvis 外参描述的是刚体与机器人之间的固定
安装关系。

## 13. 实施与现场标定的边界

代码实现和合成数据测试可以离线完成，但真实
`chingmu_pelvis_orientation_latest.json` 不能由代码仓库预先伪造。它必须
在机器人 pelvis 三轴实际对齐桌面世界系后，通过一次现场采集生成。

因此交付分为两步：

1. 完成标定、加载、运行时旋转修正和自动化测试代码。
2. 操作者摆正机器人，运行一次标定命令；检查质量指标后，再用生成的 JSON
   启动正常 bridge。

在第二步完成前，不得把单位四元数占位文件当作真实标定结果，也不得宣称
真机 `base_forward_xy` 已经完成对齐。
