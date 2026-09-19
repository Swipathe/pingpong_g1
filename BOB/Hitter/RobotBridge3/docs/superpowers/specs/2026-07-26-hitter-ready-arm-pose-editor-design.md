# HITTER 双臂等球姿态编辑器设计

日期：2026-07-26
状态：已获用户确认

## 1. 目标

在 `loco1` 上提供一个只绑定到 `127.0.0.1` 的本地网页服务。用户从 Mac 通过 SSH 端口转发打开页面，查看当前完整 G1 HITTER 机器人与右手球拍，并通过左右两侧的 14 个肩、肘、腕关节控件调整“击球恢复后等球”的候选姿态。

保存前，服务端必须使用 RobotBridge2 当前 MuJoCo 资产复核关节限位及双腕、球拍的正向运动学。保存只新建独立、带版本时间戳的 YAML 和 JSON，不修改训练配置、真机配置、机器人资产、ONNX 或任何运行中进程。

## 2. 非目标

- 第一版不提供拖动手腕或球拍的逆运动学。
- 不控制 MuJoCo、Isaac Lab 或真机。
- 不自动写入 `deploy/config/mimic/hitter.yaml`。
- 不修改 URDF、USD、MJCF、motion NPZ 或 ONNX metadata。
- 不编辑腿、腰、头或手指。29-DoF HITTER 模型没有手指控制自由度。
- 不把自碰撞或球桌碰撞作为第一版阻断式安全证明；接入真机前仍需要 MuJoCo 动态验证和平滑回位测试。

## 3. 已确认的用户选择

- 页面运行在 `loco1`，Mac 浏览器通过 SSH 端口访问。
- 采用“双侧控制台”布局：左臂 7 个控件、中间完整机器人、右臂 7 个控件。
- 第一版只使用滑杆和角度数值输入，不加入 IK。
- 保存为独立版本文件，不直接更新运行配置。
- 技术路线采用实时 URDF/STL 显示，并在保存时使用 RobotBridge2 MuJoCo MJCF 权威复核。
- UI 显示角度，内部计算和导出统一使用弧度。

## 4. 资产来源与一致性

### 4.1 显示与浏览器 FK

默认读取：

```text
/home/loco1/BOB/Hitter/MOSAIC-main/source/whole_body_tracking/whole_body_tracking/assets/unitree_description/urdf/g1_hitter_racket/main.urdf
```

网格从该 URDF 当前引用的 STL 实时读取，不在编辑器里嵌入一份长期维护的静态模型。当前训练 USD 与该 URDF 同源；完整 HITTER URDF 包含 29 个活动关节、双侧 3-DoF 腕和 `right_racket_link`。

### 4.2 保存校验

默认读取 RobotBridge2 当前部署资产：

```text
/home/loco1/BOB/Hitter/RobotBridge2/deploy/data/assets/g1/g1_29dof_hitter_racket_table_tennis.xml
```

其配置来源为：

```text
/home/loco1/BOB/Hitter/RobotBridge2/deploy/config/asset/g1_hitter_racket.yaml
```

服务启动时比较 URDF 与 MJCF 的 29 个活动关节名称、父子关系、origin、axis、硬限位和球拍固定安装变换。任一关键运动学签名不一致时，页面仍可只读显示错误，但禁用验证与保存。

导出文件记录 URDF、引用网格集合、MJCF 和关节运动学签名的 SHA-256，防止未来资产变化后静默复用旧姿态。

## 5. 系统架构

新增独立工具目录：

```text
/home/loco1/BOB/Hitter/RobotBridge2/tools/hitter_ready_pose_editor/
```

建议模块边界：

```text
server.py              localhost HTTP 服务、会话令牌和 API 路由
asset_model.py         URDF/MJCF 解析、关节树、网格路由和资产签名
pose_validation.py     14/29维映射、限位检查、MuJoCo FK 和误差比较
pose_serialization.py  YAML/JSON 契约、原子写入和载入
web/index.html         页面结构
web/app.js             WebGL 场景、浏览器 FK、控件和 API 调用
web/styles.css         双侧控制台样式
tests/                 解析、FK、序列化和 API 测试
```

服务仅绑定 `127.0.0.1`，启动时生成随机会话令牌。浏览器请求必须携带该令牌；不提供公网监听、用户系统或远程机器人控制接口。

页面必须在无互联网环境下工作，不依赖 CDN。WebGL 渲染和 STL 解析代码随工具提供；可复用现有 HITTER URDF 调整工具的解析与渲染思路，但不能复用其中会直接修改 URDF 的保存逻辑，也不能复用已过期的内嵌静态资产快照。

### 5.1 数据流

1. 服务启动并读取当前 URDF、STL、MJCF 和默认 29 维关节角。
2. 服务生成浏览器场景 manifest，包含关节层级、origin、axis、limit、默认角和网格 URL。
3. 浏览器加载完整机器人，14 个双臂控件驱动浏览器 FK，交互不依赖每帧服务端请求。
4. 用户点击“检查并保存独立姿态”。
5. 浏览器提交按名称的 14 个弧度值和客户端 FK 摘要。
6. 服务端重建 29 维姿态，执行硬/软限位检查并用 MuJoCo 计算双腕和球拍 FK。
7. 服务比较浏览器与 MuJoCo FK。通过后展示保存前摘要。
8. 用户再次确认后，服务以原子方式新建 YAML 和 JSON。

## 6. 页面交互

### 6.1 中央视图

- 显示完整 G1、右手球拍、地面网格和可隐藏球桌。
- 支持自由旋转、滚轮缩放、双击居中。
- 提供正面、背面、左侧、右侧固定视角。
- 可切换默认姿态半透明残影。

### 6.2 左右关节面板

每侧按照以下顺序固定展示：

```text
shoulder_pitch
shoulder_roll
shoulder_yaw
elbow
wrist_roll
wrist_pitch
wrist_yaw
```

控件同时包含：

- 滑杆；
- 角度数值输入框；
- 当前角度；
- 90% soft range；
- 资产 hard limit；
- 单臂复位按钮。

滑杆默认限制在训练一致的 90% soft range。数值框允许输入 soft range 之外、hard limit 之内的值，但显示黄色警告，并在保存时要求额外确认。超出 hard limit、非有限值或空值均拒绝接受，不做静默裁剪。

soft range 必须以 hard-limit 区间中点为中心，将半区间长度乘以 `0.9` 后得到，不能简单把上下限数值分别乘以 `0.9`。左右 shoulder roll 的不对称范围分别计算。

### 6.3 初始与恢复操作

- 初始姿态使用 `g1_hitter_racket.yaml` 的完整 29 维默认角。
- 页面只修改索引 15–28；腿和腰保持默认值。
- 支持“复位左臂”“复位右臂”“全部复位”。
- 支持载入最近一次保存的合法姿态。
- 页面关闭前有未保存修改时显示离开提醒。

## 7. 关节和索引契约

14 维页面与 RobotBridge2 顺序：

```text
0  left_shoulder_pitch_joint
1  left_shoulder_roll_joint
2  left_shoulder_yaw_joint
3  left_elbow_joint
4  left_wrist_roll_joint
5  left_wrist_pitch_joint
6  left_wrist_yaw_joint
7  right_shoulder_pitch_joint
8  right_shoulder_roll_joint
9  right_shoulder_yaw_joint
10 right_elbow_joint
11 right_wrist_roll_joint
12 right_wrist_pitch_joint
13 right_wrist_yaw_joint
```

对应 RobotBridge2 29 维索引：

```text
15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28
```

对应旧 motion NPZ 索引：

```text
11, 15, 19, 21, 23, 25, 27, 12, 16, 20, 22, 24, 26, 28
```

所有长期保存数据以 `joint_pos_by_name` 为主。数组只作为明确标注顺序的派生字段，禁止保存匿名 14 维数组。

## 8. 输出文件

输出目录：

```text
/home/loco1/BOB/Hitter/RobotBridge2/deploy/data/hitter_ready_poses/
```

每次确认保存生成：

```text
hitter_ready_arm_pose_YYYYMMDD_HHMMSS.yaml
hitter_ready_arm_pose_YYYYMMDD_HHMMSS.json
```

使用临时文件加 `os.replace` 原子落盘；同一秒重名时追加递增后缀，不覆盖已有文件。

YAML 与 JSON 表达相同语义，至少包含：

```yaml
schema: hitter_ready_arm_pose/v1
pose_name: ready_pose_20260726_153000
created_at: "2026-07-26T15:30:00+08:00"
unit: rad

joint_pos_by_name: {}
joint_names: []
joint_pos_rad: []

robot29:
  indices: []
  joint_names: []
  joint_pos_rad: []

motion_npz:
  indices: []
  joint_names: []
  joint_pos_rad: []

fk:
  frame: robot_base_default
  quaternion_convention: xyzw
  links:
    left_wrist_yaw_link: {}
    right_wrist_yaw_link: {}
    right_racket_link: {}

asset:
  display_urdf_path: ""
  display_urdf_sha256: ""
  display_mesh_set_sha256: ""
  validation_mjcf_path: ""
  validation_mjcf_sha256: ""
  kinematic_signature_sha256: ""

validation:
  hard_limits: passed
  soft_limits: passed
  browser_mujoco_fk: passed
  max_position_error_m: 0.0
  max_orientation_error_rad: 0.0
```

`robot_base_default` 表示以默认根姿态下的 pelvis 坐标系为参考；保存的 link 位置和四元数必须先转成 pelvis-relative 结果，避免世界坐标平移影响姿态文件。

## 9. 校验和错误处理

保存的阻断条件：

- 14 个必需关节缺失、重复或出现未知名称；
- 输入不是有限浮点数；
- 任一值超出 hard limit；
- URDF/MJCF 关键运动学签名不一致；
- MuJoCo validator 不可用或加载的不是当前配置资产；
- 浏览器与 MuJoCo 的双腕或球拍 FK 超过允许误差；
- 输出目录不可写或原子写入失败。

允许保存但必须二次确认：

- 任一关节位于 90% soft range 之外、hard limit 之内。

FK 通过阈值：

- 最大位置误差不超过 `0.0005 m`；
- 最大姿态角误差不超过 `0.1°`。

错误在页面底部以明确中文显示，并保留用户当前未保存姿态。服务端错误不得导致现有姿态文件被截断或覆盖。

## 10. API 边界

第一版只需要：

```text
GET  /api/health
GET  /api/model
GET  /api/poses
GET  /api/poses/{filename}
POST /api/validate
POST /api/save
GET  /assets/{asset_hash}/{mesh_path}
```

`/api/save` 不接受客户端提供的任意文件路径。输出目录、文件前缀和扩展名均由服务端固定，防止路径穿越和误写运行配置。

`/assets` 只允许访问启动时从 URDF 解析并登记到 manifest 的网格文件；即使 URL 带有合法会话令牌，也不得通过相对路径访问任意服务器文件。

## 11. 验证计划

### 11.1 服务端自动测试

- URDF 与 MJCF 均解析出 29 个预期活动关节。
- 14 个双臂名称、顺序、默认角和 hard limit 与当前配置一致。
- 默认姿态、每个双臂关节接近 soft min/max 的代表姿态，以及固定随机种子的 100 个合法姿态，浏览器参考 FK 与 MuJoCo FK 均满足 `0.5 mm / 0.1°`。
- NaN、无穷、缺失关节、未知关节、hard-limit 越界和资产签名变化全部拒绝保存。
- soft-range 外、hard-limit 内返回警告并要求二次确认。
- YAML/JSON 保存后重新载入，14 维、29 维和 motion NPZ 映射逐项一致。
- 同一秒多次保存不覆盖已有文件。
- 非法文件名不能通过 pose 读取接口访问输出目录之外。

### 11.2 页面验收

- 完整机器人、双手和右手球拍可见。
- 14 个滑杆与数值框实时更新对应关节，左右不串位。
- 正/背/左/右视角、自由旋转、缩放、残影和球桌开关可用。
- 单臂复位和全部复位只影响预期关节。
- 保存前摘要显示 14 个弧度值、双腕/球拍 FK、soft-limit 警告和资产 hash。
- 保存成功后页面展示两个绝对文件路径，并能重新载入刚保存的姿态。

## 12. 运行与隔离边界

- 使用 `/home/loco1/miniconda3/envs/rb/bin/python` 启动。
- 默认仅监听 `127.0.0.1`，通过 SSH 本地端口转发访问。
- 不连接 `pd_plustau_targets`、LCM、DDS、Vicon、训练 tmux 或真机进程。
- 不占用 GPU，也不启动 Isaac Sim。
- 页面服务停止不影响训练、MuJoCo 或真机。

## 13. 完成标准

当以下条件全部满足，第一版完成：

1. 用户能从 Mac 打开 loco1 页面并看到当前完整 HITTER 机器人。
2. 用户能通过 14 个关节控件调出目标双臂姿态。
3. 所有关节限位来自当前资产，而非手写副本。
4. 保存前 MuJoCo FK 复核通过。
5. YAML/JSON 独立版本文件成功生成且可重新载入。
6. 没有修改或重启任何训练、真机、资产、ONNX 或运行配置。
