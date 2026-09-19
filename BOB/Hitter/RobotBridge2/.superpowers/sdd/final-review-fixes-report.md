# ChingMu Pelvis 终审修复报告

日期：2026-07-24  
仓库：`/home/loco1/BOB/Hitter/RobotBridge2`  
起始提交：`6fede45ca1a029ec58fa35269a21775b811fc16b`  
最终提交：`e51809e76be777b872264c021562fdf4dd560a48`

## 结论

终审指出的离线代码问题已经按逐行为 RED→GREEN 修复并提交。真实
`chingmu_pelvis_orientation_latest.json` 仍未生成；本轮没有连接
ChingMu、没有运行现场标定、没有发布 LCM，因此交付状态仍是
**offline-complete**，不是 **live-calibrated**。

## 原 SDK polling diff 保留证据

开始修复前：

- `HEAD` 为 `6fede45ca1a029ec58fa35269a21775b811fc16b`。
- index 为空。
- `deploy/mocap_bridge/chingmu_sdk_client.py` 已有用户/前置工作树改动：
  `72 insertions, 1 deletion`。
- 该原始 diff 的 SHA-256 为
  `b7b360eb5e1771ec2df06e3317fcdbcd5c238c3e48e22a80a46ef809fe1878a5`。
- 原 diff 包含 `poll_body_pose`、`body_pose_poll_hz`、后台 polling
  thread、pose cache 和 close 时停止 thread；修复过程中未恢复或删除这些
  行为。

基线验证：

- focused：`22/22`。
- legacy table：`25/25`。
- legacy SDK：`15` 项中 `2 errors`；两个失败均为 cold cache 在后台
  thread 尚未缓存 pose 时直接返回空姿态：
  `test_next_frame_polls_body_pose_when_root_callback_is_absent` 和
  `test_explicit_body_pose_id_overrides_hierarchy_id_for_polling`。

代码提交 `8b4f6a314b23201ee5402296ca73d501d029092c` 纳入了完整 polling
基线，并在其上增加 timestamp 和 cold-cache fallback。提交后 legacy SDK
为 `15/15`。

## TDD RED→GREEN 记录

### 1. Calibration 路径角色

RED：

- 新纯函数 `_validate_calibration_path_roles` 不存在，CLI 测试模块按预期
  import error。
- 测试覆盖四个路径角色的全部六种两两碰撞，并使用不同字符串但
  `Path.resolve()` 后相同的路径。
- 另锁定 `--table-calib` 与 `--save-table-calib` 即使路径不同也必须
  明确拒绝。

GREEN：

- resolve 后任意角色碰撞均报告两个参数名和最终路径。
- table load/save 明确互斥。
- `PelvisOrientationCliTest`：`9/9`。

### 2. 模式相关 finite 数值预检

RED：

- 新纯函数 `_validate_numeric_arguments` 不存在，测试模块按预期 import
  error。
- 原实现的 `<= 0` 允许 NaN 和正无穷；`duration` 没有预检。

GREEN：

- table collection 才校验 `--calib-sec` finite 且 `> 0`。
- pelvis collection 才校验 `--pelvis-calib-sec` finite 且 `> 0`。
- runtime 校验 `--duration` finite 且 `>= 0`。
- 两个 rate 始终 finite 且 `> 0`。
- `main()` 构造器 spy 证明非法 duration 和 table load/save 冲突均在
  SDK 构造前失败。
- `PelvisOrientationNumericCliTest`：`7/7`。

测试首次 GREEN 运行时，Python 3.8 `argparse` 把独立的 `-inf` 识别为新
选项；测试输入改为等价的 `--argument=-inf` 后重跑通过。这是测试命令行
拼法修正，不是生产修复。

### 3. `_collect_calibration_frames` 防御

RED：

- NaN 静默返回空 list，没有抛错。
- 正无穷已经调用 `client.next_frame()`，真实运行会无限循环。

GREEN：

- NaN、正无穷和负无穷均在读取 client 前以包含参数名的 `ValueError`
  拒绝。
- 专用测试：`1/1`。

### 4. `MocapFrame` 兼容 timestamp 字段

RED：

- 显式传入 `body_pose_source_time_s` 得到
  `TypeError: unexpected keyword argument`。

GREEN：

- 在 dataclass 最末尾增加默认 `None` 字段。
- 旧六字段构造仍返回 `body_pose_source_time_s is None`。

### 5. Callback root report timestamp

RED：

- 外层 frame 首个 marker 时间为 `12.100 s`、root report 时间为
  `12.125 s` 时，输出 pose timestamp 为 `None`。

GREEN：

- root callback 保存自身 `msg_time`，输出 frame 同时保持外层
  `source_time_s=12.100` 和 pose `body_pose_source_time_s=12.125`。

### 6. SDK rate 防御

RED：

- 直接构造 SDK client 时 NaN 和正无穷 `body_pose_poll_hz` 未被拒绝。

GREEN：

- SDK client 自身也要求 `body_pose_poll_hz` finite 且 positive，错误包含
  参数名。

### 7. Poll timestamp 与 cold cache

RED：

- cache 冷启动时 `_frame_with_polled_body_pose()` 的 vendor 调用数为
  `0`，空姿态直接返回。

GREEN：

- `CMTrackerExternTC` 的 `Timeval` 与 position/quaternion 一起返回、
  缓存和注入 frame。
- cache 冷时同步 poll 一次；专用测试确认 vendor 调用恰好一次且
  `42.125 s` timestamp 被传播。
- legacy SDK 从基线 `2 errors` 恢复到 `15/15`。

### 8. Finite、严格递增且唯一的 pose 样本

RED：

- 外层时间完整时，重复 pose timestamp 和全部非有限 pose timestamp
  均错误生成标定，没有抛错。

GREEN：

- 只计入 finite 且相对上一个已接受 pose 时间严格递增的样本。
- 重复、回退和非有限 timestamp 不计入样本数。

### 9. Pose-time coverage

RED：

- 外层 frame 覆盖 `1.08 s`、pose 只覆盖 `0.90 s` 时旧算法错误通过。

GREEN：

- `source_duration_s` 改由接受后的 pose timestamp 首尾差计算，测试按
  `source-time coverage` 拒绝。

### 10. Pose/frame age

RED：

- 61 个唯一 pose 覆盖 `1.5 s`，但每个比外层 frame 老
  `0.100001 s` 时旧算法错误通过。

GREEN：

- 明确常量 `MAX_PELVIS_BODY_POSE_FRAME_AGE_S = 0.1`。
- 只接受 pose/frame 绝对时间差不超过 `0.1 s` 的样本。
- 完整 math 类：`10/10`。

### 11. 超大有限 quaternion

RED：

- `[1e308, -1e308, 1e308, 1e308]` 的普通范数上溢，随后 SciPy 报
  `Found zero norm quaternions in quat`。

GREEN：

- 先按最大绝对分量缩放，再求范数和归一化；保留原有最小模长拒绝语义。
- 完整 bridge 路径恢复预期旋转。

### 12. 非单位 table basis 与非交换旋转

RED：

- 2×/3×/4× table basis 使发布 position 三轴错误放大；最大差
  `1.33990192 m`。

GREEN：

- position 和 rotation 共用按行稳定单位化后的 table basis。
- 端到端测试锁定
  `R_table_from_raw @ R_raw_rigid @ R_rigid_from_pelvis`、刚体原点位置和
  pelvis local `+X`。

### 13. Ctrl-C 模式相关退出码

RED：

- 三条测试进入真实 `main()` 控制流，只替换 ChingMu SDK client 外部边界。
- table calibration 和 pelvis calibration 的 collection 被
  `KeyboardInterrupt` 中断时，旧实现均错误返回 `0`；断言分别得到
  `0 != 130`。
- normal runtime 的 Ctrl-C 已保持返回 `0`。

GREEN：

- table calibration 和 pelvis calibration 被 Ctrl-C 中断时返回 `130`。
- normal runtime 的 Ctrl-C 继续返回 `0`。
- 三种模式均验证 `finally` 调用了 `client.close()`；两种校准模式还验证
  没有留下目标 calibration JSON。
- 专用控制流测试：`3/3`；完整 focused：`45/45`。

## 提交

1. `8b4f6a314b23201ee5402296ca73d501d029092c`
   `fix: harden ChingMu pelvis calibration`
   - 仅修改 bridge、SDK client 和 focused test 三个代码路径。
2. `08f227bf5a247f6426fc71dbb177caf393fe44ff`
   `docs: record ChingMu pose freshness requirement`
   - 仅修改批准的 design 和 plan。
3. `e51809e76be777b872264c021562fdf4dd560a48`
   `fix: report interrupted ChingMu calibration`
   - 仅修改批准的 bridge 和 focused test。

`6fede45..e51809e` 只包含以下五个允许路径：

```text
deploy/mocap_bridge/chingmu_sdk_client.py
deploy/mocap_bridge/chingmu_table_lcm_bridge.py
deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py
docs/superpowers/plans/2026-07-24-chingmu-pelvis-orientation-calibration.md
docs/superpowers/specs/2026-07-24-chingmu-pelvis-orientation-calibration-design.md
```

## Fresh 最终验证

- focused：`Ran 45 tests`，`OK`。
- legacy SDK（`git show HEAD:<test>` stdin 执行）：`Ran 15 tests`，`OK`。
- legacy table（`git show HEAD:<test>` stdin 执行）：`Ran 25 tests`，
  `OK`。
- clean `git archive HEAD` 全部 mocap Python：`Ran 91 tests`，`OK`。
- clean `git archive HEAD` 临时编译并运行 C++ mocap：
  `test_vicon_table_lcm_bridge: PASS`。
- `py_compile`：bridge、SDK client、focused test 共 `3` 文件通过，pyc
  写入临时目录。
- CLI help：包含 pelvis load/save/calib-sec 和 body-pose-poll-hz。
- forbidden old symbols：`pelvis_offset_heading_m`、`initial_base_yaw`、
  `relative_yaw`、`yaw_quaternion_from_rotation` 均无匹配。
- clean HEAD archive AST/签名检查：
  `main()` 的唯一 `ChingMuSdkClient` 调用关键字与构造签名一致，包含
  `body_pose_id`、`poll_body_pose`、`body_pose_poll_hz`。
- 提交区间 `git diff --check` 通过。
- 五个目标文件相对 HEAD 无工作树 diff。
- index 为空。
- 真实 pelvis orientation JSON 未创建、未跟踪、未提交。
- 原有全部 `.D` 路径仍保持 `.D`；提交区间没有任何删除。

C++ 测试为非法半径用例打印四行预期错误诊断（`-0.1`、NaN、正负无穷），
最终退出码为 `0` 且打印 `PASS`。

## 自审

- 没有用姿态数值变化判断 freshness；静止且数值相同、时间递增的样本仍
  被接受。
- 路径冲突比较 resolve 后路径；不同 calibration 角色不能覆盖同一文件。
- 模式无关的 duration 不会被错误拒绝。
- 所有新增 CLI 拒绝发生在 SDK import/构造前。
- callback 和 poll 分别使用各自 SDK timestamp，不借用 marker frame
  timestamp。
- 标定数学仍集中在 bridge；SDK 变更只做 pose 获取、缓存和 timestamp
  传播。
- 校准阶段 Ctrl-C 以 `130` 明确报告未完成；normal runtime Ctrl-C 仍以
  `0` 友好退出；所有路径均关闭 SDK client。
- 未恢复、修改或暂存任何原有 `.D` 测试。
- 未运行 live calibration；现场摆正、真实 JSON 生成和真机方向验收仍是
  后续人工步骤。
