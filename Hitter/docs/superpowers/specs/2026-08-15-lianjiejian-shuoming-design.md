# 连接件仿真资产本地说明文档设计

## 目标

在 `/home/yhl/Desktop/Hitter/new_racket_v1/` 下新增本地长期维护文档 `lianjiejian_shuoming.md`，完整记录从旧手握球拍资产改为连接件、球拍套、球拍刚性总成的过程，以及 Isaac Lab/MOSAIC 和 RobotBridge/MuJoCo 中切换、播放、训练和恢复旧资产的方法。

## 文件关系

- 新建 `new_racket_v1/lianjiejian_shuoming.md`，作为主入口。
- 保留 `new_racket_v1/TRANSFORM.md`，作为坐标变换、哈希和网格校验附录。
- 保留 `new_racket_v1/SWITCH_ASSETS.md`，作为简短切换速查。
- 不修改现有 USD、URDF、STL、MuJoCo XML 或默认配置。

## 内容结构

主文档按实际工作顺序组织：

1. 修改目的与新总成结构。
2. SolidWorks 装配、输出坐标系和质量属性记录。
3. 三个 STL 的导出、坐标变换与验证结果。
4. Isaac Lab/MOSAIC 中生成的 URDF、USD 和 31-body 兼容层。
5. RobotBridge/MuJoCo 中新增 XML 和网格位置。
6. 默认旧资产与显式新资产的切换逻辑。
7. 使用新资产回放参考动作、播放 checkpoint、单卡训练和多卡训练的命令。
8. MuJoCo 静态查看和 RobotBridge ONNX 运行命令。
9. 切回旧资产的方法。
10. 已知限制、常见错误和后续需要复核的质量/惯性事项。

## 路径与命令原则

- 文档面向 YHL 当前电脑，使用 `/home/yhl/Desktop/Hitter/` 下的真实绝对路径。
- Isaac 新运行资产固定指向 `MOSAIC-main/.../g1_hitter_racket_cad_v1/main_motion_compatible.usda`。
- 说明 `MOSAIC-main-sijie-20260806` 可在启动命令中跨目录引用上述 USD，不重复复制资产。
- MuJoCo 新资产固定指向 `RobotBridge2_20260722_1726/deploy/data/assets/g1/g1_29dof_hitter_racket_table_tennis_cad_v1.xml`。
- 不使用当前 RobotBridge 未实现的 `robot.control.playback_slowdown` 参数。
- 命令不改变默认配置；省略环境变量或 Hydra override 时恢复旧资产。

## 必须记录的技术数据

- 父坐标系：`right_wrist_yaw_link_frame`。
- 子坐标系：`right_racket_link_frame`。
- 平移：`[0.22279, 0.00685, -0.00291] m`。
- 旋转：绕 X 轴 180°；URDF RPY `[3.141592654, 0, 0]`；MuJoCo `wxyz=[0,1,0,0]`。
- 总成质量：`0.36624 kg`。
- 质心：`[-0.04789, 0.00685, -0.00083] m`，相对于 `right_racket_link_frame`。
- 质量和惯性暂来自 SolidWorks 均匀密度结果，正式实物部署前需要复核。

## 已知限制

- 当前 MuJoCo Hitter 新旧 XML 都没有把 `right_wrist_yaw_link.STL` 绑定为可视 `geom`，因此末端 yaw 手腕外壳不显示；本次只记录，不修改。
- 旧 checkpoint 在新几何和质量下只用于链路检查，不保证策略效果不变。
- `main_motion_compatible.usda` 依赖旧 `g1_hitter_racket/main.usda`，旧资产不能删除。

## 验证标准

- 文档内所有本机文件路径实际存在，checkpoint 或动作示例除外时必须明确说明可替换。
- Shell 代码块语法完整。
- Isaac 和 MuJoCo 新旧切换命令与当前代码实现一致。
- 不包含有效的 `playback_slowdown` 启动参数。
- 文档明确链接 `TRANSFORM.md` 和 `SWITCH_ASSETS.md`。
