# RobotBridge4 重构资料总索引

整理日期：2026-09-19。原先散落在 `/home/yhl/Desktop/` 根目录的 8 份说明文档及配套材料，统一移入本目录。项目位于相邻的 [RobotBridge4_refactor](../RobotBridge4_refactor/README.md)。

## 目录

```text
RobotBridge4_refactor_资料/
├── README.md                         # 本索引
├── 说明文档/                         # 8 份项目、结构、使用及核验说明
├── 核验材料/                         # 两轮核验记录和结构对比清单
├── 历史交付包/                       # 原完整项目包、补丁及原校验文件
├── 整理清单.json                     # 原路径、新路径、移动前后 SHA256
├── RobotBridge4_重构文档资料_20260919.tar.gz
├── RobotBridge4_重构文档资料_20260919.tar.gz.sha256
└── 文档资料包校验.json               # 新文档包的逐文件校验结果
```

## 当前使用入口

| 文档 | 用途 |
| --- | --- |
| [项目结构对比与使用说明](说明文档/RobotBridge4_refactor_项目结构对比与使用说明.md) | 原版 RobotBridge 与当前项目的结构差异、模块用途、仿真和真机命令 |
| [Vicon 脚本结构与用途说明](说明文档/RobotBridge4_refactor_Vicon脚本结构与用途说明.md) | 当前 C++ 主桥、Python 路线、标定、探测和消息监视工具 |
| [部署启动核验](说明文档/RobotBridge4_部署启动核验.md) | 2026-09-09 的仿真、构建与离线验证结果；操作路径已于 2026-09-10 更新 |
| [目录迁移核验](../RobotBridge4_refactor/docs/relocation_20260910.md) | 项目迁至 `BOB/Hitter/` 后的构建修复、运行核验及当前命令，原文继续保留在项目 docs 内 |

## 重构过程与历史说明

| 文档 | 描述的对象与时间 |
| --- | --- |
| [RobotBridge4 README](说明文档/RobotBridge4_README.md) | 重构前原始 RobotBridge4 的功能与使用方式；源码链接仍指向原始项目 |
| [初始项目结构对比](说明文档/RobotBridge_项目结构对比.md) | 原版 RobotBridge 与重构前 RobotBridge4 的差异 |
| [重构方案](说明文档/RobotBridge4_重构方案.md) | 风格和结构调整的目标、边界及实施方案 |
| [重构后项目结构对比](说明文档/RobotBridge4_重构后项目结构对比.md) | 原版、重构前、重构后的结构关系和交付说明 |
| [2026-09-09 完整项目打包说明](说明文档/RobotBridge4_refactor_打包说明.md) | 当时完整项目包的内容、环境范围和已知限制 |

项目内部原有的 [逐文件审计与验证文档](../RobotBridge4_refactor/docs/refactor_20260909/verification_report.md) 和 [交付说明](../RobotBridge4_refactor/docs/refactor_20260909/handoff.md) 保持在原模块中。

## 核验材料

- [部署启动核验目录](核验材料/RobotBridge4_部署启动核验/)：仿真日志、轨迹对照、构建结果、测试结果及当时的核验脚本。
- [目录迁移核验目录](核验材料/RobotBridge4_refactor_迁移核验/)：迁移前后动态库检查、仿真和离线验证结果，以及旧构建备份。
- [结构对比 JSON](核验材料/RobotBridge4_refactor_结构对比清单.json) / [CSV](核验材料/RobotBridge4_refactor_结构对比清单.csv)：逐文件结构与内容差异。

日志、JSON、CSV、补丁、旧构建和核验脚本内容均保留。它们内部的绝对路径记录当时环境，可能已过时；其中核验脚本属于历史材料，不是日常部署入口。需要复现核验时，应先检查脚本中的输入、输出路径。当前部署命令以项目 README、迁移核验和当前使用说明为准。

## 历史交付包

- [完整项目包](历史交付包/RobotBridge4_refactor_完整项目_20260909.tar.gz) / [SHA256](历史交付包/RobotBridge4_refactor_完整项目_20260909.tar.gz.sha256) / [原逐文件校验](历史交付包/RobotBridge4_refactor_完整项目_20260909.tar.gz.校验.json)
- [重构补丁](历史交付包/RobotBridge4_重构变更.patch) / [原补丁校验](历史交付包/RobotBridge4_重构补丁校验.json)

这些交付文件只移动位置，内容和原校验值不变。完整项目包仍是 2026-09-09 的快照，不包含后来的迁移修复及本次资料整理。

## 文档资料压缩包

[下载文档资料包](RobotBridge4_重构文档资料_20260919.tar.gz) / [SHA256](RobotBridge4_重构文档资料_20260919.tar.gz.sha256) / [内容校验](文档资料包校验.json)

新包包含本索引、8 份说明文档、配套核验材料、重构补丁及其校验、整理清单；不重复嵌入 2026-09-09 的完整项目包及其校验文件，也不打包当前项目源码。完整项目包独立保存在上方“历史交付包”中。

解压时保留 `RobotBridge4_refactor_资料/` 目录层级。资料内部文档和证据链接使用相对路径；访问当前项目源码及其 docs 的链接，需要该资料目录与 `RobotBridge4_refactor/` 保持相邻关系。访问原始项目的链接还需要相邻的 `RobotBridge4/`。

[整理清单](整理清单.json) 记录 17 个原顶层条目的去向，共移动 152 个文件。8 份 Markdown 更新了链接和归档说明，其余 144 个移动文件的 SHA256 与移动前一致。本次仅整理资料及文档导航，部署程序、配置、模型、标定和当前构建产物未改动。
