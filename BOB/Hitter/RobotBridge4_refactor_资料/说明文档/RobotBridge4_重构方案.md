# RobotBridge4 代码风格与结构重构方案

> 2026-09-19 资料整理：本文已从 Desktop 根目录移入资料目录，相关链接已更新。[资料总索引](../README.md)。历史核验日期与结论保留。

日期：2026-09-09。状态：方案 B 已在可写副本完成，风格、结构和整体审查均通过；原目录未应用。

## 1. 目标与边界

以 `/home/yhl/Desktop/BAAI-Humanoid/RobotBridge` 的当前文件为基线，对 `/home/yhl/Desktop/BOB/Hitter/RobotBridge4` 做增量重构，执行顺序为代码风格、模块归属、验证、结构对比图。

- 基线项目保持只读。
- 目标项目中与基线相同的文件保持逐字节一致。
- 对已有文件，只调整目标项目扩展部分的风格；不对整份历史文件运行格式化。
- 对新增的第一方程序文件，参照所在模块的原有程序统一风格。
- 保留原版 `deploy/agents`、`envs`、`simulator`、`utils`、`config`、`data`、`unitree_sdk2` 等结构。
- 保持策略输入输出、关节顺序、动作变换、规划算法、状态机、线程同步、消息协议、CLI 与默认配置值不变。
- 不新增旧路径转发文件、兼容别名、运行时 fallback 或其他兼容性代码。
- 不把已有缺陷修复混入风格重构；发现问题时单独记录。例如普通 `MosaicEnv` 与基线的 800/770 维差异不在本次修复范围。

## 2. 现状证据

当前工作区包含大量已有未提交修改，不能从 Git HEAD 重新检出后当作待重构版本。应以当前磁盘文件建立快照，并记录本次新增差异。

目标项目由 `sunzhenguo` 所有，当前 `yhl` 对项目根目录无写权限。建议先在可写副本 `/home/yhl/Desktop/RobotBridge4_refactor` 中完成重构和验证，待目标可写后仅同步经过核对的本次变更。

可用的 `isaaclab` Python 环境具有 NumPy、SciPy、MuJoCo、ONNX Runtime、Hydra、LCM 和 PyTorch，但没有 pytest；基础 Python 环境也没有这些运行依赖。测试执行前需要建立独立的测试依赖覆盖层，避免修改已验证的训练环境。

## 3. 方案比较

| 方案 | 结果与取舍 |
| --- | --- |
| A：只修改排版 | 改动少，但不能解决公共工具放在诊断模块、公共帧类型放在厂商客户端中的归属问题 |
| B：按原有模块整理新增代码，提取少量公共职责（推荐） | 保留框架和主入口，统一风格，修正已确认的依赖方向；需要更新内部 import 和测试 |
| C：新增统一 HITTER 包，搬迁所有 HITTER 文件 | 大量路径和调用关系变化，与保留原版模块布局的要求不符，不采用 |

## 4. 代码风格规则

原版没有检测到统一 formatter/linter 配置，且不同文件已有混合风格。因此按模块就近对齐，不能机械照搬单一格式化工具的默认配置。

| 范围 | 参照与处理 |
| --- | --- |
| `agents/hitter_agent.py` | 参照 `agents/mosaic_agent.py` 的类结构、导入布局、推理循环和注释风格 |
| `envs/hitter.py` | 参照 `envs/mosaic.py`、`envs/level_locomotion.py` 的初始化、成员命名、函数和逻辑分段 |
| 新增 `utils/hitter_*.py` | 参照 `utils/teleop.py`、`utils/kinematics.py` 的数据类、公共工具和导入组织 |
| `simulator/mujoco.py`、`real_world.py` 的新增/修改区域 | 参照各自基线文件的上下文；原有未变区域不格式化 |
| 新增动捕、诊断、测试模块 | 四空格缩进、snake_case 函数变量、PascalCase 类名、简洁注释、清晰导入分组和合理换行 |
| 第一方 C++、Shell | 分别参照原版 C++ 文件和现有脚本的缩进、括号、命名与段落；保留 ABI、编译参数、消息定义和执行语义 |

具体整理导入的分组、空行、尾部空白、冗余括号式换行和超长表达式。不要通过删除类型标注、`dataclass(frozen=True)`、只读数组或校验逻辑来追求“看起来像原版”。不改变日志级别或删除可能承担运行提示的输出。

对于已有副作用或环境设置顺序要求的导入块，保留执行顺序。对纯格式变化使用 AST 等价检查；涉及 import 和定义移动的部分单独验证。

## 5. 新增文件的模块归属

大部分 HITTER 文件已经正确放入原有模块，因此不为了制造目录变化而移动它们。

| 当前位置 | 重构后归属 |
| --- | --- |
| `deploy/agents/hitter_agent.py` | 保留在原有 agents 模块 |
| `deploy/envs/hitter.py` | 保留在原有 envs 模块 |
| `deploy/utils/hitter_planner.py` 等五个运行工具 | 保留在原有 utils 模块 |
| 八个 HITTER YAML | 保留原有配置组及 Hydra `_target_` 路径 |
| 球拍、球桌、球和模型 | 保留在原有 data/assets、data/model 下 |
| `deploy/mocap_bridge` | 作为新增的动捕采集、标定和发布模块保留 |
| `deploy/diagnostics` | 作为新增的独立诊断模块保留，不强行归入 `utils/eval` |
| `deploy/tests`、`deploy/mocap_bridge/tests` | 保持运行测试与动捕测试的分组，统一必要的内部导入 |

### 5.1 公共序列化工具回归 utils

当前 `utils/hitter_realtime.py` 从 `diagnostics/hitter_task_models.py` 导入 `JsonValue` 和 `freeze_json_value`，形成运行核心反向依赖诊断模块。

拟新增 `deploy/utils/hitter_serialization.py`，迁入：

- `JsonScalar`、`JsonValue`。
- `freeze_json_value()`。
- 与冻结值配套的 `to_builtin_json()`。

运行模块、诊断模块及测试均直接引用新位置。诊断专属 schema、数据模型和常量继续保留在 `diagnostics/hitter_task_models.py`，不迁移到公共工具中。更新全部直接引用，不增加旧路径兼容转发。

### 5.2 动捕共享帧类型独立于厂商客户端

当前 `MocapFrame` 定义在 `chingmu_sdk_client.py`，但也被 `vicon_sdk_client.py` 和两类桥接测试使用。

拟新增 `deploy/mocap_bridge/mocap_types.py`，迁入 `MocapFrame` 数据类。ChingMu、Vicon 客户端和相关测试从新文件直接导入；ChingMu 的 ctypes 结构和回调 ABI 保持在原客户端文件中。

### 5.3 历史副本与自动生成文件的处理（已确认）

建议不格式化第三方 SDK 源码、自动生成的 LCM Python/C++ 文件和历史代码副本。它们仍纳入文件清单，不作为当前第一方运行源码修改。

确认没有运行引用的 `deploy/mocap_bridge (copy)` 和第一方 `.before-*` 源码/配置备份，移动到新增的 `_archive/refactor_20260909/` 中，保留原相对路径和文件内容。模型、校准文件及其历史标定副本保留原位，不删除任何历史文件。

如果发现历史副本仍被脚本或工具引用，则先记录引用，不自动移动。所有归档动作都有逐文件映射清单，便于复核和恢复。

## 6. 目标结构示意

```text
RobotBridge4/
├── deploy/
│   ├── run.py                         原样保留
│   ├── agents/hitter_agent.py          原模块，风格整理
│   ├── envs/hitter.py                  原模块，风格整理
│   ├── simulator/                     原结构，仅整理扩展区域
│   ├── utils/
│   │   ├── 原有工具                   原样保留
│   │   ├── hitter_*.py                 已有 HITTER 工具保留归属
│   │   └── hitter_serialization.py     从 diagnostics 迁入的公共职责
│   ├── mocap_bridge/
│   │   ├── mocap_types.py              共享帧类型
│   │   ├── 两类客户端与桥接入口       保留
│   │   ├── calibrations/              原样保留
│   │   └── tests/                     保留
│   ├── diagnostics/                   新增独立模块，保留职责
│   ├── tests/                         保留
│   ├── config/                        保留配置布局和值
│   └── data/                          保留资产和模型
├── unitree_sdk2/                       保留
├── chingmu_sdk/                        厂商 SDK 保留
├── vicon_datastream_sdk/               厂商 SDK 保留
├── docs/                              更新重构和使用说明
└── _archive/refactor_20260909/          仅归档经引用核查的新增历史副本
```

## 7. 验证与交付

1. 建立基线/重构前文件清单，保存内容摘要和文件分类。确保已有未提交修改得到完整保留。
2. 在可写副本建立重构前测试结果，区分已有失败和本次引入的失败。测试前只安装缺失的独立测试依赖，不启动真机或动捕发布。
3. 风格阶段逐文件做语法和 AST 等价检查；核对相同文件的 SHA256、扩展文件中未变区域的保护情况。
4. 结构阶段更新全部调用、测试和文档引用；验证核心 utils 不再依赖 diagnostics，Vicon 不再为了帧类型依赖 ChingMu 客户端。
5. 执行现有规划、状态机、观测、任务诊断、回放和动捕测试。为公共工具迁移增加必要的模块依赖/导入验证，不用测试重复排版实现。
6. 检查 Hydra 配置展开、原入口路径和模型/资产引用。环境允许时执行无窗口、无真机通信的仿真检查；没有执行的检查明确标记。
7. 生成“原版 RobotBridge / 重构前 RobotBridge4 / 重构后 RobotBridge4”的项目结构对比图，以及逐文件修改、迁移、归档清单。
8. 向原目标应用时仅同步本次确认的变更；不整体覆盖工作区或 Git 元数据。目标写权限未解决时交付可写副本和变更清单，并明确原目录尚未应用。

已获确认：采用方案 B、接受第 5.3 节的处理边界，并在上述可写副本中完成工作。

交付：[重构后项目结构对比](RobotBridge4_重构后项目结构对比.md)、[副本与验证说明](../../RobotBridge4_refactor/docs/refactor_20260909/handoff.md)。
