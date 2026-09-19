# RobotBridge4 重构验证说明

验证对象为 `/home/yhl/Desktop/RobotBridge4_refactor`。原版与原始 RobotBridge4 保持只读；本报告区分代码等价性检查、原有测试问题、构建环境补齐，以及尚未执行的设备验证。

## 1. 文件与代码边界

- 快照保存了 1,557 个可读文件的当前磁盘内容，没有用 Git HEAD 覆盖已有改动。
- 与原版相同的 1,313 个文件逐字节保留；最终同时核对原版项目 1,361 个文件和原始 RobotBridge4 的快照内容未变。
- 风格阶段审阅 71 个新增第一方程序文件和 2 个原文件扩展：修改 54 个 Python、6 个 C++ 及 2 个模拟器文件的扩展区域，11 个文件经检查保持原样。
- Python 风格阶段 62 个文件 AST 相同；C++ 六个文件 token 序列相同；两份模拟器文件共 63 个原版未变代码块保留原字节。独立风格审查已通过。
- 配置值、模型、活动标定、厂商 SDK、生成的 LCM 代码均不做风格改写；20 个历史文件通过映射和 SHA256 核对归档。

风格阶段可在副本 Git 历史上独立复现，后续结构调整不会影响此检查：

```bash
cd /home/yhl/Desktop/RobotBridge4_refactor
/home/yhl/miniforge3/envs/isaaclab/bin/python \
  docs/refactor_20260909/style_check.py --revision f558b7c
```

完整证据见 [风格审计](style_audit.json)、[风格报告](style_report.md)、[风格审查](style_review.md)、[原文件快照](source_snapshot.json) 和 [最终文件审计](final_audit.json)。

## 2. 测试环境与重构前结果

既有 isaaclab 环境提供 Python 3.10.20、NumPy、SciPy、MuJoCo、ONNX Runtime、Hydra、LCM、PyTorch 等依赖。缺失的 pytest 和格式工具单独安装到 `/tmp/rb4-refactor-tools`，没有修改训练环境的软件包。

实际安装的工具版本为 pytest 9.1.1、Black 26.5.1、clang-format 23.1.0。Black 在基础 Python 3.13 中执行，使用 `--target-version py310 --line-length 120 --skip-string-normalization`；Python 3.10 再做语法与 AST 核对。没有为原项目新增全局 formatter 配置。

完整 Python 命令：

```bash
cd /home/yhl/Desktop/RobotBridge4_refactor
PYTHONPATH=/tmp/rb4-refactor-tools:$PWD/deploy:$PWD \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MUJOCO_GL=egl \
/home/yhl/miniforge3/envs/isaaclab/bin/python -m pytest \
  deploy/tests deploy/mocap_bridge/tests -q --assert=plain --tb=native \
  --junitxml=/tmp/rb4-refactor-work/final.xml
```

重构前结果：`111 failed, 674 passed, 3 skipped, 254 subtests passed`。纯风格阶段复跑逐项一致；详情见 [基线说明](baseline_report.md) 与 [风格阶段结果对比](style_test_comparison.json)。

pytest 9 会单独报告 unittest 子测试，因此控制台统计与 JUnit 父测试统计不同。基线 JUnit 父测试是 110 failed、673 passed、3 skipped；比较使用完整测试标识，同时保留原始日志，不把不同口径的计数直接相减。

运行中曾遇到标准库 AST/ssl/enum 的偶发收集异常。独立导入、相关文件的 47 项测试及完整重跑可完成；本次没有为此修改项目代码。使用 `--assert=plain` 保留普通断言执行，`--tb=native` 保留 Python traceback；这两项不是跳过或弱化失败断言。仍应把该环境的不稳定性与确定的代码失败分开看待。

## 3. 离线构建、配置与资源检查

- 六个新增第一方 C++ 文件全部重新编译，输出位于 `deploy/mocap_bridge/.build-v2/`；两个 C++ 协议/球轨迹测试运行通过。
- 14 项 Python/C++ 跨语言 fixture 测试通过。预先缺少二进制所造成的一项失败和一项跳过得到解决；这是构建测试环境补齐，没有改变协议或跟踪算法。
- 启动器 `run_robotbridge4_vicon_real.sh --check-only` 通过：两份 `20260822` 标定 SHA256、LCM channel/subject 及 planner 球桌参数一致。该模式在发布分支之前退出，没有连接动捕设备或发布消息。
- 八份新增 HITTER YAML 随 `hitter` 主配置完成组合；MuJoCo 和 real_world 两种配置均可完整展开，入口仍是 `agents.hitter_agent.HitterAgent`、`envs.hitter.HitterEnv` 和原模拟器类。
- MuJoCo 球桌机器人 XML 无窗口加载成功，独立执行 10 次物理步进，qpos 有限。
- 当前 `hitter_model20500_20260818_104.onnx` 使用 CPU 完成零输入推理，输入 `[1, 104]`、输出 `[1, 29]`，输出有限。此项是模型加载/推理检查，并非完整策略控制闭环验证。

系统的 `pkg-config lcm` 不可用，但既有 isaaclab 的 LCM 安装包含头文件和动态库。编译使用实际目录的 `-I`、`-L` 和 rpath 参数；没有修改原构建脚本或增加 fallback。详见 [C++ 验证日志](final_cpp.log)、[跨语言测试](cross_language_tests.log) 与 [配置和资源结果](config_assets.json)。

## 4. 数据与验证范围

源目录中 12 个其他标定文件因权限不可读，未复制、未替代、未伪造；逐文件列表见 [快照清单](source_snapshot.json)。当前启动器实际使用的两份 `20260822` 标定均可读、已复制并通过预检。

Git 元数据、缓存、历史本地 .codex_backups、构建目录、outputs 和 recordings 等路径未作为源码快照复制；精确排除路径同样列入清单。副本中的 `.build-v2/` 为本次新建的验证构建产物。

没有连接机器人、Vicon 或 ChingMu 设备，没有启动策略真机控制、没有执行在线动捕发布。没有执行需要显式设置环境变量的 60 秒实时性能验收。浏览器测试的跳过原因保留在测试日志中。保留的旧测试失败，以及普通 MosaicEnv 已有的 800/770 维差异，均没有混入这次重构修复。

## 5. 结构阶段与最终测试结果

五个被迁移的定义（两个 JSON 类型别名、两个 JSON 函数、一个 MocapFrame 类）与风格阶段提交 `f558b7c` 的 AST 一致；其余 13 个编辑模块扣除 import 和指定迁移节点后 AST 一致。所有活动调用方从新模块导入；没有专门为旧路径增加兼容导出。新增四项测试经历了缺少新模块时失败、迁移后通过的 red-green 验证。

最终完整测试控制台结果：**110 failed, 680 passed, 2 skipped, 254 subtests passed**，27.11 秒。JUnit 父测试口径为 109 failed、679 passed、2 skipped。

与重构前逐项比较：**新增失败 0、缺失测试 0、新增测试 4（全部通过）**。只有两个旧测试结果变化：C++/Python 跟踪 fixture 从失败变为通过，C++ 消息 fixture 从跳过变为通过，原因均为测试二进制补齐。其余旧失败保持原有状态，不能把这个结果表述成“全部测试通过”。

ChingMu 桥接、Vicon 桥接、诊断监控三处 CLI 的 `--help` 均退出 0；Hydra 四个目标类可正常导入解析。这些检查没有创建真实控制/动捕对象。

详见 [结构迁移报告](structure_report.md)、[完整测试逐项对比](test_comparison.json)、[最终测试日志](final_tests.log)、[最终 JUnit](final_tests.xml) 与 [入口检查](entrypoints.json)。

## 6. 已知文档差异与交付

当前启动器早已选用 20260822 的标定组合，旧部署文档仍写 20260818；本次仅同步说明中的日期和两个标定文件名。脚本、SHA256 常量、planner 配置及实际标定内容均不改变。历史设计记录保留，并注明旧 MocapFrame 导入路径已迁移。

原始 RobotBridge4 目录无写权限，因此本次交付在已确认的可写副本完成，没有直接覆盖原项目。逐文件变化见 [JSON 清单](change_manifest.json) 或 [CSV 清单](change_manifest.csv)，20 个归档文件见 [迁移清单](archive_manifest.json)。本地 Git 保留 `e85fcae` 快照、`f558b7c` 风格阶段、`a17d1e7` 公共模块提取阶段，便于按阶段核对。

整体审查通过，无本次范围内需要修复的问题，见 [最终审查](final_review.md)。副本、补丁检查与桌面文档的交付约定见 [交付说明](handoff.md)，各阶段记录见 [执行记录](execution_record.md)。
