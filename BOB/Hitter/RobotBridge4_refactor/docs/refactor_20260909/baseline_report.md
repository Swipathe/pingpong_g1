# 重构前基线

源代码快照提交：`e85fcae`。源目录 1,557 个可读文件内容已复制并验证，12 个不可读标定文件未复制；详见 `source_snapshot.json`。Git 元数据、缓存、构建输出、运行输出、recordings、.codex_backups 等未进入副本，排除路径亦在清单中。Git 忽略的模型/动作数据仍按当前文件复制并记录哈希。

基准项目逐文件分类见 `baseline_comparison.json`：原版相同 1,313 项，修改 11 项，新增 245 项，原版独有 37 项。该清单统计名字存在的文件，新增项中有 12 个内容不可读；不可读不等同于删除。

测试依赖单独安装在 `/tmp/rb4-refactor-tools`，没有修改 isaaclab 环境。运行环境 Python 3.10；格式工具使用基础 Python 3.13。

```bash
cd /home/yhl/Desktop/RobotBridge4_refactor
PYTHONPATH=/tmp/rb4-refactor-tools:$PWD/deploy:$PWD \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MUJOCO_GL=egl \
/home/yhl/miniforge3/envs/isaaclab/bin/python -m pytest \
  deploy/tests deploy/mocap_bridge/tests -q --tb=native --assert=plain \
  --junitxml=/tmp/rb4-refactor-work/baseline.xml
```

结果：**111 failed, 674 passed, 3 skipped, 254 subtests passed**，26.56 秒。pytest 9.1.1 支持并计入 unittest 子测试；以 XML 中完整测试标识比较最终结果，不能只比较总数。

初次普通收集成功（786 tests）；启用默认断言重写并输出短 traceback 的执行遇到 pytest/Python AST 内部异常。使用 `--assert=plain --tb=native` 后完成测试；普通断言仍执行，没有禁用测试或弱化断言。全程禁用第三方 pytest 自动插件加载，保持前后环境一致。

主要已有失败：

- 53 项因 `HealthSnapshot` 不接受旧夹具字段 `planner_results_overwritten_before_consume`。
- 12 项因 `HitterRuntimeSettings` 旧构造缺少新增必填字段。
- 8 项因 `LifecycleSnapshot` 不接受旧夹具字段 `cached_key`。
- 其他包括状态机预期、仿真接口、规划器测试替身签名与当前实现不一致，以及硬编码的其他电脑 Python 路径。
- 一项 C++/Python fixture 比对缺少 `.build-v2` 二进制；另一个协议 fixture 因同样原因跳过。C++ 将在临时目录离线编译并单独核对。

详细失败保留在 `baseline_tests.xml` 和 `baseline_tests.log`。本次按批准的重构边界不修复这些现有问题，不据此宣称项目已具备全部通过的运行基线。
