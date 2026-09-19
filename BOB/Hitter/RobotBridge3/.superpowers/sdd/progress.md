# MuJoCo 纯物理乒乓球实施进度

- 基线：`c6abd3a`；语法检查通过；MJCF 可加载（`nq=43 nv=41 npair=1`）。
- 任务 1：完成（未提交工作树补丁；RED 2 failures；GREEN 2 tests OK；独立审查通过）。
- 任务 2：完成（未提交工作树补丁；RED 1 failure；GREEN 3 tests OK；语法通过；独立审查通过）。
- 任务 3：完成（最终 `solref=0.04 0.10`；旧参数 RED 捕获 episode2 数值增能；GREEN 7 tests OK；三次恢复比约 0.720/0.685/0.774；复审批准）。
- Minor：测试未单独断言原始 pair 元素数量为 3，也未逐项锁定 pair 的全部静态接触属性；当前 XML 与只读参数检查均正确，交最终跨文件审查统一判定。
- 任务 4：最终 fresh 验证通过（10 tests OK；语法、Hydra、编译后接触模型、禁用标识扫描、diff 和暂存区检查均通过）。
- 最终审查修复：已恢复 `randomize_ball_on_reset` / `ball_random_seed` 初始化；三个行为测试先 RED 后 GREEN；pair 数量和全部静态属性断言已补强。最终复审确认无 Critical/Important，技术交付结论为 Yes。

# MuJoCo 两次发球实施进度

- 基线：`5349b48`；现有纯物理乒乓球测试 10/10 通过；获准在当前 `main` 脏工作区实施。
- Task 1：完成（未提交工作树补丁；RED 1 error；GREEN 11/11；两次 mutation 均按预期失败；规格与质量复审通过）。
- Task 1 Minor：内部报告三个审查小标题仍为英文，最终清理时统一中文。
- Task 2：完成（启动时序测试先 RED 后 GREEN；public `reset()` 在首次 physics step 前于 0.000 s 消费首球；`TwoServeScheduleTests` 10/10、聚焦文件 21/21；最终复审确认原 Important 已解决，无新增 Critical/Important，Ready=Yes）。
- Task 3：完成（fresh 聚焦测试 21/21；Hydra 确认 MuJoCo、table tennis enabled 与 `[0.0, 20.0]`；py_compile、全工作树 diff check 均通过；实际 viewer 日志为 0.000 s 与 20.000 s，第二球后继续约 5 s 无第三球；无残留 MuJoCo run.py 进程）。

# ChingMu Pelvis 旋转外参标定实施进度

- 基线：`d05ad32`；分支 `local/mujoco-air-model-20260723`；暂存区为空；bridge `py_compile` 与 `--help` 通过；保留当前脏工作区执行。
- Task 1：完成（提交 `655764f`、`df9cba0`；新数学测试 6/6、Git 中旧 bridge 测试 12/12；规格与质量正式复审均通过，无 Critical/Important/Minor；旧测试在当前工作树继续保持用户指定的未暂存 `D`，index 为空）。
- Task 2：完成（提交 `d24e2b8`；JSON + 数学测试 10/10；严格字段、单位四元数、质量门限与原子失败清理均经复审；规格与质量 PASS，无 Critical/Important/Minor）。
- Task 3：完成（提交 `c3c5426`、`aab6d66`；位置改为桌面世界系刚体原点，姿态严格右乘外参，正常 runtime 强制加载标定；focused 15/15、legacy 12/12、联合 27/27；正式复审 PASS，无 Critical/Important/Minor；legacy 工作树保持未暂存 `D`）。
- Task 4：完成（提交 `6fede45`；新增一次性 pelvis 标定、table-only 与 runtime 三模式；focused 22/22、legacy 全文件 25/25；32 种模式组合、控制流和不发布边界正式复审 PASS，无 Critical/Important/Minor；未生成真实标定 JSON）。
- Task 5：离线验收完成（fresh focused 22/22、Git HEAD legacy 25/25；py_compile、help、旧符号、diff、模式安全探针均通过；桌面标定文件存在；未调用 SDK/LCM，未生成 orientation JSON）。现场边界：需操作者摆正并确认策略停机后才能生成真实外参。
- 最终 review 修复：提交 `8b4f6a3`、`08f227b`、`e51809e`；关闭标定路径覆盖、非有限时长、rigid-pose freshness、超大四元数、非单位桌面基覆盖、clean SDK polling 接口和标定 Ctrl-C 状态问题。最终 clean archive 91/91，focused 45/45，正式终审 Critical/Important/Minor 均为 0，Ready=Yes。真实 orientation JSON 仍按现场安全边界未生成。

# HITTER 每次击球目标速度日志实施进度

- 基线：`fa7d3c0`；分支 `main`；当前脏工作区和用户已有改动全部保留；设计与实施计划已提交。
- Task 1：完成（提交 `c65a543`；RED 从 1 个 error 扩展到 3 个预期 error；GREEN 3/3；独立 task review 无 Critical/Important/Minor）。
- Task 2：完成（提交 `479e97f`；RED 5 项中仅 late advance 预期失败；GREEN 5/5；独立 task review 无 Critical/Important/Minor）。
- Task 3：完成（日志测试 5/5；MuJoCo 23 项仅 1 个既有 friction 失败；语法、whitespace、静态唯一性和 Git 指纹通过；独立 review PASS）。
- 最终跨任务 review：完成。发现并复现 epsilon 边界漏记；提交 `c7e1e85` 改为依据实际 phase transition，新增 nextafter 回归；RED 6 项中 1 个预期失败，GREEN 6/6，MuJoCo 无新增失败；fix review 与最终复审均无 Critical/Important/Minor，原 finding 已关闭。
