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
