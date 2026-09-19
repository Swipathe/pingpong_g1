## Task 6: 文档、全量回归与现场验收准备

**Files:**

- Modify: `docs/hitter_task_observation_diagnostics.md`
- Test: all `deploy/tests/test_hitter_task_*.py`

- [ ] **Step 1: 更新运行说明**

写明：

- 球诊断不需要 pelvis。
- pelvis 缺失会阻塞生产 planner/task observation，这是预期且必须显式显示。
- 各状态字段的真实含义。
- 网页重启命令与不运行机器人的球链路检查命令。

- [ ] **Step 2: 运行全部 diagnostics 测试**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
/home/loco1/miniconda3/envs/rb/bin/python -m unittest discover \
  -s tests -p 'test_hitter_task_*.py'
```

预期：全部通过；环境相关 skip 保持明确。

- [ ] **Step 3: 做静态与范围检查**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2
git diff --check
git diff -- deploy/envs/hitter.py
git status --short
```

确认本任务没有触碰生产 `deploy/envs/hitter.py`；该文件原有用户改动保持不变。

- [ ] **Step 4: 本地提交**

```bash
git add docs/hitter_task_observation_diagnostics.md
git commit -m "docs: explain truthful HITTER ball diagnostics"
```

- [ ] **Step 5: 最终验收输出**

交付内容必须包含：

- 修复的根因。
- 单测总数与结果。
- pelvis 缺失时页面应看到的精确状态。
- 需要用户重启 monitor 才能加载新代码的命令。
- 明确说明没有修改生产 policy/planner/action 链路。
