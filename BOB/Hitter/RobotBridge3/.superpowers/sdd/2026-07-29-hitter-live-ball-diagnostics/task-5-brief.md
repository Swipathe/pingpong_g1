## Task 5: 网页只渲染结构化真实值

**Files:**

- Modify: `deploy/diagnostics/static/hitter_task_monitor.html`
- Test: `deploy/tests/test_hitter_task_frontend.py`

- [ ] **Step 1: 写 frontend 失败测试**

覆盖页面源代码与 JS 渲染 helper：

- `null` 显示 `—`，不能显示 `0/31`、`0/3`、`0.000 s`。
- estimator 使用 `state.ball.estimator_sample_count/window_size`。
- 球诊断 incoming 使用 `state.ball.incoming_count/required_count`。
- 生产 incoming/planner/task obs 使用 `production_gate`，不能与球诊断计数混为一项。
- pelvis 缺失时球卡仍更新，生产卡显示 `PELVIS_UNAVAILABLE`。
- history 计数来自记录值；翻页游标不在每次刷新时重置。
- 不再通过 stage 子串决定颜色和进度。

- [ ] **Step 2: 运行测试确认失败**

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
/home/loco1/miniconda3/envs/rb/bin/python -m unittest \
  tests.test_hitter_task_frontend
```

- [ ] **Step 3: 重写实时卡片绑定**

把当前来球拆成：

1. 球检测/新鲜度；
2. estimator；
3. 球诊断 incoming；
4. 生产 incoming；
5. planner gate；
6. task observation。

统一格式化 helper：

```javascript
const formatOptional = (value, formatter) =>
  value === null || value === undefined ? "—" : formatter(value);
```

TTS 只显示真实计算值；`arm_tts_s` 政名为 `ARM threshold`。

- [ ] **Step 4: 运行 Task 5 测试并提交**

```bash
git add deploy/diagnostics/static/hitter_task_monitor.html \
  deploy/tests/test_hitter_task_frontend.py
git commit -m "fix: render truthful HITTER live diagnostics"
```

---

