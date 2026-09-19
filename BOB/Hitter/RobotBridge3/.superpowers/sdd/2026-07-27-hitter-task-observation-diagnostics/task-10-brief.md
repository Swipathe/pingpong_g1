## Task 10: 实现前端逐球进度页

**Files:**

- Create: `deploy/diagnostics/static/hitter_task_monitor.html`
- Modify: `deploy/tests/test_hitter_task_web.py`
- Create: `deploy/tests/test_hitter_task_frontend.py`

- 先添加静态契约测试：必须包含 6 个主阶段、固定 shadow 声明、健康栏、历史表、detail 区域和 A/B 区域。
- 测 HTML/JS 不含 `innerHTML`、`document.write`、外部 CDN、POST/fetch mutation；动态文本必须通过 `textContent`。
- 实现顶部健康栏：LCM Hz/age、ball/pelvis frame、pelvis age/valid、planner stats、phase、diagnostic drops、config/session basename。
- 实现当前 attempt 的：

  ```text
  球检测
  → ESTIMATING n/31
  → PLANNER-READY INCOMING n/3
  → PLANNER
  → ARMED
  → SHADOW 3/100 TASK OBS 11/11 PASS
  ```

- 实现 `WAITING_FOR_PREVIOUS_RECOVERY`、`CACHED_DURING_RECOVERY`、`REACQUIRE_GRACE` 与 `POST_DEADLINE_TAIL`。
- 实现历史分页、展开 detail、11 维 pre/post clip、primary blocker、planner vectors、A/B outcome/deltas。
- SSE 仅作 invalidator：收到 ready/invalidate/reset 后调度完整 GET `/api/state`；断线指数退避并保持页面可读。
- GET 使用单 token-bucket 规则：事件可立即调度，但距上次 request 不足 100 ms 时合并到下一个 100 ms 边界；任意 1 秒滑窗不超过 10 次。阶段/终态不会等待超过 100 ms。
- 前端丢弃 watermark 小于当前已渲染 watermark 的旧 GET 响应，避免多个异步请求倒序覆盖新状态。
- 运行：

  ```bash
  cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
  PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover -s tests -p 'test_hitter_task_web.py' -v
  PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest discover -s tests -p 'test_hitter_task_frontend.py' -v
  ```

- 在 `test_hitter_task_frontend.py` 用 `html.parser` 验证 landmarks；若存在 `google-chrome-stable`，以临时 profile、`--headless=new --dump-dom` 打开真实测试 server，注入唯一 runtime marker 和形似 HTML 的恶意文本，断言 marker 已渲染且恶意文本没有成为 DOM element。调用固定为 `subprocess.run(..., timeout=10, check=True)`，测试 server 和 temp profile 在 `finally` 关闭；无 Chrome 时只跳过该一项，不跳过静态契约。
- 用浏览器手工确认 1280×720 和 1920×1080 下无横向溢出，断开 SSE 时黄灯而非空白页。
- 提交：

  ```bash
  git add deploy/diagnostics/static/hitter_task_monitor.html \
    deploy/tests/test_hitter_task_web.py \
    deploy/tests/test_hitter_task_frontend.py
  git diff --cached --check
  git commit -m "feat: show HITTER task diagnostic progress"
  ```

