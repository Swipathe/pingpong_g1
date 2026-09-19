# Task 9 实现报告

## 结果

- 状态：实现与本地验证完成，等待父任务统一 code review。
- 分支：`local/hitter-task-diagnostics-20260727`
- 提交：`c5589cb feat: serve HITTER diagnostic state`
- 提交范围严格为：
  - `deploy/diagnostics/hitter_task_web.py`
  - `deploy/tests/test_hitter_task_web.py`

## 实现

- `WebServerConfig` 只接受 `127.0.0.1`、`::1`、`localhost`；绑定后再次用
  实际地址验证 loopback，`port=0` 时 `address` 返回实际 ephemeral port。
- 使用 `ThreadingHTTPServer` 子类和 daemon accept/handler threads。总
  handler 使用 nonblocking bounded semaphore，默认 8；SSE 使用第二层
  semaphore，默认 4；两层超限均返回统一 JSON 503。
- 普通 socket 使用配置的 5 秒 timeout。`close()` 幂等、有界，关闭活动
  cursor/socket、shutdown listener，并在剩余 deadline 内 join accept 与
  handler threads。
- Host 必须精确为 `127.0.0.1:<actual-port>`、
  `localhost:<actual-port>` 或 `[::1]:<actual-port>`；所有其他形式返回
  JSON 400。
- request target 先做严格 UTF-8 URL decode，再精确匹配固定 route；
  absolute-form URL、`..`、反斜杠、encoded/double-encoded traversal 和未映射
  文件路径均返回 404。静态内容在 application 构造时只从显式 mapping 读取，
  不做 URL 到文件系统路径拼接。
- `GET /api/state` 使用 `EventHub.state_snapshot()` 的 canonical schema；
  attempt 列表严格 descending、`before` exclusive，`limit` 默认 50、最大
  200；query 与 detail id 仅接受 canonical decimal positive integer。
- detail 只调用 `AttemptDetailRepository.get()`，因此只命中 cache 或固定
  `<positive-id>.json`。API 序列化边界递归移除 path/download 字段并 redaction
  意外绝对路径字符串，不暴露 session 文件位置或下载接口。
- 400/404/405/503 使用同一
  `schema_version + error(status/code/message)` JSON schema，并完整设置
  `Content-Type`、`Content-Length`、`Cache-Control: no-store` 和
  `X-Content-Type-Options: nosniff`；没有 CORS header。
- SSE fresh cursor 发送 `ready`；resume 只把 ring event 映射为
  `invalidate(scope, attempt_id)`；bootstrap/runtime gap 发送 `reset` 并由
  cursor 跳到当前 watermark；heartbeat 只由注入 clock 判断。
- EventHub `read()` 返回 immutable event 后才做 socket write，不在 hub lock
  内写网络；每个慢客户端只占自己的 cursor/handler。断开检查每 0.1 秒一次，
  即使没有新事件也会进入 `finally` 关闭 cursor 并释放 SSE semaphore。
- `/` 已用 Task 10 并行生成的
  `deploy/diagnostics/static/hitter_task_monitor.html` 做真实 body/header
  集成测试；Task 9 不修改、也不提交该静态页。

## RED → GREEN

初始 RED：

```text
ModuleNotFoundError: No module named 'diagnostics.hitter_task_web'
Ran 1 test
FAILED (errors=1)
```

安全边界增量 RED：

```text
FAIL: test_detail_uses_only_numeric_repository_ids_and_leaks_no_paths
AssertionError: '/tmp/...' unexpectedly found in detail JSON
```

断开清理增量 RED：

```text
FAIL: test_disconnect_and_server_close_release_cursor_without_polling
AssertionError: False is not true
```

最终 Task 9 GREEN：

```text
Ran 12 tests in 1.208s
OK
```

## 最终验证

- Python 3.8 `py_compile`：通过。
- Task 2/3/4/5/9 相关回归：

  ```text
  Ran 77 tests in 1.757s
  OK
  ```

- `git diff --cached --check`：通过。
- commit 前 cached name-status：恰好两个 Task 9 新文件。
- commit 后 `git show --name-status c5589cb`：恰好两个 Task 9 新文件。

## 覆盖

- state schema、真实/fixture static body、content types、安全 headers。
- attempt 默认/最大 limit、descending/exclusive pagination、严格 query/id。
- cache/fixed numeric disk detail、404、path/download redaction。
- 400/404/405/503 统一 JSON、严格 Host、无 wildcard CORS、traversal。
- fresh ready、完整 GET state、resume invalidation、gap reset/跳 watermark、
  无 state patch、fake-clock heartbeat。
- handler/SSE 两层 semaphore、503、普通 socket timeout。
- 客户端主动断开与 server close 均通过 `threading.Event` 同步验证 cursor
  释放，不使用 `sleep` 猜时序。

## 风险与边界

- Task 9 commit 有意不包含 Task 10 静态页；真实 `/` 集成测试依赖父任务把
  `hitter_task_monitor.html` 一并保留/提交。
- HTTP 服务只读并旁路 EventHub/Repository；本任务不装配 monitor CLI，也不
  关闭共享 EventHub。
- `task-9-report.md` 是 SDD 编排元数据，不纳入 Task 9 commit。
