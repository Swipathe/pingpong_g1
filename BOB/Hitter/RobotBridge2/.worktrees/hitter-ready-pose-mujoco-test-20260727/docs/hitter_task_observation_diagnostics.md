# HITTER 任务观测旁路诊断运行手册

这套诊断程序是独立的只读 shadow 进程。它只订阅动捕 LCM 数据，在本进程中复现 estimator、100 Hz planner、50 Hz lifecycle 与 11 维 task observation 的组装过程，并把状态展示在本机网页上。它不连接机器人控制链路，也不会发送 LCM 控制消息；页面结论只代表诊断 shadow 链路，不代表生产 policy 实际收到的 observation。

## 启动

先确认当前使用的是 RobotBridge2 仓库中的配置和标定文件。打开两个终端，按下面顺序运行。

终端 A：启动 ChingMu 到 LCM 的发布桥。

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2
/home/loco1/miniconda3/envs/rb/bin/python deploy/mocap_bridge/chingmu_table_lcm_bridge.py \
  --host 192.168.2.100 \
  --base-subject G1Pelvis \
  --table-calib deploy/mocap_bridge/calibrations/chingmu_table_frame_latest.json \
  --pelvis-orientation-calib deploy/mocap_bridge/calibrations/chingmu_pelvis_orientation_latest.json \
  --publish
```

终端 B：启动只读诊断进程。

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m diagnostics.hitter_task_monitor \
  --mimic-config config/mimic/hitter.yaml \
  --control-config config/control/g1_hitter_racket.yaml \
  --table-calib mocap_bridge/calibrations/chingmu_table_frame_latest.json \
  --pelvis-calib mocap_bridge/calibrations/chingmu_pelvis_orientation_latest.json \
  --lcm-url 'udpm://239.255.76.67:7667?ttl=255' \
  --channel vicon_state_data \
  --base-name G1Pelvis \
  --port 8765
```

终端 B 打印页面地址和本次 session 名称后，在本机浏览器打开：

```text
http://127.0.0.1:8765/
```

HTTP 服务固定只监听 loopback，不提供远程 bind 参数。若 8765 已被占用，可改成其他本机端口，例如 `--port 8766`。

## 页面判断

先看顶部健康栏：

- LCM 应为已连接，`ball`、`g1pelvis` 的 age 应持续刷新；
- pelvis 应为 `VALID`，过期或无效时 task observation 不应判为通过；
- planner 的 submitted/completed 应随来球增长；
- raw、event、recorder 出现 drop 或 incomplete 时，本次记录不能用于稳定的 replay 结论。

每颗球按以下主阶段推进：

```text
球检测
→ ESTIMATING n/31
→ INCOMING_CONFIRMING n/3
→ PLANNER
→ ARMED
→ SHADOW 3/100 TASK OBS
```

`REACQUIRE_GRACE` 表示短暂丢球后仍在等待同一颗球恢复；超过 grace 才关闭 attempt。`WAITING_FOR_PREVIOUS_RECOVERY`、`CACHED_DURING_RECOVERY` 和 `POST_DEADLINE_TAIL` 都是展示层状态，不会改变生产算法的 lifecycle。

## 记录与复现

默认记录根目录固定为：

```text
/home/loco1/BOB/Hitter/RobotBridge2/recordings/hitter_task_diagnostics
```

它不受启动目录影响。每次运行会创建权限为 `0700` 的独立 session 目录，其中包含：

- `session.json`：配置、标定 SHA-256、LCM URL、命令行、Git 状态和版本快照；
- `ball_samples.csv`：按实际到达顺序保存的动捕输入；
- `events.jsonl`：planner、lifecycle、attempt 和 task observation 事件；
- `attempts.csv` 与 `attempt_details/<id>.json`：逐球终态和详情；
- `replay_inputs/attempt-<id>.json`：权限为 `0600` 的逐球完整重放输入；采用原子写入，不跟随符号链接；
- `replay_jobs.jsonl`、`replay_analysis.jsonl`：离线 replay 的任务状态和分析结果。

可以用 `--output-dir /明确/目录` 覆盖记录根目录；相对路径按执行命令时的当前目录解析，并把解析结果写入磁盘 metadata。网页/API 只显示 session basename，不显示本机绝对路径。

每个 attempt 关闭后会自动生成 replay input，并先记录 `QUEUED`。只有当前没有 active attempt 或 `REACQUIRE_GRACE` 时，独立 spawn 子进程才会运行 `3/100` baseline parity；完全一致后才继续运行使用全新 planner 的 `1/100` 反事实。新球出现时，replay 会暂停并在下一个空闲窗口从同一磁盘输入重新开始，不会阻塞 50 Hz tick 或 LCM 输入 FIFO。

完成后，`attempt_details/<id>.json` 和网页详情会写入两个 variant outcome、A/B delta 与摘要。`replay_jobs.jsonl` 的正常顺序为 `QUEUED → RUNNING → COMPLETED`；暂停时会出现 `PAUSED`，子进程异常时为 `FAILED`。以下情况一律显示 `INCONCLUSIVE`，不能解读为 `1/100` 的效果结论：

- raw、event 或输入 FIFO 溢出，导致 `recording_complete=false`；
- baseline 与线上 canonical stage、planner call、50 Hz policy tick、terminal 或 recovery 数据不一致；
- replay 仍在排队、运行、暂停，或最终失败；
- 结果落在 ARM 边界敏感窗口。

Replay 的输入和事件各最多保留 8192 条，约等于 22.8 秒的 360 Hz 输入窗口；超过窗口会明确设置 `recording_complete=false`。A/B 结果只有在 replay job、analysis 和页面事件全部排空并完成 `fsync` 后才会写入 attempt 结论，写盘异常不会显示为通过。

需要自动结束 smoke 或定时采集时加 `--duration`，例如：

```bash
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m diagnostics.hitter_task_monitor \
  --mimic-config config/mimic/hitter.yaml \
  --control-config config/control/g1_hitter_racket.yaml \
  --table-calib mocap_bridge/calibrations/chingmu_table_frame_latest.json \
  --pelvis-calib mocap_bridge/calibrations/chingmu_pelvis_orientation_latest.json \
  --port 8765 \
  --duration 30
```

## 停止与异常

正常停止使用终端 B 的 `Ctrl-C`。进程会依次停止 LCM 接收、关闭 planner/replay、drain recorder、写入并 flush session 终态、关闭 HTTP，所有 join 都有有限超时。不要直接删除仍在写入的 session 目录。

如果页面无数据：

1. 检查终端 A 是否持续发布，以及 `--channel`、`--base-name` 是否与桥一致；
2. 检查页面 LCM age 和 source frame 是否变化；
3. 检查 pelvis 标定文件是否为当前真机使用的版本；
4. 查看 session 中的 `events.jsonl` 和 `session.json`，确认是否出现 `LCM_HEARTBEAT_STALE`、`PELVIS_INVALID_OR_STALE`、drop 或 recorder error；
5. 端口占用时更换 `--port`，不要改成非 loopback 地址。

## 性能验收

2026-07-27 在本机执行 60 秒、360 Hz、21600 条输入验收：输入/raw/event drop 均为 0，LCM handler 的 p99 为 0.022 ms，RSS 增长 55,545,856 bytes，网页状态读取 8.55 Hz。这里的 handler 延迟只覆盖 decode 与 FIFO enqueue；处理线程积压由独立的 bounded FIFO、pending 计数和 drop/incomplete 状态监控。
