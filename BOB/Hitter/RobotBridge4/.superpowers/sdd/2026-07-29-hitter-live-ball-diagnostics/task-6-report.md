# Task 6 交付报告：中文运维手册、全量回归与现场验收

## 状态

已完成中文运维文档更新、当前 CLI/help 只读核验、完整
`test_hitter_task_*.py` 回归、静态/范围检查和显式路径提交。

- Base：`391f4036d9cdbb96493f06f7a8162454b33733da`
- Commit：`3a139e1623e0f0b7c11169aaa6df61d6d6f93cab`
- Subject：`docs: explain truthful HITTER ball diagnostics`
- 提交文件只有 `docs/hitter_task_observation_diagnostics.md`

本任务没有启动、停止或重启任何现场进程，没有启动 robot policy、R2 或
action/control 链路。

## 根因与文档修复

旧手册把球检测、estimator、production incoming、planner、ARM 和 task
observation 写成一条必然连续的阶段链，也把 pelvis 有效误写成所有诊断
推进的统一前提。这会让现场把“球诊断可以继续、生产门控故意阻塞”的真实组合
误判为整条链路卡死；旧页面还可能从 legacy stage/default 值形成假进度。

更新后的手册明确拆分：

- pelvis-independent 球诊断：BALL subject、ESTIMATOR、BALL-ONLY
  INCOMING；
- production-equivalent 完整门控：PELVIS、PRODUCTION INCOMING、
  PLANNER、ARMED、TASK OBS；
- schema v2 `live_snapshot` 是实时权威，`null` 显示 `—`，不再把未知值
  读成 0、PASS 或 COMPLETE；
- `ARM threshold` 是配置值，不是 live TTS。

## 文档变更

`docs/hitter_task_observation_diagnostics.md` 现在包含：

1. ChingMu bridge 的精确发布命令，显式使用当前 LCM URL、channel 和
   `--publish`；
2. diagnostics monitor 的精确 8766 命令，显式使用
   `--base-name G1Pelvis --ball-name ball --port 8766`；
3. 浏览器地址 `http://127.0.0.1:8766/`；
4. v1 页面 notice 的解释，以及只对旧 monitor 执行 `Ctrl-C` 后重跑完整
   8766 命令的重启步骤；
5. 各 subject、estimator、ball-only incoming 和 production gate 状态的
   真实含义；
6. pelvis 缺失时的精确页面组合；
7. 静止球与抛入 incoming 球各自应该和不应该推进的阶段；
8. 按 LCM ball rate/age、schema/version、estimator count、ball-only
   incoming、production blocker 排列的五步排障清单。

手册还明确了两个启动约束，避免把“pelvis-independent”误解为可以删除参数：

- ChingMu bridge 启动仍必须解析 `G1Pelvis` hierarchy，并读取 pelvis
  orientation calibration；独立性指运行中不需要持续收到有效 pelvis 位姿；
- monitor 即使只验球，启动时仍保留 `--pelvis-calib` 文件参数。

## 命令核验

只读执行：

```bash
/home/loco1/miniconda3/envs/rb/bin/python \
  deploy/mocap_bridge/chingmu_table_lcm_bridge.py --help
```

退出码为 0；当前 help 确认文档使用的
`--host`、`--base-subject`、`--table-calib`、
`--pelvis-orientation-calib`、`--lcm-url`、`--channel` 和
`--publish` 全部存在。bridge 默认 `publish=False`，因此手册显式保留
`--publish`。当前 CLI 没有独立禁用 pelvis 发布的 ball-only flag。

只读执行：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python \
  -m diagnostics.hitter_task_monitor --help
```

退出码为 0；当前 help 确认文档使用的
`--mimic-config`、`--control-config`、`--table-calib`、
`--pelvis-calib`、`--lcm-url`、`--channel`、`--base-name`、
`--ball-name` 和 `--port` 全部存在。monitor 默认端口仍为 8765，所以
现场 8766 命令必须显式写 `--port 8766`。服务代码固定绑定 loopback，
页面地址为 `http://127.0.0.1:8766/`。

文档命令引用的 mimic/control/table/pelvis calibration 文件在当前
checkout 中均存在。

## 完整回归

最终文档状态下执行 brief 指定的完整命令：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
/home/loco1/miniconda3/envs/rb/bin/python -m unittest discover \
  -s tests -p 'test_hitter_task_*.py'
```

退出码：`0`

完整输出：

```text
2026-07-29 14:41:43.503 | INFO     | utils.motion_lib.torch_humanoid_batch:<module>:41 - Using Humanoid Batch
......................s.....................................s.....................2026-07-29 14:41:58.807 | INFO     | simulator.real_world:_vicon_state_handler:349 - Vicon root information received!
2026-07-29 14:41:58.807 | INFO     | simulator.real_world:_vicon_state_handler:350 - Root Translation World: [1.5 0.  1. ]
......................................................................................................................................................................................
----------------------------------------------------------------------
Ran 264 tests in 25.700s

OK (skipped=2)
```

结果为 264 tests、0 failure、0 error、2 skipped。两个 skip 均为明确的
环境/人工验收 gate：

- 未设置 `HITTER_DIAGNOSTICS_PERF=1`，所以不自动执行 60 秒、360 Hz
  performance acceptance；
- 已安装 Chrome 在 10 秒内无法完成 localhost headless navigation，
  所以既有 frontend XSS navigation fixture skip。

没有 test-only correction。

## pelvis 缺失时的现场精确状态

球仍正常来球且 estimator/ball-only incoming 已完成时，页面应允许：

```text
BALL subject                  LIVE（rate/age/frame 持续刷新）
ESTIMATOR                     READY · 31/31
BALL-ONLY INCOMING            INCOMING · 3/3，confirmed TRUE
```

同一个 v2 snapshot 的生产门控应保持：

```text
PELVIS                        BLOCKED
PRODUCTION INCOMING           NOT_EVALUATED · —
PLANNER                       BLOCKED
planner reason                PELVIS_UNAVAILABLE
planner TTS                   —
ARMED                         NOT_EVALUATED
TASK OBS                      NOT_AVAILABLE · —
task observation clip count   —
current attempt blocker       PELVIS_UNAVAILABLE
```

`ARM threshold` 仍可显示当前配置 0.92 s；它不是 live planner/ARM TTS。

静止球可让 BALL rate/age/frame 刷新，并让 estimator 填满到 31/31；其
`vx` 接近 0 时，BALL-ONLY INCOMING 应为
`NOT_INCOMING · 0/3 / INCOMING_SPEED_REJECTED`。它不应仅因球可见而进入
planner ready、ARMED 或 task observation available。

有效的 table-world `-X` incoming 球应让 BALL-ONLY INCOMING 按
`1/3 → 2/3 → 3/3` 推进。没有 pelvis 时只推进球区；pelvis 同时有效时，
production incoming 才能继续到 CONFIRMED，planner 随后 READY 或给出真实
轨迹拒绝 reason。

## 静态与范围检查

执行：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2
git diff --check
git diff -- deploy/envs/hitter.py
git status --short
```

- `git diff --check`：退出码 0，无输出；
- 提交前 `git diff --cached --check`：退出码 0，无输出；
- cached scope：只有 `M docs/hitter_task_observation_diagnostics.md`；
- commit scope：只有 `M docs/hitter_task_observation_diagnostics.md`；
- 提交后无 staged change，其他大量 dirty-worktree 内容原样保留。

`deploy/envs/hitter.py` 在本任务开始前已经有用户改动。为区分“文件非空
diff”和“本任务没有触碰”，提交前后都对该 path 的 binary diff 取 SHA-256：

```text
e291775499b31a2173d446b9e1b297ff82b32990a512231204a08c0138e0c522
```

前后完全一致；且 commit 文件清单不含该 path。因此本任务没有修改
`deploy/envs/hitter.py`，也没有修改 production policy、planner、R2、
action 或 control 链路。

## 关注点

- 本任务没有操作 live process 或真机；现场仍需操作者按手册重启旧 monitor
  并执行静止球/来球验收。
- bridge 的“pelvis 可选”仅指有效运行时位姿可缺失；启动仍需能解析
  `G1Pelvis` hierarchy 和读取 pelvis orientation calibration。
- 上述两个环境 skip 保持明确，不影响其余 262 个实际执行测试通过。
