# Task 3 报告：HITTER 每次击球目标日志回归验证

## 结论

- 验证目录：`/home/loco1/BOB/Hitter/RobotBridge2`
- 验证 HEAD：`479e97fbea33d9e7aba8cd1b533cdec78a4da5c7`
- 总体状态：符合 Task 3 验收条件。
- 新日志测试：`Ran 5 tests`，`OK`，退出码 `0`。
- MuJoCo 物理测试：`Ran 23 tests`，`FAILED (failures=1)`，退出码 `1`。
- 上述唯一失败是 brief 明确列出的既有 friction 期望不一致：
  `test_xml_defines_only_intended_ball_contact_pairs`。
- 新增失败：无。
- `py_compile` 与全工作树 `git diff --check`：均退出码 `0`。
- 生产代码中主日志格式定义恰好 1 处，strike 边界调用恰好 1 处；调用位于
  `_log_hitter_advance_transitions()` 的 `if crossed_strike:` 内。
- index 为空；验证前后 HEAD、porcelain 状态、tracked diff、index 指纹全部一致。
- 未修改、恢复、删除、暂存或提交任何业务文件；未执行 reset/restore/clean；
  未 push，未创建 PR。

## 命令与退出码总表

| 序号 | 命令 | 退出码 | 结果 |
|---|---|---:|---|
| 1 | 新日志 unittest discover | 0 | 5/5 通过 |
| 2 | MuJoCo 物理 unittest discover | 1 | 23 项中 1 个既有失败，无新增失败 |
| 3 | `python -m py_compile ...` | 0 | 无输出 |
| 4 | `git diff --check` | 0 | 无输出 |
| 5 | brief 指定的 `rg -n ...` | 0 | 命中日志定义、测试引用和 strike 边界 |
| 6 | `git status --short --branch` | 0 | dirty worktree 保留 |
| 7 | `git log -3 --oneline` | 0 | HEAD 为 `479e97f` |
| 8 | `git show --stat --oneline HEAD` | 0 | HEAD 仅含日志 hook 与日志测试 |
| 9 | `git diff -- deploy/envs/hitter.py` | 0 | 仅显示用户原有未暂存 observation 改动 |

## 工作树指纹

验证前：

```text
HEAD
479e97fbea33d9e7aba8cd1b533cdec78a4da5c7

git status --porcelain=v1 -z | sha256sum
7dd3da2bf7631264b302579092980f701f3d4087018a97544e306ebe9cdbdb46  -

git diff --binary | sha256sum
f1e427d19b376515dd31c07e93fb17e8267244e9cb91ed159dbb9e05c08bb923  -

git diff --cached --binary | sha256sum
e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855  -
```

验证后：

```text
HEAD
479e97fbea33d9e7aba8cd1b533cdec78a4da5c7

git status --porcelain=v1 -z | sha256sum
7dd3da2bf7631264b302579092980f701f3d4087018a97544e306ebe9cdbdb46  -

git diff --binary | sha256sum
f1e427d19b376515dd31c07e93fb17e8267244e9cb91ed159dbb9e05c08bb923  -

git diff --cached --binary | sha256sum
e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855  -
```

四项前后完全一致。最终 porcelain 分类统计：

```text
staged=0 unstaged_modified=11 unstaged_deleted=32 untracked=13 other=0
```

`git diff --cached --name-only | wc -l` 的输出为 `0`；
`git diff --cached --quiet` 退出码为 `0`。

## Step 1：新日志测试

命令：

```bash
PYTHONPATH=deploy conda run --no-capture-output -n rb \
  python -m unittest discover \
  -s deploy/tests \
  -p 'test_hitter_strike_target_logging.py' \
  -v
```

退出码：`0`。

实际测试项：

```text
test_late_advance_logs_latest_override_exactly_once ... ok
test_non_strike_transition_does_not_log_target ... ok
test_invalid_ball_out_only_warns_and_does_not_change_main_validation ... ok
test_logs_all_world_velocity_vectors_and_norms ... ok
test_missing_active_result_only_warns ... ok

----------------------------------------------------------------------
Ran 5 tests in 0.005s

OK
```

测试实际打印的完整主日志样例：

```text
HITTER strike target: epoch=27 generation=8103 type=backhand v_ball_in_w_mps=[-3.0000,0.4000,-1.2000] speed_ball_in_mps=3.2558 v_ball_out_w_mps=[4.2708,-0.5000,1.8000] speed_ball_out_mps=4.6615 v_racket_target_w_mps=[1.4000,-0.1000,0.7000] speed_racket_mps=1.5684
```

该样例同时覆盖 epoch、generation、击球类型、入球/出球/球拍目标世界系速度向量和三个模长。

## Step 2：MuJoCo 物理测试

命令：

```bash
PYTHONPATH=deploy conda run --no-capture-output -n rb \
  python -m unittest discover \
  -s deploy/tests \
  -p 'test_mujoco_physical_table_tennis.py' \
  -v
```

退出码：`1`。

实际结果：

```text
Ran 23 tests in 2.195s

FAILED (failures=1)
```

22 项通过，唯一失败：

```text
FAIL: test_xml_defines_only_intended_ball_contact_pairs
      (test_mujoco_physical_table_tennis.PurePhysicalContactTests)

Traceback (most recent call last):
  File "/home/loco1/BOB/Hitter/RobotBridge2/deploy/tests/test_mujoco_physical_table_tennis.py", line 270, in test_xml_defines_only_intended_ball_contact_pairs
    self.assertEqual(pair.attrib.get(attribute), expected_value)
AssertionError: '0.20 0.20 0.005 0.0001 0.0001' != '0.20 0.005 0.0001'
- 0.20 0.20 0.005 0.0001 0.0001
?     -----      -------
+ 0.20 0.005 0.0001
```

这与 brief 的已知基线完全一致：XML 使用 5 值 pair friction，当前未跟踪测试仍期望旧 3 值。
未出现第二个失败或任何 error，因此新增失败数为 `0`。本轮没有为了全绿修改或回退 XML/测试。

## Step 3：语法与 whitespace

命令：

```bash
PYTHONPATH=deploy conda run --no-capture-output -n rb \
  python -m py_compile \
  deploy/envs/hitter.py \
  deploy/tests/test_hitter_strike_target_logging.py
```

退出码：`0`；无输出。

命令：

```bash
git diff --check
```

退出码：`0`；无输出。

## Step 4：strike 边界静态检查

命令：

```bash
rg -n \
  "_log_hitter_strike_target|HITTER strike target:|crossed_strike" \
  deploy/envs/hitter.py \
  deploy/tests/test_hitter_strike_target_logging.py
```

退出码：`0`。实际输出：

```text
deploy/tests/test_hitter_strike_target_logging.py:121:            env._log_hitter_strike_target(result)
deploy/tests/test_hitter_strike_target_logging.py:126:            if message.startswith("HITTER strike target:")
deploy/tests/test_hitter_strike_target_logging.py:171:            env._log_hitter_strike_target(
deploy/tests/test_hitter_strike_target_logging.py:177:                message.startswith("HITTER strike target:")
deploy/tests/test_hitter_strike_target_logging.py:185:                "Failed to log HITTER strike target:"
deploy/tests/test_hitter_strike_target_logging.py:195:            env._log_hitter_strike_target(None)
deploy/tests/test_hitter_strike_target_logging.py:201:                "Failed to log HITTER strike target:"
deploy/tests/test_hitter_strike_target_logging.py:260:            if message.startswith("HITTER strike target:")
deploy/tests/test_hitter_strike_target_logging.py:290:                message.startswith("HITTER strike target:")
deploy/envs/hitter.py:663:    def _log_hitter_strike_target(
deploy/envs/hitter.py:680:                "HITTER strike target: epoch={} generation={} type={} "
deploy/envs/hitter.py:703:                    "Failed to log HITTER strike target: {}",
deploy/envs/hitter.py:943:        crossed_strike = bool(
deploy/envs/hitter.py:948:        if crossed_strike:
deploy/envs/hitter.py:949:            self._log_hitter_strike_target(previous_active)
deploy/envs/hitter.py:1023:        crossed_strike = (
deploy/envs/hitter.py:1027:        if crossed_strike and getattr(self.simulator, "is_real", False):
```

补充精确计数命令的实际输出：

```text
$ rg -c '^[[:space:]]+"HITTER strike target:' deploy/envs/hitter.py
1
$ rg -c 'self\._log_hitter_strike_target\(previous_active\)' deploy/envs/hitter.py
1
```

调用上下文：

```text
934    def _log_hitter_advance_transitions(
...
943        crossed_strike = bool(
944            previous_phase == CommandPhase.ARMED
945            and previous_active is not None
946            and now >= previous_active.strike_deadline_monotonic_s
947        )
948        if crossed_strike:
949            self._log_hitter_strike_target(previous_active)
```

因此第二处 `crossed_strike`（real backend 的后续事件处理）不包含该日志调用；
目标日志只在 lifecycle strike 边界打印一次。

## Step 5：最终 dirty-worktree review

### `git status --short --branch`

退出码：`0`。实际输出：

```text
## main...jianghe20260717/main [ahead 17]
 ? chingmu_sdk
 M deploy/MUJOCO_LOG.TXT
 M deploy/agents/hitter_agent.py
 M deploy/config/hitter.yaml
 M deploy/config/mimic/hitter.yaml
 M deploy/envs/base_env.py
 M deploy/envs/hitter.py
 M deploy/mocap_bridge/calibrations/chingmu_table_frame_latest.json
 D deploy/mocap_bridge/monitor_hitter_prediction.py
 D deploy/mocap_bridge/record_vicon_ball.py
 D deploy/mocap_bridge/tests/__init__.py
 D deploy/mocap_bridge/tests/test_chingmu_sdk_client.py
 D deploy/mocap_bridge/tests/test_chingmu_table_lcm_bridge.py
 D deploy/mocap_bridge/tests/test_monitor_hitter_prediction.py
 D deploy/mocap_bridge/tests/test_monitor_vicon_lcm.py
 D deploy/mocap_bridge/tests/test_vicon_table_lcm_bridge.cpp
 M deploy/mocap_bridge/vicon_table_lcm_bridge.cpp
 M deploy/simulator/real_world.py
 D deploy/tests/__init__.py
 D deploy/tests/test_hitter_agent_timing.py
 D deploy/tests/test_hitter_env_lifecycle.py
 D deploy/tests/test_hitter_geometry_config.py
 D deploy/tests/test_hitter_planner_boundaries.py
 D deploy/tests/test_hitter_realtime.py
 D deploy/tests/test_hitter_recorded_replay.py
 D deploy/tests/test_real_world_hitter_snapshots.py
 D deploy/tests/test_record_vicon_ball.py
 M deploy/utils/hitter_planner.py
 D docs/hitter_robotbridge_integration.md
 D docs/superpowers/plans/2026-07-12-chingmu-realtime-planner-policy.md
 D docs/superpowers/plans/2026-07-13-chingmu-pelvis-origin-offset.md
 D docs/superpowers/plans/2026-07-14-vicon-tracer-datastream-switch.md
 D docs/superpowers/plans/2026-07-15-hitter-next-incoming-epoch.md
 D docs/superpowers/plans/2026-07-15-stable-incoming-track-confirmation.md
 D docs/superpowers/plans/2026-07-15-vicon-ball-recorder.md
 D docs/superpowers/specs/2026-07-10-mujoco-planner-physics-parity-design.md
 D docs/superpowers/specs/2026-07-11-chingmu-table-calibration-design.md
 D docs/superpowers/specs/2026-07-12-chingmu-realtime-planner-policy-design.md
 D docs/superpowers/specs/2026-07-13-chingmu-pelvis-origin-offset-design.md
 D docs/superpowers/specs/2026-07-14-vicon-tracer-datastream-switch-design.md
 D docs/superpowers/specs/2026-07-15-hitter-next-incoming-epoch-design.md
 D docs/superpowers/specs/2026-07-15-stable-incoming-track-confirmation-design.md
 D docs/superpowers/specs/2026-07-16-vicon-handler-timing-design.md
?? "deploy/mocap_bridge (copy)/"
?? deploy/mocap_bridge/bin/
?? deploy/mocap_bridge/calibrations/chingmu_pelvis_orientation_20260725_candidate.json
?? deploy/mocap_bridge/calibrations/chingmu_pelvis_orientation_latest.json
?? deploy/mocap_bridge/calibrations/chingmu_table_frame_20260725_candidate.json
?? deploy/mocap_bridge/calibrations/chingmu_table_frame_latest.before_20260725_promotion.json
?? deploy/mocap_bridge/calibrations/table_frame_latest.20260709_114145.json
?? deploy/robotbridge2_model7900_20260725.log
?? deploy/robotbridge2_model8000_20260725.log
?? deploy/tests/test_mujoco_physical_table_tennis.py
?? deploy/tests/test_real_world_connection_wait.py
?? recordings/
?? tracker_log.txt
```

该状态与验证前的 porcelain 指纹一致。`deploy/tests/test_hitter_strike_target_logging.py`
未出现在 dirty 列表中，说明该新测试文件已提交；所有列出的用户原有修改、删除和未跟踪路径仍在。

### `git log -3 --oneline`

退出码：`0`。实际输出：

```text
479e97f feat: log final HITTER strike target once
c65a543 feat: add HITTER strike target log payload
fa7d3c0 docs: plan per-strike target logging
```

本功能代码提交为：

```text
c65a543 feat: add HITTER strike target log payload
479e97f feat: log final HITTER strike target once
```

### `git show --stat --oneline HEAD`

退出码：`0`。实际输出：

```text
479e97f feat: log final HITTER strike target once
 deploy/envs/hitter.py                             |  1 +
 deploy/tests/test_hitter_strike_target_logging.py | 88 +++++++++++++++++++++++
 2 files changed, 89 insertions(+)
```

补充核对前一功能提交（退出码 `0`）：

```text
c65a543 feat: add HITTER strike target log payload
 deploy/envs/hitter.py                             |  52 ++++++
 deploy/tests/test_hitter_strike_target_logging.py | 205 ++++++++++++++++++++++
 2 files changed, 257 insertions(+)
```

### `git diff -- deploy/envs/hitter.py`

退出码：`0`。该 diff 只包含用户已有、未暂存的 observation 维度调整：

```diff
@@ -1144,7 +1144,10 @@ class HitterEnv(BaseEnv):
-            obs_base_target_pos = self._hitter_base_target_pos_b(robot_anchor_pos_w, robot_anchor_quat_w)
+            obs_base_target_xy = self._hitter_base_target_pos_b(
+                robot_anchor_pos_w,
+                robot_anchor_quat_w,
+            )[:2]
@@ -1156,10 +1159,10 @@ class HitterEnv(BaseEnv):
-            obs_base_target_pos = self._hitter_waiting_base_target_pos_b(
+            obs_base_target_xy = self._hitter_waiting_base_target_pos_b(
                 robot_anchor_pos_w,
                 robot_anchor_quat_w,
-            )
+            )[:2]
@@ -1179,7 +1182,7 @@ class HitterEnv(BaseEnv):
-                obs_base_target_pos,
+                obs_base_target_xy,
@@ -1189,10 +1192,8 @@ class HitterEnv(BaseEnv):
-        print(f"obs_racket_target_pos: {obs_racket_target_pos}, obs_racket_target_vel: {obs_racket_target_vel}")
-        print(f"obs_time_to_strike: {obs_time_to_strike}")
-        if obs.size != 105:
-            raise RuntimeError(f"HITTER policy expects 105 observation values, assembled {obs.size}.")
+        if obs.size != 104:
+            raise RuntimeError(f"HITTER policy expects 104 observation values, assembled {obs.size}.")
```

该未暂存 diff 没有触碰本功能的日志 helper、日志格式或 strike 边界调用。

## Concerns

- 唯一非绿色项是已知既有 friction 断言不一致；它不是本日志功能的回归，
  且本轮没有修改/回退 XML 或未跟踪测试来掩盖该失败。
- 仓库仍是用户的混合 dirty worktree；这正是本任务要求保留的现场状态。
- `.superpowers/sdd/task-3-report.md` 是本任务唯一有意写入的报告路径，
  且该路径不进入 Git index。
