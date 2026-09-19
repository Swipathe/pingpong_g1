# HITTER Complete Planner CSV Export 独立最终审查

## 审查范围与结论

审查对象：

```text
base = ebc14fb7959f8314ea7122713702f3f5401d3076
head = 26f31c7fffe72913564ac4482bdd74ce91b4edee
```

**Verdict：APPROVED。**

本次独立审查未发现仍然开放的 Critical、Important 或 Minor
correctness / fail-open finding。最终候选可以集成。

真实 session 的 Attempt 1/2 已从完整磁盘事实源成功导出。独立脚本未导入
导出器，直接重读源文件与最终 CSV，确认：

- `submitted / pending-replaced / actual completed = 2625 / 540 / 2085`
- Attempt 1 / 2 actual completed 分别为 `837 / 1248`
- `planner_inputs.csv`、`planner_calls.csv`、`planner_results.csv`
  各 `2085` 行，复合 key 序列相同且唯一
- core key SHA-256 为
  `83a6f8c272520d35de54bbc4b002e01ab8b50ed236d960a5127fb0ff557c747d`
- Attempt 2 generation `11192` 存在
- `raw / stage / policy / task = 33894 / 2102 / 1578 / 2`
- 410183 个 event id 连续，末端等于 session watermark
- 完整 transition、detail 冻结窗口、segment、raw interval、policy tick
  和最终 task observation 均对账通过
- 导出前后五个源文件 SHA-256 不变

## Findings

### Critical

无。

### Important

无开放项。

### Minor

无开放项。

## 本轮已关闭的问题

以下问题均曾由独立审查构造复现，最终候选已修复并增加回归：

1. `source_frame` 经 `int()` 截断后可能把不同 frame 静默视为相同；
2. task observation 曾只相信 attempt detail，未与完整 lifecycle 事件核对；
3. `DIAGNOSTIC_EVENT_DROPPED` 曾未触发 fail closed；
4. detail 曾被错误要求等于整个 full timeline 的最终 suffix，合法的
   detail 持久化后晚到 transition 会被误拒；
5. selected attempt 的 interval 外 lifecycle event 曾可绕过 scope 校验；
6. 关闭后的旧 binding tick 曾覆盖或冲突已经冻结的 task observation；
7. close signal 到 detail 真正冻结之间仍可合法更新 task observation，
   只有 pre-close 对比会误拒；
8. lifecycle event id 与 monotonic 时间在 `ATTEMPT_CLOSED` 两侧方向相反
   的 malformed log 曾未明确拒绝。

最终实现用 detail 的 bounded transition window 唯一定位冻结 prefix，
并将真实 transition event id 与 normalized timeline 保持一一对应。
task observation 以唯一 `ATTEMPT_CLOSED` event id 分割关闭前后，同时校验
event 顺序与 monotonic 方向一致。关闭前最后值优先；否则只允许 detail
匹配关闭后真实出现的 observation，从而兼容 close-to-freeze 并发窗口，
又不会回退选择更早的历史值。

独立定向复现确认：

- 合法 `close -> post-close task obs -> detail freeze -> late transition`
  场景成功导出；
- task CSV 使用冻结的 `99.0 / clip_count=9`；
- late transition 进入完整 stage timeline；
- 当 detail 的冻结 segment 包含该窗口时，相关 raw 和 policy tick 均保留；
- event-id/time 两种反序形状都在安装输出前 fail closed，且不留下输出目录。

## Fresh verification

在 clean `26f31c7` 上执行：

```text
unittest export + replay + recording    Ran 115 tests; OK
Python 3.8 py_compile                   PASS
git diff --check base..head             PASS
worktree implementation status         clean
```

其中关键 race/fail-closed 定向测试单独复跑通过：

```text
post-close observation is frozen snapshot       PASS
event-id / monotonic close ordering              PASS
late transition after persisted detail           PASS
post-close observation does not rewrite snapshot PASS
DIAGNOSTIC_EVENT_DROPPED                          PASS
```

另一个独立 probe 在最终边界实现上执行 53 个 exporter 测试、合法组合 race
和双向 malformed inversion，均通过。它还扫描了本机 13 个真实
`events.jsonl`：共 10,272,762 个事件、301 个 close，没有发现 selected
lifecycle tick 与 close 的 event-order/monotonic 方向反转。

## 真实 session 输出

最终候选导出到临时目录：

```text
/tmp/hitter-final-37b37889.jRmrDP/export-26f31c7
```

结果：

```text
events                    410183
submitted                    2625
pending                       540
completed                    2085
summary.csv                      2
raw_lcm.csv                  33894
planner_inputs.csv            2085
planner_calls.csv             2085
planner_results.csv           2085
stage_timeline.csv            2102
policy_ticks.csv              1578
task_observations.csv             2
generation 11192              present
```

最终八张 CSV 与前一已独立对账候选逐字节相同。manifest 的 completeness
checks 全部为 true，包括：

```text
no_diagnostic_event_drop
attempt_detail_timeline_prefix_window_verified
segments_rebuilt_from_full_transitions
task_observations_from_full_events
task_observation_detail_found_in_full_events
null_task_observation_preclose_events_absent
```

导出后源文件 SHA-256：

```text
session.json             fa3541b2575a66f2b672b37870057e682a835a21b84eee453b827d7b57f1de8e
events.jsonl             5d3f8c9091296be88789b768d2eac9de3d60aa905f26047b5c2189c6a0cdac3a
ball_samples.csv         1dbf2cf98a20b3fbafa7c2b79c35d20d254f6f12499e7a7eda64d63bec682f19
attempt_details/1.json   9143804af8570abe1878beebe1f9d317e1a6a94e9f2aca7bef7f888ee10545c0
attempt_details/2.json   4af0064ec61c229ca855d4bcfc0e470b269f12e3808a772bb4dc3dd281f3fb83
```

## 已披露的剩余协议限制

这些是文档已经明确披露的协议边界，不构成本次阻塞 finding：

- attempt detail 尚未持久化精确 capture event-id watermark，因此
  close-to-freeze 窗口只能证明 detail 值确实出现在关闭后的完整事件中，
  不能再精确缩到某一个事件；
- path-based symlink 检查与 rename 之间仍有并发 TOCTOU，未使用锁定的
  directory fd / `renameat`；
- 只靠 raw `input_seq` 连续性不能证明文件尾部未被整体截断，录制协议没有
  持久化 raw row count 或末端 input-seq watermark。

这些限制均未影响本次真实 Attempt 1/2 的完整恢复与逐行对账。

## 最终判断

`26f31c7fffe72913564ac4482bdd74ce91b4edee` 满足当前设计和交付判定：
完整 actual-completed planner 调用没有被 latest-only 替换或内存上限截断，
辅助 CSV 使用完整磁盘事件重建，损坏与边界冲突采用 fail closed，真实源文件
保持不变。独立最终结论为 **APPROVED**。
