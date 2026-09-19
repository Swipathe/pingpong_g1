# Task 3 原子性修复独立复审

## Verdict

**APPROVED**

`931ce6d` 相对 `a406253` 已关闭原 atomic review 中的阻塞性问题。本轮未发现新的 blocking finding。

## 逐项结论

### 1. output/source overlap

- `_prepare_output_path()` 在创建父目录前后都以绝对规范化路径做双向包含检查。
- output 等于 session、包含 session、位于 session 内部（包括 `attempt_details` 或源文件路径）均在创建临时目录和替换前拒绝。
- 独立注入覆盖了 session 本身、session 祖先、`attempt_details`、`session.json` 和带 `..` 的词法别名，共 5 个 case；全部拒绝，所有源文件字节保持不变。

当前策略比原 Task 3 brief 中临时 fixture 使用的 `session/analysis` 更严格，但这是已在运行文档中声明的 fail-closed 接口收紧；设计中的真实 CLI 输出位于源 session 之外，因此不阻塞既定交付目标。

### 2. 既有 symlink 组件

- `_reject_existing_symlink_components()` 从根开始逐级使用 `lstat`，最终分量、直接父级、更早祖先和 broken symlink 都会拒绝。
- 检查发生在父目录创建前、创建后以及最终目录替换前。
- 独立 4-case 矩阵确认上述四类 symlink 全部拒绝，真实目标目录没有被写入。

### 3. install fsync 失败

- 无旧 output：新目录被从 output 撤回 temporary；第二次 parent fsync 后返回失败，外层可安全清理 temporary。
- 有旧 output：新目录被隔离到唯一 `failed-new`，旧 backup 恢复到 output，并再次 fsync；调用返回失败。
- 独立注入确认旧 sentinel 恢复、新目录位于 `failed-new`、backup 已被恢复操作消费。
- 额外注入 restore fsync 也失败的分支，错误准确报告 output、`failed-new` 和已消费 backup 的状态。

### 4. backup cleanup 失败

- 安装 rename 与 parent fsync 成功后即达到 commit point。
- backup `rmtree` 失败或删除后的 parent fsync 失败只产生 `RuntimeWarning`，函数不把已提交的新 output 报告为失败。
- 独立注入覆盖 cleanup `rmtree` 和 cleanup fsync 两种错误；两者都保持新 output 可用，且返回语义与实际状态一致。

### 5. restore fsync 错误

- install rename 失败、旧 output rename 恢复成功但 restore fsync 失败时，错误明确说明旧 output 已恢复且 backup 已被消费，不再错误声称 “backup preserved”。
- install fsync 失败后的恢复 fsync 错误也准确报告旧 output、`failed-new` 和 backup 状态。
- 无旧 output 的 withdraw fsync 错误准确说明新目录已移回 temporary，但回滚持久化失败。

### 6. manifest 写入期间 source mutation

- 最终 source stat/SHA-256 检查已移动到 manifest 写入及 round-trip、manifest fsync、temporary directory fsync 之后，并紧邻 `replace_output_directory()`。
- 在真实 `_write_manifest()` 完成时修改 `session.json` 的注入会在 replace 前抛出 `source file changed during export`。
- 已有 output 的 sentinel 和目录内容保持不变，临时输出未安装。

## 残余边界判断

以下两项保留为 **nonblocking known limitations**：

1. 最终 source hash 检查与 rename 之间仍有窄小的 check-to-rename 窗口，且基于路径的多次读取不能识别“修改后恢复成相同内容”的并发写者。
2. symlink/component 检查与路径式 `os.replace()` 之间仍有路径 TOCTOU；有权限的并发进程可能在最后检查后替换路径分量。

它们不阻塞本提交，理由是：

- 静态 overlap 和所有静态既有 symlink 场景已经 fail closed；
- 正常输入是已标记 `COMPLETE` 的不可变 recording，残余问题需要并发或对抗性路径修改；
- 最终检查已尽可能靠近 commit point；
- 修复报告和运行文档明确记录了路径式 TOCTOU，修复报告也明确记录 source check-to-rename/多次路径读取边界；
- 完全消除需要锁定的 directory fd/`renameat` 以及 source snapshot/锁协议，属于超出本次局部修复的录制与安装协议升级。

## 验证证据

指定回归：

```text
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest \
  tests.test_hitter_task_csv_export \
  tests.test_hitter_task_replay \
  tests.test_hitter_task_recording -v

Ran 95 tests in 2.367s
OK
```

原 finding 对应的仓库测试：

```text
Ran 7 tests in 0.035s
OK
```

独立临时目录故障注入：

```text
OVERLAP_MATRIX=5 rejected; source bytes unchanged
SYMLINK_COMPONENT_MATRIX=4 rejected
CLEANUP_FSYNC_FAILURE=warning-only; new output valid
INSTALL_AND_RESTORE_FSYNC_FAILURE=state/message accurate
WITHDRAW_FSYNC_FAILURE=state/message accurate
```

其他检查：

```text
Python 3.8 py_compile: PASS
git diff --check a406253..931ce6d: PASS
```

本轮未修改生产源码、测试或真实 recording；仅新增本复审报告。
