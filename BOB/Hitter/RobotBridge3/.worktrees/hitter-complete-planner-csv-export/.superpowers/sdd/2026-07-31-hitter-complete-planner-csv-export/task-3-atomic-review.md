# Task 3 原子性、安全与代码质量审查

## Verdict

**CHANGES REQUIRED**

提交 `a406253` 的正常路径、核心 CSV 重读校验和第二次 rename 失败后的基本回滚均可工作，但当前仍存在一个可删除源 recording 的高严重度路径，以及三个会破坏 fail-closed/事务状态可信度的问题。因此不能批准。

## Findings

### 高：`--output` 可以覆盖并删除源 `attempt_details`

- 位置：`deploy/diagnostics/export_hitter_task_csv.py:1205-1239`、`deploy/diagnostics/export_hitter_task_csv.py:2119-2131`、`deploy/diagnostics/export_hitter_task_csv.py:2156-2165`、`deploy/diagnostics/export_hitter_task_csv.py:2295-2299`
- 影响：路径检查只拒绝 output 等于 session 或作为 session 的祖先，没有拒绝 output 等于/包含实际源文件。合法调用
  `output_dir=session_dir / "attempt_details", overwrite=True`
  会在所有源检查完成后把整个 `attempt_details` 移到 backup、安装导出目录并删除 backup。命令返回成功，但选中和未选中的 attempt detail 源文件都被删除，违反“只读源 recording”和安全覆盖语义。
- 复现：使用现有 `build_complete_session_fixture()` 调用上述参数，函数返回 `completed_call_count == 2`；随后
  `attempt_details/1.json` 不存在，而 `attempt_details/export_manifest.json` 存在。
- 建议：在创建 parent/temp 之前构造所有 canonical source paths，并拒绝 output 等于或作为任一源文件/源目录的祖先；至少显式保护 `session.json`、`events.jsonl`、`ball_samples.csv` 和整个 `attempt_details`。新增覆盖源目录及其 `..`/symlink 别名的回归测试。`overwrite=True` 只能授权替换安全的输出目录，不能授权删除输入。

### 中：提交新 output 后的 fsync/backup cleanup 失败会返回错误，但目标已经改变

- 位置：`deploy/diagnostics/export_hitter_task_csv.py:2096-2116`、`deploy/diagnostics/export_hitter_task_csv.py:2295-2303`
- 影响：第二次 `os.replace()` 一旦成功，新 output 已可见。之后 `_fsync_directory()` 或 `shutil.rmtree(backup)` 抛错时，异常直接传播；外层 cleanup 只尝试删除已不存在的 temp，既不回滚也不把状态标记为“已提交”。CLI 因而返回非零，但目标目录已从旧版本变为新版本。自动化重试会观察到一个与失败返回值矛盾的状态；backup cleanup 失败时还会留下旧 backup（实际 `rmtree` 中途失败时可能只剩部分旧数据）。
- 复现：
  - mock `shutil.rmtree(backup)` 抛 `OSError("cleanup failed")`：调用抛错，但 `output/new` 已存在、`output/old` 已不存在，backup 中仍有 old。
  - mock 安装后的 `_fsync_directory()` 抛错：同样得到“异常 + 新 output 已安装 + 旧 output 在 backup”。
- 额外问题：安装失败后 restore rename 已成功、但 restore 后目录 fsync 失败时，`2104-2112` 报告 “backup preserved”，实际 backup 已被 rename 回 output，不再存在。
- 建议：明确 commit point。安装 rename 与 parent fsync 成功后，应把 backup 删除视为提交后的可恢复 housekeeping：清理失败不得谎报整个导出未提交，应返回成功并给出明确 warning/残留 backup，或返回结构化的 committed-with-cleanup-error 状态。安装后 fsync 失败则应走专门恢复状态机，并准确报告 output/backup 的真实位置；为 install-fsync、cleanup、restore-fsync 三个分支分别加故障注入测试。

### 中：`source_files_unchanged_during_export=true` 在末次检查后仍有竞态窗

- 位置：`deploy/diagnostics/export_hitter_task_csv.py:2134-2143`、`deploy/diagnostics/export_hitter_task_csv.py:2221-2228`、`deploy/diagnostics/export_hitter_task_csv.py:2237-2299`
- 影响：最后一次 stat/hash 检查发生在 manifest 构造、manifest 写入/重读、temp directory fsync 和 output rename 之前。源文件在 `2228` 之后变化不会再被检查，但导出仍成功，manifest 仍写入初始 hash，并把 `source_files_unchanged_during_export` 固定为 `true`。这直接破坏 manifest 的可审计性和文档承诺。
- 复现：mock `_write_manifest()`，在调用真实 writer 前修改 `session.json.status`。`export_session_csv()` 仍成功安装 output；当前 `session.json` SHA-256 与 manifest 不同，而 completeness check 仍为 `true`。
- 理由：源文件还会通过独立的路径式读取多次（例如 attempt detail 在 `2171` 和 `2177` 两次解析），所以首尾 hash 不等同于所有派生表都来自同一个不可变快照；有权限的并发写者还可在检查间恢复原内容。
- 建议：先将每个源以 no-follow、已验证 inode 的 descriptor 读入一个明确快照，并让解析、hash 和 manifest 全部基于同一快照；或者提供可靠的 source snapshot/锁协议。若继续采用末次检查，至少把它移动到安装前最后一步并在 manifest 写入后再检查，但要明确这仍不能消除 check-to-rename TOCTOU。

### 中：中间路径 symlink 未被拒绝，所有安全检查和 rename 仍是路径级 TOCTOU

- 位置：`deploy/diagnostics/export_hitter_task_csv.py:1211-1239`、`deploy/diagnostics/export_hitter_task_csv.py:2073-2099`
- 影响：代码只检查 output 最终分量和 immediate parent 本身是否为 symlink。路径中更早的 symlink 会被 `mkdir`、`resolve`、`mkdtemp`、`os.stat`、`os.replace` 和 `rmtree` 跟随，因此可以把 temp、backup、最终安装及覆盖删除重定向到别的目录。检查之后替换父路径分量还会造成 check/use 不一致和孤立 temp。
- 复现：令 `root/alias -> root/real`，使用
  `output=root/alias/nested/analysis`；`output.is_symlink()` 和
  `output.parent.is_symlink()` 均为 false，导出成功，实际文件写入
  `root/real/nested/analysis`。
- 建议：定义可信 anchor，逐级用 directory fd、`O_DIRECTORY|O_NOFOLLOW` 打开/创建组件，并使用 `renameat` 风格的 fd-relative 操作；至少遍历并拒绝所有既有 symlink 组件，并在 canonical 安全检查完成后才创建 parent。新增中间 symlink 和检查后父目录替换的测试。

### 低：安全关键分支缺少测试，重复多遍读取扩大了竞态和资源开销

- 位置：`deploy/tests/test_hitter_task_csv_export.py:1165-1414`、`deploy/diagnostics/export_hitter_task_csv.py:335-414`、`deploy/diagnostics/export_hitter_task_csv.py:849-960`、`deploy/diagnostics/export_hitter_task_csv.py:2161-2178`
- 影响：Task 3 目前只有六个端到端/CLI 测试。目录事务只覆盖“第二次 rename 失败、restore 成功”；没有覆盖 unsafe output、overwrite=false、non-dir/final/intermediate symlink、restore 再失败、fsync/cleanup、源变动竞态，以及 CSV repeated/duplicate header、ragged/duplicate key、event gap/watermark 的故障分支。实现还分别维护两套近似的 no-follow 读取/hash 逻辑，并把 attempt detail 解析两次；这既增加维护面和 I/O/内存，也制造更多 source snapshot 不一致窗口。
- 建议：把上述安全矩阵参数化测试；合并 descriptor 级读取/指纹逻辑，并让已加载的 detail 直接供 input alignment 使用。对于长 session，避免同时物化三张核心表的全部 row tuple。

## 已核对且未发现问题的项目

- `overwrite=False` 在已存在目标上会拒绝，replace 前也会二次检查。
- temp 与 backup 都在 output 同级创建；静态、无父路径竞态时属于同一文件系统。
- 第二次 install rename 失败、restore rename 成功时，旧 output 可恢复；install 和 restore 都失败时 backup 保留。
- 八张 CSV 使用固定 header 和 `newline=""`；输出文件逐个 fsync，关闭后用 `csv.reader` 重读，可检测 exact header、重复 header、ragged row、核心重复 key、typed key 及三表 key 顺序。
- event validation 对 event id 从 1 连续、最终 watermark 一致和 `RECORDER_EVENT_GAP` 采用 fail-closed。
- CLI help 含四个参数，运行异常返回非零。
- 未发现 Python 3.8 不兼容语法/API。

## 验证证据

基线：

```text
PYTHONPATH=. /home/loco1/miniconda3/envs/rb/bin/python -m unittest \
  tests.test_hitter_task_csv_export \
  tests.test_hitter_task_replay \
  tests.test_hitter_task_recording -v

Ran 85 tests in 2.276s
OK
```

另运行了临时目录内的只读故障注入，确认：

```text
unsafe source overlap: returned success, detail1_exists=False
backup cleanup failure: exception, new_visible=True, backup_old=True
post-check source mutation: returned success, manifest_hash_matches=False
intermediate symlink: returned success, real_written=True
install+restore rename failure: output absent, backup_old=True
```

审查过程中未修改生产源码或测试；唯一新增文件是本报告。
