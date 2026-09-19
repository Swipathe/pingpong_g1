### Task 3: 回归验证与现场日志检查

**Files:**
- Verify: `deploy/envs/hitter.py`
- Verify: `deploy/tests/test_hitter_strike_target_logging.py`
- Verify: `deploy/tests/test_mujoco_physical_table_tennis.py`

**Interfaces:**
- Consumes: Tasks 1-2 的最终代码。
- Produces: 可供最终 review 的测试、语法、diff 和日志格式证据。

- [ ] **Step 1: 运行新日志测试**

Run:

```bash
PYTHONPATH=deploy conda run --no-capture-output -n rb \
  python -m unittest discover \
  -s deploy/tests \
  -p 'test_hitter_strike_target_logging.py' \
  -v
```

Expected:

```text
Ran 5 tests
OK
```

- [ ] **Step 2: 运行当前仍存在的 HITTER/MuJoCo 相关测试**

Run:

```bash
PYTHONPATH=deploy conda run --no-capture-output -n rb \
  python -m unittest discover \
  -s deploy/tests \
  -p 'test_mujoco_physical_table_tennis.py' \
  -v
```

Expected baseline in the current dirty worktree:

```text
Ran 23 tests
FAILED (failures=1)
```

唯一已知既有失败：

```text
test_xml_defines_only_intended_ball_contact_pairs
```

原因是当前 XML pair friction 为
`0.20 0.20 0.005 0.0001 0.0001`，测试仍期望旧的
`0.20 0.005 0.0001`。验收条件是没有新增失败；不能删除、修改或回退
用户的 contact/测试改动来制造绿色结果。

- [ ] **Step 3: 运行语法和 whitespace 验证**

Run:

```bash
PYTHONPATH=deploy conda run --no-capture-output -n rb \
  python -m py_compile \
  deploy/envs/hitter.py \
  deploy/tests/test_hitter_strike_target_logging.py
git diff --check
```

Expected: 两条命令退出码均为 0。

- [ ] **Step 4: 验证日志字段只在 strike 边界出现**

Run:

```bash
rg -n \
  "_log_hitter_strike_target|HITTER strike target:|crossed_strike" \
  deploy/envs/hitter.py \
  deploy/tests/test_hitter_strike_target_logging.py
```

Expected:

- 生产代码只有一个 `HITTER strike target:` 格式定义；
- 生产代码只有一个 `_log_hitter_strike_target(previous_active)` 调用；
- 该调用位于 `_log_hitter_advance_transitions()` 的
  `if crossed_strike:` 内。

- [ ] **Step 5: 最终 dirty-worktree review**

Run:

```bash
git status --short --branch
git log -3 --oneline
git show --stat --oneline HEAD
git diff -- deploy/envs/hitter.py
```

Expected:

- 新测试文件和日志代码均已提交；
- 用户原有的其他修改、删除和未跟踪文件仍保留；
- 最终汇报明确列出本功能 commit、测试数量和任何既有失败；
- 不 push、不创建 PR，除非用户另外明确要求。
