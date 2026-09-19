# Task 5 离线总验证与现场交接报告

日期：2026-07-24  
仓库：`/home/loco1/BOB/Hitter/RobotBridge2`  
验证提交：`6fede45ca1a029ec58fa35269a21775b811fc16b`

## 结论

**离线代码和自动化测试已完成；真实 orientation JSON 尚未生成，
需要操作者摆正机器人后执行现场标定命令。**

本轮没有连接 ChingMu，没有运行 live calibration，没有发布 LCM，也没有
调用 bridge `main()`/SDK。唯一写入的仓库内文件是本报告；该路径被
`.git/info/exclude` 中的 `.superpowers/` 规则忽略。

## 已阅读与验收边界

完整阅读：

- `.superpowers/sdd/task-5-brief.md`
- `docs/superpowers/specs/2026-07-24-chingmu-pelvis-orientation-calibration-design.md`
- `deploy/mocap_bridge/chingmu_table_lcm_bridge.py`
- `deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py`

设计的现场边界明确要求：

- 真实 `chingmu_pelvis_orientation_latest.json` 不得离线伪造。
- 必须由操作者确认 pelvis `+X/+Y/+Z` 对齐桌面世界
  `+X/+Y/+Z`、机器人静止且 policy output 未运行后现场采集。
- 在真实标定和正常运行现场验收完成前，不得宣称 live-calibrated 或真机
  `base_forward_xy` 已完成对齐。

## Fresh 离线验证

### 1. Focused 测试：22/22

命令：

```bash
PYTHONDONTWRITEBYTECODE=1 conda run --no-capture-output -n rb \
  python -m unittest \
  deploy.mocap_bridge.tests.test_chingmu_pelvis_orientation -v
```

退出码：`0`

关键输出：

```text
Ran 22 tests in 0.052s
OK
```

### 2. `py_compile`：2/2

为避免在当前脏工作树生成或改写 `__pycache__`，仍由标准库
`py_compile.compile(..., doraise=True)` 完成真实编译，但把两个 `.pyc`
输出放入自动清理的临时目录：

```bash
PYTHONDONTWRITEBYTECODE=1 conda run --no-capture-output -n rb python -c \
  'import py_compile, tempfile; from pathlib import Path; sources=("deploy/mocap_bridge/chingmu_table_lcm_bridge.py", "deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py"); tmp=tempfile.TemporaryDirectory(prefix="task5-pycompile-"); [py_compile.compile(source, cfile=str(Path(tmp.name) / (Path(source).name + ".pyc")), doraise=True) for source in sources]; print(f"py_compile: compiled {len(sources)} files")'
```

退出码：`0`

关键输出：

```text
py_compile: compiled 2 files
```

### 3. CLI help 三参数

命令：

```bash
set -o pipefail
PYTHONDONTWRITEBYTECODE=1 conda run --no-capture-output -n rb python \
  deploy/mocap_bridge/chingmu_table_lcm_bridge.py --help |
  rg -n -- \
  '--pelvis-orientation-calib|--save-pelvis-orientation-calib|--pelvis-calib-sec'
```

退出码：`0`

关键输出：

```text
--pelvis-orientation-calib PELVIS_ORIENTATION_CALIB
--save-pelvis-orientation-calib SAVE_PELVIS_ORIENTATION_CALIB
--pelvis-calib-sec PELVIS_CALIB_SEC
```

`--help` 由 `argparse` 提前退出，没有进入 `main()` 的 SDK 构造和连接部分。

### 4. 禁止的旧符号

命令：

```bash
rg -n \
  'pelvis_offset_heading_m|initial_base_yaw|relative_yaw|yaw_quaternion_from_rotation' \
  deploy/mocap_bridge/chingmu_table_lcm_bridge.py
```

`rg` 退出码：`1`，无输出，表示四个旧符号均无匹配；包装后的不变量检查
退出码为 `0`。

### 5. Diff whitespace

命令：

```bash
git diff --check -- \
  deploy/mocap_bridge/chingmu_table_lcm_bridge.py \
  deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py
```

退出码：`0`，无输出。

另对完整提交区间执行：

```bash
git diff --check d05ad32..HEAD -- \
  deploy/mocap_bridge/chingmu_table_lcm_bridge.py \
  deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py \
  deploy/mocap_bridge/tests/test_chingmu_table_lcm_bridge.py
```

退出码：`0`，无输出。

### 6. Legacy 全文件：25/25，工作树 `.D` 未恢复

测试源码直接读取
`HEAD:deploy/mocap_bridge/tests/test_chingmu_table_lcm_bridge.py`，通过 stdin
在内存中编译执行；逻辑 `__file__` 指向原仓库路径，未创建或恢复测试文件：

```bash
set -o pipefail
git show HEAD:deploy/mocap_bridge/tests/test_chingmu_table_lcm_bridge.py |
  PYTHONDONTWRITEBYTECODE=1 conda run --no-capture-output -n rb python -c \
  'import sys; from pathlib import Path; source=sys.stdin.read(); filename=str(Path.cwd() / "deploy/mocap_bridge/tests/test_chingmu_table_lcm_bridge.py"); globals()["__file__"]=filename; exec(compile(source, filename, "exec"), globals())'
```

退出码：`0`

关键输出：

```text
.........................
Ran 25 tests in 0.029s
OK
```

在正式命令前有一次 harness 试跑把代码执行在独立字典中；
`unittest.main()` 因而只看到真实 `__main__` 模块并报告 `Ran 0 tests`。
该次结果没有计入验收。随后改为在 `globals()` 中内存执行，得到以上
fresh `25/25` 结果。

复核：

```text
 D deploy/mocap_bridge/tests/__init__.py
 D deploy/mocap_bridge/tests/test_chingmu_table_lcm_bridge.py
deploy/mocap_bridge/tests/test_chingmu_table_lcm_bridge.py ABSENT_FROM_WORKTREE
deploy/mocap_bridge/tests/__init__.py ABSENT_FROM_WORKTREE
```

因此用户原有未暂存删除没有被恢复。

### 7. CLI mode 小型只读探针

探针只导入
`build_arg_parser`、`_operation_mode`、
`_load_runtime_pelvis_orientation` 和 `BridgeConfig`；没有调用 `main()`，
没有构造或启动 `ChingMuSdkClient`：

```text
calibration_publish_rejected=--save-pelvis-orientation-calib cannot be combined with --publish
runtime_without_orientation_rejected=--pelvis-orientation-calib is required outside table-only calibration mode
runtime_loader_without_orientation_rejected=--pelvis-orientation-calib is required for normal runtime
probe_result=PASS (main/SDK not called)
EXIT_CODE=0
```

结论：

- pelvis calibration mode 由 `_operation_mode` 明确禁止 `--publish`。
- 正常 runtime 未提供 orientation 文件参数时明确失败。
- 探针没有触发 SDK、网络连接或 LCM publisher。

## 提交与路径范围：`d05ad32..HEAD`

`d05ad32` 是 `HEAD` 祖先，检查退出码 `0`；区间共 `6` 个提交：

```text
655764f feat: add ChingMu pelvis orientation calibration math
df9cba0 test: align ChingMu bridge tests with absolute orientation
d24e2b feat: persist ChingMu pelvis orientation calibration
c3c5426 feat: apply calibrated ChingMu pelvis orientation
aab6d66 test: align legacy bridge tests with pelvis calibration
6fede45 feat: add ChingMu pelvis orientation calibration mode
```

逐提交 `git show --name-status` 与区间
`git diff --name-status d05ad32..HEAD` 只出现以下三条相关路径：

```text
M deploy/mocap_bridge/chingmu_table_lcm_bridge.py
A deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py
M deploy/mocap_bridge/tests/test_chingmu_table_lcm_bridge.py
```

其中两个 `test:` 提交只更新 legacy bridge 测试；其余功能提交只涉及
bridge 和 focused test。区间没有生产链路外的文件、没有 calibration
JSON、没有 planner/policy/MuJoCo/消息结构改动。

## Index、脏工作树与 JSON 状态

```bash
git diff --cached --quiet
git diff --cached --name-status
```

结果：

```text
git diff --cached --quiet: exit 0
git diff --cached --name-status: empty
```

index 为空。测试后完整 porcelain 状态共有 `63` 项，状态快照 SHA-256 为：

```text
c63271a5a98ca7642b4cd16a447a73beac73ee6a967258acb6e5980ac82d435e
```

报告文件被忽略，因此不出现在 porcelain 状态中。

真实 orientation 路径：

```text
deploy/mocap_bridge/calibrations/chingmu_pelvis_orientation_latest.json
```

只读检查结果：

- 当前文件不存在。
- `git ls-files --error-unmatch` 退出码 `1`：当前未跟踪。
- `git ls-tree -r --name-only HEAD | rg ...` 的 `rg` 退出码 `1`：`HEAD`
  未跟踪。
- `git diff --name-only d05ad32..HEAD | rg ...` 的 `rg` 退出码 `1`：
  本次提交区间没有该文件。
- `git log --all -- <path>` 输出为空：当前可达 refs 历史中没有该路径的
  提交。

## 现场命令路径预检

Task 5 现场命令引用的 table calibration：

```text
deploy/mocap_bridge/calibrations/chingmu_table_frame_latest.json
```

当前存在，是普通文件，大小 `1004 bytes`。这只证明命令引用路径存在；
本轮没有连接设备，也没有重新验证该文件对应当前真实桌面摆放。

## 现场交接命令（本轮未执行）

执行一次性标定前，操作者必须逐项确认：

```text
pelvis +X = table world +X
pelvis +Y = table world +Y
pelvis +Z = table world +Z
robot is stationary
policy output is not running
```

确认后才可运行：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2
conda run --no-capture-output -n rb python \
  deploy/mocap_bridge/chingmu_table_lcm_bridge.py \
  --host 192.168.2.100 \
  --base-subject G1Pelvis \
  --table-calib deploy/mocap_bridge/calibrations/chingmu_table_frame_latest.json \
  --save-pelvis-orientation-calib deploy/mocap_bridge/calibrations/chingmu_pelvis_orientation_latest.json \
  --pelvis-calib-sec 2.0
```

只有该命令通过有效样本、source duration、位置 RMS 和角度 RMS 门限并成功
生成真实 JSON 后，才可运行正常 bridge：

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2
conda run --no-capture-output -n rb python \
  deploy/mocap_bridge/chingmu_table_lcm_bridge.py \
  --host 192.168.2.100 \
  --base-subject G1Pelvis \
  --table-calib deploy/mocap_bridge/calibrations/chingmu_table_frame_latest.json \
  --pelvis-orientation-calib deploy/mocap_bridge/calibrations/chingmu_pelvis_orientation_latest.json \
  --publish
```

## Honest completion boundary

当前状态是 **offline-complete**，不是 **live-calibrated**。

真实 orientation JSON 尚未生成，现场标定与真机方向验收仍是明确 blocker。
在操作者完成摆正、策略停机确认、一次性采集，以及正常 bridge 的
roll/pitch/yaw、`base_forward_xy`、左右 `90 deg` 和重启一致性验收之前，
不能宣称真机朝向对齐已经完成。
