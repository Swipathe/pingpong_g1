# RobotBridge4 shared-module ownership report

Date: 2026-09-09

Pre-structure commit: `f558b7c`

Workspace: `/home/yhl/Desktop/RobotBridge4_refactor`

## Result

Task 3 moved the four shared JSON definitions from diagnostics to
`deploy/utils/hitter_serialization.py` and moved the frozen `MocapFrame` data class from the ChingMu client to
`deploy/mocap_bridge/mocap_types.py`. Every active first-party consumer and test now imports the moved symbol from its
canonical module. Diagnostic models and the ChingMu client import only the moved definitions they use internally; no
stub, compatibility alias, explicit re-export, fallback, C++ change, or unrelated behavior change was added.

The five moved AST definitions exactly match their definitions in immutable pre-structure commit `f558b7c`. After
imports and those removed definitions are excluded, the AST of all 13 edited existing Python files also exactly matches
`f558b7c`.

## Ownership and consumers

| Canonical module | Definition | Direct active consumers |
| --- | --- | --- |
| `utils.hitter_serialization` | `JsonScalar` | Type alias used by `JsonValue`; no separate consumer |
| `utils.hitter_serialization` | `JsonValue` | `diagnostics.hitter_task_models`, `hitter_task_attempts`, `hitter_task_events`, `hitter_task_pipeline`, `hitter_task_recording`, `hitter_task_replay`; `utils.hitter_realtime` |
| `utils.hitter_serialization` | `freeze_json_value` | `diagnostics.hitter_task_models`, `hitter_task_events`, `hitter_task_recording`, `hitter_task_replay`; `utils.hitter_realtime`; `tests/test_hitter_task_attempts.py`, `tests/test_hitter_serialization.py` |
| `utils.hitter_serialization` | `to_builtin_json` | `diagnostics.hitter_task_models`, `hitter_task_events`, `hitter_task_monitor`, `hitter_task_recording`, `hitter_task_replay`; `tests/test_hitter_task_attempts.py`, `tests/test_hitter_serialization.py` |
| `deploy.mocap_bridge.mocap_types` | `MocapFrame` | ChingMu and Vicon SDK clients; ChingMu pelvis-orientation and Vicon table-bridge tests |

An AST scan of every active Python file under `deploy` found no import of these serialization symbols from
`hitter_task_models` and no import of `MocapFrame` from `chingmu_sdk_client`. The historical
`deploy/mocap_bridge (copy)` directory was excluded because Task 4 owns its reference audit and archive decision.

## Focused tests and red-green evidence

`deploy/tests/test_hitter_serialization.py` verifies that freezing detaches nested lists, mappings, and NumPy arrays;
converts non-finite floats to the fixed JSON tokens; makes nested containers immutable; and produces fresh built-in
dict/list containers on every conversion. `deploy/tests/test_shared_module_isolation.py` uses fresh subprocesses with
the repository root as `cwd` and explicit repository entries in `PYTHONPATH`. It verifies that importing shared
serialization and `utils.hitter_realtime` does not load diagnostics, and that importing the shared frame and Vicon
client does not load the ChingMu SDK client. The frame subprocess also verifies the `body_pose_source_time_s=None`
default and frozen data-class behavior. The helper preserves the caller's `PYTHONPATH`; it contains no temporary or
machine-specific dependency path.

Before either canonical module existed, the following command was run:

```bash
PYTHONPATH=/tmp/rb4-refactor-tools:/home/yhl/Desktop/RobotBridge4_refactor/deploy:/home/yhl/Desktop/RobotBridge4_refactor \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MUJOCO_GL=egl \
/home/yhl/miniforge3/envs/isaaclab/bin/python -m pytest \
  --assert=plain --tb=native -q \
  deploy/tests/test_hitter_serialization.py \
  deploy/tests/test_shared_module_isolation.py
```

Red outcome: four failures, all caused by the expected missing canonical modules:
`utils.hitter_serialization` in three tests and `deploy.mocap_bridge.mocap_types` in one test.

After extraction, direct import updates, and removal of the temporary-overlay literal from test code, the same command
reported:

```text
4 passed in 0.20s
```

The relevant mocap frame/client/bridge command was:

```bash
PYTHONPATH=/tmp/rb4-refactor-tools:/home/yhl/Desktop/RobotBridge4_refactor/deploy:/home/yhl/Desktop/RobotBridge4_refactor \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MUJOCO_GL=egl \
/home/yhl/miniforge3/envs/isaaclab/bin/python -m pytest \
  --assert=plain --tb=native -q \
  deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py \
  deploy/mocap_bridge/tests/test_vicon_sdk_client.py \
  deploy/mocap_bridge/tests/test_vicon_table_lcm_bridge.py
```

Outcome: 53 passed and 59 subtests passed in 0.58 seconds.

The broader changed-consumer diagnostic command was:

```bash
PYTHONPATH=/tmp/rb4-refactor-tools:/home/yhl/Desktop/RobotBridge4_refactor/deploy:/home/yhl/Desktop/RobotBridge4_refactor \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MUJOCO_GL=egl \
/home/yhl/miniforge3/envs/isaaclab/bin/python -m pytest \
  --assert=plain --tb=native -q \
  --junitxml=/tmp/robotbridge4-structure-focused.xml \
  deploy/tests/test_hitter_serialization.py \
  deploy/tests/test_shared_module_isolation.py \
  deploy/tests/test_hitter_task_attempts.py \
  deploy/tests/test_hitter_task_events.py \
  deploy/tests/test_hitter_task_monitor.py \
  deploy/tests/test_hitter_task_pipeline.py \
  deploy/tests/test_hitter_task_recording.py \
  deploy/tests/test_hitter_task_replay.py \
  deploy/tests/test_hitter_task_replay_process.py \
  deploy/tests/test_hitter_completed_result_queue.py
```

Outcome: 71 failed, 169 passed, two warnings, and 47 subtests passed in 10.24 seconds. JUnit records 70 failing/error
testcase identities because pytest aggregates the two named subtest failures into their parent result. All 70 identities
are present in the approved post-style baseline `docs/refactor_20260909/style_tests.xml`: 22 monitor, nine pipeline, two
session-path/repository, 27 session-recorder, and ten replay testcases. The comparison found zero new failure identities.
These are the existing schema/stale-fixture failures documented before Task 3; they were not changed or weakened.

After the current Task 3 source stabilized, the controller's full offline suite reported 110 failed, 680 passed, two
skipped, and 254 passing subtests in 27.11 seconds. Its comparison against the approved post-style baseline found zero
new failures and zero missing tests. All four new tests pass; the freshly built C++ fixtures also turned one prior
failure and one prior skip into passes. The controller saved the detailed comparison in
`docs/refactor_20260909/test_comparison.json`.

## AST and reference audit

The definition-equivalence and edited-file audit used Python 3.10 to parse `git show f558b7c:<path>` and the worktree.
For each moved name it compared `ast.dump(node, include_attributes=False)`. For edited existing files it removed only
top-level imports and the five named moved definitions from both module trees, then compared the remaining module ASTs.
The exact invocation and audited paths were:

```bash
/home/yhl/miniforge3/envs/isaaclab/bin/python - <<'PY'
import ast
import subprocess
from pathlib import Path

base = "f558b7c"
moves = {
    "deploy/diagnostics/hitter_task_models.py": (
        "deploy/utils/hitter_serialization.py",
        ("JsonScalar", "JsonValue", "freeze_json_value", "to_builtin_json"),
    ),
    "deploy/mocap_bridge/chingmu_sdk_client.py": (
        "deploy/mocap_bridge/mocap_types.py",
        ("MocapFrame",),
    ),
}

def tree_at_revision(path):
    source = subprocess.check_output(["git", "show", f"{base}:{path}"], text=True)
    return ast.parse(source)

def tree_at_worktree(path):
    return ast.parse(Path(path).read_text())

def named(tree):
    result = {}
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            result[node.name] = node
        elif isinstance(node, ast.Assign):
            result.update((target.id, node) for target in node.targets if isinstance(target, ast.Name))
    return result

def strip_allowed(tree, removed):
    body = []
    for node in tree.body:
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            continue
        names = []
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            names = [node.name]
        elif isinstance(node, ast.Assign):
            names = [target.id for target in node.targets if isinstance(target, ast.Name)]
        if any(name in removed for name in names):
            continue
        body.append(node)
    tree.body = body
    return tree

for old_path, (new_path, names) in moves.items():
    before = named(tree_at_revision(old_path))
    after = named(tree_at_worktree(new_path))
    for name in names:
        assert ast.dump(before[name], include_attributes=False) == ast.dump(after[name], include_attributes=False)
        print("MOVED_DEFINITION_OK", name)

modified = (
    "deploy/diagnostics/hitter_task_attempts.py",
    "deploy/diagnostics/hitter_task_events.py",
    "deploy/diagnostics/hitter_task_models.py",
    "deploy/diagnostics/hitter_task_monitor.py",
    "deploy/diagnostics/hitter_task_pipeline.py",
    "deploy/diagnostics/hitter_task_recording.py",
    "deploy/diagnostics/hitter_task_replay.py",
    "deploy/mocap_bridge/chingmu_sdk_client.py",
    "deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py",
    "deploy/mocap_bridge/tests/test_vicon_table_lcm_bridge.py",
    "deploy/mocap_bridge/vicon_sdk_client.py",
    "deploy/tests/test_hitter_task_attempts.py",
    "deploy/utils/hitter_realtime.py",
)
removed = {
    "deploy/diagnostics/hitter_task_models.py": {
        "JsonScalar", "JsonValue", "freeze_json_value", "to_builtin_json"
    },
    "deploy/mocap_bridge/chingmu_sdk_client.py": {"MocapFrame"},
}
for path in modified:
    before = strip_allowed(tree_at_revision(path), removed.get(path, set()))
    after = strip_allowed(tree_at_worktree(path), removed.get(path, set()))
    assert ast.dump(before, include_attributes=False) == ast.dump(after, include_attributes=False), path
    print("NON_IMPORT_AST_OK", path)
PY
```

Outcome: all five moved definitions reported `MOVED_DEFINITION_OK`; all 13 edited existing files reported
`NON_IMPORT_AST_OK`.

The active import scan used this exact command:

```bash
/home/yhl/miniforge3/envs/isaaclab/bin/python - <<'PY'
import ast
from pathlib import Path

old_serialization = {"JsonScalar", "JsonValue", "freeze_json_value", "to_builtin_json"}
violations = []
for path in sorted(Path("deploy").rglob("*.py")):
    if "mocap_bridge (copy)" in str(path) or "__pycache__" in path.parts:
        continue
    tree = ast.parse(path.read_text(), filename=str(path))
    for node in ast.walk(tree):
        if not isinstance(node, ast.ImportFrom):
            continue
        names = {alias.name for alias in node.names}
        if node.module in {"diagnostics.hitter_task_models", "deploy.diagnostics.hitter_task_models"}:
            moved = names & old_serialization
            if moved:
                violations.append((str(path), node.lineno, node.module, sorted(moved)))
        if node.module in {
            "deploy.mocap_bridge.chingmu_sdk_client", "mocap_bridge.chingmu_sdk_client"
        } and "MocapFrame" in names:
            violations.append((str(path), node.lineno, node.module, ["MocapFrame"]))
assert not violations, violations
print("ACTIVE_REFERENCE_SCAN_OK")
PY
```

Outcome: `ACTIVE_REFERENCE_SCAN_OK`.

## Syntax and formatting

The touched files compile successfully under Python 3.10:

```bash
/home/yhl/miniforge3/envs/isaaclab/bin/python -m compileall -q \
  deploy/utils/hitter_serialization.py deploy/mocap_bridge/mocap_types.py \
  deploy/diagnostics/hitter_task_models.py deploy/diagnostics/hitter_task_attempts.py \
  deploy/diagnostics/hitter_task_events.py deploy/diagnostics/hitter_task_monitor.py \
  deploy/diagnostics/hitter_task_pipeline.py deploy/diagnostics/hitter_task_recording.py \
  deploy/diagnostics/hitter_task_replay.py deploy/utils/hitter_realtime.py \
  deploy/mocap_bridge/chingmu_sdk_client.py deploy/mocap_bridge/vicon_sdk_client.py \
  deploy/tests/test_hitter_serialization.py deploy/tests/test_shared_module_isolation.py
```

Outcome: exit status zero with no output.

The approved formatting profile was checked over all 17 touched Python files:

```bash
PYTHONPATH=/tmp/rb4-refactor-tools /home/yhl/miniforge3/bin/python -m black \
  --line-length 120 --skip-string-normalization --target-version py310 --check \
  deploy/utils/hitter_serialization.py deploy/mocap_bridge/mocap_types.py \
  deploy/diagnostics/hitter_task_models.py deploy/diagnostics/hitter_task_attempts.py \
  deploy/diagnostics/hitter_task_events.py deploy/diagnostics/hitter_task_monitor.py \
  deploy/diagnostics/hitter_task_pipeline.py deploy/diagnostics/hitter_task_recording.py \
  deploy/diagnostics/hitter_task_replay.py deploy/utils/hitter_realtime.py \
  deploy/mocap_bridge/chingmu_sdk_client.py deploy/mocap_bridge/vicon_sdk_client.py \
  deploy/tests/test_hitter_task_attempts.py \
  deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py \
  deploy/mocap_bridge/tests/test_vicon_table_lcm_bridge.py \
  deploy/tests/test_hitter_serialization.py deploy/tests/test_shared_module_isolation.py
```

Outcome: all 17 files would be left unchanged. `git diff --check` reported no whitespace errors.
