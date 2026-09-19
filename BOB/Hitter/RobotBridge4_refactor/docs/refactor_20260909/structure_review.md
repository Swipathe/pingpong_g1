# Task 3 shared-module structure review

Review range: `f558b7c..a17d1e7`

Reviewed inputs:

- `.superpowers/sdd/2026-09-09-robotbridge4-refactor/structure-review-brief.md`
- `.superpowers/sdd/2026-09-09-robotbridge4-refactor/structure-brief.md`
- `.superpowers/sdd/2026-09-09-robotbridge4-refactor/review-structure.diff`
- `docs/refactor_20260909/structure_report.md`
- `docs/refactor_20260909/test_comparison.json`

## Findings

No material findings.

## Specification compliance: APPROVED

The commit implements the requested ownership changes without expanding scope:

- `deploy/utils/hitter_serialization.py:9` owns `JsonScalar` and `JsonValue`; lines 17 and 43 own
  `freeze_json_value` and `to_builtin_json`. Independent AST comparison against the definitions in pre-structure commit
  `f558b7c` confirms that all four definitions are unchanged.
- `deploy/diagnostics/hitter_task_models.py:10` imports the three moved definitions it still uses. The other active
  diagnostics consumers and `deploy/utils/hitter_realtime.py:11` import moved symbols directly from the canonical utils
  module. An active Python-source scan found no moved serialization symbol imported from `hitter_task_models`.
- `deploy/mocap_bridge/mocap_types.py:9` owns the frozen `MocapFrame` dataclass with the same fields, annotations, order,
  and `body_pose_source_time_s=None` default as `f558b7c`. Independent AST comparison confirms exact definition
  equivalence.
- `deploy/mocap_bridge/chingmu_sdk_client.py:11` and `deploy/mocap_bridge/vicon_sdk_client.py:14` import the shared frame
  directly from `mocap_types`. The two active tests that previously imported the frame through the ChingMu client were
  updated to the canonical path. The active scan found no old-path `MocapFrame` import; the remaining old definition is
  confined to the excluded historical `deploy/mocap_bridge (copy)` tree.
- ChingMu's `Timeval`, `TrackerReport`, `HierarchyReport`, `EndHierarchyReport`, and three ctypes callback declarations
  remain in `deploy/mocap_bridge/chingmu_sdk_client.py:14-49`. Inspection of the scoped diff found no ABI changes.
- The 13 edited pre-existing Python files differ only in the required definition removals and import updates. The commit
  contains no vendor, generated, history, configuration, C++, schema, or framework-layout changes, and it adds no
  compatibility shim, explicit re-export, old-path forwarding, or fallback.

## Code quality: APPROVED

The new tests are focused on observable guarantees rather than source layout:

- `deploy/tests/test_shared_module_isolation.py:33` uses a fresh subprocess with explicit repository `cwd` and
  `PYTHONPATH`, imports serialization plus `utils.hitter_realtime`, exercises the functions, and verifies that no
  diagnostics package/module was loaded.
- `deploy/tests/test_shared_module_isolation.py:48` imports the shared frame and Vicon client in another fresh process,
  verifies that the ChingMu client is absent from `sys.modules`, and checks the dataclass's frozen behavior and only
  default value.
- `deploy/tests/test_hitter_serialization.py:9` checks detachment, nested immutability, NumPy conversion, and the fixed
  `NaN`/`Infinity`/`-Infinity` tokens. Its second test at line 31 verifies fresh nested dict/list containers across
  repeated conversions. Existing model serialization coverage remains in place with only its import path updated.

The tests are modest, deterministic, and avoid hardware and network access. The helper preserves the caller's
environment while placing the repository paths first, so it exercises the committed modules in isolation.

## Validation evidence

Per the review brief, I did not rerun successful tests. I inspected the committed test sources, the controller logs, and
`test_comparison.json`, and independently repeated only the read-only AST definition comparison and active-reference
inspection.

The controller's full suite for this source commit reports `110 failed, 680 passed, 2 skipped, 254 subtests passed`,
versus `111 failed, 674 passed, 3 skipped, 254 subtests passed` before the structure change. The normalized comparison
records zero new failure identities and zero missing tests. All four new tests pass. One prior failure and one prior skip
became passes after the controller rebuilt their C++ fixtures; they do not represent an incidental source fix in this
commit.

Task 3 is ready for the archive/documentation phase.
