# RobotBridge4 Refactor Implementation Plan

Status: completed in the approved writable copy on 2026-09-09. Original repositories remain unchanged. See [handoff](../../refactor_20260909/handoff.md) and [execution record](../../refactor_20260909/execution_record.md).

> For agentic workers: REQUIRED SUB-SKILL: use superpowers:subagent-driven-development to execute this plan.

Goal: Apply the approved style and incremental structure refactor in the writable current-disk snapshot.
Architecture: Retain the original deployment framework, extract shared serialization into utils and shared MocapFrame into mocap_bridge/mocap_types, archive unused new historical copies.
Tech Stack: Python, C++, Bash, Hydra, MuJoCo, LCM; offline validation using the existing isaaclab environment and a temporary dependency overlay.
Spec: docs/superpowers/specs/2026-09-09-robotbridge4-refactor.md (approved by YHL).

## Global Constraints

- Baseline and original RobotBridge4 are read-only. Work only in /home/yhl/Desktop/RobotBridge4_refactor.
- Preserve current disk changes; do not restore deleted files from source Git HEAD.
- Baseline-identical files remain byte-for-byte unchanged. In changed original files, edit only added/modified regions.
- Keep original framework directories, runtime behavior, algorithms, threading, ABI, LCM, CLI, config values, observation/action dimensions and side-effect import order unchanged.
- No compatibility shims, re-export aliases, fallback logic or incidental bug fixes.
- Exclude vendor, generated LCM and historical code from formatting; preserve readable model/calibration bytes.
- Archive only unreferenced new historical source/config files, retain their relative paths and exact bytes; do not archive calibration files.
- Report baseline test failures and inaccessible artifacts explicitly; do not claim hardware validation.

## Task 1: Snapshot and baseline (controller setup)

- Copy readable current files without Git metadata, caches, outputs, recordings and build directories; record hashes, excluded paths and permission-denied calibration files.
- Initialize independent Git history to record this refactor. Preserve ignored models/assets on disk and in hash manifest.
- Install test/format tools in /tmp only, leaving isaaclab packages unchanged.
- Run offline Python suite before changes, capture collection blockers/failures and test environment. Use temporary runner for deleted test-package markers if necessary; do not restore preexisting deletions.
- Review snapshot against source and record validation.

## Task 2: Style only (implementer)

Files: newly added active first-party Python/C++/Shell/HTML files under deploy, plus modified extensions of deploy/simulator/mujoco.py and real_world.py. Classification is in docs/refactor_20260909/baseline_comparison.json.

- Examine representative baseline module code and adopt local spacing, import grouping, four-space indentation and readable wrapping; preserve meaningful typing and comments.
- Use controlled Python formatting at 120 columns with string spelling and import execution order preserved; never format whole baseline-modified files.
- New C++ should use baseline four-space indentation/braces, no symbol renaming or token changes. Shell/HTML: inspect and apply only justified whitespace changes, preserve embedded code/string contents.
- Record reviewed-but-unchanged files, changed files and scope decisions.
- Verify AST equality against snapshot for every Python style edit; verify C++ tokens; verify baseline-identical files and protected regions.
- Report exact commands and outcomes in docs/refactor_20260909/style_report.md.

## Task 3: Shared module ownership (implementer, after Task 2)

Files: new deploy/utils/hitter_serialization.py and deploy/mocap_bridge/mocap_types.py; original definitions and every active caller/test.

- Add focused isolation checks for the new module boundaries; confirm failure before extraction.
- Move JsonScalar, JsonValue, freeze_json_value and to_builtin_json verbatim to utils. Keep diagnostic schema/models in diagnostics. Update all imports directly, no old-path compatibility exports.
- Move frozen MocapFrame dataclass verbatim to mocap_types; retain ctypes structures and callbacks in ChingMu client. Update clients/consumers/tests directly.
- Clean only imports made obsolete by extraction.
- Validate existing serialization/frame tests, new isolation checks, and definition AST equality, then full offline suite.
- Record exact commands and outcomes in docs/refactor_20260909/structure_report.md.

## Task 4: Archive and documentation (controller, after Task 3)

- Search active references to mocap_bridge (copy) and .before-* first-party source/config backups; archive only unused paths under _archive/refactor_20260909 preserving relative paths and hashes.
- Generate per-file change/move/archive manifest and a three-way structure comparison with Mermaid labels that do not begin with Markdown list markers.
- Update copy README and active docs affected by moved public definitions; retain historical design records with explicit historical context where relevant.
- Add offline reproduction instructions and list unavailable calibration/recording/build artifacts.

## Task 5: Final verification and review

- Run relevant Python suite and offline C++ protocol/tracker tests; expand eight Hydra configs without launching devices; headless model/asset load if environment permits.
- Audit source/baseline hashes, archive bytes, final import directions, Python definition equivalence, and changed-region scope.
- Separate existing failures from regressions. Never mask baseline failures by weakening assertions.
- Obtain task reviews and broad final review; address material findings with scoped validation.
- Deliver local copy, comparison document, changes and verified limitations. No merge, push or real-robot command.
