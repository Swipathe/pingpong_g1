# SDD ledger — plan: docs/superpowers/plans/2026-09-09-robotbridge4-refactor.md

Spec: docs/superpowers/specs/2026-09-09-robotbridge4-refactor.md; approved by YHL with “确认.”
Snapshot/base commit: e85fcae. Copy: /home/yhl/Desktop/RobotBridge4_refactor.

## Preflight review

| Tasks/interface | Producer/consumer check | Finding |
| --- | --- | --- |
| Task 1 | Current disk copy -> snapshot and baseline tests | Source read permission prevents copying 12 calibration files, listed explicitly; source metadata/cache/output exclusions also listed |
| Task 2 | Scoped style edits -> AST equality | Same files protected, changed-original equal regions must survive verbatim |
| Task 3 | Definitions -> clients and diagnostic consumers | Canonical imports replace old paths without compatibility code |
| Task 4 | Archive moves -> documentation and manifest | Only unreferenced new backups, no baseline paths or calibration archives |
| Task 5 | Verification -> delivery | Baseline failures remain separately reported, no hardware validation claim |
| 1/2 | Snapshot hashes and classification -> eligible styles | Snapshot complete for readable source; edits wait for baseline test capture |
| 2/3 | Style-only AST-equivalent files -> extraction | Sequential writes; structure task checks semantic equivalence separately |
| 3/4 | New canonical imports -> doc updates | Documentation follows final code |
| 1/5 | Baseline tests -> final tests | Same environment/runner, compare failures |
| 2/5 | AST report -> final hash/scope checks | Separate format vs semantic refactor evidence |
| 3/5 | Module isolation -> regression review | New checks verify boundary rather than formatting |
| 4/5 | Archive manifest -> byte validation | Validate source-to-archive hashes |

User-approved delivery is a local current-disk copy; original repositories remain untouched. Missing calibration data is not synthesized. Full real deployment is outside validation capability.

Task 1: complete (snapshot e85fcae; source hashes 1557 entries checked, unchanged; baseline 111 failed / 674 passed / 3 skipped / 254 subtests passed; no production changes)
Task 2: dispatched style implementer; brief style-brief.md; base e85fcae. Baseline execution completed before edit release.

Task 2 controller validation: full suite 111 failed / 674 passed / 3 skipped / 254 subtests passed, identical result identities to baseline (XML parent cases 110 failed / 673 passed / 3 skipped); no new failures, no missing tests. One transient stdlib ssl/enum collection error occurred; standalone imports and affected 47-test file passed, full retry completed unchanged. No source fixes made for runner instability.

Task 2 implementation final commit: f558b7c (amended 4236c18 only in audit script/report to correctly syntax-check all four Shell files). Review package is e85fcae..4236c18 plus review-style-final-audit-fix.diff; production/test source identical between those heads. Post-commit checker against f558b7c reports failures=[] for 73 records and 1322 protected out-of-scope paths.

Additional offline validation (after style, no Python definition changes): all six first-party C++ translation units compiled. Two C++ contract executables passed; launcher --check-only passed with unchanged 20260822 table/pelvis SHA256 and matching planner values. Python/C++ fixture tests: 14 passed; building missing fixture binaries resolves the earlier environment-only C++ fixture failure/skip. Deployment markdown says 20260818 while unchanged launcher actually selects 20260822; Task 4 will correct documentation only using verified script/hash evidence.

Task 2: complete (e85fcae..f558b7c, spec compliant and quality Approved, no findings). Reviewer external-mutation-history limitation is covered by controller snapshot hashing (1557 source files unchanged) and will be rechecked against 1361 baseline hashes at final audit.
Task 3: released structure implementer from read-only prep. Base f558b7c. Canonical import map and focused red-green isolation tests prepared.

Task 3 controller full-suite validation: 110 failed / 680 passed / 2 skipped / 254 subtests passed (27.11s). New isolation/serialization cases pass; missing fixture binaries were built, resolving one old failure and one old skip. Full identity comparison in test_comparison.json: no new failures or missing tests. No source changes for legacy failures. Readable external Hitter Python search found no explicit RobotBridge4 module consumers; some non-target asset/provenance directories were unreadable, so no broad external completeness claim.

Task 3 implementation complete at stable a17d1e7; five moved definitions and 13 existing edited modules AST-equivalent modulo explicit moves/imports. Focused tests 4 passed; mocap53 passed+59subtests; full controller comparison no new failures. Task-scoped review dispatched next.

Task 3: complete (f558b7c..a17d1e7, specification and quality APPROVED, no material findings). Review at docs/refactor_20260909/structure_review.md.
Task 4: archive/docs implemented using disjoint paths while Task 3 source review completed. Archived20 exact byte-preserving paths; updated README, active diagnostics guide, current calibration guide dates and historical-plan notice; generated comparison and verification reports. Initial final audit:1557 source files/1361 baseline files checked,1313 identical files protected,20archive hashes correct, errors0. All source tests ran on final deploy program content; documentation/history moves have no active test or runtime references.

Task 4: complete (a17d1e7..f83be91, final whole-refactor review Approved; archive/documentation scope has no material findings).
Task 5: complete (whole review e85fcae..f83be91 Approved; independent snapshot/archive and JUnit identity inspection agrees with controller evidence). No deferred-minor or parked findings remain.

Handoff: desktop comparison created and original desktop documents cross-linked without replacing their historical comparisons. Local-link, code-fence and Mermaid leading-list-marker checks passed; no browser-rendering claim. Portable patch against the recorded current-disk original passed git apply --check; the initial snapshot's four added plan/manifest records are included as additions. The final patch is regenerated from the final delivery commit and its digest/check result is kept outside the repository in RobotBridge4_重构补丁校验.json to avoid a self-referential patch hash.

Finish: retain the approved local copy and refactor/style-and-structure branch; do not merge, push or apply to either original. Existing 110 console-reported test failures, 12 unreadable other calibrations and absent hardware validation remain explicit. This is the user-authorized refactor boundary; no unrelated fixes are required for the local-copy handoff. Final artifact-only changes are completion status, this durable ledger, review/handoff links and manifest refresh. Production source content remains the tested/reviewed version.
