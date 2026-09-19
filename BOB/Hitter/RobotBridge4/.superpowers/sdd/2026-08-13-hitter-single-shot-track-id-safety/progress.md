# SDD ledger — plan: docs/superpowers/plans/2026-08-13-hitter-single-shot-track-id-safety.md

Branch start: ff02a32
Implementation base after approved plan amendment: c8d5945
Workspace: /home/loco1/BOB/Hitter/RobotBridge4
Execution mode: subagent-driven development with per-task TDD and review

Preflight:
- Relevant mocap baseline: 59 passed in 0.51 s.
- Current transformation_t v1 fingerprint: 71f936e3b20f1df5.
- Plan conflict found before Task 1: RobotBridge4/unitree_sdk2/build/CMakeCache.txt points at RobotBridge2. The plan's Task 14 command `cmake --build unitree_sdk2/build` conflicts with the global RobotBridge4-only write boundary.
- Baseline invocation regenerated ignored CMake metadata under RobotBridge2/unitree_sdk2/build and rebuilt ignored RobotBridge4/unitree_sdk2/build/bin/trans; no tracked RobotBridge2 source/status change was observed. No cleanup attempted.
- User-approved ruling: global boundary governs. Never use `unitree_sdk2/build`; configure and build Unitree targets only in RobotBridge4-local `unitree_sdk2/.build-robotbridge4-v2`, verify its `CMAKE_HOME_DIRECTORY`, and keep the directory locally ignored.

Task 1: dispatched (base c8d59451fd7a5f371e7a1937cae5265e6c72ee9a; implementer /root/task1_lcm_v2_wire)
Task 1: reviewer warning resolved — deterministic lcm-gen regeneration produced no diff; both local build paths hit exact `.git/info/exclude` entries; commit range contains only the six Task 1 paths and excludes user `bin/`; RED command/output is recorded in the implementer report; active online v2-only consumption is intentionally implemented and verified by later tasks.
Task 1: complete (commits c8d5945..8bc598a, review clean)

Task 2: dispatched (base 8bc598ab765341c0cd9441f4bb5050a998940d42; pre-existing dirty `vicon_table_lcm_bridge.cpp` worktree blob 006c74243e04937576325ce301194dee96fca524, diff sha256 c57c6c0bf26f48e8c9c0428261bb638a00ee61002682f20efcec6ae0f2653108, +9/-8)
Task 2: clarified source-time fallback interface — preserve the specified 7-argument `AdvanceBallTrack` with a 300 Hz deterministic wrapper; a translation-unit-internal helper may accept the selected runtime frame rate, and both paths must share/test the same normalization logic.
Task 2: resolved contradictory sample — the fixed 0.35 m association safety boundary governs; the sample now establishes velocity with an in-radius approach before crossing x=0, bounce coverage requires every CV prediction error to remain within radius, and no x/bounce exception may widen association.
Task 2: fix round 1/5 dispatched from review head 9c58d69 (open: deterministic candidate ordering; allocator exhaustion without signed overflow; strict non-throwing numeric CLI parsing).
Task 2: fix round 1/5 (3 addressed, 0 open; commits 9c58d69..c55b592)
Task 2: complete (commits 8bc598a..c55b592, review clean)

Task 3: preflight fixture correction — mirror Task 2's canonical sequence by inserting the in-radius approach frame before miss/return; strict 0.35 m association remains binding.
Task 3: dispatched (base 812b09d00a13c680a916f47fce6b8d73d381cbcd; pre-existing dirty ChingMu blob dc16d7b85f3c492373a40083a33f0a0ff36dfe65, +88/-5, diff sha256 40a36bc3118433fd4efea232b234ee96f981737552d587b3657c7e12b788e59c; monitor blob 835cecbbc85ea2d6adfddb2d6345dbcdddad68a4, +1/-1, diff sha256 fe3a2a36a8c2367d7c13adee9ba0000d6fe5eb7ce37084777c1b7219a0d10f1c)
Task 3: minor (deferred): exact-radius test derives `1.15 - 0.8`, which may land just inside 0.35; final review should triage using an exact derived radius plus nextafter-outside case.
Task 3: fix round 1/5 dispatched from review head 3d8014b (open: freeze tracker while base invalid; fail-closed resilient monitor callback; subject-scoped persistent contract error transitions).
Task 3: fix round 1/5 (3 addressed, 0 open; commits 3d8014b..72a549b)
Task 3: minor (deferred): monitor CSV flush currently checks `csv_count % 300 == 0` only at 1 Hz summaries, so crossing but not landing on a multiple can delay flush until exit; final review should triage a next-threshold counter.
Task 3: complete (commits 812b09d..72a549b, review clean with 2 deferred minors)

Task 4: dispatched (base 3b76db22f77d68c16ae6d23442b67801b2b52fa7; dirty-path fingerprints recorded separately in controller output; new runtime/key test paths absent; active tree has 10 files containing `track_epoch` before migration)
Task 4: clarified snapshot fields — add immutable `consumed=False` and `new_track=False` contract fields now, test them explicitly, but defer all wire-aware values and gating behavior to Tasks 7/9.
Task 4: process correction — SDD report was accidentally tracked by b65e24b and removed from the index by b37ede9 while preserving the ignored local file; net task diff contains no SDD artifact.
Task 4: reviewer warnings resolved — report preserves RED/GREEN evidence and dirty baselines; static package shows no G1/G2 behavior changes from Task 4, so the five reported G2 fixture failures remain pre-existing/non-identity failures for later integration triage.
Task 4: complete (commits 3b76db2..b37ede9, review clean)

Task 5: dispatched (base ebec23637c452b1a7ca1a3dc336efe81d9fa22ff; pre-existing dirty planner blob 0cdd2207cea28e009782f2356bfffb8a8174d4cd +46/-1 diff sha256 cb2d5371197485a17b0847a069bcd792965488f06bde5d6d7a97b6f8429c7ba2; runtime factory blob da9b9dd68c96487cb9843bca85d8855bdf5b1188 +4/-0 diff sha256 d2dcb17310bf689f684cdcfea481efdacd1fb4d44e169f4a87c230a2a8bdf604; env and factory-test fingerprints recorded in controller output)
Task 5: complete (commits ebec236..15063ca, review clean)

Task 6: dispatched (base 15063ca88fd2e6ff7ea120201631c40c82f1953d; realtime clean; new queue test absent; diagnostics safety pre-existing dirty blob 97224be4ec1d28495b3d796c2b4efdd75431734a +2/-2 diff sha256 7e77f15ca9eeafb4253aea301d39de728e4f175d102a24b219b56699d593a692)
Task 6: fix round 1/5 dispatched from review head b5f8c82 (open: trace-listener close lock-order deadlock; eliminate independent legacy `.error` source of truth).
Task 6: fix round 1/5 (2 addressed, 1 new open — diagnostics result/frozen failure-detail parity regression; commits b5f8c82..808befe)
Task 6: fix round 2/5 dispatched from review head 808befe (open: compare typed reason, error type, and error detail without restoring legacy `.error`).
Task 6: fix round 2/5 (implementation addressed, 1 test-contract gap open — matcher not exercised with real worker-produced pairs; commits 808befe..927a33e)
Task 6: fix round 3/5 dispatched from review head 927a33e (open: real producer pair characterization/mutation coverage for typed and unknown failures).
Task 6: fix round 3/5 (1 addressed, 0 open; commits 927a33e..49efb76)
Task 6: complete (commits 15063ca..49efb76, review clean)

Task 7: dispatched (base 49efb76b3b5705c690e33275f5d25dc19db6bf9e; real_world dirty blob e4b4baa72eb270faed77dd72d3382111973e92e3 +30/-20 diff sha256 ab78a79781867002d93206ff9e8d03fd208efd8ad8794b67ef76cde1c1411227; runtime factory dirty blob d1949bb577e15536b83ca212855a4328234488a9 +4/-0 diff sha256 6903cb0a12272d140bb61ef378eef52b727659eb0993a45644a33c65e89da38b; user connection-wait test is untracked/read-only)
Task 7: fix round 1/5 dispatched from review head d9dd20b (open: clean-commit G2/configured subject consistency; reused-ID overlap conflict; valid duplicate/backward no-ball reset; reentry snapshot consumed parity; strict frame/flags/finite-pose wire validation; bounded fail-closed transition queue).
Task 7: minor (deferred): external `received_monotonic_s` injection is only finite-checked, not globally non-decreasing; final review should decide whether production-only `time.monotonic()` makes this an API hardening item.
Task 7: minor (deferred): stream stale currently dominates ball stale in the same status tick; final review should make that priority explicit in contract/tests or require both ordered events.
Task 7: fix round 1/5 (6 addressed, 2 new important open; commits d9dd20b..e5ad336; clean archive consumer 67 passed; current consumer+connection+factory 81 passed).
Task 7: fix round 2/5 dispatched from review head e5ad336 (open: schema/overflow must quarantine active planner eligibility; seen challenger conflict must advance source-frame watermark without taking authority).
Task 7: minor (deferred): conflict parity test should explicitly use a previously unconsumed admitted incumbent, not only startup-quarantined tracks.
Task 7: minor (deferred): overflow permanence reentry test should refresh a valid base before asserting permanent schema rejection.
Task 7: fix round 2/5 (2 addressed, 0 open; commit e5ad336..1580fcb; current consumer+connection+factory 80 passed; clean archive consumer 66 passed and py_compile passed).
Task 7: minor (deferred to Task 9): listener tuple is captured under lock and invoked outside it; a concurrent consume between those operations can deliver one stale eligible snapshot, so the final HitterEnv admission gate must re-check consumed/fault state.
Task 7: complete (commits 49efb76..1580fcb, independent static review PASS with 1 deferred concurrency minor)

Task 8: contract audit complete before dispatch. Rulings added for TRACKING cancellation identity, shared result generation watermark, one-phase-per-advance, locked-deadline commit authority, immutable override reconstruction, semantic WAITING edges, enum/type validation, final numeric defaults, and persistent diagnostics state.
Task 8: Task 9 preflight added a required `reset_for_policy_reentry()` transition that clears/consumes transient state while preserving process-lifetime identity history; this prevents ordinary env reset from reviving a consumed physical track.
Task 8: initial implementation committed at a90d82f with 120 focused passes; independent static review requested changes.
Task 8: fix round 1/5 dispatched from review head a90d82f (open: consume TRACKING challenger IDs permanently; validate committed success before freeze so malformed commands become typed INTERNAL_ERROR; construct commit failure streak through public transitions).
Task 8: fix round 1/5 (2 Important + 1 Minor addressed, 0 open; commit a90d82f..d731148; lifecycle 70 passed, adjacent regressions 58 passed, py_compile and diff check passed).
Task 8: complete (commits 1580fcb..d731148, independent static re-review PASS with no remaining findings)
Task 9: preflight contract audit complete. Plan amended to use only nested `motion.vicon_consumer`, extend RealWorld with end-session/event-clear/eligibility APIs, quarantine reset-era queued and in-flight results, enforce one real lifecycle tick per agent iteration, keep the five-second first-frame transition admission-closed, latch anchor faults independently of phase, and include the shared test harness plus strike logging in scope.
Task 9: settings subunit landed at 6167d51; independent review found strict Vicon type gaps, deferred runtime queue injection, and later diagnostics/replay constructor dependencies. Config hardening fix 70c91ad rejects non-mapping/null/bool/string Vicon fields and negative seeds; current settings+consumer suite 117 passed.
Task 9: RealWorld session/eligibility/consume subunit landed at 1c61ceb and passed independent review with no findings (92 focused tests; connection/R2 user hunks preserved unstaged).
Task 9: main HitterEnv/agent runtime integration dispatched from base 70c91ad with controller brief `task-9-main-brief.md`; queue-capacity injection is mandatory here, while replay/diagnostics settings constructor migration remains a Task 10/13 hard prerequisite.
Task 9: clean-index review found and fixed three dirty-tree false positives before commit: planner velocity alignment was excluded from the staged property-only hunk; tests were made package-import safe without staging the user's `deploy/tests/__init__.py` deletion; and `HitterEnv._physics_step()` was made self-contained so it does not depend on the user's unstaged `BaseEnv` action hunk.
Task 9: complete (commit 0fe2f06; clean commit archive 333 passed; py_compile and diff check passed; independent commit-tree review PASS with no Critical or Important findings; no LCM, publisher, PD, or real-hardware process was started).
Task 10: preflight found Task 8/9 hard dependencies and two naming conflicts. Final implementation will retain the existing `ShadowTaskPipeline` name and existing separate lifecycle/observation/wall clocks, extending `TaskTickResult` rather than introducing a duplicate `HitterTaskPipeline` or merging clocks.
