# SDD ledger — plan: docs/superpowers/plans/2026-07-29-hitter-live-ball-diagnostics.md

Setup: work in place on `local/hitter-task-diagnostics-20260727` by explicit user preference; preserve unrelated dirty worktree state.
Baseline: 226 tests passed, 2 skipped (`test_hitter_task_*.py`).
Task 1: complete (commits b33e41d..5d88016, review clean)
Task 2: complete (commits 5d88016..d48a771, review clean)
Task 2: minor (deferred): persistence/API tests do not include a known real-zero counter round trip.
Task 3: fix round 1/5 (4 addressed, 0 open — throttle terminal, unique-key dedup, bounce reset, velocity immutability; commits 963fc47..1c46b98)
Task 3: complete (commits d48a771..1c46b98, review clean)
Task 4: minor (deferred): initial schema v2 may coexist with live_snapshot=None before first LIVE_SNAPSHOT.
Task 4: minor (deferred): v2 parser validation for enum/count/nonfinite payloads could be stricter.
Task 4: fix round 1/5 (5 addressed, 1 open — capture clocks sampled before shared lock; commits 1814e1e..9c1f6a8)
Task 4: minor (deferred): report overstates that no event-lane publication can occur while capture lock is held.
Task 4: fix round 2/5 (1 addressed, 0 open — capture clocks now sampled after shared lock; commits 9c1f6a8..1a9bb52)
Task 4: complete (commits 1c46b98..1a9bb52, review clean; 3 deferred minors)
Task 5: minor (deferred): history ab_summary null still renders pending instead of dash.
Task 5: minor (deferred): new browser behavior tests skip when Chrome is unavailable.
Task 5: minor (deferred): unsupported-schema test does not isolate outer-vs-inner version rejection branches.
Task 5: complete (commits 1a9bb52..391f403, review clean; 3 deferred minors)
Task 6: complete (commits 391f403..3a139e1, review clean)
Final review: Critical — committed HEAD has broken 104-D active observation path in deploy/envs/hitter.py; current dirty worktree contains an uncommitted correction, and plan scope forbids silently committing it.
Final review: Important — persisted attempt/detail payloads still use schema v1 rather than an explicit v2 writer plus v1 migration reader.
Final review: Important — history lacks per-attempt maxima/terminal structured evidence and may lose attained counts after TRACK_ENDED before close.
Final review: awaiting user authority on whether to commit the existing deploy/envs/hitter.py correction separately before one final fix wave.
