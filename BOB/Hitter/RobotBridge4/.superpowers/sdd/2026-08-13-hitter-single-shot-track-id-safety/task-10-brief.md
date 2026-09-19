# Task 10 diagnostics v2 canonical model and shadow pipeline — controller brief

Base commit: `0fe2f06e077b922f639a64b4fffdfa4a81561adb`.

Read the full Task 10 section in
`docs/superpowers/plans/2026-08-13-hitter-single-shot-track-id-safety.md`
and the completed read-only preflight findings supplied in the task message.
Implement with strict RED -> GREEN evidence, then commit only Task 10 paths.

## Scope

- `deploy/diagnostics/hitter_task_models.py`
- `deploy/diagnostics/hitter_task_events.py`
- `deploy/diagnostics/hitter_task_attempts.py`
- `deploy/diagnostics/hitter_task_pipeline.py`
- `deploy/tests/test_hitter_task_attempts.py`
- `deploy/tests/test_hitter_task_events.py`
- `deploy/tests/test_hitter_task_pipeline.py`
- `deploy/tests/test_hitter_task_input_adapter.py` (partial stage only; preserve
  existing user G2 migration hunks and reconcile them deliberately)

Do not modify Task 9 runtime files, monitor/web/recording/replay/exporter files,
configuration, RealWorld, LCM publisher, or generated types. Do not launch LCM,
network publishers, PD, or real hardware.

## Canonical rulings

1. Keep the production class name `ShadowTaskPipeline` and its separate
   `tick(lifecycle_now_s, obs_now_s, wall_time_us)` clocks. Do not add a second
   `HitterTaskPipeline` or merge the clocks.
2. Recording schema is 2; live schema is 3. Runtime online code accepts only
   current schemas. Task 13 alone owns replay-only legacy conversion.
3. Online adapter channel/type contract is strict v2:
   - `ball`: exact subject, `type(track_id) is int`, ID > 0.
   - `G2Pelvis`: exact wire subject, ID == 0.
   - `table`: exact wire subject, ID == 0.
   Reject bool, missing/negative/wrong IDs, G1Pelvis, v1 messages/channels, and
   aliases. Invalid ball frames retain the wire ID. Online identity source is
   always `wire_v2`; never allocate or change IDs locally on visibility,
   bounce, estimator reset, strike, or production reset.
4. Attempt identity is wire `track_id`; generation and optional visibility
   segment never create a new attempt. A consumed/terminal ID cannot reopen;
   only a new positive publisher ID starts another attempt.
5. Migrate pipeline atomically away from `latest_result_bundle`, lifecycle
   `cached_result/cached_binding`, string decisions, error-text inference,
   overwrite-before-consume metrics, and local ID reset semantics.
6. Tick order is exactly: one typed lifecycle `advance`; one
   `drain_completed_results`; overflow-or-ordered-ingest; final typed snapshot.
   Non-overflow results are ingested in batch order. `FrozenPlannerResult | None`
   is diagnostic mirror only and `None` is valid in deterministic tests.
7. On overflow, ingest zero individual results. Consume every ID from returned
   results plus `overflowed_track_ids` with typed
   `RESULT_QUEUE_OVERFLOW`; emit one integrity event carrying current-batch
   overflow metadata. Keep pending replacement/drop and completed queue
   overflow as independent counters.
8. `completed_result_queue_depth` is `len(batch.results)` for that drain;
   capacity is Task 9 settings; health overflow is cumulative worker total;
   event overflow count is the current batch count. `planner_submit_rate_hz` is
   the configured upper bound, not a measured rate.
9. Planner reason comes solely from `PlannerResultSnapshot.failure_reason`.
   Changing `error_text` must not change the decision.
10. The production command is the sole strike-side source. Explicitly project
    `strike_table_y_w`, `strike_side_source`,
    `expected_strike_type_from_table_y`, and `strike_type_consistent` from
    `HitterWbcCommand`; do not reimplement y-sign logic in diagnostics/tests/UI.
    The freeze helper does not capture properties automatically.

## Required canonical fields

- `NormalizedMocapSample`: `track_id`, `identity_source`, existing timing/pose/
  validity metadata.
- `BallDiagnosticState`: `track_id`, `identity_source`, `consumed`,
  `source_frame`, `generation` in both recording and live serialization.
- `HealthSnapshot`: configured submit rate plus completed queue depth/capacity/
  cumulative overflow; remove legacy completed overwrite metric.
- `LifecycleSnapshot`: typed phase/decision, active identity, failure/cancel,
  consumed, locked side/base/deadline, commit state, policy TTS, strike count,
  queue depth, and all three clocks; remove cached identity.
- Attempt summary/detail: wire identity and source, consumed, cancel/failure,
  locked side/base/deadline, production side fields, consistency, strike count.
- Critical events: identity/generation/frame, typed phase/decision/failure,
  consumed, locked side, policy TTS, queue depth. Integrity event additionally
  includes overflow count/IDs/compromised IDs.

## Test and dirty-tree requirements

- Reuse `deploy/tests/hitter_test_factories.py` and Task 9's deterministic
  runtime harness; do not invent simplified lifecycle/result contracts.
- Add all six Task 9-required `HitterRuntimeSettings` fields explicitly to old
  fixtures. Do not add production defaults to hide fixture omissions.
- Record the initial Task 10 RED suite. Current preflight baseline was 73 passed,
  32 failed; after temporarily satisfying settings, old pipeline crashes on
  missing `cached_result`, confirming the migration requirement.
- Add focused coverage for schema round-trip, strict v2 identity types (including
  bool), invalid same-ID retention, reset/strike same-ID behavior, ordered multi-
  result ingestion, three failures, success/failure ordering, typed reason vs
  unrelated text, overflow zero-ingest/complete quarantine, separate counters,
  attempts by wire ID, and direct equality with production command side
  properties.
- `deploy/tests/test_hitter_task_input_adapter.py` has pre-existing half-finished
  user G2 changes. Inspect each hunk; do not overwrite or stage it wholesale.
  Test helpers should require explicit track IDs rather than infer them.
- Preserve every unrelated modification, deletion and untracked file. Never use
  broad add, reset, restore, clean, or checkout.

## Verification and delivery

Run the four Task 10 target files, then Task 6/8/9 adjacent suites, py_compile,
`git diff --check`, and an index/archive replay. Verify the archive cannot see
unstaged user changes (especially `deploy/tests/__init__.py`). Write exact RED/
GREEN results and staged boundaries to `task-10-report.md`, then commit with:

`feat: project HITTER v2 identity diagnostics`

Do not claim completion until the commit-tree archive is green and an independent
review has no blocking findings.
