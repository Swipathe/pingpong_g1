# Task 9 main runtime integration — controller rulings

Read `task-9-brief.md` first. This file narrows the remaining work after
commits `6167d51`, `1c61ceb`, and `70c91ad` landed the settings and RealWorld
session APIs.

## Remaining production scope

- `deploy/envs/hitter.py`
- `deploy/agents/hitter_agent.py`
- `deploy/tests/test_hitter_strike_target_logging.py`
- `deploy/tests/test_hitter_policy_first_frame_transition.py`
- `deploy/tests/test_hitter_task_observation.py`
- create `deploy/tests/hitter_runtime_test_harness.py`
- create `deploy/tests/test_hitter_waiting_anchor.py`
- create `deploy/tests/test_hitter_runtime_single_shot_integration.py`

Do not modify the already reviewed consumer/settings implementations unless a
new failing integration test proves a necessary defect. Do not modify
diagnostics/replay in this subtask; their `HitterRuntimeSettings` constructors
are a hard prerequisite assigned to Tasks 10/13. Do not touch `config/hitter.yaml`.

## Mandatory runtime structure

1. Create lifecycle, worker, listener, and lifecycle `threading.RLock` once per
   process. Ordinary reset must reuse them and call
   `reset_for_policy_reentry(now=...)`; never construct another lifecycle or
   worker.
2. Add process-lifetime tracking for every submitted/outstanding track ID.
   Reset/reentry order under the lifecycle lock is: admission false, RealWorld
   end-session, lifecycle session-reset decision, drain current completed batch,
   consume consumer active/submitted/batch/overflow IDs in both lifecycle and
   simulator, clear transient command state. A result completing later for an
   old submitted ID must be ignored-consumed.
3. Inject `settings.completed_result_queue_capacity` into the production
   `LatestOnlyPlannerWorker` constructor.
4. Listener admission under the lifecycle RLock is exactly: runtime accepting,
   anchor fault, process-lifetime lifecycle consumed set, lifecycle phase,
   consumer `hitter_snapshot_planning_eligible(snapshot)`, rate gate, register
   submitted ID, nonblocking `worker.submit`. Lock order is lifecycle RLock then
   RealWorld consumer lock. RECOVERY consumes every snapshot. A `new_track` in
   non-WAITING is consumed. A stale object with old `consumed=False` must still
   be rejected by current lifecycle/consumer state.
5. Real policy tick order is status/freshness, drain direct events/cancel, one
   `advance`, one completed-result batch drain, then ordered ingest. Overflow
   makes the entire batch unusable: ingest zero results and consume all batch
   result IDs, overflow IDs, and any active ID consumed by cancellation.
6. Apply every typed `LifecycleDecision` centrally. Synchronize every
   `decision.consumed_track_ids` to `simulator.consume_hitter_track(...,
   reason=decision.kind)`. Copy policy command only from the post-decision
   `lifecycle.active_result.command`; never copy from incoming results.
7. Delete the production `latest_result()`/string decision/error and recovery
   cache path. Status logging may use worker stats but not latest-result as a
   control source. Keep summary logging at no more than 1 Hz.
8. In real mode, the agent calls `refresh_policy_observation()` before ONNX and
   that is the sole lifecycle tick for the iteration. Real `_post_physics_step`
   must not advance/drain again. MuJoCo retains its synchronous post-physics
   update path and configured waiting target.

## WAITING anchor and first-frame transition

- Real reset first closes admission/session, refreshes state, and captures an
  initial valid current pelvis world xy anchor so the first observation is safe.
- During the configured first-frame PD transition, session and planner admission
  stay closed.
- When the transition finishes, refresh pelvis, recapture the current anchor,
  quarantine old worker results again, then successfully begin the policy
  session. Only then set accepting true; the 0.50 s no-ball timer starts there.
- A zero-duration transition performs the same reentry immediately after reset.
- Any `BASE_POSE_INVALID` event/status, in any lifecycle phase including
  committed ARMED, latches `_waiting_anchor_fault` and retains an existing
  anchor. Pose recovery in the same session cannot reopen admission. Only a
  successful explicit reentry recapture clears the fault.
- Every `entered_waiting` decision refreshes simulator state and captures once.
  Invalid with no initial anchor raises `RuntimeError`; invalid with an old anchor
  returns false, retains it, and stays fail-closed.
- Real WAITING observation uses `(captured world xy - current pelvis world xy)`
  transformed by current yaw. Racket target is current FK, velocity zero, TTS
  waiting value. Observation stays `[1,104]`; ONNX and PD continue every tick.
- MuJoCo continues to use YAML `waiting_base_target_xy_w`.

## Required TDD coverage

Write tests first and record RED before production edits. Cover at minimum:

- process-lifetime identity of lifecycle/worker across reset and old
  pending/completed/in-flight result quarantine;
- queue capacity injection;
- fixed event -> advance -> batch order, all-results ingest, overflow zero-ingest;
- direct event beats same-tick success;
- listener phase/consumer/consumed gates and both lock barrier races;
- consume synchronization for late skip, third soft failure, immediate failure,
  discontinuity, strike/recovery;
- 0.49 s ID consumed and never resurrected; later unseen ID after 0.50 eligible;
- anchor drift correction, cancel/recovery WAITING edge capture exactly once,
  invalid-old-anchor latch, same-session recovery blocked, explicit reentry clears,
  invalid-no-anchor raises, MuJoCo unchanged;
- 10 WAITING iterations produce 10 ONNX and 10 `apply_action`, finite `[1,104]`;
- first-frame transition keeps admission closed and reopens only after refreshed
  recapture; zero duration uses same flow;
- migrate strike logging tests to typed `LifecycleDecision` API.

Use only pure offline fakes; no Unitree publisher, LCM network, sleep, or real
robot processes.

## Dirty worktree boundary

- `deploy/envs/hitter.py` already contains user G2 wording, 104-D xy slice, and
  print removal. Preserve and include only where required by this task.
- `deploy/agents/hitter_agent.py` has a user per-tick print removal. Preserve it;
  do not stage that old hunk unless it is inseparable from a new task hunk.
- `deploy/config/mimic/hitter.yaml` has user model/5 s/serve/geometry/velocity
  ranges/fore-back values. Settings commit already added task-owned nested config;
  do not touch or stage the remaining user diff.
- `deploy/simulator/real_world.py` has unstaged connection/R2 changes; do not
  stage them.
- Preserve every existing deletion and untracked file. Never use broad add,
  restore, reset, clean, or checkout.

## Verification and report

Run the Task 9 focused suite in `task-9-brief.md`, adjacent lifecycle/queue/
consumer suites, `py_compile`, cached diff check, and a clean Git archive replay
of the focused suite. Write RED/GREEN evidence and exact staged boundaries to
`task-9-report.md`. Commit only Task 9 main integration changes.
