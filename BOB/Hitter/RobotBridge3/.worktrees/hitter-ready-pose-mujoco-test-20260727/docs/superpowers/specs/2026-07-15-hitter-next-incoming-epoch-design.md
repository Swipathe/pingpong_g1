# HITTER Continuous Incoming-Ball Epoch Design

## Problem

After the first strike finishes, `HitterCommandLifecycle` marks that ball's
`track_epoch` as ended. Results from the same epoch are then ignored. Real-world
tracking currently creates a new epoch only after observing an outgoing ball
followed by an incoming ball. If the outgoing trajectory is occluded, the next
incoming ball remains in the ended epoch and the policy stays in `WAITING`.

## Required Behavior

- Observing the outgoing trajectory is not required.
- Crossing the predicted strike deadline ends the current ball track exactly
  once and advances the real-world ball epoch.
- The estimator starts the next track empty. Outgoing samples may still be
  observed, but they cannot produce a strike command because the planner
  requires `vx < 0`.
- The next incoming ball can therefore become ready and enter the planner even
  when no outgoing sample was observed.
- The current full swing and recovery duration remains unchanged.
- Existing outgoing-to-incoming detection remains as a secondary estimator
  cleanup mechanism, not as a prerequisite for creating the next playable
  epoch.
- No ball-data timeout or action gating is added.

## Data Flow

1. `HitterCommandLifecycle.advance()` crosses `ARMED -> RECOVERY` at the active
   result's monotonic strike deadline.
2. `HitterEnv` detects that transition and, for the real-world backend only,
   calls the existing `RealWorld.reset_ball_state_estimator()` once.
3. `RealWorld` clears estimator samples and increments `ball_track_epoch`.
4. The policy continues executing the existing recovery command for the
   configured full swing duration.
5. New Vicon ball samples use the new epoch. The planner ignores outgoing
   motion and produces a command once an incoming track (`vx < 0`) is ready and
   satisfies the existing table, hit-plane, height, and timing boundaries.

## Failure Handling

- If the real-world backend does not expose a callable estimator reset, log a
  warning and keep the existing lifecycle running; do not interrupt recovery.
- Do not reset repeatedly during `RECOVERY` or `WAITING`.
- MuJoCo keeps its existing ball-sequence reset path unchanged.

## Tests

- A real-world environment crossing the strike deadline resets the estimator
  and advances the epoch exactly once.
- Additional updates during recovery do not reset it again.
- The active command remains in recovery until the full swing deadline.
- A result from the newly advanced epoch can be cached during recovery and
  armed afterward without any observed outgoing trajectory.
- Existing lifecycle, real-world snapshot, planner-boundary, and MuJoCo tests
  remain green.
