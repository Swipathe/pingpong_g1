# MuJoCo and Real-World Planner Physics Parity Design

## Goal

Run the current HITTER planner and ONNX policy in MuJoCo with the same ball
flight and table-bounce model configured for real-world deployment. The user
will perform the final visual evaluation in the MuJoCo viewer.

## Scope

This work aligns the mathematical model already used by
`BallTrajectoryPredictor`:

- gravity;
- quadratic drag, `-c * ||v|| * v`;
- table position, dimensions, and height;
- ball radius;
- vertical and horizontal table restitution;
- 5 ms low-level integration interval.

It does not add spin, Magnus force, net contact, velocity-dependent
restitution, or racket-surface friction models because the current real-world
planner does not model them.

The racket-ball interaction remains a real MuJoCo contact. The simulator will
not inject the planner's desired outgoing ball velocity.

## Single Configuration Source

`mimic.motion.ball_planner` remains the only source for shared physics
parameters. The HITTER top-level Hydra profile will add a scoped
`sim.config.table_tennis` block whose table geometry, ball radius, drag, and
restitution values interpolate from that planner configuration.

The generic `sim/mujoco.yaml` will remain unchanged so non-HITTER MuJoCo tasks
do not acquire HITTER-specific ball settings.

The HITTER MuJoCo block will also define simulator-only properties:

- the MuJoCo body, freejoint, ball geom, and racket geom names;
- analytic table bounce enabled;
- analytic racket hit disabled;
- one fixed incoming ball position and velocity;
- ball randomization disabled for repeatability.

The fixed incoming ball starts at `[2.05, -0.20, 1.089]` metres with velocity
`[-2.3837, 0.0, -0.50]` metres per second. It travels toward the robot, bounces
on the table, and crosses the configured virtual hit plane at `x=0`.

## Planner and Policy Data Flow

MuJoCo exposes the ball freejoint position and linear velocity through
`ball_pos_world` and `ball_vel_world`. `HitterEnv` passes those values directly
to its existing `HitterSystemPlanner`; no simulated Vicon estimator is added.

At every 20 ms policy step, the planner rolls the ball model forward in 5 ms
increments, finds the future `x=0` crossing, and computes:

- time to strike;
- racket target position;
- incoming and desired outgoing ball velocity;
- racket target velocity;
- base target and forehand/backhand selection.

Those values enter the existing 105-value policy observation. The ONNX policy
then produces 29 joint targets, which the MuJoCo backend applies through its
existing PD controller.

## MuJoCo Ball Dynamics

Before every MuJoCo low-level step, the simulator will apply a world-frame
force to the ball equal to:

```text
F_drag = ball_mass * (-drag_coefficient * ||velocity|| * velocity)
```

MuJoCo already applies gravity, so gravity will not be added a second time.
The existing analytic table-bounce path will use the planner's exact table
geometry and horizontal/vertical restitution values.

This gives planner and simulator the same continuous acceleration equation and
the same discrete bounce rule at the same 5 ms interval. MuJoCo and the NumPy
predictor use different numerical integrators, so floating-point trajectories
are not required to be bit-for-bit identical; model equations and parameters
must be identical.

## Error Handling

If the configured ball body, freejoint, or geom cannot be found, HITTER ball
simulation remains disabled and logs the existing warning. A non-finite or
negative drag coefficient will fail configuration validation rather than apply
an invalid force.

## Verification

Automated checks will verify configuration and model parity without opening a
viewer:

1. Hydra composition selects the HITTER MuJoCo backend and enables table
   tennis.
2. Every shared table/ball physics value resolves equal to the corresponding
   `mimic.motion.ball_planner` value.
3. The MuJoCo drag-force calculation produces the same acceleration as the
   planner equation for representative velocities.
4. The table-bounce velocity update uses the same horizontal and vertical
   restitution equations as the planner.
5. Analytic racket hit remains disabled, so successful visual hits come from
   MuJoCo racket-ball contact.

Codex will run only these automated/static checks. The user will run the final
real-time viewer command and judge the motion and contact behavior.

## Non-Goals

- No real-world backend changes.
- No planner or policy observation changes.
- No Vicon estimator emulation in MuJoCo.
- No action gating.
- No analytic racket-hit velocity injection.
- No random trajectory suite in this first parity pass.
