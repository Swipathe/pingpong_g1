# HITTER RobotBridge Integration

This document records how HITTER is inserted into the local RobotBridge infra.
The integration follows the existing RobotBridge style: one top-level HITTER
Hydra entry point, with the simulator backend selected through the `sim`
config group. The real-world backend consumes Vicon ball and root data through
the existing `real_world` simulator LCM path. The standalone `mocap_bridge`
tools are still treated as external source adapters and are not copied in this
pass.

## Runtime Chain

```text
deploy/run.py
  -> deploy/config/hitter.yaml
    -> agents.hitter_agent.HitterAgent
      -> envs.hitter.HitterEnv
        -> utils.hitter_planner.HitterSystemPlanner
        -> sim=mujoco              -> simulator.mujoco.Mujoco
        -> sim=real_world          -> simulator.real_world.RealWorld
```

`deploy/run.py` stays generic. HITTER enters through Hydra composition, not
through a side script or a special-case branch in the runner.

## Config Entry Points

- `deploy/config/hitter.yaml`
  - Single top-level HITTER Hydra profile.
  - Composes `robot: g1_hitter_racket`, `sim: real_world`, `env: hitter`,
    `agent: hitter`, and `mimic: hitter`.
  - Switches to MuJoCo with the standard Hydra override `sim=mujoco`, rather
    than a second top-level config file or HITTER-specific simulator group.

- `deploy/config/agent/hitter.yaml`
  - Instantiates `agents.hitter_agent.HitterAgent`.
  - Passes the environment object and policy checkpoint path.

- `deploy/config/env/hitter.yaml`
  - Instantiates `envs.hitter.HitterEnv`.
  - Passes simulator, observation config, control config, motion/planner config,
    and policy config.

- `deploy/config/mimic/hitter.yaml`
  - Defines policy runtime settings and HITTER planner settings.
  - Uses `./data/model/hitter/hitter.onnx` as the default local ONNX path.
  - Keeps `update_command_while_armed: false`, so an armed strike command is
    locked instead of being overwritten every WBC frame.
  - Keeps `clip_racket_velocity_to_training_range: true`, so planner-generated
    racket velocity is clipped before entering the policy observation.

- `deploy/config/robot/g1_hitter_racket.yaml`
  - Selects HITTER-specific asset and control fragments.
  - Uses `/control: g1_hitter_racket` to avoid changing the shared
    `g1_29dof` control profile.

- `deploy/config/asset/g1_hitter_racket.yaml`
  - Selects `g1_29dof_hitter_racket_table_tennis.xml`.
  - Defines 29 active DOFs and the HITTER racket joint order/default gains.

- `deploy/config/control/g1_hitter_racket.yaml`
  - HITTER-specific control rates, clipping, torques, and FK settings.
  - Keeps `use_teleop: false` because HITTER uses planner commands instead of
    the teleop reference stream.
  - Keeps `terminate_on_r2: false` for HITTER real-world deployment.
  - Exists separately so other profiles that use `g1_29dof` are not affected.

## Runtime Commands

Default real-world HITTER run:

```bash
cd deploy
python run.py --config-name=hitter
```

MuJoCo HITTER run through the same top-level config:

```bash
cd deploy
python run.py --config-name=hitter sim=mujoco
```

Optional headless or faster-than-realtime MuJoCo overrides:

```bash
cd deploy
python run.py --config-name=hitter sim=mujoco robot.control.viewer=false robot.control.real_time=false
```

## Code Responsibilities

### `deploy/agents/hitter_agent.py`

`HitterAgent` is the policy-loop wrapper.

Responsibilities:

- Load the HITTER ONNX policy.
- Read policy metadata from ONNX and call
  `HitterEnv.configure_from_modelmeta(...)`.
- Interpolate from current joint state to the policy default pose at startup.
- Hold the default policy action while the ball planner has not armed a valid
  command.
- Run the policy and pass actions to `env.step(...)` at the normal RobotBridge
  control cadence.

It does not compute ball trajectories, assemble observations, or talk directly
to MuJoCo internals.

### `deploy/envs/hitter.py`

`HitterEnv` owns the HITTER policy interface.

Responsibilities:

- Map simulator joint order to policy joint order using ONNX metadata.
- Assemble the 105-D HITTER observation:
  - base angular velocity
  - projected gravity
  - base forward direction
  - base target position
  - racket target position
  - racket target velocity
  - time to strike
  - joint position residual
  - joint velocity
  - previous policy action
- Manage HITTER command state:
  - `hitter_command_initialized`
  - `hitter_strike_elapsed_s`
  - `hitter_strike_time_s`
  - `hitter_strike_duration_s`
- Arm a planner command only inside the configured time-to-strike window.
- Keep armed commands locked when `update_command_while_armed` is false.
- Clip racket velocity to the configured training range.
- Convert policy actions back into simulator joint order before physics step.

It uses simulator-facing methods such as `reset_hitter_ball`,
`ball_pos_world`, `ball_vel_world`, and `set_hitter_analytic_racket_hit`
instead of directly owning simulator physics.

### `deploy/utils/hitter_planner.py`

The HITTER planner is pure computation.

Responsibilities:

- Estimate ball state from recent samples.
- Detect table bounce conditions.
- Predict ball trajectory with gravity, drag, and restitution.
- Find the virtual hit-plane intersection.
- Compute desired outgoing ball velocity toward the configured landing point.
- Convert incoming/outgoing ball velocity into racket target velocity.
- Choose forehand/backhand from target geometry or forced config.
- Compute base target and racket target.
- Return `HitterWbcCommand`.

It has no LCM, MuJoCo stepping, policy execution, or robot-control side effects.

### `deploy/simulator/mujoco.py`

`Mujoco` remains the simulator backend, with HITTER table-tennis support added
behind simulator-level interfaces.

Added HITTER-facing interfaces:

- `ball_pos_world`
- `ball_vel_world`
- `reset_hitter_ball(...)`
- `set_hitter_analytic_racket_hit(...)`
- `clear_hitter_analytic_racket_hit()`

Added simulator behaviors:

- Load HITTER ball body, ball freejoint, ball geom, and racket-face geom from
  the XML model.
- Update ball state after MuJoCo state reads and resets.

This keeps HITTER physics-specific behavior inside the simulator layer rather
than moving it into the environment or runner.

### `deploy/simulator/real_world.py`

`RealWorld` remains the robot/LCM simulator backend, with HITTER ball-state
support added behind the same simulator-level interface used by MuJoCo.

Added HITTER-facing state:

- `ball_pos_world`
- `ball_vel_world`
- `ball_visible`
- `ball_state_estimator_ready`
- `ball_state_estimator_sample_count`

Added real-world behaviors:

- Subscribe to the existing `vicon_state_data` LCM channel.
- Route messages whose name contains `ball` into `BallStateEstimator`.
- Ignore table messages.
- Treat all other Vicon messages as robot root pose updates.
- Reject implausible raw ball jumps.
- Drop stale ball estimates when samples stop arriving.
- Expose `reset_ball_state_estimator()` for `HitterEnv` live safety filters.
- Allow `terminate_on_r2: false` for HITTER real deployment.

This keeps live Vicon parsing and filtering inside the real-world simulator
adapter rather than placing it in the policy runner or planner.

## Assets and Model

- `deploy/data/assets/g1/g1_29dof_hitter_racket_table_tennis.xml`
  - G1 29-DOF model with right-hand racket, table, and free ball.

- `deploy/data/assets/g1/meshes/*hitter_frame.stl`
  - Racket and grip meshes referenced by the HITTER XML.

- `deploy/data/model/hitter/hitter.onnx`
  - Local default HITTER ONNX used by `mimic/hitter.yaml`.
  - This path is ignored by `.gitignore` because `deploy/data/model` is already
    treated as local model storage in this repo.

## Explicitly Not Integrated

The following are intentionally excluded from this pass:

- `loco_hitter_agent.py`
- `envs/loco_hitter.py`
- `config/loco_hitter*.yaml`
- `deploy/logs`
- offline videos
- tmux logs
- compiled `mocap_bridge/bin`
- standalone `mocap_bridge` source/package wiring

The real-world runtime path now expects an external adapter to publish the
existing `vicon_state_data` LCM messages:

```text
Nexus/Vicon
  -> external mocap bridge
    -> vicon_state_data LCM
      -> real_world simulator ball state
      -> HitterEnv planner
        -> HitterAgent policy loop
```

## Verification Performed

- Python compile check passed for:
  - `deploy/agents/hitter_agent.py`
  - `deploy/envs/hitter.py`
  - `deploy/utils/hitter_planner.py`
  - `deploy/simulator/mujoco.py`

- Hydra compose check passed for `hitter.yaml sim=mujoco`:
  - agent target: `agents.hitter_agent.HitterAgent`
  - env target: `envs.hitter.HitterEnv`
  - simulator target: `simulator.mujoco.Mujoco`
  - asset file: `g1_29dof_hitter_racket_table_tennis.xml`
  - table tennis enabled: `True`

- Hydra compose check passed for `hitter.yaml`:
  - agent target: `agents.hitter_agent.HitterAgent`
  - env target: `envs.hitter.HitterEnv`
  - simulator target: `simulator.real_world.RealWorld`
  - motion config is passed to the real-world simulator

- MuJoCo XML load check passed:
  - model loads successfully
  - key HITTER bodies/geoms are present

- ONNX check passed:
  - input: `obs [1, 105]`
  - output: `actions [1, 29]`
  - metadata contains 29 joint names and required control metadata fields
