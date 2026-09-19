# ChingMu Table Calibration and RobotBridge Adapter Design

## Goal

Replace the Vicon input adapter used by HITTER real-world deployment with a
single ChingMu adapter while preserving the existing RobotBridge planner and
policy interface. The adapter calibrates a table-fixed world frame from four
stationary table-corner markers, transforms the `G1Pelvis` rigid body and ball
samples into that frame, and publishes the existing `transformation_t` messages
on `vicon_state_data`.

The design deliberately keeps the existing LCM channel and canonical output
names so `simulator/real_world.py`, the planner, and the policy do not need a
second mocap-specific data path.

## Confirmed Runtime Inputs

- ChingMu Avatar host: `192.168.2.100`
- VRPN source address: `MCAvatar@192.168.2.100`
- Rigid-body name: exact, case-sensitive `G1Pelvis`
- Avatar source frame rate: 360 Hz
- ChingMu raw position unit: millimetres
- ChingMu room origin: placed on the table surface
- Four table-corner markers remain stationary and are sent as unlabeled
  markers
- The `G1Pelvis` coordinate origin must not be moved onto an individual marker;
  the Avatar prompt asking to place it at a selected marker is answered `No`

## Avatar VRPN Settings

The preferred adapter uses the official ChingMu Python SDK in aggregate mode.
Avatar is configured as follows:

- `启用Vrpn`: enabled
- `按照刚体名称发送`: disabled
- `未标记的Marker点`: enabled
- `刚体`: enabled
- `重定向`: disabled
- send rate: 360 Hz

The adapter reads the hierarchy and resolves `G1Pelvis` to its current numeric
body ID instead of hard-coding the ID. This remains correct if the rigid body is
deleted and recreated.

Before implementation relies on aggregate mode, a short live probe must verify
that the official SDK returns both the resolved rigid-body pose and unlabeled
marker samples under these settings. If that probe fails, the fallback is the
already verified named standard-VRPN receiver, without changing the table-frame
or LCM design below.

## Effective Table Geometry

The user chose the dimensions fitted from the four marker centres as the
effective table bounds:

- length: `2.730738 m`
- width: `1.512451 m`
- height above the RobotBridge ground plane: `0.760000 m`
- robot-side edge centre: `x = 0, y = 0`
- far edge: `x = 2.730738 m`
- table centre: `x = 1.365369 m, y = 0`
- lateral bounds: `y = +/-0.7562255 m`
- virtual hit plane: `x = 0`

The table markers determine the table centre, axes, and tilt. Their ChingMu
height near zero is expected because the ChingMu room origin is on the tabletop.
The fitted table plane is therefore translated to RobotBridge `z = 0.760000 m`.

## Startup Calibration

At adapter startup:

1. Wait for a valid `G1Pelvis` pose and a stable unlabeled-marker stream.
2. Collect a short stationary calibration window, nominally two seconds.
3. Cluster marker samples spatially and retain stable clusters.
4. Enumerate four-point subsets and select the rectangle whose pairwise-distance
   signature best matches the effective table length, width, and diagonals.
5. Fit the table plane and longitudinal direction using all four selected
   corner centres.
6. Choose longitudinal sign using the robot position: table `+x` points away
   from `G1Pelvis`, so the robot-side edge remains `x = 0`.
7. Choose the upward plane normal, then form the right-handed lateral axis so a
   robot facing into the table has forward `+x` and left `+y`.
8. Save the calibrated centre, axes, corners, dimensions, residuals, and source
   metadata to a ChingMu-specific JSON file.
9. Freeze the transform for the process lifetime. Calibration is not recomputed
   per frame.

The current live snapshot is a useful diagnostic reference, not a hard-coded
runtime transform:

- raw table centre: approximately `(0.004394, -0.001835, 0.001791) m`
- fitted plane-normal tilt: approximately `0.239 degrees`
- plane residual RMS: approximately `2.96 mm`
- transformed initial `G1Pelvis`: approximately
  `(-0.473, -0.074, 0.774) m`

## Coordinate Transform

Let `c_Q` be the fitted table centre in ChingMu coordinates and let
`e_x`, `e_y`, and `e_z` be the orthonormal table axes expressed in ChingMu
coordinates. For any ChingMu point `p_Q` in metres:

```text
delta = p_Q - c_Q
p_W.x = 0.5 * table_length + dot(delta, e_x)
p_W.y = dot(delta, e_y)
p_W.z = table_height + dot(delta, e_z)
```

This transform is applied upstream to both the robot base and the ball. The
planner never receives raw ChingMu coordinates.

## Robot Pose Handling

- Resolve the exact `G1Pelvis` body ID from the hierarchy.
- Convert its raw position with the table transform above.
- Capture its initial orientation at adapter startup.
- Publish yaw relative to that initial orientation, expressed in table axes;
  the initial robot heading is therefore zero without editing the rigid-body
  axes in Avatar.
- Publish a yaw-only quaternion in `[x, y, z, w]` order, matching the existing
  Vicon bridge behavior.
- Publish only valid, current rigid-body samples.
- Use canonical output name `G1Pelvis` regardless of the internal numeric body
  ID.

No hidden base-position anchor is applied. If a later measured physical offset
is required, it must be an explicit adapter configuration value.

## Table Markers and Ball Selection

Unlabeled marker IDs are not assumed to survive occlusion, so table corners are
not excluded by ID alone. After calibration, candidates close to the four saved
raw corner positions are excluded with a configurable spatial radius, default
`0.05 m`.

Remaining ball candidates must satisfy the already agreed real-world region:

- `0 <= x <= 2.730738 m`
- `abs(y) <= 0.7562255 m`
- `z > 0.760000 m`

The adapter then uses temporal continuity to select the moving ball from the
remaining candidates. Direction (`vx < 0`) and hit-plane eligibility remain
planner responsibilities; the adapter does not add action gating.

When no valid ball candidate exists, the adapter publishes an explicitly
invalid ball sample only in response to the current SDK observation. It does
not add a separate wall-clock stale-data timeout.

## LCM Compatibility

The adapter publishes the existing `transformation_t` schema on
`vicon_state_data`:

- `name = "G1Pelvis"` for the transformed base pose
- `name = "ball"` for a valid transformed ball sample
- `name = "table"` for the calibrated table centre
- position unit: metres
- quaternion order: `[x, y, z, w]`
- source frame/time derived from the 360 Hz ChingMu data
- `valid` and `occluded` set from current source validity

`simulator/real_world.py` remains source-agnostic and unchanged.

## MuJoCo and Planner Parity

The effective geometry must be updated consistently in:

- HITTER planner configuration
- MuJoCo table-tennis runtime defaults/configuration
- MuJoCo XML table top, centre line, net, and relevant leg placement

The shared values are:

```text
table_length = 2.730738
table_width = 1.512451
table_center_xy_w = [1.365369, 0.0]
table_height = 0.760000
virtual_hit_plane_x = 0.0
```

Bounce restitution, drag, ball radius, and racket physics are outside this
calibration change and remain unchanged.

## Failure Behavior

- Fewer than four stable table corners: do not start publishing; report the
  observed stable clusters.
- Rectangle or plane residual above tolerance: reject calibration and report
  measured dimensions/residuals.
- `G1Pelvis` absent or invalid: wait at startup; during runtime, do not publish
  a fabricated base pose.
- Marker stream absent: do not publish a fabricated ball.
- Saved calibration metadata does not match the current source or dimensions:
  require explicit recalibration rather than silently reusing it.

## Verification

Automated tests use standard-library `unittest` and cover:

- choosing the correct four table corners in the presence of extra markers
- full 3D plane fitting and right-handed axes
- longitudinal sign chosen from robot side
- transforming all four corners to the expected effective bounds
- mapping the table plane to `z = 0.760000 m`
- rejecting/excluding table-corner markers from ball candidates
- enforcing the agreed table-region candidate bounds
- initial relative robot yaw equal to zero
- canonical LCM names, units, and quaternion order

Live verification is non-actuating:

1. Run the adapter without the robot policy.
2. Confirm four calibrated corners, dimensions, and residuals.
3. Confirm `G1Pelvis` is nonzero and maps near its measured table-relative pose.
4. Confirm stationary table markers never appear as `ball`.
5. Move a ball through the table volume and confirm continuous transformed
   samples at the expected rate.
6. Only after those checks should the normal real-world RobotBridge chain be
   started.

## Non-Goals

- No planner or policy redesign.
- No new action gating.
- No per-frame table-frame recalibration.
- No dependence on persistent unlabeled-marker IDs.
- No automatic modification of the Avatar `G1Pelvis` origin or axes.
