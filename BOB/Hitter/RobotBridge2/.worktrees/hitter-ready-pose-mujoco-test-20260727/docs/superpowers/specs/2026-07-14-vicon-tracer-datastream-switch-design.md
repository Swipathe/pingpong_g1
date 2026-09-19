# Vicon Tracer DataStream Runtime Switch Design

## Goal

Switch the HITTER real-world mocap producer from the ChingMu bridge back to
Vicon DataStream while keeping the ChingMu implementation available. Tracer's
`G1Pelvis` rigid-body centroid is the pelvis origin. The downstream LCM,
estimator, planner, policy, and WAITING lifecycle contracts remain unchanged.

## Confirmed physical setup

- Data source: Tracer through the Vicon DataStream SDK.
- Rigid-body subject name: exact, case-sensitive `G1Pelvis`.
- Tracer exposes the rigid-body centroid as the root segment translation.
- The Tracer/Vicon world origin is already located at the midpoint of the
  robot-side table edge.
- Four stationary unlabeled markers are installed at the four table corners.
- During calibration, those four corners are the only unlabeled markers; the
  ball and all other unlabeled markers are removed.
- Effective table geometry remains:

```text
length = 2.730738 m
width  = 1.512451 m
height = 0.760000 m
```

The four-corner calibration validates the existing Tracer origin and corrects
small axis or table-plane errors. The RobotBridge frame always maps the table
surface to `z = 0.760000 m`, regardless of whether Tracer's raw `z = 0` lies on
the tabletop or at floor height.

## Chosen approach

Modify the existing
`deploy/mocap_bridge/vicon_table_lcm_bridge.cpp`. Do not create a second
Tracer-specific bridge and do not remove the ChingMu bridge.

Only one process may publish `vicon_state_data` at a time. The runtime switch
therefore consists of stopping the ChingMu bridge and starting the Vicon bridge.
There is no automatic failover between sources.

## Architecture and unchanged interface

```text
Tracer / Vicon DataStream
        |-- G1Pelvis root segment centroid and quaternion
        `-- unlabeled markers
                         |
                         v
          vicon_table_lcm_bridge.cpp
        |-- table-frame transform
        |-- table-corner exclusion
        `-- ball tracking
                         |
                         v
                vicon_state_data
                         |
                         v
       RealWorld -> estimator -> planner -> policy
```

The bridge continues to publish the existing `transformation_t` messages on
`vicon_state_data`:

- `name = "G1Pelvis"` for the transformed pelvis pose;
- `name = "ball"` for the selected unlabeled ball marker;
- `name = "table"` for the fixed table reference.

No field, channel, unit, or downstream consumer changes are permitted as part
of this switch.

## Direct `G1Pelvis` root pose

After connecting, the bridge resolves the root segment with
`GetSubjectRootSegmentName("G1Pelvis")`. Every frame then reads:

```text
GetSegmentGlobalTranslation(subject, root_segment)
GetSegmentGlobalRotationQuaternion(subject, root_segment)
```

The root translation is Tracer's rigid-body centroid and is used directly as
the pelvis origin. The bridge must not:

- fit a pose from the labelled markers;
- use the marker centroid as a fallback;
- add the ChingMu pelvis offset;
- replace the measured position with a configured base anchor.

The raw translation is converted from millimetres into the calibrated table
frame. At the intended normal stance, the target x/y position is `(-0.4, 0)`;
the measured centroid supplies `z_pelvis`.

The first valid root quaternion after startup defines the zero heading. Let
`R_TB(0)` be that initial body orientation in the table frame and `R_TB(t)` be
the current orientation. The bridge computes the relative rotation

```text
R_relative(t) = R_TB(t) * transpose(R_TB(0))
```

and publishes only its yaw component. Thus a robot initially facing the table
publishes yaw zero, while later turns remain observable. Roll and pitch continue
to come from the robot IMU rather than mocap.

## Four-corner table calibration

Calibration requires a valid `G1Pelvis` root pose and exactly four stable
unlabeled corner clusters. The default sample window is 2.0 seconds and remains
configurable through `--calib-sec`.

The four points define:

1. the table centre;
2. the fitted table-plane normal;
3. the long axis;
4. the short axis.

The normal sign is chosen upward, using the pelvis position relative to the
table plane. The long-axis sign is chosen so that positive x points away from
the robot. The right-handed RobotBridge convention is then:

```text
robot-side edge: x = 0
far-side edge:   x = 2.730738
table centre:    y = 0
table surface:   z = 0.760000
+y:              z cross +x
```

Because the Tracer origin is already at the robot-side edge midpoint, the
calibrated x/y origin and yaw should be close to the raw Tracer definition.
The calibration still remains authoritative so small placement, levelling, or
axis errors do not leak into the planner frame.

The first run writes a candidate file rather than overwriting the accepted
calibration. The candidate stores the raw corners, centre, all three axes,
table dimensions, height, and rectangle residual. Loading the file restores
the saved length, width, and height so runtime flags cannot silently select a
different geometry.

Candidate acceptance requires the best one-to-one assignment of the four
transformed corners to agree with the four expected positions within a maximum
Euclidean error of 50 mm:

```text
(0,          -width/2, height)
(0,          +width/2, height)
(table_length, -width/2, height)
(table_length, +width/2, height)
```

It also requires the standing `G1Pelvis` to be on the robot side (`x < 0`) and
the reported axes to form a finite right-handed orthonormal frame. Promotion to
the accepted Vicon calibration is an explicit step after these checks; it is
never automatic.

## Ball marker handling

Unlabeled markers remain the ball source. Every runtime candidate within
50 mm of a saved table-corner raw position is removed before ball selection, so
the four calibration markers may remain installed without being treated as the
ball. Existing temporal nearest-track selection remains in place for the
remaining candidates.

The bridge uses the actual DataStream frame rate returned by `GetFrameRate()`
for frame timestamps and frame-delta velocity calculations. The existing
`--vicon-frame-rate-hz` value is retained only as an explicit fallback when the
SDK cannot report a finite positive rate.

## Failure behavior

- DataStream connection or first-frame timeout: exit nonzero.
- Missing `G1Pelvis` subject or root segment: exit nonzero before calibration or
  publication.
- Root pose occluded or invalid during runtime: publish `G1Pelvis` with
  `valid=0` and `occluded=1`; do not synthesize a pose.
- Invalid base frame: do not publish a ball that could trigger planning for that
  frame.
- Fewer or more than four stable calibration clusters, non-finite geometry,
  non-right-handed axes, or corner residual above 50 mm: reject calibration and
  leave the accepted file untouched.
- Multiple mocap publishers: treat as an operational error; stop the ChingMu
  bridge before starting Vicon publication.

## Verification

### Build and deterministic checks

- Rebuild all Vicon DataStream C++ tools with the installed SDK.
- Add deterministic coverage for table-frame construction, direct segment pose
  transformation, initial-yaw removal, corner exclusion, saved-dimension
  loading, and invalid-data handling.
- Run the existing RealWorld and HITTER lifecycle/realtime tests to prove the
  unchanged LCM contract and downstream behavior.

### Live checks without robot commands

1. Use the DataStream probe to list `G1Pelvis`, resolve its root segment, and
   report the SDK frame rate.
2. Verify finite, non-occluded root translation and quaternion samples.
3. Generate a candidate calibration with publication disabled.
4. Confirm the four expected table-corner coordinates and standing pelvis
   position near `(-0.4, 0, z_pelvis)`.
5. Move the robot manually and confirm x, y, and relative-yaw signs.
6. Keep the corner markers present, move a ball, and confirm only the ball is
   selected and its table-frame trajectory is continuous.
7. Start Vicon publication only, then inspect `G1Pelvis`, `ball`, and `table`
   using `monitor_vicon_lcm.py`.

Real-world policy startup happens only after these checks pass. Calibration and
bridge validation must not send robot commands.

## Out of scope

- Removing or rewriting the ChingMu bridge.
- Automatic source switching or failover.
- Changes to `real_world.py`, the estimator, planner, policy observations,
  WAITING target, TTS, or swing lifecycle.
- Driving the real robot during table calibration.
