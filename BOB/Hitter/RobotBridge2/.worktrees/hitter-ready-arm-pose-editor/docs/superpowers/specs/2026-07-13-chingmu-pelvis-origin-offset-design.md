# ChingMu Pelvis Origin Offset Design

## Goal

Publish the training URDF `pelvis` link origin as `G1Pelvis` while keeping the
existing ChingMu table frame, ball tracker, LCM schema, planner, and policy
interfaces unchanged.

## Calibrated transform

The nine labelled rigid-body markers define the ChingMu `G1Pelvis` rigid
origin. Four markers are on the front face and five are on the rear face. Two
temporary unlabeled markers, one on the right face and one on the top face,
locate the training pelvis origin. All marker centers are approximately 6 mm
outside the robot shell.

The resulting translation from the ChingMu rigid origin to the training URDF
pelvis origin is:

```text
[+0.003145, +0.044074, +0.048231] m
```

The offset uses RobotBridge heading axes:

- `+X`: robot forward, toward the table at startup
- `+Y`: robot left
- `+Z`: upward

The temporary right and top markers may be removed after calibration.

## Runtime behavior

The bridge already records the initial rigid-body yaw and publishes yaw
relative to that startup heading. Let `R_heading` be that relative-yaw
rotation. The corrected base position is:

```text
p_pelvis_world = p_rigid_world + R_heading * offset_heading
```

The same `R_heading` remains the published base quaternion. This rotates the
translation when the robot turns without depending on the arbitrary Avatar
rigid-body axis convention. Roll and pitch remain intentionally excluded,
matching the existing heading-only RobotBridge base contract.

## Boundaries

- Change only the published `G1Pelvis` position.
- Do not change raw-to-table calibration.
- Do not change ball selection or ball physics.
- Do not change LCM fields or channels.
- Do not change planner or policy observations.
- Keep `G1Pelvis` invalid-frame behavior unchanged.

## Verification

Automated tests must prove that the offset is applied unchanged at zero
relative yaw and rotates by 90 degrees with relative yaw. The complete ChingMu
bridge test suite must remain green before restarting the live bridge.
