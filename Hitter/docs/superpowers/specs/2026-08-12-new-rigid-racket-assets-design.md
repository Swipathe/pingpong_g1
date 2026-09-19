# New Rigid Racket Assets Design

## Goal

Replace the existing scanned hand-and-paddle attachment with the new connector,
racket holder, and racket assembly in both MOSAIC/Isaac Lab and RobotBridge/MuJoCo.

## Confirmed geometry and frame

- The three new components are rigidly fixed and remain one simulation body named
  `right_racket_link`.
- The fixed transform from `right_wrist_yaw_link` to `right_racket_link` is
  `xyz = 0.22279 0.00685 -0.00291 m` and `rpy = pi 0 0`.
- The mesh vertices and normals have already been transformed into
  `right_racket_link_frame` and are stored under `new_racket_v1/`.
- Visual geometry consists of `connector_visual.stl`,
  `racket_holder_visual.stl`, and `racket_visual.stl` at identity pose in the
  racket link.
- Only `racket_visual.stl` is used for first-version ball collision.

## Temporary dynamics

Use the SolidWorks uniform-density result for the first integration:

- mass: `0.36624 kg`
- center of mass: `(-0.04789, 0.00685, -0.00083) m`
- inertia at the center of mass, expressed in `right_racket_link_frame`:
  - `Ixx = 0.00044141196 kg m^2`
  - `Ixy = 0.00000000074 kg m^2`
  - `Ixz = 0.00002668735 kg m^2`
  - `Iyy = 0.00241312565 kg m^2`
  - `Iyz = -0.00000000161 kg m^2`
  - `Izz = 0.00199711442 kg m^2`

These values are explicitly temporary because SolidWorks used approximately
`1000 kg/m^3` for every component. They must be replaced before final training
when real material densities or measured masses are available.

## Rendering and contact

- Connector visual: neutral metallic gray.
- Holder visual: dark gray.
- Racket visual: light neutral color for the first version because STL has no
  per-face material information.
- Isaac Lab retains its task-level robot friction/restitution configuration.
- MuJoCo retains the existing named `right_racket_face_collision` contact pair,
  friction, `solref`, and `solimp` values.

## Scope and validation

- Modify only the active MOSAIC hitter asset and the active
  `RobotBridge2_20260722_1726` deployment asset.
- Preserve unrelated user changes in the dirty RobotBridge worktree.
- Validate mesh files, URDF/XML syntax, fixed transform, mass/inertia tensor,
  collision count, MuJoCo model compilation, and Isaac USD conversion/loading.

