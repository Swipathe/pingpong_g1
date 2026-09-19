# New racket asset transform

These STL files are transformed copies of the SolidWorks exports in
`/home/yhl/Downloads`. The source files remain unchanged.

## Frames

- Parent/source frame: `right_wrist_yaw_link_frame`
- Child/output frame: `right_racket_link_frame`

The child frame pose in the parent frame is:

```text
translation_m = [0.22279, 0.00685, -0.00291]
rotation_parent_from_child =
  [ 1,  0,  0]
  [ 0, -1,  0]
  [ 0,  0, -1]
```

Equivalent representations:

```text
URDF rpy_rad = [3.141592654, 0, 0]
MuJoCo quaternion_wxyz = [0, 1, 0, 0]
```

Vertices and normals were transformed as:

```text
p_child = R^T * (p_parent - translation)
n_child = R^T * n_parent
```

## Files

```text
/home/yhl/Downloads/connector_visual.STL
  -> connector_visual.stl
/home/yhl/Downloads/racket_holder_visual.STL
  -> racket_holder_visual.stl
/home/yhl/Downloads/racket_visual.STL
  -> racket_visual.stl
```

Source SHA-256:

```text
connector_visual.STL      56bf4cbbd4dc8e5c70da056ecc5ea7a4c0f2db3a8adef5f6dbbb5d38813aecee
racket_holder_visual.STL  a9131a862bb76abce3742eb3fd41c1b5d36da46705dbad9742809858725565a3
racket_visual.STL         82a4ea992cd4f822e4559d4f217978d3fcd03045ab63d8142d240ea06932f823
```

Output SHA-256:

```text
connector_visual.stl      1f3fbdbaf33fd720a6ffa28a83cb01b865bcc617dcfd0f00c277fb732c5caf54
racket_holder_visual.stl  bbbe69df456fc2cbf81f19fc2a80a9c4418ef19d6565ce11723a00ed79f4ddb7
racket_visual.stl         516327de42ef02558ddda45ba6e357843034648d3be2132e5f2f5ea55aabb425
```

## Verification

- Triangle counts and STL attributes are unchanged.
- Maximum forward-transform error: `6.55e-9 m`.
- Maximum inverse-transform error: `6.55e-9 m`.
- All three outputs have no boundary edges at a `1e-8 m` weld tolerance.
- Combined output bounds:
  - minimum: `[-0.194289818, -0.020649996, -0.076164559] m`
  - maximum: `[0.070772111, 0.034350004, 0.075523056] m`
