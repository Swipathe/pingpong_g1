# New Rigid Racket Assets Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Install the new rigid connector-holder-racket assembly in the active Isaac Lab and MuJoCo assets.

**Architecture:** Keep one fixed child body, `right_racket_link`, below `right_wrist_yaw_link`. Attach three identity-pose visual meshes and one racket collision mesh, with the same transform and temporary aggregate inertial properties in URDF/USD and MJCF.

**Tech Stack:** URDF, MJCF/XML, binary STL, Isaac Lab URDF converter, MuJoCo Python bindings.

## Global Constraints

- Preserve the three original files in `/home/yhl/Downloads` and transformed masters in `/home/yhl/Desktop/Hitter/new_racket_v1`.
- Preserve unrelated local RobotBridge changes.
- Do not add joints or degrees of freedom.
- Use `xyz="0.22279 0.00685 -0.00291"` and URDF `rpy="3.141592654 0 0"`.
- Mark the uniform-density mass and inertia as temporary.

---

### Task 1: Establish failing asset assertions

**Files:**
- Inspect: `MOSAIC-main/source/whole_body_tracking/whole_body_tracking/assets/unitree_description/urdf/g1_hitter_racket/main.urdf`
- Inspect: `RobotBridge2_20260722_1726/deploy/data/assets/g1/g1_29dof_hitter_racket_table_tennis.xml`

- [ ] Run read-only assertions for the three new mesh names, new fixed transform, temporary mass, and single collision mesh.
- [ ] Confirm the assertions fail because the old attachment is still installed.

### Task 2: Update MOSAIC source asset

**Files:**
- Create: three STL copies under `MOSAIC-main/source/whole_body_tracking/whole_body_tracking/assets/unitree_description/meshes/g1_hitter_racket/`
- Modify: `MOSAIC-main/source/whole_body_tracking/whole_body_tracking/assets/unitree_description/urdf/g1_hitter_racket/main.urdf`
- Generate: Isaac USD output used by `whole_body_tracking/robots/g1.py`

- [ ] Copy the three transformed STL files without changing their bytes.
- [ ] Replace the old fixed transform, inertial block, two visuals, and collision with the approved definitions.
- [ ] Convert the URDF using zero drive stiffness/damping and no fixed-joint merging.
- [ ] Load the generated stage headlessly and verify the racket body, joint, visuals, collision, mass, and transform.

### Task 3: Update active RobotBridge MJCF asset

**Files:**
- Create: three STL copies under `RobotBridge2_20260722_1726/deploy/data/assets/g1/meshes/`
- Modify: `RobotBridge2_20260722_1726/deploy/data/assets/g1/g1_29dof_hitter_racket_table_tennis.xml`

- [ ] Copy the same three transformed STL files without changing their bytes.
- [ ] Replace only the mesh declarations and `right_racket_link` block, retaining existing table and ball-contact edits.
- [ ] Preserve `right_racket_face_collision` so the existing contact pair remains valid.
- [ ] Compile the XML with MuJoCo and verify body, geom, mass, COM, full inertia, and transform.

### Task 4: Cross-backend verification

**Files:**
- Verify all files modified or created by Tasks 2 and 3.

- [ ] Re-run the initial asset assertions and confirm they pass.
- [ ] Compare SHA-256 hashes of copied meshes with `new_racket_v1`.
- [ ] Check both XML documents parse successfully.
- [ ] Confirm both backends use one rigid body, three visual meshes, one racket collision, and identical aggregate dynamics.
- [ ] Record that dynamics remain temporary pending real material or measured mass data.

