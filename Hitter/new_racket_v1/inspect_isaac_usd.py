"""Headless inspection of the generated G1 hitter USD asset."""

import argparse

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser()
parser.add_argument("usd_path")
args = parser.parse_args()

app = AppLauncher(headless=True).app

from pxr import Usd  # noqa: E402


stage = Usd.Stage.Open(args.usd_path)
if stage is None:
    raise RuntimeError(f"Could not open USD stage: {args.usd_path}")

print(f"STAGE_OPEN_OK default_prim={stage.GetDefaultPrim().GetPath()}", flush=True)
matched_prims = []
for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
    path = str(prim.GetPath())
    if path.startswith("/g1/right_racket_link") or "right_racket_fixed_joint" in path:
        matched_prims.append(prim)
        attrs = {
            attr.GetName(): attr.Get()
            for attr in prim.GetAttributes()
            if attr.GetName().startswith("physics:") or attr.GetName().startswith("xformOp:")
        }
        print(
            f"PRIM type={prim.GetTypeName()} path={path} schemas={prim.GetAppliedSchemas()} attrs={attrs}",
            flush=True,
        )

joint = stage.GetPrimAtPath("/g1/joints/right_racket_fixed_joint")
link = stage.GetPrimAtPath("/g1/right_racket_link")
if not joint or not link:
    raise RuntimeError("Generated USD is missing the racket joint or rigid body")

joint_pos = tuple(float(x) for x in joint.GetAttribute("physics:localPos0").Get())
joint_quat = joint.GetAttribute("physics:localRot0").Get()
mass = float(link.GetAttribute("physics:mass").Get())
com = tuple(float(x) for x in link.GetAttribute("physics:centerOfMass").Get())
if max(abs(a - b) for a, b in zip(joint_pos, (0.22279, 0.00685, -0.00291))) > 1e-7:
    raise RuntimeError(f"Unexpected racket joint position: {joint_pos}")
if abs(abs(float(joint_quat.GetImaginary()[0])) - 1.0) > 1e-7 or abs(float(joint_quat.GetReal())) > 1e-6:
    raise RuntimeError(f"Unexpected racket joint rotation: {joint_quat}")
if abs(mass - 0.36624) > 1e-7:
    raise RuntimeError(f"Unexpected racket mass: {mass}")
if max(abs(a - b) for a, b in zip(com, (-0.04789, 0.00685, -0.00083))) > 1e-7:
    raise RuntimeError(f"Unexpected racket center of mass: {com}")

visual_meshes = [
    prim for prim in matched_prims
    if prim.GetTypeName() == "Mesh" and "/visuals/" in str(prim.GetPath())
]
collision_meshes = [
    prim for prim in matched_prims
    if prim.GetTypeName() == "Mesh" and "/collisions/" in str(prim.GetPath())
]
if len(visual_meshes) != 3 or len(collision_meshes) != 1:
    raise RuntimeError(
        f"Unexpected racket mesh counts: visuals={len(visual_meshes)}, collisions={len(collision_meshes)}"
    )
print(
    "ISAAC_USD_VALIDATION_PASS",
    f"visuals={len(visual_meshes)}",
    f"collisions={len(collision_meshes)}",
    f"mass={mass}",
    flush=True,
)

app.close()
