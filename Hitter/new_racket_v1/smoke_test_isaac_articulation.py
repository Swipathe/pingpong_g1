"""Spawn the active HITTER G1 asset and verify its racket rigid body."""

from isaaclab.app import AppLauncher


app = AppLauncher(headless=True).app

from isaaclab.assets import Articulation  # noqa: E402
from isaaclab.sim import build_simulation_context  # noqa: E402
from whole_body_tracking.robots.g1 import G1_HITTER_RACKET_CFG  # noqa: E402


with build_simulation_context(device="cuda:0", add_ground_plane=False, auto_add_lighting=False) as sim:
    sim._app_control_on_stop_handle = None
    robot = Articulation(G1_HITTER_RACKET_CFG.replace(prim_path="/World/Robot"))
    sim.reset()
    if not robot.is_initialized:
        raise RuntimeError("HITTER articulation did not initialize")
    if "right_racket_link" not in robot.body_names:
        raise RuntimeError(f"right_racket_link missing from body names: {robot.body_names}")
    print(
        "ISAAC_ARTICULATION_SMOKE_PASS",
        f"asset={G1_HITTER_RACKET_CFG.spawn.usd_path}",
        f"bodies={len(robot.body_names)}",
        f"joints={len(robot.joint_names)}",
        f"racket_body_index={robot.body_names.index('right_racket_link')}",
        flush=True,
    )

app.close()
