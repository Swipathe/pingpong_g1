# bvh1h_g1_npz

MOSAIC-compatible Unitree G1 motion npz files converted from `/home/sijie/Downloads/bvh1h`.

Coordinate handling:
- Source BVH is parsed as centimeters with Y-up.
- The conversion applies the same GMR BVH rotation matrix used by `load_bvh_file`: BVH Y-up -> MOSAIC/Isaac Z-up.
- Source 100 fps BVH is downsampled on the time axis to 30 fps before retargeting.
- The bvh1h skeleton uses `LeftToeBase`/`RightToeBase`; those are used for `LeftFootMod`/`RightFootMod`.
- The bvh1h skeleton has `Spine1` but no `Spine2`; `Spine2` is aliased to `Spine1` for the GMR IK config.
- A constant per-motion z shift is applied so the lowest generated G1 body point is on ground z=0.

Output fields:
- `fps`
- `joint_pos`, `joint_vel`
- `body_pos_w`, `body_quat_w`
- `body_lin_vel_w`, `body_ang_vel_w`

`body_quat_w` is saved in Isaac/MOSAIC `wxyz` order.
