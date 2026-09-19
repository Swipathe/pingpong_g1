# bvh1h NPZ Processing Report

- Output root: `/home/sijie/Stage1_recovery_20260408/MOSAIC-main/data/motions/bvh1h_g1_npz`
- Motions converted or present: 217
- Failures: 0
- Frame count min/max/mean: 499 / 6542 / 1317.34
- Ground shift z min/max: -0.142883 / 0.048157
- Elapsed seconds: 270.5

Coordinate alignment:
- BVH centimeters are converted to meters.
- BVH Y-up is rotated to MOSAIC/Isaac Z-up.
- `Spine2` is aliased to `Spine1` for this skeleton.
- `LeftToeBase`/`RightToeBase` provide the foot orientation targets.
- Output quaternions are saved as `wxyz`.
- A constant per-motion z shift puts the lowest generated G1 body point at ground z=0.
