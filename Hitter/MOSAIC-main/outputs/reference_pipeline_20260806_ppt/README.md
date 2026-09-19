# Reference Motion Processing — F7 + B7

Presentation figure made from the user's selected frames in `20260806_mqy_capture`.
All labels are English. Human meshes and robot poses are rendered from saved
reconstruction/motion data; no generated or manually adjusted poses are used.

## Deliverables

- `reference_pipeline_editable.pptx`: editable PowerPoint, with 6 independent PNG
  pictures, 16 native text boxes, 2 native arrows and 3 native rounded panels.
  Picture bytes are preserved exactly. The slide uses the original 10:3 figure
  aspect ratio and Arial text. Object names are descriptive in the Selection Pane.
- `reference_pipeline_editable_preview.png`: preview rendered from the PPTX
  through LibreOffice, to verify layout.
- `reference_pipeline.png`: 4200 × 1260, white background, suitable for PowerPoint.
- `reference_pipeline.svg`: self-contained images, editable vector labels/arrows/layout.
- `reference_pipeline.pdf`: raster PDF export of the figure.
- `reference_pipeline_preview.png`: smaller preview.
- `renders/F7_video.png`, `renders/B7_video.png`: selected video images.
- `renders/F7_reconstruction.png`, `renders/B7_reconstruction.png`: actual SMPL-X overlays.
- `renders/F7_robot.png`, `renders/B7_robot.png`: G1 + paddle renders.
- `renders/*_rgba.png`: isolated human/robot layers with alpha channels.
- `render_manifest.json`: frame correspondence, input hashes, camera parameters and FK checks.

## Edit in PowerPoint

Open `reference_pipeline_editable.pptx`. Click text to edit its wording and
formatting, or select an arrow/panel to change its color, outline, or size. The
six pictures are independent picture objects, not part of a flattened slide.
To move the full diagram into an existing presentation, select all objects on
the slide, copy and paste them onto the destination slide, and group them before
resizing proportionally. Ungroup again for individual edits. The slide notes
contain the image/frame provenance.

`make_editable_pptx.py` reproduces the PPTX with `python-pptx`. It checks object
types, image byte equality, slide boundaries and ZIP integrity after saving.
Results are recorded in `editable_pptx_validation.json`.

## Selected frames (zero-based)

| Selection | Clip | Video / GVHMR / pre-alignment NPZ frame | Clip time | Aligned NPZ frame |
|---|---|---:|---:|---:|
| F7, forehand | forehand_manual_010_forehand_0.000_1.880_50fps | 34 | 0.68 s | 31 |
| B7, backhand | backhand_manual_011_backhand_0.000_1.880_50fps | 35 | 0.70 s | 31 |

Both chosen clips contain 94 frames, so these video-to-NPZ frame mappings are
exact. Their matching aligned poses are at frame 31, not the peak frame 43.
Joint arrays at pre-alignment and mapped aligned frames were checked for exact
equality. This is a pose-processing illustration, not a claim about ball contact.

## Rendering and fidelity

The middle panel uses `smpl_params_incam` and `K_fullimg` from the corresponding
`hmr4d_results.pt`, with the neutral SMPL-X model and GVHMR's supermotion body-model
settings (10 shape coefficients, 12 hand PCA components, flat-hand mean disabled).
The mesh is opaque and overlaid on the original image. Clothing, hair, hands, or
the racket can remain visible outside the estimated body surface. No background
inpainting or body alignment offsets were applied. The racket seen behind the
human mesh belongs to the original video, not the reconstructed human model.

The robot panel uses 29 named joint angles from the selected NPZ and the existing
`g1_hitter_racket/main.urdf` asset. All 30 non-root saved body positions were
compared with forward kinematics before display transforms. The maximum error
was approximately 1.2e-7 metres in both examples. The selected pose is loaded
statically; no policy rollout, physics settling, or new retargeting is involved.

Robot rendering uses PyBullet TinyRenderer in DIRECT mode, not an IsaacLab
viewport screenshot. Initial planar heading and horizontal placement are
normalized for presentation, with a common camera, light, and ground style.
Joint angles are unchanged. Two ASCII STL hand/paddle meshes were converted to
OBJ without changing geometry, in a separate rendering copy of the URDF.
Existing project assets and motion datasets were not modified.

The native GVHMR/PyTorch3D renderer could not import because its extension does
not match the installed PyTorch ABI. The saved camera-space SMPL-X mesh was
rendered with the same pinhole intrinsics using PyBullet's CPU rasterizer.

Video and overlay crops are identical: x=0, y=320, width=1080, height=1600.
Each standalone cropped panel is 1080 × 1600. Full-resolution source frames,
uncropped overlays, and intermediate mesh data remain in this directory.

## Reproduce final renders and figure

From `/home/yhl/Desktop`:

```bash
/home/yhl/miniforge3/envs/gmr/bin/python \
  Hitter/MOSAIC-main/outputs/reference_pipeline_20260806_ppt/render_selected.py

/home/yhl/miniforge3/envs/gmr/bin/python \
  Hitter/MOSAIC-main/outputs/reference_pipeline_20260806_ppt/compose_figure.py
```

`prepare_candidates.py` records how the original 18 candidates were selected;
its FFmpeg calls refuse to overwrite existing extracted frames.
