"""Render selected, measured human/robot poses; no pose synthesis or IK changes."""

from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import pybullet as p
from scipy.spatial.transform import Rotation
import smplx
import torch


OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
ASSETS = ROOT / "source/whole_body_tracking/whole_body_tracking/assets/unitree_description"
URDF = ASSETS / "urdf/g1_hitter_racket/main.urdf"
RENDER = OUT / "renders"
CROP = (0, 320, 1080, 1920)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_mesh(path, vertices, faces):
    triangles = vertices[faces]
    face_normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    normals = np.zeros_like(vertices)
    for i in range(3):
        np.add.at(normals, faces[:, i], face_normals)
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-10)
    with path.open("w") as f:
        for xyz in vertices:
            f.write("v %.8f %.8f %.8f\n" % tuple(xyz))
        for xyz in normals:
            f.write("vn %.8f %.8f %.8f\n" % tuple(xyz))
        for a, b, c in faces + 1:
            f.write(f"f {a}//{a} {b}//{b} {c}//{c}\n")


def projection_from_intrinsics(K, width, height, near=0.05, far=100.0):
    matrix = np.array([
        [2 * K[0, 0] / width, 0, 1 - 2 * K[0, 2] / width, 0],
        [0, 2 * K[1, 1] / height, 2 * K[1, 2] / height - 1, 0],
        [0, 0, -(far + near) / (far - near), -2 * far * near / (far - near)],
        [0, 0, -1, 0],
    ])
    return matrix.flatten(order="F").tolist()


def human_overlay(record, model):
    frame = record["gvhmr_frame_zero_based"]
    pred = torch.load(record["gvhmr_result"], map_location="cpu", weights_only=False)
    params = {k: v[frame:frame + 1].float() for k, v in pred["smpl_params_incam"].items()}
    with torch.no_grad():
        vertices = model(**params).vertices[0].cpu().numpy()
    faces = np.asarray(model.faces)
    mesh_file = RENDER / f"{record['id']}_smplx_incam.obj"
    write_mesh(mesh_file, vertices, faces)
    K = pred["K_fullimg"][frame].numpy()
    raw = Image.open(record["unannotated_image"]).convert("RGB")
    width, height = raw.size
    projected = (vertices @ K.T)
    projected = projected[:, :2] / projected[:, 2:]
    print(record["id"], "projected mesh bounds", projected.min(0), projected.max(0), flush=True)

    p.resetSimulation()
    shape = p.createVisualShape(p.GEOM_MESH, fileName=str(mesh_file), rgbaColor=[0.80, 0.82, 0.84, 1], specularColor=[0.18, 0.18, 0.18])
    body = p.createMultiBody(baseMass=0, baseVisualShapeIndex=shape)
    view = p.computeViewMatrix([0, 0, 0], [0, 0, 1], [0, -1, 0])
    result = p.getCameraImage(width, height, viewMatrix=view, projectionMatrix=projection_from_intrinsics(K, width, height),
                              renderer=p.ER_TINY_RENDERER, lightDirection=[-2, -3, -5], lightColor=[1, 1, 1],
                              lightAmbientCoeff=0.48, lightDiffuseCoeff=0.52, lightSpecularCoeff=0.14, shadow=0)
    rgb = np.asarray(result[2], dtype=np.uint8).reshape(height, width, 4)[:, :, :3]
    mask = np.asarray(result[4]).reshape(height, width) == body
    assert mask.sum() > 50000, f"Empty/small rendered human: {mask.sum()}"
    overlay = np.array(raw)
    overlay[mask] = rgb[mask]
    rgba = np.dstack([rgb, mask.astype(np.uint8) * 255])
    Image.fromarray(rgba).save(RENDER / f"{record['id']}_human_mesh_rgba.png")
    Image.fromarray(overlay).save(RENDER / f"{record['id']}_reconstruction_full.png")
    Image.fromarray(overlay).crop(CROP).save(RENDER / f"{record['id']}_reconstruction.png")
    raw.crop(CROP).save(RENDER / f"{record['id']}_video.png")
    return {"intrinsics": K.tolist(), "mesh_vertex_count": len(vertices), "mesh_face_count": len(faces),
            "rendered_mesh_pixels": int(mask.sum()), "smplx_frame": frame, "mesh_sha256": digest(mesh_file)}


def resolved_urdf():
    tree = ET.parse(URDF)
    converted_meshes = {}
    for mesh in tree.findall(".//mesh"):
        name = mesh.attrib["filename"]
        if name.startswith("package://unitree_description/"):
            mesh.attrib["filename"] = str(ASSETS / name.removeprefix("package://unitree_description/"))
        assert Path(mesh.attrib["filename"]).is_file(), mesh.attrib["filename"]
        source = Path(mesh.attrib["filename"])
        # The hand/paddle are ASCII STL. TinyRenderer needs these as OBJ.
        # Only convert the file representation; vertices and faces are unchanged.
        if source.name in {"right_hand_grip_hitter_frame.stl", "right_paddle_hitter_frame.stl"}:
            if source not in converted_meshes:
                triangles = np.array([list(map(float, line.split()[1:])) for line in source.read_text().splitlines() if line.strip().startswith("vertex ")])
                vertices, indices = np.unique(triangles, axis=0, return_inverse=True)
                target = RENDER / (source.stem + ".obj")
                write_mesh(target, vertices, indices.reshape(-1, 3))
                converted_meshes[source] = target
            mesh.attrib["filename"] = str(converted_meshes[source])
    path = RENDER / "render_robot_resolved.urdf"
    tree.write(path, encoding="utf-8", xml_declaration=True)
    return path


def robot_pose(record, urdf, manifest):
    p.resetSimulation()
    robot = p.loadURDF(str(urdf), useFixedBase=False, flags=p.URDF_MAINTAIN_LINK_ORDER)
    joint_info = [p.getJointInfo(robot, i) for i in range(p.getNumJoints(robot))]
    joints = {x[1].decode(): x[0] for x in joint_info}
    links = {x[12].decode(): x[0] for x in joint_info}
    motion = np.load(record["npz_pre_align"])
    float_frame = record["npz_pre_align_frame_nearest_video_frame"]
    assert float_frame == int(float_frame), "Selected frames should map exactly."
    frame = int(float_frame)
    root = motion["body_pos_w"][frame, 0].astype(float)
    q = motion["body_quat_w"][frame, 0].astype(float)
    rotation = Rotation.from_quat(q[[1, 2, 3, 0]])
    p.resetBasePositionAndOrientation(robot, root, rotation.as_quat())
    for name, value in zip(manifest["joint_names"], motion["joint_pos"][frame], strict=True):
        p.resetJointState(robot, joints[name], float(value))
    fk = np.array([p.getLinkState(robot, links[n], computeForwardKinematics=True)[4] for n in manifest["body_names"][1:]])
    error = np.linalg.norm(fk - motion["body_pos_w"][frame, 1:], axis=1)
    print(record["id"], "FK max discrepancy (m)", float(error.max()), "racket", float(error[-1]), flush=True)
    # A drift larger than 2 mm means the current robot asset is not the data-generating asset.
    if error.max() > 0.002:
        raise ValueError(f"Robot asset does not match saved body states: {error.max():.6f} m")
    aligned_frame = frame + record["npz_alignment_shift_frames"]
    aligned = np.load(record["npz_aligned"])
    np.testing.assert_array_equal(motion["joint_pos"][frame], aligned["joint_pos"][aligned_frame])

    # For display only, remove initial planar heading and center the robot.
    forward = rotation.apply([1, 0, 0])
    yaw = np.arctan2(forward[1], forward[0])
    centered_rotation = Rotation.from_euler("z", -yaw) * rotation
    display_root = np.array([0, 0, root[2]])
    p.resetBasePositionAndOrientation(robot, display_root, centered_rotation.as_quat())
    aabbs = [p.getAABB(robot, i) for i in range(-1, p.getNumJoints(robot))]
    # Use visual mesh ground placement through the original model dimensions.
    low = min(b[0][2] for b in aabbs)
    display_root[2] -= low
    p.resetBasePositionAndOrientation(robot, display_root, centered_rotation.as_quat())

    # Muted floor with subtle grid lines, identical camera and light for both examples.
    floor_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[20, 20, 0.005], rgbaColor=[0.91, 0.94, 0.97, 1])
    p.createMultiBody(baseMass=0, baseVisualShapeIndex=floor_shape, basePosition=[0, 0, -0.008])
    for axis in [0, 1]:
        dims = [6, 0.0015, 0.0003] if axis == 0 else [0.0015, 6, 0.0003]
        line_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=dims, rgbaColor=[0.77, 0.82, 0.88, 1])
        for step in range(-12, 13):
            pos = [0, step * 0.25, -0.002] if axis == 0 else [step * 0.25, 0, -0.002]
            p.createMultiBody(baseMass=0, baseVisualShapeIndex=line_shape, basePosition=pos)
    width, height = 1080, 1600
    camera_eye = [4.1, -0.10, 1.17]
    camera_target = [0, 0, 0.72]
    view = p.computeViewMatrix(camera_eye, camera_target, [0, 0, 1])
    projection = p.computeProjectionMatrixFOV(fov=25, aspect=width / height, nearVal=0.05, farVal=100)
    result = p.getCameraImage(width, height, viewMatrix=view, projectionMatrix=projection,
                              renderer=p.ER_TINY_RENDERER, lightDirection=[3, -4, 7], lightColor=[1, 1, 1],
                              lightAmbientCoeff=0.55, lightDiffuseCoeff=0.45, lightSpecularCoeff=0.12, shadow=0)
    rgba = np.asarray(result[2], dtype=np.uint8).reshape(height, width, 4)
    segmentation = np.asarray(result[4]).reshape(height, width)
    mask = (segmentation & ((1 << 24) - 1)) == robot
    ys, xs = np.nonzero(mask)
    assert len(xs) > 20000, "Robot not visible"
    assert xs.min() > 3 and xs.max() < width - 4 and ys.min() > 3 and ys.max() < height - 4, "Robot is cropped"
    # Background sky has no geometry; keep the floor/grid rendered as above.
    rgba[segmentation < 0, :3] = [239, 244, 249]
    Image.fromarray(rgba[:, :, :3]).save(RENDER / f"{record['id']}_robot.png")
    alpha = np.dstack([rgba[:, :, :3], mask.astype(np.uint8) * 255])
    Image.fromarray(alpha).save(RENDER / f"{record['id']}_robot_rgba.png")
    return {"pre_align_frame": frame, "aligned_frame": aligned_frame, "joint_count": len(manifest["joint_names"]),
            "max_fk_error_m": float(error.max()), "racket_fk_error_m": float(error[-1]),
            "display_camera_eye": camera_eye, "display_camera_target": camera_target,
            "display_yaw_removed_rad": float(yaw), "display_ground_translation_m": float(-low),
            "robot_pixel_bbox": [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())]}


def main():
    RENDER.mkdir(exist_ok=True)
    torch.set_num_threads(4)
    model = smplx.create(
        str(ROOT / "GVHMR/inputs/checkpoints/body_models"), model_type="smplx", gender="neutral",
        num_betas=10, num_pca_comps=12, flat_hand_mean=False, batch_size=1,
    ).eval()
    records = json.loads((OUT / "candidate_manifest.json").read_text())["candidates"]
    selected = [next(r for r in records if r["id"] == name) for name in ["F7", "B7"]]
    converted = json.loads((ROOT / "data/hitter_motions/20260806_mqy_g1_npz_raw_pre_align/_index/gvhmr_hitter_npz_manifest.json").read_text())
    client = p.connect(p.DIRECT)
    details = []
    try:
        urdf = resolved_urdf()
        for record in selected:
            human = human_overlay(record, model)
            robot = robot_pose(record, urdf, converted)
            details.append({"selection": record, "human_render": human, "robot_render": robot,
                            "source_hashes": {key: digest(record[key]) for key in ["gvhmr_result", "npz_pre_align", "unannotated_image"]}})
    finally:
        p.disconnect(client)
    (OUT / "render_manifest.json").write_text(json.dumps({
        "renderer": "PyBullet TinyRenderer (CPU), static measured poses",
        "human_model": "SMPL-X neutral, GVHMR supermotion parameters, camera-space prediction",
        "human_crop_xyxy": list(CROP), "robot_urdf": str(URDF), "robot_urdf_sha256": digest(URDF),
        "examples": details,
    }, indent=2) + "\n")
    print("Saved matched human and robot renders.", flush=True)


if __name__ == "__main__":
    main()
