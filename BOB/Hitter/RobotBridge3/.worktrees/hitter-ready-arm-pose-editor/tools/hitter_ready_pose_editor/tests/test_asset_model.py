import json
import math
import os
import shutil
import tempfile
import unittest
from pathlib import Path
from xml.etree import ElementTree

import yaml

from tools.hitter_ready_pose_editor.asset_model import (
    AssetSourceChanged,
    HitterAssetModel,
    UrdfSceneModel,
)
from tools.hitter_ready_pose_editor.constants import ARM_JOINT_NAMES


APPROVED_REPO_ROOT = Path("/home/loco1/BOB/Hitter/RobotBridge2")
APPROVED_URDF = Path(
    "/home/loco1/BOB/Hitter/MOSAIC-main/source/whole_body_tracking/"
    "whole_body_tracking/assets/unitree_description/urdf/"
    "g1_hitter_racket/main.urdf"
)
APPROVED_ASSET_ROOT = Path(
    "/home/loco1/BOB/Hitter/MOSAIC-main/source/whole_body_tracking/"
    "whole_body_tracking/assets/unitree_description"
)
APPROVED_MJCF = (
    APPROVED_REPO_ROOT
    / "deploy/data/assets/g1/g1_29dof_hitter_racket_table_tennis.xml"
)
APPROVED_ASSET_YAML = (
    APPROVED_REPO_ROOT / "deploy/config/asset/g1_hitter_racket.yaml"
)


class AssetModelTest(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.fixture_dir = Path(self.temp_dir.name)
        source_dir = Path(__file__).resolve().parent / "fixtures"
        for name in (
            "minimal_robot.urdf",
            "minimal_robot.xml",
            "minimal_asset.yaml",
            "minimal_ascii.stl",
        ):
            shutil.copy2(str(source_dir / name), str(self.fixture_dir / name))
        self.fixture_urdf = self.fixture_dir / "minimal_robot.urdf"
        self.fixture_mjcf = self.fixture_dir / "minimal_robot.xml"
        self.fixture_asset_yaml = self.fixture_dir / "minimal_asset.yaml"
        self.fixture_stl = self.fixture_dir / "minimal_ascii.stl"

        with APPROVED_ASSET_YAML.open("r", encoding="utf-8") as stream:
            asset = yaml.safe_load(stream)
        self.asset_joint_order = tuple(asset["kinematic_joint_names"])

    def tearDown(self):
        self.temp_dir.cleanup()

    def build_fixture_model(self):
        return UrdfSceneModel.from_urdf(
            urdf_path=self.fixture_urdf,
            allowed_asset_root=self.fixture_dir,
        )

    def build_fixture_bundle(self):
        return HitterAssetModel.from_paths(
            urdf_path=self.fixture_urdf,
            mjcf_path=self.fixture_mjcf,
            asset_yaml_path=self.fixture_asset_yaml,
            allowed_asset_root=self.fixture_dir,
        )

    def build_live_with_joint_order(self, values):
        with APPROVED_ASSET_YAML.open("r", encoding="utf-8") as stream:
            asset = yaml.safe_load(stream)
        names = tuple(asset["kinematic_joint_names"])
        asset["joint_order"] = dict(zip(names, values))
        copied_yaml = self.fixture_dir / "copied_live_asset.yaml"
        copied_yaml.write_text(
            yaml.safe_dump(asset, sort_keys=False),
            encoding="utf-8",
        )
        return HitterAssetModel.from_paths(
            urdf_path=APPROVED_URDF,
            mjcf_path=APPROVED_MJCF,
            asset_yaml_path=copied_yaml,
            allowed_asset_root=APPROVED_ASSET_ROOT,
        )

    def test_fixture_manifest_contains_tree_limit_and_opaque_mesh_id(self):
        model = self.build_fixture_model()
        manifest = model.build_manifest()
        joint = manifest.joint_by_name["arm_joint"]
        self.assertEqual(joint.parent_link, "base_link")
        self.assertEqual(joint.child_link, "arm_link")
        self.assertEqual(joint.axis_xyz, (0.0, 1.0, 0.0))
        self.assertEqual(joint.hard_limit_rad, (-1.0, 2.0))
        self.assertAlmostEqual(joint.soft_limit_rad[0], -0.85)
        self.assertAlmostEqual(joint.soft_limit_rad[1], 1.85)
        mesh_id = manifest.visuals[0].mesh_id
        self.assertNotIn("/", mesh_id)
        self.assertRegex(mesh_id, r"^[0-9a-f]{64}$")
        self.assertEqual(model.mesh_path(mesh_id), self.fixture_stl.resolve())

    def test_visual_origin_scale_and_named_material_are_preserved(self):
        visual = self.build_fixture_model().build_manifest().visuals[0]
        self.assertEqual(visual.origin_xyz, (0.1, 0.2, 0.3))
        self.assertEqual(visual.origin_rpy, (0.2, 0.0, -0.1))
        self.assertEqual(visual.mesh_scale_xyz, (2.0, 3.0, 4.0))
        self.assertEqual(visual.color_rgba, (0.1, 0.2, 0.3, 0.9))

    def test_unknown_or_path_like_mesh_id_is_rejected(self):
        model = self.build_fixture_model()
        with self.assertRaises(KeyError):
            model.mesh_path("../../etc/passwd")

    def test_fixed_joints_have_no_active_value_contract(self):
        manifest = self.build_fixture_model().build_manifest()
        for name in ("left_hand_palm_joint", "right_racket_fixed_joint"):
            spec = manifest.joint_by_name[name]
            self.assertEqual(spec.joint_type, "fixed")
            self.assertIsNone(spec.axis_xyz)
            self.assertIsNone(spec.hard_limit_rad)
            self.assertIsNone(spec.default_rad)
            self.assertIsNone(spec.robot29_index)

    def test_fixture_joint_tree_is_topological_and_fixed_branches_are_kept(self):
        manifest = self.build_fixture_model().build_manifest()
        self.assertEqual(
            tuple(joint.name for joint in manifest.joints_topological),
            (
                "arm_joint",
                "left_hand_palm_joint",
                "right_racket_fixed_joint",
            ),
        )
        self.assertEqual(manifest.root_link, "base_link")
        self.assertEqual(
            manifest.joint_by_name["left_hand_palm_joint"].origin_xyz,
            (0.1, 0.2, 0.0),
        )
        self.assertEqual(
            manifest.joint_by_name["right_racket_fixed_joint"].origin_xyz,
            (0.2, 0.0, 0.3),
        )
        self.assertEqual(manifest.incompatibilities, ("not_evaluated",))

    def test_live_hitter_contract_has_current_29_joint_order(self):
        model = HitterAssetModel.from_defaults(APPROVED_REPO_ROOT)
        manifest = model.build_manifest()
        self.assertEqual(tuple(manifest.active_joint_names), self.asset_joint_order)
        self.assertEqual(tuple(manifest.arm_joint_names), ARM_JOINT_NAMES)
        self.assertEqual(manifest.right_racket_link, "right_racket_link")
        self.assertEqual(
            {visual.mesh_id for visual in manifest.visuals},
            set(model.mesh_ids()),
        )

    def test_live_paths_and_meshes_are_approved(self):
        model = HitterAssetModel.from_defaults(APPROVED_REPO_ROOT)
        manifest = model.build_manifest()
        self.assertEqual(Path(manifest.asset_paths["urdf"]), APPROVED_URDF)
        self.assertEqual(Path(manifest.asset_paths["mjcf"]), APPROVED_MJCF)
        self.assertEqual(
            Path(manifest.asset_paths["asset_yaml"]), APPROVED_ASSET_YAML
        )
        for mesh_id in model.mesh_ids():
            mesh = model.mesh_path(mesh_id)
            self.assertEqual(
                os.path.commonpath((str(mesh), str(APPROVED_ASSET_ROOT))),
                str(APPROVED_ASSET_ROOT),
            )

    def test_live_wrist_limit_rounding_is_compatible(self):
        model = HitterAssetModel.from_defaults(APPROVED_REPO_ROOT)
        manifest = model.build_manifest()
        self.assertTrue(manifest.compatible_for_save, manifest.incompatibilities)
        self.assertLess(abs(-1.97222205 - -1.97222), 1e-5)
        self.assertEqual(manifest.table_boxes[0].frame, "robot_base_default")
        self.assertNotEqual(
            manifest.urdf_kinematic_sha256,
            manifest.mjcf_kinematic_sha256,
        )

    def test_live_tree_has_all_active_and_fixed_joints(self):
        manifest = HitterAssetModel.from_defaults(
            APPROVED_REPO_ROOT
        ).build_manifest()
        self.assertEqual(len(manifest.joints_topological), 37)
        self.assertEqual(
            sum(j.joint_type == "revolute" for j in manifest.joints_topological),
            29,
        )
        self.assertEqual(
            sum(j.joint_type == "fixed" for j in manifest.joints_topological),
            8,
        )
        self.assertEqual(
            tuple(
                j.name
                for j in manifest.joints_topological
                if j.joint_type == "revolute"
            ),
            self.asset_joint_order,
        )
        by_name = dict(zip(manifest.active_joint_names, manifest.default_joint_pos_rad))
        self.assertEqual(by_name["left_hip_pitch_joint"], -0.312)
        self.assertEqual(by_name["right_elbow_joint"], 0.6)
        for joint in manifest.joints_topological:
            if joint.joint_type == "fixed":
                self.assertIsNone(joint.default_rad)
            else:
                self.assertEqual(joint.default_rad, by_name[joint.name])

    def test_live_table_is_expressed_in_default_pelvis_frame(self):
        manifest = HitterAssetModel.from_defaults(
            APPROVED_REPO_ROOT
        ).build_manifest()
        top = next(box for box in manifest.table_boxes if box.name == "hitter_table_top")
        for actual, expected in zip(
            top.center_xyz_m,
            (1.765369, 0.0, -0.058),
        ):
            self.assertAlmostEqual(actual, expected, places=6)
        self.assertNotAlmostEqual(top.center_xyz_m[0], 1.365369, places=6)

    def test_fixture_bundle_is_compatible_and_origin_drift_is_not(self):
        model = self.build_fixture_bundle()
        self.assertTrue(model.build_manifest().compatible_for_save)
        tree = ElementTree.parse(str(self.fixture_urdf))
        joint = tree.getroot().find("./joint[@name='arm_joint']/origin")
        joint.set("xyz", "0 0 0.6")
        tree.write(str(self.fixture_urdf), encoding="unicode")
        drifted = self.build_fixture_bundle().build_manifest()
        self.assertFalse(drifted.compatible_for_save)
        self.assertTrue(
            any("arm_joint" in reason for reason in drifted.incompatibilities),
            drifted.incompatibilities,
        )

    def test_fixture_mjcf_nonzero_joint_position_is_incompatible(self):
        text = self.fixture_mjcf.read_text(encoding="utf-8")
        self.fixture_mjcf.write_text(
            text.replace(
                '<joint name="arm_joint" axis="0 1 0" range="-1 2"/>',
                '<joint name="arm_joint" pos="0.01 0 0" '
                'axis="0 1 0" range="-1 2"/>',
            ),
            encoding="utf-8",
        )
        manifest = self.build_fixture_bundle().build_manifest()
        self.assertFalse(manifest.compatible_for_save)
        self.assertIn("arm_joint joint position differs", manifest.incompatibilities)

    def test_fixture_jointed_racket_body_is_incompatible(self):
        text = self.fixture_mjcf.read_text(encoding="utf-8")
        self.fixture_mjcf.write_text(
            text.replace(
                'quat="0.9833474432563558 0.0342707985504821 '
                '0.1060205110617956 0.1435721750273919"/>',
                'quat="0.9833474432563558 0.0342707985504821 '
                '0.1060205110617956 0.1435721750273919">'
                '<joint name="racket_hinge" axis="0 0 1" '
                'range="-0.1 0.1"/>'
                '<geom type="sphere" size="0.01" mass="1"/>'
                '</body>',
            ),
            encoding="utf-8",
        )
        manifest = self.build_fixture_bundle().build_manifest()
        self.assertFalse(manifest.compatible_for_save)
        self.assertIn(
            "right_racket_fixed_joint body contains joints",
            manifest.incompatibilities,
        )

    def test_offset_joint_order_values_are_rejected(self):
        with self.assertRaisesRegex(
            ValueError,
            "joint_order values must be exactly 0..N-1",
        ):
            self.build_live_with_joint_order(range(1, 30))

    def test_gapped_joint_order_values_are_rejected(self):
        values = list(range(29))
        values[-1] = 29
        with self.assertRaisesRegex(
            ValueError,
            "joint_order values must be exactly 0..N-1",
        ):
            self.build_live_with_joint_order(values)

    def test_duplicate_joint_order_values_are_rejected(self):
        values = list(range(29))
        values[-1] = 27
        with self.assertRaisesRegex(
            ValueError,
            "joint_order values must be exactly 0..N-1",
        ):
            self.build_live_with_joint_order(values)

    def test_valid_joint_order_exports_exact_robot29_indices(self):
        manifest = HitterAssetModel.from_defaults(
            APPROVED_REPO_ROOT
        ).build_manifest()
        indices = tuple(
            joint.robot29_index
            for joint in manifest.joints_topological
            if joint.joint_type == "revolute"
        )
        self.assertEqual(indices, tuple(range(29)))
        self.assertEqual(
            manifest.joint_by_name["left_hip_pitch_joint"].robot29_index,
            0,
        )
        self.assertEqual(
            manifest.joint_by_name["left_shoulder_pitch_joint"].robot29_index,
            15,
        )
        self.assertEqual(
            manifest.joint_by_name["right_wrist_yaw_joint"].robot29_index,
            28,
        )

    def test_asset_yaml_drift_is_detected(self):
        model = self.build_fixture_bundle()
        self.fixture_asset_yaml.write_text(
            self.fixture_asset_yaml.read_text(encoding="utf-8") + "\n# drift\n",
            encoding="utf-8",
        )
        with self.assertRaises(AssetSourceChanged):
            model.assert_source_hashes_unchanged()

    def test_mesh_drift_is_detected(self):
        model = self.build_fixture_bundle()
        self.fixture_stl.write_text(
            self.fixture_stl.read_text(encoding="utf-8") + "\n",
            encoding="utf-8",
        )
        with self.assertRaises(AssetSourceChanged):
            model.assert_source_hashes_unchanged()

    def test_asset_yaml_hash_changes_bundle_signature(self):
        before = self.build_fixture_bundle().build_manifest()
        self.fixture_asset_yaml.write_text(
            self.fixture_asset_yaml.read_text(encoding="utf-8") + "\n# v2\n",
            encoding="utf-8",
        )
        after = self.build_fixture_bundle().build_manifest()
        self.assertNotEqual(
            before.asset_hashes["asset_yaml_sha256"],
            after.asset_hashes["asset_yaml_sha256"],
        )
        self.assertNotEqual(
            before.asset_signature_sha256,
            after.asset_signature_sha256,
        )

    def test_public_manifest_has_exact_json_contract(self):
        public = self.build_fixture_bundle().public_manifest()
        self.assertEqual(
            set(public),
            {
                "schema",
                "units",
                "limits",
                "frame",
                "rootLink",
                "activeJointNames",
                "armJointNames",
                "defaultJointPosRad",
                "joints",
                "visuals",
                "tableVisuals",
                "assetHashes",
                "compatibleForSave",
                "incompatibilities",
                "rightRacketLink",
            },
        )
        self.assertEqual(set(public["units"]), {"length", "angle"})
        self.assertEqual(
            set(public["limits"]),
            {"maxRequestBodyBytes", "maxMeshBytes", "maxStlTriangles"},
        )
        self.assertEqual(
            set(public["frame"]),
            {
                "name",
                "rootLink",
                "rootTransform",
                "handedness",
                "matrixLayout",
                "quaternionConvention",
                "groundZRobotBaseM",
            },
        )
        self.assertEqual(
            set(public["joints"][0]),
            {
                "name",
                "type",
                "parentLink",
                "childLink",
                "originXyzM",
                "originRpyRad",
                "axisXyz",
                "hardLimitRad",
                "softLimitRad",
                "defaultRad",
                "robot29Index",
            },
        )
        self.assertEqual(
            set(public["visuals"][0]),
            {
                "linkName",
                "originXyzM",
                "originRpyRad",
                "meshScaleXyz",
                "meshId",
                "colorRgba",
            },
        )
        self.assertEqual(
            set(public["tableVisuals"][0]),
            {
                "name",
                "frame",
                "centerXyzM",
                "quaternionXyzw",
                "halfSizeXyzM",
                "colorRgba",
            },
        )
        self.assertEqual(
            set(public["assetHashes"]),
            {
                "displayUrdfSha256",
                "displayMeshSetSha256",
                "validationMjcfSha256",
                "assetYamlSha256",
                "urdfKinematicSha256",
                "mjcfKinematicSha256",
                "assetSignatureSha256",
            },
        )
        self.assertEqual(public["schema"], "hitter_asset_manifest/v1")
        self.assertEqual(public["units"], {"length": "m", "angle": "rad"})
        self.assertEqual(public["rightRacketLink"], "right_racket_link")
        self.assertEqual(
            public["limits"],
            {
                "maxRequestBodyBytes": 262144,
                "maxMeshBytes": 67108864,
                "maxStlTriangles": 1000000,
            },
        )
        self.assertEqual(
            json.loads(json.dumps(public, allow_nan=False)),
            public,
        )

    def test_duplicate_links_and_joints_fail_fast(self):
        text = self.fixture_urdf.read_text(encoding="utf-8")
        cases = (
            text.replace(
                '<link name="arm_link"/>',
                '<link name="arm_link"/><link name="arm_link"/>',
            ),
            text.replace(
                '<joint name="arm_joint" type="revolute">',
                '<joint name="arm_joint" type="revolute">'
                '<parent link="base_link"/><child link="arm_link"/>'
                '<axis xyz="0 1 0"/><limit lower="-1" upper="2"/>'
                '</joint><joint name="arm_joint" type="revolute">',
            ),
        )
        for index, invalid in enumerate(cases):
            with self.subTest(index=index):
                self.fixture_urdf.write_text(invalid, encoding="utf-8")
                with self.assertRaises(ValueError):
                    self.build_fixture_model()
                self.fixture_urdf.write_text(text, encoding="utf-8")

    def test_multiple_roots_and_unsupported_joint_type_fail_fast(self):
        text = self.fixture_urdf.read_text(encoding="utf-8")
        cases = (
            text.replace("</robot>", '<link name="orphan"/></robot>'),
            text.replace('type="revolute"', 'type="continuous"', 1),
        )
        for index, invalid in enumerate(cases):
            with self.subTest(index=index):
                self.fixture_urdf.write_text(invalid, encoding="utf-8")
                with self.assertRaises(ValueError):
                    self.build_fixture_model()
                self.fixture_urdf.write_text(text, encoding="utf-8")

    def test_missing_mesh_nonfinite_origin_axis_and_limit_fail_fast(self):
        text = self.fixture_urdf.read_text(encoding="utf-8")
        cases = (
            text.replace("minimal_ascii.stl", "missing.stl"),
            text.replace('xyz="0.1 0.2 0.3"', 'xyz="nan 0.2 0.3"'),
            text.replace('axis xyz="0 1 0"', 'axis xyz="inf 1 0"'),
            text.replace(
                '<limit lower="-1" upper="2" effort="1" velocity="1"/>',
                "",
            ),
        )
        for index, invalid in enumerate(cases):
            with self.subTest(index=index):
                self.fixture_urdf.write_text(invalid, encoding="utf-8")
                with self.assertRaises((ValueError, FileNotFoundError)):
                    self.build_fixture_model()
                self.fixture_urdf.write_text(text, encoding="utf-8")

    def test_nonpositive_mesh_scale_fails_fast(self):
        text = self.fixture_urdf.read_text(encoding="utf-8")
        self.fixture_urdf.write_text(
            text.replace('scale="2 3 4"', 'scale="2 0 4"'),
            encoding="utf-8",
        )
        with self.assertRaises(ValueError):
            self.build_fixture_model()


if __name__ == "__main__":
    unittest.main()
