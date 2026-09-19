from __future__ import annotations

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from hydra import compose, initialize_config_dir
from omegaconf import OmegaConf


DEPLOY_DIR = Path(__file__).resolve().parents[1]
CONFIG_DIR = DEPLOY_DIR / "config"


def _vector(element: ET.Element, attribute: str) -> np.ndarray:
    return np.fromstring(element.attrib[attribute], sep=" ", dtype=np.float64)


class HitterGeometryConfigTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with initialize_config_dir(
            version_base=None,
            config_dir=str(CONFIG_DIR.resolve()),
        ):
            composed = compose(config_name="hitter", overrides=["sim=mujoco"])
        cls.config = OmegaConf.to_container(composed, resolve=True)
        asset = cls.config["robot"]["asset"]
        asset_root_text = str(asset["asset_root"])
        if asset_root_text.startswith("./"):
            asset_root_text = asset_root_text[2:]
        asset_root = Path(asset_root_text)
        cls.xml_path = DEPLOY_DIR / asset_root / asset["asset_file"]
        cls.xml_root = ET.parse(cls.xml_path).getroot()

    def geom(self, name: str) -> ET.Element:
        element = self.xml_root.find(f".//geom[@name='{name}']")
        self.assertIsNotNone(element, f"missing MuJoCo geom {name!r}")
        return element

    def test_hitter_mujoco_config_interpolates_planner_geometry_and_physics(self):
        planner = self.config["mimic"]["motion"]["ball_planner"]
        table_tennis = self.config["sim"]["config"]["table_tennis"]

        self.assertTrue(table_tennis["enabled"])
        self.assertTrue(table_tennis["use_analytic_table_bounce"])
        self.assertTrue(table_tennis["use_analytic_racket_hit"])
        for key in (
            "table_center_xy_w",
            "table_height",
            "table_length",
            "table_width",
            "ball_radius",
            "drag_coefficient",
            "vertical_restitution",
            "horizontal_restitution",
        ):
            self.assertEqual(table_tennis[key], planner[key], key)

    def test_generic_mujoco_config_remains_table_tennis_agnostic(self):
        generic_mujoco = OmegaConf.to_container(
            OmegaConf.load(CONFIG_DIR / "sim" / "mujoco.yaml"),
            resolve=False,
        )

        self.assertNotIn("table_tennis", generic_mujoco["config"])

    def test_table_top_center_line_and_net_use_accepted_dimensions(self):
        center_x = 1.365369
        half_length = 1.365369
        half_width = 0.7562255
        surface_z = 0.760000

        top = self.geom("hitter_table_top")
        center_line = self.geom("hitter_table_center_line")
        net = self.geom("hitter_table_net")
        top_pos, top_size = _vector(top, "pos"), _vector(top, "size")
        line_pos, line_size = _vector(center_line, "pos"), _vector(center_line, "size")
        net_pos, net_size = _vector(net, "pos"), _vector(net, "size")

        self.assertEqual(float(top_pos[0]), center_x)
        self.assertEqual(float(top_size[0]), half_length)
        self.assertEqual(float(top_size[1]), half_width)
        self.assertEqual(float(top_pos[2] + top_size[2]), surface_z)
        self.assertEqual(float(line_pos[0]), center_x)
        self.assertEqual(float(line_size[0]), half_length)
        self.assertEqual(float(line_pos[2] - line_size[2]), surface_z)
        self.assertEqual(float(net_pos[0]), center_x)
        self.assertEqual(float(net_size[1]), half_width)

    def test_table_legs_are_symmetric_and_inside_adjusted_bounds(self):
        length = 2.730738
        center_x = 1.365369
        half_width = 0.7562255
        names = (
            "hitter_table_leg_near_left",
            "hitter_table_leg_near_right",
            "hitter_table_leg_far_left",
            "hitter_table_leg_far_right",
        )
        legs = {name: self.geom(name) for name in names}
        positions = {name: _vector(geom, "pos") for name, geom in legs.items()}
        sizes = {name: _vector(geom, "size") for name, geom in legs.items()}

        self.assertEqual(positions["hitter_table_leg_near_left"][0], 0.18)
        self.assertEqual(positions["hitter_table_leg_far_left"][0], 2.550738)
        for side in ("left", "right"):
            near = positions[f"hitter_table_leg_near_{side}"]
            far = positions[f"hitter_table_leg_far_{side}"]
            self.assertEqual(float(0.5 * (near[0] + far[0])), center_x)
            self.assertEqual(float(near[1]), float(far[1]))
        for distance in ("near", "far"):
            left = positions[f"hitter_table_leg_{distance}_left"]
            right = positions[f"hitter_table_leg_{distance}_right"]
            self.assertEqual(float(left[0]), float(right[0]))
            self.assertEqual(float(left[1]), float(-right[1]))
        for name in names:
            position, size = positions[name], sizes[name]
            self.assertGreaterEqual(float(position[0] - size[0]), 0.0)
            self.assertLessEqual(float(position[0] + size[0]), length)
            self.assertLessEqual(float(abs(position[1]) + size[1]), half_width)

    def test_xml_ball_radius_matches_composed_hitter_config(self):
        ball = self.geom("hitter_ball_geom")
        table_tennis = self.config["sim"]["config"]["table_tennis"]

        self.assertEqual(float(_vector(ball, "size")[0]), table_tennis["ball_radius"])


if __name__ == "__main__":
    unittest.main()
