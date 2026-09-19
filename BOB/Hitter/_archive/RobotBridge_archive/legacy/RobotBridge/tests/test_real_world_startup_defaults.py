import unittest
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

DEPLOY_DIR = Path(__file__).resolve().parents[1] / "deploy"
sys.path.insert(0, str(DEPLOY_DIR))

from simulator.real_world import RealWorld


class RealWorldStartupDefaultsTest(unittest.TestCase):
    def test_get_state_has_safe_defaults_before_async_callbacks(self):
        sim = object.__new__(RealWorld)
        sim.num_dof = 29
        sim.num_action = 29
        sim.active_dof_idx = np.arange(29, dtype=np.int32)
        sim.cfg = SimpleNamespace(
            control=SimpleNamespace(update_with_fk=False, reset_heading_on_start=True),
            motion={},
        )

        RealWorld._init_low_state(sim)
        sim.get_state()

        np.testing.assert_allclose(sim.root_trans, np.zeros(3, dtype=np.float32))
        np.testing.assert_allclose(sim.root_quat, np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32))
        self.assertEqual(sim.dof_pos.shape, (29,))

    def test_mosaic_calibration_sends_absolute_joint_targets(self):
        sim = object.__new__(RealWorld)
        sim.cfg = SimpleNamespace(control=SimpleNamespace(is_mosaic=True, action_scale=0.25))
        default = np.array([0.1, 0.2, 0.3], dtype=np.float32)
        residual = np.array([0.4, -0.1, 0.0], dtype=np.float32)

        action = RealWorld._calibration_action_from_residual(sim, residual, default)

        np.testing.assert_allclose(action, np.array([0.5, 0.1, 0.3], dtype=np.float32))

    def test_residual_calibration_keeps_scaled_residual_action(self):
        sim = object.__new__(RealWorld)
        sim.cfg = SimpleNamespace(control=SimpleNamespace(is_mosaic=False, action_scale=0.25))
        default = np.array([0.1, 0.2, 0.3], dtype=np.float32)
        residual = np.array([0.4, -0.1, 0.0], dtype=np.float32)

        action = RealWorld._calibration_action_from_residual(sim, residual, default)

        np.testing.assert_allclose(action, np.array([1.6, -0.4, 0.0], dtype=np.float32))

    def test_can_disable_r2_runtime_termination_after_startup(self):
        sim = object.__new__(RealWorld)
        sim.cfg = SimpleNamespace(control=SimpleNamespace(terminate_on_r2=False))
        sim.right_lower_right_switch_pressed = True

        self.assertFalse(RealWorld.check_termination(sim))


if __name__ == "__main__":
    unittest.main()
