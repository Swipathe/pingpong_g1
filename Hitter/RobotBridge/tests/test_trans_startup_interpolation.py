from pathlib import Path
import unittest


TRANS_CPP = Path(__file__).resolve().parents[1] / "unitree_sdk2" / "trans.cpp"


class TransStartupInterpolationTest(unittest.TestCase):
    def test_trans_restores_startup_interpolation_before_policy_handoff(self):
        source = TRANS_CPP.read_text()

        self.assertIn("duration_(5.0)", source)
        self.assertIn("time_ < duration_", source)
        self.assertIn("startup_joint_position_", source)
        self.assertIn("Startup interpolation complete", source)
        self.assertIn("move to default pose over 5s", source)

    def test_trans_forces_ai_sport_off_before_lowcmd_thread_starts(self):
        source = TRANS_CPP.read_text()

        self.assertIn('ServiceSwitch("ai_sport", 0', source)
        self.assertLess(
            source.index('ServiceSwitch("ai_sport", 0'),
            source.index("CreateRecurrentThreadEx(\"dds_write_thread\""),
        )


if __name__ == "__main__":
    unittest.main()
