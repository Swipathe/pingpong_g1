import sys
import tempfile
import unittest
from pathlib import Path


DEPLOY_DIR = Path(__file__).resolve().parents[1] / "deploy"
sys.path.insert(0, str(DEPLOY_DIR))

import run as deploy_run


class RunSingleInstanceLockTest(unittest.TestCase):
    def test_second_policy_instance_is_rejected_while_lock_is_held(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            lock_path = Path(tmpdir) / "policy.lock"

            with deploy_run.acquire_single_instance_lock(lock_path):
                with self.assertRaisesRegex(RuntimeError, "Another RobotBridge policy process is already running"):
                    with deploy_run.acquire_single_instance_lock(lock_path):
                        pass

            with deploy_run.acquire_single_instance_lock(lock_path):
                self.assertTrue(lock_path.exists())


if __name__ == "__main__":
    unittest.main()
