import unittest

from deploy.mocap_bridge.monitor_vicon_lcm import msg_status


class Message:
    valid = 0
    occluded = 1


class MonitorViconLcmTest(unittest.TestCase):
    def test_reads_valid_and_occluded(self):
        self.assertEqual(msg_status(Message()), (0, 1))


if __name__ == "__main__":
    unittest.main()
