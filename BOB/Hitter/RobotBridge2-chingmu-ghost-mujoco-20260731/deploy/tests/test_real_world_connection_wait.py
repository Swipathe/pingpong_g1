from __future__ import annotations

import unittest
from unittest.mock import call, patch

from simulator.base_sim import BaseSim
from simulator.real_world import RealWorld


class ConnectionProbeRealWorld(RealWorld):
    def _init_communication(self):
        self.communication_initialized = True

    def spin(self):
        self.poll_thread_started = True

    def connected(self):
        self.connection_checks += 1
        return self.connection_checks >= 3


class RealWorldConnectionWaitTests(unittest.TestCase):
    @staticmethod
    def _construct_probe():
        def initialize_without_external_dependencies(instance, config):
            instance.cfg = config
            instance.connection_checks = 0

        with patch.object(
            BaseSim,
            "__init__",
            new=initialize_without_external_dependencies,
        ):
            return ConnectionProbeRealWorld(config=object())

    def test_init_checks_connection_until_first_packet_is_available(self):
        simulator = self._construct_probe()

        self.assertTrue(simulator.communication_initialized)
        self.assertTrue(simulator.poll_thread_started)
        self.assertTrue(simulator.sync)
        self.assertEqual(simulator.connection_checks, 3)

    def test_init_yields_between_unsuccessful_connection_checks(self):
        with patch("simulator.real_world.time.sleep") as sleep:
            self._construct_probe()

        self.assertEqual(sleep.call_args_list, [call(0.001), call(0.001)])


if __name__ == "__main__":
    unittest.main()
