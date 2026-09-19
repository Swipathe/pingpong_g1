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

    def test_wait_for_r2_yields_until_press_edge_arrives(self):
        simulator = object.__new__(RealWorld)
        simulator.right_lower_right_switch_pressed = False

        def mark_r2_pressed(_duration):
            simulator.right_lower_right_switch_pressed = True

        with patch(
            "simulator.real_world.time.sleep",
            side_effect=mark_r2_pressed,
        ) as sleep:
            simulator._wait_for_right_lower_right_switch_press()

        self.assertFalse(simulator.right_lower_right_switch_pressed)
        self.assertEqual(sleep.call_args_list, [call(0.002)])

    def test_wait_for_r2_accepts_current_level_and_waits_for_release(self):
        simulator = object.__new__(RealWorld)
        simulator.right_lower_right_switch_pressed = False
        simulator.right_lower_right_switch = 1
        sleep_calls = 0

        def release_r2_after_first_sleep(_duration):
            nonlocal sleep_calls
            sleep_calls += 1
            if sleep_calls == 1:
                simulator.right_lower_right_switch = 0
                return
            raise AssertionError("wait ignored current R2 level after release")

        with patch(
            "simulator.real_world.time.sleep",
            side_effect=release_r2_after_first_sleep,
        ) as sleep:
            simulator._wait_for_right_lower_right_switch_press()

        self.assertFalse(simulator.right_lower_right_switch_pressed)
        self.assertEqual(simulator.right_lower_right_switch, 0)
        self.assertEqual(sleep.call_args_list, [call(0.002)])


if __name__ == "__main__":
    unittest.main()
