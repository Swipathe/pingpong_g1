from __future__ import annotations

import unittest
from contextlib import contextmanager
from types import SimpleNamespace

import numpy as np
from loguru import logger

from envs.hitter import HitterEnv
from utils.hitter_realtime import (
    CommandPhase,
    HitterCommandLifecycle,
    PlannerResultSnapshot,
)


@contextmanager
def captured_log_messages():
    messages = []
    sink_id = logger.add(
        lambda message: messages.append(message.record["message"]),
        level="INFO",
    )
    try:
        yield messages
    finally:
        logger.remove(sink_id)


def fake_command(
    *,
    marker: float = 1.0,
    strike_type: str = "backhand",
    ball_in=None,
    ball_out=None,
    racket_velocity=None,
):
    if ball_in is None:
        ball_in = [-marker, -marker, -marker]
    if ball_out is None:
        ball_out = [marker, marker, marker]
    if racket_velocity is None:
        racket_velocity = [marker, marker, marker]
    return SimpleNamespace(
        strike_type=strike_type,
        p_base_target_xy=np.array([-0.4, 0.0], dtype=np.float64),
        v_racket_target_w=np.asarray(racket_velocity, dtype=np.float64),
        time_to_strike=0.90,
        strike_plan=SimpleNamespace(
            p_racket_target=np.array([0.0, 0.1, 1.0], dtype=np.float64),
            v_ball_in=np.asarray(ball_in, dtype=np.float64),
            v_ball_out=np.asarray(ball_out, dtype=np.float64),
        ),
    )


def planner_result(
    *,
    epoch: int = 7,
    generation: int = 11,
    deadline: float = 10.90,
    command=None,
):
    if command is None:
        command = fake_command()
    return PlannerResultSnapshot(
        track_epoch=epoch,
        source_generation=generation,
        source_frame=generation,
        strike_deadline_monotonic_s=deadline,
        completed_monotonic_s=10.01,
        command=command,
        error=None,
    )


def configured_lifecycle() -> HitterCommandLifecycle:
    return HitterCommandLifecycle(
        waiting_tts=0.92,
        arm_tts=0.90,
        minimum_arm_tts=0.80,
        maximum_policy_tts=0.92,
        swing_duration_sampler=lambda: 1.85,
    )


def minimal_env() -> HitterEnv:
    env = HitterEnv.__new__(HitterEnv)
    env.motion_cfg = {
        "ball_planner": {
            "target_base_height_w": 0.78,
        }
    }
    env.hitter_command_lifecycle = configured_lifecycle()
    env._hitter_last_logged_phase = CommandPhase.ARMED
    env._hitter_last_result_key = None
    env._hitter_minimum_track_epoch = 0
    env._hitter_minimum_generation = 0
    env._hitter_observed_track_epoch = None
    return env


class StrikeTargetPayloadTests(unittest.TestCase):
    def test_logs_all_world_velocity_vectors_and_norms(self):
        env = minimal_env()
        ball_in = np.array([-3.0, 0.4, -1.2], dtype=np.float64)
        ball_out = np.array([4.2708, -0.5, 1.8], dtype=np.float64)
        racket_velocity = np.array([1.4, -0.1, 0.7], dtype=np.float64)
        result = planner_result(
            epoch=27,
            generation=8103,
            command=fake_command(
                ball_in=ball_in,
                ball_out=ball_out,
                racket_velocity=racket_velocity,
            ),
        )

        with captured_log_messages() as messages:
            env._log_hitter_strike_target(result)

        target_messages = [
            message
            for message in messages
            if message.startswith("HITTER strike target:")
        ]
        self.assertEqual(len(target_messages), 1)
        message = target_messages[0]
        self.assertIn("epoch=27 generation=8103 type=backhand", message)
        self.assertIn(
            "v_ball_in_w_mps=[-3.0000,0.4000,-1.2000]",
            message,
        )
        self.assertIn(
            f"speed_ball_in_mps={np.linalg.norm(ball_in):.4f}",
            message,
        )
        self.assertIn(
            "v_ball_out_w_mps=[4.2708,-0.5000,1.8000]",
            message,
        )
        self.assertIn(
            f"speed_ball_out_mps={np.linalg.norm(ball_out):.4f}",
            message,
        )
        self.assertIn(
            "v_racket_target_w_mps=[1.4000,-0.1000,0.7000]",
            message,
        )
        self.assertIn(
            f"speed_racket_mps={np.linalg.norm(racket_velocity):.4f}",
            message,
        )

    def test_invalid_ball_out_only_warns_and_does_not_change_main_validation(self):
        env = minimal_env()
        command = fake_command(ball_out=[np.nan, 0.0, 0.0])

        decision = env._consume_hitter_planner_result(
            planner_result(command=command),
            now=10.0,
        )
        self.assertEqual(decision, "armed")
        self.assertIs(
            env.hitter_command_lifecycle.active_result.command,
            command,
        )

        with captured_log_messages() as messages:
            env._log_hitter_strike_target(
                planner_result(command=command)
            )

        self.assertFalse(
            any(
                message.startswith("HITTER strike target:")
                for message in messages
            )
        )
        warnings = [
            message
            for message in messages
            if message.startswith(
                "Failed to log HITTER strike target:"
            )
        ]
        self.assertEqual(len(warnings), 1)
        self.assertIn("strike_plan.v_ball_out", warnings[0])

    def test_missing_active_result_only_warns(self):
        env = minimal_env()

        with captured_log_messages() as messages:
            env._log_hitter_strike_target(None)

        warnings = [
            message
            for message in messages
            if message.startswith(
                "Failed to log HITTER strike target:"
            )
        ]
        self.assertEqual(len(warnings), 1)
        self.assertIn("missing a command", warnings[0])


class StrikeTargetBoundaryTests(unittest.TestCase):
    def test_epsilon_boundary_logs_original_target_exactly_once(self):
        env = minimal_env()
        lifecycle = configured_lifecycle()
        env.hitter_command_lifecycle = lifecycle

        result = planner_result(
            epoch=31,
            generation=401,
            deadline=10.90,
            command=fake_command(marker=3.0),
        )
        self.assertEqual(lifecycle.ingest(result, now=10.00), "armed")

        previous_phase = lifecycle.phase
        previous_active = lifecycle.active_result
        previous_end = lifecycle.command_end_deadline_s
        self.assertIs(previous_active, result)
        boundary_now = float(
            np.nextafter(
                result.strike_deadline_monotonic_s,
                -np.inf,
            )
        )
        self.assertLess(
            boundary_now,
            result.strike_deadline_monotonic_s,
        )

        lifecycle.advance(boundary_now)
        self.assertEqual(lifecycle.phase, CommandPhase.RECOVERY)

        with captured_log_messages() as messages:
            env._log_hitter_advance_transitions(
                previous_phase,
                previous_active,
                previous_end,
                now=boundary_now,
            )

            next_previous_phase = lifecycle.phase
            next_previous_active = lifecycle.active_result
            next_previous_end = lifecycle.command_end_deadline_s
            next_now = float(
                np.nextafter(
                    result.strike_deadline_monotonic_s,
                    np.inf,
                )
            )
            lifecycle.advance(next_now)
            env._log_hitter_advance_transitions(
                next_previous_phase,
                next_previous_active,
                next_previous_end,
                now=next_now,
            )

        target_messages = [
            message
            for message in messages
            if message.startswith("HITTER strike target:")
        ]
        self.assertEqual(len(target_messages), 1)
        self.assertIn("epoch=31 generation=401", target_messages[0])
        self.assertIn(
            "v_ball_out_w_mps=[3.0000,3.0000,3.0000]",
            target_messages[0],
        )

    def test_late_advance_logs_latest_override_exactly_once(self):
        env = minimal_env()
        lifecycle = configured_lifecycle()
        env.hitter_command_lifecycle = lifecycle

        first = planner_result(
            epoch=27,
            generation=1,
            deadline=10.90,
            command=fake_command(marker=1.0),
        )
        self.assertEqual(
            lifecycle.ingest(first, now=10.00),
            "armed",
        )

        final_override = planner_result(
            epoch=27,
            generation=2,
            deadline=10.88,
            command=fake_command(marker=2.0),
        )
        self.assertEqual(
            lifecycle.ingest(final_override, now=10.02),
            "overridden",
        )

        previous_phase = lifecycle.phase
        previous_active = lifecycle.active_result
        previous_end = lifecycle.command_end_deadline_s
        late_now = float(previous_end) + 0.10
        lifecycle.advance(late_now)
        self.assertEqual(lifecycle.phase, CommandPhase.WAITING)

        with captured_log_messages() as messages:
            env._log_hitter_advance_transitions(
                previous_phase,
                previous_active,
                previous_end,
                now=late_now,
            )
            env._log_hitter_advance_transitions(
                lifecycle.phase,
                lifecycle.active_result,
                lifecycle.command_end_deadline_s,
                now=late_now + 0.01,
            )

        target_messages = [
            message
            for message in messages
            if message.startswith("HITTER strike target:")
        ]
        self.assertEqual(len(target_messages), 1)
        message = target_messages[0]
        self.assertIn("epoch=27 generation=2", message)
        self.assertIn(
            "v_ball_out_w_mps=[2.0000,2.0000,2.0000]",
            message,
        )
        self.assertNotIn(
            "v_ball_out_w_mps=[1.0000,1.0000,1.0000]",
            message,
        )

    def test_non_strike_transition_does_not_log_target(self):
        env = minimal_env()
        env.hitter_command_lifecycle = SimpleNamespace(
            phase=CommandPhase.TRACKING
        )

        with captured_log_messages() as messages:
            env._log_hitter_advance_transitions(
                CommandPhase.WAITING,
                None,
                None,
                now=10.0,
            )

        self.assertFalse(
            any(
                message.startswith("HITTER strike target:")
                for message in messages
            )
        )
