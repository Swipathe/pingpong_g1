from __future__ import annotations

import unittest
from contextlib import contextmanager
from types import SimpleNamespace

import numpy as np
from loguru import logger

from envs.hitter import HitterEnv
from tests.hitter_runtime_test_harness import (
    make_hitter_env_for_test,
    result_batch,
    success_result,
)
from utils.hitter_planner import HitterWbcCommand, StrikePlan
from utils.hitter_realtime import (
    CommandPhase,
    HitterCommandLifecycle,
    LifecycleCancelReason,
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
    strike_plan = StrikePlan(
        t_strike=0.90,
        p_racket_target=np.array([0.0, 0.1, 1.0], dtype=np.float64),
        v_racket_target=np.asarray(racket_velocity, dtype=np.float64),
        v_ball_in=np.asarray(ball_in, dtype=np.float64),
        v_ball_out=np.asarray(ball_out, dtype=np.float64),
    )
    return HitterWbcCommand(
        strike_type=strike_type,
        strike_table_y_w=0.1,
        strike_side_source="table_y",
        p_base_target_xy=np.array([-0.4, 0.0], dtype=np.float64),
        v_racket_target_w=np.asarray(racket_velocity, dtype=np.float64),
        time_to_strike=0.90,
        strike_plan=strike_plan,
    )


def planner_result(
    *,
    track_id: int = 7,
    generation: int = 11,
    deadline: float = 10.90,
    command=None,
):
    if command is None:
        command = fake_command()
    return PlannerResultSnapshot(
        track_id=track_id,
        source_generation=generation,
        source_frame=generation,
        strike_deadline_monotonic_s=deadline,
        completed_monotonic_s=10.01,
        command=command,
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
    env.hitter_command_initialized = False
    env.simulator = SimpleNamespace(is_real=False)
    return env


class StrikeTargetPayloadTests(unittest.TestCase):
    def test_logs_all_world_velocity_vectors_and_norms(self):
        env = minimal_env()
        ball_in = np.array([-3.0, 0.4, -1.2], dtype=np.float64)
        ball_out = np.array([4.2708, -0.5, 1.8], dtype=np.float64)
        racket_velocity = np.array([1.4, -0.1, 0.7], dtype=np.float64)
        result = planner_result(
            track_id=27,
            generation=8103,
            command=fake_command(
                ball_in=ball_in,
                ball_out=ball_out,
                racket_velocity=racket_velocity,
            ),
        )

        with captured_log_messages() as messages:
            env._log_hitter_strike_target(result)

        target_messages = [message for message in messages if message.startswith("HITTER strike target:")]
        self.assertEqual(len(target_messages), 1)
        message = target_messages[0]
        self.assertIn("track_id=27 generation=8103 type=backhand", message)
        self.assertIn("strike_table_y_w=0.1000 side_source=table_y", message)
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

    def test_real_forehand_logs_raw_and_policy_racket_velocity(self):
        env = minimal_env()
        env.simulator = SimpleNamespace(is_real=True)
        env.forehand_policy_vx_offset_mps = 0.25
        racket_velocity = np.array([1.4, -0.1, 0.7], dtype=np.float64)
        result = planner_result(
            command=fake_command(
                strike_type="forehand",
                racket_velocity=racket_velocity,
            )
        )

        with captured_log_messages() as messages:
            env._log_hitter_strike_target(result)

        target_messages = [message for message in messages if message.startswith("HITTER strike target:")]
        self.assertEqual(len(target_messages), 1)
        message = target_messages[0]
        self.assertIn(
            "v_racket_target_w_mps=[1.4000,-0.1000,0.7000]",
            message,
        )
        self.assertIn(
            "v_racket_policy_w_mps=[1.6500,-0.1000,0.7000]",
            message,
        )
        self.assertIn("speed_racket_policy_mps=1.7951", message)
        self.assertIn("policy_vx_offset_mps=0.2500", message)

    def test_real_logs_direction_preserving_racket_speed_increment(self):
        env = minimal_env()
        env.simulator = SimpleNamespace(is_real=True)
        env.racket_velocity_magnitude_increment_mps = 1.0
        racket_velocity = np.array([3.0, 4.0, 0.0], dtype=np.float64)
        result = planner_result(
            command=fake_command(
                strike_type="backhand",
                racket_velocity=racket_velocity,
            )
        )

        with captured_log_messages() as messages:
            env._log_hitter_strike_target(result)

        target_messages = [message for message in messages if message.startswith("HITTER strike target:")]
        self.assertEqual(len(target_messages), 1)
        message = target_messages[0]
        self.assertIn(
            "v_racket_target_w_mps=[3.0000,4.0000,0.0000]",
            message,
        )
        self.assertIn("speed_racket_mps=5.0000", message)
        self.assertIn(
            "v_racket_policy_w_mps=[3.6000,4.8000,0.0000]",
            message,
        )
        self.assertIn("speed_racket_policy_mps=6.0000", message)
        self.assertIn(
            "policy_velocity_magnitude_increment_mps=1.0000",
            message,
        )
        self.assertIn("policy_vx_offset_mps=0.0000", message)
        self.assertIn("policy_vy_offset_mps=0.0000", message)

    def test_real_backhand_logs_raw_and_policy_racket_velocity(self):
        env = minimal_env()
        env.simulator = SimpleNamespace(is_real=True)
        env.backhand_policy_vx_offset_mps = 2.0
        env.backhand_policy_vy_decrement_mps = 0.3
        racket_velocity = np.array([1.4, 0.1, 0.7], dtype=np.float64)
        result = planner_result(
            command=fake_command(
                strike_type="backhand",
                racket_velocity=racket_velocity,
            )
        )

        with captured_log_messages() as messages:
            env._log_hitter_strike_target(result)

        target_messages = [message for message in messages if message.startswith("HITTER strike target:")]
        self.assertEqual(len(target_messages), 1)
        message = target_messages[0]
        self.assertIn(
            "v_racket_target_w_mps=[1.4000,0.1000,0.7000]",
            message,
        )
        self.assertIn(
            "v_racket_policy_w_mps=[3.4000,-0.2000,0.7000]",
            message,
        )
        self.assertIn("policy_vx_offset_mps=2.0000", message)
        self.assertIn("policy_vy_offset_mps=-0.3000", message)

    def test_invalid_ball_out_is_rejected_by_typed_lifecycle_validation(self):
        env = minimal_env()
        command = fake_command(ball_out=[np.nan, 0.0, 0.0])

        decision = env.hitter_command_lifecycle.ingest(planner_result(command=command), now=10.0)
        self.assertEqual(decision.kind, "cancelled")
        self.assertEqual(
            decision.cancel_reason.value,
            "INTERNAL_ERROR",
        )
        self.assertIsNone(env.hitter_command_lifecycle.active_result)

        with captured_log_messages() as messages:
            env._log_hitter_strike_target(planner_result(command=command))

        self.assertFalse(any(message.startswith("HITTER strike target:") for message in messages))
        warnings = [message for message in messages if message.startswith("Failed to log HITTER strike target:")]
        self.assertEqual(len(warnings), 1)
        self.assertIn("strike_plan.v_ball_out", warnings[0])

    def test_missing_active_result_only_warns(self):
        env = minimal_env()

        with captured_log_messages() as messages:
            env._log_hitter_strike_target(None)

        warnings = [message for message in messages if message.startswith("Failed to log HITTER strike target:")]
        self.assertEqual(len(warnings), 1)
        self.assertIn("missing a command", warnings[0])


class StrikeTargetBoundaryTests(unittest.TestCase):
    def test_epsilon_boundary_logs_original_target_exactly_once(self):
        env = minimal_env()
        lifecycle = configured_lifecycle()
        env.hitter_command_lifecycle = lifecycle

        result = planner_result(
            track_id=31,
            generation=401,
            deadline=10.90,
            command=fake_command(marker=3.0),
        )
        self.assertEqual(lifecycle.ingest(result, now=10.00).kind, "armed")

        previous_phase = lifecycle.phase
        previous_active = lifecycle.active_result
        previous_end = lifecycle.command_end_deadline_s
        self.assertIsNot(previous_active, result)
        self.assertEqual(previous_active.track_id, result.track_id)
        self.assertEqual(
            previous_active.source_generation,
            result.source_generation,
        )
        boundary_now = float(result.strike_deadline_monotonic_s)

        with captured_log_messages() as messages:
            strike_decision = lifecycle.advance(boundary_now)
            self.assertEqual(strike_decision.kind, "struck")
            env._apply_hitter_lifecycle_decision(
                strike_decision,
                now=boundary_now,
                previous_phase=previous_phase,
                previous_active=previous_active,
                previous_command_end_deadline_s=previous_end,
            )
            self.assertEqual(lifecycle.phase, CommandPhase.RECOVERY)

            next_previous_phase = lifecycle.phase
            next_previous_active = lifecycle.active_result
            next_previous_end = lifecycle.command_end_deadline_s
            next_now = float(np.nextafter(boundary_now, np.inf))
            none_decision = lifecycle.advance(next_now)
            env._apply_hitter_lifecycle_decision(
                none_decision,
                now=next_now,
                previous_phase=next_previous_phase,
                previous_active=next_previous_active,
                previous_command_end_deadline_s=next_previous_end,
            )

        target_messages = [message for message in messages if message.startswith("HITTER strike target:")]
        self.assertEqual(len(target_messages), 1)
        self.assertIn("track_id=31 generation=401", target_messages[0])
        self.assertIn(
            "v_ball_out_w_mps=[3.0000,3.0000,3.0000]",
            target_messages[0],
        )

    def test_late_advance_logs_latest_override_exactly_once(self):
        env = minimal_env()
        lifecycle = configured_lifecycle()
        env.hitter_command_lifecycle = lifecycle

        first = planner_result(
            track_id=27,
            generation=1,
            deadline=10.90,
            command=fake_command(marker=1.0),
        )
        self.assertEqual(lifecycle.ingest(first, now=10.00).kind, "armed")

        final_override = planner_result(
            track_id=27,
            generation=2,
            deadline=10.88,
            command=fake_command(marker=1.01),
        )
        self.assertEqual(
            lifecycle.ingest(final_override, now=10.02).kind,
            "overridden",
        )

        previous_phase = lifecycle.phase
        previous_active = lifecycle.active_result
        previous_end = lifecycle.command_end_deadline_s
        late_now = float(previous_end) + 0.10
        with captured_log_messages() as messages:
            strike_decision = lifecycle.advance(late_now)
            self.assertEqual(strike_decision.kind, "struck")
            env._apply_hitter_lifecycle_decision(
                strike_decision,
                now=late_now,
                previous_phase=previous_phase,
                previous_active=previous_active,
                previous_command_end_deadline_s=previous_end,
            )
            self.assertEqual(lifecycle.phase, CommandPhase.RECOVERY)
            recovery_phase = lifecycle.phase
            recovery_active = lifecycle.active_result
            recovery_end = lifecycle.command_end_deadline_s
            entered_waiting = lifecycle.advance(late_now)
            self.assertEqual(entered_waiting.kind, "entered_waiting")
            env._apply_hitter_lifecycle_decision(
                entered_waiting,
                now=late_now + 0.01,
                previous_phase=recovery_phase,
                previous_active=recovery_active,
                previous_command_end_deadline_s=recovery_end,
            )
            self.assertEqual(lifecycle.phase, CommandPhase.WAITING)

        target_messages = [message for message in messages if message.startswith("HITTER strike target:")]
        self.assertEqual(len(target_messages), 1)
        message = target_messages[0]
        self.assertIn("track_id=27 generation=2", message)
        self.assertIn(
            "v_racket_target_w_mps=[1.0100,1.0100,1.0100]",
            message,
        )
        self.assertNotIn(
            "v_racket_target_w_mps=[1.0000,1.0000,1.0000]",
            message,
        )

    def test_non_strike_transition_does_not_log_target(self):
        env = minimal_env()
        env.hitter_command_lifecycle = SimpleNamespace(phase=CommandPhase.TRACKING)

        with captured_log_messages() as messages:
            env._log_hitter_advance_transitions(
                CommandPhase.WAITING,
                None,
                None,
                now=10.0,
            )

        self.assertFalse(any(message.startswith("HITTER strike target:") for message in messages))


class LifecycleDecisionLoggingTests(unittest.TestCase):
    def test_arm_transition_is_logged_immediately_exactly_once(self):
        env = make_hitter_env_for_test(is_real=True)
        env.hitter_planner_worker.queue(result_batch(success_result()))

        with captured_log_messages() as messages:
            env._update_hitter_command(now=10.0)
            env._update_hitter_command(now=10.01)

        transitions = [message for message in messages if "HITTER lifecycle transition: waiting -> armed" in message]
        self.assertEqual(len(transitions), 1)
        self.assertIn("track_id=7", transitions[0])
        self.assertIn("decision=armed", transitions[0])

    def test_repeated_precommit_cancel_is_logged_immediately_once(self):
        env = make_hitter_env_for_test(is_real=True)
        env.hitter_planner_worker.queue(result_batch(success_result()))
        env._update_hitter_command(now=10.0)

        with captured_log_messages() as messages:
            for now in (10.1, 10.2):
                env.simulator.runtime_events.append(
                    SimpleNamespace(
                        reason=LifecycleCancelReason.BASE_POSE_INVALID,
                        track_id=7,
                        detail="invalid pelvis",
                    )
                )
                env._update_hitter_command(now=now)

        cancellations = [
            message
            for message in messages
            if message.startswith("HITTER lifecycle safety decision:")
            and "reason=BASE_POSE_INVALID" in message
            and "decision=cancelled" in message
        ]
        self.assertEqual(len(cancellations), 1)
        self.assertIn("track_id=7", cancellations[0])

    def test_same_tick_status_and_identityless_event_log_cancel_once(self):
        env = make_hitter_env_for_test(is_real=True)
        env.hitter_planner_worker.queue(result_batch(success_result()))
        env._update_hitter_command(now=10.0)
        env.simulator.base_pose_valid = False
        env.simulator.runtime_events.append(
            SimpleNamespace(
                reason=LifecycleCancelReason.BASE_POSE_INVALID,
                track_id=None,
                detail="same invalid pelvis transition",
            )
        )

        with captured_log_messages() as messages:
            env._update_hitter_command(now=10.1)

        cancellations = [
            message
            for message in messages
            if message.startswith("HITTER lifecycle safety decision:") and "reason=BASE_POSE_INVALID" in message
        ]
        self.assertEqual(len(cancellations), 1)
        self.assertIn("track_id=7", cancellations[0])

    def test_stale_status_and_ended_track_event_log_cancel_once(self):
        env = make_hitter_env_for_test(is_real=True)
        env.hitter_planner_worker.queue(result_batch(success_result()))
        env._update_hitter_command(now=10.0)
        env.simulator.latched_fault = LifecycleCancelReason.VICON_STREAM_STALE
        env.simulator.base_pose_valid = False
        env.simulator.active_track_id = None
        env.simulator.runtime_events.append(
            SimpleNamespace(
                reason=LifecycleCancelReason.VICON_STREAM_STALE,
                track_id=7,
                detail="stale stream ended active track",
            )
        )

        with captured_log_messages() as messages:
            env._update_hitter_command(now=10.1)

        cancellations = [
            message
            for message in messages
            if message.startswith("HITTER lifecycle safety decision:") and "reason=VICON_STREAM_STALE" in message
        ]
        self.assertEqual(len(cancellations), 1)
        self.assertIn(7, env.simulator.consumed_track_ids)
        self.assertEqual(
            env.simulator.consumption_reasons[7],
            "cancelled",
        )

    def test_status_fault_event_challenger_is_consumed_without_second_log(self):
        env = make_hitter_env_for_test(is_real=True)
        env.hitter_planner_worker.queue(result_batch(success_result()))
        env._update_hitter_command(now=10.0)
        env.simulator.latched_fault = LifecycleCancelReason.TRACK_ID_CONFLICT
        env.simulator.runtime_events.append(
            SimpleNamespace(
                reason=LifecycleCancelReason.TRACK_ID_CONFLICT,
                track_id=8,
                detail="challenger conflict",
            )
        )

        with captured_log_messages() as messages:
            env._update_hitter_command(now=10.1)

        cancellations = [
            message
            for message in messages
            if message.startswith("HITTER lifecycle safety decision:") and "reason=TRACK_ID_CONFLICT" in message
        ]
        self.assertEqual(len(cancellations), 1)
        self.assertIn(7, env.simulator.consumed_track_ids)
        self.assertIn(8, env.simulator.consumed_track_ids)
        self.assertEqual(
            env.simulator.consumption_reasons[8],
            "status_fault:TRACK_ID_CONFLICT",
        )
