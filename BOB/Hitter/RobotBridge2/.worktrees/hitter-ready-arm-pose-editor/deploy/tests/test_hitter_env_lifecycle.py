from __future__ import annotations

import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
from omegaconf import OmegaConf

from envs.base_env import BaseEnv
from envs.hitter import HitterEnv
from simulator.real_world import RealWorld
from utils.hitter_realtime import (
    BallEstimateSnapshot,
    CommandPhase,
    HitterCommandLifecycle,
    LatestOnlyPlannerWorker,
    PlannerResultSnapshot,
    PlannerWorkerStats,
)


TRACK_ENDED_RESULT_ERROR = "RuntimeError: HITTER_TRACK_ENDED"


def fake_command(*, marker: float, tts: float = 0.9, malformed: bool = False):
    base_target = [marker] if malformed else [marker, marker]
    return SimpleNamespace(
        strike_type="forehand" if marker == 1.0 else "backhand",
        p_base_target_xy=np.asarray(base_target, dtype=np.float64),
        v_racket_target_w=np.full(3, marker, dtype=np.float64),
        time_to_strike=float(tts),
        strike_plan=SimpleNamespace(
            p_racket_target=np.full(3, marker, dtype=np.float64),
            v_ball_in=np.full(3, -marker, dtype=np.float64),
            v_ball_out=np.full(3, marker, dtype=np.float64),
        ),
    )


def planner_result(
    *,
    epoch: int = 1,
    generation: int = 1,
    deadline: float = 10.9,
    marker: float = 1.0,
    command=None,
    error: str | None = None,
):
    if command is None and error is None:
        command = fake_command(marker=marker)
    return PlannerResultSnapshot(
        track_epoch=int(epoch),
        source_generation=int(generation),
        source_frame=int(generation),
        strike_deadline_monotonic_s=float(deadline),
        completed_monotonic_s=10.01,
        command=command,
        error=error,
    )


def ball_snapshot(
    *,
    epoch: int = 1,
    generation: int = 1,
    received: float = 10.0,
    visible: bool = True,
    ready: bool = True,
    base_valid: bool = True,
    velocity_x: float = -1.5,
):
    return BallEstimateSnapshot(
        track_epoch=int(epoch),
        generation=int(generation),
        source_frame=int(generation),
        source_time_s=generation / 360.0,
        received_monotonic_s=float(received),
        position_w=np.array([0.4, 0.0, 0.95], dtype=np.float64),
        velocity_w=np.array([velocity_x, 0.0, 0.0], dtype=np.float64),
        base_position_w=np.array([-0.4, 0.0, 0.793], dtype=np.float64),
        base_quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
        base_valid=bool(base_valid),
        visible=bool(visible),
        ready=bool(ready),
    )


class FakeKinematics:
    def forward(self, **_kwargs):
        return {
            "right_racket_link": {
                "pos": np.array([0.0, 0.0, 1.0], dtype=np.float32)
            }
        }, None


class FakeSimulator:
    def __init__(self, *, is_real: bool, events=None):
        self.is_real = bool(is_real)
        self.events = [] if events is None else events
        self.high_dt = 0.02
        self.root_trans_world = np.array([-0.4, 0.0, 0.793], dtype=np.float32)
        self.root_quat_world = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        self.ball_pos_world = np.array([0.4, 0.0, 0.95], dtype=np.float32)
        self.ball_vel_world = np.array([-1.5, 0.0, 0.0], dtype=np.float32)
        self.ball_visible = False
        self.ball_state_estimator_ready = False
        self.base_pose_valid = True
        self.ball_track_epoch = 0
        self.ball_snapshot_generation = 0
        self.latest_ball_snapshot = None
        self.kinematic = FakeKinematics()
        self.listener = None
        self.get_state_calls = 0
        self.forbid_get_state = False
        self.close_calls = 0
        self.reset_calls = 0

    def register_hitter_ball_listener(self, listener):
        self.listener = listener

        def unregister():
            if self.listener is not None:
                self.events.append("unregister")
                self.listener = None

        return unregister

    def reset_ball_state_estimator(self):
        self.reset_calls += 1
        self.ball_track_epoch += 1

    def reset_hitter_ball(self):
        self.events.append("reset_ball")
        self.ball_visible = True
        self.ball_state_estimator_ready = True

    def get_state(self):
        self.get_state_calls += 1
        if self.forbid_get_state:
            raise AssertionError("worker planner must not call simulator.get_state()")

    def update_obs(self):
        self.get_state()
        return {
            "base_ang_vel": np.zeros(3, dtype=np.float32),
            "projected_gravity": np.array([0.0, 0.0, -1.0], dtype=np.float32),
            "dof_pos": np.zeros(29, dtype=np.float32),
            "dof_vel": np.zeros(29, dtype=np.float32),
        }

    def calibrate(self, _refresh, _ref_dof_pos):
        return None

    def check_termination(self):
        return False

    def close(self):
        self.close_calls += 1
        self.events.append("simulator.close")


class FakeWorker:
    def __init__(self, *, events=None):
        self.result = None
        self.events = [] if events is None else events
        self.submitted = []
        self.closed = False

    def submit(self, snapshot):
        self.submitted.append(snapshot)

    def latest_result(self):
        return self.result

    @property
    def stats(self):
        return PlannerWorkerStats(
            submitted=len(self.submitted),
            completed=int(self.result is not None and self.result.command is not None),
            failed=int(self.result is not None and self.result.command is None),
            dropped_pending=0,
        )

    def close(self):
        if not self.closed:
            self.closed = True
            self.events.append("worker.close")


class FakePlanner:
    def __init__(self, command=None, error: Exception | None = None):
        self.command = fake_command(marker=1.0, tts=0.85) if command is None else command
        self.error = error
        self.calls = []
        self.strike_planner = SimpleNamespace(virtual_hit_plane_x=0.0)

    def plan_command(self, position, velocity, **kwargs):
        self.calls.append(
            (
                np.asarray(position).copy(),
                np.asarray(velocity).copy(),
                kwargs,
            )
        )
        if self.error is not None:
            raise self.error
        return self.command


def lifecycle():
    return HitterCommandLifecycle(
        waiting_tts=0.92,
        arm_tts=0.90,
        minimum_arm_tts=0.80,
        maximum_policy_tts=0.92,
        swing_duration_sampler=lambda: 1.85,
    )


def make_minimal_hitter_env(*, is_real: bool) -> HitterEnv:
    env = HitterEnv.__new__(HitterEnv)
    env.simulator = FakeSimulator(is_real=is_real)
    env.cfg = SimpleNamespace(control=SimpleNamespace(obs_clip_value=1000.0))
    env.policy_cfg = {"hitter_seed": 0}
    env.motion_cfg = {
        "playback_speed": 1.0,
        "waiting_time_to_strike_s": 0.92,
        "waiting_base_target_xy_w": [-0.4, 0.0],
        "reset_ball_on_command": True,
        "ball_planner": {
            "table_center_xy_w": [1.365369, 0.0],
            "table_height": 0.76,
            "table_length": 2.730738,
            "table_width": 1.512451,
            "maximum_hit_height_above_table_m": 0.50,
            "maximum_prediction_horizon_s": 5.0,
            "planner_update_rate_hz": 100.0,
            "arm_time_to_strike_s": 0.90,
            "minimum_arm_time_to_strike_s": 0.80,
            "maximum_policy_time_to_strike_s": 0.92,
            "swing_duration_range": [1.85, 1.85],
        },
    }
    env.playback_speed = 1.0
    env.action_beta = 1.0
    env.hitter_rng = np.random.default_rng(0)
    env.policy_model_meta = object()
    env.policy_default_joint_pos = np.zeros(29, dtype=np.float32)
    env.policy_action_scales = np.ones(29, dtype=np.float32)
    env.policy_joint_names = [f"joint_{index}" for index in range(29)]
    env._policy_dim = 29
    env._sim_to_policy = np.arange(29, dtype=np.int32)
    env._policy_to_sim = np.arange(29, dtype=np.int32)
    env._policy_to_sim_adapter = None
    env._sim_to_policy_adapter = None
    env.policy_default_joint_pos_sim = np.zeros(29, dtype=np.float32)
    env.prev_policy_action = np.zeros(29, dtype=np.float32)
    env.action = np.zeros((1, 29), dtype=np.float32)
    env.dof_pos = np.zeros((1, 29), dtype=np.float32)
    env.dof_vel = np.zeros((1, 29), dtype=np.float32)
    env.base_ang_vel = np.zeros((1, 3), dtype=np.float32)
    env.projected_gravity = np.array([[0.0, 0.0, -1.0]], dtype=np.float32)
    env.episode_length_buf = np.zeros(1, dtype=np.float64)
    env.first_obs_received = False
    env.hard_reset = False
    env.time_step = 0.0
    env.collected_traj = {}
    env.init_ref_dof_pos = None
    env.save_video_enabled = False
    env.reset_ball_on_command = True
    env.waiting_racket_body_name = "right_racket_link"
    env.waiting_time_to_strike_s = 0.92
    env._last_ball_estimator_not_ready_log_s = 0.0
    env._last_ball_planner_error_log_s = 0.0
    env._last_waiting_racket_target_error_log_s = 0.0
    env.hitter_ball_planner = FakePlanner()
    env._init_hitter_command_state()
    env.history_handler = SimpleNamespace(reset=lambda: None)

    env.hitter_command_lifecycle = lifecycle()
    env.hitter_planner_worker = FakeWorker() if is_real else None
    env._unregister_ball_listener = None
    env._hitter_last_result_key = None
    env._hitter_minimum_track_epoch = 0
    env._hitter_minimum_generation = 0
    env._hitter_observed_track_epoch = None
    env._hitter_sync_track_epoch = 0
    env._hitter_sync_generation = 0
    env._hitter_last_logged_phase = None
    env._hitter_last_status_log_monotonic_s = None
    env._hitter_closed = False
    return env


def inject_result(env: HitterEnv, item: PlannerResultSnapshot) -> None:
    if env.hitter_planner_worker is None:
        env.hitter_planner_worker = FakeWorker()
    env.hitter_planner_worker.result = item


def assert_command_marker(testcase: unittest.TestCase, env: HitterEnv, marker: float):
    expected = np.full(3, marker, dtype=np.float32)
    np.testing.assert_allclose(env.hitter_base_target_xy_w, expected[:2])
    np.testing.assert_allclose(env.hitter_racket_target_pos_w_fixed, expected)
    np.testing.assert_allclose(env.hitter_racket_target_vel_w, expected)
    testcase.assertEqual(env.hitter_strike_type, 0 if marker == 1.0 else 1)


def arm_env(*, epoch=2, generation=1, marker=1.0) -> HitterEnv:
    env = make_minimal_hitter_env(is_real=True)
    inject_result(
        env,
        planner_result(
            epoch=epoch,
            generation=generation,
            deadline=10.90,
            marker=marker,
        ),
    )
    env._update_hitter_command(now=10.0)
    return env


class HitterEnvLifecycleTests(unittest.TestCase):
    def test_waiting_observation_uses_training_maximum_tts(self):
        env = make_minimal_hitter_env(is_real=True)
        env._update_hitter_command(now=10.0)

        obs = env.refresh_policy_observation()["obs"]

        self.assertEqual(obs.shape, (1, 105))
        self.assertAlmostEqual(float(obs[0, 17]), 0.92)

    def test_reset_captures_current_pelvis_height_for_fixed_waiting_target(self):
        env = make_minimal_hitter_env(is_real=True)

        def calibrate(_refresh, _ref_dof_pos):
            env.simulator.root_trans_world = np.array(
                [-0.25, 0.15, 0.812], dtype=np.float32
            )

        env.simulator.calibrate = calibrate

        env._reset_envs(True)

        np.testing.assert_allclose(
            env.hitter_waiting_base_target_pos_w,
            [-0.4, 0.0, 0.812],
        )

    def test_reset_rejects_finite_but_invalid_real_world_pelvis_pose(self):
        env = make_minimal_hitter_env(is_real=True)

        def calibrate(_refresh, _ref_dof_pos):
            env.simulator.root_trans_world = np.array(
                [-0.25, 0.15, 0.812], dtype=np.float32
            )
            env.simulator.base_pose_valid = False

        env.simulator.calibrate = calibrate

        with self.assertRaisesRegex(RuntimeError, "valid G1Pelvis"):
            env._reset_envs(True)

    def test_waiting_observation_uses_fixed_world_target_in_yaw_frame(self):
        env = make_minimal_hitter_env(is_real=True)
        env.hitter_waiting_base_target_pos_w = np.array(
            [-0.4, 0.0, 0.812], dtype=np.float32
        )
        env.hitter_base_target_xy_w[:] = [9.0, 9.0]
        env.simulator.root_trans_world = np.array(
            [-0.3, 0.1, 0.792], dtype=np.float32
        )
        env.simulator.root_quat_world = np.array(
            [0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)], dtype=np.float32
        )

        env.compute_observation()

        np.testing.assert_allclose(
            env.obs_buf_dict["obs"][0, 8:11],
            [-0.1, 0.1, 0.02],
            atol=1.0e-6,
        )

    def test_result_above_arm_threshold_does_not_replace_waiting(self):
        env = make_minimal_hitter_env(is_real=True)
        before = env.hitter_base_target_xy_w.copy()
        inject_result(env, planner_result(deadline=10.95, marker=9.0))

        env._update_hitter_command(now=10.0)

        self.assertFalse(env.hitter_command_initialized)
        self.assertEqual(env.hitter_command_lifecycle.phase, CommandPhase.TRACKING)
        np.testing.assert_array_equal(env.hitter_base_target_xy_w, before)

    def test_same_track_override_updates_every_command_field_atomically(self):
        env = arm_env(epoch=2, generation=1, marker=1.0)
        inject_result(
            env,
            planner_result(
                epoch=2,
                generation=2,
                deadline=10.85,
                marker=2.0,
            ),
        )

        env._update_hitter_command(now=10.1)

        self.assertTrue(env.hitter_command_initialized)
        assert_command_marker(self, env, 2.0)

    def test_invalid_command_cannot_partially_mutate_active_fields(self):
        env = arm_env(epoch=2, generation=1, marker=1.0)
        active_before = env.hitter_command_lifecycle.active_result
        malformed = fake_command(marker=2.0, malformed=True)
        inject_result(
            env,
            planner_result(
                epoch=2,
                generation=2,
                deadline=10.85,
                command=malformed,
            ),
        )

        env._update_hitter_command(now=10.1)

        assert_command_marker(self, env, 1.0)
        self.assertIs(env.hitter_command_lifecycle.active_result, active_before)

    def test_old_generation_old_epoch_and_failed_result_do_not_mutate_command(self):
        env = arm_env(epoch=2, generation=2, marker=1.0)
        ignored = (
            planner_result(epoch=2, generation=1, deadline=10.8, marker=2.0),
            planner_result(epoch=1, generation=9, deadline=10.8, marker=3.0),
            planner_result(
                epoch=2,
                generation=3,
                deadline=float("nan"),
                command=None,
                error="ValueError: planner failed",
            ),
        )

        for item in ignored:
            inject_result(env, item)
            env._update_hitter_command(now=10.1)
            assert_command_marker(self, env, 1.0)

    def test_sticky_result_is_deduplicated_by_epoch_and_generation(self):
        env = arm_env(epoch=2, generation=1, marker=1.0)
        duplicate_key_new_object = planner_result(
            epoch=2,
            generation=1,
            deadline=10.85,
            marker=9.0,
        )
        inject_result(env, duplicate_key_new_object)

        env._update_hitter_command(now=10.1)

        assert_command_marker(self, env, 1.0)
        self.assertEqual(env._hitter_last_result_key, (2, 1))

    def test_too_large_override_retains_active_command(self):
        env = arm_env(epoch=2, generation=1, marker=1.0)
        inject_result(
            env,
            planner_result(epoch=2, generation=2, deadline=11.10, marker=2.0),
        )

        env._update_hitter_command(now=10.1)

        assert_command_marker(self, env, 1.0)

    def test_late_unarmed_result_skips_epoch_without_mutation(self):
        env = make_minimal_hitter_env(is_real=True)
        env.hitter_ball_sequence_needs_reset = False
        inject_result(env, planner_result(epoch=4, deadline=10.79, marker=2.0))

        env._update_hitter_command(now=10.0)

        self.assertFalse(env.hitter_command_initialized)
        self.assertEqual(env.hitter_command_lifecycle.phase, CommandPhase.WAITING)
        self.assertFalse(env.hitter_ball_sequence_needs_reset)
        np.testing.assert_array_equal(env.hitter_base_target_xy_w, [0.0, 0.0])

    def test_valid_planner_failure_does_not_end_track_or_block_later_override(self):
        env = arm_env(epoch=2, generation=1, marker=1.0)
        inject_result(
            env,
            planner_result(
                epoch=2,
                generation=2,
                command=None,
                error="ValueError: physical plan failed",
                deadline=float("nan"),
            ),
        )
        env._update_hitter_command(now=10.1)
        inject_result(
            env,
            planner_result(epoch=2, generation=3, deadline=10.84, marker=2.0),
        )

        env._update_hitter_command(now=10.1)

        assert_command_marker(self, env, 2.0)

    def test_old_epoch_failure_cannot_redirect_later_track_end(self):
        env = arm_env(epoch=2, generation=1, marker=1.0)
        inject_result(
            env,
            planner_result(
                epoch=1,
                generation=99,
                command=None,
                error="ValueError: stale planner failure",
                deadline=float("nan"),
            ),
        )
        env._update_hitter_command(now=10.1)
        inject_result(
            env,
            planner_result(
                epoch=3,
                generation=2,
                command=None,
                error=TRACK_ENDED_RESULT_ERROR,
                deadline=float("nan"),
            ),
        )
        env._update_hitter_command(now=10.2)
        inject_result(
            env,
            planner_result(epoch=2, generation=3, deadline=10.85, marker=2.0),
        )

        env._update_hitter_command(now=10.2)

        assert_command_marker(self, env, 1.0)

    def test_explicit_invalid_ends_track_but_preserves_active_recovery(self):
        env = arm_env(epoch=2, generation=1, marker=1.0)
        inject_result(
            env,
            planner_result(
                epoch=3,
                generation=2,
                deadline=float("nan"),
                command=None,
                error=TRACK_ENDED_RESULT_ERROR,
            ),
        )

        env._update_hitter_command(now=10.5)

        self.assertTrue(env.hitter_command_initialized)
        assert_command_marker(self, env, 1.0)
        inject_result(
            env,
            planner_result(epoch=2, generation=3, deadline=10.85, marker=2.0),
        )
        env._update_hitter_command(now=10.5)
        assert_command_marker(self, env, 1.0)

        env._update_hitter_command(now=11.86)
        self.assertFalse(env.hitter_command_initialized)

    def test_recovery_end_implicit_arm_copies_cached_command(self):
        env = arm_env(epoch=1, generation=1, marker=1.0)
        env._update_hitter_command(now=10.90)
        self.assertEqual(env.hitter_command_lifecycle.phase, CommandPhase.RECOVERY)
        inject_result(
            env,
            planner_result(epoch=2, generation=1, deadline=12.75, marker=2.0),
        )
        env._update_hitter_command(now=11.1)
        assert_command_marker(self, env, 1.0)

        env._update_hitter_command(now=11.85)

        self.assertEqual(env.hitter_command_lifecycle.phase, CommandPhase.ARMED)
        assert_command_marker(self, env, 2.0)

    def test_strike_deadline_advances_real_track_once(self):
        env = arm_env(epoch=2, generation=1, marker=1.0)
        env.simulator.ball_track_epoch = 2

        env._update_hitter_command(now=10.90)

        self.assertEqual(env.hitter_command_lifecycle.phase, CommandPhase.RECOVERY)
        self.assertEqual(env.simulator.reset_calls, 1)
        self.assertEqual(env.simulator.ball_track_epoch, 3)
        self.assertTrue(env.hitter_command_initialized)

        env._update_hitter_command(now=11.0)

        self.assertEqual(env.simulator.reset_calls, 1)
        self.assertEqual(env.simulator.ball_track_epoch, 3)
        self.assertEqual(env.hitter_command_lifecycle.phase, CommandPhase.RECOVERY)

    def test_new_epoch_can_arm_after_recovery_without_outgoing_observation(self):
        env = arm_env(epoch=2, generation=1, marker=1.0)
        env.simulator.ball_track_epoch = 2
        env._update_hitter_command(now=10.90)

        inject_result(
            env,
            planner_result(epoch=3, generation=1, deadline=12.75, marker=2.0),
        )
        env._update_hitter_command(now=11.0)

        self.assertEqual(env.simulator.reset_calls, 1)
        self.assertIsNotNone(env.hitter_command_lifecycle.cached_result)
        assert_command_marker(self, env, 1.0)

        env._update_hitter_command(now=11.85)

        self.assertEqual(env.hitter_command_lifecycle.phase, CommandPhase.ARMED)
        assert_command_marker(self, env, 2.0)

    def test_active_observation_tts_comes_from_monotonic_lifecycle(self):
        env = arm_env(epoch=2, generation=1, marker=1.0)

        with patch("envs.hitter.time.monotonic", return_value=10.2):
            env.compute_observation()

        self.assertAlmostEqual(float(env.obs_buf_dict["obs"][0, 17]), 0.7, places=6)

    def test_worker_planning_uses_only_snapshot_arrays(self):
        env = make_minimal_hitter_env(is_real=True)
        env.simulator.forbid_get_state = True
        command = fake_command(marker=2.0, tts=0.85)
        env.hitter_ball_planner = FakePlanner(command=command)
        for generation in (5, 6):
            with self.assertRaisesRegex(ValueError, "not stably incoming"):
                env._plan_hitter_snapshot(
                    ball_snapshot(epoch=4, generation=generation)
                )
        item = ball_snapshot(epoch=4, generation=7)

        planned = env._plan_hitter_snapshot(item)

        self.assertIs(planned, command)
        self.assertEqual(env.simulator.get_state_calls, 0)
        position, velocity, kwargs = env.hitter_ball_planner.calls[-1]
        np.testing.assert_array_equal(position, item.position_w)
        np.testing.assert_array_equal(velocity, item.velocity_w)
        np.testing.assert_array_equal(kwargs["current_base_xy_w"], item.base_position_w)

    def test_real_planner_requires_three_consecutive_stable_incoming_snapshots(self):
        env = make_minimal_hitter_env(is_real=True)
        command = fake_command(marker=2.0, tts=0.85)
        env.hitter_ball_planner = FakePlanner(command=command)

        for generation in (1, 2):
            with self.assertRaisesRegex(ValueError, "not stably incoming"):
                env._plan_hitter_snapshot(
                    ball_snapshot(epoch=4, generation=generation)
                )

        planned = env._plan_hitter_snapshot(
            ball_snapshot(epoch=4, generation=3)
        )

        self.assertIs(planned, command)
        self.assertEqual(len(env.hitter_ball_planner.calls), 1)

    def test_real_planner_new_epoch_requires_new_stable_confirmation(self):
        env = make_minimal_hitter_env(is_real=True)
        for generation in (1, 2, 3):
            try:
                env._plan_hitter_snapshot(
                    ball_snapshot(epoch=4, generation=generation)
                )
            except ValueError:
                pass

        with self.assertRaisesRegex(ValueError, "not stably incoming"):
            env._plan_hitter_snapshot(
                ball_snapshot(epoch=5, generation=4)
            )

    def test_mujoco_planner_does_not_require_stable_confirmation(self):
        env = make_minimal_hitter_env(is_real=False)
        command = fake_command(marker=2.0, tts=0.85)
        env.hitter_ball_planner = FakePlanner(command=command)

        planned = env._plan_hitter_snapshot(
            ball_snapshot(epoch=4, generation=1)
        )

        self.assertIs(planned, command)
        self.assertEqual(len(env.hitter_ball_planner.calls), 1)

    def test_snapshot_rejection_distinguishes_track_end_from_other_failures(self):
        env = make_minimal_hitter_env(is_real=True)
        with self.assertRaisesRegex(RuntimeError, "HITTER_TRACK_ENDED"):
            env._plan_hitter_snapshot(ball_snapshot(visible=False))
        with self.assertRaisesRegex(ValueError, "ready"):
            env._plan_hitter_snapshot(ball_snapshot(ready=False))
        with self.assertRaisesRegex(ValueError, "base"):
            env._plan_hitter_snapshot(ball_snapshot(base_valid=False))

    def test_mujoco_synchronous_plan_uses_same_result_and_lifecycle(self):
        env = make_minimal_hitter_env(is_real=False)
        env.simulator.ball_visible = True
        env.simulator.ball_state_estimator_ready = True
        env.hitter_ball_planner = FakePlanner(
            command=fake_command(marker=2.0, tts=0.85)
        )

        env._update_hitter_command(now=10.0)

        self.assertIsNone(env.hitter_planner_worker)
        self.assertIsInstance(
            env.hitter_command_lifecycle.active_result,
            PlannerResultSnapshot,
        )
        assert_command_marker(self, env, 2.0)

    def test_mujoco_late_skip_starts_new_sequence_and_can_arm(self):
        env = make_minimal_hitter_env(is_real=False)
        env.simulator.ball_visible = True
        env.simulator.ball_state_estimator_ready = True
        planner = FakePlanner(command=fake_command(marker=2.0, tts=0.79))
        env.hitter_ball_planner = planner

        env._update_hitter_command(now=10.0)
        self.assertEqual(env.hitter_command_lifecycle.phase, CommandPhase.WAITING)
        self.assertFalse(env.hitter_command_initialized)

        planner.command = fake_command(marker=2.0, tts=0.85)
        env._update_hitter_command(now=10.02)

        self.assertEqual(env.simulator.events.count("reset_ball"), 2)
        self.assertEqual(env._hitter_sync_track_epoch, 2)
        self.assertEqual(env.hitter_command_lifecycle.phase, CommandPhase.ARMED)
        self.assertTrue(env.hitter_command_initialized)
        self.assertEqual(
            env.hitter_command_lifecycle.active_result.track_epoch,
            2,
        )
        assert_command_marker(self, env, 2.0)

    def test_real_runtime_registers_actual_worker_but_mujoco_does_not(self):
        real = make_minimal_hitter_env(is_real=True)
        real.hitter_planner_worker = None
        real._initialize_hitter_realtime_runtime()
        try:
            self.assertIsInstance(real.hitter_planner_worker, LatestOnlyPlannerWorker)
            self.assertIsNotNone(real.simulator.listener)
            self.assertIs(real.simulator.listener.__self__, real)
        finally:
            real.close()

        mujoco = make_minimal_hitter_env(is_real=False)
        mujoco._initialize_hitter_realtime_runtime()
        self.assertIsNone(mujoco.hitter_planner_worker)
        self.assertIsNone(mujoco.simulator.listener)

    def test_real_planner_listener_throttles_snapshot_submissions_to_100hz(self):
        env = make_minimal_hitter_env(is_real=True)
        env.hitter_planner_worker = FakeWorker()

        for generation, received in enumerate(
            [10.000, 10.002, 10.009, 10.010, 10.019, 10.020],
            start=1,
        ):
            env._submit_hitter_planner_snapshot(
                ball_snapshot(generation=generation, received=received)
            )

        self.assertEqual(
            [snapshot.generation for snapshot in env.hitter_planner_worker.submitted],
            [1, 4, 6],
        )

    def test_reset_invalidates_sticky_and_inflight_old_results(self):
        env = make_minimal_hitter_env(is_real=True)
        old = planner_result(epoch=0, generation=1, deadline=10.9, marker=9.0)
        inject_result(env, old)

        with patch.object(BaseEnv, "reset", return_value={"obs": np.zeros((1, 105))}):
            env.reset()

        env._update_hitter_command(now=10.0)
        self.assertFalse(env.hitter_command_initialized)
        stale_inflight = planner_result(
            epoch=0,
            generation=2,
            deadline=10.9,
            marker=8.0,
        )
        inject_result(env, stale_inflight)
        env._update_hitter_command(now=10.0)
        self.assertFalse(env.hitter_command_initialized)

        fresh = planner_result(epoch=1, generation=2, deadline=10.9, marker=1.0)
        inject_result(env, fresh)
        env._update_hitter_command(now=10.0)
        assert_command_marker(self, env, 1.0)

    def test_hard_termination_invalidates_active_and_inflight_results(self):
        env = arm_env(epoch=0, generation=1, marker=1.0)
        env.simulator.check_termination = lambda: True

        with patch.object(env, "_save_collected_traj"), patch.object(
            env,
            "_reset_envs",
        ) as reset_envs:
            env._check_termination()

        reset_envs.assert_called_once_with(True)
        self.assertEqual(env.simulator.reset_calls, 1)
        self.assertFalse(env.hitter_command_initialized)
        self.assertEqual(env.hitter_command_lifecycle.phase, CommandPhase.WAITING)
        self.assertIsNone(env._hitter_last_result_key)

        inject_result(
            env,
            planner_result(epoch=0, generation=2, deadline=10.9, marker=9.0),
        )
        env._update_hitter_command(now=10.0)
        self.assertFalse(env.hitter_command_initialized)
        np.testing.assert_array_equal(env.hitter_base_target_xy_w, [0.0, 0.0])

    def test_waiting_step_keeps_policy_action_path_ungated(self):
        env = make_minimal_hitter_env(is_real=True)
        action = np.linspace(-0.5, 0.5, 29, dtype=np.float32)
        expected = action.reshape(1, -1)

        with patch.object(BaseEnv, "step", return_value={"obs": None}) as base_step:
            result = env.step(action)

        self.assertEqual(result, {"obs": None})
        np.testing.assert_array_equal(base_step.call_args.args[0], expected)

    def test_close_orders_unregister_worker_simulator_and_is_idempotent(self):
        events = []
        env = make_minimal_hitter_env(is_real=True)
        env.simulator.events = events
        env.hitter_planner_worker = FakeWorker(events=events)
        env._unregister_ball_listener = env.simulator.register_hitter_ball_listener(
            env.hitter_planner_worker.submit
        )

        self.assertIs(env.close(), True)
        self.assertIs(env.close(), True)

        self.assertEqual(events, ["unregister", "worker.close", "simulator.close"])
        self.assertEqual(env.simulator.close_calls, 1)
        self.assertTrue(env._hitter_closed)

    def test_close_retries_incomplete_real_world_after_one_time_teardown(self):
        class FakeLcm:
            def __init__(self):
                self.unsubscribed = []

            def unsubscribe(self, subscription):
                self.unsubscribed.append(subscription)

        class FakeThread:
            def __init__(self):
                self.alive = True
                self.join_timeouts = []

            def is_alive(self):
                return self.alive

            def join(self, timeout=None):
                self.join_timeouts.append(timeout)

        events = []
        env = make_minimal_hitter_env(is_real=True)
        worker = FakeWorker(events=events)
        env.hitter_planner_worker = worker

        def unregister():
            events.append("unregister")

        env._unregister_ball_listener = unregister

        real_world = RealWorld.__new__(RealWorld)
        real_world.lc = FakeLcm()
        subscriptions = {
            "root_state_subscriber": "root",
            "joint_state_subscriber": "joint",
            "vicon_state_subscriber": "vicon",
            "remote_controller_subscriber": "remote",
            "teleop_state_subscriber": "teleop",
        }
        for attribute, subscription in subscriptions.items():
            setattr(real_world, attribute, subscription)
        real_world._poll_stop_event = threading.Event()
        real_world._communication_close_lock = threading.Lock()
        real_world._communication_closed = False
        real_world.run_thread = FakeThread()
        env.simulator = real_world

        with patch("simulator.real_world.logger.warning") as warning:
            self.assertIs(env.close(), False)

        self.assertEqual(warning.call_count, 1)
        self.assertEqual(events, ["unregister", "worker.close"])
        self.assertIsNone(env._unregister_ball_listener)
        self.assertIsNone(env.hitter_planner_worker)
        self.assertFalse(env._hitter_closed)
        self.assertEqual(real_world.lc.unsubscribed, [])
        self.assertFalse(real_world._communication_closed)

        real_world.run_thread.alive = False
        self.assertIs(env.close(), True)

        self.assertTrue(env._hitter_closed)
        self.assertCountEqual(real_world.lc.unsubscribed, subscriptions.values())
        self.assertEqual(len(real_world.lc.unsubscribed), len(subscriptions))
        self.assertIs(env.close(), True)
        self.assertEqual(events, ["unregister", "worker.close"])
        self.assertEqual(real_world.run_thread.join_timeouts, [1.0])
        self.assertEqual(len(real_world.lc.unsubscribed), len(subscriptions))

    def test_close_exception_is_retryable_without_repeating_one_time_teardown(self):
        events = []
        env = make_minimal_hitter_env(is_real=True)
        env.simulator.events = events
        env.hitter_planner_worker = FakeWorker(events=events)
        env._unregister_ball_listener = env.simulator.register_hitter_ball_listener(
            env.hitter_planner_worker.submit
        )
        close_attempts = []

        def close_simulator():
            close_attempts.append(len(close_attempts) + 1)
            events.append("simulator.close")
            if len(close_attempts) == 1:
                raise RuntimeError("incomplete simulator shutdown")
            return None

        env.simulator.close = close_simulator

        with self.assertRaisesRegex(RuntimeError, "incomplete simulator shutdown"):
            env.close()

        self.assertFalse(env._hitter_closed)
        self.assertEqual(
            events,
            ["unregister", "worker.close", "simulator.close"],
        )
        self.assertIsNone(env._unregister_ball_listener)
        self.assertIsNone(env.hitter_planner_worker)

        self.assertIs(env.close(), True)
        self.assertIs(env.close(), True)
        self.assertEqual(close_attempts, [1, 2])
        self.assertEqual(
            events,
            [
                "unregister",
                "worker.close",
                "simulator.close",
                "simulator.close",
            ],
        )

    def test_close_retries_failed_unregister_without_repeating_other_cleanup(self):
        events = []
        env = make_minimal_hitter_env(is_real=True)
        env.simulator.events = events
        env.hitter_planner_worker = FakeWorker(events=events)
        unregister_attempts = []

        def unregister():
            unregister_attempts.append(len(unregister_attempts) + 1)
            events.append("unregister")
            if len(unregister_attempts) == 1:
                raise RuntimeError("unregister failed")

        env._unregister_ball_listener = unregister

        with self.assertRaisesRegex(RuntimeError, "unregister failed"):
            env.close()

        self.assertFalse(env._hitter_closed)
        self.assertIs(env._unregister_ball_listener, unregister)
        self.assertIsNone(env.hitter_planner_worker)
        self.assertEqual(env.simulator.close_calls, 1)

        self.assertIs(env.close(), True)
        self.assertIs(env.close(), True)
        self.assertEqual(unregister_attempts, [1, 2])
        self.assertEqual(env.simulator.close_calls, 1)
        self.assertEqual(
            events,
            [
                "unregister",
                "worker.close",
                "simulator.close",
                "unregister",
            ],
        )

    def test_close_retries_failed_worker_without_repeating_other_cleanup(self):
        class FlakyWorker:
            def __init__(self, events):
                self.events = events
                self.close_attempts = []

            def close(self):
                self.close_attempts.append(len(self.close_attempts) + 1)
                self.events.append("worker.close")
                if len(self.close_attempts) == 1:
                    raise RuntimeError("worker close failed")

        events = []
        env = make_minimal_hitter_env(is_real=True)
        env.simulator.events = events
        worker = FlakyWorker(events)
        env.hitter_planner_worker = worker
        env._unregister_ball_listener = env.simulator.register_hitter_ball_listener(
            lambda _snapshot: None
        )

        with self.assertRaisesRegex(RuntimeError, "worker close failed"):
            env.close()

        self.assertFalse(env._hitter_closed)
        self.assertIsNone(env._unregister_ball_listener)
        self.assertIs(env.hitter_planner_worker, worker)
        self.assertEqual(env.simulator.close_calls, 1)

        self.assertIs(env.close(), True)
        self.assertIs(env.close(), True)
        self.assertEqual(worker.close_attempts, [1, 2])
        self.assertEqual(env.simulator.close_calls, 1)
        self.assertEqual(
            events,
            [
                "unregister",
                "worker.close",
                "simulator.close",
                "worker.close",
            ],
        )

    def test_planner_builder_forwards_height_and_maximum_horizon(self):
        env = HitterEnv.__new__(HitterEnv)
        env.motion_cfg = {
            "ball_planner": {
                "table_height": 0.80,
                "maximum_hit_height_above_table_m": 0.40,
                "maximum_prediction_horizon_s": 4.5,
            }
        }

        planner = env._build_hitter_ball_planner().strike_planner

        self.assertEqual(planner.minimum_hit_height, 0.80)
        self.assertEqual(planner.maximum_hit_height, 1.20)
        self.assertEqual(planner.maximum_prediction_horizon_s, 4.5)

    def test_final_config_contains_exact_realtime_values(self):
        config_path = (
            Path(__file__).resolve().parents[1] / "config" / "mimic" / "hitter.yaml"
        )
        config = OmegaConf.to_container(OmegaConf.load(config_path), resolve=True)
        motion = config["motion"]
        planner = motion["ball_planner"]

        self.assertEqual(motion["waiting_time_to_strike_s"], 0.92)
        self.assertEqual(motion["waiting_base_target_xy_w"], [-0.4, 0.0])
        self.assertEqual(planner["table_center_xy_w"], [1.365369, 0.0])
        self.assertEqual(planner["table_length"], 2.730738)
        self.assertEqual(planner["table_width"], 1.512451)
        self.assertEqual(planner["table_height"], 0.76)
        self.assertEqual(planner["virtual_hit_plane_x"], 0.0)
        self.assertEqual(planner["state_estimator_sample_rate_hz"], 360.0)
        self.assertEqual(planner["planner_update_rate_hz"], 100.0)
        self.assertEqual(planner["minimum_stable_incoming_speed_x_mps"], 0.20)
        self.assertEqual(planner["stable_incoming_confirmation_snapshots"], 3)
        self.assertEqual(planner["arm_time_to_strike_s"], 0.92)
        self.assertEqual(planner["minimum_arm_time_to_strike_s"], 0.60)
        self.assertEqual(planner["maximum_policy_time_to_strike_s"], 0.92)
        self.assertEqual(planner["maximum_hit_height_above_table_m"], 0.50)
        self.assertEqual(planner["maximum_prediction_horizon_s"], 5.0)
        self.assertEqual(planner["swing_duration_range"], [1.75, 1.95])


def tearDownModule():
    live_workers = [
        thread.name
        for thread in threading.enumerate()
        if thread.name.startswith("LatestOnlyPlannerWorker-")
    ]
    if live_workers:
        raise AssertionError(f"live planner workers after env tests: {live_workers}")


if __name__ == "__main__":
    unittest.main()
