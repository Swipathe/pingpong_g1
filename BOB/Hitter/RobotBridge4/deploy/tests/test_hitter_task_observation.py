from __future__ import annotations

import unittest
import warnings
import threading
from dataclasses import FrozenInstanceError
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation as sRot

from envs.hitter import HitterEnv
from utils.hitter_realtime import CommandPhase
from utils.hitter_realtime import LifecycleDecision
from utils.hitter_task_observation import (
    TaskObservationAssemblyError,
    assemble_active_hitter_task_observation,
)


def legacy_active_task_slice(
    *,
    robot_anchor_position_w: np.ndarray,
    robot_anchor_quaternion_xyzw: np.ndarray,
    base_target_xy_w: np.ndarray,
    racket_target_position_w: np.ndarray,
    racket_target_velocity_w: np.ndarray,
    policy_time_to_strike_s: float,
) -> np.ndarray:
    """Copy the pre-extraction active branch formulas without using the helper."""
    robot_anchor_position_w = np.asarray(
        robot_anchor_position_w,
        dtype=np.float32,
    ).copy()
    robot_anchor_quaternion_xyzw = np.asarray(
        robot_anchor_quaternion_xyzw,
        dtype=np.float32,
    ).copy()
    base_target_xy_w = np.asarray(base_target_xy_w, dtype=np.float32).copy()
    racket_target_position_w = np.asarray(
        racket_target_position_w,
        dtype=np.float32,
    ).copy()
    racket_target_velocity_w = np.asarray(
        racket_target_velocity_w,
        dtype=np.float32,
    ).copy()

    if np.linalg.norm(robot_anchor_quaternion_xyzw) < 1.0e-6:
        robot_anchor_quaternion_xyzw = np.asarray(
            [0.0, 0.0, 0.0, 1.0],
            dtype=np.float32,
        )
    else:
        robot_anchor_quaternion_xyzw = (
            robot_anchor_quaternion_xyzw
            / np.linalg.norm(robot_anchor_quaternion_xyzw)
        ).astype(np.float32)

    base_forward_xy_w = (
        sRot.from_quat(robot_anchor_quaternion_xyzw)
        .as_matrix()[:, 0][:2]
        .astype(np.float32)
    )
    yaw = float(
        sRot.from_quat(robot_anchor_quaternion_xyzw).as_euler(
            "xyz",
            degrees=False,
        )[2]
    )
    yaw_inverse = sRot.from_euler("z", -yaw, degrees=False)

    base_target_delta_w = np.asarray(
        [
            base_target_xy_w[0] - robot_anchor_position_w[0],
            base_target_xy_w[1] - robot_anchor_position_w[1],
            0.0,
        ],
        dtype=np.float32,
    )
    base_target_xy_b = yaw_inverse.apply(base_target_delta_w).astype(
        np.float32
    )[:2]
    racket_target_pos_b = yaw_inverse.apply(
        racket_target_position_w - robot_anchor_position_w
    ).astype(np.float32)

    return np.concatenate(
        [
            base_forward_xy_w,
            base_target_xy_b,
            racket_target_pos_b,
            racket_target_velocity_w,
            np.asarray([policy_time_to_strike_s], dtype=np.float32),
        ],
        axis=0,
    ).astype(np.float32)


def valid_inputs(**overrides):
    values = {
        "robot_anchor_position_w": np.asarray(
            [1.0, 2.0, 0.5],
            dtype=np.float64,
        ),
        "robot_anchor_quaternion_xyzw": np.asarray(
            [0.0, 0.0, 0.0, 1.0],
            dtype=np.float64,
        ),
        "base_target_xy_w": np.asarray([3.0, 5.0], dtype=np.float64),
        "racket_target_position_w": np.asarray(
            [4.0, 6.0, 1.5],
            dtype=np.float64,
        ),
        "racket_target_velocity_w": np.asarray(
            [0.25, -0.5, 0.75],
            dtype=np.float64,
        ),
        "policy_time_to_strike_s": 0.75,
        "maximum_policy_time_to_strike_s": 0.92,
        "obs_clip_value": None,
    }
    values.update(overrides)
    return values


class TaskObservationAssemblyTests(unittest.TestCase):
    def test_matches_legacy_active_formulas_for_yaw_table(self):
        cases = (
            (0.0, [1.0, 0.0], [2.0, 3.0]),
            (90.0, [0.0, 1.0], [3.0, -2.0]),
            (-90.0, [0.0, -1.0], [-3.0, 2.0]),
        )

        for yaw_degrees, expected_forward, expected_base_target in cases:
            with self.subTest(yaw_degrees=yaw_degrees):
                quaternion = (
                    sRot.from_euler("z", yaw_degrees, degrees=True).as_quat()
                    * 3.0
                )
                inputs = valid_inputs(
                    robot_anchor_quaternion_xyzw=quaternion,
                )

                result = assemble_active_hitter_task_observation(**inputs)
                expected = legacy_active_task_slice(
                    robot_anchor_position_w=inputs[
                        "robot_anchor_position_w"
                    ],
                    robot_anchor_quaternion_xyzw=quaternion,
                    base_target_xy_w=inputs["base_target_xy_w"],
                    racket_target_position_w=inputs[
                        "racket_target_position_w"
                    ],
                    racket_target_velocity_w=inputs[
                        "racket_target_velocity_w"
                    ],
                    policy_time_to_strike_s=inputs[
                        "policy_time_to_strike_s"
                    ],
                )

                np.testing.assert_allclose(
                    result.pre_clip,
                    expected,
                    atol=1.0e-6,
                )
                np.testing.assert_allclose(
                    result.pre_clip[0:2],
                    expected_forward,
                    atol=1.0e-6,
                )
                np.testing.assert_allclose(
                    result.pre_clip[2:4],
                    expected_base_target,
                    atol=1.0e-6,
                )
                np.testing.assert_array_equal(
                    result.pre_clip[7:10],
                    inputs["racket_target_velocity_w"].astype(np.float32),
                )

    def test_zero_length_quaternion_uses_identity_anchor_orientation(self):
        inputs = valid_inputs(
            robot_anchor_quaternion_xyzw=np.zeros(4, dtype=np.float64),
        )

        result = assemble_active_hitter_task_observation(**inputs)

        np.testing.assert_allclose(
            result.pre_clip,
            legacy_active_task_slice(
                robot_anchor_position_w=inputs["robot_anchor_position_w"],
                robot_anchor_quaternion_xyzw=inputs[
                    "robot_anchor_quaternion_xyzw"
                ],
                base_target_xy_w=inputs["base_target_xy_w"],
                racket_target_position_w=inputs[
                    "racket_target_position_w"
                ],
                racket_target_velocity_w=inputs[
                    "racket_target_velocity_w"
                ],
                policy_time_to_strike_s=inputs[
                    "policy_time_to_strike_s"
                ],
            ),
            atol=1.0e-6,
        )

    def test_result_arrays_are_float32_fixed_shape_read_only_and_copied(self):
        inputs = valid_inputs()

        result = assemble_active_hitter_task_observation(**inputs)
        before = result.pre_clip.copy()

        self.assertEqual(result.pre_clip.shape, (11,))
        self.assertEqual(result.post_clip.shape, (11,))
        self.assertEqual(result.clip_mask.shape, (11,))
        self.assertEqual(result.pre_clip.dtype, np.dtype(np.float32))
        self.assertEqual(result.post_clip.dtype, np.dtype(np.float32))
        self.assertEqual(result.clip_mask.dtype, np.dtype(np.bool_))
        self.assertFalse(result.pre_clip.flags.writeable)
        self.assertFalse(result.post_clip.flags.writeable)
        self.assertFalse(result.clip_mask.flags.writeable)
        self.assertFalse(
            np.shares_memory(
                result.pre_clip,
                inputs["robot_anchor_position_w"],
            )
        )
        self.assertFalse(
            np.shares_memory(
                result.post_clip,
                result.pre_clip,
            )
        )

        inputs["robot_anchor_position_w"][:] = 99.0
        inputs["base_target_xy_w"][:] = -99.0
        np.testing.assert_array_equal(result.pre_clip, before)
        with self.assertRaises(ValueError):
            result.pre_clip[0] = 0.0
        with self.assertRaises(ValueError):
            result.post_clip[0] = 0.0
        with self.assertRaises(ValueError):
            result.clip_mask[0] = True
        with self.assertRaises(FrozenInstanceError):
            result.clip_count = 99

    def test_wrong_input_shapes_raise_coded_error_without_assembly(self):
        invalid_fields = {
            "robot_anchor_position_w": np.zeros(2, dtype=np.float32),
            "robot_anchor_quaternion_xyzw": np.asarray(
                [np.nan, 0.0, 1.0],
                dtype=np.float32,
            ),
            "base_target_xy_w": np.zeros((1, 2), dtype=np.float32),
            "racket_target_position_w": np.zeros(4, dtype=np.float32),
            "racket_target_velocity_w": np.zeros((3, 1), dtype=np.float32),
        }

        for field, invalid_value in invalid_fields.items():
            with self.subTest(field=field):
                inputs = valid_inputs(**{field: invalid_value})
                with self.assertRaises(
                    TaskObservationAssemblyError
                ) as raised:
                    assemble_active_hitter_task_observation(**inputs)
                self.assertEqual(
                    raised.exception.reason_code,
                    "OBS_WRONG_SHAPE",
                )

    def test_positive_clip_returns_pre_and_post_values_and_finite_mask(self):
        inputs = valid_inputs(
            robot_anchor_position_w=np.zeros(3, dtype=np.float32),
            base_target_xy_w=np.asarray([2.0, -3.0]),
            racket_target_position_w=np.asarray([0.5, -4.0, 1.0]),
            racket_target_velocity_w=np.asarray([5.0, -0.25, -6.0]),
            policy_time_to_strike_s=0.9,
            obs_clip_value=1.0,
        )

        result = assemble_active_hitter_task_observation(**inputs)

        expected_pre = np.asarray(
            [
                1.0,
                0.0,
                2.0,
                -3.0,
                0.5,
                -4.0,
                1.0,
                5.0,
                -0.25,
                -6.0,
                0.9,
            ],
            dtype=np.float32,
        )
        expected_post = np.asarray(
            [
                1.0,
                0.0,
                1.0,
                -1.0,
                0.5,
                -1.0,
                1.0,
                1.0,
                -0.25,
                -1.0,
                0.9,
            ],
            dtype=np.float32,
        )
        expected_mask = np.asarray(
            [
                False,
                False,
                True,
                True,
                False,
                True,
                False,
                True,
                False,
                True,
                False,
            ],
            dtype=bool,
        )
        np.testing.assert_array_equal(result.pre_clip, expected_pre)
        np.testing.assert_array_equal(result.post_clip, expected_post)
        np.testing.assert_array_equal(result.clip_mask, expected_mask)
        self.assertEqual(result.clip_count, 5)
        self.assertEqual(result.errors, ("OBS_CLIPPED",))

    def test_none_zero_and_negative_clip_values_leave_values_unchanged(self):
        for clip_value in (None, 0.0, -1.0):
            with self.subTest(clip_value=clip_value):
                result = assemble_active_hitter_task_observation(
                    **valid_inputs(obs_clip_value=clip_value)
                )

                np.testing.assert_array_equal(
                    result.post_clip,
                    result.pre_clip,
                )
                np.testing.assert_array_equal(
                    result.clip_mask,
                    np.zeros(11, dtype=bool),
                )
                self.assertEqual(result.clip_count, 0)
                self.assertEqual(result.errors, ())

    def test_nonfinite_values_are_not_counted_as_clipped(self):
        result = assemble_active_hitter_task_observation(
            **valid_inputs(
                robot_anchor_position_w=np.zeros(3, dtype=np.float32),
                base_target_xy_w=np.asarray([0.1, 0.2]),
                racket_target_position_w=np.asarray([0.1, -0.1, 0.2]),
                racket_target_velocity_w=np.asarray(
                    [np.nan, np.inf, -np.inf]
                ),
                policy_time_to_strike_s=0.2,
                obs_clip_value=2.0,
            )
        )

        self.assertTrue(np.isnan(result.pre_clip[7]))
        self.assertTrue(np.isposinf(result.pre_clip[8]))
        self.assertTrue(np.isneginf(result.pre_clip[9]))
        self.assertTrue(np.isnan(result.post_clip[7]))
        self.assertEqual(float(result.post_clip[8]), 2.0)
        self.assertEqual(float(result.post_clip[9]), -2.0)
        np.testing.assert_array_equal(
            result.clip_mask[7:10],
            np.zeros(3, dtype=bool),
        )
        self.assertEqual(result.clip_count, 0)
        self.assertEqual(result.errors, ("OBS_NONFINITE",))

    def test_nonfinite_quaternion_reports_error_without_runtime_warning(self):
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            result = assemble_active_hitter_task_observation(
                **valid_inputs(
                    robot_anchor_quaternion_xyzw=np.asarray(
                        [np.inf, 0.0, 0.0, 0.0],
                        dtype=np.float32,
                    )
                )
            )

        self.assertEqual(caught, [])
        self.assertEqual(result.errors, ("OBS_NONFINITE",))
        self.assertFalse(np.any(np.isfinite(result.pre_clip[0:7])))

    def test_finite_clip_and_nonfinite_report_both_without_double_count(self):
        result = assemble_active_hitter_task_observation(
            **valid_inputs(
                robot_anchor_position_w=np.zeros(3, dtype=np.float32),
                base_target_xy_w=np.asarray([0.1, 0.2]),
                racket_target_position_w=np.asarray([0.1, -0.1, 0.2]),
                racket_target_velocity_w=np.asarray([np.inf, 3.0, 0.0]),
                policy_time_to_strike_s=0.2,
                obs_clip_value=2.0,
            )
        )

        self.assertFalse(bool(result.clip_mask[7]))
        self.assertTrue(bool(result.clip_mask[8]))
        self.assertEqual(result.clip_count, 1)
        self.assertEqual(
            result.errors,
            ("OBS_NONFINITE", "OBS_CLIPPED"),
        )

    def test_tts_range_accepts_open_zero_closed_maximum_interval(self):
        cases = (
            (1.0e-6, ()),
            (0.92, ()),
            (0.0, ("OBS_TTS_OUT_OF_RANGE",)),
            (-0.01, ("OBS_TTS_OUT_OF_RANGE",)),
            (0.9201, ("OBS_TTS_OUT_OF_RANGE",)),
            (np.nan, ("OBS_NONFINITE",)),
            (np.inf, ("OBS_NONFINITE",)),
        )

        for tts, expected_errors in cases:
            with self.subTest(tts=tts):
                result = assemble_active_hitter_task_observation(
                    **valid_inputs(policy_time_to_strike_s=tts)
                )
                self.assertEqual(result.errors, expected_errors)


class RecordingLifecycle:
    def __init__(self, events):
        self.events = events
        self.phase = CommandPhase.ARMED
        self.active_result = object()
        self.cached_result = None
        self.command_end_deadline_s = None
        self.maximum_policy_tts = 0.92

    def advance(self, now):
        self.events.append(("advance", float(now)))
        return LifecycleDecision(kind="none", track_id=None)

    def policy_tts(self, *, now):
        self.events.append(("policy_tts", float(now)))
        return 10.9 - float(now)


class MinimalSimulator:
    is_real = True

    def __init__(self):
        self.root_trans_world = np.asarray(
            [1.0, 2.0, 0.5],
            dtype=np.float32,
        )
        self.root_quat_world = np.asarray(
            [0.0, 0.0, 0.0, 1.0],
            dtype=np.float32,
        )

    def update_obs(self):
        return {}

    def hitter_vicon_status(self, *, now_monotonic_s):
        del now_monotonic_s
        return SimpleNamespace(
            base_pose_valid=True,
            latched_fault=None,
            active_track_id=None,
        )

    def drain_hitter_vicon_events(self):
        return ()


def minimal_active_env(events) -> HitterEnv:
    env = HitterEnv.__new__(HitterEnv)
    env.simulator = MinimalSimulator()
    env.cfg = SimpleNamespace(
        control=SimpleNamespace(obs_clip_value=1000.0)
    )
    env.first_obs_received = True
    env.policy_default_joint_pos = np.zeros(29, dtype=np.float32)
    env.prev_policy_action = np.zeros(29, dtype=np.float32)
    env.dof_pos = np.zeros((1, 29), dtype=np.float32)
    env.dof_vel = np.zeros((1, 29), dtype=np.float32)
    env.base_ang_vel = np.zeros((1, 3), dtype=np.float32)
    env.projected_gravity = np.asarray(
        [[0.0, 0.0, -1.0]],
        dtype=np.float32,
    )
    env._policy_dim = 29
    env._sim_to_policy = np.arange(29, dtype=np.int32)
    env._sim_to_policy_adapter = None
    env.hitter_base_target_xy_w = np.asarray(
        [3.0, 5.0],
        dtype=np.float32,
    )
    env.hitter_base_target_z_w = 0.5
    env.hitter_racket_target_pos_w_fixed = np.asarray(
        [4.0, 6.0, 1.5],
        dtype=np.float32,
    )
    env.hitter_racket_target_vel_w = np.asarray(
        [0.25, -0.5, 0.75],
        dtype=np.float32,
    )
    env.hitter_command_lifecycle = RecordingLifecycle(events)
    env.hitter_command_initialized = True
    env.hitter_planner_worker = None
    env.hitter_ball_sequence_needs_reset = False
    env._hitter_last_logged_phase = CommandPhase.ARMED
    env._hitter_lifecycle_lock = threading.RLock()
    env._hitter_status_fault_reasons_applied = set()
    env._waiting_anchor_fault = None
    return env


class HitterObservationClockTests(unittest.TestCase):
    def test_refresh_uses_separate_lifecycle_and_observation_monotonic_reads(self):
        events = []
        env = minimal_active_env(events)

        with patch(
            "envs.hitter.time.monotonic",
            side_effect=[10.0, 10.2],
        ) as monotonic:
            observation = env.refresh_policy_observation()["obs"]

        self.assertEqual(monotonic.call_count, 2)
        self.assertEqual(
            events,
            [
                ("advance", 10.0),
                ("policy_tts", 10.2),
            ],
        )
        self.assertEqual(observation.shape, (1, 104))
        self.assertAlmostEqual(float(observation[0, 16]), 0.7, places=6)
        expected_task = legacy_active_task_slice(
            robot_anchor_position_w=env.simulator.root_trans_world,
            robot_anchor_quaternion_xyzw=env.simulator.root_quat_world,
            base_target_xy_w=env.hitter_base_target_xy_w,
            racket_target_position_w=env.hitter_racket_target_pos_w_fixed,
            racket_target_velocity_w=env.hitter_racket_target_vel_w,
            policy_time_to_strike_s=0.7,
        )
        np.testing.assert_allclose(
            observation[0, 6:17],
            expected_task,
            atol=1.0e-6,
        )


if __name__ == "__main__":
    unittest.main()
