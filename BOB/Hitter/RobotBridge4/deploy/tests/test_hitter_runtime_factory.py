from __future__ import annotations

import unittest
from dataclasses import FrozenInstanceError
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from omegaconf import OmegaConf

from simulator.real_world import RealWorld
from tests.hitter_test_factories import success
from utils.hitter_planner import (
    BallStateEstimator,
    BallTrajectoryPredictor,
    BaseTargetPlanner,
    HitterSystemPlanner,
    StrikePlanner,
)
from utils.hitter_runtime_factory import (
    HitterRuntimeSettings,
    build_ball_state_estimator,
    build_hitter_command_lifecycle,
    build_hitter_system_planner,
    forced_strike_type,
    resolve_hitter_runtime_settings,
    resolve_vicon_consumer_settings,
)


DEPLOY_DIR = Path(__file__).resolve().parents[1]


def load_yaml_mapping(relative_path: str) -> dict:
    loaded = OmegaConf.load(DEPLOY_DIR / relative_path)
    return OmegaConf.to_container(loaded, resolve=True)


def reference_ball_state_estimator(planner_config: dict) -> BallStateEstimator:
    """Copy of the pre-factory RealWorld estimator construction."""
    estimator_window_size = int(
        planner_config.get("state_estimator_window_size", 31)
    )
    return BallStateEstimator(
        window_size=estimator_window_size,
        min_samples=int(
            planner_config.get(
                "state_estimator_min_samples",
                estimator_window_size,
            )
        ),
        table_height=float(planner_config.get("table_height", 0.76)),
        table_center_xy=planner_config.get(
            "table_center_xy_w",
            [1.37, 0.0],
        ),
        table_length=float(planner_config.get("table_length", 2.74)),
        table_width=float(planner_config.get("table_width", 1.525)),
        ball_radius=float(planner_config.get("ball_radius", 0.02)),
        bounce_height_tolerance=float(
            planner_config.get(
                "state_estimator_bounce_height_tolerance",
                0.03,
            )
        ),
        bounce_velocity_threshold=float(
            planner_config.get(
                "state_estimator_bounce_velocity_threshold",
                0.10,
            )
        ),
        bounce_min_separation_s=float(
            planner_config.get(
                "state_estimator_bounce_min_separation_s",
                0.20,
            )
        ),
    )


def reference_hitter_system_planner(
    planner_config: dict,
) -> HitterSystemPlanner:
    """Copy of the pre-factory HitterEnv planner construction."""
    table_center_xy = planner_config.get(
        "table_center_xy_w",
        [1.37, 0.0],
    )
    predictor = BallTrajectoryPredictor(
        gravity=planner_config.get("gravity", [0.0, 0.0, -9.81]),
        drag_coefficient=float(
            planner_config.get("drag_coefficient", 0.0)
        ),
        vertical_restitution=float(
            planner_config.get("vertical_restitution", 0.8)
        ),
        horizontal_restitution=float(
            planner_config.get("horizontal_restitution", 0.9)
        ),
        dt=float(planner_config.get("prediction_dt", 0.005)),
        table_height=float(planner_config.get("table_height", 0.76)),
        table_center_xy=table_center_xy,
        table_length=float(planner_config.get("table_length", 2.74)),
        table_width=float(planner_config.get("table_width", 1.525)),
        ball_radius=float(planner_config.get("ball_radius", 0.02)),
    )
    table_height = float(planner_config.get("table_height", 0.76))
    target_base_height_w = float(
        planner_config.get("target_base_height_w", 0.793)
    )
    racket_target_z_range_b_m = planner_config.get(
        "racket_target_z_range_b_m",
        None,
    )
    if racket_target_z_range_b_m is None:
        minimum_hit_height = table_height
        maximum_hit_height = round(
            table_height
            + float(
                planner_config.get(
                    "maximum_hit_height_above_table_m",
                    0.69,
                )
            ),
            12,
        )
    else:
        minimum_z_b, maximum_z_b = racket_target_z_range_b_m
        minimum_hit_height = round(
            target_base_height_w + float(minimum_z_b),
            12,
        )
        maximum_hit_height = round(
            target_base_height_w + float(maximum_z_b),
            12,
        )
    desired_landing_point = planner_config.get(
        "desired_landing_point_w",
        [2.05, 0.0, 0.78],
    )
    strike_planner = StrikePlanner(
        predictor=predictor,
        virtual_hit_plane_x=float(
            planner_config.get("virtual_hit_plane_x", 0.0)
        ),
        desired_landing_point=desired_landing_point,
        forehand_desired_landing_point=planner_config.get(
            "forehand_desired_landing_point_w",
            desired_landing_point,
        ),
        backhand_desired_landing_point=planner_config.get(
            "backhand_desired_landing_point_w",
            desired_landing_point,
        ),
        post_hit_flight_time=float(
            planner_config.get("post_hit_flight_time", 0.55)
        ),
        racket_restitution=float(
            planner_config.get("racket_restitution", 0.85)
        ),
        prediction_horizon_s=float(
            planner_config.get("prediction_horizon_s", 2.0)
        ),
        maximum_prediction_horizon_s=float(
            planner_config.get("maximum_prediction_horizon_s", 5.0)
        ),
        minimum_hit_height=minimum_hit_height,
        maximum_hit_height=maximum_hit_height,
        require_future_hit_plane_crossing=bool(
            planner_config.get(
                "require_future_hit_plane_crossing",
                False,
            )
        ),
        racket_velocity_component_ranges_mps=planner_config.get(
            "racket_velocity_component_ranges_mps",
            None,
        ),
        backhand_edge_landing_start_y_w_m=planner_config.get(
            "backhand_edge_landing_start_y_w_m",
            0.30,
        ),
        backhand_edge_landing_full_y_w_m=planner_config.get(
            "backhand_edge_landing_full_y_w_m",
            0.50,
        ),
        backhand_edge_landing_y_decrement_m=planner_config.get(
            "backhand_edge_landing_y_decrement_m",
            0.0,
        ),
        backhand_edge_landing_threshold_y_w_m=planner_config.get(
            "backhand_edge_landing_threshold_y_w_m",
            0.20,
        ),
        backhand_edge_landing_target_y_w_m=planner_config.get(
            "backhand_edge_landing_target_y_w_m",
            None,
        ),
    )
    base_planner = BaseTargetPlanner(
        racket_x_offset_b=float(
            planner_config.get("racket_x_offset_b", 0.40)
        ),
        forehand_nominal_racket_y_b=float(
            planner_config.get(
                "forehand_nominal_racket_y_b",
                -0.5,
            )
        ),
        backhand_nominal_racket_y_b=float(
            planner_config.get(
                "backhand_nominal_racket_y_b",
                0.22,
            )
        ),
        default_base_z=target_base_height_w,
    )
    return HitterSystemPlanner(
        strike_planner=strike_planner,
        base_planner=base_planner,
    )


def assert_estimator_equal(
    testcase: unittest.TestCase,
    actual: BallStateEstimator,
    expected: BallStateEstimator,
) -> None:
    testcase.assertEqual(actual.window_size, expected.window_size)
    testcase.assertEqual(actual.min_samples, expected.min_samples)
    testcase.assertEqual(actual.table_height, expected.table_height)
    np.testing.assert_array_equal(
        actual.table_center_xy,
        expected.table_center_xy,
    )
    testcase.assertEqual(actual.table_length, expected.table_length)
    testcase.assertEqual(actual.table_width, expected.table_width)
    testcase.assertEqual(actual.ball_radius, expected.ball_radius)
    testcase.assertEqual(
        actual.bounce_height_tolerance,
        expected.bounce_height_tolerance,
    )
    testcase.assertEqual(
        actual.bounce_velocity_threshold,
        expected.bounce_velocity_threshold,
    )
    testcase.assertEqual(
        actual.bounce_min_separation_s,
        expected.bounce_min_separation_s,
    )


def assert_system_planner_equal(
    testcase: unittest.TestCase,
    actual: HitterSystemPlanner,
    expected: HitterSystemPlanner,
) -> None:
    actual_strike = actual.strike_planner
    expected_strike = expected.strike_planner
    actual_predictor = actual_strike.predictor
    expected_predictor = expected_strike.predictor

    np.testing.assert_array_equal(
        actual_predictor.gravity,
        expected_predictor.gravity,
    )
    for name in (
        "drag_coefficient",
        "vertical_restitution",
        "horizontal_restitution",
        "dt",
        "table_height",
        "table_length",
        "table_width",
        "ball_radius",
    ):
        testcase.assertEqual(
            getattr(actual_predictor, name),
            getattr(expected_predictor, name),
        )
    np.testing.assert_array_equal(
        actual_predictor.table_center_xy,
        expected_predictor.table_center_xy,
    )

    for name in (
        "virtual_hit_plane_x",
        "post_hit_flight_time",
        "racket_restitution",
        "prediction_horizon_s",
        "maximum_prediction_horizon_s",
        "minimum_hit_height",
        "maximum_hit_height",
        "require_future_hit_plane_crossing",
        "backhand_edge_landing_start_y_w_m",
        "backhand_edge_landing_full_y_w_m",
        "backhand_edge_landing_y_decrement_m",
        "backhand_edge_landing_threshold_y_w_m",
        "backhand_edge_landing_target_y_w_m",
    ):
        testcase.assertEqual(
            getattr(actual_strike, name),
            getattr(expected_strike, name),
        )
    np.testing.assert_array_equal(
        actual_strike.desired_landing_point,
        expected_strike.desired_landing_point,
    )
    np.testing.assert_array_equal(
        actual_strike.forehand_desired_landing_point,
        expected_strike.forehand_desired_landing_point,
    )
    np.testing.assert_array_equal(
        actual_strike.backhand_desired_landing_point,
        expected_strike.backhand_desired_landing_point,
    )
    testcase.assertEqual(
        actual_strike.racket_velocity_component_ranges_mps,
        expected_strike.racket_velocity_component_ranges_mps,
    )

    actual_base = actual.base_planner
    expected_base = expected.base_planner
    for name in (
        "racket_x_offset_b",
        "forehand_nominal_racket_y_b",
        "backhand_nominal_racket_y_b",
        "default_base_z",
    ):
        testcase.assertEqual(
            getattr(actual_base, name),
            getattr(expected_base, name),
        )


def armed_recovery_duration(lifecycle, sequence_number: int) -> float:
    result = success(
        track_id=sequence_number,
        generation=1,
        deadline=10.0 + lifecycle.arm_tts,
        completed=10.0,
    )
    decision = lifecycle.ingest(result, now=10.0)
    if decision.kind != "armed":
        raise AssertionError(f"expected lifecycle to arm, got {decision!r}")
    return lifecycle.recovery_duration_s


class RuntimeFactoryCharacterizationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        mimic_config = load_yaml_mapping("config/mimic/hitter.yaml")
        cls.policy_config = mimic_config["policy"]
        cls.motion_config = mimic_config["motion"]
        cls.planner_config = cls.motion_config["ball_planner"]
        cls.control_config = load_yaml_mapping(
            "config/control/g1_hitter_racket.yaml"
        )

    def test_yaml_factories_match_pre_factory_production_builders(self):
        assert_estimator_equal(
            self,
            build_ball_state_estimator(self.planner_config),
            reference_ball_state_estimator(self.planner_config),
        )
        assert_system_planner_equal(
            self,
            build_hitter_system_planner(self.planner_config),
            reference_hitter_system_planner(self.planner_config),
        )

    def test_yaml_runtime_settings_resolve_current_production_values(self):
        settings = resolve_hitter_runtime_settings(
            policy_config=self.policy_config,
            motion_config=self.motion_config,
            control_config=self.control_config,
        )

        self.assertEqual(settings.estimator_sample_rate_hz, 360.0)
        self.assertEqual(settings.planner_update_rate_hz, 100.0)
        self.assertEqual(settings.planner_update_interval_s, 0.01)
        self.assertEqual(settings.minimum_incoming_speed_x_mps, 0.20)
        self.assertEqual(settings.incoming_confirmation_snapshots, 3)
        self.assertEqual(settings.waiting_tts_s, 0.92)
        self.assertEqual(settings.arm_tts_s, 0.92)
        self.assertEqual(settings.minimum_arm_tts_s, 0.30)
        self.assertEqual(settings.maximum_policy_tts_s, 0.92)
        self.assertEqual(settings.completed_result_queue_capacity, 64)
        self.assertEqual(settings.armed_cancel_consecutive_failures, 3)
        self.assertEqual(settings.commit_time_to_strike_s, 0.30)
        self.assertEqual(
            settings.maximum_racket_target_override_delta_m,
            0.05,
        )
        self.assertEqual(
            settings.maximum_racket_velocity_override_delta_mps,
            0.75,
        )
        self.assertEqual(
            settings.maximum_strike_deadline_override_delta_s,
            0.05,
        )
        self.assertEqual(
            settings.swing_duration_range_s,
            (1.75, 1.95),
        )
        self.assertEqual(settings.hitter_seed, 0)
        self.assertEqual(settings.control_tick_s, 0.02)
        self.assertEqual(settings.obs_clip_value, 1000.0)

    def test_yaml_planner_disables_direct_backhand_landing_override(self):
        self.assertNotIn(
            "backhand_edge_landing_threshold_y_w_m",
            self.planner_config,
        )
        self.assertNotIn(
            "backhand_edge_landing_target_y_w_m",
            self.planner_config,
        )
        planner = build_hitter_system_planner(self.planner_config).strike_planner
        self.assertIsNone(planner.backhand_edge_landing_target_y_w_m)

    def test_yaml_planner_uses_side_specific_landing_targets(self):
        planner = build_hitter_system_planner(self.planner_config).strike_planner
        forehand_landing = planner.desired_landing_point_for_strike(
            [0.0, 0.50, 1.0],
            strike_type="forehand",
        )
        backhand_landing = planner.desired_landing_point_for_strike(
            [0.0, -0.50, 1.0],
            strike_type="backhand",
        )
        np.testing.assert_allclose(
            forehand_landing,
            [2.05, 0.3775, 0.78],
        )
        np.testing.assert_allclose(
            backhand_landing,
            [2.05, -0.3775, 0.78],
        )

    def test_empty_configs_preserve_all_production_defaults(self):
        assert_estimator_equal(
            self,
            build_ball_state_estimator({}),
            reference_ball_state_estimator({}),
        )
        assert_system_planner_equal(
            self,
            build_hitter_system_planner({}),
            reference_hitter_system_planner({}),
        )
        settings = resolve_hitter_runtime_settings(
            policy_config={},
            motion_config={},
            control_config={},
        )

        self.assertEqual(
            settings,
            HitterRuntimeSettings(
                estimator_sample_rate_hz=300.0,
                planner_update_rate_hz=50.0,
                planner_update_interval_s=0.02,
                minimum_incoming_speed_x_mps=0.20,
                incoming_confirmation_snapshots=3,
                waiting_tts_s=0.92,
                arm_tts_s=0.92,
                minimum_arm_tts_s=0.30,
                maximum_policy_tts_s=0.92,
                completed_result_queue_capacity=64,
                armed_cancel_consecutive_failures=3,
                commit_time_to_strike_s=0.30,
                maximum_racket_target_override_delta_m=0.05,
                maximum_racket_velocity_override_delta_mps=0.75,
                maximum_strike_deadline_override_delta_s=0.05,
                swing_duration_range_s=(1.75, 1.95),
                hitter_seed=None,
                control_tick_s=0.02,
                obs_clip_value=None,
            ),
        )
        with self.assertRaises(FrozenInstanceError):
            settings.planner_update_rate_hz = 50.0

    def test_yaml_has_one_resolved_nested_vicon_configuration(self):
        self.assertNotIn("vicon_lcm_channel", self.motion_config)
        self.assertNotIn("vicon_stream_stale_timeout_s", self.motion_config)
        self.assertNotIn("ball_message_stale_timeout_s", self.motion_config)
        self.assertNotIn("minimum_inter_serve_no_ball_s", self.motion_config)
        self.assertNotIn("vicon_event_queue_capacity", self.motion_config)
        self.assertNotIn("vicon_consumer", self.planner_config)

        settings = resolve_vicon_consumer_settings(
            self.motion_config["vicon_consumer"]
        )

        self.assertEqual(settings.channel, "vicon_state_data_v2")
        self.assertEqual(settings.base_subject, "G2Pelvis")
        self.assertEqual(settings.stream_timeout_s, 0.40)
        self.assertEqual(settings.ball_timeout_s, 0.40)
        self.assertEqual(settings.new_serve_no_ball_s, 0.20)
        self.assertEqual(settings.event_queue_capacity, 64)

    def test_runtime_rejects_invalid_float_safety_values(self):
        planner_fields = {
            "state_estimator_sample_rate_hz": (
                float("nan"),
                float("inf"),
                0.0,
                -1.0,
                True,
                "360",
            ),
            "planner_update_rate_hz": (
                float("nan"),
                float("inf"),
                0.0,
                -1.0,
                True,
                "50",
            ),
            "minimum_stable_incoming_speed_x_mps": (
                float("nan"),
                float("inf"),
                0.0,
                -1.0,
                True,
                "0.2",
            ),
            "arm_time_to_strike_s": (
                float("nan"),
                float("inf"),
                -1.0,
                True,
                "0.92",
            ),
            "minimum_arm_time_to_strike_s": (
                float("nan"),
                float("inf"),
                -1.0,
                True,
                "0.30",
            ),
            "maximum_policy_time_to_strike_s": (
                float("nan"),
                float("inf"),
                -1.0,
                True,
                "0.92",
            ),
            "commit_time_to_strike_s": (
                float("nan"),
                float("inf"),
                -1.0,
                True,
                "0.30",
            ),
            "maximum_racket_target_override_delta_m": (
                float("nan"),
                float("inf"),
                -1.0,
                True,
                "0.05",
            ),
            "maximum_racket_velocity_override_delta_mps": (
                float("nan"),
                float("inf"),
                -1.0,
                True,
                "0.75",
            ),
            "maximum_strike_deadline_override_delta_s": (
                float("nan"),
                float("inf"),
                -1.0,
                True,
                "0.05",
            ),
        }
        for key, invalid_values in planner_fields.items():
            for invalid in invalid_values:
                with self.subTest(key=key, invalid=invalid):
                    with self.assertRaises((TypeError, ValueError)):
                        resolve_hitter_runtime_settings(
                            policy_config={},
                            motion_config={"ball_planner": {key: invalid}},
                            control_config={},
                        )

        for invalid in (
            float("nan"),
            float("inf"),
            -1.0,
            True,
            "0.92",
        ):
            with self.subTest(
                key="waiting_time_to_strike_s",
                invalid=invalid,
            ):
                with self.assertRaises((TypeError, ValueError)):
                    resolve_hitter_runtime_settings(
                        policy_config={},
                        motion_config={"waiting_time_to_strike_s": invalid},
                        control_config={},
                    )

        for key, invalid_values in {
            "low_dt": (
                float("nan"),
                float("inf"),
                0.0,
                -1.0,
                True,
                "0.005",
            ),
            "obs_clip_value": (
                float("nan"),
                float("inf"),
                0.0,
                -1.0,
                True,
                "1000",
            ),
        }.items():
            for invalid in invalid_values:
                with self.subTest(key=key, invalid=invalid):
                    with self.assertRaises((TypeError, ValueError)):
                        resolve_hitter_runtime_settings(
                            policy_config={},
                            motion_config={},
                            control_config={key: invalid},
                        )

        with self.assertRaises((TypeError, ValueError)):
            resolve_hitter_runtime_settings(
                policy_config={},
                motion_config={},
                control_config={"low_dt": 1.0e308, "decimation": 2},
            )
        with self.assertRaises(ValueError):
            resolve_hitter_runtime_settings(
                policy_config={},
                motion_config={
                    "ball_planner": {
                        "planner_update_rate_hz": np.nextafter(0.0, 1.0),
                    }
                },
                control_config={},
            )

    def test_runtime_rejects_non_exact_positive_integer_counts(self):
        for key in (
            "stable_incoming_confirmation_snapshots",
            "completed_result_queue_capacity",
            "armed_cancel_consecutive_failures",
        ):
            for invalid in (True, 1.0, "1", 0, -1):
                with self.subTest(key=key, invalid=invalid):
                    with self.assertRaises((TypeError, ValueError)):
                        resolve_hitter_runtime_settings(
                            policy_config={},
                            motion_config={"ball_planner": {key: invalid}},
                            control_config={},
                        )

        for invalid in (True, 4.0, "4", 0, -1):
            with self.subTest(key="decimation", invalid=invalid):
                with self.assertRaises((TypeError, ValueError)):
                    resolve_hitter_runtime_settings(
                        policy_config={},
                        motion_config={},
                        control_config={"decimation": invalid},
                    )

    def test_runtime_rejects_invalid_seed_and_swing_range(self):
        for invalid in (True, 1.0, "1", np.int64(1), -1):
            with self.subTest(seed=invalid):
                with self.assertRaises((TypeError, ValueError)):
                    resolve_hitter_runtime_settings(
                        policy_config={"hitter_seed": invalid},
                        motion_config={},
                        control_config={},
                    )

        invalid_ranges = (
            1.0,
            "12",
            (1.0,),
            (1.0, 2.0, 3.0),
            (-0.1, 1.0),
            (1.0, 0.5),
            (float("nan"), 1.0),
            (0.0, float("inf")),
            (True, 1.0),
        )
        for invalid in invalid_ranges:
            with self.subTest(swing_duration_range=invalid):
                with self.assertRaises((TypeError, ValueError)):
                    resolve_hitter_runtime_settings(
                        policy_config={},
                        motion_config={
                            "ball_planner": {
                                "swing_duration_range": invalid,
                            }
                        },
                        control_config={},
                    )

    def test_runtime_rejects_inconsistent_lifecycle_time_ordering(self):
        invalid_planner_configs = (
            {"commit_time_to_strike_s": 0.31},
            {"minimum_arm_time_to_strike_s": 0.93},
            {"arm_time_to_strike_s": 0.93},
            {"maximum_policy_time_to_strike_s": 0.91},
        )
        for planner_config in invalid_planner_configs:
            with self.subTest(planner_config=planner_config):
                with self.assertRaises(ValueError):
                    resolve_hitter_runtime_settings(
                        policy_config={},
                        motion_config={"ball_planner": planner_config},
                        control_config={},
                    )

        with self.assertRaises(ValueError):
            resolve_hitter_runtime_settings(
                policy_config={},
                motion_config={"waiting_time_to_strike_s": 0.93},
                control_config={},
            )

    def test_runtime_accepts_zero_for_every_nonnegative_boundary(self):
        settings = resolve_hitter_runtime_settings(
            policy_config={},
            motion_config={
                "waiting_time_to_strike_s": 0.0,
                "ball_planner": {
                    "arm_time_to_strike_s": 0.0,
                    "minimum_arm_time_to_strike_s": 0.0,
                    "maximum_policy_time_to_strike_s": 0.0,
                    "commit_time_to_strike_s": 0.0,
                    "maximum_racket_target_override_delta_m": 0.0,
                    "maximum_racket_velocity_override_delta_mps": 0.0,
                    "maximum_strike_deadline_override_delta_s": 0.0,
                    "swing_duration_range": [0.0, 0.0],
                },
            },
            control_config={},
        )

        self.assertEqual(settings.waiting_tts_s, 0.0)
        self.assertEqual(settings.commit_time_to_strike_s, 0.0)
        self.assertEqual(settings.swing_duration_range_s, (0.0, 0.0))

    def test_lifecycle_factory_injects_every_single_shot_setting(self):
        settings = resolve_hitter_runtime_settings(
            policy_config={"hitter_seed": 17},
            motion_config={
                "waiting_time_to_strike_s": 0.70,
                "ball_planner": {
                    "arm_time_to_strike_s": 0.80,
                    "minimum_arm_time_to_strike_s": 0.20,
                    "maximum_policy_time_to_strike_s": 1.00,
                    "completed_result_queue_capacity": 9,
                    "armed_cancel_consecutive_failures": 5,
                    "commit_time_to_strike_s": 0.10,
                    "maximum_racket_target_override_delta_m": 0.02,
                    "maximum_racket_velocity_override_delta_mps": 0.60,
                    "maximum_strike_deadline_override_delta_s": 0.03,
                    "swing_duration_range": [1.20, 1.40],
                },
            },
            control_config={},
        )
        rng = np.random.default_rng(settings.hitter_seed)
        expected_rng = np.random.default_rng(17)
        lifecycle = build_hitter_command_lifecycle(settings, rng=rng)

        self.assertEqual(lifecycle.waiting_tts, 0.70)
        self.assertEqual(lifecycle.arm_tts, 0.80)
        self.assertEqual(lifecycle.minimum_arm_tts, 0.20)
        self.assertEqual(lifecycle.maximum_policy_tts, 1.00)
        self.assertEqual(lifecycle.armed_cancel_consecutive_failures, 5)
        self.assertEqual(lifecycle.commit_time_to_strike_s, 0.10)
        self.assertEqual(
            lifecycle.maximum_racket_target_override_delta_m,
            0.02,
        )
        self.assertEqual(
            lifecycle.maximum_racket_velocity_override_delta_mps,
            0.60,
        )
        self.assertEqual(
            lifecycle.maximum_strike_deadline_override_delta_s,
            0.03,
        )
        self.assertAlmostEqual(
            armed_recovery_duration(lifecycle, 1),
            float(expected_rng.uniform(1.20, 1.40)) - lifecycle.arm_tts,
        )

    def test_runtime_rejects_non_mapping_configuration_sections(self):
        cases = (
            ([], {}, {}),
            ({}, {"ball_planner": []}, {}),
            ({}, {"ball_planner": "planner"}, {}),
            ({}, {"ball_planner": None}, {}),
            ({}, [], {}),
            ({}, {}, []),
        )
        for policy, motion, control in cases:
            with self.subTest(
                policy_config=policy,
                motion_config=motion,
                control_config=control,
            ):
                with self.assertRaises(TypeError):
                    resolve_hitter_runtime_settings(
                        policy_config=policy,
                        motion_config=motion,
                        control_config=control,
                    )

    def test_maximum_hit_height_uses_twelve_decimal_rounding(self):
        planner = build_hitter_system_planner(
            {
                "table_height": 0.1,
                "maximum_hit_height_above_table_m": 0.2,
            }
        )

        self.assertEqual(
            planner.strike_planner.maximum_hit_height,
            0.3,
        )

    def test_training_relative_height_range_sets_world_planner_limits(self):
        planner = build_hitter_system_planner(
            {
                "table_height": 0.76,
                "target_base_height_w": 0.78,
                "maximum_hit_height_above_table_m": 9.0,
                "racket_target_z_range_b_m": [0.0, 0.7],
            }
        )

        self.assertEqual(
            planner.strike_planner.minimum_hit_height,
            0.78,
        )
        self.assertEqual(
            planner.strike_planner.maximum_hit_height,
            1.48,
        )

    def test_yaml_hit_height_limits_match_model56600_training_range(self):
        planner = build_hitter_system_planner(self.planner_config)

        self.assertEqual(
            planner.strike_planner.minimum_hit_height,
            0.78,
        )
        self.assertEqual(
            planner.strike_planner.maximum_hit_height,
            1.48,
        )

    def test_forced_strike_type_preserves_precedence_and_normalization(self):
        cases = (
            ({}, {}, None),
            ({}, {"force_strike_type": " FOREHAND "}, "forehand"),
            ({"force_strike_type": "BACKHAND"}, {}, "backhand"),
            (
                {"force_strike_type": None},
                {"force_strike_type": "forehand"},
                None,
            ),
            ({"force_strike_type": " null "}, {}, None),
        )
        for planner_config, motion_config, expected in cases:
            with self.subTest(
                planner_config=planner_config,
                motion_config=motion_config,
            ):
                self.assertEqual(
                    forced_strike_type(
                        planner_config,
                        motion_config,
                    ),
                    expected,
                )

    def test_invalid_forced_strike_type_is_rejected(self):
        with self.assertRaisesRegex(
            ValueError,
            "force_strike_type must be forehand/backhand/None",
        ):
            forced_strike_type(
                {"force_strike_type": "smash"},
                {},
            )

    def test_real_world_rejects_forced_strike_type(self):
        with self.assertRaisesRegex(
            ValueError,
            "force_strike_type is disabled",
        ):
            forced_strike_type(
                {"force_strike_type": "forehand"},
                {},
                is_real_world=True,
            )

    def test_mujoco_may_force_strike_type(self):
        self.assertEqual(
            forced_strike_type(
                {"force_strike_type": "forehand"},
                {},
                is_real_world=False,
            ),
            "forehand",
        )

    def test_seeded_lifecycle_recovery_sequence_is_deterministic(self):
        settings = resolve_hitter_runtime_settings(
            policy_config=self.policy_config,
            motion_config=self.motion_config,
            control_config=self.control_config,
        )
        first_rng = np.random.default_rng(settings.hitter_seed)
        second_rng = np.random.default_rng(settings.hitter_seed)

        expected = []
        actual = []
        for sequence_number in range(1, 7):
            expected.append(
                armed_recovery_duration(
                    build_hitter_command_lifecycle(
                        settings,
                        rng=first_rng,
                    ),
                    sequence_number,
                )
            )
            actual.append(
                armed_recovery_duration(
                    build_hitter_command_lifecycle(
                        settings,
                        rng=second_rng,
                    ),
                    sequence_number,
                )
            )

        np.testing.assert_array_equal(actual, expected)

    def test_real_world_ball_state_defaults_stay_initialized(self):
        simulator = RealWorld.__new__(RealWorld)
        simulator.cfg = SimpleNamespace(motion=self.motion_config)

        simulator._init_ball_state()

        self.assertEqual(simulator.ball_state_estimator_sample_rate_hz, 360.0)
        assert_estimator_equal(
            self,
            simulator.ball_state_estimator,
            reference_ball_state_estimator(self.planner_config),
        )
        self.assertIsNone(simulator.ball_state_estimator_time)
        self.assertIsNone(simulator.ball_state_estimator_last_host_time)
        self.assertFalse(simulator.ball_state_estimator_ready)
        self.assertFalse(simulator.ball_state_estimator_ready_tmp)
        self.assertEqual(simulator.ball_state_estimator_sample_count, 0)
        self.assertEqual(simulator.ball_state_estimator_sample_count_tmp, 0)
        self.assertEqual(
            simulator.ball_state_estimator_min_samples,
            simulator.ball_state_estimator.min_samples,
        )


if __name__ == "__main__":
    unittest.main()
