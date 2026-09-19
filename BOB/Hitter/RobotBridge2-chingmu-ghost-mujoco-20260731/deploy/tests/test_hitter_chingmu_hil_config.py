from __future__ import annotations

import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
from hydra import compose, initialize_config_dir
from omegaconf import OmegaConf

from envs.hitter import HitterEnv
from simulator.mujoco import (
    Mujoco,
    validate_hitter_ball_input_config,
)
from utils.hitter_runtime_capabilities import (
    HitterRuntimeCapabilities,
    LoopPacing,
    PlannerFeed,
)
from utils.read_only_lcm import PublicationAudit


CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"
HYBRID_CAPABILITIES = HitterRuntimeCapabilities(
    planner_feed=PlannerFeed.SNAPSHOT_STREAM,
    loop_pacing=LoopPacing.SIMULATOR,
)


def compose_hitter(*overrides):
    with initialize_config_dir(
        config_dir=str(CONFIG_DIR),
        job_name="test_hitter_chingmu_hil_config",
        version_base=None,
    ):
        return compose(config_name="hitter", overrides=list(overrides))


def hil_input_config(**updates):
    values = {
        "mode": "chingmu_ghost",
        "robot_backend": "mujoco",
        "planner_feed": "snapshot_stream",
        "loop_pacing": "simulator",
        "lcm_url": "udpm://239.255.76.67:7667?ttl=255",
        "channel": "vicon_state_data",
        "read_only": True,
        "use_mujoco_base_pose": True,
        "stale_timeout_s": 0.10,
        "ghost_collision": False,
        "ghost_max_extrapolation_s": 0.05,
        "table_position_tolerance_m": 0.03,
        "table_angle_tolerance_deg": 3.0,
        "required_table_confirmations": 3,
        "ball_xy_margin_m": 0.50,
        "ball_min_height_offset_m": -0.30,
        "ball_max_height_offset_m": 2.50,
        "trace_queue_capacity": 4096,
        "trace_rotate_bytes": 67108864,
        "trace_retained_files": 4,
        "trace_flush_interval_s": 1.0,
        "status_log_interval_s": 1.0,
    }
    values.update(updates)
    return values


def sim_config(input_updates=None, *, real_time=True):
    return OmegaConf.create(
        {
            "control": {"real_time": real_time},
            "motion": {
                "ball_planner": {
                    "table_center_xy_w": [1.365369, 0.0],
                    "table_height": 0.76,
                    "table_length": 2.730738,
                    "table_width": 1.512451,
                },
                "mujoco_serve_times_s": [0.0, 20.0],
            },
            "hitter_ball_input": hil_input_config(
                **(input_updates or {})
            ),
        }
    )


class HitterChingMuHilConfigTests(unittest.TestCase):
    def test_default_mujoco_config_keeps_direct_ball_input_and_checkpoint(self):
        cfg = compose_hitter()

        self.assertEqual(
            cfg.sim.config.hitter_ball_input.mode,
            "mujoco_direct",
        )
        self.assertEqual(
            cfg.mimic.policy.checkpoint,
            "./data/model/hitter/hitter_model8000_oldmotion_104_20260725.onnx",
        )
        self.assertFalse("mode" in cfg.mimic.motion)

    def test_hil_config_is_single_authoritative_chingmu_ghost_mode(self):
        cfg = compose_hitter("sim=mujoco_chingmu_ghost")
        ball_input = cfg.sim.config.hitter_ball_input

        self.assertEqual(cfg.sim._target_, "simulator.mujoco.Mujoco")
        self.assertEqual(ball_input.mode, "chingmu_ghost")
        self.assertEqual(ball_input.robot_backend, "mujoco")
        self.assertEqual(ball_input.planner_feed, "snapshot_stream")
        self.assertEqual(ball_input.loop_pacing, "simulator")
        self.assertEqual(ball_input.channel, "vicon_state_data")
        self.assertTrue(ball_input.read_only)
        self.assertTrue(ball_input.use_mujoco_base_pose)
        self.assertFalse(ball_input.ghost_collision)
        self.assertEqual(ball_input.required_table_confirmations, 3)
        self.assertFalse("mode" in cfg.mimic.motion)
        self.assertEqual(
            cfg.mimic.policy.checkpoint,
            "./data/model/hitter/hitter_model8000_oldmotion_104_20260725.onnx",
        )

    def test_hil_validation_rejects_unsafe_configuration(self):
        cases = (
            ("real_world backend", {"robot_backend": "real_world"}),
            ("not read only", {"read_only": False}),
            ("real base pose", {"use_mujoco_base_pose": False}),
            ("ghost collision", {"ghost_collision": True}),
            ("empty channel", {"channel": ""}),
            ("negative xy margin", {"ball_xy_margin_m": -0.01}),
            (
                "inverted height bounds",
                {
                    "ball_min_height_offset_m": 1.0,
                    "ball_max_height_offset_m": 0.0,
                },
            ),
        )

        for label, updates in cases:
            with self.subTest(label=label):
                with self.assertRaisesRegex(ValueError, "chingmu_ghost"):
                    validate_hitter_ball_input_config(
                        sim_config(updates)
                    )

        with self.assertRaisesRegex(ValueError, "real_time"):
            validate_hitter_ball_input_config(
                sim_config(real_time=False)
            )

    def test_snapshot_env_disables_internal_mujoco_serve_and_reset(self):
        env = HitterEnv.__new__(HitterEnv)
        env.simulator = SimpleNamespace(
            hitter_runtime_capabilities=HYBRID_CAPABILITIES,
            preserve_hitter_ball_state_on_calibrate=True,
        )
        env.motion_cfg = {"mujoco_serve_times_s": [0.0, 20.0]}
        env.reset_ball_on_command = True

        env._init_mujoco_serve_schedule_state()

        self.assertEqual(env._mujoco_serve_times_s, ())
        self.assertFalse(env.reset_ball_on_command)
        self.assertFalse(
            env.simulator.preserve_hitter_ball_state_on_calibrate
        )
        self.assertFalse(env._mujoco_serve_schedule_enabled())

    def test_hil_init_creates_started_chingmu_source(self):
        cfg = sim_config()
        sim = Mujoco.__new__(Mujoco)
        sim.cfg = cfg
        sim._hitter_lcm_publication_audit = PublicationAudit()
        sim.copy_hitter_base_pose_w = lambda: None
        sim.queue_hitter_ghost_snapshot = lambda _snapshot: None
        sim.set_hitter_ghost_mode = lambda enabled: setattr(
            sim,
            "ghost_enabled",
            bool(enabled),
        )

        created = {}

        class FakePipeline:
            def __init__(self, planner_cfg, base_pose_provider, *, stale_timeout_s):
                created["pipeline"] = {
                    "planner_cfg": dict(planner_cfg),
                    "base_pose_provider": base_pose_provider,
                    "stale_timeout_s": stale_timeout_s,
                }

        class FakeSource:
            def __init__(self, settings, **kwargs):
                created["source"] = self
                self.settings = settings
                self.kwargs = kwargs
                self.started = False
                self.listener = None

            def register_hitter_ball_listener(self, listener):
                self.listener = listener
                return lambda: None

            def start(self):
                self.started = True

        with patch("simulator.mujoco.RealtimeViconBallPipeline", FakePipeline):
            with patch("simulator.mujoco.ChingMuGhostBallSource", FakeSource):
                sim._init_hitter_ball_input()

        self.assertTrue(sim.ghost_enabled)
        self.assertIs(sim.hitter_ball_source, created["source"])
        self.assertTrue(sim.hitter_ball_source.started)
        self.assertEqual(
            sim.hitter_ball_source.settings.channel,
            "vicon_state_data",
        )
        self.assertEqual(
            sim.hitter_ball_source.settings.required_table_confirmations,
            3,
        )
        self.assertIs(
            sim.hitter_ball_source.kwargs["publication_audit"],
            sim._hitter_lcm_publication_audit,
        )
        self.assertIsNotNone(sim._hitter_ghost_unregister_listener)


if __name__ == "__main__":
    unittest.main()
