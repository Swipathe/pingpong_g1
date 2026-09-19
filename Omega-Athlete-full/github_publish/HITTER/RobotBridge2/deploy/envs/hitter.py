from __future__ import annotations
from typing import Dict, List, Optional, Union

import numpy as np
from loguru import logger
from omegaconf import DictConfig, OmegaConf
from scipy.spatial.transform import Rotation as sRot

from envs.base_env import BaseEnv

from utils.dataset import MosaicModelMeta as HitterModelMeta
from utils.dof import DoFAdapter
from utils.hitter_planner import (
    BallTrajectoryPredictor,
    BaseTargetPlanner,
    HitterSystemPlanner,
    StrikePlanner,
)
from utils.transformation import matrix_from_quat

import time

import os
from pathlib import Path

if "MUJOCO_GL" not in os.environ and not os.environ.get("DISPLAY"):
    os.environ["MUJOCO_GL"] = "egl"

import imageio
import mujoco

class HitterEnv(BaseEnv):
    """Environment wrapper for the HITTER target-conditioned policy."""

    def __init__(self, config: DictConfig):
        super().__init__(config)

        cfg_dict = OmegaConf.to_container(config, resolve=True) if isinstance(config, DictConfig) else config
        self.policy_cfg: Dict[str, Union[float, bool]] = cfg_dict.get("policy", {}) if isinstance(cfg_dict, dict) else {}
        self.motion_cfg: Dict[str, Union[float, bool, str]] = (
            cfg_dict.get("motion", {}) if isinstance(cfg_dict, dict) else {}
        )

        self.playback_speed = float(self.motion_cfg.get("playback_speed", 1.0))
        self.action_beta = float(self.policy_cfg.get("action_beta", 1.0))
        self.hitter_rng = np.random.default_rng(self.policy_cfg.get("hitter_seed", None))

        self.policy_model_meta: Optional[HitterModelMeta] = None
        self.policy_action_scales: Optional[np.ndarray] = None
        self.policy_default_joint_pos: Optional[np.ndarray] = None
        self.policy_joint_names: Optional[List[str]] = None
        self.policy_joint_index: Optional[Dict[str, int]] = None

        self._sim_to_policy: Optional[np.ndarray] = None
        self._policy_to_sim: Optional[np.ndarray] = None
        self._policy_dim: Optional[int] = None
        self._policy_to_sim_adapter: Optional[DoFAdapter] = None
        self._sim_to_policy_adapter: Optional[DoFAdapter] = None

        self.prev_policy_action: Optional[np.ndarray] = None
        self.time_step: float = 0.0
        self._last_ball_estimator_not_ready_log_s: float = 0.0
        self._last_ball_planner_error_log_s: float = 0.0

        self.hard_reset = False
        self.reset_ball_on_command = bool(self.motion_cfg.get("reset_ball_on_command", True))
        self.hitter_ball_planner: HitterSystemPlanner = self._build_hitter_ball_planner()
        self._init_hitter_command_state()
        logger.info("HITTER ball planner configured.")

        # Video capture.
        self.init_video()

    def init_video(self):
        self.save_video_enabled = bool(self.policy_cfg.get("save_video", False))
        self.video_fps = 30
        self.video_width = 640
        self.video_height = 480
        self.dt = self.simulator.high_dt
        self.render_every = max(1, int(round(1 / (self.video_fps * self.dt))))
        self.video_step_counter = 0
        self.video_episode_counter = 0
        self.frames = []
        self.offscreen_renderer = None
        self.video_camera = None
        self.video_dir = Path("videos_offline")

        if not self.save_video_enabled:
            return

        if not hasattr(self.simulator, "mujoco_model"):
            logger.warning("save_video=True requires a MuJoCo simulator; video capture is disabled.")
            self.save_video_enabled = False
            return

        self.video_dir.mkdir(parents=True, exist_ok=True)
        self.offscreen_renderer = mujoco.Renderer(
            self.simulator.mujoco_model,
            height=self.video_height,
            width=self.video_width,
        )
        self.video_camera = mujoco.MjvCamera()
        self.video_camera.lookat[:] = np.array([0.0, 0.0, 0.8], dtype=np.float64)
        self.video_camera.distance = 2.5
        self.video_camera.azimuth = 180.0
        self.video_camera.elevation = -10.0
        logger.info(
            "Offscreen video capture enabled: {}x{} @ {} fps -> {}",
            self.video_width,
            self.video_height,
            self.video_fps,
            self.video_dir,
        )

    # --------------------------------------------------------------------- #
    # Public API called by the agent.
    # --------------------------------------------------------------------- #
    def configure_from_modelmeta(self, model_meta: HitterModelMeta) -> None:
        """Apply joint configuration parsed from the HITTER ONNX model."""
        logger.info("Applying HITTER joint configuration from ONNX model.")
        self.policy_model_meta = model_meta
        self.policy_joint_names = model_meta.joint_names
        self.policy_joint_index = model_meta.joint_index_map()
        self.policy_default_joint_pos = model_meta.default_joint_pos.astype(np.float32)
        self.policy_action_scales = model_meta.action_scale.astype(np.float32)
        self._policy_dim = len(self.policy_joint_names)

        self.prev_policy_action = np.zeros(self._policy_dim, dtype=np.float32)

        sim_joint_names = list(self.simulator.dof_names)
        if len(sim_joint_names) != self._policy_dim:
            raise RuntimeError(
                f"the number of policy joints ({self._policy_dim}) does not match the number of simulation joints ({len(sim_joint_names)}), please check the robot configuration."
            )

        reordered = model_meta.to_joint_order(sim_joint_names)
        sim_default = reordered["default_joint_pos"].astype(np.float32)
        sim_kps = reordered["joint_stiffness"].astype(np.float32)
        sim_kds = reordered["joint_damping"].astype(np.float32)

        self.simulator.default_angles = sim_default
        self.simulator.kps = sim_kps
        self.simulator.kds = sim_kds

        if hasattr(self.simulator.cfg, "asset"):
            asset_cfg = self.simulator.cfg.asset
            asset_cfg.default_angles = sim_default.tolist()
            asset_cfg.kps = sim_kps.tolist()
            asset_cfg.kds = sim_kds.tolist()

        sim_to_policy = np.array([self.policy_joint_index[name] for name in sim_joint_names], dtype=np.int32)
        policy_to_sim = np.zeros_like(sim_to_policy)
        policy_to_sim[sim_to_policy] = np.arange(self._policy_dim, dtype=np.int32)

        self._sim_to_policy = sim_to_policy
        self._policy_to_sim = policy_to_sim
        self.policy_default_joint_pos_sim = sim_default
        self.policy_action_scales_sim = self.policy_action_scales[sim_to_policy]
        self._policy_to_sim_adapter = DoFAdapter(self.policy_joint_names, sim_joint_names)
        self._sim_to_policy_adapter = DoFAdapter(sim_joint_names, self.policy_joint_names)

        logger.info("HITTER policy metadata configured, {} degrees of freedom.", self._policy_dim)

    # --------------------------------------------------------------------- #
    # BaseEnv overrides.
    # --------------------------------------------------------------------- #
    def reset(self):
        if self.policy_model_meta is None:
            raise RuntimeError("HitterEnv.reset() called before policy metadata was configured.")
        self.playback_speed = 1.0
        self.prev_policy_action = np.zeros(self._policy_dim, dtype=np.float32)
        self._init_hitter_command_state()

        obs_buf_dict = super().reset()

        if self.save_video_enabled:
            self.check_save_video()

        return obs_buf_dict

    def step(self, action):
        if self.policy_model_meta is None:
            raise RuntimeError("HitterEnv.step() called before policy metadata was configured.")

        action = np.asarray(action, dtype=np.float32).reshape(-1)
        if action.size != self._policy_dim:
            raise ValueError(f"Expected policy action of size {self._policy_dim}, received {action.size}.")

        smoothed = (1.0 - self.action_beta) * self.prev_policy_action + self.action_beta * action
        self.prev_policy_action = smoothed
        scaled = smoothed * self.policy_action_scales
        pd_target = scaled + self.policy_default_joint_pos
        sim_action = self._policy_vector_to_sim(pd_target)

        if self.save_video_enabled:
            self.sample_video_frame()

        return super().step(sim_action[None, ...])

    def compute_observation(self):
        return self._compute_hitter_observation()

    # --------------------------------------------------------------------- #
    # HITTER command-conditioned observation branch.
    # --------------------------------------------------------------------- #
    def _init_hitter_command_state(self) -> None:
        self.hitter_command_initialized = False
        self.hitter_ball_sequence_needs_reset = True
        self.hitter_strike_type = 0
        self.hitter_strike_elapsed_s = 0.0
        self.hitter_strike_time_s = 0.86
        self.hitter_strike_duration_s = 1.85
        self.hitter_base_target_xy_w = np.zeros(2, dtype=np.float32)
        self.hitter_base_target_z_w = self._hitter_target_base_height_w()
        self.hitter_racket_target_pos_w_fixed = np.zeros(3, dtype=np.float32)
        self.hitter_racket_target_vel_w = np.zeros(3, dtype=np.float32)
        self.hitter_ball_out_vel_w = np.zeros(3, dtype=np.float32)

    def _range_from_value(self, name: str, value) -> tuple[float, float]:
        if len(value) != 2:
            raise ValueError(f"HITTER motion range `{name}` must contain two values, got {value!r}.")
        low, high = float(value[0]), float(value[1])
        if high < low:
            raise ValueError(f"HITTER motion range `{name}` has high < low: {value!r}.")
        return low, high

    def _sample_range_from_cfg(self, cfg: dict, name: str, default: tuple[float, float]) -> float:
        low, high = self._range_from_value(name, cfg.get(name, default))
        return float(self.hitter_rng.uniform(low, high))

    def _hitter_target_base_height_w(self) -> np.float32:
        planner_cfg = self.motion_cfg.get("ball_planner", {}) or {}
        return np.float32(planner_cfg.get("target_base_height_w", 0.793))

    def _build_hitter_ball_planner(self) -> HitterSystemPlanner:
        planner_cfg = self.motion_cfg.get("ball_planner", {})
        if planner_cfg is None:
            planner_cfg = {}

        table_center_xy = planner_cfg.get("table_center_xy_w", [1.37, 0.0])
        predictor = BallTrajectoryPredictor(
            gravity=planner_cfg.get("gravity", [0.0, 0.0, -9.81]),
            drag_coefficient=float(planner_cfg.get("drag_coefficient", 0.0)),
            vertical_restitution=float(planner_cfg.get("vertical_restitution", 0.8)),
            horizontal_restitution=float(planner_cfg.get("horizontal_restitution", 0.9)),
            dt=float(planner_cfg.get("prediction_dt", 0.005)),
            table_height=float(planner_cfg.get("table_height", 0.76)),
            table_center_xy=table_center_xy,
            table_length=float(planner_cfg.get("table_length", 2.74)),
            table_width=float(planner_cfg.get("table_width", 1.525)),
            ball_radius=float(planner_cfg.get("ball_radius", 0.02)),
        )
        strike_planner = StrikePlanner(
            predictor=predictor,
            virtual_hit_plane_x=float(planner_cfg.get("virtual_hit_plane_x", 0.0)),
            desired_landing_point=planner_cfg.get("desired_landing_point_w", [2.05, 0.0, 0.78]),
            post_hit_flight_time=float(planner_cfg.get("post_hit_flight_time", 0.55)),
            racket_restitution=float(planner_cfg.get("racket_restitution", 0.85)),
            prediction_horizon_s=float(planner_cfg.get("prediction_horizon_s", 2.0)),
            require_future_hit_plane_crossing=bool(planner_cfg.get("require_future_hit_plane_crossing", False)),
        )
        racket_x_offset_b = planner_cfg.get(
            "racket_x_offset_b",
            0.40,
        )
        base_planner = BaseTargetPlanner(
            racket_x_offset_b=float(racket_x_offset_b),
            forehand_nominal_racket_y_b=float(
                planner_cfg.get("forehand_nominal_racket_y_b", -0.5)
            ),
            backhand_nominal_racket_y_b=float(
                planner_cfg.get("backhand_nominal_racket_y_b", 0.22)
            ),
            default_base_z=float(planner_cfg.get("target_base_height_w", 0.793)),
        )
        return HitterSystemPlanner(strike_planner=strike_planner, base_planner=base_planner)

    def _forced_strike_type_name(self) -> Optional[str]:
        planner_cfg = self.motion_cfg.get("ball_planner", {}) or {}
        forced = planner_cfg.get("force_strike_type", self.motion_cfg.get("force_strike_type", None))
        if forced is None or str(forced).strip().lower() in {"", "none", "null"}:
            return None
        forced = str(forced).strip().lower()
        if forced not in {"forehand", "backhand"}:
            raise ValueError(f"force_strike_type must be forehand/backhand/None, got {forced!r}.")
        return forced

    def _reset_hitter_ball_sequence_if_needed(self) -> None:
        if not self.hitter_ball_sequence_needs_reset:
            return
        if self.reset_ball_on_command and hasattr(self.simulator, "reset_hitter_ball"):
            self.simulator.reset_hitter_ball()
        self.hitter_ball_sequence_needs_reset = False

    def _plan_hitter_command_from_current_ball(self, *, strict: bool = True):
        if not hasattr(self.simulator, "ball_pos_world") or not hasattr(self.simulator, "ball_vel_world"):
            message = "HITTER simulator does not expose ball state."
            if strict:
                raise RuntimeError(message)
            logger.debug(message)
            return None

        self.simulator.get_state()
        if self._ball_estimator_not_ready():
            self._log_ball_estimator_not_ready()
            if strict:
                raise RuntimeError("Ball estimator is not ready before planning HITTER command.")
            return None

        ball_pos_w = np.asarray(self.simulator.ball_pos_world, dtype=np.float64).reshape(-1)[:3]
        ball_vel_w = np.asarray(self.simulator.ball_vel_world, dtype=np.float64).reshape(-1)[:3]
        robot_anchor_pos_w, robot_anchor_quat_w = self._hitter_robot_anchor_pose_w()
        base_forward_xy_w = matrix_from_quat(robot_anchor_quat_w)[:, 0][:2]

        try:
            command = self.hitter_ball_planner.plan_command(
                ball_pos_w,
                ball_vel_w,
                current_base_xy_w=robot_anchor_pos_w,
                base_forward_xy_w=base_forward_xy_w,
                strike_type=self._forced_strike_type_name(),
            )
        except Exception as exc:
            self._log_ball_planner_error(exc, ball_pos_w, ball_vel_w)
            if strict:
                raise RuntimeError(f"Ball planner failed: {exc}") from exc
            else:
                logger.debug(f"Ball planner update skipped after arm ({exc}).")
            return None

        return command

    def _log_ball_estimator_not_ready(self) -> None:
        now = time.time()
        if now - self._last_ball_estimator_not_ready_log_s < 0.5:
            return
        self._last_ball_estimator_not_ready_log_s = now
        logger.info(
            "Ball estimator is not ready: samples={}/{} visible={} ready={} pos={} vel={}",
            getattr(self.simulator, "ball_state_estimator_sample_count", 0),
            getattr(self.simulator, "ball_state_estimator_min_samples", 0),
            getattr(self.simulator, "ball_visible", False),
            getattr(self.simulator, "ball_state_estimator_ready", False),
            np.asarray(getattr(self.simulator, "ball_pos_world", np.zeros(3)), dtype=np.float32),
            np.asarray(getattr(self.simulator, "ball_vel_world", np.zeros(3)), dtype=np.float32),
        )

    def _log_ball_planner_error(self, exc: Exception, ball_pos_w: np.ndarray, ball_vel_w: np.ndarray) -> None:
        now = time.time()
        if now - self._last_ball_planner_error_log_s < 0.5:
            return
        self._last_ball_planner_error_log_s = now
        logger.info(
            "Ball planner failed to produce a command: reason={} ball_pos_w={} ball_vel_w={} hit_plane_x={}",
            exc,
            ball_pos_w.astype(np.float32),
            ball_vel_w.astype(np.float32),
            float(self.hitter_ball_planner.strike_planner.virtual_hit_plane_x),
        )

    def _plan_hitter_command_from_ball(self, *, strict: bool = True):
        self._reset_hitter_ball_sequence_if_needed()
        return self._plan_hitter_command_from_current_ball(strict=strict)

    def _ball_estimator_not_ready(self) -> bool:
        if not hasattr(self.simulator, "ball_state_estimator_ready"):
            return False
        visible = bool(getattr(self.simulator, "ball_visible", False))
        ready = bool(getattr(self.simulator, "ball_state_estimator_ready", False))
        return not (visible and ready)

    def _apply_hitter_ball_planner_command(self, command, *, reset_elapsed: bool) -> None:
        self.hitter_strike_type = 0 if command.strike_type == "forehand" else 1
        time_to_strike = max(float(command.time_to_strike), 0.0)
        if reset_elapsed:
            self.hitter_strike_elapsed_s = 0.0
            self.hitter_strike_time_s = time_to_strike
            planner_cfg = self.motion_cfg.get("ball_planner", {}) or {}
            sampled_duration = self._sample_range_from_cfg(
                planner_cfg,
                "swing_duration_range",
                tuple(self.motion_cfg.get("swing_duration_range", (1.75, 1.95))),
            )
            self.hitter_strike_duration_s = max(sampled_duration, self.hitter_strike_time_s + 0.5)
        else:
            self.hitter_strike_time_s = self.hitter_strike_elapsed_s + time_to_strike
            self.hitter_strike_duration_s = max(self.hitter_strike_duration_s, self.hitter_strike_time_s + 0.5)
        self.hitter_base_target_xy_w = command.p_base_target_xy.astype(np.float32)
        self.hitter_base_target_z_w = self._hitter_target_base_height_w()
        self.hitter_racket_target_pos_w_fixed = command.strike_plan.p_racket_target.astype(np.float32)
        self.hitter_racket_target_vel_w = np.asarray(command.v_racket_target_w, dtype=np.float32)
        self.hitter_ball_out_vel_w = np.asarray(command.strike_plan.v_ball_out, dtype=np.float32)
        if hasattr(self.simulator, "set_hitter_analytic_racket_hit"):
            self.simulator.set_hitter_analytic_racket_hit(
                self.hitter_racket_target_pos_w_fixed,
                self.hitter_ball_out_vel_w,
            )
        self.hitter_command_initialized = True

        if reset_elapsed:
            logger.info(
                "Armed HITTER command from ball: type={}, ball_in_vel={}, ball_out_vel={}, base_pos_w={}, racket_pos_w={}, racket_vel_w={}, tts={:.3f}s",
                command.strike_type,
                np.asarray(command.strike_plan.v_ball_in, dtype=np.float32),
                np.asarray(command.strike_plan.v_ball_out, dtype=np.float32),
                np.asarray([self.hitter_base_target_xy_w[0], self.hitter_base_target_xy_w[1], self.hitter_base_target_z_w], dtype=np.float32),
                self.hitter_racket_target_pos_w_fixed,
                self.hitter_racket_target_vel_w,
                self.hitter_strike_time_s,
            )

    def _sample_hitter_command(self) -> None:
        command = self._plan_hitter_command_from_ball(strict=True)
        if command is None:
            raise RuntimeError("Ball planner did not return a HITTER command.")

        self._apply_hitter_ball_planner_command(command, reset_elapsed=True)

    def _update_hitter_command(self) -> None:
        if not self.hitter_command_initialized:
            self._sample_hitter_command()
            return
        if self.playback_speed <= 0.0:
            return
        self.hitter_strike_elapsed_s += float(self.simulator.high_dt) * float(self.playback_speed)
        command = self._plan_hitter_command_from_ball(strict=False)
        if command is not None:
            self._apply_hitter_ball_planner_command(command, reset_elapsed=False)
        if self.hitter_strike_elapsed_s >= self.hitter_strike_duration_s:
            self.hitter_command_initialized = False
            self.hitter_ball_sequence_needs_reset = True
            self._sample_hitter_command()

    def _hitter_robot_anchor_pose_w(self) -> tuple[np.ndarray, np.ndarray]:
        pos = np.asarray(self.simulator.root_trans_world, dtype=np.float32).reshape(-1)[:3]
        quat = np.asarray(self.simulator.root_quat_world, dtype=np.float32).reshape(-1)[:4]
        if np.linalg.norm(quat) < 1.0e-6:
            quat = np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        else:
            quat = quat / np.linalg.norm(quat)
        return pos, quat.astype(np.float32)

    @staticmethod
    def _yaw_inverse_apply(quat_xyzw: np.ndarray, vector_w: np.ndarray) -> np.ndarray:
        yaw = float(sRot.from_quat(np.asarray(quat_xyzw, dtype=np.float64)).as_euler("xyz", degrees=False)[2])
        return sRot.from_euler("z", -yaw, degrees=False).apply(vector_w).astype(np.float32)

    def _hitter_base_target_pos_b(self, robot_anchor_pos_w: np.ndarray, robot_anchor_quat_w: np.ndarray) -> np.ndarray:
        delta_xy_w = self.hitter_base_target_xy_w - robot_anchor_pos_w[:2]
        delta_w = np.asarray(
            [delta_xy_w[0], delta_xy_w[1], float(self.hitter_base_target_z_w) - float(robot_anchor_pos_w[2])],
            dtype=np.float32,
        )
        return self._yaw_inverse_apply(robot_anchor_quat_w, delta_w)

    def _hitter_racket_target_pos_b(self, robot_anchor_pos_w: np.ndarray, robot_anchor_quat_w: np.ndarray) -> np.ndarray:
        delta_w = self.hitter_racket_target_pos_w_fixed - robot_anchor_pos_w
        return self._yaw_inverse_apply(robot_anchor_quat_w, delta_w)

    def _compute_hitter_observation(self):
        if self.policy_default_joint_pos is None:
            raise RuntimeError("HITTER observation requested before policy metadata was configured.")
        if not self.hitter_command_initialized:
            self._sample_hitter_command()

        BaseEnv._update_obs(self)

        robot_anchor_pos_w, robot_anchor_quat_w = self._hitter_robot_anchor_pose_w()

        obs_base_ang_vel = np.asarray(self.base_ang_vel, dtype=np.float32).reshape(-1)[:3]
        obs_projected_gravity = np.asarray(self.projected_gravity, dtype=np.float32).reshape(-1)[:3]
        obs_base_forward_xy = matrix_from_quat(robot_anchor_quat_w)[:, 0][:2].astype(np.float32)
        obs_base_target_pos = self._hitter_base_target_pos_b(robot_anchor_pos_w, robot_anchor_quat_w)
        obs_racket_target_pos = self._hitter_racket_target_pos_b(robot_anchor_pos_w, robot_anchor_quat_w)
        obs_racket_target_vel = self.hitter_racket_target_vel_w.astype(np.float32)
        obs_time_to_strike = np.asarray(
            [max(self.hitter_strike_time_s - self.hitter_strike_elapsed_s, 0.0)],
            dtype=np.float32,
        )
        obs_joint_pos_rel = self._sim_vector_to_policy(self.dof_pos.squeeze()) - self.policy_default_joint_pos
        obs_joint_vel_rel = self._sim_vector_to_policy(self.dof_vel.squeeze())
        obs_prev_policy_action = self.prev_policy_action

        obs = np.concatenate(
            [
                obs_base_ang_vel,
                obs_projected_gravity,
                obs_base_forward_xy,
                obs_base_target_pos,
                obs_racket_target_pos,
                obs_racket_target_vel,
                obs_time_to_strike,
                obs_joint_pos_rel,
                obs_joint_vel_rel,
                obs_prev_policy_action,
            ],
            axis=0,
        ).astype(np.float32)

        if obs.size != 105:
            raise RuntimeError(f"HITTER policy expects 105 observation values, assembled {obs.size}.")

        clip_obs_limit = getattr(self.cfg.control, "obs_clip_value", None)
        if clip_obs_limit is not None and float(clip_obs_limit) > 0.0:
            obs = np.clip(obs, -float(clip_obs_limit), float(clip_obs_limit))

        self.obs_buf_dict = {"obs": obs.reshape(1, -1)}

    def _post_physics_step(self):
        self.episode_length_buf += 1
        self._update_hitter_command()
        self.time_step += 1 * self.playback_speed
        self.compute_observation()
        self._check_termination()

    def _physics_step(self):
        super()._physics_step()

    # --------------------------------------------------------------------- #
    # Helper functions.
    # --------------------------------------------------------------------- #
    def _policy_vector_to_sim(self, policy_vector: np.ndarray) -> np.ndarray:
        if self._policy_to_sim_adapter is not None:
            template = np.zeros_like(self.policy_default_joint_pos_sim, dtype=np.float32)
            return self._policy_to_sim_adapter.fit(policy_vector, template=template)
        return policy_vector[self._sim_to_policy].astype(np.float32)

    def _sim_vector_to_policy(self, sim_vector: np.ndarray) -> np.ndarray:
        if self._sim_to_policy_adapter is not None:
            template = np.zeros(self._policy_dim, dtype=np.float32)
            return self._sim_to_policy_adapter.fit(sim_vector, template=template)
        result = np.zeros(self._policy_dim, dtype=np.float32)
        result[self._sim_to_policy] = sim_vector.astype(np.float32)
        return result

    def _check_termination(self):
        self.hard_reset = self.simulator.check_termination()
        if self.hard_reset:
            self._save_collected_traj(False)
            self._reset_envs(True)
            self.compute_observation()
        self.hard_reset = False

    def check_save_video(self):
        if not self.save_video_enabled:
            return
        if len(self.frames) > 0 and not self.hard_reset:
            self.save_video()

        self.frames = []
        self.video_step_counter = 0
        self.video_episode_counter += 1

    def save_video(self):
        filename = self.video_dir / f"episode_{self.video_episode_counter:04d}.mp4"
        logger.info(f"Saving video: {filename}")
        imageio.mimsave(filename, self.frames, fps=self.video_fps)
        self.frames = []

    def sample_video_frame(self):
        if self.offscreen_renderer is None or self.video_camera is None:
            return

        if self.video_step_counter % self.render_every == 0:
            self.offscreen_renderer.update_scene(self.simulator.mujoco_data, camera=self.video_camera)
            pixels = self.offscreen_renderer.render()
            self.frames.append(pixels)

        self.video_step_counter += 1
