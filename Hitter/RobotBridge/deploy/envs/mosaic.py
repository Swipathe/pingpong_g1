from __future__ import annotations
from typing import Dict, List, Optional, Union

import numpy as np
from loguru import logger
from omegaconf import DictConfig, OmegaConf
from scipy.spatial.transform import Rotation as sRot

from envs.base_env import BaseEnv

from utils.dataset import MosaicModelMeta, MotionDataset
from utils.dof import DoFAdapter
from utils.hitter_planner import (
    BallTrajectoryPredictor,
    BaseTargetPlanner,
    HitterSystemPlanner,
    StrikePlanner,
)
from utils.transformation import matrix_from_quat, subtract_frame_transforms, quat_rotate_inverse

from utils.data_pub import DataPublisher

import collections

info_pub = DataPublisher()
import os
from pathlib import Path

if "MUJOCO_GL" not in os.environ and not os.environ.get("DISPLAY"):
    os.environ["MUJOCO_GL"] = "egl"

import imageio
import mujoco

class MosaicEnv(BaseEnv):
    """在当前框架上模拟 RoboJuDo Mosaic 流水线的环境封装。"""

    def __init__(self, config: DictConfig):
        super().__init__(config)

        cfg_dict = OmegaConf.to_container(config, resolve=True) if isinstance(config, DictConfig) else config
        self.policy_cfg: Dict[str, Union[float, bool]] = cfg_dict.get("policy", {}) if isinstance(cfg_dict, dict) else {}
        self.motion_cfg: Dict[str, Union[float, bool, str]] = (
            cfg_dict.get("motion", {}) if isinstance(cfg_dict, dict) else {}
        )

        self.hitter_mode = bool(self.policy_cfg.get("hitter_mode", False))
        self.motion_loader: Optional[MotionDataset] = None
        if not self.hitter_mode:
            self.motion_loader = MotionDataset(self.motion_cfg, self.simulator)

        self.loop_motion = bool(self.motion_cfg.get("loop", False))
        self.playback_speed = float(self.motion_cfg.get("playback_speed", 1.0))
        self.max_timestep = int(self.policy_cfg.get("max_timestep", -1))
        self.without_state_estimator = bool(self.policy_cfg.get("without_state_estimator", True))
        self.action_beta = float(self.policy_cfg.get("action_beta", 1.0))
        self.hitter_rng = np.random.default_rng(self.policy_cfg.get("hitter_seed", None))

        self.policy_model_meta: Optional[MosaicModelMeta] = None
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
        self.motion_finished: bool = False
        self.total_policy_steps: int = 0

        self._pending_alignment_reset: bool = True

        self.last_ref_dof_pos = np.zeros(self.num_action, dtype=np.float64)

        self.history_length = int(self.policy_cfg.get("history_length", 1))
        self.obs_command_buffer = collections.deque([np.zeros(58) for _ in range(self.history_length)], maxlen=self.history_length)
        self.obs_motion_anchor_ori_b_buffer = collections.deque([np.zeros(6) for _ in range(self.history_length)], maxlen=self.history_length)
        self.obs_base_ang_vel_buffer = collections.deque([np.zeros(3) for _ in range(self.history_length)], maxlen=self.history_length)
        self.obs_joint_pos_rel_buffer = collections.deque([np.zeros(29) for _ in range(self.history_length)], maxlen=self.history_length)
        self.obs_joint_vel_rel_buffer = collections.deque([np.zeros(29) for _ in range(self.history_length)], maxlen=self.history_length)
        self.obs_prev_policy_action_buffer = collections.deque([np.zeros(29) for _ in range(self.history_length)], maxlen=self.history_length)
        self.obs_projected_gravity_buffer = collections.deque([np.zeros(3) for _ in range(self.history_length)], maxlen=self.history_length)
        
        self.gravity_w = np.zeros(3)
        self.gravity_w[2] = -1.0
        self.use_estimator = self.policy_cfg.get("use_estimator", False)

        self.hard_reset = False
        self.use_ball_planner = bool(self.motion_cfg.get("use_ball_planner", False))
        self.reset_ball_on_command = bool(self.motion_cfg.get("reset_ball_on_command", True))
        self.hitter_ball_planner: Optional[HitterSystemPlanner] = (
            self._build_hitter_ball_planner() if self.hitter_mode and self.use_ball_planner else None
        )
        self._init_hitter_command_state()

        # 评估
        self.eval_mode = self.policy_cfg.get("eval_mode", False)
        ckpt_name = Path(self.policy_cfg.get("checkpoint")).stem 
        csv_name = f"metrics_{ckpt_name}.csv"
        log_dir = "logs" 
        os.makedirs(log_dir, exist_ok=True)
        self.metrics_path = os.path.join(log_dir, csv_name)
        
        if self.motion_loader is not None:
            self.motion_loader.set_metrics_file(self.metrics_path)
            logger.info(f"Metrics will be saved to: {self.metrics_path}")

        # 保存视频
        self.init_video()

    def init_video(self):
        self.save_video_enabled = bool(self.policy_cfg.get("save_video", False))
        self.video_fps = int(self.policy_cfg.get("video_fps", 30))
        self.video_width = int(self.policy_cfg.get("video_width", 640))
        self.video_height = int(self.policy_cfg.get("video_height", 480))
        self.dt = self.simulator.high_dt
        self.render_every = max(1, int(round(1 / (self.video_fps * self.dt))))
        self.video_step_counter = 0
        self.video_episode_counter = 0
        self.frames = []
        self.offscreen_renderer = None
        self.video_camera = None
        self.video_dir = Path(self.policy_cfg.get("video_dir", "videos_offline")).expanduser()

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
        self.video_camera.lookat[:] = np.array(self.policy_cfg.get("video_camera_lookat", [0.0, 0.0, 0.8]), dtype=np.float64)
        self.video_camera.distance = float(self.policy_cfg.get("video_camera_distance", 2.5))
        self.video_camera.azimuth = float(self.policy_cfg.get("video_camera_azimuth", 180.0))
        self.video_camera.elevation = float(self.policy_cfg.get("video_camera_elevation", -10.0))
        logger.info(
            "Offscreen video capture enabled: {}x{} @ {} fps -> {}",
            self.video_width,
            self.video_height,
            self.video_fps,
            self.video_dir,
        )

    # --------------------------------------------------------------------- #
    # agent 调用的公开 API
    # --------------------------------------------------------------------- #
    def configure_from_modelmeta(self, model_meta: MosaicModelMeta) -> None:
        """应用从 Mosaic ONNX 模型解析出的关节配置。"""
        logger.info("Applying joint configuration from ONNX model.")
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

        logger.info("Mosaic policy metadata configured, {} degrees of freedom.", self._policy_dim)

    # --------------------------------------------------------------------- #
    # 覆盖基类实现
    # --------------------------------------------------------------------- #
    def reset(self):
        if self.policy_model_meta is None:
            raise RuntimeError("MosaicEnv.reset() called before policy metadata was configured.")
        if self.motion_loader is not None:
            self.motion_loader.reset()
        self.motion_finished = False
        self.total_policy_steps = 0
        self.playback_speed = 1.0
        self.prev_policy_action = np.zeros(self._policy_dim, dtype=np.float32)
        if self.hitter_mode:
            self._sample_hitter_command()

        super().reset()
        if self.motion_loader is not None and not self.eval_mode:
            self._interpolate_to_motion_start()
        self.compute_observation()

        if self.save_video_enabled:
            self.check_save_video()

        return self.obs_buf_dict

    def step(self, action):
        if self.policy_model_meta is None:
            raise RuntimeError("MosaicEnv.step() called before policy metadata was configured.")

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

    def _get_command(self):
        if self.motion_loader is None:
            raise RuntimeError("Mosaic command data requested without a motion loader.")
        command_data = self.motion_loader.get_data()
        return (
            command_data["command"],
            command_data["robot_anchor_pos_w"],
            command_data["robot_anchor_quat_w"],
            command_data["anchor_pos_w"],
            command_data["anchor_quat_w"],
        )
    
    # def _get_reference_markers_world(self) -> Optional[np.ndarray]:
    #     """在仿真机器人坐标系中构建用于可视化的参考 marker（Nx3）。"""
    #     command_data = self.motion_loader.get_data()
    #     body_pos_aligned = command_data.get("body_pos_w_aligned", None)
    #     if body_pos_aligned is None:
    #         return None
    #     body_pos_aligned = np.asarray(body_pos_aligned, dtype=np.float32).reshape(-1, 3)
    #     if body_pos_aligned.size == 0:
    #         return None

    #     # 先把参考 body 转成 anchor-relative 点云，避免 marker “漂在空中”。
    #     # body_pos_aligned 与 anchor_pos_w 位于同一个对齐坐标系中。
    #     ref_anchor_pos_aligned = np.asarray(command_data.get("anchor_pos_w", np.zeros(3)), dtype=np.float32).reshape(-1)[:3]
    #     body_rel = body_pos_aligned - ref_anchor_pos_aligned.reshape(1, 3)

    #     # 只使用 yaw，把点云放到当前机器人 anchor 位姿（torso/pelvis）附近。
    #     robot_anchor_pos_w = np.asarray(command_data.get("robot_anchor_pos_w", self.simulator.root_trans_world), dtype=np.float32).reshape(-1)[:3]
    #     robot_anchor_quat_w = np.asarray(command_data.get("robot_anchor_quat_w", self.simulator.root_quat_world), dtype=np.float32).reshape(-1)[:4]  # xyzw
    #     try:
    #         yaw = float(sRot.from_quat(robot_anchor_quat_w).as_euler("xyz", degrees=False)[2])
    #     except Exception:
    #         yaw = 0.0
            
    #     yaw_rot = sRot.from_euler("z", yaw, degrees=False)
    #     markers_world = yaw_rot.apply(body_rel) + robot_anchor_pos_w.reshape(1, 3)
    #     return markers_world.astype(np.float32)

    def _get_reference_markers_world(self) -> Optional[np.ndarray]:
        """在仿真世界坐标系中构建用于可视化的参考 marker（Nx3）。"""
        if self.motion_loader is None:
            return None
        command_data = self.motion_loader.get_data()
        body_pos_aligned = command_data.get("body_pos_w_aligned", None)
        if body_pos_aligned is None:
            return None
        body_pos_aligned = np.asarray(body_pos_aligned, dtype=np.float32).reshape(-1, 3)

        if body_pos_aligned.size == 0:
            return None

        return body_pos_aligned.astype(np.float32)
    
    def compute_observation(self):
        if self.hitter_mode:
            return self._compute_hitter_observation()

        BaseEnv._update_obs(self)

        command, robot_anchor_pos_w, robot_anchor_quat_w, anchor_pos_w, anchor_quat_w = self._get_command()
            
        pos, ori = subtract_frame_transforms(
            np.asarray(robot_anchor_pos_w, dtype=np.float32),
            np.asarray(robot_anchor_quat_w, dtype=np.float32),
            np.asarray(anchor_pos_w, dtype=np.float32),
            np.asarray(anchor_quat_w, dtype=np.float32),
        )
        if self.eval_mode:
            self.motion_loader._update_metrics()

        obs_ref_project_gravity = quat_rotate_inverse(np.roll(anchor_quat_w, 1), self.gravity_w)

        mat = matrix_from_quat(ori)

        obs_command = command
        obs_motion_anchor_pos_b = pos
        obs_motion_anchor_ori_b = mat[:, :2].flatten()

        obs_base_lin_vel = self.base_lin_vel.squeeze()
        obs_base_ang_vel = self.base_ang_vel.squeeze()
        obs_joint_pos_rel = self._sim_vector_to_policy(self.dof_pos.squeeze()) - self.policy_default_joint_pos
        obs_joint_vel_rel = self._sim_vector_to_policy(self.dof_vel.squeeze())
        obs_prev_policy_action = self.prev_policy_action

        self.obs_command_buffer.append(obs_command)
        self.obs_motion_anchor_ori_b_buffer.append(obs_motion_anchor_ori_b)
        self.obs_base_ang_vel_buffer.append(obs_base_ang_vel)
        self.obs_joint_pos_rel_buffer.append(obs_joint_pos_rel)
        self.obs_joint_vel_rel_buffer.append(obs_joint_vel_rel)
        self.obs_prev_policy_action_buffer.append(obs_prev_policy_action)

        obs_prop = np.concatenate([
            np.array(self.obs_command_buffer).reshape(1, -1),
            np.array(self.obs_motion_anchor_ori_b_buffer).reshape(1, -1),
            np.array(self.obs_base_ang_vel_buffer).reshape(1, -1),
            np.array(self.obs_joint_pos_rel_buffer).reshape(1, -1),
            np.array(self.obs_joint_vel_rel_buffer).reshape(1, -1),
            np.array(self.obs_prev_policy_action_buffer).reshape(1, -1),
        ], axis=1)

        if self.use_estimator:
            self.obs_projected_gravity_buffer.append(obs_ref_project_gravity)
            obs_estimator = np.concatenate([
                np.array(self.obs_command_buffer).reshape(1, -1),
                np.array(self.obs_projected_gravity_buffer).reshape(1, -1),
            ], axis=1)
            obs_prop = np.concatenate([obs_prop, obs_estimator], axis=1)

        self.obs_buf_dict = {
            "obs": obs_prop,
        }

    # --------------------------------------------------------------------- #
    # HITTER 指令条件观测分支
    # --------------------------------------------------------------------- #
    def _init_hitter_command_state(self) -> None:
        self.hitter_command_initialized = False
        self.hitter_strike_type = 0
        self.hitter_strike_elapsed_s = 0.0
        self.hitter_strike_time_s = 0.86
        self.hitter_strike_duration_s = 1.85
        self.hitter_base_target_xy_w = np.zeros(2, dtype=np.float32)
        self.hitter_racket_target_pos_w_fixed = np.zeros(3, dtype=np.float32)
        self.hitter_racket_target_vel_w = np.zeros(3, dtype=np.float32)

    def _motion_range(self, name: str, default: tuple[float, float]) -> tuple[float, float]:
        value = self.motion_cfg.get(name, default)
        if len(value) != 2:
            raise ValueError(f"HITTER motion range `{name}` must contain two values, got {value!r}.")
        low, high = float(value[0]), float(value[1])
        if high < low:
            raise ValueError(f"HITTER motion range `{name}` has high < low: {value!r}.")
        return low, high

    def _sample_motion_range(self, name: str, default: tuple[float, float]) -> float:
        low, high = self._motion_range(name, default)
        return float(self.hitter_rng.uniform(low, high))

    def _motion_vector(self, name: str, default: list[float], size: int) -> np.ndarray:
        value = self.motion_cfg.get(name, default)
        arr = np.asarray(value, dtype=np.float32).reshape(-1)
        if arr.size != size:
            raise ValueError(f"HITTER motion vector `{name}` must have {size} values, got {value!r}.")
        return arr

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
            table_width=float(planner_cfg.get("table_width", self.motion_cfg.get("physical_table_width", 1.525))),
            ball_radius=float(planner_cfg.get("ball_radius", 0.02)),
        )
        strike_planner = StrikePlanner(
            predictor=predictor,
            virtual_hit_plane_x=float(planner_cfg.get("virtual_hit_plane_x", self.motion_cfg.get("virtual_hit_plane_x", 0.0))),
            desired_landing_point=planner_cfg.get("desired_landing_point_w", [2.05, 0.0, 0.78]),
            post_hit_flight_time=float(planner_cfg.get("post_hit_flight_time", 0.55)),
            racket_restitution=float(planner_cfg.get("racket_restitution", 0.85)),
            prediction_horizon_s=float(planner_cfg.get("prediction_horizon_s", 2.0)),
        )
        racket_x_offset_b = planner_cfg.get(
            "racket_x_offset_b",
            planner_cfg.get("strike_plane_x", self.motion_cfg.get("racket_x_offset_b", self.motion_cfg.get("strike_plane_x", 0.40))),
        )
        base_planner = BaseTargetPlanner(
            racket_x_offset_b=float(racket_x_offset_b),
            forehand_nominal_racket_y_b=float(
                planner_cfg.get("forehand_nominal_racket_y_b", self.motion_cfg.get("forehand_nominal_racket_y_b", -0.5))
            ),
            backhand_nominal_racket_y_b=float(
                planner_cfg.get("backhand_nominal_racket_y_b", self.motion_cfg.get("backhand_nominal_racket_y_b", 0.22))
            ),
            default_base_z=float(planner_cfg.get("target_base_height_w", self.motion_cfg.get("target_base_height_w", 0.793))),
        )
        return HitterSystemPlanner(strike_planner=strike_planner, base_planner=base_planner)

    def _forced_strike_type_name(self) -> Optional[str]:
        forced = self.motion_cfg.get("force_strike_type", None)
        if forced is None or str(forced).strip().lower() in {"", "none", "null"}:
            return None
        forced = str(forced).strip().lower()
        if forced not in {"forehand", "backhand"}:
            raise ValueError(f"force_strike_type must be forehand/backhand/None, got {forced!r}.")
        return forced

    def _sample_hitter_command_from_ball_planner(self) -> bool:
        if self.hitter_ball_planner is None:
            return False
        if not hasattr(self.simulator, "ball_pos_world") or not hasattr(self.simulator, "ball_vel_world"):
            logger.warning("use_ball_planner=True but simulator does not expose ball state; falling back to random command.")
            return False

        if self.reset_ball_on_command and hasattr(self.simulator, "reset_hitter_ball"):
            self.simulator.reset_hitter_ball()
        self.simulator.get_state()

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
            logger.warning(f"Ball planner failed ({exc}); falling back to random command.")
            return False

        self.hitter_strike_type = 0 if command.strike_type == "forehand" else 1
        self.hitter_strike_elapsed_s = 0.0
        self.hitter_strike_time_s = max(float(command.time_to_strike), 0.0)
        sampled_duration = self._sample_motion_range("swing_duration_range", (1.75, 1.95))
        self.hitter_strike_duration_s = max(sampled_duration, self.hitter_strike_time_s + 0.5)
        self.hitter_base_target_xy_w = command.p_base_target_xy.astype(np.float32)
        self.hitter_racket_target_pos_w_fixed = command.strike_plan.p_racket_target.astype(np.float32)
        self.hitter_racket_target_vel_w = self._clip_planner_racket_velocity(
            command.v_racket_target_w,
            command.strike_type,
        ).astype(np.float32)
        self.hitter_command_initialized = True

        logger.debug(
            "Planned HITTER command from ball: type={}, ball_pos={}, ball_vel={}, base_xy_w={}, racket_pos_w={}, racket_vel_w={}, tts={:.3f}s",
            command.strike_type,
            ball_pos_w.astype(np.float32),
            ball_vel_w.astype(np.float32),
            self.hitter_base_target_xy_w,
            self.hitter_racket_target_pos_w_fixed,
            self.hitter_racket_target_vel_w,
            self.hitter_strike_time_s,
        )
        return True

    def _clip_planner_racket_velocity(self, velocity_w: np.ndarray, strike_type: str) -> np.ndarray:
        planner_cfg = self.motion_cfg.get("ball_planner", {}) or {}
        if not bool(planner_cfg.get("clip_racket_velocity_to_training_range", True)):
            return np.asarray(velocity_w, dtype=np.float64)

        velocity_w = np.asarray(velocity_w, dtype=np.float64).copy()
        if strike_type == "forehand":
            x_range = self.motion_cfg.get("forehand_racket_velocity_x_range", [2.0, 3.2])
            y_range = self.motion_cfg.get("forehand_racket_velocity_y_range", [0.0, 0.0])
        else:
            x_range = self.motion_cfg.get("backhand_racket_velocity_x_range", [2.0, 3.2])
            y_range = self.motion_cfg.get("backhand_racket_velocity_y_range", [0.0, 0.0])
        z_range = self.motion_cfg.get("racket_velocity_z_range", [0.0, 0.0])

        velocity_w[0] = np.clip(velocity_w[0], float(x_range[0]), float(x_range[1]))
        velocity_w[1] = np.clip(velocity_w[1], float(y_range[0]), float(y_range[1]))
        velocity_w[2] = np.clip(velocity_w[2], float(z_range[0]), float(z_range[1]))
        return velocity_w

    def _sample_hitter_strike_type(self) -> int:
        forced = self.motion_cfg.get("force_strike_type", None)
        if forced is not None and str(forced).strip().lower() not in {"", "none", "null"}:
            forced = str(forced).strip().lower()
            if forced not in {"forehand", "backhand"}:
                raise ValueError(f"force_strike_type must be forehand/backhand/None, got {forced!r}.")
            return 0 if forced == "forehand" else 1

        ratios = self.motion_cfg.get(
            "strike_type_sampling_ratios",
            self.motion_cfg.get("motion_group_sampling_ratios", {"forehand": 0.5, "backhand": 0.5}),
        )
        forehand_weight = float(ratios.get("forehand", 0.5)) if isinstance(ratios, dict) else 0.5
        backhand_weight = float(ratios.get("backhand", 0.5)) if isinstance(ratios, dict) else 0.5
        total = max(forehand_weight + backhand_weight, 1.0e-6)
        return 0 if float(self.hitter_rng.random()) < forehand_weight / total else 1

    def _sample_hitter_command(self) -> None:
        if self.use_ball_planner and self._sample_hitter_command_from_ball_planner():
            return

        table_half_width = 0.5 * float(self.motion_cfg.get("physical_table_width", 1.525))
        forehand_nominal_y = float(self.motion_cfg.get("forehand_nominal_racket_y_b", -0.5))
        backhand_nominal_y = float(self.motion_cfg.get("backhand_nominal_racket_y_b", 0.22))

        self.hitter_strike_type = self._sample_hitter_strike_type()
        is_forehand = self.hitter_strike_type == 0
        strike_name = "forehand" if is_forehand else "backhand"
        self.hitter_strike_elapsed_s = 0.0
        self.hitter_strike_time_s = self._sample_motion_range("time_to_strike_range", (0.80, 0.92))
        self.hitter_strike_duration_s = self._sample_motion_range("swing_duration_range", (1.75, 1.95))

        if is_forehand:
            desired_y = self._sample_motion_range(
                "forehand_racket_y_range",
                (-table_half_width, 0.0),
            )
            nominal_y = forehand_nominal_y
            vx = self._sample_motion_range("forehand_racket_velocity_x_range", (2.0, 3.2))
            vy = self._sample_motion_range("forehand_racket_velocity_y_range", (0.0, 0.0))
        else:
            desired_y = self._sample_motion_range(
                "backhand_racket_y_range",
                (0.0, table_half_width),
            )
            nominal_y = backhand_nominal_y
            vx = self._sample_motion_range("backhand_racket_velocity_x_range", (2.0, 3.2))
            vy = self._sample_motion_range("backhand_racket_velocity_y_range", (0.0, 0.0))

        origin_xy_w = self._motion_vector("base_target_origin_xy_w", [0.0, 0.0], 2)
        base_x = float(self.motion_cfg.get("base_target_x_offset", 0.0))
        base_y = float(desired_y - nominal_y)
        self.hitter_base_target_xy_w = origin_xy_w + np.asarray([base_x, base_y], dtype=np.float32)

        racket_x_offset_b = float(self.motion_cfg.get("racket_x_offset_b", self.motion_cfg.get("strike_plane_x", 0.40)))
        target_base_height_w = float(self.motion_cfg.get("target_base_height_w", 0.793))
        z = self._sample_motion_range("racket_z_range", (0.00, 0.50))
        self.hitter_racket_target_pos_w_fixed = np.asarray(
            [self.hitter_base_target_xy_w[0] + racket_x_offset_b, origin_xy_w[1] + desired_y, target_base_height_w + z],
            dtype=np.float32,
        )

        vz = self._sample_motion_range("racket_velocity_z_range", (0.0, 0.0))
        self.hitter_racket_target_vel_w = np.asarray([vx, vy, vz], dtype=np.float32)
        self.hitter_command_initialized = True

        logger.debug(
            "Sampled HITTER command: type={}, base_xy_w={}, racket_pos_w={}, racket_vel_w={}, tts={:.3f}s",
            strike_name,
            self.hitter_base_target_xy_w,
            self.hitter_racket_target_pos_w_fixed,
            self.hitter_racket_target_vel_w,
            self.hitter_strike_time_s,
        )

    def _update_hitter_command(self) -> None:
        if not self.hitter_command_initialized:
            self._sample_hitter_command()
            return
        if self.playback_speed <= 0.0:
            return
        self.hitter_strike_elapsed_s += float(self.simulator.high_dt) * float(self.playback_speed)
        if self.hitter_strike_elapsed_s >= self.hitter_strike_duration_s:
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

    def _hitter_base_target_xy_b(self, robot_anchor_pos_w: np.ndarray, robot_anchor_quat_w: np.ndarray) -> np.ndarray:
        delta_xy_w = self.hitter_base_target_xy_w - robot_anchor_pos_w[:2]
        delta_w = np.asarray([delta_xy_w[0], delta_xy_w[1], 0.0], dtype=np.float32)
        return self._yaw_inverse_apply(robot_anchor_quat_w, delta_w)[:2]

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
        obs_base_target_xy = self._hitter_base_target_xy_b(robot_anchor_pos_w, robot_anchor_quat_w)
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
                obs_base_target_xy,
                obs_racket_target_pos,
                obs_racket_target_vel,
                obs_time_to_strike,
                obs_joint_pos_rel,
                obs_joint_vel_rel,
                obs_prev_policy_action,
            ],
            axis=0,
        ).astype(np.float32)

        if obs.size != 104:
            raise RuntimeError(f"HITTER policy expects 104 observation values, assembled {obs.size}.")

        self.obs_buf_dict = {"obs": obs.reshape(1, -1)}

    def _post_physics_step(self):
        if self.hitter_mode:
            self.episode_length_buf += 1
            self.total_policy_steps += 1
            self._update_hitter_command()
            self.time_step += 1 * self.playback_speed
            if self.max_timestep > 0 and self.total_policy_steps >= self.max_timestep:
                self.motion_finished = True
                self.playback_speed = 0.0
            self.compute_observation()
            self._check_termination()
            return

        super()._post_physics_step()
        self.motion_loader.post_step_callback()
        self.time_step += 1 * self.playback_speed
        if self.max_timestep > 0 and int(self.episode_length_buf[0]) >= self.max_timestep:
            self.motion_finished = True
            self.playback_speed = 0.0

    def _physics_step(self):
        super()._physics_step()
        # 如果 simulator 支持，就更新可视化 marker。
        if getattr(self.simulator, "marker", False):
            markers_world = self._get_reference_markers_world()
            if markers_world is not None:
                self.simulator.update_marker_pos(markers_world[None, ...])

    # --------------------------------------------------------------------- #
    # 辅助函数
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

    @staticmethod
    def _wxyz_to_xyzw(quat_wxyz: np.ndarray) -> np.ndarray:
        return np.asarray([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]], dtype=np.float32)

    # ------------------------------------------------------------------ #
    # 插值辅助函数
    # ------------------------------------------------------------------ #
    def _interpolate_to_motion_start(self):
        """动作开始前，将 policy 关节插值到动作首帧。"""
        if self.motion_loader is None:
            return
        steps = int(getattr(self.motion_loader, "interp_steps", 0))
        if steps <= 0:
            return

        target_policy = self.motion_loader.joint_pos  # motion 首帧（policy 关节顺序）
        target_sim = self._policy_vector_to_sim(target_policy)

        # 当前仿真关节角
        current_sim = np.asarray(self.simulator.dof_pos).squeeze().astype(np.float32)

        # 线性插值并推进物理仿真
        for i in range(steps):
            alpha = float(i + 1) / float(steps)
            blended = (1.0 - alpha) * current_sim + alpha * target_sim
            self.simulator.apply_action(blended[None, ...])

        # 更新内部状态缓存
        self.simulator.get_state()

    def next_motion(self, fail: bool = False):
        if self.motion_loader is None:
            return self.reset()
        self.check_save_video()
        self.motion_loader.next_motion(fail)
        return self.reset()
    
    def _check_termination(self):
        self.hard_reset = self.simulator.check_termination()
        if self.hard_reset:
            if self.eval_mode and self.motion_loader is not None:
                self.next_motion(fail=True)
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
