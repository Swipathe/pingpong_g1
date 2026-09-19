from __future__ import annotations
from typing import Dict, List, Optional, Union

import numpy as np
from loguru import logger
from omegaconf import DictConfig, OmegaConf
from scipy.spatial.transform import Rotation as sRot

from envs.base_env import BaseEnv

from utils.dataset import MosaicModelMeta as HitterModelMeta
from utils.dof import DoFAdapter
from utils.hitter_planner import HitterSystemPlanner
from utils.hitter_runtime_factory import (
    build_hitter_command_lifecycle,
    build_hitter_system_planner,
    forced_strike_type,
    resolve_hitter_runtime_settings,
)
from utils.hitter_realtime import (
    BallEstimateSnapshot,
    CommandPhase,
    HitterCommandLifecycle,
    IncomingTrackConfirmation,
    LatestOnlyPlannerWorker,
    PlannerResultSnapshot,
)
from utils.hitter_task_observation import (
    assemble_active_hitter_task_observation,
)
from utils.transformation import matrix_from_quat

import time

import os
from pathlib import Path

if "MUJOCO_GL" not in os.environ and not os.environ.get("DISPLAY"):
    os.environ["MUJOCO_GL"] = "egl"

import imageio
import mujoco


TRACK_ENDED_ERROR = "HITTER_TRACK_ENDED"
TRACK_ENDED_RESULT_ERROR = f"RuntimeError: {TRACK_ENDED_ERROR}"

class HitterEnv(BaseEnv):
    """Environment wrapper for the HITTER target-conditioned policy."""

    def __init__(self, config: DictConfig):
        super().__init__(config)

        cfg_dict = OmegaConf.to_container(config, resolve=True) if isinstance(config, DictConfig) else config
        self.policy_cfg: Dict[str, Union[float, bool]] = cfg_dict.get("policy", {}) if isinstance(cfg_dict, dict) else {}
        self.motion_cfg: Dict[str, Union[float, bool, str]] = (
            cfg_dict.get("motion", {}) if isinstance(cfg_dict, dict) else {}
        )
        control_cfg = (
            cfg_dict.get("control", {})
            if isinstance(cfg_dict, dict)
            else {}
        )
        self.hitter_runtime_settings = resolve_hitter_runtime_settings(
            policy_config=self.policy_cfg,
            motion_config=self.motion_cfg,
            control_config=control_cfg,
        )

        self.playback_speed = float(self.motion_cfg.get("playback_speed", 1.0))
        self.action_beta = float(self.policy_cfg.get("action_beta", 1.0))
        self.real_world_first_frame_transition_s = float(
            self.policy_cfg.get("real_world_first_frame_transition_s", 0.0)
        )
        self._real_world_first_frame_transition_pending = False
        self.hitter_rng = np.random.default_rng(
            self.hitter_runtime_settings.hitter_seed
        )

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
        self._last_waiting_racket_target_error_log_s: float = 0.0

        self.hard_reset = False
        self.reset_ball_on_command = bool(self.motion_cfg.get("reset_ball_on_command", True))
        self.waiting_racket_body_name = str(self.motion_cfg.get("waiting_racket_body_name", "right_racket_link"))
        self.waiting_time_to_strike_s = (
            self.hitter_runtime_settings.waiting_tts_s
        )
        self.hitter_ball_planner: HitterSystemPlanner = self._build_hitter_ball_planner()
        self._init_hitter_command_state()
        self._init_hitter_lifecycle_state()
        self._initialize_hitter_realtime_runtime()
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
    def _prepare_hitter_reset_state(self) -> None:
        self.playback_speed = 1.0
        self.prev_policy_action = np.zeros(self._policy_dim, dtype=np.float32)
        self._reset_real_world_first_frame_transition_pending()
        self._init_hitter_command_state()
        self._reset_hitter_lifecycle_state()

    def _reset_real_world_first_frame_transition_pending(self) -> None:
        duration_s = float(self.real_world_first_frame_transition_s)
        if not np.isfinite(duration_s) or duration_s < 0.0:
            raise ValueError(
                "policy.real_world_first_frame_transition_s must be a finite non-negative duration in seconds."
            )
        self.real_world_first_frame_transition_s = duration_s
        self._real_world_first_frame_transition_pending = (
            duration_s > 0.0 and bool(getattr(self.simulator, "is_real", False))
        )

    def _real_world_first_frame_transition_enabled(self) -> bool:
        return (
            bool(getattr(self.simulator, "is_real", False))
            and bool(self._real_world_first_frame_transition_pending)
            and float(self.real_world_first_frame_transition_s) > 0.0
        )

    def _run_real_world_first_frame_transition(self, sim_action: np.ndarray) -> None:
        target = np.asarray(sim_action, dtype=np.float32).reshape(-1)
        if not np.all(np.isfinite(target)):
            raise ValueError("real-world first-frame policy target contains non-finite values.")

        high_dt = float(self.simulator.high_dt)
        if not np.isfinite(high_dt) or high_dt <= 0.0:
            raise ValueError("simulator.high_dt must be a finite positive duration.")

        self.simulator.get_state()
        start = np.asarray(self.simulator.dof_pos, dtype=np.float32).reshape(-1)
        if start.shape != target.shape:
            raise ValueError(
                f"real-world first-frame transition start shape {start.shape} does not match target shape {target.shape}."
            )
        if not np.all(np.isfinite(start)):
            raise ValueError("real-world first-frame transition start contains non-finite values.")

        steps = max(
            1,
            int(np.ceil(float(self.real_world_first_frame_transition_s) / high_dt)),
        )
        for step_idx in range(1, steps + 1):
            step_start_s = time.monotonic()
            u = step_idx / steps
            alpha = 3.0 * u * u - 2.0 * u * u * u
            q_cmd = ((1.0 - alpha) * start + alpha * target).astype(np.float32)
            self.simulator.apply_action(q_cmd[None, ...])
            elapsed_s = time.monotonic() - step_start_s
            time.sleep(max(0.0, high_dt - elapsed_s))

        self._real_world_first_frame_transition_pending = False

    def reset(self):
        if self.policy_model_meta is None:
            raise RuntimeError("HitterEnv.reset() called before policy metadata was configured.")
        self._prepare_hitter_reset_state()

        obs_buf_dict = super().reset()
        if self._mujoco_serve_schedule_enabled():
            self._update_mujoco_serve_schedule()

        if self.save_video_enabled:
            self.check_save_video()

        return obs_buf_dict

    def _reset_envs(self, refresh):
        super()._reset_envs(refresh)
        self.simulator.get_state()
        self._capture_hitter_waiting_base_target()

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

        sim_action_batch = sim_action[None, ...]
        if self._real_world_first_frame_transition_enabled():
            self._pre_physics_step(sim_action_batch)
            self._run_real_world_first_frame_transition(self.action.reshape(-1))
            self._post_physics_step()
            return self.obs_buf_dict

        return super().step(sim_action_batch)

    def compute_observation(self):
        return self._compute_hitter_observation()

    def refresh_policy_observation(self):
        """Refresh the command lifecycle and observation immediately before policy inference."""
        self._update_hitter_command()
        self.compute_observation()
        return self.obs_buf_dict

    # --------------------------------------------------------------------- #
    # HITTER command-conditioned observation branch.
    # --------------------------------------------------------------------- #
    def _init_hitter_command_state(self) -> None:
        waiting_base_target_xy_w = self._validated_vector(
            self.motion_cfg.get("waiting_base_target_xy_w", [-0.4, 0.0]),
            name="waiting_base_target_xy_w",
            size=2,
        )
        self.hitter_command_initialized = False
        self.hitter_ball_sequence_needs_reset = True
        self.hitter_strike_type = 0
        self.hitter_strike_elapsed_s = 0.0
        self.hitter_strike_time_s = float(self.waiting_time_to_strike_s)
        self.hitter_strike_duration_s = 1.85
        self.hitter_planner_update_interval_s = (
            self.hitter_runtime_settings.planner_update_interval_s
        )
        self._hitter_last_planner_submit_monotonic_s = None
        self.hitter_incoming_track_confirmation = IncomingTrackConfirmation(
            minimum_speed_x_mps=(
                self.hitter_runtime_settings.minimum_incoming_speed_x_mps
            ),
            required_consecutive_snapshots=(
                self.hitter_runtime_settings.incoming_confirmation_snapshots
            ),
        )
        self.hitter_base_target_xy_w = np.zeros(2, dtype=np.float32)
        self.hitter_base_target_z_w = self._hitter_target_base_height_w()
        self.hitter_waiting_base_target_pos_w = np.asarray(
            [
                waiting_base_target_xy_w[0],
                waiting_base_target_xy_w[1],
                self.hitter_base_target_z_w,
            ],
            dtype=np.float32,
        )
        self.hitter_racket_target_pos_w_fixed = np.zeros(3, dtype=np.float32)
        self.hitter_racket_target_vel_w = np.zeros(3, dtype=np.float32)
        self._init_mujoco_serve_schedule_state()

    def _init_mujoco_serve_schedule_state(self) -> None:
        raw_times = self.motion_cfg.get("mujoco_serve_times_s", None)
        if raw_times is None:
            serve_times = ()
        else:
            serve_times = tuple(float(value) for value in raw_times)
            if serve_times != (0.0, 20.0):
                raise ValueError(
                    "mujoco_serve_times_s must be exactly [0.0, 20.0]."
                )
        self._mujoco_serve_times_s = serve_times
        if not hasattr(self, "_mujoco_serve_origin_time_s"):
            self._mujoco_serve_origin_time_s = None
        if not hasattr(self, "_mujoco_next_serve_index"):
            self._mujoco_next_serve_index = 0
        self.simulator.preserve_hitter_ball_state_on_calibrate = (
            self._mujoco_serve_schedule_enabled()
        )

    def _mujoco_serve_schedule_enabled(self) -> bool:
        return bool(self._mujoco_serve_times_s) and not bool(
            getattr(self.simulator, "is_real", False)
        )

    def _update_mujoco_serve_schedule(self) -> bool:
        if not self._mujoco_serve_schedule_enabled():
            return False
        data = getattr(self.simulator, "mujoco_data", None)
        if data is None:
            raise RuntimeError("MuJoCo serve schedule requires mujoco_data.")
        sim_time_s = float(data.time)
        if not np.isfinite(sim_time_s):
            raise RuntimeError("MuJoCo simulation time is not finite.")
        if self._mujoco_serve_origin_time_s is None:
            self._mujoco_serve_origin_time_s = sim_time_s
        if self._mujoco_next_serve_index >= len(self._mujoco_serve_times_s):
            self.hitter_ball_sequence_needs_reset = False
            return False

        elapsed_s = sim_time_s - self._mujoco_serve_origin_time_s
        due_s = self._mujoco_serve_times_s[self._mujoco_next_serve_index]
        if elapsed_s + 1.0e-12 < due_s:
            self.hitter_ball_sequence_needs_reset = False
            return False

        if self._mujoco_next_serve_index == 0:
            self.simulator.reset_hitter_ball()
            if not self.simulator.capture_hitter_ball_launch_state():
                raise RuntimeError(
                    "Failed to capture the first MuJoCo ball launch state."
                )
        elif not self.simulator.restore_hitter_ball_launch_state():
            raise RuntimeError("Failed to restore the MuJoCo ball launch state.")

        self._mujoco_next_serve_index += 1
        self._hitter_sync_track_epoch += 1
        self._hitter_sync_generation = 0
        self.hitter_ball_sequence_needs_reset = False
        logger.info(
            "MuJoCo HITTER serve {}/{} at simulation time {:.3f} s.",
            self._mujoco_next_serve_index,
            len(self._mujoco_serve_times_s),
            sim_time_s,
        )
        return True

    def _new_hitter_command_lifecycle(self) -> HitterCommandLifecycle:
        return build_hitter_command_lifecycle(
            self.hitter_runtime_settings,
            rng=self.hitter_rng,
        )

    def _init_hitter_lifecycle_state(self) -> None:
        self.hitter_command_lifecycle = self._new_hitter_command_lifecycle()
        self.hitter_planner_worker = None
        self._unregister_ball_listener = None
        self._hitter_last_result_key = None
        self._hitter_minimum_track_epoch = int(
            getattr(self.simulator, "ball_track_epoch", 0)
        )
        self._hitter_minimum_generation = 0
        self._hitter_observed_track_epoch = None
        self._hitter_sync_track_epoch = 0
        self._hitter_sync_generation = 0
        self._hitter_last_logged_phase = self.hitter_command_lifecycle.phase
        self._hitter_last_status_log_monotonic_s = None
        self._hitter_closed = False
        self._hitter_simulator_closed = False

    def _reset_hitter_lifecycle_state(self) -> None:
        self.hitter_command_lifecycle = self._new_hitter_command_lifecycle()
        self._hitter_last_result_key = None
        self._hitter_observed_track_epoch = None
        self._hitter_last_logged_phase = self.hitter_command_lifecycle.phase
        self._hitter_last_status_log_monotonic_s = None
        self._hitter_last_planner_submit_monotonic_s = None

        if getattr(self.simulator, "is_real", False):
            reset_estimator = getattr(
                self.simulator,
                "reset_ball_state_estimator",
                None,
            )
            if callable(reset_estimator):
                reset_estimator()
            self._hitter_minimum_track_epoch = int(
                getattr(self.simulator, "ball_track_epoch", 0)
            )
            self._hitter_minimum_generation = int(
                getattr(self.simulator, "ball_snapshot_generation", 0)
            ) + 1
        else:
            self._hitter_sync_track_epoch += 1
            self._hitter_sync_generation = 0
            self._hitter_minimum_track_epoch = self._hitter_sync_track_epoch
            self._hitter_minimum_generation = 0

    def _initialize_hitter_realtime_runtime(self) -> None:
        if not getattr(self.simulator, "is_real", False):
            self.hitter_planner_worker = None
            return
        register_listener = getattr(
            self.simulator,
            "register_hitter_ball_listener",
            None,
        )
        if not callable(register_listener):
            logger.warning(
                "Real-world HITTER simulator has no ball-listener interface; "
                "realtime planning is disabled."
            )
            return
        if self.hitter_planner_worker is not None:
            return
        self.hitter_planner_worker = LatestOnlyPlannerWorker(
            self._plan_hitter_snapshot,
            monotonic_fn=time.monotonic,
        )
        self._unregister_ball_listener = register_listener(
            self._submit_hitter_planner_snapshot
        )

    def _submit_hitter_planner_snapshot(self, snapshot: BallEstimateSnapshot) -> None:
        worker = self.hitter_planner_worker
        if worker is None:
            return
        received_monotonic_s = float(
            getattr(snapshot, "received_monotonic_s", time.monotonic())
        )
        if not np.isfinite(received_monotonic_s):
            received_monotonic_s = time.monotonic()
        last_submit = self._hitter_last_planner_submit_monotonic_s
        if (
            last_submit is not None
            and received_monotonic_s - last_submit
            < self.hitter_planner_update_interval_s - 1.0e-12
        ):
            return
        self._hitter_last_planner_submit_monotonic_s = received_monotonic_s
        worker.submit(snapshot)

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

    def _capture_hitter_waiting_base_target(self) -> None:
        if not bool(getattr(self.simulator, "base_pose_valid", True)):
            raise RuntimeError(
                "Cannot capture HITTER waiting base target without a valid G2Pelvis pose."
            )
        robot_anchor_pos_w = np.asarray(
            self.simulator.root_trans_world,
            dtype=np.float32,
        ).reshape(-1)
        if robot_anchor_pos_w.size < 3 or not np.isfinite(robot_anchor_pos_w[:3]).all():
            raise RuntimeError(
                "Cannot capture HITTER waiting base target from an invalid pelvis pose."
            )
        waiting_base_target_xy_w = self._validated_vector(
            self.motion_cfg.get("waiting_base_target_xy_w", [-0.4, 0.0]),
            name="waiting_base_target_xy_w",
            size=2,
        )
        self.hitter_waiting_base_target_pos_w = np.asarray(
            [
                waiting_base_target_xy_w[0],
                waiting_base_target_xy_w[1],
                robot_anchor_pos_w[2],
            ],
            dtype=np.float32,
        )
        logger.info(
            "HITTER waiting base target captured at world position {}.",
            self.hitter_waiting_base_target_pos_w.tolist(),
        )

    def _build_hitter_ball_planner(self) -> HitterSystemPlanner:
        planner_cfg = self.motion_cfg.get("ball_planner", {}) or {}
        return build_hitter_system_planner(planner_cfg)

    def _forced_strike_type_name(self) -> Optional[str]:
        planner_cfg = self.motion_cfg.get("ball_planner", {}) or {}
        return forced_strike_type(
            planner_cfg,
            self.motion_cfg,
        )

    def _reset_hitter_ball_sequence_if_needed(self) -> None:
        if not self.hitter_ball_sequence_needs_reset:
            return
        if self.reset_ball_on_command and hasattr(self.simulator, "reset_hitter_ball"):
            self.simulator.reset_hitter_ball()
        self.hitter_ball_sequence_needs_reset = False

    @staticmethod
    def _validated_vector(value, *, name: str, size: int) -> np.ndarray:
        array = np.asarray(value, dtype=np.float32)
        if array.shape != (size,):
            raise ValueError(
                f"HITTER command `{name}` must have shape ({size},), got {array.shape}."
            )
        if not np.isfinite(array).all():
            raise ValueError(f"HITTER command `{name}` contains non-finite values.")
        return array.copy()

    @staticmethod
    def _validated_snapshot_vector(value, *, name: str, size: int) -> np.ndarray:
        array = np.asarray(value, dtype=np.float64)
        if array.shape != (size,):
            raise ValueError(
                f"HITTER `{name}` must have shape ({size},), got {array.shape}."
            )
        if not np.isfinite(array).all():
            raise ValueError(f"HITTER `{name}` contains non-finite values.")
        return array.copy()

    def _validated_hitter_command_fields(self, command) -> dict:
        strike_type = str(command.strike_type).strip().lower()
        if strike_type not in {"forehand", "backhand"}:
            raise ValueError(
                "HITTER command `strike_type` must be forehand or backhand, "
                f"got {command.strike_type!r}."
            )
        time_to_strike = float(command.time_to_strike)
        if not np.isfinite(time_to_strike) or time_to_strike < 0.0:
            raise ValueError(
                "HITTER command `time_to_strike` must be finite and non-negative."
            )
        strike_plan = command.strike_plan
        base_target = self._validated_vector(
            command.p_base_target_xy,
            name="p_base_target_xy",
            size=2,
        )
        racket_target = self._validated_vector(
            strike_plan.p_racket_target,
            name="strike_plan.p_racket_target",
            size=3,
        )
        racket_velocity = self._validated_vector(
            command.v_racket_target_w,
            name="v_racket_target_w",
            size=3,
        )
        ball_in_velocity = self._validated_vector(
            strike_plan.v_ball_in,
            name="strike_plan.v_ball_in",
            size=3,
        )
        base_height = float(self._hitter_target_base_height_w())
        if not np.isfinite(base_height):
            raise ValueError("HITTER target base height must be finite.")
        return {
            "strike_type": strike_type,
            "strike_type_index": 0 if strike_type == "forehand" else 1,
            "time_to_strike": time_to_strike,
            "base_target": base_target,
            "base_height": np.float32(base_height),
            "racket_target": racket_target,
            "racket_velocity": racket_velocity,
            "ball_in_velocity": ball_in_velocity,
        }

    @staticmethod
    def _format_hitter_velocity_log_vector(value: np.ndarray) -> str:
        return "[" + ",".join(
            f"{float(component):.4f}" for component in value
        ) + "]"

    def _log_hitter_strike_target(
        self,
        active_result: PlannerResultSnapshot | None,
    ) -> None:
        try:
            if active_result is None or active_result.command is None:
                raise ValueError("active planner result is missing a command")
            command = active_result.command
            fields = self._validated_hitter_command_fields(command)
            ball_in_velocity = fields["ball_in_velocity"].copy()
            ball_out_velocity = self._validated_vector(
                command.strike_plan.v_ball_out,
                name="strike_plan.v_ball_out",
                size=3,
            )
            racket_velocity = fields["racket_velocity"].copy()
            logger.info(
                "HITTER strike target: epoch={} generation={} type={} "
                "v_ball_in_w_mps={} speed_ball_in_mps={:.4f} "
                "v_ball_out_w_mps={} speed_ball_out_mps={:.4f} "
                "v_racket_target_w_mps={} speed_racket_mps={:.4f}",
                int(active_result.track_epoch),
                int(active_result.source_generation),
                fields["strike_type"],
                self._format_hitter_velocity_log_vector(
                    ball_in_velocity
                ),
                float(np.linalg.norm(ball_in_velocity)),
                self._format_hitter_velocity_log_vector(
                    ball_out_velocity
                ),
                float(np.linalg.norm(ball_out_velocity)),
                self._format_hitter_velocity_log_vector(
                    racket_velocity
                ),
                float(np.linalg.norm(racket_velocity)),
            )
        except Exception as exc:
            try:
                logger.warning(
                    "Failed to log HITTER strike target: {}",
                    exc,
                )
            except Exception:
                pass

    def _copy_hitter_command(self, command, *, validated: dict | None = None) -> None:
        fields = (
            self._validated_hitter_command_fields(command)
            if validated is None
            else validated
        )
        self.hitter_strike_type = fields["strike_type_index"]
        self.hitter_base_target_xy_w = fields["base_target"].copy()
        self.hitter_base_target_z_w = fields["base_height"]
        self.hitter_racket_target_pos_w_fixed = fields["racket_target"].copy()
        self.hitter_racket_target_vel_w = fields["racket_velocity"].copy()
        self.hitter_strike_elapsed_s = 0.0
        self.hitter_strike_time_s = fields["time_to_strike"]
        self.hitter_strike_duration_s = max(
            fields["time_to_strike"]
            + float(self.hitter_command_lifecycle.recovery_duration_s),
            fields["time_to_strike"],
        )

    def _plan_hitter_snapshot(self, snapshot: BallEstimateSnapshot):
        if not snapshot.visible:
            if getattr(self.simulator, "is_real", False):
                self.hitter_incoming_track_confirmation.reset()
            raise RuntimeError(TRACK_ENDED_ERROR)
        if not snapshot.ready:
            raise ValueError("HITTER ball estimator is not ready.")
        if not snapshot.base_valid:
            raise ValueError("HITTER base pose is not valid.")

        ball_pos_w = self._validated_snapshot_vector(
            snapshot.position_w,
            name="snapshot.position_w",
            size=3,
        )
        ball_vel_w = self._validated_snapshot_vector(
            snapshot.velocity_w,
            name="snapshot.velocity_w",
            size=3,
        )
        if getattr(self.simulator, "is_real", False):
            confirmed = self.hitter_incoming_track_confirmation.observe(
                track_epoch=snapshot.track_epoch,
                velocity_x_mps=float(ball_vel_w[0]),
            )
            if not confirmed:
                raise ValueError(
                    "HITTER ball track is not stably incoming: "
                    f"vx={float(ball_vel_w[0]):.3f} m/s."
                )
        base_pos_w = self._validated_snapshot_vector(
            snapshot.base_position_w,
            name="snapshot.base_position_w",
            size=3,
        )
        base_quat_w = self._validated_snapshot_vector(
            snapshot.base_quaternion_xyzw,
            name="snapshot.base_quaternion_xyzw",
            size=4,
        )
        quat_norm = float(np.linalg.norm(base_quat_w))
        if quat_norm < 1.0e-6:
            raise ValueError("HITTER snapshot base quaternion has near-zero norm.")
        base_quat_w /= quat_norm
        base_forward_xy_w = matrix_from_quat(base_quat_w)[:, 0][:2]
        return self.hitter_ball_planner.plan_command(
            ball_pos_w,
            ball_vel_w,
            current_base_xy_w=base_pos_w,
            base_forward_xy_w=base_forward_xy_w,
            strike_type=self._forced_strike_type_name(),
        )

    def _mujoco_planner_result(self, *, now: float) -> PlannerResultSnapshot:
        if self._mujoco_serve_schedule_enabled():
            self._update_mujoco_serve_schedule()
        elif self.hitter_ball_sequence_needs_reset:
            self._reset_hitter_ball_sequence_if_needed()
            self._hitter_sync_track_epoch += 1
            self._hitter_sync_generation = 0
        self.simulator.get_state()
        self._hitter_sync_generation += 1
        snapshot = BallEstimateSnapshot(
            track_epoch=self._hitter_sync_track_epoch,
            generation=self._hitter_sync_generation,
            source_frame=self._hitter_sync_generation,
            source_time_s=float(
                getattr(getattr(self.simulator, "mujoco_data", None), "time", now)
            ),
            received_monotonic_s=now,
            position_w=np.asarray(self.simulator.ball_pos_world).copy(),
            velocity_w=np.asarray(self.simulator.ball_vel_world).copy(),
            base_position_w=np.asarray(self.simulator.root_trans_world).copy(),
            base_quaternion_xyzw=np.asarray(self.simulator.root_quat_world).copy(),
            base_valid=bool(getattr(self.simulator, "base_pose_valid", True)),
            visible=bool(getattr(self.simulator, "ball_visible", False)),
            ready=bool(
                getattr(self.simulator, "ball_state_estimator_ready", False)
            ),
        )
        try:
            command = self._plan_hitter_snapshot(snapshot)
            deadline = now + float(command.time_to_strike)
            return PlannerResultSnapshot(
                track_epoch=snapshot.track_epoch,
                source_generation=snapshot.generation,
                source_frame=snapshot.source_frame,
                strike_deadline_monotonic_s=deadline,
                completed_monotonic_s=now,
                command=command,
            )
        except Exception as exc:
            return PlannerResultSnapshot(
                track_epoch=snapshot.track_epoch,
                source_generation=snapshot.generation,
                source_frame=snapshot.source_frame,
                strike_deadline_monotonic_s=float("nan"),
                completed_monotonic_s=now,
                command=None,
                error=f"{type(exc).__name__}: {exc}",
            )

    def _hitter_result_is_before_reset(self, result: PlannerResultSnapshot) -> bool:
        epoch = int(result.track_epoch)
        generation = int(result.source_generation)
        return epoch < self._hitter_minimum_track_epoch or (
            epoch == self._hitter_minimum_track_epoch
            and generation < self._hitter_minimum_generation
        )

    def _observe_hitter_track_epoch(self, track_epoch: int) -> None:
        epoch = int(track_epoch)
        if (
            self._hitter_observed_track_epoch is None
            or epoch > self._hitter_observed_track_epoch
        ):
            self._hitter_observed_track_epoch = epoch

    def _mark_hitter_track_ended(self, result: PlannerResultSnapshot) -> bool:
        invalid_epoch = int(result.track_epoch)
        active = self.hitter_command_lifecycle.active_result
        active_epoch = None if active is None else int(active.track_epoch)
        current_epochs = [
            epoch
            for epoch in (self._hitter_observed_track_epoch, active_epoch)
            if epoch is not None
        ]
        current_epoch = max(current_epochs) if current_epochs else None
        if current_epoch is not None and invalid_epoch < current_epoch:
            return False
        ended_epoch = invalid_epoch if current_epoch is None else current_epoch
        self.hitter_command_lifecycle.mark_track_ended(int(ended_epoch))
        self._observe_hitter_track_epoch(invalid_epoch)
        logger.info(
            "HITTER track ended: ended_epoch={} invalid_epoch={} generation={}",
            ended_epoch,
            result.track_epoch,
            result.source_generation,
        )
        return True

    def _consume_hitter_planner_result(
        self,
        result: PlannerResultSnapshot | None,
        *,
        now: float,
    ) -> str:
        if result is None:
            return "none"
        key = (int(result.track_epoch), int(result.source_generation))
        if key == self._hitter_last_result_key:
            return "duplicate"
        self._hitter_last_result_key = key
        if self._hitter_result_is_before_reset(result):
            return "stale"
        if result.command is None or result.error is not None:
            if result.error == TRACK_ENDED_RESULT_ERROR:
                return (
                    "track-ended"
                    if self._mark_hitter_track_ended(result)
                    else "stale-track-end"
                )
            self._observe_hitter_track_epoch(result.track_epoch)
            return "failed"

        try:
            fields = self._validated_hitter_command_fields(result.command)
        except Exception as exc:
            logger.warning(
                "Ignored malformed HITTER planner result epoch={} generation={}: {}",
                result.track_epoch,
                result.source_generation,
                exc,
            )
            return "malformed"

        self._observe_hitter_track_epoch(result.track_epoch)
        decision = self.hitter_command_lifecycle.ingest(result, now=now)
        if decision in {"armed", "overridden"}:
            self._copy_hitter_command(result.command, validated=fields)
            logger.info(
                "HITTER command {}: epoch={} generation={} type={} tts={:.3f}s",
                decision,
                result.track_epoch,
                result.source_generation,
                fields["strike_type"],
                self.hitter_command_lifecycle.policy_tts(now=now),
            )
        elif decision == "skipped":
            logger.info(
                "HITTER command skipped as late: epoch={} generation={}",
                result.track_epoch,
                result.source_generation,
            )
        return decision

    def _log_hitter_lifecycle_transition(self, previous_phase: CommandPhase) -> None:
        current_phase = self.hitter_command_lifecycle.phase
        if current_phase != previous_phase:
            logger.info(
                "HITTER lifecycle transition: {} -> {}",
                previous_phase.value,
                current_phase.value,
            )
        self._hitter_last_logged_phase = current_phase

    def _log_hitter_advance_transitions(
        self,
        previous_phase: CommandPhase,
        previous_active: PlannerResultSnapshot | None,
        previous_command_end_deadline_s: float | None,
        *,
        now: float,
    ) -> None:
        current_phase = self.hitter_command_lifecycle.phase
        crossed_strike = bool(
            previous_phase == CommandPhase.ARMED
            and previous_active is not None
            and current_phase != CommandPhase.ARMED
        )
        if crossed_strike:
            self._log_hitter_strike_target(previous_active)
            logger.info(
                "HITTER lifecycle transition: {} -> {}",
                CommandPhase.ARMED.value,
                CommandPhase.RECOVERY.value,
            )
            if current_phase != CommandPhase.RECOVERY:
                logger.info(
                    "HITTER lifecycle transition: {} -> {}",
                    CommandPhase.RECOVERY.value,
                    current_phase.value,
                )
        elif (
            previous_phase == CommandPhase.RECOVERY
            and previous_command_end_deadline_s is not None
            and now >= previous_command_end_deadline_s
        ):
            logger.info(
                "HITTER lifecycle transition: {} -> {}",
                CommandPhase.RECOVERY.value,
                CommandPhase.WAITING.value,
            )
            if current_phase != CommandPhase.WAITING:
                logger.info(
                    "HITTER lifecycle transition: {} -> {}",
                    CommandPhase.WAITING.value,
                    current_phase.value,
                )
        elif current_phase != previous_phase:
            logger.info(
                "HITTER lifecycle transition: {} -> {}",
                previous_phase.value,
                current_phase.value,
            )
        self._hitter_last_logged_phase = current_phase

    def _log_hitter_status(self, *, now: float) -> None:
        worker = self.hitter_planner_worker
        if worker is None:
            return
        last_log = self._hitter_last_status_log_monotonic_s
        if last_log is not None and now - last_log < 1.0:
            return
        self._hitter_last_status_log_monotonic_s = now
        result = worker.latest_result()
        result_age = (
            float("nan")
            if result is None
            else max(now - float(result.completed_monotonic_s), 0.0)
        )
        stats = worker.stats
        logger.info(
            "HITTER realtime status: phase={} submitted={} completed={} failed={} "
            "dropped={} result_age_s={:.3f} epoch={} generation={} policy_tts={:.3f}",
            self.hitter_command_lifecycle.phase.value,
            stats.submitted,
            stats.completed,
            stats.failed,
            stats.dropped_pending,
            result_age,
            None if result is None else result.track_epoch,
            None if result is None else result.source_generation,
            self.hitter_command_lifecycle.policy_tts(now=now),
        )

    def _update_hitter_command(self, *, now: float | None = None) -> None:
        now = time.monotonic() if now is None else float(now)
        previous_phase = self.hitter_command_lifecycle.phase
        previous_active = self.hitter_command_lifecycle.active_result
        previous_command_end_deadline_s = (
            self.hitter_command_lifecycle.command_end_deadline_s
        )
        self.hitter_command_lifecycle.advance(now)
        phase_after_advance = self.hitter_command_lifecycle.phase
        crossed_strike = (
            previous_phase == CommandPhase.ARMED
            and phase_after_advance != CommandPhase.ARMED
        )
        if crossed_strike and getattr(self.simulator, "is_real", False):
            reset_estimator = getattr(
                self.simulator,
                "reset_ball_state_estimator",
                None,
            )
            if callable(reset_estimator):
                reset_estimator()
            else:
                logger.warning(
                    "Real-world HITTER cannot start the next ball track: "
                    "reset_ball_state_estimator() is unavailable."
                )
        self._log_hitter_advance_transitions(
            previous_phase,
            previous_active,
            previous_command_end_deadline_s,
            now=now,
        )
        current_active = self.hitter_command_lifecycle.active_result

        if current_active is not None and current_active is not previous_active:
            fields = self._validated_hitter_command_fields(current_active.command)
            self._copy_hitter_command(current_active.command, validated=fields)
            logger.info(
                "HITTER cached command armed after recovery: epoch={} generation={}",
                current_active.track_epoch,
                current_active.source_generation,
            )
        elif previous_active is not None and current_active is None:
            self.hitter_ball_sequence_needs_reset = True

        if getattr(self.simulator, "is_real", False):
            result = (
                None
                if self.hitter_planner_worker is None
                else self.hitter_planner_worker.latest_result()
            )
        else:
            result = self._mujoco_planner_result(now=now)
        decision = self._consume_hitter_planner_result(result, now=now)
        if decision == "skipped" and not getattr(self.simulator, "is_real", False):
            self.hitter_ball_sequence_needs_reset = True

        phase = self.hitter_command_lifecycle.phase
        self.hitter_command_initialized = bool(
            self.hitter_command_lifecycle.active_result is not None
            and phase in {CommandPhase.ARMED, CommandPhase.RECOVERY}
        )
        self._log_hitter_lifecycle_transition(phase_after_advance)
        self._log_hitter_status(now=now)

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

    def _hitter_waiting_base_target_pos_b(
        self,
        robot_anchor_pos_w: np.ndarray,
        robot_anchor_quat_w: np.ndarray,
    ) -> np.ndarray:
        delta_w = self.hitter_waiting_base_target_pos_w - robot_anchor_pos_w
        return self._yaw_inverse_apply(robot_anchor_quat_w, delta_w)

    def _hitter_racket_target_pos_b(self, robot_anchor_pos_w: np.ndarray, robot_anchor_quat_w: np.ndarray) -> np.ndarray:
        delta_w = self.hitter_racket_target_pos_w_fixed - robot_anchor_pos_w
        return self._yaw_inverse_apply(robot_anchor_quat_w, delta_w)

    def _waiting_racket_target_pos_b(self, robot_anchor_pos_w: np.ndarray, robot_anchor_quat_w: np.ndarray) -> np.ndarray:
        try:
            kinematic = getattr(self.simulator, "kinematic", None)
            if kinematic is None:
                raise RuntimeError("simulator kinematic model is not available")
            fk_info, _ = kinematic.forward(
                joint_pos=np.asarray(self.dof_pos, dtype=np.float64).reshape(-1),
                base_pos=robot_anchor_pos_w.astype(np.float64),
                base_quat=robot_anchor_quat_w.astype(np.float64),
            )
            body_info = fk_info.get(self.waiting_racket_body_name)
            if body_info is None:
                raise KeyError(self.waiting_racket_body_name)
            body_pos_w = np.asarray(body_info["pos"], dtype=np.float32).reshape(-1)[:3]
            return self._yaw_inverse_apply(robot_anchor_quat_w, body_pos_w - robot_anchor_pos_w)
        except Exception as exc:
            now = time.time()
            if now - self._last_waiting_racket_target_error_log_s > 0.5:
                self._last_waiting_racket_target_error_log_s = now
                logger.warning("Failed to compute waiting racket target from FK: {}", exc)
            return np.zeros(3, dtype=np.float32)

    def _compute_hitter_observation(self):
        if self.policy_default_joint_pos is None:
            raise RuntimeError("HITTER observation requested before policy metadata was configured.")

        BaseEnv._update_obs(self)

        robot_anchor_pos_w, robot_anchor_quat_w = self._hitter_robot_anchor_pose_w()

        obs_base_ang_vel = np.asarray(self.base_ang_vel, dtype=np.float32).reshape(-1)[:3]
        obs_projected_gravity = np.asarray(self.projected_gravity, dtype=np.float32).reshape(-1)[:3]
        obs_base_forward_xy = matrix_from_quat(robot_anchor_quat_w)[:, 0][:2].astype(np.float32)
        if self.hitter_command_initialized:
            policy_time_to_strike_s = (
                self.hitter_command_lifecycle.policy_tts(
                    now=time.monotonic()
                )
            )
            active_task_observation = (
                assemble_active_hitter_task_observation(
                    robot_anchor_position_w=robot_anchor_pos_w,
                    robot_anchor_quaternion_xyzw=robot_anchor_quat_w,
                    base_target_xy_w=self.hitter_base_target_xy_w,
                    racket_target_position_w=(
                        self.hitter_racket_target_pos_w_fixed
                    ),
                    racket_target_velocity_w=(
                        self.hitter_racket_target_vel_w
                    ),
                    policy_time_to_strike_s=policy_time_to_strike_s,
                    maximum_policy_time_to_strike_s=(
                        self.hitter_command_lifecycle.maximum_policy_tts
                    ),
                    obs_clip_value=getattr(
                        self.cfg.control,
                        "obs_clip_value",
                        None,
                    ),
                )
            )
            obs_base_forward_xy = active_task_observation.pre_clip[0:2]
            obs_base_target_xy = active_task_observation.pre_clip[2:4]
            obs_racket_target_pos = active_task_observation.pre_clip[4:7]
            obs_racket_target_vel = active_task_observation.pre_clip[7:10]
            obs_time_to_strike = active_task_observation.pre_clip[10:11]
        else:
            obs_base_target_xy = self._hitter_waiting_base_target_pos_b(
                robot_anchor_pos_w,
                robot_anchor_quat_w,
            )[:2]
            obs_racket_target_pos = self._waiting_racket_target_pos_b(robot_anchor_pos_w, robot_anchor_quat_w)
            obs_racket_target_vel = np.zeros(3, dtype=np.float32)
            obs_time_to_strike = np.asarray(
                [
                    self.hitter_command_lifecycle.policy_tts(
                        now=time.monotonic()
                    )
                ],
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
            self._prepare_hitter_reset_state()
            self._reset_envs(True)
            self.compute_observation()
        self.hard_reset = False

    def close(self) -> bool:
        if getattr(self, "_hitter_closed", False):
            return True

        first_error = None
        unregister = getattr(self, "_unregister_ball_listener", None)
        worker = getattr(self, "hitter_planner_worker", None)
        if unregister is not None:
            try:
                unregister()
            except BaseException as error:
                first_error = error
            else:
                self._unregister_ball_listener = None

        if worker is not None:
            try:
                worker.close()
            except BaseException as error:
                if first_error is None:
                    first_error = error
            else:
                self.hitter_planner_worker = None

        simulator_closed = bool(
            getattr(self, "_hitter_simulator_closed", False)
        )
        if not simulator_closed:
            close_simulator = getattr(self.simulator, "close", None)
            if callable(close_simulator):
                try:
                    simulator_closed = close_simulator() is not False
                except BaseException as error:
                    simulator_closed = False
                    if first_error is None:
                        first_error = error
            else:
                simulator_closed = True
            self._hitter_simulator_closed = simulator_closed

        cleanup_complete = bool(
            getattr(self, "_unregister_ball_listener", None) is None
            and getattr(self, "hitter_planner_worker", None) is None
            and simulator_closed
        )
        if first_error is None and cleanup_complete:
            self._hitter_closed = True
        if first_error is not None:
            raise first_error
        return cleanup_complete

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
