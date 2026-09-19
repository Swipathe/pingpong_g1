from __future__ import annotations

from typing import Dict, Union

import numpy as np
from loguru import logger
from omegaconf import DictConfig, OmegaConf

from envs.base_env import BaseEnv
from envs.hitter import HitterEnv
from envs.loco_mimic_switch import LocoMimicSwitchEnv
from utils.dataset import MosaicModelMeta as HitterModelMeta


class LocoHitterEnv(HitterEnv):
    """Real-robot environment that keeps locomotion active until a valid HITTER ball command is armed."""

    _init_locomotion = LocoMimicSwitchEnv._init_locomotion
    _init_teleop = LocoMimicSwitchEnv._init_teleop
    _reset_locomotion = LocoMimicSwitchEnv._reset_locomotion
    _compute_locomotion_observation = LocoMimicSwitchEnv._compute_locomotion_observation
    _build_single_locomotion_obs = LocoMimicSwitchEnv._build_single_locomotion_obs
    _update_gait_phase = LocoMimicSwitchEnv._update_gait_phase
    _update_commands = LocoMimicSwitchEnv._update_commands

    def __init__(self, config: DictConfig):
        super().__init__(config)

        cfg_dict = OmegaConf.to_container(config, resolve=True) if isinstance(config, DictConfig) else config
        cfg_dict = cfg_dict if isinstance(cfg_dict, dict) else {}

        self.loco_cfg: Dict[str, Union[float, int, list]] = cfg_dict.get("locomotion", {}) or {}
        self.teleop_cfg: Dict[str, Union[float, int, dict]] = cfg_dict.get("teleop", {}) or {}
        self.loco_policy_control: Dict[str, Union[float, int]] = cfg_dict.get("locomotion_policy_control", {}) or {}
        switch_cfg = cfg_dict.get("hitter_switch", {}) or {}

        self.policy_mode = "locomotion"
        self.pending_policy_switch = None
        self.override_dof_pos = None

        self.loco_default_angles = self._array_from_switch_cfg(
            switch_cfg,
            "locomotion_default_angles",
            self.simulator.default_angles,
        )
        self.loco_kps = self._array_from_switch_cfg(switch_cfg, "locomotion_kps", self.simulator.kps)
        self.loco_kds = self._array_from_switch_cfg(switch_cfg, "locomotion_kds", self.simulator.kds)
        self.hitter_default_angles = None
        self.hitter_kps = None
        self.hitter_kds = None

        self.auto_switch_on_ball = bool(switch_cfg.get("auto_switch_on_ball", True))
        self.switch_back_after_hitter = bool(switch_cfg.get("switch_back_after_hitter", True))
        self.loco_to_hitter_steps = int(switch_cfg.get("loco_to_hitter_steps", 10))
        self.hitter_to_loco_steps = int(switch_cfg.get("hitter_to_loco_steps", 25))
        self.hitter_return_after_strike_s = float(switch_cfg.get("return_after_strike_s", 0.15))
        self.hitter_max_duration_s = float(switch_cfg.get("hitter_max_duration_s", 2.4))
        self.stand_hold_only = bool(switch_cfg.get("stand_hold_only", False))
        self.hitter_run_steps = 0

        self.upper_dof_num = int(cfg_dict.get("upper_dof_num", 17))
        self.lower_dof_num = self.simulator.num_action - self.upper_dof_num
        self._init_locomotion()

        logger.info(
            "LocoHitterEnv initialized | Total DoF: {}, Lower: {}, Upper: {}, auto_switch_on_ball={}",
            self.simulator.num_action,
            self.lower_dof_num,
            self.upper_dof_num,
            self.auto_switch_on_ball,
        )

    def _array_from_switch_cfg(self, switch_cfg: dict, name: str, fallback: np.ndarray) -> np.ndarray:
        value = switch_cfg.get(name, None)
        if value is None:
            return np.asarray(fallback, dtype=np.float32).copy()
        arr = np.asarray(value, dtype=np.float32).reshape(-1)
        fallback_arr = np.asarray(fallback, dtype=np.float32).reshape(-1)
        if arr.size != fallback_arr.size:
            raise ValueError(f"hitter_switch.{name} must contain {fallback_arr.size} values, got {arr.size}.")
        return arr.copy()

    def configure_from_modelmeta(self, model_meta: HitterModelMeta) -> None:
        """Configure HITTER metadata, then restore locomotion as the default runtime mode."""
        super().configure_from_modelmeta(model_meta)
        self.hitter_default_angles = self.simulator.default_angles.copy()
        self.hitter_kps = self.simulator.kps.copy()
        self.hitter_kds = self.simulator.kds.copy()
        self._switch_to_locomotion_pd_params()
        self.policy_mode = "locomotion"
        logger.info("HITTER policy configured; runtime starts in locomotion mode.")

    def reset(self):
        if self.policy_model_meta is None:
            raise RuntimeError("LocoHitterEnv.reset() called before HITTER policy metadata was configured.")
        self.policy_mode = "locomotion"
        self.pending_policy_switch = None
        self.override_dof_pos = None
        self.hitter_run_steps = 0
        self._switch_to_locomotion_pd_params()
        self._reset_locomotion()

        self.motion_finished = False
        self.total_policy_steps = 0
        self.playback_speed = 1.0
        self.prev_policy_action = np.zeros(self._policy_dim, dtype=np.float32)
        self._init_hitter_command_state()

        BaseEnv.reset(self)
        return self.obs_buf_dict

    def compute_observation(self):
        if self.policy_mode == "locomotion":
            BaseEnv._update_obs(self)
            self._update_commands()
            self._compute_locomotion_observation()
            return
        if self.policy_mode == "hitter":
            HitterEnv.compute_observation(self)
            return
        raise ValueError(f"Unknown policy mode: {self.policy_mode}")

    def step(self, action):
        if self.policy_mode == "locomotion":
            return self._step_locomotion(action)
        if self.policy_mode == "hitter":
            return self._step_hitter(action)
        raise ValueError(f"Unknown policy mode: {self.policy_mode}")

    def _step_locomotion(self, action):
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        clip_action_limit = float(self.loco_policy_control.get("action_clip_value", 20.0))
        action_scale = float(self.loco_policy_control.get("action_scale", 0.25))
        self.action_obs = np.clip(action, -clip_action_limit, clip_action_limit)

        lower_default = self.simulator.default_angles[self.simulator.active_dof_idx][: self.lower_dof_num]
        if self.stand_hold_only:
            target_lower = lower_default.copy()
        else:
            target_lower = lower_default + self.action_obs[: self.lower_dof_num] * action_scale

        full_action = np.zeros((1, self.simulator.num_action), dtype=np.float32)
        full_action[0, : self.lower_dof_num] = target_lower[: self.lower_dof_num]
        if self.override_dof_pos is not None:
            full_action[0, self.lower_dof_num :] = self.override_dof_pos[self.lower_dof_num :]
        else:
            full_action[0, self.lower_dof_num :] = self.simulator.default_angles[self.simulator.active_dof_idx][
                self.lower_dof_num :
            ]

        return BaseEnv.step(self, full_action)

    def _step_hitter(self, action):
        self.hitter_run_steps += 1
        return HitterEnv.step(self, action)

    def _post_physics_step(self):
        if self.policy_mode == "locomotion":
            BaseEnv._post_physics_step(self)
            return
        if self.policy_mode == "hitter":
            HitterEnv._post_physics_step(self)
            return
        raise ValueError(f"Unknown policy mode: {self.policy_mode}")

    def set_policy_mode(self, mode: str):
        mode = "hitter" if mode == "mimic" else mode
        if mode not in {"locomotion", "hitter"}:
            raise ValueError(f"Invalid policy mode: {mode}")
        if mode == self.policy_mode:
            return

        old_mode = self.policy_mode
        self.policy_mode = mode
        logger.info("Policy mode switched from {} to {}", old_mode, mode)

        if mode == "locomotion":
            self.hitter_run_steps = 0
            self._init_hitter_command_state()
            self._switch_to_locomotion_pd_params()
        else:
            self.hitter_run_steps = 0
            self._switch_to_hitter_pd_params()
            self._reset_hitter_obs_buffers()

        self.compute_observation()

    def _handle_policy_switch(self, cmd):
        if not hasattr(cmd, "policy_switch") or cmd.policy_switch is None:
            return
        requested = "hitter" if cmd.policy_switch == "mimic" else cmd.policy_switch
        if requested in {"locomotion", "hitter"} and requested != self.policy_mode:
            logger.info("Policy switch requested: {} -> {}", self.policy_mode, requested)
            self.pending_policy_switch = requested

    def get_pending_policy_switch(self):
        switch = self.pending_policy_switch
        self.pending_policy_switch = None
        return switch

    def set_command_velocity(self, vx: float, vy: float, yaw: float):
        self.command_lin_vel[0, 0] = vx
        self.command_lin_vel[0, 1] = vy
        self.command_ang_vel[0, 0] = yaw

    def try_arm_hitter_from_ball(self) -> bool:
        if not self.auto_switch_on_ball or self.policy_mode != "locomotion":
            return False
        if self.hitter_ball_planner is None:
            return False
        self._switch_to_hitter_pd_params()
        try:
            handled = self._sample_hitter_command_from_ball_planner()
        finally:
            if self.policy_mode == "locomotion":
                self._switch_to_locomotion_pd_params()
        return bool(handled and self.hitter_command_initialized and not self.hitter_waiting_for_planner_arm)

    def hitter_should_return_to_locomotion(self) -> bool:
        if not self.switch_back_after_hitter or self.policy_mode != "hitter":
            return False
        elapsed = float(getattr(self, "hitter_strike_elapsed_s", 0.0))
        strike_time = float(getattr(self, "hitter_strike_time_s", 0.0))
        grace_hit = elapsed >= strike_time + self.hitter_return_after_strike_s
        timeout = elapsed >= self.hitter_max_duration_s
        return bool(grace_hit or timeout or getattr(self, "motion_finished", False))

    def _switch_to_locomotion_pd_params(self):
        if self.loco_default_angles is not None:
            self.simulator.default_angles = self.loco_default_angles.copy()
        if self.loco_kps is not None:
            self.simulator.kps = self.loco_kps.copy()
        if self.loco_kds is not None:
            self.simulator.kds = self.loco_kds.copy()
        self._sync_sim_asset_control_params()

    def _switch_to_hitter_pd_params(self):
        if self.hitter_default_angles is None or self.hitter_kps is None or self.hitter_kds is None:
            logger.warning("HITTER PD parameters not configured yet; keeping current simulator parameters.")
            return
        self.simulator.default_angles = self.hitter_default_angles.copy()
        self.simulator.kps = self.hitter_kps.copy()
        self.simulator.kds = self.hitter_kds.copy()
        self._sync_sim_asset_control_params()

    def _sync_sim_asset_control_params(self):
        asset_cfg = getattr(getattr(self.simulator, "cfg", None), "asset", None)
        if asset_cfg is None:
            return
        if hasattr(asset_cfg, "default_angles"):
            asset_cfg.default_angles = np.asarray(self.simulator.default_angles, dtype=np.float32).tolist()
        if hasattr(asset_cfg, "kps"):
            asset_cfg.kps = np.asarray(self.simulator.kps, dtype=np.float32).tolist()
        if hasattr(asset_cfg, "kds"):
            asset_cfg.kds = np.asarray(self.simulator.kds, dtype=np.float32).tolist()

    def _reset_hitter_obs_buffers(self):
        command_horizon = int((self.motion_cfg.get("command_horizon", 1) or 1))
        zeros = {
            "command": np.zeros(58 * command_horizon, dtype=np.float32),
            "anchor_ori": np.zeros(6, dtype=np.float32),
            "ang_vel": np.zeros(3, dtype=np.float32),
            "joint_pos": np.zeros(29, dtype=np.float32),
            "joint_vel": np.zeros(29, dtype=np.float32),
            "action": np.zeros(29, dtype=np.float32),
            "projected_gravity": np.zeros(3, dtype=np.float32),
        }
        for buffer, key in (
            (self.obs_command_buffer, "command"),
            (self.obs_motion_anchor_ori_b_buffer, "anchor_ori"),
            (self.obs_base_ang_vel_buffer, "ang_vel"),
            (self.obs_joint_pos_rel_buffer, "joint_pos"),
            (self.obs_joint_vel_rel_buffer, "joint_vel"),
            (self.obs_prev_policy_action_buffer, "action"),
            (self.obs_projected_gravity_buffer, "projected_gravity"),
        ):
            buffer.clear()
            for _ in range(self.history_length):
                buffer.append(zeros[key].copy())
