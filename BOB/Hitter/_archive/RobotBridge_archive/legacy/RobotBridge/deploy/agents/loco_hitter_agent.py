from __future__ import annotations

import time
from enum import Enum, auto
from typing import Optional

import numpy as np
import torch
from loguru import logger

from agents.base_agent import BaseAgent
from utils.dataset import MosaicModelMeta as HitterModelMeta


class InterpState(Enum):
    IDLE = auto()
    START = auto()
    IN_PROGRESS = auto()
    END = auto()


class LocoHitterInterpManager:
    """Smoothly transitions the upper body while locomotion keeps the lower body alive."""

    def __init__(self, env):
        self.env = env
        self.interp_state = InterpState.IDLE
        self.interp_timestep = 0
        self.interp_durations = [0, 1, 0]
        self.interp_callback_start = None
        self.interp_callback_end = None
        self.pending_actions = []

        self.interp_start_pos: Optional[np.ndarray] = None
        self.interp_target_pos: Optional[np.ndarray] = None
        self.interp_get_target_pos = None
        self.advance_hitter_time = False
        self.loco_dof_pos = self.env.loco_default_angles[self.env.simulator.active_dof_idx].copy()
        self.override_dof_pos = self.loco_dof_pos.copy()

    def _interpolate_init(
        self,
        get_target_pos,
        durations,
        callback_start=None,
        callback_end=None,
        advance_hitter_time=False,
    ):
        if self.interp_state != InterpState.IDLE:
            logger.warning("Ignoring policy switch while interpolation is still in progress.")
            return

        interp_steps = max(int(durations[1]), 1)
        self.interp_get_target_pos = get_target_pos
        self.interp_durations = [max(int(durations[0]), 0), interp_steps, max(int(durations[2]), 0)]
        self.interp_callback_start = callback_start
        self.interp_callback_end = callback_end
        self.advance_hitter_time = bool(advance_hitter_time)
        self.interp_state = InterpState.START
        self.interp_timestep = 0
        self.pending_actions = []

        if self.interp_durations[0] == 0:
            self._interpolate_start()
        else:
            self.pending_actions.append(("start", self.interp_durations[0]))

        self.pending_actions.append(("end", sum(self.interp_durations) + 1))

    def _interpolate_start(self):
        if self.interp_state != InterpState.START:
            return
        if self.interp_callback_start is not None:
            self.interp_callback_start()
            self.interp_callback_start = None

        self.interp_start_pos = np.asarray(self.env.dof_pos, dtype=np.float32).reshape(-1).copy()
        self.interp_target_pos = np.asarray(self.interp_get_target_pos(), dtype=np.float32).reshape(-1).copy()
        self.override_dof_pos = self.interp_start_pos.copy()
        self.interp_timestep = 0
        self.interp_state = InterpState.IN_PROGRESS

    def _interpolate_step(self):
        if self.interp_state != InterpState.IN_PROGRESS:
            return
        if self.advance_hitter_time and bool(getattr(self.env, "hitter_command_initialized", False)):
            playback_speed = float(getattr(self.env, "playback_speed", 1.0))
            if playback_speed > 0.0:
                dt = float(getattr(getattr(self.env, "simulator", None), "high_dt", 0.0))
                self.env.hitter_strike_elapsed_s += dt * playback_speed
        alpha = min(float(self.interp_timestep) / float(self.interp_durations[1]), 1.0)
        self.override_dof_pos = (1.0 - alpha) * self.interp_start_pos + alpha * self.interp_target_pos
        if self.interp_timestep < self.interp_durations[1]:
            self.interp_timestep += 1
        else:
            self.interp_state = InterpState.END

    def _interpolate_end(self):
        if self.interp_state != InterpState.END:
            return
        self.override_dof_pos = self.interp_target_pos.copy()
        if self.interp_callback_end is not None:
            self.interp_callback_end()
            self.interp_callback_end = None
        self.advance_hitter_time = False
        self.interp_state = InterpState.IDLE

    def step(self):
        new_pending = []
        for action_type, delay in self.pending_actions:
            if delay <= 0:
                if action_type == "start":
                    self._interpolate_start()
                elif action_type == "end":
                    self._interpolate_end()
            else:
                new_pending.append((action_type, delay - 1))
        self.pending_actions = new_pending
        self._interpolate_step()

    def switch_to_locomotion(self):
        if self.env.policy_mode == "locomotion" and self.interp_state == InterpState.IDLE:
            return
        logger.info("Switching HITTER -> locomotion")
        self._interpolate_init(
            get_target_pos=lambda: self.env.loco_default_angles[self.env.simulator.active_dof_idx],
            durations=[0, self.env.hitter_to_loco_steps, 0],
            callback_start=lambda: self.env.set_policy_mode("locomotion"),
        )

    def switch_to_hitter(self):
        if self.env.policy_mode == "hitter" or self.env.hitter_default_angles is None:
            return
        logger.info("Switching locomotion -> HITTER")
        self._interpolate_init(
            get_target_pos=lambda: self.env.hitter_default_angles[self.env.simulator.active_dof_idx],
            durations=[0, self.env.loco_to_hitter_steps, 0],
            callback_end=lambda: self.env.set_policy_mode("hitter"),
            advance_hitter_time=True,
        )

    @property
    def idle(self) -> bool:
        return self.interp_state == InterpState.IDLE


class LocoHitterAgent(BaseAgent):
    """Runs a locomotion policy by default and temporarily switches to HITTER for valid balls."""

    def __init__(self, config, env):
        self.config = config
        self.env = env
        self.device = self.config.get("device", "cpu")
        self.time = 0.0
        self.enable_keyboard_commands = bool(self.config.get("enable_keyboard_commands", True))
        self.enable_auto_hitter_switch = bool(self.config.get("enable_auto_hitter_switch", False))
        self.log_armed_hitter_without_switch = bool(self.config.get("log_armed_hitter_without_switch", True))

        self._load_locomotion_policy()
        self._load_hitter_policy()
        self.interp_manager = LocoHitterInterpManager(self.env)

        logger.info("LocoHitterAgent initialized. Runtime starts in locomotion mode.")

    def _load_locomotion_policy(self):
        loco_ckpt = self.config.locomotion_checkpoint
        self.loco_policy = torch.jit.load(loco_ckpt).to(self.device)
        self.loco_policy.eval()
        logger.info("Loaded locomotion policy from {}", loco_ckpt)

    def _load_hitter_policy(self):
        import onnxruntime as ort

        hitter_ckpt = self.config.hitter_checkpoint
        self.hitter_policy = ort.InferenceSession(hitter_ckpt)
        model_meta = HitterModelMeta.from_onnx_session(self.hitter_policy)
        if not hasattr(self.env, "configure_from_modelmeta"):
            raise AttributeError("Environment does not support HITTER metadata configuration.")
        self.env.configure_from_modelmeta(model_meta)
        logger.info(
            "Loaded HITTER policy from {} | joints={} anchor={}",
            hitter_ckpt,
            len(model_meta.joint_names),
            model_meta.anchor_body_name,
        )

    def _get_policy_action(self, obs_buf_dict):
        if self.env.policy_mode == "locomotion":
            if bool(getattr(self.env, "stand_hold_only", False)):
                return np.zeros((1, self.env.lower_dof_num), dtype=np.float32)
            actor_obs = torch.from_numpy(obs_buf_dict["actor_obs"]).float().to(self.device)
            with torch.no_grad():
                return self.loco_policy(actor_obs).cpu().numpy()
        if self.env.policy_mode == "hitter":
            inputs = {key: obs_buf_dict[key].astype(np.float32) for key in obs_buf_dict}
            return self.hitter_policy.run(None, inputs)[0]
        raise ValueError(f"Unknown policy mode: {self.env.policy_mode}")

    def _apply_override_dof_pos(self, action):
        if self.interp_manager.idle:
            self.env.override_dof_pos = None
            return action
        self.env.override_dof_pos = self.interp_manager.override_dof_pos.copy()
        return action

    def _handle_commands(self):
        if not self.enable_keyboard_commands:
            return
        pending_switch = self.env.get_pending_policy_switch()
        if pending_switch == "locomotion":
            self.interp_manager.switch_to_locomotion()
        elif pending_switch == "hitter":
            if self.env.policy_mode == "locomotion" and self.env.try_arm_hitter_from_ball():
                self.interp_manager.switch_to_hitter()
            else:
                logger.warning("HITTER switch requested but no valid armed ball command is available.")

    def _auto_arm_hitter_if_ready(self):
        if not self.interp_manager.idle or self.env.policy_mode != "locomotion":
            return
        if self.env.try_arm_hitter_from_ball():
            if not self.enable_auto_hitter_switch:
                if self.log_armed_hitter_without_switch:
                    logger.warning(
                        "Valid ball command armed, but auto HITTER switch is disabled; staying in locomotion."
                    )
                if hasattr(self.env, "discard_armed_hitter_command"):
                    self.env.discard_armed_hitter_command("auto HITTER switch disabled")
                return
            logger.info(
                "Valid ball command armed; switching to HITTER with tts={:.3f}s",
                float(getattr(self.env, "hitter_strike_time_s", 0.0)),
            )
            self.interp_manager.switch_to_hitter()

    def _auto_return_to_locomotion_if_done(self):
        if not self.interp_manager.idle or self.env.policy_mode != "hitter":
            return
        if self.env.hitter_should_return_to_locomotion():
            self.interp_manager.switch_to_locomotion()

    def run(self):
        obs_buf_dict = self.env.reset()
        self.time = time.time()
        prev_policy_mode = self.env.policy_mode

        while True:
            self._auto_arm_hitter_if_ready()

            action = self._get_policy_action(obs_buf_dict)
            if action.ndim == 1:
                action = action[None, :]
            action = self._apply_override_dof_pos(action)

            obs_buf_dict = self.env.step(action.squeeze())

            self.interp_manager.step()
            self._handle_commands()
            self._auto_return_to_locomotion_if_done()

            if self.env.policy_mode != prev_policy_mode:
                prev_policy_mode = self.env.policy_mode
                obs_buf_dict = self.env.obs_buf_dict

            if self.env.simulator.is_real:
                time_until_next_step = self.env.simulator.high_dt - (time.time() - self.time)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
                else:
                    logger.warning("Step timeout: {:.4f}s", time_until_next_step)
            self.time = time.time()
