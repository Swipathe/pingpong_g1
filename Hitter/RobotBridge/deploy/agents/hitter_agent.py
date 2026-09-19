import time

import numpy as np
from loguru import logger

from agents.base_agent import BaseAgent
from utils.dataset import MosaicModelMeta as HitterModelMeta


class HitterAgent(BaseAgent):
    """Agent wrapper that loads HITTER ONNX metadata and wires it to the environment."""

    def __init__(self, config, env):
        super().__init__(config, env)
        self.startup_interp_enabled = bool(self.config.get("startup_interp_enabled", True))
        self.startup_interp_seconds = float(self.config.get("startup_interp_seconds", 5.0))
        self.startup_action_clip = self.config.get("startup_action_clip", 10.0)
        self.startup_interp_steps = 0
        self.startup_interp_step = 0
        self._startup_interp_start_policy_q = None
        self._waiting_for_first_command_logged = False

    def load_onnx_policy(self):
        super().load_onnx_policy()
        model_meta = HitterModelMeta.from_onnx_session(self.policy)
        if not hasattr(self.env, "configure_from_modelmeta"):
            raise AttributeError("Environment does not support HITTER metadata configuration.")
        self.env.configure_from_modelmeta(model_meta)
        logger.info(
            "HITTER policy metadata loaded. Joints: {} Anchor body: {}",
            len(model_meta.joint_names),
            model_meta.anchor_body_name,
        )

    def _policy_action_for_policy_target(self, target_policy_q: np.ndarray) -> np.ndarray:
        target_policy_q = np.asarray(target_policy_q, dtype=np.float32).reshape(-1)
        default_q = np.asarray(self.env.policy_default_joint_pos, dtype=np.float32).reshape(-1)
        action_scales = np.asarray(self.env.policy_action_scales, dtype=np.float32).reshape(-1)
        if target_policy_q.size != default_q.size:
            raise ValueError(f"target policy q size {target_policy_q.size} does not match policy dim {default_q.size}.")

        action = np.zeros_like(default_q, dtype=np.float32)
        nonzero = np.abs(action_scales) > 1.0e-8
        action[nonzero] = (target_policy_q[nonzero] - default_q[nonzero]) / action_scales[nonzero]

        startup_action_clip = getattr(self, "startup_action_clip", None)
        if startup_action_clip is not None and float(startup_action_clip) > 0.0:
            clip = float(startup_action_clip)
            action = np.clip(action, -clip, clip)
        return action.astype(np.float32)

    def _current_policy_q(self) -> np.ndarray:
        dof_pos = np.asarray(self.env.dof_pos, dtype=np.float32).reshape(-1)
        return self.env._sim_vector_to_policy(dof_pos).astype(np.float32)

    def _prepare_startup_interpolation(self) -> None:
        self.startup_interp_step = 0
        self.startup_interp_steps = 0
        self._startup_interp_start_policy_q = None
        self._waiting_for_first_command_logged = False
        if not self.startup_interp_enabled or self.startup_interp_seconds <= 0.0:
            return

        dt = max(float(getattr(self.env.simulator, "high_dt", 0.02)), 1.0e-6)
        self.startup_interp_steps = max(1, int(round(self.startup_interp_seconds / dt)))
        self._startup_interp_start_policy_q = self._current_policy_q()
        logger.info(
            "HITTER startup interpolation enabled: {:.2f}s, {} steps, from current joints to policy default.",
            self.startup_interp_seconds,
            self.startup_interp_steps,
        )

    def _startup_interp_active(self) -> bool:
        return self._startup_interp_start_policy_q is not None and self.startup_interp_step < self.startup_interp_steps

    def _startup_interp_action(self) -> np.ndarray:
        if self._startup_interp_start_policy_q is None:
            return np.zeros_like(self.env.policy_default_joint_pos, dtype=np.float32)
        alpha = min(float(self.startup_interp_step + 1) / max(float(self.startup_interp_steps), 1.0), 1.0)
        default_q = np.asarray(self.env.policy_default_joint_pos, dtype=np.float32).reshape(-1)
        target_q = (1.0 - alpha) * self._startup_interp_start_policy_q + alpha * default_q
        return self._policy_action_for_policy_target(target_q)

    def _waiting_for_first_hitter_command(self) -> bool:
        if not (getattr(self.env, "hitter_mode", False) and getattr(self.env, "use_ball_planner", False)):
            return False
        has_valid_command = getattr(
            self.env,
            "hitter_has_valid_command",
            getattr(self.env, "hitter_command_initialized", True),
        )
        return not bool(has_valid_command)

    def _default_policy_action(self) -> np.ndarray:
        return np.zeros_like(np.asarray(self.env.policy_default_joint_pos, dtype=np.float32).reshape(-1))

    def _next_action(self, obs_buf_dict):
        if self._startup_interp_active():
            action = self._startup_interp_action()
            self.startup_interp_step += 1
            if self.startup_interp_step == self.startup_interp_steps:
                logger.info("HITTER startup interpolation complete.")
            return action

        if self._waiting_for_first_hitter_command():
            if not self._waiting_for_first_command_logged:
                logger.info("Waiting for first ball planner command; holding HITTER default pose.")
                self._waiting_for_first_command_logged = True
            return self._default_policy_action()

        if self._waiting_for_first_command_logged:
            logger.info("First ball planner command received; running HITTER ONNX policy.")
            self._waiting_for_first_command_logged = False

        inputs = {key: obs_buf_dict[key].astype(np.float32) for key in obs_buf_dict}
        ort_outputs = self.policy.run(None, inputs)
        return ort_outputs[0]

    def run(self):
        obs_buf_dict = self.env.reset()
        self._prepare_startup_interpolation()
        self.time = time.time()
        while True:
            action = self._next_action(obs_buf_dict)

            obs_buf_dict = self.env.step(action)
            if self.env.simulator.is_real:
                time_until_next_step = self.env.simulator.high_dt - (time.time() - self.time)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
                else:
                    logger.warning(f"Time until next step is negative: {time_until_next_step}")
            self.time = time.time()

            if getattr(self.env, "motion_finished", False):
                if hasattr(self.env, "check_save_video"):
                    self.env.check_save_video()
                break

            motion_loader = getattr(self.env, "motion_loader", None)
            if motion_loader is not None and motion_loader.cur_motion_end:
                obs_buf_dict = self.env.next_motion()
            
    def run_eval(self):
        obs_buf_dict = self.env.reset()
        self._prepare_startup_interpolation()
        self.time = time.time()
        while True:
            action = self._next_action(obs_buf_dict)

            obs_buf_dict = self.env.step(action)
            if self.env.simulator.is_real:
                time_until_next_step = self.env.simulator.high_dt - (time.time() - self.time)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
                else:
                    logger.warning(f"Time until next step is negative: {time_until_next_step}")
            self.time = time.time()    

            if getattr(self.env, "motion_finished", False):
                if hasattr(self.env, "check_save_video"):
                    self.env.check_save_video()
                break

            motion_loader = getattr(self.env, "motion_loader", None)
            if motion_loader is not None and motion_loader.cur_motion_end:
                obs_buf_dict = self.env.next_motion()
