import time

import numpy as np
from loguru import logger

from agents.base_agent import BaseAgent
from utils.dataset import MosaicModelMeta as HitterModelMeta


class HitterAgent(BaseAgent):
    """Agent wrapper that loads HITTER ONNX metadata and wires it to the environment."""

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

    def _run_iteration(self, obs_buf_dict):
        if self.env.simulator.is_real:
            obs_buf_dict = self.env.refresh_policy_observation()
        inputs = {key: value.astype(np.float32) for key, value in obs_buf_dict.items()}
        # print("inputs:", {key: value for key, value in inputs.items()})
        action = self.policy.run(None, inputs)[0]
        return self.env.step(action)

    def run(self):
        print("In run")
        try:
            obs_buf_dict = self.env.reset()
            self.time = time.time()
            while True:
                obs_buf_dict = self._run_iteration(obs_buf_dict)
                if self.env.simulator.is_real:
                    time_until_next_step = self.env.simulator.high_dt - (time.time() - self.time)
                    if time_until_next_step > 0:
                        time.sleep(time_until_next_step)
                    else:
                        logger.warning(f"Time until next step is negative: {time_until_next_step}")
                self.time = time.time()
        finally:
            print("something error, close!!!!!!!!!!!!!!!!!!!!!!!")
            self.env.close()

    def run_eval(self):
        try:
            obs_buf_dict = self.env.reset()
            self.time = time.time()
            while True:
                obs_buf_dict = self._run_iteration(obs_buf_dict)
                if self.env.simulator.is_real:
                    time_until_next_step = self.env.simulator.high_dt - (time.time() - self.time)
                    if time_until_next_step > 0:
                        time.sleep(time_until_next_step)
                    else:
                        logger.warning(f"Time until next step is negative: {time_until_next_step}")
                self.time = time.time()
        finally:
            self.env.close()
