# Copyright (c) 2021-2025, ETH Zurich and NVIDIA CORPORATION
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import os
from dataclasses import asdict
from pathlib import Path

from torch.utils.tensorboard import SummaryWriter

try:
    import swanlab
except ModuleNotFoundError as exc:
    raise ModuleNotFoundError("swanlab is required to log to SwanLab.") from exc


class SwanLabSummaryWriter(SummaryWriter):
    """Summary writer for SwanLab, while keeping local TensorBoard event files."""

    def __init__(self, log_dir: str, flush_secs: int, cfg):
        super().__init__(log_dir, flush_secs)

        self.run_name = os.path.split(log_dir)[-1]
        self.name_map = {
            "Train/mean_reward/time": "Train/mean_reward_time",
            "Train/mean_episode_length/time": "Train/mean_episode_length_time",
        }

        project = (
            cfg.get("swanlab_project")
            or cfg.get("wandb_project")
            or cfg.get("experiment_name")
            or "isaaclab"
        )

        api_key = os.environ.get("SWANLAB_API_KEY")
        if api_key:
            swanlab.login(api_key=api_key, save=False)

        tags = cfg.get("swanlab_tags") or os.environ.get("STAGE1_SWANLAB_TAGS") or os.environ.get("SWANLAB_TAGS")
        if isinstance(tags, str):
            tags = [tag.strip() for tag in tags.split(",") if tag.strip()]

        swanlog_dir = os.path.join(log_dir, "swanlog")
        os.makedirs(swanlog_dir, exist_ok=True)
        self.run = None
        try:
            self.run = swanlab.init(
                project=project,
                experiment_name=self.run_name,
                logdir=swanlog_dir,
                mode=os.environ.get("SWANLAB_MODE", "cloud"),
                tags=tags,
                reinit=True,
                config={"log_dir": log_dir},
            )
        except Exception as exc:
            print(f"[SwanLab] init failed; continuing with local TensorBoard logs only: {exc}", flush=True)

    def add_scalar(self, tag, scalar_value, global_step=None, walltime=None, new_style=False):
        super().add_scalar(
            tag,
            scalar_value,
            global_step=global_step,
            walltime=walltime,
            new_style=new_style,
        )
        step = None if global_step is None else int(global_step)
        if self.run is None:
            return
        try:
            swanlab.log({self._map_path(tag): self._scalar_to_python(scalar_value)}, step=step)
        except Exception as exc:
            print(f"[SwanLab] log failed at step={step}; training continues: {exc}", flush=True)

    def stop(self):
        if self.run is None:
            return
        try:
            self.run.finish()
        except Exception as exc:
            print(f"[SwanLab] finish failed; training already completed: {exc}", flush=True)

    def log_config(self, env_cfg, runner_cfg, alg_cfg, policy_cfg):
        if self.run is None:
            return
        try:
            self.run.config.update(
                {
                    "runner_cfg": self._serialize_cfg(runner_cfg),
                    "policy_cfg": self._serialize_cfg(policy_cfg),
                    "alg_cfg": self._serialize_cfg(alg_cfg),
                    "env_cfg": self._serialize_cfg(env_cfg),
                }
            )
        except Exception as exc:
            print(f"[SwanLab] config upload failed; training continues: {exc}", flush=True)

    def save_model(self, model_path, iter):
        self.save_file(model_path)

    def save_file(self, path, iter=None):
        if self.run is None:
            return
        file_path = Path(path)
        try:
            self.run.save(file_path, base_path=file_path.parent, policy="now")
        except AttributeError:
            try:
                swanlab.save(str(file_path))
            except Exception as exc:
                print(f"[SwanLab] file save failed for {file_path}; training continues: {exc}", flush=True)
        except Exception as exc:
            print(f"[SwanLab] file save failed for {file_path}; training continues: {exc}", flush=True)

    def _map_path(self, path):
        return self.name_map.get(path, path)

    def _scalar_to_python(self, value):
        if hasattr(value, "item"):
            return value.item()
        return value

    def _serialize_cfg(self, cfg):
        if isinstance(cfg, dict):
            return cfg
        if hasattr(cfg, "to_dict"):
            return cfg.to_dict()
        try:
            return asdict(cfg)
        except TypeError:
            return cfg
