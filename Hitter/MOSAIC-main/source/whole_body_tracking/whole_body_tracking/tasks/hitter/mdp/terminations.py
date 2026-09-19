from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from whole_body_tracking.tasks.hitter.mdp.commands import HitterStrikingCommand

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def _command(env: "ManagerBasedRLEnv", command_name: str) -> HitterStrikingCommand:
    return env.command_manager.get_term(command_name)


def hitter_bad_anchor_height(
    env: "ManagerBasedRLEnv",
    command_name: str,
    *,
    target_height: float = 0.793,
    threshold: float = 0.35,
) -> torch.Tensor:
    command = _command(env, command_name)
    return torch.abs(command.robot_anchor_pos_w[:, 2] - float(target_height)) > float(threshold)
