from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.utils.math import quat_apply, subtract_frame_transforms

from whole_body_tracking.tasks.hitter.mdp.commands import HitterStrikingCommand

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv, ManagerBasedRLEnv


def _hitter_command(env: "ManagerBasedEnv", command_name: str) -> HitterStrikingCommand:
    return env.command_manager.get_term(command_name)


def hitter_base_forward_xy(env: "ManagerBasedEnv", command_name: str) -> torch.Tensor:
    command = _hitter_command(env, command_name)
    forward_b = torch.zeros(env.num_envs, 3, device=env.device)
    forward_b[:, 0] = 1.0
    forward_w = quat_apply(command.robot_anchor_quat_w, forward_b)
    return forward_w[:, :2]


def hitter_base_target_xy(env: "ManagerBasedEnv", command_name: str) -> torch.Tensor:
    command = _hitter_command(env, command_name)
    return command.base_target_xy_b


def hitter_base_target_pos(env: "ManagerBasedEnv", command_name: str) -> torch.Tensor:
    command = _hitter_command(env, command_name)
    return command.base_target_pos_b


def hitter_racket_target_pos(env: "ManagerBasedEnv", command_name: str) -> torch.Tensor:
    command = _hitter_command(env, command_name)
    return command.racket_target_pos_b


def hitter_racket_target_vel(env: "ManagerBasedEnv", command_name: str) -> torch.Tensor:
    command = _hitter_command(env, command_name)
    return command.racket_target_vel_w


def hitter_time_to_strike(env: "ManagerBasedEnv", command_name: str) -> torch.Tensor:
    command = _hitter_command(env, command_name)
    return command.time_to_strike_s.unsqueeze(-1)


def hitter_time_left(env: "ManagerBasedRLEnv") -> torch.Tensor:
    max_episode_length = getattr(env, "max_episode_length", None)
    if max_episode_length is None:
        max_episode_length = int(round(env.cfg.episode_length_s / env.step_dt))
    episode_length_buf = getattr(env, "episode_length_buf", None)
    if episode_length_buf is None:
        return torch.full((env.num_envs, 1), env.cfg.episode_length_s, dtype=torch.float32, device=env.device)
    steps_left = torch.clamp(max_episode_length - episode_length_buf, min=0)
    return (steps_left.to(torch.float32) * env.step_dt).unsqueeze(-1)


def hitter_reference_joint_state(env: "ManagerBasedEnv", command_name: str) -> torch.Tensor:
    command = _hitter_command(env, command_name)
    return torch.cat([command.joint_pos, command.joint_vel], dim=-1)


def hitter_robot_body_pose(env: "ManagerBasedEnv", command_name: str) -> torch.Tensor:
    command = _hitter_command(env, command_name)
    body_pos_b, body_quat_b = subtract_frame_transforms(
        command.robot_anchor_pos_w[:, None, :].repeat(1, len(command.cfg.body_names), 1),
        command.robot_anchor_quat_w[:, None, :].repeat(1, len(command.cfg.body_names), 1),
        command.robot_body_pos_w,
        command.robot_body_quat_w,
    )
    return torch.cat([body_pos_b, body_quat_b], dim=-1).reshape(env.num_envs, -1)
