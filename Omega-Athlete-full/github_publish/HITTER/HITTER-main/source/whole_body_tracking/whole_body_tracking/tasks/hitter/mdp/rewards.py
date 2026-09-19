from __future__ import annotations

import re
from typing import TYPE_CHECKING

import torch

from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor
from isaaclab.utils.math import quat_apply, quat_rotate_inverse, yaw_quat

from whole_body_tracking.tasks.hitter.mdp.commands import HitterStrikingCommand

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


G1_SOLE_EDGE_OFFSETS = (
    (-0.0540, 0.0000, -0.0350),
    (-0.0440, 0.0280, -0.0350),
    (-0.0440, -0.0280, -0.0350),
    (0.0400, 0.0360, -0.0350),
    (0.0400, -0.0360, -0.0350),
    (0.1230, 0.0280, -0.0350),
    (0.1230, -0.0280, -0.0350),
    (0.1320, 0.0000, -0.0350),
)


def _command(env: "ManagerBasedRLEnv", command_name: str) -> HitterStrikingCommand:
    return env.command_manager.get_term(command_name)


def _before_strike(command: HitterStrikingCommand) -> torch.Tensor:
    return (command.signed_time_to_strike >= 0.0).to(torch.float32)


def _strike_window(command: HitterStrikingCommand, window_s: float | None) -> torch.Tensor:
    if window_s is None:
        return torch.ones(command.num_envs, dtype=torch.float32, device=command.device)
    return (torch.abs(command.signed_time_to_strike) <= float(window_s)).to(torch.float32)


def _strike_scaled_penalty(
    penalty: torch.Tensor,
    command: HitterStrikingCommand,
    strike_window_s: float | None = None,
    strike_scale: float = 1.0,
) -> torch.Tensor:
    if strike_window_s is None or float(strike_scale) == 1.0:
        return penalty
    window = _strike_window(command, strike_window_s)
    return penalty * (1.0 + (float(strike_scale) - 1.0) * window)


def _get_joint_indexes(command: HitterStrikingCommand, joint_names: list[str] | None) -> list[int]:
    if joint_names is None:
        return list(range(command.joint_pos.shape[1]))
    return [
        index
        for index, name in enumerate(command.robot.joint_names)
        if any(re.fullmatch(pattern, name) for pattern in joint_names)
    ]


def hitter_joint_position_imitation_exp(
    env: "ManagerBasedRLEnv",
    command_name: str,
    std: float,
    joint_names: list[str] | None = None,
) -> torch.Tensor:
    command = _command(env, command_name)
    joint_indexes = _get_joint_indexes(command, joint_names)
    if len(joint_indexes) == 0:
        return torch.zeros(env.num_envs, device=env.device)
    error = torch.square(command.joint_pos[:, joint_indexes] - command.robot_joint_pos[:, joint_indexes])
    return torch.exp(-error.mean(-1) / std**2)


def hitter_joint_default_position_error_exp(
    env: "ManagerBasedRLEnv",
    asset_cfg: SceneEntityCfg,
    std: float,
) -> torch.Tensor:
    robot = env.scene[asset_cfg.name]
    joint_pos = robot.data.joint_pos[:, asset_cfg.joint_ids]
    default_joint_pos = getattr(robot.data, "default_joint_pos_nominal", robot.data.default_joint_pos)
    if default_joint_pos.ndim == 1:
        default_joint_pos = default_joint_pos.unsqueeze(0).expand_as(robot.data.joint_pos)
    default_joint_pos = default_joint_pos[:, asset_cfg.joint_ids]
    error = torch.square(joint_pos - default_joint_pos).mean(dim=-1)
    return torch.exp(-error / std**2)


def hitter_racket_position_error_exp(
    env: "ManagerBasedRLEnv",
    command_name: str,
    std: float,
    window_s: float | None = None,
) -> torch.Tensor:
    command = _command(env, command_name)
    error = torch.sum(torch.square(command.racket_pos_b - command.racket_target_pos_b), dim=-1)
    return torch.exp(-error / std**2) * _strike_window(command, window_s)


def hitter_racket_velocity_error_exp(
    env: "ManagerBasedRLEnv",
    command_name: str,
    std: float,
    window_s: float | None = None,
) -> torch.Tensor:
    command = _command(env, command_name)
    error = torch.sum(torch.square(command.racket_vel_w - command.racket_target_vel_w), dim=-1)
    return torch.exp(-error / std**2) * _strike_window(command, window_s)


def hitter_racket_orientation_error_exp(
    env: "ManagerBasedRLEnv",
    command_name: str,
    std: float,
    use_abs_normal_alignment: bool = True,
    window_s: float | None = None,
) -> torch.Tensor:
    """让当前击球方式对应的物理球拍面法向对齐指令中的击球速度方向。

    在 HITTER G1 资产中，这个法向来自真实的 ``right_racket_link``：
    配置的局部 -Y 是正手面法向，反手使用相反的 +Y 面。目标法向由期望球拍速度推断得到。
    如果球拍两面等价，可以忽略法向正负号；需要指定击球面时则保留符号。
    """

    command = _command(env, command_name)
    target_dir = command.racket_target_vel_w / torch.norm(
        command.racket_target_vel_w, dim=-1, keepdim=True
    ).clamp_min(1.0e-6)
    alignment = torch.sum(command.racket_strike_face_normal_w * target_dir, dim=-1).clamp(min=-1.0, max=1.0)
    if bool(use_abs_normal_alignment):
        alignment = torch.abs(alignment)
    alignment_error = 1.0 - alignment
    return torch.exp(-(alignment_error**2) / std**2) * _strike_window(command, window_s)


def hitter_base_position_error_exp(
    env: "ManagerBasedRLEnv",
    command_name: str,
    std: float,
) -> torch.Tensor:
    command = _command(env, command_name)
    error = torch.sum(torch.square(command.base_target_pos_w - command.robot_anchor_pos_w), dim=-1)
    return torch.exp(-error / std**2) * _before_strike(command)


def hitter_torso_forward_lean_l2(
    env: "ManagerBasedRLEnv",
    body_cfg: SceneEntityCfg,
    command_name: str = "motion",
    target_forward_z: float = -0.10,
    roll_weight: float = 1.0,
    strike_window_s: float | None = None,
    strike_scale: float = 1.0,
) -> torch.Tensor:
    """Penalize torso roll and deviation from a small forward lean target.

    The torso local +X axis is treated as the forward direction. A negative
    world Z component means the torso is leaning forward/down slightly.
    """

    robot = env.scene[body_cfg.name]
    body_quat_w = robot.data.body_quat_w[:, body_cfg.body_ids, :]
    num_envs, num_bodies, _ = body_quat_w.shape
    device = body_quat_w.device
    dtype = body_quat_w.dtype

    forward_b = torch.zeros(num_envs, num_bodies, 3, device=device, dtype=dtype)
    forward_b[..., 0] = 1.0
    lateral_b = torch.zeros_like(forward_b)
    lateral_b[..., 1] = 1.0

    forward_w = quat_apply(body_quat_w.reshape(-1, 4), forward_b.reshape(-1, 3)).reshape(num_envs, num_bodies, 3)
    lateral_w = quat_apply(body_quat_w.reshape(-1, 4), lateral_b.reshape(-1, 3)).reshape(num_envs, num_bodies, 3)

    pitch_penalty = torch.square(forward_w[..., 2] - float(target_forward_z))
    roll_penalty = torch.square(lateral_w[..., 2])
    penalty = (pitch_penalty + float(roll_weight) * roll_penalty).mean(dim=-1)
    command = _command(env, command_name)
    return _strike_scaled_penalty(penalty, command, strike_window_s, strike_scale)


def hitter_base_ang_vel_xy_l2(
    env: "ManagerBasedRLEnv",
    asset_cfg: SceneEntityCfg,
    command_name: str = "motion",
    strike_window_s: float | None = None,
    strike_scale: float = 1.0,
) -> torch.Tensor:
    """Penalize root roll/pitch angular velocity while leaving yaw free for striking."""

    robot = env.scene[asset_cfg.name]
    penalty = torch.sum(torch.square(robot.data.root_ang_vel_b[:, :2]), dim=-1)
    command = _command(env, command_name)
    return _strike_scaled_penalty(penalty, command, strike_window_s, strike_scale)


def hitter_foot_slip_l2(
    env: "ManagerBasedRLEnv",
    sensor_cfg: SceneEntityCfg,
    asset_cfg: SceneEntityCfg,
    threshold: float = 1.0,
) -> torch.Tensor:
    """Penalize ankle-roll foot horizontal slip while recent contact is detected."""

    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    robot = env.scene[asset_cfg.name]
    contact_forces = contact_sensor.data.net_forces_w_history[:, :, sensor_cfg.body_ids, :]
    in_contact = torch.norm(contact_forces, dim=-1).max(dim=1)[0] > float(threshold)
    foot_xy_vel = robot.data.body_lin_vel_w[:, asset_cfg.body_ids, :2]
    slip = torch.norm(foot_xy_vel, dim=-1) * in_contact.to(foot_xy_vel.dtype)
    return torch.sum(slip, dim=-1)


def hitter_hit_unstable_support(
    env: "ManagerBasedRLEnv",
    sensor_cfg: SceneEntityCfg,
    command_name: str = "motion",
    window_s: float = 0.08,
    force_threshold: float = 1.0,
    single_stance_penalty: float = 1.0,
    both_air_penalty: float = 1.5,
    imbalance_weight: float = 0.5,
    max_support_force_ratio: float = 0.85,
) -> torch.Tensor:
    """Penalize unstable foot support during the strike window.

    Adapted from TTRL's hit-time support penalty. TTRL gates this term with
    ball-paddle contact; HITTER has no ball contact in this planner-domain
    setup, so the term is gated by ``signed_time_to_strike`` instead.
    """

    command = _command(env, command_name)
    strike_mask = _strike_window(command, window_s)

    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    contact_forces = contact_sensor.data.net_forces_w_history[:, :, sensor_cfg.body_ids, :]
    force_norm = torch.norm(contact_forces, dim=-1).max(dim=1)[0]
    in_contact = force_norm > float(force_threshold)
    num_contacts = torch.sum(in_contact.to(torch.int32), dim=1)

    single_stance = num_contacts == 1
    both_air = num_contacts == 0
    penalty = (
        float(single_stance_penalty) * single_stance.to(torch.float32)
        + float(both_air_penalty) * both_air.to(torch.float32)
    )

    if float(imbalance_weight) > 0.0:
        vertical_force = torch.abs(contact_forces[..., 2]).max(dim=1)[0]
        contact_vertical_force = vertical_force * in_contact.to(vertical_force.dtype)
        total_vertical_force = torch.sum(contact_vertical_force, dim=1).clamp_min(1.0e-6)
        max_force_ratio = torch.max(contact_vertical_force, dim=1)[0] / total_vertical_force
        ratio_range = max(1.0 - float(max_support_force_ratio), 1.0e-6)
        imbalance = torch.clamp((max_force_ratio - float(max_support_force_ratio)) / ratio_range, min=0.0, max=1.0)
        double_support = num_contacts >= 2
        penalty = penalty + float(imbalance_weight) * imbalance * double_support.to(torch.float32)

    return penalty * strike_mask


def hitter_foot_edge_drag(
    env: "ManagerBasedRLEnv",
    asset_cfg: SceneEntityCfg,
    pelvis_cfg: SceneEntityCfg,
    min_clearance: float = 0.025,
    ground_z: float = 0.0,
    sole_radius: float = 0.0,
    speed_deadzone: float = 0.05,
    lateral_weight: float = 1.0,
) -> torch.Tensor:
    """Penalize near-ground foot-edge lateral dragging in the pelvis-yaw frame."""

    robot = env.scene[asset_cfg.name]
    foot_pos_w = robot.data.body_pos_w[:, asset_cfg.body_ids, :]
    foot_quat_w = robot.data.body_quat_w[:, asset_cfg.body_ids, :]
    foot_lin_vel_w = robot.data.body_lin_vel_w[:, asset_cfg.body_ids, :]
    foot_ang_vel_w = robot.data.body_ang_vel_w[:, asset_cfg.body_ids, :]

    num_envs, num_feet, _ = foot_pos_w.shape
    device = foot_pos_w.device
    dtype = foot_pos_w.dtype

    offsets_b = torch.tensor(G1_SOLE_EDGE_OFFSETS, device=device, dtype=dtype).view(1, 1, -1, 3)
    num_points = offsets_b.shape[2]
    offsets_b = offsets_b.expand(num_envs, num_feet, num_points, 3)
    quat = foot_quat_w.unsqueeze(2).expand(num_envs, num_feet, num_points, 4)

    offsets_w = quat_apply(
        quat.reshape(-1, 4),
        offsets_b.reshape(-1, 3),
    ).reshape(num_envs, num_feet, num_points, 3)

    edge_pos_w = foot_pos_w.unsqueeze(2) + offsets_w
    edge_vel_w = foot_lin_vel_w.unsqueeze(2) + torch.cross(
        foot_ang_vel_w.unsqueeze(2).expand_as(offsets_w),
        offsets_w,
        dim=-1,
    )

    edge_surface_height = edge_pos_w[..., 2] - float(ground_z) - float(sole_radius)
    low_clearance = torch.clamp(
        (float(min_clearance) - edge_surface_height) / float(min_clearance),
        min=0.0,
        max=1.0,
    )

    pelvis_quat_w = robot.data.body_quat_w[:, pelvis_cfg.body_ids[0], :]
    pelvis_yaw_quat_w = yaw_quat(pelvis_quat_w)
    pelvis_yaw_quat_w = pelvis_yaw_quat_w.view(num_envs, 1, 1, 4).expand(num_envs, num_feet, num_points, 4)
    edge_vel_pelvis_yaw = quat_rotate_inverse(
        pelvis_yaw_quat_w.reshape(-1, 4),
        edge_vel_w.reshape(-1, 3),
    ).reshape(num_envs, num_feet, num_points, 3)

    lateral_speed = torch.abs(edge_vel_pelvis_yaw[..., 1])
    lateral_drag_speed = float(lateral_weight) * torch.clamp(lateral_speed - float(speed_deadzone), min=0.0)
    penalty = low_clearance.square() * lateral_drag_speed

    return torch.mean(penalty, dim=2).sum(dim=1)
