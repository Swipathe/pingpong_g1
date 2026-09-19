from __future__ import annotations

from collections.abc import Sequence
from dataclasses import MISSING

import isaaclab.sim as sim_utils
import torch

from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import (
    quat_apply,
    quat_from_euler_xyz,
    quat_inv,
    quat_mul,
    quat_rotate_inverse,
    sample_uniform,
    yaw_quat,
)

from whole_body_tracking.assets import ASSET_DIR
from whole_body_tracking.tasks.tracking.mdp.commands import MultiMotionCommand, MultiMotionCommandCfg


class HitterStrikingCommand(MultiMotionCommand):
    """用于 HITTER 风格 WBC 训练的目标条件击球指令。"""

    cfg: "HitterStrikingCommandCfg"

    def __init__(self, cfg: "HitterStrikingCommandCfg", env):
        super().__init__(cfg, env)
        self._normalize_motion_reference_root_frame()

        self.racket_body_index = self.robot.body_names.index(self.cfg.racket_body_name)
        self.racket_pos_offset_in_body_frame = torch.tensor(
            self.cfg.racket_pos_offset_in_body_frame,
            dtype=torch.float32,
            device=self.device,
        )
        self.racket_normal_axis_in_body_frame = torch.tensor(
            self.cfg.racket_normal_axis_in_body_frame,
            dtype=torch.float32,
            device=self.device,
        )
        normal_norm = torch.norm(self.racket_normal_axis_in_body_frame).clamp_min(1.0e-6)
        self.racket_normal_axis_in_body_frame = self.racket_normal_axis_in_body_frame / normal_norm

        self.strike_type = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self.elapsed_s = torch.zeros(self.num_envs, dtype=torch.float32, device=self.device)
        self.strike_time_s = torch.full((self.num_envs,), 0.86, dtype=torch.float32, device=self.device)
        self.strike_duration_s = torch.full((self.num_envs,), 1.88, dtype=torch.float32, device=self.device)
        self.signed_time_to_strike = self.strike_time_s.clone()
        self.time_to_strike_s = torch.clamp(self.signed_time_to_strike, min=0.0)

        self.base_target_origin_xy_w = torch.zeros(self.num_envs, 2, dtype=torch.float32, device=self.device)
        self.base_target_xy_w = torch.zeros(self.num_envs, 2, dtype=torch.float32, device=self.device)
        self.racket_target_pos_b = torch.zeros(self.num_envs, 3, dtype=torch.float32, device=self.device)
        self.racket_target_pos_w_fixed = torch.zeros(self.num_envs, 3, dtype=torch.float32, device=self.device)
        self.racket_target_vel_w = torch.zeros(self.num_envs, 3, dtype=torch.float32, device=self.device)

        self.metrics["hitter_strike_racket_pos_error"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["hitter_strike_racket_vel_error"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["hitter_strike_racket_ori_error"] = torch.zeros(self.num_envs, device=self.device)
        self._strike_metric_count = torch.zeros(self.num_envs, device=self.device)
        self._strike_metric_pos_error_sum = torch.zeros(self.num_envs, device=self.device)
        self._strike_metric_vel_error_sum = torch.zeros(self.num_envs, device=self.device)
        self._strike_metric_ori_error_sum = torch.zeros(self.num_envs, device=self.device)

    def _normalize_motion_reference_root_frame(self) -> None:
        if not bool(self.cfg.normalize_reference_root_xy_yaw):
            return

        loader = self.motion_dir_loader
        if len(loader) == 0:
            return

        storage_device = loader.body_pos_w.device
        offsets = loader.motion_offsets.to(device=storage_device, dtype=torch.long)
        lengths = loader.motion_lengths.to(device=storage_device, dtype=torch.long)
        anchor_idx = int(self.motion_anchor_body_index)

        root_pos0 = loader.body_pos_w.index_select(0, offsets)[:, anchor_idx].clone()
        root_quat0 = loader.body_quat_w.index_select(0, offsets)[:, anchor_idx].clone()
        yaw_inv0 = quat_inv(yaw_quat(root_quat0))

        for motion_i in range(len(loader)):
            start = int(offsets[motion_i].item())
            length = int(lengths[motion_i].item())
            if length <= 0:
                continue
            end = start + length
            heading_inv = yaw_inv0[motion_i]
            origin_xy = root_pos0[motion_i, :2]

            body_pos = loader.body_pos_w[start:end]
            body_pos_shifted = body_pos.clone()
            body_pos_shifted[..., :2] = body_pos_shifted[..., :2] - origin_xy.view(1, 1, 2)
            heading_pos = heading_inv.view(1, 1, 4).expand_as(loader.body_quat_w[start:end])
            loader.body_pos_w[start:end] = quat_apply(
                heading_pos.reshape(-1, 4),
                body_pos_shifted.reshape(-1, 3),
            ).view_as(body_pos)

            body_quat = loader.body_quat_w[start:end]
            loader.body_quat_w[start:end] = quat_mul(
                heading_pos.reshape(-1, 4),
                body_quat.reshape(-1, 4),
            ).view_as(body_quat)

            heading_vel = heading_inv.view(1, 1, 4).expand_as(body_quat)
            body_lin_vel = loader.body_lin_vel_w[start:end]
            loader.body_lin_vel_w[start:end] = quat_apply(
                heading_vel.reshape(-1, 4),
                body_lin_vel.reshape(-1, 3),
            ).view_as(body_lin_vel)

            body_ang_vel = loader.body_ang_vel_w[start:end]
            loader.body_ang_vel_w[start:end] = quat_apply(
                heading_vel.reshape(-1, 4),
                body_ang_vel.reshape(-1, 3),
            ).view_as(body_ang_vel)

    @property
    def command(self) -> torch.Tensor:
        if not self.cfg.enable_strike_targets:
            return torch.zeros(self.num_envs, 10, dtype=torch.float32, device=self.device)
        return torch.cat(
            [
                self.base_target_pos_b,
                self.racket_target_pos_b,
                self.racket_target_vel_w,
                self.time_to_strike_s.unsqueeze(-1),
            ],
            dim=-1,
        )

    def _set_metric(self, name: str, value: torch.Tensor) -> None:
        metric = self.metrics.get(name)
        if metric is not None:
            metric.copy_(value)

    def _reset_strike_metric_accumulators(self, env_ids: torch.Tensor) -> None:
        self._strike_metric_count[env_ids] = 0.0
        self._strike_metric_pos_error_sum[env_ids] = 0.0
        self._strike_metric_vel_error_sum[env_ids] = 0.0
        self._strike_metric_ori_error_sum[env_ids] = 0.0

    def _racket_orientation_error_from_normal(self, normal_w: torch.Tensor, target_dir_w: torch.Tensor) -> torch.Tensor:
        alignment = torch.sum(normal_w * target_dir_w, dim=-1).clamp(min=-1.0, max=1.0)
        if bool(self.cfg.racket_orientation_use_abs_normal_alignment):
            alignment = torch.abs(alignment)
        return 1.0 - alignment

    def _quat_from_negative_y_axis_to_vector(self, vector: torch.Tensor) -> torch.Tensor:
        vector = vector / torch.norm(vector, dim=-1, keepdim=True).clamp_min(1.0e-6)
        source = torch.zeros_like(vector)
        source[:, 1] = -1.0
        dot = torch.sum(source * vector, dim=-1).clamp(min=-1.0, max=1.0)
        quat = torch.cat([(1.0 + dot).unsqueeze(-1), torch.cross(source, vector, dim=-1)], dim=-1)
        near_opposite = dot < -0.9999
        if torch.any(near_opposite):
            quat[near_opposite] = torch.tensor([0.0, 1.0, 0.0, 0.0], dtype=torch.float32, device=self.device)
        return quat / torch.norm(quat, dim=-1, keepdim=True).clamp_min(1.0e-6)

    @property
    def racket_pos_w(self) -> torch.Tensor:
        body_pos = self.robot.data.body_pos_w[:, self.racket_body_index]
        body_quat = self.robot.data.body_quat_w[:, self.racket_body_index]
        offset = self.racket_pos_offset_in_body_frame.unsqueeze(0).expand(self.num_envs, -1)
        return body_pos + quat_apply(body_quat, offset)

    @property
    def racket_vel_w(self) -> torch.Tensor:
        body_quat = self.robot.data.body_quat_w[:, self.racket_body_index]
        offset = self.racket_pos_offset_in_body_frame.unsqueeze(0).expand(self.num_envs, -1)
        offset_w = quat_apply(body_quat, offset)
        lin_vel = self.robot.data.body_lin_vel_w[:, self.racket_body_index]
        ang_vel = self.robot.data.body_ang_vel_w[:, self.racket_body_index]
        return lin_vel + torch.cross(ang_vel, offset_w, dim=-1)

    @property
    def racket_normal_w(self) -> torch.Tensor:
        body_quat = self.robot.data.body_quat_w[:, self.racket_body_index]
        normal = self.racket_normal_axis_in_body_frame.unsqueeze(0).expand(self.num_envs, -1)
        return quat_apply(body_quat, normal)

    @property
    def racket_strike_face_normal_w(self) -> torch.Tensor:
        """返回当前击球类型应该使用的拍面法向。"""

        face_sign = torch.where(
            self.strike_type == 0,
            torch.ones_like(self.strike_type, dtype=torch.float32),
            -torch.ones_like(self.strike_type, dtype=torch.float32),
        ).unsqueeze(-1)
        return self.racket_normal_w * face_sign

    @property
    def racket_pos_b(self) -> torch.Tensor:
        anchor_yaw = yaw_quat(self.robot_anchor_quat_w)
        return quat_rotate_inverse(anchor_yaw, self.racket_pos_w - self.robot_anchor_pos_w)

    @property
    def base_target_xy_b(self) -> torch.Tensor:
        return self.base_target_pos_b[:, :2]

    @property
    def base_target_pos_w(self) -> torch.Tensor:
        return self._target_base_pos_w_from_xy(self.base_target_xy_w)

    @property
    def base_target_pos_b(self) -> torch.Tensor:
        delta_xy_w = self.base_target_xy_w - self.robot_anchor_pos_w[:, :2]
        delta_w = torch.cat(
            [
                delta_xy_w,
                torch.full(
                    (self.num_envs, 1),
                    float(self.cfg.target_base_height_w),
                    dtype=torch.float32,
                    device=self.device,
                )
                - self.robot_anchor_pos_w[:, 2:3],
            ],
            dim=-1,
        )
        return quat_rotate_inverse(yaw_quat(self.robot_anchor_quat_w), delta_w)

    @property
    def racket_target_pos_w(self) -> torch.Tensor:
        if self._uses_fixed_world_racket_target:
            return self.racket_target_pos_w_fixed
        anchor_yaw = yaw_quat(self.robot_anchor_quat_w)
        return self.robot_anchor_pos_w + quat_apply(anchor_yaw, self.racket_target_pos_b)

    @property
    def _uses_fixed_world_racket_target(self) -> bool:
        frame = str(self.cfg.racket_target_position_frame).lower()
        return frame in {"world", "world_fixed", "fixed_world"}

    def _as_env_ids(self, env_ids: Sequence[int] | torch.Tensor) -> torch.Tensor:
        if isinstance(env_ids, slice):
            return torch.arange(self.num_envs, device=self.device, dtype=torch.long)
        return torch.as_tensor(env_ids, device=self.device, dtype=torch.long)

    def _uniform(self, value_range: tuple[float, float], shape: tuple[int, ...]) -> torch.Tensor:
        low, high = float(value_range[0]), float(value_range[1])
        if high < low:
            raise ValueError(f"Invalid range {value_range}: upper bound is smaller than lower bound.")
        return low + (high - low) * torch.rand(shape, device=self.device, dtype=torch.float32)

    def _target_base_pos_w_from_xy(self, base_xy_w: torch.Tensor) -> torch.Tensor:
        z = torch.full(
            (base_xy_w.shape[0], 1),
            float(self.cfg.target_base_height_w),
            dtype=torch.float32,
            device=self.device,
        )
        return torch.cat([base_xy_w, z], dim=-1)

    def _sample_reference_motions_for_strike(self, env_ids: torch.Tensor) -> None:
        if env_ids.numel() == 0:
            return

        if not {"forehand", "backhand"}.issubset(self.group_name_to_idx.keys()):
            self._assign_motions(env_ids)
            return

        probs_motion_global = self._compute_motion_sampling_probs()
        forehand_envs = env_ids[self.strike_type[env_ids] == 0]
        backhand_envs = env_ids[self.strike_type[env_ids] == 1]
        if forehand_envs.numel() > 0:
            self._assign_motions_for_group("forehand", forehand_envs, probs_motion_global)
        if backhand_envs.numel() > 0:
            self._assign_motions_for_group("backhand", backhand_envs, probs_motion_global)

    def _store_fixed_world_racket_targets(
        self,
        env_ids: torch.Tensor,
        *,
        target_base_pos_w: torch.Tensor | None = None,
    ) -> None:
        if env_ids.numel() == 0 or not self._uses_fixed_world_racket_target:
            return
        if target_base_pos_w is None:
            target_base_pos_w = self._target_base_pos_w_from_xy(self.robot_anchor_pos_w[env_ids, :2])
        anchor_yaw = yaw_quat(self.robot_anchor_quat_w[env_ids])
        self.racket_target_pos_w_fixed[env_ids] = target_base_pos_w + quat_apply(
            anchor_yaw,
            self.racket_target_pos_b[env_ids],
        )
        self._refresh_racket_target_pos_b_from_fixed_world(env_ids)

    def _refresh_racket_target_pos_b_from_fixed_world(self, env_ids: torch.Tensor | None = None) -> None:
        if not self._uses_fixed_world_racket_target:
            return
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device, dtype=torch.long)
        if env_ids.numel() == 0:
            return
        anchor_yaw = yaw_quat(self.robot_anchor_quat_w[env_ids])
        self.racket_target_pos_b[env_ids] = quat_rotate_inverse(
            anchor_yaw,
            self.racket_target_pos_w_fixed[env_ids] - self.robot_anchor_pos_w[env_ids],
        )

    def _align_reference_phase_to_strike_time(self, env_ids: torch.Tensor) -> None:
        """平移风格参考动作相位，使其击球帧对齐规划接触时刻。"""

        if env_ids.numel() == 0 or not self.cfg.align_reference_phase_to_strike_time:
            return
        reference_strike_frame = int(max(self.cfg.reference_strike_frame, 0))
        frames_to_contact = torch.round(self.strike_time_s[env_ids] / float(self.sim_dt)).to(torch.long)
        start_frame = reference_strike_frame - frames_to_contact
        start_frame = torch.clamp(start_frame, min=0)
        max_frame = self.motion_lengths_minus_one[self.env_motion_indices[env_ids]]
        self.time_steps[env_ids] = torch.minimum(start_frame, max_frame)

    def _sample_strike_timing(self, env_ids: torch.Tensor) -> None:
        count = env_ids.numel()
        if count == 0:
            return
        self.elapsed_s[env_ids] = 0.0
        self.strike_time_s[env_ids] = self._uniform(self.cfg.time_to_strike_range, (count,))
        self.strike_duration_s[env_ids] = self._uniform(self.cfg.swing_duration_range, (count,))

    def _reset_robot_state_from_current_reference(self, env_ids: torch.Tensor) -> None:
        """根据当前选中的参考帧写入机器人状态。"""

        if env_ids.numel() == 0:
            return
        body_pos = self._gather_by_motion_for_envs("body_pos_w", env_ids)
        body_quat = self._gather_by_motion_for_envs("body_quat_w", env_ids)
        body_lin = self._gather_by_motion_for_envs("body_lin_vel_w", env_ids)
        body_ang = self._gather_by_motion_for_envs("body_ang_vel_w", env_ids)
        joint_pos = self._gather_by_motion_for_envs("joint_pos", env_ids).clone()
        joint_vel = self._gather_by_motion_for_envs("joint_vel", env_ids).clone()

        root_pos = body_pos[:, 0] + self._env.scene.env_origins[env_ids]
        root_ori = body_quat[:, 0]
        root_lin_vel = body_lin[:, 0].clone()
        root_ang_vel = body_ang[:, 0].clone()

        pose_noise = sample_uniform(
            self._pose_ranges[:, 0],
            self._pose_ranges[:, 1],
            (len(env_ids), 6),
            device=self.device,
        )
        root_pos += pose_noise[:, 0:3]
        root_ori = quat_mul(
            quat_from_euler_xyz(pose_noise[:, 3], pose_noise[:, 4], pose_noise[:, 5]),
            root_ori,
        )

        vel_noise = sample_uniform(
            self._vel_ranges[:, 0],
            self._vel_ranges[:, 1],
            (len(env_ids), 6),
            device=self.device,
        )
        root_lin_vel += vel_noise[:, :3]
        root_ang_vel += vel_noise[:, 3:]

        joint_pos += sample_uniform(*self.cfg.joint_position_range, joint_pos.shape, joint_pos.device)
        soft_joint_pos_limits = self.robot.data.soft_joint_pos_limits[env_ids]
        joint_pos = torch.clip(joint_pos, soft_joint_pos_limits[:, :, 0], soft_joint_pos_limits[:, :, 1])

        self.robot.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)
        self.robot.write_root_state_to_sim(
            torch.cat([root_pos, root_ori, root_lin_vel, root_ang_vel], dim=-1),
            env_ids=env_ids,
        )

    def _sample_strike_targets(self, env_ids: torch.Tensor, base_xy_w: torch.Tensor) -> None:
        count = env_ids.numel()
        if count == 0:
            return

        forehand = self.strike_type[env_ids] == 0
        y_forehand = self._uniform(self.cfg.forehand_racket_y_range, (count,))
        y_backhand = self._uniform(self.cfg.backhand_racket_y_range, (count,))
        desired_racket_y_b = torch.where(forehand, y_forehand, y_backhand)
        forehand_offset_range = self.cfg.forehand_racket_y_offset_range
        if forehand_offset_range is None:
            forehand_offset_range = (
                float(self.cfg.forehand_nominal_racket_y_b),
                float(self.cfg.forehand_nominal_racket_y_b),
            )
        backhand_offset_range = self.cfg.backhand_racket_y_offset_range
        if backhand_offset_range is None:
            backhand_offset_range = (
                float(self.cfg.backhand_nominal_racket_y_b),
                float(self.cfg.backhand_nominal_racket_y_b),
            )
        y_offset_forehand = self._uniform(forehand_offset_range, (count,))
        y_offset_backhand = self._uniform(backhand_offset_range, (count,))
        racket_y_offset_b = torch.where(
            forehand,
            y_offset_forehand,
            y_offset_backhand,
        )

        command_origin_xy_w = self.base_target_origin_xy_w[env_ids] if self._uses_fixed_world_racket_target else base_xy_w
        base_x = torch.full((count,), float(self.cfg.base_target_x_offset), dtype=torch.float32, device=self.device)
        base_y = desired_racket_y_b - racket_y_offset_b
        self.base_target_xy_w[env_ids] = command_origin_xy_w + torch.stack([base_x, base_y], dim=-1)

        z = self._uniform(self.cfg.racket_z_range, (count,))
        if self._uses_fixed_world_racket_target:
            # 训练侧保留历史字段名 strike_plane_x；这里实际表示球拍相对 base 的前向偏移。
            # 部署侧 planner 会在物理虚拟击球平面上计算真正的世界系击球点。
            self.racket_target_pos_w_fixed[env_ids, 0] = command_origin_xy_w[:, 0] + float(self.cfg.strike_plane_x)
            self.racket_target_pos_w_fixed[env_ids, 1] = command_origin_xy_w[:, 1] + desired_racket_y_b
            self.racket_target_pos_w_fixed[env_ids, 2] = float(self.cfg.target_base_height_w) + z
            self._refresh_racket_target_pos_b_from_fixed_world(env_ids)
        else:
            self.racket_target_pos_b[env_ids, 0] = float(self.cfg.strike_plane_x) - base_x
            self.racket_target_pos_b[env_ids, 1] = racket_y_offset_b
            self.racket_target_pos_b[env_ids, 2] = z

        vx_forehand = self._uniform(self.cfg.forehand_racket_velocity_x_range, (count,))
        vx_backhand = self._uniform(self.cfg.backhand_racket_velocity_x_range, (count,))
        vy_forehand = self._uniform(self.cfg.forehand_racket_velocity_y_range, (count,))
        vy_backhand = self._uniform(self.cfg.backhand_racket_velocity_y_range, (count,))
        vz = self._uniform(self.cfg.racket_velocity_z_range, (count,))
        sampled_racket_velocity = torch.stack(
            [
                torch.where(forehand, vx_forehand, vx_backhand),
                torch.where(forehand, vy_forehand, vy_backhand),
                vz,
            ],
            dim=-1,
        )
        self.racket_target_vel_w[env_ids] = sampled_racket_velocity
        self._update_strike_time_buffers(env_ids)

    def _start_new_strike(self, env_ids: torch.Tensor, *, reset_robot_state: bool) -> None:
        if env_ids.numel() == 0:
            return

        forced_strike_type = self.cfg.force_strike_type
        if forced_strike_type is None:
            self.strike_type[env_ids] = torch.randint(0, 2, (env_ids.numel(),), device=self.device)
        else:
            forced_strike_type = str(forced_strike_type).lower()
            if forced_strike_type not in {"forehand", "backhand"}:
                raise ValueError(f"force_strike_type must be None, 'forehand', or 'backhand', got {forced_strike_type!r}.")
            self.strike_type[env_ids] = 0 if forced_strike_type == "forehand" else 1
        self._sample_reference_motions_for_strike(env_ids)
        self._sample_strike_timing(env_ids)
        self.time_steps[env_ids] = int(max(self.cfg.reference_start_frame, 0))
        self.motion_end_buf[env_ids] = False

        if reset_robot_state:
            super()._resample_command(env_ids)
            self._align_reference_phase_to_strike_time(env_ids)
            self._reset_robot_state_from_current_reference(env_ids)
            base_xy_w = self.anchor_pos_w[env_ids, :2]
            if self._uses_fixed_world_racket_target:
                self.base_target_origin_xy_w[env_ids] = self._env.scene.env_origins[env_ids, :2]
            else:
                self.base_target_origin_xy_w[env_ids] = base_xy_w
        else:
            self._align_reference_phase_to_strike_time(env_ids)
            base_xy_w = self.robot_anchor_pos_w[env_ids, :2]
            self._refresh_relative_motion_targets()

        self._sample_strike_targets(env_ids, base_xy_w)

    def _resample_command(self, env_ids: Sequence[int]):
        env_ids = self._as_env_ids(env_ids)
        if not self.cfg.enable_strike_targets:
            super()._resample_command(env_ids)
            self.elapsed_s[env_ids] = 0.0
            self.strike_time_s[env_ids] = 0.0
            self.strike_duration_s[env_ids] = 0.0
            self._update_strike_time_buffers(env_ids)
            self.base_target_xy_w[env_ids] = self.robot_anchor_pos_w[env_ids, :2]
            self.racket_target_pos_b[env_ids] = 0.0
            self.racket_target_pos_w_fixed[env_ids] = self.robot_anchor_pos_w[env_ids]
            self.racket_target_vel_w[env_ids] = 0.0
            return
        self._start_new_strike(env_ids, reset_robot_state=True)

    def _refresh_relative_motion_targets(self) -> None:
        anchor_pos_w_repeat = self.anchor_pos_w[:, None, :].repeat(1, len(self.cfg.body_names), 1)
        anchor_quat_w_repeat = self.anchor_quat_w[:, None, :].repeat(1, len(self.cfg.body_names), 1)
        robot_anchor_pos_w_repeat = self.robot_anchor_pos_w[:, None, :].repeat(1, len(self.cfg.body_names), 1)
        robot_anchor_quat_w_repeat = self.robot_anchor_quat_w[:, None, :].repeat(1, len(self.cfg.body_names), 1)

        delta_pos_w = robot_anchor_pos_w_repeat
        delta_pos_w[..., 2] = anchor_pos_w_repeat[..., 2]
        delta_ori_w = yaw_quat(quat_mul(robot_anchor_quat_w_repeat, quat_inv(anchor_quat_w_repeat)))
        self.body_quat_relative_w = quat_mul(delta_ori_w, self.body_quat_w)
        self.body_pos_relative_w = delta_pos_w + quat_apply(delta_ori_w, self.body_pos_w - anchor_pos_w_repeat)

    def _update_strike_time_buffers(self, env_ids: torch.Tensor | None = None) -> None:
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device, dtype=torch.long)
        self.signed_time_to_strike[env_ids] = self.strike_time_s[env_ids] - self.elapsed_s[env_ids]
        self.time_to_strike_s[env_ids] = torch.clamp(self.signed_time_to_strike[env_ids], min=0.0)

    def _update_metrics(self):
        super()._update_metrics()
        if not self.cfg.enable_strike_targets:
            zeros = torch.zeros(self.num_envs, dtype=torch.float32, device=self.device)
            self._set_metric("hitter_strike_racket_pos_error", zeros)
            self._set_metric("hitter_strike_racket_vel_error", zeros)
            self._set_metric("hitter_strike_racket_ori_error", zeros)
            return
        racket_pos_error = torch.norm(self.racket_target_pos_b - self.racket_pos_b, dim=-1)
        racket_vel_error = torch.norm(self.racket_target_vel_w - self.racket_vel_w, dim=-1)
        target_speed = torch.norm(self.racket_target_vel_w, dim=-1).clamp_min(1.0e-6)
        target_dir = self.racket_target_vel_w / target_speed.unsqueeze(-1)
        racket_ori_error = self._racket_orientation_error_from_normal(self.racket_strike_face_normal_w, target_dir)
        strike_mask = (torch.abs(self.signed_time_to_strike) <= float(self.cfg.strike_metric_window_s)).to(torch.float32)
        self._strike_metric_count += strike_mask
        self._strike_metric_pos_error_sum += racket_pos_error * strike_mask
        self._strike_metric_vel_error_sum += racket_vel_error * strike_mask
        self._strike_metric_ori_error_sum += racket_ori_error * strike_mask
        count = self._strike_metric_count.clamp_min(1.0)

        self._set_metric("hitter_strike_racket_pos_error", self._strike_metric_pos_error_sum / count)
        self._set_metric("hitter_strike_racket_vel_error", self._strike_metric_vel_error_sum / count)
        self._set_metric("hitter_strike_racket_ori_error", self._strike_metric_ori_error_sum / count)

    def reset(self, env_ids: Sequence[int] | None = None) -> dict[str, float]:
        if env_ids is None:
            reset_env_ids = torch.arange(self.num_envs, device=self.device, dtype=torch.long)
        else:
            reset_env_ids = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)
        extras = super().reset(env_ids=env_ids)
        self._reset_strike_metric_accumulators(reset_env_ids)
        return extras

    def _update_command(self):
        if not self.cfg.enable_strike_targets:
            super()._update_command()
            return
        super()._update_command()
        self.elapsed_s += self.sim_dt
        self._update_strike_time_buffers()

        rollover_envs = torch.where(self.elapsed_s >= self.strike_duration_s)[0]
        if rollover_envs.numel() > 0:
            self._start_new_strike(rollover_envs, reset_robot_state=False)
            self._refresh_relative_motion_targets()
        self._refresh_racket_target_pos_b_from_fixed_world()

    def _set_debug_vis_impl(self, debug_vis: bool):
        if not self.cfg.enable_strike_targets:
            debug_vis = False
        if debug_vis:
            if not hasattr(self, "base_target_visualizer"):
                self.base_target_visualizer = VisualizationMarkers(self.cfg.base_target_visualizer_cfg)
                self.racket_target_visualizer = VisualizationMarkers(self.cfg.racket_target_visualizer_cfg)
            self.base_target_visualizer.set_visibility(True)
            self.racket_target_visualizer.set_visibility(True)
        else:
            if hasattr(self, "base_target_visualizer"):
                self.base_target_visualizer.set_visibility(False)
                self.racket_target_visualizer.set_visibility(False)

    def _debug_vis_callback(self, event):
        if not self.robot.is_initialized or not self.cfg.enable_strike_targets:
            return
        if not hasattr(self, "base_target_visualizer") or not hasattr(self, "racket_target_visualizer"):
            return

        racket_target_dir_w = self.racket_target_vel_w / torch.norm(
            self.racket_target_vel_w, dim=-1, keepdim=True
        ).clamp_min(1.0e-6)
        racket_target_marker_dir_w = torch.where(
            (self.strike_type == 0).unsqueeze(-1),
            racket_target_dir_w,
            -racket_target_dir_w,
        )
        racket_target_quat_w = self._quat_from_negative_y_axis_to_vector(racket_target_marker_dir_w)
        base_target_quat_w = torch.zeros(self.num_envs, 4, dtype=torch.float32, device=self.device)
        base_target_quat_w[:, 0] = 1.0
        self.base_target_visualizer.visualize(
            translations=self.base_target_pos_w,
            orientations=base_target_quat_w,
        )
        self.racket_target_visualizer.visualize(
            translations=self.racket_target_pos_w,
            orientations=racket_target_quat_w,
        )


@configclass
class HitterStrikingCommandCfg(MultiMotionCommandCfg):
    """HITTER 目标条件击球指令配置。"""

    class_type: type = HitterStrikingCommand

    asset_name: str = MISSING
    motion: str = MISSING
    anchor_body_name: str = MISSING
    body_names: list[str] = MISSING

    racket_body_name: str = "right_racket_link"
    # HITTER G1 资产把 right_racket_link 作为 right_wrist_yaw_link 的固定子 link。
    # 球拍 link 原点是拍面中心；局部 -Y 是拍面法向，局部 X 是手柄/拍面平面方向。
    racket_pos_offset_in_body_frame: tuple[float, float, float] = (0.0, 0.0, 0.0)
    racket_normal_axis_in_body_frame: tuple[float, float, float] = (0.0, -1.0, 0.0)
    racket_target_position_frame: str = "world_fixed"
    normalize_reference_root_xy_yaw: bool = True
    target_base_height_w: float = 0.793
    racket_radius: float = 0.075

    strike_plane_x: float = 0.40
    time_to_strike_range: tuple[float, float] = (0.80, 0.92)
    swing_duration_range: tuple[float, float] = (1.75, 1.95)
    reference_start_frame: int = 0
    reference_strike_frame: int = 43
    align_reference_phase_to_strike_time: bool = False

    base_target_x_offset: float = 0.0
    forehand_nominal_racket_y_b: float = -0.5
    backhand_nominal_racket_y_b: float = 0.22
    forehand_racket_y_offset_range: tuple[float, float] | None = None
    backhand_racket_y_offset_range: tuple[float, float] | None = None
    forehand_racket_y_range: tuple[float, float] = (-0.7625, 0.0)
    backhand_racket_y_range: tuple[float, float] = (0.0, 0.7625)
    racket_z_range: tuple[float, float] = (0.00, 0.50)

    forehand_racket_velocity_x_range: tuple[float, float] = (2.6, 3.4)
    backhand_racket_velocity_x_range: tuple[float, float] = (2.6, 3.4)
    forehand_racket_velocity_y_range: tuple[float, float] = (-0.4, 1.6)
    backhand_racket_velocity_y_range: tuple[float, float] = (-1.6, 0.4)
    racket_velocity_z_range: tuple[float, float] = (-0.3, 2.7)
    racket_orientation_use_abs_normal_alignment: bool = False
    strike_metric_window_s: float = 0.01
    force_strike_type: str | None = None
    physical_table_width: float = 1.525
    enable_strike_targets: bool = True

    base_target_visualizer_cfg: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/Command/hitter/base_target",
        markers={
            "pelvis_target": sim_utils.UrdfFileCfg(
                asset_path=f"{ASSET_DIR}/unitree_description/urdf/g1/pelvis_target_marker.urdf",
                fix_base=True,
                make_instanceable=False,
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0)),
                joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
                    gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0, damping=0)
                ),
            ),
        },
    )
    racket_target_visualizer_cfg: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/Command/hitter/racket_target",
        markers={
            "racket_target": sim_utils.UsdFileCfg(
                usd_path=f"{ASSET_DIR}/unitree_description/usd/g1_hitter_racket/target_marker.usda",
            ),
        },
    )

    motion_groups: dict[str, list[str]] | None = {"forehand": ["forehand"], "backhand": ["backhand"]}
    motion_group_sampling_ratios: dict[str, float] | None = {"forehand": 0.5, "backhand": 0.5}
    start_from_beginning: bool = True
    motion_sampling_warmup_s: float = 1000000000.0
    motion_sampling_ramp_s: float = 1000000000.0
    resample_motions_every_s: float = 1000000000.0
