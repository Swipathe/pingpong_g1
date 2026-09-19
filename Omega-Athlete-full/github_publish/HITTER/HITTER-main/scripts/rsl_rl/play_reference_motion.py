"""Replay reference motion directly in Isaac Sim.

This script visualizes the motion command itself: no checkpoint is loaded and no
policy actions are used to generate the pose.
"""

import argparse
import os
import sys
import time

from isaaclab.app import AppLauncher


DEFAULT_TASK = "Hitter-Striking-PlannerDomain-Flat-G1-v0"
DEFAULT_MOTION = "motions"


parser = argparse.ArgumentParser(description="Replay reference motion in Isaac Sim.")
parser.add_argument("--task", type=str, default=DEFAULT_TASK, help="Isaac Lab task id.")
parser.add_argument("--motion", type=str, default=DEFAULT_MOTION, help="Path to a motion file or motion directory.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to show.")
parser.add_argument("--motion_index", type=int, default=0, help="Motion index to replay.")
parser.add_argument("--show_all", action="store_true", help="Assign consecutive motions across visible environments.")
parser.add_argument("--start_frame", type=int, default=0, help="Reference start frame.")
parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier.")
parser.add_argument("--no_loop", action="store_true", default=False, help="Stop instead of looping at motion end.")
parser.add_argument("--video", action="store_true", default=False, help="Record the replay to an mp4.")
parser.add_argument("--video_length", type=int, default=160, help="Length of the recorded video in env steps.")
parser.add_argument(
    "--video_folder",
    type=str,
    default="outputs/hitter_closeup",
    help="Directory for recorded videos.",
)
parser.add_argument(
    "--camera_body",
    type=str,
    default="right_racket_link",
    help="Robot body for the viewport camera to track.",
)
parser.add_argument(
    "--camera_eye",
    type=float,
    nargs=3,
    default=(0.38, -0.36, 0.18),
    metavar=("X", "Y", "Z"),
    help="Camera eye offset from camera_body, in meters.",
)
parser.add_argument(
    "--camera_lookat",
    type=float,
    nargs=3,
    default=(0.02, 0.0, 0.0),
    metavar=("X", "Y", "Z"),
    help="Camera look-at offset from camera_body, in meters.",
)
parser.add_argument(
    "--camera_resolution",
    type=int,
    nargs=2,
    default=(1280, 720),
    metavar=("WIDTH", "HEIGHT"),
    help="Recorded frame resolution.",
)
parser.add_argument("--disable_events", action="store_true", default=True, help="Disable reset/interval events.")
parser.add_argument("--disable_motion_randomization", action="store_true", default=True, help="Zero motion noise.")
parser.add_argument(
    "--disable_motion_group_sampling",
    action="store_true",
    default=True,
    help="Use the loaded motion index directly instead of group-ratio sampling.",
)
parser.add_argument("--print_every", type=int, default=50, help="Print frame status every N sim steps.")
parser.add_argument("--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O.")
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
if args_cli.video:
    args_cli.enable_cameras = True

sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

from isaaclab.envs import DirectMARLEnv, DirectMARLEnvCfg, DirectRLEnvCfg, ManagerBasedRLEnvCfg  # noqa: E402
from isaaclab_tasks.utils.hydra import hydra_task_config  # noqa: E402
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg  # noqa: E402

import whole_body_tracking.tasks  # noqa: E402,F401


def _zero_motion_randomization(motion_cfg):
    zero_ranges = {
        "x": (0.0, 0.0),
        "y": (0.0, 0.0),
        "z": (0.0, 0.0),
        "roll": (0.0, 0.0),
        "pitch": (0.0, 0.0),
        "yaw": (0.0, 0.0),
    }
    if hasattr(motion_cfg, "pose_range"):
        motion_cfg.pose_range = dict(zero_ranges)
    if hasattr(motion_cfg, "velocity_range"):
        motion_cfg.velocity_range = dict(zero_ranges)
    if hasattr(motion_cfg, "joint_position_range"):
        motion_cfg.joint_position_range = (0.0, 0.0)


def _configure_env(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg):
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.viewer.origin_type = "asset_body"
    env_cfg.viewer.asset_name = "robot"
    env_cfg.viewer.body_name = args_cli.camera_body
    env_cfg.viewer.eye = tuple(float(x) for x in args_cli.camera_eye)
    env_cfg.viewer.lookat = tuple(float(x) for x in args_cli.camera_lookat)
    env_cfg.viewer.resolution = tuple(int(x) for x in args_cli.camera_resolution)

    motion_cfg = env_cfg.commands.motion
    motion_cfg.motion = args_cli.motion
    motion_cfg.debug_vis = False
    if hasattr(motion_cfg, "enable_strike_targets"):
        motion_cfg.enable_strike_targets = False
    motion_cfg.start_from_beginning = True
    motion_cfg.start_frame = max(int(args_cli.start_frame), 0)
    if hasattr(motion_cfg, "reference_start_frame"):
        motion_cfg.reference_start_frame = max(int(args_cli.start_frame), 0)
    if hasattr(motion_cfg, "resample_motions_every_s"):
        motion_cfg.resample_motions_every_s = 1.0e9
    if args_cli.disable_motion_group_sampling and hasattr(motion_cfg, "motion_group_sampling_ratios"):
        motion_cfg.motion_group_sampling_ratios = None
    motion_groups = getattr(motion_cfg, "motion_groups", None)
    if args_cli.show_all and hasattr(motion_cfg, "motion_dataset_load_cap"):
        group_count = len(motion_groups) if motion_groups is not None else 0
        motion_cfg.motion_dataset_load_cap = max(int(args_cli.num_envs), group_count)
    if motion_groups is not None and hasattr(motion_cfg, "motion_dataset_load_cap"):
        min_load_cap = len(motion_groups)
        load_cap = motion_cfg.motion_dataset_load_cap
        if load_cap is None or int(load_cap) < min_load_cap:
            motion_cfg.motion_dataset_load_cap = min_load_cap
    if args_cli.disable_motion_randomization:
        _zero_motion_randomization(motion_cfg)

    if hasattr(env_cfg, "terminations"):
        for term_name in vars(env_cfg.terminations):
            if not term_name.startswith("_"):
                setattr(env_cfg.terminations, term_name, None)
    if args_cli.disable_events and hasattr(env_cfg, "events"):
        for term_name in vars(env_cfg.events):
            if not term_name.startswith("_"):
                setattr(env_cfg.events, term_name, None)
    if hasattr(env_cfg, "observations"):
        for group_name in ("policy", "teacher", "critic", "ref_vel_estimator"):
            group_cfg = getattr(env_cfg.observations, group_name, None)
            if group_cfg is not None and hasattr(group_cfg, "enable_corruption"):
                group_cfg.enable_corruption = False


def _set_motion_selection(command_term, env_ids: torch.Tensor):
    motion_count = int(getattr(command_term, "num_motions_total", 1))
    if motion_count <= 0:
        raise RuntimeError("No reference motions were loaded.")
    motion_index = max(0, min(int(args_cli.motion_index), motion_count - 1))
    if args_cli.show_all:
        motion_indices = torch.remainder(env_ids + motion_index, motion_count)
    else:
        motion_indices = torch.full_like(env_ids, motion_index)
    command_term.env_motion_indices[env_ids] = motion_indices
    max_frames = command_term.motion_lengths_minus_one[motion_indices]
    start_frame = torch.full_like(env_ids, max(int(args_cli.start_frame), 0))
    command_term.time_steps[env_ids] = torch.minimum(start_frame, max_frames)
    if hasattr(command_term, "motion_end_buf"):
        command_term.motion_end_buf[env_ids] = False
    return motion_indices


def _write_reference_state(command_term, robot, env_ids: torch.Tensor):
    root_pos = command_term.body_pos_w[:, 0].clone()
    root_ori = command_term.body_quat_w[:, 0].clone()
    root_lin_vel = command_term.body_lin_vel_w[:, 0].clone()
    root_ang_vel = command_term.body_ang_vel_w[:, 0].clone()
    joint_pos = command_term.joint_pos.clone()
    joint_vel = command_term.joint_vel.clone()

    root_state = torch.cat([root_pos, root_ori, root_lin_vel, root_ang_vel], dim=-1)
    robot.write_root_state_to_sim(root_state[env_ids], env_ids=env_ids)
    robot.write_joint_state_to_sim(joint_pos[env_ids], joint_vel[env_ids], env_ids=env_ids)
    if hasattr(robot, "set_joint_position_target"):
        robot.set_joint_position_target(joint_pos[env_ids], env_ids=env_ids)
    if hasattr(robot, "set_joint_velocity_target"):
        robot.set_joint_velocity_target(joint_vel[env_ids], env_ids=env_ids)


@hydra_task_config(args_cli.task, "rsl_rl_cfg_entry_point")
def main(
    env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg,
    agent_cfg: RslRlOnPolicyRunnerCfg,
):
    del agent_cfg
    _configure_env(env_cfg)

    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)
    if args_cli.video:
        video_folder = os.path.abspath(args_cli.video_folder)
        video_kwargs = {
            "video_folder": video_folder,
            "step_trigger": lambda step: step == 0,
            "video_length": int(args_cli.video_length),
            "disable_logger": True,
        }
        print("[ReferenceReplay] recording video")
        print(f"[ReferenceReplay] video_folder={video_folder}")
        env = gym.wrappers.RecordVideo(env, **video_kwargs)
    if isinstance(env.unwrapped, DirectMARLEnv):
        raise RuntimeError("Reference replay expects a single-agent ManagerBased env.")

    base_env = env.unwrapped
    command_term = base_env.command_manager.get_term("motion")
    robot = base_env.scene["robot"]
    env_ids = torch.arange(base_env.num_envs, device=base_env.device, dtype=torch.long)
    motion_indices = _set_motion_selection(command_term, env_ids)
    env.reset()
    motion_indices = _set_motion_selection(command_term, env_ids)

    action_shape = base_env.action_manager.action.shape
    zero_actions = torch.zeros(action_shape, device=base_env.device)
    step_dt = float(base_env.step_dt) / max(float(args_cli.speed), 1.0e-6)
    frame_counter = 0
    motion_names = []
    motion_paths = getattr(command_term.motion_dir_loader, "motion_paths", None)
    if motion_paths is not None:
        for idx in motion_indices.detach().cpu().tolist():
            if idx < len(motion_paths):
                motion_names.append(str(motion_paths[idx]))

    print("[ReferenceReplay] started")
    print(f"[ReferenceReplay] task={args_cli.task}")
    print(f"[ReferenceReplay] motion={args_cli.motion}")
    print(f"[ReferenceReplay] motion_indices={motion_indices.detach().cpu().tolist()}")
    if motion_names:
        for env_idx, motion_name in enumerate(motion_names):
            print(f"[ReferenceReplay] env{env_idx}_motion_file={motion_name}")
    print(f"[ReferenceReplay] start_frame={args_cli.start_frame}, speed={args_cli.speed}")

    with torch.inference_mode():
        while simulation_app.is_running():
            loop_start = time.perf_counter()
            _write_reference_state(command_term, robot, env_ids)
            env.step(zero_actions)

            ended = getattr(command_term, "motion_end_buf", None)
            if ended is not None and torch.any(ended[env_ids]):
                if args_cli.no_loop:
                    break
                _set_motion_selection(command_term, env_ids)

            frame_counter += 1
            if args_cli.video and frame_counter >= int(args_cli.video_length):
                break
            if args_cli.print_every > 0 and frame_counter % args_cli.print_every == 0:
                current_frame = int(command_term.time_steps[0].item())
                print(f"[ReferenceReplay] step={frame_counter} frame={current_frame}", flush=True)

            elapsed = time.perf_counter() - loop_start
            sleep_s = step_dt - elapsed
            if sleep_s > 0.0:
                time.sleep(sleep_s)

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
