#!/usr/bin/env bash
set -o pipefail
cd /root/Hitter/MOSAIC-main
export PYTHONPATH=/root/Hitter/MOSAIC-main/source/whole_body_tracking:/root/Hitter/MOSAIC-main/source/rsl_rl:
export OMNI_KIT_ACCEPT_EULA=YES
export PYTHONUNBUFFERED=1
export SWANLAB_MODE=cloud
export WANDB_MODE=disabled
export WANDB_DISABLED=true
export SWANLAB_TAGS=hitter,aligned_backhand,strike43,noshard,2xa100
unset HITTER_MOTION_DATASET_LOAD_CAP
source /root/.venvs/isaaclab-mosaic/bin/activate
/root/.venvs/isaaclab-mosaic/bin/torchrun --standalone --nnodes=1 --nproc_per_node=2 scripts/rsl_rl/train.py   --task=Hitter-Striking-PlannerDomain-Flat-G1-v0   --motion=data/hitter_motions/hitter_mix_oldiphone_newwechat_20260609_backhand_strike43_aligned   --num_envs=50000   --max_iterations=10000   --run_name=hitter_mix_aligned_backhand_strike43_noshard_swanlab_2xa100_env50000_20260609_145010   --logger=swanlab   --log_project_name=hitter_striking_g1   --headless   env.commands.motion.motion_dataset_shard_across_gpus=False   2>&1 | tee -a /root/Hitter/MOSAIC-main/logs/tmux/hitter_mix_aligned_backhand_strike43_noshard_swanlab_2xa100_env50000_20260609_145010.log
