#!/usr/bin/env bash
set -euo pipefail

HITTER_TRAIN_ROOT=/home/yhl/Desktop/Hitter/MOSAIC-main-sijie-20260806
HITTER_TRAIN_PY=/home/yhl/miniforge3/envs/isaaclab/bin/python
HITTER_MOTION=data/hitter_motions/20260806_mqy_g1_npz_raw_peak43_aligned
HITTER_RESUME_CHECKPOINT="$HITTER_TRAIN_ROOT/logs/rsl_rl/hitter_striking_g1/2026-08-06_17-32-19_hitter_mqy_peak43_vx0to4_vyneg14to14_vz0to4_tts030to092_posstd018_velstd4_velwin008_fromscratch_2x5090_env32000_r1_20260806_173129/model_5400.pt"
HITTER_RUN_NAME=hitter_mqy_resume5400_vx0to4_vyneg14to14_vz0to4_tts030to092_posstd012_velstd2_velwin004_2x5090_env32000_20260807_122824
HITTER_LAUNCH_LOG="$HITTER_TRAIN_ROOT/launch_logs/${HITTER_RUN_NAME}.log"
HITTER_CONFIG="$HITTER_TRAIN_ROOT/source/whole_body_tracking/whole_body_tracking/tasks/hitter/config/g1/flat_env_cfg.py"
HITTER_CONFIG_SHA256=e183c8fad97614934b8ea7ccc98eb85a5e99bf6cff08c389363d3a5ce151f6be
HITTER_CHECKPOINT_SHA256=779548f9d87a5fd82e5ce76dd22c5ec3e8bc2736d66a7879b540209575fcf3de
HITTER_CACHE_BASE="$HITTER_TRAIN_ROOT/cache/g1_usd_hitter/configuration/main_base.usd"
HITTER_CACHE_BASE_SHA256=09a030de1eb3ad527f310a00eb520ad99f8064dbf856797c8877be44e10df402

cd "$HITTER_TRAIN_ROOT"
mkdir -p "$HITTER_TRAIN_ROOT/launch_logs"

test -f "$HITTER_CONFIG"
test -d "$HITTER_TRAIN_ROOT/$HITTER_MOTION"
test -f "$HITTER_RESUME_CHECKPOINT"
test -f "$HITTER_CACHE_BASE"
test "$(sha256sum "$HITTER_CONFIG" | cut -d ' ' -f 1)" = "$HITTER_CONFIG_SHA256"
test "$(sha256sum "$HITTER_RESUME_CHECKPOINT" | cut -d ' ' -f 1)" = "$HITTER_CHECKPOINT_SHA256"
test "$(sha256sum "$HITTER_CACHE_BASE" | cut -d ' ' -f 1)" = "$HITTER_CACHE_BASE_SHA256"

export CUDA_VISIBLE_DEVICES=0,1
export PYTHONPATH="$HITTER_TRAIN_ROOT/source/whole_body_tracking:$HITTER_TRAIN_ROOT/source/rsl_rl${PYTHONPATH:+:$PYTHONPATH}"

"$HITTER_TRAIN_PY" -m torch.distributed.run \
  --standalone \
  --nnodes=1 \
  --nproc_per_node=2 \
  scripts/rsl_rl/train.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion="$HITTER_MOTION" \
  --num_envs=32000 \
  --max_iterations=194600 \
  --save_interval=100 \
  --resume_student_checkpoint="$HITTER_RESUME_CHECKPOINT" \
  --run_name="$HITTER_RUN_NAME" \
  --logger=tensorboard \
  --headless \
  2>&1 | tee -a "$HITTER_LAUNCH_LOG"

exit "${PIPESTATUS[0]}"
