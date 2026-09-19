#!/usr/bin/env bash
set -u
LOG="/root/Hitter/MOSAIC-main/logs/tmux/hitter_swanlab_resync_20260609_2025.log"
RUN_SWAN="/root/Hitter/MOSAIC-main/logs/rsl_rl/hitter_striking_g1/2026-06-09_14-50-25_hitter_mix_aligned_backhand_strike43_noshard_swanlab_2xa100_env50000_20260609_145010/swanlog/run-20260609_145736-ygb7q4qqx1klvtsh1ss1j"
exec >> "$LOG" 2>&1
cd /root/Hitter/MOSAIC-main
echo "===== $(date -Is) swanlab resync loop started ====="
while true; do
  echo "===== $(date -Is) swanlab sync start ====="
  /root/.venvs/isaaclab-mosaic/bin/swanlab sync "$RUN_SWAN" --project hitter_striking_g1 --id ygb7q4qqx1klvtsh1ss1j
  rc=$?
  echo "===== $(date -Is) swanlab sync exit rc=$rc ====="
  sleep 300
done
