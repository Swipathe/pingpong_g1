# HITTER Isaac Training

This directory contains the IsaacLab/RSL-RL training side of HITTER.

The packaged task is:

```bash
Hitter-Striking-PlannerDomain-Flat-G1-v0
```

## Layout

- `source/whole_body_tracking/`: HITTER IsaacLab task, robot assets, motion MDP utilities, and training helpers.
- `source/rsl_rl/`: local RSL-RL source used by the training scripts.
- `scripts/rsl_rl/`: train, play, reference replay, and expert collection entry points.
- `motions/`: packaged forehand/backhand reference motions for the 20400 checkpoint.
- `checkpoints/hitter_m20400/`: packaged policy checkpoint, exported ONNX, and saved params.

The legacy generic motion task code has been folded into HITTER: generic motion loading, motion observations,
motion rewards, randomization events, and termination utilities now live under
`source/whole_body_tracking/whole_body_tracking/tasks/hitter/mdp/`.

## Setup

Install the local Python packages from this directory:

```bash
python -m pip install -e source/rsl_rl
python -m pip install -e source/whole_body_tracking
```

For direct script execution without editable installs, set:

```bash
export PYTHONPATH=$PWD/source/whole_body_tracking:$PWD/source/rsl_rl:$PYTHONPATH
```

## Reference Replay

Replay packaged HITTER motions:

```bash
python scripts/rsl_rl/play_reference_motion.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --motion motions \
  --num_envs=4
```

## Training

Run PPO training with the HITTER task:

```bash
python scripts/rsl_rl/train.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0
```

The task config resolves the packaged motion path as `motions` by default in the saved 20400 params.

## Playback

Play a saved checkpoint:

```bash
python scripts/rsl_rl/play.py \
  --task=Hitter-Striking-PlannerDomain-Flat-G1-v0 \
  --resume_student_checkpoint checkpoints/hitter_m20400/model_20400.pt
```

## Packaged Checkpoint

The included checkpoint bundle is:

- `checkpoints/hitter_m20400/model_20400.pt`
- `checkpoints/hitter_m20400/exported/policy.onnx`
- `checkpoints/hitter_m20400/params/agent.yaml`
- `checkpoints/hitter_m20400/params/env.yaml`

Large training logs, raw captures, intermediate retargeting outputs, and unrelated MOSAIC datasets are not included.
