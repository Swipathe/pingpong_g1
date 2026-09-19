# Source and validation notes

## Source inventory

- Server: `4090-0701`
- Run: `2026-08-14_16-42-38_hitter_mqy_rvx0to25_rvyneg05to05_rvz0to1_racketz0to06_tts030to092_posstd018_velstd18_velwin003_newracket_cad_v1_fromscratch_7x4090_gpu1to7_env28000_20260814_163909`
- TensorBoard event: `events.out.tfevents.1786697110.g0701.3028793.0`
- Event last modified: `2026-08-15 07:22:01 CST`
- Metric tag: `Metrics/motion/hitter_strike_racket_ori_error`
- Reward tag: `Episode_Reward/racket_ori`
- Metric implementation: `source/whole_body_tracking/whole_body_tracking/tasks/hitter/mdp/commands.py`
- Reward implementation: `source/whole_body_tracking/whole_body_tracking/tasks/hitter/mdp/rewards.py`
- Saved configuration: `params/env.yaml`

## Validation

- EventAccumulator loaded all scalar values with `size_guidance={"scalars": 0}`.
- Count: 8,624.
- Step range: 0–8623.
- Steps are strictly contiguous and unique.
- NaN count: 0.
- The metric formula and signed-normal setting were checked against both saved config and source code.
- Final-window sensitivity check: 100-step, 500-step and 1000-step windows all yield approximately 20°.

## Chart map

- Intended chart: single-series line chart of the 100-step rolling cosine-equivalent angle against training step.
- Analytical question: when did normal alignment improve, regress and stabilize?
- Takeaway: best at steps 759–858 (~14.38°), then regressed and ended near 20°.
- Palette: single blue root with neutral reference annotations.
- Supporting data: `trend.csv`.
- Chart rendering was omitted because the required portable-report Node runtime and a local static plotting renderer were unavailable. The reviewed chart-ready dataset is preserved.

## Report packaging blocker

The selected portable HTML workflow requires the packaged Data Analytics Node renderer. Neither `node`, `nodejs`, nor `npm` is installed in the local workspace or on `4090-0701`; `matplotlib` and `gnuplot` are also unavailable. No hand-authored replacement HTML/chart was created because that would bypass the canonical report contract.
