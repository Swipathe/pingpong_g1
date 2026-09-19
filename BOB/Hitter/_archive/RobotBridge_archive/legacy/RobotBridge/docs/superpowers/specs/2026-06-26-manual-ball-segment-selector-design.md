# Manual Ball Segment Selector Design

## Goal

Build an offline tool that loads historical ball trajectory CSV files, visualizes candidate trajectories, lets the operator manually mark start/end points, and exports a clean CSV for fitting and planner evaluation.

## Scope

The first version is a local matplotlib tool. It accepts one or more CSV files written by `monitor_vicon_lcm.py` or `visualize_ball_trajectory.py`, filters valid ball rows, splits them into candidate tracks by time gaps or existing `clean_segment_id`, and displays one candidate at a time.

## Interaction

The plot shows one large 3D trajectory view. The operator clicks two points directly on the 3D trajectory to define the selected interval. Internally, the selector projects trajectory points into display coordinates and chooses the trajectory sample closest to the click, which is more reliable than trying to invert a 3D click. Keyboard controls are:

- `a`: accept the selected interval as the next clean segment
- `r`: reset the current interval selection
- `n`: skip to the next candidate
- `p`: go back to the previous candidate
- `s`: save accepted segments
- `q`: save accepted segments and quit

## Output

The output CSV contains selected ball rows only. It preserves all original columns and adds `clean_segment_id`, `source_file`, and `source_row_index` for provenance. Existing `clean_segment_id` values are overwritten in the export.

## Testing

Automated tests cover the non-interactive core: loading rows, selecting time ranges, preserving original columns, assigning `clean_segment_id`, and recording source provenance. The interactive matplotlib shell is kept thin and manually exercised.
