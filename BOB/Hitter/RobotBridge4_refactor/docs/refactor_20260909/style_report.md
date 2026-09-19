# RobotBridge4 style-only refactor report

Date: 2026-09-09

Pre-style snapshot: `e85fcae`

Workspace: `/home/yhl/Desktop/RobotBridge4_refactor`

## Result

Reviewed all 71 active newly added first-party program files and the two modified original simulator files required by Task 2. The resulting program diff contains 62 formatted files: 54 newly added Python files, six newly added C++ files, and the protected changed regions of two simulator files. Eleven reviewed files required no edit: six Python files already matched the selected profile, four Shell files already had consistent layout, and the HTML monitor already had consistent two-space HTML/CSS/JavaScript layout.

This phase changes layout only. It does not extract definitions, move files, archive backups, update README content, reorder imports, alter strings, or fix existing behavior.

## Style decisions and tools

Representative neighboring files reviewed before formatting were `deploy/agents/mosaic_agent.py`, `deploy/utils/teleop.py`, `deploy/utils/kinematics.py`, `deploy/save_video.sh`, and `unitree_sdk2/trans.cpp`. The result follows the approved local conventions: four-space Python/C++ indentation, grouped imports in their existing execution order, attached C++ braces, readable wrapping, and existing string spelling.

Python formatting used Black 26.5.1 from the temporary tool overlay under Python 3.13, targeting Python 3.10:

```bash
PYTHONPATH=/tmp/rb4-refactor-tools \
  /home/yhl/miniforge3/bin/python -m black \
  --line-length 120 \
  --skip-string-normalization \
  --target-version py310 \
  <the 60 active-added Python paths recorded in style_audit.json>
```

Outcome: 54 files reformatted and six left unchanged. A final `--check` over the same 60 paths reported `60 files would be left unchanged`.

C++ formatting used clang-format 23.1.0:

```bash
/tmp/rb4-refactor-tools/clang_format/data/bin/clang-format -i \
  --style='{BasedOnStyle: LLVM, IndentWidth: 4, TabWidth: 4, UseTab: Never, ContinuationIndentWidth: 4, BreakBeforeBraces: Attach, SortIncludes: Never, SortUsingDeclarations: Never, ReflowComments: false, ColumnLimit: 120, FixNamespaceComments: false, InsertBraces: false, RemoveBracesLLVM: false, RemoveSemicolon: false, QualifierAlignment: Leave}' \
  <the six active-added C++ paths recorded in style_audit.json>
```

clang-format split one long string in `deploy/mocap_bridge/nexus_probe_cpp.cpp` into two adjacent literal tokens. That one change was manually rejected to satisfy exact C++ token preservation, leaving a single reviewed line over 120 columns. The other five C++ files pass clang-format `--dry-run --Werror` with the same explicit style.

The four Shell scripts and HTML file were reviewed manually. Their layout was already consistent, and changing embedded commands or JavaScript carried no style benefit. They remain byte-identical to the pre-style snapshot.

## Modified-original protection

The baseline reference was diffed against pre-style `HEAD` using `difflib.SequenceMatcher(..., autojunk=False)`. Black ran on the resulting changed-current line ranges in temporary copies. Each formatter edit was then admitted only when every consumed pre-style line belonged to a non-equal baseline opcode; insertions were admitted only at a boundary adjacent to a mutable line. Expanded edits were rejected. Finally, every original baseline-equal block had to remain an exact, ordered byte sequence.

| File | Equal blocks protected | Changed pre-style lines | Formatter edits accepted | Expanded edits rejected | Out-of-scope edits | Missing exact blocks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `deploy/simulator/mujoco.py` | 36 | 311 | 20 | 15 | 0 | 0 |
| `deploy/simulator/real_world.py` | 27 | 1,358 | 59 | 16 | 0 | 0 |

The deliberately unchanged original differences in `deploy/agents/base_agent.py` and `deploy/envs/mosaic.py` remain exactly as captured in `e85fcae`. No baseline-identical file changed.

## Verification

```bash
/home/yhl/miniforge3/envs/isaaclab/bin/python \\
  docs/refactor_20260909/style_check.py
```

Outcome: Python 3.10.20 compiled all 62 Python files and reported zero syntax or AST differences. The audit parses type comments and uses `feature_version=(3, 10)`. Per-file pre-style and post-style AST hashes are preserved in `style_audit.json`. The command compares immutable snapshot `e85fcae` with the working tree during this style stage; after later structural work, reproduce the same evidence with `--revision <style-commit>` using the style commit reported at handoff.

```bash
for script in \
  deploy/mocap_bridge/build_cpp_probe.sh \
  deploy/mocap_bridge/build_v2_mocap.sh \
  deploy/mocap_bridge/build_vicon_frame_stream.sh \
  deploy/mocap_bridge/run_robotbridge4_vicon_real.sh; do
  bash -n "$script"
done
```

Outcome: four files checked, zero syntax failures.

```bash
/home/yhl/miniforge3/envs/isaaclab/bin/python \
  /tmp/rb4-refactor-work/check_cpp.py \
  /home/yhl/Desktop/RobotBridge4_refactor \
  /tmp/rb4-style-cpp
```

Outcome: both `test_transformation_t_v2.cpp` and `test_vicon_ball_track_v2.cpp` compiled and ran successfully. A lexical audit over all six formatted C++ files reported exact equality for identifiers, preprocessor numbers, raw/ordinary literals, macros, and maximal-munch punctuators after ignoring whitespace and comments. Counts and token-sequence hashes are in `style_audit.json`.

```bash
git diff --check
```

Outcome: no whitespace errors.

The unchanged HTML file parsed successfully with Python's `html.parser`. Node and other JavaScript parsers were unavailable, so no separate JavaScript syntax command was run. The file's byte identity and the decision to leave embedded code untouched reduce this phase's risk.

The controller-owned pre-style suite reported 111 failed, 674 passed, three skipped, and 254 subtests passed, primarily from existing schema and stale fixture mismatches. The completed post-style suite produced exactly the same counts and identical JUnit results: zero new failures and zero missing results. This style task did not alter or weaken those tests and did not broaden into behavioral fixes. The controller recorded the full comparison in its separately owned `style_test_comparison.json`.

## Formatted files

- `deploy/agents/hitter_agent.py`
- `deploy/diagnostics/hitter_task_attempts.py`
- `deploy/diagnostics/hitter_task_events.py`
- `deploy/diagnostics/hitter_task_models.py`
- `deploy/diagnostics/hitter_task_monitor.py`
- `deploy/diagnostics/hitter_task_pipeline.py`
- `deploy/diagnostics/hitter_task_recording.py`
- `deploy/diagnostics/hitter_task_replay.py`
- `deploy/diagnostics/hitter_task_web.py`
- `deploy/envs/hitter.py`
- `deploy/mocap_bridge/chingmu_sdk_client.py`
- `deploy/mocap_bridge/chingmu_table_lcm_bridge.py`
- `deploy/mocap_bridge/monitor_vicon_lcm.py`
- `deploy/mocap_bridge/nexus_probe_cpp.cpp`
- `deploy/mocap_bridge/tests/test_chingmu_ball_track_v2.py`
- `deploy/mocap_bridge/tests/test_chingmu_ball_tracker_roi.py`
- `deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py`
- `deploy/mocap_bridge/tests/test_mocap_v2_monitor.py`
- `deploy/mocap_bridge/tests/test_transformation_t_v2.cpp`
- `deploy/mocap_bridge/tests/test_transformation_t_v2.py`
- `deploy/mocap_bridge/tests/test_vicon_ball_track_v2.cpp`
- `deploy/mocap_bridge/tests/test_vicon_sdk_client.py`
- `deploy/mocap_bridge/vicon_datastream_dump.cpp`
- `deploy/mocap_bridge/vicon_frame_stream.cpp`
- `deploy/mocap_bridge/vicon_sdk_client.py`
- `deploy/mocap_bridge/vicon_table_lcm_bridge.cpp`
- `deploy/mocap_bridge/vicon_table_lcm_bridge.py`
- `deploy/tests/hitter_runtime_test_harness.py`
- `deploy/tests/hitter_test_factories.py`
- `deploy/tests/test_hitter_backhand_edge_landing_bias.py`
- `deploy/tests/test_hitter_completed_result_queue.py`
- `deploy/tests/test_hitter_forehand_policy_vx_offset.py`
- `deploy/tests/test_hitter_planner_failure_reasons.py`
- `deploy/tests/test_hitter_policy_first_frame_transition.py`
- `deploy/tests/test_hitter_runtime_factory.py`
- `deploy/tests/test_hitter_runtime_single_shot_integration.py`
- `deploy/tests/test_hitter_single_shot_lifecycle.py`
- `deploy/tests/test_hitter_strike_target_logging.py`
- `deploy/tests/test_hitter_task_attempts.py`
- `deploy/tests/test_hitter_task_diagnostics_integration.py`
- `deploy/tests/test_hitter_task_diagnostics_performance.py`
- `deploy/tests/test_hitter_task_diagnostics_safety.py`
- `deploy/tests/test_hitter_task_events.py`
- `deploy/tests/test_hitter_task_frontend.py`
- `deploy/tests/test_hitter_task_input_adapter.py`
- `deploy/tests/test_hitter_task_monitor.py`
- `deploy/tests/test_hitter_task_observation.py`
- `deploy/tests/test_hitter_task_pipeline.py`
- `deploy/tests/test_hitter_task_recording.py`
- `deploy/tests/test_hitter_task_replay.py`
- `deploy/tests/test_hitter_task_replay_process.py`
- `deploy/tests/test_hitter_task_web.py`
- `deploy/tests/test_hitter_waiting_anchor.py`
- `deploy/tests/test_mujoco_hitter_track_id_v2.py`
- `deploy/tests/test_mujoco_physical_table_tennis.py`
- `deploy/tests/test_real_world_v2_consumer.py`
- `deploy/utils/hitter_planner.py`
- `deploy/utils/hitter_realtime.py`
- `deploy/utils/hitter_runtime_factory.py`
- `deploy/utils/hitter_task_observation.py`
- `deploy/simulator/mujoco.py`
- `deploy/simulator/real_world.py`

## Reviewed unchanged files

- `deploy/diagnostics/__init__.py` — already compliant with the selected Black profile.
- `deploy/diagnostics/static/hitter_task_monitor.html` — consistent two-space HTML/CSS/JavaScript layout; embedded code left untouched.
- `deploy/mocap_bridge/build_cpp_probe.sh` — consistent two-space shell layout; whitespace changes were not justified.
- `deploy/mocap_bridge/build_v2_mocap.sh` — consistent two-space shell layout; whitespace changes were not justified.
- `deploy/mocap_bridge/build_vicon_frame_stream.sh` — consistent two-space shell layout; whitespace changes were not justified.
- `deploy/mocap_bridge/run_robotbridge4_vicon_real.sh` — consistent two-space shell layout; whitespace changes were not justified.
- `deploy/mocap_bridge/tests/test_vicon_table_lcm_bridge.py` — already compliant with the selected Black profile.
- `deploy/tests/test_hitter_planner_velocity_alignment.py` — already compliant with the selected Black profile.
- `deploy/tests/test_hitter_runtime_identity_types.py` — already compliant with the selected Black profile.
- `deploy/tests/test_real_world_connection_wait.py` — already compliant with the selected Black profile.
- `deploy/utils/hitter_runtime_types.py` — already compliant with the selected Black profile.

## Machine-readable evidence

`docs/refactor_20260909/style_audit.json` records all 73 reviewed files, their origin and decision, before/after SHA256 values, per-file AST or C++ token-sequence hashes, protected-region counts, tool profiles, and verification outcomes. `docs/refactor_20260909/style_check.py` reproduces the AST, token, protected-region, hash, Shell, HTML, and out-of-scope checks against the working tree or an explicit style-stage revision. The pre-style source for every comparison is immutable commit `e85fcae`.
