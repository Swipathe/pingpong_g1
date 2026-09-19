# HITTER Ready Arm Pose Editor

This editor is a loopback-only, read-only viewer until an operator explicitly
confirms and saves a candidate. It uses the live display URDF/STL set and
validates poses against the current RobotBridge2 MuJoCo model.

## Authoritative assets and candidate boundary

The two authoritative live asset paths are:

- Display URDF/STL entrypoint:
  `/home/loco1/BOB/Hitter/MOSAIC-main/source/whole_body_tracking/whole_body_tracking/assets/unitree_description/urdf/g1_hitter_racket/main.urdf`
- Validation MJCF:
  `/home/loco1/BOB/Hitter/RobotBridge2/deploy/data/assets/g1/g1_29dof_hitter_racket_table_tennis.xml`

The authoritative asset configuration is:
`/home/loco1/BOB/Hitter/RobotBridge2/deploy/config/asset/g1_hitter_racket.yaml`.

The fixed candidate output directory is:
`/home/loco1/BOB/Hitter/RobotBridge2/deploy/data/hitter_ready_poses`.
Saved YAML/JSON files are candidates only. They do not change
`waiting_arm_return`, an active runtime configuration, or robot behavior.
Integration requires a separate, reviewed change after the user deliberately
selects a real posture. The first file in the fixed directory must come from
that user-selected posture, never from a smoke test.

## Start on `loco1`

Run from the post-integration RobotBridge2 checkout:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2
env -u DISPLAY \
  PYTHONDONTWRITEBYTECODE=1 \
  MUJOCO_GL=egl \
  PYTHONPATH=deploy:. \
  /home/loco1/miniconda3/envs/rb/bin/python \
  -m tools.hitter_ready_pose_editor.server \
  --host 127.0.0.1 \
  --port 8765
```

The server prints a session-specific URL and current asset hashes. Keep this
terminal open. Stop only this editor with `Ctrl-C`. Do not stop or restart
RobotBridge2, training, LCM/DDS/Vicon, real-robot, or MuJoCo deployment
processes.

## Tunnel from the Mac

In a separate Mac terminal:

```bash
ssh -J baai-omega-sunyu1997 \
  -N \
  -L 8765:127.0.0.1:8765 \
  g1-hostB-loco1
```

Open the exact tokenized URL printed by the `loco1` server, for example:

```text
http://127.0.0.1:8765/?token=<server-printed-session-token>
```

The editor binds only to loopback. Its runtime has no LCM, DDS, Vicon, or
robot-control imports and cannot command the robot. Do not share the session
token.

## Pose and mapping conventions

- The UI source of truth is a name-keyed 14-joint arm pose in radians.
- UI degree fields are converted to radians before validation; persisted
  joint values remain radians.
- The 14 arm values map to RobotBridge2 29-DoF indices `15..28`, in this
  order: left shoulder pitch/roll/yaw, left elbow, left wrist
  roll/pitch/yaw, then right shoulder pitch/roll/yaw, right elbow, and right
  wrist roll/pitch/yaw. Indices `0..14` retain the live asset defaults.
- The same 14 values map to motion NPZ indices
  `[11, 15, 19, 21, 23, 25, 27, 12, 16, 20, 22, 24, 26, 28]`.
- FK uses right-handed `robot_base_default`, radians, `xyzw` quaternions, and
  column-major browser matrices.

Hard joint limits are rejection boundaries: an out-of-hard-limit edit is
rejected without changing the accepted pose. Soft limits are the centered
90% range of each hard limit. A pose outside only a soft limit can proceed
only after the extra soft-limit confirmation. Browser/MuJoCo save parity must
remain within `0.0005 m` and `0.1 degrees` (`pi/1800 rad`).

Version 1 does not prove self-collision, table collision, torque feasibility,
transition smoothness, or true-robot safety. A candidate cannot be deployed
without later MuJoCo dynamics validation and a controlled real-robot review.

## Tests

Run the focused Python suite:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2
env -u DISPLAY \
  PYTHONDONTWRITEBYTECODE=1 \
  MUJOCO_GL=egl \
  PYTHONPATH=deploy:. \
  /home/loco1/miniconda3/envs/rb/bin/python \
  -m unittest discover \
  -s tools/hitter_ready_pose_editor/tests \
  -p 'test_*.py' \
  -v
```

Run the Node unit suite:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/tools/hitter_ready_pose_editor
npm run test:unit
```

Run the live 129-pose browser/MuJoCo parity suite:

```bash
bash -c '
set -euo pipefail
cd /home/loco1/BOB/Hitter/RobotBridge2
contract_file="$(mktemp /tmp/hitter-ready-fk.XXXXXX)"
trap "rm -f \"$contract_file\"" EXIT
env -u DISPLAY \
  PYTHONDONTWRITEBYTECODE=1 \
  MUJOCO_GL=egl \
  PYTHONPATH=deploy:. \
  /home/loco1/miniconda3/envs/rb/bin/python \
  tools/hitter_ready_pose_editor/scripts/generate_live_fk_contract.py \
  --output "$contract_file"
HITTER_LIVE_FK_CONTRACT="$contract_file" \
  npm --prefix tools/hitter_ready_pose_editor run test:live
'
```

Run the existing deploy baseline:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge2/deploy
env -u DISPLAY \
  PYTHONDONTWRITEBYTECODE=1 \
  MUJOCO_GL=egl \
  /home/loco1/miniconda3/envs/rb/bin/python \
  -m unittest discover \
  -s tests \
  -p 'test_*.py' \
  -v
```

The Task 10 brief recorded an older 31-test baseline with one unrelated
failure, `test_xml_defines_only_intended_ball_contact_pairs`: that test
expected `0.20 0.005 0.0001`, while the user-owned MJCF contains
`0.20 0.20 0.005 0.0001 0.0001` at line 319. The later branch may collect a
larger deploy suite and may no longer contain that test. Classify the actual
fresh run; do not alter the user-owned MJCF or deploy tests to recreate the
historical result.
