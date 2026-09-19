# Task 1 report: `transformation_t` v2 track-id wire contract

## Result

Added `int64_t track_id` immediately after `publish_time_us` in the sole LCM
schema source, then regenerated the C++ and Python bindings with `lcm-gen`.
The generated v2 fingerprint is `7bae371879d9c1e6`; it differs from the
known v1 fingerprint `71f936e3b20f1df5`.

## Changed files

- `unitree_sdk2/lcm_types/transformation_t.lcm`
- `unitree_sdk2/lcm_types/transformation_t.hpp` (generated only)
- `unitree_sdk2/lcm_types/transformation_t.py` (generated only)
- `deploy/mocap_bridge/tests/test_transformation_t_v2.py`
- `deploy/mocap_bridge/tests/test_transformation_t_v2.cpp`
- `deploy/mocap_bridge/build_v2_mocap.sh`

Local-only: `.git/info/exclude` gained exactly
`deploy/mocap_bridge/.build-v2/`; `git check-ignore -v` identified that exact
line for the v2 test binary. No file under the pre-existing
`deploy/mocap_bridge/bin/` was written.

## TDD evidence

Before the schema change, the required Python command was run with the new
tests present:

```text
FFFs
3 failed, 1 skipped in 0.04s
AttributeError: 'transformation_t' object has no attribute 'track_id'
```

The v1 rejection test also failed at RED because the current binding's packed
fingerprint equalled its independently supplied v1 fixture fingerprint
`71f936e3b20f1df5`. Thus the later rejection is proved to come from the v2
fingerprint mismatch, rather than payload truncation or malformed bytes.

After updating only the `.lcm` source and regenerating bindings, the C++ v2
fixture and the Python consumer were run again:

```text
....
4 passed in 0.01s
CPP_HEX_CHARS=214
V2_FINGERPRINT=7bae371879d9c1e6
```

The C++ fixture value-initializes the message so its `--emit-hex` result is
deterministic; two consecutive emissions were identical. Python decodes the
actual emitted bytes and verifies the required cross-language contract
`track_id == 9001`.

## Commands run

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. \
  /home/loco1/miniconda3/envs/rb/bin/python -m pytest -p no:cacheprovider \
  deploy/mocap_bridge/tests/test_transformation_t_v2.py -q

lcm-gen -x --cpp-hpath unitree_sdk2 unitree_sdk2/lcm_types/transformation_t.lcm
lcm-gen -p --ppath unitree_sdk2 unitree_sdk2/lcm_types/transformation_t.lcm

bash deploy/mocap_bridge/build_v2_mocap.sh
deploy/mocap_bridge/.build-v2/test_transformation_t_v2
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. \
  /home/loco1/miniconda3/envs/rb/bin/python -m pytest -p no:cacheprovider \
  deploy/mocap_bridge/tests/test_transformation_t_v2.py -q
git check-ignore -v deploy/mocap_bridge/.build-v2/test_transformation_t_v2
```

## Self-review

- The schema is the only hand-edited production source; both bindings were
  generated exclusively by the specified `lcm-gen` commands.
- `track_id` appears at the identical wire position in generated C++ and
  Python encoders/decoders.
- The C++-encoded payload is decoded by the Python v2 binding, which also
  establishes a matching cross-language fingerprint.
- The rejection test uses the prescribed pre-v2 fingerprint literal and
  explicitly asserts it differs from v2 before decoding; it is not a
  truncation-only failure.
- Build output is limited to the newly excluded `.build-v2/` directory.
- Existing dirty, deleted, and untracked worktree content was neither restored
  nor staged.

## Concerns

None.
