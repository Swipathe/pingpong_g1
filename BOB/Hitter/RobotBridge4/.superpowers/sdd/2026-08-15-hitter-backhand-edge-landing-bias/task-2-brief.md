### Task 2: Wire production configuration and disable the old policy offset

**Files:**
- Modify: `deploy/config/mimic/hitter.yaml:1-35`
- Modify: `deploy/utils/hitter_runtime_factory.py:118-155`
- Modify: `deploy/tests/test_hitter_runtime_factory.py:81-192,224-278`
- Modify: `deploy/tests/test_hitter_forehand_policy_vx_offset.py:80-106`

**Interfaces:**
- Consumes: the three new `StrikePlanner` constructor keyword arguments from Task 1.
- Produces: resolved production planner values `0.30`, `0.50`, `0.10`; production policy Y decrement `0.0`.

- [ ] **Step 1: Write failing factory/config tests**

Update the deployment-config expectation from `0.3` to `0.0`. Extend the runtime factory reference builder and equality assertion with the three new fields, then assert the production YAML resolves to the literal tuple `(0.30, 0.50, 0.10)`. Add a real-world policy adapter assertion proving a backhand input `[1.0, -0.2, 0.3]` remains unchanged when the production decrement is zero.

- [ ] **Step 2: Run tests and verify RED**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4/deploy
conda run --no-capture-output -n rb pytest -q \
  tests/test_hitter_runtime_factory.py \
  tests/test_hitter_forehand_policy_vx_offset.py
```

Expected: failures because YAML still resolves the old `0.3` decrement and the factory does not pass the new fields.

- [ ] **Step 3: Apply production wiring**

Set the policy decrement to `0.0`, add the three ball-planner values directly after `desired_landing_point_w`, and pass them from `planner_config` to `StrikePlanner`. Preserve constructor defaults when fields are absent.

- [ ] **Step 4: Run focused regression tests**

Run Task 1 and Task 2 test commands together. Expected: all pass with zero failures.

- [ ] **Step 5: Verify the resolved real-world configuration**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4/deploy
conda run --no-capture-output -n rb python - <<'PY'
from omegaconf import OmegaConf
cfg = OmegaConf.load('config/mimic/hitter.yaml')
print(cfg.policy.real_world_backhand_racket_velocity_y_decrement_mps)
print(cfg.motion.ball_planner.backhand_edge_landing_start_y_w_m)
print(cfg.motion.ball_planner.backhand_edge_landing_full_y_w_m)
print(cfg.motion.ball_planner.backhand_edge_landing_y_decrement_m)
PY
```

Expected output is four lines: `0.0`, `0.3`, `0.5`, `0.1`.

- [ ] **Step 6: Review only scoped diffs**

Run:

```bash
git diff --check -- \
  deploy/config/mimic/hitter.yaml \
  deploy/utils/hitter_planner.py \
  deploy/utils/hitter_runtime_factory.py \
  deploy/tests/test_hitter_backhand_edge_landing_bias.py \
  deploy/tests/test_hitter_runtime_factory.py \
  deploy/tests/test_hitter_forehand_policy_vx_offset.py
git diff --stat -- <same explicit paths>
```

Expected: no whitespace errors; only the named behavior/config/test files are part of this feature review. Do not stage unrelated dirty files.
