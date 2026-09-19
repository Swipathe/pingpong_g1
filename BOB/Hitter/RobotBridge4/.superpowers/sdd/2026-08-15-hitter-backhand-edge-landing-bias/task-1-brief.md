### Task 1: 用测试定义 planner 的有效落点与完整重算

**Files:**
- Create: `deploy/tests/test_hitter_backhand_edge_landing_bias.py`
- Modify: `deploy/utils/hitter_planner.py:596-954`
- Modify: `deploy/utils/hitter_planner.py:1042-1086`

**Interfaces:**
- Produces: `StrikePlanner.desired_landing_point_for_strike(strike_position, *, strike_type=None) -> np.ndarray`
- Produces: `StrikePlanner.plan(ball_position, ball_velocity, *, strike_type=None) -> StrikePlan`
- Consumes: existing `strike_type_from_table_y()` and `validate_explicit_strike_type()` semantics.

- [ ] **Step 1: Write failing behavior tests**

Create a deterministic planner fixture whose `hit_plane_intersection()` returns a selected strike Y. Assert literal effective target Y values:

```python
@pytest.mark.parametrize(
    ("strike_y", "strike_type", "expected_y"),
    [
        (-0.50, "forehand", 0.0),
        (0.30, "backhand", 0.0),
        (0.40, "backhand", -0.05),
        (0.50, "backhand", -0.10),
        (0.70, "backhand", -0.10),
    ],
)
def test_effective_landing_y_is_smooth_and_bounded(...):
    ...
```

Add a full-plan test that integrates `plan.v_ball_out` for `0.48 s` with `_free_flight_endpoint()` and asserts the endpoint equals `[2.05, -0.10, 0.78]` within `2e-5 m`. Add a disabled/default test comparing output with an unmodified planner. Add validation tests for non-finite/negative values, `full <= start`, `full > table_width/2`, and an out-of-table final landing Y.

- [ ] **Step 2: Run tests and verify RED**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4/deploy
conda run --no-capture-output -n rb pytest -q tests/test_hitter_backhand_edge_landing_bias.py
```

Expected: collection or constructor failures because the three constructor parameters and `desired_landing_point_for_strike()` do not exist.

- [ ] **Step 3: Implement the minimum planner behavior**

Extend `StrikePlanner.__init__()` with:

```python
backhand_edge_landing_start_y_w_m: float = 0.30,
backhand_edge_landing_full_y_w_m: float = 0.50,
backhand_edge_landing_y_decrement_m: float = 0.0,
```

When enabled, validate the fields against `predictor.table_width` and the base landing point. Implement:

```python
u = np.clip((strike_y - start) / (full - start), 0.0, 1.0)
weight = u * u * (3.0 - 2.0 * u)
target[1] -= decrement * weight
```

Resolve the strike type before calculating `v_ball_out`; pass explicit forced type through `HitterSystemPlanner.plan_command()` when present. Use the effective target for both the shooting initial guess and every endpoint error correction.

- [ ] **Step 4: Run the new tests and verify GREEN**

Run the Step 2 command. Expected: all tests pass.

---

