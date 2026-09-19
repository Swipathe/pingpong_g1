### Task 1: 真机正手 policy 速度适配与诊断

**Files:**
- Modify: `deploy/envs/hitter.py:50-80,1028-1100,1647-1672`
- Modify: `deploy/config/mimic/hitter.yaml:1-7`
- Modify: `deploy/tests/test_hitter_runtime_single_shot_integration.py:275-330`
- Modify: `deploy/tests/test_hitter_strike_target_logging.py:90-175`

**Interfaces:**
- Consumes: planner 原始 `HitterWbcCommand.v_racket_target_w: np.ndarray`、最终 `strike_type: str`、`simulator.is_real: bool`。
- Produces: `HitterEnv._policy_racket_target_velocity_w(...)->tuple[np.ndarray, float]`，返回独立 `float32 (3,)` policy 速度及实际 X 偏置。

- [ ] **Step 1: 写真实生命周期到 policy observation 的失败测试**

在 `test_hitter_runtime_single_shot_integration.py` 增加一个构造合法正/反手结果的 helper，并增加三组边界与不累计测试：

```python
def _result_for_strike_side(side: str):
    result = success_result(track_id=41, generation=1, marker=0.0)
    command = result.command
    y_w = -0.10 if side == "forehand" else 0.10
    position = np.asarray(
        command.strike_plan.p_racket_target,
        dtype=np.float64,
    ).copy()
    position[1] = y_w
    plan = replace(command.strike_plan, p_racket_target=position)
    return replace(
        result,
        command=replace(
            command,
            strike_type=side,
            strike_table_y_w=y_w,
            strike_plan=plan,
        ),
    )


@pytest.mark.parametrize(
    "is_real,side,expected",
    [
        (True, "forehand", [1.25, 0.1, 0.2]),
        (True, "backhand", [1.0, 0.1, 0.2]),
        (False, "forehand", [1.0, 0.1, 0.2]),
    ],
)
def test_policy_racket_velocity_x_offset_is_real_forehand_only(
    is_real,
    side,
    expected,
):
    env = make_hitter_env_for_test(is_real=is_real)
    env.forehand_policy_vx_offset_mps = 0.25
    result = _result_for_strike_side(side)
    decision = env.hitter_command_lifecycle.ingest(result, now=10.0)
    env._apply_hitter_lifecycle_decision(decision, now=10.0)
    env.compute_observation()
    np.testing.assert_allclose(env.obs_buf_dict["obs"][0, 13:16], expected)


def test_real_forehand_policy_racket_velocity_x_offset_does_not_accumulate():
    env = make_hitter_env_for_test(is_real=True)
    env.forehand_policy_vx_offset_mps = 0.25
    result = _result_for_strike_side("forehand")
    decision = env.hitter_command_lifecycle.ingest(result, now=10.0)
    env._apply_hitter_lifecycle_decision(decision, now=10.0)
    frozen = env.hitter_command_lifecycle.active_result.command

    env.compute_observation()
    first = env.obs_buf_dict["obs"][0, 13:16].copy()
    env.compute_observation()
    second = env.obs_buf_dict["obs"][0, 13:16].copy()

    np.testing.assert_allclose(first, [1.25, 0.1, 0.2])
    np.testing.assert_allclose(second, first)
    np.testing.assert_allclose(frozen.v_racket_target_w, [1.0, 0.1, 0.2])
    np.testing.assert_allclose(
        frozen.strike_plan.v_racket_target,
        [1.0, 0.1, 0.2],
    )
```

- [ ] **Step 2: 运行新测试并确认 RED 原因正确**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=deploy \
  /home/loco1/miniconda3/envs/rb/bin/python -m pytest \
  -p no:cacheprovider -q \
  deploy/tests/test_hitter_runtime_single_shot_integration.py::test_policy_racket_velocity_x_offset_is_real_forehand_only \
  deploy/tests/test_hitter_runtime_single_shot_integration.py::test_real_forehand_policy_racket_velocity_x_offset_does_not_accumulate
```

Expected: 真机正手用例失败，实际 X 为 `1.0`、期望为 `1.25`；反手和 MuJoCo 用例通过。

- [ ] **Step 3: 添加最小 policy 输入适配实现**

在 `HitterEnv.__init__` 读取并验证配置，增加纯副本 helper：

```python
self.forehand_policy_vx_offset_mps = (
    self._validated_forehand_policy_vx_offset(
        self.policy_cfg.get(
            "real_world_forehand_racket_velocity_x_offset_mps",
            0.0,
        )
    )
)

@staticmethod
def _validated_forehand_policy_vx_offset(value) -> float:
    if isinstance(value, (bool, np.bool_)) or not isinstance(
        value,
        (int, float, np.integer, np.floating),
    ):
        raise TypeError(
            "policy.real_world_forehand_racket_velocity_x_offset_mps "
            "must be a real number."
        )
    result = float(value)
    if not np.isfinite(result) or result < 0.0:
        raise ValueError(
            "policy.real_world_forehand_racket_velocity_x_offset_mps "
            "must be finite and non-negative."
        )
    return result

def _policy_racket_target_velocity_w(
    self,
    racket_velocity_w,
    *,
    strike_type: str,
) -> tuple[np.ndarray, float]:
    velocity = self._validated_vector(
        racket_velocity_w,
        name="v_racket_target_w",
        size=3,
    ).astype(np.float32)
    offset = 0.0
    if (
        bool(getattr(self.simulator, "is_real", False))
        and strike_type == "forehand"
    ):
        offset = float(getattr(self, "forehand_policy_vx_offset_mps", 0.0))
        velocity[0] += np.float32(offset)
    return velocity, offset
```

在 active observation 分支调用 helper，将返回速度传给
`assemble_active_hitter_task_observation()`；原始
`self.hitter_racket_target_vel_w` 不改写。

- [ ] **Step 4: 配置当前真机偏置并增强单次目标日志**

在 `deploy/config/mimic/hitter.yaml` 的 `policy` 下添加：

```yaml
real_world_forehand_racket_velocity_x_offset_mps: 0.25
```

在 `_log_hitter_strike_target()` 中用同一 helper 计算 policy 速度，在保留
`v_racket_target_w_mps` 和 `speed_racket_mps` 的同时追加：

```text
v_racket_policy_w_mps=[...]
speed_racket_policy_mps=...
policy_vx_offset_mps=0.2500
```

并在 `test_hitter_strike_target_logging.py` 增加真机正手断言：原始 X 仍为
`1.4000`，policy X 为 `1.6500`，offset 为 `0.2500`。

- [ ] **Step 5: 运行目标测试并确认 GREEN**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=deploy \
  /home/loco1/miniconda3/envs/rb/bin/python -m pytest \
  -p no:cacheprovider -q \
  deploy/tests/test_hitter_runtime_single_shot_integration.py \
  deploy/tests/test_hitter_strike_target_logging.py \
  deploy/tests/test_hitter_task_observation.py
```

Expected: 全部通过，无 warning/error traceback。

- [ ] **Step 6: 验证部署配置和精确差异**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
git diff --check -- \
  deploy/envs/hitter.py \
  deploy/config/mimic/hitter.yaml \
  deploy/tests/test_hitter_runtime_single_shot_integration.py \
  deploy/tests/test_hitter_strike_target_logging.py
rg -n \
  'real_world_forehand_racket_velocity_x_offset_mps|checkpoint:' \
  deploy/config/mimic/hitter.yaml
```

Expected: `diff --check` 为 0；配置同时保留 model7100 checkpoint 和
`0.25` 偏置。

- [ ] **Step 7: 保留实现为精确工作区改动**

`deploy/envs/hitter.py` 和 `deploy/config/mimic/hitter.yaml` 已含用户既有未提交
修改，因此不暂存或提交这些文件，避免把不属于本任务的变化混入提交。最终交付
精确文件/行、测试结果以及三终端真机部署命令。
