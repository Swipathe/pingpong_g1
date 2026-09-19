### Task 5: 引入 typed planner failure 与球桌绝对 y 手型规则

**Files:**
- Modify: `deploy/utils/hitter_runtime_types.py`
- Modify: `deploy/utils/hitter_planner.py: vector validation, StrikePlanner.hit_plane_intersection, BaseTargetPlanner.plan, HitterWbcCommand, HitterSystemPlanner.plan_command`
- Modify: `deploy/envs/hitter.py: _plan_hitter_snapshot, _mujoco_planner_result`
- Modify: `deploy/utils/hitter_runtime_factory.py: forced_strike_type`
- Create: `deploy/tests/test_hitter_planner_failure_reasons.py`
- Modify: `deploy/tests/test_hitter_runtime_factory.py`
- Modify: `deploy/tests/test_hitter_strike_target_logging.py`

**Interfaces:**
- Consumes: Task 4 的 canonical `track_id` types。
- Produces: `PlannerFailureReason`、`PlannerRejected`、`strike_type_from_table_y()`；`BaseTargetPlanner.plan(*, racket_target_w, current_base_xy_w, base_forward_xy_w, strike_type: str)` 不再自行判断手型；command 携带 `strike_table_y_w` 与 `strike_side_source="table_y"`。

测试文件内定义 `FixedStrikePlanner`，其 `plan()` 返回指定的有限 `StrikePlan`；`system_planner_with_fixed_strike_point(point)` 用该 stub 和真实 `BaseTargetPlanner` 构造 `HitterSystemPlanner`。`strike_planner` fixture 使用生产边界配置，不 monkeypatch `strike_type_from_table_y()`，因此三点边界测试会走真实自动手型路径。

- [ ] **Step 1: 写 typed reason 和 table-y 三点边界失败测试**

```python
@pytest.mark.parametrize(
    "table_y,expected",
    [(-1.0e-9, "forehand"), (0.0, "backhand"), (1.0e-9, "backhand")],
)
def test_strike_type_uses_absolute_table_y(table_y, expected):
    planner = system_planner_with_fixed_strike_point([0.0, table_y, 1.0])
    for base_y, yaw in [(-0.8, -1.2), (0.0, 0.0), (0.9, 2.1)]:
        forward = [np.cos(yaw), np.sin(yaw)]
        command = planner.plan_command(
            [0.8, 0.0, 1.0], [-2.0, 0.0, 0.0],
            current_base_xy_w=[-0.4, base_y],
            base_forward_xy_w=forward,
        )
        assert command.strike_type == expected
        assert command.strike_table_y_w == pytest.approx(table_y)
        assert command.strike_side_source == "table_y"


def test_no_crossing_has_typed_reason(strike_planner):
    with pytest.raises(PlannerRejected) as caught:
        strike_planner.hit_plane_intersection([0.8, 0.0, 1.0], [-0.01, 0.0, 4.0])
    assert caught.value.reason is PlannerFailureReason.NO_FUTURE_CROSSING
```

同文件以 `pytest.mark.parametrize("scenario,reason", [...])` 逐项覆盖八个 planner reason：ended、estimator-not-ready、base-invalid、not-incoming、no-crossing、height、nonfinite 和 monkeypatched unexpected exception；前七项断言具体 `PlannerRejected.reason`，unexpected exception 在 Task 6 worker 测试断言转 `INTERNAL_ERROR`。控制断言只比较 enum，不比较异常文本。

- [ ] **Step 2: 写真机 override 启动失败测试**

```python
def test_real_world_rejects_forced_strike_type():
    with pytest.raises(ValueError, match="force_strike_type is disabled"):
        forced_strike_type(
            {"force_strike_type": "forehand"}, {}, is_real_world=True
        )


def test_mujoco_may_force_strike_type():
    assert forced_strike_type(
        {"force_strike_type": "forehand"}, {}, is_real_world=False
    ) == "forehand"
```

- [ ] **Step 3: 运行测试，确认当前异常字符串和相对 base lateral y 规则失败**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  deploy/tests/test_hitter_planner_failure_reasons.py \
  deploy/tests/test_hitter_runtime_factory.py \
  deploy/tests/test_hitter_strike_target_logging.py
```

Expected: FAIL，包含缺少 `PlannerRejected` 或零点/不同 base pose 得到错误手型。

- [ ] **Step 4: 定义 planner failure enum 与带 detail 的异常**

```python
class PlannerFailureReason(str, Enum):
    TRACK_ENDED = "TRACK_ENDED"
    ESTIMATOR_NOT_READY = "ESTIMATOR_NOT_READY"
    BASE_POSE_INVALID = "BASE_POSE_INVALID"
    BALL_NOT_INCOMING = "BALL_NOT_INCOMING"
    NO_FUTURE_CROSSING = "NO_FUTURE_CROSSING"
    HIT_HEIGHT_OUT_OF_RANGE = "HIT_HEIGHT_OUT_OF_RANGE"
    NONFINITE_INPUT_OR_OUTPUT = "NONFINITE_INPUT_OR_OUTPUT"
    INTERNAL_ERROR = "INTERNAL_ERROR"


class PlannerRejected(RuntimeError):
    def __init__(self, reason: PlannerFailureReason, detail: str):
        self.reason = PlannerFailureReason(reason)
        self.detail = str(detail)
        super().__init__(f"{self.reason.value}: {self.detail}")
```

- [ ] **Step 5: 把 planner 边界映射为 typed rejection**

`StrikePlanner` 对 admission 失败发 `BALL_NOT_INCOMING`，没有 directed crossing 发 `NO_FUTURE_CROSSING`，高度区间失败发 `HIT_HEIGHT_OUT_OF_RANGE`，输入、trajectory 或输出非有限发 `NONFINITE_INPUT_OR_OUTPUT`。`HitterEnv._plan_hitter_snapshot()` 对 visible/ready/base/incoming confirmation 分别发 `TRACK_ENDED`、`ESTIMATOR_NOT_READY`、`BASE_POSE_INVALID`、`BALL_NOT_INCOMING`。

- [ ] **Step 6: 将手型判定从 BaseTargetPlanner 上移到 system planner**

```python
def strike_type_from_table_y(racket_target_w) -> str:
    target = _vec3(racket_target_w, "racket_target_w")
    return "forehand" if float(target[1]) < 0.0 else "backhand"


resolved_strike_type = (
    strike_type_from_table_y(strike_plan.p_racket_target)
    if strike_type is None
    else validate_explicit_strike_type(strike_type)
)
strike_side_source = "table_y" if strike_type is None else "forced"
resolved_strike_type, p_base_target_xy = self.base_planner.plan(
    racket_target_w=strike_plan.p_racket_target,
    current_base_xy_w=current_base_xy_w,
    base_forward_xy_w=base_forward_xy_w,
    strike_type=resolved_strike_type,
)
```

`BaseTargetPlanner.plan` 的 `strike_type` 改成必填并只校验/使用。`HitterWbcCommand` 增加 `strike_table_y_w` 和 `strike_side_source`；自动模式固定记录 `table_y`，仅 MuJoCo/离线显式 override 记录 `forced`。strike log 同时输出这两个字段。

```python
@dataclass(frozen=True)
class HitterWbcCommand:
    strike_type: str
    p_base_target_xy: np.ndarray
    v_racket_target_w: np.ndarray
    time_to_strike: float
    strike_plan: StrikePlan
    strike_table_y_w: float
    strike_side_source: str
```

- [ ] **Step 7: 在 HitterEnv 初始化时一次性校验真机 override**

缓存 `self.hitter_forced_strike_type`；构造时以 `is_real_world=bool(self.simulator.is_real)` 解析，真机非空在 worker/ONNX 启动前抛错，MuJoCo/离线继续允许。

- [ ] **Step 8: 运行 planner、factory 与日志测试**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  deploy/tests/test_hitter_planner_failure_reasons.py \
  deploy/tests/test_hitter_runtime_factory.py \
  deploy/tests/test_hitter_strike_target_logging.py
```

Expected: PASS；`y=-epsilon/0/+epsilon` 为 forehand/backhand/backhand，改变 base y/yaw 不改变结果。

- [ ] **Step 9: 提交 typed planner 单元**

```bash
git add deploy/utils/hitter_runtime_types.py \
  deploy/tests/test_hitter_planner_failure_reasons.py \
  deploy/tests/test_hitter_strike_target_logging.py
git add -p -- deploy/utils/hitter_planner.py deploy/envs/hitter.py
git add -p -- deploy/utils/hitter_runtime_factory.py
git add -p -- deploy/tests/test_hitter_runtime_factory.py
git diff --cached --check
git diff --cached --name-status
git commit -m "feat: type HITTER failures and select side from table y"
```

---

